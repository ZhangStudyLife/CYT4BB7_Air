#include "car_plan_2.h"
#include "car_plan.h"
#include "ProjectionCenter.h"
#include "car_lamp_fused.h"
#include "../Image/image_data.h"
#include <math.h>

#define CAR_PLAN_2_CAMERA_COUNT                  (3U)    /* 参与影子规划的摄像头数量。 */
#define CAR_PLAN_2_BEACON_COUNT_PER_CAMERA       (2U)    /* 每个摄像头参与规划的信标候选数量。 */
#define CAR_PLAN_2_MAX_CANDIDATE_COUNT           (6U)    /* 三摄信标候选总数上限。 */
#define CAR_PLAN_2_SAME_CAMERA_MERGE_DIST_PX     (8.0f)  /* 同摄像头重复候选的合并距离，单位px。 */
#define CAR_PLAN_2_CROSS_CAMERA_MERGE_DIST_PX    (15.0f) /* 跨摄像头同灯候选的合并距离，单位px。 */
#define CAR_PLAN_2_LOCK_MATCH_DIST_PX            (25.0f) /* 锁定目标预测位置的匹配距离，单位px。 */
#define CAR_PLAN_2_PREDICT_STEP_LIMIT_PX         (8.0f)  /* 单次位置预测增量上限，单位px。 */
#define CAR_PLAN_2_LOST_HOLD_TICKS               (10U)   /* 100Hz下目标丢失保持约100ms。 */
#define CAR_PLAN_2_VELOCITY_CONFLICT_TICKS       (10U)   /* 100Hz下车速冲突持续约100ms后切换。 */
#define CAR_PLAN_2_VELOCITY_MIN_MPS              (0.3f)  /* 启用车速方向判断的最低合速度，单位m/s。 */
#define CAR_PLAN_2_LOCKED_VELOCITY_COS_MAX       (0.2f)  /* 锁定目标与车速明显冲突的余弦上限。 */
#define CAR_PLAN_2_CHALLENGER_VELOCITY_COS_MIN   (0.85f) /* 替代目标与车速高度一致的余弦下限。 */
#define CAR_PLAN_2_CONFLICT_TARGET_DIST_PX       (20.0f) /* 车速纠错时两个物理目标的最小距离，单位px。 */
#define CAR_PLAN_2_CAR_CENTER_Y_OFFSET_PX        (10.0f) /* 车体中心相对车灯中心的Y轴偏移，单位px。 */
#define CAR_PLAN_2_MIN_TARGET_DIST_PX            (2.0f)  /* 车体中心与信标的最小有效距离，单位px。 */
#define CAR_PLAN_2_FAST_CENTER_DIST_PX           (65.0f) /* 快速速度判定的投影中心距离，单位px。 */
#define CAR_PLAN_2_ANGLE_TO_RAD                  (0.017453292519943295f) /* 角度转弧度系数。 */

typedef struct
{
    uint8 camera;
    uint8 group;
    float center_x;
    float center_y;
    float area;
} car_plan_2_candidate_t;

typedef struct
{
    uint8 group;
    float center_x;
    float center_y;
    float max_area;
    float area_sum;
} car_plan_2_cluster_t;

typedef struct
{
    uint8 valid;
    uint8 previous_valid;
    uint8 lost_ticks;
    uint8 velocity_conflict_ticks;
    float center_x;
    float center_y;
    float previous_x;
    float previous_y;
} car_plan_2_lock_t;

extern float g_car_vel_x; /* 车体右向实际速度，单位m/s。 */
extern float g_car_vel_y; /* 车体前向实际速度，单位m/s。 */

static car_plan_2_result_t s_car_plan_2_result; /* 最近一次影子规划输出。 */
static car_plan_2_lock_t s_car_plan_2_lock; /* 物理信标锁定位置、历史位置和计数状态。 */

/**
 * @brief 清零指定影子规划结果。
 * @param result 待清零的结果指针。
 * @return 无。
 */
static void CarPlan_2_ClearResult(car_plan_2_result_t *result)
{
    result->valid = 0U;
    result->target_strafe_mps = 0.0f;
    result->target_forward_mps = 0.0f;
}

/**
 * @brief 将内部影子规划结果复制到调用方。
 * @param result 输出结果指针；允许为空。
 * @return 无。
 */
static void CarPlan_2_CopyResult(car_plan_2_result_t *result)
{
    if(result != 0)
    {
        *result = s_car_plan_2_result;
    }
}

/**
 * @brief 将指定摄像头的像素点映射到Center摄像头坐标系。
 * @param camera 输入摄像头编号。
 * @param x 输入像素X坐标，单位px。
 * @param y 输入像素Y坐标，单位px。
 * @param center_x 输出Center坐标X，单位px。
 * @param center_y 输出Center坐标Y，单位px。
 * @return 无。
 */
static void CarPlan_2_MapPointToCenter(uint8 camera,
                                       float x,
                                       float y,
                                       float *center_x,
                                       float *center_y)
{
    float x2;
    float xy;
    float y2;

    if(camera == (uint8)Center)
    {
        *center_x = x;
        *center_y = y;
        return;
    }

    x2 = x * x;
    xy = x * y;
    y2 = y * y;
    if(camera == (uint8)Front)
    {
        *center_x = -3.224193f + 1.123975f * x + 0.003353f * y +
                    0.000073f * x2 - 0.004078f * xy - 0.000302f * y2;
        *center_y = -60.512112f + 0.030475f * x + 0.772429f * y +
                    0.004336f * x2 - 0.000232f * xy + 0.004678f * y2;
        return;
    }

    *center_x = -10.828701f - 1.119896f * x + 0.059751f * y -
                0.000063f * x2 + 0.004186f * xy - 0.000850f * y2;
    *center_y = 58.428997f - 0.026951f * x - 0.718077f * y -
                0.004166f * x2 + 0.000106f * xy - 0.004593f * y2;
}

/**
 * @brief 收集六个信标候选并按同灯距离合并为物理信标簇。
 * @param clusters 输出物理信标簇数组。
 * @return 有效物理信标簇数量。
 */
static uint8 CarPlan_2_BuildClusters(car_plan_2_cluster_t clusters[CAR_PLAN_2_MAX_CANDIDATE_COUNT])
{
    car_plan_2_candidate_t candidates[CAR_PLAN_2_MAX_CANDIDATE_COUNT];
    uint8 candidate_count = 0U;
    uint8 cluster_count = 0U;
    uint8 camera;
    uint8 beacon_index;
    uint8 i;
    uint8 j;

    for(camera = 0U; camera < CAR_PLAN_2_CAMERA_COUNT; camera++)
    {
        for(beacon_index = 0U; beacon_index < CAR_PLAN_2_BEACON_COUNT_PER_CAMERA; beacon_index++)
        {
            const beacon_data *beacon = &image_data[camera].beacon_data[beacon_index];
            if(image_data_beacon_valid(beacon) == 0U)
            {
                continue;
            }

            candidates[candidate_count].camera = camera;
            candidates[candidate_count].group = candidate_count;
            candidates[candidate_count].area = beacon->area;
            CarPlan_2_MapPointToCenter(camera,
                                       beacon->x,
                                       beacon->y,
                                       &candidates[candidate_count].center_x,
                                       &candidates[candidate_count].center_y);
            candidate_count++;
        }
    }

    for(i = 0U; i < candidate_count; i++)
    {
        for(j = (uint8)(i + 1U); j < candidate_count; j++)
        {
            float dx = candidates[i].center_x - candidates[j].center_x;
            float dy = candidates[i].center_y - candidates[j].center_y;
            float merge_dist = (candidates[i].camera == candidates[j].camera)
                                   ? CAR_PLAN_2_SAME_CAMERA_MERGE_DIST_PX
                                   : CAR_PLAN_2_CROSS_CAMERA_MERGE_DIST_PX;
            if((dx * dx + dy * dy) < (merge_dist * merge_dist))
            {
                uint8 old_group = candidates[j].group;
                uint8 k;
                if(candidates[i].group == old_group)
                {
                    continue;
                }
                for(k = 0U; k < candidate_count; k++)
                {
                    if(candidates[k].group == old_group)
                    {
                        candidates[k].group = candidates[i].group;
                    }
                }
            }
        }
    }

    for(i = 0U; i < candidate_count; i++)
    {
        uint8 cluster_index = 0xFFU;
        for(j = 0U; j < cluster_count; j++)
        {
            if(clusters[j].group == candidates[i].group)
            {
                cluster_index = j;
                break;
            }
        }
        if(cluster_index == 0xFFU)
        {
            cluster_index = cluster_count;
            clusters[cluster_index].group = candidates[i].group;
            clusters[cluster_index].center_x = 0.0f;
            clusters[cluster_index].center_y = 0.0f;
            clusters[cluster_index].max_area = 0.0f;
            clusters[cluster_index].area_sum = 0.0f;
            cluster_count++;
        }

        clusters[cluster_index].center_x += candidates[i].center_x * candidates[i].area;
        clusters[cluster_index].center_y += candidates[i].center_y * candidates[i].area;
        clusters[cluster_index].area_sum += candidates[i].area;
        if(candidates[i].area > clusters[cluster_index].max_area)
        {
            clusters[cluster_index].max_area = candidates[i].area;
        }
    }

    for(i = 0U; i < cluster_count; i++)
    {
        clusters[i].center_x /= clusters[i].area_sum;
        clusters[i].center_y /= clusters[i].area_sum;
    }

    return cluster_count;
}

/**
 * @brief 根据统一Center坐标中的物理信标生成车体系速度目标。
 * @param cluster 输入物理信标簇。
 * @param result 输出速度规划结果。
 * @return 结果有效时返回1，否则返回0。
 */
static uint8 CarPlan_2_MakeResult(const car_plan_2_cluster_t *cluster,
                                  car_plan_2_result_t *result)
{
    float car_center_x;
    float car_center_y;
    float dx;
    float dy;
    float dist;
    float angle_rad;
    float line_x;
    float line_y;
    float normal_x;
    float normal_y;
    float strafe;
    float forward;
    float plan_speed = Car_Speed;
    float speed_scale;

    if(g_car_lamp_fused.valid == 0U)
    {
        return 0U;
    }

    car_center_x = g_car_lamp_fused.cx;
    car_center_y = g_car_lamp_fused.cy + CAR_PLAN_2_CAR_CENTER_Y_OFFSET_PX;
    dx = cluster->center_x - car_center_x;
    dy = cluster->center_y - car_center_y;
    dist = sqrtf(dx * dx + dy * dy);
    if(dist <= CAR_PLAN_2_MIN_TARGET_DIST_PX)
    {
        return 0U;
    }

    dx /= dist;
    dy /= dist;
    angle_rad = g_car_lamp_fused.angle * CAR_PLAN_2_ANGLE_TO_RAD;
    line_x = cosf(angle_rad);
    line_y = sinf(angle_rad);
    normal_x = -line_y;
    normal_y = line_x;
    strafe = dx * line_x + dy * line_y;
    forward = -(dx * normal_x + dy * normal_y);

    if(g_projection_center.valid != 0U)
    {
        float beacon_projection_x = cluster->center_x - g_projection_center.cx;
        float beacon_projection_y = cluster->center_y - g_projection_center.cy;
        float projection_car_x = g_projection_center.cx - car_center_x;
        float projection_car_y = g_projection_center.cy - car_center_y;
        float beacon_car_x = cluster->center_x - car_center_x;
        float beacon_car_y = cluster->center_y - car_center_y;
        float direction_dot = projection_car_x * beacon_car_x +
                              projection_car_y * beacon_car_y;
        if(((beacon_projection_x * beacon_projection_x +
             beacon_projection_y * beacon_projection_y) <
            (CAR_PLAN_2_FAST_CENTER_DIST_PX * CAR_PLAN_2_FAST_CENTER_DIST_PX)) &&
           (direction_dot > 0.0f) &&
           ((direction_dot * direction_dot) >
            (0.25f *
             (projection_car_x * projection_car_x + projection_car_y * projection_car_y) *
             (beacon_car_x * beacon_car_x + beacon_car_y * beacon_car_y))))
        {
            plan_speed = Car_Speed_Fast;
        }
    }

    speed_scale = plan_speed /
                  ((fabsf(strafe) > fabsf(forward)) ? fabsf(strafe) : fabsf(forward));
    result->valid = 1U;
    result->target_strafe_mps = strafe * speed_scale;
    result->target_forward_mps = forward * speed_scale;
    return 1U;
}

/**
 * @brief 计算规划速度目标与车模实际速度方向的余弦值。
 * @param result 输入速度规划结果。
 * @return 两个速度方向的余弦值；速度无效时返回-1。
 */
static float CarPlan_2_VelocityCos(const car_plan_2_result_t *result)
{
    float car_speed = sqrtf(g_car_vel_x * g_car_vel_x + g_car_vel_y * g_car_vel_y);
    float target_speed = sqrtf(result->target_strafe_mps * result->target_strafe_mps +
                               result->target_forward_mps * result->target_forward_mps);
    if((car_speed <= CAR_PLAN_2_VELOCITY_MIN_MPS) || (target_speed <= 0.0f))
    {
        return -1.0f;
    }
    return (g_car_vel_x * result->target_strafe_mps +
            g_car_vel_y * result->target_forward_mps) /
           (car_speed * target_speed);
}

/**
 * @brief 将指定物理信标设置为新的锁定目标。
 * @param cluster 输入待锁定的物理信标簇。
 * @param result 输入该物理信标对应的规划结果。
 * @return 无。
 */
static void CarPlan_2_LockCluster(const car_plan_2_cluster_t *cluster,
                                  const car_plan_2_result_t *result)
{
    s_car_plan_2_lock.valid = 1U;
    s_car_plan_2_lock.center_x = cluster->center_x;
    s_car_plan_2_lock.center_y = cluster->center_y;
    s_car_plan_2_lock.previous_valid = 0U;
    s_car_plan_2_lock.lost_ticks = 0U;
    s_car_plan_2_lock.velocity_conflict_ticks = 0U;
    s_car_plan_2_result = *result;
}

/**
 * @brief 按最大候选面积获取新的物理信标目标。
 * @param clusters 输入物理信标簇数组。
 * @param cluster_count 输入物理信标簇数量。
 * @return 获取并生成结果成功时返回1，否则返回0。
 */
static uint8 CarPlan_2_Acquire(const car_plan_2_cluster_t *clusters,
                               uint8 cluster_count)
{
    car_plan_2_result_t result;
    uint8 selected_index = 0xFFU;
    uint8 i;

    for(i = 0U; i < cluster_count; i++)
    {
        if((selected_index == 0xFFU) ||
           (clusters[i].max_area > clusters[selected_index].max_area))
        {
            selected_index = i;
        }
    }
    if((selected_index == 0xFFU) ||
       (CarPlan_2_MakeResult(&clusters[selected_index], &result) == 0U))
    {
        CarPlan_2_ClearResult(&s_car_plan_2_result);
        return 0U;
    }

    CarPlan_2_LockCluster(&clusters[selected_index], &result);
    return 1U;
}

/**
 * @brief 重置影子车模规划器的目标锁定状态和输出结果。
 * @param 无。
 * @return 无。
 */
void CarPlan_2_Reset(void)
{
    CarPlan_2_ClearResult(&s_car_plan_2_result);
    s_car_plan_2_lock.valid = 0U;
    s_car_plan_2_lock.previous_valid = 0U;
    s_car_plan_2_lock.lost_ticks = 0U;
    s_car_plan_2_lock.velocity_conflict_ticks = 0U;
    s_car_plan_2_lock.center_x = 0.0f;
    s_car_plan_2_lock.center_y = 0.0f;
    s_car_plan_2_lock.previous_x = 0.0f;
    s_car_plan_2_lock.previous_y = 0.0f;
}

/**
 * @brief 扫描三摄信标候选并更新影子车模速度规划结果。
 * @param result 输出规划结果；允许传入空指针。
 * @return 规划结果有效时返回1，否则返回0。
 */
uint8 CarPlan_2_Update(car_plan_2_result_t *result)
{
    car_plan_2_cluster_t clusters[CAR_PLAN_2_MAX_CANDIDATE_COUNT];
    car_plan_2_result_t candidate_result;
    uint8 cluster_count = CarPlan_2_BuildClusters(clusters);
    uint8 selected_index = 0xFFU;
    uint8 i;

    if(s_car_plan_2_lock.valid == 0U)
    {
        (void)CarPlan_2_Acquire(clusters, cluster_count);
        CarPlan_2_CopyResult(result);
        return s_car_plan_2_result.valid;
    }

    {
        float predict_x = s_car_plan_2_lock.center_x;
        float predict_y = s_car_plan_2_lock.center_y;
        float best_dist_sq = CAR_PLAN_2_LOCK_MATCH_DIST_PX * CAR_PLAN_2_LOCK_MATCH_DIST_PX;
        if(s_car_plan_2_lock.previous_valid != 0U)
        {
            float step_x = s_car_plan_2_lock.center_x - s_car_plan_2_lock.previous_x;
            float step_y = s_car_plan_2_lock.center_y - s_car_plan_2_lock.previous_y;
            step_x = fmaxf(-CAR_PLAN_2_PREDICT_STEP_LIMIT_PX,
                           fminf(CAR_PLAN_2_PREDICT_STEP_LIMIT_PX, step_x));
            step_y = fmaxf(-CAR_PLAN_2_PREDICT_STEP_LIMIT_PX,
                           fminf(CAR_PLAN_2_PREDICT_STEP_LIMIT_PX, step_y));
            predict_x += step_x;
            predict_y += step_y;
        }

        for(i = 0U; i < cluster_count; i++)
        {
            float dx = clusters[i].center_x - predict_x;
            float dy = clusters[i].center_y - predict_y;
            float dist_sq = dx * dx + dy * dy;
            if(dist_sq < best_dist_sq)
            {
                best_dist_sq = dist_sq;
                selected_index = i;
            }
        }
    }

    if((selected_index != 0xFFU) &&
       (CarPlan_2_MakeResult(&clusters[selected_index], &candidate_result) != 0U))
    {
        float car_speed = sqrtf(g_car_vel_x * g_car_vel_x + g_car_vel_y * g_car_vel_y);
        float locked_velocity_cos = CarPlan_2_VelocityCos(&candidate_result);
        uint8 challenger_index = 0xFFU;
        float challenger_velocity_cos = CAR_PLAN_2_CHALLENGER_VELOCITY_COS_MIN;

        if((car_speed > CAR_PLAN_2_VELOCITY_MIN_MPS) &&
           (locked_velocity_cos < CAR_PLAN_2_LOCKED_VELOCITY_COS_MAX))
        {
            for(i = 0U; i < cluster_count; i++)
            {
                car_plan_2_result_t challenger_result;
                float dx;
                float dy;
                float velocity_cos;
                if(i == selected_index)
                {
                    continue;
                }

                dx = clusters[i].center_x - clusters[selected_index].center_x;
                dy = clusters[i].center_y - clusters[selected_index].center_y;
                if((dx * dx + dy * dy) <
                   (CAR_PLAN_2_CONFLICT_TARGET_DIST_PX * CAR_PLAN_2_CONFLICT_TARGET_DIST_PX))
                {
                    continue;
                }
                if(CarPlan_2_MakeResult(&clusters[i], &challenger_result) == 0U)
                {
                    continue;
                }

                velocity_cos = CarPlan_2_VelocityCos(&challenger_result);
                if(velocity_cos > challenger_velocity_cos)
                {
                    challenger_velocity_cos = velocity_cos;
                    challenger_index = i;
                }
            }
        }

        if(challenger_index != 0xFFU)
        {
            if(s_car_plan_2_lock.velocity_conflict_ticks < CAR_PLAN_2_VELOCITY_CONFLICT_TICKS)
            {
                s_car_plan_2_lock.velocity_conflict_ticks++;
            }
            if(s_car_plan_2_lock.velocity_conflict_ticks >= CAR_PLAN_2_VELOCITY_CONFLICT_TICKS)
            {
                CarPlan_2_MakeResult(&clusters[challenger_index], &candidate_result);
                CarPlan_2_LockCluster(&clusters[challenger_index], &candidate_result);
                CarPlan_2_CopyResult(result);
                return s_car_plan_2_result.valid;
            }
        }
        else
        {
            s_car_plan_2_lock.velocity_conflict_ticks = 0U;
        }

        s_car_plan_2_lock.previous_x = s_car_plan_2_lock.center_x;
        s_car_plan_2_lock.previous_y = s_car_plan_2_lock.center_y;
        s_car_plan_2_lock.previous_valid = 1U;
        s_car_plan_2_lock.center_x = clusters[selected_index].center_x;
        s_car_plan_2_lock.center_y = clusters[selected_index].center_y;
        s_car_plan_2_lock.lost_ticks = 0U;
        s_car_plan_2_result = candidate_result;
        CarPlan_2_CopyResult(result);
        return 1U;
    }

    s_car_plan_2_lock.velocity_conflict_ticks = 0U;
    if(s_car_plan_2_lock.lost_ticks < CAR_PLAN_2_LOST_HOLD_TICKS)
    {
        s_car_plan_2_lock.lost_ticks++;
        CarPlan_2_CopyResult(result);
        return s_car_plan_2_result.valid;
    }

    s_car_plan_2_lock.valid = 0U;
    s_car_plan_2_lock.previous_valid = 0U;
    s_car_plan_2_lock.lost_ticks = 0U;
    CarPlan_2_ClearResult(&s_car_plan_2_result);
    (void)CarPlan_2_Acquire(clusters, cluster_count);

    CarPlan_2_CopyResult(result);
    return s_car_plan_2_result.valid;
}

/**
 * @brief 获取最近一次影子车模规划结果的快照。
 * @param result 输出规划结果；允许传入空指针。
 * @return 无。
 */
void CarPlan_2_GetResult(car_plan_2_result_t *result)
{
    CarPlan_2_CopyResult(result);
}
