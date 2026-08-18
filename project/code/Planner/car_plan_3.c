#include "car_plan_3.h"
#include "Three_Camera.h"
#include "speed_plan.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include <math.h>

#define CAR_PLAN_3_DEG_TO_RAD       (0.017453292519943295f) /* 角度转弧度系数。 */
#define CAR_PLAN_3_RAD_TO_DEG       (57.29577951308232f) /* 弧度转角度系数。 */
#define CAR_PLAN_3_MIN_DISTANCE_M   (0.20f) /* 可信车灯到信标的最小水平距离，单位 m。 */
#define CAR_PLAN_3_MAX_DISTANCE_M   (6.00f) /* 可信车灯到信标的最大水平距离，单位 m。 */
#define CAR_PLAN_3_NEAR_LAMP_DIST_PX (3.0f)  /* 信标与同摄车灯中心的近距离阈值，单位 px。 */
#define CAR_PLAN_3_TRACK_MATCH_PX    (15.0f) /* 原始信标短时轨迹匹配半径，单位 px。 */
#define CAR_PLAN_3_FAR_LAMP_DIST_PX  (10.0f) /* 允许信标靠近车灯前必须到达的历史距离，单位 px。 */
#define CAR_PLAN_3_HISTORY_TICKS     (30U)   /* 100Hz 下保留约 300ms 的远距离历史。 */
#define CAR_PLAN_3_GAP_TICKS         (2U)    /* 100Hz 下允许约 20ms 的短暂丢失。 */
#define CAR_PLAN_3_CONFIRM_TICKS     (3U)    /* 连续匹配三次后确认信标轨迹。 */

typedef struct
{
    uint8 valid;
    uint8 gap_ticks;
    uint8 sample_ticks;
    uint8 far_age_ticks;
    uint8 suspect_age_ticks;
    float x;
    float y;
} car_plan_3_beacon_track_t;

extern float g_car_yaw;
static car_plan_result_t s_car_plan_3_result;
static three_camera_result_t s_car_plan_3_camera;
static int8 s_car_plan_3_selected = -1;
static car_plan_3_beacon_track_t
    s_car_plan_3_track[IMAGE_CAMERA_COUNT][IMAGE_MAX_BEACON_COUNT]; /* 三摄原始信标短时轨迹。 */

/**
 * @brief 过滤突然出现在同摄车灯附近且没有远距离连续历史的原始信标。
 * @param filtered 输出过滤后的三摄图像数据，不得为空。
 * @return 无。
 */
static void CarPlan_3_FilterNearLamp(
    struct image_data filtered[IMAGE_CAMERA_COUNT])
{
    uint8 camera;
    uint8 index;

    for(camera = 0U; camera < (uint8)IMAGE_CAMERA_COUNT; camera++)
    {
        uint8 track_used[IMAGE_MAX_BEACON_COUNT] = {0U};
        const car_lamp_data *lamp = &image_data[camera].car_lamp_data[0];
        uint8 lamp_valid = image_data_car_lamp_valid(lamp);

        filtered[camera] = image_data[camera];
        for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
        {
            car_plan_3_beacon_track_t *track = &s_car_plan_3_track[camera][index];
            if(track->valid != 0U)
            {
                if(track->far_age_ticks <= CAR_PLAN_3_HISTORY_TICKS)
                {
                    track->far_age_ticks++;
                }
                if(track->suspect_age_ticks <= CAR_PLAN_3_HISTORY_TICKS)
                {
                    track->suspect_age_ticks++;
                }
            }
        }

        for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
        {
            const beacon_data *beacon = &image_data[camera].beacon_data[index];
            uint8 best = 0xFFU;
            uint8 matched = 0U;
            float best_distance_sq = CAR_PLAN_3_TRACK_MATCH_PX *
                                     CAR_PLAN_3_TRACK_MATCH_PX;
            uint8 track_index;
            float lamp_distance = CAR_PLAN_3_FAR_LAMP_DIST_PX;

            if(image_data_beacon_valid(beacon) == 0U)
            {
                continue;
            }
            for(track_index = 0U; track_index < IMAGE_MAX_BEACON_COUNT; track_index++)
            {
                car_plan_3_beacon_track_t *track =
                    &s_car_plan_3_track[camera][track_index];
                float dx;
                float dy;
                float distance_sq;

                if((track->valid == 0U) || (track_used[track_index] != 0U) ||
                   (track->gap_ticks > CAR_PLAN_3_GAP_TICKS))
                {
                    continue;
                }
                dx = beacon->x - track->x;
                dy = beacon->y - track->y;
                distance_sq = dx * dx + dy * dy;
                if(distance_sq < best_distance_sq)
                {
                    best = track_index;
                    matched = 1U;
                    best_distance_sq = distance_sq;
                }
            }
            if(best == 0xFFU)
            {
                for(track_index = 0U; track_index < IMAGE_MAX_BEACON_COUNT; track_index++)
                {
                    if(track_used[track_index] == 0U)
                    {
                        best = track_index;
                        break;
                    }
                }
            }

            {
                car_plan_3_beacon_track_t *track = &s_car_plan_3_track[camera][best];

                if(lamp_valid != 0U)
                {
                    float dx = beacon->x - lamp->cx;
                    float dy = beacon->y - lamp->cy;
                    lamp_distance = sqrtf(dx * dx + dy * dy);
                }
                track_used[best] = 1U;
                track->valid = 1U;
                track->gap_ticks = 0U;
                track->sample_ticks = matched
                                          ? (uint8)((track->sample_ticks < 0xFFU)
                                                        ? track->sample_ticks + 1U
                                                        : 0xFFU)
                                          : 1U;
                if(matched == 0U)
                {
                    track->far_age_ticks = CAR_PLAN_3_HISTORY_TICKS + 1U;
                    track->suspect_age_ticks =
                        ((lamp_valid != 0U) &&
                         (lamp_distance < CAR_PLAN_3_TRACK_MATCH_PX))
                            ? 0U
                            : CAR_PLAN_3_HISTORY_TICKS + 1U;
                }
                track->x = beacon->x;
                track->y = beacon->y;
                if(lamp_valid != 0U)
                {
                    if(lamp_distance >= CAR_PLAN_3_FAR_LAMP_DIST_PX)
                    {
                        track->far_age_ticks = 0U;
                    }
                }
                if((track->suspect_age_ticks <= CAR_PLAN_3_HISTORY_TICKS) ||
                   ((lamp_valid != 0U) &&
                    (lamp_distance < CAR_PLAN_3_NEAR_LAMP_DIST_PX) &&
                    ((track->sample_ticks < CAR_PLAN_3_CONFIRM_TICKS) ||
                     (track->far_age_ticks > CAR_PLAN_3_HISTORY_TICKS))))
                {
                    image_data_clear_beacon(&filtered[camera].beacon_data[index]);
                }
            }
        }

        for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
        {
            car_plan_3_beacon_track_t *track = &s_car_plan_3_track[camera][index];
            if((track->valid != 0U) && (track_used[index] == 0U) &&
               (track->gap_ticks < 0xFFU))
            {
                track->gap_ticks++;
            }
            if(track->gap_ticks > CAR_PLAN_3_GAP_TICKS)
            {
                track->valid = 0U;
                track->sample_ticks = 0U;
                track->far_age_ticks = CAR_PLAN_3_HISTORY_TICKS + 1U;
                track->suspect_age_ticks = CAR_PLAN_3_HISTORY_TICKS + 1U;
            }
        }
    }
}

static void CarPlan_3_ClearResult(void)
{
    s_car_plan_3_result.valid = 0U;
    s_car_plan_3_result.target_strafe_mps = 0.0f;
    s_car_plan_3_result.target_forward_mps = 0.0f;
    s_car_plan_3_selected = -1;
}

void CarPlan_3_Reset(void)
{
    uint8 camera;
    uint8 i;

    CarPlan_3_ClearResult();
    SpeedPlan_Reset();
    s_car_plan_3_camera.car_lamp.valid = 0U;
    s_car_plan_3_camera.beacon_count = 0U;
    for(i = 0U; i < THREE_CAMERA_MAX_BEACON_COUNT; i++)
    {
        s_car_plan_3_camera.beacon[i].valid = 0U;
    }
    for(camera = 0U; camera < (uint8)IMAGE_CAMERA_COUNT; camera++)
    {
        for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
        {
            s_car_plan_3_track[camera][i].valid = 0U;
            s_car_plan_3_track[camera][i].gap_ticks = 0U;
            s_car_plan_3_track[camera][i].sample_ticks = 0U;
            s_car_plan_3_track[camera][i].far_age_ticks =
                CAR_PLAN_3_HISTORY_TICKS + 1U;
            s_car_plan_3_track[camera][i].suspect_age_ticks =
                CAR_PLAN_3_HISTORY_TICKS + 1U;
            s_car_plan_3_track[camera][i].x = 0.0f;
            s_car_plan_3_track[camera][i].y = 0.0f;
        }
    }
}

/**
 * @brief 使用三摄 Double Sphere 几何计算最近信标的车体系目标速度，同摄组合优先于跨摄组合。
 * @param result 输出车体系横向和前向目标速度；允许传入空指针。
 * @return 成功得到 0.20-6.00 m 内可信目标方向时返回 1，否则清空输出并返回 0。
 */
uint8 CarPlan_3_Update(car_plan_result_t *result)
{
    struct image_data filtered[IMAGE_CAMERA_COUNT];
    uint8 i;
    uint8 selected = 0xFFU;
    uint8 same_pair_available = 0U;
    float selected_distance_sq = 0.0f;
    float dx;
    float dy;
    float distance;
    float angle_rad;
    float right_x;
    float right_y;
    float target_strafe;
    float target_forward;
    float plan_speed;

    CarPlan_3_ClearResult();
    CarPlan_3_FilterNearLamp(filtered);
    if(Three_Camera_Update(filtered,
                           g_euler.roll,
                           g_euler.pitch,
                           g_euler.yaw,
                           g_tof_fused_height_mm,
                           g_tof_fused_valid,
                           &s_car_plan_3_camera) == 0U ||
       s_car_plan_3_camera.car_lamp.valid == 0U)
    {
        (void)SpeedPlan_Update(0U, 0.0f, 0.0f);
        if(result != 0)
        {
            *result = s_car_plan_3_result;
        }
        return 0U;
    }

    /* 只要存在同摄组合，本帧就不使用跨摄组合。 */
    for(i = 0U; i < s_car_plan_3_camera.beacon_count; i++)
    {
        if((s_car_plan_3_camera.beacon[i].valid != 0U) &&
           (s_car_plan_3_camera.beacon[i].pair_valid != 0U) &&
           (s_car_plan_3_camera.beacon[i].pair_same_camera != 0U))
        {
            same_pair_available = 1U;
            break;
        }
    }

    /* 在允许的同摄或跨摄组合中选择水平距离最近的物理信标。 */
    for(i = 0U; i < s_car_plan_3_camera.beacon_count; i++)
    {
        float candidate_dx;
        float candidate_dy;
        float candidate_distance_sq;

        if((s_car_plan_3_camera.beacon[i].valid == 0U) ||
           (s_car_plan_3_camera.beacon[i].pair_valid == 0U) ||
           ((same_pair_available != 0U) &&
            (s_car_plan_3_camera.beacon[i].pair_same_camera == 0U)))
        {
            continue;
        }
        candidate_dx = s_car_plan_3_camera.beacon[i].pair_dx_m;
        candidate_dy = s_car_plan_3_camera.beacon[i].pair_dy_m;
        candidate_distance_sq = candidate_dx * candidate_dx +
                                candidate_dy * candidate_dy;
        if(selected == 0xFFU || candidate_distance_sq < selected_distance_sq)
        {
            selected = i;
            selected_distance_sq = candidate_distance_sq;
        }
    }

    if(selected == 0xFFU ||
       selected_distance_sq < CAR_PLAN_3_MIN_DISTANCE_M * CAR_PLAN_3_MIN_DISTANCE_M ||
       selected_distance_sq > CAR_PLAN_3_MAX_DISTANCE_M * CAR_PLAN_3_MAX_DISTANCE_M)
    {
        (void)SpeedPlan_Update(0U, 0.0f, 0.0f);
        if(result != 0)
        {
            *result = s_car_plan_3_result;
        }
        return 0U;
    }

    dx = s_car_plan_3_camera.beacon[selected].pair_dx_m;
    dy = s_car_plan_3_camera.beacon[selected].pair_dy_m;
    distance = sqrtf(selected_distance_sq);
    angle_rad = s_car_plan_3_camera.beacon[selected].pair_lamp_angle_deg *
                CAR_PLAN_3_DEG_TO_RAD;
    right_x = cosf(angle_rad);
    right_y = sinf(angle_rad);
    /* 车灯长轴是车体横轴；yaw只用于将无向长轴统一到车体右向。 */
    if(cosf((s_car_plan_3_camera.beacon[selected].pair_lamp_angle_deg -
             g_car_yaw - 90.0f) *
            CAR_PLAN_3_DEG_TO_RAD) < 0.0f)
    {
        right_x = -right_x;
        right_y = -right_y;
    }

    target_strafe = (dx * right_x + dy * right_y) / distance;
    target_forward = (dx * right_y - dy * right_x) / distance;
    plan_speed = SpeedPlan_Update(1U,
                                  distance,
                                  atan2f(fabsf(target_strafe), target_forward) *
                                      CAR_PLAN_3_RAD_TO_DEG);
    s_car_plan_3_result.valid = 1U;
    s_car_plan_3_result.target_strafe_mps = target_strafe * plan_speed;
    s_car_plan_3_result.target_forward_mps = target_forward * plan_speed;
    s_car_plan_3_selected = (int8)selected;

    if(result != 0)
    {
        *result = s_car_plan_3_result;
    }
    return 1U;
}

void CarPlan_3_GetResult(car_plan_result_t *result)
{
    if(result != 0)
    {
        *result = s_car_plan_3_result;
    }
}

void CarPlan_3_GetDebug(car_plan_3_debug_t *debug)
{
    uint8 i;
    uint8 output_count;
    int8 selected_output = s_car_plan_3_selected;

    if(debug == 0)
    {
        return;
    }
    for(i = 0U; i < CAR_PLAN_3_DEBUG_BEACON_COUNT; i++)
    {
        debug->beacon[i].valid = 0U;
        debug->beacon[i].camera_mask = 0U;
        debug->beacon[i].center_x = IMAGE_DATA_INVALID_VALUE;
        debug->beacon[i].center_y = IMAGE_DATA_INVALID_VALUE;
        debug->beacon[i].area = 0.0f;
    }
    debug->car_lamp.valid = s_car_plan_3_camera.car_lamp.valid;
    debug->car_lamp.camera_mask = s_car_plan_3_camera.car_lamp.camera_mask;
    debug->car_lamp.center_x = s_car_plan_3_camera.car_lamp.x_m;
    debug->car_lamp.center_y = s_car_plan_3_camera.car_lamp.y_m;
    debug->car_lamp.angle_deg = s_car_plan_3_camera.car_lamp.angle_deg;
    debug->selected_target_id = -1;

    output_count = s_car_plan_3_camera.beacon_count;
    if(output_count > CAR_PLAN_3_DEBUG_BEACON_COUNT)
    {
        output_count = CAR_PLAN_3_DEBUG_BEACON_COUNT;
    }
    for(i = 0U; i < output_count; i++)
    {
        debug->beacon[i].valid = s_car_plan_3_camera.beacon[i].valid;
        debug->beacon[i].camera_mask = s_car_plan_3_camera.beacon[i].camera_mask;
        debug->beacon[i].center_x = s_car_plan_3_camera.beacon[i].x_m;
        debug->beacon[i].center_y = s_car_plan_3_camera.beacon[i].y_m;
        debug->beacon[i].area = s_car_plan_3_camera.beacon[i].area;
    }
    if(s_car_plan_3_result.valid != 0U)
    {
        debug->selected_target_id = selected_output;
    }
}
