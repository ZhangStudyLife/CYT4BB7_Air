/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#include "car_plan_4.h"
#include "Three_Camera.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../FlightController/fc_loop.h"
#include <math.h>

#define CAR_PLAN_4_DEG_TO_RAD       (0.017453292519943295f) /* 角度转弧度系数。 */
#define CAR_PLAN_4_RAD_TO_DEG       (57.29577951308232f) /* 弧度转角度系数。 */
#define CAR_PLAN_4_MIN_DISTANCE_M   (0.20f) /* 可信车灯到信标的最小水平距离，单位 m。 */
#define CAR_PLAN_4_MAX_DISTANCE_M   (6.00f) /* 可信车灯到信标的最大水平距离，单位 m。 */
#define CAR_PLAN_4_NEAR_LAMP_DIST_PX (3.0f)  /* 信标与同摄车灯中心的近距离阈值，单位 px。 */
#define CAR_PLAN_4_TRACK_MATCH_PX    (15.0f) /* 原始信标短时轨迹匹配半径，单位 px。 */
#define CAR_PLAN_4_FAR_LAMP_DIST_PX  (10.0f) /* 允许信标靠近车灯前必须到达的历史距离，单位 px。 */
#define CAR_PLAN_4_HISTORY_TICKS     (30U)   /* 100Hz 下保留约 300ms 的远距离历史。 */
#define CAR_PLAN_4_GAP_TICKS         (2U)    /* 100Hz 下允许约 20ms 的短暂丢失。 */
#define CAR_PLAN_4_CONFIRM_TICKS     (2U)    /* 连续匹配两次后确认信标轨迹。 */
#define CAR_PLAN_4_EDGE_MARGIN_PX    (20.0f) /* 进入惯导保持前的图像边缘余量，单位 px。 */
#define CAR_PLAN_4_ATTITUDE_GATE_DEG (12.0f) /* 允许进入惯导保持的 Roll/Pitch 模长，单位 deg。 */
#define CAR_PLAN_4_COAST_MAX_MS      (400U)  /* COAST 阶段惯导保持最长时间，单位 ms。 */
#define CAR_PLAN_4_FAR_DISTANCE_M    (1.20f) /* 远距离 COAST 准入边界，单位 m。 */
#define CAR_PLAN_4_FAR_COAST_MAX_MS  (520U)  /* 远距离 COAST 惯导保持最长时间，单位 ms。 */
#define CAR_PLAN_4_AGGRESSIVE_DISTANCE_M (2.00f) /* 强信任 COAST 的起始距离，单位 m。 */
#define CAR_PLAN_4_AGGRESSIVE_COAST_MAX_MS (960U) /* 强信任 COAST 最长盲航时间，单位 ms。 */
#define CAR_PLAN_4_COAST_STOP_DISTANCE_M (0.60f) /* COAST 预测距离到该值时立即退出，单位 m。 */
#define CAR_PLAN_4_CAR_DATA_WARN_MS  (50U)   /* 车数据超过该年龄后不再扩展惯导时间，单位 ms。 */
#define CAR_PLAN_4_CAR_DATA_STOP_MS  (100U)  /* 车数据超过该年龄后停止惯导，单位 ms。 */
#define CAR_PLAN_4_IMAGE_TIMEOUT_MS   (50U)   /* 目标来源相机超过该时间未更新时按丢失处理，单位 ms。 */
#define CAR_PLAN_4_NO_VISION_TICKS   (2U)    /* 连续两次规划周期没有新图像后进入丢失判断。 */
#define CAR_PLAN_4_INNOVATION_BASE_M (0.35f) /* 世界相对向量基础创新门限，单位 m。 */
#define CAR_PLAN_4_INNOVATION_GAIN_S (0.80f) /* 车速传播造成的创新门限增益，单位 s。 */
#define CAR_PLAN_4_INNOVATION_MAX_M  (1.20f) /* 世界相对向量最大创新门限，单位 m。 */
#define CAR_PLAN_4_REACQUIRE_DIST_M  (0.50f) /* 重捕获连续观测之间的最大距离，单位 m。 */
#define CAR_PLAN_4_AMBIGUITY_MARGIN_M (0.15f) /* 最佳与次佳创新小于该差值时拒绝歧义候选，单位 m。 */
#define CAR_PLAN_4_LAMP_ANGLE_GATE_DEG (30.0f) /* 车灯长轴相对惯导预测的最大创新，单位 deg。 */
#define CAR_PLAN_4_DIRECTION_COS_SQ  (0.8213938f) /* 相对方向创新 25deg 对应的最小余弦平方。 */
#define CAR_PLAN_4_OUTPUT_COS_MIN     (0.8660254f) /* 重捕获速度方向最大变化 30deg 对应的最小余弦。 */
#define CAR_PLAN_4_SEARCH_FORWARD_MIN (0.20f) /* 路线抢占只考虑车体前方约 78deg 内的信标。 */
#define CAR_PLAN_4_ROUTE_NEARER_MARGIN_M (0.35f) /* 新前向候选必须至少近于旧目标的距离，单位 m。 */
#define CAR_PLAN_4_ROUTE_NEARER_RATIO (0.80f) /* 或者新候选距离不超过旧目标的 80%。 */
#define CAR_PLAN_4_ROUTE_STRONG_RATIO (0.60f) /* 强优势路线候选相对当前目标的最大距离比例。 */
#define CAR_PLAN_4_ROUTE_TURN_MARGIN_DEG (25.0f) /* 普通抢占允许增加的最大转向角，单位 deg。 */
#define CAR_PLAN_4_ROUTE_CONFIRM_TICKS (2U) /* 新路线候选连续两次观测后才允许抢占。 */

#define CAR_PLAN_4_STATE_SEARCH       (0U)    /* 没有确认目标。 */
#define CAR_PLAN_4_STATE_TRACK        (1U)    /* 有新图像确认目标。 */
#define CAR_PLAN_4_STATE_COAST        (2U)    /* 图像短时丢失，使用车模惯导。 */
#define CAR_PLAN_4_ROUTE_PENDING_HOLD (2U)    /* 后方旧目标等待前向路线二次确认时直行保持。 */
#define CAR_PLAN_4_COAST_LEVEL_NEAR  (0U)    /* 近距离保守 COAST。 */
#define CAR_PLAN_4_COAST_LEVEL_FAR   (1U)    /* 远距离 COAST。 */
#define CAR_PLAN_4_COAST_LEVEL_AGGRESSIVE (2U) /* 2m 以上强信任 COAST。 */

typedef struct
{
    uint8 valid;
    uint8 gap_ticks;
    uint8 sample_ticks;
    uint8 far_age_ticks;
    uint8 suspect_age_ticks;
    float x;
    float y;
} car_plan_4_beacon_track_t;

extern float g_car_yaw; /* 车模世界 yaw，单位 deg。 */
extern float g_car_vel_x; /* 车体右向实际速度，单位 m/s。 */
extern float g_car_vel_y; /* 车体前向实际速度，单位 m/s。 */
extern float g_car_sync_time_ms; /* 最近一次车端同步时间戳，单位 ms。 */
extern uint32 g_car_last_update_time_ms; /* 最近一次车端数据的本机接收时刻，单位 ms。 */
extern volatile uint32 tick_1000us_cnt; /* 本机毫秒时基。 */
extern volatile uint32 g_image_camera_rx_seq[IMAGE_CAMERA_COUNT]; /* 三路相机真实结果接收序号。 */
static car_plan_result_t s_car_plan_4_result;
static three_camera_result_t s_car_plan_4_camera;
static int8 s_car_plan_4_selected = -1;
static uint8 s_car_plan_4_state = CAR_PLAN_4_STATE_SEARCH; /* 目标跟踪状态。 */
static uint8 s_car_plan_4_confirm_count = 0U; /* 搜索阶段连续确认的新图像数。 */
static uint8 s_car_plan_4_reacquire_count = 0U; /* COAST 阶段三帧窗口内的重捕获命中数。 */
static uint8 s_car_plan_4_reacquire_age = 0U; /* COAST 阶段重捕获确认窗口年龄，单位新图像。 */
static uint8 s_car_plan_4_no_vision_ticks = 0U; /* 未收到可信新观测的连续规划周期数。 */
static uint8 s_car_plan_4_coast_level = CAR_PLAN_4_COAST_LEVEL_NEAR; /* 当前 COAST 策略等级。 */
static uint8 s_car_plan_4_track_camera_mask = 0U; /* 最近可信目标的来源相机位掩码。 */
static uint32 s_car_plan_4_last_update_tick = 0U; /* 上次惯导传播的系统时刻，单位 ms。 */
static uint32 s_car_plan_4_last_accept_tick = 0U; /* 上次接纳图像观测的系统时刻，单位 ms。 */
static uint32 s_car_plan_4_last_camera_tick = 0U; /* 目标来源相机最近一次真实更新时刻，单位 ms。 */
static uint32 s_car_plan_4_camera_seq[IMAGE_CAMERA_COUNT]; /* 规划器已处理的各相机真实结果序号。 */
static uint32 s_car_plan_4_filter_camera_seq[IMAGE_CAMERA_COUNT]; /* 近车灯过滤器已处理的各相机序号。 */
static float s_car_plan_4_track_dx_m = 0.0f; /* 当前车灯到锁定信标的世界 X 相对量，单位 m。 */
static float s_car_plan_4_track_dy_m = 0.0f; /* 当前车灯到锁定信标的世界 Y 相对量，单位 m。 */
static float s_car_plan_4_track_lamp_angle_deg = 0.0f; /* 最近可信车灯世界长轴角，单位 deg。 */
static float s_car_plan_4_track_car_yaw_deg = 0.0f; /* 最近可信观测对应的车模世界 yaw，单位 deg。 */
static float s_car_plan_4_track_speed_mps = 0.0f; /* 最近可信规划速度模长，单位 m/s。 */
static float s_car_plan_4_pending_dx_m = 0.0f; /* 待确认信标的世界 X 相对量，单位 m。 */
static float s_car_plan_4_pending_dy_m = 0.0f; /* 待确认信标的世界 Y 相对量，单位 m。 */
static float s_car_plan_4_pending_lamp_angle_deg = 0.0f; /* 待确认车灯世界长轴角，单位 deg。 */
static float s_car_plan_4_pending_car_yaw_deg = 0.0f; /* 待确认观测对应的车模世界 yaw，单位 deg。 */
static uint8 s_car_plan_4_route_challenge_count = 0U; /* 新近灯路线候选的连续观测数。 */
static uint8 s_car_plan_4_route_challenge_hold = 0U; /* 后方旧目标等待前向候选确认时的直行保持标志。 */
static uint32 s_car_plan_4_route_challenge_tick = 0U; /* 新近灯路线候选最近观测时刻，单位 ms。 */
static float s_car_plan_4_route_challenge_dx_m = 0.0f; /* 新近灯路线候选的世界 X 相对量，单位 m。 */
static float s_car_plan_4_route_challenge_dy_m = 0.0f; /* 新近灯路线候选的世界 Y 相对量，单位 m。 */
static float s_car_plan_4_last_edge_margin_px = 1000.0f; /* 最近可信观测的最小图像边缘余量，单位 px。 */
static struct image_data s_car_plan_4_filtered[IMAGE_CAMERA_COUNT]; /* 各相机最近一次真实新图像的过滤结果。 */
static car_plan_4_beacon_track_t
    s_car_plan_4_track[IMAGE_CAMERA_COUNT][IMAGE_MAX_BEACON_COUNT]; /* 三摄原始信标短时轨迹。 */

/* 速度档位状态机持有本规划器私有的静态状态。
 * car_plan_entry 每帧顺序运行全部四套规划器，若与 car_plan_3 共用 speed_plan.c
 * 的文件级静态变量，确认计数会被双倍累加、fast 标志会被两套判据交替翻转。
 * 普通通道判据与参数和 speed_plan.c 完全一致；
 * 快速通道为双灯同线加速，满足时直接进入快档，不等待 ttc 与姿态收敛。 */
#define CAR_PLAN_4_SPEED_MIN_FORWARD_MPS    (0.5f)
#define CAR_PLAN_4_SPEED_ENTER_TTC_S        (0.50f)
#define CAR_PLAN_4_SPEED_ENTER_ANGLE_DEG    (24.0f)
#define CAR_PLAN_4_SPEED_ENTER_YAWRATE_DPS  (140.0f)
#define CAR_PLAN_4_SPEED_EXIT_TTC_S         (0.40f)
#define CAR_PLAN_4_SPEED_EXIT_ANGLE_DEG     (30.0f)
#define CAR_PLAN_4_SPEED_EXIT_YAWRATE_DPS   (100.0f)
#define CAR_PLAN_4_SPEED_CONFIRM_TICKS      (6U) /* 100Hz下连续约60ms。 */
#define CAR_PLAN_4_DUAL_ANGLE_DEG           (35.0f) /* 双灯同线目标速度向量最大夹角，单位 deg。 */
#define CAR_PLAN_4_DUAL_TARGET_ANGLE_DEG    (24.0f) /* 双灯同线目标灯最大方向角，单位 deg。 */
#define CAR_PLAN_4_DUAL_YAWRATE_DPS         (150.0f) /* 双灯同线允许的最大 yaw 角速率，单位 deg/s。 */
#define CAR_PLAN_4_DUAL_SECOND_DIST_M       (1.20f) /* 双灯同线第二灯的最小距离，单位 m。 */

extern float g_car_yaw_rate_dps; /* 车模 yaw 角速率，单位 deg/s。 */

static uint8 s_car_plan_4_speed_fast = 0U; /* 当前是否处于快速速度档。 */
static uint8 s_car_plan_4_speed_condition_ticks = 0U; /* 档位切换条件的连续成立周期数。 */

static void CarPlan_4_SpeedPlanReset(void)
{
    s_car_plan_4_speed_fast = 0U;
    s_car_plan_4_speed_condition_ticks = 0U;
}

static uint8 CarPlan_4_GetDirection(float dx_m,
                                    float dy_m,
                                    float lamp_angle_deg,
                                    float *target_strafe,
                                    float *target_forward,
                                    float *distance_m);

/**
 * @brief 判断双灯同线快速通道是否成立：目标灯前方还有第二个前向信标，
 *        且两灯目标速度单位向量夹角小、目标灯方向正、第二灯距离足够远。
 * @param selected 当前锁定目标的三摄融合信标槽位。
 * @return 双灯同线条件成立返回 1，否则返回 0。
 */
static uint8 CarPlan_4_CheckDualLineFast(uint8 selected)
{
    uint8 i;
    uint8 second_found = 0U;
    float target_strafe;
    float target_forward;
    float target_distance;
    float target_angle;
    float best_dot = -1.0f;
    float second_distance = 0.0f;
    const three_camera_beacon_t *beacon;

    if(selected >= s_car_plan_4_camera.beacon_count)
    {
        return 0U;
    }
    beacon = &s_car_plan_4_camera.beacon[selected];
    if(CarPlan_4_GetDirection(beacon->pair_dx_m,
                              beacon->pair_dy_m,
                              beacon->pair_lamp_angle_deg,
                              &target_strafe,
                              &target_forward,
                              &target_distance) == 0U)
    {
        return 0U;
    }
    if(target_forward <= 0.0f)
    {
        return 0U;
    }
    target_angle = atan2f(fabsf(target_strafe), target_forward) *
                   CAR_PLAN_4_RAD_TO_DEG;
    if(target_angle >= CAR_PLAN_4_DUAL_TARGET_ANGLE_DEG)
    {
        return 0U;
    }
    if(fabsf(g_car_yaw_rate_dps) >= CAR_PLAN_4_DUAL_YAWRATE_DPS)
    {
        return 0U;
    }
    for(i = 0U; i < s_car_plan_4_camera.beacon_count; i++)
    {
        const three_camera_beacon_t *item = &s_car_plan_4_camera.beacon[i];
        float strafe;
        float forward;
        float distance;
        float dot;

        if((item->valid == 0U) || (item->pair_valid == 0U) || (i == selected))
        {
            continue;
        }
        if(CarPlan_4_GetDirection(item->pair_dx_m,
                                  item->pair_dy_m,
                                  item->pair_lamp_angle_deg,
                                  &strafe,
                                  &forward,
                                  &distance) == 0U)
        {
            continue;
        }
        if(forward <= 0.0f)
        {
            continue;
        }
        dot = target_strafe * strafe + target_forward * forward;
        if(dot > best_dot)
        {
            best_dot = dot;
            second_distance = distance;
            second_found = 1U;
        }
    }
    if(second_found == 0U)
    {
        return 0U;
    }
    if(second_distance < CAR_PLAN_4_DUAL_SECOND_DIST_M)
    {
        return 0U;
    }
    /* best_dot 对应两灯目标速度向量夹角的最小值，夹角 = acos(dot)。 */
    if(acosf(best_dot) * CAR_PLAN_4_RAD_TO_DEG >= CAR_PLAN_4_DUAL_ANGLE_DEG)
    {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 根据目标距离、方向和当前跟随状态选择车模速度模长。
 *        快速通道（双灯同线）成立时直接进入快档，不等待 ttc 与姿态收敛；
 *        普通通道判据与参数和 speed_plan.c 一致。
 * @param target_valid 目标方向有效时为1，否则为0。
 * @param target_distance_m 车灯到目标信标的水平距离，单位m。
 * @param target_angle_deg 目标方向与车体正前方的夹角，范围0至180deg。
 * @param dual_line_fast 双灯同线快速通道成立时为1，否则为0。
 * @return Car_Speed或Car_Speed_Fast。
 */
static float CarPlan_4_SpeedPlanUpdate(uint8 target_valid,
                                       float target_distance_m,
                                       float target_angle_deg,
                                       uint8 dual_line_fast)
{
    float forward_speed;
    float ttc;
    uint8 condition_met;

    if(target_valid == 0U)
    {
        s_car_plan_4_speed_condition_ticks = 0U;
        return (s_car_plan_4_speed_fast != 0U) ? Car_Speed_Fast : Car_Speed;
    }

    forward_speed = (g_car_vel_y > CAR_PLAN_4_SPEED_MIN_FORWARD_MPS) ?
                        g_car_vel_y : CAR_PLAN_4_SPEED_MIN_FORWARD_MPS;
    ttc = target_distance_m / forward_speed;

    if(s_car_plan_4_speed_fast == 0U)
    {
        if(dual_line_fast != 0U)
        {
            /* 双灯同线快速通道：直接进入快档，不等待 ttc 与姿态收敛。 */
            s_car_plan_4_speed_fast = 1U;
            s_car_plan_4_speed_condition_ticks = 0U;
            return Car_Speed_Fast;
        }
        condition_met = ((ttc > CAR_PLAN_4_SPEED_ENTER_TTC_S) &&
                         (target_angle_deg < CAR_PLAN_4_SPEED_ENTER_ANGLE_DEG) &&
                         (fabsf(g_car_yaw_rate_dps) < CAR_PLAN_4_SPEED_ENTER_YAWRATE_DPS)) ? 1U : 0U;
    }
    else
    {
        condition_met = ((ttc < CAR_PLAN_4_SPEED_EXIT_TTC_S) ||
                         (target_angle_deg > CAR_PLAN_4_SPEED_EXIT_ANGLE_DEG) ||
                         (fabsf(g_car_yaw_rate_dps) > CAR_PLAN_4_SPEED_EXIT_YAWRATE_DPS)) ? 1U : 0U;
    }

    if(condition_met != 0U)
    {
        s_car_plan_4_speed_condition_ticks++;
        if(s_car_plan_4_speed_condition_ticks >= CAR_PLAN_4_SPEED_CONFIRM_TICKS)
        {
            s_car_plan_4_speed_fast = (s_car_plan_4_speed_fast == 0U) ? 1U : 0U;
            s_car_plan_4_speed_condition_ticks = 0U;
        }
    }
    else
    {
        s_car_plan_4_speed_condition_ticks = 0U;
    }

    return (s_car_plan_4_speed_fast != 0U) ? Car_Speed_Fast : Car_Speed;
}

/**
 * @brief 过滤突然出现在同摄车灯附近且没有远距离连续历史的原始信标。
 * @param filtered 输出过滤后的三摄图像数据，不得为空。
 * @return 无。
 */
static void CarPlan_4_FilterNearLamp(
    struct image_data filtered[IMAGE_CAMERA_COUNT])
{
    uint8 camera;
    uint8 index;

    for(camera = 0U; camera < (uint8)IMAGE_CAMERA_COUNT; camera++)
    {
        uint8 track_used[IMAGE_MAX_BEACON_COUNT] = {0U};
        const car_lamp_data *lamp = &image_data[camera].car_lamp_data[0];
        uint8 lamp_valid = image_data_car_lamp_valid(lamp);

        if(g_image_camera_rx_seq[camera] == s_car_plan_4_filter_camera_seq[camera])
        {
            filtered[camera] = s_car_plan_4_filtered[camera];
            continue;
        }
        s_car_plan_4_filter_camera_seq[camera] = g_image_camera_rx_seq[camera];
        filtered[camera] = image_data[camera];
        for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
        {
            car_plan_4_beacon_track_t *track = &s_car_plan_4_track[camera][index];
            if(track->valid != 0U)
            {
                if(track->far_age_ticks <= CAR_PLAN_4_HISTORY_TICKS)
                {
                    track->far_age_ticks++;
                }
                if(track->suspect_age_ticks <= CAR_PLAN_4_HISTORY_TICKS)
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
            float best_distance_sq = CAR_PLAN_4_TRACK_MATCH_PX *
                                     CAR_PLAN_4_TRACK_MATCH_PX;
            uint8 track_index;
            float lamp_distance = CAR_PLAN_4_FAR_LAMP_DIST_PX;

            if(image_data_beacon_valid(beacon) == 0U)
            {
                continue;
            }
            for(track_index = 0U; track_index < IMAGE_MAX_BEACON_COUNT; track_index++)
            {
                car_plan_4_beacon_track_t *track =
                    &s_car_plan_4_track[camera][track_index];
                float dx;
                float dy;
                float distance_sq;

                if((track->valid == 0U) || (track_used[track_index] != 0U) ||
                   (track->gap_ticks > CAR_PLAN_4_GAP_TICKS))
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
                car_plan_4_beacon_track_t *track = &s_car_plan_4_track[camera][best];

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
                    track->far_age_ticks = CAR_PLAN_4_HISTORY_TICKS + 1U;
                    track->suspect_age_ticks =
                        ((lamp_valid != 0U) &&
                         (lamp_distance < CAR_PLAN_4_TRACK_MATCH_PX))
                            ? 0U
                            : CAR_PLAN_4_HISTORY_TICKS + 1U;
                }
                track->x = beacon->x;
                track->y = beacon->y;
                if(lamp_valid != 0U)
                {
                    if(lamp_distance >= CAR_PLAN_4_FAR_LAMP_DIST_PX)
                    {
                        track->far_age_ticks = 0U;
                    }
                }
                if((track->suspect_age_ticks <= CAR_PLAN_4_HISTORY_TICKS) ||
                   ((lamp_valid != 0U) &&
                    (lamp_distance < CAR_PLAN_4_NEAR_LAMP_DIST_PX) &&
                    ((track->sample_ticks < CAR_PLAN_4_CONFIRM_TICKS) ||
                     (track->far_age_ticks > CAR_PLAN_4_HISTORY_TICKS))))
                {
                    image_data_clear_beacon(&filtered[camera].beacon_data[index]);
                }
            }
        }

        for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
        {
            car_plan_4_beacon_track_t *track = &s_car_plan_4_track[camera][index];
            if((track->valid != 0U) && (track_used[index] == 0U) &&
               (track->gap_ticks < 0xFFU))
            {
                track->gap_ticks++;
            }
            if(track->gap_ticks > CAR_PLAN_4_GAP_TICKS)
            {
                track->valid = 0U;
                track->sample_ticks = 0U;
                track->far_age_ticks = CAR_PLAN_4_HISTORY_TICKS + 1U;
                track->suspect_age_ticks = CAR_PLAN_4_HISTORY_TICKS + 1U;
            }
        }
        s_car_plan_4_filtered[camera] = filtered[camera];
    }
}

/**
 * @brief 获取本规划周期内真正更新的相机位掩码，并记录已处理序号。
 * @param 无。
 * @return 位 0-2 分别表示 Front、Center、Back 是否产生了真实新图像。
 */
static uint8 CarPlan_4_GetNewCameraMask(void)
{
    uint8 camera;
    uint8 mask = 0U;

    for(camera = 0U; camera < (uint8)IMAGE_CAMERA_COUNT; camera++)
    {
        if(g_image_camera_rx_seq[camera] != s_car_plan_4_camera_seq[camera])
        {
            s_car_plan_4_camera_seq[camera] = g_image_camera_rx_seq[camera];
            mask |= (uint8)(1U << camera);
        }
    }
    return mask;
}

/**
 * @brief 计算指定来源相机内已过滤信标到 188x120 图像边界的最小余量。
 * @param camera_mask 需要检查的来源相机位掩码。
 * @return 有效信标的最小边缘余量，单位 px；无有效信标时返回 1000。
 */
static float CarPlan_4_GetEdgeMarginPx(uint8 camera_mask)
{
    uint8 camera;
    uint8 index;
    float margin = 1000.0f;

    for(camera = 0U; camera < (uint8)IMAGE_CAMERA_COUNT; camera++)
    {
        if((camera_mask & (uint8)(1U << camera)) == 0U)
        {
            continue;
        }
        for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
        {
            const beacon_data *beacon =
                &s_car_plan_4_filtered[camera].beacon_data[index];
            float x_margin;
            float y_margin;
            float current;

            if(image_data_beacon_valid(beacon) == 0U)
            {
                continue;
            }
            x_margin = 94.0f - fabsf(beacon->x);
            y_margin = 60.0f - fabsf(beacon->y);
            current = (x_margin < y_margin) ? x_margin : y_margin;
            if(current < margin)
            {
                margin = current;
            }
        }
    }
    return margin;
}

/**
 * @brief 判断车模速度与航向数据是否在指定时间内保持新鲜。
 * @param max_age_ms 允许的最大未更新时间，单位 ms。
 * @return 数据有效且未超时时返回 1，否则返回 0。
 */
static uint8 CarPlan_4_CarDataFresh(uint32 max_age_ms)
{
    return ((g_car_sync_time_ms > 0.0f) &&
            ((tick_1000us_cnt - g_car_last_update_time_ms) <= max_age_ms) &&
            isfinite(g_car_vel_x) && isfinite(g_car_vel_y) &&
            isfinite(g_car_yaw)) ? 1U : 0U;
}

/**
 * @brief 使用车体右向和前向速度传播车灯到静止信标的世界相对向量。
 * @param dt_s 本次传播时间，单位 s，范围 0-0.05。
 * @return 无。
 */
static void CarPlan_4_Propagate(float dt_s)
{
    float yaw_rad;
    float velocity_x;
    float velocity_y;

    yaw_rad = g_car_yaw * CAR_PLAN_4_DEG_TO_RAD;
    velocity_x = g_car_vel_y * cosf(yaw_rad) - g_car_vel_x * sinf(yaw_rad);
    velocity_y = g_car_vel_y * sinf(yaw_rad) + g_car_vel_x * cosf(yaw_rad);
    if(s_car_plan_4_state != CAR_PLAN_4_STATE_SEARCH)
    {
        s_car_plan_4_track_dx_m -= velocity_x * dt_s;
        s_car_plan_4_track_dy_m -= velocity_y * dt_s;
    }
    if((s_car_plan_4_confirm_count != 0U) ||
       (s_car_plan_4_reacquire_count != 0U))
    {
        s_car_plan_4_pending_dx_m -= velocity_x * dt_s;
        s_car_plan_4_pending_dy_m -= velocity_y * dt_s;
    }
    if(s_car_plan_4_route_challenge_count != 0U)
    {
        s_car_plan_4_route_challenge_dx_m -= velocity_x * dt_s;
        s_car_plan_4_route_challenge_dy_m -= velocity_y * dt_s;
    }
}

/**
 * @brief 使用车模 yaw 变化传播车灯的世界长轴角。
 * @param angle_deg 参考车灯世界长轴角，单位 deg。
 * @param yaw_deg 参考观测对应的车模世界 yaw，单位 deg。
 * @return 传播到当前车模 yaw 的车灯世界长轴角，单位 deg。
 */
static float CarPlan_4_PredictLampAngle(float angle_deg, float yaw_deg)
{
    float yaw_delta = g_car_yaw - yaw_deg;

    while(yaw_delta > 180.0f)
    {
        yaw_delta -= 360.0f;
    }
    while(yaw_delta < -180.0f)
    {
        yaw_delta += 360.0f;
    }
    return angle_deg + yaw_delta;
}

/**
 * @brief 将世界相对向量转换为车体系单位速度方向。
 * @param dx_m 车灯到信标的世界 X 相对量，单位 m。
 * @param dy_m 车灯到信标的世界 Y 相对量，单位 m。
 * @param lamp_angle_deg 车灯横轴在世界坐标系中的无向角，单位 deg。
 * @param target_strafe 输出车体右向单位分量，不得为空。
 * @param target_forward 输出车体前向单位分量，不得为空。
 * @param distance_m 输出车灯到信标距离，单位 m；不得为空。
 * @return 相对向量在有效距离内返回 1，否则返回 0。
 */
static uint8 CarPlan_4_GetDirection(float dx_m,
                                    float dy_m,
                                    float lamp_angle_deg,
                                    float *target_strafe,
                                    float *target_forward,
                                    float *distance_m);

/**
 * @brief 检查候选车灯长轴是否符合车模 yaw 传播后的无向角预测。
 * @param candidate_deg 候选车灯世界长轴角，单位 deg。
 * @return 角度创新不超过门限时返回 1，否则返回 0。
 */
static uint8 CarPlan_4_LampAngleConsistent(float candidate_deg);

/**
 * @brief 更新新近灯路线候选的连续确认，并返回是否可抢占当前目标。
 * @param new_camera_mask 本周期真实更新的相机位掩码。
 * @param current_distance_m 当前目标惯导预测距离，单位 m。
 * @param tick_now 当前系统时刻，单位 ms。
 * @param selected 输出本周期候选槽位，不得为空。
 * @return 确认可抢占返回 1；后方旧目标等待直行保持返回 2；否则返回 0。
 */
static uint8 CarPlan_4_UpdateRouteChallenge(uint8 new_camera_mask,
                                            float current_distance_m,
                                            uint32 tick_now,
                                            uint8 *selected)
{
    uint8 i;
    uint8 challenger = 0xFFU;
    uint8 current_behind = 0U;
    float dx;
    float dy;
    float target_strafe;
    float target_forward;
    float distance;
    float best_distance = 1000.0f;
    float current_turn_deg = 180.0f;

    if(CarPlan_4_GetDirection(s_car_plan_4_track_dx_m,
                              s_car_plan_4_track_dy_m,
                              CarPlan_4_PredictLampAngle(
                                  s_car_plan_4_track_lamp_angle_deg,
                                  s_car_plan_4_track_car_yaw_deg),
                              &target_strafe,
                              &target_forward,
                              &distance) != 0U)
    {
        current_behind = (target_forward < 0.0f) ? 1U : 0U;
        current_turn_deg = atan2f(fabsf(target_strafe), target_forward) *
                           CAR_PLAN_4_RAD_TO_DEG;
    }

    /* 路线层只考虑稳定前向且明显更近的候选；旧目标在车后时直接选前向最近。 */
    for(i = 0U; i < s_car_plan_4_camera.beacon_count; i++)
    {
        const three_camera_beacon_t *beacon = &s_car_plan_4_camera.beacon[i];
        uint8 strong_advantage;
        float candidate_turn_deg;

        if((beacon->valid == 0U) || (beacon->pair_valid == 0U) ||
           ((beacon->camera_mask & new_camera_mask) == 0U) ||
           (CarPlan_4_LampAngleConsistent(
                beacon->pair_lamp_angle_deg) == 0U) ||
           (CarPlan_4_GetDirection(beacon->pair_dx_m,
                                   beacon->pair_dy_m,
                                   beacon->pair_lamp_angle_deg,
                                   &target_strafe,
                                   &target_forward,
                                   &distance) == 0U) ||
           (target_forward < CAR_PLAN_4_SEARCH_FORWARD_MIN) ||
           (distance < CAR_PLAN_4_COAST_STOP_DISTANCE_M))
        {
            continue;
        }
        strong_advantage =
            (distance <= current_distance_m * CAR_PLAN_4_ROUTE_STRONG_RATIO)
                ? 1U
                : 0U;
        candidate_turn_deg = atan2f(fabsf(target_strafe), target_forward) *
                             CAR_PLAN_4_RAD_TO_DEG;
        if(((current_behind == 0U) && (strong_advantage == 0U) &&
            (distance + CAR_PLAN_4_ROUTE_NEARER_MARGIN_M >
             current_distance_m) &&
            (distance > current_distance_m * CAR_PLAN_4_ROUTE_NEARER_RATIO)) ||
           ((strong_advantage == 0U) &&
            (candidate_turn_deg > current_turn_deg +
                                      CAR_PLAN_4_ROUTE_TURN_MARGIN_DEG)))
        {
            continue;
        }
        if(distance < best_distance)
        {
            challenger = i;
            best_distance = distance;
        }
    }
    if(challenger == 0xFFU)
    {
        s_car_plan_4_route_challenge_count = 0U;
        s_car_plan_4_route_challenge_hold = 0U;
        *selected = 0xFFU;
        return 0U;
    }
    dx = s_car_plan_4_camera.beacon[challenger].pair_dx_m;
    dy = s_car_plan_4_camera.beacon[challenger].pair_dy_m;
    s_car_plan_4_route_challenge_count =
        ((s_car_plan_4_route_challenge_count != 0U) &&
         ((tick_now - s_car_plan_4_route_challenge_tick) <
          CAR_PLAN_4_IMAGE_TIMEOUT_MS) &&
         (((dx - s_car_plan_4_route_challenge_dx_m) *
           (dx - s_car_plan_4_route_challenge_dx_m) +
           (dy - s_car_plan_4_route_challenge_dy_m) *
           (dy - s_car_plan_4_route_challenge_dy_m)) <=
          CAR_PLAN_4_REACQUIRE_DIST_M * CAR_PLAN_4_REACQUIRE_DIST_M))
            ? CAR_PLAN_4_ROUTE_CONFIRM_TICKS
            : 1U;
    s_car_plan_4_route_challenge_dx_m = dx;
    s_car_plan_4_route_challenge_dy_m = dy;
    s_car_plan_4_route_challenge_tick = tick_now;
    s_car_plan_4_route_challenge_hold =
        ((current_behind != 0U) &&
         (s_car_plan_4_route_challenge_count <
          CAR_PLAN_4_ROUTE_CONFIRM_TICKS)) ? 1U : 0U;
    *selected = challenger;
    if(s_car_plan_4_route_challenge_count >= CAR_PLAN_4_ROUTE_CONFIRM_TICKS)
    {
        return 1U;
    }
    return (current_behind != 0U) ? CAR_PLAN_4_ROUTE_PENDING_HOLD : 0U;
}

/**
 * @brief 检查候选车灯长轴是否符合车模 yaw 传播后的无向角预测。
 * @param candidate_deg 候选车灯世界长轴角，单位 deg。
 * @return 角度创新不超过门限时返回 1，否则返回 0。
 */
static uint8 CarPlan_4_LampAngleConsistent(float candidate_deg)
{
    float reference_angle = (s_car_plan_4_state == CAR_PLAN_4_STATE_SEARCH)
                                ? s_car_plan_4_pending_lamp_angle_deg
                                : s_car_plan_4_track_lamp_angle_deg;
    float reference_yaw = (s_car_plan_4_state == CAR_PLAN_4_STATE_SEARCH)
                              ? s_car_plan_4_pending_car_yaw_deg
                              : s_car_plan_4_track_car_yaw_deg;
    float error = candidate_deg -
                  CarPlan_4_PredictLampAngle(reference_angle, reference_yaw);

    while(error > 90.0f)
    {
        error -= 180.0f;
    }
    while(error < -90.0f)
    {
        error += 180.0f;
    }
    return (fabsf(error) <= CAR_PLAN_4_LAMP_ANGLE_GATE_DEG) ? 1U : 0U;
}

/**
 * @brief 在本次真实新图像中选择满足距离和连续性约束的候选信标。
 * @param new_camera_mask 本周期真实更新的相机位掩码。
 * @param use_reference 非零时按参考世界相对向量选择，否则按车灯距离选择。
 * @param reference_x_m 参考世界 X 相对量，单位 m。
 * @param reference_y_m 参考世界 Y 相对量，单位 m。
 * @param gate_m 使用参考向量时的最大创新距离，单位 m。
 * @param selected 输出候选槽位，不得为空。
 * @param best_innovation_m 输出最佳候选创新距离，单位 m；不得为空。
 * @param second_innovation_m 输出次佳候选创新距离，单位 m；不得为空。
 * @return 找到可信候选返回 1，否则返回 0。
 */
static uint8 CarPlan_4_SelectCandidate(uint8 new_camera_mask,
                                       uint8 use_reference,
                                       float reference_x_m,
                                       float reference_y_m,
                                       float gate_m,
                                       uint8 *selected,
                                       float *best_innovation_m,
                                       float *second_innovation_m)
{
    uint8 i;
    uint8 best_same_camera = 0U;
    uint8 second_available = 0U;
    float best_score_sq = 0.0f;
    float second_score_sq = 0.0f;
    float reference_sq = reference_x_m * reference_x_m +
                         reference_y_m * reference_y_m;
    float gate_sq = gate_m * gate_m;

    *selected = 0xFFU;
    *best_innovation_m = 1000.0f;
    *second_innovation_m = 1000.0f;

    for(i = 0U; i < s_car_plan_4_camera.beacon_count; i++)
    {
        const three_camera_beacon_t *beacon = &s_car_plan_4_camera.beacon[i];
        uint8 same_camera = (beacon->pair_same_camera != 0U) ? 1U : 0U;
        float dx = beacon->pair_dx_m;
        float dy = beacon->pair_dy_m;
        float distance_sq;
        float score_sq;

        if((beacon->valid == 0U) || (beacon->pair_valid == 0U) ||
           ((beacon->camera_mask & new_camera_mask) == 0U) ||
           (isfinite(dx) == 0) || (isfinite(dy) == 0) ||
           (isfinite(beacon->pair_lamp_angle_deg) == 0))
        {
            continue;
        }
        distance_sq = dx * dx + dy * dy;
        if((distance_sq < CAR_PLAN_4_MIN_DISTANCE_M * CAR_PLAN_4_MIN_DISTANCE_M) ||
           (distance_sq > CAR_PLAN_4_MAX_DISTANCE_M * CAR_PLAN_4_MAX_DISTANCE_M))
        {
            continue;
        }
        if(use_reference != 0U)
        {
            float direction_dot = dx * reference_x_m + dy * reference_y_m;

            if(CarPlan_4_LampAngleConsistent(beacon->pair_lamp_angle_deg) == 0U)
            {
                continue;
            }
            if((direction_dot <= 0.0f) ||
               (direction_dot * direction_dot <
                CAR_PLAN_4_DIRECTION_COS_SQ * distance_sq * reference_sq))
            {
                continue;
            }
            dx -= reference_x_m;
            dy -= reference_y_m;
            score_sq = dx * dx + dy * dy;
            if(score_sq > gate_sq)
            {
                continue;
            }
        }
        else
        {
            score_sq = distance_sq;
        }
        if((use_reference == 0U) && (*selected != 0xFFU) &&
           (best_same_camera != same_camera))
        {
            if(best_same_camera != 0U)
            {
                continue;
            }
            *selected = 0xFFU;
            second_available = 0U;
        }
        if((*selected == 0xFFU) || (score_sq < best_score_sq))
        {
            if(*selected != 0xFFU)
            {
                second_score_sq = best_score_sq;
                second_available = 1U;
            }
            best_score_sq = score_sq;
            *selected = i;
            best_same_camera = same_camera;
        }
        else if((second_available == 0U) || (score_sq < second_score_sq))
        {
            second_score_sq = score_sq;
            second_available = 1U;
        }
    }

    if(*selected == 0xFFU)
    {
        return 0U;
    }
    if(use_reference != 0U)
    {
        *best_innovation_m = sqrtf(best_score_sq);
        if(second_available != 0U)
        {
            *second_innovation_m = sqrtf(second_score_sq);
        }
    }
    return 1U;
}

/**
 * @brief 将世界相对向量转换为车体系单位速度方向。
 * @param dx_m 车灯到信标的世界 X 相对量，单位 m。
 * @param dy_m 车灯到信标的世界 Y 相对量，单位 m。
 * @param lamp_angle_deg 车灯横轴在世界坐标系中的无向角，单位 deg。
 * @param target_strafe 输出车体右向单位分量，不得为空。
 * @param target_forward 输出车体前向单位分量，不得为空。
 * @param distance_m 输出车灯到信标距离，单位 m；不得为空。
 * @return 相对向量在有效距离内返回 1，否则返回 0。
 */
static uint8 CarPlan_4_GetDirection(float dx_m,
                                    float dy_m,
                                    float lamp_angle_deg,
                                    float *target_strafe,
                                    float *target_forward,
                                    float *distance_m)
{
    float angle_rad;
    float right_x;
    float right_y;

    if((isfinite(dx_m) == 0) || (isfinite(dy_m) == 0) ||
       (isfinite(lamp_angle_deg) == 0) || (isfinite(g_car_yaw) == 0))
    {
        return 0U;
    }
    *distance_m = sqrtf(dx_m * dx_m + dy_m * dy_m);
    if((*distance_m < CAR_PLAN_4_MIN_DISTANCE_M) ||
       (*distance_m > CAR_PLAN_4_MAX_DISTANCE_M))
    {
        return 0U;
    }
    angle_rad = lamp_angle_deg * CAR_PLAN_4_DEG_TO_RAD;
    right_x = cosf(angle_rad);
    right_y = sinf(angle_rad);
    if(cosf((lamp_angle_deg - g_car_yaw - 90.0f) *
            CAR_PLAN_4_DEG_TO_RAD) < 0.0f)
    {
        right_x = -right_x;
        right_y = -right_y;
    }
    *target_strafe = (dx_m * right_x + dy_m * right_y) / *distance_m;
    *target_forward = (dx_m * right_y - dy_m * right_x) / *distance_m;
    if((isfinite(*target_strafe) == 0) || (isfinite(*target_forward) == 0))
    {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 接纳一个通过连续性检查的图像候选并更新规划输出。
 * @param selected 三摄融合信标槽位，范围 0 至 beacon_count-1。
 * @param tick_now 当前系统时刻，单位 ms。
 * @param route_switch 非零表示候选已经通过路线抢占的连续确认。
 * @return 候选方向有效时返回 1，否则返回 0。
 */
static uint8 CarPlan_4_AcceptCandidate(uint8 selected,
                                       uint32 tick_now,
                                       uint8 route_switch)
{
    const three_camera_beacon_t *beacon = &s_car_plan_4_camera.beacon[selected];
    float target_strafe;
    float target_forward;
    float distance;
    float plan_speed;

    if(CarPlan_4_GetDirection(beacon->pair_dx_m,
                              beacon->pair_dy_m,
                              beacon->pair_lamp_angle_deg,
                              &target_strafe,
                              &target_forward,
                              &distance) == 0U)
    {
        return 0U;
    }
    if(s_car_plan_4_state != CAR_PLAN_4_STATE_SEARCH)
    {
        float predicted_strafe;
        float predicted_forward;
        float predicted_distance;

        if(CarPlan_4_GetDirection(s_car_plan_4_track_dx_m,
                                  s_car_plan_4_track_dy_m,
                                  CarPlan_4_PredictLampAngle(
                                      s_car_plan_4_track_lamp_angle_deg,
                                      s_car_plan_4_track_car_yaw_deg),
                                  &predicted_strafe,
                                  &predicted_forward,
                                  &predicted_distance) == 0U)
        {
            return 0U;
        }
        if(((target_strafe * predicted_strafe +
             target_forward * predicted_forward) < CAR_PLAN_4_OUTPUT_COS_MIN) &&
           ((route_switch == 0U) ||
            ((predicted_forward >= 0.0f) &&
             (distance > predicted_distance * CAR_PLAN_4_ROUTE_STRONG_RATIO))))
        {
            return 0U;
        }
    }
    plan_speed = CarPlan_4_SpeedPlanUpdate(1U,
                                  distance,
                                  atan2f(fabsf(target_strafe), target_forward) *
                                      CAR_PLAN_4_RAD_TO_DEG,
                                  CarPlan_4_CheckDualLineFast(selected));
    s_car_plan_4_track_dx_m = beacon->pair_dx_m;
    s_car_plan_4_track_dy_m = beacon->pair_dy_m;
    s_car_plan_4_track_lamp_angle_deg = beacon->pair_lamp_angle_deg;
    s_car_plan_4_track_car_yaw_deg = g_car_yaw;
    s_car_plan_4_track_speed_mps = plan_speed;
    s_car_plan_4_track_camera_mask = beacon->camera_mask;
    s_car_plan_4_last_edge_margin_px =
        CarPlan_4_GetEdgeMarginPx(s_car_plan_4_track_camera_mask);
    s_car_plan_4_last_accept_tick = tick_now;
    s_car_plan_4_last_camera_tick = tick_now;
    s_car_plan_4_state = CAR_PLAN_4_STATE_TRACK;
    s_car_plan_4_coast_level = CAR_PLAN_4_COAST_LEVEL_NEAR;
    s_car_plan_4_confirm_count = 0U;
    s_car_plan_4_reacquire_count = 0U;
    s_car_plan_4_reacquire_age = 0U;
    s_car_plan_4_no_vision_ticks = 0U;
    s_car_plan_4_result.valid = 1U;
    s_car_plan_4_result.target_strafe_mps = target_strafe * plan_speed;
    s_car_plan_4_result.target_forward_mps = target_forward * plan_speed;
    s_car_plan_4_selected = (int8)selected;
    return 1U;
}

/**
 * @brief 使用当前惯导相对向量生成保持或平滑退出阶段的速度输出。
 * @param speed_scale 最近可信速度的缩放比例，范围 0-1。
 * @return 预测方向有效且缩放比例大于零时返回 1，否则返回 0。
 */
static uint8 CarPlan_4_OutputPrediction(float speed_scale)
{
    float target_strafe;
    float target_forward;
    float distance;

    distance = sqrtf(s_car_plan_4_track_dx_m * s_car_plan_4_track_dx_m +
                     s_car_plan_4_track_dy_m * s_car_plan_4_track_dy_m);
    if(distance <= CAR_PLAN_4_COAST_STOP_DISTANCE_M)
    {
        return 0U;
    }
    if((speed_scale <= 0.0f) ||
       (CarPlan_4_GetDirection(s_car_plan_4_track_dx_m,
                               s_car_plan_4_track_dy_m,
                               CarPlan_4_PredictLampAngle(
                                   s_car_plan_4_track_lamp_angle_deg,
                                   s_car_plan_4_track_car_yaw_deg),
                               &target_strafe,
                               &target_forward,
                               &distance) == 0U))
    {
        return 0U;
    }
    s_car_plan_4_result.valid = 1U;
    s_car_plan_4_result.target_strafe_mps =
        target_strafe * s_car_plan_4_track_speed_mps * speed_scale;
    s_car_plan_4_result.target_forward_mps =
        target_forward * s_car_plan_4_track_speed_mps * speed_scale;
    s_car_plan_4_selected = -1;
    return 1U;
}

static void CarPlan_4_ClearResult(void)
{
    s_car_plan_4_result.valid = 0U;
    s_car_plan_4_result.target_strafe_mps = 0.0f;
    s_car_plan_4_result.target_forward_mps = 0.0f;
    s_car_plan_4_selected = -1;
}

/**
 * @brief 放弃当前目标并回到搜索状态，同时清空速度输出。
 * @param 无。
 * @return 无。
 */
static void CarPlan_4_DropTarget(void)
{
    s_car_plan_4_state = CAR_PLAN_4_STATE_SEARCH;
    s_car_plan_4_confirm_count = 0U;
    s_car_plan_4_reacquire_count = 0U;
    s_car_plan_4_reacquire_age = 0U;
    s_car_plan_4_no_vision_ticks = 0U;
    s_car_plan_4_coast_level = CAR_PLAN_4_COAST_LEVEL_NEAR;
    s_car_plan_4_track_camera_mask = 0U;
    s_car_plan_4_route_challenge_count = 0U;
    s_car_plan_4_route_challenge_hold = 0U;
    s_car_plan_4_route_challenge_tick = 0U;
    s_car_plan_4_last_accept_tick = 0U;
    s_car_plan_4_last_camera_tick = 0U;
    (void)CarPlan_4_SpeedPlanUpdate(0U, 0.0f, 0.0f, 0U);
    CarPlan_4_ClearResult();
}

void CarPlan_4_Reset(void)
{
    uint8 camera;
    uint8 i;

    CarPlan_4_ClearResult();
    CarPlan_4_SpeedPlanReset();
    s_car_plan_4_state = CAR_PLAN_4_STATE_SEARCH;
    s_car_plan_4_confirm_count = 0U;
    s_car_plan_4_reacquire_count = 0U;
    s_car_plan_4_reacquire_age = 0U;
    s_car_plan_4_no_vision_ticks = 0U;
    s_car_plan_4_coast_level = CAR_PLAN_4_COAST_LEVEL_NEAR;
    s_car_plan_4_track_camera_mask = 0U;
    s_car_plan_4_last_update_tick = tick_1000us_cnt;
    s_car_plan_4_last_accept_tick = 0U;
    s_car_plan_4_last_camera_tick = 0U;
    s_car_plan_4_track_dx_m = 0.0f;
    s_car_plan_4_track_dy_m = 0.0f;
    s_car_plan_4_track_lamp_angle_deg = 0.0f;
    s_car_plan_4_track_car_yaw_deg = 0.0f;
    s_car_plan_4_track_speed_mps = 0.0f;
    s_car_plan_4_pending_dx_m = 0.0f;
    s_car_plan_4_pending_dy_m = 0.0f;
    s_car_plan_4_pending_lamp_angle_deg = 0.0f;
    s_car_plan_4_pending_car_yaw_deg = 0.0f;
    s_car_plan_4_route_challenge_count = 0U;
    s_car_plan_4_route_challenge_hold = 0U;
    s_car_plan_4_route_challenge_tick = 0U;
    s_car_plan_4_last_edge_margin_px = 1000.0f;
    s_car_plan_4_camera.car_lamp.valid = 0U;
    s_car_plan_4_camera.beacon_count = 0U;
    for(i = 0U; i < THREE_CAMERA_MAX_BEACON_COUNT; i++)
    {
        s_car_plan_4_camera.beacon[i].valid = 0U;
    }
    for(camera = 0U; camera < (uint8)IMAGE_CAMERA_COUNT; camera++)
    {
        s_car_plan_4_camera_seq[camera] = 0U;
        s_car_plan_4_filter_camera_seq[camera] = 0U;
        image_data_clear(&s_car_plan_4_filtered[camera]);
        for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
        {
            s_car_plan_4_track[camera][i].valid = 0U;
            s_car_plan_4_track[camera][i].gap_ticks = 0U;
            s_car_plan_4_track[camera][i].sample_ticks = 0U;
            s_car_plan_4_track[camera][i].far_age_ticks =
                CAR_PLAN_4_HISTORY_TICKS + 1U;
            s_car_plan_4_track[camera][i].suspect_age_ticks =
                CAR_PLAN_4_HISTORY_TICKS + 1U;
            s_car_plan_4_track[camera][i].x = 0.0f;
            s_car_plan_4_track[camera][i].y = 0.0f;
        }
    }
}

/**
 * @brief 使用三摄世界相对向量锁定信标，并在边缘姿态丢失时短时惯导保持车速。
 * @param result 输出车体系横向和前向目标速度；允许传入空指针。
 * @return TRACK 或 COAST 仍有有效目标速度时返回 1，否则返回 0。
 */
uint8 CarPlan_4_Update(car_plan_result_t *result)
{
    struct image_data filtered[IMAGE_CAMERA_COUNT];
    uint32 tick_now = tick_1000us_cnt;
    uint32 delta_ms = tick_now - s_car_plan_4_last_update_tick;
    uint32 lost_age_ms;
    uint8 new_camera_mask;
    uint8 camera_valid;
    uint8 selected = 0xFFU;
    uint8 route_selected = 0xFFU;
    uint8 candidate_valid = 0U;
    uint8 route_ready = 0U;
    float dt_s;
    float car_speed;
    float innovation_gate;
    float best_innovation;
    float second_innovation;
    float attitude;
    float track_distance;

    if(delta_ms > 50U)
    {
        delta_ms = 50U;
    }
    dt_s = (float)delta_ms * 0.001f;
    s_car_plan_4_last_update_tick = tick_now;
    CarPlan_4_ClearResult();
    CarPlan_4_FilterNearLamp(filtered);
    new_camera_mask = CarPlan_4_GetNewCameraMask();
    camera_valid = Three_Camera_Update(filtered,
                                       g_euler.roll,
                                       g_euler.pitch,
                                       g_euler.yaw,
                                       g_tof_fused_height_mm,
                                       g_tof_fused_valid,
                                       &s_car_plan_4_camera);
    if(s_car_plan_4_camera.car_lamp.valid == 0U)
    {
        camera_valid = 0U;
    }
    if(CarPlan_4_CarDataFresh(CAR_PLAN_4_CAR_DATA_STOP_MS) == 0U)
    {
        CarPlan_4_DropTarget();
        if(result != 0)
        {
            *result = s_car_plan_4_result;
        }
        return 0U;
    }
    CarPlan_4_Propagate(dt_s);

    /* 搜索阶段只按真实新图像累计三次连续确认，防止缓存或单帧假点启动车模。 */
    if(s_car_plan_4_state == CAR_PLAN_4_STATE_SEARCH)
    {
        (void)CarPlan_4_SpeedPlanUpdate(0U, 0.0f, 0.0f, 0U);
        if(new_camera_mask != 0U)
        {
            if(camera_valid == 0U)
            {
                candidate_valid = 0U;
            }
            else if(s_car_plan_4_confirm_count == 0U)
            {
                candidate_valid = CarPlan_4_SelectCandidate(new_camera_mask,
                                                             0U,
                                                             0.0f,
                                                             0.0f,
                                                             0.0f,
                                                             &selected,
                                                             &best_innovation,
                                                             &second_innovation);
            }
            else
            {
                candidate_valid = CarPlan_4_SelectCandidate(new_camera_mask,
                                                             1U,
                                                             s_car_plan_4_pending_dx_m,
                                                             s_car_plan_4_pending_dy_m,
                                                             CAR_PLAN_4_REACQUIRE_DIST_M,
                                                             &selected,
                                                             &best_innovation,
                                                             &second_innovation);
                if((candidate_valid != 0U) &&
                   ((second_innovation - best_innovation) <
                    CAR_PLAN_4_AMBIGUITY_MARGIN_M))
                {
                    candidate_valid = 0U;
                }
            }
            if(candidate_valid != 0U)
            {
                s_car_plan_4_pending_dx_m =
                    s_car_plan_4_camera.beacon[selected].pair_dx_m;
                s_car_plan_4_pending_dy_m =
                    s_car_plan_4_camera.beacon[selected].pair_dy_m;
                s_car_plan_4_pending_lamp_angle_deg =
                    s_car_plan_4_camera.beacon[selected].pair_lamp_angle_deg;
                s_car_plan_4_pending_car_yaw_deg = g_car_yaw;
                s_car_plan_4_last_camera_tick = tick_now;
                if(s_car_plan_4_confirm_count < 0xFFU)
                {
                    s_car_plan_4_confirm_count++;
                }
                if(s_car_plan_4_confirm_count >= CAR_PLAN_4_CONFIRM_TICKS)
                {
                    if(CarPlan_4_AcceptCandidate(selected,
                                                 tick_now,
                                                 0U) == 0U)
                    {
                        s_car_plan_4_confirm_count = 0U;
                    }
                }
            }
            else
            {
                s_car_plan_4_confirm_count = 0U;
            }
        }
        else if((s_car_plan_4_confirm_count != 0U) &&
                ((tick_now - s_car_plan_4_last_camera_tick) >=
                 CAR_PLAN_4_IMAGE_TIMEOUT_MS))
        {
            s_car_plan_4_confirm_count = 0U;
        }
        if(result != 0)
        {
            *result = s_car_plan_4_result;
        }
        return s_car_plan_4_result.valid;
    }

    lost_age_ms = tick_now - s_car_plan_4_last_accept_tick;
    car_speed = sqrtf(g_car_vel_x * g_car_vel_x + g_car_vel_y * g_car_vel_y);
    innovation_gate = CAR_PLAN_4_INNOVATION_BASE_M +
                      CAR_PLAN_4_INNOVATION_GAIN_S * car_speed *
                          ((float)lost_age_ms * 0.001f);
    if(innovation_gate > CAR_PLAN_4_INNOVATION_MAX_M)
    {
        innovation_gate = CAR_PLAN_4_INNOVATION_MAX_M;
    }
    track_distance = sqrtf(s_car_plan_4_track_dx_m * s_car_plan_4_track_dx_m +
                           s_car_plan_4_track_dy_m * s_car_plan_4_track_dy_m);
    if((new_camera_mask != 0U) && (camera_valid != 0U))
    {
        route_ready = CarPlan_4_UpdateRouteChallenge(new_camera_mask,
                                                      track_distance,
                                                      tick_now,
                                                      &route_selected);
    }
    if((route_ready == 0U) && (s_car_plan_4_route_challenge_hold != 0U))
    {
        if((tick_now - s_car_plan_4_route_challenge_tick) <
           CAR_PLAN_4_IMAGE_TIMEOUT_MS)
        {
            route_ready = CAR_PLAN_4_ROUTE_PENDING_HOLD;
        }
        else
        {
            s_car_plan_4_route_challenge_count = 0U;
            s_car_plan_4_route_challenge_hold = 0U;
        }
    }
    if((new_camera_mask != 0U) && (camera_valid != 0U))
    {
        candidate_valid = CarPlan_4_SelectCandidate(new_camera_mask,
                                                     1U,
                                                     s_car_plan_4_track_dx_m,
                                                     s_car_plan_4_track_dy_m,
                                                     innovation_gate,
                                                     &selected,
                                                     &best_innovation,
                                                     &second_innovation);
        if((candidate_valid != 0U) &&
           ((second_innovation - best_innovation) <
            CAR_PLAN_4_AMBIGUITY_MARGIN_M))
        {
            candidate_valid = 0U;
        }
    }
    if(route_ready == CAR_PLAN_4_ROUTE_PENDING_HOLD)
    {
        /* 前向候选尚未二次确认时只保持直行，不让后方旧目标触发反向转向。 */
        s_car_plan_4_result.valid = 1U;
        s_car_plan_4_result.target_strafe_mps = 0.0f;
        s_car_plan_4_result.target_forward_mps = s_car_plan_4_track_speed_mps;
        if(result != 0)
        {
            *result = s_car_plan_4_result;
        }
        return 1U;
    }
    if((route_ready == 1U) && (route_selected != 0xFFU))
    {
        route_ready = CarPlan_4_AcceptCandidate(route_selected, tick_now, 1U);
        s_car_plan_4_route_challenge_count = 0U;
        s_car_plan_4_route_challenge_hold = 0U;
        if(route_ready != 0U)
        {
            if(result != 0)
            {
                *result = s_car_plan_4_result;
            }
            return 1U;
        }
    }

    /* TRACK 中直接接纳连续观测；单次坏观测只短时使用预测，不立即改变路径。 */
    if(s_car_plan_4_state == CAR_PLAN_4_STATE_TRACK)
    {
        if(candidate_valid != 0U)
        {
            if(CarPlan_4_AcceptCandidate(selected, tick_now, 0U) != 0U)
            {
                if(result != 0)
                {
                    *result = s_car_plan_4_result;
                }
                return 1U;
            }
        }
        if((new_camera_mask & s_car_plan_4_track_camera_mask) != 0U)
        {
            s_car_plan_4_last_camera_tick = tick_now;
            if(s_car_plan_4_no_vision_ticks < 0xFFU)
            {
                s_car_plan_4_no_vision_ticks++;
            }
        }
        else if((tick_now - s_car_plan_4_last_camera_tick) >=
                CAR_PLAN_4_IMAGE_TIMEOUT_MS)
        {
            s_car_plan_4_no_vision_ticks = CAR_PLAN_4_NO_VISION_TICKS;
        }
        if(s_car_plan_4_no_vision_ticks < CAR_PLAN_4_NO_VISION_TICKS)
        {
            if(CarPlan_4_OutputPrediction(1.0f) == 0U)
            {
                CarPlan_4_DropTarget();
            }
            if(result != 0)
            {
                *result = s_car_plan_4_result;
            }
            return s_car_plan_4_result.valid;
        }
        if((new_camera_mask & s_car_plan_4_track_camera_mask) != 0U)
        {
            /* 目标相机有新图却无匹配候选：目标已灭或消失，立即放弃，不再惯导盲航。 */
            CarPlan_4_DropTarget();
            if(result != 0)
            {
                *result = s_car_plan_4_result;
            }
            return s_car_plan_4_result.valid;
        }

        attitude = sqrtf(g_euler.roll * g_euler.roll +
                         g_euler.pitch * g_euler.pitch);
        track_distance = sqrtf(s_car_plan_4_track_dx_m * s_car_plan_4_track_dx_m +
                               s_car_plan_4_track_dy_m * s_car_plan_4_track_dy_m);
        if((track_distance >= CAR_PLAN_4_FAR_DISTANCE_M) &&
           ((s_car_plan_4_last_edge_margin_px <= CAR_PLAN_4_EDGE_MARGIN_PX) ||
            (attitude >= CAR_PLAN_4_ATTITUDE_GATE_DEG)) &&
           (CarPlan_4_CarDataFresh(CAR_PLAN_4_CAR_DATA_WARN_MS) != 0U))
        {
            s_car_plan_4_state = CAR_PLAN_4_STATE_COAST;
            s_car_plan_4_coast_level =
                (track_distance >= CAR_PLAN_4_AGGRESSIVE_DISTANCE_M)
                    ? CAR_PLAN_4_COAST_LEVEL_AGGRESSIVE
                    : CAR_PLAN_4_COAST_LEVEL_FAR;
            s_car_plan_4_reacquire_count = 0U;
            s_car_plan_4_reacquire_age = 0U;
            (void)CarPlan_4_SpeedPlanUpdate(0U, 0.0f, 0.0f, 0U);
        }
        else if((s_car_plan_4_last_edge_margin_px <= CAR_PLAN_4_EDGE_MARGIN_PX) &&
                (attitude >= CAR_PLAN_4_ATTITUDE_GATE_DEG) &&
                (CarPlan_4_CarDataFresh(CAR_PLAN_4_CAR_DATA_WARN_MS) != 0U))
        {
            s_car_plan_4_state = CAR_PLAN_4_STATE_COAST;
            s_car_plan_4_coast_level = CAR_PLAN_4_COAST_LEVEL_NEAR;
            s_car_plan_4_reacquire_count = 0U;
            s_car_plan_4_reacquire_age = 0U;
            (void)CarPlan_4_SpeedPlanUpdate(0U, 0.0f, 0.0f, 0U);
        }
        else
        {
            CarPlan_4_DropTarget();
            if(result != 0)
            {
                *result = s_car_plan_4_result;
            }
            return 0U;
        }
    }

    /* COAST 只接纳创新门内且三帧窗口至少命中两次的重现目标。 */
    lost_age_ms = tick_now - s_car_plan_4_last_accept_tick;
    if(((s_car_plan_4_coast_level == CAR_PLAN_4_COAST_LEVEL_AGGRESSIVE) &&
        (lost_age_ms >= CAR_PLAN_4_AGGRESSIVE_COAST_MAX_MS)) ||
       ((s_car_plan_4_coast_level == CAR_PLAN_4_COAST_LEVEL_FAR) &&
        (lost_age_ms >= CAR_PLAN_4_FAR_COAST_MAX_MS)) ||
       ((s_car_plan_4_coast_level == CAR_PLAN_4_COAST_LEVEL_NEAR) &&
        (lost_age_ms >= CAR_PLAN_4_COAST_MAX_MS)))
    {
        CarPlan_4_DropTarget();
    }
    else
    {
        if((new_camera_mask != 0U) && (candidate_valid != 0U))
        {
            if(s_car_plan_4_reacquire_count == 0U)
            {
                s_car_plan_4_reacquire_count = 1U;
                s_car_plan_4_reacquire_age = 1U;
            }
            else
            {
                float pending_dx =
                    s_car_plan_4_camera.beacon[selected].pair_dx_m -
                    s_car_plan_4_pending_dx_m;
                float pending_dy =
                    s_car_plan_4_camera.beacon[selected].pair_dy_m -
                    s_car_plan_4_pending_dy_m;

                if((pending_dx * pending_dx + pending_dy * pending_dy) <=
                   CAR_PLAN_4_REACQUIRE_DIST_M * CAR_PLAN_4_REACQUIRE_DIST_M)
                {
                    s_car_plan_4_reacquire_count++;
                    s_car_plan_4_reacquire_age++;
                }
                else
                {
                    s_car_plan_4_reacquire_count = 1U;
                    s_car_plan_4_reacquire_age = 1U;
                }
            }
            s_car_plan_4_pending_dx_m =
                s_car_plan_4_camera.beacon[selected].pair_dx_m;
            s_car_plan_4_pending_dy_m =
                s_car_plan_4_camera.beacon[selected].pair_dy_m;
            s_car_plan_4_last_camera_tick = tick_now;
            if(s_car_plan_4_reacquire_count >= 2U)
            {
                (void)CarPlan_4_AcceptCandidate(selected, tick_now, 0U);
            }
        }
        else if(s_car_plan_4_reacquire_count != 0U)
        {
            if(new_camera_mask != 0U)
            {
                s_car_plan_4_reacquire_age++;
            }
            if((s_car_plan_4_reacquire_age >= 3U) ||
               ((tick_now - s_car_plan_4_last_camera_tick) >=
                CAR_PLAN_4_IMAGE_TIMEOUT_MS))
            {
                s_car_plan_4_reacquire_count = 0U;
                s_car_plan_4_reacquire_age = 0U;
            }
        }
        else if(new_camera_mask != 0U)
        {
            /* 视觉已恢复但重捕获窗口内无候选：目标已消失，停止盲航回到搜索。 */
            CarPlan_4_DropTarget();
            if(result != 0)
            {
                *result = s_car_plan_4_result;
            }
            return 0U;
        }

        if(s_car_plan_4_state != CAR_PLAN_4_STATE_TRACK)
        {
            s_car_plan_4_state = CAR_PLAN_4_STATE_COAST;
            if(CarPlan_4_OutputPrediction(1.0f) == 0U)
            {
                CarPlan_4_DropTarget();
            }
        }
    }

    if(result != 0)
    {
        *result = s_car_plan_4_result;
    }
    return s_car_plan_4_result.valid;
}

void CarPlan_4_GetResult(car_plan_result_t *result)
{
    if(result != 0)
    {
        *result = s_car_plan_4_result;
    }
}

void CarPlan_4_GetDebug(car_plan_4_debug_t *debug)
{
    uint8 i;
    uint8 output_count;
    int8 selected_output = s_car_plan_4_selected;

    if(debug == 0)
    {
        return;
    }
    for(i = 0U; i < CAR_PLAN_4_DEBUG_BEACON_COUNT; i++)
    {
        debug->beacon[i].valid = 0U;
        debug->beacon[i].camera_mask = 0U;
        debug->beacon[i].center_x = IMAGE_DATA_INVALID_VALUE;
        debug->beacon[i].center_y = IMAGE_DATA_INVALID_VALUE;
        debug->beacon[i].area = 0.0f;
    }
    debug->car_lamp.valid = s_car_plan_4_camera.car_lamp.valid;
    debug->car_lamp.camera_mask = s_car_plan_4_camera.car_lamp.camera_mask;
    debug->car_lamp.center_x = s_car_plan_4_camera.car_lamp.x_m;
    debug->car_lamp.center_y = s_car_plan_4_camera.car_lamp.y_m;
    debug->car_lamp.angle_deg = s_car_plan_4_camera.car_lamp.angle_deg;
    debug->selected_target_id = -1;

    output_count = s_car_plan_4_camera.beacon_count;
    if(output_count > CAR_PLAN_4_DEBUG_BEACON_COUNT)
    {
        output_count = CAR_PLAN_4_DEBUG_BEACON_COUNT;
    }
    for(i = 0U; i < output_count; i++)
    {
        debug->beacon[i].valid = s_car_plan_4_camera.beacon[i].valid;
        debug->beacon[i].camera_mask = s_car_plan_4_camera.beacon[i].camera_mask;
        debug->beacon[i].center_x = s_car_plan_4_camera.beacon[i].x_m;
        debug->beacon[i].center_y = s_car_plan_4_camera.beacon[i].y_m;
        debug->beacon[i].area = s_car_plan_4_camera.beacon[i].area;
    }
    if(s_car_plan_4_result.valid != 0U)
    {
        debug->selected_target_id = selected_output;
    }
}
