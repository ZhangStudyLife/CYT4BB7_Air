#include "yaw_align.h"
#include "fc_loop.h"
#include "fc_start_crsf.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../Image/image_data.h"
#include "../Planner/car_plan_3.h"
#include "../Protocols/crsf/crsf.h"
#include <math.h>

typedef struct
{
    uint8 valid;
    image_camera_e camera;
    float x;
    float y;
    float area;
} yaw_align_beacon_t;

static const float s_yaw_align_deadband_px = 3.0f;
static const float s_yaw_align_center_turn_x_px = 50.0f;
static const float s_yaw_align_x_to_deg = 0.5f;
static const float s_yaw_align_near_x_to_deg = 0.25f;
static const float s_yaw_align_near_y_px = 20.0f;
static const float s_yaw_align_max_delta_deg = 45.0f;
static const float s_yaw_align_near_max_delta_deg = 25.0f;
static const float s_yaw_align_jump_gate_px = 20.0f;
static const float s_yaw_align_yaw_to_x_px_per_deg = 1.0f;
static const float s_yaw_align_lamp_gate_y_px = 30.0f;
static const float s_yaw_align_lamp_gate_dist_px = 38.0f;
static const float s_yaw_align_lamp_gate_min_dx_px = 10.0f;
static const float s_yaw_align_lamp_gate_min_dy_px = -5.0f;
static const uint8 s_yaw_align_stable_frames = 5U;
static const uint8 s_yaw_align_lost_reset_frames = 20U;

#define YAW_ALIGN_SEARCH_HOLD_TICKS (75U) /* 每个搜索航向保持750ms，对应100Hz调用75次 */
#define YAW_ALIGN_SEARCH_STEP_DEG (45.0f) /* 相邻搜索目标的航向间隔，单位度 */
#define YAW_ALIGN_SEARCH_COMPLETE_DEG (360.0f) /* 单轮信标搜索完成角度，单位度 */
#define YAW_ALIGN_PLAN_VALID_TICKS (10U) /* CarPlan3结果连续有效100ms，对应100Hz调用10次 */
#define YAW_ALIGN_PLAN_INVALID_TICKS (15U) /* CarPlan3结果连续无效150ms后开始搜索 */

extern float g_car_yaw; /* 车端当前航向角，单位度。 */
extern float g_car_yaw_rate_dps; /* 车端当前航向角速度，单位度每秒。 */

static yaw_align_beacon_t s_locked_beacon;
static yaw_align_beacon_t s_candidate_beacon;
static float s_locked_yaw = 0.0f;
static float s_candidate_yaw = 0.0f;
static uint8 s_candidate_frames = 0U;
static uint8 s_locked = 0U;
static uint8 s_lost_frames = 0U;
static uint8 s_center_turn_active = 0U;
static float s_center_turn_target_yaw = 0.0f;
static yaw_align_beacon_t s_active_beacon;
static float s_yaw_delta_deg = 0.0f;
static uint8 s_action = YAW_ALIGN_ACTION_IDLE;
static uint8 s_control_mode = 0U; /* 上次选择的航向控制模式，范围0至2 */
static uint8 s_plan_valid = 0U; /* mode=2当前CarPlan3规划结果是否有效。 */
static uint8 s_plan_confirmed = 0U; /* 规划结果是否已连续有效100ms。 */
static uint8 s_plan_valid_ticks = 0U; /* 规划结果连续有效计数，单位10ms。 */
static uint8 s_plan_invalid_ticks = 0U; /* 规划结果连续无效计数，单位10ms。 */
static uint8 s_search_active = 0U; /* mode=2是否正在执行无信标旋转搜索。 */
static uint8 s_search_forced = 0U; /* 强制持续旋转搜索，供自动降落双旋转阶段使用。 */
static int8 s_search_direction = 1; /* mode=2搜索方向，1为yaw正方向，-1为yaw负方向。 */
static float s_search_target_yaw = 0.0f; /* mode=2当前搜索航向目标，单位度。 */
static float s_search_rotation_deg = 0.0f; /* 本轮无信标期间实际定向旋转角，单位度。 */
static float s_cable_twist_deg = 0.0f; /* 飞机相对车辆的累计线缆扭转角，单位度。 */
static float s_previous_air_yaw = 0.0f; /* 上次飞机航向角，单位度。 */
static float s_previous_car_yaw = 0.0f; /* 上次车辆航向角，单位度。 */
static uint8 s_yaw_history_valid = 0U; /* 飞机和车辆历史航向是否已经初始化。 */
static uint8 s_mode4_target_sent = 0U; /* 本次Mode4会话是否曾下发有效车目标速度。 */

static float YawAlign_Clamp(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}

static float YawAlign_DistanceSq(float x0, float y0, float x1, float y1)
{
    float dx = x0 - x1;
    float dy = y0 - y1;

    return dx * dx + dy * dy;
}

static float YawAlign_Wrap180Deg(float angle_deg)
{
    while(angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while(angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float YawAlign_RoundSearchGrid(float yaw_deg)
{
    return YawAlign_Wrap180Deg(
        floorf(yaw_deg / YAW_ALIGN_SEARCH_STEP_DEG + 0.5f) *
        YAW_ALIGN_SEARCH_STEP_DEG);
}

static float YawAlign_NextSearchGrid(float yaw_deg, int8 direction)
{
    float grid_index;

    if(direction > 0)
    {
        grid_index = floorf(yaw_deg / YAW_ALIGN_SEARCH_STEP_DEG + 0.001f) + 1.0f;
    }
    else
    {
        grid_index = ceilf(yaw_deg / YAW_ALIGN_SEARCH_STEP_DEG - 0.001f) - 1.0f;
    }

    return YawAlign_Wrap180Deg(grid_index * YAW_ALIGN_SEARCH_STEP_DEG);
}

static float YawAlign_CompensatedDistanceSq(const yaw_align_beacon_t *ref_beacon,
                                            float ref_yaw,
                                            float x,
                                            float y)
{
    float yaw_delta = YawAlign_Wrap180Deg(g_euler.yaw - ref_yaw);
    float predicted_x = ref_beacon->x -
                        yaw_delta * s_yaw_align_yaw_to_x_px_per_deg;

    return YawAlign_DistanceSq(x, y, predicted_x, ref_beacon->y);
}

static void YawAlign_HoldCurrentYaw(void)
{
    yaw_angle_target = g_euler.yaw;
    yaw_gyro_target = 0.0f;
    PID_Reset(&yaw_angle_pid);
    PID_Reset(&yaw_gyro_pid);
}

static uint8 YawAlign_IsNearCarLamp(image_camera_e camera, const beacon_data *beacon)
{
    uint8 i;

    if(beacon->y <= s_yaw_align_lamp_gate_y_px)
    {
        return 0U;
    }

    for(i = 0U; i < IMAGE_MAX_CAR_LAMP_COUNT; i++)
    {
        const car_lamp_data *lamp = &image_data[camera].car_lamp_data[i];
        float dx;
        float dy;

        if(lamp->valid == 0U)
        {
            continue;
        }

        dx = beacon->x - lamp->cx;
        dy = beacon->y - lamp->cy;

        if((fabsf(dx) > s_yaw_align_lamp_gate_min_dx_px) &&
           (dy > s_yaw_align_lamp_gate_min_dy_px) &&
           (YawAlign_DistanceSq(beacon->x, beacon->y,
                                lamp->cx, lamp->cy) <
            (s_yaw_align_lamp_gate_dist_px *
             s_yaw_align_lamp_gate_dist_px)))
        {
            return 1U;
        }
    }

    return 0U;
}

static uint8 YawAlign_FindLargestBeacon(yaw_align_beacon_t *out)
{
    image_camera_e cameras[2] = {Front, Back};
    uint8 found = 0U;
    uint8 camera_index;
    uint8 i;

    for(camera_index = 0U; camera_index < 2U; camera_index++)
    {
        image_camera_e camera = cameras[camera_index];

        for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
        {
            const beacon_data *beacon = &image_data[camera].beacon_data[i];

            if((beacon->valid == 0U) || (beacon->area <= 0.0f))
            {
                continue;
            }

            if(YawAlign_IsNearCarLamp(camera, beacon) != 0U)
            {
                continue;
            }

            if((found == 0U) || (beacon->area > out->area))
            {
                out->valid = 1U;
                out->camera = camera;
                out->x = beacon->x;
                out->y = beacon->y;
                out->area = beacon->area;
                found = 1U;
            }
        }
    }

    return found;
}

static uint8 YawAlign_FindLargestCenterBeacon(yaw_align_beacon_t *out)
{
    uint8 found = 0U;
    uint8 i;

    for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
    {
        const beacon_data *beacon = &image_data[Center].beacon_data[i];

        if((beacon->valid == 0U) || (beacon->area <= 0.0f))
        {
            continue;
        }

        if((found == 0U) || (beacon->area > out->area))
        {
            out->valid = 1U;
            out->camera = Center;
            out->x = beacon->x;
            out->y = beacon->y;
            out->area = beacon->area;
            found = 1U;
        }
    }

    return found;
}

static uint8 YawAlign_FindNearestLockedBeacon(yaw_align_beacon_t *out)
{
    uint8 found = 0U;
    uint8 i;
    float best_dist_sq = s_yaw_align_jump_gate_px * s_yaw_align_jump_gate_px;

    for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
    {
        const beacon_data *beacon = &image_data[s_locked_beacon.camera].beacon_data[i];
        float dist_sq;

        if((beacon->valid == 0U) || (beacon->area <= 0.0f))
        {
            continue;
        }

        dist_sq = YawAlign_CompensatedDistanceSq(&s_locked_beacon,
                                                 s_locked_yaw,
                                                 beacon->x,
                                                 beacon->y);
        if(dist_sq > best_dist_sq)
        {
            continue;
        }

        if((found == 0U) || (dist_sq < best_dist_sq))
        {
            out->valid = 1U;
            out->camera = s_locked_beacon.camera;
            out->x = beacon->x;
            out->y = beacon->y;
            out->area = beacon->area;
            best_dist_sq = dist_sq;
            found = 1U;
        }
    }

    return found;
}

static void YawAlign_UpdateCandidate(const yaw_align_beacon_t *beacon)
{
    float jump_gate_sq = s_yaw_align_jump_gate_px * s_yaw_align_jump_gate_px;

    if((s_candidate_frames == 0U) ||
       (s_candidate_beacon.camera != beacon->camera) ||
       (YawAlign_CompensatedDistanceSq(&s_candidate_beacon,
                                       s_candidate_yaw,
                                       beacon->x,
                                       beacon->y) > jump_gate_sq))
    {
        s_candidate_beacon = *beacon;
        s_candidate_yaw = g_euler.yaw;
        s_candidate_frames = 1U;
        return;
    }

    s_candidate_beacon = *beacon;
    s_candidate_yaw = g_euler.yaw;
    if(s_candidate_frames < s_yaw_align_stable_frames)
    {
        s_candidate_frames++;
    }

    if(s_candidate_frames >= s_yaw_align_stable_frames)
    {
        s_locked_beacon = s_candidate_beacon;
        s_locked_yaw = s_candidate_yaw;
        s_locked = 1U;
        s_lost_frames = 0U;
    }
}

/*
 * 函数功能: 清空航向对准、搜索和线缆扭转跟踪状态
 * 输入参数: 无
 * 输出参数或返回值: 无
 */
void YawAlign_Reset(void)
{
    s_locked_beacon.valid = 0U;
    s_candidate_beacon.valid = 0U;
    s_locked_yaw = 0.0f;
    s_candidate_yaw = 0.0f;
    s_candidate_frames = 0U;
    s_locked = 0U;
    s_lost_frames = 0U;
    s_center_turn_active = 0U;
    s_center_turn_target_yaw = 0.0f;
    s_active_beacon.valid = 0U;
    s_yaw_delta_deg = 0.0f;
    s_action = YAW_ALIGN_ACTION_IDLE;
    s_plan_valid = 0U;
    s_plan_confirmed = 0U;
    s_plan_valid_ticks = 0U;
    s_plan_invalid_ticks = 0U;
    s_search_active = 0U;
    s_search_direction = 1;
    s_search_target_yaw = 0.0f;
    s_search_rotation_deg = 0.0f;
    s_cable_twist_deg = 0.0f;
    s_previous_air_yaw = 0.0f;
    s_previous_car_yaw = 0.0f;
    s_yaw_history_valid = 0U;
    s_mode4_target_sent = 0U;
    s_search_forced = 0U;
    yaw_angle_pid.kp = g_fc_params.yaw_angle_kp;
}

static void YawAlign_FillDebugBeacon(const yaw_align_beacon_t *src,
                                     yaw_align_debug_beacon_t *dst)
{
    dst->valid = src->valid;
    dst->camera = (uint8)src->camera;
    dst->x = src->x;
    dst->y = src->y;
    dst->area = src->area;
}

/*
 * 函数功能: 获取航向对准和搜索状态的只读调试快照
 * 输入参数: out - 调试快照输出地址，不可为空
 * 输出参数或返回值: 无
 */
void YawAlign_GetDebug(yaw_align_debug_t *out)
{
    if(out == 0)
    {
        return;
    }

    out->locked = s_locked;
    out->candidate_frames = s_candidate_frames;
    out->lost_frames = s_lost_frames;
    out->action = s_action;
    out->beacon_visible = s_plan_confirmed;
    out->plan_valid = s_plan_valid;
    out->search_active = s_search_active;
    out->search_direction = s_search_direction;
    out->yaw_delta_deg = s_yaw_delta_deg;
    out->search_rotation_deg = s_search_rotation_deg;
    out->cable_twist_deg = s_cable_twist_deg;
    YawAlign_FillDebugBeacon(&s_active_beacon, &out->active_beacon);
    YawAlign_FillDebugBeacon(&s_locked_beacon, &out->locked_beacon);
    YawAlign_FillDebugBeacon(&s_candidate_beacon, &out->candidate_beacon);
}

static float YawAlign_GetYawDelta(const yaw_align_beacon_t *beacon)
{
    float x_to_deg = s_yaw_align_x_to_deg;
    float max_delta_deg = s_yaw_align_max_delta_deg;

    if(beacon->y > s_yaw_align_near_y_px)
    {
        x_to_deg = s_yaw_align_near_x_to_deg;
        max_delta_deg = s_yaw_align_near_max_delta_deg;
    }

    return YawAlign_Clamp(beacon->x * x_to_deg,
                          -max_delta_deg,
                          max_delta_deg);
}

/*
 * 函数功能: 执行现有的信标航向对准逻辑
 * 输入参数: 无
 * 输出参数或返回值: 1表示已锁定并使用有效信标，0表示尚未完成锁定
 */
static uint8 YawAlign_UpdateBeacon(void)
{
    yaw_align_beacon_t beacon;
    float yaw_delta;

    beacon.valid = 0U;
    s_active_beacon.valid = 0U;
    s_yaw_delta_deg = 0.0f;
    s_action = YAW_ALIGN_ACTION_IDLE;

    if(s_locked == 0U)
    {
        if(YawAlign_FindLargestBeacon(&beacon) != 0U)
        {
            s_active_beacon = beacon;
            s_action = YAW_ALIGN_ACTION_CANDIDATE;
            s_center_turn_active = 0U;
            s_center_turn_target_yaw = 0.0f;
            YawAlign_UpdateCandidate(&beacon);
            YawAlign_HoldCurrentYaw();
            return 0U;
        }

        if(s_center_turn_active == 0U)
        {
            if((YawAlign_FindLargestCenterBeacon(&beacon) == 0U) ||
               (fabsf(beacon.x) <= s_yaw_align_center_turn_x_px))
            {
                YawAlign_Reset();
                YawAlign_HoldCurrentYaw();
                return 0U;
            }

            YawAlign_Reset();
            s_center_turn_target_yaw = g_euler.yaw +
                                       ((beacon.x > 0.0f) ? 90.0f : -90.0f);
            s_center_turn_active = 1U;
            s_active_beacon = beacon;
        }

        s_action = YAW_ALIGN_ACTION_CENTER_TURN;
        s_yaw_delta_deg = s_center_turn_target_yaw - g_euler.yaw;
        yaw_angle_target = s_center_turn_target_yaw;
        return 0U;
    }

    if(YawAlign_FindNearestLockedBeacon(&beacon) == 0U)
    {
        s_active_beacon.valid = 0U;
        s_action = YAW_ALIGN_ACTION_LOST_HOLD;
        if(s_lost_frames < s_yaw_align_lost_reset_frames)
        {
            s_lost_frames++;
        }
        else
        {
            YawAlign_Reset();
        }

        YawAlign_HoldCurrentYaw();
        return 0U;
    }

    s_locked_beacon = beacon;
    s_active_beacon = beacon;
    s_locked_yaw = g_euler.yaw;
    s_lost_frames = 0U;

    if(fabsf(beacon.x) <= s_yaw_align_deadband_px)
    {
        s_action = YAW_ALIGN_ACTION_DEADBAND_HOLD;
        YawAlign_HoldCurrentYaw();
        return 1U;
    }

    yaw_delta = YawAlign_GetYawDelta(&beacon);
    s_action = YAW_ALIGN_ACTION_TRACK;
    s_yaw_delta_deg = yaw_delta;
    yaw_angle_target = g_euler.yaw + yaw_delta;
    return 1U;
}

/*
 * 函数功能: CarPlan3无有效目标时按45度整数网格循环搜索
 * 输入参数: aircraft_yaw_delta_deg - 本次飞机实际航向增量，单位度
 * 输出参数或返回值: 无
 */
static void YawAlign_UpdateSearch(float aircraft_yaw_delta_deg)
{
    car_plan_result_t plan_result;
    yaw_align_beacon_t beacon;

    CarPlan_3_GetResult(&plan_result);
    s_plan_valid = plan_result.valid;

    if((FC_START_CRSF_Get_Flight_Mode() == FC_START_CRSF_FLIGHT_MODE_4) &&
       (s_mode4_target_sent == 0U) &&
       (plan_result.valid != 0U))
    {
        s_mode4_target_sent = 1U;
    }

    /* Mode4刚进入且车尚未收到过目标速度时，等待信标系统启动，不旋转。 */
    if((FC_START_CRSF_Get_Flight_Mode() == FC_START_CRSF_FLIGHT_MODE_4) &&
       (s_mode4_target_sent == 0U) &&
       (s_search_forced == 0U))
    {
        s_search_active = 0U;
        s_search_rotation_deg = 0.0f;
        s_action = YAW_ALIGN_ACTION_IDLE;
        s_plan_valid_ticks = 0U;
        s_plan_invalid_ticks = 0U;
        yaw_angle_pid.kp = g_fc_params.yaw_angle_kp;
        yaw_angle_target = g_euler.yaw;
        s_yaw_delta_deg = 0.0f;
        return;
    }

    beacon.valid = 0U;
    s_active_beacon.valid = 0U;
    /* 原始信标仅保留给日志观察，不再参与mode=2搜索和降落判断。 */
    if((YawAlign_FindLargestBeacon(&beacon) != 0U) ||
       (YawAlign_FindLargestCenterBeacon(&beacon) != 0U))
    {
        s_active_beacon = beacon;
    }

    if(s_plan_valid != 0U)
    {
        s_plan_invalid_ticks = 0U;
        if(s_plan_valid_ticks < YAW_ALIGN_PLAN_VALID_TICKS)
        {
            s_plan_valid_ticks++;
        }
    }
    else
    {
        s_plan_valid_ticks = 0U;
        if(s_plan_invalid_ticks < YAW_ALIGN_PLAN_INVALID_TICKS)
        {
            s_plan_invalid_ticks++;
        }
    }
    s_plan_confirmed = (s_plan_valid_ticks >= YAW_ALIGN_PLAN_VALID_TICKS) ? 1U : 0U;

    if(s_search_active != 0U)
    {
        if(s_search_rotation_deg < YAW_ALIGN_SEARCH_COMPLETE_DEG)
        {
            s_search_rotation_deg += (float)s_search_direction * aircraft_yaw_delta_deg;
            if(s_search_rotation_deg < 0.0f)
            {
                s_search_rotation_deg = 0.0f;
            }
            if(s_search_rotation_deg > YAW_ALIGN_SEARCH_COMPLETE_DEG)
            {
                s_search_rotation_deg = YAW_ALIGN_SEARCH_COMPLETE_DEG;
            }
        }
    }

    if((s_plan_confirmed != 0U) && (s_search_forced == 0U))
    {
        s_search_active = 0U;
        s_lost_frames = 0U;
        s_search_rotation_deg = 0.0f;
        s_action = YAW_ALIGN_ACTION_DEADBAND_HOLD;
        yaw_angle_pid.kp = g_fc_params.yaw_angle_kp;
    }
    else if(s_search_active == 0U)
    {
        s_action = YAW_ALIGN_ACTION_IDLE;
        yaw_angle_pid.kp = g_fc_params.yaw_angle_kp;

        if(s_plan_invalid_ticks >= YAW_ALIGN_PLAN_INVALID_TICKS)
        {
            s_search_active = 1U;
            s_lost_frames = 0U;
            s_search_rotation_deg = 0.0f;
            if(s_cable_twist_deg > 0.0f)
            {
                s_search_direction = -1;
            }
            else if(s_cable_twist_deg < 0.0f)
            {
                s_search_direction = 1;
            }
            else if(g_car_yaw_rate_dps > 0.0f)
            {
                s_search_direction = 1;
            }
            else if(g_car_yaw_rate_dps < 0.0f)
            {
                s_search_direction = -1;
            }
            else
            {
                s_search_direction = (int8)(-s_search_direction);
            }
            s_search_target_yaw = YawAlign_NextSearchGrid(g_euler.yaw,
                                                           s_search_direction);
            s_action = YAW_ALIGN_ACTION_SEARCH;
            yaw_angle_pid.kp = 3.0f * g_fc_params.yaw_angle_kp;
        }
    }
    else
    {
        s_action = YAW_ALIGN_ACTION_SEARCH;
        yaw_angle_pid.kp = 3.0f * g_fc_params.yaw_angle_kp;

        /* 有效结果先保持当前45度档位；不足100ms失效后等待150ms再继续计时。 */
        if((s_plan_valid == 0U) &&
           (s_plan_invalid_ticks >= YAW_ALIGN_PLAN_INVALID_TICKS))
        {
            if(s_lost_frames < YAW_ALIGN_SEARCH_HOLD_TICKS)
            {
                s_lost_frames++;
            }
            if(s_lost_frames >= YAW_ALIGN_SEARCH_HOLD_TICKS)
            {
                s_lost_frames = 0U;
                s_search_target_yaw = YawAlign_Wrap180Deg(
                    s_search_target_yaw +
                    (float)s_search_direction * YAW_ALIGN_SEARCH_STEP_DEG);
            }
        }
    }

    yaw_angle_target = s_search_target_yaw;
    s_yaw_delta_deg = YawAlign_Wrap180Deg(yaw_angle_target - g_euler.yaw);
}

/*
 * 函数功能: 根据航向控制模式更新yaw目标和搜索状态
 * 输入参数: yaw_change_mode - 航向控制模式，0=固定0度，1=信标对准，2=步进搜索
 * 输出参数或返回值: 无
 */
void YawAlign_Update(float yaw_change_mode)
{
    uint8 control_mode;
    FC_START_CRSF_flight_mode_e flight_mode = FC_START_CRSF_Get_Flight_Mode();
    float aircraft_yaw_delta_deg = 0.0f;
    float car_yaw_delta_deg = 0.0f;

    if(yaw_change_mode < 0.5f)
    {
        control_mode = 0U;
    }
    else if(yaw_change_mode < 1.5f)
    {
        control_mode = 1U;
    }
    else
    {
        control_mode = 2U;
    }

    /* 自动降落双旋转阶段强制按步进搜索旋转，无视yawmode参数。 */
    if(s_search_forced != 0U)
    {
        control_mode = 2U;
    }

    if(flight_mode != FC_START_CRSF_FLIGHT_MODE_4)
    {
        s_mode4_target_sent = 0U;
    }

    /* 车端开关未使能时保持零航向目标，并禁止旋转搜索。 */
    if(CRSF_STD[4] != 1)
    {
        YawAlign_Reset();
        s_control_mode = 0U;
        yaw_angle_target = 0.0f;
        return;
    }

    /* 模式改变时清空旧信标锁定和搜索进度，再进入新模式；强制搜索除外。 */
    if(control_mode != s_control_mode)
    {
        if(s_search_forced == 0U)
        {
            YawAlign_Reset();
        }
        s_control_mode = control_mode;
    }

    /* 展开飞机和车辆航向，持续估计供搜索方向选择使用的线缆扭转。 */
    if(s_yaw_history_valid != 0U)
    {
        aircraft_yaw_delta_deg = YawAlign_Wrap180Deg(g_euler.yaw -
                                                      s_previous_air_yaw);
        car_yaw_delta_deg = YawAlign_Wrap180Deg(g_car_yaw -
                                                 s_previous_car_yaw);
        s_cable_twist_deg += aircraft_yaw_delta_deg - car_yaw_delta_deg;
    }
    else
    {
        s_cable_twist_deg = YawAlign_Wrap180Deg(g_euler.yaw - g_car_yaw);
        s_yaw_history_valid = 1U;
        if(control_mode == 2U)
        {
            s_search_target_yaw = YawAlign_RoundSearchGrid(g_euler.yaw);
        }
    }
    s_previous_air_yaw = g_euler.yaw;
    s_previous_car_yaw = g_car_yaw;

    if(control_mode == 0U)
    {
        yaw_angle_pid.kp = g_fc_params.yaw_angle_kp;
        yaw_angle_target = 0.0f;
    }
    else if(control_mode == 1U)
    {
        yaw_angle_pid.kp = g_fc_params.yaw_angle_kp;
        (void)YawAlign_UpdateBeacon();
    }
    else
    {
        YawAlign_UpdateSearch(aircraft_yaw_delta_deg);
    }
}

uint8 YawAlign_IsSearchActive(void)
{
    return s_search_active;
}

/*
 * 函数功能: 设置强制搜索，供自动降落双旋转阶段使用
 * 输入参数: forced - 1为强制持续旋转搜索，0为恢复正常搜索
 * 输出参数或返回值: 无
 */
void YawAlign_SetSearchForced(uint8 forced)
{
    s_search_forced = (forced != 0U) ? 1U : 0U;
}
