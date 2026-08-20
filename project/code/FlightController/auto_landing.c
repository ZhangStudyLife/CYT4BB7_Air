#include "auto_landing.h"
#include "fc_params.h"
#include "fc_mode.h"
#include "fc_start_crsf.h"
#include "yaw_align.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../Planner/car_plan_3.h"
#include "../Planner/car_plan_entry.h"
#include "../Protocols/crsf/crsf.h"
#include <math.h>

#define AUTO_LANDING_INITIAL_WAIT_TICKS (500U) /* 使能后前置等待5s，对应100Hz调用500次。 */
#define AUTO_LANDING_NO_TARGET_TICKS    (500U) /* 最近5s无速度下发判定降落。 */
#define AUTO_LANDING_VALID_TICKS        (20U)  /* CarPlan3速度下发需连续有效200ms才确认。 */
#define AUTO_LANDING_ROTATE_DEG         (360.0f) /* 飞机与车模各自旋转一圈的完成角，单位度。 */
#define AUTO_LANDING_ROTATE_RATE_DPS    (60.0f)  /* 车模旋转指令的航向推进速率，单位deg/s。 */
#define AUTO_LANDING_ROTATE_SPEED_MPS   (0.3f)   /* 车模旋转指令的平移速度，单位m/s。 */
#define AUTO_LANDING_ROTATE_TIMEOUT_TICKS (1000U) /* 双旋转阶段超时10s强制降落，防死锁。 */
#define AUTO_LANDING_DEG_TO_RAD         (0.017453292519943295f)

typedef enum
{
    AUTO_LANDING_STATE_IDLE = 0U, /* 入口条件未满足。 */
    AUTO_LANDING_STATE_DETECT,    /* 5s等待后检测最近5s是否有速度下发。 */
    AUTO_LANDING_STATE_ROTATE,    /* 关闭负压并让飞机与车模各旋转一圈。 */
    AUTO_LANDING_STATE_TRIGGERED  /* 已请求降落，锁存直到回待机。 */
} auto_landing_state_e;

extern float g_car_yaw;
extern float g_car_sync_time_ms;
extern uint32 g_car_last_update_time_ms;
extern volatile uint32 tick_1000us_cnt;

static auto_landing_state_e s_state = AUTO_LANDING_STATE_IDLE;
static uint16 s_initial_wait_ticks = 0U; /* 自动降落前置等待计数。 */
static uint16 s_no_target_ticks = 0U; /* 连续无可信目标计数。 */
static uint16 s_valid_target_ticks = 0U; /* CarPlan3连续有效计数。 */
static uint16 s_rotate_timeout_ticks = 0U; /* 双旋转阶段超时计数。 */
static float s_rotate_air_accum_deg = 0.0f; /* 双旋转阶段飞机累计旋转角。 */
static float s_rotate_car_accum_deg = 0.0f; /* 双旋转阶段车模累计旋转角。 */
static float s_rotate_prev_air_yaw = 0.0f; /* 双旋转阶段上次飞机航向。 */
static float s_rotate_prev_car_yaw = 0.0f; /* 双旋转阶段上次车模航向。 */
static float s_rotate_car_heading_deg = 0.0f; /* 车模旋转指令的绝对航向目标。 */
static float s_rotate_car_dir = 1.0f; /* 车模旋转方向，1为yaw正方向，-1为yaw负方向。 */
static float s_rotate_strafe_mps = 0.0f; /* 车模旋转指令横移速度。 */
static float s_rotate_forward_mps = 0.0f; /* 车模旋转指令前向速度。 */

static float AutoLanding_Wrap180Deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void AutoLanding_ResetAll(void)
{
    s_state = AUTO_LANDING_STATE_IDLE;
    s_initial_wait_ticks = 0U;
    s_no_target_ticks = 0U;
    s_valid_target_ticks = 0U;
    s_rotate_timeout_ticks = 0U;
    s_rotate_air_accum_deg = 0.0f;
    s_rotate_car_accum_deg = 0.0f;
    s_rotate_strafe_mps = 0.0f;
    s_rotate_forward_mps = 0.0f;
    YawAlign_SetSearchForced(0U);
}

/*
 * 函数功能: 进入双旋转阶段，飞机与车模同向各旋转一圈
 * 输入参数: 无
 * 输出参数或返回值: 无
 */
static void AutoLanding_StartRotate(void)
{
    yaw_align_debug_t yaw_debug;

    s_state = AUTO_LANDING_STATE_ROTATE;
    s_rotate_timeout_ticks = 0U;
    s_rotate_air_accum_deg = 0.0f;
    s_rotate_car_accum_deg = 0.0f;
    s_rotate_prev_air_yaw = g_euler.yaw;
    s_rotate_prev_car_yaw = g_car_yaw;
    s_rotate_car_heading_deg = g_car_yaw;
    /* 车模与飞机搜索同向旋转，避免线缆额外扭转。 */
    YawAlign_GetDebug(&yaw_debug);
    s_rotate_car_dir = (yaw_debug.search_direction < 0) ? -1.0f : 1.0f;
    /* 强制飞机持续旋转搜索，无视plan抖动有效导致的搜索停止。 */
    YawAlign_SetSearchForced(1U);
}

/*
 * 函数功能: 以100Hz推进双旋转阶段，两圈完成或超时后请求现有降落程序
 * 输入参数: 无
 * 输出参数或返回值: 无
 */
static void AutoLanding_UpdateRotate(void)
{
    float air_delta_deg;
    float car_delta_deg;
    float rel_deg;

    if (s_rotate_timeout_ticks < AUTO_LANDING_ROTATE_TIMEOUT_TICKS)
    {
        s_rotate_timeout_ticks++;
    }

    /* 按[-180,180]包角差累计飞机与车模的实际旋转角。 */
    air_delta_deg = AutoLanding_Wrap180Deg(g_euler.yaw - s_rotate_prev_air_yaw);
    car_delta_deg = AutoLanding_Wrap180Deg(g_car_yaw - s_rotate_prev_car_yaw);
    s_rotate_prev_air_yaw = g_euler.yaw;
    s_rotate_prev_car_yaw = g_car_yaw;
    if (s_rotate_air_accum_deg < AUTO_LANDING_ROTATE_DEG)
    {
        s_rotate_air_accum_deg += air_delta_deg;
    }
    if (s_rotate_car_accum_deg < AUTO_LANDING_ROTATE_DEG)
    {
        s_rotate_car_accum_deg += car_delta_deg;
    }
    if (s_rotate_air_accum_deg < 0.0f)
    {
        s_rotate_air_accum_deg = 0.0f;
    }
    if (s_rotate_car_accum_deg < 0.0f)
    {
        s_rotate_car_accum_deg = 0.0f;
    }

    /* 车模旋转指令：持续推进绝对航向，车端按atan2(strafe,forward)相对自身航向转向跟随。 */
    if (s_rotate_car_accum_deg < AUTO_LANDING_ROTATE_DEG)
    {
        s_rotate_car_heading_deg +=
            s_rotate_car_dir * AUTO_LANDING_ROTATE_RATE_DPS * 0.01f;
        rel_deg = s_rotate_car_heading_deg - g_car_yaw;
        s_rotate_strafe_mps = AUTO_LANDING_ROTATE_SPEED_MPS *
                              sinf(rel_deg * AUTO_LANDING_DEG_TO_RAD);
        s_rotate_forward_mps = AUTO_LANDING_ROTATE_SPEED_MPS *
                               cosf(rel_deg * AUTO_LANDING_DEG_TO_RAD);
    }
    else
    {
        s_rotate_strafe_mps = 0.0f;
        s_rotate_forward_mps = 0.0f;
    }

    if ((s_rotate_air_accum_deg >= AUTO_LANDING_ROTATE_DEG) &&
        (s_rotate_car_accum_deg >= AUTO_LANDING_ROTATE_DEG))
    {
        YawAlign_SetSearchForced(0U);
        s_state = AUTO_LANDING_STATE_TRIGGERED;
        FC_START_CRSF_Request_Landing();
    }
    else if (s_rotate_timeout_ticks >= AUTO_LANDING_ROTATE_TIMEOUT_TICKS)
    {
        /* 超时兜底：旋转未完成也进入降落，避免长期滞留空中。 */
        YawAlign_SetSearchForced(0U);
        s_state = AUTO_LANDING_STATE_TRIGGERED;
        FC_START_CRSF_Request_Landing();
    }
}

/*
 * 函数功能: 以100Hz检测Mode4自动降落条件并请求现有落地程序
 * 输入参数: 无
 * 输出参数或返回值: 无
 */
void AutoLanding_Update100Hz(void)
{
    FC_START_CRSF_state_e state = FC_START_CRSF_Get_State();
    FC_START_CRSF_flight_mode_e mode = FC_START_CRSF_Get_Flight_Mode();
    car_plan_result_t plan3_result;
    uint8 car_started;

    if (s_state == AUTO_LANDING_STATE_TRIGGERED)
    {
        if ((state == FC_START_CRSF_STATE_STANDBY) ||
            (state == FC_START_CRSF_STATE_INIT))
        {
            AutoLanding_ResetAll();
        }
        return;
    }

    /* 车模启动判定：最近200ms内有车端时间戳推进。 */
    car_started = ((g_car_sync_time_ms > 0.0f) &&
                   ((tick_1000us_cnt - g_car_last_update_time_ms) <
                    FC_MODE_CAR_RUN_DATA_TIMEOUT_MS)) ? 1U : 0U;

    if (s_state == AUTO_LANDING_STATE_ROTATE)
    {
        if ((state != FC_START_CRSF_STATE_FLYING) ||
            (mode != FC_START_CRSF_FLIGHT_MODE_4) ||
            (g_fc_params.yaw_change_mode4 >= 0.5f) ||
            (CRSF_STD[4] != 1))
        {
            AutoLanding_ResetAll();
            return;
        }
        AutoLanding_UpdateRotate();
        return;
    }

    if ((state != FC_START_CRSF_STATE_FLYING) ||
        (mode != FC_START_CRSF_FLIGHT_MODE_4) ||
        (g_fc_params.yaw_change_mode4 >= 0.5f) || /* 仅yawmode0自动降落，任务期间yaw目标保持0度。 */
        (CRSF_STD[4] != 1) ||
        (Car_Plan_Mode != 3) ||
        (car_started == 0U))
    {
        AutoLanding_ResetAll();
        return;
    }

    if (s_state == AUTO_LANDING_STATE_IDLE)
    {
        s_state = AUTO_LANDING_STATE_DETECT;
    }

    if (s_initial_wait_ticks < AUTO_LANDING_INITIAL_WAIT_TICKS)
    {
        s_initial_wait_ticks++;
        return;
    }

    CarPlan_3_GetResult(&plan3_result);
    if (plan3_result.valid != 0U)
    {
        if (s_valid_target_ticks < AUTO_LANDING_VALID_TICKS)
        {
            s_valid_target_ticks++;
        }
        if (s_valid_target_ticks >= AUTO_LANDING_VALID_TICKS)
        {
            s_no_target_ticks = 0U;
            return;
        }
    }
    else
    {
        s_valid_target_ticks = 0U;
    }

    if (s_no_target_ticks < AUTO_LANDING_NO_TARGET_TICKS)
    {
        s_no_target_ticks++;
    }
    if (s_no_target_ticks >= AUTO_LANDING_NO_TARGET_TICKS)
    {
        AutoLanding_StartRotate();
    }
}

/*
 * 函数功能: 查询自动降落触发锁存状态
 * 输入参数: 无
 * 输出参数或返回值: 1表示自动降落已触发，0表示未触发
 */
uint8 AutoLanding_IsTriggered(void)
{
    return (s_state == AUTO_LANDING_STATE_TRIGGERED) ? 1U : 0U;
}

/*
 * 函数功能: 查询双旋转阶段是否进行中
 * 输入参数: 无
 * 输出参数或返回值: 1表示飞机与车模正在各旋转一圈，0表示未在旋转
 */
uint8 AutoLanding_IsRotationActive(void)
{
    return (s_state == AUTO_LANDING_STATE_ROTATE) ? 1U : 0U;
}

/*
 * 函数功能: 获取双旋转阶段下发车模的旋转指令速度
 * 输入参数: strafe_mps - 车体右向速度输出地址，不可为空
 *          forward_mps - 车体前向速度输出地址，不可为空
 * 输出参数或返回值: 无
 */
void AutoLanding_GetRotateCommand(float *strafe_mps, float *forward_mps)
{
    *strafe_mps = s_rotate_strafe_mps;
    *forward_mps = s_rotate_forward_mps;
}

/*
 * 函数功能: 获取自动降落检测的只读调试快照
 * 输入参数: debug - 调试快照输出地址，不可为空
 * 输出参数或返回值: 无
 */
void AutoLanding_GetDebug(auto_landing_debug_t *debug)
{
    yaw_align_debug_t yaw_debug;

    YawAlign_GetDebug(&yaw_debug);
    debug->initial_wait_ticks = s_initial_wait_ticks;
    debug->no_target_ticks = s_no_target_ticks;
    debug->valid_target_ticks = s_valid_target_ticks;
    debug->target_valid = (s_valid_target_ticks >= AUTO_LANDING_VALID_TICKS) ? 1U : 0U;
    debug->rotation_ready = (yaw_debug.search_rotation_deg >=
                             AUTO_LANDING_ROTATE_DEG) ? 1U : 0U;
    debug->triggered = (s_state == AUTO_LANDING_STATE_TRIGGERED) ? 1U : 0U;
    debug->state = (uint8)s_state;
    debug->car_started = ((g_car_sync_time_ms > 0.0f) &&
                          ((tick_1000us_cnt - g_car_last_update_time_ms) <
                           FC_MODE_CAR_RUN_DATA_TIMEOUT_MS)) ? 1U : 0U;
    debug->rotate_active = (s_state == AUTO_LANDING_STATE_ROTATE) ? 1U : 0U;
    debug->rotate_air_deg = s_rotate_air_accum_deg;
    debug->rotate_car_deg = s_rotate_car_accum_deg;
}
