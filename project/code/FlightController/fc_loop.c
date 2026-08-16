#include "fc_loop.h"
#include "fc_mode.h"
#include <math.h>
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../IPC/ipc_image_data.h"
#include "../Planner/car_lamp_fused.h"
#include "../Planner/car_plan_3.h"
#include "../Planner/beacon_lost_detector.h"
#include "../Planner/pix_to_distance.h"
#include "../Planner/ProjectionCenter.h"
#include "../Planner/pull_detect.h"
#include "../Protocols/crsf/crsf.h"

pid_t roll_gyro_pid;
pid_t pitch_gyro_pid;
pid_t yaw_gyro_pid;
pid_t roll_angle_pid;
pid_t pitch_angle_pid;
pid_t yaw_angle_pid;
static pid_t height_pos_pid;
static pid_t height_vel_pid;
/* Roll 角速度目标，单位度每秒 */
float roll_gyro_target = 0.0f;
/* Pitch 角速度目标，单位度每秒 */
float pitch_gyro_target = 0.0f;
/* Yaw 角速度目标，单位度每秒 */
float yaw_gyro_target = 0.0f;
/* Roll 角度目标，单位度 */
float roll_angle_target = 0.0f;
/* Pitch 角度目标，单位度 */
float pitch_angle_target = 0.0f;
/* Yaw 角度目标，单位度 */
float yaw_angle_target = 0.0f;
/* 水平速度控制允许输出的最大 Roll/Pitch 目标角，单位 deg */
float angle_target_max = 30.0f;
/* 正常飞行目标高度，单位 m */
float g_fc_target_height_m = 1.1f;
/* 高度速度环输出，单位 PWM */
static float height_vel_out = 0.0f;
/* 高度位置环输出，单位米每秒 */
static float height_pos_out = 0.0f;
/* 目标高度，单位米 */
extern volatile uint32 tick_1000us_cnt;

/* Yaw 角度目标是否已经对齐当前机头方向 */
static uint8_t s_yaw_target_inited = 0U;
#define FC_LANDING_TARGET_HEIGHT_M -0.5f
/* 100Hz 锁存的飞行模式，50Hz 控制只消费该锁存值 */
static FC_START_CRSF_flight_mode_e s_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
/* 上一次锁存的飞行模式，用于检测模式切换边沿 */
static FC_START_CRSF_flight_mode_e s_prev_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
/* 上一次飞控状态，用于检测飞行态切换边沿 */
static FC_START_CRSF_state_e s_prev_fc_state = FC_START_CRSF_STATE_INIT;
/* Mode0 手动姿态前馈是否已完成目标对齐 */
static uint8_t s_mode0_angle_ff_active = 0U;
/* 悬停油门在线学习（借鉴 ArduPilot MOT_THST_HOVER） */
#define FC_HOVER_THR_TC 4.0f
#define FC_HOVER_LEARN_MIN_DELTA (-700.0f)
#define FC_HOVER_LEARN_MAX_DELTA 900.0f
#define FC_HOVER_LEARN_VZ_MAX 0.18f
#define FC_HOVER_LEARN_POS_MAX 0.08f
#define FC_HOVER_LEARN_TILT_MAX 7.0f
#define FC_HOVER_LEARN_ACC_MAX 0.20f
#define FC_HOVER_LEARN_TOF_SPREAD_MAX 250.0f
#define FC_HOVER_LEARN_RATE_MAX 220.0f
#define FC_THRUST_ACC_MPS2_PER_PWM 0.00360f
#define FC_HEIGHT_VEL_KD_ACC 0.035f
#define FC_HEIGHT_VEL_TARGET_LIMIT 0.60f
#define FC_HEIGHT_VEL_OUT_MIN (-1000.0f)
#define FC_HEIGHT_VEL_OUT_MAX 1000.0f
#define FC_HEIGHT_VEL_KD_PWM (FC_HEIGHT_VEL_KD_ACC / FC_THRUST_ACC_MPS2_PER_PWM)
static float s_hover_throttle = 3200.0f;
static float s_hover_learn_step = 0.0f;
/* 姿态角外环输出到角速度目标的限幅，单位 deg/s */
static const float s_fc_angle_out_limit = 260.0f;
static const float s_fc_yaw_out_limit = 2000.0f;
/* Yaw 角度保持修正限幅，单位 deg/s */
static const float s_fc_yaw_hold_rate_limit_dps = 500.0f;
/* Yaw 目标相对当前航向的最大超前角，单位 deg */
static const float s_fc_yaw_target_delta_limit_deg = 100.0f;
/* 姿态角外环 anti-windup 回算增益 */
static const float s_fc_angle_aw_gain = 0.15f;
/* 姿态角外环积分松弛阈值，目标变化过快时降低积分堆积 */
static const float s_fc_angle_iterm_relax_threshold = 30.0f;
static const float s_fc_deg_to_rad = 0.017453293f;
static const float s_fc_tilt_cos_min = 0.8f;
static const float s_fc_tilt_comp_throttle_max = 10000.0f;

/*
 * 函数名: fc_clampf
 * 功能: 对浮点数进行上下限钳位
 * 输入参数:
 *   value     - 输入值
 *   min_value - 最小允许值
 *   max_value - 最大允许值
 * 返回值:
 *   限幅后的浮点值
 */
static float fc_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

/*
 * 函数名: FC_Wrap180Deg
 * 功能: 将角度包裹到 [-180, 180]，避免 yaw 跨边界时出现 360 度跳变
 * 输入参数:
 *   angle_deg - 输入角度，单位 deg
 * 返回值:
 *   包裹后的角度，单位 deg
 */
static float FC_Wrap180Deg(float angle_deg)
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

/*
 * 函数名: FC_Apply_Tilt_Throttle_Compensation
 * 功能: 使用当前 Roll/Pitch 原始姿态角，对总油门做保守的垂向分力补偿
 * 输入参数:
 *   throttle_raw - 补偿前总油门，单位 mixer 输入
 * 返回值:
 *   补偿后的总油门，已限幅到[0, s_fc_tilt_comp_throttle_max]
 */
static float FC_Apply_Tilt_Throttle_Compensation(float throttle_raw)
{
    float cos_term;

    if (throttle_raw <= 0.0f)
    {
        return 0.0f;
    }

    /* 直接使用当前原始姿态角，补偿相对重力方向的总倾斜损失 */
    cos_term = cosf(g_euler.roll * s_fc_deg_to_rad) * cosf(g_euler.pitch * s_fc_deg_to_rad);
    cos_term = fc_clampf(cos_term, s_fc_tilt_cos_min, 1.0f);

    return fc_clampf(throttle_raw / cos_term, 0.0f, s_fc_tilt_comp_throttle_max);
}

/*
 * 函数名: FC_Reset_All_Mode_Control
 * 功能: 统一复位所有模式控制状态
 * 输入参数: 无
 * 返回值: 无
 */
static void FC_Reset_All_Mode_Control(void)
{
    FC_Mode0_Reset();
    FC_Mode1_Reset();
    FC_Mode2_Reset();
    FC_Mode3_Reset();
    FC_Mode4_Reset();
    FC_Mode5_Reset();
    FC_Mode6_Reset();
    FC_Mode7_Reset();
    FC_Mode8_Reset();
}

/*
 * 函数名: FC_Handle_Mode_Transition_100Hz
 * 功能: 在100Hz统一处理模式切换与飞行状态切换复位
 * 输入参数:
 *   flight_mode - 当前锁存飞行模式
 *   fc_state    - 当前飞控状态
 * 返回值: 无
 */
static void FC_Handle_Mode_Transition_100Hz(FC_START_CRSF_flight_mode_e flight_mode,
                                            FC_START_CRSF_state_e fc_state)
{
    uint8 need_reset = 0U;

    if (s_prev_flight_mode != flight_mode)
    {
        need_reset = 1U;
    }
    if (s_prev_fc_state != fc_state)
    {
        need_reset = 1U;
    }

    if (need_reset != 0U)
    {
        FC_Reset_All_Mode_Control();
    }

    s_prev_flight_mode = flight_mode;
    s_prev_fc_state = fc_state;
}

/*
 * 函数名: FC_Loop_Init
 * 功能: 初始化飞控各级 PID 与控制输出默认状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Loop_Init(void)
{
    PID_Init(&roll_angle_pid,
             g_fc_params.roll_angle_kp, g_fc_params.roll_angle_ki, g_fc_params.roll_angle_kd,
             g_fc_params.roll_angle_kff, g_fc_params.angle_dt,
             g_fc_params.roll_angle_i_limit, g_fc_params.roll_angle_d_lpf);
    /* Roll 姿态外环启用工程限幅和 anti-windup，避免持续偏差把角速度目标推得过猛 */
    roll_angle_pid.aw_enable = 1U;
    roll_angle_pid.aw_gain = s_fc_angle_aw_gain;
    roll_angle_pid.output_min = -s_fc_angle_out_limit;
    roll_angle_pid.output_max = s_fc_angle_out_limit;
    roll_angle_pid.iterm_relax_threshold = s_fc_angle_iterm_relax_threshold;
    PID_SetFeedforwardFilter(&roll_angle_pid, g_fc_params.angle_ff_smoothing_ms,
                             g_fc_params.angle_output_lpf_hz, g_fc_params.angle_ff_limit_dps);
    PID_Init(&pitch_angle_pid,
             g_fc_params.pitch_angle_kp, g_fc_params.pitch_angle_ki, g_fc_params.pitch_angle_kd,
             g_fc_params.pitch_angle_kff, g_fc_params.angle_dt,
             g_fc_params.pitch_angle_i_limit, g_fc_params.pitch_angle_d_lpf);
    /* Pitch 姿态外环与 Roll 保持相同保护策略，减小两轴控制品质分叉 */
    pitch_angle_pid.aw_enable = 1U;
    pitch_angle_pid.aw_gain = s_fc_angle_aw_gain;
    pitch_angle_pid.output_min = -s_fc_angle_out_limit;
    pitch_angle_pid.output_max = s_fc_angle_out_limit;
    pitch_angle_pid.iterm_relax_threshold = s_fc_angle_iterm_relax_threshold;
    PID_SetFeedforwardFilter(&pitch_angle_pid, g_fc_params.angle_ff_smoothing_ms,
                             g_fc_params.angle_output_lpf_hz, g_fc_params.angle_ff_limit_dps);
    PID_Init(&yaw_angle_pid,
             g_fc_params.yaw_angle_kp, g_fc_params.yaw_angle_ki, g_fc_params.yaw_angle_kd,
             g_fc_params.yaw_angle_kff, g_fc_params.angle_dt,
             g_fc_params.yaw_angle_i_limit, g_fc_params.yaw_angle_d_lpf);

    PID_Init(&roll_gyro_pid,
             g_fc_params.roll_gyro_kp, g_fc_params.roll_gyro_ki, g_fc_params.roll_gyro_kd,
             g_fc_params.roll_gyro_kff, g_fc_params.gyro_dt,
             g_fc_params.roll_gyro_i_limit, g_fc_params.roll_gyro_d_lpf);
    PID_SetFeedforwardFilter(&roll_gyro_pid, g_fc_params.gyro_ff_smoothing_ms, 0.0f,
                             g_fc_params.gyro_ff_limit);
    /* Roll 角速度目标持续变化时，提前放松积分，减少低频拉扯和线缆外力带来的积分堆积 */
    roll_gyro_pid.iterm_relax_threshold = 40.0f;
    PID_Init(&pitch_gyro_pid,
             g_fc_params.pitch_gyro_kp, g_fc_params.pitch_gyro_ki, g_fc_params.pitch_gyro_kd,
             g_fc_params.pitch_gyro_kff, g_fc_params.gyro_dt,
             g_fc_params.pitch_gyro_i_limit, g_fc_params.pitch_gyro_d_lpf);
    PID_SetFeedforwardFilter(&pitch_gyro_pid, g_fc_params.gyro_ff_smoothing_ms, 0.0f,
                             g_fc_params.gyro_ff_limit);
    /* Pitch 角速度环与 Roll 保持相同的积分放松策略，降低连续目标变化时的积分拖拽 */
    pitch_gyro_pid.iterm_relax_threshold = 40.0f;
    PID_Init(&yaw_gyro_pid,
             g_fc_params.yaw_gyro_kp, g_fc_params.yaw_gyro_ki, g_fc_params.yaw_gyro_kd,
             g_fc_params.yaw_gyro_kff, g_fc_params.gyro_dt,
             g_fc_params.yaw_gyro_i_limit, g_fc_params.yaw_gyro_d_lpf);
    PID_Init(&height_pos_pid, g_fc_params.pos_z_kp, 0.0f, 0.0f, 0.0f,
             g_fc_params.pos_z_dt, 0.0f, 0.0f);
    PID_Init(&height_vel_pid, g_fc_params.vel_z_kp, g_fc_params.vel_z_ki, FC_HEIGHT_VEL_KD_PWM, 0.0f,
             g_fc_params.vel_z_dt, g_fc_params.vel_z_i_limit, 12.0f);
    height_pos_pid.aw_enable = 1U;
    height_pos_pid.output_min = -FC_HEIGHT_VEL_TARGET_LIMIT;
    height_pos_pid.output_max = FC_HEIGHT_VEL_TARGET_LIMIT;
    height_vel_pid.aw_enable = 1U;
    height_vel_pid.output_min = FC_HEIGHT_VEL_OUT_MIN;
    height_vel_pid.output_max = FC_HEIGHT_VEL_OUT_MAX;

    FC_Mode0_Init();
    FC_Mode1_Init();
    FC_Mode2_Init();
    FC_Mode3_Init();
    FC_Mode4_Init();
    FC_Mode5_Init();
    FC_Mode6_Init();
    FC_Mode7_Init();
    FC_Mode8_Init();
    FC_Reset_All_Mode_Control();
    CarLampFused_Init();
    CarPlanEntry_Reset();
    PixToDistance_Init();
    ProjectionCenter_Init();
    PullDetect_Init();
    s_hover_throttle = (float)g_fc_params.base_throttle;
}

/*
 * 函数名: FC_Loop_Reset
 * 功能: 复位飞控控制环内部状态与关键目标量
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Loop_Reset(void)
{
    PID_Reset(&roll_angle_pid);
    PID_Reset(&pitch_angle_pid);
    PID_Reset(&yaw_angle_pid);
    PID_Reset(&roll_gyro_pid);
    PID_Reset(&pitch_gyro_pid);
    PID_Reset(&yaw_gyro_pid);
    PID_Reset(&height_pos_pid);
    PID_Reset(&height_vel_pid);
    FC_Reset_All_Mode_Control();
    CarLampFused_Init();
    CarPlanEntry_Reset();
    PixToDistance_Init();
    ProjectionCenter_Init();
    PullDetect_Init();
    roll_gyro_target = 0.0f;
    pitch_gyro_target = 0.0f;
    yaw_gyro_target = 0.0f;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    yaw_angle_target = 0.0f;
    height_pos_out = 0.0f;
    height_vel_out = 0.0f;
    s_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
    s_prev_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
    s_prev_fc_state = FC_START_CRSF_STATE_INIT;
    s_mode0_angle_ff_active = 0U;
    s_yaw_target_inited = 0U;
    s_hover_throttle = (float)g_fc_params.base_throttle;
}

/*
 * 函数名: FC_Loop_50Hz
 * 功能: 执行50Hz高度位置环，并分发非图像模式的50Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Loop_50Hz(void)
{
    static uint32 tick_1000us_cnt_last = 0;
    uint32 tick_now = tick_1000us_cnt;
    uint32 diff = tick_now - tick_1000us_cnt_last;
    float dt = diff * 0.001f;
    FC_START_CRSF_state_e fc_state = FC_START_CRSF_Get_State();
    // uint8 car_data_fresh;

    tick_1000us_cnt_last = tick_now;
    if (dt < 0.0001f)
    {
        dt = 0.02f;
    }

    if (((fc_state == FC_START_CRSF_STATE_FLYING) || (fc_state == FC_START_CRSF_STATE_LANDING)) &&
        (0U != g_tof_fused_valid) &&
        (g_height_meas_health >= 0.25f))
    {
        height_pos_pid.kp = g_fc_params.pos_z_kp;
        height_pos_out = PID_Update(&height_pos_pid,
                                    (fc_state == FC_START_CRSF_STATE_LANDING) ? FC_LANDING_TARGET_HEIGHT_M : g_fc_target_height_m,
                                    g_tof_fused_height_mm * 0.001f,
                                    dt);
        height_pos_out = fc_clampf(height_pos_out, -FC_HEIGHT_VEL_TARGET_LIMIT, FC_HEIGHT_VEL_TARGET_LIMIT);
    }
    else if ((fc_state == FC_START_CRSF_STATE_FLYING) &&
             (g_height_meas_health < 0.25f))
    {
        height_pos_out = fc_clampf(height_pos_out, -0.10f, 0.10f);
    }
    else
    {
        height_pos_out = 0.0f;
    }

    switch (s_flight_mode)
    {
    case FC_START_CRSF_FLIGHT_MODE_0:
        FC_Mode0_50Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_3:
        FC_Mode3_50Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_1:
    case FC_START_CRSF_FLIGHT_MODE_2:
    case FC_START_CRSF_FLIGHT_MODE_4:
    case FC_START_CRSF_FLIGHT_MODE_5:
    case FC_START_CRSF_FLIGHT_MODE_8:
        break;

    case FC_START_CRSF_FLIGHT_MODE_6:
        FC_Mode0_50Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_7:
        FC_Mode7_50Hz(dt);
        break;

    default:
        FC_Mode0_50Hz(dt);
        break;
    }

    // 先单纯调飞机,没有车
    // car_data_fresh = ((g_car_sync_time_ms > 0.0f) &&
    //                   ((tick_now - g_car_last_update_time_ms) < FC_MODE_CAR_RUN_DATA_TIMEOUT_MS)) ? 1U : 0U;
    // Beep_SetAlarm(BEEP_ALARM_CAR_DATA_LOST,
    //               (car_data_fresh == 0U) ? 1U : 0U);

}

/*
 * 函数名: FC_Loop_100Hz
 * 功能: 执行100Hz高度测速、高度速度环、图像链与模式分发
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Loop_100Hz(void)
{
    static uint32 tick_1000us_cnt_last = 0;
    FC_START_CRSF_state_e fc_state;
    uint32 tick_now = tick_1000us_cnt;
    uint32 diff = tick_now - tick_1000us_cnt_last;
    float dt = diff * 0.001f;

    tick_1000us_cnt_last = tick_now;
    if (dt < 0.0001f)
    {
        dt = 0.01f;
    }

    fc_state = FC_START_CRSF_Get_State();
    s_flight_mode = FC_START_CRSF_Get_Flight_Mode(); /* 检测遥控器的模式 */
    FC_Handle_Mode_Transition_100Hz(s_flight_mode, fc_state);

    CarLampFused_Update100Hz();
    ProjectionCenter_Update100Hz();
    PixToDistance_UpdateProjectionCenter2_100Hz();
    if ((fc_state == FC_START_CRSF_STATE_FLYING) || (fc_state == FC_START_CRSF_STATE_LANDING))
    {
        PullDetect_Update100Hz(g_car_lamp_fused_distance_projectioncenter_2.valid,
                               g_car_lamp_fused_distance_projectioncenter_2.x_cm,
                               g_car_lamp_fused_distance_projectioncenter_2.y_cm);
    }
    else
    {
        PullDetect_Init();
    }

    if ((fc_state == FC_START_CRSF_STATE_FLYING) || (fc_state == FC_START_CRSF_STATE_LANDING))
    {
        height_vel_pid.kp = g_fc_params.vel_z_kp;
        height_vel_pid.ki = g_fc_params.vel_z_ki;
        if ((0U == g_tof_fused_valid) || (g_height_meas_health < 0.25f))
        {
            height_pos_out = 0.0f;
        }
        height_vel_out = PID_Update(&height_vel_pid, height_pos_out, g_height_fused_vz_mps, dt);
        height_vel_out = fc_clampf(height_vel_out, FC_HEIGHT_VEL_OUT_MIN, FC_HEIGHT_VEL_OUT_MAX);
    }
    else
    {
        height_pos_out = 0.0f;
        height_vel_out = 0.0f;
    }

    /* 悬停油门在线学习：仅在接近稳态悬停时更新 */
    s_hover_learn_step = 0.0f;
    if ((fc_state == FC_START_CRSF_STATE_FLYING) &&
        (g_height_meas_health > 0.65f) &&
        (g_height_fused_vz_mps > -FC_HOVER_LEARN_VZ_MAX) && (g_height_fused_vz_mps < FC_HOVER_LEARN_VZ_MAX) &&
        (height_pos_out > -FC_HOVER_LEARN_POS_MAX) && (height_pos_out < FC_HOVER_LEARN_POS_MAX) &&
        (g_euler.roll > -FC_HOVER_LEARN_TILT_MAX) && (g_euler.roll < FC_HOVER_LEARN_TILT_MAX) &&
        (g_euler.pitch > -FC_HOVER_LEARN_TILT_MAX) && (g_euler.pitch < FC_HOVER_LEARN_TILT_MAX) &&
        (fabsf(sqrtf((g_imufilter_1000hz.accx * g_imufilter_1000hz.accx) + (g_imufilter_1000hz.accy * g_imufilter_1000hz.accy) + (g_imufilter_1000hz.accz * g_imufilter_1000hz.accz)) - 1.0f) < FC_HOVER_LEARN_ACC_MAX) &&
        ((fmaxf(fmaxf(g_tof1_height_mm, g_tof2_height_mm), fmaxf(g_tof3_height_mm, g_tof4_height_mm)) -
          fminf(fminf(g_tof1_height_mm, g_tof2_height_mm), fminf(g_tof3_height_mm, g_tof4_height_mm))) < FC_HOVER_LEARN_TOF_SPREAD_MAX))
    {
        s_hover_learn_step = fc_clampf((dt / (dt + FC_HOVER_THR_TC)) * height_vel_out,
                                       -FC_HOVER_LEARN_RATE_MAX * dt, FC_HOVER_LEARN_RATE_MAX * dt);
        s_hover_learn_step = fc_clampf(s_hover_throttle + s_hover_learn_step,
                                       (float)g_fc_params.base_throttle + FC_HOVER_LEARN_MIN_DELTA,
                                       (float)g_fc_params.base_throttle + FC_HOVER_LEARN_MAX_DELTA) -
                             s_hover_throttle;
        s_hover_throttle += s_hover_learn_step;
        height_vel_out -= s_hover_learn_step;
    }

    







    if (fc_state == FC_START_CRSF_STATE_LANDING)
    {
        FC_Mode0_100Hz();
        return;
    }

    switch (s_flight_mode)
    {
    case FC_START_CRSF_FLIGHT_MODE_0:
        FC_Mode0_100Hz();
        break;

    case FC_START_CRSF_FLIGHT_MODE_1:
        FC_Mode1_100Hz();
        FC_Mode1_Control100Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_2:
        FC_Mode2_100Hz();
        FC_Mode2_Control100Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_3:
        FC_Mode3_100Hz();
        break;

    case FC_START_CRSF_FLIGHT_MODE_4:
        FC_Mode4_100Hz();
        FC_Mode4_Control100Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_5:
        FC_Mode5_100Hz();
        FC_Mode5_Control100Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_6:
        FC_Mode0_100Hz();
        break;

    case FC_START_CRSF_FLIGHT_MODE_7:
        FC_Mode7_100Hz();
        break;

    case FC_START_CRSF_FLIGHT_MODE_8:
        FC_Mode8_100Hz();
        FC_Mode8_Control100Hz(dt);
        break;

    default:
        FC_Mode0_100Hz();
        break;
    }
}

void FC_Loop_500Hz(void)
{
    static uint32 tick_1000us_cnt_last = 0;
    uint32 tick_now = tick_1000us_cnt; // 读一次，缓存
    uint32 diff = tick_now - tick_1000us_cnt_last;
    float dt = diff * 0.001f; // 秒

    tick_1000us_cnt_last = tick_now;
    if ((FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING) ||
        (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_LANDING))
    {
        float roll_angle_meas = g_euler.roll;
        float pitch_angle_meas = g_euler.pitch;
        float yaw_angle_meas = g_euler.yaw;
        float yaw_error_deg;
        float limit;
        float roll_ctrl;
        float pitch_ctrl;
        uint8_t mode0_angle_ff_enable;

        if (roll_angle_target > angle_target_max)
        {
            roll_angle_target = angle_target_max;
        }
        if (roll_angle_target < -angle_target_max)
        {
            roll_angle_target = -angle_target_max;
        }
        if (pitch_angle_target > angle_target_max)
        {
            pitch_angle_target = angle_target_max;
        }
        if (pitch_angle_target < -angle_target_max)
        {
            pitch_angle_target = -angle_target_max;
        }

        /* 控制量限幅 */
        limit = s_fc_angle_out_limit;
        mode0_angle_ff_enable = ((s_flight_mode == FC_START_CRSF_FLIGHT_MODE_0) ||
                                 (s_flight_mode == FC_START_CRSF_FLIGHT_MODE_3) ||
                                 (s_flight_mode == FC_START_CRSF_FLIGHT_MODE_6)) ? 1U : 0U;
        if ((mode0_angle_ff_enable != 0U) && (s_mode0_angle_ff_active == 0U))
        {
            roll_angle_pid.prev_sp = roll_angle_target;
            pitch_angle_pid.prev_sp = pitch_angle_target;
            roll_angle_pid.sp_rate = 0.0f;
            pitch_angle_pid.sp_rate = 0.0f;
            roll_angle_pid.ff_pt3_filter.state1 = 0.0f;
            roll_angle_pid.ff_pt3_filter.state2 = 0.0f;
            roll_angle_pid.ff_pt3_filter.state3 = 0.0f;
            pitch_angle_pid.ff_pt3_filter.state1 = 0.0f;
            pitch_angle_pid.ff_pt3_filter.state2 = 0.0f;
            pitch_angle_pid.ff_pt3_filter.state3 = 0.0f;
        }
        roll_angle_pid.kff = (mode0_angle_ff_enable != 0U) ? g_fc_params.roll_angle_kff : 0.0f;
        pitch_angle_pid.kff = (mode0_angle_ff_enable != 0U) ? g_fc_params.pitch_angle_kff : 0.0f;
        s_mode0_angle_ff_active = mode0_angle_ff_enable;
        roll_ctrl = fc_clampf(PID_Update(&roll_angle_pid, roll_angle_target, roll_angle_meas, dt), -limit, limit);
        pitch_ctrl = fc_clampf(PID_Update(&pitch_angle_pid, pitch_angle_target, pitch_angle_meas, dt), -limit, limit);

        /* 首次进入飞行态时复位 yaw 外环，后续 yaw 目标固定为 0 度 */
        if (0U == s_yaw_target_inited)
        {
            yaw_angle_target = yaw_angle_meas;
            PID_Reset(&yaw_angle_pid);
            s_yaw_target_inited = 1U;
        }

        /* Mode1/2/4/5/8 保留独立 yaw 目标，其余模式固定为 0 度。 */
        if ((s_flight_mode != FC_START_CRSF_FLIGHT_MODE_1) &&
            (s_flight_mode != FC_START_CRSF_FLIGHT_MODE_2) &&
            (s_flight_mode != FC_START_CRSF_FLIGHT_MODE_4) &&
            (s_flight_mode != FC_START_CRSF_FLIGHT_MODE_5) &&
            (s_flight_mode != FC_START_CRSF_FLIGHT_MODE_8))
        {
            yaw_angle_target = 0.0f;
        }

        /* 限制目标相对当前航向的误差，处理线缆拉扯自旋和 +/-180 度跨界跳变 */
        yaw_error_deg = FC_Wrap180Deg(yaw_angle_target - yaw_angle_meas);
        yaw_error_deg = fc_clampf(yaw_error_deg,
                                  -s_fc_yaw_target_delta_limit_deg,
                                  s_fc_yaw_target_delta_limit_deg);

        roll_gyro_target = roll_ctrl;
        pitch_gyro_target = pitch_ctrl;
        yaw_gyro_target = fc_clampf(PID_Update(&yaw_angle_pid, yaw_error_deg, 0.0f, dt),
                                    -s_fc_yaw_hold_rate_limit_dps,
                                    s_fc_yaw_hold_rate_limit_dps);
    }
    else
    {
        roll_angle_pid.kff = 0.0f;
        pitch_angle_pid.kff = 0.0f;
        s_mode0_angle_ff_active = 0U;
        s_yaw_target_inited = 0U;
        yaw_gyro_target = 0.0f;
    }

}

void FC_Loop_1000Hz(void)
{
    static uint32 tick_1000us_cnt_last = 0;
    uint32 tick_now = tick_1000us_cnt; // 读一次，缓存
    uint32 diff = tick_now - tick_1000us_cnt_last;
    float dt = diff * 0.001f; // 秒

    tick_1000us_cnt_last = tick_now;
    if ((FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING) ||
        (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_LANDING))
    {
        /* 读取当前角速度 */
        float roll_gyro_meas = g_imufilter_1000hz.gyrox;
        float pitch_gyro_meas = g_imufilter_1000hz.gyroy;
        float yaw_gyro_meas = g_imufilter_1000hz.gyroz;

        /* 控制量限幅 */
        float limit = 10000.0f;
        int32_t roll_ctrl = (int32_t)fc_clampf(PID_Update(&roll_gyro_pid, roll_gyro_target, roll_gyro_meas, dt), -limit, limit);
        int32_t pitch_ctrl = (int32_t)fc_clampf(PID_Update(&pitch_gyro_pid, pitch_gyro_target, pitch_gyro_meas, dt), -limit, limit);
        int32_t yaw_ctrl = (int32_t)fc_clampf(PID_Update(&yaw_gyro_pid, yaw_gyro_target, yaw_gyro_meas, dt),
                                              -s_fc_yaw_out_limit, s_fc_yaw_out_limit);
        /* 角速度环调试切换到 Pitch：目标、原始陀螺、滤波后陀螺、控制输出和 PID 分项 */
        //                         pitch_gyro_raw,这两个CSV文件是我离线标定的数据
        //                         pitch_gyro_meas,
        //                         pitch_ctrl,
        //                         pitch_gyro_pid.p_term,
        //                         pitch_gyro_pid.i_term,
        //                         pitch_gyro_pid.d_term,
        //                         pitch_gyro_pid.error,
        //                         8u);
        g_motor_cmd.throttle = (int32_t)fc_clampf(
            FC_Apply_Tilt_Throttle_Compensation(s_hover_throttle) + height_vel_out,
            1700.0f, 10000.0f);
        g_motor_cmd.roll = roll_ctrl;
        g_motor_cmd.pitch = -pitch_ctrl;
        g_motor_cmd.yaw = yaw_ctrl;

        Motor_Mixer(&g_motor_cmd);

    }

}
