#include "fc_mode.h"
#include "yaw_align.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../Planner/car_lamp_fused.h"
#include "../Planner/ProjectionCenter.h"
#include <math.h>

static const float s_mode4_two_pi = 6.283185307179586f; /* 模式4角频率换算常量。 */
static const float s_mode4_deg_to_rad = 0.017453292519943295f; /* 模式4角度转弧度系数。 */
static const float s_mode4_angle_limit_deg = 30.0f; /* 模式4横向姿态目标限幅，单位deg。 */

extern float g_car_vel_x;
extern float g_car_vel_y;
extern float g_car_vel_target_x;
extern float g_car_vel_target_y;
extern float g_car_yaw;
extern float g_car_yaw_rate_dps;
extern float g_car_large_turn_state;
extern float g_car_sync_time_ms;
extern uint32 g_car_last_update_time_ms;
extern volatile uint32 tick_1000us_cnt;

pid_t g_mode4_imgx_pid; /* 模式4图像X轴PD状态，供控制与调试访问。 */
pid_t g_mode4_imgy_pid; /* 模式4图像Y轴PD状态，供控制与调试访问。 */
float g_mode4_car_accel_angle_ff_x_deg = 0.0f; /* 模式4车加速度Roll前馈，单位deg。 */
float g_mode4_car_accel_angle_ff_y_deg = 0.0f; /* 模式4车加速度Pitch前馈，单位deg。 */
float g_mode4_car_accel_x_mps2 = 0.0f; /* 模式4投影到飞机X轴的车加速度，单位m/s^2。 */
float g_mode4_car_accel_y_mps2 = 0.0f; /* 模式4投影到飞机Y轴的车加速度，单位m/s^2。 */
float g_mode4_raw_roll_correction_deg = 0.0f; /* 模式4限幅前Roll修正，单位deg。 */
float g_mode4_raw_pitch_correction_deg = 0.0f; /* 模式4限幅前Pitch修正，单位deg。 */
float g_mode4_img_error_rate_x_pxps = 0.0f; /* 模式4图像X误差变化率，单位px/s。 */
float g_mode4_img_error_rate_y_pxps = 0.0f; /* 模式4图像Y误差变化率，单位px/s。 */
float g_mode4_car_dt_ms = 0.0f; /* 模式4车端同步时间间隔，单位ms。 */
float g_mode4_car_vel_error_x_mps = 0.0f; /* 模式4滤波后的车X速度误差，单位m/s。 */
float g_mode4_car_vel_error_y_mps = 0.0f; /* 模式4滤波后的车Y速度误差，单位m/s。 */
float g_mode4_car_body_accel_x_mps2 = 0.0f; /* 模式4车体系X加速度，单位m/s^2。 */
float g_mode4_car_body_accel_y_mps2 = 0.0f; /* 模式4车体系Y加速度，单位m/s^2。 */
float g_mode4_car_turn_accel_mps2 = 0.0f; /* 模式4滤波后的车转弯加速度，单位m/s^2。 */
float g_mode4_yaw_diff_deg = 0.0f; /* 模式4车机航向差，单位deg。 */
uint32 g_mode4_control_seq = 0U; /* 模式4控制更新序号。 */

static float s_mode4_prev_img_error_x = 0.0f; /* 模式4上次图像X误差。 */
static float s_mode4_prev_img_error_y = 0.0f; /* 模式4上次图像Y误差。 */
static float s_mode4_prev_car_sync_time_ms = 0.0f; /* 模式4上次车端同步时间，单位ms。 */
static uint8 s_mode4_image_initialized = 0U; /* 模式4图像微分历史有效标志。 */

/*
 * 函数名: FC_Mode4_Init
 * 功能: 初始化模式4图像PD控制器并复位控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode4_Init(void)
{
    PID_Init(&g_mode4_imgx_pid, g_fc_params.mode4_img_kp, 0.0f,
             g_fc_params.mode4_img_kd, 0.0f, g_fc_params.vel_xy_dt,
             0.0f, g_fc_params.mode4_img_d_lpf_hz);
    PID_Init(&g_mode4_imgy_pid, g_fc_params.mode4_img_kp, 0.0f,
             g_fc_params.mode4_img_kd, 0.0f, g_fc_params.vel_xy_dt,
             0.0f, g_fc_params.mode4_img_d_lpf_hz);
    FC_Mode4_Reset();
}

/*
 * 函数名: FC_Mode4_Reset
 * 功能: 清空模式4图像、车速前馈和航向控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode4_Reset(void)
{
    Beep_SetAlarm(BEEP_ALARM_MODE4_LAMP_LOST, 0U);
    PID_Reset(&g_mode4_imgx_pid);
    PID_Reset(&g_mode4_imgy_pid);
    g_mode4_car_accel_angle_ff_x_deg = g_mode4_car_accel_angle_ff_y_deg = 0.0f;
    g_mode4_car_accel_x_mps2 = g_mode4_car_accel_y_mps2 = 0.0f;
    g_mode4_raw_roll_correction_deg = g_mode4_raw_pitch_correction_deg = 0.0f;
    g_mode4_img_error_rate_x_pxps = g_mode4_img_error_rate_y_pxps = 0.0f;
    g_mode4_car_dt_ms = 0.0f;
    s_mode4_prev_img_error_x = s_mode4_prev_img_error_y = 0.0f;
    s_mode4_prev_car_sync_time_ms = 0.0f;
    g_mode4_car_vel_error_x_mps = g_mode4_car_vel_error_y_mps = 0.0f;
    g_mode4_car_body_accel_x_mps2 = g_mode4_car_body_accel_y_mps2 = 0.0f;
    g_mode4_car_turn_accel_mps2 = 0.0f;
    g_mode4_yaw_diff_deg = 0.0f;
    s_mode4_image_initialized = 0U;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    YawAlign_Reset();
    yaw_angle_target = 0.0f;
}

/*
 * 函数名: FC_Mode4_100Hz
 * 功能: 根据模式4独立航向控制模式更新 yaw 目标
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode4_100Hz(void)
{
    if(FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING)
    {
        YawAlign_Update(g_fc_params.yaw_change_mode4);
    }
    else
    {
        YawAlign_Reset();
        yaw_angle_target = 0.0f;
    }
}

/*
 * 函数名: FC_Mode4_Control100Hz
 * 功能: 使用图像PD与车加速度前馈计算模式4横向姿态目标
 * 输入参数:
 *   dt - 本次调用周期，单位s；非正值时使用飞控默认周期
 * 返回值: 无
 */
void FC_Mode4_Control100Hz(float dt)
{
    float img_err_x = 0.0f;
    float img_err_y = 0.0f;
    float alpha;
    float yaw_diff_rad;
    float yaw_cos;
    float yaw_sin;
    float car_accel_x_mps2;
    float car_accel_y_mps2;
    uint8 car_data_fresh;
    uint8 image_valid;

    if (dt <= 0.0f)
    {
        dt = g_fc_params.vel_xy_dt;
    }
    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        FC_Mode4_Reset();
        return;
    }

    image_valid = ((g_car_lamp_fused.valid != 0U) &&
                   (g_tof_fused_valid != 0U) &&
                   (g_tof_fused_height_mm > FC_MODE_IMAGE_MIN_HEIGHT_MM))
                      ? 1U
                      : 0U;
    Beep_SetAlarm(BEEP_ALARM_MODE4_LAMP_LOST,
                  (g_car_lamp_fused.valid == 0U) ? 1U : 0U);

    if (image_valid != 0U)
    {
        img_err_x = g_car_lamp_fused.cx - g_projection_center.cx;
        img_err_y = g_car_lamp_fused.cy - g_projection_center.cy;
        if (s_mode4_image_initialized != 0U)
        {
            alpha = s_mode4_two_pi * g_fc_params.mode4_img_d_lpf_hz * dt;
            alpha /= 1.0f + alpha;
            g_mode4_img_error_rate_x_pxps += alpha *
                ((img_err_x - s_mode4_prev_img_error_x) / dt -
                 g_mode4_img_error_rate_x_pxps);
            g_mode4_img_error_rate_y_pxps += alpha *
                ((img_err_y - s_mode4_prev_img_error_y) / dt -
                 g_mode4_img_error_rate_y_pxps);
        }
        else
        {
            s_mode4_image_initialized = 1U;
        }
        s_mode4_prev_img_error_x = img_err_x;
        s_mode4_prev_img_error_y = img_err_y;
        g_mode4_imgx_pid.error = img_err_x;
        g_mode4_imgy_pid.error = img_err_y;
        g_mode4_imgx_pid.p_term = g_fc_params.mode4_img_kp * img_err_x;
        g_mode4_imgy_pid.p_term = g_fc_params.mode4_img_kp * img_err_y;
        g_mode4_imgx_pid.d_term = g_fc_params.mode4_img_kd * g_mode4_img_error_rate_x_pxps;
        g_mode4_imgy_pid.d_term = g_fc_params.mode4_img_kd * g_mode4_img_error_rate_y_pxps;
        g_mode4_imgx_pid.output = g_mode4_imgx_pid.p_term + g_mode4_imgx_pid.d_term;
        g_mode4_imgy_pid.output = g_mode4_imgy_pid.p_term + g_mode4_imgy_pid.d_term;
    }
    else
    {
        PID_Reset(&g_mode4_imgx_pid);
        PID_Reset(&g_mode4_imgy_pid);
        g_mode4_img_error_rate_x_pxps = g_mode4_img_error_rate_y_pxps = 0.0f;
        s_mode4_prev_img_error_x = s_mode4_prev_img_error_y = 0.0f;
        s_mode4_image_initialized = 0U;
    }

    car_data_fresh = ((g_car_sync_time_ms > 0.0f) &&
                      ((tick_1000us_cnt - g_car_last_update_time_ms) <
                       FC_MODE_CAR_RUN_DATA_TIMEOUT_MS))
                         ? 1U
                         : 0U;
    if ((car_data_fresh != 0U) && (g_car_large_turn_state != 2.0f))
    {
        if (g_car_sync_time_ms != s_mode4_prev_car_sync_time_ms)
        {
            g_mode4_car_dt_ms = (s_mode4_prev_car_sync_time_ms > 0.0f)
                                    ? g_car_sync_time_ms - s_mode4_prev_car_sync_time_ms
                                    : 0.0f;
            s_mode4_prev_car_sync_time_ms = g_car_sync_time_ms;
        }

        alpha = s_mode4_two_pi * g_fc_params.mode4_car_vel_error_lpf_hz * dt;
        alpha /= 1.0f + alpha;
        g_mode4_car_vel_error_x_mps += alpha *
            (g_car_vel_target_x - g_car_vel_x - g_mode4_car_vel_error_x_mps);
        g_mode4_car_vel_error_y_mps += alpha *
            (g_car_vel_target_y - g_car_vel_y - g_mode4_car_vel_error_y_mps);
        car_accel_x_mps2 =
            ((g_mode4_car_vel_error_x_mps >= 0.0f)
                 ? g_fc_params.mode4_car_accel_gain_pos
                 : g_fc_params.mode4_car_accel_gain_neg) * g_mode4_car_vel_error_x_mps;
        car_accel_y_mps2 =
            ((g_mode4_car_vel_error_y_mps >= 0.0f)
                 ? g_fc_params.mode4_car_accel_gain_pos
                 : g_fc_params.mode4_car_accel_gain_neg) * g_mode4_car_vel_error_y_mps;
        g_mode4_car_body_accel_x_mps2 = car_accel_x_mps2;
        g_mode4_car_body_accel_y_mps2 = car_accel_y_mps2;

        /* 滤波车模向心加速度，再按实时车机航向差投影到Roll/Pitch。 */
        alpha = s_mode4_two_pi * g_fc_params.mode4_car_turn_accel_lpf_hz * dt;
        alpha /= 1.0f + alpha;
        g_mode4_car_turn_accel_mps2 += alpha *
            (g_car_vel_y * g_car_yaw_rate_dps * s_mode4_deg_to_rad -
             g_mode4_car_turn_accel_mps2);

        g_mode4_yaw_diff_deg = g_car_yaw - g_euler.yaw +
                               g_car_yaw_rate_dps * 0.12f;
        while (g_mode4_yaw_diff_deg > 180.0f)
        {
            g_mode4_yaw_diff_deg -= 360.0f;
        }
        while (g_mode4_yaw_diff_deg < -180.0f)
        {
            g_mode4_yaw_diff_deg += 360.0f;
        }
        yaw_diff_rad = g_mode4_yaw_diff_deg * s_mode4_deg_to_rad;
        yaw_cos = cosf(yaw_diff_rad);
        yaw_sin = sinf(yaw_diff_rad);
        g_mode4_car_accel_x_mps2 =
            car_accel_x_mps2 * yaw_cos + car_accel_y_mps2 * yaw_sin;
        g_mode4_car_accel_y_mps2 =
            car_accel_x_mps2 * yaw_sin - car_accel_y_mps2 * yaw_cos;
        g_mode4_car_accel_angle_ff_x_deg =
            g_fc_params.mode4_car_accel_ff * g_mode4_car_accel_x_mps2 +
            g_fc_params.mode4_car_turn_accel_ff * g_mode4_car_turn_accel_mps2 * yaw_cos;
        g_mode4_car_accel_angle_ff_y_deg =
            g_fc_params.mode4_car_accel_ff * g_mode4_car_accel_y_mps2 +
            g_fc_params.mode4_car_turn_accel_ff * g_mode4_car_turn_accel_mps2 * yaw_sin;
    }
    else
    {
        g_mode4_car_accel_angle_ff_x_deg = g_mode4_car_accel_angle_ff_y_deg = 0.0f;
        g_mode4_car_accel_x_mps2 = g_mode4_car_accel_y_mps2 = 0.0f;
        g_mode4_car_dt_ms = 0.0f;
        s_mode4_prev_car_sync_time_ms = 0.0f;
        g_mode4_car_vel_error_x_mps = g_mode4_car_vel_error_y_mps = 0.0f;
        g_mode4_car_body_accel_x_mps2 = g_mode4_car_body_accel_y_mps2 = 0.0f;
        g_mode4_car_turn_accel_mps2 = 0.0f;
        g_mode4_yaw_diff_deg = 0.0f;
    }

    g_mode4_raw_roll_correction_deg = g_mode4_imgx_pid.output +
                                      g_mode4_car_accel_angle_ff_x_deg;
    g_mode4_raw_pitch_correction_deg = g_mode4_imgy_pid.output +
                                       g_mode4_car_accel_angle_ff_y_deg;
    roll_angle_target = FC_Mode_Clamp(
        FC_Mode_Get_Roll_Mech_Trim_Deg() + g_mode4_raw_roll_correction_deg,
        -s_mode4_angle_limit_deg, s_mode4_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(
        FC_Mode_Get_Pitch_Mech_Trim_Deg() + g_mode4_raw_pitch_correction_deg,
        -s_mode4_angle_limit_deg, s_mode4_angle_limit_deg);
    g_mode4_control_seq++;
}
