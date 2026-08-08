#include "fc_mode.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../Planner/car_lamp_fused.h"
#include "../Planner/pix_to_distance.h"
#include "../Planner/ProjectionCenter.h"
#include <math.h>

extern float g_car_vel_x;
extern float g_car_vel_y;
extern float g_car_yaw;
extern float g_car_yaw_rate_dps;
extern float g_car_sync_time_ms;
extern uint32 g_car_last_update_time_ms;
extern volatile uint32 tick_1000us_cnt;

pid_t g_mode2_imgx_pid; /* 模式2图像X轴PD状态，输出Roll角度修正。 */
pid_t g_mode2_imgy_pid; /* 模式2图像Y轴PD状态，输出Pitch角度修正。 */
float g_mode2_car_accel_angle_ff_x_deg = 0.0f; /* 模式2车加速度前馈Roll角度，单位deg。 */
float g_mode2_car_accel_angle_ff_y_deg = 0.0f; /* 模式2车加速度前馈Pitch角度，单位deg。 */
float g_mode2_car_accel_x_mps2 = 0.0f; /* 模式2滤波车加速度X，单位m/s^2。 */
float g_mode2_car_accel_y_mps2 = 0.0f; /* 模式2滤波车加速度Y，单位m/s^2。 */
float g_mode2_raw_roll_correction_deg = 0.0f; /* 模式2限幅前Roll修正，单位deg。 */
float g_mode2_raw_pitch_correction_deg = 0.0f; /* 模式2限幅前Pitch修正，单位deg。 */

static float s_mode2_prev_car_vel_x = 0.0f;
static float s_mode2_prev_car_vel_y = 0.0f;
static float s_mode2_car_accel_lpf_x = 0.0f;
static float s_mode2_car_accel_lpf_y = 0.0f;
static float s_mode2_prev_roll_correction = 0.0f;
static float s_mode2_prev_pitch_correction = 0.0f;
static uint8_t s_mode2_car_velocity_initialized = 0U;

/*
 * 函数名: FC_Mode2_Init
 * 功能: 初始化模式2图像PD控制器。
 * 输入参数: 无。
 * 返回值: 无。
 */
void FC_Mode2_Init(void)
{
    PID_Init(&g_mode2_imgx_pid,
             g_fc_params.mode2_img_x_kp, 0.0f, g_fc_params.mode2_img_x_kd,
             0.0f, g_fc_params.vel_xy_dt, 0.0f, g_fc_params.mode2_img_x_d_lpf);
    PID_Init(&g_mode2_imgy_pid,
             g_fc_params.mode2_img_y_kp, 0.0f, g_fc_params.mode2_img_y_kd,
             0.0f, g_fc_params.vel_xy_dt, 0.0f, g_fc_params.mode2_img_y_d_lpf);
    FC_Mode2_Reset();
}

/*
 * 函数名: FC_Mode2_Reset
 * 功能: 复位模式2图像PD并恢复机械中值目标。
 * 输入参数: 无。
 * 返回值: 无。
 */
void FC_Mode2_Reset(void)
{
    Beep_SetAlarm(BEEP_ALARM_MODE2_LAMP_LOST, 0U);
    PID_Reset(&g_mode2_imgx_pid);
    PID_Reset(&g_mode2_imgy_pid);
    g_mode2_car_accel_angle_ff_x_deg = 0.0f;
    g_mode2_car_accel_angle_ff_y_deg = 0.0f;
    g_mode2_car_accel_x_mps2 = 0.0f;
    g_mode2_car_accel_y_mps2 = 0.0f;
    g_mode2_raw_roll_correction_deg = 0.0f;
    g_mode2_raw_pitch_correction_deg = 0.0f;
    s_mode2_prev_car_vel_x = 0.0f;
    s_mode2_prev_car_vel_y = 0.0f;
    s_mode2_car_accel_lpf_x = 0.0f;
    s_mode2_car_accel_lpf_y = 0.0f;
    s_mode2_prev_roll_correction = 0.0f;
    s_mode2_prev_pitch_correction = 0.0f;
    s_mode2_car_velocity_initialized = 0U;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    yaw_angle_target = 0.0f;
}

/*
 * 函数名: FC_Mode2_100Hz
 * 功能: 将模式2的Yaw目标固定为0度。
 * 输入参数: 无。
 * 返回值: 无。
 */
void FC_Mode2_100Hz(void)
{
    yaw_angle_target = 0.0f;
}

/*
 * 函数名: FC_Mode2_50Hz
 * 功能: 用图像PD、姿态反馈和车模运动前馈生成Roll/Pitch目标。
 * 输入参数:
 *   dt - 本次调用周期，单位s。
 * 返回值: 无。
 */
void FC_Mode2_50Hz(float dt)
{
    float img_err_x = 0.0f;
    float img_err_y = 0.0f;
    float img_angle_x = 0.0f;
    float img_angle_y = 0.0f;
    float car_vel_ff_x = 0.0f;
    float car_vel_ff_y = 0.0f;
    float car_vel_x = 0.0f;
    float car_vel_y = 0.0f;
    float car_accel_ff_x = 0.0f;
    float car_accel_ff_y = 0.0f;
    float turn_ff_x = 0.0f;
    float turn_ff_y = 0.0f;
    float attitude_fb_x;
    float attitude_fb_y;
    float accel_alpha;
    float max_angle_step;
    float yaw_diff_rad;
    float yaw_cos;
    float yaw_sin;
    float car_turn_accel_x;
    float car_turn_accel_y;
    float roll_correction;
    float pitch_correction;
    float roll_trim;
    float pitch_trim;
    uint8_t car_data_fresh;
    uint8_t fused_lamp_valid;
    uint8_t tof_height_valid;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        FC_Mode2_Reset();
        return;
    }

    yaw_angle_target = 0.0f;
    fused_lamp_valid = g_car_lamp_fused_distance_projectioncenter_2.valid;
    tof_height_valid = ((0U != g_tof_fused_valid) &&
                        (g_tof_fused_height_mm > FC_MODE_IMAGE_MIN_HEIGHT_MM))
                           ? 1U
                           : 0U;
    Beep_SetAlarm(BEEP_ALARM_MODE2_LAMP_LOST,
                  (fused_lamp_valid == 0U) ? 1U : 0U);

    /* 图像PD直接输出角度修正，图像或高度无效时复位D项状态。 */
    if ((fused_lamp_valid != 0U) && (tof_height_valid != 0U))
    {
        img_err_x = g_car_lamp_fused.cx - g_projection_center.cx;
        img_err_y = g_car_lamp_fused.cy - g_projection_center.cy;
        img_angle_x = PID_Update(&g_mode2_imgx_pid, 0.0f, -img_err_x, dt);
        img_angle_y = PID_Update(&g_mode2_imgy_pid, 0.0f, -img_err_y, dt);
    }
    else
    {
        PID_Reset(&g_mode2_imgx_pid);
        PID_Reset(&g_mode2_imgy_pid);
    }

    car_data_fresh = ((g_car_sync_time_ms > 0.0f) &&
                      ((tick_1000us_cnt - g_car_last_update_time_ms) < 200U))
                         ? 1U
                         : 0U;

    /* 车端数据新鲜时，将车速和转向加速度旋转为飞机角度前馈。 */
    if (car_data_fresh != 0U)
    {
        yaw_diff_rad = g_car_yaw - g_euler.yaw;
        while (yaw_diff_rad > 180.0f)
        {
            yaw_diff_rad -= 360.0f;
        }
        while (yaw_diff_rad < -180.0f)
        {
            yaw_diff_rad += 360.0f;
        }
        yaw_diff_rad *= 0.017453292519943295f;
        yaw_cos = cosf(yaw_diff_rad);
        yaw_sin = sinf(yaw_diff_rad);

        car_vel_x = g_car_vel_x * yaw_cos + g_car_vel_y * yaw_sin;
        car_vel_y = g_car_vel_x * yaw_sin - g_car_vel_y * yaw_cos;
        car_vel_ff_x = car_vel_x * g_fc_params.mode2_car_vel_ff_x_deg_per_mps;
        car_vel_ff_y = car_vel_y * g_fc_params.mode2_car_vel_ff_y_deg_per_mps;

        if (s_mode2_car_velocity_initialized == 0U)
        {
            s_mode2_prev_car_vel_x = car_vel_x;
            s_mode2_prev_car_vel_y = car_vel_y;
            s_mode2_car_velocity_initialized = 1U;
        }
        else
        {
            accel_alpha = 6.283185307179586f * g_fc_params.mode2_car_accel_lpf_hz * dt;
            accel_alpha = accel_alpha / (1.0f + accel_alpha);
            s_mode2_car_accel_lpf_x += accel_alpha *
                                            ((car_vel_x - s_mode2_prev_car_vel_x) / dt - s_mode2_car_accel_lpf_x);
            s_mode2_car_accel_lpf_y += accel_alpha *
                                            ((car_vel_y - s_mode2_prev_car_vel_y) / dt - s_mode2_car_accel_lpf_y);
            s_mode2_prev_car_vel_x = car_vel_x;
            s_mode2_prev_car_vel_y = car_vel_y;
        }
        car_accel_ff_x = FC_Mode_Clamp(s_mode2_car_accel_lpf_x * g_fc_params.mode2_car_accel_ff_x_deg_per_mps2,
                                       -g_fc_params.mode2_car_accel_ff_limit_deg,
                                       g_fc_params.mode2_car_accel_ff_limit_deg);
        car_accel_ff_y = FC_Mode_Clamp(s_mode2_car_accel_lpf_y * g_fc_params.mode2_car_accel_ff_y_deg_per_mps2,
                                       -g_fc_params.mode2_car_accel_ff_limit_deg,
                                       g_fc_params.mode2_car_accel_ff_limit_deg);

        car_turn_accel_x = g_car_yaw_rate_dps * 0.017453292519943295f * g_car_vel_y;
        car_turn_accel_y = -g_car_yaw_rate_dps * 0.017453292519943295f * g_car_vel_x;
        turn_ff_x = g_fc_params.mode2_turn_accel_ff_gain_x * 57.29577951308232f *
                    atanf((yaw_cos * car_turn_accel_x + yaw_sin * car_turn_accel_y) / 9.80665f);
        turn_ff_y = -g_fc_params.mode2_turn_accel_ff_gain_y * 57.29577951308232f *
                    atanf((-yaw_sin * car_turn_accel_x + yaw_cos * car_turn_accel_y) / 9.80665f);
    }
    else
    {
        s_mode2_car_velocity_initialized = 0U;
        s_mode2_car_accel_lpf_x = 0.0f;
        s_mode2_car_accel_lpf_y = 0.0f;
    }

    roll_trim = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_trim = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    attitude_fb_x = g_fc_params.mode2_attitude_fb_gain * (g_euler.roll - roll_trim);
    attitude_fb_y = g_fc_params.mode2_attitude_fb_gain * (g_euler.pitch - pitch_trim);
    g_mode2_car_accel_x_mps2 = s_mode2_car_accel_lpf_x;
    g_mode2_car_accel_y_mps2 = s_mode2_car_accel_lpf_y;
    g_mode2_raw_roll_correction_deg = img_angle_x + attitude_fb_x + car_vel_ff_x + car_accel_ff_x + turn_ff_x;
    g_mode2_raw_pitch_correction_deg = img_angle_y + attitude_fb_y + car_vel_ff_y + car_accel_ff_y + turn_ff_y;
    roll_correction = FC_Mode_Clamp(g_mode2_raw_roll_correction_deg,
                                    -g_fc_params.mode2_angle_limit_deg,
                                    g_fc_params.mode2_angle_limit_deg);
    pitch_correction = FC_Mode_Clamp(g_mode2_raw_pitch_correction_deg,
                                     -g_fc_params.mode2_angle_limit_deg,
                                     g_fc_params.mode2_angle_limit_deg);
    max_angle_step = g_fc_params.mode2_angle_slew_dps * dt;
    roll_correction = FC_Mode_Clamp(roll_correction,
                                    s_mode2_prev_roll_correction - max_angle_step,
                                    s_mode2_prev_roll_correction + max_angle_step);
    pitch_correction = FC_Mode_Clamp(pitch_correction,
                                     s_mode2_prev_pitch_correction - max_angle_step,
                                     s_mode2_prev_pitch_correction + max_angle_step);
    s_mode2_prev_roll_correction = roll_correction;
    s_mode2_prev_pitch_correction = pitch_correction;
    roll_angle_target = FC_Mode_Clamp(roll_trim + roll_correction,
                                      -angle_target_max, angle_target_max);
    pitch_angle_target = FC_Mode_Clamp(pitch_trim + pitch_correction,
                                       -angle_target_max, angle_target_max);
    g_mode2_car_accel_angle_ff_x_deg = car_accel_ff_x;
    g_mode2_car_accel_angle_ff_y_deg = car_accel_ff_y;
}

/*
 * 函数名: FC_Mode2_Get_Fixed_Height_M
 * 功能: 返回模式2固定飞行高度。
 * 输入参数: 无。
 * 返回值: 固定高度，单位m。
 */
float FC_Mode2_Get_Fixed_Height_M(void)
{
    return 1.1f;
}
