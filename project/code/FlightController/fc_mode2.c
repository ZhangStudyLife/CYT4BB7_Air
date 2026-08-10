#include "fc_mode.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../Planner/car_lamp_fused.h"
#include "../Planner/ProjectionCenter.h"
#include <math.h>

static const float s_mode2_two_pi = 6.283185307179586f;
static const float s_mode2_deg_to_rad = 0.017453292519943295f;
static const float s_mode2_img_kp = 0.14f;
static const float s_mode2_img_kd = 0.08f;
static const float s_mode2_img_d_lpf_hz = 1.2f;
static const float s_mode2_car_accel_ff = 5.0f;
static const float s_mode2_car_accel_lpf_hz = 0.9f;
static const float s_mode2_car_turn_accel_ff = 3.1f; /* 向心加速度前馈，单位 deg/(m/s^2) */
static const float s_mode2_car_turn_accel_lpf_hz = 1.8f; /* 向心加速度低通截止频率，单位 Hz */
static const float s_mode2_angle_limit_deg = 20.0f;

extern float g_car_vel_y;
extern float g_car_yaw;
extern float g_car_yaw_rate_dps;
extern float g_car_sync_time_ms;
extern uint32 g_car_last_update_time_ms;
extern volatile uint32 tick_1000us_cnt;

pid_t g_mode2_imgx_pid;
pid_t g_mode2_imgy_pid;
float g_mode2_car_accel_angle_ff_x_deg = 0.0f;
float g_mode2_car_accel_angle_ff_y_deg = 0.0f;
float g_mode2_car_accel_x_mps2 = 0.0f;
float g_mode2_car_accel_y_mps2 = 0.0f;
float g_mode2_raw_roll_correction_deg = 0.0f;
float g_mode2_raw_pitch_correction_deg = 0.0f;
float g_mode2_img_error_rate_x_pxps = 0.0f;
float g_mode2_img_error_rate_y_pxps = 0.0f;
float g_mode2_car_dt_ms = 0.0f;

static float s_mode2_prev_img_error_x = 0.0f;
static float s_mode2_prev_img_error_y = 0.0f;
static float s_mode2_prev_car_speed = 0.0f;
static float s_mode2_prev_car_sync_time_ms = 0.0f;
static float s_mode2_car_accel_mps2 = 0.0f;
static float s_mode2_car_turn_accel_mps2 = 0.0f; /* 滤波后的车模向心加速度，单位 m/s^2 */
static uint8 s_mode2_image_initialized = 0U;
static uint8 s_mode2_car_initialized = 0U;

void FC_Mode2_Init(void)
{
    PID_Init(&g_mode2_imgx_pid, s_mode2_img_kp, 0.0f, s_mode2_img_kd,
             0.0f, g_fc_params.vel_xy_dt, 0.0f, s_mode2_img_d_lpf_hz);
    PID_Init(&g_mode2_imgy_pid, s_mode2_img_kp, 0.0f, s_mode2_img_kd,
             0.0f, g_fc_params.vel_xy_dt, 0.0f, s_mode2_img_d_lpf_hz);
    FC_Mode2_Reset();
}

void FC_Mode2_Reset(void)
{
    Beep_SetAlarm(BEEP_ALARM_MODE2_LAMP_LOST, 0U);
    PID_Reset(&g_mode2_imgx_pid);
    PID_Reset(&g_mode2_imgy_pid);
    g_mode2_car_accel_angle_ff_x_deg = g_mode2_car_accel_angle_ff_y_deg = 0.0f;
    g_mode2_car_accel_x_mps2 = g_mode2_car_accel_y_mps2 = 0.0f;
    g_mode2_raw_roll_correction_deg = g_mode2_raw_pitch_correction_deg = 0.0f;
    g_mode2_img_error_rate_x_pxps = g_mode2_img_error_rate_y_pxps = 0.0f;
    g_mode2_car_dt_ms = 0.0f;
    s_mode2_prev_img_error_x = s_mode2_prev_img_error_y = 0.0f;
    s_mode2_prev_car_speed = s_mode2_prev_car_sync_time_ms = 0.0f;
    s_mode2_car_accel_mps2 = 0.0f;
    s_mode2_car_turn_accel_mps2 = 0.0f;
    s_mode2_image_initialized = s_mode2_car_initialized = 0U;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    yaw_angle_target = 0.0f;
}

void FC_Mode2_100Hz(void)
{
    yaw_angle_target = 0.0f;
}

void FC_Mode2_50Hz(float dt)
{
    float img_err_x = 0.0f;
    float img_err_y = 0.0f;
    float alpha;
    float yaw_diff_rad;
    float yaw_cos;
    float yaw_sin;
    float car_dt_s;
    uint8 car_data_fresh;
    uint8 image_valid;

    if (dt <= 0.0f)
    {
        dt = g_fc_params.vel_xy_dt;
    }
    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        FC_Mode2_Reset();
        return;
    }

    yaw_angle_target = 0.0f;
    image_valid = ((g_car_lamp_fused.valid != 0U) &&
                   (g_tof_fused_valid != 0U) &&
                   (g_tof_fused_height_mm > FC_MODE_IMAGE_MIN_HEIGHT_MM))
                      ? 1U
                      : 0U;
    Beep_SetAlarm(BEEP_ALARM_MODE2_LAMP_LOST,
                  (g_car_lamp_fused.valid == 0U) ? 1U : 0U);

    if (image_valid != 0U)
    {
        img_err_x = g_car_lamp_fused.cx - g_projection_center.cx;
        img_err_y = g_car_lamp_fused.cy - g_projection_center.cy;
        if (s_mode2_image_initialized != 0U)
        {
            alpha = s_mode2_two_pi * s_mode2_img_d_lpf_hz * dt;
            alpha /= 1.0f + alpha;
            g_mode2_img_error_rate_x_pxps += alpha *
                ((img_err_x - s_mode2_prev_img_error_x) / dt -
                 g_mode2_img_error_rate_x_pxps);
            g_mode2_img_error_rate_y_pxps += alpha *
                ((img_err_y - s_mode2_prev_img_error_y) / dt -
                 g_mode2_img_error_rate_y_pxps);
        }
        else
        {
            s_mode2_image_initialized = 1U;
        }
        s_mode2_prev_img_error_x = img_err_x;
        s_mode2_prev_img_error_y = img_err_y;
        g_mode2_imgx_pid.error = img_err_x;
        g_mode2_imgy_pid.error = img_err_y;
        g_mode2_imgx_pid.p_term = s_mode2_img_kp * img_err_x;
        g_mode2_imgy_pid.p_term = s_mode2_img_kp * img_err_y;
        g_mode2_imgx_pid.d_term = s_mode2_img_kd * g_mode2_img_error_rate_x_pxps;
        g_mode2_imgy_pid.d_term = s_mode2_img_kd * g_mode2_img_error_rate_y_pxps;
        g_mode2_imgx_pid.output = g_mode2_imgx_pid.p_term + g_mode2_imgx_pid.d_term;
        g_mode2_imgy_pid.output = g_mode2_imgy_pid.p_term + g_mode2_imgy_pid.d_term;
    }
    else
    {
        PID_Reset(&g_mode2_imgx_pid);
        PID_Reset(&g_mode2_imgy_pid);
        g_mode2_img_error_rate_x_pxps = g_mode2_img_error_rate_y_pxps = 0.0f;
        s_mode2_prev_img_error_x = s_mode2_prev_img_error_y = 0.0f;
        s_mode2_image_initialized = 0U;
    }

    car_data_fresh = ((g_car_sync_time_ms > 0.0f) &&
                      ((tick_1000us_cnt - g_car_last_update_time_ms) <
                       FC_MODE_CAR_RUN_DATA_TIMEOUT_MS))
                         ? 1U
                         : 0U;
    if (car_data_fresh != 0U)
    {
        if (g_car_sync_time_ms != s_mode2_prev_car_sync_time_ms)
        {
            g_mode2_car_dt_ms = g_car_sync_time_ms - s_mode2_prev_car_sync_time_ms;
            if ((s_mode2_car_initialized != 0U) &&
                (g_mode2_car_dt_ms > 0.0f) &&
                (g_mode2_car_dt_ms <= FC_MODE_CAR_RUN_DATA_TIMEOUT_MS))
            {
                car_dt_s = g_mode2_car_dt_ms * 0.001f;
                alpha = s_mode2_two_pi * s_mode2_car_accel_lpf_hz * car_dt_s;
                alpha /= 1.0f + alpha;
                s_mode2_car_accel_mps2 += alpha *
                    ((g_car_vel_y - s_mode2_prev_car_speed) / car_dt_s -
                     s_mode2_car_accel_mps2);
            }
            else
            {
                g_mode2_car_dt_ms = 0.0f;
                s_mode2_car_accel_mps2 = 0.0f;
                s_mode2_car_initialized = 1U;
            }
            s_mode2_prev_car_speed = g_car_vel_y;
            s_mode2_prev_car_sync_time_ms = g_car_sync_time_ms;
        }

        /* 滤波车模向心加速度，再按实时车机航向差投影到 Roll/Pitch。 */
        alpha = s_mode2_two_pi * s_mode2_car_turn_accel_lpf_hz * dt;
        alpha /= 1.0f + alpha;
        s_mode2_car_turn_accel_mps2 += alpha *
            (g_car_vel_y * g_car_yaw_rate_dps * s_mode2_deg_to_rad -
             s_mode2_car_turn_accel_mps2);

        yaw_diff_rad = g_car_yaw - g_euler.yaw;
        while (yaw_diff_rad > 180.0f)
        {
            yaw_diff_rad -= 360.0f;
        }
        while (yaw_diff_rad < -180.0f)
        {
            yaw_diff_rad += 360.0f;
        }
        yaw_diff_rad *= s_mode2_deg_to_rad;
        yaw_cos = cosf(yaw_diff_rad);
        yaw_sin = sinf(yaw_diff_rad);
        g_mode2_car_accel_x_mps2 = s_mode2_car_accel_mps2 * yaw_sin;
        g_mode2_car_accel_y_mps2 = -s_mode2_car_accel_mps2 * yaw_cos;
        g_mode2_car_accel_angle_ff_x_deg =
            s_mode2_car_accel_ff * g_mode2_car_accel_x_mps2 +
            s_mode2_car_turn_accel_ff * s_mode2_car_turn_accel_mps2 * yaw_cos;
        g_mode2_car_accel_angle_ff_y_deg =
            s_mode2_car_accel_ff * g_mode2_car_accel_y_mps2 +
            s_mode2_car_turn_accel_ff * s_mode2_car_turn_accel_mps2 * yaw_sin;
    }
    else
    {
        g_mode2_car_accel_angle_ff_x_deg = g_mode2_car_accel_angle_ff_y_deg = 0.0f;
        g_mode2_car_accel_x_mps2 = g_mode2_car_accel_y_mps2 = 0.0f;
        g_mode2_car_dt_ms = 0.0f;
        s_mode2_prev_car_speed = s_mode2_prev_car_sync_time_ms = 0.0f;
        s_mode2_car_accel_mps2 = 0.0f;
        s_mode2_car_turn_accel_mps2 = 0.0f;
        s_mode2_car_initialized = 0U;
    }

    g_mode2_raw_roll_correction_deg = g_mode2_imgx_pid.output +
                                      g_mode2_car_accel_angle_ff_x_deg;
    g_mode2_raw_pitch_correction_deg = g_mode2_imgy_pid.output +
                                       g_mode2_car_accel_angle_ff_y_deg;
    roll_angle_target = FC_Mode_Clamp(
        FC_Mode_Get_Roll_Mech_Trim_Deg() + g_mode2_raw_roll_correction_deg,
        -s_mode2_angle_limit_deg, s_mode2_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(
        FC_Mode_Get_Pitch_Mech_Trim_Deg() + g_mode2_raw_pitch_correction_deg,
        -s_mode2_angle_limit_deg, s_mode2_angle_limit_deg);
}

float FC_Mode2_Get_Fixed_Height_M(void)
{
    return 1.1f;
}
