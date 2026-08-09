#include "fc_mode.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../Planner/car_lamp_fused.h"
#include "../Planner/ProjectionCenter.h"
#include <math.h>

static const float s_mode2_two_pi = 6.283185307179586f;
static const float s_mode2_deg_to_rad = 0.017453292519943295f;

/* 离线日志首飞参数：连续非线性像素PD。 */
static const float s_mode2_img_x_kc = 0.14f;
static const float s_mode2_img_x_kn = 0.08f;
static const float s_mode2_img_x_es_px = 50.0f;
static const float s_mode2_img_x_kd0 = 0.050f;
static const float s_mode2_img_y_kc = 0.16f;
static const float s_mode2_img_y_kn = 0.10f;
static const float s_mode2_img_y_es_px = 40.0f;
static const float s_mode2_img_y_kd0 = 0.055f;
static const float s_mode2_img_kd1 = 0.00010f;
static const float s_mode2_img_d_lpf_hz = 1.2f;
static const float s_mode2_img_d_limit_deg = 3.0f;

/* 车辆运动前馈：不做jerk预测，转向加速度只合并一次。 */
static const float s_mode2_car_vel_ff_deg_per_mps = 0.35f;
static const float s_mode2_car_accel_ff_deg_per_mps2 = 4.5f;
static const float s_mode2_car_accel_lpf_hz = 2.0f;
static const float s_mode2_car_accel_limit_mps2 = 4.0f;
static const float s_mode2_car_accel_ff_limit_deg = 6.0f;
static const float s_mode2_car_dt_max_ms = 200.0f;

/* 姿态意图治理：快速建立、可信刹车、同意图缓慢修整。 */
static const float s_mode2_angle_limit_deg = 20.0f;
static const float s_mode2_ref_attack_slew_dps = 220.0f;
static const float s_mode2_ref_brake_slew_dps = 240.0f;
static const float s_mode2_ref_trim_slew_dps = 55.0f;
static const float s_mode2_ref_deadband_deg = 1.25f;
static const float s_mode2_ref_reverse_hold_s = 0.06f;
static const float s_mode2_image_brake_product = -80.0f;
static const float s_mode2_car_brake_product = -0.15f;

extern float g_car_vel_x;
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

/* 仅供fc_loop飞行日志使用，不进入参数表。 */
float g_mode2_car_velocity_ff_x_deg = 0.0f;
float g_mode2_car_velocity_ff_y_deg = 0.0f;
float g_mode2_angle_limited_roll_deg = 0.0f;
float g_mode2_angle_limited_pitch_deg = 0.0f;
float g_mode2_final_reference_roll_deg = 0.0f;
float g_mode2_final_reference_pitch_deg = 0.0f;
float g_mode2_img_error_rate_x_pxps = 0.0f;
float g_mode2_img_error_rate_y_pxps = 0.0f;
float g_mode2_car_dt_ms = 0.0f;

static float s_mode2_prev_img_error_x = 0.0f;
static float s_mode2_prev_img_error_y = 0.0f;
static uint8 s_mode2_image_d_initialized = 0U;
static float s_mode2_prev_car_vel_body_x = 0.0f;
static float s_mode2_prev_car_vel_body_y = 0.0f;
static float s_mode2_prev_car_sync_time_ms = 0.0f;
static uint8 s_mode2_car_velocity_initialized = 0U;
static float s_mode2_accepted_roll_deg = 0.0f;
static float s_mode2_accepted_pitch_deg = 0.0f;
static float s_mode2_reverse_hold_x_s = 0.0f;
static float s_mode2_reverse_hold_y_s = 0.0f;

static void FC_Mode2_UpdateReference(float request_deg,
                                     float error_px,
                                     float error_rate_pxps,
                                     float car_vel_mps,
                                     float car_accel_mps2,
                                     float dt,
                                     float *accepted_deg,
                                     float *reference_deg,
                                     float *reverse_hold_s)
{
    float request_delta = request_deg - *accepted_deg;
    float reference_delta;
    float slew_dps;
    float max_step;
    uint8 clear_brake;
    uint8 opposite_request;

    clear_brake = (((error_px * error_rate_pxps) < s_mode2_image_brake_product) ||
                   ((car_vel_mps * car_accel_mps2) < s_mode2_car_brake_product))
                      ? 1U
                      : 0U;
    opposite_request = ((request_deg * *reference_deg) < 0.0f) ? 1U : 0U;

    if (fabsf(request_delta) >= s_mode2_ref_deadband_deg)
    {
        if ((opposite_request != 0U) && (clear_brake == 0U))
        {
            *reverse_hold_s += dt;
            if (*reverse_hold_s >= s_mode2_ref_reverse_hold_s)
            {
                *accepted_deg = request_deg;
                *reverse_hold_s = 0.0f;
            }
        }
        else
        {
            *accepted_deg = request_deg;
            *reverse_hold_s = 0.0f;
        }
    }
    else
    {
        *reverse_hold_s = 0.0f;
    }

    reference_delta = *accepted_deg - *reference_deg;
    if (((*accepted_deg * *reference_deg) < 0.0f) ||
        ((clear_brake != 0U) && (fabsf(*accepted_deg) < fabsf(*reference_deg))))
    {
        slew_dps = s_mode2_ref_brake_slew_dps;
    }
    else if (fabsf(*accepted_deg) > fabsf(*reference_deg))
    {
        slew_dps = s_mode2_ref_attack_slew_dps;
    }
    else
    {
        slew_dps = s_mode2_ref_trim_slew_dps;
    }

    max_step = slew_dps * dt;
    *reference_deg += FC_Mode_Clamp(reference_delta, -max_step, max_step);
}

void FC_Mode2_Init(void)
{
    PID_Init(&g_mode2_imgx_pid, s_mode2_img_x_kc, 0.0f, s_mode2_img_x_kd0,
             0.0f, g_fc_params.vel_xy_dt, 0.0f, s_mode2_img_d_lpf_hz);
    PID_Init(&g_mode2_imgy_pid, s_mode2_img_y_kc, 0.0f, s_mode2_img_y_kd0,
             0.0f, g_fc_params.vel_xy_dt, 0.0f, s_mode2_img_d_lpf_hz);
    FC_Mode2_Reset();
}

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
    g_mode2_car_velocity_ff_x_deg = 0.0f;
    g_mode2_car_velocity_ff_y_deg = 0.0f;
    g_mode2_angle_limited_roll_deg = 0.0f;
    g_mode2_angle_limited_pitch_deg = 0.0f;
    g_mode2_final_reference_roll_deg = 0.0f;
    g_mode2_final_reference_pitch_deg = 0.0f;
    g_mode2_img_error_rate_x_pxps = 0.0f;
    g_mode2_img_error_rate_y_pxps = 0.0f;
    g_mode2_car_dt_ms = 0.0f;
    s_mode2_prev_img_error_x = 0.0f;
    s_mode2_prev_img_error_y = 0.0f;
    s_mode2_image_d_initialized = 0U;
    s_mode2_prev_car_vel_body_x = 0.0f;
    s_mode2_prev_car_vel_body_y = 0.0f;
    s_mode2_prev_car_sync_time_ms = 0.0f;
    s_mode2_car_velocity_initialized = 0U;
    s_mode2_accepted_roll_deg = 0.0f;
    s_mode2_accepted_pitch_deg = 0.0f;
    s_mode2_reverse_hold_x_s = 0.0f;
    s_mode2_reverse_hold_y_s = 0.0f;
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
    float img_rate_raw_x;
    float img_rate_raw_y;
    float img_alpha;
    float d_gain_x;
    float d_gain_y;
    float yaw_diff_rad;
    float yaw_cos = 1.0f;
    float yaw_sin = 0.0f;
    float car_vel_x = 0.0f;
    float car_vel_y = 0.0f;
    float car_dt_s;
    float accel_tangent_body_x;
    float accel_tangent_body_y;
    float accel_total_body_x;
    float accel_total_body_y;
    float accel_raw_x;
    float accel_raw_y;
    float accel_alpha;
    float roll_trim;
    float pitch_trim;
    uint32 tick_now;
    uint8 car_data_fresh;
    uint8 fused_lamp_valid;
    uint8 tof_height_valid;

    if (dt <= 0.0f)
    {
        dt = g_fc_params.vel_xy_dt;
    }
    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        FC_Mode2_Reset();
        return;
    }

    tick_now = tick_1000us_cnt;
    yaw_angle_target = 0.0f;
    fused_lamp_valid = g_car_lamp_fused.valid;
    tof_height_valid = ((g_tof_fused_valid != 0U) &&
                        (g_tof_fused_height_mm > FC_MODE_IMAGE_MIN_HEIGHT_MM))
                           ? 1U
                           : 0U;
    Beep_SetAlarm(BEEP_ALARM_MODE2_LAMP_LOST,
                  (fused_lamp_valid == 0U) ? 1U : 0U);

    if ((fused_lamp_valid != 0U) && (tof_height_valid != 0U))
    {
        img_err_x = g_car_lamp_fused.cx - g_projection_center.cx;
        img_err_y = g_car_lamp_fused.cy - g_projection_center.cy;
        if (s_mode2_image_d_initialized == 0U)
        {
            s_mode2_prev_img_error_x = img_err_x;
            s_mode2_prev_img_error_y = img_err_y;
            g_mode2_img_error_rate_x_pxps = 0.0f;
            g_mode2_img_error_rate_y_pxps = 0.0f;
            s_mode2_image_d_initialized = 1U;
        }
        else
        {
            img_rate_raw_x = (img_err_x - s_mode2_prev_img_error_x) / dt;
            img_rate_raw_y = (img_err_y - s_mode2_prev_img_error_y) / dt;
            img_alpha = s_mode2_two_pi * s_mode2_img_d_lpf_hz * dt;
            img_alpha /= 1.0f + img_alpha;
            g_mode2_img_error_rate_x_pxps += img_alpha *
                (img_rate_raw_x - g_mode2_img_error_rate_x_pxps);
            g_mode2_img_error_rate_y_pxps += img_alpha *
                (img_rate_raw_y - g_mode2_img_error_rate_y_pxps);
            s_mode2_prev_img_error_x = img_err_x;
            s_mode2_prev_img_error_y = img_err_y;
        }

        d_gain_x = s_mode2_img_x_kd0 + s_mode2_img_kd1 * fabsf(img_err_x);
        d_gain_y = s_mode2_img_y_kd0 + s_mode2_img_kd1 * fabsf(img_err_y);
        g_mode2_imgx_pid.kp = s_mode2_img_x_kc;
        g_mode2_imgy_pid.kp = s_mode2_img_y_kc;
        g_mode2_imgx_pid.kd = d_gain_x;
        g_mode2_imgy_pid.kd = d_gain_y;
        g_mode2_imgx_pid.error = img_err_x;
        g_mode2_imgy_pid.error = img_err_y;
        g_mode2_imgx_pid.p_term = s_mode2_img_x_kc * img_err_x +
            s_mode2_img_x_kn * img_err_x * fabsf(img_err_x) /
            (s_mode2_img_x_es_px + fabsf(img_err_x));
        g_mode2_imgy_pid.p_term = s_mode2_img_y_kc * img_err_y +
            s_mode2_img_y_kn * img_err_y * fabsf(img_err_y) /
            (s_mode2_img_y_es_px + fabsf(img_err_y));
        g_mode2_imgx_pid.d_term = FC_Mode_Clamp(
            d_gain_x * g_mode2_img_error_rate_x_pxps,
            -s_mode2_img_d_limit_deg, s_mode2_img_d_limit_deg);
        g_mode2_imgy_pid.d_term = FC_Mode_Clamp(
            d_gain_y * g_mode2_img_error_rate_y_pxps,
            -s_mode2_img_d_limit_deg, s_mode2_img_d_limit_deg);
        g_mode2_imgx_pid.i_term = 0.0f;
        g_mode2_imgy_pid.i_term = 0.0f;
        g_mode2_imgx_pid.ff_term = 0.0f;
        g_mode2_imgy_pid.ff_term = 0.0f;
        g_mode2_imgx_pid.output = g_mode2_imgx_pid.p_term + g_mode2_imgx_pid.d_term;
        g_mode2_imgy_pid.output = g_mode2_imgy_pid.p_term + g_mode2_imgy_pid.d_term;
    }
    else
    {
        PID_Reset(&g_mode2_imgx_pid);
        PID_Reset(&g_mode2_imgy_pid);
        g_mode2_img_error_rate_x_pxps = 0.0f;
        g_mode2_img_error_rate_y_pxps = 0.0f;
        s_mode2_prev_img_error_x = 0.0f;
        s_mode2_prev_img_error_y = 0.0f;
        s_mode2_image_d_initialized = 0U;
    }

    car_data_fresh = ((g_car_sync_time_ms > 0.0f) &&
                      ((tick_now - g_car_last_update_time_ms) <
                       FC_MODE_CAR_RUN_DATA_TIMEOUT_MS))
                         ? 1U
                         : 0U;
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
        yaw_diff_rad *= s_mode2_deg_to_rad;
        yaw_cos = cosf(yaw_diff_rad);
        yaw_sin = sinf(yaw_diff_rad);
        car_vel_x = g_car_vel_x * yaw_cos + g_car_vel_y * yaw_sin;
        car_vel_y = g_car_vel_x * yaw_sin - g_car_vel_y * yaw_cos;
        g_mode2_car_velocity_ff_x_deg = car_vel_x * s_mode2_car_vel_ff_deg_per_mps;
        g_mode2_car_velocity_ff_y_deg = car_vel_y * s_mode2_car_vel_ff_deg_per_mps;

        if (g_car_sync_time_ms != s_mode2_prev_car_sync_time_ms)
        {
            g_mode2_car_dt_ms = g_car_sync_time_ms - s_mode2_prev_car_sync_time_ms;
            if ((s_mode2_car_velocity_initialized == 0U) ||
                (g_mode2_car_dt_ms <= 0.0f) ||
                (g_mode2_car_dt_ms > s_mode2_car_dt_max_ms))
            {
                g_mode2_car_dt_ms = 0.0f;
                g_mode2_car_accel_x_mps2 = 0.0f;
                g_mode2_car_accel_y_mps2 = 0.0f;
                s_mode2_car_velocity_initialized = 1U;
            }
            else
            {
                car_dt_s = g_mode2_car_dt_ms * 0.001f;
                accel_tangent_body_x =
                    (g_car_vel_x - s_mode2_prev_car_vel_body_x) / car_dt_s;
                accel_tangent_body_y =
                    (g_car_vel_y - s_mode2_prev_car_vel_body_y) / car_dt_s;
                accel_total_body_x = accel_tangent_body_x +
                    g_car_yaw_rate_dps * s_mode2_deg_to_rad * g_car_vel_y;
                accel_total_body_y = accel_tangent_body_y -
                    g_car_yaw_rate_dps * s_mode2_deg_to_rad * g_car_vel_x;
                accel_raw_x = accel_total_body_x * yaw_cos +
                              accel_total_body_y * yaw_sin;
                accel_raw_y = accel_total_body_x * yaw_sin -
                              accel_total_body_y * yaw_cos;
                accel_raw_x = FC_Mode_Clamp(accel_raw_x,
                                            -s_mode2_car_accel_limit_mps2,
                                            s_mode2_car_accel_limit_mps2);
                accel_raw_y = FC_Mode_Clamp(accel_raw_y,
                                            -s_mode2_car_accel_limit_mps2,
                                            s_mode2_car_accel_limit_mps2);
                accel_alpha = s_mode2_two_pi * s_mode2_car_accel_lpf_hz * car_dt_s;
                accel_alpha /= 1.0f + accel_alpha;
                g_mode2_car_accel_x_mps2 += accel_alpha *
                    (accel_raw_x - g_mode2_car_accel_x_mps2);
                g_mode2_car_accel_y_mps2 += accel_alpha *
                    (accel_raw_y - g_mode2_car_accel_y_mps2);
            }
            s_mode2_prev_car_vel_body_x = g_car_vel_x;
            s_mode2_prev_car_vel_body_y = g_car_vel_y;
            s_mode2_prev_car_sync_time_ms = g_car_sync_time_ms;
        }

        g_mode2_car_accel_angle_ff_x_deg = FC_Mode_Clamp(
            g_mode2_car_accel_x_mps2 * s_mode2_car_accel_ff_deg_per_mps2,
            -s_mode2_car_accel_ff_limit_deg, s_mode2_car_accel_ff_limit_deg);
        g_mode2_car_accel_angle_ff_y_deg = FC_Mode_Clamp(
            g_mode2_car_accel_y_mps2 * s_mode2_car_accel_ff_deg_per_mps2,
            -s_mode2_car_accel_ff_limit_deg, s_mode2_car_accel_ff_limit_deg);
    }
    else
    {
        g_mode2_car_velocity_ff_x_deg = 0.0f;
        g_mode2_car_velocity_ff_y_deg = 0.0f;
        g_mode2_car_accel_angle_ff_x_deg = 0.0f;
        g_mode2_car_accel_angle_ff_y_deg = 0.0f;
        g_mode2_car_accel_x_mps2 = 0.0f;
        g_mode2_car_accel_y_mps2 = 0.0f;
        g_mode2_car_dt_ms = 0.0f;
        s_mode2_prev_car_vel_body_x = 0.0f;
        s_mode2_prev_car_vel_body_y = 0.0f;
        s_mode2_prev_car_sync_time_ms = 0.0f;
        s_mode2_car_velocity_initialized = 0U;
    }

    g_mode2_raw_roll_correction_deg = g_mode2_imgx_pid.output +
                                      g_mode2_car_velocity_ff_x_deg +
                                      g_mode2_car_accel_angle_ff_x_deg;
    g_mode2_raw_pitch_correction_deg = g_mode2_imgy_pid.output +
                                       g_mode2_car_velocity_ff_y_deg +
                                       g_mode2_car_accel_angle_ff_y_deg;
    g_mode2_angle_limited_roll_deg = FC_Mode_Clamp(
        g_mode2_raw_roll_correction_deg,
        -s_mode2_angle_limit_deg, s_mode2_angle_limit_deg);
    g_mode2_angle_limited_pitch_deg = FC_Mode_Clamp(
        g_mode2_raw_pitch_correction_deg,
        -s_mode2_angle_limit_deg, s_mode2_angle_limit_deg);

    FC_Mode2_UpdateReference(g_mode2_angle_limited_roll_deg,
                             img_err_x,
                             g_mode2_img_error_rate_x_pxps,
                             car_vel_x,
                             g_mode2_car_accel_x_mps2,
                             dt,
                             &s_mode2_accepted_roll_deg,
                             &g_mode2_final_reference_roll_deg,
                             &s_mode2_reverse_hold_x_s);
    FC_Mode2_UpdateReference(g_mode2_angle_limited_pitch_deg,
                             img_err_y,
                             g_mode2_img_error_rate_y_pxps,
                             car_vel_y,
                             g_mode2_car_accel_y_mps2,
                             dt,
                             &s_mode2_accepted_pitch_deg,
                             &g_mode2_final_reference_pitch_deg,
                             &s_mode2_reverse_hold_y_s);

    roll_trim = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_trim = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    roll_angle_target = FC_Mode_Clamp(roll_trim + g_mode2_final_reference_roll_deg,
                                      -s_mode2_angle_limit_deg,
                                      s_mode2_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(pitch_trim + g_mode2_final_reference_pitch_deg,
                                       -s_mode2_angle_limit_deg,
                                       s_mode2_angle_limit_deg);
}

float FC_Mode2_Get_Fixed_Height_M(void)
{
    return 1.1f;
}
