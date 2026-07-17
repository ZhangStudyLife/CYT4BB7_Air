#include "fc_mode.h"
#include "yaw_align.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../Image/image_data.h"
#include "../Planner/car_lamp_fused.h"
#include "../Planner/pix_to_distance.h"
#include "../Planner/ProjectionCenter.h"
#include "../Protocols/wifi/wifi_justfloat/wifi_justfloat.h"
#include <math.h>

extern float g_car_vel_x;
extern float g_car_vel_y;
extern float g_car_yaw;
extern float g_car_yaw_rate_dps;
extern float g_car_sync_time_ms;
extern uint32 g_car_last_update_time_ms;
extern volatile uint32 tick_1000us_cnt;

pid_t g_mode3_imgx_pid;
pid_t g_mode3_imgy_pid;
pid_t g_mode3_velx_pid;
pid_t g_mode3_vely_pid;
float g_mode3_velx_target = 0.0f;
float g_mode3_vely_target = 0.0f;

static float s_mode3_prev_velx_target = 0.0f;
static float s_mode3_prev_vely_target = 0.0f;
static float s_mode3_velx_ff_lpf = 0.0f;
static float s_mode3_vely_ff_lpf = 0.0f;

void FC_Mode3_Init(void)
{
    PID_Init(&g_mode3_imgx_pid,
             g_fc_params.mode3_img_x_kp, g_fc_params.mode3_img_x_ki, g_fc_params.mode3_img_x_kd,
             g_fc_params.mode3_img_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.mode3_img_x_i_limit, g_fc_params.mode3_img_x_d_lpf);
    PID_Init(&g_mode3_imgy_pid,
             g_fc_params.mode3_img_y_kp, g_fc_params.mode3_img_y_ki, g_fc_params.mode3_img_y_kd,
             g_fc_params.mode3_img_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.mode3_img_y_i_limit, g_fc_params.mode3_img_y_d_lpf);
    PID_Init(&g_mode3_velx_pid,
             g_fc_params.mode3_vel_x_kp, g_fc_params.mode3_vel_x_ki, g_fc_params.mode3_vel_x_kd,
             0.0f, g_fc_params.vel_xy_dt,
             g_fc_params.mode3_vel_x_i_limit, g_fc_params.mode3_vel_x_d_lpf);
    PID_Init(&g_mode3_vely_pid,
             g_fc_params.mode3_vel_y_kp, g_fc_params.mode3_vel_y_ki, g_fc_params.mode3_vel_y_kd,
             0.0f, g_fc_params.vel_xy_dt,
             g_fc_params.mode3_vel_y_i_limit, g_fc_params.mode3_vel_y_d_lpf);
    g_mode3_velx_pid.aw_enable = 1U;
    g_mode3_velx_pid.aw_gain = 0.15f;
    g_mode3_vely_pid.aw_enable = 1U;
    g_mode3_vely_pid.aw_gain = 0.15f;
    FC_Mode3_Reset();
}
void FC_Mode3_Reset(void)
{
    Beep_SetAlarm(BEEP_ALARM_MODE3_LAMP_LOST, 0U);
    PID_Reset(&g_mode3_imgx_pid);
    PID_Reset(&g_mode3_imgy_pid);
    PID_Reset(&g_mode3_velx_pid);
    PID_Reset(&g_mode3_vely_pid);
    g_mode3_velx_target = 0.0f;
    g_mode3_vely_target = 0.0f;
    s_mode3_prev_velx_target = 0.0f;
    s_mode3_prev_vely_target = 0.0f;
    s_mode3_velx_ff_lpf = 0.0f;
    s_mode3_vely_ff_lpf = 0.0f;
    YawAlign_Reset();
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    yaw_angle_target = g_euler.yaw;
}
void FC_Mode3_100Hz(void)
{
    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        yaw_angle_target = 0.0f;
        return;
    }
    // wifi_justfloat(
    //     roll_angle_target, pitch_angle_target, yaw_angle_target,
    //     g_euler.roll, g_euler.pitch, g_euler.yaw,
    //     g_car_vel_x, g_car_vel_y, g_car_yaw, g_car_yaw_rate_dps,
    //     Pos_Est_vel_x, Pos_Est_vel_y,
    //     g_mode3_velx_target, g_mode3_vely_target,
    //     g_mode3_velx_pid.p_term, g_mode3_velx_pid.i_term,
    //     g_mode3_velx_pid.d_term, g_mode3_velx_pid.output,
    //     g_mode3_vely_pid.p_term, g_mode3_vely_pid.i_term,
    //     g_mode3_vely_pid.d_term, g_mode3_vely_pid.output,
    //     g_car_lamp_fused_distance_projectioncenter_2.x_cm, g_car_lamp_fused_distance_projectioncenter_2.y_cm,
    //     g_mode3_imgx_pid.output, g_mode3_imgy_pid.output,
    //     g_tof_fused_height_mm);
}

void FC_Mode3_50Hz(float dt)
{
    float velx_sp = 0.0f;
    float vely_sp = 0.0f;
    float velx_ff;
    float vely_ff;
    float velx_target_rate;
    float vely_target_rate;
    float velx_out;
    float vely_out;
    float img_err_x = 0.0f;
    float img_err_y = 0.0f;
    float img_fb_x = 0.0f;
    float img_fb_y = 0.0f;
    float roll_trim;
    float pitch_trim;
    float car_ff_x = 0.0f;
    float car_ff_y = 0.0f;
    float yaw_diff_rad;
    float yaw_cos;
    float yaw_sin;
    float car_turn_accel_x;
    float car_turn_accel_y;
    float turn_ff_x = 0.0f;
    float turn_ff_y = 0.0f;
    uint8_t car_data_fresh;
    uint8_t fused_lamp_valid;
    uint8_t tof_height_valid;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        FC_Mode3_Reset();
        return;
    }

    YawAlign_Update();

    fused_lamp_valid = g_car_lamp_fused_distance_projectioncenter_2.valid;

    Beep_SetAlarm(BEEP_ALARM_MODE3_LAMP_LOST,
                  (fused_lamp_valid == 0U) ? 1U : 0U);

    tof_height_valid = ((0U != g_tof_fused_valid) &&
                        (g_tof_fused_height_mm > FC_MODE_IMAGE_MIN_HEIGHT_MM))
                           ? 1U
                           : 0U;

    if ((fused_lamp_valid != 0U) && (tof_height_valid != 0U))
    {
        img_err_x = g_car_lamp_fused.cx - g_projection_center.cx;
        img_err_y = g_car_lamp_fused.cy - g_projection_center.cy;
        img_fb_x = PID_Update(&g_mode3_imgx_pid, 0.0f, -img_err_x, dt);
        img_fb_y = PID_Update(&g_mode3_imgy_pid, 0.0f, -img_err_y, dt);
        img_fb_x += img_err_x * fabsf(img_err_x) * g_fc_params.mode3_img_x_kp2 / 100.0f;
        img_fb_y += img_err_y * fabsf(img_err_y) * g_fc_params.mode3_img_y_kp2 / 100.0f;
        img_fb_x = FC_Mode_Clamp(img_fb_x, -FC_MODE_IMAGE_VEL_LIMIT_CMPS, FC_MODE_IMAGE_VEL_LIMIT_CMPS);
        img_fb_y = FC_Mode_Clamp(img_fb_y, -FC_MODE_IMAGE_VEL_LIMIT_CMPS, FC_MODE_IMAGE_VEL_LIMIT_CMPS);
    }
    else
    {
        PID_Reset(&g_mode3_imgx_pid);
        PID_Reset(&g_mode3_imgy_pid);
    }

    car_data_fresh = ((g_car_sync_time_ms > 0.0f) &&
                      ((tick_1000us_cnt - g_car_last_update_time_ms) < 200U))
                         ? 1U
                         : 0U;

    /* 将车模右/前速度旋转到飞机右/后控制坐标系，车端时间戳超时则不叠加。 */
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
        car_ff_x = (g_car_vel_x * yaw_cos + g_car_vel_y * yaw_sin) *
                   g_fc_params.mode3_kp_car_x;
        car_ff_y = (g_car_vel_x * yaw_sin - g_car_vel_y * yaw_cos) *
                   g_fc_params.mode3_kp_car_y;

        /* omega x velocity 给出车体系转弯加速度，再旋转为飞机 roll/pitch 前馈。 */
        car_turn_accel_x = g_car_yaw_rate_dps * 0.017453292519943295f * g_car_vel_y;
        car_turn_accel_y = -g_car_yaw_rate_dps * 0.017453292519943295f * g_car_vel_x;
        turn_ff_x = g_fc_params.mode3_turn_accel_ff_gain_x * 57.29577951308232f *
                    atanf((yaw_cos * car_turn_accel_x + yaw_sin * car_turn_accel_y) / 9.80665f);
        turn_ff_y = -g_fc_params.mode3_turn_accel_ff_gain_y * 57.29577951308232f *
                    atanf((-yaw_sin * car_turn_accel_x + yaw_cos * car_turn_accel_y) / 9.80665f);
        turn_ff_x = FC_Mode_Clamp(turn_ff_x, -g_fc_params.mode3_turn_accel_ff_limit_x_deg, g_fc_params.mode3_turn_accel_ff_limit_x_deg);
        turn_ff_y = FC_Mode_Clamp(turn_ff_y, -g_fc_params.mode3_turn_accel_ff_limit_y_deg, g_fc_params.mode3_turn_accel_ff_limit_y_deg);
    }
    velx_sp = img_fb_x + car_ff_x;
    vely_sp = img_fb_y + car_ff_y;
    // wifi_justfloat(g_car_vel_x, g_car_vel_y,
    //                img_fb_x, img_fb_y,
    //                velx_sp, vely_sp,
    //                g_mode3_velx_target, g_mode3_vely_target,
    //                roll_angle_target, pitch_angle_target);
    velx_target_rate = (velx_sp - s_mode3_prev_velx_target) / dt;
    vely_target_rate = (vely_sp - s_mode3_prev_vely_target) / dt;
    g_mode3_velx_target = velx_sp;
    g_mode3_vely_target = vely_sp;
    s_mode3_prev_velx_target = g_mode3_velx_target;
    s_mode3_prev_vely_target = g_mode3_vely_target;

    roll_trim = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_trim = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    /* 串口超时同时清空角度KFF滤波残量，避免断链瞬间产生反向前馈。 */
    if (car_data_fresh != 0U)
    {
        velx_ff = FC_Mode_Clamp(g_fc_params.mode3_vel_x_kff * velx_target_rate + turn_ff_x,
                                -angle_target_max, angle_target_max);
        vely_ff = FC_Mode_Clamp(g_fc_params.mode3_vel_y_kff * vely_target_rate + turn_ff_y,
                                -angle_target_max, angle_target_max);
        s_mode3_velx_ff_lpf += FC_MODE_VEL_KFF_LPF_ALPHA * (velx_ff - s_mode3_velx_ff_lpf);
        s_mode3_vely_ff_lpf += FC_MODE_VEL_KFF_LPF_ALPHA * (vely_ff - s_mode3_vely_ff_lpf);
        velx_ff = s_mode3_velx_ff_lpf;
        vely_ff = s_mode3_vely_ff_lpf;
    }
    else
    {
        s_mode3_velx_ff_lpf = 0.0f;
        s_mode3_vely_ff_lpf = 0.0f;
        velx_ff = 0.0f;
        vely_ff = 0.0f;
    }

    g_mode3_velx_pid.output_min = -angle_target_max - roll_trim - velx_ff;
    g_mode3_velx_pid.output_max = angle_target_max - roll_trim - velx_ff;
    g_mode3_vely_pid.output_min = -angle_target_max - pitch_trim - vely_ff;
    g_mode3_vely_pid.output_max = angle_target_max - pitch_trim - vely_ff;

    velx_out = PID_Update(&g_mode3_velx_pid, g_mode3_velx_target, -Pos_Est_vel_x_2, dt) + velx_ff;
    vely_out = PID_Update(&g_mode3_vely_pid, g_mode3_vely_target, -Pos_Est_vel_y_2, dt) + vely_ff;
    g_mode3_velx_pid.ff_term = velx_ff;
    g_mode3_vely_pid.ff_term = vely_ff;

    roll_angle_target = FC_Mode_Clamp(velx_out + roll_trim, -angle_target_max, angle_target_max);
    pitch_angle_target = FC_Mode_Clamp(vely_out + pitch_trim, -angle_target_max, angle_target_max);
 
}
