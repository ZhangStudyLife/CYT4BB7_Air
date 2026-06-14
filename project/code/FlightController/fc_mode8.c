#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include <math.h>

extern volatile float g_down_camera_lamp_x;
extern volatile float g_down_camera_lamp_y;
extern volatile float g_down_camera_lamp_valid;

static pid_t s_mode8_imgx_pid;
static pid_t s_mode8_imgy_pid;
pid_t g_mode8_velx_pid;
pid_t g_mode8_vely_pid;
float g_mode8_velx_target = 0.0f;
float g_mode8_vely_target = 0.0f;

static const float s_mode8_img_x_dir = -1.0f; /* image x: right positive, left negative; target x: right positive */
static const float s_mode8_img_y_dir = 1.0f;  /* image y: front positive, rear negative; target y: rear positive */
static const float s_mode8_img_fb_limit_cmps = 200.0f;
static const float s_mode8_vel_limit_cmps = 200.0f;
static const float s_mode8_vel_accel_cmps2 = 250.0f;
static const float s_mode8_vel_jerk_cmps3 = 1800.0f;
static const float s_mode8_angle_limit_deg = 15.0f;
static float s_mode8_accel_x = 0.0f;
static float s_mode8_accel_y = 0.0f;

static void FC_Mode8_LimitVector(float *x, float *y, float limit)
{
    float mag = sqrtf((*x) * (*x) + (*y) * (*y));
    if (mag > limit)
    {
        float scale = limit / mag;
        *x *= scale;
        *y *= scale;
    }
}

void FC_Mode8_Init(void)
{
    PID_Init(&s_mode8_imgx_pid,
             g_fc_params.mode8_img_x_kp, g_fc_params.mode8_img_x_ki, g_fc_params.mode8_img_x_kd,
             g_fc_params.mode8_img_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.mode8_img_x_i_limit, g_fc_params.mode8_img_x_d_lpf);
    PID_Init(&s_mode8_imgy_pid,
             g_fc_params.mode8_img_y_kp, g_fc_params.mode8_img_y_ki, g_fc_params.mode8_img_y_kd,
             g_fc_params.mode8_img_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.mode8_img_y_i_limit, g_fc_params.mode8_img_y_d_lpf);
    PID_Init(&g_mode8_velx_pid,
             g_fc_params.mode8_vel_x_kp, g_fc_params.mode8_vel_x_ki, g_fc_params.mode8_vel_x_kd,
             0.0f, g_fc_params.vel_xy_dt,
             g_fc_params.mode8_vel_x_i_limit, g_fc_params.mode8_vel_x_d_lpf);
    PID_Init(&g_mode8_vely_pid,
             g_fc_params.mode8_vel_y_kp, g_fc_params.mode8_vel_y_ki, g_fc_params.mode8_vel_y_kd,
             0.0f, g_fc_params.vel_xy_dt,
             g_fc_params.mode8_vel_y_i_limit, g_fc_params.mode8_vel_y_d_lpf);
    g_mode8_velx_pid.aw_enable = 1U;
    g_mode8_velx_pid.aw_gain = 0.15f;
    g_mode8_vely_pid.aw_enable = 1U;
    g_mode8_vely_pid.aw_gain = 0.15f;
    FC_Mode8_Reset();
}

void FC_Mode8_Reset(void)
{
    PID_Reset(&s_mode8_imgx_pid);
    PID_Reset(&s_mode8_imgy_pid);
    PID_Reset(&g_mode8_velx_pid);
    PID_Reset(&g_mode8_vely_pid);
    g_mode8_velx_target = 0.0f;
    g_mode8_vely_target = 0.0f;
    s_mode8_accel_x = 0.0f;
    s_mode8_accel_y = 0.0f;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

void FC_Mode8_100Hz(void)
{
}

void FC_Mode8_50Hz(float dt)
{
    float velx_sp = 0.0f;
    float vely_sp = 0.0f;
    float accx_sp;
    float accy_sp;
    float velx_ff;
    float vely_ff;
    float velx_out;
    float vely_out;
    float img_err_x;
    float img_err_y;
    float img_fb_x;
    float img_fb_y;
    float roll_trim;
    float pitch_trim;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        Beep_Disable();
        FC_Mode8_Reset();
        return;
    }

    if (g_down_camera_lamp_valid > 0.5f)
    {
        Beep_Disable();
    }
    else
    {
        Beep_Enable();
    }

    if ((g_down_camera_lamp_valid > 0.5f) &&
        (0U != g_tof_fused_valid) &&
        (g_tof_fused_height_mm > 500.0f))
    {
        img_err_x = s_mode8_img_x_dir * g_down_camera_lamp_x;
        img_err_y = s_mode8_img_y_dir * g_down_camera_lamp_y;
        img_fb_x = PID_Update(&s_mode8_imgx_pid, 0.0f, -img_err_x, dt);
        img_fb_y = PID_Update(&s_mode8_imgy_pid, 0.0f, -img_err_y, dt);
        img_fb_x = FC_Mode_Clamp(img_fb_x, -s_mode8_img_fb_limit_cmps, s_mode8_img_fb_limit_cmps);
        img_fb_y = FC_Mode_Clamp(img_fb_y, -s_mode8_img_fb_limit_cmps, s_mode8_img_fb_limit_cmps);

        velx_sp = img_fb_x;
        vely_sp = img_fb_y;
    }
    else
    {
        PID_Reset(&s_mode8_imgx_pid);
        PID_Reset(&s_mode8_imgy_pid);
    }

    FC_Mode8_LimitVector(&velx_sp, &vely_sp, s_mode8_vel_limit_cmps);

    accx_sp = (velx_sp - g_mode8_velx_target) / dt;
    accy_sp = (vely_sp - g_mode8_vely_target) / dt;
    FC_Mode8_LimitVector(&accx_sp, &accy_sp, s_mode8_vel_accel_cmps2);
    accx_sp -= s_mode8_accel_x;
    accy_sp -= s_mode8_accel_y;
    FC_Mode8_LimitVector(&accx_sp, &accy_sp, s_mode8_vel_jerk_cmps3 * dt);
    s_mode8_accel_x += accx_sp;
    s_mode8_accel_y += accy_sp;
    g_mode8_velx_target += s_mode8_accel_x * dt;
    g_mode8_vely_target += s_mode8_accel_y * dt;

    if (((velx_sp - (g_mode8_velx_target - s_mode8_accel_x * dt)) * (velx_sp - g_mode8_velx_target) +
         (vely_sp - (g_mode8_vely_target - s_mode8_accel_y * dt)) * (vely_sp - g_mode8_vely_target)) <= 0.0f)
    {
        g_mode8_velx_target = velx_sp;
        g_mode8_vely_target = vely_sp;
        s_mode8_accel_x = 0.0f;
        s_mode8_accel_y = 0.0f;
    }

    roll_trim = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_trim = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    velx_ff = FC_Mode_Clamp(g_fc_params.mode8_vel_x_kff * s_mode8_accel_x,
                            -s_mode8_angle_limit_deg, s_mode8_angle_limit_deg);
    vely_ff = FC_Mode_Clamp(g_fc_params.mode8_vel_y_kff * s_mode8_accel_y,
                            -s_mode8_angle_limit_deg, s_mode8_angle_limit_deg);

    g_mode8_velx_pid.output_min = -s_mode8_angle_limit_deg - roll_trim - velx_ff;
    g_mode8_velx_pid.output_max = s_mode8_angle_limit_deg - roll_trim - velx_ff;
    g_mode8_vely_pid.output_min = -s_mode8_angle_limit_deg - pitch_trim - vely_ff;
    g_mode8_vely_pid.output_max = s_mode8_angle_limit_deg - pitch_trim - vely_ff;

    velx_out = PID_Update(&g_mode8_velx_pid, g_mode8_velx_target, -Pos_Est_vel_x, dt) + velx_ff;
    vely_out = PID_Update(&g_mode8_vely_pid, g_mode8_vely_target, -Pos_Est_vel_y, dt) + vely_ff;
    g_mode8_velx_pid.ff_term = velx_ff;
    g_mode8_vely_pid.ff_term = vely_ff;

    roll_angle_target = FC_Mode_Clamp(velx_out + roll_trim, -s_mode8_angle_limit_deg, s_mode8_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(vely_out + pitch_trim, -s_mode8_angle_limit_deg, s_mode8_angle_limit_deg);
}
