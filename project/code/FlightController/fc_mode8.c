#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../Estimation/Height_Est/Height_Est.h"

extern volatile float g_car_image_target_x;
extern volatile float g_car_image_target_y;
extern volatile float g_car_image_target_valid;
extern volatile float g_car_velocity_strafe_mps;
extern volatile float g_car_velocity_forward_mps;

static pid_t s_mode8_imgx_pid;
static pid_t s_mode8_imgy_pid;
static pid_t s_mode8_velx_pid;
static pid_t s_mode8_vely_pid;
static float s_mode8_img_err_x_lpf = 0.0f;
static float s_mode8_img_err_y_lpf = 0.0f;
static uint8 s_mode8_img_lpf_inited = 0U;

static const float s_mode8_img_x_target = 94.0f;
static const float s_mode8_img_y_target = 70.0f;
static const float s_mode8_img_lpf_alpha = 0.45f;
static const float s_mode8_img_fb_limit_cmps = 80.0f;
static const float s_mode8_car_vel_ff = 1.0f;
static const float s_mode8_vel_limit_cmps = 200.0f;
static const float s_mode8_angle_limit_deg = 8.0f;

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
    PID_Init(&s_mode8_velx_pid,
             g_fc_params.vel_x_kp, g_fc_params.vel_x_ki, g_fc_params.vel_x_kd,
             g_fc_params.vel_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_x_i_limit, g_fc_params.vel_x_d_lpf);
    PID_Init(&s_mode8_vely_pid,
             g_fc_params.vel_y_kp, g_fc_params.vel_y_ki, g_fc_params.vel_y_kd,
             g_fc_params.vel_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_y_i_limit, g_fc_params.vel_y_d_lpf);
    FC_Mode8_Reset();
}

void FC_Mode8_Reset(void)
{
    PID_Reset(&s_mode8_imgx_pid);
    PID_Reset(&s_mode8_imgy_pid);
    PID_Reset(&s_mode8_velx_pid);
    PID_Reset(&s_mode8_vely_pid);
    s_mode8_img_err_x_lpf = 0.0f;
    s_mode8_img_err_y_lpf = 0.0f;
    s_mode8_img_lpf_inited = 0U;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

void FC_Mode8_100Hz(void)
{
}

void FC_Mode8_50Hz(float dt)
{
    float velx_target;
    float vely_target;
    float velx_out;
    float vely_out;
    float img_fb_x;
    float img_fb_y;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        FC_Mode8_Reset();
        return;
    }

    if ((g_car_image_target_valid > 0.5f) &&
        (0U != g_tof_fused_valid) &&
        (g_tof_fused_height_mm > 500.0f))
    {
        if (0U == s_mode8_img_lpf_inited)
        {
            s_mode8_img_err_x_lpf = g_car_image_target_x - s_mode8_img_x_target;
            s_mode8_img_err_y_lpf = g_car_image_target_y - s_mode8_img_y_target;
            s_mode8_img_lpf_inited = 1U;
            PID_Reset(&s_mode8_imgx_pid);
            PID_Reset(&s_mode8_imgy_pid);
        }

        s_mode8_img_err_x_lpf += s_mode8_img_lpf_alpha *
                                  ((g_car_image_target_x - s_mode8_img_x_target) - s_mode8_img_err_x_lpf);
        s_mode8_img_err_y_lpf += s_mode8_img_lpf_alpha *
                                  ((g_car_image_target_y - s_mode8_img_y_target) - s_mode8_img_err_y_lpf);

        img_fb_x = PID_Update(&s_mode8_imgx_pid, 0.0f, -s_mode8_img_err_x_lpf, dt);
        img_fb_y = PID_Update(&s_mode8_imgy_pid, 0.0f, -s_mode8_img_err_y_lpf, dt);
        img_fb_x = FC_Mode_Clamp(img_fb_x, -s_mode8_img_fb_limit_cmps, s_mode8_img_fb_limit_cmps);
        img_fb_y = FC_Mode_Clamp(img_fb_y, -s_mode8_img_fb_limit_cmps, s_mode8_img_fb_limit_cmps);

        velx_target = s_mode8_car_vel_ff * g_car_velocity_strafe_mps * 100.0f + img_fb_x;
        vely_target = -s_mode8_car_vel_ff * g_car_velocity_forward_mps * 100.0f + img_fb_y;
        velx_target = FC_Mode_Clamp(velx_target, -s_mode8_vel_limit_cmps, s_mode8_vel_limit_cmps);
        vely_target = FC_Mode_Clamp(vely_target, -s_mode8_vel_limit_cmps, s_mode8_vel_limit_cmps);
    }
    else
    {
        PID_Reset(&s_mode8_imgx_pid);
        PID_Reset(&s_mode8_imgy_pid);
        s_mode8_img_err_x_lpf = 0.0f;
        s_mode8_img_err_y_lpf = 0.0f;
        s_mode8_img_lpf_inited = 0U;
        velx_target = 0.0f;
        vely_target = 0.0f;
    }

    velx_out = PID_Update(&s_mode8_velx_pid, velx_target, -Pos_Est_vel_x, dt);
    vely_out = PID_Update(&s_mode8_vely_pid, vely_target, -Pos_Est_vel_y, dt);

    roll_angle_target = FC_Mode_Clamp(velx_out + FC_Mode_Get_Roll_Mech_Trim_Deg(),
                                      -s_mode8_angle_limit_deg, s_mode8_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(vely_out + FC_Mode_Get_Pitch_Mech_Trim_Deg(),
                                       -s_mode8_angle_limit_deg, s_mode8_angle_limit_deg);
}
