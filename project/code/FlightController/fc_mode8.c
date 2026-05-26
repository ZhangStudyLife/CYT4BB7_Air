#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"

extern volatile float g_car_image_target_x;
extern volatile float g_car_image_target_y;
extern volatile float g_car_image_target_valid;

/* 模式8图像X位置环PID，输出X轴目标速度，单位cm/s */
static pid_t s_mode8_imgx_pid;
/* 模式8图像Y位置环PID，输出Y轴目标速度，单位cm/s */
static pid_t s_mode8_imgy_pid;
/* 模式8 X轴速度环PID，参数与模式2速度环一致 */
static pid_t s_mode8_velx_pid;
/* 模式8 Y轴速度环PID，参数与模式2速度环一致 */
static pid_t s_mode8_vely_pid;

/* 图像X轴目标像素坐标 */
static const float s_mode8_img_x_target = 94.0f;
/* 图像Y轴目标像素坐标 */
static const float s_mode8_img_y_target = 90.0f;
/* 图像环输出速度目标限幅，单位cm/s，与模式2速度目标保持一致 */
static const float s_mode8_vel_limit_cmps = 200.0f;
/* 速度环输出姿态角限幅，单位deg */
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

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        FC_Mode8_Reset();
        return;
    }

    if (g_car_image_target_valid > 0.5f)
    {
        velx_target = -PID_Update(&s_mode8_imgx_pid, s_mode8_img_x_target, g_car_image_target_x, dt);
        vely_target = -PID_Update(&s_mode8_imgy_pid, s_mode8_img_y_target, g_car_image_target_y, dt);
        velx_target = FC_Mode_Clamp(velx_target, -s_mode8_vel_limit_cmps, s_mode8_vel_limit_cmps);
        vely_target = FC_Mode_Clamp(vely_target, -s_mode8_vel_limit_cmps, s_mode8_vel_limit_cmps);
    }
    else
    {
        PID_Reset(&s_mode8_imgx_pid);
        PID_Reset(&s_mode8_imgy_pid);
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
