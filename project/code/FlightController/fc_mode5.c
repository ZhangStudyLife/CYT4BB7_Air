#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"

/* 模式5 X 轴速度环PID */
static pid_t s_mode5_velx_pid;
/* 模式5 Y 轴速度环PID */
static pid_t s_mode5_vely_pid;

/* 遥控到速度映射比例，1000 单位 -> 100 cm/s */
static const float s_mode5_rc_to_speed_scale = 0.1f;
/* 速度目标上限，单位cm/s */
static const float s_mode5_vel_limit_cmps = 100.0f;
/* 速度目标死区，单位cm/s */
static const float s_mode5_vel_deadzone_cmps = 6.0f;
/* 姿态角输出限幅，单位度 */
static const float s_mode5_angle_limit_deg = 20.0f;
/* 模式5固定高度目标，单位m */
static const float s_mode5_fixed_height_m = 1.0f;

/*
 * 函数名: FC_Mode5_ApplyDeadzone
 * 功能: 对速度目标施加对称死区，死区内归零，死区外缩减死区量
 * 输入参数:
 *   v  - 输入速度目标，单位cm/s
 *   dz - 死区宽度，单位cm/s
 * 返回值:
 *   经死区处理后的速度目标
 */
static float FC_Mode5_ApplyDeadzone(float v, float dz)
{
    if (v > dz)
    {
        return v - dz;
    }
    if (v < -dz)
    {
        return v + dz;
    }
    return 0.0f;
}

/*
 * 函数名: FC_Mode5_Init
 * 功能: 初始化模式5速度环PID
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode5_Init(void)
{
    PID_Init(&s_mode5_velx_pid,
             g_fc_params.vel_x_kp, g_fc_params.vel_x_ki, g_fc_params.vel_x_kd,
             g_fc_params.vel_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_x_i_limit, g_fc_params.vel_x_d_lpf);
    PID_Init(&s_mode5_vely_pid,
             g_fc_params.vel_y_kp, g_fc_params.vel_y_ki, g_fc_params.vel_y_kd,
             g_fc_params.vel_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_y_i_limit, g_fc_params.vel_y_d_lpf);
    FC_Mode5_Reset();
}

/*
 * 函数名: FC_Mode5_Reset
 * 功能: 复位模式5速度环状态和姿态目标
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode5_Reset(void)
{
    PID_Reset(&s_mode5_velx_pid);
    PID_Reset(&s_mode5_vely_pid);
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

/*
 * 函数名: FC_Mode5_100Hz
 * 功能: 模式5 100Hz 占位
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode5_100Hz(void)
{
}

/*
 * 函数名: FC_Mode5_50Hz
 * 功能: 模式5 50Hz 固定高度位置保持控制
 * 输入参数:
 *   dt - 本次调用周期，单位s
 * 返回值: 无
 */
void FC_Mode5_50Hz(float dt)
{
    float ch0;
    float ch1;
    float velx_target;
    float vely_target;
    float velx_out;
    float vely_out;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0], -1000.0f, 1000.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1], -1000.0f, 1000.0f);

    if ((-150.0f < ch0) && (ch0 < 150.0f) && (-150.0f < ch1) && (ch1 < 150.0f) && g_image_circles[0].valid == 1U)
    {
        float kp = 2.0f;
        velx_target = -g_image_circles[0].x * kp;
        vely_target = g_image_circles[0].y * kp;
    }
    else
    {
        velx_target = FC_Mode5_ApplyDeadzone(
            FC_Mode_Clamp(ch0 * s_mode5_rc_to_speed_scale,
                          -s_mode5_vel_limit_cmps, s_mode5_vel_limit_cmps),
            s_mode5_vel_deadzone_cmps);
        vely_target = FC_Mode5_ApplyDeadzone(
            FC_Mode_Clamp(-ch1 * s_mode5_rc_to_speed_scale,
                          -s_mode5_vel_limit_cmps, s_mode5_vel_limit_cmps),
            s_mode5_vel_deadzone_cmps);
    }

    velx_out = PID_Update(&s_mode5_velx_pid, velx_target, -Pos_Est_vel_x_kf, dt);
    vely_out = PID_Update(&s_mode5_vely_pid, vely_target, -Pos_Est_vel_y_kf, dt);

    velx_out = FC_Mode_Clamp(velx_out, -s_mode5_angle_limit_deg, s_mode5_angle_limit_deg);
    vely_out = FC_Mode_Clamp(vely_out, -s_mode5_angle_limit_deg, s_mode5_angle_limit_deg);

    roll_angle_target = velx_out + FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = vely_out + FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

/*
 * 函数名: FC_Mode5_Get_Fixed_Height_M
 * 功能: 获取模式5使用的固定高度目标
 * 输入参数: 无
 * 返回值:
 *   模式5固定高度目标，单位m
 */
float FC_Mode5_Get_Fixed_Height_M(void)
{
    return s_mode5_fixed_height_m;
}
