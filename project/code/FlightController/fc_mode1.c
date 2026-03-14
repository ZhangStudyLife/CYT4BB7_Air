#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"

/* 模式1 X 轴速度环 PID */
static pid_t s_mode1_velx_pid;
/* 模式1 Y 轴速度环 PID */
static pid_t s_mode1_vely_pid;
/* 模式1 遥控到速度映射比例，单位 cm/s 每遥控归一化单位 */
static const float s_mode1_rc_to_speed_scale = 0.1f;
/* 模式1 最大水平目标速度，单位 cm/s */
static const float s_mode1_vel_limit_cmps = 100.0f;
/* 模式1 最大姿态输出，单位度 */
static const float s_mode1_angle_limit_deg = 20.0f;

/*
 * 函数名: FC_Mode1_Init
 * 功能: 初始化模式1水平速度环资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_Init(void)
{
    PID_Init(&s_mode1_velx_pid,
             g_fc_params.vel_x_kp, g_fc_params.vel_x_ki, g_fc_params.vel_x_kd,
             g_fc_params.vel_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_x_i_limit, g_fc_params.vel_x_d_lpf);
    PID_Init(&s_mode1_vely_pid,
             g_fc_params.vel_y_kp, g_fc_params.vel_y_ki, g_fc_params.vel_y_kd,
             g_fc_params.vel_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_y_i_limit, g_fc_params.vel_y_d_lpf);
    FC_Mode1_Reset();
}

/*
 * 函数名: FC_Mode1_Reset
 * 功能: 清空模式1水平速度环状态，并将姿态目标拉回机械配平
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_Reset(void)
{
    PID_Reset(&s_mode1_velx_pid);
    PID_Reset(&s_mode1_vely_pid);
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

/*
 * 函数名: FC_Mode1_100Hz
 * 功能: 执行模式1的100Hz控制占位
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_100Hz(void)
{
}

/*
 * 函数名: FC_Mode1_50Hz
 * 功能: 在模式1下执行50Hz水平速度环，并输出姿态目标
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode1_50Hz(float dt)
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
    velx_target = FC_Mode_Clamp(ch0 * s_mode1_rc_to_speed_scale, -s_mode1_vel_limit_cmps, s_mode1_vel_limit_cmps);
    vely_target = FC_Mode_Clamp(-ch1 * s_mode1_rc_to_speed_scale, -s_mode1_vel_limit_cmps, s_mode1_vel_limit_cmps);

    velx_out = PID_Update(&s_mode1_velx_pid, velx_target, -Pos_Est_vel_x_kf, dt);
    vely_out = PID_Update(&s_mode1_vely_pid, vely_target, -Pos_Est_vel_y_kf, dt);

    velx_out = FC_Mode_Clamp(velx_out, -s_mode1_angle_limit_deg, s_mode1_angle_limit_deg);
    vely_out = FC_Mode_Clamp(vely_out, -s_mode1_angle_limit_deg, s_mode1_angle_limit_deg);

    roll_angle_target = velx_out + FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = vely_out + FC_Mode_Get_Pitch_Mech_Trim_Deg();
}
