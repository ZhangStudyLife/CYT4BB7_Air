#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"

/* 模式1 X 轴速度环 PID */
static pid_t s_mode1_velx_pid;
/* 模式1 Y 轴速度环 PID */
static pid_t s_mode1_vely_pid;

/* 遥控到速度映射比例：1000 单位 -> 100 cm/s */
static const float s_mode1_rc_to_speed_scale = 0.1f;
/* 速度目标上限，单位 cm/s */
static const float s_mode1_vel_limit_cmps = 100.0f;
/* 速度目标死区，单位 cm/s */
static const float s_mode1_vel_deadzone_cmps = 6.0f;
/* 姿态角输出限幅，单位度 */
static const float s_mode1_angle_limit_deg = 20.0f;

/*
 * 函数名: FC_Mode1_ApplyDeadzone
 * 功能: 对速度目标施加对称死区，死区内归零，死区外缩减死区量
 * 输入参数:
 *   v  - 输入速度目标，单位 cm/s
 *   dz - 死区宽度，单位 cm/s
 * 返回值:
 *   经过死区处理后的速度目标
 */
static float FC_Mode1_ApplyDeadzone(float v, float dz)
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
 * 函数名: FC_Mode1_Init
 * 功能: 初始化模式1速度环 PID
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
 * 功能: 复位模式1速度环状态和姿态目标
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
 * 功能: 模式1 100Hz 占位
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_100Hz(void)
{
}

/*
 * 函数名: FC_Mode1_50Hz
 * 功能: 模式1 50Hz 纯速度 PI 控制
 * 输入参数:
 *   dt - 本次调用周期，单位 s
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

    if ((-150.0f < ch0) && (ch0 < 150.0f) && (-150.0f < ch1) && (ch1 < 150.0f) && 0U)
    {
        // 遥控器没怎么给多大的速度指令，并且图像圆形目标有效，此时可以认为是在视觉引导下悬停，直接把速度目标设为0，让 PID 输出稳定的姿态控制指令去追踪目标
        ch0 = 0.0f;
        ch1 = 0.0f;
        // g_image_circle.x > 0 表示飞机在信标灯右边 需要往左飞 
        // Pos_Est_vel_x_kf 单位 cm/s，往左飞为正，往右飞为负
        velx_target = 0.0f;
        // g_image_circle.y > 0 表示飞机在信标灯前方 需要往后飞
        // Pos_Est_vel_y_kf 单位 cm/s，往前飞为正，往后飞为负
        vely_target = 0.0f;
    }
    else
    {
        velx_target = FC_Mode1_ApplyDeadzone(
            FC_Mode_Clamp(ch0 * s_mode1_rc_to_speed_scale,
                          -s_mode1_vel_limit_cmps, s_mode1_vel_limit_cmps),
            s_mode1_vel_deadzone_cmps);
        vely_target = FC_Mode1_ApplyDeadzone(
            FC_Mode_Clamp(-ch1 * s_mode1_rc_to_speed_scale,
                          -s_mode1_vel_limit_cmps, s_mode1_vel_limit_cmps),
            s_mode1_vel_deadzone_cmps);
    }

    velx_out = PID_Update(&s_mode1_velx_pid, velx_target, -Pos_Est_vel_x_kf, dt);
    vely_out = PID_Update(&s_mode1_vely_pid, vely_target, -Pos_Est_vel_y_kf, dt);

    velx_out = FC_Mode_Clamp(velx_out, -s_mode1_angle_limit_deg, s_mode1_angle_limit_deg);
    vely_out = FC_Mode_Clamp(vely_out, -s_mode1_angle_limit_deg, s_mode1_angle_limit_deg);

    roll_angle_target = velx_out + FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = vely_out + FC_Mode_Get_Pitch_Mech_Trim_Deg();
}
