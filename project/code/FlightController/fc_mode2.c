#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"

/* 模式2 X 轴位置环 PID */
static pid_t s_mode2_posx_pid;
/* 模式2 Y 轴位置环 PID */
static pid_t s_mode2_posy_pid;
/* 模式2 X 轴速度环 PID */
static pid_t s_mode2_velx_pid;
/* 模式2 Y 轴速度环 PID */
static pid_t s_mode2_vely_pid;
/* 模式2 锁定点 X 轴位置，单位 cm */
static float s_mode2_hold_pos_x = 0.0f;
/* 模式2 锁定点 Y 轴位置，单位 cm */
static float s_mode2_hold_pos_y = 0.0f;
/* 模式2 摇杆是否处于活跃状态 */
static uint8_t s_mode2_stick_active = 0U;
/* 模式2 锁定点是否已经初始化 */
static uint8_t s_mode2_hold_initialized = 0U;

/* 模式2 摇杆进入活跃区阈值 */
static const float s_mode2_stick_active_threshold = 80.0f;
/* 模式2 摇杆回中阈值 */
static const float s_mode2_stick_release_threshold = 60.0f;
/* 模式2 遥控到速度映射比例，单位 cm/s 每遥控归一化单位 */
static const float s_mode2_rc_to_speed_scale = 0.08f;
/* 模式2 最大水平目标速度，单位 cm/s */
static const float s_mode2_vel_limit_cmps = 80.0f;
/* 模式2 位置环输出速度最大值，单位 cm/s */
static const float s_mode2_hold_vel_limit_cmps = 45.0f;
/* 模式2 最大姿态输出，单位度 */
static const float s_mode2_angle_limit_deg = 12.0f;

/*
 * 函数名: FC_Mode2_UpdateHoldPoint
 * 功能: 使用当前位置刷新模式2锁定点
 * 输入参数: 无
 * 返回值: 无
 */
static void FC_Mode2_UpdateHoldPoint(void)
{
    s_mode2_hold_pos_x = Pos_Est_pos_x;
    s_mode2_hold_pos_y = Pos_Est_pos_y;
    s_mode2_hold_initialized = 1U;
}

/*
 * 函数名: FC_Mode2_Init
 * 功能: 初始化模式2控制资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_Init(void)
{
    PID_Init(&s_mode2_velx_pid,
             g_fc_params.vel_x_kp, g_fc_params.vel_x_ki, g_fc_params.vel_x_kd,
             g_fc_params.vel_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_x_i_limit, g_fc_params.vel_x_d_lpf);
    PID_Init(&s_mode2_vely_pid,
             g_fc_params.vel_y_kp, g_fc_params.vel_y_ki, g_fc_params.vel_y_kd,
             g_fc_params.vel_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_y_i_limit, g_fc_params.vel_y_d_lpf);
    PID_Init(&s_mode2_posx_pid,
             g_fc_params.pos_x_kp, g_fc_params.pos_x_ki, g_fc_params.pos_x_kd,
             g_fc_params.pos_x_kff, g_fc_params.pos_xy_dt,
             g_fc_params.pos_x_i_limit, g_fc_params.pos_x_d_lpf);
    PID_Init(&s_mode2_posy_pid,
             g_fc_params.pos_y_kp, g_fc_params.pos_y_ki, g_fc_params.pos_y_kd,
             g_fc_params.pos_y_kff, g_fc_params.pos_xy_dt,
             g_fc_params.pos_y_i_limit, g_fc_params.pos_y_d_lpf);
    FC_Mode2_Reset();
}

/*
 * 函数名: FC_Mode2_Reset
 * 功能: 复位模式2的位置环、速度环和锁定点状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_Reset(void)
{
    PID_Reset(&s_mode2_posx_pid);
    PID_Reset(&s_mode2_posy_pid);
    PID_Reset(&s_mode2_velx_pid);
    PID_Reset(&s_mode2_vely_pid);
    s_mode2_hold_pos_x = 0.0f;
    s_mode2_hold_pos_y = 0.0f;
    s_mode2_stick_active = 0U;
    s_mode2_hold_initialized = 0U;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

/*
 * 函数名: FC_Mode2_100Hz
 * 功能: 执行模式2的100Hz控制占位
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_100Hz(void)
{
}

/*
 * 函数名: FC_Mode2_50Hz
 * 功能: 执行模式2位置环到速度环的50Hz闭环控制
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode2_50Hz(float dt)
{
    float ch0;
    float ch1;
    float abs_ch0;
    float abs_ch1;
    float velx_target;
    float vely_target;
    float velx_out;
    float vely_out;
    uint8_t stick_active_now;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0], -1000.0f, 1000.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1], -1000.0f, 1000.0f);
    abs_ch0 = (ch0 >= 0.0f) ? ch0 : -ch0;
    abs_ch1 = (ch1 >= 0.0f) ? ch1 : -ch1;

    if (s_mode2_stick_active != 0U)
    {
        stick_active_now = ((abs_ch0 > s_mode2_stick_release_threshold) ||
                            (abs_ch1 > s_mode2_stick_release_threshold)) ? 1U : 0U;
    }
    else
    {
        stick_active_now = ((abs_ch0 > s_mode2_stick_active_threshold) ||
                            (abs_ch1 > s_mode2_stick_active_threshold)) ? 1U : 0U;
    }

    if ((s_mode2_stick_active == 0U) && (stick_active_now != 0U))
    {
        PID_Reset(&s_mode2_posx_pid);
        PID_Reset(&s_mode2_posy_pid);
        PID_Reset(&s_mode2_velx_pid);
        PID_Reset(&s_mode2_vely_pid);
    }
    else if ((s_mode2_stick_active != 0U) && (stick_active_now == 0U))
    {
        FC_Mode2_UpdateHoldPoint();
        PID_Reset(&s_mode2_posx_pid);
        PID_Reset(&s_mode2_posy_pid);
        PID_Reset(&s_mode2_velx_pid);
        PID_Reset(&s_mode2_vely_pid);
    }
    else if ((s_mode2_stick_active == 0U) && (stick_active_now == 0U) &&
             (s_mode2_hold_initialized == 0U))
    {
        FC_Mode2_UpdateHoldPoint();
    }

    s_mode2_stick_active = stick_active_now;

    if (s_mode2_stick_active != 0U)
    {
        velx_target = FC_Mode_Clamp(ch0 * s_mode2_rc_to_speed_scale,
                                    -s_mode2_vel_limit_cmps, s_mode2_vel_limit_cmps);
        vely_target = FC_Mode_Clamp(-ch1 * s_mode2_rc_to_speed_scale,
                                    -s_mode2_vel_limit_cmps, s_mode2_vel_limit_cmps);
    }
    else
    {
        velx_target = PID_Update(&s_mode2_posx_pid, -s_mode2_hold_pos_x, -Pos_Est_pos_x, dt);
        vely_target = PID_Update(&s_mode2_posy_pid, -s_mode2_hold_pos_y, -Pos_Est_pos_y, dt);
        velx_target = FC_Mode_Clamp(velx_target, -s_mode2_hold_vel_limit_cmps, s_mode2_hold_vel_limit_cmps);
        vely_target = FC_Mode_Clamp(vely_target, -s_mode2_hold_vel_limit_cmps, s_mode2_hold_vel_limit_cmps);
    }

    velx_out = PID_Update(&s_mode2_velx_pid, velx_target, -Pos_Est_vel_x_kf, dt);
    vely_out = PID_Update(&s_mode2_vely_pid, vely_target, -Pos_Est_vel_y_kf, dt);

    velx_out = FC_Mode_Clamp(velx_out, -s_mode2_angle_limit_deg, s_mode2_angle_limit_deg);
    vely_out = FC_Mode_Clamp(vely_out, -s_mode2_angle_limit_deg, s_mode2_angle_limit_deg);

    roll_angle_target = velx_out + FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = vely_out + FC_Mode_Get_Pitch_Mech_Trim_Deg();
}
