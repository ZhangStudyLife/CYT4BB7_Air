#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../Protocols/wifi/wifi_justfloat/wifi_justfloat.h"

extern volatile uint32 tick_1000us_cnt;
extern volatile float g_car_velocity_strafe_mps;
extern volatile float g_car_velocity_forward_mps;
extern volatile float g_car_image_target_x;
extern volatile float g_car_image_target_y;
extern volatile float g_car_image_target_valid;

/* 模式2 X 轴速度环 PID */
pid_t g_mode2_velx_pid;
float g_mode2_velx_target = 0.0f;
/* 模式2 Y 轴速度环 PID */
pid_t g_mode2_vely_pid;
float g_mode2_vely_target = 0.0f;

/* 遥控到速度映射比例：1000 单位 → 200 cm/s */
static const float s_mode2_rc_to_speed_scale = 0.2f;
/* 速度目标上限，单位 cm/s */
static const float s_mode2_vel_limit_cmps = 200.0f;
/* 速度目标死区，单位 cm/s */
static const float s_mode2_vel_deadzone_cmps = 6.0f;
/* 姿态角输出限幅，单位度 */
static const float s_mode2_angle_limit_deg = 15.0f;

/*
 * 函数名: FC_Mode2_ApplyDeadzone
 * 功能: 对速度目标施加对称死区，死区内归零，死区外缩减死区量
 * 输入参数:
 *   v  - 输入速度目标，单位 cm/s
 *   dz - 死区宽度，单位 cm/s
 * 返回值:
 *   经过死区处理后的速度目标
 */
static float FC_Mode2_ApplyDeadzone(float v, float dz)
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
 * 函数名: FC_Mode2_Init
 * 功能: 初始化模式2速度环 PID
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_Init(void)
{
    PID_Init(&g_mode2_velx_pid,
             g_fc_params.vel_x_kp, g_fc_params.vel_x_ki, g_fc_params.vel_x_kd,
             g_fc_params.vel_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_x_i_limit, g_fc_params.vel_x_d_lpf);
    PID_Init(&g_mode2_vely_pid,
             g_fc_params.vel_y_kp, g_fc_params.vel_y_ki, g_fc_params.vel_y_kd,
             g_fc_params.vel_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_y_i_limit, g_fc_params.vel_y_d_lpf);
    FC_Mode2_Reset();
}

/*
 * 函数名: FC_Mode2_Reset
 * 功能: 复位模式2速度环状态和姿态目标
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_Reset(void)
{
    PID_Reset(&g_mode2_velx_pid);
    PID_Reset(&g_mode2_vely_pid);
    g_mode2_velx_target = 0.0f;
    g_mode2_vely_target = 0.0f;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

/*
 * 函数名: FC_Mode2_100Hz
 * 功能: 模式2 100Hz 占位
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_100Hz(void)
{
}

/*
 * 函数名: FC_Mode2_50Hz
 * 功能: 模式2 50Hz 纯速度 PI 控制
 *   遥控 → 速度目标（±100 cm/s，±6 cm/s 死区）→ PI → 目标角度（含机械中值）
 *   X轴：vel_x_kf 左正右负，取反后右正，PI 输出正 → roll 右倾 → 向右飞
 *   Y轴：vel_y_kf 前正后负，取反后后正，前推 ch1>0 → vely_target<0 → output<0 → pitch<0 → 前倾 → 向前飞
 * 输入参数:
 *   dt - 本次调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode2_50Hz(float dt)
{
    float ch0;
    float ch1;
    float velx_out;
    float vely_out;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0], -1000.0f, 1000.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1], -1000.0f, 1000.0f);

    g_mode2_velx_target = FC_Mode2_ApplyDeadzone(
        FC_Mode_Clamp(ch0 * s_mode2_rc_to_speed_scale,
                      -s_mode2_vel_limit_cmps, s_mode2_vel_limit_cmps),
        s_mode2_vel_deadzone_cmps);
    g_mode2_vely_target = FC_Mode2_ApplyDeadzone(
        FC_Mode_Clamp(-ch1 * s_mode2_rc_to_speed_scale,
                      -s_mode2_vel_limit_cmps, s_mode2_vel_limit_cmps),
        s_mode2_vel_deadzone_cmps);

    velx_out = PID_Update(&g_mode2_velx_pid, g_mode2_velx_target, -Pos_Est_vel_x, dt);
    vely_out = PID_Update(&g_mode2_vely_pid, g_mode2_vely_target, -Pos_Est_vel_y, dt);

    velx_out += g_fc_params.mode2_vel_x_ff_deg_per_cmps * g_mode2_velx_target;
    vely_out += g_fc_params.mode2_vel_y_ff_deg_per_cmps * g_mode2_vely_target;

    // velx_out = PID_Update(&g_mode2_velx_pid, g_mode2_velx_target, -opflow_vel_x_lpf, dt);
    // vely_out = PID_Update(&g_mode2_vely_pid, g_mode2_vely_target, -opflow_vel_y_lpf, dt);    

    roll_angle_target = FC_Mode_Clamp(velx_out+ FC_Mode_Get_Roll_Mech_Trim_Deg(), -s_mode2_angle_limit_deg, s_mode2_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(vely_out+ FC_Mode_Get_Pitch_Mech_Trim_Deg(), -s_mode2_angle_limit_deg, s_mode2_angle_limit_deg);

    // wifi_justfloat(tick_1000us_cnt,
    //                target_height_m * 1000.0f,
    //                g_tof_fused_height_mm,
    //                g_mode2_velx_target,
    //                -Pos_Est_vel_x,
    //                g_mode2_vely_target,
    //                  -Pos_Est_vel_y,
    //                  g_car_velocity_strafe_mps,
    //                     g_car_velocity_forward_mps,
    //                g_car_image_target_x,
    //                     g_car_image_target_y,
    //                g_car_image_target_valid
    //                );
}
