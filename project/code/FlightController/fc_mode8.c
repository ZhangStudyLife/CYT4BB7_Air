#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../IPC/ipc_image_data.h"

/* 模式8图像X位置环PID，输出X轴目标速度，单位cm/s */
static pid_t s_mode8_imgx_pid;
/* 模式8图像Y位置环PID，输出Y轴目标速度，单位cm/s */
static pid_t s_mode8_imgy_pid;
/* 模式8 X轴速度环PID，参数与模式2速度环一致 */
static pid_t s_mode8_velx_pid;
/* 模式8 Y轴速度环PID，参数与模式2速度环一致 */
static pid_t s_mode8_vely_pid;

/* 遥控到速度映射比例：1000遥控量对应200cm/s */
static const float s_mode8_rc_to_speed_scale = 0.2f;
/* 手动速度目标限幅，单位cm/s */
static const float s_mode8_vel_limit_cmps = 200.0f;
/* 手动速度目标死区，单位cm/s */
static const float s_mode8_vel_deadzone_cmps = 6.0f;
/* 图像位置环输出速度限幅，单位cm/s */
static const float s_mode8_img_vel_limit_cmps = 100.0f;
/* 速度环输出姿态角限幅，单位deg */
static const float s_mode8_angle_limit_deg = 15.0f;
/* 图像闭环切手动速度闭环的摇杆阈值，单位遥控标准化量 */
static const float s_mode8_visual_rc_gate = 150.0f;
/* 默认视觉X偏差，单位沿用旧图像像素中心系；主控不再从 IPC 读取图像结果 */
static const float s_mode8_default_img_x = IPC_IMAGE_DEFAULT_X;
/* 默认视觉Y偏差，单位沿用旧图像像素中心系；主控不再从 IPC 读取图像结果 */
static const float s_mode8_default_img_y = IPC_IMAGE_DEFAULT_Y;
/* 默认视觉有效标志：1=使用固定兜底坐标，0=视为无目标 */
static const uint8 s_mode8_default_img_valid = IPC_IMAGE_DEFAULT_VALID;

/*
 * 函数名: FC_Mode8_ApplyDeadzone
 * 功能: 对速度目标施加对称死区，死区外平移回零。
 * 输入参数:
 *   v  - 输入速度目标，单位cm/s。
 *   dz - 死区宽度，单位cm/s。
 * 返回值: 死区处理后的速度目标，单位cm/s。
 */
static float FC_Mode8_ApplyDeadzone(float v, float dz)
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
 * 函数名: FC_Mode8_Init
 * 功能: 初始化模式8图像位置环和速度环PID资源。
 * 输入参数: 无。
 * 返回值: 无。
 */
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

/*
 * 函数名: FC_Mode8_Reset
 * 功能: 复位模式8 PID状态和姿态目标。
 * 输入参数: 无。
 * 返回值: 无。
 */
void FC_Mode8_Reset(void)
{
    PID_Reset(&s_mode8_imgx_pid);
    PID_Reset(&s_mode8_imgy_pid);
    PID_Reset(&s_mode8_velx_pid);
    PID_Reset(&s_mode8_vely_pid);
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

/*
 * 函数名: FC_Mode8_100Hz
 * 功能: 模式8 100Hz占位，高度和Yaw继续使用通用闭环。
 * 输入参数: 无。
 * 返回值: 无。
 */
void FC_Mode8_100Hz(void)
{
}

/*
 * 函数名: FC_Mode8_50Hz
 * 功能: 执行图像误差到目标速度，再到模式2同款速度环和姿态目标的闭环控制。
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位s。
 * 返回值: 无。
 */
void FC_Mode8_50Hz(float dt)
{
    float ch0;
    float ch1;
    float velx_target;
    float vely_target;
    float velx_out;
    float vely_out;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        FC_Mode8_Reset();
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0], -1000.0f, 1000.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1], -1000.0f, 1000.0f);

    if ((ch0 <= -s_mode8_visual_rc_gate) || (ch0 >= s_mode8_visual_rc_gate) ||
        (ch1 <= -s_mode8_visual_rc_gate) || (ch1 >= s_mode8_visual_rc_gate))
    {
        PID_Reset(&s_mode8_imgx_pid);
        PID_Reset(&s_mode8_imgy_pid);
        velx_target = FC_Mode8_ApplyDeadzone(
            FC_Mode_Clamp(ch0 * s_mode8_rc_to_speed_scale,
                          -s_mode8_vel_limit_cmps, s_mode8_vel_limit_cmps),
            s_mode8_vel_deadzone_cmps);
        vely_target = FC_Mode8_ApplyDeadzone(
            FC_Mode_Clamp(-ch1 * s_mode8_rc_to_speed_scale,
                          -s_mode8_vel_limit_cmps, s_mode8_vel_limit_cmps),
            s_mode8_vel_deadzone_cmps);
    }
    else if (0U != s_mode8_default_img_valid)
    {
        velx_target = PID_Update(&s_mode8_imgx_pid, 0.0f, s_mode8_default_img_x, dt);
        vely_target = PID_Update(&s_mode8_imgy_pid, 0.0f, -s_mode8_default_img_y, dt);
        velx_target = FC_Mode_Clamp(velx_target,
                                    -s_mode8_img_vel_limit_cmps, s_mode8_img_vel_limit_cmps);
        vely_target = FC_Mode_Clamp(vely_target,
                                    -s_mode8_img_vel_limit_cmps, s_mode8_img_vel_limit_cmps);
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
