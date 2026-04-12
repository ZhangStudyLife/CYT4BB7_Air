#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../Estimation/Height_Est/TOF_data.h"
#include "../Protocols/wifi/wifi_justfloat/wifi_justfloat.h"

#define FC_MODE5_DEBUG_OUTPUT_EN (1U) /* 模式5 50Hz调参遥测开关：1-开启 0-关闭 */
#define FC_MODE5_IMG_ERR_LPF_ALPHA (0.3f) /* 视觉位置误差一阶低通系数，越小滤波越强 */

/* 模式5 X 轴图像位置环PID */
static pid_t s_mode5_imgx_pid;
/* 模式5 Y 轴图像位置环PID */
static pid_t s_mode5_imgy_pid;
/* 模式5 X 轴速度环PID */
static pid_t s_mode5_velx_pid;
/* 模式5 Y 轴速度环PID */
static pid_t s_mode5_vely_pid;
/* 模式5视觉误差X低通状态，单位像素误差 */
static float s_mode5_img_err_x_lpf = 0.0f;
/* 模式5视觉误差Y低通状态，单位像素误差 */
static float s_mode5_img_err_y_lpf = 0.0f;
/* 模式5视觉误差低通是否已初始化：1-已初始化 0-未初始化 */
static uint8 s_mode5_img_err_lpf_inited = 0U;

/* 遥控到速度映射比例，1000 单位 -> 100 cm/s */
static const float s_mode5_rc_to_speed_scale = 0.1f;
/* 模式5视觉接管摇杆阈值，单位遥控标准化量 */
static const float s_mode5_visual_rc_gate = 150.0f;
/* 模式5图像位置环X轴P增益，输出单位cm/s/像素 */
static const float s_mode5_imgx_pid_kp = 1.7f;
/* 模式5图像位置环X轴I增益 */
static const float s_mode5_imgx_pid_ki = 0.0f;
/* 模式5图像位置环X轴D增益 */
static const float s_mode5_imgx_pid_kd = 0.4f;
/* 模式5图像位置环X轴D项低通截止频率，单位Hz */
static const float s_mode5_imgx_pid_d_lpf =0.0f;
/* 模式5图像位置环Y轴P增益，输出单位cm/s/像素 */
static const float s_mode5_imgy_pid_kp = 1.7f;
/* 模式5图像位置环Y轴I增益 */
static const float s_mode5_imgy_pid_ki = 0.0f;
/* 模式5图像位置环Y轴D增益 */
static const float s_mode5_imgy_pid_kd = 0.4f;
/* 模式5图像位置环Y轴D项低通截止频率，单位Hz */
static const float s_mode5_imgy_pid_d_lpf = 0.0f;
/* 模式5图像位置环输出限幅（位置环的输出限幅），单位cm/s */
static const float s_mode5_img_pid_out_limit_cmps = 100.0f;
/* 速度目标上限（遥控器端的），单位cm/s */
static const float s_mode5_vel_limit_cmps = 100.0f;
/* 速度目标死区，单位cm/s */
static const float s_mode5_vel_deadzone_cmps = 6.0f;
/* 姿态角输出限幅，单位度 */
static const float s_mode5_angle_limit_deg = 20.0f;
/* 模式5固定高度目标，单位m */
static const float s_mode5_fixed_height_m = 1.1f;

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
    PID_Init(&s_mode5_imgx_pid,
             s_mode5_imgx_pid_kp, s_mode5_imgx_pid_ki, s_mode5_imgx_pid_kd,
             0.0f, g_fc_params.vel_xy_dt,
             0.0f, s_mode5_imgx_pid_d_lpf);

    PID_Init(&s_mode5_imgy_pid,
             s_mode5_imgy_pid_kp, s_mode5_imgy_pid_ki, s_mode5_imgy_pid_kd,
             0.0f, g_fc_params.vel_xy_dt,
             0.0f, s_mode5_imgy_pid_d_lpf);

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
    PID_Reset(&s_mode5_imgx_pid);
    PID_Reset(&s_mode5_imgy_pid);
    PID_Reset(&s_mode5_velx_pid);
    PID_Reset(&s_mode5_vely_pid);
    s_mode5_img_err_x_lpf = 0.0f;
    s_mode5_img_err_y_lpf = 0.0f;
    s_mode5_img_err_lpf_inited = 0U;
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
    float img_err_x;
    float img_err_y;
    uint8 visual_track_enable;
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
    img_err_x = 0.0f;
    img_err_y = 0.0f;
    visual_track_enable = 0U;
    /*没有拨动遥感的情况下 */
    if (0U != visual_track_enable)
    {
        if (0U == s_mode5_img_err_lpf_inited)
        {
            s_mode5_img_err_x_lpf = img_err_x;
            s_mode5_img_err_y_lpf = img_err_y;
            s_mode5_img_err_lpf_inited = 1U;
            PID_Reset(&s_mode5_imgx_pid);
            PID_Reset(&s_mode5_imgy_pid);
        }
        /* 图像位置环观测值低通滤波 */
        s_mode5_img_err_x_lpf += FC_MODE5_IMG_ERR_LPF_ALPHA * (img_err_x - s_mode5_img_err_x_lpf);
        s_mode5_img_err_y_lpf += FC_MODE5_IMG_ERR_LPF_ALPHA * (img_err_y - s_mode5_img_err_y_lpf);

        velx_target = PID_Update(&s_mode5_imgx_pid, 0.0f, -s_mode5_img_err_x_lpf, dt);
        vely_target = PID_Update(&s_mode5_imgy_pid, 0.0f, -s_mode5_img_err_y_lpf, dt);
        velx_target = FC_Mode_Clamp(velx_target, -s_mode5_img_pid_out_limit_cmps, s_mode5_img_pid_out_limit_cmps);
        vely_target = FC_Mode_Clamp(vely_target, -s_mode5_img_pid_out_limit_cmps, s_mode5_img_pid_out_limit_cmps);
    }
    else
    {
        s_mode5_img_err_lpf_inited = 0U;
        s_mode5_img_err_x_lpf = 0.0f;
        s_mode5_img_err_y_lpf = 0.0f;
        PID_Reset(&s_mode5_imgx_pid);
        PID_Reset(&s_mode5_imgy_pid);
        velx_target = FC_Mode5_ApplyDeadzone(
            FC_Mode_Clamp(ch0 * s_mode5_rc_to_speed_scale,
                          -s_mode5_vel_limit_cmps, s_mode5_vel_limit_cmps),
            s_mode5_vel_deadzone_cmps);
        vely_target = FC_Mode5_ApplyDeadzone(
            FC_Mode_Clamp(-ch1 * s_mode5_rc_to_speed_scale,
                          -s_mode5_vel_limit_cmps, s_mode5_vel_limit_cmps),
            s_mode5_vel_deadzone_cmps);
    }

    velx_out = PID_Update(&s_mode5_velx_pid, velx_target, -Pos_Est_vel_x, dt);
    vely_out = PID_Update(&s_mode5_vely_pid, vely_target, -Pos_Est_vel_y, dt);

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
