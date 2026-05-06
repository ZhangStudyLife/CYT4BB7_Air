#include "fc_mode.h"
#include "../IPC/ipc_image_data.h"

/* 模式8图像X误差PID，输出Roll目标角偏移，单位度 */
static pid_t s_mode8_imgx_pid;
/* 模式8图像Y误差PID，输出Pitch目标角偏移，单位度 */
static pid_t s_mode8_imgy_pid;

/* 模式8图像PID默认周期，单位秒 */
static const float s_mode8_pid_dt_s = 0.02f;
/* 模式8图像X误差PID P增益，单位度每像素 */
static const float s_mode8_imgx_pid_kp = 0.04f;
/* 模式8图像X误差PID I增益 */
static const float s_mode8_imgx_pid_ki = 0.0f;
/* 模式8图像X误差PID D增益 */
static const float s_mode8_imgx_pid_kd = 0.015f;
/* 模式8图像Y误差PID P增益，单位度每像素 */
static const float s_mode8_imgy_pid_kp = 0.04f;
/* 模式8图像Y误差PID I增益 */
static const float s_mode8_imgy_pid_ki = 0.0f;
/* 模式8图像Y误差PID D增益 */
static const float s_mode8_imgy_pid_kd = 0.015f;
/* 模式8图像PID积分限幅，单位度 */
static const float s_mode8_img_pid_i_limit_deg = 0.0f;
/* 模式8图像PID D项低通截止频率，单位Hz，0表示旁路 */
static const float s_mode8_img_pid_d_lpf_hz = 0.0f;
/* 模式8图像PID目标角输出限幅，单位度 */
static const float s_mode8_img_angle_limit_deg = 10.0f;
/* 模式8手动姿态映射比例，单位度每遥控归一化单位 */
static const float s_mode8_rc_to_angle_scale = 0.04f;
/* 模式8手动姿态最大角度，单位度 */
static const float s_mode8_manual_angle_limit_deg = 20.0f;
/* 模式8图像闭环接管摇杆死区，单位遥控标准化量 */
static const float s_mode8_visual_rc_gate = 150.0f;

/*
 * 函数名: FC_Mode8_Init
 * 功能: 初始化模式8控制资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode8_Init(void)
{
    PID_Init(&s_mode8_imgx_pid,
             s_mode8_imgx_pid_kp, s_mode8_imgx_pid_ki, s_mode8_imgx_pid_kd,
             0.0f, s_mode8_pid_dt_s,
             s_mode8_img_pid_i_limit_deg, s_mode8_img_pid_d_lpf_hz);
    PID_Init(&s_mode8_imgy_pid,
             s_mode8_imgy_pid_kp, s_mode8_imgy_pid_ki, s_mode8_imgy_pid_kd,
             0.0f, s_mode8_pid_dt_s,
             s_mode8_img_pid_i_limit_deg, s_mode8_img_pid_d_lpf_hz);
    FC_Mode8_Reset();
}

/*
 * 函数名: FC_Mode8_Reset
 * 功能: 复位模式8姿态目标
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode8_Reset(void)
{
    PID_Reset(&s_mode8_imgx_pid);
    PID_Reset(&s_mode8_imgy_pid);
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

/*
 * 函数名: FC_Mode8_100Hz
 * 功能: 执行模式8的100Hz控制模板
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode8_100Hz(void)
{
    float ch0;
    float ch1;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0], -1000.0f, 1000.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1], -1000.0f, 1000.0f);

    if ((ch0 <= -s_mode8_visual_rc_gate) || (ch0 >= s_mode8_visual_rc_gate) ||
        (ch1 <= -s_mode8_visual_rc_gate) || (ch1 >= s_mode8_visual_rc_gate))
    {
        PID_Reset(&s_mode8_imgx_pid);
        PID_Reset(&s_mode8_imgy_pid);
        roll_angle_target = FC_Mode_Clamp(ch0 * s_mode8_rc_to_angle_scale + FC_Mode_Get_Roll_Mech_Trim_Deg(),
                                          -s_mode8_manual_angle_limit_deg, s_mode8_manual_angle_limit_deg);
        pitch_angle_target = FC_Mode_Clamp(-ch1 * s_mode8_rc_to_angle_scale + FC_Mode_Get_Pitch_Mech_Trim_Deg(),
                                           -s_mode8_manual_angle_limit_deg, s_mode8_manual_angle_limit_deg);
    }
}

/*
 * 函数名: FC_Mode8_50Hz
 * 功能: 执行模式8的50Hz控制模板
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位s
 * 返回值: 无
 */
void FC_Mode8_50Hz(float dt)
{
    ipc_image_circle_t circle;
    float ch0;
    float ch1;
    float roll_out;
    float pitch_out;

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
        return;
    }

    if (0U == ipc_image_get_first_valid_circle(&circle))
    {
        FC_Mode8_Reset();
        return;
    }

    roll_out = PID_Update(&s_mode8_imgx_pid, 0.0f, circle.x, dt);
    pitch_out = PID_Update(&s_mode8_imgy_pid, 0.0f, -circle.y, dt);

    roll_out = FC_Mode_Clamp(roll_out, -s_mode8_img_angle_limit_deg, s_mode8_img_angle_limit_deg);
    pitch_out = FC_Mode_Clamp(pitch_out, -s_mode8_img_angle_limit_deg, s_mode8_img_angle_limit_deg);

    roll_angle_target = roll_out + FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = pitch_out + FC_Mode_Get_Pitch_Mech_Trim_Deg();

    wifi_justfloat(circle.x, circle.y, roll_angle_target, pitch_angle_target,g_euler.roll, g_euler.pitch);
}
