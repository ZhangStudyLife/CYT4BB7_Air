#include "fc_mode.h"

/* 模式0手动姿态映射比例，单位度每遥控归一化单位 */
static const float s_mode0_rc_to_angle_scale = 0.04f;
/* 模式0手动姿态最大角度，单位度 */
static const float s_mode0_manual_angle_limit_deg = 20.0f;

/*
 * 函数名: FC_Mode0_Init
 * 功能: 初始化模式0控制资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode0_Init(void)
{
    FC_Mode0_Reset();
}

/*
 * 函数名: FC_Mode0_Reset
 * 功能: 将模式0姿态目标复位到机械配平
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode0_Reset(void)
{
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

/*
 * 函数名: FC_Mode0_100Hz
 * 功能: 在模式0下根据摇杆生成手动姿态目标
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode0_100Hz(void)
{
    float ch0;
    float ch1;

    if ((FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING) &&
        (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_LANDING))
    {
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0], -1000.0f, 1000.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1], -1000.0f, 1000.0f);

    roll_angle_target = FC_Mode_Clamp(ch0 * s_mode0_rc_to_angle_scale + FC_Mode_Get_Roll_Mech_Trim_Deg(),
                                      -s_mode0_manual_angle_limit_deg, s_mode0_manual_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(-ch1 * s_mode0_rc_to_angle_scale + FC_Mode_Get_Pitch_Mech_Trim_Deg(),
                                       -s_mode0_manual_angle_limit_deg, s_mode0_manual_angle_limit_deg);
}

/*
 * 函数名: FC_Mode0_50Hz
 * 功能: 执行模式0的50Hz控制占位
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode0_50Hz(float dt)
{
    (void)dt;
}
