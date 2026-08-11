#include "fc_mode.h"
#include "yaw_align.h"

/*
 * 函数名: FC_Mode1_Init
 * 功能: 初始化模式1信标航向对准状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_Init(void)
{
    YawAlign_Reset();
}

/*
 * 函数名: FC_Mode1_Reset
 * 功能: 复位模式1信标航向对准状态和航向目标
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_Reset(void)
{
    YawAlign_Reset();
    yaw_angle_target = g_euler.yaw;
}

/*
 * 函数名: FC_Mode1_100Hz
 * 功能: 模式1的100Hz占位入口，车模规划由主循环统一更新
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_100Hz(void)
{
}

/*
 * 函数名: FC_Mode1_50Hz
 * 功能: 复用模式2跟车控制，并使用信标居中结果更新航向目标
 * 输入参数:
 *   dt - 本次调用周期，单位s
 * 返回值: 无
 */
void FC_Mode1_50Hz(float dt)
{
    FC_Mode2_50Hz(dt);
    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        YawAlign_Reset();
    }
    else if (g_fc_params.yaw_change_mode2 < 0.5f)
    {
        (void)YawAlign_Update();
    }
}
