#include "fc_mode.h"

/*
 * 函数名: FC_Mode1_Init
 * 功能: 初始化模式1控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_Init(void)
{
}

/*
 * 函数名: FC_Mode1_Reset
 * 功能: 复位模式1航向目标
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_Reset(void)
{
    yaw_angle_target = 0.0f;
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
 * 函数名: FC_Mode1_Control100Hz
 * 功能: 复用模式2跟车控制，航向目标固定为0度
 * 输入参数:
 *   dt - 本次调用周期，单位s
 * 返回值: 无
 */
void FC_Mode1_Control100Hz(float dt)
{
    FC_Mode2_Control100Hz(dt);
}
