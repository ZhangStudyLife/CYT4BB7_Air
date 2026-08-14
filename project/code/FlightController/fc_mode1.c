#include "fc_mode.h"
#include "yaw_align.h"

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
    YawAlign_Reset();
    yaw_angle_target = 0.0f;
}

/*
 * 函数名: FC_Mode1_100Hz
 * 功能: 根据模式1航向开关更新航向对准目标
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_100Hz(void)
{
    if((FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING) &&
       (g_fc_params.yaw_change_mode1 >= 0.5f))
    {
        (void)YawAlign_Update();
    }
    else
    {
        YawAlign_Reset();
        yaw_angle_target = 0.0f;
    }
}

/*
 * 函数名: FC_Mode1_Control100Hz
 * 功能: 复用模式2跟车控制，航向目标由模式1独立更新
 * 输入参数:
 *   dt - 本次调用周期，单位s
 * 返回值: 无
 */
void FC_Mode1_Control100Hz(float dt)
{
    FC_Mode2_Control100Hz(dt);
}
