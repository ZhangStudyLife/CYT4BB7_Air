#ifndef FC_MODE_H
#define FC_MODE_H

#include "fc_loop.h"
#include "zf_common_headfile.h"
#ifdef __cplusplus
extern "C"
{
#endif

/*
 * 函数名: FC_Mode_Clamp
 * 功能: 对模式控制中的浮点数进行上下限钳位
 * 输入参数:
 *   value     - 输入值
 *   min_value - 最小允许值
 *   max_value - 最大允许值
 * 返回值:
 *   限幅后的浮点值
 */
static inline float FC_Mode_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

/*
 * 函数名: FC_Mode_Get_Roll_Mech_Trim_Deg
 * 功能: 获取 Roll 机械配平角
 * 输入参数: 无
 * 返回值:
 *   Roll 机械配平角，单位度
 */
static inline float FC_Mode_Get_Roll_Mech_Trim_Deg(void)
{
    return 1.5f;
    // return 1.5f;
}

/*
 * 函数名: FC_Mode_Get_Pitch_Mech_Trim_Deg
 * 功能: 获取 Pitch 机械配平角
 * 输入参数: 无
 * 返回值:
 *   Pitch 机械配平角，单位度
 */
static inline float FC_Mode_Get_Pitch_Mech_Trim_Deg(void)
{
    // return -7.0f;
    return 0.0f;
}

/*
 * 函数名: FC_Mode0_Init
 * 功能: 初始化模式0控制所需资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode0_Init(void);

/*
 * 函数名: FC_Mode0_Reset
 * 功能: 复位模式0控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode0_Reset(void);

/*
 * 函数名: FC_Mode0_100Hz
 * 功能: 执行模式0的100Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode0_100Hz(void);

/*
 * 函数名: FC_Mode0_50Hz
 * 功能: 执行模式0的50Hz控制
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode0_50Hz(float dt);

/*
 * 函数名: FC_Mode1_Init
 * 功能: 初始化模式1控制所需资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_Init(void);

/*
 * 函数名: FC_Mode1_Reset
 * 功能: 复位模式1控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_Reset(void);

/*
 * 函数名: FC_Mode1_100Hz
 * 功能: 执行模式1的100Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_100Hz(void);

/*
 * 函数名: FC_Mode1_50Hz
 * 功能: 执行模式1的50Hz控制
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode1_50Hz(float dt);

/*
 * 函数名: FC_Mode2_Init
 * 功能: 初始化模式2控制所需资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_Init(void);

/*
 * 函数名: FC_Mode2_Reset
 * 功能: 复位模式2控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_Reset(void);

/*
 * 函数名: FC_Mode2_100Hz
 * 功能: 执行模式2的100Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_100Hz(void);

/*
 * 函数名: FC_Mode2_50Hz
 * 功能: 执行模式2的50Hz控制
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode2_50Hz(float dt);

#ifdef __cplusplus
}
#endif

#endif /* FC_MODE_H */
