#ifndef FLOW_GYRO_DECOUPLER_LC302_H
#define FLOW_GYRO_DECOUPLER_LC302_H

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 函数功能：初始化 LC302 专用光流解耦模块
 * 输入参数：无
 * 返回值：无
 */
void FlowGyroDecoupler_LC302_Init(void);

/*
 * 函数功能：重置 LC302 专用光流解耦模块内部状态
 * 输入参数：无
 * 返回值：无
 */
void FlowGyroDecoupler_LC302_Reinit(void);

/*
 * 函数功能：在 1000Hz 下推入陀螺数据，并累计本个 50Hz 窗口内的姿态补偿量
 * 输入参数：t_ms-当前毫秒时间戳；gyro_x-X 轴角速度；gyro_y-Y 轴角速度
 * 返回值：无
 */
void FlowGyroDecoupler_LC302_Push1000Hz(uint32 t_ms, float gyro_x, float gyro_y);

/*
 * 函数功能：在 50Hz 下根据当前窗口的积分补偿量更新 LC302 解耦结果
 * 输入参数：t_read_ms-当前光流读取时刻；delta_x-X 轴原始光流增量；delta_y-Y 轴原始光流增量
 * 返回值：1 表示本次已完成更新
 */
uint8 FlowGyroDecoupler_LC302_Update50Hz(uint32 t_read_ms, int16_t delta_x, int16_t delta_y);

/*
 * 函数功能：获取 LC302 X 轴解耦后的光流增量
 * 输入参数：无
 * 返回值：X 轴解耦后的光流增量
 */
float FlowGyroDecoupler_LC302_GetDecX(void);

/*
 * 函数功能：获取 LC302 Y 轴解耦后的光流增量
 * 输入参数：无
 * 返回值：Y 轴解耦后的光流增量
 */
float FlowGyroDecoupler_LC302_GetDecY(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_GYRO_DECOUPLER_LC302_H */
