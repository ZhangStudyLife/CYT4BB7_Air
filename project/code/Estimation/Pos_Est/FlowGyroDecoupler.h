#ifndef FLOW_GYRO_DECOUPLER_H
#define FLOW_GYRO_DECOUPLER_H

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 函数功能：初始化光流去陀螺补偿模块
 * 输入参数：无
 * 返回值：无
 */
void  FlowGyroDecoupler_Init(void);

/*
 * 函数功能：重新初始化光流去陀螺补偿模块
 * 输入参数：无
 * 返回值：无
 */
void  FlowGyroDecoupler_Reinit(void);

/*
 * 函数功能：在1000Hz下推入陀螺数据并累计补偿量
 * 输入参数：t_ms-时间戳；gyro_x-X轴角速度；gyro_y-Y轴角速度
 * 返回值：无
 */
void  FlowGyroDecoupler_Push1000Hz(uint32 t_ms, float gyro_x, float gyro_y);

/*
 * 函数功能：在50Hz下更新光流去耦结果
 * 输入参数：t_read_ms-光流时间戳；delta_x-X轴光流增量；delta_y-Y轴光流增量
 * 返回值：1表示本次已完成更新
 */
uint8 FlowGyroDecoupler_Update50Hz(uint32 t_read_ms, int16_t delta_x, int16_t delta_y);

/*
 * 函数功能：获取X轴去耦后的光流增量
 * 输入参数：无
 * 返回值：X轴去耦后的光流增量
 */
float FlowGyroDecoupler_GetDecX(void);

/*
 * 函数功能：获取Y轴去耦后的光流增量
 * 输入参数：无
 * 返回值：Y轴去耦后的光流增量
 */
float FlowGyroDecoupler_GetDecY(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_GYRO_DECOUPLER_H */
