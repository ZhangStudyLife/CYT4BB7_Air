/*****************************************************************************
 * 文件: wifi_cal_imu.h
 * 模块: WiFi IMU 校准命令转接
 * 职责: 解析 imu 文本命令，并转接到 Accel_Calibration 校准接口
 *****************************************************************************/

#ifndef WIFI_CAL_IMU_H
#define WIFI_CAL_IMU_H

#include "zf_common_headfile.h"

/*
 * 函数功能: 初始化 WiFi IMU 校准命令模块，并注册校准文本回传回调
 * 输入参数: 无
 * 返回值: 无
 */
void wifi_cal_imu_Init(void);

/*
 * 函数功能: 处理一条以 imu 开头的完整文本命令
 * 输入参数:
 *   line - 完整命令文本，函数内部允许原地切分
 * 返回值: 无
 */
void wifi_cal_imu_ProcessLine(char *line);

#endif /* WIFI_CAL_IMU_H */
