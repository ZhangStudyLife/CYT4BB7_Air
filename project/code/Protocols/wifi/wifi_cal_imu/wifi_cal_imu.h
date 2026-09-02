/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
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
