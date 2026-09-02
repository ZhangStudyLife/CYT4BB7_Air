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
#ifndef FLOW_GYRO_DECOUPLER_H
#define FLOW_GYRO_DECOUPLER_H

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 函数功能：初始化 PMW3901 专用光流解耦模块
 * 输入参数：无
 * 返回值：无
 */
void FlowGyroDecoupler_Init(void);

/*
 * 函数功能：重置 PMW3901 专用光流解耦模块内部状态
 * 输入参数：无
 * 返回值：无
 */
void FlowGyroDecoupler_Reinit(void);

/*
 * 函数功能：在 1000Hz 下推入陀螺数据，并累计本个 50Hz 窗口内的姿态补偿量
 * 输入参数：t_ms-当前毫秒时间戳；gyro_x-X 轴角速度；gyro_y-Y 轴角速度
 * 返回值：无
 */
void FlowGyroDecoupler_Push1000Hz(uint32 t_ms, float gyro_x, float gyro_y);

/*
 * 函数功能：在 50Hz 下根据当前窗口的积分补偿量更新 PMW3901 解耦结果
 * 输入参数：t_read_ms-当前光流读取时刻；delta_x-X 轴原始光流增量；delta_y-Y 轴原始光流增量
 * 返回值：1 表示本次已完成更新
 */
uint8 FlowGyroDecoupler_Update50Hz(uint32 t_read_ms, int16_t delta_x, int16_t delta_y);

/*
 * 函数功能：获取 PMW3901 X 轴解耦后的光流增量
 * 输入参数：无
 * 返回值：X 轴解耦后的光流增量
 */
float FlowGyroDecoupler_GetDecX(void);

/*
 * 函数功能：获取 PMW3901 Y 轴解耦后的光流增量
 * 输入参数：无
 * 返回值：Y 轴解耦后的光流增量
 */
float FlowGyroDecoupler_GetDecY(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_GYRO_DECOUPLER_H */
