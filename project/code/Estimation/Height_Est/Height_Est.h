#ifndef HEIGHT_EST_H_
#define HEIGHT_EST_H_

#include "../../HW_Drivers/VL53L1X/VL53L1X.h"
#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

extern float g_tof_fused_height_mm;  /* TOF 融合高度，单位 mm */
extern float g_tof1_height_mm;       /* 1 号电机下方 TOF 高度，单位 mm */
extern float g_tof2_height_mm;       /* 2 号电机下方 TOF 高度，单位 mm */
extern float g_tof3_height_mm;       /* 3 号电机下方 TOF 高度，单位 mm */
extern float g_tof4_height_mm;       /* 4 号电机下方 TOF 高度，单位 mm */
extern uint8 g_tof_fused_valid;      /* TOF 融合高度有效标志：1=有效，0=无效 */
extern float g_tof_fused_vz_mps;     /* 高度融合速度，单位 m/s，上升为正 */
extern float g_height_fused_vz_mps;  /* 控制环使用的高度速度，单位 m/s，上升为正 */

/*
 * 函数功能：初始化 TOF 相关状态与驱动。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void TOF_Init(void);

/*
 * 函数功能：100Hz 更新 TOF 原始测距并生成融合高度。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void TOF_update_100HZ(void);

/*
 * 函数功能：1kHz 高度估计入口，当前高度不再由加速度积分预测。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void Height_Est_update_1000HZ(void);

/*
 * 函数功能：100Hz 更新 TOF 融合观测并校正高度状态。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void Height_Est_update_100HZ(void);

#ifdef __cplusplus
}
#endif

#endif
