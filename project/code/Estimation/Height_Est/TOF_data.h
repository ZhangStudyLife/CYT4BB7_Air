#ifndef TOF_DATA_H_
#define TOF_DATA_H_

#include "../../HW_Drivers/VL53L1X/VL53L1X.h"
#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 融合高度，单位 mm，无效时为 VL53L1X_VALID_RANGE_MAX */
extern float g_tof_fused_height_mm;
/* 1 号电机下方 TOF 高度，单位 mm */
extern float g_tof1_height_mm;
/* 4 号电机下方 TOF 高度，单位 mm */
extern float g_tof4_height_mm;
/* 融合高度有效标志，1=有效，0=无效 */
extern uint8 g_tof_fused_valid;
/* 融合高度速度，单位 m/s，当前固定为 0 */
extern float g_tof_fused_vz_mps;

/*
 * 函数功能：初始化两路 TOF 驱动与融合输出。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void TOF_Init(void);

/*
 * 函数功能：100Hz 更新两路 TOF，并输出最简单的融合高度。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void TOF_update_100HZ(void);

#ifdef __cplusplus
}
#endif

#endif
