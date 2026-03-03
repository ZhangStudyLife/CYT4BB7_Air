/* TOF数据模块头文件保护宏 */
#ifndef TOF_DATA_H_
/* TOF数据模块头文件保护宏 */
#define TOF_DATA_H_

#include "../../HW_Drivers/VL53L1X/VL53L1X.h"
#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 融合后TOF高度，单位：mm，无效时为VL53L1X_VALID_RANGE_MAX */
extern uint16 g_tof_fused_height_mm;
/* TOF通道2显示高度，单位：mm，无效时为VL53L1X_INVALID_DISTANCE_MM */
extern uint16 g_tof2_height_mm;
/* TOF通道3显示高度，单位：mm，无效时为VL53L1X_INVALID_DISTANCE_MM */
extern uint16 g_tof3_height_mm;
/* 融合高度有效标志，1=有效，0=无效 */
extern uint8 g_tof_fused_valid;
/* 通道2参与融合的原始有效标志，1=有效，0=无效 */
extern uint8 g_tof2_valid;
/* 通道3参与融合的原始有效标志，1=有效，0=无效 */
extern uint8 g_tof3_valid;
/* 本帧融合是否使用了通道2，1=使用，0=未使用 */
extern uint8 g_tof2_used_in_fusion;
/* 本帧融合是否使用了通道3，1=使用，0=未使用 */
extern uint8 g_tof3_used_in_fusion;
/* 当前融合来源，取值见TOF_SRC_*宏 */
extern uint8 g_tof_fused_source;

/*
 * 函数功能：初始化双TOF模块并执行零偏标定。
 * 输入参数：
 *   无。
 * 返回值：
 *   无。
 */
void TOF_Init(void);
/*
 * 函数功能：更新双TOF观测并输出融合高度结果。
 * 输入参数：
 *   无。
 * 返回值：
 *   无。
 */
void TOF_Update(void);
/*
 * 函数功能：100Hz任务入口，更新TOF融合高度。
 * 输入参数：
 *   无。
 * 返回值：
 *   无。
 */
void TOF_update_100HZ(void);
/*
 * 函数功能：执行TOF静态标定，估计双通道零偏并对齐到共同中心。
 * 输入参数：
 *   无。
 * 返回值：
 *   无。
 */
void TOF_Calibrate(void);


#ifdef __cplusplus
}
#endif

#endif /* TOF_DATA_H_ */
