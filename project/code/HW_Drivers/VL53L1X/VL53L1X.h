#ifndef _VL53L1X_DRIVER_H_
#define _VL53L1X_DRIVER_H_

#include "zf_common_headfile.h"

/* TOF 最大有效测距，单位 mm */
#define VL53L1X_VALID_RANGE_MAX          (1400.0f)
/* TOF 无效距离哨兵值，单位 mm */
#define VL53L1X_INVALID_DISTANCE_MM      (8192U)
/* 当前使用四路 TOF */
#define VL53L1X_SENSOR_COUNT             (4U)
/* TOF1 的 I2C SCL 引脚 */
#define VL53L1X_TOF1_SCL_PIN             P17_3
/* TOF1 的 I2C SDA 引脚 */
#define VL53L1X_TOF1_SDA_PIN             P17_4
/* TOF2 的 I2C SCL 引脚 */
#define VL53L1X_TOF2_SCL_PIN             P13_2
/* TOF2 的 I2C SDA 引脚 */
#define VL53L1X_TOF2_SDA_PIN             P13_3
/* TOF3 的 I2C SCL 引脚 */
#define VL53L1X_TOF3_SCL_PIN             P23_4
/* TOF3 的 I2C SDA 引脚 */
#define VL53L1X_TOF3_SDA_PIN             P23_7
/* TOF4 的 I2C SCL 引脚 */
#define VL53L1X_TOF4_SCL_PIN             P06_4
/* TOF4 的 I2C SDA 引脚 */
#define VL53L1X_TOF4_SDA_PIN             P06_3

typedef struct
{
    uint16 distance_mm[VL53L1X_SENSOR_COUNT]; /* 四路 TOF 距离，单位 mm */
    uint8  valid[VL53L1X_SENSOR_COUNT];       /* 四路 TOF 有效标志，1=有效 */
} VL53L1X_data_struct;

/*
 * 函数功能：初始化四路 VL53L1X。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void VL53L1X_Init(void);

/*
 * 函数功能：非堵塞更新四路 VL53L1X 最新测距结果。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void VL53L1X_Update(void);

/*
 * 函数功能：获取四路 VL53L1X 最新缓存数据。
 * 输入参数：
 *   无
 * 返回值：
 *   指向内部缓存的只读指针
 */
const VL53L1X_data_struct *VL53L1X_GetData(void);

#endif
