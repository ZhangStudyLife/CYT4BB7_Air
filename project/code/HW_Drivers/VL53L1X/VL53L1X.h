/*********************************************************************************************************************
 * VL53L1X
 * - VL53L1X2: SDA=P08_2, SCL=P08_1
 * - VL53L1X3: SDA=P07_7, SCL=P07_6
 ********************************************************************************************************************/

#ifndef _VL53L1X_DRIVER_H_
#define _VL53L1X_DRIVER_H_

#include "zf_common_headfile.h"
#include <string.h>
/**
 * @brief TOF有效量程上限（单位：mm）。
 */
#define VL53L1X_VALID_RANGE_MAX (1300.0f)

/**
 * @brief TOF无效距离哨兵值（单位：mm）。
 */
#define VL53L1X_INVALID_DISTANCE_MM (8192U)

/**
 * @brief 2号TOF软IIC的SCL引脚。
 */
#define VL53L1X2_SCL_PIN    P08_1

/**
 * @brief 2号TOF软IIC的SDA引脚。
 */
#define VL53L1X2_SDA_PIN    P08_2

/**
 * @brief 3号TOF软IIC的SCL引脚。
 */
#define VL53L1X3_SCL_PIN    P07_6

/**
 * @brief 3号TOF软IIC的SDA引脚。
 */
#define VL53L1X3_SDA_PIN    P07_7

/**
 * @brief 双路VL53L1X原始测距数据结构体。
 */
typedef struct
{
    /** 2号TOF当前距离（单位：mm）。 */
    uint16 VL53L1X2_distance_mm;
    /** 3号TOF当前距离（单位：mm）。 */
    uint16 VL53L1X3_distance_mm;
    /** 2号TOF量程状态寄存器值。 */
    uint8  VL53L1X2_range_status;
    /** 3号TOF量程状态寄存器值。 */
    uint8  VL53L1X3_range_status;
} VL53L1X_data_struct;

/**
 * @brief VL53L1X链路与数据新鲜度诊断信息结构体。
 */
typedef struct
{
    /** 本帧I2C ACK是否正常，1=正常，0=失败。 */
    uint8 ack_ok;
    /** 传感器是否就绪，1=就绪，0=未就绪。 */
    uint8 ready;
    /** 量程状态是否有效，1=有效，0=无效。 */
    uint8 range_ok;
    /** 本帧数据是否新鲜，1=新鲜，0=陈旧。 */
    uint8 is_fresh;
    /** 连续陈旧帧计数。 */
    uint16 stale_count;
    /** 通信失败累计计数。 */
    uint16 comm_fail_count;
} VL53L1X_diag_struct;

/** 双路TOF最新读取结果（单位：mm，状态见结构体字段）。 */
extern VL53L1X_data_struct VL53L1X_data;
/** 2号TOF诊断状态（通信/就绪/新鲜度）。 */
extern VL53L1X_diag_struct g_vl53l1x2_diag;
/** 3号TOF诊断状态（通信/就绪/新鲜度）。 */
extern VL53L1X_diag_struct g_vl53l1x3_diag;

/**
 * @brief 初始化双路VL53L1X传感器。
 * @param 无。
 * @return 初始化错误位掩码：bit0=2号失败，bit1=3号失败；0表示全部成功。
 */
uint8 VL53L1X_init_all(void);

/**
 * @brief 读取双路VL53L1X距离与状态数据。
 * @param data 输出数据指针，函数会写入双路距离与range_status。
 * @return 有效新鲜数据位掩码：bit0=2号有效，bit1=3号有效；0表示本帧无有效新鲜数据。
 */
uint8 VL53L1X_read_data(VL53L1X_data_struct *data);

/**
 * @brief 10Hz调用的链路恢复维护接口。
 * @param 无。
 * @return 无。
 */
void VL53L1X_recover_update_10HZ(void);

#endif // _VL53L1X_DRIVER_H_
