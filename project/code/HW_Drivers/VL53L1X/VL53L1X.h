/*********************************************************************************************************************
 * VL53L1X 四路TOF测距驱动头文件
 * 通道与机臂对应关系（机体系 FRD，电机编号见AGENTS.md）：
 *   - TOF1（索引0）: 1号电机臂（右下/右后），SCL=P06_4, SDA=P06_3
 *   - TOF2（索引1）: 2号电机臂（右上/右前），SCL=P23_4, SDA=P23_7
 *   - TOF3（索引2）: 3号电机臂（左下/左后），SCL=P13_2, SDA=P13_3
 *   - TOF4（索引3）: 4号电机臂（左上/左前），SCL=P17_3, SDA=P17_4
 * 读取策略：四路同步软IIC，四路读取时间 ≈ 单路读取时间。
 ********************************************************************************************************************/

#ifndef _VL53L1X_DRIVER_H_
#define _VL53L1X_DRIVER_H_

#include "zf_common_headfile.h"
#include <string.h>

/* TOF有效量程上限（mm）。 */
#define VL53L1X_VALID_RANGE_MAX         (1300.0f)
/* TOF无效距离哨兵值（mm）。 */
#define VL53L1X_INVALID_DISTANCE_MM     (8192U)
/* TOF通道总数：1号至4号共四路。 */
#define VL53L1X_CHANNEL_COUNT           (4U)

/* TOF1（1号机臂，右下电机）软IIC SCL引脚。 */
#define VL53L1X1_SCL_PIN    P06_4
/* TOF1（1号机臂，右下电机）软IIC SDA引脚。 */
#define VL53L1X1_SDA_PIN    P06_3

/* TOF2（2号机臂，右上电机）软IIC SCL引脚。 */
#define VL53L1X2_SCL_PIN    P23_4
/* TOF2（2号机臂，右上电机）软IIC SDA引脚。 */
#define VL53L1X2_SDA_PIN    P23_7

/* TOF3（3号机臂，左下电机）软IIC SCL引脚。 */
#define VL53L1X3_SCL_PIN    P13_2
/* TOF3（3号机臂，左下电机）软IIC SDA引脚。 */
#define VL53L1X3_SDA_PIN    P13_3

/* TOF4（4号机臂，左上电机）软IIC SCL引脚。 */
#define VL53L1X4_SCL_PIN    P17_3
/* TOF4（4号机臂，左上电机）软IIC SDA引脚。 */
#define VL53L1X4_SDA_PIN    P17_4

/* 注意：TOF3 当前占用的 P13_2/P13_3 与 BMP388 默认软SPI的 SCK/MISO 引脚冲突，如需同时启用必须重分配其中一方引脚。 */

/**
 * @brief 四路TOF测距输出数据结构体。
 *        索引0~3依次对应TOF1~TOF4（与电机编号一致）。
 */
typedef struct
{
    uint16 distance_mm[VL53L1X_CHANNEL_COUNT];  /* 各路距离（mm），无效时为 VL53L1X_INVALID_DISTANCE_MM。 */
    uint8  range_status[VL53L1X_CHANNEL_COUNT]; /* 各路量程状态寄存器原始值：0x89=正常，0xFF=未知。 */
} VL53L1X_data_struct;

/**
 * @brief TOF单路通信与数据质量诊断状态结构体。
 */
typedef struct
{
    uint8  ack_ok;          /* 本帧I2C ACK是否正常，1=正常，0=失败。 */
    uint8  ready;           /* 本帧传感器测距是否就绪，1=就绪，0=未就绪。 */
    uint8  range_ok;        /* 本帧量程状态是否有效，1=有效，0=无效。 */
    uint8  is_fresh;        /* 本帧数据是否新鲜（未陈旧），1=新鲜，0=陈旧。 */
    uint16 stale_count;     /* 连续陈旧帧计数。 */
    uint16 comm_fail_count; /* 通信失败累计计数，超阈值触发重初始化。 */
} VL53L1X_diag_struct;

/* 四路TOF最新输出缓存，索引0~3对应TOF1~TOF4。 */
extern VL53L1X_data_struct VL53L1X_data;
/* 四路TOF诊断状态，索引0~3对应TOF1~TOF4。 */
extern VL53L1X_diag_struct g_vl53l1x_diag[VL53L1X_CHANNEL_COUNT];

/**
 * @brief 初始化全部四路VL53L1X通道（串行依次初始化，含总线恢复）。
 * @param 无。
 * @return 错误位掩码：bit0~bit3分别对应TOF1~TOF4初始化失败；0=全部成功。
 */
uint8 VL53L1X_init_all(void);

/**
 * @brief 读取四路VL53L1X测距数据。
 *        四路通道采用同步软IIC时序，4路读取总时间约等于单路读取时间。
 * @param data 输出数据指针，写入各路距离（distance_mm[]）与量程状态（range_status[]）。
 * @return 有效新鲜位掩码：bit0~bit3分别对应TOF1~TOF4，1=本帧新鲜有效，0=无效或陈旧。
 */
uint8 VL53L1X_read_data(VL53L1X_data_struct *data);

/**
 * @brief 10Hz维护接口：对通信异常或未初始化的通道尝试重初始化恢复。
 * @param 无。
 * @return 无。
 */
void VL53L1X_recover_update_10HZ(void);

#endif /* _VL53L1X_DRIVER_H_ */
