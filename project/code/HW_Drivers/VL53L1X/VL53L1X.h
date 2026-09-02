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
    uint8  fresh_mask;                        /* 最近一次发布时已安全消费的新数据通道掩码 */
    uint32 sample_seq;                        /* 成功发布测距快照的递增序号 */
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
 * 函数功能：请求启动一次四路 VL53L1X 测距更新。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void VL53L1X_RequestUpdate(void);

/*
 * 函数功能：执行 VL53L1X 更新状态机的一个完整寄存器事务。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void VL53L1X_TaskStep(void);

/*
 * 函数功能：同步完成一次四路 VL53L1X 更新，保留旧调用接口兼容性。
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
