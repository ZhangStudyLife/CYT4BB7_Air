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
#ifndef PROJECT_CODE_HW_DRIVERS_BMI088_BMI088_H_
#define PROJECT_CODE_HW_DRIVERS_BMI088_BMI088_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* BMI088 原始采样缓存，单位均为寄存器直接输出的 LSB 值 */
typedef struct
{
    int16 acc_x_lsb;   /* X 轴加速度原始值，单位 LSB */
    int16 acc_y_lsb;   /* Y 轴加速度原始值，单位 LSB */
    int16 acc_z_lsb;   /* Z 轴加速度原始值，单位 LSB */
    int16 gyro_x_lsb;  /* X 轴角速度原始值，单位 LSB */
    int16 gyro_y_lsb;  /* Y 轴角速度原始值，单位 LSB */
    int16 gyro_z_lsb;  /* Z 轴角速度原始值，单位 LSB */
    int16 temp_lsb;    /* 温度原始值，当前未启用，固定为 0 */
    uint32 tick_us;    /* 本帧采样对应的 1kHz 时间戳，单位 us */
} bmi088_raw_t;

/* BMI088 物理量缓存，角速度单位 dps，加速度单位 g */
typedef struct
{
    float acc_x;       /* X 轴加速度，单位 g */
    float acc_y;       /* Y 轴加速度，单位 g */
    float acc_z;       /* Z 轴加速度，单位 g */
    float gyro_x;      /* X 轴角速度，单位 dps */
    float gyro_y;      /* Y 轴角速度，单位 dps */
    float gyro_z;      /* Z 轴角速度，单位 dps */
    float temp;        /* 温度占位，当前固定为 0.0f */
} bmi088_real_t;

extern volatile bmi088_raw_t g_bmi088_raw;    /* BMI088 原始 LSB 数据缓存 */
extern volatile bmi088_real_t g_bmi088;       /* BMI088 物理量数据缓存 */
extern volatile uint8 g_bmi088_ready;         /* BMI088 初始化成功标志 */
extern volatile uint8 g_bmi088_acc_chip_id;   /* BMI088 加速度计芯片 ID */
extern volatile uint8 g_bmi088_gyro_chip_id;  /* BMI088 陀螺仪芯片 ID */

/*
 * 函数功能: 初始化 SPI1 BMI088，并写入默认量程、ODR 和电源配置
 * 输入参数: 无
 * 返回值: 1=初始化成功，0=初始化失败
 */
uint8 BMI088_Init(void);

/*
 * 函数功能: 在 1kHz 调度中读取一次 BMI088 加速度计和陀螺仪数据，并刷新缓存
 * 输入参数:
 *   tick_us - 当前 1kHz 调度时间戳，单位 us
 * 返回值: 无
 */
void BMI088_Update_1000Hz(uint32 tick_us);

#ifdef __cplusplus
}
#endif

#endif /* PROJECT_CODE_HW_DRIVERS_BMI088_BMI088_H_ */
