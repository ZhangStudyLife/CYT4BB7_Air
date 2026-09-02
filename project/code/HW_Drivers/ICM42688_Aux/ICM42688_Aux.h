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
#ifndef PROJECT_CODE_HW_DRIVERS_ICM42688_AUX_ICM42688_AUX_H_
#define PROJECT_CODE_HW_DRIVERS_ICM42688_AUX_ICM42688_AUX_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 副 ICM42688 原始采样缓存，单位均为寄存器直接输出的 LSB 值 */
typedef struct
{
    int16 acc_x_lsb;   /* X 轴加速度原始值，单位 LSB */
    int16 acc_y_lsb;   /* Y 轴加速度原始值，单位 LSB */
    int16 acc_z_lsb;   /* Z 轴加速度原始值，单位 LSB */
    int16 gyro_x_lsb;  /* X 轴角速度原始值，单位 LSB */
    int16 gyro_y_lsb;  /* Y 轴角速度原始值，单位 LSB */
    int16 gyro_z_lsb;  /* Z 轴角速度原始值，单位 LSB */
    int16 temp_lsb;    /* 温度原始值，单位 LSB */
    uint32 tick_us;    /* 本帧采样对应的 1kHz 时间戳，单位 us */
} icm42688_aux_raw_t;

/* 副 ICM42688 物理量缓存，单位与主 ICM42688 保持一致 */
typedef struct
{
    float acc_x;       /* X 轴加速度，单位 g */
    float acc_y;       /* Y 轴加速度，单位 g */
    float acc_z;       /* Z 轴加速度，单位 g */
    float gyro_x;      /* X 轴角速度，单位 dps */
    float gyro_y;      /* Y 轴角速度，单位 dps */
    float gyro_z;      /* Z 轴角速度，单位 dps */
    float temp;        /* 温度占位，当前固定为 0.0f */
} icm42688_aux_real_t;

extern volatile icm42688_aux_raw_t g_icm42688_aux_raw; /* 副 ICM42688 原始 LSB 数据缓存 */
extern volatile icm42688_aux_real_t g_icm42688_aux;    /* 副 ICM42688 物理量缓存 */
extern volatile uint8 g_icm42688_aux_ready;            /* 副 ICM42688 是否初始化成功 */

/*
 * 函数功能: 初始化 SPI1 副 ICM42688，并按主 ICM42688_CONFIG 对齐运行配置
 * 输入参数: 无
 * 返回值: 1=初始化成功，0=初始化失败
 */
uint8 ICM42688_Aux_Init(void);

/*
 * 函数功能: 在 1kHz 调度中读取一次副 ICM42688 原始寄存器并刷新原始/物理量缓存
 * 输入参数:
 *   tick_us - 当前 1kHz 调度时间戳，单位 us
 * 返回值: 无
 */
void ICM42688_Aux_Update_1000Hz(uint32 tick_us);

#ifdef __cplusplus
}
#endif

#endif /* PROJECT_CODE_HW_DRIVERS_ICM42688_AUX_ICM42688_AUX_H_ */
