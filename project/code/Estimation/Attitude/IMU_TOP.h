#ifndef IMU_TOP_H_
#define IMU_TOP_H_

#include "../../HW_Drivers/ICM42688/ICM42688.h"
#include "IMU_Filtter.h"
#include "MahonyAhrs.h"


#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- IMU 初始化参数（1kHz�?---------------- */
#define IMU_WARMUP_DISCARD_SAMPLES   (1000U) /* IMUԤ�ȶ�������������1kHzԼ1�� */
#define IMU_UPDATE_DT_SEC            (1.0f / IMU_SAMPLE_RATE_HZ)

/* ---------------- 上电自检参数 ---------------- */
#define IMU_SELFTEST_SAMPLE_COUNT        (200U)
#define IMU_SELFTEST_GYRO_MEAN_MAX_DPS   (8.0f)
#define IMU_SELFTEST_ACC_MIN_G           (0.75f)
#define IMU_SELFTEST_ACC_MAX_G           (1.25f)

/* g_imufilter_1000hz declared in IMU_Filtter.h */
extern MahonyAhrs_t g_mahony_ahrs;
extern MahonyAhrs_Euler_t g_euler;
extern uint8 g_imu_ready;

/*
 * 函数功能: 读取当前 1kHz 周期内供校准使用的原始 IMU 物理量快照。
 * 输入参数:
 *   gx, gy, gz - 输出陀螺仪原始角速度，单位 dps；已做符号映射并扣除陀螺仪零偏
 *   ax, ay, az - 输出加速度计原始比力，单位 g；仅做量程换算与符号映射
 * 输出参数/返回值:
 *   通过指针返回当前帧原始 IMU 快照；空指针会被忽略
 */
void IMU_GetRawSampleForCalibration(float *gx, float *gy, float *gz,
                                    float *ax, float *ay, float *az);

/*
 * 函数功能: 初始化 IMU 驱动、滤波器与姿态解算器。
 * 输入参数: 无
 * 输出参数/返回值: 无
 */
void IMU_Init_All(void);
void IMU_Update_1000HZ(void); /* IMU 1kHz ������� */
uint8 IMU_Is_Ready(void);


#ifdef __cplusplus
}
#endif

#endif /* IMU_TOP_H_ */

