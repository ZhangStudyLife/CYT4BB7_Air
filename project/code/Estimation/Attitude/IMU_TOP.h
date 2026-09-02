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
#ifndef IMU_TOP_H_
#define IMU_TOP_H_

#include "../../HW_Drivers/ICM42688/ICM42688.h"
#include "IMU_Filtter.h"
#include "MahonyAhrs.h"


#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- IMU 初始化参数（1kHz）---------------- */
#define IMU_WARMUP_DISCARD_SAMPLES   (1000U) /* IMU预热丢弃样本数，按1kHz约1秒 */
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
void IMU_Update_1000HZ(void); /* IMU 1kHz 更新入口 */
uint8 IMU_Is_Ready(void);


#ifdef __cplusplus
}
#endif

#endif /* IMU_TOP_H_ */

