#ifndef IMU_TOP_H_
#define IMU_TOP_H_

#include "../../HW_Drivers/ICM42688/ICM42688.h"
#include "IMU_Filtter.h"
#include "MahonyAhrs.h"


#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- IMU 初始化参数（2kHz） ---------------- */
#define IMU_WARMUP_DISCARD_SAMPLES   (4000U)
#define IMU_UPDATE_DT_SEC            (1.0f / IMU_SAMPLE_RATE_HZ)

/* ---------------- 上电自检参数 ---------------- */
#define IMU_SELFTEST_SAMPLE_COUNT        (200U)
#define IMU_SELFTEST_GYRO_MEAN_MAX_DPS   (8.0f)
#define IMU_SELFTEST_ACC_MIN_G           (0.75f)
#define IMU_SELFTEST_ACC_MAX_G           (1.25f)

extern IMUFilter_t g_imu_filter;
extern MahonyAhrs_t g_mahony_ahrs;
extern MahonyAhrs_Euler_t g_euler;
extern uint8 g_imu_ready;
extern uint32 g_imu_update_count;

void IMU_Init_All(void);
void IMU_Update_2kHz(void);
uint8 IMU_Is_Ready(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_TOP_H_ */
