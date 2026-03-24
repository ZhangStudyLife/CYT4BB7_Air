#ifndef IMU_FILTTER_H_
#define IMU_FILTTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IMU 统一输入采样率，单位 Hz */
#define IMU_SAMPLE_RATE_HZ         (1000.0f)

/* 第一级陷波中心频率，单位 Hz */
#define IMU_NOTCH0_HZ              (161.0f)
/* 第一级陷波带宽，单位 Hz */
#define IMU_NOTCH0_BW_HZ           (20.0f)
/* 第一级陷波品质因数，按 Q=f/BW 定义 */
#define IMU_NOTCH0_Q               (IMU_NOTCH0_HZ / IMU_NOTCH0_BW_HZ)

/* 第二级陷波中心频率，单位 Hz */
#define IMU_NOTCH1_HZ              (320.0f)
/* 第二级陷波带宽，单位 Hz */
#define IMU_NOTCH1_BW_HZ           (20.0f)
/* 第二级陷波品质因数，按 Q=f/BW 定义 */
#define IMU_NOTCH1_Q               (IMU_NOTCH1_HZ / IMU_NOTCH1_BW_HZ)

/* 陀螺仪抗混叠二阶 Butterworth 低通截止频率，单位 Hz */
#define IMU_GYRO_ANTI_ALIAS_LPF_HZ (250.0f)
/* 陀螺仪主低通二阶 Butterworth 截止频率，单位 Hz */
#define IMU_GYRO_LPF_HZ            (60.0f)
/* 加速度计主低通二阶 Butterworth 截止频率，单位 Hz */
#define IMU_ACCEL_LPF_HZ           (12.0f)

/* IMU 三轴数量 */
#define IMU_AXIS_NUM               (3U)

/* 二阶 IIR 滤波器系数与状态 */
typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float d1;
    float d2;
} IMUBiquad_t;

/* IMU 六轴输出数据，陀螺仪单位 dps，加速度计单位 g */
typedef struct
{
    float gyrox;
    float gyroy;
    float gyroz;
    float accx;
    float accy;
    float accz;
} imudata_t;

/* 1000Hz 控制器输入链路输出，保留接口命名 */
extern imudata_t g_imufilter_1000hz;
/* 500Hz 姿态链路输出，当前与 1000Hz 共用同一条滤波链 */
extern imudata_t g_imudata_500hz;
/* 250Hz 融合链路输出，当前与 1000Hz 共用同一条滤波链 */
extern imudata_t g_imudata_250hz;

/**
 * 函数功能: 初始化 IMU 输入链路的全部滤波器。
 * 输入参数: 无。
 * 返回值: 无。
 */
void IMUFilter_Init(void);

/**
 * 函数功能: 以 1kHz 输入原始 IMU 数据，依次执行陀螺仪抗混叠、双陷波和低通，
 *           并输出 1000Hz、500Hz、250Hz 三个结构体。
 * 输入参数:
 *   gx, gy, gz - 陀螺仪三轴输入，单位 dps。
 *   ax, ay, az - 加速度计三轴输入，单位 g。
 * 返回值: 无。
 */
void IMUFilter_Update(float gx, float gy, float gz,
                      float ax, float ay, float az);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FILTTER_H_ */
