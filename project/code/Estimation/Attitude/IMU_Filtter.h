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
#ifndef IMU_FILTTER_H_
#define IMU_FILTTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IMU 统一输入采样率，单位 Hz */
#define IMU_SAMPLE_RATE_HZ         (1000.0f)

/* 第一级陷波中心频率，单位 Hz */
#define IMU_NOTCH0_HZ              (300.0f)
/* 第一级陷波带宽，单位 Hz */
#define IMU_NOTCH0_BW_HZ           (180.0f)
/* 第一级陷波品质因数，按 Q=f/BW 定义 */
#define IMU_NOTCH0_Q               (IMU_NOTCH0_HZ / IMU_NOTCH0_BW_HZ)

/* 陀螺仪主低通二阶 Butterworth 截止频率，单位 Hz */
#define IMU_GYRO_LPF_HZ            (80.0f)
/* 加速度计主低通 PT2 截止频率，单位 Hz */
#define IMU_ACCEL_LPF_HZ           (25.0f)

/* IMU 三轴数量 */
#define IMU_AXIS_NUM               (3U)

/* IMU 单轴冲击判定阈值，单位 g */
#define IMU_SHOCK_ACCEL_AXIS_G     (12.0f)
/* IMU 合加速度冲击判定阈值，单位 g */
#define IMU_SHOCK_ACCEL_NORM_G     (8.0f)
/* IMU 单轴角速度冲击判定阈值，单位 dps */
#define IMU_SHOCK_GYRO_AXIS_DPS    (250.0f)
/* IMU 冲击标志保持时间，单位 1kHz 采样点 */
#define IMU_SHOCK_HOLD_SAMPLES     (50U)

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

typedef struct
{
    float state;
    float state1;
    float k;
} IMUPt2_t;

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
/* IMU 原始输入冲击标志：1=当前处于冲击保持窗口，供日志和融合门控使用 */
extern uint8_t g_imu_shock_flag;
/* IMU 原始输入合加速度模长，单位 g，供日志观察使用 */
extern float g_imu_acc_norm_g;

/**
 * 函数功能: 初始化 IMU 输入链路的全部滤波器。
 * 输入参数: 无。
 * 返回值: 无。
 */
void IMUFilter_Init(void);

/**
 * 函数功能: 以 1kHz 输入 IMU 数据，执行陀螺仪单陷波与低通、加速度计 PT2 低通，
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
