#ifndef IMU_FILTTER_H_
#define IMU_FILTTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 采样率 ======================== */
#define IMU_SAMPLE_RATE_HZ        (1000.0f)

/* ======================== 1000Hz角速度环滤波参数 ======================== */
/* 陷波: 消除电机基频振动 ~150Hz */
#define GYRO_1K_NOTCH_HZ          (150.0f)
#define GYRO_1K_NOTCH_Q           (5.0f)
/* 低通: 截断陷波残余及高频噪声 */
#define GYRO_1K_LPF_HZ            (120.0f)

/* ======================== 500Hz姿态解算滤波参数 ======================== */
/* 角速度低通: 80Hz带宽足以覆盖飞行姿态动态 */
#define GYRO_500_LPF_HZ           (80.0f)
/* 加速度低通: 飞行动态<10Hz, 30Hz留3倍余量 */
#define ACC_500_LPF_HZ            (30.0f)

/* ======================== 250Hz位置融合滤波参数 ======================== */
/* 加速度低通: 位置动态<5Hz, 级联在500Hz的30Hz之后 */
#define ACC_250_LPF_HZ            (15.0f)

#define IMU_AXIS_NUM              (3U)

/* 二阶 IIR (Direct Form II Transposed) */
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

/* 6轴IMU数据结构体: 陀螺仪(dps) + 加速度计(g) */
typedef struct
{
    float gyrox;
    float gyroy;
    float gyroz;
    float accx;
    float accy;
    float accz;
} imudata_t;

/* 1000Hz角速度环: gyro经Notch@150Hz + LPF@120Hz, acc为原始值 */
extern imudata_t g_imufilter_1000hz;
/* 500Hz姿态解算: gyro经LPF@80Hz, acc经LPF@30Hz (Butterworth 2阶) */
extern imudata_t g_imudata_500hz;
/* 250Hz位置融合: gyro同500Hz, acc经LPF@30Hz->LPF@15Hz级联 */
extern imudata_t g_imudata_250hz;

/**
 * 函数功能: 初始化全部滤波器链 (3个环路)
 * 输入参数: 无
 * 返回值:   无
 */
void IMUFilter_Init(void);

/**
 * 函数功能: 1kHz调用, 对原始6轴数据执行全部滤波并填充3个输出结构体
 * 输入参数: gx,gy,gz - 陀螺仪原始值(dps); ax,ay,az - 加速度计原始值(g)
 * 返回值:   无
 */
void IMUFilter_Update(float gx, float gy, float gz,
                      float ax, float ay, float az);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FILTTER_H_ */
