/********************************************************************
 * 文件    : MahonyAhrs.h
 * 模块    : 姿态融合 - Mahony AHRS 四元数解算
 * 位置    : Estimation/Attitude
 * 职责    :
 *   1) 输入滤波后的陀螺仪角速度(deg/s)与加速度(g)
 *   2) 输出姿态四元数与欧拉角
 *   3) 使用 PX4 常见互补融合策略（陀螺积分 + 重力矢量校正）
 * 不负责  : 传感器驱动初始化与底层采样（由 ICM42688 驱动完成）
 ********************************************************************/
#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#include <stdint.h>

/* ======================== 采样率与物理常数 ======================== */
#define MAHONY_SAMPLE_RATE                  2000.0f
#define MAHONY_SAMPLE_DT                    (1.0f / MAHONY_SAMPLE_RATE)
#define DEGREES_TO_RADIANS                  (3.14159265359f / 180.0f)
#define RADIANS_TO_DEGREES                  (180.0f / 3.14159265359f)

/* ======================== 融合参数（PX4风格） ======================== */
#define MAHONY_KP_DEFAULT                   2.0f
#define MAHONY_KI_DEFAULT                   0.03f

/* 加速度可信区间与权重衰减带宽（单位 g） */
#define MAHONY_ACCEL_MIN_MAGNITUDE          0.30f
#define MAHONY_ACCEL_MAX_MAGNITUDE          3.00f
#define MAHONY_ACCEL_TRUST_BAND_G           0.35f

/* 陀螺零偏学习约束 */
#define MAHONY_BIAS_LEARN_MAX_GYRO_DPS      25.0f
#define MAHONY_BIAS_LEARN_MAX_ACC_ERR_G     0.20f
#define MAHONY_BIAS_MAX_RAD_S               0.25f

/* 数值稳定阈值 */
#define MAHONY_VECTOR_NORM_MIN              1e-6f

/* ======================== 姿态状态结构体 ======================== */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;

    /* 动态零偏估计（rad/s） */
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

    /* 积分反馈项（与 gyro_bias 保持一致） */
    float integral_fbx;
    float integral_fby;
    float integral_fbz;

    float kp;
    float ki;

    uint32_t update_count;
    float accel_magnitude;

} MahonyAhrs_t;

/* ======================== 欧拉角输出结构体 ======================== */
typedef struct
{
    /* 角度输出单位：度（GetEulerDegrees） */
    float roll;
    float pitch;
    float yaw;

    /* 供控制侧直接复用的三角函数缓存 */
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;

} MahonyAhrs_Euler_t;

/* ======================== API ======================== */
void MahonyAhrs_Init(MahonyAhrs_t *ahrs);

void MahonyAhrs_Update(MahonyAhrs_t *ahrs,
                       float gyro_x, float gyro_y, float gyro_z,
                       float accel_x, float accel_y, float accel_z,
                       float dt);

void MahonyAhrs_GetQuaternion(const MahonyAhrs_t *ahrs,
                              float *q0, float *q1, float *q2, float *q3);

void MahonyAhrs_GetEuler(const MahonyAhrs_t *ahrs,
                         float *roll, float *pitch, float *yaw);

MahonyAhrs_Euler_t MahonyAhrs_GetEulerDegrees(const MahonyAhrs_t *ahrs);

void MahonyAhrs_SetGains(MahonyAhrs_t *ahrs, float kp, float ki);

void MahonyAhrs_ResetQuaternion(MahonyAhrs_t *ahrs);

void MahonyAhrs_ResetBias(MahonyAhrs_t *ahrs);

#endif /* MAHONY_AHRS_H */
