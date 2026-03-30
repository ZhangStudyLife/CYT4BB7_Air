/********************************************************************
 * 文件    : MahonyAhrs.h
 * 模块    : 姿态融合 - Mahony AHRS 四元数解算
 * 位置    : Estimation/Attitude
 * 职责    :
 *   1) 输入滤波后的机体系角速度(deg/s)与比力(g)
 *   2) 输出姿态四元数与欧拉角
 *   3) 内部计算默认对齐 INAV 纯 6 轴姿态算法
 ********************************************************************/
#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#include <stdint.h>

/* ======================== 采样率与物理常数 ======================== */
#define MAHONY_SAMPLE_RATE                    1000.0f   /* 姿态解算默认采样率，单位 Hz */
#define MAHONY_SAMPLE_DT                      (1.0f / MAHONY_SAMPLE_RATE) /* 姿态解算默认采样周期，单位 s */
#define DEGREES_TO_RADIANS                    (3.14159265359f / 180.0f)   /* 角度转弧度系数 */
#define RADIANS_TO_DEGREES                    (180.0f / 3.14159265359f)   /* 弧度转角度系数 */

/* ======================== INAV 对齐参数 ======================== */
#define MAHONY_KP_DEFAULT                     1.0f     /* 第一阶段回退参数: 提高飞行段姿态拉回力度 */
#define MAHONY_KI_DEFAULT                     0.0f     /* 第一阶段固定关闭积分，避免错误加速度学进 bias */
#define MAHONY_INPUT_ACCEL_IS_SPECIFIC_FORCE  (1U)     /* 1=输入为比力，静止时约 -1g */
#define MAHONY_ACCEL_MIN_MAGNITUDE            0.30f    /* 加速度有效最小模长，单位 g */
#define MAHONY_ACCEL_MAX_MAGNITUDE            3.00f    /* 加速度有效最大模长，单位 g */
#define MAHONY_ACCEL_TRUST_BAND_G             0.35f    /* 第一阶段线性信任带宽: |acc|-1g 超出后逐步降权 */
#define MAHONY_ACCEL_WEIGHT_MIN               0.0f     /* 第一阶段不再额外抬高加计参与门限 */
#define MAHONY_ACCEL_IGNORE_RATE_DPS          0.0f     /* 第一阶段关闭角速度门控，避免飞行段修正归零 */
#define MAHONY_ACCEL_IGNORE_SLOPE_DPS         0.0f     /* 第一阶段关闭角速度门控斜坡 */
#define MAHONY_SPIN_RATE_LIMIT_DPS            20.0f    /* INAV 6轴默认积分限速阈值，单位 dps */
#define MAHONY_ROTATION_LPF_HZ                3.0f     /* INAV 6轴默认角速度门控低通，单位 Hz */
#define MAHONY_FAST_GAIN_SCALE                1.0f     /* 第一阶段关闭未解锁快速增益，台架与飞行保持同一逻辑 */
#define MAHONY_FAST_GAIN_WINDOW_S             0.0f     /* 第一阶段关闭未解锁快速增益窗口 */
#define MAHONY_INTEGRAL_LIMIT_DEG             2.0f     /* INAV 风格积分限幅角度，单位 deg */

/* ======================== 静止检测参数 ======================== */
#define MAHONY_STATIC_GYRO_DPS_TH             1.5f     /* 静止判定角速度阈值，单位 dps */
#define MAHONY_STATIC_ACC_ERR_G_TH            0.08f    /* 静止判定加速度模长误差阈值，单位 g */
#define MAHONY_STATIC_LOCK_COUNT              (100U)   /* 静止判定锁定样本数 */

/* ======================== 数值稳定参数 ======================== */
#define MAHONY_VECTOR_NORM_MIN                1e-6f    /* 向量/四元数最小模长阈值 */
#define MAHONY_QUAT_SMALL_ANGLE_THRESHOLD     0.00489898f /* INAV 小角度增量四元数阈值 */
#define MAHONY_QUAT_MIN_UPDATE                1e-20f   /* 四元数增量最小阈值 */

/* ======================== 姿态状态结构体 ======================== */
typedef struct
{
    float q0;                  /* 四元数实部 */
    float q1;                  /* 四元数 i 分量 */
    float q2;                  /* 四元数 j 分量 */
    float q3;                  /* 四元数 k 分量 */

    float gyro_bias_x;         /* X 轴漂移积分镜像，单位 rad/s */
    float gyro_bias_y;         /* Y 轴漂移积分镜像，单位 rad/s */
    float gyro_bias_z;         /* Z 轴漂移积分镜像，单位 rad/s */
    float gyro_bias_z_static;  /* 保留旧字段，默认不参与姿态更新 */

    float integral_fbx;        /* X 轴积分反馈状态，单位 rad/s */
    float integral_fby;        /* Y 轴积分反馈状态，单位 rad/s */
    float integral_fbz;        /* Z 轴积分反馈状态，单位 rad/s */

    float gyro_lpf_x;          /* X 轴角速度门控低通状态，单位 rad/s */
    float gyro_lpf_y;          /* Y 轴角速度门控低通状态，单位 rad/s */
    float gyro_lpf_z;          /* Z 轴角速度门控低通状态，单位 rad/s */
    float elapsed_time_s;      /* 自初始化以来累计运行时间，单位 s */

    float kp;                  /* 当前姿态 P 增益 */
    float ki;                  /* 当前姿态 I 增益 */

    uint32_t update_count;     /* 姿态更新计数 */
    float accel_magnitude;     /* 当前加速度模长，单位 g */
    float acc_weight_nearness; /* 加速度 1g 接近度权重 */
    float acc_weight_rate_ignore; /* 角速度门控权重 */
    float acc_weight_final;    /* 最终用于姿态修正的加速度权重 */
    uint16_t static_count;     /* 静止连续样本计数 */
    uint8_t is_static;         /* 1=当前姿态输入满足静止条件 */
    uint8_t gyro_lpf_ready;    /* 1=角速度门控低通已初始化 */

} MahonyAhrs_t;

/* ======================== 欧拉角输出结构体 ======================== */
typedef struct
{
    float roll;       /* 横滚角，GetEulerDegrees 输出单位为度 */
    float pitch;      /* 俯仰角，GetEulerDegrees 输出单位为度 */
    float yaw;        /* 偏航角，GetEulerDegrees 输出单位为度 */

    float sin_roll;   /* 横滚角正弦缓存 */
    float cos_roll;   /* 横滚角余弦缓存 */
    float sin_pitch;  /* 俯仰角正弦缓存 */
    float cos_pitch;  /* 俯仰角余弦缓存 */

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
