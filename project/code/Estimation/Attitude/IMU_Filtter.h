#ifndef IMU_FILTTER_H_
#define IMU_FILTTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 采样参数 ======================== */
#ifndef IMU_SAMPLE_RATE_HZ
#define IMU_SAMPLE_RATE_HZ        (1000.0f)
#endif

/* ======================== 陀螺滤波链参数 ======================== */
#define IMU_GYRO_NOTCH1_HZ        (220.0f)
#define IMU_GYRO_NOTCH2_HZ        (440.0f)
#define IMU_GYRO_NOTCH_Q          (5.0f)
#define IMU_GYRO_LOWPASS_HZ       (70.0f)

/* ======================== 加速度滤波链参数 ======================== */
#define IMU_ACC_LOWPASS_EST_HZ    (20.0f)
#define IMU_ACC_LOWPASS_POS_HZ    (10.0f)

#define IMU_FILTER_AXIS_NUM        (3U)

/* 二阶 IIR（Direct Form II Transposed） */
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

/* ======================== 滤波器状态 ======================== */
typedef struct
{
    /* 原始输入（单位：gyro=dps, acc=g） */
    float gyro_raw_x;
    float gyro_raw_y;
    float gyro_raw_z;
    float acc_raw_x;
    float acc_raw_y;
    float acc_raw_z;

    /* 滤波输出（单位同输入） */
    float gyro_filt_x;
    float gyro_filt_y;
    float gyro_filt_z;
    float acc_filt_x;
    float acc_filt_y;
    float acc_filt_z;

    /* 新命名输出：控制与估计分离 */
    float gyro_ctrl_x;
    float gyro_ctrl_y;
    float gyro_ctrl_z;
    float acc_est_x;
    float acc_est_y;
    float acc_est_z;
    float acc_est_10_x;
    float acc_est_10_y;
    float acc_est_10_z;

    /* 滤波器状态：每轴独立 */
    IMUBiquad_t gyro_notch1[IMU_FILTER_AXIS_NUM];
    IMUBiquad_t gyro_notch2[IMU_FILTER_AXIS_NUM];
    IMUBiquad_t gyro_lpf[IMU_FILTER_AXIS_NUM];
    IMUBiquad_t acc_lpf20[IMU_FILTER_AXIS_NUM];
    IMUBiquad_t acc_lpf10[IMU_FILTER_AXIS_NUM];

    /* 内部状态 */
    uint8_t initialized;

} IMUFilter_t;

void IMUFilter_Init(IMUFilter_t *filter);
void IMUFilter_Update(IMUFilter_t *filter);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FILTTER_H_ */
