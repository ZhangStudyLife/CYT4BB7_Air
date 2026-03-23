#include "IMU_Filtter.h"

#include <math.h>

#define IMU_FILTER_PI  (3.14159265359f)

/* 1000Hz 控制器输入链路滤波输出 */
imudata_t g_imufilter_1000hz;
/* 500Hz 姿态链路滤波输出 */
imudata_t g_imudata_500hz;
/* 250Hz 融合链路滤波输出 */
imudata_t g_imudata_250hz;

/* IMU 内部滤波器状态 */
static struct
{
    IMUBiquad_t gyro_anti_alias_lpf[IMU_AXIS_NUM]; /* 陀螺仪抗混叠低通 */
    IMUBiquad_t gyro_notch0[IMU_AXIS_NUM];         /* 陀螺仪第一级陷波 */
    IMUBiquad_t gyro_notch1[IMU_AXIS_NUM];         /* 陀螺仪第二级陷波 */
    IMUBiquad_t gyro_lpf[IMU_AXIS_NUM];            /* 陀螺仪主低通 */

    IMUBiquad_t accel_notch0[IMU_AXIS_NUM]; /* 加速度计第一级陷波 */
    IMUBiquad_t accel_notch1[IMU_AXIS_NUM]; /* 加速度计第二级陷波 */
    IMUBiquad_t accel_lpf[IMU_AXIS_NUM];    /* 加速度计主低通 */

    uint8_t initialized; /* 首帧直通标志 */
} s_filt;

/**
 * 函数功能: 执行一次二阶 IIR 滤波。
 * 输入参数:
 *   f  - 目标滤波器状态。
 *   in - 当前输入样本。
 * 返回值:
 *   本次滤波输出值。
 */
static float IMUBiquad_Apply(IMUBiquad_t *f, float in)
{
    float out = f->b0 * in + f->d1;
    f->d1 = f->b1 * in - f->a1 * out + f->d2;
    f->d2 = f->b2 * in - f->a2 * out;
    return out;
}

/**
 * 函数功能: 初始化二阶 Butterworth 低通滤波器。
 * 输入参数:
 *   f  - 目标滤波器状态。
 *   fs - 采样频率，单位 Hz。
 *   fc - 截止频率，单位 Hz。
 * 返回值: 无。
 */
static void IMUBiquad_InitLPF(IMUBiquad_t *f, float fs, float fc)
{
    float w0;
    float sw0;
    float cw0;
    float alpha;
    float a0;

    w0 = 2.0f * IMU_FILTER_PI * fc / fs;
    sw0 = sinf(w0);
    cw0 = cosf(w0);
    alpha = sw0 / (2.0f * 0.70710678f);
    a0 = 1.0f + alpha;

    f->b0 = (1.0f - cw0) * 0.5f / a0;
    f->b1 = (1.0f - cw0) / a0;
    f->b2 = (1.0f - cw0) * 0.5f / a0;
    f->a1 = (-2.0f * cw0) / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

/**
 * 函数功能: 初始化二阶陷波滤波器。
 * 输入参数:
 *   f  - 目标滤波器状态。
 *   fs - 采样频率，单位 Hz。
 *   fc - 中心频率，单位 Hz。
 *   q  - 品质因数。
 * 返回值: 无。
 */
static void IMUBiquad_InitNotch(IMUBiquad_t *f, float fs, float fc, float q)
{
    float w0;
    float sw0;
    float cw0;
    float alpha;
    float a0;

    w0 = 2.0f * IMU_FILTER_PI * fc / fs;
    sw0 = sinf(w0);
    cw0 = cosf(w0);
    alpha = sw0 / (2.0f * q);
    a0 = 1.0f + alpha;

    f->b0 = 1.0f / a0;
    f->b1 = (-2.0f * cw0) / a0;
    f->b2 = 1.0f / a0;
    f->a1 = (-2.0f * cw0) / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

/**
 * 函数功能: 初始化 IMU 输入链路的全部滤波器。
 * 输入参数: 无。
 * 返回值: 无。
 */
void IMUFilter_Init(void)
{
    uint8_t axis;

    for (axis = 0U; axis < IMU_AXIS_NUM; axis++)
    {
        /* 陀螺仪链路：AntiAliasLPF -> Notch0 -> Notch1 -> LPF */
        IMUBiquad_InitLPF(&s_filt.gyro_anti_alias_lpf[axis], IMU_SAMPLE_RATE_HZ, IMU_GYRO_ANTI_ALIAS_LPF_HZ);
        IMUBiquad_InitNotch(&s_filt.gyro_notch0[axis], IMU_SAMPLE_RATE_HZ, IMU_NOTCH0_HZ, IMU_NOTCH0_Q);
        IMUBiquad_InitNotch(&s_filt.gyro_notch1[axis], IMU_SAMPLE_RATE_HZ, IMU_NOTCH1_HZ, IMU_NOTCH1_Q);
        IMUBiquad_InitLPF(&s_filt.gyro_lpf[axis], IMU_SAMPLE_RATE_HZ, IMU_GYRO_LPF_HZ);

        /* 加速度计链路：Notch0 -> Notch1 -> LPF */
        IMUBiquad_InitNotch(&s_filt.accel_notch0[axis], IMU_SAMPLE_RATE_HZ, IMU_NOTCH0_HZ, IMU_NOTCH0_Q);
        IMUBiquad_InitNotch(&s_filt.accel_notch1[axis], IMU_SAMPLE_RATE_HZ, IMU_NOTCH1_HZ, IMU_NOTCH1_Q);
        IMUBiquad_InitLPF(&s_filt.accel_lpf[axis], IMU_SAMPLE_RATE_HZ, IMU_ACCEL_LPF_HZ);
    }

    s_filt.initialized = 0U;
    g_imufilter_1000hz = (imudata_t){0};
    g_imudata_500hz = (imudata_t){0};
    g_imudata_250hz = (imudata_t){0};
}

/**
 * 函数功能: 以 1kHz 输入原始 IMU 数据，依次执行陀螺仪抗混叠、双陷波和低通，
 *           并输出 1000Hz、500Hz、250Hz 三个结构体。
 * 输入参数:
 *   gx, gy, gz - 陀螺仪三轴输入，单位 dps。
 *   ax, ay, az - 加速度计三轴输入，单位 g。
 * 返回值: 无。
 */
void IMUFilter_Update(float gx, float gy, float gz,
                      float ax, float ay, float az)
{
    float gyro_in[IMU_AXIS_NUM];
    float accel_in[IMU_AXIS_NUM];
    float gyro_out[IMU_AXIS_NUM];
    float accel_out[IMU_AXIS_NUM];
    uint8_t axis;

    gyro_in[0] = gx;
    gyro_in[1] = gy;
    gyro_in[2] = gz;
    accel_in[0] = ax;
    accel_in[1] = ay;
    accel_in[2] = az;

    if (0U == s_filt.initialized)
    {
        g_imufilter_1000hz = (imudata_t){gx, gy, gz, ax, ay, az};
        g_imudata_500hz = (imudata_t){gx, gy, gz, ax, ay, az};
        g_imudata_250hz = (imudata_t){gx, gy, gz, ax, ay, az};
        s_filt.initialized = 1U;
        return;
    }

    for (axis = 0U; axis < IMU_AXIS_NUM; axis++)
    {
        float gyro_stage_aa;
        float gyro_stage0;
        float gyro_stage1;
        float accel_stage0;
        float accel_stage1;

        /* 统一陀螺仪链路：先抗混叠，再双陷波，最后 60Hz 低通 */
        gyro_stage_aa = IMUBiquad_Apply(&s_filt.gyro_anti_alias_lpf[axis], gyro_in[axis]);
        gyro_stage0 = IMUBiquad_Apply(&s_filt.gyro_notch0[axis], gyro_stage_aa);
        gyro_stage1 = IMUBiquad_Apply(&s_filt.gyro_notch1[axis], gyro_stage0);
        gyro_out[axis] = IMUBiquad_Apply(&s_filt.gyro_lpf[axis], gyro_stage1);

        /* 统一加速度计链路：双陷波后进入 15Hz 低通 */
        accel_stage0 = IMUBiquad_Apply(&s_filt.accel_notch0[axis], accel_in[axis]);
        accel_stage1 = IMUBiquad_Apply(&s_filt.accel_notch1[axis], accel_stage0);
        accel_out[axis] = IMUBiquad_Apply(&s_filt.accel_lpf[axis], accel_stage1);
    }

    g_imufilter_1000hz.gyrox = gyro_out[0];
    g_imufilter_1000hz.gyroy = gyro_out[1];
    g_imufilter_1000hz.gyroz = gyro_out[2];
    g_imufilter_1000hz.accx = accel_out[0];
    g_imufilter_1000hz.accy = accel_out[1];
    g_imufilter_1000hz.accz = accel_out[2];

    g_imudata_500hz = g_imufilter_1000hz;
    g_imudata_250hz = g_imufilter_1000hz;
}
