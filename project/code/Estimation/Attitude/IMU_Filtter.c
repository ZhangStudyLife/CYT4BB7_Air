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
#include "IMU_Filtter.h"

#include <math.h>
#include <stddef.h>

#define IMU_FILTER_PI  (3.14159265359f)
#define IMU_FILTER_PT2_CUTOFF_CORRECTION (1.553773974f)

/* 1000Hz 控制器输入链路滤波输出 */
imudata_t g_imufilter_1000hz;
/* 500Hz 姿态链路滤波输出 */
imudata_t g_imudata_500hz;
/* 250Hz 融合链路滤波输出 */
imudata_t g_imudata_250hz;
/* IMU 原始输入冲击标志：1=当前处于冲击保持窗口 */
uint8_t g_imu_shock_flag;
/* IMU 原始输入合加速度模长，单位 g */
float g_imu_acc_norm_g;

/* IMU 内部滤波器状态 */
static struct
{
    IMUBiquad_t gyro_notch0[IMU_AXIS_NUM]; /* 陀螺仪 300Hz 陷波 */
    IMUBiquad_t gyro_lpf[IMU_AXIS_NUM];    /* 陀螺仪 80Hz 主低通 */

    IMUPt2_t accel_lpf[IMU_AXIS_NUM]; /* 加速度计 25Hz PT2 低通 */

    uint8_t initialized; /* 首帧直通标志 */
    uint16_t shock_hold_count; /* IMU 冲击保持计数，单位 1kHz 采样点 */
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

static float IMUPt1_Gain(float f_cut, float dt)
{
    float omega = 2.0f * IMU_FILTER_PI * f_cut * dt;
    return omega / (omega + 1.0f);
}

static void IMUPt2_InitLPF(IMUPt2_t *f, float fs, float fc)
{
    float dt;

    if ((f == NULL) || (fs <= 0.0f) || (fc <= 0.0f))
    {
        return;
    }

    dt = 1.0f / fs;
    f->state = 0.0f;
    f->state1 = 0.0f;
    f->k = IMUPt1_Gain(fc * IMU_FILTER_PT2_CUTOFF_CORRECTION, dt);
}

static float IMUPt2_Apply(IMUPt2_t *f, float in)
{
    f->state1 = f->state1 + f->k * (in - f->state1);
    f->state = f->state + f->k * (f->state1 - f->state);
    return f->state;
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
        /* 陀螺仪链路：300Hz 陷波 -> 80Hz 二阶低通 */
        IMUBiquad_InitNotch(&s_filt.gyro_notch0[axis], IMU_SAMPLE_RATE_HZ, IMU_NOTCH0_HZ, IMU_NOTCH0_Q);
        IMUBiquad_InitLPF(&s_filt.gyro_lpf[axis], IMU_SAMPLE_RATE_HZ, IMU_GYRO_LPF_HZ);

        /* 加速度计链路：25Hz PT2 低通 */
        IMUPt2_InitLPF(&s_filt.accel_lpf[axis], IMU_SAMPLE_RATE_HZ, IMU_ACCEL_LPF_HZ);
    }

    s_filt.initialized = 0U;
    s_filt.shock_hold_count = 0U;
    g_imufilter_1000hz = (imudata_t){0};
    g_imudata_500hz = (imudata_t){0};
    g_imudata_250hz = (imudata_t){0};
    g_imu_shock_flag = 0U;
    g_imu_acc_norm_g = 0.0f;
}

/**
 * 函数功能: 以 1kHz 输入 IMU 数据，执行陀螺仪单陷波与低通、加速度计 PT2 低通，
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
    float gyro_abs_max;
    float accel_abs_max;
    uint8_t axis;

    gyro_in[0] = gx;
    gyro_in[1] = gy;
    gyro_in[2] = gz;
    accel_in[0] = ax;
    accel_in[1] = ay;
    accel_in[2] = az;

    /* 用原始输入判断冲击，后续姿态/光流链路只拿这个标志做门控 */
    gyro_abs_max = fabsf(gx);
    if (fabsf(gy) > gyro_abs_max)
    {
        gyro_abs_max = fabsf(gy);
    }
    if (fabsf(gz) > gyro_abs_max)
    {
        gyro_abs_max = fabsf(gz);
    }

    accel_abs_max = fabsf(ax);
    if (fabsf(ay) > accel_abs_max)
    {
        accel_abs_max = fabsf(ay);
    }
    if (fabsf(az) > accel_abs_max)
    {
        accel_abs_max = fabsf(az);
    }

    g_imu_acc_norm_g = sqrtf(ax * ax + ay * ay + az * az);
    if ((accel_abs_max >= IMU_SHOCK_ACCEL_AXIS_G) ||
        (g_imu_acc_norm_g >= IMU_SHOCK_ACCEL_NORM_G) ||
        (gyro_abs_max >= IMU_SHOCK_GYRO_AXIS_DPS))
    {
        s_filt.shock_hold_count = IMU_SHOCK_HOLD_SAMPLES;
    }

    if (s_filt.shock_hold_count > 0U)
    {
        g_imu_shock_flag = 1U;
        s_filt.shock_hold_count--;
    }
    else
    {
        g_imu_shock_flag = 0U;
    }

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
        float gyro_stage0;

        /* 陀螺仪执行单陷波与主低通，加速度计只执行 PT2 低通。 */
        gyro_stage0 = IMUBiquad_Apply(&s_filt.gyro_notch0[axis], gyro_in[axis]);
        gyro_out[axis] = IMUBiquad_Apply(&s_filt.gyro_lpf[axis], gyro_stage0);
        accel_out[axis] = IMUPt2_Apply(&s_filt.accel_lpf[axis], accel_in[axis]);
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
