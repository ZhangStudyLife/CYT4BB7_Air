#include "IMU_Filtter.h"
#include <math.h>

#define IMU_FILTER_PI  (3.14159265359f)

/* ======================== 3个环路输出结构体 ======================== */
imudata_t g_imufilter_1000hz;  /* 1000Hz角速度环输出 */
imudata_t g_imudata_500hz;     /* 500Hz姿态解算输出 */
imudata_t g_imudata_250hz;     /* 250Hz位置融合输出 */

/* ======================== 滤波器内部状态 ======================== */
static struct
{
    /* 1000Hz角速度环: Notch@150Hz -> LPF@120Hz */
    IMUBiquad_t gyro_notch[IMU_AXIS_NUM];
    IMUBiquad_t gyro_lpf_1k[IMU_AXIS_NUM];

    /* 500Hz姿态解算: Gyro LPF@80Hz, Acc LPF@30Hz */
    IMUBiquad_t gyro_lpf_500[IMU_AXIS_NUM];
    IMUBiquad_t acc_lpf_500[IMU_AXIS_NUM];

    /* 250Hz位置融合: Acc LPF@15Hz (级联在30Hz输出之后) */
    IMUBiquad_t acc_lpf_250[IMU_AXIS_NUM];

    uint8_t initialized;
} s_filt;

/* ======================== Biquad 基础函数 ======================== */

/* 执行一次二阶IIR滤波 */
static float IMUBiquad_Apply(IMUBiquad_t *f, float in)
{
    float out = f->b0 * in + f->d1;
    f->d1 = f->b1 * in - f->a1 * out + f->d2;
    f->d2 = f->b2 * in - f->a2 * out;
    return out;
}

/* 初始化二阶Butterworth低通, Q=0.7071 */
static void IMUBiquad_InitLPF(IMUBiquad_t *f, float fs, float fc)
{
    float w0    = 2.0f * IMU_FILTER_PI * fc / fs;
    float sw0   = sinf(w0);
    float cw0   = cosf(w0);
    float alpha = sw0 / (2.0f * 0.70710678f);
    float a0    = 1.0f + alpha;

    f->b0 = (1.0f - cw0) * 0.5f / a0;
    f->b1 = (1.0f - cw0) / a0;
    f->b2 = (1.0f - cw0) * 0.5f / a0;
    f->a1 = (-2.0f * cw0) / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

/* 初始化二阶陷波滤波器 */
static void IMUBiquad_InitNotch(IMUBiquad_t *f, float fs, float fc, float q)
{
    float w0    = 2.0f * IMU_FILTER_PI * fc / fs;
    float sw0   = sinf(w0);
    float cw0   = cosf(w0);
    float alpha = sw0 / (2.0f * q);
    float a0    = 1.0f + alpha;

    f->b0 =  1.0f / a0;
    f->b1 = (-2.0f * cw0) / a0;
    f->b2 =  1.0f / a0;
    f->a1 = (-2.0f * cw0) / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

/* ======================== 对外接口 ======================== */

/* 初始化全部滤波器链 */
void IMUFilter_Init(void)
{
    uint8_t i;

    for (i = 0U; i < IMU_AXIS_NUM; i++)
    {
        /* 1000Hz角速度环: 陷波 + 低通 */
        IMUBiquad_InitNotch(&s_filt.gyro_notch[i],   IMU_SAMPLE_RATE_HZ, GYRO_1K_NOTCH_HZ, GYRO_1K_NOTCH_Q);
        IMUBiquad_InitLPF(&s_filt.gyro_lpf_1k[i],    IMU_SAMPLE_RATE_HZ, GYRO_1K_LPF_HZ);

        /* 500Hz姿态解算: 独立低通 */
        IMUBiquad_InitLPF(&s_filt.gyro_lpf_500[i],   IMU_SAMPLE_RATE_HZ, GYRO_500_LPF_HZ);
        IMUBiquad_InitLPF(&s_filt.acc_lpf_500[i],    IMU_SAMPLE_RATE_HZ, ACC_500_LPF_HZ);

        /* 250Hz位置融合: 级联低通 */
        IMUBiquad_InitLPF(&s_filt.acc_lpf_250[i],    IMU_SAMPLE_RATE_HZ, ACC_250_LPF_HZ);
    }

    s_filt.initialized = 0U;

    /* 清零输出结构体 */
    g_imufilter_1000hz = (imudata_t){0};
    g_imudata_500hz    = (imudata_t){0};
    g_imudata_250hz    = (imudata_t){0};
}

/* 1kHz主循环调用: 执行全部滤波链并填充3个输出结构体 */
void IMUFilter_Update(float gx, float gy, float gz,
                      float ax, float ay, float az)
{
    float gyro_in[IMU_AXIS_NUM];
    float acc_in[IMU_AXIS_NUM];
    float g1k[IMU_AXIS_NUM];   /* 1000Hz角速度环输出 */
    float g500[IMU_AXIS_NUM];  /* 500Hz姿态解算陀螺输出 */
    float a500[IMU_AXIS_NUM];  /* 500Hz姿态解算加速度输出 */
    float a250[IMU_AXIS_NUM];  /* 250Hz位置融合加速度输出 */
    uint8_t i;

    gyro_in[0] = gx;  gyro_in[1] = gy;  gyro_in[2] = gz;
    acc_in[0]  = ax;  acc_in[1]  = ay;  acc_in[2]  = az;

    /* 首帧直通, 避免滤波器启动瞬态 */
    if (0U == s_filt.initialized)
    {
        g_imufilter_1000hz = (imudata_t){gx, gy, gz, ax, ay, az};
        g_imudata_500hz    = (imudata_t){gx, gy, gz, ax, ay, az};
        g_imudata_250hz    = (imudata_t){gx, gy, gz, ax, ay, az};
        s_filt.initialized = 1U;
        return;
    }

    /* 逐轴执行滤波链 */
    for (i = 0U; i < IMU_AXIS_NUM; i++)
    {
        /* 1000Hz角速度环: Notch@150Hz -> LPF@120Hz */
        float notch_out = IMUBiquad_Apply(&s_filt.gyro_notch[i], gyro_in[i]);
        g1k[i] = IMUBiquad_Apply(&s_filt.gyro_lpf_1k[i], notch_out);

        /* 500Hz姿态解算: 独立LPF */
        g500[i] = IMUBiquad_Apply(&s_filt.gyro_lpf_500[i], gyro_in[i]);
        a500[i] = IMUBiquad_Apply(&s_filt.acc_lpf_500[i],  acc_in[i]);

        /* 250Hz位置融合: 30Hz输出级联15Hz, 等效4阶陡降 */
        a250[i] = IMUBiquad_Apply(&s_filt.acc_lpf_250[i], a500[i]);
    }

    /* 填充1000Hz角速度环输出: gyro滤波, acc原始 */
    g_imufilter_1000hz.gyrox = g1k[0];
    g_imufilter_1000hz.gyroy = g1k[1];
    g_imufilter_1000hz.gyroz = g1k[2];
    g_imufilter_1000hz.accx  = ax;
    g_imufilter_1000hz.accy  = ay;
    g_imufilter_1000hz.accz  = az;

    /* 填充500Hz姿态解算输出 */
    g_imudata_500hz.gyrox = g500[0];
    g_imudata_500hz.gyroy = g500[1];
    g_imudata_500hz.gyroz = g500[2];
    g_imudata_500hz.accx  = a500[0];
    g_imudata_500hz.accy  = a500[1];
    g_imudata_500hz.accz  = a500[2];

    /* 填充250Hz位置融合输出: gyro同500Hz, acc级联滤波 */
    g_imudata_250hz.gyrox = g500[0];
    g_imudata_250hz.gyroy = g500[1];
    g_imudata_250hz.gyroz = g500[2];
    g_imudata_250hz.accx  = a250[0];
    g_imudata_250hz.accy  = a250[1];
    g_imudata_250hz.accz  = a250[2];
}
