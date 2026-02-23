#include "IMU_Filtter.h"

#include <math.h>

#define IMU_FILTER_PI  (3.14159265359f)

static float IMUBiquad_Apply(IMUBiquad_t *filter, float input)
{
    float output = filter->b0 * input + filter->d1;

    filter->d1 = filter->b1 * input - filter->a1 * output + filter->d2;
    filter->d2 = filter->b2 * input - filter->a2 * output;

    return output;
}

static void IMUBiquad_ResetState(IMUBiquad_t *filter)
{
    filter->d1 = 0.0f;
    filter->d2 = 0.0f;
}

/*
 * 二阶低通（RBJ cookbook）
 * alpha = sin(w0)/(2Q), 取 Q=0.7071 近似二阶巴特沃斯
 */
static void IMUBiquad_InitLowpass(IMUBiquad_t *filter, float sample_hz, float cutoff_hz)
{
    float q = 0.70710678f;
    float w0 = 2.0f * IMU_FILTER_PI * cutoff_hz / sample_hz;
    float sin_w0 = sinf(w0);
    float cos_w0 = cosf(w0);
    float alpha = sin_w0 / (2.0f * q);
    float a0 = 1.0f + alpha;

    filter->b0 = ((1.0f - cos_w0) * 0.5f) / a0;
    filter->b1 = (1.0f - cos_w0) / a0;
    filter->b2 = ((1.0f - cos_w0) * 0.5f) / a0;
    filter->a1 = (-2.0f * cos_w0) / a0;
    filter->a2 = (1.0f - alpha) / a0;
    IMUBiquad_ResetState(filter);
}

/*
 * 二阶陷波（RBJ cookbook）
 * H(z): b = [1, -2cos(w0), 1], a = [1+alpha, -2cos(w0), 1-alpha]
 */
static void IMUBiquad_InitNotch(IMUBiquad_t *filter, float sample_hz, float center_hz, float q)
{
    float w0 = 2.0f * IMU_FILTER_PI * center_hz / sample_hz;
    float sin_w0 = sinf(w0);
    float cos_w0 = cosf(w0);
    float alpha = sin_w0 / (2.0f * q);
    float a0 = 1.0f + alpha;

    filter->b0 = 1.0f / a0;
    filter->b1 = (-2.0f * cos_w0) / a0;
    filter->b2 = 1.0f / a0;
    filter->a1 = (-2.0f * cos_w0) / a0;
    filter->a2 = (1.0f - alpha) / a0;
    IMUBiquad_ResetState(filter);
}

/*
 * 最简 IMU 滤波器实现：
 * 1) 首帧直接对齐原始值，避免启动瞬态大偏差
 * 2) 后续使用一阶 IIR 低通
 *    y(k) = y(k-1) + alpha * (x(k) - y(k-1))
 */
void IMUFilter_Init(IMUFilter_t *filter)
{
    uint8_t axis;

    if (0 == filter)
    {
        return;
    }

    filter->gyro_raw_x = 0.0f;
    filter->gyro_raw_y = 0.0f;
    filter->gyro_raw_z = 0.0f;
    filter->acc_raw_x = 0.0f;
    filter->acc_raw_y = 0.0f;
    filter->acc_raw_z = 0.0f;

    filter->gyro_filt_x = 0.0f;
    filter->gyro_filt_y = 0.0f;
    filter->gyro_filt_z = 0.0f;
    filter->acc_filt_x = 0.0f;
    filter->acc_filt_y = 0.0f;
    filter->acc_filt_z = 0.0f;

    filter->gyro_ctrl_x = 0.0f;
    filter->gyro_ctrl_y = 0.0f;
    filter->gyro_ctrl_z = 0.0f;
    filter->acc_est_x = 0.0f;
    filter->acc_est_y = 0.0f;
    filter->acc_est_z = 0.0f;
    filter->acc_est_10_x = 0.0f;
    filter->acc_est_10_y = 0.0f;
    filter->acc_est_10_z = 0.0f;

    for (axis = 0U; axis < IMU_FILTER_AXIS_NUM; axis++)
    {
        IMUBiquad_InitNotch(&filter->gyro_notch1[axis], IMU_SAMPLE_RATE_HZ, IMU_GYRO_NOTCH1_HZ, IMU_GYRO_NOTCH_Q);
        IMUBiquad_InitNotch(&filter->gyro_notch2[axis], IMU_SAMPLE_RATE_HZ, IMU_GYRO_NOTCH2_HZ, IMU_GYRO_NOTCH_Q);
        IMUBiquad_InitLowpass(&filter->gyro_lpf[axis], IMU_SAMPLE_RATE_HZ, IMU_GYRO_LOWPASS_HZ);

        IMUBiquad_InitLowpass(&filter->acc_lpf20[axis], IMU_SAMPLE_RATE_HZ, IMU_ACC_LOWPASS_EST_HZ);
        IMUBiquad_InitLowpass(&filter->acc_lpf10[axis], IMU_SAMPLE_RATE_HZ, IMU_ACC_LOWPASS_POS_HZ);
    }

    filter->initialized = 0U;
}

void IMUFilter_Update(IMUFilter_t *filter)
{
    float gyro_in[IMU_FILTER_AXIS_NUM];
    float acc_in[IMU_FILTER_AXIS_NUM];
    float gyro_out[IMU_FILTER_AXIS_NUM];
    float acc20_out[IMU_FILTER_AXIS_NUM];
    float acc10_out[IMU_FILTER_AXIS_NUM];
    uint8_t axis;

    if (0 == filter)
    {
        return;
    }

    if (0U == filter->initialized)
    {
        filter->gyro_filt_x = filter->gyro_raw_x;
        filter->gyro_filt_y = filter->gyro_raw_y;
        filter->gyro_filt_z = filter->gyro_raw_z;

        filter->acc_filt_x = filter->acc_raw_x;
        filter->acc_filt_y = filter->acc_raw_y;
        filter->acc_filt_z = filter->acc_raw_z;

        filter->gyro_ctrl_x = filter->gyro_filt_x;
        filter->gyro_ctrl_y = filter->gyro_filt_y;
        filter->gyro_ctrl_z = filter->gyro_filt_z;

        filter->acc_est_x = filter->acc_filt_x;
        filter->acc_est_y = filter->acc_filt_y;
        filter->acc_est_z = filter->acc_filt_z;

        filter->acc_est_10_x = filter->acc_filt_x;
        filter->acc_est_10_y = filter->acc_filt_y;
        filter->acc_est_10_z = filter->acc_filt_z;

        filter->initialized = 1U;
        return;
    }

    gyro_in[0] = filter->gyro_raw_x;
    gyro_in[1] = filter->gyro_raw_y;
    gyro_in[2] = filter->gyro_raw_z;

    acc_in[0] = filter->acc_raw_x;
    acc_in[1] = filter->acc_raw_y;
    acc_in[2] = filter->acc_raw_z;

    for (axis = 0U; axis < IMU_FILTER_AXIS_NUM; axis++)
    {
        float stage_1 = IMUBiquad_Apply(&filter->gyro_notch1[axis], gyro_in[axis]);
        float stage_2 = IMUBiquad_Apply(&filter->gyro_notch2[axis], stage_1);
        gyro_out[axis] = IMUBiquad_Apply(&filter->gyro_lpf[axis], stage_2);

        acc20_out[axis] = IMUBiquad_Apply(&filter->acc_lpf20[axis], acc_in[axis]);
        acc10_out[axis] = IMUBiquad_Apply(&filter->acc_lpf10[axis], acc20_out[axis]);
    }

    filter->gyro_ctrl_x = gyro_out[0];
    filter->gyro_ctrl_y = gyro_out[1];
    filter->gyro_ctrl_z = gyro_out[2];
    filter->gyro_filt_x = filter->gyro_ctrl_x;
    filter->gyro_filt_y = filter->gyro_ctrl_y;
    filter->gyro_filt_z = filter->gyro_ctrl_z;

    filter->acc_est_x = acc20_out[0];
    filter->acc_est_y = acc20_out[1];
    filter->acc_est_z = acc20_out[2];
    filter->acc_filt_x = filter->acc_est_x;
    filter->acc_filt_y = filter->acc_est_y;
    filter->acc_filt_z = filter->acc_est_z;

    filter->acc_est_10_x = acc10_out[0];
    filter->acc_est_10_y = acc10_out[1];
    filter->acc_est_10_z = acc10_out[2];
}
