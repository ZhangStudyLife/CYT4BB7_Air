#include "MahonyAhrs.h"

#include <math.h>

/* 本地差异层: Mahony 解算前对 Z 轴陀螺输入施加对称死区，单位 dps */
#define MAHONY_GYRO_Z_DEADBAND_DPS  0.2f

static float Mahony_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/*
 * 函数名: Mahony_ApplyDeadband
 * 功能: 对输入量施加对称死区，小于等于死区阈值时直接归零
 * 输入参数:
 *   value    - 输入值
 *   deadband - 死区阈值，要求为非负
 * 返回值:
 *   死区处理后的结果
 */
static float Mahony_ApplyDeadband(float value, float deadband)
{
    if (fabsf(value) <= fabsf(deadband))
    {
        return 0.0f;
    }

    return value;
}

static uint8_t Mahony_IsFinite(float value)
{
    if (value != value)
    {
        return 0U;
    }

    if ((value > 1000000.0f) || (value < -1000000.0f))
    {
        return 0U;
    }

    return 1U;
}

static float Mahony_VectorMagnitudeSquared(float x, float y, float z)
{
    return x * x + y * y + z * z;
}

static float Mahony_VectorMagnitude(float x, float y, float z)
{
    float mag_sq = Mahony_VectorMagnitudeSquared(x, y, z);

    if (mag_sq < MAHONY_VECTOR_NORM_MIN)
    {
        return 0.0f;
    }

    return sqrtf(mag_sq);
}

static uint8_t Mahony_NormalizeVector3(float *x, float *y, float *z)
{
    float norm = Mahony_VectorMagnitude(*x, *y, *z);

    if (norm < MAHONY_VECTOR_NORM_MIN)
    {
        return 0U;
    }

    norm = 1.0f / norm;
    *x *= norm;
    *y *= norm;
    *z *= norm;
    return 1U;
}

static void Mahony_QuaternionNormalize(float *q0, float *q1, float *q2, float *q3)
{
    float norm = sqrtf((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));

    if (norm < MAHONY_VECTOR_NORM_MIN)
    {
        *q0 = 1.0f;
        *q1 = 0.0f;
        *q2 = 0.0f;
        *q3 = 0.0f;
        return;
    }

    norm = 1.0f / norm;
    *q0 *= norm;
    *q1 *= norm;
    *q2 *= norm;
    *q3 *= norm;
}

static uint8_t Mahony_QuaternionIsValid(float q0, float q1, float q2, float q3)
{
    float norm_sq;

    if ((0U == Mahony_IsFinite(q0)) || (0U == Mahony_IsFinite(q1)) ||
        (0U == Mahony_IsFinite(q2)) || (0U == Mahony_IsFinite(q3)))
    {
        return 0U;
    }

    norm_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if ((norm_sq < 0.5f) || (norm_sq > 1.5f))
    {
        return 0U;
    }

    return 1U;
}

static float Mahony_Pt1Update(float state, float input, float cutoff_hz, float dt)
{
    float alpha;
    float rc;

    if ((cutoff_hz <= 0.0f) || (dt <= 0.0f))
    {
        return input;
    }

    rc = 1.0f / (2.0f * 3.14159265359f * cutoff_hz);
    alpha = dt / (rc + dt);
    alpha = Mahony_Clamp(alpha, 0.0f, 1.0f);
    return state + alpha * (input - state);
}

static void Mahony_UpdateStaticState(MahonyAhrs_t *ahrs, float gyro_abs_dps, float accel_mag)
{
    float acc_err_g = fabsf(accel_mag - 1.0f);

    /* 本地差异层: 该静止判定供机体侧逻辑使用，不参与 iNav 主 AHRS 公式本身 */

    if ((gyro_abs_dps < MAHONY_STATIC_GYRO_DPS_TH) &&
        (accel_mag > MAHONY_ACCEL_MIN_MAGNITUDE) &&
        (accel_mag < MAHONY_ACCEL_MAX_MAGNITUDE) &&
        (acc_err_g < MAHONY_STATIC_ACC_ERR_G_TH))
    {
        if (ahrs->static_count < 65535U)
        {
            ahrs->static_count++;
        }
    }
    else
    {
        ahrs->static_count = 0U;
    }

    ahrs->is_static = (ahrs->static_count >= MAHONY_STATIC_LOCK_COUNT) ? 1U : 0U;
}

static float Mahony_GetFastGainScale(const MahonyAhrs_t *ahrs)
{
    (void)ahrs;
    return 1.0f;
}

static float Mahony_CalculateAccelWeightNearness(float accel_mag)
{
    float accel_trust = 1.0f - fabsf(accel_mag - 1.0f) / MAHONY_ACCEL_TRUST_BAND_G;

    return Mahony_Clamp(accel_trust, 0.0f, 1.0f);
}

static float Mahony_CalculateAccelWeightRateIgnore(const MahonyAhrs_t *ahrs)
{
    (void)ahrs;
    return 1.0f;
}

static void Mahony_GetEstimatedGravityBody(const MahonyAhrs_t *ahrs,
                                           float *gx,
                                           float *gy,
                                           float *gz)
{
    *gx = 2.0f * (ahrs->q1 * ahrs->q3 - ahrs->q0 * ahrs->q2);
    *gy = 2.0f * (ahrs->q0 * ahrs->q1 + ahrs->q2 * ahrs->q3);
    *gz = ahrs->q0 * ahrs->q0 - ahrs->q1 * ahrs->q1 - ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3;
}

static void Mahony_ResetQuaternionFromGravity(MahonyAhrs_t *ahrs,
                                              float gravity_x,
                                              float gravity_y,
                                              float gravity_z)
{
    float acc_norm;

    acc_norm = Mahony_VectorMagnitude(gravity_x, gravity_y, gravity_z);
    if (acc_norm < MAHONY_VECTOR_NORM_MIN)
    {
        ahrs->q0 = 1.0f;
        ahrs->q1 = 0.0f;
        ahrs->q2 = 0.0f;
        ahrs->q3 = 0.0f;
        return;
    }

    ahrs->q0 = gravity_z + acc_norm;
    ahrs->q1 = gravity_y;
    ahrs->q2 = -gravity_x;
    ahrs->q3 = 0.0f;
    Mahony_QuaternionNormalize(&ahrs->q0, &ahrs->q1, &ahrs->q2, &ahrs->q3);
}

static void Mahony_CheckAndResetQuaternion(MahonyAhrs_t *ahrs,
                                           float prev_q0,
                                           float prev_q1,
                                           float prev_q2,
                                           float prev_q3,
                                           uint8_t acc_valid,
                                           float gravity_x,
                                           float gravity_y,
                                           float gravity_z)
{
    if (0U != Mahony_QuaternionIsValid(ahrs->q0, ahrs->q1, ahrs->q2, ahrs->q3))
    {
        return;
    }

    if (0U != Mahony_QuaternionIsValid(prev_q0, prev_q1, prev_q2, prev_q3))
    {
        ahrs->q0 = prev_q0;
        ahrs->q1 = prev_q1;
        ahrs->q2 = prev_q2;
        ahrs->q3 = prev_q3;
        return;
    }

    if (acc_valid != 0U)
    {
        Mahony_ResetQuaternionFromGravity(ahrs, gravity_x, gravity_y, gravity_z);
        return;
    }

    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
}

void MahonyAhrs_Init(MahonyAhrs_t *ahrs)
{
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;

    ahrs->gyro_bias_x = 0.0f;
    ahrs->gyro_bias_y = 0.0f;
    ahrs->gyro_bias_z = 0.0f;
    ahrs->gyro_bias_z_static = 0.0f;

    ahrs->integral_fbx = 0.0f;
    ahrs->integral_fby = 0.0f;
    ahrs->integral_fbz = 0.0f;

    ahrs->gyro_lpf_x = 0.0f;
    ahrs->gyro_lpf_y = 0.0f;
    ahrs->gyro_lpf_z = 0.0f;
    ahrs->elapsed_time_s = 0.0f;

    ahrs->kp = MAHONY_KP_DEFAULT;
    ahrs->ki = MAHONY_KI_DEFAULT;

    ahrs->update_count = 0U;
    ahrs->accel_magnitude = 0.0f;
    ahrs->acc_weight_nearness = 0.0f;
    ahrs->acc_weight_rate_ignore = 0.0f;
    ahrs->acc_weight_final = 0.0f;
    ahrs->static_count = 0U;
    ahrs->is_static = 0U;
    ahrs->gyro_lpf_ready = 0U;
}

void MahonyAhrs_Update(MahonyAhrs_t *ahrs,
                       float gyro_x, float gyro_y, float gyro_z,
                       float accel_x, float accel_y, float accel_z,
                       float dt)
{
    float prev_q0;
    float prev_q1;
    float prev_q2;
    float prev_q3;
    float measured_gx;
    float measured_gy;
    float measured_gz;
    float rotation_x;
    float rotation_y;
    float rotation_z;
    float accel_mag;
    float gyro_abs_dps;
    float accel_weight;
    float p_correction_x = 0.0f;
    float p_correction_y = 0.0f;
    float p_correction_z = 0.0f;
    float gravity_x = 0.0f;
    float gravity_y = 0.0f;
    float gravity_z = 0.0f;
    uint8_t acc_valid = 0U;
    uint8_t use_acc = 0U;

    if (dt <= 0.0f)
    {
        return;
    }

    ahrs->elapsed_time_s += dt;

    /* 先压掉 Z 轴零点附近的残余漂移，再进入 Mahony 后续静止判定与姿态积分 */
    gyro_z = Mahony_ApplyDeadband(gyro_z, MAHONY_GYRO_Z_DEADBAND_DPS);

    // 算模长
    accel_mag = Mahony_VectorMagnitude(accel_x, accel_y, accel_z);
    ahrs->accel_magnitude = accel_mag;
    ahrs->acc_weight_nearness = 0.0f;
    ahrs->acc_weight_rate_ignore = 0.0f;
    ahrs->acc_weight_final = 0.0f;

    // 模长和陀螺绝对值用于静止判定
    gyro_abs_dps = Mahony_VectorMagnitude(gyro_x, gyro_y, gyro_z);
    // 更新静止状态
    Mahony_UpdateStaticState(ahrs, gyro_abs_dps, accel_mag);

    measured_gx = gyro_x * DEGREES_TO_RADIANS;
    measured_gy = gyro_y * DEGREES_TO_RADIANS;
    measured_gz = gyro_z * DEGREES_TO_RADIANS;

    if (ahrs->gyro_lpf_ready == 0U)
    {
        ahrs->gyro_lpf_x = measured_gx;
        ahrs->gyro_lpf_y = measured_gy;
        ahrs->gyro_lpf_z = measured_gz;
        ahrs->gyro_lpf_ready = 1U;
    }
    else
    {
        ahrs->gyro_lpf_x = Mahony_Pt1Update(ahrs->gyro_lpf_x, measured_gx, MAHONY_ROTATION_LPF_HZ, dt);
        ahrs->gyro_lpf_y = Mahony_Pt1Update(ahrs->gyro_lpf_y, measured_gy, MAHONY_ROTATION_LPF_HZ, dt);
        ahrs->gyro_lpf_z = Mahony_Pt1Update(ahrs->gyro_lpf_z, measured_gz, MAHONY_ROTATION_LPF_HZ, dt);
    }

    prev_q0 = ahrs->q0;
    prev_q1 = ahrs->q1;
    prev_q2 = ahrs->q2;
    prev_q3 = ahrs->q3;

    rotation_x = measured_gx;
    rotation_y = measured_gy;
    rotation_z = measured_gz;

    if ((accel_mag > MAHONY_ACCEL_MIN_MAGNITUDE) && (accel_mag < MAHONY_ACCEL_MAX_MAGNITUDE))
    {
        gravity_x = accel_x / accel_mag;
        gravity_y = accel_y / accel_mag;
        gravity_z = accel_z / accel_mag;

#if (MAHONY_INPUT_ACCEL_IS_SPECIFIC_FORCE != 0U)
        gravity_x = -gravity_x;
        gravity_y = -gravity_y;
        gravity_z = -gravity_z;
#endif

        acc_valid = Mahony_NormalizeVector3(&gravity_x, &gravity_y, &gravity_z);
    }

    accel_weight = 0.0f;
    if (acc_valid != 0U)
    {
        float fast_gain_scale = Mahony_GetFastGainScale(ahrs);
        float acc_weight_nearness = Mahony_CalculateAccelWeightNearness(accel_mag);
        float acc_weight_rate_ignore = Mahony_CalculateAccelWeightRateIgnore(ahrs);

        accel_weight = fast_gain_scale * acc_weight_nearness * acc_weight_rate_ignore;
        ahrs->acc_weight_nearness = acc_weight_nearness;
        ahrs->acc_weight_rate_ignore = acc_weight_rate_ignore;
        ahrs->acc_weight_final = accel_weight;
    }

    use_acc = ((acc_valid != 0U) && (accel_weight > MAHONY_ACCEL_WEIGHT_MIN)) ? 1U : 0U;

    if (use_acc != 0U)
    {
        float est_gravity_x;
        float est_gravity_y;
        float est_gravity_z;
        float err_x;
        float err_y;
        float err_z;

        Mahony_GetEstimatedGravityBody(ahrs, &est_gravity_x, &est_gravity_y, &est_gravity_z);

        err_x = gravity_y * est_gravity_z - gravity_z * est_gravity_y;
        err_y = gravity_z * est_gravity_x - gravity_x * est_gravity_z;
        err_z = gravity_x * est_gravity_y - gravity_y * est_gravity_x;

        p_correction_x = err_x * ahrs->kp * accel_weight;
        p_correction_y = err_y * ahrs->kp * accel_weight;
        p_correction_z = err_z * ahrs->kp * accel_weight;
    }

    ahrs->integral_fbx = 0.0f;
    ahrs->integral_fby = 0.0f;
    ahrs->integral_fbz = 0.0f;
    ahrs->gyro_bias_x = 0.0f;
    ahrs->gyro_bias_y = 0.0f;
    ahrs->gyro_bias_z = 0.0f;

    rotation_x += p_correction_x;
    rotation_y += p_correction_y;
    rotation_z += p_correction_z;

    {
        float theta_x = rotation_x * 0.5f * dt;
        float theta_y = rotation_y * 0.5f * dt;
        float theta_z = rotation_z * 0.5f * dt;
        float theta_mag_sq = Mahony_VectorMagnitudeSquared(theta_x, theta_y, theta_z);

        if (theta_mag_sq >= MAHONY_QUAT_MIN_UPDATE)
        {
            float delta_q0;
            float delta_q1 = theta_x;
            float delta_q2 = theta_y;
            float delta_q3 = theta_z;

            if (theta_mag_sq < MAHONY_QUAT_SMALL_ANGLE_THRESHOLD)
            {
                float scale = 1.0f - theta_mag_sq / 6.0f;

                delta_q0 = 1.0f - theta_mag_sq / 2.0f;
                delta_q1 *= scale;
                delta_q2 *= scale;
                delta_q3 *= scale;
            }
            else
            {
                float theta_mag = sqrtf(theta_mag_sq);
                float scale = sinf(theta_mag) / theta_mag;

                delta_q0 = cosf(theta_mag);
                delta_q1 *= scale;
                delta_q2 *= scale;
                delta_q3 *= scale;
            }

            ahrs->q0 = prev_q0 * delta_q0 - prev_q1 * delta_q1 - prev_q2 * delta_q2 - prev_q3 * delta_q3;
            ahrs->q1 = prev_q0 * delta_q1 + prev_q1 * delta_q0 + prev_q2 * delta_q3 - prev_q3 * delta_q2;
            ahrs->q2 = prev_q0 * delta_q2 - prev_q1 * delta_q3 + prev_q2 * delta_q0 + prev_q3 * delta_q1;
            ahrs->q3 = prev_q0 * delta_q3 + prev_q1 * delta_q2 - prev_q2 * delta_q1 + prev_q3 * delta_q0;

            Mahony_QuaternionNormalize(&ahrs->q0, &ahrs->q1, &ahrs->q2, &ahrs->q3);
        }
    }

    Mahony_CheckAndResetQuaternion(ahrs,
                                   prev_q0, prev_q1, prev_q2, prev_q3,
                                   acc_valid,
                                   gravity_x, gravity_y, gravity_z);

    ahrs->update_count++;
}

void MahonyAhrs_GetQuaternion(const MahonyAhrs_t *ahrs,
                              float *q0, float *q1, float *q2, float *q3)
{
    *q0 = ahrs->q0;
    *q1 = ahrs->q1;
    *q2 = ahrs->q2;
    *q3 = ahrs->q3;
}

void MahonyAhrs_GetEuler(const MahonyAhrs_t *ahrs,
                         float *roll, float *pitch, float *yaw)
{
    float q0 = ahrs->q0;
    float q1 = ahrs->q1;
    float q2 = ahrs->q2;
    float q3 = ahrs->q3;
    float sin_pitch;
    

    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2));

    sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    sin_pitch = Mahony_Clamp(sin_pitch, -1.0f, 1.0f);
    *pitch = asinf(sin_pitch);

    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

MahonyAhrs_Euler_t MahonyAhrs_GetEulerDegrees(const MahonyAhrs_t *ahrs)
{
    MahonyAhrs_Euler_t euler;

    MahonyAhrs_GetEuler(ahrs, &euler.roll, &euler.pitch, &euler.yaw);

    euler.sin_roll = sinf(euler.roll);
    euler.cos_roll = cosf(euler.roll);
    euler.sin_pitch = sinf(euler.pitch);
    euler.cos_pitch = cosf(euler.pitch);

    euler.roll *= RADIANS_TO_DEGREES;
    euler.pitch *= RADIANS_TO_DEGREES;
    euler.yaw *= RADIANS_TO_DEGREES;

    return euler;
}

void MahonyAhrs_SetGains(MahonyAhrs_t *ahrs, float kp, float ki)
{
    if (kp < 0.0f)
    {
        kp = 0.0f;
    }

    if (ki < 0.0f)
    {
        ki = 0.0f;
    }

    ahrs->kp = kp;
    (void)ki;
    ahrs->ki = 0.0f;
}

void MahonyAhrs_ResetQuaternion(MahonyAhrs_t *ahrs)
{
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;

    ahrs->update_count = 0U;
}

void MahonyAhrs_ResetBias(MahonyAhrs_t *ahrs)
{
    ahrs->gyro_bias_x = 0.0f;
    ahrs->gyro_bias_y = 0.0f;
    ahrs->gyro_bias_z = 0.0f;
    ahrs->gyro_bias_z_static = 0.0f;

    ahrs->integral_fbx = 0.0f;
    ahrs->integral_fby = 0.0f;
    ahrs->integral_fbz = 0.0f;
    ahrs->gyro_lpf_x = 0.0f;
    ahrs->gyro_lpf_y = 0.0f;
    ahrs->gyro_lpf_z = 0.0f;
    ahrs->acc_weight_nearness = 0.0f;
    ahrs->acc_weight_rate_ignore = 0.0f;
    ahrs->acc_weight_final = 0.0f;
    ahrs->static_count = 0U;
    ahrs->is_static = 0U;
    ahrs->gyro_lpf_ready = 0U;
}
