#include "MahonyAhrs.h"

#include <math.h>

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

static float Mahony_ApplyDeadband(float value, float deadband)
{
    if ((value > -deadband) && (value < deadband))
    {
        return 0.0f;
    }

    return value;
}

static float Mahony_VectorMagnitude(float x, float y, float z)
{
    float mag_sq = x * x + y * y + z * z;

    if (mag_sq < MAHONY_VECTOR_NORM_MIN)
    {
        return 0.0f;
    }

    return sqrtf(mag_sq);
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

    ahrs->kp = MAHONY_KP_DEFAULT;
    ahrs->ki = MAHONY_KI_DEFAULT;

    ahrs->update_count = 0U;
    ahrs->accel_magnitude = 0.0f;
    ahrs->static_count = 0U;
    ahrs->is_static = 0U;
}

void MahonyAhrs_Update(MahonyAhrs_t *ahrs,
                       float gyro_x, float gyro_y, float gyro_z,
                       float accel_x, float accel_y, float accel_z,
                       float dt)
{
    float gx;
    float gy;
    float gz;
    float accel_mag;
    float gyro_abs_dps;
    float acc_err_g;
    const float gz_deadband_rad = MAHONY_YAW_GZ_DEADBAND_DPS * DEGREES_TO_RADIANS;

    if (dt <= 0.0f)
    {
        return;
    }

    accel_mag = Mahony_VectorMagnitude(accel_x, accel_y, accel_z);
    ahrs->accel_magnitude = accel_mag;

    gyro_abs_dps = Mahony_VectorMagnitude(gyro_x, gyro_y, gyro_z);
    acc_err_g = fabsf(accel_mag - 1.0f);

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
    if (ahrs->is_static != 0U)
    {
        float gz_rad = gyro_z * DEGREES_TO_RADIANS;
        ahrs->gyro_bias_z_static += MAHONY_YAW_BIAS_LP_ALPHA * (gz_rad - ahrs->gyro_bias_z_static);
    }

    gx = gyro_x * DEGREES_TO_RADIANS - ahrs->gyro_bias_x;
    gy = gyro_y * DEGREES_TO_RADIANS - ahrs->gyro_bias_y;
    gz = gyro_z * DEGREES_TO_RADIANS - ahrs->gyro_bias_z_static;
    gz = Mahony_ApplyDeadband(gz, gz_deadband_rad);

    if ((accel_mag > MAHONY_ACCEL_MIN_MAGNITUDE) && (accel_mag < MAHONY_ACCEL_MAX_MAGNITUDE))
    {
        float ax;
        float ay;
        float az;
        float half_vx;
        float half_vy;
        float half_vz;
        float half_ex;
        float half_ey;
#if MAHONY_YAW_CORR_ENABLE
        float half_ez;
#endif
        float accel_trust;
        float kp_eff;

        ax = accel_x / accel_mag;
        ay = accel_y / accel_mag;
        az = accel_z / accel_mag;

#if (MAHONY_INPUT_ACCEL_IS_SPECIFIC_FORCE != 0U)
        /* 加速度计输出比力，方向与重力相反，这里转换为重力方向向量 */
        ax = -ax;
        ay = -ay;
        az = -az;
#endif

        half_vx = ahrs->q1 * ahrs->q3 - ahrs->q0 * ahrs->q2;
        half_vy = ahrs->q0 * ahrs->q1 + ahrs->q2 * ahrs->q3;
        half_vz = ahrs->q0 * ahrs->q0 - 0.5f + ahrs->q3 * ahrs->q3;

        half_ex = ay * half_vz - az * half_vy;
        half_ey = az * half_vx - ax * half_vz;
#if MAHONY_YAW_CORR_ENABLE
        half_ez = ax * half_vy - ay * half_vx;
#endif

        accel_trust = 1.0f - fabsf(accel_mag - 1.0f) / MAHONY_ACCEL_TRUST_BAND_G;
        accel_trust = Mahony_Clamp(accel_trust, 0.0f, 1.0f);

        if ((gyro_abs_dps < MAHONY_BIAS_LEARN_MAX_GYRO_DPS) &&
            (acc_err_g < MAHONY_BIAS_LEARN_MAX_ACC_ERR_G) &&
            (accel_trust > 0.05f))
        {
            float ki_eff = ahrs->ki * accel_trust;

            ahrs->integral_fbx += ki_eff * half_ex * dt;
            ahrs->integral_fby += ki_eff * half_ey * dt;
            ahrs->integral_fbx = Mahony_Clamp(ahrs->integral_fbx, -MAHONY_BIAS_MAX_RAD_S, MAHONY_BIAS_MAX_RAD_S);
            ahrs->integral_fby = Mahony_Clamp(ahrs->integral_fby, -MAHONY_BIAS_MAX_RAD_S, MAHONY_BIAS_MAX_RAD_S);
            ahrs->gyro_bias_x = ahrs->integral_fbx;
            ahrs->gyro_bias_y = ahrs->integral_fby;

#if MAHONY_YAW_CORR_ENABLE
            ahrs->integral_fbz += ki_eff * half_ez * dt;
            ahrs->integral_fbz = Mahony_Clamp(ahrs->integral_fbz, -MAHONY_BIAS_MAX_RAD_S, MAHONY_BIAS_MAX_RAD_S);
            ahrs->gyro_bias_z = ahrs->integral_fbz;
#else
            ahrs->integral_fbz = 0.0f;
            ahrs->gyro_bias_z = 0.0f;
#endif
        }

        kp_eff = ahrs->kp * accel_trust;
        gx += kp_eff * half_ex;
        gy += kp_eff * half_ey;

#if MAHONY_YAW_CORR_ENABLE
        gz += kp_eff * half_ez;
#endif
    }

    {
        float q0 = ahrs->q0;
        float q1 = ahrs->q1;
        float q2 = ahrs->q2;
        float q3 = ahrs->q3;

        ahrs->q0 += (-q1 * gx - q2 * gy - q3 * gz) * 0.5f * dt;
        ahrs->q1 += ( q0 * gx + q2 * gz - q3 * gy) * 0.5f * dt;
        ahrs->q2 += ( q0 * gy - q1 * gz + q3 * gx) * 0.5f * dt;
        ahrs->q3 += ( q0 * gz + q1 * gy - q2 * gx) * 0.5f * dt;
    }

    Mahony_QuaternionNormalize(&ahrs->q0, &ahrs->q1, &ahrs->q2, &ahrs->q3);
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
    ahrs->ki = ki;
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
    ahrs->static_count = 0U;
    ahrs->is_static = 0U;
}
