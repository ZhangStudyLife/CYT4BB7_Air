/********************************************************************
 * 文件名  : accel_calibration.c
 * 说明    : ICM42688 加速度标定与垂向加速度预处理
 * 核心流程: rotate -> bias/scale -> 去重力 -> 垂向投影 -> 低通 -> 积分
 * 关键入口:
 *   1) AccelCalibration_Start()      启动静止标定
 *   2) AccelCalibration_Update2kHz() 实时更新（2kHz）
 ********************************************************************/

#include "Accel_Calibration.h"
#include "../Attitude/IMU_TOP.h"
#include "zf_common_headfile.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define DEG_TO_RAD                               (0.017453292519943295f)

#define ACC_DOWN_LPF_ALPHA_EKF                   (0.3f)
#define ACC_DOWN_LPF_ALPHA_OUTPUT                (0.1f)

#define IMU_ACCEL_G_MAX_ABS                      (20.0f)
#define IMU_GYRO_DPS_MAX_ABS                     (6000.0f)
#define CALIB_MAX_TRY_SAMPLES                    (6000U)
#define ACCEL_DOWN_SIGN_FOR_EKF                  (+1.0f)

/* AP 风格：分窗口收敛判定 */
#define ACCEL_CALIBRATION_WINDOW_SAMPLES         (400U)
#define ACCEL_CALIBRATION_MAX_WINDOWS            (24U)
#define ACCEL_CALIBRATION_CONVERGE_WINDOWS       (3U)
#define ACCEL_CALIBRATION_STATIC_ACCEL_MIN_G     (0.80f)
#define ACCEL_CALIBRATION_STATIC_ACCEL_MAX_G     (1.20f)
#define ACCEL_CALIBRATION_STATIC_GYRO_MAX_DPS    (2.5f)
#define ACCEL_CALIBRATION_STATIC_GYRO_STD_MAX_DPS (0.80f)
#define ACCEL_CALIBRATION_CONVERGE_BIAS_DELTA_G  (0.004f)
#define ACCEL_CALIBRATION_CONVERGE_ACC_STD_G     (0.025f)

/* 在线微调：仅在静止窗口内慢速修正 bias */
#define ACCEL_CALIBRATION_ONLINE_BIAS_ALPHA      (0.0025f)
#define ACCEL_CALIBRATION_ONLINE_GYRO_MAX_DPS    (1.2f)
#define ACCEL_CALIBRATION_ONLINE_ACCEL_ERR_MAX_G (0.08f)
#define ACCEL_CALIBRATION_BIAS_MAX_G             (0.35f)
#define ACCEL_CALIBRATION_SCALE_MIN              (0.85f)
#define ACCEL_CALIBRATION_SCALE_MAX              (1.15f)
#define ACCEL_CALIBRATION_START_ACCEPT_STD_G     (0.045f)
#define ACCEL_CALIBRATION_ONLINE_SCALE_ALPHA     (0.0015f)
#define ACCEL_CALIBRATION_ONLINE_SCALE_ERR_MAX_G (0.10f)
#define ACCEL_CALIBRATION_QUALITY_ALPHA_STATIC   (0.0020f)
#define ACCEL_CALIBRATION_QUALITY_ALPHA_DYNAMIC  (0.0120f)

AccelCalibration_t g_accel_calibration = {0};

/* ========================= 基础工具函数 ========================= */

static float clampf_local(float v, float min_v, float max_v)
{
    if (v < min_v)
    {
        return min_v;
    }
    if (v > max_v)
    {
        return max_v;
    }
    return v;
}

static bool is_finitef_local(float v)
{
    return !(isnan(v) || isinf(v));
}

static float vec3_norm(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static float fabsf_local(float v)
{
    return (v >= 0.0f) ? v : -v;
}

static void set_identity_matrix(float matrix[3][3])
{
    matrix[0][0] = 1.0f; matrix[0][1] = 0.0f; matrix[0][2] = 0.0f;
    matrix[1][0] = 0.0f; matrix[1][1] = 1.0f; matrix[1][2] = 0.0f;
    matrix[2][0] = 0.0f; matrix[2][1] = 0.0f; matrix[2][2] = 1.0f;
}

static bool matrix_is_identity(const float matrix[3][3])
{
    const float eps = 1.0e-6f;
    if (fabsf_local(matrix[0][0] - 1.0f) > eps || fabsf_local(matrix[1][1] - 1.0f) > eps || fabsf_local(matrix[2][2] - 1.0f) > eps)
    {
        return false;
    }
    if (fabsf_local(matrix[0][1]) > eps || fabsf_local(matrix[0][2]) > eps ||
        fabsf_local(matrix[1][0]) > eps || fabsf_local(matrix[1][2]) > eps ||
        fabsf_local(matrix[2][0]) > eps || fabsf_local(matrix[2][1]) > eps)
    {
        return false;
    }
    return true;
}

static void mat3_mul_vec(const float matrix[3][3], const float vec_in[3], float vec_out[3])
{
    vec_out[0] = matrix[0][0] * vec_in[0] + matrix[0][1] * vec_in[1] + matrix[0][2] * vec_in[2];
    vec_out[1] = matrix[1][0] * vec_in[0] + matrix[1][1] * vec_in[1] + matrix[1][2] * vec_in[2];
    vec_out[2] = matrix[2][0] * vec_in[0] + matrix[2][1] * vec_in[1] + matrix[2][2] * vec_in[2];
}

static bool euler_ready(void)
{
    /* 用 sin^2+cos^2≈1 判断姿态三角量是否有效，避免未初始化姿态参与计算 */
    const float s2r = g_euler.sin_roll * g_euler.sin_roll;
    const float c2r = g_euler.cos_roll * g_euler.cos_roll;
    const float s2p = g_euler.sin_pitch * g_euler.sin_pitch;
    const float c2p = g_euler.cos_pitch * g_euler.cos_pitch;

    if (!is_finitef_local(g_euler.sin_roll) || !is_finitef_local(g_euler.cos_roll) ||
        !is_finitef_local(g_euler.sin_pitch) || !is_finitef_local(g_euler.cos_pitch))
    {
        return false;
    }

    if (fabsf_local((s2r + c2r) - 1.0f) > 0.2f || fabsf_local((s2p + c2p) - 1.0f) > 0.2f)
    {
        return false;
    }

    return true;
}

static void get_gravity_body_g(float *gx, float *gy, float *gz)
{
    if ((gx == NULL) || (gy == NULL) || (gz == NULL))
    {
        return;
    }

    /* 单位向量（g）：重力在机体系的分量 */
    *gx = -g_euler.sin_pitch;
    *gy = g_euler.sin_roll * g_euler.cos_pitch;
    *gz = g_euler.cos_roll * g_euler.cos_pitch;
}

static float calc_accel_down_from_body(const float accel_body_mps2[3])
{
    /* 使用姿态矩阵第三行，将机体系加速度投影到 NED 的 Down 轴 */
    const float sin_pitch = g_euler.sin_pitch;
    const float cos_pitch = g_euler.cos_pitch;
    const float sin_roll = g_euler.sin_roll;
    const float cos_roll = g_euler.cos_roll;

    const float r31 = -cos_roll * sin_pitch;
    const float r32 = sin_roll;
    const float r33 = cos_roll * cos_pitch;

    return r31 * accel_body_mps2[0] +
           r32 * accel_body_mps2[1] +
           r33 * accel_body_mps2[2];
}

static void rotate_body_linear_to_level(const float accel_body_mps2[3], float accel_level_mps2[3])
{
    const float sin_pitch = g_euler.sin_pitch;
    const float cos_pitch = g_euler.cos_pitch;
    const float sin_roll = g_euler.sin_roll;
    const float cos_roll = g_euler.cos_roll;
    float r11;
    float r12;
    float r13;
    float r21;
    float r22;
    float r23;
    float r31;
    float r32;
    float r33;

    if ((accel_body_mps2 == NULL) || (accel_level_mps2 == NULL))
    {
        return;
    }

#if ACCEL_CALIBRATION_LEVEL_USE_YAW
    const float yaw_rad = g_euler.yaw * DEG_TO_RAD;
    const float sin_yaw = sinf(yaw_rad);
    const float cos_yaw = cosf(yaw_rad);

    r11 = cos_pitch * cos_yaw;
    r12 = cos_pitch * sin_yaw;
    r13 = -sin_pitch;

    r21 = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
    r22 = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
    r23 = sin_roll * cos_pitch;

    r31 = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;
    r32 = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;
    r33 = cos_roll * cos_pitch;
#else
    /* yaw=0: 仅补偿 roll/pitch，满足当前“倾斜解耦”需求 */
    r11 = cos_pitch;
    r12 = 0.0f;
    r13 = -sin_pitch;

    r21 = sin_roll * sin_pitch;
    r22 = cos_roll;
    r23 = sin_roll * cos_pitch;

    r31 = cos_roll * sin_pitch;
    r32 = -sin_roll;
    r33 = cos_roll * cos_pitch;
#endif

    accel_level_mps2[0] = r11 * accel_body_mps2[0] + r12 * accel_body_mps2[1] + r13 * accel_body_mps2[2];
    accel_level_mps2[1] = r21 * accel_body_mps2[0] + r22 * accel_body_mps2[1] + r23 * accel_body_mps2[2];
    accel_level_mps2[2] = r31 * accel_body_mps2[0] + r32 * accel_body_mps2[1] + r33 * accel_body_mps2[2];
}

static void sanitize_scale(void)
{
    /* 防止 scale 非法（NaN/Inf/过小），并限制在安全范围 */
    uint8_t i;
    for (i = 0U; i < 3U; i++)
    {
        if (!is_finitef_local(g_accel_calibration.accel_scale[i]) || (fabsf_local(g_accel_calibration.accel_scale[i]) < 1.0e-6f))
        {
            g_accel_calibration.accel_scale[i] = 1.0f;
        }
        else
        {
            g_accel_calibration.accel_scale[i] = clampf_local(
                g_accel_calibration.accel_scale[i],
                ACCEL_CALIBRATION_SCALE_MIN,
                ACCEL_CALIBRATION_SCALE_MAX);
        }
    }
}

static void clamp_bias(void)
{
    uint8_t i;
    for (i = 0U; i < 3U; i++)
    {
        g_accel_calibration.accel_bias_g[i] = clampf_local(
            g_accel_calibration.accel_bias_g[i],
            -ACCEL_CALIBRATION_BIAS_MAX_G,
            ACCEL_CALIBRATION_BIAS_MAX_G);
    }
}

static float mean_scale(void)
{
    return (g_accel_calibration.accel_scale[0] +
            g_accel_calibration.accel_scale[1] +
            g_accel_calibration.accel_scale[2]) / 3.0f;
}

static void apply_uniform_scale(float scale)
{
    uint8_t i;
    const float scale_limited = clampf_local(scale,
                                             ACCEL_CALIBRATION_SCALE_MIN,
                                             ACCEL_CALIBRATION_SCALE_MAX);

    for (i = 0U; i < 3U; i++)
    {
        g_accel_calibration.accel_scale[i] = scale_limited;
    }
}

static bool imu_sample_valid(float ax, float ay, float az, float gx, float gy, float gz)
{
    /* 统一样本有效性门限：数值有效 + 量程合理 + 姿态可用 */
    if (!is_finitef_local(ax) || !is_finitef_local(ay) || !is_finitef_local(az) ||
        !is_finitef_local(gx) || !is_finitef_local(gy) || !is_finitef_local(gz))
    {
        return false;
    }

    if (fabsf_local(ax) > IMU_ACCEL_G_MAX_ABS || fabsf_local(ay) > IMU_ACCEL_G_MAX_ABS || fabsf_local(az) > IMU_ACCEL_G_MAX_ABS)
    {
        return false;
    }

    if (fabsf_local(gx) > IMU_GYRO_DPS_MAX_ABS || fabsf_local(gy) > IMU_GYRO_DPS_MAX_ABS || fabsf_local(gz) > IMU_GYRO_DPS_MAX_ABS)
    {
        return false;
    }

    return euler_ready();
}

static void rotate_imu_to_body(const float vec_in[3], float vec_out[3])
{
    /* 坐标统一：后续所有标定与估计都在机体系下完成 */
    if (g_accel_calibration.imu_to_body_identity)
    {
        vec_out[0] = vec_in[0];
        vec_out[1] = vec_in[1];
        vec_out[2] = vec_in[2];
    }
    else
    {
        mat3_mul_vec(g_accel_calibration.imu_to_body, vec_in, vec_out);
    }
}

static void update_running_stats(float sample, float *mean, float *m2, uint32_t sample_count)
{
    float delta;
    float delta2;

    if ((mean == NULL) || (m2 == NULL) || (sample_count == 0U))
    {
        return;
    }

    delta = sample - *mean;
    *mean += delta / (float)sample_count;
    delta2 = sample - *mean;
    *m2 += delta * delta2;
}

static float std_from_m2(float m2, uint32_t sample_count)
{
    if (sample_count < 2U)
    {
        return 0.0f;
    }
    return sqrtf(m2 / (float)(sample_count - 1U));
}

static bool static_calibration_sample_valid(const float accel_body_g[3],
                                            const float gyro_body_dps[3],
                                            float *acc_norm_g,
                                            float *gyro_norm_dps)
{
    /* 启动标定仅接受“近似静止”样本：|a|≈1g 且角速度很小 */
    float accel_norm;
    float gyro_norm;

    if ((accel_body_g == NULL) || (gyro_body_dps == NULL))
    {
        return false;
    }

    accel_norm = vec3_norm(accel_body_g[0], accel_body_g[1], accel_body_g[2]);
    gyro_norm = vec3_norm(gyro_body_dps[0], gyro_body_dps[1], gyro_body_dps[2]);

    if (!is_finitef_local(accel_norm) || !is_finitef_local(gyro_norm))
    {
        return false;
    }

    if ((accel_norm < ACCEL_CALIBRATION_STATIC_ACCEL_MIN_G) ||
        (accel_norm > ACCEL_CALIBRATION_STATIC_ACCEL_MAX_G))
    {
        return false;
    }

    if (gyro_norm > ACCEL_CALIBRATION_STATIC_GYRO_MAX_DPS)
    {
        return false;
    }

    if (acc_norm_g != NULL)
    {
        *acc_norm_g = accel_norm;
    }
    if (gyro_norm_dps != NULL)
    {
        *gyro_norm_dps = gyro_norm;
    }

    return true;
}

static void update_bias_online(const float accel_body_g[3],
                               const float gyro_body_dps[3],
                               float gravity_x_g,
                               float gravity_y_g,
                               float gravity_z_g)
{
    /* 在线慢速 bias 微调：只在静止窗口内生效，防止动态误修正 */
    float target_bias[3];
    float accel_norm;
    float gyro_norm;
    uint8_t i;

    if ((accel_body_g == NULL) || (gyro_body_dps == NULL))
    {
        return;
    }

    accel_norm = vec3_norm(accel_body_g[0], accel_body_g[1], accel_body_g[2]);
    gyro_norm = vec3_norm(gyro_body_dps[0], gyro_body_dps[1], gyro_body_dps[2]);

    if (!is_finitef_local(accel_norm) || !is_finitef_local(gyro_norm))
    {
        return;
    }

    if (gyro_norm > ACCEL_CALIBRATION_ONLINE_GYRO_MAX_DPS)
    {
        return;
    }

    if (fabsf_local(accel_norm - 1.0f) > ACCEL_CALIBRATION_ONLINE_ACCEL_ERR_MAX_G)
    {
        return;
    }

    target_bias[0] = accel_body_g[0] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_x_g;
    target_bias[1] = accel_body_g[1] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_y_g;
    target_bias[2] = accel_body_g[2] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_z_g;

    for (i = 0U; i < 3U; i++)
    {
        g_accel_calibration.accel_bias_g[i] +=
            ACCEL_CALIBRATION_ONLINE_BIAS_ALPHA * (target_bias[i] - g_accel_calibration.accel_bias_g[i]);
    }

    clamp_bias();
}

static void update_scale_online(const float accel_body_g[3], const float gyro_body_dps[3])
{
    /* 在线慢速 scale 微调：让去偏置后的 |a| 逐步逼近 1g */
    float accel_unbiased_g[3];
    float accel_norm_g;
    float gyro_norm_dps;
    float cur_scale;
    float target_scale;

    if ((accel_body_g == NULL) || (gyro_body_dps == NULL))
    {
        return;
    }

    gyro_norm_dps = vec3_norm(gyro_body_dps[0], gyro_body_dps[1], gyro_body_dps[2]);
    if (!is_finitef_local(gyro_norm_dps) || (gyro_norm_dps > ACCEL_CALIBRATION_ONLINE_GYRO_MAX_DPS))
    {
        return;
    }

    accel_unbiased_g[0] = accel_body_g[0] - g_accel_calibration.accel_bias_g[0];
    accel_unbiased_g[1] = accel_body_g[1] - g_accel_calibration.accel_bias_g[1];
    accel_unbiased_g[2] = accel_body_g[2] - g_accel_calibration.accel_bias_g[2];
    accel_norm_g = vec3_norm(accel_unbiased_g[0], accel_unbiased_g[1], accel_unbiased_g[2]);

    if (!is_finitef_local(accel_norm_g) || (accel_norm_g < 0.2f))
    {
        return;
    }

    if (fabsf_local(accel_norm_g - 1.0f) > ACCEL_CALIBRATION_ONLINE_SCALE_ERR_MAX_G)
    {
        return;
    }

    cur_scale = mean_scale();
    if (!is_finitef_local(cur_scale) || (cur_scale < 1.0e-6f))
    {
        cur_scale = 1.0f;
    }

    target_scale = cur_scale / accel_norm_g;
    target_scale = clampf_local(target_scale,
                                ACCEL_CALIBRATION_SCALE_MIN,
                                ACCEL_CALIBRATION_SCALE_MAX);

    cur_scale += ACCEL_CALIBRATION_ONLINE_SCALE_ALPHA * (target_scale - cur_scale);
    apply_uniform_scale(cur_scale);
}

static void update_runtime_quality(const float accel_corrected_body_g[3], const float gyro_body_dps[3])
{
    /* 运行质量评估：统计 |a| 的均值和“近似标准差”用于健康度观察 */
    float accel_norm_g;
    float gyro_norm_dps;
    float mean;
    float std;
    float alpha;
    float dev;

    if ((accel_corrected_body_g == NULL) || (gyro_body_dps == NULL))
    {
        return;
    }

    accel_norm_g = vec3_norm(accel_corrected_body_g[0],
                             accel_corrected_body_g[1],
                             accel_corrected_body_g[2]);
    gyro_norm_dps = vec3_norm(gyro_body_dps[0],
                              gyro_body_dps[1],
                              gyro_body_dps[2]);

    if (!is_finitef_local(accel_norm_g) || !is_finitef_local(gyro_norm_dps))
    {
        return;
    }

    alpha = (gyro_norm_dps < ACCEL_CALIBRATION_ONLINE_GYRO_MAX_DPS) ?
            ACCEL_CALIBRATION_QUALITY_ALPHA_STATIC :
            ACCEL_CALIBRATION_QUALITY_ALPHA_DYNAMIC;

    mean = g_accel_calibration.accel_norm_mean_g;
    std = g_accel_calibration.accel_norm_std_g;

    if (!is_finitef_local(mean) || (mean <= 0.0f))
    {
        mean = accel_norm_g;
    }
    if (!is_finitef_local(std) || (std < 0.0f))
    {
        std = 0.0f;
    }

    mean += alpha * (accel_norm_g - mean);
    dev = fabsf_local(accel_norm_g - mean);
    std += alpha * (dev - std);

    g_accel_calibration.accel_norm_mean_g = clampf_local(mean, 0.6f, 1.4f);
    g_accel_calibration.accel_norm_std_g = clampf_local(std, 0.0f, 0.25f);
}

void AccelCalibration_Init(void)
{
    /* 初始化等价于复位 */
    AccelCalibration_Reset();
}

/* Reset calibration but keep IMU to body rotation matrix if valid */
void AccelCalibration_Reset(void)
{
    float saved_matrix[3][3];
    bool matrix_valid = false;
    uint8_t i;
    uint8_t j;

    /* 重要：复位时保留已有安装矩阵，避免重复配置 IMU 安装方向 */
    memcpy(saved_matrix, g_accel_calibration.imu_to_body, sizeof(saved_matrix));

    for (i = 0U; i < 3U; i++)
    {
        for (j = 0U; j < 3U; j++)
        {
            if (saved_matrix[i][j] != 0.0f)
            {
                matrix_valid = true;
            }
        }
    }

    memset(&g_accel_calibration, 0, sizeof(g_accel_calibration));

    if (matrix_valid)
    {
        memcpy(g_accel_calibration.imu_to_body, saved_matrix, sizeof(saved_matrix));
    }
    else
    {
        set_identity_matrix(g_accel_calibration.imu_to_body);
    }

    g_accel_calibration.imu_to_body_identity = matrix_is_identity(g_accel_calibration.imu_to_body);

    g_accel_calibration.accel_scale[0] = 1.0f;
    g_accel_calibration.accel_scale[1] = 1.0f;
    g_accel_calibration.accel_scale[2] = 1.0f;

    g_accel_calibration.gravity_mps2 = ACCEL_CALIBRATION_GRAVITY_MSS;
}

void AccelCalibration_SetImuToBodyMatrix(const float matrix[3][3])
{
    if (matrix == NULL)
    {
        return;
    }

    memcpy(g_accel_calibration.imu_to_body, matrix, sizeof(g_accel_calibration.imu_to_body));
    g_accel_calibration.imu_to_body_identity = matrix_is_identity(g_accel_calibration.imu_to_body);
}

void AccelCalibration_SetImuToBodyEulerDeg(float roll_deg, float pitch_deg, float yaw_deg)
{
    /* ZYX 欧拉角转旋转矩阵（yaw->pitch->roll） */
    const float roll = roll_deg * DEG_TO_RAD;
    const float pitch = pitch_deg * DEG_TO_RAD;
    const float yaw = yaw_deg * DEG_TO_RAD;

    const float sr = sinf(roll);
    const float cr = cosf(roll);
    const float sp = sinf(pitch);
    const float cp = cosf(pitch);
    const float sy = sinf(yaw);
    const float cy = cosf(yaw);

    g_accel_calibration.imu_to_body[0][0] = cy * cp;
    g_accel_calibration.imu_to_body[0][1] = cy * sp * sr - sy * cr;
    g_accel_calibration.imu_to_body[0][2] = cy * sp * cr + sy * sr;

    g_accel_calibration.imu_to_body[1][0] = sy * cp;
    g_accel_calibration.imu_to_body[1][1] = sy * sp * sr + cy * cr;
    g_accel_calibration.imu_to_body[1][2] = sy * sp * cr - cy * sr;

    g_accel_calibration.imu_to_body[2][0] = -sp;
    g_accel_calibration.imu_to_body[2][1] = cp * sr;
    g_accel_calibration.imu_to_body[2][2] = cp * cr;

    g_accel_calibration.imu_to_body_identity = matrix_is_identity(g_accel_calibration.imu_to_body);
}

/* ========================= 重要函数：启动标定 =========================
 * AP 风格：静止窗口批量校准。
 * - 每个窗口收集固定数量静止样本
 * - 用窗口内稳定性和 bias 大小评分，选最优窗口
 * - 连续多个窗口收敛则判定标定成功
 */
bool AccelCalibration_Start(void)
{
    uint8_t window_idx;
    uint8_t converged_windows = 0U;
    uint32_t total_valid_samples = 0U;
    uint32_t total_tries = 0U;
    bool have_prev_window = false;
    bool have_best_window = false;
    bool converged = false;

    float prev_bias[3] = {0.0f, 0.0f, 0.0f};
    float selected_bias[3] = {0.0f, 0.0f, 0.0f};
    float best_bias[3] = {0.0f, 0.0f, 0.0f};
    float best_score = FLT_MAX;

    float global_mean_norm = 0.0f;
    float global_m2_norm = 0.0f;

    /* 每次启动标定前清空历史状态 */
    AccelCalibration_Reset();

    for (window_idx = 0U;
         (window_idx < ACCEL_CALIBRATION_MAX_WINDOWS) && (total_tries < CALIB_MAX_TRY_SAMPLES);
         window_idx++)
    {
        uint16_t window_valid = 0U;
        uint16_t window_tries = 0U;

        float window_bias_sum_x = 0.0f;
        float window_bias_sum_y = 0.0f;
        float window_bias_sum_z = 0.0f;
        float window_m2_norm = 0.0f;
        float window_mean_norm = 0.0f;
        float window_m2_gyro = 0.0f;
        float window_mean_gyro = 0.0f;

        while ((window_valid < ACCEL_CALIBRATION_WINDOW_SAMPLES) &&
               (window_tries < (uint16_t)(ACCEL_CALIBRATION_WINDOW_SAMPLES * 6U)) &&
               (total_tries < CALIB_MAX_TRY_SAMPLES))
        {
            float accel_sensor_g[3];
            float gyro_sensor_dps[3];
            float accel_body_g[3];
            float gyro_body_dps[3];
            float gx;
            float gy;
            float gz;
            float accel_norm_g;
            float gyro_norm_dps;

            /* 读取一帧 IMU 2kHz 数据 */
            IMU_Update_2kHz();
            window_tries++;
            total_tries++;

            accel_sensor_g[0] = g_imu_filter.acc_filt_x;
            accel_sensor_g[1] = g_imu_filter.acc_filt_y;
            accel_sensor_g[2] = g_imu_filter.acc_filt_z;

            gyro_sensor_dps[0] = g_imu_filter.gyro_filt_x;
            gyro_sensor_dps[1] = g_imu_filter.gyro_filt_y;
            gyro_sensor_dps[2] = g_imu_filter.gyro_filt_z;

            /* 先做基础有效性筛选 */
            if (!imu_sample_valid(accel_sensor_g[0], accel_sensor_g[1], accel_sensor_g[2],
                                  gyro_sensor_dps[0], gyro_sensor_dps[1], gyro_sensor_dps[2]))
            {
                g_accel_calibration.invalid_sample_count++;
                continue;
            }

            rotate_imu_to_body(accel_sensor_g, accel_body_g);
            rotate_imu_to_body(gyro_sensor_dps, gyro_body_dps);

            /* 再做静止条件筛选（|a|≈1g 且角速度小） */
            if (!static_calibration_sample_valid(accel_body_g, gyro_body_dps, &accel_norm_g, &gyro_norm_dps))
            {
                g_accel_calibration.invalid_sample_count++;
                continue;
            }

            get_gravity_body_g(&gx, &gy, &gz);

            /* 在机体系下估计 bias = measured - static_sign * gravity */
            window_bias_sum_x += (accel_body_g[0] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gx);
            window_bias_sum_y += (accel_body_g[1] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gy);
            window_bias_sum_z += (accel_body_g[2] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gz);

            window_valid++;
            total_valid_samples++;

            update_running_stats(accel_norm_g, &window_mean_norm, &window_m2_norm, window_valid);
            update_running_stats(gyro_norm_dps, &window_mean_gyro, &window_m2_gyro, window_valid);
            update_running_stats(accel_norm_g, &global_mean_norm, &global_m2_norm, total_valid_samples);
        }

        if (window_valid < ACCEL_CALIBRATION_WINDOW_SAMPLES)
        {
            break;
        }

        {
            float current_bias[3];
            float bias_delta = 0.0f;
            float window_acc_std = std_from_m2(window_m2_norm, window_valid);
            float window_gyro_std = std_from_m2(window_m2_gyro, window_valid);
            float score;

            current_bias[0] = window_bias_sum_x / (float)window_valid;
            current_bias[1] = window_bias_sum_y / (float)window_valid;
            current_bias[2] = window_bias_sum_z / (float)window_valid;

                /* 评分越小越好：加速度波动小、角速度波动小、bias 小 */
                score = window_acc_std +
                    0.25f * window_gyro_std +
                    0.50f * vec3_norm(current_bias[0], current_bias[1], current_bias[2]);

            if (!have_best_window || (score < best_score))
            {
                best_score = score;
                best_bias[0] = current_bias[0];
                best_bias[1] = current_bias[1];
                best_bias[2] = current_bias[2];
                have_best_window = true;
            }

            if (have_prev_window)
            {
                bias_delta = vec3_norm(current_bias[0] - prev_bias[0],
                                       current_bias[1] - prev_bias[1],
                                       current_bias[2] - prev_bias[2]);
            }

            if (have_prev_window &&
                (bias_delta < ACCEL_CALIBRATION_CONVERGE_BIAS_DELTA_G) &&
                (window_acc_std < ACCEL_CALIBRATION_CONVERGE_ACC_STD_G) &&
                (window_gyro_std < ACCEL_CALIBRATION_STATIC_GYRO_STD_MAX_DPS))
            {
                converged_windows++;
            }
            else
            {
                converged_windows = 0U;
            }

            prev_bias[0] = current_bias[0];
            prev_bias[1] = current_bias[1];
            prev_bias[2] = current_bias[2];
            selected_bias[0] = current_bias[0];
            selected_bias[1] = current_bias[1];
            selected_bias[2] = current_bias[2];
            have_prev_window = true;

            /* 连续收敛窗口达到阈值后结束 */
            if ((total_valid_samples >= ACCEL_CALIBRATION_SAMPLES) &&
                (converged_windows >= ACCEL_CALIBRATION_CONVERGE_WINDOWS))
            {
                converged = true;
                break;
            }
        }
    }

    if (total_valid_samples < ACCEL_CALIBRATION_WINDOW_SAMPLES)
    {
        AccelCalibration_Reset();
        return false;
    }

    /* 未严格收敛时，退化为使用“最佳窗口”结果 */
    if (!converged && have_best_window)
    {
        selected_bias[0] = best_bias[0];
        selected_bias[1] = best_bias[1];
        selected_bias[2] = best_bias[2];
    }

    g_accel_calibration.accel_bias_g[0] = selected_bias[0];
    g_accel_calibration.accel_bias_g[1] = selected_bias[1];
    g_accel_calibration.accel_bias_g[2] = selected_bias[2];
    clamp_bias();

    {
        /* 启动 scale 初值：让全局平均模长尽量接近 1g */
        float startup_scale = 1.0f;

        if (is_finitef_local(global_mean_norm) && (global_mean_norm > 0.2f))
        {
            const float norm_limited = clampf_local(global_mean_norm, 0.85f, 1.15f);
            startup_scale = 1.0f / norm_limited;
        }

        apply_uniform_scale(startup_scale);
    }

    g_accel_calibration.accel_norm_mean_g = global_mean_norm;
    g_accel_calibration.accel_norm_std_g = std_from_m2(global_m2_norm, total_valid_samples);

    if (!is_finitef_local(g_accel_calibration.accel_norm_std_g) ||
        (g_accel_calibration.accel_norm_std_g > ACCEL_CALIBRATION_STD_G_FAIL_MAX))
    {
        g_accel_calibration.is_calibrated = false;
        g_accel_calibration.sample_count = (uint16_t)total_valid_samples;
        return false;
    }

    {
        /* 根据标定阶段观测到的平均模长，微调重力常数 */
        const float g_est = global_mean_norm * ACCEL_CALIBRATION_GRAVITY_MSS;
        if (is_finitef_local(g_est) && (g_est > 6.0f) && (g_est < 13.0f))
        {
            g_accel_calibration.gravity_mps2 = g_est;
        }
        else
        {
            g_accel_calibration.gravity_mps2 = ACCEL_CALIBRATION_GRAVITY_MSS;
        }
    }

    {
        /* 最终通过条件：收敛，或“最佳窗口 + 样本足够 + 质量达标” */
        const bool quality_ok = (g_accel_calibration.accel_norm_std_g <= ACCEL_CALIBRATION_START_ACCEPT_STD_G);
        const bool enough_samples = (total_valid_samples >= (ACCEL_CALIBRATION_SAMPLES / 2U));
        const bool calibrated = converged || (have_best_window && enough_samples && quality_ok);

        g_accel_calibration.sample_count = (uint16_t)total_valid_samples;
        g_accel_calibration.is_calibrated = calibrated;
        return calibrated;
    }
}

void AccelCalibration_Update2kHz(void)
{
    float accel_sensor_g[3];
    float gyro_sensor_dps[3];
    float gravity_x_g;
    float gravity_y_g;
    float gravity_z_g;
    float accel_body_real_mps2[3];
    float accel_level_mps2[3];
    float accel_down_mps2;

    /* ===================== 重要函数：实时2kHz更新 ===================== */
    sanitize_scale();

    accel_sensor_g[0] = g_imu_filter.acc_filt_x;
    accel_sensor_g[1] = g_imu_filter.acc_filt_y;
    accel_sensor_g[2] = g_imu_filter.acc_filt_z;

    gyro_sensor_dps[0] = g_imu_filter.gyro_filt_x;
    gyro_sensor_dps[1] = g_imu_filter.gyro_filt_y;
    gyro_sensor_dps[2] = g_imu_filter.gyro_filt_z;

    if (!imu_sample_valid(accel_sensor_g[0], accel_sensor_g[1], accel_sensor_g[2], gyro_sensor_dps[0], gyro_sensor_dps[1], gyro_sensor_dps[2]))
    {
        /* 无效样本处理：平滑衰减，防止积分突变 */
        g_accel_calibration.invalid_sample_count++;

        g_accel_calibration.accel_down_for_ekf_mps2 *= 0.98f;
        g_accel_calibration.accel_down_for_output_mps2 *= 0.99f;
        g_accel_calibration.accel_level_mps2[0] *= 0.98f;
        g_accel_calibration.accel_level_mps2[1] *= 0.98f;
        g_accel_calibration.accel_level_mps2[2] *= 0.98f;

        g_accel_calibration.vel_up_mps += (-g_accel_calibration.accel_down_for_ekf_mps2) * ACCEL_CALIBRATION_DT_S;
        g_accel_calibration.pos_up_m += g_accel_calibration.vel_up_mps * ACCEL_CALIBRATION_DT_S;
        return;
    }

    rotate_imu_to_body(accel_sensor_g, g_accel_calibration.accel_raw_body_g);
    rotate_imu_to_body(gyro_sensor_dps, g_accel_calibration.gyro_raw_body_dps);

    /* 在线微调（仅满足静止门限时有效） */
    get_gravity_body_g(&gravity_x_g, &gravity_y_g, &gravity_z_g);
    update_bias_online(
        g_accel_calibration.accel_raw_body_g,
        g_accel_calibration.gyro_raw_body_dps,
        gravity_x_g,
        gravity_y_g,
        gravity_z_g);
    update_scale_online(
        g_accel_calibration.accel_raw_body_g,
        g_accel_calibration.gyro_raw_body_dps);

    /* 关键修正：raw -> (raw - bias) * scale */
    g_accel_calibration.accel_corrected_body_g[0] =
        (g_accel_calibration.accel_raw_body_g[0] - g_accel_calibration.accel_bias_g[0]) * g_accel_calibration.accel_scale[0];
    g_accel_calibration.accel_corrected_body_g[1] =
        (g_accel_calibration.accel_raw_body_g[1] - g_accel_calibration.accel_bias_g[1]) * g_accel_calibration.accel_scale[1];
    g_accel_calibration.accel_corrected_body_g[2] =
        (g_accel_calibration.accel_raw_body_g[2] - g_accel_calibration.accel_bias_g[2]) * g_accel_calibration.accel_scale[2];
    update_runtime_quality(
        g_accel_calibration.accel_corrected_body_g,
        g_accel_calibration.gyro_raw_body_dps);

    /* 去除重力，得到真实机体线加速度 */
    accel_body_real_mps2[0] =
        (g_accel_calibration.accel_corrected_body_g[0] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_x_g) *
        g_accel_calibration.gravity_mps2;
    accel_body_real_mps2[1] =
        (g_accel_calibration.accel_corrected_body_g[1] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_y_g) *
        g_accel_calibration.gravity_mps2;
    accel_body_real_mps2[2] =
        (g_accel_calibration.accel_corrected_body_g[2] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_z_g) *
        g_accel_calibration.gravity_mps2;

    g_accel_calibration.accel_real_body_mps2[0] = accel_body_real_mps2[0];
    g_accel_calibration.accel_real_body_mps2[1] = accel_body_real_mps2[1];
    g_accel_calibration.accel_real_body_mps2[2] = accel_body_real_mps2[2];

    rotate_body_linear_to_level(accel_body_real_mps2, accel_level_mps2);
    g_accel_calibration.accel_level_mps2[0] = accel_level_mps2[0];
    g_accel_calibration.accel_level_mps2[1] = accel_level_mps2[1];
    g_accel_calibration.accel_level_mps2[2] = accel_level_mps2[2];

    /* 投影到 Down 轴并统一符号（供 EKF） */
    accel_down_mps2 = ACCEL_DOWN_SIGN_FOR_EKF * calc_accel_down_from_body(accel_body_real_mps2);

    /* 两路低通：EKF 更快，输出更稳 */
    g_accel_calibration.accel_down_for_ekf_mps2 =
        ACC_DOWN_LPF_ALPHA_EKF * accel_down_mps2 +
        (1.0f - ACC_DOWN_LPF_ALPHA_EKF) * g_accel_calibration.accel_down_for_ekf_mps2;

    g_accel_calibration.accel_down_for_output_mps2 =
        ACC_DOWN_LPF_ALPHA_OUTPUT * accel_down_mps2 +
        (1.0f - ACC_DOWN_LPF_ALPHA_OUTPUT) * g_accel_calibration.accel_down_for_output_mps2;

    /* 垂向积分（向上为正） */
    g_accel_calibration.vel_up_mps += (-g_accel_calibration.accel_down_for_ekf_mps2) * ACCEL_CALIBRATION_DT_S;
    g_accel_calibration.pos_up_m += g_accel_calibration.vel_up_mps * ACCEL_CALIBRATION_DT_S;
}

bool AccelCalibration_IsCalibrated(void)
{
    return g_accel_calibration.is_calibrated;
}

float AccelCalibration_GetGravityMps2(void)
{
    return g_accel_calibration.gravity_mps2;
}

float AccelCalibration_GetVerticalAccelUpMps2(void)
{
    return -g_accel_calibration.accel_down_for_output_mps2;
}

float AccelCalibration_GetVerticalVelocityUpMps(void)
{
    return g_accel_calibration.vel_up_mps;
}

float AccelCalibration_GetVerticalPositionUpM(void)
{
    return g_accel_calibration.pos_up_m;
}

float AccelCalibration_GetAccelDownMps2(void)
{
    return g_accel_calibration.accel_down_for_ekf_mps2;
}

float AccelCalibration_GetAccelDownForEkfMps2(void)
{
    return g_accel_calibration.accel_down_for_ekf_mps2;
}

float AccelCalibration_GetAccelDownForOutputMps2(void)
{
    return g_accel_calibration.accel_down_for_output_mps2;
}

void AccelCalibration_GetBodyAccelMps2(float *ax, float *ay, float *az)
{
    if (ax != NULL)
    {
        *ax = g_accel_calibration.accel_real_body_mps2[0];
    }
    if (ay != NULL)
    {
        *ay = g_accel_calibration.accel_real_body_mps2[1];
    }
    if (az != NULL)
    {
        *az = g_accel_calibration.accel_real_body_mps2[2];
    }
}

void AccelCalibration_GetBodyGyroDps(float *gx, float *gy, float *gz)
{
    if (gx != NULL)
    {
        *gx = g_accel_calibration.gyro_raw_body_dps[0];
    }
    if (gy != NULL)
    {
        *gy = g_accel_calibration.gyro_raw_body_dps[1];
    }
    if (gz != NULL)
    {
        *gz = g_accel_calibration.gyro_raw_body_dps[2];
    }
}

void AccelCalibration_GetLevelAccelMps2(float *ax_level, float *ay_level, float *az_level)
{
    if (ax_level != NULL)
    {
        *ax_level = g_accel_calibration.accel_level_mps2[0];
    }
    if (ay_level != NULL)
    {
        *ay_level = g_accel_calibration.accel_level_mps2[1];
    }
    if (az_level != NULL)
    {
        *az_level = g_accel_calibration.accel_level_mps2[2];
    }
}

void AccelCalibration_GetHorizontalAccelMps2(float *ax_h, float *ay_h)
{
    if (ax_h != NULL)
    {
        *ax_h = g_accel_calibration.accel_level_mps2[0];
    }
    if (ay_h != NULL)
    {
        *ay_h = g_accel_calibration.accel_level_mps2[1];
    }
}

