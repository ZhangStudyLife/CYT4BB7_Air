/********************************************************************
 * 文件名  : accel_calibration.c
 * 说明    : ICM42688 加速度标定与垂向加速度预处理
 * 核心流程: rotate -> bias/scale -> 去重力 -> 垂向投影 -> 低通 -> 积分
 * 关键入口:
 *   1) AccelCalibration_Start()      启动静止标定
 *   2) AccelCalibration_Update_2000HZ() 实时更新（2kHz）
 ********************************************************************/

#include "Accel_Calibration.h"
#include "../Attitude/IMU_TOP.h"
#include "zf_common_headfile.h"

#include <float.h>
#include <ctype.h>
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

/* 在线微调：默认关闭，专注“一次性校准”参数稳定性 */
#define ACCEL_CALIBRATION_ENABLE_ONLINE_TRIM      (0U)
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

/* 静止重锁定：抑制长时间静止时姿态/重力解耦误差导致的慢漂 */
#define ACCEL_CALIBRATION_STATIC_RELOCK_ENABLE        (1U)
#define ACCEL_CALIBRATION_STATIC_RELOCK_ALPHA         (0.0010f)
#define ACCEL_CALIBRATION_STATIC_RELOCK_GYRO_MAX_DPS  (0.35f)
#define ACCEL_CALIBRATION_STATIC_RELOCK_ACC_ERR_MAX_G (0.025f)
#define ACCEL_CALIBRATION_STATIC_RELOCK_TRIM_MAX_G    (0.60f)

#define IMU_CALIB_GYRO_TARGET_VALID_SAMPLES      (120000U)
#define IMU_CALIB_GYRO_TIMEOUT_SAMPLES           (600000U)
#define IMU_CALIB_GYRO_STATIC_MAX_DPS            (1.5f)
#define IMU_CALIB_GYRO_STATIC_ACC_ERR_G          (0.06f)
#define IMU_CALIB_GYRO_STD_MAX_DPS               (0.20f)
#define IMU_CALIB_GYRO_BIAS_MAX_DPS              (3.0f)
#define IMU_CALIB_GYRO_PRE_STABLE_SAMPLES        (3000U)

#define IMU_CALIB_ACC6_FACE_TARGET_SAMPLES       (5000U)
#define IMU_CALIB_ACC6_FACE_STABLE_SAMPLES       (1500U)
#define IMU_CALIB_ACC6_FACE_HOLD_DELAY_SAMPLES   (1000U)
#define IMU_CALIB_ACC6_TIMEOUT_SAMPLES           (960000U)
#define IMU_CALIB_ACC6_STATIC_MAX_DPS            (3.0f)
#define IMU_CALIB_ACC6_DOM_MIN_G                 (0.90f)
#define IMU_CALIB_ACC6_OTHER_MAX_G               (0.25f)
#define IMU_CALIB_ACC6_VALID_MIN_G               (0.75f)
#define IMU_CALIB_ACC6_VALID_MAX_G               (1.25f)
#define IMU_CALIB_ACC6_DOM_STD_MAX_G             (0.03f)
#define IMU_CALIB_ACC6_AXIS_STD_MAX_G            (0.025f)
#define IMU_CALIB_ACC6_POST_NORM_ERR_MAX_G       (0.030f)
#define IMU_CALIB_ACC6_POST_DOM_ERR_MAX_G        (0.060f)
#define IMU_CALIB_ACC6_POST_OFF_AXIS_MAX_G       (0.090f)
#define IMU_CALIB_ACC6_PRE_STABLE_SAMPLES        (2500U)

#define IMU_CALIB_CMD_LINE_MAX                   (64U)
#define IMU_CALIB_CMD_READ_MAX                   (64U)

typedef enum
{
    IMU_CALIB_MODE_IDLE = 0,
    IMU_CALIB_MODE_GYRO = 1,
    IMU_CALIB_MODE_ACC6 = 2,
    IMU_CALIB_MODE_ALL = 3
} IMUCalibMode_e;

typedef enum
{
    IMU_CALIB_ALL_STAGE_NONE = 0,
    IMU_CALIB_ALL_STAGE_GYRO = 1,
    IMU_CALIB_ALL_STAGE_ACC6 = 2
} IMUCalibAllStage_e;

typedef enum
{
    IMU_CALIB_FACE_X_POS = 0,
    IMU_CALIB_FACE_X_NEG = 1,
    IMU_CALIB_FACE_Y_POS = 2,
    IMU_CALIB_FACE_Y_NEG = 3,
    IMU_CALIB_FACE_Z_POS = 4,
    IMU_CALIB_FACE_Z_NEG = 5,
    IMU_CALIB_FACE_NUM = 6
} IMUCalibFace_e;

typedef struct
{
    uint8_t busy;
    uint8_t mode;
    uint8_t all_stage;

    float gyro_sum_dps[3];
    float gyro_mean_dps[3];
    float gyro_m2_dps[3];
    uint32_t gyro_valid_samples;
    uint32_t gyro_total_samples;
    uint32_t gyro_static_stable_samples;
    uint8_t gyro_stable_progress_bucket;
    uint8_t gyro_collect_progress_bucket;
    float gyro_prev_bias_dps[3];
    uint8_t gyro_prev_enabled;

    uint8_t acc6_done_mask;
    int8_t acc6_candidate_face;
    uint16_t acc6_candidate_stable_samples;
    uint16_t acc6_face_hold_delay_samples;
    uint8_t acc6_face_hold_progress_bucket;
    uint32_t acc6_total_samples;
    uint32_t acc6_static_stable_samples;
    int8_t acc6_collect_face;
    uint8_t acc6_collect_progress_bucket;
    uint32_t acc6_face_samples[IMU_CALIB_FACE_NUM];
    float acc6_face_sum[IMU_CALIB_FACE_NUM][3];
    float acc6_face_mean[IMU_CALIB_FACE_NUM][3];
    float acc6_face_m2[IMU_CALIB_FACE_NUM][3];
    float acc6_face_dom_mean[IMU_CALIB_FACE_NUM];
    float acc6_face_dom_m2[IMU_CALIB_FACE_NUM];

    char cmd_line[IMU_CALIB_CMD_LINE_MAX];
    uint8_t cmd_line_len;
} IMUCalibRuntime_t;

AccelCalibration_t g_accel_calibration = {0};
static IMUCalibRuntime_t s_imu_calib = {0};
#if ACCEL_CALIBRATION_STATIC_RELOCK_ENABLE
static float s_static_relock_trim_g[3] = {0.0f, 0.0f, 0.0f};
static uint8_t s_static_relock_trim_ready = 0U;
#endif

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

#if ACCEL_CALIBRATION_ENABLE_ONLINE_TRIM
static float mean_scale(void)
{
    return (g_accel_calibration.accel_scale[0] +
            g_accel_calibration.accel_scale[1] +
            g_accel_calibration.accel_scale[2]) / 3.0f;
}
#endif

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

static float max3f_local(float a, float b, float c)
{
    float max_ab = (a > b) ? a : b;
    return (max_ab > c) ? max_ab : c;
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

#if ACCEL_CALIBRATION_STATIC_RELOCK_ENABLE
static bool static_relock_sample_valid(const float accel_corrected_body_g[3],
                                       const float gyro_body_dps[3])
{
    float accel_norm_g;
    float gyro_norm_dps;

    if ((accel_corrected_body_g == NULL) || (gyro_body_dps == NULL))
    {
        return false;
    }

    accel_norm_g = vec3_norm(accel_corrected_body_g[0],
                             accel_corrected_body_g[1],
                             accel_corrected_body_g[2]);
    gyro_norm_dps = vec3_norm(gyro_body_dps[0],
                              gyro_body_dps[1],
                              gyro_body_dps[2]);

    if (!is_finitef_local(accel_norm_g) || !is_finitef_local(gyro_norm_dps))
    {
        return false;
    }

    if (gyro_norm_dps > ACCEL_CALIBRATION_STATIC_RELOCK_GYRO_MAX_DPS)
    {
        return false;
    }

    if (fabsf_local(accel_norm_g - 1.0f) > ACCEL_CALIBRATION_STATIC_RELOCK_ACC_ERR_MAX_G)
    {
        return false;
    }

    if (g_mahony_ahrs.is_static == 0U)
    {
        return false;
    }

    return true;
}

static void static_relock_update_trim(const float accel_corrected_body_g[3],
                                      const float gyro_body_dps[3],
                                      float gravity_x_g,
                                      float gravity_y_g,
                                      float gravity_z_g)
{
    float residual_g[3];
    uint8_t i;

    if (!static_relock_sample_valid(accel_corrected_body_g, gyro_body_dps))
    {
        return;
    }

    residual_g[0] = accel_corrected_body_g[0] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_x_g;
    residual_g[1] = accel_corrected_body_g[1] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_y_g;
    residual_g[2] = accel_corrected_body_g[2] - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_z_g;

    if (s_static_relock_trim_ready == 0U)
    {
        s_static_relock_trim_g[0] = clampf_local(residual_g[0],
                                                 -ACCEL_CALIBRATION_STATIC_RELOCK_TRIM_MAX_G,
                                                 ACCEL_CALIBRATION_STATIC_RELOCK_TRIM_MAX_G);
        s_static_relock_trim_g[1] = clampf_local(residual_g[1],
                                                 -ACCEL_CALIBRATION_STATIC_RELOCK_TRIM_MAX_G,
                                                 ACCEL_CALIBRATION_STATIC_RELOCK_TRIM_MAX_G);
        s_static_relock_trim_g[2] = clampf_local(residual_g[2],
                                                 -ACCEL_CALIBRATION_STATIC_RELOCK_TRIM_MAX_G,
                                                 ACCEL_CALIBRATION_STATIC_RELOCK_TRIM_MAX_G);
        s_static_relock_trim_ready = 1U;
        return;
    }

    for (i = 0U; i < 3U; i++)
    {
        s_static_relock_trim_g[i] += ACCEL_CALIBRATION_STATIC_RELOCK_ALPHA *
                                     (residual_g[i] - s_static_relock_trim_g[i]);
        s_static_relock_trim_g[i] = clampf_local(s_static_relock_trim_g[i],
                                                 -ACCEL_CALIBRATION_STATIC_RELOCK_TRIM_MAX_G,
                                                 ACCEL_CALIBRATION_STATIC_RELOCK_TRIM_MAX_G);
    }
}
#endif

#if ACCEL_CALIBRATION_ENABLE_ONLINE_TRIM
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
#endif

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

static uint8_t imu_calib_count_done_faces(uint8_t done_mask)
{
    uint8_t i;
    uint8_t count = 0U;
    for (i = 0U; i < IMU_CALIB_FACE_NUM; i++)
    {
        if ((done_mask & (uint8_t)(1U << i)) != 0U)
        {
            count++;
        }
    }
    return count;
}

static uint8_t imu_calib_blob_is_valid(const IMUCalibBlob_t *blob)
{
    if (blob == NULL)
    {
        return 0U;
    }
    if (blob->magic != IMU_CALIB_FLASH_MAGIC)
    {
        return 0U;
    }
    if (blob->version != IMU_CALIB_FLASH_VERSION)
    {
        return 0U;
    }
    if (blob->size != (uint16_t)sizeof(IMUCalibBlob_t))
    {
        return 0U;
    }
    return 1U;
}

static void imu_calib_fill_blob(IMUCalibBlob_t *blob)
{
    AccelCalibrationParams_t params;
    uint8_t enabled = 0U;

    if (blob == NULL)
    {
        return;
    }

    memset(blob, 0, sizeof(*blob));
    blob->magic = IMU_CALIB_FLASH_MAGIC;
    blob->version = IMU_CALIB_FLASH_VERSION;
    blob->size = (uint16_t)sizeof(IMUCalibBlob_t);

    ICM42688_GetGyroBiasDps(&blob->gyro_bias_dps[0], &blob->gyro_bias_dps[1], &blob->gyro_bias_dps[2], &enabled);
    if (enabled == 0U)
    {
        blob->gyro_bias_dps[0] = 0.0f;
        blob->gyro_bias_dps[1] = 0.0f;
        blob->gyro_bias_dps[2] = 0.0f;
    }

    AccelCalibration_GetParams(&params);
    memcpy(blob->accel_bias_g, params.accel_bias_g, sizeof(blob->accel_bias_g));
    memcpy(blob->accel_scale, params.accel_scale, sizeof(blob->accel_scale));
    memcpy(blob->imu_to_body, params.imu_to_body, sizeof(blob->imu_to_body));
}

static uint8_t imu_calib_apply_blob(const IMUCalibBlob_t *blob)
{
    AccelCalibrationParams_t params;

    if (imu_calib_blob_is_valid(blob) == 0U)
    {
        return 0U;
    }

    params.accel_bias_g[0] = blob->accel_bias_g[0];
    params.accel_bias_g[1] = blob->accel_bias_g[1];
    params.accel_bias_g[2] = blob->accel_bias_g[2];
    params.accel_scale[0] = blob->accel_scale[0];
    params.accel_scale[1] = blob->accel_scale[1];
    params.accel_scale[2] = blob->accel_scale[2];
    memcpy(params.imu_to_body, blob->imu_to_body, sizeof(params.imu_to_body));
    params.gravity_mps2 = ACCEL_CALIBRATION_GRAVITY_MSS;

    if (!AccelCalibration_LoadParams(&params))
    {
        return 0U;
    }

    ICM42688_SetGyroBiasDps(blob->gyro_bias_dps[0], blob->gyro_bias_dps[1], blob->gyro_bias_dps[2], 1U);
    return 1U;
}

static void imu_calib_print_runtime_params(void)
{
    AccelCalibrationParams_t params;
    float bx = 0.0f;
    float by = 0.0f;
    float bz = 0.0f;
    uint8_t enabled = 0U;

    ICM42688_GetGyroBiasDps(&bx, &by, &bz, &enabled);
    AccelCalibration_GetParams(&params);

    printf("cal,dump,gyro,%u,%f,%f,%f\r\n",
           (unsigned int)enabled, bx, by, bz);
    printf("cal,dump,acc,%f,%f,%f,%f,%f,%f,%f\r\n",
           params.accel_bias_g[0], params.accel_bias_g[1], params.accel_bias_g[2],
           params.accel_scale[0], params.accel_scale[1], params.accel_scale[2],
           params.gravity_mps2);
    printf("cal,dump,r0,%f,%f,%f\r\n",
           params.imu_to_body[0][0], params.imu_to_body[0][1], params.imu_to_body[0][2]);
    printf("cal,dump,r1,%f,%f,%f\r\n",
           params.imu_to_body[1][0], params.imu_to_body[1][1], params.imu_to_body[1][2]);
    printf("cal,dump,r2,%f,%f,%f\r\n",
           params.imu_to_body[2][0], params.imu_to_body[2][1], params.imu_to_body[2][2]);
}

static void imu_calib_print_flash_params(void)
{
    IMUCalibBlob_t blob;
    uint8_t valid;
    const uint32_t words = (uint32_t)((sizeof(IMUCalibBlob_t) + sizeof(uint32_t) - 1U) / sizeof(uint32_t));

    memset(&blob, 0, sizeof(blob));
    flash_read_page(0U, IMU_CALIB_FLASH_PAGE, (uint32_t *)&blob, words);
    valid = imu_calib_blob_is_valid(&blob);

    printf("cal,flash,meta,%u,0x%08lX,%u,%u\r\n",
           (unsigned int)valid,
           (unsigned long)blob.magic,
           (unsigned int)blob.version,
           (unsigned int)blob.size);

    if (valid == 0U)
    {
        return;
    }

    printf("cal,flash,gyro,%f,%f,%f\r\n",
           blob.gyro_bias_dps[0], blob.gyro_bias_dps[1], blob.gyro_bias_dps[2]);
    printf("cal,flash,acc,%f,%f,%f,%f,%f,%f\r\n",
           blob.accel_bias_g[0], blob.accel_bias_g[1], blob.accel_bias_g[2],
           blob.accel_scale[0], blob.accel_scale[1], blob.accel_scale[2]);
    printf("cal,flash,r0,%f,%f,%f\r\n",
           blob.imu_to_body[0][0], blob.imu_to_body[0][1], blob.imu_to_body[0][2]);
    printf("cal,flash,r1,%f,%f,%f\r\n",
           blob.imu_to_body[1][0], blob.imu_to_body[1][1], blob.imu_to_body[1][2]);
    printf("cal,flash,r2,%f,%f,%f\r\n",
           blob.imu_to_body[2][0], blob.imu_to_body[2][1], blob.imu_to_body[2][2]);
}

static void imu_calib_reset_runtime(void)
{
    memset(&s_imu_calib, 0, sizeof(s_imu_calib));
    s_imu_calib.acc6_candidate_face = -1;
    s_imu_calib.acc6_collect_face = -1;
}

static void imu_calib_start_gyro(void)
{
    imu_calib_reset_runtime();
    ICM42688_GetGyroBiasDps(&s_imu_calib.gyro_prev_bias_dps[0],
                            &s_imu_calib.gyro_prev_bias_dps[1],
                            &s_imu_calib.gyro_prev_bias_dps[2],
                            &s_imu_calib.gyro_prev_enabled);
    ICM42688_SetGyroBiasDps(0.0f, 0.0f, 0.0f, 0U);
    s_imu_calib.busy = 1U;
    s_imu_calib.mode = IMU_CALIB_MODE_GYRO;
}

static void imu_calib_prepare_acc6_state(void)
{
    s_imu_calib.acc6_done_mask = 0U;
    s_imu_calib.acc6_candidate_face = -1;
    s_imu_calib.acc6_candidate_stable_samples = 0U;
    s_imu_calib.acc6_face_hold_delay_samples = 0U;
    s_imu_calib.acc6_face_hold_progress_bucket = 0U;
    s_imu_calib.acc6_total_samples = 0U;
    s_imu_calib.acc6_static_stable_samples = 0U;
    s_imu_calib.acc6_collect_face = -1;
    s_imu_calib.acc6_collect_progress_bucket = 0U;
    memset(s_imu_calib.acc6_face_samples, 0, sizeof(s_imu_calib.acc6_face_samples));
    memset(s_imu_calib.acc6_face_sum, 0, sizeof(s_imu_calib.acc6_face_sum));
    memset(s_imu_calib.acc6_face_mean, 0, sizeof(s_imu_calib.acc6_face_mean));
    memset(s_imu_calib.acc6_face_m2, 0, sizeof(s_imu_calib.acc6_face_m2));
    memset(s_imu_calib.acc6_face_dom_mean, 0, sizeof(s_imu_calib.acc6_face_dom_mean));
    memset(s_imu_calib.acc6_face_dom_m2, 0, sizeof(s_imu_calib.acc6_face_dom_m2));
}

static void imu_calib_start_acc6(void)
{
    imu_calib_reset_runtime();
    s_imu_calib.busy = 1U;
    s_imu_calib.mode = IMU_CALIB_MODE_ACC6;
    imu_calib_prepare_acc6_state();
}

static void imu_calib_start_all(void)
{
    imu_calib_reset_runtime();
    ICM42688_GetGyroBiasDps(&s_imu_calib.gyro_prev_bias_dps[0],
                            &s_imu_calib.gyro_prev_bias_dps[1],
                            &s_imu_calib.gyro_prev_bias_dps[2],
                            &s_imu_calib.gyro_prev_enabled);
    ICM42688_SetGyroBiasDps(0.0f, 0.0f, 0.0f, 0U);
    s_imu_calib.busy = 1U;
    s_imu_calib.mode = IMU_CALIB_MODE_ALL;
    s_imu_calib.all_stage = IMU_CALIB_ALL_STAGE_GYRO;
}

static int8_t imu_calib_pick_face(const float accel_body_g[3])
{
    float abs_x;
    float abs_y;
    float abs_z;
    float max_abs;
    uint8_t axis = 0U;
    float axis_value;
    float other_1;
    float other_2;

    if (accel_body_g == NULL)
    {
        return -1;
    }

    abs_x = fabsf_local(accel_body_g[0]);
    abs_y = fabsf_local(accel_body_g[1]);
    abs_z = fabsf_local(accel_body_g[2]);

    max_abs = abs_x;
    axis = 0U;
    if (abs_y > max_abs)
    {
        max_abs = abs_y;
        axis = 1U;
    }
    if (abs_z > max_abs)
    {
        max_abs = abs_z;
        axis = 2U;
    }

    if (max_abs < IMU_CALIB_ACC6_DOM_MIN_G)
    {
        return -1;
    }

    axis_value = accel_body_g[axis];
    if (axis == 0U)
    {
        other_1 = abs_y;
        other_2 = abs_z;
    }
    else if (axis == 1U)
    {
        other_1 = abs_x;
        other_2 = abs_z;
    }
    else
    {
        other_1 = abs_x;
        other_2 = abs_y;
    }

    if ((other_1 > IMU_CALIB_ACC6_OTHER_MAX_G) || (other_2 > IMU_CALIB_ACC6_OTHER_MAX_G))
    {
        return -1;
    }

    return (int8_t)(axis * 2U + ((axis_value >= 0.0f) ? 0U : 1U));
}

static uint8_t imu_calib_all_faces_done(void)
{
    return (s_imu_calib.acc6_done_mask == (uint8_t)((1U << IMU_CALIB_FACE_NUM) - 1U)) ? 1U : 0U;
}

static const char *imu_calib_face_name(uint8_t face_idx)
{
    switch (face_idx)
    {
    case IMU_CALIB_FACE_X_POS:
        return "front";
    case IMU_CALIB_FACE_X_NEG:
        return "back";
    case IMU_CALIB_FACE_Y_POS:
        return "right";
    case IMU_CALIB_FACE_Y_NEG:
        return "left";
    case IMU_CALIB_FACE_Z_POS:
        return "down";
    case IMU_CALIB_FACE_Z_NEG:
        return "up";
    default:
        return "unknown";
    }
}

static int32_t imu_calib_update_gyro_step(void)
{
    float gx = g_imu_filter.gyro_filt_x;
    float gy = g_imu_filter.gyro_filt_y;
    float gz = g_imu_filter.gyro_filt_z;
    float ax = g_imu_filter.acc_filt_x;
    float ay = g_imu_filter.acc_filt_y;
    float az = g_imu_filter.acc_filt_z;
    float gyro_norm_dps;
    float acc_norm_g;
    uint8_t static_ok;

    if (!is_finitef_local(gx) || !is_finitef_local(gy) || !is_finitef_local(gz) ||
        !is_finitef_local(ax) || !is_finitef_local(ay) || !is_finitef_local(az))
    {
        s_imu_calib.gyro_total_samples++;
        if (s_imu_calib.gyro_total_samples >= IMU_CALIB_GYRO_TIMEOUT_SAMPLES)
        {
            return -1;
        }
        return 0;
    }

    gyro_norm_dps = vec3_norm(gx, gy, gz);
    acc_norm_g = vec3_norm(ax, ay, az);
    s_imu_calib.gyro_total_samples++;
    static_ok = ((gyro_norm_dps < IMU_CALIB_GYRO_STATIC_MAX_DPS) &&
                 (fabsf_local(acc_norm_g - 1.0f) < IMU_CALIB_GYRO_STATIC_ACC_ERR_G)) ? 1U : 0U;

    if (s_imu_calib.gyro_valid_samples == 0U)
    {
        if (static_ok != 0U)
        {
            s_imu_calib.gyro_static_stable_samples++;
        }
        else
        {
            s_imu_calib.gyro_static_stable_samples = 0U;
        }

        if (s_imu_calib.gyro_static_stable_samples < IMU_CALIB_GYRO_PRE_STABLE_SAMPLES)
        {
            uint8_t stable_bucket = (uint8_t)((s_imu_calib.gyro_static_stable_samples * 4U) / IMU_CALIB_GYRO_PRE_STABLE_SAMPLES);
            if (stable_bucket > s_imu_calib.gyro_stable_progress_bucket)
            {
                s_imu_calib.gyro_stable_progress_bucket = stable_bucket;
                if ((stable_bucket >= 1U) && (stable_bucket <= 3U))
                {
                    printf("cal,gyro,stabilizing,%u,%lu,%u\r\n",
                           (unsigned int)(stable_bucket * 25U),
                           (unsigned long)s_imu_calib.gyro_static_stable_samples,
                           (unsigned int)IMU_CALIB_GYRO_PRE_STABLE_SAMPLES);
                }
            }
            return 0;
        }

        if (s_imu_calib.gyro_stable_progress_bucket < 4U)
        {
            s_imu_calib.gyro_stable_progress_bucket = 4U;
            printf("cal,gyro,collect,start\r\n");
        }
    }

    if (static_ok != 0U)
    {
        s_imu_calib.gyro_sum_dps[0] += gx;
        s_imu_calib.gyro_sum_dps[1] += gy;
        s_imu_calib.gyro_sum_dps[2] += gz;
        s_imu_calib.gyro_valid_samples++;
        update_running_stats(gx, &s_imu_calib.gyro_mean_dps[0], &s_imu_calib.gyro_m2_dps[0], s_imu_calib.gyro_valid_samples);
        update_running_stats(gy, &s_imu_calib.gyro_mean_dps[1], &s_imu_calib.gyro_m2_dps[1], s_imu_calib.gyro_valid_samples);
        update_running_stats(gz, &s_imu_calib.gyro_mean_dps[2], &s_imu_calib.gyro_m2_dps[2], s_imu_calib.gyro_valid_samples);

        {
            uint8_t collect_bucket = (uint8_t)((s_imu_calib.gyro_valid_samples * 4U) / IMU_CALIB_GYRO_TARGET_VALID_SAMPLES);
            if (collect_bucket > s_imu_calib.gyro_collect_progress_bucket)
            {
                s_imu_calib.gyro_collect_progress_bucket = collect_bucket;
                if ((collect_bucket >= 1U) && (collect_bucket <= 3U))
                {
                    printf("cal,gyro,progress,%u,%lu,%u\r\n",
                           (unsigned int)(collect_bucket * 25U),
                           (unsigned long)s_imu_calib.gyro_valid_samples,
                           (unsigned int)IMU_CALIB_GYRO_TARGET_VALID_SAMPLES);
                }
            }
        }
    }

    if (s_imu_calib.gyro_valid_samples >= IMU_CALIB_GYRO_TARGET_VALID_SAMPLES)
    {
        float bx = s_imu_calib.gyro_sum_dps[0] / (float)s_imu_calib.gyro_valid_samples;
        float by = s_imu_calib.gyro_sum_dps[1] / (float)s_imu_calib.gyro_valid_samples;
        float bz = s_imu_calib.gyro_sum_dps[2] / (float)s_imu_calib.gyro_valid_samples;
        float std_x = std_from_m2(s_imu_calib.gyro_m2_dps[0], s_imu_calib.gyro_valid_samples);
        float std_y = std_from_m2(s_imu_calib.gyro_m2_dps[1], s_imu_calib.gyro_valid_samples);
        float std_z = std_from_m2(s_imu_calib.gyro_m2_dps[2], s_imu_calib.gyro_valid_samples);

        if (!is_finitef_local(std_x) || !is_finitef_local(std_y) || !is_finitef_local(std_z) ||
            !is_finitef_local(bx) || !is_finitef_local(by) || !is_finitef_local(bz))
        {
            return -1;
        }

        if ((std_x > IMU_CALIB_GYRO_STD_MAX_DPS) ||
            (std_y > IMU_CALIB_GYRO_STD_MAX_DPS) ||
            (std_z > IMU_CALIB_GYRO_STD_MAX_DPS) ||
            (fabsf_local(bx) > IMU_CALIB_GYRO_BIAS_MAX_DPS) ||
            (fabsf_local(by) > IMU_CALIB_GYRO_BIAS_MAX_DPS) ||
            (fabsf_local(bz) > IMU_CALIB_GYRO_BIAS_MAX_DPS))
        {
            printf("cal,gyro,quality_fail,%f,%f,%f,%f,%f,%f\r\n",
                   bx, by, bz, std_x, std_y, std_z);
            return -1;
        }

        ICM42688_SetGyroBiasDps(bx, by, bz, 1U);
        printf("cal,gyro,ok,%lu,%f,%f,%f,%f,%f,%f\r\n",
               (unsigned long)s_imu_calib.gyro_valid_samples, bx, by, bz, std_x, std_y, std_z);
        return 1;
    }

    if (s_imu_calib.gyro_total_samples >= IMU_CALIB_GYRO_TIMEOUT_SAMPLES)
    {
        return -1;
    }

    return 0;
}

static int32_t imu_calib_update_acc6_step(void)
{
    float accel_sensor_g[3];
    float gyro_sensor_dps[3];
    float accel_body_g[3];
    float gyro_body_dps[3];
    float accel_norm_g;
    float gyro_norm_dps;
    int8_t face;
    uint8_t face_idx;
    uint32_t n;

    s_imu_calib.acc6_total_samples++;
    if (s_imu_calib.acc6_total_samples >= IMU_CALIB_ACC6_TIMEOUT_SAMPLES)
    {
        return -1;
    }

    accel_sensor_g[0] = g_imu_filter.acc_filt_x;
    accel_sensor_g[1] = g_imu_filter.acc_filt_y;
    accel_sensor_g[2] = g_imu_filter.acc_filt_z;
    gyro_sensor_dps[0] = g_imu_filter.gyro_filt_x;
    gyro_sensor_dps[1] = g_imu_filter.gyro_filt_y;
    gyro_sensor_dps[2] = g_imu_filter.gyro_filt_z;

    if (!imu_sample_valid(accel_sensor_g[0], accel_sensor_g[1], accel_sensor_g[2],
                          gyro_sensor_dps[0], gyro_sensor_dps[1], gyro_sensor_dps[2]))
    {
        s_imu_calib.acc6_static_stable_samples = 0U;
        s_imu_calib.acc6_candidate_stable_samples = 0U;
        s_imu_calib.acc6_face_hold_delay_samples = 0U;
        s_imu_calib.acc6_face_hold_progress_bucket = 0U;
        return 0;
    }

    rotate_imu_to_body(accel_sensor_g, accel_body_g);
    rotate_imu_to_body(gyro_sensor_dps, gyro_body_dps);

    accel_norm_g = vec3_norm(accel_body_g[0], accel_body_g[1], accel_body_g[2]);
    gyro_norm_dps = vec3_norm(gyro_body_dps[0], gyro_body_dps[1], gyro_body_dps[2]);

    if ((gyro_norm_dps > IMU_CALIB_ACC6_STATIC_MAX_DPS) ||
        (accel_norm_g < IMU_CALIB_ACC6_VALID_MIN_G) ||
        (accel_norm_g > IMU_CALIB_ACC6_VALID_MAX_G))
    {
        s_imu_calib.acc6_static_stable_samples = 0U;
        s_imu_calib.acc6_candidate_stable_samples = 0U;
        s_imu_calib.acc6_face_hold_delay_samples = 0U;
        s_imu_calib.acc6_face_hold_progress_bucket = 0U;
        return 0;
    }

    if (s_imu_calib.acc6_static_stable_samples < IMU_CALIB_ACC6_PRE_STABLE_SAMPLES)
    {
        s_imu_calib.acc6_static_stable_samples++;
        if (s_imu_calib.acc6_static_stable_samples == IMU_CALIB_ACC6_PRE_STABLE_SAMPLES)
        {
            printf("cal,acc6,stabilized,start_collect\r\n");
        }
        return 0;
    }

    face = imu_calib_pick_face(accel_body_g);
    if (face < 0)
    {
        s_imu_calib.acc6_candidate_stable_samples = 0U;
        s_imu_calib.acc6_face_hold_delay_samples = 0U;
        s_imu_calib.acc6_face_hold_progress_bucket = 0U;
        return 0;
    }

    face_idx = (uint8_t)face;
    if ((s_imu_calib.acc6_done_mask & (uint8_t)(1U << face_idx)) != 0U)
    {
        s_imu_calib.acc6_candidate_stable_samples = 0U;
        s_imu_calib.acc6_face_hold_delay_samples = 0U;
        s_imu_calib.acc6_face_hold_progress_bucket = 0U;
        return 0;
    }

    if (s_imu_calib.acc6_candidate_face != face)
    {
        s_imu_calib.acc6_candidate_face = face;
        s_imu_calib.acc6_candidate_stable_samples = 1U;
        s_imu_calib.acc6_face_hold_delay_samples = 0U;
        s_imu_calib.acc6_face_hold_progress_bucket = 0U;
        return 0;
    }

    if (s_imu_calib.acc6_candidate_stable_samples < IMU_CALIB_ACC6_FACE_STABLE_SAMPLES)
    {
        s_imu_calib.acc6_candidate_stable_samples++;
        s_imu_calib.acc6_face_hold_delay_samples = 0U;
        s_imu_calib.acc6_face_hold_progress_bucket = 0U;
        return 0;
    }

    if (s_imu_calib.acc6_face_samples[face_idx] == 0U)
    {
        if (s_imu_calib.acc6_face_hold_delay_samples == 0U)
        {
            printf("cal,acc6,face_hold_start,%s,%u\r\n",
                   imu_calib_face_name(face_idx),
                   (unsigned int)IMU_CALIB_ACC6_FACE_HOLD_DELAY_SAMPLES);
        }

        if (s_imu_calib.acc6_face_hold_delay_samples < IMU_CALIB_ACC6_FACE_HOLD_DELAY_SAMPLES)
        {
            s_imu_calib.acc6_face_hold_delay_samples++;
            {
                uint8_t hold_bucket = (uint8_t)((s_imu_calib.acc6_face_hold_delay_samples * 4U) / IMU_CALIB_ACC6_FACE_HOLD_DELAY_SAMPLES);
                if (hold_bucket > s_imu_calib.acc6_face_hold_progress_bucket)
                {
                    s_imu_calib.acc6_face_hold_progress_bucket = hold_bucket;
                    if ((hold_bucket >= 1U) && (hold_bucket <= 3U))
                    {
                        printf("cal,acc6,face_hold_progress,%s,%u,%u,%u\r\n",
                               imu_calib_face_name(face_idx),
                               (unsigned int)(hold_bucket * 25U),
                               (unsigned int)s_imu_calib.acc6_face_hold_delay_samples,
                               (unsigned int)IMU_CALIB_ACC6_FACE_HOLD_DELAY_SAMPLES);
                    }
                }
            }
            return 0;
        }

        if (s_imu_calib.acc6_face_hold_progress_bucket < 4U)
        {
            s_imu_calib.acc6_face_hold_progress_bucket = 4U;
            printf("cal,acc6,face_hold_done,%s\r\n", imu_calib_face_name(face_idx));
        }
    }

    n = s_imu_calib.acc6_face_samples[face_idx] + 1U;
    s_imu_calib.acc6_face_samples[face_idx] = n;
    if (s_imu_calib.acc6_collect_face != face)
    {
        s_imu_calib.acc6_collect_face = face;
        s_imu_calib.acc6_collect_progress_bucket = 0U;
        printf("cal,acc6,face_collect_start,%s\r\n", imu_calib_face_name(face_idx));
    }

    s_imu_calib.acc6_face_sum[face_idx][0] += accel_body_g[0];
    s_imu_calib.acc6_face_sum[face_idx][1] += accel_body_g[1];
    s_imu_calib.acc6_face_sum[face_idx][2] += accel_body_g[2];
    update_running_stats(accel_body_g[0], &s_imu_calib.acc6_face_mean[face_idx][0], &s_imu_calib.acc6_face_m2[face_idx][0], n);
    update_running_stats(accel_body_g[1], &s_imu_calib.acc6_face_mean[face_idx][1], &s_imu_calib.acc6_face_m2[face_idx][1], n);
    update_running_stats(accel_body_g[2], &s_imu_calib.acc6_face_mean[face_idx][2], &s_imu_calib.acc6_face_m2[face_idx][2], n);

    {
        uint8_t dom_axis = (uint8_t)(face_idx / 2U);
        float dom_value = accel_body_g[dom_axis];
        float delta = dom_value - s_imu_calib.acc6_face_dom_mean[face_idx];
        s_imu_calib.acc6_face_dom_mean[face_idx] += delta / (float)n;
        s_imu_calib.acc6_face_dom_m2[face_idx] += delta * (dom_value - s_imu_calib.acc6_face_dom_mean[face_idx]);
    }

    {
        uint8_t face_bucket = (uint8_t)((n * 4U) / IMU_CALIB_ACC6_FACE_TARGET_SAMPLES);
        if (face_bucket > s_imu_calib.acc6_collect_progress_bucket)
        {
            s_imu_calib.acc6_collect_progress_bucket = face_bucket;
            if ((face_bucket >= 1U) && (face_bucket <= 3U))
            {
                printf("cal,acc6,face_progress,%s,%u,%lu,%u\r\n",
                       imu_calib_face_name(face_idx),
                       (unsigned int)(face_bucket * 25U),
                       (unsigned long)n,
                       (unsigned int)IMU_CALIB_ACC6_FACE_TARGET_SAMPLES);
            }
        }
    }

    if (s_imu_calib.acc6_face_samples[face_idx] >= IMU_CALIB_ACC6_FACE_TARGET_SAMPLES)
    {
        float std_dom = std_from_m2(s_imu_calib.acc6_face_dom_m2[face_idx], s_imu_calib.acc6_face_samples[face_idx]);
        float std_x = std_from_m2(s_imu_calib.acc6_face_m2[face_idx][0], s_imu_calib.acc6_face_samples[face_idx]);
        float std_y = std_from_m2(s_imu_calib.acc6_face_m2[face_idx][1], s_imu_calib.acc6_face_samples[face_idx]);
        float std_z = std_from_m2(s_imu_calib.acc6_face_m2[face_idx][2], s_imu_calib.acc6_face_samples[face_idx]);
        float std_all_max = max3f_local(std_x, std_y, std_z);

        if ((std_dom > IMU_CALIB_ACC6_DOM_STD_MAX_G) ||
            (std_all_max > IMU_CALIB_ACC6_AXIS_STD_MAX_G))
        {
            printf("cal,acc6,face_fail,%s,%f,%f,%f,%f\r\n",
                   imu_calib_face_name(face_idx), std_dom, std_x, std_y, std_z);
            return -1;
        }

        s_imu_calib.acc6_done_mask |= (uint8_t)(1U << face_idx);
        s_imu_calib.acc6_candidate_face = -1;
        s_imu_calib.acc6_candidate_stable_samples = 0U;
        s_imu_calib.acc6_face_hold_delay_samples = 0U;
        s_imu_calib.acc6_face_hold_progress_bucket = 0U;
        s_imu_calib.acc6_static_stable_samples = 0U;
        s_imu_calib.acc6_collect_face = -1;
        s_imu_calib.acc6_collect_progress_bucket = 0U;
        printf("cal,acc6,face_done,%s,%lu,%f,%f\r\n",
               imu_calib_face_name(face_idx),
               (unsigned long)s_imu_calib.acc6_face_samples[face_idx],
               std_dom,
               std_all_max);
    }

    if (imu_calib_all_faces_done() != 0U)
    {
        AccelCalibrationParams_t params;
        uint8_t axis;
        float max_norm_err = 0.0f;
        float max_dom_err = 0.0f;
        float max_off_axis = 0.0f;

        AccelCalibration_GetParams(&params);

        for (axis = 0U; axis < 3U; axis++)
        {
            uint8_t face_pos = (uint8_t)(axis * 2U);
            uint8_t face_neg = (uint8_t)(axis * 2U + 1U);
            float mean_pos;
            float mean_neg;
            float delta;

            if ((s_imu_calib.acc6_face_samples[face_pos] < IMU_CALIB_ACC6_FACE_TARGET_SAMPLES) ||
                (s_imu_calib.acc6_face_samples[face_neg] < IMU_CALIB_ACC6_FACE_TARGET_SAMPLES))
            {
                return -1;
            }

            mean_pos = s_imu_calib.acc6_face_sum[face_pos][axis] / (float)s_imu_calib.acc6_face_samples[face_pos];
            mean_neg = s_imu_calib.acc6_face_sum[face_neg][axis] / (float)s_imu_calib.acc6_face_samples[face_neg];
            delta = mean_pos - mean_neg;

            if (!is_finitef_local(mean_pos) || !is_finitef_local(mean_neg) || (fabsf_local(delta) < 0.40f))
            {
                return -1;
            }

            params.accel_bias_g[axis] = 0.5f * (mean_pos + mean_neg);
            params.accel_scale[axis] = 2.0f / fabsf_local(delta);
            params.accel_scale[axis] = clampf_local(params.accel_scale[axis], ACCEL_CALIBRATION_SCALE_MIN, ACCEL_CALIBRATION_SCALE_MAX);
        }

        if (!AccelCalibration_LoadParams(&params))
        {
            return -1;
        }

        for (face_idx = 0U; face_idx < IMU_CALIB_FACE_NUM; face_idx++)
        {
            uint8_t dom_axis = (uint8_t)(face_idx / 2U);
            uint8_t off_axis_1 = (uint8_t)((dom_axis + 1U) % 3U);
            uint8_t off_axis_2 = (uint8_t)((dom_axis + 2U) % 3U);
            float expect_sign = ((face_idx % 2U) == 0U) ? 1.0f : -1.0f;
            float mean_vec[3];
            float corr_vec[3];
            float norm_err;
            float dom_err;
            float off_1;
            float off_2;

            if (s_imu_calib.acc6_face_samples[face_idx] < IMU_CALIB_ACC6_FACE_TARGET_SAMPLES)
            {
                return -1;
            }

            mean_vec[0] = s_imu_calib.acc6_face_sum[face_idx][0] / (float)s_imu_calib.acc6_face_samples[face_idx];
            mean_vec[1] = s_imu_calib.acc6_face_sum[face_idx][1] / (float)s_imu_calib.acc6_face_samples[face_idx];
            mean_vec[2] = s_imu_calib.acc6_face_sum[face_idx][2] / (float)s_imu_calib.acc6_face_samples[face_idx];

            corr_vec[0] = (mean_vec[0] - params.accel_bias_g[0]) * params.accel_scale[0];
            corr_vec[1] = (mean_vec[1] - params.accel_bias_g[1]) * params.accel_scale[1];
            corr_vec[2] = (mean_vec[2] - params.accel_bias_g[2]) * params.accel_scale[2];

            norm_err = fabsf_local(vec3_norm(corr_vec[0], corr_vec[1], corr_vec[2]) - 1.0f);
            dom_err = fabsf_local(corr_vec[dom_axis] - expect_sign);
            off_1 = fabsf_local(corr_vec[off_axis_1]);
            off_2 = fabsf_local(corr_vec[off_axis_2]);

            if (norm_err > max_norm_err)
            {
                max_norm_err = norm_err;
            }
            if (dom_err > max_dom_err)
            {
                max_dom_err = dom_err;
            }
            if (off_1 > max_off_axis)
            {
                max_off_axis = off_1;
            }
            if (off_2 > max_off_axis)
            {
                max_off_axis = off_2;
            }

            if ((norm_err > IMU_CALIB_ACC6_POST_NORM_ERR_MAX_G) ||
                (dom_err > IMU_CALIB_ACC6_POST_DOM_ERR_MAX_G) ||
                (off_1 > IMU_CALIB_ACC6_POST_OFF_AXIS_MAX_G) ||
                (off_2 > IMU_CALIB_ACC6_POST_OFF_AXIS_MAX_G))
            {
                printf("cal,acc6,post_fail,%s,%f,%f,%f,%f\r\n",
                       imu_calib_face_name(face_idx), norm_err, dom_err, off_1, off_2);
                return -1;
            }
        }

        printf("cal,acc6,ok,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
               params.accel_bias_g[0], params.accel_bias_g[1], params.accel_bias_g[2],
               params.accel_scale[0], params.accel_scale[1], params.accel_scale[2],
               max_norm_err, max_dom_err, max_off_axis);
        return 1;
    }

    return 0;
}

static uint32_t imu_calib_progress_percent(void)
{
    uint32_t progress = 0U;

    if (s_imu_calib.busy == 0U)
    {
        return 0U;
    }

    if (s_imu_calib.mode == IMU_CALIB_MODE_GYRO)
    {
        progress = (uint32_t)(100.0f * (float)s_imu_calib.gyro_valid_samples / (float)IMU_CALIB_GYRO_TARGET_VALID_SAMPLES);
    }
    else if (s_imu_calib.mode == IMU_CALIB_MODE_ACC6)
    {
        uint8_t done = imu_calib_count_done_faces(s_imu_calib.acc6_done_mask);
        progress = (uint32_t)((100U * done) / IMU_CALIB_FACE_NUM);
    }
    else if (s_imu_calib.mode == IMU_CALIB_MODE_ALL)
    {
        if (s_imu_calib.all_stage == IMU_CALIB_ALL_STAGE_GYRO)
        {
            progress = (uint32_t)(50.0f * (float)s_imu_calib.gyro_valid_samples / (float)IMU_CALIB_GYRO_TARGET_VALID_SAMPLES);
        }
        else if (s_imu_calib.all_stage == IMU_CALIB_ALL_STAGE_ACC6)
        {
            uint8_t done = imu_calib_count_done_faces(s_imu_calib.acc6_done_mask);
            progress = 50U + (uint32_t)((50U * done) / IMU_CALIB_FACE_NUM);
        }
    }

    if (progress > 100U)
    {
        progress = 100U;
    }
    return progress;
}

static void imu_calib_print_status(void)
{
    uint8_t face_done = imu_calib_count_done_faces(s_imu_calib.acc6_done_mask);
    uint32_t progress = imu_calib_progress_percent();
    printf("cal,status,%u,%u,%u,%u,%u\r\n",
           (unsigned int)s_imu_calib.busy,
           (unsigned int)s_imu_calib.mode,
           (unsigned int)s_imu_calib.all_stage,
           (unsigned int)progress,
           (unsigned int)face_done);
}

static void imu_calib_command_to_lower(char *line)
{
    uint8_t i;
    uint8_t len;

    if (line == NULL)
    {
        return;
    }

    len = (uint8_t)strlen(line);
    for (i = 0U; i < len; i++)
    {
        line[i] = (char)tolower((int)line[i]);
    }
}

static void imu_calib_process_command(char *line)
{
    uint32_t irq_state;

    if (line == NULL)
    {
        return;
    }

    imu_calib_command_to_lower(line);

    if (strcmp(line, "cal status") == 0)
    {
        imu_calib_print_status();
        return;
    }

    if (strcmp(line, "cal load") == 0)
    {
        if (s_imu_calib.busy != 0U)
        {
            printf("cal,busy\r\n");
            return;
        }
        printf("cal,load,%u\r\n", (unsigned int)IMUCalib_LoadFromFlashAndApply());
        return;
    }

    if (strcmp(line, "cal save") == 0)
    {
        if (s_imu_calib.busy != 0U)
        {
            printf("cal,busy\r\n");
            return;
        }
        printf("cal,save,%u\r\n", (unsigned int)IMUCalib_SaveCurrentToFlash());
        return;
    }

    if (strcmp(line, "cal clear") == 0)
    {
        if (s_imu_calib.busy != 0U)
        {
            printf("cal,busy\r\n");
            return;
        }
        printf("cal,clear,%u\r\n", (unsigned int)IMUCalib_ClearFlash());
        return;
    }

    if (strcmp(line, "cal dump") == 0)
    {
        if (s_imu_calib.busy != 0U)
        {
            printf("cal,busy\r\n");
            return;
        }
        imu_calib_print_runtime_params();
        imu_calib_print_flash_params();
        return;
    }

    if (strcmp(line, "cal gyro start") == 0)
    {
        if (s_imu_calib.busy != 0U)
        {
            printf("cal,busy\r\n");
            return;
        }
        irq_state = interrupt_global_disable();
        imu_calib_start_gyro();
        interrupt_global_enable(irq_state);
        printf("cal,gyro,start\r\n");
        return;
    }

    if (strcmp(line, "cal acc6 start") == 0)
    {
        if (s_imu_calib.busy != 0U)
        {
            printf("cal,busy\r\n");
            return;
        }
        irq_state = interrupt_global_disable();
        imu_calib_start_acc6();
        interrupt_global_enable(irq_state);
        printf("cal,acc6,start\r\n");
        return;
    }

    if (strcmp(line, "cal all start") == 0)
    {
        if (s_imu_calib.busy != 0U)
        {
            printf("cal,busy\r\n");
            return;
        }
        irq_state = interrupt_global_disable();
        imu_calib_start_all();
        interrupt_global_enable(irq_state);
        printf("cal,all,start\r\n");
        return;
    }

    printf("cal,unknown\r\n");
}

static void imu_calib_print_boot_reminder(void)
{
    printf("cal,remind,cmd,cal status\r\n");
    printf("cal,remind,cmd,cal dump\r\n");
    printf("cal,remind,cmd,cal all start\r\n");
    printf("cal,remind,cmd,cal load\r\n");
    printf("cal,remind,cmd,cal save\r\n");
    printf("cal,remind,cmd,cal clear\r\n");
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

#if ACCEL_CALIBRATION_STATIC_RELOCK_ENABLE
    s_static_relock_trim_g[0] = 0.0f;
    s_static_relock_trim_g[1] = 0.0f;
    s_static_relock_trim_g[2] = 0.0f;
    s_static_relock_trim_ready = 0U;
#endif
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
            IMU_Update_2000HZ();
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

void AccelCalibration_Update_2000HZ(void)
{
    float accel_sensor_g[3];
    float gyro_sensor_dps[3];
    float gravity_x_g;
    float gravity_y_g;
    float gravity_z_g;
    float accel_body_real_mps2[3];
    float accel_level_mps2[3];
    float accel_down_mps2;
    float trim_x_g = 0.0f;
    float trim_y_g = 0.0f;
    float trim_z_g = 0.0f;

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
        g_accel_calibration.realtime_sample_valid = 0U;

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
#if ACCEL_CALIBRATION_ENABLE_ONLINE_TRIM
    update_bias_online(
        g_accel_calibration.accel_raw_body_g,
        g_accel_calibration.gyro_raw_body_dps,
        gravity_x_g,
        gravity_y_g,
        gravity_z_g);
    update_scale_online(
        g_accel_calibration.accel_raw_body_g,
        g_accel_calibration.gyro_raw_body_dps);
#endif

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

#if ACCEL_CALIBRATION_STATIC_RELOCK_ENABLE
    static_relock_update_trim(
        g_accel_calibration.accel_corrected_body_g,
        g_accel_calibration.gyro_raw_body_dps,
        gravity_x_g,
        gravity_y_g,
        gravity_z_g);

    trim_x_g = s_static_relock_trim_g[0];
    trim_y_g = s_static_relock_trim_g[1];
    trim_z_g = s_static_relock_trim_g[2];
#endif

    /* 去除重力，得到真实机体线加速度 */
    accel_body_real_mps2[0] =
        (g_accel_calibration.accel_corrected_body_g[0] - trim_x_g - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_x_g) *
        g_accel_calibration.gravity_mps2;
    accel_body_real_mps2[1] =
        (g_accel_calibration.accel_corrected_body_g[1] - trim_y_g - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_y_g) *
        g_accel_calibration.gravity_mps2;
    accel_body_real_mps2[2] =
        (g_accel_calibration.accel_corrected_body_g[2] - trim_z_g - ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN * gravity_z_g) *
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
    g_accel_calibration.realtime_sample_valid = 1U;
}

bool AccelCalibration_IsCalibrated(void)
{
    return g_accel_calibration.is_calibrated;
}

uint8_t AccelCalibration_IsRealtimeDataValid(void)
{
    return g_accel_calibration.realtime_sample_valid;
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

void AccelCalibration_GetCorrectedSpecificForceG(float *ax_g, float *ay_g, float *az_g)
{
    if (ax_g != NULL)
    {
        *ax_g = g_accel_calibration.accel_corrected_body_g[0];
    }
    if (ay_g != NULL)
    {
        *ay_g = g_accel_calibration.accel_corrected_body_g[1];
    }
    if (az_g != NULL)
    {
        *az_g = g_accel_calibration.accel_corrected_body_g[2];
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

bool AccelCalibration_LoadParams(const AccelCalibrationParams_t *params)
{
    if (params == NULL)
    {
        return false;
    }

    if (!is_finitef_local(params->accel_bias_g[0]) ||
        !is_finitef_local(params->accel_bias_g[1]) ||
        !is_finitef_local(params->accel_bias_g[2]) ||
        !is_finitef_local(params->accel_scale[0]) ||
        !is_finitef_local(params->accel_scale[1]) ||
        !is_finitef_local(params->accel_scale[2]))
    {
        return false;
    }

    g_accel_calibration.accel_bias_g[0] = params->accel_bias_g[0];
    g_accel_calibration.accel_bias_g[1] = params->accel_bias_g[1];
    g_accel_calibration.accel_bias_g[2] = params->accel_bias_g[2];
    g_accel_calibration.accel_scale[0] = params->accel_scale[0];
    g_accel_calibration.accel_scale[1] = params->accel_scale[1];
    g_accel_calibration.accel_scale[2] = params->accel_scale[2];
    memcpy(g_accel_calibration.imu_to_body, params->imu_to_body, sizeof(g_accel_calibration.imu_to_body));

    if (is_finitef_local(params->gravity_mps2) && (params->gravity_mps2 > 6.0f) && (params->gravity_mps2 < 13.0f))
    {
        g_accel_calibration.gravity_mps2 = params->gravity_mps2;
    }
    else
    {
        g_accel_calibration.gravity_mps2 = ACCEL_CALIBRATION_GRAVITY_MSS;
    }

    g_accel_calibration.imu_to_body_identity = matrix_is_identity(g_accel_calibration.imu_to_body);
    clamp_bias();
    sanitize_scale();

#if ACCEL_CALIBRATION_STATIC_RELOCK_ENABLE
    s_static_relock_trim_g[0] = 0.0f;
    s_static_relock_trim_g[1] = 0.0f;
    s_static_relock_trim_g[2] = 0.0f;
    s_static_relock_trim_ready = 0U;
#endif

    g_accel_calibration.is_calibrated = true;
    return true;
}

void AccelCalibration_GetParams(AccelCalibrationParams_t *params)
{
    if (params == NULL)
    {
        return;
    }

    params->accel_bias_g[0] = g_accel_calibration.accel_bias_g[0];
    params->accel_bias_g[1] = g_accel_calibration.accel_bias_g[1];
    params->accel_bias_g[2] = g_accel_calibration.accel_bias_g[2];
    params->accel_scale[0] = g_accel_calibration.accel_scale[0];
    params->accel_scale[1] = g_accel_calibration.accel_scale[1];
    params->accel_scale[2] = g_accel_calibration.accel_scale[2];
    memcpy(params->imu_to_body, g_accel_calibration.imu_to_body, sizeof(params->imu_to_body));
    params->gravity_mps2 = g_accel_calibration.gravity_mps2;
}

void IMUCalib_Init(void)
{
    flash_init();
    imu_calib_reset_runtime();
    if (IMUCalib_LoadFromFlashAndApply() != 0U)
    {
        printf("cal,loaded\r\n");
    }
    else
    {
        printf("cal,default\r\n");
    }
    imu_calib_print_boot_reminder();
}

uint8_t IMUCalib_LoadFromFlashAndApply(void)
{
    IMUCalibBlob_t blob;
    const uint32_t words = (uint32_t)((sizeof(IMUCalibBlob_t) + sizeof(uint32_t) - 1U) / sizeof(uint32_t));

    memset(&blob, 0, sizeof(blob));
    flash_read_page(0U, IMU_CALIB_FLASH_PAGE, (uint32_t *)&blob, words);
    return imu_calib_apply_blob(&blob);
}

uint8_t IMUCalib_SaveCurrentToFlash(void)
{
    IMUCalibBlob_t blob;
    const uint32_t words = (uint32_t)((sizeof(IMUCalibBlob_t) + sizeof(uint32_t) - 1U) / sizeof(uint32_t));

    imu_calib_fill_blob(&blob);
    flash_write_page(0U, IMU_CALIB_FLASH_PAGE, (const uint32_t *)&blob, words);
    return 1U;
}

uint8_t IMUCalib_ClearFlash(void)
{
    flash_erase_page(0U, IMU_CALIB_FLASH_PAGE);
    return 1U;
}

uint8_t IMUCalib_IsBusy(void)
{
    return s_imu_calib.busy;
}

void IMUCalib_Update_2000HZ(void)
{
    int32_t ret;

    if (s_imu_calib.busy == 0U)
    {
        return;
    }

    if (s_imu_calib.mode == IMU_CALIB_MODE_GYRO)
    {
        ret = imu_calib_update_gyro_step();
        if (ret > 0)
        {
            s_imu_calib.busy = 0U;
            s_imu_calib.mode = IMU_CALIB_MODE_IDLE;
        }
        else if (ret < 0)
        {
            printf("cal,gyro,fail,%lu,%lu\r\n",
                   (unsigned long)s_imu_calib.gyro_valid_samples,
                   (unsigned long)s_imu_calib.gyro_total_samples);
            ICM42688_SetGyroBiasDps(s_imu_calib.gyro_prev_bias_dps[0],
                                    s_imu_calib.gyro_prev_bias_dps[1],
                                    s_imu_calib.gyro_prev_bias_dps[2],
                                    s_imu_calib.gyro_prev_enabled);
            s_imu_calib.busy = 0U;
            s_imu_calib.mode = IMU_CALIB_MODE_IDLE;
        }
        return;
    }

    if (s_imu_calib.mode == IMU_CALIB_MODE_ACC6)
    {
        ret = imu_calib_update_acc6_step();
        if (ret > 0)
        {
            s_imu_calib.busy = 0U;
            s_imu_calib.mode = IMU_CALIB_MODE_IDLE;
        }
        else if (ret < 0)
        {
            printf("cal,acc6,fail,%lu,%u\r\n",
                   (unsigned long)s_imu_calib.acc6_total_samples,
                   (unsigned int)imu_calib_count_done_faces(s_imu_calib.acc6_done_mask));
            s_imu_calib.busy = 0U;
            s_imu_calib.mode = IMU_CALIB_MODE_IDLE;
        }
        return;
    }

    if (s_imu_calib.mode == IMU_CALIB_MODE_ALL)
    {
        if (s_imu_calib.all_stage == IMU_CALIB_ALL_STAGE_GYRO)
        {
            ret = imu_calib_update_gyro_step();
            if (ret > 0)
            {
                s_imu_calib.all_stage = IMU_CALIB_ALL_STAGE_ACC6;
                imu_calib_prepare_acc6_state();
                printf("cal,all,stage,acc6\r\n");
            }
            else if (ret < 0)
            {
                printf("cal,all,fail,gyro\r\n");
                ICM42688_SetGyroBiasDps(s_imu_calib.gyro_prev_bias_dps[0],
                                        s_imu_calib.gyro_prev_bias_dps[1],
                                        s_imu_calib.gyro_prev_bias_dps[2],
                                        s_imu_calib.gyro_prev_enabled);
                s_imu_calib.busy = 0U;
                s_imu_calib.mode = IMU_CALIB_MODE_IDLE;
                s_imu_calib.all_stage = IMU_CALIB_ALL_STAGE_NONE;
            }
            return;
        }

        if (s_imu_calib.all_stage == IMU_CALIB_ALL_STAGE_ACC6)
        {
            ret = imu_calib_update_acc6_step();
            if (ret > 0)
            {
                uint8_t save_ok = IMUCalib_SaveCurrentToFlash();
                printf("cal,all,ok,%u\r\n", (unsigned int)save_ok);
                s_imu_calib.busy = 0U;
                s_imu_calib.mode = IMU_CALIB_MODE_IDLE;
                s_imu_calib.all_stage = IMU_CALIB_ALL_STAGE_NONE;
            }
            else if (ret < 0)
            {
                printf("cal,all,fail,acc6\r\n");
                s_imu_calib.busy = 0U;
                s_imu_calib.mode = IMU_CALIB_MODE_IDLE;
                s_imu_calib.all_stage = IMU_CALIB_ALL_STAGE_NONE;
            }
            return;
        }
    }
}

void IMUCalib_CommandPoll(void)
{
    uint8_t rx_buf[IMU_CALIB_CMD_READ_MAX];
    uint32_t len;
    uint32_t i;

    len = debug_read_ring_buffer(rx_buf, sizeof(rx_buf));
    if (len == 0U)
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        char c = (char)rx_buf[i];
        if ((c == '\r') || (c == '\n'))
        {
            if (s_imu_calib.cmd_line_len > 0U)
            {
                s_imu_calib.cmd_line[s_imu_calib.cmd_line_len] = '\0';
                imu_calib_process_command(s_imu_calib.cmd_line);
                s_imu_calib.cmd_line_len = 0U;
            }
            continue;
        }

        if (((uint8_t)c < 32U) || ((uint8_t)c > 126U))
        {
            continue;
        }

        if (s_imu_calib.cmd_line_len < (IMU_CALIB_CMD_LINE_MAX - 1U))
        {
            s_imu_calib.cmd_line[s_imu_calib.cmd_line_len++] = c;
        }
        else
        {
            s_imu_calib.cmd_line_len = 0U;
        }
    }
}


