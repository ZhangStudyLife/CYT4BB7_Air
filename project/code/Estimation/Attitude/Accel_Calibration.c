/********************************************************************
 * �ļ���  : accel_calibration.c
 * ˵��    : ICM42688 ���ٶȱ궨�봹ֱ���ٶ�Ԥ����
 * ������ˮ��: rotate -> bias/scale -> ȥ���� -> ����ͶӰ -> �˲� -> ����
 * �ؼ��ӿ�:
 *   1) AccelCalibration_Start()         ������ֹ�궨
 *   2) AccelCalibration_Update_1000HZ() ʵʱ���£�1kHz��
 ********************************************************************/

#include "Accel_Calibration.h"
#include "IMU_TOP.h"
#include "zf_common_headfile.h"

#include <float.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

#define DEG_TO_RAD                               (0.017453292519943295f)

#define ACC_DOWN_LPF_ALPHA_EKF                   (0.3f)
#define ACC_DOWN_LPF_ALPHA_OUTPUT                (0.1f)

#define IMU_ACCEL_G_MAX_ABS                      (20.0f)
#define IMU_GYRO_DPS_MAX_ABS                     (6000.0f)
#define CALIB_MAX_TRY_SAMPLES                    (3000U) /* ���ٶȾ�ֹ�궨�������ܴ��� */
#define ACCEL_DOWN_SIGN_FOR_EKF                  (+1.0f)

/* AP ��񣺷ִ��ڲɼ����ж� */
#define ACCEL_CALIBRATION_WINDOW_SAMPLES         (200U)  /* ��ֹ����������������1kHzԼ0.2�� */
#define ACCEL_CALIBRATION_MAX_WINDOWS            (24U)
#define ACCEL_CALIBRATION_CONVERGE_WINDOWS       (3U)
#define ACCEL_CALIBRATION_STATIC_ACCEL_MIN_G     (0.80f)
#define ACCEL_CALIBRATION_STATIC_ACCEL_MAX_G     (1.20f)
#define ACCEL_CALIBRATION_STATIC_GYRO_MAX_DPS    (2.5f)
#define ACCEL_CALIBRATION_STATIC_GYRO_STD_MAX_DPS (0.80f)
#define ACCEL_CALIBRATION_CONVERGE_BIAS_DELTA_G  (0.004f)
#define ACCEL_CALIBRATION_CONVERGE_ACC_STD_G     (0.025f)

/* ����΢��Ĭ�Ϲرգ�רעһ����У׼��ȷ����Ч */
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

/* ��ֹ��������������ֹʱ�Զ�΢��ƫ������Ӧ��Ư */
#define ACCEL_CALIBRATION_STATIC_RELOCK_ENABLE        (1U)
#define ACCEL_CALIBRATION_STATIC_RELOCK_ALPHA         (0.0010f)
#define ACCEL_CALIBRATION_STATIC_RELOCK_GYRO_MAX_DPS  (0.35f)
#define ACCEL_CALIBRATION_STATIC_RELOCK_ACC_ERR_MAX_G (0.025f)
#define ACCEL_CALIBRATION_STATIC_RELOCK_TRIM_MAX_G    (0.60f)

#define IMU_CALIB_GYRO_TARGET_VALID_SAMPLES      (60000U) /* �����Ǿ�ֹ�궨Ŀ����Ч��������Լ60�� */
#define IMU_CALIB_GYRO_TIMEOUT_SAMPLES           (300000U) /* �����Ǿ�ֹ�궨��ʱ������ */
#define IMU_CALIB_GYRO_STATIC_MAX_DPS            (1.5f)
#define IMU_CALIB_GYRO_STATIC_ACC_ERR_G          (0.06f)
#define IMU_CALIB_GYRO_STD_MAX_DPS               (0.20f)
#define IMU_CALIB_GYRO_BIAS_MAX_DPS              (3.0f)
#define IMU_CALIB_GYRO_PRE_STABLE_SAMPLES        (1500U) /* �����Ǳ궨Ԥ���ȶ������� */

#define IMU_CALIB_ACC6_FACE_TARGET_SAMPLES       (2500U) /* 6��궨����Ŀ������� */
#define IMU_CALIB_ACC6_FACE_STABLE_SAMPLES       (750U)  /* 6��궨����ȷ���ȶ������� */
#define IMU_CALIB_ACC6_FACE_HOLD_DELAY_SAMPLES   (500U)  /* 6��궨���汣����ʱ������ */
#define IMU_CALIB_ACC6_TIMEOUT_SAMPLES           (480000U) /* 6��궨��ʱ�������� */
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
#define IMU_CALIB_ACC6_PRE_STABLE_SAMPLES        (1250U) /* 6��궨Ԥ���ȶ������� */

#define IMU_CALIB_CMD_LINE_MAX                   (64U)
#define IMU_CALIB_CMD_READ_MAX                   (64U)

typedef enum
{
    IMU_CALIB_MODE_IDLE = 0,
    IMU_CALIB_MODE_GYRO = 1,
    IMU_CALIB_MODE_ACC6 = 2,
    IMU_CALIB_MODE_ALL = 3,
    IMU_CALIB_MODE_ELLIP = 4,
    IMU_CALIB_MODE_ELLIP_MANUAL = 5
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
/* IMU 校准文本输出回调，优先用于 WiFi 文本提示 */
static IMUCalibTextSink_t s_imu_calib_text_sink = NULL;

/* ========================= Ellipsoid calibration runtime ========================= */
#define ELLIP_MAX_ORIENT         (20U)
#define ELLIP_MIN_ORIENT         (12U)
#define ELLIP_ORIENT_SAMPLES     (2000U)
#define ELLIP_PRE_STABLE_SAMPLES (1250U)
#define ELLIP_TIMEOUT_SAMPLES    (600000U)  /* 10 min at 1kHz */
#define ELLIP_STATIC_GYRO_MAX_DPS  (3.0f)
#define ELLIP_STATIC_ACC_MIN_G   (0.75f)
#define ELLIP_STATIC_ACC_MAX_G   (1.25f)
#define ELLIP_ORIENT_STD_MAX_G   (0.025f)
#define ELLIP_ORIENT_ANGLE_MIN_DEG (15.0f)
#define ELLIP_POST_NORM_ERR_MAX_G  (0.03f)
#define ELLIP_POST_DIAG_MIN      (0.85f)
#define ELLIP_POST_DIAG_MAX      (1.15f)
#define ELLIP_POST_BIAS_NORM_MAX_G (0.35f)
#define ELLIP_MOVING_GYRO_DPS    (8.0f)
#define ELLIP_MANUAL_MAX_ORIENT  (128U)     /* 手动椭球模式允许缓存的最大姿态点数 */
#define ELLIP_MANUAL_ORIENT_SAMPLES (5000U) /* 手动椭球模式单个姿态点采样数 */
#define ELLIP_MANUAL_TIMEOUT_SAMPLES (1800000U) /* 手动椭球模式总超时，约 30 分钟 */

typedef struct
{
    float orient_mean[ELLIP_MAX_ORIENT][3];
    float orient_sum[3];
    float orient_m2[3];
    uint32_t orient_samples;
    uint8_t orient_count;
    uint32_t total_samples;
    uint32_t stable_samples;
    uint8_t collecting;     /* 1 = actively collecting an orientation */
    uint8_t wait_move;      /* 1 = waiting for user to move to next orientation */
} EllipCalibRuntime_t;

typedef struct
{
    float orient_mean[ELLIP_MANUAL_MAX_ORIENT][3];
    float pose_std_max[ELLIP_MANUAL_MAX_ORIENT];
    float collect_mean[3];
    float collect_m2[3];
    uint32_t orient_samples;
    uint8_t orient_count;
    uint32_t total_samples;
    uint32_t stable_samples;
    uint8_t substate;
} ManualEllipCalibRuntime_t;

static EllipCalibRuntime_t s_ellip = {0};
static ManualEllipCalibRuntime_t s_ellip_manual = {0};
#if ACCEL_CALIBRATION_STATIC_RELOCK_ENABLE
static float s_static_relock_trim_g[3] = {0.0f, 0.0f, 0.0f};
static uint8_t s_static_relock_trim_ready = 0U;
#endif

/* ========================= �������ߺ��� ========================= */

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

/*
 * 函数功能: 输出 IMU 校准过程文本
 * 输入参数:
 *   format - printf 风格格式串
 *   ...    - 格式化参数
 * 返回值: 无
 */
static void imu_calib_emit_text(const char *format, ...)
{
    char text[192];
    va_list args;
    int len;

    if (NULL == format)
    {
        return;
    }

    va_start(args, format);
    len = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (len <= 0)
    {
        return;
    }

    text[sizeof(text) - 1U] = '\0';
    if (NULL != s_imu_calib_text_sink)
    {
        s_imu_calib_text_sink(text);
    }
    else
    {
        printf("%s\r\n", text);
    }
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

/* ========================= Ellipsoid fitting math ========================= */

/* 9x9 Gaussian elimination with partial pivoting, solves A*x = b in-place.
 * A is stored row-major in a[9][10] (augmented matrix). Solution returned in x[9].
 * Returns 0 on success, -1 on singular matrix. */
static int32_t gauss_solve_9x9(float a[9][10], float x[9])
{
    uint8_t i, j, k;
    for (i = 0; i < 9; i++)
    {
        /* partial pivoting */
        float max_val = fabsf_local(a[i][i]);
        uint8_t max_row = i;
        for (k = (uint8_t)(i + 1U); k < 9U; k++)
        {
            float v = fabsf_local(a[k][i]);
            if (v > max_val)
            {
                max_val = v;
                max_row = k;
            }
        }
        if (max_val < 1.0e-12f)
        {
            return -1;
        }
        if (max_row != i)
        {
            for (j = 0; j < 10; j++)
            {
                float tmp = a[i][j];
                a[i][j] = a[max_row][j];
                a[max_row][j] = tmp;
            }
        }
        /* elimination */
        for (k = (uint8_t)(i + 1U); k < 9U; k++)
        {
            float factor = a[k][i] / a[i][i];
            for (j = i; j < 10; j++)
            {
                a[k][j] -= factor * a[i][j];
            }
        }
    }
    /* back substitution */
    for (i = 9; i-- > 0;)
    {
        float s = a[i][9];
        for (j = (uint8_t)(i + 1U); j < 9U; j++)
        {
            s -= a[i][j] * x[j];
        }
        x[i] = s / a[i][i];
    }
    return 0;
}

/* 3x3 symmetric matrix inverse using adjugate method.
 * Input: symmetric A (only uses upper triangle pattern but reads all).
 * Output: inv. Returns 0 on success, -1 if singular. */
static int32_t mat3_inverse_sym(const float A[3][3], float inv[3][3])
{
    float det;
    float cofactor[3][3];

    cofactor[0][0] = A[1][1] * A[2][2] - A[1][2] * A[2][1];
    cofactor[0][1] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]);
    cofactor[0][2] = A[1][0] * A[2][1] - A[1][1] * A[2][0];
    cofactor[1][0] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]);
    cofactor[1][1] = A[0][0] * A[2][2] - A[0][2] * A[2][0];
    cofactor[1][2] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]);
    cofactor[2][0] = A[0][1] * A[1][2] - A[0][2] * A[1][1];
    cofactor[2][1] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]);
    cofactor[2][2] = A[0][0] * A[1][1] - A[0][1] * A[1][0];

    det = A[0][0] * cofactor[0][0] + A[0][1] * cofactor[0][1] + A[0][2] * cofactor[0][2];
    if (fabsf_local(det) < 1.0e-12f)
    {
        return -1;
    }

    {
        float inv_det = 1.0f / det;
        uint8_t r, c;
        for (r = 0; r < 3; r++)
        {
            for (c = 0; c < 3; c++)
            {
                inv[r][c] = cofactor[c][r] * inv_det; /* transpose of cofactor */
            }
        }
    }
    return 0;
}

/* 3x3 Cholesky decomposition: A = L * L^T.
 * A must be symmetric positive-definite. L is lower-triangular output.
 * Returns 0 on success, -1 if not positive-definite. */
static int32_t cholesky_3x3(const float A[3][3], float L[3][3])
{
    memset(L, 0, 9U * sizeof(float));

    if (A[0][0] <= 0.0f)
    {
        return -1;
    }
    L[0][0] = sqrtf(A[0][0]);
    L[1][0] = A[1][0] / L[0][0];
    L[2][0] = A[2][0] / L[0][0];

    {
        float v = A[1][1] - L[1][0] * L[1][0];
        if (v <= 0.0f)
        {
            return -1;
        }
        L[1][1] = sqrtf(v);
    }
    L[2][1] = (A[2][1] - L[2][0] * L[1][0]) / L[1][1];

    {
        float v = A[2][2] - L[2][0] * L[2][0] - L[2][1] * L[2][1];
        if (v <= 0.0f)
        {
            return -1;
        }
        L[2][2] = sqrtf(v);
    }
    return 0;
}

/* Ellipsoid fit solver.
 * Given n (>=9) static orientation measurements orient[n][3] (in g),
 * fits an ellipsoid and returns:
 *   bias[3] = ellipsoid center
 *   M[3][3] = correction matrix such that M*(raw-bias) maps to unit sphere.
 * Returns 0 on success, -1 on failure. */
static int32_t ellip_fit_solve(const float orient[][3], uint8_t n, float bias[3], float M[3][3])
{
    /* Fit: p0*x^2 + p1*y^2 + p2*z^2 + p3*xy + p4*xz + p5*yz + p6*x + p7*y + p8*z = 1
     * Build normal equations AtA * p = Atb  (9x9 system) */
    float aug[9][10]; /* augmented matrix [AtA | Atb] */
    float p[9];
    float Q[3][3], Q_inv[3][3], g_vec[3];
    float k_val;
    float Q_scaled[3][3];
    uint8_t i, r, c;

    if (n < 9U)
    {
        return -1;
    }

    memset(aug, 0, sizeof(aug));

    for (i = 0; i < n; i++)
    {
        float x = orient[i][0];
        float y = orient[i][1];
        float z = orient[i][2];
        float row[9];

        row[0] = x * x;
        row[1] = y * y;
        row[2] = z * z;
        row[3] = x * y;
        row[4] = x * z;
        row[5] = y * z;
        row[6] = x;
        row[7] = y;
        row[8] = z;

        /* Accumulate AtA and Atb */
        for (r = 0; r < 9; r++)
        {
            for (c = 0; c < 9; c++)
            {
                aug[r][c] += row[r] * row[c];
            }
            aug[r][9] += row[r]; /* rhs = 1 for each measurement */
        }
    }

    if (gauss_solve_9x9(aug, p) != 0)
    {
        return -1;
    }

    /* Assemble symmetric Q and vector g */
    Q[0][0] = p[0];           Q[0][1] = p[3] * 0.5f;  Q[0][2] = p[4] * 0.5f;
    Q[1][0] = p[3] * 0.5f;   Q[1][1] = p[1];          Q[1][2] = p[5] * 0.5f;
    Q[2][0] = p[4] * 0.5f;   Q[2][1] = p[5] * 0.5f;  Q[2][2] = p[2];

    g_vec[0] = p[6] * 0.5f;
    g_vec[1] = p[7] * 0.5f;
    g_vec[2] = p[8] * 0.5f;

    /* bias = -Q^{-1} * g */
    if (mat3_inverse_sym(Q, Q_inv) != 0)
    {
        return -1;
    }

    {
        float neg_g[3] = {-g_vec[0], -g_vec[1], -g_vec[2]};
        mat3_mul_vec(Q_inv, neg_g, bias);
    }

    /* k = g^T * Q^{-1} * g - d, where d = -1 (since rhs was 1)
     * Actually k = bias^T * Q * bias + 1 is simpler since bias = -Q^{-1}*g */
    {
        float Qb[3];
        mat3_mul_vec(Q, bias, Qb);
        k_val = bias[0] * Qb[0] + bias[1] * Qb[1] + bias[2] * Qb[2] + 1.0f;
    }

    if (k_val < 1.0e-6f)
    {
        return -1;
    }

    /* Q_scaled = Q / k, so that the fitted ellipsoid is (v-bias)^T * Q_scaled * (v-bias) = 1 */
    for (r = 0; r < 3; r++)
    {
        for (c = 0; c < 3; c++)
        {
            Q_scaled[r][c] = Q[r][c] / k_val;
        }
    }

    /* M = cholesky(Q_scaled)^T so that M*(v-bias) maps to unit sphere */
    {
        float L[3][3];
        if (cholesky_3x3(Q_scaled, L) != 0)
        {
            return -1;
        }
        /* M = L^T (transpose of lower-triangular Cholesky factor) */
        for (r = 0; r < 3; r++)
        {
            for (c = 0; c < 3; c++)
            {
                M[r][c] = L[c][r];
            }
        }
    }

    return 0;
}

typedef enum
{
    ELLIP_VALIDATE_OK = 0,
    ELLIP_VALIDATE_ERR_BIAS = -1,
    ELLIP_VALIDATE_ERR_DIAG = -2,
    ELLIP_VALIDATE_ERR_NORM = -3
} EllipValidateResult_e;

static int32_t ellip_validate_solution(const float orient[][3],
                                       uint8_t orient_count,
                                       const float bias[3],
                                       const float M[3][3],
                                       float *bias_norm_g,
                                       float *fit_rms_g,
                                       float *max_norm_err_g,
                                       uint8_t *diag_axis,
                                       float *diag_value)
{
    float bias_norm;
    float sum_sq_norm_err = 0.0f;
    float max_norm_err = 0.0f;
    uint8_t i;

    if ((NULL == orient) || (NULL == bias) || (NULL == M) || (orient_count == 0U))
    {
        return ELLIP_VALIDATE_ERR_NORM;
    }

    bias_norm = vec3_norm(bias[0], bias[1], bias[2]);
    if (bias_norm_g != NULL)
    {
        *bias_norm_g = bias_norm;
    }
    if (bias_norm > ELLIP_POST_BIAS_NORM_MAX_G)
    {
        return ELLIP_VALIDATE_ERR_BIAS;
    }

    for (i = 0U; i < 3U; i++)
    {
        if ((M[i][i] < ELLIP_POST_DIAG_MIN) || (M[i][i] > ELLIP_POST_DIAG_MAX))
        {
            if (diag_axis != NULL)
            {
                *diag_axis = i;
            }
            if (diag_value != NULL)
            {
                *diag_value = M[i][i];
            }
            return ELLIP_VALIDATE_ERR_DIAG;
        }
    }

    for (i = 0U; i < orient_count; i++)
    {
        float centered[3];
        float corrected[3];
        float corr_norm;
        float norm_err;
        uint8_t axis;

        for (axis = 0U; axis < 3U; axis++)
        {
            centered[axis] = orient[i][axis] - bias[axis];
        }

        mat3_mul_vec(M, centered, corrected);
        corr_norm = vec3_norm(corrected[0], corrected[1], corrected[2]);
        norm_err = fabsf_local(corr_norm - 1.0f);
        sum_sq_norm_err += norm_err * norm_err;
        if (norm_err > max_norm_err)
        {
            max_norm_err = norm_err;
        }
    }

    if (fit_rms_g != NULL)
    {
        *fit_rms_g = sqrtf(sum_sq_norm_err / (float)orient_count);
    }
    if (max_norm_err_g != NULL)
    {
        *max_norm_err_g = max_norm_err;
    }

    if (max_norm_err > ELLIP_POST_NORM_ERR_MAX_G)
    {
        return ELLIP_VALIDATE_ERR_NORM;
    }

    return ELLIP_VALIDATE_OK;
}

static int32_t ellip_apply_solution(const float bias[3], const float M[3][3])
{
    AccelCalibrationParams_t params;

    if ((NULL == bias) || (NULL == M))
    {
        return -1;
    }

    AccelCalibration_GetParams(&params);
    memcpy(params.accel_bias_g, bias, sizeof(params.accel_bias_g));
    memcpy(params.accel_corr_matrix, M, sizeof(params.accel_corr_matrix));
    params.accel_scale[0] = M[0][0];
    params.accel_scale[1] = M[1][1];
    params.accel_scale[2] = M[2][2];
    params.use_full_matrix = 1U;

    return AccelCalibration_LoadParams(&params) ? 0 : -1;
}

static bool euler_ready(void)
{
    /* �� sin^2+cos^2��1 �ж���̬�����Ǻ�����Ч���ų�δ��ʼ����̬���� */
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

    /* ��λ����ʸ��(g)�ڻ���ϵ�ķ��� */
    *gx = -g_euler.sin_pitch;
    *gy = g_euler.sin_roll * g_euler.cos_pitch;
    *gz = g_euler.cos_roll * g_euler.cos_pitch;
}

static float calc_accel_down_from_body(const float accel_body_mps2[3])
{
    /* 输入为机体系FRD线性加速度，输出为水平系Down方向加速度。
     * 忽略yaw，仅去除roll/pitch对坐标轴的倾斜影响。
     */
    /* ʹ����̬�ǽ�У�������ϵ���ٶ�ͶӰ�� NED �� Down �� */
    const float sin_pitch = g_euler.sin_pitch;
    const float cos_pitch = g_euler.cos_pitch;
    const float sin_roll = g_euler.sin_roll;
    const float cos_roll = g_euler.cos_roll;

    const float r31 = -sin_pitch;
    const float r32 = sin_roll * cos_pitch;
    const float r33 = cos_roll * cos_pitch;

    return r31 * accel_body_mps2[0] +
           r32 * accel_body_mps2[1] +
           r33 * accel_body_mps2[2];
}

static void rotate_body_linear_to_level(const float accel_body_mps2[3], float accel_level_mps2[3])
{
    /* 输入为机体系FRD线性加速度，单位m/s^2。
     * 输出为水平系线性加速度：+X机头前方，+Y机体右侧，+Z为Down。
     * 当关闭 yaw 参与时，仅去除roll/pitch导致的坐标倾斜。
     */
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
    r12 = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
    r13 = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

    r21 = cos_pitch * sin_yaw;
    r22 = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
    r23 = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

    r31 = -sin_pitch;
    r32 = sin_roll * cos_pitch;
    r33 = cos_roll * cos_pitch;
#else
    /* yaw=0: ���� roll/pitch ��ת�����Ե�ǰ����ƫб���򻯼��� */
    r11 = cos_pitch;
    r12 = sin_roll * sin_pitch;
    r13 = cos_roll * sin_pitch;

    r21 = 0.0f;
    r22 = cos_roll;
    r23 = -sin_roll;

    r31 = -sin_pitch;
    r32 = sin_roll * cos_pitch;
    r33 = cos_roll * cos_pitch;
#endif

    accel_level_mps2[0] = r11 * accel_body_mps2[0] + r12 * accel_body_mps2[1] + r13 * accel_body_mps2[2];
    accel_level_mps2[1] = r21 * accel_body_mps2[0] + r22 * accel_body_mps2[1] + r23 * accel_body_mps2[2];
    accel_level_mps2[2] = r31 * accel_body_mps2[0] + r32 * accel_body_mps2[1] + r33 * accel_body_mps2[2];
}

static void sanitize_scale(void)
{
    /* ��ֹ scale �쳣��NaN/Inf/��С����ǯλ����ȫ��Χ */
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
    /* ͳһ������Ч�Լ�飺��ֵ��Ч + ���̺��� + ��̬���� */
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
    /* ����������ϵͳһ��ת������ϵ���궨������ʱ��ʹ�ã� */
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

/*
 * 函数功能: 读取当前帧供 IMU 校准使用的原始传感器物理量。
 * 输入参数:
 *   accel_sensor_g - 输出传感器坐标系原始加速度，单位 g；仅量程换算与符号映射
 *   gyro_sensor_dps - 输出传感器坐标系原始角速度，单位 dps；已扣除陀螺仪零偏
 * 输出参数/返回值:
 *   通过数组返回当前帧原始 IMU 数据；空指针时直接返回
 */
static void imu_calib_get_raw_sensor_sample(float accel_sensor_g[3], float gyro_sensor_dps[3])
{
    if ((accel_sensor_g == NULL) || (gyro_sensor_dps == NULL))
    {
        return;
    }

    IMU_GetRawSampleForCalibration(&gyro_sensor_dps[0],
                                   &gyro_sensor_dps[1],
                                   &gyro_sensor_dps[2],
                                   &accel_sensor_g[0],
                                   &accel_sensor_g[1],
                                   &accel_sensor_g[2]);
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
    /* ��ֹ�궨����У�飺ȷ�Ͻ��ƾ�ֹ��|a|��1g �ҽ��ٶȺ�С */
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
    /* ����ƫ�� bias ΢�������ھ�ֹ����Ч�����¸��� */
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
    /* ���� scale ΢����ʹȥƫ�� |a| �𲽱ƽ� 1g */
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
    /* ����ָ��ƽ��ͳ�� |a| �ľ�ֵ�ͱ�׼�����У׼�����۲� */
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

/* V1 legacy blob layout for backward compatibility */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float gyro_bias_dps[3];
    float accel_bias_g[3];
    float accel_scale[3];
    float imu_to_body[3][3];
    uint32_t reserved[8];
} IMUCalibBlobV1_t;

/* Check if raw flash data contains a valid blob (v1 or v2) */
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
    /* Accept v2 directly */
    if ((blob->version == IMU_CALIB_FLASH_VERSION) && (blob->size == (uint16_t)sizeof(IMUCalibBlob_t)))
    {
        return 1U;
    }
    /* Accept v1 by checking magic+version on same raw memory (sizes match for both structs) */
    if (blob->version == IMU_CALIB_FLASH_VERSION_V1)
    {
        return 1U;
    }
    return 0U;
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
    memcpy(blob->accel_corr_matrix, params.accel_corr_matrix, sizeof(blob->accel_corr_matrix));
    memcpy(blob->imu_to_body, params.imu_to_body, sizeof(blob->imu_to_body));
}

static uint8_t imu_calib_apply_blob(const IMUCalibBlob_t *blob)
{
    AccelCalibrationParams_t params;

    if (blob == NULL)
    {
        return 0U;
    }

    /* Check raw magic first */
    if (blob->magic != IMU_CALIB_FLASH_MAGIC)
    {
        return 0U;
    }

    memset(&params, 0, sizeof(params));

    if (blob->version == IMU_CALIB_FLASH_VERSION_V1)
    {
        /* V1 layout differs from V2: use V1 struct to correctly parse fields.
         * Both structs are the same total size, just field layout differs. */
        const IMUCalibBlobV1_t *v1 = (const IMUCalibBlobV1_t *)blob;

        params.accel_bias_g[0] = v1->accel_bias_g[0];
        params.accel_bias_g[1] = v1->accel_bias_g[1];
        params.accel_bias_g[2] = v1->accel_bias_g[2];
        params.accel_scale[0] = v1->accel_scale[0];
        params.accel_scale[1] = v1->accel_scale[1];
        params.accel_scale[2] = v1->accel_scale[2];
        params.use_full_matrix = 0U;
        memset(params.accel_corr_matrix, 0, sizeof(params.accel_corr_matrix));
        params.accel_corr_matrix[0][0] = v1->accel_scale[0];
        params.accel_corr_matrix[1][1] = v1->accel_scale[1];
        params.accel_corr_matrix[2][2] = v1->accel_scale[2];
        memcpy(params.imu_to_body, v1->imu_to_body, sizeof(params.imu_to_body));
        params.gravity_mps2 = ACCEL_CALIBRATION_GRAVITY_MSS;

        if (!AccelCalibration_LoadParams(&params))
        {
            return 0U;
        }

        ICM42688_SetGyroBiasDps(v1->gyro_bias_dps[0], v1->gyro_bias_dps[1], v1->gyro_bias_dps[2], 1U);
        printf("cal,loaded_v1_as_diag\r\n");
        return 1U;
    }

    if (blob->version == IMU_CALIB_FLASH_VERSION)
    {
        if (blob->size != (uint16_t)sizeof(IMUCalibBlob_t))
        {
            return 0U;
        }

        params.accel_bias_g[0] = blob->accel_bias_g[0];
        params.accel_bias_g[1] = blob->accel_bias_g[1];
        params.accel_bias_g[2] = blob->accel_bias_g[2];
        memcpy(params.accel_corr_matrix, blob->accel_corr_matrix, sizeof(params.accel_corr_matrix));
        params.accel_scale[0] = blob->accel_corr_matrix[0][0];
        params.accel_scale[1] = blob->accel_corr_matrix[1][1];
        params.accel_scale[2] = blob->accel_corr_matrix[2][2];

        /* Determine if it's a full matrix or diagonal only */
        {
            float off_diag_sum = fabsf_local(blob->accel_corr_matrix[0][1]) +
                                 fabsf_local(blob->accel_corr_matrix[0][2]) +
                                 fabsf_local(blob->accel_corr_matrix[1][0]) +
                                 fabsf_local(blob->accel_corr_matrix[1][2]) +
                                 fabsf_local(blob->accel_corr_matrix[2][0]) +
                                 fabsf_local(blob->accel_corr_matrix[2][1]);
            params.use_full_matrix = (off_diag_sum > 1.0e-6f) ? 1U : 0U;
        }

        memcpy(params.imu_to_body, blob->imu_to_body, sizeof(params.imu_to_body));
        params.gravity_mps2 = ACCEL_CALIBRATION_GRAVITY_MSS;

        if (!AccelCalibration_LoadParams(&params))
        {
            return 0U;
        }

        ICM42688_SetGyroBiasDps(blob->gyro_bias_dps[0], blob->gyro_bias_dps[1], blob->gyro_bias_dps[2], 1U);
        return 1U;
    }

    return 0U;
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
    printf("cal,dump,acc,%f,%f,%f,%f,%f,%f,%f,%u\r\n",
           params.accel_bias_g[0], params.accel_bias_g[1], params.accel_bias_g[2],
           params.accel_scale[0], params.accel_scale[1], params.accel_scale[2],
           params.gravity_mps2, (unsigned int)params.use_full_matrix);
    if (params.use_full_matrix != 0U)
    {
        printf("cal,dump,cm0,%f,%f,%f\r\n",
               params.accel_corr_matrix[0][0], params.accel_corr_matrix[0][1], params.accel_corr_matrix[0][2]);
        printf("cal,dump,cm1,%f,%f,%f\r\n",
               params.accel_corr_matrix[1][0], params.accel_corr_matrix[1][1], params.accel_corr_matrix[1][2]);
        printf("cal,dump,cm2,%f,%f,%f\r\n",
               params.accel_corr_matrix[2][0], params.accel_corr_matrix[2][1], params.accel_corr_matrix[2][2]);
    }
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

    if (blob.version == IMU_CALIB_FLASH_VERSION_V1)
    {
        const IMUCalibBlobV1_t *v1 = (const IMUCalibBlobV1_t *)&blob;
        printf("cal,flash,gyro,%f,%f,%f\r\n",
               v1->gyro_bias_dps[0], v1->gyro_bias_dps[1], v1->gyro_bias_dps[2]);
        printf("cal,flash,acc,bias,%f,%f,%f,scale,%f,%f,%f\r\n",
               v1->accel_bias_g[0], v1->accel_bias_g[1], v1->accel_bias_g[2],
               v1->accel_scale[0], v1->accel_scale[1], v1->accel_scale[2]);
        printf("cal,flash,r0,%f,%f,%f\r\n",
               v1->imu_to_body[0][0], v1->imu_to_body[0][1], v1->imu_to_body[0][2]);
        printf("cal,flash,r1,%f,%f,%f\r\n",
               v1->imu_to_body[1][0], v1->imu_to_body[1][1], v1->imu_to_body[1][2]);
        printf("cal,flash,r2,%f,%f,%f\r\n",
               v1->imu_to_body[2][0], v1->imu_to_body[2][1], v1->imu_to_body[2][2]);
    }
    else
    {
        printf("cal,flash,gyro,%f,%f,%f\r\n",
               blob.gyro_bias_dps[0], blob.gyro_bias_dps[1], blob.gyro_bias_dps[2]);
        printf("cal,flash,acc,bias,%f,%f,%f\r\n",
               blob.accel_bias_g[0], blob.accel_bias_g[1], blob.accel_bias_g[2]);
        printf("cal,flash,cm0,%f,%f,%f\r\n",
               blob.accel_corr_matrix[0][0], blob.accel_corr_matrix[0][1], blob.accel_corr_matrix[0][2]);
        printf("cal,flash,cm1,%f,%f,%f\r\n",
               blob.accel_corr_matrix[1][0], blob.accel_corr_matrix[1][1], blob.accel_corr_matrix[1][2]);
        printf("cal,flash,cm2,%f,%f,%f\r\n",
               blob.accel_corr_matrix[2][0], blob.accel_corr_matrix[2][1], blob.accel_corr_matrix[2][2]);
        printf("cal,flash,r0,%f,%f,%f\r\n",
               blob.imu_to_body[0][0], blob.imu_to_body[0][1], blob.imu_to_body[0][2]);
        printf("cal,flash,r1,%f,%f,%f\r\n",
               blob.imu_to_body[1][0], blob.imu_to_body[1][1], blob.imu_to_body[1][2]);
        printf("cal,flash,r2,%f,%f,%f\r\n",
               blob.imu_to_body[2][0], blob.imu_to_body[2][1], blob.imu_to_body[2][2]);
    }
}

static void imu_calib_reset_runtime(void)
{
    memset(&s_imu_calib, 0, sizeof(s_imu_calib));
    memset(&s_ellip, 0, sizeof(s_ellip));
    memset(&s_ellip_manual, 0, sizeof(s_ellip_manual));
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
    imu_calib_emit_text("OK imu gyro 开始 请保持飞机静止");
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

static void imu_calib_manual_emit_next_commands(void)
{
    imu_calib_emit_text("OK imu accel_man 提示 继续采点请发 imu acc collect");
    imu_calib_emit_text("OK imu accel_man 提示 结束并求解请发 imu acc stop");
}

static void imu_calib_manual_reset_current_pose(void)
{
    s_ellip_manual.stable_samples = 0U;
    s_ellip_manual.orient_samples = 0U;
    memset(s_ellip_manual.collect_mean, 0, sizeof(s_ellip_manual.collect_mean));
    memset(s_ellip_manual.collect_m2, 0, sizeof(s_ellip_manual.collect_m2));
}

static void imu_calib_manual_enter_ready_state(void)
{
    imu_calib_manual_reset_current_pose();
    s_ellip_manual.substate = IMU_CALIB_MANUAL_SUBSTATE_READY;
}

static void imu_calib_manual_calc_pose_std_stats(float *mean_pose_std_g, float *max_pose_std_g)
{
    float mean_std = 0.0f;
    float max_std = 0.0f;
    uint8_t i;

    if (s_ellip_manual.orient_count == 0U)
    {
        if (mean_pose_std_g != NULL)
        {
            *mean_pose_std_g = 0.0f;
        }
        if (max_pose_std_g != NULL)
        {
            *max_pose_std_g = 0.0f;
        }
        return;
    }

    for (i = 0U; i < s_ellip_manual.orient_count; i++)
    {
        mean_std += s_ellip_manual.pose_std_max[i];
        if (s_ellip_manual.pose_std_max[i] > max_std)
        {
            max_std = s_ellip_manual.pose_std_max[i];
        }
    }

    if (mean_pose_std_g != NULL)
    {
        *mean_pose_std_g = mean_std / (float)s_ellip_manual.orient_count;
    }
    if (max_pose_std_g != NULL)
    {
        *max_pose_std_g = max_std;
    }
}

static void imu_calib_manual_begin_collect(void)
{
    imu_calib_manual_reset_current_pose();
    s_ellip_manual.substate = IMU_CALIB_MANUAL_SUBSTATE_WAIT_STATIC;
    imu_calib_emit_text("OK imu accel_man 等待静止 pose=%u stable_target=%u",
                        (unsigned int)(s_ellip_manual.orient_count + 1U),
                        (unsigned int)ELLIP_PRE_STABLE_SAMPLES);
    imu_calib_emit_text("OK imu accel_man 提示 静止通过后自动采集 target=%u",
                        (unsigned int)ELLIP_MANUAL_ORIENT_SAMPLES);
}

/* ========================= Ellipsoid calibration ========================= */

static void imu_calib_start_ellip(void)
{
    imu_calib_reset_runtime();
    memset(&s_ellip, 0, sizeof(s_ellip));
    s_imu_calib.busy = 1U;
    s_imu_calib.mode = IMU_CALIB_MODE_ELLIP;
    imu_calib_emit_text("OK imu accel 开始 请缓慢切换多个静止姿态");
    imu_calib_emit_text("OK imu accel 提示 每次放稳后保持静止 系统会自动采集姿态点");
}

static void imu_calib_start_ellip_manual(void)
{
    imu_calib_reset_runtime();
    s_imu_calib.busy = 1U;
    s_imu_calib.mode = IMU_CALIB_MODE_ELLIP_MANUAL;
    s_ellip_manual.substate = IMU_CALIB_MANUAL_SUBSTATE_READY;
    imu_calib_emit_text("OK imu accel_man 开始 请手动切换多个静止姿态");
    imu_calib_emit_text("OK imu accel_man 提示 单次采点请发 imu acc collect");
    imu_calib_emit_text("OK imu accel_man 提示 结束并求解请发 imu acc stop");
}

static int32_t imu_calib_manual_solve(void)
{
    float bias[3];
    float M[3][3];
    float bias_norm_g = 0.0f;
    float fit_rms_g = 0.0f;
    float max_norm_err_g = 0.0f;
    float mean_pose_std_g = 0.0f;
    float max_pose_std_g = 0.0f;
    float diag_value = 0.0f;
    uint8_t diag_axis = 0U;
    int32_t validate_ret;

    if (s_ellip_manual.orient_count < ELLIP_MIN_ORIENT)
    {
        imu_calib_emit_text("ERR imu accel_man 失败 原因=姿态点不足 count=%u min=%u",
                            (unsigned int)s_ellip_manual.orient_count,
                            (unsigned int)ELLIP_MIN_ORIENT);
        return -1;
    }

    imu_calib_manual_calc_pose_std_stats(&mean_pose_std_g, &max_pose_std_g);

    if (ellip_fit_solve(s_ellip_manual.orient_mean, s_ellip_manual.orient_count, bias, M) != 0)
    {
        imu_calib_emit_text("ERR imu accel_man 失败 原因=求解失败 pose_count=%u",
                            (unsigned int)s_ellip_manual.orient_count);
        return -1;
    }

    validate_ret = ellip_validate_solution(s_ellip_manual.orient_mean,
                                           s_ellip_manual.orient_count,
                                           bias,
                                           M,
                                           &bias_norm_g,
                                           &fit_rms_g,
                                           &max_norm_err_g,
                                           &diag_axis,
                                           &diag_value);
    if (validate_ret == ELLIP_VALIDATE_ERR_BIAS)
    {
        imu_calib_emit_text("ERR imu accel_man 失败 原因=偏置过大 value=%f",
                            (double)bias_norm_g);
        return -1;
    }
    if (validate_ret == ELLIP_VALIDATE_ERR_DIAG)
    {
        imu_calib_emit_text("ERR imu accel_man 失败 原因=矩阵对角异常 axis=%u value=%f",
                            (unsigned int)diag_axis,
                            (double)diag_value);
        return -1;
    }
    if (validate_ret == ELLIP_VALIDATE_ERR_NORM)
    {
        imu_calib_emit_text("ERR imu accel_man 失败 原因=范数误差过大 fit_rms_g=%f max_norm_err_g=%f",
                            (double)fit_rms_g,
                            (double)max_norm_err_g);
        return -1;
    }

    if (ellip_apply_solution(bias, M) != 0)
    {
        imu_calib_emit_text("ERR imu accel_man 失败 原因=应用失败");
        return -1;
    }

    {
        uint8_t save_ok = IMUCalib_SaveCurrentToFlash();
        imu_calib_emit_text("OK imu accel_man 结果 pose_count=%u mean_pose_std_g=%f max_pose_std_g=%f",
                            (unsigned int)s_ellip_manual.orient_count,
                            (double)mean_pose_std_g,
                            (double)max_pose_std_g);
        imu_calib_emit_text("OK imu accel_man 结果 bias_norm_g=%f fit_rms_g=%f max_norm_err_g=%f",
                            (double)bias_norm_g,
                            (double)fit_rms_g,
                            (double)max_norm_err_g);
        imu_calib_emit_text("OK imu accel_man 完成 save=%u bias_g=%f,%f,%f",
                            (unsigned int)save_ok,
                            (double)bias[0],
                            (double)bias[1],
                            (double)bias[2]);
        imu_calib_emit_text("OK imu accel_man 矩阵 r0=%f,%f,%f",
                            (double)M[0][0], (double)M[0][1], (double)M[0][2]);
        imu_calib_emit_text("OK imu accel_man 矩阵 r1=%f,%f,%f",
                            (double)M[1][0], (double)M[1][1], (double)M[1][2]);
        imu_calib_emit_text("OK imu accel_man 矩阵 r2=%f,%f,%f",
                            (double)M[2][0], (double)M[2][1], (double)M[2][2]);
    }

    return 1;
}

static int32_t imu_calib_update_ellip_manual_step(void)
{
    float accel_sensor_g[3];
    float gyro_sensor_dps[3];
    float accel_body_g[3];
    float gyro_body_dps[3];
    float accel_norm_g;
    float gyro_norm_dps;

    if (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_READY)
    {
        return 0;
    }

    s_ellip_manual.total_samples++;
    if (s_ellip_manual.total_samples >= ELLIP_MANUAL_TIMEOUT_SAMPLES)
    {
        imu_calib_emit_text("ERR imu accel_man 失败 原因=超时 total_samples=%lu",
                            (unsigned long)s_ellip_manual.total_samples);
        return -1;
    }

    if (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_SOLVING)
    {
        return imu_calib_manual_solve();
    }

    imu_calib_get_raw_sensor_sample(accel_sensor_g, gyro_sensor_dps);

    if (!imu_sample_valid(accel_sensor_g[0], accel_sensor_g[1], accel_sensor_g[2],
                          gyro_sensor_dps[0], gyro_sensor_dps[1], gyro_sensor_dps[2]))
    {
        if (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_WAIT_STATIC)
        {
            s_ellip_manual.stable_samples = 0U;
            return 0;
        }

        imu_calib_emit_text("ERR imu accel_man 当前姿态点作废 原因=样本无效 samples=%lu",
                            (unsigned long)s_ellip_manual.orient_samples);
        imu_calib_manual_enter_ready_state();
        imu_calib_manual_emit_next_commands();
        return 0;
    }

    rotate_imu_to_body(accel_sensor_g, accel_body_g);
    rotate_imu_to_body(gyro_sensor_dps, gyro_body_dps);

    accel_norm_g = vec3_norm(accel_body_g[0], accel_body_g[1], accel_body_g[2]);
    gyro_norm_dps = vec3_norm(gyro_body_dps[0], gyro_body_dps[1], gyro_body_dps[2]);

    if ((gyro_norm_dps > ELLIP_STATIC_GYRO_MAX_DPS) ||
        (accel_norm_g < ELLIP_STATIC_ACC_MIN_G) ||
        (accel_norm_g > ELLIP_STATIC_ACC_MAX_G))
    {
        if (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_WAIT_STATIC)
        {
            s_ellip_manual.stable_samples = 0U;
            return 0;
        }

        imu_calib_emit_text("ERR imu accel_man 当前姿态点作废 原因=采集中检测到晃动 samples=%lu",
                            (unsigned long)s_ellip_manual.orient_samples);
        imu_calib_manual_enter_ready_state();
        imu_calib_manual_emit_next_commands();
        return 0;
    }

    if (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_WAIT_STATIC)
    {
        s_ellip_manual.stable_samples++;
        if (s_ellip_manual.stable_samples == ELLIP_PRE_STABLE_SAMPLES)
        {
            imu_calib_manual_reset_current_pose();
            s_ellip_manual.substate = IMU_CALIB_MANUAL_SUBSTATE_COLLECTING;
            imu_calib_emit_text("OK imu accel_man 开始采集 pose=%u target=%u",
                                (unsigned int)(s_ellip_manual.orient_count + 1U),
                                (unsigned int)ELLIP_MANUAL_ORIENT_SAMPLES);
        }
        return 0;
    }

    if (s_ellip_manual.substate != IMU_CALIB_MANUAL_SUBSTATE_COLLECTING)
    {
        return 0;
    }

    {
        uint32_t n = s_ellip_manual.orient_samples + 1U;
        uint8_t axis;
        s_ellip_manual.orient_samples = n;
        for (axis = 0U; axis < 3U; axis++)
        {
            update_running_stats(accel_body_g[axis],
                                 &s_ellip_manual.collect_mean[axis],
                                 &s_ellip_manual.collect_m2[axis],
                                 n);
        }
    }

    if (s_ellip_manual.orient_samples >= ELLIP_MANUAL_ORIENT_SAMPLES)
    {
        float std_x = std_from_m2(s_ellip_manual.collect_m2[0], s_ellip_manual.orient_samples);
        float std_y = std_from_m2(s_ellip_manual.collect_m2[1], s_ellip_manual.orient_samples);
        float std_z = std_from_m2(s_ellip_manual.collect_m2[2], s_ellip_manual.orient_samples);
        float std_max = max3f_local(std_x, std_y, std_z);

        if (std_max > ELLIP_ORIENT_STD_MAX_G)
        {
            imu_calib_emit_text("ERR imu accel_man 当前姿态点作废 原因=噪声过大 pose=%u std_g=%f",
                                (unsigned int)(s_ellip_manual.orient_count + 1U),
                                (double)std_max);
            imu_calib_manual_enter_ready_state();
            imu_calib_manual_emit_next_commands();
            return 0;
        }

        s_ellip_manual.orient_mean[s_ellip_manual.orient_count][0] = s_ellip_manual.collect_mean[0];
        s_ellip_manual.orient_mean[s_ellip_manual.orient_count][1] = s_ellip_manual.collect_mean[1];
        s_ellip_manual.orient_mean[s_ellip_manual.orient_count][2] = s_ellip_manual.collect_mean[2];
        s_ellip_manual.pose_std_max[s_ellip_manual.orient_count] = std_max;
        s_ellip_manual.orient_count++;

        imu_calib_emit_text("OK imu accel_man 姿态点完成 pose=%u mean_g=%f,%f,%f std_g=%f,%f,%f std_max_g=%f",
                            (unsigned int)s_ellip_manual.orient_count,
                            (double)s_ellip_manual.collect_mean[0],
                            (double)s_ellip_manual.collect_mean[1],
                            (double)s_ellip_manual.collect_mean[2],
                            (double)std_x,
                            (double)std_y,
                            (double)std_z,
                            (double)std_max);

        imu_calib_manual_enter_ready_state();
        if (s_ellip_manual.orient_count >= ELLIP_MANUAL_MAX_ORIENT)
        {
            imu_calib_emit_text("OK imu accel_man 提示 姿态点已达上限 请发 imu acc stop");
            return 0;
        }

        imu_calib_manual_emit_next_commands();
    }

    return 0;
}

static int32_t imu_calib_update_ellip_step(void)
{
    float accel_sensor_g[3];
    float gyro_sensor_dps[3];
    float accel_body_g[3];
    float gyro_body_dps[3];
    float accel_norm_g;
    float gyro_norm_dps;

    s_ellip.total_samples++;
    if (s_ellip.total_samples >= ELLIP_TIMEOUT_SAMPLES)
    {
        imu_calib_emit_text("ERR imu accel 失败 原因=超时 total_samples=%lu",
                            (unsigned long)s_ellip.total_samples);
        return -1;
    }

    imu_calib_get_raw_sensor_sample(accel_sensor_g, gyro_sensor_dps);

    if (!imu_sample_valid(accel_sensor_g[0], accel_sensor_g[1], accel_sensor_g[2],
                          gyro_sensor_dps[0], gyro_sensor_dps[1], gyro_sensor_dps[2]))
    {
        s_ellip.stable_samples = 0U;
        s_ellip.collecting = 0U;
        s_ellip.orient_samples = 0U;
        return 0;
    }

    rotate_imu_to_body(accel_sensor_g, accel_body_g);
    rotate_imu_to_body(gyro_sensor_dps, gyro_body_dps);

    accel_norm_g = vec3_norm(accel_body_g[0], accel_body_g[1], accel_body_g[2]);
    gyro_norm_dps = vec3_norm(gyro_body_dps[0], gyro_body_dps[1], gyro_body_dps[2]);

    /* If waiting for movement after completing an orientation */
    if (s_ellip.wait_move != 0U)
    {
        if (gyro_norm_dps > ELLIP_MOVING_GYRO_DPS)
        {
            s_ellip.wait_move = 0U;
            s_ellip.stable_samples = 0U;
            s_ellip.collecting = 0U;
            s_ellip.orient_samples = 0U;
        }
        return 0;
    }

    /* Check static condition */
    if ((gyro_norm_dps > ELLIP_STATIC_GYRO_MAX_DPS) ||
        (accel_norm_g < ELLIP_STATIC_ACC_MIN_G) ||
        (accel_norm_g > ELLIP_STATIC_ACC_MAX_G))
    {
        s_ellip.stable_samples = 0U;
        s_ellip.collecting = 0U;
        s_ellip.orient_samples = 0U;
        memset(s_ellip.orient_sum, 0, sizeof(s_ellip.orient_sum));
        memset(s_ellip.orient_m2, 0, sizeof(s_ellip.orient_m2));
        return 0;
    }

    /* Pre-stabilization */
    if (s_ellip.stable_samples < ELLIP_PRE_STABLE_SAMPLES)
    {
        s_ellip.stable_samples++;
        if (s_ellip.stable_samples == ELLIP_PRE_STABLE_SAMPLES)
        {
            /* Start collecting */
            s_ellip.collecting = 1U;
            s_ellip.orient_samples = 0U;
            memset(s_ellip.orient_sum, 0, sizeof(s_ellip.orient_sum));
            memset(s_ellip.orient_m2, 0, sizeof(s_ellip.orient_m2));
            imu_calib_emit_text("OK imu accel 开始采集 pose=%u target=%u",
                                (unsigned int)(s_ellip.orient_count + 1U),
                                (unsigned int)ELLIP_ORIENT_SAMPLES);
        }
        return 0;
    }

    if (s_ellip.collecting == 0U)
    {
        return 0;
    }

    /* Collect samples using Welford online statistics */
    {
        uint32_t n = s_ellip.orient_samples + 1U;
        uint8_t ax;
        s_ellip.orient_samples = n;
        for (ax = 0; ax < 3; ax++)
        {
            float delta = accel_body_g[ax] - s_ellip.orient_sum[ax];
            s_ellip.orient_sum[ax] += delta / (float)n;  /* orient_sum is actually mean */
            s_ellip.orient_m2[ax] += delta * (accel_body_g[ax] - s_ellip.orient_sum[ax]);
        }
    }

    if (s_ellip.orient_samples >= ELLIP_ORIENT_SAMPLES)
    {
        /* Check standard deviation */
        float std_max = 0.0f;
        uint8_t ax;
        for (ax = 0; ax < 3; ax++)
        {
            float std_val = sqrtf(s_ellip.orient_m2[ax] / (float)s_ellip.orient_samples);
            if (std_val > std_max)
            {
                std_max = std_val;
            }
        }

        if (std_max > ELLIP_ORIENT_STD_MAX_G)
        {
            imu_calib_emit_text("ERR imu accel 姿态点噪声过大 pose=%u std_g=%f",
                                (unsigned int)(s_ellip.orient_count + 1U),
                                (double)std_max);
            /* Reset and retry this orientation */
            s_ellip.collecting = 0U;
            s_ellip.orient_samples = 0U;
            s_ellip.stable_samples = 0U;
            return 0;
        }

        /* Check angular separation from existing orientations */
        {
            uint8_t oi;
            float new_norm = vec3_norm(s_ellip.orient_sum[0], s_ellip.orient_sum[1], s_ellip.orient_sum[2]);
            float cos_min_angle = cosf(ELLIP_ORIENT_ANGLE_MIN_DEG * DEG_TO_RAD);

            if (new_norm < 0.01f)
            {
                s_ellip.collecting = 0U;
                s_ellip.orient_samples = 0U;
                s_ellip.stable_samples = 0U;
                return 0;
            }

            for (oi = 0; oi < s_ellip.orient_count; oi++)
            {
                float old_norm = vec3_norm(s_ellip.orient_mean[oi][0], s_ellip.orient_mean[oi][1], s_ellip.orient_mean[oi][2]);
                float dot_val = s_ellip.orient_sum[0] * s_ellip.orient_mean[oi][0] +
                                s_ellip.orient_sum[1] * s_ellip.orient_mean[oi][1] +
                                s_ellip.orient_sum[2] * s_ellip.orient_mean[oi][2];
                float cos_angle = dot_val / (new_norm * old_norm + 1.0e-9f);

                if (cos_angle > cos_min_angle)
                {
                    imu_calib_emit_text("ERR imu accel 姿态点过近 ref=%u",
                                        (unsigned int)(oi + 1U));
                    s_ellip.collecting = 0U;
                    s_ellip.orient_samples = 0U;
                    s_ellip.stable_samples = 0U;
                    s_ellip.wait_move = 1U;
                    return 0;
                }
            }
        }

        /* Accept this orientation */
        s_ellip.orient_mean[s_ellip.orient_count][0] = s_ellip.orient_sum[0];
        s_ellip.orient_mean[s_ellip.orient_count][1] = s_ellip.orient_sum[1];
        s_ellip.orient_mean[s_ellip.orient_count][2] = s_ellip.orient_sum[2];
        s_ellip.orient_count++;

        imu_calib_emit_text("OK imu accel 姿态点完成 pose=%u mean_g=%f,%f,%f",
                            (unsigned int)s_ellip.orient_count,
                            (double)s_ellip.orient_sum[0],
                            (double)s_ellip.orient_sum[1],
                            (double)s_ellip.orient_sum[2]);

        s_ellip.collecting = 0U;
        s_ellip.orient_samples = 0U;
        s_ellip.wait_move = 1U;

        /* Check if we have enough orientations */
        if (s_ellip.orient_count >= ELLIP_MIN_ORIENT)
        {
            /* Attempt solve */
            float bias[3];
            float M[3][3];

            imu_calib_emit_text("OK imu accel 开始求解 pose_count=%u",
                                (unsigned int)s_ellip.orient_count);

            if (ellip_fit_solve(s_ellip.orient_mean, s_ellip.orient_count, bias, M) != 0)
            {
                if (s_ellip.orient_count >= ELLIP_MAX_ORIENT)
                {
                    imu_calib_emit_text("ERR imu accel 失败 原因=求解失败 pose_count=%u",
                                        (unsigned int)s_ellip.orient_count);
                    return -1;
                }
                imu_calib_emit_text("OK imu accel 求解未通过 请继续增加姿态点 current=%u",
                                    (unsigned int)s_ellip.orient_count);
                s_ellip.wait_move = 1U;
                return 0;
            }

            /* Post-validation */
            {
                float bias_norm = vec3_norm(bias[0], bias[1], bias[2]);
                uint8_t vi;
                float max_norm_err = 0.0f;

                if (bias_norm > ELLIP_POST_BIAS_NORM_MAX_G)
                {
                    imu_calib_emit_text("%s imu accel %s value=%f",
                                        (s_ellip.orient_count >= ELLIP_MAX_ORIENT) ? "ERR" : "OK",
                                        (s_ellip.orient_count >= ELLIP_MAX_ORIENT) ? "失败 原因=偏置过大" : "求解未通过 原因=偏置过大",
                                        (double)bias_norm);
                    if (s_ellip.orient_count >= ELLIP_MAX_ORIENT)
                    {
                        return -1;
                    }
                    return 0;
                }

                /* Check diagonal elements of M */
                {
                    uint8_t di;
                    for (di = 0; di < 3; di++)
                    {
                        if ((M[di][di] < ELLIP_POST_DIAG_MIN) || (M[di][di] > ELLIP_POST_DIAG_MAX))
                        {
                            imu_calib_emit_text("%s imu accel %s axis=%u value=%f",
                                                (s_ellip.orient_count >= ELLIP_MAX_ORIENT) ? "ERR" : "OK",
                                                (s_ellip.orient_count >= ELLIP_MAX_ORIENT) ? "失败 原因=矩阵对角异常" : "求解未通过 原因=矩阵对角异常",
                                                (unsigned int)di,
                                                (double)M[di][di]);
                            if (s_ellip.orient_count >= ELLIP_MAX_ORIENT)
                            {
                                return -1;
                            }
                            return 0;
                        }
                    }
                }

                /* Check corrected norm for all orientations */
                for (vi = 0; vi < s_ellip.orient_count; vi++)
                {
                    float centered[3];
                    float corrected[3];
                    float corr_norm, norm_err;
                    uint8_t ci;

                    for (ci = 0; ci < 3; ci++)
                    {
                        centered[ci] = s_ellip.orient_mean[vi][ci] - bias[ci];
                    }
                    mat3_mul_vec(M, centered, corrected);
                    corr_norm = vec3_norm(corrected[0], corrected[1], corrected[2]);
                    norm_err = fabsf_local(corr_norm - 1.0f);
                    if (norm_err > max_norm_err)
                    {
                        max_norm_err = norm_err;
                    }
                }

                if (max_norm_err > ELLIP_POST_NORM_ERR_MAX_G)
                {
                    imu_calib_emit_text("%s imu accel %s value=%f",
                                        (s_ellip.orient_count >= ELLIP_MAX_ORIENT) ? "ERR" : "OK",
                                        (s_ellip.orient_count >= ELLIP_MAX_ORIENT) ? "失败 原因=范数误差过大" : "求解未通过 原因=范数误差过大",
                                        (double)max_norm_err);
                    if (s_ellip.orient_count >= ELLIP_MAX_ORIENT)
                    {
                        return -1;
                    }
                    return 0;
                }
            }

            /* Apply calibration */
            {
                AccelCalibrationParams_t params;
                AccelCalibration_GetParams(&params);

                memcpy(params.accel_bias_g, bias, sizeof(params.accel_bias_g));
                memcpy(params.accel_corr_matrix, M, sizeof(params.accel_corr_matrix));
                params.accel_scale[0] = M[0][0];
                params.accel_scale[1] = M[1][1];
                params.accel_scale[2] = M[2][2];
                params.use_full_matrix = 1U;

                if (!AccelCalibration_LoadParams(&params))
                {
                    imu_calib_emit_text("ERR imu accel 失败 原因=应用失败");
                    return -1;
                }
            }

            /* Save to flash */
            {
                uint8_t save_ok = IMUCalib_SaveCurrentToFlash();
                imu_calib_emit_text("OK imu accel 完成 save=%u bias_g=%f,%f,%f",
                                    (unsigned int)save_ok,
                                    (double)bias[0], (double)bias[1], (double)bias[2]);
                imu_calib_emit_text("OK imu accel 矩阵 r0=%f,%f,%f",
                                    (double)M[0][0], (double)M[0][1], (double)M[0][2]);
                imu_calib_emit_text("OK imu accel 矩阵 r1=%f,%f,%f",
                                    (double)M[1][0], (double)M[1][1], (double)M[1][2]);
                imu_calib_emit_text("OK imu accel 矩阵 r2=%f,%f,%f",
                                    (double)M[2][0], (double)M[2][1], (double)M[2][2]);
            }
            return 1;
        }

        /* Not enough orientations yet, continue */
        if (s_ellip.orient_count >= ELLIP_MAX_ORIENT)
        {
            imu_calib_emit_text("ERR imu accel 失败 原因=姿态点已达上限 count=%u",
                                (unsigned int)s_ellip.orient_count);
            return -1;
        }
    }

    return 0;
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
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gyro_norm_dps;
    float acc_norm_g;
    uint8_t static_ok;

    IMU_GetRawSampleForCalibration(&gx, &gy, &gz, &ax, &ay, &az);

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
                    imu_calib_emit_text("OK imu gyro 预稳定 progress=%u samples=%lu/%u",
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
            imu_calib_emit_text("OK imu gyro 进入采集阶段 target=%u",
                                (unsigned int)IMU_CALIB_GYRO_TARGET_VALID_SAMPLES);
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
                    imu_calib_emit_text("OK imu gyro 采集 progress=%u valid=%lu/%u",
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
            imu_calib_emit_text("ERR imu gyro 失败 原因=质量不足 bias_dps=%f,%f,%f std_dps=%f,%f,%f",
                                bx, by, bz, std_x, std_y, std_z);
            return -1;
        }

        ICM42688_SetGyroBiasDps(bx, by, bz, 1U);
        imu_calib_emit_text("OK imu gyro 完成 samples=%lu bias_dps=%f,%f,%f std_dps=%f,%f,%f",
                            (unsigned long)s_imu_calib.gyro_valid_samples,
                            bx, by, bz, std_x, std_y, std_z);
        return 1;
    }

    if (s_imu_calib.gyro_total_samples >= IMU_CALIB_GYRO_TIMEOUT_SAMPLES)
    {
        imu_calib_emit_text("ERR imu gyro 失败 原因=超时 valid=%lu total=%lu",
                            (unsigned long)s_imu_calib.gyro_valid_samples,
                            (unsigned long)s_imu_calib.gyro_total_samples);
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

    imu_calib_get_raw_sensor_sample(accel_sensor_g, gyro_sensor_dps);

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
        params.use_full_matrix = 0U;
        memset(params.accel_corr_matrix, 0, sizeof(params.accel_corr_matrix));

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
            params.accel_corr_matrix[axis][axis] = params.accel_scale[axis];
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
    else if (s_imu_calib.mode == IMU_CALIB_MODE_ELLIP)
    {
        progress = (uint32_t)((100U * s_ellip.orient_count) / ELLIP_MIN_ORIENT);
    }
    else if (s_imu_calib.mode == IMU_CALIB_MODE_ELLIP_MANUAL)
    {
        uint32_t base_progress = (uint32_t)((100U * s_ellip_manual.orient_count) / ELLIP_MIN_ORIENT);
        progress = base_progress;

        if (s_ellip_manual.orient_count < ELLIP_MIN_ORIENT)
        {
            uint32_t stage_current = 0U;
            const uint32_t stage_target = ELLIP_PRE_STABLE_SAMPLES + ELLIP_MANUAL_ORIENT_SAMPLES;

            if (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_WAIT_STATIC)
            {
                stage_current = s_ellip_manual.stable_samples;
            }
            else if (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_COLLECTING)
            {
                stage_current = ELLIP_PRE_STABLE_SAMPLES + s_ellip_manual.orient_samples;
            }
            else if (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_SOLVING)
            {
                progress = 100U;
            }

            if ((stage_current > 0U) && (stage_target > 0U))
            {
                progress = (uint32_t)(((uint64_t)(s_ellip_manual.orient_count * 100U) +
                                       ((uint64_t)stage_current * 100U / (uint64_t)stage_target)) /
                                      (uint64_t)ELLIP_MIN_ORIENT);
            }
        }
        else if (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_SOLVING)
        {
            progress = 100U;
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

    if (strcmp(line, "cal ellip start") == 0)
    {
        if (s_imu_calib.busy != 0U)
        {
            printf("cal,busy\r\n");
            return;
        }
        irq_state = interrupt_global_disable();
        imu_calib_start_ellip();
        interrupt_global_enable(irq_state);
        printf("cal,ellip,start\r\n");
        return;
    }

    printf("cal,unknown\r\n");
}

static void imu_calib_print_boot_reminder(void)
{
    printf("cal,remind,cmd,cal status\r\n");
    printf("cal,remind,cmd,cal dump\r\n");
    printf("cal,remind,cmd,cal gyro start\r\n");
    printf("cal,remind,cmd,cal ellip start\r\n");
    printf("cal,remind,cmd,cal load\r\n");
    printf("cal,remind,cmd,cal save\r\n");
    printf("cal,remind,cmd,cal clear\r\n");
}

void AccelCalibration_Init(void)
{
    /* ��ʼ����ȫ����λ */
    AccelCalibration_Reset();
}

/* Reset calibration but keep IMU to body rotation matrix if valid */
void AccelCalibration_Reset(void)
{
    float saved_matrix[3][3];
    bool matrix_valid = false;
    uint8_t i;
    uint8_t j;

    /* ��λǰ�ݴ� IMU ��װ���󣬱��⸴λ��ʧ IMU ��װ���� */
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
    g_accel_calibration.use_full_matrix = 0U;
    set_identity_matrix(g_accel_calibration.accel_corr_matrix);

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
    /* ZYX ŷ����ת��ת����yaw->pitch->roll�� */
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

/* ========================= ��ֹ�궨������ =========================
 * AP ��񣺷ִ��ڲɼ���ֹ����
 * - ÿ�����ڲɼ��̶������ľ�ֹ����
 * - �Ƚϴ��ڼ� bias �仯����ѡ�����Ŵ���
 * - �ﵽ�����������ж��궨�ɹ�
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

    /* ÿ�������궨ǰ��λ��ʷ״̬ */
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

            /* ��ȡһ֡ IMU 1kHz ���� */
            IMU_Update_1000HZ();
            window_tries++;
            total_tries++;
            system_delay_us(ICM42688_SAMPLE_INTERVAL_US);

            imu_calib_get_raw_sensor_sample(accel_sensor_g, gyro_sensor_dps);

            /* ������Ч��ɸѡ */
            if (!imu_sample_valid(accel_sensor_g[0], accel_sensor_g[1], accel_sensor_g[2],
                                  gyro_sensor_dps[0], gyro_sensor_dps[1], gyro_sensor_dps[2]))
            {
                g_accel_calibration.invalid_sample_count++;
                continue;
            }

            rotate_imu_to_body(accel_sensor_g, accel_body_g);
            rotate_imu_to_body(gyro_sensor_dps, gyro_body_dps);

            /* ���ƾ�ֹ����ɸѡ��|a|��1g �ҽ��ٶ�С */
            if (!static_calibration_sample_valid(accel_body_g, gyro_body_dps, &accel_norm_g, &gyro_norm_dps))
            {
                g_accel_calibration.invalid_sample_count++;
                continue;
            }

            get_gravity_body_g(&gx, &gy, &gz);

            /* �ڻ���ϵ�ۼӹ��� bias = measured - static_sign * gravity */
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

                /* �÷�ԽСԽ�ã����ٶȲ���С�����ٶȲ���С��bias С */
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

            /* �������������ﵽ��ֵ����� */
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

    /* δ����ʱ�˻�ʹ�����Ŵ��ڵ� bias */
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
        /* ���� scale ��ֵ��ʹȫ��ƽ��ģ���ӽ� 1g */
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
        /* ���ݱ궨�׶ι۲⵽��ƽ��ģ��΢�������������� */
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
        /* �ж��Ƿ�ͨ����������� + �����㹻 + ���������� */
        const bool quality_ok = (g_accel_calibration.accel_norm_std_g <= ACCEL_CALIBRATION_START_ACCEPT_STD_G);
        const bool enough_samples = (total_valid_samples >= (ACCEL_CALIBRATION_SAMPLES / 2U));
        const bool calibrated = converged || (have_best_window && enough_samples && quality_ok);

        g_accel_calibration.sample_count = (uint16_t)total_valid_samples;
        g_accel_calibration.is_calibrated = calibrated;
        return calibrated;
    }
}

/* 传感器坐标系加速度计校准：在滤波器之前调用，保证滤波器收到的是校准后数据。
 * is_calibrated==false 时直接 return，不干预原始数据（保证校准收集到原始值）。 */
void AccelCalibration_ApplySensorCorrection(float *ax, float *ay, float *az)
{
    if (!g_accel_calibration.is_calibrated)
    {
        return;
    }

    if (g_accel_calibration.use_full_matrix != 0U)
    {
        float centered[3];
        float corrected[3];
        centered[0] = *ax - g_accel_calibration.accel_bias_g[0];
        centered[1] = *ay - g_accel_calibration.accel_bias_g[1];
        centered[2] = *az - g_accel_calibration.accel_bias_g[2];
        mat3_mul_vec(g_accel_calibration.accel_corr_matrix, centered, corrected);
        *ax = corrected[0];
        *ay = corrected[1];
        *az = corrected[2];
    }
    else
    {
        *ax = (*ax - g_accel_calibration.accel_bias_g[0]) * g_accel_calibration.accel_scale[0];
        *ay = (*ay - g_accel_calibration.accel_bias_g[1]) * g_accel_calibration.accel_scale[1];
        *az = (*az - g_accel_calibration.accel_bias_g[2]) * g_accel_calibration.accel_scale[2];
    }
}

/* �����º��������ٶ�У׼�봹ֱ���ٶ�Ԥ������1kHz ��ѭ���е��� */
void AccelCalibration_Update_1000HZ(void)
{
    float accel_sensor_g[3];
    float accel_corrected_sensor_g[3];
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

    /* ===================== ����Ϊʵʱ1kHz���� ===================== */
    sanitize_scale();

    imu_calib_get_raw_sensor_sample(accel_sensor_g, gyro_sensor_dps);

    if (!imu_sample_valid(accel_sensor_g[0], accel_sensor_g[1], accel_sensor_g[2], gyro_sensor_dps[0], gyro_sensor_dps[1], gyro_sensor_dps[2]))
    {
        /* ��Ч�����������������ƽ��˥������ֹͻ�� */
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

    /* ����΢�����£�����ֹ����ʱ��Ч�� */
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

    /* 校准已在 ApplySensorCorrection 前置完成,
       g_imufilter_1000hz 中的 acc 已经是校准后的值,
       rotate_imu_to_body 后的 accel_raw_body_g 即为校准后机体系值 */
    /* 原始传感器值先做零偏/矩阵校准，再旋转到机体系，得到实时校准输出 */
    accel_corrected_sensor_g[0] = accel_sensor_g[0];
    accel_corrected_sensor_g[1] = accel_sensor_g[1];
    accel_corrected_sensor_g[2] = accel_sensor_g[2];
    AccelCalibration_ApplySensorCorrection(&accel_corrected_sensor_g[0],
                                           &accel_corrected_sensor_g[1],
                                           &accel_corrected_sensor_g[2]);
    rotate_imu_to_body(accel_corrected_sensor_g, g_accel_calibration.accel_corrected_body_g);
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

    /* ȥ�������õ���ʵ���Լ��ٶ� */
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

    /* ͶӰ�� Down �Ტͳһ���ţ��� EKF �ã� */
    accel_down_mps2 = ACCEL_DOWN_SIGN_FOR_EKF * calc_accel_down_from_body(accel_body_real_mps2);

    /* ˫·��ͨ��EKF ͨ�������ͨ�� */
    g_accel_calibration.accel_down_for_ekf_mps2 =
        ACC_DOWN_LPF_ALPHA_EKF * accel_down_mps2 +
        (1.0f - ACC_DOWN_LPF_ALPHA_EKF) * g_accel_calibration.accel_down_for_ekf_mps2;

    g_accel_calibration.accel_down_for_output_mps2 =
        ACC_DOWN_LPF_ALPHA_OUTPUT * accel_down_mps2 +
        (1.0f - ACC_DOWN_LPF_ALPHA_OUTPUT) * g_accel_calibration.accel_down_for_output_mps2;

    /* �������ٶȺ�λ�� */
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

void AccelCalibration_RotateImuToBody(const float vec_sensor[3], float vec_body[3])
{
    if ((vec_sensor == NULL) || (vec_body == NULL))
    {
        return;
    }
    rotate_imu_to_body(vec_sensor, vec_body);
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
    g_accel_calibration.use_full_matrix = params->use_full_matrix;
    if (params->use_full_matrix != 0U)
    {
        memcpy(g_accel_calibration.accel_corr_matrix, params->accel_corr_matrix,
               sizeof(g_accel_calibration.accel_corr_matrix));
    }
    else
    {
        /* Diagonal matrix from scale for consistency */
        memset(g_accel_calibration.accel_corr_matrix, 0, sizeof(g_accel_calibration.accel_corr_matrix));
        g_accel_calibration.accel_corr_matrix[0][0] = params->accel_scale[0];
        g_accel_calibration.accel_corr_matrix[1][1] = params->accel_scale[1];
        g_accel_calibration.accel_corr_matrix[2][2] = params->accel_scale[2];
    }
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
    memcpy(params->accel_corr_matrix, g_accel_calibration.accel_corr_matrix, sizeof(params->accel_corr_matrix));
    params->use_full_matrix = g_accel_calibration.use_full_matrix;
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

/*
 * 函数功能: 设置 IMU 校准文本输出回调
 * 输入参数:
 *   sink - 文本输出回调，传入 NULL 表示关闭外部回调
 * 返回值: 无
 */
void IMUCalib_SetTextSink(IMUCalibTextSink_t sink)
{
    s_imu_calib_text_sink = sink;
}

/*
 * 函数功能: 读取 Flash 中保存的 IMU 校准参数
 * 输入参数:
 *   info - 输出的可读化校准信息结构体指针
 * 返回值:
 *   1 - 调用成功，info->valid 指示是否存在有效数据
 *   0 - 输入参数无效
 */
uint8_t IMUCalib_ReadFlashInfo(IMUCalibFlashInfo_t *info)
{
    IMUCalibBlob_t blob;
    const uint32_t words = (uint32_t)((sizeof(IMUCalibBlob_t) + sizeof(uint32_t) - 1U) / sizeof(uint32_t));

    if (NULL == info)
    {
        return 0U;
    }

    memset(info, 0, sizeof(*info));
    memset(&blob, 0, sizeof(blob));
    flash_read_page(0U, IMU_CALIB_FLASH_PAGE, (uint32_t *)&blob, words);

    if (0U == imu_calib_blob_is_valid(&blob))
    {
        info->valid = 0U;
        info->version = (uint16_t)blob.version;
        return 1U;
    }

    info->valid = 1U;
    info->version = (uint16_t)blob.version;
    if (blob.version == IMU_CALIB_FLASH_VERSION_V1)
    {
        const IMUCalibBlobV1_t *v1 = (const IMUCalibBlobV1_t *)&blob;

        memcpy(info->gyro_bias_dps, v1->gyro_bias_dps, sizeof(info->gyro_bias_dps));
        memcpy(info->accel_bias_g, v1->accel_bias_g, sizeof(info->accel_bias_g));
        memset(info->accel_corr_matrix, 0, sizeof(info->accel_corr_matrix));
        info->accel_corr_matrix[0][0] = v1->accel_scale[0];
        info->accel_corr_matrix[1][1] = v1->accel_scale[1];
        info->accel_corr_matrix[2][2] = v1->accel_scale[2];
        memcpy(info->imu_to_body, v1->imu_to_body, sizeof(info->imu_to_body));
        info->use_full_matrix = 0U;
        return 1U;
    }

    memcpy(info->gyro_bias_dps, blob.gyro_bias_dps, sizeof(info->gyro_bias_dps));
    memcpy(info->accel_bias_g, blob.accel_bias_g, sizeof(info->accel_bias_g));
    memcpy(info->accel_corr_matrix, blob.accel_corr_matrix, sizeof(info->accel_corr_matrix));
    memcpy(info->imu_to_body, blob.imu_to_body, sizeof(info->imu_to_body));
    info->use_full_matrix = (fabsf_local(blob.accel_corr_matrix[0][1]) +
                             fabsf_local(blob.accel_corr_matrix[0][2]) +
                             fabsf_local(blob.accel_corr_matrix[1][0]) +
                             fabsf_local(blob.accel_corr_matrix[1][2]) +
                             fabsf_local(blob.accel_corr_matrix[2][0]) +
                             fabsf_local(blob.accel_corr_matrix[2][1]) > 1.0e-6f) ? 1U : 0U;
    return 1U;
}

/* �����º�����IMUУ׼״̬����1kHz ��ѭ���е��� */
uint8_t IMUCalib_StartGyro(void)
{
    uint32_t irq_state;

    if (0U != s_imu_calib.busy)
    {
        return 0U;
    }

    irq_state = interrupt_global_disable();
    if (0U == s_imu_calib.busy)
    {
        imu_calib_start_gyro();
        interrupt_global_enable(irq_state);
        return 1U;
    }

    interrupt_global_enable(irq_state);
    return 0U;
}

/*
 * 函数功能: 启动加速度椭球校准流程
 * 输入参数: 无
 * 返回值:
 *   1 - 启动成功
 *   0 - 当前忙，启动失败
 */
uint8_t IMUCalib_StartAccel(void)
{
    uint32_t irq_state;

    if (0U != s_imu_calib.busy)
    {
        return 0U;
    }

    irq_state = interrupt_global_disable();
    if (0U == s_imu_calib.busy)
    {
        imu_calib_start_ellip();
        interrupt_global_enable(irq_state);
        return 1U;
    }

    interrupt_global_enable(irq_state);
    return 0U;
}

uint8_t IMUCalib_StartAccelManual(void)
{
    uint32_t irq_state;

    if (0U != s_imu_calib.busy)
    {
        return 0U;
    }

    irq_state = interrupt_global_disable();
    if (0U == s_imu_calib.busy)
    {
        imu_calib_start_ellip_manual();
        interrupt_global_enable(irq_state);
        return 1U;
    }

    interrupt_global_enable(irq_state);
    return 0U;
}

uint8_t IMUCalib_ManualCollect(void)
{
    uint32_t irq_state;

    if ((0U == s_imu_calib.busy) || (s_imu_calib.mode != IMU_CALIB_MODE_ELLIP_MANUAL))
    {
        return 0U;
    }

    irq_state = interrupt_global_disable();
    if ((s_imu_calib.busy != 0U) &&
        (s_imu_calib.mode == IMU_CALIB_MODE_ELLIP_MANUAL) &&
        (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_READY) &&
        (s_ellip_manual.orient_count < ELLIP_MANUAL_MAX_ORIENT))
    {
        imu_calib_manual_begin_collect();
        interrupt_global_enable(irq_state);
        return 1U;
    }

    interrupt_global_enable(irq_state);
    return 0U;
}

uint8_t IMUCalib_ManualStop(void)
{
    uint32_t irq_state;

    if ((0U == s_imu_calib.busy) || (s_imu_calib.mode != IMU_CALIB_MODE_ELLIP_MANUAL))
    {
        return 0U;
    }

    irq_state = interrupt_global_disable();
    if ((s_imu_calib.busy != 0U) &&
        (s_imu_calib.mode == IMU_CALIB_MODE_ELLIP_MANUAL) &&
        (s_ellip_manual.substate != IMU_CALIB_MANUAL_SUBSTATE_SOLVING))
    {
        if (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_COLLECTING)
        {
            imu_calib_emit_text("OK imu accel_man 提示 当前未完成姿态点已丢弃");
        }

        imu_calib_manual_reset_current_pose();
        s_ellip_manual.substate = IMU_CALIB_MANUAL_SUBSTATE_SOLVING;
        imu_calib_emit_text("OK imu accel_man 开始求解 pose_count=%u",
                            (unsigned int)s_ellip_manual.orient_count);
        interrupt_global_enable(irq_state);
        return 1U;
    }

    interrupt_global_enable(irq_state);
    return 0U;
}

void IMUCalib_GetStatus(IMUCalibStatus_t *status)
{
    if (NULL == status)
    {
        return;
    }

    status->busy = s_imu_calib.busy;
    status->mode = s_imu_calib.mode;
    status->pose_count = (s_imu_calib.mode == IMU_CALIB_MODE_ELLIP_MANUAL) ? s_ellip_manual.orient_count : s_ellip.orient_count;
    status->substate = (s_imu_calib.mode == IMU_CALIB_MODE_ELLIP_MANUAL) ? s_ellip_manual.substate : IMU_CALIB_MANUAL_SUBSTATE_NONE;
    status->progress_percent = imu_calib_progress_percent();
    status->current_samples = 0U;
    status->target_samples = 0U;

    if (s_imu_calib.mode == IMU_CALIB_MODE_ELLIP_MANUAL)
    {
        if (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_WAIT_STATIC)
        {
            status->current_samples = s_ellip_manual.stable_samples;
            status->target_samples = ELLIP_PRE_STABLE_SAMPLES;
        }
        else if (s_ellip_manual.substate == IMU_CALIB_MANUAL_SUBSTATE_COLLECTING)
        {
            status->current_samples = s_ellip_manual.orient_samples;
            status->target_samples = ELLIP_MANUAL_ORIENT_SAMPLES;
        }
    }
}

void IMUCalib_Update_1000HZ(void)
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

    if (s_imu_calib.mode == IMU_CALIB_MODE_ELLIP)
    {
        ret = imu_calib_update_ellip_step();
        if (ret > 0)
        {
            s_imu_calib.busy = 0U;
            s_imu_calib.mode = IMU_CALIB_MODE_IDLE;
        }
        else if (ret < 0)
        {
            s_imu_calib.busy = 0U;
            s_imu_calib.mode = IMU_CALIB_MODE_IDLE;
        }
        return;
    }

    if (s_imu_calib.mode == IMU_CALIB_MODE_ELLIP_MANUAL)
    {
        ret = imu_calib_update_ellip_manual_step();
        if (ret > 0)
        {
            s_imu_calib.busy = 0U;
            s_imu_calib.mode = IMU_CALIB_MODE_IDLE;
        }
        else if (ret < 0)
        {
            s_imu_calib.busy = 0U;
            s_imu_calib.mode = IMU_CALIB_MODE_IDLE;
        }
        return;
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
