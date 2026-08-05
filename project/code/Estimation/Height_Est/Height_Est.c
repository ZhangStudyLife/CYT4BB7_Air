#include "Height_Est.h"
#include "../Attitude/Accel_Calibration.h"
#include "../Attitude/IMU_TOP.h"
#include <math.h>

extern volatile uint32 tick_1000us_cnt; /* 1ms系统墙钟，用于判断ToF数据真实年龄 */

float g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
float g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
float g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
float g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
float g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
uint8 g_tof_fused_valid = 0U;
float g_height_fused_vz_mps = 0.0f;
float g_height_meas_health = 0.0f;
float g_height_acc_up_mps2 = 0.0f;

#define HEIGHT_PREDICT_DT_S         0.001f
#define HEIGHT_TOF_MIN_MM           50.0f
#define HEIGHT_TOF_MAX_MM           2200.0f
#define HEIGHT_STATE_MAX_MM         2200.0f
#define HEIGHT_MIN_MM               50.0f
#define HEIGHT_TILT_DEG_LIMIT       50.0f
#define HEIGHT_MEDIAN_GATE_MIN_MM   90.0f
#define HEIGHT_MEDIAN_GATE_MAX_MM   170.0f
#define HEIGHT_RATE_BAD_MM          55.0f
#define HEIGHT_LOW_DROP_MM          140.0f
#define HEIGHT_LOW_OUTLIER_MIN_MM   120.0f
#define HEIGHT_LOW_OUTLIER_MAX_MM   320.0f
#define HEIGHT_CONF_MIN             0.12f
#define HEIGHT_MISS_MAX             300U
#define HEIGHT_TOF_TIMEOUT_MS       (HEIGHT_MISS_MAX * 10U) /* ToF连续无有效测量超时，单位ms */
#define HEIGHT_TOF_ALPHA            0.18f
#define HEIGHT_TOF_ALPHA_STRONG     0.32f
#define HEIGHT_TOF_ALPHA_MEDIUM     0.18f
#define HEIGHT_TOF_ALPHA_WEAK       0.06f
#define HEIGHT_VEL_TOF_ALPHA        0.18f
#define HEIGHT_VEL_LPF              0.018f
#define HEIGHT_ACC_LPF              0.08f
#define HEIGHT_ACC_LIMIT_MPS2       5.0f
#define HEIGHT_ACC_VEL_GAIN         0.35f
#define HEIGHT_TRUSTED_TOF_CAP_M    0.015f
#define HEIGHT_SOFT_TOF_CAP_M       0.080f
#define HEIGHT_SOFT_TOF_CAP_GAIN    0.25f
#define HEIGHT_VEL_LEAK             0.9990f
#define HEIGHT_NO_MEAS_VEL_LEAK     0.9960f
#define HEIGHT_DEG_TO_RAD           0.017453293f
#define HEIGHT_HISTORY_GATE_MIN_MM  120.0f
#define HEIGHT_HISTORY_GATE_MAX_MM  260.0f

static float s_h_m = 0.0f;
static float s_v_mps = 0.0f;
static float s_v_lpf_mps = 0.0f;
static float s_acc_lpf_mps2 = 0.0f;
static float s_prev_tof_mm[VL53L1X_SENSOR_COUNT];
static uint8 s_ready = 0U;
static uint16 s_miss = HEIGHT_MISS_MAX;
static uint32 s_last_tof_sample_seq = 0U; /* 高度融合最近一次消费的ToF快照序号 */
static uint32 s_last_valid_measurement_tick_ms = 0U; /* 最近一次有效ToF测量的墙钟，单位ms */
static uint8 s_have_valid_measurement = 0U; /* 是否已经接收过有效ToF测量 */

static const float s_roll_corr_m[VL53L1X_SENSOR_COUNT] = {
    -0.008643f, 0.089082f, -0.058806f, 0.032884f
};
static const float s_pitch_corr_m[VL53L1X_SENSOR_COUNT] = {
    -0.012797f, -0.006581f, 0.088272f, 0.003979f
};
static const float s_bias_corr_m[VL53L1X_SENSOR_COUNT] = {
    -0.002834f, -0.016172f, 0.022442f, -0.003449f
};

static float Height_Clamp(float x, float min_x, float max_x)
{
    if (x < min_x)
    {
        return min_x;
    }
    if (x > max_x)
    {
        return max_x;
    }
    return x;
}

static void Height_Sort(float *v, uint8 n)
{
    uint8 i;
    uint8 j;
    float t;

    for (i = 0U; i < n; i++)
    {
        for (j = i + 1U; j < n; j++)
        {
            if (v[j] < v[i])
            {
                t = v[i];
                v[i] = v[j];
                v[j] = t;
            }
        }
    }
}

static float Height_Median(float *v, uint8 n)
{
    Height_Sort(v, n);
    return (0U != (n & 1U)) ? v[n >> 1U] : (0.5f * (v[(n >> 1U) - 1U] + v[n >> 1U]));
}

static uint8 Height_RawValid(const VL53L1X_data_struct *tof, uint8 i)
{
    float d = (float)tof->distance_mm[i];
    return (((tof->fresh_mask & (uint8)(1U << i)) != 0U) &&
        (0U != tof->valid[i]) && (d >= HEIGHT_TOF_MIN_MM) &&
        (d <= HEIGHT_TOF_MAX_MM) && (tof->distance_mm[i] < VL53L1X_INVALID_DISTANCE_MM)) ? 1U : 0U;
}

static float Height_ChannelMm(const VL53L1X_data_struct *tof, uint8 i, float roll_rad, float pitch_rad)
{
    float h = (float)tof->distance_mm[i] * g_euler.cos_roll * g_euler.cos_pitch;
    h -= 1000.0f * ((s_roll_corr_m[i] * roll_rad) + (s_pitch_corr_m[i] * pitch_rad) + s_bias_corr_m[i]);
    return Height_Clamp(h, HEIGHT_MIN_MM, HEIGHT_STATE_MAX_MM);
}

static uint8 Height_BuildMeasure(const VL53L1X_data_struct *tof, float *meas_mm, float *health, uint8 *inlier_count)
{
    float h[VL53L1X_SENSOR_COUNT];
    float s[VL53L1X_SENSOR_COUNT];
    float roll_rad = g_euler.roll * HEIGHT_DEG_TO_RAD;
    float pitch_rad = g_euler.pitch * HEIGHT_DEG_TO_RAD;
    float tilt_deg = fabsf(g_euler.roll) + fabsf(g_euler.pitch);
    float center;
    float gate;
    float inlier_min = (float)VL53L1X_INVALID_DISTANCE_MM;
    float inlier_max = 0.0f;
    float inlier_spread = 0.0f;
    float sum_h = 0.0f;
    float sum_w = 0.0f;
    float hist_sum = 0.0f;
    float hist_min = (float)VL53L1X_INVALID_DISTANCE_MM;
    float hist_max = 0.0f;
    float hist_gate = HEIGHT_HISTORY_GATE_MIN_MM;
    float hist_ref_mm = s_h_m * 1000.0f;
    float best_sum = 0.0f;
    float best_min = (float)VL53L1X_INVALID_DISTANCE_MM;
    float best_max = 0.0f;
    float best_spread;
    uint8 best_n = 0U;
    uint8 hist_n = 0U;
    uint8 n = 0U;
    uint8 inlier_n = 0U;
    uint8 i;
    uint8 j;

    for (i = 0U; i < VL53L1X_SENSOR_COUNT; i++)
    {
        h[i] = (float)VL53L1X_INVALID_DISTANCE_MM;
        if ((0 != tof) && (0U != Height_RawValid(tof, i)) && (tilt_deg < HEIGHT_TILT_DEG_LIMIT))
        {
            h[i] = Height_ChannelMm(tof, i, roll_rad, pitch_rad);
            s[n++] = h[i];
        }
        else if ((0 != tof) && ((tof->fresh_mask & (uint8)(1U << i)) != 0U))
        {
            s_prev_tof_mm[i] = -1.0f;
        }
    }

    g_tof1_height_mm = h[0];
    g_tof2_height_mm = h[1];
    g_tof3_height_mm = h[2];
    g_tof4_height_mm = h[3];
    *health = 0.0f;
    *inlier_count = 0U;

    if (n < 2U)
    {
        return 0U;
    }

    center = Height_Median(s, n);
    gate = Height_Clamp(0.10f * center, HEIGHT_MEDIAN_GATE_MIN_MM, HEIGHT_MEDIAN_GATE_MAX_MM);

    if (0U != s_ready)
    {
        hist_gate = Height_Clamp(0.15f * hist_ref_mm, HEIGHT_HISTORY_GATE_MIN_MM, HEIGHT_HISTORY_GATE_MAX_MM);
    }

    for (i = 0U; i < n; i++)
    {
        float local_sum = 0.0f;
        float local_min = (float)VL53L1X_INVALID_DISTANCE_MM;
        float local_max = 0.0f;
        uint8 local_n = 0U;

        for (j = 0U; j < n; j++)
        {
            if (fabsf(s[j] - s[i]) <= gate)
            {
                local_sum += s[j];
                if (s[j] < local_min)
                {
                    local_min = s[j];
                }
                if (s[j] > local_max)
                {
                    local_max = s[j];
                }
                local_n++;
            }
        }

        if ((local_n > best_n) ||
            ((local_n == best_n) && ((local_max - local_min) < (best_max - best_min))))
        {
            best_n = local_n;
            best_sum = local_sum;
            best_min = local_min;
            best_max = local_max;
        }
    }

    if (0U != s_ready)
    {
        for (i = 0U; i < n; i++)
        {
            if (fabsf(s[i] - hist_ref_mm) <= hist_gate)
            {
                hist_sum += s[i];
                if (s[i] < hist_min)
                {
                    hist_min = s[i];
                }
                if (s[i] > hist_max)
                {
                    hist_max = s[i];
                }
                hist_n++;
            }
        }
    }

    if ((best_n < 3U) && (hist_n > 0U))
    {
        best_n = hist_n;
        best_sum = hist_sum;
        best_min = hist_min;
        best_max = hist_max;
    }

    if (best_n > 0U)
    {
        best_spread = best_max - best_min;
        if ((best_n >= 2U) || (0U != s_ready))
        {
            *meas_mm = best_sum / (float)best_n;
            *inlier_count = best_n;
            if (best_n >= 3U)
            {
                *health = Height_Clamp(((float)best_n / 3.0f) * Height_Clamp(1.0f - best_spread / 260.0f, 0.20f, 1.0f), 0.0f, 1.0f);
            }
            else if (2U == best_n)
            {
                *health = Height_Clamp(0.55f * Height_Clamp(1.0f - best_spread / 260.0f, 0.20f, 1.0f), 0.12f, 0.65f);
            }
            else
            {
                *health = 0.16f;
            }
            return 1U;
        }
    }

    for (i = 0U; i < VL53L1X_SENSOR_COUNT; i++)
    {
        if (h[i] < (float)VL53L1X_INVALID_DISTANCE_MM)
        {
            float r = h[i] - center;
            float a = fabsf(r);
            float w = 1.0f / (1.0f + (a / gate) * (a / gate));
            if (a > gate)
            {
                w *= 0.20f;
            }
            else
            {
                if (h[i] < inlier_min)
                {
                    inlier_min = h[i];
                }
                if (h[i] > inlier_max)
                {
                    inlier_max = h[i];
                }
                inlier_n++;
            }
            if ((s_prev_tof_mm[i] > 0.0f) && (fabsf(h[i] - s_prev_tof_mm[i]) > HEIGHT_RATE_BAD_MM))
            {
                w *= 0.50f;
            }
            if ((s_prev_tof_mm[i] > 0.0f) && ((h[i] - s_prev_tof_mm[i]) < -HEIGHT_LOW_DROP_MM) &&
                (h[i] < (center - Height_Clamp(0.25f * center, HEIGHT_LOW_OUTLIER_MIN_MM, HEIGHT_LOW_OUTLIER_MAX_MM))))
            {
                w *= 0.05f;
            }
            sum_h += w * h[i];
            sum_w += w;
            s_prev_tof_mm[i] = h[i];
        }
    }

    if (sum_w < 0.70f)
    {
        return 0U;
    }

    *meas_mm = sum_h / sum_w;
    if (inlier_n >= 2U)
    {
        inlier_spread = inlier_max - inlier_min;
        *health = Height_Clamp((sum_w / 3.0f) * Height_Clamp(1.0f - inlier_spread / 260.0f, 0.10f, 1.0f), 0.0f, 1.0f);
        *inlier_count = inlier_n;
    }
    else
    {
        *health = Height_Clamp(sum_w / 4.0f, 0.0f, 0.20f);
    }
    return 1U;
}

static void Height_CorrectObserver(float meas_mm, float health, uint8 meas_valid, uint8 inlier_count)
{
    float residual_m = 0.0f;
    float old_h_m;
    float inst_v_mps;
    float measurement_dt_s;
    float tof_alpha;
    uint32 now_tick_ms = tick_1000us_cnt;

    g_height_meas_health = health;

    if (0U == s_ready)
    {
        if ((0U == meas_valid) || (health < HEIGHT_CONF_MIN))
        {
            g_tof_fused_valid = 0U;
            return;
        }
        s_h_m = meas_mm * 0.001f;
        s_v_mps = 0.0f;
        s_v_lpf_mps = 0.0f;
        s_ready = 1U;
        s_miss = 0U;
    }

    if ((0U != meas_valid) && (health >= HEIGHT_CONF_MIN))
    {
        if (0U != s_have_valid_measurement)
        {
            measurement_dt_s = (float)(now_tick_ms - s_last_valid_measurement_tick_ms) * 0.001f;
            measurement_dt_s = Height_Clamp(measurement_dt_s, 0.001f, 3.0f);
        }
        else
        {
            measurement_dt_s = 0.01f;
        }
        old_h_m = s_h_m;
        residual_m = (meas_mm * 0.001f) - s_h_m;
        residual_m = Height_Clamp(residual_m, -0.45f, 0.45f);
        if (inlier_count >= 3U)
        {
            tof_alpha = HEIGHT_TOF_ALPHA_STRONG * Height_Clamp(health, 0.40f, 1.0f);
        }
        else if (2U == inlier_count)
        {
            tof_alpha = HEIGHT_TOF_ALPHA_MEDIUM * Height_Clamp(health, 0.20f, 0.70f);
        }
        else
        {
            tof_alpha = HEIGHT_TOF_ALPHA_WEAK * Height_Clamp(health, 0.10f, 0.30f);
        }
        s_h_m += tof_alpha * residual_m;

        if ((inlier_count >= 3U) && (health > 0.45f) && (s_h_m > ((meas_mm * 0.001f) + HEIGHT_TRUSTED_TOF_CAP_M)))
        {
            s_h_m = (meas_mm * 0.001f) + HEIGHT_TRUSTED_TOF_CAP_M;
            if (s_v_mps > 0.0f)
            {
                s_v_mps *= 0.20f;
            }
        }
        else if ((2U == inlier_count) && (health > 0.35f) && (s_h_m > ((meas_mm * 0.001f) + HEIGHT_SOFT_TOF_CAP_M)))
        {
            s_h_m += HEIGHT_SOFT_TOF_CAP_GAIN * (((meas_mm * 0.001f) + HEIGHT_SOFT_TOF_CAP_M) - s_h_m);
            if (s_v_mps > 0.0f)
            {
                s_v_mps *= 0.50f;
            }
        }

        inst_v_mps = (s_h_m - old_h_m) / measurement_dt_s;
        s_v_mps += HEIGHT_VEL_TOF_ALPHA * (inst_v_mps - s_v_mps);
        s_miss = 0U;
        s_last_valid_measurement_tick_ms = now_tick_ms;
        s_have_valid_measurement = 1U;
    }
    else if (s_miss < HEIGHT_MISS_MAX)
    {
        s_miss++;
        s_v_mps *= HEIGHT_NO_MEAS_VEL_LEAK;
    }

    s_h_m = Height_Clamp(s_h_m, HEIGHT_MIN_MM * 0.001f, HEIGHT_STATE_MAX_MM * 0.001f);
    s_v_mps = Height_Clamp(s_v_mps, -3.0f, 3.0f);

    g_tof_fused_height_mm = s_h_m * 1000.0f;
    g_tof_fused_valid = ((0U != s_ready) &&
                         (s_miss < HEIGHT_MISS_MAX) &&
                         ((0U == s_have_valid_measurement) ||
                          ((now_tick_ms - s_last_valid_measurement_tick_ms) < HEIGHT_TOF_TIMEOUT_MS))) ? 1U : 0U;
}

static void Height_Reset(void)
{
    uint8 i;

    for (i = 0U; i < VL53L1X_SENSOR_COUNT; i++)
    {
        s_prev_tof_mm[i] = -1.0f;
    }

    s_h_m = 0.0f;
    s_v_mps = 0.0f;
    s_v_lpf_mps = 0.0f;
    s_acc_lpf_mps2 = 0.0f;
    s_ready = 0U;
    s_miss = HEIGHT_MISS_MAX;
    s_last_tof_sample_seq = 0U;
    s_last_valid_measurement_tick_ms = 0U;
    s_have_valid_measurement = 0U;
    g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_valid = 0U;
    g_height_fused_vz_mps = 0.0f;
    g_height_meas_health = 0.0f;
    g_height_acc_up_mps2 = 0.0f;
}

void TOF_Init(void)
{
    Height_Reset();
    VL53L1X_Init();
}

void Height_Est_predict_1000HZ(void)
{
    float acc;

    if ((0U != s_have_valid_measurement) &&
        ((tick_1000us_cnt - s_last_valid_measurement_tick_ms) >= HEIGHT_TOF_TIMEOUT_MS))
    {
        g_tof_fused_valid = 0U;
        g_height_meas_health = 0.0f;
    }

    g_height_acc_up_mps2 = AccelCalibration_GetVerticalAccelUpMps2();
    acc = Height_Clamp(g_height_acc_up_mps2, -HEIGHT_ACC_LIMIT_MPS2, HEIGHT_ACC_LIMIT_MPS2);
    s_acc_lpf_mps2 += HEIGHT_ACC_LPF * (acc - s_acc_lpf_mps2);

    if (0U != s_ready)
    {
        s_v_mps += HEIGHT_ACC_VEL_GAIN * s_acc_lpf_mps2 * HEIGHT_PREDICT_DT_S;
        s_v_mps *= HEIGHT_VEL_LEAK;

        s_h_m = Height_Clamp(s_h_m, HEIGHT_MIN_MM * 0.001f, HEIGHT_STATE_MAX_MM * 0.001f);
        s_v_mps = Height_Clamp(s_v_mps, -3.0f, 3.0f);
        s_v_lpf_mps += HEIGHT_VEL_LPF * (s_v_mps - s_v_lpf_mps);

        g_tof_fused_height_mm = s_h_m * 1000.0f;
        g_height_fused_vz_mps = s_v_lpf_mps;
    }
}

void Height_Est_update_100HZ(void)
{
    const VL53L1X_data_struct *tof;
    float meas_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float health = 0.0f;
    uint8 inlier_count = 0U;
    uint8 meas_valid;

    tof = VL53L1X_GetData();
    if ((0 == tof) || (tof->sample_seq == s_last_tof_sample_seq))
    {
        Height_CorrectObserver(meas_mm, health, 0U, inlier_count);
        return;
    }

    s_last_tof_sample_seq = tof->sample_seq;
    meas_valid = Height_BuildMeasure(tof, &meas_mm, &health, &inlier_count);
    Height_CorrectObserver(meas_mm, health, meas_valid, inlier_count);
}
