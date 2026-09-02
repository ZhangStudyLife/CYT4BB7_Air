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
#include "Height_Est.h"
#include "../Attitude/Accel_Calibration.h"
#include "../Attitude/IMU_TOP.h"
#include <math.h>

extern volatile uint32 tick_1000us_cnt;

#define HEIGHT_PREDICT_DT_S                    0.001f
#define HEIGHT_TOF_MIN_MM                      50.0f
#define HEIGHT_TOF_MAX_MM                      2200.0f
#define HEIGHT_STATE_MAX_MM                    2200.0f
#define HEIGHT_MIN_MM                          50.0f
#define HEIGHT_TILT_DEG_LIMIT                  50.0f
#define HEIGHT_CONF_MIN                        0.12f
#define HEIGHT_MISS_MAX                        300U
#define HEIGHT_NO_ASCENT_TIMEOUT_MS            100U
#define HEIGHT_CONTROLLED_DESCENT_TIMEOUT_MS   500U
#define HEIGHT_CONSENSUS_GATE_MM               120.0f
#define HEIGHT_NORMAL_ALPHA                    0.80f
#define HEIGHT_NORMAL_BETA                     0.10f
#define HEIGHT_MEAS_RESIDUAL_LIMIT_M           0.18f
#define HEIGHT_SAFE_LATCH_MS                   100U
#define HEIGHT_SAFE_MARGIN_MM                  25.0f
#define HEIGHT_SAFE_RESPONSE_S                 0.08f
#define HEIGHT_SAFE_BRAKE_ACCEL_MPS2           1.80f
#define HEIGHT_SAFE_SINGLE_HIGH_MM             1250.0f
#define HEIGHT_POLLUTION_DROP_MM               160.0f
#define HEIGHT_POLLUTION_MAX_MM                850.0f
#define HEIGHT_POLLUTION_CLEAR_FRAMES          8U
#define HEIGHT_ACC_LPF                         0.08f
#define HEIGHT_ACC_LIMIT_MPS2                  5.0f
#define HEIGHT_ACC_VEL_GAIN                    0.10f
#define HEIGHT_VEL_LEAK                        0.9990f
#define HEIGHT_NO_MEAS_VEL_LEAK                0.9900f
#define HEIGHT_DEG_TO_RAD                      0.017453293f

float g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
float g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
float g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
float g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
float g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
uint8 g_tof_fused_valid = 0U;
float g_height_fused_vz_mps = 0.0f;
float g_height_meas_health = 0.0f;
float g_height_acc_up_mps2 = 0.0f;
uint32 g_height_tof_sample_seq = 0U;
uint8 g_height_tof_fresh_mask = 0U;
uint8 g_height_tof_valid_mask = 0U;
uint8 g_height_meas_valid = 0U;
uint8 g_height_inlier_count = 0U;
float g_height_meas_mm = (float)VL53L1X_VALID_RANGE_MAX;
float g_height_residual_m = 0.0f;
float g_height_measurement_dt_s = 0.0f;
float g_height_inst_v_mps = 0.0f;
float g_height_acc_lpf_mps2 = 0.0f;
float g_height_observer_v_mps = 0.0f;
uint16 g_height_miss_count = HEIGHT_MISS_MAX;
float g_height_measurement_age_ms = 0.0f;
float g_height_tof_alpha = 0.0f;
float g_height_safe_upper_mm = (float)VL53L1X_VALID_RANGE_MAX;
float g_height_safe_lower_mm = (float)VL53L1X_VALID_RANGE_MAX;
float g_height_safe_brake_distance_mm = 0.0f;
uint8 g_height_dropout_mode = HEIGHT_DROPOUT_CONTROLLED_DESCENT;
uint8 g_height_pollution_active = 0U;
uint8 g_height_safety_valid = 0U;

static float s_h_m = 0.0f;
static float s_v_mps = 0.0f;
static float s_acc_lpf_mps2 = 0.0f;
static uint8 s_ready = 0U;
static uint16 s_miss = HEIGHT_MISS_MAX;
static uint32 s_last_tof_sample_seq = 0U;
static uint32 s_last_valid_measurement_tick_ms = 0U;
static uint8 s_have_valid_measurement = 0U;
static float s_last_safe_high_m = 0.0f;
static float s_last_safe_low_m = 0.0f;
static uint32 s_last_safe_high_tick_ms = 0U;
static uint32 s_last_safe_low_tick_ms = 0U;
static uint8 s_pollution_latched = 0U;
static uint8 s_pollution_clear_count = 0U;

static const float s_consensus_corr_mm[VL53L1X_SENSOR_COUNT] = {
    60.0f, -3.0f, -54.0f, -4.0f
};

static const float s_roll_corr_m[VL53L1X_SENSOR_COUNT] = {
    0.022948f, 0.090888f, 0.004973f, -0.118808f
};

static const float s_pitch_corr_m[VL53L1X_SENSOR_COUNT] = {
    -0.132881f, -0.048531f, 0.129657f, 0.051755f
};

static const float s_bias_corr_m[VL53L1X_SENSOR_COUNT] = {
    0.030566f, -0.008795f, -0.019068f, -0.002704f
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

static uint8 Height_RawValid(const VL53L1X_data_struct *tof, uint8 i)
{
    float d = (float)tof->distance_mm[i];

    return (((tof->fresh_mask & (uint8)(1U << i)) != 0U) &&
            (tof->valid[i] != 0U) &&
            (d >= HEIGHT_TOF_MIN_MM) &&
            (d <= HEIGHT_TOF_MAX_MM) &&
            (tof->distance_mm[i] < VL53L1X_INVALID_DISTANCE_MM)) ? 1U : 0U;
}

static float Height_ChannelMm(const VL53L1X_data_struct *tof,
                              uint8 i,
                              float roll_rad,
                              float pitch_rad)
{
    float h = (float)tof->distance_mm[i] * g_euler.cos_roll * g_euler.cos_pitch;

    h -= 1000.0f * ((s_roll_corr_m[i] * roll_rad) +
                    (s_pitch_corr_m[i] * pitch_rad) + s_bias_corr_m[i]);
    h += s_consensus_corr_mm[i];
    return Height_Clamp(h, HEIGHT_MIN_MM, HEIGHT_STATE_MAX_MM);
}

static uint8 Height_BuildMeasure(const VL53L1X_data_struct *tof,
                                 float *meas_mm,
                                 float *health,
                                 uint8 *inlier_count,
                                 float *safe_high_mm,
                                 float *safe_low_mm,
                                 uint8 *safe_mask,
                                 uint8 *pollution_detected)
{
    float h[VL53L1X_SENSOR_COUNT];
    float roll_rad = g_euler.roll * HEIGHT_DEG_TO_RAD;
    float pitch_rad = g_euler.pitch * HEIGHT_DEG_TO_RAD;
    float tilt_deg = fabsf(g_euler.roll) + fabsf(g_euler.pitch);
    float max_valid_mm = 0.0f;
    float best_mean = 0.0f;
    float best_min = (float)VL53L1X_INVALID_DISTANCE_MM;
    float best_max = 0.0f;
    float best_spread = (float)VL53L1X_INVALID_DISTANCE_MM;
    uint8 best_n = 0U;
    uint8 valid_n = 0U;
    uint8 i;
    uint8 j;

    for (i = 0U; i < VL53L1X_SENSOR_COUNT; i++)
    {
        h[i] = (float)VL53L1X_INVALID_DISTANCE_MM;
        if ((tof != 0) && (Height_RawValid(tof, i) != 0U) &&
            (tilt_deg < HEIGHT_TILT_DEG_LIMIT))
        {
            h[i] = Height_ChannelMm(tof, i, roll_rad, pitch_rad);
            if (h[i] > max_valid_mm)
            {
                max_valid_mm = h[i];
            }
            valid_n++;
        }
    }

    g_tof1_height_mm = h[0];
    g_tof2_height_mm = h[1];
    g_tof3_height_mm = h[2];
    g_tof4_height_mm = h[3];
    *health = 0.0f;
    *inlier_count = 0U;
    *safe_high_mm = (float)VL53L1X_VALID_RANGE_MAX;
    *safe_low_mm = (float)VL53L1X_VALID_RANGE_MAX;
    *safe_mask = 0U;
    *pollution_detected = 0U;

    if (valid_n == 0U)
    {
        return 0U;
    }

    for (i = 0U; i < VL53L1X_SENSOR_COUNT; i++)
    {
        float local_sum = 0.0f;
        float local_min = (float)VL53L1X_INVALID_DISTANCE_MM;
        float local_max = 0.0f;
        float local_mean;
        float local_spread;
        uint8 local_n = 0U;

        if (h[i] >= (float)VL53L1X_INVALID_DISTANCE_MM)
        {
            continue;
        }

        for (j = 0U; j < VL53L1X_SENSOR_COUNT; j++)
        {
            if ((h[j] < (float)VL53L1X_INVALID_DISTANCE_MM) &&
                (fabsf(h[j] - h[i]) <= HEIGHT_CONSENSUS_GATE_MM))
            {
                local_sum += h[j];
                if (h[j] < local_min)
                {
                    local_min = h[j];
                }
                if (h[j] > local_max)
                {
                    local_max = h[j];
                }
                local_n++;
            }
        }

        local_spread = local_max - local_min;
        if (local_spread > HEIGHT_CONSENSUS_GATE_MM)
        {
            continue;
        }
        local_mean = local_sum / (float)local_n;
        if ((local_n > best_n) ||
            ((local_n == best_n) && (local_mean > best_mean)) ||
            ((local_n == best_n) && (local_mean == best_mean) &&
             (local_spread < best_spread)))
        {
            best_n = local_n;
            best_mean = local_mean;
            best_min = local_min;
            best_max = local_max;
            best_spread = local_spread;
        }
    }

    if (best_n >= 2U)
    {
        if ((s_ready != 0U) &&
            (best_mean < (s_h_m * 1000.0f - HEIGHT_POLLUTION_DROP_MM)) &&
            (best_mean < HEIGHT_POLLUTION_MAX_MM))
        {
            *pollution_detected = 1U;
            if ((s_v_mps > -0.15f) && (g_height_acc_up_mps2 > -0.50f))
            {
                if (max_valid_mm >= HEIGHT_SAFE_SINGLE_HIGH_MM)
                {
                    *safe_high_mm = max_valid_mm;
                    *safe_mask = 0x01U;
                }
                return 0U;
            }
        }
        *meas_mm = best_mean;
        *inlier_count = best_n;
        *safe_high_mm = (max_valid_mm > best_max) ? max_valid_mm : best_max;
        *safe_low_mm = best_min;
        *safe_mask = 0x03U;
        if (best_n >= 3U)
        {
            *health = Height_Clamp(((float)best_n / 4.0f) *
                                   (1.0f - best_spread / 260.0f),
                                   0.35f, 1.0f);
        }
        else
        {
            *health = Height_Clamp(0.55f * (1.0f - best_spread / 260.0f),
                                   0.25f, 0.55f);
        }

        for (i = 0U; i < VL53L1X_SENSOR_COUNT; i++)
        {
            if ((h[i] < HEIGHT_POLLUTION_MAX_MM) &&
                (h[i] < (best_mean - HEIGHT_POLLUTION_DROP_MM)))
            {
                *pollution_detected = 1U;
            }
        }
        return 1U;
    }

    if (max_valid_mm >= HEIGHT_SAFE_SINGLE_HIGH_MM)
    {
        *safe_high_mm = max_valid_mm;
        *safe_mask = 0x01U;
    }
    return 0U;
}

static void Height_UpdateSafety(void)
{
    uint32 now_tick_ms = tick_1000us_cnt;
    uint32 age_ms = HEIGHT_CONTROLLED_DESCENT_TIMEOUT_MS + 1U;
    float base_high_m;
    float base_low_m;
    float up_v_mps;
    float down_v_mps;
    float up_brake_m;
    float down_brake_m;

    if (s_have_valid_measurement != 0U)
    {
        age_ms = now_tick_ms - s_last_valid_measurement_tick_ms;
    }

    if (age_ms > HEIGHT_CONTROLLED_DESCENT_TIMEOUT_MS)
    {
        g_height_dropout_mode = HEIGHT_DROPOUT_CONTROLLED_DESCENT;
    }
    else if (age_ms > HEIGHT_NO_ASCENT_TIMEOUT_MS)
    {
        g_height_dropout_mode = HEIGHT_DROPOUT_NO_ASCENT;
    }
    else
    {
        g_height_dropout_mode = HEIGHT_DROPOUT_NONE;
    }

    g_height_measurement_age_ms = (s_have_valid_measurement != 0U) ?
                                  (float)age_ms : 0.0f;
    g_tof_fused_valid = ((s_ready != 0U) &&
                         (g_height_dropout_mode == HEIGHT_DROPOUT_NONE)) ? 1U : 0U;
    g_height_safety_valid = (s_ready != 0U) ? 1U : 0U;
    if (s_ready == 0U)
    {
        g_height_safe_upper_mm = (float)VL53L1X_VALID_RANGE_MAX;
        g_height_safe_lower_mm = (float)VL53L1X_VALID_RANGE_MAX;
        g_height_safe_brake_distance_mm = 0.0f;
        return;
    }

    base_high_m = s_h_m;
    base_low_m = s_h_m;
    if ((now_tick_ms - s_last_safe_high_tick_ms) <= HEIGHT_SAFE_LATCH_MS)
    {
        if (s_last_safe_high_m > base_high_m)
        {
            base_high_m = s_last_safe_high_m;
        }
    }
    if ((now_tick_ms - s_last_safe_low_tick_ms) <= HEIGHT_SAFE_LATCH_MS)
    {
        if ((s_last_safe_low_m > 0.0f) && (s_last_safe_low_m < base_low_m))
        {
            base_low_m = s_last_safe_low_m;
        }
    }

    up_v_mps = (s_v_mps > 0.0f) ? s_v_mps : 0.0f;
    down_v_mps = (s_v_mps < 0.0f) ? -s_v_mps : 0.0f;
    up_brake_m = (up_v_mps * up_v_mps) /
                 (2.0f * HEIGHT_SAFE_BRAKE_ACCEL_MPS2);
    down_brake_m = (down_v_mps * down_v_mps) /
                   (2.0f * HEIGHT_SAFE_BRAKE_ACCEL_MPS2);

    g_height_safe_brake_distance_mm = up_brake_m * 1000.0f;
    g_height_safe_upper_mm = Height_Clamp(
        base_high_m * 1000.0f + HEIGHT_SAFE_MARGIN_MM +
        up_v_mps * HEIGHT_SAFE_RESPONSE_S * 1000.0f + up_brake_m * 1000.0f,
        HEIGHT_MIN_MM, HEIGHT_STATE_MAX_MM);
    g_height_safe_lower_mm = Height_Clamp(
        base_low_m * 1000.0f - HEIGHT_SAFE_MARGIN_MM -
        down_v_mps * HEIGHT_SAFE_RESPONSE_S * 1000.0f - down_brake_m * 1000.0f,
        HEIGHT_MIN_MM, HEIGHT_STATE_MAX_MM);
}

static void Height_CorrectObserver(float meas_mm,
                                   float health,
                                   uint8 meas_valid,
                                   uint8 inlier_count,
                                   float safe_high_mm,
                                   float safe_low_mm,
                                   uint8 safe_mask,
                                   uint8 pollution_detected)
{
    uint32 now_tick_ms = tick_1000us_cnt;
    uint32 prior_age_ms = HEIGHT_CONTROLLED_DESCENT_TIMEOUT_MS + 1U;
    float measurement_dt_s;
    float residual_m;
    float old_h_m;

    if (s_have_valid_measurement != 0U)
    {
        prior_age_ms = now_tick_ms - s_last_valid_measurement_tick_ms;
    }

    g_height_meas_health = health;
    g_height_meas_valid = ((meas_valid != 0U) && (inlier_count >= 2U) &&
                           (health >= HEIGHT_CONF_MIN)) ? 1U : 0U;
    g_height_inlier_count = inlier_count;
    g_height_residual_m = 0.0f;
    g_height_measurement_dt_s = 0.0f;
    g_height_inst_v_mps = 0.0f;
    g_height_tof_alpha = 0.0f;

    if ((safe_mask & 0x01U) != 0U)
    {
        s_last_safe_high_m = safe_high_mm * 0.001f;
        s_last_safe_high_tick_ms = now_tick_ms;
    }
    if ((safe_mask & 0x02U) != 0U)
    {
        s_last_safe_low_m = safe_low_mm * 0.001f;
        s_last_safe_low_tick_ms = now_tick_ms;
    }

    if (pollution_detected != 0U)
    {
        s_pollution_latched = 1U;
        s_pollution_clear_count = 0U;
    }
    else if ((meas_valid != 0U) && (s_pollution_latched != 0U))
    {
        s_pollution_clear_count++;
        if (s_pollution_clear_count >= HEIGHT_POLLUTION_CLEAR_FRAMES)
        {
            s_pollution_latched = 0U;
            s_pollution_clear_count = 0U;
        }
    }
    g_height_pollution_active = s_pollution_latched;

    if (g_height_meas_valid != 0U)
    {
        g_height_meas_mm = meas_mm;
        if (s_ready == 0U)
        {
            s_h_m = meas_mm * 0.001f;
            s_v_mps = 0.0f;
            s_ready = 1U;
        }

        measurement_dt_s = (s_have_valid_measurement != 0U) ?
            (float)(now_tick_ms - s_last_valid_measurement_tick_ms) * 0.001f : 0.01f;
        measurement_dt_s = Height_Clamp(measurement_dt_s, 0.001f, 3.0f);
        if (prior_age_ms > HEIGHT_NO_ASCENT_TIMEOUT_MS)
        {
            s_v_mps = 0.0f;
        }

        old_h_m = s_h_m;
        residual_m = meas_mm * 0.001f - s_h_m;
        residual_m = Height_Clamp(residual_m, -HEIGHT_MEAS_RESIDUAL_LIMIT_M,
                                  HEIGHT_MEAS_RESIDUAL_LIMIT_M);
        s_h_m += HEIGHT_NORMAL_ALPHA * residual_m;
        s_v_mps += HEIGHT_NORMAL_BETA * residual_m / measurement_dt_s;
        s_v_mps = Height_Clamp(s_v_mps, -2.0f, 2.0f);

        g_height_residual_m = residual_m;
        g_height_measurement_dt_s = measurement_dt_s;
        g_height_inst_v_mps = (s_h_m - old_h_m) / measurement_dt_s;
        g_height_tof_alpha = HEIGHT_NORMAL_ALPHA;
        s_miss = 0U;
        s_last_valid_measurement_tick_ms = now_tick_ms;
        s_have_valid_measurement = 1U;
    }
    else if (s_miss < HEIGHT_MISS_MAX)
    {
        s_miss++;
    }

    s_h_m = Height_Clamp(s_h_m, HEIGHT_MIN_MM * 0.001f,
                         HEIGHT_STATE_MAX_MM * 0.001f);
    g_tof_fused_height_mm = (s_ready != 0U) ?
                            s_h_m * 1000.0f : (float)VL53L1X_VALID_RANGE_MAX;
    g_height_fused_vz_mps = s_v_mps;
    g_height_observer_v_mps = s_v_mps;
    g_height_miss_count = s_miss;
    Height_UpdateSafety();
}

static void Height_Reset(void)
{
    s_h_m = 0.0f;
    s_v_mps = 0.0f;
    s_acc_lpf_mps2 = 0.0f;
    s_ready = 0U;
    s_miss = HEIGHT_MISS_MAX;
    s_last_tof_sample_seq = 0U;
    s_last_valid_measurement_tick_ms = 0U;
    s_have_valid_measurement = 0U;
    s_last_safe_high_m = 0.0f;
    s_last_safe_low_m = 0.0f;
    s_last_safe_high_tick_ms = 0U;
    s_last_safe_low_tick_ms = 0U;
    s_pollution_latched = 0U;
    s_pollution_clear_count = 0U;

    g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_valid = 0U;
    g_height_fused_vz_mps = 0.0f;
    g_height_meas_health = 0.0f;
    g_height_acc_up_mps2 = 0.0f;
    g_height_tof_sample_seq = 0U;
    g_height_tof_fresh_mask = 0U;
    g_height_tof_valid_mask = 0U;
    g_height_meas_valid = 0U;
    g_height_inlier_count = 0U;
    g_height_meas_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_height_residual_m = 0.0f;
    g_height_measurement_dt_s = 0.0f;
    g_height_inst_v_mps = 0.0f;
    g_height_acc_lpf_mps2 = 0.0f;
    g_height_observer_v_mps = 0.0f;
    g_height_miss_count = HEIGHT_MISS_MAX;
    g_height_measurement_age_ms = 0.0f;
    g_height_tof_alpha = 0.0f;
    g_height_safe_upper_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_height_safe_lower_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_height_safe_brake_distance_mm = 0.0f;
    g_height_dropout_mode = HEIGHT_DROPOUT_CONTROLLED_DESCENT;
    g_height_pollution_active = 0U;
    g_height_safety_valid = 0U;
}

void TOF_Init(void)
{
    Height_Reset();
    VL53L1X_Init();
}

void Height_Est_predict_1000HZ(void)
{
    uint32 age_ms = HEIGHT_CONTROLLED_DESCENT_TIMEOUT_MS + 1U;
    float acc;

    g_height_acc_up_mps2 = AccelCalibration_GetVerticalAccelUpMps2();
    acc = Height_Clamp(g_height_acc_up_mps2,
                       -HEIGHT_ACC_LIMIT_MPS2, HEIGHT_ACC_LIMIT_MPS2);
    s_acc_lpf_mps2 += HEIGHT_ACC_LPF * (acc - s_acc_lpf_mps2);
    g_height_acc_lpf_mps2 = s_acc_lpf_mps2;

    if (s_have_valid_measurement != 0U)
    {
        age_ms = tick_1000us_cnt - s_last_valid_measurement_tick_ms;
    }

    if (s_ready != 0U)
    {
        if (age_ms <= HEIGHT_NO_ASCENT_TIMEOUT_MS)
        {
            s_v_mps += HEIGHT_ACC_VEL_GAIN * s_acc_lpf_mps2 * HEIGHT_PREDICT_DT_S;
            s_v_mps *= HEIGHT_VEL_LEAK;
        }
        else
        {
            s_v_mps *= HEIGHT_NO_MEAS_VEL_LEAK;
        }

        s_h_m += s_v_mps * HEIGHT_PREDICT_DT_S;
        s_h_m = Height_Clamp(s_h_m, HEIGHT_MIN_MM * 0.001f,
                             HEIGHT_STATE_MAX_MM * 0.001f);
        s_v_mps = Height_Clamp(s_v_mps, -3.0f, 3.0f);
        g_tof_fused_height_mm = s_h_m * 1000.0f;
        g_height_fused_vz_mps = s_v_mps;
        g_height_observer_v_mps = s_v_mps;
        g_height_miss_count = s_miss;
    }

    Height_UpdateSafety();
}

void Height_Est_update_100HZ(void)
{
    const VL53L1X_data_struct *tof = VL53L1X_GetData();
    float meas_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float health = 0.0f;
    float safe_high_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float safe_low_mm = (float)VL53L1X_VALID_RANGE_MAX;
    uint8 inlier_count = 0U;
    uint8 safe_mask = 0U;
    uint8 pollution_detected = 0U;
    uint8 meas_valid = 0U;

    if ((tof == 0) || (tof->sample_seq == s_last_tof_sample_seq))
    {
        g_height_tof_fresh_mask = 0U;
        Height_CorrectObserver(meas_mm, health, 0U, inlier_count,
                               safe_high_mm, safe_low_mm, safe_mask,
                               pollution_detected);
        return;
    }

    s_last_tof_sample_seq = tof->sample_seq;
    g_height_tof_sample_seq = tof->sample_seq;
    g_height_tof_fresh_mask = tof->fresh_mask;
    g_height_tof_valid_mask = 0U;
    if (tof->valid[0] != 0U) { g_height_tof_valid_mask |= 0x01U; }
    if (tof->valid[1] != 0U) { g_height_tof_valid_mask |= 0x02U; }
    if (tof->valid[2] != 0U) { g_height_tof_valid_mask |= 0x04U; }
    if (tof->valid[3] != 0U) { g_height_tof_valid_mask |= 0x08U; }

    meas_valid = Height_BuildMeasure(tof, &meas_mm, &health, &inlier_count,
                                    &safe_high_mm, &safe_low_mm, &safe_mask,
                                    &pollution_detected);
    Height_CorrectObserver(meas_mm, health, meas_valid, inlier_count,
                           safe_high_mm, safe_low_mm, safe_mask,
                           pollution_detected);
}
