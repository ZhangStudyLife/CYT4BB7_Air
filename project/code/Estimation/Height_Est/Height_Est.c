#include "Height_Est.h"
#include "zf_common_headfile.h"
#include <math.h>

/* 融合高度，单位 mm */
float g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
float g_tof_fused_height_mm_last = (float)VL53L1X_VALID_RANGE_MAX;
/* 1 号电机下方 TOF 高度 */
float g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
/* 4 号电机下方 TOF 高度 */
float g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
/* 融合高度有效标志 */
uint8 g_tof_fused_valid = 0U;
/* TOF单次微分的速度*/
float g_tof_fused_vz_mps = 0.0f;
/* 融合高度速度 */
float g_height_fused_vz_mps = 0.0f;
/* 纯粹的加速度积分 */
float g_acc_sum_vz_mps2 = 0.0f;
static uint8_t s_vz_fusion_inited = 0U;
static uint32_t s_tof_update_tick = 0U;
static uint8_t s_tof_new_sample = 0U;
static uint8_t s_tof_speed_prev_valid = 0U;

#define HEIGHT_EST_DT_1000HZ           0.001f
#define HEIGHT_EST_DT_TOF_NOMINAL_S    0.010f
#define HEIGHT_EST_DT_TOF_NORM_MIN_S   0.009f
#define HEIGHT_EST_DT_TOF_NORM_MAX_S   0.011f
#define HEIGHT_EST_VZ_TOF_BLEND        0.02f
#define HEIGHT_EST_VZ_TOF_BLEND_SOFT   0.003f
#define HEIGHT_EST_VZ_MAX_ABS_MPS      3.0f
#define HEIGHT_EST_VZ_SOFT_GATE_MPS    0.6f
#define HEIGHT_EST_VZ_OUTLIER_GATE_MPS 1.5f
#define HEIGHT_EST_TOF_VZ_MAX_ABS_MPS  2.5f
#define HEIGHT_EST_TOF_TILT_FADE_START_DEG 10.0f
#define HEIGHT_EST_TOF_TILT_FADE_END_DEG   16.0f
#define HEIGHT_EST_STATIC_PRED_VZ_MAX_MPS  0.12f
#define HEIGHT_EST_STATIC_TOF_VZ_MAX_MPS   0.18f
#define HEIGHT_EST_STATIC_HEIGHT_DELTA_MM  8.0f
#define HEIGHT_EST_STATIC_ERR_DEADZONE_MPS 0.18f

extern volatile uint32 tick_1000us_cnt;
static void TOF_ResetOutput(void)
{
    g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_tof_fused_valid = 0U;
    g_tof_fused_vz_mps = 0.0f;
}

/*
 * 函数功能：初始化两路 TOF 驱动与融合输出。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void TOF_Init(void)
{
    TOF_ResetOutput();
    s_vz_fusion_inited = 0U;
    g_height_fused_vz_mps = 0.0f;
    g_acc_sum_vz_mps2 = 0.0f;
    g_tof_fused_height_mm_last = (float)VL53L1X_VALID_RANGE_MAX;
    s_tof_update_tick = 0U;
    s_tof_new_sample = 0U;
    s_tof_speed_prev_valid = 0U;
    VL53L1X_Init();
}

/*
 * 函数功能：100Hz 更新两路 TOF，并输出最简单的融合高度。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void TOF_update_100HZ(void)
{
    const VL53L1X_data_struct *tof_data = 0;
    uint8 tof1_valid = 0U;
    uint8 tof4_valid = 0U;
    static float s_prev_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    static uint8 s_prev_fused_valid = 0U;
    static uint8 s_both_invalid_hold_cnt = 0U;
    float acc_down_g = 0.0f;
    float candidate_height_mm = 0.0f;
    float diff_14_mm = 0.0f;

    s_tof_new_sample = 0U;
    VL53L1X_Update();
    tof_data = VL53L1X_GetData();

    TOF_ResetOutput();

    if (0 == tof_data)
    {
        return;
    }

    tof1_valid = tof_data->valid[0U];
    tof4_valid = tof_data->valid[1U];

    if (0U != tof1_valid)
    {
        g_tof1_height_mm = (float)tof_data->distance_mm[0U] * g_euler.cos_pitch * g_euler.cos_roll;
    }

    if (0U != tof4_valid)
    {
        g_tof4_height_mm = (float)tof_data->distance_mm[1U] * g_euler.cos_pitch * g_euler.cos_roll;
    }

    acc_down_g = -g_euler.sin_pitch * g_imufilter_1000hz.accx +
                 g_euler.sin_roll * g_euler.cos_pitch * g_imufilter_1000hz.accy +
                 g_euler.cos_roll * g_euler.cos_pitch * g_imufilter_1000hz.accz +
                 1.0f;

    if ((0U != tof1_valid) && (0U != tof4_valid))
    {
        diff_14_mm = fabsf(g_tof1_height_mm - g_tof4_height_mm);
        if (diff_14_mm <= 100.0f)
        {
            candidate_height_mm = 0.5f * (g_tof1_height_mm + g_tof4_height_mm);
        }
        else if ((g_tof1_height_mm < 300.0f) && (g_tof4_height_mm > g_tof1_height_mm))
        {
            candidate_height_mm = g_tof4_height_mm;
        }
        else if ((g_tof4_height_mm < 300.0f) && (g_tof1_height_mm > g_tof4_height_mm))
        {
            candidate_height_mm = g_tof1_height_mm;
        }
        else if (0U != s_prev_fused_valid)
        {
            if (fabsf(g_tof1_height_mm - s_prev_fused_height_mm) <= fabsf(g_tof4_height_mm - s_prev_fused_height_mm))
            {
                candidate_height_mm = g_tof1_height_mm;
            }
            else
            {
                candidate_height_mm = g_tof4_height_mm;
            }
        }
        else
        {
            candidate_height_mm = 0.5f * (g_tof1_height_mm + g_tof4_height_mm);
        }

        if ((0U != s_prev_fused_valid) &&
            (fabsf(candidate_height_mm - s_prev_fused_height_mm) > 100.0f) &&
            (fabsf(acc_down_g) < 0.08f))
        {
            candidate_height_mm = s_prev_fused_height_mm;
        }

        g_tof_fused_height_mm = candidate_height_mm;
        g_tof_fused_valid = 1U;
        s_both_invalid_hold_cnt = 0U;
        s_tof_new_sample = 1U;
    }
    else if (0U != tof1_valid)
    {
        candidate_height_mm = g_tof1_height_mm;
        if ((0U != s_prev_fused_valid) &&
            (fabsf(candidate_height_mm - s_prev_fused_height_mm) > 100.0f) &&
            (fabsf(acc_down_g) < 0.08f))
        {
            candidate_height_mm = s_prev_fused_height_mm;
        }

        g_tof_fused_height_mm = candidate_height_mm;
        g_tof_fused_valid = 1U;
        s_both_invalid_hold_cnt = 0U;
        s_tof_new_sample = 1U;
    }
    else if (0U != tof4_valid)
    {
        candidate_height_mm = g_tof4_height_mm;
        if ((0U != s_prev_fused_valid) &&
            (fabsf(candidate_height_mm - s_prev_fused_height_mm) > 100.0f) &&
            (fabsf(acc_down_g) < 0.08f))
        {
            candidate_height_mm = s_prev_fused_height_mm;
        }

        g_tof_fused_height_mm = candidate_height_mm;
        g_tof_fused_valid = 1U;
        s_both_invalid_hold_cnt = 0U;
        s_tof_new_sample = 1U;
    }
    else if ((0U != s_prev_fused_valid) && (s_both_invalid_hold_cnt < 3U))
    {
        g_tof_fused_height_mm = s_prev_fused_height_mm;
        g_tof_fused_valid = 1U;
        s_both_invalid_hold_cnt++;
    }

    if (0U != g_tof_fused_valid)
    {
        s_prev_fused_height_mm = g_tof_fused_height_mm;
        s_prev_fused_valid = 1U;
    }
}


void Height_Est_update_100HZ(void)
{
    float dt_tof_s = HEIGHT_EST_DT_TOF_NOMINAL_S;
    float dt_norm_s = HEIGHT_EST_DT_TOF_NOMINAL_S;
    float predict_vz_mps = g_height_fused_vz_mps;
    float base_weight = 0.0f;
    float tof_weight = 0.0f;
    float height_delta_mm = 0.0f;
    float tof_vz_abs_mps = 0.0f;
    float tof_vz_err_abs_mps = 0.0f;
    float tilt_abs_deg = 0.0f;
    float tilt_weight = 1.0f;
    uint32_t tick_delta = 0U;
    uint8_t tof_speed_ready = 0U;
    uint8_t tof_speed_valid = 0U;
    uint8_t tof_abs_block = 0U;
    uint8_t tof_hard_block = 0U;
    uint8_t tof_static_block = 0U;

    TOF_update_100HZ();

    g_tof_fused_vz_mps = 0.0f;

    if ((0U != s_tof_new_sample) && (0U != g_tof_fused_valid))
    {
        if (0U != s_tof_update_tick)
        {
            tick_delta = tick_1000us_cnt - s_tof_update_tick;
            if ((tick_delta >= 1U) && (tick_delta <= 50U))
            {
                dt_tof_s = (float)tick_delta * HEIGHT_EST_DT_1000HZ;
            }
        }

        s_tof_update_tick = tick_1000us_cnt;

        if (0U != s_tof_speed_prev_valid)
        {
            height_delta_mm = g_tof_fused_height_mm - g_tof_fused_height_mm_last;
            g_tof_fused_vz_mps = height_delta_mm * 0.001f / dt_tof_s;
            tof_speed_ready = 1U;
        }

        g_tof_fused_height_mm_last = g_tof_fused_height_mm;
        s_tof_speed_prev_valid = 1U;
    }
    else if (0U == g_tof_fused_valid)
    {
        s_tof_speed_prev_valid = 0U;
    }

    tof_vz_abs_mps = fabsf(g_tof_fused_vz_mps);
    tof_vz_err_abs_mps = fabsf(g_tof_fused_vz_mps - predict_vz_mps);
    tilt_abs_deg = fabsf(g_euler.pitch) + fabsf(g_euler.roll);
    tof_speed_valid = ((0U != s_tof_new_sample) && (0U != g_tof_fused_valid) && (0U != tof_speed_ready)) ? 1U : 0U;

    if (tof_vz_abs_mps > HEIGHT_EST_TOF_VZ_MAX_ABS_MPS)
    {
        tof_abs_block = 1U;
    }

    if (tilt_abs_deg >= HEIGHT_EST_TOF_TILT_FADE_END_DEG)
    {
        tilt_weight = 0.0f;
    }
    else if (tilt_abs_deg > HEIGHT_EST_TOF_TILT_FADE_START_DEG)
    {
        tilt_weight = (HEIGHT_EST_TOF_TILT_FADE_END_DEG - tilt_abs_deg) /
                      (HEIGHT_EST_TOF_TILT_FADE_END_DEG - HEIGHT_EST_TOF_TILT_FADE_START_DEG);
    }

    if (tof_vz_err_abs_mps > HEIGHT_EST_VZ_OUTLIER_GATE_MPS)
    {
        tof_hard_block = 1U;
    }

    if ((fabsf(predict_vz_mps) <= HEIGHT_EST_STATIC_PRED_VZ_MAX_MPS) &&
        (tof_vz_abs_mps <= HEIGHT_EST_STATIC_TOF_VZ_MAX_MPS) &&
        (fabsf(height_delta_mm) <= HEIGHT_EST_STATIC_HEIGHT_DELTA_MM) &&
        (tof_vz_err_abs_mps <= HEIGHT_EST_STATIC_ERR_DEADZONE_MPS))
    {
        tof_static_block = 1U;
    }

    if ((0U != tof_speed_valid) &&
        (0U == tof_abs_block) &&
        (0U == tof_hard_block) &&
        (0U == tof_static_block))
    {
        if (tof_vz_err_abs_mps <= HEIGHT_EST_VZ_SOFT_GATE_MPS)
        {
            base_weight = HEIGHT_EST_VZ_TOF_BLEND;
        }
        else
        {
            base_weight = HEIGHT_EST_VZ_TOF_BLEND_SOFT;
        }
    }

    if (base_weight > 0.0f)
    {
        dt_norm_s = dt_tof_s;
        if (dt_norm_s < HEIGHT_EST_DT_TOF_NORM_MIN_S)
        {
            dt_norm_s = HEIGHT_EST_DT_TOF_NORM_MIN_S;
        }
        else if (dt_norm_s > HEIGHT_EST_DT_TOF_NORM_MAX_S)
        {
            dt_norm_s = HEIGHT_EST_DT_TOF_NORM_MAX_S;
        }

        tof_weight = 1.0f - powf(1.0f - base_weight, dt_norm_s / HEIGHT_EST_DT_1000HZ);
        tof_weight *= tilt_weight;
    }

    g_height_fused_vz_mps = (1.0f - tof_weight) * predict_vz_mps + tof_weight * g_tof_fused_vz_mps;
    if (g_height_fused_vz_mps > HEIGHT_EST_VZ_MAX_ABS_MPS)
    {
        g_height_fused_vz_mps = HEIGHT_EST_VZ_MAX_ABS_MPS;
    }
    else if (g_height_fused_vz_mps < -HEIGHT_EST_VZ_MAX_ABS_MPS)
    {
        g_height_fused_vz_mps = -HEIGHT_EST_VZ_MAX_ABS_MPS;
    }
}

void Height_Est_update_1000HZ(void)
{
    /* 1000Hz 更新高度估计 */
    float s_accup_mps2 = 0.0f;

    s_accup_mps2 = AccelCalibration_GetVerticalAccelUpMps2();
    g_acc_sum_vz_mps2 += s_accup_mps2 * HEIGHT_EST_DT_1000HZ;

    if (0U == s_vz_fusion_inited)
    {
        g_height_fused_vz_mps = 0.0f;
        s_vz_fusion_inited = 1U;
    }

    g_height_fused_vz_mps += s_accup_mps2 * HEIGHT_EST_DT_1000HZ;
    if (g_height_fused_vz_mps > HEIGHT_EST_VZ_MAX_ABS_MPS)
    {
        g_height_fused_vz_mps = HEIGHT_EST_VZ_MAX_ABS_MPS;
    }
    else if (g_height_fused_vz_mps < -HEIGHT_EST_VZ_MAX_ABS_MPS)
    {
        g_height_fused_vz_mps = -HEIGHT_EST_VZ_MAX_ABS_MPS;
    }

    wifi_justfloat(tick_1000us_cnt,
                   s_accup_mps2,
                   g_acc_sum_vz_mps2,
                   g_tof_fused_vz_mps,
                   g_height_fused_vz_mps,
                   g_tof_fused_height_mm,
                   g_tof_fused_valid,
                   g_euler.pitch,
                   g_euler.roll);
}
