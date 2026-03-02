#include "Height_Est.h"

#define HEIGHT_EST_UPDATE_HZ_F             (100.0f)
#define HEIGHT_EST_TOF_STEP_MAX_M          (0.013f)
#define HEIGHT_EST_TOF_STEP_SAFE_M         (0.008f)
#define HEIGHT_EST_TOF_STEP_HARD_M         (0.005f)
#define HEIGHT_EST_TOF_MISMATCH_GATE_M     (0.030f)
#define HEIGHT_EST_TOF_MISMATCH_HARD_GATE_M (0.120f)
#define HEIGHT_EST_TOF_LPF_ALPHA           (0.16f)
#define HEIGHT_EST_BARO_FALLBACK_ALPHA     (0.03f)
#define HEIGHT_EST_BARO_BIAS_ALPHA         (0.002f)
#define HEIGHT_EST_VZ_ALPHA                (0.12f)
#define HEIGHT_EST_INVALID_HOLD_FRAMES     (8U)
#define HEIGHT_EST_BARO_DECIM              (1U)
#define HEIGHT_EST_VL53_RECOVER_DECIM      (10U)

uint16 g_height_est_mm = 0U;
float g_height_est_m = 0.0f;
float g_height_vz_mps = 0.0f;
uint8 g_height_est_valid = 0U;
uint8 g_height_est_source = HEIGHT_EST_SOURCE_NONE;

static uint8 s_height_est_inited = 0U;
static float s_height_est_z_m = 0.0f;
static float s_height_est_prev_z_m = 0.0f;
static float s_height_est_baro_bias_m = 0.0f;
static float s_height_est_tof_lpf_m = 0.0f;
static uint8 s_height_est_tof_lpf_inited = 0U;
static uint16 s_height_est_invalid_hold_cnt = 0U;

static float Height_Est_ClampFloat(float value, float min_value, float max_value)
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

static float Height_Est_AbsFloat(float value)
{
    if (value >= 0.0f)
    {
        return value;
    }
    return -value;
}

static uint16 Height_Est_ToMM(float value_m)
{
    float value_mm = value_m * 1000.0f;

    if (value_mm < 0.0f)
    {
        value_mm = 0.0f;
    }
    else if (value_mm > 65535.0f)
    {
        value_mm = 65535.0f;
    }

    return (uint16)(value_mm + 0.5f);
}

void Height_Est_Reset(void)
{
    s_height_est_z_m = 0.0f;
    s_height_est_prev_z_m = 0.0f;
    s_height_est_baro_bias_m = 0.0f;
    s_height_est_tof_lpf_m = 0.0f;
    s_height_est_tof_lpf_inited = 0U;
    s_height_est_invalid_hold_cnt = 0U;

    g_height_est_mm = 0U;
    g_height_est_m = 0.0f;
    g_height_vz_mps = 0.0f;
    g_height_est_valid = 0U;
    g_height_est_source = HEIGHT_EST_SOURCE_NONE;
}

void Height_Est_Init(void)
{
    TOF_Init();
    Baro_Init();
    Height_Est_Reset();
    s_height_est_inited = 1U;
}

void Height_Est_Update_100HZ(void)
{
    static uint8 s_baro_div_cnt = 0U;
    static uint8 s_vl53_recover_div_cnt = 0U;
    uint8 tof_valid;
    uint8 baro_valid;
    uint8 baro_updated = 0U;
    float z_tof_m = 0.0f;
    float z_baro_m;
    float z_baro_corrected_m;
    float z_tof_use_m = 0.0f;
    float innovation_m;
    float tof_step_limit_m;
    float tof_mismatch_m;
    float delta_z_m;
    float vz_raw_mps;

    if (0U == s_height_est_inited)
    {
        return;
    }

    TOF_update_100HZ();

    s_baro_div_cnt++;
    if (s_baro_div_cnt >= HEIGHT_EST_BARO_DECIM)
    {
        s_baro_div_cnt = 0U;
        Baro_update_100HZ();
        baro_updated = g_baro_sample_new;
    }

    s_vl53_recover_div_cnt++;
    if (s_vl53_recover_div_cnt >= HEIGHT_EST_VL53_RECOVER_DECIM)
    {
        s_vl53_recover_div_cnt = 0U;
        VL53L1X_recover_update_10HZ();
    }

    tof_valid = g_tof_fused_valid;
    baro_valid = (g_baro_ref_pressure > 1000.0f) ? 1U : 0U;
    z_baro_m = g_baro_altitude;

    if (0U != tof_valid)
    {
        z_tof_m = 0.001f * (float)g_tof_fused_height_mm;
        if (0U == s_height_est_tof_lpf_inited)
        {
            s_height_est_tof_lpf_m = z_tof_m;
            s_height_est_tof_lpf_inited = 1U;
        }
        else
        {
            s_height_est_tof_lpf_m += HEIGHT_EST_TOF_LPF_ALPHA * (z_tof_m - s_height_est_tof_lpf_m);
        }
        z_tof_use_m = s_height_est_tof_lpf_m;

        if (0U == g_height_est_valid)
        {
            s_height_est_z_m = z_tof_use_m;
            s_height_est_prev_z_m = z_tof_use_m;
            g_height_vz_mps = 0.0f;
        }
        else
        {
            tof_step_limit_m = HEIGHT_EST_TOF_STEP_MAX_M;
            if ((0U != g_tof2_valid) && (0U != g_tof3_valid))
            {
                tof_mismatch_m = Height_Est_AbsFloat(0.001f * (float)g_tof2_height_mm - 0.001f * (float)g_tof3_height_mm);
                if (tof_mismatch_m > HEIGHT_EST_TOF_MISMATCH_HARD_GATE_M)
                {
                    tof_step_limit_m = HEIGHT_EST_TOF_STEP_HARD_M;
                }
                else if (tof_mismatch_m > HEIGHT_EST_TOF_MISMATCH_GATE_M)
                {
                    tof_step_limit_m = HEIGHT_EST_TOF_STEP_SAFE_M;
                }
            }

            innovation_m = z_tof_use_m - s_height_est_z_m;
            s_height_est_z_m += Height_Est_ClampFloat(innovation_m, -tof_step_limit_m, tof_step_limit_m);
        }

        if (0U != baro_valid)
        {
            if (0U != baro_updated)
            {
                s_height_est_baro_bias_m += HEIGHT_EST_BARO_BIAS_ALPHA * (z_tof_use_m - (z_baro_m + s_height_est_baro_bias_m));
            }
            g_height_est_source = HEIGHT_EST_SOURCE_MIXED;
        }
        else
        {
            g_height_est_source = HEIGHT_EST_SOURCE_TOF;
        }

        g_height_est_valid = 1U;
        s_height_est_invalid_hold_cnt = 0U;
    }
    else if (0U != baro_valid)
    {
        s_height_est_tof_lpf_inited = 0U;
        z_baro_corrected_m = z_baro_m + s_height_est_baro_bias_m;

        if (0U == g_height_est_valid)
        {
            s_height_est_z_m = z_baro_corrected_m;
            s_height_est_prev_z_m = z_baro_corrected_m;
            g_height_vz_mps = 0.0f;
            g_height_est_valid = 1U;
        }
        else if (0U != baro_updated)
        {
            s_height_est_z_m += HEIGHT_EST_BARO_FALLBACK_ALPHA * (z_baro_corrected_m - s_height_est_z_m);
        }

        g_height_est_source = HEIGHT_EST_SOURCE_BARO;
        s_height_est_invalid_hold_cnt = 0U;
    }
    else
    {
        s_height_est_tof_lpf_inited = 0U;
        if ((0U != g_height_est_valid) && (s_height_est_invalid_hold_cnt < HEIGHT_EST_INVALID_HOLD_FRAMES))
        {
            s_height_est_invalid_hold_cnt++;
            g_height_est_source = HEIGHT_EST_SOURCE_NONE;
        }
        else
        {
            g_height_est_valid = 0U;
            g_height_est_source = HEIGHT_EST_SOURCE_NONE;
            g_height_vz_mps = 0.0f;
        }
    }

    if (0U != g_height_est_valid)
    {
        delta_z_m = s_height_est_z_m - s_height_est_prev_z_m;
        vz_raw_mps = delta_z_m * HEIGHT_EST_UPDATE_HZ_F;
        g_height_vz_mps += HEIGHT_EST_VZ_ALPHA * (vz_raw_mps - g_height_vz_mps);
        s_height_est_prev_z_m = s_height_est_z_m;
    }

    g_height_est_m = s_height_est_z_m;
    g_height_est_mm = Height_Est_ToMM(s_height_est_z_m);
}


