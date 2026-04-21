#include "Height_Est.h"
#include "zf_common_headfile.h"
#include <math.h>

/* 融合高度，单位 mm */
float g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
/* 1 号电机下方 TOF 高度 */
float g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
/* 4 号电机下方 TOF 高度 */
float g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
/* 融合高度有效标志 */
uint8 g_tof_fused_valid = 0U;
/* TOF 融合高度单次差分速度，单位 m/s，上升为正 */
float g_tof_fused_vz_mps = 0.0f;
/* TOF 差分速度经 3Hz 一阶低通后的速度，单位 m/s，上升为正 */
float g_height_fused_vz_mps = 0.0f;
/* 上一帧有效融合高度，单位 mm */
static float s_tof_vz_prev_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
/* TOF 速度微分历史高度有效标志 */
static uint8_t s_tof_vz_prev_valid = 0U;

#define HEIGHT_EST_TOF_DIFF_BLEND_GATE_MM      100.0f  /* 双探头高度差在该阈值内取平均，单位 mm */
#define HEIGHT_EST_TOF_NEAR_SWITCH_MM          300.0f  /* 近距离判据阈值，单位 mm */
#define HEIGHT_EST_TOF_BOTH_INVALID_HOLD_CNT   3U      /* 双探头同时无效时的短时保持帧数 */
#define HEIGHT_EST_TOF_DH_TO_VZ_MPS_SCALE      0.1f    /* 100Hz 下高度差(mm)转速度(m/s)比例 */
#define HEIGHT_EST_VZ_LPF_ALPHA                0.1586f /* 速度低通系数：fs=100Hz、fc=3Hz */
static void TOF_ResetOutput(void)
{
    g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_tof_fused_valid = 0U;
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
    g_tof_fused_vz_mps = 0.0f;
    g_height_fused_vz_mps = 0.0f;
    s_tof_vz_prev_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    s_tof_vz_prev_valid = 0U;
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
    float candidate_height_mm = 0.0f;
    float diff_14_mm = 0.0f;

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

    if ((0U != tof1_valid) && (0U != tof4_valid))
    {
        diff_14_mm = fabsf(g_tof1_height_mm - g_tof4_height_mm);
        if (diff_14_mm <= HEIGHT_EST_TOF_DIFF_BLEND_GATE_MM)
        {
            candidate_height_mm = 0.5f * (g_tof1_height_mm + g_tof4_height_mm);
        }
        else if ((g_tof1_height_mm < HEIGHT_EST_TOF_NEAR_SWITCH_MM) && (g_tof4_height_mm > g_tof1_height_mm))
        {
            candidate_height_mm = g_tof4_height_mm;
        }
        else if ((g_tof4_height_mm < HEIGHT_EST_TOF_NEAR_SWITCH_MM) && (g_tof1_height_mm > g_tof4_height_mm))
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

        g_tof_fused_height_mm = candidate_height_mm;
        g_tof_fused_valid = 1U;
        s_both_invalid_hold_cnt = 0U;
    }
    else if (0U != tof1_valid)
    {
        candidate_height_mm = g_tof1_height_mm;
        g_tof_fused_height_mm = candidate_height_mm;
        g_tof_fused_valid = 1U;
        s_both_invalid_hold_cnt = 0U;
    }
    else if (0U != tof4_valid)
    {
        candidate_height_mm = g_tof4_height_mm;
        g_tof_fused_height_mm = candidate_height_mm;
        g_tof_fused_valid = 1U;
        s_both_invalid_hold_cnt = 0U;
    }
    else if ((0U != s_prev_fused_valid) && (s_both_invalid_hold_cnt < HEIGHT_EST_TOF_BOTH_INVALID_HOLD_CNT))
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

/*
 * 函数功能：100Hz 更新 TOF 高度速度，输出 raw 差分速度与 3Hz 低通速度。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void Height_Est_update_100HZ(void)
{
    float tof_vz_raw_mps = 0.0f;

    TOF_update_100HZ();

    if (0U != g_tof_fused_valid)
    {
        /* 首帧或重获有效时执行零速重同步，避免速度尖峰 */
        if (0U == s_tof_vz_prev_valid)
        {
            s_tof_vz_prev_height_mm = g_tof_fused_height_mm;
            s_tof_vz_prev_valid = 1U;
            tof_vz_raw_mps = 0.0f;
        }
        else
        {
            tof_vz_raw_mps = (g_tof_fused_height_mm - s_tof_vz_prev_height_mm) * HEIGHT_EST_TOF_DH_TO_VZ_MPS_SCALE;
            s_tof_vz_prev_height_mm = g_tof_fused_height_mm;
        }

        g_tof_fused_vz_mps = tof_vz_raw_mps;
    }
    else
    {
        /* TOF 无效时保持上一帧速度，并清除历史有效位以便重获时重同步 */
        s_tof_vz_prev_valid = 0U;
    }

    /* 一阶 IIR 低通：fs=100Hz，fc=3Hz，alpha=0.1586 */
    g_height_fused_vz_mps += HEIGHT_EST_VZ_LPF_ALPHA * (g_tof_fused_vz_mps - g_height_fused_vz_mps);

    // wifi_justfloat(g_tof_fused_height_mm, g_tof_fused_vz_mps, g_height_fused_vz_mps);
}

