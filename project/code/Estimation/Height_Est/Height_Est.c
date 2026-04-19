#include "Height_Est.h"

/* 融合高度，单位 mm */
float g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
/* 1 号电机下方 TOF 高度 */
float g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
/* 4 号电机下方 TOF 高度 */
float g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
/* 融合高度有效标志 */
uint8 g_tof_fused_valid = 0U;
/* 融合高度速度，当前固定为 0 */
float g_tof_fused_vz_mps = 0.0f;

/*
 * 函数功能：将全部 TOF 输出恢复为默认无效值。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
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
