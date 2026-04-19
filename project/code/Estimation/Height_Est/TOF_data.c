#include "TOF_data.h"

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
        g_tof_fused_height_mm = 0.5f * (g_tof1_height_mm + g_tof4_height_mm);
        g_tof_fused_valid = 1U;
    }
    else if (0U != tof1_valid)
    {
        g_tof_fused_height_mm = g_tof1_height_mm;
        g_tof_fused_valid = 1U;
    }
    else if (0U != tof4_valid)
    {
        g_tof_fused_height_mm = g_tof4_height_mm;
        g_tof_fused_valid = 1U;
    }
}
