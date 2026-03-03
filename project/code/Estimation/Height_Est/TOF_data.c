#include "TOF_data.h"
#include "zf_common_headfile.h"

/* TOF通道2有效位掩码 */
#define TOF_CH2_VALID_MASK               (1U << 0)
/* TOF通道3有效位掩码 */
#define TOF_CH3_VALID_MASK               (1U << 1)

/* TOF最小有效测距，单位：mm */
#define TOF_MIN_VALID_MM                 (40U)
/* TOF最大有效测距，单位：mm */
#define TOF_MAX_VALID_MM                 (1300U)
/* 姿态补偿最小余弦项，防止大倾角时过度衰减 */
#define TOF_COS_TERM_MIN                 (0.35f)

/* 双TOF一致性门限，单位：mm */
#define TOF_AGREE_GATE_MM                (22U)
/* 双TOF强失配门限，单位：mm */
#define TOF_HARD_MISMATCH_GATE_MM        (120U)
/* 切换融合来源所需连续确认帧数 */
#define TOF_SWITCH_CONFIRM_FRAMES        (7U)
/* 融合失效后保持上一融合值的帧数 */
#define TOF_FUSED_HOLD_FRAMES            (8U)
/* 一致区间下融合输出单步最大变化，单位：mm */
#define TOF_STEP_FAST_MM                 (16U)
/* 常规安全区间下融合输出单步最大变化，单位：mm */
#define TOF_STEP_SAFE_MM                 (9U)
/* 强失配区间下融合输出单步最大变化，单位：mm */
#define TOF_STEP_HARD_MM                 (5U)

/* 判定通道“冻结”的连续相同值帧阈值 */
#define TOF_SAME_VALUE_FRAMES_TH         (25U)
/* 冻结惩罚基值，单位：mm */
#define TOF_SAME_VALUE_PENALTY_BASE      (120U)
/* 冻结惩罚每超一帧增量，单位：mm */
#define TOF_SAME_VALUE_PENALTY_STEP      (2U)
/* 冻结惩罚上限，单位：mm */
#define TOF_SAME_VALUE_PENALTY_MAX       (400U)
/* 非新鲜数据附加惩罚，单位：mm */
#define TOF_NOT_FRESH_PENALTY            (80U)

/* 开机标定采样点数 */
#define TOF_CALIBRATION_SAMPLES          (100U)
/* 标定采样间隔，单位：ms */
#define TOF_CALIBRATION_DT_MS            (10U)

/* 融合来源：无有效来源 */
#define TOF_SRC_NONE                     (0U)
/* 融合来源：仅通道2 */
#define TOF_SRC_CH2                      (1U)
/* 融合来源：仅通道3 */
#define TOF_SRC_CH3                      (2U)
/* 融合来源：通道2与通道3均值 */
#define TOF_SRC_AVG                      (3U)

typedef struct
{
    uint8 has_last_mm;
    uint8 fresh_now;
    uint16 last_mm;
    uint16 invalid_streak;
    uint16 same_value_streak;
} TOFChannelState_t;

/* 融合后TOF高度，单位：mm，无效时为VL53L1X_INVALID_DISTANCE_MM */
uint16 g_tof_fused_height_mm = VL53L1X_INVALID_DISTANCE_MM;
/* TOF通道2显示高度，单位：mm，无效时为VL53L1X_INVALID_DISTANCE_MM */
uint16 g_tof2_height_mm = VL53L1X_INVALID_DISTANCE_MM;
/* TOF通道3显示高度，单位：mm，无效时为VL53L1X_INVALID_DISTANCE_MM */
uint16 g_tof3_height_mm = VL53L1X_INVALID_DISTANCE_MM;
/* 融合高度有效标志，1=有效，0=无效 */
uint8 g_tof_fused_valid = 0U;
/* 通道2参与融合的原始有效标志，1=有效，0=无效 */
uint8 g_tof2_valid = 0U;
/* 通道3参与融合的原始有效标志，1=有效，0=无效 */
uint8 g_tof3_valid = 0U;
/* 本帧融合是否使用了通道2，1=使用，0=未使用 */
uint8 g_tof2_used_in_fusion = 0U;
/* 本帧融合是否使用了通道3，1=使用，0=未使用 */
uint8 g_tof3_used_in_fusion = 0U;
/* 当前融合来源，取值见TOF_SRC_*宏 */
uint8 g_tof_fused_source = TOF_SRC_NONE;

/* TOF模块初始化完成标志，1=已初始化，0=未初始化 */
static uint8 s_tof_inited = 0U;
/* TOF通道2零偏补偿，单位：mm */
static int16 s_tof2_offset_mm = 0;
/* TOF通道3零偏补偿，单位：mm */
static int16 s_tof3_offset_mm = 0;

/* TOF通道2运行状态缓存 */
static TOFChannelState_t s_tof2_state = {0};
/* TOF通道3运行状态缓存 */
static TOFChannelState_t s_tof3_state = {0};
/* 当前已选融合来源 */
static uint8 s_selected_source = TOF_SRC_NONE;
/* 待确认切换的融合来源 */
static uint8 s_pending_source = TOF_SRC_NONE;
/* 待确认来源累计计数 */
static uint8 s_pending_count = 0U;
/* 融合失效保持计数 */
static uint8 s_fused_invalid_hold_count = 0U;

/*
 * 函数功能：计算两个无符号16位数的绝对差。
 * 输入参数：
 *   a - 输入值A。
 *   b - 输入值B。
 * 返回值：
 *   两者差值的绝对值。
 */
static uint16 TOF_AbsDiffU16(uint16 a, uint16 b)
{
    if (a >= b)
    {
        return (uint16)(a - b);
    }
    return (uint16)(b - a);
}

/*
 * 函数功能：对原始测距应用零偏并进行范围钳位。
 * 输入参数：
 *   raw_mm - 原始高度值，单位：mm。
 *   offset_mm - 需要叠加的通道零偏，单位：mm。
 * 返回值：
 *   补偿并钳位后的高度值，范围[0, 65535]，单位：mm。
 */
static uint16 TOF_ApplyOffsetClamp(uint16 raw_mm, int16 offset_mm)
{
    int32 corrected = (int32)raw_mm + (int32)offset_mm;

    if (corrected < 0)
    {
        corrected = 0;
    }
    else if (corrected > 65535)
    {
        corrected = 65535;
    }

    return (uint16)corrected;
}

/*
 * 函数功能：依据当前姿态将斜距补偿为机体垂向距离。
 * 输入参数：
 *   slant_mm - TOF斜距测量值，单位：mm。
 * 返回值：
 *   姿态补偿后的垂向距离，单位：mm。
 */
static uint16 TOF_CompensateVerticalByAttitude(uint16 slant_mm)
{
    float cos_term = 1.0f;
    float vertical_mm;

    if (0U != IMU_Is_Ready())
    {
        cos_term = g_euler.cos_roll * g_euler.cos_pitch;
        if (cos_term < TOF_COS_TERM_MIN)
        {
            cos_term = TOF_COS_TERM_MIN;
        }
        else if (cos_term > 1.0f)
        {
            cos_term = 1.0f;
        }
    }

    vertical_mm = (float)slant_mm * cos_term;
    if (vertical_mm < 0.0f)
    {
        vertical_mm = 0.0f;
    }
    else if (vertical_mm > 65535.0f)
    {
        vertical_mm = 65535.0f;
    }

    return (uint16)(vertical_mm + 0.5f);
}

/*
 * 函数功能：判定单次TOF原始测量是否有效。
 * 输入参数：
 *   distance_mm - 原始距离，单位：mm。
 *   range_status - VL53L1X测距状态码。
 *   ready - 传感器就绪标志。
 *   fresh - 数据新鲜标志（当前未参与判定，保留接口）。
 * 返回值：
 *   1表示有效，0表示无效。
 */
static uint8 TOF_IsRawMeasurementValid(uint16 distance_mm, uint8 range_status, uint8 ready, uint8 fresh)
{
    (void)fresh;

    if (0U == ready)
    {
        return 0U;
    }

    if (0x89U != range_status)
    {
        return 0U;
    }

    if ((distance_mm < TOF_MIN_VALID_MM) || (distance_mm > TOF_MAX_VALID_MM))
    {
        return 0U;
    }

    return 1U;
}

/*
 * 函数功能：重置单通道运行状态缓存。
 * 输入参数：
 *   state - 通道状态结构体指针。
 * 返回值：
 *   无。
 */
static void TOF_ResetChannelState(TOFChannelState_t *state)
{
    if (0 == state)
    {
        return;
    }

    state->has_last_mm = 0U;
    state->fresh_now = 0U;
    state->last_mm = 0U;
    state->invalid_streak = 0U;
    state->same_value_streak = 0U;
}

/*
 * 函数功能：重置TOF融合输出与来源状态。
 * 输入参数：
 *   无。
 * 返回值：
 *   无。
 */
static void TOF_ResetFusionState(void)
{
    g_tof2_height_mm = VL53L1X_INVALID_DISTANCE_MM;
    g_tof3_height_mm = VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_height_mm = VL53L1X_INVALID_DISTANCE_MM;
    g_tof2_valid = 0U;
    g_tof3_valid = 0U;
    g_tof_fused_valid = 0U;
    g_tof2_used_in_fusion = 0U;
    g_tof3_used_in_fusion = 0U;
    g_tof_fused_source = TOF_SRC_NONE;

    s_selected_source = TOF_SRC_NONE;
    s_pending_source = TOF_SRC_NONE;
    s_pending_count = 0U;
    s_fused_invalid_hold_count = 0U;
}

/*
 * 函数功能：根据融合来源更新“通道使用标志”与来源标识。
 * 输入参数：
 *   source - 当前融合来源，取值见TOF_SRC_*。
 * 返回值：
 *   无。
 */
static void TOF_UpdateFusionUsage(uint8 source)
{
    g_tof_fused_source = source;
    g_tof2_used_in_fusion = 0U;
    g_tof3_used_in_fusion = 0U;

    if (TOF_SRC_CH2 == source)
    {
        g_tof2_used_in_fusion = 1U;
    }
    else if (TOF_SRC_CH3 == source)
    {
        g_tof3_used_in_fusion = 1U;
    }
    else if (TOF_SRC_AVG == source)
    {
        g_tof2_used_in_fusion = 1U;
        g_tof3_used_in_fusion = 1U;
    }
}

/*
 * 函数功能：更新单个TOF通道观测，输出显示值与融合值有效性。
 * 输入参数：
 *   state - 通道状态缓存指针。
 *   raw_mm - 原始测距值，单位：mm。
 *   range_status - VL53L1X测距状态码。
 *   ready - 传感器就绪标志。
 *   fresh - 数据新鲜标志。
 *   offset_mm - 通道零偏补偿，单位：mm。
 *   display_mm - 输出显示高度指针，单位：mm。
 *   display_valid - 输出显示有效标志指针。
 *   fusion_mm - 输出融合高度指针，单位：mm。
 *   fusion_valid - 输出融合有效标志指针。
 * 返回值：
 *   1表示本次有可显示数据（新值或保持值），0表示无可显示数据。
 */
static uint8 TOF_UpdateChannelObservation(TOFChannelState_t *state,
                                          uint16 raw_mm,
                                          uint8 range_status,
                                          uint8 ready,
                                          uint8 fresh,
                                          int16 offset_mm,
                                          uint16 *display_mm,
                                          uint8 *display_valid,
                                          uint16 *fusion_mm,
                                          uint8 *fusion_valid)
{
    uint16 corrected_mm;
    uint8 raw_valid;

    if ((0 == state) || (0 == display_mm) || (0 == display_valid) || (0 == fusion_mm) || (0 == fusion_valid))
    {
        return 0U;
    }

    /* 原始数据有效：执行姿态补偿、零偏修正并更新通道状态 */
    raw_valid = TOF_IsRawMeasurementValid(raw_mm, range_status, ready, fresh);
    if (0U != raw_valid)
    {
        corrected_mm = TOF_ApplyOffsetClamp(TOF_CompensateVerticalByAttitude(raw_mm), offset_mm);

        if ((0U != state->has_last_mm) && (corrected_mm == state->last_mm))
        {
            if (state->same_value_streak < 65535U)
            {
                state->same_value_streak++;
            }
        }
        else
        {
            state->same_value_streak = 0U;
        }

        state->last_mm = corrected_mm;
        state->has_last_mm = 1U;
        state->fresh_now = fresh;
        state->invalid_streak = 0U;

        *display_mm = corrected_mm;
        *display_valid = 1U;
        *fusion_mm = corrected_mm;
        *fusion_valid = 1U;
        return 1U;
    }

    /* 原始数据无效：更新失效计数，并按策略输出保持值或无效值 */
    if (state->invalid_streak < 65535U)
    {
        state->invalid_streak++;
    }
    state->fresh_now = 0U;
    state->same_value_streak = 0U;

    *fusion_valid = 0U;
    *fusion_mm = 0U;

    if (0U != state->has_last_mm)
    {
        *display_mm = state->last_mm;
        *display_valid = 1U;
        return 1U;
    }

    *display_mm = VL53L1X_INVALID_DISTANCE_MM;
    *display_valid = 0U;
    return 0U;
}

/*
 * 函数功能：计算通道冻结惩罚分，用于融合来源选择。
 * 输入参数：
 *   state - 通道状态缓存只读指针。
 * 返回值：
 *   惩罚值，单位：mm。
 */
static uint16 TOF_ChannelFrozenPenalty(const TOFChannelState_t *state)
{
    uint16 penalty;
    uint16 exceed;

    if (0 == state)
    {
        return 0U;
    }

    if (state->same_value_streak <= TOF_SAME_VALUE_FRAMES_TH)
    {
        penalty = 0U;
    }
    else
    {
        exceed = (uint16)(state->same_value_streak - TOF_SAME_VALUE_FRAMES_TH);
        penalty = (uint16)(TOF_SAME_VALUE_PENALTY_BASE + (uint16)(exceed * TOF_SAME_VALUE_PENALTY_STEP));
        if (penalty > TOF_SAME_VALUE_PENALTY_MAX)
        {
            penalty = TOF_SAME_VALUE_PENALTY_MAX;
        }
    }

    if (0U == state->fresh_now)
    {
        penalty = (uint16)(penalty + TOF_NOT_FRESH_PENALTY);
    }

    return penalty;
}

/*
 * 函数功能：判断指定融合来源在当前帧是否可用。
 * 输入参数：
 *   source - 待判断来源，取值见TOF_SRC_*。
 *   ch2_valid - 通道2融合有效标志。
 *   ch3_valid - 通道3融合有效标志。
 *   diff_mm - 双通道差值，单位：mm。
 * 返回值：
 *   1表示可用，0表示不可用。
 */
static uint8 TOF_IsSourceAvailable(uint8 source, uint8 ch2_valid, uint8 ch3_valid, uint16 diff_mm)
{
    if (TOF_SRC_CH2 == source)
    {
        return ch2_valid;
    }

    if (TOF_SRC_CH3 == source)
    {
        return ch3_valid;
    }

    if (TOF_SRC_AVG == source)
    {
        if ((0U != ch2_valid) && (0U != ch3_valid) && (diff_mm <= TOF_AGREE_GATE_MM))
        {
            return 1U;
        }
        return 0U;
    }

    return 0U;
}

/*
 * 函数功能：根据双通道状态与参考高度选择“期望融合来源”。
 * 输入参数：
 *   ch2_valid - 通道2融合有效标志。
 *   ch2_mm - 通道2融合候选高度，单位：mm。
 *   ch3_valid - 通道3融合有效标志。
 *   ch3_mm - 通道3融合候选高度，单位：mm。
 *   ref_mm - 参考高度，单位：mm。
 *   diff_mm - 输出双通道差值指针，单位：mm，可为NULL。
 * 返回值：
 *   期望来源，取值见TOF_SRC_*。
 */
static uint8 TOF_GetDesiredSource(uint8 ch2_valid,
                                  uint16 ch2_mm,
                                  uint8 ch3_valid,
                                  uint16 ch3_mm,
                                  uint16 ref_mm,
                                  uint16 *diff_mm)
{
    uint16 innovation2;
    uint16 innovation3;
    uint16 score2;
    uint16 score3;

    if (0 != diff_mm)
    {
        *diff_mm = 0U;
    }

    if ((0U != ch2_valid) && (0U != ch3_valid))
    {
        if (0 != diff_mm)
        {
            *diff_mm = TOF_AbsDiffU16(ch2_mm, ch3_mm);
            if (*diff_mm <= TOF_AGREE_GATE_MM)
            {
                return TOF_SRC_AVG;
            }
        }

        innovation2 = TOF_AbsDiffU16(ch2_mm, ref_mm);
        innovation3 = TOF_AbsDiffU16(ch3_mm, ref_mm);

        score2 = (uint16)(innovation2 + TOF_ChannelFrozenPenalty(&s_tof2_state));
        score3 = (uint16)(innovation3 + TOF_ChannelFrozenPenalty(&s_tof3_state));
        return (score2 <= score3) ? TOF_SRC_CH2 : TOF_SRC_CH3;
    }

    if (0U != ch2_valid)
    {
        return TOF_SRC_CH2;
    }

    if (0U != ch3_valid)
    {
        return TOF_SRC_CH3;
    }

    return TOF_SRC_NONE;
}

/*
 * 函数功能：对期望来源执行去抖确认并更新当前选中来源。
 * 输入参数：
 *   desired_source - 当前帧期望来源。
 *   ch2_valid - 通道2融合有效标志。
 *   ch3_valid - 通道3融合有效标志。
 *   diff_mm - 双通道差值，单位：mm。
 * 返回值：
 *   当前最终选中的来源，取值见TOF_SRC_*。
 */
static uint8 TOF_UpdateSelectedSource(uint8 desired_source, uint8 ch2_valid, uint8 ch3_valid, uint16 diff_mm)
{
    if (TOF_SRC_NONE == desired_source)
    {
        s_selected_source = TOF_SRC_NONE;
        s_pending_source = TOF_SRC_NONE;
        s_pending_count = 0U;
        return TOF_SRC_NONE;
    }

    if (0U == TOF_IsSourceAvailable(s_selected_source, ch2_valid, ch3_valid, diff_mm))
    {
        s_selected_source = desired_source;
        s_pending_source = TOF_SRC_NONE;
        s_pending_count = 0U;
        return s_selected_source;
    }

    if (desired_source == s_selected_source)
    {
        s_pending_source = TOF_SRC_NONE;
        s_pending_count = 0U;
        return s_selected_source;
    }

    if (desired_source != s_pending_source)
    {
        s_pending_source = desired_source;
        s_pending_count = 1U;
        return s_selected_source;
    }

    if (s_pending_count < 255U)
    {
        s_pending_count++;
    }

    if (s_pending_count >= TOF_SWITCH_CONFIRM_FRAMES)
    {
        s_selected_source = desired_source;
        s_pending_source = TOF_SRC_NONE;
        s_pending_count = 0U;
    }

    return s_selected_source;
}

/*
 * 函数功能：按来源类型提取本帧源高度。
 * 输入参数：
 *   source - 来源类型，取值见TOF_SRC_*。
 *   ch2_mm - 通道2高度，单位：mm。
 *   ch3_mm - 通道3高度，单位：mm。
 * 返回值：
 *   来源对应高度，单位：mm；来源无效时返回0。
 */
static uint16 TOF_GetSourceHeight(uint8 source, uint16 ch2_mm, uint16 ch3_mm)
{
    if (TOF_SRC_CH2 == source)
    {
        return ch2_mm;
    }

    if (TOF_SRC_CH3 == source)
    {
        return ch3_mm;
    }

    if (TOF_SRC_AVG == source)
    {
        return (uint16)(((uint32)ch2_mm + (uint32)ch3_mm) / 2U);
    }

    return 0U;
}

/*
 * 函数功能：初始化双TOF模块并执行零偏标定。
 * 输入参数：
 *   无。
 * 返回值：
 *   无。
 */
void TOF_Init(void)
{
    uint8 init_err;

    s_tof2_offset_mm = 0;
    s_tof3_offset_mm = 0;
    TOF_ResetChannelState(&s_tof2_state);
    TOF_ResetChannelState(&s_tof3_state);
    TOF_ResetFusionState();

    /* 初始化双TOF硬件，任一通道成功即认为模块可运行 */
    init_err = VL53L1X_init_all();
    s_tof_inited = (init_err == (TOF_CH2_VALID_MASK | TOF_CH3_VALID_MASK)) ? 0U : 1U;
    if (0U == s_tof_inited)
    {
        return;
    }

    /* 初始化后执行一次静态标定，统一双通道零偏中心 */
    TOF_Calibrate();
}

/*
 * 函数功能：执行TOF静态标定，估计双通道零偏并对齐到共同中心。
 * 输入参数：
 *   无。
 * 返回值：
 *   无。
 */
void TOF_Calibrate(void)
{
    uint32 i;
    uint32 sum2 = 0U;
    uint32 sum3 = 0U;
    uint32 cnt2 = 0U;
    uint32 cnt3 = 0U;
    uint8 valid_mask;
    uint8 raw2_valid;
    uint8 raw3_valid;
    int32 mean2;
    int32 mean3;
    int32 center;
    VL53L1X_data_struct data;

    if (0U == s_tof_inited)
    {
        return;
    }

    s_tof2_offset_mm = 0;
    s_tof3_offset_mm = 0;
    TOF_ResetChannelState(&s_tof2_state);
    TOF_ResetChannelState(&s_tof3_state);
    TOF_ResetFusionState();

    /* 按固定采样窗口统计双通道有效垂向距离均值 */
    for (i = 0U; i < TOF_CALIBRATION_SAMPLES; ++i)
    {
        valid_mask = VL53L1X_read_data(&data);
        (void)valid_mask;

        raw2_valid = TOF_IsRawMeasurementValid(data.VL53L1X2_distance_mm,
                                               data.VL53L1X2_range_status,
                                               g_vl53l1x2_diag.ready,
                                               g_vl53l1x2_diag.is_fresh);
        raw3_valid = TOF_IsRawMeasurementValid(data.VL53L1X3_distance_mm,
                                               data.VL53L1X3_range_status,
                                               g_vl53l1x3_diag.ready,
                                               g_vl53l1x3_diag.is_fresh);

        if (0U != raw2_valid)
        {
            sum2 += TOF_CompensateVerticalByAttitude(data.VL53L1X2_distance_mm);
            cnt2++;
        }

        if (0U != raw3_valid)
        {
            sum3 += TOF_CompensateVerticalByAttitude(data.VL53L1X3_distance_mm);
            cnt3++;
        }

        system_delay_ms(TOF_CALIBRATION_DT_MS);
    }

    /* 双通道均有有效样本时，按中心对齐计算各自零偏 */
    if ((cnt2 > 0U) && (cnt3 > 0U))
    {
        mean2 = (int32)(sum2 / cnt2);
        mean3 = (int32)(sum3 / cnt3);
        center = (mean2 + mean3) / 2;
        s_tof2_offset_mm = (int16)(center - mean2);
        s_tof3_offset_mm = (int16)(center - mean3);
    }

    TOF_ResetChannelState(&s_tof2_state);
    TOF_ResetChannelState(&s_tof3_state);
    TOF_ResetFusionState();
}

/*
 * 函数功能：更新双TOF观测并输出融合高度结果。
 * 输入参数：
 *   无。
 * 返回值：
 *   无。
 */
void TOF_Update(void)
{
    uint8 valid_mask;
    uint8 ch2_display_valid;
    uint8 ch3_display_valid;
    uint8 ch2_fusion_valid;
    uint8 ch3_fusion_valid;
    uint8 desired_source;
    uint8 selected_source;
    uint16 ch2_display_mm;
    uint16 ch3_display_mm;
    uint16 ch2_fusion_mm;
    uint16 ch3_fusion_mm;
    uint16 source_mm;
    uint16 ref_mm;
    uint16 diff_mm;
    uint16 step_limit;
    int32 fused_delta;
    int32 fused_next;
    VL53L1X_data_struct data;

    if (0U == s_tof_inited)
    {
        g_tof2_valid = 0U;
        g_tof3_valid = 0U;
        g_tof_fused_valid = 0U;
        TOF_UpdateFusionUsage(TOF_SRC_NONE);
        return;
    }

    /* 读取双通道最新原始数据并更新单通道观测状态 */
    valid_mask = VL53L1X_read_data(&data);
    (void)valid_mask;

    TOF_UpdateChannelObservation(&s_tof2_state,
                                 data.VL53L1X2_distance_mm,
                                 data.VL53L1X2_range_status,
                                 g_vl53l1x2_diag.ready,
                                 g_vl53l1x2_diag.is_fresh,
                                 s_tof2_offset_mm,
                                 &ch2_display_mm,
                                 &ch2_display_valid,
                                 &ch2_fusion_mm,
                                 &ch2_fusion_valid);

    TOF_UpdateChannelObservation(&s_tof3_state,
                                 data.VL53L1X3_distance_mm,
                                 data.VL53L1X3_range_status,
                                 g_vl53l1x3_diag.ready,
                                 g_vl53l1x3_diag.is_fresh,
                                 s_tof3_offset_mm,
                                 &ch3_display_mm,
                                 &ch3_display_valid,
                                 &ch3_fusion_mm,
                                 &ch3_fusion_valid);

    g_tof2_height_mm = (0U != ch2_display_valid) ? ch2_display_mm : VL53L1X_INVALID_DISTANCE_MM;
    g_tof3_height_mm = (0U != ch3_display_valid) ? ch3_display_mm : VL53L1X_INVALID_DISTANCE_MM;
    g_tof2_valid = ch2_fusion_valid;
    g_tof3_valid = ch3_fusion_valid;

    /* 生成本帧参考高度，优先沿用上一帧融合值 */
    if (0U != g_tof_fused_valid)
    {
        ref_mm = g_tof_fused_height_mm;
    }
    else if ((0U != ch2_fusion_valid) && (0U != ch3_fusion_valid))
    {
        ref_mm = (uint16)(((uint32)ch2_fusion_mm + (uint32)ch3_fusion_mm) / 2U);
    }
    else if (0U != ch2_fusion_valid)
    {
        ref_mm = ch2_fusion_mm;
    }
    else
    {
        ref_mm = ch3_fusion_mm;
    }

    /* 基于一致性、创新量与冻结惩罚选择期望来源，并经过去抖确认 */
    desired_source = TOF_GetDesiredSource(ch2_fusion_valid,
                                          ch2_fusion_mm,
                                          ch3_fusion_valid,
                                          ch3_fusion_mm,
                                          ref_mm,
                                          &diff_mm);

    selected_source = TOF_UpdateSelectedSource(desired_source, ch2_fusion_valid, ch3_fusion_valid, diff_mm);
    /* 无可用来源：短时保持，超时后输出无效 */
    if (TOF_SRC_NONE == selected_source)
    {
        if ((0U != g_tof_fused_valid) && (s_fused_invalid_hold_count < TOF_FUSED_HOLD_FRAMES))
        {
            s_fused_invalid_hold_count++;
            TOF_UpdateFusionUsage(TOF_SRC_NONE);
            return;
        }

        g_tof_fused_valid = 0U;
        g_tof_fused_height_mm = VL53L1X_INVALID_DISTANCE_MM;
        TOF_UpdateFusionUsage(TOF_SRC_NONE);
        return;
    }

    s_fused_invalid_hold_count = 0U;
    source_mm = TOF_GetSourceHeight(selected_source, ch2_fusion_mm, ch3_fusion_mm);

    /* 首次获得有效来源时直接建立融合高度 */
    if (0U == g_tof_fused_valid)
    {
        g_tof_fused_height_mm = source_mm;
        g_tof_fused_valid = 1U;
        TOF_UpdateFusionUsage(selected_source);
        return;
    }

    /* 按双通道一致性选择融合步进限幅，抑制突变 */
    if ((0U != ch2_fusion_valid) && (0U != ch3_fusion_valid))
    {
        if (diff_mm <= TOF_AGREE_GATE_MM)
        {
            step_limit = TOF_STEP_FAST_MM;
        }
        else if (diff_mm <= TOF_HARD_MISMATCH_GATE_MM)
        {
            step_limit = TOF_STEP_SAFE_MM;
        }
        else
        {
            step_limit = TOF_STEP_HARD_MM;
        }
    }
    else
    {
        step_limit = TOF_STEP_SAFE_MM;
    }

    fused_delta = (int32)source_mm - (int32)g_tof_fused_height_mm;
    if (fused_delta > (int32)step_limit)
    {
        fused_delta = (int32)step_limit;
    }
    else if (fused_delta < -(int32)step_limit)
    {
        fused_delta = -(int32)step_limit;
    }

    fused_next = (int32)g_tof_fused_height_mm + fused_delta;
    if (fused_next < 0)
    {
        fused_next = 0;
    }
    else if (fused_next > 65535)
    {
        fused_next = 65535;
    }

    g_tof_fused_height_mm = (uint16)fused_next;
    g_tof_fused_valid = 1U;
    TOF_UpdateFusionUsage(selected_source);
}

/*
 * 函数功能：100Hz任务入口，更新TOF融合高度。
 * 输入参数：
 *   无。
 * 返回值：
 *   无。
 */
void TOF_update_100HZ(void)
{
    TOF_Update();
}
