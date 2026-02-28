#include "TOF_data.h"
#include "zf_common_headfile.h"

#define TOF_CH2_VALID_MASK               (1U << 0)
#define TOF_CH3_VALID_MASK               (1U << 1)

#define TOF_MIN_VALID_MM                 (40U)
#define TOF_MAX_VALID_MM                 (1300U)
#define TOF_COS_TERM_MIN                 (0.35f)

#define TOF_AGREE_GATE_MM                (40U)
#define TOF_SWITCH_CONFIRM_FRAMES        (3U)
#define TOF_FUSED_HOLD_FRAMES            (4U)
#define TOF_STEP_FAST_MM                 (28U)
#define TOF_STEP_SAFE_MM                 (18U)

#define TOF_SAME_VALUE_FRAMES_TH         (25U)
#define TOF_SAME_VALUE_PENALTY_BASE      (120U)
#define TOF_SAME_VALUE_PENALTY_STEP      (2U)
#define TOF_SAME_VALUE_PENALTY_MAX       (400U)
#define TOF_NOT_FRESH_PENALTY            (80U)

#define TOF_CALIBRATION_SAMPLES          (100U)
#define TOF_CALIBRATION_DT_MS            (10U)

#define TOF_SRC_NONE                     (0U)
#define TOF_SRC_CH2                      (1U)
#define TOF_SRC_CH3                      (2U)
#define TOF_SRC_AVG                      (3U)

typedef struct
{
    uint8 has_last_mm;
    uint8 fresh_now;
    uint16 last_mm;
    uint16 invalid_streak;
    uint16 same_value_streak;
} TOFChannelState_t;

uint16 g_tof_fused_height_mm = VL53L1X_INVALID_DISTANCE_MM;
uint16 g_tof2_height_mm = VL53L1X_INVALID_DISTANCE_MM;
uint16 g_tof3_height_mm = VL53L1X_INVALID_DISTANCE_MM;
uint8 g_tof_fused_valid = 0U;
uint8 g_tof2_valid = 0U;
uint8 g_tof3_valid = 0U;
uint8 g_tof2_used_in_fusion = 0U;
uint8 g_tof3_used_in_fusion = 0U;
uint8 g_tof_fused_source = TOF_SRC_NONE;

static uint8 s_tof_inited = 0U;
static int16 s_tof2_offset_mm = 0;
static int16 s_tof3_offset_mm = 0;

static TOFChannelState_t s_tof2_state = {0};
static TOFChannelState_t s_tof3_state = {0};
static uint8 s_selected_source = TOF_SRC_NONE;
static uint8 s_pending_source = TOF_SRC_NONE;
static uint8 s_pending_count = 0U;
static uint8 s_fused_invalid_hold_count = 0U;

static uint16 TOF_AbsDiffU16(uint16 a, uint16 b)
{
    if (a >= b)
    {
        return (uint16)(a - b);
    }
    return (uint16)(b - a);
}

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

void TOF_Init(void)
{
    uint8 init_err;

    s_tof2_offset_mm = 0;
    s_tof3_offset_mm = 0;
    TOF_ResetChannelState(&s_tof2_state);
    TOF_ResetChannelState(&s_tof3_state);
    TOF_ResetFusionState();

    init_err = VL53L1X_init_all();
    s_tof_inited = (init_err == (TOF_CH2_VALID_MASK | TOF_CH3_VALID_MASK)) ? 0U : 1U;
    if (0U == s_tof_inited)
    {
        return;
    }

    TOF_Calibrate();
}

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

    desired_source = TOF_GetDesiredSource(ch2_fusion_valid,
                                          ch2_fusion_mm,
                                          ch3_fusion_valid,
                                          ch3_fusion_mm,
                                          ref_mm,
                                          &diff_mm);

    selected_source = TOF_UpdateSelectedSource(desired_source, ch2_fusion_valid, ch3_fusion_valid, diff_mm);
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

    if (0U == g_tof_fused_valid)
    {
        g_tof_fused_height_mm = source_mm;
        g_tof_fused_valid = 1U;
        TOF_UpdateFusionUsage(selected_source);
        return;
    }

    if ((0U != ch2_fusion_valid) && (0U != ch3_fusion_valid) && (diff_mm <= TOF_AGREE_GATE_MM))
    {
        step_limit = TOF_STEP_FAST_MM;
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

void TOF_update_100HZ(void)
{
    TOF_Update();
}
