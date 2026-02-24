#include "TOF_data.h"
#include "../Attitude/IMU_TOP.h"

#define TOF_CH2_VALID_MASK               (1U << 0)
#define TOF_CH3_VALID_MASK               (1U << 1)

#define TOF_MIN_VALID_MM                 (40U)
#define TOF_MAX_VALID_MM                 (1300U)
#define TOF_JUMP_REJECT_MM               (120U)
#define TOF_STEP_LIMIT_MM                (80U)
#define TOF_LPF_ALPHA                    (0.45f)
#define TOF_MATCH_GATE_MM                (100U)
#define TOF_INVALID_HOLD_FRAMES          (4U)
#define TOF_SWITCH_CONFIRM_FRAMES        (2U)
#define TOF_COS_TERM_MIN                 (0.35f)
#define TOF_RAW_SAME_STREAK_TH           (30U)
#define TOF_RAW_SAME_STREAK_PENALTY      (250U)

#define TOF_CALIBRATION_SAMPLES          (100U)
#define TOF_CALIBRATION_DT_MS            (10U)

#define TOF_SRC_NONE                     (0U)
#define TOF_SRC_CH2                      (1U)
#define TOF_SRC_CH3                      (2U)
#define TOF_SRC_AVG                      (3U)

typedef struct
{
    uint8 has_value;
    float value_mm;

    uint8 has_last_raw;
    uint16 last_raw_mm;

    uint16 invalid_streak;
    uint16 stable_valid_cnt;
    uint16 last_delta_mm;
    uint16 raw_same_streak;

    uint8 raw_valid_now;
    uint8 fresh_now;
    uint8 eligible_for_fusion;
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

static uint8 TOF_IsRawMeasurementValid(uint16 distance_mm, uint8 range_status, uint8 ready)
{
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
    state->has_value = 0U;
    state->value_mm = 0.0f;
    state->has_last_raw = 0U;
    state->last_raw_mm = 0U;
    state->invalid_streak = 0U;
    state->stable_valid_cnt = 0U;
    state->last_delta_mm = 0U;
    state->raw_same_streak = 0U;
    state->raw_valid_now = 0U;
    state->fresh_now = 0U;
    state->eligible_for_fusion = 0U;
}

static void TOF_ResetAllStates(void)
{
    TOF_ResetChannelState(&s_tof2_state);
    TOF_ResetChannelState(&s_tof3_state);

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
}

static uint8 TOF_UpdateChannelState(TOFChannelState_t *state,
                                    uint8 raw_valid,
                                    uint8 raw_fresh,
                                    uint16 raw_mm,
                                    int16 offset_mm,
                                    uint16 *output_mm)
{
    uint16 candidate_mm;
    uint16 corrected_mm;
    uint16 prev_output_mm;
    int32 delta_raw;
    int32 candidate_i32;
    float prev_value;
    float target_value;

    if ((0 == state) || (0 == output_mm))
    {
        return 0U;
    }

    state->raw_valid_now = raw_valid;
    state->fresh_now = raw_fresh;
    state->eligible_for_fusion = 0U;

    if (0U == raw_valid)
    {
        if (state->invalid_streak < 65535U)
        {
            state->invalid_streak++;
        }
        state->raw_same_streak = 0U;
        if (state->stable_valid_cnt > 0U)
        {
            state->stable_valid_cnt--;
        }

        if ((state->has_value != 0U) && (state->invalid_streak <= TOF_INVALID_HOLD_FRAMES))
        {
            *output_mm = (uint16)(state->value_mm + 0.5f);
            state->last_delta_mm = 0U;
            state->eligible_for_fusion = (state->stable_valid_cnt > 0U) ? 1U : 0U;
            return 1U;
        }

        return 0U;
    }

    if (0U == raw_fresh)
    {
        state->invalid_streak = 0U;
        state->stable_valid_cnt = 0U;
        state->raw_same_streak = 0U;

        if (state->has_value != 0U)
        {
            *output_mm = (uint16)(state->value_mm + 0.5f);
            state->last_delta_mm = 0U;
            return 1U;
        }

        return 0U;
    }

    state->invalid_streak = 0U;
    if (state->stable_valid_cnt < 65535U)
    {
        state->stable_valid_cnt++;
    }

    corrected_mm = TOF_ApplyOffsetClamp(TOF_CompensateVerticalByAttitude(raw_mm), offset_mm);
    candidate_mm = corrected_mm;

    if (0U == state->has_last_raw)
    {
        state->last_raw_mm = corrected_mm;
        state->has_last_raw = 1U;
        state->raw_same_streak = 0U;
    }
    else
    {
        if (corrected_mm == state->last_raw_mm)
        {
            if (state->raw_same_streak < 65535U)
            {
                state->raw_same_streak++;
            }
        }
        else
        {
            state->raw_same_streak = 0U;
        }

        delta_raw = (int32)corrected_mm - (int32)state->last_raw_mm;
        if (delta_raw > (int32)TOF_JUMP_REJECT_MM)
        {
            delta_raw = (int32)TOF_STEP_LIMIT_MM;
        }
        else if (delta_raw < -(int32)TOF_JUMP_REJECT_MM)
        {
            delta_raw = -(int32)TOF_STEP_LIMIT_MM;
        }

        candidate_i32 = (int32)state->last_raw_mm + delta_raw;
        if (candidate_i32 < 0)
        {
            candidate_i32 = 0;
        }
        else if (candidate_i32 > 65535)
        {
            candidate_i32 = 65535;
        }

        candidate_mm = (uint16)candidate_i32;
        state->last_raw_mm = candidate_mm;
    }

    if (0U == state->has_value)
    {
        state->value_mm = (float)candidate_mm;
        state->has_value = 1U;
        state->last_delta_mm = 0U;
        state->eligible_for_fusion = 1U;
        *output_mm = candidate_mm;
        return 1U;
    }

    prev_output_mm = (uint16)(state->value_mm + 0.5f);
    if (candidate_mm > (uint16)(prev_output_mm + TOF_STEP_LIMIT_MM))
    {
        candidate_mm = (uint16)(prev_output_mm + TOF_STEP_LIMIT_MM);
    }
    else if (prev_output_mm > (uint16)(candidate_mm + TOF_STEP_LIMIT_MM))
    {
        candidate_mm = (uint16)(prev_output_mm - TOF_STEP_LIMIT_MM);
    }

    prev_value = state->value_mm;
    target_value = (float)candidate_mm;
    state->value_mm = prev_value + TOF_LPF_ALPHA * (target_value - prev_value);

    *output_mm = (uint16)(state->value_mm + 0.5f);
    state->last_delta_mm = TOF_AbsDiffU16(*output_mm, prev_output_mm);
    state->eligible_for_fusion = 1U;
    return 1U;
}

static uint16 TOF_ChannelScore(const TOFChannelState_t *state)
{
    uint16 score = 0U;
    uint16 stable_penalty = 0U;
    uint16 delta_penalty = state->last_delta_mm;
    uint16 same_streak_penalty = 0U;

    if (delta_penalty > 400U)
    {
        delta_penalty = 400U;
    }

    if (state->stable_valid_cnt < 3U)
    {
        stable_penalty = (uint16)((3U - state->stable_valid_cnt) * 30U);
    }

    score = (uint16)(delta_penalty + stable_penalty);

    if (state->raw_same_streak > TOF_RAW_SAME_STREAK_TH)
    {
        same_streak_penalty = (uint16)(state->raw_same_streak - TOF_RAW_SAME_STREAK_TH);
        if (same_streak_penalty > 100U)
        {
            same_streak_penalty = 100U;
        }
        score = (uint16)(score + TOF_RAW_SAME_STREAK_PENALTY + (uint16)(same_streak_penalty * 2U));
    }

    if (0U == state->raw_valid_now)
    {
        score = (uint16)(score + 1000U);
    }

    if (0U == state->fresh_now)
    {
        score = (uint16)(score + 500U);
    }

    if (state->invalid_streak > 20U)
    {
        score = (uint16)(score + 400U);
    }
    else
    {
        score = (uint16)(score + (uint16)(state->invalid_streak * 20U));
    }

    return score;
}

static uint8 TOF_GetDesiredSource(uint8 ch2_eligible, uint8 ch3_eligible, uint16 diff_mm)
{
    uint16 score2;
    uint16 score3;

    if ((0U != ch2_eligible) && (0U != ch3_eligible))
    {
        if (diff_mm <= TOF_MATCH_GATE_MM)
        {
            return TOF_SRC_AVG;
        }

        score2 = TOF_ChannelScore(&s_tof2_state);
        score3 = TOF_ChannelScore(&s_tof3_state);
        return (score2 <= score3) ? TOF_SRC_CH2 : TOF_SRC_CH3;
    }

    if (0U != ch2_eligible)
    {
        return TOF_SRC_CH2;
    }

    if (0U != ch3_eligible)
    {
        return TOF_SRC_CH3;
    }

    return TOF_SRC_NONE;
}

static uint8 TOF_IsSourceAvailable(uint8 source, uint8 ch2_eligible, uint8 ch3_eligible, uint16 diff_mm)
{
    if (TOF_SRC_CH2 == source)
    {
        return ch2_eligible;
    }

    if (TOF_SRC_CH3 == source)
    {
        return ch3_eligible;
    }

    if (TOF_SRC_AVG == source)
    {
        if ((0U != ch2_eligible) && (0U != ch3_eligible) && (diff_mm <= TOF_MATCH_GATE_MM))
        {
            return 1U;
        }
        return 0U;
    }

    return 0U;
}

static uint8 TOF_UpdateSelectedSource(uint8 desired_source, uint8 ch2_eligible, uint8 ch3_eligible, uint16 diff_mm)
{
    if (TOF_SRC_NONE == desired_source)
    {
        s_selected_source = TOF_SRC_NONE;
        s_pending_source = TOF_SRC_NONE;
        s_pending_count = 0U;
        return TOF_SRC_NONE;
    }

    if (0U == TOF_IsSourceAvailable(s_selected_source, ch2_eligible, ch3_eligible, diff_mm))
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

static uint16 TOF_GetFusedHeightBySource(uint8 source)
{
    if (TOF_SRC_CH2 == source)
    {
        return g_tof2_height_mm;
    }

    if (TOF_SRC_CH3 == source)
    {
        return g_tof3_height_mm;
    }

    if (TOF_SRC_AVG == source)
    {
        return (uint16)(((uint32)g_tof2_height_mm + (uint32)g_tof3_height_mm) / 2U);
    }

    return g_tof_fused_height_mm;
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

void TOF_Init(void)
{
    uint8 init_err;

    s_tof2_offset_mm = 0;
    s_tof3_offset_mm = 0;
    TOF_ResetAllStates();

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
    int32 mean2;
    int32 mean3;
    int32 center;
    uint8 valid_mask;
    uint8 raw2_valid;
    uint8 raw3_valid;
    VL53L1X_data_struct data;

    if (0U == s_tof_inited)
    {
        return;
    }

    s_tof2_offset_mm = 0;
    s_tof3_offset_mm = 0;
    TOF_ResetAllStates();

    for (i = 0U; i < TOF_CALIBRATION_SAMPLES; ++i)
    {
        valid_mask = VL53L1X_read_data(&data);
        (void)valid_mask;

        raw2_valid = TOF_IsRawMeasurementValid(data.VL53L1X2_distance_mm, data.VL53L1X2_range_status, g_vl53l1x2_diag.ready);
        raw3_valid = TOF_IsRawMeasurementValid(data.VL53L1X3_distance_mm, data.VL53L1X3_range_status, g_vl53l1x3_diag.ready);

        if (0U != raw2_valid)
        {
            sum2 += TOF_CompensateVerticalByAttitude(data.VL53L1X2_distance_mm);
            ++cnt2;
        }

        if (0U != raw3_valid)
        {
            sum3 += TOF_CompensateVerticalByAttitude(data.VL53L1X3_distance_mm);
            ++cnt3;
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
}

void TOF_Update(void)
{
    uint8 valid_mask;
    uint8 raw2_valid;
    uint8 raw3_valid;
    uint8 raw2_fresh;
    uint8 raw3_fresh;
    uint8 ch2_display;
    uint8 ch3_display;
    uint8 ch2_eligible;
    uint8 ch3_eligible;
    uint8 desired_source;
    uint8 selected_source;
    uint16 diff_mm = 0U;
    uint16 ch2_mm = 0U;
    uint16 ch3_mm = 0U;
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

    raw2_valid = TOF_IsRawMeasurementValid(data.VL53L1X2_distance_mm, data.VL53L1X2_range_status, g_vl53l1x2_diag.ready);
    raw3_valid = TOF_IsRawMeasurementValid(data.VL53L1X3_distance_mm, data.VL53L1X3_range_status, g_vl53l1x3_diag.ready);
    raw2_fresh = g_vl53l1x2_diag.is_fresh;
    raw3_fresh = g_vl53l1x3_diag.is_fresh;

    ch2_display = TOF_UpdateChannelState(&s_tof2_state, raw2_valid, raw2_fresh, data.VL53L1X2_distance_mm, s_tof2_offset_mm, &ch2_mm);
    ch3_display = TOF_UpdateChannelState(&s_tof3_state, raw3_valid, raw3_fresh, data.VL53L1X3_distance_mm, s_tof3_offset_mm, &ch3_mm);

    if (0U != ch2_display)
    {
        g_tof2_height_mm = ch2_mm;
    }
    else if (0U == s_tof2_state.has_value)
    {
        g_tof2_height_mm = VL53L1X_INVALID_DISTANCE_MM;
    }

    if (0U != ch3_display)
    {
        g_tof3_height_mm = ch3_mm;
    }
    else if (0U == s_tof3_state.has_value)
    {
        g_tof3_height_mm = VL53L1X_INVALID_DISTANCE_MM;
    }

    ch2_eligible = s_tof2_state.eligible_for_fusion;
    ch3_eligible = s_tof3_state.eligible_for_fusion;
    g_tof2_valid = ch2_eligible;
    g_tof3_valid = ch3_eligible;

    if ((0U != ch2_eligible) && (0U != ch3_eligible))
    {
        diff_mm = TOF_AbsDiffU16(g_tof2_height_mm, g_tof3_height_mm);
    }

    desired_source = TOF_GetDesiredSource(ch2_eligible, ch3_eligible, diff_mm);
    selected_source = TOF_UpdateSelectedSource(desired_source, ch2_eligible, ch3_eligible, diff_mm);

    if (TOF_SRC_NONE == selected_source)
    {
        g_tof_fused_valid = 0U;
        TOF_UpdateFusionUsage(TOF_SRC_NONE);
        return;
    }

    g_tof_fused_height_mm = TOF_GetFusedHeightBySource(selected_source);
    g_tof_fused_valid = 1U;
    TOF_UpdateFusionUsage(selected_source);
}
