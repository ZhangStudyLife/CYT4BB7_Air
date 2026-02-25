#include "Pos_Est.h"

#include "../Height_Est/Height_Est.h"
#include "Accel_Calibration.h"
#include "../../HW_Drivers/PMW3901/PMW3901.h"

typedef struct
{
    float position_x_m;
    float position_y_m;
    float velocity_x_mps;
    float velocity_y_mps;
    float acceleration_x_mps2;
    float acceleration_y_mps2;
    float acceleration_bias_x_mps2;
    float acceleration_bias_y_mps2;
    float measured_position_x_m;
    float measured_position_y_m;
    float flow_height_used_m;
    float flow_translation_lpf_x_rps;
    float flow_translation_lpf_y_rps;
    float flow_elapsed_seconds;
    float flow_missing_seconds;
    uint8_t flow_height_ready;
    uint8_t flow_translation_lpf_ready;
    uint8_t flow_reference_ready;
} pos_est_state_t;

volatile pos_est_output_t g_pos_est_output = {0};
volatile pos_est_debug_t g_pos_est_debug = {0};
volatile velocity_input_100hz_t g_velocity_input_100hz = {0};
static pos_est_state_t s_pos_est_state = {0};

static uint8_t pos_est_abs_i16_over_limit(int16_t value, int16_t limit)
{
    int32_t abs_value = (int32_t)value;
    if (abs_value < 0)
    {
        abs_value = -abs_value;
    }
    return (uint8_t)(abs_value > (int32_t)limit);
}

static float pos_est_clampf(float value, float min_value, float max_value)
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

static float pos_est_absf(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }
    return value;
}

static float pos_est_apply_deadband(float value, float deadband)
{
    if (pos_est_absf(value) < deadband)
    {
        return 0.0f;
    }
    return value;
}

static uint8_t pos_est_flow_dt_valid(float flow_dt_seconds)
{
    if (flow_dt_seconds < POS_EST_FLOW_VALID_DT_MIN_SECONDS)
    {
        return 0U;
    }
    if (flow_dt_seconds > POS_EST_FLOW_VALID_DT_MAX_SECONDS)
    {
        return 0U;
    }
    return 1U;
}

static void pos_est_reset_flow_debug_measurement(void)
{
    g_pos_est_debug.flow_mapped_delta_x_count = 0.0f;
    g_pos_est_debug.flow_mapped_delta_y_count = 0.0f;
    g_pos_est_debug.flow_rate_x_rps = 0.0f;
    g_pos_est_debug.flow_rate_y_rps = 0.0f;
    g_pos_est_debug.flow_gyro_x_rps = 0.0f;
    g_pos_est_debug.flow_gyro_y_rps = 0.0f;
    g_pos_est_debug.flow_translation_x_rps = 0.0f;
    g_pos_est_debug.flow_translation_y_rps = 0.0f;
    g_pos_est_debug.measured_velocity_x_mps = 0.0f;
    g_pos_est_debug.measured_velocity_y_mps = 0.0f;
    g_pos_est_debug.position_residual_x_m = 0.0f;
    g_pos_est_debug.position_residual_y_m = 0.0f;
}

static float pos_est_update_flow_height_used(float height_m, float dt_seconds, uint8_t reinit)
{
    const float height_clamped_m = pos_est_clampf(
        height_m,
        POS_EST_FLOW_HEIGHT_MIN_M,
        POS_EST_FLOW_HEIGHT_MAX_M);
    float alpha;

    if ((reinit != 0U) || (s_pos_est_state.flow_height_ready == 0U))
    {
        s_pos_est_state.flow_height_used_m = height_clamped_m;
        s_pos_est_state.flow_height_ready = 1U;
    }
    else
    {
        alpha = dt_seconds / (POS_EST_FLOW_HEIGHT_LPF_TAU_SECONDS + dt_seconds);
        s_pos_est_state.flow_height_used_m +=
            alpha * (height_clamped_m - s_pos_est_state.flow_height_used_m);
        s_pos_est_state.flow_height_used_m = pos_est_clampf(
            s_pos_est_state.flow_height_used_m,
            POS_EST_FLOW_HEIGHT_MIN_M,
            POS_EST_FLOW_HEIGHT_MAX_M);
    }

    return s_pos_est_state.flow_height_used_m;
}

static void pos_est_limit_state(void)
{
    s_pos_est_state.velocity_x_mps = pos_est_clampf(
        s_pos_est_state.velocity_x_mps,
        -POS_EST_VELOCITY_LIMIT_MPS,
        POS_EST_VELOCITY_LIMIT_MPS);

    s_pos_est_state.velocity_y_mps = pos_est_clampf(
        s_pos_est_state.velocity_y_mps,
        -POS_EST_VELOCITY_LIMIT_MPS,
        POS_EST_VELOCITY_LIMIT_MPS);

    s_pos_est_state.acceleration_x_mps2 = pos_est_clampf(
        s_pos_est_state.acceleration_x_mps2,
        -POS_EST_ACCELERATION_LIMIT_MPS2,
        POS_EST_ACCELERATION_LIMIT_MPS2);

    s_pos_est_state.acceleration_y_mps2 = pos_est_clampf(
        s_pos_est_state.acceleration_y_mps2,
        -POS_EST_ACCELERATION_LIMIT_MPS2,
        POS_EST_ACCELERATION_LIMIT_MPS2);
}

static uint8_t pos_est_height_valid(void)
{
    if (g_height_est_source == HEIGHT_EST_SOURCE_NONE)
    {
        return 0U;
    }
    if (g_height_est_valid == 0U)
    {
        return 0U;
    }
    if (g_height_est_m <= 0.0f)
    {
        return 0U;
    }
    return 1U;
}

void pos_est_init(void)
{
    s_pos_est_state.position_x_m = 0.0f;
    s_pos_est_state.position_y_m = 0.0f;
    s_pos_est_state.velocity_x_mps = 0.0f;
    s_pos_est_state.velocity_y_mps = 0.0f;
    s_pos_est_state.acceleration_x_mps2 = 0.0f;
    s_pos_est_state.acceleration_y_mps2 = 0.0f;
    s_pos_est_state.acceleration_bias_x_mps2 = 0.0f;
    s_pos_est_state.acceleration_bias_y_mps2 = 0.0f;
    s_pos_est_state.measured_position_x_m = 0.0f;
    s_pos_est_state.measured_position_y_m = 0.0f;
    s_pos_est_state.flow_height_used_m = 0.0f;
    s_pos_est_state.flow_translation_lpf_x_rps = 0.0f;
    s_pos_est_state.flow_translation_lpf_y_rps = 0.0f;
    s_pos_est_state.flow_elapsed_seconds = 0.0f;
    s_pos_est_state.flow_missing_seconds = 0.0f;
    s_pos_est_state.flow_height_ready = 0U;
    s_pos_est_state.flow_translation_lpf_ready = 0U;
    s_pos_est_state.flow_reference_ready = 0U;

    g_pos_est_output.position_x_m = 0.0f;
    g_pos_est_output.position_y_m = 0.0f;
    g_pos_est_output.velocity_x_mps = 0.0f;
    g_pos_est_output.velocity_y_mps = 0.0f;
    g_pos_est_output.acceleration_x_mps2 = 0.0f;
    g_pos_est_output.acceleration_y_mps2 = 0.0f;
    g_pos_est_output.height_m = 0.0f;
    g_pos_est_output.flow_update_valid = 0U;

    g_velocity_input_100hz.velocity_x_mps = 0.0f;
    g_velocity_input_100hz.velocity_y_mps = 0.0f;
    g_velocity_input_100hz.valid = 0U;

    g_pos_est_debug.acceleration_input_x_mps2 = 0.0f;
    g_pos_est_debug.acceleration_input_y_mps2 = 0.0f;
    g_pos_est_debug.acceleration_bias_x_mps2 = 0.0f;
    g_pos_est_debug.acceleration_bias_y_mps2 = 0.0f;
    g_pos_est_debug.body_acceleration_x_mps2 = 0.0f;
    g_pos_est_debug.body_acceleration_y_mps2 = 0.0f;
    g_pos_est_debug.body_gyro_x_dps = 0.0f;
    g_pos_est_debug.body_gyro_y_dps = 0.0f;
    g_pos_est_debug.is_static_state = 0U;
    g_pos_est_debug.flow_missing_seconds = 0.0f;
    g_pos_est_debug.flow_dt_seconds = 0.0f;
    g_pos_est_debug.flow_height_used_m = 0.0f;
    g_pos_est_debug.raw_delta_x_count = 0;
    g_pos_est_debug.raw_delta_y_count = 0;
    g_pos_est_debug.raw_squal = 0U;
    g_pos_est_debug.flow_gate_state = POS_EST_FLOW_GATE_PASS;
    pos_est_reset_flow_debug_measurement();
}

void pos_est_update_2khz(void)
{
    const float dt_seconds = POS_EST_DT_2KHZ_SECONDS;
    float body_acceleration_x_mps2;
    float body_acceleration_y_mps2;
    float body_acceleration_z_mps2;
    float body_gyro_x_dps;
    float body_gyro_y_dps;
    float body_gyro_z_dps;
    float acceleration_input_x_mps2;
    float acceleration_input_y_mps2;
    float bias_alpha;
    float velocity_zero_alpha;
    float velocity_input_alpha;
    float velocity_input_target_x_mps;
    float velocity_input_target_y_mps;
    float velocity_input_tau_seconds;
    uint8_t velocity_input_gate_valid = 0U;
    uint8_t is_static_state = 0U;

    AccelCalibration_GetBodyAccelMps2(
        &body_acceleration_x_mps2,
        &body_acceleration_y_mps2,
        &body_acceleration_z_mps2);
    AccelCalibration_GetBodyGyroDps(
        &body_gyro_x_dps,
        &body_gyro_y_dps,
        &body_gyro_z_dps);

    (void)body_acceleration_z_mps2;
    (void)body_gyro_z_dps;

    acceleration_input_x_mps2 = POS_EST_ACCELERATION_X_SIGN * body_acceleration_x_mps2;
    acceleration_input_y_mps2 = POS_EST_ACCELERATION_Y_SIGN * body_acceleration_y_mps2;

    if ((pos_est_absf(acceleration_input_x_mps2) < POS_EST_STATIC_ACCEL_THRESHOLD_MPS2) &&
        (pos_est_absf(acceleration_input_y_mps2) < POS_EST_STATIC_ACCEL_THRESHOLD_MPS2) &&
        (pos_est_absf(body_gyro_x_dps) < POS_EST_STATIC_GYRO_THRESHOLD_DPS) &&
        (pos_est_absf(body_gyro_y_dps) < POS_EST_STATIC_GYRO_THRESHOLD_DPS))
    {
        is_static_state = 1U;
    }

    if (is_static_state != 0U)
    {
        bias_alpha = dt_seconds / (POS_EST_ACCEL_BIAS_LEARN_TC_SECONDS + dt_seconds);
        s_pos_est_state.acceleration_bias_x_mps2 +=
            bias_alpha * (acceleration_input_x_mps2 - s_pos_est_state.acceleration_bias_x_mps2);
        s_pos_est_state.acceleration_bias_y_mps2 +=
            bias_alpha * (acceleration_input_y_mps2 - s_pos_est_state.acceleration_bias_y_mps2);
    }

    s_pos_est_state.acceleration_x_mps2 =
        acceleration_input_x_mps2 - s_pos_est_state.acceleration_bias_x_mps2;
    s_pos_est_state.acceleration_y_mps2 =
        acceleration_input_y_mps2 - s_pos_est_state.acceleration_bias_y_mps2;

    s_pos_est_state.acceleration_x_mps2 =
        pos_est_apply_deadband(s_pos_est_state.acceleration_x_mps2, POS_EST_ACCEL_DEADBAND_MPS2);
    s_pos_est_state.acceleration_y_mps2 =
        pos_est_apply_deadband(s_pos_est_state.acceleration_y_mps2, POS_EST_ACCEL_DEADBAND_MPS2);

    // 位置的预测 = 位置的旧估计 + 速度 * 时间增量 + 0.5 * 加速度 * 时间增量^2
    s_pos_est_state.position_x_m =
        s_pos_est_state.position_x_m +
        s_pos_est_state.velocity_x_mps * dt_seconds +
        0.5f * s_pos_est_state.acceleration_x_mps2 * dt_seconds * dt_seconds;

    // 速度的预测 = 速度的旧估计 + 加速度 * 时间增量
    s_pos_est_state.velocity_x_mps =
        s_pos_est_state.velocity_x_mps +
        s_pos_est_state.acceleration_x_mps2 * dt_seconds;
    
    // 同理更新 Y 轴位置
    s_pos_est_state.position_y_m =
        s_pos_est_state.position_y_m +
        s_pos_est_state.velocity_y_mps * dt_seconds +
        0.5f * s_pos_est_state.acceleration_y_mps2 * dt_seconds * dt_seconds;

    // 同理更新 Y 轴速度
    s_pos_est_state.velocity_y_mps =
        s_pos_est_state.velocity_y_mps +
        s_pos_est_state.acceleration_y_mps2 * dt_seconds;

    if ((is_static_state != 0U) &&
        (s_pos_est_state.flow_missing_seconds > POS_EST_NO_FLOW_STATIC_HOLD_DELAY_SECONDS))
    {
        velocity_zero_alpha = dt_seconds / (POS_EST_STATIC_VELOCITY_DAMP_TC_SECONDS + dt_seconds);
        s_pos_est_state.velocity_x_mps += velocity_zero_alpha * (0.0f - s_pos_est_state.velocity_x_mps);
        s_pos_est_state.velocity_y_mps += velocity_zero_alpha * (0.0f - s_pos_est_state.velocity_y_mps);

        if (pos_est_absf(s_pos_est_state.velocity_x_mps) < POS_EST_STATIC_VELOCITY_ZERO_MPS)
        {
            s_pos_est_state.velocity_x_mps = 0.0f;
        }
        if (pos_est_absf(s_pos_est_state.velocity_y_mps) < POS_EST_STATIC_VELOCITY_ZERO_MPS)
        {
            s_pos_est_state.velocity_y_mps = 0.0f;
        }
    }

    pos_est_limit_state();

    g_pos_est_debug.acceleration_input_x_mps2 = acceleration_input_x_mps2;
    g_pos_est_debug.acceleration_input_y_mps2 = acceleration_input_y_mps2;
    g_pos_est_debug.acceleration_bias_x_mps2 = s_pos_est_state.acceleration_bias_x_mps2;
    g_pos_est_debug.acceleration_bias_y_mps2 = s_pos_est_state.acceleration_bias_y_mps2;
    g_pos_est_debug.body_acceleration_x_mps2 = s_pos_est_state.acceleration_x_mps2;
    g_pos_est_debug.body_acceleration_y_mps2 = s_pos_est_state.acceleration_y_mps2;
    g_pos_est_debug.body_gyro_x_dps = body_gyro_x_dps;
    g_pos_est_debug.body_gyro_y_dps = body_gyro_y_dps;
    g_pos_est_debug.is_static_state = is_static_state;
    g_pos_est_debug.flow_missing_seconds = s_pos_est_state.flow_missing_seconds;

    g_pos_est_output.position_x_m = s_pos_est_state.position_x_m;
    g_pos_est_output.position_y_m = s_pos_est_state.position_y_m;
    g_pos_est_output.velocity_x_mps = s_pos_est_state.velocity_x_mps;
    g_pos_est_output.velocity_y_mps = s_pos_est_state.velocity_y_mps;
    g_pos_est_output.acceleration_x_mps2 = s_pos_est_state.acceleration_x_mps2;
    g_pos_est_output.acceleration_y_mps2 = s_pos_est_state.acceleration_y_mps2;

    if ((s_pos_est_state.flow_reference_ready != 0U) &&
        ((g_pos_est_output.flow_update_valid != 0U) ||
         (s_pos_est_state.flow_missing_seconds <= POS_EST_VELOCITY_INPUT_FLOW_FRESH_SECONDS)))
    {
        velocity_input_gate_valid = 1U;
    }

    if (velocity_input_gate_valid != 0U)
    {
        velocity_input_target_x_mps = s_pos_est_state.velocity_x_mps;
        velocity_input_target_y_mps = s_pos_est_state.velocity_y_mps;
        velocity_input_tau_seconds = POS_EST_VELOCITY_INPUT_VALID_LPF_TAU_SECONDS;
    }
    else
    {
        velocity_input_target_x_mps = 0.0f;
        velocity_input_target_y_mps = 0.0f;
        velocity_input_tau_seconds = POS_EST_VELOCITY_INPUT_LOST_LPF_TAU_SECONDS;
    }

    if (velocity_input_tau_seconds <= 0.0f)
    {
        velocity_input_alpha = 1.0f;
    }
    else
    {
        velocity_input_alpha = dt_seconds / (velocity_input_tau_seconds + dt_seconds);
    }

    g_velocity_input_100hz.velocity_x_mps +=
        velocity_input_alpha * (velocity_input_target_x_mps - g_velocity_input_100hz.velocity_x_mps);
    g_velocity_input_100hz.velocity_y_mps +=
        velocity_input_alpha * (velocity_input_target_y_mps - g_velocity_input_100hz.velocity_y_mps);

    g_velocity_input_100hz.velocity_x_mps = pos_est_clampf(
        g_velocity_input_100hz.velocity_x_mps,
        -POS_EST_VELOCITY_LIMIT_MPS,
        POS_EST_VELOCITY_LIMIT_MPS);
    g_velocity_input_100hz.velocity_y_mps = pos_est_clampf(
        g_velocity_input_100hz.velocity_y_mps,
        -POS_EST_VELOCITY_LIMIT_MPS,
        POS_EST_VELOCITY_LIMIT_MPS);
    g_velocity_input_100hz.valid = velocity_input_gate_valid;
}

void pos_est_update_200hz(void)
{
    const float flow_call_dt_seconds = POS_EST_FLOW_CALL_DT_SECONDS;
    float flow_dt_seconds;
    float height_m = 0.0f;
    float flow_height_used_m = 0.0f;
    float body_gyro_x_dps;
    float body_gyro_y_dps;
    float body_gyro_z_dps;
    float flow_delta_x_count;
    float flow_delta_y_count;
    float flow_mapped_delta_x_count;
    float flow_mapped_delta_y_count;
    float flow_rate_x_rps;
    float flow_rate_y_rps;
    float gyro_for_flow_x_dps;
    float gyro_for_flow_y_dps;
    float flow_gyro_x_rps;
    float flow_gyro_y_rps;
    float flow_gyro_rot_comp_x_rps;
    float flow_gyro_rot_comp_y_rps;
    float flow_translation_x_rps;
    float flow_translation_y_rps;
    float measured_velocity_x_mps;
    float measured_velocity_y_mps;
    float position_residual_x_m;
    float position_residual_y_m;
    float flow_translation_lpf_alpha;

    s_pos_est_state.flow_elapsed_seconds += flow_call_dt_seconds;
    if (s_pos_est_state.flow_elapsed_seconds > 1.0f)
    {
        s_pos_est_state.flow_elapsed_seconds = 1.0f;
    }
    s_pos_est_state.flow_missing_seconds += flow_call_dt_seconds;
    if (s_pos_est_state.flow_missing_seconds > 10.0f)
    {
        s_pos_est_state.flow_missing_seconds = 10.0f;
    }

    g_pos_est_debug.flow_dt_seconds = s_pos_est_state.flow_elapsed_seconds;
    g_pos_est_debug.flow_missing_seconds = s_pos_est_state.flow_missing_seconds;
    g_pos_est_debug.raw_delta_x_count = g_pmw3901_raw.deltaX;
    g_pos_est_debug.raw_delta_y_count = g_pmw3901_raw.deltaY;
    g_pos_est_debug.raw_squal = g_pmw3901_raw.squal;

    height_m = g_height_est_m;
    g_pos_est_output.height_m = height_m;

    if (!pos_est_height_valid())
    {
        s_pos_est_state.flow_height_ready = 0U;
        s_pos_est_state.flow_translation_lpf_ready = 0U;
        g_pos_est_debug.flow_height_used_m = 0.0f;
        g_pos_est_debug.flow_gate_state = POS_EST_FLOW_GATE_HEIGHT_INVALID;
        pos_est_reset_flow_debug_measurement();
        g_pos_est_output.flow_update_valid = 0U;
        return;
    }

    /* 高度低通独立于光流门控，避免 valid/squal 门控失败时调试高度闪零 */
    flow_height_used_m = pos_est_update_flow_height_used(height_m, flow_call_dt_seconds, 0U);
    g_pos_est_debug.flow_height_used_m = flow_height_used_m;

    if (g_pmw3901_raw.motionOccured == 0U)
    {
        g_pos_est_debug.flow_gate_state = POS_EST_FLOW_GATE_VALID_FLAG_ZERO;
        pos_est_reset_flow_debug_measurement();
        g_pos_est_output.flow_update_valid = 0U;
        return;
    }

    if (g_pmw3901_raw.squal < POS_EST_FLOW_SQUAL_MIN)
    {
        g_pos_est_debug.flow_gate_state = POS_EST_FLOW_GATE_SQUAL_LOW;
        pos_est_reset_flow_debug_measurement();
        g_pos_est_output.flow_update_valid = 0U;
        return;
    }

    if (pos_est_abs_i16_over_limit(g_pmw3901_raw.deltaX, POS_EST_FLOW_DELTA_MAX) != 0U ||
        pos_est_abs_i16_over_limit(g_pmw3901_raw.deltaY, POS_EST_FLOW_DELTA_MAX) != 0U)
    {
        g_pos_est_debug.flow_gate_state = POS_EST_FLOW_GATE_DELTA_OVERFLOW;
        pos_est_reset_flow_debug_measurement();
        g_pos_est_output.flow_update_valid = 0U;
        return;
    }

    flow_dt_seconds = s_pos_est_state.flow_elapsed_seconds;
    if (!pos_est_flow_dt_valid(flow_dt_seconds))
    {
        s_pos_est_state.flow_reference_ready = 0U;
        s_pos_est_state.flow_elapsed_seconds = 0.0f;
        s_pos_est_state.flow_height_ready = 0U;
        s_pos_est_state.flow_translation_lpf_ready = 0U;
        g_pos_est_debug.flow_gate_state = POS_EST_FLOW_GATE_DT_INVALID;
        pos_est_reset_flow_debug_measurement();
        g_pos_est_output.flow_update_valid = 0U;
        return;
    }

    flow_delta_x_count = (float)g_pmw3901_raw.deltaX;
    flow_delta_y_count = (float)g_pmw3901_raw.deltaY;
#if POS_EST_FLOW_SWAP_XY
    {
        const float temp_delta_count = flow_delta_x_count;
        flow_delta_x_count = flow_delta_y_count;
        flow_delta_y_count = temp_delta_count;
    }
#endif

    flow_mapped_delta_x_count = POS_EST_FLOW_X_SIGN * flow_delta_x_count;
    flow_mapped_delta_y_count = POS_EST_FLOW_Y_SIGN * flow_delta_y_count;

    g_pos_est_debug.flow_mapped_delta_x_count = flow_mapped_delta_x_count;
    g_pos_est_debug.flow_mapped_delta_y_count = flow_mapped_delta_y_count;

    flow_rate_x_rps = flow_mapped_delta_x_count *
                      POS_EST_FLOW_RADIANS_PER_COUNT / flow_dt_seconds;

    flow_rate_y_rps = flow_mapped_delta_y_count *
                      POS_EST_FLOW_RADIANS_PER_COUNT / flow_dt_seconds;

    g_pos_est_debug.flow_rate_x_rps = flow_rate_x_rps;
    g_pos_est_debug.flow_rate_y_rps = flow_rate_y_rps;

    AccelCalibration_GetBodyGyroDps(&body_gyro_x_dps, &body_gyro_y_dps, &body_gyro_z_dps);
    (void)body_gyro_z_dps;

    g_pos_est_debug.body_gyro_x_dps = body_gyro_x_dps;
    g_pos_est_debug.body_gyro_y_dps = body_gyro_y_dps;

    gyro_for_flow_x_dps = body_gyro_x_dps;
    gyro_for_flow_y_dps = body_gyro_y_dps;
#if POS_EST_FLOW_GYRO_SWAP_XY
    {
        const float temp_gyro_dps = gyro_for_flow_x_dps;
        gyro_for_flow_x_dps = gyro_for_flow_y_dps;
        gyro_for_flow_y_dps = temp_gyro_dps;
    }
#endif

    flow_gyro_x_rps =
        POS_EST_FLOW_GYRO_X_SIGN * POS_EST_GYRO_X_SIGN * gyro_for_flow_x_dps * POS_EST_DEG_TO_RAD;
    flow_gyro_y_rps =
        POS_EST_FLOW_GYRO_Y_SIGN * POS_EST_GYRO_Y_SIGN * gyro_for_flow_y_dps * POS_EST_DEG_TO_RAD;

    g_pos_est_debug.flow_gyro_x_rps = flow_gyro_x_rps;
    g_pos_est_debug.flow_gyro_y_rps = flow_gyro_y_rps;

    flow_gyro_rot_comp_x_rps =
        POS_EST_GYRO_DEC_K11 * flow_gyro_x_rps +
        POS_EST_GYRO_DEC_K12 * flow_gyro_y_rps;
    flow_gyro_rot_comp_y_rps =
        POS_EST_GYRO_DEC_K21 * flow_gyro_x_rps +
        POS_EST_GYRO_DEC_K22 * flow_gyro_y_rps;

    flow_translation_x_rps = flow_rate_x_rps - flow_gyro_rot_comp_x_rps;
    flow_translation_y_rps = flow_rate_y_rps - flow_gyro_rot_comp_y_rps;

    flow_translation_x_rps = pos_est_clampf(
        flow_translation_x_rps,
        -POS_EST_FLOW_TRANSLATION_LIMIT_RPS,
        POS_EST_FLOW_TRANSLATION_LIMIT_RPS);
    flow_translation_y_rps = pos_est_clampf(
        flow_translation_y_rps,
        -POS_EST_FLOW_TRANSLATION_LIMIT_RPS,
        POS_EST_FLOW_TRANSLATION_LIMIT_RPS);

    if (POS_EST_FLOW_TRANSLATION_LPF_TAU_SECONDS > 0.0f)
    {
        if (s_pos_est_state.flow_translation_lpf_ready == 0U)
        {
            s_pos_est_state.flow_translation_lpf_x_rps = flow_translation_x_rps;
            s_pos_est_state.flow_translation_lpf_y_rps = flow_translation_y_rps;
            s_pos_est_state.flow_translation_lpf_ready = 1U;
        }
        else
        {
            flow_translation_lpf_alpha =
                flow_dt_seconds /
                (POS_EST_FLOW_TRANSLATION_LPF_TAU_SECONDS + flow_dt_seconds);
            s_pos_est_state.flow_translation_lpf_x_rps +=
                flow_translation_lpf_alpha *
                (flow_translation_x_rps - s_pos_est_state.flow_translation_lpf_x_rps);
            s_pos_est_state.flow_translation_lpf_y_rps +=
                flow_translation_lpf_alpha *
                (flow_translation_y_rps - s_pos_est_state.flow_translation_lpf_y_rps);
        }

        flow_translation_x_rps = s_pos_est_state.flow_translation_lpf_x_rps;
        flow_translation_y_rps = s_pos_est_state.flow_translation_lpf_y_rps;
    }

    g_pos_est_debug.flow_translation_x_rps = flow_translation_x_rps;
    g_pos_est_debug.flow_translation_y_rps = flow_translation_y_rps;

    measured_velocity_x_mps = flow_translation_x_rps * flow_height_used_m;
    measured_velocity_y_mps = flow_translation_y_rps * flow_height_used_m;

    g_pos_est_debug.measured_velocity_x_mps = measured_velocity_x_mps;
    g_pos_est_debug.measured_velocity_y_mps = measured_velocity_y_mps;

    if (s_pos_est_state.flow_reference_ready == 0U)
    {
        s_pos_est_state.measured_position_x_m = s_pos_est_state.position_x_m;
        s_pos_est_state.measured_position_y_m = s_pos_est_state.position_y_m;
        s_pos_est_state.flow_reference_ready = 1U;
    }

    s_pos_est_state.measured_position_x_m =
        s_pos_est_state.measured_position_x_m + measured_velocity_x_mps * flow_dt_seconds;

    s_pos_est_state.measured_position_y_m =
        s_pos_est_state.measured_position_y_m + measured_velocity_y_mps * flow_dt_seconds;

    position_residual_x_m = s_pos_est_state.measured_position_x_m - s_pos_est_state.position_x_m;
    position_residual_y_m = s_pos_est_state.measured_position_y_m - s_pos_est_state.position_y_m;

    position_residual_x_m = pos_est_clampf(
        position_residual_x_m,
        -POS_EST_POSITION_RESIDUAL_LIMIT_M,
        POS_EST_POSITION_RESIDUAL_LIMIT_M);
    position_residual_y_m = pos_est_clampf(
        position_residual_y_m,
        -POS_EST_POSITION_RESIDUAL_LIMIT_M,
        POS_EST_POSITION_RESIDUAL_LIMIT_M);

    g_pos_est_debug.position_residual_x_m = position_residual_x_m;
    g_pos_est_debug.position_residual_y_m = position_residual_y_m;

    s_pos_est_state.position_x_m =
        s_pos_est_state.position_x_m + POS_EST_ALPHA * position_residual_x_m;
    s_pos_est_state.velocity_x_mps =
        s_pos_est_state.velocity_x_mps + (POS_EST_BETA / flow_dt_seconds) * position_residual_x_m;
#if POS_EST_ENABLE_GAMMA_UPDATE
    s_pos_est_state.acceleration_x_mps2 =
        s_pos_est_state.acceleration_x_mps2 +
        (2.0f * POS_EST_GAMMA / (flow_dt_seconds * flow_dt_seconds)) * position_residual_x_m;
#endif

    s_pos_est_state.position_y_m =
        s_pos_est_state.position_y_m + POS_EST_ALPHA * position_residual_y_m;
    s_pos_est_state.velocity_y_mps =
        s_pos_est_state.velocity_y_mps + (POS_EST_BETA / flow_dt_seconds) * position_residual_y_m;
#if POS_EST_ENABLE_GAMMA_UPDATE
    s_pos_est_state.acceleration_y_mps2 =
        s_pos_est_state.acceleration_y_mps2 +
        (2.0f * POS_EST_GAMMA / (flow_dt_seconds * flow_dt_seconds)) * position_residual_y_m;
#endif

    pos_est_limit_state();

    g_pos_est_output.position_x_m = s_pos_est_state.position_x_m;
    g_pos_est_output.position_y_m = s_pos_est_state.position_y_m;
    g_pos_est_output.velocity_x_mps = s_pos_est_state.velocity_x_mps;
    g_pos_est_output.velocity_y_mps = s_pos_est_state.velocity_y_mps;
    g_pos_est_output.acceleration_x_mps2 = s_pos_est_state.acceleration_x_mps2;
    g_pos_est_output.acceleration_y_mps2 = s_pos_est_state.acceleration_y_mps2;
    g_pos_est_output.flow_update_valid = 1U;
    g_pos_est_debug.flow_gate_state = POS_EST_FLOW_GATE_PASS;
    s_pos_est_state.flow_missing_seconds = 0.0f;
    g_pos_est_debug.flow_missing_seconds = 0.0f;
    s_pos_est_state.flow_elapsed_seconds = 0.0f;
}

