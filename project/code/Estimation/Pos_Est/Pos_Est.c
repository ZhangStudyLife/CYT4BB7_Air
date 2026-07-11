#include "Pos_Est.h"
#include "FlowGyroDecoupler_LC302.h"
#include "../Attitude/Accel_Calibration.h"
#include "../Attitude/IMU_Filtter.h"
#include "../Height_Est/Height_Est.h"
#include "HW_Drivers/LC302/LC302.h"
#include "FlightController/fc_params.h"
#include "FlightController/fc_start_crsf.h"
#include <math.h>

extern volatile uint32 tick_1000us_cnt;

/*
 * Coordinate and unit contract:
 *   body FRD acceleration: forward/right/down, cm/s^2 after projection;
 *   public velocity: X left-positive, Y forward-positive, cm/s;
 *   gyro: deg/s, positive body Z increases yaw;
 *   LC302 integration period: fixed 20800 us.
 */
#define POS_EST_ACC_DT_S (0.001f)
#define POS_EST_ACC_FWD_LIMIT_CMSS (600.0f)
#define POS_EST_ACC_RIGHT_LIMIT_CMSS (600.0f)
#define POS_EST_DEG_TO_RAD (0.017453292519943295f)

/* Used only by the static accelerometer-bias learner, not by velocity propagation. */
#define POS_EST_BIAS_INPUT_LPF_ALPHA (0.06089863f)
#define POS_EST_STATIC_GYRO_MAX_DPS (3.0f)
#define POS_EST_STATIC_TILT_MAX_DEG (6.0f)
#define POS_EST_STATIC_HEIGHT_MIN_MM (70.0f)
#define POS_EST_STATIC_HEIGHT_MAX_MM (200.0f)
#define POS_EST_STATIC_LOCK_SAMPLES (300U)
#define POS_EST_ACC_BIAS_ALPHA (0.001f)
#define POS_EST_ACC_BIAS_LIMIT_CMSS (120.0f)

/* Fixed-period LC302 conversion and one-shot publication handshake. */
#define POS_EST_FLOW_FRAME_US (20800U)
#define POS_EST_FLOW_TO_RADPS (0.00480769231f)
#define POS_EST_FLOW_TO_CMPS (0.48076923f)
#define POS_EST_FLOW_MIN_HEIGHT_M (0.20f)
#define POS_EST_FLOW_FULL_GAIN_HEIGHT_M (0.30f)
#define POS_EST_FLOW_GAIN_MAX (0.25f)
#define POS_EST_FLOW_INNOVATION_LIMIT_CMPS (100.0f)
#define POS_EST_FLOW_CORRECTION_LIMIT_CMPS (18.0f)

/* Data age, outage propagation, and bounded-output policy. */
#define POS_EST_FLOW_TIMEOUT_RESET_MS (35U)
#define POS_EST_FLOW_INERTIAL_HOLD_MS (150U)
#define POS_EST_FLOW_LONG_OUTAGE_MS (500U)
#define POS_EST_FLOW_OUTPUT_LIMIT_CMPS (250.0f)

/* Small optical-flow health state machine. */
#define POS_EST_FLOW_INVALID_LIMIT (3U)
#define POS_EST_FLOW_REACQUIRE_FRAMES (3U)
#define POS_EST_FLOW_HEALTH_SCORE_LIMIT (3U)
#define POS_EST_FLOW_REF_REANCHOR_FRAMES (5U)
#define POS_EST_FLOW_RAMP_START (0.25f)
#define POS_EST_FLOW_RAMP_STEP (0.125f)

/* Hard gates reject immediately. */
#define POS_EST_FLOW_RATE_HARD_RADPS (3.0f)
#define POS_EST_FLOW_RATE_JUMP_HARD_RADPS (2.5f)
#define POS_EST_FLOW_SPEED_HARD_CMPS (300.0f)
#define POS_EST_FLOW_CONTINUITY_HARD_CMPS (220.0f)
#define POS_EST_FLOW_INNOVATION_HARD_CMPS (250.0f)

/* Soft gates first reduce gain, then reject if they persist. */
#define POS_EST_FLOW_RATE_SOFT_RADPS (2.0f)
#define POS_EST_FLOW_RATE_JUMP_SOFT_RADPS (1.2f)
#define POS_EST_FLOW_CONTINUITY_SOFT_CMPS (80.0f)
#define POS_EST_FLOW_INNOVATION_SOFT_CMPS (100.0f)
#define POS_EST_FLOW_REACQUIRE_RATE_RADPS (2.5f)
#define POS_EST_FLOW_REACQUIRE_JUMP_RADPS (1.5f)
#define POS_EST_FLOW_REACQUIRE_SPEED_CMPS (260.0f)
#define POS_EST_FLOW_REACQUIRE_INNOVATION_CMPS (300.0f)

float Pos_Est_vel_x = 0.0f;
float Pos_Est_vel_y = 0.0f;

static float s_vel_pred_x = 0.0f;
static float s_vel_pred_y = 0.0f;
static float s_bias_input_lp_x = 0.0f;
static float s_bias_input_lp_y = 0.0f;
static float s_acc_bias_x_cmss = 0.0f;
static float s_acc_bias_y_cmss = 0.0f;
static uint16_t s_static_sample_count = 0U;

/* LC302 arrival time and timeout state. */
static uint32_t s_flow_last_arrival_ms = 0U;
static uint32_t s_flow_last_accepted_ms = 0U;
static uint8_t s_flow_decoupler_timed_out = 0U;

/* Optical-flow anomaly protection and gradual reacquisition. */
static uint8_t s_flow_healthy = 1U;
static uint8_t s_flow_health_score = 0U;
static uint8_t s_flow_invalid_count = 0U;
static uint8_t s_flow_reacquire_count = 0U;
static float s_flow_gain_ramp = 1.0f;
static float s_flow_prev_rate_x = 0.0f;
static float s_flow_prev_rate_y = 0.0f;
static uint8_t s_flow_prev_rate_valid = 0U;

/* Last trusted flow velocity, propagated by IMU for continuity checking. */
static float s_flow_ref_x = 0.0f;
static float s_flow_ref_y = 0.0f;
static uint8_t s_flow_ref_valid = 0U;
static uint8_t s_flow_ref_frame_count = 0U;

static float Pos_Est_ClampFloat(float value, float min_value, float max_value)
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

static float Pos_Est_VectorNorm(float x, float y)
{
    return sqrtf(x * x + y * y);
}

static void Pos_Est_RotateBodyVelocity(float *vel_x, float *vel_y, float yaw_delta_rad)
{
    float old_x = *vel_x;
    float old_y = *vel_y;
    float cos_delta = 1.0f - 0.5f * yaw_delta_rad * yaw_delta_rad;

    /* R(-d_yaw) for [left, forward]. */
    *vel_x = cos_delta * old_x + yaw_delta_rad * old_y;
    *vel_y = cos_delta * old_y - yaw_delta_rad * old_x;
}

static void Pos_Est_SetFlowUnhealthy(void)
{
    s_flow_healthy = 0U;
    s_flow_health_score = POS_EST_FLOW_HEALTH_SCORE_LIMIT;
    s_flow_reacquire_count = 0U;
    s_flow_gain_ramp = 0.0f;
}

static float Pos_Est_GetFlowGain(float height_m)
{
    float gain = g_fc_params.pos_est_k_flow;

    gain = Pos_Est_ClampFloat(gain, 0.0f, POS_EST_FLOW_GAIN_MAX);

    if (height_m < POS_EST_FLOW_MIN_HEIGHT_M)
    {
        return 0.0f;
    }
    if (height_m < POS_EST_FLOW_FULL_GAIN_HEIGHT_M)
    {
        gain *= (height_m - POS_EST_FLOW_MIN_HEIGHT_M) /
                (POS_EST_FLOW_FULL_GAIN_HEIGHT_M - POS_EST_FLOW_MIN_HEIGHT_M);
    }
    return gain;
}

static uint8_t Pos_Est_IsStaticForAccelBias(void)
{
    FC_START_CRSF_state_e fc_state = FC_START_CRSF_Get_State();

    if ((fc_state != FC_START_CRSF_STATE_STANDBY) &&
        (fc_state != FC_START_CRSF_STATE_LANDING))
    {
        return 0U;
    }
    if ((g_tof_fused_valid == 0U) ||
        (g_tof_fused_height_mm < POS_EST_STATIC_HEIGHT_MIN_MM) ||
        (g_tof_fused_height_mm > POS_EST_STATIC_HEIGHT_MAX_MM))
    {
        return 0U;
    }
    if ((fabsf(g_euler.roll) > POS_EST_STATIC_TILT_MAX_DEG) ||
        (fabsf(g_euler.pitch) > POS_EST_STATIC_TILT_MAX_DEG))
    {
        return 0U;
    }
    if ((fabsf(g_imufilter_1000hz.gyrox) > POS_EST_STATIC_GYRO_MAX_DPS) ||
        (fabsf(g_imufilter_1000hz.gyroy) > POS_EST_STATIC_GYRO_MAX_DPS) ||
        (fabsf(g_imufilter_1000hz.gyroz) > POS_EST_STATIC_GYRO_MAX_DPS))
    {
        return 0U;
    }
    return (g_imu_shock_flag == 0U) ? 1U : 0U;
}

static void Pos_Est_ResetFlowState(uint32_t now_ms)
{
    s_flow_last_arrival_ms = now_ms;
    s_flow_last_accepted_ms = now_ms;
    s_flow_decoupler_timed_out = 0U;
    s_flow_healthy = 1U;
    s_flow_health_score = 0U;
    s_flow_invalid_count = 0U;
    s_flow_reacquire_count = 0U;
    s_flow_gain_ramp = 1.0f;
    s_flow_prev_rate_x = 0.0f;
    s_flow_prev_rate_y = 0.0f;
    s_flow_prev_rate_valid = 0U;
    s_flow_ref_x = 0.0f;
    s_flow_ref_y = 0.0f;
    s_flow_ref_valid = 0U;
    s_flow_ref_frame_count = 0U;
}

static void Pos_Est_ProcessFlowFrame(int16_t flow_x,
                                     int16_t flow_y,
                                     uint8_t sensor_valid,
                                     uint32_t now_ms)
{
    float height_m = g_tof_fused_height_mm * 0.001f;
    uint8_t height_valid = ((g_tof_fused_valid != 0U) &&
                            (height_m >= POS_EST_FLOW_MIN_HEIGHT_M) &&
                            (g_tof_fused_height_mm <= VL53L1X_VALID_RANGE_MAX)) ? 1U : 0U;
    uint8_t dec_valid;
    float dec_x;
    float dec_y;
    float flow_vel_x;
    float flow_vel_y;
    float flow_rate_x;
    float flow_rate_y;
    float flow_rate_norm;
    float flow_rate_jump = 0.0f;
    float flow_speed_norm;
    float innovation_norm;
    float continuity_norm = 0.0f;
    uint8_t hard_bad;
    uint8_t soft_count = 0U;
    uint8_t continuity_soft = 0U;
    float gain;
    float innovation_x;
    float innovation_y;
    float correction_x;
    float correction_y;
    float correction_norm;

    dec_valid = FlowGyroDecoupler_LC302_Update50Hz(now_ms,
                                                   flow_x,
                                                   flow_y,
                                                   sensor_valid);
    if ((sensor_valid == 0U) || (height_valid == 0U) || (dec_valid == 0U))
    {
        if ((height_valid != 0U) && (sensor_valid == 0U))
        {
            if (s_flow_invalid_count < POS_EST_FLOW_INVALID_LIMIT)
            {
                s_flow_invalid_count++;
            }
            if (s_flow_invalid_count >= POS_EST_FLOW_INVALID_LIMIT)
            {
                Pos_Est_SetFlowUnhealthy();
            }
        }
        else
        {
            s_flow_invalid_count = 0U;
        }
        return;
    }

    s_flow_invalid_count = 0U;
    dec_x = FlowGyroDecoupler_LC302_GetDecX();
    dec_y = FlowGyroDecoupler_LC302_GetDecY();
    flow_rate_x = dec_x * POS_EST_FLOW_TO_RADPS;
    flow_rate_y = dec_y * POS_EST_FLOW_TO_RADPS;
    flow_vel_x = height_m * dec_x * POS_EST_FLOW_TO_CMPS;
    flow_vel_y = height_m * dec_y * POS_EST_FLOW_TO_CMPS;

    flow_rate_norm = Pos_Est_VectorNorm(flow_rate_x, flow_rate_y);
    flow_speed_norm = Pos_Est_VectorNorm(flow_vel_x, flow_vel_y);
    innovation_norm = Pos_Est_VectorNorm(flow_vel_x - s_vel_pred_x,
                                         flow_vel_y - s_vel_pred_y);
    if (s_flow_prev_rate_valid != 0U)
    {
        flow_rate_jump = Pos_Est_VectorNorm(flow_rate_x - s_flow_prev_rate_x,
                                            flow_rate_y - s_flow_prev_rate_y);
    }
    if (s_flow_ref_valid != 0U)
    {
        continuity_norm = Pos_Est_VectorNorm(flow_vel_x - s_flow_ref_x,
                                             flow_vel_y - s_flow_ref_y);
    }

    hard_bad = ((flow_rate_norm > POS_EST_FLOW_RATE_HARD_RADPS) ||
                (flow_rate_jump > POS_EST_FLOW_RATE_JUMP_HARD_RADPS) ||
                (flow_speed_norm > POS_EST_FLOW_SPEED_HARD_CMPS) ||
                ((s_flow_ref_valid != 0U) &&
                 (continuity_norm > POS_EST_FLOW_CONTINUITY_HARD_CMPS)) ||
                (innovation_norm > POS_EST_FLOW_INNOVATION_HARD_CMPS)) ? 1U : 0U;

    if (flow_rate_norm > POS_EST_FLOW_RATE_SOFT_RADPS)
    {
        soft_count++;
    }
    if (flow_rate_jump > POS_EST_FLOW_RATE_JUMP_SOFT_RADPS)
    {
        soft_count++;
    }
    if ((s_flow_ref_valid != 0U) &&
        (continuity_norm > POS_EST_FLOW_CONTINUITY_SOFT_CMPS))
    {
        soft_count++;
        continuity_soft = 1U;
    }
    if (innovation_norm > POS_EST_FLOW_INNOVATION_SOFT_CMPS)
    {
        soft_count++;
    }

    if (s_flow_healthy != 0U)
    {
        if (hard_bad != 0U)
        {
            Pos_Est_SetFlowUnhealthy();
        }
        else
        {
            if ((continuity_soft != 0U) || (soft_count >= 2U))
            {
                if (s_flow_health_score < POS_EST_FLOW_HEALTH_SCORE_LIMIT)
                {
                    s_flow_health_score++;
                }
            }
            else if (s_flow_health_score > 0U)
            {
                s_flow_health_score--;
            }

            if (s_flow_health_score >= POS_EST_FLOW_HEALTH_SCORE_LIMIT)
            {
                Pos_Est_SetFlowUnhealthy();
            }
        }
    }
    else
    {
        uint8_t reacquire_good = ((flow_rate_norm < POS_EST_FLOW_REACQUIRE_RATE_RADPS) &&
                                  (flow_rate_jump < POS_EST_FLOW_REACQUIRE_JUMP_RADPS) &&
                                  (flow_speed_norm < POS_EST_FLOW_REACQUIRE_SPEED_CMPS) &&
                                  (innovation_norm < POS_EST_FLOW_REACQUIRE_INNOVATION_CMPS)) ? 1U : 0U;

        if (reacquire_good != 0U)
        {
            if (s_flow_reacquire_count < POS_EST_FLOW_REACQUIRE_FRAMES)
            {
                s_flow_reacquire_count++;
            }
        }
        else
        {
            s_flow_reacquire_count = 0U;
        }

        if (s_flow_reacquire_count >= POS_EST_FLOW_REACQUIRE_FRAMES)
        {
            s_flow_healthy = 1U;
            s_flow_health_score = 0U;
            s_flow_reacquire_count = 0U;
            s_flow_gain_ramp = POS_EST_FLOW_RAMP_START;
            s_flow_ref_x = flow_vel_x;
            s_flow_ref_y = flow_vel_y;
            s_flow_ref_valid = 1U;
            s_flow_ref_frame_count = 0U;
            hard_bad = 0U;
        }
    }

    if ((s_flow_healthy != 0U) && (hard_bad == 0U))
    {
        gain = Pos_Est_GetFlowGain(height_m) * s_flow_gain_ramp;
        if (soft_count >= 2U)
        {
            gain *= 0.25f;
        }
        else if (soft_count == 1U)
        {
            gain *= 0.5f;
        }

        innovation_x = Pos_Est_ClampFloat(flow_vel_x - s_vel_pred_x,
                                          -POS_EST_FLOW_INNOVATION_LIMIT_CMPS,
                                          POS_EST_FLOW_INNOVATION_LIMIT_CMPS);
        innovation_y = Pos_Est_ClampFloat(flow_vel_y - s_vel_pred_y,
                                          -POS_EST_FLOW_INNOVATION_LIMIT_CMPS,
                                          POS_EST_FLOW_INNOVATION_LIMIT_CMPS);
        correction_x = gain * innovation_x;
        correction_y = gain * innovation_y;
        correction_norm = Pos_Est_VectorNorm(correction_x, correction_y);
        if (correction_norm > POS_EST_FLOW_CORRECTION_LIMIT_CMPS)
        {
            float correction_scale = POS_EST_FLOW_CORRECTION_LIMIT_CMPS /
                                     correction_norm;
            correction_x *= correction_scale;
            correction_y *= correction_scale;
        }
        s_vel_pred_x += correction_x;
        s_vel_pred_y += correction_y;
        s_flow_last_accepted_ms = now_ms;
        if (s_flow_ref_valid == 0U)
        {
            s_flow_ref_x = flow_vel_x;
            s_flow_ref_y = flow_vel_y;
            s_flow_ref_valid = 1U;
            s_flow_ref_frame_count = 0U;
        }
        else
        {
            if (s_flow_ref_frame_count < POS_EST_FLOW_REF_REANCHOR_FRAMES)
            {
                s_flow_ref_frame_count++;
            }
            /*
             * Keep an approximately 100 ms IMU-propagated anchor. Smooth cable
             * garbage must remain consistent with IMU over the whole window,
             * not merely with the immediately previous optical-flow frame.
             */
            if ((s_flow_ref_frame_count >= POS_EST_FLOW_REF_REANCHOR_FRAMES) &&
                (continuity_norm < POS_EST_FLOW_CONTINUITY_SOFT_CMPS))
            {
                s_flow_ref_x = flow_vel_x;
                s_flow_ref_y = flow_vel_y;
                s_flow_ref_frame_count = 0U;
            }
        }

        if (s_flow_gain_ramp < 1.0f)
        {
            s_flow_gain_ramp += POS_EST_FLOW_RAMP_STEP;
            if (s_flow_gain_ramp > 1.0f)
            {
                s_flow_gain_ramp = 1.0f;
            }
        }
    }

    s_flow_prev_rate_x = flow_rate_x;
    s_flow_prev_rate_y = flow_rate_y;
    s_flow_prev_rate_valid = 1U;
}

void Pos_Est_Init(void)
{
    LC302_Init();
    FlowGyroDecoupler_LC302_Init();

    Pos_Est_vel_x = 0.0f;
    Pos_Est_vel_y = 0.0f;
    s_vel_pred_x = 0.0f;
    s_vel_pred_y = 0.0f;
    s_bias_input_lp_x = 0.0f;
    s_bias_input_lp_y = 0.0f;
    s_acc_bias_x_cmss = 0.0f;
    s_acc_bias_y_cmss = 0.0f;
    s_static_sample_count = 0U;
    Pos_Est_ResetFlowState(tick_1000us_cnt);
}

void Pos_Est_Update_1000HZ(void)
{
    float acc_sensor[3];
    float acc_body[3];
    float gyro_body_x;
    float gyro_body_y;
    float gyro_body_z;
    float sp;
    float cp;
    float sr;
    float cr;
    float yaw_rate_dps;
    float yaw_delta_rad;
    float output_speed;
    float acc_x_temp;
    float acc_y_temp;
    float acc_x_lp;
    float acc_y_lp;
    uint32_t flow_age_ms;
    int16_t frame_flow_x = 0;
    int16_t frame_flow_y = 0;
    uint8_t accel_bias_locked = 0U;
    uint8_t flow_new_frame = 0U;
    uint8_t frame_valid = 0U;

    AccelCalibration_GetBodyGyroDps(&gyro_body_x, &gyro_body_y, &gyro_body_z);
    FlowGyroDecoupler_LC302_Push1000Hz(tick_1000us_cnt, gyro_body_x, gyro_body_y);

    acc_sensor[0] = g_imufilter_1000hz.accx;
    acc_sensor[1] = g_imufilter_1000hz.accy;
    acc_sensor[2] = g_imufilter_1000hz.accz;
    AccelCalibration_RotateImuToBody(acc_sensor, acc_body);

    sp = g_euler.sin_pitch;
    cp = g_euler.cos_pitch;
    sr = g_euler.sin_roll;
    cr = g_euler.cos_roll;

    acc_x_temp = (cp * acc_body[0] + sp * sr * acc_body[1] + sp * cr * acc_body[2]) *
                 9.80665f * 100.0f;
    acc_y_temp = (cr * acc_body[1] - sr * acc_body[2]) * 9.80665f * 100.0f;

    if (g_imu_shock_flag == 0U)
    {
        s_bias_input_lp_x += POS_EST_BIAS_INPUT_LPF_ALPHA *
                             (acc_x_temp - s_bias_input_lp_x);
        s_bias_input_lp_y += POS_EST_BIAS_INPUT_LPF_ALPHA *
                             (acc_y_temp - s_bias_input_lp_y);
    }

    if (Pos_Est_IsStaticForAccelBias() != 0U)
    {
        if (s_static_sample_count < POS_EST_STATIC_LOCK_SAMPLES)
        {
            s_static_sample_count++;
        }
        else
        {
            accel_bias_locked = 1U;
            s_acc_bias_x_cmss += POS_EST_ACC_BIAS_ALPHA *
                                 (s_bias_input_lp_x - s_acc_bias_x_cmss);
            s_acc_bias_y_cmss += POS_EST_ACC_BIAS_ALPHA *
                                 (s_bias_input_lp_y - s_acc_bias_y_cmss);
            s_acc_bias_x_cmss = Pos_Est_ClampFloat(s_acc_bias_x_cmss,
                                                   -POS_EST_ACC_BIAS_LIMIT_CMSS,
                                                   POS_EST_ACC_BIAS_LIMIT_CMSS);
            s_acc_bias_y_cmss = Pos_Est_ClampFloat(s_acc_bias_y_cmss,
                                                   -POS_EST_ACC_BIAS_LIMIT_CMSS,
                                                   POS_EST_ACC_BIAS_LIMIT_CMSS);
        }
    }
    else
    {
        s_static_sample_count = 0U;
    }

    /* Upstream IMU already has a 40 Hz PT2, so do not add another velocity LPF. */
    acc_x_lp = acc_x_temp - s_acc_bias_x_cmss;
    acc_y_lp = acc_y_temp - s_acc_bias_y_cmss;
    if (g_imu_shock_flag != 0U)
    {
        acc_x_lp = 0.0f;
        acc_y_lp = 0.0f;
    }
    acc_x_lp = Pos_Est_ClampFloat(acc_x_lp,
                                  -POS_EST_ACC_FWD_LIMIT_CMSS,
                                  POS_EST_ACC_FWD_LIMIT_CMSS);
    acc_y_lp = Pos_Est_ClampFloat(acc_y_lp,
                                  -POS_EST_ACC_RIGHT_LIMIT_CMSS,
                                  POS_EST_ACC_RIGHT_LIMIT_CMSS);

    /*
     * Exact Euler yaw rate for FRD at normal tilt:
     * yaw_dot = (q*sin(roll) + r*cos(roll)) / cos(pitch).
     */
    if ((fabsf(cp) > 0.5f) && isfinite(gyro_body_z))
    {
        yaw_rate_dps = (gyro_body_y * sr + gyro_body_z * cr) / cp;
    }
    else
    {
        yaw_rate_dps = gyro_body_z;
    }
    yaw_delta_rad = yaw_rate_dps * POS_EST_DEG_TO_RAD * POS_EST_ACC_DT_S;

    if (accel_bias_locked == 0U)
    {
        Pos_Est_RotateBodyVelocity(&s_vel_pred_x, &s_vel_pred_y, yaw_delta_rad);
        if (s_flow_ref_valid != 0U)
        {
            Pos_Est_RotateBodyVelocity(&s_flow_ref_x, &s_flow_ref_y, yaw_delta_rad);
        }

        if (g_imu_shock_flag == 0U)
        {
            s_vel_pred_x -= acc_y_lp * POS_EST_ACC_DT_S;
            s_vel_pred_y += acc_x_lp * POS_EST_ACC_DT_S;
            if (s_flow_ref_valid != 0U)
            {
                s_flow_ref_x -= acc_y_lp * POS_EST_ACC_DT_S;
                s_flow_ref_y += acc_x_lp * POS_EST_ACC_DT_S;
            }
        }
    }

    /*
     * LC302_Update_50HZ publishes lc302_data only when the ISR has a new frame.
     * The measured integration_timespan is always 20800 us. Clearing that field
     * after consumption therefore provides a one-shot new-frame handshake even
     * when two consecutive frames contain identical flow values.
     */
    LC302_Update_50HZ();
    if (lc302_data.integration_timespan == POS_EST_FLOW_FRAME_US)
    {
        flow_new_frame = 1U;
        frame_flow_x = lc302_data.flow_x_integral;
        frame_flow_y = lc302_data.flow_y_integral;
        frame_valid = lc302_data.valid;

        lc302_data.integration_timespan = 0U;
        lc302_data.valid = 0U;
        s_flow_last_arrival_ms = tick_1000us_cnt;
        s_flow_decoupler_timed_out = 0U;
        Pos_Est_ProcessFlowFrame(frame_flow_x,
                                 frame_flow_y,
                                 frame_valid,
                                 tick_1000us_cnt);
    }

    flow_age_ms = tick_1000us_cnt - s_flow_last_arrival_ms;
    if ((flow_age_ms > POS_EST_FLOW_TIMEOUT_RESET_MS) &&
        (s_flow_decoupler_timed_out == 0U))
    {
        /* A late next frame no longer matches a fixed 20.8 ms gyro window. */
        FlowGyroDecoupler_LC302_Reinit();
        s_flow_decoupler_timed_out = 1U;
        s_flow_prev_rate_valid = 0U;
        Pos_Est_SetFlowUnhealthy();
    }

    if (accel_bias_locked != 0U)
    {
        s_vel_pred_x = 0.0f;
        s_vel_pred_y = 0.0f;
        s_flow_ref_x = 0.0f;
        s_flow_ref_y = 0.0f;
        s_flow_ref_valid = 0U;
        s_flow_ref_frame_count = 0U;
        s_flow_invalid_count = 0U;
        s_flow_healthy = 1U;
        s_flow_health_score = 0U;
        s_flow_reacquire_count = 0U;
        s_flow_gain_ramp = 1.0f;
        s_flow_last_accepted_ms = tick_1000us_cnt;
    }

    flow_age_ms = tick_1000us_cnt - s_flow_last_accepted_ms;
    if ((accel_bias_locked == 0U) &&
        (flow_age_ms > POS_EST_FLOW_LONG_OUTAGE_MS))
    {
        s_vel_pred_x *= 0.998f;  /* approximately 0.5 s time constant at 1 kHz */
        s_vel_pred_y *= 0.998f;
    }
    else if ((accel_bias_locked == 0U) &&
             (flow_age_ms > POS_EST_FLOW_INERTIAL_HOLD_MS))
    {
        s_vel_pred_x *= 0.9995f; /* approximately 2 s time constant at 1 kHz */
        s_vel_pred_y *= 0.9995f;
    }

    output_speed = Pos_Est_VectorNorm(s_vel_pred_x, s_vel_pred_y);
    if (output_speed > POS_EST_FLOW_OUTPUT_LIMIT_CMPS)
    {
        float output_scale = POS_EST_FLOW_OUTPUT_LIMIT_CMPS / output_speed;
        s_vel_pred_x *= output_scale;
        s_vel_pred_y *= output_scale;
    }

    /* Public velocity remains current body-frame velocity at the full 1 kHz rate. */
    Pos_Est_vel_x = s_vel_pred_x;
    Pos_Est_vel_y = s_vel_pred_y;

    wifi_justfloat(
        acc_x_lp,
        acc_y_lp,
        yaw_rate_dps,

        flow_new_frame,
        frame_flow_x,
        frame_flow_y,
        frame_valid,

        FlowGyroDecoupler_LC302_GetDecX(),
        FlowGyroDecoupler_LC302_GetDecY(),

        g_tof_fused_height_mm,
        g_tof_fused_valid,
        g_imu_shock_flag,
        accel_bias_locked,

        Pos_Est_vel_x,
        Pos_Est_vel_y
    );

}
