#include "Pos_Est.h"
#include "FlowGyroDecoupler_LC302.h"
#include "../Attitude/Accel_Calibration.h"
#include "../Attitude/IMU_Filtter.h"
#include "../Height_Est/Height_Est.h"
#include "HW_Drivers/LC302/LC302.h"
#include "FlightController/fc_params.h"
#include "FlightController/fc_start_crsf.h"
#include "Planner/car_lamp_fused.h"
#include "Planner/ProjectionCenter.h"
#include <math.h>

extern volatile uint32 tick_1000us_cnt;
extern float g_car_vel_x;
extern float g_car_vel_y;
extern float g_car_yaw;
extern float g_car_yaw_rate_dps;
extern float g_car_sync_time_ms;
extern uint32 g_car_last_update_time_ms;

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

/* 新估计器的光流校正增益。 */
#define POS_EST2_FLOW_GAIN (0.06f)
/* 车端速度数据允许的最大更新时间，单位ms。 */
#define POS_EST2_CAR_DATA_TIMEOUT_MS (200U)
/* 解耦器启动或超时重置后恢复融合所需的连续有效帧数。 */
#define POS_EST2_FLOW_REACQUIRE_FRAMES (4U)
/* 车灯距离投影中心达到该值时认为不再覆盖光流落点，单位px。 */
#define POS_EST2_COVERAGE_ZERO_RADIUS_PX (70.0f)
/* 车灯覆盖率从1衰减到0所使用的半径跨度，单位px。 */
#define POS_EST2_COVERAGE_RADIUS_SPAN_PX (50.0f)
/* 车灯面积归一化基准，单位px^2。 */
#define POS_EST2_LAMP_AREA_NOMINAL_PX2 (75.0f)
/* 车灯面积权重下限。 */
#define POS_EST2_LAMP_AREA_WEIGHT_MIN (0.6f)
/* 车灯面积权重上限。 */
#define POS_EST2_LAMP_AREA_WEIGHT_MAX (1.2f)

float Pos_Est_vel_x = 0.0f;
float Pos_Est_vel_y = 0.0f;
/* 去除车模平移速度影响后的机体X轴速度，左正，单位cm/s。 */
float Pos_Est_vel_x_2 = 0.0f;
/* 去除车模平移速度影响后的机体Y轴速度，前正，单位cm/s。 */
float Pos_Est_vel_y_2 = 0.0f;

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

/* 旧估计器发布给新估计器的本周期加速度和偏航角速度快照。 */
static float s_common_acc_x_cmss = 0.0f;
static float s_common_acc_y_cmss = 0.0f;
static float s_common_yaw_rate_dps = 0.0f;
static uint8_t s_common_accel_bias_locked = 0U;

/* 旧估计器唯一消费LC302后发布的原始积分和解耦光流快照。 */
static uint32_t s_common_flow_sequence = 0U;
static float s_common_flow_raw_x = 0.0f;
static float s_common_flow_raw_y = 0.0f;
static float s_common_flow_dec_x = 0.0f;
static float s_common_flow_dec_y = 0.0f;
static float s_common_flow_height_mm = 0.0f;
static uint32_t s_common_flow_time_ms = 0U;
static uint8_t s_common_flow_valid = 0U;
static uint8_t s_common_flow_height_valid = 0U;
static uint32_t s_common_flow_reset_sequence = 0U;

/* 新估计器独立维护的速度和光流消费状态。 */
static float s_vel_pred_x_2 = 0.0f;
static float s_vel_pred_y_2 = 0.0f;
static uint32_t s_flow_sequence_consumed_2 = 0U;
static uint32_t s_flow_last_accepted_ms_2 = 0U;
static uint32_t s_flow_reset_sequence_consumed_2 = 0U;
static uint8_t s_flow_reacquire_count_2 = 0U;
static uint8_t s_flow_invalid_count_2 = 0U;
static float s_flow_prev_observation_x_2 = 0.0f;
static float s_flow_prev_observation_y_2 = 0.0f;
static uint8_t s_flow_prev_observation_valid_2 = 0U;

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
    s_common_acc_x_cmss = 0.0f;
    s_common_acc_y_cmss = 0.0f;
    s_common_yaw_rate_dps = 0.0f;
    s_common_accel_bias_locked = 0U;
    s_common_flow_sequence = 0U;
    s_common_flow_raw_x = 0.0f;
    s_common_flow_raw_y = 0.0f;
    s_common_flow_dec_x = 0.0f;
    s_common_flow_dec_y = 0.0f;
    s_common_flow_height_mm = 0.0f;
    s_common_flow_time_ms = tick_1000us_cnt;
    s_common_flow_valid = 0U;
    s_common_flow_height_valid = 0U;
    s_common_flow_reset_sequence = 0U;
    Pos_Est_ResetFlowState(tick_1000us_cnt);
}

/*
 * 函数功能：读取最近一帧LC302原始积分和姿态解耦光流快照。
 * 输入参数：telemetry - 输出快照指针。
 * 返回值：无，结果写入telemetry指向的结构体。
 */
void Pos_Est_GetFlowTelemetry(pos_est_flow_telemetry_t *telemetry)
{
    telemetry->raw_flow_x_integral = s_common_flow_raw_x;
    telemetry->raw_flow_y_integral = s_common_flow_raw_y;
    telemetry->decoupled_flow_x = s_common_flow_dec_x;
    telemetry->decoupled_flow_y = s_common_flow_dec_y;
    telemetry->frame_valid = s_common_flow_valid;
    telemetry->data_age_ms = (s_common_flow_sequence == 0U)
                                 ? -1.0f
                                 : (float)(tick_1000us_cnt - s_common_flow_time_ms);
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

    /* 发布已经完成标定、限幅和冲击门控的公共IMU快照。 */
    s_common_acc_x_cmss = acc_x_lp;
    s_common_acc_y_cmss = acc_y_lp;
    s_common_yaw_rate_dps = yaw_rate_dps;
    s_common_accel_bias_locked = accel_bias_locked;

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

        /* 序号最后更新，保证新估计器看到的是完整的同一帧快照。 */
        s_common_flow_raw_x = (float)frame_flow_x;
        s_common_flow_raw_y = (float)frame_flow_y;
        s_common_flow_dec_x = (frame_valid != 0U) ? FlowGyroDecoupler_LC302_GetDecX() : 0.0f;
        s_common_flow_dec_y = (frame_valid != 0U) ? FlowGyroDecoupler_LC302_GetDecY() : 0.0f;
        s_common_flow_height_mm = g_tof_fused_height_mm;
        s_common_flow_time_ms = tick_1000us_cnt;
        s_common_flow_valid = frame_valid;
        s_common_flow_height_valid = g_tof_fused_valid;
        s_common_flow_sequence++;
    }

    flow_age_ms = tick_1000us_cnt - s_flow_last_arrival_ms;
    if ((flow_age_ms > POS_EST_FLOW_TIMEOUT_RESET_MS) &&
        (s_flow_decoupler_timed_out == 0U))
    {
        /* A late next frame no longer matches a fixed 20.8 ms gyro window. */
        FlowGyroDecoupler_LC302_Reinit();
        s_common_flow_reset_sequence++;
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

    float img_err_x = g_car_lamp_fused.cx - g_projection_center.cx;
    float img_err_y = g_car_lamp_fused.cy - g_projection_center.cy;
}

/*
 * 函数功能：初始化去除车模平移速度影响的并行速度估计器。
 * 输入参数：无。
 * 返回值：无。
 */
void Pos_Est_Init_2(void)
{
    Pos_Est_vel_x_2 = 0.0f;
    Pos_Est_vel_y_2 = 0.0f;
    s_vel_pred_x_2 = 0.0f;
    s_vel_pred_y_2 = 0.0f;
    s_flow_sequence_consumed_2 = s_common_flow_sequence;
    s_flow_last_accepted_ms_2 = tick_1000us_cnt;
    s_flow_reset_sequence_consumed_2 = s_common_flow_reset_sequence;
    s_flow_reacquire_count_2 = 0U;
    s_flow_invalid_count_2 = 0U;
    s_flow_prev_observation_x_2 = 0.0f;
    s_flow_prev_observation_y_2 = 0.0f;
    s_flow_prev_observation_valid_2 = 0U;
}

/*
 * 函数功能：在1000Hz下更新去除车模平移速度影响的机体速度估计值。
 * 输入参数：无。
 * 返回值：无，结果写入Pos_Est_vel_x_2和Pos_Est_vel_y_2。
 */
void Pos_Est_Update_1000HZ_2(void)
{
    float yaw_delta_rad;
    uint32_t flow_age_ms;
    float output_speed;

    if (s_common_accel_bias_locked != 0U)
    {
        s_vel_pred_x_2 = 0.0f;
        s_vel_pred_y_2 = 0.0f;
        s_flow_sequence_consumed_2 = s_common_flow_sequence;
        s_flow_reset_sequence_consumed_2 = s_common_flow_reset_sequence;
        s_flow_reacquire_count_2 = 0U;
        s_flow_invalid_count_2 = 0U;
        s_flow_prev_observation_valid_2 = 0U;
        s_flow_last_accepted_ms_2 = tick_1000us_cnt;
        Pos_Est_vel_x_2 = 0.0f;
        Pos_Est_vel_y_2 = 0.0f;
        return;
    }

    if (s_flow_reset_sequence_consumed_2 != s_common_flow_reset_sequence)
    {
        s_flow_reset_sequence_consumed_2 = s_common_flow_reset_sequence;
        s_flow_reacquire_count_2 = 0U;
        s_flow_invalid_count_2 = 0U;
        s_flow_prev_observation_valid_2 = 0U;
    }

    /* 与旧估计器一致：先旋转机体系速度，再积分机体加速度。 */
    yaw_delta_rad = s_common_yaw_rate_dps * POS_EST_DEG_TO_RAD * POS_EST_ACC_DT_S;
    Pos_Est_RotateBodyVelocity(&s_vel_pred_x_2,
                               &s_vel_pred_y_2,
                               yaw_delta_rad);
    s_vel_pred_x_2 -= s_common_acc_y_cmss * POS_EST_ACC_DT_S;
    s_vel_pred_y_2 += s_common_acc_x_cmss * POS_EST_ACC_DT_S;

    if (s_flow_sequence_consumed_2 != s_common_flow_sequence)
    {
        float height_m;
        float flow_observation_x;
        float flow_observation_y;
        float observation_speed;
        float innovation_norm;
        float continuity_norm = 0.0f;
        float car_coverage = 0.0f;
        uint8_t observation_available = 1U;

        s_flow_sequence_consumed_2 = s_common_flow_sequence;

        if ((s_common_flow_valid != 0U) &&
            (s_common_flow_height_valid != 0U) &&
            (s_common_flow_height_mm >= POS_EST_FLOW_MIN_HEIGHT_M * 1000.0f) &&
            (s_common_flow_height_mm <= VL53L1X_VALID_RANGE_MAX))
        {
            s_flow_invalid_count_2 = 0U;
            height_m = s_common_flow_height_mm * 0.001f;
            flow_observation_x = height_m * s_common_flow_dec_x * POS_EST_FLOW_TO_CMPS;
            flow_observation_y = height_m * s_common_flow_dec_y * POS_EST_FLOW_TO_CMPS;

            /* 根据车灯相对投影中心的位置和面积估计车对光流落点的覆盖率。 */
            if (g_car_lamp_fused.valid != 0U)
            {
                if ((g_projection_center.valid != 0U) &&
                    isfinite(g_car_lamp_fused.cx) &&
                    isfinite(g_car_lamp_fused.cy) &&
                    isfinite(g_car_lamp_fused.width) &&
                    isfinite(g_car_lamp_fused.length) &&
                    isfinite(g_projection_center.cx) &&
                    isfinite(g_projection_center.cy))
                {
                    float image_error_x = g_car_lamp_fused.cx - g_projection_center.cx;
                    float image_error_y = g_car_lamp_fused.cy - g_projection_center.cy;
                    float image_error_radius = Pos_Est_VectorNorm(image_error_x,
                                                                  image_error_y);
                    float center_weight = Pos_Est_ClampFloat(
                        (POS_EST2_COVERAGE_ZERO_RADIUS_PX - image_error_radius) /
                            POS_EST2_COVERAGE_RADIUS_SPAN_PX,
                        0.0f,
                        1.0f);
                    float area_weight = Pos_Est_ClampFloat(
                        g_car_lamp_fused.width * g_car_lamp_fused.length /
                            POS_EST2_LAMP_AREA_NOMINAL_PX2,
                        POS_EST2_LAMP_AREA_WEIGHT_MIN,
                        POS_EST2_LAMP_AREA_WEIGHT_MAX);

                    car_coverage = Pos_Est_ClampFloat(center_weight * area_weight,
                                                      0.0f,
                                                      1.0f);
                }
                else
                {
                    observation_available = 0U;
                }
            }

            /* 覆盖率大于0但车速已过期时，拒绝可能被车污染的光流帧。 */
            if ((observation_available != 0U) && (car_coverage > 0.0f))
            {
                if ((g_car_sync_time_ms > 0.0f) &&
                    ((tick_1000us_cnt - g_car_last_update_time_ms) <
                     POS_EST2_CAR_DATA_TIMEOUT_MS) &&
                    isfinite(g_car_vel_x) &&
                    isfinite(g_car_vel_y) &&
                    isfinite(g_car_yaw) &&
                    isfinite(g_euler.yaw))
                {
                    float yaw_diff_deg = g_car_yaw - g_euler.yaw;
                    float yaw_diff_rad;
                    float yaw_cos;
                    float yaw_sin;
                    float car_vel_left_cmps;
                    float car_vel_forward_cmps;

                    while (yaw_diff_deg > 180.0f)
                    {
                        yaw_diff_deg -= 360.0f;
                    }
                    while (yaw_diff_deg < -180.0f)
                    {
                        yaw_diff_deg += 360.0f;
                    }

                    yaw_diff_rad = yaw_diff_deg * POS_EST_DEG_TO_RAD;
                    yaw_cos = cosf(yaw_diff_rad);
                    yaw_sin = sinf(yaw_diff_rad);
                    car_vel_left_cmps = -100.0f *
                                        (g_car_vel_x * yaw_cos +
                                         g_car_vel_y * yaw_sin);
                    car_vel_forward_cmps = 100.0f *
                                           (-g_car_vel_x * yaw_sin +
                                            g_car_vel_y * yaw_cos);
                    flow_observation_x += car_coverage * car_vel_left_cmps;
                    flow_observation_y += car_coverage * car_vel_forward_cmps;
                }
                else
                {
                    observation_available = 0U;
                }
            }

            observation_speed = Pos_Est_VectorNorm(flow_observation_x,
                                                   flow_observation_y);
            innovation_norm = Pos_Est_VectorNorm(flow_observation_x - s_vel_pred_x_2,
                                                 flow_observation_y - s_vel_pred_y_2);
            if (s_flow_prev_observation_valid_2 != 0U)
            {
                continuity_norm = Pos_Est_VectorNorm(
                    flow_observation_x - s_flow_prev_observation_x_2,
                    flow_observation_y - s_flow_prev_observation_y_2);
            }

            if ((observation_available != 0U) &&
                isfinite(flow_observation_x) &&
                isfinite(flow_observation_y) &&
                (observation_speed <= POS_EST_FLOW_SPEED_HARD_CMPS) &&
                (innovation_norm <= POS_EST_FLOW_INNOVATION_HARD_CMPS) &&
                ((s_flow_prev_observation_valid_2 == 0U) ||
                 (continuity_norm <= POS_EST_FLOW_CONTINUITY_HARD_CMPS)))
            {
                float innovation_x;
                float innovation_y;
                float correction_x;
                float correction_y;
                float correction_norm;

                s_flow_prev_observation_x_2 = flow_observation_x;
                s_flow_prev_observation_y_2 = flow_observation_y;
                s_flow_prev_observation_valid_2 = 1U;
                if (s_flow_reacquire_count_2 < POS_EST2_FLOW_REACQUIRE_FRAMES)
                {
                    s_flow_reacquire_count_2++;
                }

                if (s_flow_reacquire_count_2 >= POS_EST2_FLOW_REACQUIRE_FRAMES)
                {
                    innovation_x = Pos_Est_ClampFloat(
                        flow_observation_x - s_vel_pred_x_2,
                        -POS_EST_FLOW_INNOVATION_LIMIT_CMPS,
                        POS_EST_FLOW_INNOVATION_LIMIT_CMPS);
                    innovation_y = Pos_Est_ClampFloat(
                        flow_observation_y - s_vel_pred_y_2,
                        -POS_EST_FLOW_INNOVATION_LIMIT_CMPS,
                        POS_EST_FLOW_INNOVATION_LIMIT_CMPS);
                    correction_x = POS_EST2_FLOW_GAIN * innovation_x;
                    correction_y = POS_EST2_FLOW_GAIN * innovation_y;
                    correction_norm = Pos_Est_VectorNorm(correction_x, correction_y);
                    if (correction_norm > POS_EST_FLOW_CORRECTION_LIMIT_CMPS)
                    {
                        float correction_scale = POS_EST_FLOW_CORRECTION_LIMIT_CMPS /
                                                 correction_norm;
                        correction_x *= correction_scale;
                        correction_y *= correction_scale;
                    }
                    s_vel_pred_x_2 += correction_x;
                    s_vel_pred_y_2 += correction_y;
                    s_flow_last_accepted_ms_2 = s_common_flow_time_ms;
                }
            }
            else
            {
                s_flow_reacquire_count_2 = 0U;
                s_flow_prev_observation_valid_2 = 0U;
            }
        }
        else
        {
            if (s_flow_invalid_count_2 < POS_EST_FLOW_INVALID_LIMIT)
            {
                s_flow_invalid_count_2++;
            }
            if (s_flow_invalid_count_2 >= POS_EST_FLOW_INVALID_LIMIT)
            {
                s_flow_reacquire_count_2 = 0U;
                s_flow_prev_observation_valid_2 = 0U;
            }
        }
    }

    flow_age_ms = tick_1000us_cnt - s_flow_last_accepted_ms_2;
    if ((s_common_accel_bias_locked == 0U) &&
        (flow_age_ms > POS_EST_FLOW_LONG_OUTAGE_MS))
    {
        s_vel_pred_x_2 *= 0.998f;
        s_vel_pred_y_2 *= 0.998f;
    }
    else if ((s_common_accel_bias_locked == 0U) &&
             (flow_age_ms > POS_EST_FLOW_INERTIAL_HOLD_MS))
    {
        s_vel_pred_x_2 *= 0.9995f;
        s_vel_pred_y_2 *= 0.9995f;
    }

    output_speed = Pos_Est_VectorNorm(s_vel_pred_x_2, s_vel_pred_y_2);
    if (output_speed > POS_EST_FLOW_OUTPUT_LIMIT_CMPS)
    {
        float output_scale = POS_EST_FLOW_OUTPUT_LIMIT_CMPS / output_speed;
        s_vel_pred_x_2 *= output_scale;
        s_vel_pred_y_2 *= output_scale;
    }

    Pos_Est_vel_x_2 = s_vel_pred_x_2;
    Pos_Est_vel_y_2 = s_vel_pred_y_2;
}
