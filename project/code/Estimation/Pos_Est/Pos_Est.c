#include "Pos_Est.h"

#include "../Height_Est/Height_Est.h"
#include "../Attitude/IMU_TOP.h"
#include "Accel_Calibration.h"
#include "../../HW_Drivers/PMW3901/PMW3901.h"
#include <math.h>

typedef struct
{
    float accel_bias_x_mps2;
    float accel_bias_y_mps2;
    float flow_vx_lpf_mps;
    float flow_vy_lpf_mps;
    float flow_pos_x_m;
    float flow_pos_y_m;
    float flow_dead_time_s;
    float prev_tan_pitch;
    float prev_tan_roll;
    uint8_t flow_ref_ready;
} PosEstState_t;

volatile PosEstOutput_t g_pos_est_output = {0};
volatile PosEstDebug_t g_pos_est_debug = {0};

static PosEstState_t s_pos_est_state = {0};

static float Pos_Est_Clampf(float value, float min_value, float max_value)
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

static float Pos_Est_Absf(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }
    return value;
}

static float Pos_Est_TanLimited(float rad)
{
    float value = tanf(rad);
    return Pos_Est_Clampf(value, -POS_EST_TILT_TAN_LIMIT, POS_EST_TILT_TAN_LIMIT);
}

static void Pos_Est_ClearFlowDebug(void)
{
    g_pos_est_debug.flow_pix_x_corr = 0.0f;
    g_pos_est_debug.flow_pix_y_corr = 0.0f;
    g_pos_est_debug.flow_vx_mps = 0.0f;
    g_pos_est_debug.flow_vy_mps = 0.0f;
}

static void Pos_Est_HandleFlowInvalid(uint8_t gate_state)
{
    g_pos_est_output.flow_valid = 0U;
    g_pos_est_debug.flow_gate_state = gate_state;
    Pos_Est_ClearFlowDebug();

    s_pos_est_state.flow_dead_time_s += POS_EST_DT_100HZ_S;
    if (s_pos_est_state.flow_dead_time_s > POS_EST_FLOW_DEAD_MAX_S)
    {
        g_pos_est_output.velocity_x_mps *= POS_EST_FLOW_DEAD_VEL_DAMP_RATIO;
        g_pos_est_output.velocity_y_mps *= POS_EST_FLOW_DEAD_VEL_DAMP_RATIO;
    }
}

void Pos_Est_Init(void)
{
    s_pos_est_state.accel_bias_x_mps2 = 0.0f;
    s_pos_est_state.accel_bias_y_mps2 = 0.0f;
    s_pos_est_state.flow_vx_lpf_mps = 0.0f;
    s_pos_est_state.flow_vy_lpf_mps = 0.0f;
    s_pos_est_state.flow_pos_x_m = 0.0f;
    s_pos_est_state.flow_pos_y_m = 0.0f;
    s_pos_est_state.flow_dead_time_s = 0.0f;
    s_pos_est_state.prev_tan_pitch = 0.0f;
    s_pos_est_state.prev_tan_roll = 0.0f;
    s_pos_est_state.flow_ref_ready = 0U;

    g_pos_est_output.position_x_m = 0.0f;
    g_pos_est_output.position_y_m = 0.0f;
    g_pos_est_output.velocity_x_mps = 0.0f;
    g_pos_est_output.velocity_y_mps = 0.0f;
    g_pos_est_output.height_m = 0.0f;
    g_pos_est_output.flow_valid = 0U;

    g_pos_est_debug.raw_flow_dx_count = 0;
    g_pos_est_debug.raw_flow_dy_count = 0;
    g_pos_est_debug.raw_flow_squal = 0U;
    g_pos_est_debug.flow_gate_state = POS_EST_FLOW_GATE_PASS;
    Pos_Est_ClearFlowDebug();
    g_pos_est_debug.accel_bias_x_mps2 = 0.0f;
    g_pos_est_debug.accel_bias_y_mps2 = 0.0f;
}

void Pos_Est_Update_250HZ(void)
{
    float accel_x_mps2;
    float accel_y_mps2;
    float body_gx_dps;
    float body_gy_dps;
    float body_gz_dps;
    float bias_alpha;
    float ax_use_mps2;
    float ay_use_mps2;

    AccelCalibration_GetHorizontalAccelMps2(&accel_x_mps2, &accel_y_mps2);
    AccelCalibration_GetBodyGyroDps(&body_gx_dps, &body_gy_dps, &body_gz_dps);

    if ((Pos_Est_Absf(accel_x_mps2) < POS_EST_STATIC_ACCEL_TH_MPS2) &&
        (Pos_Est_Absf(accel_y_mps2) < POS_EST_STATIC_ACCEL_TH_MPS2) &&
        (Pos_Est_Absf(body_gx_dps) < POS_EST_STATIC_GYRO_TH_DPS) &&
        (Pos_Est_Absf(body_gy_dps) < POS_EST_STATIC_GYRO_TH_DPS) &&
        (Pos_Est_Absf(body_gz_dps) < POS_EST_STATIC_GYRO_TH_DPS))
    {
        bias_alpha = POS_EST_DT_250HZ_S / (POS_EST_ACCEL_BIAS_TC_S + POS_EST_DT_250HZ_S);
        s_pos_est_state.accel_bias_x_mps2 +=
            bias_alpha * (accel_x_mps2 - s_pos_est_state.accel_bias_x_mps2);
        s_pos_est_state.accel_bias_y_mps2 +=
            bias_alpha * (accel_y_mps2 - s_pos_est_state.accel_bias_y_mps2);
    }

    s_pos_est_state.accel_bias_x_mps2 =
        Pos_Est_Clampf(s_pos_est_state.accel_bias_x_mps2,
                       -POS_EST_ACCEL_BIAS_MAX_MPS2,
                       POS_EST_ACCEL_BIAS_MAX_MPS2);
    s_pos_est_state.accel_bias_y_mps2 =
        Pos_Est_Clampf(s_pos_est_state.accel_bias_y_mps2,
                       -POS_EST_ACCEL_BIAS_MAX_MPS2,
                       POS_EST_ACCEL_BIAS_MAX_MPS2);

    ax_use_mps2 = accel_x_mps2 - s_pos_est_state.accel_bias_x_mps2;
    ay_use_mps2 = accel_y_mps2 - s_pos_est_state.accel_bias_y_mps2;

    g_pos_est_output.velocity_x_mps += ax_use_mps2 * POS_EST_DT_250HZ_S;
    g_pos_est_output.velocity_y_mps += ay_use_mps2 * POS_EST_DT_250HZ_S;

    g_pos_est_output.position_x_m += g_pos_est_output.velocity_x_mps * POS_EST_DT_250HZ_S;
    g_pos_est_output.position_y_m += g_pos_est_output.velocity_y_mps * POS_EST_DT_250HZ_S;

    g_pos_est_output.velocity_x_mps =
        Pos_Est_Clampf(g_pos_est_output.velocity_x_mps,
                       -POS_EST_VELOCITY_MAX_MPS,
                       POS_EST_VELOCITY_MAX_MPS);
    g_pos_est_output.velocity_y_mps =
        Pos_Est_Clampf(g_pos_est_output.velocity_y_mps,
                       -POS_EST_VELOCITY_MAX_MPS,
                       POS_EST_VELOCITY_MAX_MPS);

    g_pos_est_debug.accel_bias_x_mps2 = s_pos_est_state.accel_bias_x_mps2;
    g_pos_est_debug.accel_bias_y_mps2 = s_pos_est_state.accel_bias_y_mps2;
    g_pos_est_output.height_m = g_height_est_m;
}

void Pos_Est_Update_100HZ(void)
{
    float height_m;
    float roll_rad;
    float pitch_rad;
    float tan_pitch;
    float tan_roll;
    float dtan_pitch;
    float dtan_roll;
    float pix_x;
    float pix_y;
    float pix_x_corr;
    float pix_y_corr;
    float meter_per_count;
    float flow_vx_raw_mps;
    float flow_vy_raw_mps;
    float flow_vx_lpf_candidate_mps;
    float flow_vy_lpf_candidate_mps;
    float evx;
    float evy;
    float epx;
    float epy;

    PMW3901_Update();

    g_pos_est_debug.raw_flow_dx_count = g_pmw3901_raw.deltaX;
    g_pos_est_debug.raw_flow_dy_count = g_pmw3901_raw.deltaY;
    g_pos_est_debug.raw_flow_squal = g_pmw3901_raw.squal;

    height_m = g_height_est_m;
    g_pos_est_output.height_m = height_m;

    pitch_rad = g_euler.pitch * 0.01745329251994f;
    roll_rad = g_euler.roll * 0.01745329251994f;
    tan_pitch = Pos_Est_TanLimited(pitch_rad);
    tan_roll = Pos_Est_TanLimited(roll_rad);
    if (s_pos_est_state.flow_ref_ready == 0U)
    {
        s_pos_est_state.prev_tan_pitch = tan_pitch;
        s_pos_est_state.prev_tan_roll = tan_roll;
        dtan_pitch = 0.0f;
        dtan_roll = 0.0f;
    }
    else
    {
        dtan_pitch = tan_pitch - s_pos_est_state.prev_tan_pitch;
        dtan_roll = tan_roll - s_pos_est_state.prev_tan_roll;
    }

    if ((g_height_est_valid == 0U) ||
        (height_m < POS_EST_FLOW_HEIGHT_MIN_M) ||
        (height_m > POS_EST_FLOW_HEIGHT_MAX_M))
    {
        s_pos_est_state.prev_tan_pitch = tan_pitch;
        s_pos_est_state.prev_tan_roll = tan_roll;
        Pos_Est_HandleFlowInvalid(POS_EST_FLOW_GATE_HEIGHT_INVALID);
        return;
    }

    if (g_pmw3901_raw.squal < POS_EST_FLOW_SQUAL_MIN)
    {
        s_pos_est_state.prev_tan_pitch = tan_pitch;
        s_pos_est_state.prev_tan_roll = tan_roll;
        Pos_Est_HandleFlowInvalid(POS_EST_FLOW_GATE_SQUAL_LOW);
        return;
    }

    if ((g_pmw3901_raw.deltaX > POS_EST_FLOW_PIX_MAX) ||
        (g_pmw3901_raw.deltaX < -POS_EST_FLOW_PIX_MAX) ||
        (g_pmw3901_raw.deltaY > POS_EST_FLOW_PIX_MAX) ||
        (g_pmw3901_raw.deltaY < -POS_EST_FLOW_PIX_MAX))
    {
        s_pos_est_state.prev_tan_pitch = tan_pitch;
        s_pos_est_state.prev_tan_roll = tan_roll;
        Pos_Est_HandleFlowInvalid(POS_EST_FLOW_GATE_DELTA_OVERFLOW);
        return;
    }

    if ((g_pmw3901_raw.deltaX == 0) && (g_pmw3901_raw.deltaY == 0))
    {
        s_pos_est_state.prev_tan_pitch = tan_pitch;
        s_pos_est_state.prev_tan_roll = tan_roll;
        Pos_Est_HandleFlowInvalid(POS_EST_FLOW_GATE_DELTA_ZERO);
        return;
    }

    pix_x = (float)g_pmw3901_raw.deltaY;
    pix_y = (float)g_pmw3901_raw.deltaX;

    // 使用 dTan 去耦：补偿旋转引入的伪光流，避免静态倾角持续注入速度
    pix_x_corr = pix_x +
                 POS_EST_K_DTILT_X_PITCH_DEFAULT * dtan_pitch +
                 POS_EST_K_DTILT_X_ROLL_DEFAULT * dtan_roll;
    pix_y_corr = pix_y +
                 POS_EST_K_DTILT_Y_PITCH_DEFAULT * dtan_pitch +
                 POS_EST_K_DTILT_Y_ROLL_DEFAULT * dtan_roll;

    meter_per_count = POS_EST_K_PIX_TO_M_AT_1M_DEFAULT * height_m;
    flow_vx_raw_mps = (pix_x_corr * meter_per_count) / POS_EST_DT_100HZ_S;
    flow_vy_raw_mps = (pix_y_corr * meter_per_count) / POS_EST_DT_100HZ_S;
    // printf("%f,%f,%f,%f,%f,%f\r\n",pix_x_corr, pix_y_corr,pix_x,pix_y,dtan_pitch,dtan_roll);
    flow_vx_raw_mps = Pos_Est_Clampf(flow_vx_raw_mps,
                                     -POS_EST_FLOW_VEL_MAX_MPS,
                                     POS_EST_FLOW_VEL_MAX_MPS);
    flow_vy_raw_mps = Pos_Est_Clampf(flow_vy_raw_mps,
                                     -POS_EST_FLOW_VEL_MAX_MPS,
                                     POS_EST_FLOW_VEL_MAX_MPS);

    flow_vx_lpf_candidate_mps = s_pos_est_state.flow_vx_lpf_mps +
        POS_EST_FLOW_VEL_LPF_ALPHA * (flow_vx_raw_mps - s_pos_est_state.flow_vx_lpf_mps);
    flow_vy_lpf_candidate_mps = s_pos_est_state.flow_vy_lpf_mps +
        POS_EST_FLOW_VEL_LPF_ALPHA * (flow_vy_raw_mps - s_pos_est_state.flow_vy_lpf_mps);

    evx = flow_vx_lpf_candidate_mps - g_pos_est_output.velocity_x_mps;
    evy = flow_vy_lpf_candidate_mps - g_pos_est_output.velocity_y_mps;

    if ((Pos_Est_Absf(evx) > POS_EST_FLOW_INNOV_GATE_MPS) ||
        (Pos_Est_Absf(evy) > POS_EST_FLOW_INNOV_GATE_MPS))
    {
        s_pos_est_state.prev_tan_pitch = tan_pitch;
        s_pos_est_state.prev_tan_roll = tan_roll;
        Pos_Est_HandleFlowInvalid(POS_EST_FLOW_GATE_INNOV_REJECT);
        return;
    }

    s_pos_est_state.flow_vx_lpf_mps = flow_vx_lpf_candidate_mps;
    s_pos_est_state.flow_vy_lpf_mps = flow_vy_lpf_candidate_mps;

    if (s_pos_est_state.flow_ref_ready == 0U)
    {
        s_pos_est_state.flow_pos_x_m = g_pos_est_output.position_x_m;
        s_pos_est_state.flow_pos_y_m = g_pos_est_output.position_y_m;
        s_pos_est_state.flow_ref_ready = 1U;
    }

    s_pos_est_state.flow_pos_x_m += s_pos_est_state.flow_vx_lpf_mps * POS_EST_DT_100HZ_S;
    s_pos_est_state.flow_pos_y_m += s_pos_est_state.flow_vy_lpf_mps * POS_EST_DT_100HZ_S;

    epx = s_pos_est_state.flow_pos_x_m - g_pos_est_output.position_x_m;
    epy = s_pos_est_state.flow_pos_y_m - g_pos_est_output.position_y_m;

    g_pos_est_output.velocity_x_mps += POS_EST_W_V_DEFAULT * evx;
    g_pos_est_output.velocity_y_mps += POS_EST_W_V_DEFAULT * evy;
    g_pos_est_output.position_x_m += POS_EST_W_P_DEFAULT * epx;
    g_pos_est_output.position_y_m += POS_EST_W_P_DEFAULT * epy;

    s_pos_est_state.accel_bias_x_mps2 += POS_EST_K_B_DEFAULT * evx * POS_EST_DT_100HZ_S;
    s_pos_est_state.accel_bias_y_mps2 += POS_EST_K_B_DEFAULT * evy * POS_EST_DT_100HZ_S;
    s_pos_est_state.accel_bias_x_mps2 =
        Pos_Est_Clampf(s_pos_est_state.accel_bias_x_mps2,
                       -POS_EST_ACCEL_BIAS_MAX_MPS2,
                       POS_EST_ACCEL_BIAS_MAX_MPS2);
    s_pos_est_state.accel_bias_y_mps2 =
        Pos_Est_Clampf(s_pos_est_state.accel_bias_y_mps2,
                       -POS_EST_ACCEL_BIAS_MAX_MPS2,
                       POS_EST_ACCEL_BIAS_MAX_MPS2);

    s_pos_est_state.flow_dead_time_s = 0.0f;
    s_pos_est_state.prev_tan_pitch = tan_pitch;
    s_pos_est_state.prev_tan_roll = tan_roll;

    g_pos_est_output.flow_valid = 1U;
    g_pos_est_debug.flow_gate_state = POS_EST_FLOW_GATE_PASS;
    g_pos_est_debug.flow_pix_x_corr = pix_x_corr;
    g_pos_est_debug.flow_pix_y_corr = pix_y_corr;
    g_pos_est_debug.flow_vx_mps = s_pos_est_state.flow_vx_lpf_mps;
    g_pos_est_debug.flow_vy_mps = s_pos_est_state.flow_vy_lpf_mps;
    g_pos_est_debug.accel_bias_x_mps2 = s_pos_est_state.accel_bias_x_mps2;
    g_pos_est_debug.accel_bias_y_mps2 = s_pos_est_state.accel_bias_y_mps2;
}

