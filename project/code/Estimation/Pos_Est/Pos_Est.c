#include "Pos_Est.h"
#include "zf_common_headfile.h"
#include <math.h>

typedef struct
{
    float accel_bias_x_mps2;
    float accel_bias_y_mps2;
    float accel_lpf_x_mps2;
    float accel_lpf_y_mps2;
    float accel_vibe_metric;
    float acc_weight_xy;
    float flow_vx_lpf_mps;
    float flow_vy_lpf_mps;
    float flow_pos_x_m;
    float flow_pos_y_m;
    float flow_dead_time_s;
    uint8_t flow_ref_ready;
} PosEstState_t;

volatile PosEstOutput_t g_pos_est_output = {0};
volatile PosEstDebug_t g_pos_est_debug = {0};

static PosEstState_t s_pos_est_state = {0};

/* 2000Hz陀螺仪角度累积器（pitch轴，单位 deg），每100Hz周期读取并清零 */
static volatile float s_gyro_pitch_accum_deg = 0.0f;
/* 2000Hz陀螺仪角度累积器（roll轴，单位 deg），每100Hz周期读取并清零 */
static volatile float s_gyro_roll_accum_deg  = 0.0f;
/* 前一100Hz帧的pitch累积角度（deg），用于补偿PMW3901管线延迟 */
static float s_prev_gyro_pitch_deg = 0.0f;
/* 前一100Hz帧的roll累积角度（deg），用于补偿PMW3901管线延迟 */
static float s_prev_gyro_roll_deg  = 0.0f;
/* 补偿后像素X轴IIR低通滤波状态，抑制姿态解耦残差噪声 */
static float s_pix_corr_lpf_x = 0.0f;
/* 补偿后像素Y轴IIR低通滤波状态，抑制姿态解耦残差噪声 */
static float s_pix_corr_lpf_y = 0.0f;

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

    /* 光流无效期间重置像素域IIR状态，避免恢复时跳变 */
    s_pix_corr_lpf_x = 0.0f;
    s_pix_corr_lpf_y = 0.0f;

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
    s_pos_est_state.accel_lpf_x_mps2 = 0.0f;
    s_pos_est_state.accel_lpf_y_mps2 = 0.0f;
    s_pos_est_state.accel_vibe_metric = 0.0f;
    s_pos_est_state.acc_weight_xy = POS_EST_ACC_WEIGHT_MAX;
    s_pos_est_state.flow_vx_lpf_mps = 0.0f;
    s_pos_est_state.flow_vy_lpf_mps = 0.0f;
    s_pos_est_state.flow_pos_x_m = 0.0f;
    s_pos_est_state.flow_pos_y_m = 0.0f;
    s_pos_est_state.flow_dead_time_s = 0.0f;
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

    /* 初始化光流用陀螺仪角度积分状态 */
    s_gyro_pitch_accum_deg = 0.0f;
    s_gyro_roll_accum_deg  = 0.0f;
    s_prev_gyro_pitch_deg  = 0.0f;
    s_prev_gyro_roll_deg   = 0.0f;
    /* 初始化补偿后像素域IIR滤波状态 */
    s_pix_corr_lpf_x = 0.0f;
    s_pix_corr_lpf_y = 0.0f;
}

/**
 * @brief  2000Hz陀螺仪角度积分更新
 *         每个2000Hz周期将 pitch/roll 角速度乘以 dt(0.5ms) 累加到角度累积器，
 *         等效于矩形窗平均滤波，在 100Hz 中读取时自然得到该帧内的旋转角度(deg)
 *
 * @param  无
 * @return 无，结果累加到 s_gyro_pitch_accum_deg / s_gyro_roll_accum_deg
 */
void Pos_Est_Update_2000HZ(void)
{
    /* 角度积分：deg += deg/s * s */
    s_gyro_pitch_accum_deg += g_imu_filter.gyro_filt_y * POS_EST_DT_2000HZ_S;
    s_gyro_roll_accum_deg  += g_imu_filter.gyro_filt_x * POS_EST_DT_2000HZ_S;
}

void Pos_Est_Update_250HZ(void)
{
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_hp_x_mps2;
    float accel_hp_y_mps2;
    float vibe_inst;
    float rc_alpha;
    float vibe_norm;
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

    s_pos_est_state.accel_lpf_x_mps2 +=
        POS_EST_ACC_LPF_ALPHA_250HZ * (accel_x_mps2 - s_pos_est_state.accel_lpf_x_mps2);
    s_pos_est_state.accel_lpf_y_mps2 +=
        POS_EST_ACC_LPF_ALPHA_250HZ * (accel_y_mps2 - s_pos_est_state.accel_lpf_y_mps2);
    accel_hp_x_mps2 = accel_x_mps2 - s_pos_est_state.accel_lpf_x_mps2;
    accel_hp_y_mps2 = accel_y_mps2 - s_pos_est_state.accel_lpf_y_mps2;
    vibe_inst = Pos_Est_Absf(accel_hp_x_mps2) + Pos_Est_Absf(accel_hp_y_mps2);
    rc_alpha = POS_EST_DT_250HZ_S / (POS_EST_ACC_VIBE_RC_250HZ + POS_EST_DT_250HZ_S);
    s_pos_est_state.accel_vibe_metric +=
        rc_alpha * (vibe_inst - s_pos_est_state.accel_vibe_metric);
    vibe_norm = (s_pos_est_state.accel_vibe_metric - POS_EST_ACC_VIBE_LOW) /
                (POS_EST_ACC_VIBE_HIGH - POS_EST_ACC_VIBE_LOW);
    vibe_norm = Pos_Est_Clampf(vibe_norm, 0.0f, 1.0f);
    s_pos_est_state.acc_weight_xy =
        POS_EST_ACC_WEIGHT_MAX -
        (POS_EST_ACC_WEIGHT_MAX - POS_EST_ACC_WEIGHT_MIN) * vibe_norm;

    ax_use_mps2 = accel_x_mps2 - s_pos_est_state.accel_bias_x_mps2;
    ay_use_mps2 = accel_y_mps2 - s_pos_est_state.accel_bias_y_mps2;
    ax_use_mps2 *= s_pos_est_state.acc_weight_xy;
    ay_use_mps2 *= s_pos_est_state.acc_weight_xy;

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
    float flow_quality_scale;
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
    float corr_boost;
    float vel_gain;
    float pos_gain;
    float curr_pitch_deg;
    float curr_roll_deg;
    float gyro_pitch_avg_dps;
    float gyro_roll_avg_dps;
    float pitch_for_comp_deg;
    float roll_for_comp_deg;

    /* 读取并重置2000Hz陀螺仪角度累积器（必须在任何 return 前执行） */
    curr_pitch_deg = s_gyro_pitch_accum_deg;
    curr_roll_deg  = s_gyro_roll_accum_deg;
    s_gyro_pitch_accum_deg = 0.0f;
    s_gyro_roll_accum_deg  = 0.0f;

    /* 等效平均角速度(deg/s)，用于调试输出 */
    gyro_pitch_avg_dps = curr_pitch_deg / POS_EST_DT_100HZ_S;
    gyro_roll_avg_dps  = curr_roll_deg  / POS_EST_DT_100HZ_S;

    /* 取前一帧累积角度用于补偿，并更新延迟缓冲 */
    pitch_for_comp_deg = s_prev_gyro_pitch_deg;
    roll_for_comp_deg  = s_prev_gyro_roll_deg;
    s_prev_gyro_pitch_deg = curr_pitch_deg;
    s_prev_gyro_roll_deg  = curr_roll_deg;

    /* 调试输出：等效平均角速度 */
    g_pos_est_debug.gyro_filt_y = gyro_pitch_avg_dps;
    g_pos_est_debug.gyro_filt_x = gyro_roll_avg_dps;

    PMW3901_Update();

    g_pos_est_debug.raw_flow_dx_count = -g_pmw3901_raw.deltaX;
    g_pos_est_debug.raw_flow_dy_count = -g_pmw3901_raw.deltaY;
    g_pos_est_debug.raw_flow_squal = g_pmw3901_raw.squal;

    height_m = g_height_est_m;
    g_pos_est_output.height_m = height_m;

    if ((g_height_est_valid == 0U) ||
        (height_m < POS_EST_FLOW_HEIGHT_MIN_M) ||
        (height_m > POS_EST_FLOW_HEIGHT_MAX_M))
    {
        Pos_Est_HandleFlowInvalid(POS_EST_FLOW_GATE_HEIGHT_INVALID);
        return;
    }

    flow_quality_scale = 1.0f;
    if (g_pmw3901_raw.squal < POS_EST_FLOW_SQUAL_MIN)
    {
        if (g_pmw3901_raw.squal < (POS_EST_FLOW_SQUAL_MIN / 2U))
        {
            Pos_Est_HandleFlowInvalid(POS_EST_FLOW_GATE_SQUAL_LOW);
            return;
        }
        flow_quality_scale = 0.5f;
    }

    if ((g_pmw3901_raw.deltaX > POS_EST_FLOW_PIX_MAX) ||
        (g_pmw3901_raw.deltaX < -POS_EST_FLOW_PIX_MAX) ||
        (g_pmw3901_raw.deltaY > POS_EST_FLOW_PIX_MAX) ||
        (g_pmw3901_raw.deltaY < -POS_EST_FLOW_PIX_MAX))
    {
        Pos_Est_HandleFlowInvalid(POS_EST_FLOW_GATE_DELTA_OVERFLOW);
        return;
    }

    pix_x = (float)g_pos_est_debug.raw_flow_dx_count; 
    pix_y = (float)g_pos_est_debug.raw_flow_dy_count;

    /* 光流姿态解耦补偿：扣除机体旋转引起的像素伪位移
     * 使用前一帧的陀螺仪累积角度(deg)，补偿PMW3901约10ms管线延迟
     * pix_x(-deltaX) 主要受 gyro_roll 影响
     * pix_y(-deltaY) 主要受 gyro_pitch 影响
     * K_angle单位 pix/deg，由实测数据最小二乘拟合，与理论值 8.31 一致 */
    pix_x_corr = pix_x +
                 POS_EST_K_FLOW_DECOUPLE_X_ROLL * roll_for_comp_deg;
    pix_y_corr = pix_y +
                 POS_EST_K_FLOW_DECOUPLE_Y_PITCH * pitch_for_comp_deg;

    /* 补偿后像素域一阶IIR低通滤波
     * 目的：抑制姿态解耦后的残差噪声 (lag=1~2自相关约0.36~0.41)
     * fc ≈ 11.5Hz @100Hz，群延迟约10ms */
    s_pix_corr_lpf_x += POS_EST_FLOW_PIX_CORR_LPF_ALPHA *
                         (pix_x_corr - s_pix_corr_lpf_x);
    s_pix_corr_lpf_y += POS_EST_FLOW_PIX_CORR_LPF_ALPHA *
                         (pix_y_corr - s_pix_corr_lpf_y);
    pix_x_corr = s_pix_corr_lpf_x;
    pix_y_corr = s_pix_corr_lpf_y;
    wifi_vofa_JustFloat(10,
                        pix_x,
                        pix_y,
                        (float)g_pmw3901_raw.deltaX,
                        (float)g_pmw3901_raw.deltaY,
                        (float)g_pmw3901_raw.squal,
                        (float)g_pmw3901_raw.motionOccured,
                        gyro_pitch_avg_dps,
                        gyro_roll_avg_dps,
                        pix_x_corr,
                        pix_y_corr);

    meter_per_count = POS_EST_K_PIX_TO_M_AT_1M_DEFAULT * height_m;
    flow_vx_raw_mps = (pix_x_corr * meter_per_count) / POS_EST_DT_100HZ_S;
    flow_vy_raw_mps = (pix_y_corr * meter_per_count) / POS_EST_DT_100HZ_S;
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

    corr_boost = 1.0f / Pos_Est_Clampf(s_pos_est_state.acc_weight_xy, 0.01f, 1.0f);
    corr_boost = Pos_Est_Clampf(corr_boost, 1.0f, POS_EST_CORR_BOOST_MAX);
    vel_gain = POS_EST_W_V_DEFAULT * flow_quality_scale * corr_boost;
    pos_gain = POS_EST_W_P_DEFAULT * flow_quality_scale * corr_boost;

    g_pos_est_output.velocity_x_mps += vel_gain * evx;
    g_pos_est_output.velocity_y_mps += vel_gain * evy;
    g_pos_est_output.position_x_m += pos_gain * epx;
    g_pos_est_output.position_y_m += pos_gain * epy;

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

    g_pos_est_output.flow_valid = 1U;
    g_pos_est_debug.flow_gate_state = POS_EST_FLOW_GATE_PASS;
    g_pos_est_debug.flow_pix_x_corr = pix_x_corr;
    g_pos_est_debug.flow_pix_y_corr = pix_y_corr;
    g_pos_est_debug.flow_vx_mps = s_pos_est_state.flow_vx_lpf_mps;
    g_pos_est_debug.flow_vy_mps = s_pos_est_state.flow_vy_lpf_mps;
    g_pos_est_debug.accel_bias_x_mps2 = s_pos_est_state.accel_bias_x_mps2;
    g_pos_est_debug.accel_bias_y_mps2 = s_pos_est_state.accel_bias_y_mps2;
}
