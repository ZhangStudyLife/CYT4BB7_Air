#include "Height_Est.h"
#include "../Attitude/Accel_Calibration.h"
#include "../../FlightController/fc_loop.h"
#include "zf_common_headfile.h"
#include "filter.h"
#include <math.h>

float g_height_meas_mm = (float)VL53L1X_VALID_RANGE_MAX;
float g_height_meas_health = 0.0f;
float g_height_tof_spread_mm = 0.0f;
float g_height_acc_up_mps2 = 0.0f;
float g_height_acc_corr_mps2 = 0.0f;
float g_height_obs_z_m = 0.0f;
float g_height_obs_vz_raw_mps = 0.0f;
float g_height_obs_residual_m = 0.0f;
float g_height_obs_weight = 0.0f;
float g_height_obs_delta_v_mps = 0.0f;
float g_height_state_mm = (float)VL53L1X_VALID_RANGE_MAX;
float g_height_gate_residual_mm = 0.0f;
uint8 g_height_meas_valid = 0U;
uint8 g_height_tof_accept_count = 0U;
uint8 g_height_tof_valid_mask = 0U;
uint8 g_height_beacon_polluted = 0U;
uint8 g_height_gate_reason = 0U;

float g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;  /* TOF 融合高度，单位 mm */
float g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;   /* 1 号 TOF 姿态补偿后高度，单位 mm */
float g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;   /* 2 号 TOF 姿态补偿后高度，单位 mm */
float g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;   /* 3 号 TOF 姿态补偿后高度，单位 mm */
float g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;   /* 4 号 TOF 姿态补偿后高度，单位 mm */
uint8 g_tof_fused_valid = 0U;                                  /* TOF 融合高度有效标志：1=有效，0=无效 */
float g_tof_fused_vz_mps = 0.0f;                               /* 高度融合速度，单位 m/s，上升为正 */
float g_height_fused_vz_mps = 0.0f;                            /* 控制环使用的高度速度，单位 m/s，上升为正 */

#define HEIGHT_EST_TOF1_INDEX           0U                  /* 1 号 TOF 数据索引 */
#define HEIGHT_EST_TOF2_INDEX           1U                  /* 2 号 TOF 数据索引 */
#define HEIGHT_EST_TOF3_INDEX           2U                  /* 3 号 TOF 数据索引 */
#define HEIGHT_EST_TOF4_INDEX           3U                  /* 4 号 TOF 数据索引 */
#define HEIGHT_EST_TOF_DT_S             0.01f               /* TOF 速度观测周期，单位 s */
#define HEIGHT_EST_HEIGHT_LPF_ALPHA     0.22120f            /* 高度输出 4Hz 一阶低通系数 */
#define HEIGHT_EST_MEAS_STEP_LIMIT_MM   35.0f               /* 正常 TOF 高度每帧最大跟随量，单位 mm */
#define HEIGHT_EST_MEAS_STEP_FAST_MM    90.0f               /* 大高度误差时每帧最大跟随量，单位 mm */
#define HEIGHT_EST_MEAS_FAST_ERR_MM     250.0f              /* 进入快速跟随的高度误差，单位 mm */
#define HEIGHT_EST_HEIGHT_MISS_MAX      15U                 /* 高度估计器允许 TOF 短时丢失的 100Hz 次数 */
#define HEIGHT_EST_VZ_LPF_ALPHA         0.22120f            /* 控制速度输出 4Hz 一阶低通系数 */
#define HEIGHT_EST_VZ_OBS_LIMIT_MPS     1.5f                /* 控制速度观测器输出速度限幅，单位 m/s */
#define HEIGHT_EST_TOF_TRIM_GATE_MM     110.0f              /* 控制速度观测器 TOF 截尾门限，单位 mm */
#define HEIGHT_EST_TOF1_BIAS_MM         (-14.18f)            /* 1 号 TOF 相对偏置，单位 mm */
#define HEIGHT_EST_TOF2_BIAS_MM         (14.33f)             /* 2 号 TOF 相对偏置，单位 mm */
#define HEIGHT_EST_TOF3_BIAS_MM         (-37.02f)            /* 3 号 TOF 相对偏置，单位 mm */
#define HEIGHT_EST_TOF4_BIAS_MM         (55.94f)             /* 4 号 TOF 相对偏置，单位 mm */
#define HEIGHT_EST_MEDIAN_WIN           3U                  /* 单路 TOF 中值滤波窗口长度 */
#define HEIGHT_EST_STEP_LIMIT_MM        45.0f               /* 单路 TOF 每次更新最大跳变，单位 mm */
#define HEIGHT_EST_VEL_DECAY            0.95f               /* TOF 长时间无效后的速度衰减系数 */
#define HEIGHT_EST_WEIGHT_EPS           0.001f              /* 融合权重有效下限 */
#define HEIGHT_EST_TOF_PITCH_ARM_MM     65.40f              /* TOF 相对中心的 pitch 力臂，单位 mm */
#define HEIGHT_EST_TOF_ROLL_ARM_MM      84.81f              /* TOF 相对中心的 roll 力臂，单位 mm */
#define HEIGHT_EST_TOF_VALID_MIN_MM     50.0f               /* TOF 原始测距有效最小值，单位 mm */
#define HEIGHT_EST_TOF_VALID_MAX_MM     VL53L1X_VALID_RANGE_MAX /* TOF 原始测距有效最大值，单位 mm */
#define HEIGHT_EST_TOF_MIN_COUNT        3U                  /* 高度融合最少 TOF 有效路数 */
#define HEIGHT_EST_TOF_SPREAD_GOOD_MM   150.0f              /* TOF 样本离散良好门限，单位 mm */
#define HEIGHT_EST_TOF_SPREAD_OK_MM     240.0f              /* TOF 样本离散可用门限，单位 mm */
#define HEIGHT_EST_TOF_RANGE_HIGH_MM    VL53L1X_VALID_RANGE_MAX
#define HEIGHT_EST_TOF_RANGE_HIGH_RAW_MM 1390.0f
#define HEIGHT_EST_TOF_RANGE_HIGH_ENTER_MM 1320.0f
#define HEIGHT_EST_TOF_RANGE_HIGH_EXIT_MM 1250.0f
#define HEIGHT_EST_TOF_RANGE_HIGH_MIN_STATE_MM 900.0f
#define HEIGHT_EST_TOF_RANGE_HIGH_CONFIRM_COUNT 3U
#define HEIGHT_EST_GATE_REASON_NONE     0U
#define HEIGHT_EST_GATE_REASON_BEACON   1U
#define HEIGHT_EST_GATE_REASON_SLEW     2U
#define HEIGHT_EST_GATE_REASON_REJOIN   3U
#define HEIGHT_EST_BEACON_ENTER_DROP_MM 220.0f
#define HEIGHT_EST_BEACON_SPREAD_MAX_MM HEIGHT_EST_TOF_SPREAD_OK_MM
#define HEIGHT_EST_DESCENT_RECOVER_MM   30.0f
#define HEIGHT_EST_DESCENT_FORCED_MM    55.0f
#define HEIGHT_EST_POLLUTE_DROP_MM      160.0f
#define HEIGHT_EST_POLLUTE_MEAS_MAX_MM  850.0f
#define HEIGHT_EST_POLLUTE_EXIT_RESIDUAL_MM 140.0f
#define HEIGHT_EST_POLLUTE_EXIT_LOW_RESIDUAL_MM (-220.0f)
#define HEIGHT_EST_POLLUTE_EXIT_MEAS_MM 850.0f
#define HEIGHT_EST_POLLUTE_EXIT_COUNT   8U
#define HEIGHT_EST_SOFT_REJOIN_COUNT    20U
#define HEIGHT_EST_SOFT_REJOIN_STEP_MM  8.0f
#define HEIGHT_EST_STATE_VZ_LIMIT_MPS   1.2f
#define HEIGHT_EST_POLLUTE_VZ_LIMIT_MPS 0.35f
#define HEIGHT_EST_POLLUTE_VZ_DECAY     0.94f
#define HEIGHT_EST_POLLUTE_ACC_GAIN     0.08f
#define HEIGHT_EST_STATE_VZ_BLEND       0.20f
#define HEIGHT_EST_RANGE_HIGH_OBS_COUNT 50U
#define HEIGHT_EST_RANGE_HIGH_STEP_MM   8.0f
#define HEIGHT_EST_MISS_TARGET_HOLD_COUNT 5U
#define HEIGHT_EST_MISS_TARGET_MARGIN_MM 160.0f
#define HEIGHT_EST_STATE_MAX_MM         VL53L1X_VALID_RANGE_MAX /* 高度内部状态最大值，单位 mm */

static Median_t s_tof_median[VL53L1X_SENSOR_COUNT];          /* 四路 TOF 中值滤波状态 */
static StepLim_t s_tof_step[VL53L1X_SENSOR_COUNT];           /* 四路 TOF 步进限幅状态 */
static float s_height_est_mm = (float)VL53L1X_VALID_RANGE_MAX; /* TOF 高度状态，单位 mm */
static float s_height_output_mm = (float)VL53L1X_VALID_RANGE_MAX; /* 高度输出低通状态，单位 mm */
static uint8 s_height_output_ready = 0U;                     /* 高度输出低通是否已初始化 */
static uint8 s_height_est_ready = 0U;                        /* 高度估计器是否已完成首帧初始化 */
static uint8 s_height_miss_cnt = 0U;                         /* 高度估计器 TOF 连续丢失计数 */
static LPF1_t s_height_out_vz_lpf;                          /* 控制速度输出低通滤波器，单位 m/s */
static uint8 s_height_out_vz_ready = 0U;                     /* 控制速度观测器是否已初始化 */
static float s_height_prev_output_mm = 0.0f;                 /* 上一帧高度输出，单位 mm */
static float s_height_state_vz_mps = 0.0f;
static uint8 s_tof_range_high_latched[VL53L1X_SENSOR_COUNT];
static uint8 s_tof_range_high_confirm_cnt = 0U;
static uint8 s_height_beacon_exit_cnt = 0U;
static uint8 s_height_soft_rejoin_cnt = 0U;
static uint8 s_height_range_high_cnt = 0U;
static const float s_tof_pitch_sign[VL53L1X_SENSOR_COUNT] = {1.0f, -1.0f, 1.0f, -1.0f}; /* 四路 TOF pitch 补偿极性 */
static const float s_tof_roll_sign[VL53L1X_SENSOR_COUNT] = {1.0f, 1.0f, -1.0f, -1.0f};  /* 四路 TOF roll 补偿极性 */
static const float s_tof_height_bias_mm[VL53L1X_SENSOR_COUNT] = {
    HEIGHT_EST_TOF1_BIAS_MM,
    HEIGHT_EST_TOF2_BIAS_MM,
    HEIGHT_EST_TOF3_BIAS_MM,
    HEIGHT_EST_TOF4_BIAS_MM
}; /* 四路 TOF 控制速度观测偏置，单位 mm */

/*
 * 函数功能：限制浮点数到指定范围内。
 * 输入参数：
 *   value：待限幅数值。
 *   min_value：最小允许值。
 *   max_value：最大允许值。
 * 返回值：
 *   限幅后的数值。
 */
static float HeightEst_ClampFloat(float value, float min_value, float max_value)
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

/*
 * 函数功能：限制输出高度到 TOF 有效量程内。
 * 输入参数：
 *   height_mm：待限幅高度，单位 mm。
 * 返回值：
 *   限幅后的高度，单位 mm。
 */
static float HeightEst_ClampHeightMm(float height_mm)
{
    if (height_mm < 0.0f)
    {
        return 0.0f;
    }
    if (height_mm > (float)VL53L1X_VALID_RANGE_MAX)
    {
        return (float)VL53L1X_VALID_RANGE_MAX;
    }
    return height_mm;
}

/*
 * 函数功能：限制高度估计内部状态范围。
 * 输入参数：
 *   height_mm：待限幅高度状态，单位 mm。
 * 返回值：
 *   限幅后的高度状态，单位 mm。
 */
static float HeightEst_ClampStateHeightMm(float height_mm)
{
    if (height_mm < 0.0f)
    {
        return 0.0f;
    }
    if (height_mm > HEIGHT_EST_STATE_MAX_MM)
    {
        return HEIGHT_EST_STATE_MAX_MM;
    }
    return height_mm;
}

/*
 * 函数功能：用首个健康 TOF 高度初始化高度状态。
 * 输入参数：
 *   height_mm：首帧有效融合高度，单位 mm。
 * 返回值：
 *   无
 */
static void HeightEst_InitHeightState(float height_mm)
{
    s_height_est_mm = HeightEst_ClampStateHeightMm(height_mm);
    s_height_output_mm = s_height_est_mm;
    s_height_output_ready = 1U;
    s_height_est_ready = 1U;
    s_height_miss_cnt = 0U;
    s_height_beacon_exit_cnt = 0U;
    s_height_soft_rejoin_cnt = 0U;
    s_height_range_high_cnt = 0U;
    s_height_state_vz_mps = 0.0f;
    g_height_beacon_polluted = 0U;
    g_height_gate_reason = HEIGHT_EST_GATE_REASON_NONE;
    g_height_gate_residual_mm = 0.0f;
}

/*
 * 函数功能：对称死区处理，保留超过死区后的有效加速度。
 * 输入参数：
 *   value：待处理数值。
 *   deadband：死区宽度。
 * 返回值：
 *   去除死区后的数值。
 */
static uint8 HeightEst_IsTofRangeHighRaw(uint16 distance_mm)
{
    return ((distance_mm >= (uint16)HEIGHT_EST_TOF_RANGE_HIGH_RAW_MM) ||
        (distance_mm >= (uint16)VL53L1X_INVALID_DISTANCE_MM)) ? 1U : 0U;
}

static uint8 HeightEst_IsLikelyRangeHigh(uint8 index, uint16 distance_mm, float height_mm, uint8 raw_valid)
{
    if ((0U != raw_valid) &&
        ((distance_mm >= (uint16)HEIGHT_EST_TOF_RANGE_HIGH_RAW_MM) ||
        (height_mm >= HEIGHT_EST_TOF_RANGE_HIGH_ENTER_MM)))
    {
        s_tof_range_high_latched[index] = 1U;
        return 1U;
    }

    if ((0U != raw_valid) && (height_mm <= HEIGHT_EST_TOF_RANGE_HIGH_EXIT_MM))
    {
        s_tof_range_high_latched[index] = 0U;
        return 0U;
    }

    if ((distance_mm >= (uint16)VL53L1X_INVALID_DISTANCE_MM) &&
        (0U != s_tof_range_high_latched[index]))
    {
        return 1U;
    }

    return 0U;
}

/*
 * 函数功能：根据残差大小计算控制速度观测器的 TOF 校正权重。
 * 输入参数：
 *   residual_m：TOF 高度观测减预测高度的残差，单位 m。
 * 返回值：
 *   残差权重，范围 0~1。
 */
static float HeightEst_MedianMm(float *values, uint8 count)
{
    uint8 i;
    uint8 j;
    float temp;

    for (i = 0U; i < count; i++)
    {
        for (j = i + 1U; j < count; j++)
        {
            if (values[j] < values[i])
            {
                temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }

    if (0U != (count & 1U))
    {
        return values[count >> 1U];
    }

    return 0.5f * (values[(count >> 1U) - 1U] + values[count >> 1U]);
}

static uint8 HeightEst_IsRealDescentLikely(void)
{
    uint8 target_landing = (target_height_m < 0.5f) ? 1U : 0U;
    uint8 acc_and_vz_drop =
        ((g_height_acc_up_mps2 < -2.8f) &&
        (s_height_state_vz_mps < -0.55f)) ? 1U : 0U;
    uint8 controller_drop =
        ((height_pos_out < -0.35f) &&
        (g_height_acc_up_mps2 < -1.4f) &&
        (s_height_state_vz_mps < -0.35f)) ? 1U : 0U;

    return ((0U != target_landing) ||
        (0U != acc_and_vz_drop) ||
        (0U != controller_drop)) ? 1U : 0U;
}

static uint8 HeightEst_IsLowPollution(float meas_height_mm, float residual_mm)
{
    float reference_mm = s_height_est_mm;
    uint8 low_consensus =
        ((g_height_tof_accept_count >= HEIGHT_EST_TOF_MIN_COUNT) &&
        (g_height_tof_spread_mm <= HEIGHT_EST_BEACON_SPREAD_MAX_MM)) ? 1U : 0U;

    if ((target_height_m > 0.5f) && ((target_height_m * 1000.0f) < reference_mm))
    {
        reference_mm = target_height_m * 1000.0f;
    }

    if ((0U != low_consensus) &&
        (meas_height_mm < HEIGHT_EST_POLLUTE_MEAS_MAX_MM) &&
        ((meas_height_mm - reference_mm) < -HEIGHT_EST_POLLUTE_DROP_MM) &&
        (0U == HeightEst_IsRealDescentLikely()))
    {
        return 1U;
    }

    return 0U;
}

static void HeightEst_PredictState(void)
{
    float acc_mps2 = HeightEst_ClampFloat(g_height_acc_up_mps2, -4.0f, 4.0f);
    float target_mm;
    float lower_hold_mm;

    if (0U != g_height_beacon_polluted)
    {
        s_height_state_vz_mps =
            (HEIGHT_EST_POLLUTE_VZ_DECAY * s_height_state_vz_mps) +
            (HEIGHT_EST_POLLUTE_ACC_GAIN * acc_mps2 * HEIGHT_EST_TOF_DT_S);
        s_height_state_vz_mps = HeightEst_ClampFloat(s_height_state_vz_mps,
            -HEIGHT_EST_POLLUTE_VZ_LIMIT_MPS, HEIGHT_EST_POLLUTE_VZ_LIMIT_MPS);
    }
    else
    {
        s_height_state_vz_mps =
            (0.985f * s_height_state_vz_mps) +
            (0.15f * acc_mps2 * HEIGHT_EST_TOF_DT_S);
        if (0U != s_height_soft_rejoin_cnt)
        {
            s_height_state_vz_mps = HeightEst_ClampFloat(s_height_state_vz_mps,
                -HEIGHT_EST_POLLUTE_VZ_LIMIT_MPS, HEIGHT_EST_POLLUTE_VZ_LIMIT_MPS);
        }
        else
        {
            s_height_state_vz_mps = HeightEst_ClampFloat(s_height_state_vz_mps,
                -HEIGHT_EST_STATE_VZ_LIMIT_MPS, HEIGHT_EST_STATE_VZ_LIMIT_MPS);
        }
    }

    s_height_est_mm = HeightEst_ClampStateHeightMm(s_height_est_mm +
        (s_height_state_vz_mps * HEIGHT_EST_TOF_DT_S * 1000.0f));

    if ((s_height_miss_cnt >= HEIGHT_EST_MISS_TARGET_HOLD_COUNT) &&
        (target_height_m > 0.5f) &&
        (0U == HeightEst_IsRealDescentLikely()))
    {
        target_mm = target_height_m * 1000.0f;
        lower_hold_mm = target_mm - HEIGHT_EST_MISS_TARGET_MARGIN_MM;

        if (s_height_est_mm < lower_hold_mm)
        {
            s_height_est_mm = lower_hold_mm;
            if (s_height_state_vz_mps < 0.0f)
            {
                s_height_state_vz_mps = 0.0f;
            }
        }
    }
}

/*
 * 函数功能：使用四路 TOF 偏置补偿后的截尾均值生成统一高度观测和健康度。
 * 输入参数：
 *   tof_height_mm：四路姿态补偿后的 TOF 高度，单位 mm。
 *   tof_valid：四路 TOF 有效标志。
 *   meas_height_mm：输出观测高度，单位 mm。
 *   health：输出 TOF 健康度，范围 0~1。
 * 返回值：
 *   1=观测有效，0=观测无效。
 */
static uint8 HeightEst_BuildTofMeasure(const float *tof_height_mm, const uint8 *tof_valid,
    const uint8 *tof_range_high, float *meas_height_mm, float *health, uint8 *range_high_active)
{
    float sample[VL53L1X_SENSOR_COUNT];
    float center_mm;
    float sum_mm = 0.0f;
    float spread_mm;
    float spread_weight;
    uint8 sample_count = 0U;
    uint8 accept_count = 0U;
    uint8 range_high_count = 0U;
    uint8 index;

    g_height_tof_spread_mm = 0.0f;
    g_height_tof_accept_count = 0U;
    g_height_tof_valid_mask = 0U;
    *range_high_active = 0U;

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        if (0U != tof_range_high[index])
        {
            sample[sample_count++] = HEIGHT_EST_TOF_RANGE_HIGH_MM - s_tof_height_bias_mm[index];
            g_height_tof_valid_mask |= (uint8)(1U << index);
            range_high_count++;
        }
        else if (0U != tof_valid[index])
        {
            sample[sample_count++] = tof_height_mm[index] - s_tof_height_bias_mm[index];
            g_height_tof_valid_mask |= (uint8)(1U << index);
        }
    }

    *health = 0.0f;
    if (sample_count < HEIGHT_EST_TOF_MIN_COUNT)
    {
        return 0U;
    }

    if (range_high_count >= HEIGHT_EST_TOF_MIN_COUNT)
    {
        *meas_height_mm = HEIGHT_EST_TOF_RANGE_HIGH_MM;
        *health = 1.0f;
        *range_high_active = 1U;
        g_height_tof_spread_mm = 0.0f;
        g_height_tof_accept_count = range_high_count;
        return 1U;
    }

    center_mm = HeightEst_MedianMm(sample, sample_count);
    spread_mm = sample[sample_count - 1U] - sample[0U];
    g_height_tof_spread_mm = spread_mm;
    if (spread_mm > HEIGHT_EST_TOF_SPREAD_OK_MM)
    {
        return 0U;
    }

    spread_weight = 1.0f;
    if (spread_mm > HEIGHT_EST_TOF_SPREAD_GOOD_MM)
    {
        spread_weight = (HEIGHT_EST_TOF_SPREAD_OK_MM - spread_mm) /
            (HEIGHT_EST_TOF_SPREAD_OK_MM - HEIGHT_EST_TOF_SPREAD_GOOD_MM);
    }
    *health = ((float)sample_count / (float)VL53L1X_SENSOR_COUNT) * spread_weight;

    for (index = 0U; index < sample_count; index++)
    {
        if (fabsf(sample[index] - center_mm) <= HEIGHT_EST_TOF_TRIM_GATE_MM)
        {
            sum_mm += sample[index];
            accept_count++;
        }
    }

    if (accept_count < 2U)
    {
        *meas_height_mm = center_mm;
        g_height_tof_accept_count = accept_count;
        return 1U;
    }

    *meas_height_mm = sum_mm / (float)accept_count;
    g_height_tof_accept_count = accept_count;
    return 1U;
}

/*
 * 函数功能：使用健康门控 TOF 观测更新高度状态。
 * 输入参数：
 *   meas_height_mm：TOF 融合观测高度，单位 mm。
 *   meas_health：TOF 健康度，范围 0~1。
 *   meas_valid：TOF 融合观测有效标志，1=有效，0=无效。
 * 返回值：
 *   1=高度估计有效，0=高度估计无效。
 */
static uint8 HeightEst_UpdateHeight(float meas_height_mm, float meas_health, uint8 meas_valid, uint8 range_high_active)
{
    float residual_mm;
    float step_limit_mm;
    float state_before_update_mm;
    uint8 low_polluted = 0U;
    uint8 soft_rejoin_active = 0U;
    uint8 height_valid = 1U;

    (void)meas_health;

    if (0U == s_height_est_ready)
    {
        if (0U == meas_valid)
        {
            return 0U;
        }

        HeightEst_InitHeightState(meas_height_mm);
        s_height_state_vz_mps = 0.0f;
        return 1U;
    }

    HeightEst_PredictState();
    g_height_gate_reason = HEIGHT_EST_GATE_REASON_NONE;

    if (0U != meas_valid)
    {
        if (0U != range_high_active)
        {
            residual_mm = HEIGHT_EST_TOF_RANGE_HIGH_MM - s_height_est_mm;
            if (s_height_range_high_cnt < HEIGHT_EST_RANGE_HIGH_OBS_COUNT)
            {
                s_height_range_high_cnt++;
            }
            /* 高量程只是 h>=量程上界。只有持续成立时，才慢慢承认已经超过量程。 */
            if ((s_height_range_high_cnt >= HEIGHT_EST_RANGE_HIGH_OBS_COUNT) &&
                (0U == g_height_beacon_polluted) &&
                (0U == s_height_soft_rejoin_cnt) &&
                (s_height_est_mm < HEIGHT_EST_TOF_RANGE_HIGH_MM))
            {
                state_before_update_mm = s_height_est_mm;
                s_height_est_mm = HeightEst_ClampStateHeightMm(s_height_est_mm +
                    HeightEst_ClampFloat(HEIGHT_EST_TOF_RANGE_HIGH_MM - s_height_est_mm,
                    0.0f, HEIGHT_EST_RANGE_HIGH_STEP_MM));
                s_height_state_vz_mps += HEIGHT_EST_STATE_VZ_BLEND *
                    (s_height_est_mm - state_before_update_mm) * 0.001f / HEIGHT_EST_TOF_DT_S;
                s_height_state_vz_mps = HeightEst_ClampFloat(s_height_state_vz_mps,
                    -HEIGHT_EST_STATE_VZ_LIMIT_MPS, HEIGHT_EST_STATE_VZ_LIMIT_MPS);
                g_height_gate_reason = HEIGHT_EST_GATE_REASON_SLEW;
            }
            s_height_miss_cnt = 0U;
            g_height_gate_residual_mm = residual_mm;
        }
        else
        {
            s_height_range_high_cnt = 0U;
            residual_mm = meas_height_mm - s_height_est_mm;
            g_height_gate_residual_mm = residual_mm;
            low_polluted = HeightEst_IsLowPollution(meas_height_mm, residual_mm);

            if (0U != g_height_beacon_polluted)
            {
                if ((fabsf(residual_mm) <= HEIGHT_EST_POLLUTE_EXIT_RESIDUAL_MM) ||
                    ((meas_height_mm >= HEIGHT_EST_POLLUTE_EXIT_MEAS_MM) &&
                    (residual_mm >= HEIGHT_EST_POLLUTE_EXIT_LOW_RESIDUAL_MM)))
                {
                    if (s_height_beacon_exit_cnt < HEIGHT_EST_POLLUTE_EXIT_COUNT)
                    {
                        s_height_beacon_exit_cnt++;
                    }
                }
                else
                {
                    s_height_beacon_exit_cnt = 0U;
                }

                if (s_height_beacon_exit_cnt < HEIGHT_EST_POLLUTE_EXIT_COUNT)
                {
                    g_height_gate_reason = HEIGHT_EST_GATE_REASON_BEACON;
                    low_polluted = 1U;
                    s_height_miss_cnt = 0U;
                }
                else
                {
                    g_height_beacon_polluted = 0U;
                    s_height_beacon_exit_cnt = 0U;
                    s_height_soft_rejoin_cnt = HEIGHT_EST_SOFT_REJOIN_COUNT;
                }
            }
            else if (0U != low_polluted)
            {
                g_height_beacon_polluted = 1U;
                s_height_beacon_exit_cnt = 0U;
                g_height_gate_reason = HEIGHT_EST_GATE_REASON_BEACON;
                s_height_miss_cnt = 0U;
            }

            if (0U == low_polluted)
            {
                state_before_update_mm = s_height_est_mm;
                step_limit_mm = (fabsf(residual_mm) > HEIGHT_EST_MEAS_FAST_ERR_MM) ?
                    HEIGHT_EST_MEAS_STEP_FAST_MM : HEIGHT_EST_MEAS_STEP_LIMIT_MM;

                if (0U != s_height_soft_rejoin_cnt)
                {
                    soft_rejoin_active = 1U;
                    step_limit_mm = HEIGHT_EST_SOFT_REJOIN_STEP_MM;
                    g_height_gate_reason = HEIGHT_EST_GATE_REASON_REJOIN;
                    s_height_soft_rejoin_cnt--;
                }

                if (residual_mm < 0.0f)
                {
                    if ((residual_mm < -HEIGHT_EST_BEACON_ENTER_DROP_MM) &&
                        (0U == HeightEst_IsRealDescentLikely()))
                    {
                        step_limit_mm = HEIGHT_EST_DESCENT_RECOVER_MM;
                        g_height_gate_reason = HEIGHT_EST_GATE_REASON_SLEW;
                    }
                    else if (step_limit_mm > HEIGHT_EST_DESCENT_FORCED_MM)
                    {
                        step_limit_mm = HEIGHT_EST_DESCENT_FORCED_MM;
                        g_height_gate_reason = HEIGHT_EST_GATE_REASON_SLEW;
                    }
                }

                s_height_est_mm = HeightEst_ClampStateHeightMm(s_height_est_mm +
                    HeightEst_ClampFloat(residual_mm, -step_limit_mm, step_limit_mm));
                s_height_state_vz_mps += HEIGHT_EST_STATE_VZ_BLEND *
                    (s_height_est_mm - state_before_update_mm) * 0.001f / HEIGHT_EST_TOF_DT_S;
                if (0U != soft_rejoin_active)
                {
                    s_height_state_vz_mps = HeightEst_ClampFloat(s_height_state_vz_mps,
                        -HEIGHT_EST_POLLUTE_VZ_LIMIT_MPS, HEIGHT_EST_POLLUTE_VZ_LIMIT_MPS);
                }
                else
                {
                    s_height_state_vz_mps = HeightEst_ClampFloat(s_height_state_vz_mps,
                        -HEIGHT_EST_STATE_VZ_LIMIT_MPS, HEIGHT_EST_STATE_VZ_LIMIT_MPS);
                }
                s_height_miss_cnt = 0U;
            }
        }
    }
    else if (s_height_miss_cnt < HEIGHT_EST_HEIGHT_MISS_MAX)
    {
        s_height_miss_cnt++;
        g_height_gate_residual_mm = 0.0f;
    }
    else
    {
        height_valid = 0U;
        g_height_gate_residual_mm = 0.0f;
    }

    if (0U == s_height_output_ready)
    {
        s_height_output_mm = s_height_est_mm;
        s_height_output_ready = 1U;
    }
    else
    {
        s_height_output_mm += HEIGHT_EST_HEIGHT_LPF_ALPHA * (s_height_est_mm - s_height_output_mm);
    }

    return height_valid;
}

static float HeightEst_UpdateOutputVz(uint8 fused_valid)
{
    float vz_raw_mps = 0.0f;
    float vz_lpf_mps;

    g_height_acc_corr_mps2 = 0.0f;
    g_height_obs_residual_m = 0.0f;
    g_height_obs_weight = ((0U != fused_valid) && (0U == g_height_beacon_polluted)) ? 1.0f : 0.0f;
    g_height_obs_delta_v_mps = 0.0f;

    if (0U == fused_valid)
    {
        LPF1_Reset(&s_height_out_vz_lpf);
        s_height_out_vz_ready = 0U;
        g_height_obs_z_m = 0.0f;
        g_height_obs_vz_raw_mps = 0.0f;
        return 0.0f;
    }

    if ((0U != g_height_beacon_polluted) || (0U != s_height_soft_rejoin_cnt))
    {
        s_height_prev_output_mm = s_height_output_mm;
        g_height_obs_z_m = s_height_output_mm * 0.001f;
        vz_raw_mps = HeightEst_ClampFloat(s_height_state_vz_mps,
            -HEIGHT_EST_POLLUTE_VZ_LIMIT_MPS, HEIGHT_EST_POLLUTE_VZ_LIMIT_MPS);
        g_height_obs_vz_raw_mps = vz_raw_mps;
        return vz_raw_mps;
    }

    if (0U == s_height_out_vz_ready)
    {
        s_height_prev_output_mm = s_height_output_mm;
        s_height_out_vz_ready = 1U;
        LPF1_Reset(&s_height_out_vz_lpf);
        g_height_obs_z_m = s_height_output_mm * 0.001f;
        g_height_obs_vz_raw_mps = 0.0f;
        return 0.0f;
    }

    vz_raw_mps = (s_height_output_mm - s_height_prev_output_mm) * 0.001f / HEIGHT_EST_TOF_DT_S;
    s_height_prev_output_mm = s_height_output_mm;
    vz_raw_mps = HeightEst_ClampFloat(vz_raw_mps,
        -HEIGHT_EST_VZ_OBS_LIMIT_MPS, HEIGHT_EST_VZ_OBS_LIMIT_MPS);
    vz_lpf_mps = LPF1_Update(&s_height_out_vz_lpf, vz_raw_mps);
    g_height_obs_z_m = s_height_output_mm * 0.001f;
    g_height_obs_vz_raw_mps = vz_raw_mps;

    return vz_lpf_mps;
}

static void HeightEst_ResetChannel(uint8 index)
{
    Median_Reset(&s_tof_median[index]);
    StepLim_Reset(&s_tof_step[index]);
}

/*
 * 函数功能：复位高度估计模块的全部滤波和输出状态。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
static void HeightEst_ResetAll(void)
{
    uint8 index;

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        Median_Init(&s_tof_median[index], HEIGHT_EST_MEDIAN_WIN);
        StepLim_Init(&s_tof_step[index], HEIGHT_EST_STEP_LIMIT_MM);
        s_tof_range_high_latched[index] = 0U;
    }
    s_tof_range_high_confirm_cnt = 0U;

    s_height_est_mm = (float)VL53L1X_VALID_RANGE_MAX;
    s_height_output_mm = (float)VL53L1X_VALID_RANGE_MAX;
    s_height_output_ready = 0U;
    s_height_est_ready = 0U;
    s_height_miss_cnt = 0U;
    LPF1_Init(&s_height_out_vz_lpf, HEIGHT_EST_VZ_LPF_ALPHA);
    s_height_prev_output_mm = 0.0f;
    s_height_out_vz_ready = 0U;
    s_height_state_vz_mps = 0.0f;
    g_height_meas_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_height_meas_health = 0.0f;
    g_height_tof_spread_mm = 0.0f;
    g_height_acc_up_mps2 = 0.0f;
    g_height_acc_corr_mps2 = 0.0f;
    g_height_obs_z_m = 0.0f;
    g_height_obs_vz_raw_mps = 0.0f;
    g_height_obs_residual_m = 0.0f;
    g_height_obs_weight = 0.0f;
    g_height_obs_delta_v_mps = 0.0f;
    g_height_state_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_height_gate_residual_mm = 0.0f;
    g_height_meas_valid = 0U;
    g_height_tof_accept_count = 0U;
    g_height_tof_valid_mask = 0U;
    g_height_beacon_polluted = 0U;
    g_height_gate_reason = HEIGHT_EST_GATE_REASON_NONE;

    g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_tof_fused_valid = 0U;
    g_tof_fused_vz_mps = 0.0f;
    g_height_fused_vz_mps = 0.0f;
    s_height_beacon_exit_cnt = 0U;
    s_height_soft_rejoin_cnt = 0U;
    s_height_range_high_cnt = 0U;
}

/*
 * 函数功能：对单路 TOF 测距做姿态补偿、中值滤波和步进限幅。
 * 输入参数：
 *   index：TOF 通道索引，范围 0~3。
 *   distance_mm：TOF 原始测距，单位 mm。
 * 返回值：
 *   姿态补偿后的单路高度，单位 mm。
 */
static float HeightEst_ProcessChannel(uint8 index, uint16 distance_mm)
{
    float height_mm = (float)distance_mm * g_euler.cos_pitch * g_euler.cos_roll;

    height_mm += s_tof_pitch_sign[index] * HEIGHT_EST_TOF_PITCH_ARM_MM * g_euler.sin_pitch;
    height_mm += s_tof_roll_sign[index] * HEIGHT_EST_TOF_ROLL_ARM_MM * g_euler.sin_roll * g_euler.cos_pitch;

    height_mm = HeightEst_ClampHeightMm(height_mm);
    height_mm = Median_Update(&s_tof_median[index], height_mm);
    height_mm = StepLim_Update(&s_tof_step[index], height_mm);

    return HeightEst_ClampHeightMm(height_mm);
}

/*
 * 函数功能：初始化 TOF 驱动和高度估计状态。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void TOF_Init(void)
{
    HeightEst_ResetAll();
    VL53L1X_Init();
}

/*
 * 函数功能：100Hz 更新 TOF 原始测距，执行健康门控高度融合。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void TOF_update_100HZ(void)
{
    const VL53L1X_data_struct *tof_data = 0;
    uint8 meas_valid = 0U;
    uint8 height_valid = 0U;
    float meas_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float meas_health = 0.0f;
    float tof_height_mm[VL53L1X_SENSOR_COUNT] = {
        (float)VL53L1X_INVALID_DISTANCE_MM,
        (float)VL53L1X_INVALID_DISTANCE_MM,
        (float)VL53L1X_INVALID_DISTANCE_MM,
        (float)VL53L1X_INVALID_DISTANCE_MM
    };
    uint8 tof_valid[VL53L1X_SENSOR_COUNT] = {0U, 0U, 0U, 0U};
    uint8 tof_range_high[VL53L1X_SENSOR_COUNT] = {0U, 0U, 0U, 0U};
    uint8 range_high_active = 0U;
    uint8 raw_range_high_count = 0U;
    uint8 latched_range_high_count = 0U;
    float output_vz_mps = 0.0f;

    g_height_acc_up_mps2 = AccelCalibration_GetVerticalAccelUpMps2();
    VL53L1X_Update();
    tof_data = VL53L1X_GetData();

    g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_valid = 0U;

    if (0 != tof_data)
    {
        if ((0U != tof_data->valid[HEIGHT_EST_TOF1_INDEX]) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF1_INDEX] >= HEIGHT_EST_TOF_VALID_MIN_MM) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF1_INDEX] <= HEIGHT_EST_TOF_VALID_MAX_MM))
        {
            tof_height_mm[HEIGHT_EST_TOF1_INDEX] = HeightEst_ProcessChannel(HEIGHT_EST_TOF1_INDEX, tof_data->distance_mm[HEIGHT_EST_TOF1_INDEX]);
            tof_range_high[HEIGHT_EST_TOF1_INDEX] = HeightEst_IsLikelyRangeHigh(HEIGHT_EST_TOF1_INDEX,
                tof_data->distance_mm[HEIGHT_EST_TOF1_INDEX], tof_height_mm[HEIGHT_EST_TOF1_INDEX], 1U);
            tof_valid[HEIGHT_EST_TOF1_INDEX] = (0U == tof_range_high[HEIGHT_EST_TOF1_INDEX]) ? 1U : 0U;
            if (0U != tof_range_high[HEIGHT_EST_TOF1_INDEX])
            {
                tof_height_mm[HEIGHT_EST_TOF1_INDEX] = HEIGHT_EST_TOF_RANGE_HIGH_MM;
            }
        }
        else
        {
            tof_range_high[HEIGHT_EST_TOF1_INDEX] = HeightEst_IsLikelyRangeHigh(HEIGHT_EST_TOF1_INDEX,
                tof_data->distance_mm[HEIGHT_EST_TOF1_INDEX], HEIGHT_EST_TOF_RANGE_HIGH_MM, 0U);
            if (0U != tof_range_high[HEIGHT_EST_TOF1_INDEX])
            {
                tof_height_mm[HEIGHT_EST_TOF1_INDEX] = HEIGHT_EST_TOF_RANGE_HIGH_MM;
            }
            if (0U == tof_range_high[HEIGHT_EST_TOF1_INDEX])
            {
                HeightEst_ResetChannel(HEIGHT_EST_TOF1_INDEX);
            }
        }

        if ((0U != tof_data->valid[HEIGHT_EST_TOF2_INDEX]) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF2_INDEX] >= HEIGHT_EST_TOF_VALID_MIN_MM) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF2_INDEX] <= HEIGHT_EST_TOF_VALID_MAX_MM))
        {
            tof_height_mm[HEIGHT_EST_TOF2_INDEX] = HeightEst_ProcessChannel(HEIGHT_EST_TOF2_INDEX, tof_data->distance_mm[HEIGHT_EST_TOF2_INDEX]);
            tof_range_high[HEIGHT_EST_TOF2_INDEX] = HeightEst_IsLikelyRangeHigh(HEIGHT_EST_TOF2_INDEX,
                tof_data->distance_mm[HEIGHT_EST_TOF2_INDEX], tof_height_mm[HEIGHT_EST_TOF2_INDEX], 1U);
            tof_valid[HEIGHT_EST_TOF2_INDEX] = (0U == tof_range_high[HEIGHT_EST_TOF2_INDEX]) ? 1U : 0U;
            if (0U != tof_range_high[HEIGHT_EST_TOF2_INDEX])
            {
                tof_height_mm[HEIGHT_EST_TOF2_INDEX] = HEIGHT_EST_TOF_RANGE_HIGH_MM;
            }
        }
        else
        {
            tof_range_high[HEIGHT_EST_TOF2_INDEX] = HeightEst_IsLikelyRangeHigh(HEIGHT_EST_TOF2_INDEX,
                tof_data->distance_mm[HEIGHT_EST_TOF2_INDEX], HEIGHT_EST_TOF_RANGE_HIGH_MM, 0U);
            if (0U != tof_range_high[HEIGHT_EST_TOF2_INDEX])
            {
                tof_height_mm[HEIGHT_EST_TOF2_INDEX] = HEIGHT_EST_TOF_RANGE_HIGH_MM;
            }
            if (0U == tof_range_high[HEIGHT_EST_TOF2_INDEX])
            {
                HeightEst_ResetChannel(HEIGHT_EST_TOF2_INDEX);
            }
        }

        if ((0U != tof_data->valid[HEIGHT_EST_TOF3_INDEX]) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF3_INDEX] >= HEIGHT_EST_TOF_VALID_MIN_MM) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF3_INDEX] <= HEIGHT_EST_TOF_VALID_MAX_MM))
        {
            tof_height_mm[HEIGHT_EST_TOF3_INDEX] = HeightEst_ProcessChannel(HEIGHT_EST_TOF3_INDEX, tof_data->distance_mm[HEIGHT_EST_TOF3_INDEX]);
            tof_range_high[HEIGHT_EST_TOF3_INDEX] = HeightEst_IsLikelyRangeHigh(HEIGHT_EST_TOF3_INDEX,
                tof_data->distance_mm[HEIGHT_EST_TOF3_INDEX], tof_height_mm[HEIGHT_EST_TOF3_INDEX], 1U);
            tof_valid[HEIGHT_EST_TOF3_INDEX] = (0U == tof_range_high[HEIGHT_EST_TOF3_INDEX]) ? 1U : 0U;
            if (0U != tof_range_high[HEIGHT_EST_TOF3_INDEX])
            {
                tof_height_mm[HEIGHT_EST_TOF3_INDEX] = HEIGHT_EST_TOF_RANGE_HIGH_MM;
            }
        }
        else
        {
            tof_range_high[HEIGHT_EST_TOF3_INDEX] = HeightEst_IsLikelyRangeHigh(HEIGHT_EST_TOF3_INDEX,
                tof_data->distance_mm[HEIGHT_EST_TOF3_INDEX], HEIGHT_EST_TOF_RANGE_HIGH_MM, 0U);
            if (0U != tof_range_high[HEIGHT_EST_TOF3_INDEX])
            {
                tof_height_mm[HEIGHT_EST_TOF3_INDEX] = HEIGHT_EST_TOF_RANGE_HIGH_MM;
            }
            if (0U == tof_range_high[HEIGHT_EST_TOF3_INDEX])
            {
                HeightEst_ResetChannel(HEIGHT_EST_TOF3_INDEX);
            }
        }

        if ((0U != tof_data->valid[HEIGHT_EST_TOF4_INDEX]) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF4_INDEX] >= HEIGHT_EST_TOF_VALID_MIN_MM) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF4_INDEX] <= HEIGHT_EST_TOF_VALID_MAX_MM))
        {
            tof_height_mm[HEIGHT_EST_TOF4_INDEX] = HeightEst_ProcessChannel(HEIGHT_EST_TOF4_INDEX, tof_data->distance_mm[HEIGHT_EST_TOF4_INDEX]);
            tof_range_high[HEIGHT_EST_TOF4_INDEX] = HeightEst_IsLikelyRangeHigh(HEIGHT_EST_TOF4_INDEX,
                tof_data->distance_mm[HEIGHT_EST_TOF4_INDEX], tof_height_mm[HEIGHT_EST_TOF4_INDEX], 1U);
            tof_valid[HEIGHT_EST_TOF4_INDEX] = (0U == tof_range_high[HEIGHT_EST_TOF4_INDEX]) ? 1U : 0U;
            if (0U != tof_range_high[HEIGHT_EST_TOF4_INDEX])
            {
                tof_height_mm[HEIGHT_EST_TOF4_INDEX] = HEIGHT_EST_TOF_RANGE_HIGH_MM;
            }
        }
        else
        {
            tof_range_high[HEIGHT_EST_TOF4_INDEX] = HeightEst_IsLikelyRangeHigh(HEIGHT_EST_TOF4_INDEX,
                tof_data->distance_mm[HEIGHT_EST_TOF4_INDEX], HEIGHT_EST_TOF_RANGE_HIGH_MM, 0U);
            if (0U != tof_range_high[HEIGHT_EST_TOF4_INDEX])
            {
                tof_height_mm[HEIGHT_EST_TOF4_INDEX] = HEIGHT_EST_TOF_RANGE_HIGH_MM;
            }
            if (0U == tof_range_high[HEIGHT_EST_TOF4_INDEX])
            {
                HeightEst_ResetChannel(HEIGHT_EST_TOF4_INDEX);
            }
        }

        raw_range_high_count += HeightEst_IsTofRangeHighRaw(tof_data->distance_mm[HEIGHT_EST_TOF1_INDEX]);
        raw_range_high_count += HeightEst_IsTofRangeHighRaw(tof_data->distance_mm[HEIGHT_EST_TOF2_INDEX]);
        raw_range_high_count += HeightEst_IsTofRangeHighRaw(tof_data->distance_mm[HEIGHT_EST_TOF3_INDEX]);
        raw_range_high_count += HeightEst_IsTofRangeHighRaw(tof_data->distance_mm[HEIGHT_EST_TOF4_INDEX]);
        latched_range_high_count += s_tof_range_high_latched[HEIGHT_EST_TOF1_INDEX];
        latched_range_high_count += s_tof_range_high_latched[HEIGHT_EST_TOF2_INDEX];
        latched_range_high_count += s_tof_range_high_latched[HEIGHT_EST_TOF3_INDEX];
        latched_range_high_count += s_tof_range_high_latched[HEIGHT_EST_TOF4_INDEX];
        if ((raw_range_high_count >= HEIGHT_EST_TOF_MIN_COUNT) &&
            (0U != s_height_est_ready) &&
            ((latched_range_high_count > 0U) ||
            (s_height_est_mm >= HEIGHT_EST_TOF_RANGE_HIGH_MIN_STATE_MM)))
        {
            if (s_tof_range_high_confirm_cnt < HEIGHT_EST_TOF_RANGE_HIGH_CONFIRM_COUNT)
            {
                s_tof_range_high_confirm_cnt++;
            }
            if (s_tof_range_high_confirm_cnt >= HEIGHT_EST_TOF_RANGE_HIGH_CONFIRM_COUNT)
            {
                tof_range_high[HEIGHT_EST_TOF1_INDEX] = 1U;
                tof_range_high[HEIGHT_EST_TOF2_INDEX] = 1U;
                tof_range_high[HEIGHT_EST_TOF3_INDEX] = 1U;
                tof_range_high[HEIGHT_EST_TOF4_INDEX] = 1U;
                s_tof_range_high_latched[HEIGHT_EST_TOF1_INDEX] = 1U;
                s_tof_range_high_latched[HEIGHT_EST_TOF2_INDEX] = 1U;
                s_tof_range_high_latched[HEIGHT_EST_TOF3_INDEX] = 1U;
                s_tof_range_high_latched[HEIGHT_EST_TOF4_INDEX] = 1U;
                tof_height_mm[HEIGHT_EST_TOF1_INDEX] = HEIGHT_EST_TOF_RANGE_HIGH_MM;
                tof_height_mm[HEIGHT_EST_TOF2_INDEX] = HEIGHT_EST_TOF_RANGE_HIGH_MM;
                tof_height_mm[HEIGHT_EST_TOF3_INDEX] = HEIGHT_EST_TOF_RANGE_HIGH_MM;
                tof_height_mm[HEIGHT_EST_TOF4_INDEX] = HEIGHT_EST_TOF_RANGE_HIGH_MM;
                tof_valid[HEIGHT_EST_TOF1_INDEX] = 0U;
                tof_valid[HEIGHT_EST_TOF2_INDEX] = 0U;
                tof_valid[HEIGHT_EST_TOF3_INDEX] = 0U;
                tof_valid[HEIGHT_EST_TOF4_INDEX] = 0U;
            }
        }
        else
        {
            s_tof_range_high_confirm_cnt = 0U;
        }
    }
    else
    {
        HeightEst_ResetChannel(HEIGHT_EST_TOF1_INDEX);
        HeightEst_ResetChannel(HEIGHT_EST_TOF2_INDEX);
        HeightEst_ResetChannel(HEIGHT_EST_TOF3_INDEX);
        HeightEst_ResetChannel(HEIGHT_EST_TOF4_INDEX);
    }

    g_tof1_height_mm = tof_height_mm[HEIGHT_EST_TOF1_INDEX];
    g_tof2_height_mm = tof_height_mm[HEIGHT_EST_TOF2_INDEX];
    g_tof3_height_mm = tof_height_mm[HEIGHT_EST_TOF3_INDEX];
    g_tof4_height_mm = tof_height_mm[HEIGHT_EST_TOF4_INDEX];

    meas_valid = HeightEst_BuildTofMeasure(tof_height_mm, tof_valid, tof_range_high,
        &meas_height_mm, &meas_health, &range_high_active);
    g_height_meas_mm = meas_height_mm;
    g_height_meas_health = meas_health;
    g_height_meas_valid = meas_valid;
    height_valid = HeightEst_UpdateHeight(meas_height_mm, meas_health, meas_valid, range_high_active);
    g_height_state_mm = s_height_est_mm;
    g_tof_fused_valid = height_valid;
    if (0U != height_valid)
    {
        g_tof_fused_height_mm = HeightEst_ClampStateHeightMm(s_height_output_mm);
    }
    else
    {
        if (0U != s_height_output_ready)
        {
            g_tof_fused_height_mm = HeightEst_ClampStateHeightMm(s_height_output_mm);
        }
        else
        {
            g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
        }
    }

    output_vz_mps = HeightEst_UpdateOutputVz(g_tof_fused_valid);
    g_tof_fused_vz_mps = output_vz_mps;
    g_height_fused_vz_mps = output_vz_mps;
}

/*
 * 函数功能：100Hz 高度估计入口，更新 TOF 融合观测和高度状态。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void Height_Est_update_100HZ(void)
{
    TOF_update_100HZ();
}

/*
 * 函数功能：1kHz 高度估计入口，当前高度不再由加速度积分预测。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void Height_Est_update_1000HZ(void)
{
}
