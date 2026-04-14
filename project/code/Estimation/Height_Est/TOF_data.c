#include "TOF_data.h"
#include "../../HW_Drivers/Motor/Motor_Drive.h"
#include "../../filter.h"
#include "zf_common_headfile.h"
#include <math.h>

/* ==================== 物理参数宏 ==================== */

/* TOF最小有效测距，单位：mm */
#define TOF_MIN_VALID_MM                 (40U)
/* TOF最大有效测距，单位：mm */
#define TOF_MAX_VALID_MM                 (1300U)

/* 姿态补偿cos项下限，防止大倾角时距离估计过度衰减 */
#define TOF_COS_TERM_MIN                 (0.35f)
/* 允许的最大倾角（度），超过此值认为TOF可能测到侧面障碍物而非地面 */
#define TOF_MAX_TILT_DEG                 (45.0f)
/* 角度转弧度系数 */
#define TOF_DEG_TO_RAD                   (0.017453293f)

/* ==================== 融合参数宏 ==================== */

/* 多通道一致性门限，单位：mm；差距在此范围内使用快速步进 */
#define TOF_AGREE_GATE_MM                (22U)
/* 融合失效后保持上一融合值的最大帧数 */
#define TOF_FUSED_HOLD_FRAMES            (8U)
#define TOF_FUSED_KF_Q_MM2               (16.0f)
#define TOF_FUSED_KF_R_MM2               (100.0f)
#define TOF_FUSED_KF_P0_MM2              (400.0f)
/* 一致区间（所有融合通道差距<22mm）下的融合输出单步最大变化，单位：mm
 * 对应最大跟踪速度：12mm × 100Hz = 1.2m/s，满足正常飞行需求 */
#define TOF_STEP_FAST_MM                 (12U)
/* 常规区间下的融合输出单步最大变化，单位：mm */
#define TOF_STEP_SAFE_MM                 (7U)
/* 最终输出IIR低通滤波系数（0~1），兼顾实时性与平滑性
 * α=0.45：有效时间常数≈12ms；稳态噪声降低约70%；1m/s高度变化滞后≈12ms */
#define TOF_FUSED_LPF_ALPHA              (0.45f)

/* ==================== 通道质量检测宏 ==================== */

/* 判定通道"冻结"的连续相同值帧阈值 */
#define TOF_SAME_VALUE_FRAMES_TH         (25U)

/* 单通道相邻帧变化量超过此值视为突变（电缆遮挡等瞬时干扰），单位：mm
 * 依据数据分析：正常单帧最大变化约15mm，电缆干扰通常达400-500mm，此门限充分区分 */
#define TOF_SPIKE_GATE_MM                (50U)
/* 突变持续超过此帧数后接受为真实高度变化 */
#define TOF_SPIKE_PERSIST_FRAMES         (3U)

/* 多通道异常值剔除门限：距离中值超过此值视为异常，单位：mm
 * 依据数据分析：正常通道间正常波动幅度约20mm，40mm门限可拦截30-80mm中等干扰 */
#define TOF_OUTLIER_GATE_MM              (40U)

/* ==================== 校准参数宏 ==================== */

/* 标定TOF采样点数（阶段2，用于通道零偏） */
#define TOF_CALIBRATION_SAMPLES          (100U)
/* 标定IMU采样点数（阶段1，用于姿态静态偏差） */
#define TOF_ATTITUDE_CALIB_SAMPLES       (50U)
/* 标定采样间隔，单位：ms */
#define TOF_CALIBRATION_DT_MS            (10U)
/* 校准通过的最大通道间偏差，单位：mm（用户要求：10cm） */
#define TOF_CALIB_DEVIATION_MAX_MM       (100U)
#define TOF_SHADOW_TIMEOUT_FRAMES        (20U)
#define TOF_SHADOW_DROP_RESIDUAL_MM      (20.0f)
#define TOF_SHADOW_R_4CH_MM2             (400.0f)
#define TOF_SHADOW_R_3CH_MM2             (900.0f)
#define TOF_SHADOW_R_2CH_DIAG_MM2        (1600.0f)
#define TOF_SHADOW_R_2CH_EDGE_MM2        (4900.0f)
#define TOF_SHADOW_R_1CH_MM2             (10000.0f)
#define TOF_SHADOW_P0_H_MM2              (400.0f)
#define TOF_SHADOW_P0_V_MM2PS2           (40000.0f)
#define TOF_SHADOW_Q_ACC_MM2PS4          (360000.0f)
#define TOF_FUSED_TIMEOUT_FRAMES         (12U)
#define TOF_FUSED_R_4CH_MM2              (144.0f)
#define TOF_FUSED_R_3CH_MM2              (400.0f)
#define TOF_FUSED_R_2CH_MM2              (1296.0f)
#define TOF_FUSED_R_2CH_EDGE_MM2         (4900.0f)
#define TOF_FUSED_R_1CH_MM2              (10000.0f)
#define TOF_FUSED_P0_H_MM2               (400.0f)
#define TOF_FUSED_P0_V_MM2PS2            (40000.0f)
#define TOF_FUSED_Q_ACC_MM2PS4           (800000.0f)
#define TOF_FUSED_4CH_SPREAD_MM          (35U)
#define TOF_FUSED_3CH_SPREAD_MM          (30U)
#define TOF_FUSED_2CH_SPREAD_MM          (25U)
#define TOF_FUSED_2CH_SPLIT_MM           (120U)
#define TOF_FUSED_STICK_FRAMES           (6U)
#define TOF_FUSED_STICK_SPREAD_MM        (8U)
#define TOF_FUSED_STICK_PRED_MM          (15.0f)
#define TOF_FUSED_MISS_VZ_DAMP           (0.70f)
#define TOF_FUSED_MISS_VZ_STOP_MMPS      (30.0f)
/* 速度观测差分窗口长度；适当拉长可抑制量测切换帧带来的速度尖峰
 * 7帧 × 10ms = 60ms差分基线，相比5帧(40ms)可降低高空速度噪声约30% */
#define TOF_FUSED_VZ_HIST_LEN            (7U)
/* 高度自适应增益衰减起始高度，单位：mm */
#define TOF_VZ_ADAPT_H_LOW_MM            (600.0f)
/* 高度自适应增益衰减终止高度，单位：mm */
#define TOF_VZ_ADAPT_H_HIGH_MM           (1200.0f)
/* 高空时beta/gamma的最小缩放系数(0~1)；越小滤波越强、延迟越大 */
#define TOF_VZ_ADAPT_GAIN_MIN            (0.55f)
/* 相邻两帧速度观测允许逼近当前速度状态的最大变化量，单位：mm/s */
#define TOF_FUSED_VZ_DV_LIMIT_MMPS       (450.0f)
/* 量测来源掩码切换时，速度残差更新缩放系数 */
#define TOF_FUSED_VZ_SWITCH_BETA_SCALE   (0.45f)
/* 量测来源掩码切换时，速度差分更新缩放系数 */
#define TOF_FUSED_VZ_SWITCH_GAMMA_SCALE  (0.35f)

/* ==================== 融合来源标识宏 ==================== */

/* 融合来源：无有效来源 */
#define TOF_SRC_NONE                     (0U)
/* 融合来源：仅通道1 */
#define TOF_SRC_CH1                      (1U)
/* 融合来源：仅通道2 */
#define TOF_SRC_CH2                      (2U)
/* 融合来源：仅通道3 */
#define TOF_SRC_CH3                      (3U)
/* 融合来源：仅通道4 */
#define TOF_SRC_CH4                      (4U)
/* 融合来源：多通道均值融合 */
#define TOF_SRC_MULTI                    (5U)

/* ==================== 数据结构 ==================== */

/*
 * 单通道运行状态缓存。
 */
typedef struct
{
    uint8  has_last_mm;        /* 是否有历史有效值，1=有，0=无 */
    uint8  fresh_now;          /* 当前数据是否新鲜 */
    float  last_mm;            /* 上一帧有效的中心高度估计，单位：mm */
    uint16 invalid_streak;     /* 连续无效帧计数 */
    uint16 same_value_streak;  /* 连续相同值帧计数（冻结检测用） */
    uint8  spike_count;        /* 连续突变帧计数（电缆干扰抑制用） */
} TOFChannelState_t;

/*
 * 每帧预计算的姿态修正参数（四通道共用，每帧调用2次sinf+2次cosf）。
 */
typedef struct
{
    float cos_term;  /* cos(pitch_comp) * cos(roll_comp)，已钳位至[TOF_COS_TERM_MIN, 1.0] */
    float sin_pitch; /* sin(pitch_comp)，pitch_comp = pitch - pitch_bias（度转弧度后） */
    float sin_roll;  /* sin(roll_comp)，roll_comp = roll - roll_bias（度转弧度后） */
    uint8 valid;     /* 1=倾角在允许范围内；0=倾角过大，全部通道数据不可信 */
} TOFAttitudeTerms_t;

typedef struct
{
    float height_mm;
    float slope_x;
    float slope_y;
    float rms_mm;
    float max_abs_residual_mm;
    uint8 count;
    uint8 worst_ch;
    uint8 solved;
} TOFShadowPlaneFit_t;

typedef struct
{
    uint8 seeded;
    float height_mm;
    float vz_mmps;
    float p00;
    float p01;
    float p10;
    float p11;
} TOFShadowState_t;

typedef struct
{
    uint8 valid;
    uint8 mask;
    uint8 count;
    float measure_mm;
    float spread_mm;
    float pred_err_mm;
    float measure_var_mm2;
} TOFFusedCandidate_t;

typedef struct
{
    uint8 seeded;
    float height_mm;
    float vz_mmps;
    float p00;
    float p01;
    float p10;
    float p11;
} TOFFusedState_t;

/* ==================== 机臂几何参数（Mark5 Pro，Quad-X布局） ==================== */
/*
 * 各TOF在机体坐标系中的位置（x=前方向正，y=右方向正），单位：mm。
 * 索引0=TOF1/M1右后，1=TOF2/M2右前，2=TOF3/M3左后，3=TOF4/M4左前。
 *
 * 中心高度推算公式（推导自几何约束）：
 *   h_center_i = slant_i * cos(pitch_comp) * cos(roll_comp)
 *               - x_arm_i * sin(pitch_comp)
 *               + y_arm_i * sin(roll_comp)
 * 物理意义：
 *   第一项：将TOF斜距投影到垂直方向（飞机倾斜时TOF不再竖直向下）
 *   第二项：pitch倾斜时，前/后机臂距地面高度不同（cos修正后再加位置修正）
 *   第三项：roll倾斜时，左/右机臂距地面高度不同
 * 当pitch_comp≈0且roll_comp≈0（校准位置）时，后两项趋近于0，公式退化为 h = slant。
 */
static const int16 s_tof_arm_x_mm[VL53L1X_CHANNEL_COUNT] = {
    -(int16)MOTOR_ARM_PITCH_MM,   /* TOF1：M1右后，位于后方 */
    +(int16)MOTOR_ARM_PITCH_MM,   /* TOF2：M2右前，位于前方 */
    -(int16)MOTOR_ARM_PITCH_MM,   /* TOF3：M3左后，位于后方 */
    +(int16)MOTOR_ARM_PITCH_MM    /* TOF4：M4左前，位于前方 */
};
static const int16 s_tof_arm_y_mm[VL53L1X_CHANNEL_COUNT] = {
    +(int16)MOTOR_ARM_ROLL_MM,    /* TOF1：M1右后，位于右侧 */
    +(int16)MOTOR_ARM_ROLL_MM,    /* TOF2：M2右前，位于右侧 */
    -(int16)MOTOR_ARM_ROLL_MM,    /* TOF3：M3左后，位于左侧 */
    -(int16)MOTOR_ARM_ROLL_MM     /* TOF4：M4左前，位于左侧 */
};

/* ==================== 全局变量定义 ==================== */

/* 融合后TOF高度，单位：mm，无效时为VL53L1X_VALID_RANGE_MAX */
float g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
/* 各通道中心高度估计（几何修正+零偏后），无效时为VL53L1X_INVALID_DISTANCE_MM */
float g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
float g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
float g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
float g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
/* 融合高度有效标志，1=有效，0=无效 */
uint8 g_tof_fused_valid = 0U;
/* 各通道原始有效标志（通过范围与状态码检查），1=有效，0=无效 */
uint8 g_tof1_valid = 0U;
uint8 g_tof2_valid = 0U;
uint8 g_tof3_valid = 0U;
uint8 g_tof4_valid = 0U;
/* 本帧融合是否使用了通道2/3（向后兼容），1=使用，0=未使用 */
uint8 g_tof2_used_in_fusion = 0U;
uint8 g_tof3_used_in_fusion = 0U;
/* 当前融合来源，取值见TOF_SRC_*宏 */
uint8 g_tof_fused_source = TOF_SRC_NONE;
/* 校准通过标志：1=各通道偏差在允许范围内，0=校准失败 */
uint8 g_tof_calibration_ok = 0U;
float g_tof_fused_vz_mps = 0.0f;
float g_tof_measure_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
uint8 g_tof_measure_class = TOF_MEASURE_CLASS_NONE;
uint8 g_tof_measure_mask = 0U;
float g_tof_shadow_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
float g_tof_shadow_vz_mps = 0.0f;
uint8 g_tof_shadow_valid = 0U;

/* ==================== 静态变量 ==================== */

/* TOF模块初始化完成标志，1=已初始化，0=未初始化 */
static uint8 s_tof_inited = 0U;
/* 各通道零偏补偿，单位：mm；校准后对齐到公共中心 */
static int16 s_tof_offset_mm[VL53L1X_CHANNEL_COUNT] = {0};
/* 各通道运行状态缓存 */
static TOFChannelState_t s_tof_state[VL53L1X_CHANNEL_COUNT] = {{0}};
/* 静态姿态偏差：陀螺仪静置时的系统性pitch/roll偏置，校准时采样记录 */
static float s_attitude_pitch_bias_deg = 0.0f;
static float s_attitude_roll_bias_deg  = 0.0f;
/* 融合失效保持计数（最多保持TOF_FUSED_HOLD_FRAMES帧后输出无效） */
static TOFFusedState_t s_tof_fused_state = {0};
static uint8 s_tof_fused_miss_count = 0U;
static float s_tof_fused_meas_hist_mm[TOF_FUSED_VZ_HIST_LEN] = {0};
static uint8 s_tof_fused_meas_hist_count = 0U;
static uint8 s_tof_fused_last_measure_mask = 0U;
static uint8 s_tof_fused_pending_mask = 0U;
static uint8 s_tof_fused_pending_count = 0U;
static uint8 s_fused_invalid_hold_count = 0U;
static Kalman1D_t s_tof_fused_kf = {0};
static int16 s_tof_shadow_offset_mm[VL53L1X_CHANNEL_COUNT] = {0};
static TOFShadowState_t s_tof_shadow_state = {0};
static float s_tof_shadow_slope_x = 0.0f;
static float s_tof_shadow_slope_y = 0.0f;
static uint8 s_tof_shadow_slope_valid = 0U;
static uint8 s_tof_shadow_miss_count = 0U;
/* 步进限幅内部参考值（浮点，与滤波输出解耦，防止滤波滞后影响步进判断） */
/* 最终输出IIR滤波状态（浮点，避免整数累积误差） */

/* ==================== 内部工具函数 ==================== */

/*
 * 函数功能：计算两个无符号16位数的绝对差。
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
 * 函数功能：对整数高度值应用零偏并钳位到[0, 65535]。
 * 输入参数：
 *   raw_mm    - 原始高度值，单位：mm。
 *   offset_mm - 通道零偏（可为负），单位：mm。
 * 返回值：
 *   补偿并钳位后的高度值，单位：mm。
 */
static float TOF_ApplyOffsetClamp(float raw_mm, int16 offset_mm)
{
    float corrected = raw_mm + (float)offset_mm;

    if (corrected < 0.0f)
    {
        corrected = 0.0f;
    }
    else if (corrected > 65535.0f)
    {
        corrected = 65535.0f;
    }

    return corrected;
}

/*
 * 函数功能：判定单次TOF原始测量是否有效。
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
 */
static void TOF_ResetChannelState(TOFChannelState_t *state)
{
    if (0 == state)
    {
        return;
    }

    state->has_last_mm = 0U;
    state->fresh_now = 0U;
    state->last_mm = 0.0f;
    state->invalid_streak = 0U;
    state->same_value_streak = 0U;
    state->spike_count = 0U;
}

static void TOF_ShadowResetState(void);

/*
 * 函数功能：重置TOF融合输出与全局状态。
 */
static void TOF_ResetFusionState(void)
{
    g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_tof1_valid = 0U;
    g_tof2_valid = 0U;
    g_tof3_valid = 0U;
    g_tof4_valid = 0U;
    g_tof_fused_valid = 0U;
    g_tof2_used_in_fusion = 0U;
    g_tof3_used_in_fusion = 0U;
    g_tof_fused_source = TOF_SRC_NONE;
    g_tof_fused_vz_mps = 0.0f;
    g_tof_measure_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_tof_measure_class = TOF_MEASURE_CLASS_NONE;
    g_tof_measure_mask = 0U;

    s_tof_fused_state.seeded = 0U;
    s_tof_fused_state.height_mm = 0.0f;
    s_tof_fused_state.vz_mmps = 0.0f;
    s_tof_fused_state.p00 = TOF_FUSED_P0_H_MM2;
    s_tof_fused_state.p01 = 0.0f;
    s_tof_fused_state.p10 = 0.0f;
    s_tof_fused_state.p11 = TOF_FUSED_P0_V_MM2PS2;
    s_tof_fused_miss_count = 0U;
    s_tof_fused_meas_hist_count = 0U;
    s_tof_fused_last_measure_mask = 0U;
    s_tof_fused_pending_mask = 0U;
    s_tof_fused_pending_count = 0U;
    s_fused_invalid_hold_count = 0U;
    Kalman1D_Init(&s_tof_fused_kf,
                  TOF_FUSED_KF_Q_MM2,
                  TOF_FUSED_KF_R_MM2,
                  TOF_FUSED_KF_P0_MM2,
                  0.0f);
    TOF_ShadowResetState();
}

/*
 * 函数功能：计算当前帧的姿态修正参数（使用bias修正后的角度，每帧计算一次）。
 * 返回值：
 *   TOFAttitudeTerms_t，包含 cos_term、sin_pitch、sin_roll 和有效标志。
 */
static void TOF_Sort4(uint16 *vals, uint8 n);
static float TOF_ClampF(float value, float min_value, float max_value);

static uint8 TOF_IsDiagonalMask(uint8 mask)
{
    if ((0x09U == mask) || (0x06U == mask))
    {
        return 1U;
    }
    return 0U;
}

static float TOF_FusedBaseVariance(uint8 count)
{
    if (TOF_MEASURE_CLASS_4CH == count)
    {
        return TOF_FUSED_R_4CH_MM2;
    }
    if (TOF_MEASURE_CLASS_3CH == count)
    {
        return TOF_FUSED_R_3CH_MM2;
    }
    if (TOF_MEASURE_CLASS_1CH == count)
    {
        return TOF_FUSED_R_1CH_MM2;
    }
    return TOF_FUSED_R_2CH_MM2;
}

static void TOF_FusedSeed(float height_mm)
{
    s_tof_fused_state.seeded = 1U;
    s_tof_fused_state.height_mm = height_mm;
    s_tof_fused_state.vz_mmps = 0.0f;
    s_tof_fused_state.p00 = TOF_FUSED_P0_H_MM2;
    s_tof_fused_state.p01 = 0.0f;
    s_tof_fused_state.p10 = 0.0f;
    s_tof_fused_state.p11 = TOF_FUSED_P0_V_MM2PS2;
    s_tof_fused_miss_count = 0U;
    s_tof_fused_meas_hist_mm[0] = height_mm;
    s_tof_fused_meas_hist_count = 1U;
}

static void TOF_FusedPredict(float dt_s)
{
    float dt2;
    float dt3;
    float dt4;
    float p00;
    float p01;
    float p10;
    float p11;

    if (0U == s_tof_fused_state.seeded)
    {
        return;
    }

    dt_s = TOF_ClampF(dt_s, 0.001f, 0.05f);
    dt2 = dt_s * dt_s;
    dt3 = dt2 * dt_s;
    dt4 = dt2 * dt2;

    s_tof_fused_state.height_mm += s_tof_fused_state.vz_mmps * dt_s;

    p00 = s_tof_fused_state.p00;
    p01 = s_tof_fused_state.p01;
    p10 = s_tof_fused_state.p10;
    p11 = s_tof_fused_state.p11;

    s_tof_fused_state.p00 = p00 + dt_s * (p10 + p01) + dt2 * p11 + 0.25f * dt4 * TOF_FUSED_Q_ACC_MM2PS4;
    s_tof_fused_state.p01 = p01 + dt_s * p11 + 0.5f * dt3 * TOF_FUSED_Q_ACC_MM2PS4;
    s_tof_fused_state.p10 = p10 + dt_s * p11 + 0.5f * dt3 * TOF_FUSED_Q_ACC_MM2PS4;
    s_tof_fused_state.p11 = p11 + dt2 * TOF_FUSED_Q_ACC_MM2PS4;
}

static void TOF_FusedCorrect(float measure_mm, float measure_var_mm2)
{
    float dt_s;
    float alpha;
    float beta;
    float gamma;
    float innov;
    float vz_meas_mmps;
    float adapt_scale;
    float vz_delta_mmps;
    uint8 i;
    uint8 hist_span;
    uint8 mask_changed;

    if (0U == s_tof_fused_state.seeded)
    {
        return;
    }

    measure_var_mm2 = TOF_ClampF(measure_var_mm2, 1.0f, 40000.0f);
    dt_s = 0.01f;
    alpha = 0.75f;
    beta = 0.08f;
    gamma = 0.18f;

    if (TOF_MEASURE_CLASS_4CH == g_tof_measure_class)
    {
        alpha = 0.90f;
        beta = 0.14f;
        gamma = 0.22f;
    }
    else if (TOF_MEASURE_CLASS_3CH == g_tof_measure_class)
    {
        alpha = 0.84f;
        beta = 0.09f;
        gamma = 0.16f;
    }
    else if (TOF_MEASURE_CLASS_2CH == g_tof_measure_class)
    {
        alpha = 0.72f;
        beta = 0.06f;
        gamma = 0.10f;
    }
    else if (TOF_MEASURE_CLASS_1CH == g_tof_measure_class)
    {
        alpha = 0.55f;
        beta = 0.03f;
        gamma = 0.06f;
    }

    if (measure_var_mm2 > 6000.0f)
    {
        alpha *= 0.85f;
        beta *= 0.80f;
        gamma *= 0.80f;
    }
    else if (measure_var_mm2 < 400.0f)
    {
        alpha = TOF_ClampF(alpha * 1.05f, 0.0f, 0.95f);
        beta = TOF_ClampF(beta * 1.10f, 0.0f, 0.30f);
        gamma = TOF_ClampF(gamma * 1.05f, 0.0f, 0.70f);
    }

    mask_changed = ((0U != s_tof_fused_last_measure_mask) &&
                    (g_tof_measure_mask != s_tof_fused_last_measure_mask)) ? 1U : 0U;
    if (0U != mask_changed)
    {
        s_tof_fused_meas_hist_count = 0U;
        beta *= TOF_FUSED_VZ_SWITCH_BETA_SCALE;
        gamma *= TOF_FUSED_VZ_SWITCH_GAMMA_SCALE;
    }

    if (s_tof_fused_state.height_mm > TOF_VZ_ADAPT_H_LOW_MM)
    {
        adapt_scale = 1.0f - (1.0f - TOF_VZ_ADAPT_GAIN_MIN)
                    * (s_tof_fused_state.height_mm - TOF_VZ_ADAPT_H_LOW_MM)
                    / (TOF_VZ_ADAPT_H_HIGH_MM - TOF_VZ_ADAPT_H_LOW_MM);
        if (adapt_scale < TOF_VZ_ADAPT_GAIN_MIN)
        {
            adapt_scale = TOF_VZ_ADAPT_GAIN_MIN;
        }
        beta *= adapt_scale;
        gamma *= adapt_scale;
    }

    innov = measure_mm - s_tof_fused_state.height_mm;
    s_tof_fused_state.height_mm += alpha * innov;
    s_tof_fused_state.vz_mmps += (beta * innov) / dt_s;

    if (0U != s_tof_fused_miss_count)
    {
        s_tof_fused_meas_hist_count = 0U;
    }

    if (s_tof_fused_meas_hist_count < TOF_FUSED_VZ_HIST_LEN)
    {
        s_tof_fused_meas_hist_mm[s_tof_fused_meas_hist_count] = measure_mm;
        s_tof_fused_meas_hist_count++;
    }
    else
    {
        for (i = 1U; i < TOF_FUSED_VZ_HIST_LEN; i++)
        {
            s_tof_fused_meas_hist_mm[i - 1U] = s_tof_fused_meas_hist_mm[i];
        }
        s_tof_fused_meas_hist_mm[TOF_FUSED_VZ_HIST_LEN - 1U] = measure_mm;
    }

    if (s_tof_fused_meas_hist_count >= 2U)
    {
        hist_span = (uint8)(s_tof_fused_meas_hist_count - 1U);
        vz_meas_mmps = (s_tof_fused_meas_hist_mm[s_tof_fused_meas_hist_count - 1U]
                      - s_tof_fused_meas_hist_mm[0]) / ((float)hist_span * dt_s);
        vz_meas_mmps = TOF_ClampF(vz_meas_mmps, -3000.0f, 3000.0f);
        vz_delta_mmps = TOF_ClampF(vz_meas_mmps - s_tof_fused_state.vz_mmps,
                                   -TOF_FUSED_VZ_DV_LIMIT_MMPS,
                                   TOF_FUSED_VZ_DV_LIMIT_MMPS);
        vz_meas_mmps = s_tof_fused_state.vz_mmps + vz_delta_mmps;

        s_tof_fused_state.vz_mmps =
            ((1.0f - gamma) * s_tof_fused_state.vz_mmps) + (gamma * vz_meas_mmps);
    }
}

static uint8 TOF_FusedBuildCandidate(const uint8 *ch_valid,
                                     const float *ch_center_mm,
                                     uint8 mask,
                                     TOFFusedCandidate_t *candidate)
{
    uint8 ch;
    uint8 count;
    uint8 i;
    uint8 j;
    uint8 key_mask;
    uint8 order_mask[VL53L1X_CHANNEL_COUNT];
    float key;
    float sorted[VL53L1X_CHANNEL_COUNT];
    float diff_lo;
    float diff_hi;
    float spread_mm;
    float measure_var_mm2;
    float err0_mm;
    float err1_mm;
    uint8 use_idx;

    if ((0 == ch_valid) || (0 == ch_center_mm) || (0 == candidate))
    {
        return 0U;
    }

    candidate->valid = 0U;
    candidate->mask = mask;
    candidate->count = 0U;
    candidate->measure_mm = 0.0f;
    candidate->spread_mm = 0.0f;
    candidate->pred_err_mm = 0.0f;
    candidate->measure_var_mm2 = 0.0f;

    count = 0U;
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        if (0U == (mask & (uint8)(1U << ch)))
        {
            continue;
        }

        if (0U == ch_valid[ch])
        {
            return 0U;
        }

        sorted[count] = ch_center_mm[ch];
        order_mask[count] = (uint8)(1U << ch);
        count++;
    }

    if (0U == count)
    {
        return 0U;
    }

    for (i = 1U; i < count; i++)
    {
        key = sorted[i];
        key_mask = order_mask[i];
        j = i;
        while ((j > 0U) && (sorted[j - 1U] > key))
        {
            sorted[j] = sorted[j - 1U];
            order_mask[j] = order_mask[j - 1U];
            j--;
        }
        sorted[j] = key;
        order_mask[j] = key_mask;
    }

    spread_mm = 0.0f;
    measure_var_mm2 = 0.0f;

    if (TOF_MEASURE_CLASS_4CH == count)
    {
        candidate->measure_mm = 0.5f * (sorted[1] + sorted[2]);
        spread_mm = sorted[2] - sorted[1];
        measure_var_mm2 = TOF_FusedBaseVariance(count) + 0.25f * spread_mm * spread_mm;
    }
    else if (TOF_MEASURE_CLASS_3CH == count)
    {
        candidate->measure_mm = sorted[1];
        diff_lo = sorted[1] - sorted[0];
        diff_hi = sorted[2] - sorted[1];
        spread_mm = (diff_lo < diff_hi) ? diff_lo : diff_hi;
        measure_var_mm2 = TOF_FusedBaseVariance(count) + 0.25f * spread_mm * spread_mm;
    }
    else if (TOF_MEASURE_CLASS_2CH == count)
    {
        spread_mm = sorted[1] - sorted[0];

        if ((spread_mm > (float)TOF_FUSED_2CH_SPLIT_MM) && (0U != s_tof_fused_state.seeded))
        {
            err0_mm = fabsf(sorted[0] - s_tof_fused_state.height_mm);
            err1_mm = fabsf(sorted[1] - s_tof_fused_state.height_mm);
            use_idx = (err0_mm <= err1_mm) ? 0U : 1U;
            candidate->measure_mm = sorted[use_idx];
            candidate->mask = order_mask[use_idx];
            candidate->count = TOF_MEASURE_CLASS_1CH;
            candidate->spread_mm = 0.0f;
            candidate->pred_err_mm = (use_idx == 0U) ? err0_mm : err1_mm;
            candidate->measure_var_mm2 = TOF_FUSED_R_1CH_MM2 + 0.0625f * spread_mm * spread_mm;
            candidate->valid = 1U;
            return 1U;
        }

        candidate->measure_mm = 0.5f * (sorted[0] + sorted[1]);
        if (0U != TOF_IsDiagonalMask(mask))
        {
            measure_var_mm2 = TOF_FUSED_R_2CH_MM2 + 0.25f * spread_mm * spread_mm;
        }
        else
        {
            measure_var_mm2 = TOF_FUSED_R_2CH_EDGE_MM2 + 0.25f * spread_mm * spread_mm;
        }
    }
    else
    {
        candidate->measure_mm = sorted[0];
        measure_var_mm2 = TOF_FusedBaseVariance(TOF_MEASURE_CLASS_1CH);
    }

    candidate->valid = 1U;
    candidate->count = count;
    candidate->spread_mm = spread_mm;
    if (0U != s_tof_fused_state.seeded)
    {
        candidate->pred_err_mm = fabsf(candidate->measure_mm - s_tof_fused_state.height_mm);
    }
    candidate->measure_var_mm2 = measure_var_mm2;
    return 1U;
}

static uint8 TOF_FusedSelectCandidate(const uint8 *ch_valid,
                                      const float *ch_center_mm,
                                      TOFFusedCandidate_t *selected)
{
    uint8 ch;
    uint8 valid_mask;

    if ((0 == ch_valid) || (0 == ch_center_mm) || (0 == selected))
    {
        return 0U;
    }

    valid_mask = 0U;
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        if (0U != ch_valid[ch])
        {
            valid_mask |= (uint8)(1U << ch);
        }
    }

    if (0U == valid_mask)
    {
        s_tof_fused_last_measure_mask = 0U;
        s_tof_fused_pending_mask = 0U;
        s_tof_fused_pending_count = 0U;
        return 0U;
    }

    if (0U == TOF_FusedBuildCandidate(ch_valid, ch_center_mm, valid_mask, selected))
    {
        s_tof_fused_last_measure_mask = 0U;
        s_tof_fused_pending_mask = 0U;
        s_tof_fused_pending_count = 0U;
        return 0U;
    }

    s_tof_fused_pending_mask = 0U;
    s_tof_fused_pending_count = 0U;
    return 1U;
}

static void TOF_SeedFusionKalman(uint16 seed_mm)
{
    Kalman1D_Init(&s_tof_fused_kf,
                  TOF_FUSED_KF_Q_MM2,
                  TOF_FUSED_KF_R_MM2,
                  TOF_FUSED_KF_P0_MM2,
                  (float)seed_mm);
}

static uint8 TOF_BuildFusionMeasurement(const uint8 *ch_fusion_valid,
                                        const uint16 *ch_center_mm,
                                        uint16 *measure_mm,
                                        uint8 *source)
{
    (void)ch_fusion_valid;
    (void)ch_center_mm;
    if (0 != measure_mm)
    {
        *measure_mm = 0U;
    }
    if (0 != source)
    {
        *source = TOF_SRC_NONE;
    }
    return 0U;
}

static TOFAttitudeTerms_t TOF_ComputeAttitudeTerms(void)
{
    TOFAttitudeTerms_t att;
    float pitch_comp_deg;
    float roll_comp_deg;
    float pitch_comp_rad;
    float roll_comp_rad;

    att.cos_term  = 1.0f;
    att.sin_pitch = 0.0f;
    att.sin_roll  = 0.0f;
    att.valid     = 1U;

    if (0U == IMU_Is_Ready())
    {
        return att;
    }

    pitch_comp_deg = g_euler.pitch - s_attitude_pitch_bias_deg;
    roll_comp_deg  = g_euler.roll  - s_attitude_roll_bias_deg;

    /* 大角度保护：倾角超过限值时TOF可能测到侧面障碍物，全部标记不可信 */
    if ((pitch_comp_deg >  TOF_MAX_TILT_DEG) || (pitch_comp_deg < -TOF_MAX_TILT_DEG) ||
        (roll_comp_deg  >  TOF_MAX_TILT_DEG) || (roll_comp_deg  < -TOF_MAX_TILT_DEG))
    {
        att.valid = 0U;
    }

    pitch_comp_rad = pitch_comp_deg * TOF_DEG_TO_RAD;
    roll_comp_rad  = roll_comp_deg  * TOF_DEG_TO_RAD;

    att.sin_pitch = sinf(pitch_comp_rad);
    att.sin_roll  = sinf(roll_comp_rad);

    att.cos_term = cosf(pitch_comp_rad) * cosf(roll_comp_rad);
    if (att.cos_term < TOF_COS_TERM_MIN)
    {
        att.cos_term = TOF_COS_TERM_MIN;
    }
    else if (att.cos_term > 1.0f)
    {
        att.cos_term = 1.0f;
    }

    return att;
}

/*
 * 函数功能：将TOF斜距结合姿态和机臂位置转换为飞机中心高度估计。
 * 公式：h_center = slant * cos_term - x_arm * sin_pitch + y_arm * sin_roll
 * 输入参数：
 *   slant_mm - TOF斜距测量值，单位：mm。
 *   att      - 本帧预计算的姿态参数指针（常量）。
 *   ch       - 通道索引（0~3），用于查找对应机臂位置常量。
 * 返回值：
 *   飞机中心高度估计，单位：mm，已钳位到[0, 65535]。
 */
static float TOF_EstimateCenterHeight(uint16 slant_mm, const TOFAttitudeTerms_t *att, uint8 ch)
{
    float h;

    h = (float)slant_mm * att->cos_term
        - (float)s_tof_arm_x_mm[ch] * att->sin_pitch
        + (float)s_tof_arm_y_mm[ch] * att->sin_roll;

    if (h < 0.0f)
    {
        h = 0.0f;
    }
    else if (h > 65535.0f)
    {
        h = 65535.0f;
    }

    return h;
}

/*
 * 函数功能：对最多4个uint16值进行升序排序（插入排序，最多5次比较）。
 * 输入参数：
 *   vals - 待排序数组指针（原地修改）。
 *   n    - 元素个数（0~4）。
 */
static void TOF_Sort4(uint16 *vals, uint8 n)
{
    uint8 i;
    uint8 j;
    uint16 key;

    for (i = 1U; i < n; i++)
    {
        key = vals[i];
        j = i;
        while ((j > 0U) && (vals[j - 1U] > key))
        {
            vals[j] = vals[j - 1U];
            j--;
        }
        vals[j] = key;
    }
}

/* ==================== 模块对外接口实现 ==================== */

/*
 * 函数功能：初始化四路TOF模块并执行零偏标定。
 */
static float TOF_ClampF(float value, float min_value, float max_value)
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

static void TOF_ShadowResetState(void)
{
    s_tof_shadow_state.seeded = 0U;
    s_tof_shadow_state.height_mm = 0.0f;
    s_tof_shadow_state.vz_mmps = 0.0f;
    s_tof_shadow_state.p00 = TOF_SHADOW_P0_H_MM2;
    s_tof_shadow_state.p01 = 0.0f;
    s_tof_shadow_state.p10 = 0.0f;
    s_tof_shadow_state.p11 = TOF_SHADOW_P0_V_MM2PS2;
    s_tof_shadow_slope_x = 0.0f;
    s_tof_shadow_slope_y = 0.0f;
    s_tof_shadow_slope_valid = 0U;
    s_tof_shadow_miss_count = 0U;
    g_tof_shadow_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_tof_shadow_vz_mps = 0.0f;
    g_tof_shadow_valid = 0U;
}

static void TOF_ShadowSeed(float height_mm)
{
    s_tof_shadow_state.seeded = 1U;
    s_tof_shadow_state.height_mm = height_mm;
    s_tof_shadow_state.vz_mmps = 0.0f;
    s_tof_shadow_state.p00 = TOF_SHADOW_P0_H_MM2;
    s_tof_shadow_state.p01 = 0.0f;
    s_tof_shadow_state.p10 = 0.0f;
    s_tof_shadow_state.p11 = TOF_SHADOW_P0_V_MM2PS2;
    s_tof_shadow_miss_count = 0U;
}

static void TOF_ShadowPredict(float dt_s)
{
    float dt2;
    float dt3;
    float dt4;
    float p00;
    float p01;
    float p10;
    float p11;

    if (0U == s_tof_shadow_state.seeded)
    {
        return;
    }

    dt_s = TOF_ClampF(dt_s, 0.001f, 0.05f);
    dt2 = dt_s * dt_s;
    dt3 = dt2 * dt_s;
    dt4 = dt2 * dt2;

    s_tof_shadow_state.height_mm += s_tof_shadow_state.vz_mmps * dt_s;

    p00 = s_tof_shadow_state.p00;
    p01 = s_tof_shadow_state.p01;
    p10 = s_tof_shadow_state.p10;
    p11 = s_tof_shadow_state.p11;

    s_tof_shadow_state.p00 = p00 + dt_s * (p10 + p01) + dt2 * p11 + 0.25f * dt4 * TOF_SHADOW_Q_ACC_MM2PS4;
    s_tof_shadow_state.p01 = p01 + dt_s * p11 + 0.5f * dt3 * TOF_SHADOW_Q_ACC_MM2PS4;
    s_tof_shadow_state.p10 = p10 + dt_s * p11 + 0.5f * dt3 * TOF_SHADOW_Q_ACC_MM2PS4;
    s_tof_shadow_state.p11 = p11 + dt2 * TOF_SHADOW_Q_ACC_MM2PS4;
}

static void TOF_ShadowCorrect(float measure_mm, float measure_var_mm2)
{
    float innov;
    float s;
    float k0;
    float k1;
    float p00;
    float p01;
    float p10;
    float p11;

    if (0U == s_tof_shadow_state.seeded)
    {
        return;
    }

    measure_var_mm2 = TOF_ClampF(measure_var_mm2, 1.0f, 40000.0f);

    p00 = s_tof_shadow_state.p00;
    p01 = s_tof_shadow_state.p01;
    p10 = s_tof_shadow_state.p10;
    p11 = s_tof_shadow_state.p11;

    innov = measure_mm - s_tof_shadow_state.height_mm;
    s = p00 + measure_var_mm2;
    if (s < 1.0f)
    {
        s = 1.0f;
    }

    k0 = p00 / s;
    k1 = p10 / s;

    s_tof_shadow_state.height_mm += k0 * innov;
    s_tof_shadow_state.vz_mmps += k1 * innov;
    s_tof_shadow_state.p00 = (1.0f - k0) * p00;
    s_tof_shadow_state.p01 = (1.0f - k0) * p01;
    s_tof_shadow_state.p10 = p10 - k1 * p00;
    s_tof_shadow_state.p11 = p11 - k1 * p01;
}

static float TOF_ShadowProjectDistanceMm(uint16 distance_mm, const TOFAttitudeTerms_t *att, uint8 ch)
{
    float projected_mm;

    projected_mm = ((float)distance_mm * att->cos_term) + (float)s_tof_shadow_offset_mm[ch];
    return TOF_ClampF(projected_mm, 0.0f, 65535.0f);
}

static uint8 TOF_ShadowIsDiagonalPair(uint8 ch_a, uint8 ch_b)
{
    if (((0U == ch_a) && (3U == ch_b)) ||
        ((3U == ch_a) && (0U == ch_b)) ||
        ((1U == ch_a) && (2U == ch_b)) ||
        ((2U == ch_a) && (1U == ch_b)))
    {
        return 1U;
    }
    return 0U;
}

static uint8 TOF_ShadowSolvePlane(const uint8 *ch_use,
                                  const float *meas_mm,
                                  TOFShadowPlaneFit_t *fit)
{
    uint8 ch;
    uint8 count;
    float a00;
    float a01;
    float a02;
    float a11;
    float a12;
    float a22;
    float b0;
    float b1;
    float b2;
    float det;
    float inv00;
    float inv01;
    float inv02;
    float inv10;
    float inv11;
    float inv12;
    float inv20;
    float inv21;
    float inv22;
    float pred;
    float residual;
    float residual_sq_sum;
    float max_abs_residual;
    uint8 worst_ch;

    if ((0 == ch_use) || (0 == meas_mm) || (0 == fit))
    {
        return 0U;
    }

    fit->height_mm = 0.0f;
    fit->slope_x = 0.0f;
    fit->slope_y = 0.0f;
    fit->rms_mm = 0.0f;
    fit->max_abs_residual_mm = 0.0f;
    fit->count = 0U;
    fit->worst_ch = 0U;
    fit->solved = 0U;

    count = 0U;
    a00 = 0.0f;
    a01 = 0.0f;
    a02 = 0.0f;
    a11 = 0.0f;
    a12 = 0.0f;
    a22 = 0.0f;
    b0 = 0.0f;
    b1 = 0.0f;
    b2 = 0.0f;

    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        float x_mm;
        float y_mm;
        float z_mm;

        if (0U == ch_use[ch])
        {
            continue;
        }

        x_mm = (float)s_tof_arm_x_mm[ch];
        y_mm = (float)s_tof_arm_y_mm[ch];
        z_mm = meas_mm[ch];

        count++;
        a00 += 1.0f;
        a01 += x_mm;
        a02 += y_mm;
        a11 += x_mm * x_mm;
        a12 += x_mm * y_mm;
        a22 += y_mm * y_mm;
        b0 += z_mm;
        b1 += x_mm * z_mm;
        b2 += y_mm * z_mm;
    }

    if (count < 3U)
    {
        return 0U;
    }

    det = a00 * (a11 * a22 - a12 * a12)
        - a01 * (a01 * a22 - a12 * a02)
        + a02 * (a01 * a12 - a11 * a02);
    if (fabsf(det) < 1.0e-3f)
    {
        return 0U;
    }

    inv00 = (a11 * a22 - a12 * a12) / det;
    inv01 = (a02 * a12 - a01 * a22) / det;
    inv02 = (a01 * a12 - a02 * a11) / det;
    inv10 = (a12 * a02 - a01 * a22) / det;
    inv11 = (a00 * a22 - a02 * a02) / det;
    inv12 = (a01 * a02 - a00 * a12) / det;
    inv20 = (a01 * a12 - a11 * a02) / det;
    inv21 = (a01 * a02 - a00 * a12) / det;
    inv22 = (a00 * a11 - a01 * a01) / det;

    fit->height_mm = inv00 * b0 + inv01 * b1 + inv02 * b2;
    fit->slope_x   = inv10 * b0 + inv11 * b1 + inv12 * b2;
    fit->slope_y   = inv20 * b0 + inv21 * b1 + inv22 * b2;

    residual_sq_sum = 0.0f;
    max_abs_residual = 0.0f;
    worst_ch = 0U;

    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        if (0U == ch_use[ch])
        {
            continue;
        }

        pred = fit->height_mm
             + fit->slope_x * (float)s_tof_arm_x_mm[ch]
             + fit->slope_y * (float)s_tof_arm_y_mm[ch];
        residual = meas_mm[ch] - pred;
        residual_sq_sum += residual * residual;
        if (fabsf(residual) > max_abs_residual)
        {
            max_abs_residual = fabsf(residual);
            worst_ch = ch;
        }
    }

    fit->rms_mm = sqrtf(residual_sq_sum / (float)count);
    fit->max_abs_residual_mm = max_abs_residual;
    fit->count = count;
    fit->worst_ch = worst_ch;
    fit->solved = 1U;
    return 1U;
}

static uint8 TOF_ShadowBuildMeasurement(const uint8 *ch_valid,
                                        const float *meas_mm,
                                        float *measure_mm,
                                        float *measure_var_mm2)
{
    uint8 ch;
    uint8 count;
    uint8 ch_list[VL53L1X_CHANNEL_COUNT];
    uint8 use_all[VL53L1X_CHANNEL_COUNT];
    uint8 use_drop[VL53L1X_CHANNEL_COUNT];
    TOFShadowPlaneFit_t fit;
    TOFShadowPlaneFit_t fit_drop;

    if ((0 == ch_valid) || (0 == meas_mm) || (0 == measure_mm) || (0 == measure_var_mm2))
    {
        return 0U;
    }

    count = 0U;
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        use_all[ch] = ch_valid[ch];
        use_drop[ch] = ch_valid[ch];
        if (0U != ch_valid[ch])
        {
            ch_list[count] = ch;
            count++;
        }
    }

    if (count >= 3U)
    {
        if (0U != TOF_ShadowSolvePlane(use_all, meas_mm, &fit))
        {
            if ((4U == count) && (fit.max_abs_residual_mm > TOF_SHADOW_DROP_RESIDUAL_MM))
            {
                use_drop[fit.worst_ch] = 0U;
                if (0U != TOF_ShadowSolvePlane(use_drop, meas_mm, &fit_drop))
                {
                    fit = fit_drop;
                }
            }

            *measure_mm = fit.height_mm;
            if (fit.count >= 4U)
            {
                *measure_var_mm2 = TOF_SHADOW_R_4CH_MM2 + fit.rms_mm * fit.rms_mm;
            }
            else
            {
                *measure_var_mm2 = TOF_SHADOW_R_3CH_MM2 + fit.rms_mm * fit.rms_mm;
            }

            s_tof_shadow_slope_x = fit.slope_x;
            s_tof_shadow_slope_y = fit.slope_y;
            s_tof_shadow_slope_valid = 1U;
            return 1U;
        }
    }

    if (2U == count)
    {
        if (0U != TOF_ShadowIsDiagonalPair(ch_list[0], ch_list[1]))
        {
            *measure_mm = 0.5f * (meas_mm[ch_list[0]] + meas_mm[ch_list[1]]);
            *measure_var_mm2 = TOF_SHADOW_R_2CH_DIAG_MM2;
            return 1U;
        }

        if (0U != s_tof_shadow_slope_valid)
        {
            *measure_mm = 0.5f * ((meas_mm[ch_list[0]]
                                 - s_tof_shadow_slope_x * (float)s_tof_arm_x_mm[ch_list[0]]
                                 - s_tof_shadow_slope_y * (float)s_tof_arm_y_mm[ch_list[0]])
                                + (meas_mm[ch_list[1]]
                                 - s_tof_shadow_slope_x * (float)s_tof_arm_x_mm[ch_list[1]]
                                 - s_tof_shadow_slope_y * (float)s_tof_arm_y_mm[ch_list[1]]));
            *measure_var_mm2 = TOF_SHADOW_R_2CH_EDGE_MM2;
            return 1U;
        }

        return 0U;
    }

    if ((1U == count) && (0U != s_tof_shadow_slope_valid))
    {
        *measure_mm = meas_mm[ch_list[0]]
                    - s_tof_shadow_slope_x * (float)s_tof_arm_x_mm[ch_list[0]]
                    - s_tof_shadow_slope_y * (float)s_tof_arm_y_mm[ch_list[0]];
        *measure_var_mm2 = TOF_SHADOW_R_1CH_MM2;
        return 1U;
    }

    return 0U;
}

static void TOF_Update_Robust(void)
{
    uint8 ch;
    uint8 raw_valid;
    uint8 ch_raw_valid[VL53L1X_CHANNEL_COUNT];
    float ch_center_mm[VL53L1X_CHANNEL_COUNT];
    uint8 ch_fusion_valid[VL53L1X_CHANNEL_COUNT];
    float display_mm[VL53L1X_CHANNEL_COUNT];
    TOFFusedCandidate_t selected;
    const float dt_s = 0.01f;
    TOFAttitudeTerms_t att;
    VL53L1X_data_struct data;

    if (0U == s_tof_inited)
    {
        g_tof1_valid = 0U;
        g_tof2_valid = 0U;
        g_tof3_valid = 0U;
        g_tof4_valid = 0U;
        g_tof_fused_valid = 0U;
        g_tof_fused_vz_mps = 0.0f;
        g_tof_fused_source = TOF_SRC_NONE;
        g_tof_measure_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
        g_tof_measure_class = TOF_MEASURE_CLASS_NONE;
        g_tof_measure_mask = 0U;
        return;
    }

    att = TOF_ComputeAttitudeTerms();
    (void)VL53L1X_read_data(&data);

    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        raw_valid = TOF_IsRawMeasurementValid(data.distance_mm[ch],
                                              data.range_status[ch],
                                              g_vl53l1x_diag[ch].ready,
                                              g_vl53l1x_diag[ch].is_fresh);
        ch_raw_valid[ch] = raw_valid;
        ch_fusion_valid[ch] = 0U;
        ch_center_mm[ch] = (float)VL53L1X_INVALID_DISTANCE_MM;
        display_mm[ch] = (float)VL53L1X_INVALID_DISTANCE_MM;

        if ((0U != raw_valid) && (0U != att.valid))
        {
            float center_mm;
            float corrected_mm;
            float delta;

            center_mm = TOF_EstimateCenterHeight(data.distance_mm[ch], &att, ch);
            corrected_mm = TOF_ApplyOffsetClamp(center_mm, s_tof_offset_mm[ch]);

            ch_center_mm[ch] = corrected_mm;
            display_mm[ch] = corrected_mm;

            if ((0U != s_tof_state[ch].has_last_mm) &&
                (fabsf(corrected_mm - s_tof_state[ch].last_mm) < 0.5f))
            {
                if (s_tof_state[ch].same_value_streak < 65535U)
                {
                    s_tof_state[ch].same_value_streak++;
                }
            }
            else
            {
                s_tof_state[ch].same_value_streak = 0U;
            }

            if (0U != s_tof_state[ch].has_last_mm)
            {
                delta = fabsf(corrected_mm - s_tof_state[ch].last_mm);
                if (delta > (float)TOF_SPIKE_GATE_MM)
                {
                    s_tof_state[ch].last_mm = corrected_mm;
                    s_tof_state[ch].fresh_now = g_vl53l1x_diag[ch].is_fresh;
                    s_tof_state[ch].invalid_streak = 0U;

                    if (s_tof_state[ch].spike_count < TOF_SPIKE_PERSIST_FRAMES)
                    {
                        s_tof_state[ch].spike_count++;
                        continue;
                    }

                    s_tof_state[ch].spike_count = 0U;
                    s_tof_state[ch].has_last_mm = 1U;
                    ch_fusion_valid[ch] = 1U;
                    continue;
                }

                s_tof_state[ch].spike_count = 0U;
            }
            else
            {
                s_tof_state[ch].spike_count = 0U;
            }

            s_tof_state[ch].last_mm = corrected_mm;
            s_tof_state[ch].has_last_mm = 1U;
            s_tof_state[ch].fresh_now = g_vl53l1x_diag[ch].is_fresh;
            s_tof_state[ch].invalid_streak = 0U;
            ch_fusion_valid[ch] = 1U;
        }
        else
        {
            if (s_tof_state[ch].invalid_streak < 65535U)
            {
                s_tof_state[ch].invalid_streak++;
            }
            s_tof_state[ch].fresh_now = 0U;
            s_tof_state[ch].same_value_streak = 0U;
            s_tof_state[ch].spike_count = 0U;

            if (0U != s_tof_state[ch].has_last_mm)
            {
                display_mm[ch] = s_tof_state[ch].last_mm;
            }
        }
    }

    g_tof1_height_mm = display_mm[0];
    g_tof2_height_mm = display_mm[1];
    g_tof3_height_mm = display_mm[2];
    g_tof4_height_mm = display_mm[3];
    g_tof1_valid = ch_raw_valid[0];
    g_tof2_valid = ch_raw_valid[1];
    g_tof3_valid = ch_raw_valid[2];
    g_tof4_valid = ch_raw_valid[3];

    TOF_FusedPredict(dt_s);

    g_tof_measure_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_tof_measure_class = TOF_MEASURE_CLASS_NONE;
    g_tof_measure_mask = 0U;
    g_tof_fused_source = TOF_SRC_NONE;
    g_tof2_used_in_fusion = 0U;
    g_tof3_used_in_fusion = 0U;

    if ((0U != att.valid) &&
        (0U != TOF_FusedSelectCandidate(ch_fusion_valid, ch_center_mm, &selected)))
    {
        g_tof_measure_height_mm = selected.measure_mm;
        g_tof_measure_class = selected.count;
        g_tof_measure_mask = selected.mask;
        g_tof2_used_in_fusion = (0U != (selected.mask & (1U << 1))) ? 1U : 0U;
        g_tof3_used_in_fusion = (0U != (selected.mask & (1U << 2))) ? 1U : 0U;

        if (0U == s_tof_fused_state.seeded)
        {
            TOF_FusedSeed(selected.measure_mm);
        }
        else
        {
            TOF_FusedCorrect(selected.measure_mm, selected.measure_var_mm2);
        }
        s_tof_fused_last_measure_mask = selected.mask;

        s_tof_fused_miss_count = 0U;
        g_tof_fused_valid = 1U;
        if (selected.count >= TOF_MEASURE_CLASS_2CH)
        {
            g_tof_fused_source = TOF_SRC_MULTI;
        }
        else if (0U != (selected.mask & 0x01U))
        {
            g_tof_fused_source = TOF_SRC_CH1;
        }
        else if (0U != (selected.mask & 0x02U))
        {
            g_tof_fused_source = TOF_SRC_CH2;
        }
        else if (0U != (selected.mask & 0x04U))
        {
            g_tof_fused_source = TOF_SRC_CH3;
        }
        else if (0U != (selected.mask & 0x08U))
        {
            g_tof_fused_source = TOF_SRC_CH4;
        }
    }
    else if (0U != s_tof_fused_state.seeded)
    {
        if (s_tof_fused_miss_count < 255U)
        {
            s_tof_fused_miss_count++;
        }

        s_tof_fused_state.vz_mmps *= TOF_FUSED_MISS_VZ_DAMP;
        if ((s_tof_fused_state.vz_mmps < TOF_FUSED_MISS_VZ_STOP_MMPS) &&
            (s_tof_fused_state.vz_mmps > -TOF_FUSED_MISS_VZ_STOP_MMPS))
        {
            s_tof_fused_state.vz_mmps = 0.0f;
        }

        if (s_tof_fused_miss_count > TOF_FUSED_TIMEOUT_FRAMES)
        {
            s_tof_fused_state.seeded = 0U;
            s_tof_fused_state.vz_mmps = 0.0f;
            s_tof_fused_meas_hist_count = 0U;
            g_tof_fused_valid = 0U;
            g_tof_fused_vz_mps = 0.0f;
            s_tof_fused_last_measure_mask = 0U;
            s_tof_fused_pending_mask = 0U;
            s_tof_fused_pending_count = 0U;
        }
        else
        {
            g_tof_fused_valid = 1U;
        }
    }
    else
    {
        g_tof_fused_valid = 0U;
        g_tof_fused_vz_mps = 0.0f;
        s_tof_fused_meas_hist_count = 0U;
    }

    if (0U != s_tof_fused_state.seeded)
    {
        s_tof_fused_state.height_mm = TOF_ClampF(s_tof_fused_state.height_mm, 0.0f, 65535.0f);
        s_tof_fused_state.vz_mmps = TOF_ClampF(s_tof_fused_state.vz_mmps, -3000.0f, 3000.0f);
        g_tof_fused_height_mm = s_tof_fused_state.height_mm;
        g_tof_fused_vz_mps = 0.001f * s_tof_fused_state.vz_mmps;
    }
}

void TOF_Init(void)
{
    uint8 ch;
    uint8 init_err;

    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        s_tof_offset_mm[ch] = 0;
        s_tof_shadow_offset_mm[ch] = 0;
        TOF_ResetChannelState(&s_tof_state[ch]);
    }
    s_attitude_pitch_bias_deg = 0.0f;
    s_attitude_roll_bias_deg  = 0.0f;
    TOF_ResetFusionState();

    /* 初始化四路TOF硬件；0x0F表示全部失败，此时模块不可运行 */
    init_err = VL53L1X_init_all();
    s_tof_inited = (init_err != 0x0FU) ? 1U : 0U;
    if (0U == s_tof_inited)
    {
        return;
    }

    /* 初始化后执行一次静态标定 */
    TOF_Calibrate();
}

/*
 * 函数功能：执行TOF静态标定（阻塞约1.5秒）。
 *   阶段1（500ms）：采样IMU，记录静态姿态偏差，消除陀螺仪固有倾斜。
 *   阶段2（1000ms）：采样四路TOF，用几何修正公式估算各通道中心高度均值，对齐零偏。
 *   阶段3：检查通道间最大偏差；<=100mm则置g_tof_calibration_ok=1，否则=0。
 */
void TOF_Calibrate(void)
{
    uint32 i;
    uint8  ch;
    float  sum_pitch;
    float  sum_roll;
    uint32 sum_mm[VL53L1X_CHANNEL_COUNT];
    float  sum_shadow_mm[VL53L1X_CHANNEL_COUNT];
    uint32 cnt[VL53L1X_CHANNEL_COUNT];
    int32  mean_mm[VL53L1X_CHANNEL_COUNT];
    int32  mean_shadow_mm[VL53L1X_CHANNEL_COUNT];
    int32  valid_ch_count;
    int32  common_center;
    int32  common_shadow;
    int32  max_mean;
    int32  min_mean;
    uint8  raw_valid;
    uint16 center_mm;
    TOFAttitudeTerms_t att;
    VL53L1X_data_struct data;

    if (0U == s_tof_inited)
    {
        return;
    }

    /* 重置所有偏差与状态 */
    s_attitude_pitch_bias_deg = 0.0f;
    s_attitude_roll_bias_deg  = 0.0f;
    sum_pitch = 0.0f;
    sum_roll  = 0.0f;
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        s_tof_offset_mm[ch] = 0;
        s_tof_shadow_offset_mm[ch] = 0;
        TOF_ResetChannelState(&s_tof_state[ch]);
        sum_mm[ch]   = 0U;
        sum_shadow_mm[ch] = 0.0f;
        cnt[ch]      = 0U;
        mean_mm[ch]  = -1;
        mean_shadow_mm[ch] = -1;
    }
    TOF_ResetFusionState();

    /* ---- 阶段1：IMU静态偏差采样（500ms） ---- */
    for (i = 0U; i < TOF_ATTITUDE_CALIB_SAMPLES; i++)
    {
        if (0U != IMU_Is_Ready())
        {
            sum_pitch += g_euler.pitch;
            sum_roll  += g_euler.roll;
        }
        system_delay_ms(TOF_CALIBRATION_DT_MS);
    }
    s_attitude_pitch_bias_deg = sum_pitch / (float)TOF_ATTITUDE_CALIB_SAMPLES;
    s_attitude_roll_bias_deg  = sum_roll  / (float)TOF_ATTITUDE_CALIB_SAMPLES;

    /* ---- 阶段2：四路TOF采样（1000ms），计算各通道中心高度均值 ---- */
    for (i = 0U; i < TOF_CALIBRATION_SAMPLES; i++)
    {
        /* 使用已更新的姿态偏差计算本次几何修正参数 */
        att = TOF_ComputeAttitudeTerms();
        (void)VL53L1X_read_data(&data);

        for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
        {
            raw_valid = TOF_IsRawMeasurementValid(data.distance_mm[ch],
                                                  data.range_status[ch],
                                                  g_vl53l1x_diag[ch].ready,
                                                  g_vl53l1x_diag[ch].is_fresh);
            if ((0U != raw_valid) && (0U != att.valid))
            {
                center_mm = TOF_EstimateCenterHeight(data.distance_mm[ch], &att, ch);
                sum_mm[ch] += (uint32)center_mm;
                sum_shadow_mm[ch] += TOF_ShadowProjectDistanceMm(data.distance_mm[ch], &att, ch);
                cnt[ch]++;
            }
        }
        system_delay_ms(TOF_CALIBRATION_DT_MS);
    }

    /* ---- 阶段3：计算各通道均值、公共中心、零偏和偏差检查 ---- */
    valid_ch_count = 0;
    common_center  = 0;
    common_shadow  = 0;
    max_mean = 0;
    min_mean = 65535;

    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        if (cnt[ch] > 0U)
        {
            mean_mm[ch] = (int32)(sum_mm[ch] / cnt[ch]);
            mean_shadow_mm[ch] = (int32)(sum_shadow_mm[ch] / (float)cnt[ch]);
            common_center += mean_mm[ch];
            common_shadow += mean_shadow_mm[ch];
            valid_ch_count++;
            if (mean_mm[ch] > max_mean)
            {
                max_mean = mean_mm[ch];
            }
            if (mean_mm[ch] < min_mean)
            {
                min_mean = mean_mm[ch];
            }
        }
    }

    g_tof_calibration_ok = 0U;

    if (valid_ch_count >= 2)
    {
        /* 对齐各通道到公共中心 */
        common_center /= valid_ch_count;
        common_shadow /= valid_ch_count;
        for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
        {
            if (mean_mm[ch] >= 0)
            {
                s_tof_offset_mm[ch] = (int16)(common_center - mean_mm[ch]);
                s_tof_shadow_offset_mm[ch] = (int16)(common_shadow - mean_shadow_mm[ch]);
            }
        }

        /* 偏差检查：通道间最大差距不超过100mm才认为校准通过 */
        if ((max_mean - min_mean) <= (int32)TOF_CALIB_DEVIATION_MAX_MM)
        {
            g_tof_calibration_ok = 1U;
        }
        /* else: 偏差过大，g_tof_calibration_ok = 0U（已初始化） */
    }
    /* valid_ch_count == 0 或 1：通道数量不足，校准失败 */

    /* 重置通道状态（清除校准采样过程中积累的历史缓存） */
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        TOF_ResetChannelState(&s_tof_state[ch]);
    }
    TOF_ResetFusionState();
}

/*
 * 函数功能：更新四路TOF观测并输出融合高度结果（100Hz调用）。
 *
 * 融合算法：
 *   1. 每帧计算一次bias修正的姿态参数（2次sinf + 2次cosf）。
 *   2. 逐通道：原始有效性检查 → 几何修正推算中心高度 → 零偏修正 → 突变检测。
 *   3. 多通道中值计算 + 异常值剔除（距中值>80mm的通道剔除）。
 *   4. 等权均值融合剩余通道，步进限幅平滑输出。
 */
void TOF_Update(void)
{
#if 0
    uint16 ch_center_mm[VL53L1X_CHANNEL_COUNT];   /* 几何修正+零偏后的中心高度估计 */
    uint8  ch_fusion_valid[VL53L1X_CHANNEL_COUNT]; /* 通过突变检测后可参与融合的标志 */
    uint8  in_fusion[VL53L1X_CHANNEL_COUNT];       /* 通过异常值剔除后的最终融合标志 */
    uint16 display_mm[VL53L1X_CHANNEL_COUNT];      /* 各通道显示值（保持或实测） */
#endif
    TOF_Update_Robust();
#if 0

    if (0U == s_tof_inited)
    {
        g_tof1_valid = 0U;
        g_tof2_valid = 0U;
        g_tof3_valid = 0U;
        g_tof4_valid = 0U;
        g_tof_fused_valid = 0U;
        g_tof_fused_vz_mps = 0.0f;
        g_tof_fused_source = TOF_SRC_NONE;
        g_tof_measure_height_mm = (uint16)VL53L1X_VALID_RANGE_MAX;
        g_tof_measure_class = TOF_MEASURE_CLASS_NONE;
        g_tof_measure_mask = 0U;
        return;
    }

    /* ---- 步骤1：预计算本帧姿态参数（每帧只调用一次sinf/cosf对） ---- */
    att = TOF_ComputeAttitudeTerms();

    /* 读取四路最新原始数据 */
    (void)VL53L1X_read_data(&data);

    /* ---- 步骤2：逐通道处理（有效性 → 几何修正 → 突变检测） ---- */
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        raw_valid = TOF_IsRawMeasurementValid(data.distance_mm[ch],
                                              data.range_status[ch],
                                              g_vl53l1x_diag[ch].ready,
                                              g_vl53l1x_diag[ch].is_fresh);
        ch_raw_valid[ch]    = raw_valid;
        ch_fusion_valid[ch] = 0U;
        ch_center_mm[ch]    = VL53L1X_INVALID_DISTANCE_MM;
        display_mm[ch]      = VL53L1X_INVALID_DISTANCE_MM;

        if ((0U != raw_valid) && (0U != att.valid))
        {
            uint16 center_mm;
            uint16 corrected_mm;
            uint16 delta;

            /* 几何修正：将TOF斜距转换为飞机中心高度估计 */
            center_mm    = TOF_EstimateCenterHeight(data.distance_mm[ch], &att, ch);
            /* 零偏修正：对齐通道到公共中心 */
            corrected_mm = TOF_ApplyOffsetClamp(center_mm, s_tof_offset_mm[ch]);

            ch_center_mm[ch] = corrected_mm;
            display_mm[ch]   = corrected_mm;

            /* 冻结检测：连续相同值计数 */
            if ((0U != s_tof_state[ch].has_last_mm) &&
                (corrected_mm == s_tof_state[ch].last_mm))
            {
                if (s_tof_state[ch].same_value_streak < 65535U)
                {
                    s_tof_state[ch].same_value_streak++;
                }
            }
            else
            {
                s_tof_state[ch].same_value_streak = 0U;
            }

            /* 突变检测（电缆遮挡等瞬时干扰抑制）：
             * 若当前值与上一帧差超过SPIKE_GATE，视为突变候选。
             * 突变持续不足SPIKE_PERSIST_FRAMES帧时，该通道本帧不参与融合。
             * 突变持续达到阈值帧数后接受为真实变化（重置计数）。 */
            if (0U != s_tof_state[ch].has_last_mm)
            {
                delta = TOF_AbsDiffU16(corrected_mm, s_tof_state[ch].last_mm);
                if (delta > TOF_SPIKE_GATE_MM)
                {
                    /* 更新last_mm追踪变化趋势，但本帧不参与融合 */
                    s_tof_state[ch].last_mm = corrected_mm;
                    s_tof_state[ch].fresh_now = g_vl53l1x_diag[ch].is_fresh;
                    s_tof_state[ch].invalid_streak = 0U;

                    if (s_tof_state[ch].spike_count < TOF_SPIKE_PERSIST_FRAMES)
                    {
                        s_tof_state[ch].spike_count++;
                        continue; /* 不设 ch_fusion_valid = 1，跳过本通道 */
                    }
                    else
                    {
                        /* 持续变化已超过阈值帧数，接受为真实高度变化 */
                        s_tof_state[ch].spike_count = 0U;
                        s_tof_state[ch].has_last_mm = 1U;
                        ch_fusion_valid[ch] = 1U;
                        continue;
                    }
                }
                else
                {
                    s_tof_state[ch].spike_count = 0U;
                }
            }
            else
            {
                s_tof_state[ch].spike_count = 0U;
            }

            s_tof_state[ch].last_mm     = corrected_mm;
            s_tof_state[ch].has_last_mm = 1U;
            s_tof_state[ch].fresh_now   = g_vl53l1x_diag[ch].is_fresh;
            s_tof_state[ch].invalid_streak = 0U;
            ch_fusion_valid[ch] = 1U;
        }
        else
        {
            /* 原始无效或倾角过大 */
            if (s_tof_state[ch].invalid_streak < 65535U)
            {
                s_tof_state[ch].invalid_streak++;
            }
            s_tof_state[ch].fresh_now = 0U;
            s_tof_state[ch].same_value_streak = 0U;
            s_tof_state[ch].spike_count = 0U;

            /* 显示值：若有历史有效值，保持显示 */
            if (0U != s_tof_state[ch].has_last_mm)
            {
                display_mm[ch] = s_tof_state[ch].last_mm;
            }
        }
    }

    /* 更新各通道显示变量和原始有效标志 */
    g_tof1_height_mm = display_mm[0];
    g_tof2_height_mm = display_mm[1];
    g_tof3_height_mm = display_mm[2];
    g_tof4_height_mm = display_mm[3];
    g_tof1_valid = ch_raw_valid[0];
    g_tof2_valid = ch_raw_valid[1];
    g_tof3_valid = ch_raw_valid[2];
    g_tof4_valid = ch_raw_valid[3];

    /* ---- 步骤2.5：大倾角保护 ---- */
    TOF_FusedPredict(dt_s);
    {
        /* 飞机倾角过大，TOF数据不可信，短时保持融合值 */
        if (0U != att.valid)
        {
            s_fused_invalid_hold_count++;
        }
        else
        {
            g_tof_fused_valid = 0U;
            g_tof_fused_source = TOF_SRC_NONE;
        }
        g_tof2_used_in_fusion = 0U;
        g_tof3_used_in_fusion = 0U;
        return;
    }

    /* ---- 步骤3：统计融合候选通道 ---- */
    candidate_count = 0U;
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        if (0U != ch_fusion_valid[ch])
        {
            candidate_count++;
        }
        in_fusion[ch] = 0U;
    }

    /* ---- 步骤4：异常值剔除（>=2路时做中值过滤，单路直接使用） ---- */
    candidate_count = TOF_BuildFusionMeasurement(ch_fusion_valid,
                                                 ch_center_mm,
                                                 &source_mm,
                                                 &source);
    if (candidate_count >= 2U)
    {
        /* 将候选值排序，计算中值 */
        sorted_count = 0U;
        for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
        {
            if (0U != ch_fusion_valid[ch])
            {
                sorted[sorted_count] = ch_center_mm[ch];
                sorted_count++;
            }
        }
        TOF_Sort4(sorted, sorted_count);

        if (sorted_count == 2U)
        {
            median_val = (uint16)(((uint32)sorted[0] + (uint32)sorted[1]) / 2U);
        }
        else if (sorted_count == 3U)
        {
            median_val = sorted[1];
        }
        else
        {
            /* 4路：中间两者均值 */
            median_val = (uint16)(((uint32)sorted[1] + (uint32)sorted[2]) / 2U);
        }

        /* 剔除距中值超过OUTLIER_GATE_MM的通道（突发异常值） */
        for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
        {
            if ((0U != ch_fusion_valid[ch]) &&
                (TOF_AbsDiffU16(ch_center_mm[ch], median_val) <= TOF_OUTLIER_GATE_MM))
            {
                in_fusion[ch] = 1U;
            }
        }
    }
    else
    {
        /* 0路或1路有效：直接标记参与融合 */
        for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
        {
            if (0U != ch_fusion_valid[ch])
            {
                in_fusion[ch] = 1U;
            }
        }
    }

    /* ---- 步骤5：计算融合均值和来源标识 ---- */
    sum_fused   = 0U;
    final_count = 0U;
    source      = TOF_SRC_NONE;

    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        if (0U != in_fusion[ch])
        {
            sum_fused += (uint32)ch_center_mm[ch];
            if (final_count == 0U)
            {
                /* 记录第一个通道为单通道来源标识，若后续多路则覆盖为MULTI */
                source = (uint8)(TOF_SRC_CH1 + ch);
            }
            final_count++;
        }
    }

    if ((0U == final_count) && (candidate_count >= 2U))
    {
        sum_fused = (uint32)median_val;
        final_count = 1U;
        source = TOF_SRC_MULTI;
    }

    if (final_count == 0U)
    {
        /* 无有效来源：短时保持，超时后输出无效 */
        if ((0U != g_tof_fused_valid) && (s_fused_invalid_hold_count < TOF_FUSED_HOLD_FRAMES))
        {
            s_fused_invalid_hold_count++;
        }
        else
        {
            g_tof_fused_valid = 0U;
            g_tof_fused_source = TOF_SRC_NONE;
        }
        g_tof2_used_in_fusion = 0U;
        g_tof3_used_in_fusion = 0U;
        return;
    }

    if (final_count >= 2U)
    {
        source = TOF_SRC_MULTI;
    }

    source_mm = (uint16)(sum_fused / (uint32)final_count);
    s_fused_invalid_hold_count = 0U;

    /* ---- 步骤6：步进限幅 + IIR低通滤波输出 ----
     *
     * 设计说明：
     *   步进限幅参考值（s_fused_step_ref_mm）与IIR滤波状态（s_fused_lpf_mm）分开维护。
     *   步进限幅相对于s_fused_step_ref_mm计算，不受滤波输出的滞后干扰。
     *   IIR滤波平滑步进限幅后的值，进一步抑制传感器测量噪声，兼顾实时性。
     *
     * 噪声抑制效果（α=0.45，100Hz）：
     *   - 稳定悬停时：通道均值噪声约±0.75mm/帧，IIR后约±0.22mm/帧（降噪70%）
     *   - 动态跟踪（1m/s下降）：IIR引入约12ms额外滞后，控制系统可接受
     */

    /* 确定步进限幅量：多通道高度一致时允许更大步进以提高响应速度 */
    if (0U == g_tof_fused_valid)
    {
        /* 首次获得有效融合值：直接建立内部状态，不做步进限幅 */
        TOF_SeedFusionKalman(source_mm);
    }
    else
    {
        float fused_mm = Kalman1D_Update(&s_tof_fused_kf, (float)source_mm);

        /* 步进限幅：相对于内部参考值（s_fused_step_ref_mm）计算，防止滤波滞后影响 */

        /* IIR低通滤波：平滑步进后的值，抑制剩余传感器噪声 */
        if (fused_mm < 0.0f)
        {
            fused_mm = 0.0f;
        }
        else if (fused_mm > 65535.0f)
        {
            fused_mm = 65535.0f;
        }

        g_tof_fused_height_mm = (uint16)(fused_mm + 0.5f);
        g_tof_fused_valid = 1U;
    }

    g_tof_fused_source = source;
    /* 向后兼容：更新通道2/3使用标志 */
    g_tof2_used_in_fusion = (candidate_count > 0U) ? ch_fusion_valid[1] : 0U;
    g_tof3_used_in_fusion = (candidate_count > 0U) ? ch_fusion_valid[2] : 0U;
#endif

}

/*
 * 函数功能：100Hz任务入口，更新TOF融合高度。
 * 0411 0228 zyz实际测试执行一次 花费791.5 μs
 */
void TOF_Shadow_Update(float dt_s)
{
    uint8 ch;
    uint8 raw_valid;
    uint8 ch_valid[VL53L1X_CHANNEL_COUNT];
    float ch_meas_mm[VL53L1X_CHANNEL_COUNT];
    float measure_mm;
    float measure_var_mm2;
    TOFAttitudeTerms_t att;

    if (0U == s_tof_inited)
    {
        TOF_ShadowResetState();
        return;
    }

    att = TOF_ComputeAttitudeTerms();

    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        raw_valid = TOF_IsRawMeasurementValid(VL53L1X_data.distance_mm[ch],
                                              VL53L1X_data.range_status[ch],
                                              g_vl53l1x_diag[ch].ready,
                                              g_vl53l1x_diag[ch].is_fresh);
        ch_valid[ch] = 0U;
        ch_meas_mm[ch] = 0.0f;

        if ((0U != raw_valid) && (0U != att.valid))
        {
            ch_valid[ch] = 1U;
            ch_meas_mm[ch] = TOF_ShadowProjectDistanceMm(VL53L1X_data.distance_mm[ch], &att, ch);
        }
    }

    TOF_ShadowPredict(dt_s);

    if ((0U != att.valid) &&
        (0U != TOF_ShadowBuildMeasurement(ch_valid, ch_meas_mm, &measure_mm, &measure_var_mm2)))
    {
        measure_mm = TOF_ClampF(measure_mm, 0.0f, 65535.0f);

        if (0U == s_tof_shadow_state.seeded)
        {
            TOF_ShadowSeed(measure_mm);
        }
        else
        {
            TOF_ShadowCorrect(measure_mm, measure_var_mm2);
        }

        s_tof_shadow_miss_count = 0U;
        g_tof_shadow_valid = 1U;
    }
    else if (0U != s_tof_shadow_state.seeded)
    {
        if (s_tof_shadow_miss_count < 255U)
        {
            s_tof_shadow_miss_count++;
        }

        if (s_tof_shadow_miss_count > TOF_SHADOW_TIMEOUT_FRAMES)
        {
            s_tof_shadow_state.seeded = 0U;
            s_tof_shadow_state.vz_mmps = 0.0f;
            s_tof_shadow_slope_valid = 0U;
            g_tof_shadow_valid = 0U;
        }
        else
        {
            g_tof_shadow_valid = 1U;
        }
    }
    else
    {
        g_tof_shadow_valid = 0U;
        g_tof_shadow_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
        g_tof_shadow_vz_mps = 0.0f;
        return;
    }

    s_tof_shadow_state.height_mm = TOF_ClampF(s_tof_shadow_state.height_mm, 0.0f, 65535.0f);
    s_tof_shadow_state.vz_mmps = TOF_ClampF(s_tof_shadow_state.vz_mmps, -3000.0f, 3000.0f);

    g_tof_shadow_height_mm = s_tof_shadow_state.height_mm;
    g_tof_shadow_vz_mps = 0.001f * s_tof_shadow_state.vz_mmps;
}

void TOF_update_100HZ(void)
{
    TOF_Update();
}
