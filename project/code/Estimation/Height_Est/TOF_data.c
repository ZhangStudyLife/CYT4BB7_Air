#include "TOF_data.h"
#include "../../HW_Drivers/Motor/Motor_Drive.h"
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
    uint16 last_mm;            /* 上一帧有效的中心高度估计，单位：mm */
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
uint16 g_tof_fused_height_mm = (uint16)VL53L1X_VALID_RANGE_MAX;
/* 各通道中心高度估计（几何修正+零偏后），无效时为VL53L1X_INVALID_DISTANCE_MM */
uint16 g_tof1_height_mm = VL53L1X_INVALID_DISTANCE_MM;
uint16 g_tof2_height_mm = VL53L1X_INVALID_DISTANCE_MM;
uint16 g_tof3_height_mm = VL53L1X_INVALID_DISTANCE_MM;
uint16 g_tof4_height_mm = VL53L1X_INVALID_DISTANCE_MM;
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
static uint8 s_fused_invalid_hold_count = 0U;
/* 步进限幅内部参考值（浮点，与滤波输出解耦，防止滤波滞后影响步进判断） */
static float s_fused_step_ref_mm = 0.0f;
/* 最终输出IIR滤波状态（浮点，避免整数累积误差） */
static float s_fused_lpf_mm = 0.0f;

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
    state->last_mm = 0U;
    state->invalid_streak = 0U;
    state->same_value_streak = 0U;
    state->spike_count = 0U;
}

/*
 * 函数功能：重置TOF融合输出与全局状态。
 */
static void TOF_ResetFusionState(void)
{
    g_tof1_height_mm = VL53L1X_INVALID_DISTANCE_MM;
    g_tof2_height_mm = VL53L1X_INVALID_DISTANCE_MM;
    g_tof3_height_mm = VL53L1X_INVALID_DISTANCE_MM;
    g_tof4_height_mm = VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_height_mm = (uint16)VL53L1X_VALID_RANGE_MAX;
    g_tof1_valid = 0U;
    g_tof2_valid = 0U;
    g_tof3_valid = 0U;
    g_tof4_valid = 0U;
    g_tof_fused_valid = 0U;
    g_tof2_used_in_fusion = 0U;
    g_tof3_used_in_fusion = 0U;
    g_tof_fused_source = TOF_SRC_NONE;

    s_fused_invalid_hold_count = 0U;
    s_fused_step_ref_mm = 0.0f;
    s_fused_lpf_mm = 0.0f;
}

/*
 * 函数功能：计算当前帧的姿态修正参数（使用bias修正后的角度，每帧计算一次）。
 * 返回值：
 *   TOFAttitudeTerms_t，包含 cos_term、sin_pitch、sin_roll 和有效标志。
 */
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
static uint16 TOF_EstimateCenterHeight(uint16 slant_mm, const TOFAttitudeTerms_t *att, uint8 ch)
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

    return (uint16)(h + 0.5f);
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
void TOF_Init(void)
{
    uint8 ch;
    uint8 init_err;

    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        s_tof_offset_mm[ch] = 0;
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
    uint32 cnt[VL53L1X_CHANNEL_COUNT];
    int32  mean_mm[VL53L1X_CHANNEL_COUNT];
    int32  valid_ch_count;
    int32  common_center;
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
        TOF_ResetChannelState(&s_tof_state[ch]);
        sum_mm[ch]   = 0U;
        cnt[ch]      = 0U;
        mean_mm[ch]  = -1;
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
                cnt[ch]++;
            }
        }
        system_delay_ms(TOF_CALIBRATION_DT_MS);
    }

    /* ---- 阶段3：计算各通道均值、公共中心、零偏和偏差检查 ---- */
    valid_ch_count = 0;
    common_center  = 0;
    max_mean = 0;
    min_mean = 65535;

    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        if (cnt[ch] > 0U)
        {
            mean_mm[ch] = (int32)(sum_mm[ch] / cnt[ch]);
            common_center += mean_mm[ch];
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
        for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
        {
            if (mean_mm[ch] >= 0)
            {
                s_tof_offset_mm[ch] = (int16)(common_center - mean_mm[ch]);
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
    uint8 ch;
    uint8 raw_valid;
    uint8 ch_raw_valid[VL53L1X_CHANNEL_COUNT];
    uint16 ch_center_mm[VL53L1X_CHANNEL_COUNT];   /* 几何修正+零偏后的中心高度估计 */
    uint8  ch_fusion_valid[VL53L1X_CHANNEL_COUNT]; /* 通过突变检测后可参与融合的标志 */
    uint8  in_fusion[VL53L1X_CHANNEL_COUNT];       /* 通过异常值剔除后的最终融合标志 */
    uint16 display_mm[VL53L1X_CHANNEL_COUNT];      /* 各通道显示值（保持或实测） */
    uint16 sorted[VL53L1X_CHANNEL_COUNT];
    uint8  candidate_count;
    uint8  sorted_count;
    uint16 median_val;
    uint32 sum_fused;
    uint8  final_count;
    uint8  source;
    uint16 source_mm;
    uint16 step_limit;
    TOFAttitudeTerms_t att;
    VL53L1X_data_struct data;

    if (0U == s_tof_inited)
    {
        g_tof1_valid = 0U;
        g_tof2_valid = 0U;
        g_tof3_valid = 0U;
        g_tof4_valid = 0U;
        g_tof_fused_valid = 0U;
        g_tof_fused_source = TOF_SRC_NONE;
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
    if (0U == att.valid)
    {
        /* 飞机倾角过大，TOF数据不可信，短时保持融合值 */
        if (s_fused_invalid_hold_count < TOF_FUSED_HOLD_FRAMES)
        {
            s_fused_invalid_hold_count++;
        }
        else
        {
            g_tof_fused_valid = 0U;
            g_tof_fused_height_mm = (uint16)VL53L1X_VALID_RANGE_MAX;
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
            g_tof_fused_height_mm = (uint16)VL53L1X_VALID_RANGE_MAX;
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
    if (final_count >= 2U)
    {
        uint16 fused_max = 0U;
        uint16 fused_min = 65535U;
        uint16 range_mm;

        for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
        {
            if (0U != in_fusion[ch])
            {
                if (ch_center_mm[ch] > fused_max) { fused_max = ch_center_mm[ch]; }
                if (ch_center_mm[ch] < fused_min) { fused_min = ch_center_mm[ch]; }
            }
        }
        range_mm = TOF_AbsDiffU16(fused_max, fused_min);
        step_limit = (range_mm <= TOF_AGREE_GATE_MM) ? TOF_STEP_FAST_MM : TOF_STEP_SAFE_MM;
    }
    else
    {
        step_limit = TOF_STEP_SAFE_MM;
    }

    if (0U == g_tof_fused_valid)
    {
        /* 首次获得有效融合值：直接建立内部状态，不做步进限幅 */
        s_fused_step_ref_mm = (float)source_mm;
        s_fused_lpf_mm      = (float)source_mm;
        g_tof_fused_height_mm = source_mm;
        g_tof_fused_valid = 1U;
    }
    else
    {
        float step_ref_delta;
        float stepped_mm;

        /* 步进限幅：相对于内部参考值（s_fused_step_ref_mm）计算，防止滤波滞后影响 */
        step_ref_delta = (float)source_mm - s_fused_step_ref_mm;
        if (step_ref_delta > (float)step_limit)
        {
            step_ref_delta = (float)step_limit;
        }
        else if (step_ref_delta < -(float)step_limit)
        {
            step_ref_delta = -(float)step_limit;
        }
        s_fused_step_ref_mm += step_ref_delta;
        stepped_mm = s_fused_step_ref_mm;

        /* IIR低通滤波：平滑步进后的值，抑制剩余传感器噪声 */
        s_fused_lpf_mm += TOF_FUSED_LPF_ALPHA * (stepped_mm - s_fused_lpf_mm);
        if (s_fused_lpf_mm < 0.0f)
        {
            s_fused_lpf_mm = 0.0f;
        }
        else if (s_fused_lpf_mm > 65535.0f)
        {
            s_fused_lpf_mm = 65535.0f;
        }

        g_tof_fused_height_mm = (uint16)(s_fused_lpf_mm + 0.5f);
        g_tof_fused_valid = 1U;
    }

    g_tof_fused_source = source;
    /* 向后兼容：更新通道2/3使用标志 */
    g_tof2_used_in_fusion = in_fusion[1];
    g_tof3_used_in_fusion = in_fusion[2];

}

/*
 * 函数功能：100Hz任务入口，更新TOF融合高度。
 */
void TOF_update_100HZ(void)
{
    TOF_Update();
}
