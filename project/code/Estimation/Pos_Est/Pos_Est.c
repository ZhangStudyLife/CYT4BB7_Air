#include "Pos_Est.h"
#include "zf_common_headfile.h"
#include <math.h>

#ifndef sq
#define sq(x) ((x) * (x))
#endif

#define GRAVITY_CMSS (980.0f)                                 /* 重力加速度，单位 cm/s^2 */
#define INAV_ACC_BIAS_ACCEPTANCE_VALUE (GRAVITY_CMSS * 0.25f) /* 加速度偏置最大可接受范围 */
#define POS_EST_PI (3.14159265359f)

#define POS_EST_AXIS_RIGHT (0U)   /* 机体右向速度通道，对应 roll 控制 */
#define POS_EST_AXIS_FORWARD (1U) /* 机体前向速度通道，对应 pitch 控制 */
#define POS_EST_AXIS_COUNT (2U)

#define POS_EST_DEBUG_LOG_ENABLE (1U) /* 1=输出 16 路并行验证日志 */

#define POS_EST_ACC_DEADBAND_CMSS (6.0f)     /* 水平加速度死区 */
#define POS_EST_ACC_UP_DEADBAND_MMSS (60.0f) /* 竖直加速度死区 */
#define POS_EST_ACC_LIMIT_CMSS (1000.0f)     /* 水平加速度限幅 */
#define POS_EST_ACC_Z_LIMIT_MMSS (10000.0f)  /* 竖直加速度限幅 */

#define POS_EST_ACC_NOTCH_HZ (85.0f) /* 离线筛选得到的优选窄带抑制中心 */
#define POS_EST_ACC_NOTCH_Q (3.0f)   /* 离线筛选得到的优选 Q 值 */
#define POS_EST_ACC_LPF_HZ (15.0f)   /* 在噪声抑制与时延之间折中的优选截止频率 */

#define POS_EST_FUSION_W_XY_FLOW_P (1.0f)     /* 保持位置锚定权重不变，避免实飞悬停时过度拉扯 */
#define POS_EST_FUSION_W_XY_FLOW_V (4.5f)     /* 两份实飞日志联合回放后继续上调速度观测权重 */
#define POS_EST_FUSION_W_ACC_BIAS (0.003f)    /* 进一步降低偏置学习，减少悬停慢偏移被吸收到零偏 */
#define POS_EST_FUSION_W_XY_RES_V (1.0f)      /* 延长失效后更快收敛到零速，降低长失效漂移 */
#define POS_EST_FUSION_DEAD_MAX_S (0.20f)     /* 实飞下缩短纯惯导死推窗口，减少创新超门限连锁失效 */
#define POS_EST_FUSION_RECOVER_RAMP_S (0.20f) /* 光流恢复增益拉起时间 */

#define POS_EST_FLOW_TIMEOUT_S (0.06f)        /* 60ms 未见新流观测则超时 */
#define POS_EST_FLOW_INNOV_GATE_CMPS (130.0f) /* 实飞回放后适度放宽创新门限，减少误判失效 */
#define POS_EST_FLOW_MIN_HEIGHT_M (0.1f)      /* 光流有效最小高度 */
#define POS_EST_FLOW_MAX_HEIGHT_M (2.0f)      /* 光流有效最大高度 */
#define POS_EST_FLOW_SQUAL_HIGH (40U)         /* 光流有效滞回高阈值 */
#define POS_EST_FLOW_SQUAL_LOW (25U)          /* 光流有效滞回低阈值 */
#define POS_EST_FLOW_PIXEL_LIMIT (40.0f)      /* 像素突发值阈值 */
#define POS_EST_FLOW_HEIGHT_OFFSET_M (0.05f)  /* 光流模块相对 TOF 的安装高度偏移 */
#define POS_EST_GYRO_LIMIT_DPS (68.0f)        /* 光流补偿角速度限幅 */

extern volatile uint32 tick_1000us_cnt;

typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float d1;
    float d2;
} PosEstBiquad_t;

typedef struct
{
    PosEstBiquad_t notch[POS_EST_AXIS_COUNT];
    PosEstBiquad_t lpf[POS_EST_AXIS_COUNT];
    uint8 initialized;
} PosEstAccFilterState_t;

typedef struct
{
    float acc_raw_cmpss[POS_EST_AXIS_COUNT];   /* 原始水平线性加速度，单位 cm/s^2 */
    float acc_filt_cmpss[POS_EST_AXIS_COUNT];  /* 滤波后水平线性加速度，单位 cm/s^2 */
    float acc_bias_cmpss[POS_EST_AXIS_COUNT];  /* 在线估计偏置，单位 cm/s^2 */
    float flow_pos_cm[POS_EST_AXIS_COUNT];     /* 光流积分位置，单位 cm */
    float innovation_cmps[POS_EST_AXIS_COUNT]; /* 光流速度新息，单位 cm/s */
    float recover_gain;                        /* 光流恢复权重 0~1 */
    float dead_reckon_time_s;                  /* 连续死推时长，单位 s */
    float last_flow_dt_s;                      /* 最近一帧光流时间间隔，单位 s */
    uint32 last_flow_update_tick;              /* 最近一帧有效光流的毫秒 tick */
    uint8 flow_valid;                          /* 光流是否可用于融合 */
    uint8 flow_quality_ok;                     /* 光流质量滞回状态 */
    uint8 flow_pos_initialized;                /* 光流积分位置是否已经锚定 */
    uint8 fusion_mode;                         /* 当前融合模式 */
} PosEstXYParallelState_t;

volatile opFlow_t opFlow = {0};
estimator_t estimator =
    {
        .vAccDeadband = POS_EST_ACC_DEADBAND_CMSS,
        .accBias = {0.0f, 0.0f, 0.0f},
        .acc = {0.0f, 0.0f, 0.0f},
        .vel = {0.0f, 0.0f, 0.0f},
        .pos = {0.0f, 0.0f, 0.0f}};
volatile PosEstXYOutput_t g_pos_est_xy_output = {0};

static PosEstAccFilterState_t s_pos_est_acc_filter = {0};
static PosEstXYParallelState_t s_pos_est_xy_state = {0};

static float gx_sum = 0.0f;      /* 光流补偿窗口内 X 轴陀螺累计 */
static float gy_sum = 0.0f;      /* 光流补偿窗口内 Y 轴陀螺累计 */
static uint16_t gyro_count = 0U; /* 光流补偿窗口内累计样本数 */

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

static float Pos_Est_ApplyDeadband(float value, float deadband)
{
    if (value > deadband)
    {
        return value - deadband;
    }
    if (value < -deadband)
    {
        return value + deadband;
    }
    return 0.0f;
}

static float Pos_Est_GetLoopDt(uint32 *last_tick, float fallback_dt_s, float min_dt_s, float max_dt_s)
{
    uint32 current_tick = tick_1000us_cnt;
    float dt_s = fallback_dt_s;

    if (*last_tick != 0U)
    {
        dt_s = (float)(current_tick - *last_tick) * 0.001f;
        if ((dt_s < min_dt_s) || (dt_s > max_dt_s))
        {
            dt_s = fallback_dt_s;
        }
    }

    *last_tick = current_tick;
    return dt_s;
}

static void inavFilterPredict(int axis, float dt, float acc)
{
    estimator.pos[axis] += estimator.vel[axis] * dt + acc * dt * dt * 0.5f;
    estimator.vel[axis] += acc * dt;
}

static void inavFilterCorrectPos(int axis, float dt, float residual, float weight)
{
    float ewdt = residual * weight * dt;
    estimator.pos[axis] += ewdt;
    estimator.vel[axis] += weight * ewdt;
}

static void inavFilterCorrectVel(int axis, float dt, float residual, float weight)
{
    estimator.vel[axis] += residual * weight * dt;
}

static void PosEstBiquad_Reset(PosEstBiquad_t *filter)
{
    filter->d1 = 0.0f;
    filter->d2 = 0.0f;
}

static void PosEstBiquad_InitLPF(PosEstBiquad_t *filter, float sample_rate_hz, float cutoff_hz)
{
    float w0 = 2.0f * POS_EST_PI * cutoff_hz / sample_rate_hz;
    float sw0 = sinf(w0);
    float cw0 = cosf(w0);
    float alpha = sw0 / (2.0f * 0.70710678f);
    float a0 = 1.0f + alpha;

    filter->b0 = (1.0f - cw0) * 0.5f / a0;
    filter->b1 = (1.0f - cw0) / a0;
    filter->b2 = (1.0f - cw0) * 0.5f / a0;
    filter->a1 = (-2.0f * cw0) / a0;
    filter->a2 = (1.0f - alpha) / a0;
    PosEstBiquad_Reset(filter);
}

static void PosEstBiquad_InitNotch(PosEstBiquad_t *filter, float sample_rate_hz, float center_hz, float q)
{
    float w0 = 2.0f * POS_EST_PI * center_hz / sample_rate_hz;
    float sw0 = sinf(w0);
    float cw0 = cosf(w0);
    float alpha = sw0 / (2.0f * q);
    float a0 = 1.0f + alpha;

    filter->b0 = 1.0f / a0;
    filter->b1 = (-2.0f * cw0) / a0;
    filter->b2 = 1.0f / a0;
    filter->a1 = (-2.0f * cw0) / a0;
    filter->a2 = (1.0f - alpha) / a0;
    PosEstBiquad_Reset(filter);
}

static float PosEstBiquad_Apply(PosEstBiquad_t *filter, float input)
{
    float output = filter->b0 * input + filter->d1;
    filter->d1 = filter->b1 * input - filter->a1 * output + filter->d2;
    filter->d2 = filter->b2 * input - filter->a2 * output;
    return output;
}

static void Pos_Est_ResetParallelState(void)
{
    uint8_t axis;

    s_pos_est_xy_state = (PosEstXYParallelState_t){0};
    s_pos_est_xy_state.fusion_mode = POS_EST_FUSION_MODE_DEAD_RECKONING;

    for (axis = 0U; axis < POS_EST_AXIS_COUNT; ++axis)
    {
        PosEstBiquad_InitNotch(&s_pos_est_acc_filter.notch[axis], 250.0f, POS_EST_ACC_NOTCH_HZ, POS_EST_ACC_NOTCH_Q);
        PosEstBiquad_InitLPF(&s_pos_est_acc_filter.lpf[axis], 250.0f, POS_EST_ACC_LPF_HZ);
    }

    s_pos_est_acc_filter.initialized = 0U;

    g_pos_est_xy_output = (PosEstXYOutput_t){0};
    g_pos_est_xy_output.fusion_mode = POS_EST_FUSION_MODE_DEAD_RECKONING;
}

static void Pos_Est_ResetFlowAnchor(void)
{
    s_pos_est_xy_state.flow_pos_cm[POS_EST_AXIS_RIGHT] = estimator.pos[POS_EST_AXIS_RIGHT];
    s_pos_est_xy_state.flow_pos_cm[POS_EST_AXIS_FORWARD] = estimator.pos[POS_EST_AXIS_FORWARD];
    s_pos_est_xy_state.flow_pos_initialized = 1U;
}

static void Pos_Est_UpdateXYOutput(void)
{
    uint8_t axis;

    for (axis = 0U; axis < POS_EST_AXIS_COUNT; ++axis) // 两个轴
    {
        g_pos_est_xy_output.vel_flow_cmps[axis] = opFlow.velLpf[axis];  // 光流速度观测
        g_pos_est_xy_output.vel_fused_cmps[axis] = estimator.vel[axis]; // 融合速度输出
        g_pos_est_xy_output.innovation_cmps[axis] = s_pos_est_xy_state.innovation_cmps[axis];
        g_pos_est_xy_output.acc_bias_cmpss[axis] = s_pos_est_xy_state.acc_bias_cmpss[axis]; // 在线估计的加速度偏置
    }

    g_pos_est_xy_output.flow_valid = s_pos_est_xy_state.flow_valid;                 // 当前光流是否可用于融合
    g_pos_est_xy_output.fusion_mode = s_pos_est_xy_state.fusion_mode;               // 当前融合模式
    g_pos_est_xy_output.dead_reckon_time_s = s_pos_est_xy_state.dead_reckon_time_s; // 连续死推时长，单位 s
}

static void Pos_Est_DebugLog(void)
{
#if POS_EST_DEBUG_LOG_ENABLE
    wifi_vofa_JustFloat(16u,
                        s_pos_est_xy_state.acc_raw_cmpss[POS_EST_AXIS_RIGHT],
                        s_pos_est_xy_state.acc_raw_cmpss[POS_EST_AXIS_FORWARD],
                        s_pos_est_xy_state.acc_filt_cmpss[POS_EST_AXIS_RIGHT],
                        s_pos_est_xy_state.acc_filt_cmpss[POS_EST_AXIS_FORWARD],
                        opFlow.velLpf[POS_EST_AXIS_RIGHT],
                        opFlow.velLpf[POS_EST_AXIS_FORWARD],
                        estimator.vel[POS_EST_AXIS_RIGHT],
                        estimator.vel[POS_EST_AXIS_FORWARD],
                        s_pos_est_xy_state.innovation_cmps[POS_EST_AXIS_RIGHT],
                        s_pos_est_xy_state.innovation_cmps[POS_EST_AXIS_FORWARD],
                        (float)g_pmw3901_raw.squal,
                        (float)g_tof_fused_height_mm / 1000.0f,
                        (float)g_pmw3901_raw.deltaX,
                        (float)g_pmw3901_raw.deltaY,
                        (float)s_pos_est_xy_state.flow_valid,
                        (float)s_pos_est_xy_state.fusion_mode);
#endif
}

void Pos_Est_Init(void)
{
    estimator.vAccDeadband = POS_EST_ACC_DEADBAND_CMSS;
    estimator.accBias[0] = 0.0f;
    estimator.accBias[1] = 0.0f;
    estimator.accBias[2] = 0.0f;
    estimator.acc[0] = 0.0f;
    estimator.acc[1] = 0.0f;
    estimator.acc[2] = 0.0f;
    estimator.vel[0] = 0.0f;
    estimator.vel[1] = 0.0f;
    estimator.vel[2] = 0.0f;
    estimator.pos[0] = 0.0f;
    estimator.pos[1] = 0.0f;
    estimator.pos[2] = 0.0f;

    opFlow.pixSum[0] = 0.0f;
    opFlow.pixSum[1] = 0.0f;
    opFlow.pixComp[0] = 0.0f;
    opFlow.pixComp[1] = 0.0f;
    opFlow.pixValid[0] = 0.0f;
    opFlow.pixValid[1] = 0.0f;
    opFlow.pixValidLast[0] = 0.0f;
    opFlow.pixValidLast[1] = 0.0f;
    opFlow.deltaPos[0] = 0.0f;
    opFlow.deltaPos[1] = 0.0f;
    opFlow.deltaVel[0] = 0.0f;
    opFlow.deltaVel[1] = 0.0f;
    opFlow.posSum[0] = 0.0f;
    opFlow.posSum[1] = 0.0f;
    opFlow.velLpf[0] = 0.0f;
    opFlow.velLpf[1] = 0.0f;
    opFlow.isOpFlowOk = 0U;
    opFlow.isDataValid = 0U;

    gx_sum = 0.0f;
    gy_sum = 0.0f;
    gyro_count = 0U;

    Pos_Est_ResetParallelState();
}

void Pos_Est_Update_1000HZ(void)
{
    gx_sum += g_imudata_250hz.gyrox;
    gy_sum += g_imudata_250hz.gyroy;
    gyro_count += 1U;
}

void Pos_Est_Update_250HZ(void)
{
    static uint32 last_pos_tick = 0U;
    static float fusedHeightLpfMm = 0.0f; /* Z 轴融合高度低通，单位 mm */

    float dt_s;
    float ax_level_mps2;
    float ay_level_mps2;
    float az_level_mps2;
    float az_up_mps2;
    float acc_right_raw_cmpss;
    float acc_forward_raw_cmpss;
    float acc_right_use_cmpss;
    float acc_forward_use_cmpss;
    float errPosZMm;
    float flow_age_s;

    dt_s = Pos_Est_GetLoopDt(&last_pos_tick, POS_EST_250HZ_DT, 0.0015f, 0.0100f);

    fusedHeightLpfMm += ((float)g_tof_fused_height_mm - fusedHeightLpfMm) * 0.1f;

    AccelCalibration_GetLevelAccelMps2(&ax_level_mps2, &ay_level_mps2, &az_level_mps2);
    az_up_mps2 = AccelCalibration_GetVerticalAccelUpMps2();
    (void)az_level_mps2;

    acc_right_raw_cmpss = Pos_Est_ApplyDeadband(ay_level_mps2 * 100.0f, POS_EST_ACC_DEADBAND_CMSS);         // 注意 IMU 坐标系与机体坐标系的轴向关系，acc_right_raw_cmpss 对应机体右向
    acc_forward_raw_cmpss = Pos_Est_ApplyDeadband(ax_level_mps2 * 100.0f, POS_EST_ACC_DEADBAND_CMSS);       // 注意 IMU 坐标系与机体坐标系的轴向关系，acc_forward_raw_cmpss 对应机体前向

    // 限幅，防止异常值对滤波器造成过大冲击
    s_pos_est_xy_state.acc_raw_cmpss[POS_EST_AXIS_RIGHT] = Pos_Est_Clampf(acc_right_raw_cmpss, -POS_EST_ACC_LIMIT_CMSS, POS_EST_ACC_LIMIT_CMSS);
    s_pos_est_xy_state.acc_raw_cmpss[POS_EST_AXIS_FORWARD] = Pos_Est_Clampf(acc_forward_raw_cmpss, -POS_EST_ACC_LIMIT_CMSS, POS_EST_ACC_LIMIT_CMSS);

    // 对原始加速度输入进行一级 notch + 一级低通滤波，抑制光流补偿相关频率的噪声，同时控制时延
    // notch参数为 85 Hz, Q=3
    // lpf参数为 15 Hz
    if (s_pos_est_acc_filter.initialized == 0U)
    {
        s_pos_est_xy_state.acc_filt_cmpss[POS_EST_AXIS_RIGHT] = s_pos_est_xy_state.acc_raw_cmpss[POS_EST_AXIS_RIGHT];
        s_pos_est_xy_state.acc_filt_cmpss[POS_EST_AXIS_FORWARD] = s_pos_est_xy_state.acc_raw_cmpss[POS_EST_AXIS_FORWARD];
        s_pos_est_acc_filter.initialized = 1U;
    }
    else
    {
        float acc_right_notch = PosEstBiquad_Apply(&s_pos_est_acc_filter.notch[POS_EST_AXIS_RIGHT], s_pos_est_xy_state.acc_raw_cmpss[POS_EST_AXIS_RIGHT]);
        float acc_forward_notch = PosEstBiquad_Apply(&s_pos_est_acc_filter.notch[POS_EST_AXIS_FORWARD], s_pos_est_xy_state.acc_raw_cmpss[POS_EST_AXIS_FORWARD]);

        s_pos_est_xy_state.acc_filt_cmpss[POS_EST_AXIS_RIGHT] = PosEstBiquad_Apply(&s_pos_est_acc_filter.lpf[POS_EST_AXIS_RIGHT], acc_right_notch);
        s_pos_est_xy_state.acc_filt_cmpss[POS_EST_AXIS_FORWARD] = PosEstBiquad_Apply(&s_pos_est_acc_filter.lpf[POS_EST_AXIS_FORWARD], acc_forward_notch);
    }

    // 对滤波过后的数据仍然进行限幅，防止滤波器状态突变后输出异常值
    s_pos_est_xy_state.acc_filt_cmpss[POS_EST_AXIS_RIGHT] = Pos_Est_Clampf(s_pos_est_xy_state.acc_filt_cmpss[POS_EST_AXIS_RIGHT], -POS_EST_ACC_LIMIT_CMSS, POS_EST_ACC_LIMIT_CMSS);
    s_pos_est_xy_state.acc_filt_cmpss[POS_EST_AXIS_FORWARD] = Pos_Est_Clampf(s_pos_est_xy_state.acc_filt_cmpss[POS_EST_AXIS_FORWARD], -POS_EST_ACC_LIMIT_CMSS, POS_EST_ACC_LIMIT_CMSS);

    // 加速度滤波过后的结果供惯导预测使用，未滤波的原始加速度结果保留给并行融合使用，以免滤波器时变特性对融合稳定性造成影响
    estimator.acc[POS_EST_AXIS_RIGHT] = s_pos_est_xy_state.acc_filt_cmpss[POS_EST_AXIS_RIGHT];
    estimator.acc[POS_EST_AXIS_FORWARD] = s_pos_est_xy_state.acc_filt_cmpss[POS_EST_AXIS_FORWARD];

    az_up_mps2 *= 1000.0f;                          // 此处*了之后单位实际为 mm/s^2
    az_up_mps2 = Pos_Est_ApplyDeadband(az_up_mps2, POS_EST_ACC_UP_DEADBAND_MMSS);       // 死区
    estimator.acc[2] = Pos_Est_Clampf(az_up_mps2, -POS_EST_ACC_Z_LIMIT_MMSS, POS_EST_ACC_Z_LIMIT_MMSS);         // 限幅

    inavFilterPredict(2, dt_s, estimator.acc[2]);
    errPosZMm = fusedHeightLpfMm - estimator.pos[2];             // 竖直方向高度误差，单位 mm
    inavFilterCorrectPos(2, dt_s, errPosZMm, 0.35f);             // 竖直方向位置校正，权重较大以快速锁定高度

    // 惯导预测时的加速度输入为滤波后的加速度减去在线估计的加速度偏置
    acc_right_use_cmpss = estimator.acc[POS_EST_AXIS_RIGHT] - s_pos_est_xy_state.acc_bias_cmpss[POS_EST_AXIS_RIGHT];
    acc_forward_use_cmpss = estimator.acc[POS_EST_AXIS_FORWARD] - s_pos_est_xy_state.acc_bias_cmpss[POS_EST_AXIS_FORWARD];

    // 通过accelerometer进行惯导预测，纯预测时的加速度输入为滤波后的加速度减去在线估计的加速度偏置
    inavFilterPredict(POS_EST_AXIS_RIGHT, dt_s, acc_right_use_cmpss);
    inavFilterPredict(POS_EST_AXIS_FORWARD, dt_s, acc_forward_use_cmpss);

    estimator.accBias[POS_EST_AXIS_RIGHT] = s_pos_est_xy_state.acc_bias_cmpss[POS_EST_AXIS_RIGHT];
    estimator.accBias[POS_EST_AXIS_FORWARD] = s_pos_est_xy_state.acc_bias_cmpss[POS_EST_AXIS_FORWARD];

    if (s_pos_est_xy_state.last_flow_update_tick != 0U)
    {
        flow_age_s = (float)(tick_1000us_cnt - s_pos_est_xy_state.last_flow_update_tick) * 0.001f;
    }
    else
    {
        flow_age_s = 1000.0f;
    }

    if (s_pos_est_xy_state.flow_valid && (flow_age_s <= POS_EST_FLOW_TIMEOUT_S))            // 如果光流有效且未超时，则进行融合
    {
        float w_pos = POS_EST_FUSION_W_XY_FLOW_P * s_pos_est_xy_state.recover_gain;         // 位置权重还要叠乘一个恢复增益，随着连续观测到的有效光流增多而从0逐渐拉升到1，以平滑融合过渡
        float w_vel = POS_EST_FUSION_W_XY_FLOW_V * s_pos_est_xy_state.recover_gain;
        float residual_pos_right;
        float residual_pos_forward;
        float residual_vel_right;
        float residual_vel_forward;

        s_pos_est_xy_state.dead_reckon_time_s = 0.0f;                                       // 收到有效光流观测，重置死推计时

        residual_vel_right = opFlow.velLpf[POS_EST_AXIS_RIGHT] - estimator.vel[POS_EST_AXIS_RIGHT];                     // 速度残差 = 光流观测速度 - 惯导预测速度       
        residual_vel_forward = opFlow.velLpf[POS_EST_AXIS_FORWARD] - estimator.vel[POS_EST_AXIS_FORWARD];               

        residual_pos_right = s_pos_est_xy_state.flow_pos_cm[POS_EST_AXIS_RIGHT] - estimator.pos[POS_EST_AXIS_RIGHT];        // 位置残差 = 光流积分位置 - 惯导预测位置
        residual_pos_forward = s_pos_est_xy_state.flow_pos_cm[POS_EST_AXIS_FORWARD] - estimator.pos[POS_EST_AXIS_FORWARD];

        s_pos_est_xy_state.innovation_cmps[POS_EST_AXIS_RIGHT] = residual_vel_right;        // 速度新息记录在并行状态中，供调试使用
        s_pos_est_xy_state.innovation_cmps[POS_EST_AXIS_FORWARD] = residual_vel_forward;    // 速度新息记录在并行状态中，供调试使用

        inavFilterCorrectPos(POS_EST_AXIS_RIGHT, dt_s, residual_pos_right, w_pos);          // 位置融合，权重较小以避免对惯导预测造成过大冲击
        inavFilterCorrectPos(POS_EST_AXIS_FORWARD, dt_s, residual_pos_forward, w_pos);

        inavFilterCorrectVel(POS_EST_AXIS_RIGHT, dt_s, residual_vel_right, w_vel);          // 速度融合，权重较大以快速锁定速度，进而通过预测快速修正位置
        inavFilterCorrectVel(POS_EST_AXIS_FORWARD, dt_s, residual_vel_forward, w_vel);

        s_pos_est_xy_state.acc_bias_cmpss[POS_EST_AXIS_RIGHT] += residual_vel_right * POS_EST_FUSION_W_ACC_BIAS * dt_s;
        s_pos_est_xy_state.acc_bias_cmpss[POS_EST_AXIS_FORWARD] += residual_vel_forward * POS_EST_FUSION_W_ACC_BIAS * dt_s;

        s_pos_est_xy_state.acc_bias_cmpss[POS_EST_AXIS_RIGHT] =
            Pos_Est_Clampf(s_pos_est_xy_state.acc_bias_cmpss[POS_EST_AXIS_RIGHT],
                           -INAV_ACC_BIAS_ACCEPTANCE_VALUE,
                           INAV_ACC_BIAS_ACCEPTANCE_VALUE);
        s_pos_est_xy_state.acc_bias_cmpss[POS_EST_AXIS_FORWARD] =
            Pos_Est_Clampf(s_pos_est_xy_state.acc_bias_cmpss[POS_EST_AXIS_FORWARD],
                           -INAV_ACC_BIAS_ACCEPTANCE_VALUE,
                           INAV_ACC_BIAS_ACCEPTANCE_VALUE);

        if (s_pos_est_xy_state.recover_gain < 0.999f)
        {
            s_pos_est_xy_state.fusion_mode = POS_EST_FUSION_MODE_RECOVERING;
        }
        else
        {
            s_pos_est_xy_state.fusion_mode = POS_EST_FUSION_MODE_FLOW_HOLD;
        }
    }
    else
    {
        s_pos_est_xy_state.dead_reckon_time_s += dt_s;
        s_pos_est_xy_state.flow_valid = 0U;
        s_pos_est_xy_state.fusion_mode = POS_EST_FUSION_MODE_DEAD_RECKONING;

        if (s_pos_est_xy_state.dead_reckon_time_s > POS_EST_FUSION_DEAD_MAX_S)
        {
            float decay_factor = 1.0f - POS_EST_FUSION_W_XY_RES_V * dt_s;
            if (decay_factor < 0.0f)
            {
                decay_factor = 0.0f;
            }

            estimator.vel[POS_EST_AXIS_RIGHT] *= decay_factor;
            estimator.vel[POS_EST_AXIS_FORWARD] *= decay_factor;
            s_pos_est_xy_state.flow_pos_initialized = 0U;
        }
    }

    Pos_Est_UpdateXYOutput();
    Pos_Est_DebugLog();
}

void Pos_Est_Update_100HZ(void)
{
    static uint32 last_flow_tick = 0U;
    static float pixelDxLpf = 0.0f;
    static float pixelDyLpf = 0.0f;
    static float gyroRollHist[5] = {0.0f};
    static float gyroPitchHist[5] = {0.0f};
    static float gyroRollSum = 0.0f;
    static float gyroPitchSum = 0.0f;
    static uint8_t gyroHistIndex = 0U;
    float pixelDx;
    float pixelDy;
    float pixelDxRaw;
    float pixelDyRaw;
    float height;
    float coeff;
    float flow_dt_s;
    float gyroRollDps;
    float gyroPitchDps;
    float innov_x;
    float innov_y;
    float innov_norm;
    uint8 flow_valid;

    PMW3901_Update_100HZ();

    // 获取两次调用这个函数的准确的dt，单位秒，允许的范围是0.005s~0.060s，默认值是0.01s
    flow_dt_s = Pos_Est_GetLoopDt(&last_flow_tick, POS_EST_100HZ_DT, 0.005f, 0.060f);

    pixelDxRaw = (float)g_pmw3901_raw.deltaX;
    pixelDyRaw = (float)g_pmw3901_raw.deltaY;
    pixelDx = pixelDxRaw;
    pixelDy = pixelDyRaw;
    height = (float)g_tof_fused_height_mm / 1000.0f;

    if (Pos_Est_Absf(pixelDx) > POS_EST_FLOW_PIXEL_LIMIT)
    {
        pixelDx = 0.0f;
    }
    if (Pos_Est_Absf(pixelDy) > POS_EST_FLOW_PIXEL_LIMIT)
    {
        pixelDy = 0.0f;
    }

    gyroRollDps = 0.0f;
    gyroPitchDps = 0.0f;
    if (gyro_count != 0U)
    {
        gyroRollDps = gx_sum / (float)gyro_count;
        gyroPitchDps = gy_sum / (float)gyro_count;
        gx_sum = 0.0f;
        gy_sum = 0.0f;
        gyro_count = 0U;
    }

    gyroRollDps = Pos_Est_Clampf(g_imudata_250hz.gyrox, -POS_EST_GYRO_LIMIT_DPS, POS_EST_GYRO_LIMIT_DPS);
    gyroPitchDps = Pos_Est_Clampf(g_imudata_250hz.gyroy, -POS_EST_GYRO_LIMIT_DPS, POS_EST_GYRO_LIMIT_DPS);

    // 给光流数据 pixelDx和pixelDy进行低通滤波
    pixelDxLpf += (pixelDx - pixelDxLpf) * 0.4f;
    pixelDyLpf += (pixelDy - pixelDyLpf) * 0.4f;
    pixelDx = pixelDxLpf;
    pixelDy = pixelDyLpf;

    // 给角速度数据 维护一个长度为5的陀螺历史窗口，计算窗口内陀螺的平均值作为补偿，并且更新窗口
    gyroRollSum += gyroRollDps - gyroRollHist[gyroHistIndex];
    gyroPitchSum += gyroPitchDps - gyroPitchHist[gyroHistIndex];
    gyroRollHist[gyroHistIndex] = gyroRollDps;
    gyroPitchHist[gyroHistIndex] = gyroPitchDps;
    gyroHistIndex++;
    if (gyroHistIndex >= 5U)
    {
        gyroHistIndex = 0U;
    }
    gyroRollDps = gyroRollSum * 0.2f;
    gyroPitchDps = gyroPitchSum * 0.2f;

    coeff = RESOLUTION * (height - POS_EST_FLOW_HEIGHT_OFFSET_M);
    if (height < 0.2f)
    {
        coeff = 0.0f;
    }

    opFlow.deltaPos[0] = (pixelDx + 0.1719264517f * gyroRollDps) * coeff;
    opFlow.deltaPos[1] = (pixelDy - 0.1664168828f * gyroPitchDps) * coeff;

    opFlow.deltaVel[0] = opFlow.deltaPos[0] / POS_EST_100HZ_DT;
    opFlow.deltaVel[1] = opFlow.deltaPos[1] / POS_EST_100HZ_DT;

    // 给 deltaVel[X] 一阶卡尔曼滤波，过程噪声 Q=0.05，测量噪声 R=0.3，初始估计误差 P=1.0
    {
        static float velKalmanP[2] = {1.0f, 1.0f};
        const float velKalmanQ = 0.05f;
        const float velKalmanR = 0.3f;
        float velKalmanK;

        velKalmanP[0] += velKalmanQ;
        velKalmanK = velKalmanP[0] / (velKalmanP[0] + velKalmanR);
        opFlow.velLpf[0] += velKalmanK * (opFlow.deltaVel[0] - opFlow.velLpf[0]);
        velKalmanP[0] = (1.0f - velKalmanK) * velKalmanP[0];

        velKalmanP[1] += velKalmanQ;
        velKalmanK = velKalmanP[1] / (velKalmanP[1] + velKalmanR);
        opFlow.velLpf[1] += velKalmanK * (opFlow.deltaVel[1] - opFlow.velLpf[1]);
        velKalmanP[1] = (1.0f - velKalmanK) * velKalmanP[1];
    }

    opFlow.velLpf[0] = Pos_Est_Clampf(opFlow.velLpf[0], -POS_EST_VEL_LIMIT, POS_EST_VEL_LIMIT);
    opFlow.velLpf[1] = Pos_Est_Clampf(opFlow.velLpf[1], -POS_EST_VEL_LIMIT, POS_EST_VEL_LIMIT);

    opFlow.posSum[0] += opFlow.deltaPos[0];
    opFlow.posSum[1] += opFlow.deltaPos[1];

    // 根据当前的 squal 和之前的状态更新 flow_quality_ok
    // 只有当 squal 从低于 POS_EST_FLOW_SQUAL_LOW 上升到高于 POS_EST_FLOW_SQUAL_HIGH 时，flow_quality_ok 才会从 0 变为 1
    if (s_pos_est_xy_state.flow_quality_ok == 0U)
    {
        s_pos_est_xy_state.flow_quality_ok = (g_pmw3901_raw.squal >= POS_EST_FLOW_SQUAL_HIGH) ? 1U : 0U;
    }
    else
    {
        s_pos_est_xy_state.flow_quality_ok = (g_pmw3901_raw.squal >= POS_EST_FLOW_SQUAL_LOW) ? 1U : 0U;
    }

    // innov_x 和 innov_y 是光流速度与估计器速度的差值，单位 cm/s
    innov_x = opFlow.velLpf[POS_EST_AXIS_RIGHT] - estimator.vel[POS_EST_AXIS_RIGHT];
    innov_y = opFlow.velLpf[POS_EST_AXIS_FORWARD] - estimator.vel[POS_EST_AXIS_FORWARD];
    // innov_norm 是创新的欧几里得范数，单位 cm/s
    // 作用是如果 innov_norm 过大，说明光流速度与估计器速度差距过大，可能是光流数据异常，此时应该丢弃光流观测
    innov_norm = sqrtf(innov_x * innov_x + innov_y * innov_y);

    // 将创新保存到状态中，供调试输出使用
    s_pos_est_xy_state.innovation_cmps[POS_EST_AXIS_RIGHT] = innov_x;
    s_pos_est_xy_state.innovation_cmps[POS_EST_AXIS_FORWARD] = innov_y;

    flow_valid = 1U;
    if (s_pos_est_xy_state.flow_quality_ok == 0U)
    {
        flow_valid = 0U;
    }
    if ((height < POS_EST_FLOW_MIN_HEIGHT_M) || (height > POS_EST_FLOW_MAX_HEIGHT_M))
    {
        flow_valid = 0U;
    }
    if ((Pos_Est_Absf(pixelDxRaw) > POS_EST_FLOW_PIXEL_LIMIT) || (Pos_Est_Absf(pixelDyRaw) > POS_EST_FLOW_PIXEL_LIMIT))
    {
        flow_valid = 0U;
    }

    // 如果创新过大，说明光流观测与估计器预测差距过大，可能是光流数据异常，此时也应该丢弃光流观测
    if (innov_norm > POS_EST_FLOW_INNOV_GATE_CMPS)
    {
        flow_valid = 0U;
    }

    opFlow.isOpFlowOk = flow_valid;
    opFlow.isDataValid = flow_valid;

    if (flow_valid != 0U)
    {
        if (s_pos_est_xy_state.flow_pos_initialized == 0U) // 光流积分位置是否已经锚定
        {
            Pos_Est_ResetFlowAnchor();
        }

        s_pos_est_xy_state.flow_pos_cm[POS_EST_AXIS_RIGHT] += opFlow.velLpf[POS_EST_AXIS_RIGHT] * flow_dt_s;     // 位置+=速度*时间
        s_pos_est_xy_state.flow_pos_cm[POS_EST_AXIS_FORWARD] += opFlow.velLpf[POS_EST_AXIS_FORWARD] * flow_dt_s; // 位置+=速度*时间
        s_pos_est_xy_state.last_flow_dt_s = flow_dt_s;                                                           // 上一次有效光流时间间隔
        s_pos_est_xy_state.last_flow_update_tick = tick_1000us_cnt;                                              // 光流有效的最后更新时间(时间戳)
        s_pos_est_xy_state.flow_valid = 1U;                                                                      // 光流是否可用于融合
        s_pos_est_xy_state.recover_gain += flow_dt_s / POS_EST_FUSION_RECOVER_RAMP_S;                            // 光流如果数据有效，恢复增益逐渐拉起，拉起时间为 POS_EST_FUSION_RECOVER_RAMP_S 秒
        s_pos_est_xy_state.recover_gain = Pos_Est_Clampf(s_pos_est_xy_state.recover_gain, 0.0f, 1.0f);

        if (s_pos_est_xy_state.recover_gain < 0.999f)
        {
            s_pos_est_xy_state.fusion_mode = POS_EST_FUSION_MODE_RECOVERING; // 光流数据有效，但恢复增益还没有完全拉起，融合模式为恢复中
        }
        else
        {
            s_pos_est_xy_state.fusion_mode = POS_EST_FUSION_MODE_FLOW_HOLD; // 光流数据有效，恢复增益已经完全拉起，融合模式为光流保持
        }
    }
    else
    {
        s_pos_est_xy_state.flow_valid = 0U;
        s_pos_est_xy_state.recover_gain = 0.0f;                              // 光流数据无效，恢复增益直接给0
        s_pos_est_xy_state.fusion_mode = POS_EST_FUSION_MODE_DEAD_RECKONING; // 光流数据无效，融合模式切换到纯死推
    }

    Pos_Est_UpdateXYOutput();
}
