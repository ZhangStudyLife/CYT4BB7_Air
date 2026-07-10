// #include "Pos_Est.h"
// #include "FlightController/fc_params.h"

// extern volatile uint32 tick_1000us_cnt;

// /* X 轴加速度一阶低通输出，单位 cm/s^2，飞机往前加速为正，往后加速为负 */
// float acc_x_lp = 0.0f;
// /* Y 轴加速度一阶低通输出，单位 cm/s^2，飞机往右加速为正，往左加速为负 */
// float acc_y_lp = 0.0f;

// float vel_x_pred = 0.0f;
// float vel_y_pred = 0.0f;

// /* 光流解算得到的 X 轴速度，单位 cm/s，往左飞为正，往右飞为负 */
// float opflow_vel_x = 0.0f;
// /* 光流解算得到的 Y 轴速度，单位 cm/s，往前飞为正，往后飞为负 */
// float opflow_vel_y = 0.0f;

// /* 位置估计的 X 轴速度，单位 cm/s */
// float Pos_Est_vel_x = 0.0f;
// /* 位置估计的 Y 轴速度，单位 cm/s */
// float Pos_Est_vel_y = 0.0f;

// /*
//  * 函数名: Pos_Est_Init
//  * 功能: 初始化光流位置估计模块和相关滤波器
//  * 输入参数: 无
//  * 返回值: 无
//  */
// void Pos_Est_Init(void)
// {
//     PMW3901_Init();
//     FlowGyroDecoupler_Init();
//     acc_x_lp = 0.0f;
//     acc_y_lp = 0.0f;
//     opflow_vel_x = 0.0f;
//     opflow_vel_y = 0.0f;
//     Pos_Est_vel_x = 0.0f;
//     Pos_Est_vel_y = 0.0f;
//     vel_x_pred = 0.0f;
//     vel_y_pred = 0.0f;
// }

// /*
//  * 函数名: Pos_Est_Reinit
//  * 功能: 重置光流和滤波器内部状态
//  * 输入参数: 无
//  * 返回值: 无
//  */
// void Pos_Est_Reinit(void)
// {
//     PMW3901_ReInit();
//     FlowGyroDecoupler_Reinit();

//     acc_x_lp = 0.0f;
//     acc_y_lp = 0.0f;
//     opflow_vel_x = 0.0f;
//     opflow_vel_y = 0.0f;
//     Pos_Est_vel_x = 0.0f;
//     Pos_Est_vel_y = 0.0f;
//     vel_x_pred = 0.0f;
//     vel_y_pred = 0.0f;
// }

// /*
//  * 函数名: Pos_Est_Update_1000HZ
//  * 功能: 推送光流去旋转解耦所需陀螺数据，并更新水平加速度低通输出
//  * 输入参数: 无
//  * 返回值: 无
//  *
//  * 加速度数据路径说明：
//  *   使用 g_imufilter_1000hz（已经过陷波161Hz/320Hz + 12Hz低通滤波）作为输入，
//  *   通过 AccelCalibration_RotateImuToBody 旋转到机体系后，利用旋转矩阵投影到水平系。
//  *   重力分量在旋转投影中自动消除，无需额外去重力步骤（数学推导验证）。
//  *   相比原 AccelCalibration_GetLevelAccelMps2 路径（原始未滤波acc），
//  *   可消除飞行中电机振动导致的 ±100~300 cm/s² 污染。
//  */
// void Pos_Est_Update_1000HZ(void)
// {
//     float acc_sensor[3];
//     float acc_body[3];
//     float sp;
//     float cp;
//     float sr;
//     float cr;

//     FlowGyroDecoupler_Push1000Hz(tick_1000us_cnt, g_imudata_250hz.gyrox, g_imudata_250hz.gyroy);

//     /* 取陷波+低通滤波后的传感器系加速度（单位 g，已校准零偏/尺度） */
//     acc_sensor[0] = g_imufilter_1000hz.accx;
//     acc_sensor[1] = g_imufilter_1000hz.accy;
//     acc_sensor[2] = g_imufilter_1000hz.accz;
//     // AccelCalibration_RotateImuToBody(acc_sensor, acc_body);
//     // 不用上面那个函数脱裤子放屁了，直接当传感器系就是机体系，毕竟没有安装误差
//     acc_body[0] = acc_sensor[0];
//     acc_body[1] = acc_sensor[1];
//     acc_body[2] = acc_sensor[2];

//     sp = g_euler.sin_pitch;
//     cp = g_euler.cos_pitch;
//     sr = g_euler.sin_roll;
//     cr = g_euler.cos_roll;

//     /* 旋转到水平系（yaw=0近似），重力自动消除。
//      * level_x = cp*body_x + sp*sr*body_y + sp*cr*body_z  (前进为正)
//      * level_y = cr*body_y - sr*body_z                     (向右为正)
//      * 单位 g → cm/s^2 */
//     acc_x_lp = (cp * acc_body[0] + sp * sr * acc_body[1] + sp * cr * acc_body[2]) * 9.80665f * 100.0f;
//     acc_y_lp = (cr * acc_body[1] - sr * acc_body[2]) * 9.80665f * 100.0f;

//     // 需要注意的是acc_y_lp右正左负，acc_x_lp前正后负
//     vel_x_pred -= acc_y_lp * 0.001f;
//     vel_y_pred += acc_x_lp * 0.001f;

// }

// /*
//  * 函数名: Pos_Est_Update_50HZ
//  * 功能: 更新 PMW3901 光流速度，并执行速度融合与位置积分
//  * 输入参数: 无
//  * 返回值: 无
//  */
// void Pos_Est_Update_50HZ(void)
// {
//     float dec_x;
//     float dec_y;
//     float height;
//     float coeff;

//     PMW3901_Update_50HZ();
//     FlowGyroDecoupler_Update50Hz(tick_1000us_cnt, g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY);
//     dec_x = FlowGyroDecoupler_GetDecX();
//     dec_y = FlowGyroDecoupler_GetDecY();
//     height = g_tof_fused_height_mm * 0.001f;

//     /* 1m 高度下 1 像素约对应 0.2131946 cm 位移，同时对过低高度做保护 */
//     coeff = 0.2131946f * (height - 0.05f);
//     if (height < 0.2f)
//     {
//         coeff = 0.0f;dec_x = 0.0f;dec_y = 0.0f;
//     }

//     /* 计算光流速度，此刻加入高度补偿，单位 cm/s */
//     opflow_vel_x = dec_x * coeff * 50.0f;
//     opflow_vel_y = dec_y * coeff * 50.0f;

//     /* squal 有效性门控：光流质量过低时不参与融合 */
//     uint8 opflow_valid = (g_pmw3901_raw.squal >= 25U) && (height >= 0.2f);
//     float k_use = opflow_valid ? g_fc_params.pos_est_k_flow : 0.0f;

//     /* 用光流测量对惯性预测做互补校正 */
//     Pos_Est_vel_x = vel_x_pred + k_use * (opflow_vel_x - vel_x_pred);
//     Pos_Est_vel_y = vel_y_pred + k_use * (opflow_vel_y - vel_y_pred);

//     /* 光流失效时对融合速度施加摩擦衰减，防止加速度偏置积分漂移 */
//     if (!opflow_valid)
//     {
//         Pos_Est_vel_x *= 0.96f;
//         Pos_Est_vel_y *= 0.96f;
//     }

//     vel_x_pred = Pos_Est_vel_x;
//     vel_y_pred = Pos_Est_vel_y;

//                         acc_x_lp, acc_y_lp,
//                         opflow_vel_x, opflow_vel_y,
//                         Pos_Est_vel_x, Pos_Est_vel_y,
//                         g_pmw3901_raw.squal, height, lc302_data.flow_x_integral, lc302_data.flow_y_integral);

// }

#include "Pos_Est.h"
#include "FlowGyroDecoupler_LC302.h"
#include "../Attitude/Accel_Calibration.h"
#include "../Attitude/IMU_Filtter.h"
#include "../Height_Est/Height_Est.h"
// #include "HW_Drivers/PMW3901/PMW3901.h"
#include "HW_Drivers/LC302/LC302.h"
#include "FlightController/fc_start_crsf.h"
#include <math.h>
#include <string.h>

extern volatile uint32 tick_1000us_cnt;

/* 1000Hz 水平加速度相位补偿低通 alpha，截止频率 fc≈5.00Hz */
#define POS_EST_RAW_ACC_LPF_ALPHA (0.06089863f)
/* 前后轴水平加速度限幅，单位 cm/s^2 */
#define POS_EST_ACC_FWD_LIMIT_CMSS (600.0f)
/* 左右轴水平加速度限幅，单位 cm/s^2 */
#define POS_EST_ACC_RIGHT_LIMIT_CMSS (600.0f)
#define POS_EST_DEG_TO_RAD (0.017453292519943295f)
#define POS_EST_STATIC_GYRO_MAX_DPS (3.0f)
#define POS_EST_STATIC_TILT_MAX_DEG (6.0f)
#define POS_EST_STATIC_HEIGHT_MIN_MM (70.0f)
#define POS_EST_STATIC_HEIGHT_MAX_MM (200.0f)
#define POS_EST_STATIC_LOCK_SAMPLES (300U)
#define POS_EST_ACC_BIAS_ALPHA (0.001f)
#define POS_EST_ACC_BIAS_LIMIT_CMSS (120.0f)
/* V2固定延迟历史长度，覆盖96ms的1000Hz样本。 */
#define POS_EST_V2_HISTORY_LEN (96U)
/* V2 LC302时钟相位最小值窗口长度，覆盖约1.3秒。 */
#define POS_EST_V2_CLOCK_WINDOW_LEN (64U)
/* LC302光流帧固定周期，单位秒。 */
#define POS_EST_V2_FLOW_DT_S (0.0208f)
/* LC302传感器时钟到有效光流测量的延迟，单位毫秒。 */
#define POS_EST_V2_FLOW_DELAY_MS (36.0f)
/* LC302解耦量转换为视线角速度的比例。 */
#define POS_EST_V2_FLOW_TO_RADPS (0.00480769231f)
/* V2允许使用光流时的最小姿态余弦。 */
#define POS_EST_V2_TILT_COS_MIN (0.71f)
/* V2允许的最大光流角速度，单位rad/s。 */
#define POS_EST_V2_FLOW_RATE_MAX_RADPS (2.5f)
/* V2加速度过程噪声标准差，单位cm/s^2/sqrt(Hz)。 */
#define POS_EST_V2_SIGMA_ACC_CMSS (30.0f)
/* V2加速度偏置随机游走标准差，单位cm/s^3/sqrt(Hz)。 */
#define POS_EST_V2_SIGMA_BIAS_CMSSS (0.2f)
/* V2光流测量噪声标准差，单位rad/s。 */
#define POS_EST_V2_SIGMA_FLOW_RADPS (0.18f)
/* V2正常更新的二维NIS软门限。 */
#define POS_EST_V2_NIS_NORMAL (11.83f)
/* V2重捕获更新的二维NIS软门限。 */
#define POS_EST_V2_NIS_REACQUIRE (25.0f)
/* V2拒绝光流更新的二维NIS硬门限。 */
#define POS_EST_V2_NIS_MAX (100.0f)
/* V2通过连续性检查后允许真重捕获使用的NIS上限。 */
#define POS_EST_V2_NIS_REACQUIRE_MAX (100.0f)
/* V2正常更新允许的当前速度最大修正量，单位cm/s。 */
#define POS_EST_V2_CORRECTION_NORMAL_CMPS (25.0f)
/* V2重捕获允许的当前速度最大修正量，单位cm/s。 */
#define POS_EST_V2_CORRECTION_REACQUIRE_CMPS (35.0f)
/* V2真重捕获取得三帧一致前的探测修正上限，单位cm/s。 */
#define POS_EST_V2_CORRECTION_PROBE_CMPS (10.0f)
/* V2进入重捕获状态的无有效光流时间，单位毫秒。 */
#define POS_EST_V2_REACQUIRE_MS (200U)
/* V2进入降级状态的无有效光流时间，单位毫秒。 */
#define POS_EST_V2_DEGRADED_MS (100U)
/* V2真重捕获要求的连续一致光流帧数。 */
#define POS_EST_V2_REACQUIRE_GOOD_FRAMES (3U)
/* V2相邻重捕获光流速度允许的最大差，单位cm/s。 */
#define POS_EST_V2_REACQUIRE_CONSISTENCY_CMPS (100.0f)
/* V2输出预测器修正注入时间常数，单位秒。 */
#define POS_EST_V2_OUTPUT_TAU_S (0.080f)
/* V2输出预测器最大修正注入加速度，单位cm/s^2。 */
#define POS_EST_V2_OUTPUT_CORRECTION_ACC_CMSS (500.0f)
/* V2输出预测器允许积累的最大待注入修正，单位cm/s。 */
#define POS_EST_V2_OUTPUT_PENDING_MAX_CMPS (200.0f)
/* V2输出预测器超过该待注入修正时标记不可靠，单位cm/s。 */
#define POS_EST_V2_OUTPUT_PENDING_UNRELIABLE_CMPS (100.0f)
/* V2加速度偏置状态限幅，单位cm/s^2。 */
#define POS_EST_V2_BIAS_LIMIT_CMSS (100.0f)
/* V2光流有效高度下限，单位米。 */
#define POS_EST_V2_HEIGHT_MIN_M (0.20f)
/* V2光流有效高度上限，单位米。 */
#define POS_EST_V2_HEIGHT_MAX_M (1.40f)
/* V2光流曝光半窗口，单位毫秒。 */
#define POS_EST_V2_EXPOSURE_HALF_MS (10.4f)
/* V2允许进行历史光流更新的最大测量年龄，单位毫秒。 */
#define POS_EST_V2_HISTORY_MAX_AGE_MS (75.0f)
/* V2可连续推算的LC302最大序号跳变量。 */
#define POS_EST_V2_SEQ_JUMP_MAX (10U)
/* V2新光流帧状态位。 */
#define POS_EST_V2_STATUS_NEW_FLOW (1U << 0U)
/* V2历史测量时刻已找到状态位。 */
#define POS_EST_V2_STATUS_HISTORY_READY (1U << 1U)
/* V2光流原始有效状态位。 */
#define POS_EST_V2_STATUS_FLOW_VALID (1U << 2U)
/* V2延迟时刻ToF有效状态位。 */
#define POS_EST_V2_STATUS_TOF_VALID (1U << 3U)
/* V2光流几何条件有效状态位。 */
#define POS_EST_V2_STATUS_GEOMETRY_VALID (1U << 4U)
/* V2光流曝光区间存在冲击状态位。 */
#define POS_EST_V2_STATUS_EXPOSURE_SHOCK (1U << 5U)
/* V2本帧光流更新已接受状态位。 */
#define POS_EST_V2_STATUS_ACCEPTED (1U << 6U)
/* V2本帧光流更新被拒绝状态位。 */
#define POS_EST_V2_STATUS_REJECTED (1U << 7U)
/* V2当前处于光流重捕获状态位。 */
#define POS_EST_V2_STATUS_REACQUIRE (1U << 8U)
/* V2光流更新时间超过100ms。 */
#define POS_EST_V2_STATUS_DEGRADED (1U << 9U)
/* V2光流更新时间超过200ms或尚未接受光流。 */
#define POS_EST_V2_STATUS_UNRELIABLE (1U << 10U)
/* V2本帧冻结bias更新。 */
#define POS_EST_V2_STATUS_BIAS_FROZEN (1U << 11U)
/* V2输出修正注入触发限速。 */
#define POS_EST_V2_STATUS_OUTPUT_LIMITED (1U << 12U)
/* V2真重捕获已取得连续一致光流。 */
#define POS_EST_V2_STATUS_REACQUIRE_CONSISTENT (1U << 13U)
/* V2待注入修正触发硬限幅。 */
#define POS_EST_V2_STATUS_PENDING_CLAMPED (1U << 14U)

/* V2单个1000Hz历史样本，保存固定延迟更新和回放所需的全部数据。 */
typedef struct
{
    float state[4];
    float covariance[4][4];
    float accel_left_cmss;
    float accel_forward_cmss;
    float yaw_rate_rps;
    float height_m;
    float vertical_up_cmps;
    float sin_pitch;
    float cos_pitch;
    float sin_roll;
    float cos_roll;
    float gyro_norm_dps;
    uint32_t time_ms;
    uint8_t tof_valid;
    uint8_t shock;
    uint8_t static_locked;
    uint8_t reserved;
} Pos_Est_V2_History_t;
/* 光流解算得到的 X 轴速度，单位 cm/s，往左飞为正，往右飞为负 */
float opflow_vel_x = 0.0f;
/* 光流解算得到的 Y 轴速度，单位 cm/s，往前飞为正，往后飞为负 */
float opflow_vel_y = 0.0f;
/* 保留旧调试接口，当前与最新有效光流速度测量一致。 */
float opflow_vel_x_lpf = 0.0f;
/* 保留旧调试接口，当前与最新有效光流速度测量一致。 */
float opflow_vel_y_lpf = 0.0f;
/* 位置估计的 X 轴速度，单位 cm/s */
float Pos_Est_vel_x = 0.0f;
/* 位置估计的 Y 轴速度，单位 cm/s */
float Pos_Est_vel_y = 0.0f;
float Pos_Est_vel_x_level = 0.0f;
float Pos_Est_vel_y_level = 0.0f;
static float s_raw_acc_lp_x = 0.0f;
static float s_raw_acc_lp_y = 0.0f;
static uint16_t s_static_sample_count = 0U;
static uint32_t s_pos_est_last_update_ms = 0U;
static uint8_t s_pos_est_update_time_ready = 0U;

/* X 轴加速度 单位 cm/s^2，飞机往前加速为正，往后加速为负 */
float acc_x_temp = 0.0f;
/* Y 轴加速度 单位 cm/s^2，飞机往右加速为正，往左加速为负 */
float acc_y_temp = 0.0f;
float acc_x_lp = 0.0f;
float acc_y_lp = 0.0f;
float Pos_Est_acc_bias_x_cmss = 0.0f;
float Pos_Est_acc_bias_y_cmss = 0.0f;

/* V2固定延迟状态、协方差和IMU输入历史。 */
static Pos_Est_V2_History_t s_pos_est_v2_history[POS_EST_V2_HISTORY_LEN];
/* V2历史缓冲当前最新样本下标。 */
static uint16_t s_pos_est_v2_history_head = 0U;
/* V2历史缓冲当前有效样本数量。 */
static uint16_t s_pos_est_v2_history_count = 0U;
/* V2上一次消费的LC302串口帧序号。 */
static uint32_t s_pos_est_v2_last_flow_seq = 0U;
/* V2最近一次接受光流更新的MCU时刻，单位毫秒。 */
static uint32_t s_pos_est_v2_last_accepted_ms = 0U;
/* V2在线重建的LC302传感器时刻，单位毫秒。 */
static float s_pos_est_v2_sensor_time_ms = 0.0f;
/* V2最近LC302帧的轮询相位候选值，单位毫秒。 */
static float s_pos_est_v2_clock_phase[POS_EST_V2_CLOCK_WINDOW_LEN];
/* V2 LC302时钟相位窗口最新下标。 */
static uint8_t s_pos_est_v2_clock_phase_head = 0U;
/* V2 LC302时钟相位窗口有效数量。 */
static uint8_t s_pos_est_v2_clock_phase_count = 0U;
/* V2当前LC302时钟分段的起始帧序号。 */
static uint32_t s_pos_est_v2_clock_base_seq = 0U;
/* V2当前左向内部KF速度，单位cm/s。 */
static float s_pos_est_v2_vel_x = 0.0f;
/* V2当前前向内部KF速度，单位cm/s。 */
static float s_pos_est_v2_vel_y = 0.0f;
/* V2当前左向残余加速度偏置，单位cm/s^2。 */
static float s_pos_est_v2_bias_x = 0.0f;
/* V2当前前向残余加速度偏置，单位cm/s^2。 */
static float s_pos_est_v2_bias_y = 0.0f;
/* V2最近新光流帧的左向速度创新，单位cm/s。 */
static float s_pos_est_v2_innovation_x = 0.0f;
/* V2最近新光流帧的前向速度创新，单位cm/s。 */
static float s_pos_est_v2_innovation_y = 0.0f;
/* V2最近光流更新传播到当前时刻后的左向修正量，单位cm/s。 */
static float s_pos_est_v2_correction_x = 0.0f;
/* V2最近光流更新传播到当前时刻后的前向修正量，单位cm/s。 */
static float s_pos_est_v2_correction_y = 0.0f;
/* V2最近新光流帧的二维NIS。 */
static float s_pos_est_v2_nis = 0.0f;
/* V2最近新光流帧选中的历史测量年龄，单位毫秒。 */
static float s_pos_est_v2_measurement_age_ms = 0.0f;
/* V2最近一次50Hz处理结果的压缩状态位。 */
static uint32_t s_pos_est_v2_status = 0U;
/* V2是否已经建立LC302传感器时钟。 */
static uint8_t s_pos_est_v2_sensor_clock_ready = 0U;
/* V2是否至少接受过一次光流更新。 */
static uint8_t s_pos_est_v2_has_accepted = 0U;
static uint8_t s_pos_est_v2_reacquire_active = 0U;
static uint8_t s_pos_est_v2_reacquire_good_count = 0U;
static uint8_t s_pos_est_v2_reacquire_stable_count = 0U;
static float s_pos_est_v2_reacquire_prev_x = 0.0f;
static float s_pos_est_v2_reacquire_prev_y = 0.0f;
static float s_pos_est_v2_output_vel_x = 0.0f;
static float s_pos_est_v2_output_vel_y = 0.0f;
static float s_pos_est_v2_pending_x = 0.0f;
static float s_pos_est_v2_pending_y = 0.0f;
static float s_pos_est_v2_last_accept_age_ms = 0.0f;
static float s_pos_est_v2_sigma_flow_radps = POS_EST_V2_SIGMA_FLOW_RADPS;

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
    if (g_imu_shock_flag != 0U)
    {
        return 0U;
    }
    return 1U;
}

/*
 * 函数名: Pos_Est_V2_Propagate
 * 功能: 使用当前历史样本的IMU输入，将V2状态和协方差从上一样本传播到当前样本
 * 输入参数: current-待写入的当前历史样本；previous-时间上相邻的上一历史样本
 * 返回值: 无
 */
static void Pos_Est_V2_Propagate(Pos_Est_V2_History_t *current,
                                 const Pos_Est_V2_History_t *previous)
{
    float dt;
    float gap;
    float dpsi;
    float yaw_cos;
    float yaw_sin;
    float old_left;
    float old_forward;
    float qa2 = POS_EST_V2_SIGMA_ACC_CMSS * POS_EST_V2_SIGMA_ACC_CMSS;
    float qb2 = POS_EST_V2_SIGMA_BIAS_CMSSS * POS_EST_V2_SIGMA_BIAS_CMSSS;
    float a[4][4];
    float temp[4][4];
    float covariance[4][4];
    uint8_t i;
    uint8_t j;
    uint8_t k;

    dt = (float)(current->time_ms - previous->time_ms) * 0.001f;
    if (dt <= 0.0f)
    {
        memcpy(current->state, previous->state, sizeof(current->state));
        memcpy(current->covariance, previous->covariance, sizeof(current->covariance));
        return;
    }

    old_left = previous->state[0];
    old_forward = previous->state[1];
    current->state[2] = previous->state[2];
    current->state[3] = previous->state[3];

    if (dt > 0.050f)
    {
        gap = (dt < 5.0f) ? dt : 5.0f;
        dpsi = current->yaw_rate_rps * ((gap < 1.0f) ? gap : 1.0f);
        yaw_cos = cosf(dpsi);
        yaw_sin = sinf(dpsi);
        current->state[0] = yaw_cos * old_left + yaw_sin * old_forward;
        current->state[1] = -yaw_sin * old_left + yaw_cos * old_forward;

        memset(a, 0, sizeof(a));
        a[0][0] = yaw_cos;
        a[0][1] = yaw_sin;
        a[1][0] = -yaw_sin;
        a[1][1] = yaw_cos;
        a[2][2] = 1.0f;
        a[3][3] = 1.0f;
        for (i = 0U; i < 4U; i++)
        {
            for (j = 0U; j < 4U; j++)
            {
                temp[i][j] = 0.0f;
                for (k = 0U; k < 4U; k++)
                {
                    temp[i][j] += a[i][k] * previous->covariance[k][j];
                }
            }
        }
        for (i = 0U; i < 4U; i++)
        {
            for (j = 0U; j < 4U; j++)
            {
                covariance[i][j] = 0.0f;
                for (k = 0U; k < 4U; k++)
                {
                    covariance[i][j] += temp[i][k] * a[j][k];
                }
            }
        }
        for (i = 0U; i < 4U; i++)
        {
            for (j = 0U; j < 4U; j++)
            {
                current->covariance[i][j] =
                    0.5f * (covariance[i][j] + covariance[j][i]);
            }
        }
        current->covariance[0][0] += qa2 * gap + qb2 * gap * gap * gap / 3.0f;
        current->covariance[1][1] += qa2 * gap + qb2 * gap * gap * gap / 3.0f;
        current->covariance[0][2] -= qb2 * gap * gap * 0.5f;
        current->covariance[2][0] = current->covariance[0][2];
        current->covariance[1][3] -= qb2 * gap * gap * 0.5f;
        current->covariance[3][1] = current->covariance[1][3];
        current->covariance[2][2] += qb2 * gap;
        current->covariance[3][3] += qb2 * gap;
    }
    else
    {
        dpsi = current->yaw_rate_rps * dt;
        yaw_cos = cosf(dpsi);
        yaw_sin = sinf(dpsi);
        current->state[0] = yaw_cos * old_left + yaw_sin * old_forward +
                            (current->accel_left_cmss - previous->state[2]) * dt;
        current->state[1] = -yaw_sin * old_left + yaw_cos * old_forward +
                            (current->accel_forward_cmss - previous->state[3]) * dt;

        memset(a, 0, sizeof(a));
        a[0][0] = yaw_cos;
        a[0][1] = yaw_sin;
        a[1][0] = -yaw_sin;
        a[1][1] = yaw_cos;
        a[0][2] = -dt;
        a[1][3] = -dt;
        a[2][2] = 1.0f;
        a[3][3] = 1.0f;

        for (i = 0U; i < 4U; i++)
        {
            for (j = 0U; j < 4U; j++)
            {
                temp[i][j] = 0.0f;
                for (k = 0U; k < 4U; k++)
                {
                    temp[i][j] += a[i][k] * previous->covariance[k][j];
                }
            }
        }
        for (i = 0U; i < 4U; i++)
        {
            for (j = 0U; j < 4U; j++)
            {
                covariance[i][j] = 0.0f;
                for (k = 0U; k < 4U; k++)
                {
                    covariance[i][j] += temp[i][k] * a[j][k];
                }
            }
        }

        covariance[0][0] += qa2 * dt + qb2 * dt * dt * dt / 3.0f;
        covariance[1][1] += qa2 * dt + qb2 * dt * dt * dt / 3.0f;
        covariance[0][2] -= qb2 * dt * dt * 0.5f;
        covariance[2][0] -= qb2 * dt * dt * 0.5f;
        covariance[1][3] -= qb2 * dt * dt * 0.5f;
        covariance[3][1] -= qb2 * dt * dt * 0.5f;
        covariance[2][2] += qb2 * dt;
        covariance[3][3] += qb2 * dt;

        for (i = 0U; i < 4U; i++)
        {
            for (j = 0U; j < 4U; j++)
            {
                current->covariance[i][j] = 0.5f * (covariance[i][j] + covariance[j][i]);
            }
        }
    }

    if ((current->static_locked != 0U) && (current->height_m < POS_EST_V2_HEIGHT_MIN_M))
    {
        float dt_static = (dt < 0.010f) ? dt : 0.010f;
        float alpha = dt_static / (5.0f + dt_static);
        float bias_left_cov = current->covariance[2][2];
        float bias_forward_cov = current->covariance[3][3];

        current->state[2] += alpha * (current->accel_left_cmss - current->state[2]);
        current->state[3] += alpha * (current->accel_forward_cmss - current->state[3]);
        current->state[0] = 0.0f;
        current->state[1] = 0.0f;
        for (i = 0U; i < 2U; i++)
        {
            for (j = 0U; j < 4U; j++)
            {
                current->covariance[i][j] = 0.0f;
                current->covariance[j][i] = 0.0f;
            }
        }
        current->covariance[0][0] = 0.25f;
        current->covariance[1][1] = 0.25f;
        current->covariance[2][2] = (bias_left_cov < 4.0f) ? bias_left_cov : 4.0f;
        current->covariance[3][3] = (bias_forward_cov < 4.0f) ? bias_forward_cov : 4.0f;
        current->covariance[2][3] = 0.0f;
        current->covariance[3][2] = 0.0f;
    }
}

/*
 * 函数名: Pos_Est_V2_Init
 * 功能: 初始化V2影子速度估计器的状态、协方差、固定延迟历史和诊断量
 * 输入参数: 无
 * 返回值: 无
 */
static void Pos_Est_V2_Init(void)
{
    memset(s_pos_est_v2_history, 0, sizeof(s_pos_est_v2_history));
    s_pos_est_v2_history_head = 0U;
    s_pos_est_v2_history_count = 0U;
    s_pos_est_v2_last_flow_seq = lc302_data_seq;
    s_pos_est_v2_last_accepted_ms = 0U;
    s_pos_est_v2_sensor_time_ms = 0.0f;
    memset(s_pos_est_v2_clock_phase, 0, sizeof(s_pos_est_v2_clock_phase));
    s_pos_est_v2_clock_phase_head = 0U;
    s_pos_est_v2_clock_phase_count = 0U;
    s_pos_est_v2_clock_base_seq = lc302_data_seq;
    s_pos_est_v2_vel_x = 0.0f;
    s_pos_est_v2_vel_y = 0.0f;
    s_pos_est_v2_bias_x = 0.0f;
    s_pos_est_v2_bias_y = 0.0f;
    s_pos_est_v2_innovation_x = 0.0f;
    s_pos_est_v2_innovation_y = 0.0f;
    s_pos_est_v2_correction_x = 0.0f;
    s_pos_est_v2_correction_y = 0.0f;
    s_pos_est_v2_nis = 0.0f;
    s_pos_est_v2_measurement_age_ms = 0.0f;
    s_pos_est_v2_status = 0U;
    s_pos_est_v2_sensor_clock_ready = 0U;
    s_pos_est_v2_has_accepted = 0U;
    s_pos_est_v2_reacquire_active = 0U;
    s_pos_est_v2_reacquire_good_count = 0U;
    s_pos_est_v2_reacquire_stable_count = 0U;
    s_pos_est_v2_reacquire_prev_x = 0.0f;
    s_pos_est_v2_reacquire_prev_y = 0.0f;
    s_pos_est_v2_output_vel_x = 0.0f;
    s_pos_est_v2_output_vel_y = 0.0f;
    s_pos_est_v2_pending_x = 0.0f;
    s_pos_est_v2_pending_y = 0.0f;
    s_pos_est_v2_last_accept_age_ms = 0.0f;
    s_pos_est_v2_sigma_flow_radps = POS_EST_V2_SIGMA_FLOW_RADPS;
}

/*
 * 函数名: Pos_Est_V2_Update_1000HZ
 * 功能: 以1000Hz传播V2机体系速度和协方差，并保存固定延迟回放历史
 * 输入参数: 无，使用当前IMU、姿态、高度和旧加速度预处理结果
 * 返回值: 无
 */
static void Pos_Est_V2_Update_1000HZ(void)
{
    Pos_Est_V2_History_t *current;
    Pos_Est_V2_History_t *previous = NULL;
    uint32_t elapsed_ms = 0U;
    uint16_t current_index;
    uint8_t time_gap = 0U;
    float yaw_cos_pitch;
    float dt;
    float dpsi;
    float yaw_cos;
    float yaw_sin;
    float pending_x;
    float pending_y;
    float injection_x;
    float injection_y;
    float alpha;
    float injection_norm;
    float injection_limit;
    float injection_scale;

    if (s_pos_est_v2_history_count == 0U)
    {
        current_index = 0U;
    }
    else
    {
        previous = &s_pos_est_v2_history[s_pos_est_v2_history_head];
        elapsed_ms = tick_1000us_cnt - previous->time_ms;
        if (elapsed_ms == 0U)
        {
            return;
        }
        time_gap = (elapsed_ms > 50U) ? 1U : 0U;
        current_index = (uint16_t)((s_pos_est_v2_history_head + 1U) % POS_EST_V2_HISTORY_LEN);
    }

    current = &s_pos_est_v2_history[current_index];
    memset(current, 0, sizeof(*current));
    current->time_ms = tick_1000us_cnt;
    current->accel_left_cmss = -acc_y_lp;
    current->accel_forward_cmss = acc_x_lp;
    current->height_m = g_tof_fused_height_mm * 0.001f;
    current->vertical_up_cmps = g_height_fused_vz_mps * 100.0f;
    current->sin_pitch = g_euler.sin_pitch;
    current->cos_pitch = g_euler.cos_pitch;
    current->sin_roll = g_euler.sin_roll;
    current->cos_roll = g_euler.cos_roll;
    current->gyro_norm_dps = sqrtf(g_imufilter_1000hz.gyrox * g_imufilter_1000hz.gyrox +
                                   g_imufilter_1000hz.gyroy * g_imufilter_1000hz.gyroy +
                                   g_imufilter_1000hz.gyroz * g_imufilter_1000hz.gyroz);
    yaw_cos_pitch = current->cos_pitch;
    if (yaw_cos_pitch < POS_EST_V2_TILT_COS_MIN)
    {
        yaw_cos_pitch = POS_EST_V2_TILT_COS_MIN;
    }
    current->yaw_rate_rps = (g_imufilter_1000hz.gyroy * current->sin_roll +
                             g_imufilter_1000hz.gyroz * current->cos_roll) /
                            yaw_cos_pitch * POS_EST_DEG_TO_RAD;
    current->tof_valid = (g_tof_fused_valid != 0U) ? 1U : 0U;
    current->shock = (g_imu_shock_flag != 0U) ? 1U : 0U;
    current->static_locked = ((Pos_Est_IsStaticForAccelBias() != 0U) &&
                              (s_static_sample_count >= POS_EST_STATIC_LOCK_SAMPLES))
                                 ? 1U
                                 : 0U;

    if (previous == NULL)
    {
        current->covariance[0][0] = 400.0f;
        current->covariance[1][1] = 400.0f;
        current->covariance[2][2] = 100.0f;
        current->covariance[3][3] = 100.0f;
        s_pos_est_v2_history_count = 1U;
    }
    else
    {
        Pos_Est_V2_Propagate(current, previous);
        if (time_gap != 0U)
        {
            s_pos_est_v2_history_count = 1U;
            s_pos_est_v2_sensor_clock_ready = 0U;
            s_pos_est_v2_clock_phase_count = 0U;
            s_pos_est_v2_reacquire_active = 1U;
            s_pos_est_v2_reacquire_good_count = 0U;
            s_pos_est_v2_reacquire_stable_count = 0U;
            s_pos_est_v2_reacquire_prev_x = 0.0f;
            s_pos_est_v2_reacquire_prev_y = 0.0f;
        }
        else if (s_pos_est_v2_history_count < POS_EST_V2_HISTORY_LEN)
        {
            s_pos_est_v2_history_count++;
        }
    }

    s_pos_est_v2_history_head = current_index;
    s_pos_est_v2_vel_x = current->state[0];
    s_pos_est_v2_vel_y = current->state[1];
    s_pos_est_v2_bias_x = current->state[2];
    s_pos_est_v2_bias_y = current->state[3];

    s_pos_est_v2_status &= ~(POS_EST_V2_STATUS_DEGRADED |
                             POS_EST_V2_STATUS_UNRELIABLE |
                             POS_EST_V2_STATUS_OUTPUT_LIMITED |
                             POS_EST_V2_STATUS_REACQUIRE);
    if ((previous == NULL) ||
        ((current->static_locked != 0U) && (current->height_m < POS_EST_V2_HEIGHT_MIN_M)))
    {
        s_pos_est_v2_pending_x = 0.0f;
        s_pos_est_v2_pending_y = 0.0f;
    }
    else
    {
        dt = (float)(current->time_ms - previous->time_ms) * 0.001f;
        if (dt > 0.050f)
        {
            dpsi = current->yaw_rate_rps * ((dt < 1.0f) ? dt : 1.0f);
            yaw_cos = cosf(dpsi);
            yaw_sin = sinf(dpsi);
            pending_x = yaw_cos * s_pos_est_v2_pending_x +
                        yaw_sin * s_pos_est_v2_pending_y;
            pending_y = -yaw_sin * s_pos_est_v2_pending_x +
                        yaw_cos * s_pos_est_v2_pending_y;
            s_pos_est_v2_pending_x = pending_x;
            s_pos_est_v2_pending_y = pending_y;
            s_pos_est_v2_status |= POS_EST_V2_STATUS_UNRELIABLE;
        }
        else if (dt > 0.0f)
        {
            dpsi = current->yaw_rate_rps * dt;
            yaw_cos = cosf(dpsi);
            yaw_sin = sinf(dpsi);
            pending_x = yaw_cos * s_pos_est_v2_pending_x +
                        yaw_sin * s_pos_est_v2_pending_y;
            pending_y = -yaw_sin * s_pos_est_v2_pending_x +
                        yaw_cos * s_pos_est_v2_pending_y;
            s_pos_est_v2_pending_x = pending_x;
            s_pos_est_v2_pending_y = pending_y;

            alpha = dt / (POS_EST_V2_OUTPUT_TAU_S + dt);
            injection_x = alpha * s_pos_est_v2_pending_x;
            injection_y = alpha * s_pos_est_v2_pending_y;
            injection_norm = sqrtf(injection_x * injection_x + injection_y * injection_y);
            injection_limit = POS_EST_V2_OUTPUT_CORRECTION_ACC_CMSS * dt;
            if (injection_norm > injection_limit)
            {
                injection_scale = injection_limit / injection_norm;
                injection_x *= injection_scale;
                injection_y *= injection_scale;
                s_pos_est_v2_status |= POS_EST_V2_STATUS_OUTPUT_LIMITED;
            }
            s_pos_est_v2_pending_x -= injection_x;
            s_pos_est_v2_pending_y -= injection_y;
            injection_norm = sqrtf(s_pos_est_v2_pending_x * s_pos_est_v2_pending_x +
                                   s_pos_est_v2_pending_y * s_pos_est_v2_pending_y);
            if (injection_norm > POS_EST_V2_OUTPUT_PENDING_UNRELIABLE_CMPS)
            {
                s_pos_est_v2_status |= POS_EST_V2_STATUS_UNRELIABLE;
            }
        }
    }
    s_pos_est_v2_output_vel_x = s_pos_est_v2_vel_x - s_pos_est_v2_pending_x;
    s_pos_est_v2_output_vel_y = s_pos_est_v2_vel_y - s_pos_est_v2_pending_y;

    if (s_pos_est_v2_has_accepted == 0U)
    {
        s_pos_est_v2_last_accept_age_ms = 65535.0f;
        s_pos_est_v2_status |= POS_EST_V2_STATUS_UNRELIABLE;
    }
    else
    {
        s_pos_est_v2_last_accept_age_ms =
            (float)(tick_1000us_cnt - s_pos_est_v2_last_accepted_ms);
        if ((s_pos_est_v2_reacquire_active != 0U) ||
            (s_pos_est_v2_last_accept_age_ms > (float)POS_EST_V2_REACQUIRE_MS))
        {
            s_pos_est_v2_status |= POS_EST_V2_STATUS_UNRELIABLE |
                                   POS_EST_V2_STATUS_REACQUIRE;
        }
        else if ((s_pos_est_v2_last_accept_age_ms > (float)POS_EST_V2_DEGRADED_MS) &&
                 ((s_pos_est_v2_status & POS_EST_V2_STATUS_UNRELIABLE) == 0U))
        {
            s_pos_est_v2_status |= POS_EST_V2_STATUS_DEGRADED;
        }
    }
}

/*
 * 函数名: Pos_Est_V2_Update_50HZ
 * 功能: 对新的LC302帧执行延迟LOS卡尔曼更新，并将修正后的历史状态回放到当前时刻
 * 输入参数: 无，使用当前LC302、ToF和V2历史数据
 * 返回值: 无
 */
static void Pos_Est_V2_Update_50HZ(void)
{
    uint32_t flow_seq = lc302_data_seq;
    uint32_t seq_delta;
    uint32_t status = s_pos_est_v2_status &
                      (POS_EST_V2_STATUS_DEGRADED |
                       POS_EST_V2_STATUS_UNRELIABLE |
                       POS_EST_V2_STATUS_OUTPUT_LIMITED |
                       POS_EST_V2_STATUS_REACQUIRE);
    uint16_t offset;
    uint16_t index;
    uint16_t measurement_index = 0U;
    uint16_t replay_index;
    uint16_t next_index;
    uint8_t history_found = 0U;
    uint8_t exposure_shock = 0U;
    uint8_t geometry_valid;
    uint8_t reacquiring;
    uint8_t bias_frozen;
    uint8_t bias_left_clipped = 0U;
    uint8_t bias_forward_clipped = 0U;
    float now_ms;
    float nominal_elapsed_ms;
    float phase_candidate_ms;
    float phase_min_ms;
    float measurement_time_ms;
    float sample_time_ms;
    float q_measured[2];
    float flow_rate;
    float cos_tilt;
    float rho_cm;
    float vertical_up_cmps;
    float flow_velocity_x;
    float flow_velocity_y;
    float flow_velocity_delta;
    float reacquire_yaw_cos;
    float reacquire_yaw_sin;
    float reacquire_pred_x;
    float reacquire_pred_y;
    float h[2][4];
    float h_p[2][4];
    float innovation[2];
    float offset_q[2];
    float s_matrix[2][2];
    float s_inverse[2][2];
    float determinant;
    float nis_limit;
    float sigma_scale;
    float sigma;
    float r_variance;
    float tilt_deg;
    float gain[4][2];
    float robust_scale;
    float correction_scale;
    float correction_limit;
    float correction_norm;
    float bias_delta_limit;
    float bias_delta_norm;
    float delta[4];
    float state_after[4];
    float kh[4][4];
    float temp[4][4];
    float covariance_after[4][4];
    float current_before[2];
    uint8_t i;
    uint8_t j;
    uint8_t k;
    Pos_Est_V2_History_t *measurement_sample;
    Pos_Est_V2_History_t *current_sample;

    s_pos_est_v2_status = status;
    if (flow_seq == s_pos_est_v2_last_flow_seq)
    {
        return;
    }

    seq_delta = flow_seq - s_pos_est_v2_last_flow_seq;
    s_pos_est_v2_last_flow_seq = flow_seq;
    status |= POS_EST_V2_STATUS_NEW_FLOW;
    s_pos_est_v2_innovation_x = 0.0f;
    s_pos_est_v2_innovation_y = 0.0f;
    s_pos_est_v2_correction_x = 0.0f;
    s_pos_est_v2_correction_y = 0.0f;
    s_pos_est_v2_nis = 0.0f;
    s_pos_est_v2_measurement_age_ms = 0.0f;

    if (lc302_data.valid != 0U)
    {
        status |= POS_EST_V2_STATUS_FLOW_VALID;
    }

    if ((s_pos_est_v2_has_accepted == 0U) ||
        ((tick_1000us_cnt - s_pos_est_v2_last_accepted_ms) > POS_EST_V2_REACQUIRE_MS))
    {
        if (s_pos_est_v2_reacquire_active == 0U)
        {
            s_pos_est_v2_reacquire_good_count = 0U;
            s_pos_est_v2_reacquire_stable_count = 0U;
        }
        s_pos_est_v2_reacquire_active = 1U;
    }
    reacquiring = s_pos_est_v2_reacquire_active;
    if (reacquiring != 0U)
    {
        status |= POS_EST_V2_STATUS_REACQUIRE;
    }

    now_ms = (float)tick_1000us_cnt;
    if ((s_pos_est_v2_sensor_clock_ready == 0U) ||
        (seq_delta > POS_EST_V2_SEQ_JUMP_MAX) ||
        ((now_ms - s_pos_est_v2_sensor_time_ms) > 500.0f))
    {
        s_pos_est_v2_clock_base_seq = flow_seq;
        s_pos_est_v2_clock_phase_head = 0U;
        s_pos_est_v2_clock_phase_count = 1U;
        s_pos_est_v2_clock_phase[0] = now_ms;
        s_pos_est_v2_sensor_time_ms = now_ms;
        s_pos_est_v2_sensor_clock_ready = 1U;
    }
    else
    {
        nominal_elapsed_ms = (float)(flow_seq - s_pos_est_v2_clock_base_seq) *
                             POS_EST_V2_FLOW_DT_S * 1000.0f;
        phase_candidate_ms = now_ms - nominal_elapsed_ms;
        s_pos_est_v2_clock_phase_head =
            (uint8_t)((s_pos_est_v2_clock_phase_head + 1U) % POS_EST_V2_CLOCK_WINDOW_LEN);
        s_pos_est_v2_clock_phase[s_pos_est_v2_clock_phase_head] = phase_candidate_ms;
        if (s_pos_est_v2_clock_phase_count < POS_EST_V2_CLOCK_WINDOW_LEN)
        {
            s_pos_est_v2_clock_phase_count++;
        }
        phase_min_ms = s_pos_est_v2_clock_phase[0];
        for (i = 1U; i < s_pos_est_v2_clock_phase_count; i++)
        {
            if (s_pos_est_v2_clock_phase[i] < phase_min_ms)
            {
                phase_min_ms = s_pos_est_v2_clock_phase[i];
            }
        }
        s_pos_est_v2_sensor_time_ms = phase_min_ms + nominal_elapsed_ms;
        if (s_pos_est_v2_sensor_time_ms > now_ms)
        {
            s_pos_est_v2_sensor_time_ms = now_ms;
        }
    }
    measurement_time_ms = s_pos_est_v2_sensor_time_ms - POS_EST_V2_FLOW_DELAY_MS;

    for (offset = 0U; offset < s_pos_est_v2_history_count; offset++)
    {
        index = (uint16_t)((s_pos_est_v2_history_head + POS_EST_V2_HISTORY_LEN - offset) %
                           POS_EST_V2_HISTORY_LEN);
        if ((float)s_pos_est_v2_history[index].time_ms <= measurement_time_ms)
        {
            measurement_index = index;
            history_found = 1U;
            break;
        }
    }
    if (history_found == 0U)
    {
        s_pos_est_v2_status = status;
        return;
    }

    measurement_sample = &s_pos_est_v2_history[measurement_index];
    s_pos_est_v2_measurement_age_ms = now_ms - (float)measurement_sample->time_ms;
    if (s_pos_est_v2_measurement_age_ms > POS_EST_V2_HISTORY_MAX_AGE_MS)
    {
        s_pos_est_v2_status = status;
        return;
    }
    status |= POS_EST_V2_STATUS_HISTORY_READY;
    if (measurement_sample->tof_valid != 0U)
    {
        status |= POS_EST_V2_STATUS_TOF_VALID;
    }

    q_measured[0] = FlowGyroDecoupler_LC302_GetDecX() * POS_EST_V2_FLOW_TO_RADPS;
    q_measured[1] = FlowGyroDecoupler_LC302_GetDecY() * POS_EST_V2_FLOW_TO_RADPS;
    flow_rate = sqrtf(q_measured[0] * q_measured[0] + q_measured[1] * q_measured[1]);
    cos_tilt = measurement_sample->cos_pitch * measurement_sample->cos_roll;
    geometry_valid = ((measurement_sample->height_m >= POS_EST_V2_HEIGHT_MIN_M) &&
                      (measurement_sample->height_m <= POS_EST_V2_HEIGHT_MAX_M) &&
                      (cos_tilt >= POS_EST_V2_TILT_COS_MIN) &&
                      (flow_rate <= POS_EST_V2_FLOW_RATE_MAX_RADPS))
                         ? 1U
                         : 0U;
    if (geometry_valid != 0U)
    {
        status |= POS_EST_V2_STATUS_GEOMETRY_VALID;
    }

    for (offset = 0U; offset < s_pos_est_v2_history_count; offset++)
    {
        index = (uint16_t)((s_pos_est_v2_history_head + POS_EST_V2_HISTORY_LEN - offset) %
                           POS_EST_V2_HISTORY_LEN);
        sample_time_ms = (float)s_pos_est_v2_history[index].time_ms;
        if ((sample_time_ms >= measurement_time_ms - POS_EST_V2_EXPOSURE_HALF_MS) &&
            (sample_time_ms <= measurement_time_ms + POS_EST_V2_EXPOSURE_HALF_MS) &&
            (s_pos_est_v2_history[index].shock != 0U))
        {
            exposure_shock = 1U;
            break;
        }
    }
    if (exposure_shock != 0U)
    {
        status |= POS_EST_V2_STATUS_EXPOSURE_SHOCK;
    }

    if (((status & POS_EST_V2_STATUS_FLOW_VALID) == 0U) ||
        ((status & POS_EST_V2_STATUS_TOF_VALID) == 0U) ||
        (geometry_valid == 0U) ||
        (exposure_shock != 0U))
    {
        if (reacquiring != 0U)
        {
            s_pos_est_v2_reacquire_good_count = 0U;
            s_pos_est_v2_reacquire_stable_count = 0U;
        }
        s_pos_est_v2_status = status;
        return;
    }

    rho_cm = measurement_sample->height_m * 100.0f / cos_tilt;
    vertical_up_cmps = measurement_sample->vertical_up_cmps;
    memset(h, 0, sizeof(h));
    h[0][0] = measurement_sample->cos_roll / rho_cm;
    h[0][1] = -measurement_sample->sin_roll * measurement_sample->sin_pitch / rho_cm;
    h[1][1] = measurement_sample->cos_pitch / rho_cm;
    offset_q[0] = measurement_sample->sin_roll * measurement_sample->cos_pitch *
                  vertical_up_cmps / rho_cm;
    offset_q[1] = measurement_sample->sin_pitch * vertical_up_cmps / rho_cm;
    innovation[0] = q_measured[0] -
                    (h[0][0] * measurement_sample->state[0] +
                     h[0][1] * measurement_sample->state[1] + offset_q[0]);
    innovation[1] = q_measured[1] -
                    (h[1][1] * measurement_sample->state[1] + offset_q[1]);
    s_pos_est_v2_innovation_y = innovation[1] / h[1][1];
    s_pos_est_v2_innovation_x =
        (innovation[0] - h[0][1] * s_pos_est_v2_innovation_y) / h[0][0];
    flow_velocity_x = measurement_sample->state[0] + s_pos_est_v2_innovation_x;
    flow_velocity_y = measurement_sample->state[1] + s_pos_est_v2_innovation_y;
    opflow_vel_x = flow_velocity_x;
    opflow_vel_y = flow_velocity_y;
    opflow_vel_x_lpf = flow_velocity_x;
    opflow_vel_y_lpf = flow_velocity_y;

    tilt_deg = acosf(Pos_Est_ClampFloat(cos_tilt, -1.0f, 1.0f)) / POS_EST_DEG_TO_RAD;
    sigma_scale = 1.0f +
                  0.004f * ((measurement_sample->gyro_norm_dps > 10.0f)
                                ? (measurement_sample->gyro_norm_dps - 10.0f)
                                : 0.0f) +
                  0.020f * ((tilt_deg > 5.0f) ? (tilt_deg - 5.0f) : 0.0f) +
                  ((seq_delta > 1U) ? 0.50f : 0.0f);
    sigma = POS_EST_V2_SIGMA_FLOW_RADPS * sigma_scale;
    s_pos_est_v2_sigma_flow_radps = sigma;
    r_variance = sigma * sigma;

    for (i = 0U; i < 2U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            h_p[i][j] = 0.0f;
            for (k = 0U; k < 4U; k++)
            {
                h_p[i][j] += h[i][k] * measurement_sample->covariance[k][j];
            }
        }
    }
    for (i = 0U; i < 2U; i++)
    {
        for (j = 0U; j < 2U; j++)
        {
            s_matrix[i][j] = 0.0f;
            for (k = 0U; k < 4U; k++)
            {
                s_matrix[i][j] += h_p[i][k] * h[j][k];
            }
            if (i == j)
            {
                s_matrix[i][j] += r_variance;
            }
        }
    }

    determinant = s_matrix[0][0] * s_matrix[1][1] -
                  s_matrix[0][1] * s_matrix[1][0];
    if (fabsf(determinant) < 1.0e-12f)
    {
        status |= POS_EST_V2_STATUS_REJECTED;
        if (reacquiring != 0U)
        {
            s_pos_est_v2_reacquire_good_count = 0U;
            s_pos_est_v2_reacquire_stable_count = 0U;
        }
        s_pos_est_v2_status = status;
        return;
    }
    s_inverse[0][0] = s_matrix[1][1] / determinant;
    s_inverse[0][1] = -s_matrix[0][1] / determinant;
    s_inverse[1][0] = -s_matrix[1][0] / determinant;
    s_inverse[1][1] = s_matrix[0][0] / determinant;
    s_pos_est_v2_nis = innovation[0] *
                           (s_inverse[0][0] * innovation[0] +
                            s_inverse[0][1] * innovation[1]) +
                       innovation[1] *
                           (s_inverse[1][0] * innovation[0] +
                            s_inverse[1][1] * innovation[1]);
    if (s_pos_est_v2_nis < 0.0f)
    {
        s_pos_est_v2_nis = 0.0f;
    }

    if (((reacquiring == 0U) && (s_pos_est_v2_nis > POS_EST_V2_NIS_MAX)) ||
        ((reacquiring != 0U) && (s_pos_est_v2_nis > POS_EST_V2_NIS_REACQUIRE_MAX)))
    {
        status |= POS_EST_V2_STATUS_REJECTED;
        if (reacquiring != 0U)
        {
            s_pos_est_v2_reacquire_good_count = 0U;
            s_pos_est_v2_reacquire_stable_count = 0U;
        }
        s_pos_est_v2_status = status;
        return;
    }

    if (reacquiring != 0U)
    {
        if (s_pos_est_v2_nis <= POS_EST_V2_NIS_REACQUIRE)
        {
            if ((s_pos_est_v2_reacquire_good_count == 0U) || (seq_delta != 1U))
            {
                s_pos_est_v2_reacquire_good_count = 1U;
                s_pos_est_v2_reacquire_stable_count = 0U;
            }
            else
            {
                flow_velocity_delta = measurement_sample->yaw_rate_rps *
                                      POS_EST_V2_FLOW_DT_S;
                reacquire_yaw_cos = cosf(flow_velocity_delta);
                reacquire_yaw_sin = sinf(flow_velocity_delta);
                reacquire_pred_x =
                    reacquire_yaw_cos * s_pos_est_v2_reacquire_prev_x +
                    reacquire_yaw_sin * s_pos_est_v2_reacquire_prev_y +
                    (measurement_sample->accel_left_cmss - measurement_sample->state[2]) *
                        POS_EST_V2_FLOW_DT_S;
                reacquire_pred_y =
                    -reacquire_yaw_sin * s_pos_est_v2_reacquire_prev_x +
                    reacquire_yaw_cos * s_pos_est_v2_reacquire_prev_y +
                    (measurement_sample->accel_forward_cmss - measurement_sample->state[3]) *
                        POS_EST_V2_FLOW_DT_S;
                flow_velocity_delta = sqrtf((flow_velocity_x - reacquire_pred_x) *
                                                (flow_velocity_x - reacquire_pred_x) +
                                            (flow_velocity_y - reacquire_pred_y) *
                                                (flow_velocity_y - reacquire_pred_y));
                if (flow_velocity_delta <= POS_EST_V2_REACQUIRE_CONSISTENCY_CMPS)
                {
                    if (s_pos_est_v2_reacquire_good_count <
                        POS_EST_V2_REACQUIRE_GOOD_FRAMES)
                    {
                        s_pos_est_v2_reacquire_good_count++;
                    }
                }
                else
                {
                    s_pos_est_v2_reacquire_good_count = 1U;
                    s_pos_est_v2_reacquire_stable_count = 0U;
                }
            }
            s_pos_est_v2_reacquire_prev_x = flow_velocity_x;
            s_pos_est_v2_reacquire_prev_y = flow_velocity_y;
            if (s_pos_est_v2_reacquire_good_count >= POS_EST_V2_REACQUIRE_GOOD_FRAMES)
            {
                status |= POS_EST_V2_STATUS_REACQUIRE_CONSISTENT;
            }
        }
        else
        {
            s_pos_est_v2_reacquire_good_count = 0U;
            s_pos_est_v2_reacquire_stable_count = 0U;
            s_pos_est_v2_reacquire_prev_x = flow_velocity_x;
            s_pos_est_v2_reacquire_prev_y = flow_velocity_y;
        }
    }

    for (i = 0U; i < 4U; i++)
    {
        gain[i][0] = h_p[0][i] * s_inverse[0][0] + h_p[1][i] * s_inverse[1][0];
        gain[i][1] = h_p[0][i] * s_inverse[0][1] + h_p[1][i] * s_inverse[1][1];
    }
    nis_limit = (reacquiring != 0U) ? POS_EST_V2_NIS_REACQUIRE : POS_EST_V2_NIS_NORMAL;
    robust_scale = sqrtf(nis_limit / ((s_pos_est_v2_nis > 1.0e-9f)
                                          ? s_pos_est_v2_nis
                                          : 1.0e-9f));
    if (robust_scale > 1.0f)
    {
        robust_scale = 1.0f;
    }
    for (i = 0U; i < 4U; i++)
    {
        gain[i][0] *= robust_scale;
        gain[i][1] *= robust_scale;
    }
    for (i = 0U; i < 4U; i++)
    {
        delta[i] = gain[i][0] * innovation[0] + gain[i][1] * innovation[1];
    }
    if ((reacquiring != 0U) &&
        (s_pos_est_v2_reacquire_good_count < POS_EST_V2_REACQUIRE_GOOD_FRAMES))
    {
        correction_limit = POS_EST_V2_CORRECTION_PROBE_CMPS;
    }
    else
    {
        correction_limit = (reacquiring != 0U) ? POS_EST_V2_CORRECTION_REACQUIRE_CMPS
                                                : POS_EST_V2_CORRECTION_NORMAL_CMPS;
    }
    correction_norm = sqrtf(delta[0] * delta[0] + delta[1] * delta[1]);
    correction_scale = correction_limit /
                       ((correction_norm > 1.0e-9f) ? correction_norm : 1.0e-9f);
    if (correction_scale > 1.0f)
    {
        correction_scale = 1.0f;
    }
    for (i = 0U; i < 4U; i++)
    {
        gain[i][0] *= correction_scale;
        gain[i][1] *= correction_scale;
        delta[i] *= correction_scale;
    }

    bias_frozen = ((reacquiring != 0U) ||
                   (robust_scale < 0.999f) ||
                   (correction_scale < 0.999f) ||
                   (s_pos_est_v2_nis > POS_EST_V2_NIS_NORMAL))
                      ? 1U
                      : 0U;
    if (bias_frozen != 0U)
    {
        gain[2][0] = 0.0f;
        gain[2][1] = 0.0f;
        gain[3][0] = 0.0f;
        gain[3][1] = 0.0f;
        delta[2] = 0.0f;
        delta[3] = 0.0f;
        status |= POS_EST_V2_STATUS_BIAS_FROZEN;
    }
    else
    {
        bias_delta_limit = 5.0f;
        bias_delta_norm = sqrtf(delta[2] * delta[2] + delta[3] * delta[3]);
        if (bias_delta_norm > bias_delta_limit)
        {
            float bias_scale = bias_delta_limit / bias_delta_norm;
            gain[2][0] *= bias_scale;
            gain[2][1] *= bias_scale;
            gain[3][0] *= bias_scale;
            gain[3][1] *= bias_scale;
            delta[2] *= bias_scale;
            delta[3] *= bias_scale;
        }
    }

    for (i = 0U; i < 4U; i++)
    {
        state_after[i] = measurement_sample->state[i] + delta[i];
    }
    if (state_after[2] > POS_EST_V2_BIAS_LIMIT_CMSS)
    {
        state_after[2] = POS_EST_V2_BIAS_LIMIT_CMSS;
        bias_left_clipped = 1U;
    }
    else if (state_after[2] < -POS_EST_V2_BIAS_LIMIT_CMSS)
    {
        state_after[2] = -POS_EST_V2_BIAS_LIMIT_CMSS;
        bias_left_clipped = 1U;
    }
    if (state_after[3] > POS_EST_V2_BIAS_LIMIT_CMSS)
    {
        state_after[3] = POS_EST_V2_BIAS_LIMIT_CMSS;
        bias_forward_clipped = 1U;
    }
    else if (state_after[3] < -POS_EST_V2_BIAS_LIMIT_CMSS)
    {
        state_after[3] = -POS_EST_V2_BIAS_LIMIT_CMSS;
        bias_forward_clipped = 1U;
    }

    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            kh[i][j] = (i == j) ? 1.0f : 0.0f;
            kh[i][j] -= gain[i][0] * h[0][j] + gain[i][1] * h[1][j];
        }
    }
    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            temp[i][j] = 0.0f;
            for (k = 0U; k < 4U; k++)
            {
                temp[i][j] += kh[i][k] * measurement_sample->covariance[k][j];
            }
        }
    }
    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            covariance_after[i][j] = 0.0f;
            for (k = 0U; k < 4U; k++)
            {
                covariance_after[i][j] += temp[i][k] * kh[j][k];
            }
            covariance_after[i][j] += r_variance *
                                      (gain[i][0] * gain[j][0] + gain[i][1] * gain[j][1]);
        }
    }
    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            measurement_sample->covariance[i][j] =
                0.5f * (covariance_after[i][j] + covariance_after[j][i]);
        }
        measurement_sample->state[i] = state_after[i];
    }
    if (bias_left_clipped != 0U)
    {
        for (i = 0U; i < 4U; i++)
        {
            measurement_sample->covariance[2][i] = 0.0f;
            measurement_sample->covariance[i][2] = 0.0f;
        }
        measurement_sample->covariance[2][2] = 25.0f;
    }
    if (bias_forward_clipped != 0U)
    {
        for (i = 0U; i < 4U; i++)
        {
            measurement_sample->covariance[3][i] = 0.0f;
            measurement_sample->covariance[i][3] = 0.0f;
        }
        measurement_sample->covariance[3][3] = 25.0f;
    }

    current_sample = &s_pos_est_v2_history[s_pos_est_v2_history_head];
    current_before[0] = current_sample->state[0];
    current_before[1] = current_sample->state[1];
    replay_index = measurement_index;
    while (replay_index != s_pos_est_v2_history_head)
    {
        next_index = (uint16_t)((replay_index + 1U) % POS_EST_V2_HISTORY_LEN);
        Pos_Est_V2_Propagate(&s_pos_est_v2_history[next_index],
                             &s_pos_est_v2_history[replay_index]);
        replay_index = next_index;
    }

    current_sample = &s_pos_est_v2_history[s_pos_est_v2_history_head];
    s_pos_est_v2_vel_x = current_sample->state[0];
    s_pos_est_v2_vel_y = current_sample->state[1];
    s_pos_est_v2_bias_x = current_sample->state[2];
    s_pos_est_v2_bias_y = current_sample->state[3];
    s_pos_est_v2_correction_x = current_sample->state[0] - current_before[0];
    s_pos_est_v2_correction_y = current_sample->state[1] - current_before[1];
    s_pos_est_v2_pending_x += s_pos_est_v2_correction_x;
    s_pos_est_v2_pending_y += s_pos_est_v2_correction_y;
    correction_norm = sqrtf(s_pos_est_v2_pending_x * s_pos_est_v2_pending_x +
                            s_pos_est_v2_pending_y * s_pos_est_v2_pending_y);
    if (correction_norm > POS_EST_V2_OUTPUT_PENDING_MAX_CMPS)
    {
        correction_scale = POS_EST_V2_OUTPUT_PENDING_MAX_CMPS / correction_norm;
        s_pos_est_v2_pending_x *= correction_scale;
        s_pos_est_v2_pending_y *= correction_scale;
        status |= POS_EST_V2_STATUS_UNRELIABLE |
                  POS_EST_V2_STATUS_OUTPUT_LIMITED |
                  POS_EST_V2_STATUS_PENDING_CLAMPED;
    }
    s_pos_est_v2_output_vel_x = s_pos_est_v2_vel_x - s_pos_est_v2_pending_x;
    s_pos_est_v2_output_vel_y = s_pos_est_v2_vel_y - s_pos_est_v2_pending_y;
    s_pos_est_v2_last_accepted_ms = tick_1000us_cnt;
    s_pos_est_v2_has_accepted = 1U;
    status |= POS_EST_V2_STATUS_ACCEPTED;
    if (reacquiring != 0U)
    {
        if ((s_pos_est_v2_reacquire_good_count >= POS_EST_V2_REACQUIRE_GOOD_FRAMES) &&
            (s_pos_est_v2_nis <= POS_EST_V2_NIS_NORMAL) &&
            (s_pos_est_v2_reacquire_stable_count < POS_EST_V2_REACQUIRE_GOOD_FRAMES))
        {
            s_pos_est_v2_reacquire_stable_count++;
        }
        else if (s_pos_est_v2_nis > POS_EST_V2_NIS_NORMAL)
        {
            s_pos_est_v2_reacquire_stable_count = 0U;
        }
        if (s_pos_est_v2_reacquire_stable_count >= POS_EST_V2_REACQUIRE_GOOD_FRAMES)
        {
            s_pos_est_v2_reacquire_active = 0U;
            s_pos_est_v2_reacquire_good_count = 0U;
            s_pos_est_v2_reacquire_stable_count = 0U;
        }
    }
    s_pos_est_v2_status = status;
}

/*
 * 函数名: Pos_Est_Init
 * 功能: 初始化光流位置估计模块和相关滤波器
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Init(void)
{
    // PMW3901_Init();
    LC302_Init();
    // LC302_Init_Aux();
    // FlowGyroDecoupler_Init();
    FlowGyroDecoupler_LC302_Init();

    opflow_vel_x = 0.0f;
    opflow_vel_y = 0.0f;
    opflow_vel_x_lpf = 0.0f;
    opflow_vel_y_lpf = 0.0f;
    Pos_Est_vel_x = 0.0f;
    Pos_Est_vel_y = 0.0f;
    Pos_Est_vel_x_level = 0.0f;
    Pos_Est_vel_y_level = 0.0f;
    s_raw_acc_lp_x = 0.0f;
    s_raw_acc_lp_y = 0.0f;
    Pos_Est_acc_bias_x_cmss = 0.0f;
    Pos_Est_acc_bias_y_cmss = 0.0f;
    s_static_sample_count = 0U;
    s_pos_est_last_update_ms = 0U;
    s_pos_est_update_time_ready = 0U;
    Pos_Est_V2_Init();
}

/*
 * 函数名: Pos_Est_Reinit
 * 功能: 重置光流和滤波器内部状态
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Reinit(void)
{
    // FlowGyroDecoupler_Reinit();
    FlowGyroDecoupler_LC302_Reinit();

    acc_x_temp = 0.0f;
    acc_y_temp = 0.0f;
    opflow_vel_x = 0.0f;
    opflow_vel_y = 0.0f;
    opflow_vel_x_lpf = 0.0f;
    opflow_vel_y_lpf = 0.0f;
    Pos_Est_vel_x = 0.0f;
    Pos_Est_vel_y = 0.0f;
    Pos_Est_vel_x_level = 0.0f;
    Pos_Est_vel_y_level = 0.0f;
    s_raw_acc_lp_x = 0.0f;
    s_raw_acc_lp_y = 0.0f;
    Pos_Est_acc_bias_x_cmss = 0.0f;
    Pos_Est_acc_bias_y_cmss = 0.0f;
    s_static_sample_count = 0U;
    s_pos_est_last_update_ms = 0U;
    s_pos_est_update_time_ready = 0U;
    Pos_Est_V2_Init();
}

/*
 * 函数名: Pos_Est_Update_1000HZ
 * 功能: 推送光流去旋转解耦所需陀螺数据，并更新水平加速度低通输出
 * 输入参数: 无
 * 返回值: 无
 *
 * 加速度数据路径说明：
 *   使用 g_imufilter_1000hz（已经过陷波161Hz/320Hz + 12Hz低通滤波）作为输入，
 *   通过 AccelCalibration_RotateImuToBody 旋转到机体系后，利用旋转矩阵投影到水平系。
 *   重力分量在旋转投影中自动消除，无需额外去重力步骤（数学推导验证）。
 *   相比原 AccelCalibration_GetLevelAccelMps2 路径（原始未滤波acc），
 *   可消除飞行中电机振动导致的 ±100~300 cm/s² 污染。
 */
void Pos_Est_Update_1000HZ(void)
{
    float acc_sensor[3];
    float acc_body[3];
    float sp;
    float cp;
    float sr;
    float cr;
    uint8_t accel_bias_static;
    uint8_t accel_bias_locked = 0U;

    if ((s_pos_est_update_time_ready != 0U) &&
        (tick_1000us_cnt == s_pos_est_last_update_ms))
    {
        return;
    }
    s_pos_est_last_update_ms = tick_1000us_cnt;
    s_pos_est_update_time_ready = 1U;

    // FlowGyroDecoupler_Push1000Hz(tick_1000us_cnt, g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy);
    FlowGyroDecoupler_LC302_Push1000Hz(tick_1000us_cnt, g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy);

    /* Use filtered IMU accel, then rotate to body frame and apply level-accel LPF. */
    acc_sensor[0] = g_imufilter_1000hz.accx;
    acc_sensor[1] = g_imufilter_1000hz.accy;
    acc_sensor[2] = g_imufilter_1000hz.accz;
    AccelCalibration_RotateImuToBody(acc_sensor, acc_body);

    sp = g_euler.sin_pitch;
    cp = g_euler.cos_pitch;
    sr = g_euler.sin_roll;
    cr = g_euler.cos_roll;

    /* 旋转到水平系（yaw=0近似），重力自动消除。
     * level_x = cp*body_x + sp*sr*body_y + sp*cr*body_z  (前进为正)
     * level_y = cr*body_y - sr*body_z                     (向右为正)
     * 单位 g → cm/s^2 */
    acc_x_temp = (cp * acc_body[0] + sp * sr * acc_body[1] + sp * cr * acc_body[2]) * 9.80665f * 100.0f;
    acc_y_temp = (cr * acc_body[1] - sr * acc_body[2]) * 9.80665f * 100.0f;
    s_raw_acc_lp_x += POS_EST_RAW_ACC_LPF_ALPHA * (acc_x_temp - s_raw_acc_lp_x);
    s_raw_acc_lp_y += POS_EST_RAW_ACC_LPF_ALPHA * (acc_y_temp - s_raw_acc_lp_y);
    accel_bias_static = Pos_Est_IsStaticForAccelBias();
    if (accel_bias_static != 0U)
    {
        if (s_static_sample_count < POS_EST_STATIC_LOCK_SAMPLES)
        {
            s_static_sample_count++;
        }
        else
        {
            accel_bias_locked = 1U;
            Pos_Est_acc_bias_x_cmss += POS_EST_ACC_BIAS_ALPHA * (s_raw_acc_lp_x - Pos_Est_acc_bias_x_cmss);
            Pos_Est_acc_bias_y_cmss += POS_EST_ACC_BIAS_ALPHA * (s_raw_acc_lp_y - Pos_Est_acc_bias_y_cmss);
            Pos_Est_acc_bias_x_cmss = Pos_Est_ClampFloat(Pos_Est_acc_bias_x_cmss,
                                                         -POS_EST_ACC_BIAS_LIMIT_CMSS,
                                                         POS_EST_ACC_BIAS_LIMIT_CMSS);
            Pos_Est_acc_bias_y_cmss = Pos_Est_ClampFloat(Pos_Est_acc_bias_y_cmss,
                                                         -POS_EST_ACC_BIAS_LIMIT_CMSS,
                                                         POS_EST_ACC_BIAS_LIMIT_CMSS);
        }
    }
    else
    {
        s_static_sample_count = 0U;
    }

    acc_x_lp = s_raw_acc_lp_x - Pos_Est_acc_bias_x_cmss;
    acc_y_lp = s_raw_acc_lp_y - Pos_Est_acc_bias_y_cmss;

    /* IMU 冲击窗口内不让水平加速度污染后续速度积分 */
    if (g_imu_shock_flag != 0U)
    {
        acc_x_lp = 0.0f;
        acc_y_lp = 0.0f;
        s_raw_acc_lp_x = 0.0f;
        s_raw_acc_lp_y = 0.0f;
    }

    if (acc_x_lp > POS_EST_ACC_FWD_LIMIT_CMSS)
    {
        acc_x_lp = POS_EST_ACC_FWD_LIMIT_CMSS;
    }
    else if (acc_x_lp < -POS_EST_ACC_FWD_LIMIT_CMSS)
    {
        acc_x_lp = -POS_EST_ACC_FWD_LIMIT_CMSS;
    }

    if (acc_y_lp > POS_EST_ACC_RIGHT_LIMIT_CMSS)
    {
        acc_y_lp = POS_EST_ACC_RIGHT_LIMIT_CMSS;
    }
    else if (acc_y_lp < -POS_EST_ACC_RIGHT_LIMIT_CMSS)
    {
        acc_y_lp = -POS_EST_ACC_RIGHT_LIMIT_CMSS;
    }

    // float dec_x_pmw3901 = FlowGyroDecoupler_GetDecX();
    // float dec_y_pmw3901 = FlowGyroDecoupler_GetDecY();
    // float dec_x_lc302 = FlowGyroDecoupler_LC302_GetDecX();
    // float dec_y_lc302 = FlowGyroDecoupler_LC302_GetDecY();
    // FC_START_CRSF_state_e FC_START_CRSF_state = FC_START_CRSF_Get_State();
    // float dec_x, dec_y;
    // dec_x = FlowGyroDecoupler_LC302_GetDecX();
    // dec_y = FlowGyroDecoupler_LC302_GetDecY();
    //                acc_x_temp, acc_y_temp,
    //                g_tof_fused_height_mm * 0.001f,
    //                lc302_data.flow_x_integral, lc302_data.flow_y_integral,
    //                g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy, g_imufilter_1000hz.gyroz, g_euler.pitch, g_euler.roll, g_euler.yaw,
    //                dec_x, dec_y, lc302_data.integration_timespan, lc302_data.valid);
    float dec_x;
    float dec_y;
    dec_x = FlowGyroDecoupler_LC302_GetDecX();
    dec_y = FlowGyroDecoupler_LC302_GetDecY();

    Pos_Est_V2_Update_1000HZ();
    Pos_Est_vel_x = s_pos_est_v2_output_vel_x;
    Pos_Est_vel_y = s_pos_est_v2_output_vel_y;
    {
        float yaw_sin = sinf(g_euler.yaw * POS_EST_DEG_TO_RAD);
        float yaw_cos = cosf(g_euler.yaw * POS_EST_DEG_TO_RAD);

        Pos_Est_vel_x_level = yaw_cos * Pos_Est_vel_x - yaw_sin * Pos_Est_vel_y;
        Pos_Est_vel_y_level = yaw_sin * Pos_Est_vel_x + yaw_cos * Pos_Est_vel_y;
    }

    if ((tick_1000us_cnt & 1U) == 0U)
    {
        wifi_justfloat(acc_x_temp, acc_y_temp,
                acc_x_lp, acc_y_lp,
                Pos_Est_acc_bias_x_cmss, Pos_Est_acc_bias_y_cmss,
                g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy, g_imufilter_1000hz.gyroz,
                g_euler.pitch, g_euler.roll, g_euler.yaw,
                g_tof_fused_height_mm * 0.001f,
                g_height_fused_vz_mps * 100.0f,
                (float)(((lc302_data_seq & 0x7FFFFU) << 5U) |
                        ((uint32)(lc302_data.valid != 0U)) |
                        ((uint32)(g_tof_fused_valid != 0U) << 1U) |
                        ((uint32)(g_imu_shock_flag != 0U) << 2U) |
                        ((uint32)(accel_bias_static != 0U) << 3U) |
                        ((uint32)(accel_bias_locked != 0U) << 4U)),
                lc302_data.flow_x_integral, lc302_data.flow_y_integral,
                dec_x, dec_y,
                Pos_Est_vel_x, Pos_Est_vel_y,
                s_pos_est_v2_vel_x, s_pos_est_v2_vel_y,
                s_pos_est_v2_bias_x, s_pos_est_v2_bias_y,
                s_pos_est_v2_innovation_x, s_pos_est_v2_innovation_y,
                s_pos_est_v2_correction_x, s_pos_est_v2_correction_y,
                s_pos_est_v2_nis, (float)s_pos_est_v2_status,
                s_pos_est_v2_measurement_age_ms,
                s_pos_est_v2_last_accept_age_ms,
                s_pos_est_v2_pending_x, s_pos_est_v2_pending_y,
                s_pos_est_v2_sigma_flow_radps,
                (float)s_pos_est_v2_reacquire_good_count
                );
    }
    
    //                acc_x_temp, acc_y_temp,
    //                ICM42688.acc_x, ICM42688.acc_y, ICM42688.acc_z,
    //                acc_x_lp, acc_y_lp,
    //                g_tof_fused_height_mm * 0.001f,
    //                lc302_data.flow_x_integral, lc302_data.flow_y_integral,
    //                dec_x, dec_y,
    //                opflow_vel_x, opflow_vel_y,
    //                Pos_Est_vel_x, Pos_Est_vel_y,
    //                velx_target, vely_target,
    //                velx_pid->p_term, velx_pid->i_term, velx_pid->d_term, velx_pid->output,
    //                vely_pid->p_term, vely_pid->i_term, vely_pid->d_term, vely_pid->output,
    //                //    opflow_vel_x_lpf, opflow_vel_y_lpf,
    //                pitch_angle_target, roll_angle_target,
    //                g_euler.pitch, g_euler.roll, g_euler.yaw);
}

/*
 * 函数名: Pos_Est_Update_50HZ
 * 功能: 更新 LC302 光流速度，并执行速度融合与位置积分
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Update_50HZ(void)
{
    uint32_t previous_flow_seq = lc302_data_seq;

    // PMW3901_Update_50HZ();
    LC302_Update_50HZ();
    // LC302_Update_50HZ_Aux();
    if (lc302_data_seq == previous_flow_seq)
    {
        return;
    }
    // FlowGyroDecoupler_Update50Hz(tick_1000us_cnt, g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY);
    (void)FlowGyroDecoupler_LC302_Update50Hz(tick_1000us_cnt, lc302_data.flow_x_integral, lc302_data.flow_y_integral, lc302_data.valid);
    Pos_Est_V2_Update_50HZ();
}
