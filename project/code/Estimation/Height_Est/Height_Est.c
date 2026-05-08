#include "Height_Est.h"
#include "../Attitude/Accel_Calibration.h"
#include "zf_common_headfile.h"
#include "filter.h"
#include <math.h>

float g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;  /* TOF/KF 融合高度，单位 mm */
float g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;   /* 1 号 TOF 姿态补偿后高度，单位 mm */
float g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;   /* 2 号 TOF 姿态补偿后高度，单位 mm */
float g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;   /* 3 号 TOF 姿态补偿后高度，单位 mm */
float g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;   /* 4 号 TOF 姿态补偿后高度，单位 mm */
uint8 g_tof_fused_valid = 0U;                                  /* TOF/KF 融合高度有效标志：1=有效，0=无效 */
float g_tof_fused_vz_mps = 0.0f;                               /* 高度融合速度，单位 m/s，上升为正 */
float g_height_fused_vz_mps = 0.0f;                            /* 控制环使用的高度速度，单位 m/s，上升为正 */
extern volatile uint32 tick_1000us_cnt;                        /* 1ms 系统节拍计数，用于遥测输出 */

#define HEIGHT_EST_TOF1_INDEX           0U                  /* 1 号 TOF 数据索引 */
#define HEIGHT_EST_TOF2_INDEX           1U                  /* 2 号 TOF 数据索引 */
#define HEIGHT_EST_TOF3_INDEX           2U                  /* 3 号 TOF 数据索引 */
#define HEIGHT_EST_TOF4_INDEX           3U                  /* 4 号 TOF 数据索引 */
#define HEIGHT_EST_IMU_DT_S             0.001f              /* IMU 预测周期，单位 s */
#define HEIGHT_EST_TOF_DT_S             0.01f               /* TOF 速度观测周期，单位 s */
#define HEIGHT_EST_VZ_LPF_ALPHA         0.39508f            /* 控制速度输出 8Hz 一阶低通系数 */
#define HEIGHT_EST_VZ_OBS_ALPHA         0.14f               /* 控制速度观测器高度校正系数 */
#define HEIGHT_EST_VZ_OBS_BETA          0.022f              /* 控制速度观测器速度校正系数 */
#define HEIGHT_EST_VZ_OBS_LEAK_ALPHA    0.97183f            /* 控制速度观测器 0.35s 速度泄漏系数 */
#define HEIGHT_EST_VZ_OBS_ACC_DEADBAND  0.22f               /* 控制速度观测器加速度死区，单位 m/s^2 */
#define HEIGHT_EST_VZ_OBS_ACC_CLIP      4.0f                /* 控制速度观测器加速度限幅，单位 m/s^2 */
#define HEIGHT_EST_VZ_OBS_RES_SOFT_M    0.08f               /* 控制速度观测器残差软门限，单位 m */
#define HEIGHT_EST_VZ_OBS_RES_HARD_M    0.20f               /* 控制速度观测器残差硬门限，单位 m */
#define HEIGHT_EST_VZ_OBS_DV_LIMIT      0.03f               /* 控制速度观测器单次速度校正限幅，单位 m/s */
#define HEIGHT_EST_VZ_OBS_LIMIT_MPS     1.5f                /* 控制速度观测器输出速度限幅，单位 m/s */
#define HEIGHT_EST_VZ_OBS_MISS_MAX      15U                 /* 控制速度观测器允许 TOF 短时丢失的 100Hz 次数 */
#define HEIGHT_EST_TOF_TRIM_GATE_MM     80.0f               /* 控制速度观测器 TOF 截尾门限，单位 mm */
#define HEIGHT_EST_TOF1_BIAS_MM         (-14.18f)            /* 1 号 TOF 相对偏置，单位 mm */
#define HEIGHT_EST_TOF2_BIAS_MM         (14.33f)             /* 2 号 TOF 相对偏置，单位 mm */
#define HEIGHT_EST_TOF3_BIAS_MM         (-37.02f)            /* 3 号 TOF 相对偏置，单位 mm */
#define HEIGHT_EST_TOF4_BIAS_MM         (55.94f)             /* 4 号 TOF 相对偏置，单位 mm */
#define HEIGHT_EST_MEDIAN_WIN           3U                  /* 单路 TOF 中值滤波窗口长度 */
#define HEIGHT_EST_STEP_LIMIT_MM        30.0f               /* 单路 TOF 每次更新最大跳变，单位 mm */
#define HEIGHT_EST_RESIDUAL_GATE_MM     120.0f              /* 预测高度残差门限，单位 mm */
#define HEIGHT_EST_PREDICT_HOLD_CNT     15U                 /* TOF 丢失后继续保持预测有效的 100Hz 次数 */
#define HEIGHT_EST_VEL_DECAY            0.95f               /* TOF 长时间无效后的速度衰减系数 */
#define HEIGHT_EST_WEIGHT_EPS           0.001f              /* 融合权重有效下限 */
#define HEIGHT_EST_HUBER_K_MM           20.0f               /* TOF 鲁棒融合 Huber 拐点，单位 mm */
#define HEIGHT_EST_TOF_PITCH_ARM_MM     65.40f              /* TOF 相对中心的 pitch 力臂，单位 mm */
#define HEIGHT_EST_TOF_ROLL_ARM_MM      84.81f              /* TOF 相对中心的 roll 力臂，单位 mm */
#define HEIGHT_EST_TOF_VALID_MIN_MM     50.0f               /* TOF 原始测距有效最小值，单位 mm */
#define HEIGHT_EST_TOF_VALID_MAX_MM     1299.5f             /* TOF 原始测距有效最大值，单位 mm */
#define HEIGHT_EST_STATE_MAX_MM         1600.0f             /* KF 内部高度状态最大值，单位 mm */
#define HEIGHT_EST_ACC_LPF_ALPHA        0.035f              /* 垂直加速度 1kHz 一阶低通系数 */
#define HEIGHT_EST_ACC_CLIP_MPS2        5.0f                /* 垂直加速度预测限幅，单位 m/s^2 */
#define HEIGHT_EST_VEL_LIMIT_MPS        3.5f                /* 高度速度状态限幅，单位 m/s */
#define HEIGHT_EST_BIAS_LIMIT_MPS2      1.0f                /* 垂直加速度偏置状态限幅，单位 m/s^2 */
#define HEIGHT_EST_KF_SIGMA_A           3.2f                /* KF 加速度过程噪声，单位 m/s^2 */
#define HEIGHT_EST_KF_SIGMA_B           0.08f               /* KF 加速度偏置过程噪声，单位 m/s^2 */
#define HEIGHT_EST_KF_SIGMA_H           0.025f              /* KF 标准 TOF 观测噪声，单位 m */
#define HEIGHT_EST_KF_SIGMA_H_BAD       0.08f               /* KF 离散 TOF 观测噪声，单位 m */
#define HEIGHT_EST_KF_SPREAD_BAD_M      0.10f               /* TOF 样本离散加重噪声阈值，单位 m */
#define HEIGHT_EST_KF_VEL_CORR_LIMIT    0.08f               /* 单次 TOF 校正速度最大修正量，单位 m/s */
#define HEIGHT_EST_KF_BIAS_CORR_LIMIT   0.03f               /* 单次 TOF 校正偏置最大修正量，单位 m/s^2 */
#define HEIGHT_EST_HARD_RELOCK_SPREAD_MM 220.0f             /* TOF 重锁允许的样本最大离散度，单位 mm */
#define HEIGHT_EST_HARD_RELOCK_VEL_LIMIT 0.60f              /* TOF 重锁时保留速度的限幅，单位 m/s */
#define HEIGHT_EST_KF_P_H0              (0.08f * 0.08f)     /* KF 高度初始方差，单位 m^2 */
#define HEIGHT_EST_KF_P_V0              (0.80f * 0.80f)     /* KF 速度初始方差，单位 (m/s)^2 */
#define HEIGHT_EST_KF_P_B0              (0.40f * 0.40f)     /* KF 加速度偏置初始方差，单位 (m/s^2)^2 */

static Median_t s_tof_median[VL53L1X_SENSOR_COUNT];          /* 四路 TOF 中值滤波状态 */
static StepLim_t s_tof_step[VL53L1X_SENSOR_COUNT];           /* 四路 TOF 步进限幅状态 */
static float s_height_est_mm = (float)VL53L1X_VALID_RANGE_MAX; /* KF 高度状态，单位 mm */
static float s_height_est_vz_mps = 0.0f;                     /* KF 垂直速度状态，单位 m/s，上升为正 */
static LPF1_t s_height_out_vz_lpf;                          /* 控制速度输出低通滤波器，单位 m/s */
static float s_height_out_z_m = 0.0f;                        /* 控制速度观测器高度状态，单位 m */
static float s_height_out_vz_mps = 0.0f;                     /* 控制速度观测器速度状态，单位 m/s */
static uint8 s_height_out_vz_ready = 0U;                     /* 控制速度观测器是否已初始化 */
static uint8 s_height_out_miss_cnt = 0U;                     /* 控制速度观测器 TOF 连续丢失计数 */
static float s_height_acc_bias_up_mps2 = 0.0f;               /* KF 垂直向上加速度偏置，单位 m/s^2 */
static float s_height_acc_lpf_up_mps2 = 0.0f;                /* 垂直向上加速度低通状态，单位 m/s^2 */
static uint8 s_height_acc_lpf_ready = 0U;                    /* 垂直加速度低通是否已初始化 */
static float s_height_kf_p00 = HEIGHT_EST_KF_P_H0;           /* KF 协方差 P00：高度-高度 */
static float s_height_kf_p01 = 0.0f;                         /* KF 协方差 P01：高度-速度 */
static float s_height_kf_p02 = 0.0f;                         /* KF 协方差 P02：高度-偏置 */
static float s_height_kf_p11 = HEIGHT_EST_KF_P_V0;           /* KF 协方差 P11：速度-速度 */
static float s_height_kf_p12 = 0.0f;                         /* KF 协方差 P12：速度-偏置 */
static float s_height_kf_p22 = HEIGHT_EST_KF_P_B0;           /* KF 协方差 P22：偏置-偏置 */
static uint8 s_height_est_ready = 0U;                        /* 高度估计器是否已完成首帧初始化 */
static uint8 s_predict_hold_cnt = 0U;                        /* 无有效 TOF 观测时的预测保持计数 */
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
 * 函数功能：限制 KF 内部高度状态范围。
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
 * 函数功能：复位高度 KF 协方差矩阵。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
static void HeightEst_ResetKfCovariance(void)
{
    s_height_kf_p00 = HEIGHT_EST_KF_P_H0;
    s_height_kf_p01 = 0.0f;
    s_height_kf_p02 = 0.0f;
    s_height_kf_p11 = HEIGHT_EST_KF_P_V0;
    s_height_kf_p12 = 0.0f;
    s_height_kf_p22 = HEIGHT_EST_KF_P_B0;
}

/*
 * 函数功能：用首个有效 TOF 高度初始化 KF 高度、速度、偏置和协方差状态。
 * 输入参数：
 *   height_mm：首帧有效融合高度，单位 mm。
 * 返回值：
 *   无
 */
static void HeightEst_InitKfState(float height_mm)
{
    s_height_est_mm = HeightEst_ClampStateHeightMm(height_mm);
    s_height_est_vz_mps = 0.0f;
    s_height_acc_bias_up_mps2 = 0.0f;
    s_height_acc_lpf_up_mps2 = 0.0f;
    s_height_acc_lpf_ready = 0U;
    HeightEst_ResetKfCovariance();
}

/*
 * 函数功能：对称死区处理，保留超过死区后的有效加速度。
 * 输入参数：
 *   value：待处理数值。
 *   deadband：死区宽度。
 * 返回值：
 *   去除死区后的数值。
 */
static float HeightEst_DeadbandFloat(float value, float deadband)
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

/*
 * 函数功能：根据残差大小计算控制速度观测器的 TOF 校正权重。
 * 输入参数：
 *   residual_m：TOF 高度观测减预测高度的残差，单位 m。
 * 返回值：
 *   残差权重，范围 0~1。
 */
static float HeightEst_OutputVzResidualWeight(float residual_m)
{
    float abs_residual_m = fabsf(residual_m);

    if (abs_residual_m >= HEIGHT_EST_VZ_OBS_RES_HARD_M)
    {
        return 0.0f;
    }
    if (abs_residual_m > HEIGHT_EST_VZ_OBS_RES_SOFT_M)
    {
        return (HEIGHT_EST_VZ_OBS_RES_HARD_M - abs_residual_m) /
            (HEIGHT_EST_VZ_OBS_RES_HARD_M - HEIGHT_EST_VZ_OBS_RES_SOFT_M);
    }

    return 1.0f;
}

/*
 * 函数功能：使用四路 TOF 偏置补偿后的截尾均值生成控制速度观测高度。
 * 输入参数：
 *   tof_height_mm：四路姿态补偿后的 TOF 高度，单位 mm。
 *   tof_valid：四路 TOF 有效标志。
 *   meas_height_m：输出观测高度，单位 m。
 * 返回值：
 *   1=观测有效，0=观测无效。
 */
static uint8 HeightEst_BuildOutputVzMeasure(const float *tof_height_mm, const uint8 *tof_valid, float *meas_height_m)
{
    float sample[VL53L1X_SENSOR_COUNT];
    float center_mm;
    float sum_mm = 0.0f;
    float temp_mm;
    uint8 sample_count = 0U;
    uint8 accept_count = 0U;
    uint8 index;
    uint8 sort_index;

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        if (0U != tof_valid[index])
        {
            sample[sample_count++] = tof_height_mm[index] - s_tof_height_bias_mm[index];
        }
    }

    if (sample_count < 3U)
    {
        return 0U;
    }

    for (index = 0U; index < sample_count; index++)
    {
        for (sort_index = (uint8)(index + 1U); sort_index < sample_count; sort_index++)
        {
            if (sample[sort_index] < sample[index])
            {
                temp_mm = sample[index];
                sample[index] = sample[sort_index];
                sample[sort_index] = temp_mm;
            }
        }
    }

    if (0U != (sample_count & 1U))
    {
        center_mm = sample[sample_count >> 1U];
    }
    else
    {
        center_mm = 0.5f * (sample[(sample_count >> 1U) - 1U] + sample[sample_count >> 1U]);
    }

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
        *meas_height_m = center_mm * 0.001f;
        return 1U;
    }

    *meas_height_m = (sum_mm / (float)accept_count) * 0.001f;
    return 1U;
}

/*
 * 函数功能：更新控制环使用的高度速度观测器，保持高度输出不受加速度接管。
 * 输入参数：
 *   tof_height_mm：四路姿态补偿后的 TOF 高度，单位 mm。
 *   tof_valid：四路 TOF 有效标志。
 *   fused_valid：高度融合有效标志，1=有效，0=无效。
 * 返回值：
 *   控制环使用的高度速度，单位 m/s，上升为正。
 */
static float HeightEst_UpdateOutputVz(const float *tof_height_mm, const uint8 *tof_valid, uint8 fused_valid)
{
    float meas_height_m = 0.0f;
    float acc_up_mps2 = 0.0f;
    float acc_corr_mps2 = 0.0f;
    float residual_m = 0.0f;
    float weight = 0.0f;
    float delta_v_mps = 0.0f;
    uint8 meas_valid;

    if (0U == fused_valid)
    {
        LPF1_Reset(&s_height_out_vz_lpf);
        s_height_out_z_m = 0.0f;
        s_height_out_vz_mps = 0.0f;
        s_height_out_vz_ready = 0U;
        s_height_out_miss_cnt = 0U;
        return 0.0f;
    }

    meas_valid = HeightEst_BuildOutputVzMeasure(tof_height_mm, tof_valid, &meas_height_m);
    if ((0U == s_height_out_vz_ready) && (0U != meas_valid))
    {
        s_height_out_z_m = meas_height_m;
        s_height_out_vz_mps = 0.0f;
        s_height_out_vz_ready = 1U;
        s_height_out_miss_cnt = 0U;
        LPF1_Reset(&s_height_out_vz_lpf);
        return 0.0f;
    }

    if (0U == s_height_out_vz_ready)
    {
        return 0.0f;
    }

    if (0U != AccelCalibration_IsRealtimeDataValid())
    {
        acc_up_mps2 = AccelCalibration_GetVerticalAccelUpMps2();
        acc_up_mps2 = HeightEst_ClampFloat(acc_up_mps2, -HEIGHT_EST_VZ_OBS_ACC_CLIP, HEIGHT_EST_VZ_OBS_ACC_CLIP);
        acc_corr_mps2 = HeightEst_DeadbandFloat(acc_up_mps2, HEIGHT_EST_VZ_OBS_ACC_DEADBAND);
    }

    s_height_out_z_m += s_height_out_vz_mps * HEIGHT_EST_TOF_DT_S +
        0.5f * acc_corr_mps2 * HEIGHT_EST_TOF_DT_S * HEIGHT_EST_TOF_DT_S;
    s_height_out_vz_mps = HEIGHT_EST_VZ_OBS_LEAK_ALPHA *
        (s_height_out_vz_mps + acc_corr_mps2 * HEIGHT_EST_TOF_DT_S);

    if (0U != meas_valid)
    {
        residual_m = meas_height_m - s_height_out_z_m;
        weight = HeightEst_OutputVzResidualWeight(residual_m);
        s_height_out_z_m += HEIGHT_EST_VZ_OBS_ALPHA * weight * residual_m;
        delta_v_mps = (HEIGHT_EST_VZ_OBS_BETA / HEIGHT_EST_TOF_DT_S) * weight * residual_m;
        delta_v_mps = HeightEst_ClampFloat(delta_v_mps, -HEIGHT_EST_VZ_OBS_DV_LIMIT, HEIGHT_EST_VZ_OBS_DV_LIMIT);
        s_height_out_vz_mps += delta_v_mps;

        if (weight > 0.0f)
        {
            s_height_out_miss_cnt = 0U;
        }
        else if (s_height_out_miss_cnt < HEIGHT_EST_VZ_OBS_MISS_MAX)
        {
            s_height_out_miss_cnt++;
        }
        else
        {
            s_height_out_z_m = meas_height_m;
            s_height_out_vz_mps *= HEIGHT_EST_VEL_DECAY;
        }
    }
    else if (s_height_out_miss_cnt < HEIGHT_EST_VZ_OBS_MISS_MAX)
    {
        s_height_out_miss_cnt++;
    }
    else
    {
        s_height_out_vz_mps *= HEIGHT_EST_VEL_DECAY;
    }

    s_height_out_vz_mps = HeightEst_ClampFloat(s_height_out_vz_mps,
        -HEIGHT_EST_VZ_OBS_LIMIT_MPS, HEIGHT_EST_VZ_OBS_LIMIT_MPS);

    return LPF1_Update(&s_height_out_vz_lpf, s_height_out_vz_mps);
}

/*
 * 函数功能：根据可用 TOF 数量和样本离散度估算观测噪声。
 * 输入参数：
 *   count：参与融合的 TOF 数量。
 *   spread_mm：参与融合的 TOF 样本最大离散度，单位 mm。
 * 返回值：
 *   观测噪声标准差，单位 m。
 */
static float HeightEst_MeasureNoiseM(uint8 count, float spread_mm)
{
    float noise_m = HEIGHT_EST_KF_SIGMA_H;
    float spread_m = spread_mm * 0.001f;
    float scale = 1.0f;

    if (count < 3U)
    {
        noise_m = fmaxf(noise_m, 0.055f);
    }

    if (spread_m > HEIGHT_EST_KF_SPREAD_BAD_M)
    {
        scale = spread_m / HEIGHT_EST_KF_SPREAD_BAD_M;
        scale = HeightEst_ClampFloat(scale, 1.0f, 2.0f);
        noise_m = fmaxf(noise_m, HEIGHT_EST_KF_SIGMA_H_BAD * scale);
    }

    return noise_m;
}

/*
 * 函数功能：使用 1kHz 垂直加速度推进高度 KF 状态。
 * 输入参数：
 *   dt_s：预测周期，单位 s。
 * 返回值：
 *   无
 */
static void HeightEst_KfPredict(float dt_s)
{
    float acc_up_mps2;
    float acc_corr_mps2;
    float f[3][3];
    float p[3][3];
    float fp[3][3];
    float pn[3][3];
    float sigma_a2;
    uint8 i;
    uint8 j;
    uint8 k;

    if (0U == s_height_est_ready)
    {
        return;
    }

    if (0U == AccelCalibration_IsRealtimeDataValid())
    {
        return;
    }

    acc_up_mps2 = AccelCalibration_GetVerticalAccelUpMps2();
    acc_up_mps2 = HeightEst_ClampFloat(acc_up_mps2, -HEIGHT_EST_ACC_CLIP_MPS2, HEIGHT_EST_ACC_CLIP_MPS2);

    if (0U == s_height_acc_lpf_ready)
    {
        s_height_acc_lpf_up_mps2 = acc_up_mps2;
        s_height_acc_lpf_ready = 1U;
    }
    else
    {
        s_height_acc_lpf_up_mps2 += HEIGHT_EST_ACC_LPF_ALPHA * (acc_up_mps2 - s_height_acc_lpf_up_mps2);
    }

    acc_corr_mps2 = s_height_acc_lpf_up_mps2 - s_height_acc_bias_up_mps2;
    s_height_est_mm = HeightEst_ClampStateHeightMm(s_height_est_mm +
        s_height_est_vz_mps * dt_s * 1000.0f +
        0.5f * acc_corr_mps2 * dt_s * dt_s * 1000.0f);
    s_height_est_vz_mps = HeightEst_ClampFloat(s_height_est_vz_mps + acc_corr_mps2 * dt_s,
        -HEIGHT_EST_VEL_LIMIT_MPS, HEIGHT_EST_VEL_LIMIT_MPS);

    f[0][0] = 1.0f;
    f[0][1] = dt_s;
    f[0][2] = -0.5f * dt_s * dt_s;
    f[1][0] = 0.0f;
    f[1][1] = 1.0f;
    f[1][2] = -dt_s;
    f[2][0] = 0.0f;
    f[2][1] = 0.0f;
    f[2][2] = 1.0f;

    p[0][0] = s_height_kf_p00;
    p[0][1] = s_height_kf_p01;
    p[0][2] = s_height_kf_p02;
    p[1][0] = s_height_kf_p01;
    p[1][1] = s_height_kf_p11;
    p[1][2] = s_height_kf_p12;
    p[2][0] = s_height_kf_p02;
    p[2][1] = s_height_kf_p12;
    p[2][2] = s_height_kf_p22;

    for (i = 0U; i < 3U; i++)
    {
        for (j = 0U; j < 3U; j++)
        {
            fp[i][j] = 0.0f;
            for (k = 0U; k < 3U; k++)
            {
                fp[i][j] += f[i][k] * p[k][j];
            }
        }
    }

    for (i = 0U; i < 3U; i++)
    {
        for (j = 0U; j < 3U; j++)
        {
            pn[i][j] = 0.0f;
            for (k = 0U; k < 3U; k++)
            {
                pn[i][j] += fp[i][k] * f[j][k];
            }
        }
    }

    sigma_a2 = HEIGHT_EST_KF_SIGMA_A * HEIGHT_EST_KF_SIGMA_A;
    pn[0][0] += sigma_a2 * dt_s * dt_s * dt_s * dt_s * 0.25f;
    pn[0][1] += sigma_a2 * dt_s * dt_s * dt_s * 0.5f;
    pn[1][0] = pn[0][1];
    pn[1][1] += sigma_a2 * dt_s * dt_s;
    pn[2][2] += HEIGHT_EST_KF_SIGMA_B * HEIGHT_EST_KF_SIGMA_B * dt_s;

    s_height_kf_p00 = fmaxf(pn[0][0], 1.0e-8f);
    s_height_kf_p01 = 0.5f * (pn[0][1] + pn[1][0]);
    s_height_kf_p02 = 0.5f * (pn[0][2] + pn[2][0]);
    s_height_kf_p11 = fmaxf(pn[1][1], 1.0e-8f);
    s_height_kf_p12 = 0.5f * (pn[1][2] + pn[2][1]);
    s_height_kf_p22 = fmaxf(pn[2][2], 1.0e-8f);
}

/*
 * 函数功能：使用 100Hz TOF 融合高度校正 KF 高度、速度和加速度偏置。
 * 输入参数：
 *   z_meas_mm：TOF 融合观测高度，单位 mm。
 *   meas_noise_m：TOF 观测噪声标准差，单位 m。
 * 返回值：
 *   无
 */
static void HeightEst_KfCorrect(float z_meas_mm, float meas_noise_m)
{
    float residual_m = z_meas_mm * 0.001f - s_height_est_mm * 0.001f;
    float s = s_height_kf_p00 + meas_noise_m * meas_noise_m;
    float k0;
    float k1;
    float k2;
    float old_p00;
    float old_p01;
    float old_p02;
    float delta_v_mps;
    float delta_b_mps2;

    if (s <= 1.0e-8f)
    {
        return;
    }

    k0 = s_height_kf_p00 / s;
    k1 = s_height_kf_p01 / s;
    k2 = s_height_kf_p02 / s;
    old_p00 = s_height_kf_p00;
    old_p01 = s_height_kf_p01;
    old_p02 = s_height_kf_p02;

    s_height_est_mm = HeightEst_ClampStateHeightMm(s_height_est_mm + k0 * residual_m * 1000.0f);
    delta_v_mps = HeightEst_ClampFloat(k1 * residual_m,
        -HEIGHT_EST_KF_VEL_CORR_LIMIT, HEIGHT_EST_KF_VEL_CORR_LIMIT);
    delta_b_mps2 = HeightEst_ClampFloat(k2 * residual_m,
        -HEIGHT_EST_KF_BIAS_CORR_LIMIT, HEIGHT_EST_KF_BIAS_CORR_LIMIT);

    s_height_est_vz_mps = HeightEst_ClampFloat(s_height_est_vz_mps + delta_v_mps,
        -HEIGHT_EST_VEL_LIMIT_MPS, HEIGHT_EST_VEL_LIMIT_MPS);
    s_height_acc_bias_up_mps2 = HeightEst_ClampFloat(s_height_acc_bias_up_mps2 + delta_b_mps2,
        -HEIGHT_EST_BIAS_LIMIT_MPS2, HEIGHT_EST_BIAS_LIMIT_MPS2);

    s_height_kf_p00 = fmaxf((1.0f - k0) * old_p00, 1.0e-8f);
    s_height_kf_p01 = (1.0f - k0) * old_p01;
    s_height_kf_p02 = (1.0f - k0) * old_p02;
    s_height_kf_p11 = fmaxf(s_height_kf_p11 - k1 * old_p01, 1.0e-8f);
    s_height_kf_p12 = s_height_kf_p12 - k1 * old_p02;
    s_height_kf_p22 = fmaxf(s_height_kf_p22 - k2 * old_p02, 1.0e-8f);
}

/*
 * 函数功能：复位指定 TOF 通道的中值滤波与步进限幅状态。
 * 输入参数：
 *   index：TOF 通道索引，范围 0~3。
 * 返回值：
 *   无
 */
static void HeightEst_ResetChannel(uint8 index)
{
    Median_Reset(&s_tof_median[index]);
    StepLim_Reset(&s_tof_step[index]);
}

/*
 * 函数功能：复位高度估计模块的全部滤波、KF 和输出状态。
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
    }

    s_height_est_mm = (float)VL53L1X_VALID_RANGE_MAX;
    s_height_est_vz_mps = 0.0f;
    LPF1_Init(&s_height_out_vz_lpf, HEIGHT_EST_VZ_LPF_ALPHA);
    s_height_out_z_m = 0.0f;
    s_height_out_vz_mps = 0.0f;
    s_height_out_vz_ready = 0U;
    s_height_out_miss_cnt = 0U;
    s_height_acc_bias_up_mps2 = 0.0f;
    s_height_acc_lpf_up_mps2 = 0.0f;
    s_height_acc_lpf_ready = 0U;
    s_height_est_ready = 0U;
    s_predict_hold_cnt = 0U;
    HeightEst_ResetKfCovariance();

    g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_tof_fused_valid = 0U;
    g_tof_fused_vz_mps = 0.0f;
    g_height_fused_vz_mps = 0.0f;
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
 * 函数功能：根据观测高度与预测高度残差计算 TOF 权重。
 * 输入参数：
 *   valid：TOF 有效标志，1=有效，0=无效。
 *   residual_mm：观测高度减预测高度的残差，单位 mm。
 * 返回值：
 *   残差权重，范围 0~1。
 */
static float HeightEst_ResidualWeight(uint8 valid, float residual_mm)
{
    float abs_residual_mm;

    if (0U == valid)
    {
        return 0.0f;
    }

    abs_residual_mm = fabsf(residual_mm);
    if (abs_residual_mm >= HEIGHT_EST_RESIDUAL_GATE_MM)
    {
        return 0.0f;
    }

    return 1.0f - abs_residual_mm / HEIGHT_EST_RESIDUAL_GATE_MM;
}

/*
 * 函数功能：计算短数组的中位数，并会原地排序输入数组。
 * 输入参数：
 *   values：待排序的高度数组，单位 mm。
 *   count：数组有效元素数量。
 * 返回值：
 *   中位数高度，单位 mm。
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
 * 函数功能：100Hz 更新 TOF 原始测距，执行鲁棒融合并校正高度 KF。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void TOF_update_100HZ(void)
{
    const VL53L1X_data_struct *tof_data = 0;
    uint8 index = 0U;
    uint8 sample_count = 0U;
    uint8 valid_count = 0U;
    uint8 meas_valid = 0U;
    uint8 hard_relock = 0U;
    uint8 accept_count = 0U;
    float h_pred_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float weighted_sum = 0.0f;
    float weight_sum = 0.0f;
    float center_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float sample_spread_mm = 0.0f;
    float meas_noise_m = HEIGHT_EST_KF_SIGMA_H;
    float tof_height_mm[VL53L1X_SENSOR_COUNT] = {
        (float)VL53L1X_INVALID_DISTANCE_MM,
        (float)VL53L1X_INVALID_DISTANCE_MM,
        (float)VL53L1X_INVALID_DISTANCE_MM,
        (float)VL53L1X_INVALID_DISTANCE_MM
    };
    float q[VL53L1X_SENSOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    float center_buf[VL53L1X_SENSOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8 tof_valid[VL53L1X_SENSOR_COUNT] = {0U, 0U, 0U, 0U};
    float z_meas_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float log_tof1_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float log_tof2_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float log_tof3_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float log_tof4_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float acc_z_mps2 = 0.0f;
    float output_vz_mps = 0.0f;
    float deviation_mm = 0.0f;
    float robust_weight = 1.0f;

    VL53L1X_Update();
    tof_data = VL53L1X_GetData();

    g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_valid = 0U;

    if (0U != s_height_est_ready)
    {
        h_pred_mm = s_height_est_mm;
    }

    if (0 != tof_data)
    {
        if ((0U != tof_data->valid[HEIGHT_EST_TOF1_INDEX]) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF1_INDEX] >= HEIGHT_EST_TOF_VALID_MIN_MM) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF1_INDEX] < HEIGHT_EST_TOF_VALID_MAX_MM))
        {
            tof_height_mm[HEIGHT_EST_TOF1_INDEX] = HeightEst_ProcessChannel(HEIGHT_EST_TOF1_INDEX, tof_data->distance_mm[HEIGHT_EST_TOF1_INDEX]);
            tof_valid[HEIGHT_EST_TOF1_INDEX] = 1U;
        }
        else
        {
            HeightEst_ResetChannel(HEIGHT_EST_TOF1_INDEX);
        }

        if ((0U != tof_data->valid[HEIGHT_EST_TOF2_INDEX]) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF2_INDEX] >= HEIGHT_EST_TOF_VALID_MIN_MM) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF2_INDEX] < HEIGHT_EST_TOF_VALID_MAX_MM))
        {
            tof_height_mm[HEIGHT_EST_TOF2_INDEX] = HeightEst_ProcessChannel(HEIGHT_EST_TOF2_INDEX, tof_data->distance_mm[HEIGHT_EST_TOF2_INDEX]);
            tof_valid[HEIGHT_EST_TOF2_INDEX] = 1U;
        }
        else
        {
            HeightEst_ResetChannel(HEIGHT_EST_TOF2_INDEX);
        }

        if ((0U != tof_data->valid[HEIGHT_EST_TOF3_INDEX]) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF3_INDEX] >= HEIGHT_EST_TOF_VALID_MIN_MM) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF3_INDEX] < HEIGHT_EST_TOF_VALID_MAX_MM))
        {
            tof_height_mm[HEIGHT_EST_TOF3_INDEX] = HeightEst_ProcessChannel(HEIGHT_EST_TOF3_INDEX, tof_data->distance_mm[HEIGHT_EST_TOF3_INDEX]);
            tof_valid[HEIGHT_EST_TOF3_INDEX] = 1U;
        }
        else
        {
            HeightEst_ResetChannel(HEIGHT_EST_TOF3_INDEX);
        }

        if ((0U != tof_data->valid[HEIGHT_EST_TOF4_INDEX]) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF4_INDEX] >= HEIGHT_EST_TOF_VALID_MIN_MM) &&
            ((float)tof_data->distance_mm[HEIGHT_EST_TOF4_INDEX] < HEIGHT_EST_TOF_VALID_MAX_MM))
        {
            tof_height_mm[HEIGHT_EST_TOF4_INDEX] = HeightEst_ProcessChannel(HEIGHT_EST_TOF4_INDEX, tof_data->distance_mm[HEIGHT_EST_TOF4_INDEX]);
            tof_valid[HEIGHT_EST_TOF4_INDEX] = 1U;
        }
        else
        {
            HeightEst_ResetChannel(HEIGHT_EST_TOF4_INDEX);
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

    if (0U != s_height_est_ready)
    {
        for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
        {
            if (0U != tof_valid[index])
            {
                q[index] = HeightEst_ResidualWeight(1U, tof_height_mm[index] - h_pred_mm);
                valid_count++;
                if (q[index] > 0.0f)
                {
                    center_buf[sample_count++] = tof_height_mm[index];
                }
            }
        }

        if (sample_count >= 2U)
        {
            center_mm = HeightEst_MedianMm(center_buf, sample_count);
            sample_spread_mm = center_buf[sample_count - 1U] - center_buf[0U];

            for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
            {
                if (q[index] > 0.0f)
                {
                    deviation_mm = fabsf(tof_height_mm[index] - center_mm);
                    robust_weight = 1.0f;

                    if (deviation_mm > HEIGHT_EST_HUBER_K_MM)
                    {
                        robust_weight = HEIGHT_EST_HUBER_K_MM / deviation_mm;
                    }

                    q[index] *= robust_weight;
                    weighted_sum += q[index] * tof_height_mm[index];
                    weight_sum += q[index];
                    accept_count++;
                }
            }
        }
    }
    else
    {
        for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
        {
            if (0U != tof_valid[index])
            {
                center_buf[sample_count++] = tof_height_mm[index];
                valid_count++;
            }
        }

        if (sample_count > 0U)
        {
            center_mm = HeightEst_MedianMm(center_buf, sample_count);
            sample_spread_mm = center_buf[sample_count - 1U] - center_buf[0U];

            for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
            {
                if (0U != tof_valid[index])
                {
                    deviation_mm = fabsf(tof_height_mm[index] - center_mm);
                    robust_weight = 1.0f;

                    if (deviation_mm > HEIGHT_EST_HUBER_K_MM)
                    {
                        robust_weight = HEIGHT_EST_HUBER_K_MM / deviation_mm;
                    }

                    weighted_sum += robust_weight * tof_height_mm[index];
                    weight_sum += robust_weight;
                }
            }
        }
    }

    if ((weight_sum <= HEIGHT_EST_WEIGHT_EPS) &&
        (valid_count >= 2U) &&
        (s_predict_hold_cnt >= HEIGHT_EST_PREDICT_HOLD_CNT))
    {
        weighted_sum = 0.0f;
        weight_sum = 0.0f;
        sample_count = 0U;

        for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
        {
            if (0U != tof_valid[index])
            {
                center_buf[sample_count++] = tof_height_mm[index];
            }
        }

        if (sample_count > 0U)
        {
            center_mm = HeightEst_MedianMm(center_buf, sample_count);
            sample_spread_mm = center_buf[sample_count - 1U] - center_buf[0U];

            if (sample_spread_mm <= HEIGHT_EST_HARD_RELOCK_SPREAD_MM)
            {
                hard_relock = 1U;

                for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
                {
                    if (0U != tof_valid[index])
                    {
                        deviation_mm = fabsf(tof_height_mm[index] - center_mm);
                        robust_weight = 1.0f;

                        if (deviation_mm > HEIGHT_EST_HUBER_K_MM)
                        {
                            robust_weight = HEIGHT_EST_HUBER_K_MM / deviation_mm;
                        }

                        weighted_sum += robust_weight * tof_height_mm[index];
                        weight_sum += robust_weight;
                    }
                }
            }
        }
    }

    if (weight_sum > HEIGHT_EST_WEIGHT_EPS)
    {
        z_meas_mm = weighted_sum / weight_sum;
        meas_valid = 1U;
    }
    else
    {
        z_meas_mm = h_pred_mm;
    }

    if (0U == s_height_est_ready)
    {
        if (0U == meas_valid)
        {
            g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
            g_tof_fused_vz_mps = 0.0f;
            g_height_fused_vz_mps = 0.0f;
            return;
        }

        s_height_est_mm = HeightEst_ClampHeightMm(z_meas_mm);
        HeightEst_InitKfState(s_height_est_mm);
        s_height_est_ready = 1U;
        s_predict_hold_cnt = 0U;
        g_tof_fused_valid = 1U;
    }
    else if (0U != meas_valid)
    {
        if (0U != hard_relock)
        {
            s_height_est_mm = HeightEst_ClampStateHeightMm(z_meas_mm);
            s_height_est_vz_mps = HeightEst_ClampFloat(s_height_est_vz_mps,
                -HEIGHT_EST_HARD_RELOCK_VEL_LIMIT, HEIGHT_EST_HARD_RELOCK_VEL_LIMIT);
            HeightEst_ResetKfCovariance();
        }
        else
        {
            meas_noise_m = HeightEst_MeasureNoiseM(accept_count, sample_spread_mm);
            HeightEst_KfCorrect(z_meas_mm, meas_noise_m);
        }

        s_predict_hold_cnt = 0U;
        g_tof_fused_valid = 1U;
    }
    else
    {
        s_height_est_mm = HeightEst_ClampStateHeightMm(h_pred_mm);

        if (s_predict_hold_cnt < HEIGHT_EST_PREDICT_HOLD_CNT)
        {
            s_predict_hold_cnt++;
            g_tof_fused_valid = 1U;
        }
        else
        {
            g_tof_fused_valid = 0U;
            s_height_est_vz_mps *= HEIGHT_EST_VEL_DECAY;
        }
    }

    acc_z_mps2 = AccelCalibration_GetAccelDownForOutputMps2();
    g_tof_fused_height_mm = s_height_est_mm;
    output_vz_mps = HeightEst_UpdateOutputVz(tof_height_mm, tof_valid, g_tof_fused_valid);
    g_tof_fused_vz_mps = output_vz_mps;
    g_height_fused_vz_mps = output_vz_mps;
    log_tof1_height_mm = HeightEst_ClampHeightMm(g_tof1_height_mm);
    log_tof2_height_mm = HeightEst_ClampHeightMm(g_tof2_height_mm);
    log_tof3_height_mm = HeightEst_ClampHeightMm(g_tof3_height_mm);
    log_tof4_height_mm = HeightEst_ClampHeightMm(g_tof4_height_mm);
    // /* 仅发送四路姿态解耦后的高度，1300 mm 视为无效，离线再重算融合参数 */
    // FC_START_CRSF_state_e fc_state = FC_START_CRSF_Get_State();
    // if (fc_state == FC_START_CRSF_STATE_FLYING)
    // {
    //     wifi_justfloat(
    //         tick_1000us_cnt,
    //         log_tof1_height_mm, log_tof2_height_mm, log_tof3_height_mm, log_tof4_height_mm,
    //         g_euler.roll, g_euler.pitch, acc_z_mps2, g_tof_fused_vz_mps,g_tof_fused_height_mm
    //     );
    // }
}

/*
 * 函数功能：100Hz 高度估计入口，更新 TOF 融合观测和 KF 校正。
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
 * 函数功能：1kHz 高度估计入口，使用 IMU 垂直加速度预测高度 KF。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void Height_Est_update_1000HZ(void)
{
    HeightEst_KfPredict(HEIGHT_EST_IMU_DT_S);
}
