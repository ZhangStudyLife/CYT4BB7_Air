#ifndef POS_EST_H
#define POS_EST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POS_EST_DT_2000HZ_S                    (0.0005f)  /* 2000Hz周期 0.5ms */
#define POS_EST_DT_250HZ_S                     (0.004f)
#define POS_EST_DT_100HZ_S                     (0.010f)

/* 光流姿态解耦补偿系数（角度域，单位 pix/deg）
 * 理论值 K_angle = focal_pixel * deg2rad = 476.2 * (pi/180) = 8.31 pix/deg
 * 实测多批次数据 d=1 延迟补偿最小二乘拟合，主轴系数与理论一致
 * 交叉耦合项(X-pitch, Y-roll)实测值均 < 0.003，已忽略
 * 在2000Hz中累积陀螺仪角度(deg)，100Hz中取前一帧累积值做补偿
 * 补偿 PMW3901 约 10ms 管线延迟 */
#define POS_EST_K_FLOW_DECOUPLE_X_ROLL         (+8.40f)  /* X轴对roll主耦合 (拟合值，理论101.1%) */
#define POS_EST_K_FLOW_DECOUPLE_Y_PITCH        (-8.85f)  /* Y轴对pitch主耦合 (拟合值，理论106.5%) */

/* 补偿后像素域一阶IIR低通滤波系数
 * 作用：抑制补偿后残差中的高频噪声(残差自相关lag=1~2约0.36~0.41)
 * fc ≈ 11.5Hz @100Hz采样率 (alpha=0.50)
 * 群延迟约10ms，对室内悬停可接受
 * 增大alpha→带宽增大、噪声增多；减小alpha→噪声更低、延迟增大 */
#define POS_EST_FLOW_PIX_CORR_LPF_ALPHA        (0.50f)

#define POS_EST_FLOW_GATE_PASS                 (0U)
#define POS_EST_FLOW_GATE_HEIGHT_INVALID       (1U)
#define POS_EST_FLOW_GATE_SQUAL_LOW            (2U)
#define POS_EST_FLOW_GATE_DELTA_OVERFLOW       (3U)
#define POS_EST_FLOW_GATE_DELTA_ZERO           (4U)
#define POS_EST_FLOW_GATE_INNOV_REJECT         (5U)

#define POS_EST_FLOW_SQUAL_MIN                 (20U)
#define POS_EST_FLOW_PIX_MAX                   (100)
#define POS_EST_FLOW_HEIGHT_MIN_M              (0.08f)
#define POS_EST_FLOW_HEIGHT_MAX_M              (2.00f)
#define POS_EST_FLOW_VEL_MAX_MPS               (1.50f)
#define POS_EST_FLOW_VEL_LPF_ALPHA             (0.38f)
#define POS_EST_FLOW_INNOV_GATE_MPS            (2.20f)
#define POS_EST_FLOW_DEAD_MAX_S                (0.30f)
#define POS_EST_FLOW_DEAD_VEL_DAMP_RATIO       (0.985f)

#define POS_EST_TILT_TAN_LIMIT                 (0.80f)
#define POS_EST_K_PIX_TO_M_AT_1M_DEFAULT       (0.00210f)

/* 注: 旧速率域系数已移除，改用角度域系数 POS_EST_K_FLOW_DECOUPLE_*_* */

#define POS_EST_W_V_DEFAULT                    (0.68f)
#define POS_EST_W_P_DEFAULT                    (0.24f) 
#define POS_EST_K_B_DEFAULT                    (0.02f)

#define POS_EST_ACC_LPF_ALPHA_250HZ            (0.08f)
#define POS_EST_ACC_VIBE_RC_250HZ              (0.08f)
#define POS_EST_ACC_WEIGHT_MIN                 (0.25f)
#define POS_EST_ACC_WEIGHT_MAX                 (1.00f)
#define POS_EST_ACC_VIBE_LOW                   (0.20f)
#define POS_EST_ACC_VIBE_HIGH                  (1.80f)
#define POS_EST_CORR_BOOST_MAX                 (2.20f)

#define POS_EST_STATIC_ACCEL_TH_MPS2           (0.10f)
#define POS_EST_STATIC_GYRO_TH_DPS             (2.00f)
#define POS_EST_ACCEL_BIAS_TC_S                (8.00f)
#define POS_EST_ACCEL_BIAS_MAX_MPS2            (1.00f)
#define POS_EST_VELOCITY_MAX_MPS               (3.00f)

typedef struct
{
    float position_x_m;
    float position_y_m;
    float velocity_x_mps;
    float velocity_y_mps;
    float height_m;
    uint8_t flow_valid;
} PosEstOutput_t;

typedef struct
{
    int16_t raw_flow_dx_count;          // flow原始像素增量，向右移动为正 向左移动为负
    int16_t raw_flow_dy_count;          // flow原始像素增量，向前移动为正 向后移动为负
    uint8_t raw_flow_squal;             // flow原始质量值，范围0-255，数值越大质量越好
    uint8_t flow_gate_state;            // flow有效性状态，0为通过，非0为不通过的原因

    float flow_pix_x_corr;
    float flow_pix_y_corr;
    float flow_vx_mps;
    float flow_vy_mps;

    float accel_bias_x_mps2;
    float accel_bias_y_mps2;

    float gyro_filt_y;
    float gyro_filt_x;
} PosEstDebug_t;

extern volatile PosEstOutput_t g_pos_est_output;
extern volatile PosEstDebug_t g_pos_est_debug;

/**
 * @brief  位置估计模块初始化
 *         清零所有状态变量、输出和调试结构体
 */
void Pos_Est_Init(void);

/**
 * @brief  2000Hz陀螺仪角度积分更新
 *         在每个2000Hz周期累积 pitch/roll 轴角位移(deg)，
 *         等效于对角速度做矩形窗平均滤波(20点@2000Hz)，
 *         在100Hz周期中读取并重置累积值，供光流姿态解耦补偿使用
 */
void Pos_Est_Update_2000HZ(void);

/**
 * @brief  250Hz加速度计积分更新
 *         执行加速度偏置估计、振动加权、速度/位置积分
 */
void Pos_Est_Update_250HZ(void);

/**
 * @brief  100Hz光流融合更新
 *         读取光流传感器，进行姿态解耦补偿，与加速度计积分融合
 */
void Pos_Est_Update_100HZ(void);

#ifdef __cplusplus
}
#endif

#endif
