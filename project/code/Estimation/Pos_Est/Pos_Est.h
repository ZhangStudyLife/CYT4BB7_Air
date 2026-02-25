#ifndef POS_EST_H
#define POS_EST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 加速度极性 */
#ifndef POS_EST_ACCELERATION_X_SIGN
#define POS_EST_ACCELERATION_X_SIGN      (+1.0f)
#endif
#ifndef POS_EST_ACCELERATION_Y_SIGN
#define POS_EST_ACCELERATION_Y_SIGN      (+1.0f)
#endif

/* 陀螺极性 */
#ifndef POS_EST_GYRO_X_SIGN
#define POS_EST_GYRO_X_SIGN              (+1.0f)
#endif
#ifndef POS_EST_GYRO_Y_SIGN
#define POS_EST_GYRO_Y_SIGN              (+1.0f)
#endif

/* 光流极性 */
#ifndef POS_EST_FLOW_SWAP_XY
/* Mount mapping: body_x <- -raw_delta_y, body_y <- -raw_delta_x */
#define POS_EST_FLOW_SWAP_XY            (1U)
#endif
#ifndef POS_EST_FLOW_X_SIGN
#define POS_EST_FLOW_X_SIGN              (-1.0f)
#endif
#ifndef POS_EST_FLOW_Y_SIGN
#define POS_EST_FLOW_Y_SIGN              (-1.0f)
#endif

#ifndef POS_EST_FLOW_GYRO_SWAP_XY
/* Keep gyro compensation in the same mapped frame as flow deltas */
#define POS_EST_FLOW_GYRO_SWAP_XY       (1U)
#endif
#ifndef POS_EST_FLOW_GYRO_X_SIGN
#define POS_EST_FLOW_GYRO_X_SIGN         (-1.0f)
#endif
#ifndef POS_EST_FLOW_GYRO_Y_SIGN
#define POS_EST_FLOW_GYRO_Y_SIGN         (-1.0f)
#endif

/* 光流旋转解耦矩阵（flow_translation = flow_rate - K * gyro） */
#ifndef POS_EST_GYRO_DEC_K11
#define POS_EST_GYRO_DEC_K11             (0.949415f)
#endif
#ifndef POS_EST_GYRO_DEC_K12
#define POS_EST_GYRO_DEC_K12             (-0.133459f)
#endif
#ifndef POS_EST_GYRO_DEC_K21
#define POS_EST_GYRO_DEC_K21             (-0.091574f)
#endif
#ifndef POS_EST_GYRO_DEC_K22
#define POS_EST_GYRO_DEC_K22             (-0.935116f)
#endif

/* 单位换算 */
#ifndef POS_EST_DEG_TO_RAD
#define POS_EST_DEG_TO_RAD               (0.01745329251994f)
#endif
#ifndef POS_EST_FLOW_RADIANS_PER_COUNT
#define POS_EST_FLOW_RADIANS_PER_COUNT   (0.00126f)
#endif

/* 固定调度周期 */
#ifndef POS_EST_DT_2KHZ_SECONDS
#define POS_EST_DT_2KHZ_SECONDS          (0.0005f)
#endif
#ifndef POS_EST_DT_200HZ_SECONDS
#define POS_EST_DT_200HZ_SECONDS         (0.005f)
#endif

/* 200Hz更新函数实际被调用的周期（当前工程建议放在5ms任务） */
#ifndef POS_EST_FLOW_CALL_DT_SECONDS
#define POS_EST_FLOW_CALL_DT_SECONDS     (POS_EST_DT_200HZ_SECONDS)
#endif

/* 光流有效时间窗口，避免长时间无新帧后一次性爆发 */
#ifndef POS_EST_FLOW_VALID_DT_MIN_SECONDS
#define POS_EST_FLOW_VALID_DT_MIN_SECONDS (0.002f)
#endif
#ifndef POS_EST_FLOW_VALID_DT_MAX_SECONDS
#define POS_EST_FLOW_VALID_DT_MAX_SECONDS (0.060f)
#endif

/* 光流速度换算用高度低通，减少姿态引起的高度抖动放大 */
#ifndef POS_EST_FLOW_HEIGHT_LPF_TAU_SECONDS
#define POS_EST_FLOW_HEIGHT_LPF_TAU_SECONDS (0.20f)
#endif
#ifndef POS_EST_FLOW_HEIGHT_MIN_M
#define POS_EST_FLOW_HEIGHT_MIN_M         (0.05f)
#endif
#ifndef POS_EST_FLOW_HEIGHT_MAX_M
#define POS_EST_FLOW_HEIGHT_MAX_M         (1.50f)
#endif

/* 光流门限 */
#ifndef POS_EST_FLOW_SQUAL_MIN
#define POS_EST_FLOW_SQUAL_MIN           (20U)
#endif
#ifndef POS_EST_FLOW_DELTA_MAX
#define POS_EST_FLOW_DELTA_MAX           (200)
#endif
#ifndef POS_EST_FLOW_TRANSLATION_LIMIT_RPS
#define POS_EST_FLOW_TRANSLATION_LIMIT_RPS (1.5f)
#endif
#ifndef POS_EST_FLOW_TRANSLATION_LPF_TAU_SECONDS
#define POS_EST_FLOW_TRANSLATION_LPF_TAU_SECONDS (0.03f)
#endif

/* 残差与状态限幅，避免异常帧导致发散 */
#ifndef POS_EST_POSITION_RESIDUAL_LIMIT_M
#define POS_EST_POSITION_RESIDUAL_LIMIT_M (0.25f)
#endif
#ifndef POS_EST_VELOCITY_LIMIT_MPS
#define POS_EST_VELOCITY_LIMIT_MPS       (20.0f)
#endif
#ifndef POS_EST_ACCELERATION_LIMIT_MPS2
#define POS_EST_ACCELERATION_LIMIT_MPS2  (50.0f)
#endif

/* 静止判定与自稳参数 */
#ifndef POS_EST_STATIC_ACCEL_THRESHOLD_MPS2
#define POS_EST_STATIC_ACCEL_THRESHOLD_MPS2   (0.08f)
#endif
#ifndef POS_EST_STATIC_GYRO_THRESHOLD_DPS
#define POS_EST_STATIC_GYRO_THRESHOLD_DPS     (2.0f)
#endif
#ifndef POS_EST_ACCEL_BIAS_LEARN_TC_SECONDS
#define POS_EST_ACCEL_BIAS_LEARN_TC_SECONDS   (8.0f)
#endif
#ifndef POS_EST_ACCEL_DEADBAND_MPS2
#define POS_EST_ACCEL_DEADBAND_MPS2           (0.004f)
#endif
#ifndef POS_EST_NO_FLOW_STATIC_HOLD_DELAY_SECONDS
#define POS_EST_NO_FLOW_STATIC_HOLD_DELAY_SECONDS (0.25f)
#endif
#ifndef POS_EST_STATIC_VELOCITY_DAMP_TC_SECONDS
#define POS_EST_STATIC_VELOCITY_DAMP_TC_SECONDS (2.0f)
#endif
#ifndef POS_EST_STATIC_VELOCITY_ZERO_MPS
#define POS_EST_STATIC_VELOCITY_ZERO_MPS      (0.02f)
#endif

/* α-β-γ 参数 */
#ifndef POS_EST_VELOCITY_INPUT_VALID_LPF_TAU_SECONDS
#define POS_EST_VELOCITY_INPUT_VALID_LPF_TAU_SECONDS  (0.03f)
#endif
#ifndef POS_EST_VELOCITY_INPUT_LOST_LPF_TAU_SECONDS
#define POS_EST_VELOCITY_INPUT_LOST_LPF_TAU_SECONDS   (0.30f)
#endif
#ifndef POS_EST_VELOCITY_INPUT_FLOW_FRESH_SECONDS
#define POS_EST_VELOCITY_INPUT_FLOW_FRESH_SECONDS     (0.2f)
#endif
#ifndef POS_EST_ALPHA
#define POS_EST_ALPHA                    (0.80f)
#endif
#ifndef POS_EST_BETA
#define POS_EST_BETA                     (0.12f)
#endif
#ifndef POS_EST_GAMMA
#define POS_EST_GAMMA                    (0.01f)
#endif

/* 默认关闭 gamma 更新，保持加速度由IMU直接给出，避免加速度状态发散 */
#ifndef POS_EST_ENABLE_GAMMA_UPDATE
#define POS_EST_ENABLE_GAMMA_UPDATE      (0U)
#endif

/* flow_gate_state 含义 */
#define POS_EST_FLOW_GATE_PASS            (0U)
#define POS_EST_FLOW_GATE_HEIGHT_INVALID  (1U)
#define POS_EST_FLOW_GATE_VALID_FLAG_ZERO (2U)
#define POS_EST_FLOW_GATE_SQUAL_LOW       (3U)
#define POS_EST_FLOW_GATE_DELTA_OVERFLOW  (4U)
#define POS_EST_FLOW_GATE_DT_INVALID      (5U)

typedef struct
{
    float velocity_x_mps;
    float velocity_y_mps;
    uint8_t valid; 
} velocity_input_100hz_t;


typedef struct
{
    float position_x_m;
    float position_y_m;
    float velocity_x_mps;
    float velocity_y_mps;
    float acceleration_x_mps2;
    float acceleration_y_mps2;
    float height_m;
    uint8_t flow_update_valid;
} pos_est_output_t;

typedef struct
{
    float acceleration_input_x_mps2;
    float acceleration_input_y_mps2;
    float acceleration_bias_x_mps2;
    float acceleration_bias_y_mps2;
    float body_acceleration_x_mps2;
    float body_acceleration_y_mps2;
    float body_gyro_x_dps;
    float body_gyro_y_dps;
    uint8_t is_static_state;
    float flow_missing_seconds;

    float flow_dt_seconds;
    float flow_height_used_m;
    int16_t raw_delta_x_count;
    int16_t raw_delta_y_count;
    uint8_t raw_squal;
    uint8_t flow_gate_state;
    float flow_mapped_delta_x_count;
    float flow_mapped_delta_y_count;

    float flow_rate_x_rps;
    float flow_rate_y_rps;
    float flow_gyro_x_rps;
    float flow_gyro_y_rps;
    float flow_translation_x_rps;
    float flow_translation_y_rps;
    float measured_velocity_x_mps;
    float measured_velocity_y_mps;
    float position_residual_x_m;
    float position_residual_y_m;
} pos_est_debug_t;

extern volatile pos_est_output_t g_pos_est_output;
extern volatile pos_est_debug_t g_pos_est_debug;
extern volatile velocity_input_100hz_t g_velocity_input_100hz;

void pos_est_init(void);
void pos_est_update_2khz(void);
void pos_est_update_200hz(void);

#ifdef __cplusplus
}
#endif

#endif
