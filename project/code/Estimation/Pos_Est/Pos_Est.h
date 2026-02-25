#ifndef POS_EST_H
#define POS_EST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POS_EST_DT_250HZ_S                     (0.004f)
#define POS_EST_DT_100HZ_S                     (0.010f)

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
#define POS_EST_FLOW_VEL_LPF_ALPHA             (0.20f)
#define POS_EST_FLOW_INNOV_GATE_MPS            (1.80f)
#define POS_EST_FLOW_DEAD_MAX_S                (0.30f)
#define POS_EST_FLOW_DEAD_VEL_DAMP_RATIO       (0.98f)

#define POS_EST_TILT_TAN_LIMIT                 (0.80f)
#define POS_EST_K_PIX_TO_M_AT_1M_DEFAULT       (0.00210f)
#define POS_EST_K_DTILT_X_PITCH_DEFAULT        (432.0f)
#define POS_EST_K_DTILT_X_ROLL_DEFAULT         (7.0f)
#define POS_EST_K_DTILT_Y_PITCH_DEFAULT        (3.0f)
#define POS_EST_K_DTILT_Y_ROLL_DEFAULT         (-439.0f)

#define POS_EST_W_V_DEFAULT                    (0.35f)
#define POS_EST_W_P_DEFAULT                    (0.12f)
#define POS_EST_K_B_DEFAULT                    (0.02f)

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
    int16_t raw_flow_dx_count;
    int16_t raw_flow_dy_count;
    uint8_t raw_flow_squal;
    uint8_t flow_gate_state;

    float flow_pix_x_corr;
    float flow_pix_y_corr;
    float flow_vx_mps;
    float flow_vy_mps;

    float accel_bias_x_mps2;
    float accel_bias_y_mps2;
} PosEstDebug_t;

extern volatile PosEstOutput_t g_pos_est_output;
extern volatile PosEstDebug_t g_pos_est_debug;

void Pos_Est_Init(void);
void Pos_Est_Update_250HZ(void);
void Pos_Est_Update_100HZ(void);

#ifdef __cplusplus
}
#endif

#endif
