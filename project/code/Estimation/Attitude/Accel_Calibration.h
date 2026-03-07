/********************************************************************
 * 文件名  : Accel_Calibration.h
 * 说明    : ICM42688 加速度计校准与垂向加速度预处理
 *
 * 坐标与极性规范（AGENTS.md）：
 * 1. 机体系采用 FRD：+X 前，+Y 右，+Z 下。
 * 2. 加速度计比力：+ax 向前加速，+ay 向右加速，+az 向下加速。
 *    静止平放时的 az 符号由 ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN 配置，
 *    当前默认按 az≈-1g 处理。
 * 3. 陀螺角速度：gx>0 右滚，gy>0 抬头，gz>0 俯视顺时针。
 * 4. 本模块 Down 向输出（NED）：GetAccelDown* 为 Down 正方向。
 * 5. 本模块 Up 向输出：GetVerticalAccelUpMps2 为 Up 正方向，且 Up = -Down。
 * 6. IMU 到机体旋转矩阵语义：v_body = R_imu_to_body * v_imu。
 ********************************************************************/

#ifndef ACCEL_CALIBRATION_H_
#define ACCEL_CALIBRATION_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ACCEL_CALIBRATION_GRAVITY_MSS            (9.80665f)
#define ACCEL_CALIBRATION_DT_S                   (0.0005f)
#define ACCEL_CALIBRATION_SAMPLES                (2000U)

#define ACCEL_CALIBRATION_STD_G_WARN_MAX         (0.050f)
#define ACCEL_CALIBRATION_STD_G_FAIL_MAX         (0.080f)

/* 比力静止符号约定：
 * +1.0f: 静止平放 az≈+1g
 * -1.0f: 静止平放 az≈-1g
 */
#define ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN (-1.0f)

/* 水平化输出是否使用 yaw：
 * 0U: 仅 roll/pitch（按当前需求）
 * 1U: roll/pitch/yaw 全补偿
 */
#define ACCEL_CALIBRATION_LEVEL_USE_YAW          (0U)

/* 方向符号标记（用于可读性与联调检查） */
#define ACCEL_CALIBRATION_BODY_AXIS_X_FORWARD    (+1.0f)
#define ACCEL_CALIBRATION_BODY_AXIS_Y_RIGHT      (+1.0f)
#define ACCEL_CALIBRATION_BODY_AXIS_Z_DOWN       (+1.0f)
#define ACCEL_CALIBRATION_OUTPUT_DOWN_SIGN       (+1.0f)
#define ACCEL_CALIBRATION_OUTPUT_UP_SIGN         (-1.0f)

typedef struct
{
    bool is_calibrated;
    uint16_t sample_count;

    /* 机体系校准参数：corrected = (raw - bias) * scale */
    float accel_bias_g[3];
    float accel_scale[3];

    /* 机体系原始值/校正值 */
    float accel_raw_body_g[3];
    float accel_corrected_body_g[3];
    float gyro_raw_body_dps[3];

    /* 机体系线加速度（去重力后），单位 m/s^2 */
    float accel_real_body_mps2[3];

    /* 地面水平系线加速度（去重力后），默认仅补偿 roll/pitch） */
    float accel_level_mps2[3];

    /* 垂向加速度输出 */
    float accel_down_for_ekf_mps2;
    float accel_down_for_output_mps2;

    /* 垂向积分状态（Up 为正） */
    float vel_up_mps;
    float pos_up_m;

    /* IMU -> Body 旋转矩阵 */
    float imu_to_body[3][3];
    bool imu_to_body_identity;

    /* 运行质量指标 */
    float accel_norm_mean_g;
    float accel_norm_std_g;

    float gravity_mps2;
    uint32_t invalid_sample_count;
    uint8_t realtime_sample_valid;
} AccelCalibration_t;

typedef struct
{
    float accel_bias_g[3];
    float accel_scale[3];
    float imu_to_body[3][3];
    float gravity_mps2;
} AccelCalibrationParams_t;

#define IMU_CALIB_FLASH_MAGIC                (0x43414C49UL)
#define IMU_CALIB_FLASH_VERSION              (1U)
#define IMU_CALIB_FLASH_PAGE                 (95U)

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    float gyro_bias_dps[3];
    float accel_bias_g[3];
    float accel_scale[3];
    float imu_to_body[3][3];

    uint32_t reserved[8];
} IMUCalibBlob_t;

extern AccelCalibration_t g_accel_calibration;

void AccelCalibration_Init(void);
void AccelCalibration_Reset(void);
bool AccelCalibration_Start(void);
void AccelCalibration_Update_2000HZ(void);

void AccelCalibration_SetImuToBodyMatrix(const float matrix[3][3]);
void AccelCalibration_SetImuToBodyEulerDeg(float roll_deg, float pitch_deg, float yaw_deg);

bool AccelCalibration_IsCalibrated(void);
uint8_t AccelCalibration_IsRealtimeDataValid(void);
float AccelCalibration_GetGravityMps2(void);

float AccelCalibration_GetVerticalAccelUpMps2(void);
float AccelCalibration_GetVerticalVelocityUpMps(void);
float AccelCalibration_GetVerticalPositionUpM(void);

float AccelCalibration_GetAccelDownMps2(void);
float AccelCalibration_GetAccelDownForEkfMps2(void);
float AccelCalibration_GetAccelDownForOutputMps2(void);

/* 机体系线加速度（去重力），静止应接近 0,0,0 */
void AccelCalibration_GetBodyAccelMps2(float *ax, float *ay, float *az);
/* 机体系陀螺角速度，单位 dps */
void AccelCalibration_GetBodyGyroDps(float *gx, float *gy, float *gz);
/* 机体系校准后比力（单位 g，未去重力），供姿态解算使用 */
void AccelCalibration_GetCorrectedSpecificForceG(float *ax_g, float *ay_g, float *az_g);

/* 地面水平系线加速度（去重力） */
void AccelCalibration_GetLevelAccelMps2(float *ax_level, float *ay_level, float *az_level);
void AccelCalibration_GetHorizontalAccelMps2(float *ax_h, float *ay_h);

bool AccelCalibration_LoadParams(const AccelCalibrationParams_t *params);
void AccelCalibration_GetParams(AccelCalibrationParams_t *params);

void IMUCalib_Init(void);
void IMUCalib_Update_2000HZ(void);
void IMUCalib_CommandPoll(void);
uint8_t IMUCalib_LoadFromFlashAndApply(void);
uint8_t IMUCalib_SaveCurrentToFlash(void);
uint8_t IMUCalib_ClearFlash(void);
uint8_t IMUCalib_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* ACCEL_CALIBRATION_H_ */

