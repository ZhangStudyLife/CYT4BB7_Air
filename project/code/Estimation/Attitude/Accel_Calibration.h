/********************************************************************
 * 文件名  : Accel_Calibration.h
 * 说明    : ICM42688 加速度计校准与惯性加速度预处理
 *
 * ======================== 串口校准命令一览 ========================
 *
 *   cal status       — 查看当前校准状态（是否已校准、模式、进度）
 *   cal dump         — 打印当前运行时参数和Flash中已保存的参数
 *   cal load         — 从Flash加载校准参数并应用（上电自动执行一次）
 *   cal save         — 将当前运行时校准参数写入Flash（掉电保存）
 *   cal clear        — 擦除Flash中的校准数据（恢复出厂）
 *   cal gyro start   — 仅启动陀螺仪零偏校准（需静止放置）
 *   cal acc6 start   — 仅启动6面加速度计校准（需依次放6个面朝上）
 *   cal all start    — 依次执行陀螺仪校准 + 6面加速度计校准
 *   cal ellip start  — 启动椭球拟合加速度计校准（推荐，见下方详细说明）
 *
 * ======================== 校准方式选择 ========================
 *
 * 【方式一】传统6面校准（cal all start 或 cal acc6 start）
 *   - 需要将飞机精确对齐6个面（top/bottom/left/right/front/back）
 *   - 对摆放精度要求较高（偏斜>6°可能失败）
 *   - 仅校准3轴偏置+3轴比例因子，不补偿交叉轴耦合
 *
 * 【方式二】椭球拟合校准（cal ellip start）【推荐】
 *   - 无需精确对齐任何轴，随意放置≥12个不同朝向即可
 *   - 对摆放精度要求极低，只要每个朝向之间角度>15°
 *   - 校准结果为 3×3 校正矩阵 + 3D偏置，天然补偿交叉轴耦合
 *   - 精度优于6面校准
 *
 * ======================== 椭球拟合校准操作流程 ========================
 *
 * 1. 串口发送: cal ellip start
 *    → 串口回复: cal,ellip,start
 *
 * 2. 将飞机静止放置在某个朝向（随意倾斜均可）
 *    → 保持静止约 3.25 秒（1.25秒稳定 + 2秒采集）
 *    → 串口打印: cal,ellip,collecting,N    — 开始采集第N个朝向
 *    → 串口打印: cal,ellip,orient_done,N,x,y,z — 第N个朝向采集完成
 *
 * 3. 拿起飞机，换一个朝向放下，重复步骤2
 *    → 每次朝向需与已有朝向夹角>15°，否则提示 orient_too_close
 *    → 建议覆盖各个方向：正放、倒扣、左右侧、前后倾、各种斜放
 *
 * 4. 采集满12个有效朝向后自动触发求解
 *    → 串口打印: cal,ellip,solving,12
 *    → 求解成功: cal,ellip,ok,bias,...  — 校准完成！
 *    → 求解失败: cal,ellip,solve_fail_retry — 需继续添加更多朝向
 *
 * 5. 校准成功后发送: cal save  — 将结果写入Flash掉电保存
 *
 * ======================== 椭球校准注意事项 ========================
 *
 * - 每个朝向采集期间必须完全静止，振动会导致该朝向被拒绝(orient_noisy)
 * - 朝向越多越好（12~20个），覆盖越均匀精度越高
 * - 建议在桌面、地面等稳固平面上操作，避免手持
 * - 超时时间10分钟（600秒），超时未完成会打印 cal,ellip,timeout
 * - 如求解多次失败，检查是否有朝向过于集中（如全是水平放置的变体）
 * - 校准成功后务必执行 cal save，否则掉电丢失
 * - 可用 cal dump 查看校准结果：bias应<0.35g，M对角线应在0.85~1.15
 *
 * ======================== Flash与版本兼容 ========================
 *
 * - Flash存储支持V1（6面校准，对角scale）和V2（椭球拟合，3×3矩阵）
 * - V1数据加载后自动转为对角矩阵模式，不影响正常使用
 * - V2数据包含完整3×3校正矩阵，精度更高
 * - cal load 自动识别版本，无需手动区分
 *
 * ======================== 常用串口输出含义 ========================
 *
 * cal,loaded_v1_as_diag   — Flash中读到V1格式数据，已转为对角矩阵模式
 * cal,loaded              — 校准数据加载成功
 * cal,ellip,collecting,N  — 正在采集第N个朝向
 * cal,ellip,orient_done,N — 第N个朝向采集完成
 * cal,ellip,orient_noisy  — 该朝向采集期间振动过大，已丢弃
 * cal,ellip,orient_too_close — 该朝向与已有朝向太接近（<15°），需换方向
 * cal,ellip,solving,N     — 开始用N个朝向求解椭球
 * cal,ellip,ok            — 椭球拟合校准成功
 * cal,ellip,solve_fail    — 求解失败
 * cal,ellip,timeout       — 10分钟超时
 * cal,busy                — 校准正在进行中，拒绝新命令
 *
 * ======================== 坐标系与极性规范 ========================
 *
 * 1. 机体系定义 FRD：+X 前，+Y 右，+Z 下。
 * 2. 加速度计标定：+ax 向前加速，+ay 向右加速，+az 向下加速。
 *    静止平放时 az 符号由 ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN 设置，
 *    当前默认按 az=-1g 处理。
 * 3. 角速度：gx>0 右滚，gy>0 抬头，gz>0 机头顺时针。
 * 4. 导航 Down 加速度（NED）：GetAccelDown* 为 Down 正方向。
 * 5. 导航 Up 加速度：GetVerticalAccelUpMps2 为 Up 正方向， Up = -Down。
 * 6. IMU 传感器轴→机体转换：v_body = R_imu_to_body * v_imu。
 ********************************************************************/

#ifndef ACCEL_CALIBRATION_H_
#define ACCEL_CALIBRATION_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ACCEL_CALIBRATION_GRAVITY_MSS            (9.80665f)
#define ACCEL_CALIBRATION_DT_S                   (0.001f)  /* 实时更新周期，单位秒 */
#define ACCEL_CALIBRATION_SAMPLES                (1000U)  /* 加速度静止标定目标有效样本数，1kHz约1秒 */

#define ACCEL_CALIBRATION_STD_G_WARN_MAX         (0.050f)
#define ACCEL_CALIBRATION_STD_G_FAIL_MAX         (0.080f)

/* 加速度静止比力约定
 * +1.0f: 静止平放 az≈+1g
 * -1.0f: 静止平放 az≈-1g
 */
#define ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN (-1.0f)

/* 水平旋转是否使用 yaw：
 * 0U: 仅 roll/pitch，忽略当前航向
 * 1U: roll/pitch/yaw 全旋转
 */
#define ACCEL_CALIBRATION_LEVEL_USE_YAW          (0U)

/* 机体轴符号标记（用于可读性，勿随意修改） */
#define ACCEL_CALIBRATION_BODY_AXIS_X_FORWARD    (+1.0f)
#define ACCEL_CALIBRATION_BODY_AXIS_Y_RIGHT      (+1.0f)
#define ACCEL_CALIBRATION_BODY_AXIS_Z_DOWN       (+1.0f)
#define ACCEL_CALIBRATION_OUTPUT_DOWN_SIGN       (+1.0f)
#define ACCEL_CALIBRATION_OUTPUT_UP_SIGN         (-1.0f)

typedef struct
{
    bool is_calibrated;
    uint16_t sample_count;

    /* 机体系校准参数
     * use_full_matrix==1: corrected = accel_corr_matrix * (raw - bias)
     * use_full_matrix==0: corrected[i] = (raw[i] - bias[i]) * accel_scale[i]  (legacy)
     */
    float accel_bias_g[3];
    float accel_scale[3];
    float accel_corr_matrix[3][3];
    uint8_t use_full_matrix;

    /* 机体系原始值/校正值 */
    float accel_raw_body_g[3];
    float accel_corrected_body_g[3];
    float gyro_raw_body_dps[3];

    /* 机体系线性加速度（去重力后），单位 m/s^2 */
    float accel_real_body_mps2[3];

    /* 导航水平系线性加速度（去重力后），默认仅旋转 roll/pitch */
    float accel_level_mps2[3];

    /* 向下加速度输出 */
    float accel_down_for_ekf_mps2;
    float accel_down_for_output_mps2;

    /* 垂直积分状态，Up 为正向 */
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
    float accel_corr_matrix[3][3];
    uint8_t use_full_matrix;
    float imu_to_body[3][3];
    float gravity_mps2;
} AccelCalibrationParams_t;

#define IMU_CALIB_FLASH_MAGIC                (0x43414C49UL)
#define IMU_CALIB_FLASH_VERSION_V1           (1U)
#define IMU_CALIB_FLASH_VERSION              (2U)
#define IMU_CALIB_FLASH_PAGE                 (95U)

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    float gyro_bias_dps[3];
    float accel_bias_g[3];
    float accel_corr_matrix[3][3];   /* v2: full 3x3 correction matrix; v1 legacy: diagonal = scale */
    float imu_to_body[3][3];

    uint32_t reserved[2];
} IMUCalibBlob_t;

extern AccelCalibration_t g_accel_calibration;

void AccelCalibration_Init(void);
void AccelCalibration_Reset(void);
bool AccelCalibration_Start(void);
void AccelCalibration_ApplySensorCorrection(float *ax, float *ay, float *az);
void AccelCalibration_Update_1000HZ(void); /* 1kHz 实时更新入口 */

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

/* 机体系线性加速度（去重力后），静止应接近 0,0,0 */
void AccelCalibration_GetBodyAccelMps2(float *ax, float *ay, float *az);
/* 机体系陀螺仪角速度，单位 dps */
void AccelCalibration_GetBodyGyroDps(float *gx, float *gy, float *gz);
/* 机体系校准后比力，单位 g，未去重力（供姿态解算使用） */
void AccelCalibration_GetCorrectedSpecificForceG(float *ax_g, float *ay_g, float *az_g);

/* 导航水平系线性加速度（去重力后） */
void AccelCalibration_GetLevelAccelMps2(float *ax_level, float *ay_level, float *az_level);
void AccelCalibration_GetHorizontalAccelMps2(float *ax_h, float *ay_h);

bool AccelCalibration_LoadParams(const AccelCalibrationParams_t *params);
void AccelCalibration_GetParams(AccelCalibrationParams_t *params);

void IMUCalib_Init(void);
void IMUCalib_Update_1000HZ(void); /* 1kHz IMU 校准状态机更新入口 */
void IMUCalib_CommandPoll(void);
uint8_t IMUCalib_LoadFromFlashAndApply(void);
uint8_t IMUCalib_SaveCurrentToFlash(void);
uint8_t IMUCalib_ClearFlash(void);
uint8_t IMUCalib_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* ACCEL_CALIBRATION_H_ */

