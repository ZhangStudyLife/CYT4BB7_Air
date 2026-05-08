/********************************************************************
 * 文件名  : Accel_Calibration.h
 * 说明    : ICM42688 加速度计校准、重力去除与 IMU 校准接口声明
 * 约束    : 当前对外保留两类 IMU 校准能力
 *           1. 角速度静态校准
 *           2. 加速度椭球校准
 ********************************************************************/

#ifndef ACCEL_CALIBRATION_H_
#define ACCEL_CALIBRATION_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ACCEL_CALIBRATION_GRAVITY_MSS            (9.80665f) /* 标准重力加速度，单位 m/s^2 */
#define ACCEL_CALIBRATION_DT_S                   (0.001f)   /* 实时更新周期，单位 s */
#define ACCEL_CALIBRATION_SAMPLES                (1000U)    /* 静止标定目标样本数，按 1kHz 约为 1s */

#define ACCEL_CALIBRATION_STD_G_WARN_MAX         (0.050f)   /* 静止质量告警阈值，单位 g */
#define ACCEL_CALIBRATION_STD_G_FAIL_MAX         (0.080f)   /* 静止质量失败阈值，单位 g */

/* 静止平放时的比力方向约定 */
#define ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN (-1.0f) /* 平放静止时默认 az≈-1g */

/* 水平系补偿时是否显式使用 yaw */
#define ACCEL_CALIBRATION_LEVEL_USE_YAW          (0U)       /* 0 仅使用 roll/pitch，1 使用 roll/pitch/yaw */

/* 机体系方向符号约定 */
#define ACCEL_CALIBRATION_BODY_AXIS_X_FORWARD    (+1.0f)    /* 机体系 +X 向前 */
#define ACCEL_CALIBRATION_BODY_AXIS_Y_RIGHT      (+1.0f)    /* 机体系 +Y 向右 */
#define ACCEL_CALIBRATION_BODY_AXIS_Z_DOWN       (+1.0f)    /* 机体系 +Z 向下 */
#define ACCEL_CALIBRATION_OUTPUT_DOWN_SIGN       (+1.0f)    /* Down 输出方向符号 */

typedef struct
{
    bool is_calibrated;
    uint16_t sample_count;

    /* 机体系校准参数 */
    float accel_bias_g[3];
    float accel_scale[3];
    float accel_corr_matrix[3][3];
    uint8_t use_full_matrix;

    /* 机体系原始值与校正值 */
    float accel_raw_body_g[3];
    float accel_corrected_body_g[3];
    float gyro_body_dps[3];

    /* 机体系线性加速度，单位 m/s^2 */
    float accel_real_body_mps2[3];

    /* 水平系线性加速度，单位 m/s^2 */
    float accel_level_mps2[3];

    /* 向下加速度输出 */
    float accel_down_output_mps2;

    /* 垂直积分状态，Up 为正 */
    float vel_up_mps;
    float pos_up_m;

    /* IMU 到机体的旋转矩阵 */
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

#define IMU_CALIB_FLASH_MAGIC                (0x43414C49UL) /* IMU 校准 Flash 魔数 */
#define IMU_CALIB_FLASH_VERSION_V1           (1U)           /* 旧版 Flash 数据版本 */
#define IMU_CALIB_FLASH_VERSION              (2U)           /* 当前 Flash 数据版本 */
#define IMU_CALIB_FLASH_PAGE                 (95U)          /* IMU 校准参数存储页 */

#define IMU_CALIB_STATUS_MODE_IDLE           (0U)           /* IMU 校准空闲模式 */
#define IMU_CALIB_STATUS_MODE_GYRO           (1U)           /* 陀螺仪静态校准模式 */
#define IMU_CALIB_STATUS_MODE_ACC6           (2U)           /* 六面加速度校准模式 */
#define IMU_CALIB_STATUS_MODE_ALL            (3U)           /* 组合校准模式 */
#define IMU_CALIB_STATUS_MODE_ELLIP          (4U)           /* 自动椭球加速度校准模式 */
#define IMU_CALIB_STATUS_MODE_ELLIP_MANUAL   (5U)           /* 手动椭球加速度校准模式 */

#define IMU_CALIB_MANUAL_SUBSTATE_NONE       (0U)           /* 手动校准未进入子状态 */
#define IMU_CALIB_MANUAL_SUBSTATE_READY      (1U)           /* 手动校准准备态 */
#define IMU_CALIB_MANUAL_SUBSTATE_WAIT_STATIC (2U)          /* 手动校准等待静止 */
#define IMU_CALIB_MANUAL_SUBSTATE_COLLECTING (3U)           /* 手动校准采集当前姿态点 */
#define IMU_CALIB_MANUAL_SUBSTATE_SOLVING    (4U)           /* 手动校准正在求解 */

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    float gyro_bias_dps[3];
    float accel_bias_g[3];
    float accel_corr_matrix[3][3];
    float imu_to_body[3][3];

    uint32_t reserved[2];
} IMUCalibBlob_t;

typedef struct
{
    uint8_t busy;              /* 当前是否正在执行校准 */
    uint8_t mode;              /* 当前校准模式 */
    uint8_t pose_count;        /* 加速度校准已完成的姿态点数量 */
    uint8_t substate;          /* 手动校准子状态 */
    uint32_t progress_percent; /* 当前校准进度百分比 */
    uint32_t current_samples;  /* 当前子阶段累计样本数 */
    uint32_t target_samples;   /* 当前子阶段目标样本数 */
} IMUCalibStatus_t;

typedef struct
{
    uint8_t valid;                 /* Flash 中是否存在有效校准数据 */
    uint8_t use_full_matrix;       /* 1 表示使用完整 3x3 校正矩阵 */
    uint16_t version;              /* Flash 中校准数据版本 */
    float gyro_bias_dps[3];        /* 陀螺仪零偏，单位 dps */
    float accel_bias_g[3];         /* 加速度偏置，单位 g */
    float accel_corr_matrix[3][3]; /* 加速度 3x3 校正矩阵 */
    float imu_to_body[3][3];       /* IMU 到机体的安装旋转矩阵 */
} IMUCalibFlashInfo_t;

typedef void (*IMUCalibTextSink_t)(const char *text);

/* 全局加速度校准运行态对象 */
extern AccelCalibration_t g_accel_calibration;

/* 初始化加速度校准模块 */
void AccelCalibration_Init(void);
/* 复位加速度校准运行态 */
void AccelCalibration_Reset(void);
/* 启动静止校准流程 */
bool AccelCalibration_Start(void);
/* 对原始加速度做安装与校准补偿 */
void AccelCalibration_ApplySensorCorrection(float *ax, float *ay, float *az);
/* 1kHz 更新加速度校准运行态 */
void AccelCalibration_Update_1000HZ(void);

/* 设置 IMU 到机体的旋转矩阵 */
void AccelCalibration_SetImuToBodyMatrix(const float matrix[3][3]);
/* 按欧拉角设置 IMU 到机体的旋转矩阵 */
void AccelCalibration_SetImuToBodyEulerDeg(float roll_deg, float pitch_deg, float yaw_deg);

/* 查询当前是否已有有效校准参数 */
bool AccelCalibration_IsCalibrated(void);
/* 查询当前实时数据是否有效 */
uint8_t AccelCalibration_IsRealtimeDataValid(void);
/* 获取当前重力标称值 */
float AccelCalibration_GetGravityMps2(void);

/* 获取垂直方向 Up 线性加速度 */
float AccelCalibration_GetVerticalAccelUpMps2(void);
/* 获取垂直方向 Up 速度 */
float AccelCalibration_GetVerticalVelocityUpMps(void);
/* 获取垂直方向 Up 位置 */
float AccelCalibration_GetVerticalPositionUpM(void);

/* 获取供输出使用的 Down 加速度 */
float AccelCalibration_GetAccelDownForOutputMps2(void);

/* 获取机体系线性加速度，单位 m/s^2 */
void AccelCalibration_GetBodyAccelMps2(float *ax, float *ay, float *az);
/* 获取机体系角速度，单位 dps */
void AccelCalibration_GetBodyGyroDps(float *gx, float *gy, float *gz);
/* 获取校正后的机体系比力，单位 g */
void AccelCalibration_GetCorrectedSpecificForceG(float *ax_g, float *ay_g, float *az_g);

/* 获取水平面加速度，单位 m/s^2 */
void AccelCalibration_GetHorizontalAccelMps2(float *ax_h, float *ay_h);
/* 将传感器坐标系向量旋转到机体坐标系（供外部使用已滤波加速度时调用） */
void AccelCalibration_RotateImuToBody(const float vec_sensor[3], float vec_body[3]);

/* 加载一组加速度校准参数到运行态 */
bool AccelCalibration_LoadParams(const AccelCalibrationParams_t *params);
/* 读取当前运行态加速度校准参数 */
void AccelCalibration_GetParams(AccelCalibrationParams_t *params);

/* 初始化 IMU 校准子系统并尝试从 Flash 恢复参数 */
void IMUCalib_Init(void);
/* 1kHz 更新 IMU 校准状态机 */
void IMUCalib_Update_1000HZ(void);
/* 从 Flash 读取 IMU 校准参数并应用到当前运行态 */
uint8_t IMUCalib_LoadFromFlashAndApply(void);
/* 将当前 IMU 校准参数保存到 Flash */
uint8_t IMUCalib_SaveCurrentToFlash(void);
/* 将代码内置默认 IMU 加速度校准参数写入 Flash */
uint8_t IMUCalib_ApplyDefaultToFlash(void);
/* 擦除 Flash 中的 IMU 校准参数 */
uint8_t IMUCalib_ClearFlash(void);
/* 查询 IMU 校准是否正在执行 */
uint8_t IMUCalib_IsBusy(void);
/* 设置 IMU 校准过程文本输出回调 */
void IMUCalib_SetTextSink(IMUCalibTextSink_t sink);
/* 读取 Flash 中保存的 IMU 校准参数并转成人类可读结果 */
uint8_t IMUCalib_ReadFlashInfo(IMUCalibFlashInfo_t *info);
/* 启动陀螺仪静态校准 */
uint8_t IMUCalib_StartGyro(void);
/* 启动加速度自动椭球校准 */
uint8_t IMUCalib_StartAccel(void);
/* 启动加速度手动椭球校准准备流程 */
uint8_t IMUCalib_StartAccelManual(void);
/* 在手动椭球模式下触发一次姿态点采集 */
uint8_t IMUCalib_ManualCollect(void);
/* 在手动椭球模式下停止采点并开始求解 */
uint8_t IMUCalib_ManualStop(void);
/* 读取当前 IMU 校准状态 */
void IMUCalib_GetStatus(IMUCalibStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif /* ACCEL_CALIBRATION_H_ */
