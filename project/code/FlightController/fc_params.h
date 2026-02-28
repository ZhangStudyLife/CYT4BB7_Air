/*****************************************************************************
 * 文件名  : fc_params.h
 * 模块    : 飞控 - 参数集中管理
 * 职责    : 所有飞控参数用变量而非宏，方便运行时调整
 *****************************************************************************/

#ifndef FC_PARAMS_H
#define FC_PARAMS_H

#include <stdint.h>

/* ==================== 飞控参数结构体 ==================== */
typedef struct
{
    /* ===== 控制周期 ===== */
    float gyro_dt;  /* 角速度环控制周期 */
    float angle_dt; /* 角度环控制周期 */
    float pos_xy_dt; /* 空间位置环控制周期 */
    float pos_z_dt;  /* 空间位置环控制周期 */
    float vel_z_dt; /* 空间速度环控制周期 */

    /* ===== 油门参数 ===== */
    int32_t base_throttle; /* 悬停油门 */

    /* ===== Roll轴角速度环参数 ===== */
    float roll_gyro_kp;
    float roll_gyro_ki;
    float roll_gyro_kd;
    float roll_gyro_kff; /* 前馈增益 */
    float roll_gyro_i_limit;
    float roll_gyro_d_lpf;

    /* ===== Pitch轴角速度环参数 ===== */
    float pitch_gyro_kp;
    float pitch_gyro_ki;
    float pitch_gyro_kd;
    float pitch_gyro_kff; /* 前馈增益 */
    float pitch_gyro_i_limit;
    float pitch_gyro_d_lpf;

    /* ===== Yaw轴角速度环参数 ===== */
    float yaw_gyro_kp;
    float yaw_gyro_ki;
    float yaw_gyro_kd;
    float yaw_gyro_kff; /* 前馈增益 */
    float yaw_gyro_i_limit;
    float yaw_gyro_d_lpf;

    /* ===== roll轴角度环参数 ===== */
    float roll_angle_kp;
    float roll_angle_ki;
    float roll_angle_kd;
    float roll_angle_kff;   /* 前馈增益 */
    float roll_angle_i_limit;
    float roll_angle_d_lpf; /* Roll角度环D项滤波系数（1.0=不滤波） */

    /* ===== pitch轴角度环参数 ===== */
    float pitch_angle_kp;
    float pitch_angle_ki;
    float pitch_angle_kd;
    float pitch_angle_kff;   /* 前馈增益 */
    float pitch_angle_i_limit;
    float pitch_angle_d_lpf; /* Pitch角度环D项滤波系数（1.0=不滤波） */

    /* ===== yaw轴角度环参数 ===== */
    float yaw_angle_kp;
    float yaw_angle_ki;
    float yaw_angle_kd;
    float yaw_angle_kff;   /* 前馈增益 */
    float yaw_angle_i_limit;
    float yaw_angle_d_lpf; /* Yaw角度环D项滤波系数（1.0=不滤波） */

    /* ===== X轴的Pos环参数 ===== */
    float pos_x_kp;
    float pos_x_ki;
    float pos_x_kd;
    float pos_x_kff;      /* 前馈增益 */
    float pos_x_i_limit;
    float pos_x_d_lpf;

    /* ===== Y轴的Pos环参数 ===== */
    float pos_y_kp;
    float pos_y_ki;
    float pos_y_kd;
    float pos_y_kff;      /* 前馈增益 */
    float pos_y_i_limit;
    float pos_y_d_lpf;

    /* ===== Z轴的Pos环参数 ===== */
    float pos_z_kp;
    float pos_z_ki;
    float pos_z_kd;
    float pos_z_kff;      /* 前馈增益 */
    float pos_z_i_limit;
    float pos_z_d_lpf;


    /* ===== z轴的Vel环参数 ===== */
    float vel_z_kp;
    float vel_z_ki;
    float vel_z_kd;
    float vel_z_kff;      /* 前馈增益 */
    float vel_z_i_limit;
    float vel_z_d_lpf;

} fc_params_t;

/* ==================== 全局参数实例 ==================== */
extern fc_params_t g_fc_params;

/* ==================== 初始化函数 ==================== */
void FC_Params_Init(void);

#endif /* FC_PARAMS_H */
