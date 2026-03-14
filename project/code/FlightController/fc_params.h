/*****************************************************************************
 * 文件: fc_params.h
 * 模块: 飞控 - 参数集中管理
 * 职责: 定义飞控控制周期、油门基准和各控制环参数结构体
 *****************************************************************************/

#ifndef FC_PARAMS_H
#define FC_PARAMS_H

#include <stdint.h>

/* ==================== 飞控参数结构体 ==================== */
typedef struct
{
    /* ===== 控制周期 ===== */
    float gyro_dt;   /* 角速度环控制周期，单位 s */
    float angle_dt;  /* 角度环控制周期，单位 s */
    float pos_xy_dt; /* 水平位置环控制周期，单位 s */
    float pos_z_dt;  /* 垂直位置环控制周期，单位 s */
    float vel_xy_dt; /* 水平速度环控制周期，单位 s */
    float vel_z_dt;  /* 垂直速度环控制周期，单位 s */

    /* ===== 油门参数 ===== */
    int32_t base_throttle; /* 悬停油门基准 */

    /* ===== Roll轴角速度环参数 ===== */
    float roll_gyro_kp;
    float roll_gyro_ki;
    float roll_gyro_kd;
    float roll_gyro_kff;   /* 前馈增益 */
    float roll_gyro_i_limit;
    float roll_gyro_d_lpf;

    /* ===== Pitch轴角速度环参数 ===== */
    float pitch_gyro_kp;
    float pitch_gyro_ki;
    float pitch_gyro_kd;
    float pitch_gyro_kff;  /* 前馈增益 */
    float pitch_gyro_i_limit;
    float pitch_gyro_d_lpf;

    /* ===== Yaw轴角速度环参数 ===== */
    float yaw_gyro_kp;
    float yaw_gyro_ki;
    float yaw_gyro_kd;
    float yaw_gyro_kff;    /* 前馈增益 */
    float yaw_gyro_i_limit;
    float yaw_gyro_d_lpf;

    /* ===== Roll轴角度环参数 ===== */
    float roll_angle_kp;
    float roll_angle_ki;
    float roll_angle_kd;
    float roll_angle_kff;  /* 前馈增益 */
    float roll_angle_i_limit;
    float roll_angle_d_lpf;

    /* ===== Pitch轴角度环参数 ===== */
    float pitch_angle_kp;
    float pitch_angle_ki;
    float pitch_angle_kd;
    float pitch_angle_kff; /* 前馈增益 */
    float pitch_angle_i_limit;
    float pitch_angle_d_lpf;

    /* ===== Yaw轴角度环参数 ===== */
    float yaw_angle_kp;
    float yaw_angle_ki;
    float yaw_angle_kd;
    float yaw_angle_kff;   /* 前馈增益 */
    float yaw_angle_i_limit;
    float yaw_angle_d_lpf;

    /* ===== X轴位置环参数 ===== */
    float pos_x_kp;
    float pos_x_ki;
    float pos_x_kd;
    float pos_x_kff;       /* 前馈增益 */
    float pos_x_i_limit;
    float pos_x_d_lpf;

    /* ===== Y轴位置环参数 ===== */
    float pos_y_kp;
    float pos_y_ki;
    float pos_y_kd;
    float pos_y_kff;       /* 前馈增益 */
    float pos_y_i_limit;
    float pos_y_d_lpf;

    /* ===== Z轴位置环参数 ===== */
    float pos_z_kp;
    float pos_z_ki;
    float pos_z_kd;
    float pos_z_kff;       /* 前馈增益 */
    float pos_z_i_limit;
    float pos_z_d_lpf;

    /* ===== X轴速度环参数 ===== */
    float vel_x_kp;
    float vel_x_ki;
    float vel_x_kd;
    float vel_x_kff;       /* 前馈增益 */
    float vel_x_i_limit;
    float vel_x_d_lpf;

    /* ===== Y轴速度环参数 ===== */
    float vel_y_kp;
    float vel_y_ki;
    float vel_y_kd;
    float vel_y_kff;       /* 前馈增益 */
    float vel_y_i_limit;
    float vel_y_d_lpf;

    /* ===== Z轴速度环参数 ===== */
    float vel_z_kp;
    float vel_z_ki;
    float vel_z_kd;
    float vel_z_kff;       /* 前馈增益 */
    float vel_z_i_limit;
    float vel_z_d_lpf;

    /* ===== 模式1 常调参数 ===== */
    float mode1_track_ff_deg_per_cmps; /* 模式1 跟杆前馈斜率，单位 deg/(cm/s) */
    float mode1_brake_kp;              /* 模式1 刹车阶段速度环 P 增益 */
    float mode1_brake_exit_vel_cmps;   /* 模式1 退出刹车的速度阈值，单位 cm/s */
} fc_params_t;

/* ==================== 全局参数实例 ==================== */
extern fc_params_t g_fc_params;

/* ==================== 初始化函数 ==================== */
void FC_Params_Init(void);

#endif /* FC_PARAMS_H */
