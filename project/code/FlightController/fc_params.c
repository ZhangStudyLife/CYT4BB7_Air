/*****************************************************************************
 * 文件名  : fc_params.c
 * 模块    : 飞控 - 参数管理实现
 * 职责    : 定义全局参数变量g_fc_params，初始化所有参数
 *****************************************************************************/

#include "fc_params.h"

/* ==================== 全局参数实例 ==================== */
fc_params_t g_fc_params;

/* ==================== 初始化函数 ==================== */
void FC_Params_Init(void)
{
    /* ===== 控制周期 ===== */
    g_fc_params.gyro_dt = 0.0005f; /* 2kHz */
    g_fc_params.angle_dt = 0.002f; /* 500Hz */
    g_fc_params.pos_xy_dt = 0.01f; /* 100Hz */
    g_fc_params.pos_z_dt = 0.04f;  /* 25Hz */
    g_fc_params.vel_z_dt = 0.02f;  /* 50Hz */

    /* ===== 油门参数 ===== */
    g_fc_params.base_throttle = 2500; /* 悬停油门 */

    /* ===== Roll轴角速度环参数 ===== */
    g_fc_params.roll_gyro_kp = 7.2f;
    g_fc_params.roll_gyro_ki = 4.6f;
    g_fc_params.roll_gyro_kd = 0.018f;
    g_fc_params.roll_gyro_kff = 0.0f;
    g_fc_params.roll_gyro_i_limit = 2200.0f;
    g_fc_params.roll_gyro_d_lpf = 0.12f;

    /* ===== Pitch轴角速度环参数 ===== */
    g_fc_params.pitch_gyro_kp = 7.2f;
    g_fc_params.pitch_gyro_ki = 4.6f;
    g_fc_params.pitch_gyro_kd = 0.018f;
    g_fc_params.pitch_gyro_kff = 0.0f;
    g_fc_params.pitch_gyro_i_limit = 2200.0f;
    g_fc_params.pitch_gyro_d_lpf = 0.12f;

    /* ===== Yaw轴角速度环参数 ===== */
    g_fc_params.yaw_gyro_kp = 10.5f;
    g_fc_params.yaw_gyro_ki = 5.5f;
    g_fc_params.yaw_gyro_kd = 0.0f;
    g_fc_params.yaw_gyro_kff = 0.0f;
    g_fc_params.yaw_gyro_i_limit = 1800.0f;
    g_fc_params.yaw_gyro_d_lpf = 0.18f;

    /* ===== roll轴角度环参数 ===== */
    g_fc_params.roll_angle_kp = 3.3f;
    g_fc_params.roll_angle_ki = 0.10f;
    g_fc_params.roll_angle_kd = 0.0f;
    g_fc_params.roll_angle_kff = 0.0f;
    g_fc_params.roll_angle_i_limit = 110.0f;
    g_fc_params.roll_angle_d_lpf = 0.0f;

    /* ===== pitch轴角度环参数 ===== */
    g_fc_params.pitch_angle_kp = 3.3f;
    g_fc_params.pitch_angle_ki = 0.10f;
    g_fc_params.pitch_angle_kd = 0.0f;
    g_fc_params.pitch_angle_kff = 0.0f;
    g_fc_params.pitch_angle_i_limit = 110.0f;
    g_fc_params.pitch_angle_d_lpf = 0.0f;

    /* ===== yaw轴角度环参数 ===== */
    g_fc_params.yaw_angle_kp = 0.0f;
    g_fc_params.yaw_angle_ki = 0.0f;
    g_fc_params.yaw_angle_kd = 0.0f;
    g_fc_params.yaw_angle_kff = 0.0f;
    g_fc_params.yaw_angle_i_limit = 0.0f;
    g_fc_params.yaw_angle_d_lpf = 0.0f;

    /* ===== X轴的Pos环参数 ===== */
    g_fc_params.pos_x_kp = 12.0f;
    g_fc_params.pos_x_ki = 0.0f;
    g_fc_params.pos_x_kd = 0.0f;
    g_fc_params.pos_x_kff = 0.0f;
    g_fc_params.pos_x_i_limit = 0.8f;
    g_fc_params.pos_x_d_lpf = 0.0f;

    /* ===== Y轴的Pos环参数 ===== */
    g_fc_params.pos_y_kp = 6.0f;
    g_fc_params.pos_y_ki = 0.0f;
    g_fc_params.pos_y_kd = 0.0f;
    g_fc_params.pos_y_kff = 0.0f;
    g_fc_params.pos_y_i_limit = 0.8f;
    g_fc_params.pos_y_d_lpf = 0.0f;

    /* ===== Z轴的Pos环参数 ===== */
    g_fc_params.pos_z_kp = 0.9f;
    g_fc_params.pos_z_ki = 0.03f;
    g_fc_params.pos_z_kd = 0.0f;
    g_fc_params.pos_z_kff = 0.0f;
    g_fc_params.pos_z_i_limit = 400.0f;
    g_fc_params.pos_z_d_lpf = 0.0f;

    /* ===== z轴的Vel环参数 ===== */
    g_fc_params.vel_z_kp = 1.0f;
    g_fc_params.vel_z_ki = 0.25f;
    g_fc_params.vel_z_kd = 0.0f;
    g_fc_params.vel_z_kff = 0.0f;
    g_fc_params.vel_z_i_limit = 1800.0f;
    g_fc_params.vel_z_d_lpf = 0.0f;
}

// /* ==================== 初始化函数 ==================== */
// void FC_Params_Init(void)
// {

//     /* ===== 控制周期 ===== */
//     g_fc_params.gyro_dt = 0.0005f;   /* 2kHz */
//     g_fc_params.angle_dt = 0.002f;   /* 500Hz */

//     /* ===== Roll轴角速度环 ===== */
//     g_fc_params.roll_gyro_kp = 6.0f;
//     g_fc_params.roll_gyro_ki = 4.0f;
//     g_fc_params.roll_gyro_kd = 0.0f;
//     g_fc_params.roll_gyro_kff = 0.06f;           /* 前馈增益 */
//     g_fc_params.roll_gyro_i_limit = 2000.0f;
//     g_fc_params.roll_gyro_d_lpf = 0.5f;

//     /* ===== Pitch轴角速度环 ===== */
//     g_fc_params.pitch_gyro_kp = 6.0f;
//     g_fc_params.pitch_gyro_ki = 4.0f;
//     g_fc_params.pitch_gyro_kd = 0.0f;
//     g_fc_params.pitch_gyro_kff = 0.06f;          /* 前馈增益 */
//     g_fc_params.pitch_gyro_i_limit = 2000.0f;
//     g_fc_params.pitch_gyro_d_lpf = 0.5f;

//     /* ===== Yaw轴角速度环 ===== */
//     g_fc_params.yaw_gyro_kp = 6.0f;
//     g_fc_params.yaw_gyro_ki = 4.0f;
//     g_fc_params.yaw_gyro_kd = 0.0f;
//     g_fc_params.yaw_gyro_kff = 0.0f;            /* 前馈增益（暂不启用） */
//     g_fc_params.yaw_gyro_i_limit = 2000.0f;
//     g_fc_params.yaw_gyro_d_lpf = 0.5f;

//     /* ===== 角度环 ===== */
//     g_fc_params.angle_kp = 6.0f;
//     g_fc_params.angle_kd = 0.04f;                /* D增益（微分先行） */
//     g_fc_params.angle_d_lpf = 0.8f;             /* D滤波系数（1.0=不滤波） */
//     g_fc_params.max_gyro = 1000.0f;

//     /* ===== 油门 ===== */
//     g_fc_params.base_throttle = 2500;   /* 基础油门，用于调试 */

//     /* ===== 控制量限幅 ===== */
//     g_fc_params.gyro_output_limit = 10000.0f;

// }

// 以下参数低油门很硬 , 高油门抖动厉害
// /* ==================== 初始化函数 ==================== */
// void FC_Params_Init(void)
// {

//     /* ===== 控制周期 ===== */
//     g_fc_params.gyro_dt = 0.0005f;   /* 2kHz */
//     g_fc_params.angle_dt = 0.002f;   /* 500Hz */

//     /* ===== Roll轴角速度环 ===== */
//     g_fc_params.roll_gyro_kp = 18.0f;
//     g_fc_params.roll_gyro_ki = 10.0f;
//     g_fc_params.roll_gyro_kd = 0.0f;
//     g_fc_params.roll_gyro_kff = 0.06f;           /* 前馈增益 */
//     g_fc_params.roll_gyro_i_limit = 2000.0f;
//     g_fc_params.roll_gyro_d_lpf = 0.5f;

//     /* ===== Pitch轴角速度环 ===== */
//     g_fc_params.pitch_gyro_kp = 18.0f;
//     g_fc_params.pitch_gyro_ki = 10.0f;
//     g_fc_params.pitch_gyro_kd = 0.0f;
//     g_fc_params.pitch_gyro_kff = 0.06f;          /* 前馈增益 */
//     g_fc_params.pitch_gyro_i_limit = 2000.0f;
//     g_fc_params.pitch_gyro_d_lpf = 0.5f;

//     /* ===== Yaw轴角速度环 ===== */
//     g_fc_params.yaw_gyro_kp = 20.0f;
//     g_fc_params.yaw_gyro_ki = 15.0f;
//     g_fc_params.yaw_gyro_kd = 0.0f;
//     g_fc_params.yaw_gyro_kff = 0.0f;            /* 前馈增益（暂不启用） */
//     g_fc_params.yaw_gyro_i_limit = 2000.0f;
//     g_fc_params.yaw_gyro_d_lpf = 0.5f;

//     /* ===== 角度环 ===== */
//     g_fc_params.angle_kp = 8.0f;
//     g_fc_params.angle_kd = 0.06f;                /* D增益（微分先行） */
//     g_fc_params.angle_d_lpf = 0.8f;             /* D滤波系数（1.0=不滤波） */
//     g_fc_params.max_gyro = 1000.0f;

//     /* ===== 油门 ===== */
//     g_fc_params.base_throttle = 4200;   /* 基础油门，用于调试 */

//     /* ===== 控制量限幅 ===== */
//     g_fc_params.gyro_output_limit = 10000.0f;

// }
