/*****************************************************************************
 * 文件: fc_params.c
 * 模块: 飞控 - 参数管理实现
 * 职责: 定义全局参数变量 g_fc_params，并初始化所有控制参数
 *****************************************************************************/

#include "fc_params.h"

/* ==================== 全局参数实例 ==================== */
/* 飞控参数全局实例，集中保存控制周期、油门基准和各控制环PID参数 */
fc_params_t g_fc_params;

/*
 * 函数名: FC_Params_Init
 * 功能: 初始化飞控参数默认值
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Params_Init(void)
{
    /* ===== 控制周期参数 ===== */
    g_fc_params.gyro_dt = 0.001f; /* 1kHz */
    g_fc_params.angle_dt = 0.002f; /* 500Hz */
    g_fc_params.pos_xy_dt = 0.02f; /* 50Hz */
    g_fc_params.pos_z_dt = 0.02f;  /* 50Hz */
    g_fc_params.vel_xy_dt = 0.02f;  /* 50Hz */
    g_fc_params.vel_z_dt = 0.01f;  /* 100Hz */

    /* ===== 油门参数 ===== */
    g_fc_params.base_throttle = 3500; /* 悬停油门 */

    /* ===== Roll轴角速度环参数 ===== */
    g_fc_params.roll_gyro_kp = 3.0f;
    g_fc_params.roll_gyro_ki = 0.8f;
    g_fc_params.roll_gyro_kd = 0.030f;
    g_fc_params.roll_gyro_kff = 0.0f;
    g_fc_params.roll_gyro_i_limit = 300.0f;
    g_fc_params.roll_gyro_d_lpf = 0.12f;

    /* ===== Pitch轴角速度环参数 ===== */
    g_fc_params.pitch_gyro_kp = 3.0f;
    g_fc_params.pitch_gyro_ki = 0.8f;
    g_fc_params.pitch_gyro_kd = 0.030f;
    g_fc_params.pitch_gyro_kff = 0.0f;
    g_fc_params.pitch_gyro_i_limit = 300.0f;
    g_fc_params.pitch_gyro_d_lpf = 0.12f;

    /* ===== Yaw轴角速度环参数 ===== */
    g_fc_params.yaw_gyro_kp = 14.0f;
    g_fc_params.yaw_gyro_ki = 8.0f;
    g_fc_params.yaw_gyro_kd = 0.0f;
    g_fc_params.yaw_gyro_kff = 0.0f;
    g_fc_params.yaw_gyro_i_limit = 1800.0f;
    g_fc_params.yaw_gyro_d_lpf = 0.18f;

    /* ===== Roll轴角度环参数 ===== */
    g_fc_params.roll_angle_kp = 5.0f;
    g_fc_params.roll_angle_ki = 0.04f;
    g_fc_params.roll_angle_kd = 0.0f;
    g_fc_params.roll_angle_kff = 0.0f;
    g_fc_params.roll_angle_i_limit = 110.0f;
    g_fc_params.roll_angle_d_lpf = 0.0f;

    /* ===== Pitch轴角度环参数 ===== */
    g_fc_params.pitch_angle_kp = 5.0f;
    g_fc_params.pitch_angle_ki = 0.04f;
    g_fc_params.pitch_angle_kd = 0.0f;
    g_fc_params.pitch_angle_kff = 0.0f;
    g_fc_params.pitch_angle_i_limit = 110.0f;
    g_fc_params.pitch_angle_d_lpf = 0.0f;

    /* ===== Yaw轴角度环参数 ===== */
    g_fc_params.yaw_angle_kp = 0.0f;
    g_fc_params.yaw_angle_ki = 0.0f;
    g_fc_params.yaw_angle_kd = 0.0f;
    g_fc_params.yaw_angle_kff = 0.0f;
    g_fc_params.yaw_angle_i_limit = 0.0f;
    g_fc_params.yaw_angle_d_lpf = 0.0f;

    /* ===== X轴位置环参数 ===== */
    g_fc_params.pos_x_kp = 0.90f;
    g_fc_params.pos_x_ki = 0.0f;
    g_fc_params.pos_x_kd = 0.0f;
    g_fc_params.pos_x_kff = 0.0f;
    g_fc_params.pos_x_i_limit = 0.0f;
    g_fc_params.pos_x_d_lpf = 0.0f;

    /* ===== Y轴位置环参数 ===== */
    g_fc_params.pos_y_kp = 0.90f;
    g_fc_params.pos_y_ki = 0.0f;
    g_fc_params.pos_y_kd = 0.0f;
    g_fc_params.pos_y_kff = 0.0f;
    g_fc_params.pos_y_i_limit = 0.0f;
    g_fc_params.pos_y_d_lpf = 0.0f;

    /* ===== Z轴位置环参数 ===== */
    g_fc_params.pos_z_kp = 0.8f;
    g_fc_params.pos_z_ki = 0.0f;
    g_fc_params.pos_z_kd = 0.06f;
    g_fc_params.pos_z_kff = 0.0f;
    g_fc_params.pos_z_i_limit = 0.0f;
    g_fc_params.pos_z_d_lpf = 0.20f;

    /* ===== X轴速度环参数 ===== */
    g_fc_params.vel_x_kp = 0.12f;
    g_fc_params.vel_x_ki = 0.02f;
    g_fc_params.vel_x_kd = 0.0f;
    g_fc_params.vel_x_kff = 0.0f;
    g_fc_params.vel_x_i_limit = 3.0f;
    g_fc_params.vel_x_d_lpf = 0.0f;

    /* ===== Y轴速度环参数 ===== */
    g_fc_params.vel_y_kp = 0.12f;
    g_fc_params.vel_y_ki = 0.02f;
    g_fc_params.vel_y_kd = 0.0f;
    g_fc_params.vel_y_kff = 0.0f;
    g_fc_params.vel_y_i_limit = 3.0f;
    g_fc_params.vel_y_d_lpf = 0.0f;

    /* ===== Z轴速度环参数 ===== */
    /* 保留电池压降下的积分补偿能力，同时提高 P 响应并削弱 D 对测速毛刺的放大 */
    g_fc_params.vel_z_kp = 900.0f;
    g_fc_params.vel_z_ki = 95.0f;
    g_fc_params.vel_z_kd = 38.0f;
    g_fc_params.vel_z_kff = 0.0f;
    g_fc_params.vel_z_i_limit = 450.0f;
    g_fc_params.vel_z_d_lpf = 0.08f;

    /* ===== 模式1 跟杆前馈与刹车参数 ===== */
    g_fc_params.mode1_track_ff_deg_per_cmps = 0.06f;
    g_fc_params.mode1_track_ff_limit_deg = 6.0f;
    g_fc_params.mode1_brake_kp = 0.18f;
    g_fc_params.mode1_brake_angle_limit_deg = 25.0f;
    g_fc_params.mode1_brake_entry_delay_s = 0.04f;
    g_fc_params.mode1_brake_exit_hold_s = 0.16f;
    g_fc_params.mode1_brake_exit_vel_cmps = 10.0f;
    g_fc_params.mode1_brake_deadzone_cmps = 3.0f;
    g_fc_params.mode1_zero_damp_deadzone_cmps = 1.0f;
    g_fc_params.mode1_brake_blend_low_cmps = 8.0f;
    g_fc_params.mode1_brake_blend_high_cmps = 20.0f;
}

