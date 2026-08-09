/*****************************************************************************
 * 文件: fc_params.c
 * 模块: 飞控 - 参数管理实现
 * 职责: 管理默认参数、运行时参数以及 Flash 持久化
 *****************************************************************************/

#include "fc_params.h"

#include <string.h>

/* 飞控参数全局实例：集中保存控制周期、油门基准和各控制环 PID 参数 */
fc_params_t g_fc_params;
/* 2BL3 图传发送开关：0=关闭，1=仅非飞行发送，2=始终发送 */
volatile uint8_t g_2bl3_image_send_enable = 2U;
/*
 * 函数名: fc_params_fill_defaults
 * 功能: 向目标参数结构体写入编译期默认值
 * 输入参数:
 *   params - 目标参数结构体指针
 * 返回值: 无
 */
static void fc_params_fill_defaults(fc_params_t *params)
{
    if (NULL == params)
    {
        return;
    }

    memset(params, 0, sizeof(*params));

    /* ===== 控制周期参数 ===== */
    params->gyro_dt = 0.001f;   /* 1kHz */
    params->angle_dt = 0.002f;  /* 500Hz */
    params->pos_xy_dt = 0.02f;  /* 50Hz */
    params->pos_z_dt = 0.02f;   /* 50Hz */
    params->vel_xy_dt = 0.02f;  /* 50Hz */
    params->vel_z_dt = 0.01f;   /* 100Hz */

    /* ===== 油门与机械配平参数 ===== */
    params->base_throttle = 3500;         /* 悬停油门 */
    params->roll_mech_trim_deg = 0.5f;    /* Roll 机械配平角 */
    params->pitch_mech_trim_deg = -1.5f;   /* Pitch 机械配平角 */

    /* ===== Roll/Pitch 前馈与输出滤波参数 ===== */
    params->gyro_ff_smoothing_ms = 8.0f;
    params->gyro_ff_limit = 200.0f;
    params->angle_ff_smoothing_ms = 10.0f;
    params->angle_ff_limit_dps = 150.0f;
    params->angle_output_lpf_hz = 50.0f;

    /* ===== Roll 轴角速度环参数 ===== */
    params->roll_gyro_kp = 6.8f;
    params->roll_gyro_ki = 0.14f;
    params->roll_gyro_kd = 0.020f;
    params->roll_gyro_kff = 0.05f;
    params->roll_gyro_i_limit = 2000.0f;
    params->roll_gyro_d_lpf = 60.0f;

    /* ===== Pitch 轴角速度环参数 ===== */
    params->pitch_gyro_kp = 6.8f;
    params->pitch_gyro_ki = 0.18f;
    params->pitch_gyro_kd = 0.014f;
    params->pitch_gyro_kff = 0.05f;
    params->pitch_gyro_i_limit = 2000.0f;
    params->pitch_gyro_d_lpf = 60.0f;

    /* ===== Yaw 轴角速度环参数 ===== */
    params->yaw_gyro_kp = 13.0f;
    params->yaw_gyro_ki = 4.8f;
    params->yaw_gyro_kd = 0.0f;
    params->yaw_gyro_kff = 0.0f;
    params->yaw_gyro_i_limit = 2000.0f;
    params->yaw_gyro_d_lpf = 30.0f;

    /* ===== Roll 轴角度环参数 ===== */
    params->roll_angle_kp = 8.2f;
    params->roll_angle_ki = 0.0f;
    params->roll_angle_kd = 0.0f;
    params->roll_angle_kff = 0.42f;
    params->roll_angle_i_limit = 80.0f;
    params->roll_angle_d_lpf = 15.0f;

    /* ===== Pitch 轴角度环参数 ===== */
    params->pitch_angle_kp = 7.5f;
    params->pitch_angle_ki = 0.0f;
    params->pitch_angle_kd = 0.0f;
    params->pitch_angle_kff = 0.42f;
    params->pitch_angle_i_limit = 80.0f;
    params->pitch_angle_d_lpf = 15.0f;

    /* ===== Yaw 轴角度环参数 ===== */
    params->yaw_angle_kp = 5.4f;
    params->yaw_angle_ki = 0.0f;
    params->yaw_angle_kd = 0.0f;
    params->yaw_angle_kff = 0.0f;
    params->yaw_angle_i_limit = 0.0f;
    params->yaw_angle_d_lpf = 0.0f;

    /* ===== Z 轴位置环参数 ===== */
    params->pos_z_kp = 1.3f;
    params->pos_z_ki = 0.0f;
    params->pos_z_kd = 0.0f;
    params->pos_z_kff = 0.0f;
    params->pos_z_i_limit = 0.0f;
    params->pos_z_d_lpf = 0.0f;

    /* ===== Z 轴速度环参数 ===== */
    params->vel_z_kp = 500.0f;
    params->vel_z_ki = 120.0f;
    params->vel_z_kd =0.0f;
    params->vel_z_kff = 0.0f;
    params->vel_z_i_limit = 700.0f;
    params->vel_z_d_lpf = 0.0f;

    /* ===== 模式 1 跟杆前馈与刹车参数 ===== */
    params->mode1_track_ff_deg_per_cmps = 0.06f;
    params->mode1_brake_kp = 0.18f;
    params->mode1_brake_exit_vel_cmps = 10.0f;

    /* ===== 位置估计参数 ===== */
    params->pos_est_k_flow = 0.5f;

    /* ===== 模式 2 直接图像 PD 与角度前馈参数 ===== */
    params->mode2_img_x_kp = 0.09f;
    params->mode2_img_x_kp2 = 0.08f;
    params->mode2_img_x_kd = 0.18f;
    params->mode2_img_x_kd_slope = 0.00255f;
    params->mode2_img_x_d_lpf = 2.5f;
    params->mode2_img_y_kp = 0.10f;
    params->mode2_img_y_kp2 = 0.12f;
    params->mode2_img_y_kd = 0.18f;
    params->mode2_img_y_kd_slope = 0.00350f;
    params->mode2_img_y_d_lpf = 2.5f;
    params->mode2_img_d_limit_deg = 2.5f;
    params->mode2_car_vel_ff_x_deg_per_mps = 0.35f;
    params->mode2_car_vel_ff_y_deg_per_mps = 0.35f;
    params->mode2_car_accel_ff_x_deg_per_mps2 = 5.2f;
    params->mode2_car_accel_ff_y_deg_per_mps2 = 5.2f;
    params->mode2_car_accel_lpf_hz = 3.0f;
    params->mode2_car_accel_lead_s = 0.08f;
    params->mode2_car_accel_raw_limit_mps2 = 4.0f;
    params->mode2_car_accel_ff_limit_deg = 8.0f;
    params->mode2_attitude_fb_gain = 0.0f;
    params->mode2_turn_accel_ff_gain_x = 0.60f;
    params->mode2_turn_accel_ff_gain_y = 0.60f;
    params->mode2_angle_limit_deg = 14.0f;
    params->mode2_angle_slew_dps = 70.0f;
    params->mode2_angle_brake_slew_dps = 120.0f;

    /* ===== 模式 3 图像、速度与转弯前馈参数 ===== */
    params->mode3_img_x_kp = 2.0f;
    params->mode3_img_x_kp2 = 1.0f;
    params->mode3_img_x_ki = 0.0f;
    params->mode3_img_x_kd = 0.7f;
    params->mode3_img_x_kff = 0.0f;
    params->mode3_img_x_i_limit = 0.0f;
    params->mode3_img_x_d_lpf = 1.5f;
    params->mode3_img_y_kp = 2.0f;
    params->mode3_img_y_kp2 = 1.0f;
    params->mode3_img_y_ki = 0.0f;
    params->mode3_img_y_kd = 0.7f;
    params->mode3_img_y_kff = 0.0f;
    params->mode3_img_y_i_limit = 0.0f;
    params->mode3_img_y_d_lpf = 1.5f;
    params->mode3_vel_x_kp = 0.12f;
    params->mode3_vel_x_ki = 0.0f;
    params->mode3_vel_x_kd = 0.0f;
    params->mode3_vel_x_kff = 0.0f;
    params->mode3_vel_x_i_limit = 3.0f;
    params->mode3_vel_x_d_lpf = 10.0f;
    params->mode3_vel_y_kp = 0.12f;
    params->mode3_vel_y_ki = 0.0f;
    params->mode3_vel_y_kd = 0.0f;
    params->mode3_vel_y_kff = 0.0f;
    params->mode3_vel_y_i_limit = 3.0f;
    params->mode3_vel_y_d_lpf = 10.0f;
    params->mode3_kp_car_x = 60.0f;
    params->mode3_kp_car_y = 70.0f;
    params->mode3_turn_accel_ff_gain_x = 0.72f;
    params->mode3_turn_accel_ff_gain_y = 0.30f;
    params->mode3_turn_accel_ff_limit_x_deg = 18.0f;
    params->mode3_turn_accel_ff_limit_y_deg = 14.0f;

    /* ===== 模式 4 图像、速度与转弯前馈参数 ===== */
    params->mode4_img_x_kp = 2.0f;
    params->mode4_img_x_kp2 = 1.0f;
    params->mode4_img_x_ki = 0.0f;
    params->mode4_img_x_kd = 0.7f;
    params->mode4_img_x_kff = 0.0f;
    params->mode4_img_x_i_limit = 0.0f;
    params->mode4_img_x_d_lpf = 1.5f;
    params->mode4_img_y_kp = 2.0f;
    params->mode4_img_y_kp2 = 1.0f;
    params->mode4_img_y_ki = 0.0f;
    params->mode4_img_y_kd = 0.7f;
    params->mode4_img_y_kff = 0.0f;
    params->mode4_img_y_i_limit = 0.0f;
    params->mode4_img_y_d_lpf = 1.5f;
    params->mode4_vel_x_kp = 0.2f;
    params->mode4_vel_x_ki = 0.0f;
    params->mode4_vel_x_kd = 0.0f;
    params->mode4_vel_x_kff = 0.0f;
    params->mode4_vel_x_i_limit = 3.0f;
    params->mode4_vel_x_d_lpf = 10.0f;
    params->mode4_vel_y_kp = 0.2f;
    params->mode4_vel_y_ki = 0.0f;
    params->mode4_vel_y_kd = 0.0f;
    params->mode4_vel_y_kff = 0.0f;
    params->mode4_vel_y_i_limit = 3.0f;
    params->mode4_vel_y_d_lpf = 10.0f;
    params->mode4_kp_car_x = 60.0f;
    params->mode4_kp_car_y = 90.0f;
    params->mode4_turn_accel_ff_gain_x = 0.72f;
    params->mode4_turn_accel_ff_gain_y = 0.30f;
    params->mode4_turn_accel_ff_limit_x_deg = 18.0f;
    params->mode4_turn_accel_ff_limit_y_deg = 14.0f;

    /* ===== Mode 5 image and velocity params ===== */
    params->mode5_img_x_kp = 2.8f;
    params->mode5_img_x_ki = 0.0f;
    params->mode5_img_x_kd = 0.0f;
    params->mode5_img_x_kff = 0.0f;
    params->mode5_img_x_i_limit = 0.0f;
    params->mode5_img_x_d_lpf = 0.0f;
    params->mode5_img_y_kp = 2.2f;
    params->mode5_img_y_ki = 0.0f;
    params->mode5_img_y_kd = 0.0f;
    params->mode5_img_y_kff = 0.0f;
    params->mode5_img_y_i_limit = 0.0f;
    params->mode5_img_y_d_lpf = 0.0f;
    params->mode5_vel_x_kp = 0.2f;
    params->mode5_vel_x_ki = 0.02f;
    params->mode5_vel_x_kd = 0.0f;
    params->mode5_vel_x_kff = 0.001f;
    params->mode5_vel_x_i_limit = 3.0f;
    params->mode5_vel_x_d_lpf = 10.0f;
    params->mode5_vel_y_kp = 0.2f;
    params->mode5_vel_y_ki = 0.02f;
    params->mode5_vel_y_kd = 0.00f;
    params->mode5_vel_y_kff = 0.001f;
    params->mode5_vel_y_i_limit = 3.0f;
    params->mode5_vel_y_d_lpf = 10.0f;
    params->mode5_kp_car_x = 50.0f;
    params->mode5_kp_car_y = 65.0f;

    /* ===== 模式 7 速度环参数 ===== */
    params->mode7_vel_x_kp = 0.2f;
    params->mode7_vel_x_ki = 0.0f;
    params->mode7_vel_x_kd = 0.0f;
    params->mode7_vel_x_kff = 0.0f;
    params->mode7_vel_x_i_limit = 3.0f;
    params->mode7_vel_x_d_lpf = 10.0f;
    params->mode7_vel_y_kp = 0.2f;
    params->mode7_vel_y_ki = 0.0f;
    params->mode7_vel_y_kd = 0.00f;
    params->mode7_vel_y_kff = 0.0f;
    params->mode7_vel_y_i_limit = 3.0f;
    params->mode7_vel_y_d_lpf = 10.0f;

    /* ===== 模式 8 图像位置环参数 ===== */
    params->mode8_img_x_kp = 2.4f;
    params->mode8_img_x_ki = 0.0f;
    params->mode8_img_x_kd = 0.0f;
    params->mode8_img_x_kff = 0.0f;
    params->mode8_img_x_i_limit = 0.0f;
    params->mode8_img_x_d_lpf = 0.0f;
    params->mode8_img_y_kp = 2.2f;
    params->mode8_img_y_ki = 0.0f;
    params->mode8_img_y_kd = 0.0f;
    params->mode8_img_y_kff = 0.0f;
    params->mode8_img_y_i_limit = 0.0f;
    params->mode8_img_y_d_lpf = 0.0f;
    params->mode8_vel_x_kp = 0.2f;
    params->mode8_vel_x_ki = 0.02f;
    params->mode8_vel_x_kd = 0.0f;
    params->mode8_vel_x_kff = 0.0f;
    params->mode8_vel_x_i_limit = 3.0f;
    params->mode8_vel_x_d_lpf = 10.0f;
    params->mode8_vel_y_kp = 0.2f;
    params->mode8_vel_y_ki = 0.02f;
    params->mode8_vel_y_kd = 0.0f;
    params->mode8_vel_y_kff = 0.0f;
    params->mode8_vel_y_i_limit = 3.0f;
    params->mode8_vel_y_d_lpf = 10.0f;
    params->mode8_kp_car_x = 50.0f;
    params->mode8_kp_car_y = 65.0f;

    /* ===== 各模式航向对准开关 ===== */
    params->yaw_change_mode3 = 1.0f;
    params->yaw_change_mode4 = 0.0f;
    params->yaw_change_mode5 = 0.0f;
    params->yaw_change_mode8 = 0.0f;

}

/*
 * 函数名: FC_Params_Init
 * 功能: 初始化飞控参数默认值
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Params_Init(void)
{
    fc_params_fill_defaults(&g_fc_params);
}

/* Current build does not load flight-control parameters from Flash. */
uint8_t FC_Params_LoadFromFlash(void)
{
    return 0U;
}

/* Current build does not save flight-control parameters to Flash. */
uint8_t FC_Params_SaveToFlash(void)
{
    return 0U;
}

/* Current build has no flight-control Flash parameters to clear. */
uint8_t FC_Params_ClearFlash(void)
{
    return 1U;
}
