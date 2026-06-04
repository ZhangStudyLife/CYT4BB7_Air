/*****************************************************************************
 * 文件: fc_params.c
 * 模块: 飞控 - 参数管理实现
 * 职责: 管理默认参数、运行时参数以及 Flash 持久化
 *****************************************************************************/

#include "fc_params.h"

#include <string.h>

/* 飞控参数全局实例：集中保存控制周期、油门基准和各控制环 PID 参数 */
fc_params_t g_fc_params;

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
    params->base_throttle = 2800;         /* 悬停油门 */
    params->roll_mech_trim_deg = 2.3f;    /* Roll 机械配平角 */
    params->pitch_mech_trim_deg = 2.6f;   /* Pitch 机械配平角 */

    /* ===== Roll 轴角速度环参数 ===== */
    params->roll_gyro_kp = 5.4f;
    params->roll_gyro_ki = 0.18f;
    params->roll_gyro_kd = 0.010f;
    params->roll_gyro_kff = 0.0f;
    params->roll_gyro_i_limit = 180.0f;
    params->roll_gyro_d_lpf = 25.0f;

    /* ===== Pitch 轴角速度环参数 ===== */
    params->pitch_gyro_kp = 5.3f;
    params->pitch_gyro_ki = 0.14f;
    params->pitch_gyro_kd = 0.010f;
    params->pitch_gyro_kff = 0.0f;
    params->pitch_gyro_i_limit = 140.0f;
    params->pitch_gyro_d_lpf = 25.0f;

    /* ===== Yaw 轴角速度环参数 ===== */
    params->yaw_gyro_kp = 11.0f;
    params->yaw_gyro_ki = 3.0f;
    params->yaw_gyro_kd = 0.0f;
    params->yaw_gyro_kff = 0.0f;
    params->yaw_gyro_i_limit = 700.0f;
    params->yaw_gyro_d_lpf = 30.0f;

    /* ===== Roll 轴角度环参数 ===== */
    params->roll_angle_kp = 6.0f;
    params->roll_angle_ki = 0.0f;
    params->roll_angle_kd = 0.0f;
    params->roll_angle_kff = 0.02f;
    params->roll_angle_i_limit = 80.0f;
    params->roll_angle_d_lpf = 15.0f;

    /* ===== Pitch 轴角度环参数 ===== */
    params->pitch_angle_kp = 6.2f;
    params->pitch_angle_ki = 0.0f;
    params->pitch_angle_kd = 0.0f;
    params->pitch_angle_kff = 0.04f;
    params->pitch_angle_i_limit = 80.0f;
    params->pitch_angle_d_lpf = 15.0f;

    /* ===== Yaw 轴角度环参数 ===== */
    params->yaw_angle_kp = 1.5f;
    params->yaw_angle_ki = 0.0f;
    params->yaw_angle_kd = 0.0f;
    params->yaw_angle_kff = 0.0f;
    params->yaw_angle_i_limit = 0.0f;
    params->yaw_angle_d_lpf = 0.0f;

    /* ===== X 轴位置环参数 ===== */
    params->pos_x_kp = 0.90f;
    params->pos_x_ki = 0.0f;
    params->pos_x_kd = 0.0f;
    params->pos_x_kff = 0.0f;
    params->pos_x_i_limit = 0.0f;
    params->pos_x_d_lpf = 0.0f;

    /* ===== Y 轴位置环参数 ===== */
    params->pos_y_kp = 0.90f;
    params->pos_y_ki = 0.0f;
    params->pos_y_kd = 0.0f;
    params->pos_y_kff = 0.0f;
    params->pos_y_i_limit = 0.0f;
    params->pos_y_d_lpf = 0.0f;

    /* ===== Z 轴位置环参数 ===== */
    params->pos_z_kp = 1.3f;
    params->pos_z_ki = 0.0f;
    params->pos_z_kd = 0.0f;
    params->pos_z_kff = 0.0f;
    params->pos_z_i_limit = 0.0f;
    params->pos_z_d_lpf = 0.0f;

    /* ===== X 轴速度环参数 ===== */
    params->vel_x_kp = 0.14f;
    params->vel_x_ki = 0.02f;
    params->vel_x_kd = 0.0f;
    params->vel_x_kff = 0.0f;
    params->vel_x_i_limit = 3.0f;
    params->vel_x_d_lpf = 0.0f;

    /* ===== Y 轴速度环参数 ===== */
    params->vel_y_kp = 0.14f;
    params->vel_y_ki = 0.02f;
    params->vel_y_kd = 0.0f;
    params->vel_y_kff = 0.0f;
    params->vel_y_i_limit = 3.0f;
    params->vel_y_d_lpf = 0.0f;

    /* ===== Z 轴速度环参数 ===== */
    params->vel_z_kp = 30.0f;
    params->vel_z_ki = 50.0f;
    params->vel_z_kd =0.0f;
    params->vel_z_kff = 0.0f;
    params->vel_z_i_limit = 450.0f;
    params->vel_z_d_lpf = 0.0f;

    /* ===== 模式 1 跟杆前馈与刹车参数 ===== */
    params->mode1_track_ff_deg_per_cmps = 0.06f;
    params->mode1_brake_kp = 0.18f;
    params->mode1_brake_exit_vel_cmps = 10.0f;

    /* ===== 模式 2 速度环前馈参数 ===== */
    params->reserved_vel_x_ff_deg_per_cmps = 0.010f;
    params->reserved_vel_y_ff_deg_per_cmps = 0.010f;
    params->mode4_adrc_enable = 1.0f;
    params->mode4_adrc_log_enable = 1.0f;
    params->mode4_adrc_b0_x = 16.0f;
    params->mode4_adrc_b0_y = 14.5f;
    params->mode4_adrc_td_acc_limit_cmss = 100.0f;
    params->mode4_adrc_td_jerk_limit_cmsss = 450.0f;
    params->mode4_adrc_td_brake_acc_limit_cmss = 160.0f;
    params->mode4_adrc_td_brake_jerk_limit_cmsss = 900.0f;
    params->mode4_adrc_eso_beta1 = 12.0f;
    params->mode4_adrc_eso_beta2 = 36.0f;
    params->mode4_adrc_eso_alpha1 = 0.50f;
    params->mode4_adrc_eso_alpha2 = 0.25f;
    params->mode4_adrc_eso_delta_cmps = 15.0f;
    params->mode4_adrc_nl_kp = 4.2f;
    params->mode4_adrc_nl_alpha = 0.75f;
    params->mode4_adrc_nl_delta_cmps = 12.0f;
    params->mode4_adrc_comp_limit_cmss = 45.0f;
    params->mode4_adrc_angle_limit_deg = 10.0f;
    params->mode4_adrc_output_rate_limit_degps = 35.0f;
    params->mode4_adrc_zero_quiet_cmps = 8.0f;

    params->mode2_vel_x_kp = 0.12f;
    params->mode2_vel_x_ki = 0.02f;
    params->mode2_vel_x_kd = 0.0f;
    params->mode2_vel_x_kff = 0.0f;
    params->mode2_vel_x_i_limit = 3.0f;
    params->mode2_vel_x_d_lpf = 0.0f;
    params->mode2_vel_y_kp = 0.12f;
    params->mode2_vel_y_ki = 0.02f;
    params->mode2_vel_y_kd = 0.0f;
    params->mode2_vel_y_kff = 0.0f;
    params->mode2_vel_y_i_limit = 3.0f;
    params->mode2_vel_y_d_lpf = 0.0f;

    /* ===== 位置估计参数 ===== */
    params->pos_est_k_flow = 0.04f;

    /* ===== 模式 8 图像位置环参数 ===== */
    params->mode8_img_x_kp = 1.1f;
    params->mode8_img_x_ki = 0.0f;
    params->mode8_img_x_kd = 0.10f;
    params->mode8_img_x_kff = 0.0f;
    params->mode8_img_x_i_limit = 0.0f;
    params->mode8_img_x_d_lpf = 0.0f;
    params->mode8_img_y_kp = 0.9f;
    params->mode8_img_y_ki = 0.0f;
    params->mode8_img_y_kd = 0.10f;
    params->mode8_img_y_kff = 0.0f;
    params->mode8_img_y_i_limit = 0.0f;
    params->mode8_img_y_d_lpf = 0.0f;
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
