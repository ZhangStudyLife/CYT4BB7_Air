/*****************************************************************************
 * 鏂囦欢: fc_params.c
 * 妯″潡: 椋炴帶 - 鍙傛暟绠＄悊瀹炵幇
 * 鑱岃矗: 瀹氫箟鍏ㄥ眬鍙傛暟鍙橀噺 g_fc_params锛屽苟鍒濆鍖栨墍鏈夋帶鍒跺弬鏁?
 *****************************************************************************/

#include "fc_params.h"

/* ==================== 鍏ㄥ眬鍙傛暟瀹炰緥 ==================== */
fc_params_t g_fc_params;

/* ==================== 鍙傛暟鍒濆鍖栧嚱鏁?==================== */
void FC_Params_Init(void)
{
    /* ===== 鎺у埗鍛ㄦ湡 ===== */
    g_fc_params.gyro_dt = 0.0005f; /* 2kHz */
    g_fc_params.angle_dt = 0.002f; /* 500Hz */
    g_fc_params.pos_xy_dt = 0.01f; /* 100Hz */
    g_fc_params.pos_z_dt = 0.02f;  /* 50Hz */
    g_fc_params.vel_z_dt = 0.01f;  /* 100Hz */

    /* ===== 娌归棬鍙傛暟 ===== */
    g_fc_params.base_throttle = 3590; /* 鎮仠娌归棬 */

    /* ===== Roll 杞磋閫熷害鐜弬鏁?===== */
    g_fc_params.roll_gyro_kp = 2.0f;
    g_fc_params.roll_gyro_ki = 2.0f;
    g_fc_params.roll_gyro_kd = 0.0f;
    g_fc_params.roll_gyro_kff = 0.0f;
    g_fc_params.roll_gyro_i_limit = 1400.0f;
    g_fc_params.roll_gyro_d_lpf = 0.08f;

    /* ===== Pitch 杞磋閫熷害鐜弬鏁?===== */
    g_fc_params.pitch_gyro_kp = 2.0f;
    g_fc_params.pitch_gyro_ki = 2.0f;
    g_fc_params.pitch_gyro_kd = 0.0f;
    g_fc_params.pitch_gyro_kff = 0.0f;
    g_fc_params.pitch_gyro_i_limit = 1400.0f;
    g_fc_params.pitch_gyro_d_lpf = 0.08f;

    /* ===== Yaw 杞磋閫熷害鐜弬鏁?===== */
    g_fc_params.yaw_gyro_kp = 14.0f;
    g_fc_params.yaw_gyro_ki = 8.0f;
    g_fc_params.yaw_gyro_kd = 0.0f;
    g_fc_params.yaw_gyro_kff = 0.0f;
    g_fc_params.yaw_gyro_i_limit = 1800.0f;
    g_fc_params.yaw_gyro_d_lpf = 0.18f;

    /* ===== Roll 杞磋搴︾幆鍙傛暟 ===== */
    g_fc_params.roll_angle_kp = 4.0f;
    g_fc_params.roll_angle_ki = 0.08f;
    g_fc_params.roll_angle_kd = 0.0f;
    g_fc_params.roll_angle_kff = 0.0f;
    g_fc_params.roll_angle_i_limit = 110.0f;
    g_fc_params.roll_angle_d_lpf = 0.0f;

    /* ===== Pitch 杞磋搴︾幆鍙傛暟 ===== */
    g_fc_params.pitch_angle_kp = 4.0f;
    g_fc_params.pitch_angle_ki = 0.08f;
    g_fc_params.pitch_angle_kd = 0.0f;
    g_fc_params.pitch_angle_kff = 0.0f;
    g_fc_params.pitch_angle_i_limit = 110.0f;
    g_fc_params.pitch_angle_d_lpf = 0.0f;

    /* ===== Yaw 杞磋搴︾幆鍙傛暟 ===== */
    g_fc_params.yaw_angle_kp = 0.0f;
    g_fc_params.yaw_angle_ki = 0.0f;
    g_fc_params.yaw_angle_kd = 0.0f;
    g_fc_params.yaw_angle_kff = 0.0f;
    g_fc_params.yaw_angle_i_limit = 0.0f;
    g_fc_params.yaw_angle_d_lpf = 0.0f;

    /* ===== X 杞翠綅缃幆鍙傛暟 ===== */
    g_fc_params.pos_x_kp = 12.0f;
    g_fc_params.pos_x_ki = 0.0f;
    g_fc_params.pos_x_kd = 0.0f;
    g_fc_params.pos_x_kff = 0.0f;
    g_fc_params.pos_x_i_limit = 0.8f;
    g_fc_params.pos_x_d_lpf = 0.0f;

    /* ===== Y 杞翠綅缃幆鍙傛暟 ===== */
    g_fc_params.pos_y_kp = 6.0f;
    g_fc_params.pos_y_ki = 0.0f;
    g_fc_params.pos_y_kd = 0.0f;
    g_fc_params.pos_y_kff = 0.0f;
    g_fc_params.pos_y_i_limit = 0.8f;
    g_fc_params.pos_y_d_lpf = 0.0f;

    /* ===== Z 杞翠綅缃幆鍙傛暟 ===== */
    g_fc_params.pos_z_kp = 0.65f;
    g_fc_params.pos_z_ki = 0.0f;
    g_fc_params.pos_z_kd = 0.0f;
    g_fc_params.pos_z_kff = 0.0f;
    g_fc_params.pos_z_i_limit = 0.0f;
    g_fc_params.pos_z_d_lpf = 0.0f;

    /* ===== Z 杞撮€熷害鐜弬鏁?===== */
    g_fc_params.vel_z_kp = 720.0f;
    g_fc_params.vel_z_ki = 110.0f;
    g_fc_params.vel_z_kd = 0.0f;
    g_fc_params.vel_z_kff = 0.0f;
    g_fc_params.vel_z_i_limit = 380.0f;
    g_fc_params.vel_z_d_lpf = 0.0f;
}

