/*****************************************************************************
 * 文件: fc_params.h
 * 模块: 飞控 - 参数集中管理
 * 职责: 定义飞控控制周期、油门基准、机械配平角和各控制环参数结构体，并提供 Flash 持久化接口
 *****************************************************************************/

#ifndef FC_PARAMS_H
#define FC_PARAMS_H

#include <stdint.h>

/* 飞控参数存储页号：固定使用 94 页，避开 IMU 校准占用的 95 页 */
#define FC_PARAMS_FLASH_PAGE (94U)

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

    /* ===== 油门与机械配平 ===== */
    int32_t base_throttle;       /* 悬停油门基准，单位 mixer 输入 */
    float roll_mech_trim_deg;    /* Roll 机械配平角，单位 deg */
    float pitch_mech_trim_deg;   /* Pitch 机械配平角，单位 deg */

    /* ===== Roll/Pitch 前馈与输出滤波参数 ===== */
    float gyro_ff_smoothing_ms;  /* 角速度环前馈 PT3 单级平滑时间，单位 ms */
    float gyro_ff_limit;         /* 角速度环前馈控制量限幅 */
    float angle_ff_smoothing_ms; /* 角度环前馈 PT3 单级平滑时间，单位 ms */
    float angle_ff_limit_dps;    /* 角度环前馈限幅，单位 deg/s */
    float angle_output_lpf_hz;   /* 角度环输出 PT3 低通截止频率，单位 Hz */

    /* ===== Roll 轴角速度环参数 ===== */
    float roll_gyro_kp;
    float roll_gyro_ki;
    float roll_gyro_kd;
    float roll_gyro_kff;     /* 前馈增益 */
    float roll_gyro_i_limit;
    float roll_gyro_d_lpf;   /* D 项低通截止频率，单位 Hz，0 表示旁路 */

    /* ===== Pitch 轴角速度环参数 ===== */
    float pitch_gyro_kp;
    float pitch_gyro_ki;
    float pitch_gyro_kd;
    float pitch_gyro_kff;    /* 前馈增益 */
    float pitch_gyro_i_limit;
    float pitch_gyro_d_lpf;  /* D 项低通截止频率，单位 Hz，0 表示旁路 */

    /* ===== Yaw 轴角速度环参数 ===== */
    float yaw_gyro_kp;
    float yaw_gyro_ki;
    float yaw_gyro_kd;
    float yaw_gyro_kff;      /* 前馈增益 */
    float yaw_gyro_i_limit;
    float yaw_gyro_d_lpf;    /* D 项低通截止频率，单位 Hz，0 表示旁路 */

    /* ===== Roll 轴角度环参数 ===== */
    float roll_angle_kp;
    float roll_angle_ki;
    float roll_angle_kd;
    float roll_angle_kff;    /* 前馈增益 */
    float roll_angle_i_limit;
    float roll_angle_d_lpf;  /* D 项低通截止频率，单位 Hz，0 表示旁路 */

    /* ===== Pitch 轴角度环参数 ===== */
    float pitch_angle_kp;
    float pitch_angle_ki;
    float pitch_angle_kd;
    float pitch_angle_kff;   /* 前馈增益 */
    float pitch_angle_i_limit;
    float pitch_angle_d_lpf; /* D 项低通截止频率，单位 Hz，0 表示旁路 */

    /* ===== Yaw 轴角度环参数 ===== */
    float yaw_angle_kp;
    float yaw_angle_ki;
    float yaw_angle_kd;
    float yaw_angle_kff;     /* 前馈增益 */
    float yaw_angle_i_limit;
    float yaw_angle_d_lpf;   /* D 项低通截止频率，单位 Hz，0 表示旁路 */

    /* ===== Z 轴位置环参数 ===== */
    float pos_z_kp;
    float pos_z_ki;
    float pos_z_kd;
    float pos_z_kff;         /* 前馈增益 */
    float pos_z_i_limit;
    float pos_z_d_lpf;       /* D 项低通截止频率，单位 Hz，0 表示旁路 */

    /* ===== Z 轴速度环参数 ===== */
    float vel_z_kp;
    float vel_z_ki;
    float vel_z_kd;
    float vel_z_kff;         /* 前馈增益 */
    float vel_z_i_limit;
    float vel_z_d_lpf;       /* D 项低通截止频率，单位 Hz，0 表示旁路 */

    /* ===== 模式 1 直接图像 PD 与角度前馈参数 ===== */
    float mode1_img_kp;                     /* 模式1图像P增益，单位deg/px */
    float mode1_img_kd;                     /* 模式1图像D增益，单位deg/(px/s) */
    float mode1_img_d_lpf_hz;               /* 模式1图像D项低通截止频率，单位Hz */
    float mode1_car_vel_error_lpf_hz;       /* 模式1车速误差低通截止频率，单位Hz */
    float mode1_car_accel_gain_pos;         /* 模式1正向车速误差加速度增益 */
    float mode1_car_accel_gain_neg;         /* 模式1负向车速误差加速度增益 */
    float mode1_car_accel_ff;               /* 模式1车加速度角度前馈，单位deg/(m/s^2) */
    float mode1_car_turn_accel_ff;          /* 模式1向心加速度角度前馈，单位deg/(m/s^2) */
    float mode1_car_turn_accel_lpf_hz;      /* 模式1向心加速度低通截止频率，单位Hz */

    /* ===== 位置估计参数 ===== */
    float pos_est_k_flow;              /* 光流速度融合权重，范围 0~1 */
    /* ===== 模式 8 图像位置环参数 ===== */
    float mode8_img_x_kp;              /* 模式8图像X位置环P增益 */
    float mode8_img_x_ki;              /* 模式8图像X位置环I增益 */
    float mode8_img_x_kd;              /* 模式8图像X位置环D增益 */
    float mode8_img_x_kff;             /* 模式8图像X位置环前馈增益 */
    float mode8_img_x_i_limit;         /* 模式8图像X位置环积分限幅 */
    float mode8_img_x_d_lpf;           /* 模式8图像X位置环D项低通截止频率，单位Hz */
    float mode8_img_y_kp;              /* 模式8图像Y位置环P增益 */
    float mode8_img_y_ki;              /* 模式8图像Y位置环I增益 */
    float mode8_img_y_kd;              /* 模式8图像Y位置环D增益 */
    float mode8_img_y_kff;             /* 模式8图像Y位置环前馈增益 */
    float mode8_img_y_i_limit;         /* 模式8图像Y位置环积分限幅 */
    float mode8_img_y_d_lpf;           /* 模式8图像Y位置环D项低通截止频率，单位Hz */
    /* ===== 模式 2 直接图像 PD 与角度前馈参数 ===== */
    float mode2_img_kp;                     /* 模式2图像P增益，单位deg/px */
    float mode2_img_kd;                     /* 模式2图像D增益，单位deg/(px/s) */
    float mode2_img_d_lpf_hz;               /* 模式2图像D项低通截止频率，单位Hz */
    float mode2_car_vel_error_lpf_hz;       /* 模式2车速误差低通截止频率，单位Hz */
    float mode2_car_accel_gain_pos;         /* 模式2正向车速误差加速度增益 */
    float mode2_car_accel_gain_neg;         /* 模式2负向车速误差加速度增益 */
    float mode2_car_accel_ff;               /* 模式2车加速度角度前馈，单位deg/(m/s^2) */
    float mode2_car_turn_accel_ff;          /* 模式2向心加速度角度前馈，单位deg/(m/s^2) */
    float mode2_car_turn_accel_lpf_hz;      /* 模式2向心加速度低通截止频率，单位Hz */
    /* ===== 模式 5 直接图像 PD 与角度前馈参数 ===== */
    float mode5_img_kp;                     /* 模式5图像P增益，单位deg/px */
    float mode5_img_kd;                     /* 模式5图像D增益，单位deg/(px/s) */
    float mode5_img_d_lpf_hz;               /* 模式5图像D项低通截止频率，单位Hz */
    float mode5_car_vel_error_lpf_hz;       /* 模式5车速误差低通截止频率，单位Hz */
    float mode5_car_accel_gain_pos;         /* 模式5正向车速误差加速度增益 */
    float mode5_car_accel_gain_neg;         /* 模式5负向车速误差加速度增益 */
    float mode5_car_accel_ff;               /* 模式5车加速度角度前馈，单位deg/(m/s^2) */
    float mode5_car_turn_accel_ff;          /* 模式5向心加速度角度前馈，单位deg/(m/s^2) */
    float mode5_car_turn_accel_lpf_hz;      /* 模式5向心加速度低通截止频率，单位Hz */
    float mode8_vel_x_kp;
    float mode8_vel_x_ki;
    float mode8_vel_x_kd;
    float mode8_vel_x_kff;
    float mode8_vel_x_i_limit;
    float mode8_vel_x_d_lpf;
    float mode8_vel_y_kp;
    float mode8_vel_y_ki;
    float mode8_vel_y_kd;
    float mode8_vel_y_kff;
    float mode8_vel_y_i_limit;
    float mode8_vel_y_d_lpf;
    float mode8_kp_car_x;
    float mode8_kp_car_y;
    /* ===== 模式 4 直接图像 PD 与角度前馈参数 ===== */
    float mode4_img_kp;                     /* 模式4图像P增益，单位deg/px */
    float mode4_img_kd;                     /* 模式4图像D增益，单位deg/(px/s) */
    float mode4_img_d_lpf_hz;               /* 模式4图像D项低通截止频率，单位Hz */
    float mode4_car_vel_error_lpf_hz;       /* 模式4车速误差低通截止频率，单位Hz */
    float mode4_car_accel_gain_pos;         /* 模式4正向车速误差加速度增益 */
    float mode4_car_accel_gain_neg;         /* 模式4负向车速误差加速度增益 */
    float mode4_car_accel_ff;               /* 模式4车加速度角度前馈，单位deg/(m/s^2) */
    float mode4_car_turn_accel_ff;          /* 模式4向心加速度角度前馈，单位deg/(m/s^2) */
    float mode4_car_turn_accel_lpf_hz;      /* 模式4向心加速度低通截止频率，单位Hz */
    float mode7_vel_x_kp;
    float mode7_vel_x_ki;
    float mode7_vel_x_kd;
    float mode7_vel_x_kff;
    float mode7_vel_x_i_limit;
    float mode7_vel_x_d_lpf;
    float mode7_vel_y_kp;
    float mode7_vel_y_ki;
    float mode7_vel_y_kd;
    float mode7_vel_y_kff;
    float mode7_vel_y_i_limit;
    float mode7_vel_y_d_lpf;
    /* ===== 各模式航向控制：0=固定0度，1=信标对准，2=步进搜索 ===== */
    float yaw_change_mode1;
    float yaw_change_mode2;
    float yaw_change_mode4;
    float yaw_change_mode5;
    float yaw_change_mode8;
} fc_params_t;

/* 飞控参数全局实例：运行时调参只会修改这份 RAM 变量 */
extern fc_params_t g_fc_params;
/* 2BL3 图传发送开关：0=关闭，1=仅非飞行发送，2=始终发送 */
extern volatile uint8_t g_2bl3_image_send_enable;

/*
 * 函数名: FC_Params_Init
 * 功能: 装载飞控参数默认值
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Params_Init(void);

/*
 * 函数名: FC_Params_LoadFromFlash
 * 功能: 从 Flash 读取参数并覆盖当前运行时参数
 * 输入参数: 无
 * 返回值: 1=读取成功，0=Flash 中无有效参数
 */
uint8_t FC_Params_LoadFromFlash(void);

/*
 * 函数名: FC_Params_SaveToFlash
 * 功能: 将当前运行时参数保存到 Flash
 * 输入参数: 无
 * 返回值: 1=保存成功，0=保存失败
 */
uint8_t FC_Params_SaveToFlash(void);

/*
 * 函数名: FC_Params_ClearFlash
 * 功能: 擦除飞控参数 Flash 页
 * 输入参数: 无
 * 返回值: 1=擦除成功，0=擦除失败
 */
uint8_t FC_Params_ClearFlash(void);

#endif /* FC_PARAMS_H */
