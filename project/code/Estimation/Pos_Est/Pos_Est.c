// #include "Pos_Est.h"
// #include "FlightController/fc_params.h"

// extern volatile uint32 tick_1000us_cnt;

// /* X 轴加速度一阶低通输出，单位 cm/s^2，飞机往前加速为正，往后加速为负 */
// float acc_x_lp = 0.0f;
// /* Y 轴加速度一阶低通输出，单位 cm/s^2，飞机往右加速为正，往左加速为负 */
// float acc_y_lp = 0.0f;

// float vel_x_pred = 0.0f;
// float vel_y_pred = 0.0f;

// /* 光流解算得到的 X 轴速度，单位 cm/s，往左飞为正，往右飞为负 */
// float opflow_vel_x = 0.0f;
// /* 光流解算得到的 Y 轴速度，单位 cm/s，往前飞为正，往后飞为负 */
// float opflow_vel_y = 0.0f;

// /* 位置估计的 X 轴速度，单位 cm/s */
// float Pos_Est_vel_x = 0.0f;
// /* 位置估计的 Y 轴速度，单位 cm/s */
// float Pos_Est_vel_y = 0.0f;

// /*
//  * 函数名: Pos_Est_Init
//  * 功能: 初始化光流位置估计模块和相关滤波器
//  * 输入参数: 无
//  * 返回值: 无
//  */
// void Pos_Est_Init(void)
// {
//     PMW3901_Init();
//     FlowGyroDecoupler_Init();
//     acc_x_lp = 0.0f;
//     acc_y_lp = 0.0f;
//     opflow_vel_x = 0.0f;
//     opflow_vel_y = 0.0f;
//     Pos_Est_vel_x = 0.0f;
//     Pos_Est_vel_y = 0.0f;
//     vel_x_pred = 0.0f;
//     vel_y_pred = 0.0f;
// }

// /*
//  * 函数名: Pos_Est_Reinit
//  * 功能: 重置光流和滤波器内部状态
//  * 输入参数: 无
//  * 返回值: 无
//  */
// void Pos_Est_Reinit(void)
// {
//     PMW3901_ReInit();
//     FlowGyroDecoupler_Reinit();

//     acc_x_lp = 0.0f;
//     acc_y_lp = 0.0f;
//     opflow_vel_x = 0.0f;
//     opflow_vel_y = 0.0f;
//     Pos_Est_vel_x = 0.0f;
//     Pos_Est_vel_y = 0.0f;
//     vel_x_pred = 0.0f;
//     vel_y_pred = 0.0f;
// }

// /*
//  * 函数名: Pos_Est_Update_1000HZ
//  * 功能: 推送光流去旋转解耦所需陀螺数据，并更新水平加速度低通输出
//  * 输入参数: 无
//  * 返回值: 无
//  *
//  * 加速度数据路径说明：
//  *   使用 g_imufilter_1000hz（已经过陷波161Hz/320Hz + 12Hz低通滤波）作为输入，
//  *   通过 AccelCalibration_RotateImuToBody 旋转到机体系后，利用旋转矩阵投影到水平系。
//  *   重力分量在旋转投影中自动消除，无需额外去重力步骤（数学推导验证）。
//  *   相比原 AccelCalibration_GetLevelAccelMps2 路径（原始未滤波acc），
//  *   可消除飞行中电机振动导致的 ±100~300 cm/s² 污染。
//  */
// void Pos_Est_Update_1000HZ(void)
// {
//     float acc_sensor[3];
//     float acc_body[3];
//     float sp;
//     float cp;
//     float sr;
//     float cr;

//     FlowGyroDecoupler_Push1000Hz(tick_1000us_cnt, g_imudata_250hz.gyrox, g_imudata_250hz.gyroy);

//     /* 取陷波+低通滤波后的传感器系加速度（单位 g，已校准零偏/尺度） */
//     acc_sensor[0] = g_imufilter_1000hz.accx;
//     acc_sensor[1] = g_imufilter_1000hz.accy;
//     acc_sensor[2] = g_imufilter_1000hz.accz;
//     // AccelCalibration_RotateImuToBody(acc_sensor, acc_body);
//     // 不用上面那个函数脱裤子放屁了，直接当传感器系就是机体系，毕竟没有安装误差
//     acc_body[0] = acc_sensor[0];
//     acc_body[1] = acc_sensor[1];
//     acc_body[2] = acc_sensor[2];

//     sp = g_euler.sin_pitch;
//     cp = g_euler.cos_pitch;
//     sr = g_euler.sin_roll;
//     cr = g_euler.cos_roll;

//     /* 旋转到水平系（yaw=0近似），重力自动消除。
//      * level_x = cp*body_x + sp*sr*body_y + sp*cr*body_z  (前进为正)
//      * level_y = cr*body_y - sr*body_z                     (向右为正)
//      * 单位 g → cm/s^2 */
//     acc_x_lp = (cp * acc_body[0] + sp * sr * acc_body[1] + sp * cr * acc_body[2]) * 9.80665f * 100.0f;
//     acc_y_lp = (cr * acc_body[1] - sr * acc_body[2]) * 9.80665f * 100.0f;

//     // 需要注意的是acc_y_lp右正左负，acc_x_lp前正后负
//     vel_x_pred -= acc_y_lp * 0.001f;
//     vel_y_pred += acc_x_lp * 0.001f;

// }

// /*
//  * 函数名: Pos_Est_Update_50HZ
//  * 功能: 更新 PMW3901 光流速度，并执行速度融合与位置积分
//  * 输入参数: 无
//  * 返回值: 无
//  */
// void Pos_Est_Update_50HZ(void)
// {
//     float dec_x;
//     float dec_y;
//     float height;
//     float coeff;

//     PMW3901_Update_50HZ();
//     FlowGyroDecoupler_Update50Hz(tick_1000us_cnt, g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY);
//     dec_x = FlowGyroDecoupler_GetDecX();
//     dec_y = FlowGyroDecoupler_GetDecY();
//     height = g_tof_fused_height_mm * 0.001f;

//     /* 1m 高度下 1 像素约对应 0.2131946 cm 位移，同时对过低高度做保护 */
//     coeff = 0.2131946f * (height - 0.05f);
//     if (height < 0.2f)
//     {
//         coeff = 0.0f;dec_x = 0.0f;dec_y = 0.0f;
//     }

//     /* 计算光流速度，此刻加入高度补偿，单位 cm/s */
//     opflow_vel_x = dec_x * coeff * 50.0f;
//     opflow_vel_y = dec_y * coeff * 50.0f;

//     /* squal 有效性门控：光流质量过低时不参与融合 */
//     uint8 opflow_valid = (g_pmw3901_raw.squal >= 25U) && (height >= 0.2f);
//     float k_use = opflow_valid ? g_fc_params.pos_est_k_flow : 0.0f;

//     /* 用光流测量对惯性预测做互补校正 */
//     Pos_Est_vel_x = vel_x_pred + k_use * (opflow_vel_x - vel_x_pred);
//     Pos_Est_vel_y = vel_y_pred + k_use * (opflow_vel_y - vel_y_pred);

//     /* 光流失效时对融合速度施加摩擦衰减，防止加速度偏置积分漂移 */
//     if (!opflow_valid)
//     {
//         Pos_Est_vel_x *= 0.96f;
//         Pos_Est_vel_y *= 0.96f;
//     }

//     vel_x_pred = Pos_Est_vel_x;
//     vel_y_pred = Pos_Est_vel_y;

//     wifi_justfloat(tick_1000us_cnt,
//                         acc_x_lp, acc_y_lp,
//                         opflow_vel_x, opflow_vel_y,
//                         Pos_Est_vel_x, Pos_Est_vel_y,
//                         g_pmw3901_raw.squal, height, lc302_data.flow_x_integral, lc302_data.flow_y_integral);

// }

#include "Pos_Est.h"
#include "FlowGyroDecoupler_LC302.h"
#include "../Attitude/Accel_Calibration.h"
#include "../Attitude/IMU_Filtter.h"
#include "../Height_Est/Height_Est.h"
// #include "HW_Drivers/PMW3901/PMW3901.h"
#include "HW_Drivers/LC302/LC302.h"
#include "HW_Drivers/LC302/LC302_Aux.h"
#include "HW_Drivers/ICM42688/ICM42688.h"
#include "filter.h"
#include "FlightController/fc_mode.h"
#include "FlightController/fc_params.h"

extern volatile uint32 tick_1000us_cnt;

/* 50Hz 末端速度一阶低通截止频率约 15.1Hz，对应 alpha = 0.85 */
#define POS_EST_VEL_OUT_LPF_ALPHA (0.85f)
/* 50Hz 光流速度一阶低通截止频率约 15Hz，对应 alpha ≈ 0.8482 */
#define POS_EST_OPFLOW_VEL_LPF_ALPHA (0.84816420f)

/* 1000Hz 水平加速度相位补偿低通 alpha，截止频率 fc≈5.00Hz */
#define POS_EST_RAW_ACC_LPF_ALPHA (0.11163521f)
/* 1000Hz 加速度速度预测积分步长，单位 s */
#define POS_EST_ACC_DT_S (0.001f)
/* 前后轴水平加速度限幅，单位 cm/s^2 */
#define POS_EST_ACC_FWD_LIMIT_CMSS (600.0f)
/* 左右轴水平加速度限幅，单位 cm/s^2 */
#define POS_EST_ACC_RIGHT_LIMIT_CMSS (600.0f)
/* 光流解算得到的 X 轴速度，单位 cm/s，往左飞为正，往右飞为负 */
float opflow_vel_x = 0.0f;
/* 光流解算得到的 Y 轴速度，单位 cm/s，往前飞为正，往后飞为负 */
float opflow_vel_y = 0.0f;
/* 光流解算得到的 X 轴速度，单位 cm/s，往左飞为正，往右飞为负 经过4HZ低通*/
float opflow_vel_x_lpf = 0.0f;
/* 光流解算得到的 Y 轴速度，单位 cm/s，往前飞为正，往后飞为负 经过4HZ低通*/
float opflow_vel_y_lpf = 0.0f;
/* 位置估计的 X 轴速度，单位 cm/s */
float Pos_Est_vel_x = 0.0f;
/* 位置估计的上一拍 X 轴速度，单位 cm/s */
float Pos_Est_vel_x_last = 0.0f;
/* 位置估计的 Y 轴速度，单位 cm/s */
float Pos_Est_vel_y = 0.0f;
/* 位置估计的上一拍 Y 轴速度，单位 cm/s */
float Pos_Est_vel_y_last = 0.0f;

/* 位置估计的 X 轴位置，单位 cm，往左飞为正，往右飞为负 */
float Pos_Est_pos_x = 0.0f;
/* 位置估计的上一拍 X 轴位置，单位 cm */
float Pos_Est_pos_x_last = 0.0f;
/* 位置估计的 Y 轴位置，单位 cm，往前飞为正，往后飞为负 */
float Pos_Est_pos_y = 0.0f;
/* 位置估计的上一拍 Y 轴位置，单位 cm */
float Pos_Est_pos_y_last = 0.0f;

/* X 轴光流速度一阶低通状态 */
static LPF1_t s_opflow_vel_lp_x;
/* Y 轴光流速度一阶低通状态 */
static LPF1_t s_opflow_vel_lp_y;
/* X 轴速度预测状态，单位 cm/s */
static float s_vel_pred_x = 0.0f;
/* Y 轴速度预测状态，单位 cm/s */
static float s_vel_pred_y = 0.0f;
static float s_raw_acc_lp_x = 0.0f;
static float s_raw_acc_lp_y = 0.0f;

/* X 轴加速度 单位 cm/s^2，飞机往前加速为正，往后加速为负 */
float acc_x_temp = 0.0f;
/* Y 轴加速度 单位 cm/s^2，飞机往右加速为正，往左加速为负 */
float acc_y_temp = 0.0f;
float acc_x_lp = 0.0f;
float acc_y_lp = 0.0f;

/*
 * 函数名: Pos_Est_Init
 * 功能: 初始化光流位置估计模块和相关滤波器
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Init(void)
{
    // PMW3901_Init();
    LC302_Init();
    // LC302_Init_Aux();
    // FlowGyroDecoupler_Init();
    FlowGyroDecoupler_LC302_Init();
    LPF1_Init(&s_opflow_vel_lp_x, POS_EST_OPFLOW_VEL_LPF_ALPHA);
    LPF1_Init(&s_opflow_vel_lp_y, POS_EST_OPFLOW_VEL_LPF_ALPHA);

    opflow_vel_x = 0.0f;
    opflow_vel_y = 0.0f;
    opflow_vel_x_lpf = 0.0f;
    opflow_vel_y_lpf = 0.0f;
    Pos_Est_vel_x = 0.0f;
    Pos_Est_vel_y = 0.0f;
    Pos_Est_vel_x_last = 0.0f;
    Pos_Est_vel_y_last = 0.0f;
    Pos_Est_pos_x = 0.0f;
    Pos_Est_pos_y = 0.0f;
    Pos_Est_pos_x_last = 0.0f;
    Pos_Est_pos_y_last = 0.0f;
    s_vel_pred_x = 0.0f;
    s_vel_pred_y = 0.0f;
    s_raw_acc_lp_x = 0.0f;
    s_raw_acc_lp_y = 0.0f;
}

/*
 * 函数名: Pos_Est_Reinit
 * 功能: 重置光流和滤波器内部状态
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Reinit(void)
{
    // FlowGyroDecoupler_Reinit();
    FlowGyroDecoupler_LC302_Reinit();
    LPF1_Reset(&s_opflow_vel_lp_x);
    LPF1_Reset(&s_opflow_vel_lp_y);

    acc_x_temp = 0.0f;
    acc_y_temp = 0.0f;
    opflow_vel_x = 0.0f;
    opflow_vel_y = 0.0f;
    opflow_vel_x_lpf = 0.0f;
    opflow_vel_y_lpf = 0.0f;
    Pos_Est_vel_x = 0.0f;
    Pos_Est_vel_y = 0.0f;
    Pos_Est_vel_x_last = 0.0f;
    Pos_Est_vel_y_last = 0.0f;
    Pos_Est_pos_x = 0.0f;
    Pos_Est_pos_y = 0.0f;
    Pos_Est_pos_x_last = 0.0f;
    Pos_Est_pos_y_last = 0.0f;
    s_vel_pred_x = 0.0f;
    s_vel_pred_y = 0.0f;
    s_raw_acc_lp_x = 0.0f;
    s_raw_acc_lp_y = 0.0f;
}

/*
 * 函数名: Pos_Est_Update_1000HZ
 * 功能: 推送光流去旋转解耦所需陀螺数据，并更新水平加速度低通输出
 * 输入参数: 无
 * 返回值: 无
 *
 * 加速度数据路径说明：
 *   使用 g_imufilter_1000hz（已经过陷波161Hz/320Hz + 12Hz低通滤波）作为输入，
 *   通过 AccelCalibration_RotateImuToBody 旋转到机体系后，利用旋转矩阵投影到水平系。
 *   重力分量在旋转投影中自动消除，无需额外去重力步骤（数学推导验证）。
 *   相比原 AccelCalibration_GetLevelAccelMps2 路径（原始未滤波acc），
 *   可消除飞行中电机振动导致的 ±100~300 cm/s² 污染。
 */
void Pos_Est_Update_1000HZ(void)
{
    float acc_sensor[3];
    float acc_body[3];
    float sp;
    float cp;
    float sr;
    float cr;

    // FlowGyroDecoupler_Push1000Hz(tick_1000us_cnt, g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy);
    FlowGyroDecoupler_LC302_Push1000Hz(tick_1000us_cnt, g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy);

    /* Use filtered IMU accel, then rotate to body frame and apply level-accel LPF. */
    acc_sensor[0] = g_imufilter_1000hz.accx;
    acc_sensor[1] = g_imufilter_1000hz.accy;
    acc_sensor[2] = g_imufilter_1000hz.accz;
    AccelCalibration_RotateImuToBody(acc_sensor, acc_body);

    sp = g_euler.sin_pitch;
    cp = g_euler.cos_pitch;
    sr = g_euler.sin_roll;
    cr = g_euler.cos_roll;

    /* 旋转到水平系（yaw=0近似），重力自动消除。
     * level_x = cp*body_x + sp*sr*body_y + sp*cr*body_z  (前进为正)
     * level_y = cr*body_y - sr*body_z                     (向右为正)
     * 单位 g → cm/s^2 */
    acc_x_temp = (cp * acc_body[0] + sp * sr * acc_body[1] + sp * cr * acc_body[2]) * 9.80665f * 100.0f;
    acc_y_temp = (cr * acc_body[1] - sr * acc_body[2]) * 9.80665f * 100.0f;
    s_raw_acc_lp_x += POS_EST_RAW_ACC_LPF_ALPHA * (acc_x_temp - s_raw_acc_lp_x);
    s_raw_acc_lp_y += POS_EST_RAW_ACC_LPF_ALPHA * (acc_y_temp - s_raw_acc_lp_y);
    acc_x_lp = s_raw_acc_lp_x;
    acc_y_lp = s_raw_acc_lp_y;

    /* IMU 冲击窗口内不让水平加速度污染后续速度积分 */
    if (g_imu_shock_flag != 0U)
    {
        acc_x_temp = 0.0f;
        acc_y_temp = 0.0f;
        acc_x_lp = 0.0f;
        acc_y_lp = 0.0f;
        s_raw_acc_lp_x = 0.0f;
        s_raw_acc_lp_y = 0.0f;
    }

    if (acc_x_lp > POS_EST_ACC_FWD_LIMIT_CMSS)
    {
        acc_x_lp = POS_EST_ACC_FWD_LIMIT_CMSS;
    }
    else if (acc_x_lp < -POS_EST_ACC_FWD_LIMIT_CMSS)
    {
        acc_x_lp = -POS_EST_ACC_FWD_LIMIT_CMSS;
    }

    if (acc_y_lp > POS_EST_ACC_RIGHT_LIMIT_CMSS)
    {
        acc_y_lp = POS_EST_ACC_RIGHT_LIMIT_CMSS;
    }
    else if (acc_y_lp < -POS_EST_ACC_RIGHT_LIMIT_CMSS)
    {
        acc_y_lp = -POS_EST_ACC_RIGHT_LIMIT_CMSS;
    }

    if (g_imu_shock_flag == 0U)
    {
        s_vel_pred_x -= acc_y_lp * POS_EST_ACC_DT_S;
        s_vel_pred_y += acc_x_lp * POS_EST_ACC_DT_S;
    }

    // float dec_x_pmw3901 = FlowGyroDecoupler_GetDecX();
    // float dec_y_pmw3901 = FlowGyroDecoupler_GetDecY();
    // float dec_x_lc302 = FlowGyroDecoupler_LC302_GetDecX();
    // float dec_y_lc302 = FlowGyroDecoupler_LC302_GetDecY();
    // FC_START_CRSF_state_e FC_START_CRSF_state = FC_START_CRSF_Get_State();
    // float dec_x, dec_y;
    // dec_x = FlowGyroDecoupler_LC302_GetDecX();
    // dec_y = FlowGyroDecoupler_LC302_GetDecY();
    // wifi_justfloat(tick_1000us_cnt,
    //                acc_x_temp, acc_y_temp,
    //                g_tof_fused_height_mm * 0.001f,
    //                lc302_data.flow_x_integral, lc302_data.flow_y_integral,
    //                g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy, g_imufilter_1000hz.gyroz, g_euler.pitch, g_euler.roll, g_euler.yaw,
    //                dec_x, dec_y, lc302_data.integration_timespan, lc302_data.valid);
    float dec_x;
    float dec_y;
    dec_x = FlowGyroDecoupler_LC302_GetDecX();
    dec_y = FlowGyroDecoupler_LC302_GetDecY();
    
    wifi_justfloat(tick_1000us_cnt,
                   acc_x_temp, acc_y_temp,
                   ICM42688.acc_x, ICM42688.acc_y, ICM42688.acc_z,
                   acc_x_lp, acc_y_lp,
                   g_tof_fused_height_mm * 0.001f,
                   lc302_data.flow_x_integral, lc302_data.flow_y_integral,
                   dec_x, dec_y,
                   opflow_vel_x, opflow_vel_y,
                   Pos_Est_vel_x, Pos_Est_vel_y,
                   g_mode2_velx_target, g_mode2_vely_target,
                   g_mode2_velx_pid.p_term, g_mode2_velx_pid.i_term, g_mode2_velx_pid.d_term, g_mode2_velx_pid.output,
                   //    opflow_vel_x_lpf, opflow_vel_y_lpf,
                   pitch_angle_target, roll_angle_target,
                   g_euler.pitch, g_euler.roll, g_euler.yaw);
}

/*
 * 函数名: Pos_Est_Update_50HZ
 * 功能: 更新 LC302 光流速度，并执行速度融合与位置积分
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Update_50HZ(void)
{
    float dec_x;
    float dec_y;
    float height_mm;
    float height_m;
    float k_flow_eff;
    uint8_t opflow_valid;
    float vel_x_pred;
    float vel_y_pred;
    const float dt = 0.02f;
    float innovation_x;
    float innovation_y;

    // PMW3901_Update_50HZ();
    LC302_Update_50HZ();
    // LC302_Update_50HZ_Aux();
    // FlowGyroDecoupler_Update50Hz(tick_1000us_cnt, g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY);
    (void)FlowGyroDecoupler_LC302_Update50Hz(tick_1000us_cnt, lc302_data.flow_x_integral, lc302_data.flow_y_integral, lc302_data.valid);
    dec_x = FlowGyroDecoupler_LC302_GetDecX();
    dec_y = FlowGyroDecoupler_LC302_GetDecY();
    height_mm = g_tof_fused_height_mm;
    if (height_mm > VL53L1X_VALID_RANGE_MAX)
    {
        height_mm = VL53L1X_VALID_RANGE_MAX;
    }
    else if (height_mm < 200.0f)
    {
        height_mm = 200.0f;
    }
    height_m = height_mm * 0.001f;
    if (height_m < 0.20f)
    {
        k_flow_eff = 0.0f;
    }
    else if (height_m < 0.50f)
    {
        k_flow_eff = g_fc_params.pos_est_k_flow * (height_m - 0.20f) / 0.30f;
    }
    else
    {
        k_flow_eff = g_fc_params.pos_est_k_flow;
    }
    opflow_valid = (height_mm >= 200.0f);

    /* 计算光流速度，此刻加入高度补偿，单位 cm/s */
    if (opflow_valid != 0U)
    {
        opflow_vel_x = (height_mm * 0.001f) * dec_x * 0.48076923f;
        opflow_vel_y = (height_mm * 0.001f) * dec_y * 0.48076923f;
        opflow_vel_x_lpf = LPF1_Update(&s_opflow_vel_lp_x, opflow_vel_x);
        opflow_vel_y_lpf = LPF1_Update(&s_opflow_vel_lp_y, opflow_vel_y);
    }
    /* IMU 冲击窗口内冻结加速度预测，只保留光流校正 */
    if (g_imu_shock_flag != 0U)
    {
        vel_x_pred = Pos_Est_vel_x;
        vel_y_pred = Pos_Est_vel_y;
    }
    else
    {
        /* X: 光流左正右负，acc_y 右正左负，所以预测项取负 */
        vel_x_pred = s_vel_pred_x;
        /* Y: 光流前正后负，acc_x 前正后负，所以预测项同号 */
        vel_y_pred = s_vel_pred_y;
    }

    Pos_Est_vel_x_last = Pos_Est_vel_x;
    Pos_Est_vel_y_last = Pos_Est_vel_y;

    if (opflow_valid != 0U)
    {
        innovation_x = opflow_vel_x_lpf - vel_x_pred;
        innovation_y = opflow_vel_y_lpf - vel_y_pred;
        if (innovation_x > 100.0f)
        {
            innovation_x = 100.0f;
        }
        else if (innovation_x < -100.0f)
        {
            innovation_x = -100.0f;
        }

        if (innovation_y > 100.0f)
        {
            innovation_y = 100.0f;
        }
        else if (innovation_y < -100.0f)
        {
            innovation_y = -100.0f;
        }

        Pos_Est_vel_x = vel_x_pred + k_flow_eff * innovation_x;
        Pos_Est_vel_y = vel_y_pred + k_flow_eff * innovation_y;
    }
    else
    {
        Pos_Est_vel_x = vel_x_pred;
        Pos_Est_vel_y = vel_y_pred;
    }

    s_vel_pred_x = Pos_Est_vel_x;
    s_vel_pred_y = Pos_Est_vel_y;

    Pos_Est_pos_x_last = Pos_Est_pos_x;
    Pos_Est_pos_y_last = Pos_Est_pos_y;

    Pos_Est_pos_x = Pos_Est_pos_x_last + 0.5f * (Pos_Est_vel_x_last + Pos_Est_vel_x) * dt;
    Pos_Est_pos_y = Pos_Est_pos_y_last + 0.5f * (Pos_Est_vel_y_last + Pos_Est_vel_y) * dt;
}
