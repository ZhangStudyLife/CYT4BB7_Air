#include "Pos_Est.h"
#include "filter.h"

extern volatile uint32 tick_1000us_cnt;

/* 光流解算得到的 X 轴速度，单位 cm/s */ // 往左飞为正，往右飞为负
float opflow_vel_x = 0.0f;
/* 光流解算得到的 Y 轴速度，单位 cm/s */
float opflow_vel_y = 0.0f; // 往前飞为正，往后飞为负

/* 位置估计的 X 轴速度，单位 cm/s */
float Pos_Est_vel_x = 0.0f;
float Pos_Est_vel_x_last = 0.0f;
/* 位置估计的 Y 轴速度，单位 cm/s */
float Pos_Est_vel_y = 0.0f;
float Pos_Est_vel_y_last = 0.0f;

/* 位置估计的 X 轴位置，单位 cm */              // 往左飞为正 , 往右飞为负
float Pos_Est_pos_x = 0.0f;
float Pos_Est_pos_x_last = 0.0f;
/* 位置估计的 Y 轴位置，单位 cm */              //  往前飞为正，往后飞为负
float Pos_Est_pos_y = 0.0f;
float Pos_Est_pos_y_last = 0.0f;

/* X 轴速度 Kalman 滤波输出，单位 cm/s */
float Pos_Est_vel_x_kf = 0.0f;
/* Y 轴速度 Kalman 滤波输出，单位 cm/s */
float Pos_Est_vel_y_kf = 0.0f;

/* X 轴速度一维 Kalman 滤波器状态 */
static Kalman1D_t s_kf_x;
/* Y 轴速度一维 Kalman 滤波器状态 */
static Kalman1D_t s_kf_y;
/* X 轴加速度一阶低通状态 */
static LPF1_t s_acc_lp_x;
/* Y 轴加速度一阶低通状态 */
static LPF1_t s_acc_lp_y;

/* X 轴加速度一阶低通输出 单位CM/S^2*/
float acc_x_lp = 0.0f; // 飞机往前加速为正，往后加速为负
/* Y 轴加速度一阶低通输出 单位CM/S^2*/
float acc_y_lp = 0.0f; // 飞机往右加速为正，往左加速为负

/*
 * 函数名: Pos_Est_Init
 * 功能: 初始化光流位置估计模块和相关滤波器
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Init(void)
{
    PMW3901_Init();
    FlowGyroDecoupler_Init();
    Kalman1D_Init(&s_kf_x, 200.0f, 600.0f, 1.0f, 0.0f);
    Kalman1D_Init(&s_kf_y, 200.0f, 600.0f, 1.0f, 0.0f);

    /* 保留原有一阶低通输出 */
    LPF1_Init(&s_acc_lp_x, 0.04f);
    LPF1_Init(&s_acc_lp_y, 0.04f);
    Pos_Est_pos_x = 0.0f;
    Pos_Est_pos_y = 0.0f;
    Pos_Est_pos_x_last = 0.0f;
    Pos_Est_pos_y_last = 0.0f;
}

/*
 * 函数名: Pos_Est_Reinit
 * 功能: 重置光流和滤波器内部状态
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Reinit(void)
{
    PMW3901_ReInit();
    FlowGyroDecoupler_Reinit();
    Kalman1D_Reset(&s_kf_x);
    Kalman1D_Reset(&s_kf_y);
    LPF1_Reset(&s_acc_lp_x);
    LPF1_Reset(&s_acc_lp_y);
    acc_x_lp = 0.0f;
    acc_y_lp = 0.0f;
    Pos_Est_pos_x = 0.0f;
    Pos_Est_pos_y = 0.0f;
    Pos_Est_pos_x_last = 0.0f;
    Pos_Est_pos_y_last = 0.0f;
}

/*
 * 函数名: Pos_Est_Update_1000HZ
 * 功能: 推送光流角速度解耦数据，并更新加速度对比滤波输出
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Update_1000HZ(void)
{
    /* 保持原有 1kHz 陀螺推送，同时更新加速度一阶低通输出 */
    FlowGyroDecoupler_Push1000Hz(tick_1000us_cnt, g_imudata_250hz.gyrox, g_imudata_250hz.gyroy);
    float acc_x_temp, acc_y_temp;
    AccelCalibration_GetLevelAccelMps2(&acc_x_temp, &acc_y_temp, NULL);
    acc_x_temp *= 100.0f; // 转换为 cm/s^2
    acc_y_temp *= 100.0f; // 转换为 cm/s^2
    acc_x_lp = LPF1_Update(&s_acc_lp_x, acc_x_temp);
    acc_y_lp = LPF1_Update(&s_acc_lp_y, acc_y_temp);
}

/*
 * 函数名: Pos_Est_Update_50HZ
 * 功能: 更新 PMW3901 光流速度并执行速度 Kalman 滤波
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Update_50HZ(void)
{
    float dec_x;
    float dec_y;
    float height;
    float coeff;
    const float dt = 0.02f;     // 50Hz
    const float k_flow = 0.25f; // 光流校正权重

    PMW3901_Update_50HZ();
    FlowGyroDecoupler_Update50Hz(tick_1000us_cnt, g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY);
    dec_x = FlowGyroDecoupler_GetDecX();
    dec_y = FlowGyroDecoupler_GetDecY();
    height = g_tof_fused_height_mm / 1000.0f;

    /* 1m 高度下 1 像素对应约 0.2131946 cm 位移，并对低高度做保护 */
    coeff = 0.2131946f * (height - 0.05f);
    if (height < 0.2f)
    {
        coeff = 0.0f;
    }

    // 计算速度 此刻加入高度补偿，单位 cm/s
    opflow_vel_x = dec_x * coeff * 50.0f;
    opflow_vel_y = dec_y * coeff * 50.0f;

    // X: 光流速度是“左正右负”，acc_y 是“右正左负”，符号相反
    // 预测速度 , 拿取上一帧的 KF 输出作为当前帧的预测值，加入加速度积分预测下一帧速度的先验
    float vel_x_pred = Pos_Est_vel_x_kf - acc_y_lp * dt;

    // Y: 光流速度是“前正后负”，acc_x 也是“前正后负”，符号相同
    // 预测速度 , 拿取上一帧的 KF 输出作为当前帧的预测值，加入加速度积分预测下一帧速度的先验
    float vel_y_pred = Pos_Est_vel_y_kf + acc_x_lp * dt;

    Pos_Est_vel_x_last = Pos_Est_vel_x;
    Pos_Est_vel_y_last = Pos_Est_vel_y;

    // 本次测量的新的光流数据+预测速度 进行互补滤波，得到位置估计的速度输出
    Pos_Est_vel_x = vel_x_pred + k_flow * (opflow_vel_x - vel_x_pred);
    Pos_Est_vel_y = vel_y_pred + k_flow * (opflow_vel_y - vel_y_pred);

    // 加个低通滤波，平滑一下
    Pos_Est_vel_x_kf = Kalman1D_Update(&s_kf_x, Pos_Est_vel_x);
    Pos_Est_vel_y_kf = Kalman1D_Update(&s_kf_y, Pos_Est_vel_y);

    Pos_Est_pos_x_last = Pos_Est_pos_x;
    Pos_Est_pos_y_last = Pos_Est_pos_y;

    Pos_Est_pos_x = Pos_Est_pos_x_last + 0.5 * (Pos_Est_vel_x_last + Pos_Est_vel_x) * 0.02f;
    Pos_Est_pos_y = Pos_Est_pos_y_last + 0.5 * (Pos_Est_vel_y_last + Pos_Est_vel_y) * 0.02f;

    wifi_vofa_JustFloat(12u, opflow_vel_x, opflow_vel_y, acc_x_lp, acc_y_lp,
                        vel_x_pred, vel_y_pred, Pos_Est_vel_x,Pos_Est_vel_y,Pos_Est_vel_x_kf, Pos_Est_vel_y_kf,Pos_Est_pos_x,Pos_Est_pos_y);
}


