#include "Pos_Est.h"
#include "filter.h"

extern volatile uint32 tick_1000us_cnt;

/* 光流解算得到的 X 轴速度，单位 cm/s，往左飞为正，往右飞为负 */
float opflow_vel_x = 0.0f;
/* 光流解算得到的 Y 轴速度，单位 cm/s，往前飞为正，往后飞为负 */
float opflow_vel_y = 0.0f;

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

/* X 轴速度 Kalman 输出，单位 cm/s */
float Pos_Est_vel_x_kf = 0.0f;
/* Y 轴速度 Kalman 输出，单位 cm/s */
float Pos_Est_vel_y_kf = 0.0f;

/* X 轴速度一维 Kalman 滤波器状态 */
static Kalman1D_t s_kf_x;
/* Y 轴速度一维 Kalman 滤波器状态 */
static Kalman1D_t s_kf_y;
/* X 轴加速度一阶低通状态 */
static LPF1_t s_acc_lp_x;
/* Y 轴加速度一阶低通状态 */
static LPF1_t s_acc_lp_y;

/* X 轴加速度一阶低通输出，单位 cm/s^2，飞机往前加速为正，往后加速为负 */
float acc_x_lp = 0.0f;
/* Y 轴加速度一阶低通输出，单位 cm/s^2，飞机往右加速为正，往左加速为负 */
float acc_y_lp = 0.0f;

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
    Kalman1D_Init(&s_kf_x, 200.0f, 900.0f, 1.0f, 0.0f);
    Kalman1D_Init(&s_kf_y, 200.0f, 900.0f, 1.0f, 0.0f);

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
 * 功能: 推送光流去旋转解耦所需陀螺数据，并更新水平加速度低通输出
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Update_1000HZ(void)
{
    float acc_x_temp;
    float acc_y_temp;

    FlowGyroDecoupler_Push1000Hz(tick_1000us_cnt, g_imudata_250hz.gyrox, g_imudata_250hz.gyroy);

    AccelCalibration_GetLevelAccelMps2(&acc_x_temp, &acc_y_temp, NULL);
    acc_x_temp *= 100.0f;
    acc_y_temp *= 100.0f;
    acc_x_lp = LPF1_Update(&s_acc_lp_x, acc_x_temp);
    acc_y_lp = LPF1_Update(&s_acc_lp_y, acc_y_temp);
}

/*
 * 函数名: Pos_Est_Update_50HZ
 * 功能: 更新 PMW3901 光流速度，并执行速度融合与位置积分
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Update_50HZ(void)
{
    float dec_x;
    float dec_y;
    float height;
    float coeff;
    float vel_x_pred;
    float vel_y_pred;
    const float dt = 0.02f;
    const float k_flow = 0.28f;

    PMW3901_Update_50HZ();
    FlowGyroDecoupler_Update50Hz(tick_1000us_cnt, g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY);
    dec_x = FlowGyroDecoupler_GetDecX();
    dec_y = FlowGyroDecoupler_GetDecY();
    height = g_tof_fused_height_mm / 1000.0f;

    /* 1m 高度下 1 像素约对应 0.2131946 cm 位移，同时对过低高度做保护 */
    coeff = 0.2131946f * (height - 0.05f);
    if (height < 0.2f)
    {
        coeff = 0.0f;
    }

    /* 计算光流速度，此刻加入高度补偿，单位 cm/s */
    opflow_vel_x = dec_x * coeff * 50.0f;
    opflow_vel_y = dec_y * coeff * 50.0f;

    /* X: 光流左正右负，acc_y 右正左负，所以预测项取负 */
    vel_x_pred = Pos_Est_vel_x_kf - acc_y_lp * dt;
    /* Y: 光流前正后负，acc_x 前正后负，所以预测项同号 */
    vel_y_pred = Pos_Est_vel_y_kf + acc_x_lp * dt;

    Pos_Est_vel_x_last = Pos_Est_vel_x;
    Pos_Est_vel_y_last = Pos_Est_vel_y;

    /* 用光流测量对惯性预测做互补校正 */
    Pos_Est_vel_x = vel_x_pred + k_flow * (opflow_vel_x - vel_x_pred);
    Pos_Est_vel_y = vel_y_pred + k_flow * (opflow_vel_y - vel_y_pred);

    /* 速度再做一层轻量 Kalman 平滑 */
    Pos_Est_vel_x_kf = Kalman1D_Update(&s_kf_x, Pos_Est_vel_x);
    Pos_Est_vel_y_kf = Kalman1D_Update(&s_kf_y, Pos_Est_vel_y);

    Pos_Est_pos_x_last = Pos_Est_pos_x;
    Pos_Est_pos_y_last = Pos_Est_pos_y;

    Pos_Est_pos_x = Pos_Est_pos_x_last + 0.5f * (Pos_Est_vel_x_last + Pos_Est_vel_x) * dt;
    Pos_Est_pos_y = Pos_Est_pos_y_last + 0.5f * (Pos_Est_vel_y_last + Pos_Est_vel_y) * dt;

    // wifi_justfloat_JustFloat(opflow_vel_x,
    //                         opflow_vel_y,
    //                         acc_x_lp,
    //                         acc_y_lp,
    //                         vel_x_pred,
    //                         vel_y_pred,
    //                         Pos_Est_vel_x,
    //                         Pos_Est_vel_y,
    //                         Pos_Est_vel_x_kf,
    //                         Pos_Est_vel_y_kf,
    //                         Pos_Est_pos_x,
    //                         Pos_Est_pos_y,
    //                         12u);
}
