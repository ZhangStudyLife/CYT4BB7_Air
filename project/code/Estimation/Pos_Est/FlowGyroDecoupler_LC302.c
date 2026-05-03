#include "FlowGyroDecoupler_LC302.h"

/* 1000Hz 积分步长，单位 s */
#define FLOW_GYRO_LC302_DT_S       (0.001f)
/* LC302 X 轴当前窗口角速度积分系数 */
#define FLOW_GYRO_LC302_KX0        (10.177451f)
/* LC302 X 轴上一窗口角速度积分系数 */
#define FLOW_GYRO_LC302_KX1        (50.349200f)
/* LC302 X 轴上二窗口角速度积分系数 */
#define FLOW_GYRO_LC302_KX2        (70.427565f)
/* LC302 X 轴上三窗口角速度积分系数 */
#define FLOW_GYRO_LC302_KX3        (56.809830f)
/* LC302 Y 轴当前窗口角速度积分系数 */
#define FLOW_GYRO_LC302_KY0        (-10.408851f)
/* LC302 Y 轴上一窗口角速度积分系数 */
#define FLOW_GYRO_LC302_KY1        (65.870257f)
/* LC302 Y 轴上二窗口角速度积分系数 */
#define FLOW_GYRO_LC302_KY2        (95.571516f)
/* LC302 Y 轴上三窗口角速度积分系数 */
#define FLOW_GYRO_LC302_KY3        (38.557333f)
/* LC302 X 轴静态偏置补偿量，单位像素/帧 */
#define FLOW_GYRO_LC302_BIAS_X     (-0.106741f)
/* LC302 Y 轴静态偏置补偿量，单位像素/帧 */
#define FLOW_GYRO_LC302_BIAS_Y     (-1.022975f)

/* LC302 X 轴当前 50Hz 窗口角速度积分量，单位 deg */
static float s_window_gyro_x = 0.0f;
/* LC302 Y 轴当前 50Hz 窗口角速度积分量，单位 deg */
static float s_window_gyro_y = 0.0f;
/* LC302 X 轴最近 4 个 50Hz 窗口角速度积分历史，单位 deg */
static float s_gyro_hist_x[4] = {0.0f, 0.0f, 0.0f, 0.0f};
/* LC302 Y 轴最近 4 个 50Hz 窗口角速度积分历史，单位 deg */
static float s_gyro_hist_y[4] = {0.0f, 0.0f, 0.0f, 0.0f};
/* LC302 X 轴解耦后的光流增量，单位像素/帧 */
static float s_dec_x = 0.0f;
/* LC302 Y 轴解耦后的光流增量，单位像素/帧 */
static float s_dec_y = 0.0f;

/*
 * 函数功能：初始化 LC302 专用光流解耦模块
 * 输入参数：无
 * 返回值：无
 */
void FlowGyroDecoupler_LC302_Init(void)
{
    s_window_gyro_x = 0.0f;
    s_window_gyro_y = 0.0f;
    s_gyro_hist_x[0] = 0.0f;
    s_gyro_hist_x[1] = 0.0f;
    s_gyro_hist_x[2] = 0.0f;
    s_gyro_hist_x[3] = 0.0f;
    s_gyro_hist_y[0] = 0.0f;
    s_gyro_hist_y[1] = 0.0f;
    s_gyro_hist_y[2] = 0.0f;
    s_gyro_hist_y[3] = 0.0f;
    s_dec_x = 0.0f;
    s_dec_y = 0.0f;
}

/*
 * 函数功能：重置 LC302 专用光流解耦模块内部状态
 * 输入参数：无
 * 返回值：无
 */
void FlowGyroDecoupler_LC302_Reinit(void)
{
    FlowGyroDecoupler_LC302_Init();
}

/*
 * 函数功能：在 1000Hz 下推入陀螺数据，并累计本个 50Hz 窗口内的角速度积分量
 * 输入参数：t_ms-当前毫秒时间戳；gyro_x-X 轴角速度，单位 deg/s；gyro_y-Y 轴角速度，单位 deg/s
 * 返回值：无
 */
void FlowGyroDecoupler_LC302_Push1000Hz(uint32 t_ms, float gyro_x, float gyro_y)
{
    (void)t_ms;

    /* 累计当前 50Hz 光流帧内的角速度积分量 */
    s_window_gyro_x += gyro_x * FLOW_GYRO_LC302_DT_S;
    s_window_gyro_y += gyro_y * FLOW_GYRO_LC302_DT_S;
}

/*
 * 函数功能：在 50Hz 下根据对角 FIR4 角速度模型更新 LC302 解耦结果
 * 输入参数：t_read_ms-当前光流读取时刻；delta_x-X 轴原始光流增量；delta_y-Y 轴原始光流增量
 * 返回值：1 表示本次已完成更新
 */
uint8 FlowGyroDecoupler_LC302_Update50Hz(uint32 t_read_ms, int16_t delta_x, int16_t delta_y)
{
    float comp_x;
    float comp_y;

    (void)t_read_ms;

    /* 更新最近 4 帧角速度积分历史，补偿 LC302 光流积分链路的时间滞后 */
    s_gyro_hist_x[3] = s_gyro_hist_x[2];
    s_gyro_hist_x[2] = s_gyro_hist_x[1];
    s_gyro_hist_x[1] = s_gyro_hist_x[0];
    s_gyro_hist_x[0] = s_window_gyro_x;
    s_gyro_hist_y[3] = s_gyro_hist_y[2];
    s_gyro_hist_y[2] = s_gyro_hist_y[1];
    s_gyro_hist_y[1] = s_gyro_hist_y[0];
    s_gyro_hist_y[0] = s_window_gyro_y;

    /* 用对角 FIR4 模型估计旋转造成的光流分量 */
    comp_x = FLOW_GYRO_LC302_BIAS_X +
             FLOW_GYRO_LC302_KX0 * s_gyro_hist_x[0] +
             FLOW_GYRO_LC302_KX1 * s_gyro_hist_x[1] +
             FLOW_GYRO_LC302_KX2 * s_gyro_hist_x[2] +
             FLOW_GYRO_LC302_KX3 * s_gyro_hist_x[3];
    comp_y = FLOW_GYRO_LC302_BIAS_Y +
             FLOW_GYRO_LC302_KY0 * s_gyro_hist_y[0] +
             FLOW_GYRO_LC302_KY1 * s_gyro_hist_y[1] +
             FLOW_GYRO_LC302_KY2 * s_gyro_hist_y[2] +
             FLOW_GYRO_LC302_KY3 * s_gyro_hist_y[3];

    s_dec_x = (float)delta_x - comp_x;
    s_dec_y = (float)delta_y - comp_y;

    /* 清空当前窗口积分，等待下一帧重新累计 */
    s_window_gyro_x = 0.0f;
    s_window_gyro_y = 0.0f;

    return 1U;
}

/*
 * 函数功能：获取 LC302 X 轴解耦后的光流增量
 * 输入参数：无
 * 返回值：X 轴解耦后的光流增量
 */
float FlowGyroDecoupler_LC302_GetDecX(void)
{
    return s_dec_x;
}

/*
 * 函数功能：获取 LC302 Y 轴解耦后的光流增量
 * 输入参数：无
 * 返回值：Y 轴解耦后的光流增量
 */
float FlowGyroDecoupler_LC302_GetDecY(void)
{
    return s_dec_y;
}
