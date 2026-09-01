#include "FlowGyroDecoupler.h"

/* PMW3901 X 轴一阶低通 alpha，对应 fc=10Hz */
#define FLOW_GYRO_PMW3901_LPF_ALPHA_X   (0.059117f)
/* PMW3901 Y 轴一阶低通 alpha，对应 fc=7Hz */
#define FLOW_GYRO_PMW3901_LPF_ALPHA_Y   (0.042129f)
/* PMW3901 X 轴姿态补偿倍率 */
#define FLOW_GYRO_PMW3901_GAIN_X        (8.009662f)
/* PMW3901 Y 轴姿态补偿倍率 */
#define FLOW_GYRO_PMW3901_GAIN_Y        (9.120377f)
/* 1000Hz 积分步长，单位 s */
#define FLOW_GYRO_PMW3901_DT_S          (0.001f)

/* PMW3901 X 轴陀螺低通状态 */
static float s_gyro_lpf_x = 0.0f;
/* PMW3901 Y 轴陀螺低通状态 */
static float s_gyro_lpf_y = 0.0f;
/* PMW3901 X 轴当前 50Hz 窗口积分补偿量 */
static float s_window_sum_x = 0.0f;
/* PMW3901 Y 轴当前 50Hz 窗口积分补偿量 */
static float s_window_sum_y = 0.0f;
/* PMW3901 X 轴解耦后的光流增量 */
static float s_dec_x = 0.0f;
/* PMW3901 Y 轴解耦后的光流增量 */
static float s_dec_y = 0.0f;

/*
 * 函数功能：初始化 PMW3901 专用光流解耦模块
 * 输入参数：无
 * 返回值：无
 */
void FlowGyroDecoupler_Init(void)
{
    s_gyro_lpf_x = 0.0f;
    s_gyro_lpf_y = 0.0f;
    s_window_sum_x = 0.0f;
    s_window_sum_y = 0.0f;
    s_dec_x = 0.0f;
    s_dec_y = 0.0f;
}

/*
 * 函数功能：重置 PMW3901 专用光流解耦模块内部状态
 * 输入参数：无
 * 返回值：无
 */
void FlowGyroDecoupler_Reinit(void)
{
    FlowGyroDecoupler_Init();
}

/*
 * 函数功能：在 1000Hz 下推入陀螺数据，并累计本个 50Hz 窗口内的姿态补偿量
 * 输入参数：t_ms-当前毫秒时间戳；gyro_x-X 轴角速度；gyro_y-Y 轴角速度
 * 返回值：无
 */
void FlowGyroDecoupler_Push1000Hz(uint32 t_ms, float gyro_x, float gyro_y)
{
    (void)t_ms;

    /* 按拟合结论分别对 X/Y 轴角速度执行一阶低通 */
    s_gyro_lpf_x += FLOW_GYRO_PMW3901_LPF_ALPHA_X * (gyro_x - s_gyro_lpf_x);
    s_gyro_lpf_y += FLOW_GYRO_PMW3901_LPF_ALPHA_Y * (gyro_y - s_gyro_lpf_y);

    /* 在当前 50Hz 窗口内累计角速度积分补偿量 */
    s_window_sum_x += FLOW_GYRO_PMW3901_GAIN_X * s_gyro_lpf_x * FLOW_GYRO_PMW3901_DT_S;
    s_window_sum_y += FLOW_GYRO_PMW3901_GAIN_Y * s_gyro_lpf_y * FLOW_GYRO_PMW3901_DT_S;
}

/*
 * 函数功能：在 50Hz 下根据当前窗口的积分补偿量更新 PMW3901 解耦结果
 * 输入参数：t_read_ms-当前光流读取时刻；delta_x-X 轴原始光流增量；delta_y-Y 轴原始光流增量
 * 返回值：1 表示本次已完成更新
 */
uint8 FlowGyroDecoupler_Update50Hz(uint32 t_read_ms, int16_t delta_x, int16_t delta_y)
{
    (void)t_read_ms;

    /* 当前新帧直接减去上一窗口累计的姿态补偿量 */
    s_dec_x = (float)delta_x - s_window_sum_x;
    s_dec_y = (float)delta_y - s_window_sum_y;

    /* 清零窗口累计量，等待下一轮 1000Hz 重新积分 */
    s_window_sum_x = 0.0f;
    s_window_sum_y = 0.0f;

    return 1U;
}

/*
 * 函数功能：获取 PMW3901 X 轴解耦后的光流增量
 * 输入参数：无
 * 返回值：X 轴解耦后的光流增量
 */
float FlowGyroDecoupler_GetDecX(void)
{
    return s_dec_x;
}

/*
 * 函数功能：获取 PMW3901 Y 轴解耦后的光流增量
 * 输入参数：无
 * 返回值：Y 轴解耦后的光流增量
 */
float FlowGyroDecoupler_GetDecY(void)
{
    return s_dec_y;
}
