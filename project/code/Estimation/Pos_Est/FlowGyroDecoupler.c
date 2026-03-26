#include "FlowGyroDecoupler.h"

/* 1000Hz采样下15Hz一阶低通系数 */
#define FLOW_GYRO_LPF_ALPHA   (0.05f)
/* 光流拟合系数 */
#define FLOW_GYRO_FIT_K       (10.0f)
/* 1000Hz积分步长，单位s */
#define FLOW_GYRO_DT_S        (0.001f)

/* X轴陀螺低通状态 */
static float s_gyro_lpf_x = 0.0f;
/* Y轴陀螺低通状态 */
static float s_gyro_lpf_y = 0.0f;
/* X轴窗口累计补偿量 */
static float s_fit_sum_x = 0.0f;
/* Y轴窗口累计补偿量 */
static float s_fit_sum_y = 0.0f;
/* X轴去耦后的光流增量 */
static float s_dec_x = 0.0f;
/* Y轴去耦后的光流增量 */
static float s_dec_y = 0.0f;

/*
 * 函数功能：初始化光流去陀螺补偿状态
 * 输入参数：无
 * 返回值：无
 */
void FlowGyroDecoupler_Init(void)
{
    s_gyro_lpf_x = 0.0f;
    s_gyro_lpf_y = 0.0f;
    s_fit_sum_x  = 0.0f;
    s_fit_sum_y  = 0.0f;
    s_dec_x      = 0.0f;
    s_dec_y      = 0.0f;
}

/*
 * 函数功能：重新初始化光流去陀螺补偿状态
 * 输入参数：无
 * 返回值：无
 */
void FlowGyroDecoupler_Reinit(void)
{
    FlowGyroDecoupler_Init();
}

/*
 * 函数功能：在1000Hz下对陀螺数据低通并累计窗口补偿量
 * 输入参数：t_ms-时间戳；gyro_x-X轴角速度；gyro_y-Y轴角速度
 * 返回值：无
 */
void FlowGyroDecoupler_Push1000Hz(uint32 t_ms, float gyro_x, float gyro_y)
{
    (void)t_ms;

    /* 对输入角速度做15Hz一阶低通 */
    s_gyro_lpf_x += FLOW_GYRO_LPF_ALPHA * (gyro_x - s_gyro_lpf_x);
    s_gyro_lpf_y += FLOW_GYRO_LPF_ALPHA * (gyro_y - s_gyro_lpf_y);

    /* 按拟合公式累计当前50Hz窗口内的光流等效补偿量 */
    s_fit_sum_x += FLOW_GYRO_FIT_K * s_gyro_lpf_x * FLOW_GYRO_DT_S;
    s_fit_sum_y += FLOW_GYRO_FIT_K * s_gyro_lpf_y * FLOW_GYRO_DT_S;
}

/*
 * 函数功能：在50Hz下用窗口累计补偿量对光流增量做最简去耦
 * 输入参数：t_read_ms-光流时间戳；delta_x-X轴光流增量；delta_y-Y轴光流增量
 * 返回值：1表示本次已完成更新
 */
uint8 FlowGyroDecoupler_Update50Hz(uint32 t_read_ms, int16_t delta_x, int16_t delta_y)
{
    (void)t_read_ms;

    /* 光流增量直接减去本窗口累计补偿量 */
    s_dec_x = (float)delta_x - s_fit_sum_x;
    s_dec_y = (float)delta_y - s_fit_sum_y;

    /* 清零窗口累计，等待下一轮1000Hz重新累计 */
    s_fit_sum_x = 0.0f;
    s_fit_sum_y = 0.0f;

    return 1U;
}

/*
 * 函数功能：获取X轴去耦后的光流增量
 * 输入参数：无
 * 返回值：X轴去耦后的光流增量
 */
float FlowGyroDecoupler_GetDecX(void)
{
    return s_dec_x;
}

/*
 * 函数功能：获取Y轴去耦后的光流增量
 * 输入参数：无
 * 返回值：Y轴去耦后的光流增量
 */
float FlowGyroDecoupler_GetDecY(void)
{
    return s_dec_y;
}
