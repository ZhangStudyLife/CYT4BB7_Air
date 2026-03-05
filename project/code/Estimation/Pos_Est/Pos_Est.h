#ifndef POS_EST_H
#define POS_EST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define POS_EST_DEG2RAD (0.017453292519943295f)  /* 角度转弧度的常数 */
#define POS_EST_RAD2DEG (57.29577951308232f)    /* 弧度转角度的常数 */
#define POS_EST_SQUAL_MIN 20U              /* 光流有效的最小表面质量指标 */


typedef struct
{
    float position_x_m;
    float position_y_m;
    float velocity_x_mps;
    float velocity_y_mps;
    float height_m;
    uint8_t flow_valid;
} PosEstOutput_t;

typedef struct
{
    int16_t deltaX;          // flow原始像素增量，向右移动为正 向左移动为负
    int16_t deltaY;          // flow原始像素增量，向前移动为正 向后移动为负
    uint8_t squal;           // 表面质量指标，0~255，越大质量越好
    float pixel_flow_X;      // 像素流动指标
    float pixel_flow_Y;      // 像素流动指标

} PosEstDebug_t;

extern volatile PosEstOutput_t g_pos_est_output;
extern volatile PosEstDebug_t g_pos_est_debug;

/**
 * @brief  位置估计模块初始化
 *         清零所有状态变量、输出和调试结构体
 */
void Pos_Est_Init(void);

/**
 * @brief  2000Hz陀螺仪角度积分更新
 *         在每个2000Hz周期累积 pitch/roll 轴角位移(deg)，
 *         等效于对角速度做矩形窗平均滤波(20点@2000Hz)，
 *         在100Hz周期中读取并重置累积值，供光流姿态解耦补偿使用
 */
void Pos_Est_Update_2000HZ(void);

/**
 * @brief  250Hz加速度计积分更新
 *         执行加速度偏置估计、振动加权、速度/位置积分
 */
void Pos_Est_Update_250HZ(void);

/**
 * @brief  100Hz光流融合更新
 *         读取光流传感器，进行姿态解耦补偿，与加速度计积分融合
 */
void Pos_Est_Update_50HZ(void);

#ifdef __cplusplus
}
#endif

#endif
