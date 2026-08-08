#ifndef POS_EST_H
#define POS_EST_H

#include "zf_common_headfile.h"
// #include "FlowGyroDecoupler.h"

/* 最近一帧LC302原始积分和姿态解耦结果，用于飞行日志。 */
typedef struct
{
    float raw_flow_x_integral; /* LC302 X轴原始积分。 */
    float raw_flow_y_integral; /* LC302 Y轴原始积分。 */
    float decoupled_flow_x;    /* 姿态解耦后的X轴光流。 */
    float decoupled_flow_y;    /* 姿态解耦后的Y轴光流。 */
    float data_age_ms;         /* 距最近光流帧的时间，首帧前为-1，单位ms。 */
    uint8_t frame_valid;       /* 最近一帧光流有效标志。 */
} pos_est_flow_telemetry_t;

#ifdef __cplusplus
extern "C"
{
#endif

    void Pos_Est_Init(void);
    void Pos_Est_Update_1000HZ(void);

    /*
     * 函数功能：初始化去除车模平移速度影响的并行速度估计器。
     * 输入参数：无。
     * 返回值：无。
     */
    void Pos_Est_Init_2(void);

    /*
     * 函数功能：在1000Hz下更新去除车模平移速度影响的机体速度估计值。
     * 输入参数：无。
     * 返回值：无，结果写入Pos_Est_vel_x_2和Pos_Est_vel_y_2。
     */
    void Pos_Est_Update_1000HZ_2(void);

    /*
     * 函数功能：读取最近一帧LC302原始积分和姿态解耦光流快照。
     * 输入参数：telemetry - 输出快照指针。
     * 返回值：无，结果写入telemetry指向的结构体。
     */
    void Pos_Est_GetFlowTelemetry(pos_est_flow_telemetry_t *telemetry);

#ifdef __cplusplus
}
#endif

extern float Pos_Est_vel_x;
extern float Pos_Est_vel_y;

/* 去除车模平移速度影响后的机体X轴速度，左正，单位cm/s。 */
extern float Pos_Est_vel_x_2;
/* 去除车模平移速度影响后的机体Y轴速度，前正，单位cm/s。 */
extern float Pos_Est_vel_y_2;

#endif /* POS_EST_H */
