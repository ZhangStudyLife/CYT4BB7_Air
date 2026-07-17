#ifndef POS_EST_H
#define POS_EST_H

#include "zf_common_headfile.h"
// #include "FlowGyroDecoupler.h"

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
