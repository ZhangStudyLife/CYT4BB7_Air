#ifndef POS_EST_H
#define POS_EST_H

#include "zf_common_headfile.h"
// #include "FlowGyroDecoupler.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void Pos_Est_Init(void);
    void Pos_Est_Reinit(void);
    void Pos_Est_Update_1000HZ(void);
    void Pos_Est_Update_50HZ(void);

#ifdef __cplusplus
}
#endif

extern float opflow_vel_x;
extern float opflow_vel_y;
extern float opflow_vel_x_lpf;
extern float opflow_vel_y_lpf;
extern float Pos_Est_vel_x;
extern float Pos_Est_vel_y;
extern float Pos_Est_vel_x_level;
extern float Pos_Est_vel_y_level;
extern float acc_x_lp;
extern float acc_y_lp;
extern float acc_x_temp;
extern float acc_y_temp;

#endif /* POS_EST_H */
