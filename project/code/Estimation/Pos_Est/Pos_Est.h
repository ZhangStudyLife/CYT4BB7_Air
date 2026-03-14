#ifndef POS_EST_H
#define POS_EST_H

#include "zf_common_headfile.h"
#include "FlowGyroDecoupler.h"

#ifdef __cplusplus
extern "C" {
#endif

void Pos_Est_Init(void);
void Pos_Est_Reinit(void);
void Pos_Est_Update_1000HZ(void);
void Pos_Est_Update_50HZ(void);

#ifdef __cplusplus
}
#endif

extern float Pos_Est_vel_x;
extern float Pos_Est_vel_y;
extern float Pos_Est_vel_x_kf;
extern float Pos_Est_vel_y_kf;
extern float Pos_Est_pos_x;
extern float Pos_Est_pos_y;

#endif /* POS_EST_H */
