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

#ifdef __cplusplus
}
#endif

extern float Pos_Est_vel_x;
extern float Pos_Est_vel_y;

#endif /* POS_EST_H */
