#ifndef HEIGHT_EST_H_
#define HEIGHT_EST_H_

#include "../../HW_Drivers/VL53L1X/VL53L1X.h"
#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

extern float g_tof_fused_height_mm;
extern float g_tof1_height_mm;
extern float g_tof4_height_mm;
extern uint8 g_tof_fused_valid;
extern float g_tof_fused_vz_mps;
extern float g_height_fused_vz_mps;
extern float g_acc_sum_vz_mps2;

void TOF_Init(void);
void TOF_update_100HZ(void);
void Height_Est_update_100HZ(void);
void Height_Est_update_1000HZ(void);

#ifdef __cplusplus
}
#endif

#endif
