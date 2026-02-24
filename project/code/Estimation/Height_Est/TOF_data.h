#ifndef TOF_DATA_H_
#define TOF_DATA_H_

#include "../../HW_Drivers/VL53L1X/VL53L1X.h"


#ifdef __cplusplus
extern "C" {
#endif

extern uint16 g_tof_fused_height_mm;
extern uint16 g_tof2_height_mm;
extern uint16 g_tof3_height_mm;
extern uint8 g_tof_fused_valid;
extern uint8 g_tof2_valid;
extern uint8 g_tof3_valid;
extern uint8 g_tof2_used_in_fusion;
extern uint8 g_tof3_used_in_fusion;
extern uint8 g_tof_fused_source;

void TOF_Init(void);
void TOF_Update(void);
void TOF_update_100HZ(void);
void TOF_Calibrate(void);


#ifdef __cplusplus
}
#endif

#endif /* TOF_DATA_H_ */
