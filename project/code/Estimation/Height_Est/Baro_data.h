#ifndef BARO_DATA_H_
#define BARO_DATA_H_

#include "../../HW_Drivers/BMP388/BMP388.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BARO_UPDATE_HZ                  (100.0f)
#define BARO_UPDATE_DT_SEC              (1.0f / BARO_UPDATE_HZ)
#define BARO_CALIBRATION_SAMPLES        (200U)
#define BARO_TO_HEIGHT_SCALE_FACTOR     (8.3f)

extern float g_baro_ref_pressure;
extern float g_baro_altitude;
extern float g_baro_pressure_raw_pa;
extern float g_baro_pressure_filt_pa;
extern float g_baro_pressure_comp_pa;
extern float g_baro_prop_bias_hat_pa;
extern float g_baro_altitude_raw_m;
extern uint8 g_baro_sample_new;

void Baro_Init(void);
void Baro_Update(void);
void Baro_update_100HZ(void);
void Baro_Calibrate(void);

#ifdef __cplusplus
}
#endif

#endif /* BARO_DATA_H_ */
