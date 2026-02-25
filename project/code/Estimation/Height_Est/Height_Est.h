#ifndef HEIGHT_EST_H_
#define HEIGHT_EST_H_

#include "TOF_data.h"
#include "Baro_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HEIGHT_EST_UPDATE_HZ                (100U)
#define HEIGHT_EST_SOURCE_NONE              (0U)
#define HEIGHT_EST_SOURCE_TOF               (1U)
#define HEIGHT_EST_SOURCE_BARO              (2U)
#define HEIGHT_EST_SOURCE_MIXED             (3U)

extern uint16 g_height_est_mm;
extern float g_height_est_m;
extern float g_height_vz_mps;
extern uint8 g_height_est_valid;
extern uint8 g_height_est_source;

void Height_Est_Init(void);
void Height_Est_Reset(void);
void Height_Est_Update_100HZ(void);

#ifdef __cplusplus
}
#endif

#endif /* HEIGHT_EST_H_ */

