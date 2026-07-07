#ifndef BEACON_LOST_DETECTOR_H
#define BEACON_LOST_DETECTOR_H

#include "zf_common_typedef.h"
#include "../Image/image_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BEACON_LOST_SQUARE_LEFT_PX       (25.0f)
#define BEACON_LOST_SQUARE_RIGHT_PX      (25.0f)
#define BEACON_LOST_SQUARE_UP_PX         (15.0f)
#define BEACON_LOST_SQUARE_DOWN_PX       (30.0f)
#define BEACON_LOST_DISAPPEAR_RADIUS_PX  (10.0f)

extern uint8 g_beacon_lost_flag;

void BeaconLostDetector_Init(void);
uint8 BeaconLostDetector_Update(void);
uint8 BeaconLostDetector_GetFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* BEACON_LOST_DETECTOR_H */
