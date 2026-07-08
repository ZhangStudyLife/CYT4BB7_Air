#ifndef PIX_TO_DISTANCE_H
#define PIX_TO_DISTANCE_H

#include "zf_common_typedef.h"

typedef struct
{
    uint8 valid;
    float x_cm;
    float y_cm;
} pix_to_distance_result_t;

extern pix_to_distance_result_t g_car_lamp_fused_distance;

void PixToDistance_Init(void);
uint8 PixToDistance_Update(void);

#endif /* PIX_TO_DISTANCE_H */
