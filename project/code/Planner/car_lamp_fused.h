#ifndef CAR_LAMP_FUSED_H
#define CAR_LAMP_FUSED_H

#include "zf_common_typedef.h"

typedef struct
{
    uint8 valid;
    float cx;
    float cy;
} car_lamp_fused_result_t;

extern car_lamp_fused_result_t g_car_lamp_fused;

void CarLampFused_Init(void);
uint8 CarLampFused_Update50Hz(void);

#endif /* CAR_LAMP_FUSED_H */
