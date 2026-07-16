#ifndef CAR_LAMP_FUSED_H
#define CAR_LAMP_FUSED_H

#include "zf_common_typedef.h"

typedef struct
{
    uint8 valid;
    float cx;
    float cy;
    float angle;
    float width;
    float length;
} car_lamp_fused_result_t;

extern car_lamp_fused_result_t g_car_lamp_fused;
extern uint8 only_front_see_car_lamp;
extern uint8 only_back_see_car_lamp;

void CarLampFused_Init(void);
uint8 CarLampFused_Update50Hz(void);

#endif /* CAR_LAMP_FUSED_H */
