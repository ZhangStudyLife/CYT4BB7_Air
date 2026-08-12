#ifndef PROJECTION_CENTER_H
#define PROJECTION_CENTER_H

#include "zf_common_typedef.h"

typedef struct
{
    uint8 valid;
    float cx;
    float cy;
} projection_center_result_t;

extern projection_center_result_t g_projection_center;

void ProjectionCenter_Init(void);
uint8 ProjectionCenter_Update100Hz(void);

#endif /* PROJECTION_CENTER_H */
