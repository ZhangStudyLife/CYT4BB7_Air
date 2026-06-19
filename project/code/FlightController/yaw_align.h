#ifndef YAW_ALIGN_H
#define YAW_ALIGN_H

#include "zf_common_headfile.h"

typedef struct
{
    uint8 valid;
    uint8 camera;
    float x;
    float y;
    float area;
} yaw_align_debug_beacon_t;

typedef struct
{
    uint8 locked;
    uint8 candidate_frames;
    uint8 lost_frames;
    yaw_align_debug_beacon_t locked_beacon;
    yaw_align_debug_beacon_t candidate_beacon;
} yaw_align_debug_t;

void YawAlign_Reset(void);
uint8 YawAlign_Update(void);
void YawAlign_GetDebug(yaw_align_debug_t *out);

#endif /* YAW_ALIGN_H */
