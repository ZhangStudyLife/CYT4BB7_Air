#ifndef IMAGE_FUSION_H
#define IMAGE_FUSION_H

#include "zf_common_headfile.h"
#include "Image/image_data.h"

typedef struct
{
    uint8 active;
    uint8 valid;
    uint8 camera_id;
    uint16 frame_id;
    float image_x;
    float image_y;
    uint8 center_delta_valid;
    float center_delta_x;
    float center_delta_y;
    float raw_center_delta_x;
    float raw_center_delta_y;
    uint8 lamp_angle_valid;
    float lamp_angle_deg;
    float car_lamp_cx;
    float car_lamp_cy;
    float transform_angle_deg;
    float area;
    uint8 missing_frame_count;
} image_fusion_state_t;

extern image_fusion_state_t g_image_fusion;

void image_fusion_init(void);
uint8 image_fusion_update_100HZ(const struct image_data camera[IMAGE_CAMERA_COUNT]);

#endif /* IMAGE_FUSION_H */
