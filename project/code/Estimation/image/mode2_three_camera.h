#ifndef MODE2_THREE_CAMERA_H
#define MODE2_THREE_CAMERA_H

#include "zf_common_headfile.h"
#define MODE2_THREE_CAMERA_CAMERA_COUNT   (3U)
#define MODE2_THREE_CAMERA_CAMERA_TARGETS (4U)

#define MODE2_THREE_CAMERA_CAMERA_FRONT   (0U)
#define MODE2_THREE_CAMERA_CAMERA_CENTER  (1U)
#define MODE2_THREE_CAMERA_CAMERA_REAR    (2U)

typedef struct
{
    uint8 valid;
    float x;
    float y;
    float area;
} mode2_three_camera_target_t;

typedef struct
{
    mode2_three_camera_target_t target[MODE2_THREE_CAMERA_CAMERA_TARGETS];
} mode2_three_camera_frame_t;

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
} mode2_three_camera_state_t;

extern mode2_three_camera_state_t g_mode2_three_camera;

void mode2_three_camera_init(void);
void mode2_three_camera_set_auto_max_count(uint8 max_count);
void mode2_three_camera_set_center_car_lamp(uint8 valid, float cx, float cy, float angle_deg);
uint8 mode2_three_camera_get_center_car_lamp_ref(float *ref_x, float *ref_y);
uint8 mode2_three_camera_update_100HZ(const mode2_three_camera_frame_t camera[MODE2_THREE_CAMERA_CAMERA_COUNT]);

#endif /* MODE2_THREE_CAMERA_H */
