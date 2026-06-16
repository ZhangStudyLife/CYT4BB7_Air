#ifndef MODE2_CENTER_IMAGE_H_
#define MODE2_CENTER_IMAGE_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float x;
    float y;
    float radius;
    float area;
    unsigned char valid;
} mode2_center_image_beacon_t;

typedef struct
{
    float cx;
    float cy;
    float width;
    float length;
    float angle;
    unsigned char valid;
} mode2_center_image_car_lamp_t;

#define MODE2_CENTER_IMAGE_MAX_BEACON_COUNT     (4U)
#define MODE2_CENTER_IMAGE_MAX_CAR_LAMP_COUNT   (2U)

extern mode2_center_image_beacon_t g_mode2_center_image_beacons[MODE2_CENTER_IMAGE_MAX_BEACON_COUNT];
extern uint8 g_mode2_center_image_beacon_count;
extern mode2_center_image_car_lamp_t g_mode2_center_image_car_lamps[MODE2_CENTER_IMAGE_MAX_CAR_LAMP_COUNT];
extern uint8 g_mode2_center_image_car_lamp_count;

void mode2_center_image_init(void);
void mode2_center_image_update(void);
uint8 *mode2_center_image_get_frame_buffer(void);

#ifdef __cplusplus
}
#endif

#endif
