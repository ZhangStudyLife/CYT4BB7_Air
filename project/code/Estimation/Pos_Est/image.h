#ifndef IMAGE_H_
#define IMAGE_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float x;
    float y;
    float radius;
    uint8 valid;
} beacon_circle_t;

typedef struct
{
    float cx;
    float cy;
    float width;
    float length;
    float angle;
    uint8 valid;
} beacon_rect_t;

#define IMAGE_MAX_BEACON_COUNT     (4U)
#define IMAGE_MAX_CAR_LAMP_COUNT   (2U)

extern beacon_circle_t g_image_beacons[IMAGE_MAX_BEACON_COUNT];
extern uint8 g_image_beacon_count;
extern beacon_rect_t g_image_car_lamps[IMAGE_MAX_CAR_LAMP_COUNT];
extern uint8 g_image_car_lamp_count;

void image_init(void);
void image_update(void);
uint8 *image_get_frame_buffer(void);

#ifdef __cplusplus
}
#endif

#endif /* IMAGE_H_ */
