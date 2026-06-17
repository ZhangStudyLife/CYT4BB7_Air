#ifndef IMAGE_DOWN_H_
#define IMAGE_DOWN_H_

#include "zf_common_headfile.h"
#include "Image/image_data.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float x;
    float y;
    float radius;
    float area;
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

extern struct image_data image_data[IMAGE_CAMERA_COUNT];

void image_down_init(void);
uint8 image_down_update(void);
uint8 *image_down_get_frame_buffer(void);

#ifdef __cplusplus
}
#endif

#endif /* IMAGE_DOWN_H_ */
