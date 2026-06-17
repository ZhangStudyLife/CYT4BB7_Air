#ifndef IMAGE_DATA_H_
#define IMAGE_DATA_H_

#include "zf_common_typedef.h"

#define IMAGE_MAX_BEACON_COUNT     (4U)
#define IMAGE_MAX_CAR_LAMP_COUNT   (2U)

typedef enum
{
    Front = 0U,
    Center,
    Back,
    IMAGE_CAMERA_COUNT
} image_camera_e;

typedef struct
{
    uint8 valid;
    float x;
    float y;
    float area;
} beacon_data;

typedef struct
{
    uint8 valid;
    float cx;
    float cy;
    float width;
    float length;
    float angle;
} car_lamp_data;

struct image_data
{
    beacon_data beacon_data[IMAGE_MAX_BEACON_COUNT];
    car_lamp_data car_lamp_data[IMAGE_MAX_CAR_LAMP_COUNT];
};

#endif /* IMAGE_DATA_H_ */
