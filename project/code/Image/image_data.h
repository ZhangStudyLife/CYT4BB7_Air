/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#ifndef IMAGE_DATA_H_
#define IMAGE_DATA_H_

#include "zf_common_typedef.h"

#define IMAGE_MAX_BEACON_COUNT     (4U)
#define IMAGE_MAX_CAR_LAMP_COUNT   (2U)
#define IMAGE_DATA_INVALID_VALUE   (-999.0f)

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

static inline void image_data_clear_beacon(beacon_data *beacon)
{
    if(beacon == 0)
    {
        return;
    }

    beacon->valid = 0U;
    beacon->x = IMAGE_DATA_INVALID_VALUE;
    beacon->y = IMAGE_DATA_INVALID_VALUE;
    beacon->area = 0.0f;
}

static inline void image_data_clear_car_lamp(car_lamp_data *lamp)
{
    if(lamp == 0)
    {
        return;
    }

    lamp->valid = 0U;
    lamp->cx = IMAGE_DATA_INVALID_VALUE;
    lamp->cy = IMAGE_DATA_INVALID_VALUE;
    lamp->width = 0.0f;
    lamp->length = 0.0f;
    lamp->angle = IMAGE_DATA_INVALID_VALUE;
}

static inline uint8 image_data_beacon_valid(const beacon_data *beacon)
{
    return ((beacon != 0) &&
            (beacon->valid != 0U) &&
            (beacon->x != IMAGE_DATA_INVALID_VALUE) &&
            (beacon->y != IMAGE_DATA_INVALID_VALUE) &&
            (beacon->area > 0.0f)) ? 1U : 0U;
}

static inline uint8 image_data_car_lamp_valid(const car_lamp_data *lamp)
{
    return ((lamp != 0) &&
            (lamp->valid != 0U) &&
            (lamp->cx != IMAGE_DATA_INVALID_VALUE) &&
            (lamp->cy != IMAGE_DATA_INVALID_VALUE) &&
            (lamp->angle != IMAGE_DATA_INVALID_VALUE)) ? 1U : 0U;
}

static inline void image_data_clear(struct image_data *data)
{
    uint8 i;

    if(data == 0)
    {
        return;
    }

    for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
    {
        image_data_clear_beacon(&data->beacon_data[i]);
    }
    for(i = 0U; i < IMAGE_MAX_CAR_LAMP_COUNT; i++)
    {
        image_data_clear_car_lamp(&data->car_lamp_data[i]);
    }
}

extern struct image_data image_data[IMAGE_CAMERA_COUNT];

#endif /* IMAGE_DATA_H_ */
