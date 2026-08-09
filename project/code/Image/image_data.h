#ifndef IMAGE_DATA_H_
#define IMAGE_DATA_H_

#include "zf_common_typedef.h"

#define IMAGE_MAX_BEACON_COUNT     (4U)
#define IMAGE_MAX_CAR_LAMP_COUNT   (2U)
#define IMAGE_DATA_INVALID_VALUE   (-999.0f)
#define IMAGE_SYNC_HISTORY_DEPTH   (2U)  /* 每摄像头保留的最近帧数量。 */
#define IMAGE_SYNC_MAX_SKEW_MS     (10U) /* 三摄同步组允许的最大时差。 */

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
    uint8 car_lamp_measured_mask;
};

typedef struct
{
    uint32 frame_sequence;
    uint32 capture_time_ms;
    uint8 source_camera;
    uint8 frame_valid;
    uint8 timestamp_valid;
    uint8 reserved;
} image_frame_meta_t;

typedef struct
{
    struct image_data camera[IMAGE_CAMERA_COUNT];
    image_frame_meta_t meta[IMAGE_CAMERA_COUNT];
    uint32 max_skew_ms;
    uint8 valid;
    uint8 reserved[3];
} image_sync_set_t;

typedef struct
{
    uint32 anchor_sequence;
    uint32 max_skew_ms;
    uint8 valid;
    uint8 reserved[3];
} image_sync_diag_t;

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
    data->car_lamp_measured_mask = 0U;
}

/**
 * @brief 将帧元数据清为无效并保留指定来源身份。
 * @param meta 待清理的帧元数据指针。
 * @param source 需要写入的摄像头来源。
 * @return 无。
 */
static inline void image_frame_meta_clear(image_frame_meta_t *meta,
                                          image_camera_e source)
{
    if(meta == 0)
    {
        return;
    }

    meta->frame_sequence = 0U;
    meta->capture_time_ms = 0U;
    meta->source_camera = (uint8)source;
    meta->frame_valid = 0U;
    meta->timestamp_valid = 0U;
    meta->reserved = 0U;
}

/**
 * @brief 计算支持32位毫秒计数回绕的绝对时间差。
 * @param left 第一个毫秒时间戳。
 * @param right 第二个毫秒时间戳。
 * @return 两时间戳的最短无符号差值，单位ms。
 */
static inline uint32 image_frame_time_difference_ms(uint32 left,
                                                     uint32 right)
{
    uint32 difference = left - right;

    return (difference > 0x7FFFFFFFUL) ? (0U - difference) : difference;
}

extern struct image_data image_data[IMAGE_CAMERA_COUNT];
extern image_frame_meta_t image_frame_meta[IMAGE_CAMERA_COUNT]; /* 三摄最新来源帧元数据共享区。 */
extern volatile image_sync_diag_t g_image_sync_diag; /* 核心1发布给核心0的同步门控诊断。 */

#endif /* IMAGE_DATA_H_ */
