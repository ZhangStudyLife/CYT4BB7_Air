#ifndef CAMERA_CROP_BOUNDARY_MAPPING_H
#define CAMERA_CROP_BOUNDARY_MAPPING_H

#include "camera_crop_boundary_final.h"

typedef struct
{
    float source_x;
    float source_y;
    float center_x;
    float center_y;
} camera_crop_boundary_map_node_t;

#define CAMERA_FRONT_BOUNDARY_MAP_NODE_COUNT (4U)
#define CAMERA_REAR_BOUNDARY_MAP_NODE_COUNT  (6U)
#define CAMERA_FRONT_BOUNDARY_CENTER_Y       (3.0f)
#define CAMERA_REAR_BOUNDARY_CENTER_Y        (102.0f)

/*
 * Trusted calibration nodes. source_y records the measured same-point sample.
 * Boundary y is calculated by camera_*_crop_boundary_y().
 */
static const camera_crop_boundary_map_node_t camera_front_boundary_map_nodes[CAMERA_FRONT_BOUNDARY_MAP_NODE_COUNT] = {
    {28.0f, 37.0f, 23.0f, 2.0f},
    {94.0f, 51.0f, 94.0f, 2.0f},
    {147.0f, 41.0f, 152.0f, 5.0f},
    {158.0f, 36.0f, 164.0f, 4.0f},
};

/* Rear source points are converted from the user's (188 - x, 120 - y) form. */
static const camera_crop_boundary_map_node_t camera_rear_boundary_map_nodes[CAMERA_REAR_BOUNDARY_MAP_NODE_COUNT] = {
    {21.0f, 34.0f, 172.0f, 99.0f},
    {30.0f, 35.0f, 164.0f, 98.0f},
    {31.0f, 40.0f, 168.0f, 76.0f},
    {37.0f, 39.0f, 157.0f, 98.0f},
    {100.0f, 41.0f, 85.0f, 107.0f},
    {166.0f, 47.0f, 17.0f, 88.0f},
};

static inline const camera_crop_boundary_map_node_t *camera_front_boundary_map_node(unsigned int index)
{
    if(index >= CAMERA_FRONT_BOUNDARY_MAP_NODE_COUNT)
    {
        return 0;
    }

    return &camera_front_boundary_map_nodes[index];
}

static inline const camera_crop_boundary_map_node_t *camera_rear_boundary_map_node(unsigned int index)
{
    if(index >= CAMERA_REAR_BOUNDARY_MAP_NODE_COUNT)
    {
        return 0;
    }

    return &camera_rear_boundary_map_nodes[index];
}

static inline float camera_front_boundary_center_x(float front_x)
{
    return front_x;
}

static inline float camera_front_boundary_center_y(float front_x)
{
    (void)front_x;
    return CAMERA_FRONT_BOUNDARY_CENTER_Y;
}

static inline float camera_rear_mapping_boundary_y(float rear_x)
{
    return camera_rear_crop_boundary_y(rear_x);
}

static inline void camera_rear_point_center_xy(float rear_x, float rear_y, float *center_x, float *center_y)
{
    (void)rear_y;

    if(center_x != 0)
    {
        *center_x = 187.0f - rear_x;
    }

    if(center_y != 0)
    {
        *center_y = CAMERA_REAR_BOUNDARY_CENTER_Y;
    }
}

static inline float camera_rear_boundary_center_x(float rear_x)
{
    float center_x_value;

    camera_rear_point_center_xy(rear_x, camera_rear_mapping_boundary_y(rear_x), &center_x_value, 0);
    return center_x_value;
}

static inline float camera_rear_boundary_center_y(float rear_x)
{
    float center_y_value;

    camera_rear_point_center_xy(rear_x, camera_rear_mapping_boundary_y(rear_x), 0, &center_y_value);
    return center_y_value;
}

static inline void camera_front_boundary_center_xy(float front_x, float *center_x, float *center_y)
{
    if(center_x != 0)
    {
        *center_x = camera_front_boundary_center_x(front_x);
    }

    if(center_y != 0)
    {
        *center_y = camera_front_boundary_center_y(front_x);
    }
}

static inline void camera_rear_boundary_center_xy(float rear_x, float *center_x, float *center_y)
{
    camera_rear_point_center_xy(rear_x, camera_rear_mapping_boundary_y(rear_x), center_x, center_y);
}

static inline void camera_front_boundary_point_to_center(float front_x,
                                                         float *front_y,
                                                         float *center_x,
                                                         float *center_y)
{
    if(front_y != 0)
    {
        *front_y = camera_front_crop_boundary_y(front_x);
    }

    camera_front_boundary_center_xy(front_x, center_x, center_y);
}

static inline void camera_rear_boundary_point_to_center(float rear_x,
                                                        float *rear_y,
                                                        float *center_x,
                                                        float *center_y)
{
    float rear_boundary_y;

    rear_boundary_y = camera_rear_mapping_boundary_y(rear_x);

    if(rear_y != 0)
    {
        *rear_y = rear_boundary_y;
    }

    camera_rear_point_center_xy(rear_x, rear_boundary_y, center_x, center_y);
}

#endif /* CAMERA_CROP_BOUNDARY_MAPPING_H */
