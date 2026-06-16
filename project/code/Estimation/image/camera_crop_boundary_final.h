#ifndef CAMERA_CROP_BOUNDARY_FINAL_H
#define CAMERA_CROP_BOUNDARY_FINAL_H

/*
 * Final crop boundaries in image pixel coordinates.
 * x is in [0, 187]. The area below the curve is cropped because image y grows
 * downward, so the crop test is y > boundary_y.
 */

#define CAMERA_CROP_BOUNDARY_CENTER_X      (94.0f)
#define CAMERA_CROP_BOUNDARY_BOTTOM_ROW_Y  (54.0f)
#define CAMERA_CROP_BOUNDARY_CURVE_K       (13.0f / 8742.0f)

static inline float camera_crop_boundary_y(float x)
{
    const float dx = CAMERA_CROP_BOUNDARY_CENTER_X - x;

    return CAMERA_CROP_BOUNDARY_BOTTOM_ROW_Y +
           (CAMERA_CROP_BOUNDARY_CURVE_K * dx) -
           (CAMERA_CROP_BOUNDARY_CURVE_K * dx * dx);
}

static inline float camera_front_crop_boundary_y(float x)
{
    return camera_crop_boundary_y(x);
}

static inline float camera_rear_crop_boundary_y(float x)
{
    return camera_crop_boundary_y(x);
}

static inline int camera_front_crop_below(float x, float y)
{
    return y > camera_front_crop_boundary_y(x);
}

static inline int camera_rear_crop_below(float x, float y)
{
    return y > camera_rear_crop_boundary_y(x);
}

static inline int camera_front_crop_outside(float x, float y)
{
    return camera_front_crop_below(x, y);
}

static inline int camera_rear_crop_outside(float x, float y)
{
    return camera_rear_crop_below(x, y);
}

#endif /* CAMERA_CROP_BOUNDARY_FINAL_H */
