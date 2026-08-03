#ifndef IMAGE_DOWN_HORIZON_H_
#define IMAGE_DOWN_HORIZON_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMAGE_DOWN_HORIZON_WIDTH        (188U)
#define IMAGE_DOWN_HORIZON_HEIGHT       (120U)
#define IMAGE_DOWN_HORIZON_POINT_COUNT  (360U)

extern float g_image_down_horizon_x[IMAGE_DOWN_HORIZON_POINT_COUNT];
extern float g_image_down_horizon_y[IMAGE_DOWN_HORIZON_POINT_COUNT];
extern float g_image_down_horizon_top_y[IMAGE_DOWN_HORIZON_WIDTH];
extern float g_image_down_horizon_bottom_y[IMAGE_DOWN_HORIZON_WIDTH];
extern uint8 g_image_down_horizon_column_valid[IMAGE_DOWN_HORIZON_WIDTH];
extern uint8 g_image_down_horizon_valid;
extern uint8 g_image_down_horizon_extrapolated;

void image_down_horizon_init(void);
void image_down_horizon_invalidate(void);
void image_down_horizon_update(float roll_deg,
                               float pitch_deg,
                               float height_mm,
                               uint8 attitude_valid,
                               uint8 height_valid);
uint8 image_down_horizon_get_point(uint16 index, float *x, float *y);
uint8 image_down_horizon_get_column(uint16 x, float *top_y, float *bottom_y);
uint8 image_down_horizon_contains(float x, float y, float margin);

#ifdef __cplusplus
}
#endif

#endif
