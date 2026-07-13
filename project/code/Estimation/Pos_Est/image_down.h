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

/* 核1信标二值化运行时阈值，范围0..255，从下一帧图像开始生效。 */
extern int32 g_image_down_beacon_binary_threshold;

void image_down_init(void);
uint8 image_down_update(void);
uint8 *image_down_get_frame_buffer(void);

/* 在核1图像帧边界执行参数SET/GET，并立即读回实际值。 */
uint8 image_down_remote_param_execute(uint8 op,
                                      uint8 type,
                                      uint16 param_id,
                                      uint32 value_bits,
                                      uint32 *actual_bits);

#ifdef __cplusplus
}
#endif

#endif /* IMAGE_DOWN_H_ */
