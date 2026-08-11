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
extern int32 g_image_down_beacon_min_area;
extern int32 g_image_down_side_edge_min_area;
extern int32 g_image_down_side_edge_threshold;
extern int32 g_image_down_car_lamp_binary_threshold;
extern int32 g_image_down_car_lamp_min_area;
extern int32 g_image_down_car_lamp_max_area;
extern float g_image_down_car_lamp_min_elongation;
extern float g_image_down_car_lamp_min_length;
extern int32 g_image_down_near_lamp_pad;
extern int32 g_image_down_near_lamp_min_area;
extern int32 g_image_down_near_lamp_isolated_min_area;
extern int32 g_image_down_near_lamp_background_max;
extern float g_image_down_match_distance;
extern float g_image_down_gate_distance;
extern float g_image_down_new_target_distance;
extern int32 g_image_down_confirm_frames;
extern int32 g_image_down_max_misses;
extern float g_image_down_filter_pos_alpha;
extern float g_image_down_filter_vel_alpha;

/*
 * 函数功能: 初始化下摄图像算法、摄像头接口及5000帧性能统计窗口。
 * 输入参数: 无。
 * 返回值: 无。
 */
void image_down_init(void);

/*
 * 函数功能: 仅在摄像头发布真实新帧时锁存图像并执行下摄算法。
 * 输入参数: 无。
 * 返回值: 1表示本次完成了一帧处理；0表示没有可处理的新帧。
 */
uint8 image_down_update(void);
uint8 *image_down_get_frame_buffer(void);
const uint8 *image_down_get_binary_buffer(void);
const uint8 *image_down_get_car_lamp_binary_buffer(void);

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
