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
extern int32 g_image_down_car_lamp_binary_threshold;
extern int32 g_image_down_car_lamp_max_area;
extern float g_image_down_match_distance;
extern float g_image_down_gate_distance;
extern float g_image_down_new_target_distance;
extern int32 g_image_down_confirm_frames;
extern int32 g_image_down_max_misses;
extern int32 g_image_down_beacon_coast_frames; /* 信标短暂丢失后的最大保留帧数。 */
extern float g_image_down_filter_pos_alpha;
extern float g_image_down_filter_vel_alpha;
extern float g_image_down_beacon_boundary_clearance;
extern float g_image_down_gray_dedup_distance;
extern int32 g_image_down_gray_edge_min_peak;
extern float g_image_down_gray_edge_max_occupancy;
extern int32 g_image_down_beacon_scan_delta; /* 信标扫描阈值相对场景均值的增量。 */
extern int32 g_image_down_beacon_scan_floor; /* 信标扫描阈值的最低灰度。 */
extern int32 g_image_down_beacon_response_min; /* 信标候选的最低灰度响应。 */
extern int32 g_image_down_beacon_normal_peak; /* 普通信标的最低峰值灰度。 */
extern int32 g_image_down_beacon_normal_min_area; /* 普通信标的最小半峰面积。 */
extern int32 g_image_down_beacon_normal_max_area; /* 普通信标的最大半峰面积。 */
extern int32 g_image_down_beacon_medium_peak; /* 中等信标的最低峰值灰度。 */
extern int32 g_image_down_beacon_medium_min_area; /* 中等信标的最小半峰面积。 */
extern int32 g_image_down_beacon_medium_max_area; /* 中等信标的最大半峰面积。 */
extern int32 g_image_down_beacon_large_peak; /* 大信标的最低峰值灰度。 */
extern int32 g_image_down_beacon_large_min_area; /* 大信标的最小半峰面积。 */
extern int32 g_image_down_beacon_large_max_area; /* 大信标的最大半峰面积。 */
extern float g_image_down_gray_weak_peak_delta;
extern int32 g_image_down_gray_weak_peak_floor;
extern int32 g_image_down_gray_weak_min_area;
extern int32 g_image_down_gray_weak_max_area;
extern float g_image_down_car_score_strong;
extern float g_image_down_car_score_weak;
extern float g_image_down_car_score_track;
extern float g_image_down_car_score_margin;

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
