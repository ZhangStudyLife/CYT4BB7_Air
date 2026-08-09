#ifndef CAR_LAMP_CROSS_CHECK_H_
#define CAR_LAMP_CROSS_CHECK_H_

#include "Image/image_data.h"
#include "Image/car_lamp_projection.h"

#define CAR_LAMP_CAMERA_BIT(camera_) ((uint8)(1U << (uint8)(camera_))) /* 摄像头来源位掩码。 */
#define CAR_LAMP_CROSS_CHECK_INVALID_ERROR (-1.0f) /* 当前摄像头没有可比较观测时的误差值。 */

typedef enum
{
    CAR_LAMP_TRACK_SEARCH = 0U,
    CAR_LAMP_TRACK_ACQUIRE,
    CAR_LAMP_TRACK_TRACKED,
    CAR_LAMP_TRACK_COAST,
    CAR_LAMP_TRACK_LOST
} car_lamp_track_state_e;

typedef struct
{
    uint32 anchor_sequence;
    uint32 anchor_time_ms;
    uint32 source_switch_count;
    uint32 transition_count;
    uint32 coast_total_ms;
    uint32 full_frame_fallback_count;
    float center_x;
    float center_y;
    float velocity_x;
    float velocity_y;
    float expected_x[IMAGE_CAMERA_COUNT];
    float expected_y[IMAGE_CAMERA_COUNT];
    float match_error[IMAGE_CAMERA_COUNT];
    float roi_half_size[IMAGE_CAMERA_COUNT];
    uint8 state;
    uint8 support_camera_mask;
    uint8 roi_valid_mask;
    uint8 roi_hit_mask;
    uint8 conflict_camera_mask;
    uint8 projection_enabled;
    uint8 full_frame_fallback_mask;
    uint8 reserved;
} car_lamp_cross_check_diag_t;

extern uint32 g_car_lamp_roi_sample_count[IMAGE_CAMERA_COUNT]; /* 各摄影子ROI累计有效样本数。 */
extern uint32 g_car_lamp_roi_hit_count[IMAGE_CAMERA_COUNT]; /* 各摄全图实测落入预计ROI的累计次数。 */
extern uint32 g_car_lamp_source_handoff_count; /* 公共轨迹观测来源发生变化的累计次数。 */
extern uint32 g_car_lamp_coast_max_ms; /* 公共轨迹无实测预测保持的最长时间，单位ms。 */
extern uint32 g_car_lamp_full_frame_fallback_count; /* 实际ROI启用后的全图回退累计次数，影子模式固定为0。 */

/**
 * @brief 初始化单车灯三摄公共轨迹影子状态机。
 * @param local_camera 当前芯片本地摄像头，用作同步快照锚点。
 * @return 无。
 */
void CarLampCrossCheck_Init(image_camera_e local_camera);

/**
 * @brief 使用只读三摄同步快照更新公共轨迹和影子ROI诊断。
 * @param frames 最近两帧缓存选出的三摄快照；valid为0时仍可使用其中满足独立时序条件的帧。
 * @param roll_deg 当前横滚角，单位deg。
 * @param pitch_deg 当前俯仰角，单位deg。
 * @param height_mm 当前融合高度，单位mm。
 * @param attitude_valid 姿态有效时为1。
 * @param height_valid 高度有效时为1。
 * @return 1表示消费了新的本摄锚点帧，0表示输入无效或锚点未更新。
 */
uint8 CarLampCrossCheck_Update(const image_sync_set_t *frames,
                               float roll_deg,
                               float pitch_deg,
                               float height_mm,
                               uint8 attitude_valid,
                               uint8 height_valid);

/**
 * @brief 读取当前公共轨迹和影子ROI诊断。
 * @return 只读诊断指针，生命周期覆盖整个程序运行期。
 */
const car_lamp_cross_check_diag_t *CarLampCrossCheck_GetDiag(void);

#endif /* CAR_LAMP_CROSS_CHECK_H_ */
