#ifndef CAR_LAMP_CROSS_CHECK_H_
#define CAR_LAMP_CROSS_CHECK_H_

#include "Image/image_data.h"
#include "Image/car_lamp_projection.h"
#include "Image/car_lamp_track_types.h"

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

typedef enum
{
    CAR_LAMP_EVIDENCE_NONE = 0U,
    CAR_LAMP_EVIDENCE_LOCATE,
    CAR_LAMP_EVIDENCE_POSITIVE,
    CAR_LAMP_EVIDENCE_STRONG
} car_lamp_evidence_level_e;

/* 坐标均为以图像中心为原点的像素坐标，边界已裁剪到有效图像。 */
typedef struct
{
    uint32 snapshot_sequence;
    float expected_x;
    float expected_y;
    float expected_center_x; /* 同一采集时刻的公共坐标预计横坐标。 */
    float expected_center_y; /* 同一采集时刻的公共坐标预计纵坐标。 */
    float half_size;
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    uint8 valid;
    uint8 evidence_level;
    uint8 roi_mode;
    uint8 reserved;
} car_lamp_roi_t;

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
    uint8 roi_mode;
} car_lamp_cross_check_diag_t;

extern uint32 g_car_lamp_roi_sample_count[IMAGE_CAMERA_COUNT]; /* 各摄影子ROI累计有效样本数。 */
extern uint32 g_car_lamp_roi_hit_count[IMAGE_CAMERA_COUNT]; /* 各摄全图实测落入预计ROI的累计次数。 */
extern uint32 g_car_lamp_source_handoff_count; /* 公共轨迹观测来源发生变化的累计次数。 */
extern uint32 g_car_lamp_last_handoff_frame_sequence[IMAGE_CAMERA_COUNT];
extern uint32 g_car_lamp_coast_max_ms; /* 公共轨迹无实测预测保持的最长时间，单位ms。 */
extern uint32 g_car_lamp_full_frame_fallback_count; /* 实际ROI启用后的全图回退累计次数，影子模式固定为0。 */
extern uint32 g_car_lamp_stale_frame_reject_count;
extern uint32 g_car_lamp_duplicate_frame_reject_count;
extern uint32 g_car_lamp_invalid_time_reject_count;
extern uint32 g_car_lamp_soft_pair_hint_count; /* 前后摄误差位于16至20像素软提示区间的累计次数。 */
extern uint8 g_car_lamp_roi_mode;

/**
 * @brief 初始化单车灯三摄公共轨迹影子状态机。
 * @param local_camera 当前芯片本地摄像头，用作最新帧组更新锚点。
 * @return 无。
 */
void CarLampCrossCheck_Init(image_camera_e local_camera);

/**
 * @brief 使用只读三摄最新帧组更新公共轨迹和影子ROI诊断。
 * @param frames 三摄最新帧快照；valid为0时仍可使用其中独立未过期的帧。
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
 * @brief 按核心1当前统一时间消费三摄任意来源的新帧。
 */
uint8 CarLampCrossCheck_UpdateAt(const image_sync_set_t *frames,
                                uint32 now_ms,
                                float roll_deg,
                                float pitch_deg,
                                float height_mm,
                                uint8 attitude_valid,
                                uint8 height_valid);

/**
 * @brief 读取核心1当前权威轨迹快照。
 */
void CarLampCrossCheck_GetSnapshot(car_lamp_track_snapshot_t *out);

/**
 * @brief 将权威轨迹预测到指定采集时刻并投影到目标摄像头。
 */
uint8 CarLampCrossCheck_GetRoiAt(
    const car_lamp_track_snapshot_t *snapshot,
    image_camera_e camera,
    uint32 capture_time_ms,
    car_lamp_roi_t *out);

/**
 * @brief 按最近真实灯条半长生成指定采集时刻的ROI，通信快照布局保持不变。
 * @param snapshot 核心1权威轨迹快照。
 * @param camera 目标摄像头。
 * @param capture_time_ms 目标图像的核心1统一采集时间，单位ms。
 * @param lamp_half_length_px 最近有效真实灯条半长，单位像素。
 * @param out 输出未夹紧预计中心、公共预计中心及与图像相交的ROI。
 * @return 1表示ROI与目标图像相交，0表示输入、时间或投影无效。
 */
uint8 CarLampCrossCheck_GetRoiAtWithLampHalfLength(
    const car_lamp_track_snapshot_t *snapshot,
    image_camera_e camera,
    uint32 capture_time_ms,
    float lamp_half_length_px,
    car_lamp_roi_t *out);

/**
 * @brief 将本摄候选映射到公共坐标并与ROI的公共预计中心闭环比较。
 * @param roi 当前帧权威ROI。
 * @param camera 候选所属摄像头。
 * @param source 候选在本摄中的中心坐标，单位像素。
 * @param gate_px 公共坐标匹配门限，单位像素。
 * @return 1表示候选通过闭环门限，0表示输入、投影或距离无效。
 */
uint8 CarLampCrossCheck_CandidateMatchesRoi(
    const car_lamp_roi_t *roi,
    image_camera_e camera,
    const car_lamp_projection_point_t *source,
    float gate_px);

/**
 * @brief 设置实际ROI模式；0为影子模式，1为实际ROI模式。
 */
void CarLampCrossCheck_SetRoiMode(uint8 enabled);

/* Merge per-camera ROI runtime flags after the authority update for this cycle. */
void CarLampCrossCheck_ApplyRuntimeDiag(uint8 roi_hit_mask,
                                        uint8 fallback_mask,
                                        uint8 conflict_mask,
                                        uint8 sample_mask);

/**
 * @brief 读取当前公共轨迹和影子ROI诊断。
 * @return 只读诊断指针，生命周期覆盖整个程序运行期。
 */
const car_lamp_cross_check_diag_t *CarLampCrossCheck_GetDiag(void);

#endif /* CAR_LAMP_CROSS_CHECK_H_ */
