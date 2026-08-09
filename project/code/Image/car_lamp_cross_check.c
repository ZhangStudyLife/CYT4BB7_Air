#include "car_lamp_cross_check.h"

#include <math.h>
#include <string.h>

#define CAR_LAMP_ACQUIRE_SINGLE_FRAMES (3U) /* 单摄建立公共轨迹所需连续实测帧数。 */
#define CAR_LAMP_ACQUIRE_PAIR_FRAMES   (2U) /* 双摄一致建立公共轨迹所需连续同步帧数。 */
#define CAR_LAMP_ACQUIRE_MAX_GAP_MS    (500U) /* 相邻来源实测帧允许的最大到达间隔。 */
#define CAR_LAMP_ACQUIRE_TOTAL_WINDOW_MS (1000U) /* 单次连续获取允许的最大总时间。 */
#define CAR_LAMP_TRACK_COAST_MAX_MS    (50U) /* 无原始实测时允许速度预测的最长时间。 */
#define CAR_LAMP_TRACK_FRONT_GATE_PX   (8.0f) /* 前摄投影到公共坐标后的匹配门限。 */
#define CAR_LAMP_TRACK_CENTER_GATE_PX  (8.0f) /* 下摄公共坐标内的匹配门限。 */
#define CAR_LAMP_TRACK_BACK_GATE_PX    (12.0f) /* 后摄投影到公共坐标后的匹配门限。 */
#define CAR_LAMP_TRACK_FB_GATE_PX      (16.0f) /* 前后摄间接比较的一致性门限。 */
#define CAR_LAMP_TRACK_POSITION_GAIN   (0.6f) /* 测量残差更新到公共位置的比例。 */
#define CAR_LAMP_TRACK_OLD_VELOCITY    (0.7f) /* 速度低通中旧速度权重。 */
#define CAR_LAMP_TRACK_NEW_VELOCITY    (0.3f) /* 速度低通中本帧测量速度权重。 */
#define CAR_LAMP_TRACK_MIN_HEIGHT_MM   (800.0f) /* 投影启用的最低融合高度。 */
#define CAR_LAMP_TRACK_MAX_HEIGHT_MM   (1300.0f) /* 投影启用的最高融合高度。 */
#define CAR_LAMP_TRACK_MAX_TILT_DEG    (30.0f) /* 大姿态日志验证20至30度投影误差仍位于匹配门限内。 */
#define CAR_LAMP_TRACK_ROI_FRONT_PX    (20.0f) /* 前摄影子ROI基础半尺寸。 */
#define CAR_LAMP_TRACK_ROI_CENTER_PX   (20.0f) /* 下摄影子ROI基础半尺寸。 */
#define CAR_LAMP_TRACK_ROI_BACK_PX     (24.0f) /* 后摄影子ROI基础半尺寸。 */
#define CAR_LAMP_TRACK_ROI_MARGIN_PX   (2.0f) /* 影子ROI额外边界余量。 */

typedef struct
{
    uint8 valid;
    uint8 fresh;
    image_camera_e camera;
    uint32 sequence;
    uint32 time_ms;
    car_lamp_projection_point_t source;
    car_lamp_projection_point_t center;
    car_lamp_projection_point_t aligned_center;
    float length;
    float error;
} car_lamp_observation_t;

typedef struct
{
    image_camera_e local_camera;
    uint8 initialized;
    uint8 active;
    uint8 support_camera_mask;
    uint8 acquire_count[IMAGE_CAMERA_COUNT];
    uint8 acquire_pair_mask;
    uint8 acquire_pair_count;
    uint32 last_anchor_sequence;
    uint32 last_frame_sequence[IMAGE_CAMERA_COUNT];
    uint32 accepted_frame_sequence[IMAGE_CAMERA_COUNT];
    uint32 acquire_first_time_ms[IMAGE_CAMERA_COUNT];
    uint32 acquire_time_ms[IMAGE_CAMERA_COUNT];
    uint32 acquire_pair_sequence[IMAGE_CAMERA_COUNT];
    uint32 acquire_pair_first_time_ms;
    uint32 acquire_pair_time_ms;
    uint32 last_update_time_ms;
    uint32 last_support_time_ms;
    float last_lamp_length[IMAGE_CAMERA_COUNT];
    car_lamp_projection_point_t acquire_point[IMAGE_CAMERA_COUNT];
    car_lamp_projection_point_t position;
    car_lamp_projection_point_t velocity;
    car_lamp_cross_check_diag_t diag;
} car_lamp_cross_check_tracker_t;

/* 单车灯公共轨迹的芯片本地状态。 */
static car_lamp_cross_check_tracker_t s_tracker;

/* 各摄影子ROI累计有效样本数。 */
uint32 g_car_lamp_roi_sample_count[IMAGE_CAMERA_COUNT];
/* 各摄全图实测落入预计ROI的累计次数。 */
uint32 g_car_lamp_roi_hit_count[IMAGE_CAMERA_COUNT];
/* 公共轨迹观测来源发生变化的累计次数。 */
uint32 g_car_lamp_source_handoff_count;
/* 公共轨迹无实测预测保持的最长时间，单位ms。 */
uint32 g_car_lamp_coast_max_ms;
/* 实际ROI启用后的全图回退累计次数，影子模式固定为0。 */
uint32 g_car_lamp_full_frame_fallback_count;

/**
 * @brief 计算二维欧氏距离。
 * @param left 第一个公共坐标点。
 * @param right 第二个公共坐标点。
 * @return 两点距离，单位像素。
 */
static float car_lamp_cross_check_distance(
    const car_lamp_projection_point_t *left,
    const car_lamp_projection_point_t *right)
{
    float dx = left->x - right->x;
    float dy = left->y - right->y;

    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 返回摄像头观测相对公共轨迹的独立匹配门限。
 * @param camera 摄像头编号。
 * @return 匹配门限，单位像素。
 */
static float car_lamp_cross_check_camera_gate(image_camera_e camera)
{
    if(camera == Back)
    {
        return CAR_LAMP_TRACK_BACK_GATE_PX;
    }
    if(camera == Front)
    {
        return CAR_LAMP_TRACK_FRONT_GATE_PX;
    }
    return CAR_LAMP_TRACK_CENTER_GATE_PX;
}

/**
 * @brief 返回两摄投影结果之间的一致性门限。
 * @param left 第一摄像头编号。
 * @param right 第二摄像头编号。
 * @return 一致性门限，单位像素。
 */
static float car_lamp_cross_check_pair_gate(image_camera_e left,
                                             image_camera_e right)
{
    if(((left == Front) && (right == Back)) ||
       ((left == Back) && (right == Front)))
    {
        return CAR_LAMP_TRACK_FB_GATE_PX;
    }
    if((left == Back) || (right == Back))
    {
        return CAR_LAMP_TRACK_BACK_GATE_PX;
    }
    return CAR_LAMP_TRACK_FRONT_GATE_PX;
}

/**
 * @brief 将公共轨迹按常速度预测到指定统一时间。
 * @param time_ms 目标核心1统一时间，单位ms。
 * @param point 输出预测公共坐标。
 * @return 无。
 */
static void car_lamp_cross_check_predict(
    uint32 time_ms,
    car_lamp_projection_point_t *point)
{
    int32 delta_ms = (int32)(time_ms - s_tracker.last_update_time_ms);
    float delta_s;

    if(delta_ms < 0)
    {
        delta_ms = 0;
    }
    if(delta_ms > (int32)CAR_LAMP_TRACK_COAST_MAX_MS)
    {
        delta_ms = (int32)CAR_LAMP_TRACK_COAST_MAX_MS;
    }
    delta_s = (float)delta_ms * 0.001f;
    point->x = s_tracker.position.x + s_tracker.velocity.x * delta_s;
    point->y = s_tracker.position.y + s_tracker.velocity.y * delta_s;
}

/**
 * @brief 切换轨迹状态并累计状态转换次数。
 * @param state 新状态。
 * @return 无。
 */
static void car_lamp_cross_check_set_state(car_lamp_track_state_e state)
{
    if(s_tracker.diag.state != (uint8)state)
    {
        s_tracker.diag.transition_count++;
        s_tracker.diag.state = (uint8)state;
    }
}

/**
 * @brief 判断姿态、高度和时间是否允许使用跨摄投影。
 * @param anchor 锚点帧元数据。
 * @param roll_deg 当前横滚角，单位deg。
 * @param pitch_deg 当前俯仰角，单位deg。
 * @param height_mm 当前融合高度，单位mm。
 * @param attitude_valid 姿态有效标志。
 * @param height_valid 高度有效标志。
 * @return 1表示允许投影，0表示必须保持原全图识别。
 */
static uint8 car_lamp_cross_check_projection_enabled(
    const image_frame_meta_t *anchor,
    float roll_deg,
    float pitch_deg,
    float height_mm,
    uint8 attitude_valid,
    uint8 height_valid)
{
    if((anchor == 0) || (anchor->frame_valid == 0U) ||
       (attitude_valid == 0U) || (height_valid == 0U))
    {
        return 0U;
    }
    if((height_mm < CAR_LAMP_TRACK_MIN_HEIGHT_MM) ||
       (height_mm > CAR_LAMP_TRACK_MAX_HEIGHT_MM) ||
       (fabsf(roll_deg) > CAR_LAMP_TRACK_MAX_TILT_DEG) ||
       (fabsf(pitch_deg) > CAR_LAMP_TRACK_MAX_TILT_DEG))
    {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 将公共轨迹投影为ROI中心，并保留仍与图像相交的边缘ROI。
 * @param camera 目标摄像头。
 * @param center 公共坐标中的预测中心。
 * @param half_size ROI半尺寸，单位像素。
 * @param source 输出摄像头坐标；边缘相交时夹紧到图像边界。
 * @return 1表示ROI与目标摄像头图像相交，0表示完全位于视野外。
 */
static uint8 car_lamp_cross_check_project_roi(
    image_camera_e camera,
    const car_lamp_projection_point_t *center,
    float half_size,
    car_lamp_projection_point_t *source)
{
    if((source == 0) || (half_size <= 0.0f))
    {
        return 0U;
    }

    source->x = IMAGE_DATA_INVALID_VALUE;
    source->y = IMAGE_DATA_INVALID_VALUE;
    if(CarLampProjection_FromCenter(camera, center, source) != 0U)
    {
        return 1U;
    }
    if((source->x < (-CAR_LAMP_IMAGE_HALF_WIDTH - half_size)) ||
       (source->x > (CAR_LAMP_IMAGE_HALF_WIDTH + half_size)) ||
       (source->y < (-CAR_LAMP_IMAGE_HALF_HEIGHT - half_size)) ||
       (source->y > (CAR_LAMP_IMAGE_HALF_HEIGHT + half_size)))
    {
        return 0U;
    }

    if(source->x < -CAR_LAMP_IMAGE_HALF_WIDTH)
    {
        source->x = -CAR_LAMP_IMAGE_HALF_WIDTH;
    }
    else if(source->x > CAR_LAMP_IMAGE_HALF_WIDTH)
    {
        source->x = CAR_LAMP_IMAGE_HALF_WIDTH;
    }
    if(source->y < -CAR_LAMP_IMAGE_HALF_HEIGHT)
    {
        source->y = -CAR_LAMP_IMAGE_HALF_HEIGHT;
    }
    else if(source->y > CAR_LAMP_IMAGE_HALF_HEIGHT)
    {
        source->y = CAR_LAMP_IMAGE_HALF_HEIGHT;
    }
    return 1U;
}

/**
 * @brief 从各摄最新帧快照提取本轮可用于公共轨迹的原始实测。
 * @param frames 三摄只读快照。
 * @param observations 输出三摄观测数组。
 * @return 当前存在原始实测的摄像头位掩码。
 */
static uint8 car_lamp_cross_check_build_observations(
    const image_sync_set_t *frames,
    car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint32 anchor_time_ms)
{
    uint8 camera;
    uint8 measured_mask = 0U;

    memset(observations, 0,
           sizeof(car_lamp_observation_t) * IMAGE_CAMERA_COUNT);
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        const image_frame_meta_t *meta = &frames->meta[camera];
        const struct image_data *data = &frames->camera[camera];
        car_lamp_observation_t *observation = &observations[camera];
        const car_lamp_data *lamp = &data->car_lamp_data[0];

        observation->camera = (image_camera_e)camera;
        observation->error = CAR_LAMP_CROSS_CHECK_INVALID_ERROR;
        if((meta->frame_valid == 0U) || (meta->frame_sequence == 0U) ||
           (meta->source_camera != camera))
        {
            continue;
        }

        observation->fresh =
            (meta->frame_sequence != s_tracker.last_frame_sequence[camera]) ?
            1U : 0U;
        if(observation->fresh != 0U)
        {
            s_tracker.last_frame_sequence[camera] = meta->frame_sequence;
        }
        if(((data->car_lamp_measured_mask & 0x01U) == 0U) ||
           (image_data_car_lamp_valid(lamp) == 0U))
        {
            continue;
        }

        observation->source.x = lamp->cx;
        observation->source.y = lamp->cy;
        if(CarLampProjection_ToCenter((image_camera_e)camera,
                                      &observation->source,
                                      &observation->center) == 0U)
        {
            continue;
        }
        observation->valid = 1U;
        observation->sequence = meta->frame_sequence;
        observation->time_ms =
            (meta->timestamp_valid != 0U) ?
            meta->capture_time_ms : anchor_time_ms;
        observation->length = lamp->length;
        if(observation->fresh != 0U)
        {
            measured_mask |= CAR_LAMP_CAMERA_BIT(camera);
        }
    }
    return measured_mask;
}

/**
 * @brief 在给定候选掩码中选择距离最近且一致的两摄组合。
 * @param observations 三摄观测数组。
 * @param candidate_mask 允许参与的摄像头位掩码。
 * @return 最佳一致双摄位掩码；没有一致组合时返回0。
 */
static uint8 car_lamp_cross_check_best_pair(
    const car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint8 candidate_mask)
{
    uint8 left;
    uint8 right;
    uint8 best_mask = 0U;
    float best_distance = 1000000.0f;

    for(left = 0U; left < IMAGE_CAMERA_COUNT; left++)
    {
        for(right = (uint8)(left + 1U);
            right < IMAGE_CAMERA_COUNT;
            right++)
        {
            float distance;
            uint8 pair_mask = (uint8)(CAR_LAMP_CAMERA_BIT(left) |
                                      CAR_LAMP_CAMERA_BIT(right));

            if((candidate_mask & pair_mask) != pair_mask)
            {
                continue;
            }
            distance = car_lamp_cross_check_distance(
                &observations[left].aligned_center,
                &observations[right].aligned_center);
            if((distance <= car_lamp_cross_check_pair_gate(
                    (image_camera_e)left, (image_camera_e)right)) &&
               (distance < best_distance))
            {
                best_distance = distance;
                best_mask = pair_mask;
            }
        }
    }
    return best_mask;
}

/**
 * @brief 用单摄三帧或双摄两帧规则推进未建立轨迹的获取状态。
 * @param observations 三摄观测数组。
 * @param anchor_time_ms 本摄锚点时间，单位ms。
 * @return 1表示本轮建立公共轨迹，0表示仍在搜索或获取。
 */
static uint8 car_lamp_cross_check_acquire(
    car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint32 anchor_time_ms)
{
    uint8 camera;
    uint8 fresh_mask = 0U;
    uint8 valid_mask = 0U;
    uint8 pending_mask = 0U;
    uint8 pair_mask;
    uint8 pair_changed = 0U;
    uint8 support_mask = 0U;
    car_lamp_projection_point_t initial = {0.0f, 0.0f};

    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        car_lamp_observation_t *observation = &observations[camera];
        uint8 camera_bit = CAR_LAMP_CAMERA_BIT(camera);

        observation->aligned_center = observation->center;
        if(observation->valid != 0U)
        {
            valid_mask |= camera_bit;
        }
        if(observation->fresh == 0U)
        {
            continue;
        }
        fresh_mask |= camera_bit;
        if(observation->valid == 0U)
        {
            s_tracker.acquire_count[camera] = 0U;
            s_tracker.acquire_first_time_ms[camera] = 0U;
            continue;
        }

        if((s_tracker.acquire_count[camera] != 0U) &&
           ((uint32)(anchor_time_ms -
                     s_tracker.acquire_time_ms[camera]) <=
            CAR_LAMP_ACQUIRE_MAX_GAP_MS) &&
           ((uint32)(anchor_time_ms -
                     s_tracker.acquire_first_time_ms[camera]) <=
            CAR_LAMP_ACQUIRE_TOTAL_WINDOW_MS) &&
           (car_lamp_cross_check_distance(
                &s_tracker.acquire_point[camera],
                &observation->center) <=
            car_lamp_cross_check_camera_gate((image_camera_e)camera)))
        {
            if(s_tracker.acquire_count[camera] < 0xFFU)
            {
                s_tracker.acquire_count[camera]++;
            }
        }
        else
        {
            s_tracker.acquire_count[camera] = 1U;
            s_tracker.acquire_first_time_ms[camera] = anchor_time_ms;
        }
        s_tracker.acquire_point[camera] = observation->center;
        s_tracker.acquire_time_ms[camera] = anchor_time_ms;
    }

    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        if((s_tracker.acquire_count[camera] != 0U) &&
           ((uint32)(anchor_time_ms -
                     s_tracker.acquire_time_ms[camera]) >
            CAR_LAMP_ACQUIRE_MAX_GAP_MS))
        {
            s_tracker.acquire_count[camera] = 0U;
            s_tracker.acquire_first_time_ms[camera] = 0U;
        }
    }

    pair_mask = car_lamp_cross_check_best_pair(observations, valid_mask);
    if((pair_mask != 0U) && ((fresh_mask & pair_mask) != 0U))
    {
        for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
        {
            if(((pair_mask & CAR_LAMP_CAMERA_BIT(camera)) != 0U) &&
               (observations[camera].sequence !=
                s_tracker.acquire_pair_sequence[camera]))
            {
                pair_changed = 1U;
            }
        }
        if(pair_changed != 0U)
        {
            if((pair_mask == s_tracker.acquire_pair_mask) &&
               (s_tracker.acquire_pair_count != 0U) &&
               ((uint32)(anchor_time_ms -
                         s_tracker.acquire_pair_time_ms) <=
                CAR_LAMP_ACQUIRE_MAX_GAP_MS) &&
               ((uint32)(anchor_time_ms -
                         s_tracker.acquire_pair_first_time_ms) <=
                CAR_LAMP_ACQUIRE_TOTAL_WINDOW_MS))
            {
                if(s_tracker.acquire_pair_count < 0xFFU)
                {
                    s_tracker.acquire_pair_count++;
                }
            }
            else
            {
                s_tracker.acquire_pair_mask = pair_mask;
                s_tracker.acquire_pair_count = 1U;
                s_tracker.acquire_pair_first_time_ms = anchor_time_ms;
            }
            memset(s_tracker.acquire_pair_sequence, 0,
                   sizeof(s_tracker.acquire_pair_sequence));
            for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
            {
                if((pair_mask & CAR_LAMP_CAMERA_BIT(camera)) != 0U)
                {
                    s_tracker.acquire_pair_sequence[camera] =
                        observations[camera].sequence;
                }
            }
            s_tracker.acquire_pair_time_ms = anchor_time_ms;
        }
    }
    else if((s_tracker.acquire_pair_count != 0U) &&
            (((fresh_mask & s_tracker.acquire_pair_mask) != 0U) ||
             ((uint32)(anchor_time_ms -
                       s_tracker.acquire_pair_time_ms) >
              CAR_LAMP_ACQUIRE_MAX_GAP_MS)))
    {
        s_tracker.acquire_pair_mask = 0U;
        s_tracker.acquire_pair_count = 0U;
        s_tracker.acquire_pair_first_time_ms = 0U;
        memset(s_tracker.acquire_pair_sequence, 0,
               sizeof(s_tracker.acquire_pair_sequence));
    }

    if((s_tracker.acquire_pair_count >= CAR_LAMP_ACQUIRE_PAIR_FRAMES) &&
       (pair_mask == s_tracker.acquire_pair_mask))
    {
        for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
        {
            if((pair_mask & CAR_LAMP_CAMERA_BIT(camera)) != 0U)
            {
                initial.x += observations[camera].center.x;
                initial.y += observations[camera].center.y;
                support_mask |= CAR_LAMP_CAMERA_BIT(camera);
            }
        }
        initial.x *= 0.5f;
        initial.y *= 0.5f;
    }
    else
    {
        for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
        {
            if((s_tracker.acquire_count[camera] >=
                CAR_LAMP_ACQUIRE_SINGLE_FRAMES) &&
               (observations[camera].valid != 0U))
            {
                initial = observations[camera].center;
                support_mask = CAR_LAMP_CAMERA_BIT(camera);
                break;
            }
        }
    }

    if(support_mask == 0U)
    {
        for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
        {
            if(s_tracker.acquire_count[camera] != 0U)
            {
                pending_mask |= CAR_LAMP_CAMERA_BIT(camera);
            }
        }
        car_lamp_cross_check_set_state(
            ((pending_mask != 0U) ||
             (s_tracker.acquire_pair_count != 0U)) ?
            CAR_LAMP_TRACK_ACQUIRE : CAR_LAMP_TRACK_SEARCH);
        return 0U;
    }

    s_tracker.active = 1U;
    s_tracker.position = initial;
    s_tracker.velocity.x = 0.0f;
    s_tracker.velocity.y = 0.0f;
    s_tracker.last_update_time_ms = anchor_time_ms;
    s_tracker.last_support_time_ms = anchor_time_ms;
    s_tracker.support_camera_mask = support_mask;
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        s_tracker.accepted_frame_sequence[camera] =
            ((support_mask & CAR_LAMP_CAMERA_BIT(camera)) != 0U) ?
            observations[camera].sequence : 0U;
    }
    memset(s_tracker.acquire_count, 0, sizeof(s_tracker.acquire_count));
    memset(s_tracker.acquire_first_time_ms, 0,
           sizeof(s_tracker.acquire_first_time_ms));
    s_tracker.acquire_pair_mask = 0U;
    s_tracker.acquire_pair_count = 0U;
    s_tracker.acquire_pair_first_time_ms = 0U;
    memset(s_tracker.acquire_pair_sequence, 0,
           sizeof(s_tracker.acquire_pair_sequence));
    car_lamp_cross_check_set_state(CAR_LAMP_TRACK_TRACKED);
    return 1U;
}

/**
 * @brief 从匹配观测中优先选择一致双摄并形成锚点时刻测量。
 * @param observations 三摄观测数组。
 * @param match_mask 与预测轨迹匹配的摄像头位掩码。
 * @param anchor_time_ms 本摄锚点时间，单位ms。
 * @param measurement 输出锚点时刻的公共坐标测量。
 * @return 实际用于更新轨迹的摄像头位掩码。
 */
static uint8 car_lamp_cross_check_select_measurement(
    car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint8 match_mask,
    uint32 anchor_time_ms,
    car_lamp_projection_point_t *measurement)
{
    uint8 camera;
    uint8 selected_mask;
    uint8 selected_count = 0U;
    float best_error = 1000000.0f;

    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        car_lamp_observation_t *observation = &observations[camera];
        uint8 camera_bit = CAR_LAMP_CAMERA_BIT(camera);
        int32 delta_ms = (int32)(anchor_time_ms - observation->time_ms);
        float delta_s = (float)delta_ms * 0.001f;

        if((match_mask & camera_bit) == 0U)
        {
            observation->aligned_center = observation->center;
            continue;
        }
        observation->aligned_center.x =
            observation->center.x + s_tracker.velocity.x * delta_s;
        observation->aligned_center.y =
            observation->center.y + s_tracker.velocity.y * delta_s;
    }

    selected_mask = car_lamp_cross_check_best_pair(observations, match_mask);
    if(selected_mask == 0U)
    {
        for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
        {
            if(((match_mask & CAR_LAMP_CAMERA_BIT(camera)) != 0U) &&
               (observations[camera].error < best_error))
            {
                best_error = observations[camera].error;
                selected_mask = CAR_LAMP_CAMERA_BIT(camera);
            }
        }
    }
    else
    {
        car_lamp_projection_point_t pair_center = {0.0f, 0.0f};

        for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
        {
            if((selected_mask & CAR_LAMP_CAMERA_BIT(camera)) != 0U)
            {
                pair_center.x += observations[camera].aligned_center.x;
                pair_center.y += observations[camera].aligned_center.y;
            }
        }
        pair_center.x *= 0.5f;
        pair_center.y *= 0.5f;
        for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
        {
            uint8 camera_bit = CAR_LAMP_CAMERA_BIT(camera);

            if(((match_mask & camera_bit) != 0U) &&
               ((selected_mask & camera_bit) == 0U) &&
               (car_lamp_cross_check_distance(
                    &pair_center, &observations[camera].aligned_center) <=
                car_lamp_cross_check_camera_gate((image_camera_e)camera)))
            {
                selected_mask |= camera_bit;
            }
        }
    }

    measurement->x = 0.0f;
    measurement->y = 0.0f;
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        if((selected_mask & CAR_LAMP_CAMERA_BIT(camera)) != 0U)
        {
            measurement->x += observations[camera].aligned_center.x;
            measurement->y += observations[camera].aligned_center.y;
            selected_count++;
        }
    }
    if(selected_count != 0U)
    {
        measurement->x /= (float)selected_count;
        measurement->y /= (float)selected_count;
    }
    return selected_mask;
}

/**
 * @brief 用符合轨迹门限的原始实测更新位置和速度。
 * @param observations 三摄观测数组。
 * @param measured_mask 当前存在原始实测的摄像头掩码。
 * @param anchor_time_ms 本摄锚点时间，单位ms。
 * @return 实际支持本轮轨迹更新的摄像头掩码。
 */
static uint8 car_lamp_cross_check_update_track(
    car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint8 measured_mask,
    uint32 anchor_time_ms)
{
    car_lamp_projection_point_t predicted;
    car_lamp_projection_point_t measurement;
    uint8 camera;
    uint8 match_mask = 0U;
    uint8 retained_mask = 0U;
    uint8 selected_mask;
    uint32 delta_ms;
    float delta_s;
    float measured_velocity_x;
    float measured_velocity_y;

    car_lamp_cross_check_predict(anchor_time_ms, &predicted);
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        car_lamp_projection_point_t predicted_at_frame;
        car_lamp_observation_t *observation = &observations[camera];

        if(observation->valid == 0U)
        {
            continue;
        }
        if(observation->fresh == 0U)
        {
            uint8 camera_bit = CAR_LAMP_CAMERA_BIT(camera);

            if(((s_tracker.support_camera_mask & camera_bit) != 0U) &&
               (observation->sequence ==
                s_tracker.accepted_frame_sequence[camera]))
            {
                retained_mask |= camera_bit;
            }
            continue;
        }
        car_lamp_cross_check_predict(
            observation->time_ms, &predicted_at_frame);
        observation->error = car_lamp_cross_check_distance(
            &predicted_at_frame, &observation->center);
        s_tracker.diag.match_error[camera] = observation->error;
        if(observation->error <=
           car_lamp_cross_check_camera_gate((image_camera_e)camera))
        {
            match_mask |= CAR_LAMP_CAMERA_BIT(camera);
        }
        else
        {
            s_tracker.diag.conflict_camera_mask |=
                CAR_LAMP_CAMERA_BIT(camera);
        }
    }

    selected_mask = car_lamp_cross_check_select_measurement(
        observations, match_mask, anchor_time_ms, &measurement);
    s_tracker.diag.conflict_camera_mask |=
        (uint8)(measured_mask & (uint8)(~selected_mask));
    if(selected_mask == 0U)
    {
        if(retained_mask != 0U)
        {
            s_tracker.position = predicted;
            s_tracker.last_update_time_ms = anchor_time_ms;
            s_tracker.last_support_time_ms = anchor_time_ms;
            s_tracker.support_camera_mask = retained_mask;
            car_lamp_cross_check_set_state(CAR_LAMP_TRACK_TRACKED);
            return retained_mask;
        }

        uint32 coast_ms =
            (uint32)(anchor_time_ms - s_tracker.last_support_time_ms);
        uint32 coast_step_ms =
            (uint32)(anchor_time_ms - s_tracker.last_update_time_ms);

        if(coast_ms <= CAR_LAMP_TRACK_COAST_MAX_MS)
        {
            s_tracker.position = predicted;
            s_tracker.last_update_time_ms = anchor_time_ms;
            s_tracker.diag.coast_total_ms +=
                coast_step_ms;
            if(coast_ms > g_car_lamp_coast_max_ms)
            {
                g_car_lamp_coast_max_ms = coast_ms;
            }
            car_lamp_cross_check_set_state(CAR_LAMP_TRACK_COAST);
        }
        else
        {
            s_tracker.active = 0U;
            s_tracker.support_camera_mask = 0U;
            s_tracker.velocity.x = 0.0f;
            s_tracker.velocity.y = 0.0f;
            memset(s_tracker.accepted_frame_sequence, 0,
                   sizeof(s_tracker.accepted_frame_sequence));
            memset(s_tracker.acquire_count, 0,
                   sizeof(s_tracker.acquire_count));
            memset(s_tracker.acquire_first_time_ms, 0,
                   sizeof(s_tracker.acquire_first_time_ms));
            s_tracker.acquire_pair_mask = 0U;
            s_tracker.acquire_pair_count = 0U;
            s_tracker.acquire_pair_first_time_ms = 0U;
            memset(s_tracker.acquire_pair_sequence, 0,
                   sizeof(s_tracker.acquire_pair_sequence));
            car_lamp_cross_check_set_state(CAR_LAMP_TRACK_LOST);
        }
        return 0U;
    }

    delta_ms = (uint32)(anchor_time_ms - s_tracker.last_update_time_ms);
    if(delta_ms == 0U)
    {
        delta_ms = 1U;
    }
    delta_s = (float)delta_ms * 0.001f;
    measured_velocity_x =
        (measurement.x - s_tracker.position.x) / delta_s;
    measured_velocity_y =
        (measurement.y - s_tracker.position.y) / delta_s;
    s_tracker.position.x =
        predicted.x + CAR_LAMP_TRACK_POSITION_GAIN *
        (measurement.x - predicted.x);
    s_tracker.position.y =
        predicted.y + CAR_LAMP_TRACK_POSITION_GAIN *
        (measurement.y - predicted.y);
    s_tracker.velocity.x =
        CAR_LAMP_TRACK_OLD_VELOCITY * s_tracker.velocity.x +
        CAR_LAMP_TRACK_NEW_VELOCITY * measured_velocity_x;
    s_tracker.velocity.y =
        CAR_LAMP_TRACK_OLD_VELOCITY * s_tracker.velocity.y +
        CAR_LAMP_TRACK_NEW_VELOCITY * measured_velocity_y;
    s_tracker.last_update_time_ms = anchor_time_ms;
    s_tracker.last_support_time_ms = anchor_time_ms;
    if(selected_mask != s_tracker.support_camera_mask)
    {
        g_car_lamp_source_handoff_count++;
        s_tracker.diag.source_switch_count =
            g_car_lamp_source_handoff_count;
    }
    s_tracker.support_camera_mask = selected_mask;
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        s_tracker.accepted_frame_sequence[camera] =
            ((selected_mask & CAR_LAMP_CAMERA_BIT(camera)) != 0U) ?
            observations[camera].sequence : 0U;
    }
    car_lamp_cross_check_set_state(CAR_LAMP_TRACK_TRACKED);
    return selected_mask;
}

/**
 * @brief 计算三摄预计ROI并统计当前全图实测的影子命中率。
 * @param observations 三摄观测数组。
 * @param anchor_time_ms 本摄锚点时间，单位ms。
 * @return 无。
 */
static void car_lamp_cross_check_update_roi(
    const car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint32 anchor_time_ms)
{
    uint8 camera;
    car_lamp_projection_point_t predicted;

    if(s_tracker.active == 0U)
    {
        return;
    }
    car_lamp_cross_check_predict(anchor_time_ms, &predicted);
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        car_lamp_projection_point_t expected;
        float base_half_size =
            (camera == Back) ? CAR_LAMP_TRACK_ROI_BACK_PX :
            ((camera == Front) ? CAR_LAMP_TRACK_ROI_FRONT_PX :
                                 CAR_LAMP_TRACK_ROI_CENTER_PX);

        if(car_lamp_cross_check_project_roi(
               (image_camera_e)camera, &predicted,
               base_half_size + CAR_LAMP_TRACK_ROI_MARGIN_PX,
               &expected) == 0U)
        {
            continue;
        }
        s_tracker.diag.expected_x[camera] = expected.x;
        s_tracker.diag.expected_y[camera] = expected.y;
        s_tracker.diag.roi_valid_mask |= CAR_LAMP_CAMERA_BIT(camera);
        s_tracker.diag.roi_half_size[camera] =
            base_half_size + CAR_LAMP_TRACK_ROI_MARGIN_PX;

        if((observations[camera].valid != 0U) &&
           (observations[camera].fresh != 0U))
        {
            car_lamp_projection_point_t frame_prediction;
            car_lamp_projection_point_t frame_expected;
            uint32 skew_ms = image_frame_time_difference_ms(
                observations[camera].time_ms, anchor_time_ms);
            float speed = sqrtf(
                s_tracker.velocity.x * s_tracker.velocity.x +
                s_tracker.velocity.y * s_tracker.velocity.y);
            float half_size =
                base_half_size +
                s_tracker.last_lamp_length[camera] * 0.5f +
                speed * (float)skew_ms * 0.001f +
                CAR_LAMP_TRACK_ROI_MARGIN_PX;

            car_lamp_cross_check_predict(
                observations[camera].time_ms, &frame_prediction);
            if(car_lamp_cross_check_project_roi(
                   (image_camera_e)camera, &frame_prediction,
                   half_size, &frame_expected) == 0U)
            {
                continue;
            }
            s_tracker.diag.roi_half_size[camera] = half_size;
            g_car_lamp_roi_sample_count[camera]++;
            if((fabsf(observations[camera].source.x -
                      frame_expected.x) <= half_size) &&
               (fabsf(observations[camera].source.y -
                      frame_expected.y) <= half_size))
            {
                s_tracker.diag.roi_hit_mask |=
                    CAR_LAMP_CAMERA_BIT(camera);
                g_car_lamp_roi_hit_count[camera]++;
            }
        }
    }
}

/**
 * @brief 初始化单车灯三摄公共轨迹影子状态机。
 * @param local_camera 当前芯片本地摄像头，用作最新帧组更新锚点。
 * @return 无。
 */
void CarLampCrossCheck_Init(image_camera_e local_camera)
{
    uint8 camera;

    memset(&s_tracker, 0, sizeof(s_tracker));
    memset(g_car_lamp_roi_sample_count, 0,
           sizeof(g_car_lamp_roi_sample_count));
    memset(g_car_lamp_roi_hit_count, 0,
           sizeof(g_car_lamp_roi_hit_count));
    g_car_lamp_source_handoff_count = 0U;
    g_car_lamp_coast_max_ms = 0U;
    g_car_lamp_full_frame_fallback_count = 0U;
    s_tracker.local_camera = local_camera;
    s_tracker.initialized =
        (local_camera < IMAGE_CAMERA_COUNT) ? 1U : 0U;
    s_tracker.diag.state = (uint8)CAR_LAMP_TRACK_SEARCH;
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        s_tracker.diag.expected_x[camera] = IMAGE_DATA_INVALID_VALUE;
        s_tracker.diag.expected_y[camera] = IMAGE_DATA_INVALID_VALUE;
        s_tracker.diag.match_error[camera] =
            CAR_LAMP_CROSS_CHECK_INVALID_ERROR;
    }
}

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
                               uint8 height_valid)
{
    const image_frame_meta_t *anchor;
    car_lamp_observation_t observations[IMAGE_CAMERA_COUNT];
    uint8 camera;
    uint8 measured_mask = 0U;

    if((frames == 0) || (s_tracker.initialized == 0U))
    {
        return 0U;
    }
    anchor = &frames->meta[s_tracker.local_camera];
    if((anchor->frame_valid == 0U) ||
       (anchor->frame_sequence == 0U) ||
       (anchor->source_camera != (uint8)s_tracker.local_camera) ||
       (anchor->frame_sequence == s_tracker.last_anchor_sequence))
    {
        return 0U;
    }

    s_tracker.last_anchor_sequence = anchor->frame_sequence;
    s_tracker.diag.anchor_sequence = anchor->frame_sequence;
    s_tracker.diag.anchor_time_ms = anchor->capture_time_ms;
    s_tracker.diag.support_camera_mask = 0U;
    s_tracker.diag.roi_valid_mask = 0U;
    s_tracker.diag.roi_hit_mask = 0U;
    s_tracker.diag.conflict_camera_mask = 0U;
    s_tracker.diag.full_frame_fallback_mask = 0U;
    s_tracker.diag.projection_enabled =
        car_lamp_cross_check_projection_enabled(
            anchor, roll_deg, pitch_deg, height_mm,
            attitude_valid, height_valid);
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        s_tracker.diag.expected_x[camera] = IMAGE_DATA_INVALID_VALUE;
        s_tracker.diag.expected_y[camera] = IMAGE_DATA_INVALID_VALUE;
        s_tracker.diag.match_error[camera] =
            CAR_LAMP_CROSS_CHECK_INVALID_ERROR;
        s_tracker.diag.roi_half_size[camera] = 0.0f;
    }

    if(s_tracker.diag.projection_enabled != 0U)
    {
        measured_mask = car_lamp_cross_check_build_observations(
            frames, observations, anchor->capture_time_ms);
        for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
        {
            if((observations[camera].valid != 0U) &&
               (observations[camera].fresh != 0U))
            {
                s_tracker.last_lamp_length[camera] =
                    observations[camera].length;
            }
        }
        if(s_tracker.active == 0U)
        {
            (void)car_lamp_cross_check_acquire(
                observations, anchor->capture_time_ms);
        }
        else
        {
            (void)car_lamp_cross_check_update_track(
                observations, measured_mask, anchor->capture_time_ms);
        }
        if(s_tracker.active != 0U)
        {
            car_lamp_cross_check_update_roi(
                observations, anchor->capture_time_ms);
        }
    }
    else if(s_tracker.active != 0U)
    {
        car_lamp_observation_t empty[IMAGE_CAMERA_COUNT];

        memset(empty, 0, sizeof(empty));
        (void)car_lamp_cross_check_update_track(
            empty, 0U, anchor->capture_time_ms);
    }

    s_tracker.diag.support_camera_mask =
        (s_tracker.diag.state == (uint8)CAR_LAMP_TRACK_TRACKED) ?
        s_tracker.support_camera_mask : 0U;
    s_tracker.diag.center_x =
        (s_tracker.active != 0U) ?
        s_tracker.position.x : IMAGE_DATA_INVALID_VALUE;
    s_tracker.diag.center_y =
        (s_tracker.active != 0U) ?
        s_tracker.position.y : IMAGE_DATA_INVALID_VALUE;
    s_tracker.diag.velocity_x =
        (s_tracker.active != 0U) ? s_tracker.velocity.x : 0.0f;
    s_tracker.diag.velocity_y =
        (s_tracker.active != 0U) ? s_tracker.velocity.y : 0.0f;
    s_tracker.diag.source_switch_count =
        g_car_lamp_source_handoff_count;
    s_tracker.diag.full_frame_fallback_count =
        g_car_lamp_full_frame_fallback_count;
    return 1U;
}

/**
 * @brief 读取当前公共轨迹和影子ROI诊断。
 * @return 只读诊断指针，生命周期覆盖整个程序运行期。
 */
const car_lamp_cross_check_diag_t *CarLampCrossCheck_GetDiag(void)
{
    return &s_tracker.diag;
}
