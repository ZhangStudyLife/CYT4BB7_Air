#include "car_lamp_cross_check.h"

#include <math.h>
#include <string.h>

#define CAR_LAMP_ACQUIRE_SINGLE_FRAMES       (3U)
#define CAR_LAMP_ACQUIRE_PAIR_FRAMES         (2U)
#define CAR_LAMP_ACQUIRE_MAX_GAP_MS          (80U)
#define CAR_LAMP_ACQUIRE_TOTAL_WINDOW_MS     (160U)
#define CAR_LAMP_TRACK_STRONG_MAX_AGE_MS      (30U)
#define CAR_LAMP_TRACK_POSITIVE_MAX_AGE_MS    (50U)
#define CAR_LAMP_TRACK_LOCATE_MAX_AGE_MS      (80U)
#define CAR_LAMP_TRACK_COAST_MAX_MS           (50U)
#define CAR_LAMP_TRACK_PAIR_MAX_SKEW_MS       (30U)
#define CAR_LAMP_TRACK_FRONT_GATE_PX          (8.0f)
#define CAR_LAMP_TRACK_CENTER_GATE_PX         (8.0f)
#define CAR_LAMP_TRACK_BACK_GATE_PX           (12.0f)
#define CAR_LAMP_TRACK_FB_GATE_PX             (16.0f)
#define CAR_LAMP_TRACK_FB_SOFT_GATE_PX        (20.0f) /* 前后摄仅记录提示、不参与确认的软门限。 */
#define CAR_LAMP_TRACK_OLD_VELOCITY           (0.7f)
#define CAR_LAMP_TRACK_NEW_VELOCITY           (0.3f)
#define CAR_LAMP_TRACK_MIN_VELOCITY_DT_MS     (5U)
#define CAR_LAMP_TRACK_MAX_VELOCITY_DT_MS     (80U)
#define CAR_LAMP_TRACK_MIN_HEIGHT_MM          (800.0f)
#define CAR_LAMP_TRACK_MAX_HEIGHT_MM          (1300.0f)
#define CAR_LAMP_TRACK_SINGLE_TILT_DEG        (20.0f)
#define CAR_LAMP_TRACK_MAX_TILT_DEG           (30.0f)
#define CAR_LAMP_TRACK_ROI_FRONT_PX           (20.0f)
#define CAR_LAMP_TRACK_ROI_CENTER_PX          (20.0f)
#define CAR_LAMP_TRACK_ROI_BACK_PX            (24.0f)
#define CAR_LAMP_TRACK_ROI_DEFAULT_HALF_PX    (6.0f) /* 没有新鲜灯条长度时采用的半长。 */
#define CAR_LAMP_TRACK_ROI_MARGIN_PX          (2.0f)
#define CAR_LAMP_TRACK_ROI_LOCATE_MIN_PX      (30.0f)
#define CAR_LAMP_TRACK_ROI_MAX_PX             (37.0f)

typedef struct
{
    uint8 valid;
    uint8 frame_fresh;
    uint8 evidence_level;
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
    uint8 has_solution;
    uint8 support_camera_mask;
    uint8 acquire_count[IMAGE_CAMERA_COUNT];
    uint8 acquire_pair_mask;
    uint8 acquire_pair_count;
    uint8 acquire_pair_changed_mask;
    uint32 last_frame_sequence[IMAGE_CAMERA_COUNT];
    uint32 last_frame_time_ms[IMAGE_CAMERA_COUNT];
    uint32 acquire_first_time_ms[IMAGE_CAMERA_COUNT];
    uint32 acquire_time_ms[IMAGE_CAMERA_COUNT];
    uint32 acquire_pair_sequence[IMAGE_CAMERA_COUNT];
    uint32 acquire_pair_first_time_ms;
    uint32 acquire_pair_time_ms;
    uint32 last_update_time_ms;
    uint32 last_support_time_ms;
    uint32 last_eval_time_ms;
    car_lamp_projection_point_t acquire_point[IMAGE_CAMERA_COUNT];
    car_lamp_projection_point_t position;
    car_lamp_projection_point_t velocity;
    car_lamp_track_snapshot_t snapshot;
    car_lamp_cross_check_diag_t diag;
} car_lamp_cross_check_tracker_t;

static car_lamp_cross_check_tracker_t s_tracker;

uint32 g_car_lamp_roi_sample_count[IMAGE_CAMERA_COUNT];
uint32 g_car_lamp_roi_hit_count[IMAGE_CAMERA_COUNT];
uint32 g_car_lamp_source_handoff_count;
uint32 g_car_lamp_last_handoff_frame_sequence[IMAGE_CAMERA_COUNT];
uint32 g_car_lamp_coast_max_ms;
uint32 g_car_lamp_full_frame_fallback_count;
uint32 g_car_lamp_stale_frame_reject_count;
uint32 g_car_lamp_duplicate_frame_reject_count;
uint32 g_car_lamp_invalid_time_reject_count;
uint32 g_car_lamp_soft_pair_hint_count; /* 前后摄16至20像素软提示累计次数。 */
uint8 g_car_lamp_roi_mode;

static float car_lamp_cross_check_distance(
    const car_lamp_projection_point_t *left,
    const car_lamp_projection_point_t *right)
{
    float dx = left->x - right->x;
    float dy = left->y - right->y;

    return sqrtf(dx * dx + dy * dy);
}

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

static uint8 car_lamp_cross_check_evidence(uint32 now_ms,
                                           uint32 capture_time_ms)
{
    uint32 age_ms = image_frame_time_difference_ms(now_ms, capture_time_ms);

    if(age_ms <= CAR_LAMP_TRACK_STRONG_MAX_AGE_MS)
    {
        return (uint8)CAR_LAMP_EVIDENCE_STRONG;
    }
    if(age_ms <= CAR_LAMP_TRACK_POSITIVE_MAX_AGE_MS)
    {
        return (uint8)CAR_LAMP_EVIDENCE_POSITIVE;
    }
    if(age_ms <= CAR_LAMP_TRACK_LOCATE_MAX_AGE_MS)
    {
        return (uint8)CAR_LAMP_EVIDENCE_LOCATE;
    }
    return (uint8)CAR_LAMP_EVIDENCE_NONE;
}

static uint8 car_lamp_cross_check_sequence_is_new(
    image_camera_e camera,
    uint32 sequence,
    uint32 capture_time_ms,
    uint8 timestamp_valid)
{
    uint32 previous = s_tracker.last_frame_sequence[camera];

    if(previous == 0U)
    {
        return 1U;
    }
    if((int32)(sequence - previous) > 0)
    {
        return 1U;
    }

    /* 图像板重启后帧号会回到1；只有统一时间明确前进时才接受复位。 */
    if((timestamp_valid != 0U) &&
       ((int32)(capture_time_ms -
                s_tracker.last_frame_time_ms[camera]) >
        (int32)CAR_LAMP_TRACK_LOCATE_MAX_AGE_MS))
    {
        return 1U;
    }
    return 0U;
}

static void car_lamp_cross_check_predict_from(
    const car_lamp_projection_point_t *position,
    const car_lamp_projection_point_t *velocity,
    uint32 reference_time_ms,
    uint32 target_time_ms,
    car_lamp_projection_point_t *point)
{
    int32 delta_ms = (int32)(target_time_ms - reference_time_ms);
    float delta_s = (float)delta_ms * 0.001f;

    point->x = position->x + velocity->x * delta_s;
    point->y = position->y + velocity->y * delta_s;
}

static void car_lamp_cross_check_predict(
    uint32 target_time_ms,
    car_lamp_projection_point_t *point)
{
    car_lamp_cross_check_predict_from(
        &s_tracker.position, &s_tracker.velocity,
        s_tracker.last_update_time_ms, target_time_ms, point);
}

static void car_lamp_cross_check_set_state(car_lamp_track_state_e state)
{
    if(s_tracker.diag.state != (uint8)state)
    {
        s_tracker.diag.transition_count++;
        s_tracker.diag.state = (uint8)state;
    }
}

static uint8 car_lamp_cross_check_projection_enabled(
    float roll_deg,
    float pitch_deg,
    float height_mm,
    uint8 attitude_valid,
    uint8 height_valid)
{
    if((attitude_valid == 0U) || (height_valid == 0U))
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

static uint8 car_lamp_cross_check_high_tilt(float roll_deg,
                                             float pitch_deg)
{
    return ((fabsf(roll_deg) > CAR_LAMP_TRACK_SINGLE_TILT_DEG) ||
            (fabsf(pitch_deg) > CAR_LAMP_TRACK_SINGLE_TILT_DEG)) ? 1U : 0U;
}

static uint8 car_lamp_cross_check_build_observations(
    const image_sync_set_t *frames,
    uint32 now_ms,
    car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint8 *strong_measured_mask)
{
    uint8 camera;
    uint8 fresh_mask = 0U;

    *strong_measured_mask = 0U;
    memset(observations, 0,
           sizeof(car_lamp_observation_t) * IMAGE_CAMERA_COUNT);
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        const image_frame_meta_t *meta = &frames->meta[camera];
        const struct image_data *data = &frames->camera[camera];
        const car_lamp_data *lamp = &data->car_lamp_data[0];
        car_lamp_observation_t *observation = &observations[camera];

        observation->camera = (image_camera_e)camera;
        observation->error = CAR_LAMP_CROSS_CHECK_INVALID_ERROR;
        if((meta->frame_valid == 0U) || (meta->frame_sequence == 0U) ||
           (meta->source_camera != camera))
        {
            continue;
        }
        if(car_lamp_cross_check_sequence_is_new(
               (image_camera_e)camera, meta->frame_sequence,
               meta->capture_time_ms, meta->timestamp_valid) != 0U)
        {
            observation->frame_fresh = 1U;
            observation->sequence = meta->frame_sequence;
            s_tracker.last_frame_sequence[camera] = meta->frame_sequence;
            fresh_mask |= CAR_LAMP_CAMERA_BIT(camera);
        }
        else if(meta->frame_sequence ==
                s_tracker.last_frame_sequence[camera])
        {
            g_car_lamp_duplicate_frame_reject_count++;
            observation->sequence = meta->frame_sequence;
        }
        else
        {
            g_car_lamp_duplicate_frame_reject_count++;
            continue;
        }
        if(meta->timestamp_valid == 0U)
        {
            if(observation->frame_fresh != 0U)
            {
                g_car_lamp_invalid_time_reject_count++;
            }
            continue;
        }

        observation->time_ms = meta->capture_time_ms;
        s_tracker.last_frame_time_ms[camera] = meta->capture_time_ms;
        observation->evidence_level =
            car_lamp_cross_check_evidence(now_ms, meta->capture_time_ms);
        if(observation->evidence_level ==
           (uint8)CAR_LAMP_EVIDENCE_NONE)
        {
            if(observation->frame_fresh != 0U)
            {
                g_car_lamp_stale_frame_reject_count++;
            }
            continue;
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
        observation->length = lamp->length;
        if(observation->evidence_level ==
           (uint8)CAR_LAMP_EVIDENCE_STRONG &&
           (observation->frame_fresh != 0U))
        {
            *strong_measured_mask |= CAR_LAMP_CAMERA_BIT(camera);
        }
    }
    return fresh_mask;
}

static uint32 car_lamp_cross_check_newest_time(
    const car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint8 mask)
{
    uint8 camera;
    uint8 initialized = 0U;
    uint32 newest = 0U;

    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        if((mask & CAR_LAMP_CAMERA_BIT(camera)) == 0U)
        {
            continue;
        }
        if((initialized == 0U) ||
           ((int32)(observations[camera].time_ms - newest) > 0))
        {
            newest = observations[camera].time_ms;
            initialized = 1U;
        }
    }
    return newest;
}

static void car_lamp_cross_check_align_observations(
    car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint8 mask,
    uint32 target_time_ms)
{
    uint8 camera;

    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        car_lamp_observation_t *observation = &observations[camera];

        observation->aligned_center = observation->center;
        if((mask & CAR_LAMP_CAMERA_BIT(camera)) == 0U)
        {
            continue;
        }
        car_lamp_cross_check_predict_from(
            &observation->center, &s_tracker.velocity,
            observation->time_ms, target_time_ms,
            &observation->aligned_center);
    }
}

/**
 * @brief 记录前后摄位于硬门外、软门内的一致性提示，不改变轨迹裁决。
 * @param observations 当前三摄观测及统一时间。
 * @param strong_measured_mask 本轮新鲜强实测来源位掩码。
 * @return 无。
 */
static void car_lamp_cross_check_record_soft_pair_hint(
    car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint8 strong_measured_mask)
{
    const uint8 pair_mask = (uint8)(CAR_LAMP_CAMERA_BIT(Front) |
                                    CAR_LAMP_CAMERA_BIT(Back));
    uint32 target_time_ms;
    float distance;

    if((strong_measured_mask & pair_mask) != pair_mask)
    {
        return;
    }
    if(image_frame_time_difference_ms(
           observations[Front].time_ms,
           observations[Back].time_ms) > CAR_LAMP_TRACK_PAIR_MAX_SKEW_MS)
    {
        return;
    }
    target_time_ms = car_lamp_cross_check_newest_time(
        observations, pair_mask);
    car_lamp_cross_check_align_observations(
        observations, pair_mask, target_time_ms);
    distance = car_lamp_cross_check_distance(
        &observations[Front].aligned_center,
        &observations[Back].aligned_center);
    if((distance > CAR_LAMP_TRACK_FB_GATE_PX) &&
       (distance <= CAR_LAMP_TRACK_FB_SOFT_GATE_PX))
    {
        g_car_lamp_soft_pair_hint_count++;
    }
}

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
            right < IMAGE_CAMERA_COUNT; right++)
        {
            uint8 pair_mask = (uint8)(CAR_LAMP_CAMERA_BIT(left) |
                                      CAR_LAMP_CAMERA_BIT(right));
            float distance;

            if((candidate_mask & pair_mask) != pair_mask)
            {
                continue;
            }
            if(image_frame_time_difference_ms(
                   observations[left].time_ms,
                   observations[right].time_ms) >
               CAR_LAMP_TRACK_PAIR_MAX_SKEW_MS)
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

static void car_lamp_cross_check_reset_acquire(void)
{
    memset(s_tracker.acquire_count, 0, sizeof(s_tracker.acquire_count));
    memset(s_tracker.acquire_first_time_ms, 0,
           sizeof(s_tracker.acquire_first_time_ms));
    memset(s_tracker.acquire_time_ms, 0,
           sizeof(s_tracker.acquire_time_ms));
    s_tracker.acquire_pair_mask = 0U;
    s_tracker.acquire_pair_count = 0U;
    s_tracker.acquire_pair_changed_mask = 0U;
    s_tracker.acquire_pair_first_time_ms = 0U;
    s_tracker.acquire_pair_time_ms = 0U;
    memset(s_tracker.acquire_pair_sequence, 0,
           sizeof(s_tracker.acquire_pair_sequence));
}

static uint8 car_lamp_cross_check_acquire(
    car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint32 now_ms,
    uint8 high_tilt)
{
    uint8 camera;
    uint8 strong_valid_mask = 0U;
    uint8 fresh_strong_mask = 0U;
    uint8 pending_mask = 0U;
    uint8 pair_mask;
    uint8 support_mask = 0U;
    uint32 support_time_ms = 0U;
    car_lamp_projection_point_t initial = {0.0f, 0.0f};

    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        car_lamp_observation_t *observation = &observations[camera];

        observation->aligned_center = observation->center;
        if((observation->valid != 0U) &&
           (observation->evidence_level ==
            (uint8)CAR_LAMP_EVIDENCE_STRONG))
        {
            strong_valid_mask |= CAR_LAMP_CAMERA_BIT(camera);
            if(observation->frame_fresh != 0U)
            {
                fresh_strong_mask |= CAR_LAMP_CAMERA_BIT(camera);
            }
        }
        if((observation->frame_fresh == 0U) ||
           (observation->evidence_level !=
            (uint8)CAR_LAMP_EVIDENCE_STRONG))
        {
            continue;
        }
        if(observation->valid == 0U)
        {
            s_tracker.acquire_count[camera] = 0U;
            s_tracker.acquire_first_time_ms[camera] = 0U;
            continue;
        }

        if((s_tracker.acquire_count[camera] != 0U) &&
           ((uint32)(observation->time_ms -
                     s_tracker.acquire_time_ms[camera]) <=
            CAR_LAMP_ACQUIRE_MAX_GAP_MS) &&
           ((uint32)(observation->time_ms -
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
            s_tracker.acquire_first_time_ms[camera] = observation->time_ms;
        }
        s_tracker.acquire_point[camera] = observation->center;
        s_tracker.acquire_time_ms[camera] = observation->time_ms;
    }

    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        if((s_tracker.acquire_count[camera] != 0U) &&
           (image_frame_time_difference_ms(
                now_ms, s_tracker.acquire_time_ms[camera]) >
            CAR_LAMP_ACQUIRE_MAX_GAP_MS))
        {
            s_tracker.acquire_count[camera] = 0U;
            s_tracker.acquire_first_time_ms[camera] = 0U;
        }
    }
    if((s_tracker.acquire_pair_count != 0U) &&
       (image_frame_time_difference_ms(
            now_ms, s_tracker.acquire_pair_time_ms) >
        CAR_LAMP_ACQUIRE_MAX_GAP_MS))
    {
        s_tracker.acquire_pair_mask = 0U;
        s_tracker.acquire_pair_count = 0U;
        s_tracker.acquire_pair_changed_mask = 0U;
        s_tracker.acquire_pair_first_time_ms = 0U;
        memset(s_tracker.acquire_pair_sequence, 0,
               sizeof(s_tracker.acquire_pair_sequence));
    }

    support_time_ms = car_lamp_cross_check_newest_time(
        observations, strong_valid_mask);
    car_lamp_cross_check_align_observations(
        observations, strong_valid_mask, support_time_ms);
    pair_mask = car_lamp_cross_check_best_pair(
        observations, strong_valid_mask);
    if(pair_mask != 0U)
    {
        uint8 changed_mask = 0U;

        support_time_ms = car_lamp_cross_check_newest_time(
            observations, pair_mask);
        car_lamp_cross_check_align_observations(
            observations, pair_mask, support_time_ms);
        for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
        {
            if(((pair_mask & CAR_LAMP_CAMERA_BIT(camera)) != 0U) &&
               (observations[camera].sequence !=
                s_tracker.acquire_pair_sequence[camera]))
            {
                changed_mask |= CAR_LAMP_CAMERA_BIT(camera);
            }
        }
        if((pair_mask != s_tracker.acquire_pair_mask) ||
           (s_tracker.acquire_pair_count == 0U) ||
           ((uint32)(support_time_ms -
                     s_tracker.acquire_pair_time_ms) >
            CAR_LAMP_ACQUIRE_MAX_GAP_MS) ||
           ((uint32)(support_time_ms -
                     s_tracker.acquire_pair_first_time_ms) >
            CAR_LAMP_ACQUIRE_TOTAL_WINDOW_MS))
        {
            s_tracker.acquire_pair_mask = pair_mask;
            s_tracker.acquire_pair_count = 1U;
            s_tracker.acquire_pair_changed_mask = 0U;
            s_tracker.acquire_pair_first_time_ms = support_time_ms;
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
            s_tracker.acquire_pair_time_ms = support_time_ms;
        }
        else
        {
            s_tracker.acquire_pair_changed_mask |= changed_mask;
            if((s_tracker.acquire_pair_changed_mask & pair_mask) == pair_mask)
            {
                if(s_tracker.acquire_pair_count < 0xFFU)
                {
                    s_tracker.acquire_pair_count++;
                }
                for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
                {
                    if((pair_mask & CAR_LAMP_CAMERA_BIT(camera)) != 0U)
                    {
                        s_tracker.acquire_pair_sequence[camera] =
                            observations[camera].sequence;
                    }
                }
                s_tracker.acquire_pair_changed_mask = 0U;
                s_tracker.acquire_pair_time_ms = support_time_ms;
            }
        }
    }
    else if((s_tracker.acquire_pair_count != 0U) &&
            ((fresh_strong_mask & s_tracker.acquire_pair_mask) != 0U))
    {
        s_tracker.acquire_pair_mask = 0U;
        s_tracker.acquire_pair_count = 0U;
        s_tracker.acquire_pair_changed_mask = 0U;
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
                initial.x += observations[camera].aligned_center.x;
                initial.y += observations[camera].aligned_center.y;
                support_mask |= CAR_LAMP_CAMERA_BIT(camera);
            }
        }
        initial.x *= 0.5f;
        initial.y *= 0.5f;
    }
    else if(high_tilt == 0U)
    {
        for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
        {
            if((s_tracker.acquire_count[camera] >=
                CAR_LAMP_ACQUIRE_SINGLE_FRAMES) &&
               (observations[camera].valid != 0U) &&
               (observations[camera].evidence_level ==
                (uint8)CAR_LAMP_EVIDENCE_STRONG))
            {
                initial = observations[camera].center;
                support_time_ms = observations[camera].time_ms;
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
        if((pending_mask != 0U) ||
           (s_tracker.acquire_pair_count != 0U))
        {
            car_lamp_cross_check_set_state(CAR_LAMP_TRACK_ACQUIRE);
        }
        else if(s_tracker.has_solution != 0U)
        {
            car_lamp_cross_check_set_state(CAR_LAMP_TRACK_LOST);
        }
        else
        {
            car_lamp_cross_check_set_state(CAR_LAMP_TRACK_SEARCH);
        }
        return 0U;
    }

    s_tracker.active = 1U;
    s_tracker.has_solution = 1U;
    s_tracker.position = initial;
    s_tracker.velocity.x = 0.0f;
    s_tracker.velocity.y = 0.0f;
    s_tracker.last_update_time_ms = support_time_ms;
    s_tracker.last_support_time_ms = support_time_ms;
    s_tracker.support_camera_mask = support_mask;
    car_lamp_cross_check_reset_acquire();
    car_lamp_cross_check_set_state(CAR_LAMP_TRACK_TRACKED);
    return 1U;
}

static uint8 car_lamp_cross_check_select_measurement(
    car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint8 match_mask,
    car_lamp_projection_point_t *measurement,
    uint32 *measurement_time_ms)
{
    uint8 camera;
    uint8 selected_mask;
    uint8 selected_count = 0U;
    float best_error = 1000000.0f;

    *measurement_time_ms = car_lamp_cross_check_newest_time(
        observations, match_mask);
    car_lamp_cross_check_align_observations(
        observations, match_mask, *measurement_time_ms);
    selected_mask = car_lamp_cross_check_best_pair(
        observations, match_mask);
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

static uint8 car_lamp_cross_check_update_track(
    car_lamp_observation_t observations[IMAGE_CAMERA_COUNT],
    uint8 strong_measured_mask)
{
    car_lamp_projection_point_t predicted_at_frame;
    car_lamp_projection_point_t measurement;
    uint8 camera;
    uint8 match_mask = 0U;
    uint8 selected_mask;
    uint32 measurement_time_ms = 0U;
    uint32 delta_ms;

    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        car_lamp_observation_t *observation = &observations[camera];
        uint8 camera_bit = CAR_LAMP_CAMERA_BIT(camera);

        if((strong_measured_mask & camera_bit) == 0U)
        {
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
            match_mask |= camera_bit;
        }
        else
        {
            s_tracker.diag.conflict_camera_mask |= camera_bit;
        }
    }

    selected_mask = car_lamp_cross_check_select_measurement(
        observations, match_mask, &measurement, &measurement_time_ms);
    s_tracker.diag.conflict_camera_mask |=
        (uint8)(strong_measured_mask & (uint8)(~selected_mask));
    if((selected_mask == 0U) ||
       ((int32)(measurement_time_ms -
                s_tracker.last_update_time_ms) < 0))
    {
        return 0U;
    }

    delta_ms = (uint32)(measurement_time_ms -
                        s_tracker.last_update_time_ms);
    if((delta_ms >= CAR_LAMP_TRACK_MIN_VELOCITY_DT_MS) &&
       (delta_ms <= CAR_LAMP_TRACK_MAX_VELOCITY_DT_MS))
    {
        float delta_s = (float)delta_ms * 0.001f;
        float measured_velocity_x =
            (measurement.x - s_tracker.position.x) / delta_s;
        float measured_velocity_y =
            (measurement.y - s_tracker.position.y) / delta_s;

        s_tracker.velocity.x =
            CAR_LAMP_TRACK_OLD_VELOCITY * s_tracker.velocity.x +
            CAR_LAMP_TRACK_NEW_VELOCITY * measured_velocity_x;
        s_tracker.velocity.y =
            CAR_LAMP_TRACK_OLD_VELOCITY * s_tracker.velocity.y +
            CAR_LAMP_TRACK_NEW_VELOCITY * measured_velocity_y;
    }
    s_tracker.position = measurement;
    s_tracker.last_update_time_ms = measurement_time_ms;
    s_tracker.last_support_time_ms = measurement_time_ms;
    if(selected_mask != s_tracker.support_camera_mask)
    {
        g_car_lamp_source_handoff_count++;
        for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
        {
            g_car_lamp_last_handoff_frame_sequence[camera] =
                ((selected_mask & CAR_LAMP_CAMERA_BIT(camera)) != 0U) ?
                observations[camera].sequence : 0U;
        }
    }
    s_tracker.support_camera_mask = selected_mask;
    s_tracker.active = 1U;
    s_tracker.has_solution = 1U;
    car_lamp_cross_check_set_state(CAR_LAMP_TRACK_TRACKED);
    return selected_mask;
}

static void car_lamp_cross_check_age_track(uint32 now_ms)
{
    uint32 age_ms;
    uint32 step_ms = (uint32)(now_ms - s_tracker.last_eval_time_ms);

    s_tracker.last_eval_time_ms = now_ms;
    if((s_tracker.active == 0U) || (s_tracker.has_solution == 0U))
    {
        return;
    }
    age_ms = (uint32)(now_ms - s_tracker.last_support_time_ms);
    if((int32)age_ms < 0)
    {
        age_ms = 0U;
    }
    if(age_ms <= CAR_LAMP_TRACK_STRONG_MAX_AGE_MS)
    {
        car_lamp_cross_check_set_state(CAR_LAMP_TRACK_TRACKED);
        return;
    }
    if(age_ms <= CAR_LAMP_TRACK_COAST_MAX_MS)
    {
        if(step_ms <= CAR_LAMP_TRACK_LOCATE_MAX_AGE_MS)
        {
            s_tracker.diag.coast_total_ms += step_ms;
        }
        if(age_ms > g_car_lamp_coast_max_ms)
        {
            g_car_lamp_coast_max_ms = age_ms;
        }
        car_lamp_cross_check_set_state(CAR_LAMP_TRACK_COAST);
        return;
    }

    s_tracker.active = 0U;
    s_tracker.support_camera_mask = 0U;
    car_lamp_cross_check_reset_acquire();
    car_lamp_cross_check_set_state(CAR_LAMP_TRACK_LOST);
}

static uint8 car_lamp_cross_check_snapshot_quality(uint32 now_ms)
{
    if(s_tracker.has_solution == 0U)
    {
        return (uint8)CAR_LAMP_EVIDENCE_NONE;
    }
    return car_lamp_cross_check_evidence(
        now_ms, s_tracker.last_support_time_ms);
}

static void car_lamp_cross_check_publish_snapshot(uint32 now_ms)
{
    uint8 quality = car_lamp_cross_check_snapshot_quality(now_ms);

    s_tracker.snapshot.sequence++;
    if(s_tracker.snapshot.sequence == 0U)
    {
        s_tracker.snapshot.sequence = 1U;
    }
    s_tracker.snapshot.reference_time_ms = s_tracker.last_update_time_ms;
    s_tracker.snapshot.last_support_time_ms =
        s_tracker.last_support_time_ms;
    s_tracker.snapshot.center_x =
        (quality != (uint8)CAR_LAMP_EVIDENCE_NONE) ?
        s_tracker.position.x : IMAGE_DATA_INVALID_VALUE;
    s_tracker.snapshot.center_y =
        (quality != (uint8)CAR_LAMP_EVIDENCE_NONE) ?
        s_tracker.position.y : IMAGE_DATA_INVALID_VALUE;
    s_tracker.snapshot.velocity_x =
        (quality != (uint8)CAR_LAMP_EVIDENCE_NONE) ?
        s_tracker.velocity.x : 0.0f;
    s_tracker.snapshot.velocity_y =
        (quality != (uint8)CAR_LAMP_EVIDENCE_NONE) ?
        s_tracker.velocity.y : 0.0f;
    s_tracker.snapshot.state = s_tracker.diag.state;
    s_tracker.snapshot.support_camera_mask =
        (quality == (uint8)CAR_LAMP_EVIDENCE_STRONG) ?
        s_tracker.support_camera_mask : 0U;
    s_tracker.snapshot.quality = quality;
    s_tracker.snapshot.roi_mode = g_car_lamp_roi_mode;
}

static void car_lamp_cross_check_reset_diag(uint32 now_ms)
{
    uint8 camera;

    s_tracker.diag.anchor_time_ms = now_ms;
    s_tracker.diag.support_camera_mask = 0U;
    s_tracker.diag.roi_valid_mask = 0U;
    s_tracker.diag.roi_hit_mask = 0U;
    s_tracker.diag.conflict_camera_mask = 0U;
    s_tracker.diag.full_frame_fallback_mask = 0U;
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        s_tracker.diag.expected_x[camera] = IMAGE_DATA_INVALID_VALUE;
        s_tracker.diag.expected_y[camera] = IMAGE_DATA_INVALID_VALUE;
        s_tracker.diag.match_error[camera] =
            CAR_LAMP_CROSS_CHECK_INVALID_ERROR;
        s_tracker.diag.roi_half_size[camera] = 0.0f;
    }
}

static void car_lamp_cross_check_measure_prior_roi(
    const car_lamp_track_snapshot_t *prior_snapshot,
    const car_lamp_observation_t observations[IMAGE_CAMERA_COUNT])
{
    uint8 camera;

    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        const car_lamp_observation_t *observation = &observations[camera];
        car_lamp_roi_t roi;

        if((observation->frame_fresh == 0U) ||
           (observation->valid == 0U) ||
           (CarLampCrossCheck_GetRoiAt(
                prior_snapshot, (image_camera_e)camera,
                observation->time_ms, &roi) == 0U))
        {
            continue;
        }
        g_car_lamp_roi_sample_count[camera]++;
        if((observation->source.x >= roi.min_x) &&
           (observation->source.x <= roi.max_x) &&
           (observation->source.y >= roi.min_y) &&
           (observation->source.y <= roi.max_y))
        {
            s_tracker.diag.roi_hit_mask |= CAR_LAMP_CAMERA_BIT(camera);
            g_car_lamp_roi_hit_count[camera]++;
        }
    }
}

static void car_lamp_cross_check_update_diag_roi(
    const car_lamp_track_snapshot_t *snapshot,
    uint32 now_ms)
{
    uint8 camera;

    if(s_tracker.diag.projection_enabled == 0U)
    {
        return;
    }
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        car_lamp_roi_t roi;

        if(CarLampCrossCheck_GetRoiAt(
               snapshot, (image_camera_e)camera,
               now_ms, &roi) == 0U)
        {
            continue;
        }
        s_tracker.diag.expected_x[camera] = roi.expected_x;
        s_tracker.diag.expected_y[camera] = roi.expected_y;
        s_tracker.diag.roi_half_size[camera] = roi.half_size;
        s_tracker.diag.roi_valid_mask |= CAR_LAMP_CAMERA_BIT(camera);
    }
}

void CarLampCrossCheck_Init(image_camera_e local_camera)
{
    uint8 camera;

    memset(&s_tracker, 0, sizeof(s_tracker));
    memset(g_car_lamp_roi_sample_count, 0,
           sizeof(g_car_lamp_roi_sample_count));
    memset(g_car_lamp_roi_hit_count, 0,
           sizeof(g_car_lamp_roi_hit_count));
    g_car_lamp_source_handoff_count = 0U;
    memset(g_car_lamp_last_handoff_frame_sequence, 0,
           sizeof(g_car_lamp_last_handoff_frame_sequence));
    g_car_lamp_coast_max_ms = 0U;
    g_car_lamp_full_frame_fallback_count = 0U;
    g_car_lamp_stale_frame_reject_count = 0U;
    g_car_lamp_duplicate_frame_reject_count = 0U;
    g_car_lamp_invalid_time_reject_count = 0U;
    g_car_lamp_soft_pair_hint_count = 0U;
    g_car_lamp_roi_mode = 0U;
    s_tracker.local_camera = local_camera;
    s_tracker.initialized =
        (local_camera < IMAGE_CAMERA_COUNT) ? 1U : 0U;
    s_tracker.diag.state = (uint8)CAR_LAMP_TRACK_SEARCH;
    s_tracker.snapshot.center_x = IMAGE_DATA_INVALID_VALUE;
    s_tracker.snapshot.center_y = IMAGE_DATA_INVALID_VALUE;
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        s_tracker.diag.expected_x[camera] = IMAGE_DATA_INVALID_VALUE;
        s_tracker.diag.expected_y[camera] = IMAGE_DATA_INVALID_VALUE;
        s_tracker.diag.match_error[camera] =
            CAR_LAMP_CROSS_CHECK_INVALID_ERROR;
    }
}

uint8 CarLampCrossCheck_UpdateAt(const image_sync_set_t *frames,
                                uint32 now_ms,
                                float roll_deg,
                                float pitch_deg,
                                float height_mm,
                                uint8 attitude_valid,
                                uint8 height_valid)
{
    car_lamp_track_snapshot_t prior_snapshot;
    car_lamp_observation_t observations[IMAGE_CAMERA_COUNT];
    uint8 strong_measured_mask = 0U;
    uint8 fresh_mask;
    uint8 projection_enabled;

    if((frames == 0) || (s_tracker.initialized == 0U))
    {
        return 0U;
    }

    prior_snapshot = s_tracker.snapshot;
    car_lamp_cross_check_reset_diag(now_ms);
    projection_enabled = car_lamp_cross_check_projection_enabled(
        roll_deg, pitch_deg, height_mm, attitude_valid, height_valid);
    s_tracker.diag.projection_enabled = projection_enabled;
    fresh_mask = car_lamp_cross_check_build_observations(
        frames, now_ms, observations, &strong_measured_mask);
    car_lamp_cross_check_record_soft_pair_hint(
        observations, strong_measured_mask);
    car_lamp_cross_check_measure_prior_roi(&prior_snapshot, observations);

    car_lamp_cross_check_age_track(now_ms);
    if(projection_enabled != 0U)
    {
        if(s_tracker.active != 0U)
        {
            (void)car_lamp_cross_check_update_track(
                observations, strong_measured_mask);
        }
        else
        {
            (void)car_lamp_cross_check_acquire(
                observations,
                now_ms,
                car_lamp_cross_check_high_tilt(roll_deg, pitch_deg));
        }
    }
    car_lamp_cross_check_age_track(now_ms);

    if((s_tracker.has_solution != 0U) &&
       (car_lamp_cross_check_snapshot_quality(now_ms) ==
        (uint8)CAR_LAMP_EVIDENCE_NONE))
    {
        s_tracker.has_solution = 0U;
        s_tracker.velocity.x = 0.0f;
        s_tracker.velocity.y = 0.0f;
    }

    if(frames->meta[s_tracker.local_camera].frame_valid != 0U)
    {
        s_tracker.diag.anchor_sequence =
            frames->meta[s_tracker.local_camera].frame_sequence;
    }
    car_lamp_cross_check_publish_snapshot(now_ms);
    car_lamp_cross_check_update_diag_roi(&prior_snapshot, now_ms);
    s_tracker.diag.support_camera_mask =
        s_tracker.snapshot.support_camera_mask;
    s_tracker.diag.center_x = s_tracker.snapshot.center_x;
    s_tracker.diag.center_y = s_tracker.snapshot.center_y;
    s_tracker.diag.velocity_x = s_tracker.snapshot.velocity_x;
    s_tracker.diag.velocity_y = s_tracker.snapshot.velocity_y;
    s_tracker.diag.source_switch_count =
        g_car_lamp_source_handoff_count;
    s_tracker.diag.full_frame_fallback_count =
        g_car_lamp_full_frame_fallback_count;
    s_tracker.diag.roi_mode = g_car_lamp_roi_mode;
    return (fresh_mask != 0U) ? 1U : 0U;
}

uint8 CarLampCrossCheck_Update(const image_sync_set_t *frames,
                               float roll_deg,
                               float pitch_deg,
                               float height_mm,
                               uint8 attitude_valid,
                               uint8 height_valid)
{
    uint8 camera;
    uint8 found = 0U;
    uint32 now_ms = 0U;

    if(frames == 0)
    {
        return 0U;
    }
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        const image_frame_meta_t *meta = &frames->meta[camera];

        if((meta->frame_valid == 0U) ||
           (meta->timestamp_valid == 0U))
        {
            continue;
        }
        if((found == 0U) || ((int32)(meta->capture_time_ms - now_ms) > 0))
        {
            now_ms = meta->capture_time_ms;
            found = 1U;
        }
    }
    if(found == 0U)
    {
        return 0U;
    }
    return CarLampCrossCheck_UpdateAt(
        frames, now_ms, roll_deg, pitch_deg, height_mm,
        attitude_valid, height_valid);
}

void CarLampCrossCheck_GetSnapshot(car_lamp_track_snapshot_t *out)
{
    if(out != 0)
    {
        *out = s_tracker.snapshot;
    }
}

/**
 * @brief 使用最近真实灯条半长生成目标摄像头在指定时刻的ROI。
 * @param snapshot 核心1权威轨迹快照。
 * @param camera 目标摄像头。
 * @param capture_time_ms 目标图像统一采集时间，单位ms。
 * @param lamp_half_length_px 最近有效真实灯条半长，单位像素。
 * @param out 输出预计位置、裁剪边界和证据等级。
 * @return 1表示ROI与图像相交，0表示输入、时间或投影无效。
 */
uint8 CarLampCrossCheck_GetRoiAtWithLampHalfLength(
    const car_lamp_track_snapshot_t *snapshot,
    image_camera_e camera,
    uint32 capture_time_ms,
    float lamp_half_length_px,
    car_lamp_roi_t *out)
{
    car_lamp_projection_point_t center;
    car_lamp_projection_point_t snapshot_center;
    car_lamp_projection_point_t snapshot_velocity;
    car_lamp_projection_point_t source;
    uint8 evidence_level;
    uint32 age_ms;
    uint32 motion_ms;
    float base_half_size;
    float speed;
    float half_size;

    if(out != 0)
    {
        memset(out, 0, sizeof(*out));
    }
    if((snapshot == 0) || (out == 0) ||
       (camera >= IMAGE_CAMERA_COUNT) ||
       (snapshot->sequence == 0U) ||
       (snapshot->quality == (uint8)CAR_LAMP_EVIDENCE_NONE) ||
       (snapshot->quality > (uint8)CAR_LAMP_EVIDENCE_STRONG) ||
       (snapshot->center_x == IMAGE_DATA_INVALID_VALUE) ||
       (snapshot->center_y == IMAGE_DATA_INVALID_VALUE) ||
       (snapshot->center_x != snapshot->center_x) ||
       (snapshot->center_y != snapshot->center_y))
    {
        return 0U;
    }

    evidence_level = car_lamp_cross_check_evidence(
        capture_time_ms, snapshot->last_support_time_ms);
    if(evidence_level == (uint8)CAR_LAMP_EVIDENCE_NONE)
    {
        return 0U;
    }
    snapshot_center.x = snapshot->center_x;
    snapshot_center.y = snapshot->center_y;
    snapshot_velocity.x = snapshot->velocity_x;
    snapshot_velocity.y = snapshot->velocity_y;
    car_lamp_cross_check_predict_from(
        &snapshot_center, &snapshot_velocity,
        snapshot->reference_time_ms, capture_time_ms, &center);
    source.x = IMAGE_DATA_INVALID_VALUE;
    source.y = IMAGE_DATA_INVALID_VALUE;
    (void)CarLampProjection_FromCenter(camera, &center, &source);
    if((source.x == IMAGE_DATA_INVALID_VALUE) ||
       (source.y == IMAGE_DATA_INVALID_VALUE) ||
       (source.x != source.x) || (source.y != source.y))
    {
        return 0U;
    }

    base_half_size = (camera == Back) ? CAR_LAMP_TRACK_ROI_BACK_PX :
                     ((camera == Front) ? CAR_LAMP_TRACK_ROI_FRONT_PX :
                                          CAR_LAMP_TRACK_ROI_CENTER_PX);
    if((lamp_half_length_px <= 0.0f) ||
       (lamp_half_length_px != lamp_half_length_px))
    {
        lamp_half_length_px = CAR_LAMP_TRACK_ROI_DEFAULT_HALF_PX;
    }
    motion_ms = image_frame_time_difference_ms(
        capture_time_ms, snapshot->reference_time_ms);
    speed = sqrtf(snapshot->velocity_x * snapshot->velocity_x +
                  snapshot->velocity_y * snapshot->velocity_y);
    half_size = base_half_size + lamp_half_length_px +
                CAR_LAMP_TRACK_ROI_MARGIN_PX +
                speed * (float)motion_ms * 0.001f;
    if((evidence_level == (uint8)CAR_LAMP_EVIDENCE_LOCATE) &&
       (half_size < CAR_LAMP_TRACK_ROI_LOCATE_MIN_PX))
    {
        half_size = CAR_LAMP_TRACK_ROI_LOCATE_MIN_PX;
    }
    if(half_size > CAR_LAMP_TRACK_ROI_MAX_PX)
    {
        half_size = CAR_LAMP_TRACK_ROI_MAX_PX;
    }

    if((source.x + half_size < -CAR_LAMP_IMAGE_HALF_WIDTH) ||
       (source.x - half_size > CAR_LAMP_IMAGE_HALF_WIDTH) ||
       (source.y + half_size < -CAR_LAMP_IMAGE_HALF_HEIGHT) ||
       (source.y - half_size > CAR_LAMP_IMAGE_HALF_HEIGHT))
    {
        return 0U;
    }

    age_ms = image_frame_time_difference_ms(
        capture_time_ms, snapshot->last_support_time_ms);
    out->snapshot_sequence = snapshot->sequence;
    out->expected_x = source.x;
    out->expected_y = source.y;
    out->expected_center_x = center.x;
    out->expected_center_y = center.y;
    out->half_size = half_size;
    out->min_x = source.x - half_size;
    out->max_x = source.x + half_size;
    out->min_y = source.y - half_size;
    out->max_y = source.y + half_size;
    if(out->min_x < -CAR_LAMP_IMAGE_HALF_WIDTH)
    {
        out->min_x = -CAR_LAMP_IMAGE_HALF_WIDTH;
    }
    if(out->max_x > CAR_LAMP_IMAGE_HALF_WIDTH)
    {
        out->max_x = CAR_LAMP_IMAGE_HALF_WIDTH;
    }
    if(out->min_y < -CAR_LAMP_IMAGE_HALF_HEIGHT)
    {
        out->min_y = -CAR_LAMP_IMAGE_HALF_HEIGHT;
    }
    if(out->max_y > CAR_LAMP_IMAGE_HALF_HEIGHT)
    {
        out->max_y = CAR_LAMP_IMAGE_HALF_HEIGHT;
    }
    out->valid = (age_ms <= CAR_LAMP_TRACK_LOCATE_MAX_AGE_MS) ? 1U : 0U;
    out->evidence_level = evidence_level;
    out->roi_mode = snapshot->roi_mode;
    return out->valid;
}

/**
 * @brief 使用默认灯条半长生成目标摄像头在指定时刻的ROI。
 * @param snapshot 核心1权威轨迹快照。
 * @param camera 目标摄像头。
 * @param capture_time_ms 目标图像统一采集时间，单位ms。
 * @param out 输出预计位置、裁剪边界和证据等级。
 * @return 1表示ROI与图像相交，0表示输入、时间或投影无效。
 */
uint8 CarLampCrossCheck_GetRoiAt(
    const car_lamp_track_snapshot_t *snapshot,
    image_camera_e camera,
    uint32 capture_time_ms,
    car_lamp_roi_t *out)
{
    return CarLampCrossCheck_GetRoiAtWithLampHalfLength(
        snapshot, camera, capture_time_ms,
        CAR_LAMP_TRACK_ROI_DEFAULT_HALF_PX, out);
}

/**
 * @brief 将本摄候选映射到公共坐标并与ROI预计中心闭环比较。
 * @param roi 当前帧权威ROI。
 * @param camera 候选所属摄像头。
 * @param source 候选本摄中心坐标，单位像素。
 * @param gate_px 公共坐标匹配门限，单位像素。
 * @return 1表示候选通过闭环门限，0表示输入、投影或距离无效。
 */
uint8 CarLampCrossCheck_CandidateMatchesRoi(
    const car_lamp_roi_t *roi,
    image_camera_e camera,
    const car_lamp_projection_point_t *source,
    float gate_px)
{
    car_lamp_projection_point_t center;
    float dx;
    float dy;

    if((roi == 0) || (roi->valid == 0U) ||
       (camera >= IMAGE_CAMERA_COUNT) || (source == 0) ||
       (gate_px <= 0.0f) ||
       (roi->expected_center_x != roi->expected_center_x) ||
       (roi->expected_center_y != roi->expected_center_y) ||
       (CarLampProjection_ToCenter(camera, source, &center) == 0U))
    {
        return 0U;
    }
    dx = center.x - roi->expected_center_x;
    dy = center.y - roi->expected_center_y;
    return ((dx * dx + dy * dy) <= gate_px * gate_px) ? 1U : 0U;
}

void CarLampCrossCheck_SetRoiMode(uint8 enabled)
{
    g_car_lamp_roi_mode = (enabled != 0U) ? 1U : 0U;
    s_tracker.snapshot.roi_mode = g_car_lamp_roi_mode;
    s_tracker.diag.roi_mode = g_car_lamp_roi_mode;
}

void CarLampCrossCheck_ApplyRuntimeDiag(uint8 roi_hit_mask,
                                        uint8 fallback_mask,
                                        uint8 conflict_mask,
                                        uint8 sample_mask)
{
    uint8 counted_fallback =
        (uint8)(fallback_mask & sample_mask & 0x07U);
    uint8 camera;

    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        if((counted_fallback & CAR_LAMP_CAMERA_BIT(camera)) != 0U)
        {
            g_car_lamp_full_frame_fallback_count++;
        }
    }
    s_tracker.diag.roi_hit_mask |= (uint8)(roi_hit_mask & 0x07U);
    s_tracker.diag.full_frame_fallback_mask =
        (uint8)(fallback_mask & 0x07U);
    s_tracker.diag.conflict_camera_mask |=
        (uint8)(conflict_mask & 0x07U);
    s_tracker.diag.full_frame_fallback_count =
        g_car_lamp_full_frame_fallback_count;
}

const car_lamp_cross_check_diag_t *CarLampCrossCheck_GetDiag(void)
{
    return &s_tracker.diag;
}
