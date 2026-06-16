#include "mode2_three_camera.h"

#include <math.h>
#include <string.h>

#include "camera_crop_boundary_mapping.h"

#define MODE2_THREE_CAMERA_IMAGE_CENTER_X          (94.0f)
#define MODE2_THREE_CAMERA_IMAGE_CENTER_Y          (60.0f)
#define MODE2_THREE_CAMERA_MIN_TARGET_AREA         (1.0f)
#define TRACK_CENTER_START_AREA_BONUS         (5.0f)
#define TRACK_MATCH_RADIUS                    (15.0f)
#define TRACK_LOST_FRAME_LIMIT                (5U)
#define TRACK_BOUNDARY_SWITCH_Y_EPS           (5.0f)
#define TRACK_CENTER_SEARCH_RADIUS            (20.0f)
#define MODE2_THREE_CAMERA_CAR_LAMP_OFFSET_PX      (13.0f)
#define MODE2_THREE_CAMERA_CAR_LAMP_HOLD_FRAME_LIMIT (5U)
#define MODE2_THREE_CAMERA_DEG_TO_RAD              (0.017453292519943295f)
#define MODE2_THREE_CAMERA_LAMP_LEVEL_ZERO_DEG     (0.0f)
#define MODE2_THREE_CAMERA_AIR_TO_CAR_ANGLE_SIGN   (1.0f)

typedef struct
{
    int camera_index;
    uint8 target_index;
    float image_x;
    float image_y;
    float area;
    uint8 valid;
} mode2_three_camera_candidate_t;

typedef struct
{
    int camera_index;
    uint16 frame_id;
    float image_x;
    float image_y;
    float area;
    uint8 valid;
} mode2_three_camera_point_t;

mode2_three_camera_state_t g_mode2_three_camera;

static mode2_three_camera_point_t s_last_point;
static uint8 s_missing_frame_count;
static uint8 s_auto_max_count = MODE2_THREE_CAMERA_CAMERA_TARGETS;
static uint16 s_frame_id;
static float s_car_lamp_ref_x;
static float s_car_lamp_ref_y;
static float s_car_lamp_cx;
static float s_car_lamp_cy;
static uint8 s_car_lamp_ref_valid;
static float s_car_lamp_angle_deg;
static uint8 s_car_lamp_angle_valid;
static uint8 s_car_lamp_invalid_frame_count;

static float mode2_three_camera_square(float value)
{
    return value * value;
}

static float mode2_three_camera_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float mode2_three_camera_normalize_signed_lamp_angle_deg(float angle_deg)
{
    while(angle_deg >= 90.0f)
    {
        angle_deg -= 180.0f;
    }

    while(angle_deg < -90.0f)
    {
        angle_deg += 180.0f;
    }

    return angle_deg;
}

static float mode2_three_camera_angle_diff_deg(float target_deg, float reference_deg)
{
    return mode2_three_camera_normalize_signed_lamp_angle_deg(target_deg - reference_deg);
}

static void mode2_three_camera_rotate_air_to_car(float air_x,
                                            float air_y,
                                            float angle_deg,
                                            float *car_x,
                                            float *car_y)
{
    const float angle_rad = angle_deg * MODE2_THREE_CAMERA_DEG_TO_RAD;
    const float cos_angle = cosf(angle_rad);
    const float sin_angle = sinf(angle_rad);

    if((car_x == NULL) || (car_y == NULL))
    {
        return;
    }

    *car_x = (cos_angle * air_x) + (sin_angle * air_y);
    *car_y = (-sin_angle * air_x) + (cos_angle * air_y);
}

static void mode2_three_camera_lamp_ref_from_center(float cx,
                                               float cy,
                                               float angle_deg,
                                               float *ref_x,
                                               float *ref_y)
{
    const float angle_rad = angle_deg * MODE2_THREE_CAMERA_DEG_TO_RAD;
    const float normal_x = -sinf(angle_rad);
    const float normal_y = cosf(angle_rad);

    if((ref_x == NULL) || (ref_y == NULL))
    {
        return;
    }

    *ref_x = cx + (MODE2_THREE_CAMERA_CAR_LAMP_OFFSET_PX * normal_x);
    *ref_y = cy + (MODE2_THREE_CAMERA_CAR_LAMP_OFFSET_PX * normal_y);
}

static float mode2_three_camera_transform_angle_deg(void)
{
    if(s_car_lamp_angle_valid == 0U)
    {
        return 0.0f;
    }

    return MODE2_THREE_CAMERA_AIR_TO_CAR_ANGLE_SIGN *
           mode2_three_camera_angle_diff_deg(s_car_lamp_angle_deg,
                                        MODE2_THREE_CAMERA_LAMP_LEVEL_ZERO_DEG);
}

static float mode2_three_camera_distance2(float lhs_x, float lhs_y, float rhs_x, float rhs_y)
{
    const float dx = lhs_x - rhs_x;
    const float dy = lhs_y - rhs_y;

    return (dx * dx) + (dy * dy);
}

static float mode2_three_camera_candidate_score(const mode2_three_camera_candidate_t *candidate)
{
    return candidate->area;
}

static float mode2_three_camera_start_candidate_score(const mode2_three_camera_candidate_t *candidate)
{
    float score;

    score = mode2_three_camera_candidate_score(candidate);
    if(candidate->camera_index == (int)MODE2_THREE_CAMERA_CAMERA_CENTER)
    {
        score += TRACK_CENTER_START_AREA_BONUS;
    }

    return score;
}

static uint8 mode2_three_camera_target_limit(void)
{
    if(s_auto_max_count > MODE2_THREE_CAMERA_CAMERA_TARGETS)
    {
        return MODE2_THREE_CAMERA_CAMERA_TARGETS;
    }

    return s_auto_max_count;
}

static void mode2_three_camera_clear_point(mode2_three_camera_point_t *point)
{
    point->camera_index = -1;
    point->frame_id = 0U;
    point->image_x = 0.0f;
    point->image_y = 0.0f;
    point->area = 0.0f;
    point->valid = 0U;
}

static void mode2_three_camera_clear_track(void)
{
    mode2_three_camera_clear_point(&s_last_point);
    s_missing_frame_count = 0U;
}

static void mode2_three_camera_reset_state(void)
{
    memset(&g_mode2_three_camera, 0, sizeof(g_mode2_three_camera));
    mode2_three_camera_clear_track();
    s_frame_id = 0U;
    s_car_lamp_ref_x = MODE2_THREE_CAMERA_IMAGE_CENTER_X;
    s_car_lamp_ref_y = MODE2_THREE_CAMERA_IMAGE_CENTER_Y;
    s_car_lamp_cx = 0.0f;
    s_car_lamp_cy = 0.0f;
    s_car_lamp_ref_valid = 0U;
    s_car_lamp_angle_deg = 0.0f;
    s_car_lamp_angle_valid = 0U;
    s_car_lamp_invalid_frame_count = 0U;
}

static mode2_three_camera_candidate_t mode2_three_camera_candidate_from_target(const mode2_three_camera_target_t *target,
                                                                     uint8 camera_index,
                                                                     uint8 target_index)
{
    mode2_three_camera_candidate_t candidate;

    candidate.camera_index = (int)camera_index;
    candidate.target_index = target_index;
    candidate.image_x = target->x;
    candidate.image_y = target->y;
    candidate.area = target->area;
    candidate.valid = target->valid;

    if((candidate.valid == 0U) ||
       (mode2_three_camera_candidate_score(&candidate) < MODE2_THREE_CAMERA_MIN_TARGET_AREA))
    {
        candidate.valid = 0U;
    }

    return candidate;
}

static void mode2_three_camera_point_from_candidate(const mode2_three_camera_candidate_t *candidate,
                                               uint16 frame_id,
                                               mode2_three_camera_point_t *point)
{
    if((candidate == NULL) || (point == NULL) || (candidate->valid == 0U))
    {
        return;
    }

    point->camera_index = candidate->camera_index;
    point->frame_id = frame_id;
    point->image_x = candidate->image_x;
    point->image_y = candidate->image_y;
    point->area = candidate->area;
    point->valid = 1U;
}

static mode2_three_camera_point_t mode2_three_camera_last_point(void)
{
    return s_last_point;
}

static void mode2_three_camera_store_last_point(const mode2_three_camera_point_t *point)
{
    if((point == NULL) || (point->valid == 0U))
    {
        return;
    }

    s_last_point = *point;
}

static uint8 mode2_three_camera_calc_center_delta(const mode2_three_camera_point_t *point,
                                             float *delta_x,
                                             float *delta_y)
{
    float center_x;
    float center_y;

    if((point == NULL) || (delta_x == NULL) || (delta_y == NULL) || (point->valid == 0U))
    {
        return 0U;
    }

    if(point->camera_index == (int)MODE2_THREE_CAMERA_CAMERA_FRONT)
    {
        center_x = camera_front_boundary_center_x(point->image_x);
        center_y = camera_front_boundary_center_y(point->image_x) +
                   (point->image_y - CAMERA_CROP_BOUNDARY_BOTTOM_ROW_Y);
    }
    else if(point->camera_index == (int)MODE2_THREE_CAMERA_CAMERA_CENTER)
    {
        center_x = point->image_x;
        center_y = point->image_y;
    }
    else if(point->camera_index == (int)MODE2_THREE_CAMERA_CAMERA_REAR)
    {
        center_x = camera_rear_boundary_center_x(point->image_x);
        center_y = camera_rear_boundary_center_y(point->image_x) +
                   (CAMERA_CROP_BOUNDARY_BOTTOM_ROW_Y - point->image_y);
    }
    else
    {
        return 0U;
    }

    *delta_x = center_x - s_car_lamp_ref_x;
    *delta_y = center_y - s_car_lamp_ref_y;
    return 1U;
}

static void mode2_three_camera_publish_point(const mode2_three_camera_point_t *point)
{
    float center_delta_x;
    float center_delta_y;

    if((point == NULL) || (point->valid == 0U))
    {
        g_mode2_three_camera.active = 0U;
        g_mode2_three_camera.valid = 0U;
        g_mode2_three_camera.camera_id = 0U;
        g_mode2_three_camera.frame_id = s_frame_id;
        g_mode2_three_camera.image_x = 0.0f;
        g_mode2_three_camera.image_y = 0.0f;
        g_mode2_three_camera.center_delta_valid = 0U;
        g_mode2_three_camera.center_delta_x = 0.0f;
        g_mode2_three_camera.center_delta_y = 0.0f;
        g_mode2_three_camera.raw_center_delta_x = 0.0f;
        g_mode2_three_camera.raw_center_delta_y = 0.0f;
        g_mode2_three_camera.lamp_angle_valid = s_car_lamp_angle_valid;
        g_mode2_three_camera.lamp_angle_deg = s_car_lamp_angle_deg;
        g_mode2_three_camera.car_lamp_cx = s_car_lamp_cx;
        g_mode2_three_camera.car_lamp_cy = s_car_lamp_cy;
        g_mode2_three_camera.transform_angle_deg = mode2_three_camera_transform_angle_deg();
        g_mode2_three_camera.area = 0.0f;
        g_mode2_three_camera.missing_frame_count = s_missing_frame_count;
        return;
    }

    g_mode2_three_camera.active = 1U;
    g_mode2_three_camera.valid = 1U;
    g_mode2_three_camera.camera_id = (uint8)point->camera_index;
    g_mode2_three_camera.frame_id = point->frame_id;
    g_mode2_three_camera.image_x = point->image_x;
    g_mode2_three_camera.image_y = point->image_y;
    center_delta_x = 0.0f;
    center_delta_y = 0.0f;
    g_mode2_three_camera.center_delta_valid =
        mode2_three_camera_calc_center_delta(point, &center_delta_x, &center_delta_y);
    g_mode2_three_camera.raw_center_delta_x = center_delta_x;
    g_mode2_three_camera.raw_center_delta_y = center_delta_y;
    g_mode2_three_camera.lamp_angle_valid = s_car_lamp_angle_valid;
    g_mode2_three_camera.lamp_angle_deg = s_car_lamp_angle_deg;
    g_mode2_three_camera.car_lamp_cx = s_car_lamp_cx;
    g_mode2_three_camera.car_lamp_cy = s_car_lamp_cy;
    g_mode2_three_camera.transform_angle_deg = mode2_three_camera_transform_angle_deg();
    if(g_mode2_three_camera.center_delta_valid != 0U)
    {
        mode2_three_camera_rotate_air_to_car(center_delta_x,
                                        center_delta_y,
                                        g_mode2_three_camera.transform_angle_deg,
                                        &g_mode2_three_camera.center_delta_x,
                                        &g_mode2_three_camera.center_delta_y);
    }
    else
    {
        g_mode2_three_camera.center_delta_x = 0.0f;
        g_mode2_three_camera.center_delta_y = 0.0f;
    }
    g_mode2_three_camera.area = point->area;
    g_mode2_three_camera.missing_frame_count = s_missing_frame_count;
}

static uint8 mode2_three_camera_find_largest_candidate(const mode2_three_camera_frame_t camera[MODE2_THREE_CAMERA_CAMERA_COUNT],
                                                  mode2_three_camera_candidate_t *best_candidate)
{
    uint8 camera_index;
    uint8 target_index;
    uint8 target_limit;
    float best_score;

    best_score = 0.0f;
    target_limit = mode2_three_camera_target_limit();
    best_candidate->valid = 0U;

    for(camera_index = 0U; camera_index < MODE2_THREE_CAMERA_CAMERA_COUNT; camera_index++)
    {
        for(target_index = 0U; target_index < target_limit; target_index++)
        {
            const mode2_three_camera_candidate_t candidate =
                mode2_three_camera_candidate_from_target(&camera[camera_index].target[target_index],
                                                    camera_index,
                                                    target_index);
            const float score = mode2_three_camera_start_candidate_score(&candidate);

            if(candidate.valid == 0U)
            {
                continue;
            }

            if((best_candidate->valid == 0U) || (score > best_score))
            {
                *best_candidate = candidate;
                best_score = score;
            }
        }
    }

    return best_candidate->valid;
}

static uint8 mode2_three_camera_find_nearest_candidate(const mode2_three_camera_frame_t camera[MODE2_THREE_CAMERA_CAMERA_COUNT],
                                                  uint8 camera_index,
                                                  float search_x,
                                                  float search_y,
                                                  float search_radius,
                                                  mode2_three_camera_candidate_t *best_candidate)
{
    uint8 target_index;
    uint8 target_limit;
    float best_distance;
    const float max_distance2 = mode2_three_camera_square(search_radius);

    if((camera_index >= MODE2_THREE_CAMERA_CAMERA_COUNT) || (best_candidate == NULL))
    {
        return 0U;
    }

    target_limit = mode2_three_camera_target_limit();
    best_distance = 0.0f;
    best_candidate->valid = 0U;

    for(target_index = 0U; target_index < target_limit; target_index++)
    {
        const mode2_three_camera_candidate_t candidate =
            mode2_three_camera_candidate_from_target(&camera[camera_index].target[target_index],
                                                camera_index,
                                                target_index);
        const float distance2 = mode2_three_camera_distance2(candidate.image_x,
                                                        candidate.image_y,
                                                        search_x,
                                                        search_y);

        if(candidate.valid == 0U)
        {
            continue;
        }
        if(distance2 > max_distance2)
        {
            continue;
        }
        if((best_candidate->valid == 0U) || (distance2 < best_distance))
        {
            *best_candidate = candidate;
            best_distance = distance2;
        }
    }

    return best_candidate->valid;
}

static uint8 mode2_three_camera_start_from_largest(const mode2_three_camera_frame_t camera[MODE2_THREE_CAMERA_CAMERA_COUNT],
                                              uint16 frame_id,
                                              mode2_three_camera_point_t *next_point)
{
    mode2_three_camera_candidate_t start_candidate;

    mode2_three_camera_clear_point(next_point);
    if(mode2_three_camera_find_largest_candidate(camera, &start_candidate) == 0U)
    {
        return 0U;
    }

    mode2_three_camera_clear_track();
    mode2_three_camera_point_from_candidate(&start_candidate, frame_id, next_point);
    mode2_three_camera_store_last_point(next_point);
    return 1U;
}

static uint8 mode2_three_camera_update_current_camera(const mode2_three_camera_frame_t camera[MODE2_THREE_CAMERA_CAMERA_COUNT],
                                                 uint16 frame_id,
                                                 mode2_three_camera_point_t *next_point)
{
    uint8 current_camera;
    mode2_three_camera_candidate_t best_candidate;
    const mode2_three_camera_point_t last_point = mode2_three_camera_last_point();

    mode2_three_camera_clear_point(next_point);
    if((last_point.valid == 0U) ||
       (last_point.camera_index < 0) ||
       (last_point.camera_index >= (int)MODE2_THREE_CAMERA_CAMERA_COUNT))
    {
        return 0U;
    }

    current_camera = (uint8)last_point.camera_index;
    if(mode2_three_camera_find_nearest_candidate(camera,
                                            current_camera,
                                            last_point.image_x,
                                            last_point.image_y,
                                            TRACK_MATCH_RADIUS,
                                            &best_candidate) == 0U)
    {
        return 0U;
    }

    mode2_three_camera_point_from_candidate(&best_candidate, frame_id, next_point);
    return 1U;
}

static uint8 mode2_three_camera_calc_boundary_center_point(const mode2_three_camera_point_t *point,
                                                      float *center_x,
                                                      float *center_y)
{
    float boundary_y;

    if((point == NULL) || (center_x == NULL) || (center_y == NULL) || (point->valid == 0U))
    {
        return 0U;
    }

    if(point->camera_index == (int)MODE2_THREE_CAMERA_CAMERA_FRONT)
    {
        camera_front_boundary_point_to_center(point->image_x, &boundary_y, center_x, center_y);
    }
    else if(point->camera_index == (int)MODE2_THREE_CAMERA_CAMERA_REAR)
    {
        camera_rear_boundary_point_to_center(point->image_x, &boundary_y, center_x, center_y);
    }
    else
    {
        return 0U;
    }

    if(mode2_three_camera_abs(point->image_y - boundary_y) >= TRACK_BOUNDARY_SWITCH_Y_EPS)
    {
        return 0U;
    }

    return 1U;
}

static uint8 mode2_three_camera_try_switch_to_center(const mode2_three_camera_frame_t camera[MODE2_THREE_CAMERA_CAMERA_COUNT],
                                                const mode2_three_camera_point_t *source_point,
                                                uint16 frame_id,
                                                mode2_three_camera_point_t *next_point)
{
    float center_x;
    float center_y;
    mode2_three_camera_candidate_t best_candidate;

    mode2_three_camera_clear_point(next_point);
    if(mode2_three_camera_calc_boundary_center_point(source_point, &center_x, &center_y) == 0U)
    {
        return 0U;
    }

    if(mode2_three_camera_find_nearest_candidate(camera,
                                            MODE2_THREE_CAMERA_CAMERA_CENTER,
                                            center_x,
                                            center_y,
                                            TRACK_CENTER_SEARCH_RADIUS,
                                            &best_candidate) == 0U)
    {
        return 0U;
    }

    mode2_three_camera_point_from_candidate(&best_candidate, frame_id, next_point);
    return 1U;
}

void mode2_three_camera_init(void)
{
    mode2_three_camera_reset_state();
}

void mode2_three_camera_set_auto_max_count(uint8 max_count)
{
    s_auto_max_count = (max_count > MODE2_THREE_CAMERA_CAMERA_TARGETS) ?
                       MODE2_THREE_CAMERA_CAMERA_TARGETS :
                       max_count;
}

void mode2_three_camera_set_center_car_lamp(uint8 valid, float cx, float cy, float angle_deg)
{
    float normalized_angle;

    if(valid == 0U)
    {
        if(s_car_lamp_invalid_frame_count <= MODE2_THREE_CAMERA_CAR_LAMP_HOLD_FRAME_LIMIT)
        {
            s_car_lamp_invalid_frame_count++;
        }

        if((s_car_lamp_ref_valid != 0U) &&
           (s_car_lamp_invalid_frame_count <= MODE2_THREE_CAMERA_CAR_LAMP_HOLD_FRAME_LIMIT))
        {
            return;
        }

        s_car_lamp_ref_valid = 0U;
        s_car_lamp_cx = 0.0f;
        s_car_lamp_cy = 0.0f;
        s_car_lamp_angle_valid = 0U;
        s_car_lamp_angle_deg = 0.0f;
        g_mode2_three_camera.car_lamp_cx = 0.0f;
        g_mode2_three_camera.car_lamp_cy = 0.0f;
        g_mode2_three_camera.lamp_angle_valid = 0U;
        g_mode2_three_camera.lamp_angle_deg = 0.0f;
        g_mode2_three_camera.transform_angle_deg = 0.0f;
        return;
    }

    s_car_lamp_invalid_frame_count = 0U;

    normalized_angle = mode2_three_camera_normalize_signed_lamp_angle_deg(angle_deg);
    s_car_lamp_angle_deg = normalized_angle;
    s_car_lamp_cx = cx;
    s_car_lamp_cy = cy;
    mode2_three_camera_lamp_ref_from_center(cx,
                                       cy,
                                       s_car_lamp_angle_deg,
                                       &s_car_lamp_ref_x,
                                       &s_car_lamp_ref_y);
    s_car_lamp_ref_valid = 1U;
    s_car_lamp_angle_valid = 1U;
    g_mode2_three_camera.car_lamp_cx = s_car_lamp_cx;
    g_mode2_three_camera.car_lamp_cy = s_car_lamp_cy;
}

uint8 mode2_three_camera_get_center_car_lamp_ref(float *ref_x, float *ref_y)
{
    if((ref_x == NULL) || (ref_y == NULL) || (s_car_lamp_ref_valid == 0U))
    {
        return 0U;
    }

    *ref_x = s_car_lamp_ref_x;
    *ref_y = s_car_lamp_ref_y;
    return 1U;
}

uint8 mode2_three_camera_update_100HZ(const mode2_three_camera_frame_t camera[MODE2_THREE_CAMERA_CAMERA_COUNT])
{
    mode2_three_camera_point_t next_point;

    s_frame_id++;
    if(camera == NULL)
    {
        mode2_three_camera_reset_state();
        return 0U;
    }

    if(g_mode2_three_camera.active == 0U)
    {
        if(mode2_three_camera_start_from_largest(camera, s_frame_id, &next_point) == 0U)
        {
            mode2_three_camera_publish_point(NULL);
            return 0U;
        }

        s_missing_frame_count = 0U;
        mode2_three_camera_publish_point(&next_point);
        return 1U;
    }

    if(mode2_three_camera_update_current_camera(camera, s_frame_id, &next_point) != 0U)
    {
        mode2_three_camera_point_t center_point;

        if(mode2_three_camera_try_switch_to_center(camera, &next_point, s_frame_id, &center_point) != 0U)
        {
            next_point = center_point;
        }

        mode2_three_camera_store_last_point(&next_point);
        s_missing_frame_count = 0U;
        mode2_three_camera_publish_point(&next_point);
        return 1U;
    }

    if(s_last_point.valid == 0U)
    {
        mode2_three_camera_publish_point(NULL);
        mode2_three_camera_clear_track();
        return 0U;
    }

    s_missing_frame_count++;
    if(s_missing_frame_count <= TRACK_LOST_FRAME_LIMIT)
    {
        next_point = mode2_three_camera_last_point();
        mode2_three_camera_publish_point(&next_point);
        return 1U;
    }

    mode2_three_camera_publish_point(NULL);
    mode2_three_camera_clear_track();
    return 0U;
}
