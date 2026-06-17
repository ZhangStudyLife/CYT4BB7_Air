#include "image_fusion.h"

#include <math.h>
#include <string.h>

#include "camera_crop_boundary_mapping.h"

#define IMAGE_FUSION_IMAGE_CENTER_X          (94.0f)
#define IMAGE_FUSION_IMAGE_CENTER_Y          (60.0f)
#define IMAGE_FUSION_MIN_TARGET_AREA         (1.0f)
#define TRACK_CENTER_START_AREA_BONUS         (5.0f)
#define TRACK_MATCH_RADIUS                    (15.0f)
#define TRACK_LOST_FRAME_LIMIT                (5U)
#define TRACK_BOUNDARY_SWITCH_Y_EPS           (5.0f)
#define TRACK_CENTER_SEARCH_RADIUS            (20.0f)
#define IMAGE_FUSION_CAR_LAMP_OFFSET_PX      (13.0f)
#define IMAGE_FUSION_CAR_LAMP_HOLD_FRAME_LIMIT (5U)
#define IMAGE_FUSION_DEG_TO_RAD              (0.017453292519943295f)
#define IMAGE_FUSION_LAMP_LEVEL_ZERO_DEG     (0.0f)
#define IMAGE_FUSION_AIR_TO_CAR_ANGLE_SIGN   (1.0f)

typedef struct
{
    int camera_index;
    uint8 target_index;
    float image_x;
    float image_y;
    float area;
    uint8 valid;
} image_fusion_candidate_t;

typedef struct
{
    int camera_index;
    uint16 frame_id;
    float image_x;
    float image_y;
    float area;
    uint8 valid;
} image_fusion_point_t;

image_fusion_state_t g_image_fusion;

static image_fusion_point_t s_last_point;
static uint8 s_missing_frame_count;
static uint16 s_frame_id;
static float s_car_lamp_ref_x;
static float s_car_lamp_ref_y;
static float s_car_lamp_cx;
static float s_car_lamp_cy;
static uint8 s_car_lamp_ref_valid;
static float s_car_lamp_angle_deg;
static uint8 s_car_lamp_angle_valid;
static uint8 s_car_lamp_invalid_frame_count;

static float image_fusion_square(float value)
{
    return value * value;
}

static float image_fusion_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float image_fusion_normalize_signed_lamp_angle_deg(float angle_deg)
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

static float image_fusion_angle_diff_deg(float target_deg, float reference_deg)
{
    return image_fusion_normalize_signed_lamp_angle_deg(target_deg - reference_deg);
}

static void image_fusion_rotate_air_to_car(float air_x,
                                            float air_y,
                                            float angle_deg,
                                            float *car_x,
                                            float *car_y)
{
    const float angle_rad = angle_deg * IMAGE_FUSION_DEG_TO_RAD;
    const float cos_angle = cosf(angle_rad);
    const float sin_angle = sinf(angle_rad);

    if((car_x == NULL) || (car_y == NULL))
    {
        return;
    }

    *car_x = (cos_angle * air_x) + (sin_angle * air_y);
    *car_y = (-sin_angle * air_x) + (cos_angle * air_y);
}

static void image_fusion_lamp_ref_from_center(float cx,
                                               float cy,
                                               float angle_deg,
                                               float *ref_x,
                                               float *ref_y)
{
    const float angle_rad = angle_deg * IMAGE_FUSION_DEG_TO_RAD;
    const float normal_x = -sinf(angle_rad);
    const float normal_y = cosf(angle_rad);

    if((ref_x == NULL) || (ref_y == NULL))
    {
        return;
    }

    *ref_x = cx + (IMAGE_FUSION_CAR_LAMP_OFFSET_PX * normal_x);
    *ref_y = cy + (IMAGE_FUSION_CAR_LAMP_OFFSET_PX * normal_y);
}

static float image_fusion_transform_angle_deg(void)
{
    if(s_car_lamp_angle_valid == 0U)
    {
        return 0.0f;
    }

    return IMAGE_FUSION_AIR_TO_CAR_ANGLE_SIGN *
           image_fusion_angle_diff_deg(s_car_lamp_angle_deg,
                                        IMAGE_FUSION_LAMP_LEVEL_ZERO_DEG);
}

static float image_fusion_distance2(float lhs_x, float lhs_y, float rhs_x, float rhs_y)
{
    const float dx = lhs_x - rhs_x;
    const float dy = lhs_y - rhs_y;

    return (dx * dx) + (dy * dy);
}

static float image_fusion_candidate_score(const image_fusion_candidate_t *candidate)
{
    return candidate->area;
}

static float image_fusion_start_candidate_score(const image_fusion_candidate_t *candidate)
{
    float score;

    score = image_fusion_candidate_score(candidate);
    if(candidate->camera_index == (int)Center)
    {
        score += TRACK_CENTER_START_AREA_BONUS;
    }

    return score;
}

static void image_fusion_clear_point(image_fusion_point_t *point)
{
    point->camera_index = -1;
    point->frame_id = 0U;
    point->image_x = 0.0f;
    point->image_y = 0.0f;
    point->area = 0.0f;
    point->valid = 0U;
}

static void image_fusion_clear_track(void)
{
    image_fusion_clear_point(&s_last_point);
    s_missing_frame_count = 0U;
}

static void image_fusion_reset_state(void)
{
    memset(&g_image_fusion, 0, sizeof(g_image_fusion));
    image_fusion_clear_track();
    s_frame_id = 0U;
    s_car_lamp_ref_x = IMAGE_FUSION_IMAGE_CENTER_X;
    s_car_lamp_ref_y = IMAGE_FUSION_IMAGE_CENTER_Y;
    s_car_lamp_cx = 0.0f;
    s_car_lamp_cy = 0.0f;
    s_car_lamp_ref_valid = 0U;
    s_car_lamp_angle_deg = 0.0f;
    s_car_lamp_angle_valid = 0U;
    s_car_lamp_invalid_frame_count = 0U;
}

static image_fusion_candidate_t image_fusion_candidate_from_target(const beacon_data *target,
                                                                     uint8 camera_index,
                                                                     uint8 target_index)
{
    image_fusion_candidate_t candidate;

    candidate.camera_index = (int)camera_index;
    candidate.target_index = target_index;
    candidate.image_x = target->x;
    candidate.image_y = target->y;
    candidate.area = target->area;
    candidate.valid = target->valid;

    if((candidate.valid == 0U) ||
       (image_fusion_candidate_score(&candidate) < IMAGE_FUSION_MIN_TARGET_AREA))
    {
        candidate.valid = 0U;
    }

    return candidate;
}

static void image_fusion_point_from_candidate(const image_fusion_candidate_t *candidate,
                                               uint16 frame_id,
                                               image_fusion_point_t *point)
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

static image_fusion_point_t image_fusion_last_point(void)
{
    return s_last_point;
}

static void image_fusion_store_last_point(const image_fusion_point_t *point)
{
    if((point == NULL) || (point->valid == 0U))
    {
        return;
    }

    s_last_point = *point;
}

static uint8 image_fusion_calc_center_delta(const image_fusion_point_t *point,
                                             float *delta_x,
                                             float *delta_y)
{
    float center_x;
    float center_y;

    if((point == NULL) || (delta_x == NULL) || (delta_y == NULL) || (point->valid == 0U))
    {
        return 0U;
    }

    if(point->camera_index == (int)Front)
    {
        center_x = camera_front_boundary_center_x(point->image_x);
        center_y = camera_front_boundary_center_y(point->image_x) +
                   (point->image_y - CAMERA_CROP_BOUNDARY_BOTTOM_ROW_Y);
    }
    else if(point->camera_index == (int)Center)
    {
        center_x = point->image_x;
        center_y = point->image_y;
    }
    else if(point->camera_index == (int)Back)
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

static void image_fusion_publish_point(const image_fusion_point_t *point)
{
    float center_delta_x;
    float center_delta_y;

    if((point == NULL) || (point->valid == 0U))
    {
        g_image_fusion.active = 0U;
        g_image_fusion.valid = 0U;
        g_image_fusion.camera_id = 0U;
        g_image_fusion.frame_id = s_frame_id;
        g_image_fusion.image_x = 0.0f;
        g_image_fusion.image_y = 0.0f;
        g_image_fusion.center_delta_valid = 0U;
        g_image_fusion.center_delta_x = 0.0f;
        g_image_fusion.center_delta_y = 0.0f;
        g_image_fusion.raw_center_delta_x = 0.0f;
        g_image_fusion.raw_center_delta_y = 0.0f;
        g_image_fusion.lamp_angle_valid = s_car_lamp_angle_valid;
        g_image_fusion.lamp_angle_deg = s_car_lamp_angle_deg;
        g_image_fusion.car_lamp_cx = s_car_lamp_cx;
        g_image_fusion.car_lamp_cy = s_car_lamp_cy;
        g_image_fusion.transform_angle_deg = image_fusion_transform_angle_deg();
        g_image_fusion.area = 0.0f;
        g_image_fusion.missing_frame_count = s_missing_frame_count;
        return;
    }

    g_image_fusion.active = 1U;
    g_image_fusion.valid = 1U;
    g_image_fusion.camera_id = (uint8)point->camera_index;
    g_image_fusion.frame_id = point->frame_id;
    g_image_fusion.image_x = point->image_x;
    g_image_fusion.image_y = point->image_y;
    center_delta_x = 0.0f;
    center_delta_y = 0.0f;
    g_image_fusion.center_delta_valid =
        image_fusion_calc_center_delta(point, &center_delta_x, &center_delta_y);
    g_image_fusion.raw_center_delta_x = center_delta_x;
    g_image_fusion.raw_center_delta_y = center_delta_y;
    g_image_fusion.lamp_angle_valid = s_car_lamp_angle_valid;
    g_image_fusion.lamp_angle_deg = s_car_lamp_angle_deg;
    g_image_fusion.car_lamp_cx = s_car_lamp_cx;
    g_image_fusion.car_lamp_cy = s_car_lamp_cy;
    g_image_fusion.transform_angle_deg = image_fusion_transform_angle_deg();
    if(g_image_fusion.center_delta_valid != 0U)
    {
        image_fusion_rotate_air_to_car(center_delta_x,
                                        center_delta_y,
                                        g_image_fusion.transform_angle_deg,
                                        &g_image_fusion.center_delta_x,
                                        &g_image_fusion.center_delta_y);
    }
    else
    {
        g_image_fusion.center_delta_x = 0.0f;
        g_image_fusion.center_delta_y = 0.0f;
    }
    g_image_fusion.area = point->area;
    g_image_fusion.missing_frame_count = s_missing_frame_count;
}

static uint8 image_fusion_find_largest_candidate(const struct image_data camera[IMAGE_CAMERA_COUNT],
                                                  image_fusion_candidate_t *best_candidate)
{
    uint8 camera_index;
    uint8 target_index;
    float best_score;

    best_score = 0.0f;
    best_candidate->valid = 0U;

    for(camera_index = 0U; camera_index < IMAGE_CAMERA_COUNT; camera_index++)
    {
        for(target_index = 0U; target_index < IMAGE_MAX_BEACON_COUNT; target_index++)
        {
            const image_fusion_candidate_t candidate =
                image_fusion_candidate_from_target(&camera[camera_index].beacon_data[target_index],
                                                    camera_index,
                                                    target_index);
            const float score = image_fusion_start_candidate_score(&candidate);

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

static uint8 image_fusion_find_nearest_candidate(const struct image_data camera[IMAGE_CAMERA_COUNT],
                                                  uint8 camera_index,
                                                  float search_x,
                                                  float search_y,
                                                  float search_radius,
                                                  image_fusion_candidate_t *best_candidate)
{
    uint8 target_index;
    float best_distance;
    const float max_distance2 = image_fusion_square(search_radius);

    if((camera_index >= IMAGE_CAMERA_COUNT) || (best_candidate == NULL))
    {
        return 0U;
    }

    best_distance = 0.0f;
    best_candidate->valid = 0U;

    for(target_index = 0U; target_index < IMAGE_MAX_BEACON_COUNT; target_index++)
    {
        const image_fusion_candidate_t candidate =
            image_fusion_candidate_from_target(&camera[camera_index].beacon_data[target_index],
                                                camera_index,
                                                target_index);
        const float distance2 = image_fusion_distance2(candidate.image_x,
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

static uint8 image_fusion_start_from_largest(const struct image_data camera[IMAGE_CAMERA_COUNT],
                                              uint16 frame_id,
                                              image_fusion_point_t *next_point)
{
    image_fusion_candidate_t start_candidate;

    image_fusion_clear_point(next_point);
    if(image_fusion_find_largest_candidate(camera, &start_candidate) == 0U)
    {
        return 0U;
    }

    image_fusion_clear_track();
    image_fusion_point_from_candidate(&start_candidate, frame_id, next_point);
    image_fusion_store_last_point(next_point);
    return 1U;
}

static uint8 image_fusion_update_current_camera(const struct image_data camera[IMAGE_CAMERA_COUNT],
                                                 uint16 frame_id,
                                                 image_fusion_point_t *next_point)
{
    uint8 current_camera;
    image_fusion_candidate_t best_candidate;
    const image_fusion_point_t last_point = image_fusion_last_point();

    image_fusion_clear_point(next_point);
    if((last_point.valid == 0U) ||
       (last_point.camera_index < 0) ||
       (last_point.camera_index >= (int)IMAGE_CAMERA_COUNT))
    {
        return 0U;
    }

    current_camera = (uint8)last_point.camera_index;
    if(image_fusion_find_nearest_candidate(camera,
                                            current_camera,
                                            last_point.image_x,
                                            last_point.image_y,
                                            TRACK_MATCH_RADIUS,
                                            &best_candidate) == 0U)
    {
        return 0U;
    }

    image_fusion_point_from_candidate(&best_candidate, frame_id, next_point);
    return 1U;
}

static uint8 image_fusion_calc_boundary_center_point(const image_fusion_point_t *point,
                                                      float *center_x,
                                                      float *center_y)
{
    float boundary_y;

    if((point == NULL) || (center_x == NULL) || (center_y == NULL) || (point->valid == 0U))
    {
        return 0U;
    }

    if(point->camera_index == (int)Front)
    {
        camera_front_boundary_point_to_center(point->image_x, &boundary_y, center_x, center_y);
    }
    else if(point->camera_index == (int)Back)
    {
        camera_rear_boundary_point_to_center(point->image_x, &boundary_y, center_x, center_y);
    }
    else
    {
        return 0U;
    }

    if(image_fusion_abs(point->image_y - boundary_y) >= TRACK_BOUNDARY_SWITCH_Y_EPS)
    {
        return 0U;
    }

    return 1U;
}

static uint8 image_fusion_try_switch_to_center(const struct image_data camera[IMAGE_CAMERA_COUNT],
                                                const image_fusion_point_t *source_point,
                                                uint16 frame_id,
                                                image_fusion_point_t *next_point)
{
    float center_x;
    float center_y;
    image_fusion_candidate_t best_candidate;

    image_fusion_clear_point(next_point);
    if(image_fusion_calc_boundary_center_point(source_point, &center_x, &center_y) == 0U)
    {
        return 0U;
    }

    if(image_fusion_find_nearest_candidate(camera,
                                            Center,
                                            center_x,
                                            center_y,
                                            TRACK_CENTER_SEARCH_RADIUS,
                                            &best_candidate) == 0U)
    {
        return 0U;
    }

    image_fusion_point_from_candidate(&best_candidate, frame_id, next_point);
    return 1U;
}

void image_fusion_init(void)
{
    image_fusion_reset_state();
}

static void image_fusion_update_center_car_lamp(const car_lamp_data *lamp)
{
    float normalized_angle;

    if((lamp == NULL) || (lamp->valid == 0U))
    {
        if(s_car_lamp_invalid_frame_count <= IMAGE_FUSION_CAR_LAMP_HOLD_FRAME_LIMIT)
        {
            s_car_lamp_invalid_frame_count++;
        }

        if((s_car_lamp_ref_valid != 0U) &&
           (s_car_lamp_invalid_frame_count <= IMAGE_FUSION_CAR_LAMP_HOLD_FRAME_LIMIT))
        {
            return;
        }

        s_car_lamp_ref_valid = 0U;
        s_car_lamp_cx = 0.0f;
        s_car_lamp_cy = 0.0f;
        s_car_lamp_angle_valid = 0U;
        s_car_lamp_angle_deg = 0.0f;
        g_image_fusion.car_lamp_cx = 0.0f;
        g_image_fusion.car_lamp_cy = 0.0f;
        g_image_fusion.lamp_angle_valid = 0U;
        g_image_fusion.lamp_angle_deg = 0.0f;
        g_image_fusion.transform_angle_deg = 0.0f;
        return;
    }

    s_car_lamp_invalid_frame_count = 0U;

    normalized_angle = image_fusion_normalize_signed_lamp_angle_deg(lamp->angle);
    s_car_lamp_angle_deg = normalized_angle;
    s_car_lamp_cx = lamp->cx;
    s_car_lamp_cy = lamp->cy;
    image_fusion_lamp_ref_from_center(lamp->cx,
                                       lamp->cy,
                                       s_car_lamp_angle_deg,
                                       &s_car_lamp_ref_x,
                                       &s_car_lamp_ref_y);
    s_car_lamp_ref_valid = 1U;
    s_car_lamp_angle_valid = 1U;
    g_image_fusion.car_lamp_cx = s_car_lamp_cx;
    g_image_fusion.car_lamp_cy = s_car_lamp_cy;
}

uint8 image_fusion_update_100HZ(const struct image_data camera[IMAGE_CAMERA_COUNT])
{
    image_fusion_point_t next_point;

    s_frame_id++;
    if(camera == NULL)
    {
        image_fusion_reset_state();
        return 0U;
    }

    image_fusion_update_center_car_lamp(&camera[Center].car_lamp_data[0]);

    if(g_image_fusion.active == 0U)
    {
        if(image_fusion_start_from_largest(camera, s_frame_id, &next_point) == 0U)
        {
            image_fusion_publish_point(NULL);
            return 0U;
        }

        s_missing_frame_count = 0U;
        image_fusion_publish_point(&next_point);
        return 1U;
    }

    if(image_fusion_update_current_camera(camera, s_frame_id, &next_point) != 0U)
    {
        image_fusion_point_t center_point;

        if(image_fusion_try_switch_to_center(camera, &next_point, s_frame_id, &center_point) != 0U)
        {
            next_point = center_point;
        }

        image_fusion_store_last_point(&next_point);
        s_missing_frame_count = 0U;
        image_fusion_publish_point(&next_point);
        return 1U;
    }

    if(s_last_point.valid == 0U)
    {
        image_fusion_publish_point(NULL);
        image_fusion_clear_track();
        return 0U;
    }

    s_missing_frame_count++;
    if(s_missing_frame_count <= TRACK_LOST_FRAME_LIMIT)
    {
        next_point = image_fusion_last_point();
        image_fusion_publish_point(&next_point);
        return 1U;
    }

    image_fusion_publish_point(NULL);
    image_fusion_clear_track();
    return 0U;
}
