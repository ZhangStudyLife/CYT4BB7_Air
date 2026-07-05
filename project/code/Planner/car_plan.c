#include "car_plan.h"
#include "../Image/image_data.h"
#include <math.h>

#define CAR_PLAN_BEACON_SCAN_COUNT         (2U)
#define CAR_PLAN_MIN_DIST_PX               (2.0f)
#define CAR_PLAN_ANGLE_TO_RAD              (0.017453292519943295f)
#define CAR_PLAN_STALE_LAMP_MAX_TICKS      (150U)
#define CAR_PLAN_STALE_LAMP_MIN_DIST_PX    (12.0f)
#define CAR_PLAN_STALE_LAMP_SPEED_SCALE    (0.5f)
#define CAR_PLAN_FRONT_FORWARD_Y_PX        (-20.0f)
#define CAR_PLAN_FRONT_STALE_Y_PX          (-10.0f)
#define CAR_PLAN_FRONT_LEFT_Y_PX           (33.0f)
#define CAR_PLAN_FRONT_LEFT_X_PX           (-50.0f)
#define CAR_PLAN_BACK_STRAFE_X_SCALE_PX    (70.0f)
#define CAR_PLAN_BACK_STRAFE_LIMIT         (0.9f)
#define CAR_PLAN_BACK_FORWARD_Y_PX         (-20.0f)
#define CAR_PLAN_BACK_FORWARD_HIGH_Y_PX    (40.0f)
#define CAR_PLAN_INTENT_HOLD_TICKS         (200U)
#define CAR_PLAN_INTENT_NEG_FORWARD_MPS    (-0.15f)
#define CAR_PLAN_INTENT_MIN_SPEED_MPS      (0.2f)
#define CAR_PLAN_CONTINUITY_DOT_MIN        (0.35f)
#define CAR_PLAN_CONTINUITY_REVERSE_DOT    (-0.35f)
#define CAR_PLAN_CONTINUITY_DIST_MARGIN_PX (55.0f)
#define CAR_PLAN_CONTINUITY_MIN_SPEED_MPS  (0.05f)

typedef struct
{
    uint8 found;
    car_plan_result_t result;
} car_plan_candidate_t;

typedef struct
{
    uint8 valid;
    uint16 age_ticks;
    float cx;
    float cy;
    float angle;
} car_plan_lamp_memory_t;

static car_plan_result_t s_car_plan_result;
static uint8 s_lock_valid = 0U;
static uint8 s_lock_camera = 0U;
static uint8 s_lock_beacon_index = 0U;
static uint16 s_lock_lost_ticks = 0U;
static car_plan_lamp_memory_t s_lamp_memory[IMAGE_CAMERA_COUNT];
static uint8 s_last_valid_result_valid = 0U;
static uint16 s_last_valid_result_age_ticks = 0U;
static car_plan_result_t s_last_valid_result;

static void CarPlan_ClearResult(car_plan_result_t *result)
{
    memset(result, 0, sizeof(*result));
}

static void CarPlan_CopyResult(car_plan_result_t *dst, const car_plan_result_t *src)
{
    if (dst != 0)
    {
        *dst = *src;
    }
}

static float CarPlan_LimitAbs(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static void CarPlan_LimitVector(float *strafe, float *forward)
{
    float mag = sqrtf((*strafe * *strafe) + (*forward * *forward));

    if (mag > 1.0f)
    {
        float scale = 1.0f / mag;
        *strafe *= scale;
        *forward *= scale;
    }
}

static uint8 CarPlan_GetDirectionDot(const car_plan_result_t *a,
                                     const car_plan_result_t *b,
                                     float *dot)
{
    float a_mag = sqrtf((a->target_strafe_mps * a->target_strafe_mps) +
                        (a->target_forward_mps * a->target_forward_mps));
    float b_mag = sqrtf((b->target_strafe_mps * b->target_strafe_mps) +
                        (b->target_forward_mps * b->target_forward_mps));

    if ((a_mag < CAR_PLAN_CONTINUITY_MIN_SPEED_MPS) ||
        (b_mag < CAR_PLAN_CONTINUITY_MIN_SPEED_MPS))
    {
        return 0U;
    }

    *dot = ((a->target_strafe_mps * b->target_strafe_mps) +
            (a->target_forward_mps * b->target_forward_mps)) /
           (a_mag * b_mag);
    return 1U;
}

static uint8 CarPlan_ContinuityPrefersFirst(const car_plan_result_t *first,
                                            const car_plan_result_t *second)
{
    float first_dot;
    float second_dot;

    if ((s_last_valid_result_valid == 0U) ||
        (s_last_valid_result_age_ticks >= CAR_PLAN_INTENT_HOLD_TICKS) ||
        (CarPlan_GetDirectionDot(first, &s_last_valid_result, &first_dot) == 0U) ||
        (CarPlan_GetDirectionDot(second, &s_last_valid_result, &second_dot) == 0U))
    {
        return 0U;
    }

    return ((first_dot > CAR_PLAN_CONTINUITY_DOT_MIN) &&
            (second_dot < CAR_PLAN_CONTINUITY_REVERSE_DOT) &&
            (first->dist_px <=
             (second->dist_px + CAR_PLAN_CONTINUITY_DIST_MARGIN_PX)))
               ? 1U
               : 0U;
}

static uint8 CarPlan_MakeCandidateFromGeometry(uint8 camera,
                                               uint8 beacon_index,
                                               float lamp_cx,
                                               float lamp_cy,
                                               float lamp_angle,
                                               float beacon_x,
                                               float beacon_y,
                                               float speed_scale,
                                               car_plan_result_t *out)
{
    float dx;
    float dy;
    float dist;
    float inv_dist;
    float angle_rad;
    float line_x;
    float line_y;
    float normal_x;
    float normal_y;
    float along;
    float perp;
    float strafe;
    float forward;

    dx = beacon_x - lamp_cx;
    dy = beacon_y - lamp_cy;
    dist = sqrtf(dx * dx + dy * dy);
    if (dist <= CAR_PLAN_MIN_DIST_PX)
    {
        return 0U;
    }

    inv_dist = 1.0f / dist;
    dx *= inv_dist;
    dy *= inv_dist;

    angle_rad = lamp_angle * CAR_PLAN_ANGLE_TO_RAD;
    line_x = cosf(angle_rad);
    line_y = sinf(angle_rad);
    normal_x = -line_y;
    normal_y = line_x;
    along = dx * line_x + dy * line_y;
    perp = dx * normal_x + dy * normal_y;

    if (camera == (uint8)Back)
    {
        strafe = -along;
        forward = perp;
    }
    else
    {
        strafe = along;
        forward = -perp;
    }

    out->valid = 1U;
    out->camera = camera;
    out->beacon_index = beacon_index;
    out->target_strafe_mps = strafe * CAR_PLAN_SPEED_MPS * speed_scale;
    out->target_forward_mps = forward * CAR_PLAN_SPEED_MPS * speed_scale;
    out->dist_px = dist;
    out->along = along;
    out->perp = perp;
    return 1U;
}

static uint8 CarPlan_MakeCandidate(uint8 camera, uint8 beacon_index, car_plan_result_t *out)
{
    const car_lamp_data *lamp = &image_data[camera].car_lamp_data[0];
    const beacon_data *beacon = &image_data[camera].beacon_data[beacon_index];

    if ((lamp->valid == 0U) || (beacon->valid == 0U))
    {
        return 0U;
    }

    return CarPlan_MakeCandidateFromGeometry(camera,
                                             beacon_index,
                                             lamp->cx,
                                             lamp->cy,
                                             lamp->angle,
                                             beacon->x,
                                             beacon->y,
                                             1.0f,
                                             out);
}

static void CarPlan_TryUpdateBest(const car_plan_result_t *candidate, car_plan_candidate_t *best)
{
    if ((best->found == 0U) ||
        (CarPlan_ContinuityPrefersFirst(candidate, &best->result) != 0U) ||
        ((CarPlan_ContinuityPrefersFirst(&best->result, candidate) == 0U) &&
         (candidate->dist_px < best->result.dist_px)))
    {
        best->found = 1U;
        best->result = *candidate;
    }
}

static void CarPlan_UpdateLampMemory(void)
{
    uint8 camera;

    for (camera = 0U; camera < (uint8)IMAGE_CAMERA_COUNT; camera++)
    {
        const car_lamp_data *lamp = &image_data[camera].car_lamp_data[0];

        if (lamp->valid != 0U)
        {
            s_lamp_memory[camera].valid = 1U;
            s_lamp_memory[camera].age_ticks = 0U;
            s_lamp_memory[camera].cx = lamp->cx;
            s_lamp_memory[camera].cy = lamp->cy;
            s_lamp_memory[camera].angle = lamp->angle;
        }
        else if ((s_lamp_memory[camera].valid != 0U) &&
                 (s_lamp_memory[camera].age_ticks < 0xFFFFU))
        {
            s_lamp_memory[camera].age_ticks++;
        }
    }
}

static void CarPlan_AgeLastValidResult(void)
{
    if ((s_last_valid_result_valid != 0U) &&
        (s_last_valid_result_age_ticks < 0xFFFFU))
    {
        s_last_valid_result_age_ticks++;
    }
}

static void CarPlan_UpdateLastValidResult(const car_plan_result_t *result)
{
    if (result->valid != 0U)
    {
        s_last_valid_result = *result;
        s_last_valid_result_valid = 1U;
        s_last_valid_result_age_ticks = 0U;
    }
    else
    {
        CarPlan_AgeLastValidResult();
    }
}

static uint8 CarPlan_MakeStaleLampCandidate(car_plan_result_t *out)
{
    car_plan_candidate_t best;
    car_plan_result_t candidate;
    uint8 camera;
    uint8 beacon_index;

    memset(&best, 0, sizeof(best));

    for (camera = 0U; camera < (uint8)IMAGE_CAMERA_COUNT; camera++)
    {
        if (camera != (uint8)Front)
        {
            continue;
        }

        if ((s_lamp_memory[camera].valid == 0U) ||
            (s_lamp_memory[camera].age_ticks > CAR_PLAN_STALE_LAMP_MAX_TICKS))
        {
            continue;
        }

        for (beacon_index = 0U; beacon_index < CAR_PLAN_BEACON_SCAN_COUNT; beacon_index++)
        {
            const beacon_data *beacon = &image_data[camera].beacon_data[beacon_index];

            if (beacon->valid == 0U)
            {
                continue;
            }

            if (CarPlan_MakeCandidateFromGeometry(camera,
                                                  beacon_index,
                                                  s_lamp_memory[camera].cx,
                                                  s_lamp_memory[camera].cy,
                                                  s_lamp_memory[camera].angle,
                                                  beacon->x,
                                                  beacon->y,
                                                  CAR_PLAN_STALE_LAMP_SPEED_SCALE,
                                                  &candidate) == 0U)
            {
                continue;
            }

            if (candidate.dist_px < CAR_PLAN_STALE_LAMP_MIN_DIST_PX)
            {
                continue;
            }

            if ((camera == (uint8)Front) &&
                (beacon->y > CAR_PLAN_FRONT_STALE_Y_PX) &&
                (beacon->y < CAR_PLAN_FRONT_LEFT_Y_PX) &&
                (beacon->x > CAR_PLAN_FRONT_LEFT_X_PX))
            {
                continue;
            }

            CarPlan_TryUpdateBest(&candidate, &best);
        }
    }

    if (best.found == 0U)
    {
        return 0U;
    }

    *out = best.result;
    return 1U;
}

static uint8 CarPlan_GetFirstValidBeacon(uint8 camera, uint8 *beacon_index, const beacon_data **beacon)
{
    uint8 index;

    for (index = 0U; index < CAR_PLAN_BEACON_SCAN_COUNT; index++)
    {
        const beacon_data *item = &image_data[camera].beacon_data[index];

        if (item->valid != 0U)
        {
            *beacon_index = index;
            *beacon = item;
            return 1U;
        }
    }

    return 0U;
}

static void CarPlan_SetFallbackResult(uint8 camera,
                                      uint8 beacon_index,
                                      float strafe,
                                      float forward,
                                      float dist_px,
                                      car_plan_result_t *out)
{
    CarPlan_LimitVector(&strafe, &forward);
    out->valid = 1U;
    out->camera = camera;
    out->beacon_index = beacon_index;
    out->target_strafe_mps = strafe * CAR_PLAN_SPEED_MPS;
    out->target_forward_mps = forward * CAR_PLAN_SPEED_MPS;
    out->dist_px = dist_px;
    out->along = strafe;
    out->perp = forward;
}

static uint8 CarPlan_MakeFrontSplitCandidate(car_plan_result_t *out)
{
    const beacon_data *beacon;
    uint8 beacon_index;
    float strafe;
    float forward;
    float dist_px;

    if (((image_data[(uint8)Center].car_lamp_data[0].valid == 0U) &&
         (image_data[(uint8)Back].car_lamp_data[0].valid == 0U)) ||
        (CarPlan_GetFirstValidBeacon((uint8)Front, &beacon_index, &beacon) == 0U))
    {
        return 0U;
    }

    if (beacon->y < CAR_PLAN_FRONT_FORWARD_Y_PX)
    {
        if ((s_last_valid_result_valid != 0U) &&
            (s_last_valid_result_age_ticks < CAR_PLAN_INTENT_HOLD_TICKS) &&
            (s_last_valid_result.target_forward_mps > CAR_PLAN_INTENT_MIN_SPEED_MPS))
        {
            strafe = s_last_valid_result.target_strafe_mps / CAR_PLAN_SPEED_MPS;
            forward = s_last_valid_result.target_forward_mps / CAR_PLAN_SPEED_MPS;
        }
        else
        {
            strafe = 0.0f;
            forward = 1.0f;
        }
    }
    else if ((beacon->y > CAR_PLAN_FRONT_LEFT_Y_PX) ||
             (beacon->x < CAR_PLAN_FRONT_LEFT_X_PX))
    {
        strafe = -1.0f;
        forward = 0.0f;
    }
    else if ((s_last_valid_result_valid != 0U) &&
             (s_last_valid_result_age_ticks < CAR_PLAN_INTENT_HOLD_TICKS) &&
             (sqrtf((s_last_valid_result.target_strafe_mps *
                     s_last_valid_result.target_strafe_mps) +
                    (s_last_valid_result.target_forward_mps *
                     s_last_valid_result.target_forward_mps)) >
              CAR_PLAN_INTENT_MIN_SPEED_MPS))
    {
        strafe = s_last_valid_result.target_strafe_mps / CAR_PLAN_SPEED_MPS;
        forward = s_last_valid_result.target_forward_mps / CAR_PLAN_SPEED_MPS;
    }
    else
    {
        return 0U;
    }

    dist_px = sqrtf((beacon->x * beacon->x) + (beacon->y * beacon->y));
    CarPlan_SetFallbackResult((uint8)Front, beacon_index, strafe, forward, dist_px, out);
    return 1U;
}

static uint8 CarPlan_MakeCenterSplitCandidate(car_plan_result_t *out)
{
    const beacon_data *beacon;
    uint8 beacon_index;

    if (((image_data[(uint8)Front].car_lamp_data[0].valid == 0U) &&
         (image_data[(uint8)Back].car_lamp_data[0].valid == 0U)) ||
        (CarPlan_GetFirstValidBeacon((uint8)Center, &beacon_index, &beacon) == 0U) ||
        (s_last_valid_result_valid == 0U) ||
        (s_last_valid_result_age_ticks >= CAR_PLAN_INTENT_HOLD_TICKS))
    {
        return 0U;
    }

    if (sqrtf((s_last_valid_result.target_strafe_mps * s_last_valid_result.target_strafe_mps) +
              (s_last_valid_result.target_forward_mps * s_last_valid_result.target_forward_mps)) <
        CAR_PLAN_INTENT_MIN_SPEED_MPS)
    {
        return 0U;
    }

    *out = s_last_valid_result;
    out->valid = 1U;
    out->camera = (uint8)Center;
    out->beacon_index = beacon_index;
    out->dist_px = sqrtf((beacon->x * beacon->x) + (beacon->y * beacon->y));
    return 1U;
}

static uint8 CarPlan_MakeBackSplitCandidate(car_plan_result_t *out)
{
    const beacon_data *beacon;
    uint8 beacon_index;
    float strafe;
    float forward;
    float dist_px;

    if (((image_data[(uint8)Front].car_lamp_data[0].valid == 0U) &&
         (image_data[(uint8)Center].car_lamp_data[0].valid == 0U)) ||
        (CarPlan_GetFirstValidBeacon((uint8)Back, &beacon_index, &beacon) == 0U))
    {
        return 0U;
    }

    strafe = CarPlan_LimitAbs(-beacon->x / CAR_PLAN_BACK_STRAFE_X_SCALE_PX,
                              CAR_PLAN_BACK_STRAFE_LIMIT);
    forward = 0.0f;

    if (beacon->y < CAR_PLAN_BACK_FORWARD_Y_PX)
    {
        if (image_data[(uint8)Center].car_lamp_data[0].valid != 0U)
        {
            forward = -0.6f;
        }
        else
        {
            forward = 0.6f;
        }
    }
    else if (beacon->y > CAR_PLAN_BACK_FORWARD_HIGH_Y_PX)
    {
        forward = 0.25f;
    }

    if ((beacon->x < CAR_PLAN_FRONT_LEFT_X_PX) &&
        (beacon->y <= CAR_PLAN_BACK_FORWARD_Y_PX))
    {
        forward = -0.6f;
    }

    dist_px = sqrtf((beacon->x * beacon->x) + (beacon->y * beacon->y));
    CarPlan_SetFallbackResult((uint8)Back, beacon_index, strafe, forward, dist_px, out);
    return 1U;
}

static uint8 CarPlan_MakeLampOnlyIntentCandidate(car_plan_result_t *out)
{
    if (((image_data[(uint8)Front].car_lamp_data[0].valid == 0U) &&
         (image_data[(uint8)Center].car_lamp_data[0].valid == 0U) &&
         (image_data[(uint8)Back].car_lamp_data[0].valid == 0U)) ||
        (s_last_valid_result_valid == 0U) ||
        (s_last_valid_result_age_ticks >= CAR_PLAN_INTENT_HOLD_TICKS) ||
        (s_last_valid_result.target_forward_mps >= CAR_PLAN_INTENT_NEG_FORWARD_MPS))
    {
        return 0U;
    }

    *out = s_last_valid_result;
    out->valid = 1U;
    return 1U;
}

static uint8 CarPlan_MakeFallbackCandidate(car_plan_result_t *out, uint8 *refresh_last_valid)
{
    if (CarPlan_MakeStaleLampCandidate(out) != 0U)
    {
        *refresh_last_valid = 1U;
        return 1U;
    }
    if (CarPlan_MakeFrontSplitCandidate(out) != 0U)
    {
        *refresh_last_valid = 1U;
        return 1U;
    }
    if (CarPlan_MakeCenterSplitCandidate(out) != 0U)
    {
        *refresh_last_valid = 1U;
        return 1U;
    }
    if (CarPlan_MakeBackSplitCandidate(out) != 0U)
    {
        *refresh_last_valid = 1U;
        return 1U;
    }
    if (CarPlan_MakeLampOnlyIntentCandidate(out) != 0U)
    {
        *refresh_last_valid = 0U;
        return 1U;
    }
    return 0U;
}

void CarPlan_Reset(void)
{
    CarPlan_ClearResult(&s_car_plan_result);
    CarPlan_ClearResult(&s_last_valid_result);
    s_lock_valid = 0U;
    s_lock_camera = 0U;
    s_lock_beacon_index = 0U;
    s_lock_lost_ticks = 0U;
    s_last_valid_result_valid = 0U;
    s_last_valid_result_age_ticks = 0U;
    memset(s_lamp_memory, 0, sizeof(s_lamp_memory));
}

uint8 CarPlan_Update(car_plan_result_t *result)
{
    car_plan_candidate_t best;
    car_plan_candidate_t locked;
    car_plan_result_t candidate;
    uint8 camera;
    uint8 beacon_index;
    uint8 refresh_last_valid;

    memset(&best, 0, sizeof(best));
    memset(&locked, 0, sizeof(locked));
    CarPlan_UpdateLampMemory();

    for (camera = 0U; camera < (uint8)IMAGE_CAMERA_COUNT; camera++)
    {
        for (beacon_index = 0U; beacon_index < CAR_PLAN_BEACON_SCAN_COUNT; beacon_index++)
        {
            if (CarPlan_MakeCandidate(camera, beacon_index, &candidate) == 0U)
            {
                continue;
            }

            CarPlan_TryUpdateBest(&candidate, &best);
            if ((s_lock_valid != 0U) &&
                (camera == s_lock_camera) &&
                (beacon_index == s_lock_beacon_index))
            {
                locked.found = 1U;
                locked.result = candidate;
            }
        }
    }

    if (best.found == 0U)
    {
        refresh_last_valid = 1U;
        if (CarPlan_MakeFallbackCandidate(&candidate, &refresh_last_valid) != 0U)
        {
            s_car_plan_result = candidate;
            s_lock_valid = 0U;
            s_lock_lost_ticks = 0U;
        }
        else
        {
            CarPlan_ClearResult(&s_car_plan_result);
            if (s_lock_valid != 0U)
            {
                if (s_lock_lost_ticks < CAR_PLAN_LOCK_LOST_HOLD_TICKS)
                {
                    s_lock_lost_ticks++;
                }
                else
                {
                    s_lock_valid = 0U;
                    s_lock_lost_ticks = 0U;
                }
            }
        }
        if (refresh_last_valid != 0U)
        {
            CarPlan_UpdateLastValidResult(&s_car_plan_result);
        }
        else
        {
            CarPlan_AgeLastValidResult();
        }
        CarPlan_CopyResult(result, &s_car_plan_result);
        return s_car_plan_result.valid;
    }

    if (locked.found != 0U)
    {
        if ((best.result.dist_px + CAR_PLAN_LOCK_SWITCH_MARGIN_PX < locked.result.dist_px) ||
            (CarPlan_ContinuityPrefersFirst(&best.result, &locked.result) != 0U))
        {
            s_car_plan_result = best.result;
        }
        else
        {
            s_car_plan_result = locked.result;
        }
        s_lock_valid = 1U;
        s_lock_camera = s_car_plan_result.camera;
        s_lock_beacon_index = s_car_plan_result.beacon_index;
        s_lock_lost_ticks = 0U;
    }
    else
    {
        s_car_plan_result = best.result;
        s_lock_valid = 1U;
        s_lock_camera = s_car_plan_result.camera;
        s_lock_beacon_index = s_car_plan_result.beacon_index;
        s_lock_lost_ticks = 0U;
    }

    CarPlan_UpdateLastValidResult(&s_car_plan_result);
    CarPlan_CopyResult(result, &s_car_plan_result);
    return s_car_plan_result.valid;
}

void CarPlan_GetResult(car_plan_result_t *result)
{
    CarPlan_CopyResult(result, &s_car_plan_result);
}
