#include "car_plan.h"
#include "../Image/image_data.h"
#include <math.h>

#define CAR_PLAN_BEACON_SCAN_COUNT         (2U)
#define CAR_PLAN_MIN_DIST_PX               (2.0f)
#define CAR_PLAN_ANGLE_TO_RAD              (0.017453292519943295f)

typedef struct
{
    uint8 found;
    car_plan_result_t result;
} car_plan_candidate_t;

static car_plan_result_t s_car_plan_result;
static uint8 s_lock_valid = 0U;
static uint8 s_lock_camera = 0U;
static uint8 s_lock_beacon_index = 0U;
static uint16 s_lock_lost_ticks = 0U;

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

static uint8 CarPlan_MakeCandidate(uint8 camera, uint8 beacon_index, car_plan_result_t *out)
{
    const car_lamp_data *lamp = &image_data[camera].car_lamp_data[0];
    const beacon_data *beacon = &image_data[camera].beacon_data[beacon_index];
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

    if ((lamp->valid == 0U) || (beacon->valid == 0U))
    {
        return 0U;
    }

    dx = beacon->x - lamp->cx;
    dy = beacon->y - lamp->cy;
    dist = sqrtf(dx * dx + dy * dy);
    if (dist <= CAR_PLAN_MIN_DIST_PX)
    {
        return 0U;
    }

    inv_dist = 1.0f / dist;
    dx *= inv_dist;
    dy *= inv_dist;

    angle_rad = lamp->angle * CAR_PLAN_ANGLE_TO_RAD;
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
    out->target_strafe_mps = strafe * CAR_PLAN_SPEED_MPS;
    out->target_forward_mps = forward * CAR_PLAN_SPEED_MPS;
    out->dist_px = dist;
    out->along = along;
    out->perp = perp;
    return 1U;
}

static void CarPlan_TryUpdateBest(const car_plan_result_t *candidate, car_plan_candidate_t *best)
{
    if ((best->found == 0U) || (candidate->dist_px < best->result.dist_px))
    {
        best->found = 1U;
        best->result = *candidate;
    }
}

void CarPlan_Reset(void)
{
    CarPlan_ClearResult(&s_car_plan_result);
    s_lock_valid = 0U;
    s_lock_camera = 0U;
    s_lock_beacon_index = 0U;
    s_lock_lost_ticks = 0U;
}

uint8 CarPlan_Update(car_plan_result_t *result)
{
    car_plan_candidate_t best;
    car_plan_candidate_t locked;
    car_plan_result_t candidate;
    uint8 camera;
    uint8 beacon_index;

    memset(&best, 0, sizeof(best));
    memset(&locked, 0, sizeof(locked));

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
        CarPlan_CopyResult(result, &s_car_plan_result);
        return 0U;
    }

    if (locked.found != 0U)
    {
        if (best.result.dist_px + CAR_PLAN_LOCK_SWITCH_MARGIN_PX < locked.result.dist_px)
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
    else if ((s_lock_valid == 0U) || (s_lock_lost_ticks >= CAR_PLAN_LOCK_LOST_HOLD_TICKS))
    {
        s_car_plan_result = best.result;
        s_lock_valid = 1U;
        s_lock_camera = s_car_plan_result.camera;
        s_lock_beacon_index = s_car_plan_result.beacon_index;
        s_lock_lost_ticks = 0U;
    }
    else
    {
        s_lock_lost_ticks++;
        CarPlan_ClearResult(&s_car_plan_result);
    }

    CarPlan_CopyResult(result, &s_car_plan_result);
    return s_car_plan_result.valid;
}

void CarPlan_GetResult(car_plan_result_t *result)
{
    CarPlan_CopyResult(result, &s_car_plan_result);
}
