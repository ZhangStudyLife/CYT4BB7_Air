#include "car_plan.h"
#include "../Image/image_data.h"
#include <math.h>
#include <string.h>

#define CAR_PLAN_BEACON0_INDEX             (0U)
#define CAR_PLAN_MIN_DIST_PX               (2.0f)
#define CAR_PLAN_ANGLE_TO_RAD              (0.017453292519943295f)
#define CAR_PLAN_CENTER_X_LIMIT_PX         (70.0f)

static car_plan_result_t s_car_plan_result;

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

static uint8 CarPlan_BeaconValid(uint8 camera)
{
    return image_data_beacon_valid(&image_data[camera].beacon_data[CAR_PLAN_BEACON0_INDEX]);
}

static uint8 CarPlan_CarLampValid(uint8 camera)
{
    return image_data_car_lamp_valid(&image_data[camera].car_lamp_data[0]);
}

static uint8 CarPlan_MakeGeometryResult(uint8 camera, car_plan_result_t *out)
{
    const car_lamp_data *lamp = &image_data[camera].car_lamp_data[0];
    const beacon_data *beacon = &image_data[camera].beacon_data[CAR_PLAN_BEACON0_INDEX];
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

    if ((image_data_car_lamp_valid(lamp) == 0U) ||
        (image_data_beacon_valid(beacon) == 0U))
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
    out->beacon_index = CAR_PLAN_BEACON0_INDEX;
    out->target_strafe_mps = strafe * CAR_PLAN_SPEED_MPS;
    out->target_forward_mps = forward * CAR_PLAN_SPEED_MPS;
    out->dist_px = dist;
    out->along = along;
    out->perp = perp;
    return 1U;
}

static void CarPlan_SetForcedForwardResult(uint8 camera,
                                           float forward,
                                           car_plan_result_t *out)
{
    const beacon_data *beacon = &image_data[camera].beacon_data[CAR_PLAN_BEACON0_INDEX];

    out->valid = 1U;
    out->camera = camera;
    out->beacon_index = CAR_PLAN_BEACON0_INDEX;
    out->target_strafe_mps = 0.0f;
    out->target_forward_mps = forward;
    out->dist_px = sqrtf((beacon->x * beacon->x) + (beacon->y * beacon->y));
    out->along = 0.0f;
    out->perp = forward;
}

static uint8 CarPlan_SelectSideBeacon0(uint8 *camera)
{
    const beacon_data *front_beacon = &image_data[Front].beacon_data[CAR_PLAN_BEACON0_INDEX];
    const beacon_data *back_beacon = &image_data[Back].beacon_data[CAR_PLAN_BEACON0_INDEX];
    uint8 front_valid = CarPlan_BeaconValid((uint8)Front);
    uint8 back_valid = CarPlan_BeaconValid((uint8)Back);

    if ((front_valid == 0U) && (back_valid == 0U))
    {
        return 0U;
    }

    if ((front_valid != 0U) && (back_valid == 0U))
    {
        *camera = (uint8)Front;
        return 1U;
    }

    if ((front_valid == 0U) && (back_valid != 0U))
    {
        *camera = (uint8)Back;
        return 1U;
    }

    if (front_beacon->area > back_beacon->area)
    {
        *camera = (uint8)Front;
        return 1U;
    }

    if (back_beacon->area > front_beacon->area)
    {
        *camera = (uint8)Back;
        return 1U;
    }

    if ((CarPlan_CarLampValid((uint8)Back) != 0U) &&
        (CarPlan_CarLampValid((uint8)Front) == 0U))
    {
        *camera = (uint8)Back;
        return 1U;
    }

    *camera = (uint8)Front;
    return 1U;
}

void CarPlan_Reset(void)
{
    CarPlan_ClearResult(&s_car_plan_result);
}

uint8 CarPlan_Update(car_plan_result_t *result)
{
    const beacon_data *center_beacon = &image_data[Center].beacon_data[CAR_PLAN_BEACON0_INDEX];
    car_plan_result_t candidate;
    uint8 side_camera;

    if ((CarPlan_CarLampValid((uint8)Center) != 0U) &&
        (CarPlan_BeaconValid((uint8)Center) != 0U) &&
        (fabsf(center_beacon->x) <= CAR_PLAN_CENTER_X_LIMIT_PX) &&
        (CarPlan_MakeGeometryResult((uint8)Center, &candidate) != 0U))
    {
        s_car_plan_result = candidate;
        CarPlan_CopyResult(result, &s_car_plan_result);
        return s_car_plan_result.valid;
    }

    if (CarPlan_SelectSideBeacon0(&side_camera) != 0U)
    {
        if (CarPlan_CarLampValid(side_camera) != 0U)
        {
            if (CarPlan_MakeGeometryResult(side_camera, &candidate) != 0U)
            {
                s_car_plan_result = candidate;
                CarPlan_CopyResult(result, &s_car_plan_result);
                return s_car_plan_result.valid;
            }

            CarPlan_ClearResult(&s_car_plan_result);
            CarPlan_CopyResult(result, &s_car_plan_result);
            return 0U;
        }

        CarPlan_SetForcedForwardResult(side_camera,
                                       (side_camera == (uint8)Back)
                                           ? -CAR_PLAN_SPEED_MPS
                                           : CAR_PLAN_SPEED_MPS,
                                       &candidate);
        s_car_plan_result = candidate;
        CarPlan_CopyResult(result, &s_car_plan_result);
        return s_car_plan_result.valid;
    }

    CarPlan_ClearResult(&s_car_plan_result);
    CarPlan_CopyResult(result, &s_car_plan_result);
    return 0U;
}

void CarPlan_GetResult(car_plan_result_t *result)
{
    CarPlan_CopyResult(result, &s_car_plan_result);
}
