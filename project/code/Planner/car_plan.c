#include "car_plan.h"
#include "ProjectionCenter.h"
#include "car_lamp_fused.h"
#include "../Image/image_data.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include <math.h>
#include <string.h>

#define CAR_PLAN_BEACON0_INDEX             (0U)
#define CAR_PLAN_MIN_DIST_PX               (2.0f)
#define CAR_PLAN_ANGLE_TO_RAD              (0.017453292519943295f)
#define CAR_PLAN_CENTER_MAX_DIST_PX        (65.0f)
#define CAR_PLAN_CAR_CENTER_Y_OFFSET_PX    (10.0f) /* 车体中心相对车灯中心向图像 y 正方向偏移，单位 px。 */

float Car_Speed = 2.2f; /* 车模规划速度，单位 m/s，可由车机通过 AirComm 修改 */
float Car_Speed_Fast = 2.2f; /* 车模快速前进速度，单位 m/s，可由车机通过 AirComm 修改 */
float Car_Plan_Mode = 2.0f; /* 车模规划算法选择：1=car_plan，2=car_plan_2，可由车机通过 AirComm 修改 */
extern float g_car_yaw; /* 车模yaw角，单位deg。 */
extern float g_car_sync_time_ms; /* 最近一次车端同步时间戳，单位ms。 */
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

/**
 * @brief 将指定相机的像素点映射到 Center 相机坐标系。
 * @param camera 输入相机编号。
 * @param x 输入像素点 X 坐标，单位 px。
 * @param y 输入像素点 Y 坐标，单位 px。
 * @param center_x 输出 Center 坐标系 X 坐标，单位 px。
 * @param center_y 输出 Center 坐标系 Y 坐标，单位 px。
 * @return 无。
 */
static void CarPlan_MapPointToCenter(uint8 camera,
                                     float x,
                                     float y,
                                     float *center_x,
                                     float *center_y)
{
    float x2;
    float xy;
    float y2;

    if (camera == (uint8)Center)
    {
        *center_x = x;
        *center_y = y;
        return;
    }

    x2 = x * x;
    xy = x * y;
    y2 = y * y;
    if (camera == (uint8)Front)
    {
        *center_x = -3.224193f + 1.123975f * x + 0.003353f * y +
                    0.000073f * x2 - 0.004078f * xy - 0.000302f * y2;
        *center_y = -60.512112f + 0.030475f * x + 0.772429f * y +
                    0.004336f * x2 - 0.000232f * xy + 0.004678f * y2;
        return;
    }

    *center_x = -10.828701f - 1.119896f * x + 0.059751f * y -
                0.000063f * x2 + 0.004186f * xy - 0.000850f * y2;
    *center_y = 58.428997f - 0.026951f * x - 0.718077f * y -
                0.004166f * x2 + 0.000106f * xy - 0.004593f * y2;
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
    float abs_strafe;
    float abs_forward;
    float plan_speed;
    float speed_scale;
    float beacon_vector_x;
    float beacon_vector_y;
    float projection_vector_x;
    float projection_vector_y;
    float direction_dot;

    if ((image_data_car_lamp_valid(lamp) == 0U) ||
        (image_data_beacon_valid(beacon) == 0U))
    {
        return 0U;
    }

    dx = beacon->x - lamp->cx;
    dy = beacon->y - (lamp->cy + CAR_PLAN_CAR_CENTER_Y_OFFSET_PX);
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
    if ((g_car_lamp_fused.valid != 0U) &&
        (g_car_sync_time_ms > 0.0f))
    {
        /* 将各相机无向长轴统一到真实车体右向。 */
        normal_x = cosf(g_car_lamp_fused.angle * CAR_PLAN_ANGLE_TO_RAD);
        normal_y = sinf(g_car_lamp_fused.angle * CAR_PLAN_ANGLE_TO_RAD);
        if (cosf((g_car_lamp_fused.angle - g_car_yaw + g_euler.yaw) *
                 CAR_PLAN_ANGLE_TO_RAD) < 0.0f)
        {
            normal_x = -normal_x;
            normal_y = -normal_y;
        }
        CarPlan_MapPointToCenter(camera, lamp->cx, lamp->cy,
                                 &beacon_vector_x, &beacon_vector_y);
        CarPlan_MapPointToCenter(camera, lamp->cx + line_x, lamp->cy + line_y,
                                 &projection_vector_x, &projection_vector_y);
        if ((projection_vector_x - beacon_vector_x) * normal_x +
            (projection_vector_y - beacon_vector_y) * normal_y < 0.0f)
        {
            line_x = -line_x;
            line_y = -line_y;
        }
    }
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

    plan_speed = Car_Speed;
    if (g_projection_center.valid != 0U)
    {
        CarPlan_MapPointToCenter(camera, beacon->x, beacon->y,
                                 &beacon_vector_x, &beacon_vector_y);
        CarPlan_MapPointToCenter(camera, lamp->cx,
                                 lamp->cy + CAR_PLAN_CAR_CENTER_Y_OFFSET_PX,
                                 &projection_vector_x, &projection_vector_y);
        beacon_vector_x -= g_projection_center.cx;
        beacon_vector_y -= g_projection_center.cy;
        projection_vector_x = g_projection_center.cx - projection_vector_x;
        projection_vector_y = g_projection_center.cy - projection_vector_y;
        if ((beacon_vector_x * beacon_vector_x + beacon_vector_y * beacon_vector_y) <
            (CAR_PLAN_CENTER_MAX_DIST_PX * CAR_PLAN_CENTER_MAX_DIST_PX))
        {
            beacon_vector_x += projection_vector_x;
            beacon_vector_y += projection_vector_y;
            direction_dot = projection_vector_x * beacon_vector_x +
                            projection_vector_y * beacon_vector_y;
            /* 两个车灯出发向量夹角小于 60 度时使用快速速度。 */
            if ((direction_dot > 0.0f) &&
                ((direction_dot * direction_dot) >
                 (0.25f *
                  (projection_vector_x * projection_vector_x +
                   projection_vector_y * projection_vector_y) *
                  (beacon_vector_x * beacon_vector_x + beacon_vector_y * beacon_vector_y))))
            {
                plan_speed = Car_Speed_Fast;
            }
        }
    }

    /* 取 strafe/forward 中绝对值较大者满速为 plan_speed，另一轴按比例缩放（符号保持不变） */
    abs_strafe = fabsf(strafe);
    abs_forward = fabsf(forward);
    speed_scale = plan_speed / ((abs_strafe > abs_forward) ? abs_strafe : abs_forward);

    out->valid = 1U;
    out->camera = camera;
    out->beacon_index = CAR_PLAN_BEACON0_INDEX;
    out->target_strafe_mps = strafe * speed_scale;
    out->target_forward_mps = forward * speed_scale;
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
    const car_lamp_data *center_lamp = &image_data[Center].car_lamp_data[0];
    const beacon_data *center_beacon = &image_data[Center].beacon_data[CAR_PLAN_BEACON0_INDEX];
    car_plan_result_t candidate;
    uint8 side_camera;

    if ((CarPlan_CarLampValid((uint8)Center) != 0U) &&
        (CarPlan_BeaconValid((uint8)Center) != 0U) &&
        (((center_beacon->x - center_lamp->cx) * (center_beacon->x - center_lamp->cx) +
          (center_beacon->y - (center_lamp->cy + CAR_PLAN_CAR_CENTER_Y_OFFSET_PX)) *
          (center_beacon->y - (center_lamp->cy + CAR_PLAN_CAR_CENTER_Y_OFFSET_PX))) <
         (90.0f * 90.0f)) &&
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
                                           ? -Car_Speed
                                           : Car_Speed,
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
