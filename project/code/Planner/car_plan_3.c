#include "car_plan_3.h"
#include "Three_Camera.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include <math.h>

#define CAR_PLAN_3_DEG_TO_RAD       (0.017453292519943295f)
#define CAR_PLAN_3_MIN_DISTANCE_M   (0.001f)

extern float g_car_yaw;
extern float Car_Speed;

static car_plan_3_result_t s_car_plan_3_result;
static three_camera_result_t s_car_plan_3_camera;
static int8 s_car_plan_3_selected = -1;

static void CarPlan_3_ClearResult(void)
{
    s_car_plan_3_result.valid = 0U;
    s_car_plan_3_result.camera_mask = 0U;
    s_car_plan_3_result.target_strafe_mps = 0.0f;
    s_car_plan_3_result.target_forward_mps = 0.0f;
    s_car_plan_3_result.target_center_x = 0.0f;
    s_car_plan_3_result.target_center_y = 0.0f;
    s_car_plan_3_selected = -1;
}

void CarPlan_3_Reset(void)
{
    uint8 i;

    CarPlan_3_ClearResult();
    s_car_plan_3_camera.car_lamp.valid = 0U;
    s_car_plan_3_camera.beacon_count = 0U;
    for(i = 0U; i < THREE_CAMERA_MAX_BEACON_COUNT; i++)
    {
        s_car_plan_3_camera.beacon[i].valid = 0U;
    }
}

uint8 CarPlan_3_Update(car_plan_3_result_t *result)
{
    uint8 i;
    uint8 selected = 0xFFU;
    float selected_distance_sq = 0.0f;
    float dx;
    float dy;
    float distance;
    float angle_rad;
    float right_x;
    float right_y;
    float scale;

    CarPlan_3_ClearResult();
    if(Three_Camera_Update(image_data,
                           g_euler.roll,
                           g_euler.pitch,
                           g_euler.yaw,
                           g_tof_fused_height_mm,
                           g_tof_fused_valid,
                           &s_car_plan_3_camera) == 0U ||
       s_car_plan_3_camera.car_lamp.valid == 0U)
    {
        if(result != 0)
        {
            *result = s_car_plan_3_result;
        }
        return 0U;
    }

    for(i = 0U; i < s_car_plan_3_camera.beacon_count; i++)
    {
        float candidate_dx;
        float candidate_dy;
        float candidate_distance_sq;

        if(s_car_plan_3_camera.beacon[i].valid == 0U)
        {
            continue;
        }
        candidate_dx = s_car_plan_3_camera.beacon[i].x_m -
                       s_car_plan_3_camera.car_lamp.x_m;
        candidate_dy = s_car_plan_3_camera.beacon[i].y_m -
                       s_car_plan_3_camera.car_lamp.y_m;
        candidate_distance_sq = candidate_dx * candidate_dx +
                                candidate_dy * candidate_dy;
        if(selected == 0xFFU || candidate_distance_sq < selected_distance_sq)
        {
            selected = i;
            selected_distance_sq = candidate_distance_sq;
        }
    }

    if(selected == 0xFFU ||
       selected_distance_sq <= CAR_PLAN_3_MIN_DISTANCE_M * CAR_PLAN_3_MIN_DISTANCE_M)
    {
        if(result != 0)
        {
            *result = s_car_plan_3_result;
        }
        return 0U;
    }

    dx = s_car_plan_3_camera.beacon[selected].x_m - s_car_plan_3_camera.car_lamp.x_m;
    dy = s_car_plan_3_camera.beacon[selected].y_m - s_car_plan_3_camera.car_lamp.y_m;
    distance = sqrtf(selected_distance_sq);
    angle_rad = s_car_plan_3_camera.car_lamp.angle_deg * CAR_PLAN_3_DEG_TO_RAD;
    right_x = cosf(angle_rad);
    right_y = sinf(angle_rad);
    /* 车灯长轴是车体横轴；yaw只用于将无向长轴统一到车体右向。 */
    if(cosf((s_car_plan_3_camera.car_lamp.angle_deg - g_car_yaw - 90.0f) *
            CAR_PLAN_3_DEG_TO_RAD) < 0.0f)
    {
        right_x = -right_x;
        right_y = -right_y;
    }

    scale = Car_Speed / distance;
    s_car_plan_3_result.valid = 1U;
    s_car_plan_3_result.camera_mask = s_car_plan_3_camera.beacon[selected].camera_mask;
    s_car_plan_3_result.target_strafe_mps =
        (dx * right_x + dy * right_y) * scale;
    s_car_plan_3_result.target_forward_mps =
        (dx * right_y - dy * right_x) * scale;
    s_car_plan_3_result.target_center_x = s_car_plan_3_camera.beacon[selected].x_m;
    s_car_plan_3_result.target_center_y = s_car_plan_3_camera.beacon[selected].y_m;
    s_car_plan_3_selected = (int8)selected;

    if(result != 0)
    {
        *result = s_car_plan_3_result;
    }
    return 1U;
}

void CarPlan_3_GetResult(car_plan_3_result_t *result)
{
    if(result != 0)
    {
        *result = s_car_plan_3_result;
    }
}

void CarPlan_3_GetDebug(car_plan_3_debug_t *debug)
{
    uint8 i;
    uint8 output_count;
    int8 selected_output = s_car_plan_3_selected;

    if(debug == 0)
    {
        return;
    }
    for(i = 0U; i < CAR_PLAN_3_DEBUG_BEACON_COUNT; i++)
    {
        debug->beacon[i].valid = 0U;
        debug->beacon[i].camera_mask = 0U;
        debug->beacon[i].center_x = IMAGE_DATA_INVALID_VALUE;
        debug->beacon[i].center_y = IMAGE_DATA_INVALID_VALUE;
        debug->beacon[i].area = 0.0f;
    }
    debug->selected_target_id = -1;

    output_count = s_car_plan_3_camera.beacon_count;
    if(output_count > CAR_PLAN_3_DEBUG_BEACON_COUNT)
    {
        output_count = CAR_PLAN_3_DEBUG_BEACON_COUNT;
    }
    for(i = 0U; i < output_count; i++)
    {
        debug->beacon[i].valid = s_car_plan_3_camera.beacon[i].valid;
        debug->beacon[i].camera_mask = s_car_plan_3_camera.beacon[i].camera_mask;
        debug->beacon[i].center_x = s_car_plan_3_camera.beacon[i].x_m;
        debug->beacon[i].center_y = s_car_plan_3_camera.beacon[i].y_m;
        debug->beacon[i].area = s_car_plan_3_camera.beacon[i].area;
    }
    if(s_car_plan_3_selected >= (int8)CAR_PLAN_3_DEBUG_BEACON_COUNT)
    {
        i = CAR_PLAN_3_DEBUG_BEACON_COUNT - 1U;
        debug->beacon[i].valid = s_car_plan_3_camera.beacon[s_car_plan_3_selected].valid;
        debug->beacon[i].camera_mask = s_car_plan_3_camera.beacon[s_car_plan_3_selected].camera_mask;
        debug->beacon[i].center_x = s_car_plan_3_camera.beacon[s_car_plan_3_selected].x_m;
        debug->beacon[i].center_y = s_car_plan_3_camera.beacon[s_car_plan_3_selected].y_m;
        debug->beacon[i].area = s_car_plan_3_camera.beacon[s_car_plan_3_selected].area;
        selected_output = (int8)i;
    }
    if(s_car_plan_3_result.valid != 0U)
    {
        debug->selected_target_id = selected_output;
    }
}
