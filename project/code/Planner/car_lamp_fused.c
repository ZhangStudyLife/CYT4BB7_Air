#include "car_lamp_fused.h"
#include "../Image/image_data.h"
#include <math.h>

static const float s_lamp_lpf_alpha = 0.557f;
static const float s_lamp_step_limit_px = 6.0f;
static const float s_center_weight = 1.0f;
static const float s_side_weight_cap = 0.25f;
static const float s_front_x_weight = 0.227180f;
static const float s_front_y_weight = 0.135893f;
static const float s_back_x_weight = 0.194550f;
static const float s_back_y_weight = 0.134159f;
static const uint8 s_lamp_hold_max_ticks = 20U;

car_lamp_fused_result_t g_car_lamp_fused;
uint8 only_front_see_car_lamp;
uint8 only_back_see_car_lamp;
static uint8 s_lamp_hold_ticks = 0U;

void CarLampFused_Init(void)
{
    g_car_lamp_fused.valid = 0U;
    g_car_lamp_fused.cx = 0.0f;
    g_car_lamp_fused.cy = 0.0f;
    only_front_see_car_lamp = 0U;
    only_back_see_car_lamp = 0U;
    s_lamp_hold_ticks = 0U;
}

uint8 CarLampFused_Update50Hz(void)
{
    const car_lamp_data *front_lamp = &image_data[Front].car_lamp_data[0];
    const car_lamp_data *center_lamp = &image_data[Center].car_lamp_data[0];
    const car_lamp_data *back_lamp = &image_data[Back].car_lamp_data[0];
    uint8 front_valid = image_data_car_lamp_valid(front_lamp);
    uint8 center_valid = image_data_car_lamp_valid(center_lamp);
    uint8 back_valid = image_data_car_lamp_valid(back_lamp);
    uint8 raw_only_front = ((front_valid != 0U) && (center_valid == 0U) && (back_valid == 0U)) ? 1U : 0U;
    uint8 raw_only_back = ((back_valid != 0U) && (center_valid == 0U) && (front_valid == 0U)) ? 1U : 0U;
    float x_sum = 0.0f;
    float y_sum = 0.0f;
    float x_weight = 0.0f;
    float y_weight = 0.0f;
    float side_x_sum = 0.0f;
    float side_y_sum = 0.0f;
    float side_x_weight = 0.0f;
    float side_y_weight = 0.0f;
    float cx;
    float cy;
    float next_cx;
    float next_cy;
    float dx;
    float dy;
    float step;
    float scale;

    only_front_see_car_lamp = raw_only_front;
    only_back_see_car_lamp = raw_only_back;

    if(center_valid != 0U)
    {
        x_sum = center_lamp->cx * s_center_weight;
        y_sum = center_lamp->cy * s_center_weight;
        x_weight = s_center_weight;
        y_weight = s_center_weight;
    }

    if(front_valid != 0U)
    {
        cx = -2.742171f + 1.000615f * front_lamp->cx - 0.007433f * front_lamp->cy;
        cy = -62.238740f - 0.000411f * front_lamp->cx + 1.033626f * front_lamp->cy;
        side_x_sum += cx * s_front_x_weight;
        side_y_sum += cy * s_front_y_weight;
        side_x_weight += s_front_x_weight;
        side_y_weight += s_front_y_weight;
    }

    if(back_valid != 0U)
    {
        cx = -11.578788f - 0.975309f * back_lamp->cx + 0.045800f * back_lamp->cy;
        cy = 58.365121f - 0.049687f * back_lamp->cx - 1.023470f * back_lamp->cy;
        side_x_sum += cx * s_back_x_weight;
        side_y_sum += cy * s_back_y_weight;
        side_x_weight += s_back_x_weight;
        side_y_weight += s_back_y_weight;
    }

    if(center_valid != 0U)
    {
        if(side_x_weight > s_side_weight_cap)
        {
            side_x_sum *= s_side_weight_cap / side_x_weight;
            side_x_weight = s_side_weight_cap;
        }
        if(side_y_weight > s_side_weight_cap)
        {
            side_y_sum *= s_side_weight_cap / side_y_weight;
            side_y_weight = s_side_weight_cap;
        }
        x_sum += side_x_sum;
        y_sum += side_y_sum;
        x_weight += side_x_weight;
        y_weight += side_y_weight;
    }
    else
    {
        x_sum = side_x_sum;
        y_sum = side_y_sum;
        x_weight = side_x_weight;
        y_weight = side_y_weight;
    }

    if((x_weight <= 0.0f) || (y_weight <= 0.0f))
    {
        if(g_car_lamp_fused.valid == 0U)
        {
            g_car_lamp_fused.cx = 0.0f;
            g_car_lamp_fused.cy = 0.0f;
            return 0U;
        }
        if(s_lamp_hold_ticks < s_lamp_hold_max_ticks)
        {
            s_lamp_hold_ticks++;
            return 1U;
        }
        g_car_lamp_fused.valid = 0U;
        g_car_lamp_fused.cx = 0.0f;
        g_car_lamp_fused.cy = 0.0f;
        s_lamp_hold_ticks = 0U;
        return 0U;
    }

    cx = x_sum / x_weight;
    cy = y_sum / y_weight;
    s_lamp_hold_ticks = 0U;
    if(g_car_lamp_fused.valid == 0U)
    {
        g_car_lamp_fused.valid = 1U;
        g_car_lamp_fused.cx = cx;
        g_car_lamp_fused.cy = cy;
        return 1U;
    }

    next_cx = g_car_lamp_fused.cx + s_lamp_lpf_alpha * (cx - g_car_lamp_fused.cx);
    next_cy = g_car_lamp_fused.cy + s_lamp_lpf_alpha * (cy - g_car_lamp_fused.cy);
    dx = next_cx - g_car_lamp_fused.cx;
    dy = next_cy - g_car_lamp_fused.cy;
    step = sqrtf(dx * dx + dy * dy);
    if(step > s_lamp_step_limit_px)
    {
        scale = s_lamp_step_limit_px / step;
        dx *= scale;
        dy *= scale;
    }
    g_car_lamp_fused.cx += dx;
    g_car_lamp_fused.cy += dy;
    return 1U;
}
