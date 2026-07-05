#include "car_lamp_fused.h"
#include "../Image/image_data.h"

static const float s_lamp_lpf_alpha = 0.557f;
static const float s_lamp_decay = 0.95f;
static const float s_center_weight = 1.0f;
static const float s_side_weight_cap = 0.25f;
static const float s_front_x_weight = 0.227180f;
static const float s_front_y_weight = 0.135893f;
static const float s_back_x_weight = 0.194550f;
static const float s_back_y_weight = 0.134159f;

car_lamp_fused_result_t g_car_lamp_fused;

void CarLampFused_Init(void)
{
    g_car_lamp_fused.valid = 0U;
    g_car_lamp_fused.cx = 0.0f;
    g_car_lamp_fused.cy = 0.0f;
}

uint8 CarLampFused_Update50Hz(void)
{
    const car_lamp_data *front_lamp = &image_data[Front].car_lamp_data[0];
    const car_lamp_data *center_lamp = &image_data[Center].car_lamp_data[0];
    const car_lamp_data *back_lamp = &image_data[Back].car_lamp_data[0];
    uint8 center_valid = image_data_car_lamp_valid(center_lamp);
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

    if(center_valid != 0U)
    {
        x_sum = center_lamp->cx * s_center_weight;
        y_sum = center_lamp->cy * s_center_weight;
        x_weight = s_center_weight;
        y_weight = s_center_weight;
    }

    if(image_data_car_lamp_valid(front_lamp) != 0U)
    {
        cx = -8.902509f + 1.016973f * front_lamp->cx + 0.031254f * front_lamp->cy;
        cy = -51.990433f - 0.020303f * front_lamp->cx + 1.003250f * front_lamp->cy;
        side_x_sum += cx * s_front_x_weight;
        side_y_sum += cy * s_front_y_weight;
        side_x_weight += s_front_x_weight;
        side_y_weight += s_front_y_weight;
    }

    if(image_data_car_lamp_valid(back_lamp) != 0U)
    {
        cx = 3.636157f - 0.996270f * back_lamp->cx - 0.085060f * back_lamp->cy;
        cy = 25.907502f + 0.111016f * back_lamp->cx - 1.074126f * back_lamp->cy;
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
        g_car_lamp_fused.cx *= s_lamp_decay;
        g_car_lamp_fused.cy *= s_lamp_decay;
        return 1U;
    }

    cx = x_sum / x_weight;
    cy = y_sum / y_weight;
    if(g_car_lamp_fused.valid == 0U)
    {
        g_car_lamp_fused.valid = 1U;
        g_car_lamp_fused.cx = cx;
        g_car_lamp_fused.cy = cy;
        return 1U;
    }

    g_car_lamp_fused.cx += s_lamp_lpf_alpha * (cx - g_car_lamp_fused.cx);
    g_car_lamp_fused.cy += s_lamp_lpf_alpha * (cy - g_car_lamp_fused.cy);
    return 1U;
}
