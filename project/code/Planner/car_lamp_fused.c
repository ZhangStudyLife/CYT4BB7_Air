#include "car_lamp_fused.h"
#include "../Image/image_data.h"
#include <math.h>

static const float s_lamp_lpf_alpha = 0.500f;
static const float s_lamp_step_limit_px = 6.0f;
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
    g_car_lamp_fused.angle = 0.0f;
    g_car_lamp_fused.width = 0.0f;
    g_car_lamp_fused.length = 0.0f;
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
    uint8 valid_count = 0U;
    float x_sum = 0.0f;
    float y_sum = 0.0f;
    float angle_sum = 0.0f;
    float width_sum = 0.0f;
    float length_sum = 0.0f;
    float source_x;
    float source_y;
    float source_x2;
    float source_xy;
    float source_y2;
    float cx;
    float cy;
    float angle;
    float width;
    float length;
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
        x_sum += center_lamp->cx;
        y_sum += center_lamp->cy;
        angle_sum += center_lamp->angle;
        width_sum += center_lamp->width;
        length_sum += center_lamp->length;
        valid_count++;
    }

    if(front_valid != 0U)
    {
        source_x = front_lamp->cx;
        source_y = front_lamp->cy;
        source_x2 = source_x * source_x;
        source_xy = source_x * source_y;
        source_y2 = source_y * source_y;
        cx = -3.224193f + 1.123975f * source_x + 0.003353f * source_y +
             0.000073f * source_x2 - 0.004078f * source_xy - 0.000302f * source_y2;
        cy = -60.512112f + 0.030475f * source_x + 0.772429f * source_y +
             0.004336f * source_x2 - 0.000232f * source_xy + 0.004678f * source_y2;
        angle = 0.098951f - 0.145981f * front_lamp->angle;
        width = 3.073313f + 0.250410f * front_lamp->width;
        length = 6.702278f + 0.608019f * front_lamp->length;
        x_sum += cx;
        y_sum += cy;
        angle_sum += angle;
        width_sum += width;
        length_sum += length;
        valid_count++;
    }

    if(back_valid != 0U)
    {
        source_x = back_lamp->cx;
        source_y = back_lamp->cy;
        source_x2 = source_x * source_x;
        source_xy = source_x * source_y;
        source_y2 = source_y * source_y;
        cx = -10.828701f - 1.119896f * source_x + 0.059751f * source_y -
             0.000063f * source_x2 + 0.004186f * source_xy - 0.000850f * source_y2;
        cy = 58.428997f - 0.026951f * source_x - 0.718077f * source_y -
             0.004166f * source_x2 + 0.000106f * source_xy - 0.004593f * source_y2;
        angle = 1.206762f - 0.084711f * back_lamp->angle;
        width = 3.109584f + 0.265335f * back_lamp->width;
        length = 8.415704f + 0.525904f * back_lamp->length;
        x_sum += cx;
        y_sum += cy;
        angle_sum += angle;
        width_sum += width;
        length_sum += length;
        valid_count++;
    }

    if(valid_count == 0U)
    {
        if(g_car_lamp_fused.valid == 0U)
        {
            g_car_lamp_fused.cx = 0.0f;
            g_car_lamp_fused.cy = 0.0f;
            g_car_lamp_fused.angle = 0.0f;
            g_car_lamp_fused.width = 0.0f;
            g_car_lamp_fused.length = 0.0f;
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
        g_car_lamp_fused.angle = 0.0f;
        g_car_lamp_fused.width = 0.0f;
        g_car_lamp_fused.length = 0.0f;
        s_lamp_hold_ticks = 0U;
        return 0U;
    }

    cx = x_sum / (float)valid_count;
    cy = y_sum / (float)valid_count;
    angle = angle_sum / (float)valid_count;
    width = width_sum / (float)valid_count;
    length = length_sum / (float)valid_count;
    s_lamp_hold_ticks = 0U;
    if(g_car_lamp_fused.valid == 0U)
    {
        g_car_lamp_fused.valid = 1U;
        g_car_lamp_fused.cx = cx;
        g_car_lamp_fused.cy = cy;
        g_car_lamp_fused.angle = angle;
        g_car_lamp_fused.width = width;
        g_car_lamp_fused.length = length;
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
    g_car_lamp_fused.angle += s_lamp_lpf_alpha * (angle - g_car_lamp_fused.angle);
    g_car_lamp_fused.width += s_lamp_lpf_alpha * (width - g_car_lamp_fused.width);
    g_car_lamp_fused.length += s_lamp_lpf_alpha * (length - g_car_lamp_fused.length);
    return 1U;
}
