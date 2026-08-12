#include "car_lamp_fused.h"
#include "../Image/image_data.h"
#include <math.h>

static const uint8 s_lamp_hold_max_ticks = 40U; /* 100Hz下保持约400ms */

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

uint8 CarLampFused_Update100Hz(void)
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
    float angle_reference = 0.0f;
    float width;
    float length;
    float line_x;
    float line_y;

    only_front_see_car_lamp = raw_only_front;
    only_back_see_car_lamp = raw_only_back;

    if(center_valid != 0U)
    {
        x_sum += center_lamp->cx;
        y_sum += center_lamp->cy;
        angle_reference = center_lamp->angle;
        angle_sum += angle_reference;
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
        cx = -2.877714471f + 1.068667486f * source_x + 0.014106778f * source_y -
             0.000050497f * source_x2 - 0.002795043f * source_xy - 0.000176757f * source_y2;
        cy = -66.41345462f - 0.041888826f * source_x + 0.803140254f * source_y +
             0.004303238f * source_x2 + 0.000040255f * source_xy + 0.002781124f * source_y2;
        angle = front_lamp->angle * 0.017453293f;
        line_x = cosf(angle);
        line_y = sinf(angle);
        angle = atan2f((-0.041888826f + 0.008606475f * source_x + 0.000040255f * source_y) * line_x +
                       (0.803140254f + 0.000040255f * source_x + 0.005562248f * source_y) * line_y,
                       (1.068667486f - 0.000100995f * source_x - 0.002795043f * source_y) * line_x +
                       (0.014106778f - 0.002795043f * source_x - 0.000353513f * source_y) * line_y) * 57.29577951f;
        if(valid_count == 0U)
        {
            angle_reference = angle;
        }
        while((angle - angle_reference) > 90.0f)
        {
            angle -= 180.0f;
        }
        while((angle - angle_reference) < -90.0f)
        {
            angle += 180.0f;
        }
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
        cx = 1.001882691f - 1.067786481f * source_x - 0.076861896f * source_y +
             0.000250691f * source_x2 + 0.003736022f * source_xy + 0.000809775f * source_y2;
        cy = 49.24573601f + 0.024195958f * source_x - 0.747055821f * source_y -
             0.004815078f * source_x2 - 0.000515285f * source_xy - 0.004288377f * source_y2;
        angle = back_lamp->angle * 0.017453293f;
        line_x = cosf(angle);
        line_y = sinf(angle);
        angle = atan2f((0.024195958f - 0.009630155f * source_x - 0.000515285f * source_y) * line_x +
                       (-0.747055821f - 0.000515285f * source_x - 0.008576754f * source_y) * line_y,
                       (-1.067786481f + 0.000501382f * source_x + 0.003736022f * source_y) * line_x +
                       (-0.076861896f + 0.003736022f * source_x + 0.001619549f * source_y) * line_y) * 57.29577951f;
        if(valid_count == 0U)
        {
            angle_reference = angle;
        }
        while((angle - angle_reference) > 90.0f)
        {
            angle -= 180.0f;
        }
        while((angle - angle_reference) < -90.0f)
        {
            angle += 180.0f;
        }
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
    while(angle > 90.0f)
    {
        angle -= 180.0f;
    }
    while(angle < -90.0f)
    {
        angle += 180.0f;
    }
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

    g_car_lamp_fused.cx = cx;
    g_car_lamp_fused.cy = cy;
    g_car_lamp_fused.angle = angle;
    g_car_lamp_fused.width = width;
    g_car_lamp_fused.length = length;
    return 1U;
}
