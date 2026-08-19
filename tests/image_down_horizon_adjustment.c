#include <math.h>
#include <stdio.h>

#include "../project/code/Image/image_down_horizon.c"

static unsigned long count_accepted_pixels(void)
{
    unsigned long count = 0UL;
    uint16 x;
    uint16 y;

    for(x = 0U; x < IMAGE_DOWN_HORIZON_WIDTH; x++)
    {
        for(y = 0U; y < IMAGE_DOWN_HORIZON_HEIGHT; y++)
        {
            count += image_down_horizon_contains((float)x, (float)y, 0.0f);
        }
    }
    return count;
}

static int columns_are_valid(void)
{
    uint16 x;

    for(x = 0U; x < IMAGE_DOWN_HORIZON_WIDTH; x++)
    {
        if(g_image_down_horizon_column_valid[x] == 0U)
        {
            continue;
        }
        if((g_image_down_horizon_top_y[x] < 0.0f) ||
           (g_image_down_horizon_bottom_y[x] >
            (float)(IMAGE_DOWN_HORIZON_HEIGHT - 1U)) ||
           (g_image_down_horizon_top_y[x] > g_image_down_horizon_bottom_y[x]) ||
           isnan(g_image_down_horizon_top_y[x]) ||
           isnan(g_image_down_horizon_bottom_y[x]))
        {
            return 0;
        }
    }
    return 1;
}

int main(void)
{
    unsigned long base_count;
    unsigned long expanded_count;
    unsigned long tightened_count;
    unsigned long lower_height_count;
    unsigned long higher_height_count;

    image_down_horizon_init();
    if(g_image_down_horizon_margin_px != -5.0f)
    {
        return 1;
    }
    g_image_down_horizon_height_offset_mm = 0.0f;
    g_image_down_horizon_margin_px = 0.0f;
    image_down_horizon_update(0.0f, 0.0f, 1000.0f, 1U, 1U);
    base_count = count_accepted_pixels();
    if((g_image_down_horizon_valid == 0U) ||
       (base_count == 0UL) || (columns_are_valid() == 0))
    {
        return 2;
    }

    g_image_down_horizon_margin_px = 10.0f;
    image_down_horizon_update(0.0f, 0.0f, 1000.0f, 1U, 1U);
    expanded_count = count_accepted_pixels();
    if((expanded_count < base_count) || (columns_are_valid() == 0))
    {
        return 3;
    }

    g_image_down_horizon_margin_px = -10.0f;
    image_down_horizon_update(0.0f, 0.0f, 1000.0f, 1U, 1U);
    tightened_count = count_accepted_pixels();
    if((tightened_count > base_count) || (columns_are_valid() == 0))
    {
        return 4;
    }

    g_image_down_horizon_margin_px = 0.0f;
    g_image_down_horizon_height_offset_mm = -100.0f;
    image_down_horizon_update(0.0f, 0.0f, 1000.0f, 1U, 1U);
    lower_height_count = count_accepted_pixels();
    g_image_down_horizon_height_offset_mm = 100.0f;
    image_down_horizon_update(0.0f, 0.0f, 1000.0f, 1U, 1U);
    higher_height_count = count_accepted_pixels();
    if((lower_height_count == higher_height_count) || (columns_are_valid() == 0))
    {
        return 5;
    }

    printf("down horizon height and margin adjustment passed\n");
    return 0;
}
