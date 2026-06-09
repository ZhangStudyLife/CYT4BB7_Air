#include "zf_common_headfile.h"

#include <math.h>

#define IMAGE_TEXT_X                    (188U)
#define IMAGE_TEXT_Y                    (8U)
#define IMAGE_TEXT_MAX_TARGETS          (3U)
#define IMAGE_MARKER_SIZE               (3)
#define IMAGE_CAR_LAMP_LINE_WIDTH       (3)

static uint16 s_display_image[MT9V03X_H][MT9V03X_W];

static int16 image_round_float_to_int16(float value)
{
    if(value >= 0.0f)
    {
        return (int16)(value + 0.5f);
    }
    return (int16)(value - 0.5f);
}

static void image_build_display_from_frame(void)
{
    uint16 row;
    uint16 col;
    uint8 *frame = image_get_frame_buffer();

    for(row = 0U; row < MT9V03X_H; row++)
    {
        for(col = 0U; col < MT9V03X_W; col++)
        {
            uint8 gray = frame[(row * MT9V03X_W) + col];
            s_display_image[row][col] = (uint16)((((uint16)gray >> 3) << 11) |
                                                 (((uint16)gray >> 2) << 5) |
                                                 ((uint16)gray >> 3));
        }
    }
}

static uint8 image_target_to_screen_point(float target_x, float target_y, int16 *screen_x, int16 *screen_y)
{
    int16 pixel_x;
    int16 pixel_y;

    pixel_x = image_round_float_to_int16(((float)MT9V03X_W * 0.5f) - target_x);
    pixel_y = image_round_float_to_int16(target_y + ((float)MT9V03X_H * 0.5f));

    if((pixel_x < 0) || (pixel_x >= (int16)MT9V03X_W) ||
       (pixel_y < 0) || (pixel_y >= (int16)MT9V03X_H))
    {
        return 0U;
    }

    *screen_x = pixel_x;
    *screen_y = pixel_y;
    return 1U;
}

static void image_overlay_brush(int16 center_x, int16 center_y, int16 size, uint16 color)
{
    int16 dx;
    int16 dy;
    int16 x;
    int16 y;

    for(dy = -(size / 2); dy <= (size / 2); dy++)
    {
        for(dx = -(size / 2); dx <= (size / 2); dx++)
        {
            x = (int16)(center_x + dx);
            y = (int16)(center_y + dy);

            if((x >= 0) && (x < (int16)MT9V03X_W) &&
               (y >= 0) && (y < (int16)MT9V03X_H))
            {
                s_display_image[(uint16)y][(uint16)x] = color;
            }
        }
    }
}

static void image_show_beacons(void)
{
    uint8 slot;
    uint8 i;
    uint16 y;
    const beacon_circle_t *beacon;
    char buf[9];
    int16 screen_x;
    int16 screen_y;

    ips114_set_font(IPS114_6X8_FONT);
    ips114_set_color(RGB565_GREEN, RGB565_BLACK);

    for(i = 0U, slot = 0U; (i < IMAGE_MAX_BEACON_COUNT) && (slot < IMAGE_TEXT_MAX_TARGETS); i++)
    {
        beacon = &g_image_beacons[i];
        if(0U == beacon->valid)
        {
            continue;
        }

        y = (uint16)(IMAGE_TEXT_Y + (slot * 24U));
        snprintf(buf, sizeof(buf), "%uR%4d ", (unsigned int)i, (int)image_round_float_to_int16(beacon->radius));
        ips114_show_string(IMAGE_TEXT_X, y, buf);

        snprintf(buf, sizeof(buf), "X%5d ", (int)image_round_float_to_int16(beacon->x));
        ips114_show_string(IMAGE_TEXT_X, (uint16)(y + 8U), buf);

        snprintf(buf, sizeof(buf), "Y%5d ", (int)image_round_float_to_int16(beacon->y));
        ips114_show_string(IMAGE_TEXT_X, (uint16)(y + 16U), buf);

        if(0U != image_target_to_screen_point(beacon->x, beacon->y, &screen_x, &screen_y))
        {
            image_overlay_brush(screen_x, screen_y, IMAGE_MARKER_SIZE, RGB565_PURPLE);
        }
        slot++;
    }

    for(; slot < IMAGE_TEXT_MAX_TARGETS; slot++)
    {
        y = (uint16)(IMAGE_TEXT_Y + (slot * 24U));
        ips114_show_string(IMAGE_TEXT_X, y, "        ");
        ips114_show_string(IMAGE_TEXT_X, (uint16)(y + 8U), "        ");
        ips114_show_string(IMAGE_TEXT_X, (uint16)(y + 16U), "        ");
    }
}

static void image_overlay_car_lamp_line(const beacon_rect_t *lamp)
{
    int16 center_x;
    int16 center_y;
    int16 step;
    int16 steps;
    int16 x;
    int16 y;
    float angle_rad;
    float half_length;
    float dir_x;
    float dir_y;

    if((0U == lamp->valid) || (0U == image_target_to_screen_point(lamp->cx, lamp->cy, &center_x, &center_y)))
    {
        return;
    }

    half_length = lamp->length * 0.5f;
    if(half_length < 1.0f)
    {
        half_length = 1.0f;
    }

    angle_rad = lamp->angle * 3.1415926f / 180.0f;
    dir_x = cosf(angle_rad);
    dir_y = sinf(angle_rad);
    steps = image_round_float_to_int16(half_length);

    for(step = (int16)-steps; step <= steps; step++)
    {
        x = image_round_float_to_int16((float)center_x + (dir_x * (float)step));
        y = image_round_float_to_int16((float)center_y + (dir_y * (float)step));

        image_overlay_brush(x, y, IMAGE_CAR_LAMP_LINE_WIDTH, RGB565_RED);
    }
}

static void image_show_debug_frame(void)
{
    uint8 i;

    image_build_display_from_frame();
    image_show_beacons();
    for(i = 0U; i < IMAGE_MAX_CAR_LAMP_COUNT; i++)
    {
        image_overlay_car_lamp_line(&g_image_car_lamps[i]);
    }
    ips114_show_rgb565_image(0U,
                             0U,
                             s_display_image[0],
                             MT9V03X_W,
                             MT9V03X_H,
                             MT9V03X_W,
                             MT9V03X_H,
                             0U);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();

    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);
    ips114_init();
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_clear();

    image_init();
    /* mt9v03x_init() re-enables DCache; keep camera/IPC shared RAM coherent. */
    SCB_DisableDCache();

    while(true)
    {
        if(mt9v03x_finish_flag)
        {
            image_update();
            ipc_image_send();
            if(0U == ipc_core0_is_flying())
            {
                image_show_debug_frame();
            }
        }
    }
}
