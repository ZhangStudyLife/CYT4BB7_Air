#include "image_debug_screen.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "zf_common_headfile.h"
#include "Estimation/Pos_Est/image_down.h"
#include "IPC/ipc_image_data.h"

#define IMAGE_SCREEN_WIDTH             (188U)
#define IMAGE_SCREEN_HEIGHT            (120U)
#define IMAGE_SCREEN_BLOCK_ROWS         (8U)
#define IMAGE_SCREEN_FIRST_HALF_ROWS   (64U)
#define IMAGE_SCREEN_DATA_PERIOD_TICKS (1U)
#define IMAGE_SCREEN_PHYSICAL_WIDTH     (240U)
#define IMAGE_SCREEN_PHYSICAL_HEIGHT    (135U)
#define IMAGE_SCREEN_CLEAR_SLICE_ROWS   (64U)
#define IMAGE_SCREEN_AUX_X              (190U)
#define IMAGE_SCREEN_AUX_LINE_HEIGHT    (8U)
#define IMAGE_SCREEN_AUX_LINE_COUNT     (14U)
#define IMAGE_SCREEN_AUX_LINES_PER_TICK (4U)
#define IMAGE_SCREEN_AUX_TEXT_LENGTH    (8U)
#define IMAGE_SCREEN_PI_F               (3.1415926f)

#define IMAGE_SCREEN_X_VALUE  (32U)
#define IMAGE_SCREEN_Y_LABEL  (88U)
#define IMAGE_SCREEN_Y_VALUE  (112U)
#define IMAGE_SCREEN_A_LABEL  (160U)
#define IMAGE_SCREEN_A_VALUE  (184U)
#define IMAGE_SCREEN_ROW_H    (16U)

typedef char image_debug_screen_size_must_match_camera[
    ((MT9V03X_W == IMAGE_SCREEN_WIDTH) &&
     (MT9V03X_H == IMAGE_SCREEN_HEIGHT)) ? 1 : -1];

static volatile uint32 s_screen_tick_10ms;
static uint32 s_next_refresh_tick;
static uint8 s_screen_mode;
static uint8 s_hw_initialized;
static uint8 s_refresh_was_enabled;
static uint8 s_layout_dirty;
static uint8 s_data_layout_stage;
static uint8 s_data_update_stage;
static uint8 s_image_frame_active;
static uint8 s_image_frame_pending;
static uint8 s_image_next_row;
static uint8 s_aux_frame_active;
static uint8 s_aux_next_line;
static uint16 s_layout_clear_row;
static uint8 s_image_snapshot[IMAGE_SCREEN_HEIGHT][IMAGE_SCREEN_WIDTH];
static uint8 s_clear_block[IMAGE_SCREEN_BLOCK_ROWS][IMAGE_SCREEN_PHYSICAL_WIDTH];
static char s_aux_lines[IMAGE_SCREEN_AUX_LINE_COUNT][IMAGE_SCREEN_AUX_TEXT_LENGTH + 1U];

extern volatile uint8 g_image_tick_100hz;

/* 已消费节拍与ISR待处理节拍之和等于当前真实10ms序号。 */
static uint32 ImageDebugScreen_Now(void)
{
    uint32 consumed_tick = s_screen_tick_10ms;
    uint8 pending_tick = g_image_tick_100hz;

    return consumed_tick + pending_tick;
}

static uint8 ImageDebugScreen_RefreshAllowed(void)
{
    return ipc_core0_screen_refresh_enable();
}

static uint8 ImageDebugScreen_TaskBacklog(void)
{
    return (g_image_tick_100hz > 0U) ? 1U : 0U;
}

static uint8 ImageDebugScreen_TickDue(uint32 now, uint32 deadline)
{
    return (((int32)(now - deadline)) >= 0) ? 1U : 0U;
}

static void ImageDebugScreen_AbortImageFrame(void)
{
    s_image_frame_active = 0U;
    s_image_next_row = 0U;
}

static void ImageDebugScreen_AbortAuxFrame(void)
{
    s_aux_frame_active = 0U;
    s_aux_next_line = 0U;
}

static void ImageDebugScreen_MarkLayoutDirty(void)
{
    s_layout_dirty = 1U;
    s_layout_clear_row = 0U;
    s_data_layout_stage = 0U;
    s_data_update_stage = 0U;
    s_image_frame_pending = 0U;
    ImageDebugScreen_AbortImageFrame();
    ImageDebugScreen_AbortAuxFrame();
}

static uint8 ImageDebugScreen_ShowLamp(uint16 y, const car_lamp_data *lamp)
{
    if((ImageDebugScreen_RefreshAllowed() == 0U) ||
       (ImageDebugScreen_TaskBacklog() != 0U))
    {
        return 0U;
    }

    ips114_set_color((image_data_car_lamp_valid(lamp) != 0U) ?
                     RGB565_BLACK : RGB565_RED,
                     RGB565_WHITE);
    ips114_show_float(IMAGE_SCREEN_X_VALUE, y, lamp->cx, 3U, 1U);
    ips114_show_float(IMAGE_SCREEN_Y_VALUE, y, lamp->cy, 3U, 1U);
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    return 1U;
}

static uint8 ImageDebugScreen_ShowBeacon(uint16 y, const beacon_data *beacon)
{
    if((ImageDebugScreen_RefreshAllowed() == 0U) ||
       (ImageDebugScreen_TaskBacklog() != 0U))
    {
        return 0U;
    }

    ips114_set_color((image_data_beacon_valid(beacon) != 0U) ?
                     RGB565_BLACK : RGB565_RED,
                     RGB565_WHITE);
    ips114_show_float(IMAGE_SCREEN_X_VALUE, y, beacon->x, 3U, 1U);
    ips114_show_float(IMAGE_SCREEN_Y_VALUE, y, beacon->y, 3U, 1U);
    ips114_show_float(IMAGE_SCREEN_A_VALUE, y, beacon->area, 4U, 1U);
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    return 1U;
}

static uint8 ImageDebugScreen_DrawDataLayoutTop(void)
{
    if((ImageDebugScreen_RefreshAllowed() == 0U) ||
       (ImageDebugScreen_TaskBacklog() != 0U))
    {
        return 0U;
    }

    ips114_show_string(0U, 0U, "Beacon[0] XYA");
    ips114_show_string(0U, 16U, "F x:");
    ips114_show_string(IMAGE_SCREEN_Y_LABEL, 16U, " y:");
    ips114_show_string(IMAGE_SCREEN_A_LABEL, 16U, " a:");
    ips114_show_string(0U, 32U, "C x:");
    ips114_show_string(IMAGE_SCREEN_Y_LABEL, 32U, " y:");
    ips114_show_string(IMAGE_SCREEN_A_LABEL, 32U, " a:");
    ips114_show_string(0U, 48U, "B x:");
    ips114_show_string(IMAGE_SCREEN_Y_LABEL, 48U, " y:");
    ips114_show_string(IMAGE_SCREEN_A_LABEL, 48U, " a:");
    return 1U;
}

static uint8 ImageDebugScreen_DrawDataLayoutBottom(void)
{
    if((ImageDebugScreen_RefreshAllowed() == 0U) ||
       (ImageDebugScreen_TaskBacklog() != 0U))
    {
        return 0U;
    }

    ips114_show_string(0U, 64U, "Lamp[0] CXY");
    ips114_show_string(0U, 80U, "F x:");
    ips114_show_string(IMAGE_SCREEN_Y_LABEL, 80U, " y:");
    ips114_show_string(0U, 96U, "C x:");
    ips114_show_string(IMAGE_SCREEN_Y_LABEL, 96U, " y:");
    ips114_show_string(0U, 112U, "B x:");
    ips114_show_string(IMAGE_SCREEN_Y_LABEL, 112U, " y:");
    return 1U;
}

static uint8 ImageDebugScreen_PrepareLayout(void)
{
    uint16 end_row;
    uint16 row;
    uint16 rows;

    if((ImageDebugScreen_RefreshAllowed() == 0U) ||
       (ImageDebugScreen_TaskBacklog() != 0U))
    {
        return 0U;
    }

    ips114_set_font(IPS114_8X16_FONT);
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    if(s_layout_clear_row < IMAGE_SCREEN_PHYSICAL_HEIGHT)
    {
        end_row = s_layout_clear_row + IMAGE_SCREEN_CLEAR_SLICE_ROWS;
        if(end_row > IMAGE_SCREEN_PHYSICAL_HEIGHT)
        {
            end_row = IMAGE_SCREEN_PHYSICAL_HEIGHT;
        }

        for(row = s_layout_clear_row; row < end_row; row += rows)
        {
            if((ImageDebugScreen_RefreshAllowed() == 0U) ||
               (ImageDebugScreen_TaskBacklog() != 0U))
            {
                s_layout_clear_row = row;
                return 0U;
            }
            rows = end_row - row;
            if(rows > IMAGE_SCREEN_BLOCK_ROWS)
            {
                rows = IMAGE_SCREEN_BLOCK_ROWS;
            }
            ips114_show_gray_image(0U,
                                   row,
                                   &s_clear_block[0][0],
                                   IMAGE_SCREEN_PHYSICAL_WIDTH,
                                   rows,
                                   IMAGE_SCREEN_PHYSICAL_WIDTH,
                                   rows,
                                   0U);
        }
        s_layout_clear_row = end_row;
        if(s_layout_clear_row < IMAGE_SCREEN_PHYSICAL_HEIGHT)
        {
            return 0U;
        }
    }

    if(s_screen_mode != IMAGE_DEBUG_SCREEN_MODE_DATA)
    {
        s_layout_dirty = 0U;
        return 1U;
    }

    if(s_data_layout_stage == 0U)
    {
        if(ImageDebugScreen_DrawDataLayoutTop() == 0U)
        {
            return 0U;
        }
        s_data_layout_stage = 1U;
        return 0U;
    }
    if(ImageDebugScreen_DrawDataLayoutBottom() == 0U)
    {
        return 0U;
    }
    s_data_layout_stage = 2U;
    s_layout_dirty = 0U;
    return 1U;
}

static uint8 ImageDebugScreen_UpdateData(void)
{
    uint8 updated;

    switch(s_data_update_stage)
    {
        case 0U:
            updated = ImageDebugScreen_ShowBeacon(
                IMAGE_SCREEN_ROW_H, &image_data[Front].beacon_data[0]);
            break;
        case 1U:
            updated = ImageDebugScreen_ShowBeacon(
                2U * IMAGE_SCREEN_ROW_H, &image_data[Center].beacon_data[0]);
            break;
        case 2U:
            updated = ImageDebugScreen_ShowBeacon(
                3U * IMAGE_SCREEN_ROW_H, &image_data[Back].beacon_data[0]);
            break;
        case 3U:
            updated = ImageDebugScreen_ShowLamp(
                5U * IMAGE_SCREEN_ROW_H, &image_data[Front].car_lamp_data[0]);
            break;
        case 4U:
            updated = ImageDebugScreen_ShowLamp(
                6U * IMAGE_SCREEN_ROW_H, &image_data[Center].car_lamp_data[0]);
            break;
        case 5U:
            updated = ImageDebugScreen_ShowLamp(
                7U * IMAGE_SCREEN_ROW_H, &image_data[Back].car_lamp_data[0]);
            break;
        default:
            s_data_update_stage = 0U;
            return 0U;
    }

    if(updated == 0U)
    {
        return 0U;
    }

    s_data_update_stage++;
    if(s_data_update_stage < 6U)
    {
        return 0U;
    }

    s_data_update_stage = 0U;
    return 1U;
}

static void ImageDebugScreen_SetSnapshotPixel(int x, int y, uint8 gray)
{
    if((x >= 0) && (x < (int)IMAGE_SCREEN_WIDTH) &&
       (y >= 0) && (y < (int)IMAGE_SCREEN_HEIGHT))
    {
        s_image_snapshot[y][x] = gray;
    }
}

static void ImageDebugScreen_DrawCross(int cx, int cy)
{
    int offset;
    for(offset = -5; offset <= 5; offset++)
    {
        ImageDebugScreen_SetSnapshotPixel(cx + offset, cy - 1, 255U);
        ImageDebugScreen_SetSnapshotPixel(cx + offset, cy + 1, 255U);
        ImageDebugScreen_SetSnapshotPixel(cx - 1, cy + offset, 255U);
        ImageDebugScreen_SetSnapshotPixel(cx + 1, cy + offset, 255U);
    }
    for(offset = -5; offset <= 5; offset++)
    {
        ImageDebugScreen_SetSnapshotPixel(cx + offset, cy, 0U);
        ImageDebugScreen_SetSnapshotPixel(cx, cy + offset, 0U);
    }
}

static int ImageDebugScreen_RoundToInt(float value)
{
    return (value >= 0.0f) ? (int)(value + 0.5f) : (int)(value - 0.5f);
}

static void ImageDebugScreen_DrawSnapshotLine(int x0,
                                              int y0,
                                              int x1,
                                              int y1,
                                              uint8 gray)
{
    int dx = (x1 >= x0) ? (x1 - x0) : (x0 - x1);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = (y1 >= y0) ? (y0 - y1) : (y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int error = dx + dy;

    for(;;)
    {
        int error2;
        ImageDebugScreen_SetSnapshotPixel(x0, y0, gray);
        if((x0 == x1) && (y0 == y1))
        {
            break;
        }
        error2 = 2 * error;
        if(error2 >= dy)
        {
            error += dy;
            x0 += sx;
        }
        if(error2 <= dx)
        {
            error += dx;
            y0 += sy;
        }
    }
}

static void ImageDebugScreen_DrawRotatedRect(int cx,
                                             int cy,
                                             float half_length,
                                             float half_width,
                                             float angle_deg,
                                             uint8 gray)
{
    float angle = angle_deg * (IMAGE_SCREEN_PI_F / 180.0f);
    float major_x = cosf(angle) * half_length;
    float major_y = sinf(angle) * half_length;
    float minor_x = -sinf(angle) * half_width;
    float minor_y = cosf(angle) * half_width;
    int x[4];
    int y[4];
    uint8 corner;

    x[0] = ImageDebugScreen_RoundToInt((float)cx + major_x + minor_x);
    y[0] = ImageDebugScreen_RoundToInt((float)cy + major_y + minor_y);
    x[1] = ImageDebugScreen_RoundToInt((float)cx + major_x - minor_x);
    y[1] = ImageDebugScreen_RoundToInt((float)cy + major_y - minor_y);
    x[2] = ImageDebugScreen_RoundToInt((float)cx - major_x - minor_x);
    y[2] = ImageDebugScreen_RoundToInt((float)cy - major_y - minor_y);
    x[3] = ImageDebugScreen_RoundToInt((float)cx - major_x + minor_x);
    y[3] = ImageDebugScreen_RoundToInt((float)cy - major_y + minor_y);

    for(corner = 0U; corner < 4U; corner++)
    {
        uint8 next = (uint8)((corner + 1U) % 4U);
        ImageDebugScreen_DrawSnapshotLine(x[corner], y[corner],
                                          x[next], y[next], gray);
    }
}

static void ImageDebugScreen_ApplyOverlay(void)
{
    uint8 index;

    for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
    {
        const beacon_data *beacon = &image_data[Center].beacon_data[index];
        if(image_data_beacon_valid(beacon) != 0U)
        {
            int cx = (int)(beacon->x + (float)IMAGE_SCREEN_WIDTH * 0.5f + 0.5f);
            int cy = (int)(beacon->y + (float)IMAGE_SCREEN_HEIGHT * 0.5f + 0.5f);
            ImageDebugScreen_DrawCross(cx, cy);
        }
    }

    for(index = 0U; index < IMAGE_MAX_CAR_LAMP_COUNT; index++)
    {
        const car_lamp_data *lamp = &image_data[Center].car_lamp_data[index];
        if(image_data_car_lamp_valid(lamp) != 0U)
        {
            int cx = (int)(lamp->cx + (float)IMAGE_SCREEN_WIDTH * 0.5f + 0.5f);
            int cy = (int)(lamp->cy + (float)IMAGE_SCREEN_HEIGHT * 0.5f + 0.5f);
            float half_length = lamp->length * 0.5f;
            float half_width = lamp->width * 0.5f;
            if(half_length < 3.0f) half_length = 3.0f;
            if(half_width < 2.0f) half_width = 2.0f;
            if(half_length > 40.0f) half_length = 40.0f;
            if(half_width > 20.0f) half_width = 20.0f;
            ImageDebugScreen_DrawRotatedRect(cx, cy,
                                             half_length + 1.0f,
                                             half_width + 1.0f,
                                             lamp->angle, 255U);
            ImageDebugScreen_DrawRotatedRect(cx, cy,
                                             half_length,
                                             half_width,
                                             lamp->angle, 0U);
        }
    }
}

static void ImageDebugScreen_SetAuxLine(uint8 line, const char *text)
{
    size_t length;
    if((line >= IMAGE_SCREEN_AUX_LINE_COUNT) || (text == NULL))
    {
        return;
    }
    memset(s_aux_lines[line], ' ', IMAGE_SCREEN_AUX_TEXT_LENGTH);
    length = strlen(text);
    if(length > IMAGE_SCREEN_AUX_TEXT_LENGTH)
    {
        length = IMAGE_SCREEN_AUX_TEXT_LENGTH;
    }
    memcpy(s_aux_lines[line], text, length);
    s_aux_lines[line][IMAGE_SCREEN_AUX_TEXT_LENGTH] = '\0';
}

static const char *ImageDebugScreen_ModeName(uint8 mode)
{
    switch(mode)
    {
        case IMAGE_DEBUG_SCREEN_MODE_RAW:           return "M:RAW";
        case IMAGE_DEBUG_SCREEN_MODE_BEACON_BINARY: return "M:B-BIN";
        case IMAGE_DEBUG_SCREEN_MODE_LAMP_BINARY:   return "M:L-BIN";
        case IMAGE_DEBUG_SCREEN_MODE_OVERLAY:       return "M:OVER";
        default:                                    return "M:DATA";
    }
}

static void ImageDebugScreen_CaptureAuxSnapshot(void)
{
    const beacon_data *first_beacon = NULL;
    const car_lamp_data *first_lamp = NULL;
    uint8 beacon_count = 0U;
    uint8 lamp_count = 0U;
    uint8 index;
    char text[16];

    for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
    {
        const beacon_data *beacon = &image_data[Center].beacon_data[index];
        if(image_data_beacon_valid(beacon) != 0U)
        {
            if(first_beacon == NULL) first_beacon = beacon;
            beacon_count++;
        }
    }
    for(index = 0U; index < IMAGE_MAX_CAR_LAMP_COUNT; index++)
    {
        const car_lamp_data *lamp = &image_data[Center].car_lamp_data[index];
        if(image_data_car_lamp_valid(lamp) != 0U)
        {
            if(first_lamp == NULL) first_lamp = lamp;
            lamp_count++;
        }
    }

    ImageDebugScreen_SetAuxLine(0U, ImageDebugScreen_ModeName(s_screen_mode));
    (void)snprintf(text, sizeof(text), "BT:%ld",
                   (long)g_image_down_beacon_binary_threshold);
    ImageDebugScreen_SetAuxLine(1U, text);
    (void)snprintf(text, sizeof(text), "LT:%ld",
                   (long)g_image_down_car_lamp_binary_threshold);
    ImageDebugScreen_SetAuxLine(2U, text);
    (void)snprintf(text, sizeof(text), "EX:%u", (unsigned int)g_mt9v03x_exp_time);
    ImageDebugScreen_SetAuxLine(3U, text);
    (void)snprintf(text, sizeof(text), "B#:%u", (unsigned int)beacon_count);
    ImageDebugScreen_SetAuxLine(4U, text);
    if(first_beacon != NULL)
    {
        (void)snprintf(text, sizeof(text), "BX:%+d",
                       ImageDebugScreen_RoundToInt(first_beacon->x));
        ImageDebugScreen_SetAuxLine(5U, text);
        (void)snprintf(text, sizeof(text), "BY:%+d",
                       ImageDebugScreen_RoundToInt(first_beacon->y));
        ImageDebugScreen_SetAuxLine(6U, text);
        (void)snprintf(text, sizeof(text), "BA:%d",
                       ImageDebugScreen_RoundToInt(first_beacon->area));
        ImageDebugScreen_SetAuxLine(7U, text);
    }
    else
    {
        ImageDebugScreen_SetAuxLine(5U, "BX:--");
        ImageDebugScreen_SetAuxLine(6U, "BY:--");
        ImageDebugScreen_SetAuxLine(7U, "BA:--");
    }
    (void)snprintf(text, sizeof(text), "L#:%u", (unsigned int)lamp_count);
    ImageDebugScreen_SetAuxLine(8U, text);
    if(first_lamp != NULL)
    {
        (void)snprintf(text, sizeof(text), "LX:%+d",
                       ImageDebugScreen_RoundToInt(first_lamp->cx));
        ImageDebugScreen_SetAuxLine(9U, text);
        (void)snprintf(text, sizeof(text), "LY:%+d",
                       ImageDebugScreen_RoundToInt(first_lamp->cy));
        ImageDebugScreen_SetAuxLine(10U, text);
        (void)snprintf(text, sizeof(text), "LA:%+d",
                       ImageDebugScreen_RoundToInt(first_lamp->angle));
        ImageDebugScreen_SetAuxLine(11U, text);
        (void)snprintf(text, sizeof(text), "LW:%d",
                       ImageDebugScreen_RoundToInt(first_lamp->width));
        ImageDebugScreen_SetAuxLine(12U, text);
        (void)snprintf(text, sizeof(text), "LL:%d",
                       ImageDebugScreen_RoundToInt(first_lamp->length));
        ImageDebugScreen_SetAuxLine(13U, text);
    }
    else
    {
        ImageDebugScreen_SetAuxLine(9U, "LX:--");
        ImageDebugScreen_SetAuxLine(10U, "LY:--");
        ImageDebugScreen_SetAuxLine(11U, "LA:--");
        ImageDebugScreen_SetAuxLine(12U, "LW:--");
        ImageDebugScreen_SetAuxLine(13U, "LL:--");
    }
    s_aux_frame_active = 1U;
    s_aux_next_line = 0U;
}

static uint8 ImageDebugScreen_UpdateAuxPanel(void)
{
    uint8 end_line;
    uint8 line;

    if((ImageDebugScreen_RefreshAllowed() == 0U) ||
       (ImageDebugScreen_TaskBacklog() != 0U))
    {
        return 0U;
    }

    ips114_set_font(IPS114_6X8_FONT);
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    if(s_aux_next_line == 0U)
    {
        ips114_draw_line(IMAGE_SCREEN_WIDTH, 0U,
                         IMAGE_SCREEN_WIDTH, IMAGE_SCREEN_HEIGHT - 1U,
                         RGB565_BLACK);
    }
    end_line = (uint8)(s_aux_next_line + IMAGE_SCREEN_AUX_LINES_PER_TICK);
    if(end_line > IMAGE_SCREEN_AUX_LINE_COUNT)
    {
        end_line = IMAGE_SCREEN_AUX_LINE_COUNT;
    }
    for(line = s_aux_next_line; line < end_line; line++)
    {
        ips114_show_string(IMAGE_SCREEN_AUX_X,
                           (uint16)line * IMAGE_SCREEN_AUX_LINE_HEIGHT,
                           s_aux_lines[line]);
    }
    s_aux_next_line = end_line;
    if(s_aux_next_line >= IMAGE_SCREEN_AUX_LINE_COUNT)
    {
        ImageDebugScreen_AbortAuxFrame();
    }
    return 1U;
}

static uint8 ImageDebugScreen_StartImageFrame(void)
{
    const uint8 *source;

    switch(s_screen_mode)
    {
        case IMAGE_DEBUG_SCREEN_MODE_BEACON_BINARY:
            source = image_down_get_binary_buffer();
            break;
        case IMAGE_DEBUG_SCREEN_MODE_LAMP_BINARY:
            source = image_down_get_car_lamp_binary_buffer();
            break;
        case IMAGE_DEBUG_SCREEN_MODE_OVERLAY:
        case IMAGE_DEBUG_SCREEN_MODE_RAW:
        default:
            source = image_down_get_frame_buffer();
            break;
    }
    if(source == NULL)
    {
        return 0U;
    }

    memcpy(s_image_snapshot[0], source, sizeof(s_image_snapshot));
    if(s_screen_mode == IMAGE_DEBUG_SCREEN_MODE_OVERLAY)
    {
        ImageDebugScreen_ApplyOverlay();
    }
    ImageDebugScreen_CaptureAuxSnapshot();
    s_image_frame_active = 1U;
    s_image_next_row = 0U;
    return 1U;
}

static uint8 ImageDebugScreen_DrawImageHalf(void)
{
    uint8 end_row;
    uint8 row;
    uint8 rows;
    uint8 threshold;

    end_row = (s_image_next_row == 0U) ?
              IMAGE_SCREEN_FIRST_HALF_ROWS : IMAGE_SCREEN_HEIGHT;
    threshold = ((s_screen_mode == IMAGE_DEBUG_SCREEN_MODE_BEACON_BINARY) ||
                 (s_screen_mode == IMAGE_DEBUG_SCREEN_MODE_LAMP_BINARY)) ? 1U : 0U;
    for(row = s_image_next_row; row < end_row; row = (uint8)(row + rows))
    {
        if(ImageDebugScreen_RefreshAllowed() == 0U)
        {
            return 0U;
        }

        rows = (uint8)(end_row - row);
        if(rows > IMAGE_SCREEN_BLOCK_ROWS)
        {
            rows = IMAGE_SCREEN_BLOCK_ROWS;
        }
        ips114_show_gray_image(0U,
                               row,
                               &s_image_snapshot[row][0],
                               IMAGE_SCREEN_WIDTH,
                               rows,
                               IMAGE_SCREEN_WIDTH,
                               rows,
                               threshold);
    }

    s_image_next_row = end_row;
    if(s_image_next_row >= IMAGE_SCREEN_HEIGHT)
    {
        ImageDebugScreen_AbortImageFrame();
    }
    return ImageDebugScreen_RefreshAllowed();
}

void ImageDebugScreen_Init(void)
{
    s_screen_tick_10ms = 0U;
    s_next_refresh_tick = 0U;
    s_screen_mode = IMAGE_DEBUG_SCREEN_MODE_DATA;
    s_hw_initialized = 0U;
    s_refresh_was_enabled = 0U;
    s_image_frame_pending = 0U;
    s_aux_frame_active = 0U;
    s_aux_next_line = 0U;
    memset(s_clear_block, 0xFF, sizeof(s_clear_block));
    ImageDebugScreen_MarkLayoutDirty();
}

void ImageDebugScreen_Tick10ms(void)
{
    s_screen_tick_10ms++;
}

uint8 ImageDebugScreen_SetMode(uint8 mode)
{
    if(mode > IMAGE_DEBUG_SCREEN_MODE_OVERLAY)
    {
        return 0U;
    }
    if(mode != s_screen_mode)
    {
        s_screen_mode = mode;
        ImageDebugScreen_MarkLayoutDirty();
        s_next_refresh_tick = ImageDebugScreen_Now();
    }
    return 1U;
}

uint8 ImageDebugScreen_GetMode(void)
{
    return s_screen_mode;
}

void ImageDebugScreen_Update(uint8 image_frame_updated)
{
    uint32 now = ImageDebugScreen_Now();

    if(ImageDebugScreen_RefreshAllowed() == 0U)
    {
        s_refresh_was_enabled = 0U;
        s_image_frame_pending = 0U;
        ImageDebugScreen_AbortImageFrame();
        ImageDebugScreen_AbortAuxFrame();
        return;
    }

    if(s_hw_initialized == 0U)
    {
        ips114_set_dir(IPS114_PORTAIT);
        ips114_init();
        s_hw_initialized = 1U;
        ImageDebugScreen_MarkLayoutDirty();
    }

    if(s_refresh_was_enabled == 0U)
    {
        s_refresh_was_enabled = 1U;
        ImageDebugScreen_MarkLayoutDirty();
        s_next_refresh_tick = now;
    }

    if((s_screen_mode != IMAGE_DEBUG_SCREEN_MODE_DATA) &&
       (image_frame_updated != 0U))
    {
        s_image_frame_pending = 1U;
    }

    if(s_layout_dirty != 0U)
    {
        if(ImageDebugScreen_TickDue(now, s_next_refresh_tick) == 0U)
        {
            return;
        }
        if(ImageDebugScreen_PrepareLayout() == 0U)
        {
            if(ImageDebugScreen_RefreshAllowed() == 0U)
            {
                s_refresh_was_enabled = 0U;
            }
            s_next_refresh_tick = ImageDebugScreen_Now() + 1U;
            return;
        }
        s_next_refresh_tick = ImageDebugScreen_Now() + 1U;
        return;
    }

    if(s_screen_mode == IMAGE_DEBUG_SCREEN_MODE_DATA)
    {
        if((ImageDebugScreen_TaskBacklog() != 0U) ||
           (ImageDebugScreen_TickDue(now, s_next_refresh_tick) == 0U))
        {
            return;
        }
        if(ImageDebugScreen_UpdateData() == 0U)
        {
            if(ImageDebugScreen_RefreshAllowed() == 0U)
            {
                s_refresh_was_enabled = 0U;
            }
            s_next_refresh_tick = ImageDebugScreen_Now() + 1U;
            return;
        }
        s_next_refresh_tick =
            ImageDebugScreen_Now() + IMAGE_SCREEN_DATA_PERIOD_TICKS;
        return;
    }

    if((s_image_frame_active == 0U) && (s_aux_frame_active != 0U))
    {
        if((ImageDebugScreen_TickDue(now, s_next_refresh_tick) == 0U) ||
           (ImageDebugScreen_UpdateAuxPanel() == 0U))
        {
            return;
        }
        s_next_refresh_tick = ImageDebugScreen_Now() + 1U;
        return;
    }

    if(s_image_frame_active == 0U)
    {
        if((s_image_frame_pending == 0U) ||
           (ImageDebugScreen_TaskBacklog() != 0U))
        {
            return;
        }
        if(ImageDebugScreen_StartImageFrame() == 0U)
        {
            return;
        }
        s_image_frame_pending = 0U;
    }
    if(ImageDebugScreen_DrawImageHalf() == 0U)
    {
        s_refresh_was_enabled = 0U;
        s_image_frame_pending = 0U;
        ImageDebugScreen_AbortImageFrame();
        ImageDebugScreen_AbortAuxFrame();
    }

}
