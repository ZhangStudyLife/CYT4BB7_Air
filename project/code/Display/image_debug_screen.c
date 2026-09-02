/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#include "image_debug_screen.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "zf_common_headfile.h"
#include "Estimation/Pos_Est/image_down.h"
#include "Image/image_down_horizon.h"
#include "IPC/ipc_image_data.h"

#define IMAGE_SCREEN_WIDTH             (188U)
#define IMAGE_SCREEN_HEIGHT            (120U)
#define IMAGE_SCREEN_PHYSICAL_WIDTH    (240U)
#define IMAGE_SCREEN_DATA_ROW_HEIGHT   (16U)
#define IMAGE_SCREEN_DATA_PERIOD_TICKS (1U)
#define IMAGE_SCREEN_SPI_SPEED         (20U * 1000U * 1000U)
#define IMAGE_SCREEN_AUX_REFRESH_FRAMES (5U)
#define IMAGE_SCREEN_FORCE_DISABLE     (0U) /* 仅安全待机时按核0许可初始化和刷新。 */
#define IMAGE_SCREEN_LCD_X_OFFSET       (40U)
#define IMAGE_SCREEN_LCD_Y_OFFSET       (52U)
#define IMAGE_SCREEN_AUX_X              (190U)
#define IMAGE_SCREEN_AUX_LINE_HEIGHT    (8U)
#define IMAGE_SCREEN_AUX_LINE_COUNT     (14U)
#define IMAGE_SCREEN_AUX_TEXT_LENGTH    (8U)
#define IMAGE_SCREEN_PI_F               (3.1415926f)

#define IMAGE_SCREEN_BEACON_COLOR       (RGB565_RED)
#define IMAGE_SCREEN_LAMP_COLOR         (RGB565_YELLOW)
#define IMAGE_SCREEN_HORIZON_COLOR      (RGB565_GREEN)
#define IMAGE_SCREEN_HORIZON_EXT_COLOR  (RGB565_CYAN)

#define IMAGE_SCREEN_CLIP_LEFT          (0x01U)
#define IMAGE_SCREEN_CLIP_RIGHT         (0x02U)
#define IMAGE_SCREEN_CLIP_TOP           (0x04U)
#define IMAGE_SCREEN_CLIP_BOTTOM        (0x08U)

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
static uint8 s_startup_layout_ready;
static uint8 s_layout_dirty;
static uint8 s_image_frame_pending;
static uint8 s_aux_refresh_counter;
static uint16 s_image_rgb565[IMAGE_SCREEN_HEIGHT][IMAGE_SCREEN_WIDTH];
static char s_aux_lines[IMAGE_SCREEN_AUX_LINE_COUNT][IMAGE_SCREEN_AUX_TEXT_LENGTH + 1U];

extern volatile uint8 g_image_tick_100hz;

static int ImageDebugScreen_RoundToInt(float value);
static void ImageDebugScreen_ShowRgb565Region(uint16 x,
                                              uint16 y,
                                              uint16 width,
                                              uint16 height,
                                              const uint16 *pixels);

/* 已消费节拍与ISR待处理节拍之和等于当前真实10ms序号。 */
static uint32 ImageDebugScreen_Now(void)
{
    uint32 consumed_tick = s_screen_tick_10ms;
    uint8 pending_tick = g_image_tick_100hz;

    return consumed_tick + pending_tick;
}

static uint8 ImageDebugScreen_RefreshAllowed(void)
{
#if (IMAGE_SCREEN_FORCE_DISABLE != 0U)
    return 0U;
#else
    return ipc_core0_screen_refresh_enable();
#endif
}

static uint8 ImageDebugScreen_TaskBacklog(void)
{
    return (g_image_tick_100hz > 0U) ? 1U : 0U;
}

static uint8 ImageDebugScreen_TickDue(uint32 now, uint32 deadline)
{
    return (((int32)(now - deadline)) >= 0) ? 1U : 0U;
}

static void ImageDebugScreen_MarkLayoutDirty(void)
{
    s_startup_layout_ready = 0U;
    s_layout_dirty = 1U;
    s_image_frame_pending = 0U;
    s_aux_refresh_counter = 0U;
}

static void ImageDebugScreen_RenderDataLayoutTop(void)
{
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
}

static void ImageDebugScreen_RenderDataLayoutBottom(void)
{
    ips114_show_string(0U, 64U, "Lamp[0] CXY");
    ips114_show_string(0U, 80U, "F x:");
    ips114_show_string(IMAGE_SCREEN_Y_LABEL, 80U, " y:");
    ips114_show_string(0U, 96U, "C x:");
    ips114_show_string(IMAGE_SCREEN_Y_LABEL, 96U, " y:");
    ips114_show_string(0U, 112U, "B x:");
    ips114_show_string(IMAGE_SCREEN_Y_LABEL, 112U, " y:");
}

static uint8 ImageDebugScreen_PrepareLayout(void)
{
    if((ImageDebugScreen_RefreshAllowed() == 0U) ||
       (ImageDebugScreen_TaskBacklog() != 0U))
    {
        return 0U;
    }

    ips114_set_font(IPS114_8X16_FONT);
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    ips114_clear();
    if(s_screen_mode == IMAGE_DEBUG_SCREEN_MODE_DATA)
    {
        ImageDebugScreen_RenderDataLayoutTop();
        ImageDebugScreen_RenderDataLayoutBottom();
    }
    s_layout_dirty = 0U;
    return 1U;
}

static void ImageDebugScreen_DrawText8x16(uint16 *row_buffer,
                                          uint16 x,
                                          const char *text,
                                          uint16 color)
{
    while((*text != '\0') && (x + 7U < IMAGE_SCREEN_PHYSICAL_WIDTH))
    {
        uint8 character = (uint8)*text;
        uint8 column;

        if((character < 32U) || (character > 126U))
        {
            character = (uint8)'?';
        }
        for(column = 0U; column < 8U; column++)
        {
            uint8 top = ascii_font_8x16[character - 32U][column];
            uint8 bottom = ascii_font_8x16[character - 32U][column + 8U];
            uint8 row;

            for(row = 0U; row < 8U; row++)
            {
                if((top & (uint8)(1U << row)) != 0U)
                {
                    row_buffer[(uint16)row * IMAGE_SCREEN_PHYSICAL_WIDTH +
                               x + column] = color;
                }
                if((bottom & (uint8)(1U << row)) != 0U)
                {
                    row_buffer[(uint16)(row + 8U) *
                               IMAGE_SCREEN_PHYSICAL_WIDTH +
                               x + column] = color;
                }
            }
        }
        x += 8U;
        text++;
    }
}

static void ImageDebugScreen_FormatFixed1(char *text,
                                          size_t text_size,
                                          float value)
{
    int scaled = ImageDebugScreen_RoundToInt(value * 10.0f);
    unsigned int magnitude;

    if(scaled > 9999) scaled = 9999;
    else if(scaled < -9999) scaled = -9999;
    magnitude = (unsigned int)((scaled < 0) ? -scaled : scaled);
    (void)snprintf(text,
                   text_size,
                   "%c%u.%u",
                   (scaled < 0) ? '-' : '+',
                   magnitude / 10U,
                   magnitude % 10U);
}

static void ImageDebugScreen_PrepareDataRow(uint16 *row_buffer)
{
    uint32 pixel;

    for(pixel = 0U;
        pixel < IMAGE_SCREEN_PHYSICAL_WIDTH * IMAGE_SCREEN_DATA_ROW_HEIGHT;
        pixel++)
    {
        row_buffer[pixel] = RGB565_WHITE;
    }
}

static void ImageDebugScreen_ShowBeaconRow(uint16 y,
                                           char camera,
                                           const beacon_data *beacon)
{
    uint16 *row_buffer = &s_image_rgb565[0][0];
    uint16 value_color = (image_data_beacon_valid(beacon) != 0U) ?
                         RGB565_BLACK : RGB565_RED;
    char label[5] = {camera, ' ', 'x', ':', '\0'};
    char value[16];

    ImageDebugScreen_PrepareDataRow(row_buffer);
    ImageDebugScreen_DrawText8x16(row_buffer, 0U, label, RGB565_BLACK);
    ImageDebugScreen_FormatFixed1(value, sizeof(value), beacon->x);
    ImageDebugScreen_DrawText8x16(row_buffer,
                                  IMAGE_SCREEN_X_VALUE,
                                  value,
                                  value_color);
    ImageDebugScreen_DrawText8x16(row_buffer,
                                  IMAGE_SCREEN_Y_LABEL,
                                  " y:",
                                  RGB565_BLACK);
    ImageDebugScreen_FormatFixed1(value, sizeof(value), beacon->y);
    ImageDebugScreen_DrawText8x16(row_buffer,
                                  IMAGE_SCREEN_Y_VALUE,
                                  value,
                                  value_color);
    ImageDebugScreen_DrawText8x16(row_buffer,
                                  IMAGE_SCREEN_A_LABEL,
                                  " a:",
                                  RGB565_BLACK);
    (void)snprintf(value,
                   sizeof(value),
                   "%d",
                   ImageDebugScreen_RoundToInt(beacon->area));
    ImageDebugScreen_DrawText8x16(row_buffer,
                                  IMAGE_SCREEN_A_VALUE,
                                  value,
                                  value_color);
    ImageDebugScreen_ShowRgb565Region(0U,
                                      y,
                                      IMAGE_SCREEN_PHYSICAL_WIDTH,
                                      IMAGE_SCREEN_DATA_ROW_HEIGHT,
                                      row_buffer);
}

static void ImageDebugScreen_ShowLampRow(uint16 y,
                                         char camera,
                                         const car_lamp_data *lamp)
{
    uint16 *row_buffer = &s_image_rgb565[0][0];
    uint16 value_color = (image_data_car_lamp_valid(lamp) != 0U) ?
                         RGB565_BLACK : RGB565_RED;
    char label[5] = {camera, ' ', 'x', ':', '\0'};
    char value[16];

    ImageDebugScreen_PrepareDataRow(row_buffer);
    ImageDebugScreen_DrawText8x16(row_buffer, 0U, label, RGB565_BLACK);
    ImageDebugScreen_FormatFixed1(value, sizeof(value), lamp->cx);
    ImageDebugScreen_DrawText8x16(row_buffer,
                                  IMAGE_SCREEN_X_VALUE,
                                  value,
                                  value_color);
    ImageDebugScreen_DrawText8x16(row_buffer,
                                  IMAGE_SCREEN_Y_LABEL,
                                  " y:",
                                  RGB565_BLACK);
    ImageDebugScreen_FormatFixed1(value, sizeof(value), lamp->cy);
    ImageDebugScreen_DrawText8x16(row_buffer,
                                  IMAGE_SCREEN_Y_VALUE,
                                  value,
                                  value_color);
    ImageDebugScreen_ShowRgb565Region(0U,
                                      y,
                                      IMAGE_SCREEN_PHYSICAL_WIDTH,
                                      IMAGE_SCREEN_DATA_ROW_HEIGHT,
                                      row_buffer);
}

static void ImageDebugScreen_UpdateData(void)
{
    static uint8 row_index;

    switch(row_index)
    {
        case 0U:
            ImageDebugScreen_ShowBeaconRow(
                IMAGE_SCREEN_ROW_H, 'F', &image_data[Front].beacon_data[0]);
            break;
        case 1U:
            ImageDebugScreen_ShowBeaconRow(
                2U * IMAGE_SCREEN_ROW_H, 'C', &image_data[Center].beacon_data[0]);
            break;
        case 2U:
            ImageDebugScreen_ShowBeaconRow(
                3U * IMAGE_SCREEN_ROW_H, 'B', &image_data[Back].beacon_data[0]);
            break;
        case 3U:
            ImageDebugScreen_ShowLampRow(
                5U * IMAGE_SCREEN_ROW_H, 'F', &image_data[Front].car_lamp_data[0]);
            break;
        case 4U:
            ImageDebugScreen_ShowLampRow(
                6U * IMAGE_SCREEN_ROW_H, 'C', &image_data[Center].car_lamp_data[0]);
            break;
        default:
            ImageDebugScreen_ShowLampRow(
                7U * IMAGE_SCREEN_ROW_H, 'B', &image_data[Back].car_lamp_data[0]);
            break;
    }

    row_index++;
    if(row_index >= 6U)
    {
        row_index = 0U;
    }
}

static void ImageDebugScreen_SetSnapshotPixel(int x, int y, uint16 color)
{
    if((x >= 0) && (x < (int)IMAGE_SCREEN_WIDTH) &&
       (y >= 0) && (y < (int)IMAGE_SCREEN_HEIGHT))
    {
        s_image_rgb565[y][x] = color;
    }
}

static void ImageDebugScreen_DrawCross(int cx, int cy, uint16 color)
{
    int offset;
    for(offset = -5; offset <= 5; offset++)
    {
        ImageDebugScreen_SetSnapshotPixel(cx + offset, cy - 1, RGB565_WHITE);
        ImageDebugScreen_SetSnapshotPixel(cx + offset, cy + 1, RGB565_WHITE);
        ImageDebugScreen_SetSnapshotPixel(cx - 1, cy + offset, RGB565_WHITE);
        ImageDebugScreen_SetSnapshotPixel(cx + 1, cy + offset, RGB565_WHITE);
    }
    for(offset = -5; offset <= 5; offset++)
    {
        ImageDebugScreen_SetSnapshotPixel(cx + offset, cy, color);
        ImageDebugScreen_SetSnapshotPixel(cx, cy + offset, color);
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
                                              uint16 color)
{
    int dx = (x1 >= x0) ? (x1 - x0) : (x0 - x1);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = (y1 >= y0) ? (y0 - y1) : (y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int error = dx + dy;

    for(;;)
    {
        int error2;
        ImageDebugScreen_SetSnapshotPixel(x0, y0, color);
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
                                             uint16 color)
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
                                          x[next], y[next], color);
    }
}

static uint8 ImageDebugScreen_LineOutCode(float x, float y)
{
    uint8 code = 0U;

    if(x < 0.0f) code |= IMAGE_SCREEN_CLIP_LEFT;
    else if(x > (float)(IMAGE_SCREEN_WIDTH - 1U)) code |= IMAGE_SCREEN_CLIP_RIGHT;
    if(y < 0.0f) code |= IMAGE_SCREEN_CLIP_TOP;
    else if(y > (float)(IMAGE_SCREEN_HEIGHT - 1U)) code |= IMAGE_SCREEN_CLIP_BOTTOM;
    return code;
}

static uint8 ImageDebugScreen_ClipLine(float *x0,
                                       float *y0,
                                       float *x1,
                                       float *y1)
{
    uint8 code0 = ImageDebugScreen_LineOutCode(*x0, *y0);
    uint8 code1 = ImageDebugScreen_LineOutCode(*x1, *y1);

    for(;;)
    {
        uint8 outside;
        float x;
        float y;

        if((code0 | code1) == 0U)
        {
            return 1U;
        }
        if((code0 & code1) != 0U)
        {
            return 0U;
        }

        outside = (code0 != 0U) ? code0 : code1;
        if((outside & IMAGE_SCREEN_CLIP_BOTTOM) != 0U)
        {
            y = (float)(IMAGE_SCREEN_HEIGHT - 1U);
            x = *x0 + (*x1 - *x0) * (y - *y0) / (*y1 - *y0);
        }
        else if((outside & IMAGE_SCREEN_CLIP_TOP) != 0U)
        {
            y = 0.0f;
            x = *x0 + (*x1 - *x0) * (y - *y0) / (*y1 - *y0);
        }
        else if((outside & IMAGE_SCREEN_CLIP_RIGHT) != 0U)
        {
            x = (float)(IMAGE_SCREEN_WIDTH - 1U);
            y = *y0 + (*y1 - *y0) * (x - *x0) / (*x1 - *x0);
        }
        else
        {
            x = 0.0f;
            y = *y0 + (*y1 - *y0) * (x - *x0) / (*x1 - *x0);
        }

        if(outside == code0)
        {
            *x0 = x;
            *y0 = y;
            code0 = ImageDebugScreen_LineOutCode(*x0, *y0);
        }
        else
        {
            *x1 = x;
            *y1 = y;
            code1 = ImageDebugScreen_LineOutCode(*x1, *y1);
        }
    }
}

static void ImageDebugScreen_DrawHorizonBranch(const float *y_values,
                                                uint16 color)
{
    float previous_x = 0.0f;
    float previous_y = 0.0f;
    uint8 previous_valid = 0U;
    uint16 x;

    for(x = 0U; x < IMAGE_DOWN_HORIZON_WIDTH; x++)
    {
        float current_x;
        float current_y;

        if(g_image_down_horizon_column_valid[x] == 0U)
        {
            previous_valid = 0U;
            continue;
        }
        current_x = (float)x;
        current_y = y_values[x];
        if(previous_valid != 0U)
        {
            float x0 = previous_x;
            float y0 = previous_y;
            float x1 = current_x;
            float y1 = current_y;

            if(ImageDebugScreen_ClipLine(&x0, &y0, &x1, &y1) != 0U)
            {
                ImageDebugScreen_DrawSnapshotLine(
                    ImageDebugScreen_RoundToInt(x0),
                    ImageDebugScreen_RoundToInt(y0),
                    ImageDebugScreen_RoundToInt(x1),
                    ImageDebugScreen_RoundToInt(y1),
                    color);
            }
        }
        previous_x = current_x;
        previous_y = current_y;
        previous_valid = 1U;
    }
}

static void ImageDebugScreen_DrawHorizon(void)
{
    uint16 color;

    if(g_image_down_horizon_valid == 0U)
    {
        return;
    }
    color = (g_image_down_horizon_extrapolated != 0U) ?
            IMAGE_SCREEN_HORIZON_EXT_COLOR : IMAGE_SCREEN_HORIZON_COLOR;
    ImageDebugScreen_DrawHorizonBranch(g_image_down_horizon_top_y, color);
    ImageDebugScreen_DrawHorizonBranch(g_image_down_horizon_bottom_y, color);
}

static void ImageDebugScreen_ApplyOverlay(void)
{
    uint8 index;

    ImageDebugScreen_DrawHorizon();

    for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
    {
        const beacon_data *beacon = &image_data[Center].beacon_data[index];
        if(image_data_beacon_valid(beacon) != 0U)
        {
            int cx = (int)(beacon->x + (float)IMAGE_SCREEN_WIDTH * 0.5f + 0.5f);
            int cy = (int)(beacon->y + (float)IMAGE_SCREEN_HEIGHT * 0.5f + 0.5f);
            ImageDebugScreen_DrawCross(cx, cy, IMAGE_SCREEN_BEACON_COLOR);
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
                                             lamp->angle, RGB565_WHITE);
            ImageDebugScreen_DrawRotatedRect(cx, cy,
                                             half_length,
                                             half_width,
                                             lamp->angle,
                                             IMAGE_SCREEN_LAMP_COLOR);
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
    (void)snprintf(text, sizeof(text), "EP:%ld",
                   (long)g_image_down_gray_edge_min_peak);
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
}

static void ImageDebugScreen_DrawAuxPanel(void)
{
    uint8 line;

    ips114_set_font(IPS114_6X8_FONT);
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    ips114_draw_line(IMAGE_SCREEN_WIDTH, 0U,
                     IMAGE_SCREEN_WIDTH, IMAGE_SCREEN_HEIGHT - 1U,
                     RGB565_BLACK);
    for(line = 0U; line < IMAGE_SCREEN_AUX_LINE_COUNT; line++)
    {
        ips114_show_string(IMAGE_SCREEN_AUX_X,
                           (uint16)line * IMAGE_SCREEN_AUX_LINE_HEIGHT,
                           s_aux_lines[line]);
    }
}

static uint16 ImageDebugScreen_GrayToRgb565(uint8 gray)
{
    return (uint16)((((uint16)gray & 0x00F8U) << 8) |
                    (((uint16)gray & 0x00FCU) << 3) |
                    (((uint16)gray & 0x00F8U) >> 3));
}

static void ImageDebugScreen_ConfigureSpi(void)
{
    spi_init(IPS114_SPI,
             SPI_MODE0,
             IMAGE_SCREEN_SPI_SPEED,
             IPS114_SCL_PIN,
             IPS114_SDA_PIN,
             IPS114_SDA_IN_PIN,
             SPI_CS_NULL);
}

static void ImageDebugScreen_WriteCommand(uint8 command)
{
    IPS114_DC(0);
    spi_write_8bit(IPS114_SPI, command);
    IPS114_DC(1);
}

static void ImageDebugScreen_ShowRgb565Region(uint16 x,
                                              uint16 y,
                                              uint16 width,
                                              uint16 height,
                                              const uint16 *pixels)
{
    IPS114_CS(0);
    ImageDebugScreen_WriteCommand(0x2AU);
    spi_write_16bit(IPS114_SPI, IMAGE_SCREEN_LCD_X_OFFSET + x);
    spi_write_16bit(IPS114_SPI,
                    IMAGE_SCREEN_LCD_X_OFFSET + x + width - 1U);
    ImageDebugScreen_WriteCommand(0x2BU);
    spi_write_16bit(IPS114_SPI, IMAGE_SCREEN_LCD_Y_OFFSET + y);
    spi_write_16bit(IPS114_SPI,
                    IMAGE_SCREEN_LCD_Y_OFFSET + y + height - 1U);
    ImageDebugScreen_WriteCommand(0x2CU);
    spi_write_16bit_array(IPS114_SPI,
                          pixels,
                          (uint32)width * (uint32)height);
    IPS114_CS(1);
}

static void ImageDebugScreen_ShowRgb565Frame(const uint16 *pixels)
{
    ImageDebugScreen_ShowRgb565Region(0U,
                                      0U,
                                      IMAGE_SCREEN_WIDTH,
                                      IMAGE_SCREEN_HEIGHT,
                                      pixels);
}

static uint8 ImageDebugScreen_StartImageFrame(void)
{
    const uint8 *source;
    uint16 *target = &s_image_rgb565[0][0];
    uint32 pixel;
    uint8 binary_mode;

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

    binary_mode = ((s_screen_mode == IMAGE_DEBUG_SCREEN_MODE_BEACON_BINARY) ||
                   (s_screen_mode == IMAGE_DEBUG_SCREEN_MODE_LAMP_BINARY)) ?
                  1U : 0U;
    for(pixel = 0U; pixel < IMAGE_SCREEN_WIDTH * IMAGE_SCREEN_HEIGHT; pixel++)
    {
        uint8 gray = source[pixel];
        target[pixel] = (binary_mode != 0U) ?
                        ((gray != 0U) ? RGB565_WHITE : RGB565_BLACK) :
                        ImageDebugScreen_GrayToRgb565(gray);
    }
    if(s_screen_mode == IMAGE_DEBUG_SCREEN_MODE_OVERLAY)
    {
        ImageDebugScreen_ApplyOverlay();
    }
    if(s_aux_refresh_counter == 0U)
    {
        ImageDebugScreen_CaptureAuxSnapshot();
    }
    return 1U;
}

static void ImageDebugScreen_DrawImageFrame(void)
{
    ImageDebugScreen_ShowRgb565Frame(&s_image_rgb565[0][0]);
    if(s_aux_refresh_counter == 0U)
    {
        ImageDebugScreen_DrawAuxPanel();
        s_aux_refresh_counter = IMAGE_SCREEN_AUX_REFRESH_FRAMES - 1U;
    }
    else
    {
        s_aux_refresh_counter--;
    }
}

void ImageDebugScreen_Init(void)
{
    s_screen_tick_10ms = 0U;
    s_next_refresh_tick = 0U;
    s_screen_mode = IMAGE_DEBUG_SCREEN_MODE_DATA;
    s_hw_initialized = 0U;
    s_refresh_was_enabled = 0U;
    s_startup_layout_ready = 0U;
    s_layout_dirty = 0U;
    s_image_frame_pending = 0U;
    s_aux_refresh_counter = 0U;

#if (IMAGE_SCREEN_FORCE_DISABLE != 0U)
    return;
#endif
    gpio_init(IPS114_BLK_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
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
        if(s_hw_initialized != 0U)
        {
            IPS114_BLK(0);
        }
        s_refresh_was_enabled = 0U;
        s_image_frame_pending = 0U;
        return;
    }

    if(s_hw_initialized == 0U)
    {
        ips114_set_dir(IPS114_PORTAIT);
        ips114_init();
        ImageDebugScreen_ConfigureSpi();
        s_hw_initialized = 1U;
        s_refresh_was_enabled = 1U;
        ImageDebugScreen_MarkLayoutDirty();
        IPS114_BLK(1);
        return;
    }

    if(s_refresh_was_enabled == 0U)
    {
        s_refresh_was_enabled = 1U;
        IPS114_BLK(1);
        if((s_startup_layout_ready != 0U) &&
           (s_screen_mode == IMAGE_DEBUG_SCREEN_MODE_DATA) &&
           (s_layout_dirty == 0U))
        {
            s_startup_layout_ready = 0U;
        }
        else
        {
            ImageDebugScreen_MarkLayoutDirty();
        }
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
        ImageDebugScreen_UpdateData();
        s_next_refresh_tick =
            ImageDebugScreen_Now() + IMAGE_SCREEN_DATA_PERIOD_TICKS;
        return;
    }

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
    ImageDebugScreen_DrawImageFrame();

}
