#include "image_debug_screen.h"

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
static uint8 s_image_frame_active;
static uint8 s_image_frame_pending;
static uint8 s_image_next_row;
static uint16 s_layout_clear_row;
static uint8 s_image_snapshot[IMAGE_SCREEN_HEIGHT][IMAGE_SCREEN_WIDTH];
static uint8 s_clear_block[IMAGE_SCREEN_BLOCK_ROWS][IMAGE_SCREEN_PHYSICAL_WIDTH];

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

static void ImageDebugScreen_MarkLayoutDirty(void)
{
    s_layout_dirty = 1U;
    s_layout_clear_row = 0U;
    s_data_layout_stage = 0U;
    s_image_frame_pending = 0U;
    ImageDebugScreen_AbortImageFrame();
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
    return ((ImageDebugScreen_RefreshAllowed() != 0U) &&
            (ImageDebugScreen_TaskBacklog() == 0U)) ? 1U : 0U;
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
    return ((ImageDebugScreen_RefreshAllowed() != 0U) &&
            (ImageDebugScreen_TaskBacklog() == 0U)) ? 1U : 0U;
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
    return ((ImageDebugScreen_RefreshAllowed() != 0U) &&
            (ImageDebugScreen_TaskBacklog() == 0U)) ? 1U : 0U;
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
    return ((ImageDebugScreen_RefreshAllowed() != 0U) &&
            (ImageDebugScreen_TaskBacklog() == 0U)) ? 1U : 0U;
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
    if(ImageDebugScreen_ShowBeacon(IMAGE_SCREEN_ROW_H,
                                   &image_data[Front].beacon_data[0]) == 0U)
    {
        return 0U;
    }
    if(ImageDebugScreen_ShowBeacon(2U * IMAGE_SCREEN_ROW_H,
                                   &image_data[Center].beacon_data[0]) == 0U)
    {
        return 0U;
    }
    if(ImageDebugScreen_ShowBeacon(3U * IMAGE_SCREEN_ROW_H,
                                   &image_data[Back].beacon_data[0]) == 0U)
    {
        return 0U;
    }
    if(ImageDebugScreen_ShowLamp(5U * IMAGE_SCREEN_ROW_H,
                                 &image_data[Front].car_lamp_data[0]) == 0U)
    {
        return 0U;
    }
    if(ImageDebugScreen_ShowLamp(6U * IMAGE_SCREEN_ROW_H,
                                 &image_data[Center].car_lamp_data[0]) == 0U)
    {
        return 0U;
    }
    return ImageDebugScreen_ShowLamp(7U * IMAGE_SCREEN_ROW_H,
                                     &image_data[Back].car_lamp_data[0]);
}

static uint8 ImageDebugScreen_StartImageFrame(void)
{
    const uint8 *source;

    source = (s_screen_mode == IMAGE_DEBUG_SCREEN_MODE_RAW) ?
             image_down_get_frame_buffer() :
             image_down_get_binary_buffer();
    if(source == NULL)
    {
        return 0U;
    }

    memcpy(s_image_snapshot[0], source, sizeof(s_image_snapshot));
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
    threshold = (s_screen_mode == IMAGE_DEBUG_SCREEN_MODE_BINARY) ? 1U : 0U;
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
    memset(s_clear_block, 0xFF, sizeof(s_clear_block));
    ImageDebugScreen_MarkLayoutDirty();
}

void ImageDebugScreen_Tick10ms(void)
{
    s_screen_tick_10ms++;
}

uint8 ImageDebugScreen_SetMode(uint8 mode)
{
    if(mode > IMAGE_DEBUG_SCREEN_MODE_BINARY)
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
    }

}
