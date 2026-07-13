#include "zf_common_headfile.h"
#include "Estimation/Pos_Est/image_down.h"

#define IMAGE_PIT  (PIT_CH10)
#define IMAGE_SCREEN_X_VALUE  (32U)
#define IMAGE_SCREEN_Y_LABEL  (88U)
#define IMAGE_SCREEN_Y_VALUE  (112U)
#define IMAGE_SCREEN_A_LABEL  (160U)
#define IMAGE_SCREEN_A_VALUE  (184U)
#define IMAGE_SCREEN_ROW_H    (16U)

volatile uint8 g_image_tick_100hz = 0U;

static void Get_Image_data(void)
{
    CameraSpi_Update();
    (void)image_down_update();
    CameraSpi_GetSnapshot(image_data);
}

static void ImageDebugScreen_Init(void)
{
    ips114_init();
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_font(IPS114_8X16_FONT);
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    ips114_clear();

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

    ips114_show_string(0U, 64U, "Lamp[0] CXY");
    ips114_show_string(0U, 80U, "F x:");
    ips114_show_string(IMAGE_SCREEN_Y_LABEL, 80U, " y:");
    ips114_show_string(0U, 96U, "C x:");
    ips114_show_string(IMAGE_SCREEN_Y_LABEL, 96U, " y:");
    ips114_show_string(0U, 112U, "B x:");
    ips114_show_string(IMAGE_SCREEN_Y_LABEL, 112U, " y:");
}

static void ImageDebugScreen_ShowLamp(uint16 y, const car_lamp_data *lamp)
{
    ips114_set_color((image_data_car_lamp_valid(lamp) != 0U) ? RGB565_BLACK : RGB565_RED, RGB565_WHITE);
    ips114_show_float(IMAGE_SCREEN_X_VALUE, y, lamp->cx, 3U, 1U);
    ips114_show_float(IMAGE_SCREEN_Y_VALUE, y, lamp->cy, 3U, 1U);
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
}

static void ImageDebugScreen_ShowBeacon(uint16 y, const beacon_data *beacon)
{
    ips114_set_color((image_data_beacon_valid(beacon) != 0U) ? RGB565_BLACK : RGB565_RED, RGB565_WHITE);
    ips114_show_float(IMAGE_SCREEN_X_VALUE, y, beacon->x, 3U, 1U);
    ips114_show_float(IMAGE_SCREEN_Y_VALUE, y, beacon->y, 3U, 1U);
    ips114_show_float(IMAGE_SCREEN_A_VALUE, y, beacon->area, 4U, 1U);
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
}

static void ImageDebugScreen_Update(void)
{
    ImageDebugScreen_ShowBeacon(IMAGE_SCREEN_ROW_H, &image_data[Front].beacon_data[0]);
    ImageDebugScreen_ShowBeacon(2U * IMAGE_SCREEN_ROW_H, &image_data[Center].beacon_data[0]);
    ImageDebugScreen_ShowBeacon(3U * IMAGE_SCREEN_ROW_H, &image_data[Back].beacon_data[0]);

    ImageDebugScreen_ShowLamp(5U * IMAGE_SCREEN_ROW_H, &image_data[Front].car_lamp_data[0]);
    ImageDebugScreen_ShowLamp(6U * IMAGE_SCREEN_ROW_H, &image_data[Center].car_lamp_data[0]);
    ImageDebugScreen_ShowLamp(7U * IMAGE_SCREEN_ROW_H, &image_data[Back].car_lamp_data[0]);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();

    ImageDebugScreen_Init();
    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);
    ipc_remote_param_core1_init();
    image_data_clear(&image_data[Front]);
    image_data_clear(&image_data[Center]);
    image_data_clear(&image_data[Back]);
    image_down_init();
    CameraSpi_Init();
    pit_ms_init(IMAGE_PIT, 10U);

    while(true)
    {
        if(g_image_tick_100hz > 0U)
        {
            g_image_tick_100hz--;
            ipc_remote_param_core1_poll();
            Get_Image_data();
            ipc_remote_param_core1_poll();
            ipc_image_publish();
            ImageDebugScreen_Update();
        }
    }
}
