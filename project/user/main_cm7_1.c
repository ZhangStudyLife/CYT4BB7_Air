#include "zf_common_headfile.h"
#include "Estimation/Pos_Est/image_down.h"
#include "Estimation/image/image_fusion.h"

#define IMAGE_PIT  (PIT_CH10)
#define IMAGE_SCREEN_X_VALUE  (32U)
#define IMAGE_SCREEN_Y_LABEL  (88U)
#define IMAGE_SCREEN_Y_VALUE  (112U)
#define IMAGE_SCREEN_A_LABEL  (160U)
#define IMAGE_SCREEN_A_VALUE  (184U)
#define IMAGE_SCREEN_ROW_H    (16U)

struct image_data image_data[IMAGE_CAMERA_COUNT];
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

static void ImageDebugScreen_ShowXY(uint16 y, uint8 valid, float x, float y_value)
{
    ips114_set_color((valid != 0U) ? RGB565_BLACK : RGB565_RED, RGB565_WHITE);
    ips114_show_float(IMAGE_SCREEN_X_VALUE, y, x, 3U, 1U);
    ips114_show_float(IMAGE_SCREEN_Y_VALUE, y, y_value, 3U, 1U);
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
}

static void ImageDebugScreen_ShowBeacon(uint16 y, const beacon_data *beacon)
{
    ips114_set_color((beacon->valid != 0U) ? RGB565_BLACK : RGB565_RED, RGB565_WHITE);
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

    ImageDebugScreen_ShowXY(5U * IMAGE_SCREEN_ROW_H,
                            image_data[Front].car_lamp_data[0].valid,
                            image_data[Front].car_lamp_data[0].cx,
                            image_data[Front].car_lamp_data[0].cy);
    ImageDebugScreen_ShowXY(6U * IMAGE_SCREEN_ROW_H,
                            image_data[Center].car_lamp_data[0].valid,
                            image_data[Center].car_lamp_data[0].cx,
                            image_data[Center].car_lamp_data[0].cy);
    ImageDebugScreen_ShowXY(7U * IMAGE_SCREEN_ROW_H,
                            image_data[Back].car_lamp_data[0].valid,
                            image_data[Back].car_lamp_data[0].cx,
                            image_data[Back].car_lamp_data[0].cy);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();

    ImageDebugScreen_Init();
    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);
    image_down_init();
    CameraSpi_Init();
    image_fusion_init();
    pit_ms_init(IMAGE_PIT, 10U);

    while(true)
    {
        if(g_image_tick_100hz > 0U)
        {
            float tv;
            float lv;

            g_image_tick_100hz--;
            Get_Image_data();
            image_fusion_update_100HZ(image_data);

            tv = (g_image_fusion.center_delta_valid != 0U) ? 1.0f : 0.0f;
            lv = (g_image_fusion.lamp_angle_valid != 0U) ? 1.0f : 0.0f;
            ipc_mode2_send(tv,
                           g_image_fusion.center_delta_x,
                           g_image_fusion.center_delta_y,
                           lv,
                           g_image_fusion.car_lamp_cx,
                           g_image_fusion.car_lamp_cy,
                           g_image_fusion.lamp_angle_deg);
            ImageDebugScreen_Update();
        }
    }
}
