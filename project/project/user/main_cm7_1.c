#include "zf_common_headfile.h"
#include "Estimation/image/mode2_center_image.h"
#include "Estimation/image/mode2_three_camera.h"

#define CAMERA_SPI_PIT  (PIT_CH10)

volatile uint8 g_camera_spi_tick_100hz = 0U;

static void mode2_update_100HZ(void)
{
    uint8 i;
    camera_spi_board_snapshot_t spi_snap[CAMERA_SPI_BOARD_COUNT];
    mode2_three_camera_frame_t camera[MODE2_THREE_CAMERA_CAMERA_COUNT];

    CameraSpi_Update();
    mode2_center_image_update();
    CameraSpi_GetSnapshot(spi_snap);

    memset(camera, 0, sizeof(camera));

    for(i = 0U; i < MODE2_THREE_CAMERA_CAMERA_TARGETS; i++)
    {
        camera[MODE2_THREE_CAMERA_CAMERA_FRONT].target[i].valid = spi_snap[0].beacons[i].valid;
        camera[MODE2_THREE_CAMERA_CAMERA_FRONT].target[i].x = spi_snap[0].beacons[i].x;
        camera[MODE2_THREE_CAMERA_CAMERA_FRONT].target[i].y = spi_snap[0].beacons[i].y;
        camera[MODE2_THREE_CAMERA_CAMERA_FRONT].target[i].area = spi_snap[0].beacons[i].area;
    }

    for(i = 0U; i < MODE2_THREE_CAMERA_CAMERA_TARGETS; i++)
    {
        if(i < g_mode2_center_image_beacon_count)
        {
            camera[MODE2_THREE_CAMERA_CAMERA_CENTER].target[i].valid = g_mode2_center_image_beacons[i].valid;
            camera[MODE2_THREE_CAMERA_CAMERA_CENTER].target[i].x = g_mode2_center_image_beacons[i].x;
            camera[MODE2_THREE_CAMERA_CAMERA_CENTER].target[i].y = g_mode2_center_image_beacons[i].y;
            camera[MODE2_THREE_CAMERA_CAMERA_CENTER].target[i].area = g_mode2_center_image_beacons[i].area;
        }
    }

    for(i = 0U; i < MODE2_THREE_CAMERA_CAMERA_TARGETS; i++)
    {
        camera[MODE2_THREE_CAMERA_CAMERA_REAR].target[i].valid = spi_snap[1].beacons[i].valid;
        camera[MODE2_THREE_CAMERA_CAMERA_REAR].target[i].x = spi_snap[1].beacons[i].x;
        camera[MODE2_THREE_CAMERA_CAMERA_REAR].target[i].y = spi_snap[1].beacons[i].y;
        camera[MODE2_THREE_CAMERA_CAMERA_REAR].target[i].area = spi_snap[1].beacons[i].area;
    }

    if(g_mode2_center_image_car_lamp_count > 0U)
    {
        mode2_three_camera_set_center_car_lamp(
            g_mode2_center_image_car_lamps[0].valid,
            g_mode2_center_image_car_lamps[0].cx,
            g_mode2_center_image_car_lamps[0].cy,
            g_mode2_center_image_car_lamps[0].angle);
    }
    else
    {
        mode2_three_camera_set_center_car_lamp(0U, 0.0f, 0.0f, 0.0f);
    }

    mode2_three_camera_update_100HZ(camera);

    {
        float tv = (g_mode2_three_camera.center_delta_valid != 0U) ? 1.0f : 0.0f;
        float lv = (g_mode2_three_camera.lamp_angle_valid != 0U) ? 1.0f : 0.0f;

        ipc_mode2_send(tv,
                       g_mode2_three_camera.center_delta_x,
                       g_mode2_three_camera.center_delta_y,
                       lv,
                       g_mode2_three_camera.car_lamp_cx,
                       g_mode2_three_camera.car_lamp_cy,
                       g_mode2_three_camera.lamp_angle_deg);
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();

    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);
    mode2_center_image_init();
    CameraSpi_Init();
    mode2_three_camera_init();
    pit_ms_init(CAMERA_SPI_PIT, 10U);

    while(true)
    {
        if(g_camera_spi_tick_100hz > 0U)
        {
            g_camera_spi_tick_100hz--;
            mode2_update_100HZ();
        }
    }
}
