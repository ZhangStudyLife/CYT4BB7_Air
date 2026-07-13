#include "zf_common_headfile.h"
#include "Estimation/Pos_Est/image_down.h"
#include "Display/image_debug_screen.h"

#define IMAGE_PIT  (PIT_CH10)

volatile uint8 g_image_tick_100hz = 0U;

static uint8 Get_Image_data(void)
{
    uint8 image_frame_updated;

    CameraSpi_Update();
    image_frame_updated = image_down_update();
    CameraSpi_GetSnapshot(image_data);
    return image_frame_updated;
}

int main(void)
{
    uint8 image_frame_updated;

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
            ImageDebugScreen_Tick10ms();
            ipc_remote_param_core1_poll();
            image_frame_updated = Get_Image_data();
            ipc_remote_param_core1_poll();
            ipc_image_publish();
            ImageDebugScreen_Update(image_frame_updated);
        }
    }
}
