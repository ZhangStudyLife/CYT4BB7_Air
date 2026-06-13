#include "zf_common_headfile.h"

#define CAMERA_SPI_PIT       (PIT_CH10)

volatile uint8 g_camera_spi_tick_100hz = 0U;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();

    CameraSpi_Init();
    pit_ms_init(CAMERA_SPI_PIT, 10U);

    while(true)
    {
        if(g_camera_spi_tick_100hz > 0U)
        {
            g_camera_spi_tick_100hz--;
            CameraSpi_Update();
        }
    }
}
