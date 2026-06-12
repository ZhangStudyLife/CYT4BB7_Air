#include "zf_common_headfile.h"

#define SPI_TEST_TIMER       (TC_TIME2_CH0)
#define SPI_TEST_PERIOD_MS   (10U)

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();

    CameraSpi_Init();
    timer_init(SPI_TEST_TIMER, TIMER_MS);
    timer_start(SPI_TEST_TIMER);

    while(true)
    {
        if(timer_get(SPI_TEST_TIMER) >= SPI_TEST_PERIOD_MS)
        {
            timer_clear(SPI_TEST_TIMER);
            CameraSpi_Update();
        }
    }
}
