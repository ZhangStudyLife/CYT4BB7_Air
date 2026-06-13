#include "zf_common_headfile.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();

    CameraSpi_Init();

    while(true)
    {
        system_delay_ms(10U);
        CameraSpi_Update();
    }
}
