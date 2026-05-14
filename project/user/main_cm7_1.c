#include "zf_common_headfile.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    SCB_DisableDCache();

    /* CM7_1 不再承载图像处理和 IPS114 刷屏，只保留 IPC 基础通道接收飞控状态。 */
    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);


    while(true)
    {
        system_delay_ms(1000);
    }
}
