#include "zf_common_headfile.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();

    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);
    image_init();
    /* mt9v03x_init() re-enables DCache; keep camera/IPC shared RAM coherent. */
    SCB_DisableDCache();

    while(true)
    {
        if(mt9v03x_finish_flag)
        {
            image_update();
            ipc_image_send();
        }
    }
}
