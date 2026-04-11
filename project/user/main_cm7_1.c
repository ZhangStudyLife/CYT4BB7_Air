#include "zf_common_headfile.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    image_init();
    SCB_DisableDCache();
    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);

    printf("Hello CYT4BB7!\n");

    while(true)
    {
        image_update();
        ipc_image_send();

        printf("%u,%.2f,%.2f\r\n", (unsigned int)g_image_circles[0].valid, g_image_circles[0].x, g_image_circles[0].y);
    }
}
