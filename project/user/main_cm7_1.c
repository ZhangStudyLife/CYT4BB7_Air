#include "zf_common_headfile.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    ips114_init();
    ips114_show_string(0, 0, "mt9v03x init.");

    image_init();
    system_delay_ms(100);
    while(0U == mt9v03x_finish_flag)
    {
        if(mt9v03x_init())
        {
            ips114_show_string(0, 16, "mt9v03x reinit.");
            system_delay_ms(500);
        }
        else
        {
            break;
        }
    }
    ips114_show_string(0, 16, "init success.");

    SCB_DisableDCache();
    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);

    printf("Hello CYT4BB7!\n");

    while(true)
    {
        if(mt9v03x_finish_flag)
        {
            ips114_displayimage03x((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
        }
        image_update();
        ipc_image_send();

        printf("%u,%.2f,%.2f\r\n", (unsigned int)g_image_circles[0].valid, g_image_circles[0].x, g_image_circles[0].y);
    }
}
