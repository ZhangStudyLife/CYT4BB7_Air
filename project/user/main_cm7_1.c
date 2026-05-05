#include "zf_common_headfile.h"

#define CM7_1_IMAGE_DISPLAY_X       0U      /* 图像显示起点X */
#define CM7_1_IMAGE_DISPLAY_Y       0U      /* 图像显示起点Y */
#define CM7_1_CIRCLE_TEXT_X         188U    /* 圆识别信息显示起点X */
#define CM7_1_CIRCLE_TEXT_Y         0U      /* 圆识别信息显示起点Y */
#define CM7_1_CIRCLE_TEXT_LINE_H    8U      /* 6x8字体行高 */
#define CM7_1_CIRCLE_TEXT_MAX_LINES 16U     /* 右侧最多显示行数 */

/*
 * 函数功能：在 IPS114 右侧显示图像识别出的圆信息。
 * 输入参数：
 *   无。
 * 返回值：
 *   无。
 */
static void cm7_1_show_circle_info(void)
{
    uint8 i;
    char buf[9];

    ips114_set_font(IPS114_6X8_FONT);
    ips114_set_color(RGB565_GREEN, RGB565_BLACK);
    ips114_show_string(CM7_1_CIRCLE_TEXT_X, CM7_1_CIRCLE_TEXT_Y, "CIRCLE  ");

    for (i = 0U; (i < IMAGE_MAX_CIRCLE_COUNT) && ((uint16)(1U + 3U * i + 2U) < CM7_1_CIRCLE_TEXT_MAX_LINES); i++)
    {
        if (0U != g_image_circles[i].valid)
        {
            snprintf(buf, sizeof(buf), "%uV%uR%3d ",
                     (unsigned int)i,
                     (unsigned int)g_image_circles[i].valid,
                     (int)g_image_circles[i].radius);
            ips114_show_string(CM7_1_CIRCLE_TEXT_X,
                               CM7_1_CIRCLE_TEXT_Y + ((uint16)(1U + 3U * i) * CM7_1_CIRCLE_TEXT_LINE_H),
                               buf);

            snprintf(buf, sizeof(buf), "X%5d  ",
                     (int)g_image_circles[i].x);
            ips114_show_string(CM7_1_CIRCLE_TEXT_X,
                               CM7_1_CIRCLE_TEXT_Y + ((uint16)(2U + 3U * i) * CM7_1_CIRCLE_TEXT_LINE_H),
                               buf);

            snprintf(buf, sizeof(buf), "Y%5d  ",
                     (int)g_image_circles[i].y);
        }
        else
        {
            snprintf(buf, sizeof(buf), "%uV0R -- ",
                     (unsigned int)i);
            ips114_show_string(CM7_1_CIRCLE_TEXT_X,
                               CM7_1_CIRCLE_TEXT_Y + ((uint16)(1U + 3U * i) * CM7_1_CIRCLE_TEXT_LINE_H),
                               buf);

            snprintf(buf, sizeof(buf), "X --    ");
            ips114_show_string(CM7_1_CIRCLE_TEXT_X,
                               CM7_1_CIRCLE_TEXT_Y + ((uint16)(2U + 3U * i) * CM7_1_CIRCLE_TEXT_LINE_H),
                               buf);

            snprintf(buf, sizeof(buf), "Y --    ");
        }

        ips114_show_string(CM7_1_CIRCLE_TEXT_X,
                           CM7_1_CIRCLE_TEXT_Y + ((uint16)(3U + 3U * i) * CM7_1_CIRCLE_TEXT_LINE_H),
                           buf);
    }
}

int main(void)
{
    uint8 screen_enabled_last = 1U;

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
        uint8 screen_enabled = (0U == ipc_core0_is_flying()) ? 1U : 0U;

        if (0U != screen_enabled)
        {
            if (0U == screen_enabled_last)
            {
                screen_enabled_last = 1U;
                ips114_clear();
            }

            if(mt9v03x_finish_flag)
            {
                ips114_show_gray_image(CM7_1_IMAGE_DISPLAY_X,
                                       CM7_1_IMAGE_DISPLAY_Y,
                                       (const uint8 *)mt9v03x_image,
                                       MT9V03X_W,
                                       MT9V03X_H,
                                       MT9V03X_W,
                                       MT9V03X_H,
                                       0);
            }
        }

        image_update();
        ipc_image_send();

        if (0U != screen_enabled)
        {
            cm7_1_show_circle_info();
        }
        else
        {
            screen_enabled_last = 0U;
        }

        printf("%u,%.2f,%.2f\r\n", (unsigned int)g_image_circles[0].valid, g_image_circles[0].x, g_image_circles[0].y);
    }
}
