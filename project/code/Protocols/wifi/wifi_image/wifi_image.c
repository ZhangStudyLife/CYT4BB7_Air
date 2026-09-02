/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
/*****************************************************************************
 * 文件: wifi_image.c
 * 模块: WiFi 图像传输
 * 职责: 通过现有 WiFi-UDP 链路向 VOFA+ 发送图像数据包
 *****************************************************************************/

#include "wifi_image.h"

#include <string.h>

#define WIFI_IMAGE_PRE_FRAME_WORD_COUNT    (7U)           /* VOFA+ 图像前导帧字数 */
#define WIFI_IMAGE_PRE_FRAME_TAIL          (0x7F800000UL) /* VOFA+ 图像前导帧结束标记 */

extern volatile uint32 tick_1000us_cnt;                   /* 1ms系统节拍计数 */

/* 图像发送统计信息 */
static wifi_image_tx_stats_t s_wifi_image_stats = {0};
static uint8_t s_wifi_image_assistant_inited = 0U;        /* 逐飞助手接口是否已初始化 */
static uint16_t s_wifi_image_cfg_width = 0U;              /* 当前逐飞助手图像宽度配置 */
static uint16_t s_wifi_image_cfg_height = 0U;             /* 当前逐飞助手图像高度配置 */

/*
 * 函数功能：初始化图像发送模块默认配置和统计信息。
 * 输入参数：无。
 * 返回值：无。
 */
void wifi_image_Init(void)
{
    memset(&s_wifi_image_stats, 0, sizeof(s_wifi_image_stats));
    s_wifi_image_assistant_inited = 0U;
    s_wifi_image_cfg_width = 0U;
    s_wifi_image_cfg_height = 0U;
}

/*
 * 函数功能：发送一帧 VOFA+ 图像数据包。
 * 输入参数：image-图像数据首地址；image_size-图像字节数；width-图像宽度；height-图像高度；
 *           image_format-VOFA+ 图像格式；image_id-图像通道ID。
 * 返回值：1-发送成功；0-发送失败。
 */
uint8_t wifi_image_SendFrame(const uint8_t *image,
                             uint32_t image_size,
                             int32_t width,
                             int32_t height,
                             int32_t image_format,
                             int32_t image_id)
{
#if (0U == WIFI_IMAGE_ENABLE)
    (void)image;
    (void)image_size;
    (void)width;
    (void)height;
    (void)image_format;
    (void)image_id;
    return 1U;
#else
    uint32_t start_tick_ms;
    uint32_t end_tick_ms;
    uint16_t cfg_width;
    uint16_t cfg_height;

    if ((NULL == image) || (0U == image_size) || (0U == wifi_cmd_IsReady()))
    {
        s_wifi_image_stats.fail_count++;
        return 0U;
    }

    if ((width <= 0) || (height <= 0))
    {
        s_wifi_image_stats.fail_count++;
        return 0U;
    }

    if ((int32_t)WIFI_IMAGE_VOFA_FORMAT_GRAYSCALE8 != image_format)
    {
        s_wifi_image_stats.fail_count++;
        return 0U;
    }

    (void)image_size;
    (void)image_id;
    cfg_width = (uint16_t)width;
    cfg_height = (uint16_t)height;

    if ((0U == s_wifi_image_assistant_inited) ||
        (s_wifi_image_cfg_width != cfg_width) ||
        (s_wifi_image_cfg_height != cfg_height))
    {
        // seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
        s_wifi_image_assistant_inited = 1U;
        s_wifi_image_cfg_width = cfg_width;
        s_wifi_image_cfg_height = cfg_height;
    }

    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,
                                                 (void *)image,
                                                 cfg_width,
                                                 cfg_height);

    start_tick_ms = tick_1000us_cnt;

    /* 先发图片前导帧，再发图像本体，最后统一触发一次UDP立即发送 */
    seekfree_assistant_camera_send();

    end_tick_ms = tick_1000us_cnt;
    s_wifi_image_stats.last_cost_ms = end_tick_ms - start_tick_ms;

    s_wifi_image_stats.ok_count++;
    s_wifi_image_stats.last_send_tick_ms = end_tick_ms;
    return 1U;
#endif
}

/*
 * 函数功能：读取图像发送统计信息。
 * 输入参数：stats-统计信息输出指针。
 * 返回值：无。
 */
void wifi_image_GetTxStats(wifi_image_tx_stats_t *stats)
{
    if (NULL == stats)
    {
        return;
    }

    *stats = s_wifi_image_stats;
}
