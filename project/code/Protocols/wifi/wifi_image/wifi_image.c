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

/*
 * 函数功能：初始化图像发送模块默认配置和统计信息。
 * 输入参数：无。
 * 返回值：无。
 */
void wifi_image_Init(void)
{
    memset(&s_wifi_image_stats, 0, sizeof(s_wifi_image_stats));
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
    uint32_t start_tick_ms;
    uint32_t end_tick_ms;
    uint32_t pre_frame[WIFI_IMAGE_PRE_FRAME_WORD_COUNT];
    uint8_t ok = 0U;

    if ((NULL == image) || (0U == image_size) || (0U == wifi_cmd_IsReady()))
    {
        s_wifi_image_stats.fail_count++;
        return 0U;
    }

    pre_frame[0] = (uint32_t)image_id;
    pre_frame[1] = image_size;
    pre_frame[2] = (uint32_t)width;
    pre_frame[3] = (uint32_t)height;
    pre_frame[4] = (uint32_t)image_format;
    pre_frame[5] = WIFI_IMAGE_PRE_FRAME_TAIL;
    pre_frame[6] = WIFI_IMAGE_PRE_FRAME_TAIL;

    start_tick_ms = tick_1000us_cnt;

    /* 先发图片前导帧，再发图像本体，最后统一触发一次UDP立即发送 */
    if ((0U != wifi_cmd_SendBufferNoFlush((const uint8_t *)pre_frame, sizeof(pre_frame))) &&
        (0U != wifi_cmd_SendBufferNoFlush(image, image_size)) &&
        (0U != wifi_cmd_FlushNow()))
    {
        ok = 1U;
    }

    end_tick_ms = tick_1000us_cnt;
    s_wifi_image_stats.last_cost_ms = end_tick_ms - start_tick_ms;

    if (0U == ok)
    {
        s_wifi_image_stats.fail_count++;
        return 0U;
    }

    s_wifi_image_stats.ok_count++;
    s_wifi_image_stats.last_send_tick_ms = end_tick_ms;
    return 1U;
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
