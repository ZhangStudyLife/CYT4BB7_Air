/*****************************************************************************
 * 文件: wifi_image.h
 * 模块: WiFi 图像传输
 * 职责: 通过现有 WiFi-UDP 链路向 VOFA+ 发送图像数据包
 *****************************************************************************/

#ifndef WIFI_IMAGE_H
#define WIFI_IMAGE_H

#include "zf_common_headfile.h"

#define WIFI_IMAGE_VOFA_FORMAT_GRAYSCALE8   (24)   /* VOFA+ 8位灰度图格式 */

/* 图像发送统计信息 */
typedef struct
{
    uint32_t last_cost_ms;         /* 最近一次发送耗时，单位ms */
    uint32_t last_send_tick_ms;    /* 最近一次成功发送时的系统tick，单位ms */
    uint32_t ok_count;             /* 成功发送次数 */
    uint32_t fail_count;           /* 发送失败次数 */
} wifi_image_tx_stats_t;

/*
 * 函数功能：初始化图像发送模块默认配置和统计信息。
 * 输入参数：无。
 * 返回值：无。
 */
void wifi_image_Init(void);

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
                             int32_t image_id);

/*
 * 函数功能：读取图像发送统计信息。
 * 输入参数：stats-统计信息输出指针。
 * 返回值：无。
 */
void wifi_image_GetTxStats(wifi_image_tx_stats_t *stats);

#endif /* WIFI_IMAGE_H */
