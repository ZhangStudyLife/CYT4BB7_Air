/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          image
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2026-4-1          OpenAI              first version
*********************************************************************************************************************/

#include "image.h"

#include <string.h>

#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_device_wifi_spi.h"

#define IMAGE_DEFAULT_THRESHOLD    (200U)   /* 默认高亮目标提取阈值 */

uint8 g_image_frame[MT9V03X_H][MT9V03X_W];   /* 完整单帧灰度图缓存 */
uint8 g_image_binary[MT9V03X_H][MT9V03X_W];  /* 默认二值化图像缓存 */
uint8 g_image_threshold = IMAGE_DEFAULT_THRESHOLD; /* 默认固定二值化阈值 */

static uint8 s_image_frame_ready = 0U;       /* 完整单帧缓存是否已经就绪 */

/*
 * 函数功能：检测摄像头整帧完成标志，并将最新完整帧复制到内部缓存。
 * 输入参数：无。
 * 返回值：无。
 */
static void image_refresh_frame_cache(void)
{
    if (0U != mt9v03x_finish_flag)
    {
        memcpy(g_image_frame[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
        mt9v03x_finish_flag = 0U;
        s_image_frame_ready = 1U;
    }
}

/*
 * 函数功能：初始化图像处理模块，完成摄像头、缓存和图传接口准备。
 * 输入参数：无。
 * 返回值：无。
 */
void image_init(void)
{
    memset(g_image_frame, 0, sizeof(g_image_frame));
    memset(g_image_binary, 0, sizeof(g_image_binary));

    g_image_threshold = IMAGE_DEFAULT_THRESHOLD;
    s_image_frame_ready = 0U;
    mt9v03x_finish_flag = 0U;

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    (void)mt9v03x_init();
}

/*
 * 函数功能：对输入灰度图执行固定阈值二值化，提取高亮目标。
 * 输入参数：src-输入灰度图首地址；width-图像宽度，单位像素；height-图像高度，单位像素；
 *           threshold-二值化阈值；dst-输出二值图首地址。
 * 返回值：无。
 */
void image_binary_threshold(const uint8 *src, uint16 width, uint16 height, uint8 threshold, uint8 *dst)
{
    uint32 pixel_count;
    uint32 i;
    const uint8 *stable_src;

    if ((NULL == dst) || (0U == width) || (0U == height))
    {
        return;
    }

    image_refresh_frame_cache();

    stable_src = src;
    if ((NULL == stable_src) || ((const uint8 *)mt9v03x_image[0] == stable_src))
    {
        if (0U == s_image_frame_ready)
        {
            memset(dst, 0, (uint32)width * (uint32)height);
            return;
        }
        stable_src = g_image_frame[0];
    }

    pixel_count = (uint32)width * (uint32)height;

    /* 单次遍历完成固定阈值二值化，优先保证实时性。 */
    for (i = 0U; i < pixel_count; i++)
    {
        dst[i] = (stable_src[i] > threshold) ? 255U : 0U;
    }
}

/*
 * 函数功能：通过逐飞助手接口发送图像数据，支持灰度图或二值图。
 * 输入参数：image-待发送图像首地址；width-图像宽度，单位像素；height-图像高度，单位像素。
 * 返回值：无。
 */
void image_send(const uint8 *image, uint16 width, uint16 height)
{
    const uint8 *stable_image;

    if ((0U == width) || (0U == height))
    {
        return;
    }

    image_refresh_frame_cache();

    stable_image = image;
    if ((NULL == stable_image) || ((const uint8 *)mt9v03x_image[0] == stable_image))
    {
        if (0U == s_image_frame_ready)
        {
            return;
        }
        stable_image = g_image_frame[0];
    }

    /* 每次发送前清空边线配置，避免逐飞助手沿用旧边界数据。 */
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, (void *)stable_image, width, height);
    seekfree_assistant_camera_boundary_config(NO_BOUNDARY, 0U, NULL, NULL, NULL, NULL, NULL, NULL);
    seekfree_assistant_camera_send();
    (void)wifi_spi_udp_send_now();
}
