#include "image.h"

#include <string.h>

#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"
#include "../../Protocols/wifi/wifi_image/wifi_image.h"

#define IMAGE_DEFAULT_THRESHOLD    (120U)   /* 默认高亮目标提取阈值 */

/* 内部完整单帧灰度图缓存，单位像素灰度值 */
static uint8 s_image_frame[MT9V03X_H][MT9V03X_W];
/* 内部默认二值化图像缓存，单位像素灰度值 */
static uint8 s_image_binary[MT9V03X_H][MT9V03X_W];
/* 内部默认固定二值化阈值，单位灰度级 */
static uint8 s_image_threshold = IMAGE_DEFAULT_THRESHOLD;



/*
 * 函数功能：尝试抓取最新完整图像并锁存到内部缓存。
 * 输入参数：无。
 * 返回值：1-本次成功获取到新图像；0-当前没有新的完整图像。
 */
static uint8 image_latch_frame(void)
{
    if (0U == mt9v03x_finish_flag)
    {
        return 0U;
    }

    /* 锁存完整帧，避免后续处理直接读取DMA工作缓冲导致撕裂 */
    memcpy(s_image_frame[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    mt9v03x_finish_flag = 0U;
    return 1U;
}

/*
 * 函数功能：对内部灰度图执行固定阈值二值化，生成默认二值图缓存。
 * 输入参数：无。
 * 返回值：无。
 */
static void image_binary_threshold(void)
{
    uint32 i;

    /* 单次遍历完成固定阈值二值化，优先保证实时性 */
    for (i = 0U; i < MT9V03X_IMAGE_SIZE; i++)
    {
        s_image_binary[0][i] = (s_image_frame[0][i] > s_image_threshold) ? 255U : 0U;
    }
}

/*
 * 函数功能：初始化图像模块，完成摄像头、内部缓存和默认阈值初始化。
 * 输入参数：无。
 * 返回值：无。
 */
void image_init(void)
{
    memset(s_image_frame, 0, sizeof(s_image_frame));
    memset(s_image_binary, 0, sizeof(s_image_binary));

    s_image_threshold = IMAGE_DEFAULT_THRESHOLD;
    mt9v03x_finish_flag = 0U;

    (void)mt9v03x_init();
}

/*
 * 函数功能：更新图像模块，获取最新完整图像，缓存到内部缓冲区，完成二值化，并发送原始图像到上位机。
 * 输入参数：无。
 * 返回值：无。
 */
void image_update(void)
{
    if (0U == image_latch_frame())
    {
        return;
    }

    image_binary_threshold();

    // (void)wifi_image_SendFrame(s_image_binary[0],
    //                            MT9V03X_IMAGE_SIZE,
    //                            MT9V03X_W,
    //                            MT9V03X_H,
    //                            WIFI_IMAGE_VOFA_FORMAT_GRAYSCALE8,
    //                            0);
}
