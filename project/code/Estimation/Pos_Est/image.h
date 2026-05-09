#ifndef IMAGE_H_
#define IMAGE_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float x;        /* 飞机相对目标投影点在右侧，X为正 */
    float y;        /* 飞机相对目标投影点在前方，Y为正 */
    float radius;   /* 白色圆形目标半径，单位像素 */
    uint8 valid;    /* 白色圆形目标是否有效，1=有效，0=无效 */
} image_circle;

#define IMAGE_MAX_CIRCLE_COUNT (5U) /* 最多检测几个圆形目标 */
#define IMAGE_MIN_COMPONENT_AREA (10U) /* 连通域最小面积阈值（像素数），小于该值不输出到圆形结果数组 */
extern image_circle g_image_circles[IMAGE_MAX_CIRCLE_COUNT]; /* 当前帧白色圆形目标检测结果,数组按照从大到小排序 */





/*
 * 函数功能：初始化图像模块，完成摄像头、内部缓存和默认阈值初始化。
 * 输入参数：无。
 * 返回值：无。
 */
void image_init(void);

/*
 * 函数功能：更新图像模块，获取最新完整图像，缓存到内部缓冲区，完成二值化，并发送原始图像到上位机。
 * 输入参数：无。
 * 返回值：无。
 */
void image_update(void);

#ifdef __cplusplus
}
#endif

#endif /* IMAGE_H_ */
