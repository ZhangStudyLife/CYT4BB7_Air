#ifndef IMAGE_DEBUG_SCREEN_H_
#define IMAGE_DEBUG_SCREEN_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMAGE_DEBUG_SCREEN_MODE_DATA           (0U)
#define IMAGE_DEBUG_SCREEN_MODE_RAW            (1U)
#define IMAGE_DEBUG_SCREEN_MODE_BEACON_BINARY  (2U)
#define IMAGE_DEBUG_SCREEN_MODE_LAMP_BINARY    (3U)
#define IMAGE_DEBUG_SCREEN_MODE_OVERLAY        (4U)

/* 同步初始化IPS114硬件并绘制Data模式静态启动页面。 */
void ImageDebugScreen_Init(void);

/* 主循环每消费一个10 ms任务节拍调用一次，仅推进显示调度时钟。 */
void ImageDebugScreen_Tick10ms(void);

uint8 ImageDebugScreen_SetMode(uint8 mode);
uint8 ImageDebugScreen_GetMode(void);

/* 新帧到达且无任务积压时整帧刷新；Overlay包含彩色检测结果和下摄闭合边界。 */
void ImageDebugScreen_Update(uint8 image_frame_updated);

#ifdef __cplusplus
}
#endif

#endif /* IMAGE_DEBUG_SCREEN_H_ */
