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
