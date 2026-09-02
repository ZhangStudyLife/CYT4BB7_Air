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
#ifndef BEACON_LOST_DETECTOR_H
#define BEACON_LOST_DETECTOR_H

#include "zf_common_typedef.h"
#include "../Image/image_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BEACON_LOST_SQUARE_LEFT_PX       (25.0f)
#define BEACON_LOST_SQUARE_RIGHT_PX      (25.0f)
#define BEACON_LOST_SQUARE_UP_PX         (15.0f)
#define BEACON_LOST_SQUARE_DOWN_PX       (30.0f)
#define BEACON_LOST_DISAPPEAR_RADIUS_PX  (10.0f)

extern uint8 g_beacon_lost_flag;

void BeaconLostDetector_Init(void);
uint8 BeaconLostDetector_Update(void);
uint8 BeaconLostDetector_GetFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* BEACON_LOST_DETECTOR_H */
