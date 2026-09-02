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
#ifndef PIX_TO_DISTANCE_H
#define PIX_TO_DISTANCE_H

#include "zf_common_typedef.h"

typedef struct
{
    uint8 valid;
    float x_cm;
    float y_cm;
} pix_to_distance_result_t;

// 经过我的实际测试 , 算法PixToDistance_UpdateProjectionCenter2_100Hz最好
// 也就是投影点像素和车灯融合坐标像素相减,然后再进行拟合
extern pix_to_distance_result_t g_car_lamp_fused_distance;
extern pix_to_distance_result_t g_car_lamp_fused_distance_projectioncenter;
extern pix_to_distance_result_t g_car_lamp_fused_distance_projectioncenter_2;

void PixToDistance_Init(void);
uint8 PixToDistance_Update(void);
uint8 PixToDistance_Update_ProjectionCenter(void);
uint8 PixToDistance_UpdateProjectionCenter2_100Hz(void);

#endif /* PIX_TO_DISTANCE_H */
