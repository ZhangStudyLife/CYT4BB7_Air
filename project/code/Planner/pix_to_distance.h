#ifndef PIX_TO_DISTANCE_H
#define PIX_TO_DISTANCE_H

#include "zf_common_typedef.h"

typedef struct
{
    uint8 valid;
    float x_cm;
    float y_cm;
} pix_to_distance_result_t;

// 经过我的实际测试 , 算法PixToDistance_Update_ProjectionCenter_2最好
// 也就是投影点像素和车灯融合坐标像素相减,然后再进行拟合
extern pix_to_distance_result_t g_car_lamp_fused_distance;
extern pix_to_distance_result_t g_car_lamp_fused_distance_projectioncenter;
extern pix_to_distance_result_t g_car_lamp_fused_distance_projectioncenter_2;

void PixToDistance_Init(void);
uint8 PixToDistance_Update(void);
uint8 PixToDistance_Update_ProjectionCenter(void);
uint8 PixToDistance_Update_ProjectionCenter_2(void);

#endif /* PIX_TO_DISTANCE_H */
