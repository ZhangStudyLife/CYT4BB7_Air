#ifndef CAR_PLAN_3_H
#define CAR_PLAN_3_H

#include "zf_common_typedef.h"

typedef struct
{
    uint8 valid;
    uint8 camera_mask;
    float target_strafe_mps;  /* 目标在车体右向轴的速度分量，单位 m/s。 */
    float target_forward_mps; /* 目标在车体前向轴的速度分量，单位 m/s。 */
    float target_center_x; /* 所选信标的世界对齐局部 X 坐标，单位 m。 */
    float target_center_y; /* 所选信标的世界对齐局部 Y 坐标，单位 m。 */
} car_plan_3_result_t;

#define CAR_PLAN_3_DEBUG_BEACON_COUNT (4U) /* 调试输出的全局融合信标最大数量。 */

typedef struct
{
    uint8 valid;
    uint8 camera_mask;
    float center_x; /* 世界对齐局部 X 坐标，单位 m。 */
    float center_y; /* 世界对齐局部 Y 坐标，单位 m。 */
    float area;
} car_plan_3_debug_beacon_t;

typedef struct
{
    uint8 valid;
    uint8 camera_mask;
    float center_x; /* 世界对齐局部 X 坐标，单位 m。 */
    float center_y; /* 世界对齐局部 Y 坐标，单位 m。 */
    float angle_deg; /* 车灯长轴在水平全局坐标系中的无向角度，单位 deg。 */
} car_plan_3_debug_lamp_t;

typedef struct
{
    car_plan_3_debug_beacon_t beacon[CAR_PLAN_3_DEBUG_BEACON_COUNT];
    car_plan_3_debug_lamp_t car_lamp;
    int8 selected_target_id;
} car_plan_3_debug_t;

void CarPlan_3_Reset(void);

/*
 * 使用三摄图像确定车灯到最近信标的目标向量，并使用融合车灯长轴建立车体横轴。
 * g_car_yaw 只用于消除车灯长轴的 180 度方向歧义，不替代图像目标方向。
 */
uint8 CarPlan_3_Update(car_plan_3_result_t *result);
void CarPlan_3_GetResult(car_plan_3_result_t *result);
void CarPlan_3_GetDebug(car_plan_3_debug_t *debug);

#endif /* CAR_PLAN_3_H */
