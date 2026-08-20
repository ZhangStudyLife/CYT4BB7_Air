#ifndef CAR_PLAN_4_H
#define CAR_PLAN_4_H

#include "car_plan_entry.h"

#define CAR_PLAN_4_DEBUG_BEACON_COUNT (4U) /* 调试输出的全局融合信标最大数量。 */

typedef struct
{
    uint8 valid;
    uint8 camera_mask;
    float center_x; /* 世界对齐局部 X 坐标，单位 m。 */
    float center_y; /* 世界对齐局部 Y 坐标，单位 m。 */
    float area;
} car_plan_4_debug_beacon_t;

typedef struct
{
    uint8 valid;
    uint8 camera_mask;
    float center_x; /* 世界对齐局部 X 坐标，单位 m。 */
    float center_y; /* 世界对齐局部 Y 坐标，单位 m。 */
    float angle_deg; /* 车灯长轴在水平全局坐标系中的无向角度，单位 deg。 */
} car_plan_4_debug_lamp_t;

typedef struct
{
    car_plan_4_debug_beacon_t beacon[CAR_PLAN_4_DEBUG_BEACON_COUNT];
    car_plan_4_debug_lamp_t car_lamp;
    int8 selected_target_id;
} car_plan_4_debug_t;

void CarPlan_4_Reset(void);

/**
 * @brief 使用三摄 Double Sphere 几何确定车灯到最近信标的目标速度，同摄组合优先于跨摄组合。
 * @param result 输出车体系横向和前向目标速度；允许传入空指针。
 * @return 成功得到可信目标方向时返回 1，否则清空输出并返回 0。
 * @note g_car_yaw 只用于消除所选车灯长轴的 180 度方向歧义，不替代图像目标方向。
 */
uint8 CarPlan_4_Update(car_plan_result_t *result);
void CarPlan_4_GetResult(car_plan_result_t *result);
void CarPlan_4_GetDebug(car_plan_4_debug_t *debug);

#endif /* CAR_PLAN_4_H */
