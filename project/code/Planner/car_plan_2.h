#ifndef CAR_PLAN_2_H
#define CAR_PLAN_2_H

#include "zf_common_headfile.h"

typedef struct
{
    uint8 valid;
    uint8 camera_mask;
    float target_strafe_mps;
    float target_forward_mps;
    float target_center_x;
    float target_center_y;
} car_plan_2_result_t;

/**
 * @brief 重置影子车模规划器的目标锁定状态和输出结果。
 * @param 无。
 * @return 无。
 */
void CarPlan_2_Reset(void);

/**
 * @brief 扫描三摄信标候选并更新影子车模速度规划结果。
 * @param result 输出规划结果；允许传入空指针。
 * @return 规划结果有效时返回1，否则返回0。
 */
uint8 CarPlan_2_Update(car_plan_2_result_t *result);

/**
 * @brief 获取最近一次影子车模规划结果的快照。
 * @param result 输出规划结果；允许传入空指针。
 * @return 无。
 */
void CarPlan_2_GetResult(car_plan_2_result_t *result);

#endif /* CAR_PLAN_2_H */
