#ifndef CAR_PLAN_ENTRY_H
#define CAR_PLAN_ENTRY_H

#include "zf_common_typedef.h"

#define CAR_PLAN_COUNT (3U) /* 车模速度规划算法数量。 */

typedef struct
{
    uint8 valid;
    float target_strafe_mps; /* 车体右向目标速度，单位m/s。 */
    float target_forward_mps; /* 车体前向目标速度，单位m/s。 */
} car_plan_result_t;

extern float Car_Speed; /* 车模常规规划速度，单位m/s，可由车机通过AirComm修改。 */
extern float Car_Speed_Fast; /* 车模快速规划速度，单位m/s，可由车机通过AirComm修改。 */
extern int32 Car_Plan_Mode; /* 下发车模的规划算法编号，范围1至3。 */

/**
 * @brief 复位三套车模速度规划算法及其输出结果。
 * @param 无。
 * @return 无。
 */
void CarPlanEntry_Reset(void);

/**
 * @brief 依次运行三套规划算法并输出当前选中算法的结果。
 * @param result 当前选中算法的速度规划结果；允许传入空指针。
 * @return 无。
 */
void CarPlanEntry_Update(car_plan_result_t *result);

#endif /* CAR_PLAN_ENTRY_H */
