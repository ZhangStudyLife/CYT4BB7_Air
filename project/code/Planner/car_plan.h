#ifndef CAR_PLAN_H
#define CAR_PLAN_H

#include "zf_common_headfile.h"

#define CAR_PLAN_LOCK_SWITCH_MARGIN_PX     (25.0f)
#define CAR_PLAN_LOCK_LOST_HOLD_TICKS      (30U)

extern float Car_Speed; /* 车模规划速度，单位 m/s，可由车机通过 AirComm 修改 */
extern float Car_Speed_Fast; /* 车模固定前进速度，单位 m/s，可由车机通过 AirComm 修改 */
extern float Car_Plan_Mode; /* 车模规划算法选择：1=car_plan，2=car_plan_2，可由车机通过 AirComm 修改 */

typedef struct
{
    uint8 valid;
    uint8 camera;
    uint8 beacon_index;
    float target_strafe_mps;
    float target_forward_mps;
    float dist_px;
    float along;
    float perp;
} car_plan_result_t;

void CarPlan_Reset(void);
uint8 CarPlan_Update(car_plan_result_t *result);
void CarPlan_GetResult(car_plan_result_t *result);

#endif /* CAR_PLAN_H */
