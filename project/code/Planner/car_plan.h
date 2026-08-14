#ifndef CAR_PLAN_H
#define CAR_PLAN_H

#include "car_plan_entry.h"

#define CAR_PLAN_LOCK_SWITCH_MARGIN_PX     (25.0f)
#define CAR_PLAN_LOCK_LOST_HOLD_TICKS      (30U)

void CarPlan_Reset(void);
uint8 CarPlan_Update(car_plan_result_t *result);
void CarPlan_GetResult(car_plan_result_t *result);

#endif /* CAR_PLAN_H */
