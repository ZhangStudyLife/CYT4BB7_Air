#include "car_plan_entry.h"
#include "car_plan.h"
#include "car_plan_2.h"
#include "car_plan_3.h"
#include <string.h>

float Car_Speed = 2.0f; /* 车模常规规划速度，单位m/s，可由车机通过AirComm修改。 */
float Car_Speed_Fast = 2.0f; /* 车模快速规划速度，单位m/s，可由车机通过AirComm修改。 */
int32 Car_Plan_Mode = 3; /* 下发车模的规划算法编号，范围1至3。 */

static car_plan_result_t s_car_plan_results[CAR_PLAN_COUNT]; /* 三套算法的最近一次速度规划结果。 */

/**
 * @brief 复位三套车模速度规划算法及其输出结果。
 * @param 无。
 * @return 无。
 */
void CarPlanEntry_Reset(void)
{
    memset(s_car_plan_results, 0, sizeof(s_car_plan_results));
    CarPlan_Reset();
    CarPlan_2_Reset();
    CarPlan_3_Reset();
}

/**
 * @brief 依次运行三套规划算法并输出当前选中算法的结果。
 * @param result 当前选中算法的速度规划结果；允许传入空指针。
 * @return 无。
 */
void CarPlanEntry_Update(car_plan_result_t *result)
{
    (void)CarPlan_Update(&s_car_plan_results[0]);
    (void)CarPlan_2_Update(&s_car_plan_results[1]);
    (void)CarPlan_3_Update(&s_car_plan_results[2]);

    if(result != 0)
    {
        *result = s_car_plan_results[Car_Plan_Mode - 1];
    }
}
