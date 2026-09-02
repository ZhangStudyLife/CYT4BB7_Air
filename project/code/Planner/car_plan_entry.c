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
#include "car_plan_entry.h"
#include "car_plan.h"
#include "car_plan_2.h"
#include "car_plan_3.h"
#include "car_plan_4.h"
#include <string.h>

float Car_Speed = 2.0f; /* 车模常规规划速度，单位m/s，可由车机通过AirComm修改。 */
float Car_Speed_Fast = 3.0f; /* 车模快速规划速度，单位m/s，可由车机通过AirComm修改。 */
int32 Car_Plan_Mode = 3; /* 下发车模的规划算法编号，范围1至4。 */

static car_plan_result_t s_car_plan_results[CAR_PLAN_COUNT]; /* 四套算法的最近一次速度规划结果。 */

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
    CarPlan_4_Reset();
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
    (void)CarPlan_4_Update(&s_car_plan_results[3]);

    if(result != 0)
    {
        *result = s_car_plan_results[Car_Plan_Mode - 1];
    }
}
