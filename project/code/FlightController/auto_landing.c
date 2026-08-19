#include "auto_landing.h"
#include "fc_params.h"
#include "fc_start_crsf.h"
#include "../Planner/car_plan_3.h"
#include "../Protocols/crsf/crsf.h"

#define AUTO_LANDING_INITIAL_WAIT_TICKS (500U) /* 前置等待5s，对应100Hz调用500次。 */
#define AUTO_LANDING_NO_TARGET_TICKS    (500U) /* 连续无可信目标5s后触发降落。 */
#define AUTO_LANDING_VALID_TICKS        (20U)  /* CarPlan3连续有效200ms才确认目标。 */

static uint16 s_initial_wait_ticks = 0U; /* 自动降落前置等待计数。 */
static uint16 s_no_target_ticks = 0U; /* 连续无可信目标计数。 */
static uint16 s_valid_target_ticks = 0U; /* CarPlan3连续有效计数。 */
static uint8 s_auto_landing_triggered = 0U; /* 自动降落触发锁存标志。 */

/*
 * 函数功能: 以100Hz检测Mode4自动降落条件并请求现有落地程序
 * 输入参数: 无
 * 输出参数或返回值: 无
 */
void AutoLanding_Update100Hz(void)
{
    FC_START_CRSF_state_e state = FC_START_CRSF_Get_State();
    car_plan_result_t plan3_result;

    if(s_auto_landing_triggered != 0U)
    {
        if((state == FC_START_CRSF_STATE_STANDBY) ||
           (state == FC_START_CRSF_STATE_INIT))
        {
            s_initial_wait_ticks = 0U;
            s_no_target_ticks = 0U;
            s_valid_target_ticks = 0U;
            s_auto_landing_triggered = 0U;
        }
        return;
    }

    if((state != FC_START_CRSF_STATE_FLYING) ||
       (FC_START_CRSF_Get_Flight_Mode() != FC_START_CRSF_FLIGHT_MODE_4) ||
       (g_fc_params.yaw_change_mode4 < 1.5f) ||
       (CRSF_STD[4] != 1))
    {
        s_initial_wait_ticks = 0U;
        s_no_target_ticks = 0U;
        s_valid_target_ticks = 0U;
        return;
    }

    if(s_initial_wait_ticks < AUTO_LANDING_INITIAL_WAIT_TICKS)
    {
        s_initial_wait_ticks++;
        return;
    }

    CarPlan_3_GetResult(&plan3_result);
    if(plan3_result.valid != 0U)
    {
        if(s_valid_target_ticks < AUTO_LANDING_VALID_TICKS)
        {
            s_valid_target_ticks++;
        }
        if(s_valid_target_ticks >= AUTO_LANDING_VALID_TICKS)
        {
            s_no_target_ticks = 0U;
            return;
        }
    }
    else
    {
        s_valid_target_ticks = 0U;
    }

    if(s_no_target_ticks < AUTO_LANDING_NO_TARGET_TICKS)
    {
        s_no_target_ticks++;
    }
    if(s_no_target_ticks >= AUTO_LANDING_NO_TARGET_TICKS)
    {
        s_auto_landing_triggered = 1U;
        FC_START_CRSF_Request_Landing();
    }
}

/*
 * 函数功能: 查询自动降落触发锁存状态
 * 输入参数: 无
 * 输出参数或返回值: 1表示自动降落已触发，0表示未触发
 */
uint8 AutoLanding_IsTriggered(void)
{
    return s_auto_landing_triggered;
}
