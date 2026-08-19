#include "auto_landing.h"
#include "fc_params.h"
#include "fc_start_crsf.h"
#include "yaw_align.h"
#include "../Protocols/crsf/crsf.h"

#define AUTO_LANDING_INITIAL_WAIT_TICKS (500U) /* 前置等待5s，对应100Hz调用500次。 */
#define AUTO_LANDING_NO_BEACON_TICKS    (500U) /* 连续无信标5s后允许触发降落。 */
#define AUTO_LANDING_SEARCH_ROTATION_DEG (360.0f) /* 降落前要求的实际定向搜索角，单位度。 */

static uint16 s_initial_wait_ticks = 0U; /* 自动降落前置等待计数。 */
static uint16 s_no_beacon_ticks = 0U; /* 连续无信标计数。 */
static uint8 s_auto_landing_triggered = 0U; /* 自动降落触发锁存标志。 */

/*
 * 函数功能: 以100Hz检测Mode4自动降落条件并请求现有落地程序
 * 输入参数: 无
 * 输出参数或返回值: 无
 */
void AutoLanding_Update100Hz(void)
{
    FC_START_CRSF_state_e state = FC_START_CRSF_Get_State();
    yaw_align_debug_t yaw_debug;

    if(s_auto_landing_triggered != 0U)
    {
        if((state == FC_START_CRSF_STATE_STANDBY) ||
           (state == FC_START_CRSF_STATE_INIT))
        {
            s_initial_wait_ticks = 0U;
            s_no_beacon_ticks = 0U;
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
        s_no_beacon_ticks = 0U;
        return;
    }

    if(s_initial_wait_ticks < AUTO_LANDING_INITIAL_WAIT_TICKS)
    {
        s_initial_wait_ticks++;
        return;
    }

    YawAlign_GetDebug(&yaw_debug);
    if(yaw_debug.beacon_visible != 0U)
    {
        s_no_beacon_ticks = 0U;
        return;
    }

    if(s_no_beacon_ticks < AUTO_LANDING_NO_BEACON_TICKS)
    {
        s_no_beacon_ticks++;
    }

    if((s_no_beacon_ticks >= AUTO_LANDING_NO_BEACON_TICKS) &&
       (yaw_debug.search_rotation_deg >= AUTO_LANDING_SEARCH_ROTATION_DEG))
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

/*
 * 函数功能: 获取自动降落检测的只读调试快照
 * 输入参数: debug - 调试快照输出地址，不可为空
 * 输出参数或返回值: 无
 */
void AutoLanding_GetDebug(auto_landing_debug_t *debug)
{
    yaw_align_debug_t yaw_debug;

    YawAlign_GetDebug(&yaw_debug);
    debug->initial_wait_ticks = s_initial_wait_ticks;
    debug->no_beacon_ticks = s_no_beacon_ticks;
    debug->beacon_visible = yaw_debug.beacon_visible;
    debug->rotation_ready = (yaw_debug.search_rotation_deg >=
                             AUTO_LANDING_SEARCH_ROTATION_DEG) ? 1U : 0U;
    debug->triggered = s_auto_landing_triggered;
}
