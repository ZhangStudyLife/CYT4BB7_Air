#include "fc_start_crsf.h"
#include "zf_common_headfile.h"

#define FC_START_CRSF_TASK_PERIOD_MS (100U)
#define FC_START_CRSF_TAKEOFF_PAIR1_RAMP_MS (800U)
#define FC_START_CRSF_TAKEOFF_PAIR2_RAMP_MS (800U)
#define FC_START_CRSF_TAKEOFF_TO_2500_RAMP_MS (800U)
#define FC_START_CRSF_TAKEOFF_HOLD_2500_MS (2000U)
#define FC_START_CRSF_TAKEOFF_THR_STAGE1 (1500)
#define FC_START_CRSF_TAKEOFF_THR_STAGE2 (2500)

static FC_START_CRSF_state_e s_fc_start_state = FC_START_CRSF_STATE_INIT;
static FC_START_CRSF_flight_mode_e s_fc_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;

static uint16_t s_unlock_timer_tick = 0U;
static uint16_t s_takeoff_timer_tick = 0U;
static uint8_t s_motor_armed = 0U;
static uint8_t s_landing_request = 0U;
/* 起飞前校准是否已完成：0=未执行，1=已执行（进入TAKEOFF状态时首次触发校准） */
static uint8_t s_takeoff_calib_done = 0U;

static uint8_t FC_START_CRSF_IsUnlockStickCommand(void)
{
    const int16_t throttle = CRSF_STD[2];
    const int16_t roll = CRSF_STD[0];
    const int16_t pitch = CRSF_STD[1];
    const int16_t yaw = CRSF_STD[3];

    if ((throttle < FC_START_CRSF_RC_RAW_LOW_THR) &&
        (roll < FC_START_CRSF_RC_RAW_LOW_THR) &&
        (pitch < FC_START_CRSF_RC_RAW_LOW_THR) &&
        (yaw > FC_START_CRSF_RC_RAW_HIGH_THR))
    {
        return 1U;
    }

    return 0U;
}

static uint8_t FC_START_CRSF_IsEmergencyStopRequested(void)
{
    static uint8_t zero_streak = 0U;

    if (CRSF_STD[7] == 0)
    {
        if (zero_streak < 3U)
        {
            zero_streak++;
        }
    }
    else
    {
        zero_streak = 0U;
    }
    return (zero_streak >= 3U) ? 1U : 0U;
}


static void FC_START_CRSF_UpdateModeFromCH6(void)
{
    const int16_t mode_sel = CRSF_STD[6];

    if (mode_sel <= 0)
    {
        s_fc_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
    }
    else if (mode_sel == 1)
    {
        s_fc_flight_mode = FC_START_CRSF_FLIGHT_MODE_1;
    }
    else
    {
        s_fc_flight_mode = FC_START_CRSF_FLIGHT_MODE_2;
    }
}

static void FC_START_CRSF_ForceStopToStandby(void)
{
    s_motor_armed = 0U;
    s_takeoff_timer_tick = 0U;
    s_unlock_timer_tick = 0U;
    s_landing_request = 0U;
    s_takeoff_calib_done = 0U;   /* 重置校准标志，下次起飞时重新执行校准 */

    Motor_EmergencyStop();
    Motor_Disable();

    s_fc_start_state = FC_START_CRSF_STATE_STANDBY;
}

static void FC_START_CRSF_Mode0_Update(void)
{
    /* TODO: add CH6 mode 0 behavior */
}

static void FC_START_CRSF_Mode1_Update(void)
{
    /* TODO: add CH6 mode 1 behavior */
}

static void FC_START_CRSF_Mode2_Update(void)
{
    /* TODO: add CH6 mode 2 behavior */
}

static void FC_START_CRSF_RunFlightModeHook(void)
{
    switch (s_fc_flight_mode)
    {
    case FC_START_CRSF_FLIGHT_MODE_0:
        FC_START_CRSF_Mode0_Update();
        break;

    case FC_START_CRSF_FLIGHT_MODE_1:
        FC_START_CRSF_Mode1_Update();
        break;

    case FC_START_CRSF_FLIGHT_MODE_2:
    default:
        FC_START_CRSF_Mode2_Update();
        break;
    }
}

#define FC_START_CRSF_TAKEOFF_PAIR1_MOTOR_A (MOTOR_1)
#define FC_START_CRSF_TAKEOFF_PAIR1_MOTOR_B (MOTOR_4)
#define FC_START_CRSF_TAKEOFF_PAIR2_MOTOR_A (MOTOR_2)
#define FC_START_CRSF_TAKEOFF_PAIR2_MOTOR_B (MOTOR_4)

static uint16_t FC_START_CRSF_MsToTicks(uint16_t time_ms)
{
    uint16_t ticks = (uint16_t)(time_ms / FC_START_CRSF_TASK_PERIOD_MS);
    return (ticks > 0U) ? ticks : 1U;
}

static uint8_t FC_START_CRSF_TakeoffState_Update(void)
{
    const uint16_t pair1_ramp_ticks = FC_START_CRSF_MsToTicks(FC_START_CRSF_TAKEOFF_PAIR1_RAMP_MS);
    const uint16_t pair2_ramp_ticks = FC_START_CRSF_MsToTicks(FC_START_CRSF_TAKEOFF_PAIR2_RAMP_MS);
    const uint16_t all_ramp_ticks = FC_START_CRSF_MsToTicks(FC_START_CRSF_TAKEOFF_TO_2500_RAMP_MS);
    const uint16_t hold_ticks = FC_START_CRSF_MsToTicks(FC_START_CRSF_TAKEOFF_HOLD_2500_MS);
    const uint16_t phase1_end = pair1_ramp_ticks;
    const uint16_t phase2_end = (uint16_t)(phase1_end + pair2_ramp_ticks);
    const uint16_t phase3_end = (uint16_t)(phase2_end + all_ramp_ticks);
    const uint16_t phase4_end = (uint16_t)(phase3_end + hold_ticks);
    int32_t motor_throttle[MOTOR_NUM] = {0};
    uint16_t phase_tick = 0U;
    int32_t ramp_throttle = 0;

    /* TODO: replace with closed-loop takeoff behavior */
    if (!Motor_IsEnabled())
    {
        Motor_Enable();
    }

    if (s_takeoff_timer_tick < phase4_end)
    {
        s_takeoff_timer_tick++;
    }

    if (s_takeoff_timer_tick <= phase1_end)
    {
        phase_tick = s_takeoff_timer_tick;
        ramp_throttle = (int32_t)((FC_START_CRSF_TAKEOFF_THR_STAGE1 * (int32_t)phase_tick) / (int32_t)pair1_ramp_ticks);
        motor_throttle[FC_START_CRSF_TAKEOFF_PAIR1_MOTOR_A] = ramp_throttle;
        motor_throttle[FC_START_CRSF_TAKEOFF_PAIR1_MOTOR_B] = ramp_throttle;
    }
    else if (s_takeoff_timer_tick <= phase2_end)
    {
        phase_tick = (uint16_t)(s_takeoff_timer_tick - phase1_end);
        ramp_throttle = (int32_t)((FC_START_CRSF_TAKEOFF_THR_STAGE1 * (int32_t)phase_tick) / (int32_t)pair2_ramp_ticks);
        motor_throttle[FC_START_CRSF_TAKEOFF_PAIR1_MOTOR_A] = FC_START_CRSF_TAKEOFF_THR_STAGE1;
        motor_throttle[FC_START_CRSF_TAKEOFF_PAIR1_MOTOR_B] = FC_START_CRSF_TAKEOFF_THR_STAGE1;
        motor_throttle[FC_START_CRSF_TAKEOFF_PAIR2_MOTOR_A] = ramp_throttle;

        if ((FC_START_CRSF_TAKEOFF_PAIR2_MOTOR_B == FC_START_CRSF_TAKEOFF_PAIR1_MOTOR_A) ||
            (FC_START_CRSF_TAKEOFF_PAIR2_MOTOR_B == FC_START_CRSF_TAKEOFF_PAIR1_MOTOR_B))
        {
            motor_throttle[FC_START_CRSF_TAKEOFF_PAIR2_MOTOR_B] = FC_START_CRSF_TAKEOFF_THR_STAGE1;
        }
        else
        {
            motor_throttle[FC_START_CRSF_TAKEOFF_PAIR2_MOTOR_B] = ramp_throttle;
        }
    }
    else if (s_takeoff_timer_tick <= phase3_end)
    {
        phase_tick = (uint16_t)(s_takeoff_timer_tick - phase2_end);
        ramp_throttle = FC_START_CRSF_TAKEOFF_THR_STAGE1 +
                        (int32_t)(((FC_START_CRSF_TAKEOFF_THR_STAGE2 - FC_START_CRSF_TAKEOFF_THR_STAGE1) * (int32_t)phase_tick) /
                                  (int32_t)all_ramp_ticks);

        for (uint8_t i = 0U; i < MOTOR_NUM; i++)
        {
            motor_throttle[i] = ramp_throttle;
        }
    }
    else
    {
        for (uint8_t i = 0U; i < MOTOR_NUM; i++)
        {
            motor_throttle[i] = FC_START_CRSF_TAKEOFF_THR_STAGE2;
        }
    }

    for (uint8_t i = 0U; i < MOTOR_NUM; i++)
    {
        Motor_SetThrottle((motor_index_e)i, motor_throttle[i]);
    }

    return (s_takeoff_timer_tick >= phase4_end) ? 1U : 0U;
}

static uint8_t FC_START_CRSF_LandingState_Update(void)
{
    /* TODO: add closed-loop landing behavior */
    return 1U;
}

static void FC_START_CRSF_PrepareTakeoff(void)
{
    /* Keep 100Hz task non-blocking, avoid tick backlog compressing takeoff timeline. */
    Motor_Init();
    Motor_Disable();
}

static void FC_START_CRSF_StateMachine_Update(void)
{
    if ((CRSF_LINK_UP == 0U) || (FC_START_CRSF_IsEmergencyStopRequested() != 0U))
    {
        FC_START_CRSF_ForceStopToStandby();
        return;
    }

    switch (s_fc_start_state)
    {
    case FC_START_CRSF_STATE_INIT:
        s_motor_armed = 0U;
        s_unlock_timer_tick = 0U;
        s_takeoff_timer_tick = 0U;
        s_landing_request = 0U;
        Motor_Disable();
        s_fc_start_state = FC_START_CRSF_STATE_STANDBY;
        break;

    case FC_START_CRSF_STATE_STANDBY:
        if (FC_START_CRSF_IsUnlockStickCommand() != 0U)
        {
            s_unlock_timer_tick++;
        }
        else
        {
            s_unlock_timer_tick = 0U;
        }

        if (s_unlock_timer_tick >= (uint16_t)(FC_START_CRSF_UNLOCK_HOLD_TIME_MS / FC_START_CRSF_TASK_PERIOD_MS))
        {
            
            FC_START_CRSF_PrepareTakeoff();
            s_takeoff_timer_tick = 0U;
            s_fc_start_state = FC_START_CRSF_STATE_TAKEOFF;
        }
        break;

    case FC_START_CRSF_STATE_TAKEOFF:
        /* 阶段1：起飞前TOF校准（仅在电机启动前执行一次）
         * TOF_Calibrate()会阻塞约1.5秒（500ms IMU采样 + 1000ms TOF采样）。
         * 校准完成后检查四路TOF偏差：超过10cm则回STANDBY拒绝起飞。 */
        if (0U == s_takeoff_calib_done)
        {
            TOF_Calibrate();
            s_takeoff_calib_done = 1U;
            if (0U == g_tof_calibration_ok)
            {
                /* 四路TOF偏差过大（>100mm），拒绝起飞，回到待机状态 */
                Beep_Play(50,2,3); /* 50%占空比，2s周期，3次循环的报警提示 */
                FC_START_CRSF_ForceStopToStandby();
                break;
            }
            break;
        }
        /* 阶段2：正常起飞序列（电机渐进加速） */
        if (FC_START_CRSF_TakeoffState_Update() != 0U)
        {
            s_motor_armed = 1U;
            s_fc_start_state = FC_START_CRSF_STATE_FLYING;
        }
        break;

    case FC_START_CRSF_STATE_FLYING:
        FC_START_CRSF_UpdateModeFromCH6();
        FC_START_CRSF_RunFlightModeHook();
        if (s_landing_request != 0U)
        {
            s_fc_start_state = FC_START_CRSF_STATE_LANDING;
        }
        break;

    case FC_START_CRSF_STATE_LANDING:
        if (FC_START_CRSF_LandingState_Update() != 0U)
        {
            FC_START_CRSF_ForceStopToStandby();
        }
        break;

    default:
        FC_START_CRSF_ForceStopToStandby();
        break;
    }
}

void FC_START_CRSF_Init(void)
{
    s_fc_start_state = FC_START_CRSF_STATE_INIT;
    s_fc_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
    s_unlock_timer_tick = 0U;
    s_takeoff_timer_tick = 0U;
    s_motor_armed = 0U;
    s_landing_request = 0U;
    s_takeoff_calib_done = 0U;

    Motor_Disable();
}

void FC_START_CRSF_Update(void)
{
    FC_START_CRSF_StateMachine_Update();
}

FC_START_CRSF_state_e FC_START_CRSF_Get_State(void)
{
    return s_fc_start_state;
}

FC_START_CRSF_flight_mode_e FC_START_CRSF_Get_Flight_Mode(void)
{
    return s_fc_flight_mode;
}

uint8_t FC_START_CRSF_Is_Armed(void)
{
    return s_motor_armed;
}

void FC_START_CRSF_Trigger_Emergency_Stop(void)
{
    FC_START_CRSF_ForceStopToStandby();
}

void FC_START_CRSF_Request_Landing(void)
{
    if (s_fc_start_state == FC_START_CRSF_STATE_FLYING)
    {
        s_landing_request = 1U;
    }
}

uint16_t FC_START_CRSF_Get_Unlock_Timer(void)
{
    return s_unlock_timer_tick;
}

uint16_t FC_START_CRSF_Get_Arm_Delay_Timer(void)
{
    return s_takeoff_timer_tick;
}
