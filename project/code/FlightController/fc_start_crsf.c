#include "fc_start_crsf.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "zf_common_headfile.h"

#define FC_START_CRSF_TASK_PERIOD_MS (100U)
#define FC_START_CRSF_LANDING_HOLD_TICKS_100HZ (20U)
#define FC_START_CRSF_LANDING_STOP_HEIGHT_MM (200.0f)
#define FC_START_CRSF_TAKEOFF_PAIR1_RAMP_MS (800U)
#define FC_START_CRSF_TAKEOFF_PAIR2_RAMP_MS (800U)
#define FC_START_CRSF_TAKEOFF_TO_2500_RAMP_MS (800U)
#define FC_START_CRSF_TAKEOFF_HOLD_2500_MS (2000U)
#define FC_START_CRSF_TAKEOFF_THR_STAGE1 (1500)
#define FC_START_CRSF_TAKEOFF_THR_STAGE2 (2500)

FC_START_CRSF_state_e s_fc_start_state = FC_START_CRSF_STATE_INIT;
static FC_START_CRSF_flight_mode_e s_fc_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;

static uint16_t s_unlock_timer_tick = 0U;
static uint16_t s_takeoff_timer_tick = 0U;
static uint8_t s_landing_button_tick = 0U;
static uint8_t s_landing_low_tick = 0U;
static uint8_t s_motor_armed = 0U;
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
        if (zero_streak < 8u)
        {
            zero_streak++;
        }
    }
    else
    zero_streak = 0U;
    {
    }
    return (zero_streak >= 3U) ? 1U : 0U;
}


static void FC_START_CRSF_UpdateModeFromCH5CH6(void)
{
    s_fc_flight_mode = (FC_START_CRSF_flight_mode_e)(CRSF_STD[5] * 3 + CRSF_STD[6]);
}

static void FC_START_CRSF_ForceStopToStandby(void)
{
    s_motor_armed = 0U;
    s_takeoff_timer_tick = 0U;
    s_unlock_timer_tick = 0U;
    s_landing_button_tick = 0U;
    s_landing_low_tick = 0U;
    s_takeoff_calib_done = 0U;   /* 重置校准标志，下次起飞时重新执行校准 */

    Motor_EmergencyStop();
    Motor_Disable();

    s_fc_start_state = FC_START_CRSF_STATE_STANDBY;
}

#define FC_START_CRSF_TAKEOFF_PAIR1_MOTOR_A (MOTOR_1)
#define FC_START_CRSF_TAKEOFF_PAIR1_MOTOR_B (MOTOR_4)
#define FC_START_CRSF_TAKEOFF_PAIR2_MOTOR_A (MOTOR_2)
#define FC_START_CRSF_TAKEOFF_PAIR2_MOTOR_B (MOTOR_4)

static uint8_t FC_START_CRSF_TakeoffState_Update(void)
{
    // 新起飞逻辑：初始4电机油门1000，对角电机1/4从1000线性加速到1500（1秒），其余保持1000，加速后全部1500保持2秒
    const uint16_t ramp_ticks = 10U; // 1秒（100ms*10）
    const uint16_t hold_ticks = 20U; // 2秒（100ms*20）
    const uint16_t total_ticks = ramp_ticks + hold_ticks;
    int32_t motor_throttle[MOTOR_NUM] = {1000, 1000, 1000, 1000};

    if (!Motor_IsEnabled()) {
        Motor_Enable();
    }

    if (s_takeoff_timer_tick < total_ticks) {
        s_takeoff_timer_tick++;
    }

    if (s_takeoff_timer_tick <= ramp_ticks) {
        // 对角电机M1/M4从1000线性加速到1500
        int32_t ramp = 1000 + (int32_t)((500 * s_takeoff_timer_tick) / ramp_ticks);
        motor_throttle[0] = ramp; // MOTOR_1
        motor_throttle[3] = ramp; // MOTOR_4
        // 其余电机保持1000
        motor_throttle[1] = 1000; // MOTOR_2
        motor_throttle[2] = 1000; // MOTOR_3
    } else {
        // 所有电机保持1500
        for (uint8_t i = 0U; i < MOTOR_NUM; i++) {
            motor_throttle[i] = 1500;
        }
    }

    for (uint8_t i = 0U; i < MOTOR_NUM; i++) {
        Motor_SetThrottle((motor_index_e)i, motor_throttle[i]);
    }

    return (s_takeoff_timer_tick >= total_ticks) ? 1U : 0U;
}

static void FC_START_CRSF_PrepareTakeoff(void)
{
    /* Keep 100Hz task non-blocking, avoid tick backlog compressing takeoff timeline. */
    Motor_Init();
    Motor_Disable();
}

static void FC_START_CRSF_StateMachine_Update(void)
{
    if (CRSF_LINK_UP == 0U) { FC_START_CRSF_ForceStopToStandby(); return; }
    if (FC_START_CRSF_IsEmergencyStopRequested() != 0U) { if (s_fc_start_state >= FC_START_CRSF_STATE_TAKEOFF) { Beep_Play(50, 2, 3); } FC_START_CRSF_ForceStopToStandby(); return; }

    switch (s_fc_start_state)
    {
    case FC_START_CRSF_STATE_INIT:
        s_motor_armed = 0U;
        s_unlock_timer_tick = 0U;
        s_takeoff_timer_tick = 0U;
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
            // TOF_Calibrate();
            s_takeoff_calib_done = 1U;
            // if (0U == g_tof_calibration_ok)
            // {
            //     /* 四路TOF偏差过大（>100mm），拒绝起飞，回到待机状态 */
            //     Beep_Play(50,2,3); /* 50%占空比，2s周期，3次循环的报警提示 */
            //     FC_START_CRSF_ForceStopToStandby();
            //     break;
            // }
            // break;
        }
        /* 阶段2：正常起飞序列（电机渐进加速） */
        if (FC_START_CRSF_TakeoffState_Update() != 0U)
        {
            s_motor_armed = 1U;
            s_fc_start_state = FC_START_CRSF_STATE_FLYING;
        }
        break;

    case FC_START_CRSF_STATE_FLYING:
        FC_START_CRSF_UpdateModeFromCH5CH6();
        break;

    case FC_START_CRSF_STATE_LANDING:
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
    s_landing_button_tick = 0U;
    s_landing_low_tick = 0U;
    s_motor_armed = 0U;
    s_takeoff_calib_done = 0U;

    Motor_Disable();
}

void FC_START_CRSF_Update(void)
{
    FC_START_CRSF_StateMachine_Update();
}

void FC_START_CRSF_UpdateLandingButton100Hz(void)
{
    if ((s_fc_start_state == FC_START_CRSF_STATE_FLYING) && (CRSF_STD[8] == 1))
    {
        if (s_landing_button_tick < FC_START_CRSF_LANDING_HOLD_TICKS_100HZ)
        {
            s_landing_button_tick++;
        }
        if (s_landing_button_tick >= FC_START_CRSF_LANDING_HOLD_TICKS_100HZ)
        {
            FC_START_CRSF_Request_Landing();
        }
    }
    else
    {
        s_landing_button_tick = 0U;
    }

    if (s_fc_start_state == FC_START_CRSF_STATE_LANDING)
    {
        if ((0U != g_tof_fused_valid) && (g_tof_fused_height_mm < FC_START_CRSF_LANDING_STOP_HEIGHT_MM))
        {
            if (s_landing_low_tick < FC_START_CRSF_LANDING_HOLD_TICKS_100HZ)
            {
                s_landing_low_tick++;
            }
            if (s_landing_low_tick >= FC_START_CRSF_LANDING_HOLD_TICKS_100HZ)
            {
                FC_START_CRSF_ForceStopToStandby();
            }
        }
        else
        {
            s_landing_low_tick = 0U;
        }
    }
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
        s_landing_low_tick = 0U;
        s_fc_start_state = FC_START_CRSF_STATE_LANDING;
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
