#include "speed_plan.h"
#include "car_plan_entry.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../FlightController/fc_loop.h"
#include <math.h>

#define SPEED_PLAN_MIN_FORWARD_MPS       (0.5f)
#define SPEED_PLAN_ENTER_TTC_S           (0.50f)
#define SPEED_PLAN_ENTER_ANGLE_DEG       (24.0f)
#define SPEED_PLAN_ENTER_YAWRATE_DPS     (60.0f)
#define SPEED_PLAN_ENTER_ATTITUDE_DEG    (20.0f)
#define SPEED_PLAN_EXIT_TTC_S            (0.20f)
#define SPEED_PLAN_EXIT_ANGLE_DEG        (30.0f)
#define SPEED_PLAN_EXIT_YAWRATE_DPS      (100.0f)
#define SPEED_PLAN_EXIT_ATTITUDE_DEG     (30.0f)
#define SPEED_PLAN_CONFIRM_TICKS         (6U) /* 100Hz下连续约60ms。 */

extern float g_car_vel_y;
extern float g_car_yaw_rate_dps;

static uint8 s_speed_plan_fast = 0U;
static uint8 s_speed_plan_condition_ticks = 0U;

void SpeedPlan_Reset(void)
{
    s_speed_plan_fast = 0U;
    s_speed_plan_condition_ticks = 0U;
}

float SpeedPlan_Update(uint8 target_valid,
                       float target_distance_m,
                       float target_angle_deg)
{
    float forward_speed;
    float yaw_error;
    float attitude_error;
    float ttc;
    uint8 condition_met;

    if(target_valid == 0U)
    {
        s_speed_plan_condition_ticks = 0U;
        return (s_speed_plan_fast != 0U) ? Car_Speed_Fast : Car_Speed;
    }

    forward_speed = (g_car_vel_y > SPEED_PLAN_MIN_FORWARD_MPS) ?
                        g_car_vel_y : SPEED_PLAN_MIN_FORWARD_MPS;
    yaw_error = g_euler.yaw - yaw_angle_target;
    if(yaw_error > 180.0f)
    {
        yaw_error -= 360.0f;
    }
    else if(yaw_error < -180.0f)
    {
        yaw_error += 360.0f;
    }
    attitude_error = sqrtf((g_euler.roll - roll_angle_target) *
                               (g_euler.roll - roll_angle_target) +
                           (g_euler.pitch - pitch_angle_target) *
                               (g_euler.pitch - pitch_angle_target) +
                           yaw_error * yaw_error);
    ttc = target_distance_m / forward_speed;

    if(s_speed_plan_fast == 0U)
    {
        condition_met = ((ttc > SPEED_PLAN_ENTER_TTC_S) &&
                         (target_angle_deg < SPEED_PLAN_ENTER_ANGLE_DEG) &&
                         (fabsf(g_car_yaw_rate_dps) < SPEED_PLAN_ENTER_YAWRATE_DPS) &&
                         (attitude_error < SPEED_PLAN_ENTER_ATTITUDE_DEG)) ? 1U : 0U;
    }
    else
    {
        condition_met = ((ttc < SPEED_PLAN_EXIT_TTC_S) ||
                         (target_angle_deg > SPEED_PLAN_EXIT_ANGLE_DEG) ||
                         (fabsf(g_car_yaw_rate_dps) > SPEED_PLAN_EXIT_YAWRATE_DPS) ||
                         (attitude_error > SPEED_PLAN_EXIT_ATTITUDE_DEG)) ? 1U : 0U;
    }

    if(condition_met != 0U)
    {
        s_speed_plan_condition_ticks++;
        if(s_speed_plan_condition_ticks >= SPEED_PLAN_CONFIRM_TICKS)
        {
            s_speed_plan_fast = (s_speed_plan_fast == 0U) ? 1U : 0U;
            s_speed_plan_condition_ticks = 0U;
        }
    }
    else
    {
        s_speed_plan_condition_ticks = 0U;
    }

    return (s_speed_plan_fast != 0U) ? Car_Speed_Fast : Car_Speed;
}
