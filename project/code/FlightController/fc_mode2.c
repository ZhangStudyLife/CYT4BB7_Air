#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../Protocols/wifi/wifi_justfloat/wifi_justfloat.h"

extern volatile uint32 tick_1000us_cnt;

pid_t g_mode2_velx_pid;
pid_t g_mode2_vely_pid;
float g_mode2_velx_target = 0.0f;
float g_mode2_vely_target = 0.0f;

/* Mode2 uses an independent PI velocity loop. */
static const float s_mode2_rc_to_speed_scale = 0.2f;
static const float s_mode2_vel_limit_cmps = 200.0f;
static const float s_mode2_vel_deadzone_cmps = 6.0f;
static const float s_mode2_angle_limit_deg = 15.0f;

static float FC_Mode2_ApplyDeadzone(float v, float dz)
{
    if (v > dz)
    {
        return v - dz;
    }
    if (v < -dz)
    {
        return v + dz;
    }
    return 0.0f;
}

void FC_Mode2_Init(void)
{
    PID_Init(&g_mode2_velx_pid,
             g_fc_params.mode2_vel_x_kp, g_fc_params.mode2_vel_x_ki, g_fc_params.mode2_vel_x_kd,
             g_fc_params.mode2_vel_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.mode2_vel_x_i_limit, g_fc_params.mode2_vel_x_d_lpf);
    PID_Init(&g_mode2_vely_pid,
             g_fc_params.mode2_vel_y_kp, g_fc_params.mode2_vel_y_ki, g_fc_params.mode2_vel_y_kd,
             g_fc_params.mode2_vel_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.mode2_vel_y_i_limit, g_fc_params.mode2_vel_y_d_lpf);
    FC_Mode2_Reset();
}

void FC_Mode2_Reset(void)
{
    PID_Reset(&g_mode2_velx_pid);
    PID_Reset(&g_mode2_vely_pid);
    g_mode2_velx_target = 0.0f;
    g_mode2_vely_target = 0.0f;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

void FC_Mode2_100Hz(void)
{
}

void FC_Mode2_50Hz(float dt)
{
    float ch0;
    float ch1;
    float velx_out;
    float vely_out;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        FC_Mode2_Reset();
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0], -1000.0f, 1000.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1], -1000.0f, 1000.0f);

    g_mode2_velx_target = FC_Mode2_ApplyDeadzone(
        FC_Mode_Clamp(ch0 * s_mode2_rc_to_speed_scale,
                      -s_mode2_vel_limit_cmps, s_mode2_vel_limit_cmps),
        s_mode2_vel_deadzone_cmps);
    g_mode2_vely_target = FC_Mode2_ApplyDeadzone(
        FC_Mode_Clamp(-ch1 * s_mode2_rc_to_speed_scale,
                      -s_mode2_vel_limit_cmps, s_mode2_vel_limit_cmps),
        s_mode2_vel_deadzone_cmps);

    velx_out = PID_Update(&g_mode2_velx_pid, g_mode2_velx_target, -Pos_Est_vel_x, dt);
    vely_out = PID_Update(&g_mode2_vely_pid, g_mode2_vely_target, -Pos_Est_vel_y, dt);

    roll_angle_target = FC_Mode_Clamp(velx_out + FC_Mode_Get_Roll_Mech_Trim_Deg(),
                                      -s_mode2_angle_limit_deg, s_mode2_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(vely_out + FC_Mode_Get_Pitch_Mech_Trim_Deg(),
                                       -s_mode2_angle_limit_deg, s_mode2_angle_limit_deg);

    wifi_justfloat(tick_1000us_cnt,
                   target_height_m * 1000.0f,
                   g_tof_fused_height_mm,
                   opflow_vel_x, opflow_vel_y,
                   acc_x_lp, acc_y_lp,
                   g_mode2_velx_target,
                   -Pos_Est_vel_x,
                   g_mode2_velx_pid.p_term,
                   g_mode2_velx_pid.i_term,
                   g_mode2_velx_pid.d_term,
                   g_mode2_velx_pid.output,
                   g_mode2_vely_target,
                   -Pos_Est_vel_y,
                   g_mode2_vely_pid.p_term,
                   g_mode2_vely_pid.i_term,
                   g_mode2_vely_pid.d_term,
                   g_mode2_vely_pid.output,
                   roll_angle_target,
                   pitch_angle_target,
                   g_euler.roll,
                   g_euler.pitch,
                   g_euler.yaw);
}
