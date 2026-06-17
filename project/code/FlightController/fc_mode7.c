#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include <math.h>

pid_t g_mode7_velx_pid;
pid_t g_mode7_vely_pid;
float g_mode7_velx_target = 0.0f;
float g_mode7_vely_target = 0.0f;

/* Mode7 uses an independent PI velocity loop. */
static const float s_mode7_vel_limit_cmps = 200.0f;
static const float s_mode7_vel_deadzone_cmps = 6.0f;
static const float s_mode7_vel_expo = 0.60f;
static const float s_mode7_vel_accel_cmps2 = 250.0f;
static const float s_mode7_vel_jerk_cmps3 = 1800.0f;
static const float s_mode7_angle_limit_deg = 15.0f;
static float s_mode7_accel_x = 0.0f;
static float s_mode7_accel_y = 0.0f;

static void FC_Mode7_LimitVector(float *x, float *y, float limit)
{
    float mag = sqrtf((*x) * (*x) + (*y) * (*y));
    if (mag > limit)
    {
        float scale = limit / mag;
        *x *= scale;
        *y *= scale;
    }
}

static float FC_Mode7_StickToSpeed(float v)
{
    float dz = s_mode7_vel_deadzone_cmps / s_mode7_vel_limit_cmps;
    float a = (v >= 0.0f) ? v : -v;
    if (a <= dz)
    {
        return 0.0f;
    }
    a = (a - dz) / (1.0f - dz);
    a = ((1.0f - s_mode7_vel_expo) * a + s_mode7_vel_expo * a * a * a) * s_mode7_vel_limit_cmps;
    return (v >= 0.0f) ? a : -a;
}

void FC_Mode7_Init(void)
{
    PID_Init(&g_mode7_velx_pid,
             g_fc_params.mode7_vel_x_kp, g_fc_params.mode7_vel_x_ki, g_fc_params.mode7_vel_x_kd,
             0.0f, g_fc_params.vel_xy_dt,
             g_fc_params.mode7_vel_x_i_limit, g_fc_params.mode7_vel_x_d_lpf);
    PID_Init(&g_mode7_vely_pid,
             g_fc_params.mode7_vel_y_kp, g_fc_params.mode7_vel_y_ki, g_fc_params.mode7_vel_y_kd,
             0.0f, g_fc_params.vel_xy_dt,
             g_fc_params.mode7_vel_y_i_limit, g_fc_params.mode7_vel_y_d_lpf);
    g_mode7_velx_pid.aw_enable = 1U;
    g_mode7_velx_pid.aw_gain = 0.15f;
    g_mode7_vely_pid.aw_enable = 1U;
    g_mode7_vely_pid.aw_gain = 0.15f;
    FC_Mode7_Reset();
}

void FC_Mode7_Reset(void)
{
    PID_Reset(&g_mode7_velx_pid);
    PID_Reset(&g_mode7_vely_pid);
    g_mode7_velx_target = 0.0f;
    g_mode7_vely_target = 0.0f;
    s_mode7_accel_x = 0.0f;
    s_mode7_accel_y = 0.0f;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

void FC_Mode7_100Hz(void)
{
}

void FC_Mode7_50Hz(float dt)
{
    float ch0;
    float ch1;
    float velx_sp;
    float vely_sp;
    float accx_sp;
    float accy_sp;
    float velx_ff;
    float vely_ff;
    float velx_out;
    float vely_out;
    float roll_trim;
    float pitch_trim;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        FC_Mode7_Reset();
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0] * 0.001f, -1.0f, 1.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1] * -0.001f, -1.0f, 1.0f);

    velx_sp = FC_Mode7_StickToSpeed(ch0);
    vely_sp = FC_Mode7_StickToSpeed(ch1);
    FC_Mode7_LimitVector(&velx_sp, &vely_sp, s_mode7_vel_limit_cmps);

    accx_sp = (velx_sp - g_mode7_velx_target) / dt;
    accy_sp = (vely_sp - g_mode7_vely_target) / dt;
    FC_Mode7_LimitVector(&accx_sp, &accy_sp, s_mode7_vel_accel_cmps2);
    accx_sp -= s_mode7_accel_x;
    accy_sp -= s_mode7_accel_y;
    FC_Mode7_LimitVector(&accx_sp, &accy_sp, s_mode7_vel_jerk_cmps3 * dt);
    s_mode7_accel_x += accx_sp;
    s_mode7_accel_y += accy_sp;
    g_mode7_velx_target += s_mode7_accel_x * dt;
    g_mode7_vely_target += s_mode7_accel_y * dt;

    if (((velx_sp - (g_mode7_velx_target - s_mode7_accel_x * dt)) * (velx_sp - g_mode7_velx_target) +
         (vely_sp - (g_mode7_vely_target - s_mode7_accel_y * dt)) * (vely_sp - g_mode7_vely_target)) <= 0.0f)
    {
        g_mode7_velx_target = velx_sp;
        g_mode7_vely_target = vely_sp;
        s_mode7_accel_x = 0.0f;
        s_mode7_accel_y = 0.0f;
    }

    roll_trim = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_trim = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    velx_ff = FC_Mode_Clamp(g_fc_params.mode7_vel_x_kff * s_mode7_accel_x,
                            -s_mode7_angle_limit_deg, s_mode7_angle_limit_deg);
    vely_ff = FC_Mode_Clamp(g_fc_params.mode7_vel_y_kff * s_mode7_accel_y,
                            -s_mode7_angle_limit_deg, s_mode7_angle_limit_deg);

    g_mode7_velx_pid.output_min = -s_mode7_angle_limit_deg - roll_trim - velx_ff;
    g_mode7_velx_pid.output_max = s_mode7_angle_limit_deg - roll_trim - velx_ff;
    g_mode7_vely_pid.output_min = -s_mode7_angle_limit_deg - pitch_trim - vely_ff;
    g_mode7_vely_pid.output_max = s_mode7_angle_limit_deg - pitch_trim - vely_ff;

    velx_out = PID_Update(&g_mode7_velx_pid, g_mode7_velx_target, -Pos_Est_vel_x, dt) + velx_ff;
    vely_out = PID_Update(&g_mode7_vely_pid, g_mode7_vely_target, -Pos_Est_vel_y, dt) + vely_ff;
    g_mode7_velx_pid.ff_term = velx_ff;
    g_mode7_vely_pid.ff_term = vely_ff;

    roll_angle_target = FC_Mode_Clamp(velx_out + roll_trim, -s_mode7_angle_limit_deg, s_mode7_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(vely_out + pitch_trim, -s_mode7_angle_limit_deg, s_mode7_angle_limit_deg);
}
