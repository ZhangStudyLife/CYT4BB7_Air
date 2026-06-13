#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include <math.h>

pid_t g_mode4_velx_pid;
pid_t g_mode4_vely_pid;
float g_mode4_velx_target = 0.0f;
float g_mode4_vely_target = 0.0f;

typedef struct
{
    float td_v;
    float td_a;
    float z1;
    float z2;
    float u_last;
    float obs_error;
    float ctrl_error;
    float fb_acc;
    float comp_acc;
    float u_acc;
    float u_deg;
    float u_deg_sat;
    float sat_flag;
    uint8 inited;
} mode4_adrc_axis_t;

static mode4_adrc_axis_t s_mode4_adrc_x;
static mode4_adrc_axis_t s_mode4_adrc_y;

/* Mode4 uses ADRC velocity loop. */
static const float s_mode4_rc_to_speed_scale = 0.2f;
static const float s_mode4_vel_limit_cmps = 200.0f;
static const float s_mode4_vel_deadzone_cmps = 6.0f;
static const float s_mode4_angle_limit_deg = 15.0f;

static float FC_Mode4_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float FC_Mode4_Sign(float value)
{
    if (value > 0.0f)
    {
        return 1.0f;
    }
    if (value < 0.0f)
    {
        return -1.0f;
    }
    return 0.0f;
}

static float FC_Mode4_SafeDt(float dt)
{
    return (dt > 0.0001f) ? dt : g_fc_params.vel_xy_dt;
}

static float FC_Mode4_SafePositive(float value, float fallback)
{
    return (value > 0.0001f) ? value : fallback;
}

static float FC_Mode4_Fal(float e, float alpha, float delta)
{
    float abs_e;
    float safe_delta;
    float safe_alpha;

    safe_delta = FC_Mode4_SafePositive(delta, 0.0001f);
    safe_alpha = FC_Mode_Clamp(alpha, 0.01f, 0.99f);
    abs_e = FC_Mode4_Abs(e);

    if (abs_e <= safe_delta)
    {
        return e / powf(safe_delta, 1.0f - safe_alpha);
    }

    return FC_Mode4_Sign(e) * powf(abs_e, safe_alpha);
}

static uint8 FC_Mode4_ADRC_NeedBrakeTd(float target, float td_v, float zero_quiet)
{
    float target_abs;
    float td_abs;

    target_abs = FC_Mode4_Abs(target);
    td_abs = FC_Mode4_Abs(td_v);

    if ((target_abs < zero_quiet) && (td_abs > target_abs))
    {
        return 1U;
    }
    if ((target * td_v) < 0.0f)
    {
        return 1U;
    }
    if (target_abs < td_abs)
    {
        return 1U;
    }
    return 0U;
}

static void FC_Mode4_ADRC_ResetAxis(mode4_adrc_axis_t *axis)
{
    if (axis == NULL)
    {
        return;
    }

    axis->td_v = 0.0f;
    axis->td_a = 0.0f;
    axis->z1 = 0.0f;
    axis->z2 = 0.0f;
    axis->u_last = 0.0f;
    axis->obs_error = 0.0f;
    axis->ctrl_error = 0.0f;
    axis->fb_acc = 0.0f;
    axis->comp_acc = 0.0f;
    axis->u_acc = 0.0f;
    axis->u_deg = 0.0f;
    axis->u_deg_sat = 0.0f;
    axis->sat_flag = 0.0f;
    axis->inited = 0U;
}

static void FC_Mode4_ADRC_Reset(void)
{
    FC_Mode4_ADRC_ResetAxis(&s_mode4_adrc_x);
    FC_Mode4_ADRC_ResetAxis(&s_mode4_adrc_y);
}

static float FC_Mode4_ADRC_UpdateAxis(mode4_adrc_axis_t *axis,
                                      float target,
                                      float measurement,
                                      float b0,
                                      float dt)
{
    float safe_dt;
    float safe_b0;
    float acc_limit;
    float jerk_limit;
    float angle_limit;
    float rate_limit;
    float zero_quiet;
    float output_step_limit;
    float desired_acc;
    float acc_step_limit;
    float z1_dot;
    float z2_dot;
    float u_deg_limited;

    if (axis == NULL)
    {
        return 0.0f;
    }

    safe_dt = FC_Mode4_SafeDt(dt);
    safe_b0 = FC_Mode4_SafePositive(b0, 17.0f);
    acc_limit = FC_Mode4_SafePositive(g_fc_params.mode4_adrc_td_acc_limit_cmss, 100.0f);
    jerk_limit = FC_Mode4_SafePositive(g_fc_params.mode4_adrc_td_jerk_limit_cmsss, 450.0f);
    angle_limit = FC_Mode4_SafePositive(g_fc_params.mode4_adrc_angle_limit_deg, 10.0f);
    rate_limit = FC_Mode4_SafePositive(g_fc_params.mode4_adrc_output_rate_limit_degps, 35.0f);
    zero_quiet = FC_Mode4_SafePositive(g_fc_params.mode4_adrc_zero_quiet_cmps, 8.0f);

    if (axis->inited == 0U)
    {
        axis->td_v = measurement;
        axis->td_a = 0.0f;
        axis->z1 = measurement;
        axis->z2 = 0.0f;
        axis->u_last = 0.0f;
        axis->inited = 1U;
    }

    if (FC_Mode4_ADRC_NeedBrakeTd(target, axis->td_v, zero_quiet) != 0U)
    {
        acc_limit = FC_Mode4_SafePositive(g_fc_params.mode4_adrc_td_brake_acc_limit_cmss, 160.0f);
        jerk_limit = FC_Mode4_SafePositive(g_fc_params.mode4_adrc_td_brake_jerk_limit_cmsss, 900.0f);
    }

    desired_acc = (target - axis->td_v) / safe_dt;
    desired_acc = FC_Mode_Clamp(desired_acc, -acc_limit, acc_limit);
    acc_step_limit = jerk_limit * safe_dt;
    axis->td_a += FC_Mode_Clamp(desired_acc - axis->td_a, -acc_step_limit, acc_step_limit);
    axis->td_a = FC_Mode_Clamp(axis->td_a, -acc_limit, acc_limit);
    axis->td_v += axis->td_a * safe_dt;
    if ((FC_Mode4_Abs(target) < zero_quiet) &&
        (FC_Mode4_Abs(axis->td_v) < zero_quiet) &&
        (FC_Mode4_Abs(measurement) < zero_quiet))
    {
        axis->td_v = 0.0f;
        axis->td_a = 0.0f;
    }

    axis->obs_error = axis->z1 - measurement;
    z1_dot = axis->z2 -
             g_fc_params.mode4_adrc_eso_beta1 *
                 FC_Mode4_Fal(axis->obs_error,
                              g_fc_params.mode4_adrc_eso_alpha1,
                              g_fc_params.mode4_adrc_eso_delta_cmps) +
             safe_b0 * axis->u_last;
    z2_dot = -g_fc_params.mode4_adrc_eso_beta2 *
             FC_Mode4_Fal(axis->obs_error,
                          g_fc_params.mode4_adrc_eso_alpha2,
                          g_fc_params.mode4_adrc_eso_delta_cmps);
    axis->z1 += z1_dot * safe_dt;
    axis->z2 += z2_dot * safe_dt;

    axis->ctrl_error = axis->td_v - axis->z1;
    axis->fb_acc = g_fc_params.mode4_adrc_nl_kp *
                   FC_Mode4_Fal(axis->ctrl_error,
                                g_fc_params.mode4_adrc_nl_alpha,
                                g_fc_params.mode4_adrc_nl_delta_cmps);
    axis->comp_acc = FC_Mode_Clamp(axis->z2,
                                   -g_fc_params.mode4_adrc_comp_limit_cmss,
                                   g_fc_params.mode4_adrc_comp_limit_cmss);
    axis->u_acc = axis->td_a + axis->fb_acc - axis->comp_acc;
    axis->u_deg = axis->u_acc / safe_b0;
    u_deg_limited = FC_Mode_Clamp(axis->u_deg, -angle_limit, angle_limit);
    output_step_limit = rate_limit * safe_dt;
    axis->u_deg_sat = axis->u_last +
                      FC_Mode_Clamp(u_deg_limited - axis->u_last,
                                    -output_step_limit,
                                    output_step_limit);
    axis->u_deg_sat = FC_Mode_Clamp(axis->u_deg_sat, -angle_limit, angle_limit);
    axis->sat_flag = (FC_Mode4_Abs(axis->u_deg - axis->u_deg_sat) > 0.001f) ? 1.0f : 0.0f;
    axis->u_last = axis->u_deg_sat;

    return axis->u_deg_sat;
}

static void FC_Mode4_ADRC_FillPidDebug(pid_t *pid, const mode4_adrc_axis_t *axis, float b0)
{
    float safe_b0;

    if ((pid == NULL) || (axis == NULL))
    {
        return;
    }

    safe_b0 = FC_Mode4_SafePositive(b0, 17.0f);
    pid->error = axis->ctrl_error;
    pid->p_term = axis->fb_acc / safe_b0;
    pid->i_term = -axis->comp_acc / safe_b0;
    pid->d_term = axis->td_a / safe_b0;
    pid->ff_term = 0.0f;
    pid->output = axis->u_deg_sat;
}

static float FC_Mode4_ApplyDeadzone(float v, float dz)
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

void FC_Mode4_Init(void)
{
    PID_Init(&g_mode4_velx_pid,
             g_fc_params.vel_x_kp, g_fc_params.vel_x_ki, g_fc_params.vel_x_kd,
             g_fc_params.vel_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_x_i_limit, g_fc_params.vel_x_d_lpf);
    PID_Init(&g_mode4_vely_pid,
             g_fc_params.vel_y_kp, g_fc_params.vel_y_ki, g_fc_params.vel_y_kd,
             g_fc_params.vel_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_y_i_limit, g_fc_params.vel_y_d_lpf);
    FC_Mode4_Reset();
}

void FC_Mode4_Reset(void)
{
    PID_Reset(&g_mode4_velx_pid);
    PID_Reset(&g_mode4_vely_pid);
    FC_Mode4_ADRC_Reset();
    g_mode4_velx_target = 0.0f;
    g_mode4_vely_target = 0.0f;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

void FC_Mode4_100Hz(void)
{
}

void FC_Mode4_50Hz(float dt)
{
    float ch0;
    float ch1;
    float velx_target_raw;
    float vely_target_raw;
    float meas_x;
    float meas_y;
    float velx_out;
    float vely_out;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        FC_Mode4_Reset();
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0], -1000.0f, 1000.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1], -1000.0f, 1000.0f);
    velx_target_raw = FC_Mode_Clamp(ch0 * s_mode4_rc_to_speed_scale,
                                    -s_mode4_vel_limit_cmps, s_mode4_vel_limit_cmps);
    vely_target_raw = FC_Mode_Clamp(-ch1 * s_mode4_rc_to_speed_scale,
                                    -s_mode4_vel_limit_cmps, s_mode4_vel_limit_cmps);

    g_mode4_velx_target = FC_Mode4_ApplyDeadzone(velx_target_raw, s_mode4_vel_deadzone_cmps);
    g_mode4_vely_target = FC_Mode4_ApplyDeadzone(vely_target_raw, s_mode4_vel_deadzone_cmps);
    meas_x = -Pos_Est_vel_x;
    meas_y = -Pos_Est_vel_y;

    if (g_fc_params.mode4_adrc_enable >= 0.5f)
    {
        velx_out = FC_Mode4_ADRC_UpdateAxis(&s_mode4_adrc_x,
                                            g_mode4_velx_target,
                                            meas_x,
                                            g_fc_params.mode4_adrc_b0_x,
                                            dt);
        vely_out = FC_Mode4_ADRC_UpdateAxis(&s_mode4_adrc_y,
                                            g_mode4_vely_target,
                                            meas_y,
                                            g_fc_params.mode4_adrc_b0_y,
                                            dt);

        FC_Mode4_ADRC_FillPidDebug(&g_mode4_velx_pid, &s_mode4_adrc_x, g_fc_params.mode4_adrc_b0_x);
        FC_Mode4_ADRC_FillPidDebug(&g_mode4_vely_pid, &s_mode4_adrc_y, g_fc_params.mode4_adrc_b0_y);
    }
    else
    {
        FC_Mode4_ADRC_Reset();
        velx_out = PID_Update(&g_mode4_velx_pid, g_mode4_velx_target, meas_x, dt);
        vely_out = PID_Update(&g_mode4_vely_pid, g_mode4_vely_target, meas_y, dt);
    }

    roll_angle_target = FC_Mode_Clamp(velx_out + FC_Mode_Get_Roll_Mech_Trim_Deg(),
                                      -s_mode4_angle_limit_deg, s_mode4_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(vely_out + FC_Mode_Get_Pitch_Mech_Trim_Deg(),
                                       -s_mode4_angle_limit_deg, s_mode4_angle_limit_deg);

}
