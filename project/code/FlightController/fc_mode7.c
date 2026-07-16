#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"

pid_t g_mode7_velx_pid;
pid_t g_mode7_vely_pid;
float g_mode7_velx_target = 0.0f;
float g_mode7_vely_target = 0.0f;

static float s_mode7_prev_velx_target = 0.0f;
static float s_mode7_prev_vely_target = 0.0f;
static float s_mode7_velx_ff_lpf = 0.0f;
static float s_mode7_vely_ff_lpf = 0.0f;

static float FC_Mode7_StickToSpeed(float v)
{
    float a = (v >= 0.0f) ? v : -v;
    if (a <= FC_MODE7_STICK_DEADZONE)
    {
        return 0.0f;
    }
    a = (a - FC_MODE7_STICK_DEADZONE) / (1.0f - FC_MODE7_STICK_DEADZONE);
    a = ((1.0f - FC_MODE7_STICK_EXPO) * a + FC_MODE7_STICK_EXPO * a * a * a) * FC_MODE7_VEL_LIMIT_CMPS;
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
    s_mode7_prev_velx_target = 0.0f;
    s_mode7_prev_vely_target = 0.0f;
    s_mode7_velx_ff_lpf = 0.0f;
    s_mode7_vely_ff_lpf = 0.0f;
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
    float velx_ff;
    float vely_ff;
    float velx_target_rate;
    float vely_target_rate;
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
    velx_target_rate = (velx_sp - s_mode7_prev_velx_target) / dt;
    vely_target_rate = (vely_sp - s_mode7_prev_vely_target) / dt;
    g_mode7_velx_target = velx_sp;
    g_mode7_vely_target = vely_sp;
    s_mode7_prev_velx_target = g_mode7_velx_target;
    s_mode7_prev_vely_target = g_mode7_vely_target;

    roll_trim = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_trim = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    velx_ff = FC_Mode_Clamp(g_fc_params.mode7_vel_x_kff * velx_target_rate,
                            -angle_target_max, angle_target_max);
    vely_ff = FC_Mode_Clamp(g_fc_params.mode7_vel_y_kff * vely_target_rate,
                            -angle_target_max, angle_target_max);
    s_mode7_velx_ff_lpf += FC_MODE_VEL_KFF_LPF_ALPHA * (velx_ff - s_mode7_velx_ff_lpf);
    s_mode7_vely_ff_lpf += FC_MODE_VEL_KFF_LPF_ALPHA * (vely_ff - s_mode7_vely_ff_lpf);
    velx_ff = s_mode7_velx_ff_lpf;
    vely_ff = s_mode7_vely_ff_lpf;

    g_mode7_velx_pid.output_min = -angle_target_max - roll_trim - velx_ff;
    g_mode7_velx_pid.output_max = angle_target_max - roll_trim - velx_ff;
    g_mode7_vely_pid.output_min = -angle_target_max - pitch_trim - vely_ff;
    g_mode7_vely_pid.output_max = angle_target_max - pitch_trim - vely_ff;

    velx_out = PID_Update(&g_mode7_velx_pid, g_mode7_velx_target, -Pos_Est_vel_x, dt) + velx_ff;
    vely_out = PID_Update(&g_mode7_vely_pid, g_mode7_vely_target, -Pos_Est_vel_y, dt) + vely_ff;
    g_mode7_velx_pid.ff_term = velx_ff;
    g_mode7_vely_pid.ff_term = vely_ff;

    roll_angle_target = FC_Mode_Clamp(velx_out + roll_trim, -angle_target_max, angle_target_max);
    pitch_angle_target = FC_Mode_Clamp(vely_out + pitch_trim, -angle_target_max, angle_target_max);
}
