#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../Protocols/wifi/wifi_justfloat/wifi_justfloat.h"
#include <math.h>

extern volatile uint32 tick_1000us_cnt;
extern volatile float g_car_velocity_strafe_mps;
extern volatile float g_car_velocity_forward_mps;
extern volatile float g_car_image_target_x;
extern volatile float g_car_image_target_y;
extern volatile float g_car_image_target_valid;

/* 模式2 X 轴速度环 PID */
pid_t g_mode2_velx_pid;
float g_mode2_velx_target = 0.0f;
/* 模式2 Y 轴速度环 PID */
pid_t g_mode2_vely_pid;
float g_mode2_vely_target = 0.0f;

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
} mode2_adrc_axis_t;

static mode2_adrc_axis_t s_mode2_adrc_x;
static mode2_adrc_axis_t s_mode2_adrc_y;

/* 遥控到速度映射比例：1000 单位 → 200 cm/s */
static const float s_mode2_rc_to_speed_scale = 0.2f;
/* 速度目标上限，单位 cm/s */
static const float s_mode2_vel_limit_cmps = 200.0f;
/* 速度目标死区，单位 cm/s */
static const float s_mode2_vel_deadzone_cmps = 6.0f;
/* 姿态角输出限幅，单位度 */
static const float s_mode2_angle_limit_deg = 15.0f;

static float FC_Mode2_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float FC_Mode2_Sign(float value)
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

static float FC_Mode2_SafeDt(float dt)
{
    return (dt > 0.0001f) ? dt : g_fc_params.vel_xy_dt;
}

static float FC_Mode2_SafePositive(float value, float fallback)
{
    return (value > 0.0001f) ? value : fallback;
}

static float FC_Mode2_Fal(float e, float alpha, float delta)
{
    float abs_e;
    float safe_delta;
    float safe_alpha;

    safe_delta = FC_Mode2_SafePositive(delta, 0.0001f);
    safe_alpha = FC_Mode_Clamp(alpha, 0.01f, 0.99f);
    abs_e = FC_Mode2_Abs(e);

    if (abs_e <= safe_delta)
    {
        return e / powf(safe_delta, 1.0f - safe_alpha);
    }

    return FC_Mode2_Sign(e) * powf(abs_e, safe_alpha);
}

static uint8 FC_Mode2_ADRC_NeedBrakeTd(float target, float td_v, float zero_quiet)
{
    float target_abs;
    float td_abs;

    target_abs = FC_Mode2_Abs(target);
    td_abs = FC_Mode2_Abs(td_v);

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

static void FC_Mode2_ADRC_ResetAxis(mode2_adrc_axis_t *axis)
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

static void FC_Mode2_ADRC_Reset(void)
{
    FC_Mode2_ADRC_ResetAxis(&s_mode2_adrc_x);
    FC_Mode2_ADRC_ResetAxis(&s_mode2_adrc_y);
}

static float FC_Mode2_ADRC_UpdateAxis(mode2_adrc_axis_t *axis,
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

    safe_dt = FC_Mode2_SafeDt(dt);
    safe_b0 = FC_Mode2_SafePositive(b0, 17.0f);
    acc_limit = FC_Mode2_SafePositive(g_fc_params.mode2_adrc_td_acc_limit_cmss, 100.0f);
    jerk_limit = FC_Mode2_SafePositive(g_fc_params.mode2_adrc_td_jerk_limit_cmsss, 450.0f);
    angle_limit = FC_Mode2_SafePositive(g_fc_params.mode2_adrc_angle_limit_deg, 10.0f);
    rate_limit = FC_Mode2_SafePositive(g_fc_params.mode2_adrc_output_rate_limit_degps, 35.0f);
    zero_quiet = FC_Mode2_SafePositive(g_fc_params.mode2_adrc_zero_quiet_cmps, 8.0f);

    if (axis->inited == 0U)
    {
        axis->td_v = measurement;
        axis->td_a = 0.0f;
        axis->z1 = measurement;
        axis->z2 = 0.0f;
        axis->u_last = 0.0f;
        axis->inited = 1U;
    }

    if (FC_Mode2_ADRC_NeedBrakeTd(target, axis->td_v, zero_quiet) != 0U)
    {
        acc_limit = FC_Mode2_SafePositive(g_fc_params.mode2_adrc_td_brake_acc_limit_cmss, 160.0f);
        jerk_limit = FC_Mode2_SafePositive(g_fc_params.mode2_adrc_td_brake_jerk_limit_cmsss, 900.0f);
    }

    desired_acc = (target - axis->td_v) / safe_dt;
    desired_acc = FC_Mode_Clamp(desired_acc, -acc_limit, acc_limit);
    acc_step_limit = jerk_limit * safe_dt;
    axis->td_a += FC_Mode_Clamp(desired_acc - axis->td_a, -acc_step_limit, acc_step_limit);
    axis->td_a = FC_Mode_Clamp(axis->td_a, -acc_limit, acc_limit);
    axis->td_v += axis->td_a * safe_dt;
    if ((FC_Mode2_Abs(target) < zero_quiet) &&
        (FC_Mode2_Abs(axis->td_v) < zero_quiet) &&
        (FC_Mode2_Abs(measurement) < zero_quiet))
    {
        axis->td_v = 0.0f;
        axis->td_a = 0.0f;
    }

    axis->obs_error = axis->z1 - measurement;
    z1_dot = axis->z2 -
             g_fc_params.mode2_adrc_eso_beta1 *
                 FC_Mode2_Fal(axis->obs_error,
                              g_fc_params.mode2_adrc_eso_alpha1,
                              g_fc_params.mode2_adrc_eso_delta_cmps) +
             safe_b0 * axis->u_last;
    z2_dot = -g_fc_params.mode2_adrc_eso_beta2 *
             FC_Mode2_Fal(axis->obs_error,
                          g_fc_params.mode2_adrc_eso_alpha2,
                          g_fc_params.mode2_adrc_eso_delta_cmps);
    axis->z1 += z1_dot * safe_dt;
    axis->z2 += z2_dot * safe_dt;

    axis->ctrl_error = axis->td_v - axis->z1;
    axis->fb_acc = g_fc_params.mode2_adrc_nl_kp *
                   FC_Mode2_Fal(axis->ctrl_error,
                                g_fc_params.mode2_adrc_nl_alpha,
                                g_fc_params.mode2_adrc_nl_delta_cmps);
    axis->comp_acc = FC_Mode_Clamp(axis->z2,
                                   -g_fc_params.mode2_adrc_comp_limit_cmss,
                                   g_fc_params.mode2_adrc_comp_limit_cmss);
    axis->u_acc = axis->td_a + axis->fb_acc - axis->comp_acc;
    axis->u_deg = axis->u_acc / safe_b0;
    u_deg_limited = FC_Mode_Clamp(axis->u_deg, -angle_limit, angle_limit);
    output_step_limit = rate_limit * safe_dt;
    axis->u_deg_sat = axis->u_last +
                      FC_Mode_Clamp(u_deg_limited - axis->u_last,
                                    -output_step_limit,
                                    output_step_limit);
    axis->u_deg_sat = FC_Mode_Clamp(axis->u_deg_sat, -angle_limit, angle_limit);
    axis->sat_flag = (FC_Mode2_Abs(axis->u_deg - axis->u_deg_sat) > 0.001f) ? 1.0f : 0.0f;
    axis->u_last = axis->u_deg_sat;

    return axis->u_deg_sat;
}

static void FC_Mode2_ADRC_FillPidDebug(pid_t *pid, const mode2_adrc_axis_t *axis, float b0)
{
    float safe_b0;

    if ((pid == NULL) || (axis == NULL))
    {
        return;
    }

    safe_b0 = FC_Mode2_SafePositive(b0, 17.0f);
    pid->error = axis->ctrl_error;
    pid->p_term = axis->fb_acc / safe_b0;
    pid->i_term = -axis->comp_acc / safe_b0;
    pid->d_term = axis->td_a / safe_b0;
    pid->ff_term = 0.0f;
    pid->output = axis->u_deg_sat;
}

static void FC_Mode2_ADRC_SendLog(float dt, float ch0, float ch1,
                                  float target_x_raw, float target_y_raw,
                                  float meas_x, float meas_y)
{
    float data[40];

    if (g_fc_params.mode2_adrc_log_enable < 0.5f)
    {
        return;
    }

    data[0] = 2002.0f;
    data[1] = (float)tick_1000us_cnt;
    data[2] = dt;
    data[3] = ch0;
    data[4] = ch1;
    data[5] = target_x_raw;
    data[6] = target_y_raw;
    data[7] = s_mode2_adrc_x.td_v;
    data[8] = s_mode2_adrc_y.td_v;
    data[9] = meas_x;
    data[10] = meas_y;

    data[11] = s_mode2_adrc_x.td_v;
    data[12] = s_mode2_adrc_x.td_a;
    data[13] = s_mode2_adrc_x.z1;
    data[14] = s_mode2_adrc_x.z2;
    data[15] = s_mode2_adrc_x.obs_error;
    data[16] = s_mode2_adrc_x.ctrl_error;
    data[17] = s_mode2_adrc_x.fb_acc;
    data[18] = s_mode2_adrc_x.comp_acc;
    data[19] = s_mode2_adrc_x.u_acc;
    data[20] = s_mode2_adrc_x.u_deg;
    data[21] = s_mode2_adrc_x.u_deg_sat;

    data[22] = s_mode2_adrc_y.td_v;
    data[23] = s_mode2_adrc_y.td_a;
    data[24] = s_mode2_adrc_y.z1;
    data[25] = s_mode2_adrc_y.z2;
    data[26] = s_mode2_adrc_y.obs_error;
    data[27] = s_mode2_adrc_y.ctrl_error;
    data[28] = s_mode2_adrc_y.fb_acc;
    data[29] = s_mode2_adrc_y.comp_acc;
    data[30] = s_mode2_adrc_y.u_acc;
    data[31] = s_mode2_adrc_y.u_deg;
    data[32] = s_mode2_adrc_y.u_deg_sat;

    data[33] = roll_angle_target;
    data[34] = pitch_angle_target;
    data[35] = g_euler.roll;
    data[36] = g_euler.pitch;
    data[37] = g_fc_params.mode2_adrc_enable;
    data[38] = s_mode2_adrc_x.sat_flag;
    data[39] = s_mode2_adrc_y.sat_flag;

    (void)wifi_justfloat_Array(data, 40U);
}

/*
 * 函数名: FC_Mode2_ApplyDeadzone
 * 功能: 对速度目标施加对称死区，死区内归零，死区外缩减死区量
 * 输入参数:
 *   v  - 输入速度目标，单位 cm/s
 *   dz - 死区宽度，单位 cm/s
 * 返回值:
 *   经过死区处理后的速度目标
 */
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

/*
 * 函数名: FC_Mode2_Init
 * 功能: 初始化模式2速度环 PID
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_Init(void)
{
    PID_Init(&g_mode2_velx_pid,
             g_fc_params.vel_x_kp, g_fc_params.vel_x_ki, g_fc_params.vel_x_kd,
             g_fc_params.vel_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_x_i_limit, g_fc_params.vel_x_d_lpf);
    PID_Init(&g_mode2_vely_pid,
             g_fc_params.vel_y_kp, g_fc_params.vel_y_ki, g_fc_params.vel_y_kd,
             g_fc_params.vel_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_y_i_limit, g_fc_params.vel_y_d_lpf);
    FC_Mode2_Reset();
}

/*
 * 函数名: FC_Mode2_Reset
 * 功能: 复位模式2速度环状态和姿态目标
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_Reset(void)
{
    PID_Reset(&g_mode2_velx_pid);
    PID_Reset(&g_mode2_vely_pid);
    FC_Mode2_ADRC_Reset();
    g_mode2_velx_target = 0.0f;
    g_mode2_vely_target = 0.0f;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

/*
 * 函数名: FC_Mode2_100Hz
 * 功能: 模式2 100Hz 占位
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_100Hz(void)
{
}

/*
 * 函数名: FC_Mode2_50Hz
 * 功能: 模式2 50Hz 纯速度 PI 控制
 *   遥控 → 速度目标（±100 cm/s，±6 cm/s 死区）→ PI → 目标角度（含机械中值）
 *   X轴：vel_x_kf 左正右负，取反后右正，PI 输出正 → roll 右倾 → 向右飞
 *   Y轴：vel_y_kf 前正后负，取反后后正，前推 ch1>0 → vely_target<0 → output<0 → pitch<0 → 前倾 → 向前飞
 * 输入参数:
 *   dt - 本次调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode2_50Hz(float dt)
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
        FC_Mode2_Reset();
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0], -1000.0f, 1000.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1], -1000.0f, 1000.0f);
    velx_target_raw = FC_Mode_Clamp(ch0 * s_mode2_rc_to_speed_scale,
                                    -s_mode2_vel_limit_cmps, s_mode2_vel_limit_cmps);
    vely_target_raw = FC_Mode_Clamp(-ch1 * s_mode2_rc_to_speed_scale,
                                    -s_mode2_vel_limit_cmps, s_mode2_vel_limit_cmps);

    g_mode2_velx_target = FC_Mode2_ApplyDeadzone(velx_target_raw, s_mode2_vel_deadzone_cmps);
    g_mode2_vely_target = FC_Mode2_ApplyDeadzone(vely_target_raw, s_mode2_vel_deadzone_cmps);
    meas_x = -Pos_Est_vel_x;
    meas_y = -Pos_Est_vel_y;

    if (g_fc_params.mode2_adrc_enable >= 0.5f)
    {
        velx_out = FC_Mode2_ADRC_UpdateAxis(&s_mode2_adrc_x,
                                            g_mode2_velx_target,
                                            meas_x,
                                            g_fc_params.mode2_adrc_b0_x,
                                            dt);
        vely_out = FC_Mode2_ADRC_UpdateAxis(&s_mode2_adrc_y,
                                            g_mode2_vely_target,
                                            meas_y,
                                            g_fc_params.mode2_adrc_b0_y,
                                            dt);

        FC_Mode2_ADRC_FillPidDebug(&g_mode2_velx_pid, &s_mode2_adrc_x, g_fc_params.mode2_adrc_b0_x);
        FC_Mode2_ADRC_FillPidDebug(&g_mode2_vely_pid, &s_mode2_adrc_y, g_fc_params.mode2_adrc_b0_y);
    }
    else
    {
        FC_Mode2_ADRC_Reset();
        velx_out = PID_Update(&g_mode2_velx_pid, g_mode2_velx_target, meas_x, dt);
        vely_out = PID_Update(&g_mode2_vely_pid, g_mode2_vely_target, meas_y, dt);

        velx_out += g_fc_params.mode2_vel_x_ff_deg_per_cmps * g_mode2_velx_target;
        vely_out += g_fc_params.mode2_vel_y_ff_deg_per_cmps * g_mode2_vely_target;
    }

    // velx_out = PID_Update(&g_mode2_velx_pid, g_mode2_velx_target, -opflow_vel_x_lpf, dt);
    // vely_out = PID_Update(&g_mode2_vely_pid, g_mode2_vely_target, -opflow_vel_y_lpf, dt);    

    roll_angle_target = FC_Mode_Clamp(velx_out+ FC_Mode_Get_Roll_Mech_Trim_Deg(), -s_mode2_angle_limit_deg, s_mode2_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(vely_out+ FC_Mode_Get_Pitch_Mech_Trim_Deg(), -s_mode2_angle_limit_deg, s_mode2_angle_limit_deg);

    FC_Mode2_ADRC_SendLog(dt, ch0, ch1, velx_target_raw, vely_target_raw, meas_x, meas_y);

    // wifi_justfloat(tick_1000us_cnt,
    //                target_height_m * 1000.0f,
    //                g_tof_fused_height_mm,
    //                g_mode2_velx_target,
    //                -Pos_Est_vel_x,
    //                g_mode2_vely_target,
    //                  -Pos_Est_vel_y,
    //                  g_car_velocity_strafe_mps,
    //                     g_car_velocity_forward_mps,
    //                g_car_image_target_x,
    //                     g_car_image_target_y,
    //                g_car_image_target_valid
    //                );
}
