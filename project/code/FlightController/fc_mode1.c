#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"

/* 模式1 状态枚举 */
typedef enum
{
    FC_MODE1_STATE_TRACK = 0U,     /* 正常跟杆速度模式 */
    FC_MODE1_STATE_BRAKE = 1U,     /* 摇杆回中后的强刹车模式 */
    FC_MODE1_STATE_ZERO_DAMP = 2U  /* 刹停后的零速阻尼模式 */
} fc_mode1_state_e;

/* 模式1 X 轴速度环 PID */
static pid_t s_mode1_velx_pid;
/* 模式1 Y 轴速度环 PID */
static pid_t s_mode1_vely_pid;
/* 模式1 当前状态 */
static fc_mode1_state_e s_mode1_state = FC_MODE1_STATE_ZERO_DAMP;
/* 模式1 摇杆是否处于活跃状态 */
static uint8_t s_mode1_stick_active = 0U;
/* 模式1 摇杆回中持续时间，单位 s */
static float s_mode1_neutral_timer_s = 0.0f;
/* 模式1 刹车阶段低速稳定计时，单位 s */
static float s_mode1_brake_settle_timer_s = 0.0f;

/* 模式1 遥控到速度映射比例，单位 cm/s 每遥控归一化单位 */
static const float s_mode1_rc_to_speed_scale = 0.1f;
/* 模式1 最大水平目标速度，单位 cm/s */
static const float s_mode1_vel_limit_cmps = 100.0f;
/* 模式1 普通跟杆姿态角限幅，单位 deg */
static const float s_mode1_track_angle_limit_deg = 20.0f;
/* 模式1 摇杆回中死区阈值 */
static const float s_mode1_stick_center_db = 60.0f;
/* 模式1 摇杆重新激活阈值 */
static const float s_mode1_stick_reactivate_db = 80.0f;
/* 模式1 进入强刹车的回中保持时间，单位 s */
static const float s_mode1_brake_entry_delay_s = 0.06f;
/* 模式1 退出强刹车所需的低速保持时间，单位 s */
static const float s_mode1_brake_exit_hold_s = 0.20f;
/* 模式1 强刹车 anti-windup 回算增益 */
static const float s_mode1_brake_aw_gain = 0.25f;

/*
 * 函数名: FC_Mode1_Abs
 * 功能: 计算浮点数绝对值
 * 输入参数:
 *   value - 输入浮点数
 * 返回值:
 *   输入值的绝对值
 */
static float FC_Mode1_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/*
 * 函数名: FC_Mode1_MapStickToVel
 * 功能: 将单轴遥控量映射为速度目标，并在中位死区内直接归零
 * 输入参数:
 *   ch_value - 单轴遥控输入，范围约 -1000~1000
 * 返回值:
 *   单轴速度目标，单位 cm/s
 */
static float FC_Mode1_MapStickToVel(float ch_value)
{
    if (FC_Mode1_Abs(ch_value) <= s_mode1_stick_center_db)
    {
        return 0.0f;
    }

    return FC_Mode_Clamp(ch_value * s_mode1_rc_to_speed_scale,
                         -s_mode1_vel_limit_cmps, s_mode1_vel_limit_cmps);
}

/*
 * 函数名: FC_Mode1_ConfigPid
 * 功能: 按当前模式写入速度环 PID 的工作参数
 * 输入参数:
 *   pid          - 需要配置的 PID 实例
 *   kp           - 比例增益
 *   ki           - 积分增益
 *   kd           - 微分增益
 *   kff          - 前馈增益
 *   i_limit      - 积分限幅
 *   output_limit - 输出限幅绝对值
 *   aw_enable    - 是否启用 anti-windup
 *   aw_gain      - anti-windup 回算增益
 * 返回值:
 *   无
 */
static void FC_Mode1_ConfigPid(pid_t *pid,
                               float kp,
                               float ki,
                               float kd,
                               float kff,
                               float i_limit,
                               float output_limit,
                               uint8_t aw_enable,
                               float aw_gain)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = kff;
    pid->i_limit = FC_Mode1_Abs(i_limit);
    pid->output_min = -FC_Mode1_Abs(output_limit);
    pid->output_max = FC_Mode1_Abs(output_limit);
    pid->aw_enable = aw_enable;
    pid->aw_gain = (aw_enable != 0U) ? aw_gain : 0.0f;
}

/*
 * 函数名: FC_Mode1_ResetPidPair
 * 功能: 清空模式1 两个速度环 PID 的内部状态
 * 输入参数: 无
 * 返回值: 无
 */
static void FC_Mode1_ResetPidPair(void)
{
    PID_Reset(&s_mode1_velx_pid);
    PID_Reset(&s_mode1_vely_pid);
}

/*
 * 函数名: FC_Mode1_EnterTrack
 * 功能: 切换到正常跟杆速度模式
 * 输入参数: 无
 * 返回值: 无
 */
static void FC_Mode1_EnterTrack(void)
{
    s_mode1_state = FC_MODE1_STATE_TRACK;
    s_mode1_neutral_timer_s = 0.0f;
    s_mode1_brake_settle_timer_s = 0.0f;
    FC_Mode1_ResetPidPair();
}

/*
 * 函数名: FC_Mode1_EnterBrake
 * 功能: 切换到强刹车模式
 * 输入参数: 无
 * 返回值: 无
 */
static void FC_Mode1_EnterBrake(void)
{
    s_mode1_state = FC_MODE1_STATE_BRAKE;
    s_mode1_neutral_timer_s = 0.0f;
    s_mode1_brake_settle_timer_s = 0.0f;
    FC_Mode1_ResetPidPair();
}

/*
 * 函数名: FC_Mode1_EnterZeroDamp
 * 功能: 切换到零速阻尼模式
 * 输入参数: 无
 * 返回值: 无
 */
static void FC_Mode1_EnterZeroDamp(void)
{
    s_mode1_state = FC_MODE1_STATE_ZERO_DAMP;
    s_mode1_neutral_timer_s = 0.0f;
    s_mode1_brake_settle_timer_s = 0.0f;
    FC_Mode1_ResetPidPair();
}

/*
 * 函数名: FC_Mode1_UpdateStickActive
 * 功能: 按滞回阈值更新摇杆活跃状态
 * 输入参数:
 *   ch0 - 横滚摇杆输入
 *   ch1 - 俯仰摇杆输入
 * 返回值:
 *   1 表示摇杆活跃，0 表示摇杆回中
 */
static uint8_t FC_Mode1_UpdateStickActive(float ch0, float ch1)
{
    float abs_ch0 = FC_Mode1_Abs(ch0);
    float abs_ch1 = FC_Mode1_Abs(ch1);

    if (s_mode1_stick_active != 0U)
    {
        if ((abs_ch0 <= s_mode1_stick_center_db) &&
            (abs_ch1 <= s_mode1_stick_center_db))
        {
            s_mode1_stick_active = 0U;
        }
    }
    else
    {
        if ((abs_ch0 >= s_mode1_stick_reactivate_db) ||
            (abs_ch1 >= s_mode1_stick_reactivate_db))
        {
            s_mode1_stick_active = 1U;
        }
    }

    return s_mode1_stick_active;
}

/*
 * 函数名: FC_Mode1_Init
 * 功能: 初始化模式1 水平速度环控制资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_Init(void)
{
    PID_Init(&s_mode1_velx_pid,
             g_fc_params.vel_x_kp, g_fc_params.vel_x_ki, g_fc_params.vel_x_kd,
             g_fc_params.vel_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_x_i_limit, g_fc_params.vel_x_d_lpf);
    PID_Init(&s_mode1_vely_pid,
             g_fc_params.vel_y_kp, g_fc_params.vel_y_ki, g_fc_params.vel_y_kd,
             g_fc_params.vel_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_y_i_limit, g_fc_params.vel_y_d_lpf);
    FC_Mode1_Reset();
}

/*
 * 函数名: FC_Mode1_Reset
 * 功能: 复位模式1 状态机、速度环和姿态目标
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_Reset(void)
{
    s_mode1_state = FC_MODE1_STATE_ZERO_DAMP;
    s_mode1_stick_active = 1U;
    s_mode1_neutral_timer_s = 0.0f;
    s_mode1_brake_settle_timer_s = 0.0f;
    FC_Mode1_ResetPidPair();
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

/*
 * 函数名: FC_Mode1_100Hz
 * 功能: 模式1 当前不需要 100Hz 专用控制，保留空实现
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_100Hz(void)
{
}

/*
 * 函数名: FC_Mode1_50Hz
 * 功能: 执行模式1 的 50Hz 跟杆、强刹车与零速阻尼控制
 * 输入参数:
 *   dt - 本次 50Hz 调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode1_50Hz(float dt)
{
    float ch0;
    float ch1;
    float velx_target = 0.0f;
    float vely_target = 0.0f;
    float velx_feedback;
    float vely_feedback;
    float velx_out;
    float vely_out;
    float track_ff_x = 0.0f;
    float track_ff_y = 0.0f;
    float angle_limit_deg;
    float feedback_is_kf = 1.0f;
    uint8_t stick_active_now;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0], -1000.0f, 1000.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1], -1000.0f, 1000.0f);
    stick_active_now = FC_Mode1_UpdateStickActive(ch0, ch1);

    if (stick_active_now != 0U)
    {
        if (s_mode1_state != FC_MODE1_STATE_TRACK)
        {
            FC_Mode1_EnterTrack();
        }
        s_mode1_neutral_timer_s = 0.0f;
        s_mode1_brake_settle_timer_s = 0.0f;
    }
    else
    {
        if (s_mode1_state == FC_MODE1_STATE_TRACK)
        {
            s_mode1_neutral_timer_s += dt;
            if (s_mode1_neutral_timer_s >= s_mode1_brake_entry_delay_s)
            {
                FC_Mode1_EnterBrake();
            }
        }
        else if (s_mode1_state == FC_MODE1_STATE_BRAKE)
        {
            if ((FC_Mode1_Abs(-Pos_Est_vel_x) < g_fc_params.mode1_brake_exit_vel_cmps) &&
                (FC_Mode1_Abs(-Pos_Est_vel_y) < g_fc_params.mode1_brake_exit_vel_cmps))
            {
                s_mode1_brake_settle_timer_s += dt;
            }
            else
            {
                s_mode1_brake_settle_timer_s = 0.0f;
            }

            if (s_mode1_brake_settle_timer_s >= s_mode1_brake_exit_hold_s)
            {
                FC_Mode1_EnterZeroDamp();
            }
        }
    }

    if (s_mode1_state == FC_MODE1_STATE_TRACK)
    {
        velx_target = FC_Mode1_MapStickToVel(ch0);
        vely_target = FC_Mode1_MapStickToVel(-ch1);
        track_ff_x = FC_Mode_Clamp(g_fc_params.mode1_track_ff_deg_per_cmps * velx_target,
                                   -g_fc_params.mode1_track_ff_limit_deg,
                                   g_fc_params.mode1_track_ff_limit_deg);
        track_ff_y = FC_Mode_Clamp(g_fc_params.mode1_track_ff_deg_per_cmps * vely_target,
                                   -g_fc_params.mode1_track_ff_limit_deg,
                                   g_fc_params.mode1_track_ff_limit_deg);
        velx_feedback = -Pos_Est_vel_x_kf;
        vely_feedback = -Pos_Est_vel_y_kf;
        angle_limit_deg = s_mode1_track_angle_limit_deg;
        feedback_is_kf = 1.0f;

        FC_Mode1_ConfigPid(&s_mode1_velx_pid,
                           g_fc_params.vel_x_kp, g_fc_params.vel_x_ki, g_fc_params.vel_x_kd,
                           g_fc_params.vel_x_kff, g_fc_params.vel_x_i_limit,
                           angle_limit_deg, 0U, 0.0f);
        FC_Mode1_ConfigPid(&s_mode1_vely_pid,
                           g_fc_params.vel_y_kp, g_fc_params.vel_y_ki, g_fc_params.vel_y_kd,
                           g_fc_params.vel_y_kff, g_fc_params.vel_y_i_limit,
                           angle_limit_deg, 0U, 0.0f);
    }
    else if (s_mode1_state == FC_MODE1_STATE_BRAKE)
    {
        velx_feedback = -Pos_Est_vel_x;
        vely_feedback = -Pos_Est_vel_y;
        angle_limit_deg = g_fc_params.mode1_brake_angle_limit_deg;
        feedback_is_kf = 0.0f;

        FC_Mode1_ConfigPid(&s_mode1_velx_pid,
                           g_fc_params.mode1_brake_kp, 0.0f, 0.0f,
                           0.0f, 0.0f, angle_limit_deg, 1U, s_mode1_brake_aw_gain);
        FC_Mode1_ConfigPid(&s_mode1_vely_pid,
                           g_fc_params.mode1_brake_kp, 0.0f, 0.0f,
                           0.0f, 0.0f, angle_limit_deg, 1U, s_mode1_brake_aw_gain);
    }
    else
    {
        velx_feedback = -Pos_Est_vel_x_kf;
        vely_feedback = -Pos_Est_vel_y_kf;
        angle_limit_deg = s_mode1_track_angle_limit_deg;
        feedback_is_kf = 1.0f;

        FC_Mode1_ConfigPid(&s_mode1_velx_pid,
                           g_fc_params.vel_x_kp, g_fc_params.vel_x_ki, g_fc_params.vel_x_kd,
                           g_fc_params.vel_x_kff, g_fc_params.vel_x_i_limit,
                           angle_limit_deg, 0U, 0.0f);
        FC_Mode1_ConfigPid(&s_mode1_vely_pid,
                           g_fc_params.vel_y_kp, g_fc_params.vel_y_ki, g_fc_params.vel_y_kd,
                           g_fc_params.vel_y_kff, g_fc_params.vel_y_i_limit,
                           angle_limit_deg, 0U, 0.0f);
    }

    velx_out = PID_Update(&s_mode1_velx_pid, velx_target, velx_feedback, dt) + track_ff_x;
    vely_out = PID_Update(&s_mode1_vely_pid, vely_target, vely_feedback, dt) + track_ff_y;

    velx_out = FC_Mode_Clamp(velx_out, -angle_limit_deg, angle_limit_deg);
    vely_out = FC_Mode_Clamp(vely_out, -angle_limit_deg, angle_limit_deg);

    roll_angle_target = velx_out + FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = vely_out + FC_Mode_Get_Pitch_Mech_Trim_Deg();

    wifi_vofa_JustFloat(16u,
                        -opflow_vel_x,
                        -Pos_Est_vel_x,
                        -Pos_Est_vel_x_kf,
                        velx_target,
                        s_mode1_velx_pid.p_term,
                        s_mode1_velx_pid.i_term,
                        velx_out,
                        roll_angle_target,
                        g_euler.roll,
                        Pos_Est_pos_x,
                        roll_gyro_target,
                        g_imufilter_1000hz.gyrox,
                        (float)s_mode1_state,
                        s_mode1_neutral_timer_s * 1000.0f,
                        feedback_is_kf,
                        track_ff_x);
}
