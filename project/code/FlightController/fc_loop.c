#include "fc_loop.h"

pid_t roll_gyro_pid;
pid_t pitch_gyro_pid;
pid_t yaw_gyro_pid;
pid_t roll_angle_pid;
pid_t pitch_angle_pid;
pid_t yaw_angle_pid;
pid_t height_pos_pid;
pid_t height_vel_pid;
float roll_gyro_target = 0.0f;
float pitch_gyro_target = 0.0f;
float yaw_gyro_target = 0.0f;
float roll_angle_target = 0.0f;
float pitch_angle_target = 0.0f;
float yaw_angle_target = 0.0f;
float height_vel_out = 0.0f;
float height_pos_out = 0.0f;
float target_height_m = 1.0f;
extern volatile uint32 tick_500us_cnt;


#define FC_ROLL_MECH_TRIM_DEG  (-2.0)
#define FC_PITCH_MECH_TRIM_DEG (4.0f)

static float fc_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

/*
 * 100Hz基础油门自适应更新：
 * 仅在飞行+高度可信+姿态平稳时，根据垂向速度误差慢速修正 g_fc_params.base_throttle
 */
void FC_ThrTrim_Update_100Hz(void)
{
    static uint8 s_inited = 0U;
    static uint8 s_fly_prev = 0U; 
    static int32_t s_base_init = 0;
    static float s_base_f = 0.0f;
    static float s_vz_lpf = 0.0f;
    static uint16 s_gate_ok_cnt = 0U;
    static uint16 s_gate_bad_cnt = 0U;

    const float tau_up_s = 8.0f;
    const float tau_down_s = 26.0f;
    const float dt_s = 0.01f;
    const float learn_clip = 160.0f;
    const float vz_gain = 1100.0f; /* m/s -> 油门修正量 */
    const float vz_lpf_alpha = 0.25f;
    const float level_max_deg = 6.0f;
    const float min_learn_h_m = 0.55f;
    const float vz_gate_mps = 0.15f;
    const uint16 gate_ok_min_samples = 25U; /* 0.25s */
    const int32_t base_span = 650;
    const int32_t hard_min = 2600;
    const int32_t hard_max = 5600;
    const int32_t step_max = 4; /* 每次100Hz最大变化 */
    FC_START_CRSF_state_e state;
    uint8 is_flying;
    uint8 gate_ok = 1U;
    float learn;
    float tau_s;
    float alpha;
    int32_t base_min;
    int32_t base_max;
    int32_t target_i32;
    int32_t curr_i32;

    state = FC_START_CRSF_Get_State();
    is_flying = (state == FC_START_CRSF_STATE_FLYING) ? 1U : 0U;

    if (0U == s_inited)
    {
        s_base_init = g_fc_params.base_throttle;
        s_base_f = (float)s_base_init;
        s_vz_lpf = g_height_vz_mps;
        s_inited = 1U;
    }

    /* 进入FLYING时重置学习基线，避免跨阶段污染 */
    if ((0U == s_fly_prev) && (0U != is_flying))
    {
        s_base_init = g_fc_params.base_throttle;
        s_base_f = (float)s_base_init;
        s_vz_lpf = g_height_vz_mps;
        s_gate_ok_cnt = 0U;
        s_gate_bad_cnt = 0U;
    }
    s_fly_prev = is_flying;

    if (0U == is_flying)
    {
        return;
    }

    s_vz_lpf += vz_lpf_alpha * (g_height_vz_mps - s_vz_lpf);

    if ((0U == g_height_est_valid) || (0U == g_tof_fused_valid))
    {
        gate_ok = 0U;
    }

    if (g_height_est_m < min_learn_h_m)
    {
        gate_ok = 0U;
    }

    if ((g_euler.roll > level_max_deg) || (g_euler.roll < -level_max_deg) ||
        (g_euler.pitch > level_max_deg) || (g_euler.pitch < -level_max_deg))
    {
        gate_ok = 0U;
    }

    if ((s_vz_lpf > vz_gate_mps) || (s_vz_lpf < -vz_gate_mps))
    {
        gate_ok = 0U;
    }

    if (0U != gate_ok)
    {
        if (s_gate_ok_cnt < 65535U)
        {
            s_gate_ok_cnt++;
        }
        s_gate_bad_cnt = 0U;
    }
    else
    {
        if (s_gate_bad_cnt < 65535U)
        {
            s_gate_bad_cnt++;
        }
        s_gate_ok_cnt = 0U;
        return;
    }

    if (s_gate_ok_cnt < gate_ok_min_samples)
    {
        return;
    }

    learn = -vz_gain * s_vz_lpf;
    learn = fc_clampf(learn, -learn_clip, learn_clip);

    tau_s = (learn >= 0.0f) ? tau_up_s : tau_down_s;
    alpha = dt_s / (tau_s + dt_s);
    s_base_f += alpha * learn;

    base_min = s_base_init - base_span;
    base_max = s_base_init + base_span;
    if (base_min < hard_min)
    {
        base_min = hard_min;
    }
    if (base_max > hard_max)
    {
        base_max = hard_max;
    }

    s_base_f = fc_clampf(s_base_f, (float)base_min, (float)base_max);

    curr_i32 = g_fc_params.base_throttle;
    target_i32 = (int32_t)(s_base_f + 0.5f);
    if (target_i32 > (curr_i32 + step_max))
    {
        target_i32 = curr_i32 + step_max;
    }
    else if (target_i32 < (curr_i32 - step_max))
    {
        target_i32 = curr_i32 - step_max;
    }

    g_fc_params.base_throttle = target_i32;
}

void FC_Loop_Init(void)
{
    PID_Init(&roll_angle_pid,
             g_fc_params.roll_angle_kp, g_fc_params.roll_angle_ki, g_fc_params.roll_angle_kd,
             g_fc_params.roll_angle_kff, g_fc_params.angle_dt,
             g_fc_params.roll_angle_i_limit, g_fc_params.roll_angle_d_lpf);
    PID_Init(&pitch_angle_pid,
             g_fc_params.pitch_angle_kp, g_fc_params.pitch_angle_ki, g_fc_params.pitch_angle_kd,
             g_fc_params.pitch_angle_kff, g_fc_params.angle_dt,
             g_fc_params.pitch_angle_i_limit, g_fc_params.pitch_angle_d_lpf);
    PID_Init(&yaw_angle_pid,
             g_fc_params.yaw_angle_kp, g_fc_params.yaw_angle_ki, g_fc_params.yaw_angle_kd,
             g_fc_params.yaw_angle_kff, g_fc_params.angle_dt,
             g_fc_params.yaw_angle_i_limit, g_fc_params.yaw_angle_d_lpf);

    PID_Init(&roll_gyro_pid,
             g_fc_params.roll_gyro_kp, g_fc_params.roll_gyro_ki, g_fc_params.roll_gyro_kd,
             g_fc_params.roll_gyro_kff, g_fc_params.gyro_dt,
             g_fc_params.roll_gyro_i_limit, g_fc_params.roll_gyro_d_lpf);
    PID_Init(&pitch_gyro_pid,
             g_fc_params.pitch_gyro_kp, g_fc_params.pitch_gyro_ki, g_fc_params.pitch_gyro_kd,
             g_fc_params.pitch_gyro_kff, g_fc_params.gyro_dt,
             g_fc_params.pitch_gyro_i_limit, g_fc_params.pitch_gyro_d_lpf);
    PID_Init(&yaw_gyro_pid,
             g_fc_params.yaw_gyro_kp, g_fc_params.yaw_gyro_ki, g_fc_params.yaw_gyro_kd,
             g_fc_params.yaw_gyro_kff, g_fc_params.gyro_dt,
             g_fc_params.yaw_gyro_i_limit, g_fc_params.yaw_gyro_d_lpf);
    PID_Init(&height_pos_pid,
             g_fc_params.pos_z_kp, g_fc_params.pos_z_ki, g_fc_params.pos_z_kd,
             g_fc_params.pos_z_kff, g_fc_params.pos_z_dt,
             g_fc_params.pos_z_i_limit, g_fc_params.pos_z_d_lpf);
    PID_Init(&height_vel_pid,
             g_fc_params.vel_z_kp, g_fc_params.vel_z_ki, g_fc_params.vel_z_kd,
             g_fc_params.vel_z_kff, g_fc_params.vel_z_dt,
             g_fc_params.vel_z_i_limit, g_fc_params.vel_z_d_lpf);

    /* 仅高度速度环启用 anti-windup，其他环保持默认关闭 */
    height_vel_pid.aw_enable = 1U;
    height_vel_pid.aw_gain = 0.30f;
    height_vel_pid.output_min = -1500.0f;
    height_vel_pid.output_max = 1500.0f;
}

void FC_Loop_Reset(void)
{
    PID_Reset(&roll_angle_pid);
    PID_Reset(&pitch_angle_pid);
    PID_Reset(&yaw_angle_pid);
    PID_Reset(&roll_gyro_pid);
    PID_Reset(&pitch_gyro_pid);
    PID_Reset(&yaw_gyro_pid);
    PID_Reset(&height_pos_pid);
    PID_Reset(&height_vel_pid);
}

void FC_Loop_50Hz(void)
{
    static uint8 s_height_pos_sp_inited = 0U;
    static uint32 tick_500us_cnt_last = 0;
    uint32 tick_now = tick_500us_cnt; // 读一次，缓存
    uint32 diff = tick_now - tick_500us_cnt_last;
    float dt = diff * 0.0005f; // 秒
    float height_pos_out_raw;
    float pos_sp_step_max;

    tick_500us_cnt_last = tick_now;
    if (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING)
    {
        // FC_ThrTrim_Update_100Hz();
        float height_m = g_height_est_m;
        height_pos_out_raw = PID_Update(&height_pos_pid, target_height_m, height_m, dt);
        height_pos_out_raw = fc_clampf(height_pos_out_raw, -0.26f, 0.26f);

        if (0U == s_height_pos_sp_inited)
        {
            height_pos_out = height_pos_out_raw;
            s_height_pos_sp_inited = 1U;
        }

        /* 限制高度外环速度指令变化率，降低低频慢摆引起的内环激励 */
        pos_sp_step_max = 0.040f; /* 精度档：50Hz下约 2.0 m/s^2，减小外环激励 */
        height_pos_out += fc_clampf(height_pos_out_raw - height_pos_out,
                                    -pos_sp_step_max,
                                    pos_sp_step_max);

    }
    else
    {
        s_height_pos_sp_inited = 0U;
        height_pos_out = 0.0f;
    }
}


void FC_Loop_100Hz(void)
{
    static float s_height_vz_ctrl_lpf = 0.0f;
    static float s_height_vel_out_lpf = 0.0f;
    static uint8 s_height_vz_ctrl_lpf_inited = 0U;
    static uint8 s_height_vel_out_lpf_inited = 0U;
    static uint32 tick_500us_cnt_last = 0;
    FC_START_CRSF_state_e state;
    float vz_lpf_alpha;
    float out_lpf_alpha;
    float height_vz_ctrl_mps;
    float height_vel_out_raw;
    float status_code;
    const float vz_lpf_tau_s = 0.045f;
    const float out_lpf_tau_s = 0.030f;
    const float height_vel_out_min = -1500.0f;
    const float height_vel_out_max = 1500.0f;
    uint32 tick_now = tick_500us_cnt; // 读一次，缓存
    uint32 diff = tick_now - tick_500us_cnt_last;
    float dt = diff * 0.0005f; // 秒

    tick_500us_cnt_last = tick_now;
    state = FC_START_CRSF_Get_State();
    if (state == FC_START_CRSF_STATE_FLYING)
    {
        float ch0 = fc_clampf((float)CRSF_STD[0], -1000.0f, 1000.0f);
        float ch1 = fc_clampf((float)CRSF_STD[1], -1000.0f, 1000.0f);
        float ch2 = fc_clampf((float)CRSF_STD[2], -1000.0f, 1000.0f);

        target_height_m = (ch2 + 1000.0f) * (1.5f / 2000.0f); /* CH2: 0m~1.5m */
        roll_angle_target = fc_clampf(ch0 * (20.0f / 1000.0f) + FC_ROLL_MECH_TRIM_DEG, -20.0f, 20.0f);   /* roll>0 右倾 */
        pitch_angle_target = fc_clampf(-ch1 * (20.0f / 1000.0f) + FC_PITCH_MECH_TRIM_DEG, -20.0f, 20.0f); /* pitch>0 抬头，前倾为负 */

        if (0U == s_height_vz_ctrl_lpf_inited)
        {
            s_height_vz_ctrl_lpf = g_height_vz_mps;
            s_height_vz_ctrl_lpf_inited = 1U;
        }
        if (0U == s_height_vel_out_lpf_inited)
        {
            s_height_vel_out_lpf = height_vel_out;
            s_height_vel_out_lpf_inited = 1U;
        }

        if (dt < 0.0001f)
        {
            dt = 0.01f;
        }
        vz_lpf_alpha = dt / (vz_lpf_tau_s + dt);
        vz_lpf_alpha = fc_clampf(vz_lpf_alpha, 0.0f, 1.0f);
        out_lpf_alpha = dt / (out_lpf_tau_s + dt);
        out_lpf_alpha = fc_clampf(out_lpf_alpha, 0.0f, 1.0f);

        s_height_vz_ctrl_lpf += vz_lpf_alpha * (g_height_vz_mps - s_height_vz_ctrl_lpf);
        height_vz_ctrl_mps = s_height_vz_ctrl_lpf;

        height_vel_out_raw = PID_Update(&height_vel_pid, height_pos_out, height_vz_ctrl_mps, dt);
        height_vel_out_raw = fc_clampf(height_vel_out_raw, height_vel_out_min, height_vel_out_max);

        s_height_vel_out_lpf += out_lpf_alpha * (height_vel_out_raw - s_height_vel_out_lpf);
        height_vel_out = fc_clampf(s_height_vel_out_lpf, height_vel_out_min, height_vel_out_max);

        status_code = (float)g_tof_fused_valid +
                      10.0f * (float)g_height_est_valid +
                      100.0f * (float)g_height_est_source;

        wifi_vofa_JustFloat(16U,
                            target_height_m,
                            g_height_est_m,
                            height_pos_out,
                            g_height_vz_mps,
                            height_vz_ctrl_mps,
                            height_vel_pid.p_term,
                            height_vel_pid.i_term,
                            height_vel_pid.d_term,
                            height_vel_out,
                            (float)g_fc_params.base_throttle + height_vel_out,
                            (float)g_tof2_height_mm * 0.001f,
                            (float)g_tof3_height_mm * 0.001f,
                            (float)g_tof_fused_height_mm * 0.001f,
                            g_baro_altitude,
                            g_baro_prop_bias_hat_pa,
                            status_code);
    }
    else
    {
        s_height_vz_ctrl_lpf_inited = 0U;
        s_height_vel_out_lpf_inited = 0U;
        height_vel_out = 0.0f;
    }
}

void FC_Loop_500Hz(void)
{
    static uint32 tick_500us_cnt_last = 0;
    uint32 tick_now = tick_500us_cnt; // 读一次，缓存
    uint32 diff = tick_now - tick_500us_cnt_last;
    float dt = diff * 0.0005f; // 秒

    tick_500us_cnt_last = tick_now;
    if (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING)
    {
        float roll_angle_meas = g_euler.roll;
        float pitch_angle_meas = g_euler.pitch;
        float yaw_angle_meas = g_euler.yaw;

        /* 控制量限幅 */
        float limit = 10000.0f;
        int32_t roll_ctrl = (int32_t)fc_clampf(PID_Update(&roll_angle_pid, roll_angle_target, roll_angle_meas, dt), -limit, limit);
        int32_t pitch_ctrl = (int32_t)fc_clampf(PID_Update(&pitch_angle_pid, pitch_angle_target, pitch_angle_meas, dt), -limit, limit);
        int32_t yaw_ctrl = (int32_t)fc_clampf(PID_Update(&yaw_angle_pid, yaw_angle_target, yaw_angle_meas, dt), -limit, limit);

        (void)yaw_ctrl;
        roll_gyro_target = roll_ctrl;
        pitch_gyro_target = pitch_ctrl;
        yaw_gyro_target = 0; // 不闭环航向角，保持当前值

    }
}

void FC_Loop_2000Hz(void)
{
    static uint32 tick_500us_cnt_last = 0;
    uint32 tick_now = tick_500us_cnt; // 读一次，缓存
    uint32 diff = tick_now - tick_500us_cnt_last;
    float dt = diff * 0.0005f; // 秒

    tick_500us_cnt_last = tick_now;
    if (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING)
    {
        /* 读取当前角速度 */
        float roll_gyro_meas = g_imu_filter.gyro_filt_x;
        float pitch_gyro_meas = g_imu_filter.gyro_filt_y;
        float yaw_gyro_meas = g_imu_filter.gyro_filt_z;

        /* 控制量限幅 */
        float limit = 10000.0f;
        int32_t roll_ctrl = (int32_t)fc_clampf(PID_Update(&roll_gyro_pid, roll_gyro_target, roll_gyro_meas, dt), -limit, limit);
        int32_t pitch_ctrl = (int32_t)fc_clampf(PID_Update(&pitch_gyro_pid, pitch_gyro_target, pitch_gyro_meas, dt), -limit, limit);
        int32_t yaw_ctrl = (int32_t)fc_clampf(PID_Update(&yaw_gyro_pid, yaw_gyro_target, yaw_gyro_meas, dt), -limit, limit);

        (void)yaw_ctrl;
        /* 电机混控：总油门 = 基础油门 + 高度控制输出 */
        g_motor_cmd.throttle = g_fc_params.base_throttle+ (int32_t)height_vel_out;
        g_motor_cmd.roll = roll_ctrl;
        g_motor_cmd.pitch = -pitch_ctrl;
        g_motor_cmd.yaw = yaw_ctrl;

        Motor_Mixer(&g_motor_cmd);
    }
}

