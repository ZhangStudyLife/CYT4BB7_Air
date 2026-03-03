#include "fc_loop.h"
#include "../Estimation/Height_Est/TOF_data.h"

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

float g_height_est_m = 0.0f;
float g_height_vz_mps = 0.0f;
uint8 g_height_est_valid = 0U;
uint8 g_height_est_source = 0U;

#define FC_HEIGHT_SRC_NONE (0U)
#define FC_HEIGHT_SRC_TOF (1U)
#define FC_HEIGHT_SRC_BARO (2U)

#define FC_ROLL_MECH_TRIM_DEG (-2.0)
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

void FC_ThrTrim_Update_100Hz(void)
{
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
    static uint32 tick_500us_cnt_last = 0;
    uint32 tick_now = tick_500us_cnt; // 读一次，缓存
    uint32 diff = tick_now - tick_500us_cnt_last;
    float dt = diff * 0.0005f; // 秒
    float height_m;

    tick_500us_cnt_last = tick_now;
    if (dt < 0.0001f)
    {
        dt = 0.02f;
    }

    if (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING)
    {
        height_m = (float)g_tof_fused_height_mm * 0.001f;
        height_pos_out = PID_Update(&height_pos_pid, target_height_m, height_m, dt);
        height_pos_out = fc_clampf(height_pos_out, -1.0f, 0.8f);
    }
    else
    {
        height_pos_out = 0.0f;
    }
}

void FC_Loop_100Hz(void)
{
    static uint8 s_vl53_recover_div = 0U;
    static uint8 s_tof_hist_inited = 0U;
    static float s_height_prev_m = 0.0f;
    static float s_height_vz_lpf_mps = 0.0f;
    static uint32 tick_500us_cnt_last = 0;
    float height_m;
    float height_vz_raw_mps;
    const float vz_lpf_alpha = 0.08f;
    const float height_vel_out_min = -1500.0f;
    const float height_vel_out_max = 1500.0f;
    uint32 tick_now = tick_500us_cnt; // 读一次，缓存
    uint32 diff = tick_now - tick_500us_cnt_last;
    float dt = diff * 0.0005f; // 秒
    TOF_update_100HZ();
    s_vl53_recover_div++;
    if (s_vl53_recover_div >= 10U)
    {
        s_vl53_recover_div = 0U;
        VL53L1X_recover_update_10HZ();
    }
    tick_500us_cnt_last = tick_now;
    if (dt < 0.0001f)
    {
        dt = 0.01f;
    }
    height_m = (float)g_tof_fused_height_mm * 0.001f;
    if (0U == s_tof_hist_inited)
    {
        s_height_prev_m = height_m;
        s_height_vz_lpf_mps = 0.0f;
        s_tof_hist_inited = 1U;
    }

    height_vz_raw_mps = (height_m - s_height_prev_m) / dt;
    s_height_prev_m = height_m;
    s_height_vz_lpf_mps += vz_lpf_alpha * (height_vz_raw_mps - s_height_vz_lpf_mps);

    g_height_est_m = height_m;
    g_height_vz_mps = s_height_vz_lpf_mps;

    if (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING)
    {
        float ch0 = fc_clampf((float)CRSF_STD[0], -1000.0f, 1000.0f);
        float ch1 = fc_clampf((float)CRSF_STD[1], -1000.0f, 1000.0f);
        float ch2 = fc_clampf((float)CRSF_STD[2], -1000.0f, 1000.0f);

        target_height_m = (ch2 + 1000.0f) * (1 / 2000.0f);                                                /* CH2: 0m~1m */
        roll_angle_target = fc_clampf(ch0 * (40.0f / 1000.0f) + FC_ROLL_MECH_TRIM_DEG, -40.0f, 40.0f);    /* roll>0 右倾 */
        pitch_angle_target = fc_clampf(-ch1 * (40.0f / 1000.0f) + FC_PITCH_MECH_TRIM_DEG, -40.0f, 40.0f); /* pitch>0 抬头，前倾为负 */

        height_vel_out = PID_Update(&height_vel_pid, height_pos_out, g_height_vz_mps, dt);
        height_vel_out = fc_clampf(height_vel_out, height_vel_out_min, height_vel_out_max);
    }
    else
    {
        s_tof_hist_inited = 0U;
        height_vel_out = 0.0f;
    }
    float debug_state = (float)FC_START_CRSF_Get_State()*1 + (float)g_tof_fused_valid*10;
    wifi_vofa_JustFloat(16U,
                        target_height_m * 1000.0f,
                        g_tof_fused_height_mm,
                        height_pos_pid.p_term,
                        height_pos_pid.i_term,
                        height_pos_pid.d_term,
                        height_pos_out,
                        g_height_vz_mps,
                        height_vel_pid.p_term,
                        height_vel_pid.i_term,
                        height_vel_pid.d_term,
                        -AccelCalibration_GetAccelDownMps2(),
                    g_euler.roll,
                    g_euler.pitch,
                    roll_angle_target,
                    pitch_angle_target,debug_state
                );
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
        g_motor_cmd.throttle = g_fc_params.base_throttle + (int32_t)height_vel_out;
        g_motor_cmd.roll = roll_ctrl;
        g_motor_cmd.pitch = -pitch_ctrl;
        g_motor_cmd.yaw = yaw_ctrl;

        Motor_Mixer(&g_motor_cmd);
    }
}
