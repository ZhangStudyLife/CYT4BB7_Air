#include "fc_loop.h"

pid_t roll_gyro_pid;
pid_t pitch_gyro_pid;
pid_t yaw_gyro_pid;
pid_t roll_angle_pid;
pid_t pitch_angle_pid;
pid_t yaw_angle_pid;
float roll_gyro_target = 0.0f;
float pitch_gyro_target = 0.0f;
float yaw_gyro_target = 0.0f;
float roll_angle_target = 0.0f;
float pitch_angle_target = 0.0f;
float yaw_angle_target = 0.0f;

extern volatile uint32 tick_500us_cnt;

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
}

void FC_Loop_Reset(void)
{
    PID_Reset(&roll_angle_pid);
    PID_Reset(&pitch_angle_pid);
    PID_Reset(&yaw_angle_pid);
    PID_Reset(&roll_gyro_pid);
    PID_Reset(&pitch_gyro_pid);
    PID_Reset(&yaw_gyro_pid);
}

void FC_Loop_500Hz(void)
{
    static uint32 tick_500us_cnt_last = 0;
    uint32 tick_now = tick_500us_cnt;          // 读一次，缓存
    uint32 diff = tick_now - tick_500us_cnt_last;
    float dt = diff * 0.0005f;                 // 秒

    tick_500us_cnt_last = tick_now;
    if (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING)
    {
        /* 读取当前角速度*/
        float roll_angle_meas = g_euler.roll;
        float pitch_angle_meas = g_euler.pitch;
        float yaw_angle_meas = g_euler.yaw;

        /* 控制量限幅 */
        float limit = 10000.0f;
        int32_t roll_ctrl = (int32_t)fc_clampf(PID_Update(&roll_angle_pid, roll_angle_target, roll_angle_meas, dt), -limit, limit);
        int32_t pitch_ctrl = (int32_t)fc_clampf(PID_Update(&pitch_angle_pid, pitch_angle_target, pitch_angle_meas, dt), -limit, limit);
        int32_t yaw_ctrl = (int32_t)fc_clampf(PID_Update(&yaw_angle_pid, yaw_angle_target, yaw_angle_meas, dt), -limit, limit);

        
        roll_gyro_target = roll_ctrl;
        pitch_gyro_target = pitch_ctrl;
        yaw_gyro_target = 0;        //不闭环航向角，保持当前值

    }
}


void FC_Loop_2000Hz(void)
{
    static uint32 tick_500us_cnt_last = 0;
    uint32 tick_now = tick_500us_cnt;          // 读一次，缓存
    uint32 diff = tick_now - tick_500us_cnt_last;
    float dt = diff * 0.0005f;                 // 秒

    tick_500us_cnt_last = tick_now;
    if (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING)
    {
        /* 读取当前角速度*/
        float roll_gyro_meas = g_imu_filter.gyro_filt_x;
        float pitch_gyro_meas = g_imu_filter.gyro_filt_y;
        float yaw_gyro_meas = -g_imu_filter.gyro_filt_z;

        /* 控制量限幅 */
        float limit = 10000.0f;
        int32_t roll_ctrl = (int32_t)fc_clampf(PID_Update(&roll_gyro_pid, roll_gyro_target, roll_gyro_meas, dt), -limit, limit);
        int32_t pitch_ctrl = (int32_t)fc_clampf(PID_Update(&pitch_gyro_pid, pitch_gyro_target, pitch_gyro_meas, dt), -limit, limit);
        int32_t yaw_ctrl = (int32_t)fc_clampf(PID_Update(&yaw_gyro_pid, yaw_gyro_target, yaw_gyro_meas, dt), -limit, limit);

        /* 电机混控：总油门 = 基础油门 + 高度控制输出 */
        g_motor_cmd.throttle = g_fc_params.base_throttle;
        g_motor_cmd.roll = roll_ctrl;
        g_motor_cmd.pitch = -pitch_ctrl;
        g_motor_cmd.yaw = 0;

        Motor_Mixer(&g_motor_cmd);
    }
}
