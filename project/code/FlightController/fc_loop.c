#include "fc_loop.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../Estimation/Height_Est/TOF_data.h"

pid_t roll_gyro_pid;
pid_t pitch_gyro_pid;
pid_t yaw_gyro_pid;
pid_t roll_angle_pid;
pid_t pitch_angle_pid;
pid_t yaw_angle_pid;
pid_t height_pos_pid;
pid_t height_vel_pid;
pid_t velx_pid;
pid_t vely_pid;
/* Roll 角速度目标，单位度每秒 */
float roll_gyro_target = 0.0f;
/* Pitch 角速度目标，单位度每秒 */
float pitch_gyro_target = 0.0f;
/* Yaw 角速度目标，单位度每秒 */
float yaw_gyro_target = 0.0f;
/* Roll 角度目标，单位度 */
float roll_angle_target = 0.0f;
/* Pitch 角度目标，单位度 */
float pitch_angle_target = 0.0f;
/* Yaw 角度目标，单位度 */
float yaw_angle_target = 0.0f;
/* 高度速度环输出，单位 PWM */
float height_vel_out = 0.0f;
/* 高度位置环输出，单位米每秒 */
float height_pos_out = 0.0f;
/* 目标高度，单位米 */
float target_height_m = 1.0f;
extern volatile uint32 tick_1000us_cnt;

/* 当前高度速度估计，仅供本文件高度速度环使用，单位 m/s */
static float s_height_vz_mps = 0.0f;
/* 100Hz 锁存的飞行模式，50Hz 水平速度环只消费该锁存值 */
static FC_START_CRSF_flight_mode_e s_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
/* 上一次锁存的飞行模式，用于检测模式切换边沿 */
static FC_START_CRSF_flight_mode_e s_prev_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
/* 上一次飞控状态，用于检测飞行态切换边沿 */
static FC_START_CRSF_state_e s_prev_fc_state = FC_START_CRSF_STATE_INIT;
/* Roll 机械配平角，单位度 */
static const float s_fc_roll_mech_trim_deg = -2.6f;
/* Pitch 机械配平角，单位度 */
static const float s_fc_pitch_mech_trim_deg = 4.2f;
/* 高度速度环输出最小限幅 */
static const float s_fc_height_vel_out_min = -1500.0f;
/* 高度速度环输出最大限幅 */
static const float s_fc_height_vel_out_max = 1500.0f;

/*
 * 函数名: fc_clampf
 * 功能: 对浮点数进行上下限钳位
 * 输入参数:
 *   value     - 输入值
 *   min_value - 最小允许值
 *   max_value - 最大允许值
 * 返回值:
 *   限幅后的浮点值
 */
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
 * 函数名: FC_Map_TargetHeightFromCh2
 * 功能: 将遥控器 CH2 映射为目标高度
 * 输入参数:
 *   ch2_std - CRSF 标准化通道值，范围[-1000,1000]
 * 返回值:
 *   目标高度，单位 m
 */
static float FC_Map_TargetHeightFromCh2(float ch2_std)
{
    return (ch2_std + 1000.0f) * 0.0005f;
}

/*
 * 函数名: FC_Reset_Mode1_XY_Control
 * 功能: 清空模式1水平速度环状态，并将水平角度目标拉回机械配平
 * 输入参数: 无
 * 返回值: 无
 */
static void FC_Reset_Mode1_XY_Control(void)
{
    PID_Reset(&velx_pid);
    PID_Reset(&vely_pid);
    roll_angle_target = s_fc_roll_mech_trim_deg;
    pitch_angle_target = s_fc_pitch_mech_trim_deg;
}

/*
 * 函数名: FC_Handle_Mode1_XY_Transition_100Hz
 * 功能: 在100Hz统一处理模式1水平速度环的模式切换与飞行状态切换复位
 * 输入参数:
 *   flight_mode - 当前锁存飞行模式
 *   fc_state    - 当前飞控状态
 * 返回值: 无
 */
static void FC_Handle_Mode1_XY_Transition_100Hz(FC_START_CRSF_flight_mode_e flight_mode,
                                                FC_START_CRSF_state_e fc_state)
{
    uint8 need_reset = 0U;

    if ((s_prev_flight_mode != FC_START_CRSF_FLIGHT_MODE_1) &&
        (flight_mode == FC_START_CRSF_FLIGHT_MODE_1))
    {
        need_reset = 1U;
    }
    if ((s_prev_flight_mode == FC_START_CRSF_FLIGHT_MODE_1) &&
        (flight_mode != FC_START_CRSF_FLIGHT_MODE_1))
    {
        need_reset = 1U;
    }
    if (((s_prev_fc_state == FC_START_CRSF_STATE_FLYING) ||
         (fc_state == FC_START_CRSF_STATE_FLYING)) &&
        (s_prev_fc_state != fc_state) &&
        ((s_prev_flight_mode == FC_START_CRSF_FLIGHT_MODE_1) ||
         (flight_mode == FC_START_CRSF_FLIGHT_MODE_1)))
    {
        need_reset = 1U;
    }

    if (need_reset != 0U)
    {
        FC_Reset_Mode1_XY_Control();
    }

    s_prev_flight_mode = flight_mode;
    s_prev_fc_state = fc_state;
}

/*
 * 函数名: FC_Update_Mode1_XY_50Hz
 * 功能: 在50Hz节拍下执行模式1的水平速度环，并输出姿态目标
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位 s
 * 返回值: 无
 */
static void FC_Update_Mode1_XY_50Hz(float dt)
{
    const float rc_to_speed_scale = 0.1f;
    const float mode1_vel_limit_cmps = 100.0f;
    const float mode1_angle_limit_deg = 20.0f;
    float ch0 = fc_clampf((float)CRSF_STD[0], -1000.0f, 1000.0f);
    float ch1 = fc_clampf((float)CRSF_STD[1], -1000.0f, 1000.0f);
    float velx_target = fc_clampf(ch0 * rc_to_speed_scale, -mode1_vel_limit_cmps, mode1_vel_limit_cmps);
    float vely_target = fc_clampf(-ch1 * rc_to_speed_scale, -mode1_vel_limit_cmps, mode1_vel_limit_cmps);
    float velx_out = PID_Update(&velx_pid, velx_target, -Pos_Est_vel_x_kf, dt);
    float vely_out = PID_Update(&vely_pid, vely_target, -Pos_Est_vel_y_kf, dt);

    velx_out = fc_clampf(velx_out, -mode1_angle_limit_deg, mode1_angle_limit_deg);
    vely_out = fc_clampf(vely_out, -mode1_angle_limit_deg, mode1_angle_limit_deg);

    roll_angle_target = velx_out + s_fc_roll_mech_trim_deg;
    pitch_angle_target = vely_out + s_fc_pitch_mech_trim_deg;
}

/*
 * 函数名: FC_Update_Manual_Angle_Target_100Hz
 * 功能: 在模式0和模式2下根据摇杆直接生成手动姿态目标
 * 输入参数: 无
 * 返回值: 无
 */
static void FC_Update_Manual_Angle_Target_100Hz(void)
{
    const float rc_to_angle_scale = 0.04f;
    const float manual_angle_limit_deg = 40.0f;
    float ch0 = fc_clampf((float)CRSF_STD[0], -1000.0f, 1000.0f);
    float ch1 = fc_clampf((float)CRSF_STD[1], -1000.0f, 1000.0f);

    roll_angle_target = fc_clampf(ch0 * rc_to_angle_scale + s_fc_roll_mech_trim_deg,
                                  -manual_angle_limit_deg, manual_angle_limit_deg);
    pitch_angle_target = fc_clampf(-ch1 * rc_to_angle_scale + s_fc_pitch_mech_trim_deg,
                                   -manual_angle_limit_deg, manual_angle_limit_deg);
}

void FC_ThrTrim_Update_100Hz(void)
{
}

/*
 * 函数名: FC_Loop_Init
 * 功能: 初始化飞控各级 PID 与控制输出默认状态
 * 输入参数: 无
 * 返回值: 无
 */
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
    /* Roll 角速度目标持续变化时，提前放松积分，减少低频拉扯和线缆外力带来的积分堆积 */
    roll_gyro_pid.iterm_relax_threshold = 40.0f;
    PID_Init(&pitch_gyro_pid,
             g_fc_params.pitch_gyro_kp, g_fc_params.pitch_gyro_ki, g_fc_params.pitch_gyro_kd,
             g_fc_params.pitch_gyro_kff, g_fc_params.gyro_dt,
             g_fc_params.pitch_gyro_i_limit, g_fc_params.pitch_gyro_d_lpf);
    /* Pitch 角速度环与 Roll 保持相同的积分放松策略，降低连续目标变化时的积分拖拽 */
    pitch_gyro_pid.iterm_relax_threshold = 40.0f;
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
    PID_Init(&velx_pid,
             g_fc_params.vel_x_kp, g_fc_params.vel_x_ki, g_fc_params.vel_x_kd,
             g_fc_params.vel_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_x_i_limit, g_fc_params.vel_x_d_lpf);
    PID_Init(&vely_pid,
             g_fc_params.vel_y_kp, g_fc_params.vel_y_ki, g_fc_params.vel_y_kd,
             g_fc_params.vel_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.vel_y_i_limit, g_fc_params.vel_y_d_lpf);

    /* 仅高度速度环启用 anti-windup，其他环保持默认关闭 */
    height_vel_pid.aw_enable = 1U;
    height_vel_pid.aw_gain = 0.30f;
    height_vel_pid.output_min = s_fc_height_vel_out_min;
    height_vel_pid.output_max = s_fc_height_vel_out_max;
    /* 高度速度目标快速变化时提前放松积分，保留电池压降积分补偿，同时避免阶跃时积分抢主控制 */
    height_vel_pid.iterm_relax_threshold = 1.5f;

    FC_Reset_Mode1_XY_Control();
}

/*
 * 函数名: FC_Loop_Reset
 * 功能: 复位飞控控制环内部状态与关键目标量
 * 输入参数: 无
 * 返回值: 无
 */
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
    FC_Reset_Mode1_XY_Control();
    roll_gyro_target = 0.0f;
    pitch_gyro_target = 0.0f;
    yaw_gyro_target = 0.0f;
    yaw_angle_target = 0.0f;
    height_pos_out = 0.0f;
    height_vel_out = 0.0f;
    s_height_vz_mps = 0.0f;
    s_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
    s_prev_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
    s_prev_fc_state = FC_START_CRSF_STATE_INIT;
    target_height_m = 1.0f;
}

/*
 * 函数名: FC_Loop_50Hz
 * 功能: 执行50Hz高度位置环，以及模式1的50Hz水平速度环
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Loop_50Hz(void)
{
    static uint32 tick_1000us_cnt_last = 0;
    uint32 tick_now = tick_1000us_cnt; // 读一次，缓存
    uint32 diff = tick_now - tick_1000us_cnt_last;
    float dt = diff * 0.001f; // 秒
    FC_START_CRSF_state_e fc_state = FC_START_CRSF_Get_State();
    float height_m;

    tick_1000us_cnt_last = tick_now;
    if (dt < 0.0001f)
    {
        dt = 0.02f;
    }

    if (fc_state == FC_START_CRSF_STATE_FLYING)
    {
        height_m = (float)g_tof_fused_height_mm * 0.001f;
        height_pos_out = PID_Update(&height_pos_pid, target_height_m, height_m, dt);
        height_pos_out = fc_clampf(height_pos_out, -1.0f, 0.8f);
    }
    else
    {
        height_pos_out = 0.0f;
    }

    if ((s_flight_mode == FC_START_CRSF_FLIGHT_MODE_1) && (fc_state == FC_START_CRSF_STATE_FLYING))
    {
        FC_Update_Mode1_XY_50Hz(dt);
    }
}

/*
 * 函数名: FC_Loop_100Hz
 * 功能: 执行100Hz高度测速、模式锁存、模式切换复位与手动模式姿态目标更新
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Loop_100Hz(void)
{
    static uint8 s_vl53_recover_div = 0U;
    static uint8 s_tof_hist_inited = 0U;
    static float s_height_prev_m = 0.0f;
    static float s_height_vz_lpf_mps = 0.0f;
    static uint32 tick_1000us_cnt_last = 0;
    FC_START_CRSF_state_e fc_state;
    float ch2;
    float height_m;
    float height_vz_raw_mps;
    const float vz_lpf_alpha = 0.08f;
    uint32 tick_now = tick_1000us_cnt; // 读一次，缓存
    uint32 diff = tick_now - tick_1000us_cnt_last;
    float dt = diff * 0.001f; // 秒
    TOF_update_100HZ();
    s_vl53_recover_div++;
    if (s_vl53_recover_div >= 10U)
    {
        s_vl53_recover_div = 0U;
        VL53L1X_recover_update_10HZ();
    }
    tick_1000us_cnt_last = tick_now;
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

    s_height_vz_mps = s_height_vz_lpf_mps;
    fc_state = FC_START_CRSF_Get_State();
    s_flight_mode = FC_START_CRSF_Get_Flight_Mode(); /* 检测遥控器的模式 */
    FC_Handle_Mode1_XY_Transition_100Hz(s_flight_mode, fc_state);

    if (fc_state == FC_START_CRSF_STATE_FLYING)
    {
        ch2 = fc_clampf((float)CRSF_STD[2], -1000.0f, 1000.0f);
        target_height_m = FC_Map_TargetHeightFromCh2(ch2);
        height_vel_out = PID_Update(&height_vel_pid, height_pos_out, s_height_vz_mps, dt);
        height_vel_out = fc_clampf(height_vel_out, s_fc_height_vel_out_min, s_fc_height_vel_out_max);

        if ((s_flight_mode == FC_START_CRSF_FLIGHT_MODE_0) ||
            (s_flight_mode == FC_START_CRSF_FLIGHT_MODE_2))
        {
            FC_Update_Manual_Angle_Target_100Hz();
        }
    }
    else
    {
        s_tof_hist_inited = 0U;
        s_height_vz_lpf_mps = 0.0f;
        s_height_vz_mps = 0.0f;
        height_pos_out = 0.0f;
        height_vel_out = 0.0f;
    }


    wifi_vofa_JustFloat(3u,g_euler.roll,g_euler.pitch,g_euler.yaw);

    // wifi_vofa_JustFloat(4u,g_pmw3901_raw.deltaX,g_pmw3901_raw.deltaY,roll_angle_target,pitch_angle_target);

    /* 光流调试：前4路保持原有顺序，后4路补充纹理质量与原始像素统计 */
    // wifi_vofa_JustFloat(8u,
    //                     g_pos_est_debug.raw_flow_dx_count,
    //                     g_pos_est_debug.raw_flow_dy_count,
    //                     g_pos_est_output.flow_valid,
    //                     g_pmw3901_raw.squal,
    //                     g_pmw3901_raw.rawDataSum,
    //                     g_pmw3901_raw.maxRawData,
    //                     g_pmw3901_raw.minRawData,
    //                     g_pmw3901_raw.motionOccured);
}

void FC_Loop_500Hz(void)
{
    static uint32 tick_1000us_cnt_last = 0;
    uint32 tick_now = tick_1000us_cnt; // 读一次，缓存
    uint32 diff = tick_now - tick_1000us_cnt_last;
    float dt = diff * 0.001f; // 秒

    tick_1000us_cnt_last = tick_now;
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

void FC_Loop_1000Hz(void)
{
    static uint32 tick_1000us_cnt_last = 0;
    uint32 tick_now = tick_1000us_cnt; // 读一次，缓存
    uint32 diff = tick_now - tick_1000us_cnt_last;
    float dt = diff * 0.001f; // 秒

    tick_1000us_cnt_last = tick_now;
    if (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING)
    {
        /* 读取当前角速度 */
        float roll_gyro_meas = g_imufilter_1000hz.gyrox;
        float pitch_gyro_meas = g_imufilter_1000hz.gyroy;
        float yaw_gyro_meas = g_imufilter_1000hz.gyroz;

        /* 控制量限幅 */
        float limit = 10000.0f;
        int32_t roll_ctrl = (int32_t)fc_clampf(PID_Update(&roll_gyro_pid, roll_gyro_target, roll_gyro_meas, dt), -limit, limit);
        int32_t pitch_ctrl = (int32_t)fc_clampf(PID_Update(&pitch_gyro_pid, pitch_gyro_target, pitch_gyro_meas, dt), -limit, limit);
        int32_t yaw_ctrl = (int32_t)fc_clampf(PID_Update(&yaw_gyro_pid, yaw_gyro_target, yaw_gyro_meas, dt), -limit, limit);
        /* 角速度环调试切换到 Pitch：目标、原始陀螺、滤波后陀螺、控制输出和 PID 分项 */
        // wifi_vofa_JustFloat(8u,
        //                     pitch_gyro_target,
        //                     pitch_gyro_raw,
        //                     pitch_gyro_meas,
        //                     pitch_ctrl,
        //                     pitch_gyro_pid.p_term,
        //                     pitch_gyro_pid.i_term,
        //                     pitch_gyro_pid.d_term,
        //                     pitch_gyro_pid.error);
        (void)yaw_ctrl;
        /* 电机混控：总油门 = 基础油门 + 高度控制输出 */
        g_motor_cmd.throttle = g_fc_params.base_throttle + (int32_t)height_vel_out;
        g_motor_cmd.roll = roll_ctrl;
        g_motor_cmd.pitch = -pitch_ctrl;
        g_motor_cmd.yaw = yaw_ctrl;

        Motor_Mixer(&g_motor_cmd);
    }
}
