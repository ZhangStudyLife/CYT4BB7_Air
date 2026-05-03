#include "fc_loop.h"
#include "fc_mode.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../Estimation/Height_Est/Height_Est.h"

pid_t roll_gyro_pid;
pid_t pitch_gyro_pid;
pid_t yaw_gyro_pid;
pid_t roll_angle_pid;
pid_t pitch_angle_pid;
pid_t yaw_angle_pid;
pid_t height_pos_pid;
pid_t height_vel_pid;
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
float target_height_m = 0.0f;
extern volatile uint32 tick_1000us_cnt;

/* 当前高度速度估计，仅供本文件高度速度环使用，单位 m/s */
static float s_height_vz_mps = 0.0f;
/* 目标高度斜坡限速器状态 */
static float s_target_height_slew_m = 0.0f;
static uint8_t s_height_slew_inited = 0U;
/* 目标高度上升斜坡限速，单位 m/s */
#define FC_TARGET_H_RAMP_UP_MPS 0.15f
/* 目标高度下降斜坡限速，单位 m/s */
#define FC_TARGET_H_RAMP_DOWN_MPS 0.15f
/* 100Hz 锁存的飞行模式，50Hz 控制只消费该锁存值 */
static FC_START_CRSF_flight_mode_e s_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
/* 上一次锁存的飞行模式，用于检测模式切换边沿 */
static FC_START_CRSF_flight_mode_e s_prev_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
/* 上一次飞控状态，用于检测飞行态切换边沿 */
static FC_START_CRSF_state_e s_prev_fc_state = FC_START_CRSF_STATE_INIT;
/* 悬停油门在线学习（借鉴 ArduPilot MOT_THST_HOVER） */
static float s_hover_throttle = 3150.0f;
#define FC_HOVER_THR_TC 3.0f /* 学习时间常数，秒 */
#define FC_HOVER_THR_MIN 2800.0f
#define FC_HOVER_THR_MAX 3500.0f
#define FC_HOVER_LEARN_VZ_MAX 0.3f   /* |vz|<此值时才学习，m/s */
#define FC_HOVER_LEARN_POS_MAX 0.05f /* |pos_out|<此值时才学习，m/s */
/* 高度速度环输出最小限幅 */
static const float s_fc_height_vel_out_min = -1500.0f;
/* 高度速度环输出最大限幅 */
static const float s_fc_height_vel_out_max = 1500.0f;
/* 姿态角外环输出到角速度目标的限幅，单位 deg/s */
static const float s_fc_angle_out_limit = 260.0f;
/* 姿态角外环 anti-windup 回算增益 */
static const float s_fc_angle_aw_gain = 0.15f;
/* 姿态角外环积分松弛阈值，目标变化过快时降低积分堆积 */
static const float s_fc_angle_iterm_relax_threshold = 30.0f;
static const float s_fc_deg_to_rad = 0.017453293f;
static const float s_fc_tilt_cos_min = 0.8f;
static const float s_fc_tilt_comp_throttle_max = 10000.0f;

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
 * 函数名: FC_Apply_Tilt_Throttle_Compensation
 * 功能: 使用当前 Roll/Pitch 原始姿态角，对总油门做保守的垂向分力补偿
 * 输入参数:
 *   throttle_raw - 补偿前总油门，单位 mixer 输入
 * 返回值:
 *   补偿后的总油门，已限幅到[0, s_fc_tilt_comp_throttle_max]
 */
static float FC_Apply_Tilt_Throttle_Compensation(float throttle_raw)
{
    float cos_term;

    if (throttle_raw <= 0.0f)
    {
        return 0.0f;
    }

    /* 直接使用当前原始姿态角，补偿相对重力方向的总倾斜损失 */
    cos_term = cosf(g_euler.roll * s_fc_deg_to_rad) * cosf(g_euler.pitch * s_fc_deg_to_rad);
    cos_term = fc_clampf(cos_term, s_fc_tilt_cos_min, 1.0f);

    return fc_clampf(throttle_raw / cos_term, 0.0f, s_fc_tilt_comp_throttle_max);
}

/*
 * 函数名: FC_Map_TargetHeightFromCh2
 * 功能: 将遥控器 CH2 映射为目标高度
 * 输入参数:
 *   ch2_std - CRSF 标准化通道值，范围[-1000,1000]
 * 返回值:
 *   目标高度，单位 m
 * 0325 0129 : 修改映射关系,CH2 1000 对应1.2M,-1000 对应-0.2M(如果目标是0M,当前控制,会保持在0.1M,所以故意将-1000 映射到-0.2M,增加下行余量)
 * 0404 1944 : 修改映射关系,CH2 1000 对应1.2M,-1000 对应0M(安装了10cm的脚撑)
 */
static float FC_Map_TargetHeightFromCh2(float ch2_std)
{
    return (ch2_std + 1000.0f) * 0.0006f;
}

/*
 * 函数名: FC_Reset_All_Mode_Control
 * 功能: 统一复位所有模式控制状态
 * 输入参数: 无
 * 返回值: 无
 */
static void FC_Reset_All_Mode_Control(void)
{
    FC_Mode0_Reset();
    FC_Mode1_Reset();
    FC_Mode2_Reset();
    FC_Mode3_Reset();
    FC_Mode4_Reset();
    FC_Mode5_Reset();
    FC_Mode6_Reset();
    FC_Mode7_Reset();
    FC_Mode8_Reset();
}

/*
 * 函数名: FC_Handle_Mode_Transition_100Hz
 * 功能: 在100Hz统一处理模式切换与飞行状态切换复位
 * 输入参数:
 *   flight_mode - 当前锁存飞行模式
 *   fc_state    - 当前飞控状态
 * 返回值: 无
 */
static void FC_Handle_Mode_Transition_100Hz(FC_START_CRSF_flight_mode_e flight_mode,
                                            FC_START_CRSF_state_e fc_state)
{
    uint8 need_reset = 0U;

    if (s_prev_flight_mode != flight_mode)
    {
        need_reset = 1U;
    }
    if (s_prev_fc_state != fc_state)
    {
        need_reset = 1U;
    }

    if (need_reset != 0U)
    {
        FC_Reset_All_Mode_Control();
    }

    s_prev_flight_mode = flight_mode;
    s_prev_fc_state = fc_state;
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
    /* Roll 姿态外环启用工程限幅和 anti-windup，避免持续偏差把角速度目标推得过猛 */
    roll_angle_pid.aw_enable = 1U;
    roll_angle_pid.aw_gain = s_fc_angle_aw_gain;
    roll_angle_pid.output_min = -s_fc_angle_out_limit;
    roll_angle_pid.output_max = s_fc_angle_out_limit;
    roll_angle_pid.iterm_relax_threshold = s_fc_angle_iterm_relax_threshold;
    PID_Init(&pitch_angle_pid,
             g_fc_params.pitch_angle_kp, g_fc_params.pitch_angle_ki, g_fc_params.pitch_angle_kd,
             g_fc_params.pitch_angle_kff, g_fc_params.angle_dt,
             g_fc_params.pitch_angle_i_limit, g_fc_params.pitch_angle_d_lpf);
    /* Pitch 姿态外环与 Roll 保持相同保护策略，减小两轴控制品质分叉 */
    pitch_angle_pid.aw_enable = 1U;
    pitch_angle_pid.aw_gain = s_fc_angle_aw_gain;
    pitch_angle_pid.output_min = -s_fc_angle_out_limit;
    pitch_angle_pid.output_max = s_fc_angle_out_limit;
    pitch_angle_pid.iterm_relax_threshold = s_fc_angle_iterm_relax_threshold;
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

    /* 仅高度速度环启用 anti-windup，其他环保持默认关闭 */
    height_vel_pid.aw_enable = 1U;
    height_vel_pid.aw_gain = 0.30f;
    height_vel_pid.output_min = s_fc_height_vel_out_min;
    height_vel_pid.output_max = s_fc_height_vel_out_max;
    /* 高度速度目标快速变化时提前放松积分，保留电池压降积分补偿，同时避免阶跃时积分抢主控制 */
    height_vel_pid.iterm_relax_threshold = 1.5f;

    FC_Mode0_Init();
    FC_Mode1_Init();
    FC_Mode2_Init();
    FC_Mode3_Init();
    FC_Mode4_Init();
    FC_Mode5_Init();
    FC_Mode6_Init();
    FC_Mode7_Init();
    FC_Mode8_Init();
    FC_Reset_All_Mode_Control();
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
    FC_Reset_All_Mode_Control();
    roll_gyro_target = 0.0f;
    pitch_gyro_target = 0.0f;
    yaw_gyro_target = 0.0f;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    yaw_angle_target = 0.0f;
    height_pos_out = 0.0f;
    height_vel_out = 0.0f;
    s_height_vz_mps = 0.0f;
    s_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
    s_prev_flight_mode = FC_START_CRSF_FLIGHT_MODE_0;
    s_prev_fc_state = FC_START_CRSF_STATE_INIT;
    target_height_m = 1.0f;
    s_height_slew_inited = 0U;
    s_target_height_slew_m = 0.0f;
    s_hover_throttle = (float)g_fc_params.base_throttle;
}

/*
 * 函数名: FC_Loop_50Hz
 * 功能: 执行50Hz高度位置环，并分发对应模式的50Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Loop_50Hz(void)
{
    static uint32 tick_1000us_cnt_last = 0;
    uint32 tick_now = tick_1000us_cnt;
    uint32 diff = tick_now - tick_1000us_cnt_last;
    float dt = diff * 0.001f;
    FC_START_CRSF_state_e fc_state = FC_START_CRSF_Get_State();
    float height_m;

    tick_1000us_cnt_last = tick_now;
    if (dt < 0.0001f)
    {
        dt = 0.02f;
    }

    if ((fc_state == FC_START_CRSF_STATE_FLYING) && (0U != g_tof_fused_valid))
    {
        height_m = g_tof_fused_height_mm * 0.001f;
        height_pos_out = PID_Update(&height_pos_pid, target_height_m, height_m, dt);
        height_pos_out = fc_clampf(height_pos_out, -0.40f, 0.25f);
    }
    else
    {
        height_pos_out = 0.0f;
    }

    switch (s_flight_mode)
    {
    case FC_START_CRSF_FLIGHT_MODE_0:
        FC_Mode0_50Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_1:
        FC_Mode0_50Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_2:
        FC_Mode0_50Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_3:
        FC_Mode0_50Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_4:
        FC_Mode1_50Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_5:
        FC_Mode5_50Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_6:
        FC_Mode0_50Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_7:
        FC_Mode2_50Hz(dt);
        break;

    case FC_START_CRSF_FLIGHT_MODE_8:
        FC_Mode0_50Hz(dt);
        break;

    default:
        FC_Mode0_50Hz(dt);
        break;
    }
}

/*
 * 函数名: FC_Loop_100Hz
 * 功能: 执行100Hz高度测速、高度速度环与模式分发
 * 输入参数: 无
 * 返回值: 无
 * 在04 11 0213测试中 调用一次花费时间 792.5us
 */
void FC_Loop_100Hz(void)
{
    static uint32 tick_1000us_cnt_last = 0;
    FC_START_CRSF_state_e fc_state;
    float ch2;
    float height_m;
    float height_err_mm;
    float throttle_z_cmd;
    uint32 tick_now = tick_1000us_cnt;
    uint32 diff = tick_now - tick_1000us_cnt_last;
    float dt = diff * 0.001f;

    tick_1000us_cnt_last = tick_now;
    if (dt < 0.0001f)
    {
        dt = 0.01f;
    }

    height_m = g_tof_fused_height_mm * 0.001f;

    // wifi_justfloat(tick_1000us_cnt,g_tof1_height_mm,g_tof4_height_mm,g_tof_fused_height_mm,height_vz_raw_mps,dt,
    // g_imufilter_1000hz.accx,g_imufilter_1000hz.accy,g_imufilter_1000hz.accz,g_euler.pitch,g_euler.roll,g_euler.yaw);

    s_height_vz_mps = g_height_fused_vz_mps;
    fc_state = FC_START_CRSF_Get_State();
    s_flight_mode = FC_START_CRSF_Get_Flight_Mode(); /* 检测遥控器的模式 */
    FC_Handle_Mode_Transition_100Hz(s_flight_mode, fc_state);

    if (fc_state == FC_START_CRSF_STATE_FLYING)
    {
        /*定高的目标高度获取来源，模式5为固定高度模式，其他模式为CH2映射 */
        if (s_flight_mode == FC_START_CRSF_FLIGHT_MODE_5)
        {
            target_height_m = FC_Mode5_Get_Fixed_Height_M();
        }
        else
        {
            ch2 = fc_clampf((float)CRSF_STD[2], -1000.0f, 1000.0f);
            target_height_m = FC_Map_TargetHeightFromCh2(ch2);
        }
        /* 目标高度斜坡限速：首次进入FLYING时从当前实测高度开始 */
        if (0U == s_height_slew_inited)
        {
            s_target_height_slew_m = height_m;
            s_height_slew_inited = 1U;
        }
        {
            float delta = target_height_m - s_target_height_slew_m;
            float max_up = FC_TARGET_H_RAMP_UP_MPS * dt;
            float max_down = -FC_TARGET_H_RAMP_DOWN_MPS * dt;
            if (delta > max_up)
            {
                delta = max_up;
            }
            if (delta < max_down)
            {
                delta = max_down;
            }
            s_target_height_slew_m += delta;
            target_height_m = s_target_height_slew_m;
        }
        if (0U == g_tof_fused_valid)
        {
            height_pos_out = 0.0f;
        }
        height_vel_out = PID_Update(&height_vel_pid, height_pos_out, s_height_vz_mps, dt);
        height_vel_out = fc_clampf(height_vel_out, s_fc_height_vel_out_min, s_fc_height_vel_out_max);
    }
    else
    {
        s_height_vz_mps = 0.0f;
        height_pos_out = 0.0f;
        height_vel_out = 0.0f;
        s_height_slew_inited = 0U;
    }

    height_err_mm = target_height_m * 1000.0f - g_tof_fused_height_mm;
    throttle_z_cmd = s_hover_throttle + height_vel_out;
    /* 悬停油门在线学习：仅在接近稳态悬停时更新 */
    if ((fc_state == FC_START_CRSF_STATE_FLYING) &&
        (s_height_vz_mps > -FC_HOVER_LEARN_VZ_MAX) && (s_height_vz_mps < FC_HOVER_LEARN_VZ_MAX) &&
        (height_pos_out > -FC_HOVER_LEARN_POS_MAX) && (height_pos_out < FC_HOVER_LEARN_POS_MAX))
    {
        float alpha = dt / (dt + FC_HOVER_THR_TC);
        s_hover_throttle += alpha * (throttle_z_cmd - s_hover_throttle);
        s_hover_throttle = fc_clampf(s_hover_throttle, FC_HOVER_THR_MIN, FC_HOVER_THR_MAX);
    }
    // // if (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING)
    // {
    //     wifi_justfloat(tick_1000us_cnt,
    //                    target_height_m * 1000.0f,
    //                    g_tof_fused_height_mm,
    //                    height_pos_out,
    //                    s_height_vz_mps,
    //                    height_vel_pid.p_term,
    //                    height_vel_pid.i_term,
    //                    height_vel_pid.d_term,
    //                    height_vel_out,
    //                    g_motor_cmd.throttle,
    //                    g_tof_fused_valid
    //                    );
    // }

    switch (s_flight_mode)
    {
    case FC_START_CRSF_FLIGHT_MODE_0:
        FC_Mode0_100Hz();
        break;

    case FC_START_CRSF_FLIGHT_MODE_1:
        FC_Mode0_100Hz();
        break;

    case FC_START_CRSF_FLIGHT_MODE_2:
        FC_Mode0_100Hz();
        break;

    case FC_START_CRSF_FLIGHT_MODE_3:
        FC_Mode0_100Hz();
        break;

    case FC_START_CRSF_FLIGHT_MODE_4:
        FC_Mode1_100Hz();
        break;

    case FC_START_CRSF_FLIGHT_MODE_5:
        FC_Mode5_100Hz();
        break;

    case FC_START_CRSF_FLIGHT_MODE_6:
        FC_Mode0_100Hz();
        break;

    case FC_START_CRSF_FLIGHT_MODE_7:
        FC_Mode2_100Hz();
        break;

    case FC_START_CRSF_FLIGHT_MODE_8:
        FC_Mode0_100Hz();
        break;

    default:
        FC_Mode0_100Hz();
        break;
    }

    // wifi_justfloat(tick_1000us_cnt,fc_state,
    //                target_height_m * 1000.0f,
    //                g_tof_fused_height_mm,
    //                roll_angle_target, g_euler.roll,
    //                pitch_angle_target, g_euler.pitch,
    //                roll_gyro_target, g_imufilter_1000hz.gyrox,
    //                pitch_gyro_target, g_imufilter_1000hz.gyroy,
    //                CRSF_STD[7]);

    // wifi_justfloat((float)tick_1000us_cnt,      /* 时间戳 */
    //                target_height_m * 1000.0f,   /* 目标高度，单位 mm */
    //                g_tof_fused_height_mm,       /* 当前高度，单位 mm */
    //                height_pos_out,              /* 速度目标(位置环输出) */
    //                s_height_vz_mps,             /* 速度反馈 */
    //                height_pos_pid.p_term,       /* 位置环P */
    //                height_pos_pid.d_term,       /* 位置环D */
    //                height_vel_pid.p_term,       /* 速度环P */
    //                height_vel_pid.i_term,       /* 速度环I */
    //                height_vel_out,              /* 速度环输出 */
    //                (float)g_motor_cmd.throttle, /* 总油门指令 */
    //                g_tof1_height_mm,
    //                g_tof2_height_mm,
    //                g_tof3_height_mm,
    //                g_tof4_height_mm);
    // wifi_justfloat(g_tof1_height_mm, g_tof2_height_mm, g_tof3_height_mm, g_tof4_height_mm);
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

        if (roll_angle_target > 20.0f)
        {
            roll_angle_target = 20.0f;
        }
        if (roll_angle_target < -20.0f)
        {
            roll_angle_target = -20.0f;
        }
        if (pitch_angle_target > 20.0f)
        {
            pitch_angle_target = 20.0f;
        }
        if (pitch_angle_target < -20.0f)
        {
            pitch_angle_target = -20.0f;
        }

        /* 控制量限幅 */
        float limit = s_fc_angle_out_limit;
        float roll_ctrl = fc_clampf(PID_Update(&roll_angle_pid, roll_angle_target, roll_angle_meas, dt), -limit, limit);
        float pitch_ctrl = fc_clampf(PID_Update(&pitch_angle_pid, pitch_angle_target, pitch_angle_meas, dt), -limit, limit);
        (void)PID_Update(&yaw_angle_pid, yaw_angle_target, yaw_angle_meas, dt);

        roll_gyro_target = roll_ctrl;
        pitch_gyro_target = pitch_ctrl;
        yaw_gyro_target = 0; // 不闭环航向角，保持当前值


    }
            // wifi_justfloat(tick_1000us_cnt,
            //            pitch_angle_target,
            //            g_euler.pitch,
            //            pitch_gyro_target,
            //            g_imufilter_1000hz.gyroy,
            //            ICM42688.gyro_y,
            //            pitch_gyro_pid.p_term,
            //            pitch_gyro_pid.i_term,
            //            pitch_gyro_pid.d_term,
            //            pitch_gyro_pid.output,
            //            g_motor_cmd.pitch,
            //            g_motor_cmd.throttle);
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
        // wifi_justfloat(pitch_gyro_target,
        //                         pitch_gyro_raw,这两个CSV文件是我离线标定的数据
        //                         pitch_gyro_meas,
        //                         pitch_ctrl,
        //                         pitch_gyro_pid.p_term,
        //                         pitch_gyro_pid.i_term,
        //                         pitch_gyro_pid.d_term,
        //                         pitch_gyro_pid.error,
        //                         8u);
        (void)yaw_ctrl;
        /* 电机混控：总油门 = 基础油门 + 高度控制输出 */
        {

            float base_throttle = FC_Apply_Tilt_Throttle_Compensation(g_fc_params.base_throttle);
            float throttle_cmd_raw = base_throttle + height_vel_out;
            g_motor_cmd.throttle = (int32_t)fc_clampf((float)(int32_t)throttle_cmd_raw, 2500.0f, 6000.0f);
        }
        g_motor_cmd.roll = roll_ctrl;
        g_motor_cmd.pitch = -pitch_ctrl;
        g_motor_cmd.yaw = yaw_ctrl;

        // 测试固定的控制输出
        g_motor_cmd.roll = 0;
        g_motor_cmd.pitch = -0;
        g_motor_cmd.yaw = 0;
        g_motor_cmd.throttle = 3000;
        // CRSF_STD[2] -1000~1000 映射到油门的 2600 ~ 5200
        // g_motor_cmd.throttle = (int32_t)(CRSF_STD[2] * 1.3f + 3900.0f);

        Motor_Mixer(&g_motor_cmd);
    }

    // wifi_justfloat(tick_1000us_cnt,
    //     g_imufilter_1000hz.gyrox,roll_gyro_target,roll_gyro_pid.p_term,roll_gyro_pid.i_term,roll_gyro_pid.d_term,
    //     g_euler.roll,roll_angle_target,roll_angle_pid.p_term,roll_angle_pid.i_term,roll_angle_pid.d_term,
    //     pitch_angle_target,g_euler.pitch,g_tof_fused_height_mm,target_height_m * 1000.0f
    //                );

    // float dec_x;
    // float dec_y;
    // dec_x = FlowGyroDecoupler_GetDecX();
    // dec_y = FlowGyroDecoupler_GetDecY();
    // wifi_justfloat(tick_1000us_cnt,
    //     g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY,g_pmw3901_raw.squal,
    //     g_imudata_250hz.gyrox, g_imudata_250hz.gyroy, g_imudata_250hz.gyroz,
    //     g_imudata_250hz.accx, g_imudata_250hz.accy, g_imudata_250hz.accz,
    //     g_euler.pitch, g_euler.roll, g_euler.yaw,dec_x, dec_y);
}
