#include "fc_mode.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include <math.h>

/* 模式3手动姿态映射比例，单位度每遥控归一化单位 */
static const float s_mode3_rc_to_angle_scale = 0.04f;
/* 模式3手动姿态最大角度，单位度 */
static const float s_mode3_manual_angle_limit_deg = 20.0f;
/* 模式3 yaw 目标旋转速度，单位度每秒 */
static const float s_mode3_yaw_rate_dps = 60.0f;
static const float s_mode3_deg_to_rad = 0.0174532925f;
/* 模式3高度目标范围与变化速度，单位 m、m/s */
static const float s_mode3_height_min_m = 0.9f;
static const float s_mode3_height_max_m = 1.2f;
static const float s_mode3_height_rate_mps = 0.03f;

static uint32 s_mode3_random_state = 1U;
static uint16 s_mode3_yaw_reverse_count = 600U;
static int8 s_mode3_yaw_direction = 1;
static float s_mode3_yaw_target_deg = 0.0f;
static float s_mode3_height_target_m = 1.1f;
static float s_mode3_height_destination_m = 1.1f;

extern volatile uint32 tick_1000us_cnt;

static uint32 FC_Mode3_Random(void)
{
    s_mode3_random_state = s_mode3_random_state * 1664525U + 1013904223U;
    return s_mode3_random_state;
}

static float FC_Mode3_Random_Height_M(void)
{
    return s_mode3_height_min_m +
           (s_mode3_height_max_m - s_mode3_height_min_m) *
               (float)(FC_Mode3_Random() & 0xFFFFU) / 65535.0f;
}

/*
 * 函数名: FC_Mode3_Init
 * 功能: 初始化模式3控制资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode3_Init(void)
{
    FC_Mode3_Reset();
}

/*
 * 函数名: FC_Mode3_Reset
 * 功能: 复位模式3姿态、yaw旋转和高度漂移目标
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode3_Reset(void)
{
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    s_mode3_random_state = tick_1000us_cnt | 1U;
    s_mode3_yaw_target_deg = g_euler.yaw;
    s_mode3_yaw_direction = ((FC_Mode3_Random() & 0x10000U) != 0U) ? 1 : -1;
    s_mode3_yaw_reverse_count = (uint16)(600U + (FC_Mode3_Random() % 601U));
    s_mode3_height_target_m = FC_Mode_Clamp(g_fc_target_height_m,
                                             s_mode3_height_min_m,
                                             s_mode3_height_max_m);
    s_mode3_height_destination_m = FC_Mode3_Random_Height_M();
}

/*
 * 函数名: FC_Mode3_100Hz
 * 功能: 将全局坐标系摇杆目标转换为机体姿态目标，并更新随机yaw目标
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode3_100Hz(void)
{
    float ch0;
    float ch1;
    float world_roll_deg;
    float world_pitch_deg;
    float yaw_rad;
    float yaw_cos;
    float yaw_sin;

    if ((FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING) &&
        (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_LANDING))
    {
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0], -1000.0f, 1000.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1], -1000.0f, 1000.0f);

    world_roll_deg = FC_Mode_Clamp(ch0 * s_mode3_rc_to_angle_scale,
                                   -s_mode3_manual_angle_limit_deg, s_mode3_manual_angle_limit_deg);
    world_pitch_deg = FC_Mode_Clamp(-ch1 * s_mode3_rc_to_angle_scale,
                                    -s_mode3_manual_angle_limit_deg, s_mode3_manual_angle_limit_deg);
    yaw_rad = g_euler.yaw * s_mode3_deg_to_rad;
    yaw_cos = cosf(yaw_rad);
    yaw_sin = sinf(yaw_rad);

    roll_angle_target = FC_Mode_Clamp(world_roll_deg * yaw_cos + world_pitch_deg * yaw_sin +
                                          FC_Mode_Get_Roll_Mech_Trim_Deg(),
                                      -s_mode3_manual_angle_limit_deg, s_mode3_manual_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(-world_roll_deg * yaw_sin + world_pitch_deg * yaw_cos +
                                           FC_Mode_Get_Pitch_Mech_Trim_Deg(),
                                       -s_mode3_manual_angle_limit_deg, s_mode3_manual_angle_limit_deg);

    if (s_mode3_yaw_reverse_count == 0U)
    {
        s_mode3_yaw_direction = -s_mode3_yaw_direction;
        s_mode3_yaw_reverse_count = (uint16)(600U + (FC_Mode3_Random() % 601U));
    }
    s_mode3_yaw_reverse_count--;
    s_mode3_yaw_target_deg += (float)s_mode3_yaw_direction * s_mode3_yaw_rate_dps * 0.01f;
    if (s_mode3_yaw_target_deg > 180.0f)
    {
        s_mode3_yaw_target_deg -= 360.0f;
    }
    else if (s_mode3_yaw_target_deg < -180.0f)
    {
        s_mode3_yaw_target_deg += 360.0f;
    }
    yaw_angle_target = s_mode3_yaw_target_deg;
}

/*
 * 函数名: FC_Mode3_50Hz
 * 功能: 缓慢更新模式3随机高度目标
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode3_50Hz(float dt)
{
    float step = s_mode3_height_rate_mps * dt;

    if (s_mode3_height_target_m < s_mode3_height_destination_m)
    {
        s_mode3_height_target_m += step;
        if (s_mode3_height_target_m >= s_mode3_height_destination_m)
        {
            s_mode3_height_target_m = s_mode3_height_destination_m;
            s_mode3_height_destination_m = FC_Mode3_Random_Height_M();
        }
    }
    else
    {
        s_mode3_height_target_m -= step;
        if (s_mode3_height_target_m <= s_mode3_height_destination_m)
        {
            s_mode3_height_target_m = s_mode3_height_destination_m;
            s_mode3_height_destination_m = FC_Mode3_Random_Height_M();
        }
    }
}

float FC_Mode3_Get_Target_Height_M(void)
{
    return s_mode3_height_target_m;
}
