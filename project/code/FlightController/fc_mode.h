/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#ifndef FC_MODE_H
#define FC_MODE_H

/*
 * 模式号与遥控开关位置映射表
 * CH5 为行选择，CH6 为列选择，两个通道均为三段开关，取值范围均为 0/1/2
 *
 * CH5=0, CH6=0 -> MODE0
 * CH5=0, CH6=1 -> MODE1
 * CH5=0, CH6=2 -> MODE2
 * CH5=1, CH6=0 -> MODE3
 * CH5=1, CH6=1 -> MODE4
 * CH5=1, CH6=2 -> MODE5
 * CH5=2, CH6=0 -> MODE6
 * CH5=2, CH6=1 -> MODE7
 * CH5=2, CH6=2 -> MODE8
 *
 * 当前模式号计算公式：
 * mode = CH5 * 3 + CH6
 *
 * 当前模式功能分配(这是实际使用的模式功能，和代码库里面的模式功能不一定对应，具体去看fc_loop.c里面50HZ，100HZ的状态机)：
 * MODE0 - 纯手动姿态
 * MODE1 - 纯手动姿态
 * MODE2 - 纯手动姿态
 * MODE3 - 纯手动姿态
 * MODE4 - 纯手动姿态
 * MODE5 - 固定高度位置保持模式，横向复用原模式1逻辑
 * MODE6 - 纯手动姿态
 * MODE7 - 速度环模式，复用原模式2逻辑
 * MODE8 - 纯手动姿态
 */

#include "fc_loop.h"

/* 图像位置环输出速度限幅，单位 cm/s。 */
#define FC_MODE_IMAGE_VEL_LIMIT_CMPS (400.0f)
/* 图像控制允许工作的最低融合高度，单位 mm。 */
#define FC_MODE_IMAGE_MIN_HEIGHT_MM (400.0f)
/* 水平速度前馈一阶低通系数：按调用频率保持截止频率不变。 */
#define FC_MODE_VEL_KFF_LPF_ALPHA_50HZ (0.672624f) /* 50Hz */
#define FC_MODE_VEL_KFF_LPF_ALPHA_100HZ (0.427832f) /* 100Hz */
/* 小车运行数据用于 yaw 坐标转换的最大未更新时间，单位 ms。 */
#define FC_MODE_CAR_RUN_DATA_TIMEOUT_MS (200U)
/* 模式7摇杆满量程对应的最大水平目标速度，单位 cm/s。 */
#define FC_MODE7_VEL_LIMIT_CMPS (200.0f)
/* 模式7归一化摇杆死区。 */
#define FC_MODE7_STICK_DEADZONE (0.03f)
/* 模式7三次速度曲线占比。 */
#define FC_MODE7_STICK_EXPO (0.40f)

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * 函数名: FC_Mode_Clamp
 * 功能: 对模式控制中的浮点数进行上下限钳位
 * 输入参数:
 *   value     - 输入值
 *   min_value - 最小允许值
 *   max_value - 最大允许值
 * 返回值:
 *   限幅后的浮点值
 */
static inline float FC_Mode_Clamp(float value, float min_value, float max_value)
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
 * 函数名: FC_Mode_Get_Roll_Mech_Trim_Deg
 * 功能: 获取 Roll 机械配平角
 * 输入参数: 无
 * 返回值:
 *   Roll 机械配平角，单位度
 */
static inline float FC_Mode_Get_Roll_Mech_Trim_Deg(void)
{
    return g_fc_params.roll_mech_trim_deg;
}

/*
 * 函数名: FC_Mode_Get_Pitch_Mech_Trim_Deg
 * 功能: 获取 Pitch 机械配平角
 * 输入参数: 无
 * 返回值:
 *   Pitch 机械配平角，单位度
 */
static inline float FC_Mode_Get_Pitch_Mech_Trim_Deg(void)
{
    return g_fc_params.pitch_mech_trim_deg;
}

/*
 * 函数名: FC_Mode0_Init
 * 功能: 初始化模式0控制所需资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode0_Init(void);

/*
 * 函数名: FC_Mode0_Reset
 * 功能: 复位模式0控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode0_Reset(void);

/*
 * 函数名: FC_Mode0_100Hz
 * 功能: 执行模式0的100Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode0_100Hz(void);

/*
 * 函数名: FC_Mode0_50Hz
 * 功能: 执行模式0的50Hz控制
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode0_50Hz(float dt);

/*
 * 函数名: FC_Mode1_Init
 * 功能: 初始化模式1控制所需资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_Init(void);

/*
 * 函数名: FC_Mode1_Reset
 * 功能: 复位模式1控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_Reset(void);

/*
 * 函数名: FC_Mode1_100Hz
 * 功能: 执行模式1的100Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode1_100Hz(void);

/*
 * 函数名: FC_Mode1_Control100Hz
 * 功能: 执行模式1的100Hz图像控制
 * 输入参数:
 *   dt - 本次100Hz调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode1_Control100Hz(float dt);

/*
 * 函数名: FC_Mode2_Init
 * 功能: 初始化模式2控制所需资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_Init(void);

/*
 * 函数名: FC_Mode2_Reset
 * 功能: 复位模式2控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_Reset(void);

/*
 * 函数名: FC_Mode2_100Hz
 * 功能: 执行模式2的100Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode2_100Hz(void);

/*
 * 函数名: FC_Mode2_Control100Hz
 * 功能: 执行模式2的100Hz图像控制
 * 输入参数:
 *   dt - 本次100Hz调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode2_Control100Hz(float dt);

/*
 * 函数名: FC_Mode2_Get_Fixed_Height_M
 * 功能: 获取模式2固定飞行高度
 * 输入参数: 无
 * 返回值: 固定高度，单位 m
 */
float FC_Mode2_Get_Fixed_Height_M(void);

/*
 * 函数名: FC_Mode3_Init
 * 功能: 初始化模式3控制所需资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode3_Init(void);

/*
 * 函数名: FC_Mode3_Reset
 * 功能: 复位模式3控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode3_Reset(void);

/*
 * 函数名: FC_Mode3_100Hz
 * 功能: 执行模式3的100Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode3_100Hz(void);

/*
 * 函数名: FC_Mode3_50Hz
 * 功能: 执行模式3的50Hz控制
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位 s
 * 返回值: 无
 */
void FC_Mode3_50Hz(float dt);

/* 获取模式3缓慢随机变化的目标高度，单位 m。 */
float FC_Mode3_Get_Target_Height_M(void);

/*
 * 模式7 X/Y 轴速度环 PID 实例。
 * 作用: 供外部模块读取速度环输出和调试状态。
 */
extern pid_t g_mode7_velx_pid;
extern pid_t g_mode7_vely_pid;
extern float g_mode7_velx_target;
extern float g_mode7_vely_target;
/* 模式1图像PD、车加速度前馈与调试状态。 */
extern pid_t g_mode1_imgx_pid;
extern pid_t g_mode1_imgy_pid;
extern float g_mode1_car_accel_angle_ff_x_deg;
extern float g_mode1_car_accel_angle_ff_y_deg;
extern float g_mode1_car_accel_x_mps2;
extern float g_mode1_car_accel_y_mps2;
extern float g_mode1_raw_roll_correction_deg;
extern float g_mode1_raw_pitch_correction_deg;
extern float g_mode1_img_error_rate_x_pxps;
extern float g_mode1_img_error_rate_y_pxps;
extern float g_mode1_car_dt_ms;
extern float g_mode1_car_vel_error_x_mps;
extern float g_mode1_car_vel_error_y_mps;
extern float g_mode1_car_body_accel_x_mps2;
extern float g_mode1_car_body_accel_y_mps2;
extern float g_mode1_car_turn_accel_mps2;
extern float g_mode1_yaw_diff_deg;
extern uint32 g_mode1_control_seq;

/* 模式2图像PD、车加速度前馈与调试状态。 */
extern pid_t g_mode2_imgx_pid;
extern pid_t g_mode2_imgy_pid;
extern float g_mode2_car_accel_angle_ff_x_deg;
extern float g_mode2_car_accel_angle_ff_y_deg;
extern float g_mode2_car_accel_x_mps2;
extern float g_mode2_car_accel_y_mps2;
extern float g_mode2_raw_roll_correction_deg;
extern float g_mode2_raw_pitch_correction_deg;
extern float g_mode2_img_error_rate_x_pxps;
extern float g_mode2_img_error_rate_y_pxps;
extern float g_mode2_car_dt_ms;
extern float g_mode2_car_vel_error_x_mps;
extern float g_mode2_car_vel_error_y_mps;
extern float g_mode2_car_body_accel_x_mps2;
extern float g_mode2_car_body_accel_y_mps2;
extern float g_mode2_car_turn_accel_mps2;
extern float g_mode2_yaw_diff_deg;
extern uint32 g_mode2_control_seq;

/* 模式4图像PD、车加速度前馈与调试状态。 */
extern pid_t g_mode4_imgx_pid;
extern pid_t g_mode4_imgy_pid;
extern float g_mode4_car_accel_angle_ff_x_deg;
extern float g_mode4_car_accel_angle_ff_y_deg;
extern float g_mode4_car_accel_x_mps2;
extern float g_mode4_car_accel_y_mps2;
extern float g_mode4_raw_roll_correction_deg;
extern float g_mode4_raw_pitch_correction_deg;
extern float g_mode4_img_error_rate_x_pxps;
extern float g_mode4_img_error_rate_y_pxps;
extern float g_mode4_car_dt_ms;
extern float g_mode4_car_vel_error_x_mps;
extern float g_mode4_car_vel_error_y_mps;
extern float g_mode4_car_body_accel_x_mps2;
extern float g_mode4_car_body_accel_y_mps2;
extern float g_mode4_car_turn_accel_mps2;
extern float g_mode4_yaw_diff_deg;
extern uint32 g_mode4_control_seq;

/* 模式5图像PD、车加速度前馈与调试状态。 */
extern pid_t g_mode5_imgx_pid;
extern pid_t g_mode5_imgy_pid;
extern float g_mode5_car_accel_angle_ff_x_deg;
extern float g_mode5_car_accel_angle_ff_y_deg;
extern float g_mode5_car_accel_x_mps2;
extern float g_mode5_car_accel_y_mps2;
extern float g_mode5_raw_roll_correction_deg;
extern float g_mode5_raw_pitch_correction_deg;
extern float g_mode5_img_error_rate_x_pxps;
extern float g_mode5_img_error_rate_y_pxps;
extern float g_mode5_car_dt_ms;
extern float g_mode5_car_vel_error_x_mps;
extern float g_mode5_car_vel_error_y_mps;
extern float g_mode5_car_body_accel_x_mps2;
extern float g_mode5_car_body_accel_y_mps2;
extern float g_mode5_car_turn_accel_mps2;
extern float g_mode5_yaw_diff_deg;
extern uint32 g_mode5_control_seq;

/*
 * 模式8 X/Y 轴速度环 PID 实例。
 * 作用: 供外部模块读取图像跟随速度目标和速度环调试状态。
 */
extern pid_t g_mode8_velx_pid;
extern pid_t g_mode8_vely_pid;
extern pid_t g_mode8_imgx_pid;
extern pid_t g_mode8_imgy_pid;
extern float g_mode8_velx_target;
extern float g_mode8_vely_target;

/*
 * 函数名: FC_Mode4_Init
 * 功能: 初始化模式4控制所需资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode4_Init(void);

/*
 * 函数名: FC_Mode4_Reset
 * 功能: 复位模式4控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode4_Reset(void);

/*
 * 函数名: FC_Mode4_100Hz
 * 功能: 执行模式4的100Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode4_100Hz(void);

/*
 * 函数名: FC_Mode4_Control100Hz
 * 功能: 执行模式4的100Hz图像控制
 * 输入参数:
 *   dt - 本次100Hz调用周期，单位s
 * 返回值: 无
 */
void FC_Mode4_Control100Hz(float dt);

/*
 * 函数名: FC_Mode5_Init
 * 功能: 初始化模式5控制所需资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode5_Init(void);

/*
 * 函数名: FC_Mode5_Reset
 * 功能: 复位模式5控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode5_Reset(void);

/*
 * 函数名: FC_Mode5_100Hz
 * 功能: 执行模式5的100Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode5_100Hz(void);

/*
 * 函数名: FC_Mode5_Control100Hz
 * 功能: 执行模式5的100Hz图像控制
 * 输入参数:
 *   dt - 本次100Hz调用周期，单位s
 * 返回值: 无
 */
void FC_Mode5_Control100Hz(float dt);

/*
 * 函数名: FC_Mode5_Get_Fixed_Height_M
 * 功能: 获取模式5使用的固定高度目标
 * 输入参数: 无
 * 返回值:
 *   模式5固定高度目标，单位m
 */
float FC_Mode5_Get_Fixed_Height_M(void);

/*
 * 函数名: FC_Mode6_Init
 * 功能: 初始化模式6控制所需资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode6_Init(void);

/*
 * 函数名: FC_Mode6_Reset
 * 功能: 复位模式6控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode6_Reset(void);

/*
 * 函数名: FC_Mode6_100Hz
 * 功能: 执行模式6的100Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode6_100Hz(void);

/*
 * 函数名: FC_Mode6_50Hz
 * 功能: 执行模式6的50Hz控制
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位s
 * 返回值: 无
 */
void FC_Mode6_50Hz(float dt);

/*
 * 函数名: FC_Mode7_Init
 * 功能: 初始化模式7控制所需资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode7_Init(void);

/*
 * 函数名: FC_Mode7_Reset
 * 功能: 复位模式7控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode7_Reset(void);

/*
 * 函数名: FC_Mode7_100Hz
 * 功能: 执行模式7的100Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode7_100Hz(void);

/*
 * 函数名: FC_Mode7_50Hz
 * 功能: 执行模式7的50Hz控制
 * 输入参数:
 *   dt - 本次50Hz调用周期，单位s
 * 返回值: 无
 */
void FC_Mode7_50Hz(float dt);

/*
 * 函数名: FC_Mode8_Init
 * 功能: 初始化模式8控制所需资源
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode8_Init(void);

/*
 * 函数名: FC_Mode8_Reset
 * 功能: 复位模式8控制状态
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode8_Reset(void);

/*
 * 函数名: FC_Mode8_100Hz
 * 功能: 执行模式8的100Hz控制
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Mode8_100Hz(void);

/*
 * 函数名: FC_Mode8_Control100Hz
 * 功能: 执行模式8的100Hz图像控制
 * 输入参数:
 *   dt - 本次100Hz调用周期，单位s
 * 返回值: 无
 */
void FC_Mode8_Control100Hz(float dt);

#ifdef __cplusplus
}
#endif

#endif /* FC_MODE_H */
