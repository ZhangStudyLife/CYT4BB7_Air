/**
 * @file    Motor_Drive.h
 * @brief   四轴飞行器电机驱动与混控模块
 * @note    针对Mark5 Pro非对称机架设计，Quad X布局
 *          包含PWM输出、安全限制和力臂补偿混控
 *          所有接口使用整数类型（0~10000范围），便于调试
 */

#ifndef MOTOR_DRIVE_H_
#define MOTOR_DRIVE_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 输入范围定义 ======================== */
#define MOTOR_INPUT_MAX         (10000)     /* 油门/控制量最大值 */
#define MOTOR_INPUT_MIN         (-10000)    /* 控制量最小值（仅roll/pitch/yaw） */

/* ======================== 机架几何参数（Mark5 Pro） ======================== */
#define MOTOR_ARM_PITCH_MM      (69)        /* 前后方向力臂：138mm/2 */
#define MOTOR_ARM_ROLL_MM       (90)        /* 左右方向力臂：180mm/2 */

/*
 * 力臂补偿系数说明（整数版本，基数10000）：
 * - 力矩 = 力 × 力臂，力臂短则需要更大的力产生相同力矩
 * - Roll力臂(90mm)长于Pitch力臂(69mm)，所以Pitch方向需要更大增益
 * - Roll系数 = 10000（基准）
 * - Pitch系数 = 90/69 × 10000 ≈ 13043（补偿较短力臂）
 */
#define MOTOR_MIX_ROLL_SCALE_I  (10000)                                         /* Roll基准 */
#define MOTOR_MIX_PITCH_SCALE_I ((MOTOR_ARM_ROLL_MM * 10000) / MOTOR_ARM_PITCH_MM) /* ≈13043 */

/* ======================== PWM参数配置 ======================== */
#define MOTOR_PWM_FREQ          (400U)      /* PWM频率：400Hz */
#define MOTOR_PWM_PERIOD_US     (2500U)     /* PWM周期：2500us */
#define MOTOR_PWM_DUTY_MAX      (10000U)    /* PWM占空比最大值（驱动层定义） */

/* 油门脉宽范围（微秒） */
#define MOTOR_PULSE_MIN_US      (1000U)     /* 最小油门脉宽：电机停止 */
#define MOTOR_PULSE_MAX_US      (2000U)     /* 最大油门脉宽：全速 */

/* 脉宽转占空比（基于2500us周期） */
#define MOTOR_DUTY_MIN          (4000U)     /* 1000us → 40% → duty=4000 */
#define MOTOR_DUTY_MAX          (8000U)     /* 2000us → 80% → duty=8000 */

/* ======================== 安全限制 ======================== */
#define MOTOR_THROTTLE_LIMIT_MAX    (6000)      /* 最大油门限制：60% = 6000/10000 */
#define MOTOR_THROTTLE_LIMIT_MIN    (0)         /* 最小油门限制：0% = 0/10000 */
#define MOTOR_DUTY_LIMIT        (MOTOR_DUTY_MIN + ((MOTOR_DUTY_MAX - MOTOR_DUTY_MIN) * MOTOR_THROTTLE_LIMIT_MAX / MOTOR_INPUT_MAX)) /* 最大duty限制 */

/* 混控模式下的最低怠速（防止BLDC六步换相失速） */
#define MOTOR_IDLE_THROTTLE     (2000)      /* 最低怠速：20% = 2000/10000，约1200us脉宽 */

/* ======================== 电机编号定义（Quad X布局） ======================== */
/*
 * 机架俯视图（机头朝上/前）：
 *
 *        前(机头)
 *     M4(CW)   M2(CCW)
 *         \   /
 *          \ /
 *           X
 *          / \
 *         /   \
 *     M3(CCW)  M1(CW)
 *        后
 *
 * 电机旋转方向：CW=顺时针, CCW=逆时针
 */
typedef enum
{
    MOTOR_1 = 0,    /* 右后，CW */
    MOTOR_2,        /* 右前，CCW */
    MOTOR_3,        /* 左后，CCW */
    MOTOR_4,        /* 左前，CW */
    MOTOR_NUM
} motor_index_e;

/* ======================== 混控输入结构体 ======================== */
typedef struct
{
    int32 throttle;     /* 油门量：0~10000 */
    int32 roll;         /* 横滚控制：-10000~10000，正值=右倾 */
    int32 pitch;        /* 俯仰控制：-10000~10000，正值=抬头 */
    int32 yaw;          /* 偏航控制：-10000~10000，正值=逆时针 */
} motor_mixer_input_t;

/* ======================== 电机状态结构体 ======================== */
typedef struct
{
    uint8   is_armed;               /* 解锁状态：0=锁定，1=解锁 */
    uint32  duty[MOTOR_NUM];        /* 各电机当前duty值（0~10000） */
    int32   output[MOTOR_NUM];      /* 各电机混控后输出（0~10000） */
} motor_state_t;

/* ======================== 全局状态变量 ======================== */
extern motor_state_t g_motor_state;

/* ======================== 函数接口 ======================== */

/**
 * @brief   电机驱动模块初始化
 * @note    初始化4路PWM输出，设置为最小油门
 */
void Motor_Init(void);

/**
 * @brief   设置单个电机油门
 * @param   motor   电机编号（MOTOR_1~MOTOR_4）
 * @param   throttle 油门量（0~10000），超过安全限制自动截断
 */
void Motor_SetThrottle(motor_index_e motor, int32 throttle);

/**
 * @brief   同时设置4路电机油门
 * @param   throttle 油门数组，长度为MOTOR_NUM，范围0~10000
 */
void Motor_SetThrottleAll(const int32 throttle[MOTOR_NUM]);

/**
 * @brief   紧急停止所有电机
 * @note    立即将所有电机设为最小油门
 */
void Motor_EmergencyStop(void);

/**
 * @brief   电机混控计算与输出
 * @param   input   混控输入（油门0~10000、roll/pitch/yaw为-10000~10000）
 * @note    内部进行力臂补偿、安全限制和饱和处理
 */
void Motor_Mixer(const motor_mixer_input_t *input);

/**
 * @brief   解锁电机
 * @note    允许电机响应油门指令
 */
void Motor_Enable(void);

/**
 * @brief   锁定电机
 * @note    电机停止响应油门指令，输出最小油门
 */
void Motor_Disable(void);

/**
 * @brief   获取电机解锁状态
 * @return  1=已解锁，0=已锁定
 */
uint8 Motor_IsEnabled(void);

/**
 * @brief   开机前怠速启动电机
 * @note    用于启动阶段缓慢提升电机转速，帮助姿态收敛
 */
void Motor_IdleStart(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVE_H_ */
