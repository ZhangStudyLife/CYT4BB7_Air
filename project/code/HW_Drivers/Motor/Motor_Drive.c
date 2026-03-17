/**
 * @file    Motor_Drive.c
 * @brief   四轴飞行器电机驱动与混控实现
 * @note    针对Mark5 Pro非对称机架设计
 *          所有接口使用整数类型（0~10000范围），便于调试
 */

#include "Motor_Drive.h"
#include "zf_common_headfile.h"

/* ======================== 引脚映射配置 ======================== */
/*
 * 电机引脚分配：
 * M1 = P05_0 → TCPWM_CH09_P05_0
 * M2 = P05_1 → TCPWM_CH10_P05_1
 * M3 = P9.0  → TCPWM_CH24_P09_0
 * M4 = P9.1  → TCPWM_CH25_P09_1
 */
static const pwm_channel_enum MOTOR_PWM_CH[MOTOR_NUM] = {
    TCPWM_CH09_P05_0,   /* M1: 右后 */
    TCPWM_CH10_P05_1,   /* M2: 右前 */
    TCPWM_CH24_P09_0,   /* M3: 左后 */
    TCPWM_CH25_P09_1    /* M4: 左前 */
};

/* ======================== 混控矩阵定义（整数版本） ======================== */
/*
 * 混控矩阵：[ROLL, PITCH, YAW]，单位：万分比（基数10000）
 *
 * 方向定义（右手法则）：
 * - Roll正 = 右倾（右侧下沉）→ 左侧电机加速，右侧减速
 * - Pitch正 = 抬头（机头上升）→ 后方电机加速，前方减速
 * - Yaw正 = 逆时针旋转 → CCW电机加速，CW电机减速
 *
 * 电机布局与旋转方向：
 * - M1: 右后, CW  → Roll-, Pitch+, Yaw-
 * - M2: 右前, CCW → Roll-, Pitch-, Yaw+
 * - M3: 左后, CCW → Roll+, Pitch+, Yaw+
 * - M4: 左前, CW  → Roll+, Pitch-, Yaw-
 *
 * 力臂补偿（整数版本）：
 * - Roll方向力臂90mm，Pitch方向力臂69mm
 * - Roll系数 = 10000（基准）
 * - Pitch系数 = 90/69 × 10000 ≈ 13043（补偿较短力臂）
 */
static const int32 MOTOR_MIX_MATRIX[MOTOR_NUM][3] = {
    /* { Roll,                    Pitch,                  Yaw    } */
    { -MOTOR_MIX_ROLL_SCALE_I,  +MOTOR_MIX_PITCH_SCALE_I,  -10000 },  /* M1: 右后CW */
    { -MOTOR_MIX_ROLL_SCALE_I,  -MOTOR_MIX_PITCH_SCALE_I,  +10000 },  /* M2: 右前CCW */
    { +MOTOR_MIX_ROLL_SCALE_I,  +MOTOR_MIX_PITCH_SCALE_I,  +10000 },  /* M3: 左后CCW */
    { +MOTOR_MIX_ROLL_SCALE_I,  -MOTOR_MIX_PITCH_SCALE_I,  -10000 }   /* M4: 左前CW */
};

/* ======================== 全局状态变量 ======================== */
motor_state_t g_motor_state = {0};
motor_mixer_input_t g_motor_cmd = {0};
/* ======================== 内部辅助函数 ======================== */

/**
 * @brief   整数限幅
 * @param   value   输入值
 * @param   min_val 最小值
 * @param   max_val 最大值
 * @return  限幅后的值
 */
static inline int32 clamp_i(int32 value, int32 min_val, int32 max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief   油门值转换为PWM duty值
 * @param   throttle 油门量（0~10000）
 * @return  PWM duty值（4000~6000，受安全限制）
 *
 * @note    映射关系：
 *          - 输入0     → duty=4000 (1000us脉宽)
 *          - 输入5000  → duty=6000 (1500us脉宽，安全上限)
 *          - 输入10000 → 先限幅到5000 → duty=6000 (1500us脉宽)
 */
static inline uint32 throttle_to_duty(int32 throttle)
{
    /* 限幅到安全范围：0 ~ MOTOR_THROTTLE_LIMIT_MAX(5000) */
    throttle = clamp_i(throttle, 0, MOTOR_THROTTLE_LIMIT_MAX);

    /* 线性映射：0~10000 → MOTOR_DUTY_MIN(4000)~MOTOR_DUTY_MAX(8000) */
    /* 公式：duty = DUTY_MIN + throttle × (DUTY_MAX - DUTY_MIN) / 10000 */
    uint32 duty = MOTOR_DUTY_MIN + (uint32)throttle * (MOTOR_DUTY_MAX - MOTOR_DUTY_MIN) / MOTOR_INPUT_MAX;

    /* 二次限幅确保不超过安全限制 */
    if (duty > MOTOR_DUTY_LIMIT)
    {
        duty = MOTOR_DUTY_LIMIT;
    }

    return duty;
}

/**
 * @brief   设置电机PWM输出
 * @param   motor   电机编号
 * @param   duty    PWM duty值
 */
static void motor_set_pwm(motor_index_e motor, uint32 duty)
{
    if (motor >= MOTOR_NUM) return;

    g_motor_state.duty[motor] = duty;
    pwm_set_duty(MOTOR_PWM_CH[motor], duty);
}

/* ======================== 公开接口实现 ======================== */

void Motor_Init(void)
{
    /* 初始化状态 */
    g_motor_state.is_armed = 0;
    for (uint8 i = 0; i < MOTOR_NUM; i++)
    {
        g_motor_state.duty[i] = MOTOR_DUTY_MIN;
        g_motor_state.output[i] = 0;
    }

    /* 初始化4路PWM，频率400Hz，初始duty为最小油门 */
    for (uint8 i = 0; i < MOTOR_NUM; i++)
    {
        pwm_init(MOTOR_PWM_CH[i], MOTOR_PWM_FREQ, MOTOR_DUTY_MIN);
    }

    pwm_set_duty(MOTOR_PWM_CH[MOTOR_1], MOTOR_DUTY_MIN);
    pwm_set_duty(MOTOR_PWM_CH[MOTOR_2], MOTOR_DUTY_MIN);
    pwm_set_duty(MOTOR_PWM_CH[MOTOR_3], MOTOR_DUTY_MIN);
    pwm_set_duty(MOTOR_PWM_CH[MOTOR_4], MOTOR_DUTY_MIN);

    // 电调上电要给低油门信号一段时间
    system_delay_ms(3000);
}

void Motor_SetThrottle(motor_index_e motor, int32 throttle)
{
    if (motor >= MOTOR_NUM) return;

    /* 未解锁时只输出最小油门 */
    if (!g_motor_state.is_armed)
    {
        motor_set_pwm(motor, MOTOR_DUTY_MIN);
        g_motor_state.output[motor] = 0;
        return;
    }

    /* 限幅到有效范围 */
    throttle = clamp_i(throttle, 0, MOTOR_INPUT_MAX);
    g_motor_state.output[motor] = throttle;

    /* 转换为duty并输出 */
    uint32 duty = throttle_to_duty(throttle);
    motor_set_pwm(motor, duty);
}

void Motor_SetThrottleAll(const int32 throttle[MOTOR_NUM])
{
    for (uint8 i = 0; i < MOTOR_NUM; i++)
    {
        Motor_SetThrottle((motor_index_e)i, throttle[i]);
    }
}

void Motor_EmergencyStop(void)
{
    /* 强制锁定并停止所有电机 */
    g_motor_state.is_armed = 0;

    for (uint8 i = 0; i < MOTOR_NUM; i++)
    {
        g_motor_state.output[i] = 0;
        motor_set_pwm((motor_index_e)i, MOTOR_DUTY_MIN);
    }
}

void Motor_Mixer(const motor_mixer_input_t *input)
{
    if (input == NULL) return;

    /* SWD拨码开关没有向下拨 不输出*/
    // SWD -> rc_get_channel(RC_CH_AUX1)
    // 打到高数值为-765 打到低数值为764
    // if (rc_get_channel(RC_CH_AUX1) < 0)
    // {
    //     for (uint8 i = 0; i < MOTOR_NUM; i++)
    //     {
    //         g_motor_state.output[i] = 0;
    //         motor_set_pwm((motor_index_e)i, MOTOR_DUTY_MIN);
    //     }
    //     Motor_EmergencyStop();
    //     return;
    // }

    /* 未解锁时只输出最小油门->0%油门 */
    if (!g_motor_state.is_armed)
    {
        Motor_EmergencyStop();
        return;
    }

    /* 限幅输入值（整数范围） */
    int32 throttle = clamp_i(input->throttle, 0, MOTOR_INPUT_MAX);
    int32 roll     = clamp_i(input->roll,    MOTOR_INPUT_MIN, MOTOR_INPUT_MAX);
    int32 pitch    = clamp_i(input->pitch,   MOTOR_INPUT_MIN, MOTOR_INPUT_MAX);
    int32 yaw      = clamp_i(input->yaw,     MOTOR_INPUT_MIN, MOTOR_INPUT_MAX);

    /* 计算各电机输出 */
    int32 motor_out[MOTOR_NUM];
    int32 max_out = 0;
    int32 min_out = MOTOR_INPUT_MAX;

    for (uint8 i = 0; i < MOTOR_NUM; i++)
    {
        /*
         * 混控公式（整数运算）：
         * output = throttle + roll×mix_roll/10000 + pitch×mix_pitch/10000 + yaw×mix_yaw/10000
         *
         * 各分量先乘后除，避免精度损失
         */
        int32 roll_contrib  = (roll  * MOTOR_MIX_MATRIX[i][0]) / MOTOR_INPUT_MAX;
        int32 pitch_contrib = (pitch * MOTOR_MIX_MATRIX[i][1]) / MOTOR_INPUT_MAX;
        int32 yaw_contrib   = (yaw   * MOTOR_MIX_MATRIX[i][2]) / MOTOR_INPUT_MAX;

        motor_out[i] = throttle + roll_contrib + pitch_contrib + yaw_contrib;

        if (motor_out[i] > max_out) max_out = motor_out[i];
        if (motor_out[i] < min_out) min_out = motor_out[i];
    }

    /* 饱和处理：如果超出范围，整体平移保持姿态控制能力 */
    int32 offset = 0;

    if (max_out > MOTOR_INPUT_MAX)
    {
        /* 上限饱和：整体下移 */
        offset = MOTOR_INPUT_MAX - max_out;
    }
    else if (min_out < MOTOR_IDLE_THROTTLE)
    {
        /* 下限饱和：整体上移，确保不低于怠速 */
        offset = MOTOR_IDLE_THROTTLE - min_out;
    }

    /* 应用偏移并限幅输出（下限为怠速，防止BLDC失速） */
    for (uint8 i = 0; i < MOTOR_NUM; i++)
    {
        motor_out[i] += offset;
        motor_out[i] = clamp_i(motor_out[i], MOTOR_IDLE_THROTTLE, MOTOR_INPUT_MAX);

        g_motor_state.output[i] = motor_out[i];

        /* 转换为duty并输出 */
        uint32 duty = throttle_to_duty(motor_out[i]);
        motor_set_pwm((motor_index_e)i, duty);
    }
}

void Motor_Enable(void)
{
    g_motor_state.is_armed = 1;
}

void Motor_Disable(void)
{
    g_motor_state.is_armed = 0;
    Motor_EmergencyStop();
}

uint8 Motor_IsEnabled(void)
{
    return g_motor_state.is_armed;
}

void Motor_IdleStart(void)
{
    // /* SWD拨码开关没有向下拨 不输出*/
    // // SWD -> rc_get_channel(RC_CH_AUX1)
    // // 打到高数值为-765 打到低数值为764
    // if (rc_get_channel(RC_CH_AUX1) < 0)
    // {
    //     for (uint8 i = 0; i < MOTOR_NUM; i++)
    //     {
    //         g_motor_state.output[i] = 0;
    //         motor_set_pwm((motor_index_e)i, MOTOR_DUTY_MIN);
    //     }
    //     Motor_EmergencyStop();
    //     return;
    // }

    for (uint16_t temp_throttle = 2000; temp_throttle < 3000; temp_throttle += 100)
    {
        Motor_Mixer(&(motor_mixer_input_t){.throttle=temp_throttle, .roll=0, .pitch=0, .yaw=0});
        system_delay_ms(200);
    }
    system_delay_ms(1000);
}
