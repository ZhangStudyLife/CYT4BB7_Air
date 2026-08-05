#ifndef PID_CORE_H
#define PID_CORE_H

#include <stdint.h>

/* PID D 项二阶滤波器状态 */
typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float d1;
    float d2;
} pid_biquad_t;

/* PID PT3 低通滤波器状态 */
typedef struct
{
    float k;
    float state1;
    float state2;
    float state3;
} pid_pt3_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kff;
    float dt;

    float i_limit;
    float ff_smoothing_ms; /* 前馈 PT3 平滑时间，单位 ms，0 表示旁路 */
    float output_lpf_hz;   /* 输出 PT3 低通截止频率，单位 Hz，0 表示旁路 */
    float d_lpf_hz; /* D 项低通截止频率，单位 Hz，0 表示旁路 */

    float integral;
    float prev_meas;
    float prev_sp;
    uint8_t d_initialized;
    pid_biquad_t d_lpf_filter; /* D 项滤波器系数与状态 */

    pid_pt3_t ff_pt3_filter;     /* 前馈 PT3 滤波器状态 */
    pid_pt3_t output_pt3_filter; /* 输出 PT3 滤波器状态 */

    float error;
    float p_term;
    float i_term;
    float d_term;
    float ff_term;
    float output;
    float sp_rate;
    float iterm_relax_threshold;

    /* anti-windup 配置：默认关闭，不影响未启用的 PID */
    float output_min;
    float output_max;
    float aw_gain;
    uint8_t aw_enable;
} pid_t;

/**
 * 函数功能: 初始化 PID 控制器参数和 D 项滤波器。
 * 输入参数:
 *   pid     - PID 控制器实例指针。
 *   kp      - 比例增益。
 *   ki      - 积分增益。
 *   kd      - 微分增益。
 *   kff     - 前馈增益。
 *   dt      - 控制周期，单位 s。
 *   i_limit - 积分限幅绝对值。
 *   d_lpf   - D 项低通截止频率，单位 Hz，0 表示旁路。
 * 返回值: 无。
 */
void PID_Init(pid_t *pid, float kp, float ki, float kd, float kff,
              float dt, float i_limit, float d_lpf);

/**
 * 函数功能: 配置 PID 前馈和输出 PT3 滤波。
 * 输入参数:
 *   pid             - PID 控制器实例指针。
 *   ff_smoothing_ms - 前馈平滑时间，单位 ms，0 表示旁路。
 *   output_lpf_hz   - 输出低通截止频率，单位 Hz，0 表示旁路。
 * 返回值: 无。
 */
void PID_SetFeedforwardFilter(pid_t *pid, float ff_smoothing_ms, float output_lpf_hz);

/**
 * 函数功能: 使用当前设定值和测量值更新 PID 输出。
 * 输入参数:
 *   pid         - PID 控制器实例指针。
 *   setpoint    - 目标值。
 *   measurement - 测量值。
 *   dt          - 本次更新周期，单位 s；若非正值则沿用上次有效周期。
 * 返回值:
 *   PID 当前输出值。
 */
float PID_Update(pid_t *pid, float setpoint, float measurement, float dt);

/**
 * 函数功能: 清零 PID 积分项、D 项状态和调试输出。
 * 输入参数:
 *   pid - PID 控制器实例指针。
 * 返回值: 无。
 */
void PID_Reset(pid_t *pid);

#endif /* PID_CORE_H */
