#ifndef PID_CORE_H
#define PID_CORE_H

#include <stdint.h>

typedef struct
{
    float kp;                     // 比例增益，决定当前误差的直接响应强度
    float ki;                     // 积分增益，决定误差累积补偿速度
    float kd;                     // 微分增益，决定变化趋势的抑制强度
    float kff;                    // 前馈增益，按设定值变化提前输出
    float dt;                     // 控制周期（秒），用于积分和微分计算

    float i_limit;                // 积分限幅绝对值，防止积分饱和
    float d_lpf_alpha;            // 微分低通系数，范围0~1，越大越跟随瞬时微分

    float integral;               // 积分状态累计值（I状态）
    float prev_meas;              // 上一次测量值，用于计算测量微分
    float prev_sp;                // 上一次设定值，用于计算设定变化率
    float d_filtered;             // 微分低通滤波后的状态值
    uint8_t d_initialized;        // 微分状态是否已初始化（0未初始化，1已初始化）

    float error;                  // 当前误差（setpoint - measurement）
    float p_term;                 // 比例项输出
    float i_term;                 // 积分项输出
    float d_term;                 // 微分项输出
    float ff_term;                // 前馈项输出
    float output;                 // 总输出（P+I+D+FF）
    float sp_rate;                // 设定值变化率（每秒）
    float iterm_relax_threshold;  // I项放松阈值，超过后降低积分累积速度
} pid_t;

/*
 * 初始化 PID 参数与内部状态。
 * 该函数会写入增益、周期和限幅参数，并清空运行时状态。
 * 其中 dt 会做安全保护，i_limit 会取绝对值，d_lpf 会被约束到 0~1。
 */
void PID_Init(pid_t *pid, float kp, float ki, float kd, float kff,
              float dt, float i_limit, float d_lpf);

/*
 * 执行一次 PID 更新并返回控制输出。
 * 支持外部传入 dt；当 dt 无效时会回退到结构体中的周期参数。
 * 计算流程包含 P/I/D/FF、I项放松与积分限幅、微分低通滤波。
 */
float PID_Update(pid_t *pid, float setpoint, float measurement, float dt);

/*
 * 清空 PID 运行时状态和调试输出项。
 * 不修改 kp/ki/kd/kff 等配置参数。
 */
void PID_Reset(pid_t *pid);

#endif /* PID_CORE_H */
