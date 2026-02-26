#include "pid_core.h"

/*
 * 该函数用于把数值限制在指定区间内。
 * 当输入小于下限时直接返回下限，避免参数继续减小。
 * 当输入大于上限时直接返回上限，避免参数继续增大。
 * 在 PID 中它主要用于积分限幅、滤波系数约束等边界保护。
 */
static inline float pid_clampf(float v, float min_v, float max_v)
{
    if (v < min_v)
    {
        return min_v;
    }
    if (v > max_v)
    {
        return max_v;
    }
    return v;
}

/*
 * 该函数返回浮点数的绝对值。
 * 用于把阈值和限幅参数统一按“幅值”处理。
 * 这样即使外部误传了负值，也能得到可预期的控制行为。
 */
static inline float pid_absf(float v)
{
    return (v >= 0.0f) ? v : -v;
}

/*
 * 该函数对控制周期做最小安全保护。
 * 当 dt 非法（小于等于 0）时，回退到 1ms 的默认值。
 * 这样可以避免后续微分计算出现除零或数值爆炸。
 */
static inline float pid_safe_dt(float dt)
{
    return (dt > 0.0f) ? dt : 0.001f;
}

void PID_Init(pid_t *pid, float kp, float ki, float kd, float kff,
              float dt, float i_limit, float d_lpf)
{
    /*
     * 初始化首先做空指针保护，避免错误调用造成非法访问。
     * 随后把外部配置写入 PID 结构体，建立本次控制器参数基线。
     * dt 会做安全化处理，i_limit 会按绝对值使用，d_lpf 会限制在 0~1。
     * 最后清空运行状态和调试量，保证控制器从“干净状态”启动。
     */
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = kff;
    pid->dt = pid_safe_dt(dt);
    pid->i_limit = pid_absf(i_limit);
    pid->d_lpf_alpha = pid_clampf(d_lpf, 0.0f, 1.0f);
    pid->iterm_relax_threshold = 100.0f;

    PID_Reset(pid);
}

float PID_Update(pid_t *pid, float setpoint, float measurement, float dt)
{
    float effective_dt;
    float relax_factor = 1.0f;
    float d_raw;
    float alpha;
    float iterm_relax_threshold;

    /*
     * 更新入口先做空指针保护，确保函数可安全返回。
     * 然后确定本次有效 dt：优先使用外部传入值，无效时回退到内部周期。
     * 这样可以兼容“固定周期调用”和“外部动态周期调用”两种模式。
     */
    if (pid == 0)
    {
        return 0.0f;
    }

    effective_dt = (dt > 0.0f) ? dt : pid_safe_dt(pid->dt);
    pid->dt = effective_dt;

    /*
     * P 项直接基于当前误差计算，响应最快。
     * 误差定义为设定值减测量值，符号方向与控制目标一致。
     * 该项不记忆历史，也不预测未来，主要负责即时纠偏。
     */
    pid->error = setpoint - measurement;
    pid->p_term = pid->kp * pid->error;

    /*
     * 先计算设定值变化率，用于前馈和 I 项放松判断。
     * 当设定变化过快时，积分项若继续全速累积，容易造成超调。
     * 因此这里引入 relax_factor，将积分增量按比例衰减到 0~1。
     * 阈值以下不衰减，阈值以上按反比缩放，可保持操纵感与稳定性平衡。
     */
    pid->sp_rate = (setpoint - pid->prev_sp) / effective_dt;
    iterm_relax_threshold = pid_absf(pid->iterm_relax_threshold);
    if ((iterm_relax_threshold > 0.0f) && (pid_absf(pid->sp_rate) > iterm_relax_threshold))
    {
        relax_factor = iterm_relax_threshold / pid_absf(pid->sp_rate);
        relax_factor = pid_clampf(relax_factor, 0.0f, 1.0f);
    }

    /*
     * I 项按误差与时间累积，并乘以放松系数。
     * 积分能消除稳态误差，但也最容易发生饱和。
     * 所以积分状态会被限制在 ±i_limit 内，确保输出可控。
     */
    pid->integral += pid->ki * pid->error * effective_dt * relax_factor;
    pid->integral = pid_clampf(pid->integral, -pid->i_limit, pid->i_limit);
    pid->i_term = pid->integral;

    /*
     * D 项采用“测量微分”形式，并在首次运行时跳过微分输出。
     * 首次调用时没有有效历史样本，强行微分会产生突变。
     * 进入稳定阶段后，D 项按测量变化率计算并乘以 kd。
     * 负号用于与误差导数方向对应，抑制快速变化带来的过冲。
     */
    if (pid->d_initialized == 0U)
    {
        pid->d_initialized = 1U;
        pid->prev_meas = measurement;
        pid->prev_sp = setpoint;
        d_raw = 0.0f;
    }
    else
    {
        d_raw = -pid->kd * (measurement - pid->prev_meas) / effective_dt;
    }

    /*
     * 原始微分量通常噪声较大，需要一阶低通滤波。
     * alpha 越大越贴近当前 d_raw，响应更快但更敏感。
     * alpha 越小越依赖历史滤波值，响应更平滑但会增加滞后。
     */
    alpha = pid_clampf(pid->d_lpf_alpha, 0.0f, 1.0f);
    pid->d_filtered = alpha * d_raw + (1.0f - alpha) * pid->d_filtered;
    pid->d_term = pid->d_filtered;

    /*
     * FF 项由设定值变化率直接生成，作用是“提前给量”。
     * 它不依赖误差，适合在快速指令变化时减少滞后感。
     * 最终输出由 P、I、D、FF 四部分相加得到。
     * 在返回前更新历史测量与历史设定，供下一周期使用。
     */
    pid->ff_term = pid->kff * pid->sp_rate;

    pid->prev_meas = measurement;
    pid->prev_sp = setpoint;

    pid->output = pid->p_term + pid->i_term + pid->d_term + pid->ff_term;
    return pid->output;
}

void PID_Reset(pid_t *pid)
{
    /*
     * 复位函数只清除运行状态和观测输出。
     * PID 增益、限幅和滤波配置保持不变。
     * 适用于解锁重启控制环、模式切换后软复位等场景。
     */
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_meas = 0.0f;
    pid->prev_sp = 0.0f;
    pid->d_filtered = 0.0f;
    pid->d_initialized = 0U;

    pid->error = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->ff_term = 0.0f;
    pid->output = 0.0f;
    pid->sp_rate = 0.0f;
}
