#include "pid_core.h"

#include <math.h>

#define PID_FILTER_PI             (3.14159265359f)
#define PID_BUTTERWORTH_Q         (0.70710678f)
#define PID_LPF_MIN_HZ            (0.0f)
#define PID_LPF_DT_REINIT_RATIO   (0.25f) /* dt变化超过25%时才重建D项滤波器 */
#define PID_PT3_MIN_HZ            (0.0f) /* PT3 截止频率旁路阈值，单位 Hz */
#define PID_PT3_MIN_MS            (0.0f) /* PT3 平滑时间旁路阈值，单位 ms */

/**
 * 函数功能: 将浮点值限制在指定范围内。
 * 输入参数:
 *   v     - 待限制值。
 *   min_v - 下限。
 *   max_v - 上限。
 * 返回值:
 *   限幅后的值。
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

/**
 * 函数功能: 计算浮点绝对值。
 * 输入参数:
 *   v - 输入值。
 * 返回值:
 *   绝对值结果。
 */
static inline float pid_absf(float v)
{
    return (v >= 0.0f) ? v : -v;
}

/**
 * 函数功能: 兜底控制周期，防止出现非正 dt。
 * 输入参数:
 *   dt - 外部传入控制周期，单位 s。
 * 返回值:
 *   大于 0 的有效控制周期，单位 s。
 */
static inline float pid_safe_dt(float dt)
{
    return (dt > 0.0f) ? dt : 0.001f;
}

/**
 * 函数功能: 清零二阶滤波器内部状态。
 * 输入参数:
 *   filt - 目标滤波器状态。
 * 返回值: 无。
 */
static void pid_biquad_reset(pid_biquad_t *filt)
{
    if (filt == 0)
    {
        return;
    }

    filt->d1 = 0.0f;
    filt->d2 = 0.0f;
}

/**
 * 函数功能: 清零 PT3 滤波器状态。
 * 输入参数:
 *   filt - 目标 PT3 滤波器。
 * 返回值: 无。
 */
static void pid_pt3_reset(pid_pt3_t *filt)
{
    if (filt == 0)
    {
        return;
    }

    filt->state1 = 0.0f;
    filt->state2 = 0.0f;
    filt->state3 = 0.0f;
}

/**
 * 函数功能: 配置 PT3 低通滤波器系数。
 * 输入参数:
 *   filt - 目标 PT3 滤波器。
 *   dt   - 控制周期，单位 s。
 *   hz   - 截止频率，单位 Hz。
 * 返回值: 无。
 */
static void pid_pt3_init_lpf(pid_pt3_t *filt, float dt, float hz)
{
    float rc;
    float safe_dt;

    if (filt == 0)
    {
        return;
    }

    if (hz <= PID_PT3_MIN_HZ)
    {
        filt->k = 1.0f;
        pid_pt3_reset(filt);
        return;
    }

    safe_dt = pid_safe_dt(dt);
    rc = 1.0f / (2.0f * PID_FILTER_PI * hz);
    filt->k = safe_dt / (safe_dt + rc);
    filt->k = pid_clampf(filt->k, 0.0f, 1.0f);
    pid_pt3_reset(filt);
}

/**
 * 函数功能: 执行一次 PT3 低通滤波。
 * 输入参数:
 *   filt - 目标 PT3 滤波器。
 *   in   - 当前输入样本。
 * 返回值:
 *   本次滤波输出。
 */
static float pid_pt3_apply(pid_pt3_t *filt, float in)
{
    float k;

    if (filt == 0)
    {
        return in;
    }

    k = filt->k;
    filt->state1 += k * (in - filt->state1);
    filt->state2 += k * (filt->state1 - filt->state2);
    filt->state3 += k * (filt->state2 - filt->state3);
    return filt->state3;
}

/**
 * 函数功能: 初始化二阶 Butterworth 低通滤波器。
 * 输入参数:
 *   filt - 目标滤波器状态。
 *   fs   - 采样频率，单位 Hz。
 *   fc   - 截止频率，单位 Hz。
 * 返回值: 无。
 */
static void pid_biquad_init_lpf(pid_biquad_t *filt, float fs, float fc)
{
    float w0;
    float sw0;
    float cw0;
    float alpha;
    float a0;
    float limited_fc;
    float max_fc;

    if ((filt == 0) || (fs <= 0.0f) || (fc <= PID_LPF_MIN_HZ))
    {
        return;
    }

    max_fc = 0.49f * fs;
    limited_fc = pid_clampf(fc, PID_LPF_MIN_HZ, max_fc);
    w0 = 2.0f * PID_FILTER_PI * limited_fc / fs;
    sw0 = sinf(w0);
    cw0 = cosf(w0);
    alpha = sw0 / (2.0f * PID_BUTTERWORTH_Q);
    a0 = 1.0f + alpha;

    filt->b0 = (1.0f - cw0) * 0.5f / a0;
    filt->b1 = (1.0f - cw0) / a0;
    filt->b2 = (1.0f - cw0) * 0.5f / a0;
    filt->a1 = (-2.0f * cw0) / a0;
    filt->a2 = (1.0f - alpha) / a0;
    pid_biquad_reset(filt);
}

/**
 * 函数功能: 执行一次二阶 IIR 低通滤波。
 * 输入参数:
 *   filt - 目标滤波器状态。
 *   in   - 当前输入样本。
 * 返回值:
 *   本次滤波输出。
 */
static float pid_biquad_apply(pid_biquad_t *filt, float in)
{
    float out;

    if (filt == 0)
    {
        return in;
    }

    out = filt->b0 * in + filt->d1;
    filt->d1 = filt->b1 * in - filt->a1 * out + filt->d2;
    filt->d2 = filt->b2 * in - filt->a2 * out;
    return out;
}

/**
 * 函数功能: 按当前周期重建 D 项低通滤波器。
 * 输入参数:
 *   pid - PID 控制器实例指针。
 * 返回值: 无。
 */
static void pid_refresh_dterm_filter(pid_t *pid)
{
    float fs;

    if ((pid == 0) || (pid->d_lpf_hz <= PID_LPF_MIN_HZ))
    {
        return;
    }

    fs = 1.0f / pid_safe_dt(pid->dt);
    pid_biquad_init_lpf(&pid->d_lpf_filter, fs, pid->d_lpf_hz);
}

/**
 * 函数功能: 按当前周期重建 PT3 滤波器。
 * 输入参数:
 *   pid - PID 控制器实例指针。
 * 返回值: 无。
 */
static void pid_refresh_pt3_filters(pid_t *pid)
{
    float ff_cutoff_hz;

    if (pid == 0)
    {
        return;
    }

    if (pid->ff_smoothing_ms > PID_PT3_MIN_MS)
    {
        ff_cutoff_hz = 1000.0f / (2.0f * PID_FILTER_PI * pid->ff_smoothing_ms);
        pid_pt3_init_lpf(&pid->ff_pt3_filter, pid->dt, ff_cutoff_hz);
    }
    else
    {
        pid_pt3_init_lpf(&pid->ff_pt3_filter, pid->dt, 0.0f);
    }

    if (pid->output_lpf_hz > PID_PT3_MIN_HZ)
    {
        pid_pt3_init_lpf(&pid->output_pt3_filter, pid->dt, pid->output_lpf_hz);
    }
    else
    {
        pid_pt3_init_lpf(&pid->output_pt3_filter, pid->dt, 0.0f);
    }
}

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
              float dt, float i_limit, float d_lpf)
{
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
    pid->d_lpf_hz = (d_lpf > PID_LPF_MIN_HZ) ? d_lpf : 0.0f;
    pid->ff_smoothing_ms = 0.0f;
    pid->output_lpf_hz = 0.0f;
    pid->iterm_relax_threshold = 100.0f;

    /* anti-windup 默认关闭，保证历史调用行为不变 */
    pid->output_min = -1000000000.0f;
    pid->output_max = 1000000000.0f;
    pid->aw_gain = 0.0f;
    pid->aw_enable = 0U;

    pid_refresh_dterm_filter(pid);
    pid_refresh_pt3_filters(pid);
    PID_Reset(pid);
}

/**
 * 函数功能: 配置 PID 前馈和输出 PT3 滤波。
 * 输入参数:
 *   pid             - PID 控制器实例指针。
 *   ff_smoothing_ms - 前馈平滑时间，单位 ms，0 表示旁路。
 *   output_lpf_hz   - 输出低通截止频率，单位 Hz，0 表示旁路。
 * 返回值: 无。
 */
void PID_SetFeedforwardFilter(pid_t *pid, float ff_smoothing_ms, float output_lpf_hz)
{
    if (pid == 0)
    {
        return;
    }

    pid->ff_smoothing_ms = (ff_smoothing_ms > PID_PT3_MIN_MS) ? ff_smoothing_ms : 0.0f;
    pid->output_lpf_hz = (output_lpf_hz > PID_PT3_MIN_HZ) ? output_lpf_hz : 0.0f;
    pid_refresh_pt3_filters(pid);
}

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
float PID_Update(pid_t *pid, float setpoint, float measurement, float dt)
{
    float effective_dt;
    float previous_dt;
    float relax_factor = 1.0f;
    float d_raw;
    float iterm_relax_threshold;
    float i_candidate;

    if (pid == 0)
    {
        return 0.0f;
    }

    previous_dt = pid_safe_dt(pid->dt);
    effective_dt = (dt > 0.0f) ? dt : previous_dt;
    pid->dt = effective_dt;

    if ((pid->d_lpf_hz > PID_LPF_MIN_HZ) &&
        (pid_absf(effective_dt - previous_dt) > (previous_dt * PID_LPF_DT_REINIT_RATIO)))
    {
        pid_refresh_dterm_filter(pid);
        pid->d_initialized = 0U;
    }

    pid->error = setpoint - measurement;
    pid->p_term = pid->kp * pid->error;

    pid->sp_rate = (setpoint - pid->prev_sp) / effective_dt;
    iterm_relax_threshold = pid_absf(pid->iterm_relax_threshold);
    if ((iterm_relax_threshold > 0.0f) && (pid_absf(pid->sp_rate) > iterm_relax_threshold))
    {
        relax_factor = iterm_relax_threshold / pid_absf(pid->sp_rate);
        relax_factor = pid_clampf(relax_factor, 0.0f, 1.0f);
    }

    if (pid->d_initialized == 0U)
    {
        pid->d_initialized = 1U;
        pid->prev_meas = measurement;
        pid->prev_sp = setpoint;
        d_raw = 0.0f;
        pid_biquad_reset(&pid->d_lpf_filter);
    }
    else
    {
        d_raw = -pid->kd * (measurement - pid->prev_meas) / effective_dt;
    }

    if (pid->d_lpf_hz > PID_LPF_MIN_HZ)
    {
        pid->d_term = pid_biquad_apply(&pid->d_lpf_filter, d_raw);
    }
    else
    {
        pid->d_term = d_raw;
    }

    pid->ff_term = pid->kff * pid->sp_rate;
    if (pid->ff_smoothing_ms > PID_PT3_MIN_MS)
    {
        pid->ff_term = pid_pt3_apply(&pid->ff_pt3_filter, pid->ff_term);
    }
    i_candidate = pid->integral + pid->ki * pid->error * effective_dt * relax_factor;

    if (pid->aw_enable != 0U)
    {
        float out_min = pid->output_min;
        float out_max = pid->output_max;
        float out_unsat;
        float out_sat;
        uint8_t sat_high;
        uint8_t sat_low;
        uint8_t push_high;
        uint8_t push_low;

        if (out_min > out_max)
        {
            float tmp = out_min;
            out_min = out_max;
            out_max = tmp;
        }

        out_unsat = pid->p_term + i_candidate + pid->d_term + pid->ff_term;
        out_sat = pid_clampf(out_unsat, out_min, out_max);

        sat_high = (out_unsat > out_max) ? 1U : 0U;
        sat_low = (out_unsat < out_min) ? 1U : 0U;
        push_high = (pid->error > 0.0f) ? 1U : 0U;
        push_low = (pid->error < 0.0f) ? 1U : 0U;

        /* 饱和且误差继续同向推动时，冻结本周期积分增量 */
        if (((0U != sat_high) && (0U != push_high)) || ((0U != sat_low) && (0U != push_low)))
        {
            i_candidate = pid->integral;
            out_unsat = pid->p_term + i_candidate + pid->d_term + pid->ff_term;
            out_sat = pid_clampf(out_unsat, out_min, out_max);
        }

        /* 回算 anti-windup: I += Kaw * (u_sat - u_unsat) * dt */
        if (pid->aw_gain > 0.0f)
        {
            i_candidate += pid->aw_gain * (out_sat - out_unsat) * effective_dt;
        }

        pid->integral = pid_clampf(i_candidate, -pid->i_limit, pid->i_limit);
        pid->i_term = pid->integral;
        pid->output = pid->p_term + pid->i_term + pid->d_term + pid->ff_term;
        pid->output = pid_clampf(pid->output, out_min, out_max);
    }
    else
    {
        pid->integral = pid_clampf(i_candidate, -pid->i_limit, pid->i_limit);
        pid->i_term = pid->integral;
        pid->output = pid->p_term + pid->i_term + pid->d_term + pid->ff_term;
    }

    if (pid->output_lpf_hz > PID_PT3_MIN_HZ)
    {
        pid->output = pid_pt3_apply(&pid->output_pt3_filter, pid->output);
        if (pid->aw_enable != 0U)
        {
            float out_min = pid->output_min;
            float out_max = pid->output_max;

            if (out_min > out_max)
            {
                float tmp = out_min;
                out_min = out_max;
                out_max = tmp;
            }
            pid->output = pid_clampf(pid->output, out_min, out_max);
        }
    }

    pid->prev_meas = measurement;
    pid->prev_sp = setpoint;

    return pid->output;
}

/**
 * 函数功能: 清零 PID 积分项、D 项状态和调试输出。
 * 输入参数:
 *   pid - PID 控制器实例指针。
 * 返回值: 无。
 */
void PID_Reset(pid_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_meas = 0.0f;
    pid->prev_sp = 0.0f;
    pid->d_initialized = 0U;
    pid_biquad_reset(&pid->d_lpf_filter);
    pid_pt3_reset(&pid->ff_pt3_filter);
    pid_pt3_reset(&pid->output_pt3_filter);

    pid->error = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->ff_term = 0.0f;
    pid->output = 0.0f;
    pid->sp_rate = 0.0f;
}
