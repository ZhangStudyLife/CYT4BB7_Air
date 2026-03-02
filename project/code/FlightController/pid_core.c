#include "pid_core.h"

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

static inline float pid_absf(float v)
{
    return (v >= 0.0f) ? v : -v;
}

static inline float pid_safe_dt(float dt)
{
    return (dt > 0.0f) ? dt : 0.001f;
}

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
    pid->d_lpf_alpha = pid_clampf(d_lpf, 0.0f, 1.0f);
    pid->iterm_relax_threshold = 100.0f;

    /* anti-windup 默认关闭，保证历史调用行为不变 */
    pid->output_min = -1000000000.0f;
    pid->output_max = 1000000000.0f;
    pid->aw_gain = 0.0f;
    pid->aw_enable = 0U;

    PID_Reset(pid);
}

float PID_Update(pid_t *pid, float setpoint, float measurement, float dt)
{
    float effective_dt;
    float relax_factor = 1.0f;
    float d_raw;
    float alpha;
    float iterm_relax_threshold;
    float i_candidate;

    if (pid == 0)
    {
        return 0.0f;
    }

    effective_dt = (dt > 0.0f) ? dt : pid_safe_dt(pid->dt);
    pid->dt = effective_dt;

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
    }
    else
    {
        d_raw = -pid->kd * (measurement - pid->prev_meas) / effective_dt;
    }

    alpha = pid_clampf(pid->d_lpf_alpha, 0.0f, 1.0f);
    pid->d_filtered = alpha * d_raw + (1.0f - alpha) * pid->d_filtered;
    pid->d_term = pid->d_filtered;

    pid->ff_term = pid->kff * pid->sp_rate;

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
        if ((0U != sat_high && 0U != push_high) || (0U != sat_low && 0U != push_low))
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

    pid->prev_meas = measurement;
    pid->prev_sp = setpoint;

    return pid->output;
}

void PID_Reset(pid_t *pid)
{
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
