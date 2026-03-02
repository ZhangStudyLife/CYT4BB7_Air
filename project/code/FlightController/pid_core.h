#ifndef PID_CORE_H
#define PID_CORE_H

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kff;
    float dt;

    float i_limit;
    float d_lpf_alpha;

    float integral;
    float prev_meas;
    float prev_sp;
    float d_filtered;
    uint8_t d_initialized;

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

void PID_Init(pid_t *pid, float kp, float ki, float kd, float kff,
              float dt, float i_limit, float d_lpf);

float PID_Update(pid_t *pid, float setpoint, float measurement, float dt);

void PID_Reset(pid_t *pid);

#endif /* PID_CORE_H */
