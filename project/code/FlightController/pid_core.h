#ifndef PID_CORE_H
#define PID_CORE_H

/* ==================== PID控制器结构体 ==================== */
typedef struct {
    /* 增益参数 */
    float kp;
    float ki;
    float kd;
    float kff;              /* 前馈增益（0=禁用） */
    float dt;

    /* 限幅和滤波 */
    float i_limit;          /* 积分限幅 */
    float d_lpf_alpha;      /* D项低通滤波系数（0~1，越大滤波越弱） */

    /* 中间变量（内部使用） */
    float integral;         /* 积分累积 */
    float prev_meas;        /* 上次测量值（微分先行用） */
    float prev_sp;          /* 上次setpoint（前馈用） */
    float d_filtered;       /* 滤波后的D项 */

    /* 中间量 用于debug*/
    float error;
    float p_term;
    float i_term;
    float d_term;
    float ff_term;
    float output;
    float sp_rate;          /* setpoint变化率 */

} pid_t;

/* ==================== 三个核心API ==================== */

/**
 * 初始化PID（一次性配置所有参数）
 * @param pid       PID结构体指针
 * @param kp        比例增益
 * @param ki        积分增益
 * @param kd        微分增益
 * @param kff       前馈增益（0=禁用前馈）
 * @param dt        采样周期（秒）
 * @param i_limit   积分限幅
 * @param d_lpf     D项低通滤波系数（0~1）
 */
void PID_Init(pid_t *pid, float kp, float ki, float kd, float kff,
              float dt, float i_limit, float d_lpf);

/**
 * PID计算（BetaFlight风格）
 * - 微分先行：D项作用于测量值
 * - D项低通滤波：抑制高频噪声
 * - I-term Relax：setpoint变化快时减少积分
 * - 前馈：对setpoint变化率补偿
 *
 * @param pid         PID结构体指针
 * @param setpoint    目标值
 * @param measurement 测量值
 * @return            控制输出
 */
float PID_Update(pid_t *pid, float setpoint, float measurement);

/**
 * 清除中间变量
 * @param pid  PID结构体指针
 */
void PID_Reset(pid_t *pid);

#endif /* PID_CORE_H */
