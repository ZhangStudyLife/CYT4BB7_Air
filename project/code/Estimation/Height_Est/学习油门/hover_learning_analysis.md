# 基础油门学习离线分析

## 数据口径

- 日志：`无学习油门,长时间飞行对照组.csv`，共 246714 行，飞行段 233917 行，约 297.114 s。
- 列映射沿用当前 25 路 JustFloat：`I14` 融合竖直速度，`I15` 融合高度，`I16` 位置环输出，`I17/I18/I19` 速度环 P/I/D，`I20` 速度环总输出，`I24` 总油门。
- 本日志飞行段 `I15` 和 `I10..I13` 已经是 mm 量级，目标高度按 1000 mm 计算。
- 离线回放只能评估“学习器会学到什么、是否容易吸收错误信号”，不能等价为开启学习后的真实闭环轨迹。

## 长航时对照组现象

- 飞行段高度绝对误差：mean 54.039 mm，P95 184.361 mm。
- `|vz|`：mean 0.061 m/s，P95 0.182 m/s。
- 速度环 I 项：mean 55.812 PWM，P50 10.748 PWM，P95 290.510 PWM，max 354.531 PWM。
- 速度环总输出：mean 53.748 PWM，P50 19.590 PWM，P95 342.466 PWM。
- 总油门：mean 3204.962，P95 3352.000，饱和/贴边比例 0.000%。
- 由 `throttle - height_vel_out` 反推的当前固定基础油门：宽门控 median 3228.218，严格门控 median 3266.389。离线学习表使用严格门控中位数作为起始基础油门。
- 加速度模长：mean 1.006 g，P95 1.394 g；TOF 四路 spread：P95 145.382 mm，P99 668.918 mm。
- 当前宽门控可学习样本 137904 行，占飞行段 58.954%；严格门控样本 32806 行，占 14.025%；明显倾角样本 39992 行。

## 开源飞控做法

1. ArduPilot 使用 `MOT_THST_HOVER` 表示悬停所需归一化推力，`MOT_HOVER_LEARN` 控制是否学习/保存。源码中的 `update_throttle_hover()` 本质是 10 s 时间常数的低通平均：让 hover throttle 缓慢靠近当前 throttle，并限制在合理范围内。源码链接：https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_MotorsMulticopter.cpp
2. PX4 使用 `MPC_THR_HOVER` 初始化 hover thrust estimator。估计器根据当前分配后的垂直推力和竖直加速度做 EKF 更新，并带有创新门限、噪声自适应、速度过大时降低敏感度、估计范围限制。源码链接：https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_hover_thrust_estimator
3. 两者共同点不是“让 PID 自己消失”，而是只吸收慢变化的悬停偏置；快响应仍由高度/速度闭环完成。

## 离线学习算法对比

| 算法 | 学习源 | 更新时长 | 风险更新比例 | 起始基础油门 | 末端基础油门 | 学习量 | 每 V 学习量 | 残余输出P95 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| raw_throttle_tau10 | raw_throttle | 175.082 s | 76.211% | 3266.389 | 3331.663 | 65.275 | 81.593 | 346.233 |
| base_from_throttle_minus_pid_tau10 | throttle | 175.082 s | 76.211% | 3266.389 | 3327.184 | 60.795 | 75.994 | 358.844 |
| velout_current_tau6 | vel_out | 175.082 s | 76.211% | 3266.389 | 3267.703 | 1.314 | 1.642 | 341.356 |
| velout_strict_tau15 | vel_out | 41.989 s | 0.000% | 3266.389 | 3273.853 | 7.464 | 9.330 | 343.440 |
| ki_strict_tau20_ratelimit | vel_i | 41.989 s | 0.000% | 3266.389 | 3274.840 | 8.451 | 10.564 | 286.713 |
| ki_strict_tau30_slow | vel_i | 41.989 s | 0.000% | 3266.389 | 3274.409 | 8.020 | 10.025 | 287.711 |
| voltage_linear_165pwm_per_v | voltage | 297.007 s | 85.975% | 3266.389 | 3389.503 | 123.114 | 153.893 | 344.261 |

## 结论

1. 这条长航时日志里，总油门从中段约 3130 上升到后段约 3280，再到尾段约 3096；速度环 I 项在不同阶段有正有负，说明不能只按全局均值判断基础油门是否不足，必须在稳态门控内学习。
2. 不建议直接用“所有飞行时刻的总油门”学习。总油门包含高度误差、速度误差、D 项、姿态补偿和 TOF 抖动导致的快动作，门控不严会把控制器动作学进基础油门。
3. 当前代码的 `hover += alpha * height_vel_out` 方向是对的，但门控还应加入姿态、加速度模长、TOF spread、油门饱和判断，并把时间常数从 6 s 放慢到 15..25 s。
4. 对你现在的系统，更推荐先用 KI-only 或“I 项优先、总输出兜底”的学习：基础油门只吸收速度环 I 项的慢偏置，P/D/总输出仍负责闭环响应。这样最不影响后续调 Kp/Ki。
5. 本日志上推荐的保守离线结果是 `ki_strict_tau20_ratelimit`：末端基础油门约 3274.840，相对本次反推基础油门学习 8.451 PWM，折算约 10.564 PWM/V。这个量级适合作为在线学习上限参考，而不是一次性写死标定值。

## 推荐在线方案

```c
if (flying && tof_health_good && fabsf(vz) < 0.12f && fabsf(height_pos_out) < 0.06f &&
    fabsf(roll) < 4.5f && fabsf(pitch) < 4.5f && fabsf(acc_norm_g - 1.0f) < 0.08f &&
    tof_spread_mm < 120.0f && throttle_not_saturated) {
    float residual_i = height_vel_pid.i_term - (hover_throttle - hover_init);
    float alpha = dt / (20.0f + dt);
    float step = clamp(alpha * residual_i, -60.0f * dt, 60.0f * dt);
    hover_throttle = clamp(hover_throttle + step, hover_init - 200.0f, hover_init + 300.0f);
}
```

- 学习目标：让速度环 I 项在长时间悬停后回到接近 0，而不是让 `height_vel_out` 变成 0。
- 防过拟合：每次飞行只在线学习 RAM 值；只有连续多次飞行学到相近结果，再更新持久化默认值。
- 防抵消 PID：学习只在稳态门控内发生，速率限制小于正常 P/D 动作，且学习量限幅在基础油门附近。
- 防学习不足：如果飞行末段 KI 仍持续偏正，可把上限从 `+250` 放到 `+350`，但不要先放宽门控。
