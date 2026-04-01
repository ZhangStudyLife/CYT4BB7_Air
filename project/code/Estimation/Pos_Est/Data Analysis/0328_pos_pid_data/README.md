# 0328 Pos PID Data 说明

本目录保存 `mode2` 室内飞行时采集的速度环日志，当前文件均为 12 列 CSV。

说明：

- 文件名里的前六位时间戳采用 `MMDDHHMM` 风格。
- `velx` 表示 X 轴速度环日志，`vely` 表示 Y 轴速度环日志。
- 提交号用于追溯当时仓库里的代码和默认参数。
- 实际飞行如果启动后又从 Flash 载入旧参数，则机上真实参数以 Flash 为准，不一定等于代码默认值。

## 坐标与符号

- PMW3901 原始 `deltaX > 0`：机体向左运动。
- PMW3901 原始 `deltaY > 0`：机体向前运动。
- `opflow_vel_x`：左正右负，单位 `cm/s`。
- `opflow_vel_y`：前正后负，单位 `cm/s`。
- `Pos_Est_vel_x` / `Pos_Est_vel_y`：估计器内部速度，符号与 `opflow_vel_*` 一致。
- `fc_mode2` 速度环实际送入 PID 的测量量是 `-Pos_Est_vel_*_kf`，所以：
- X 轴控制量是右正左负。
- Y 轴控制量是后正前负。

## 12 列日志格式

### X 轴日志格式

对应代码：

```c
wifi_justfloat(
    g_pmw3901_raw.deltaX, g_pmw3901_raw.squal,
    opflow_vel_x,
    acc_y_lp,
    -Pos_Est_vel_x, -Pos_Est_vel_x_kf, s_mode2_velx_pid.p_term, s_mode2_velx_pid.i_term, velx_target,
    roll_angle_target,
    g_euler.roll, g_tof_fused_height_mm / 1000.0f);
```

列定义：

| 列序号 | 字段 | 含义 | 单位 | 符号方向 |
| --- | --- | --- | --- | --- |
| 1 | `deltaX_raw` | PMW3901 原始 X 增量 | 原始计数 | 左正右负 |
| 2 | `squal` | PMW3901 质量值 | 无量纲 | 越大越可靠 |
| 3 | `opflow_vel_x` | 光流解算 X 速度 | `cm/s` | 左正右负 |
| 4 | `acc_y_lp` | 机体系 Y 轴低通加速度 | `cm/s^2` | 右正左负 |
| 5 | `-Pos_Est_vel_x` | 融合原始 X 速度取反后的值 | `cm/s` | 右正左负 |
| 6 | `-Pos_Est_vel_x_kf` | 融合末端低通 X 速度取反后的值 | `cm/s` | 右正左负 |
| 7 | `velx_p_term` | X 轴速度 PID 的 P 项 | `deg` | 右正左负 |
| 8 | `velx_i_term` | X 轴速度 PID 的 I 项 | `deg` | 右正左负 |
| 9 | `velx_target` | X 轴速度目标 | `cm/s` | 右正左负 |
| 10 | `roll_angle_target` | 输出的目标横滚角 | `deg` | 右正左负 |
| 11 | `g_euler.roll` | 实际横滚角 | `deg` | 右正左负 |
| 12 | `height_m` | TOF 融合高度 | `m` | 正值 |

### Y 轴日志格式

对应代码：

```c
wifi_justfloat(
    g_pmw3901_raw.deltaY, g_pmw3901_raw.squal,
    opflow_vel_y,
    acc_x_lp,
    -Pos_Est_vel_y, -Pos_Est_vel_y_kf, s_mode2_vely_pid.p_term, s_mode2_vely_pid.i_term, vely_target,
    pitch_angle_target,
    g_euler.pitch, g_tof_fused_height_mm / 1000.0f);
```

列定义：

| 列序号 | 字段 | 含义 | 单位 | 符号方向 |
| --- | --- | --- | --- | --- |
| 1 | `deltaY_raw` | PMW3901 原始 Y 增量 | 原始计数 | 前正后负 |
| 2 | `squal` | PMW3901 质量值 | 无量纲 | 越大越可靠 |
| 3 | `opflow_vel_y` | 光流解算 Y 速度 | `cm/s` | 前正后负 |
| 4 | `acc_x_lp` | 机体系 X 轴低通加速度 | `cm/s^2` | 前正后负 |
| 5 | `-Pos_Est_vel_y` | 融合原始 Y 速度取反后的值 | `cm/s` | 后正前负 |
| 6 | `-Pos_Est_vel_y_kf` | 融合末端低通 Y 速度取反后的值 | `cm/s` | 后正前负 |
| 7 | `vely_p_term` | Y 轴速度 PID 的 P 项 | `deg` | 后正前负 |
| 8 | `vely_i_term` | Y 轴速度 PID 的 I 项 | `deg` | 后正前负 |
| 9 | `vely_target` | Y 轴速度目标 | `cm/s` | 后正前负 |
| 10 | `pitch_angle_target` | 输出的目标俯仰角 | `deg` | 后正前负 |
| 11 | `g_euler.pitch` | 实际俯仰角 | `deg` | 后正前负 |
| 12 | `height_m` | TOF 融合高度 | `m` | 正值 |

## 各 CSV 文件与参数快照

| 文件 | 轴向 | 对应提交 | 控制结构 | 关键默认参数与估计参数 | 备注 |
| --- | --- | --- | --- | --- | --- |
| `03280742_velx_pid.csv` | X | `badcf01` | 纯速度 PI | `roll_trim=-0.5`，`pitch_trim=0.0`，`vel_x=0.10/0.02/4.5`，`vel_y=0.10/0.02/4.5`，`pos_est_k_flow=0.60`，`vel_out_alpha=0.71539046`，`acc_alpha=0.20`，`squal>=25` | `mode2` 定速为 0 的 X 轴日志 |
| `03302334_velx_pid.csv` | X | `0d24730` | 纯速度 PI | `roll_trim=-0.5`，`pitch_trim=0.0`，`vel_x=0.10/0.02/4.5`，`vel_y=0.10/0.02/4.5`，`pos_est_k_flow=0.30`，`vel_out_alpha=0.71539046`，`acc_alpha=0.20`，`squal>=25` | 第一份明确用于模式 2 X 轴分析的日志 |
| `03310025_velx_pid.csv` | X | `6cd8b83` | 纯速度 PI | `roll_trim=-0.5`，`pitch_trim=0.0`，`vel_x=0.14/0.015/4.0`，`vel_y=0.10/0.02/4.5`，`pos_est_k_flow=0.45`，`vel_out_alpha=0.80`，`acc_alpha=0.30`，`squal>=25` | 第一轮提速后复飞日志 |
| `03310718_velx_pid.csv` | X | `bd1c2e3` | 纯速度 PI | `roll_trim=0.65`，`pitch_trim=0.0`，`vel_x=0.15/0.012/4.0`，`vel_y=0.10/0.02/4.5`，`pos_est_k_flow=0.50`，`vel_out_alpha=0.85`，`acc_alpha=0.30`，`squal>=25` | 回退后公认手感较好的一版 X 轴日志 |
| `03311103_velx_pid.csv` | X | `b939380` | 前馈/刹车实验版 | `roll_trim=0.42`，`pitch_trim=0.0`，`vel_x=0.135/0.010/3.5`，`vel_y=0.135/0.010/3.5`，`pos_est_k_flow=0.50`，`vel_out_alpha=0.90`，`acc_alpha=0.30`，`squal>=35` | 这版实飞主观明显更抖，后续整体回退 |
| `03311130_velx_pid.csv` | X | `92e48c1` | 回退后的纯速度 PI | `roll_trim=0.60`，`pitch_trim=0.0`，`vel_x=0.15/0.010/4.0`，`vel_y=0.10/0.02/4.5`，`pos_est_k_flow=0.50`，`vel_out_alpha=0.85`，`acc_alpha=0.30`，`squal>=25` | 回退后重新采集的 X 轴日志 |
| `03311215_velx_pid.csv` | X | `5e0992c` | 纯速度 PI | `roll_trim=0.46`，`pitch_trim=0.0`，`vel_x=0.145/0.010/4.0`，`vel_y=0.10/0.02/4.5`，`pos_est_k_flow=0.50`，`vel_out_alpha=0.85`，`acc_alpha=0.30`，`squal>=25` | 基于 `03311130` 微调后采集的 X 轴日志 |
| `03311224_vely_pid.csv` | Y | `5e0992c` | 纯速度 PI | 与 `03311215_velx_pid.csv` 同一组参数：`roll_trim=0.46`，`pitch_trim=0.0`，`vel_x=0.145/0.010/4.0`，`vel_y=0.10/0.02/4.5`，`pos_est_k_flow=0.50`，`vel_out_alpha=0.85`，`acc_alpha=0.30`，`squal>=25` | 第一份 Y 轴专用日志，日志口切换到 `deltaY/opflow_vel_y/acc_x_lp/vely_target/pitch_angle_target` |
| `03311300_vely_pid.csv` | Y | `b5d47d6` | 纯速度 PI | `roll_trim=0.46`，`pitch_trim=-1.24`，`vel_x=0.140/0.010/4.0`，`vel_y=0.10/0.02/4.5`，`pos_est_k_flow=0.50`，`vel_out_alpha=0.85`，`acc_alpha=0.30`，`squal>=25` | 在第一次 Y 轴日志基础上，先修正 `pitch` 机械中值后采集的第二份 Y 轴日志 |

## 当前两份最新日志的分析结论

### `03311215_velx_pid.csv`

- 相比 `03311130_velx_pid.csv`，X 轴零杆更稳了：
- 零杆 `vel_kf RMS`：`20.31 -> 18.15 cm/s`
- 零杆 `roll_angle_target RMS`：`3.08 -> 2.76 deg`
- 零杆实际 `roll RMS`：`2.40 -> 2.07 deg`
- 但主动段跟杆变肉了：
- 主动段目标误差 RMS：`34.70 -> 40.94 cm/s`
- 主动段幅值比：`0.769 -> 0.719`
- 当前零杆抖动主要还是 P 项主导：零杆 `P RMS=2.63`，`I RMS=0.46`。

### `03311224_vely_pid.csv`

- 这是 Y 轴第一次专门采集的数据。
- 相比当前 X 轴 `03311215_velx_pid.csv`，Y 轴零杆附近确实更稳一些：
- 零杆 `vel_kf RMS=17.79 cm/s`，优于 X 轴的 `18.15 cm/s`
- 零杆 `pitch_angle_target RMS=2.13 deg`，优于 X 轴的 `2.76 deg`
- 零杆 `P RMS=1.78`，明显小于 X 轴的 `2.63`
- 但它并不是全面更好：
- 回中零穿越中位时间约 `1.04s`，明显慢于 X 轴约 `0.68s`
- 零杆 `I` 项长期偏在 `-1.23 deg` 附近，说明 `pitch_mech_trim_deg` 很可能还没校到位
- 所以当前只能说：Y 轴零杆抖动比当前 X 轴更小，但回中更慢，而且带着明显偏置。

### `03311300_vely_pid.csv`

- 这是把 `pitch_mech_trim_deg` 先改到 `-1.24 deg` 之后的第二份 Y 轴日志。
- 相比 `03311224_vely_pid.csv`，它的跟杆和回中明显更利索：
- 主动段目标误差 RMS：`41.15 -> 34.00 cm/s`
- 零杆 `vel_kf RMS`：`17.79 -> 16.35 cm/s`
- 回中零穿越中位时间：`1.04s -> 0.10s`
- 回中收进 `|vel|<=8 cm/s` 的中位时间：`0.90s -> 0.08s`
- 但这次还没完全把稳态偏置卸干净：
- 零杆低动态段 `i_term` 均值约 `-0.29 ~ -0.33 deg`
- 对应的稳态 `pitch_angle_target` 均值约 `-1.51 ~ -1.57 deg`
- 所以 `pitch_mech_trim_deg=-1.24` 方向是对的，但量还不够，后续继续往负方向微调更合理。
- 估计器侧这份日志给出的结论是：
- `POS_EST_ACC_LPF_ALPHA=0.30` 先不动。当前日志只记录了已经滤过的 `acc_x_lp`，没有原始 1000Hz 水平加速度，无法对这一层做更科学的重整定。
- `POS_EST_VEL_OUT_LPF_ALPHA=0.85` 先不动。当前内部 `vel_y_kf` 与 `opflow_vel_y` 在主动段的相关峰值出现在 `0` 个采样点，说明这一级末端低通没有明显额外相位滞后。
- `pos_est_k_flow` 可以从 `0.50` 再小提到 `0.55`。按同一份日志回放，主动段 `vel_kf` 相对光流的 RMS 偏差可从约 `14.30` 降到约 `13.39 cm/s`，零杆速度 RMS 只从约 `16.35` 增到约 `16.63 cm/s`，属于可接受交换。
- 速度环侧当前主要矛盾已经从“大偏置”变成“输出还略硬”，所以下一步更适合小降 `vel_y_kp/ki`，不需要引入新算法或外环。

## 后续使用建议

- 后面继续采日志时，不要随手改列顺序；否则前后文件就无法直接横向对比。
- 如果切换 X/Y 轴日志口，README 里保留两种固定格式即可，不要混写。
- 做参数结论时，优先同时记录：提交号、代码默认参数、飞控是否从 Flash 覆盖默认值。
