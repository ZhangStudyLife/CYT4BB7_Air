# 0415 角速度环日志说明与科学调参结论

## 1. 先说结论

- 当前目录里 `2026-04-15 01:05 ~ 01:43` 一共 6 份主试飞日志，对应本次 `Roll/Pitch` 角速度环主对比：
  - `04150104.csv`
  - `04150111.csv`
  - `04150117.csv`
  - `04150132.csv`
  - `04150137.csv`
  - `04150143.csv`
- `04150147.csv` 是 `2026-04-15 01:47` 的补充试飞，参数从 `KI=0.7` 改成了 `KI=1.0`，不建议和前 6 次混成一个主结论，所以单独放在附录分析。
- 基于这 6 次主试飞，当前最稳妥的结论不是继续把 `P` 往上怼，也不是把 `D` 直接砍成 `0`：
  - `KD=0.03` 这条线比 `KD=0` 更像“有效阻尼”，不是主要矛盾。
  - `KP=1.8` 明显偏软，悬停更丝滑，但大动作跟踪太肉。
  - `KP>=4.0` 已经进入过激区，尤其 `pitch` 轴抖动和控制忙碌度明显变坏。
  - 当前更像是 `KP` 最佳区间在 `2.3 ~ 2.6`，`KI` 先别加大，建议留在 `0.6 ~ 0.7`。
- 这批日志里 `pitch(Y轴)` 始终比 `roll(X轴)` 更吵，悬停窗口内 `pitch` 角速度 RMS 大约是 `roll` 的 `1.2 ~ 1.8` 倍，因此下一轮不要死守 `roll/pitch` 完全同参。
- `04150132.csv` 丢包最重，按 `I0` 估算丢包率约 `42.73%`，而且存在一次 `87496ms` 的超大间断，这份日志只能当弱证据，不能当主依据。

## 2. 上位机到底收到了什么

当前日志来自下面这段发送代码：

```c
wifi_justfloat(tick_1000us_cnt,
               g_imufilter_1000hz.gyrox, roll_gyro_target, roll_gyro_pid.p_term, roll_gyro_pid.i_term, roll_gyro_pid.d_term,
               g_imufilter_1000hz.gyroy, pitch_gyro_target, pitch_gyro_pid.p_term, pitch_gyro_pid.i_term, pitch_gyro_pid.d_term,
               g_euler.roll, g_euler.pitch, g_euler.yaw);
```

发送链路不是文本，而是 `VOFA+ JustFloat` 二进制帧：

- 每帧发送 `14` 个 `float`
- 每个 `float` 按 `4` 字节原样打包
- 帧尾固定是 `00 00 80 7F`
- 通过 WiFi UDP 发往上位机
- 上位机 IP/端口来自代码：
  - `UDP_REMOTE_IP = 192.168.110.183`
  - `UDP_REMOTE_PORT = 1347`
  - `UDP_LOCAL_PORT = 6666`

这意味着：

- CSV 里看到的 `I0 ~ I13`，本质上就是上面 14 个 `float` 按顺序落盘。
- 列名没带语义，只是上位机导出时没有保留别名。
- `wifi_justfloat()` 只是“打包并提交发送”。
- 真正的 UDP 发包是在后续 `wifi_cmd_Poll()` 里推进完成。
- 所以只要 WiFi、SPI、上位机接收或 VOFA 落盘链路有抖动，就可能丢包。

## 3. CSV 列映射

| 列号    | 含义                         | 单位           | 说明                                                                                                  |
| ------- | ---------------------------- | -------------- | ----------------------------------------------------------------------------------------------------- |
| `I0`  | `tick_1000us_cnt`          | `ms`         | 1kHz 计数时间戳，第一个变量就是它。正常应基本每行 `+1`。如果跳了，优先按“链路丢包/落包丢帧”理解。 |
| `I1`  | `g_imufilter_1000hz.gyrox` | `deg/s`      | 机体系 `X` 轴角速度测量值，对应 `roll rate`。                                                     |
| `I2`  | `roll_gyro_target`         | `deg/s`      | `X` 轴角速度目标值。                                                                                |
| `I3`  | `roll_gyro_pid.p_term`     | 控制量内部单位 | `X` 轴角速度环 `P` 项。                                                                           |
| `I4`  | `roll_gyro_pid.i_term`     | 控制量内部单位 | `X` 轴角速度环 `I` 项。                                                                           |
| `I5`  | `roll_gyro_pid.d_term`     | 控制量内部单位 | `X` 轴角速度环 `D` 项。                                                                           |
| `I6`  | `g_imufilter_1000hz.gyroy` | `deg/s`      | 机体系 `Y` 轴角速度测量值，对应 `pitch rate`。                                                    |
| `I7`  | `pitch_gyro_target`        | `deg/s`      | `Y` 轴角速度目标值。                                                                                |
| `I8`  | `pitch_gyro_pid.p_term`    | 控制量内部单位 | `Y` 轴角速度环 `P` 项。                                                                           |
| `I9`  | `pitch_gyro_pid.i_term`    | 控制量内部单位 | `Y` 轴角速度环 `I` 项。                                                                           |
| `I10` | `pitch_gyro_pid.d_term`    | 控制量内部单位 | `Y` 轴角速度环 `D` 项。                                                                           |
| `I11` | `g_euler.roll`             | `deg`        | 欧拉角 `roll`。                                                                                     |
| `I12` | `g_euler.pitch`            | `deg`        | 欧拉角 `pitch`。                                                                                    |
| `I13` | `g_euler.yaw`              | `deg`        | 欧拉角 `yaw`。                                                                                      |

### 关于 `I0` 的正确理解

- `I0` 不是上位机采样序号，而是飞控侧的 `1ms` Tick。
- 如果 `I0` 连续为 `1000, 1001, 1002, 1003`，说明日志连续。
- 如果 `I0` 跳成 `1000, 1001, 1004`，大概率不是飞控停了，而是 `1002, 1003` 那两帧没到 CSV 里。
- 估算丢包率可以用：

```text
丢包率 ≈ 1 - 实收行数 / (I0_last - I0_first + 1)
```

## 4. 这次分析怎么做才算“科学”

参考 PX4、Betaflight、INAV 官方资料后，这批日志的分析原则如下：

- 先调 `rate loop`，再调 `angle/self-level` 外环。
- 先用日志判断“P 太低 / P 太高 / D 是否有效 / I 是否过大”，不要凭主观手感乱猜。
- 不把滤波当成修硬件振动的万能药。
- 对悬停类需求，重点看“小目标值窗口”的角速度抖动和控制忙碌度，而不是只看大动作时爽不爽。

本文件统一用了 3 类指标：

### 4.1 数据质量指标

- `丢包率`：根据 `I0` 估算。
- `最大时间跳变`：看有没有异常大空洞。

### 4.2 悬停窗口指标

悬停窗口定义：

```text
|roll_gyro_target| < 5 deg/s 且 |pitch_gyro_target| < 5 deg/s
```

在这个窗口里统计：

- `hover_meas_rms`：角速度测量 RMS，越小越丝滑。
- `hover_ctrl_detrended_rms`：控制输出去均值 RMS，越小越不神经质。

这里必须用“去均值 RMS”，因为你的机体存在长期偏置：

- 线缆外力
- 配平误差
- 重心/安装偏置
- 室内定点时的长期小扰动

这些会让 `I_term` 带一个很大的直流偏置。若不去均值，就会把“托住机体的长期补偿”误当成“高频抖动”。

### 4.3 大动作跟踪指标

大动作窗口定义：

```text
|roll_gyro_target| > 20 deg/s 或 |pitch_gyro_target| > 20 deg/s
```

在这个窗口里只看 `active_err_rms` 作为趋势参考。

注意：

- 这 6 次飞行的动作幅度并不完全一致。
- 所以大动作误差只能辅助判断“偏软还是偏激”，不能像实验室同轨迹复现实验那样做绝对排名。

## 5. 官方调参方法摘要

### 5.1 PX4 的核心思路

PX4 官方《Multicopter PID Tuning Guide》强调：

- 先调角速度环，再谈角度环和 setpoint smoothing。
- `P` 尽量往高调，但不能进入高频振荡。
- `D` 只加到足以压住过冲为止，过高会让电机发热、控制变得 twitchy。
- `I` 用来消除长期误差，过大则会产生慢振荡。
- 官方建议通过“悬停时快速给阶跃输入”观察是否立即跟随、不过冲、不持续振荡。

PX4 官方《MC Filter Tuning & Control Latency》还强调：

- 滤波就是“噪声”和“延迟”的交换。
- 截止频率越低，噪声越小，但延迟越大。
- 不要拿滤波去掩盖机械振动或 PID 明显过高的问题。
- 先做一轮保守 PID，再根据日志/FFT 调滤波。

### 5.2 Betaflight 的核心思路

Betaflight 官方《PID tuning》强调：

- 目标是让机体实际角速度尽量跟上命令角速度。
- `P` 太低会软，太高会快速抖。
- `I` 管长期误差，太低会漂，太高会慢振荡。
- `D` 负责阻尼，能减过冲，但会放大噪声。

Betaflight 官方《Modes》和 CLI 文档说明：

- `Angle/Horizon` 只是“自稳外环”。
- 真正的 `rate PID` 内环逻辑并没有消失。
- `Angle`/`Horizon` 额外要调的是：
  - `angle_p_gain`
  - `angle_feedforward`
  - `horizon_level_strength`
  - `horizon_limit_sticks`
- 所以对你这个项目，先把 `gyro rate loop` 调顺，再调自稳手感，顺序不能反。

### 5.3 INAV 的核心思路

INAV 官方文档和仓库说明：

- 多旋翼用的是 `PIDCD` 控制器。
- `D` 项来自 gyro measurement。
- 有 `Iterm Relax`。
- `D` 路有两级低通。
- `CD-term` 本质上相当于 setpoint 导数增强，接近 Betaflight 的 Feedforward 思路。
- INAV 官方也明确提供 `Blackbox Explorer` 与黑盒工具链，说明它的标准流程也是“打日志 -> 比较 -> 再改参数”。

INAV 官方设置文档里对 5 寸机的 `dterm_lpf_hz` 默认建议区间远高于你当前工程里的 `30Hz`，这说明：

- 你当前的 `D` 滤波已经相当保守。
- 所以现阶段没有证据支持“继续把 D 往下砍”。

## 6. 6 次主试飞参数表

| 次数 | 日志             | Roll 参数                 | Pitch 参数                | 备注                         |
| ---- | ---------------- | ------------------------- | ------------------------- | ---------------------------- |
| 1    | `04150104.csv` | `KP=2.5 KI=0.7 KD=0.03` | `KP=2.5 KI=0.7 KD=0.03` | 基线组，唯一一组 `KD=0.03` |
| 2    | `04150111.csv` | `KP=2.5 KI=0.7 KD=0.00` | `KP=2.5 KI=0.7 KD=0.00` | 只把 `D` 去掉              |
| 3    | `04150117.csv` | `KP=1.8 KI=0.7 KD=0.00` | `KP=1.8 KI=0.7 KD=0.00` | 降低 `P`                   |
| 4    | `04150132.csv` | `KP=3.2 KI=0.7 KD=0.00` | `KP=3.2 KI=0.7 KD=0.00` | 提高 `P`，但日志质量差     |
| 5    | `04150137.csv` | `KP=4.0 KI=0.7 KD=0.00` | `KP=4.0 KI=0.7 KD=0.00` | 更高 `P`                   |
| 6    | `04150143.csv` | `KP=5.0 KI=0.7 KD=0.00` | `KP=5.0 KI=0.7 KD=0.00` | 极高 `P`                   |

## 7. 6 次主试飞逐次分析

### 第 1 次：`04150104.csv`

- 参数：`KP=2.5 KI=0.7 KD=0.03`
- 丢包率：约 `15.97%`
- 最大时间跳变：`3002ms`
- 悬停窗口：
  - `hover_meas_rms`：`roll=7.84 deg/s`，`pitch=10.09 deg/s`
  - `hover_ctrl_detrended_rms`：`roll=23.46`，`pitch=33.00`
  - `hover D_detrended_rms`：`roll=10.34`，`pitch=11.60`
- 大动作误差 RMS：
  - `roll=20.17`
  - `pitch=28.16`

结论：

- 这是当前 6 次主试飞里“最像平衡点”的一组。
- `D=0.03` 并没有把系统搞得不可控，反而提供了实打实的阻尼贡献。
- 后面几组把 `D` 去掉之后，并没有得到更干净的悬停结果。

### 第 2 次：`04150111.csv`

- 参数：`KP=2.5 KI=0.7 KD=0.00`
- 丢包率：约 `17.16%`
- 最大时间跳变：`3002ms`
- 悬停窗口：
  - `hover_meas_rms`：`roll=9.18 deg/s`，`pitch=13.54 deg/s`
  - `hover_ctrl_detrended_rms`：`roll=24.45`，`pitch=36.31`
- 大动作误差 RMS：
  - `roll=27.47`
  - `pitch=32.74`

结论：

- 这组最有价值的意义就是证明：`D` 直接砍成 `0` 不是正确方向。
- 相比第 1 次，悬停抖动更大，跟踪误差也更差。
- 这说明当前 `KD=0.03` 更像“有效阻尼”，而不是主要噪声源。

### 第 3 次：`04150117.csv`

- 参数：`KP=1.8 KI=0.7 KD=0.00`
- 丢包率：约 `17.59%`
- 最大时间跳变：`3030ms`
- 悬停窗口：
  - `hover_meas_rms`：`roll=4.99 deg/s`，`pitch=6.10 deg/s`
  - `hover_ctrl_detrended_rms`：`roll=18.96`，`pitch=13.87`
- 大动作误差 RMS：
  - `roll=30.93`
  - `pitch=33.95`

结论：

- 这组悬停窗口指标最好，说明 `KP=1.8` 的确更丝滑。
- 但大动作跟踪已经明显偏软，说明 `P` 被砍过头了。
- 如果目标是“室内 1m ~ 1.5m 定高定点，既稳又别太肉”，这组不是最终解，更适合作为“过软下界”。

### 第 4 次：`04150132.csv`

- 参数：`KP=3.2 KI=0.7 KD=0.00`
- 丢包率：约 `42.73%`
- 最大时间跳变：`87496ms`
- 悬停窗口：
  - `hover_meas_rms`：`roll=8.49 deg/s`，`pitch=13.84 deg/s`
  - `hover_ctrl_detrended_rms`：`roll=28.61`，`pitch=46.41`
- 大动作误差 RMS：
  - `roll=21.69`
  - `pitch=27.52`

结论：

- 这组最大的问题不是参数本身，而是日志质量烂掉了。
- 从现有数据看，它没有明显优于第 1 次，反而悬停忙碌度更大。
- 因为丢包严重，这组不能拿来支持“`KP=3.2` 更优”这种结论。

### 第 5 次：`04150137.csv`

- 参数：`KP=4.0 KI=0.7 KD=0.00`
- 丢包率：约 `18.50%`
- 最大时间跳变：`3002ms`
- 悬停窗口：
  - `hover_meas_rms`：`roll=7.12 deg/s`，`pitch=10.79 deg/s`
  - `hover_ctrl_detrended_rms`：`roll=31.29`，`pitch=43.92`
- 大动作误差 RMS：
  - `roll=47.01`
  - `pitch=38.81`

结论：

- `P` 拉到 `4.0` 后，控制忙碌度已经明显变重。
- 大动作误差也没有变得更漂亮，反而更像“过激后跟踪变差”。
- 这组可以视为“过高 P 的开始”。

### 第 6 次：`04150143.csv`

- 参数：`KP=5.0 KI=0.7 KD=0.00`
- 丢包率：约 `16.69%`
- 最大时间跳变：`1711ms`
- 悬停窗口：
  - `hover_meas_rms`：`roll=7.95 deg/s`，`pitch=14.44 deg/s`
  - `hover_ctrl_detrended_rms`：`roll=41.81`，`pitch=72.58`
- 大动作误差 RMS：
  - `roll=35.11`
  - `pitch=30.07`

结论：

- `KP=5.0` 已经非常明确地过了。
- 尤其 `pitch` 轴，控制去均值 RMS 飙到 `72.58`，比第 1 次基线高太多。
- 这组不适合继续往这个方向试，已经属于“证据足够，别再加了”。

## 8. 补充试飞：`04150147.csv`

- 这不是 6 次主对比的一部分，而是补充验证。
- 参数：`KP=5.0 KI=1.0 KD=0.00`
- 丢包率：约 `19.71%`
- 最大时间跳变：`3072ms`
- 悬停窗口：
  - `hover_meas_rms`：`roll=7.90 deg/s`，`pitch=14.62 deg/s`
  - `hover_ctrl_detrended_rms`：`roll=41.02`，`pitch=74.11`

结论：

- 在 `KP=5.0` 已经过高的前提下，再把 `KI` 从 `0.7` 加到 `1.0`，没有带来更干净的悬停。
- `pitch` 轴反而更忙。
- 所以这组只能说明：当前阶段绝不该继续往“大 `P` + 大 `I`”方向走。

## 9. 横向总结

把 6 次主试飞放在一起看，可以得到 5 条硬结论：

1. `KD=0.03` 比 `KD=0` 更合理。第 1 次和第 2 次只差 `D`，结果第 2 次更差，所以不要继续把 `D` 砍成 `0`。
2. `KP=1.8` 是“平滑下界”，不是最终解。它证明低 `P` 可以让悬停更丝滑，但也证明系统会偏软。
3. `KP>=4.0` 已经进入过激区。第 5 次和第 6 次的悬停控制忙碌度明显变坏，尤其 `pitch`。
4. 当前最值得继续细扫的区间不是 `3.2 ~ 5.0`，而是 `2.3 ~ 2.6`。因为：

   - `2.5 / 0.7 / 0.03` 是当前最平衡基线
   - `1.8` 过软
   - `4.0`、`5.0` 过激
5. `pitch` 轴应当比 `roll` 轴更保守。这批日志里 `pitch` 在悬停窗口下始终更吵，说明后续可以接受：

   - `pitch_kp` 比 `roll_kp` 小 `0.2 ~ 0.3`
   - 或 `pitch` 轴额外一点滤波

## 10. 建议的下一轮参数

### 10.1 最稳妥起点

如果你下一轮只想先上一个“高概率比现在好”的组合，建议先试：

```c
roll_gyro_kp  = 2.5f;
roll_gyro_ki  = 0.65f;
roll_gyro_kd  = 0.03f;

pitch_gyro_kp = 2.3f;
pitch_gyro_ki = 0.65f;
pitch_gyro_kd = 0.03f;
```

理由：

- 保留 `D=0.03`，因为日志已经证明 `D=0` 更差。
- `KI` 从 `0.7` 轻微回到 `0.65`，先避免 I 过重。
- `pitch` 比 `roll` 低一点 `P`，更符合当前日志证据。

### 10.2 如果你坚持先保持两轴同参

那就先试：

```c
roll_gyro_kp  = 2.4f;
roll_gyro_ki  = 0.65f;
roll_gyro_kd  = 0.03f;

pitch_gyro_kp = 2.4f;
pitch_gyro_ki = 0.65f;
pitch_gyro_kd = 0.03f;
```

这组的定位是：

- 比 `2.5/0.7/0.03` 稍微温和一点
- 但不至于像 `1.8` 那样过软

## 11. 下一轮应该怎么飞才更科学

下一轮别再大杂烩瞎飞，统一按下面动作来：

1. 每组参数单独飞一份日志。
2. 每份日志动作尽量一致：
   - 起飞后稳定悬停 `15 ~ 20s`
   - 做 `2 ~ 3` 次小幅 `roll/pitch` 阶跃拨杆
   - 再悬停 `10s`
3. 每次只改一个维度：
   - 先定 `KD=0.03`
   - 再扫 `KP`
   - 最后微调 `KI`
4. 下一轮推荐扫点：
   - `KP=2.3 KI=0.65 KD=0.03`
   - `KP=2.5 KI=0.65 KD=0.03`
   - `KP=2.7 KI=0.65 KD=0.03`
5. 如果 `pitch` 仍明显比 `roll` 吵，再做非对称参数：
   - `roll_kp=2.5`
   - `pitch_kp=2.3`

## 12. 这批日志还缺什么

如果后面想把结论再做硬一点，建议加打下面几列：

- 电机混控后输出
- 输出限幅/饱和标志
- armed 状态
- 遥控输入原始值
- 如果后面要分析自稳外环，再加 angle target 或 outer loop 输出

没有这些量，就只能把当前结论停留在：

- 能判断 `P` 过高还是过低
- 能判断 `D=0` 不合适
- 能判断 `pitch` 比 `roll` 更敏感

但还不能百分之百断言某一段异常到底是：

- 纯粹的 rate loop 问题
- 混控限幅
- 电机输出饱和
- 还是外力/配平造成的长期偏置

## 13. 官方参考链接

- PX4 Multicopter PID Tuning Guidehttps://docs.px4.io/v1.14/zh/config_mc/pid_tuning_guide_multicopter
- PX4 MC Filter Tuning & Control Latencyhttps://docs.px4.io/v1.14/zh/config_mc/filter_tuning
- PX4 Setpoint Tuning / Trajectory Generatorhttps://docs.px4.io/main/zh/config_mc/mc_trajectory_tuning
- Betaflight PID tuninghttps://betaflight.com/docs/development/PID-tuning
- Betaflight Modeshttps://betaflight.com/docs/development/Modes
- Betaflight CLI（Angle/Horizon 参数可查）https://betaflight.com/docs/development/Cli
- INAV PID Controllerhttps://github.com/iNavFlight/inav/blob/master/docs/INAV%20PID%20Controller.md
- INAV Settingshttps://github.com/iNavFlight/inav/blob/master/docs/Settings.md
- INAV README / Blackbox 工具入口
  https://github.com/iNavFlight/inav/blob/master/readme.md



---

project\code\Estimation\Attitude\0415_gyro_data\04150228.csv

    /* ===== Roll 轴角速度环参数 ===== */

    params->roll_gyro_kp=2.6f;

    params->roll_gyro_ki=0.75f;

    params->roll_gyro_kd=0.040f;

    params->roll_gyro_kff=0.0f;

    params->roll_gyro_i_limit=300.0f;

    params->roll_gyro_d_lpf=30.0f;

    /* ===== Pitch 轴角速度环参数 ===== */

    params->pitch_gyro_kp=2.6f;

    params->pitch_gyro_ki=0.75f;

    params->pitch_gyro_kd=0.040f;

    params->pitch_gyro_kff=0.0f;

    params->pitch_gyro_i_limit=300.0f;

    params->pitch_gyro_d_lpf=30.0f;



================================================

project\code\Estimation\Attitude\0415_gyro_data\04150234.csv

/* ===== Roll 轴角速度环参数 ===== */

    params->roll_gyro_kp=3.0f;

    params->roll_gyro_ki=0.75f;

    params->roll_gyro_kd=0.050f;

    params->roll_gyro_kff=0.0f;

    params->roll_gyro_i_limit=300.0f;

    params->roll_gyro_d_lpf=30.0f;

    /* ===== Pitch 轴角速度环参数 ===== */

    params->pitch_gyro_kp=3.0f;

    params->pitch_gyro_ki=0.75f;

    params->pitch_gyro_kd=0.050f;

    params->pitch_gyro_kff=0.0f;

    params->pitch_gyro_i_limit=300.0f;

    params->pitch_gyro_d_lpf=30.0f;


==================================================

project\code\Estimation\Attitude\0415_gyro_data\04150244.csv

/* ===== Roll 轴角速度环参数 ===== */

    params->roll_gyro_kp=2.8f;

    params->roll_gyro_ki=0.8f;

    params->roll_gyro_kd=0.050f;

    params->roll_gyro_kff=0.0f;

    params->roll_gyro_i_limit=300.0f;

    params->roll_gyro_d_lpf=30.0f;

    /* ===== Pitch 轴角速度环参数 ===== */

    params->pitch_gyro_kp=2.8f;

    params->pitch_gyro_ki=0.8f;

    params->pitch_gyro_kd=0.050f;

    params->pitch_gyro_kff=0.0f;

    params->pitch_gyro_i_limit=300.0f;

    params->pitch_gyro_d_lpf=30.0f;

## 14. 给 AI 用的深入分析提示词（9 份日志版）

下面这段提示词是给另一个更强的 AI 用的。目标不是做一遍简单统计，而是要求它分 Agent、分阶段、分片段地深入分析 9 份日志，并在正式分析前先学习 PX4 官方调参和 PX4 日志查看方法。

```text
你是一个高级飞控日志分析代理。不要展示思维过程，不要输出链式推理，只输出最终结论、证据、表格、图表建议和整定建议。

你的任务不是做简单的数据处理，也不是只看整份文件平均值。你必须对 9 份角速度环日志做深入、分段、证据驱动的分析，目标是找出最适合“室内定点悬停、越稳越好”的参数方向。

如果你的运行环境支持多 Agent / 子任务并行，请必须派遣多个 Agent 分头行动。至少拆成下面 4 个角色：

1. Agent A：PX4 官方调参方法研究
- 只看 PX4 官方资料
- 研究 PX4 对 multicopter rate loop / attitude loop 是如何调参的
- 提炼 PX4 如何判断 P 太低、P 太高、D 是否有效、I 是否过大
- 重点关注稳定悬停场景，而不是竞速手感

2. Agent B：PX4 官方日志分析方法研究
- 只看 PX4 官方日志体系
- 明确指出 PX4 严格来说不是 Betaflight Blackbox，而是 ULog + Flight Review
- 研究 PX4 通常如何查看：
  - rate tracking
  - vibration
  - actuator outputs
  - dropouts
  - saturation
  - hover 段
  - 阶跃响应段
- 提炼哪些方法可以迁移到当前 CSV 日志分析

3. Agent C：9 份日志的数据质量与分段分析
- 检查每份日志的时间连续性、疑似丢包、异常长间断
- 不要把整份文件当成一个整体
- 必须把每份日志拆成不同性质的片段分别分析，至少包括：
  - 稳定段
  - 悬停段
  - 大打杆段
  - 恢复段
- 如果存在明显“类阶跃输入”片段，必须单独识别并分析

4. Agent D：整定建议综合
- 基于全部证据给出参数建议
- 不能只说哪个好一点
- 必须区分：
  - 哪组更适合悬停
  - 哪组更适合大动作跟踪
  - 哪组更容易过激
  - 哪组更容易偏软
  - 哪组最适合室内定点目标

如果你的环境不支持多 Agent，也必须严格按上面 4 个角色分阶段完成，不允许跳步。

一、任务背景
- 机体：Mark5 五寸机
- 场景：室内、无 GPS、光照不足
- 目标：1m~1.5m 定高定点
- 当前优先分析对象：roll / pitch 角速度内环
- 最终目标不是竞速手感，而是“停在空中稳稳的样子”
- 结论必须优先服务于室内悬停稳定性，打杆爽感只能做次级参考

二、主分析文件（共 9 份）
请把下面 9 份文件作为主分析对象，逐个建立独立分析小节：

- 04150104.csv：基线组，KP=2.5 KI=0.7 KD=0.03
- 04150111.csv：去 D 对照组，KP=2.5 KI=0.7 KD=0.00
- 04150117.csv：低 P 组，KP=1.8 KI=0.7 KD=0.00
- 04150132.csv：中高 P 组，KP=3.2 KI=0.7 KD=0.00
- 04150137.csv：高 P 组，KP=4.0 KI=0.7 KD=0.00
- 04150143.csv：极高 P 组，KP=5.0 KI=0.7 KD=0.00
- 04150228.csv：新试飞 1，KP=2.6 KI=0.75 KD=0.04
- 04150234.csv：新试飞 2，KP=3.0 KI=0.75 KD=0.05
- 04150244.csv：新试飞 3，KP=2.8 KI=0.8 KD=0.05

如果目录里还有其他 CSV，例如历史补充样本，可以作为旁证，但不要混进主排名。

三、已知数据格式
当前 CSV 列映射如下：
- I0=tick_1000us_cnt(ms)
- I1=gyrox
- I2=roll_gyro_target
- I3=roll_p_term
- I4=roll_i_term
- I5=roll_d_term
- I6=gyroy
- I7=pitch_gyro_target
- I8=pitch_p_term
- I9=pitch_i_term
- I10=pitch_d_term
- I11=euler_roll
- I12=euler_pitch
- I13=euler_yaw

但不要被这些字段名绑死。请抽象成下面 4 类核心量再分析：
- 时间戳
- 目标值
- 测量值
- PID 分量输出

I0 是飞控侧时间戳，不是上位机行号。I0 跳变时，优先按丢包、链路抖动、落盘缺帧理解。

四、在分析前，你必须先理解这套飞控大致怎么实现
这 9 份 CSV 不是随便抓的，它们来自当前飞控里 `wifi_justfloat(...)` 输出的 14 个 float。你要把它理解成“内环调参日志”，不是全飞控全量日志。

当前飞控大致控制链路如下：
- 姿态外环大约运行在 500Hz
- 外环使用欧拉角姿态作为反馈，输出 `roll_gyro_target` 和 `pitch_gyro_target`
- 这两个 target 的物理含义是“目标角速度”，单位是 `deg/s`
- 角速度内环大约运行在 1000Hz
- 内环使用滤波后的陀螺仪角速度 `gyrox / gyroy / gyroz` 作为测量值
- 内环 PID 计算后得到 roll / pitch / yaw 控制输出
- 然后再进入电机混控

所以这批日志的核心含义是：
- 它记录了“角速度内环”正在看什么、追什么、各个 PID 项在出多少力
- 它没有直接记录最终电机输出、饱和标志、原始未滤波 gyro、外环角度目标
- 因此你能很好地分析内环整定趋势，但不能把所有异常都直接归因到电机或混控

五、字段含义与单位说明
你必须先按下面方式理解每一列，不允许乱猜单位：

- I0 = `tick_1000us_cnt`
  - 含义：飞控侧时间戳
  - 单位：`ms`
  - 说明：正常情况下相邻行大多 `+1`，如果跨很多，优先视为丢包或记录缺帧

- I1 = `g_imufilter_1000hz.gyrox`
  - 含义：X 轴角速度测量值，也就是 roll rate 测量值
  - 单位：`deg/s`
  - 说明：这是滤波后的角速度，不是原始 gyro

- I2 = `roll_gyro_target`
  - 含义：X 轴目标角速度
  - 单位：`deg/s`
  - 说明：由外环或控制逻辑给到内环的目标

- I3 = `roll_p_term`
  - 含义：roll 角速度环 P 项输出
  - 单位：飞控内部控制输出单位
  - 说明：不是物理单位，不要把它当成 deg/s 或 N·m。它只适合在同一套代码、同一版本固件里做相对比较

- I4 = `roll_i_term`
  - 含义：roll 角速度环 I 项输出
  - 单位：飞控内部控制输出单位
  - 说明：主要反映长期偏差补偿、配平补偿、外力补偿

- I5 = `roll_d_term`
  - 含义：roll 角速度环 D 项输出
  - 单位：飞控内部控制输出单位
  - 说明：主要提供阻尼，数值越大不一定越好，要结合噪声和振荡看

- I6 = `g_imufilter_1000hz.gyroy`
  - 含义：Y 轴角速度测量值，也就是 pitch rate 测量值
  - 单位：`deg/s`

- I7 = `pitch_gyro_target`
  - 含义：Y 轴目标角速度
  - 单位：`deg/s`

- I8 = `pitch_p_term`
  - 含义：pitch 角速度环 P 项输出
  - 单位：飞控内部控制输出单位

- I9 = `pitch_i_term`
  - 含义：pitch 角速度环 I 项输出
  - 单位：飞控内部控制输出单位

- I10 = `pitch_d_term`
  - 含义：pitch 角速度环 D 项输出
  - 单位：飞控内部控制输出单位

- I11 = `g_euler.roll`
  - 含义：当前 roll 欧拉角
  - 单位：`deg`

- I12 = `g_euler.pitch`
  - 含义：当前 pitch 欧拉角
  - 单位：`deg`

- I13 = `g_euler.yaw`
  - 含义：当前 yaw 欧拉角
  - 单位：`deg`

六、调参参数本身是什么意思
每个 CSV 文件名对应一组参数。你要先理解这些参数的物理角色，再去评价它们：

- `roll_gyro_kp / pitch_gyro_kp`
  - 含义：角速度环 P 增益
  - 作用：对“目标角速度 - 实际角速度”的瞬时误差做比例放大
  - 单位：实现相关的内部增益系数，不要跨飞控直接比较

- `roll_gyro_ki / pitch_gyro_ki`
  - 含义：角速度环 I 增益
  - 作用：累计长期误差，用来抵消持续偏差、配平误差、缓慢外力
  - 单位：实现相关的内部增益系数

- `roll_gyro_kd / pitch_gyro_kd`
  - 含义：角速度环 D 增益
  - 作用：提供阻尼，抑制过冲和快速振荡
  - 单位：实现相关的内部增益系数

- `roll_gyro_kff / pitch_gyro_kff`
  - 含义：前馈增益
  - 作用：用于让目标变化时更早给控制量
  - 当前这些日志里基本都是 `0.0`
  - 单位：实现相关的内部增益系数

- `roll_gyro_i_limit / pitch_gyro_i_limit`
  - 含义：I 项限幅
  - 作用：防止积分项无限长大
  - 单位：飞控内部控制输出单位

- `roll_gyro_d_lpf / pitch_gyro_d_lpf`
  - 含义：D 项低通截止频率
  - 单位：`Hz`
  - 作用：抑制 D 项高频噪声

七、你将同时收到提示词和原始 CSV 文件
- 你会同时收到这段提示词和 9 份 CSV 原始文件
- 请基于原始 CSV 做分析，而不是只基于别人整理过的摘要
- 如果没有原始 CSV，只拿到截图或摘要，请明确写出“无法完成深入分段分析”

八、必须采用的分析方法
你不能只做全文件平均值或简单 RMS 排序，必须按下面流程做：

1. 数据质量检查
- 每份日志给出：
  - I0 起止值
  - 实收行数
  - 估算丢包率
  - 最大时间跳变
  - 该文件是否可信
- 明确指出哪些日志只能作为弱证据

2. 分段分析
- 对每份日志至少拆成：
  - 稳定段
  - 悬停段
  - 大打杆段
  - 恢复段
- 如果存在明显阶跃输入段，单独列出“阶跃响应段”
- 必须说明分段规则，不能黑箱乱切

3. 片段级分析
- 悬停段重点看：
  - 测量噪声
  - 控制忙碌度
  - P/I/D 各项谁在主导
  - roll 和 pitch 哪个更吵
- 大打杆或阶跃段重点看：
  - 跟踪速度
  - 过冲
  - 回正时间
  - 是否有二次振荡
  - D 是否提供有效阻尼
- 恢复段重点看：
  - 是否能平稳回到安静状态
  - I 是否拖尾
  - P 是否导致持续 hunting

4. 片段结论不能混为一谈
- 必须分别给出：
  - 最适合悬停的参数
  - 最适合大动作跟踪的参数
  - 最过激的参数
  - 最偏软的参数
- 最后再综合成“室内定点目标最优方案”

九、必须研究 PX4 官方方法
你必须单独输出一节：
“PX4 官方调参与日志分析方法，对这 9 份日志有什么启发”

这一节至少回答：
1. PX4 官方是如何调多旋翼角速度环和姿态环的？
2. PX4 如何判断 P 太高、P 太低、D 太高、I 太大？
3. PX4 日志体系如何查看？必须明确说明：
- PX4 不是 Betaflight Blackbox
- PX4 主要是 ULog + Flight Review
- Flight Review 常看哪些图
- 哪些图最接近当前要关心的 rate tracking、振动、输出饱和、dropout
4. PX4 的方法里，哪些能直接迁移到当前 9 份 CSV 分析？
5. PX4 的方法里，哪些因为当前日志字段不足而做不到？必须明确写出原因

优先使用 PX4 官方文档与官方工具说明，不要拿论坛经验贴当核心依据。

十、你最终必须输出的内容
1. 总结论
- 先用 5~10 条高密度结论概括，不要先铺垫

2. 数据质量总表
- 9 份文件全部列出

3. 逐文件分析
- 每个文件单独小节
- 每个小节都要写：
  - 参数
  - 分段结果
  - 关键现象
  - 对整定的启发

4. 横向比较表
- 至少比较下面几个维度：
  - 悬停稳定性
  - 大打杆跟踪
  - 恢复段平顺性
  - 过冲/振荡风险
  - 是否适合作为室内定点方案

5. PX4 官方方法启发

6. 最终整定建议
- 给出 1 组当前最值得先上的参数
- 给出 2~4 组后续验证参数
- 每组都要说明验证目的，不能只报数字

7. 风险与缺失数据
- 说明当前日志缺什么
- 说明哪些结论置信度高，哪些只是趋势判断

十一、输出要求
- 全部使用中文简体
- 先结论，后证据
- 多用表格
- 不要展示思维过程
- 不要只做简单统计
- 必须体现“多 Agent 分工后的综合结果”
- 如果证据不足，就直接写“证据不足”
- 最终建议必须优先服务于“室内定点悬停越稳越好”
```
