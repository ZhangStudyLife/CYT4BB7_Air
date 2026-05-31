# 0531 自稳模式 6 次试飞 PID 深入分析

## 结论先行

推荐以第 4 次参数作为当前自稳模式的主基线。它不是单项指标绝对最强，但在平静、稳定保持、阶跃动作和较大角度下，Roll/Pitch 的角度误差、角速度误差、D 阻尼和 I 项负担最均衡。

第 5 次更像“强自稳/大角度跟随”参数：角度误差最低，尤其大角度时更贴目标，但 Roll 角速度误差和动作段超调明显增加。第 6 次更像“重载平滑/GoPro”参数：角速度跟随最干净，动作更柔，但角度外环响应更慢，Roll I 项负担明显偏大。

不建议继续沿第 3 次方向加大角度 P 或内环 P/I。第 3 次响应变快，但大角度和阶跃段同时出现更大的角度误差、角速度误差和超调，属于“推得更猛但控不住”。

## 分析依据和限制

日志字段来自本目录参数记录：角度环记录 target/measured/output/P/I/D/error，角速度环记录 target/measured/output/P/I/D。FF 没有单独记录，所以报告中 `angle_ff_residual = output - P - I - D` 只能视为“FF + 输出 PT3 滤波滞后残差”，不是纯 FF。

源码确认控制链路：

- `fc_loop.c`: 500 Hz 角度外环把角度误差转换为 `roll_gyro_target` / `pitch_gyro_target`。
- `fc_loop.c`: 1000 Hz 角速度内环输出到 `g_motor_cmd.roll` 和 `g_motor_cmd.pitch`，其中 Pitch 输出进入混控前取反。
- `pid_core.c`: `P = kp * error`，`D = -kd * measurement_delta / dt`，FF 来自 setpoint 变化率。
- `Motor_Drive.c`: Roll 正输出增加左侧电机、减少右侧电机；Pitch 混控正输出增加后方电机、减少前方电机。注意日志里的 Pitch gyro output 进入混控前会取负。

本批日志没有记录总油门、四个电机输出、电压、电机温度。因此“某项输出会让哪侧电机加/减油门”只能按源码混控推断，不能判断是否实际发生电机饱和、油门裁剪或电调响应不足。

## 官方调参原则对照

Betaflight FeedForward 文档指出 FF 与 setpoint 变化率相关，用于降低输入到响应的延迟，但过多 FF 会造成动作开始处超调、RC 阶跃放大、马达轨迹尖峰、gyro 超前 setpoint。这个现象在第 1/3/5 次里都能看到不同程度的体现。

Betaflight PID 调参文档对 P/I/D 的描述与本次日志吻合：P 决定短期误差推动力度，I 处理长期偏差和外力，D 提供阻尼并降低 P/FF 引起的超调，但 D 会放大高频噪声，需要小步增加并检查电机温度。

ArduPilot 对姿态稳定环的描述也与本工程结构一致：角度误差先转换为目标角速度，再由角速度 PID/FF 环跟踪目标角速度，最终得到姿态。

参考：

- https://betaflight.com/docs/wiki/guides/current/Feed-Forward-2-0
- https://betaflight.com/docs/wiki/guides/current/PID-Tuning-Guide
- https://ardupilot.ardupilot.org/plane/docs/new-roll-and-pitch-tuning.html

## 6 次总体表现

| 次数 | 方向 | 参数特征 | 总体判断 |
| --- | --- | --- | --- |
| 1 | 基线 | angle FF=0.12，Roll 内环无 I/D，Pitch 少量 I | FF 偏大且无 D 阻尼，Roll 大角度和阶跃超调重 |
| 2 | 关闭角度 FF | angle FF=0，其余基本同第 1 次 | 误差明显下降，说明第 1 次 FF=0.12 对这台机的自稳离散目标不合适 |
| 3 | 提高角度 P，低 FF，内环增强但无 D | angle P 更高，内环 P/I 增加，无 D | 响应更快但最不稳，大角度和阶跃误差最差 |
| 4 | 小 D 阻尼 | angle P/FF 中等，内环加入小 D | 最均衡，Pitch 尤其好，是当前推荐基线 |
| 5 | 强自稳 | angle P/FF 更高，内环更强 | 角度跟随最积极，但 Roll 角速度误差和动作超调变大 |
| 6 | 重载平滑 | angle P/FF 最低，内环 I/D 较高，D LPF 25 Hz | 角速度最干净，响应更慢，Roll I 项负担偏大 |

总体 active 段关键指标：

| 次数 | Roll 角度 RMS | Pitch 角度 RMS | Roll gyro RMS | Pitch gyro RMS | 角度滞后 Roll/Pitch | gyro 滞后 Roll/Pitch |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| 1 | 3.82 deg | 3.20 deg | 17.87 deg/s | 13.76 deg/s | 93 / 88 ms | 42 / 30 ms |
| 2 | 2.94 | 2.65 | 13.81 | 12.86 | 102 / 92 ms | 47 / 33 ms |
| 3 | 4.19 | 3.66 | 20.84 | 16.16 | 101 / 91 ms | 45 / 33 ms |
| 4 | 2.53 | 1.67 | 12.37 | 9.40 | 100 / 86 ms | 44 / 32 ms |
| 5 | 2.28 | 1.81 | 14.97 | 10.32 | 82 / 80 ms | 46 / 32 ms |
| 6 | 2.21 | 1.95 | 10.60 | 9.54 | 106 / 101 ms | 46 / 32 ms |

## 分状态分析

### 平静/小角度状态

平静状态下，目标角接近机械中位，角速度目标和实际角速度都较小。第 4 次表现最好：Roll/Pitch 角度 RMS 为 1.42/1.06 deg，gyro RMS 为 9.66/7.53 deg/s。第 6 次 gyro 也干净，但角度滞后更大，Pitch/Roll I 项负担更高。

这说明当前机体不是单纯“P 不够”。第 4 次加入小 D 后，内环跟随变干净，外环不用把误差推得很大；第 6 次靠更高 I 和 D 维持平滑，但响应钝一些。

### 阶跃/快速响应状态

第 3 次最快，但最不可取：Pitch t63 约 185 ms，Roll 216 ms，但动作后角度误差 RMS 约 5.69/5.39 deg，gyro RMS 约 26 deg/s，明显推过头。

第 4 次 Pitch 阶跃最健康：动作后角度 RMS 1.75 deg，gyro RMS 8.67 deg/s，超调约 1.28 deg。Roll 阶跃比第 5 次慢一点，但 gyro 更可控。

第 5 次角度滞后最小，但 Roll 动作段 gyro RMS 约 19.3 deg/s，Pitch 平均超调约 3.28 deg。它提高响应的代价是内环更忙、动作后更容易回弹。

第 6 次 Roll t63 约 318 ms，明显更慢，但 gyro RMS 只有约 12.1 deg/s，适合重载/平滑，不适合追求快速手感。

### 大角度状态

大角度角度误差：

- 第 5 次最好：Roll/Pitch 角度 RMS 3.32/2.29 deg。
- 第 6 次 gyro 最干净：Roll/Pitch gyro RMS 12.95/8.70 deg/s，但角度 RMS 为 4.01/3.49 deg。
- 第 4 次 Pitch 仍然很好，但 Roll 大角度 RMS 5.22 deg，说明 Roll 大角度跟随可以继续优化。
- 第 3 次最差：Roll/Pitch 大角度 RMS 7.68/7.92 deg，gyro RMS 35.88/32.93 deg/s。

因此，大角度优先时可以借鉴第 5 次的角度外环强度，但不能直接照搬第 5 次 Roll 参数，否则动作段 gyro 误差偏大。

### 稳定保持状态

稳定保持时第 4 次最均衡：Roll/Pitch 角度 RMS 1.55/1.31 deg，gyro RMS 9.72/8.53 deg/s。第 6 次 gyro 稍干净，但角度误差和相位滞后更大；第 5 次 Pitch 持稳不错，但 Roll gyro 偏高。

## P/I/D/FF 各项是否起作用

### 角度环

角度环 P 项方向正确。所有状态下 `angle_p_help_pct` 基本为 100%，说明外环 P 的符号和作用方向没有问题：目标角大于测量角时，外环输出正角速度目标，反之输出负角速度目标。

角度环 I/D 在这 6 次里都是 0，所以稳态偏差不是由角度环 I 修正，而是交给角速度环 I 和机械 trim/混控去处理。

角度 FF 的结论更谨慎：由于 FF 没有单独记录，只能看残差。经验上：

- 第 1 次 FF=0.12 相比第 2 次 FF=0，误差更大，说明 0.12 对当前自稳目标跳变过强。
- 第 3 次 FF=0.08 加高 P 后仍然很差，主要问题是外环/内环推动太猛且缺少 D。
- 第 4 次 FF=0.06 是较好的平衡点。
- 第 5 次 FF=0.10 降低角度滞后，但带来更多动作段 gyro 误差/超调。
- 第 6 次 FF=0.04 最平滑，但响应慢。

结论：当前自稳模式不宜把 angle FF 作为主要提响应手段。FF 建议维持 0.04-0.06，最多小步试到 0.08，并重点看动作开始处超调。

### 角速度环 P

角速度 P 项方向正确，gyro P help 基本接近 100%。它是实际差动油门输出的主体。图中第 5 次 Roll 阶跃可以看到 P 项快速拉高输出，gyro measured 随后追上并越过 gyro target，这就是响应快但回弹/超调的来源。

Roll 正输出按混控会增加左侧电机、减少右侧电机。Pitch 要注意日志输出进入混控前取负：日志里 Pitch gyro output 为正时，实际 mixer pitch 为负，会增加前方电机、减少后方电机。

### 角速度环 I

I 项承担稳态误差、重心偏、线缆/载荷偏置等长期偏差。它不是瞬时误差项，所以 `gyro_i_help_pct` 只有约 45%-60% 是正常的：当动作反向或误差穿越零点后，I 会短时间保持原方向，表现为“瞬时上看在帮倒忙”，但从稳态看是在补偿偏置。

第 6 次 Roll I RMS 明显偏高：总体约 8.13，阶跃事件约 9.17，大角度约 10.11。这说明第 6 次 Roll 主要靠 I 在长期补偿。优点是稳、抗偏置；缺点是可能带来低频僵硬、回正慢或 I unwind。

Pitch I 从第 4 次开始较高且有效，Pitch 稳定保持变好。第 4/5/6 次 Pitch I RMS 约 6-7，是 Pitch 能明显优于 Roll 的原因之一，但也要继续观察是否有慢摆。

### 角速度环 D

第 1-3 次 D=0，尤其第 3 次证明“加 P/I 但不加 D”会变成快而不稳。

第 4-6 次 D 项开始工作。D damping pct 大约 59%-63%，说明 D 多数时候在反向阻尼角速度变化；D 与 P 反向的比例约 27%-41%，符合“D 抵消 P/FF 引起的过冲”的角色。D RMS 仍然很小，Roll 约 0.66-1.04，Pitch 约 1.32-1.83，说明当前 D 不是主输出，只是阻尼。

风险：日志没有电机温度、马达输出高频尖峰和电压，所以不能断言 D 完全安全。下一轮加 D 必须短飞后摸电机/看温升。

## 推荐参数方向

### 当前推荐基线：第 4 次

```text
roll_angle_kp  6.8
roll_angle_kff 0.06
pitch_angle_kp  6.7
pitch_angle_kff 0.06
roll_gyro_kp 4.0
roll_gyro_ki 0.06
roll_gyro_kd 0.006
pitch_gyro_kp 4.8
pitch_gyro_ki 0.11
pitch_gyro_kd 0.007
```

适合继续开发自稳模式和后续自动模式叠加，因为它对平静、稳定、快速动作、大角度都没有明显短板。

### 如果追求更强大角度跟随

不要直接使用第 5 次全套。建议只小步加强 Roll 外环：

```text
roll_angle_kp  7.0
roll_angle_kff 0.06
roll_gyro_kp   4.1
roll_gyro_ki   0.06~0.07
roll_gyro_kd   0.007~0.008
```

Pitch 可以先保持第 4 次，因为第 4 次 Pitch 已经很干净。

### 如果追求重载/视频平滑

第 6 次可作为平滑参数，但建议重点观察 Roll I：

```text
roll_angle_kp  6.2
roll_angle_kff 0.04
pitch_angle_kp  6.1
pitch_angle_kff 0.04
roll_gyro_kp 4.3
roll_gyro_ki 0.08
roll_gyro_kd 0.008
pitch_gyro_kp 5.1
pitch_gyro_ki 0.14
pitch_gyro_kd 0.010
```

如果出现回正慢、低频拖拽或动作后停不干净，优先把 `roll_gyro_ki` 从 0.08 降到 0.06-0.07，而不是继续降 P。

## 下一轮日志建议

必须增加以下遥测，否则无法判断“该加油门时是否真的加油门”、是否饱和、是否电机被 D 噪声打热：

- `g_motor_cmd.throttle`
- `g_motor_cmd.roll`
- `g_motor_cmd.pitch`
- 四个电机最终输出 `g_motor_state.output[0..3]`
- 电池电压或电调电压
- 如果可行，电机温度或至少飞后温度记录

有了电机输出后，可以进一步判断：

- Roll/Pitch 输出是否被总油门和 idle 限幅裁剪。
- D 项是否造成电机高频抖动。
- Pitch 取负后是否确实按预期让前/后电机差动。
- 大角度时是不是推力余量不足，而不是 PID 参数不足。

