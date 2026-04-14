# 0414_rawdata 说明与后续分析需求

## 1. 日志来源

这 4 份日志都来自 `IMU_Update_1000HZ()` 里的这一句：

```c
wifi_justfloat(tick_1000us_cnt,
               ICM42688.gyro_x, ICM42688.gyro_y, ICM42688.gyro_z,
               ICM42688.acc_x, ICM42688.acc_y, ICM42688.acc_z,
               g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy, g_imufilter_1000hz.gyroz,
               g_imufilter_1000hz.accx, g_imufilter_1000hz.accy, g_imufilter_1000hz.accz,
               g_euler.roll, g_euler.pitch, g_euler.yaw);
```

说明：

- `IMU_Update_1000HZ()` 机内执行频率是 `1000Hz`
- 理想情况下每 `1ms` 产生 1 条日志
- 但 `wifi_justfloat` 经过 WiFi 发送，CSV 会丢包
- 所以 CSV 的“行频率”不等于真实采样频率

## 2. 第一列的真实含义

第一列是：

- `tick_1000us_cnt`
- 含义：`1ms` 系统节拍计数
- 单位：`ms`

这个量表示这条数据在固件里输出时对应的真实时间刻度。

离线分析时必须按第一列恢复真实时间轴：

- 相邻两行差值为 `1`：表示真实间隔 `1ms`
- 相邻两行差值为 `2`：表示中间丢了 `1` 帧
- 相邻两行差值为 `3`：表示中间丢了 `2` 帧

结论：

- 这批日志的机内原始频率是 `1000Hz`
- 但离线频域分析、时域分析、姿态对齐时，必须以 `I0=tick_1000us_cnt` 为准
- 不能直接按 CSV 行号默认等间隔处理

## 3. CSV 列定义

这批日志共 `16` 列，顺序固定如下：

| 列号 | 名称 | 来源变量 | 含义 | 单位 |
| --- | --- | --- | --- | --- |
| I0 | tick | `tick_1000us_cnt` | 真实时间刻度 | ms |
| I1 | gyro_x_raw | `ICM42688.gyro_x` | 原始 X 轴角速度 | dps |
| I2 | gyro_y_raw | `ICM42688.gyro_y` | 原始 Y 轴角速度 | dps |
| I3 | gyro_z_raw | `ICM42688.gyro_z` | 原始 Z 轴角速度 | dps |
| I4 | acc_x_raw | `ICM42688.acc_x` | 原始 X 轴加速度 | g |
| I5 | acc_y_raw | `ICM42688.acc_y` | 原始 Y 轴加速度 | g |
| I6 | acc_z_raw | `ICM42688.acc_z` | 原始 Z 轴加速度 | g |
| I7 | gyro_x_filt | `g_imufilter_1000hz.gyrox` | 滤波后 X 轴角速度 | dps |
| I8 | gyro_y_filt | `g_imufilter_1000hz.gyroy` | 滤波后 Y 轴角速度 | dps |
| I9 | gyro_z_filt | `g_imufilter_1000hz.gyroz` | 滤波后 Z 轴角速度 | dps |
| I10 | acc_x_filt | `g_imufilter_1000hz.accx` | 滤波后 X 轴加速度 | g |
| I11 | acc_y_filt | `g_imufilter_1000hz.accy` | 滤波后 Y 轴加速度 | g |
| I12 | acc_z_filt | `g_imufilter_1000hz.accz` | 滤波后 Z 轴加速度 | g |
| I13 | roll | `g_euler.roll` | 欧拉角 roll | deg |
| I14 | pitch | `g_euler.pitch` | 欧拉角 pitch | deg |
| I15 | yaw | `g_euler.yaw` | 欧拉角 yaw | deg |

## 4. 四份飞行日志与对应滤波器

### 4.1 `04142037.csv`

这是旧参数飞行日志，对应旧滤波链路：

- gyro：
  `250Hz 二阶 Butterworth 低通 -> 161Hz 陷波(20Hz 带宽) -> 320Hz 陷波(20Hz 带宽) -> 60Hz 二阶 Butterworth 低通`
- acc：
  `161Hz 陷波(20Hz 带宽) -> 320Hz 陷波(20Hz 带宽) -> 12Hz 二阶 Butterworth 低通`

对应参数：

- `IMU_SAMPLE_RATE_HZ = 1000`
- `IMU_NOTCH0_HZ = 161`
- `IMU_NOTCH0_BW_HZ = 20`
- `IMU_NOTCH1_HZ = 320`
- `IMU_NOTCH1_BW_HZ = 20`
- `IMU_GYRO_ANTI_ALIAS_LPF_HZ = 250`
- `IMU_GYRO_LPF_HZ = 60`
- `IMU_ACCEL_LPF_HZ = 12`

### 4.2 `04142126.csv`

这是第一版新参数飞行日志，对应新滤波链路 v1：

- gyro：
  `250Hz 二阶 Butterworth 低通 -> 155Hz 陷波(45Hz 带宽) -> 80Hz 二阶 Butterworth 低通`
- acc：
  `155Hz 陷波(22Hz 带宽) -> 15Hz 二阶 Butterworth 低通`

说明：

- 第二静态陷波宏虽然还保留在代码里
- 但 `IMU_NOTCH1_ENABLE = 0`
- 所以第二陷波在实际链路中是旁路的，不参与运算

对应参数：

- `IMU_SAMPLE_RATE_HZ = 1000`
- `IMU_NOTCH0_HZ = 155`
- `IMU_NOTCH0_BW_HZ = 45`
- `IMU_ACCEL_NOTCH0_BW_HZ = 22`
- `IMU_NOTCH1_ENABLE = 0`
- `IMU_NOTCH1_HZ = 450`
- `IMU_NOTCH1_BW_HZ = 70`
- `IMU_GYRO_ANTI_ALIAS_LPF_HZ = 250`
- `IMU_GYRO_LPF_HZ = 80`
- `IMU_ACCEL_LPF_HZ = 15`

### 4.3 `04142139.csv`

这是第二版新参数飞行日志，对应新滤波链路 v2：

- gyro：
  `250Hz 二阶 Butterworth 低通 -> 155Hz 陷波(45Hz 带宽) -> 40Hz 二阶 Butterworth 低通`
- acc：
  `155Hz 陷波(22Hz 带宽) -> 15Hz 二阶 Butterworth 低通`

说明：

- 第二静态陷波依然旁路
- 这一版和 `04142126.csv` 的主要差异只有 gyro 主低通从 `80Hz` 改成了 `40Hz`

对应参数：

- `IMU_SAMPLE_RATE_HZ = 1000`
- `IMU_NOTCH0_HZ = 155`
- `IMU_NOTCH0_BW_HZ = 45`
- `IMU_ACCEL_NOTCH0_BW_HZ = 22`
- `IMU_NOTCH1_ENABLE = 0`
- `IMU_NOTCH1_HZ = 450`
- `IMU_NOTCH1_BW_HZ = 70`
- `IMU_GYRO_ANTI_ALIAS_LPF_HZ = 250`
- `IMU_GYRO_LPF_HZ = 40`
- `IMU_ACCEL_LPF_HZ = 15`

### 4.4 `04142146.csv`

这是第三版新参数飞行日志，对应新滤波链路 v3：

- gyro：
  `250Hz 二阶 Butterworth 低通 -> 155Hz 陷波(45Hz 带宽) -> 60Hz 二阶 Butterworth 低通`
- acc：
  `155Hz 陷波(22Hz 带宽) -> 15Hz 二阶 Butterworth 低通`

说明：

- 第二静态陷波依然旁路
- 这一版和 `04142126.csv`、`04142139.csv` 的主要差异是 gyro 主低通为 `60Hz`

对应参数：

- `IMU_SAMPLE_RATE_HZ = 1000`
- `IMU_NOTCH0_HZ = 155`
- `IMU_NOTCH0_BW_HZ = 45`
- `IMU_ACCEL_NOTCH0_BW_HZ = 22`
- `IMU_NOTCH1_ENABLE = 0`
- `IMU_NOTCH1_HZ = 450`
- `IMU_NOTCH1_BW_HZ = 70`
- `IMU_GYRO_ANTI_ALIAS_LPF_HZ = 250`
- `IMU_GYRO_LPF_HZ = 60`
- `IMU_ACCEL_LPF_HZ = 15`

## 5. 当前姿态解算代码现状

当前姿态解算代码在：

- `project/code/Estimation/Attitude/MahonyAhrs.c`
- `project/code/Estimation/Attitude/MahonyAhrs.h`

当前实现里，后续分析必须重点关注这些地方：

1. 当前主算法是 Mahony AHRS，输入频率 `1000Hz`
2. 当前 `MAHONY_KP_DEFAULT = 1.0f`
3. 当前 `MAHONY_KI_DEFAULT = 0.0f`
4. `Mahony_CalculateAccelWeightNearness()` 只按 `|acc|-1g` 线性降权
5. `Mahony_CalculateAccelWeightRateIgnore()` 当前直接返回 `1.0f`，等于没有角速度门控
6. `Mahony_GetFastGainScale()` 当前直接返回 `1.0f`，等于没有快速增益逻辑
7. `MahonyAhrs_Update()` 里每次都会把
   - `integral_fbx/fby/fbz`
   - `gyro_bias_x/y/z`
   重新清零
   这意味着当前实现实际上没有持续 bias 学习
8. `MahonyAhrs_SetGains()` 里也会强行把 `ki` 置成 `0.0f`
9. 当前有一个 `gyro_z` 死区：
   - `MAHONY_GYRO_Z_DEADBAND_DPS = 0.2f`
10. 当前有静止判定逻辑，但静止判定更多是状态标记，不等于已经把静止 bias 学习真正用起来

这些点很关键，因为后面“欧拉角会不会漂”和“响应是否又快又稳”，不只是滤波器问题，也和 Mahony 权重、bias、门控策略直接相关。

## 6. 后续深入分析的目标

后面要做的深入分析，目标有两个：

### 目标 1：找出最适合当前机架的 IMU 滤波参数

需要基于这 4 份 `1000Hz` 原始飞行日志做频域分析，找到：

- 哪一组参数最能把噪声压下去
- 哪一组参数最能保留真实机体姿态运动
- 哪一组参数在“降噪”和“响应速度”之间最平衡
- 哪一组参数最适合作为后续角速度环、角度环、速度环的基础输入

### 目标 2：让滤波器输出和姿态解算结果对得上

不仅要看滤波后 `gyro/acc` 是否干净，还要看：

- 滤波后的 IMU 数据是否能支撑出稳定、真实、不乱飘的欧拉角
- 不同滤波参数下，`g_euler.roll/pitch/yaw` 的响应、延迟、漂移、抖动分别怎样
- 当前 `MahonyAhrs.c` 是否存在可以简单优化的点，能进一步提高响应和结算精度，减少飘移

## 7. 给 Code X 的复制提示词

下面这段可以直接复制给新的对话：

```text
请你基于项目 D:\Car_Air_Protocol\CYT4BB7_Air，深入分析以下 4 份 1000Hz IMU 飞行日志：

1. project\code\Estimation\Attitude\0414_rawdata\04142037.csv
2. project\code\Estimation\Attitude\0414_rawdata\04142126.csv
3. project\code\Estimation\Attitude\0414_rawdata\04142139.csv
4. project\code\Estimation\Attitude\0414_rawdata\04142146.csv

先看这个说明文件：

- project\code\Estimation\Attitude\0414_rawdata\params.md

再重点查看这些代码文件：

- project\code\Estimation\Attitude\IMU_TOP.c
- project\code\Estimation\Attitude\IMU_Filtter.h
- project\code\Estimation\Attitude\IMU_Filtter.c
- project\code\Estimation\Attitude\MahonyAhrs.h
- project\code\Estimation\Attitude\MahonyAhrs.c
- project\code\HW_Drivers\ICM42688\ICM42688.h
- project\code\HW_Drivers\ICM42688\ICM42688.c

先说明我的背景和目标：

- 这是 Mark5 五寸穿越机机架
- 电机是 2207 1950KV，室内飞行，不追求激进，只追求稳定、稳稳地飞
- 当前任务是把 IMU 原始滤波和姿态结算打磨好，给后续角速度环、角度环、速度环、定高定点打基础
- 这 4 份日志都是在 IMU_Update_1000HZ() 内通过 wifi_justfloat(...) 打出来的
- 第一列 tick_1000us_cnt 不是样本编号，而是 1ms 系统节拍，必须用它恢复真实时间轴
- WiFi 会丢包，所以不要直接按 CSV 行号假设等间隔采样

这 4 份日志对应的滤波器如下：

1) 04142037.csv
- gyro: 250Hz 二阶 Butterworth 低通 -> 161Hz 陷波(20Hz带宽) -> 320Hz 陷波(20Hz带宽) -> 60Hz 二阶 Butterworth 低通
- acc: 161Hz 陷波(20Hz带宽) -> 320Hz 陷波(20Hz带宽) -> 12Hz 二阶 Butterworth 低通

2) 04142126.csv
- gyro: 250Hz 二阶 Butterworth 低通 -> 155Hz 陷波(45Hz带宽) -> 80Hz 二阶 Butterworth 低通
- acc: 155Hz 陷波(22Hz带宽) -> 15Hz 二阶 Butterworth 低通
- 第二静态陷波旁路，不参与实际链路

3) 04142139.csv
- gyro: 250Hz 二阶 Butterworth 低通 -> 155Hz 陷波(45Hz带宽) -> 40Hz 二阶 Butterworth 低通
- acc: 155Hz 陷波(22Hz带宽) -> 15Hz 二阶 Butterworth 低通
- 第二静态陷波旁路，不参与实际链路

4) 04142146.csv
- gyro: 250Hz 二阶 Butterworth 低通 -> 155Hz 陷波(45Hz带宽) -> 60Hz 二阶 Butterworth 低通
- acc: 155Hz 陷波(22Hz带宽) -> 15Hz 二阶 Butterworth 低通
- 第二静态陷波旁路，不参与实际链路

请你完成以下工作：

第一部分：四份日志的频域分析
- 对原始 gyro 和原始 acc 做频域分析
- 对滤波后 gyro 和滤波后 acc 做频域分析
- 识别每份日志的主振动峰、次峰、宽带噪声分布
- 对比 4 份日志的差异
- 给出哪组滤波器更适合当前机架的结论，并说清楚原因
- 重点关注“噪声压制”和“相位/延迟/响应速度”的平衡，不要只看谁最平滑

第二部分：滤波器与欧拉角的一致性分析
- 结合 roll/pitch/yaw 输出，分析不同滤波参数下欧拉角的抖动、漂移、延迟、跟手性
- 解释哪组滤波参数最能得到“既稳定又真实”的姿态
- 判断哪组参数最适合作为后续角速度环、角度环、速度环的基础

第三部分：对比其他飞控的做法
- 上网查看 PX4 是如何处理 1kHz 原始 gyro/acc 滤波与姿态估计输入的
- 上网查看 Betaflight 是如何处理 5 寸穿越机 gyro 滤波、notch、lowpass、RPM/dynamic notch 设计的
- 如果需要，也可以参考 ArduPilot、Madgwick、经典 Mahony、互补滤波等资料
- 必须优先引用官方文档、官方代码、原始论文或主仓库源码，不要只看二手博客
- 给出针对我这个五寸机、室内稳飞目标，哪些思路值得借鉴，哪些不值得照搬

第四部分：审查当前 MahonyAhrs.c
- 深入查看 project\code\Estimation\Attitude\MahonyAhrs.c 和 MahonyAhrs.h
- 判断有没有简单、低风险、适合当前无人机的优化点，可以提高响应、精度、稳定性，减少飘移
- 重点关注：
  1. accel 权重是否只靠 |acc|-1g 过于粗糙
  2. 角速度门控当前是否实际上未启用
  3. bias / integral 当前是否实际上被关闭
  4. ki 是否被代码强制废掉
  5. gyro_z deadband 是否合理
  6. 静止检测是否真正服务于 bias 学习
- 不要搞大改，不要过度设计，优先给出简单、能落地、风险低的优化方案

第五部分：输出形式
- 先给出事实结论，再给出理由
- 对每份日志分别总结
- 最后给一个总推荐方案：
  - 推荐的 gyro 滤波链
  - 推荐的 acc 滤波链
  - 推荐保留/删除哪些 notch
  - 推荐的 cutoff / notch 带宽
  - MahonyAhrs.c 建议做的简单优化
- 如果你建议改代码，请明确指出改哪些宏、哪些函数、哪些行为
- 请给出引用链接和文件路径

注意：
- 不要忽略第一列 tick_1000us_cnt 的真实时间意义
- 不要直接按 CSV 行号当成固定 1ms
- 不要只做滤波器分析而不看欧拉角
- 不要只看欧拉角而不看原始 1000Hz 频域
- 我的目标不是激进手感，而是稳定、真实、可靠，为后续控制环打基础
```
