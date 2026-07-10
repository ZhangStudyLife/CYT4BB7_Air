# Pos_Est V2 影子速度估计器设计

## 目标

在 `Pos_Est.c` 中实现离线 v2 的四状态固定延迟卡尔曼滤波器，并以影子方式在 MCU 上运行。V2 只输出日志，不修改旧的 `Pos_Est_vel_x`、`Pos_Est_vel_y`、速度环或飞行控制行为。

成功条件：

- 新增并运行 `Pos_Est_V2_Init()`、`Pos_Est_V2_Update_1000HZ()`、`Pos_Est_V2_Update_50HZ()`。
- 旧估计器及其控制链保持原样。
- 只消费新的 `lc302_data_seq` 光流帧。
- 使用固定延迟历史更新并回放至当前时刻。
- C 语言离线回放完整处理 `速度估计数据_2.csv`，结果有限且行数一致。
- C 与 Python v2 的速度差异中位数不超过 5 cm/s，P95 不超过 20 cm/s；未达到时继续定位原因。

## 范围

嵌入式实现只修改 `project/code/Estimation/Pos_Est/Pos_Est.c`。三个 V2 入口由旧的初始化、1000 Hz 和 50 Hz 函数在文件内部调用，不修改调度文件或头文件。允许增加一个文件内 `static` 传播函数，以避免复制状态和协方差传播代码。

离线验证新增独立的主机回放文件，不加入 IAR 工程。不得从命令行调用 IAR。

## 坐标和状态

V2 使用机体水平坐标：

- X：向左为正，单位 cm/s。
- Y：向前为正，单位 cm/s。

状态为：

```text
x = [left_velocity, forward_velocity, left_acc_bias, forward_acc_bias]
```

1000 Hz 传播使用：

```text
left_new = cos(dyaw) * left + sin(dyaw) * forward
           + (left_acc - left_bias) * dt

forward_new = -sin(dyaw) * left + cos(dyaw) * forward
              + (forward_acc - forward_bias) * dt
```

其中：

```text
left_acc    = -acc_y_lp
forward_acc =  acc_x_lp
yaw_rate    = (gyro_y * sin(roll) + gyro_z * cos(roll)) / cos(pitch)
```

`yaw_rate` 从 deg/s 转为 rad/s。`cos(pitch)` 设置下限保护，符号与当前 FRD、左正前正坐标一致。

## 参数

采用离线 v2 选出的参数：

```text
LC302 frame period       20.8 ms
sensor-clock flow delay  36 ms
sigma_acc                70 cm/s^2/sqrt(Hz)
sigma_bias_rw            0.2 cm/s^3/sqrt(Hz)
sigma_flow               0.18 rad/s
normal NIS limit         11.83
reacquire NIS limit      25.0
hard NIS limit           100.0
normal correction limit  25 cm/s
reacquire correction     35 cm/s
reacquire timeout        200 ms
bias limit               100 cm/s^2
```

## 固定延迟历史

使用 96 项、约 96 ms 的静态环形缓冲。每项保存：

- 四状态和 4x4 协方差。
- 当前样本的左右加速度和 yaw 角速度。
- 高度、垂直速度、roll/pitch 三角函数和陀螺模长。
- ToF、冲击、静态锁定标志与 MCU 时间戳。

预计静态占用约 13 至 15 KB。禁止动态内存。

每个 1000 Hz 样本先传播至当前时刻，再写入环形缓冲。静止锁定且高度低于 0.2 m 时，V2 速度约束为零，并以 5 s 时间常数学习残余加速度偏置。

## LC302 时间重建

50 Hz 更新检测 `lc302_data_seq` 是否变化。首次新帧以当前 MCU tick 初始化传感器时钟；后续按序号差累计 `20.8 ms`。若预测传感器时刻晚于当前 tick，则向当前 tick 收紧，从而在线逼近最小串口和轮询延迟。序号回退、跳变过大或长时间中断时重新建立时钟。

有效测量时刻为：

```text
measurement_time = reconstructed_sensor_time - 36 ms
```

在历史中选择不晚于该时刻的最近样本。历史不足时跳过该次更新，不修改状态。

## 光流观测

使用解耦后的 `dec_x`、`dec_y`：

```text
q = dec * 1e-4 / 0.0208
rho = vertical_height / (cos(roll) * cos(pitch))
```

LOS 模型为：

```text
qx = (cos(roll) * left
      - sin(roll) * sin(pitch) * forward
      + sin(roll) * cos(pitch) * vertical_up) / rho

qy = (cos(pitch) * forward
      + sin(pitch) * vertical_up) / rho
```

MCU 使用现有 `g_height_fused_vz_mps` 作为垂直速度，转换为 cm/s。相比旧日志中从高度因果重建垂直速度，这更接近真实在线输入。

测量必须满足：新 LC302 帧、光流有效、延迟时刻 ToF 有效、高度 0.2 至 1.4 m、倾斜余弦不低于 0.71、光流角速度不超过 2.5 rad/s，并且曝光区间内没有 IMU 冲击。

## 鲁棒更新和回放

二维 KF 使用 2x2 创新协方差和 NIS。高陀螺模长、大倾角或序号跳帧会增大测量噪声。正常和重捕获阶段使用不同 NIS 与修正限幅；重捕获时冻结 bias 更新。NIS 超过 100、矩阵不可逆或输入不可用时拒绝更新。

历史状态更新后，使用缓冲中的 IMU、yaw 和静态标志逐样本重放到当前时刻，同时覆盖后续历史状态和协方差。最终 V2 输出始终对应当前机体系，而不是延迟测量时刻。

## 影子隔离

V2 维护独立状态和输出，不写入：

- `Pos_Est_vel_x`、`Pos_Est_vel_y`
- `s_vel_pred_x`、`s_vel_pred_y`
- 旧加速度 bias
- 位置积分和任何 PID 状态

即使 V2 数值异常，也只能影响新增日志字段。初始化和重新初始化时清空全部 V2 状态、历史、时间重建和诊断值。

## 遥测字段

原 I1 至 I23 顺序和值保持不变，追加：

```text
I24  V2 left velocity, cm/s
I25  V2 forward velocity, cm/s
I26  V2 left acceleration bias, cm/s^2
I27  V2 forward acceleration bias, cm/s^2
I28  V2 left innovation, cm/s
I29  V2 forward innovation, cm/s
I30  V2 current-time correction X, cm/s
I31  V2 current-time correction Y, cm/s
I32  V2 NIS
I33  V2 packed status
I34  selected measurement age, ms
I35  fused vertical-up velocity, cm/s
I36  yaw angle, deg
```

状态位记录新帧、历史就绪、光流有效、ToF有效、几何有效、曝光冲击、接受、拒绝和重捕获状态。状态整数小于 float 的精确整数上限。

完整帧为时间戳加 36 个用户通道，共 152 字节。`wifi_justfloat` 已支持最多 60 个用户通道，且队列按最大帧静态分配，因此不会增加其队列 RAM。

## 离线 C 回放

主机回放程序通过桩定义直接包含并调用 `Pos_Est.c` 中实际的 V2 入口，不复制一份独立算法。它逐行注入 CSV 中的加速度、陀螺、姿态、高度、序号标志和光流数据。由于旧日志没有融合垂直速度，回放器仅为该日志使用与 Python v2 相同的因果高度微分；新飞行日志直接使用记录的 I35。

输出文件为 `D:\Downloads\速度估计数据_2_C语言V2回放.csv`，保留原始列并追加 `c_v2_vel_x`、`c_v2_vel_y`。同时生成指标 JSON，至少包含：

- C 与 Python v2 的逐行差异。
- 旧算法与 C V2 的光流更新前残差和相对延迟。
- 高 yaw 且双轴运动区间统计。
- 自然光流中断区间统计。
- 接受率、NIS、连续拒绝和修正量分布。

## 验证和交付

验证顺序：

1. 静态检查旧控制输出没有新增写入者。
2. 主机 C 编译器构建回放程序，不调用 IAR。
3. 完整回放 459,406 行，检查无 NaN/Inf、行数和原始列一致。
4. 与 Python v2 比较；超过既定差异阈值则继续修正。
5. 检查高 yaw、双轴速度和掉帧区间没有符号反转或异常发散。
6. 用户使用 IAR 编译并反馈 MCU 编译结果。
7. 用户完成无拉扯且含 yaw、自转加线缆拉扯两次飞行，再根据新增诊断字段分析。

本阶段不把 V2 接入速度环，也不根据影子结果自动切换估计器。
