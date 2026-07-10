# Pos_Est V2 方案B第一步设计

## 目标

根据两份实飞日志修正V2影子速度估计器中已被数据证实的闭环风险，同时保持旧
`Pos_Est_vel_x`、`Pos_Est_vel_y`和全部控制链不变。

本阶段成功条件：

- 普通离群处理不再进入放大修正上限的伪重捕获循环。
- 真正断流重捕获必须经过连续三帧一致性检查。
- 内部固定延迟KF仍可回放修正，但影子控制输出不产生单帧25/35 cm/s硬跳。
- 提供光流更新时间健康度，明确区分正常、降级和不可用状态。
- 遥测改为500 Hz并记录输出预测器及重捕获诊断量。
- 主机C回放通过严格编译，并完整重跑现有日志。

## 明确不做

- 不将V2接入速度环。
- 不修改旧估计器的控制输出。
- 不修改LC302去旋转FIR系数或fresh-seq调度；该项需要下一次飞行重新标定。
- 不把固定光流延迟从36 ms改成单日志拟合出的其他值。
- 不增加在线尺度、安装角或延迟状态。

## 已确认的模型

- 状态仍为左向速度、前向速度、左向残余加速度偏置、前向残余加速度偏置。
- 左向加速度为`-acc_y_lp`，前向加速度为`acc_x_lp`。
- yaw机体系旋转、LOS斜距和ToF几何保持不变。
- `g_tof_fused_height_mm`是垂直高度，LOS中的`h/(cos roll*cos pitch)`不是重复修正。
- 保留原V2的`g_height_fused_vz_mps`垂直补偿。日志未证明移除该项能改善跨场景结果，
  相位问题继续通过下一次飞行诊断，不在本阶段扩大模型改动。

## 过程和观测噪声

第一版交叉日志参数：

```text
sigma_acc        70 cm/s^2/sqrt(Hz)
sigma_bias_rw    0.2 cm/s^3/sqrt(Hz)
base sigma_flow  0.18 rad/s
normal soft NIS  11.83
normal hard NIS  100.0
normal correction limit 25 cm/s
reacquire probe limit 10 cm/s
reacquire correction limit 35 cm/s
```

动态光流噪声使用三轴陀螺模长、倾角和序号跳变。现有日志在限制低gyro X/Y、
低倾角和低加速度后，创新仍随`|gyro Z|`增加，因此本阶段不移除yaw角速度权重。

```text
sigma_scale = 1
            + 0.004 * max(gyro_xyz_dps - 10, 0)
            + 0.020 * max(tilt_deg - 5, 0)
            + 0.50  * (seq_delta > 1)
sigma_flow = 0.18 * sigma_scale
```

## 重捕获状态机

真重捕获仅在以下情况进入：

- 尚未接受过光流；或
- 距离上次接受光流超过200 ms。

普通NIS软限幅、速度修正限幅或单帧离群不得进入真重捕获，也不得提高下一帧修正
上限。

真重捕获期间：

1. 由当前LOS创新反解光流速度测量。
2. 将上一帧光流速度按20.8 ms内的yaw旋转和加速度传播到当前机体系；当前测量与该
   预测差不超过100 cm/s且NIS不超过25时，一致计数递增，否则重置。
3. 一致计数不足3时允许速度状态以最多10 cm/s/帧探测性靠近，bias保持冻结。
4. 达到3帧后允许真重捕获使用最多35 cm/s/帧的内部修正；该修正由输出预测器隔离，
   不直接出现在控制候选速度上。
5. 达到一致条件后至少执行连续3次NIS不超过11.83的更新，随后退出重捕获。

正常状态下NIS超过100直接拒绝。NIS软缩放、修正限幅或重捕获期间，本帧bias增益清零，
避免异常帧污染残余bias。

## 输出预测器

内部`s_pos_est_v2_vel_x/y`继续表示固定延迟KF回放到当前时刻后的原始速度。

新增两轴pending correction。50 Hz回放产生当前时刻修正时，将修正加入pending，使控制候选
输出保持连续。1000 Hz下：

- pending随yaw旋转到当前机体系。
- 以20 ms时间常数向原始KF速度注入。
- 修正注入加速度模长限制为1000 cm/s^2。
- pending correction超过100 cm/s时标记unreliable，超过200 cm/s时执行异常硬限幅并记录状态。
- 输出速度等于原始KF速度减去尚未注入的pending correction。

该输出仍只用于遥测，不写入旧速度变量。

## 健康度

记录距离最近一次接受光流的时间：

- 不超过100 ms：正常。
- 100至200 ms：degraded。
- 超过200 ms或从未接受：unreliable。

本阶段只输出健康度，不修改PID或飞行模式。后续接速度环时，degraded需要冻结速度环I项，
unreliable必须退出速度保持或降级为保守姿态控制。

## 遥测

遥测从1 kHz改成固定500 Hz，保留I1至I36的含义。shock发生时I1/I2保留清零前的水平
加速度，传播使用的I3/I4仍按现有保护清零。

追加：

```text
I37 V2 output left velocity, cm/s
I38 V2 output forward velocity, cm/s
I39 pending correction left, cm/s
I40 pending correction forward, cm/s
I41 time since last accepted flow, ms
I42 dynamic sigma_flow, rad/s
I43 reacquire consistency count
I44 latest output injection left, cm/s
I45 latest output injection forward, cm/s
```

I33追加状态位：bit9 degraded、bit10 unreliable、bit11 bias frozen、bit12 output rate limited、
bit13 reacquire consistent、bit14 pending clamped。
`wifi_justfloat`传入45个用户通道；上位机自动追加时间戳后CSV为I0至I45共46列，约188字节；
500 Hz约94 kB/s，低于当前36通道1 kHz的152 kB/s。

## 验证

1. `git diff --check`通过。
2. GCC使用`-Wall -Wextra -Werror`检查新增代码；仓库既有死遥测变量单独豁免。
3. 主机回放器直接包含真实`Pos_Est.c`，24列旧日志重建垂速，37/46列日志直接使用板上
   I35垂速和I36 yaw，输出原始V2和预测输出。
4. 完整重跑旧459,406行日志及两份新实飞日志，无NaN/Inf或行数变化。
5. 比较NIS、拒绝率、真重捕获次数、修正饱和、1/10/20 ms速度步进和相对光流延迟。
6. 用户用IAR检查RAM、栈和1000 Hz最坏执行时间后，再进行一次shadow飞行。
