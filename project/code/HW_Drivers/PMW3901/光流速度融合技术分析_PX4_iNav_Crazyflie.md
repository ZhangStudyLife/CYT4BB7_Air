# 光流传感器(PMW3901)速度融合技术深度分析

## —— PX4 / iNav / Crazyflie 三大开源飞控光流管线对比与本项目实践

---

## 目录

1. [概述与背景](#1-概述与背景)
2. [核心数学原理：光流 → 速度](#2-核心数学原理光流--速度)
3. [PX4 完整管线分析](#3-px4-完整管线分析)
   - 3.1 驱动层：PMW3901.cpp
   - 3.2 桥接层：EKF2::UpdateFlowSample()
   - 3.3 融合层：EKF2 optical_flow_control / fusion
4. [iNav 完整管线分析](#4-inav-完整管线分析)
   - 4.1 传感器层：opflow.c
   - 4.2 导航层：navigation_pos_estimator_flow.c
5. [Crazyflie 的处理方式](#5-crazyflie-的处理方式)
6. [三大飞控对比总结](#6-三大飞控对比总结)
7. [本项目（CYT4BB7）现有方案对比](#7-本项目cyt4bb7现有方案对比)
8. [关键设计建议](#8-关键设计建议)

---

## 1. 概述与背景

### 1.1 问题定义

PMW3901 光流传感器输出的是 **像素位移量**（ΔX, ΔY），即相邻两帧图像之间的平移像素数。要将其转化为飞行器的 **水平速度（m/s）**，需要结合以下信息：

| 信息 | 来源 | 作用 |
|------|------|------|
| ΔX, ΔY（像素位移） | PMW3901 SPI 读取 | 原始光流测量 |
| 对地高度 h（米/mm） | TOF 测距仪 / 气压计 | 像素→真实距离的缩放因子 |
| 姿态角（roll, pitch, yaw） | IMU 陀螺仪积分 / 互补滤波 | 去除机体旋转引起的虚假光流 |
| 陀螺仪角速度（gx, gy, gz） | ICM42688 陀螺仪 | 实时旋转补偿 |

### 1.2 核心挑战

```
光流传感器测量到的 = 真实平移导致的光流 + 机体旋转导致的虚假光流
                     ↓                        ↓
                需要保留的                    需要去除的
```

当飞机倾斜飞行时，陀螺仪检测到的旋转会在光流传感器上产生 **大量虚假像素位移**，必须精确补偿后才能提取真实的平移速度。

---

## 2. 核心数学原理：光流 → 速度

### 2.1 光流角速率模型

PMW3901 本质上测量的是图像在焦平面上的角位移。设传感器的焦距像素常数为 $F_{pix}$（单位：pix/rad），则：

$$
\text{flowRate}_{optical} = \frac{\Delta_{pix}}{F_{pix} \cdot \Delta t} \quad [\text{rad/s}]
$$

其中：
- $\Delta_{pix}$：原始像素位移（ΔX 或 ΔY）
- $F_{pix}$：焦距像素常数（PX4 中为 385.0 pix/rad，本项目标定约为 476.2 pix/rad）
- $\Delta t$：积分时间（秒）

### 2.2 旋转补偿

机体绕某轴旋转时，光流传感器会看到整个图像的平移。设陀螺仪测得的角速率为 $\omega_{gyro}$，则：

$$
\text{flowRate}_{compensated} = \text{flowRate}_{optical} - \omega_{gyro}
$$

其中 $\omega_{gyro}$ 需要与光流测量在 **同一时间区间** 内平均。

### 2.3 速度计算

补偿后的光流角速率乘以对地距离（range）得到线速度：

$$
v = \text{flowRate}_{compensated} \times \text{range}
$$

其中 range 是沿光流传感器视线方向的对地距离：

$$
\text{range} = \frac{h}{\cos(\theta_{tilt})}
$$

- $h$：垂直对地高度（HAGL, Height Above Ground Level）
- $\theta_{tilt}$：总倾斜角，即光流传感器法线与地面法线之间的夹角

### 2.4 轴映射（关键！）

由于 PMW3901 光流传感器轴与机体轴的物理排列关系，光流 X 轴的位移对应机体 **Y 轴** 的运动（右移），光流 Y 轴的位移对应机体 **X 轴** 的运动（前进），且存在符号翻转：

$$
v_{body,x} = -\text{flowRate}_{comp,Y} \times \text{range}
$$
$$
v_{body,y} = +\text{flowRate}_{comp,X} \times \text{range}
$$

---

## 3. PX4 完整管线分析

PX4 的光流处理分为三层：**驱动层** → **桥接层** → **EKF2 融合层**。

### 3.1 驱动层：PMW3901.cpp

**源文件**：`src/drivers/optical_flow/pmw3901/PMW3901.cpp`

#### 3.1.1 SPI 通信与采样频率

```cpp
#define PMW3901_SPI_BUS_SPEED   (2000000)   // SPI 时钟 2MHz
#define PMW3901_SAMPLE_INTERVAL (10000)     // 10ms = 100Hz SPI轮询频率
```

PMW3901 以 **100Hz** 的频率通过 SPI 被轮询，读取原始 ΔX, ΔY 寄存器。

#### 3.1.2 数据累积（Accumulation）

PX4 驱动 **不对每次 SPI 读取立即发布**，而是累积多次读取结果：

```cpp
// 每次SPI读取后累加
_flow_sum_x += delta_x_raw;
_flow_sum_y += delta_y_raw;
_sum_count++;

// 当累积时间达到 15ms（约66Hz发布频率）时才发布
if (_collect_time >= 15000) {
    // 像素→弧度转换：除以焦距常数 385.0
    report.pixel_flow[0] = (float)_flow_sum_x / 385.0f;  // [rad]
    report.pixel_flow[1] = (float)_flow_sum_y / 385.0f;  // [rad]
    report.integration_timespan_us = _collect_time;       // [us]
    report.quality = _sum_squal / _sum_count;             // 平均SQUAL

    // 重置累加器
    _flow_sum_x = 0;
    _flow_sum_y = 0;
    _collect_time = 0;
    _sum_count = 0;
}
```

**关键参数**：
| 参数 | 值 | 说明 |
|------|-----|------|
| `385.0f` | pix/rad | 焦距像素常数（弧度域） |
| `15000` us | 15ms | 最小累积时间 → ~50-66Hz 发布 |
| `±240` pix | 离群值门限 | 超过 ±240 像素的单次读数被丢弃 |

#### 3.1.3 质量门限

```cpp
// 读取 SQUAL 寄存器（0x07）
uint8_t squal = registerRead(0x07);

// 质量为 0 时不累加（无有效运动特征）
if (squal > 0) {
    _flow_sum_x += delta_x_raw;
    _flow_sum_y += delta_y_raw;
}
```

#### 3.1.4 输出格式

驱动发布 `vehicle_optical_flow` uORB 消息，关键字段：

```
pixel_flow[0]           : X轴角位移 [rad]（已 ÷ 385.0）
pixel_flow[1]           : Y轴角位移 [rad]（已 ÷ 385.0）
integration_timespan_us : 积分时间 [us]
quality                 : 质量指标（0~255）
delta_angle[0..2]       : NaN（PMW3901 无板载陀螺仪）
```

> **注意**：PX4 驱动层 **不做任何滤波**，只做 **累积 + 离群值剔除 + 像素→弧度转换**。

---

### 3.2 桥接层：EKF2::UpdateFlowSample()

**源文件**：`src/modules/ekf2/EKF2.cpp`

该函数负责将驱动发布的 `vehicle_optical_flow` 消息转换为 EKF2 内部的 `flowSample` 格式。

#### 3.2.1 符号翻转

```cpp
// EKF 使用与光流传感器 **相反** 的符号约定
// EKF 假设正的 LOS（视线）角速率由图像绕传感器轴的右手旋转产生
flow_rate = Vector2f(-optical_flow.pixel_flow[0],
                     -optical_flow.pixel_flow[1]) / dt;

gyro_rate = Vector3f(-optical_flow.delta_angle[0],
                     -optical_flow.delta_angle[1],
                     -optical_flow.delta_angle[2]) / dt;
```

**重要**：这里对光流和陀螺仪数据同时取反号，保持内部一致性。

#### 3.2.2 时间戳修正

```cpp
flowSample flow {
    // 将时间戳修正到积分区间的中点
    .time_us = optical_flow.timestamp_sample
               - optical_flow.integration_timespan_us / 2,
    .flow_rate = flow_rate,
    .gyro_rate = gyro_rate,
    .quality = optical_flow.quality
};
```

时间戳修正到 **积分区间中点** 是一个重要的细节，因为累积的像素位移代表的是整个积分区间内的平均运动，其最佳时间对齐点是中点。

#### 3.2.3 有效性检查

```cpp
if (Vector2f(optical_flow.pixel_flow).isAllFinite()
    && optical_flow.integration_timespan_us < 1e6) {
    _ekf.setOpticalFlowData(flow);
}
```

积分时间超过 1 秒的数据被丢弃（视为超时/无效）。

#### 3.2.4 测距仪回退

```cpp
// 如果没有独立的距离传感器，可以使用光流模块自带的距离测量
if (PX4_ISFINITE(optical_flow.distance_m)
    && (ekf2_timestamps.timestamp > _last_range_sensor_update + 1_s)) {
    _ekf.setRangeData(range_sample);
}
```

---

### 3.3 融合层：EKF2 optical_flow_control.cpp / optical_flow_fusion.cpp

这是 PX4 光流管线中最复杂、最核心的部分。

#### 3.3.1 陀螺仪偏差估计

**源文件**：`EKF/optical_flow_control.cpp`

PX4 EKF2 内部维护了一个独立的 **光流陀螺仪偏差估计器**：

```cpp
// 参考角速率 = EKF 内部的陀螺仪角速率（已含偏差校正）
const Vector2f ref_body_rate(
    -(delta_ang / delta_ang_dt - getGyroBias()).xy()
);

// IIR 低通滤波估计陀螺仪偏差
// α = 0.01 → 时间常数约 100 × dt ≈ 1~2秒
_flow_gyro_bias = 0.99f * _flow_gyro_bias
    + 0.01f * constrain(
        flow.gyro_rate.xy() - ref_body_rate,
        -0.1f, 0.1f    // 尖峰限幅 ±0.1 rad/s ≈ ±5.7°/s
    );
```

**设计要点**：
- 使用 **极慢的 IIR 滤波器**（α=0.01）来追踪陀螺仪偏差
- 配合 **尖峰限幅**（±0.1 rad/s），避免突变干扰影响偏差估计
- 偏差估计是 **2D 向量**（X, Y 两轴独立）

#### 3.3.2 光流补偿

```cpp
// 获取陀螺仪角速率
const Vector2f gyro_rate =
    flow.gyro_rate.xy() - _flow_gyro_bias;

// 补偿后的光流 = 原始光流 - 陀螺仪角速率
const Vector2f flow_compensated =
    flow.flow_rate - gyro_rate;
```

公式：

$$
\vec{\omega}_{flow,comp} = \vec{\omega}_{flow,raw} - (\vec{\omega}_{gyro} - \vec{b}_{gyro,flow})
$$

#### 3.3.3 Range 计算（倾斜校正）

```cpp
// predictFlowHagl() 返回 EKF 估计的对地高度 (HAGL)
// _R_to_earth(2,2) = cos(tilt) = 旋转矩阵的 (3,3) 元素
float range = predictFlowHagl() / _R_to_earth(2,2);
```

物理含义：

$$
\text{range} = \frac{h_{AGL}}{\cos(\theta_{tilt})} = \frac{h_{AGL}}{R_{33}}
$$

当飞机倾斜时，光流传感器到地面的斜距 > 垂直高度，需要用 $\cos(\theta)$ 修正。

#### 3.3.4 速度预测（轴交叉映射）

```cpp
// 速度 = 补偿后光流 × 斜距
// 注意轴交叉映射：flow_x → vel_y, flow_y → vel_x（且取反）
_flow_vel_body(0) = -flow_compensated(1) * range;  // X速度 ← -Y光流
_flow_vel_body(1) =  flow_compensated(0) * range;  // Y速度 ← +X光流
```

#### 3.3.5 完整预测模型 predictFlow()

EKF2 预测当前状态应该产生的光流测量值，用于计算新息（innovation）：

```cpp
Vector2f Ekf::predictFlow(const Vector3f &vel_body) const
{
    const float range_sq = range * range;

    // 考虑传感器安装偏移（flow_pos_body）产生的额外角速率
    Vector3f vel_rel = vel_body
        - _state.vel + _R_to_earth.transpose() * Vector3f(0, 0, _state.vz)
        + omega % flow_pos_body;

    // 预测光流
    flow_pred(0) = vel_rel(1) * range_inv;   // flow_x = vy / range
    flow_pred(1) = -vel_rel(0) * range_inv;  // flow_y = -vx / range

    return flow_pred;
}
```

#### 3.3.6 自适应噪声模型

```cpp
// 质量权重：质量越高，噪声越低
const float R_best  = fmaxf(_params.ekf2_of_n_min, 0.05f);  // 默认 0.15 rad/s
const float R_worst = fmaxf(_params.ekf2_of_n_max, 0.10f);  // 默认 0.50 rad/s

// 在 R_best 和 R_worst 之间根据质量插值
float weight = (float)quality / 255.0f;
float R_LOS = R_best * weight + R_worst * (1.0f - weight);
R_LOS = R_LOS * R_LOS;  // 方差 = σ²
```

**含义**：当光流质量高时（表面纹理丰富、光照充足），使用更小的噪声方差，EKF 更信任光流数据；质量差时增大方差，EKF 更信任惯性预测。

#### 3.3.7 EKF 新息门限检测

```cpp
// 新息检测门限，默认 3.0 个标准差
const float gate = fmaxf(_params.ekf2_of_gate, 1.0f);

// 串行融合：先融合 X 轴，再融合 Y 轴
if (test_ratio[0] < gate * gate) {  // 通过卡方检验
    fuseOptFlow(0);  // 融合第一个轴
}
if (test_ratio[1] < gate * gate) {
    fuseOptFlow(1);  // 融合第二个轴
}
```

#### 3.3.8 Kalman 增益计算与状态更新

```cpp
void Ekf::fuseOptFlow(int axis) {
    // H 矩阵：光流观测对状态的雅可比
    // 主要涉及速度状态（vx, vy）和地形高度

    // Kalman 增益 K = P * H^T / (H * P * H^T + R)
    const float innovation_var = H * P * H_T + R_LOS;
    const VectorState K = P * H_T / innovation_var;

    // 状态更新
    _state.vector() += K * innovation;

    // 协方差更新：Joseph 形式
    // P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    P = (I_KH) * P * (I_KH).T + K * R_LOS * K.T;
}
```

#### 3.3.9 PX4 光流参数总结

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `EKF2_OF_DELAY` | 5 ms | 光流测量延迟补偿 |
| `EKF2_OF_N_MIN` | 0.15 rad/s | 最佳情况下的 LOS 噪声 σ |
| `EKF2_OF_N_MAX` | 0.50 rad/s | 最差情况下的 LOS 噪声 σ |
| `EKF2_OF_QMIN` | 1 | 最低可接受质量（0~255） |
| `EKF2_OF_GATE` | 3.0 | 新息门限（标准差倍数） |
| `EKF2_OF_GYR_SRC` | 0 | 陀螺仪来源（0=EKF内部） |

---

### 3.4 PX4 完整管线数据流图

```
PMW3901 (硬件)
    │
    │  SPI 2MHz, 100Hz 轮询
    ▼
┌──────────────────────────┐
│  PMW3901.cpp (驱动)      │
│  ① 读取 ΔX, ΔY 寄存器    │
│  ② SQUAL > 0 质量门限     │
│  ③ ±240 pix 离群值剔除    │
│  ④ 累积 15ms → ÷385.0    │
│     → pixel_flow [rad]    │
│  ⑤ 发布 vehicle_optical_  │
│     flow uORB 消息        │
└──────────┬───────────────┘
           │  ~50-66Hz
           ▼
┌──────────────────────────┐
│  EKF2::UpdateFlowSample  │
│  (桥接层)                │
│  ① 符号翻转（-pixel_flow）│
│  ② ÷ dt → flow_rate      │
│     [rad/s]               │
│  ③ 时间戳修正到积分中点    │
│  ④ 有效性检查（finite     │
│     + timespan < 1s）     │
│  ⑤ 送入 EKF 延迟缓冲区    │
└──────────┬───────────────┘
           │
           ▼
┌──────────────────────────┐
│  optical_flow_control.cpp│
│  (融合控制)              │
│  ① 陀螺偏差估计           │
│     α=0.01 IIR + ±0.1    │
│     rad/s 尖峰限幅        │
│  ② flow_comp = flow_rate │
│     - (gyro - bias)      │
│  ③ range = hagl/cos(tilt)│
│  ④ vel_x = -flow_y×range │
│     vel_y = +flow_x×range│
│  ⑤ 自适应噪声 R_LOS      │
│     线性插值 R_best/worst │
└──────────┬───────────────┘
           │
           ▼
┌──────────────────────────┐
│  optical_flow_fusion.cpp │
│  (EKF Kalman 融合)       │
│  ① 计算 H 矩阵           │
│  ② 新息与新息方差         │
│  ③ 卡方门限检验           │
│  ④ Kalman 增益 K          │
│  ⑤ 状态更新 x += K × inn │
│  ⑥ 协方差更新 (Joseph)    │
│  ⑦ 串行融合（X, Y 独立）  │
└──────────────────────────┘
           │
           ▼
     EKF 速度/位置状态
     → vehicle_local_position
```

---

## 4. iNav 完整管线分析

iNav 的光流处理也分为两层：**传感器层** 和 **导航估计层**。

### 4.1 传感器层：opflow.c

**源文件（本地）**：`inav/src/main/sensors/opflow.c`

#### 4.1.1 架构差异

与 PX4 直接使用 SPI 驱动 PMW3901 不同，iNav 通过 **串口协议** (CXOF) 或 **MSP 协议** 接收光流数据。这意味着 iNav 收到的已经是 **预处理过的像素位移** 而非原始寄存器值。

#### 4.1.2 质量滞回滤波

```c
#define OPFLOW_SQUAL_THRESHOLD_HIGH   35
#define OPFLOW_SQUAL_THRESHOLD_LOW    10

// 滞回（Hysteresis）：上升阈值 35，下降阈值 10
if (!opflowIsDataValid) {
    if (rawQuality >= OPFLOW_SQUAL_THRESHOLD_HIGH) {
        opflowIsDataValid = true;   // 质量从低于35上升到35以上 → 有效
    }
} else {
    if (rawQuality < OPFLOW_SQUAL_THRESHOLD_LOW) {
        opflowIsDataValid = false;  // 质量降到10以下 → 无效
    }
}
```

**设计优势**：防止质量在阈值附近反复跳变导致光流数据频繁开关。

#### 4.1.3 高频陀螺仪积分回调

这是 iNav 光流处理中最精巧的设计之一：

```c
// 该回调在每次陀螺仪更新时被调用（通常 1kHz~8kHz）
static void opflowGyroUpdateCallback(timeUs_t gyroUpdateDeltaUs,
                                     float gyroDps[XYZ_AXIS_COUNT])
{
    // 高频累积陀螺仪角速率
    for (int axis = 0; axis < 3; axis++) {
        gyroBodyRateAcc[axis] += gyroDps[axis] * gyroUpdateDeltaUs;
    }
    gyroBodyRateTimeUs += gyroUpdateDeltaUs;
}
```

在光流更新时（~100Hz）读取并重置：

```c
// 计算光流两次更新之间的平均陀螺仪角速率
for (int axis = 0; axis < 3; axis++) {
    bodyRate[axis] = gyroBodyRateAcc[axis] / gyroBodyRateTimeUs;
    gyroBodyRateAcc[axis] = 0;  // 重置
}
gyroBodyRateTimeUs = 0;
```

**优势**：
- 陀螺仪以 **kHz 级** 频率累积，光流以 **~100Hz** 消费
- 确保陀螺仪的时间区间与光流的时间区间 **完美对齐**
- 无需人为指定延迟补偿，自然匹配

#### 4.1.4 像素→角速率转换

```c
// opflow_scale：像素/度 的标定系数，默认 10.5
// 可自动标定！（见 4.1.5）

// deg/s = rawPixel / opflow_scale * (1e6 / deltaTime)
flowRate[X] = (rawDeltaX / opflow_scale) * 1e6f / deltaTime;  // [deg/s]
flowRate[Y] = (rawDeltaY / opflow_scale) * 1e6f / deltaTime;  // [deg/s]

// deg/s → rad/s
flowRate[X] = DEGREES_TO_RADIANS(flowRate[X]);
flowRate[Y] = DEGREES_TO_RADIANS(flowRate[Y]);
```

**换算关系**：
$$
\text{opflow\_scale} = 10.5 \text{ pix/deg} = 10.5 \times \frac{180}{\pi} \approx 601.5 \text{ pix/rad}
$$

> 注意：iNav 的 `opflow_scale` 以 pix/deg 为单位，而 PX4 的 385.0 以 pix/rad 为单位。
> PX4: 385.0 pix/rad = 385.0 × π/180 ≈ 6.72 pix/deg。两者差异很大，说明各自标定条件不同。

#### 4.1.5 自动标定 opflow_scale

iNav 支持在飞行中自动标定光流系数：

```c
// 条件：飞行时间 > 30秒 且 陀螺仪累积旋转 > 3600 度（10圈）
if (gyroTotalRotation > 3600.0f && flightTime > 30_s) {
    // 新系数 = 光流累积 / 陀螺仪累积
    float newScale = flowAccumulator / bodyRateAccumulator;

    // 更新 opflow_scale
    opflow_scale = newScale;
}
```

**原理**：在足够长的时间内，光流传感器看到的旋转量应该等于陀螺仪测到的旋转量（假设无平移），通过二者的比值就能标定出 opflow_scale。

### 4.2 导航层：navigation_pos_estimator_flow.c

**源文件（本地）**：`inav/src/main/navigation/navigation_pos_estimator_flow.c`

#### 4.2.1 速度计算

```c
// 补偿后光流角速率 × 对地高度 = 线速度
// 注意轴交叉映射（与 PX4 相同）
flowVel.x = -(opflow.flowRate[Y] - opflow.bodyRate[Y]) * surface.alt;
flowVel.y =  (opflow.flowRate[X] - opflow.bodyRate[X]) * surface.alt;
```

与 PX4 的公式完全一致：

$$
v_x = -(\omega_{flow,Y} - \omega_{gyro,Y}) \times h
$$
$$
v_y = +(\omega_{flow,X} - \omega_{gyro,X}) \times h
$$

#### 4.2.2 坐标系变换

```c
// 机体系 → 地球系（NED）
imuTransformVectorBodyToEarth(&flowVel);
```

使用 IMU 姿态矩阵将机体系速度变换到 NED 地球系。

#### 4.2.3 互补滤波融合

iNav 不使用 EKF，而是使用 **互补滤波器** 融合光流速度：

```c
// 速度校正：w_xy_flow_v = 2.0（默认）
float velocityWeight = posEstConfig.w_xy_flow_v;  // 权重 2.0
estVelCorr[X] += (flowVel.x - estVel[X]) * velocityWeight * dt;
estVelCorr[Y] += (flowVel.y - estVel[Y]) * velocityWeight * dt;
```

互补滤波公式：

$$
\vec{v}_{est} = \vec{v}_{est} + (\vec{v}_{flow} - \vec{v}_{est}) \times w \times dt
$$

其中 $w = 2.0$ 是融合权重，越大则越信任光流。

#### 4.2.4 死推算位置积分（Dead Reckoning）

```c
// 仅在死推算模式下（无GPS）使用光流积分位置
if (dead_reckoning) {
    flowCoordinates[X] += flowVel.x * dt;
    flowCoordinates[Y] += flowVel.y * dt;

    // 位置校正
    float posWeight = posEstConfig.w_xy_flow_p;  // 权重 1.0
    estPosCorr[X] += (flowCoordinates[X] - estPos[X]) * posWeight * dt;
    estPosCorr[Y] += (flowCoordinates[Y] - estPos[Y]) * posWeight * dt;
}
```

#### 4.2.5 表面可靠性检查

```c
// 必须有可靠的测距仪数据
if (surface.reliability < RANGEFINDER_RELIABILITY_LOW_THRESHOLD) {
    return;  // 不使用光流
}
```

---

### 4.3 iNav 完整管线数据流图

```
光流模块（CXOF串口/MSP）
    │
    │  ~100Hz 原始像素位移
    ▼
┌──────────────────────────┐
│  opflow.c (传感器层)      │
│  ① 质量滞回（35/10）      │
│  ② 高频陀螺积分回调       │
│     gyroAccumulator +=    │
│     gyro×dt (1-8kHz)      │
│  ③ bodyRate = acc/time    │
│  ④ flowRate = pixel /     │
│     opflow_scale / dt     │
│     → deg/s → rad/s       │
│  ⑤ 可选自动标定 scale     │
│     (30s, >3600°旋转)     │
└──────────┬───────────────┘
           │
           ▼
┌──────────────────────────┐
│  nav_pos_estimator_flow.c│
│  (导航估计层)            │
│  ① vel_x = -(flowY -     │
│     bodyY) × alt          │
│  ② vel_y = +(flowX -     │
│     bodyX) × alt          │
│  ③ body→earth 变换        │
│  ④ 互补滤波融合           │
│     w_v = 2.0 (速度)      │
│     w_p = 1.0 (位置)      │
│  ⑤ 表面可靠性检查         │
└──────────────────────────┘
           │
           ▼
      导航估计速度/位置
```

---

## 5. Crazyflie 的处理方式

Crazyflie 是一款微型四旋翼，其光流处理简洁实用。

### 5.1 SPI Burst Read

```c
// 寄存器 0x16 突发读取模式，一次读取全部运动数据
// 100Hz 读取频率
motionBurst_t motionData;
spiRead(0x16, &motionData, sizeof(motionData));

// 运动检测位
bool motionDetected = (motionData.motion & 0xB0);
```

### 5.2 可选滤波方案

Crazyflie 提供了多种可选滤波器（通过编译开关选择）：

```c
// 方案1：直接使用原始值（默认）
flow_x = motionData.deltaX;

// 方案2：IIR 低通滤波
flow_x = 0.8f * flow_x + 0.2f * motionData.deltaX;

// 方案3：滑动平均（窗口=4）
buffer[idx] = motionData.deltaX;
flow_x = (buffer[0] + buffer[1] + buffer[2] + buffer[3]) / 4;
```

### 5.3 自适应噪声模型（基于快门时间）

```c
// 快门时间越长 → 曝光越久 → 光照越暗 → 噪声越大
float stdFlow = 0.0007984f * shutter + 0.4335f;
```

快门时间（shutter）反映了光照条件：
- 光照充足 → 快门短 → 噪声小
- 光照不足 → 快门长 → 噪声大

这是一个非常实用的 **室内弱光环境** 噪声估计方法。

---

## 6. 三大飞控对比总结

| 特性 | PX4 | iNav | Crazyflie |
|------|-----|------|-----------|
| **PMW3901 接口** | SPI 直驱 | 串口/MSP 协议 | SPI Burst Read |
| **SPI 轮询频率** | 100Hz | N/A | 100Hz |
| **发布频率** | ~50-66Hz（15ms累积） | ~100Hz | ~100Hz |
| **像素→弧度常数** | 385.0 pix/rad | 10.5 pix/deg（≈601 pix/rad） | 无统一标准 |
| **自动标定** | 无 | 有（30s+3600°） | 无 |
| **陀螺仪同步** | EKF内部延迟缓冲 | 高频回调积分（kHz） | 直接采样 |
| **旋转补偿** | flow - (gyro - bias) | flowRate - bodyRate | 简化版 |
| **陀螺偏差估计** | IIR α=0.01 + ±0.1 rad/s 限幅 | 无独立偏差估计 | 无 |
| **高度/距离** | EKF HAGL / cos(tilt) | surface.alt（测距仪） | 直接使用高度 |
| **倾斜校正** | ÷ cos(tilt) → 斜距 | 未显式校正 | 未校正 |
| **轴映射** | vel_x = -flow_y, vel_y = +flow_x | 相同 | 相同 |
| **融合算法** | 扩展 Kalman 滤波（EKF） | 互补滤波（CF） | 扩展 Kalman |
| **噪声模型** | 基于质量的线性插值 | 固定权重 | 基于快门时间 |
| **新息门限** | 卡方检验 3.0σ | 无 | 简化版 |
| **驱动层滤波** | 无（仅累积+离群值剔除） | 无 | 可选 IIR/MA |
| **质量滤波** | SQUAL > 0 | 滞回 35/10 | motion bit |

---

## 7. 本项目（CYT4BB7）现有方案对比

### 7.1 当前架构回顾

本项目的光流→速度管线在 `Pos_Est.c` 中实现：

```
PMW3901 (SPI3, 100Hz)
    │
    ▼ 原始 ΔX, ΔY
    │
┌──────────────────────────────────────┐
│  Pos_Est_Update_2000HZ()            │
│  2kHz 陀螺仪角度积分：               │
│  roll_accum += gyro_x_dps × 0.0005  │
│  pitch_accum += gyro_y_dps × 0.0005 │
└──────────┬───────────────────────────┘
           │ 每 10ms 一次（100Hz）
           ▼
┌──────────────────────────────────────┐
│  Pos_Est_Update_50HZ()             │
│  ① pix_x = -deltaX, pix_y = -deltaY│
│  ② 读取 & 重置角度积累器             │
│  ③ 使用 prev_deg 补偿（延迟一帧）    │
│  ④ pix_corr = pix - K × prev_deg    │
│  ⑤ IIR 后滤波 α=0.5                 │
│  ⑥ vel = pix_corr × h / F_pix / dt  │
└──────────────────────────────────────┘
```

### 7.2 与 PX4 的关键差异

| 维度 | 本项目 | PX4 |
|------|--------|-----|
| **补偿域** | 像素域（K × deg） | 角速率域（rad/s） |
| **焦距常数** | K_X=8.40, K_Y=8.85 pix/deg（≈481/507 pix/rad） | 385.0 pix/rad |
| **陀螺积分** | 2kHz 硬件定时器 | EKF 内部延迟缓冲 |
| **延迟补偿** | prev_frame 机制 | 时间戳中点修正 |
| **后滤波** | IIR α=0.5 | 无（EKF 自带） |
| **融合层** | 无 EKF，直接计算速度 | 24 维 EKF |
| **噪声模型** | 固定滤波系数 | 自适应（质量插值） |
| **倾斜校正** | 未实现 | ÷ cos(tilt) |

### 7.3 当前方案的优势

1. **计算量极低**：纯整数/浮点算术，无矩阵运算，适合资源受限的 CYT4BB7
2. **2kHz 陀螺积分**：比 iNav 的回调更确定性，积分精度高
3. **prev_frame 延迟补偿**：简洁有效地解决了 PMW3901 的 10-15ms pipeline delay
4. **IIR 后滤波**：在保持响应性的同时减少 ~50% 残余噪声

### 7.4 当前方案可改进的方向

1. **添加倾斜校正**：`range = h / cos(tilt)` 可显著提高大角度飞行时的精度
2. **质量滞回**：参考 iNav 的 35/10 滞回设计，避免质量抖动
3. **自适应噪声**：参考 Crazyflie 的快门时间或 PX4 的质量插值
4. **陀螺偏差估计**：参考 PX4 的 α=0.01 慢速 IIR 追踪偏差

---

## 8. 关键设计建议

### 8.1 短期优化（推荐优先实施）

#### 8.1.1 倾斜校正

```c
// 在 Pos_Est_Update_50HZ() 中添加
float cos_tilt = cosf(roll_rad) * cosf(pitch_rad);
if (cos_tilt < 0.5f) cos_tilt = 0.5f;  // 限制最大60°倾斜
float range_mm = height_mm / cos_tilt;

// 然后用 range_mm 代替 height_mm 进行速度计算
```

#### 8.1.2 质量滞回

```c
#define SQUAL_HIGH_THRESH  35
#define SQUAL_LOW_THRESH   10

static bool flow_quality_ok = false;
if (!flow_quality_ok) {
    flow_quality_ok = (squal >= SQUAL_HIGH_THRESH);
} else {
    flow_quality_ok = (squal >= SQUAL_LOW_THRESH);
}
```

### 8.2 中期优化（进阶性能提升）

#### 8.2.1 角速率域补偿（替代像素域）

将当前的像素域补偿改为角速率域，与 PX4/iNav 统一：

```c
// 当前方式（像素域）：
// pix_corr = pix_raw - K * gyro_deg

// 改为（角速率域）：
float flow_rate_x = -deltaX / F_PIX / dt;  // rad/s
float flow_rate_y = -deltaY / F_PIX / dt;  // rad/s
float gyro_rate_x = gyro_x_rad_s;          // rad/s
float gyro_rate_y = gyro_y_rad_s;          // rad/s

float flow_comp_x = flow_rate_x - gyro_rate_x;  // 补偿后
float flow_comp_y = flow_rate_y - gyro_rate_y;  // 补偿后

float vel_x = -flow_comp_y * range;  // 轴交叉
float vel_y =  flow_comp_x * range;  // 轴交叉
```

#### 8.2.2 慢速陀螺偏差追踪

```c
// α = 0.01, 约 1秒时间常数（100Hz 更新）
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;

float bias_innov_x = constrain(gyro_x - ref_x, -0.1f, 0.1f);
float bias_innov_y = constrain(gyro_y - ref_y, -0.1f, 0.1f);

gyro_bias_x = 0.99f * gyro_bias_x + 0.01f * bias_innov_x;
gyro_bias_y = 0.99f * gyro_bias_y + 0.01f * bias_innov_y;
```

### 8.3 长期演进方向

如果需要更高精度的位置保持，可以考虑实现简化版 EKF：
- 4 维状态：[vx, vy, px, py]
- 2 维观测：[flow_comp_x × range, flow_comp_y × range]
- 预测：加速度计积分
- 计算量适中，可在 CYT4BB7 上 100Hz 运行

---

## 附录 A：焦距像素常数对比

| 飞控/项目 | 常数 | 单位 | 换算 (pix/rad) | 换算 (pix/deg) |
|-----------|------|------|----------------|----------------|
| PX4 | 385.0 | pix/rad | 385.0 | 6.72 |
| iNav | 10.5 | pix/deg | 601.5 | 10.5 |
| CYT4BB7 (本项目) | 8.40 / 8.85 | pix/deg | 481 / 507 | 8.40 / 8.85 |
| 理论值 (本项目光学标定) | 476.2 | pix/rad | 476.2 | 8.31 |

> 注意：不同传感器个体、安装高度、镜片差异都会影响焦距常数值，实飞标定值
> 与理论值 ±10% 的差异是正常的。

## 附录 B：参考代码文件

| 文件 | 路径 | 说明 |
|------|------|------|
| PX4 PMW3901 驱动 | `PX4-Autopilot/src/drivers/optical_flow/pmw3901/PMW3901.cpp` | 硬件驱动 |
| PX4 EKF2 桥接 | `PX4-Autopilot/src/modules/ekf2/EKF2.cpp :: UpdateFlowSample()` | 数据格式转换 |
| PX4 光流融合控制 | `PX4-Autopilot/src/modules/ekf2/EKF/optical_flow_control.cpp` | 补偿与噪声 |
| PX4 光流融合数学 | `PX4-Autopilot/src/modules/ekf2/EKF/optical_flow_fusion.cpp` | Kalman 运算 |
| iNav 光流传感器 | `inav/src/main/sensors/opflow.c` | 传感器抽象层 |
| iNav 导航光流估计 | `inav/src/main/navigation/navigation_pos_estimator_flow.c` | 互补滤波融合 |
| 本项目光流驱动 | `project/code/HW_Drivers/PMW3901/PMW3901.c` | SPI 驱动 |
| 本项目位置估计 | `project/code/Pos_Est/Pos_Est.c` | 补偿与速度计算 |

---

*文档撰写日期：2025年*
*适用硬件：CYT4BB7 + PMW3901 + ICM42688 + VL53L1X*
