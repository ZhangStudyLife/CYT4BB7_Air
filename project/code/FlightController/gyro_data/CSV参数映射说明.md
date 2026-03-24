# CSV 参数映射说明

## 目的

本文件用于说明 `project/code/FlightController/gyro_data` 目录下各个飞行日志 CSV 分别对应哪一套 `roll/pitch` 角速度环参数，避免后续分析时把不同参数集混在一起对比。

## 通用说明

- 所有日志均为 `1kHz` 采样。
- CSV 列定义保持一致：
  - `I0~I5` 为 `pitch target/meas/ctrl/P/I/D`
  - `I6~I11` 为 `roll target/meas/ctrl/P/I/D`
  - `I12~I14` 为欧拉角 `roll/pitch/yaw`
- 当前目录内日志共分为 3 套参数。

## 参数集 1

### 参数

- `roll_gyro_kp=3.0`
- `roll_gyro_ki=0.8`
- `roll_gyro_kd=0.03`
- `roll_gyro_kff=0`
- `roll_gyro_i_limit=300`
- `roll_gyro_d_lpf=30`
- `pitch_gyro_kp=3.0`
- `pitch_gyro_ki=0.8`
- `pitch_gyro_kd=0.03`
- `pitch_gyro_kff=0`
- `pitch_gyro_i_limit=300`
- `pitch_gyro_d_lpf=30`

### 对应日志

- `03241545_gyropid.csv`
- `03241548_gyropid.csv`
- `03241550_gyropid.csv`
- `03241552_gyropid.csv`
- `03241554_gyropid.csv`
- `03241555_gyropid.csv`

### 备注

- 这是最早的基线参数集。
- 本参数集共有 6 次飞行。

## 参数集 2

### 参数

- 记录时间：`2026-03-24 17:47:28.926`
- `roll_gyro_kp=2.5`
- `roll_gyro_ki=0.7`
- `roll_gyro_kd=0.03`
- `roll_gyro_kff=0`
- `roll_gyro_i_limit=300`
- `roll_gyro_d_lpf=30`
- `pitch_gyro_kp=2.5`
- `pitch_gyro_ki=0.7`
- `pitch_gyro_kd=0.03`
- `pitch_gyro_kff=0`
- `pitch_gyro_i_limit=300`
- `pitch_gyro_d_lpf=30`

### 对应日志

- `03241750_gyropid.csv`

### 备注

- 相比参数集 1，`roll/pitch` 同步降低了 `kp` 和 `ki`，其余不变。
- 本参数集共有 1 次飞行。

## 参数集 3

### 参数

- 记录时间：`2026-03-24 18:25:30.717`
- `roll_gyro_kp=2.5`
- `roll_gyro_ki=0.7`
- `roll_gyro_kd=0.03`
- `roll_gyro_kff=0`
- `roll_gyro_i_limit=300`
- `roll_gyro_d_lpf=40`
- `pitch_gyro_kp=2.5`
- `pitch_gyro_ki=0.7`
- `pitch_gyro_kd=0.03`
- `pitch_gyro_kff=0`
- `pitch_gyro_i_limit=300`
- `pitch_gyro_d_lpf=30`

### 对应日志

- `03241823_gyropid.csv`
- `03241825_gyropid.csv`

### 备注

- 相比参数集 2，仅修改了 `roll_gyro_d_lpf: 30 -> 40`。
- `pitch` 参数保持与参数集 2 一致。
- 本参数集共有 2 次飞行。

## 快速对照表

| 参数集 | Roll 参数 | Pitch 参数 | 对应日志数量 |
| --- | --- | --- | ---: |
| 参数集 1 | `3.0 / 0.8 / 0.03 / d_lpf=30` | `3.0 / 0.8 / 0.03 / d_lpf=30` | 6 |
| 参数集 2 | `2.5 / 0.7 / 0.03 / d_lpf=30` | `2.5 / 0.7 / 0.03 / d_lpf=30` | 1 |
| 参数集 3 | `2.5 / 0.7 / 0.03 / d_lpf=40` | `2.5 / 0.7 / 0.03 / d_lpf=30` | 2 |

## 后续维护建议

- 新增日志时，优先按“参数集”追加到本文件，不要只凭文件名记忆。
- 如果后续出现同一参数集下多种飞行场景，建议在文件名旁增加场景标签，例如“定点悬停 / 轻扰动 / 人工拨杆 / 遮挡 TOF”等，后续横向对比会更稳。
