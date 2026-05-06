# Mode2 光流速度估计与速度控制最终设计

## 设计结论

Mode2 不做硬切换大角度刹车，而做三态状态机、输出连续性、速度目标整形、动态估计权重。

最终状态：

```text
HOLD  : 低速悬停，零速度 PI，小 k_flow，允许小积分抗电缆外力
TRACK : 跟杆运动，速度目标整形 + PI + 前馈，中等 k_flow
BRAKE : 松杆刹车，旁路 PI，反向速度 P + slew 渐入，大一点 k_flow
```

核心原则：

- 跟随靠前馈和目标整形，不靠加大 PI 积分。
- 刹车靠独立 BRAKE，不让历史 PI 积分拖尾。
- 稳定靠输出连续性，不允许 `+4deg -> -12deg` 这种一帧硬跳。
- 光流融合权重跟飞行意图联动，但仍由估计器做高度和质量门控。

## 开源飞控参考结论

### ArduPilot Loiter

ArduPilot Loiter 有明确的刹车参数：

- `BRK_ACCEL`：松杆后刹车加速度。
- `BRK_JERK`：刹车过渡 jerk 限制。
- `BRK_DELAY`：松杆后延迟一段时间再开始 brake。

它不是一松杆就直接给最大反向角，而是把刹车加速度按 jerk 限制逐步推上去，再把期望速度逐步减到 0。

对本项目的启发：

- Mode2 BRAKE 必须有输出 slew/jerk 限制。
- 松杆进入 BRAKE 必须有短确认时间，避免杆量或估计速度毛刺触发。
- 刹车输出应限制二维总倾角，而不是只限制单轴。

### INAV MC Braking

INAV 的 MC Braking 是 POSHOLD 的可选修饰模式。

它的触发逻辑包含：

- 必须处于 POSHOLD 相关控制。
- 用户松杆。
- 当前水平速度超过阈值。
- 进入 braking 后锁定当前停止位置。
- 速度降低、重新打杆或超时后退出。
- 速度很高时有 braking boost。
- 速度控制内部有 acceleration/jerk limiting，braking 时可提高 jerk 限制。

对本项目的启发：

- BRAKE 不是替代 HOLD，而是松杆高速时的过渡状态。
- BRAKE 退出后应该进入 HOLD，并把当前低速状态作为新的保持状态。
- 重新打杆必须立即退出 BRAKE。
- 需要 timeout，避免估计器错误时长时间锁在 BRAKE。

### PX4 PositionControl

PX4 是位置 P 到速度目标，速度 PID 到加速度目标，再转换成姿态/推力。

关键点：

- 速度 setpoint 支持 feedforward。
- 速度控制输出的是 acceleration setpoint，不是直接角度硬跳。
- 水平输出有饱和和 anti-windup。

对本项目的启发：

- 当前项目简化为速度 PI 直接输出角度，所以更需要输出斜率限制和二维角度限幅。
- 前馈应该有独立限幅，不能无限叠到 PI 上。
- PI 积分必须有状态切换 reset/freeze 规则。

### DJI 公开资料

DJI A2/NAZA 等公开手册只说明行为，不公开内部算法。

公开行为包括：

- GPS ATTI 模式下，横滚/俯仰杆回中后，飞机会锁定水平位置或悬停。
- Cruise Control 可以在松杆后维持当前水平速度。
- GPS 不满足条件时，系统退出相关模式并进入悬停或 ATTI 逻辑。

对本项目的启发：

- DJI 的稳不是简单暴力刹车，而是模式切换不突兀、目标连续、传感器不可靠时降级。
- 本项目室内无 GPS，只能借鉴行为：松杆后速度目标平滑走到 0，姿态角不能硬跳。

## 参数设计

### 基础默认参数

```text
pos_est_k_flow = 0.03

vel_x_kp = 0.13
vel_y_kp = 0.13
vel_x_ki = 0.02
vel_y_ki = 0.02
vel_x_i_limit = 3.0
vel_y_i_limit = 3.0
```

### 新增 Mode2 参数

采用独立 Mode2 参数，不复用 `mode1_*`。

```c
float mode2_track_ff_deg_per_cmps;
float mode2_track_ff_limit_deg;
float mode2_target_slew_cmps2;
float mode2_angle_limit_deg;
float mode2_brake_kp;
float mode2_brake_enter_vel_cmps;
float mode2_brake_exit_vel_cmps;
float mode2_brake_exit_time_s;
float mode2_brake_timeout_s;
float mode2_brake_angle_limit_deg;
float mode2_target_enter_track_cmps;
float mode2_target_release_cmps;
float mode2_state_min_time_s;
float mode2_brake_enter_confirm_s;
float mode2_angle_slew_track_dps;
float mode2_angle_slew_brake_dps;
float mode2_k_flow_hover;
float mode2_k_flow_move;
float mode2_k_flow_brake;
float mode2_k_flow_slew_per_s;
```

### 推荐默认值

第一版默认值以稳为主：

```text
mode2_track_ff_deg_per_cmps = 0.03
mode2_track_ff_limit_deg = 5.0
mode2_target_slew_cmps2 = 400.0
mode2_angle_limit_deg = 15.0

mode2_brake_kp = 0.16
mode2_brake_enter_vel_cmps = 25.0
mode2_brake_exit_vel_cmps = 10.0
mode2_brake_exit_time_s = 0.30
mode2_brake_timeout_s = 1.20
mode2_brake_angle_limit_deg = 12.0

mode2_target_enter_track_cmps = 8.0
mode2_target_release_cmps = 2.0
mode2_state_min_time_s = 0.10
mode2_brake_enter_confirm_s = 0.06

mode2_angle_slew_track_dps = 100.0
mode2_angle_slew_brake_dps = 150.0

mode2_k_flow_hover = 0.03
mode2_k_flow_move = 0.04
mode2_k_flow_brake = 0.04
mode2_k_flow_slew_per_s = 0.10
```

说明：

- 前馈先用 `0.03`，比 `0.04` 更稳，避免五寸机室内起步冲。
- 前馈限幅 `5deg`，避免大速度目标直接把姿态推满。
- 目标速度 slew `400cm/s^2`，避免一帧把目标速度打满。
- TRACK/HOLD 角度总限幅 `15deg`。
- Brake P 先用 `0.16`，比 `0.18` 更保守。
- Brake 进入速度先用 `25cm/s`，避免低速悬停毛刺触发刹车。
- Brake 角度先限到 `12deg`，刹不住再升到 `15deg`。
- 输出 slew 是防顿挫关键，TRACK/HOLD 用 `100deg/s`，BRAKE 用 `150deg/s`。

## Mode2 状态机

### 状态

```c
typedef enum
{
    MODE2_STATE_HOLD = 0,
    MODE2_STATE_TRACK = 1,
    MODE2_STATE_BRAKE = 2
} mode2_state_e;
```

### 状态量

```text
target_abs = max(abs(g_mode2_velx_target_raw), abs(g_mode2_vely_target_raw))
vel_x_ctrl = -Pos_Est_vel_x
vel_y_ctrl = -Pos_Est_vel_y
vel_abs = sqrt(vel_x_ctrl^2 + vel_y_ctrl^2)
```

注意：

- 状态判断用 raw target，因为它代表飞手当前真实打杆意图。
- 速度判断用 `vel_x_ctrl/vel_y_ctrl`，因为当前速度 PI 的测量值就是 `-Pos_Est_vel_x/y`。
- BRAKE 输出公式单独说明，不混淆符号。

### 目标速度变量语义

Mode2 必须同时保留原始目标和整形后目标：

```c
float g_mode2_velx_target_raw;
float g_mode2_vely_target_raw;
float g_mode2_velx_target;
float g_mode2_vely_target;
```

语义：

- `g_mode2_velx_target_raw/y_raw`：RC 死区和限幅后的原始速度目标，用于判断飞手意图和调试。
- `g_mode2_velx_target/y_target`：经过 `mode2_target_slew_cmps2` 整形后的控制目标，送入 PI 和前馈。

日志输出优先使用整形后目标：

```text
g_mode2_velx_target
g_mode2_vely_target
```

原因：

- 控制器实际跟踪的是整形后目标。
- 如果只记录 raw target，会误判速度环延迟。
- 如果需要看打杆输入，再临时输出 `g_mode2_velx_target_raw/y_raw`。

### 状态切换总表

| 当前状态 | 条件 | 下一状态 | 动作 |
|---|---|---|---|
| HOLD | `target_abs > enter_track` 且满足最小驻留时间 | TRACK | reset PI，开始目标整形 |
| HOLD | `target_abs <= release` 且 `vel_abs > brake_enter` 持续 `brake_enter_confirm` | BRAKE | reset PI，记录进入 BRAKE 的速度向量 |
| TRACK | `target_abs <= release` 且 `vel_abs > brake_enter` 持续 `brake_enter_confirm` | BRAKE | reset PI，记录进入 BRAKE 的速度向量 |
| TRACK | `target_abs <= release` 且 `vel_abs <= brake_enter` 且满足最小驻留时间 | HOLD | reset PI |
| BRAKE | `target_abs > enter_track` | TRACK | reset PI，清 brake 计时 |
| BRAKE | `vel_abs < brake_exit` 持续 `exit_time` | HOLD | reset PI，清 brake 计时 |
| BRAKE | `vel_abs > brake_exit` 且当前速度与进入 BRAKE 的速度点积 `<= 0` | HOLD | reset PI，防止反冲来回拉 |
| BRAKE | BRAKE 超时 | HOLD | reset PI，防止估计异常锁死 |

### 为什么需要最小驻留和确认时间

没有确认时间会出现：

```text
HOLD -> BRAKE -> HOLD -> BRAKE
```

原因：

- 光流速度估计本来就有几 cm/s 到十几 cm/s 的低频波动。
- RC 死区附近也会有小抖。
- 状态机不能直接相信单帧阈值。

默认：

```text
state_min_time = 0.10s
brake_enter_confirm = 0.06s
```

## HOLD 状态

目标：

- 安静悬停。
- 抵消慢外力。
- 不追求快速动态。

控制：

```text
velx_out_raw = PID_Update(&g_mode2_velx_pid, 0, -Pos_Est_vel_x, dt)
vely_out_raw = PID_Update(&g_mode2_vely_pid, 0, -Pos_Est_vel_y, dt)
```

规则：

- 不加前馈。
- 不每帧 reset PI。
- 允许 `Ki=0.02` 的小积分抵消电缆外力。
- 估计器模式：`HOVER`，基础 `k_flow = mode2_k_flow_hover`。
- 输出使用 TRACK/HOLD slew：`mode2_angle_slew_track_dps`。

## TRACK 状态

目标：

- 跟杆快。
- 不靠积分堆速度。
- 切入切出不突兀。

### 目标速度整形

不要直接把 RC 目标速度喂给 PI，而是做限速变化：

```text
g_mode2_velx_target_raw/y_raw = RC 映射后的目标速度
g_mode2_velx_target/y_target = rate_limit(raw_target, previous_target, mode2_target_slew_cmps2)
```

默认：

```text
mode2_target_slew_cmps2 = 400cm/s^2
```

作用：

- 大杆量起步不会一帧给 200cm/s 目标。
- 松杆时目标速度平滑回零，为 BRAKE/HOLD 切换提供连续入口。

### PI + 前馈

```text
velx_pi = PID_Update(&g_mode2_velx_pid, g_mode2_velx_target, -Pos_Est_vel_x, dt)
vely_pi = PID_Update(&g_mode2_vely_pid, g_mode2_vely_target, -Pos_Est_vel_y, dt)

velx_ff = clamp(g_mode2_velx_target * mode2_track_ff_deg_per_cmps,
                -mode2_track_ff_limit_deg,
                 mode2_track_ff_limit_deg)

vely_ff = clamp(g_mode2_vely_target * mode2_track_ff_deg_per_cmps,
                -mode2_track_ff_limit_deg,
                 mode2_track_ff_limit_deg)

velx_out_raw = velx_pi + velx_ff
vely_out_raw = vely_pi + vely_ff
```

规则：

- 前馈只在 TRACK 用。
- 前馈有独立限幅。
- PI 输出与前馈叠加后再做二维角度限幅。
- 估计器模式：`MOVE`，基础 `k_flow = mode2_k_flow_move`。
- 输出使用 TRACK/HOLD slew：`mode2_angle_slew_track_dps`。

## BRAKE 状态

目标：

- 松杆后快速减速。
- 不被 PI 积分拖尾。
- 不出现刹车反冲后的来回摆。
- 不出现一帧大角度跳变。

### 进入 BRAKE 时记录速度方向

进入时记录：

```text
brake_entry_vel_x = vel_x_ctrl
brake_entry_vel_y = vel_y_ctrl
```

后续用于判断是否已经刹过头：

```text
dot = vel_x_ctrl * brake_entry_vel_x + vel_y_ctrl * brake_entry_vel_y

if vel_abs > mode2_brake_exit_vel_cmps and dot <= 0:
    BRAKE -> HOLD
```

作用：

- 速度方向反了，说明已经刹过头。
- 这时继续 BRAKE 会让飞机来回拉。
- 直接退出到 HOLD，让小 PI 接管更稳。
- 必须加 `vel_abs > mode2_brake_exit_vel_cmps`，避免接近停止时被速度估计噪声误触发。

### Brake 输出

当前普通速度 PI 在目标为 0 时：

```text
error = 0 - (-Pos_Est_vel) = Pos_Est_vel
output = Kp * Pos_Est_vel
```

因此 BRAKE 输出保持同样方向：

```text
velx_out_raw = mode2_brake_kp * Pos_Est_vel_x
vely_out_raw = mode2_brake_kp * Pos_Est_vel_y
```

规则：

- BRAKE 期间不调用 `PID_Update()`。
- BRAKE 期间 PI 积分不更新。
- BRAKE 输出先做二维总倾角限幅，再做 slew。

### Brake 二维限幅

不要单独限制 roll/pitch，否则对角方向会超过总倾角。

```text
mag = sqrt(velx_out_raw^2 + vely_out_raw^2)

if mag > mode2_brake_angle_limit_deg:
    scale = mode2_brake_angle_limit_deg / mag
    velx_out_raw *= scale
    vely_out_raw *= scale
```

### Brake slew

最终输出不能直接跳到 `velx_out_raw/vely_out_raw`。

```text
max_step = mode2_angle_slew_brake_dps * dt
velx_out = step_limit(previous_velx_out, velx_out_raw, max_step)
vely_out = step_limit(previous_vely_out, vely_out_raw, max_step)
```

默认：

```text
mode2_angle_slew_brake_dps = 150deg/s
```

50Hz 下每帧最大角度变化：

```text
150deg/s * 0.02s = 3deg/frame
```

这样刹车仍果断，但不会一帧从正角跳到大反角。

### Brake 退出

退出条件：

```text
重新打杆 -> TRACK
速度低于 brake_exit 并保持 exit_time -> HOLD
速度方向反转且 vel_abs > brake_exit -> HOLD
brake 超时 -> HOLD
```

退出动作：

```text
PID_Reset(&g_mode2_velx_pid)
PID_Reset(&g_mode2_vely_pid)
```

估计器模式：

```text
BRAKE -> Pos_Est_Set_XY_Mode(enable_mode2=1, moving=0, braking=1)
```

基础 `k_flow`：

```text
mode2_k_flow_brake = 0.04
```

## 输出连续性总规则

所有状态统一走一个输出后处理：

```text
状态控制器生成 velx_out_raw / vely_out_raw
-> 二维总倾角限幅
-> 状态对应 slew limit
-> 加机械 trim
-> 写 roll_angle_target / pitch_angle_target
```

TRACK/HOLD 总倾角限幅：

```text
mode2_angle_limit_deg = 15deg
```

BRAKE 总倾角限幅：

```text
mode2_brake_angle_limit_deg = 12deg
```

TRACK/HOLD slew：

```text
mode2_angle_slew_track_dps = 100deg/s
```

BRAKE slew：

```text
mode2_angle_slew_brake_dps = 150deg/s
```

实现注意：

- 二维速度模长和二维角度模长使用 `sqrtf()`。
- 修改 `fc_mode2.c` 时需要确认已通过工程公共头或显式 `#include <math.h>` 获得 `sqrtf()` 声明。
- 如果 IAR 链接数学库有问题，可以退化为平方比较，只在需要缩放时再处理，但优先使用 `sqrtf()`，逻辑最清楚。

## PI reset/freeze 规则

必须执行：

- Mode2 reset/disarm：reset PI，状态回 HOLD，清定时器，清输出 slew 状态。
- `HOLD -> TRACK`：reset PI。
- `TRACK -> HOLD`：reset PI。
- `TRACK/HOLD -> BRAKE`：reset PI，BRAKE 期间不更新 PI。
- `BRAKE -> HOLD/TRACK`：reset PI。

不能执行：

- HOLD 每帧 reset PI。

原因：

- HOLD 需要小积分抵消电缆外力。
- 但状态切换时必须清掉上一状态的积分，避免拖尾。

## 动态 k_flow 设计

### Mode2 通知估计器

新增接口：

```c
void Pos_Est_Set_XY_Mode(uint8_t enable_mode2, uint8_t moving, uint8_t braking);
```

映射：

```text
Mode2 未激活 -> enable_mode2=0, moving=0, braking=0
HOLD        -> enable_mode2=1, moving=0, braking=0
TRACK       -> enable_mode2=1, moving=1, braking=0
BRAKE       -> enable_mode2=1, moving=0, braking=1
```

### 估计器基础 k_flow

```text
Mode2 未激活 -> pos_est_k_flow
HOLD        -> mode2_k_flow_hover
TRACK       -> mode2_k_flow_move
BRAKE       -> mode2_k_flow_brake
```

说明：

- `pos_est_k_flow` 保留为非 Mode2 或兼容默认融合权重。
- Mode2 激活时，`mode2_k_flow_*` 优先。
- 这样不会让上一次 Mode2 状态污染其他飞行模式。

### k_flow slew

`k_flow_base` 不能硬跳。

```text
k_flow_base = slew(k_flow_base, k_flow_target, mode2_k_flow_slew_per_s * dt)
```

默认：

```text
mode2_k_flow_slew_per_s = 0.10
```

0.03 到 0.04 约 0.1 秒完成，足够平滑。

### 高度权重

```text
height_m < 0.20:
    height_weight = 0

0.20 <= height_m < 0.50:
    height_weight = (height_m - 0.20) / 0.30

height_m >= 0.50:
    height_weight = 1
```

### 光流质量门控

必须加入：

```text
flow_valid == 0:
    本帧不使用光流修正

flow invalid 连续出现:
    k_flow_eff 渐退到 0

flow valid 恢复:
    k_flow_eff 约 0.2s 渐入
```

推荐实现：

```c
flow_valid = FlowGyroDecoupler_LC302_Update50Hz(tick_1000us_cnt,
                                                lc302_data.flow_x_integral,
                                                lc302_data.flow_y_integral,
                                                lc302_data.valid);
```

说明：

- 当前 `opflow_valid = height_mm >= 200` 在高度 clamp 后基本恒真，这不够。
- 必须使用 `FlowGyroDecoupler_LC302_Update50Hz()` 的返回值作为光流有效性核心依据。
- `lc302_data.valid` 只代表 LC302 数据包有效。
- 解旋函数内部还可能判断时间间隔、数据是否可用、是否完成有效解耦。
- 光流失效时不能清零速度，只是不修正。

### 最终融合权重

```text
k_flow_eff = k_flow_base * height_weight * flow_quality_weight
```

## 光流 spike 抑制

处理链路：

```text
opflow_vel_raw
-> Median3
-> StepLimit
-> LPF1
-> InnovationLimit
-> Fusion
```

默认：

```text
Median window = 3
Step limit = 60 cm/s per 50Hz frame
Innovation limit low-alt = 50 cm/s
Innovation limit normal = 80 cm/s
```

低空定义：

```text
height_m < 0.50
```

作用：

- Median3 拦单点光流毛刺。
- StepLimit 拦连续帧突变。
- InnovationLimit 拦光流和惯性预测差太大的修正。
- 低空时光流更容易跳，innovation limit 更小。

## Mode2 Reset 必须清理的状态

`FC_Mode2_Reset()` 不只 reset PID，还必须清理完整状态：

```text
g_mode2_velx_target_raw = 0
g_mode2_vely_target_raw = 0
g_mode2_velx_target = 0
g_mode2_vely_target = 0

state = HOLD
state_timer = 0
brake_enter_confirm_timer = 0
brake_exit_timer = 0
brake_timeout_timer = 0

brake_entry_vel_x = 0
brake_entry_vel_y = 0

previous_velx_out = 0
previous_vely_out = 0

PID_Reset(&g_mode2_velx_pid)
PID_Reset(&g_mode2_vely_pid)

Pos_Est_Set_XY_Mode(0, 0, 0)
```

原因：

- 防止重新解锁或切回 Mode2 时继承旧 brake 向量。
- 防止输出 slew 从旧角度继续。
- 防止估计器继续使用上一次 Mode2 的 MOVE/BRAKE k_flow。
- 防止日志里的全局目标速度残留。

## 为什么这样会更稳

顿挫的根源：

```text
状态硬切换 + 输出无 slew + 前馈无上限 + 刹车 P 突然接管
```

改进后：

```text
目标速度先整形
前馈有限幅
状态切换有确认和最小驻留
BRAKE 有方向反转退出
所有角度输出有 slew
k_flow 切换有 slew
光流恢复有渐入
```

最终效果目标：

- 起步：前馈让速度目标一变就有角度，但目标整形避免猛冲。
- 跟随：MOVE 模式提高 k_flow，减少估计滞后。
- 松杆：BRAKE 接管，旁路 PI，直接按速度反向刹。
- 刹停：速度低或方向反了立即退出 BRAKE，HOLD 小 PI 稳住。
- 悬停：低 k_flow，小积分，光流毛刺被挡住。

## 实现时必须保持的规则

- Mode2 使用独立参数，不再复用 `mode1_*`。
- `FC_PARAMS_FLASH_VERSION` 从当前 `5` 递增。
- WiFi 参数表必须加入所有 `mode2_*` 参数。
- `pos_est_k_flow` 保留为全局默认/兼容参数，但 Mode2 激活时优先使用 `mode2_k_flow_*`。
- `g_mode2_velx_target/y_target` 表示整形后控制目标。
- `g_mode2_velx_target_raw/y_raw` 表示 RC 原始意图目标。
- BRAKE 期间不调用速度 PI。
- BRAKE 退出必须 reset 速度 PI。
- TRACK 使用前馈，HOLD/BRAKE 不使用前馈。
- 所有状态输出必须走二维限幅和 slew。
- 光流有效性必须使用 `FlowGyroDecoupler_LC302_Update50Hz()` 返回值。
- 光流异常只能降低融合权重或限制 innovation，不能突然清零速度估计。
- 高度超 TOF 量程时按最大有效高度继续估计。
- 状态切换必须有滞回、确认时间和最小驻留时间。
