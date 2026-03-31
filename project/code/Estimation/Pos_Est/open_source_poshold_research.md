# 室内无 GPS 光流定点模式研究

## 1. 先看当前工程到底在干什么

你现在这套 `mode2`，本质上是一个很干净的“速度模式”。

控制链路在 [fc_mode2.c](/D:/Car_Air_Protocol/CYT4BB7_Air/project/code/FlightController/fc_mode2.c) 里很直接：

1. 遥控器横滚/俯仰摇杆先映射成目标水平速度。
2. `vel_target` 和 `-Pos_Est_vel_*_kf` 做速度 PI。
3. PI 输出直接变成 `roll_angle_target / pitch_angle_target`。
4. 再往下走姿态环和角速度环。

也就是说，你现在的摇杆语义是：

- 摇杆偏转：我要一个水平速度。
- 摇杆回中：我要零速度。

这和很多成熟飞控的“手动速度模式”其实已经很接近了。

估计器在 [Pos_Est.c](/D:/Car_Air_Protocol/CYT4BB7_Air/project/code/Estimation/Pos_Est/Pos_Est.c) 里也很清楚：

1. 光流和高度换算出 `opflow_vel_x / opflow_vel_y`。
2. 水平加速度低通后做预测项。
3. 用互补形式把预测速度和光流速度融合成 `Pos_Est_vel_x / y`。
4. 末端再做一阶低通得到 `Pos_Est_vel_x_kf / y_kf`。
5. 最后用速度积分得到 `Pos_Est_pos_x / y`。

你现在的位置更新：

```c
Pos_Est_pos_x = Pos_Est_pos_x_last + 0.5f * (Pos_Est_vel_x_last + Pos_Est_vel_x) * dt;
Pos_Est_pos_y = Pos_Est_pos_y_last + 0.5f * (Pos_Est_vel_y_last + Pos_Est_vel_y) * dt;
```

这个思路本身没错，问题不在“积分公式太土”，而在它现在还是一条裸状态：

- 什么时候位置可信，没有单独门控。
- 光流失效后，位置状态怎么处理，没有单独策略。
- 摇杆回中后，当前位置要不要抓成新的 hold 点，没有这层逻辑。

所以你现在能做到“回中刹停”，但还不是真正意义上的“位置保持”。

## 2. INAV：它不是小 flow hold，而是完整导航框架

### 2.1 摇杆在 INAV 里映射成什么

INAV 不是所有模式都把摇杆映射成同一种量。

在 [inav/docs/Navigation.md](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/docs/Navigation.md) 里，`NAV ALTHOLD` 和 `NAV POSHOLD` 的语义写得很清楚：

- `NAV ALTHOLD`：
  - 油门不是直接推力。
  - 油门映射成爬升率。
  - 松杆后保持当前高度。
- `NAV POSHOLD`：
  - `nav_user_control_mode = ATTI` 时，右杆更像 Angle 模式，摇杆控制倾角，松杆后记录并保持新位置。
  - `nav_user_control_mode = CRUISE` 时，右杆语义上控制水平速度。

这个设置在 [inav/src/main/fc/settings.yaml](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/fc/settings.yaml) 里也能看到：

- `nav_user_control_mode`
- `ATTI`
- `CRUISE`

### 2.2 INAV 真代码里怎么把摇杆和位置环搭起来

这个地方最值得看的是 [inav/src/main/navigation/navigation_multicopter.c](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/navigation/navigation_multicopter.c) 里的 `adjustMulticopterPositionFromRCInput()`。

如果当前是 `NAV_GPS_CRUISE`，INAV 不是直接把“摇杆速度”塞到最终控制器里，而是做了一个很巧但很实用的变换：

1. 先把摇杆变成机体系速度命令 `rcVelX / rcVelY`。
2. 再按当前 yaw 把它转到地理系 `neuVelX / neuVelY`。
3. 然后构造一个新的位置目标：

```c
desired_pos = current_pos + commanded_vel / pos_kP
```

源码就是这两句：

- `posControl.desiredState.pos.x = current_pos.x + neuVelX / pos_kP`
- `posControl.desiredState.pos.y = current_pos.y + neuVelY / pos_kP`

这玩意很值钱，因为它说明：

- 摇杆偏转时，飞手主观上感觉自己在“控速度”。
- 但内部实现上，控制器其实还是围着“位置目标”在干活。

这样做的好处很直接：

- 摇杆一回中，位置目标就不再继续移动。
- 位置 P 环自然会把目标速度拉回 0。
- 飞机会停在“松杆时的新位置”。

这比“摇杆直接映射位置”更顺手，也比“永远只控速度”更接近真定点。

### 2.3 INAV 的位置环和速度环怎么串

INAV 的控制链条一点都不玄学，在 [Navigation.md](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/docs/Navigation.md) 就直接写了：

- `POS`：位置误差到目标速度
- `POSR`：速度误差到目标加速度

真代码在 [navigation_multicopter.c](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/navigation/navigation_multicopter.c) 里：

1. `updatePositionVelocityController_MC()`
   - `posError = desired_pos - actual_pos`
   - `desired_vel = posError * pos_kP`
2. `updatePositionAccelController_MC()`
   - `velError = desired_vel - actual_vel`
   - `desired_accel = POSR PID(velError)`
3. `updatePositionAccelController_MC()` 后半段
   - 把目标加速度转成 `desiredPitch / desiredRoll`
   - 本质就是 `atan2(accel, g)`

这就是很标准的：

- 位置环：`pos -> vel`
- 速度环：`vel -> accel`
- 姿态映射：`accel -> angle`

### 2.4 INAV 进 PosHold 时不是“原地冻结”，而是先算刹车距离

这个设计很像老司机思路。

在 [inav/src/main/navigation/navigation.c](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/navigation/navigation.c) 进入 `NAV_STATE_POSHOLD_3D_INITIALIZE` 时，会去准备位置控制器。

而 [inav/src/main/navigation/navigation_multicopter.c](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/navigation/navigation_multicopter.c) 的 `calculateMulticopterInitialHoldPosition()` 不会简单地把当前位置当成 hold 点，而是：

```c
hold_pos = current_pos + current_vel * posDecelerationTime
```

说人话就是：

- 飞机本来就在动。
- 你这时候切进 PosHold。
- 控制器先估一把“按当前减速度大概会停在哪”。
- 把那个点当成初始 hold 点。

这样切模式不会一下子反向猛拽。

### 2.5 INAV 的位置估计怎么来

如果不展开复杂 EKF，用人话总结就是：

- 主状态用 IMU 做预测。
- 光流和高度给水平速度修正。
- 开了允许 dead reckoning 以后，光流自己还能再积分出一份“辅助位置轨迹”来拉主位置状态。

核心代码在：

- [inav/src/main/navigation/navigation_pos_estimator.c](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/navigation/navigation_pos_estimator.c)
- [inav/src/main/navigation/navigation_pos_estimator_flow.c](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/navigation/navigation_pos_estimator_flow.c)

`estimationPredict()` 做的事情很朴素：

1. 先用当前速度推位置。
2. 如果加速度可信，再补 `0.5 * a * dt^2`。
3. 再用加速度更新速度。

XY 预测步就是：

- `pos += vel * dt`
- `pos += accel * dt^2 / 2`
- `vel += accel * dt`

光流修正在 `estimationCalculateCorrection_XY_FLOW()` 里：

1. 先用 `flowRate - bodyRate` 去掉自转。
2. 再乘当前高度，把角速度变成线速度。
3. 再从 body frame 转到 earth frame。
4. 这份 `flowVel` 先拿来修主速度状态。

如果 `allow_dead_reckoning` 开了，INAV 还会维护 `flowCoordinates`：

1. 先把 `flowVel` 自己积分一遍。
2. 形成一条“光流自己推出来的位置轨迹”。
3. 再用这条轨迹和主位置状态的残差，去修正主位置。

这一步很关键。它说明 INAV 不是简单地：

- “先融合速度”
- “再裸积分位置”

而是把“光流自积分位置”也作为一个独立参考量在用。

### 2.6 INAV 在“室内无 GPS 光流定点”这件事上有文档冲突

这点必须直接说，不然容易误导人。

[inav/docs/Navigation.md](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/docs/Navigation.md) 还写着：

- `POSHOLD requires GPS`

但 [inav/docs/Rangefinder.md](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/docs/Rangefinder.md) 又写着：

- `rangefinder + optical flow` 可以做 `GPS-free position hold indoors`

源码激活条件在 [inav/src/main/navigation/navigation.c](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/navigation/navigation.c) 的 `canActivatePosHoldMode()`：

- `estPosStatus >= EST_USABLE`
- `estVelStatus == EST_TRUSTED`
- `estHeadingStatus >= EST_USABLE`

这里没有硬编码“必须有 GPS”。

所以更靠谱的判断是：

- 按源码，INAV 的 PosHold 激活依赖的是“位置/速度/航向状态是否可用”。
- 光流 + 测距 + 姿态/航向状态如果能把这些状态喂到可用等级，室内无 GPS 也有机会工作。
- 但它走的是完整导航框架，不是一个小而轻的 flow hold 特性。

### 2.7 对你最有价值的 INAV 启发

INAV 真正值得你学的不是它那一大坨导航状态机，而是这两件事：

1. 摇杆给速度意图时，可以反推成“新的位置目标”，这样松杆自然进入 hold。
2. 位置状态不要只有一条裸积分主状态，至少要补门控、重抓、冻结和漂移管理。

## 3. PX4：最像“正规升级版 mode2”的路线

### 3.1 PX4 Position 模式的手感定义

PX4 官方文档：

- Position Mode: <https://docs.px4.io/main/en/flight_modes_mc/position>
- Optical Flow: <https://docs.px4.io/main/en/sensor/optical_flow.html>

PX4 对 Position Mode 的描述非常直接：

- Roll/Pitch 摇杆控制地面系水平运动意图。
- 摇杆回中后，飞机会主动刹车、回平，并锁在当前位置。

从手感上看，它比你现在的 `mode2` 就多了一层：

- 你现在：摇杆回中 = 目标速度变 0
- PX4：摇杆回中 = 目标速度变 0，并且当前位置被抓成新的 hold 点

### 3.2 PX4 的摇杆和位置锁定是怎么搭起来的

源码重点看：

- `FlightTaskManualPosition.cpp`
  - <https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/flight_mode_manager/tasks/ManualPosition/FlightTaskManualPosition.cpp>
- `PositionControl.cpp`
  - <https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/PositionControl/PositionControl.cpp>

最值得看的逻辑是：

1. `activate()`
   - 一进模式先把位置目标设成当前位置。
   - 速度目标清零。
2. `_scaleSticks()`
   - 摇杆缩放成水平运动命令。
   - 受 `MPC_VEL_MANUAL` 等参数限制。
3. `_updateXYlock()`
   - 摇杆偏转时，位置锁定让开。
   - 摇杆回中且速度足够小时，抓当前位置作为新的位置目标。

这就是 PX4 最核心的“位置模式手感来源”：

- 摇杆偏转：你觉得自己在手动飞。
- 摇杆回中：控制器自动接管当前位置保持。

### 3.3 PX4 的位置环和速度环怎么串

在 `PositionControl.cpp` 里，结构很标准：

1. `_positionControl()`
   - 位置误差先变成目标速度
   - 再叠加上层给的速度意图
2. `_velocityControl()`
   - 速度误差再变成目标加速度/推力
3. 再由推力和姿态控制器往下走

说白了就是：

- `pos_error -> vel_sp`
- `vel_error -> accel_sp`
- `accel/thrust -> angle`

这个骨架非常适合你这种工程，因为你已经有现成的速度环和姿态环了。

### 3.4 PX4 的位置估计对你意味着什么

PX4 控制器自己不负责“算位置”，它只吃 estimator 给的 `vehicle_local_position`。

对你这个工程来说，真正值得学的不是 PX4 复杂的 estimator，而是它对控制器的接口设计：

- 控制器只认“当前位置、当前速度、当前位置目标、当前速度目标”。
- 位置状态到底是 EKF 算出来，还是你自己用光流速度积分出来，控制器并不关心。

所以如果你不想上复杂 EKF，完全可以先学 PX4 的控制接口，而不是学它完整估计器。

### 3.5 对你最有价值的 PX4 启发

PX4 对你最有价值的就一句话：

> 摇杆偏转时按速度飞，摇杆回中时抓当前位置做位置目标。

这句话落到你工程里，已经够做出一个很好用的第一版定点模式了。

## 4. ArduPilot：两条路，一条是真 hold，一条是高级刹车

### 4.1 FlowHold：更像“高级刹停模式”

文档：

- <https://ardupilot.org/copter/docs/flowhold-mode.html>

本地代码：

- [mode_flowhold.cpp](/D:/Car_Air_Protocol/ardupilot/ArduCopter/mode_flowhold.cpp)

`ModeFlowHold::flowhold_flow_to_angle()` 的逻辑特别直白：

1. `flowRate - bodyRate` 去旋转。
2. 对 flow 做低通。
3. 乘高度，把光流角速度变成线速度。
4. 转到 earth frame。
5. 送进 `flow_pi_xy`。
6. 摇杆有输入时，允许直接飞。
7. 摇杆一回中，进入 braking，用 flow 速度把机体刹停。

这玩意本质上不是“真位置环”，而是：

- 有杆：飞手主导
- 松杆：控制器用光流速度把飞机拉回零速

所以它更像“高级刹车 + 高级止漂”，不是长期可靠的位置锁定。

ArduPilot 官方文档自己也说了：

- FlowHold 不需要 GPS，也不需要 LiDAR。
- 但很多机体上会抖，效果不如“rangefinder + regular Loiter”。

### 4.2 Loiter：更像完整位置保持

本地代码：

- [AC_Loiter.cpp](/D:/Car_Air_Protocol/ardupilot/libraries/AC_WPNav/AC_Loiter.cpp)
- [AC_PosControl.cpp](/D:/Car_Air_Protocol/ardupilot/libraries/AC_AttitudeControl/AC_PosControl.cpp)

`AC_Loiter::set_pilot_desired_acceleration_rad()` 说明，ArduPilot Loiter 更倾向于：

- 摇杆先表达倾角/加速度意图
- 再由位置控制器做平滑和 braking

而 `AC_PosControl::NE_update_controller()` 的结构和 PX4 / INAV 非常像：

1. 位置 P 环先给目标速度
2. 速度 PID 再给目标加速度
3. 最后限制倾角和加速度

所以：

- `FlowHold` 更像简单好用的“停下来”
- `Loiter` 更像完整好用的“停在这”

### 4.3 对你最有价值的 ArduPilot 启发

如果你只是想把当前 `mode2` 做成更强的“松杆就停”，那 `FlowHold` 很值得学。

如果你想做真正的位置保持，那还是要回到 `Loiter / PX4 Position / INAV POSHOLD` 这类“位置环 + 速度环”的路线。

## 5. 三家开源飞控，真正该学哪一条

| 系统 | 摇杆偏转时 | 摇杆回中时 | 位置从哪来 | 更像什么 | 适合你当前工程吗 |
| --- | --- | --- | --- | --- | --- |
| INAV POSHOLD / CRUISE | 速度意图，经位置目标重构 | 抓新位置并保持 | IMU 预测 + flow/rangefinder 修正，可选 flow 自积分轨迹 | 完整导航框架 | 思路很值钱，但整套太重 |
| PX4 Position | 速度/加速度意图 | 主动刹车并锁当前位置 | estimator 给 local position | 正规版室内位置模式 | 最值得学 |
| ArduPilot FlowHold | 倾角/直接飞行 | 光流 PI + braking 拉回零速 | 主要依赖 flow 速度和高度估计 | 高级刹车模式 | 很适合做简化版 |
| ArduPilot Loiter | 加速度意图 + 位置控制 | 持续保持位置 | 位置控制器消费完整位置状态 | 完整位置保持 | 结构上可学，但不如 PX4 顺手 |

一句话总结：

- 如果你要“最简单但立刻好用”，学 `FlowHold`。
- 如果你要“真定点而且结构不难抄”，学 `PX4 Position`。
- 如果你要看“完整导航框架如何把 flow 也并进位置估计”，学 `INAV`。

## 6. 对当前工程，最值得做的最小路线

这一节只讲“怎么在你当前代码上最小代价落地”，不讲大而全架构。

### 6.1 推荐路线：PX4 / INAV-CRUISE Lite

推荐路线不是“上 EKF”，而是下面这套最小改法：

1. 摇杆偏转时：
   - 继续沿用现在的 `mode2`
   - 摇杆映射成目标水平速度
   - 位置环不工作
2. 摇杆回中时：
   - 如果当前水平速度已经够小
   - 把当前 `Pos_Est_pos_x / y` 抓成 `pos_target_x / y`
   - 开一个很薄的位置 P 环：

```c
vel_target_x = (pos_target_x - Pos_Est_pos_x) * pos_x_kp;
vel_target_y = (pos_target_y - Pos_Est_pos_y) * pos_y_kp;
```

3. 然后：
   - 这个 `vel_target` 再走你现有速度环
   - 现有姿态环和角速度环完全不用重写

这条路线的优点是：

- 改动小
- 手感连续
- 代码结构和你现在的 `mode2` 最接近

### 6.2 位置状态怎么做，才不至于一开位置环就漂

你现在最需要补的不是公式，而是策略。

第一版就补下面 4 件事：

1. 位置更新门控
   - 只有在 `squal`、高度、姿态/航向都可信时，才更新 `Pos_Est_pos_x / y`
2. 位置冻结
   - 光流失效后，不再继续用裸积分把位置越推越远
3. hold 点重抓
   - 摇杆从有输入回到中位，且速度落到阈值内时，重抓当前位置
4. 位置状态重置
   - 起飞、降落、明显光流失效恢复后，允许显式重置位置状态

说白了，第一版最需要的是“什么时候信位置”，而不是“位置公式多复杂”。

### 6.3 第一版不需要上的东西

为了简单好用，下面这些第一版都不必上：

- 复杂 EKF
- jerk limited 轨迹
- 单独的 flow 自积分第二状态
- 基于 drag 的模型
- 复杂 braking 状态机

先把“松杆抓点 + 薄位置环 + 门控/冻结/重置”做对，效果就会和纯速度模式拉开明显差距。

### 6.4 如果你暂时不想上位置环，最简单的增强路线

如果你下一步还不想碰位置环，那就走 `FlowHold` 路线：

- 摇杆偏转：继续人工飞
- 摇杆回中：不要只让 `vel_target = 0`
- 而是加一个“刹车型零速控制”

这条路能显著提升“松杆就停”的感觉，但它不等于真定点。

## 7. 最建议先看的源码

### 当前工程

- [project/code/FlightController/fc_mode2.c](/D:/Car_Air_Protocol/CYT4BB7_Air/project/code/FlightController/fc_mode2.c)
- [project/code/Estimation/Pos_Est/Pos_Est.c](/D:/Car_Air_Protocol/CYT4BB7_Air/project/code/Estimation/Pos_Est/Pos_Est.c)

### INAV

- [inav/src/main/navigation/navigation_multicopter.c](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/navigation/navigation_multicopter.c)
- [inav/src/main/navigation/navigation.c](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/navigation/navigation.c)
- [inav/src/main/navigation/navigation_pos_estimator.c](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/navigation/navigation_pos_estimator.c)
- [inav/src/main/navigation/navigation_pos_estimator_flow.c](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/navigation/navigation_pos_estimator_flow.c)
- [inav/src/main/fc/settings.yaml](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/src/main/fc/settings.yaml)
- [inav/docs/Navigation.md](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/docs/Navigation.md)
- [inav/docs/Rangefinder.md](/D:/Car_Air_Protocol/CYT4BB7_Air/inav/docs/Rangefinder.md)

### PX4

- Position Mode 文档：
  - <https://docs.px4.io/main/en/flight_modes_mc/position>
- Optical Flow 文档：
  - <https://docs.px4.io/main/en/sensor/optical_flow.html>
- 手动位置任务：
  - <https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/flight_mode_manager/tasks/ManualPosition/FlightTaskManualPosition.cpp>
- 多旋翼位置控制器：
  - <https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/PositionControl/PositionControl.cpp>

### ArduPilot

- FlowHold 文档：
  - <https://ardupilot.org/copter/docs/flowhold-mode.html>
- [ArduCopter/mode_flowhold.cpp](/D:/Car_Air_Protocol/ardupilot/ArduCopter/mode_flowhold.cpp)
- [libraries/AC_WPNav/AC_Loiter.cpp](/D:/Car_Air_Protocol/ardupilot/libraries/AC_WPNav/AC_Loiter.cpp)
- [libraries/AC_AttitudeControl/AC_PosControl.cpp](/D:/Car_Air_Protocol/ardupilot/libraries/AC_AttitudeControl/AC_PosControl.cpp)

## 8. 最后的结论

如果只看“最简单好用”，那成熟飞控给出的答案其实很一致：

- 真定点一定离不开位置环。
- 但第一版的位置环完全可以很薄。
- 摇杆偏转时，让飞手继续控速度或控加速度。
- 摇杆回中时，自动抓当前位置做 hold 点。
- 位置状态不要裸跑，必须有门控、冻结和重抓。

对你当前工程来说，最值得抄的不是复杂 EKF，而是：

1. `PX4` 的“零杆抓当前位置”
2. `INAV` 的“速度意图反推位置目标”
3. `ArduPilot FlowHold` 的“先把松杆刹停做好”

这三件事抄明白了，你这套 `mode2` 就能从“纯速度模式”升级到“室内好用的定点模式”。
