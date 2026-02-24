# INAV 四旋翼定高（室内约 1.5m）气压计数据处理深度说明

## 1. 目标与范围

本文聚焦 INAV `inav/` 代码中四旋翼定高链路里，**气压计数据如何被采样、标定、滤波、融合并最终驱动油门控制**。  
重点场景是室内约 1.5m 悬停（低空、小速度、对抖动和漂移敏感）。

不覆盖：
- 机体硬件改造
- 固定翼高度控制细节
- 传感器驱动芯片寄存器级实现

---

## 2. 端到端链路（从传感器到油门）

### 2.1 调度入口

1. `TASK_BARO` 定时执行（默认 20Hz）
- `inav/src/main/fc/fc_tasks.c:563`

2. Baro 任务中先调用驱动更新，再喂给位置估计器
- `baroUpdate()`：`inav/src/main/fc/fc_tasks.c:205`
- `updatePositionEstimator_BaroTopic()`：`inav/src/main/fc/fc_tasks.c:210`

3. 主控制循环每圈都会执行位置估计与导航控制
- `updatePositionEstimator()`：`inav/src/main/fc/fc_core.c:957`
- `applyWaypointNavigationAndAltitudeHold()`：`inav/src/main/fc/fc_core.c:958`

### 2.2 高度控制数据流

`baroUpdate -> baroCalculateAltitude -> updatePositionEstimator_BaroTopic -> estimationCalculateCorrection_Z -> updateActualAltitudeAndClimbRate -> applyMulticopterAltitudeController`

核心文件：
- `inav/src/main/sensors/barometer.c`
- `inav/src/main/navigation/navigation_pos_estimator.c`
- `inav/src/main/navigation/navigation_multicopter.c`
- `inav/src/main/navigation/navigation.c`

---

## 3. 气压计数据处理细节（源码级）

## 3.1 采样状态机（UT/UP交替）

`baroUpdate()` 通过状态机交替执行温度与压强采样流程：
- `BAROMETER_NEEDS_SAMPLES`
- `BAROMETER_NEEDS_CALCULATION`
- 位置：`inav/src/main/sensors/barometer.c:254`

关键行为：
- 调用驱动的 `get_ut/start_up/get_up/start_ut/calculate`
- 输出 `baro.baroPressure` 与 `baro.baroTemperature`
- 返回下一次调度延时（微秒）

这意味着 baro 实际刷新周期不只看 `TASK_BARO` 固定周期，还受各驱动 `*_delay` 影响。

## 3.2 压强换算高度

高度与压强转换公式：
- `pressureToAltitude()`：`inav/src/main/sensors/barometer.c:292`
- `altitudeToPressure()`：`inav/src/main/sensors/barometer.c:297`

其中高度单位为厘米（cm），是后续导航高度环使用的基础单位。

## 3.3 上电零点标定（地面基准）

`baroCalculateAltitude()` 流程：
- 若标定未完成：持续把 `baroPressure` 喂给零点标定器
- 标定完成后保存：
  - `baroGroundPressure`
  - `baroGroundAltitude`
- 标定前高度固定输出 0
- 标定后输出：`pressureToAltitude(currentPressure) - baroGroundAltitude`

位置：
- `inav/src/main/sensors/barometer.c:313`
- `inav/src/main/sensors/barometer.c:319`
- `inav/src/main/sensors/barometer.c:328`

标定容差参数：
- `baro_cal_tolerance`，默认 `150` cm
- 定义：`inav/src/main/fc/settings.yaml:651`

## 3.4 温漂补偿

baro 高度在换算后叠加温度补偿：
- `applySensorTempCompensation(...)`
- 位置：`inav/src/main/sensors/barometer.c:329`

补偿逻辑实现：
- `inav/src/main/sensors/sensors.c:39`
- `baro_temp_correction` 支持固定值与 `51` 自动标定模式（5分钟或首次解锁结束）

参数定义：
- `baro_temp_correction`：`inav/src/main/fc/settings.yaml:657`

---

## 4. 进入位置估计器后的处理

## 4.1 Baro Topic 更新与一阶滤波

`updatePositionEstimator_BaroTopic()`（由 `TASK_BARO` 调用）执行：
- 调 `baroCalculateAltitude()` 取当前 baro 高度
- 应用 `initialBaroAltitudeOffset`（根据 `inav_reset_altitude` 策略）
- 写入 `posEstimator.baro.alt`
- 赋值 `posEstimator.baro.epv = inav_baro_epv`
- 若未超时则做 PT1 平滑和垂速估计

位置：
- `inav/src/main/navigation/navigation_pos_estimator.c:285`
- PT1 与 baroAltRate：`inav/src/main/navigation/navigation_pos_estimator.c:301`

超时阈值与滤波常量：
- `INAV_BARO_TIMEOUT_MS = 200`：`inav/src/main/navigation/navigation_pos_estimator_private.h:44`
- `INAV_BARO_AVERAGE_HZ = 1.0f`：`inav/src/main/navigation/navigation_pos_estimator_private.h:51`

## 4.2 高度有效性判定

`calculateCurrentValidityFlags()` 根据传感器可用性与超时设置 flags：
- `EST_BARO_VALID` 条件：有 baro + 未超时
- 位置：`inav/src/main/navigation/navigation_pos_estimator.c:472`

## 4.3 Z 轴融合修正（核心）

`estimationCalculateCorrection_Z()` 做了以下事：

1. 根据 `inav_default_alt_sensor` 决定初始 `wGps/wBaro`
- 位置：`inav/src/main/navigation/navigation_pos_estimator.c:557`

2. 当 GPS 与 Baro 同时可用时，根据高度残差动态衰减非默认传感器权重
- 位置：`inav/src/main/navigation/navigation_pos_estimator.c:561`

3. 对 Baro 进行位置与速度残差修正
- `baroAltResidual`
- `baroVelZResidual`
- `ctx->estPosCorr.z += ... * w_z_baro_p`
- `ctx->estVelCorr.z += ... * w_z_baro_v`
- 位置：`inav/src/main/navigation/navigation_pos_estimator.c:601`

4. 多旋翼近地起飞地效抑制
- 识别条件含“地面附近 + 油门高于半悬停 + AGL/Baro 条件”
- 触发时抑制 baro 的位置/速度修正，避免起飞瞬间误判
- 位置：`inav/src/main/navigation/navigation_pos_estimator.c:579`

5. 更新估计不确定度 `EPV`
- 位置：`inav/src/main/navigation/navigation_pos_estimator.c:615`

相关默认参数：
- `inav_w_z_baro_p = 0.35`：`inav/src/main/fc/settings.yaml:2471`
- `inav_w_z_baro_v = 0.35`：`inav/src/main/fc/settings.yaml:2477`
- `inav_baro_epv = 100`：`inav/src/main/fc/settings.yaml:2531`
- `inav_default_alt_sensor = GPS`：`inav/src/main/fc/settings.yaml:2537`

## 4.4 发布给导航控制层

`publishEstimatedTopic()` 会把估计高度/垂速发布给导航层：
- 位置：`inav/src/main/navigation/navigation_pos_estimator.c:808`
- 调用：`updateActualAltitudeAndClimbRate(...)`  
  `inav/src/main/navigation/navigation_pos_estimator.c:833`

导航层接收后写入：
- `actualState.abs.pos.z / vel.z`
- `actualState.agl.pos.z / vel.z`
- 并设置 `verticalPositionDataNew`
- 位置：`inav/src/main/navigation/navigation.c:2842`

---

## 5. 四旋翼定高控制如何消费该高度

## 5.1 高度目标 -> 垂速目标

多旋翼 Z 轴位置控制在 `updateAltitudeVelocityController_MC()`：
- 位置：`inav/src/main/navigation/navigation_multicopter.c:78`
- 通过 `getDesiredClimbRate()` 计算目标垂速  
  `inav/src/main/navigation/navigation_multicopter.c:80`

`getDesiredClimbRate()` 在导航层统一实现：
- 位置：`inav/src/main/navigation/navigation.c:3608`
- 多旋翼分支使用 `sqrtController` 平滑逼近目标高度

## 5.2 垂速目标 -> 油门输出

`updateAltitudeThrottleController_MC()`：
- 位置：`inav/src/main/navigation/navigation_multicopter.c:108`
- 对 `desired vel.z` 与 `actual vel.z` 做 PID
- 以 `hover_throttle` 为中心加减修正输出油门

## 5.3 遥杆对定高的作用

`adjustMulticopterAltitudeFromRCInput()`：
- 位置：`inav/src/main/navigation/navigation_multicopter.c:122`
- 油门在 `alt_hold_deadband` 之外时，转成常值爬升率
- 回到死区后，锁定当前高度（`ROC_TO_ALT_CURRENT`）

相关参数默认值：
- `nav_mc_althold_throttle = STICK`：`inav/src/main/fc/settings.yaml:2569`
- `nav_mc_hover_thr = 1300`：`inav/src/main/fc/settings.yaml:1138`
- `alt_hold_deadband = 50`：`inav/src/main/fc/settings.yaml:1807`
- `nav_mc_manual_climb_rate = 200` cm/s：`inav/src/main/fc/settings.yaml:2848`
- `nav_mc_auto_climb_rate = 500` cm/s：`inav/src/main/fc/settings.yaml:2842`

---

## 6. 室内 1.5m 场景分析结论

## 6.1 仅用气压计时的特点

在 1.5m 场景，Baro 会更容易暴露这些问题：
- 桨流/地效引起短周期压强扰动
- 板载温漂引起慢漂
- 室内空调气流引起缓慢偏置

INAV 侧对应机制：
- 低通平滑（PT1）
- 不确定度建模（`inav_baro_epv`）
- 地效期的修正抑制（起飞近地）

但结论是：**仅 Baro 可飞，但低空悬停质量上限通常受限**。

## 6.2 Baro + 测距（推荐）

对于 1.5m，若测距可用并稳定，推荐启用：
- `rangefinder_hardware` 正确配置
- `rangefinder_median_filter` 开启（抑制偶发毛刺）
- `inav_max_surface_altitude` 覆盖目标高度（默认 200cm，已覆盖 1.5m）

相关实现：
- 测距预处理与倾角补偿：`inav/src/main/sensors/rangefinder.c:240`
- AGL 可靠度状态机：`inav/src/main/navigation/navigation_pos_estimator_agl.c:89`
- AGL 融合权重参数：`inav_w_z_surface_p/v`

> 说明：导航控制实际使用 ABS 还是 AGL 由 `isTerrainFollowEnabled` 决定，见 `navGetCurrentActualPositionAndVelocity()`：`inav/src/main/navigation/navigation.c:2936`。

---

## 7. 1.5m 室内调参建议（可执行）

## 7.1 前置约束

- 先完成静置热机，确认无明显漂移再飞。
- `nav_mc_hover_thr` 要接近真实悬停油门（偏差过大将导致高度环积分负担增大）。
- 室内建议降低手动爬升率，避免高度步进过猛。

## 7.2 方案 A：仅气压计（无测距）

建议起点（在默认基础上微调）：
- `set inav_default_alt_sensor = BARO_ONLY`
- `set inav_w_z_baro_p = 0.30 ~ 0.45`
- `set inav_w_z_baro_v = 0.20 ~ 0.35`
- `set inav_baro_epv = 80 ~ 150`
- `set nav_mc_althold_throttle = HOVER`
- `set nav_mc_hover_thr = <实测悬停油门>`
- `set nav_mc_manual_climb_rate = 100 ~ 150`
- `set nav_mc_auto_climb_rate = 200 ~ 300`
- `set alt_hold_deadband = 40 ~ 60`

调参方向：
- 上下抖明显：先降 `inav_w_z_baro_v`，再降 `inav_w_z_baro_p`
- 慢漂明显：先校准 `baro_temp_correction`，再校 `hover_thr`

## 7.3 方案 B：气压计 + 测距（推荐）

建议起点：
- `set rangefinder_hardware = <对应型号>`
- `set rangefinder_median_filter = ON`
- `set inav_max_surface_altitude = 180 ~ 250`（1.5m 建议保留余量）
- `set inav_w_z_surface_p = 3.5 ~ 5.0`
- `set inav_w_z_surface_v = 5.0 ~ 8.0`
- `set inav_w_z_baro_p = 0.25 ~ 0.35`
- `set inav_w_z_baro_v = 0.20 ~ 0.30`

调参方向：
- 低空抖动：优先检查测距噪声与安装角，再小降 `w_z_surface_v`
- 高度慢漂：增加 `w_z_surface_p` 或检查地面材质反射稳定性

---

## 8. 推荐验证流程（按步骤执行）

## 8.1 地面静置（上电 5 分钟）

目标：
- 观察传感器页 Baro 高度漂移是否可接受
- 若漂移大，先处理 `baro_temp_correction`

## 8.2 低高度阶跃测试

动作：
1. 起飞到 0.8m 悬停 20s
2. 切 1.5m 悬停 30s
3. 回 1.0m 悬停 20s

观察项：
- 是否出现持续上下振荡
- 是否出现慢漂（单向爬升/下沉）
- 高度切换是否过冲

## 8.3 黑盒与调试信号

优先关注：
- `DEBUG_ALTITUDE`（估计高度/垂速、baro、gps）
  - 代码：`inav/src/main/navigation/navigation_pos_estimator.c:547`
- `DEBUG_AGL`（surface reliability、AGL质量/高度/速度）
  - 代码：`inav/src/main/navigation/navigation_pos_estimator_agl.c:190`

---

## 9. 常见症状 -> 优先处理项

1. 症状：1.5m 悬停持续高频上下抖  
优先处理：
- 降 `inav_w_z_baro_v`
- 校正 `nav_mc_hover_thr`
- 检查桨流是否直吹气压计开孔

2. 症状：不抖但慢慢爬升/下沉  
优先处理：
- 做 `baro_temp_correction`（必要时自动标定）
- 检查 `nav_mc_althold_throttle` 是否适配（建议 `HOVER`）

3. 症状：起飞离地阶段高度读数异常  
优先处理：
- 关注地效期行为（代码已有抑制逻辑）
- 避免在离地极低高度快速大油门抽升

4. 症状：加了测距后反而跳变  
优先处理：
- 开 `rangefinder_median_filter`
- 确认安装俯仰角与有效量程
- 检查 `inav_max_surface_altitude` 是否过低导致反复失效

---

## 10. 结论

- INAV 对气压计高度链路实现是完整的：采样状态机、零点标定、温漂补偿、PT1、Z轴融合和控制闭环均已具备。
- 对“室内 1.5m 定高”而言，**仅 Baro 能用但稳定性受环境影响明显**。
- 工程上推荐 `Baro + Rangefinder`，并配合 `hover_thr` 与 `manual_climb_rate` 的保守设置，可显著提升低空悬停体验与可调性。

