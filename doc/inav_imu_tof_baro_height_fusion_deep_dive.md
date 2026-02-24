# INAV 深度源码解析：ICM42688 + VL53L1X + BMP388 高度融合

## 1. 结论先说

你的组合在 INAV 中是这样落地的：

1. `ICM42688` 实际复用 `ICM42605` 驱动框架（同寄存器族），输出姿态与机体系加速度，作为高度预测项（惯导预测）。
2. `BMP388` 提供绝对高度基准（相对起飞参考点），通过压力换算 + 温漂补偿进入 Z 轴估计修正。
3. `VL53L1X` 提供近地 AGL（离地高度）约束，通过可靠度状态机进入 AGL 子估计器。

关键现实点：

1. 在 INAV 里，**高度主控制默认用 ABS 高度（baro/gps 融合）**。  
2. `VL53L1X` 的 AGL 融合会一直算，但只有地形跟随（`BOXSURFACE`）时，控制器才直接用 AGL 高度闭环。  
3. 所以你想要“1.5m 室内更准高度”，核心不是只接上 ToF，而是要让模式和参数让 ToF 真正进入控制闭环。

---

## 2. 三个传感器在 INAV 的真实入口

## 2.1 ICM42688（IMU）入口

INAV 用 `accgyro_icm42605.c` 同时兼容 ICM42605/ICM42688P：

- WHO_AM_I 判断：
  - `ICM42605_WHO_AM_I_CONST`
  - `ICM42688P_WHO_AM_I_CONST`
  - 代码：`inav/src/main/drivers/accgyro/accgyro_icm42605.c:291`
- 检测成功后：
  - 加速度读函数 `icm42605AccRead()`：`inav/src/main/drivers/accgyro/accgyro_icm42605.c:160`
  - 陀螺读函数 `icm42605GyroRead()`：`inav/src/main/drivers/accgyro/accgyro_icm42605.c:309`
  - 初始化采样/LPF/AAF：`icm42605AccAndGyroInit()`：`inav/src/main/drivers/accgyro/accgyro_icm42605.c:204`

板级注册一般使用 `DEVHW_ICM42605`：

- `BUSDEV_REGISTER_SPI(... DEVHW_ICM42605 ...)`
- `inav/src/main/target/common_hardware.c:56`

## 2.2 BMP388（气压计）入口

检测与初始化：

- `bmp388Detect()`：`inav/src/main/drivers/barometer/barometer_bmp388.c:328`
- 读取校准参数（NVM trimming）：`inav/src/main/drivers/barometer/barometer_bmp388.c:342`
- 过采样配置：
  - 压力 `8x`
  - 温度 `1x`
  - 代码：`inav/src/main/drivers/barometer/barometer_bmp388.c:353`

采样与补偿：

- 读取原始 UP/UT：`bmp388GetUP()`：`inav/src/main/drivers/barometer/barometer_bmp388.c:201`
- 温度补偿：`bmp388CompensateTemperature()`：`inav/src/main/drivers/barometer/barometer_bmp388.c:217`
- 压力补偿：`bmp388CompensatePressure()`：`inav/src/main/drivers/barometer/barometer_bmp388.c:240`
- 输出 `pressure`/`temperature`：`bmp388Calculate()`：`inav/src/main/drivers/barometer/barometer_bmp388.c:279`

## 2.3 VL53L1X（测距）入口

检测与初始化：

- `vl53l1xDetect()`：`inav/src/main/drivers/rangefinder/rangefinder_vl53l1x.c:1672`
- `VL53L1X_SensorInit()`：`inav/src/main/drivers/rangefinder/rangefinder_vl53l1x.c:1610`
- 模式与时序：
  - DistanceMode = long(2)
  - TimingBudget = 33ms
  - InterMeasurement = 40ms
  - 代码：`inav/src/main/drivers/rangefinder/rangefinder_vl53l1x.c:1612`

输出：

- 新数据读取后单位换算 `mm -> cm`：`Distance / 10`
- 超过 `VL53L1X_MAX_RANGE_CM(300)` 视作超量程
- 代码：`inav/src/main/drivers/rangefinder/rangefinder_vl53l1x.c:1631`

---

## 3. 运行时调度和数据链路

## 3.1 任务层

- `TASK_BARO` 默认 20Hz：`inav/src/main/fc/fc_tasks.c:563`
- `TASK_RANGEFINDER` 默认 70ms，但 VL53L1X 检测后重设为 40ms：  
  - 默认任务：`inav/src/main/fc/fc_tasks.c:581`
  - 重设：`inav/src/main/sensors/rangefinder.c:113`
- 主循环每圈执行：
  - `updatePositionEstimator()`：`inav/src/main/fc/fc_core.c:957`
  - `applyWaypointNavigationAndAltitudeHold()`：`inav/src/main/fc/fc_core.c:958`

## 3.2 Baro 链路

1. 驱动采样状态机 `baroUpdate()`（UT/UP 交替）：`inav/src/main/sensors/barometer.c:254`
2. 压力换高度 `pressureToAltitude()`：`inav/src/main/sensors/barometer.c:292`
3. 零点标定后计算相对高度 `baroCalculateAltitude()`：`inav/src/main/sensors/barometer.c:313`
4. 写入估计器 `updatePositionEstimator_BaroTopic()`：`inav/src/main/navigation/navigation_pos_estimator.c:285`
5. Baro PT1 和 baro 垂速：`inav/src/main/navigation/navigation_pos_estimator.c:301`

## 3.3 ToF 链路

1. `rangefinderUpdate()` + `rangefinderProcess(cosTilt)`：`inav/src/main/sensors/rangefinder.c:228`
2. 中值滤波（可选）+ 倾角补偿：`inav/src/main/sensors/rangefinder.c:254`
3. 写入估计器 `updatePositionEstimator_SurfaceTopic()`：`inav/src/main/navigation/navigation_pos_estimator_agl.c:49`

## 3.4 IMU 链路（和高度最相关部分）

1. `imuMeasuredAccelBF` 从 ICM 驱动读入：`inav/src/main/flight/imu.c:900`
2. 估计器将机体系加速度转地理系 `Body(FRD) -> Earth(NEU)`：`inav/src/main/navigation/navigation_pos_estimator.c:397`
3. 去重力并叠加偏置估计：`inav/src/main/navigation/navigation_pos_estimator.c:430`
4. 用于 Z 预测：`z += v*dt + 0.5*a*dt^2`, `v += a*dt`  
   `inav/src/main/navigation/navigation_pos_estimator.c:523`

---

## 4. 高度融合核心算法（INAV 真实机制）

## 4.1 Z 预测（IMU）

`updateEstimatedTopic()` 内先预测：

- `posEstimator.est.pos.z += posEstimator.est.vel.z * dt`
- `posEstimator.est.pos.z += accelNEU.z * dt^2 / 2`
- `posEstimator.est.vel.z += accelNEU.z * dt`

代码：`inav/src/main/navigation/navigation_pos_estimator.c:521`

## 4.2 Z 修正（Baro/GPS 残差）

核心在 `estimationCalculateCorrection_Z()`：

- Baro 残差：
  - `baroAltResidual = (baroAlt - estZ)`（含权重和地效逻辑）
  - `baroVelResidual = (baroRate - estVz)`
- 修正注入：
  - `estPosCorr.z += baroAltResidual * w_z_baro_p * dt`
  - `estVelCorr.z += baroVelResidual * w_z_baro_v * dt`

代码：`inav/src/main/navigation/navigation_pos_estimator.c:545`

地效抑制（近地起飞）也在这里：

- 触发时机：多旋翼、离地很低、推力上来时
- 行为：暂时抑制 baro 的位置/速度修正，避免起飞瞬间气垫效应误导高度估计
- 代码：`inav/src/main/navigation/navigation_pos_estimator.c:579`

## 4.3 AGL 子估计器（ToF + IMU + 全局高度）

`estimationCalculateAGL()` 内部：

1. 维护 `surface.reliability`（可靠度一阶模型）
- 代码：`inav/src/main/navigation/navigation_pos_estimator_agl.c:72`
2. 依据阈值切换质量状态 `LOW/MID/HIGH`
- 代码：`inav/src/main/navigation/navigation_pos_estimator_agl.c:95`
3. AGL 预测同样用 IMU 积分
- 代码：`inav/src/main/navigation/navigation_pos_estimator_agl.c:149`
4. `HIGH` 时强信任 ToF，`MID` 时 ToF 与全局高度混合，`LOW` 时退化为全局高度偏移
- 代码：`inav/src/main/navigation/navigation_pos_estimator_agl.c:155`

## 4.4 哪个高度真正进入控制器

控制器取值由 `navGetCurrentActualPositionAndVelocity()` 决定：

- `isTerrainFollowEnabled == false` -> 用 `ABS` 高度
- `isTerrainFollowEnabled == true` -> 用 `AGL` 高度

代码：`inav/src/main/navigation/navigation.c:2936`

地形跟随启用条件（多旋翼）：

- `BOXSURFACE` 打开时请求地形跟随
- 代码：`inav/src/main/navigation/navigation.c:1269`

这就是你场景最关键的一条：  
**接入 VL53L1X 不等于默认就拿它做主高度闭环，必须通过模式和参数让它进入闭环。**

---

## 5. 你的组合（室内 1.5m）对应的源码级建议

## 5.1 传感器层

1. ICM42688 使用 `accgyro_icm42605` 路径是正常现象，不是识别错误。
2. VL53L1X 驱动内是 long mode + 40ms 间隔 + 300cm上限，1.5m 在有效区间内。
3. BMP388 走 forced measurement，每次任务周期发起并读取一帧，配合上层 PT1 稳定高度。

## 5.2 融合与模式层

1. 室内无 GPS 时，Z 修正本质上是 `IMU预测 + Baro修正`；ToF 主要进入 AGL 分支。
2. 若要让 1.5m 高度“贴地更准”，建议在 ALTHOLD/POSHOLD 时使用 `BOXSURFACE`。
3. 关注参数：
   - `inav_max_surface_altitude`
   - `inav_w_z_surface_p / inav_w_z_surface_v`
   - `rangefinder_median_filter`
   - `inav_w_z_baro_p / inav_w_z_baro_v`
   - `inav_baro_epv`

参数默认定义：

- `inav_max_surface_altitude=200`：`inav/src/main/fc/settings.yaml:2441`
- `inav_w_z_surface_p=3.5`：`inav/src/main/fc/settings.yaml:2447`
- `inav_w_z_surface_v=6.1`：`inav/src/main/fc/settings.yaml:2453`
- `inav_w_z_baro_p=0.35`：`inav/src/main/fc/settings.yaml:2471`
- `inav_w_z_baro_v=0.35`：`inav/src/main/fc/settings.yaml:2477`
- `inav_baro_epv=100`：`inav/src/main/fc/settings.yaml:2531`
- `rangefinder_median_filter=OFF`：`inav/src/main/fc/settings.yaml:528`

---

## 6. 核心算法最小示例代码（C，可独立编译）

说明：

1. 这段代码不是 INAV 源码拷贝，而是把 INAV 的高度融合核心机制抽成最小模型。
2. 保留了你场景最关键的机制：IMU 预测、Baro 修正、ToF 可靠度状态机、地效抑制、AGL 混合。

```c
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

typedef enum {
    AGL_LOW = 0,
    AGL_MID = 1,
    AGL_HIGH = 2
} agl_quality_e;

typedef struct {
    // 全局高度估计（近似 INAV est.pos.z / est.vel.z）
    float z_cm;
    float vz_cms;

    // AGL 估计（近似 INAV est.aglAlt / est.aglVel）
    float agl_cm;
    float agl_vz_cms;
    float agl_offset_cm;
    agl_quality_e agl_q;

    // 传感器状态
    float baro_ground_cm;      // 上电/解锁前基准
    float baro_lpf_cm;
    float baro_prev_cm;
    float surface_lpf_cm;
    float surface_rel;         // 0~1

    bool initialized;
} alt_estimator_t;

typedef struct {
    // INAV 同类参数（默认值取 INAV）
    float w_z_baro_p;      // 0.35
    float w_z_baro_v;      // 0.35
    float w_z_surface_p;   // 3.5
    float w_z_surface_v;   // 6.1
    float max_surface_alt_cm; // 200

    float baro_lpf_hz;     // INAV baro avg 对应低速平滑，示例用 1Hz
    float surface_lpf_hz;  // 示例用 1Hz
} alt_params_t;

static float constrainf_local(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static float pt1_apply(float prev, float input, float dt, float cutoff_hz) {
    if (cutoff_hz <= 0.0f || dt <= 0.0f) return input;
    const float rc = 1.0f / (2.0f * 3.1415926f * cutoff_hz);
    const float a = dt / (dt + rc);
    return prev + a * (input - prev);
}

static float bell_curve(float x, float sigma) {
    if (sigma <= 0.0f) return 1.0f;
    const float t = x / sigma;
    return expf(-0.5f * t * t);
}

void alt_step(
    alt_estimator_t *s,
    const alt_params_t *p,
    float dt_s,
    float imu_acc_z_neu_cmss,      // 机体加速度转 NEU 后并去重力
    bool baro_valid,
    float baro_alt_cm,             // baro topic（相对高度）
    bool tof_valid,
    float tof_alt_cm,              // 倾角修正后的测距高度
    bool armed,
    bool throttle_above_mid_hover
) {
    if (!s->initialized) {
        s->baro_lpf_cm = baro_alt_cm;
        s->baro_prev_cm = baro_alt_cm;
        s->baro_ground_cm = baro_alt_cm;
        s->surface_lpf_cm = tof_alt_cm > 0 ? tof_alt_cm : 0.0f;
        s->initialized = true;
    }

    // 1) IMU 预测（对应 INAV 预测步）
    s->z_cm += s->vz_cms * dt_s + 0.5f * imu_acc_z_neu_cmss * dt_s * dt_s;
    s->vz_cms += imu_acc_z_neu_cmss * dt_s;

    // 2) Baro 更新与修正
    float baro_rate_cms = 0.0f;
    if (baro_valid) {
        s->baro_lpf_cm = pt1_apply(s->baro_lpf_cm, baro_alt_cm, dt_s, p->baro_lpf_hz);
        baro_rate_cms = (s->baro_lpf_cm - s->baro_prev_cm) / fmaxf(dt_s, 1e-4f);
        s->baro_prev_cm = s->baro_lpf_cm;
    }

    if (!armed && baro_valid) {
        // 对应 INAV 解锁前近地基准更新
        s->baro_ground_cm = s->baro_lpf_cm;
    }

    // 地效抑制近似：离地很低 + 推力上来时，减弱 baro 修正
    const bool air_cushion =
        armed && ((tof_valid && tof_alt_cm < 20.0f) || (s->baro_lpf_cm < s->baro_ground_cm + 20.0f));

    if (baro_valid) {
        float baro_ref = air_cushion ? s->baro_ground_cm : s->baro_lpf_cm;
        float res_p = baro_ref - s->z_cm;
        if (!(air_cushion && throttle_above_mid_hover)) {
            s->z_cm += res_p * p->w_z_baro_p * dt_s;
        }
        if (!air_cushion) {
            float res_v = baro_rate_cms - s->vz_cms;
            s->vz_cms += res_v * p->w_z_baro_v * dt_s;
        }
    }

    // 3) Surface(ToF) 可靠度与 AGL 更新
    float new_rel_measure = 0.0f;
    bool surface_in_range = false;
    if (tof_valid && tof_alt_cm >= 0.0f && tof_alt_cm <= p->max_surface_alt_cm) {
        new_rel_measure = 1.0f;
        surface_in_range = true;
    }

    // 对应 INAV 的 reliability RC 常量 0.47802
    const float rel_alpha = dt_s / (dt_s + 0.47802f);
    s->surface_rel = s->surface_rel * (1.0f - rel_alpha) + new_rel_measure * rel_alpha;
    s->surface_rel = constrainf_local(s->surface_rel, 0.0f, 1.0f);

    if (surface_in_range) {
        s->surface_lpf_cm = pt1_apply(s->surface_lpf_cm, tof_alt_cm, dt_s, p->surface_lpf_hz);
    }

    // 质量状态机（同 INAV 阈值）
    if (s->surface_rel >= 0.75f) s->agl_q = AGL_HIGH;
    else if (s->surface_rel >= 0.33f) s->agl_q = AGL_MID;
    else s->agl_q = AGL_LOW;

    // AGL 预测
    s->agl_cm += s->agl_vz_cms * dt_s + 0.5f * imu_acc_z_neu_cmss * dt_s * dt_s;
    s->agl_vz_cms += imu_acc_z_neu_cmss * dt_s;

    if (s->agl_q == AGL_HIGH && surface_in_range) {
        float res = tof_alt_cm - s->agl_cm;
        float k = 0.1f + 0.9f * bell_curve(res, 75.0f);
        s->agl_cm += res * p->w_z_surface_p * k * s->surface_rel * dt_s;
        s->agl_vz_cms += res * p->w_z_surface_v * k * k * s->surface_rel * s->surface_rel * dt_s;
        s->agl_offset_cm = s->z_cm - s->surface_lpf_cm;
    } else if (s->agl_q == AGL_MID && surface_in_range) {
        float res_surface = tof_alt_cm - s->agl_cm;
        float res_est = (s->z_cm - s->agl_offset_cm) - s->agl_cm;
        float ws = (0.1f + 0.9f * bell_curve(res_surface, 50.0f)) * s->surface_rel;
        float mixed = res_surface * ws + res_est * (1.0f - ws);
        s->agl_cm += mixed * p->w_z_surface_p * dt_s;
        s->agl_vz_cms += mixed * p->w_z_surface_v * dt_s;
    } else {
        // 低质量时退化到全局高度偏移
        s->agl_cm = s->z_cm - s->agl_offset_cm;
        s->agl_vz_cms = s->vz_cms;
    }
}

int main(void) {
    alt_estimator_t s = {0};
    alt_params_t p = {
        .w_z_baro_p = 0.35f,
        .w_z_baro_v = 0.35f,
        .w_z_surface_p = 3.5f,
        .w_z_surface_v = 6.1f,
        .max_surface_alt_cm = 200.0f,
        .baro_lpf_hz = 1.0f,
        .surface_lpf_hz = 1.0f
    };

    // 示例：静止悬停 1.5m，IMU净加速度约0，baro/tof 都在 150cm 附近
    for (int i = 0; i < 200; i++) {
        alt_step(&s, &p, 0.01f, 0.0f, true, 150.0f, true, 150.0f, true, true);
    }

    printf("Z=%.2fcm, VZ=%.2fcm/s, AGL=%.2fcm, REL=%.2f, Q=%d\n",
           s.z_cm, s.vz_cms, s.agl_cm, s.surface_rel, s.agl_q);
    return 0;
}
```

---

## 7. 最小示例和 INAV 原码的对应关系

1. `IMU 预测步` 对应 `estimationPredict()`：`inav/src/main/navigation/navigation_pos_estimator.c:517`
2. `baro 残差修正` 对应 `estimationCalculateCorrection_Z()`：`inav/src/main/navigation/navigation_pos_estimator.c:545`
3. `地效抑制` 对应 `isAirCushionEffectDetected` 分支：`inav/src/main/navigation/navigation_pos_estimator.c:579`
4. `surface reliability + AGL` 对应 `estimationCalculateAGL()`：`inav/src/main/navigation/navigation_pos_estimator_agl.c:89`
5. `ABS/AGL 选择` 对应 `navGetCurrentActualPositionAndVelocity()`：`inav/src/main/navigation/navigation.c:2936`

---

## 8. 你场景下的实操重点（源码导出的直接结论）

1. 如果你只开 `NAV ALTHOLD`，但没启用 `BOXSURFACE`，控制器默认走 ABS 高度，ToF 不是主闭环高度源。
2. 如果你的目标是“室内 1.5m 更稳”，建议：
   - 打开 `BOXSURFACE` 做地形跟随模式验证；
   - 打开 `rangefinder_median_filter`；
   - 将 `inav_max_surface_altitude` 至少覆盖到 150cm（默认 200cm 已覆盖）；
   - 再微调 `w_z_surface_*` 与 `w_z_baro_*`。
3. ICM42688 质量直接影响预测项；BMP388 决定全局参考稳定性；VL53L1X 决定近地约束精度。三者必须同时健康才会“融合后更准”。

