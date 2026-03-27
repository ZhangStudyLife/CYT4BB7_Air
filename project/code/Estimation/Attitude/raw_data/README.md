# angle.csv 列映射

该目录下的 `angle.csv` 为 `wifi_justfloat` 输出，共 16 路，排序如下：
- `I0~I2`: `g_imufilter_1000hz.gyrox/gyroy/gyroz`（降噪后的角速度，单位 dps）
- `I3~I5`: `g_imufilter_1000hz.accx/accy/accz`（校准后的加速度，单位 g）
- `I6~I8`: `g_mahony_ahrs.accel_magnitude` 与各类加速权重：
  * `I6`: 加速度模长
  * `I7`: `acc_weight_nearness`
  * `I8`: `acc_weight_rate_ignore`
- `I9`: `g_mahony_ahrs.acc_weight_final`
- `I10~I12`: `g_mahony_ahrs.gyro_bias_x/y/z`
- `I13~I15`: `g_euler.pitch/roll/yaw`，单位 `deg`

## 离线参数标定流程

1. 在 `IMU_Update_1000HZ()` 的 `MahonyAhrs_Update` 之后保持 `wifi_justfloat` 这 16 路输出，记录当前 `MahonyAhrs` 的控制变量、bias 与欧拉角。每次调 `Mahony` 的 `MAHONY_KP_DEFAULT`、`MAHONY_KI_DEFAULT`、`MAHONY_ACCEL_NEARNESS_WIDTH_G`、`MAHONY_ACCEL_WEIGHT_MIN`、`MAHONY_SPIN_RATE_LIMIT_DPS` 形成一组新固件，飞行日志会自动保存在此目录。
2. 每组飞行建议包含：静置 5 秒（`acc_weight_final` 高于 0.7 并且 `gyro_bias_*` 统计稳定）、轻度手操 10 秒、然后一段 1 分钟的实际悬停或前后左右小修正，目的是同时验证静态粘性和动态扰动下的收敛性。
3. 运行 `python mahony_tune.py`（见下文）分析每个日志，参考输出的 `pitch/roll` 误差与 `acc_weight_final` 分布，筛选出误差最小且 `acc_weight_final` 在静止段保持高值的参数组。
4. 推荐的起始增益区间（按当前 iNav 6轴对齐口径）：`KP 0.20~0.30`，`KI 0.003~0.008`，`accel_nearness_width 0.18~0.24`，`acc_weight_min 0.001~0.005`，`spin_rate_limit 15~25 dps`。建议先用默认值飞一轮拿基线，再围绕上述区间做小步迭代。
5. 选定参数后再飞一次，在多个 `angle.csv` 中对比 `pitch` / `roll` 的静态漂移（`mahony_tune.py` 会输出平均误差）和 `acc_weight_nearness` 在急转弯时降到 0 的行为。理想的 `acc_weight_final` 高低变化应该与 `|acc|` 远离 1g 同步，而不是突然归零。

## 离线分析脚本

`mahony_tune.py` 提取所有 `raw_data/*.csv`，输出每份日志的：
- 均值加速度 tilt error（Mahony 欧拉 vs. 加速度计算）
- `acc_weight_final` 中位数和 `gyro_bias_*` 平均值

运行方式：`python project/code/Estimation/Attitude/mahony_tune.py`。脚本便于快速对比参数组，增强调参效率。
