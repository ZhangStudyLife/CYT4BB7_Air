# 相机模型标定日志设计

## 目标

在 `main_cm7_0.c` 新增 `camera_model_calibration_log_200hz()`，以 200 Hz 通过 JustFloat 发送离线标定 `Three_Camera` 所需的数据。临时停用当前 Mode1/2/4/5 调试日志，不修改 `Three_Camera.c/.h` 的算法与接口。

## 调用切换

- 保持已注释的 `car_plan_debug_200hz()` 调用不变。
- 注释 `mode1245_wifi_debug_200hz()` 调用。
- 在同一快循环调用 `camera_model_calibration_log_200hz()`。
- 新函数使用 5 ms 本地节拍限频，不新增全局状态。

## JustFloat 通道

单包固定为 63 个用户 `float`：

- I1-I36：Front、Center、Back 各 4 个原始信标，每个依次为 `x, y, area`。
- I37-I48：Front、Center、Back 的第 1 个原始车灯，每个依次为 `cx, cy, angle, length`。
- I49-I50：车端 `g_car_yaw, g_car_yaw_rate_dps`。
- I51-I55：飞机 `g_euler.roll, g_euler.pitch, g_euler.yaw, g_tof_fused_height_mm, g_tof_fused_valid`。
- I56-I59：`Three_Camera` 融合车灯 `x_m, y_m, angle_deg, camera_mask`。
- I60-I63：第 1 个有效的 `Three_Camera` 融合信标 `x_m, y_m, area, camera_mask`。

原始检测无效时，坐标和角度填 `IMAGE_DATA_INVALID_VALUE`，尺寸或面积填 `0.0f`。融合目标无效时，坐标、角度或面积填 `IMAGE_DATA_INVALID_VALUE`，`camera_mask` 填 `0.0f`。

## 数据来源

- 原始像素数据直接读取全局 `image_data`，保留全部信标候选，便于分析误检和候选排序变化。
- 融合数据通过 `CarPlan_3_GetDebug()` 读取当前 `Three_Camera` 结果，避免在日志函数中重复运行投影算法。
- 同一数据包内携带车端与飞机姿态，离线拟合无需跨日志对时。

## 验证

- 确认两个旧调试函数均没有活动调用，新函数存在唯一活动调用。
- 确认写入索引恰好为 63，且不超过 `WIFI_JUSTFLOAT_MAX_FLOAT_NUM - 1U`。
- 确认不修改 `Three_Camera.c/.h`，不新增无关变量、函数或配置。
- 按工程约定只做静态检查，不从命令行调用 IAR 编译。
