# WiFi 协议总览说明

## 1. 模块结构

当前 WiFi 通信栈统一放在 `project/code/Protocols/wifi/` 下，分为四层：

- `wifi_cmd`
  - 负责 `wifi_spi` 初始化、UDP 建链、严格 `CRLF` 文本分帧、命令路由与统一文本回包
- `wifi_params`
  - 负责参数白名单注册、查询、修改、保存、加载以及待机态遥测开关
- `wifi_cal_imu`
  - 负责 `imu ...` 命令解析，并把命令转接到 `Accel_Calibration`
- `wifi_justfloat`
  - 负责 JustFloat 遥测发送，线协议保持不变

数据流关系如下：

`wifi_spi -> wifi_cmd -> wifi_params / wifi_cal_imu / wifi_justfloat`

## 2. 文本协议通用规则

- 所有文本命令必须以 `\r\n` 结束
- 命令字与参数名大小写不敏感
- 成功回包统一为 `OK ...\r\n`
- 失败回包统一为 `ERR ...\r\n`
- `start/stop` 只影响 JustFloat 遥测，不影响文本命令的收发

## 3. 参数命令

支持的参数命令：

- `help`
- `get <name>`
- `set <name> <value>`
- `save`
- `load`
- `list`
- `start`
- `stop`

说明：

- `get/set/save/load/list` 只在待机且未解锁时允许执行
- `set` 只修改 RAM
- `save` 才会将当前参数写入 Flash
- `load` 会从 Flash 重新装载参数
- `stop` 会停止待机态 JustFloat 遥测
- `start` 会恢复待机态 JustFloat 遥测

## 4. IMU 校准命令

当前对外保留三类 IMU 校准入口：

- `imu start gyro`
- `imu start accel`
- `imu start accel_man`

以及以下辅助命令：

- `imu help`
- `imu help start`
- `imu help acc`
- `imu help status`
- `imu status`
- `imu acc collect`
- `imu acc stop`
- `imu load`
- `imu save`
- `imu default`
- `imu clear`
- `imu flash`

说明：

- `imu status` 与 `imu flash` 任意状态都允许执行
- `imu load/save/default/clear/start .../imu acc ...` 只在待机且未解锁时允许执行
- 校准进行中再次下发 `imu load/save/clear/start ...` 会返回 `ERR imu busy`
- `imu start accel` 为自动椭球校准
- `imu start accel_man` 为手动椭球校准

### 4.1 角速度校准

命令：

- `imu start gyro`

行为：

- 启动后会先输出开始提示
- 预稳定阶段会输出进度
- 进入采集阶段后会输出采集进度
- 完成时会输出 `bias_dps` 与 `std_dps`
- 失败时会输出失败原因

示例回包：

- `OK imu gyro 开始 请保持飞机静止`
- `OK imu gyro 预稳定 progress=25 samples=375/1500`
- `OK imu gyro 进入采集阶段 target=60000`
- `OK imu gyro 采集 progress=50 valid=30000/60000`
- `OK imu gyro 完成 samples=60000 bias_dps=0.01,-0.02,0.03 std_dps=0.05,0.04,0.05`

### 4.2 加速度自动椭球校准

命令：

- `imu start accel`

行为：

- 启动后会输出开始提示
- 每次进入新的静止姿态点时会自动开始采样
- 姿态点采样完成后会输出该点均值
- 姿态点过近、噪声过大、求解失败等情况会主动提示
- 求解成功后会自动保存到 Flash，并输出偏置与矩阵

示例回包：

- `OK imu accel 开始 请缓慢切换多个静止姿态`
- `OK imu accel 提示 每次放稳后保持静止 系统会自动采集姿态点`
- `OK imu accel 开始采集 pose=1 target=2000`
- `OK imu accel 姿态点完成 pose=1 mean_g=0.01,-0.03,-0.99`
- `OK imu accel 开始求解 pose_count=12`
- `OK imu accel 完成 save=1 bias_g=0.02,-0.01,0.03`
- `OK imu accel 矩阵 r0=1.01,0.00,-0.01`

### 4.3 加速度手动椭球校准

命令：

- `imu start accel_man`
- `imu acc collect`
- `imu acc stop`

行为：

- `imu start accel_man` 只进入手动校准准备态，不会自动开始采点
- `imu acc collect` 会先检测静止，再采集 `5000` 个样本形成 1 个姿态点
- 手动模式不做“姿态点过近”拒绝，适合人工采集 `50~100+` 个不同姿态点
- 每个姿态点完成后都会回传 `mean_g`、`std_g`、`std_max_g`
- `imu acc stop` 会结束手动会话，并对已完成姿态点做椭球拟合
- 若 `stop` 时当前点未采满，该未完成点会被丢弃
- 拟合成功后会自动保存到 Flash，并输出矩阵和质量指标

最终质量指标：

- `pose_count`
- `mean_pose_std_g`
- `max_pose_std_g`
- `bias_norm_g`
- `fit_rms_g`
- `max_norm_err_g`
- `bias_g`
- `accel_matrix r0/r1/r2`

示例回包：

- `OK imu accel_man 开始 请手动切换多个静止姿态`
- `OK imu accel_man 提示 单次采点请发 imu acc collect`
- `OK imu accel_man 等待静止 pose=1 stable_target=1250`
- `OK imu accel_man 开始采集 pose=1 target=5000`
- `OK imu accel_man 姿态点完成 pose=1 mean_g=0.01,-0.02,-0.99 std_g=0.002,0.003,0.004 std_max_g=0.004`
- `OK imu accel_man 提示 继续采点请发 imu acc collect`
- `OK imu accel_man 提示 结束并求解请发 imu acc stop`
- `OK imu accel_man 开始求解 pose_count=50`
- `OK imu accel_man 结果 pose_count=50 mean_pose_std_g=0.003 max_pose_std_g=0.008`
- `OK imu accel_man 结果 bias_norm_g=0.067 fit_rms_g=0.006 max_norm_err_g=0.018`
- `OK imu accel_man 完成 save=1 bias_g=0.006,-0.001,0.065`

### 4.4 Flash 校准参数查询

命令：

- `imu flash`

行为：

- 如果 Flash 中没有有效校准参数，返回 `OK imu flash empty`
- 如果存在有效数据，会按多行人类可读格式输出

示例回包：

- `OK imu flash begin version=2 full_matrix=1`
- `OK imu flash gyro_bias_dps x=0.01 y=-0.02 z=0.03`
- `OK imu flash accel_bias_g x=0.01 y=-0.01 z=0.02`
- `OK imu flash accel_matrix r0=1.00,0.00,0.00`
- `OK imu flash imu_to_body r0=1.00,0.00,0.00`
- `OK imu flash end`

## 5. 文本回包示例

- `OK ping`
- `OK set roll_angle_kp=5.5`
- `OK list begin count=76`
- `OK list end`
- `OK imu gyro 开始 请保持飞机静止`
- `OK imu accel 开始 请缓慢切换多个静止姿态`
- `OK imu accel_man 开始 请手动切换多个静止姿态`
- `OK imu flash begin version=2 full_matrix=1`
- `ERR state`
- `ERR unknown param`
- `ERR imu busy`

## 6. JustFloat 接口

JustFloat 线协议没有变化，调用方式保持不变。

注意：

- 最多支持 `16` 个浮点通道
- 发送端与上位机端应保持通道顺序一致

### 6.1 当前发送链路

当前 WiFi 链路可以按下面理解：

`wifi_cmd_Init -> wifi_spi_init -> wifi_spi_socket_connect(UDP)`  
`wifi_justfloat -> wifi_cmd_SendBuffer -> wifi_spi_send_buffer`  
`wifi_cmd_Poll -> wifi_spi_send_poll -> wifi_cmd_FlushNow -> wifi_spi_udp_send_now`

两个函数的意义：

- `wifi_justfloat`
  - 把一组 `float` 按 JustFloat 协议打包，并提交到 WiFi 发送缓存。
  - 它只负责“提交待发送数据”，不等待底层真正发完。
- `wifi_cmd_Poll`
  - 推进底层 WiFi SPI 发送状态机，并在发送完成后触发 UDP 发包。
  - 非飞行态下也负责轮询上位机文本命令接收。

频率说明：

- 实际遥测发送频率，主要由 `wifi_justfloat` 的调用频率决定。
- `wifi_cmd_Poll` 建议在主循环空闲区持续调用，它主要影响发送延迟和丢帧，不决定采样频率。

DMA 说明：

- 底层 `wifi_spi_send_poll` 在发送数据载荷时优先尝试 DMA。
- 如果 DMA 启动失败，会退回普通 SPI 发送。
- 发送头部、应答处理、接收轮询和 UDP 触发不是整段 DMA 全异步处理。

效果：

- 使用 DMA 后，较长载荷发送时 CPU 占用更低，主循环抖动更小。
- 但 `wifi_cmd_Poll` 仍然需要持续运行，因为底层还有状态推进、应答读取和命令接收开销。
- 想省 CPU，优先降低 `wifi_justfloat` 的调用频率，不要停掉 `wifi_cmd_Poll`。

### 6.2 当前实测耗时

按当前工程与当前测试方法，得到下面这组结果：

- 单独调用 `wifi_justfloat()`：`99 / 10000 = 0.0099 ms`，约 `9.9 us/次`
- 调用 `wifi_justfloat() + wifi_cmd_Poll()`：`433 / 5000 = 0.0866 ms`，约 `86.6 us/次`
- 按差值估算，`wifi_cmd_Poll()` 本次额外耗时约为：`86.6 - 9.9 = 76.7 us/次`

这组数据说明：

- `wifi_justfloat()` 本体主要是打包和提交发送，耗时较小。
- 主要 CPU 开销在 `wifi_cmd_Poll()`，因为它会推进底层 SPI 发送状态机、处理 UDP 触发和接收轮询。
- `wifi_justfloat()` 的计时不等于“数据已经真正发到上位机”，真正发送过程主要在后续的 `wifi_cmd_Poll()` 中完成。

使用建议：

- `wifi_justfloat()` 适合放在 `50Hz` 或 `100Hz` 这类较低频率循环。
- `wifi_cmd_Poll()` 适合放在主循环空闲区持续运行，不建议塞进高实时要求的控制闭环里当作固定频率任务。

[返回 Air 总文档](../../../../README.md) · [返回母仓库 README](../../../../../README.md)

## 7. 兼容性说明

旧目录仍保留在磁盘上，但主循环与 IAR 工程入口已经切到新的 `project/code/Protocols/wifi/` 目录。
