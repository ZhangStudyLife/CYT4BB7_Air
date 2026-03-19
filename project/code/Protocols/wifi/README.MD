# WiFi 协议总线说明

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

当前对外只保留两类 IMU 校准：

- `imu start gyro`
- `imu start accel`

以及以下辅助命令：

- `imu help`
- `imu help status`
- `imu status`
- `imu load`
- `imu save`
- `imu clear`
- `imu flash`

说明：

- `imu status` 与 `imu flash` 任意状态都允许执行
- `imu load/save/clear/start ...` 只在待机且未解锁时允许执行
- 校准进行中再次下发 `imu load/save/clear/start ...` 会返回 `ERR imu busy`
- 加速度校准使用椭球法，不再对外暴露六面法和 `all` 组合校准
- `imu start gyro` 与 `imu start accel` 会主动输出中文过程提示

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

### 4.2 加速度校准

命令：

- `imu start accel`

行为：

- 启动后会输出开始提示
- 每次进入新的静止姿态点时会输出采集提示
- 姿态点采集完成会输出该点均值
- 姿态点过近、噪声过大、求解失败等情况都会主动提示
- 求解成功后会自动保存到 Flash，并输出偏置与矩阵

示例回包：

- `OK imu accel 开始 请缓慢切换多个静止姿态`
- `OK imu accel 提示 每次放稳后保持静止 系统会自动采集姿态点`
- `OK imu accel 开始采集 pose=1 target=2000`
- `OK imu accel 姿态点完成 pose=1 mean_g=0.01,-0.03,-0.99`
- `OK imu accel 开始求解 pose_count=12`
- `OK imu accel 完成 save=1 bias_g=0.02,-0.01,0.03`
- `OK imu accel 矩阵 r0=1.01,0.00,-0.01`

### 4.3 Flash 校准参数查询

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
- `OK imu flash begin version=2 full_matrix=1`
- `ERR state`
- `ERR unknown param`
- `ERR imu busy`

## 6. JustFloat 接口

JustFloat 线协议没有变化，但公开调用形式已经改成尾参数量形式：

旧写法：

```c
wifi_justfloat(a, b, c, d, e, f);
```

新写法：

```c
wifi_justfloat(a, b, c, d, e, f);
```

注意：

- 最后一个参数必须是通道总数
- 实际参数个数与尾部数量不一致时发送会失败
- 最大支持 `16` 个浮点通道

## 7. 兼容性说明

旧目录仍保留在磁盘上：


但主循环与 IAR 工程入口已经切到新的 `project/code/Protocols/wifi/` 目录。
