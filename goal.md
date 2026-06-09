# Goal 模式多智能体重构提示词方案

## 摘要
用“文档先行、分仓库写入、只读互审、总控集成”的方式跑长期 Goal。所有子智能体统一使用 `gpt-5.5 / xhigh / priority`。  
关键事实锁定：旧 SPI 是 `97B` 固定 transfer，图像上行应用 payload 是 `77B`；Air 侧按给定网络应落到 `SPI_3`：`P03_2 SCK`、`P03_1 MOSI`、`P03_0 MISO`、软件 CS `P03_3/P03_4`、READY `P01_0/P01_1`。

## 总控提示词
```text
你是本轮 Goal 总控。目标：车端移除旧 2BL3 SPI 图像融合链路；飞机 CYT4BB7 核心1作为 SPI 主机读取两块 CYT2BL3 图像板；2BL3 只做图像算法和 SPI 从机。

硬约束：
1. 先读 AGENTS.md、相关代码、IAR 工程引用，再改。
2. 不做无关重构，不格式化无关文件，不改厂商库。
3. 同一阶段只允许一个 worker 写同一仓库；reviewer 只读。
4. 删除前必须 rg 调用点，并清理 IAR 工程引用。
5. Air 核心0不得初始化/调用 IPS114，也不得初始化/轮询外置图像板 SPI。
6. Air 核心1负责下视摄像头、IPS114、外置 2BL3 SPI 主机、IPC 汇总。
7. 2BL3 board_id 由 Air 下发，只允许 0/1。
8. 最终说明必须包含：受限于硬件环境，没有实际测试代码可行性。
```

## 智能体分配
### Gate 1：协议文档
Agent 1，worker，只写 `CYT4BB7_Air/doc/飞控和图像板的SPI主从通信设计方案.md`：

```text
只写设计文档，不改 C/H/IAR。阅读车端旧 CameraSpi、2BL3 从机、Air CM7_1、IPC。

文档必须包含：
- 拓扑：Air CM7_1 为 SPI 主机，两块 2BL3 为从机。
- 引脚：Air SPI_3，SCK=P03_2，MOSI=P03_1，MISO=P03_0，CS0=P03_3，CS1=P03_4，READY0=P01_0，READY1=P01_1；2BL3 端 P0_2/P0_1/P0_0/P0_3/P18_6。
- 线协议：AA55、cmd=0x20、len BE、CRC16 LE、ED、97B 固定 transfer。
- payload：上行 77B 图像包，即 4 beacon + 1 car_lamp；下行也占 77B 容量，有效字段为 magic、board_id、counter、flight_state、image_tcp、image_display，其余清零。
- 两板策略：board_id 只用 0/1，废弃旧三从机和三相机融合。
- 调度：主循环非阻塞轮询 READY/downlink mask，round-robin，独立软件 CS。
- 飞行状态：CM7_0 通过 IPC 告诉 CM7_1；CM7_1 下发给 2BL3；飞行中 Air 和 2BL3 调试屏不刷新。
- 验证：rg、IAR 工程引用、逻辑分析仪建议、硬件未测限制。
```

Agent 2，reviewer，只读审查文档：

```text
重点找 blocking：是否误用旧 12B/32B 手册、是否漏写 97B/77B、是否残留三从机、是否引脚/CS/READY 不一致、是否让 Air 核心0碰 IPS114 或外置 SPI、是否存在 TBD。blocking 反馈 Agent 1 修复。
```

### Gate 2：车端清理
Agent 3，worker，只写 `CYT4bb7_Car`：

```text
目标：车端不再接外置 2BL3，不再做 CameraSpi 图像融合。先 rg，再最小删除。

删除候选：
- project/code/Estimation/Image/beacon_fusion.[ch]
- project/code/Protocols/CameraSpi/*
- project/code/HW_Drivers/CameraSpi/*
- car_loop.h/c 里的 g_image_spi、CameraSpi 初始化/轮询、payload 解析、car_loop_camera_spi_update_100HZ、图像下行、图像 WiFi 遥测
- zf_common_headfile.h 对 beacon_fusion.h 的包含
- IAR 工程里上述文件/目录引用

保留：
- Protocols/AirComm 空地串口
- WiFi SPI 调试栈
- IMU、编码器、电机、UWB/AOA、fixator、beacon_config、Beacon_Detection
- SBUS/WirelessControl/GNSS 等非图像链路，除非另有明确要求

注意：`air_comm_send_run_data(car_data, 10)` 里原 car_data[2..9] 是图像字段。不要擅自改变空地 RUN_DATA 长度；优先保留 10 个 float，图像位填 0 作为过渡。
```

Agent 4，reviewer，只读审查车端：

```text
检查 CameraSpi/g_image_spi/beacon_fusion/Estimation/Image/HW_Drivers/CameraSpi 是否无业务残留；IAR 是否不引用不存在文件；car_loop 初始化和 1000Hz/100Hz/25Hz 节拍是否完整；AirComm、WiFi、IMU、UWB/AOA、电机控制是否未被误删；是否有多余新函数/变量。
```

### Gate 3：Air 与 2BL3 实现
Agent 5，worker，只写 `CYT4BB7_Air`：

```text
根据通过的文档实现 Air CM7_1 SPI 主机。
- 新建最小 CameraSpi 主机模块。
- 硬件层只管 SPI_3、P03_3/P03_4 软件 CS、P01_0/P01_1 READY、单次 97B 传输。
- 协议层只管两板轮询、CRC、seq/ack、online、最新快照、SendRaw/ReceiveRaw。
- main_cm7_1 初始化并在 while 空闲路径非阻塞 Poll，不绑定 mt9v03x_finish_flag。
- 不写 g_image_beacons/g_image_car_lamps，不混淆下视摄像头语义。
- 如发给 CM7_0，追加独立 IPC 快照字段或新 IPC 模块，不改变现有下视字段语义。
- 禁止修改核心0控制策略，禁止让核心0初始化 SPI/IPS114。
```

Agent 6，worker，只写 `CYT2BL3_Image`：

```text
适配 2BL3 通用从机固件。
- 保持 SCB0 从机引脚：P0_0 MISO、P0_1 MOSI、P0_2 SCK、P0_3 CS、P18_6 READY。
- 与 Air 文档一致：97B transfer、77B 图像 payload、board_id 只接受 0/1。
- 接收 flight_state，飞行中停止本地屏幕刷新。
- 不改图像算法，不启用 debug_init，因为它占用 SCB0/P0_0/P0_1。
```

### Gate 4：最终总审查
Agent 7，reviewer，只读全仓库：

```text
检查：
1. Air 核心0无 IPS114 初始化/显示，无外置 2BL3 SPI 初始化/轮询。
2. Air 核心1负责下视图像、IPS114、SPI_3 两板轮询、IPC 汇总。
3. 2BL3 引脚、帧长、payload、board_id、flight_state 与文档一致。
4. Car 端旧 CameraSpi/Estimation/Image/beacon_fusion/g_image_spi 已清理。
5. IAR 工程不引用不存在文件。
6. 没有三从机常量或 board[2] 业务残留。
7. 改动保持最少代码、最少函数、最少变量。
```

## 验收标准
- 文档通过审查后才编码。
- 车端只保留车辆本体控制和空地串口等必要链路。
- Air CM7_1 能独立管理两块 2BL3；CM7_0 不碰显示和外置 SPI。
- 2BL3 固件可刷两块板，通过下行 board_id 区分。
- 所有 blocking findings 必须修完。
- 最终汇报必须明确：受限于硬件环境，没有实际测试代码可行性。