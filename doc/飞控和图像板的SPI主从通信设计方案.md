# 飞控与图像板 SPI 主从通信设计说明

## 1. 文档范围与当前结论

本文描述当前代码中已经实现的通信链路，重点说明：

- Air 飞控板如何通过一组 SPI 总线连接两块外置 CYT2BL3 图像板；
- SPI 引脚、4BB7 内部外设、通信模式、速率和调度方式；
- Air 与图像板双向传输了哪些数据；
- 当前实现如何减少阻塞、总线占用和无效数据处理；
- Car-F 如何取得这条 SPI 链路的诊断信息，以及它与 SPI 总线的边界。

本文以当前工程代码为准：

| 工程 | 核对版本 | 主要依据 |
| --- | --- | --- |
| `CYT4BB7_Air` | `3c6a366bdd66c51ce0d4dd1b9da00b058f990e9c` | `project/code/Protocols/CameraSpi/camera_spi.c`、`project/user/main_cm7_1.c`、`project/code/IPC/ipc_image_data.*`、`project/code/Protocols/AirComm/*` |
| `CYT4BB7_Car_F` | `e9807698097105fb33a73d193e6f37afe0afb4ad` | `project/code/Protocols/AirComm/*`、`project/code/Controller/car_loop.c`、`project/code/Menu/menu_config.c` |
| `CYT2BL3_Image` | 当前工作区代码，仅用于核对从机引脚和协议对端 | `project/code/Protocols/CameraSpi/*` |

先明确一个容易混淆的结论：**Car-F 不是这条 SPI 总线的从机。** 当前 SPI 主机是 Air 的 `CM7_1`，SPI 从机是两块 CYT2BL3 图像板。Car-F 与 Air 之间使用独立 UART 链路；Air 只把 SPI 在线状态、READY、错误码等诊断数据通过 UART 转发给 Car-F。

## 2. 系统拓扑与数据流

```text
                                  Air CYT4BB7
                         +---------------------------+
                         | CM7_0：飞控、规划、参数入口 |
                         |    ^              |       |
                         |    | 双核共享内存  | IPC   |
                         |    | + IPC 中断    v       |
                         | CM7_1：图像汇总、SPI 主机  |
                         |    | SCB6 / SPI Mode 0     |
                         +----+-----------+-----------+
                              |           |
                  共用 SCK/MOSI/MISO      |
                    独立 CS0/READY0        | 独立 CS1/READY1
                              |           |
                       +------v--+     +--v------+
                       | 2BL3 #0 |     | 2BL3 #1 |
                       | 映射 Front |   | 映射 Back |
                       +---------+     +---------+

Air CM7_0 UART_2  <---------- 1.152 Mbps UART ---------->  Car-F CM7_0 UART_3
  P10_1 / P10_0                                      P17_1 / P17_2
```

三路图像在 Air 内部的来源为：

| Air 图像槽位 | 数据来源 | 进入 CM7_1 的方式 |
| --- | --- | --- |
| `Front` | 外置图像板 board 0 | SPI，CS0/READY0 |
| `Center` | Air 板载下视摄像头 | Air 本地图像处理 |
| `Back` | 外置图像板 board 1 | SPI，CS1/READY1 |

board 0/1 的身份不是由两份不同的图像板固件决定，而是 Air 根据当前拉低的 CS，在下行 payload 中写入 `board_id=0/1`。Air 收到数据后，再把 board 0 固定放进 `Front`，board 1 固定放进 `Back`。

## 3. SPI 接线与引脚对应关系

### 3.1 Air 到两块图像板

两块图像板共用 `SCK`、`MOSI` 和 `MISO`，每块板各有一根独立的 `CS` 和 `READY`：

| 信号 | Air CYT4BB7 引脚 | Air 功能 | 图像板 #0 | 图像板 #1 | 方向 |
| --- | --- | --- | --- | --- | --- |
| SCK | `P03_2` | `SCB6 SPI CLK` | `P0_2 / SCB0 CLK` | `P0_2 / SCB0 CLK` | Air -> 两板 |
| MOSI | `P03_1` | `SCB6 SPI MOSI` | `P0_1 / SCB0 MOSI` | `P0_1 / SCB0 MOSI` | Air -> 两板 |
| MISO | `P03_0` | `SCB6 SPI MISO` | `P0_0 / SCB0 MISO` | `P0_0 / SCB0 MISO` | 两板 -> Air |
| CS0 | `P03_3` | 软件 GPIO 片选，低有效 | `P0_3 / SCB0 SELECT0` | 不连接 | Air -> board 0 |
| CS1 | `P03_4` | 软件 GPIO 片选，低有效 | 不连接 | `P0_3 / SCB0 SELECT0` | Air -> board 1 |
| READY0 | `P01_0` | GPIO 输入，下拉 | `P18_6` | 不连接 | board 0 -> Air |
| READY1 | `P01_1` | GPIO 输入，下拉 | 不连接 | `P18_6` | board 1 -> Air |
| GND | Air GND | 公共参考地 | 图像板 GND | 图像板 GND | 必须共地 |

接线时需要注意：

1. `MOSI` 和 `MISO` 按主机视角命名，不能因为图像板是从机而交叉连接。Air `MOSI` 接图像板 `MOSI`，Air `MISO` 接图像板 `MISO`。
2. 两块图像板的 `MISO` 可以并接，是因为只有被 CS 选中的从机允许驱动总线；任何时刻 Air 只会拉低一个 CS。
3. READY 高电平表示该图像板已有可供主机读取的上行帧。Air 端给 READY 输入配置了内部下拉，因此断线时默认按“未就绪”处理。
4. 代码只定义 MCU 引脚，不定义连接器针脚编号和供电电压。实际线束仍应按所用 PCB 原理图确认电源与接口序号。

### 3.2 Air 与 Car-F 的独立 UART 接线

这组引脚不属于 Camera SPI，但 Car-F 通过它观察 SPI 状态并发起图像板参数操作：

| 信号 | Air CYT4BB7 | Car-F CYT4BB7 | 连接方式 |
| --- | --- | --- | --- |
| Air TX -> Car RX | `UART_2 TX = P10_1` | `UART_3 RX = P17_1` | `P10_1 -> P17_1` |
| Car TX -> Air RX | `UART_2 RX = P10_0` | `UART_3 TX = P17_2` | `P17_2 -> P10_0` |
| GND | Air GND | Car-F GND | 共地 |

UART 波特率为 `1,152,000 bit/s`。不要把 Car-F 的 UART 引脚接到 Air 的 `P03_0~P03_4` Camera SPI 引脚上。

## 4. Camera SPI 使用了 4BB7 的哪些模块

### 4.1 CM7_1 图像核

Camera SPI 完全由 Air 的 `CM7_1` 管理。`CM7_1` 同时负责 Air 本地下视摄像头处理、两块外置图像板的 SPI 轮询、三路图像汇总，以及通过 IPC 把结果交给 `CM7_0`。这样可把图像采集和通信从飞控主核 `CM7_0` 的高频控制任务中分离出来。

### 4.2 SCB6

Air 使用 4BB7 的 `SCB6`，配置为 Motorola SPI 主机：

| 配置项 | 当前值 |
| --- | --- |
| 外设实例 | `SCB6` |
| 模式 | Master，Motorola SPI |
| SPI 模式 | Mode 0，`CPOL=0`、`CPHA=0` |
| 数据宽度 | 8 bit |
| 位序 | MSB first |
| SCK/MOSI/MISO | `P03_2/P03_1/P03_0` |
| MISO 采样 | late sample 开启 |
| SCB oversample | 4 |

`P03_3/P03_4` 没有交给 SCB 硬件片选，而是作为普通 GPIO 由软件控制，从而用同一 SCB 外设选择两块从机。

### 4.3 GPIO、HSIOM 与资源占用

- HSIOM 将 `P03_0/P03_1/P03_2` 复用到 `SCB6` 的 MISO/MOSI/CLK。
- MOSI、CLK 配置为强驱动输出；MISO 配置为高阻输入。
- `P03_3/P03_4` 配置为推挽输出，初始化为高电平。
- `P01_0/P01_1` 配置为带下拉的 GPIO 输入。

`SCB6` 的这些引脚也可以被库映射为 `SPI_3` 或 `UART_6`，旧 PMW3901 定义也占用 `P03_0~P03_3`。当前 Camera SPI 已经取得这组引脚的所有权，因此 PMW3901 或 UART6 不能再同时初始化到这些引脚。

### 4.4 外设时钟

代码给 `PCLK_SCB6_CLOCK` 分配 24.5 bit 分频器槽 10。目标内部采样时钟为：

```text
SCB 目标时钟 = SPI 波特率 x oversample
             = 2 MHz x 4
             = 8 MHz
```

整数和 1/32 精度的小数分频值由 `CY_INITIAL_TARGET_PERI_FREQ` 运行时计算，因此不依赖把外设总线频率硬编码成某个固定值。

### 4.5 SCB 中断、PIT、DWT 与 IPC

| 4BB7 模块 | 当前用途 |
| --- | --- |
| SCB6 中断 | `scb_6_interrupt_IRQn` 经 `CPUIntIdx5_IRQn` 进入 CM7_1，优先级 6 |
| `PIT_CH10`（TCPWM 计数器封装） | 每 `5000 us` 产生一次 Camera SPI 调度节拍 |
| DWT `CYCCNT` | 以 CPU 周期判断单次传输是否超过 `2000 us` |
| IPC 通道/中断 | 在 CM7_0 与 CM7_1 之间通知图像、飞行状态和参数事件 |
| 共享 SRAM | 存放三路图像、姿态和 Camera SPI 诊断快照 |
| D-Cache | CM7_0、CM7_1 当前都显式关闭；IPC 辅助函数仍保留 clean/invalidate 调用 |

SCB 中断处理函数调用 `Cy_SCB_SPI_Interrupt()` 推进 PDL 的传输上下文。当前 Camera SPI **没有配置 DMAC/PDMA 通道**；它使用 SCB FIFO、PDL 中断传输和静态收发缓冲，因此“非阻塞”不等于“DMA 传输”，CPU 仍会处理 SCB 中断。

## 5. 通信速度、周期与总线占用

### 5.1 基本参数

| 项目 | 当前值 |
| --- | --- |
| SCK 频率 | `2 MHz` |
| 单次 transfer | 固定 `97 byte = 776 bit` |
| 调度周期 | `5 ms`，即最高 200 轮/s |
| 单板理想线时 | `776 / 2 MHz = 388 us` |
| 两板均 READY 时理想线时 | `2 x 388 us = 776 us` |
| 单板每周期总线占用率 | `388 us / 5 ms = 7.76%` |
| 双板每周期总线占用率 | `776 us / 5 ms = 15.52%` |
| SPI 单方向原始带宽 | `2 Mbit/s = 250 kB/s` |

上述线时不含软件拉 CS、函数调用以及两个 transfer 之间的短间隔，所以逻辑分析仪实测值会略大。SPI 是全双工的：主机发送 97 字节请求的同时，也接收 97 字节响应，不需要再增加一个 388 us 的独立接收阶段。

当两块板都持续 READY，并且每 5 ms 都各传一次时：

```text
总线时钟出的字节数 = 97 B x 2 x 200 = 38,800 B/s
每块板的最大 transfer 频率 = 200 次/s
每块板的线字节量 = 97 B x 200 = 19,400 B/s
```

这是调度上限，不代表图像算法一定每秒产生 200 份不同结果。Air 使用图像结果序号去重，相同 `result_sequence` 不会被当成新图像再次发布。

### 5.2 2 MHz 的余量

从当前负载看，即使两块板每轮都通信，理想总线占用也只有约 15.52%，5 ms 周期内仍留有约 4.2 ms 余量。这个速率在满足图像结果传输的同时，给线束质量、从机响应和中断抖动保留了余量。

这里的“余量”来自理论计算，不是信号完整性测试结论。最终允许的线长、边沿质量和误码率仍应使用示波器或逻辑分析仪实测。

## 6. 一轮通信如何执行

当前调度由 `CameraSpi_Update()` 和 `CameraSpi_Service()` 共同推进：

1. PIT 每 5 ms 产生一个任务节拍。
2. `CameraSpi_Update()` 更新 50/100 ms 新鲜度计数，读取 CM7_0 共享的姿态、飞行状态和输出开关。
3. Air 读取 `READY0/READY1`，只把当前 READY 为高的板加入本轮 `pending_mask`。
4. `CameraSpi_Service()` 按 board 0、board 1 的固定顺序检查待处理板。
5. 启动某块板前再次读取 READY；如果已经变低，则本轮跳过该板。
6. Air 构造固定 97 字节 TX，清空 RX/TX FIFO，拉低对应软件 CS。
7. 调用 `Cy_SCB_SPI_Transfer()` 后立即回到状态机，不在原地等待 97 字节完成。
8. SCB6 中断推进收发；主循环反复调用 `CameraSpi_Service()` 查询状态。
9. 传输结束且总线不忙后，Air 拉高 CS，校验响应并更新该 board 快照。
10. 本轮所有板结束后，统一推进参数事务并发布 Camera SPI 诊断日志。

当前代码有两个必须理解的行为：

- **只有 READY 为高才启动 transfer。** 当前没有旧设计中的 `ready_mask | downlink_mask` 强制下发路径；飞行状态、姿态或参数命令也要等从机 READY 后，才能随下一次 transfer 下发。
- **顺序是 board 0 再 board 1，不是 round-robin。** 单次传输是非阻塞的，但两板同时 READY 时，board 1 要等 board 0 完成后才开始。

## 7. 固定 97 字节线帧

### 7.1 公共帧格式

每次 CS 拉低期间固定产生 97 字节时钟。请求和响应都使用同一外层格式：

| 偏移 | 长度 | 字节序 | 字段 | 当前值/说明 |
| --- | ---: | --- | --- | --- |
| 0 | 1 | - | `head0` | `0xAA` |
| 1 | 1 | - | `head1` | `0x55` |
| 2 | 1 | - | `cmd` | `0x20`，同步数据 |
| 3 | 2 | 大端 | `payload_len` | 请求 `83`，响应 `89` |
| 5 | N | 按子字段定义 | `payload` | meta + application data |
| 5+N | 2 | 小端 | `crc16` | 低字节在前 |
| 7+N | 1 | - | `tail` | `0xED` |

CRC 初始值为 `0xFFFF`，按位右移，多项式为 `0xA001`。CRC 覆盖 `cmd + 两字节 payload_len + payload`，不包含帧头、CRC 自身和帧尾。

### 7.2 请求与响应长度

| 方向 | meta | app 容量 | payload_len | 有效帧长 | 实际 transfer |
| --- | ---: | ---: | ---: | ---: | ---: |
| Air -> 图像板 | 6 B | 77 B | 83 B | 91 B | 97 B |
| 图像板 -> Air | 12 B | 77 B | 89 B | 97 B | 97 B |

Air 请求帧在第 90 字节写入 `0xED`，后面 6 字节为清零填充；填充不属于请求 CRC 和协议内容。响应帧正好占满 97 字节。

SPI 是同时收发的。图像板要在 READY 拉高前准备好响应帧，Air 拉低 CS 后，在发送本次请求的同时读取已经准备好的上行数据。

## 8. Air 下发给图像板的数据

### 8.1 请求 meta：6 字节

| payload 内偏移 | 长度 | 字节序 | 字段 | 说明 |
| --- | ---: | --- | --- | --- |
| 0 | 4 | 小端 | `sequence` | Air 对该 board 的请求序号 |
| 4 | 2 | 小端 | `app_len` | 常规 43 B，参数命令时 24 B |
| 6 | 77 | - | `app_data` | 固定应用区，未用字节清零 |

### 8.2 常规控制、身份和姿态

| app 偏移 | 长度 | 字节序 | 字段 | 说明 |
| --- | ---: | --- | --- | --- |
| 0 | 1 | - | magic | 固定 `0x5A` |
| 1 | 1 | - | board_id | CS0 写 0，CS1 写 1 |
| 2 | 4 | 小端 | counter | 每块板独立递增的下行计数器 |
| 6 | 1 | - | flight_state | 0=未飞行，1=飞行中 |
| 7 | 1 | - | output_control | bit0 WiFi/TCP，bit1 屏幕，bit2 地平线处理 |
| 8 | 1 | - | param_write_lock | 0=允许 SET，1=禁止 SET；GET 不受限制 |
| 9..23 | 15 | - | parameter command | 没有参数事务时清零 |
| 24 | 1 | - | attitude magic | 固定 `0xA6` |
| 25 | 1 | - | attitude version | 当前为 1 |
| 26 | 4 | 小端 | attitude sequence | Air 姿态快照序号 |
| 30 | 4 | 小端 float | roll_deg | 横滚角，单位度 |
| 34 | 4 | 小端 float | pitch_deg | 俯仰角，单位度 |
| 38 | 4 | 小端 float | height_mm | 高度，单位 mm |
| 42 | 1 | - | attitude flags | bit0=高度有效 |
| 43..76 | 34 | - | reserved | 清零 |

常规帧的 `app_len=43`。`output_control.bit0` 的规则是：模式 2 始终允许图传；模式 1 只在未飞行时允许；模式 0 关闭。屏幕和地平线开关由 CM7_0 经 IPC 同步。

### 8.3 图像板参数命令

Car-F 菜单可通过 UART 向 Air 发起参数 GET/SET，Air 经 IPC 交给 CM7_1，再广播到两块图像板。参数命令占用 `app_data[9..23]`：

| app 偏移 | 长度 | 字节序 | 字段 |
| --- | ---: | --- | --- |
| 9 | 1 | - | magic=`0xC3` |
| 10 | 1 | - | version=1 |
| 11 | 1 | - | op：1=SET，2=GET |
| 12 | 1 | - | type：float32/int32 |
| 13 | 2 | 小端 | parameter_id |
| 15 | 1 | - | flags，bit0 请求 ACK |
| 16 | 4 | 小端 | transaction id |
| 20 | 4 | 小端 | value bits |

参数命令帧的 `app_len=24`。Air 仍会在偏移 24 后写入姿态；当前 2BL3 对端对此有专门兼容处理，在识别到参数帧和姿态 magic 后，把缓存长度扩展到 43 字节再消费姿态。

普通 SET 先 GET 两块板的旧值，再执行 SET；如果只成功一部分，会尝试按各板旧值回滚。持久化类命令跳过预读。transaction id 用于 ACK 匹配和幂等重放，并区分超时、不一致、部分成功和回滚失败。

## 9. 图像板上行给 Air 的数据

### 9.1 响应 meta：12 字节

| payload 内偏移 | 长度 | 字节序 | 字段 | 说明 |
| --- | ---: | --- | --- | --- |
| 0 | 4 | 小端 | sequence | 图像板上行序号 |
| 4 | 4 | 小端 | ack_sequence | 已处理的 Air 下行序号 |
| 8 | 2 | 小端 | app_len | 图像结果 77 B；参数 ACK 20 B |
| 10 | 1 | - | flags | bit0 上行待取；bit1 收到新下行 |
| 11 | 1 | - | peer_last_error | 图像板最近协议错误 |
| 12 | 77 | - | app_data | 图像结果或参数 ACK |

### 9.2 图像结果：77 字节，版本 4

```text
77 B = 4 B header + 4 x 13 B beacon slot + 1 x 21 B car-lamp slot
```

Header：

| app 偏移 | 长度 | 字段 | 说明 |
| --- | ---: | --- | --- |
| 0 | 1 | version | 固定为 4 |
| 1 | 1 | beacon_count | 有效信标数，最大 4 |
| 2 | 1 | car_lamp_count | 有效车灯数，最大 1 |
| 3 | 1 | result_sequence | 图像算法结果序号，8 bit 循环 |

每个 beacon slot 为 13 字节：

| slot 偏移 | 长度 | 字节序 | 字段 | 说明 |
| --- | ---: | --- | --- | --- |
| 0 | 1 | - | valid | 0/1 |
| 1 | 4 | 小端 float | x | 图像中心为原点，向右为正 |
| 5 | 4 | 小端 float | y | 图像中心为原点，向下为正 |
| 9 | 4 | 小端 float | area | 连通域面积，单位 pixel |

每个 car-lamp slot 为 21 字节：

| slot 偏移 | 长度 | 字节序 | 字段 |
| --- | ---: | --- | --- |
| 0 | 1 | - | valid |
| 1 | 4 | 小端 float | cx |
| 5 | 4 | 小端 float | cy |
| 9 | 4 | 小端 float | width |
| 13 | 4 | 小端 float | length |
| 17 | 4 | 小端 float | angle |

旧文档将 beacon 最后一个 float 写成 `radius`，当前协议类型和 2BL3 打包代码实际传输的是连通域 `area`，因此本文按 `area` 描述。

### 9.3 参数 ACK：20 字节

参数事务期间，图像板可以用 20 字节 ACK 替代 77 字节图像应用数据。ACK 包含命令 magic `0xC3`、ACK magic `0x3C`、版本、GET/SET 操作、状态、数据类型、parameter_id、board_id、transaction id 和实际值。

Air 只有在 transaction、操作、类型、参数号和 board_id 都匹配时，才把 ACK 计入当前双板参数事务。

## 10. 如何节约 CPU、总线和内存开销

### 10.1 一组数据线连接两块图像板

两块图像板共享 SCK/MOSI/MISO，只增加两根 CS 和两根 READY。相比给两块板各占一套串口或 SPI，既减少 4BB7 外设实例占用，也减少高速数据引脚数量。

### 10.2 全双工固定长度传输

97 字节请求和响应在同一段时钟内完成。固定长度省去先读长度、再启动第二段传输的额外片选和状态切换，也让主从双方复用固定大小静态缓冲区。

### 10.3 READY 门控

Air 只轮询 READY 为高的板。没有待取上行数据时不产生 97 字节 SPI 时钟，避免空轮询占用总线和 SCB 中断时间。代价是所有下行控制也依赖 READY；从机如果不再拉高 READY，主机无法单方面用 SPI 时钟恢复它。

### 10.4 中断式非阻塞状态机

`Cy_SCB_SPI_Transfer()` 启动后，单次 `CameraSpi_Service()` 不在函数内部循环读取 97 字节，而是立即返回；SCB 中断搬运 FIFO，主循环随后优先反复调用 `CameraSpi_Service()` 检查状态并衔接下一块板。

这避免了在一个函数调用中持续忙等，也使 2 ms 超时能够统一收敛，但当前主循环在传输结束前不会执行本轮图像快照等后续业务。CPU 仍要处理 SCB 中断和状态轮询；当前没有 DMA。

### 10.5 静态缓冲、去重与双核分工

- 主机只有一组 `97 B TX + 97 B RX` 静态缓冲，两块板串行复用。
- payload 在固定数组中构建，没有动态内存分配。
- `result_sequence` 未变化时不重复发布图像结果。
- `fresh_mask` 和 `changed_mask` 只标记真实新结果或超时清空。
- CM7_1 负责外设和图像汇总，CM7_0 只消费共享快照。当前两个核均关闭 D-Cache，IPC 代码仍保留共享结构的 clean/invalidate 调用，便于以后调整缓存策略时维持接口边界。

### 10.6 失效隔离

- 单次 transfer 超时：`2 ms`，用 DWT 实际周期与最多 100000 次 service 轮询双重限制；
- 图像超过 `50 ms` 没有新序号：清除该板目标，防止继续使用旧坐标；
- 链路超过 `100 ms` 没有合法响应：该板标记离线；
- 某块板超时后只中止当前传输并复位 SCB，不让它永久卡住 CM7_1 或另一块板。

## 11. 数据进入 Air 双核后的去向

CM7_1 将 SPI 数据解析到 `image_data[Front/Center/Back]`：board 0 -> `Front`，Air 下视摄像头 -> `Center`，board 1 -> `Back`。三路数据经共享 SRAM 和 IPC 发布给 CM7_0。CM7_0 后续执行相机模型、信标/车灯融合、规划与控制；SPI 层不做跨相机融合。

反向路径为：

```text
CM7_0 飞行状态、姿态、高度、图像开关、参数请求
    -> IPC/共享 SRAM
    -> CM7_1 CameraSpi
    -> 两块 2BL3
```

## 12. Car-F 能看到哪些 SPI 数据

Car-F 不接收两块图像板的原始 77 字节 SPI payload。Air CM7_1 先把每块板的诊断状态写入共享日志，CM7_0 再把部分日志放入 52 个 float 的 UART `RUN_DATA` 诊断帧：

| Air `RUN_DATA` 索引 | Car-F 含义 |
| ---: | --- |
| 45 | board 0：bit0 online，bit1 READY0 电平 |
| 46 | board 1：bit0 online，bit1 READY1 电平 |
| 47 | 高 8 bit=board 0 last_error，低 8 bit=board 1 last_error |
| 48 | board 0 最近 RX head0，正常应为 `0xAA` |
| 49 | board 0 最近 RX head1，正常应为 `0x55` |
| 50 | board 1 最近 RX head0，正常应为 `0xAA` |
| 51 | board 1 最近 RX head1，正常应为 `0x55` |

Car-F 菜单的 `2BL3 Status` 页面据此显示 UART 在线/新鲜度、两板 online、READY、错误码和最近帧头。飞行期间 Air 只发送 17 个关键 `RUN_DATA` float，不包含索引 45..51；常态诊断帧才发送完整 52 个 float。

Car-F 发起图像参数操作的完整路径是：

```text
Car-F 菜单
  -> UART3 / AirComm
  -> Air UART2 / CM7_0
  -> IPC 参数请求
  -> Air CM7_1 / CameraSpi
  -> 两块 2BL3
  -> 参数 ACK 原路返回
```

## 13. 校验、错误码与在线判断

Air 接收响应后依次检查 board_id、`AA 55` 帧头、`0x20` 命令、89 字节 payload、`0xED` 帧尾、CRC16、app_len，以及图像版本或参数 ACK 字段。当前错误码为：

| 值 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | `OK` | 合法响应 |
| 1 | `INVALID_BOARD` | board_id 越界 |
| 2 | `NOT_READY` | 100 ms 未形成有效链路 |
| 3 | `TRANSFER_BUSY` | 上一次传输未结束又尝试启动 |
| 4 | `HW` | SCB 启动传输失败 |
| 5 | `TIMEOUT` | transfer 超过 2 ms 或轮询上限 |
| 6 | `HEAD` | 帧头错误 |
| 7 | `CMD` | 命令错误 |
| 8 | `LEN` | 外层 payload 长度错误 |
| 9 | `TAIL` | 帧尾错误 |
| 10 | `CRC` | CRC16 错误 |
| 11 | `APP_LEN` | 应用长度、版本或参数 ACK 格式错误 |

只有完整校验通过后，Air 才将该板设为 online、清零链路年龄并增加 `rx_ok_count`。错误响应不会覆盖上一份有效图像快照。

## 14. 调试与逻辑分析仪检查清单

### 14.1 上电静态检查

- 两路 CS 空闲时均为高；
- 图像板没有数据时 READY 为低，准备好响应后 READY 为高；
- Air READY 输入断线时因内部下拉保持低；
- 两块图像板与 Air 共地；
- 没有同时初始化 PMW3901 或 UART6 去抢占 `P03_0~P03_3/SCB6`。

### 14.2 波形检查

- SCK 空闲为低，在第一个有效边沿采样，符合 Mode 0；
- SCK 频率约 2 MHz；
- 单次 CS 低电平内有 `97 x 8 = 776` 个时钟；
- 理想 CS 低电平时间约 388 us；
- 任意时刻只能有一个 CS 为低；
- MOSI 开头应为 `AA 55 20 00 53`，其中 `0x0053=83`；
- MISO 合法响应开头应为 `AA 55 20 00 59`，其中 `0x0059=89`；
- 响应最后一字节应为 `ED`。

### 14.3 软件诊断

- Car-F `2BL3 Status` 页面 `S:11` 表示两块板均 online；
- `R:11` 表示两路 READY 当前都为高，不等同于 online；
- 正常帧头应显示 `H:AA55/AA55`；
- READY 一直为低时，先查 READY 接线和从机是否准备响应，因为主机不会强制发送；
- READY 为高但帧头不是 `AA55` 时，重点检查 MISO、CS 对应关系、SPI Mode 和两块从机是否同时驱动 MISO。

## 15. 当前实现的边界

- `2 MHz`、`388 us` 和总线占用率是代码配置与理论线时，不能替代硬件误码率和 CPU 占用实测。
- Camera SPI 当前没有 DMA；工程包含 DMA 驱动不等于本链路使用了 DMA。
- 当前调度依赖 READY，无法在 READY 永久为低时主动下发恢复命令。
- 双板顺序固定为 board 0 后 board 1，没有 round-robin 公平调度。
- Car-F 只获得 SPI 诊断摘要和 Air 处理后的业务数据，不透明转发 77 字节原始 payload。
- 引脚表描述 MCU 端口号；PCB 连接器编号、电平和供电必须再对照硬件原理图。

以上边界均来自当前实现，不应在未修改代码并验证前写成其他行为。
