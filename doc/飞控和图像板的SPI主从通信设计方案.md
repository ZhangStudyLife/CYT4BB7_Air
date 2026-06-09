# 飞控和图像板的 SPI 主从通信设计方案

## 1. 范围与结论

本文是 Gate 1 协议设计文档，只定义 Air CYT4BB7 与两块 CYT2BL3 图像板之间的 SPI 主从通信方案，不修改任何 C/H/IAR/厂商库文件。

结论锁定如下：

- 拓扑：Air CYT4BB7 的 CM7_1 作为 SPI 主机，两块 CYT2BL3 图像板作为 SPI 从机。
- 物理总线：Air 使用 `SPI_3`，两块 2BL3 共用 SCK/MOSI/MISO，各自独立软件 CS 和 READY。
- 线协议：固定 `97B` 单次 transfer；帧头 `AA 55`，命令 `0x20`，长度大端，CRC16 小端，帧尾 `ED`。
- 应用 payload：上行图像应用数据固定 `77B`，内容为 `4 beacon + 1 car_lamp`；下行线帧也携带 `77B` 应用容量，仅 `magic/board_id/counter/flight_state/image_tcp/image_display` 有效，其余清零；请求 meta 的 `app_len` 固定填当前有效字段长度 `9B`，不得填 `77B`。
- 板号策略：只允许 `board_id=0/1`，废弃旧三从机、三相机融合和 `board_id=2` 业务路径。
- 调度策略：CM7_1 主循环非阻塞轮询 `READY/downlink mask`，两板 round-robin，独立软件 CS。
- 飞行状态：CM7_0 通过 IPC 通知 CM7_1；CM7_1 再下发给 2BL3；飞行中 Air 与 2BL3 调试屏都不刷新。

本文不采用旧的 `12B/32B` 手册或旧三从机说明。后续实现以本文的 `97B transfer` 和 `77B app_data` 为唯一规格。

## 2. 系统拓扑与职责

```text
Air CYT4BB7 CM7_0
    └─ IPC: flight_state
Air CYT4BB7 CM7_1
    ├─ 下视摄像头图像算法与 Air 本地 IPS114 调试显示
    ├─ SPI_3 主机，轮询两块 CYT2BL3 图像板
    └─ 将 flight_state、board_id、调试开关下发给 2BL3

CYT2BL3 board 0
    └─ 图像算法 + SPI 从机，只输出本板图像结果

CYT2BL3 board 1
    └─ 图像算法 + SPI 从机，只输出本板图像结果
```

Air CM7_0 只负责飞控主循环和通过 IPC 发布飞行状态，不初始化、不轮询 IPS114，不初始化、不轮询外置 2BL3 SPI。Air CM7_1 是外置图像板 SPI 链路的唯一主机。

车端旧链路只作为历史依据：旧小车 CM7_0 通过 `CameraSpi` 读取三块 2BL3 并在 `beacon_fusion` 中做三相机融合。总目标要求车端移除该链路，所以新方案不得把旧三相机融合搬到 Air，也不得保留第三块从机概念。

## 3. 引脚分配

Air CYT4BB7 主机侧：

| 信号 | Air 资源 | 引脚 | 说明 |
| --- | --- | --- | --- |
| SCK | `SPI_3` | `P03_2` | 两块 2BL3 共用 |
| MOSI | `SPI_3` | `P03_1` | Air 主出从入 |
| MISO | `SPI_3` | `P03_0` | Air 主入从出 |
| CS0 | GPIO 软件 CS | `P03_3` | 低电平选中 board 0 |
| CS1 | GPIO 软件 CS | `P03_4` | 低电平选中 board 1 |
| READY0 | GPIO 输入 | `P01_0` | board 0 数据就绪，高电平有效 |
| READY1 | GPIO 输入 | `P01_1` | board 1 数据就绪，高电平有效 |

2BL3 从机侧：

| 信号 | 2BL3 资源 | 引脚 | 说明 |
| --- | --- | --- | --- |
| SCK | `SCB0 SPI CLK` | `P0_2` | 接 Air `P03_2` |
| MOSI | `SCB0 SPI MOSI` | `P0_1` | 接 Air `P03_1` |
| MISO | `SCB0 SPI MISO` | `P0_0` | 接 Air `P03_0` |
| CS | `SCB0 SPI SELECT0` | `P0_3` | 分别接 Air `P03_3/P03_4` |
| READY | GPIO 输出 | `P18_6` | 分别接 Air `P01_0/P01_1` |

Air 侧 `SPI_3` 必须归属 CM7_1 的 2BL3 主机链路。当前 Air 工程中 `PMW3901` 也声明过 `SPI_3/P03_1/P03_0/P03_2/P03_3`，后续实现不得在 CM7_0 或其他路径同时初始化该外设，否则会抢线。

当前 Air CM7_0 通过 `air_comm_air_init()` / `air_comm_air_update_100HZ()` 走到 `AirComm` 屏幕命令路径，`air_comm_air.c` 内部调用 `ips114_*`，CM7_0 IAR 工程也引用 IPS114 库。后续实现必须先清理或迁移这条 CM7_0 屏幕路径，达到“CM7_0 不初始化、不调用 IPS114”的硬约束。

## 4. 线协议

单次 SPI 传输固定 clock `97B`。SPI 模式沿用现有代码的 Motorola `CPOL0/CPHA0`、8 bit、MSB first。CS 由软件独立控制，空闲为高，选中为低。

帧布局：

| 字段 | 字节数 | 字节序 | 说明 |
| --- | --- | --- | --- |
| head | 2 | 固定 | `0xAA 0x55` |
| cmd | 1 | 固定 | `0x20`，同步数据命令 |
| len | 2 | BE | 后续 frame payload 字节数，不含 head/cmd/len/crc/tail |
| frame payload | `len` | 按字段定义 | 协议 meta + `77B app_data` |
| crc16 | 2 | LE | CRC16 覆盖 `cmd + len + frame payload` |
| tail | 1 | 固定 | `0xED` |

CRC16 使用现有 `CameraSpi` 代码中的算法：初值 `0xFFFF`，右移，异或多项式 `0xA001`。CRC 写入顺序为低字节在前、高字节在后。

`77B` 指应用数据容量 `app_data`，不是线协议 `len`。线协议中：

| 方向 | frame payload 组成 | frame payload len | 有效帧长 | 实际 clock |
| --- | --- | --- | --- | --- |
| Air -> 2BL3 请求 | `6B request_meta + 77B app_data` | `83B` | `91B` | `97B` |
| 2BL3 -> Air 响应 | `12B response_meta + 77B app_data` | `89B` | `97B` | `97B` |

请求方向有效帧长为 `91B`，但主机仍 clock `97B`，尾部 6 字节填充不参与 CRC，不承载协议语义。从机解析时按 `len=83` 找到帧尾，忽略固定 transfer 的尾部填充。响应方向刚好完整占满 `97B`。

## 5. 协议 meta

请求 meta，即 Air 下发给 2BL3 的 frame payload 前 6 字节：

| 偏移 | 字节数 | 字节序 | 字段 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 4 | LE | sequence | Air 对该 board 的下行序号 |
| 4 | 2 | LE | app_len | 本次有效下行应用数据长度，固定填 `9B` |
| 6 | 77 | - | app_data | 下行应用 payload 容量 |

响应 meta，即 2BL3 上行给 Air 的 frame payload 前 12 字节：

| 偏移 | 字节数 | 字节序 | 字段 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 4 | LE | sequence | 2BL3 上行序号 |
| 4 | 4 | LE | ack_sequence | 已接收的 Air 下行序号 |
| 8 | 2 | LE | app_len | 本次有效上行应用数据长度，固定使用 `77B` |
| 10 | 1 | - | flags | bit0: uplink pending；bit1: downlink new |
| 11 | 1 | - | peer_last_error | 2BL3 最近一次协议错误码 |
| 12 | 77 | - | app_data | 上行应用 payload 容量 |

Air 只在收到合法响应并看到 `ack_sequence >= 本板下行 sequence` 后，清除该 board 的 `downlink_mask`。CRC、帧头、命令、长度、尾字节任一错误时，本次响应不得更新最新图像快照。

注意：`app_len` 描述应用层本次需要拷贝给业务代码的有效字节数，不等于线帧中的 `77B app_data` 容量。当前下行有效字段到 `image_display` 为止，最后有效偏移是 8，所以 Air 请求 meta 的 `app_len` 必须填 `9B`。填 `77B` 会导致现有 2BL3 应用层 `12B` 接收缓冲溢出；填 `12B` 没有新增语义，也会把保留字节误当有效字段边界。线帧仍然按 `6B request_meta + 77B app_data` 计算 CRC 并 clock `97B`。

## 6. 上行 77B 图像 payload

上行应用 payload 固定 `77B`，布局沿用现有 `camera_spi_types.h` 的图像包定义：

```text
77B = 4B header + 4 * 13B beacon_slot + 1 * 21B car_lamp_slot
```

Header：

| 偏移 | 字节数 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | 1 | version | 图像协议版本，当前为 `2` |
| 1 | 1 | beacon_count | 有效 beacon 数，最大 4 |
| 2 | 1 | car_lamp_count | 有效 car_lamp 数，最大 1 |
| 3 | 1 | reserved | 保留，发送端清零 |

每个 beacon slot 共 `13B`：

| slot 内偏移 | 字节数 | 字节序 | 字段 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 1 | - | valid | 0/1 |
| 1 | 4 | LE | x | `float`，图像中心为原点 |
| 5 | 4 | LE | y | `float`，图像中心为原点 |
| 9 | 4 | LE | radius | `float`，等效半径，单位像素 |

car_lamp slot 共 `21B`：

| slot 内偏移 | 字节数 | 字节序 | 字段 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 1 | - | valid | 0/1 |
| 1 | 4 | LE | cx | `float`，图像中心为原点 |
| 5 | 4 | LE | cy | `float`，图像中心为原点 |
| 9 | 4 | LE | width | `float` |
| 13 | 4 | LE | length | `float` |
| 17 | 4 | LE | angle | `float`，角度 |

2BL3 只负责填充本板图像算法结果，不做跨板融合。Air CM7_1 只保存 board 0/1 的最新快照，后续是否融合由 Air 侧单独设计，不复用车端旧三相机 `beacon_fusion`。

## 7. 下行 77B 控制 payload

下行应用 payload 在线帧中同样固定占 `77B` 容量。当前有效字段如下，其余字节必须清零；请求 meta 的 `app_len` 固定为 `9B`，只让 2BL3 应用层拷贝这些有效字段。

| 偏移 | 字节数 | 字节序 | 字段 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 1 | - | magic | 固定 `0x5A` |
| 1 | 1 | - | board_id | 只允许 `0` 或 `1` |
| 2 | 4 | LE | counter | Air 下行应用计数器 |
| 6 | 1 | - | flight_state | 0=未飞行，1=飞行中 |
| 7 | 1 | - | image_tcp | 0=2BL3 不发 WiFi/TCP 图像调试，1=允许 |
| 8 | 1 | - | image_display | 0=2BL3 不刷新本地 IPS114，1=允许 |
| 9..76 | 68 | - | reserved | 发送端清零，接收端忽略 |

`board_id` 由 Air CM7_1 按物理 CS 通道下发：CS0 对应 `board_id=0`，CS1 对应 `board_id=1`。2BL3 固件刷入两块板时不靠编译期差异区分身份，只接受 Air 下发的 `board_id`。

`flight_state=1` 时，2BL3 必须停止本地 IPS114 调试屏刷新；此规则优先于 `image_display`。Air CM7_1 自身也必须在飞行中停止 IPS114 调试屏刷新。`image_tcp` 只控制 2BL3 的图像调试传输，不改变 SPI 上行图像结果。

## 8. 两板策略

新系统只有两块外置 CYT2BL3 图像板：

- `board_id=0`：Air CS0=`P03_3`，READY0=`P01_0`。
- `board_id=1`：Air CS1=`P03_4`，READY1=`P01_1`。

协议层、硬件层、调度层和上层快照数组都只允许两个槽位。旧代码中的 `CAMERA_SPI_BOARD_COUNT=3`、`CAMERA_SPI_SLAVE_COUNT=3`、`BEACON_FUSION_CAMERA_COUNT=3`、`board[2]`、第三路 WiFi 端口、第三路 CS/READY 均属于后续清理对象，不进入新方案。

不得保留“第三块板暂时不用”的兼容逻辑。留着这种影子路径，后面一接线就是玄学调试，没必要给自己挖坑。

## 9. Air CM7_1 调度

CM7_1 在主循环空闲路径调用 SPI poll，不能绑定 `mt9v03x_finish_flag`。下视摄像头出帧和外置 2BL3 轮询是两条独立链路。

推荐状态机：

1. 若已有传输进行中：只检查硬件完成标志或软件超时；完成后拉高当前 CS，解析 `97B` RX，更新对应 board 快照或错误计数，然后立即返回主循环。
2. 若空闲：读取 READY0/READY1 电平，形成 `ready_mask`；将 `ready_mask | downlink_mask` 作为调度候选。
3. 用 round-robin 从候选中选择下一块 board，避免一块板 READY 高频占满总线。
4. 根据 board 构造下行 payload：`magic=0x5A`、`board_id=0/1`、递增 `counter`、当前 `flight_state`、`image_tcp`、`image_display`，剩余清零，并把请求 meta 的 `app_len` 固定写为 `9B`。
5. 拉低对应软件 CS，启动一次固定 `97B` SPI transfer，然后立即返回主循环。
6. 只有该 board 被选中时才拉低其 CS；任意时刻最多一个 CS 为低。

READY 为高表示 2BL3 有上行数据待取。即使 READY 为低，只要该 board 的 `downlink_mask` 置位，Air 也要安排一次 transfer，确保 board_id、飞行状态和调试开关能下发。

## 10. 飞行状态与调试屏

当前 Air IPC 已有飞行状态方向：

- CM7_0 通过 `ipc_flight_state_send(flying)` 发送飞行状态。
- CM7_1 在 IPC 回调中记录状态，`ipc_core0_is_flying()` 返回该状态。
- CM7_1 现有下视调试显示已经以 `ipc_core0_is_flying()==0` 为刷新条件。

新 SPI 主机必须复用这条 IPC 状态，不让 CM7_0 直接接触 IPS114 或外置 SPI。CM7_1 将该状态写入下行 `flight_state`。当 `flight_state=1`：

- Air CM7_1 不刷新本地 IPS114 调试屏。
- 2BL3 不刷新本地 IPS114 调试屏。
- SPI 主从通信继续运行，保证图像结果仍可上行。

## 11. 当前代码与 IAR 引用依据

已用 `rg` 阅读并确认的关键依据：

- 车端旧主机协议：`CYT4bb7_Car/project/code/Protocols/CameraSpi/camera_spi_protocol.h` 定义 `AA55/cmd=0x20/len BE/CRC16 LE/ED`、`REQ_META=6`、`RESP_META=12`、固定 transfer 来自 `RESP_FRAME_LEN`。
- 车端旧主机实现：`CYT4bb7_Car/project/code/Protocols/CameraSpi/camera_spi.c` 使用 `ready_mask | downlink_mask`、round-robin、非阻塞传输和独立 CS。
- 车端旧硬件：`CYT4bb7_Car/project/code/HW_Drivers/CameraSpi/camera_spi_hw.c/.h` 是旧 `SPI_0/SCB7`、三从机 CS/INT，只作为历史参考，不进入 Air 新引脚。
- 车端旧融合：`CYT4bb7_Car/project/code/Estimation/Image/beacon_fusion.[ch]` 明确是三相机融合，后续应清理而不是迁移。
- 2BL3 从机协议：`CYT2BL3_Image/project/code/Protocols/CameraSpi/*` 与主机协议同源，`camera_spi_slave_config.h` 锁定 `SCB0` 与 `P0_0/P0_1/P0_2/P0_3/P18_6`。
- 2BL3 应用层：`CYT2BL3_Image/project/user/main_cm4.c` 已打包 `4 beacon + 1 car_lamp` 上行，并接收 `magic/board_id/counter/flight_state/image_tcp/image_display` 下行字段；当前仍有 `board_id < 3` 旧逻辑，后续实现需收敛到 0/1。
- Air CM7_1：`CYT4BB7_Air/project/user/main_cm7_1.c` 当前负责下视图像、IPC 上发、IPS114 调试显示，并在飞行中不刷新调试屏。
- Air CM7_0 屏幕残留：`CYT4BB7_Air/project/user/main_cm7_0.c` 当前调用 `air_comm_air_init()` / `air_comm_air_update_100HZ()`，`CYT4BB7_Air/project/code/Protocols/AirComm/air_comm_air.c` 内部调用 `ips114_*`；这与目标硬约束冲突，后续实现必须清理或迁移。
- Air IPC：`CYT4BB7_Air/project/code/IPC/ipc_image_data.[ch]` 已提供 CM7_0 -> CM7_1 的 `ipc_flight_state_send()` / `ipc_core0_is_flying()`。
- Air SPI_3 引脚：`CYT4BB7_Air/libraries/zf_driver/zf_driver_spi.h/.c` 支持 `SPI3_CLK_P03_2`、`SPI3_MOSI_P03_1`、`SPI3_MISO_P03_0`、`SPI3_CS0_P03_3`、`SPI3_CS1_P03_4`，底层映射到 `SCB6`。
- Air PMW3901 资源冲突：`CYT4BB7_Air/project/code/HW_Drivers/PMW3901/PMW3901.h` 使用 `SPI_3/P03_1/P03_0/P03_2/P03_3`，CM7_0 IAR 工程仍引用 `PMW3901.c/.h`；后续实现必须确保它不与 2BL3 主机链路同时占用 `SPI_3`。
- IAR 引用：车端 CM7_0 工程仍引用旧 `CameraSpi` 与 `beacon_fusion`；2BL3 工程引用 `camera_spi_slave` 和 IPS114；Air CM7_0 工程引用 `AirComm`、PMW3901 和 IPS114；Air CM7_1 工程引用 `main_cm7_1.c`、IPC 和 IPS114；当前没有 Air `CameraSpi` 模块引用。

## 12. 验证计划与限制

Gate 1 验证：

- 用 `rg` 核对本文所有固定事实：`97B`、`77B`、下行 `app_len=9B`、`AA55`、`cmd=0x20`、CRC16、2BL3 引脚、Air `SPI_3` 引脚、IPC 飞行状态、IAR 工程引用。
- 文档中不得采用旧 `12B/32B` 协议规格，不得保留三从机设计，不得留下占位项。
- 文档只新增本 Markdown 文件，不改 C/H/IAR/厂商库。

后续硬件验证建议：

- 逻辑分析仪同时观察 `SCK/MOSI/MISO/CS0/READY0`，再观察 `CS1/READY1`。重点确认单次 CS 低电平期间 clock 为 `97B`，帧头为 `AA 55`，命令为 `0x20`，尾字节为 `ED`。
- 分别拉起 board 0 和 board 1 的 READY，确认 Air round-robin，不会长时间饿死另一块板。
- 检查下行 payload：board 0 只收到 `board_id=0`，board 1 只收到 `board_id=1`，`flight_state` 随 CM7_0 IPC 状态变化。
- 检查 Air 请求 meta：`app_len` 必须为 `9B`，同时逻辑分析仪上仍能看到固定 `97B` transfer。
- 飞行状态置 1 后，确认 Air 和两块 2BL3 的 IPS114 都不刷新，但 SPI 上行仍持续。
- 人为破坏 CRC、帧头或长度时，确认 Air 不更新最新图像快照，并记录错误。

限制说明：当前受限于硬件环境，本文没有实际测试代码可行性，也没有逻辑分析仪实测波形；Gate 1 只完成协议和调度设计锁定。
