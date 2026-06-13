# IPC共享内存地址不一致Bug修复记录

## 背景

4BB7 双核系统中，CM7_1 负责通过 SPI 读取两块 2BL3 图像板数据，并写入共享内存
`g_ipc_camera_spi_log`；CM7_0 负责从共享内存读取该日志并通过 WiFi JustFloat 发给上位机。

问题现象是：CM7_1 调试时可以看到 `camera_spi_publish_log()` 正常执行，SPI 成功计数持续增长；
但 CM7_0 通过上位机看到的 `seq`、`online`、`rx_ok_count`、`rx_error_count` 等字段全部为 0。

## 正常对照日志

对照日志：

`D:\Downloads\回退到节点c2a74b46065afaed以后1347端口的数据.csv`

快速统计结果：

- 共 2847 行、27 列。
- `I0` 从 6440ms 到 34900ms，持续约 28.46s。
- 相邻两行 `I0` 间隔固定为 10ms，无跳变，日志频率稳定为 100Hz。
- `I3` 全程为 1，0 号图像板在线。
- `I14` 全程为 1，1 号图像板在线。
- `I1` 单调递增，说明主循环日志持续运行。
- `I2` 全程为 1，说明 WiFi 日志链路正常。

结论：该日志没有明显问题，可作为 SPI/IPC 正常工作的对照样本。

## 根因

调试发现两个核心看到的同一个全局变量地址不同：

| 核心  | `&g_ipc_camera_spi_log` |
| ----- | ------------------------- |
| CM7_1 | `0x28001000`            |
| CM7_0 | `0x28001060`            |

CM7_1 写入的是 `0x28001000`，CM7_0 读取的是 `0x28001060`，两边不是同一块内存。
因此 CM7_0 读到全 0 不是 SPI 通信失败，也不是 `camera_spi_publish_log()` 没有执行，而是共享变量地址不一致。

触发原因是原代码把多个共享变量都放在同一个 `.global_ram_data` section：

```c
#pragma location=".global_ram_data"
volatile ipc_image_payload_t g_ipc_image_shared;
#pragma location=".global_ram_data"
volatile ipc_camera_spi_log_t g_ipc_camera_spi_log;
```

两个核心工程的链接输入顺序不同，导致 `.global_ram_data` 内部变量排列顺序不同。
结果就是同名变量在 CM7_0 和 CM7_1 中被链接到了不同偏移地址。

## 修复方式

将两个共享变量拆到独立 section，并在 IAR 链接脚本中固定绝对地址。

`project/code/IPC/ipc_image_data.c`：

```c
#pragma location=".ipc_image_shared"
volatile ipc_image_payload_t g_ipc_image_shared;
#pragma location=".ipc_camera_spi_log"
volatile ipc_camera_spi_log_t g_ipc_camera_spi_log;
```

`project/iar/icf/linker_directives_tviibh.icf`：

```icf
place at address mem:0x28001000 { section .ipc_image_shared };
place at address mem:0x28001060 { section .ipc_camera_spi_log };
place in ICFEDIT_region_RAM { section .global_ram_data };
```

其中 `0x28001060` 是 `g_ipc_image_shared` 后面的固定位置，避免两个核心因 section 内部排序不同产生地址偏移。

## 验证方法

重新编译并烧录 CM7_0 和 CM7_1 后，在两个核心分别查看：

```c
&g_ipc_camera_spi_log
g_ipc_camera_spi_log.seq
```

期望结果：

- CM7_0 和 CM7_1 的 `&g_ipc_camera_spi_log` 都应为 `0x28001060`。
- CM7_1 中 `g_ipc_camera_spi_log.seq` 持续递增。
- CM7_0 通过 `ipc_camera_spi_log_get()` 读取到的 `seq` 也持续递增。
- 上位机日志中两块板的 `online`、`rx_ok_count`、`last_error` 等字段不再全部为 0。

## 经验

双核共享内存中，不能依赖“同一个 section 内多个变量的链接顺序”来保证地址一致。
只要两个核心工程的编译文件或链接顺序不同，同名变量就可能落到不同偏移。

后续新增双核共享变量时，应优先使用以下方式之一：

- 每个共享变量使用独立 section，并在链接脚本中固定地址。
- 或者只定义一个共享内存总结构体，内部字段顺序由同一个头文件统一描述。
