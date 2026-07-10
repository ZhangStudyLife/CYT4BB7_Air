# Core 0 与 MT9V03X SRAM 隔离设计

## 问题

Core 1 的 MT9V03X 采集驱动把 22560 字节临时帧固定在
`0x28026024-0x2802B843`。Core 0 当前把
`0x28020000-0x2805FFFF` 整段作为普通 SRAM，导致 JustFloat 队列被链接到
`0x280207C8-0x2802FFC7`，与完整摄像头帧窗口重叠。

Core 0 启动代码还会根据链接器导出的 SRAM 起止地址执行连续 ECC 初始化，
因此只在普通 section 分配中挖洞，仍可能在启动阶段写入摄像头窗口。

## 方案比较

1. 将 Core 0 普通 SRAM 整体移到摄像头窗口之后。修改范围最小，普通变量和
   ECC 初始化都不会触碰摄像头窗口，但放弃窗口之前约 24 KB SRAM。
2. 把 Core 0 SRAM 拆成前后两段。可以保留约 24 KB，但现有启动代码只有一组
   SRAM ECC 起止地址，需要同时修改公共 SDK 启动逻辑，风险和修改范围更大。
3. 移动 MT9V03X 临时帧。固定地址属于逐飞采集协议的一部分，仓库中没有可配置
   采集端实现，无法证明移动后硬件仍会向新地址写入，因此不采用。

采用方案 1。

## 链接布局

- MT9V03X 固定帧：`0x28026024-0x2802B843`，保持不变。
- Core 0 普通 SRAM：`0x2802B880-0x2805FFFF`。
- `0x2802B844-0x2802B87F` 作为对齐填充，不分配。
- Core 1 普通 SRAM 起点仍为 `0x28060000`，保持不变。
- IPC 固定共享区 `0x28001000` 等地址保持不变。

Core 0 新区域大小为 214912 字节。当前 map 的 readwrite 总量为 97887 字节，
仍保留约 117 KB 余量。新起点按 128 字节对齐，同时满足 CM7 向量表和 8 字节
ECC 初始化要求。

## 修改范围

只修改 `project/iar/icf/linker_directives_tviibh.icf`：

- 将 `_base_SRAM_CM7_0` 设置为 `0x2802B880`。
- 根据 Core 0 原物理分区终点 `0x28060000` 重新计算 `_size_SRAM_CM7_0`。

不修改 MT9V03X 驱动、JustFloat 队列、启动 ECC 代码或业务代码。

## 验证标准

重新使用 IAR 编译两个核心后：

1. Core 0 map 中 SRAM placement 范围必须是
   `0x2802B880-0x2805FFFF`。
2. Core 0 的 `.data`、`.bss`、`.noinit`、heap、stack 均不得进入
   `0x28026024-0x2802B843`。
3. Core 0 的 `__ecc_init_sram_start_address` 必须为 `0x2802B880`，结束地址必须为
   `0x2805FFFF`。
4. Core 1 的 `mt9v03x_image_temp` 必须仍为
   `0x28026024-0x2802B843`。
5. JustFloat 队列容量和发送逻辑保持不变，运行时遥测与中心摄像头图像均正常。
