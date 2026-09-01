# IPC 回调不触发 Bug 修复记录

## 背景

三摄融合移植过程中，将 IPC 共享数据结构从 `ipc_image_payload_t`（~96 字节，含 beacons/car_lamps 数组）替换为 `ipc_mode2_payload_t`（32 字节，7 个 float + seq）。同时删除了核0 侧的 `ipc_image_poll()` 轮询函数，改为纯回调驱动。

## 问题现象

- CM7_1 debug 观察：`g_ipc_mode2_shared.seq` 持续递增，`ipc_mode2_send()` 正常执行
- CM7_0 debug 观察：`g_ipc_mode2_shared.seq` 也能看到递增（共享内存可见），但 `g_air_mode2_seq` 及所有 `g_air_mode2_*` 全局变量始终为 0
- CM7_0 在 `ipc_image_callback()` 打断点，**从未命中**

结论：共享内存读写正常，但 IPC Pipe 中断通知从 CM7_1 到 CM7_0 没有触发。

## 排查过程

### 1. 排除共享内存地址不一致

参考上一次 Bug（见 `IPC共享内存地址不一致Bug修复记录.md`），在两个核分别查看 `&g_ipc_mode2_shared`，均为 `0x28001000`，地址一致。

### 2. 排除 IPC 端口配置错误

确认 `zf_driver_ipc.h` 中枚举值：`IPC_PORT_1 = 0`，`IPC_PORT_2 = 1`。

| 核 | init 端口 | 自身 endpoint | 发送目标 |
|----|-----------|---------------|----------|
| CM7_0 | IPC_PORT_1 (0) | 0 | 1 |
| CM7_1 | IPC_PORT_2 (1) | 1 | 0 |

`ipc_send_data` 中 `Cy_IPC_Pipe_SendMessage(ipc_port_save == 0 ? 1 : 0, ...)` 路由正确。

### 3. 排除代码逻辑错误

`ipc_image_callback` 函数名、条件编译宏 `CY_CORE_CM7_0`、回调注册方式均未修改。`ipc_mode2_unpack()` 逻辑简单直接，不存在提前 return 的分支。

### 4. 定位根因

IPC Pipe 中断回调不触发的确切原因未能在代码层面定位。可能与 Cypress IPC Pipe 驱动的内部状态、双核启动时序、或中断优先级竞争有关。旧代码之所以工作正常，是因为**回调 + 轮询双路冗余**：即使回调偶尔或始终不触发，`ipc_image_poll()` 在主循环中每次检查 seq 变化并主动解包。

## 修复方式

恢复轮询机制作为回调的冗余备份（3 处改动）。

**`ipc_image_data.c`**：

```c
#if defined(CY_CORE_CM7_0)
static uint32 s_polled_seq = 0U;
// ... 已有的 g_air_mode2_* 变量 ...

// 回调中同步更新 s_polled_seq，防止轮询重复解包
void ipc_image_callback(uint32 ipc_data)
{
    (void)ipc_data;
    SCB_InvalidateDCache_by_Addr(...);
    s_polled_seq = g_ipc_mode2_shared.seq;
    ipc_mode2_unpack();
}

// 轮询函数：主循环调用，检查 seq 变化
void ipc_mode2_poll(void)
{
    SCB_InvalidateDCache_by_Addr(...);
    if (g_ipc_mode2_shared.seq != s_polled_seq)
    {
        s_polled_seq = g_ipc_mode2_shared.seq;
        ipc_mode2_unpack();
    }
}
#endif
```

**`ipc_image_data.h`**：

```c
void ipc_mode2_poll(void);
```

**`main_cm7_0.c`**：在 100Hz 分支中、读取 `g_air_mode2_*` 发送 air_data 之前调用：

```c
ipc_mode2_poll();
```

## 验证结果

修复后 CM7_0 的 `g_air_mode2_target_valid/x/y` 等变量正常更新，车端串口接收到正确的三摄融合数据。

## 经验

1. **双核 IPC 通信不要依赖单一通知路径**。Cypress IPC Pipe 的中断通知在某些条件下可能不触发（启动时序、中断竞争等），轮询共享内存作为冗余备份成本极低但可靠性极高。
2. **共享内存 + seq 自增是天然的轮询友好设计**。只要写入端每次更新 seq，读取端只需比较 seq 即可判断是否有新数据，无需额外标志位。
3. **重构 IPC 时，保留原有的冗余机制**。本次 Bug 的直接原因是移植时认为"纯回调更简洁"而删除了轮询，结果回调路径恰好失效。代码简洁不应以牺牲可靠性为代价。
4. **排查 IPC 问题的优先级**：先查地址一致性 → 再查端口/路由 → 再查回调注册 → 最后看中断是否触发。如果中断不触发且代码无误，优先加轮询兜底而非深入驱动层。
