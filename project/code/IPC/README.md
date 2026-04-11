# IPC 核间通信模块

CM7_1 图像处理结果 → CM7_0 飞控核，50Hz。

## 原理

1. 共享内存（`.global_ram_data` 段，地址 0x28001000）存放 `ipc_image_payload_t` 结构体
2. CM7_1 每帧写入共享内存后，通过 `ipc_send_data(seq)` 发送帧序号通知
3. CM7_0 在 IPC 中断回调中置标志，主循环 50Hz 槽中读取共享数据

## 接口

| 函数 | 核 | 作用 |
|------|----|------|
| `ipc_image_send()` | CM7_1 | 写共享内存 + 发送通知 |
| `ipc_image_callback()` | CM7_0 | IPC 回调，置新数据标志 |
| `ipc_image_is_new()` | CM7_0 | 查询并清除新数据标志 |
| `ipc_image_get(out)` | CM7_0 | 拷贝共享数据到本地 |

## 注意

- 两核均需 `SCB_DisableDCache()` 避免缓存一致性问题
- 条件编译宏：`CY_CORE_CM7_0` / `CY_CORE_CM7_1`
