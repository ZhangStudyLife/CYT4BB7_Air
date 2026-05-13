#ifndef AIR_REMOTE_CMD_H
#define AIR_REMOTE_CMD_H

#include "zf_common_headfile.h"

typedef enum
{
    AIR_REMOTE_CMD_MODE_POLLING = 0,
    AIR_REMOTE_CMD_MODE_INSTANT
} air_remote_cmd_mode_t;

typedef uint8 (*air_remote_cmd_start_fn)(void);
typedef void (*air_remote_cmd_poll_fn)(void);
typedef void (*air_remote_cmd_stop_fn)(void);

/* 初始化远程命令注册表，并接管 AirComm 的 0x03 函数名命令回调。 */
void air_remote_cmd_init(void);

/* 注册远程命令：轮询型提供 poll/stop，立即型可用 poll 做非阻塞状态机。 */
uint8 air_remote_cmd_register(const char *name,
                              air_remote_cmd_mode_t mode,
                              air_remote_cmd_start_fn start,
                              air_remote_cmd_poll_fn poll,
                              air_remote_cmd_stop_fn stop);

/* 100Hz 调度入口：刷新轮询型函数，推进立即型函数状态机。 */
void air_remote_cmd_update_100HZ(void);

#endif
