#ifndef AIR_REMOTE_CMD_H
#define AIR_REMOTE_CMD_H

#include "zf_common_headfile.h"

typedef void (*air_remote_cmd_fn)(void);

/*
 * 兼容旧文件名的远程命令接口。
 * 当前主链路由 air_comm_air.c 直接处理 0x03 远程命令，这里只转发到统一注册接口，
 * 避免保留旧 start/poll/stop 模型造成后续维护混淆。
 */
void air_remote_cmd_init(void);
uint8 air_remote_cmd_register_polling(const char *name, air_remote_cmd_fn run);
uint8 air_remote_cmd_register_instant(const char *name, air_remote_cmd_fn run);
void air_remote_cmd_update_100HZ(void);

#endif
