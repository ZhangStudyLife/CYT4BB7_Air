#include "air_remote_cmd.h"

void air_remote_cmd_init(void)
{
    /* 当前远程命令主链路在 air_comm_air_init() 中初始化，这里保留空实现兼容旧调用。 */
}

uint8 air_remote_cmd_register_polling(const char *name, air_remote_cmd_fn run)
{
    return air_comm_air_register_polling_command(name, run);
}

uint8 air_remote_cmd_register_instant(const char *name, air_remote_cmd_fn run)
{
    return air_comm_air_register_instant_command(name, run);
}

void air_remote_cmd_update_100HZ(void)
{
    /* 当前远程命令由 air_comm_air_update_100HZ() 统一调度。 */
}
