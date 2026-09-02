/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
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
    /* 当前远程命令由 air_comm_air_update_200HZ() 统一调度。 */
}
