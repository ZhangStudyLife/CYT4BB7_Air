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
