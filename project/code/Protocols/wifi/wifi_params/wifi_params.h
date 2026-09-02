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
/*****************************************************************************
 * 文件: wifi_params.h
 * 模块: WiFi 参数调节
 * 职责: 处理参数注册、查询、修改、保存、加载与待机态遥测开关命令
 *****************************************************************************/

#ifndef WIFI_PARAMS_H
#define WIFI_PARAMS_H

#include "zf_common_headfile.h"

/* WiFi 参数最近一次处理结果码 */
typedef enum
{
    WIFI_PARAMS_RESULT_OK = 0,             /* 执行成功 */
    WIFI_PARAMS_RESULT_ERR_FORMAT = 1,     /* 命令格式错误 */
    WIFI_PARAMS_RESULT_ERR_UNKNOWN_CMD = 2,/* 未知命令 */
    WIFI_PARAMS_RESULT_ERR_UNKNOWN_PARAM = 3,/* 未知参数 */
    WIFI_PARAMS_RESULT_ERR_RANGE = 4,      /* 参数超范围 */
    WIFI_PARAMS_RESULT_ERR_STATE = 5,      /* 当前状态不允许 */
    WIFI_PARAMS_RESULT_ERR_FLASH = 6       /* Flash 操作失败 */
} wifi_params_result_e;

/* WiFi 参数最近一次处理命令码 */
typedef enum
{
    WIFI_PARAMS_COMMAND_NONE = 0,          /* 无命令 */
    WIFI_PARAMS_COMMAND_PING = 1,          /* ping */
    WIFI_PARAMS_COMMAND_HELP = 2,          /* help */
    WIFI_PARAMS_COMMAND_GET = 3,           /* get */
    WIFI_PARAMS_COMMAND_SET = 4,           /* set */
    WIFI_PARAMS_COMMAND_SAVE = 5,          /* save */
    WIFI_PARAMS_COMMAND_LOAD = 6,          /* load */
    WIFI_PARAMS_COMMAND_LIST = 7,          /* list */
    WIFI_PARAMS_COMMAND_START = 8,         /* start */
    WIFI_PARAMS_COMMAND_STOP = 9           /* stop */
} wifi_params_command_e;

/* WiFi 参数诊断结构体：用于上位机查看最近一次命令处理结果 */
typedef struct
{
    uint8_t last_command_code;  /* 最近一次命令码 */
    uint8_t last_result_code;   /* 最近一次结果码 */
    uint16_t last_param_index;  /* 最近一次参数索引，0 表示无具体参数 */
    float last_value;           /* 最近一次相关参数值 */
} wifi_params_diag_t;

/*
 * 函数名: wifi_params_Init
 * 功能: 初始化 WiFi 参数模块内部状态
 * 输入参数: 无
 * 返回值: 无
 */
void wifi_params_Init(void);

/*
 * 函数名: wifi_params_ProcessLine
 * 功能: 处理一条完整文本命令
 * 输入参数:
 *   line - 完整文本命令，函数内部允许原地切分
 * 返回值: 无
 */
void wifi_params_ProcessLine(char *line);

/*
 * 函数名: wifi_params_GetDiag
 * 功能: 获取最近一次参数命令处理诊断信息
 * 输入参数:
 *   diag - 输出诊断结构体指针
 * 返回值: 无
 */
void wifi_params_GetDiag(wifi_params_diag_t *diag);

#endif /* WIFI_PARAMS_H */
