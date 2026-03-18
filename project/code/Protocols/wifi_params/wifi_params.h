/*****************************************************************************
 * 文件: wifi_params.h
 * 模块: WiFi 参数调节
 * 职责: 通过现有 WiFi SPI-UDP 链路接收人工可读文本命令，并在待机状态安全调节飞控参数
 *****************************************************************************/

#ifndef WIFI_PARAMS_H
#define WIFI_PARAMS_H

#include "zf_common_headfile.h"

/* WiFi 参数文本单行最大长度，单位字节，包含末尾字符串结束符 */
#define WIFI_PARAMS_LINE_MAX (128U)

/* WiFi 参数最近一次处理结果码 */
typedef enum
{
    WIFI_PARAMS_RESULT_OK = 0,
    WIFI_PARAMS_RESULT_ERR_FORMAT = 1,
    WIFI_PARAMS_RESULT_ERR_UNKNOWN_CMD = 2,
    WIFI_PARAMS_RESULT_ERR_UNKNOWN_PARAM = 3,
    WIFI_PARAMS_RESULT_ERR_RANGE = 4,
    WIFI_PARAMS_RESULT_ERR_STATE = 5,
    WIFI_PARAMS_RESULT_ERR_FLASH = 6
} wifi_params_result_e;

/* WiFi 参数最近一次处理命令码 */
typedef enum
{
    WIFI_PARAMS_COMMAND_NONE = 0,
    WIFI_PARAMS_COMMAND_PING = 1,
    WIFI_PARAMS_COMMAND_HELP = 2,
    WIFI_PARAMS_COMMAND_GET = 3,
    WIFI_PARAMS_COMMAND_SET = 4,
    WIFI_PARAMS_COMMAND_SAVE = 5,
    WIFI_PARAMS_COMMAND_LOAD = 6,
    WIFI_PARAMS_COMMAND_LIST = 7,
    WIFI_PARAMS_COMMAND_START = 8,
    WIFI_PARAMS_COMMAND_STOP = 9
} wifi_params_command_e;

/* WiFi 参数诊断结构体：用于通过 VOFA 回传最近一次命令处理状态 */
typedef struct
{
    uint8_t last_command_code;  /* 最近一次命令码 */
    uint8_t last_result_code;   /* 最近一次结果码 */
    uint16_t last_param_index;  /* 最近一次参数索引，0 表示无匹配参数 */
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
 * 函数名: wifi_params_Poll
 * 功能: 轮询读取并解析 WiFi 参数命令，必要时在待机态刷新 PID 参数
 * 输入参数: 无
 * 返回值: 无
 */
void wifi_params_Poll(void);

/*
 * 函数名: wifi_params_GetDiag
 * 功能: 读取最近一次命令处理诊断状态
 * 输入参数:
 *   diag - 输出诊断结构体指针
 * 返回值: 无
 */
void wifi_params_GetDiag(wifi_params_diag_t *diag);

#endif /* WIFI_PARAMS_H */
