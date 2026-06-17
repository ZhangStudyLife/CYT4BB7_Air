/*****************************************************************************
 * 文件: wifi_cmd.h
 * 模块: WiFi 命令基础层
 * 职责: 负责 wifi_spi 初始化、UDP socket 建立、文本命令收发与基础文本解析工具
 *****************************************************************************/

#ifndef WIFI_CMD_H
#define WIFI_CMD_H

#include "zf_common_headfile.h"

#ifndef WIFI_SSID_TEST
#define WIFI_SSID_TEST      "HDUASC_saidao"      /* WiFi 路由器 SSID */
#endif

#ifndef WIFI_PASSWORD_TEST
#define WIFI_PASSWORD_TEST  "zyz520520"          /* WiFi 路由器密码 */
#endif

#ifndef WIFI_IMAGE_ENABLE
#define WIFI_IMAGE_ENABLE   (0U)                 /* WiFi image mode switch: 0=normal UDP command mode, 1=image TCP mode */
#endif

#ifndef WIFI_IMAGE_TCP_SERVER_TRANSPORT
#define WIFI_IMAGE_TCP_SERVER_TRANSPORT  "TCP_SERVER" /* Preferred TCP transport token in image mode */
#endif

#ifndef WIFI_IMAGE_TCP_CLIENT_TRANSPORT
#define WIFI_IMAGE_TCP_CLIENT_TRANSPORT  "TCP"   /* Fallback TCP transport token in image mode */
#endif

#ifndef UDP_REMOTE_IP
#define UDP_REMOTE_IP       "192.168.110.183"    /* 上位机 IP 地址 */
#endif

#ifndef UDP_REMOTE_PORT
#define UDP_REMOTE_PORT     "1347"               /* 上位机 UDP 端口 */
#endif

#ifndef UDP_LOCAL_PORT
#define UDP_LOCAL_PORT      "6666"               /* 机载 WiFi 模块本地端口 */
#endif

#define WIFI_CMD_RX_BUFFER_SIZE   (128U)         /* WiFi 底层接收缓冲区大小，单位字节 */
#define WIFI_CMD_LINE_MAX         (128U)         /* 单条文本命令最大长度，单位字节 */
#define WIFI_CMD_TX_LINE_MAX      (256U)         /* 单条文本回包最大长度，单位字节 */

/*
 * 函数名: wifi_cmd_Init
 * 功能: 初始化 WiFi 链路并建立 UDP socket
 * 输入参数: 无
 * 返回值: 无
 */
void wifi_cmd_Init(void);

/*
 * 函数名: wifi_cmd_Poll
 * 功能: 轮询读取 WiFi 文本命令并按命令族路由到上层模块
 * 输入参数: 无
 * 返回值: 无
 */
void wifi_cmd_Poll(void);

/*
 * 函数名: wifi_cmd_IsReady
 * 功能: 查询当前 WiFi-UDP 链路是否已建立
 * 输入参数: 无
 * 返回值:
 *   1 - 链路已就绪
 *   0 - 链路未就绪
 */
uint8_t wifi_cmd_IsReady(void);

/* Query whether text replies are queued or in flight. */
uint8_t wifi_cmd_IsTextBusy(void);
/* Query whether the raw WiFi SPI transmit path is busy. */
uint8_t wifi_cmd_IsRawBusy(void);

/*
 * 函数名: wifi_cmd_SendBuffer
 * 功能: 提交一段原始二进制数据到非阻塞发送链路，并在后续轮询中触发 UDP 发送
 * 输入参数:
 *   buffer - 待发送数据首地址
 *   len    - 待发送长度，单位字节
 * 返回值:
 *   1 - 提交成功
 *   0 - 提交失败
 */
uint8_t wifi_cmd_SendBuffer(const uint8_t *buffer, uint32_t len);
/*
 * 函数名: wifi_cmd_SendBufferNoFlush
 * 功能: 向 WiFi SPI 发送一段原始二进制数据，但不立即触发 UDP 发包
 * 输入参数:
 *   buffer - 待发送数据首地址
 *   len    - 待发送长度，单位字节
 * 返回值:
 *   1 - 写入成功
 *   0 - 写入失败
 */
uint8_t wifi_cmd_SendBufferNoFlush(const uint8_t *buffer, uint32_t len);
/*
 * 函数名: wifi_cmd_FlushNow
 * 功能: 立即触发一次 UDP 发包，将之前写入缓冲区的数据整体发出
 * 输入参数:
 *   无
 * 返回值:
 *   1 - 触发成功
 *   0 - 触发失败
 */
uint8_t wifi_cmd_FlushNow(void);

/*
 * 函数名: wifi_cmd_SendLine
 * 功能: 发送一行文本并自动补齐 CRLF
 * 输入参数:
 *   format - printf 风格格式串
 *   ...    - 可变参数
 * 返回值:
 *   1 - 发送成功
 *   0 - 发送失败
 */
uint8_t wifi_cmd_SendLine(const char *format, ...);

/*
 * 函数名: wifi_cmd_ascii_stricmp
 * 功能: 对两个 ASCII 字符串做大小写无关比较
 * 输入参数:
 *   lhs - 左字符串
 *   rhs - 右字符串
 * 返回值:
 *   0  - 相等
 *   非0 - 不相等
 */
int wifi_cmd_ascii_stricmp(const char *lhs, const char *rhs);

/*
 * 函数名: wifi_cmd_ascii_strtolower
 * 功能: 将 ASCII 字符串原地转成小写
 * 输入参数:
 *   text - 待转换字符串
 * 返回值: 无
 */
void wifi_cmd_ascii_strtolower(char *text);

/*
 * 函数名: wifi_cmd_trim_line
 * 功能: 去除文本行首尾空白字符
 * 输入参数:
 *   text - 待裁剪文本
 * 返回值:
 *   裁剪后的首地址
 */
char *wifi_cmd_trim_line(char *text);

/*
 * 函数名: wifi_cmd_next_token
 * 功能: 按空白分隔提取下一个 token，函数会原地写入字符串结束符
 * 输入参数:
 *   cursor - 当前解析游标地址
 * 返回值:
 *   token 首地址；无可用 token 时返回 NULL
 */
char *wifi_cmd_next_token(char **cursor);

/*
 * 函数名: wifi_cmd_is_help_flag
 * 功能: 判断 token 是否为帮助标志
 * 输入参数:
 *   text - 待判断 token
 * 返回值:
 *   1 - 是帮助标志
 *   0 - 不是帮助标志
 */
uint8_t wifi_cmd_is_help_flag(const char *text);

#endif /* WIFI_CMD_H */
