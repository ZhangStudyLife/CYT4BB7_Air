/*****************************************************************************
 * 文件: wifi_cmd.c
 * 模块: WiFi 命令基础层
 * 职责: 负责 wifi_spi 初始化、UDP socket 建立、文本命令收发与命令路由
 *****************************************************************************/

#include "wifi_cmd.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "zf_device_wifi_spi.h"
#include "../../HW_Drivers/Beep/Beep.h"
#include "../wifi_cal_imu/wifi_cal_imu.h"
#include "../wifi_params/wifi_params.h"

/* WiFi 文本接收状态 */
static uint8_t s_wifi_cmd_ready = 0U;            /* WiFi 链路就绪标志 */
static char s_wifi_cmd_line[WIFI_CMD_LINE_MAX] = {0}; /* 当前接收中的文本行 */
static uint16_t s_wifi_cmd_line_len = 0U;        /* 当前文本行长度 */
static uint8_t s_wifi_cmd_line_overflow = 0U;    /* 当前文本行是否溢出 */
static uint8_t s_wifi_cmd_line_invalid = 0U;     /* 当前文本行是否包含非法字符 */
static uint8_t s_wifi_cmd_line_expect_lf = 0U;   /* 当前是否已收到 CR，等待 LF */

/*
 * 函数名: wifi_cmd_is_space_char
 * 功能: 判断字符是否为空白分隔符
 * 输入参数:
 *   ch - 待判断字符
 * 返回值:
 *   1 - 空白字符
 *   0 - 非空白字符
 */
static uint8_t wifi_cmd_is_space_char(char ch)
{
    return ((' ' == ch) || ('\t' == ch)) ? 1U : 0U;
}

/*
 * 函数名: wifi_cmd_ascii_tolower
 * 功能: 将单个 ASCII 字符转换为小写
 * 输入参数:
 *   ch - 待转换字符
 * 返回值:
 *   转换后的字符
 */
static char wifi_cmd_ascii_tolower(char ch)
{
    if ((ch >= 'A') && (ch <= 'Z'))
    {
        return (char)(ch - 'A' + 'a');
    }

    return ch;
}

/*
 * 函数名: wifi_cmd_reset_line_state
 * 功能: 重置文本命令接收状态机
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cmd_reset_line_state(void)
{
    s_wifi_cmd_line_len = 0U;
    s_wifi_cmd_line_overflow = 0U;
    s_wifi_cmd_line_invalid = 0U;
    s_wifi_cmd_line_expect_lf = 0U;
}

/*
 * 函数名: wifi_cmd_dispatch_line
 * 功能: 根据首 token 将完整文本命令路由到具体模块
 * 输入参数:
 *   line - 完整文本命令，函数内部可原地修改
 * 返回值: 无
 */
static void wifi_cmd_dispatch_line(char *line)
{
    char inspect_line[WIFI_CMD_LINE_MAX];
    char *cursor;
    char *token_cmd;
    char *trimmed_line;

    if (NULL == line)
    {
        return;
    }

    trimmed_line = wifi_cmd_trim_line(line);
    if ((NULL == trimmed_line) || ('\0' == trimmed_line[0]))
    {
        return;
    }

    strncpy(inspect_line, trimmed_line, sizeof(inspect_line) - 1U);
    inspect_line[sizeof(inspect_line) - 1U] = '\0';

    cursor = inspect_line;
    token_cmd = wifi_cmd_next_token(&cursor);
    if (NULL == token_cmd)
    {
        return;
    }

    wifi_cmd_ascii_strtolower(token_cmd);

    if (0 == strcmp(token_cmd, "imu"))
    {
        wifi_cal_imu_ProcessLine(trimmed_line);
        return;
    }

    wifi_params_ProcessLine(trimmed_line);
}

/*
 * 函数名: wifi_cmd_feed_byte
 * 功能: 向文本命令状态机喂入单字节
 * 输入参数:
 *   ch - 新收到的字节
 * 返回值: 无
 */
static void wifi_cmd_feed_byte(char ch)
{
    if (0U != s_wifi_cmd_line_expect_lf)
    {
        if ('\n' == ch)
        {
            if ((0U != s_wifi_cmd_line_overflow) || (0U != s_wifi_cmd_line_invalid))
            {
                (void)wifi_cmd_SendLine("ERR format");
            }
            else if (s_wifi_cmd_line_len > 0U)
            {
                s_wifi_cmd_line[s_wifi_cmd_line_len] = '\0';
                wifi_cmd_dispatch_line(s_wifi_cmd_line);
            }

            wifi_cmd_reset_line_state();
            return;
        }

        (void)wifi_cmd_SendLine("ERR format");
        wifi_cmd_reset_line_state();
        return;
    }

    if ('\n' == ch)
    {
        (void)wifi_cmd_SendLine("ERR format");
        wifi_cmd_reset_line_state();
        return;
    }

    if ('\r' == ch)
    {
        s_wifi_cmd_line_expect_lf = 1U;
        return;
    }

    if (((unsigned char)ch < 32U) || ((unsigned char)ch > 126U))
    {
        if ('\t' != ch)
        {
            s_wifi_cmd_line_invalid = 1U;
            return;
        }
    }

    if (0U != s_wifi_cmd_line_overflow)
    {
        return;
    }

    if (s_wifi_cmd_line_len >= (WIFI_CMD_LINE_MAX - 1U))
    {
        s_wifi_cmd_line_overflow = 1U;
        return;
    }

    s_wifi_cmd_line[s_wifi_cmd_line_len++] = ch;
}

void wifi_cmd_Init(void)
{
    uint8_t ret;

    s_wifi_cmd_ready = 0U;
    memset(s_wifi_cmd_line, 0, sizeof(s_wifi_cmd_line));
    wifi_cmd_reset_line_state();

    ret = wifi_spi_init((char *)WIFI_SSID_TEST, (char *)WIFI_PASSWORD_TEST);
    if (0U == ret)
    {
        ret = wifi_spi_socket_connect("UDP", (char *)UDP_REMOTE_IP, (char *)UDP_REMOTE_PORT, (char *)UDP_LOCAL_PORT);
    }

    if (0U == ret)
    {
        s_wifi_cmd_ready = 1U;
    }
    else
    {
        Beep_Stop();
        Beep_Play(50U, 0.5f, 5U);
    }
}

void wifi_cmd_Poll(void)
{
    uint8_t rx_buffer[WIFI_CMD_RX_BUFFER_SIZE];
    uint32_t read_len;
    uint32_t i;

    read_len = wifi_spi_read_buffer(rx_buffer, (uint32_t)sizeof(rx_buffer));
    for (i = 0U; i < read_len; i++)
    {
        wifi_cmd_feed_byte((char)rx_buffer[i]);
    }
}

uint8_t wifi_cmd_IsReady(void)
{
    return s_wifi_cmd_ready;
}

/*
 * 函数功能：向 WiFi SPI 发送一段原始二进制数据，但不立即触发 UDP 发包。
 * 输入参数：buffer-待发送数据首地址；len-待发送长度，单位字节。
 * 返回值：1-写入成功；0-写入失败。
 */
uint8_t wifi_cmd_SendBufferNoFlush(const uint8_t *buffer, uint32_t len)
{
    uint32_t remain_len;

    if ((NULL == buffer) || (0U == len) || (0U == s_wifi_cmd_ready))
    {
        return 0U;
    }

    remain_len = wifi_spi_send_buffer(buffer, len);
    return (0U == remain_len) ? 1U : 0U;
}

/*
 * 函数功能：立即触发一次 UDP 发包，将当前发送缓冲区数据整体送出。
 * 输入参数：无。
 * 返回值：1-触发成功；0-触发失败。
 */
uint8_t wifi_cmd_FlushNow(void)
{
    if (0U == s_wifi_cmd_ready)
    {
        return 0U;
    }

    return (0U == wifi_spi_udp_send_now()) ? 1U : 0U;
}

/*
 * 函数功能：发送一段原始二进制数据，并立即触发一次 UDP 发包。
 * 输入参数：buffer-待发送数据首地址；len-待发送长度，单位字节。
 * 返回值：1-发送成功；0-发送失败。
 */
uint8_t wifi_cmd_SendBuffer(const uint8_t *buffer, uint32_t len)
{
    if (0U == wifi_cmd_SendBufferNoFlush(buffer, len))
    {
        return 0U;
    }

    return wifi_cmd_FlushNow();
}

uint8_t wifi_cmd_SendLine(const char *format, ...)
{
    char line[WIFI_CMD_TX_LINE_MAX];
    int write_len;
    va_list ap;

    if (NULL == format)
    {
        return 0U;
    }

    va_start(ap, format);
    write_len = vsnprintf(line, (int)(sizeof(line) - 3U), format, ap);
    va_end(ap);

    if (write_len < 0)
    {
        return 0U;
    }

    if ((uint32_t)write_len > (sizeof(line) - 3U))
    {
        write_len = (int)(sizeof(line) - 3U);
    }

    line[write_len + 0] = '\r';
    line[write_len + 1] = '\n';
    line[write_len + 2] = '\0';

    return wifi_cmd_SendBuffer((const uint8_t *)line, (uint32_t)(write_len + 2));
}

int wifi_cmd_ascii_stricmp(const char *lhs, const char *rhs)
{
    char left_ch;
    char right_ch;

    if ((NULL == lhs) || (NULL == rhs))
    {
        return (lhs == rhs) ? 0 : 1;
    }

    while (('\0' != *lhs) && ('\0' != *rhs))
    {
        left_ch = wifi_cmd_ascii_tolower(*lhs);
        right_ch = wifi_cmd_ascii_tolower(*rhs);
        if (left_ch != right_ch)
        {
            return ((int)(unsigned char)left_ch - (int)(unsigned char)right_ch);
        }

        lhs++;
        rhs++;
    }

    left_ch = wifi_cmd_ascii_tolower(*lhs);
    right_ch = wifi_cmd_ascii_tolower(*rhs);
    return ((int)(unsigned char)left_ch - (int)(unsigned char)right_ch);
}

void wifi_cmd_ascii_strtolower(char *text)
{
    if (NULL == text)
    {
        return;
    }

    while ('\0' != *text)
    {
        *text = wifi_cmd_ascii_tolower(*text);
        text++;
    }
}

char *wifi_cmd_trim_line(char *text)
{
    char *end;

    if (NULL == text)
    {
        return NULL;
    }

    while (wifi_cmd_is_space_char(*text))
    {
        text++;
    }

    if ('\0' == *text)
    {
        return text;
    }

    end = text + strlen(text) - 1;
    while ((end >= text) && wifi_cmd_is_space_char(*end))
    {
        *end = '\0';
        end--;
    }

    return text;
}

char *wifi_cmd_next_token(char **cursor)
{
    char *token;
    char *end;

    if ((NULL == cursor) || (NULL == *cursor))
    {
        return NULL;
    }

    while (wifi_cmd_is_space_char(**cursor))
    {
        (*cursor)++;
    }

    if ('\0' == **cursor)
    {
        *cursor = NULL;
        return NULL;
    }

    token = *cursor;
    end = token;
    while ((*end != '\0') && (0U == wifi_cmd_is_space_char(*end)))
    {
        end++;
    }

    if ('\0' == *end)
    {
        *cursor = NULL;
    }
    else
    {
        *end = '\0';
        *cursor = end + 1;
    }

    return token;
}

uint8_t wifi_cmd_is_help_flag(const char *text)
{
    if (NULL == text)
    {
        return 0U;
    }

    if (0 == wifi_cmd_ascii_stricmp(text, "help"))
    {
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(text, "--help"))
    {
        return 1U;
    }

    return (0 == wifi_cmd_ascii_stricmp(text, "-h")) ? 1U : 0U;
}
