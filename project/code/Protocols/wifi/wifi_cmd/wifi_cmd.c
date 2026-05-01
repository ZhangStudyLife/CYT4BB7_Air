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
#include "FlightController/fc_start_crsf.h"
#include "../../HW_Drivers/Beep/Beep.h"
#include "../wifi_cal_imu/wifi_cal_imu.h"
#include "../wifi_params/wifi_params.h"

#define WIFI_CMD_TEXT_SEND_POLL_LIMIT   (20000U)

static uint8 s_wifi_cmd_text_tx_active = 0U;

static uint8 s_wifi_cmd_use_udp_flush = 1U;    /* 当前链路是否使用UDP立即发送命令：1=UDP，0=TCP */
static uint8 s_wifi_cmd_flush_pending = 0U;    /* 是否存在待触发的UDP立即发送请求：1-待触发，0-无请求 */

/* WiFi 文本接收状态 */
static uint8 s_wifi_cmd_ready = 0U;            /* WiFi 链路就绪标志 */
static char s_wifi_cmd_line[WIFI_CMD_LINE_MAX] = {0}; /* 当前接收中的文本行 */
static uint16 s_wifi_cmd_line_len = 0U;        /* 当前文本行长度 */
static uint8 s_wifi_cmd_line_overflow = 0U;    /* 当前文本行是否溢出 */
static uint8 s_wifi_cmd_line_invalid = 0U;     /* 当前文本行是否包含非法字符 */
static uint8 s_wifi_cmd_line_expect_lf = 0U;   /* 当前是否已收到 CR，等待 LF */

/*
 * 函数名: wifi_cmd_is_space_char
 * 功能: 判断字符是否为空白分隔符
 * 输入参数:
 *   ch - 待判断字符
 * 返回值:
 *   1 - 空白字符
 *   0 - 非空白字符
 */
static uint8 wifi_cmd_is_space_char(char ch)
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

static uint8 wifi_cmd_wait_tx_idle(void)
{
    uint32 guard = 0U;

    while ((0U != wifi_spi_is_busy()) && (guard < WIFI_CMD_TEXT_SEND_POLL_LIMIT))
    {
        wifi_spi_send_poll();
        guard++;
    }

    return (0U == wifi_spi_is_busy()) ? 1U : 0U;
}

static uint8 wifi_cmd_finish_pending_flush(void)
{
    if (0U == wifi_cmd_wait_tx_idle())
    {
        return 0U;
    }

    if (0U != s_wifi_cmd_flush_pending)
    {
        if (0U == wifi_cmd_FlushNow())
        {
            return 0U;
        }

        s_wifi_cmd_flush_pending = 0U;
        if (0U == wifi_cmd_wait_tx_idle())
        {
            return 0U;
        }
    }

    return 1U;
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
    uint8 ret;

    s_wifi_cmd_ready = 0U;
    s_wifi_cmd_use_udp_flush = 1U;
    s_wifi_cmd_flush_pending = 0U;
    s_wifi_cmd_text_tx_active = 0U;
    memset(s_wifi_cmd_line, 0, sizeof(s_wifi_cmd_line));
    wifi_cmd_reset_line_state();

    ret = wifi_spi_init((char *)WIFI_SSID_TEST, (char *)WIFI_PASSWORD_TEST);
    if (0U == ret)
    {
#if (0U == WIFI_IMAGE_ENABLE)
        ret = wifi_spi_socket_connect("UDP", (char *)UDP_REMOTE_IP, (char *)UDP_REMOTE_PORT, (char *)UDP_LOCAL_PORT);
#else
        ret = wifi_spi_socket_connect((char *)WIFI_IMAGE_TCP_CLIENT_TRANSPORT,
                                      (char *)UDP_REMOTE_IP,
                                      (char *)UDP_REMOTE_PORT,
                                      (char *)UDP_LOCAL_PORT);
#endif
    }

    if (0U == ret)
    {
#if (0U == WIFI_IMAGE_ENABLE)
        s_wifi_cmd_use_udp_flush = 1U;
#else
        s_wifi_cmd_use_udp_flush = 0U;
#endif
        s_wifi_cmd_ready = 1U;
    }
    else
    {
        Beep_Stop();
        Beep_Play(50U, 0.5f, 5U);
    }
}

// 04110316 zyz实际测试花费88us
void wifi_cmd_Poll(void)
{
    uint8 rx_buffer[WIFI_CMD_RX_BUFFER_SIZE];
    uint32 read_len;
    uint32 i;

    if (0U == s_wifi_cmd_ready)
    {
        return;
    }

    /* 推进非阻塞发送状态机，确保发送在主循环中持续推进 */
    wifi_spi_send_poll();

    if (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_FLYING)
    {
        if ((0U != s_wifi_cmd_flush_pending) && (0U == wifi_spi_is_busy()))
        {
            s_wifi_cmd_flush_pending = 0U;
        }
        return;
    }

    /* 发送完成后再触发UDP立即发送，避免与发送事务重叠 */
    if ((0U != s_wifi_cmd_flush_pending) && (0U == wifi_spi_is_busy()))
    {
        if (0U != wifi_cmd_FlushNow())
        {
            s_wifi_cmd_flush_pending = 0U;
        }
    }

    /* 仅在非飞行状态下轮询 */
    read_len = wifi_spi_read_buffer(rx_buffer, (uint32)sizeof(rx_buffer));
    for (i = 0U; i < read_len; i++)
    {
        wifi_cmd_feed_byte((char)rx_buffer[i]);
    }
}

uint8 wifi_cmd_IsReady(void)
{
    return s_wifi_cmd_ready;
}

uint8 wifi_cmd_IsTextBusy(void)
{
    return s_wifi_cmd_text_tx_active;
}

/*
 * 函数功能：向 WiFi SPI 发送一段原始二进制数据，但不立即触发 UDP 发包。
 * 输入参数：buffer-待发送数据首地址；len-待发送长度，单位字节。
 * 返回值：1-写入成功；0-写入失败。
 */
uint8 wifi_cmd_SendBufferNoFlush(const uint8 *buffer, uint32 len)
{
    uint32 remain_len;

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
uint8 wifi_cmd_FlushNow(void)
{
    if (0U == s_wifi_cmd_ready)
    {
        return 0U;
    }

    if (0U == s_wifi_cmd_use_udp_flush)
    {
        return 1U;
    }

    return (0U == wifi_spi_udp_send_now()) ? 1U : 0U;
}

/*
 * 函数功能：提交一段原始二进制数据到非阻塞发送链路，并在后续轮询中触发 UDP 发包。
 * 输入参数：buffer-待发送数据首地址；len-待发送长度，单位字节。
 * 返回值：1-提交成功；0-提交失败。
 */
uint8 wifi_cmd_SendBuffer(const uint8 *buffer, uint32 len)
{
    if (0U == wifi_cmd_SendBufferNoFlush(buffer, len))
    {
        return 0U;
    }

    if (0U == s_wifi_cmd_use_udp_flush)
    {
        return 1U;
    }

    s_wifi_cmd_flush_pending = 1U;
    return 1U;
}

uint8  wifi_cmd_SendLine(const char *format, ...)
{
    char line[WIFI_CMD_TX_LINE_MAX];
    uint8 ret = 0U;
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

    if ((uint32)write_len > (sizeof(line) - 3U))
    {
        write_len = (int)(sizeof(line) - 3U);
    }

    line[write_len + 0] = '\r';
    line[write_len + 1] = '\n';
    line[write_len + 2] = '\0';

    if (0U == s_wifi_cmd_ready)
    {
        return 0U;
    }

    s_wifi_cmd_text_tx_active = 1U;

    if (0U == wifi_cmd_finish_pending_flush())
    {
        goto wifi_cmd_send_line_exit;
    }

    if (0U == wifi_cmd_SendBufferNoFlush((const uint8 *)line, (uint32)(write_len + 2)))
    {
        goto wifi_cmd_send_line_exit;
    }

    if (0U == wifi_cmd_wait_tx_idle())
    {
        goto wifi_cmd_send_line_exit;
    }

    if (0U != s_wifi_cmd_use_udp_flush)
    {
        s_wifi_cmd_flush_pending = 1U;
        if (0U == wifi_cmd_FlushNow())
        {
            goto wifi_cmd_send_line_exit;
        }

        s_wifi_cmd_flush_pending = 0U;
        if (0U == wifi_cmd_wait_tx_idle())
        {
            goto wifi_cmd_send_line_exit;
        }
    }

    ret = 1U;

wifi_cmd_send_line_exit:
    s_wifi_cmd_text_tx_active = 0U;
    return ret;
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

uint8 wifi_cmd_is_help_flag(const char *text)
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
