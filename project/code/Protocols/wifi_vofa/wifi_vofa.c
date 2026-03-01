#include "wifi_vofa.h"
#include <stdarg.h>

/*
 * wifi_vofa.c
 * - 基于 UDP 的 VOFA+ JustFloat 发送实现
 * - 目标IP由 UDP_REMOTE_IP 配置（当前为 192.168.110.218）
 */

#define WIFI_VOFA_JUSTFLOAT_TAIL_0    (0x00U)
#define WIFI_VOFA_JUSTFLOAT_TAIL_1    (0x00U)
#define WIFI_VOFA_JUSTFLOAT_TAIL_2    (0x80U)
#define WIFI_VOFA_JUSTFLOAT_TAIL_3    (0x7FU)
#define WIFI_VOFA_TIMER_INDEX         (TC_TIME2_CH1)

static uint8 s_wifi_vofa_ready = 0U;
static uint8 s_wifi_vofa_timer_inited = 0U;

typedef struct
{
    uint32 last_us;
    uint32 min_us;
    uint32 max_us;
    uint64 sum_us;
    uint32 ok_count;
    uint32 fail_count;
} wifi_vofa_tx_profile_t;

static wifi_vofa_tx_profile_t s_wifi_vofa_tx_profile = {0};

/* 重置发送耗时统计 */
static void wifi_vofa_tx_profile_reset(void)
{
    memset(&s_wifi_vofa_tx_profile, 0, sizeof(s_wifi_vofa_tx_profile));
    s_wifi_vofa_tx_profile.min_us = 0xFFFFFFFFU;
}

static void wifi_vofa_tx_profile_update(uint32 cost_us, uint8 ok)
{
    s_wifi_vofa_tx_profile.last_us = cost_us;
    if (cost_us < s_wifi_vofa_tx_profile.min_us)
    {
        s_wifi_vofa_tx_profile.min_us = cost_us;
    }
    if (cost_us > s_wifi_vofa_tx_profile.max_us)
    {
        s_wifi_vofa_tx_profile.max_us = cost_us;
    }

    if (0U == ok)
    {
        s_wifi_vofa_tx_profile.fail_count++;
        return;
    }

    s_wifi_vofa_tx_profile.ok_count++;
    s_wifi_vofa_tx_profile.sum_us += (uint64)cost_us;
}

void wifi_vofa_Init(void)
{
    uint8 ret;

    /* 初始化微秒计时器，用于统计发送耗时 */
    s_wifi_vofa_ready = 0U;
    if (0U == s_wifi_vofa_timer_inited)
    {
        timer_init(WIFI_VOFA_TIMER_INDEX, TIMER_US);
        timer_start(WIFI_VOFA_TIMER_INDEX);
        s_wifi_vofa_timer_inited = 1U;
    }
    timer_clear(WIFI_VOFA_TIMER_INDEX);
    wifi_vofa_tx_profile_reset();

    /* 阻塞式初始化：先连WiFi，再建立UDP socket */
    ret = wifi_spi_init((char *)WIFI_SSID_TEST, (char *)WIFI_PASSWORD_TEST);
    if (0U == ret)
    {
        /*
         * 遥测链路使用UDP，优先低时延与主循环实时性
         * 远端IP：192.168.110.218（见 UDP_REMOTE_IP）
         */
        ret = wifi_spi_socket_connect("UDP", (char *)UDP_REMOTE_IP, (char *)UDP_REMOTE_PORT, (char *)UDP_LOCAL_PORT);
    }

    if (0U == ret)
    {
        s_wifi_vofa_ready = 1U;
    }
    else
    {
        /* 初始化失败蜂鸣提示 */
        Beep_Stop();
        Beep_Play(50U, 0.5f, 10U);
    }
}

/* 连接状态：1=可发送，0=未就绪 */
uint8 wifi_vofa_IsReady(void)
{
    return s_wifi_vofa_ready;
}

/* 对外重置统计 */
void wifi_vofa_ResetTxStats(void)
{
    wifi_vofa_tx_profile_reset();
}

/* 对外读取统计，avg_us仅统计成功发送 */
void wifi_vofa_GetTxStats(wifi_vofa_tx_stats_t *stats)
{
    uint64 avg_us;

    if (NULL == stats)
    {
        return;
    }

    stats->last_us = s_wifi_vofa_tx_profile.last_us;
    stats->min_us = (s_wifi_vofa_tx_profile.min_us == 0xFFFFFFFFU) ? 0U : s_wifi_vofa_tx_profile.min_us;
    stats->max_us = s_wifi_vofa_tx_profile.max_us;
    stats->ok_count = s_wifi_vofa_tx_profile.ok_count;
    stats->fail_count = s_wifi_vofa_tx_profile.fail_count;

    avg_us = (s_wifi_vofa_tx_profile.ok_count > 0U)
                 ? (s_wifi_vofa_tx_profile.sum_us / (uint64)s_wifi_vofa_tx_profile.ok_count)
                 : 0U;
    stats->avg_us = (uint32)avg_us;
}

/* JustFloat发送：data_num通道，每通道4字节float，末尾4字节协议尾 */
uint8 wifi_vofa_JustFloat(uint8 data_num, ...)
{
    uint8 i;
    uint8 ret;
    uint16 payload_len;
    uint16 frame_len;
    uint32 start_us;
    uint32 cost_us;
    uint32 sent_len;
    uint8 frame[WIFI_VOFA_MAX_FLOAT_NUM * 4U + 4U];
    va_list ap;

    if ((0U == data_num) || (data_num > WIFI_VOFA_MAX_FLOAT_NUM))
    {
        /* 参数非法 */
        s_wifi_vofa_tx_profile.fail_count++;
        return 1U;
    }

    if (0U == s_wifi_vofa_ready)
    {
        /* 尚未连通UDP链路 */
        s_wifi_vofa_tx_profile.fail_count++;
        return 1U;
    }

    /* 可变参数读取：float会按double传递，需按double取出 */
    va_start(ap, data_num);
    for (i = 0U; i < data_num; i++)
    {
        float value_f = (float)va_arg(ap, double);
        memcpy(&frame[i * 4U], &value_f, sizeof(float));
    }
    va_end(ap);

    payload_len = (uint16)data_num * 4U;
    frame[payload_len + 0U] = WIFI_VOFA_JUSTFLOAT_TAIL_0;
    frame[payload_len + 1U] = WIFI_VOFA_JUSTFLOAT_TAIL_1;
    frame[payload_len + 2U] = WIFI_VOFA_JUSTFLOAT_TAIL_2;
    frame[payload_len + 3U] = WIFI_VOFA_JUSTFLOAT_TAIL_3;

    frame_len = payload_len + 4U;

    /* 记录发送耗时并更新统计 */
    start_us = timer_get(WIFI_VOFA_TIMER_INDEX);
    sent_len = wifi_spi_send_buffer(frame, frame_len);
    cost_us = timer_get(WIFI_VOFA_TIMER_INDEX) - start_us;

    ret = (sent_len == 0U) ? 0U : 1U;
    wifi_vofa_tx_profile_update(cost_us, (ret == 0U) ? 1U : 0U);
    return ret;
}
