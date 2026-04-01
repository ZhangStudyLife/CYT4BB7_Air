/*****************************************************************************
 * 文件: wifi_justfloat.c
 * 模块: WiFi JustFloat 遥测
 * 职责: 负责 VOFA JustFloat 二进制帧打包、待机态控流与发送耗时统计
 *****************************************************************************/

#include "wifi_justfloat.h"

#include <stdarg.h>
#include <string.h>

#include "../wifi_cmd/wifi_cmd.h"

#define WIFI_JUSTFLOAT_TAIL_0      (0x00U)       /* JustFloat 尾部字节 0 */
#define WIFI_JUSTFLOAT_TAIL_1      (0x00U)       /* JustFloat 尾部字节 1 */
#define WIFI_JUSTFLOAT_TAIL_2      (0x80U)       /* JustFloat 尾部字节 2 */
#define WIFI_JUSTFLOAT_TAIL_3      (0x7FU)       /* JustFloat 尾部字节 3 */
#define WIFI_JUSTFLOAT_TIMER_INDEX (TC_TIME2_CH1)/* JustFloat 发送耗时统计用定时器 */

/* JustFloat 发送剖面统计结构体 */
typedef struct
{
    uint32_t last_us;     /* 最近一次发送耗时 */
    uint32_t min_us;      /* 最小发送耗时 */
    uint32_t max_us;      /* 最大发送耗时 */
    uint64_t sum_us;      /* 成功发送总耗时 */
    uint32_t ok_count;    /* 成功发送次数 */
    uint32_t fail_count;  /* 失败次数 */
    uint32_t skip_count;  /* 被待机态 stop 抑制次数 */
} wifi_justfloat_tx_profile_t;

static uint8_t s_wifi_justfloat_timer_inited = 0U;     /* 发送耗时统计定时器是否已初始化 */
static uint8_t s_wifi_justfloat_standby_context = 0U;  /* 当前是否处于待机态 */
static uint8_t s_wifi_justfloat_standby_user_enable = 1U; /* 用户是否允许待机态发送 */
static wifi_justfloat_tx_profile_t s_wifi_justfloat_profile = {0}; /* 发送统计数据 */

/*
 * 函数名: wifi_justfloat_profile_reset
 * 功能: 重置发送耗时统计
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_justfloat_profile_reset(void)
{
    memset(&s_wifi_justfloat_profile, 0, sizeof(s_wifi_justfloat_profile));
    s_wifi_justfloat_profile.min_us = 0xFFFFFFFFU;
}

/*
 * 函数名: wifi_justfloat_should_send
 * 功能: 判断当前是否允许发送遥测
 * 输入参数: 无
 * 返回值:
 *   1 - 允许发送
 *   0 - 禁止发送
 */
static uint8_t wifi_justfloat_should_send(void)
{
    if ((0U != s_wifi_justfloat_standby_context) && (0U == s_wifi_justfloat_standby_user_enable))
    {
        return 0U;
    }

    return 1U;
}

/*
 * 函数名: wifi_justfloat_profile_update
 * 功能: 更新发送耗时统计
 * 输入参数:
 *   cost_us - 本次发送耗时，单位 us
 *   ok      - 1 表示发送成功，0 表示发送失败
 * 返回值: 无
 */
static void wifi_justfloat_profile_update(uint32_t cost_us, uint8_t ok)
{
    s_wifi_justfloat_profile.last_us = cost_us;
    if (cost_us < s_wifi_justfloat_profile.min_us)
    {
        s_wifi_justfloat_profile.min_us = cost_us;
    }
    if (cost_us > s_wifi_justfloat_profile.max_us)
    {
        s_wifi_justfloat_profile.max_us = cost_us;
    }

    if (0U == ok)
    {
        s_wifi_justfloat_profile.fail_count++;
        return;
    }

    s_wifi_justfloat_profile.ok_count++;
    s_wifi_justfloat_profile.sum_us += (uint64_t)cost_us;
}

void wifi_justfloat_Init(void)
{
    if (0U == s_wifi_justfloat_timer_inited)
    {
        timer_init(WIFI_JUSTFLOAT_TIMER_INDEX, TIMER_US);
        timer_start(WIFI_JUSTFLOAT_TIMER_INDEX);
        s_wifi_justfloat_timer_inited = 1U;
    }

    timer_clear(WIFI_JUSTFLOAT_TIMER_INDEX);
    s_wifi_justfloat_standby_context = 0U;
    s_wifi_justfloat_standby_user_enable = 1U;
    wifi_justfloat_profile_reset();
}

uint8_t wifi_justfloat_IsReady(void)
{
    return wifi_cmd_IsReady();
}

void wifi_justfloat_SetStandbyContext(uint8_t is_standby)
{
    s_wifi_justfloat_standby_context = (0U == is_standby) ? 0U : 1U;
}

void wifi_justfloat_SetStandbyUserEnable(uint8_t enable)
{
    s_wifi_justfloat_standby_user_enable = (0U == enable) ? 0U : 1U;
}

uint8_t wifi_justfloat_GetStandbyUserEnable(void)
{
    return s_wifi_justfloat_standby_user_enable;
}

void wifi_justfloat_ResetTxStats(void)
{
    wifi_justfloat_profile_reset();
}

void wifi_justfloat_GetTxStats(wifi_justfloat_tx_stats_t *stats)
{
    uint64_t avg_us;

    if (NULL == stats)
    {
        return;
    }

    stats->last_us = s_wifi_justfloat_profile.last_us;
    stats->min_us = (s_wifi_justfloat_profile.min_us == 0xFFFFFFFFU) ? 0U : s_wifi_justfloat_profile.min_us;
    stats->max_us = s_wifi_justfloat_profile.max_us;
    stats->ok_count = s_wifi_justfloat_profile.ok_count;
    stats->fail_count = s_wifi_justfloat_profile.fail_count;
    stats->skip_count = s_wifi_justfloat_profile.skip_count;

    avg_us = (s_wifi_justfloat_profile.ok_count > 0U)
                 ? (s_wifi_justfloat_profile.sum_us / (uint64_t)s_wifi_justfloat_profile.ok_count)
                 : 0U;
    stats->avg_us = (uint32_t)avg_us;
}

uint8_t wifi_justfloat_Impl(uint8_t declared_num, uint8_t actual_num, ...)
{
    uint8_t i;
    uint8_t ret;
    uint16_t payload_len;
    uint16_t frame_len;
    uint32_t start_us;
    uint32_t cost_us;
    uint8_t frame[WIFI_JUSTFLOAT_MAX_FLOAT_NUM * 4U + 4U];
    va_list ap;

#if (1U == WIFI_IMAGE_ENABLE)
    (void)declared_num;
    (void)actual_num;
    return 0U;
#endif

    if ((0U == actual_num) || (actual_num > WIFI_JUSTFLOAT_MAX_FLOAT_NUM))
    {
        s_wifi_justfloat_profile.fail_count++;
        return 1U;
    }

    if (declared_num != actual_num)
    {
        s_wifi_justfloat_profile.fail_count++;
        return 1U;
    }

    if (0U == wifi_cmd_IsReady())
    {
        s_wifi_justfloat_profile.fail_count++;
        return 1U;
    }

    if (0U == wifi_justfloat_should_send())
    {
        s_wifi_justfloat_profile.skip_count++;
        return 0U;
    }

    va_start(ap, actual_num);
    for (i = 0U; i < actual_num; i++)
    {
        float value_f = (float)va_arg(ap, double);
        memcpy(&frame[i * 4U], &value_f, sizeof(float));
    }
    va_end(ap);

    payload_len = (uint16_t)actual_num * 4U;
    frame[payload_len + 0U] = WIFI_JUSTFLOAT_TAIL_0;
    frame[payload_len + 1U] = WIFI_JUSTFLOAT_TAIL_1;
    frame[payload_len + 2U] = WIFI_JUSTFLOAT_TAIL_2;
    frame[payload_len + 3U] = WIFI_JUSTFLOAT_TAIL_3;
    frame_len = payload_len + 4U;

    start_us = timer_get(WIFI_JUSTFLOAT_TIMER_INDEX);
    ret = (0U != wifi_cmd_SendBuffer(frame, frame_len)) ? 0U : 1U;
    cost_us = timer_get(WIFI_JUSTFLOAT_TIMER_INDEX) - start_us;
    wifi_justfloat_profile_update(cost_us, (0U == ret) ? 1U : 0U);
    return ret;
}
