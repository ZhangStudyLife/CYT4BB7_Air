/*****************************************************************************
 * File: wifi_justfloat.c
 * Module: WiFi JustFloat telemetry
 * Purpose: packetize VOFA JustFloat frames and queue them for non-blocking send.
 *****************************************************************************/

#include "wifi_justfloat.h"

#include <stdarg.h>
#include <string.h>

#include "../wifi_cmd/wifi_cmd.h"

#define WIFI_JUSTFLOAT_TAIL_0            (0x00U)
#define WIFI_JUSTFLOAT_TAIL_1            (0x00U)
#define WIFI_JUSTFLOAT_TAIL_2            (0x80U)
#define WIFI_JUSTFLOAT_TAIL_3            (0x7FU)
#define WIFI_JUSTFLOAT_TIMER_INDEX       (TC_TIME2_CH1)
#define WIFI_JUSTFLOAT_USER_MAX_NUM      (WIFI_JUSTFLOAT_MAX_FLOAT_NUM - 1U)
#define WIFI_JUSTFLOAT_FRAME_MAX_BYTES   (WIFI_JUSTFLOAT_MAX_FLOAT_NUM * 4U + 4U)
#define WIFI_JUSTFLOAT_QUEUE_FRAME_NUM   (384U)

extern volatile uint32 tick_1000us_cnt;

typedef struct
{
    uint32_t last_us;
    uint32_t min_us;
    uint32_t max_us;
    uint64_t sum_us;
    uint32_t ok_count;
    uint32_t fail_count;
    uint32_t skip_count;
    uint32_t queued_count;
    uint32_t overflow_count;
} wifi_justfloat_tx_profile_t;

static uint8_t s_wifi_justfloat_timer_inited = 0U;
static uint8_t s_wifi_justfloat_standby_context = 0U;
static uint8_t s_wifi_justfloat_standby_user_enable = 1U;
static uint32_t s_wifi_justfloat_last_queued_tick = 0xFFFFFFFFU;
static wifi_justfloat_tx_profile_t s_wifi_justfloat_profile = {0};

static uint8_t s_wifi_justfloat_frame_queue[WIFI_JUSTFLOAT_QUEUE_FRAME_NUM][WIFI_JUSTFLOAT_FRAME_MAX_BYTES];
static uint16_t s_wifi_justfloat_frame_len[WIFI_JUSTFLOAT_QUEUE_FRAME_NUM];
static uint16_t s_wifi_justfloat_q_head = 0U;
static uint16_t s_wifi_justfloat_q_tail = 0U;
static uint16_t s_wifi_justfloat_q_used = 0U;
static uint32_t s_wifi_justfloat_q_bytes = 0U;
static uint8_t s_wifi_justfloat_tx_frame[WIFI_JUSTFLOAT_FRAME_MAX_BYTES];

uint8_t wifi_justfloat_Poll(void);

static uint16_t wifi_justfloat_next_index(uint16_t index)
{
    index++;
    if (index >= WIFI_JUSTFLOAT_QUEUE_FRAME_NUM)
    {
        index = 0U;
    }
    return index;
}

static void wifi_justfloat_queue_reset(void)
{
    uint32_t irq_state = interrupt_global_disable();
    s_wifi_justfloat_q_head = 0U;
    s_wifi_justfloat_q_tail = 0U;
    s_wifi_justfloat_q_used = 0U;
    s_wifi_justfloat_q_bytes = 0U;
    interrupt_global_enable(irq_state);
}

static uint32_t wifi_justfloat_queue_bytes(void)
{
    uint32_t bytes;
    uint32_t irq_state = interrupt_global_disable();
    bytes = s_wifi_justfloat_q_bytes;
    interrupt_global_enable(irq_state);
    return bytes;
}

static uint8_t wifi_justfloat_queue_push(const uint8_t *frame, uint16_t frame_len)
{
    uint16_t head;
    uint32_t irq_state;

    if ((NULL == frame) || (0U == frame_len) || (frame_len > WIFI_JUSTFLOAT_FRAME_MAX_BYTES))
    {
        return 0U;
    }

    irq_state = interrupt_global_disable();
    if (s_wifi_justfloat_q_used >= WIFI_JUSTFLOAT_QUEUE_FRAME_NUM)
    {
        interrupt_global_enable(irq_state);
        return 0U;
    }

    head = s_wifi_justfloat_q_head;
    memcpy(s_wifi_justfloat_frame_queue[head], frame, frame_len);
    s_wifi_justfloat_frame_len[head] = frame_len;
    s_wifi_justfloat_q_head = wifi_justfloat_next_index(head);
    s_wifi_justfloat_q_used++;
    s_wifi_justfloat_q_bytes += frame_len;
    interrupt_global_enable(irq_state);
    return 1U;
}

static uint16_t wifi_justfloat_queue_peek_frame(uint8_t *out, uint16_t max_len)
{
    uint16_t len = 0U;
    uint32_t irq_state;

    if ((NULL == out) || (0U == max_len))
    {
        return 0U;
    }

    irq_state = interrupt_global_disable();
    if (s_wifi_justfloat_q_used > 0U)
    {
        uint16_t tail = s_wifi_justfloat_q_tail;
        len = s_wifi_justfloat_frame_len[tail];
        if ((0U != len) && (len <= max_len))
        {
            memcpy(out, s_wifi_justfloat_frame_queue[tail], len);
        }
        else
        {
            len = 0U;
        }
    }
    interrupt_global_enable(irq_state);
    return len;
}

static void wifi_justfloat_queue_commit(uint16_t frame_count)
{
    uint16_t i;
    uint32_t irq_state;

    if (0U == frame_count)
    {
        return;
    }

    irq_state = interrupt_global_disable();
    for (i = 0U; (i < frame_count) && (s_wifi_justfloat_q_used > 0U); i++)
    {
        uint16_t tail = s_wifi_justfloat_q_tail;
        uint16_t len = s_wifi_justfloat_frame_len[tail];
        s_wifi_justfloat_frame_len[tail] = 0U;
        s_wifi_justfloat_q_tail = wifi_justfloat_next_index(tail);
        s_wifi_justfloat_q_used--;
        if (s_wifi_justfloat_q_bytes >= len)
        {
            s_wifi_justfloat_q_bytes -= len;
        }
        else
        {
            s_wifi_justfloat_q_bytes = 0U;
        }
    }
    interrupt_global_enable(irq_state);
}

static void wifi_justfloat_profile_reset(void)
{
    memset(&s_wifi_justfloat_profile, 0, sizeof(s_wifi_justfloat_profile));
    s_wifi_justfloat_profile.min_us = 0xFFFFFFFFU;
}

static uint8_t wifi_justfloat_should_send(void)
{
    if ((0U != s_wifi_justfloat_standby_context) && (0U == s_wifi_justfloat_standby_user_enable))
    {
        return 0U;
    }

    if (0U != wifi_cmd_IsTextBusy())
    {
        return 0U;
    }

    return 1U;
}

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

static uint8_t wifi_justfloat_pack_frame(uint32_t timestamp_tick,
                                         const float *data, uint8_t user_num,
                                         uint8_t *frame, uint16_t *frame_len)
{
    uint8_t i;
    uint8_t total_num;
    uint16_t payload_len;
    float timestamp = (float)timestamp_tick;

    if ((NULL == data) || (NULL == frame) || (NULL == frame_len) ||
        (user_num > WIFI_JUSTFLOAT_USER_MAX_NUM))
    {
        return 0U;
    }

    memcpy(&frame[0], &timestamp, sizeof(float));
    for (i = 0U; i < user_num; i++)
    {
        memcpy(&frame[(uint16_t)(i + 1U) * 4U], &data[i], sizeof(float));
    }

    total_num = (uint8_t)(user_num + 1U);
    payload_len = (uint16_t)total_num * 4U;
    frame[payload_len + 0U] = WIFI_JUSTFLOAT_TAIL_0;
    frame[payload_len + 1U] = WIFI_JUSTFLOAT_TAIL_1;
    frame[payload_len + 2U] = WIFI_JUSTFLOAT_TAIL_2;
    frame[payload_len + 3U] = WIFI_JUSTFLOAT_TAIL_3;
    *frame_len = payload_len + 4U;
    return 1U;
}

static uint8_t wifi_justfloat_enqueue_frame(const float *data, uint8_t user_num)
{
    uint32_t timestamp_tick = tick_1000us_cnt;
    uint16_t frame_len;
    uint8_t frame[WIFI_JUSTFLOAT_FRAME_MAX_BYTES];

    if (timestamp_tick == s_wifi_justfloat_last_queued_tick)
    {
        s_wifi_justfloat_profile.skip_count++;
        return 0U;
    }

    if (0U == wifi_justfloat_pack_frame(timestamp_tick, data, user_num, frame, &frame_len))
    {
        s_wifi_justfloat_profile.fail_count++;
        return 1U;
    }

    if (0U == wifi_justfloat_queue_push(frame, frame_len))
    {
        s_wifi_justfloat_profile.overflow_count++;
        return 1U;
    }

    s_wifi_justfloat_last_queued_tick = timestamp_tick;
    s_wifi_justfloat_profile.queued_count++;
    (void)wifi_justfloat_Poll();
    return 0U;
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
    s_wifi_justfloat_last_queued_tick = 0xFFFFFFFFU;
    wifi_justfloat_profile_reset();
    wifi_justfloat_queue_reset();
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
    stats->queued_count = s_wifi_justfloat_profile.queued_count;
    stats->overflow_count = s_wifi_justfloat_profile.overflow_count;
    stats->pending_bytes = wifi_justfloat_queue_bytes();

    avg_us = (s_wifi_justfloat_profile.ok_count > 0U)
                 ? (s_wifi_justfloat_profile.sum_us / (uint64_t)s_wifi_justfloat_profile.ok_count)
                 : 0U;
    stats->avg_us = (uint32_t)avg_us;
}

uint8_t wifi_justfloat_Poll(void)
{
    uint16_t len;
    uint32_t start_us;
    uint32_t cost_us;
    uint8_t ok;

#if (1U == WIFI_IMAGE_ENABLE)
    return 0U;
#endif

    if ((0U == wifi_cmd_IsReady()) || (0U == wifi_justfloat_should_send()) ||
        (0U != wifi_cmd_IsRawBusy()))
    {
        return 0U;
    }

    len = wifi_justfloat_queue_peek_frame(s_wifi_justfloat_tx_frame,
                                          (uint16_t)sizeof(s_wifi_justfloat_tx_frame));
    if (0U == len)
    {
        return 0U;
    }

    start_us = timer_get(WIFI_JUSTFLOAT_TIMER_INDEX);
    ok = wifi_cmd_SendBuffer(s_wifi_justfloat_tx_frame, len);
    cost_us = timer_get(WIFI_JUSTFLOAT_TIMER_INDEX) - start_us;

    if (0U == ok)
    {
        wifi_justfloat_profile_update(cost_us, 0U);
        return 0U;
    }

    wifi_justfloat_queue_commit(1U);
    wifi_justfloat_profile_update(cost_us, 1U);
    return 1U;
}

uint8_t wifi_justfloat_Array(const float *data, uint8_t num)
{
#if (1U == WIFI_IMAGE_ENABLE)
    (void)data;
    (void)num;
    return 0U;
#endif

    if ((NULL == data) || (num > WIFI_JUSTFLOAT_USER_MAX_NUM))
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

    return wifi_justfloat_enqueue_frame(data, num);
}

uint8_t wifi_justfloat_Impl(uint8_t declared_num, uint8_t actual_num, ...)
{
    uint8_t i;
    uint8_t ret;
    float values[WIFI_JUSTFLOAT_USER_MAX_NUM];
    va_list ap;

#if (1U == WIFI_IMAGE_ENABLE)
    (void)declared_num;
    (void)actual_num;
    return 0U;
#endif

    if (actual_num > WIFI_JUSTFLOAT_USER_MAX_NUM)
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
        values[i] = (float)va_arg(ap, double);
    }
    va_end(ap);

    ret = wifi_justfloat_enqueue_frame(values, actual_num);
    return ret;
}
