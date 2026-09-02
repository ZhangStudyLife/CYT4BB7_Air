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
 * File: wifi_justfloat.c
 * Module: WiFi JustFloat telemetry
 * Purpose: packetize VOFA JustFloat frames and queue them for non-blocking send.
 *****************************************************************************/

#include "wifi_justfloat.h"

#include <string.h>

#include "../wifi_cmd/wifi_cmd.h"

#define WIFI_JUSTFLOAT_TAIL_0            (0x00U)
#define WIFI_JUSTFLOAT_TAIL_1            (0x00U)
#define WIFI_JUSTFLOAT_TAIL_2            (0x80U)
#define WIFI_JUSTFLOAT_TAIL_3            (0x7FU)
#define WIFI_JUSTFLOAT_USER_MAX_NUM      (WIFI_JUSTFLOAT_MAX_FLOAT_NUM - 1U)
#define WIFI_JUSTFLOAT_FRAME_MAX_BYTES   (WIFI_JUSTFLOAT_MAX_FLOAT_NUM * 4U + 4U)
#define WIFI_JUSTFLOAT_QUEUE_FRAME_NUM   (256U)

extern volatile uint32 tick_1000us_cnt;

static uint8_t s_wifi_justfloat_standby_context = 0U;
static uint8_t s_wifi_justfloat_standby_user_enable = 1U;
static uint32_t s_wifi_justfloat_last_queued_tick = 0xFFFFFFFFU;

static uint8_t s_wifi_justfloat_frame_queue[WIFI_JUSTFLOAT_QUEUE_FRAME_NUM][WIFI_JUSTFLOAT_FRAME_MAX_BYTES];
static uint16_t s_wifi_justfloat_frame_len[WIFI_JUSTFLOAT_QUEUE_FRAME_NUM];
static uint16_t s_wifi_justfloat_q_head = 0U;
static uint16_t s_wifi_justfloat_q_tail = 0U;
static uint16_t s_wifi_justfloat_q_used = 0U;
static uint8_t s_wifi_justfloat_tx_frame[WIFI_JUSTFLOAT_FRAME_MAX_BYTES];

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
    interrupt_global_enable(irq_state);
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
        s_wifi_justfloat_frame_len[tail] = 0U;
        s_wifi_justfloat_q_tail = wifi_justfloat_next_index(tail);
        s_wifi_justfloat_q_used--;
    }
    interrupt_global_enable(irq_state);
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
        return 0U;
    }

    if (0U == wifi_justfloat_pack_frame(timestamp_tick, data, user_num, frame, &frame_len))
    {
        return 1U;
    }

    if (0U == wifi_justfloat_queue_push(frame, frame_len))
    {
        return 1U;
    }

    s_wifi_justfloat_last_queued_tick = timestamp_tick;
    (void)wifi_justfloat_Poll();
    return 0U;
}

void wifi_justfloat_Init(void)
{
    s_wifi_justfloat_standby_context = 0U;
    s_wifi_justfloat_standby_user_enable = 1U;
    s_wifi_justfloat_last_queued_tick = 0xFFFFFFFFU;
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

uint8_t wifi_justfloat_Poll(void)
{
    uint16_t len;

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

    if (0U == wifi_cmd_SendBuffer(s_wifi_justfloat_tx_frame, len))
    {
        return 0U;
    }

    wifi_justfloat_queue_commit(1U);
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
        return 1U;
    }

    if (0U == wifi_cmd_IsReady())
    {
        return 1U;
    }

    if (0U == wifi_justfloat_should_send())
    {
        return 0U;
    }

    return wifi_justfloat_enqueue_frame(data, num);
}
