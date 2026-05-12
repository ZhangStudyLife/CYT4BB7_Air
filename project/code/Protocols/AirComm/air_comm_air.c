#include "air_comm_air.h"

#define AIR_COMM_HEADER_0                    (0xAAU)
#define AIR_COMM_HEADER_1                    (0xAAU)
#define AIR_COMM_HEADER_2                    (0x55U)
#define AIR_COMM_HEADER_3                    (0x55U)

#define AIR_COMM_MAX_PAYLOAD                 (250U)
#define AIR_COMM_FRAME_OVERHEAD              (9U)
#define AIR_COMM_MAX_FRAME                   (AIR_COMM_MAX_PAYLOAD + AIR_COMM_FRAME_OVERHEAD)
#define AIR_COMM_RX_QUEUE_SIZE               (512U)
#define AIR_COMM_PARAM_TABLE_MAX             (12U)
#define AIR_COMM_FUNC_TABLE_MAX              (16U)

#define AIR_COMM_MSG_SET_PARAM               (0x01U)
#define AIR_COMM_MSG_ACK_PARAM               (0x02U)
#define AIR_COMM_MSG_EXEC_FUNC               (0x03U)
#define AIR_COMM_MSG_ACK_FUNC                (0x04U)
#define AIR_COMM_MSG_HEARTBEAT               (0x05U)
#define AIR_COMM_MSG_RUN_DATA                (0x06U)

#define AIR_COMM_HEARTBEAT_MS                (200U)
#define AIR_COMM_OFFLINE_MS                  (600U)

typedef struct
{
    const char *name;
    float *var;
    float min;
    float max;
} air_comm_air_param_t;

typedef struct
{
    uint8 func_id;
    void (*func)(void);
} air_comm_air_func_t;

typedef struct
{
    uint8 state;
    uint8 header_count;
    uint8 info_count;
    uint8 type;
    uint8 seq;
    uint8 len;
    uint8 payload_count;
    uint8 payload[AIR_COMM_MAX_PAYLOAD];
    uint16 crc;
    uint8 crc_count;
} air_comm_air_rx_parser_t;

typedef struct
{
    uint8 data[AIR_COMM_RX_QUEUE_SIZE];
    volatile uint16 head;
    volatile uint16 tail;
} air_comm_air_rx_queue_t;

float air_min_area = 5.0f;
float air_hold_ms = 30.0f;
float air_x_bias = 0.0f;
float air_y_bias = 0.0f;

static uint8 s_air_comm_initialized = 0U;
static uint8 s_air_comm_seq = 0U;
static uint32 s_air_comm_tick_ms = 0U;
static uint32 s_air_comm_last_heartbeat_ms = 0U;
static uint32 s_air_comm_last_car_ms = 0U;

static air_comm_air_rx_parser_t s_air_comm_rx;
static air_comm_air_rx_queue_t s_air_comm_rx_queue;
static air_comm_air_stats_t s_air_comm_stats;
static air_comm_air_param_t s_air_comm_params[AIR_COMM_PARAM_TABLE_MAX];
static air_comm_air_func_t s_air_comm_funcs[AIR_COMM_FUNC_TABLE_MAX];
static uint8 s_air_comm_param_count = 0U;
static uint8 s_air_comm_func_count = 0U;

static uint16 air_comm_crc16(const uint8 *data, uint16 len)
{
    uint16 crc = 0xFFFFU;
    uint16 i;
    uint8 j;

    for(i = 0U; i < len; i++)
    {
        crc ^= (uint16)data[i] << 8;
        for(j = 0U; j < 8U; j++)
        {
            if((crc & 0x8000U) != 0U)
            {
                crc = (uint16)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

static float air_comm_read_float(const uint8 *buffer)
{
    float value;
    uint8 *ptr = (uint8 *)&value;

    ptr[0] = buffer[0];
    ptr[1] = buffer[1];
    ptr[2] = buffer[2];
    ptr[3] = buffer[3];

    return value;
}

static void air_comm_write_float(uint8 *buffer, float value)
{
    uint8 *ptr = (uint8 *)&value;

    buffer[0] = ptr[0];
    buffer[1] = ptr[1];
    buffer[2] = ptr[2];
    buffer[3] = ptr[3];
}

static void air_comm_write_u32(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
    buffer[2] = (uint8)((value >> 16) & 0xFFU);
    buffer[3] = (uint8)((value >> 24) & 0xFFU);
}

static uint8 air_comm_name_equal(const char *name, const uint8 *bytes, uint8 len)
{
    uint8 i;

    if((name == NULL) || (bytes == NULL))
    {
        return 0U;
    }

    for(i = 0U; i < len; i++)
    {
        if(name[i] == '\0')
        {
            return 0U;
        }
        if((uint8)name[i] != bytes[i])
        {
            return 0U;
        }
    }

    return (name[len] == '\0') ? 1U : 0U;
}

static uint8 air_comm_send_uart(const uint8 *data, uint16 len)
{
    if((data == NULL) || (len == 0U))
    {
        return 0U;
    }

    uart_write_buffer(UART_2, data, len);
    return 1U;
}

static uint8 air_comm_send_frame(uint8 type, uint8 seq, const uint8 *payload, uint8 len)
{
    uint8 frame[AIR_COMM_MAX_FRAME];
    uint16 pos = 0U;
    uint16 crc;

    if((len > AIR_COMM_MAX_PAYLOAD) || ((len > 0U) && (payload == NULL)))
    {
        return 0U;
    }

    frame[pos++] = AIR_COMM_HEADER_0;
    frame[pos++] = AIR_COMM_HEADER_1;
    frame[pos++] = AIR_COMM_HEADER_2;
    frame[pos++] = AIR_COMM_HEADER_3;
    frame[pos++] = type;
    frame[pos++] = seq;
    frame[pos++] = len;

    if(len > 0U)
    {
        memcpy(&frame[pos], payload, len);
        pos = (uint16)(pos + len);
    }

    crc = air_comm_crc16(frame, pos);
    frame[pos++] = (uint8)(crc & 0xFFU);
    frame[pos++] = (uint8)((crc >> 8) & 0xFFU);

    if(air_comm_send_uart(frame, pos) == 0U)
    {
        return 0U;
    }

    s_air_comm_stats.tx_frame_count++;
    s_air_comm_stats.tx_byte_count += pos;
    if(type == AIR_COMM_MSG_HEARTBEAT)
    {
        s_air_comm_stats.heartbeat_tx_count++;
    }

    return 1U;
}

static void air_comm_send_heartbeat(void)
{
    uint8 payload[6];

    payload[0] = 0U;
    payload[1] = 0U;
    air_comm_write_u32(&payload[2], s_air_comm_tick_ms);

    if(air_comm_send_frame(AIR_COMM_MSG_HEARTBEAT, s_air_comm_seq, payload, 6U) != 0U)
    {
        s_air_comm_seq++;
    }
}

static uint8 air_comm_send_ack_param(uint8 seq,
                                     uint8 status,
                                     const uint8 *name,
                                     uint8 name_len,
                                     float actual)
{
    uint8 payload[2U + AIR_COMM_AIR_PARAM_NAME_MAX + 4U];
    uint8 pos = 0U;

    if(name_len > AIR_COMM_AIR_PARAM_NAME_MAX)
    {
        name_len = 0U;
        name = NULL;
    }

    payload[pos++] = status;
    payload[pos++] = name_len;
    if((name_len > 0U) && (name != NULL))
    {
        memcpy(&payload[pos], name, name_len);
        pos = (uint8)(pos + name_len);
    }
    air_comm_write_float(&payload[pos], actual);
    pos = (uint8)(pos + 4U);

    return air_comm_send_frame(AIR_COMM_MSG_ACK_PARAM, seq, payload, pos);
}

static uint8 air_comm_send_ack_func(uint8 seq, uint8 func_id, uint8 status, float result)
{
    uint8 payload[6];

    payload[0] = func_id;
    payload[1] = status;
    air_comm_write_float(&payload[2], result);

    return air_comm_send_frame(AIR_COMM_MSG_ACK_FUNC, seq, payload, 6U);
}

static void air_comm_mark_car_online(void)
{
    s_air_comm_last_car_ms = s_air_comm_tick_ms;
    s_air_comm_stats.online_status = 1U;
}

static air_comm_air_param_t *air_comm_find_param(const uint8 *name, uint8 name_len)
{
    uint8 i;

    for(i = 0U; i < s_air_comm_param_count; i++)
    {
        if(air_comm_name_equal(s_air_comm_params[i].name, name, name_len) != 0U)
        {
            return &s_air_comm_params[i];
        }
    }

    return NULL;
}

static air_comm_air_func_t *air_comm_find_func(uint8 func_id)
{
    uint8 i;

    for(i = 0U; i < s_air_comm_func_count; i++)
    {
        if(s_air_comm_funcs[i].func_id == func_id)
        {
            return &s_air_comm_funcs[i];
        }
    }

    return NULL;
}

static void air_comm_handle_set_param(uint8 seq, const uint8 *payload, uint8 len)
{
    uint8 name_len;
    const uint8 *name;
    float value;
    float actual = 0.0f;
    uint8 status = AIR_COMM_AIR_STATUS_ERROR;
    air_comm_air_param_t *param = NULL;

    if((payload == NULL) || (len < 5U))
    {
        s_air_comm_stats.set_param_fail_count++;
        (void)air_comm_send_ack_param(seq, status, NULL, 0U, actual);
        return;
    }

    name_len = payload[0];
    name = &payload[1];
    if((name_len == 0U) ||
       (name_len > AIR_COMM_AIR_PARAM_NAME_MAX) ||
       (len < (uint8)(1U + name_len + 4U)))
    {
        s_air_comm_stats.set_param_fail_count++;
        (void)air_comm_send_ack_param(seq, status, NULL, 0U, actual);
        return;
    }

    value = air_comm_read_float(&payload[1U + name_len]);
    param = air_comm_find_param(name, name_len);
    if(param == NULL)
    {
        status = AIR_COMM_AIR_STATUS_NOT_FOUND;
    }
    else if(value < param->min)
    {
        actual = param->min;
        *(param->var) = actual;
        status = AIR_COMM_AIR_STATUS_OUT_OF_RANGE;
    }
    else if(value > param->max)
    {
        actual = param->max;
        *(param->var) = actual;
        status = AIR_COMM_AIR_STATUS_OUT_OF_RANGE;
    }
    else
    {
        actual = value;
        *(param->var) = actual;
        status = AIR_COMM_AIR_STATUS_OK;
    }

    if(status == AIR_COMM_AIR_STATUS_OK)
    {
        s_air_comm_stats.set_param_ok_count++;
    }
    else
    {
        s_air_comm_stats.set_param_fail_count++;
    }

    (void)air_comm_send_ack_param(seq, status, name, name_len, actual);
}

static void air_comm_handle_exec_func(uint8 seq, const uint8 *payload, uint8 len)
{
    uint8 func_id = 0U;
    uint8 status = AIR_COMM_AIR_STATUS_ERROR;
    air_comm_air_func_t *func = NULL;

    if((payload != NULL) && (len >= 1U))
    {
        func_id = payload[0];
        func = air_comm_find_func(func_id);
        if(func == NULL)
        {
            status = AIR_COMM_AIR_STATUS_NOT_FOUND;
        }
        else
        {
            func->func();
            status = AIR_COMM_AIR_STATUS_OK;
        }
    }

    if(status == AIR_COMM_AIR_STATUS_OK)
    {
        s_air_comm_stats.exec_func_ok_count++;
    }
    else
    {
        s_air_comm_stats.exec_func_fail_count++;
    }

    (void)air_comm_send_ack_func(seq, func_id, status, 0.0f);
}

static void air_comm_handle_frame(uint8 type, uint8 seq, const uint8 *payload, uint8 len)
{
    switch(type)
    {
        case AIR_COMM_MSG_SET_PARAM:
            air_comm_handle_set_param(seq, payload, len);
            break;

        case AIR_COMM_MSG_EXEC_FUNC:
            air_comm_handle_exec_func(seq, payload, len);
            break;

        case AIR_COMM_MSG_HEARTBEAT:
            s_air_comm_stats.heartbeat_rx_count++;
            air_comm_mark_car_online();
            break;

        default:
            break;
    }
}

static void air_comm_process_rx_frame(void)
{
    uint8 frame[AIR_COMM_MAX_FRAME];
    uint16 pos = 0U;
    uint16 crc_calc;

    frame[pos++] = AIR_COMM_HEADER_0;
    frame[pos++] = AIR_COMM_HEADER_1;
    frame[pos++] = AIR_COMM_HEADER_2;
    frame[pos++] = AIR_COMM_HEADER_3;
    frame[pos++] = s_air_comm_rx.type;
    frame[pos++] = s_air_comm_rx.seq;
    frame[pos++] = s_air_comm_rx.len;

    if(s_air_comm_rx.len > 0U)
    {
        memcpy(&frame[pos], s_air_comm_rx.payload, s_air_comm_rx.len);
        pos = (uint16)(pos + s_air_comm_rx.len);
    }

    crc_calc = air_comm_crc16(frame, pos);
    if(crc_calc != s_air_comm_rx.crc)
    {
        s_air_comm_stats.crc_error_count++;
        return;
    }

    s_air_comm_stats.rx_frame_count++;
    s_air_comm_stats.rx_byte_count += (uint32)(pos + 2U);
    air_comm_handle_frame(s_air_comm_rx.type,
                          s_air_comm_rx.seq,
                          s_air_comm_rx.payload,
                          s_air_comm_rx.len);
}

static void air_comm_rx_parser_reset(void)
{
    s_air_comm_rx.state = 0U;
    s_air_comm_rx.header_count = 0U;
    s_air_comm_rx.info_count = 0U;
    s_air_comm_rx.type = 0U;
    s_air_comm_rx.seq = 0U;
    s_air_comm_rx.len = 0U;
    s_air_comm_rx.payload_count = 0U;
    s_air_comm_rx.crc = 0U;
    s_air_comm_rx.crc_count = 0U;
}

static void air_comm_rx_byte_parser(uint8 byte)
{
    switch(s_air_comm_rx.state)
    {
        case 0:
            if((s_air_comm_rx.header_count == 0U) && (byte == AIR_COMM_HEADER_0))
            {
                s_air_comm_rx.header_count = 1U;
            }
            else if((s_air_comm_rx.header_count == 1U) && (byte == AIR_COMM_HEADER_1))
            {
                s_air_comm_rx.header_count = 2U;
            }
            else if((s_air_comm_rx.header_count == 2U) && (byte == AIR_COMM_HEADER_2))
            {
                s_air_comm_rx.header_count = 3U;
            }
            else if((s_air_comm_rx.header_count == 3U) && (byte == AIR_COMM_HEADER_3))
            {
                s_air_comm_rx.state = 1U;
                s_air_comm_rx.info_count = 0U;
            }
            else
            {
                s_air_comm_rx.header_count = (byte == AIR_COMM_HEADER_0) ? 1U : 0U;
            }
            break;

        case 1:
            if(s_air_comm_rx.info_count == 0U)
            {
                s_air_comm_rx.type = byte;
                s_air_comm_rx.info_count = 1U;
            }
            else if(s_air_comm_rx.info_count == 1U)
            {
                s_air_comm_rx.seq = byte;
                s_air_comm_rx.info_count = 2U;
            }
            else
            {
                if(byte > AIR_COMM_MAX_PAYLOAD)
                {
                    s_air_comm_stats.rx_oversize_count++;
                    air_comm_rx_parser_reset();
                    break;
                }
                s_air_comm_rx.len = byte;
                s_air_comm_rx.payload_count = 0U;
                s_air_comm_rx.crc = 0U;
                s_air_comm_rx.crc_count = 0U;
                s_air_comm_rx.state = (byte == 0U) ? 3U : 2U;
            }
            break;

        case 2:
            if(s_air_comm_rx.payload_count < AIR_COMM_MAX_PAYLOAD)
            {
                s_air_comm_rx.payload[s_air_comm_rx.payload_count++] = byte;
                if(s_air_comm_rx.payload_count >= s_air_comm_rx.len)
                {
                    s_air_comm_rx.state = 3U;
                    s_air_comm_rx.crc = 0U;
                    s_air_comm_rx.crc_count = 0U;
                }
            }
            else
            {
                s_air_comm_stats.rx_oversize_count++;
                air_comm_rx_parser_reset();
            }
            break;

        case 3:
            if(s_air_comm_rx.crc_count == 0U)
            {
                s_air_comm_rx.crc = byte;
                s_air_comm_rx.crc_count = 1U;
            }
            else
            {
                s_air_comm_rx.crc |= (uint16)byte << 8;
                air_comm_process_rx_frame();
                air_comm_rx_parser_reset();
            }
            break;

        default:
            air_comm_rx_parser_reset();
            break;
    }
}

static uint8 air_comm_rx_queue_pop(uint8 *byte)
{
    uint16 tail;

    if(byte == NULL)
    {
        return 0U;
    }

    tail = s_air_comm_rx_queue.tail;
    if(tail == s_air_comm_rx_queue.head)
    {
        return 0U;
    }

    *byte = s_air_comm_rx_queue.data[tail];
    tail++;
    if(tail >= AIR_COMM_RX_QUEUE_SIZE)
    {
        tail = 0U;
    }
    s_air_comm_rx_queue.tail = tail;

    return 1U;
}

static void air_comm_task_online(void)
{
    if((s_air_comm_stats.online_status != 0U) &&
       ((s_air_comm_tick_ms - s_air_comm_last_car_ms) > AIR_COMM_OFFLINE_MS))
    {
        s_air_comm_stats.online_status = 2U;
    }
}

void air_comm_air_init(void)
{
    memset(&s_air_comm_rx, 0, sizeof(s_air_comm_rx));
    memset(&s_air_comm_rx_queue, 0, sizeof(s_air_comm_rx_queue));
    memset(&s_air_comm_stats, 0, sizeof(s_air_comm_stats));
    memset(s_air_comm_params, 0, sizeof(s_air_comm_params));
    memset(s_air_comm_funcs, 0, sizeof(s_air_comm_funcs));

    s_air_comm_initialized = 0U;
    s_air_comm_seq = 0U;
    s_air_comm_tick_ms = 0U;
    s_air_comm_last_heartbeat_ms = 0U;
    s_air_comm_last_car_ms = 0U;
    s_air_comm_param_count = 0U;
    s_air_comm_func_count = 0U;

    air_min_area = 5.0f;
    air_hold_ms = 30.0f;
    air_x_bias = 0.0f;
    air_y_bias = 0.0f;

    (void)air_comm_air_register_param("air_min_area", &air_min_area, 0.0f, 500.0f);
    (void)air_comm_air_register_param("air_hold_ms", &air_hold_ms, 0.0f, 200.0f);
    (void)air_comm_air_register_param("air_x_bias", &air_x_bias, -40.0f, 40.0f);
    (void)air_comm_air_register_param("air_y_bias", &air_y_bias, -40.0f, 40.0f);

    uart_init(UART_2, AIR_COMM_AIR_BAUDRATE, UART2_TX_P10_1, UART2_RX_P10_0);
    uart_rx_interrupt(UART_2, 1U);

    s_air_comm_initialized = 1U;
}

void air_comm_air_tick_1MS(void)
{
    if(s_air_comm_initialized == 0U)
    {
        return;
    }

    s_air_comm_tick_ms++;
}

void air_comm_air_poll(void)
{
    uint8 byte;
    uint16 guard = AIR_COMM_RX_QUEUE_SIZE;

    if(s_air_comm_initialized == 0U)
    {
        return;
    }

    while((guard > 0U) && (air_comm_rx_queue_pop(&byte) != 0U))
    {
        air_comm_rx_byte_parser(byte);
        guard--;
    }

    air_comm_task_online();
}

void air_comm_air_update_100HZ(void)
{
    if(s_air_comm_initialized == 0U)
    {
        return;
    }

    if((s_air_comm_tick_ms - s_air_comm_last_heartbeat_ms) >= AIR_COMM_HEARTBEAT_MS)
    {
        air_comm_send_heartbeat();
        s_air_comm_last_heartbeat_ms = s_air_comm_tick_ms;
    }

    air_comm_task_online();
}

void air_comm_air_rx_byte(uint8 byte)
{
    uint16 next_head;

    if(s_air_comm_initialized == 0U)
    {
        return;
    }

    next_head = s_air_comm_rx_queue.head + 1U;
    if(next_head >= AIR_COMM_RX_QUEUE_SIZE)
    {
        next_head = 0U;
    }

    if(next_head == s_air_comm_rx_queue.tail)
    {
        s_air_comm_stats.rx_queue_overflow_count++;
        return;
    }

    s_air_comm_rx_queue.data[s_air_comm_rx_queue.head] = byte;
    s_air_comm_rx_queue.head = next_head;
    s_air_comm_stats.raw_rx_byte_count++;
}

uint8 air_comm_air_is_car_online(void)
{
    return (s_air_comm_stats.online_status == 1U) ? 1U : 0U;
}

uint8 air_comm_air_register_param(const char *name, float *var, float min, float max)
{
    uint8 i;

    if((name == NULL) || (var == NULL) || (min > max))
    {
        return 0U;
    }

    for(i = 0U; i < s_air_comm_param_count; i++)
    {
        if(strcmp(s_air_comm_params[i].name, name) == 0)
        {
            s_air_comm_params[i].var = var;
            s_air_comm_params[i].min = min;
            s_air_comm_params[i].max = max;
            return 1U;
        }
    }

    if(s_air_comm_param_count >= AIR_COMM_PARAM_TABLE_MAX)
    {
        return 0U;
    }

    if(strlen(name) > AIR_COMM_AIR_PARAM_NAME_MAX)
    {
        return 0U;
    }

    s_air_comm_params[s_air_comm_param_count].name = name;
    s_air_comm_params[s_air_comm_param_count].var = var;
    s_air_comm_params[s_air_comm_param_count].min = min;
    s_air_comm_params[s_air_comm_param_count].max = max;
    s_air_comm_param_count++;

    return 1U;
}

uint8 air_comm_air_register_func(uint8 func_id, void (*func)(void))
{
    uint8 i;

    if(func == NULL)
    {
        return 0U;
    }

    for(i = 0U; i < s_air_comm_func_count; i++)
    {
        if(s_air_comm_funcs[i].func_id == func_id)
        {
            s_air_comm_funcs[i].func = func;
            return 1U;
        }
    }

    if(s_air_comm_func_count >= AIR_COMM_FUNC_TABLE_MAX)
    {
        return 0U;
    }

    s_air_comm_funcs[s_air_comm_func_count].func_id = func_id;
    s_air_comm_funcs[s_air_comm_func_count].func = func;
    s_air_comm_func_count++;

    return 1U;
}

uint8 air_comm_air_send_run_data(const float *data, uint8 count)
{
    uint8 payload[1U + (AIR_COMM_AIR_RUN_DATA_MAX_FLOATS * 4U)];
    uint8 pos = 0U;
    uint8 i;

    if((s_air_comm_initialized == 0U) ||
       (data == NULL) ||
       (count == 0U) ||
       (count > AIR_COMM_AIR_RUN_DATA_MAX_FLOATS))
    {
        return 0U;
    }

    payload[pos++] = count;
    for(i = 0U; i < count; i++)
    {
        air_comm_write_float(&payload[pos], data[i]);
        pos = (uint8)(pos + 4U);
    }

    if(air_comm_send_frame(AIR_COMM_MSG_RUN_DATA, s_air_comm_seq, payload, pos) == 0U)
    {
        return 0U;
    }

    s_air_comm_seq++;
    return 1U;
}

void air_comm_air_get_stats(air_comm_air_stats_t *stats)
{
    if(stats == NULL)
    {
        return;
    }

    s_air_comm_stats.tick_ms = s_air_comm_tick_ms;
    *stats = s_air_comm_stats;
}
