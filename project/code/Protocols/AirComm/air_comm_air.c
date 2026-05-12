#include "air_comm_air.h"

/* 帧头 4 字节：0xAA 0xAA 0x55 0x55，选这个是为了在串口噪声中容易识别 */
#define AIR_COMM_HEADER_0                    (0xAAU)
#define AIR_COMM_HEADER_1                    (0xAAU)
#define AIR_COMM_HEADER_2                    (0x55U)
#define AIR_COMM_HEADER_3                    (0x55U)

#define AIR_COMM_MAX_PAYLOAD                 (250U)  /* 单帧 payload 最大字节数 */
#define AIR_COMM_FRAME_OVERHEAD              (9U)    /* 帧开销：4帧头 + type + seq + len + 2crc */
#define AIR_COMM_MAX_FRAME                   (AIR_COMM_MAX_PAYLOAD + AIR_COMM_FRAME_OVERHEAD)
#define AIR_COMM_RX_QUEUE_SIZE               (512U)  /* 接收环形队列大小（字节） */
#define AIR_COMM_PARAM_TABLE_MAX             (12U)   /* 最多注册参数个数 */
#define AIR_COMM_FUNC_TABLE_MAX              (16U)   /* 最多注册函数个数 */

/* 消息类型定义 */
#define AIR_COMM_MSG_SET_PARAM               (0x01U) /* 小车→无人机：设置参数 */
#define AIR_COMM_MSG_ACK_PARAM               (0x02U) /* 无人机→小车：参数设置回执 */
#define AIR_COMM_MSG_EXEC_FUNC               (0x03U) /* 小车→无人机：调用函数 */
#define AIR_COMM_MSG_ACK_FUNC                (0x04U) /* 无人机→小车：函数调用回执 */
#define AIR_COMM_MSG_HEARTBEAT               (0x05U) /* 双向：心跳 */
#define AIR_COMM_MSG_RUN_DATA                (0x06U) /* 无人机→小车：运行数据上报 */

#define AIR_COMM_HEARTBEAT_MS                (200U)  /* 心跳发送间隔（ms） */
#define AIR_COMM_OFFLINE_MS                  (600U)  /* 超过此时间没收到心跳判定离线 */

/* 参数表条目：名字、绑定变量指针、允许范围 */
typedef struct
{
    const char *name;
    float *var;
    float min;
    float max;
} air_comm_air_param_t;

/* 函数表条目：func_id 编号 + 函数指针 */
typedef struct
{
    uint8 func_id;
    void (*func)(void);
} air_comm_air_func_t;

/*
 * 接收解析器状态机。
 * state=0: 找帧头（逐字节匹配 4 字节 header）
 * state=1: 读 info 字段（type, seq, len）
 * state=2: 收 payload（len 个字节）
 * state=3: 收 CRC（2 字节，低字节在前）
 */
typedef struct
{
    uint8 state;            /* 当前状态 0~3 */
    uint8 header_count;     /* 已匹配的帧头字节数 */
    uint8 info_count;       /* 已读取的 info 字段数（0=type, 1=seq, 2=len） */
    uint8 type;             /* 消息类型 */
    uint8 seq;              /* 序列号 */
    uint8 len;              /* payload 长度 */
    uint8 payload_count;    /* 已接收的 payload 字节数 */
    uint8 payload[AIR_COMM_MAX_PAYLOAD]; /* payload 缓冲 */
    uint16 crc;             /* 接收到的 CRC 值 */
    uint8 crc_count;        /* 已接收的 CRC 字节数 */
} air_comm_air_rx_parser_t;

/* 接收环形队列，中断入队，poll 出队 */
typedef struct
{
    uint8 data[AIR_COMM_RX_QUEUE_SIZE];
    volatile uint16 head;   /* 写指针，中断更新 */
    volatile uint16 tail;   /* 读指针，poll 更新 */
} air_comm_air_rx_queue_t;

/* 可被小车远程设置的无人机参数（默认值在 init 里重置） */
float air_min_area = 5.0f;   /* 信标检测最小面积 */
float air_hold_ms = 30.0f;   /* 跟踪保持时间（ms） */
float air_x_bias = 0.0f;     /* X 偏差补偿 */
float air_y_bias = 0.0f;     /* Y 偏差补偿 */

/* 模块私有状态 */
static uint8 s_air_comm_initialized = 0U;       /* init 完成标志 */
static uint8 s_air_comm_seq = 0U;               /* 发送帧序列号，每发一帧自增 */
static uint32 s_air_comm_tick_ms = 0U;          /* 1ms 累加计数器 */
static uint32 s_air_comm_last_heartbeat_ms = 0U; /* 上次发心跳的 tick */
static uint32 s_air_comm_last_car_ms = 0U;      /* 上次收到小车心跳的 tick */

static air_comm_air_rx_parser_t s_air_comm_rx;          /* 接收解析器 */
static air_comm_air_rx_queue_t s_air_comm_rx_queue;     /* 接收环形队列 */
static air_comm_air_stats_t s_air_comm_stats;           /* 统计计数器 */
static air_comm_air_param_t s_air_comm_params[AIR_COMM_PARAM_TABLE_MAX]; /* 参数表 */
static air_comm_air_func_t s_air_comm_funcs[AIR_COMM_FUNC_TABLE_MAX];    /* 函数表 */
static uint8 s_air_comm_param_count = 0U;  /* 已注册参数数 */
static uint8 s_air_comm_func_count = 0U;   /* 已注册函数数 */

/*
 * CRC-16/CCITT-FALSE 计算。
 * 多项式 0x1021，初值 0xFFFF，覆盖 type + seq + len + payload（不含帧头）。
 * 发送时算完 CRC 追加到帧尾，接收时重新算一遍比对。
 */
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

/* 从字节缓冲区按小端序读取 float */
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

/* 按小端序将 float 写入字节缓冲区 */
static void air_comm_write_float(uint8 *buffer, float value)
{
    uint8 *ptr = (uint8 *)&value;

    buffer[0] = ptr[0];
    buffer[1] = ptr[1];
    buffer[2] = ptr[2];
    buffer[3] = ptr[3];
}

/* 按小端序将 uint32 写入字节缓冲区 */
static void air_comm_write_u32(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
    buffer[2] = (uint8)((value >> 16) & 0xFFU);
    buffer[3] = (uint8)((value >> 24) & 0xFFU);
}

/*
 * 比较参数名和接收到的字节序列是否完全相等。
 * name 是 C 字符串（以 '\0' 结尾），bytes 是不带结尾的字节数组。
 * 长度和内容都必须一致才返回 1。
 */
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

/* 通过 UART_2 发送原始字节，直接调用底层驱动 */
static uint8 air_comm_send_uart(const uint8 *data, uint16 len)
{
    if((data == NULL) || (len == 0U))
    {
        return 0U;
    }

    uart_write_buffer(UART_2, data, len);
    return 1U;
}

/*
 * 组装并发送一帧完整数据。
 * 帧结构：4帧头 + type + seq + len + payload + 2crc
 * CRC 覆盖从帧头到 payload 的全部字节（即 frame[0..pos-1]）。
 * 发送成功后更新统计计数。
 */
static uint8 air_comm_send_frame(uint8 type, uint8 seq, const uint8 *payload, uint8 len)
{
    uint8 frame[AIR_COMM_MAX_FRAME];
    uint16 pos = 0U;
    uint16 crc;

    if((len > AIR_COMM_MAX_PAYLOAD) || ((len > 0U) && (payload == NULL)))
    {
        return 0U;
    }

    /* 组帧头 */
    frame[pos++] = AIR_COMM_HEADER_0;
    frame[pos++] = AIR_COMM_HEADER_1;
    frame[pos++] = AIR_COMM_HEADER_2;
    frame[pos++] = AIR_COMM_HEADER_3;
    frame[pos++] = type;
    frame[pos++] = seq;
    frame[pos++] = len;

    /* 拷贝 payload */
    if(len > 0U)
    {
        memcpy(&frame[pos], payload, len);
        pos = (uint16)(pos + len);
    }

    /* 计算 CRC 并追加到帧尾 */
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

/*
 * 发送心跳包。
 * payload 格式：2 字节保留 + 4 字节当前 tick_ms（小车用来算延迟）。
 * 发送成功才自增 seq。
 */
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

/* 发送参数设置回执，告知小车设置结果和实际写入值 */
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

/* 发送函数调用回执，告知小车调用结果（result 目前保留为 0） */
static uint8 air_comm_send_ack_func(uint8 seq, uint8 func_id, uint8 status, float result)
{
    uint8 payload[6];

    payload[0] = func_id;
    payload[1] = status;
    air_comm_write_float(&payload[2], result);

    return air_comm_send_frame(AIR_COMM_MSG_ACK_FUNC, seq, payload, 6U);
}

/* 收到小车心跳后，刷新在线状态和最后通信时间 */
static void air_comm_mark_car_online(void)
{
    s_air_comm_last_car_ms = s_air_comm_tick_ms;
    s_air_comm_stats.online_status = 1U;
}

/* 按名字在参数表中查找，找不到返回 NULL */
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

/* 按 func_id 在函数表中查找，找不到返回 NULL */
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

/*
 * 处理 SET_PARAM 消息。
 * payload 格式：name_len(1B) + name(name_len B) + value(4B float, 小端)
 * 流程：解析参数名→查表→检查范围→写入变量→发送 ACK。
 * 值超出 [min, max] 时自动限幅到边界，返回 OUT_OF_RANGE。
 */
static void air_comm_handle_set_param(uint8 seq, const uint8 *payload, uint8 len)
{
    uint8 name_len;
    const uint8 *name;
    float value;
    float actual = 0.0f;
    uint8 status = AIR_COMM_AIR_STATUS_ERROR;
    air_comm_air_param_t *param = NULL;

    /* payload 长度不够至少 5 字节（name_len + name至少0 + 4字节float），直接报错 */
    if((payload == NULL) || (len < 5U))
    {
        s_air_comm_stats.set_param_fail_count++;
        (void)air_comm_send_ack_param(seq, status, NULL, 0U, actual);
        return;
    }

    /* 解析参数名长度和指针，检查合法性 */
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

    /* 读取要设置的值，按名字查找参数，然后检查范围并写入 */
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

/*
 * 处理 EXEC_FUNC 消息。
 * payload 格式：func_id(1B)
 * 按 func_id 查表，找到就直接调用，然后发 ACK。
 * 注意：被调用的函数在中断/主循环上下文执行，不能阻塞。
 */
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

/* 帧分发：根据消息类型调用对应的处理函数 */
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

/*
 * 收到完整帧后，重新组装帧数据并校验 CRC。
 * CRC 正确则更新统计并分发处理；CRC 错误则累计错误计数并丢弃。
 * 为什么不直接用解析器算好的 CRC？因为要覆盖帧头字节一起算，解析器阶段没存完整帧。
 */
static void air_comm_process_rx_frame(void)
{
    uint8 frame[AIR_COMM_MAX_FRAME];
    uint16 pos = 0U;
    uint16 crc_calc;

    /* 重新组装帧（帧头 + info + payload），然后算 CRC 比对 */
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

/* 重置接收解析器到初始状态，准备接收下一帧 */
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

/*
 * 逐字节接收状态机。
 * 每收到一个字节推进一次状态：
 *   state 0: 匹配 4 字节帧头，任何不匹配都回到等待第一个 0xAA
 *   state 1: 读 type、seq、len 三个字段；len 超限直接重置
 *   state 2: 收 payload 字节，收够 len 个后转到收 CRC
 *   state 3: 收 2 字节 CRC，收完后调用 process_rx_frame 处理
 * 异常字节会导致状态回退到 state 0 重新找帧头。
 */
static void air_comm_rx_byte_parser(uint8 byte)
{
    switch(s_air_comm_rx.state)
    {
        /* state 0: 逐字节匹配帧头 0xAA 0xAA 0x55 0x55 */
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
                /* 不匹配时，如果当前字节是 0xAA 则从头开始匹配，否则归零 */
                s_air_comm_rx.header_count = (byte == AIR_COMM_HEADER_0) ? 1U : 0U;
            }
            break;

        /* state 1: 读取 type、seq、len 三个 info 字段 */
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
                /* len 字段：payload 长度，超过最大值说明帧有问题，丢弃 */
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
                /* len=0 的帧没有 payload，直接跳到收 CRC */
                s_air_comm_rx.state = (byte == 0U) ? 3U : 2U;
            }
            break;

        /* state 2: 收 payload 字节 */
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

        /* state 3: 收 2 字节 CRC（低字节在前），收完后校验并处理 */
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

/* 从环形队列弹出一个字节，队列空返回 0 */
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

/*
 * 在线超时检测。
 * 如果之前标记过在线（online_status=1），且距离上次收到小车心跳超过 600ms，
 * 则标记为离线（online_status=2）。
 * 注意：online_status=0 表示从未连接过，不会被这个函数改成 2。
 */
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
    /* 清零所有状态和表 */
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

    /* 重置可远程调参的默认值 */
    air_min_area = 5.0f;
    air_hold_ms = 30.0f;
    air_x_bias = 0.0f;
    air_y_bias = 0.0f;

    /* 注册 4 个可被小车远程设置的参数 */
    (void)air_comm_air_register_param("air_min_area", &air_min_area, 0.0f, 500.0f);
    (void)air_comm_air_register_param("air_hold_ms", &air_hold_ms, 0.0f, 200.0f);
    (void)air_comm_air_register_param("air_x_bias", &air_x_bias, -40.0f, 40.0f);
    (void)air_comm_air_register_param("air_y_bias", &air_y_bias, -40.0f, 40.0f);

    /* 配置 UART_2：波特率 1152000，TX=P10_1，RX=P10_0，开接收中断 */
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

/*
 * 主循环轮询入口。
 * 从环形队列取出字节喂给状态机解析器，每轮最多处理队列大小个字节（防死循环）。
 * 解析出完整帧后会触发 CRC 校验和消息分发。
 * 处理完字节后检查小车在线超时。
 */
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

    /* 每 200ms 发一次心跳，用差值判断避免 tick 溢出问题 */
    if((s_air_comm_tick_ms - s_air_comm_last_heartbeat_ms) >= AIR_COMM_HEARTBEAT_MS)
    {
        air_comm_send_heartbeat();
        s_air_comm_last_heartbeat_ms = s_air_comm_tick_ms;
    }

    air_comm_task_online();
}

/*
 * UART 中断回调：收到一个字节就入队。
 * 只做入队操作，不解析、不阻塞，中断安全。
 * 队列满了就丢弃新字节并累计溢出计数。
 */
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

    /* 队列满：head 追上 tail，丢弃这个字节 */
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

/*
 * 注册可远程设置的参数。
 * 同名参数会更新绑定的变量和范围（不会重复占位）。
 * 名字长度超过 16 字节（不含 '\0'）会失败。
 */
uint8 air_comm_air_register_param(const char *name, float *var, float min, float max)
{
    uint8 i;

    if((name == NULL) || (var == NULL) || (min > max))
    {
        return 0U;
    }

    /* 同名参数更新，不重复占位 */
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

/*
 * 注册可远程调用的函数。
 * 同 func_id 会更新函数指针（不重复占位）。
 */
uint8 air_comm_air_register_func(uint8 func_id, void (*func)(void))
{
    uint8 i;

    if(func == NULL)
    {
        return 0U;
    }

    /* 同 func_id 更新，不重复占位 */
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

/*
 * 向小车上报运行数据。
 * payload 格式：count(1B) + float0(4B) + float1(4B) + ...
 * 最多 32 个 float，每个 4 字节，小端序。
 * 发送成功才自增 seq。
 */
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

    /* 第一个字节是 float 个数，后面逐个写入 */
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

/* 获取统计快照，把当前 tick_ms 也写进去 */
void air_comm_air_get_stats(air_comm_air_stats_t *stats)
{
    if(stats == NULL)
    {
        return;
    }

    s_air_comm_stats.tick_ms = s_air_comm_tick_ms;
    *stats = s_air_comm_stats;
}
