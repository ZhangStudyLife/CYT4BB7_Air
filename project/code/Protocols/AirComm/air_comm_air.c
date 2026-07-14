#include "air_comm_air.h"

#include "FlightController/fc_loop.h"
#include "FlightController/fc_params.h"
#include "FlightController/fc_start_crsf.h"

#include <math.h>

#define AIR_COMM_MSG_GET_PARAM               (0x07U)
#define AIR_COMM_MSG_ACK_GET_PARAM           (0x08U)

/* 帧头 4 字节：0xAA 0xAA 0x55 0x55，选这个是为了在串口噪声中容易识别 */
#define AIR_COMM_HEADER_0                    (0xAAU)
#define AIR_COMM_HEADER_1                    (0xAAU)
#define AIR_COMM_HEADER_2                    (0x55U)
#define AIR_COMM_HEADER_3                    (0x55U)

#define AIR_COMM_MAX_PAYLOAD                 (250U)  /* 单帧 payload 最大字节数 */
#define AIR_COMM_FRAME_OVERHEAD              (9U)    /* 帧开销：4帧头 + type + seq + len + 2crc */
#define AIR_COMM_MAX_FRAME                   (AIR_COMM_MAX_PAYLOAD + AIR_COMM_FRAME_OVERHEAD)
#define AIR_COMM_RX_QUEUE_SIZE               (512U)  /* 接收环形队列大小（字节） */
#define AIR_COMM_PARAM_TABLE_MAX             (128U)  /* 最多注册参数个数 */
#define AIR_COMM_DEFAULT_PARAM_COUNT         (128U)
#define AIR_COMM_REMOTE_CANCEL_MS            (400U)
#define AIR_COMM_REMOTE_TIMEOUT_MS           (700U)
#define AIR_COMM_REMOTE_EXP_CANCEL_MS        (1800U)
#define AIR_COMM_REMOTE_EXP_TIMEOUT_MS       (2600U)
#define AIR_COMM_PARAM_ACK_CACHE_MS          (1000U)
#define AIR_COMM_PARAM_TARGET_LOCAL          (0U)
#define AIR_COMM_COMMAND_TABLE_MAX           (8U)    /* 最多注册远程命令个数 */

typedef char air_comm_default_param_count_must_fit_table[
    (AIR_COMM_DEFAULT_PARAM_COUNT <= AIR_COMM_PARAM_TABLE_MAX) ? 1 : -1];

/* 消息类型定义 */
#define AIR_COMM_MSG_SET_PARAM               (0x01U) /* 小车→无人机：设置参数 */
#define AIR_COMM_MSG_ACK_PARAM               (0x02U) /* 无人机→小车：参数设置回执 */
#define AIR_COMM_MSG_EXEC_COMMAND            (0x03U) /* 小车→无人机：执行远程命令 */
#define AIR_COMM_MSG_ACK_COMMAND             (0x04U) /* 无人机→小车：远程命令回执 */
#define AIR_COMM_MSG_HEARTBEAT               (0x05U) /* 双向：心跳 */
#define AIR_COMM_MSG_RUN_DATA                (0x06U) /* 双向：实时数据 */

#define AIR_COMM_HEARTBEAT_MS                (200U)  /* 心跳发送间隔（ms） */
#define AIR_COMM_OFFLINE_MS                  (600U)  /* 超过此时间没收到心跳判定离线 */

/* 参数表条目：名字、绑定变量指针、允许范围 */
typedef struct
{
    const char *name;
    void *var;
    uint8 type;
    uint8 target;
    uint16 param_id;
    float min;
    float max;
} air_comm_air_param_t;

typedef struct
{
    uint8 active;
    uint8 message_type;
    uint8 seq;
    uint8 name_len;
    uint8 requested_status;
    uint8 op;
    uint8 cancel_requested;
    uint32 request_value_bits;
    uint32 expected_value_bits;
    uint32 transaction;
    uint32 start_tick_ms;
    air_comm_air_param_t *param;
    char name[AIR_COMM_AIR_PARAM_NAME_MAX + 1U];
} air_comm_remote_param_pending_t;

typedef struct
{
    uint8 valid;
    uint8 message_type;
    uint8 seq;
    uint8 name_len;
    uint8 status;
    uint32 request_value_bits;
    uint32 completed_tick_ms;
    float actual;
    char name[AIR_COMM_AIR_PARAM_NAME_MAX + 1U];
} air_comm_param_ack_cache_t;

typedef struct
{
    const char *name;
    air_comm_air_command_mode_t mode;
    air_comm_air_command_fn run;
} air_comm_air_command_t;

#define AIR_COMM_REGISTER_FLOAT(name, variable, min_v, max_v)                              \
    do                                                                                     \
    {                                                                                      \
        if(air_comm_air_register_param(#name, &(variable),                                 \
                                       AIR_COMM_AIR_PARAM_TYPE_FLOAT,                       \
                                       (min_v), (max_v)) == 0U)                             \
        {                                                                                  \
            s_air_comm_registration_ok = 0U;                                               \
        }                                                                                  \
    } while(0)

#define AIR_COMM_REGISTER_INT32(name, variable, min_v, max_v)                              \
    do                                                                                     \
    {                                                                                      \
        if(air_comm_air_register_param(#name, &(variable),                                 \
                                       AIR_COMM_AIR_PARAM_TYPE_INT32,                       \
                                       (float)(min_v), (float)(max_v)) == 0U)               \
        {                                                                                  \
            s_air_comm_registration_ok = 0U;                                               \
        }                                                                                  \
    } while(0)

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
/* 核1阈值在核0的菜单镜像，仅在下游确认成功后更新。 */
int32 c1_beacon_thr = 120;
/* 两颗2BL3共同阈值在核0的菜单镜像，仅在两板确认一致后更新。 */
int32 bl3_beacon_thr = 120;
/* 核1摄像头曝光时间的核0菜单镜像，仅在下游确认成功后更新。 */
int32 c1_exp_time = 400;
/* 两颗2BL3共同曝光时间的核0菜单镜像，仅在两板确认一致后更新。 */
int32 bl3_exp_time = 500;
/* 核1屏幕显示模式的核0菜单镜像，仅在核1读回成功后更新。 */
int32 c1_screen_mode = 0;

static air_comm_run_data_fn s_air_comm_run_data_callback;
static air_comm_air_command_t s_air_comm_commands[AIR_COMM_COMMAND_TABLE_MAX];
static const air_comm_air_command_t *s_air_comm_active_command;
static float s_air_comm_last_run_data[AIR_COMM_AIR_RUN_DATA_MAX_FLOATS];
static char s_air_comm_last_done_name[AIR_COMM_AIR_COMMAND_NAME_MAX + 1U];
static uint8 s_air_comm_last_run_data_count;
static uint8 s_air_comm_last_run_data_valid;
static uint8 s_air_comm_command_count;
static uint8 s_air_comm_active_seq;
static uint8 s_air_comm_screen_ready;
static uint8 s_air_comm_last_done_valid;
static uint8 s_air_comm_last_done_seq;
static uint16 s_air_comm_param_count = 0U; /* 128项参数不能使用uint8计数。 */
static uint8 s_air_comm_registration_ok;
static air_comm_remote_param_pending_t s_air_comm_remote_pending;
static air_comm_param_ack_cache_t s_air_comm_param_ack_cache;

static uint8 air_comm_register_remote_param(const char *name,
                                            void *var,
                                            uint8 type,
                                            float min,
                                            float max,
                                            uint8 target,
                                            uint16 param_id);

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

static uint8 air_comm_command_name_equal(const char *left, const char *right)
{
    if((left == NULL) || (right == NULL))
    {
        return 0U;
    }

    return (strcmp(left, right) == 0) ? 1U : 0U;
}

static const air_comm_air_command_t *air_comm_find_command(const char *name)
{
    uint8 index;

    if(name == NULL)
    {
        return NULL;
    }

    for(index = 0U; index < s_air_comm_command_count; index++)
    {
        if(air_comm_command_name_equal(s_air_comm_commands[index].name, name) != 0U)
        {
            return &s_air_comm_commands[index];
        }
    }

    return NULL;
}

static void air_comm_screen_reset(void)
{
    s_air_comm_screen_ready = 1U;
}

static void air_comm_screen_stop(void)
{
    s_air_comm_screen_ready = 0U;
}

static void air_comm_stop_active_command(void)
{
    if(s_air_comm_active_command != NULL)
    {
        if(s_air_comm_active_command->mode == AIR_COMM_AIR_COMMAND_MODE_POLLING)
        {
            air_comm_screen_stop();
        }
    }

    s_air_comm_active_command = NULL;
}

static uint8 air_comm_remote_operation_allowed(void)
{
    return ((FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_STANDBY) &&
            (FC_START_CRSF_Is_Armed() == 0U)) ? 1U : 0U;
}

static void air_comm_beep(void)
{
    Beep_Play(50U, 0.2f, 3U);
}

static uint8 air_comm_register_default_commands(void)
{
    return air_comm_air_register_instant_command("beep", air_comm_beep);
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

static uint8 air_comm_send_ack_get_param(uint8 seq,
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

    return air_comm_send_frame(AIR_COMM_MSG_ACK_GET_PARAM, seq, payload, pos);
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
    uint16 i;

    for(i = 0U; i < s_air_comm_param_count; i++)
    {
        if(air_comm_name_equal(s_air_comm_params[i].name, name, name_len) != 0U)
        {
            return &s_air_comm_params[i];
        }
    }

    return NULL;
}

static float air_comm_param_read(const air_comm_air_param_t *param)
{
    if(param == NULL)
    {
        return 0.0f;
    }

    if(param->type == AIR_COMM_AIR_PARAM_TYPE_INT32)
    {
        return (float)(*(const int32_t *)param->var);
    }

    return *(const float *)param->var;
}

static void air_comm_param_write(air_comm_air_param_t *param, float value)
{
    if(param == NULL)
    {
        return;
    }

    if(param->type == AIR_COMM_AIR_PARAM_TYPE_INT32)
    {
        *(int32_t *)param->var = (int32_t)value;
    }
    else
    {
        *(float *)param->var = value;
    }
}

/* 将参数值转换为跨核协议的32位位模式。 */
static uint32 air_comm_param_value_bits(const air_comm_air_param_t *param, float value)
{
    uint32 bits;

    if((param != NULL) && (param->type == AIR_COMM_AIR_PARAM_TYPE_INT32))
    {
        return (uint32)(int32)value;
    }
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

/* 将目标端读回位模式转换为空地协议使用的float数值。 */
static float air_comm_param_bits_value(const air_comm_air_param_t *param, uint32 bits)
{
    float value;

    if((param != NULL) && (param->type == AIR_COMM_AIR_PARAM_TYPE_INT32))
    {
        return (float)(int32)bits;
    }
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/* 判断收到的参数帧是否为最近一笔已完成帧的UART重传。 */
static uint8 air_comm_param_cache_match(uint8 message_type,
                                        uint8 seq,
                                        const uint8 *name,
                                        uint8 name_len,
                                        uint32 request_value_bits)
{
    if((s_air_comm_param_ack_cache.valid == 0U) ||
       ((s_air_comm_tick_ms - s_air_comm_param_ack_cache.completed_tick_ms) >
        AIR_COMM_PARAM_ACK_CACHE_MS) ||
       (s_air_comm_param_ack_cache.message_type != message_type) ||
       (s_air_comm_param_ack_cache.seq != seq) ||
       (s_air_comm_param_ack_cache.name_len != name_len) ||
       (s_air_comm_param_ack_cache.request_value_bits != request_value_bits))
    {
        return 0U;
    }

    return (memcmp(s_air_comm_param_ack_cache.name, name, name_len) == 0) ? 1U : 0U;
}

/* 重新发送缓存ACK，避免同序号重传重复修改下游参数。 */
static void air_comm_param_cache_send(void)
{
    if(s_air_comm_param_ack_cache.message_type == AIR_COMM_MSG_SET_PARAM)
    {
        (void)air_comm_send_ack_param(s_air_comm_param_ack_cache.seq,
                                      s_air_comm_param_ack_cache.status,
                                      (const uint8 *)s_air_comm_param_ack_cache.name,
                                      s_air_comm_param_ack_cache.name_len,
                                      s_air_comm_param_ack_cache.actual);
    }
    else
    {
        (void)air_comm_send_ack_get_param(s_air_comm_param_ack_cache.seq,
                                          s_air_comm_param_ack_cache.status,
                                          (const uint8 *)s_air_comm_param_ack_cache.name,
                                          s_air_comm_param_ack_cache.name_len,
                                          s_air_comm_param_ack_cache.actual);
    }
}

/* 保存最近一笔最终ACK，缓存窗口内相同UART帧直接复用。 */
static void air_comm_param_cache_store(uint8 message_type,
                                       uint8 seq,
                                       const uint8 *name,
                                       uint8 name_len,
                                       uint32 request_value_bits,
                                       uint8 status,
                                       float actual)
{
    memset(&s_air_comm_param_ack_cache, 0, sizeof(s_air_comm_param_ack_cache));
    s_air_comm_param_ack_cache.valid = 1U;
    s_air_comm_param_ack_cache.message_type = message_type;
    s_air_comm_param_ack_cache.seq = seq;
    s_air_comm_param_ack_cache.name_len = name_len;
    s_air_comm_param_ack_cache.status = status;
    s_air_comm_param_ack_cache.request_value_bits = request_value_bits;
    s_air_comm_param_ack_cache.completed_tick_ms = s_air_comm_tick_ms;
    s_air_comm_param_ack_cache.actual = actual;
    memcpy(s_air_comm_param_ack_cache.name, name, name_len);
    s_air_comm_param_ack_cache.name[name_len] = '\0';
}

/* 判断UART重传是否对应当前仍在执行的远端事务。 */
static uint8 air_comm_remote_pending_match(uint8 message_type,
                                           uint8 seq,
                                           const uint8 *name,
                                           uint8 name_len,
                                           uint32 request_value_bits)
{
    if((s_air_comm_remote_pending.active == 0U) ||
       (s_air_comm_remote_pending.message_type != message_type) ||
       (s_air_comm_remote_pending.seq != seq) ||
       (s_air_comm_remote_pending.name_len != name_len) ||
       (s_air_comm_remote_pending.request_value_bits != request_value_bits))
    {
        return 0U;
    }

    return (memcmp(s_air_comm_remote_pending.name, name, name_len) == 0) ? 1U : 0U;
}

/* 启动核0到核1的异步请求，最终ACK由高频轮询收到目标读回值后发送。 */
static uint8 air_comm_remote_start(uint8 message_type,
                                   uint8 seq,
                                   const uint8 *name,
                                   uint8 name_len,
                                   air_comm_air_param_t *param,
                                   uint8 op,
                                   float value,
                                   uint8 requested_status,
                                   uint32 request_value_bits)
{
    uint32 transaction;
    uint32 value_bits = air_comm_param_value_bits(param, value);
    uint32 previous_bits = air_comm_param_value_bits(param, air_comm_param_read(param));

    if((param == NULL) || (s_air_comm_remote_pending.active != 0U) ||
       (ipc_remote_param_core0_start(param->target,
                                     op,
                                     param->type,
                                     param->param_id,
                                     value_bits,
                                     previous_bits,
                                     &transaction) == 0U))
    {
        return 0U;
    }

    memset(&s_air_comm_remote_pending, 0, sizeof(s_air_comm_remote_pending));
    s_air_comm_remote_pending.active = 1U;
    s_air_comm_remote_pending.message_type = message_type;
    s_air_comm_remote_pending.seq = seq;
    s_air_comm_remote_pending.name_len = name_len;
    s_air_comm_remote_pending.requested_status = requested_status;
    s_air_comm_remote_pending.op = op;
    s_air_comm_remote_pending.request_value_bits = request_value_bits;
    s_air_comm_remote_pending.expected_value_bits = value_bits;
    s_air_comm_remote_pending.transaction = transaction;
    s_air_comm_remote_pending.start_tick_ms = s_air_comm_tick_ms;
    s_air_comm_remote_pending.param = param;
    memcpy(s_air_comm_remote_pending.name, name, name_len);
    s_air_comm_remote_pending.name[name_len] = '\0';
    return 1U;
}

/* 高频轮询远端事务；仅目标端最终读回一致后更新代理变量并回复车端。 */
static void air_comm_remote_poll(void)
{
    ipc_remote_param_mailbox_t response;
    air_comm_remote_param_pending_t pending;
    float actual;
    uint8 status;
    uint32 cancel_ms;
    uint32 timeout_ms;

    if(s_air_comm_remote_pending.active == 0U)
    {
        return;
    }

    pending = s_air_comm_remote_pending;
    cancel_ms = (pending.param->param_id == IPC_REMOTE_PARAM_ID_EXP_TIME) ?
                AIR_COMM_REMOTE_EXP_CANCEL_MS : AIR_COMM_REMOTE_CANCEL_MS;
    timeout_ms = (pending.param->param_id == IPC_REMOTE_PARAM_ID_EXP_TIME) ?
                 AIR_COMM_REMOTE_EXP_TIMEOUT_MS : AIR_COMM_REMOTE_TIMEOUT_MS;
    if(ipc_remote_param_core0_poll(&response) != 0U)
    {
        status = response.status;
        actual = air_comm_param_read(pending.param);
        if(status > AIR_COMM_AIR_STATUS_ROLLBACK_FAIL)
        {
            status = AIR_COMM_AIR_STATUS_ERROR;
        }
        if((response.transaction != pending.transaction) ||
           (response.op != pending.op) ||
           (response.target != pending.param->target) ||
           (response.type != pending.param->type) ||
           (response.param_id != pending.param->param_id))
        {
            status = AIR_COMM_AIR_STATUS_ERROR;
        }
        else if(status == AIR_COMM_AIR_STATUS_OUT_OF_RANGE)
        {
            /* 核0已完成限幅，下游再次报越界说明固件约束不一致。 */
            status = AIR_COMM_AIR_STATUS_MISMATCH;
        }
        else if((status == AIR_COMM_AIR_STATUS_OK) &&
                (pending.op == IPC_REMOTE_PARAM_OP_SET) &&
                (response.value_bits != pending.expected_value_bits))
        {
            status = AIR_COMM_AIR_STATUS_MISMATCH;
        }
        else if(status == AIR_COMM_AIR_STATUS_OK)
        {
            actual = air_comm_param_bits_value(pending.param, response.value_bits);
            air_comm_param_write(pending.param, actual);
            if(pending.requested_status == AIR_COMM_AIR_STATUS_OUT_OF_RANGE)
            {
                status = AIR_COMM_AIR_STATUS_OUT_OF_RANGE;
            }
        }
    }
    else if((pending.cancel_requested == 0U) &&
            (((pending.op == IPC_REMOTE_PARAM_OP_SET) &&
              (air_comm_remote_operation_allowed() == 0U)) ||
             ((s_air_comm_tick_ms - pending.start_tick_ms) >= cancel_ms)))
    {
        if(ipc_remote_param_core0_request_cancel(pending.transaction) != 0U)
        {
            s_air_comm_remote_pending.cancel_requested = 1U;
        }
        return;
    }
    else if((s_air_comm_tick_ms - pending.start_tick_ms) >= timeout_ms)
    {
        ipc_remote_param_core0_cancel(pending.transaction);
        status = AIR_COMM_AIR_STATUS_TIMEOUT;
        actual = air_comm_param_read(pending.param);
    }
    else
    {
        return;
    }

    s_air_comm_remote_pending.active = 0U;
    air_comm_param_cache_store(pending.message_type,
                               pending.seq,
                               (const uint8 *)pending.name,
                               pending.name_len,
                               pending.request_value_bits,
                               status,
                               actual);
    air_comm_param_cache_send();
    if(pending.message_type == AIR_COMM_MSG_SET_PARAM)
    {
        if(status == AIR_COMM_AIR_STATUS_OK)
        {
            s_air_comm_stats.set_param_ok_count++;
        }
        else
        {
            s_air_comm_stats.set_param_fail_count++;
        }
    }
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
    uint32 request_value_bits = 0U;
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
    memcpy(&request_value_bits, &payload[1U + name_len], sizeof(request_value_bits));
    if(air_comm_param_cache_match(AIR_COMM_MSG_SET_PARAM,
                                  seq,
                                  name,
                                  name_len,
                                  request_value_bits) != 0U)
    {
        air_comm_param_cache_send();
        return;
    }
    if(air_comm_remote_pending_match(AIR_COMM_MSG_SET_PARAM,
                                     seq,
                                     name,
                                     name_len,
                                     request_value_bits) != 0U)
    {
        return;
    }
    if(s_air_comm_remote_pending.active != 0U)
    {
        s_air_comm_stats.set_param_fail_count++;
        (void)air_comm_send_ack_param(seq,
                                      AIR_COMM_AIR_STATUS_BUSY,
                                      name,
                                      name_len,
                                      0.0f);
        return;
    }
    param = air_comm_find_param(name, name_len);
    if(param == NULL)
    {
        status = AIR_COMM_AIR_STATUS_NOT_FOUND;
    }
    else if((isnan(value) != 0) ||
            (isinf(value) != 0) ||
            (air_comm_remote_operation_allowed() == 0U))
    {
        actual = air_comm_param_read(param);
        status = AIR_COMM_AIR_STATUS_ERROR;
    }
    else if(value < param->min)
    {
        actual = param->min;
        if(param->target == AIR_COMM_PARAM_TARGET_LOCAL)
        {
            air_comm_param_write(param, actual);
        }
        status = AIR_COMM_AIR_STATUS_OUT_OF_RANGE;
    }
    else if(value > param->max)
    {
        actual = param->max;
        if(param->target == AIR_COMM_PARAM_TARGET_LOCAL)
        {
            air_comm_param_write(param, actual);
        }
        status = AIR_COMM_AIR_STATUS_OUT_OF_RANGE;
    }
    else
    {
        actual = value;
        if(param->target == AIR_COMM_PARAM_TARGET_LOCAL)
        {
            air_comm_param_write(param, actual);
        }
        status = AIR_COMM_AIR_STATUS_OK;
    }

    if((param != NULL) &&
       (status != AIR_COMM_AIR_STATUS_ERROR))
    {
        if(param->target != AIR_COMM_PARAM_TARGET_LOCAL)
        {
            if(air_comm_remote_start(AIR_COMM_MSG_SET_PARAM,
                                     seq,
                                     name,
                                     name_len,
                                     param,
                                     IPC_REMOTE_PARAM_OP_SET,
                                     actual,
                                     status,
                                     request_value_bits) != 0U)
            {
                return;
            }
            status = AIR_COMM_AIR_STATUS_BUSY;
            actual = air_comm_param_read(param);
        }
        else
        {
            actual = air_comm_param_read(param);
            FC_Loop_Init();
        }
    }

    if(status == AIR_COMM_AIR_STATUS_OK)
    {
        s_air_comm_stats.set_param_ok_count++;
    }
    else
    {
        s_air_comm_stats.set_param_fail_count++;
    }

    air_comm_param_cache_store(AIR_COMM_MSG_SET_PARAM,
                               seq,
                               name,
                               name_len,
                               request_value_bits,
                               status,
                               actual);
    air_comm_param_cache_send();
}

static void air_comm_handle_get_param(uint8 seq, const uint8 *payload, uint8 len)
{
    uint8 name_len;
    const uint8 *name;
    float actual = 0.0f;
    uint8 status = AIR_COMM_AIR_STATUS_ERROR;
    air_comm_air_param_t *param = NULL;

    if((payload == NULL) || (len < 1U))
    {
        (void)air_comm_send_ack_get_param(seq, status, NULL, 0U, actual);
        return;
    }

    name_len = payload[0];
    name = &payload[1];
    if((name_len == 0U) ||
       (name_len > AIR_COMM_AIR_PARAM_NAME_MAX) ||
       (len < (uint8)(1U + name_len)))
    {
        (void)air_comm_send_ack_get_param(seq, status, NULL, 0U, actual);
        return;
    }

    if(air_comm_param_cache_match(AIR_COMM_MSG_GET_PARAM,
                                  seq,
                                  name,
                                  name_len,
                                  0U) != 0U)
    {
        air_comm_param_cache_send();
        return;
    }
    if(air_comm_remote_pending_match(AIR_COMM_MSG_GET_PARAM,
                                     seq,
                                     name,
                                     name_len,
                                     0U) != 0U)
    {
        return;
    }
    if(s_air_comm_remote_pending.active != 0U)
    {
        (void)air_comm_send_ack_get_param(seq,
                                          AIR_COMM_AIR_STATUS_BUSY,
                                          name,
                                          name_len,
                                          0.0f);
        return;
    }

    param = air_comm_find_param(name, name_len);
    if(param == NULL)
    {
        status = AIR_COMM_AIR_STATUS_NOT_FOUND;
    }
    else if(param->target != AIR_COMM_PARAM_TARGET_LOCAL)
    {
        actual = air_comm_param_read(param);
        if(air_comm_remote_start(AIR_COMM_MSG_GET_PARAM,
                                 seq,
                                 name,
                                 name_len,
                                 param,
                                 IPC_REMOTE_PARAM_OP_GET,
                                 actual,
                                 AIR_COMM_AIR_STATUS_OK,
                                 0U) != 0U)
        {
            return;
        }
        status = AIR_COMM_AIR_STATUS_BUSY;
    }
    else
    {
        actual = air_comm_param_read(param);
        status = AIR_COMM_AIR_STATUS_OK;
    }

    air_comm_param_cache_store(AIR_COMM_MSG_GET_PARAM,
                               seq,
                               name,
                               name_len,
                               0U,
                               status,
                               actual);
    air_comm_param_cache_send();
}

/*
 * 处理 EXEC_COMMAND 消息。
 * payload 格式：[name_len][name...]，按命令名查表并返回文本 ACK。
 * 注意：命令入口在主循环上下文执行，不能长时间阻塞。
 */
static void air_comm_handle_exec_command(uint8 seq, const uint8 *payload, uint8 len)
{
    uint8 name_len;
    const air_comm_air_command_t *command;
    char name[AIR_COMM_AIR_COMMAND_NAME_MAX + 1U];
    char ack[AIR_COMM_AIR_ACK_TEXT_MAX + 1U];

    if((payload == NULL) || (len < 1U))
    {
        s_air_comm_stats.command_fail_count++;
        (void)air_comm_air_send_command_ack_text(seq, "ACK_ERROR 3 bad_payload");
        return;
    }

    name_len = payload[0];
    if((name_len == 0U) ||
       (name_len > AIR_COMM_AIR_COMMAND_NAME_MAX) ||
       ((uint16)len < (uint16)(1U + name_len)))
    {
        s_air_comm_stats.command_fail_count++;
        (void)air_comm_air_send_command_ack_text(seq, "ACK_ERROR 3 bad_name");
        return;
    }

    memcpy(name, &payload[1], name_len);
    name[name_len] = '\0';

    if(air_comm_command_name_equal(name, "NONE") != 0U)
    {
        air_comm_stop_active_command();
        (void)air_comm_air_send_command_ack_text(seq, "ACK_EXIT_OK");
        s_air_comm_stats.command_ok_count++;
        return;
    }

    if((s_air_comm_last_done_valid != 0U) &&
       (seq == s_air_comm_last_done_seq) &&
       (air_comm_command_name_equal(s_air_comm_last_done_name, name) != 0U))
    {
        (void)air_comm_air_send_command_ack_text(seq, "ACK_EXIT_OK");
        s_air_comm_stats.command_ok_count++;
        return;
    }

    if(air_comm_remote_operation_allowed() == 0U)
    {
        air_comm_stop_active_command();
        (void)air_comm_air_send_command_ack_text(seq, "ACK_ERROR 3 state");
        s_air_comm_stats.command_fail_count++;
        return;
    }

    if(s_air_comm_active_command != NULL)
    {
        if((seq == s_air_comm_active_seq) &&
           (air_comm_command_name_equal(s_air_comm_active_command->name, name) != 0U))
        {
            snprintf(ack, sizeof(ack), "ACK_OK %s", s_air_comm_active_command->name);
            (void)air_comm_air_send_command_ack_text(seq, ack);
            s_air_comm_stats.command_ok_count++;
            return;
        }

        (void)air_comm_air_send_command_ack_text(seq, "ACK_ERROR 4 busy");
        s_air_comm_stats.command_fail_count++;
        return;
    }

    command = air_comm_find_command(name);
    if(command == NULL)
    {
        (void)air_comm_air_send_command_ack_text(seq, "ACK_ERROR 1 not_found");
        s_air_comm_stats.command_fail_count++;
        return;
    }

    s_air_comm_active_command = command;
    s_air_comm_active_seq = seq;
    s_air_comm_last_done_valid = 0U;

    if(command->run == NULL)
    {
        s_air_comm_active_command = NULL;
        (void)air_comm_air_send_command_ack_text(seq, "ACK_ERROR 3 bad_command");
        s_air_comm_stats.command_fail_count++;
        return;
    }

    if(command->mode == AIR_COMM_AIR_COMMAND_MODE_POLLING)
    {
        air_comm_screen_reset();
    }

    snprintf(ack, sizeof(ack), "ACK_OK %s", command->name);
    (void)air_comm_air_send_command_ack_text(seq, ack);
    s_air_comm_stats.command_ok_count++;

    /* 立即型命令也交给 100Hz 调度执行，避免在串口帧解析过程中运行用户代码。 */
}

static void air_comm_handle_run_data(const uint8 *payload, uint8 len)
{
    uint8 count;
    uint16 required_len;
    uint8 index;
    float data[AIR_COMM_AIR_RUN_DATA_MAX_FLOATS];

    if((payload == NULL) || (len < 1U))
    {
        return;
    }

    count = payload[0];
    if(count > AIR_COMM_AIR_RUN_DATA_MAX_FLOATS)
    {
        return;
    }

    required_len = (uint16)(1U + ((uint16)count * 4U));
    if((uint16)len < required_len)
    {
        return;
    }

    for(index = 0U; index < count; index++)
    {
        data[index] = air_comm_read_float(&payload[1U + ((uint16)index * 4U)]);
    }

    if(count > 0U)
    {
        memcpy(s_air_comm_last_run_data, data, (size_t)count * sizeof(float));
    }
    s_air_comm_last_run_data_count = count;
    s_air_comm_last_run_data_valid = 1U;

    if(s_air_comm_run_data_callback != NULL)
    {
        s_air_comm_run_data_callback(data, count);
    }
}

/* 帧分发：根据消息类型调用对应的处理函数 */
static void air_comm_handle_frame(uint8 type, uint8 seq, const uint8 *payload, uint8 len)
{
    switch(type)
    {
        case AIR_COMM_MSG_SET_PARAM:
            air_comm_handle_set_param(seq, payload, len);
            break;

        case AIR_COMM_MSG_EXEC_COMMAND:
            air_comm_handle_exec_command(seq, payload, len);
            break;

        case AIR_COMM_MSG_GET_PARAM:
            air_comm_handle_get_param(seq, payload, len);
            break;

        case AIR_COMM_MSG_HEARTBEAT:
            s_air_comm_stats.heartbeat_rx_count++;
            air_comm_mark_car_online();
            break;

        case AIR_COMM_MSG_RUN_DATA:
            air_comm_handle_run_data(payload, len);
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
    air_comm_mark_car_online();
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
    memset(s_air_comm_commands, 0, sizeof(s_air_comm_commands));
    memset(s_air_comm_last_run_data, 0, sizeof(s_air_comm_last_run_data));
    memset(s_air_comm_last_done_name, 0, sizeof(s_air_comm_last_done_name));
    memset(&s_air_comm_remote_pending, 0, sizeof(s_air_comm_remote_pending));
    memset(&s_air_comm_param_ack_cache, 0, sizeof(s_air_comm_param_ack_cache));

    s_air_comm_initialized = 0U;
    s_air_comm_seq = 0U;
    s_air_comm_tick_ms = 0U;
    s_air_comm_last_heartbeat_ms = 0U;
    s_air_comm_last_car_ms = 0U;
    s_air_comm_run_data_callback = NULL;
    s_air_comm_active_command = NULL;
    s_air_comm_last_run_data_count = 0U;
    s_air_comm_last_run_data_valid = 0U;
    s_air_comm_param_count = 0U;
    s_air_comm_command_count = 0U;
    s_air_comm_active_seq = 0U;
    s_air_comm_screen_ready = 0U;
    s_air_comm_last_done_valid = 0U;
    s_air_comm_last_done_seq = 0U;
    s_air_comm_registration_ok = 1U;
    c1_beacon_thr = 120;
    bl3_beacon_thr = 120;
    c1_exp_time = 400;
    bl3_exp_time = 500;
    c1_screen_mode = 0;

    AIR_COMM_REGISTER_FLOAT(gyro_dt, g_fc_params.gyro_dt, 0.0001f, 0.1f);
    AIR_COMM_REGISTER_FLOAT(angle_dt, g_fc_params.angle_dt, 0.0001f, 0.1f);
    AIR_COMM_REGISTER_FLOAT(pos_z_dt, g_fc_params.pos_z_dt, 0.0001f, 0.2f);
    AIR_COMM_REGISTER_FLOAT(vel_xy_dt, g_fc_params.vel_xy_dt, 0.0001f, 0.2f);
    AIR_COMM_REGISTER_FLOAT(vel_z_dt, g_fc_params.vel_z_dt, 0.0001f, 0.2f);
    AIR_COMM_REGISTER_INT32(base_throttle, g_fc_params.base_throttle, 0, 6000);
    AIR_COMM_REGISTER_FLOAT(roll_mech_trim_deg, g_fc_params.roll_mech_trim_deg, -30.0f, 30.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_mech_trim_deg, g_fc_params.pitch_mech_trim_deg, -30.0f, 30.0f);

    AIR_COMM_REGISTER_FLOAT(roll_gyro_kp, g_fc_params.roll_gyro_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(roll_gyro_ki, g_fc_params.roll_gyro_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(roll_gyro_kd, g_fc_params.roll_gyro_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(roll_gyro_kff, g_fc_params.roll_gyro_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(roll_gyro_i_limit, g_fc_params.roll_gyro_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(roll_gyro_d_lpf, g_fc_params.roll_gyro_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_gyro_kp, g_fc_params.pitch_gyro_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_gyro_ki, g_fc_params.pitch_gyro_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_gyro_kd, g_fc_params.pitch_gyro_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_gyro_kff, g_fc_params.pitch_gyro_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_gyro_i_limit, g_fc_params.pitch_gyro_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_gyro_d_lpf, g_fc_params.pitch_gyro_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(yaw_gyro_kp, g_fc_params.yaw_gyro_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(yaw_gyro_ki, g_fc_params.yaw_gyro_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(yaw_gyro_kd, g_fc_params.yaw_gyro_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(yaw_gyro_kff, g_fc_params.yaw_gyro_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(yaw_gyro_i_limit, g_fc_params.yaw_gyro_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(yaw_gyro_d_lpf, g_fc_params.yaw_gyro_d_lpf, 0.0f, 500.0f);

    AIR_COMM_REGISTER_FLOAT(roll_angle_kp, g_fc_params.roll_angle_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(roll_angle_ki, g_fc_params.roll_angle_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(roll_angle_kd, g_fc_params.roll_angle_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(roll_angle_kff, g_fc_params.roll_angle_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(roll_angle_i_limit, g_fc_params.roll_angle_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(roll_angle_d_lpf, g_fc_params.roll_angle_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_angle_kp, g_fc_params.pitch_angle_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_angle_ki, g_fc_params.pitch_angle_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_angle_kd, g_fc_params.pitch_angle_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_angle_kff, g_fc_params.pitch_angle_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_angle_i_limit, g_fc_params.pitch_angle_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(pitch_angle_d_lpf, g_fc_params.pitch_angle_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(yaw_angle_kp, g_fc_params.yaw_angle_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(yaw_angle_ki, g_fc_params.yaw_angle_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(yaw_angle_kd, g_fc_params.yaw_angle_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(yaw_angle_kff, g_fc_params.yaw_angle_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(yaw_angle_i_limit, g_fc_params.yaw_angle_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(yaw_angle_d_lpf, g_fc_params.yaw_angle_d_lpf, 0.0f, 500.0f);

    AIR_COMM_REGISTER_FLOAT(vel_x_kp, g_fc_params.vel_x_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(vel_x_ki, g_fc_params.vel_x_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(vel_x_kd, g_fc_params.vel_x_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(vel_x_kff, g_fc_params.vel_x_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(vel_x_i_limit, g_fc_params.vel_x_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(vel_x_d_lpf, g_fc_params.vel_x_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(vel_y_kp, g_fc_params.vel_y_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(vel_y_ki, g_fc_params.vel_y_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(vel_y_kd, g_fc_params.vel_y_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(vel_y_kff, g_fc_params.vel_y_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(vel_y_i_limit, g_fc_params.vel_y_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(vel_y_d_lpf, g_fc_params.vel_y_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(vel_z_ki, g_fc_params.vel_z_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(vel_z_i_limit, g_fc_params.vel_z_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(mode7_vel_x_kp, g_fc_params.mode7_vel_x_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode7_vel_x_ki, g_fc_params.mode7_vel_x_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode7_vel_x_kd, g_fc_params.mode7_vel_x_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode7_vel_x_kff, g_fc_params.mode7_vel_x_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode7_vel_x_i_limit, g_fc_params.mode7_vel_x_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(mode7_vel_x_d_lpf, g_fc_params.mode7_vel_x_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(mode7_vel_y_kp, g_fc_params.mode7_vel_y_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode7_vel_y_ki, g_fc_params.mode7_vel_y_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode7_vel_y_kd, g_fc_params.mode7_vel_y_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode7_vel_y_kff, g_fc_params.mode7_vel_y_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode7_vel_y_i_limit, g_fc_params.mode7_vel_y_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(mode7_vel_y_d_lpf, g_fc_params.mode7_vel_y_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(pos_est_k_flow, g_fc_params.pos_est_k_flow, 0.0f, 1.0f);

    AIR_COMM_REGISTER_FLOAT(mode5_img_x_kp, g_fc_params.mode5_img_x_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_img_x_ki, g_fc_params.mode5_img_x_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_img_x_kd, g_fc_params.mode5_img_x_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_img_x_kff, g_fc_params.mode5_img_x_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_img_x_i_limit, g_fc_params.mode5_img_x_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_img_x_d_lpf, g_fc_params.mode5_img_x_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_img_y_kp, g_fc_params.mode5_img_y_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_img_y_ki, g_fc_params.mode5_img_y_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_img_y_kd, g_fc_params.mode5_img_y_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_img_y_kff, g_fc_params.mode5_img_y_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_img_y_i_limit, g_fc_params.mode5_img_y_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_img_y_d_lpf, g_fc_params.mode5_img_y_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_vel_x_kp, g_fc_params.mode5_vel_x_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_vel_x_ki, g_fc_params.mode5_vel_x_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_vel_x_kd, g_fc_params.mode5_vel_x_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_vel_x_kff, g_fc_params.mode5_vel_x_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_vel_x_i_limit, g_fc_params.mode5_vel_x_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_vel_x_d_lpf, g_fc_params.mode5_vel_x_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_vel_y_kp, g_fc_params.mode5_vel_y_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_vel_y_ki, g_fc_params.mode5_vel_y_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_vel_y_kd, g_fc_params.mode5_vel_y_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_vel_y_kff, g_fc_params.mode5_vel_y_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_vel_y_i_limit, g_fc_params.mode5_vel_y_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_vel_y_d_lpf, g_fc_params.mode5_vel_y_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_kp_car_x, g_fc_params.mode5_kp_car_x, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode5_kp_car_y, g_fc_params.mode5_kp_car_y, 0.0f, 3000.0f);

    AIR_COMM_REGISTER_FLOAT(mode8_img_x_kp, g_fc_params.mode8_img_x_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_img_x_ki, g_fc_params.mode8_img_x_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_img_x_kd, g_fc_params.mode8_img_x_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_img_x_kff, g_fc_params.mode8_img_x_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_img_x_i_limit, g_fc_params.mode8_img_x_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_img_x_d_lpf, g_fc_params.mode8_img_x_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_img_y_kp, g_fc_params.mode8_img_y_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_img_y_ki, g_fc_params.mode8_img_y_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_img_y_kd, g_fc_params.mode8_img_y_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_img_y_kff, g_fc_params.mode8_img_y_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_img_y_i_limit, g_fc_params.mode8_img_y_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_img_y_d_lpf, g_fc_params.mode8_img_y_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_vel_x_kp, g_fc_params.mode8_vel_x_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_vel_x_ki, g_fc_params.mode8_vel_x_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_vel_x_kd, g_fc_params.mode8_vel_x_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_vel_x_kff, g_fc_params.mode8_vel_x_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_vel_x_i_limit, g_fc_params.mode8_vel_x_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_vel_x_d_lpf, g_fc_params.mode8_vel_x_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_vel_y_kp, g_fc_params.mode8_vel_y_kp, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_vel_y_ki, g_fc_params.mode8_vel_y_ki, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_vel_y_kd, g_fc_params.mode8_vel_y_kd, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_vel_y_kff, g_fc_params.mode8_vel_y_kff, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_vel_y_i_limit, g_fc_params.mode8_vel_y_i_limit, 0.0f, 5000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_vel_y_d_lpf, g_fc_params.mode8_vel_y_d_lpf, 0.0f, 500.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_kp_car_x, g_fc_params.mode8_kp_car_x, 0.0f, 3000.0f);
    AIR_COMM_REGISTER_FLOAT(mode8_kp_car_y, g_fc_params.mode8_kp_car_y, 0.0f, 3000.0f);

    if(air_comm_register_remote_param("c1_beacon_thr",
                                      &c1_beacon_thr,
                                      AIR_COMM_AIR_PARAM_TYPE_INT32,
                                      0.0f,
                                      255.0f,
                                      IPC_REMOTE_PARAM_TARGET_CORE1,
                                      IPC_REMOTE_PARAM_ID_BEACON_THRESHOLD) == 0U)
    {
        s_air_comm_registration_ok = 0U;
    }
    if(air_comm_register_remote_param("bl3_beacon_thr",
                                      &bl3_beacon_thr,
                                      AIR_COMM_AIR_PARAM_TYPE_INT32,
                                      0.0f,
                                      255.0f,
                                      IPC_REMOTE_PARAM_TARGET_2BL3,
                                      IPC_REMOTE_PARAM_ID_BEACON_THRESHOLD) == 0U)
    {
        s_air_comm_registration_ok = 0U;
    }
    if(air_comm_register_remote_param("c1_exp_time",
                                      &c1_exp_time,
                                      AIR_COMM_AIR_PARAM_TYPE_INT32,
                                      0.0f,
                                      636.0f,
                                      IPC_REMOTE_PARAM_TARGET_CORE1,
                                      IPC_REMOTE_PARAM_ID_EXP_TIME) == 0U)
    {
        s_air_comm_registration_ok = 0U;
    }
    if(air_comm_register_remote_param("bl3_exp_time",
                                      &bl3_exp_time,
                                      AIR_COMM_AIR_PARAM_TYPE_INT32,
                                      0.0f,
                                      636.0f,
                                      IPC_REMOTE_PARAM_TARGET_2BL3,
                                      IPC_REMOTE_PARAM_ID_EXP_TIME) == 0U)
    {
        s_air_comm_registration_ok = 0U;
    }
    if(air_comm_register_remote_param("c1_screen_mode",
                                      &c1_screen_mode,
                                      AIR_COMM_AIR_PARAM_TYPE_INT32,
                                      0.0f,
                                      2.0f,
                                      IPC_REMOTE_PARAM_TARGET_CORE1,
                                      IPC_REMOTE_PARAM_ID_SCREEN_MODE) == 0U)
    {
        s_air_comm_registration_ok = 0U;
    }

    if((s_air_comm_param_count != AIR_COMM_DEFAULT_PARAM_COUNT) ||
       (air_comm_register_default_commands() == 0U))
    {
        s_air_comm_registration_ok = 0U;
    }

    if(s_air_comm_registration_ok == 0U)
    {
        return;
    }

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

    air_comm_remote_poll();

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

    if((s_air_comm_active_command != NULL) &&
       (air_comm_remote_operation_allowed() == 0U))
    {
        uint8 active_seq = s_air_comm_active_seq;

        air_comm_stop_active_command();
        s_air_comm_last_done_valid = 0U;
        (void)air_comm_air_send_command_ack_text(active_seq, "ACK_ERROR 3 state");
        s_air_comm_stats.command_fail_count++;
    }
    else if(s_air_comm_active_command != NULL)
    {
        if((s_air_comm_active_command->mode == AIR_COMM_AIR_COMMAND_MODE_POLLING) &&
           (s_air_comm_active_command->run != NULL))
        {
            if(s_air_comm_screen_ready == 0U)
            {
                air_comm_screen_reset();
            }
            s_air_comm_active_command->run();
        }
        else if(s_air_comm_active_command->mode == AIR_COMM_AIR_COMMAND_MODE_INSTANT)
        {
            if(s_air_comm_active_command->run != NULL)
            {
                s_air_comm_active_command->run();
            }
            s_air_comm_last_done_valid = 1U;
            s_air_comm_last_done_seq = s_air_comm_active_seq;
            strncpy(s_air_comm_last_done_name,
                    s_air_comm_active_command->name,
                    AIR_COMM_AIR_COMMAND_NAME_MAX);
            s_air_comm_last_done_name[AIR_COMM_AIR_COMMAND_NAME_MAX] = '\0';
            s_air_comm_active_command = NULL;
            (void)air_comm_air_send_command_ack_text(s_air_comm_active_seq, "ACK_EXIT_OK");
        }
    }

    air_comm_task_online();
}

uint8 air_comm_air_remote_param_busy(void)
{
    return ((s_air_comm_remote_pending.active != 0U) ||
            (ipc_remote_param_core0_is_busy() != 0U)) ? 1U : 0U;
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
uint8 air_comm_air_register_param(const char *name, void *var, uint8 type, float min, float max)
{
    uint16 i;

    if((name == NULL) || (var == NULL) || (min > max) || (type > AIR_COMM_AIR_PARAM_TYPE_INT32))
    {
        return 0U;
    }

    /* 同名参数更新，不重复占位 */
    for(i = 0U; i < s_air_comm_param_count; i++)
    {
        if(strcmp(s_air_comm_params[i].name, name) == 0)
        {
            s_air_comm_params[i].var = var;
            s_air_comm_params[i].type = type;
            s_air_comm_params[i].target = AIR_COMM_PARAM_TARGET_LOCAL;
            s_air_comm_params[i].param_id = 0U;
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
    s_air_comm_params[s_air_comm_param_count].type = type;
    s_air_comm_params[s_air_comm_param_count].target = AIR_COMM_PARAM_TARGET_LOCAL;
    s_air_comm_params[s_air_comm_param_count].param_id = 0U;
    s_air_comm_params[s_air_comm_param_count].min = min;
    s_air_comm_params[s_air_comm_param_count].max = max;
    s_air_comm_param_count++;

    return 1U;
}

/* 注册只作为核0菜单镜像、实际由核1或2BL3执行的参数。 */
static uint8 air_comm_register_remote_param(const char *name,
                                            void *var,
                                            uint8 type,
                                            float min,
                                            float max,
                                            uint8 target,
                                            uint16 param_id)
{
    uint16 index;

    if(((target != IPC_REMOTE_PARAM_TARGET_CORE1) &&
        (target != IPC_REMOTE_PARAM_TARGET_2BL3)) ||
       (param_id == 0U) ||
       (air_comm_air_register_param(name, var, type, min, max) == 0U))
    {
        return 0U;
    }

    for(index = 0U; index < s_air_comm_param_count; index++)
    {
        if(strcmp(s_air_comm_params[index].name, name) == 0)
        {
            s_air_comm_params[index].target = target;
            s_air_comm_params[index].param_id = param_id;
            return 1U;
        }
    }

    return 0U;
}

/*
 * 向小车上报运行数据。
 * payload 格式：count(1B) + float0(4B) + float1(4B) + ...
 * 最多 32 个 float，每个 4 字节，小端序。
 * 发送成功才自增 seq。
 */
uint8 air_comm_send_run_data(const float *data, uint8 count)
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

uint8 air_comm_air_send_run_data(const float *data, uint8 count)
{
    return air_comm_send_run_data(data, count);
}

void air_comm_set_run_data_callback(air_comm_run_data_fn callback)
{
    s_air_comm_run_data_callback = callback;
}

static uint8 air_comm_air_register_command_internal(const char *name,
                                                    air_comm_air_command_mode_t mode,
                                                    air_comm_air_command_fn run)
{
    uint8 index;

    if((name == NULL) || (run == NULL) || (mode > AIR_COMM_AIR_COMMAND_MODE_INSTANT))
    {
        return 0U;
    }

    if((strlen(name) == 0U) || (strlen(name) > AIR_COMM_AIR_COMMAND_NAME_MAX))
    {
        return 0U;
    }

    for(index = 0U; index < s_air_comm_command_count; index++)
    {
        if(air_comm_command_name_equal(s_air_comm_commands[index].name, name) != 0U)
        {
            s_air_comm_commands[index].mode = mode;
            s_air_comm_commands[index].run = run;
            return 1U;
        }
    }

    if(s_air_comm_command_count >= AIR_COMM_COMMAND_TABLE_MAX)
    {
        return 0U;
    }

    s_air_comm_commands[s_air_comm_command_count].name = name;
    s_air_comm_commands[s_air_comm_command_count].mode = mode;
    s_air_comm_commands[s_air_comm_command_count].run = run;
    s_air_comm_command_count++;

    return 1U;
}

uint8 air_comm_air_register_polling_command(const char *name, air_comm_air_command_fn run)
{
    return air_comm_air_register_command_internal(name, AIR_COMM_AIR_COMMAND_MODE_POLLING, run);
}

uint8 air_comm_air_register_instant_command(const char *name, air_comm_air_command_fn run)
{
    return air_comm_air_register_command_internal(name, AIR_COMM_AIR_COMMAND_MODE_INSTANT, run);
}

uint8 air_comm_air_send_command_ack_text(uint8 seq, const char *text)
{
    uint16 text_len;
    uint8 payload[AIR_COMM_AIR_ACK_TEXT_MAX];

    if((s_air_comm_initialized == 0U) || (text == NULL))
    {
        return 0U;
    }

    text_len = (uint16)strlen(text);
    if(text_len > AIR_COMM_AIR_ACK_TEXT_MAX)
    {
        text_len = AIR_COMM_AIR_ACK_TEXT_MAX;
    }

    if(text_len > 0U)
    {
        memcpy(payload, text, text_len);
    }

    return air_comm_send_frame(AIR_COMM_MSG_ACK_COMMAND, seq, payload, (uint8)text_len);
}

uint8 air_comm_get_last_run_data(float *data, uint8 max_count, uint8 *count)
{
    if(count != NULL)
    {
        *count = 0U;
    }

    if((data == NULL) ||
       (count == NULL) ||
       (s_air_comm_last_run_data_valid == 0U) ||
       (max_count < s_air_comm_last_run_data_count))
    {
        return 0U;
    }

    if(s_air_comm_last_run_data_count > 0U)
    {
        memcpy(data,
               s_air_comm_last_run_data,
               (size_t)s_air_comm_last_run_data_count * sizeof(float));
    }
    *count = s_air_comm_last_run_data_count;

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
