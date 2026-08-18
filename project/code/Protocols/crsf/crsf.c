#include "crsf.h"
#include "zf_common_headfile.h"

/* CRSF 接收机使用 UART4，引脚为 P14_1(TX) / P14_0(RX)，底层在 zf_driver_uart 中映射到 SCB2。 */
#define CRSF_UART_INDEX        (UART_4)          /* CRSF 使用的串口号。 */
#define CRSF_UART_TX_PIN       (UART4_TX_P14_1) /* CRSF 回传使用的发送引脚。 */
#define CRSF_UART_RX_PIN       (UART4_RX_P14_0) /* CRSF 接收机数据输入引脚。 */
#define CRSF_UART_BAUDRATE     (420000)         /* CRSF 串口波特率，单位 bit/s。 */

#define CRSF_TIMER_INDEX       (TC_TIME2_CH0)   /* CRSF 链路超时检测使用的定时器通道。 */
#define CRSF_LINK_TIMEOUT_US   (500000)         /* CRSF 判定失联的超时时间，单位 us。 */

#define CRSF_CH_MID            (992)            /* 居中类通道的默认中位值。 */
#define CRSF_CH_LOW            (172)            /* 油门/开关类通道的默认低位值。 */
#define CRSF_SYNC_ADDR_FC      (0xC8)           /* 发往飞控的 CRSF 同步地址。 */
#define CRSF_ADDR_TX           (0xEA)           /* 飞控回传时使用的设备地址。 */
#define CRSF_FRAME_MAX_LEN     (64)             /* CRSF 单帧最大长度。 */
#define CRSF_FRAME_MIN_LEN     (2)              /* CRSF 单帧最小长度。 */
#define CRSF_TYPE_RC_CHANNELS  (0x16)           /* CRSF 遥控通道帧类型。 */
#define CRSF_TYPE_ATTITUDE     (0x1E)           /* CRSF 姿态回传帧类型。 */
#define CRSF_RC_PAYLOAD_LEN    (22)             /* 10 通道 RC 数据负载长度。 */
#define CRSF_ATT_PAYLOAD_LEN   (6)              /* 姿态回传负载长度。 */

volatile uint16_t CRSF_CH[CRSF_CH_COUNT] = {0}; /* CRSF 原始通道值，范围 0~2047。 */
volatile int16_t CRSF_STD[CRSF_CH_COUNT] = {0}; /* CRSF 标准化后的通道值。 */
volatile uint32_t CRSF_LAST_UPDATE_TIME = 0;    /* 最近一次收到有效 CRSF 帧的时间戳，单位 us。 */
volatile uint8_t CRSF_LINK_UP = 0;              /* CRSF 链路状态，1=正常，0=失联。 */

static uint8_t crsf_rx_len = 0;
static uint8_t crsf_rx_pos = 0;
static uint8_t crsf_rx_state = 0;
static uint8_t crsf_rx_buf[CRSF_FRAME_MAX_LEN] = {0};

static uint8_t crsf_crc8_update(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (crc & 0x80)
        {
            crc = (uint8_t)((crc << 1) ^ 0xD5);
        }
        else
        {
            crc <<= 1;
        }
    }
    return crc;
}

static uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc = crsf_crc8_update(crc, data[i]);
    }
    return crc;
}

static void crsf_write_be16(uint8_t *buf, uint8_t *idx, int16_t value)
{
    buf[(*idx)++] = (uint8_t)((value >> 8) & 0xFF);
    buf[(*idx)++] = (uint8_t)(value & 0xFF);
}

static int16_t crsf_map_range(uint16_t value, uint16_t in_min, uint16_t in_max, int16_t out_min, int16_t out_max)
{
    if (in_max <= in_min)
    {
        return out_min;
    }

    if (value <= in_min)
    {
        return out_min;
    }
    if (value >= in_max)
    {
        return out_max;
    }

    int32_t num = (int32_t)(value - in_min) * (int32_t)(out_max - out_min);
    int32_t den = (int32_t)(in_max - in_min);
    return (int16_t)(out_min + (num / den));
}

static int16_t crsf_convert_channel(uint16_t raw, const CRSF_ChannelConfig *cfg)
{
    switch (cfg->type)
    {
        case CRSF_CH_TYPE_AXIS_CENTER:
        case CRSF_CH_TYPE_THROTTLE:
            if (raw >= cfg->mid)
            {
                return crsf_map_range(raw, cfg->mid, cfg->max, 0, 1000);
            }
            return crsf_map_range(raw, cfg->min, cfg->mid, -1000, 0);

        case CRSF_CH_TYPE_SWITCH_2POS:
        case CRSF_CH_TYPE_BUTTON:
            return (raw >= cfg->high_th) ? 1 : 0;

        case CRSF_CH_TYPE_SWITCH_3POS:
            if (raw <= cfg->low_th)
            {
                return 0;
            }
            if (raw >= cfg->high_th)
            {
                return 2;
            }
            return 1;

        default:
            return 0;
    }
}

static void crsf_update_std(void)
{
    for (uint8_t i = 0; i < CRSF_CH_COUNT; i++)
    {
        CRSF_STD[i] = crsf_convert_channel(CRSF_CH[i], &CRSF_CHANNEL_CONFIG[i]);
    }
}

static void crsf_parse_channels(const uint8_t *payload)
{
    CRSF_CH[0]  = (uint16_t)((payload[0] | (payload[1] << 8)) & 0x07FF);
    CRSF_CH[1]  = (uint16_t)(((payload[1] >> 3) | (payload[2] << 5)) & 0x07FF);
    CRSF_CH[2]  = (uint16_t)(((payload[2] >> 6) | (payload[3] << 2) | (payload[4] << 10)) & 0x07FF);
    CRSF_CH[3]  = (uint16_t)(((payload[4] >> 1) | (payload[5] << 7)) & 0x07FF);
    CRSF_CH[4]  = (uint16_t)(((payload[5] >> 4) | (payload[6] << 4)) & 0x07FF);
    CRSF_CH[5]  = (uint16_t)(((payload[6] >> 7) | (payload[7] << 1) | (payload[8] << 9)) & 0x07FF);
    CRSF_CH[6]  = (uint16_t)(((payload[8] >> 2) | (payload[9] << 6)) & 0x07FF);
    CRSF_CH[7]  = (uint16_t)(((payload[9] >> 5) | (payload[10] << 3)) & 0x07FF);
    CRSF_CH[8]  = (uint16_t)((payload[11] | (payload[12] << 8)) & 0x07FF);
    CRSF_CH[9]  = (uint16_t)(((payload[12] >> 3) | (payload[13] << 5)) & 0x07FF);

    CRSF_LAST_UPDATE_TIME = timer_get(CRSF_TIMER_INDEX);
    crsf_update_std();
}

static void crsf_rx_byte(uint8_t byte)
{
    switch (crsf_rx_state)
    {
        case 0:
            if (byte == CRSF_SYNC_ADDR_FC)
            {
                crsf_rx_state = 1;
            }
            break;

        case 1:
            if (byte < CRSF_FRAME_MIN_LEN || byte > CRSF_FRAME_MAX_LEN)
            {
                crsf_rx_state = 0;
                break;
            }
            crsf_rx_len = byte;
            crsf_rx_pos = 0;
            crsf_rx_state = 2;
            break;

        case 2:
            crsf_rx_buf[crsf_rx_pos++] = byte;
            if (crsf_rx_pos >= crsf_rx_len)
            {
                uint8_t crc = crsf_crc8(crsf_rx_buf, (uint8_t)(crsf_rx_len - 1));
                uint8_t crc_rx = crsf_rx_buf[crsf_rx_len - 1];

                if (crc == crc_rx)
                {
                    uint8_t frame_type = crsf_rx_buf[0];
                    if ((frame_type == CRSF_TYPE_RC_CHANNELS) && (crsf_rx_len == (CRSF_RC_PAYLOAD_LEN + 2)))
                    {
                        crsf_parse_channels(&crsf_rx_buf[1]);
                    }
                }

                crsf_rx_state = 0;
            }
            break;

        default:
            crsf_rx_state = 0;
            break;
    }
}

void CRSF_Update_100HZ(void)
{
    volatile stc_SCB_t *crsf_scb = get_scb_module(CRSF_UART_INDEX);

    while (Cy_SCB_GetNumInRxFifo(crsf_scb))
    {
        uint8_t data = (uint8_t)Cy_SCB_ReadRxFifo(crsf_scb);
        crsf_rx_byte(data);
    }

    uint32_t now = timer_get(CRSF_TIMER_INDEX);
    if ((uint32_t)(now - CRSF_LAST_UPDATE_TIME) > CRSF_LINK_TIMEOUT_US)
    {
        CRSF_LINK_UP = 0;
        CRSF_CH[0] = CRSF_CH_MID;
        CRSF_CH[1] = CRSF_CH_MID;
        CRSF_CH[2] = CRSF_CH_LOW;
        CRSF_CH[3] = CRSF_CH_MID;
        CRSF_CH[4] = CRSF_CH_LOW;
        CRSF_CH[5] = CRSF_CH_LOW;
        CRSF_CH[6] = CRSF_CH_LOW;
        CRSF_CH[7] = CRSF_CH_LOW;
        CRSF_CH[8] = CRSF_CH_LOW;
        CRSF_CH[9] = CRSF_CH_LOW;
        crsf_update_std();
    }
    else
    {
        CRSF_LINK_UP = 1;
    }
}

void crsf_init(void)
{
    uart_init(CRSF_UART_INDEX, CRSF_UART_BAUDRATE, CRSF_UART_TX_PIN, CRSF_UART_RX_PIN);
    uart_rx_interrupt(CRSF_UART_INDEX, 0);

    timer_init(CRSF_TIMER_INDEX, TIMER_US);
    timer_start(CRSF_TIMER_INDEX);
}

static void crsf_send_attitude(int16_t roll, int16_t pitch, int16_t yaw)
{
    volatile stc_SCB_t *crsf_scb = get_scb_module(CRSF_UART_INDEX);
    uint8_t buf[1 + 1 + 1 + CRSF_ATT_PAYLOAD_LEN + 1];
    uint8_t idx = 0;

    buf[idx++] = CRSF_ADDR_TX;
    buf[idx++] = (uint8_t)(1 + CRSF_ATT_PAYLOAD_LEN + 1);
    buf[idx++] = CRSF_TYPE_ATTITUDE;

    crsf_write_be16(buf, &idx, roll);
    crsf_write_be16(buf, &idx, pitch);
    crsf_write_be16(buf, &idx, yaw);

    buf[idx++] = crsf_crc8(&buf[2], (uint8_t)(1 + CRSF_ATT_PAYLOAD_LEN));

    if ((Cy_SCB_GetFifoSize(crsf_scb) - Cy_SCB_GetNumInTxFifo(crsf_scb)) >= idx)
    {
        (void)Cy_SCB_UART_PutArray(crsf_scb, buf, idx);
    }
}


/* 生成并尝试提交低频CRSF姿态回传帧，硬件FIFO忙时直接丢弃。 */
void crsf_send_10hz(void)
{

    const float k_deg_to_rad_1e4 = 174.532925f; // deg * (pi/180) * 10000
    int16_t pitch = (int16_t)(g_euler.pitch * k_deg_to_rad_1e4);
    int16_t roll = (int16_t)(g_euler.roll * k_deg_to_rad_1e4);
    int16_t yaw = (int16_t)(g_euler.yaw * k_deg_to_rad_1e4);

    crsf_send_attitude(pitch, roll, yaw);
}

