#include "camera_spi.h"

#include "IPC/ipc_image_data.h"
#include "gpio/cy_gpio.h"
#include "scb/cy_scb_spi.h"
#include "sysclk/cy_sysclk.h"

#define CAMERA_SPI_TRANSFER_LEN             (97U)
#define CAMERA_SPI_APP_DATA_CAPACITY        (77U)
#define CAMERA_SPI_PARAM_APP_LEN             (24U)
#define CAMERA_SPI_PARAM_ACK_APP_LEN        (20U)
#define CAMERA_SPI_ATTITUDE_APP_LEN          (43U)

#define CAMERA_SPI_FRAME_HEAD_0             (0xAAU)
#define CAMERA_SPI_FRAME_HEAD_1             (0x55U)
#define CAMERA_SPI_FRAME_TAIL               (0xEDU)
#define CAMERA_SPI_CMD_SYNC_DATA            (0x20U)
#define CAMERA_SPI_REQ_META_SIZE            (6U)
#define CAMERA_SPI_RESP_META_SIZE           (12U)
#define CAMERA_SPI_REQ_PAYLOAD_SIZE         (CAMERA_SPI_REQ_META_SIZE + CAMERA_SPI_APP_DATA_CAPACITY)
#define CAMERA_SPI_RESP_PAYLOAD_SIZE        (CAMERA_SPI_RESP_META_SIZE + CAMERA_SPI_APP_DATA_CAPACITY)
#define CAMERA_SPI_FRAME_OVERHEAD           (8U)
#define CAMERA_SPI_REQ_FRAME_LEN            (CAMERA_SPI_FRAME_OVERHEAD + CAMERA_SPI_REQ_PAYLOAD_SIZE)
#define CAMERA_SPI_RESP_FRAME_LEN           (CAMERA_SPI_FRAME_OVERHEAD + CAMERA_SPI_RESP_PAYLOAD_SIZE)
#define CAMERA_SPI_DOWNLINK_MAGIC           (0x5AU)
#define CAMERA_SPI_BAUDRATE                 (2000000UL)
#define CAMERA_SPI_CLOCK_OVERSAMPLE         (4U)
#define CAMERA_SPI_TRANSFER_TIMEOUT_US      (2000U)
#define CAMERA_SPI_TRANSFER_TIMEOUT_POLLS   (100000U)
#define CAMERA_SPI_IMAGE_STALE_TIME_US      (50000U)
#define CAMERA_SPI_LINK_STALE_TIME_US       (100000U)
#define CAMERA_SPI_SCREEN_LINK_STALE_TIME_US (350000U) /* 开屏初始化期间的链路失效时间。 */
#define CAMERA_SPI_IMAGE_STALE_CYCLES \
    ((CAMERA_SPI_IMAGE_STALE_TIME_US + CAMERA_SPI_UPDATE_PERIOD_US - 1U) / \
     CAMERA_SPI_UPDATE_PERIOD_US)
#define CAMERA_SPI_LINK_STALE_CYCLES \
    ((CAMERA_SPI_LINK_STALE_TIME_US + CAMERA_SPI_UPDATE_PERIOD_US - 1U) / \
     CAMERA_SPI_UPDATE_PERIOD_US)
/* 开屏链路失效时间对应的调度周期数。 */
#define CAMERA_SPI_SCREEN_LINK_STALE_CYCLES \
    ((CAMERA_SPI_SCREEN_LINK_STALE_TIME_US + CAMERA_SPI_UPDATE_PERIOD_US - 1U) / \
     CAMERA_SPI_UPDATE_PERIOD_US)
#define CAMERA_SPI_ALL_BOARD_MASK           ((1U << CAMERA_SPI_BOARD_COUNT) - 1U)
#define CAMERA_SPI_DWT_UNLOCK_KEY           (0xC5ACCE55UL)
#define CAMERA_SPI_READY0_PIN               P01_0
#define CAMERA_SPI_READY1_PIN               P01_1
#define CAMERA_SPI_CS0_PIN                  P03_3
#define CAMERA_SPI_CS1_PIN                  P03_4
#define CAMERA_SPI_SCB                      SCB6
#define CAMERA_SPI_IRQ_SRC                  scb_6_interrupt_IRQn
#define CAMERA_SPI_CPU_IRQ                  CPUIntIdx5_IRQn
#define CAMERA_SPI_PERI_FREQ                CY_INITIAL_TARGET_PERI_FREQ
/* 下行字节8复用为参数写锁：0允许SET，1禁止SET；GET不受影响。 */
#define CAMERA_SPI_DOWNLINK_PARAM_WRITE_LOCK_OFFSET (8U)
#define CAMERA_SPI_PARAM_WRITE_ALLOWED      (0U)
#define CAMERA_SPI_PARAM_WRITE_LOCKED       (1U)

/* 2BL3参数命令与ACK协议固定字段。 */
#define CAMERA_SPI_PARAM_MAGIC              (0xC3U)
#define CAMERA_SPI_PARAM_ACK_MAGIC          (0x3CU)
#define CAMERA_SPI_PARAM_VERSION            (1U)
#define CAMERA_SPI_PARAM_TIMEOUT_US         (120000U)
#define CAMERA_SPI_SCREEN_PARAM_TIMEOUT_US  (350000U) /* 开屏时普通参数单阶段超时时间。 */
#define CAMERA_SPI_PARAM_EXP_TIMEOUT_US     (800000U)
#define CAMERA_SPI_PARAM_MAX_CYCLES \
    ((CAMERA_SPI_PARAM_TIMEOUT_US + CAMERA_SPI_UPDATE_PERIOD_US - 1U) / \
     CAMERA_SPI_UPDATE_PERIOD_US)
/* 开屏参数超时对应的调度周期数。 */
#define CAMERA_SPI_SCREEN_PARAM_MAX_CYCLES \
    ((CAMERA_SPI_SCREEN_PARAM_TIMEOUT_US + CAMERA_SPI_UPDATE_PERIOD_US - 1U) / \
     CAMERA_SPI_UPDATE_PERIOD_US)
#define CAMERA_SPI_PARAM_EXP_MAX_CYCLES \
    ((CAMERA_SPI_PARAM_EXP_TIMEOUT_US + CAMERA_SPI_UPDATE_PERIOD_US - 1U) / \
     CAMERA_SPI_UPDATE_PERIOD_US)
/* GET不一致时使用不同事务号复读，避免2BL3返回同一事务的缓存ACK。 */
#define CAMERA_SPI_PARAM_GET_MISMATCH_ATTEMPTS (3U)
#define CAMERA_SPI_PARAM_PREFLIGHT_TXN_MASK (0x40000000UL)
#define CAMERA_SPI_PARAM_ROLLBACK_TXN_MASK  (0x80000000UL)
#define CAMERA_SPI_PARAM_ORIGINAL_TXN_MASK  (0xC0000000UL)
#define CAMERA_SPI_PARAM_ACK_REQUEST        (0x01U)

#define CAMERA_SPI_ATTITUDE_MAGIC            (0xA6U)
#define CAMERA_SPI_ATTITUDE_VERSION          (1U)
#define CAMERA_SPI_ATTITUDE_MAGIC_OFFSET     (24U)
#define CAMERA_SPI_ATTITUDE_VERSION_OFFSET   (25U)
#define CAMERA_SPI_ATTITUDE_SEQUENCE_OFFSET  (26U)
#define CAMERA_SPI_ATTITUDE_ROLL_OFFSET      (30U)
#define CAMERA_SPI_ATTITUDE_PITCH_OFFSET     (34U)
#define CAMERA_SPI_ATTITUDE_HEIGHT_OFFSET    (38U)
#define CAMERA_SPI_ATTITUDE_FLAGS_OFFSET     (42U)
#define CAMERA_SPI_ATTITUDE_HEIGHT_VALID     (0x01U)
#define CAMERA_SPI_OUTPUT_WIFI_TCP_ENABLE     (0x01U) /* 下行输出控制的WiFi TCP使能位。 */
#define CAMERA_SPI_OUTPUT_SCREEN_ENABLE       (0x02U) /* 下行输出控制的本地屏幕使能位。 */
#define CAMERA_SPI_OUTPUT_HORIZON_ENABLE      (0x04U) /* 下行输出控制的前后摄地平线使能位。 */

#define CAMERA_SPI_IMAGE_VERSION_OFFSET        (0U)
#define CAMERA_SPI_IMAGE_BEACON_COUNT_OFFSET   (1U)
#define CAMERA_SPI_IMAGE_LAMP_COUNT_OFFSET     (2U)
#define CAMERA_SPI_IMAGE_RESULT_SEQUENCE_OFFSET (3U)
#define CAMERA_SPI_IMAGE_PROTOCOL_VERSION      (4U)
#define CAMERA_SPI_IMAGE_HEADER_SIZE           (4U)
#define CAMERA_SPI_IMAGE_BEACON_VALID_OFFSET   (0U)
#define CAMERA_SPI_IMAGE_BEACON_X_OFFSET       (1U)  /* float LE: centered image x, right positive */
#define CAMERA_SPI_IMAGE_BEACON_Y_OFFSET       (5U)  /* float LE: centered image y, down positive */
#define CAMERA_SPI_IMAGE_BEACON_AREA_OFFSET    (9U)
#define CAMERA_SPI_IMAGE_BEACON_SLOT_SIZE      (13U)
#define CAMERA_SPI_IMAGE_BEACON_COUNT          (4U)
#define CAMERA_SPI_IMAGE_LAMP_VALID_OFFSET     (0U)
#define CAMERA_SPI_IMAGE_LAMP_CX_OFFSET        (1U)  /* float LE: centered image x, right positive */
#define CAMERA_SPI_IMAGE_LAMP_CY_OFFSET        (5U)  /* float LE: centered image y, down positive */
#define CAMERA_SPI_IMAGE_LAMP_WIDTH_OFFSET     (9U)
#define CAMERA_SPI_IMAGE_LAMP_LENGTH_OFFSET    (13U)
#define CAMERA_SPI_IMAGE_LAMP_ANGLE_OFFSET     (17U)
#define CAMERA_SPI_IMAGE_LAMP_SLOT_SIZE        (21U)
#define CAMERA_SPI_IMAGE_BEACON_PACKET_OFFSET  CAMERA_SPI_IMAGE_HEADER_SIZE
#define CAMERA_SPI_IMAGE_LAMP_PACKET_OFFSET \
    (CAMERA_SPI_IMAGE_BEACON_PACKET_OFFSET + \
     (CAMERA_SPI_IMAGE_BEACON_COUNT * CAMERA_SPI_IMAGE_BEACON_SLOT_SIZE))

#define CAMERA_SPI_ERR_OK                   (0U)
#define CAMERA_SPI_ERR_INVALID_BOARD        (1U)
#define CAMERA_SPI_ERR_NOT_READY            (2U)
#define CAMERA_SPI_ERR_TRANSFER_BUSY        (3U)
#define CAMERA_SPI_ERR_HW                   (4U)
#define CAMERA_SPI_ERR_TIMEOUT              (5U)
#define CAMERA_SPI_ERR_HEAD                 (6U)
#define CAMERA_SPI_ERR_CMD                  (7U)
#define CAMERA_SPI_ERR_LEN                  (8U)
#define CAMERA_SPI_ERR_TAIL                 (9U)
#define CAMERA_SPI_ERR_CRC                  (10U)
#define CAMERA_SPI_ERR_APP_LEN              (11U)

typedef struct
{
    uint8 online;
    uint8 last_error;
    uint8 peer_last_error;
    uint8 flags;
    uint8 version;
    uint8 beacon_count;
    uint8 car_lamp_count;
    uint8 result_sequence;
    uint8 result_sequence_initialized;
    uint32 tx_sequence;
    uint32 tx_counter;
    uint32 rx_sequence;
    uint32 ack_sequence;
    uint32 rx_ok_count;
    uint32 rx_error_count;
    uint8 link_age_cycles;
    uint8 image_age_cycles;
    uint8 last_rx_head0;
    uint8 last_rx_head1;
    beacon_data beacons[IMAGE_MAX_BEACON_COUNT];
    car_lamp_data car_lamps[CAMERA_SPI_MAX_CAR_LAMPS];
} camera_spi_board_state_t;

typedef enum
{
    CAMERA_SPI_PARAM_IDLE = 0,
    CAMERA_SPI_PARAM_PREFLIGHT,
    CAMERA_SPI_PARAM_ACTIVE,
    CAMERA_SPI_PARAM_ROLLBACK,
    CAMERA_SPI_PARAM_COMPLETE
} camera_spi_param_state_t;

typedef struct
{
    camera_spi_param_state_t state;
    uint8 op;
    uint8 type;
    uint8 command_mask;
    uint8 command_due_mask;
    uint8 ack_mask;
    uint8 set_sent_mask;
    uint8 cycle_count;
    uint8 final_status;
    uint8 cancel_requested;
    uint8 rollback_fail_seen;
    uint8 get_mismatch_count;
    uint8 get_mismatch_stable;
    uint16 param_id;
    uint32 transaction;
    uint32 value_bits;
    uint32 fallback_bits;
    uint32 previous_bits[CAMERA_SPI_BOARD_COUNT];
    uint32 actual_bits[CAMERA_SPI_BOARD_COUNT];
    uint32 get_mismatch_reference[CAMERA_SPI_BOARD_COUNT];
    uint8 board_status[CAMERA_SPI_BOARD_COUNT];
} camera_spi_param_transaction_t;

typedef struct
{
    uint8 valid;
    uint8 type;
    uint16 param_id;
    uint32 transaction;
    uint32 previous_bits[CAMERA_SPI_BOARD_COUNT];
} camera_spi_param_rollback_cache_t;

static camera_spi_board_state_t s_boards[CAMERA_SPI_BOARD_COUNT];
static cy_stc_scb_spi_context_t s_spi_context;
static uint8 s_tx_frame[CAMERA_SPI_TRANSFER_LEN];
static uint8 s_rx_frame[CAMERA_SPI_TRANSFER_LEN];
static uint8 s_initialized;
static uint8 s_active;
static uint8 s_active_board;
static uint8 s_flight_state;
/* 核0同步的参数写锁，1表示当前禁止2BL3参数SET。 */
static uint8 s_param_write_locked;
/* 核0同步过来的 2BL3 图传发送模式 */
static uint8 s_image_send_enable;
static uint8 s_bl3_screen_enable; /* 核0下发的前后摄屏幕使能状态。 */
static uint8 s_bl3_horizon_enable; /* 核0下发的前后摄地平线使能状态。 */
static uint8 s_ready_mask;
static uint8 s_cycle_active;
static uint8 s_cycle_pending_mask;
static uint32 s_active_poll_count;
static uint32 s_active_start_cycles;
static uint32 s_transfer_timeout_cycles;
static uint8 s_snapshot_fresh_mask;                                              /* 尚未被快照消费的真实新结果掩码。 */
static uint8 s_snapshot_changed_mask;                                            /* 尚未被快照消费的结果状态变化掩码。 */
static uint32 s_log_seq;
static ipc_attitude_data_t s_attitude;
/* 两颗2BL3广播参数事务状态，仅由核1的100Hz主循环访问。 */
static camera_spi_param_transaction_t s_param_transaction;
/* 最近一次已被上层消费的成功SET回滚快照，用于处理迟到取消。 */
static camera_spi_param_rollback_cache_t s_param_rollback_cache;

static void camera_spi_write_u16_be(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value >> 8);
    buffer[1] = (uint8)(value & 0xFFU);
}

static uint16 camera_spi_read_u16_be(const uint8 *buffer)
{
    return (uint16)(((uint16)buffer[0] << 8) | buffer[1]);
}

static void camera_spi_write_u16_le(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)(value >> 8);
}

static uint16 camera_spi_read_u16_le(const uint8 *buffer)
{
    return (uint16)(((uint16)buffer[1] << 8) | buffer[0]);
}

static void camera_spi_write_u32_le(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
    buffer[2] = (uint8)((value >> 16) & 0xFFU);
    buffer[3] = (uint8)((value >> 24) & 0xFFU);
}

static uint32 camera_spi_read_u32_le(const uint8 *buffer)
{
    return ((uint32)buffer[0]) |
           ((uint32)buffer[1] << 8) |
           ((uint32)buffer[2] << 16) |
           ((uint32)buffer[3] << 24);
}

static void camera_spi_write_float_le(uint8 *buffer, float value)
{
    uint32 bits;

    memcpy(&bits, &value, sizeof(bits));
    camera_spi_write_u32_le(buffer, bits);
}

static uint16 camera_spi_crc16(const uint8 *data, uint16 len)
{
    uint16 crc = 0xFFFFU;
    uint16 i;
    uint8 j;

    for(i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for(j = 0U; j < 8U; j++)
        {
            if((crc & 0x0001U) != 0U)
            {
                crc = (uint16)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static void camera_spi_set_cs(uint8 board_id, uint8 selected)
{
    gpio_pin_enum pin = (board_id == 0U) ? CAMERA_SPI_CS0_PIN : CAMERA_SPI_CS1_PIN;

    if(selected != 0U)
    {
        gpio_low(pin);
    }
    else
    {
        gpio_high(pin);
    }
}

static uint8 camera_spi_ready_mask(void)
{
    uint8 mask = 0U;

    if(gpio_get_level(CAMERA_SPI_READY0_PIN) != 0U)
    {
        mask |= 0x01U;
    }
    if(gpio_get_level(CAMERA_SPI_READY1_PIN) != 0U)
    {
        mask |= 0x02U;
    }

    return mask;
}

static uint16 camera_spi_build_downlink_app(uint8 board_id, uint8 *app)
{
    camera_spi_board_state_t *board = &s_boards[board_id];
    uint8 board_mask = (uint8)(1U << board_id);
    uint32 transaction;
    uint32 value_bits;
    uint8 output_control = 0U;

    memset(app, 0, CAMERA_SPI_APP_DATA_CAPACITY);
    app[0] = CAMERA_SPI_DOWNLINK_MAGIC;
    app[1] = board_id;
    camera_spi_write_u32_le(&app[2], board->tx_counter++);
    app[6] = s_flight_state;
    if((s_image_send_enable == 2U) ||
       ((s_image_send_enable == 1U) && (s_flight_state == 0U)))
    {
        output_control |= CAMERA_SPI_OUTPUT_WIFI_TCP_ENABLE;
    }
    if(s_bl3_screen_enable != 0U)
    {
        output_control |= CAMERA_SPI_OUTPUT_SCREEN_ENABLE;
    }
    if(s_bl3_horizon_enable != 0U)
    {
        output_control |= CAMERA_SPI_OUTPUT_HORIZON_ENABLE;
    }
    app[7] = output_control;
    app[CAMERA_SPI_DOWNLINK_PARAM_WRITE_LOCK_OFFSET] = s_param_write_locked;
    app[CAMERA_SPI_ATTITUDE_MAGIC_OFFSET] = CAMERA_SPI_ATTITUDE_MAGIC;
    app[CAMERA_SPI_ATTITUDE_VERSION_OFFSET] = CAMERA_SPI_ATTITUDE_VERSION;
    camera_spi_write_u32_le(&app[CAMERA_SPI_ATTITUDE_SEQUENCE_OFFSET], s_attitude.sequence);
    camera_spi_write_float_le(&app[CAMERA_SPI_ATTITUDE_ROLL_OFFSET], s_attitude.roll_deg);
    camera_spi_write_float_le(&app[CAMERA_SPI_ATTITUDE_PITCH_OFFSET], s_attitude.pitch_deg);
    camera_spi_write_float_le(&app[CAMERA_SPI_ATTITUDE_HEIGHT_OFFSET], s_attitude.height_mm);
    app[CAMERA_SPI_ATTITUDE_FLAGS_OFFSET] =
        ((s_attitude.flags & IPC_ATTITUDE_FLAG_HEIGHT_VALID) != 0U) ?
        CAMERA_SPI_ATTITUDE_HEIGHT_VALID : 0U;

    if(((s_param_transaction.state == CAMERA_SPI_PARAM_PREFLIGHT) ||
        (s_param_transaction.state == CAMERA_SPI_PARAM_ACTIVE) ||
        (s_param_transaction.state == CAMERA_SPI_PARAM_ROLLBACK)) &&
       ((s_param_transaction.command_mask & board_mask) != 0U))
    {
        transaction = s_param_transaction.transaction;
        value_bits = s_param_transaction.value_bits;
        if(s_param_transaction.state == CAMERA_SPI_PARAM_PREFLIGHT)
        {
            transaction ^= CAMERA_SPI_PARAM_PREFLIGHT_TXN_MASK;
            value_bits = 0U;
        }
        else if(s_param_transaction.state == CAMERA_SPI_PARAM_ROLLBACK)
        {
            transaction |= CAMERA_SPI_PARAM_ROLLBACK_TXN_MASK;
            value_bits = s_param_transaction.previous_bits[board_id];
        }
        else if(s_param_transaction.op == IPC_REMOTE_PARAM_OP_GET)
        {
            if(s_param_transaction.get_mismatch_count == 1U)
            {
                transaction |= CAMERA_SPI_PARAM_PREFLIGHT_TXN_MASK;
            }
            else if(s_param_transaction.get_mismatch_count >= 2U)
            {
                transaction |= CAMERA_SPI_PARAM_ROLLBACK_TXN_MASK;
            }
        }
        else if(s_param_transaction.op == IPC_REMOTE_PARAM_OP_SET)
        {
            s_param_transaction.set_sent_mask |= board_mask;
        }

        app[9] = CAMERA_SPI_PARAM_MAGIC;
        app[10] = CAMERA_SPI_PARAM_VERSION;
        app[11] = (s_param_transaction.state == CAMERA_SPI_PARAM_PREFLIGHT) ?
                  IPC_REMOTE_PARAM_OP_GET :
                  ((s_param_transaction.state == CAMERA_SPI_PARAM_ROLLBACK) ?
                   IPC_REMOTE_PARAM_OP_SET : s_param_transaction.op);
        app[12] = s_param_transaction.type;
        camera_spi_write_u16_le(&app[13], s_param_transaction.param_id);
        app[15] = ((s_param_transaction.command_due_mask & board_mask) != 0U) ?
                  CAMERA_SPI_PARAM_ACK_REQUEST : 0U;
        camera_spi_write_u32_le(&app[16], transaction);
        camera_spi_write_u32_le(&app[20], value_bits);
        if((s_param_transaction.command_due_mask & board_mask) != 0U)
        {
            s_param_transaction.command_due_mask &= (uint8)(~board_mask);
        }
        return CAMERA_SPI_PARAM_APP_LEN;
    }

    return CAMERA_SPI_ATTITUDE_APP_LEN;
}

static void camera_spi_refresh_flight_state(void)
{
    uint8 board_id;
    uint8 flying = (ipc_core0_is_flying() != 0U) ? 1U : 0U;
    uint8 image_send_enable = ipc_core0_image_send_enable();
    uint8 bl3_screen_enable = ipc_core0_bl3_screen_enable();
    uint8 bl3_horizon_enable = ipc_core0_bl3_horizon_enable();
    uint8 param_write_locked =
        (ipc_core0_screen_refresh_enable() != 0U) ?
            CAMERA_SPI_PARAM_WRITE_ALLOWED : CAMERA_SPI_PARAM_WRITE_LOCKED;

    if(image_send_enable > 2U)
    {
        image_send_enable = 0U;
    }

    if((flying == s_flight_state) &&
       (image_send_enable == s_image_send_enable) &&
       (bl3_screen_enable == s_bl3_screen_enable) &&
       (bl3_horizon_enable == s_bl3_horizon_enable) &&
       (param_write_locked == s_param_write_locked))
    {
        return;
    }

    s_flight_state = flying;
    s_image_send_enable = image_send_enable;
    s_bl3_screen_enable = bl3_screen_enable;
    s_bl3_horizon_enable = bl3_horizon_enable;
    s_param_write_locked = param_write_locked;
    for(board_id = 0U; board_id < CAMERA_SPI_BOARD_COUNT; board_id++)
    {
        s_boards[board_id].tx_sequence++;
    }
}

static void camera_spi_build_request_frame(uint8 board_id)
{
    uint16 crc;
    uint16 app_len;
    uint8 *payload;
    camera_spi_board_state_t *board = &s_boards[board_id];

    memset(s_tx_frame, 0, sizeof(s_tx_frame));
    s_tx_frame[0] = CAMERA_SPI_FRAME_HEAD_0;
    s_tx_frame[1] = CAMERA_SPI_FRAME_HEAD_1;
    s_tx_frame[2] = CAMERA_SPI_CMD_SYNC_DATA;
    camera_spi_write_u16_be(&s_tx_frame[3], CAMERA_SPI_REQ_PAYLOAD_SIZE);

    payload = &s_tx_frame[5];
    camera_spi_write_u32_le(&payload[0], board->tx_sequence);
    app_len = camera_spi_build_downlink_app(board_id, &payload[6]);
    camera_spi_write_u16_le(&payload[4], app_len);

    crc = camera_spi_crc16(&s_tx_frame[2], (uint16)(3U + CAMERA_SPI_REQ_PAYLOAD_SIZE));
    s_tx_frame[5U + CAMERA_SPI_REQ_PAYLOAD_SIZE] = (uint8)(crc & 0xFFU);
    s_tx_frame[6U + CAMERA_SPI_REQ_PAYLOAD_SIZE] = (uint8)(crc >> 8);
    s_tx_frame[7U + CAMERA_SPI_REQ_PAYLOAD_SIZE] = CAMERA_SPI_FRAME_TAIL;
}

static void camera_spi_record_error(uint8 board_id, uint8 error)
{
    if(board_id < CAMERA_SPI_BOARD_COUNT)
    {
        s_boards[board_id].last_error = error;
        if(error != CAMERA_SPI_ERR_OK)
        {
            s_boards[board_id].rx_error_count++;
        }
    }
}

static void camera_spi_read_float(const uint8 *data, uint16 offset, float *value)
{
    memcpy(value, &data[offset], sizeof(float));
}

static void camera_spi_clear_board_targets(camera_spi_board_state_t *board)
{
    uint8 i;

    for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
    {
        image_data_clear_beacon(&board->beacons[i]);
    }
    for(i = 0U; i < CAMERA_SPI_MAX_CAR_LAMPS; i++)
    {
        image_data_clear_car_lamp(&board->car_lamps[i]);
    }
}

/* 清除失去新鲜度的图像目标，避免控制层继续使用旧数据。 */
static void camera_spi_invalidate_board_targets(camera_spi_board_state_t *board)
{
    if(board == NULL)
    {
        return;
    }

    board->beacon_count = 0U;
    board->car_lamp_count = 0U;
    camera_spi_clear_board_targets(board);
}

/* 每个固定采集周期更新链路和图像新鲜度，超时后只隔离对应图像板。 */
static void camera_spi_update_freshness(void)
{
    uint8 board_id;
    uint8 link_stale_cycles = (s_bl3_screen_enable != 0U) ?
                              CAMERA_SPI_SCREEN_LINK_STALE_CYCLES :
                              CAMERA_SPI_LINK_STALE_CYCLES;

    for(board_id = 0U; board_id < CAMERA_SPI_BOARD_COUNT; board_id++)
    {
        camera_spi_board_state_t *board = &s_boards[board_id];

        if(board->link_age_cycles < 0xFFU)
        {
            board->link_age_cycles++;
        }
        if(board->image_age_cycles < 0xFFU)
        {
            board->image_age_cycles++;
        }

        if(board->image_age_cycles == CAMERA_SPI_IMAGE_STALE_CYCLES)
        {
            camera_spi_invalidate_board_targets(board);
            if(board->result_sequence_initialized != 0U)
            {
                s_snapshot_changed_mask |=
                    (board_id == 0U) ? (uint8)(1U << Front) :
                                       (uint8)(1U << Back);
            }
        }
        if(board->link_age_cycles >= link_stale_cycles)
        {
            board->online = 0U;
            board->last_error = CAMERA_SPI_ERR_NOT_READY;
            board->result_sequence_initialized = 0U;
        }
    }
}

static void camera_spi_parse_image_payload(uint8 board_id, const uint8 *data)
{
    uint8 i;
    uint8 count;
    uint8 result_sequence;
    uint8 camera_mask;
    const uint8 *slot;
    camera_spi_board_state_t *board = &s_boards[board_id];

    board->version = data[CAMERA_SPI_IMAGE_VERSION_OFFSET];
    result_sequence = data[CAMERA_SPI_IMAGE_RESULT_SEQUENCE_OFFSET];
    if((board->result_sequence_initialized == 0U) &&
       (result_sequence == 0U))
    {
        return;
    }
    if(board->result_sequence_initialized != 0U)
    {
        if(result_sequence == board->result_sequence)
        {
            return;
        }
    }
    else
    {
        board->result_sequence_initialized = 1U;
    }

    board->result_sequence = result_sequence;
    board->image_age_cycles = 0U;
    camera_mask = (board_id == 0U) ? (uint8)(1U << Front) :
                                     (uint8)(1U << Back);
    s_snapshot_fresh_mask |= camera_mask;
    s_snapshot_changed_mask |= camera_mask;
    board->beacon_count = data[CAMERA_SPI_IMAGE_BEACON_COUNT_OFFSET];
    board->car_lamp_count = data[CAMERA_SPI_IMAGE_LAMP_COUNT_OFFSET];
    camera_spi_clear_board_targets(board);

    count = board->beacon_count;
    if(count > IMAGE_MAX_BEACON_COUNT) { count = IMAGE_MAX_BEACON_COUNT; }
    for(i = 0U; i < count; i++)
    {
        slot = &data[CAMERA_SPI_IMAGE_BEACON_PACKET_OFFSET + (uint16)i * CAMERA_SPI_IMAGE_BEACON_SLOT_SIZE];
        board->beacons[i].valid = slot[CAMERA_SPI_IMAGE_BEACON_VALID_OFFSET];
        camera_spi_read_float(slot, CAMERA_SPI_IMAGE_BEACON_X_OFFSET, &board->beacons[i].x);
        camera_spi_read_float(slot, CAMERA_SPI_IMAGE_BEACON_Y_OFFSET, &board->beacons[i].y);
        camera_spi_read_float(slot, CAMERA_SPI_IMAGE_BEACON_AREA_OFFSET, &board->beacons[i].area);
        if(image_data_beacon_valid(&board->beacons[i]) == 0U)
        {
            image_data_clear_beacon(&board->beacons[i]);
        }
    }

    count = board->car_lamp_count;
    if(count > CAMERA_SPI_MAX_CAR_LAMPS) { count = CAMERA_SPI_MAX_CAR_LAMPS; }
    for(i = 0U; i < count; i++)
    {
        slot = &data[CAMERA_SPI_IMAGE_LAMP_PACKET_OFFSET + (uint16)i * CAMERA_SPI_IMAGE_LAMP_SLOT_SIZE];
        board->car_lamps[i].valid = slot[CAMERA_SPI_IMAGE_LAMP_VALID_OFFSET];
        camera_spi_read_float(slot, CAMERA_SPI_IMAGE_LAMP_CX_OFFSET, &board->car_lamps[i].cx);
        camera_spi_read_float(slot, CAMERA_SPI_IMAGE_LAMP_CY_OFFSET, &board->car_lamps[i].cy);
        camera_spi_read_float(slot, CAMERA_SPI_IMAGE_LAMP_WIDTH_OFFSET, &board->car_lamps[i].width);
        camera_spi_read_float(slot, CAMERA_SPI_IMAGE_LAMP_LENGTH_OFFSET, &board->car_lamps[i].length);
        camera_spi_read_float(slot, CAMERA_SPI_IMAGE_LAMP_ANGLE_OFFSET, &board->car_lamps[i].angle);
        if(image_data_car_lamp_valid(&board->car_lamps[i]) == 0U)
        {
            image_data_clear_car_lamp(&board->car_lamps[i]);
        }
    }
}

/* 解析2BL3主循环应用参数后生成的20字节ACK。 */
static uint8 camera_spi_parse_param_ack(uint8 board_id, const uint8 *data)
{
    uint8 board_mask = (uint8)(1U << board_id);
    uint32 expected_transaction;
    uint32 transaction;
    uint8 expected_op;
    uint8 status;

    if((data[0] != CAMERA_SPI_PARAM_MAGIC) ||
       (data[1] != CAMERA_SPI_PARAM_ACK_MAGIC) ||
       (data[2] != CAMERA_SPI_PARAM_VERSION))
    {
        return CAMERA_SPI_ERR_APP_LEN;
    }

    transaction = camera_spi_read_u32_le(&data[12]);
    if((s_param_transaction.state != CAMERA_SPI_PARAM_PREFLIGHT) &&
       (s_param_transaction.state != CAMERA_SPI_PARAM_ACTIVE) &&
       (s_param_transaction.state != CAMERA_SPI_PARAM_ROLLBACK))
    {
        return CAMERA_SPI_ERR_OK;
    }

    expected_transaction = s_param_transaction.transaction;
    expected_op = s_param_transaction.op;
    if(s_param_transaction.state == CAMERA_SPI_PARAM_PREFLIGHT)
    {
        expected_transaction ^= CAMERA_SPI_PARAM_PREFLIGHT_TXN_MASK;
        expected_op = IPC_REMOTE_PARAM_OP_GET;
    }
    else if(s_param_transaction.state == CAMERA_SPI_PARAM_ROLLBACK)
    {
        expected_transaction |= CAMERA_SPI_PARAM_ROLLBACK_TXN_MASK;
        expected_op = IPC_REMOTE_PARAM_OP_SET;
    }
    else if(s_param_transaction.op == IPC_REMOTE_PARAM_OP_GET)
    {
        if(s_param_transaction.get_mismatch_count == 1U)
        {
            expected_transaction |= CAMERA_SPI_PARAM_PREFLIGHT_TXN_MASK;
        }
        else if(s_param_transaction.get_mismatch_count >= 2U)
        {
            expected_transaction |= CAMERA_SPI_PARAM_ROLLBACK_TXN_MASK;
        }
    }
    if(transaction != expected_transaction)
    {
        return CAMERA_SPI_ERR_OK;
    }

    if((data[3] != expected_op) ||
       (data[5] != s_param_transaction.type) ||
       (camera_spi_read_u16_le(&data[6]) != s_param_transaction.param_id) ||
       (data[8] != board_id) ||
       ((s_param_transaction.command_mask & board_mask) == 0U))
    {
        s_param_transaction.board_status[board_id] = IPC_REMOTE_PARAM_STATUS_ERROR;
        s_param_transaction.actual_bits[board_id] = camera_spi_read_u32_le(&data[16]);
        s_param_transaction.ack_mask |= board_mask;
        return CAMERA_SPI_ERR_OK;
    }

    status = data[4];
    if(status > IPC_REMOTE_PARAM_STATUS_ROLLBACK_FAIL)
    {
        status = IPC_REMOTE_PARAM_STATUS_ERROR;
    }
    if(status == IPC_REMOTE_PARAM_STATUS_ROLLBACK_FAIL)
    {
        s_param_transaction.rollback_fail_seen = 1U;
    }
    s_param_transaction.board_status[board_id] = status;
    s_param_transaction.actual_bits[board_id] = camera_spi_read_u32_le(&data[16]);
    s_param_transaction.ack_mask |= board_mask;
    return CAMERA_SPI_ERR_OK;
}

static uint8 camera_spi_parse_response(uint8 board_id)
{
    uint16 payload_len;
    uint16 app_len;
    uint16 crc_calc;
    uint16 crc_recv;
    const uint8 *payload;
    camera_spi_board_state_t *board;

    if(board_id >= CAMERA_SPI_BOARD_COUNT)
    {
        return CAMERA_SPI_ERR_INVALID_BOARD;
    }

    if((s_rx_frame[0] != CAMERA_SPI_FRAME_HEAD_0) ||
       (s_rx_frame[1] != CAMERA_SPI_FRAME_HEAD_1))
    {
        return CAMERA_SPI_ERR_HEAD;
    }
    if(s_rx_frame[2] != CAMERA_SPI_CMD_SYNC_DATA)
    {
        return CAMERA_SPI_ERR_CMD;
    }

    payload_len = camera_spi_read_u16_be(&s_rx_frame[3]);
    if(payload_len != CAMERA_SPI_RESP_PAYLOAD_SIZE)
    {
        return CAMERA_SPI_ERR_LEN;
    }
    if(s_rx_frame[7U + payload_len] != CAMERA_SPI_FRAME_TAIL)
    {
        return CAMERA_SPI_ERR_TAIL;
    }

    crc_calc = camera_spi_crc16(&s_rx_frame[2], (uint16)(3U + payload_len));
    crc_recv = (uint16)s_rx_frame[5U + payload_len] |
               ((uint16)s_rx_frame[6U + payload_len] << 8);
    if(crc_calc != crc_recv)
    {
        return CAMERA_SPI_ERR_CRC;
    }

    payload = &s_rx_frame[5];
    board = &s_boards[board_id];
    board->rx_sequence = camera_spi_read_u32_le(&payload[0]);
    board->ack_sequence = camera_spi_read_u32_le(&payload[4]);
    board->flags = payload[10];
    board->peer_last_error = payload[11];
    app_len = camera_spi_read_u16_le(&payload[8]);
    if(app_len == CAMERA_SPI_PARAM_ACK_APP_LEN)
    {
        uint8 ack_error = camera_spi_parse_param_ack(board_id, &payload[12]);
        if(ack_error != CAMERA_SPI_ERR_OK)
        {
            return ack_error;
        }
    }
    else if(app_len == CAMERA_SPI_APP_DATA_CAPACITY)
    {
        if(payload[12U + CAMERA_SPI_IMAGE_VERSION_OFFSET] !=
           CAMERA_SPI_IMAGE_PROTOCOL_VERSION)
        {
            return CAMERA_SPI_ERR_APP_LEN;
        }
        camera_spi_parse_image_payload(board_id, &payload[12]);
    }
    else
    {
        return CAMERA_SPI_ERR_APP_LEN;
    }
    board->online = 1U;
    board->link_age_cycles = 0U;
    board->last_error = CAMERA_SPI_ERR_OK;
    board->rx_ok_count++;

    return CAMERA_SPI_ERR_OK;
}

/* 将两板参数事务置为完成态，等待核1 IPC层读取。 */
static void camera_spi_param_complete(uint8 status, uint32 actual_bits)
{
    s_param_transaction.final_status =
        (s_param_transaction.rollback_fail_seen != 0U) ?
            IPC_REMOTE_PARAM_STATUS_ROLLBACK_FAIL : status;
    s_param_transaction.actual_bits[0] = actual_bits;
    s_param_transaction.state = CAMERA_SPI_PARAM_COMPLETE;
}

/* 进入回滚态，向已成功或尚未确认写入结果的板发送各自真实旧值。 */
static void camera_spi_param_start_rollback(uint8 board_mask)
{
    s_param_transaction.state = CAMERA_SPI_PARAM_ROLLBACK;
    s_param_transaction.command_mask = board_mask;
    s_param_transaction.command_due_mask = board_mask;
    s_param_transaction.ack_mask = 0U;
    s_param_transaction.cycle_count = 0U;
    s_param_transaction.board_status[0] = IPC_REMOTE_PARAM_STATUS_TIMEOUT;
    s_param_transaction.board_status[1] = IPC_REMOTE_PARAM_STATUS_TIMEOUT;
}

/* 每轮两板SPI结束后汇总ACK；普通参数等待12周期，曝光和持久化命令等待80周期。 */
/* 未收到ACK的板每轮都请求ACK，避免从机主循环错过首个请求帧。 */
static void camera_spi_param_schedule_next(void)
{
    uint8 pending_mask;

    if((s_param_transaction.state != CAMERA_SPI_PARAM_PREFLIGHT) &&
       (s_param_transaction.state != CAMERA_SPI_PARAM_ACTIVE) &&
       (s_param_transaction.state != CAMERA_SPI_PARAM_ROLLBACK))
    {
        return;
    }

    pending_mask = s_param_transaction.command_mask &
                   (uint8)(~s_param_transaction.ack_mask);
    s_param_transaction.command_due_mask = pending_mask;
}

static void camera_spi_param_evaluate(void)
{
    uint8 required_mask;
    uint8 success_mask = 0U;
    uint8 board_id;
    uint8 max_cycles;

    if((s_param_transaction.state != CAMERA_SPI_PARAM_PREFLIGHT) &&
       (s_param_transaction.state != CAMERA_SPI_PARAM_ACTIVE) &&
       (s_param_transaction.state != CAMERA_SPI_PARAM_ROLLBACK))
    {
        return;
    }

    required_mask = s_param_transaction.command_mask;
    max_cycles = (s_param_transaction.param_id == IPC_REMOTE_PARAM_ID_BL3_EXP_TIME) ?
                 CAMERA_SPI_PARAM_EXP_MAX_CYCLES :
                 ((s_bl3_screen_enable != 0U) ?
                  CAMERA_SPI_SCREEN_PARAM_MAX_CYCLES : CAMERA_SPI_PARAM_MAX_CYCLES);
    for(board_id = 0U; board_id < CAMERA_SPI_BOARD_COUNT; board_id++)
    {
        uint8 board_mask = (uint8)(1U << board_id);
        if(((required_mask & board_mask) != 0U) &&
           ((s_param_transaction.ack_mask & board_mask) != 0U) &&
           (s_param_transaction.board_status[board_id] == IPC_REMOTE_PARAM_STATUS_OK))
        {
            success_mask |= board_mask;
        }
    }

    if((s_param_transaction.ack_mask & required_mask) == required_mask)
    {
        if(s_param_transaction.state == CAMERA_SPI_PARAM_ROLLBACK)
        {
            uint8 rollback_ok = (success_mask == required_mask) ? 1U : 0U;
            for(board_id = 0U; board_id < CAMERA_SPI_BOARD_COUNT; board_id++)
            {
                uint8 board_mask = (uint8)(1U << board_id);
                if(((required_mask & board_mask) != 0U) &&
                   (s_param_transaction.actual_bits[board_id] !=
                    s_param_transaction.previous_bits[board_id]))
                {
                    rollback_ok = 0U;
                }
            }
            camera_spi_param_complete((rollback_ok != 0U) ?
                                      ((s_param_transaction.cancel_requested != 0U) ?
                                       IPC_REMOTE_PARAM_STATUS_TIMEOUT :
                                       IPC_REMOTE_PARAM_STATUS_PARTIAL) :
                                      IPC_REMOTE_PARAM_STATUS_ROLLBACK_FAIL,
                                      s_param_transaction.fallback_bits);
            return;
        }

        if(s_param_transaction.state == CAMERA_SPI_PARAM_PREFLIGHT)
        {
            if(success_mask == 0x03U)
            {
                s_param_transaction.previous_bits[0] = s_param_transaction.actual_bits[0];
                s_param_transaction.previous_bits[1] = s_param_transaction.actual_bits[1];
                s_param_transaction.state = CAMERA_SPI_PARAM_ACTIVE;
                s_param_transaction.command_mask = 0x03U;
                s_param_transaction.command_due_mask = 0x03U;
                s_param_transaction.set_sent_mask = 0U;
                s_param_transaction.ack_mask = 0U;
                s_param_transaction.cycle_count = 0U;
                s_param_transaction.board_status[0] = IPC_REMOTE_PARAM_STATUS_TIMEOUT;
                s_param_transaction.board_status[1] = IPC_REMOTE_PARAM_STATUS_TIMEOUT;
            }
            else
            {
                uint8 status = (s_param_transaction.board_status[0] ==
                                s_param_transaction.board_status[1]) ?
                               s_param_transaction.board_status[0] :
                               IPC_REMOTE_PARAM_STATUS_ERROR;
                camera_spi_param_complete(status, s_param_transaction.fallback_bits);
            }
            return;
        }

        if(s_param_transaction.op == IPC_REMOTE_PARAM_OP_GET)
        {
            if(success_mask != 0x03U)
            {
                uint8 status = (s_param_transaction.board_status[0] ==
                                s_param_transaction.board_status[1]) ?
                               s_param_transaction.board_status[0] :
                               IPC_REMOTE_PARAM_STATUS_ERROR;
                camera_spi_param_complete(status, s_param_transaction.fallback_bits);
            }
            else if(s_param_transaction.actual_bits[0] != s_param_transaction.actual_bits[1])
            {
                if(s_param_transaction.get_mismatch_count == 0U)
                {
                    s_param_transaction.get_mismatch_reference[0] =
                        s_param_transaction.actual_bits[0];
                    s_param_transaction.get_mismatch_reference[1] =
                        s_param_transaction.actual_bits[1];
                    s_param_transaction.get_mismatch_stable = 1U;
                }
                else if((s_param_transaction.actual_bits[0] !=
                         s_param_transaction.get_mismatch_reference[0]) ||
                        (s_param_transaction.actual_bits[1] !=
                         s_param_transaction.get_mismatch_reference[1]))
                {
                    s_param_transaction.get_mismatch_stable = 0U;
                }

                s_param_transaction.get_mismatch_count++;
                if(s_param_transaction.get_mismatch_count <
                   CAMERA_SPI_PARAM_GET_MISMATCH_ATTEMPTS)
                {
                    s_param_transaction.command_due_mask = 0x03U;
                    s_param_transaction.ack_mask = 0U;
                    s_param_transaction.cycle_count = 0U;
                    s_param_transaction.board_status[0] = IPC_REMOTE_PARAM_STATUS_TIMEOUT;
                    s_param_transaction.board_status[1] = IPC_REMOTE_PARAM_STATUS_TIMEOUT;
                }
                else
                {
                    camera_spi_param_complete(
                        (s_param_transaction.get_mismatch_stable != 0U) ?
                            IPC_REMOTE_PARAM_STATUS_MISMATCH :
                            IPC_REMOTE_PARAM_STATUS_ERROR,
                        s_param_transaction.fallback_bits);
                }
            }
            else
            {
                camera_spi_param_complete(IPC_REMOTE_PARAM_STATUS_OK,
                                          s_param_transaction.actual_bits[0]);
            }
            return;
        }

        if((success_mask == 0x03U) &&
           (s_param_transaction.actual_bits[0] == s_param_transaction.actual_bits[1]) &&
           (s_param_transaction.actual_bits[0] == s_param_transaction.value_bits))
        {
            camera_spi_param_complete(IPC_REMOTE_PARAM_STATUS_OK,
                                      s_param_transaction.actual_bits[0]);
        }
        else if(success_mask != 0U)
        {
            camera_spi_param_start_rollback(success_mask);
        }
        else
        {
            uint8 status = (s_param_transaction.board_status[0] ==
                            s_param_transaction.board_status[1]) ?
                           s_param_transaction.board_status[0] :
                           IPC_REMOTE_PARAM_STATUS_ERROR;
            camera_spi_param_complete(status, s_param_transaction.fallback_bits);
        }
        return;
    }

    s_param_transaction.cycle_count++;
    if(s_param_transaction.cycle_count < max_cycles)
    {
        return;
    }

    if(s_param_transaction.state == CAMERA_SPI_PARAM_PREFLIGHT)
    {
        camera_spi_param_complete(IPC_REMOTE_PARAM_STATUS_TIMEOUT,
                                  s_param_transaction.fallback_bits);
    }
    else if(s_param_transaction.state == CAMERA_SPI_PARAM_ROLLBACK)
    {
        camera_spi_param_complete(IPC_REMOTE_PARAM_STATUS_ROLLBACK_FAIL,
                                  s_param_transaction.fallback_bits);
    }
    else if(s_param_transaction.op == IPC_REMOTE_PARAM_OP_SET)
    {
        uint8 possible_write_mask = success_mask |
            (s_param_transaction.command_mask &
             (uint8)(~s_param_transaction.ack_mask));
        if(possible_write_mask != 0U)
        {
            camera_spi_param_start_rollback(possible_write_mask);
        }
        else
        {
            camera_spi_param_complete(IPC_REMOTE_PARAM_STATUS_TIMEOUT,
                                      s_param_transaction.fallback_bits);
        }
    }
    else
    {
        camera_spi_param_complete(IPC_REMOTE_PARAM_STATUS_TIMEOUT,
                                  s_param_transaction.fallback_bits);
    }
}

static void camera_spi_irq_handler(void)
{
    Cy_SCB_SPI_Interrupt(CAMERA_SPI_SCB, &s_spi_context);
}

static void camera_spi_init_irq(void)
{
    cy_stc_sysint_irq_t irq_cfg;

    irq_cfg.sysIntSrc = CAMERA_SPI_IRQ_SRC;
    irq_cfg.intIdx = CAMERA_SPI_CPU_IRQ;
    irq_cfg.isEnabled = true;
    interrupt_init(&irq_cfg, camera_spi_irq_handler, 6U);
}

static void camera_spi_init_pins(void)
{
    cy_stc_gpio_pin_config_t pin_cfg = {0};

    pin_cfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    pin_cfg.hsiom = P3_1_SCB6_SPI_MOSI;
    Cy_GPIO_Pin_Init(get_port(P03_1), (P03_1 % 8), &pin_cfg);

    pin_cfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    pin_cfg.hsiom = P3_2_SCB6_SPI_CLK;
    Cy_GPIO_Pin_Init(get_port(P03_2), (P03_2 % 8), &pin_cfg);

    pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;
    pin_cfg.hsiom = P3_0_SCB6_SPI_MISO;
    Cy_GPIO_Pin_Init(get_port(P03_0), (P03_0 % 8), &pin_cfg);

    gpio_init(CAMERA_SPI_CS0_PIN, GPO, 1U, GPO_PUSH_PULL);
    gpio_init(CAMERA_SPI_CS1_PIN, GPO, 1U, GPO_PUSH_PULL);
    gpio_init(CAMERA_SPI_READY0_PIN, GPI, 0U, GPI_PULL_DOWN);
    gpio_init(CAMERA_SPI_READY1_PIN, GPI, 0U, GPI_PULL_DOWN);
}

static void camera_spi_init_scb(void)
{
    uint32 target_freq = CAMERA_SPI_CLOCK_OVERSAMPLE * CAMERA_SPI_BAUDRATE;
    uint32 div_int = CAMERA_SPI_PERI_FREQ / target_freq;
    uint32 div_frac = (uint32)((double)(CAMERA_SPI_PERI_FREQ - (div_int * target_freq)) /
                               (double)target_freq * 32.0);
    cy_stc_scb_spi_config_t spi_cfg = {0};

    Cy_SysClk_PeriphAssignDivider(PCLK_SCB6_CLOCK, CY_SYSCLK_DIV_24_5_BIT, 10U);
    Cy_SysClk_PeriphSetFracDivider(Cy_SysClk_GetClockGroup(PCLK_SCB6_CLOCK),
                                   CY_SYSCLK_DIV_24_5_BIT,
                                   10U,
                                   (div_int - 1U),
                                   div_frac);
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(PCLK_SCB6_CLOCK),
                                  CY_SYSCLK_DIV_24_5_BIT,
                                  10U);

    spi_cfg.spiMode = CY_SCB_SPI_MASTER;
    spi_cfg.subMode = CY_SCB_SPI_MOTOROLA;
    spi_cfg.sclkMode = CY_SCB_SPI_CPHA0_CPOL0;
    spi_cfg.oversample = CAMERA_SPI_CLOCK_OVERSAMPLE;
    spi_cfg.rxDataWidth = 8U;
    spi_cfg.txDataWidth = 8U;
    spi_cfg.enableMsbFirst = true;
    spi_cfg.enableMisoLateSample = true;

    Cy_SCB_SPI_DeInit(CAMERA_SPI_SCB);
    (void)Cy_SCB_SPI_Init(CAMERA_SPI_SCB, &spi_cfg, &s_spi_context);
    Cy_SCB_SPI_SetActiveSlaveSelect(CAMERA_SPI_SCB, CY_SCB_SPI_SLAVE_SELECT0);
    Cy_SCB_SPI_Enable(CAMERA_SPI_SCB);
}

static void camera_spi_start_transfer(uint8 board_id)
{
    cy_en_scb_spi_status_t status;
    uint32 transfer_start_cycles;

    if((board_id >= CAMERA_SPI_BOARD_COUNT) || (s_active != 0U))
    {
        camera_spi_record_error(board_id, CAMERA_SPI_ERR_TRANSFER_BUSY);
        return;
    }

    camera_spi_build_request_frame(board_id);
    memset(s_rx_frame, 0, sizeof(s_rx_frame));
    Cy_SCB_SPI_ClearRxFifo(CAMERA_SPI_SCB);
    Cy_SCB_SPI_ClearTxFifo(CAMERA_SPI_SCB);
    transfer_start_cycles = DWT->CYCCNT;
    camera_spi_set_cs(board_id, 1U);
    status = Cy_SCB_SPI_Transfer(CAMERA_SPI_SCB,
                                 s_tx_frame,
                                 s_rx_frame,
                                 CAMERA_SPI_TRANSFER_LEN,
                                 &s_spi_context);
    if(status == CY_SCB_SPI_SUCCESS)
    {
        s_active = 1U;
        s_active_board = board_id;
        s_active_poll_count = 0U;
        s_active_start_cycles = transfer_start_cycles;
    }
    else
    {
        camera_spi_set_cs(board_id, 0U);
        camera_spi_record_error(board_id, CAMERA_SPI_ERR_HW);
    }
}

/* 中止当前传输并复位SCB，单板异常不得阻塞核心1主循环。 */
static void camera_spi_abort_active(uint8 error)
{
    if(s_active == 0U)
    {
        return;
    }

    camera_spi_set_cs(s_active_board, 0U);
    Cy_SCB_SPI_AbortTransfer(CAMERA_SPI_SCB, &s_spi_context);
    Cy_SCB_SPI_Disable(CAMERA_SPI_SCB, &s_spi_context);
    Cy_SCB_SPI_Enable(CAMERA_SPI_SCB);
    camera_spi_record_error(s_active_board, error);
    s_active_poll_count = 0U;
    s_active = 0U;
}

/* 使用DWT真实时间和轮询次数双重判断传输超时。 */
static uint8 camera_spi_active_timed_out(void)
{
    uint32 elapsed_cycles = DWT->CYCCNT - s_active_start_cycles;

    s_active_poll_count++;
    if((elapsed_cycles >= s_transfer_timeout_cycles) ||
       (s_active_poll_count > CAMERA_SPI_TRANSFER_TIMEOUT_POLLS))
    {
        return 1U;
    }

    return 0U;
}

/* 推进当前异步传输；未完成时立即返回，不在100Hz任务内忙等。 */
static void camera_spi_finish_active(void)
{
    uint32 status;
    uint8 error;

    if(s_active == 0U)
    {
        return;
    }

    status = Cy_SCB_SPI_GetTransferStatus(CAMERA_SPI_SCB, &s_spi_context);
    if((status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0U)
    {
        if(camera_spi_active_timed_out() != 0U)
        {
            camera_spi_abort_active(CAMERA_SPI_ERR_TIMEOUT);
        }
        return;
    }

    if(Cy_SCB_SPI_IsBusBusy(CAMERA_SPI_SCB))
    {
        if(camera_spi_active_timed_out() != 0U)
        {
            camera_spi_abort_active(CAMERA_SPI_ERR_TIMEOUT);
        }
        return;
    }

    camera_spi_set_cs(s_active_board, 0U);
    s_boards[s_active_board].last_rx_head0 = s_rx_frame[0];
    s_boards[s_active_board].last_rx_head1 = s_rx_frame[1];
    error = camera_spi_parse_response(s_active_board);
    camera_spi_record_error(s_active_board, error);
    s_active_poll_count = 0U;
    s_active = 0U;
}

static void camera_spi_publish_log(void)
{
    ipc_camera_spi_log_t log;
    uint8 board_id;

    memset(&log, 0, sizeof(log));
    log.seq = ++s_log_seq;
    for(board_id = 0U; board_id < CAMERA_SPI_BOARD_COUNT; board_id++)
    {
        const camera_spi_board_state_t *state = &s_boards[board_id];
        ipc_camera_spi_board_log_t *blog = &log.board[board_id];

        blog->online = state->online;
        blog->beacon_count = state->beacon_count;
        blog->car_lamp_count = state->car_lamp_count;
        blog->first_beacon_valid = state->beacons[0].valid;
        blog->first_lamp_valid = state->car_lamps[0].valid;
        blog->last_error = state->last_error;
        blog->last_rx_head0 = state->last_rx_head0;
        blog->last_rx_head1 = state->last_rx_head1;
        blog->rx_ok_count = state->rx_ok_count;
        blog->rx_error_count = state->rx_error_count;
        blog->first_beacon_x = state->beacons[0].x;
        blog->first_beacon_y = state->beacons[0].y;
        blog->first_beacon_radius = state->beacons[0].area;
        blog->first_lamp_cx = state->car_lamps[0].cx;
        blog->first_lamp_cy = state->car_lamps[0].cy;
        blog->first_lamp_angle = state->car_lamps[0].angle;
    }

    ipc_camera_spi_log_publish(&log);
}

/* 完成一轮两板采集并统一推进参数事务及状态发布。 */
static void camera_spi_complete_cycle(void)
{
    s_cycle_active = 0U;
    camera_spi_param_schedule_next();
    camera_spi_param_evaluate();
    camera_spi_publish_log();
}

/*
 * 非阻塞推进Camera SPI状态机。
 * 返回1表示仍有硬件传输待完成，返回0表示当前轮次已经收敛。
 */
uint8 CameraSpi_Service(void)
{
    uint8 board_id;

    if(s_initialized == 0U)
    {
        return 0U;
    }

    camera_spi_finish_active();
    if(s_active != 0U)
    {
        return 1U;
    }

    while((s_cycle_active != 0U) && (s_cycle_pending_mask != 0U))
    {
        for(board_id = 0U; board_id < CAMERA_SPI_BOARD_COUNT; board_id++)
        {
            uint8 board_mask = (uint8)(1U << board_id);

            if((s_cycle_pending_mask & board_mask) == 0U)
            {
                continue;
            }

            s_cycle_pending_mask &= (uint8)(~board_mask);
            if((camera_spi_ready_mask() & board_mask) == 0U)
            {
                break;
            }

            camera_spi_start_transfer(board_id);
            break;
        }

        if(s_active != 0U)
        {
            return 1U;
        }
    }

    if(s_cycle_active != 0U)
    {
        camera_spi_complete_cycle();
    }

    return 0U;
}

void CameraSpi_Init(void)
{
    uint8 board_id;

    memset(s_boards, 0, sizeof(s_boards));
    memset(s_tx_frame, 0, sizeof(s_tx_frame));
    memset(s_rx_frame, 0, sizeof(s_rx_frame));
    memset(&s_spi_context, 0, sizeof(s_spi_context));
    memset(&s_param_transaction, 0, sizeof(s_param_transaction));
    memset(&s_param_rollback_cache, 0, sizeof(s_param_rollback_cache));
    s_initialized = 0U;
    s_active = 0U;
    s_active_board = 0U;
    s_flight_state = (ipc_core0_is_flying() != 0U) ? 1U : 0U;
    s_param_write_locked =
        (ipc_core0_screen_refresh_enable() != 0U) ?
            CAMERA_SPI_PARAM_WRITE_ALLOWED : CAMERA_SPI_PARAM_WRITE_LOCKED;
    s_image_send_enable = ipc_core0_image_send_enable();
    if(s_image_send_enable > 2U)
    {
        s_image_send_enable = 0U;
    }
    s_bl3_screen_enable = ipc_core0_bl3_screen_enable();
    s_bl3_horizon_enable = ipc_core0_bl3_horizon_enable();
    s_ready_mask = 0U;
    s_cycle_active = 0U;
    s_cycle_pending_mask = 0U;
    s_active_poll_count = 0U;
    s_active_start_cycles = 0U;
    s_transfer_timeout_cycles = 0U;
    s_snapshot_fresh_mask = 0U;
    s_snapshot_changed_mask = 0U;
    s_log_seq = 0U;

    for(board_id = 0U; board_id < CAMERA_SPI_BOARD_COUNT; board_id++)
    {
        camera_spi_clear_board_targets(&s_boards[board_id]);
        s_boards[board_id].tx_sequence = 1U;
    }

    camera_spi_init_pins();
    camera_spi_init_scb();
    camera_spi_init_irq();

    /* 使用DWT周期计数器提供不依赖主循环速度的SPI硬超时。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = CAMERA_SPI_DWT_UNLOCK_KEY;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_transfer_timeout_cycles =
        (SystemCoreClock / 1000000U) * CAMERA_SPI_TRANSFER_TIMEOUT_US;
    if(s_transfer_timeout_cycles == 0U)
    {
        s_transfer_timeout_cycles = 1U;
    }

    s_initialized = 1U;
}

void CameraSpi_Update(void)
{
    if(s_initialized == 0U)
    {
        return;
    }

    (void)CameraSpi_Service();
    if((s_active != 0U) || (s_cycle_active != 0U))
    {
        return;
    }

    camera_spi_update_freshness();
    ipc_attitude_get(&s_attitude);
    camera_spi_refresh_flight_state();
    s_ready_mask = camera_spi_ready_mask();
    s_cycle_pending_mask = s_ready_mask & CAMERA_SPI_ALL_BOARD_MASK;
    s_cycle_active = 1U;
    (void)CameraSpi_Service();
}

/*
 * 函数功能: 将两块图像板的最新结果复制到三摄数组，并消费本轮结果变化标志。
 * 输入参数: camera为三摄结果数组；fresh_mask输出真实新算法结果对应的摄像头位掩码。
 * 返回值: 本轮结果内容发生变化的摄像头位掩码，包含新结果和超时清空。
 */
uint8 CameraSpi_GetSnapshot(struct image_data camera[IMAGE_CAMERA_COUNT],
                            uint8 *fresh_mask)
{
    uint8 board_id;
    uint8 changed_mask;

    if(camera == NULL)
    {
        if(fresh_mask != NULL)
        {
            *fresh_mask = 0U;
        }
        return 0U;
    }

    changed_mask = s_snapshot_changed_mask;
    if(fresh_mask != NULL)
    {
        *fresh_mask = s_snapshot_fresh_mask;
    }

    for(board_id = 0U; board_id < CAMERA_SPI_BOARD_COUNT; board_id++)
    {
        const camera_spi_board_state_t *state = &s_boards[board_id];
        const image_camera_e camera_id = (board_id == 0U) ? Front : Back;

        image_data_clear(&camera[camera_id]);
        memcpy(camera[camera_id].beacon_data, state->beacons, sizeof(state->beacons));
        memcpy(camera[camera_id].car_lamp_data, state->car_lamps, sizeof(state->car_lamps));
    }
    s_snapshot_fresh_mask = 0U;
    s_snapshot_changed_mask = 0U;
    return changed_mask;
}

/* 启动同时发往两颗2BL3的SET/GET参数事务。 */
uint8 CameraSpi_RemoteParamStart(uint8 op,
                                 uint8 type,
                                 uint16 param_id,
                                 uint32 transaction,
                                 uint32 value_bits,
                                 uint32 previous_bits)
{
    if((s_initialized == 0U) ||
       (s_param_transaction.state != CAMERA_SPI_PARAM_IDLE) ||
       ((op != IPC_REMOTE_PARAM_OP_SET) && (op != IPC_REMOTE_PARAM_OP_GET)) ||
       (type > IPC_REMOTE_PARAM_TYPE_INT32) ||
       (param_id == 0U) ||
       (transaction == 0U) ||
       ((transaction & CAMERA_SPI_PARAM_ORIGINAL_TXN_MASK) != 0U))
    {
        return 0U;
    }

    s_param_rollback_cache.valid = 0U;
    memset(&s_param_transaction, 0, sizeof(s_param_transaction));
    s_param_transaction.state = (op == IPC_REMOTE_PARAM_OP_SET) ?
                                CAMERA_SPI_PARAM_PREFLIGHT : CAMERA_SPI_PARAM_ACTIVE;
    s_param_transaction.op = op;
    s_param_transaction.type = type;
    s_param_transaction.command_mask = 0x03U;
    s_param_transaction.command_due_mask = 0x03U;
    s_param_transaction.param_id = param_id;
    s_param_transaction.transaction = transaction;
    s_param_transaction.value_bits = value_bits;
    s_param_transaction.fallback_bits = previous_bits;
    s_param_transaction.previous_bits[0] = previous_bits;
    s_param_transaction.previous_bits[1] = previous_bits;
    s_param_transaction.board_status[0] = IPC_REMOTE_PARAM_STATUS_TIMEOUT;
    s_param_transaction.board_status[1] = IPC_REMOTE_PARAM_STATUS_TIMEOUT;
    return 1U;
}

/* 读取并消费两板参数事务最终结果。 */
uint8 CameraSpi_RemoteParamTakeResult(camera_spi_remote_param_result_t *result)
{
    if((result == NULL) || (s_param_transaction.state != CAMERA_SPI_PARAM_COMPLETE))
    {
        return 0U;
    }

    result->transaction = s_param_transaction.transaction;
    result->actual_bits = s_param_transaction.actual_bits[0];
    result->param_id = s_param_transaction.param_id;
    result->op = s_param_transaction.op;
    result->type = s_param_transaction.type;
    result->status = s_param_transaction.final_status;
    if((s_param_transaction.op == IPC_REMOTE_PARAM_OP_SET) &&
       (s_param_transaction.final_status == IPC_REMOTE_PARAM_STATUS_OK))
    {
        s_param_rollback_cache.valid = 1U;
        s_param_rollback_cache.type = s_param_transaction.type;
        s_param_rollback_cache.param_id = s_param_transaction.param_id;
        s_param_rollback_cache.transaction = s_param_transaction.transaction;
        s_param_rollback_cache.previous_bits[0] = s_param_transaction.previous_bits[0];
        s_param_rollback_cache.previous_bits[1] = s_param_transaction.previous_bits[1];
    }
    memset(&s_param_transaction, 0, sizeof(s_param_transaction));
    return 1U;
}

/* 取消指定事务；已经下发SET时必须先恢复两板各自的真实旧值。 */
uint8 CameraSpi_RemoteParamCancel(uint32 transaction)
{
    uint8 rollback_mask;

    if(transaction == 0U)
    {
        return 0U;
    }

    if(s_param_transaction.state == CAMERA_SPI_PARAM_IDLE)
    {
        if((s_param_rollback_cache.valid == 0U) ||
           (s_param_rollback_cache.transaction != transaction))
        {
            return 0U;
        }

        memset(&s_param_transaction, 0, sizeof(s_param_transaction));
        s_param_transaction.op = IPC_REMOTE_PARAM_OP_SET;
        s_param_transaction.type = s_param_rollback_cache.type;
        s_param_transaction.param_id = s_param_rollback_cache.param_id;
        s_param_transaction.transaction = s_param_rollback_cache.transaction;
        s_param_transaction.fallback_bits = s_param_rollback_cache.previous_bits[0];
        s_param_transaction.previous_bits[0] = s_param_rollback_cache.previous_bits[0];
        s_param_transaction.previous_bits[1] = s_param_rollback_cache.previous_bits[1];
        s_param_transaction.cancel_requested = 1U;
        s_param_rollback_cache.valid = 0U;
        camera_spi_param_start_rollback(0x03U);
        return 1U;
    }

    if(s_param_transaction.transaction != transaction)
    {
        return 0U;
    }

    if(s_param_transaction.state == CAMERA_SPI_PARAM_COMPLETE)
    {
        if((s_param_transaction.op == IPC_REMOTE_PARAM_OP_SET) &&
           (s_param_transaction.final_status == IPC_REMOTE_PARAM_STATUS_OK))
        {
            s_param_transaction.cancel_requested = 1U;
            camera_spi_param_start_rollback(0x03U);
        }
        return 1U;
    }

    if((s_param_transaction.state == CAMERA_SPI_PARAM_PREFLIGHT) ||
       ((s_param_transaction.state == CAMERA_SPI_PARAM_ACTIVE) &&
        (s_param_transaction.op == IPC_REMOTE_PARAM_OP_GET)))
    {
        camera_spi_param_complete(IPC_REMOTE_PARAM_STATUS_TIMEOUT,
                                  s_param_transaction.fallback_bits);
        return 1U;
    }

    s_param_transaction.cancel_requested = 1U;
    if(s_param_transaction.state == CAMERA_SPI_PARAM_ROLLBACK)
    {
        return 1U;
    }

    rollback_mask = s_param_transaction.set_sent_mask;
    if(rollback_mask == 0U)
    {
        camera_spi_param_complete(IPC_REMOTE_PARAM_STATUS_TIMEOUT,
                                  s_param_transaction.fallback_bits);
    }
    else
    {
        camera_spi_param_start_rollback(rollback_mask);
    }
    return 1U;
}
