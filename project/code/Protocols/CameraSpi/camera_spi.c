#include "camera_spi.h"

#include "gpio/cy_gpio.h"
#include "scb/cy_scb_spi.h"
#include "sysclk/cy_sysclk.h"

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
#define CAMERA_SPI_TRANSFER_TIMEOUT_POLLS   (100000U)
#define CAMERA_SPI_DOWNLINK_MASK_ALL        (0x03U)
#define CAMERA_SPI_READY0_PIN               P01_0
#define CAMERA_SPI_READY1_PIN               P01_1
#define CAMERA_SPI_CS0_PIN                  P03_3
#define CAMERA_SPI_CS1_PIN                  P03_4
#define CAMERA_SPI_SCB                      SCB6
#define CAMERA_SPI_IRQ_SRC                  scb_6_interrupt_IRQn
#define CAMERA_SPI_CPU_IRQ                  CPUIntIdx5_IRQn
#define CAMERA_SPI_PERI_FREQ                CY_INITIAL_TARGET_PERI_FREQ

#define CAMERA_SPI_ERR_OK                   (0U)
#define CAMERA_SPI_ERR_INVALID_BOARD        (1U)
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
    uint8 has_new;
    uint8 last_error;
    uint8 peer_last_error;
    uint8 flags;
    uint16 app_len;
    uint32 tx_sequence;
    uint32 tx_counter;
    uint32 rx_sequence;
    uint32 ack_sequence;
    uint32 rx_ok_count;
    uint32 rx_error_count;
    uint8 last_rx_head0;
    uint8 last_rx_head1;
    uint8 app_data[CAMERA_SPI_APP_DATA_CAPACITY];
    uint8 downlink_data[CAMERA_SPI_APP_DATA_CAPACITY];
    uint8 has_custom_downlink;
} camera_spi_board_state_t;

static camera_spi_board_state_t s_boards[CAMERA_SPI_BOARD_COUNT];
static cy_stc_scb_spi_context_t s_spi_context;
static uint8 s_tx_frame[CAMERA_SPI_TRANSFER_LEN];
static uint8 s_rx_frame[CAMERA_SPI_TRANSFER_LEN];
static uint8 s_initialized;
static uint8 s_active;
static uint8 s_active_board;
static uint8 s_next_board;
static uint8 s_downlink_mask;
static uint8 s_flight_state;
static uint8 s_image_tcp;
static uint8 s_image_display = 1U;
static uint8 s_ready_mask;
static uint8 s_last_polled_board;
static uint32 s_active_poll_count;

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

static uint8 camera_spi_select_board(uint8 candidates, uint8 *board_id)
{
    uint8 i;

    if(board_id == NULL)
    {
        return 0U;
    }

    for(i = 0U; i < CAMERA_SPI_BOARD_COUNT; i++)
    {
        uint8 id = (uint8)((s_next_board + i) % CAMERA_SPI_BOARD_COUNT);
        if((candidates & (uint8)(1U << id)) != 0U)
        {
            *board_id = id;
            s_next_board = (uint8)((id + 1U) % CAMERA_SPI_BOARD_COUNT);
            return 1U;
        }
    }

    return 0U;
}

static void camera_spi_build_downlink_app(uint8 board_id, uint8 *app)
{
    camera_spi_board_state_t *board = &s_boards[board_id];

    memset(app, 0, CAMERA_SPI_APP_DATA_CAPACITY);
    if(board->has_custom_downlink != 0U)
    {
        memcpy(app, board->downlink_data, CAMERA_SPI_APP_DATA_CAPACITY);
    }

    app[0] = CAMERA_SPI_DOWNLINK_MAGIC;
    app[1] = board_id;
    camera_spi_write_u32_le(&app[2], board->tx_counter++);
    app[6] = s_flight_state;
    app[7] = s_image_tcp;
    app[8] = (s_flight_state == 0U) ? s_image_display : 0U;
}

static void camera_spi_refresh_flight_state(void)
{
    uint8 board_id;
    uint8 flying = (ipc_core0_is_flying() != 0U) ? 1U : 0U;

    if(flying == s_flight_state)
    {
        return;
    }

    s_flight_state = flying;
    for(board_id = 0U; board_id < CAMERA_SPI_BOARD_COUNT; board_id++)
    {
        s_boards[board_id].tx_sequence++;
    }
    s_downlink_mask |= CAMERA_SPI_DOWNLINK_MASK_ALL;
}

static void camera_spi_build_request_frame(uint8 board_id)
{
    uint16 crc;
    uint8 *payload;
    camera_spi_board_state_t *board = &s_boards[board_id];

    memset(s_tx_frame, 0, sizeof(s_tx_frame));
    s_tx_frame[0] = CAMERA_SPI_FRAME_HEAD_0;
    s_tx_frame[1] = CAMERA_SPI_FRAME_HEAD_1;
    s_tx_frame[2] = CAMERA_SPI_CMD_SYNC_DATA;
    camera_spi_write_u16_be(&s_tx_frame[3], CAMERA_SPI_REQ_PAYLOAD_SIZE);

    payload = &s_tx_frame[5];
    camera_spi_write_u32_le(&payload[0], board->tx_sequence);
    camera_spi_write_u16_le(&payload[4], CAMERA_SPI_DOWNLINK_APP_LEN);
    camera_spi_build_downlink_app(board_id, &payload[6]);

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

static uint8 camera_spi_parse_response(uint8 board_id)
{
    uint16 payload_len;
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
    if(camera_spi_read_u16_le(&payload[8]) != CAMERA_SPI_APP_DATA_CAPACITY)
    {
        return CAMERA_SPI_ERR_APP_LEN;
    }

    board = &s_boards[board_id];
    board->rx_sequence = camera_spi_read_u32_le(&payload[0]);
    board->ack_sequence = camera_spi_read_u32_le(&payload[4]);
    board->app_len = CAMERA_SPI_APP_DATA_CAPACITY;
    board->flags = payload[10];
    board->peer_last_error = payload[11];
    memcpy(board->app_data, &payload[12], CAMERA_SPI_APP_DATA_CAPACITY);
    board->online = 1U;
    board->has_new = 1U;
    board->last_error = CAMERA_SPI_ERR_OK;
    board->rx_ok_count++;

    if(board->ack_sequence >= board->tx_sequence)
    {
        s_downlink_mask &= (uint8)~(1U << board_id);
        board->has_custom_downlink = 0U;
    }

    return CAMERA_SPI_ERR_OK;
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

    if((board_id >= CAMERA_SPI_BOARD_COUNT) || (s_active != 0U))
    {
        camera_spi_record_error(board_id, CAMERA_SPI_ERR_TRANSFER_BUSY);
        return;
    }

    camera_spi_build_request_frame(board_id);
    memset(s_rx_frame, 0, sizeof(s_rx_frame));
    Cy_SCB_SPI_ClearRxFifo(CAMERA_SPI_SCB);
    Cy_SCB_SPI_ClearTxFifo(CAMERA_SPI_SCB);
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
    }
    else
    {
        camera_spi_set_cs(board_id, 0U);
        camera_spi_record_error(board_id, CAMERA_SPI_ERR_HW);
    }
}

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
        s_active_poll_count++;
        if(s_active_poll_count > CAMERA_SPI_TRANSFER_TIMEOUT_POLLS)
        {
            Cy_SCB_SPI_AbortTransfer(CAMERA_SPI_SCB, &s_spi_context);
            camera_spi_set_cs(s_active_board, 0U);
            camera_spi_record_error(s_active_board, CAMERA_SPI_ERR_TIMEOUT);
            s_active = 0U;
        }
        return;
    }

    if(Cy_SCB_SPI_IsBusBusy(CAMERA_SPI_SCB))
    {
        return;
    }

    camera_spi_set_cs(s_active_board, 0U);
    s_boards[s_active_board].last_rx_head0 = s_rx_frame[0];
    s_boards[s_active_board].last_rx_head1 = s_rx_frame[1];
    error = camera_spi_parse_response(s_active_board);
    camera_spi_record_error(s_active_board, error);
    s_active = 0U;
}

void CameraSpi_Init(void)
{
    uint8 board_id;

    memset(s_boards, 0, sizeof(s_boards));
    memset(s_tx_frame, 0, sizeof(s_tx_frame));
    memset(s_rx_frame, 0, sizeof(s_rx_frame));
    memset(&s_spi_context, 0, sizeof(s_spi_context));
    s_initialized = 0U;
    s_active = 0U;
    s_active_board = 0U;
    s_next_board = 0U;
    s_downlink_mask = CAMERA_SPI_DOWNLINK_MASK_ALL;
    s_flight_state = (ipc_core0_is_flying() != 0U) ? 1U : 0U;
    s_image_tcp = 0U;
    s_image_display = 1U;
    s_ready_mask = 0U;
    s_last_polled_board = 0xFFU;
    s_active_poll_count = 0U;

    for(board_id = 0U; board_id < CAMERA_SPI_BOARD_COUNT; board_id++)
    {
        s_boards[board_id].tx_sequence = 1U;
    }

    camera_spi_init_pins();
    camera_spi_init_scb();
    camera_spi_init_irq();

    s_initialized = 1U;
}

void CameraSpi_Poll(void)
{
    uint8 candidates;
    uint8 board_id;

    if(s_initialized == 0U)
    {
        return;
    }

    camera_spi_finish_active();
    if(s_active != 0U)
    {
        return;
    }

    camera_spi_refresh_flight_state();
    s_ready_mask = camera_spi_ready_mask();
    candidates = (uint8)(s_ready_mask | s_downlink_mask);
    if(camera_spi_select_board(candidates, &board_id) != 0U)
    {
        s_last_polled_board = board_id;
        camera_spi_start_transfer(board_id);
    }
}

uint8 CameraSpi_SendRaw(uint8 board_id, const uint8 *data, uint16 len)
{
    uint16 copy_len = len;

    if(board_id >= CAMERA_SPI_BOARD_COUNT)
    {
        return 0U;
    }
    if((data == NULL) && (len > 0U))
    {
        return 0U;
    }
    if(copy_len > CAMERA_SPI_APP_DATA_CAPACITY)
    {
        copy_len = CAMERA_SPI_APP_DATA_CAPACITY;
    }

    memset(s_boards[board_id].downlink_data, 0, CAMERA_SPI_APP_DATA_CAPACITY);
    if(copy_len > 0U)
    {
        memcpy(s_boards[board_id].downlink_data, data, copy_len);
    }
    s_boards[board_id].tx_sequence++;
    s_boards[board_id].has_custom_downlink = 1U;
    s_downlink_mask |= (uint8)(1U << board_id);

    return 1U;
}

uint8 CameraSpi_ReceiveRaw(uint8 board_id, uint8 *data, uint16 *len)
{
    if((board_id >= CAMERA_SPI_BOARD_COUNT) || (data == NULL) || (len == NULL))
    {
        return 0U;
    }
    if(s_boards[board_id].has_new == 0U)
    {
        *len = 0U;
        return 0U;
    }
    if(*len < CAMERA_SPI_APP_DATA_CAPACITY)
    {
        return 0U;
    }

    memcpy(data, s_boards[board_id].app_data, CAMERA_SPI_APP_DATA_CAPACITY);
    *len = CAMERA_SPI_APP_DATA_CAPACITY;
    s_boards[board_id].has_new = 0U;

    return 1U;
}

uint8 CameraSpi_GetSnapshot(uint8 board_id, camera_spi_snapshot_t *snapshot)
{
    const camera_spi_board_state_t *board;

    if((board_id >= CAMERA_SPI_BOARD_COUNT) || (snapshot == NULL))
    {
        return 0U;
    }

    board = &s_boards[board_id];
    snapshot->online = board->online;
    snapshot->has_new = board->has_new;
    snapshot->last_error = board->last_error;
    snapshot->peer_last_error = board->peer_last_error;
    snapshot->flags = board->flags;
    snapshot->app_len = board->app_len;
    snapshot->rx_sequence = board->rx_sequence;
    snapshot->ack_sequence = board->ack_sequence;
    snapshot->rx_ok_count = board->rx_ok_count;
    snapshot->rx_error_count = board->rx_error_count;
    snapshot->ready_mask = s_ready_mask;
    snapshot->last_rx_head0 = board->last_rx_head0;
    snapshot->last_rx_head1 = board->last_rx_head1;
    snapshot->last_polled_board = s_last_polled_board;
    memcpy(snapshot->app_data, board->app_data, CAMERA_SPI_APP_DATA_CAPACITY);

    return 1U;
}

void CameraSpi_SetDebugOptions(uint8 image_tcp, uint8 image_display)
{
    uint8 board_id;

    s_image_tcp = (image_tcp != 0U) ? 1U : 0U;
    s_image_display = (image_display != 0U) ? 1U : 0U;
    for(board_id = 0U; board_id < CAMERA_SPI_BOARD_COUNT; board_id++)
    {
        s_boards[board_id].tx_sequence++;
    }
    s_downlink_mask |= CAMERA_SPI_DOWNLINK_MASK_ALL;
}
