#include "zf_common_headfile.h"

#define SPI_TEST_IMAGE_PROTOCOL_VERSION          (2U)
#define SPI_TEST_BOARD_COUNT                     (CAMERA_SPI_BOARD_COUNT)
#define SPI_TEST_TIMER                           (TC_TIME2_CH0)
#define SPI_TEST_PERIOD_MS                       (10U)

#define SPI_TEST_IMAGE_VERSION_OFFSET            (0U)
#define SPI_TEST_IMAGE_BEACON_COUNT_OFFSET       (1U)
#define SPI_TEST_IMAGE_CAR_LAMP_COUNT_OFFSET     (2U)
#define SPI_TEST_IMAGE_HEADER_SIZE               (4U)

#define SPI_TEST_IMAGE_BEACON_VALID_OFFSET       (0U)
#define SPI_TEST_IMAGE_BEACON_X_OFFSET           (1U)
#define SPI_TEST_IMAGE_BEACON_Y_OFFSET           (5U)
#define SPI_TEST_IMAGE_BEACON_RADIUS_OFFSET      (9U)
#define SPI_TEST_IMAGE_BEACON_SLOT_SIZE          (13U)
#define SPI_TEST_IMAGE_BEACON_COUNT              (4U)

#define SPI_TEST_IMAGE_CAR_LAMP_VALID_OFFSET     (0U)
#define SPI_TEST_IMAGE_CAR_LAMP_CX_OFFSET        (1U)
#define SPI_TEST_IMAGE_CAR_LAMP_CY_OFFSET        (5U)
#define SPI_TEST_IMAGE_CAR_LAMP_ANGLE_OFFSET     (17U)
#define SPI_TEST_IMAGE_CAR_LAMP_SLOT_SIZE        (21U)
#define SPI_TEST_IMAGE_CAR_LAMP_COUNT            (1U)

#define SPI_TEST_IMAGE_BEACON_PACKET_OFFSET      SPI_TEST_IMAGE_HEADER_SIZE
#define SPI_TEST_IMAGE_CAR_LAMP_PACKET_OFFSET \
    (SPI_TEST_IMAGE_BEACON_PACKET_OFFSET + \
     (SPI_TEST_IMAGE_BEACON_COUNT * SPI_TEST_IMAGE_BEACON_SLOT_SIZE))

typedef struct
{
    uint8 version;
    uint8 beacon_count;
    uint8 car_lamp_count;
    uint8 first_beacon_valid;
    uint8 first_lamp_valid;
    float first_beacon_x;
    float first_beacon_y;
    float first_beacon_radius;
    float first_lamp_cx;
    float first_lamp_cy;
    float first_lamp_angle;
} spi_test_image_result_t;

static camera_spi_snapshot_t s_board_snapshot[SPI_TEST_BOARD_COUNT];
static spi_test_image_result_t s_board_result[SPI_TEST_BOARD_COUNT];
static uint32 s_ipc_log_seq;

static void spi_test_read_float(const uint8 *data, uint16 offset, float *value)
{
    memcpy(value, &data[offset], sizeof(float));
}

static void spi_test_parse_image_packet(uint8 board_id, const uint8 *data, uint16 len)
{
    const uint8 *slot;
    spi_test_image_result_t *result;

    if((board_id >= SPI_TEST_BOARD_COUNT) || (data == NULL) ||
       (len < CAMERA_SPI_APP_DATA_CAPACITY))
    {
        return;
    }

    result = &s_board_result[board_id];
    memset(result, 0, sizeof(*result));

    result->version = data[SPI_TEST_IMAGE_VERSION_OFFSET];
    result->beacon_count = data[SPI_TEST_IMAGE_BEACON_COUNT_OFFSET];
    result->car_lamp_count = data[SPI_TEST_IMAGE_CAR_LAMP_COUNT_OFFSET];

    slot = &data[SPI_TEST_IMAGE_BEACON_PACKET_OFFSET];
    result->first_beacon_valid = slot[SPI_TEST_IMAGE_BEACON_VALID_OFFSET];
    spi_test_read_float(slot, SPI_TEST_IMAGE_BEACON_X_OFFSET, &result->first_beacon_x);
    spi_test_read_float(slot, SPI_TEST_IMAGE_BEACON_Y_OFFSET, &result->first_beacon_y);
    spi_test_read_float(slot, SPI_TEST_IMAGE_BEACON_RADIUS_OFFSET, &result->first_beacon_radius);

    slot = &data[SPI_TEST_IMAGE_CAR_LAMP_PACKET_OFFSET];
    result->first_lamp_valid = slot[SPI_TEST_IMAGE_CAR_LAMP_VALID_OFFSET];
    spi_test_read_float(slot, SPI_TEST_IMAGE_CAR_LAMP_CX_OFFSET, &result->first_lamp_cx);
    spi_test_read_float(slot, SPI_TEST_IMAGE_CAR_LAMP_CY_OFFSET, &result->first_lamp_cy);
    spi_test_read_float(slot, SPI_TEST_IMAGE_CAR_LAMP_ANGLE_OFFSET, &result->first_lamp_angle);
}

static void spi_test_publish_ipc_log(void)
{
    ipc_camera_spi_log_t log;
    uint8 board_id;

    memset(&log, 0, sizeof(log));
    log.seq = ++s_ipc_log_seq;
    log.ready_mask = s_board_snapshot[0].ready_mask;
    log.last_polled_board = s_board_snapshot[0].last_polled_board;

    for(board_id = 0U; board_id < SPI_TEST_BOARD_COUNT; board_id++)
    {
        const camera_spi_snapshot_t *snapshot = &s_board_snapshot[board_id];
        const spi_test_image_result_t *result = &s_board_result[board_id];
        ipc_camera_spi_board_log_t *board = &log.board[board_id];

        board->online = snapshot->online;
        board->last_error = snapshot->last_error;
        board->peer_last_error = snapshot->peer_last_error;
        board->flags = snapshot->flags;
        board->version = result->version;
        board->beacon_count = result->beacon_count;
        board->car_lamp_count = result->car_lamp_count;
        board->first_beacon_valid = result->first_beacon_valid;
        board->first_lamp_valid = result->first_lamp_valid;
        board->ready_mask = snapshot->ready_mask;
        board->last_rx_head0 = snapshot->last_rx_head0;
        board->last_rx_head1 = snapshot->last_rx_head1;
        board->last_polled_board = snapshot->last_polled_board;
        board->rx_ok_count = snapshot->rx_ok_count;
        board->rx_error_count = snapshot->rx_error_count;
        board->rx_sequence = snapshot->rx_sequence;
        board->ack_sequence = snapshot->ack_sequence;
        board->first_beacon_x = result->first_beacon_x;
        board->first_beacon_y = result->first_beacon_y;
        board->first_beacon_radius = result->first_beacon_radius;
        board->first_lamp_cx = result->first_lamp_cx;
        board->first_lamp_cy = result->first_lamp_cy;
        board->first_lamp_angle = result->first_lamp_angle;
    }

    ipc_camera_spi_log_publish(&log);
}

static void spi_test_poll_receive(void)
{
    uint8 board_id;
    uint8 rx[CAMERA_SPI_APP_DATA_CAPACITY];

    for(board_id = 0U; board_id < SPI_TEST_BOARD_COUNT; board_id++)
    {
        uint16 len = sizeof(rx);

        if(CameraSpi_ReceiveRaw(board_id, rx, &len) != 0U)
        {
            spi_test_parse_image_packet(board_id, rx, len);
        }
        (void)CameraSpi_GetSnapshot(board_id, &s_board_snapshot[board_id]);
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();

    CameraSpi_Init();
    CameraSpi_SetDebugOptions(1U, 0U);
    timer_init(SPI_TEST_TIMER, TIMER_MS);
    timer_start(SPI_TEST_TIMER);

    while(true)
    {
        if(timer_get(SPI_TEST_TIMER) >= SPI_TEST_PERIOD_MS)
        {
            timer_clear(SPI_TEST_TIMER);
            CameraSpi_Poll();
            spi_test_poll_receive();
            spi_test_publish_ipc_log();
        }
    }
}
