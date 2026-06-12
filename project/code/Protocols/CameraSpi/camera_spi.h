#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include "zf_common_headfile.h"

#define CAMERA_SPI_BOARD_COUNT              (2U)
#define CAMERA_SPI_TRANSFER_LEN             (97U)
#define CAMERA_SPI_APP_DATA_CAPACITY        (77U)
#define CAMERA_SPI_DOWNLINK_APP_LEN         (9U)

typedef struct
{
    uint8 online;
    uint8 has_new;
    uint8 last_error;
    uint8 peer_last_error;
    uint8 flags;
    uint16 app_len;
    uint32 rx_sequence;
    uint32 ack_sequence;
    uint32 rx_ok_count;
    uint32 rx_error_count;
    uint8 ready_mask;
    uint8 last_rx_head0;
    uint8 last_rx_head1;
    uint8 last_polled_board;
    uint8 app_data[CAMERA_SPI_APP_DATA_CAPACITY];
} camera_spi_snapshot_t;

void CameraSpi_Init(void);
void CameraSpi_Poll(void);
uint8 CameraSpi_SendRaw(uint8 board_id, const uint8 *data, uint16 len);
uint8 CameraSpi_ReceiveRaw(uint8 board_id, uint8 *data, uint16 *len);
uint8 CameraSpi_GetSnapshot(uint8 board_id, camera_spi_snapshot_t *snapshot);
void CameraSpi_SetDebugOptions(uint8 image_tcp, uint8 image_display);

#endif
