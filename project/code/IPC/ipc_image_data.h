#ifndef IPC_IMAGE_DATA_H_
#define IPC_IMAGE_DATA_H_

#include "zf_common_headfile.h"
#include "Image/image_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IPC_CAMERA_SPI_BOARD_COUNT (2U)

typedef struct
{
    uint8 online;
    uint8 beacon_count;
    uint8 car_lamp_count;
    uint8 first_beacon_valid;
    uint8 first_lamp_valid;
    uint8 last_error;
    uint8 last_rx_head0;
    uint8 last_rx_head1;
    uint32 rx_ok_count;
    uint32 rx_error_count;
    float first_beacon_x;
    float first_beacon_y;
    float first_beacon_radius;
    float first_lamp_cx;
    float first_lamp_cy;
    float first_lamp_angle;
} ipc_camera_spi_board_log_t;

typedef struct
{
    volatile uint32 seq;
    ipc_camera_spi_board_log_t board[IPC_CAMERA_SPI_BOARD_COUNT];
} ipc_camera_spi_log_t;

extern volatile uint32 g_image_data_seq;
extern volatile ipc_camera_spi_log_t g_ipc_camera_spi_log;

void ipc_image_callback(uint32 ipc_data);
void ipc_image_publish(void);
void ipc_image_poll(void);

uint8 ipc_flight_state_send(uint8 flying);
uint8 ipc_core0_is_flying(void);
void ipc_camera_spi_log_publish(const ipc_camera_spi_log_t *log);
void ipc_camera_spi_log_get(ipc_camera_spi_log_t *out);

#ifdef __cplusplus
}
#endif

#endif /* IPC_IMAGE_DATA_H_ */
