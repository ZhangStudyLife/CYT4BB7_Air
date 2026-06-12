#ifndef IPC_IMAGE_DATA_H_
#define IPC_IMAGE_DATA_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IPC_IMAGE_MAX_BEACONS      (4U)
#define IPC_IMAGE_MAX_CAR_LAMPS    (1U)
#define IPC_CAMERA_SPI_BOARD_COUNT (2U)

typedef struct
{
    uint8 online;
    uint8 last_error;
    uint8 peer_last_error;
    uint8 flags;
    uint8 version;
    uint8 beacon_count;
    uint8 car_lamp_count;
    uint8 first_beacon_valid;
    uint8 first_lamp_valid;
    uint8 ready_mask;
    uint8 last_rx_head0;
    uint8 last_rx_head1;
    uint8 last_polled_board;
    uint8 _pad[3];
    uint32 rx_ok_count;
    uint32 rx_error_count;
    uint32 rx_sequence;
    uint32 ack_sequence;
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
    uint8 ready_mask;
    uint8 last_polled_board;
    uint8 _pad[2];
    ipc_camera_spi_board_log_t board[IPC_CAMERA_SPI_BOARD_COUNT];
} ipc_camera_spi_log_t;

typedef struct
{
    float x;
    float y;
    float radius;
    uint8 valid;
    uint8 _pad[3];
} ipc_image_beacon_t;

typedef struct
{
    float cx;
    float cy;
    float width;
    float length;
    float angle;
    uint8 valid;
    uint8 _pad[3];
} ipc_image_car_lamp_t;

typedef struct
{
    volatile uint32 seq;
    uint8 beacon_count;
    uint8 car_lamp_count;
    uint8 _pad[2];
    ipc_image_beacon_t beacons[IPC_IMAGE_MAX_BEACONS];
    ipc_image_car_lamp_t car_lamps[IPC_IMAGE_MAX_CAR_LAMPS];
} ipc_image_payload_t;

#if defined(CY_CORE_CM7_0)
extern volatile uint32 g_air_image_seq;
extern volatile uint8 g_air_image_beacon_count;
extern volatile uint8 g_air_image_car_lamp_count;
extern volatile ipc_image_beacon_t g_air_image_beacons[IPC_IMAGE_MAX_BEACONS];
extern volatile ipc_image_car_lamp_t g_air_image_car_lamps[IPC_IMAGE_MAX_CAR_LAMPS];
extern volatile float g_down_camera_lamp_x;
extern volatile float g_down_camera_lamp_y;
extern volatile float g_down_camera_lamp_width;
extern volatile float g_down_camera_lamp_length;
extern volatile float g_down_camera_lamp_angle;
extern volatile float g_down_camera_lamp_valid;
#endif

extern volatile ipc_camera_spi_log_t g_ipc_camera_spi_log;

void ipc_image_send(void);
void ipc_image_callback(uint32 ipc_data);
uint8 ipc_image_is_new(void);
void ipc_image_get(ipc_image_payload_t *out);

uint8 ipc_flight_state_send(uint8 flying);
uint8 ipc_core0_is_flying(void);
void ipc_camera_spi_log_publish(const ipc_camera_spi_log_t *log);
void ipc_camera_spi_log_get(ipc_camera_spi_log_t *out);

#ifdef __cplusplus
}
#endif

#endif /* IPC_IMAGE_DATA_H_ */
