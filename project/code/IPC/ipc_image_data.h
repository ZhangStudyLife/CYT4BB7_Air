#ifndef IPC_IMAGE_DATA_H_
#define IPC_IMAGE_DATA_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IPC_IMAGE_MAX_BEACONS      (4U)
#define IPC_IMAGE_MAX_CAR_LAMPS    (1U)

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
#endif

void ipc_image_send(void);
void ipc_image_callback(uint32 ipc_data);
uint8 ipc_image_is_new(void);
void ipc_image_get(ipc_image_payload_t *out);

uint8 ipc_flight_state_send(uint8 flying);
uint8 ipc_core0_is_flying(void);

#ifdef __cplusplus
}
#endif

#endif /* IPC_IMAGE_DATA_H_ */
