#include "ipc_image_data.h"
#include "string.h"

#if defined(CY_CORE_CM7_1)
#include "../Estimation/Pos_Est/image.h"
#endif

#pragma location=".global_ram_data"
volatile ipc_image_payload_t g_ipc_image_shared;

#define IPC_FLIGHT_STATE_MAGIC   (0xA5000000UL)
#define IPC_FLIGHT_STATE_MASK    (0xFFFF0000UL)
#define IPC_FLIGHT_STATE_FLYING  (0x00000001UL)

#if defined(CY_CORE_CM7_1)

static uint32 s_tx_seq = 0U;
static volatile uint8 s_core0_flying = 0U;

void ipc_image_send(void)
{
    uint8 i;
    uint8 beacon_count = g_image_beacon_count;
    uint8 car_lamp_count = g_image_car_lamp_count;

    memset((void *)&g_ipc_image_shared, 0, sizeof(g_ipc_image_shared));

    if (beacon_count > IPC_IMAGE_MAX_BEACONS)
    {
        beacon_count = IPC_IMAGE_MAX_BEACONS;
    }
    if (car_lamp_count > IPC_IMAGE_MAX_CAR_LAMPS)
    {
        car_lamp_count = IPC_IMAGE_MAX_CAR_LAMPS;
    }

    g_ipc_image_shared.beacon_count = beacon_count;
    g_ipc_image_shared.car_lamp_count = car_lamp_count;

    for (i = 0U; i < beacon_count; i++)
    {
        g_ipc_image_shared.beacons[i].x = g_image_beacons[i].x;
        g_ipc_image_shared.beacons[i].y = g_image_beacons[i].y;
        g_ipc_image_shared.beacons[i].radius = g_image_beacons[i].radius;
        g_ipc_image_shared.beacons[i].valid = g_image_beacons[i].valid;
    }

    for (i = 0U; i < car_lamp_count; i++)
    {
        g_ipc_image_shared.car_lamps[i].cx = g_image_car_lamps[i].cx;
        g_ipc_image_shared.car_lamps[i].cy = g_image_car_lamps[i].cy;
        g_ipc_image_shared.car_lamps[i].width = g_image_car_lamps[i].width;
        g_ipc_image_shared.car_lamps[i].length = g_image_car_lamps[i].length;
        g_ipc_image_shared.car_lamps[i].angle = g_image_car_lamps[i].angle;
        g_ipc_image_shared.car_lamps[i].valid = g_image_car_lamps[i].valid;
    }

    s_tx_seq++;
    g_ipc_image_shared.seq = s_tx_seq;
    SCB_CleanDCache_by_Addr((volatile void *)&g_ipc_image_shared, sizeof(g_ipc_image_shared));
    (void)ipc_send_data(s_tx_seq);
}

#endif

uint8 ipc_flight_state_send(uint8 flying)
{
#if defined(CY_CORE_CM7_0)
    uint32 ipc_data = IPC_FLIGHT_STATE_MAGIC;

    if (0U != flying)
    {
        ipc_data |= IPC_FLIGHT_STATE_FLYING;
    }

    return ipc_send_data(ipc_data);
#else
    (void)flying;
    return 1U;
#endif
}

uint8 ipc_core0_is_flying(void)
{
#if defined(CY_CORE_CM7_1)
    return s_core0_flying;
#else
    return 0U;
#endif
}

#if defined(CY_CORE_CM7_0)

static volatile uint8 s_new_data = 0U;
static ipc_image_payload_t s_latest_image;

volatile uint32 g_air_image_seq = 0U;
volatile uint8 g_air_image_beacon_count = 0U;
volatile uint8 g_air_image_car_lamp_count = 0U;
volatile ipc_image_beacon_t g_air_image_beacons[IPC_IMAGE_MAX_BEACONS] = {0};
volatile ipc_image_car_lamp_t g_air_image_car_lamps[IPC_IMAGE_MAX_CAR_LAMPS] = {0};
volatile float g_down_camera_lamp_x = 0.0f;
volatile float g_down_camera_lamp_y = 0.0f;
volatile float g_down_camera_lamp_width = 0.0f;
volatile float g_down_camera_lamp_length = 0.0f;
volatile float g_down_camera_lamp_angle = 0.0f;
volatile float g_down_camera_lamp_valid = 0.0f;

static void ipc_image_publish_latest(void)
{
    uint8 i;

    g_air_image_seq = s_latest_image.seq;
    g_air_image_beacon_count = s_latest_image.beacon_count;
    g_air_image_car_lamp_count = s_latest_image.car_lamp_count;

    for (i = 0U; i < IPC_IMAGE_MAX_BEACONS; i++)
    {
        g_air_image_beacons[i].x = s_latest_image.beacons[i].x;
        g_air_image_beacons[i].y = s_latest_image.beacons[i].y;
        g_air_image_beacons[i].radius = s_latest_image.beacons[i].radius;
        g_air_image_beacons[i].valid = s_latest_image.beacons[i].valid;
    }

    for (i = 0U; i < IPC_IMAGE_MAX_CAR_LAMPS; i++)
    {
        g_air_image_car_lamps[i].cx = s_latest_image.car_lamps[i].cx;
        g_air_image_car_lamps[i].cy = s_latest_image.car_lamps[i].cy;
        g_air_image_car_lamps[i].width = s_latest_image.car_lamps[i].width;
        g_air_image_car_lamps[i].length = s_latest_image.car_lamps[i].length;
        g_air_image_car_lamps[i].angle = s_latest_image.car_lamps[i].angle;
        g_air_image_car_lamps[i].valid = s_latest_image.car_lamps[i].valid;
    }

    if ((s_latest_image.car_lamp_count > 0U) &&
        (0U != s_latest_image.car_lamps[0].valid))
    {
        g_down_camera_lamp_x = s_latest_image.car_lamps[0].cx;
        g_down_camera_lamp_y = s_latest_image.car_lamps[0].cy;
        g_down_camera_lamp_width = s_latest_image.car_lamps[0].width;
        g_down_camera_lamp_length = s_latest_image.car_lamps[0].length;
        g_down_camera_lamp_angle = s_latest_image.car_lamps[0].angle;
        g_down_camera_lamp_valid = 1.0f;
    }
    else
    {
        g_down_camera_lamp_x = 0.0f;
        g_down_camera_lamp_y = 0.0f;
        g_down_camera_lamp_width = 0.0f;
        g_down_camera_lamp_length = 0.0f;
        g_down_camera_lamp_angle = 0.0f;
        g_down_camera_lamp_valid = 0.0f;
    }
}

#endif

void ipc_image_callback(uint32 ipc_data)
{
#if defined(CY_CORE_CM7_0)
    (void)ipc_data;
    SCB_InvalidateDCache_by_Addr((volatile void *)&g_ipc_image_shared, sizeof(g_ipc_image_shared));
    memcpy((void *)&s_latest_image, (const void *)&g_ipc_image_shared, sizeof(s_latest_image));
    ipc_image_publish_latest();
    s_new_data = 1U;
#elif defined(CY_CORE_CM7_1)
    if ((ipc_data & IPC_FLIGHT_STATE_MASK) == IPC_FLIGHT_STATE_MAGIC)
    {
        s_core0_flying = (0U != (ipc_data & IPC_FLIGHT_STATE_FLYING)) ? 1U : 0U;
    }
#else
    (void)ipc_data;
#endif
}

#if defined(CY_CORE_CM7_0)

uint8 ipc_image_is_new(void)
{
    if (0U != s_new_data)
    {
        s_new_data = 0U;
        return 1U;
    }
    return 0U;
}

void ipc_image_get(ipc_image_payload_t *out)
{
    if (out != 0)
    {
        memcpy((void *)out, (const void *)&s_latest_image, sizeof(ipc_image_payload_t));
    }
}

#else

uint8 ipc_image_is_new(void)
{
    return 0U;
}

void ipc_image_get(ipc_image_payload_t *out)
{
    if (out != 0)
    {
        memset((void *)out, 0, sizeof(ipc_image_payload_t));
    }
}

#endif
