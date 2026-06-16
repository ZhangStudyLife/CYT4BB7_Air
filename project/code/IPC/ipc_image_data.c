#include "ipc_image_data.h"
#include "string.h"

#pragma location=".ipc_image_shared"
volatile ipc_mode2_payload_t g_ipc_mode2_shared;
#pragma location=".ipc_camera_spi_log"
volatile ipc_camera_spi_log_t g_ipc_camera_spi_log;

#define IPC_FLIGHT_STATE_MAGIC   (0xA5000000UL)
#define IPC_FLIGHT_STATE_MASK    (0xFFFF0000UL)
#define IPC_FLIGHT_STATE_FLYING  (0x00000001UL)

#if defined(CY_CORE_CM7_1)

static uint32 s_tx_seq = 0U;
static volatile uint8 s_core0_flying = 0U;

void ipc_mode2_send(float target_valid, float target_x, float target_y,
                    float car_lamp_valid, float car_lamp_cx, float car_lamp_cy,
                    float lamp_angle_deg)
{
    s_tx_seq++;
    g_ipc_mode2_shared.target_valid = target_valid;
    g_ipc_mode2_shared.target_x = target_x;
    g_ipc_mode2_shared.target_y = target_y;
    g_ipc_mode2_shared.car_lamp_valid = car_lamp_valid;
    g_ipc_mode2_shared.car_lamp_cx = car_lamp_cx;
    g_ipc_mode2_shared.car_lamp_cy = car_lamp_cy;
    g_ipc_mode2_shared.lamp_angle_deg = lamp_angle_deg;
    g_ipc_mode2_shared.seq = s_tx_seq;
    SCB_CleanDCache_by_Addr((volatile void *)&g_ipc_mode2_shared, sizeof(g_ipc_mode2_shared));
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

void ipc_camera_spi_log_publish(const ipc_camera_spi_log_t *log)
{
#if defined(CY_CORE_CM7_1)
    if(log == NULL) { return; }
    memcpy((void *)&g_ipc_camera_spi_log, (const void *)log, sizeof(g_ipc_camera_spi_log));
    SCB_CleanDCache_by_Addr((volatile void *)&g_ipc_camera_spi_log, sizeof(g_ipc_camera_spi_log));
#else
    (void)log;
#endif
}

void ipc_camera_spi_log_get(ipc_camera_spi_log_t *out)
{
    if(out == NULL) { return; }
#if defined(CY_CORE_CM7_0)
    SCB_InvalidateDCache_by_Addr((volatile void *)&g_ipc_camera_spi_log, sizeof(g_ipc_camera_spi_log));
#endif
    memcpy((void *)out, (const void *)&g_ipc_camera_spi_log, sizeof(ipc_camera_spi_log_t));
}

#if defined(CY_CORE_CM7_0)

static uint32 s_polled_seq = 0U;
volatile uint32 g_air_mode2_seq = 0U;
volatile float g_air_mode2_target_valid = 0.0f;
volatile float g_air_mode2_target_x = 0.0f;
volatile float g_air_mode2_target_y = 0.0f;
volatile float g_air_mode2_car_lamp_valid = 0.0f;
volatile float g_air_mode2_car_lamp_cx = 0.0f;
volatile float g_air_mode2_car_lamp_cy = 0.0f;
volatile float g_air_mode2_lamp_angle_deg = 0.0f;

volatile float g_down_camera_lamp_x = 0.0f;
volatile float g_down_camera_lamp_y = 0.0f;
volatile float g_down_camera_lamp_width = 0.0f;
volatile float g_down_camera_lamp_length = 0.0f;
volatile float g_down_camera_lamp_angle = 0.0f;
volatile float g_down_camera_lamp_valid = 0.0f;

static void ipc_mode2_unpack(void)
{
    g_air_mode2_seq = g_ipc_mode2_shared.seq;
    g_air_mode2_target_valid = g_ipc_mode2_shared.target_valid;
    g_air_mode2_target_x = g_ipc_mode2_shared.target_x;
    g_air_mode2_target_y = g_ipc_mode2_shared.target_y;
    g_air_mode2_car_lamp_valid = g_ipc_mode2_shared.car_lamp_valid;
    g_air_mode2_car_lamp_cx = g_ipc_mode2_shared.car_lamp_cx;
    g_air_mode2_car_lamp_cy = g_ipc_mode2_shared.car_lamp_cy;
    g_air_mode2_lamp_angle_deg = g_ipc_mode2_shared.lamp_angle_deg;

    if(g_air_mode2_car_lamp_valid > 0.5f)
    {
        g_down_camera_lamp_x = 94.0f - g_air_mode2_car_lamp_cx;
        g_down_camera_lamp_y = g_air_mode2_car_lamp_cy - 60.0f;
        g_down_camera_lamp_angle = g_air_mode2_lamp_angle_deg;
        g_down_camera_lamp_valid = 1.0f;
    }
    else
    {
        g_down_camera_lamp_x = 0.0f;
        g_down_camera_lamp_y = 0.0f;
        g_down_camera_lamp_angle = 0.0f;
        g_down_camera_lamp_valid = 0.0f;
    }
}

#endif

void ipc_image_callback(uint32 ipc_data)
{
#if defined(CY_CORE_CM7_0)
    (void)ipc_data;
    SCB_InvalidateDCache_by_Addr((volatile void *)&g_ipc_mode2_shared, sizeof(g_ipc_mode2_shared));
    s_polled_seq = g_ipc_mode2_shared.seq;
    ipc_mode2_unpack();
#elif defined(CY_CORE_CM7_1)
    if ((ipc_data & IPC_FLIGHT_STATE_MASK) == IPC_FLIGHT_STATE_MAGIC)
    {
        s_core0_flying = (0U != (ipc_data & IPC_FLIGHT_STATE_FLYING)) ? 1U : 0U;
    }
#else
    (void)ipc_data;
#endif
}

void ipc_mode2_poll(void)
{
#if defined(CY_CORE_CM7_0)
    SCB_InvalidateDCache_by_Addr((volatile void *)&g_ipc_mode2_shared, sizeof(g_ipc_mode2_shared));
    if(g_ipc_mode2_shared.seq != s_polled_seq)
    {
        s_polled_seq = g_ipc_mode2_shared.seq;
        ipc_mode2_unpack();
    }
#endif
}
