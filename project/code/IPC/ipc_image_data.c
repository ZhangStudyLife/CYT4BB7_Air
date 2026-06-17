#include "ipc_image_data.h"
#include "string.h"

#pragma location=".ipc_image_data"
struct image_data image_data[IMAGE_CAMERA_COUNT];
#pragma location=".ipc_image_seq"
volatile uint32 g_image_data_seq;
#pragma location=".ipc_camera_spi_log"
volatile ipc_camera_spi_log_t g_ipc_camera_spi_log;

#define IPC_FLIGHT_STATE_MAGIC   (0xA5000000UL)
#define IPC_FLIGHT_STATE_MASK    (0xFFFF0000UL)
#define IPC_FLIGHT_STATE_FLYING  (0x00000001UL)

#if defined(CY_CORE_CM7_1)

static uint32 s_tx_seq = 0U;
static volatile uint8 s_core0_flying = 0U;

void ipc_image_publish(void)
{
    s_tx_seq++;
    g_image_data_seq = s_tx_seq;
    SCB_CleanDCache_by_Addr((volatile void *)image_data, sizeof(image_data));
    SCB_CleanDCache_by_Addr((volatile void *)&g_image_data_seq, sizeof(g_image_data_seq));
    (void)ipc_send_data(s_tx_seq);
}

#else

void ipc_image_publish(void)
{
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

void ipc_image_callback(uint32 ipc_data)
{
#if defined(CY_CORE_CM7_0)
    (void)ipc_data;
    ipc_image_poll();
#elif defined(CY_CORE_CM7_1)
    if ((ipc_data & IPC_FLIGHT_STATE_MASK) == IPC_FLIGHT_STATE_MAGIC)
    {
        s_core0_flying = (0U != (ipc_data & IPC_FLIGHT_STATE_FLYING)) ? 1U : 0U;
    }
#else
    (void)ipc_data;
#endif
}

void ipc_image_poll(void)
{
#if defined(CY_CORE_CM7_0)
    SCB_InvalidateDCache_by_Addr((volatile void *)image_data, sizeof(image_data));
    SCB_InvalidateDCache_by_Addr((volatile void *)&g_image_data_seq, sizeof(g_image_data_seq));
#endif
}
