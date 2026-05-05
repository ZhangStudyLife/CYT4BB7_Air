#include "ipc_image_data.h"
#include "string.h"

/* 共享内存：放在 .global_ram_data 段 (0x28001000)，双核可访问 */
#pragma location=".global_ram_data"
volatile ipc_image_payload_t g_ipc_image_shared;

#define IPC_FLIGHT_STATE_MAGIC   (0xA5000000UL)  /* 核0到核1飞行状态 IPC 命令标记 */
#define IPC_FLIGHT_STATE_MASK    (0xFFFF0000UL)  /* 飞行状态 IPC 命令匹配掩码 */
#define IPC_FLIGHT_STATE_FLYING  (0x00000001UL)  /* 飞行状态 IPC 命令：1=飞行中 */

/* ======================== CM7_1 发送侧 ======================== */
#if defined(CY_CORE_CM7_1)

#include "../Estimation/Pos_Est/image.h"

static uint32 s_tx_seq = 0;
static volatile uint8 s_core0_flying = 0U;

void ipc_image_send(void)
{
    uint8 i;
    uint8 cnt = 0;

    for (i = 0; i < IMAGE_MAX_CIRCLE_COUNT && i < IPC_IMAGE_MAX_CIRCLES; i++)
    {
        g_ipc_image_shared.circles[i].x      = g_image_circles[i].x;
        g_ipc_image_shared.circles[i].y      = g_image_circles[i].y;
        g_ipc_image_shared.circles[i].radius  = g_image_circles[i].radius;
        g_ipc_image_shared.circles[i].valid   = g_image_circles[i].valid;
        if (g_image_circles[i].valid)
            cnt++;
    }
    g_ipc_image_shared.count = cnt;
    s_tx_seq++;
    g_ipc_image_shared.seq = s_tx_seq;

    ipc_send_data(s_tx_seq);
}

#endif /* CY_CORE_CM7_1 */

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

/* ======================== 通用回调（两核都编译） ======================== */

#if defined(CY_CORE_CM7_0)
static volatile uint8  s_new_data = 0;
static volatile uint32 s_rx_seq   = 0;
#endif

void ipc_image_callback(uint32 ipc_data)
{
#if defined(CY_CORE_CM7_0)
    s_rx_seq   = ipc_data;
    s_new_data = 1;
#elif defined(CY_CORE_CM7_1)
    if ((ipc_data & IPC_FLIGHT_STATE_MASK) == IPC_FLIGHT_STATE_MAGIC)
    {
        s_core0_flying = (0U != (ipc_data & IPC_FLIGHT_STATE_FLYING)) ? 1U : 0U;
    }
#else
    (void)ipc_data;
#endif
}

/* ======================== CM7_0 接收侧 ======================== */
#if defined(CY_CORE_CM7_0)

uint8 ipc_image_is_new(void)
{
    if (s_new_data)
    {
        s_new_data = 0;
        return 1;
    }
    return 0;
}

void ipc_image_get(ipc_image_payload_t *out)
{
    memcpy((void *)out, (const void *)&g_ipc_image_shared, sizeof(ipc_image_payload_t));
}

#endif /* CY_CORE_CM7_0 */
