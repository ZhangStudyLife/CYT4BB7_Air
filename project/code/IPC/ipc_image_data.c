/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#include "ipc_image_data.h"
#include "string.h"

#if defined(CY_CORE_CM7_1)
#include "Estimation/Pos_Est/image_down.h"
#include "Protocols/CameraSpi/camera_spi.h"
#endif

typedef char ipc_image_shared_data_size_must_be_336[
    ((sizeof(struct image_data) * IMAGE_CAMERA_COUNT) == 336U) ? 1 : -1];

#pragma location=".ipc_image_data"
static volatile struct image_data s_ipc_image_data[IMAGE_CAMERA_COUNT];
#pragma location=".ipc_image_camera_seq"
volatile uint32 g_image_camera_seq[IMAGE_CAMERA_COUNT];
#pragma location=".ipc_image_fresh_mask"
volatile uint32 g_image_data_fresh_mask;
#pragma location=".ipc_image_seq"
volatile uint32 g_image_data_seq;
#pragma location=".ipc_image_guard"
volatile uint32 g_image_data_guard;
#pragma location=".ipc_camera_spi_log"
volatile ipc_camera_spi_log_t g_ipc_camera_spi_log;
#pragma location=".ipc_remote_param_request"
#pragma data_alignment=64
volatile ipc_remote_param_mailbox_t g_ipc_remote_param_request;
#pragma location=".ipc_remote_param_response"
#pragma data_alignment=64
volatile ipc_remote_param_mailbox_t g_ipc_remote_param_response;
#pragma location=".ipc_attitude_data"
#pragma data_alignment=64
volatile ipc_attitude_data_t g_ipc_attitude_data;

struct image_data image_data[IMAGE_CAMERA_COUNT];                               /* 本核使用的三路一致性图像工作副本。 */
volatile uint32 g_image_data_rx_seq;                                             /* CM7_0最近接收的一致性快照序号。 */
volatile uint32 g_image_camera_rx_seq[IMAGE_CAMERA_COUNT];                      /* CM7_0最近接收的三路真实结果序号。 */
volatile uint32 g_image_data_rx_fresh_mask;                                     /* CM7_0最近接收快照的新结果掩码。 */

#define IPC_FLIGHT_STATE_MAGIC   (0xA5000000UL)
#define IPC_FLIGHT_STATE_MASK    (0xFFFF0000UL)
#define IPC_FLIGHT_STATE_FLYING  (0x00000001UL)
#define IPC_FLIGHT_STATE_SCREEN_REFRESH_ENABLE (0x00000002UL)
#define IPC_FLIGHT_STATE_BL3_SCREEN_ENABLE (0x00000004UL)
#define IPC_FLIGHT_STATE_BL3_HORIZON_ENABLE (0x00000008UL)
#define IPC_FLIGHT_STATE_PARAM_WRITE_ENABLE (0x00000010UL)
/* 图传发送模式在 IPC 数据低 16 位中的偏移 */
#define IPC_IMAGE_SEND_SHIFT     (8U)
/* 图传发送模式在 IPC 数据低 16 位中的掩码 */
#define IPC_IMAGE_SEND_MASK      (0x0000FF00UL)
#define IPC_IMAGE_SNAPSHOT_MAX_ATTEMPTS (4U) /* CM7_0单次轮询最多尝试的一致性快照次数。 */

/* 远程参数邮箱校验盐值，避免全零内容被误判为合法事务。 */
#define IPC_REMOTE_PARAM_CHECKSUM_SALT (0xA57C31E9UL)

/* 计算远程参数邮箱协议字段的32位校验值。 */
static uint32 ipc_remote_param_checksum(const ipc_remote_param_mailbox_t *mailbox)
{
    uint32 meta;

    if(mailbox == NULL)
    {
        return 0U;
    }

    meta = ((uint32)mailbox->version << 16) | mailbox->param_id;
    meta ^= ((uint32)mailbox->op) |
            ((uint32)mailbox->type << 8) |
            ((uint32)mailbox->target << 16) |
            ((uint32)mailbox->status << 24);
    return IPC_REMOTE_PARAM_CHECKSUM_SALT ^ mailbox->magic ^ meta ^
           mailbox->transaction ^ mailbox->value_bits ^ mailbox->previous_bits;
}

/* 检查远程参数邮箱的版本、事务号和校验值。 */
static uint8 ipc_remote_param_mailbox_valid(const ipc_remote_param_mailbox_t *mailbox)
{
    if((mailbox == NULL) ||
       (mailbox->magic != IPC_REMOTE_PARAM_MAGIC) ||
       (mailbox->version != IPC_REMOTE_PARAM_VERSION) ||
       (mailbox->transaction == 0U) ||
       (mailbox->transaction > IPC_REMOTE_PARAM_TRANSACTION_MAX))
    {
        return 0U;
    }

    return (mailbox->checksum == ipc_remote_param_checksum(mailbox)) ? 1U : 0U;
}

#if defined(CY_CORE_CM7_1)

static uint32 s_tx_seq = 0U;
static volatile uint8 s_core0_flying = 0U;
/* 核0同步过来的 2BL3 图传发送模式 */
static volatile uint8 s_core0_image_send_enable = 0U;
/* 核1上电默认禁止刷屏，收到核0明确许可后才初始化IPS114。 */
static volatile uint8 s_core0_screen_refresh_enable = 0U;
static volatile uint8 s_core0_param_write_enable = 0U;
static volatile uint8 s_core0_bl3_screen_enable = 0U;
static volatile uint8 s_core0_bl3_horizon_enable = 0U;
static volatile uint8 s_remote_param_hint = 0U;
static uint32 s_remote_param_last_transaction = 0U;
static uint32 s_remote_param_last_checksum = 0U;
static uint8 s_remote_param_2bl3_active = 0U;
static uint8 s_remote_param_2bl3_cancel_requested = 0U;
static ipc_remote_param_mailbox_t s_remote_param_2bl3_request;

typedef struct
{
    uint8 valid;
    uint32 previous_bits;
    ipc_remote_param_mailbox_t request;
} ipc_remote_param_core1_set_cache_t;

/* 最近一次已回复成功的核1 SET，用于处理响应与取消墓碑交叉时的补偿回滚。 */
static ipc_remote_param_core1_set_cache_t s_remote_param_core1_set_cache;

/*
 * 函数功能: 将CM7_1本地图像结果一致性发布到共享内存并发送非阻塞通知。
 * 输入参数: fresh_mask为本次真实新算法结果对应的摄像头位掩码。
 * 返回值: 无。
 */
void ipc_image_publish(uint8 fresh_mask)
{
    uint8 camera_id;

    g_image_data_guard++;
    SCB_CleanDCache_by_Addr((volatile void *)&g_image_data_guard,
                            sizeof(g_image_data_guard));
    __DMB();

    memcpy((void *)s_ipc_image_data, image_data, sizeof(image_data));
    SCB_CleanDCache_by_Addr((volatile void *)s_ipc_image_data,
                            sizeof(s_ipc_image_data));
    for(camera_id = 0U; camera_id < IMAGE_CAMERA_COUNT; camera_id++)
    {
        if((fresh_mask & (uint8)(1U << camera_id)) != 0U)
        {
            g_image_camera_seq[camera_id]++;
        }
    }
    g_image_data_fresh_mask = fresh_mask;
    SCB_CleanDCache_by_Addr((volatile void *)g_image_camera_seq,
                            sizeof(g_image_camera_seq));
    SCB_CleanDCache_by_Addr((volatile void *)&g_image_data_fresh_mask,
                            sizeof(g_image_data_fresh_mask));
    __DMB();

    s_tx_seq++;
    g_image_data_seq = s_tx_seq;
    SCB_CleanDCache_by_Addr((volatile void *)&g_image_data_seq,
                            sizeof(g_image_data_seq));
    __DMB();
    g_image_data_guard++;
    SCB_CleanDCache_by_Addr((volatile void *)&g_image_data_guard,
                            sizeof(g_image_data_guard));
    __DSB();

    (void)ipc_try_send_data(s_tx_seq);
}

#else

static volatile uint8 s_remote_param_hint = 0U;
static uint8 s_remote_param_core0_active = 0U;
static uint32 s_remote_param_core0_transaction = 0U;
static uint32 s_remote_param_core0_counter = 0U;
static volatile uint8 s_image_data_hint = 0U;
static ipc_remote_param_mailbox_t s_remote_param_core0_request;
static uint32 s_attitude_sequence = 0U;
static uint32 s_image_data_last_seq = 0U;
static struct image_data s_image_data_snapshot[IMAGE_CAMERA_COUNT];

void ipc_image_publish(uint8 fresh_mask)
{
    (void)fresh_mask;
}

#endif

/*
 * 函数功能: 初始化本核图像工作副本及跨核发布或接收状态。
 * 输入参数: 无。
 * 返回值: 无。
 */
void ipc_image_init(void)
{
    uint8 camera_id;

    for(camera_id = 0U; camera_id < IMAGE_CAMERA_COUNT; camera_id++)
    {
        image_data_clear(&image_data[camera_id]);
    }
    g_image_data_rx_seq = 0U;
    g_image_data_rx_fresh_mask = 0U;
    memset((void *)g_image_camera_rx_seq, 0, sizeof(g_image_camera_rx_seq));

#if defined(CY_CORE_CM7_1)
    s_tx_seq = 0U;
    g_image_data_seq = 0U;
    g_image_data_fresh_mask = 0U;
    g_image_data_guard = 0U;
    memset((void *)g_image_camera_seq, 0, sizeof(g_image_camera_seq));
    memcpy((void *)s_ipc_image_data, image_data, sizeof(image_data));
    SCB_CleanDCache_by_Addr((volatile void *)s_ipc_image_data,
                            sizeof(s_ipc_image_data));
    SCB_CleanDCache_by_Addr((volatile void *)g_image_camera_seq,
                            sizeof(g_image_camera_seq));
    SCB_CleanDCache_by_Addr((volatile void *)&g_image_data_fresh_mask,
                            sizeof(g_image_data_fresh_mask));
    SCB_CleanDCache_by_Addr((volatile void *)&g_image_data_seq,
                            sizeof(g_image_data_seq));
    SCB_CleanDCache_by_Addr((volatile void *)&g_image_data_guard,
                            sizeof(g_image_data_guard));
    __DSB();
#else
    s_image_data_last_seq = 0U;
    s_image_data_hint = 0U;
    memset(s_image_data_snapshot, 0, sizeof(s_image_data_snapshot));
#endif
}

void ipc_attitude_publish(float roll_deg,
                          float pitch_deg,
                          float height_mm,
                          uint8 height_valid)
{
#if defined(CY_CORE_CM7_0)
    s_attitude_sequence++;
    if(s_attitude_sequence == 0U)
    {
        s_attitude_sequence = 1U;
    }
    g_ipc_attitude_data.roll_deg = roll_deg;
    g_ipc_attitude_data.pitch_deg = pitch_deg;
    g_ipc_attitude_data.height_mm = height_mm;
    g_ipc_attitude_data.flags = (height_valid != 0U) ?
                                IPC_ATTITUDE_FLAG_HEIGHT_VALID : 0U;
    g_ipc_attitude_data.sequence = s_attitude_sequence;
    SCB_CleanDCache_by_Addr((volatile void *)&g_ipc_attitude_data,
                            sizeof(g_ipc_attitude_data));
#else
    (void)roll_deg;
    (void)pitch_deg;
    (void)height_mm;
    (void)height_valid;
#endif
}

void ipc_attitude_get(ipc_attitude_data_t *out)
{
    if(out != NULL)
    {
#if defined(CY_CORE_CM7_1)
        SCB_InvalidateDCache_by_Addr((volatile void *)&g_ipc_attitude_data,
                                    sizeof(g_ipc_attitude_data));
#endif
        memcpy(out, (const void *)&g_ipc_attitude_data, sizeof(*out));
    }
}

#if defined(CY_CORE_CM7_0)
static uint8 ipc_remote_param_response_matches_request(
    const ipc_remote_param_mailbox_t *response)
{
    if((response == NULL) ||
       (response->transaction != s_remote_param_core0_transaction) ||
       (response->op != s_remote_param_core0_request.op) ||
       (response->type != s_remote_param_core0_request.type) ||
       (response->target != s_remote_param_core0_request.target) ||
       (response->param_id != s_remote_param_core0_request.param_id))
    {
        return 0U;
    }

    return 1U;
}

static uint8 ipc_remote_param_is_cancel_terminal(
    const ipc_remote_param_mailbox_t *response)
{
    if(response == NULL)
    {
        return 0U;
    }

    return ((response->status == IPC_REMOTE_PARAM_STATUS_TIMEOUT) ||
            (response->status == IPC_REMOTE_PARAM_STATUS_ROLLBACK_FAIL)) ? 1U : 0U;
}

/* SET硬超时保留取消标记；GET可立即释放，迟到事务由核1在新请求到达时清理。 */
static void ipc_remote_param_core0_reap_cancelled(void)
{
    ipc_remote_param_mailbox_t response;

    if((s_remote_param_core0_active == 0U) ||
       (s_remote_param_core0_request.status != IPC_REMOTE_PARAM_STATUS_TIMEOUT))
    {
        return;
    }

    SCB_InvalidateDCache_by_Addr((volatile void *)&g_ipc_remote_param_response,
                                 sizeof(g_ipc_remote_param_response));
    memcpy(&response, (const void *)&g_ipc_remote_param_response, sizeof(response));
    if((ipc_remote_param_mailbox_valid(&response) != 0U) &&
       (ipc_remote_param_response_matches_request(&response) != 0U) &&
       (ipc_remote_param_is_cancel_terminal(&response) != 0U))
    {
        s_remote_param_core0_active = 0U;
        s_remote_param_hint = 0U;
    }
}
#endif

/* 核1发布最终参数响应，只有目标端赋值并读回后才调用。 */
static void ipc_remote_param_publish_response(const ipc_remote_param_mailbox_t *request,
                                              uint8 status,
                                              uint32 actual_bits)
{
#if defined(CY_CORE_CM7_1)
    ipc_remote_param_mailbox_t response;

    if(request == NULL)
    {
        return;
    }

    memset(&response, 0, sizeof(response));
    response.magic = IPC_REMOTE_PARAM_MAGIC;
    response.version = IPC_REMOTE_PARAM_VERSION;
    response.op = request->op;
    response.type = request->type;
    response.target = request->target;
    response.status = status;
    response.param_id = request->param_id;
    response.transaction = request->transaction;
    response.value_bits = actual_bits;
    response.previous_bits = request->previous_bits;
    response.checksum = ipc_remote_param_checksum(&response);

    memcpy((void *)&g_ipc_remote_param_response, &response, sizeof(response));
    SCB_CleanDCache_by_Addr((volatile void *)&g_ipc_remote_param_response,
                            sizeof(g_ipc_remote_param_response));
    (void)ipc_try_send_data(IPC_REMOTE_PARAM_NOTIFY_MAGIC |
                            (request->transaction & 0x0000FFFFUL));
#else
    (void)request;
    (void)status;
    (void)actual_bits;
#endif
}

uint8 ipc_flight_state_send(uint8 flying,
                            uint8 image_send_enable,
                            uint8 screen_refresh_enable,
                            uint8 param_write_enable,
                            uint8 bl3_screen_enable,
                            uint8 bl3_horizon_enable)
{
#if defined(CY_CORE_CM7_0)
    uint32 ipc_data = IPC_FLIGHT_STATE_MAGIC;

    if(image_send_enable > 2U)
    {
        image_send_enable = 0U;
    }

    if (0U != flying)
    {
        ipc_data |= IPC_FLIGHT_STATE_FLYING;
    }
    if(0U != screen_refresh_enable)
    {
        ipc_data |= IPC_FLIGHT_STATE_SCREEN_REFRESH_ENABLE;
    }
    if(0U != param_write_enable)
    {
        ipc_data |= IPC_FLIGHT_STATE_PARAM_WRITE_ENABLE;
    }
    if(0U != bl3_screen_enable)
    {
        ipc_data |= IPC_FLIGHT_STATE_BL3_SCREEN_ENABLE;
    }
    if(0U != bl3_horizon_enable)
    {
        ipc_data |= IPC_FLIGHT_STATE_BL3_HORIZON_ENABLE;
    }
    ipc_data |= (((uint32)image_send_enable << IPC_IMAGE_SEND_SHIFT) & IPC_IMAGE_SEND_MASK);

    return ipc_try_send_data(ipc_data);
#else
    (void)flying;
    (void)image_send_enable;
    (void)screen_refresh_enable;
    (void)param_write_enable;
    (void)bl3_screen_enable;
    (void)bl3_horizon_enable;
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

uint8 ipc_core0_image_send_enable(void)
{
#if defined(CY_CORE_CM7_1)
    return s_core0_image_send_enable;
#else
    return 0U;
#endif
}

uint8 ipc_core0_screen_refresh_enable(void)
{
#if defined(CY_CORE_CM7_1)
    return s_core0_screen_refresh_enable;
#else
    return 0U;
#endif
}

uint8 ipc_core0_param_write_enable(void)
{
#if defined(CY_CORE_CM7_1)
    return s_core0_param_write_enable;
#else
    return 0U;
#endif
}

uint8 ipc_core0_bl3_screen_enable(void)
{
#if defined(CY_CORE_CM7_1)
    return s_core0_bl3_screen_enable;
#else
    return 0U;
#endif
}

uint8 ipc_core0_bl3_horizon_enable(void)
{
#if defined(CY_CORE_CM7_1)
    return s_core0_bl3_horizon_enable;
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

/* 初始化核0请求端，并清除核0拥有的请求邮箱。 */
void ipc_remote_param_core0_init(void)
{
#if defined(CY_CORE_CM7_0)
    ipc_remote_param_mailbox_t response;

    SCB_InvalidateDCache_by_Addr((volatile void *)&g_ipc_remote_param_response,
                                 sizeof(g_ipc_remote_param_response));
    memcpy(&response, (const void *)&g_ipc_remote_param_response, sizeof(response));
    memset((void *)&g_ipc_remote_param_request, 0, sizeof(g_ipc_remote_param_request));
    SCB_CleanDCache_by_Addr((volatile void *)&g_ipc_remote_param_request,
                            sizeof(g_ipc_remote_param_request));
    s_remote_param_core0_active = 0U;
    s_remote_param_core0_transaction = 0U;
    s_remote_param_core0_counter = (ipc_remote_param_mailbox_valid(&response) != 0U) ?
                                   (response.transaction & IPC_REMOTE_PARAM_TRANSACTION_MAX) : 0U;
    memset(&s_remote_param_core0_request, 0, sizeof(s_remote_param_core0_request));
    s_remote_param_hint = 0U;
#endif
}

/* 核0发布一笔远程参数请求，IPC Pipe仅作为加速提示。 */
uint8 ipc_remote_param_core0_start(uint8 target,
                                   uint8 op,
                                   uint8 type,
                                   uint16 param_id,
                                   uint32 value_bits,
                                   uint32 previous_bits,
                                   uint32 *transaction_out)
{
#if defined(CY_CORE_CM7_0)
    ipc_remote_param_mailbox_t request;

    ipc_remote_param_core0_reap_cancelled();

    if((transaction_out == NULL) ||
       (s_remote_param_core0_active != 0U) ||
       ((target != IPC_REMOTE_PARAM_TARGET_CORE1) &&
        (target != IPC_REMOTE_PARAM_TARGET_2BL3)) ||
       ((op != IPC_REMOTE_PARAM_OP_SET) && (op != IPC_REMOTE_PARAM_OP_GET)) ||
       (type > IPC_REMOTE_PARAM_TYPE_INT32) ||
       (param_id == 0U))
    {
        return 0U;
    }

    if(s_remote_param_core0_counter >= IPC_REMOTE_PARAM_TRANSACTION_MAX)
    {
        s_remote_param_core0_counter = 1U;
    }
    else
    {
        s_remote_param_core0_counter++;
    }

    memset(&request, 0, sizeof(request));
    request.magic = IPC_REMOTE_PARAM_MAGIC;
    request.version = IPC_REMOTE_PARAM_VERSION;
    request.op = op;
    request.type = type;
    request.target = target;
    request.status = IPC_REMOTE_PARAM_STATUS_OK;
    request.param_id = param_id;
    request.transaction = s_remote_param_core0_counter;
    request.value_bits = value_bits;
    request.previous_bits = previous_bits;
    request.checksum = ipc_remote_param_checksum(&request);

    s_remote_param_core0_active = 1U;
    s_remote_param_core0_transaction = request.transaction;
    s_remote_param_core0_request = request;
    s_remote_param_hint = 0U;
    memcpy((void *)&g_ipc_remote_param_request, &request, sizeof(request));
    SCB_CleanDCache_by_Addr((volatile void *)&g_ipc_remote_param_request,
                            sizeof(g_ipc_remote_param_request));
    *transaction_out = request.transaction;
    (void)ipc_send_data(IPC_REMOTE_PARAM_NOTIFY_MAGIC |
                        (request.transaction & 0x0000FFFFUL));
    return 1U;
#else
    (void)target;
    (void)op;
    (void)type;
    (void)param_id;
    (void)value_bits;
    (void)previous_bits;
    (void)transaction_out;
    return 0U;
#endif
}

/* 核0高频轮询响应邮箱，不依赖IPC中断回调。 */
uint8 ipc_remote_param_core0_poll(ipc_remote_param_mailbox_t *response)
{
#if defined(CY_CORE_CM7_0)
    ipc_remote_param_mailbox_t snapshot;

    if((response == NULL) || (s_remote_param_core0_active == 0U))
    {
        return 0U;
    }

    SCB_InvalidateDCache_by_Addr((volatile void *)&g_ipc_remote_param_response,
                                 sizeof(g_ipc_remote_param_response));
    memcpy(&snapshot, (const void *)&g_ipc_remote_param_response, sizeof(snapshot));
    if((ipc_remote_param_mailbox_valid(&snapshot) == 0U) ||
       (ipc_remote_param_response_matches_request(&snapshot) == 0U) ||
       ((s_remote_param_core0_request.status == IPC_REMOTE_PARAM_STATUS_TIMEOUT) &&
        (ipc_remote_param_is_cancel_terminal(&snapshot) == 0U)))
    {
        return 0U;
    }
    *response = snapshot;
    s_remote_param_core0_active = 0U;
    s_remote_param_hint = 0U;
    return 1U;
#else
    (void)response;
    return 0U;
#endif
}

/* 核0发布同事务取消标记，但继续等待核1确认下游已停止或完成回滚。 */
uint8 ipc_remote_param_core0_request_cancel(uint32 transaction)
{
#if defined(CY_CORE_CM7_0)
    if((s_remote_param_core0_active == 0U) ||
       (s_remote_param_core0_transaction != transaction))
    {
        return 0U;
    }

    s_remote_param_core0_request.status = IPC_REMOTE_PARAM_STATUS_TIMEOUT;
    s_remote_param_core0_request.checksum =
        ipc_remote_param_checksum(&s_remote_param_core0_request);
    memcpy((void *)&g_ipc_remote_param_request,
           &s_remote_param_core0_request,
           sizeof(s_remote_param_core0_request));
    SCB_CleanDCache_by_Addr((volatile void *)&g_ipc_remote_param_request,
                            sizeof(g_ipc_remote_param_request));
    s_remote_param_hint = 0U;
    (void)ipc_send_data(IPC_REMOTE_PARAM_NOTIFY_MAGIC |
                        (transaction & 0x0000FFFFUL));
    return 1U;
#else
    (void)transaction;
    return 0U;
#endif
}

/* GET硬超时立即释放Core0；SET继续占用邮箱，等待核1完成回滚。 */
void ipc_remote_param_core0_cancel(uint32 transaction)
{
#if defined(CY_CORE_CM7_0)
    if((s_remote_param_core0_active != 0U) &&
       (s_remote_param_core0_transaction == transaction))
    {
        (void)ipc_remote_param_core0_request_cancel(transaction);
        if(s_remote_param_core0_request.op == IPC_REMOTE_PARAM_OP_GET)
        {
            s_remote_param_core0_active = 0U;
        }
        s_remote_param_hint = 0U;
    }
#else
    (void)transaction;
#endif
}

uint8 ipc_remote_param_core0_is_busy(void)
{
#if defined(CY_CORE_CM7_0)
    ipc_remote_param_core0_reap_cancelled();
    return s_remote_param_core0_active;
#else
    return 0U;
#endif
}

/* 初始化核1执行端，并清除核1拥有的响应邮箱。 */
void ipc_remote_param_core1_init(void)
{
#if defined(CY_CORE_CM7_1)
    memset((void *)&g_ipc_remote_param_response, 0, sizeof(g_ipc_remote_param_response));
    SCB_CleanDCache_by_Addr((volatile void *)&g_ipc_remote_param_response,
                            sizeof(g_ipc_remote_param_response));
    memset(&s_remote_param_2bl3_request, 0, sizeof(s_remote_param_2bl3_request));
    memset(&s_remote_param_core1_set_cache, 0, sizeof(s_remote_param_core1_set_cache));
    s_remote_param_last_transaction = 0U;
    s_remote_param_last_checksum = 0U;
    s_remote_param_2bl3_active = 0U;
    s_remote_param_2bl3_cancel_requested = 0U;
    s_remote_param_hint = 0U;
#endif
}

/* 核1在100Hz帧边界执行本核参数，或推进两颗2BL3广播事务。 */
void ipc_remote_param_core1_poll(void)
{
#if defined(CY_CORE_CM7_1)
    ipc_remote_param_mailbox_t request;
    camera_spi_remote_param_result_t result;
    uint32 actual_bits = 0U;
    uint32 previous_bits = 0U;
    uint8 status;

    if(s_remote_param_2bl3_active != 0U)
    {
        SCB_InvalidateDCache_by_Addr((volatile void *)&g_ipc_remote_param_request,
                                     sizeof(g_ipc_remote_param_request));
        memcpy(&request, (const void *)&g_ipc_remote_param_request, sizeof(request));
        if((ipc_remote_param_mailbox_valid(&request) != 0U) &&
           (request.transaction != s_remote_param_2bl3_request.transaction) &&
           (s_remote_param_2bl3_request.op == IPC_REMOTE_PARAM_OP_GET))
        {
            (void)CameraSpi_RemoteParamCancel(
                s_remote_param_2bl3_request.transaction);
            (void)CameraSpi_RemoteParamTakeResult(&result);
            s_remote_param_2bl3_active = 0U;
            s_remote_param_2bl3_cancel_requested = 0U;
        }
        else
        {
            if((ipc_remote_param_mailbox_valid(&request) != 0U) &&
               (request.transaction == s_remote_param_2bl3_request.transaction) &&
               (request.status != IPC_REMOTE_PARAM_STATUS_OK))
            {
                s_remote_param_last_transaction = request.transaction;
                s_remote_param_last_checksum = request.checksum;
                if(s_remote_param_2bl3_cancel_requested == 0U)
                {
                    s_remote_param_2bl3_cancel_requested = 1U;
                    (void)CameraSpi_RemoteParamCancel(request.transaction);
                }
            }

            if(CameraSpi_RemoteParamTakeResult(&result) != 0U)
            {
                status = result.status;
                actual_bits = result.actual_bits;
                if(s_remote_param_2bl3_cancel_requested != 0U)
                {
                    if(status != IPC_REMOTE_PARAM_STATUS_ROLLBACK_FAIL)
                    {
                        status = IPC_REMOTE_PARAM_STATUS_TIMEOUT;
                        actual_bits = s_remote_param_2bl3_request.previous_bits;
                    }
                }
                ipc_remote_param_publish_response(&s_remote_param_2bl3_request,
                                                  status,
                                                  actual_bits);
                s_remote_param_2bl3_active = 0U;
                s_remote_param_2bl3_cancel_requested = 0U;
            }
            return;
        }
    }

    SCB_InvalidateDCache_by_Addr((volatile void *)&g_ipc_remote_param_request,
                                 sizeof(g_ipc_remote_param_request));
    memcpy(&request, (const void *)&g_ipc_remote_param_request, sizeof(request));
    s_remote_param_hint = 0U;
    if(ipc_remote_param_mailbox_valid(&request) == 0U)
    {
        s_remote_param_last_transaction = 0U;
        s_remote_param_last_checksum = 0U;
        return;
    }
    if((request.transaction == s_remote_param_last_transaction) &&
       (request.checksum == s_remote_param_last_checksum))
    {
        return;
    }

    s_remote_param_last_transaction = request.transaction;
    s_remote_param_last_checksum = request.checksum;
    if(request.status != IPC_REMOTE_PARAM_STATUS_OK)
    {
        if((s_remote_param_core1_set_cache.valid != 0U) &&
           (request.transaction == s_remote_param_core1_set_cache.request.transaction) &&
           (request.target == IPC_REMOTE_PARAM_TARGET_CORE1) &&
           (request.op == IPC_REMOTE_PARAM_OP_SET) &&
           (request.type == s_remote_param_core1_set_cache.request.type) &&
           (request.param_id == s_remote_param_core1_set_cache.request.param_id) &&
           (request.value_bits == s_remote_param_core1_set_cache.request.value_bits))
        {
            previous_bits = s_remote_param_core1_set_cache.previous_bits;
            status = image_down_remote_param_execute(IPC_REMOTE_PARAM_OP_SET,
                                                     request.type,
                                                     request.param_id,
                                                     previous_bits,
                                                     &actual_bits);
            if(status == IPC_REMOTE_PARAM_STATUS_OK)
            {
                status = image_down_remote_param_execute(IPC_REMOTE_PARAM_OP_GET,
                                                         request.type,
                                                         request.param_id,
                                                         0U,
                                                         &actual_bits);
            }
            ipc_remote_param_publish_response(
                &request,
                ((status == IPC_REMOTE_PARAM_STATUS_OK) &&
                 (actual_bits == previous_bits)) ?
                    IPC_REMOTE_PARAM_STATUS_TIMEOUT :
                    IPC_REMOTE_PARAM_STATUS_ROLLBACK_FAIL,
                actual_bits);
            s_remote_param_core1_set_cache.valid = 0U;
        }
        else if((request.target == IPC_REMOTE_PARAM_TARGET_2BL3) &&
                (CameraSpi_RemoteParamCancel(request.transaction) != 0U))
        {
            s_remote_param_2bl3_request = request;
            s_remote_param_2bl3_active = 1U;
            s_remote_param_2bl3_cancel_requested = 1U;
        }
        else
        {
            ipc_remote_param_publish_response(&request,
                                              IPC_REMOTE_PARAM_STATUS_TIMEOUT,
                                              request.previous_bits);
        }
    }
    else
    {
        /* 新的正常请求意味着上一笔成功响应已被核0接收，不再允许迟到取消回滚。 */
        s_remote_param_core1_set_cache.valid = 0U;
        if(request.target == IPC_REMOTE_PARAM_TARGET_CORE1)
        {
            if(request.op == IPC_REMOTE_PARAM_OP_SET)
            {
                status = image_down_remote_param_execute(IPC_REMOTE_PARAM_OP_GET,
                                                         request.type,
                                                         request.param_id,
                                                         0U,
                                                         &previous_bits);
                if(status == IPC_REMOTE_PARAM_STATUS_OK)
                {
                    status = image_down_remote_param_execute(request.op,
                                                             request.type,
                                                             request.param_id,
                                                             request.value_bits,
                                                             &actual_bits);
                }
                else
                {
                    actual_bits = previous_bits;
                }

                if((status == IPC_REMOTE_PARAM_STATUS_OK) &&
                   (actual_bits == request.value_bits))
                {
                    s_remote_param_core1_set_cache.valid = 1U;
                    s_remote_param_core1_set_cache.previous_bits = previous_bits;
                    s_remote_param_core1_set_cache.request = request;
                }
            }
            else
            {
                status = image_down_remote_param_execute(request.op,
                                                         request.type,
                                                         request.param_id,
                                                         request.value_bits,
                                                         &actual_bits);
            }
            ipc_remote_param_publish_response(&request, status, actual_bits);
        }
        else if(request.target == IPC_REMOTE_PARAM_TARGET_2BL3)
        {
            if(CameraSpi_RemoteParamBoardsOnline() == 0U)
            {
                ipc_remote_param_publish_response(&request,
                                                  IPC_REMOTE_PARAM_STATUS_TIMEOUT,
                                                  request.previous_bits);
            }
            else if(CameraSpi_RemoteParamStart(request.op,
                                          request.type,
                                          request.param_id,
                                          request.transaction,
                                          request.value_bits,
                                          request.previous_bits) != 0U)
            {
                s_remote_param_2bl3_request = request;
                s_remote_param_2bl3_active = 1U;
                s_remote_param_2bl3_cancel_requested = 0U;
            }
            else
            {
                ipc_remote_param_publish_response(&request,
                                                  IPC_REMOTE_PARAM_STATUS_BUSY,
                                                  request.previous_bits);
            }
        }
        else
        {
            ipc_remote_param_publish_response(&request,
                                              IPC_REMOTE_PARAM_STATUS_NOT_FOUND,
                                              request.previous_bits);
        }
    }
#endif
}

void ipc_image_callback(uint32 ipc_data)
{
#if defined(CY_CORE_CM7_0)
    if((ipc_data & IPC_REMOTE_PARAM_NOTIFY_MASK) == IPC_REMOTE_PARAM_NOTIFY_MAGIC)
    {
        s_remote_param_hint = 1U;
    }
    else
    {
        s_image_data_hint = 1U;
    }
#elif defined(CY_CORE_CM7_1)
    if((ipc_data & IPC_REMOTE_PARAM_NOTIFY_MASK) == IPC_REMOTE_PARAM_NOTIFY_MAGIC)
    {
        s_remote_param_hint = 1U;
    }
    else if ((ipc_data & IPC_FLIGHT_STATE_MASK) == IPC_FLIGHT_STATE_MAGIC)
    {
        s_core0_flying = (0U != (ipc_data & IPC_FLIGHT_STATE_FLYING)) ? 1U : 0U;
        s_core0_screen_refresh_enable =
            (0U != (ipc_data & IPC_FLIGHT_STATE_SCREEN_REFRESH_ENABLE)) ? 1U : 0U;
        s_core0_param_write_enable =
            (0U != (ipc_data & IPC_FLIGHT_STATE_PARAM_WRITE_ENABLE)) ? 1U : 0U;
        s_core0_bl3_screen_enable =
            (0U != (ipc_data & IPC_FLIGHT_STATE_BL3_SCREEN_ENABLE)) ? 1U : 0U;
        s_core0_bl3_horizon_enable =
            (0U != (ipc_data & IPC_FLIGHT_STATE_BL3_HORIZON_ENABLE)) ? 1U : 0U;
        s_core0_image_send_enable = (uint8)((ipc_data & IPC_IMAGE_SEND_MASK) >> IPC_IMAGE_SEND_SHIFT);
        if(s_core0_image_send_enable > 2U)
        {
            s_core0_image_send_enable = 0U;
        }
    }
#else
    (void)ipc_data;
#endif
}

/*
 * 函数功能: CM7_0主动读取并提交一份跨核一致性图像快照。
 * 输入参数: 无。
 * 返回值: 接收到新发布快照返回1，否则返回0。
 */
uint8 ipc_image_poll(void)
{
#if defined(CY_CORE_CM7_0)
    uint8 attempt;
    uint8 snapshot_valid = 0U;
    uint32 guard_before;
    uint32 guard_after;
    uint32 snapshot_seq = 0U;
    uint32 snapshot_fresh_mask = 0U;
    uint32 camera_seq_snapshot[IMAGE_CAMERA_COUNT];

    for(attempt = 0U; attempt < IPC_IMAGE_SNAPSHOT_MAX_ATTEMPTS; attempt++)
    {
        SCB_InvalidateDCache_by_Addr((volatile void *)&g_image_data_guard,
                                     sizeof(g_image_data_guard));
        guard_before = g_image_data_guard;
        if((guard_before & 1U) != 0U)
        {
            continue;
        }

        SCB_InvalidateDCache_by_Addr((volatile void *)s_ipc_image_data,
                                     sizeof(s_ipc_image_data));
        SCB_InvalidateDCache_by_Addr((volatile void *)g_image_camera_seq,
                                     sizeof(g_image_camera_seq));
        SCB_InvalidateDCache_by_Addr((volatile void *)&g_image_data_fresh_mask,
                                     sizeof(g_image_data_fresh_mask));
        SCB_InvalidateDCache_by_Addr((volatile void *)&g_image_data_seq,
                                     sizeof(g_image_data_seq));
        __DMB();
        memcpy(s_image_data_snapshot,
               (const void *)s_ipc_image_data,
               sizeof(s_image_data_snapshot));
        memcpy(camera_seq_snapshot,
               (const void *)g_image_camera_seq,
               sizeof(camera_seq_snapshot));
        snapshot_fresh_mask = g_image_data_fresh_mask;
        snapshot_seq = g_image_data_seq;
        __DMB();
        SCB_InvalidateDCache_by_Addr((volatile void *)&g_image_data_guard,
                                     sizeof(g_image_data_guard));
        guard_after = g_image_data_guard;
        if((guard_before == guard_after) && ((guard_after & 1U) == 0U))
        {
            snapshot_valid = 1U;
            break;
        }
    }

    s_image_data_hint = 0U;
    if((snapshot_valid == 0U) || (snapshot_seq == s_image_data_last_seq))
    {
        return 0U;
    }

    memcpy(image_data, s_image_data_snapshot, sizeof(image_data));
    memcpy((void *)g_image_camera_rx_seq,
           camera_seq_snapshot,
           sizeof(g_image_camera_rx_seq));
    g_image_data_rx_fresh_mask = snapshot_fresh_mask;
    g_image_data_rx_seq = snapshot_seq;
    s_image_data_last_seq = snapshot_seq;
    __DMB();
    return 1U;
#else
    return 0U;
#endif
}
