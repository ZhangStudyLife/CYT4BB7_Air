#ifndef IPC_IMAGE_DATA_H_
#define IPC_IMAGE_DATA_H_

#include "zf_common_headfile.h"
#include "Image/image_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IPC_CAMERA_SPI_BOARD_COUNT (2U)

/* 跨核远程参数协议固定版本。 */
#define IPC_REMOTE_PARAM_MAGIC                 (0x5250414DUL)
#define IPC_REMOTE_PARAM_VERSION               (1U)
#define IPC_REMOTE_PARAM_NOTIFY_MAGIC          (0x52500000UL)
#define IPC_REMOTE_PARAM_NOTIFY_MASK           (0xFFFF0000UL)
#define IPC_REMOTE_PARAM_TRANSACTION_MAX       (0x3FFFFFFFUL)

/* 远程参数操作码。 */
#define IPC_REMOTE_PARAM_OP_SET                (1U)
#define IPC_REMOTE_PARAM_OP_GET                (2U)

/* 远程参数数值类型。 */
#define IPC_REMOTE_PARAM_TYPE_FLOAT            (0U)
#define IPC_REMOTE_PARAM_TYPE_INT32            (1U)

/* 远程参数目标。 */
#define IPC_REMOTE_PARAM_TARGET_CORE1          (1U)
#define IPC_REMOTE_PARAM_TARGET_2BL3           (2U)

/* 远程参数统一状态码，与空地通信参数 ACK 保持一致。 */
#define IPC_REMOTE_PARAM_STATUS_OK             (0U)
#define IPC_REMOTE_PARAM_STATUS_NOT_FOUND      (1U)
#define IPC_REMOTE_PARAM_STATUS_OUT_OF_RANGE   (2U)
#define IPC_REMOTE_PARAM_STATUS_ERROR          (3U)
#define IPC_REMOTE_PARAM_STATUS_BUSY           (4U)
#define IPC_REMOTE_PARAM_STATUS_TIMEOUT        (5U)
#define IPC_REMOTE_PARAM_STATUS_MISMATCH       (6U)
#define IPC_REMOTE_PARAM_STATUS_PARTIAL        (7U)
#define IPC_REMOTE_PARAM_STATUS_ROLLBACK_FAIL  (8U)

/* 首批图像参数 ID。不同目标拥有独立 ID 空间。 */
#define IPC_REMOTE_PARAM_ID_BEACON_THRESHOLD   (1U)
#define IPC_REMOTE_PARAM_ID_EXP_TIME           (2U)
#define IPC_REMOTE_PARAM_ID_SCREEN_MODE        (3U)

/* 固定64字节共享邮箱，reserved用于后续兼容扩展。 */
typedef struct
{
    uint32 magic;
    uint16 version;
    uint8 op;
    uint8 type;
    uint8 target;
    uint8 status;
    uint16 param_id;
    uint32 transaction;
    uint32 value_bits;
    uint32 previous_bits;
    uint32 checksum;
    uint8 reserved[36];
} ipc_remote_param_mailbox_t;

typedef char ipc_remote_param_mailbox_size_must_be_64[
    (sizeof(ipc_remote_param_mailbox_t) == 64U) ? 1 : -1];

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
extern volatile ipc_remote_param_mailbox_t g_ipc_remote_param_request;
extern volatile ipc_remote_param_mailbox_t g_ipc_remote_param_response;

void ipc_image_callback(uint32 ipc_data);
void ipc_image_publish(void);
void ipc_image_poll(void);

uint8 ipc_flight_state_send(uint8 flying,
                            uint8 image_send_enable,
                            uint8 screen_refresh_enable);
uint8 ipc_core0_is_flying(void);
uint8 ipc_core0_image_send_enable(void);
uint8 ipc_core0_screen_refresh_enable(void);
void ipc_camera_spi_log_publish(const ipc_camera_spi_log_t *log);
void ipc_camera_spi_log_get(ipc_camera_spi_log_t *out);

/* 初始化核0远程参数请求端。 */
void ipc_remote_param_core0_init(void);

/* 核0发布一笔远程参数请求，返回1表示请求已发布。 */
uint8 ipc_remote_param_core0_start(uint8 target,
                                   uint8 op,
                                   uint8 type,
                                   uint16 param_id,
                                   uint32 value_bits,
                                   uint32 previous_bits,
                                   uint32 *transaction_out);

/* 核0轮询响应邮箱；取消墓碑存在时仅消费TIMEOUT或ROLLBACK_FAIL终态。 */
uint8 ipc_remote_param_core0_poll(ipc_remote_param_mailbox_t *response);

/* 核0发布取消标记但保持事务活动，等待核1返回最终取消结果。 */
uint8 ipc_remote_param_core0_request_cancel(uint32 transaction);

/* 核0硬超时后保留取消标记，核1返回终态前禁止新事务覆盖邮箱。 */
void ipc_remote_param_core0_cancel(uint32 transaction);

/* 查询核0IPC参数邮箱是否仍被活动事务或取消墓碑占用。 */
uint8 ipc_remote_param_core0_is_busy(void);

/* 初始化核1远程参数执行端。 */
void ipc_remote_param_core1_init(void);

/* 核1在100Hz主循环帧边界轮询并推进远程参数事务。 */
void ipc_remote_param_core1_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* IPC_IMAGE_DATA_H_ */
