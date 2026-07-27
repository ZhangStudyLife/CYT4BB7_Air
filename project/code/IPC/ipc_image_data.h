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

/* Core1图像算法标量参数固定ID。 */
#define IPC_REMOTE_PARAM_ID_C1_BEACON_THR      (0x0300U)
#define IPC_REMOTE_PARAM_ID_C1_EXP_TIME        (0x0301U)
#define IPC_REMOTE_PARAM_ID_C1_SCREEN_MODE     (0x0302U)
#define IPC_REMOTE_PARAM_ID_C1_BEACON_MIN      (0x0303U)
#define IPC_REMOTE_PARAM_ID_C1_EDGE_MIN        (0x0304U)
#define IPC_REMOTE_PARAM_ID_C1_EDGE_THR        (0x0305U)
#define IPC_REMOTE_PARAM_ID_C1_LAMP_THR        (0x0306U)
#define IPC_REMOTE_PARAM_ID_C1_LAMP_MIN        (0x0307U)
#define IPC_REMOTE_PARAM_ID_C1_LAMP_MAX        (0x0308U)
#define IPC_REMOTE_PARAM_ID_C1_LAMP_ELONG      (0x0309U)
#define IPC_REMOTE_PARAM_ID_C1_LAMP_LEN        (0x030AU)
#define IPC_REMOTE_PARAM_ID_C1_NEAR_PAD        (0x030BU)
#define IPC_REMOTE_PARAM_ID_C1_NEAR_MIN        (0x030CU)
#define IPC_REMOTE_PARAM_ID_C1_NEAR_ISO_MIN    (0x030DU)
#define IPC_REMOTE_PARAM_ID_C1_NEAR_BG         (0x030EU)
#define IPC_REMOTE_PARAM_ID_C1_MATCH_DIST      (0x030FU)
#define IPC_REMOTE_PARAM_ID_C1_GATE_DIST       (0x0310U)
#define IPC_REMOTE_PARAM_ID_C1_NEW_DIST        (0x0311U)
#define IPC_REMOTE_PARAM_ID_C1_CONFIRM         (0x0312U)
#define IPC_REMOTE_PARAM_ID_C1_MISSES          (0x0313U)
#define IPC_REMOTE_PARAM_ID_C1_POS_ALPHA       (0x0314U)
#define IPC_REMOTE_PARAM_ID_C1_VEL_ALPHA       (0x0315U)

/* 两颗2BL3信标算法标量参数固定 ID。 */
#define IPC_REMOTE_PARAM_ID_BL3_EDGE_THR        (0x0100U) /* 边缘区域二值化阈值 */
#define IPC_REMOTE_PARAM_ID_BL3_TRACK_THR       (0x0101U) /* 跟踪补强二值化阈值 */
#define IPC_REMOTE_PARAM_ID_BL3_LAMP_THR        (0x0102U) /* 车灯普通区域二值化阈值 */
#define IPC_REMOTE_PARAM_ID_BL3_LAMP_UP_THR     (0x0103U) /* 车灯上部区域二值化阈值 */
#define IPC_REMOTE_PARAM_ID_BL3_LAMP_UP_Y       (0x0104U) /* 车灯上部区域纵向边界 */
#define IPC_REMOTE_PARAM_ID_BL3_BRIDGE_GAP      (0x0105U) /* 车灯横向暗缝最大连接宽度 */
#define IPC_REMOTE_PARAM_ID_BL3_BEACON_MIN      (0x0106U) /* 普通信标最小连通域面积 */
#define IPC_REMOTE_PARAM_ID_BL3_EDGE_MIN        (0x0107U) /* 边缘信标最小连通域面积 */
#define IPC_REMOTE_PARAM_ID_BL3_TOP_MAX         (0x0108U) /* 顶部信标最大连通域面积 */
#define IPC_REMOTE_PARAM_ID_BL3_EDGE_MAX        (0x0109U) /* 侧边信标最大连通域面积 */
#define IPC_REMOTE_PARAM_ID_BL3_LAMP_MIN        (0x010AU) /* 车灯最小连通域面积 */
#define IPC_REMOTE_PARAM_ID_BL3_LAMP_MAX        (0x010BU) /* 车灯普通最大连通域面积 */
#define IPC_REMOTE_PARAM_ID_BL3_LAMP_ELONG      (0x010DU) /* 车灯最小长宽比 */
#define IPC_REMOTE_PARAM_ID_BL3_BACK_LEN        (0x010FU) /* 后摄车灯最小长轴长度 */
#define IPC_REMOTE_PARAM_ID_BL3_ISO_GRAY        (0x0110U) /* 孤立小信标最小峰值灰度 */
#define IPC_REMOTE_PARAM_ID_BL3_ISO_BG          (0x0111U) /* 孤立小信标最大背景灰度 */
#define IPC_REMOTE_PARAM_ID_BL3_RING_IN         (0x0112U) /* 局部背景采样方环内半径 */
#define IPC_REMOTE_PARAM_ID_BL3_RING_OUT        (0x0113U) /* 局部背景采样方环外半径 */
#define IPC_REMOTE_PARAM_ID_BL3_NEAR_PAD        (0x0114U) /* 车灯附近信标判定外扩距离 */
#define IPC_REMOTE_PARAM_ID_BL3_NEAR_MIN        (0x0115U) /* 车灯附近信标最小面积 */
#define IPC_REMOTE_PARAM_ID_BL3_NEAR_GRAY       (0x0116U) /* 车灯附近孤立信标最小峰值灰度 */
#define IPC_REMOTE_PARAM_ID_BL3_NEAR_BG         (0x0117U) /* 车灯附近孤立信标最大背景灰度 */
#define IPC_REMOTE_PARAM_ID_BL3_MATCH_DIST      (0x0118U) /* 信标跟踪匹配距离 */
#define IPC_REMOTE_PARAM_ID_BL3_GATE_DIST       (0x0119U) /* 已确认目标匹配门距离 */
#define IPC_REMOTE_PARAM_ID_BL3_NEW_DIST        (0x011AU) /* 新目标重建距离 */
#define IPC_REMOTE_PARAM_ID_BL3_CONFIRM         (0x011BU) /* 目标初始化确认帧数 */
#define IPC_REMOTE_PARAM_ID_BL3_MISSES          (0x011CU) /* 信标最大连续丢失帧数 */
#define IPC_REMOTE_PARAM_ID_BL3_POS_ALPHA       (0x011DU) /* 位置滤波当前测量权重 */
#define IPC_REMOTE_PARAM_ID_BL3_VEL_ALPHA       (0x011EU) /* 速度滤波当前测量权重 */
#define IPC_REMOTE_PARAM_ID_BL3_STREAM_MODE     (0x0120U) /* 图传内容模式 */
#define IPC_REMOTE_PARAM_ID_BL3_LAMP_WIDTH      (0x0121U)
#define IPC_REMOTE_PARAM_ID_BL3_NARROW_WIDTH    (0x0122U)
#define IPC_REMOTE_PARAM_ID_BL3_NARROW_ELONG    (0x0123U)
#define IPC_REMOTE_PARAM_ID_BL3_UPPER_AREA      (0x0124U)
#define IPC_REMOTE_PARAM_ID_BL3_UPPER_LEN       (0x0125U)
#define IPC_REMOTE_PARAM_ID_BL3_UPPER_WIDTH     (0x0126U)
#define IPC_REMOTE_PARAM_ID_BL3_COMPACT_Y       (0x0127U)
#define IPC_REMOTE_PARAM_ID_BL3_COMPACT_AREA    (0x0128U)
#define IPC_REMOTE_PARAM_ID_BL3_COMPACT_LEN     (0x0129U)
#define IPC_REMOTE_PARAM_ID_BL3_COMPACT_WIDTH   (0x012AU)
#define IPC_REMOTE_PARAM_ID_BL3_COMPACT_ELONG   (0x012BU)
#define IPC_REMOTE_PARAM_ID_BL3_VGLARE_ELONG    (0x012FU)
#define IPC_REMOTE_PARAM_ID_BL3_VGLARE_GRAY     (0x0130U)
#define IPC_REMOTE_PARAM_ID_BL3_LINEAR_ELONG    (0x0131U)
#define IPC_REMOTE_PARAM_ID_BL3_WEAK_C_THR      (0x0132U)
#define IPC_REMOTE_PARAM_ID_BL3_WEAK_C_MIN      (0x0133U)
#define IPC_REMOTE_PARAM_ID_BL3_WEAK_C_MAX      (0x0134U)
#define IPC_REMOTE_PARAM_ID_BL3_WEAK_C_GRAY     (0x0135U)
#define IPC_REMOTE_PARAM_ID_BL3_WEAK_C_BG       (0x0136U)
#define IPC_REMOTE_PARAM_ID_BL3_SHAPE_MIN       (0x0137U)
#define IPC_REMOTE_PARAM_ID_BL3_SHAPE_RATIO     (0x0138U)
#define IPC_REMOTE_PARAM_ID_BL3_SHAPE_FILL      (0x0139U)
#define IPC_REMOTE_PARAM_ID_BL3_SHAPE_S_FILL    (0x013AU)
#define IPC_REMOTE_PARAM_ID_BL3_TOP_V_ELONG     (0x013FU)
#define IPC_REMOTE_PARAM_ID_BL3_SAT_T_GRAY      (0x0140U)
#define IPC_REMOTE_PARAM_ID_BL3_BEACON_THR      (0x0141U)
#define IPC_REMOTE_PARAM_ID_BL3_EXP_TIME        (0x0142U)

/* 两颗2BL3位置面积表动态参数及持久化命令 ID。 */
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

typedef struct
{
    uint32 sequence;
    float roll_deg;
    float pitch_deg;
    uint32 reserved;
} ipc_attitude_data_t;

typedef char ipc_attitude_data_size_must_be_16[
    (sizeof(ipc_attitude_data_t) == 16U) ? 1 : -1];

extern volatile uint32 g_image_data_seq;
extern volatile ipc_camera_spi_log_t g_ipc_camera_spi_log;
extern volatile ipc_remote_param_mailbox_t g_ipc_remote_param_request;
extern volatile ipc_remote_param_mailbox_t g_ipc_remote_param_response;
extern volatile ipc_attitude_data_t g_ipc_attitude_data;

void ipc_image_callback(uint32 ipc_data);
void ipc_image_publish(void);
void ipc_image_poll(void);
void ipc_attitude_publish(float roll_deg, float pitch_deg);
void ipc_attitude_get(ipc_attitude_data_t *out);

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
