#ifndef IPC_IMAGE_DATA_H_
#define IPC_IMAGE_DATA_H_

#include "zf_common_headfile.h"
#include "Image/image_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IPC_CAMERA_SPI_BOARD_COUNT (2U)

#define IPC_ATTITUDE_FLAG_HEIGHT_VALID          (0x00000001UL)

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
#define IPC_REMOTE_PARAM_ID_C1_LAMP_EDGE_MISSES (0x0316U) /* 边缘车灯最大丢失帧数 */
#define IPC_REMOTE_PARAM_ID_C1_LAMP_CENTER_MISSES (0x0317U) /* 中部车灯最大丢失帧数 */
#define IPC_REMOTE_PARAM_ID_C1_TEMP_CORE_PAD    (0x0318U) /* 时序核心外扩距离 */
#define IPC_REMOTE_PARAM_ID_C1_TEMP_TAKEOVER_PAD (0x0319U) /* 时序接管外扩距离 */
#define IPC_REMOTE_PARAM_ID_C1_TEMP_MIN_BRIGHT  (0x031AU) /* 时序车灯最小亮区面积 */
#define IPC_REMOTE_PARAM_ID_C1_OUTPUT_ENV_PAD   (0x031BU) /* 车灯输出包络外扩距离 */
#define IPC_REMOTE_PARAM_ID_C1_GROUP_ANGLE_MAX  (0x031CU) /* 车灯分组最大角度差 */
#define IPC_REMOTE_PARAM_ID_C1_GROUP_MINOR_PAD  (0x031DU) /* 车灯分组短轴外扩距离 */
#define IPC_REMOTE_PARAM_ID_C1_GROUP_MAJOR_GAP  (0x031EU) /* 车灯分组长轴最大间隙 */
#define IPC_REMOTE_PARAM_ID_C1_RECOVER_MIN_CORE (0x031FU) /* 车灯恢复核心最小面积 */
#define IPC_REMOTE_PARAM_ID_C1_RECOVER_MAX_CORE (0x0320U) /* 车灯恢复核心最大面积 */
#define IPC_REMOTE_PARAM_ID_C1_RECOVER_CONNECT_THR (0x0321U) /* 车灯连接恢复阈值 */
#define IPC_REMOTE_PARAM_ID_C1_RECOVER_TRACK_THR (0x0322U) /* 车灯跟踪恢复阈值 */
#define IPC_REMOTE_PARAM_ID_C1_RECOVER_BRIDGE_THR (0x0323U) /* 车灯桥接恢复阈值 */
#define IPC_REMOTE_PARAM_ID_C1_RECOVER_SUPPORT_PAD (0x0324U) /* 车灯恢复支持区外扩距离 */
#define IPC_REMOTE_PARAM_ID_C1_SUPPORT_PAD      (0x0325U) /* 车灯支持区外扩距离 */
#define IPC_REMOTE_PARAM_ID_C1_GRAY_SUPPORT_THR (0x0326U) /* 车灯支持区灰度阈值 */
#define IPC_REMOTE_PARAM_ID_C1_COMPACT_EDGE_MARGIN (0x0327U) /* 紧凑车灯边缘余量 */
#define IPC_REMOTE_PARAM_ID_C1_COMPACT_MAX_MAJOR (0x0328U) /* 边缘紧凑车灯最大长轴 */
#define IPC_REMOTE_PARAM_ID_C1_BEACON_COAST     (0x032AU) /* 信标惯性跟踪帧数 */
#define IPC_REMOTE_PARAM_ID_C1_BEACON_CORNER_MARGIN (0x032BU) /* 信标角落判定边距 */
#define IPC_REMOTE_PARAM_ID_C1_NEAR_BEACON_CONFIRM (0x032CU) /* 近车灯信标确认帧数 */
#define IPC_REMOTE_PARAM_ID_C1_NEAR_BEACON_MISSES (0x032DU) /* 近车灯信标最大丢失帧数 */
#define IPC_REMOTE_PARAM_ID_C1_EDGE_BEACON_CONFIRM (0x032EU) /* 边缘信标确认帧数 */
#define IPC_REMOTE_PARAM_ID_C1_EDGE_BEACON_RADIUS (0x032FU) /* 边缘信标恢复搜索半径 */
#define IPC_REMOTE_PARAM_ID_C1_EDGE_BEACON_PEAK (0x0330U) /* 边缘信标恢复最小峰值 */

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
#define IPC_REMOTE_PARAM_ID_BL3_TEMP_MIN_BRIGHT (0x0143U) /* 时序车灯最小亮区面积 */
#define IPC_REMOTE_PARAM_ID_BL3_FRAG_MIN_AREA   (0x0144U) /* 跟踪车灯碎片最小面积 */
#define IPC_REMOTE_PARAM_ID_BL3_FRAG_MIN_LEN    (0x0145U) /* 跟踪车灯碎片最小长度 */
#define IPC_REMOTE_PARAM_ID_BL3_FRAG_MIN_WIDTH  (0x0146U) /* 跟踪车灯碎片最小宽度 */
#define IPC_REMOTE_PARAM_ID_BL3_FRAG_PAD        (0x0147U) /* 跟踪车灯碎片外扩距离 */
#define IPC_REMOTE_PARAM_ID_BL3_FRAG_VALID_LIMIT (0x0148U) /* 每帧碎片验证数量上限 */
#define IPC_REMOTE_PARAM_ID_BL3_TAKEOVER_CONFIRM (0x0149U) /* 车灯轨迹接管确认帧数 */
#define IPC_REMOTE_PARAM_ID_BL3_PENDING_ANGLE_DELTA (0x014AU) /* 待确认车灯最大角度差 */
#define IPC_REMOTE_PARAM_ID_BL3_TRACK_ANGLE_DELTA (0x014BU) /* 跟踪车灯最大角度差 */
#define IPC_REMOTE_PARAM_ID_BL3_ANGLE_MIN_ELONG (0x014CU) /* 采用实测角度的最小长宽比 */
#define IPC_REMOTE_PARAM_ID_BL3_ANGLE_MIN_LEN   (0x014DU) /* 采用实测角度的最小长度 */
#define IPC_REMOTE_PARAM_ID_BL3_ANGLE_MIN_LEN_RATIO (0x014EU) /* 实测角度最小长度比例 */
#define IPC_REMOTE_PARAM_ID_BL3_ANGLE_MAX_LEN_RATIO (0x014FU) /* 实测角度最大长度比例 */
#define IPC_REMOTE_PARAM_ID_BL3_ANGLE_MIN_WIDTH_RATIO (0x0150U) /* 实测角度最小宽度比例 */
#define IPC_REMOTE_PARAM_ID_BL3_ANGLE_MAX_WIDTH_RATIO (0x0151U) /* 实测角度最大宽度比例 */
#define IPC_REMOTE_PARAM_ID_BL3_B0_SWITCH_CONFIRM (0x0152U) /* B0目标切换确认帧数 */
#define IPC_REMOTE_PARAM_ID_BL3_B0_REACQUIRE_CONFIRM (0x0153U) /* B0目标重捕获确认帧数 */
#define IPC_REMOTE_PARAM_ID_BL3_B0_REACQUIRE_MIN_AREA (0x0154U) /* B0目标重捕获最小面积 */
#define IPC_REMOTE_PARAM_ID_BL3_B0_REACQUIRE_MIN_RATIO (0x0155U) /* B0重捕获最小面积比例 */
#define IPC_REMOTE_PARAM_ID_BL3_B0_REACQUIRE_MAX_RATIO (0x0156U) /* B0重捕获最大面积比例 */
#define IPC_REMOTE_PARAM_ID_BL3_B0_DIRECT_CLOSE_DIST (0x0157U) /* B0直接匹配近距离 */
#define IPC_REMOTE_PARAM_ID_BL3_B0_DIRECT_MIN_RATIO (0x0158U) /* B0直接匹配最小面积比例 */
#define IPC_REMOTE_PARAM_ID_BL3_B0_DIRECT_MAX_RATIO (0x0159U) /* B0直接匹配最大面积比例 */
#define IPC_REMOTE_PARAM_ID_BL3_CENTER_LOWER_MIN_AREA (0x015AU) /* 中下区域信标最小面积 */
#define IPC_REMOTE_PARAM_ID_BL3_CENTER_LOWER_HARD_MIN (0x015BU) /* 中下区域信标绝对最小面积 */
#define IPC_REMOTE_PARAM_ID_BL3_COMPACT_REACQUIRE_HITS (0x015CU) /* 紧凑信标重捕获命中数 */
#define IPC_REMOTE_PARAM_ID_BL3_TRACK_BOOTSTRAP_GRAY (0x015DU) /* 信标跟踪启动最小灰度 */
#define IPC_REMOTE_PARAM_ID_BL3_TRACK_MIN_AREA_RATIO (0x015FU) /* 信标重捕获最小面积比例 */

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
    float height_mm;
    uint32 flags;
} ipc_attitude_data_t;

typedef char ipc_attitude_data_size_must_be_20[
    (sizeof(ipc_attitude_data_t) == 20U) ? 1 : -1];

extern volatile uint32 g_image_data_seq;
extern volatile uint32 g_image_camera_seq[IMAGE_CAMERA_COUNT];                  /* CM7_1已发布的三路真实结果序号。 */
extern volatile uint32 g_image_data_fresh_mask;                                 /* 最近一次发布包含的真实新结果掩码。 */
extern volatile uint32 g_image_data_guard;                                      /* 跨核图像快照奇偶一致性保护序号。 */
extern volatile uint32 g_image_data_rx_seq;                                     /* CM7_0最近接收的一致性快照序号。 */
extern volatile uint32 g_image_camera_rx_seq[IMAGE_CAMERA_COUNT];               /* CM7_0最近接收的三路真实结果序号。 */
extern volatile uint32 g_image_data_rx_fresh_mask;                              /* CM7_0最近接收快照的新结果掩码。 */
extern volatile ipc_camera_spi_log_t g_ipc_camera_spi_log;
extern volatile ipc_remote_param_mailbox_t g_ipc_remote_param_request;
extern volatile ipc_remote_param_mailbox_t g_ipc_remote_param_response;
extern volatile ipc_attitude_data_t g_ipc_attitude_data;

void ipc_image_callback(uint32 ipc_data);

/*
 * 函数功能: 初始化本核图像工作副本及跨核发布或接收状态。
 * 输入参数: 无。
 * 返回值: 无。
 */
void ipc_image_init(void);

/*
 * 函数功能: 将CM7_1本地图像结果一致性发布到共享内存并发送非阻塞通知。
 * 输入参数: fresh_mask为本次真实新算法结果对应的摄像头位掩码。
 * 返回值: 无。
 */
void ipc_image_publish(uint8 fresh_mask);

/*
 * 函数功能: CM7_0主动读取并提交一份跨核一致性图像快照。
 * 输入参数: 无。
 * 返回值: 接收到新发布快照返回1，否则返回0。
 */
uint8 ipc_image_poll(void);
void ipc_attitude_publish(float roll_deg,
                          float pitch_deg,
                          float height_mm,
                          uint8 height_valid);
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
