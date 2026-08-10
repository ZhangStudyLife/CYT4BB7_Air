#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include "zf_common_headfile.h"
#include "Image/image_data.h"

#define CAMERA_SPI_BOARD_COUNT    (2U) /* 前后两颗 2BL3 图像板数量 */
#define CAMERA_SPI_MAX_CAR_LAMPS  (2U) /* 兼容image_data的存储槽位；线协议仅传车灯0。 */

#define CAMERA_SPI_ROI_DIAG_VALID        (0x01U)
#define CAMERA_SPI_ROI_DIAG_HIT          (0x02U)
#define CAMERA_SPI_ROI_DIAG_FALLBACK     (0x04U)
#define CAMERA_SPI_ROI_DIAG_CONFLICT     (0x08U)
#define CAMERA_SPI_ROI_DIAG_ACTUAL       (0x10U)
#define CAMERA_SPI_ROI_DIAG_FORCED_FULL  (0x20U)

extern volatile uint8 g_camera_spi_roi_diag[IMAGE_CAMERA_COUNT];

typedef struct
{
    uint32 transaction;
    uint32 actual_bits;
    uint16 param_id;
    uint8 op;
    uint8 type;
    uint8 status;
} camera_spi_remote_param_result_t;

/* Air核心1本地Live Watch诊断，不进入IPC或Camera SPI协议。 */
typedef struct
{
    uint32 ready_raw_high_ticks;
    uint32 ready_eligible_ticks;
    uint32 ready_low_ticks;
    uint32 ready_rearm_block_ticks;
    uint32 control_attempt_count;
    uint32 control_start_count;
    uint32 control_success_count;
    uint32 control_error_count;
    uint32 ready_race_count;
    uint32 timeout_count;
    uint32 crc_error_count;
    uint32 hw_error_count;
    uint32 image_response_count;
    uint32 invalid_image_count;
    uint32 new_frame_count;
    uint32 duplicate_frame_count;
    uint32 out_of_order_frame_count;
    uint32 source_frame_count;
    uint32 skipped_frame_count;
    uint32 last_frame_sequence;
    uint32 max_frame_delta;
    uint8 last_error;
    float average_transfer_us;
    float max_transfer_us;
    float new_frame_fps;
    float source_fps;
} camera_spi_debug_board_t;

extern volatile uint8 g_camera_spi_debug_window_done;
extern volatile uint32 g_camera_spi_debug_scheduler_ticks;
extern volatile uint32 g_camera_spi_debug_cycles_started;
extern volatile uint32 g_camera_spi_debug_cycles_completed;
extern volatile uint32 g_camera_spi_debug_busy_ticks;
extern volatile uint32 g_camera_spi_debug_elapsed_ms;
extern volatile float g_camera_spi_debug_scheduler_hz;
extern volatile float g_camera_spi_debug_average_cycle_us;
extern volatile float g_camera_spi_debug_max_cycle_us;
extern volatile camera_spi_debug_board_t
    g_camera_spi_debug_board[CAMERA_SPI_BOARD_COUNT];

/**
 * @brief 初始化核心1 Camera SPI主机、双板状态和三摄同步缓存。
 * @return 无。
 */
void CameraSpi_Init(void);

/*
 * 非阻塞推进Camera SPI硬件传输。
 * 无输入参数；返回1表示仍有传输待完成，返回0表示当前轮次已完成或空闲。
 */
uint8 CameraSpi_Service(void);

/**
 * @brief 启动一次100Hz双板控制采集及待发送图像透传轮次。
 * @return 无，后续由CameraSpi_Service非阻塞推进。
 */
void CameraSpi_Update(void);

/**
 * @brief 将前后图像板的最新结果和元数据复制到核心1共享快照。
 * @param camera 三摄图像结果数组；本函数更新Front和Back槽。
 * @return 无。
 */
void CameraSpi_GetSnapshot(struct image_data camera[IMAGE_CAMERA_COUNT]);

/**
 * @brief 提交核心1刚完成识别的下摄帧，供同步缓存和两板透传使用。
 * @param data 下摄识别结果的只读指针。
 * @param meta 下摄来源帧元数据的只读指针。
 * @return 无。
 */
void CameraSpi_SubmitLocalFrame(const struct image_data *data,
                                const image_frame_meta_t *meta);

/**
 * @brief 读取三摄最新帧；严格同步使用50ms，公共轨迹保留500ms。
 * @param out 输出只读最新帧组；max_skew_ms仅用于诊断采集时差。
 * @return 1表示三路均处于严格新鲜且时间有效状态，否则返回0。
 */
uint8 CameraSpi_GetLatestSet(image_sync_set_t *out);

/* 启动一笔同时发往两颗2BL3的参数事务；持久化命令SET跳过预读，返回1表示已启动。 */
uint8 CameraSpi_RemoteParamStart(uint8 op,
                                 uint8 type,
                                 uint16 param_id,
                                 uint32 transaction,
                                 uint32 value_bits,
                                 uint32 previous_bits);

/* 读取并消费已完成的两板参数事务结果，返回1表示取得结果。 */
uint8 CameraSpi_RemoteParamTakeResult(camera_spi_remote_param_result_t *result);

/* 取消指定事务；普通SET恢复两板各自旧值，持久化命令直接超时且不回滚。 */
uint8 CameraSpi_RemoteParamCancel(uint32 transaction);

#endif
