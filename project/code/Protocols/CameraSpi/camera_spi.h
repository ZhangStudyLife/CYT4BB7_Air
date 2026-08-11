#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include "zf_common_headfile.h"
#include "Image/image_data.h"

#define CAMERA_SPI_BOARD_COUNT       (2U)
#define CAMERA_SPI_MAX_CAR_LAMPS     (1U)
#define CAMERA_SPI_UPDATE_PERIOD_US  (5000U) /* Air图像核调用CameraSpi_Update的固定周期。 */

typedef struct
{
    uint32 transaction;
    uint32 actual_bits;
    uint16 param_id;
    uint8 op;
    uint8 type;
    uint8 status;
} camera_spi_remote_param_result_t;

extern volatile uint8 g_camera_spi_perf_window_done[CAMERA_SPI_BOARD_COUNT];          /* 两块图像板的事务统计窗口完成标志。 */
extern volatile uint32 g_camera_spi_perf_transfer_count[CAMERA_SPI_BOARD_COUNT];      /* 两块图像板已统计的完整事务数。 */
extern volatile uint32 g_camera_spi_perf_last_transfer_us[CAMERA_SPI_BOARD_COUNT];    /* 两块图像板最近事务耗时，单位微秒。 */
extern volatile uint32 g_camera_spi_perf_average_transfer_us[CAMERA_SPI_BOARD_COUNT]; /* 两块图像板平均事务耗时，单位微秒。 */
extern volatile uint32 g_camera_spi_perf_max_transfer_us[CAMERA_SPI_BOARD_COUNT];     /* 两块图像板最大事务耗时，单位微秒。 */
extern volatile uint8 g_camera_spi_result_perf_window_done[CAMERA_SPI_BOARD_COUNT];   /* 两块图像板真实结果统计窗口完成标志。 */
extern volatile uint32 g_camera_spi_result_perf_received_updates[CAMERA_SPI_BOARD_COUNT]; /* Air实际接收的唯一新结果数。 */
extern volatile uint32 g_camera_spi_result_perf_source_updates[CAMERA_SPI_BOARD_COUNT]; /* 根据结果序号推算的图像板结果数。 */
extern volatile uint32 g_camera_spi_result_perf_duplicate_packets[CAMERA_SPI_BOARD_COUNT]; /* 重复图像结果包数量。 */
extern volatile uint32 g_camera_spi_result_perf_skipped_updates[CAMERA_SPI_BOARD_COUNT]; /* Air轮询期间跨过的图像板结果数。 */
extern volatile float g_camera_spi_result_perf_received_fps[CAMERA_SPI_BOARD_COUNT];  /* Air实际接收唯一结果频率，单位赫兹。 */
extern volatile float g_camera_spi_result_perf_source_fps[CAMERA_SPI_BOARD_COUNT];    /* 图像板真实结果频率，单位赫兹。 */
extern volatile uint32 g_camera_spi_result_perf_max_gap_us[CAMERA_SPI_BOARD_COUNT];   /* 相邻唯一结果最大到达间隔，单位微秒。 */
extern volatile uint32 g_camera_spi_result_perf_over_10ms_gap_count[CAMERA_SPI_BOARD_COUNT]; /* 唯一结果到达间隔超过10毫秒的次数。 */

void CameraSpi_Init(void);

/*
 * 非阻塞推进Camera SPI硬件传输。
 * 无输入参数；返回1表示仍有传输待完成，返回0表示当前轮次已完成或空闲。
 */
uint8 CameraSpi_Service(void);

void CameraSpi_Update(void);

/*
 * 函数功能: 将两块图像板的最新结果复制到三摄数组，并消费本轮结果变化标志。
 * 输入参数: camera为三摄结果数组；fresh_mask输出真实新算法结果对应的摄像头位掩码。
 * 返回值: 本轮结果内容发生变化的摄像头位掩码，包含新结果和超时清空。
 */
uint8 CameraSpi_GetSnapshot(struct image_data camera[IMAGE_CAMERA_COUNT],
                            uint8 *fresh_mask);

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
