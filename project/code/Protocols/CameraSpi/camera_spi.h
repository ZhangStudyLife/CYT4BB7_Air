#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include "zf_common_headfile.h"
#include "Image/image_data.h"

#define CAMERA_SPI_BOARD_COUNT    (2U) /* 前后两颗 2BL3 图像板数量 */
#define CAMERA_SPI_MAX_CAR_LAMPS  (2U) /* 每颗图像板最大车灯槽位数 */

typedef struct
{
    uint32 transaction;
    uint32 actual_bits;
    uint16 param_id;
    uint8 op;
    uint8 type;
    uint8 status;
} camera_spi_remote_param_result_t;

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
 * @brief 读取以下摄为锚点的最近三摄同步组。
 * @param out 输出只读同步组快照。
 * @return 1表示三路有效且最大时差不超过10ms，否则返回0。
 */
uint8 CameraSpi_GetAlignedSet(image_sync_set_t *out);

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
