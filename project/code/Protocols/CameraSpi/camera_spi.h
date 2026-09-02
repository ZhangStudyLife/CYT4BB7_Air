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

/* 两块图像板都已在线时返回1。 */
uint8 CameraSpi_RemoteParamBoardsOnline(void);

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
