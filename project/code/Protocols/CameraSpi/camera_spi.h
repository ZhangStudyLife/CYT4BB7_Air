#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include "zf_common_headfile.h"
#include "Image/image_data.h"

#define CAMERA_SPI_BOARD_COUNT    (2U)
#define CAMERA_SPI_MAX_CAR_LAMPS  (1U)

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
void CameraSpi_Update(void);
void CameraSpi_GetSnapshot(struct image_data camera[IMAGE_CAMERA_COUNT]);

/* 启动一笔同时发往两颗2BL3的参数事务，返回1表示已启动。 */
uint8 CameraSpi_RemoteParamStart(uint8 op,
                                 uint8 type,
                                 uint16 param_id,
                                 uint32 transaction,
                                 uint32 value_bits,
                                 uint32 previous_bits);

/* 读取并消费已完成的两板参数事务结果，返回1表示取得结果。 */
uint8 CameraSpi_RemoteParamTakeResult(camera_spi_remote_param_result_t *result);

/* 取消指定事务；SET已下发或成功结果刚被消费时，均先恢复两板各自旧值。 */
uint8 CameraSpi_RemoteParamCancel(uint32 transaction);

#endif
