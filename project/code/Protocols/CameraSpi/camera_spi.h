#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include "zf_common_headfile.h"
#include "Image/image_data.h"

#define CAMERA_SPI_BOARD_COUNT    (2U)
#define CAMERA_SPI_MAX_CAR_LAMPS  (1U)

void CameraSpi_Init(void);
void CameraSpi_Update(void);
void CameraSpi_GetSnapshot(struct image_data camera[IMAGE_CAMERA_COUNT]);

#endif
