#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include "zf_common_headfile.h"

#define CAMERA_SPI_MAX_BEACONS    (4U)
#define CAMERA_SPI_MAX_CAR_LAMPS  (1U)
#define CAMERA_SPI_BOARD_COUNT    (2U)

typedef struct
{
    uint8 valid;
    float x;
    float y;
    float area;
} camera_spi_beacon_t;

typedef struct
{
    uint8 valid;
    float cx;
    float cy;
    float width;
    float length;
    float angle;
} camera_spi_car_lamp_t;

typedef struct
{
    uint8 online;
    uint8 version;
    uint8 beacon_count;
    uint8 car_lamp_count;
    camera_spi_beacon_t beacons[CAMERA_SPI_MAX_BEACONS];
    camera_spi_car_lamp_t car_lamps[CAMERA_SPI_MAX_CAR_LAMPS];
} camera_spi_board_snapshot_t;

void CameraSpi_Init(void);
void CameraSpi_Update(void);
void CameraSpi_GetSnapshot(camera_spi_board_snapshot_t out[CAMERA_SPI_BOARD_COUNT]);

#endif
