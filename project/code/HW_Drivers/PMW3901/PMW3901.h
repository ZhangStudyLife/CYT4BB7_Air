#ifndef PMW3901_H_
#define PMW3901_H_

#include "zf_common_headfile.h"

#ifndef __packed
#define __packed __PACKED
#endif

#define PMW3901_SPI              (SPI_3)
#define PMW3901_MOSI_PIN         (SPI3_MOSI_P03_1)
#define PMW3901_MISO_PIN         (SPI3_MISO_P03_0)
#define PMW3901_SCK_PIN          (SPI3_CLK_P03_2)
#define PMW3901_CS_Pin           (P03_3)
#define PMW3901_SPI_SPEED        (2 * 1000 * 1000)

// Set to 1 to verify register writes via readback.
// Some PMW3901 registers are write-only on some boards, so 0 avoids false failures.
#define PMW3901_VERIFY_WRITES    (1)

typedef __packed struct motionBurst_s
{
    // motion register bit layout
    __packed union
    {
        uint8 motion;
        __packed struct
        {
            uint8 frameFrom0    : 1;        
            uint8 runMode       : 2;
            uint8 reserved1     : 1;
            uint8 rawFrom0      : 1;
            uint8 reserved2     : 2;
            uint8 motionOccured : 1;
        };
    };

    // accumulated frame observation info
    uint8 observation;

    // pixel delta in chip frame
    int16 deltaX;
    int16 deltaY;

    // image quality and raw stats
    uint8 squal;
    uint8 rawDataSum;
    uint8 maxRawData;
    uint8 minRawData;

    // readout is big-endian in burst stream, swapped in driver
    uint16 shutter;
} pmw3901_raw_t;

extern volatile pmw3901_raw_t g_pmw3901_raw;

uint8 PMW3901_Init(void);
uint8 PMW3901_ReInit(void);
void PMW3901_Update(void);

#endif /* PMW3901_H_ */
