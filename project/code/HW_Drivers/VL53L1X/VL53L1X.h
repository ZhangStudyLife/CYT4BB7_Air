/*********************************************************************************************************************
 * VL53L1X
 * - VL53L1X2: SDA=P08_2, SCL=P08_1
 * - VL53L1X3: SDA=P07_7, SCL=P07_6
 ********************************************************************************************************************/

#ifndef _VL53L1X_DRIVER_H_
#define _VL53L1X_DRIVER_H_

#include "zf_common_headfile.h"

/*********************************************************************************************************************
 * 宏定�? ********************************************************************************************************************/
#define VL53L1X_VALID_RANGE_MAX (1300.0f)
#define VL53L1X_INVALID_DISTANCE_MM (8192U)

#define VL53L1X2_SCL_PIN    P08_1
#define VL53L1X2_SDA_PIN    P08_2
#define VL53L1X3_SCL_PIN    P07_6
#define VL53L1X3_SDA_PIN    P07_7

/*********************************************************************************************************************
 * 函数声明
 ********************************************************************************************************************/
typedef struct
{
    uint16 VL53L1X2_distance_mm;
    uint16 VL53L1X3_distance_mm;
    uint8  VL53L1X2_range_status;
    uint8  VL53L1X3_range_status;
} VL53L1X_data_struct;

extern VL53L1X_data_struct VL53L1X_data;

uint8 VL53L1X_init_all(void);
uint8 VL53L1X_read_data(VL53L1X_data_struct *data);

#endif // _VL53L1X_DRIVER_H_
