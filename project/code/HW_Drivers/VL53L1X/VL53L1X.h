/*********************************************************************************************************************
 * VL53L1X
 * - TOF2: SDA=P08_2, SCL=P08_1
 * - TOF3: SDA=P07_7, SCL=P07_6
 ********************************************************************************************************************/

#ifndef _TOF_DRIVER_H_
#define _TOF_DRIVER_H_

#include "zf_common_headfile.h"

/*********************************************************************************************************************
 * 宏定义
 ********************************************************************************************************************/
#define TOF_VALID_RANGE_MAX (1300.0f)
#define TOF_INVALID_DISTANCE_MM (8192U)

#define TOF2_SCL_PIN    P08_1
#define TOF2_SDA_PIN    P08_2
#define TOF3_SCL_PIN    P07_6
#define TOF3_SDA_PIN    P07_7

/*********************************************************************************************************************
 * 函数声明
 ********************************************************************************************************************/
typedef struct
{
    uint16 tof2_distance_mm;
    uint16 tof3_distance_mm;
    uint8  tof2_range_status;
    uint8  tof3_range_status;
} tof_data_struct;

extern tof_data_struct tof_data;

uint8 tof_init_all(void);
uint8 tof_read_data(tof_data_struct *data);

#endif // _TOF_DRIVER_H_
