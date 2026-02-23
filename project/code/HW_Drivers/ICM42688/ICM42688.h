/*
 * icm42688.h
 *
 *  Created on: 2024ĺš?ć?ć?
 *      Author: ljk
 *      mail:983688746@qq.com
 */
/*MIT License
*
*Copyright (c) 2024 ljk
*
*Permission is hereby granted, free of charge, to any person obtaining a copy
*of this software and associated documentation files (the "Software"), to deal
*in the Software without restriction, including without limitation the rights
*to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*copies of the Software, and to permit persons to whom the Software is
*furnished to do so, subject to the following conditions:
*
*The above copyright notice and this permission notice shall be included in all
*copies or substantial portions of the Software.
*
*THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*SOFTWARE.
*/
#ifndef CODE_ICM42688_H_
#define CODE_ICM42688_H_
#include "zf_common_typedef.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

/* ========== éŁć§ä¸ç¨éç˝Žďź?kHz@BetaflightéŁć źďź?========== */
#define ICM42688_SAMPLE_RATE_HZ          2000  /* éć ˇç?kHz */

#define   WHO_AM_I  0xF500
#define ICM42688_SPI          (SPI_2)                                                           // SPIć¨ĄĺĺˇĺŽäš?
#define ICM42688_MOSI_Pin     (SPI2_MOSI_P15_1)                                                // MOSIĺźčĺŽäš
#define ICM42688_MISO_Pin     (SPI2_MISO_P15_0)                                                // MISOĺźčĺŽäš
#define ICM42688_CS_Pin       (P15_3)                                                           // CSçéĺźčĺŽäš?
#define ICM42688_SCK_Pin      (SPI2_CLK_P15_2)                                                 // SCLKĺźčĺŽäš
#define ICM42688_SPEED        (10 * 1000 * 1000)                                                // SPIćśééçďźć­¤ĺ¤ä¸ş10M

#define ICM42688_ID           0X47
#define READ_ACC_X_HIGH       0X9F

#define SENSITIVITY_ICM42688_GYRO_15_625dps  2097.2
#define SENSITIVITY_ICM42688_GYRO_31_25dps   1048.6
#define SENSITIVITY_ICM42688_GYRO_62_5dps    524.3
#define SENSITIVITY_ICM42688_GYRO_125dps     262.0
#define SENSITIVITY_ICM42688_GYRO_250dps     131.0
#define SENSITIVITY_ICM42688_GYRO_500dps     65.5
#define SENSITIVITY_ICM42688_GYRO_1000dps    32.8
#define SENSITIVITY_ICM42688_GYRO_2000dps    16.4

#define SENSITIVITY_ICM42688_ACC_16G  2048
#define SENSITIVITY_ICM42688_ACC_8G   4096
#define SENSITIVITY_ICM42688_ACC_4G   8192
#define SENSITIVITY_ICM42688_ACC_2G   16384
typedef struct ICM42688_RAW_DATA{

    int16 acc_x_lsb;
    int16 acc_y_lsb;
    int16 acc_z_lsb;
    int16 gyro_x_lsb;
    int16 gyro_y_lsb;
    int16 gyro_z_lsb;
    int16 temp_lsb;// ć¸ŠĺşŚćä¸čŻťĺ
} ICM42688_RAW_DATA;// ä¸äź ćĺ¨éäżĄčˇĺçLSBĺĺ§ć°ćŽďźčżéčŚč˝Źĺä¸şćçŠçćäšçć°ćŽ

typedef struct ICM42688_real_data {

    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;// ć¸ŠĺşŚćä¸čŻťĺ

} ICM42688_real_data;// ćçŠçćäšççĺŽć°ćŽ

typedef enum        // ćä¸žéčşäťŞéć ˇç?
{
    GYRO_ODR_12_5HZ,
    GYRO_ODR_25HZ,
    GYRO_ODR_50HZ,
    GYRO_ODR_100HZ,
    GYRO_ODR_200HZ,
    GYRO_ODR_500HZ,
    GYRO_ODR_1000HZ,
    GYRO_ODR_2000HZ,
    GYRO_ODR_4000HZ,
    GYRO_ODR_8000HZ,
    GYRO_ODR_16000HZ,
    GYRO_ODR_32000HZ,
}GYRO_ODR;          //ODR -->Output Data Rate

typedef enum        // ćä¸žĺ éĺşŚčŽĄéć ˇç
{
    ACC_ODR_12_5HZ,
    ACC_ODR_25HZ,
    ACC_ODR_50HZ,
    ACC_ODR_100HZ,
    ACC_ODR_200HZ,
    ACC_ODR_500HZ,
    ACC_ODR_1000HZ,
    ACC_ODR_2000HZ,
    ACC_ODR_4000HZ,
    ACC_ODR_8000HZ,
    ACC_ODR_16000HZ,
    ACC_ODR_32000HZ,
}ACC_ODR;

typedef enum   // éčşäťŞéç¨ćä¸ž  DPS --> Degree Per Second
{
    GYRO_2000DPS,
    GYRO_1000DPS,
    GYRO_500DPS,
    GYRO_250DPS,
    GYRO_125DPS,
    GYRO_62_5DPS,
    GYRO_31_25DPS,
    GYRO_15_625DPS,
}GYRO_FSR;      //FSR --> Full Scan Range

typedef enum   // ĺ éĺşŚčŽĄéç¨ćä¸?
{
    ACC_16G,
    ACC_8G,
    ACC_4G,
    ACC_2G,
}ACC_FSR;

typedef enum   // ĺé¨ćť¤ćł˘ĺ¨éść°ćä¸?čĽä¸čŽžç˝Ž éťčŽ¤ä˝żç¨äşéś
{
    _1st,      // ä¸é?
    _2st,      // äşéś
    _3st,      // ä¸éś
}Filter_Order;

typedef enum   // ĺ¸ŚĺŽ˝çłťć°ćä¸ž  ĺˇä˝ä˝ç¨čŻˇéčŻťćĺ?1 81éĄ?ćçťç3dBĺ¸ŚĺŽ˝ç?ODR(éć ˇç? ćť¤ćł˘ĺ¨éść°čŽžç˝?ĺ¸ŚĺŽ˝çłťć° ĺąĺĺłĺŽ čŻˇĺčćĺ?1éĄľĺžĺçčĄ¨ć ź
{
    Bandwidth_Factor_2,
    Bandwidth_Factor_4,
    Bandwidth_Factor_5,
    Bandwidth_Factor_8,
    Bandwidth_Factor_10,
    Bandwidth_Factor_16,
    Bandwidth_Factor_20,
    Bandwidth_Factor_40,
    Low_latency_1,
    Low_Latency_2,
}Bandwidth_Factor;

typedef enum   // ćŻĺŚĺŻç¨čŻçĺç˝Žçĺťéśĺĺč˝ čĽĺŻç?ĺĺ§ĺčżç¨éäżćéć­˘ ĺŽćĺéśĺäźčŁĺĽç¸ĺłĺŻĺ­ĺ?ICM42688äźčŞĺ¨ĺĺ?ĺč˝ććśĺźç¨
{
    Bias_On_Chip_On,
    Bias_On_Chip_Off,
}Bias_On_Chip;

typedef struct ICM42688_CONFIG_STRUCT
{
    GYRO_FSR GYRO_FSR;
    GYRO_ODR GYRO_ODR;
    ACC_FSR ACC_FSR;
    ACC_ODR ACC_ODR;
    Filter_Order Gyro_Filter_Order;
    Bandwidth_Factor Gyro_Bandwidth_Factor;
    Filter_Order Acc_Filter_Order;
    Bandwidth_Factor Acc_Bandwidth_Factor;
    Bias_On_Chip Bias_On_Chip;
}ICM42688_CONFIG_STRUCT;// ICM42688ĺĺ§ĺçťćä˝
extern ICM42688_CONFIG_STRUCT ICM42688_CONFIG;
extern float Gyro_Sensitivity;
extern float Acc_Sensitivity;
extern ICM42688_RAW_DATA ICM42688_RAW;
extern int32 ICM42688_Bias_gyro_x;
extern int32 ICM42688_Bias_gyro_y;
extern int32 ICM42688_Bias_gyro_z;
extern uint8_t ICM42688_Bias_Init_Flag;
void ICM42688_Init(ICM42688_CONFIG_STRUCT*ICM42688_CONFIG);
void ICM42688_Bias_Init(uint32 times);
void ICM42688_Get_Data(void);
extern ICM42688_real_data ICM42688;
#endif /* CODE_ICM42688_H_ */

