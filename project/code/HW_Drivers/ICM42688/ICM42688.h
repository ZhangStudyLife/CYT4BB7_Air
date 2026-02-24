/*
 * ICM42688.h
 *
 *  Created on: 2024-08-02
 *      Author: ljk
 *      Mail: 983688746@qq.com
 */
/* MIT License
 *
 * Copyright (c) 2024 ljk
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef CODE_ICM42688_H_
#define CODE_ICM42688_H_

#include "zf_common_typedef.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

/* 飞控默认配置：2kHz 输出速率（Betaflight 风格） */
#define ICM42688_SAMPLE_RATE_HZ          2000

/* 常用命令与设备信息 */
#define WHO_AM_I                         0xF500
#define ICM42688_ID                      0X47
#define READ_ACC_X_HIGH                  0X9F

/* SPI 硬件配置 */
#define ICM42688_SPI                     (SPI_2)
#define ICM42688_MOSI_Pin                (SPI2_MOSI_P15_1)
#define ICM42688_MISO_Pin                (SPI2_MISO_P15_0)
#define ICM42688_CS_Pin                  (P15_3)
#define ICM42688_SCK_Pin                 (SPI2_CLK_P15_2)
#define ICM42688_SPEED                   (10 * 1000 * 1000)

/* 陀螺仪灵敏度（LSB / dps） */
#define SENSITIVITY_ICM42688_GYRO_15_625dps  2097.2
#define SENSITIVITY_ICM42688_GYRO_31_25dps   1048.6
#define SENSITIVITY_ICM42688_GYRO_62_5dps    524.3
#define SENSITIVITY_ICM42688_GYRO_125dps     262.0
#define SENSITIVITY_ICM42688_GYRO_250dps     131.0
#define SENSITIVITY_ICM42688_GYRO_500dps     65.5
#define SENSITIVITY_ICM42688_GYRO_1000dps    32.8
#define SENSITIVITY_ICM42688_GYRO_2000dps    16.4

/* 加速度计灵敏度（LSB / g） */
#define SENSITIVITY_ICM42688_ACC_16G         2048
#define SENSITIVITY_ICM42688_ACC_8G          4096
#define SENSITIVITY_ICM42688_ACC_4G          8192
#define SENSITIVITY_ICM42688_ACC_2G          16384

#define ICM42688_SIGN_GX                      (1.0f)
#define ICM42688_SIGN_GY                     (-1.0f)
#define ICM42688_SIGN_GZ                     (-1.0f)
#define ICM42688_SIGN_AX                      (1.0f)
#define ICM42688_SIGN_AY                     (-1.0f)
#define ICM42688_SIGN_AZ                     (-1.0f)

/* 传感器原始数据（寄存器直接读出的 LSB 值） */
typedef struct ICM42688_RAW_DATA {
    int16 acc_x_lsb;
    int16 acc_y_lsb;
    int16 acc_z_lsb;
    int16 gyro_x_lsb;
    int16 gyro_y_lsb;
    int16 gyro_z_lsb;
    int16 temp_lsb;     /* 当前驱动未启用温度换算 */
} ICM42688_RAW_DATA;

/* 物理量数据（陀螺仪：dps，加速度：g） */
typedef struct ICM42688_real_data {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;         /* 当前驱动未启用温度换算 */
} ICM42688_real_data;

/* ODR：Output Data Rate（输出数据速率） */
typedef enum {
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
} GYRO_ODR;

typedef enum {
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
} ACC_ODR;

/* FSR：Full Scale Range（满量程） */
typedef enum {
    GYRO_2000DPS,
    GYRO_1000DPS,
    GYRO_500DPS,
    GYRO_250DPS,
    GYRO_125DPS,
    GYRO_62_5DPS,
    GYRO_31_25DPS,
    GYRO_15_625DPS,
} GYRO_FSR;

typedef enum {
    ACC_16G,
    ACC_8G,
    ACC_4G,
    ACC_2G,
} ACC_FSR;

/* 数字低通滤波器阶数 */
typedef enum {
    _1st,
    _2st,
    _3st,
} Filter_Order;

/* 带宽因子（详细含义请参考数据手册滤波章节） */
typedef enum {
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
} Bandwidth_Factor;

/* 是否启用芯片内部零偏估计 */
typedef enum {
    Bias_On_Chip_On,
    Bias_On_Chip_Off,
} Bias_On_Chip;

/* ICM42688 初始化参数集合 */
typedef struct ICM42688_CONFIG_STRUCT {
    GYRO_FSR GYRO_FSR;
    GYRO_ODR GYRO_ODR;
    ACC_FSR ACC_FSR;
    ACC_ODR ACC_ODR;
    Filter_Order Gyro_Filter_Order;
    Bandwidth_Factor Gyro_Bandwidth_Factor;
    Filter_Order Acc_Filter_Order;
    Bandwidth_Factor Acc_Bandwidth_Factor;
    Bias_On_Chip Bias_On_Chip;
} ICM42688_CONFIG_STRUCT;

extern ICM42688_CONFIG_STRUCT ICM42688_CONFIG;
extern float Gyro_Sensitivity;
extern float Acc_Sensitivity;
extern ICM42688_RAW_DATA ICM42688_RAW;
extern float ICM42688_Bias_gyro_x;
extern float ICM42688_Bias_gyro_y;
extern float ICM42688_Bias_gyro_z;
extern uint8 ICM42688_Bias_Init_Flag;
extern ICM42688_real_data ICM42688;

/* 初始化芯片并按照配置写入寄存器 */
void ICM42688_Init(ICM42688_CONFIG_STRUCT *ICM42688_CONFIG);

/* 静态标定零偏，times 建议 >= 500 */
void ICM42688_Bias_Init(uint32 times);

/* 读取一次传感器数据并完成单位换算 */
void ICM42688_Get_Data(void);

#endif /* CODE_ICM42688_H_ */
