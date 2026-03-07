/*
 * ICM42688.c
 *
 *  Created on: 2024-08-02
 *      Author: ljk
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

#include "ICM42688.h"

/*
 * ICM42688 默认配置（1kHz）
 * 1) 陀螺仪量程 2000dps，适合飞控快速机动场景
 * 2) 加速度量程 16g，覆盖常见动态范围
 * 3) 二阶滤波 + 合适带宽，兼顾噪声与延迟
 * 4) 上电后进入 LN（Low Noise）模式
 */
ICM42688_CONFIG_STRUCT ICM42688_CONFIG = {
    GYRO_2000DPS,
    GYRO_ODR_1000HZ,
    ACC_16G,
    ACC_ODR_1000HZ,
    _3st,
    Bandwidth_Factor_2,
    _3st,
    Bandwidth_Factor_2,
    Bias_On_Chip_Off
};

float Gyro_Sensitivity, Acc_Sensitivity;
ICM42688_RAW_DATA ICM42688_RAW;      /* 原始 LSB 数据 */
ICM42688_real_data ICM42688;         /* 换算后的物理量 */
float ICM42688_Bias_gyro_x = 0, ICM42688_Bias_gyro_y = 0, ICM42688_Bias_gyro_z = 0;
uint8 ICM42688_Bias_Init_Flag = 0;

/* 初始化 SPI 与片选 GPIO */
static void ICM42688_SPI_HardWare_Init(void)
{
    spi_init(ICM42688_SPI, SPI_MODE0, ICM42688_SPEED,
             ICM42688_SCK_Pin, ICM42688_MOSI_Pin, ICM42688_MISO_Pin, SPI_CS_NULL);
    gpio_init(ICM42688_CS_Pin, GPO, 1, GPO_PUSH_PULL);
}

/* 发送并接收 16bit SPI 数据 */
static uint16_t SPI_Transfer_16bit(uint16_t dout)
{
    uint16_t return_data;

    gpio_low(ICM42688_CS_Pin);
    spi_transfer_16bit(ICM42688_SPI, &dout, &return_data, 1);
    gpio_high(ICM42688_CS_Pin);

    return return_data;
}

/*
 * 连续读取加速度与陀螺仪原始值
 * 数据顺序：ax ay az gx gy gz
 */
static void ICM42688_Read_Burst(ICM42688_RAW_DATA *raw)
{
    uint8_t tx_buf[13] = {0};
    uint8_t rx_buf[13] = {0};

    if (raw == NULL)
    {
        return;
    }

    tx_buf[0] = READ_ACC_X_HIGH;

    gpio_low(ICM42688_CS_Pin);
    spi_transfer_8bit(ICM42688_SPI, tx_buf, rx_buf, 13);
    gpio_high(ICM42688_CS_Pin);

    raw->acc_x_lsb = (int16)(((uint16)rx_buf[1] << 8) | rx_buf[2]);
    raw->acc_y_lsb = (int16)(((uint16)rx_buf[3] << 8) | rx_buf[4]);
    raw->acc_z_lsb = (int16)(((uint16)rx_buf[5] << 8) | rx_buf[6]);
    raw->gyro_x_lsb = (int16)(((uint16)rx_buf[7] << 8) | rx_buf[8]);
    raw->gyro_y_lsb = (int16)(((uint16)rx_buf[9] << 8) | rx_buf[10]);
    raw->gyro_z_lsb = (int16)(((uint16)rx_buf[11] << 8) | rx_buf[12]);
}

/* 读取并校验芯片 ID */
static void Find_ICM42688(void)
{
    SPI_Transfer_16bit(WHO_AM_I);
    uint8 ID = (uint8)SPI_Transfer_16bit(WHO_AM_I);

    if (ID != ICM42688_ID)
    {
        printf("ICM42688 not recognized,Read:%d\n", ID);
        while (1)
        {
        }
    }
}

/* 配置陀螺仪量程与输出速率 */
static void ICM42688_SET_GYRO(GYRO_FSR gyro_fsr, GYRO_ODR gyro_odr)
{
    uint8 WRITE_GYRO_CONFIG0 = 0x4F;
    uint8 GYRO_CONFIG0 = 0x00;
    uint8 GYRO_CONFIG_Verify = 0x00;
    uint16 READ_GYRO_CONFIG0 = 0XCF00;

    switch (gyro_fsr)
    {
        case GYRO_15_625DPS:
            GYRO_CONFIG0 |= 0xE0;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_15_625dps;
            break;
        case GYRO_31_25DPS:
            GYRO_CONFIG0 |= 0xC0;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_31_25dps;
            break;
        case GYRO_62_5DPS:
            GYRO_CONFIG0 |= 0xA0;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_62_5dps;
            break;
        case GYRO_125DPS:
            GYRO_CONFIG0 |= 0x80;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_125dps;
            break;
        case GYRO_250DPS:
            GYRO_CONFIG0 |= 0x60;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_250dps;
            break;
        case GYRO_500DPS:
            GYRO_CONFIG0 |= 0x40;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_500dps;
            break;
        case GYRO_1000DPS:
            GYRO_CONFIG0 |= 0x20;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_1000dps;
            break;
        case GYRO_2000DPS:
            GYRO_CONFIG0 |= 0x00;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_2000dps;
            break;
    }

    switch (gyro_odr)
    {
        case GYRO_ODR_12_5HZ:
            GYRO_CONFIG0 |= 0x0B;
            break;
        case GYRO_ODR_25HZ:
            GYRO_CONFIG0 |= 0x0A;
            break;
        case GYRO_ODR_50HZ:
            GYRO_CONFIG0 |= 0x09;
            break;
        case GYRO_ODR_100HZ:
            GYRO_CONFIG0 |= 0x08;
            break;
        case GYRO_ODR_200HZ:
            GYRO_CONFIG0 |= 0x07;
            break;
        case GYRO_ODR_500HZ:
            GYRO_CONFIG0 |= 0x0F;
            break;
        case GYRO_ODR_1000HZ:
            GYRO_CONFIG0 |= 0x06;
            break;
        case GYRO_ODR_2000HZ:
            GYRO_CONFIG0 |= 0x05;
            break;
        case GYRO_ODR_4000HZ:
            GYRO_CONFIG0 |= 0x04;
            break;
        case GYRO_ODR_8000HZ:
            GYRO_CONFIG0 |= 0x03;
            break;
        case GYRO_ODR_16000HZ:
            GYRO_CONFIG0 |= 0x02;
            break;
        case GYRO_ODR_32000HZ:
            GYRO_CONFIG0 |= 0x01;
            break;
    }

    SPI_Transfer_16bit(((uint16)WRITE_GYRO_CONFIG0 << 8 | GYRO_CONFIG0));
    GYRO_CONFIG_Verify = (uint8)SPI_Transfer_16bit(READ_GYRO_CONFIG0);

    if (GYRO_CONFIG_Verify != GYRO_CONFIG0)
    {
        printf("Gyroscope set failed!\n");
        while (1)
        {
        }
    }
}

/* 配置加速度计量程与输出速率 */
static void ICM42688_SET_ACC(ACC_FSR acc_fsr, ACC_ODR acc_odr)
{
    uint8 WRITE_ACCEL_CONFIG0 = 0x50;
    uint8 ACCEL_CONFIG0 = 0x00;
    uint8 ACCEL_CONFIG_Verify = 0x00;
    uint16 READ_ACCEL_CONFIG0 = 0XD000;

    switch (acc_fsr)
    {
        case ACC_2G:
            ACCEL_CONFIG0 |= 0x60;
            Acc_Sensitivity = SENSITIVITY_ICM42688_ACC_2G;
            break;
        case ACC_4G:
            ACCEL_CONFIG0 |= 0x40;
            Acc_Sensitivity = SENSITIVITY_ICM42688_ACC_4G;
            break;
        case ACC_8G:
            ACCEL_CONFIG0 |= 0x20;
            Acc_Sensitivity = SENSITIVITY_ICM42688_ACC_8G;
            break;
        case ACC_16G:
            ACCEL_CONFIG0 |= 0x00;
            Acc_Sensitivity = SENSITIVITY_ICM42688_ACC_16G;
            break;
    }

    switch (acc_odr)
    {
        case ACC_ODR_12_5HZ:
            ACCEL_CONFIG0 |= 0x0B;
            break;
        case ACC_ODR_25HZ:
            ACCEL_CONFIG0 |= 0x0A;
            break;
        case ACC_ODR_50HZ:
            ACCEL_CONFIG0 |= 0x09;
            break;
        case ACC_ODR_100HZ:
            ACCEL_CONFIG0 |= 0x08;
            break;
        case ACC_ODR_200HZ:
            ACCEL_CONFIG0 |= 0x07;
            break;
        case ACC_ODR_500HZ:
            ACCEL_CONFIG0 |= 0x0F;
            break;
        case ACC_ODR_1000HZ:
            ACCEL_CONFIG0 |= 0x06;
            break;
        case ACC_ODR_2000HZ:
            ACCEL_CONFIG0 |= 0x05;
            break;
        case ACC_ODR_4000HZ:
            ACCEL_CONFIG0 |= 0x04;
            break;
        case ACC_ODR_8000HZ:
            ACCEL_CONFIG0 |= 0x03;
            break;
        case ACC_ODR_16000HZ:
            ACCEL_CONFIG0 |= 0x02;
            break;
        case ACC_ODR_32000HZ:
            ACCEL_CONFIG0 |= 0x01;
            break;
    }

    SPI_Transfer_16bit(((uint16)WRITE_ACCEL_CONFIG0 << 8 | ACCEL_CONFIG0));
    ACCEL_CONFIG_Verify = (uint8)SPI_Transfer_16bit(READ_ACCEL_CONFIG0);

    if (ACCEL_CONFIG_Verify != ACCEL_CONFIG0)
    {
        printf("Accelerometer set failed!\n");
        while (1)
        {
        }
    }
}

/* 配置陀螺仪/加速度计数字滤波器 */
static void ICM42688_SET_FILTER(Bandwidth_Factor Gyro_Bandwidth_Factor,
                                Filter_Order Gyro_Filter_Order,
                                Bandwidth_Factor Acc_Bandwidth_Factor,
                                Filter_Order Acc_Filter_Order)
{
    uint8 WRITE_GYRO_ACCEL_CONFIG0 = 0X52;
    uint8 WRITE_GYRO_CONFIG1 = 0X51;
    uint8 WRITE_ACCEL_CONFIG1 = 0X53;
    uint8 Gyro_Filter_Ord_Config = 0x00;
    uint8 Acc_Filter_Ord_Config = 0x00;
    uint8 Filter_Config = 0x00;

    switch (Gyro_Bandwidth_Factor)
    {
        case Bandwidth_Factor_2:
            Filter_Config |= 0x00;
            break;
        case Bandwidth_Factor_4:
            Filter_Config |= 0x01;
            break;
        case Bandwidth_Factor_5:
            Filter_Config |= 0x02;
            break;
        case Bandwidth_Factor_8:
            Filter_Config |= 0x03;
            break;
        case Bandwidth_Factor_10:
            Filter_Config |= 0x04;
            break;
        case Bandwidth_Factor_16:
            Filter_Config |= 0x05;
            break;
        case Bandwidth_Factor_20:
            Filter_Config |= 0x06;
            break;
        case Bandwidth_Factor_40:
            Filter_Config |= 0x07;
            break;
        case Low_latency_1:
            Filter_Config |= 0x0E;
            break;
        case Low_Latency_2:
            Filter_Config |= 0x0F;
            break;
    }

    switch (Acc_Bandwidth_Factor)
    {
        case Bandwidth_Factor_2:
            Filter_Config |= 0x00;
            break;
        case Bandwidth_Factor_4:
            Filter_Config |= 0x10;
            break;
        case Bandwidth_Factor_5:
            Filter_Config |= 0x20;
            break;
        case Bandwidth_Factor_8:
            Filter_Config |= 0x30;
            break;
        case Bandwidth_Factor_10:
            Filter_Config |= 0x40;
            break;
        case Bandwidth_Factor_16:
            Filter_Config |= 0x50;
            break;
        case Bandwidth_Factor_20:
            Filter_Config |= 0x60;
            break;
        case Bandwidth_Factor_40:
            Filter_Config |= 0x70;
            break;
        case Low_latency_1:
            Filter_Config |= 0xE0;
            break;
        case Low_Latency_2:
            Filter_Config |= 0xF0;
            break;
    }

    SPI_Transfer_16bit(((uint16)WRITE_GYRO_ACCEL_CONFIG0 << 8 | Filter_Config));

    switch (Gyro_Filter_Order)
    {
        case _1st:
            Gyro_Filter_Ord_Config |= 0x02;
            break;
        case _2st:
            Gyro_Filter_Ord_Config |= 0x06;
            break;
        case _3st:
            Gyro_Filter_Ord_Config |= 0xA0;
            break;
    }
    SPI_Transfer_16bit(((uint16)WRITE_GYRO_CONFIG1 << 8 | Gyro_Filter_Ord_Config));

    switch (Acc_Filter_Order)
    {
        case _1st:
            Acc_Filter_Ord_Config |= 0x02;
            break;
        case _2st:
            Acc_Filter_Ord_Config |= 0x06;
            break;
        case _3st:
            Acc_Filter_Ord_Config |= 0xA0;
            break;
    }
    SPI_Transfer_16bit(((uint16)WRITE_ACCEL_CONFIG1 << 8 | Acc_Filter_Ord_Config));
}

/* 复位 ICM42688（软复位路径） */
static void Rest_ICM42688(void)
{
    uint8 WRITE_PWR_MGMT0 = 0X4E;
    SPI_Transfer_16bit(((uint16)WRITE_PWR_MGMT0 << 8 | 0X00));
}

/* 进入 Low Noise 模式（陀螺仪 + 加速度计） */
static void Set_ICM42688_LN_Mode(void)
{
    uint8 WRITE_PWR_MGMT0 = 0X4E;
    SPI_Transfer_16bit(((uint16)WRITE_PWR_MGMT0 << 8 | 0X0F));
}

/*
 * 获取一次传感器数据
 * - 陀螺仪单位：dps
 * - 加速度单位：g
 * - 若已完成零偏标定，则自动扣除陀螺仪零偏
 */
void ICM42688_Get_Data(void)
{
    float gyro_x_raw;
    float gyro_y_raw;
    float gyro_z_raw;
    float acc_x_raw;
    float acc_y_raw;
    float acc_z_raw;

    ICM42688_Read_Burst(&ICM42688_RAW);

    gyro_x_raw = ICM42688_RAW.gyro_x_lsb / Gyro_Sensitivity;
    gyro_y_raw = ICM42688_RAW.gyro_y_lsb / Gyro_Sensitivity;
    gyro_z_raw = ICM42688_RAW.gyro_z_lsb / Gyro_Sensitivity;

    acc_x_raw = ICM42688_RAW.acc_x_lsb / Acc_Sensitivity;
    acc_y_raw = ICM42688_RAW.acc_y_lsb / Acc_Sensitivity;
    acc_z_raw = ICM42688_RAW.acc_z_lsb / Acc_Sensitivity;

    ICM42688.gyro_x = ICM42688_SIGN_GX * gyro_x_raw;
    ICM42688.gyro_y = ICM42688_SIGN_GY * gyro_y_raw;
    ICM42688.gyro_z = ICM42688_SIGN_GZ * gyro_z_raw;

    ICM42688.acc_x = ICM42688_SIGN_AX * acc_x_raw;
    ICM42688.acc_y = ICM42688_SIGN_AY * acc_y_raw;
    ICM42688.acc_z = ICM42688_SIGN_AZ * acc_z_raw;

    if (ICM42688_Bias_Init_Flag == 1)
    {
        ICM42688.gyro_x -= ICM42688_Bias_gyro_x;
        ICM42688.gyro_y -= ICM42688_Bias_gyro_y;
        ICM42688.gyro_z -= ICM42688_Bias_gyro_z;
    }
}

/*
 * 陀螺仪静态零偏标定
 * - 标定期间需保持设备静止
 * - times 最小限制为 500
 */
void ICM42688_Bias_Init(uint32 times)
{
    int64_t sum_gyro_x = 0;
    int64_t sum_gyro_y = 0;
    int64_t sum_gyro_z = 0;
    uint32 i;

    if (ICM42688_Bias_Init_Flag == 1)
    {
        return;
    }

    if (times < 500U)
    {
        times = 500U;
    }

    for (i = 0U; i < times; i++)
    {
        ICM42688_Read_Burst(&ICM42688_RAW);
        sum_gyro_x += ICM42688_RAW.gyro_x_lsb;
        sum_gyro_y += ICM42688_RAW.gyro_y_lsb;
        sum_gyro_z += ICM42688_RAW.gyro_z_lsb;
        system_delay_us(ICM42688_SAMPLE_INTERVAL_US);
    }

    ICM42688_Bias_gyro_x = ICM42688_SIGN_GX * (((float)sum_gyro_x / (float)times) / Gyro_Sensitivity);
    ICM42688_Bias_gyro_y = ICM42688_SIGN_GY * (((float)sum_gyro_y / (float)times) / Gyro_Sensitivity);
    ICM42688_Bias_gyro_z = ICM42688_SIGN_GZ * (((float)sum_gyro_z / (float)times) / Gyro_Sensitivity);
    ICM42688_Bias_Init_Flag = 1;
}

void ICM42688_SetGyroBiasDps(float bx, float by, float bz, uint8 enable)
{
    ICM42688_Bias_gyro_x = bx;
    ICM42688_Bias_gyro_y = by;
    ICM42688_Bias_gyro_z = bz;
    ICM42688_Bias_Init_Flag = (enable != 0U) ? 1U : 0U;
}

void ICM42688_GetGyroBiasDps(float *bx, float *by, float *bz, uint8 *enable)
{
    if (bx != NULL)
    {
        *bx = ICM42688_Bias_gyro_x;
    }
    if (by != NULL)
    {
        *by = ICM42688_Bias_gyro_y;
    }
    if (bz != NULL)
    {
        *bz = ICM42688_Bias_gyro_z;
    }
    if (enable != NULL)
    {
        *enable = ICM42688_Bias_Init_Flag;
    }
}

/* 初始化 ICM42688 并加载配置 */
void ICM42688_Init(ICM42688_CONFIG_STRUCT *ICM42688_CONFIG)
{
    ICM42688_SPI_HardWare_Init();
    Rest_ICM42688();
    system_delay_ms(10);

    Find_ICM42688();
    ICM42688_SET_GYRO((*ICM42688_CONFIG).GYRO_FSR, (*ICM42688_CONFIG).GYRO_ODR);
    ICM42688_SET_ACC((*ICM42688_CONFIG).ACC_FSR, (*ICM42688_CONFIG).ACC_ODR);
    ICM42688_SET_FILTER((*ICM42688_CONFIG).Gyro_Bandwidth_Factor,
                        (*ICM42688_CONFIG).Gyro_Filter_Order,
                        (*ICM42688_CONFIG).Acc_Bandwidth_Factor,
                        (*ICM42688_CONFIG).Acc_Filter_Order);

    system_delay_ms(10);
    Set_ICM42688_LN_Mode();
    system_delay_ms(500);

    /* 清零软件侧零偏状态 */
    ICM42688_Bias_Init_Flag = 0;
    ICM42688_Bias_gyro_x = 0;
    ICM42688_Bias_gyro_y = 0;
    ICM42688_Bias_gyro_z = 0;
}
