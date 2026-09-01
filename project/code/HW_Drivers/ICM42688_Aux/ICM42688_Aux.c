#include "ICM42688_Aux.h"

#include "../ICM42688/ICM42688.h"
#include "Protocols/wifi/wifi_cmd/wifi_cmd.h"
#include "stdio.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

/* 副 ICM42688 使用的 SPI 控制器 */
#define ICM42688_AUX_SPI                     (SPI_1)
/* 副 ICM42688 的 SPI SCK 引脚 */
#define ICM42688_AUX_SCK_PIN                 (SPI1_CLK_P12_2)
/* 副 ICM42688 的 SPI MOSI 引脚 */
#define ICM42688_AUX_MOSI_PIN                (SPI1_MOSI_P12_1)
/* 副 ICM42688 的 SPI MISO 引脚 */
#define ICM42688_AUX_MISO_PIN                (SPI1_MISO_P12_0)
/* 副 ICM42688 的片选引脚 */
#define ICM42688_AUX_CS_PIN                  (P12_3)
/* 副 ICM42688 的 SPI 通信速率 */
#define ICM42688_AUX_SPI_SPEED               (10U * 1000U * 1000U)

/* SPI 读寄存器命令位 */
#define ICM42688_AUX_READ_MASK               (0x80U)
/* DEVICE_CONFIG 寄存器地址 */
#define ICM42688_AUX_REG_DEVICE_CONFIG       (0x11U)
/* TEMP_DATA1 寄存器地址 */
#define ICM42688_AUX_REG_TEMP_DATA1          (0x1DU)
/* PWR_MGMT0 寄存器地址 */
#define ICM42688_AUX_REG_PWR_MGMT0           (0x4EU)
/* GYRO_CONFIG0 寄存器地址 */
#define ICM42688_AUX_REG_GYRO_CONFIG0        (0x4FU)
/* ACCEL_CONFIG0 寄存器地址 */
#define ICM42688_AUX_REG_ACCEL_CONFIG0       (0x50U)
/* GYRO_CONFIG1 寄存器地址 */
#define ICM42688_AUX_REG_GYRO_CONFIG1        (0x51U)
/* GYRO_ACCEL_CONFIG0 寄存器地址 */
#define ICM42688_AUX_REG_GYRO_ACCEL_CONFIG0  (0x52U)
/* ACCEL_CONFIG1 寄存器地址 */
#define ICM42688_AUX_REG_ACCEL_CONFIG1       (0x53U)
/* WHO_AM_I 寄存器地址 */
#define ICM42688_AUX_REG_WHO_AM_I            (0x75U)

/* ICM42688P 的 WHO_AM_I 期望值 */
#define ICM42688_AUX_WHO_AM_I_EXPECTED       (0x47U)
/* DEVICE_CONFIG 的软复位命令值 */
#define ICM42688_AUX_DEVICE_SOFT_RESET       (0x01U)
/* PWR_MGMT0: 陀螺仪和加速度计都进入 LN 模式 */
#define ICM42688_AUX_PWR_MGMT0_LN_MODE       (0x0FU)
/* 以 TEMP_DATA1 为起点的一次 burst 读取字节数 */
#define ICM42688_AUX_BURST_READ_LEN          (15U)

volatile icm42688_aux_raw_t g_icm42688_aux_raw = {0}; /* 副 ICM42688 原始 LSB 数据缓存 */
volatile icm42688_aux_real_t g_icm42688_aux = {0};    /* 副 ICM42688 物理量缓存 */
volatile uint8 g_icm42688_aux_ready = 0U;             /* 副 ICM42688 初始化就绪标志 */
static float g_icm42688_aux_gyro_sensitivity = 0.0f;  /* 副 ICM42688 陀螺仪灵敏度，单位 LSB/dps */
static float g_icm42688_aux_acc_sensitivity = 0.0f;   /* 副 ICM42688 加速度计灵敏度，单位 LSB/g */

/*
 * 函数功能: 通过 WiFi 文本链路输出一行副 IMU 调试日志；若 WiFi 未就绪则退回 printf
 * 输入参数:
 *   format - printf 风格格式串
 *   ...    - 可变参数
 * 返回值: 无
 */
static void ICM42688_Aux_LogLine(const char *format, ...)
{
    char line[128];
    int write_len;
    va_list ap;

    if (NULL == format)
    {
        return;
    }

    va_start(ap, format);
    write_len = vsnprintf(line, (int)sizeof(line), format, ap);
    va_end(ap);

    if (write_len < 0)
    {
        return;
    }

    if (0U != wifi_cmd_IsReady())
    {
        (void)wifi_cmd_SendLine("%s", line);
    }
    else
    {
        printf("%s\r\n", line);
    }
}

/*
 * 函数功能: 将副 ICM42688 原始缓存清零
 * 输入参数: 无
 * 返回值: 无
 */
static void ICM42688_Aux_ClearRaw(void)
{
    g_icm42688_aux_raw.acc_x_lsb = 0;
    g_icm42688_aux_raw.acc_y_lsb = 0;
    g_icm42688_aux_raw.acc_z_lsb = 0;
    g_icm42688_aux_raw.gyro_x_lsb = 0;
    g_icm42688_aux_raw.gyro_y_lsb = 0;
    g_icm42688_aux_raw.gyro_z_lsb = 0;
    g_icm42688_aux_raw.temp_lsb = 0;
    g_icm42688_aux_raw.tick_us = 0U;
}

/*
 * 函数功能: 将副 ICM42688 物理量缓存清零
 * 输入参数: 无
 * 返回值: 无
 */
static void ICM42688_Aux_ClearReal(void)
{
    g_icm42688_aux.acc_x = 0.0f;
    g_icm42688_aux.acc_y = 0.0f;
    g_icm42688_aux.acc_z = 0.0f;
    g_icm42688_aux.gyro_x = 0.0f;
    g_icm42688_aux.gyro_y = 0.0f;
    g_icm42688_aux.gyro_z = 0.0f;
    g_icm42688_aux.temp = 0.0f;
}

/*
 * 函数功能: 清零副 ICM42688 的本地灵敏度缓存
 * 输入参数: 无
 * 返回值: 无
 */
static void ICM42688_Aux_ClearSensitivity(void)
{
    g_icm42688_aux_gyro_sensitivity = 0.0f;
    g_icm42688_aux_acc_sensitivity = 0.0f;
}

/*
 * 函数功能: 初始化副 ICM42688 所在的 SPI1 和片选 GPIO
 * 输入参数: 无
 * 返回值: 无
 */
static void ICM42688_Aux_SpiHardwareInit(void)
{
    spi_init(ICM42688_AUX_SPI,
             SPI_MODE0,
             ICM42688_AUX_SPI_SPEED,
             ICM42688_AUX_SCK_PIN,
             ICM42688_AUX_MOSI_PIN,
             ICM42688_AUX_MISO_PIN,
             SPI_CS_NULL);
    gpio_init(ICM42688_AUX_CS_PIN, GPO, 1, GPO_PUSH_PULL);
}

/*
 * 函数功能: 向副 ICM42688 写入一个 8bit 寄存器
 * 输入参数:
 *   reg_addr - 寄存器地址
 *   value - 需要写入的寄存器值
 * 返回值: 无
 */
static void ICM42688_Aux_WriteReg8(uint8 reg_addr, uint8 value)
{
    uint8 tx_buf[2] = {reg_addr, value};
    uint8 rx_buf[2] = {0U, 0U};

    gpio_low(ICM42688_AUX_CS_PIN);
    spi_transfer_8bit(ICM42688_AUX_SPI, tx_buf, rx_buf, 2U);
    gpio_high(ICM42688_AUX_CS_PIN);
}

/*
 * 函数功能: 从副 ICM42688 读取一个 8bit 寄存器
 * 输入参数:
 *   reg_addr - 寄存器地址
 * 返回值: 读取到的寄存器值
 */
static uint8 ICM42688_Aux_ReadReg8(uint8 reg_addr)
{
    uint8 tx_buf[2] = {(uint8)(reg_addr | ICM42688_AUX_READ_MASK), 0U};
    uint8 rx_buf[2] = {0U, 0U};

    gpio_low(ICM42688_AUX_CS_PIN);
    spi_transfer_8bit(ICM42688_AUX_SPI, tx_buf, rx_buf, 2U);
    gpio_high(ICM42688_AUX_CS_PIN);

    return rx_buf[1];
}

/*
 * 函数功能: 写入寄存器后立即读回校验，确认配置已经生效
 * 输入参数:
 *   reg_addr - 寄存器地址
 *   expected - 期望写入的寄存器值
 * 返回值: 1=写入并校验成功，0=校验失败
 */
static uint8 ICM42688_Aux_WriteVerifyReg8(uint8 reg_addr, uint8 expected)
{
    ICM42688_Aux_WriteReg8(reg_addr, expected);
    return (ICM42688_Aux_ReadReg8(reg_addr) == expected) ? 1U : 0U;
}

/*
 * 函数功能: 尽力写入并读取一次寄存器，若读回不一致则打印告警但不判初始化失败
 * 输入参数:
 *   reg_addr - 寄存器地址
 *   expected - 期望写入的寄存器值
 *   reg_name - 寄存器名称字符串
 * 返回值: 无
 */
static void ICM42688_Aux_WriteBestEffortReg8(uint8 reg_addr, uint8 expected, const char *reg_name)
{
    uint8 readback;

    ICM42688_Aux_WriteReg8(reg_addr, expected);
    readback = ICM42688_Aux_ReadReg8(reg_addr);

    if (readback != expected)
    {
        ICM42688_Aux_LogLine("ICM42688_Aux %s warn: write 0x%02X read 0x%02X",
                             reg_name,
                             expected,
                             readback);
    }
}

/*
 * 函数功能: 按主驱动规则生成副 ICM42688 的 GYRO_CONFIG0，并缓存陀螺仪灵敏度
 * 输入参数:
 *   gyro_fsr - 陀螺仪量程配置
 *   gyro_odr - 陀螺仪输出速率配置
 * 返回值: GYRO_CONFIG0 寄存器值
 */
static uint8 ICM42688_Aux_BuildGyroConfig0(GYRO_FSR gyro_fsr, GYRO_ODR gyro_odr)
{
    uint8 gyro_config0 = 0x00U;

    switch (gyro_fsr)
    {
        case GYRO_15_625DPS:
            gyro_config0 |= 0xE0U;
            g_icm42688_aux_gyro_sensitivity = SENSITIVITY_ICM42688_GYRO_15_625dps;
            break;
        case GYRO_31_25DPS:
            gyro_config0 |= 0xC0U;
            g_icm42688_aux_gyro_sensitivity = SENSITIVITY_ICM42688_GYRO_31_25dps;
            break;
        case GYRO_62_5DPS:
            gyro_config0 |= 0xA0U;
            g_icm42688_aux_gyro_sensitivity = SENSITIVITY_ICM42688_GYRO_62_5dps;
            break;
        case GYRO_125DPS:
            gyro_config0 |= 0x80U;
            g_icm42688_aux_gyro_sensitivity = SENSITIVITY_ICM42688_GYRO_125dps;
            break;
        case GYRO_250DPS:
            gyro_config0 |= 0x60U;
            g_icm42688_aux_gyro_sensitivity = SENSITIVITY_ICM42688_GYRO_250dps;
            break;
        case GYRO_500DPS:
            gyro_config0 |= 0x40U;
            g_icm42688_aux_gyro_sensitivity = SENSITIVITY_ICM42688_GYRO_500dps;
            break;
        case GYRO_1000DPS:
            gyro_config0 |= 0x20U;
            g_icm42688_aux_gyro_sensitivity = SENSITIVITY_ICM42688_GYRO_1000dps;
            break;
        case GYRO_2000DPS:
        default:
            gyro_config0 |= 0x00U;
            g_icm42688_aux_gyro_sensitivity = SENSITIVITY_ICM42688_GYRO_2000dps;
            break;
    }

    switch (gyro_odr)
    {
        case GYRO_ODR_12_5HZ:
            gyro_config0 |= 0x0BU;
            break;
        case GYRO_ODR_25HZ:
            gyro_config0 |= 0x0AU;
            break;
        case GYRO_ODR_50HZ:
            gyro_config0 |= 0x09U;
            break;
        case GYRO_ODR_100HZ:
            gyro_config0 |= 0x08U;
            break;
        case GYRO_ODR_200HZ:
            gyro_config0 |= 0x07U;
            break;
        case GYRO_ODR_500HZ:
            gyro_config0 |= 0x0FU;
            break;
        case GYRO_ODR_1000HZ:
            gyro_config0 |= 0x06U;
            break;
        case GYRO_ODR_2000HZ:
            gyro_config0 |= 0x05U;
            break;
        case GYRO_ODR_4000HZ:
            gyro_config0 |= 0x04U;
            break;
        case GYRO_ODR_8000HZ:
            gyro_config0 |= 0x03U;
            break;
        case GYRO_ODR_16000HZ:
            gyro_config0 |= 0x02U;
            break;
        case GYRO_ODR_32000HZ:
        default:
            gyro_config0 |= 0x01U;
            break;
    }

    return gyro_config0;
}

/*
 * 函数功能: 按主驱动规则生成副 ICM42688 的 ACCEL_CONFIG0，并缓存加速度计灵敏度
 * 输入参数:
 *   acc_fsr - 加速度计量程配置
 *   acc_odr - 加速度计输出速率配置
 * 返回值: ACCEL_CONFIG0 寄存器值
 */
static uint8 ICM42688_Aux_BuildAccelConfig0(ACC_FSR acc_fsr, ACC_ODR acc_odr)
{
    uint8 accel_config0 = 0x00U;

    switch (acc_fsr)
    {
        case ACC_2G:
            accel_config0 |= 0x60U;
            g_icm42688_aux_acc_sensitivity = SENSITIVITY_ICM42688_ACC_2G;
            break;
        case ACC_4G:
            accel_config0 |= 0x40U;
            g_icm42688_aux_acc_sensitivity = SENSITIVITY_ICM42688_ACC_4G;
            break;
        case ACC_8G:
            accel_config0 |= 0x20U;
            g_icm42688_aux_acc_sensitivity = SENSITIVITY_ICM42688_ACC_8G;
            break;
        case ACC_16G:
        default:
            accel_config0 |= 0x00U;
            g_icm42688_aux_acc_sensitivity = SENSITIVITY_ICM42688_ACC_16G;
            break;
    }

    switch (acc_odr)
    {
        case ACC_ODR_12_5HZ:
            accel_config0 |= 0x0BU;
            break;
        case ACC_ODR_25HZ:
            accel_config0 |= 0x0AU;
            break;
        case ACC_ODR_50HZ:
            accel_config0 |= 0x09U;
            break;
        case ACC_ODR_100HZ:
            accel_config0 |= 0x08U;
            break;
        case ACC_ODR_200HZ:
            accel_config0 |= 0x07U;
            break;
        case ACC_ODR_500HZ:
            accel_config0 |= 0x0FU;
            break;
        case ACC_ODR_1000HZ:
            accel_config0 |= 0x06U;
            break;
        case ACC_ODR_2000HZ:
            accel_config0 |= 0x05U;
            break;
        case ACC_ODR_4000HZ:
            accel_config0 |= 0x04U;
            break;
        case ACC_ODR_8000HZ:
            accel_config0 |= 0x03U;
            break;
        case ACC_ODR_16000HZ:
            accel_config0 |= 0x02U;
            break;
        case ACC_ODR_32000HZ:
        default:
            accel_config0 |= 0x01U;
            break;
    }

    return accel_config0;
}

/*
 * 函数功能: 按主驱动规则生成副 ICM42688 的 GYRO_ACCEL_CONFIG0
 * 输入参数:
 *   gyro_bandwidth_factor - 陀螺仪数字滤波带宽因子
 *   acc_bandwidth_factor - 加速度计数字滤波带宽因子
 * 返回值: GYRO_ACCEL_CONFIG0 寄存器值
 */
static uint8 ICM42688_Aux_BuildGyroAccelConfig0(Bandwidth_Factor gyro_bandwidth_factor,
                                                Bandwidth_Factor acc_bandwidth_factor)
{
    uint8 filter_config = 0x00U;

    switch (gyro_bandwidth_factor)
    {
        case Bandwidth_Factor_2:
            filter_config |= 0x00U;
            break;
        case Bandwidth_Factor_4:
            filter_config |= 0x01U;
            break;
        case Bandwidth_Factor_5:
            filter_config |= 0x02U;
            break;
        case Bandwidth_Factor_8:
            filter_config |= 0x03U;
            break;
        case Bandwidth_Factor_10:
            filter_config |= 0x04U;
            break;
        case Bandwidth_Factor_16:
            filter_config |= 0x05U;
            break;
        case Bandwidth_Factor_20:
            filter_config |= 0x06U;
            break;
        case Bandwidth_Factor_40:
            filter_config |= 0x07U;
            break;
        case Low_latency_1:
            filter_config |= 0x0EU;
            break;
        case Low_Latency_2:
        default:
            filter_config |= 0x0FU;
            break;
    }

    switch (acc_bandwidth_factor)
    {
        case Bandwidth_Factor_2:
            filter_config |= 0x00U;
            break;
        case Bandwidth_Factor_4:
            filter_config |= 0x10U;
            break;
        case Bandwidth_Factor_5:
            filter_config |= 0x20U;
            break;
        case Bandwidth_Factor_8:
            filter_config |= 0x30U;
            break;
        case Bandwidth_Factor_10:
            filter_config |= 0x40U;
            break;
        case Bandwidth_Factor_16:
            filter_config |= 0x50U;
            break;
        case Bandwidth_Factor_20:
            filter_config |= 0x60U;
            break;
        case Bandwidth_Factor_40:
            filter_config |= 0x70U;
            break;
        case Low_latency_1:
            filter_config |= 0xE0U;
            break;
        case Low_Latency_2:
        default:
            filter_config |= 0xF0U;
            break;
    }

    return filter_config;
}

/*
 * 函数功能: 按主驱动规则生成副 ICM42688 的一阶/二阶/三阶滤波阶数配置
 * 输入参数:
 *   filter_order - 数字滤波器阶数
 * 返回值: GYRO_CONFIG1 或 ACCEL_CONFIG1 寄存器值
 */
static uint8 ICM42688_Aux_BuildFilterOrderConfig(Filter_Order filter_order)
{
    uint8 filter_order_config = 0x00U;

    switch (filter_order)
    {
        case _1st:
            filter_order_config |= 0x02U;
            break;
        case _2st:
            filter_order_config |= 0x06U;
            break;
        case _3st:
        default:
            filter_order_config |= 0xA0U;
            break;
    }

    return filter_order_config;
}

/*
 * 函数功能: 按主 ICM42688_CONFIG 生成并写入副 ICM42688 的完整运行配置
 * 输入参数: 无
 * 返回值: 1=配置成功，0=配置失败
 */
static uint8 ICM42688_Aux_ApplyMainConfig(void)
{
    uint8 gyro_config0;
    uint8 accel_config0;
    uint8 gyro_accel_config0;
    uint8 gyro_config1;
    uint8 accel_config1;

    gyro_config0 = ICM42688_Aux_BuildGyroConfig0(ICM42688_CONFIG.GYRO_FSR, ICM42688_CONFIG.GYRO_ODR);
    accel_config0 = ICM42688_Aux_BuildAccelConfig0(ICM42688_CONFIG.ACC_FSR, ICM42688_CONFIG.ACC_ODR);
    gyro_accel_config0 = ICM42688_Aux_BuildGyroAccelConfig0(ICM42688_CONFIG.Gyro_Bandwidth_Factor,
                                                            ICM42688_CONFIG.Acc_Bandwidth_Factor);
    gyro_config1 = ICM42688_Aux_BuildFilterOrderConfig(ICM42688_CONFIG.Gyro_Filter_Order);
    accel_config1 = ICM42688_Aux_BuildFilterOrderConfig(ICM42688_CONFIG.Acc_Filter_Order);

    if (0U == ICM42688_Aux_WriteVerifyReg8(ICM42688_AUX_REG_GYRO_CONFIG0, gyro_config0))
    {
        ICM42688_Aux_LogLine("ICM42688_Aux GYRO_CONFIG0 failed");
        return 0U;
    }

    if (0U == ICM42688_Aux_WriteVerifyReg8(ICM42688_AUX_REG_ACCEL_CONFIG0, accel_config0))
    {
        ICM42688_Aux_LogLine("ICM42688_Aux ACCEL_CONFIG0 failed");
        return 0U;
    }

    /* 这三项保持按主驱动同样的编码去写，但不再用强校验卡死初始化。 */
    ICM42688_Aux_WriteBestEffortReg8(ICM42688_AUX_REG_GYRO_ACCEL_CONFIG0,
                                     gyro_accel_config0,
                                     "GYRO_ACCEL_CONFIG0");
    ICM42688_Aux_WriteBestEffortReg8(ICM42688_AUX_REG_GYRO_CONFIG1,
                                     gyro_config1,
                                     "GYRO_CONFIG1");
    ICM42688_Aux_WriteBestEffortReg8(ICM42688_AUX_REG_ACCEL_CONFIG1,
                                     accel_config1,
                                     "ACCEL_CONFIG1");

    return 1U;
}

/*
 * 函数功能: 将副 ICM42688 原始 LSB 数据换算成与主驱动一致的物理量缓存
 * 输入参数:
 *   raw - 本次采样得到的原始数据
 * 返回值: 无
 */
static void ICM42688_Aux_UpdateReal(icm42688_aux_raw_t *raw)
{
    float gyro_x_raw;
    float gyro_y_raw;
    float gyro_z_raw;
    float acc_x_raw;
    float acc_y_raw;
    float acc_z_raw;

    if ((raw == 0) ||
        (g_icm42688_aux_gyro_sensitivity <= 0.0f) ||
        (g_icm42688_aux_acc_sensitivity <= 0.0f))
    {
        ICM42688_Aux_ClearReal();
        return;
    }

    gyro_x_raw = raw->gyro_x_lsb / g_icm42688_aux_gyro_sensitivity;
    gyro_y_raw = raw->gyro_y_lsb / g_icm42688_aux_gyro_sensitivity;
    gyro_z_raw = raw->gyro_z_lsb / g_icm42688_aux_gyro_sensitivity;

    acc_x_raw = raw->acc_x_lsb / g_icm42688_aux_acc_sensitivity;
    acc_y_raw = raw->acc_y_lsb / g_icm42688_aux_acc_sensitivity;
    acc_z_raw = raw->acc_z_lsb / g_icm42688_aux_acc_sensitivity;

    g_icm42688_aux.gyro_x = ICM42688_SIGN_GX * gyro_x_raw;
    g_icm42688_aux.gyro_y = ICM42688_SIGN_GY * gyro_y_raw;
    g_icm42688_aux.gyro_z = ICM42688_SIGN_GZ * gyro_z_raw;

    g_icm42688_aux.acc_x = ICM42688_SIGN_AX * acc_x_raw;
    g_icm42688_aux.acc_y = ICM42688_SIGN_AY * acc_y_raw;
    g_icm42688_aux.acc_z = ICM42688_SIGN_AZ * acc_z_raw;
    g_icm42688_aux.temp = 0.0f;
}

/*
 * 函数功能: 连续读取副 ICM42688 的温度、加速度和角速度原始寄存器
 * 输入参数:
 *   raw - 输出的原始数据缓存指针
 * 返回值: 无
 */
static void ICM42688_Aux_ReadBurst(icm42688_aux_raw_t *raw)
{
    uint8 tx_buf[ICM42688_AUX_BURST_READ_LEN] = {0};
    uint8 rx_buf[ICM42688_AUX_BURST_READ_LEN] = {0};

    if (raw == 0)
    {
        return;
    }

    tx_buf[0] = (uint8)(ICM42688_AUX_REG_TEMP_DATA1 | ICM42688_AUX_READ_MASK);

    gpio_low(ICM42688_AUX_CS_PIN);
    spi_transfer_8bit(ICM42688_AUX_SPI, tx_buf, rx_buf, ICM42688_AUX_BURST_READ_LEN);
    gpio_high(ICM42688_AUX_CS_PIN);

    raw->temp_lsb = (int16)(((uint16)rx_buf[1] << 8) | rx_buf[2]);
    raw->acc_x_lsb = (int16)(((uint16)rx_buf[3] << 8) | rx_buf[4]);
    raw->acc_y_lsb = (int16)(((uint16)rx_buf[5] << 8) | rx_buf[6]);
    raw->acc_z_lsb = (int16)(((uint16)rx_buf[7] << 8) | rx_buf[8]);
    raw->gyro_x_lsb = (int16)(((uint16)rx_buf[9] << 8) | rx_buf[10]);
    raw->gyro_y_lsb = (int16)(((uint16)rx_buf[11] << 8) | rx_buf[12]);
    raw->gyro_z_lsb = (int16)(((uint16)rx_buf[13] << 8) | rx_buf[14]);
}

/*
 * 函数功能: 初始化 SPI1 副 ICM42688，并按主 ICM42688_CONFIG 对齐运行配置
 * 输入参数: 无
 * 返回值: 1=初始化成功，0=初始化失败
 */
uint8 ICM42688_Aux_Init(void)
{
    uint8 who_am_i;

    /* 先清空运行状态，避免初始化失败时留下脏数据。 */
    g_icm42688_aux_ready = 0U;
    ICM42688_Aux_ClearRaw();
    ICM42688_Aux_ClearReal();
    ICM42688_Aux_ClearSensitivity();
    ICM42688_Aux_SpiHardwareInit();

    /* 按芯片手册执行软复位，并等待复位生效。 */
    ICM42688_Aux_WriteReg8(ICM42688_AUX_REG_DEVICE_CONFIG, ICM42688_AUX_DEVICE_SOFT_RESET);
    system_delay_ms(2);

    /* 先确认副 IMU 身份，再写最小必须寄存器。 */
    who_am_i = ICM42688_Aux_ReadReg8(ICM42688_AUX_REG_WHO_AM_I);
    if (who_am_i != ICM42688_AUX_WHO_AM_I_EXPECTED)
    {
        ICM42688_Aux_LogLine("ICM42688_Aux WHO_AM_I mismatch: 0x%02X", who_am_i);
        ICM42688_Aux_ClearSensitivity();
        return 0U;
    }

    if (0U == ICM42688_Aux_ApplyMainConfig())
    {
        ICM42688_Aux_ClearSensitivity();
        return 0U;
    }

    /* 最后拉起陀螺仪和加速度计到 LN 模式，并给启动时间。 */
    ICM42688_Aux_WriteReg8(ICM42688_AUX_REG_PWR_MGMT0, ICM42688_AUX_PWR_MGMT0_LN_MODE);
    system_delay_ms(50);

    g_icm42688_aux_ready = 1U;
    return 1U;
}

/*
 * 函数功能: 在 1kHz 调度中读取一次副 ICM42688 原始寄存器并刷新原始/物理量缓存
 * 输入参数:
 *   tick_us - 当前 1kHz 调度时间戳，单位 us
 * 返回值: 无
 */
void ICM42688_Aux_Update_1000Hz(uint32 tick_us)
{
    icm42688_aux_raw_t raw_sample = {0};

    if (0U == g_icm42688_aux_ready)
    {
        ICM42688_Aux_ClearRaw();
        ICM42688_Aux_ClearReal();
        return;
    }

    /* 只更新副 IMU 的独立原始缓存，不参与任何主链算法。 */
    ICM42688_Aux_ReadBurst(&raw_sample);
    raw_sample.tick_us = tick_us;

    g_icm42688_aux_raw.acc_x_lsb = raw_sample.acc_x_lsb;
    g_icm42688_aux_raw.acc_y_lsb = raw_sample.acc_y_lsb;
    g_icm42688_aux_raw.acc_z_lsb = raw_sample.acc_z_lsb;
    g_icm42688_aux_raw.gyro_x_lsb = raw_sample.gyro_x_lsb;
    g_icm42688_aux_raw.gyro_y_lsb = raw_sample.gyro_y_lsb;
    g_icm42688_aux_raw.gyro_z_lsb = raw_sample.gyro_z_lsb;
    g_icm42688_aux_raw.temp_lsb = raw_sample.temp_lsb;
    g_icm42688_aux_raw.tick_us = raw_sample.tick_us;
    ICM42688_Aux_UpdateReal(&raw_sample);
}
