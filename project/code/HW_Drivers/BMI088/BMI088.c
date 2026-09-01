#include "BMI088.h"

#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

/* BMI088 使用的 SPI 控制器 */
#define BMI088_SPI                              (SPI_1)
/* BMI088 的 SPI SCK 引脚 */
#define BMI088_SCK_PIN                          (SPI1_CLK_P12_2)
/* BMI088 的 SPI MOSI 引脚 */
#define BMI088_MOSI_PIN                         (SPI1_MOSI_P12_1)
/* BMI088 的 SPI MISO 引脚 */
#define BMI088_MISO_PIN                         (SPI1_MISO_P12_0)
/* BMI088 加速度计片选引脚，对应模块 CS1/CSB1 */
#define BMI088_ACC_CS_PIN                       (P12_3)
/* BMI088 陀螺仪片选引脚，对应模块 CS2/CSB2 */
#define BMI088_GYRO_CS_PIN                      (P12_4)
/* BMI088 的 SPI 通信速率，芯片手册最大 10MHz */
#define BMI088_SPI_SPEED                        (10U * 1000U * 1000U)

/* SPI 读寄存器命令位 */
#define BMI088_SPI_READ_MASK                    (0x80U)
/* SPI 写寄存器命令位 */
#define BMI088_SPI_WRITE_MASK                   (0x7FU)
/* 连续读取传感器数据的最大字节数 */
#define BMI088_MAX_BURST_READ_LEN               (6U)

/* 加速度计 CHIP_ID 寄存器地址 */
#define BMI088_ACC_REG_CHIP_ID                  (0x00U)
/* 加速度计 X 轴数据低字节寄存器地址 */
#define BMI088_ACC_REG_X_LSB                    (0x12U)
/* 加速度计配置寄存器地址 */
#define BMI088_ACC_REG_CONF                     (0x40U)
/* 加速度计量程寄存器地址 */
#define BMI088_ACC_REG_RANGE                    (0x41U)
/* 加速度计电源配置寄存器地址 */
#define BMI088_ACC_REG_PWR_CONF                 (0x7CU)
/* 加速度计电源控制寄存器地址 */
#define BMI088_ACC_REG_PWR_CTRL                 (0x7DU)
/* 加速度计软复位寄存器地址 */
#define BMI088_ACC_REG_SOFTRESET                (0x7EU)

/* 陀螺仪 CHIP_ID 寄存器地址 */
#define BMI088_GYRO_REG_CHIP_ID                 (0x00U)
/* 陀螺仪 X 轴数据低字节寄存器地址 */
#define BMI088_GYRO_REG_RATE_X_LSB              (0x02U)
/* 陀螺仪量程寄存器地址 */
#define BMI088_GYRO_REG_RANGE                   (0x0FU)
/* 陀螺仪带宽和 ODR 寄存器地址 */
#define BMI088_GYRO_REG_BW                      (0x10U)
/* 陀螺仪电源模式寄存器地址 */
#define BMI088_GYRO_REG_LPM1                    (0x11U)
/* 陀螺仪软复位寄存器地址 */
#define BMI088_GYRO_REG_SOFTRESET               (0x14U)

/* BMI088 加速度计期望 CHIP_ID */
#define BMI088_ACC_CHIP_ID_EXPECTED             (0x1EU)
/* BMI088 陀螺仪期望 CHIP_ID */
#define BMI088_GYRO_CHIP_ID_EXPECTED            (0x0FU)
/* BMI088 软复位命令值 */
#define BMI088_SOFTRESET_CMD                    (0xB6U)

/* 加速度计进入正常功耗模式 */
#define BMI088_ACC_PWR_CONF_ACTIVE              (0x00U)
/* 加速度计使能命令值 */
#define BMI088_ACC_PWR_CTRL_ENABLE              (0x04U)
/* 加速度计 1600Hz ODR，normal 带宽 */
#define BMI088_ACC_CONF_1600HZ_NORMAL           (0xACU)
/* 加速度计量程 +/-24g */
#define BMI088_ACC_RANGE_24G                    (0x03U)
/* 陀螺仪量程 +/-2000dps */
#define BMI088_GYRO_RANGE_2000DPS               (0x00U)
/* 陀螺仪 1000Hz ODR 默认低通档 */
#define BMI088_GYRO_BW_1000HZ                   (0x82U)
/* 陀螺仪正常工作模式 */
#define BMI088_GYRO_LPM1_NORMAL                 (0x00U)

/* 加速度计 +/-24g 灵敏度，单位 LSB/g */
#define BMI088_ACC_SENSITIVITY_LSB_PER_G        (1365.0f)
/* 陀螺仪 +/-2000dps 灵敏度，单位 LSB/dps */
#define BMI088_GYRO_SENSITIVITY_LSB_PER_DPS     (16.384f)

/* BMI088 X 轴角速度极性，保持与当前主 IMU 输出语义一致 */
#define BMI088_SIGN_GX                          (1.0f)
/* BMI088 Y 轴角速度极性，保持与当前主 IMU 输出语义一致 */
#define BMI088_SIGN_GY                          (-1.0f)
/* BMI088 Z 轴角速度极性，保持与当前主 IMU 输出语义一致 */
#define BMI088_SIGN_GZ                          (-1.0f)
/* BMI088 X 轴加速度极性，保持与当前主 IMU 输出语义一致 */
#define BMI088_SIGN_AX                          (1.0f)
/* BMI088 Y 轴加速度极性，保持与当前主 IMU 输出语义一致 */
#define BMI088_SIGN_AY                          (-1.0f)
/* BMI088 Z 轴加速度极性，静止平放目标为 az 约 -1g */
#define BMI088_SIGN_AZ                          (-1.0f)

volatile bmi088_raw_t g_bmi088_raw = {0};       /* BMI088 原始 LSB 数据缓存 */
volatile bmi088_real_t g_bmi088 = {0};          /* BMI088 物理量数据缓存 */
volatile uint8 g_bmi088_ready = 0U;             /* BMI088 初始化成功标志 */
volatile uint8 g_bmi088_acc_chip_id = 0U;       /* BMI088 加速度计芯片 ID */
volatile uint8 g_bmi088_gyro_chip_id = 0U;      /* BMI088 陀螺仪芯片 ID */

/*
 * 函数功能: 清空 BMI088 原始数据缓存
 * 输入参数: 无
 * 返回值: 无
 */
static void BMI088_ClearRaw(void)
{
    g_bmi088_raw.acc_x_lsb = 0;
    g_bmi088_raw.acc_y_lsb = 0;
    g_bmi088_raw.acc_z_lsb = 0;
    g_bmi088_raw.gyro_x_lsb = 0;
    g_bmi088_raw.gyro_y_lsb = 0;
    g_bmi088_raw.gyro_z_lsb = 0;
    g_bmi088_raw.temp_lsb = 0;
    g_bmi088_raw.tick_us = 0U;
}

/*
 * 函数功能: 清空 BMI088 物理量数据缓存
 * 输入参数: 无
 * 返回值: 无
 */
static void BMI088_ClearReal(void)
{
    g_bmi088.acc_x = 0.0f;
    g_bmi088.acc_y = 0.0f;
    g_bmi088.acc_z = 0.0f;
    g_bmi088.gyro_x = 0.0f;
    g_bmi088.gyro_y = 0.0f;
    g_bmi088.gyro_z = 0.0f;
    g_bmi088.temp = 0.0f;
}

/*
 * 函数功能: 初始化 BMI088 使用的 SPI1 和两个片选 GPIO
 * 输入参数: 无
 * 返回值: 无
 */
static void BMI088_SpiHardwareInit(void)
{
    spi_init(BMI088_SPI,
             SPI_MODE0,
             BMI088_SPI_SPEED,
             BMI088_SCK_PIN,
             BMI088_MOSI_PIN,
             BMI088_MISO_PIN,
             SPI_CS_NULL);
    gpio_init(BMI088_ACC_CS_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(BMI088_GYRO_CS_PIN, GPO, 1, GPO_PUSH_PULL);
}

/*
 * 函数功能: 向 BMI088 加速度计写入一个 8bit 寄存器
 * 输入参数:
 *   reg_addr - 加速度计寄存器地址
 *   value - 需要写入的寄存器值
 * 返回值: 无
 */
static void BMI088_AccWriteReg8(uint8 reg_addr, uint8 value)
{
    uint8 tx_buf[2] = {(uint8)(reg_addr & BMI088_SPI_WRITE_MASK), value};
    uint8 rx_buf[2] = {0U, 0U};

    gpio_low(BMI088_ACC_CS_PIN);
    spi_transfer_8bit(BMI088_SPI, tx_buf, rx_buf, 2U);
    gpio_high(BMI088_ACC_CS_PIN);
    system_delay_us(2U);
}

/*
 * 函数功能: 从 BMI088 加速度计连续读取寄存器，读操作自动丢弃 dummy 字节
 * 输入参数:
 *   reg_addr - 加速度计寄存器起始地址
 *   buf - 读取结果缓存
 *   len - 读取字节数，最大 6
 * 返回值: 1=读取成功，0=参数错误
 */
static uint8 BMI088_AccReadRegs(uint8 reg_addr, uint8 *buf, uint8 len)
{
    uint8 i;
    uint8 tx_buf[BMI088_MAX_BURST_READ_LEN + 2U] = {0};
    uint8 rx_buf[BMI088_MAX_BURST_READ_LEN + 2U] = {0};

    if ((0 == buf) || (0U == len) || (len > BMI088_MAX_BURST_READ_LEN))
    {
        return 0U;
    }

    tx_buf[0] = (uint8)(reg_addr | BMI088_SPI_READ_MASK);

    gpio_low(BMI088_ACC_CS_PIN);
    spi_transfer_8bit(BMI088_SPI, tx_buf, rx_buf, (uint32)(len + 2U));
    gpio_high(BMI088_ACC_CS_PIN);

    for (i = 0U; i < len; ++i)
    {
        buf[i] = rx_buf[i + 2U];
    }

    return 1U;
}

/*
 * 函数功能: 从 BMI088 加速度计读取一个 8bit 寄存器
 * 输入参数:
 *   reg_addr - 加速度计寄存器地址
 * 返回值: 读取到的寄存器值
 */
static uint8 BMI088_AccReadReg8(uint8 reg_addr)
{
    uint8 value = 0U;

    (void)BMI088_AccReadRegs(reg_addr, &value, 1U);
    return value;
}

/*
 * 函数功能: 向 BMI088 陀螺仪写入一个 8bit 寄存器
 * 输入参数:
 *   reg_addr - 陀螺仪寄存器地址
 *   value - 需要写入的寄存器值
 * 返回值: 无
 */
static void BMI088_GyroWriteReg8(uint8 reg_addr, uint8 value)
{
    uint8 tx_buf[2] = {(uint8)(reg_addr & BMI088_SPI_WRITE_MASK), value};
    uint8 rx_buf[2] = {0U, 0U};

    gpio_low(BMI088_GYRO_CS_PIN);
    spi_transfer_8bit(BMI088_SPI, tx_buf, rx_buf, 2U);
    gpio_high(BMI088_GYRO_CS_PIN);
    system_delay_us(2U);
}

/*
 * 函数功能: 从 BMI088 陀螺仪连续读取寄存器
 * 输入参数:
 *   reg_addr - 陀螺仪寄存器起始地址
 *   buf - 读取结果缓存
 *   len - 读取字节数，最大 6
 * 返回值: 1=读取成功，0=参数错误
 */
static uint8 BMI088_GyroReadRegs(uint8 reg_addr, uint8 *buf, uint8 len)
{
    uint8 i;
    uint8 tx_buf[BMI088_MAX_BURST_READ_LEN + 1U] = {0};
    uint8 rx_buf[BMI088_MAX_BURST_READ_LEN + 1U] = {0};

    if ((0 == buf) || (0U == len) || (len > BMI088_MAX_BURST_READ_LEN))
    {
        return 0U;
    }

    tx_buf[0] = (uint8)(reg_addr | BMI088_SPI_READ_MASK);

    gpio_low(BMI088_GYRO_CS_PIN);
    spi_transfer_8bit(BMI088_SPI, tx_buf, rx_buf, (uint32)(len + 1U));
    gpio_high(BMI088_GYRO_CS_PIN);

    for (i = 0U; i < len; ++i)
    {
        buf[i] = rx_buf[i + 1U];
    }

    return 1U;
}

/*
 * 函数功能: 从 BMI088 陀螺仪读取一个 8bit 寄存器
 * 输入参数:
 *   reg_addr - 陀螺仪寄存器地址
 * 返回值: 读取到的寄存器值
 */
static uint8 BMI088_GyroReadReg8(uint8 reg_addr)
{
    uint8 value = 0U;

    (void)BMI088_GyroReadRegs(reg_addr, &value, 1U);
    return value;
}

/*
 * 函数功能: 写入加速度计寄存器并读回校验
 * 输入参数:
 *   reg_addr - 加速度计寄存器地址
 *   expected - 期望写入的寄存器值
 * 返回值: 1=校验成功，0=校验失败
 */
static uint8 BMI088_AccWriteVerifyReg8(uint8 reg_addr, uint8 expected)
{
    BMI088_AccWriteReg8(reg_addr, expected);
    return (BMI088_AccReadReg8(reg_addr) == expected) ? 1U : 0U;
}

/*
 * 函数功能: 写入陀螺仪寄存器并读回校验
 * 输入参数:
 *   reg_addr - 陀螺仪寄存器地址
 *   expected - 期望写入的寄存器值
 * 返回值: 1=校验成功，0=校验失败
 */
static uint8 BMI088_GyroWriteVerifyReg8(uint8 reg_addr, uint8 expected)
{
    BMI088_GyroWriteReg8(reg_addr, expected);
    return (BMI088_GyroReadReg8(reg_addr) == expected) ? 1U : 0U;
}

/*
 * 函数功能: 对 BMI088 加速度计执行一次 dummy read，使其进入 SPI 模式
 * 输入参数: 无
 * 返回值: 无
 */
static void BMI088_AccEnableSpiMode(void)
{
    (void)BMI088_AccReadReg8(BMI088_ACC_REG_CHIP_ID);
    system_delay_ms(1U);
}

/*
 * 函数功能: 配置 BMI088 加速度计量程、ODR 和电源模式
 * 输入参数: 无
 * 返回值: 1=配置成功，0=配置失败
 */
static uint8 BMI088_ApplyAccConfig(void)
{
    BMI088_AccWriteReg8(BMI088_ACC_REG_PWR_CONF, BMI088_ACC_PWR_CONF_ACTIVE);
    system_delay_ms(1U);
    BMI088_AccWriteReg8(BMI088_ACC_REG_PWR_CTRL, BMI088_ACC_PWR_CTRL_ENABLE);
    system_delay_ms(50U);

    if (0U == BMI088_AccWriteVerifyReg8(BMI088_ACC_REG_RANGE, BMI088_ACC_RANGE_24G))
    {
        return 0U;
    }

    if (0U == BMI088_AccWriteVerifyReg8(BMI088_ACC_REG_CONF, BMI088_ACC_CONF_1600HZ_NORMAL))
    {
        return 0U;
    }

    return 1U;
}

/*
 * 函数功能: 配置 BMI088 陀螺仪量程、ODR 和电源模式
 * 输入参数: 无
 * 返回值: 1=配置成功，0=配置失败
 */
static uint8 BMI088_ApplyGyroConfig(void)
{
    BMI088_GyroWriteReg8(BMI088_GYRO_REG_LPM1, BMI088_GYRO_LPM1_NORMAL);
    system_delay_ms(30U);

    if (0U == BMI088_GyroWriteVerifyReg8(BMI088_GYRO_REG_RANGE, BMI088_GYRO_RANGE_2000DPS))
    {
        return 0U;
    }

    if (0U == BMI088_GyroWriteVerifyReg8(BMI088_GYRO_REG_BW, BMI088_GYRO_BW_1000HZ))
    {
        return 0U;
    }

    return 1U;
}

/*
 * 函数功能: 将 BMI088 原始 LSB 数据换算为机体系物理量
 * 输入参数:
 *   raw - 本次采样得到的原始数据
 * 返回值: 无
 */
static void BMI088_UpdateReal(bmi088_raw_t *raw)
{
    float gyro_x_raw;
    float gyro_y_raw;
    float gyro_z_raw;
    float acc_x_raw;
    float acc_y_raw;
    float acc_z_raw;

    if (0 == raw)
    {
        BMI088_ClearReal();
        return;
    }

    gyro_x_raw = ((float)raw->gyro_x_lsb) / BMI088_GYRO_SENSITIVITY_LSB_PER_DPS;
    gyro_y_raw = ((float)raw->gyro_y_lsb) / BMI088_GYRO_SENSITIVITY_LSB_PER_DPS;
    gyro_z_raw = ((float)raw->gyro_z_lsb) / BMI088_GYRO_SENSITIVITY_LSB_PER_DPS;

    acc_x_raw = ((float)raw->acc_x_lsb) / BMI088_ACC_SENSITIVITY_LSB_PER_G;
    acc_y_raw = ((float)raw->acc_y_lsb) / BMI088_ACC_SENSITIVITY_LSB_PER_G;
    acc_z_raw = ((float)raw->acc_z_lsb) / BMI088_ACC_SENSITIVITY_LSB_PER_G;

    g_bmi088.gyro_x = BMI088_SIGN_GX * gyro_x_raw;
    g_bmi088.gyro_y = BMI088_SIGN_GY * gyro_y_raw;
    g_bmi088.gyro_z = BMI088_SIGN_GZ * gyro_z_raw;

    g_bmi088.acc_x = BMI088_SIGN_AX * acc_x_raw;
    g_bmi088.acc_y = BMI088_SIGN_AY * acc_y_raw;
    g_bmi088.acc_z = BMI088_SIGN_AZ * acc_z_raw;
    g_bmi088.temp = 0.0f;
}

/*
 * 函数功能: 连续读取 BMI088 加速度计和陀螺仪原始数据
 * 输入参数:
 *   raw - 输出的原始数据缓存指针
 * 返回值: 1=读取成功，0=读取失败
 */
static uint8 BMI088_ReadBurst(bmi088_raw_t *raw)
{
    uint8 acc_buf[6] = {0};
    uint8 gyro_buf[6] = {0};

    if (0 == raw)
    {
        return 0U;
    }

    if (0U == BMI088_AccReadRegs(BMI088_ACC_REG_X_LSB, acc_buf, 6U))
    {
        return 0U;
    }

    if (0U == BMI088_GyroReadRegs(BMI088_GYRO_REG_RATE_X_LSB, gyro_buf, 6U))
    {
        return 0U;
    }

    raw->acc_x_lsb = (int16)(((uint16)acc_buf[1] << 8) | acc_buf[0]);
    raw->acc_y_lsb = (int16)(((uint16)acc_buf[3] << 8) | acc_buf[2]);
    raw->acc_z_lsb = (int16)(((uint16)acc_buf[5] << 8) | acc_buf[4]);
    raw->gyro_x_lsb = (int16)(((uint16)gyro_buf[1] << 8) | gyro_buf[0]);
    raw->gyro_y_lsb = (int16)(((uint16)gyro_buf[3] << 8) | gyro_buf[2]);
    raw->gyro_z_lsb = (int16)(((uint16)gyro_buf[5] << 8) | gyro_buf[4]);
    raw->temp_lsb = 0;

    return 1U;
}

/*
 * 函数功能: 初始化 SPI1 BMI088，并写入默认量程、ODR 和电源配置
 * 输入参数: 无
 * 返回值: 1=初始化成功，0=初始化失败
 */
uint8 BMI088_Init(void)
{
    g_bmi088_ready = 0U;
    g_bmi088_acc_chip_id = 0U;
    g_bmi088_gyro_chip_id = 0U;
    BMI088_ClearRaw();
    BMI088_ClearReal();
    BMI088_SpiHardwareInit();

    /* 加速度计上电默认不是 SPI，需要先通过 CS1 产生一次 SPI 访问。 */
    BMI088_AccEnableSpiMode();
    g_bmi088_acc_chip_id = BMI088_AccReadReg8(BMI088_ACC_REG_CHIP_ID);
    g_bmi088_gyro_chip_id = BMI088_GyroReadReg8(BMI088_GYRO_REG_CHIP_ID);

    if ((g_bmi088_acc_chip_id != BMI088_ACC_CHIP_ID_EXPECTED) ||
        (g_bmi088_gyro_chip_id != BMI088_GYRO_CHIP_ID_EXPECTED))
    {
        return 0U;
    }

    BMI088_AccWriteReg8(BMI088_ACC_REG_SOFTRESET, BMI088_SOFTRESET_CMD);
    BMI088_GyroWriteReg8(BMI088_GYRO_REG_SOFTRESET, BMI088_SOFTRESET_CMD);
    system_delay_ms(50U);

    BMI088_AccEnableSpiMode();
    g_bmi088_acc_chip_id = BMI088_AccReadReg8(BMI088_ACC_REG_CHIP_ID);
    g_bmi088_gyro_chip_id = BMI088_GyroReadReg8(BMI088_GYRO_REG_CHIP_ID);

    if ((g_bmi088_acc_chip_id != BMI088_ACC_CHIP_ID_EXPECTED) ||
        (g_bmi088_gyro_chip_id != BMI088_GYRO_CHIP_ID_EXPECTED))
    {
        return 0U;
    }

    if (0U == BMI088_ApplyAccConfig())
    {
        return 0U;
    }

    if (0U == BMI088_ApplyGyroConfig())
    {
        return 0U;
    }

    g_bmi088_ready = 1U;
    return 1U;
}

/*
 * 函数功能: 在 1kHz 调度中读取一次 BMI088 加速度计和陀螺仪数据，并刷新缓存
 * 输入参数:
 *   tick_us - 当前 1kHz 调度时间戳，单位 us
 * 返回值: 无
 */
void BMI088_Update_1000Hz(uint32 tick_us)
{
    bmi088_raw_t raw_sample = {0};

    if (0U == g_bmi088_ready)
    {
        BMI088_ClearRaw();
        BMI088_ClearReal();
        return;
    }

    if (0U == BMI088_ReadBurst(&raw_sample))
    {
        return;
    }

    raw_sample.tick_us = tick_us;
    g_bmi088_raw.acc_x_lsb = raw_sample.acc_x_lsb;
    g_bmi088_raw.acc_y_lsb = raw_sample.acc_y_lsb;
    g_bmi088_raw.acc_z_lsb = raw_sample.acc_z_lsb;
    g_bmi088_raw.gyro_x_lsb = raw_sample.gyro_x_lsb;
    g_bmi088_raw.gyro_y_lsb = raw_sample.gyro_y_lsb;
    g_bmi088_raw.gyro_z_lsb = raw_sample.gyro_z_lsb;
    g_bmi088_raw.temp_lsb = raw_sample.temp_lsb;
    g_bmi088_raw.tick_us = raw_sample.tick_us;
    BMI088_UpdateReal(&raw_sample);
}
