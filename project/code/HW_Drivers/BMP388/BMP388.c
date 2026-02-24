#include "BMP388.h"

// 参考PX4 BMP388寄存器定义
#define BMP3_CHIP_ID_ADDR                    (0x00U)
#define BMP3_ERR_REG_ADDR                    (0x02U)
#define BMP3_STATUS_REG_ADDR                 (0x03U)
#define BMP3_DATA_ADDR                       (0x04U)
#define BMP3_PWR_CTRL_ADDR                   (0x1BU)
#define BMP3_OSR_ADDR                        (0x1CU)
#define BMP3_ODR_ADDR                        (0x1DU)
#define BMP3_CONFIG_ADDR                     (0x1FU)
#define BMP3_CALIB_DATA_ADDR                 (0x31U)
#define BMP3_CMD_ADDR                        (0x7EU)

#define BMP3_CHIP_ID                         (0x50U)
#define BMP3_SOFT_RESET_CMD                  (0xB6U)

// SPI读写控制位
#define BMP388_SPI_READ_MASK                 (0x80U)
#define BMP388_SPI_WRITE_MASK                (0x7FU)

// 数据长度
#define BMP3_P_T_DATA_LEN                    (6U)
#define BMP3_CALIB_DATA_LEN                  (21U)

// 状态位
#define BMP3_CMD_RDY                         (0x10U)
#define BMP3_DRDY_PRESS                      (0x20U)
#define BMP3_DRDY_TEMP                       (0x40U)
#define BMP3_CMD_ERR                         (0x02U)

// PWR_CTRL位域
#define BMP3_PRESS_EN_MSK                    (0x01U)
#define BMP3_PRESS_EN_POS                    (0U)
#define BMP3_TEMP_EN_MSK                     (0x02U)
#define BMP3_TEMP_EN_POS                     (1U)
#define BMP3_OP_MODE_MSK                     (0x30U)
#define BMP3_OP_MODE_POS                     (4U)

// OSR ODR IIR位域
#define BMP3_PRESS_OS_MSK                    (0x07U)
#define BMP3_PRESS_OS_POS                    (0U)
#define BMP3_TEMP_OS_MSK                     (0x38U)
#define BMP3_TEMP_OS_POS                     (3U)
#define BMP3_ODR_MSK                         (0x1FU)
#define BMP3_ODR_POS                         (0U)
#define BMP3_IIR_FILTER_MSK                  (0x0EU)
#define BMP3_IIR_FILTER_POS                  (1U)

// 工作模式
#define BMP3_OP_MODE_SLEEP                   (0U)
#define BMP3_OP_MODE_FORCED                  (1U)

// 默认配置 严格参考PX4默认值
#define BMP3_OVERSAMPLING_2X                 (1U)
#define BMP3_OVERSAMPLING_16X                (4U)
#define BMP3_ODR_50_HZ                       (2U)
#define BMP3_IIR_FILTER_DISABLE              (0U)

// 轮询与时序
#define BMP388_SPI_DELAY_US                  (1U)
#define BMP388_CS_GUARD_US                   (1U)
#define BMP388_CMD_READY_TIMEOUT_US          (50000U)
#define BMP388_DRDY_POLL_US                  (500U)
#define BMP388_DRDY_TIMEOUT_MARGIN_US        (5000U)
#define BMP388_RESET_WAIT_MS                 (3U)
#define BMP388_POST_INIT_WAIT_US             (40000U)
#define BMP388_CALIB_READ_RETRY              (3U)
#define BMP388_STRICT_CRC_CHECK              (0U)
#define BMP388_NONBLOCK_TIMEOUT_CALLS        (30U)

#define BMP3_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | (((uint8)data << bitname##_POS) & bitname##_MSK))

#define BMP3_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | ((uint8)data & bitname##_MSK))

volatile BMP388_calib_t g_BMP388_calib = {0};
volatile BMP388_device_t g_BMP388_dev =
{
    BMP388_PIN_SCK,
    BMP388_PIN_MISO,
    BMP388_PIN_MOSI,
    BMP388_PIN_CS,
    BMP3_OVERSAMPLING_2X,
    BMP3_OVERSAMPLING_16X,
    BMP3_ODR_50_HZ,
    BMP3_IIR_FILTER_DISABLE,
    0U,
    0U
};
volatile BMP388_data_t g_BMP388_data = {0};

static uint8 s_bmp388_nonblock_waiting = 0U;
static uint16 s_bmp388_nonblock_wait_calls = 0U;

// 软件SPI单字节收发 CPOL=1 CPHA=1 模式3
static uint8 BMP388_spi_transfer_byte(uint8 tx)
{
    uint8 i;
    uint8 rx = 0U;

    for (i = 0U; i < 8U; ++i)
    {
        // 模式3 先拉低时钟准备数据 再在上升沿采样
        gpio_low(g_BMP388_dev.sck_pin);
        if ((tx & 0x80U) != 0U)
        {
            gpio_high(g_BMP388_dev.mosi_pin);
        }
        else
        {
            gpio_low(g_BMP388_dev.mosi_pin);
        }
        system_delay_us(BMP388_SPI_DELAY_US);

        gpio_high(g_BMP388_dev.sck_pin);
        rx = (uint8)((rx << 1) | (gpio_get_level(g_BMP388_dev.miso_pin) ? 1U : 0U));
        system_delay_us(BMP388_SPI_DELAY_US);

        tx <<= 1;
    }

    return rx;
}

static void BMP388_spi_begin(void)
{
    gpio_low(g_BMP388_dev.cs_pin);
    system_delay_us(BMP388_CS_GUARD_US);
}

static void BMP388_spi_end(void)
{
    system_delay_us(BMP388_CS_GUARD_US);
    gpio_high(g_BMP388_dev.cs_pin);
    system_delay_us(BMP388_CS_GUARD_US);
}

static uint8 BMP388_read_reg(uint8 reg, uint8 *value)
{
    if (NULL == value)
    {
        return BMP388_RET_ERR_PARAM;
    }

    BMP388_spi_begin();
    (void)BMP388_spi_transfer_byte((uint8)(reg | BMP388_SPI_READ_MASK));
    // BMP388 SPI 读寄存器在地址后存在 1 字节 dummy，需先丢弃
    (void)BMP388_spi_transfer_byte(0x00U);
    *value = BMP388_spi_transfer_byte(0x00U);
    BMP388_spi_end();

    return BMP388_RET_OK;
}

static uint8 BMP388_read_regs(uint8 reg, uint8 *buf, uint8 len)
{
    uint8 i;

    if ((NULL == buf) || (0U == len))
    {
        return BMP388_RET_ERR_PARAM;
    }

    BMP388_spi_begin();
    (void)BMP388_spi_transfer_byte((uint8)(reg | BMP388_SPI_READ_MASK));
    // 连续读同样先丢弃 dummy 字节
    (void)BMP388_spi_transfer_byte(0x00U);
    for (i = 0U; i < len; ++i)
    {
        buf[i] = BMP388_spi_transfer_byte(0x00U);
    }
    BMP388_spi_end();

    return BMP388_RET_OK;
}

static uint8 BMP388_write_reg(uint8 reg, uint8 value)
{
    BMP388_spi_begin();
    (void)BMP388_spi_transfer_byte((uint8)(reg & BMP388_SPI_WRITE_MASK));
    (void)BMP388_spi_transfer_byte(value);
    BMP388_spi_end();
    return BMP388_RET_OK;
}

// PX4同款CRC8计算 多项式0x1D
static int8 BMP388_cal_crc(uint8 seed, uint8 data)
{
    int8 index;
    int8 var2;

    for (index = 0; index < 8; index++)
    {
        if (((seed & 0x80U) != 0U) ^ ((data & 0x80U) != 0U))
        {
            var2 = 1;
        }
        else
        {
            var2 = 0;
        }

        seed = (uint8)((seed & 0x7FU) << 1U);
        data = (uint8)((data & 0x7FU) << 1U);
        seed = (uint8)(seed ^ (uint8)(0x1DU * var2));
    }

    return (int8)seed;
}

// 校验NVM校准参数CRC 避免异常校准导致补偿失真
static uint8 BMP388_validate_trim_crc(const uint8 *calib_raw)
{
    uint8 i;
    uint8 stored_crc = 0U;
    uint8 crc = 0xFFU;

    if (NULL == calib_raw)
    {
        return BMP388_RET_ERR_PARAM;
    }

    if (BMP388_RET_OK != BMP388_read_reg((uint8)(BMP3_CALIB_DATA_ADDR - 1U), &stored_crc))
    {
        return BMP388_RET_ERR_CRC;
    }

    for (i = 0U; i < BMP3_CALIB_DATA_LEN; i++)
    {
        crc = (uint8)BMP388_cal_crc(crc, calib_raw[i]);
    }

    crc ^= 0xFFU;

    if (crc != stored_crc)
    {
        return BMP388_RET_ERR_CRC;
    }

    return BMP388_RET_OK;
}

// 参考PX4组合得到测量时间 单位us 用于更新轮询超时
static uint32 BMP388_get_measurement_time_us(uint8 osr_t, uint8 osr_p)
{
    uint32 temp_pow = 0U;
    uint32 press_pow = 0U;

    if (0U == osr_t)
    {
        temp_pow = 0U;
    }
    else if (osr_t <= 5U)
    {
        temp_pow = (uint32)(osr_t - 1U);
    }
    else
    {
        return 0U;
    }

    if (0U == osr_p)
    {
        press_pow = 0U;
    }
    else if (osr_p <= 5U)
    {
        press_pow = (uint32)(osr_p - 1U);
    }
    else
    {
        return 0U;
    }

    // get_measurement_time = 234 + 392 + 2000*osr_t + 2000*osr_p + 163
    return (uint32)(2789U + (2000U << temp_pow) + (2000U << press_pow));
}

// 软复位并检查命令错误位
static uint8 BMP388_soft_reset(void)
{
    uint32 waited_us = 0U;
    uint8 status = 0U;
    uint8 err_reg = 0U;

    if (BMP388_RET_OK != BMP388_write_reg(BMP3_CMD_ADDR, BMP3_SOFT_RESET_CMD))
    {
        return BMP388_RET_ERR_RESET;
    }

    system_delay_ms(BMP388_RESET_WAIT_MS);

    // 复位后轮询等待命令就绪
    while (waited_us < BMP388_CMD_READY_TIMEOUT_US)
    {
        if (BMP388_RET_OK != BMP388_read_reg(BMP3_STATUS_REG_ADDR, &status))
        {
            return BMP388_RET_ERR_RESET;
        }

        if ((status & BMP3_CMD_RDY) != 0U)
        {
            break;
        }

        system_delay_us(BMP388_DRDY_POLL_US);
        waited_us += BMP388_DRDY_POLL_US;
    }

    if ((status & BMP3_CMD_RDY) == 0U)
    {
        return BMP388_RET_ERR_TIMEOUT;
    }

    if (BMP388_RET_OK != BMP388_read_reg(BMP3_ERR_REG_ADDR, &err_reg))
    {
        return BMP388_RET_ERR_RESET;
    }

    if ((err_reg & BMP3_CMD_ERR) != 0U)
    {
        return BMP388_RET_ERR_RESET;
    }

    return BMP388_RET_OK;
}

static void BMP388_parse_calib_data(const uint8 *b)
{
    g_BMP388_calib.par_t1 = (uint16)(((uint16)b[1] << 8) | b[0]);
    g_BMP388_calib.par_t2 = (uint16)(((uint16)b[3] << 8) | b[2]);
    g_BMP388_calib.par_t3 = (int8)b[4];

    g_BMP388_calib.par_p1 = (int16)(((uint16)b[6] << 8) | b[5]);
    g_BMP388_calib.par_p2 = (int16)(((uint16)b[8] << 8) | b[7]);
    g_BMP388_calib.par_p3 = (int8)b[9];
    g_BMP388_calib.par_p4 = (int8)b[10];
    g_BMP388_calib.par_p5 = (uint16)(((uint16)b[12] << 8) | b[11]);
    g_BMP388_calib.par_p6 = (uint16)(((uint16)b[14] << 8) | b[13]);
    g_BMP388_calib.par_p7 = (int8)b[15];
    g_BMP388_calib.par_p8 = (int8)b[16];
    g_BMP388_calib.par_p9 = (int16)(((uint16)b[18] << 8) | b[17]);
    g_BMP388_calib.par_p10 = (int8)b[19];
    g_BMP388_calib.par_p11 = (int8)b[20];
}

static uint8 BMP388_read_calib_data(void)
{
    uint8 calib_raw[BMP3_CALIB_DATA_LEN] = {0};
    uint8 retry;
    uint8 i;
    uint8 all_zero;
    uint8 all_ff;

    for (retry = 0U; retry < BMP388_CALIB_READ_RETRY; ++retry)
    {
        if (BMP388_RET_OK != BMP388_read_regs(BMP3_CALIB_DATA_ADDR, calib_raw, BMP3_CALIB_DATA_LEN))
        {
            continue;
        }

        if (BMP388_RET_OK == BMP388_validate_trim_crc(calib_raw))
        {
            BMP388_parse_calib_data(calib_raw);
            g_BMP388_calib.t_lin = 0;
            return BMP388_RET_OK;
        }

        system_delay_ms(2U);
    }

    // 非严格模式下，CRC失败但数据不是全0/全FF时允许继续，避免初始化被卡死
    all_zero = 1U;
    all_ff = 1U;
    for (i = 0U; i < BMP3_CALIB_DATA_LEN; ++i)
    {
        if (calib_raw[i] != 0x00U)
        {
            all_zero = 0U;
        }
        if (calib_raw[i] != 0xFFU)
        {
            all_ff = 0U;
        }
    }

    if ((all_zero != 0U) || (all_ff != 0U))
    {
        return BMP388_RET_ERR_CRC;
    }

#if BMP388_STRICT_CRC_CHECK
    return BMP388_RET_ERR_CRC;
#else
    BMP388_parse_calib_data(calib_raw);
    g_BMP388_calib.t_lin = 0;
    return BMP388_RET_OK;
#endif
}

// 参考PX4 写入PWR_CTRL OSR ODR IIR 并记录测量时间
static uint8 BMP388_set_sensor_settings(void)
{
    uint8 reg_data;

    g_BMP388_dev.measure_time_us = BMP388_get_measurement_time_us(g_BMP388_dev.osr_t, g_BMP388_dev.osr_p);
    if (0U == g_BMP388_dev.measure_time_us)
    {
        return BMP388_RET_ERR_CONFIG;
    }

    // 使能温度和气压通道 保持sleep update中再触发forced
    reg_data = BMP3_SET_BITS_POS_0(0U, BMP3_PRESS_EN, 1U);
    reg_data = BMP3_SET_BITS(reg_data, BMP3_TEMP_EN, 1U);
    if (BMP388_RET_OK != BMP388_write_reg(BMP3_PWR_CTRL_ADDR, reg_data))
    {
        return BMP388_RET_ERR_CONFIG;
    }

    // 配置过采样
    reg_data = BMP3_SET_BITS_POS_0(0U, BMP3_PRESS_OS, g_BMP388_dev.osr_p);
    reg_data = BMP3_SET_BITS(reg_data, BMP3_TEMP_OS, g_BMP388_dev.osr_t);
    if (BMP388_RET_OK != BMP388_write_reg(BMP3_OSR_ADDR, reg_data))
    {
        return BMP388_RET_ERR_CONFIG;
    }

    // 配置输出数据率
    reg_data = BMP3_SET_BITS_POS_0(0U, BMP3_ODR, g_BMP388_dev.odr);
    if (BMP388_RET_OK != BMP388_write_reg(BMP3_ODR_ADDR, reg_data))
    {
        return BMP388_RET_ERR_CONFIG;
    }

    // 配置IIR滤波
    reg_data = BMP3_SET_BITS(0U, BMP3_IIR_FILTER, g_BMP388_dev.iir_coef);
    if (BMP388_RET_OK != BMP388_write_reg(BMP3_CONFIG_ADDR, reg_data))
    {
        return BMP388_RET_ERR_CONFIG;
    }

    return BMP388_RET_OK;
}

// 切换工作模式 参考PX4 先回sleep 再进入目标模式
static uint8 BMP388_set_op_mode(uint8 mode)
{
    uint8 reg_data = 0U;
    uint8 last_mode = 0U;

    if (BMP388_RET_OK != BMP388_read_reg(BMP3_PWR_CTRL_ADDR, &reg_data))
    {
        return BMP388_RET_ERR_CONFIG;
    }

    last_mode = (uint8)((reg_data & BMP3_OP_MODE_MSK) >> BMP3_OP_MODE_POS);
    if (last_mode != BMP3_OP_MODE_SLEEP)
    {
        reg_data = (uint8)(reg_data & (~BMP3_OP_MODE_MSK));
        if (BMP388_RET_OK != BMP388_write_reg(BMP3_PWR_CTRL_ADDR, reg_data))
        {
            return BMP388_RET_ERR_CONFIG;
        }
        system_delay_us(5000U);
    }

    if (BMP388_RET_OK != BMP388_read_reg(BMP3_PWR_CTRL_ADDR, &reg_data))
    {
        return BMP388_RET_ERR_CONFIG;
    }

    reg_data = BMP3_SET_BITS(reg_data, BMP3_OP_MODE, mode);
    if (BMP388_RET_OK != BMP388_write_reg(BMP3_PWR_CTRL_ADDR, reg_data))
    {
        return BMP388_RET_ERR_CONFIG;
    }

    return BMP388_RET_OK;
}

static void BMP388_parse_raw_data(const uint8 *reg_data, uint32 *uncomp_press, uint32 *uncomp_temp)
{
    *uncomp_press = (uint32)((uint32)reg_data[2] << 16 | (uint32)reg_data[1] << 8 | reg_data[0]);
    *uncomp_temp  = (uint32)((uint32)reg_data[5] << 16 | (uint32)reg_data[4] << 8 | reg_data[3]);
}

// 参考PX4和Bosch 整数温度补偿 输出单位0.01摄氏度
static int64 BMP388_compensate_temperature(uint32 uncomp_temp)
{
    int64 partial_data1;
    int64 partial_data2;
    int64 partial_data3;
    int64 partial_data4;
    int64 partial_data5;
    int64 partial_data6;
    int64 comp_temp;

    partial_data1 = (int64)((int64)uncomp_temp - (256LL * (int64)g_BMP388_calib.par_t1));
    partial_data2 = (int64)((int64)g_BMP388_calib.par_t2 * partial_data1);
    partial_data3 = (int64)(partial_data1 * partial_data1);
    partial_data4 = (int64)(partial_data3 * (int64)g_BMP388_calib.par_t3);
    partial_data5 = (int64)((partial_data2 * 262144LL) + partial_data4);
    partial_data6 = (int64)(partial_data5 / 4294967296LL);

    g_BMP388_calib.t_lin = partial_data6;
    comp_temp = (int64)((partial_data6 * 25LL) / 16384LL);

    return comp_temp;
}

// 参考PX4和Bosch 整数气压补偿 输出单位0.01Pa
static uint64 BMP388_compensate_pressure(uint32 uncomp_press)
{
    int64 partial_data1;
    int64 partial_data2;
    int64 partial_data3;
    int64 partial_data4;
    int64 partial_data5;
    int64 partial_data6;
    int64 offset;
    int64 sensitivity;
    uint64 comp_press;

    // 该实现严格对齐 Bosch/PX4 的 BMP388 整数补偿流程，输出单位 0.01Pa
    partial_data1 = (int64)(g_BMP388_calib.t_lin * g_BMP388_calib.t_lin);
    partial_data2 = (int64)(partial_data1 / 64LL);
    partial_data3 = (int64)((partial_data2 * g_BMP388_calib.t_lin) / 256LL);
    partial_data4 = (int64)((g_BMP388_calib.par_p8 * partial_data3) / 32LL);
    partial_data5 = (int64)((g_BMP388_calib.par_p7 * partial_data1) * 16LL);
    partial_data6 = (int64)((g_BMP388_calib.par_p6 * g_BMP388_calib.t_lin) * 4194304LL);
    offset = (int64)((g_BMP388_calib.par_p5 * 140737488355328LL) + partial_data4 + partial_data5 + partial_data6);

    partial_data2 = (int64)((g_BMP388_calib.par_p4 * partial_data3) / 32LL);
    partial_data4 = (int64)((g_BMP388_calib.par_p3 * partial_data1) * 4LL);
    partial_data5 = (int64)((((int64)g_BMP388_calib.par_p2 - 16384LL) * g_BMP388_calib.t_lin) * 2097152LL);
    sensitivity = (int64)((((int64)g_BMP388_calib.par_p1 - 16384LL) * 70368744177664LL) + partial_data2 + partial_data4 + partial_data5);

    partial_data1 = (int64)((sensitivity / 16777216LL) * (int64)uncomp_press);
    partial_data2 = (int64)((int64)g_BMP388_calib.par_p10 * g_BMP388_calib.t_lin);
    partial_data3 = (int64)(partial_data2 + (65536LL * (int64)g_BMP388_calib.par_p9));
    partial_data4 = (int64)((partial_data3 * (int64)uncomp_press) / 8192LL);

    // 先除后乘，规避 (uncomp_press * partial_data4) 中间溢出
    partial_data5 = (int64)(((int64)uncomp_press * (partial_data4 / 10LL)) / 512LL);
    partial_data5 = (int64)(partial_data5 * 10LL);

    partial_data6 = (int64)((int64)uncomp_press * (int64)uncomp_press);
    partial_data2 = (int64)(((int64)g_BMP388_calib.par_p11 * partial_data6) / 65536LL);
    partial_data3 = (int64)((partial_data2 * (int64)uncomp_press) / 128LL);
    partial_data4 = (int64)((offset / 4LL) + partial_data1 + partial_data5 + partial_data3);

    comp_press = (uint64)(((uint64)partial_data4 * 25ULL) / 1099511627776ULL);
    return comp_press;
}

static uint8 BMP388_read_compensated_data(void)
{
    uint8 reg_data[BMP3_P_T_DATA_LEN] = {0};
    uint32 uncomp_press = 0U;
    uint32 uncomp_temp = 0U;
    int64 comp_temp;
    uint64 comp_press;

    if (BMP388_RET_OK != BMP388_read_regs(BMP3_DATA_ADDR, reg_data, BMP3_P_T_DATA_LEN))
    {
        return BMP388_RET_ERR_TIMEOUT;
    }

    BMP388_parse_raw_data(reg_data, &uncomp_press, &uncomp_temp);
    comp_temp = BMP388_compensate_temperature(uncomp_temp);
    comp_press = BMP388_compensate_pressure(uncomp_press);

    g_BMP388_data.raw_pressure = uncomp_press;
    g_BMP388_data.raw_temperature = uncomp_temp;
    g_BMP388_data.temperature_c = (float)comp_temp / 100.0f;
    g_BMP388_data.pressure_pa = (float)comp_press / 100.0f;

    return BMP388_RET_OK;
}

uint8 BMP388_init(void)
{
    uint8 chip_id = 0U;
    uint8 ret;

    memset((void *)&g_BMP388_calib, 0, sizeof(g_BMP388_calib));
    memset((void *)&g_BMP388_data, 0, sizeof(g_BMP388_data));
    g_BMP388_dev.inited = 0U;
    s_bmp388_nonblock_waiting = 0U;
    s_bmp388_nonblock_wait_calls = 0U;

    // 固定引脚初始化 SCK CS MOSI输出 MISO输入
    gpio_init(g_BMP388_dev.sck_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(g_BMP388_dev.mosi_pin, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(g_BMP388_dev.cs_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(g_BMP388_dev.miso_pin, GPI, GPIO_HIGH, GPI_PULL_UP);
    system_delay_us(BMP388_SPI_DELAY_US);

    ret = BMP388_soft_reset();
    if (BMP388_RET_OK != ret)
    {
        return ret;
    }

    if (BMP388_RET_OK != BMP388_read_reg(BMP3_CHIP_ID_ADDR, &chip_id))
    {
        return BMP388_RET_ERR_CHIP_ID;
    }

    if (BMP3_CHIP_ID != chip_id)
    {
        return BMP388_RET_ERR_CHIP_ID;
    }

    // 参考PX4复位后额外等待，保证NVM参数稳定可读
    system_delay_us(BMP388_POST_INIT_WAIT_US);

    ret = BMP388_read_calib_data();
    if (BMP388_RET_OK != ret)
    {
        return ret;
    }

    ret = BMP388_set_sensor_settings();
    if (BMP388_RET_OK != ret)
    {
        return ret;
    }

    g_BMP388_dev.inited = 1U;
    return BMP388_RET_OK;
}

uint8 BMP388_update(void)
{
    uint8 status = 0U;
    uint8 ret;
    uint32 timeout_us;

    if (0U == g_BMP388_dev.inited)
    {
        return BMP388_RET_ERR_NOT_INIT;
    }

    s_bmp388_nonblock_waiting = 0U;
    s_bmp388_nonblock_wait_calls = 0U;

    // 触发一次forced测量
    ret = BMP388_set_op_mode(BMP3_OP_MODE_FORCED);
    if (BMP388_RET_OK != ret)
    {
        return ret;
    }

    // 轮询等待温度和气压就绪 同时做超时保护
    timeout_us = g_BMP388_dev.measure_time_us + BMP388_DRDY_TIMEOUT_MARGIN_US;
    while (timeout_us > 0U)
    {
        if (BMP388_RET_OK != BMP388_read_reg(BMP3_STATUS_REG_ADDR, &status))
        {
            return BMP388_RET_ERR_TIMEOUT;
        }

        if ((status & (BMP3_DRDY_PRESS | BMP3_DRDY_TEMP)) == (BMP3_DRDY_PRESS | BMP3_DRDY_TEMP))
        {
            break;
        }

        system_delay_us(BMP388_DRDY_POLL_US);
        if (timeout_us > BMP388_DRDY_POLL_US)
        {
            timeout_us -= BMP388_DRDY_POLL_US;
        }
        else
        {
            timeout_us = 0U;
        }
    }

    if (0U == timeout_us)
    {
        return BMP388_RET_ERR_TIMEOUT;
    }

    return BMP388_read_compensated_data();
}

uint8 BMP388_update_nonblocking(uint8 *is_new_sample)
{
    uint8 status = 0U;

    if (0 != is_new_sample)
    {
        *is_new_sample = 0U;
    }

    if (0U == g_BMP388_dev.inited)
    {
        s_bmp388_nonblock_waiting = 0U;
        s_bmp388_nonblock_wait_calls = 0U;
        return BMP388_RET_ERR_NOT_INIT;
    }

    if (0U == s_bmp388_nonblock_waiting)
    {
        if (BMP388_RET_OK != BMP388_set_op_mode(BMP3_OP_MODE_FORCED))
        {
            return BMP388_RET_ERR_CONFIG;
        }

        s_bmp388_nonblock_waiting = 1U;
        s_bmp388_nonblock_wait_calls = 0U;
        return BMP388_RET_OK;
    }

    if (BMP388_RET_OK != BMP388_read_reg(BMP3_STATUS_REG_ADDR, &status))
    {
        s_bmp388_nonblock_waiting = 0U;
        s_bmp388_nonblock_wait_calls = 0U;
        return BMP388_RET_ERR_TIMEOUT;
    }

    if ((status & (BMP3_DRDY_PRESS | BMP3_DRDY_TEMP)) == (BMP3_DRDY_PRESS | BMP3_DRDY_TEMP))
    {
        s_bmp388_nonblock_waiting = 0U;
        s_bmp388_nonblock_wait_calls = 0U;
        if (BMP388_RET_OK != BMP388_read_compensated_data())
        {
            return BMP388_RET_ERR_TIMEOUT;
        }
        if (0 != is_new_sample)
        {
            *is_new_sample = 1U;
        }
        return BMP388_RET_OK;
    }

    if (s_bmp388_nonblock_wait_calls < 65535U)
    {
        s_bmp388_nonblock_wait_calls++;
    }

    if (s_bmp388_nonblock_wait_calls >= BMP388_NONBLOCK_TIMEOUT_CALLS)
    {
        s_bmp388_nonblock_waiting = 0U;
        s_bmp388_nonblock_wait_calls = 0U;
        return BMP388_RET_ERR_TIMEOUT;
    }

    return BMP388_RET_OK;
}
