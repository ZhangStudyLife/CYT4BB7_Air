#include "PMW3901.h"

/* ======================== 寄存器地址定义 ======================== */
#define PMW3901_REG_PRODUCT_ID         (0x00U)     /* 产品ID寄存器地址 */
#define PMW3901_REG_MOTION             (0x02U)     /* 运动检测寄存器地址 */
#define PMW3901_REG_DELTA_X_L          (0x03U)     /* X轴像素位移低字节寄存器 */
#define PMW3901_REG_DELTA_X_H          (0x04U)     /* X轴像素位移高字节寄存器 */
#define PMW3901_REG_DELTA_Y_L          (0x05U)     /* Y轴像素位移低字节寄存器 */
#define PMW3901_REG_DELTA_Y_H          (0x06U)     /* Y轴像素位移高字节寄存器 */
#define PMW3901_REG_POWER_RST          (0x3AU)     /* 电源复位寄存器地址 */
#define PMW3901_REG_INV_PROD_ID2       (0x5FU)     /* 反码产品ID寄存器地址，值应为~PRODUCT_ID */
#define PMW3901_REG_MOT_BURST2         (0x16U)     /* Burst读取启动寄存器地址 */

/* ======================== 常量定义 ======================== */
#define PMW3901_PRODUCT_ID_3901        (0x49U)     /* PMW3901的产品ID期望值 */
#define PMW3901_POWER_ON_RESET_CMD     (0x5AU)     /* 上电复位命令字 */
#define PMW3901_WRITE_FLAG             (0x80U)     /* SPI写操作标志位（寄存器地址最高位置1） */
#define PMW3901_DUMMY_BYTE             (0x00U)     /* SPI读取时发送的填充字节 */

/* ======================== 时序延迟参数 ======================== */
#define PMW3901_POWERUP_DELAY_MS       (50U)       /* 上电复位后等待时间 ms */
#define PMW3901_STAGE_GAP_DELAY_MS     (100U)      /* 两阶段初始化之间的间隔时间 ms */
#define PMW3901_READY_DELAY_MS         (50U)       /* 初始化完成后的就绪等待时间 ms */
#define PMW3901_REG_RETRY_MAX          (5U)        /* 寄存器写入/ID校验最大重试次数 */

/* ======================== SPI时序延迟（微秒） ======================== */
#define PMW3901_TSRAD_US               (300U)      /* 地址字节与数据字节之间的延迟 tSRAD */
#define PMW3901_TSR_US                 (500U)      /* 读操作地址后的等待时间 tSR */
#define PMW3901_TSWX_US                (120U)      /* 写操作完成后的等待时间 tSWx */
#define PMW3901_TSRX_US                (200U)      /* 读操作完成后的等待时间 tSRx */
#define PMW3901_TBEXIT_US              (1U)        /* Burst读取结束后的CS拉高延迟 tBEXIT */
#define PMW3901_MOTION_BURST_DELAY_US  (150U)      /* Burst读取命令发送后的等待时间 */
#define PMW3901_INIT_FLUSH_COUNT       (3U)        /* 初始化后丢弃的脏数据帧数 */
#define PMW3901_FLUSH_INTERVAL_MS      (20U)       /* 丢弃帧之间的间隔时间 ms */

/**
 * @brief 寄存器配置项结构体，用于初始化寄存器表
 */
typedef struct
{
    uint8 reg;      /* 寄存器地址 */
    uint8 value;    /* 要写入的值 */
} pmw3901_reg_cfg_t;

/* 第一阶段初始化寄存器配置表（传感器内部参数调优） */
static const pmw3901_reg_cfg_t pmw3901_init_stage1[] =
{
    { 0x7F, 0x00 },
    { 0x61, 0xAD },
    { 0x7F, 0x03 },
    { 0x40, 0x00 },
    { 0x7F, 0x05 },
    { 0x41, 0xB3 },
    { 0x43, 0xF1 },
    { 0x45, 0x14 },
    { 0x5B, 0x32 },
    { 0x5F, 0x34 },
    { 0x7B, 0x08 },
    { 0x7F, 0x06 },
    { 0x44, 0x1B },
    { 0x40, 0xBF },
    { 0x4E, 0x3F },
    { 0x7F, 0x08 },
    { 0x65, 0x20 },
    { 0x6A, 0x18 },
    { 0x7F, 0x09 },
    { 0x4F, 0xAF },
    { 0x5F, 0x40 },
    { 0x48, 0x80 },
    { 0x49, 0x80 },
    { 0x57, 0x77 },
    { 0x60, 0x78 },
    { 0x61, 0x78 },
    { 0x62, 0x08 },
    { 0x63, 0x50 },
    { 0x7F, 0x0A },
    { 0x45, 0x60 },
    { 0x7F, 0x00 },
    { 0x4D, 0x11 },
    { 0x55, 0x80 },
    { 0x74, 0x1F },
    { 0x75, 0x1F },
    { 0x4A, 0x78 },
    { 0x4B, 0x78 },
    { 0x44, 0x08 },
    { 0x45, 0x50 },
    { 0x64, 0xFF },
    { 0x65, 0x1F },
    { 0x7F, 0x14 },
    { 0x65, 0x67 },
    { 0x66, 0x08 },
    { 0x63, 0x70 },
    { 0x7F, 0x15 },
    { 0x48, 0x48 },
    { 0x7F, 0x07 },
    { 0x41, 0x0D },
    { 0x43, 0x14 },
    { 0x4B, 0x0E },
    { 0x45, 0x0F },
    { 0x44, 0x42 },
    { 0x4C, 0x80 },
    { 0x7F, 0x10 },
    { 0x5B, 0x02 },
    { 0x7F, 0x07 },
    { 0x40, 0x41 },
    { 0x70, 0x00 }
};

/* 第二阶段初始化寄存器配置表（LED_N启用、最终工作模式配置） */
static const pmw3901_reg_cfg_t pmw3901_init_stage2[] =
{
    { 0x32, 0x44 },
    { 0x7F, 0x07 },
    { 0x40, 0x40 },
    { 0x7F, 0x06 },
    { 0x62, 0xF0 },
    { 0x63, 0x00 },
    { 0x7F, 0x0D },
    { 0x48, 0xC0 },
    { 0x6F, 0xD5 },
    { 0x7F, 0x00 },
    { 0x5B, 0xA0 },
    { 0x4E, 0xA8 },
    { 0x5A, 0x50 },
    { 0x40, 0x80 },
    { 0x7F, 0x0E },
    { 0x72, 0x0F },
    { 0x7F, 0x00 }
};

/* PMW3901光流传感器原始数据，由PMW3901_Update_50HZ()周期性填充，已做极性映射 */
volatile pmw3901_raw_t g_pmw3901_raw = {0};
/* 初始化完成标志，1=已初始化，0=未初始化或初始化失败 */
static uint8 pmw3901_inited = 0U;

/**
 * @brief  SPI单字节收发
 *         同时发送一个字节并接收一个字节（全双工）
 *
 * @param  tx  要发送的字节
 * @return 接收到的字节
 */
static uint8 pmw3901_spi_transfer_byte(uint8 tx)
{
    uint8 rx = 0U;
    spi_transfer_8bit(PMW3901_SPI, &tx, &rx, 1U);
    return rx;
}

/**
 * @brief  写入PMW3901寄存器
 *         拉低CS→发送地址(置写标志)→等待tSRAD→发送数据→拉高CS→等待tSWx
 *
 * @param  reg    寄存器地址（0x00~0x7F）
 * @param  value  要写入的值
 * @return 无
 */
static void pmw3901_reg_write(uint8 reg, uint8 value)
{
    gpio_low(PMW3901_CS_Pin);
    pmw3901_spi_transfer_byte((uint8)(reg | PMW3901_WRITE_FLAG));
    system_delay_us(PMW3901_TSRAD_US);
    pmw3901_spi_transfer_byte(value);
    gpio_high(PMW3901_CS_Pin);
    system_delay_us(PMW3901_TSWX_US);
}

/**
 * @brief  读取PMW3901寄存器
 *         拉低CS→发送地址→等待tSR→读取数据→拉高CS→等待tSRx
 *
 * @param  reg  寄存器地址（0x00~0x7F）
 * @return 读取到的寄存器值
 */
static uint8 pmw3901_reg_read(uint8 reg)
{
    uint8 value;
    gpio_low(PMW3901_CS_Pin);
    pmw3901_spi_transfer_byte(reg);
    system_delay_us(PMW3901_TSR_US);
    value = pmw3901_spi_transfer_byte(PMW3901_DUMMY_BYTE);
    gpio_high(PMW3901_CS_Pin);
    system_delay_us(PMW3901_TSRX_US);
    return value;
}

/**
 * @brief  批量加载寄存器配置表
 *         遍历配置表逐项写入，开启校验时写后回读验证，失败则重试
 *
 * @param  table      寄存器配置表指针
 * @param  table_len  配置表元素数量
 * @return 0=加载完成
 */
static uint8 pmw3901_load_config(const pmw3901_reg_cfg_t *table, uint32 table_len)
{
    uint32 i;

    for (i = 0U; i < table_len; ++i)
    {
#if PMW3901_VERIFY_WRITES
        uint8 try_idx;
        uint8 matched = 0U;

        /* 写入后回读校验，失败则重试最多 REG_RETRY_MAX 次 */
        for (try_idx = 0U; try_idx < PMW3901_REG_RETRY_MAX; ++try_idx)
        {
            pmw3901_reg_write(table[i].reg, table[i].value);
            if (pmw3901_reg_read(table[i].reg) == table[i].value)
            {
                matched = 1U;
                break;
            }
        }

        if (matched == 0U)
        {
            continue;
        }
#else
        pmw3901_reg_write(table[i].reg, table[i].value);
#endif
    }

    return 0U;
}

/**
 * @brief  校验PMW3901芯片ID
 *         读取产品ID和反码ID，验证两者互为按位取反关系
 *
 * @param  无
 * @return 0=ID校验通过，1=校验失败
 */
static uint8 pmw3901_check_id(void)
{
    uint8 id1 = pmw3901_reg_read(PMW3901_REG_PRODUCT_ID);
    uint8 id2 = pmw3901_reg_read(PMW3901_REG_INV_PROD_ID2);

    if ((id1 == PMW3901_PRODUCT_ID_3901) && (id2 == (uint8)(~id1)))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief  复位光流原始数据和初始化标志
 *         将 g_pmw3901_raw 清零，并将 pmw3901_inited 置0
 *
 * @param  无
 * @return 无
 */
static void pmw3901_reset_raw_data(void)
{
    g_pmw3901_raw = (pmw3901_raw_t){0};
    pmw3901_inited = 0U;
}

/**
 * @brief  Motion Burst读取
 *         一次性读取全部运动数据（像素位移、质量、曝光等），
 *         并对大端序传输的shutter字段做字节交换
 *
 * @param  motion  输出指针，存放读取到的原始运动数据
 * @return 无
 */
static void pmw3901_motion_burst_read(pmw3901_raw_t *motion)
{
    uint8 idx;
    uint8 *raw = (uint8 *)motion;
    uint16 shutter_value;

    /* 拉低CS，发送Burst读取命令，等待数据就绪 */
    gpio_low(PMW3901_CS_Pin);
    pmw3901_spi_transfer_byte(PMW3901_REG_MOT_BURST2);
    system_delay_us(PMW3901_MOTION_BURST_DELAY_US);

    /* 逐字节读取整个Burst数据帧 */
    for (idx = 0U; idx < (uint8)sizeof(pmw3901_raw_t); ++idx)
    {
        raw[idx] = pmw3901_spi_transfer_byte(PMW3901_DUMMY_BYTE);
    }
    gpio_high(PMW3901_CS_Pin);
    system_delay_us(PMW3901_TBEXIT_US);

    /* shutter字段为大端序传输，需要字节交换转为小端序 */
    shutter_value = motion->shutter;
    motion->shutter = (uint16)(((shutter_value >> 8) & 0x00FFU)
                               | ((shutter_value & 0x00FFU) << 8));
}

/**
 * @brief  PMW3901光流传感器初始化
 *         执行SPI外设初始化、芯片上电复位、ID校验（带重试）、
 *         两阶段寄存器配置，并执行首次数据读取
 *
 * @param  无
 * @return 0=初始化成功，1=初始化失败（ID校验或寄存器配置失败）
 */
uint8 PMW3901_Init(void)
{
    uint8 retry_cnt = 0U;
    pmw3901_reset_raw_data();

    /* 初始化SPI外设和CS引脚 */
    spi_init(PMW3901_SPI,
             SPI_MODE3,
             PMW3901_SPI_SPEED,
             PMW3901_SCK_PIN,
             PMW3901_MOSI_PIN,
             PMW3901_MISO_PIN,
             SPI_CS_NULL);
    gpio_init(PMW3901_CS_Pin, GPO, 1, GPO_PUSH_PULL);
    system_delay_ms(10U);

    /* 发送上电复位命令，等待芯片就绪 */
    pmw3901_reg_write(PMW3901_REG_POWER_RST, PMW3901_POWER_ON_RESET_CMD);
    system_delay_ms(PMW3901_POWERUP_DELAY_MS);

    /* ID校验，失败则重新复位并重试，超过最大次数则返回失败 */
    while (pmw3901_check_id() != 0U)
    {
        retry_cnt++;
        if (retry_cnt >= PMW3901_REG_RETRY_MAX)
        {
            pmw3901_reset_raw_data();
            return 1U;
        }
        system_delay_ms(10U);
        pmw3901_reg_write(PMW3901_REG_POWER_RST, PMW3901_POWER_ON_RESET_CMD);
        system_delay_ms(PMW3901_POWERUP_DELAY_MS);
    }

    /* 加载第一阶段寄存器配置 */
    if (pmw3901_load_config(pmw3901_init_stage1,
                            (uint32)(sizeof(pmw3901_init_stage1) / sizeof(pmw3901_init_stage1[0]))) != 0U)
    {
        pmw3901_reset_raw_data();
        return 1U;
    }
    system_delay_ms(PMW3901_STAGE_GAP_DELAY_MS);

    /* 加载第二阶段寄存器配置 */
    if (pmw3901_load_config(pmw3901_init_stage2,
                            (uint32)(sizeof(pmw3901_init_stage2) / sizeof(pmw3901_init_stage2[0]))) != 0U)
    {
        pmw3901_reset_raw_data();
        return 1U;
    }
    system_delay_ms(PMW3901_READY_DELAY_MS);

    /* 标记初始化完成 */
    pmw3901_inited = 1U;

    /* 清除初始脏数据：Burst Read会同时清除motion和delta累加器，无需额外单寄存器读取 */
    {
        uint8 flush_i;
        pmw3901_raw_t discard = {0};
        for (flush_i = 0U; flush_i < PMW3901_INIT_FLUSH_COUNT; ++flush_i)
        {
            pmw3901_motion_burst_read(&discard);
            system_delay_ms(PMW3901_FLUSH_INTERVAL_MS);
        }
    }

    /* 执行首次有效数据读取 */
    PMW3901_Update_50HZ();
    return 0U;
}

/**
 * @brief  PMW3901重新初始化
 *         先清零原始数据和初始化标志，再调用PMW3901_Init()完成完整初始化
 *
 * @param  无
 * @return 0=重新初始化成功，1=失败
 */
uint8 PMW3901_ReInit(void)
{
    pmw3901_reset_raw_data();
    return PMW3901_Init();
}

/**
 * @brief  PMW3901数据更新（需周期性调用，典型50Hz）
 *         执行Burst Read读取全部运动数据，检查motionOccured标志位，
 *         仅当传感器报告有新运动时才更新deltaX/deltaY并施加极性映射，
 *         否则将deltaX/deltaY清零防止残留脏数据
 *
 * @param  无
 * @return 无
 */
void PMW3901_Update_50HZ(void)
{
    pmw3901_raw_t burst_data = {0};

    if (pmw3901_inited == 0U)
    {
        return;
    }

    pmw3901_motion_burst_read(&burst_data);

    /* 检查motionOccured标志：传感器未完成新帧处理时delta可能为脏数据 */
    if (burst_data.squal == 0U)
    {
        burst_data.deltaX = 0;
        burst_data.deltaY = 0;
    }

    /* 对像素位移施加极性映射，将芯片坐标系转换为机体坐标系 */
    burst_data.deltaX = (int16)(PMW3901_SIGN_X * burst_data.deltaX);
    burst_data.deltaY = (int16)(PMW3901_SIGN_Y * burst_data.deltaY);
    if (burst_data.deltaX < -100)
    {
        burst_data.deltaX = 0;
    }else if (burst_data.deltaX > 100)
    {
        burst_data.deltaX = 0;
    }
    if (burst_data.deltaY < -100)
    {
        burst_data.deltaY = 0;
    }else if (burst_data.deltaY > 100)
    {
        burst_data.deltaY = 0;
    }
    if (burst_data.squal < 10)
    {
        burst_data.deltaX = 0;
        burst_data.deltaY = 0;
    }
    if (burst_data.squal > 240)
    {
        burst_data.deltaX = 0;
        burst_data.deltaY = 0;
    }
    
    g_pmw3901_raw = burst_data;
}
