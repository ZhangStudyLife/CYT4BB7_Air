#include "PMW3901.h"

#define PMW3901_REG_PRODUCT_ID         (0x00U)
#define PMW3901_REG_POWER_RST          (0x3AU)
#define PMW3901_REG_INV_PROD_ID2       (0x5FU)
#define PMW3901_REG_MOT_BURST2         (0x16U)

#define PMW3901_PRODUCT_ID_3901        (0x49U)
#define PMW3901_POWER_ON_RESET_CMD     (0x5AU)
#define PMW3901_WRITE_FLAG             (0x80U)
#define PMW3901_DUMMY_BYTE             (0x00U)

#define PMW3901_POWERUP_DELAY_MS       (50U)
#define PMW3901_STAGE_GAP_DELAY_MS     (100U)
#define PMW3901_READY_DELAY_MS         (50U)
#define PMW3901_REG_RETRY_MAX          (5U)

#define PMW3901_TSRAD_US               (300U)
#define PMW3901_TSR_US                 (500U)
#define PMW3901_TSWX_US                (120U)
#define PMW3901_TSRX_US                (200U)
#define PMW3901_TBEXIT_US              (1U)
#define PMW3901_MOTION_BURST_DELAY_US  (150U)

typedef struct
{
    uint8 reg;
    uint8 value;
} pmw3901_reg_cfg_t;

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

volatile pmw3901_raw_t g_pmw3901_raw = {0};
static uint8 pmw3901_inited = 0U;

static uint8 pmw3901_spi_transfer_byte(uint8 tx)
{
    uint8 rx = 0U;
    spi_transfer_8bit(PMW3901_SPI, &tx, &rx, 1U);
    return rx;
}

static void pmw3901_reg_write(uint8 reg, uint8 value)
{
    gpio_low(PMW3901_CS_Pin);
    pmw3901_spi_transfer_byte((uint8)(reg | PMW3901_WRITE_FLAG));
    system_delay_us(PMW3901_TSRAD_US);
    pmw3901_spi_transfer_byte(value);
    gpio_high(PMW3901_CS_Pin);
    system_delay_us(PMW3901_TSWX_US);
}

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

static uint8 pmw3901_load_config(const pmw3901_reg_cfg_t *table, uint32 table_len)
{
    uint32 i;

    for (i = 0U; i < table_len; ++i)
    {
#if PMW3901_VERIFY_WRITES
        uint8 try_idx;
        uint8 matched = 0U;

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

static void pmw3901_reset_raw_data(void)
{
    g_pmw3901_raw = (pmw3901_raw_t){0};
    pmw3901_inited = 0U;
}

static void pmw3901_motion_burst_read(pmw3901_raw_t *motion)
{
    uint8 idx;
    uint8 *raw = (uint8 *)motion;
    uint16 shutter_value;

    gpio_low(PMW3901_CS_Pin);
    pmw3901_spi_transfer_byte(PMW3901_REG_MOT_BURST2);
    system_delay_us(PMW3901_MOTION_BURST_DELAY_US);
    for (idx = 0U; idx < (uint8)sizeof(pmw3901_raw_t); ++idx)
    {
        raw[idx] = pmw3901_spi_transfer_byte(PMW3901_DUMMY_BYTE);
    }
    gpio_high(PMW3901_CS_Pin);
    system_delay_us(PMW3901_TBEXIT_US);

    shutter_value = motion->shutter;
    motion->shutter = (uint16)(((shutter_value >> 8) & 0x00FFU)
                               | ((shutter_value & 0x00FFU) << 8));
}

uint8 PMW3901_Init(void)
{
    uint8 retry_cnt = 0U;
    pmw3901_reset_raw_data();

    spi_init(PMW3901_SPI,
             SPI_MODE3,
             PMW3901_SPI_SPEED,
             PMW3901_SCK_PIN,
             PMW3901_MOSI_PIN,
             PMW3901_MISO_PIN,
             SPI_CS_NULL);
    gpio_init(PMW3901_CS_Pin, GPO, 1, GPO_PUSH_PULL);
    system_delay_ms(10U);

    pmw3901_reg_write(PMW3901_REG_POWER_RST, PMW3901_POWER_ON_RESET_CMD);
    system_delay_ms(PMW3901_POWERUP_DELAY_MS);

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

    if (pmw3901_load_config(pmw3901_init_stage1,
                            (uint32)(sizeof(pmw3901_init_stage1) / sizeof(pmw3901_init_stage1[0]))) != 0U)
    {
        pmw3901_reset_raw_data();
        return 1U;
    }
    system_delay_ms(PMW3901_STAGE_GAP_DELAY_MS);

    if (pmw3901_load_config(pmw3901_init_stage2,
                            (uint32)(sizeof(pmw3901_init_stage2) / sizeof(pmw3901_init_stage2[0]))) != 0U)
    {
        pmw3901_reset_raw_data();
        return 1U;
    }
    system_delay_ms(PMW3901_READY_DELAY_MS);

    pmw3901_inited = 1U;
    PMW3901_Update();
    return 0U;
}

uint8 PMW3901_ReInit(void)
{
    pmw3901_reset_raw_data();
    return PMW3901_Init();
}

void PMW3901_Update(void)
{
    pmw3901_raw_t burst_data = {0};

    if (pmw3901_inited == 0U)
    {
        return;
    }

    pmw3901_motion_burst_read(&burst_data);
    burst_data.deltaX = (int16)(PMW3901_SIGN_X * burst_data.deltaX);
    burst_data.deltaY = (int16)(PMW3901_SIGN_Y * burst_data.deltaY);
    g_pmw3901_raw = burst_data;
}
