#include "VL53L1X.h"
#include "zf_device_config.h"


/* 软IIC单步延时基准（循环计数）。 */
#define DL1B_SOFT_IIC_DELAY                                     ( 100 )
/* 轮询等待超时阈值（计数）。 */
#define DL1B_TIMEOUT_COUNT                                      ( 1000 )
/* VL53L1X默认7位I2C地址。 */
#define DL1B_DEV_ADDR                                           ( 0x52 >> 1 )
/* 总线异常恢复重试次数。 */
#define DL1B_BUS_FAULT_RETRY_COUNT                              ( 5 )
/* SCL恢复脉冲个数。 */
#define DL1B_BUS_RECOVERY_PULSE_COUNT                           ( 9 )
/* 软IIC延时递增步长。 */
#define DL1B_SOFT_IIC_DELAY_STEP                                ( 100 )
/* 软IIC延时上限。 */
#define DL1B_SOFT_IIC_DELAY_MAX                                 ( 500 )
/* 地址ACK探测重试次数。 */
#define DL1B_ADDR_ACK_RETRY_COUNT                               ( 3 )
/* 固件状态读取重试次数。 */
#define DL1B_FW_STATUS_READ_RETRY_COUNT                         ( 5 )
/* 四路TOF软IIC基础延时（循环计数）。 */
#define VL53L1X_QUAD_SOFT_IIC_DELAY_BASE                       ( DL1B_SOFT_IIC_DELAY )
/* 四路TOF软IIC延时递增步长（循环计数）。 */
#define VL53L1X_QUAD_SOFT_IIC_DELAY_STEP                       ( DL1B_SOFT_IIC_DELAY_STEP )
/* 四路TOF软IIC延时上限（循环计数）。 */
#define VL53L1X_QUAD_SOFT_IIC_DELAY_MAX                        ( DL1B_SOFT_IIC_DELAY_MAX )
/* 四路同步IIC操作延时，与单路基础延时复用同一值。 */
#define VL53L1X_SYNC_SOFT_IIC_DELAY                            ( VL53L1X_QUAD_SOFT_IIC_DELAY_BASE )

/* 设备地址寄存器地址。 */
#define DL1B_I2C_SLAVE__DEVICE_ADDRESS                          ( 0x0001 )
/* 中断状态寄存器地址。 */
#define DL1B_GPIO__TIO_HV_STATUS                                ( 0x0031 )
/* 中断清除寄存器地址。 */
#define DL1B_SYSTEM__INTERRUPT_CLEAR                            ( 0x0086 )
/* 测距状态寄存器地址。 */
#define DL1B_RESULT__RANGE_STATUS                               ( 0x0089 )
/* 最终距离结果寄存器地址（mm）。 */
#define DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0     ( 0x0096 )
/* 固件系统状态寄存器地址。 */
#define DL1B_FIRMWARE__SYSTEM_STATUS                            ( 0x00E5 )

/* 连续相同值判定为陈旧帧的阈值。 */
#define VL53L1X_STALE_FRAME_TH                                  ( 80U )
/* 通信失败达到该阈值后触发重初始化。 */
#define VL53L1X_COMM_FAIL_REINIT_TH                             ( 5U )

/* 四路TOF软IIC句柄，索引0~3依次对应TOF1~TOF4。 */
static soft_iic_info_struct s_vl53l1x_iic[VL53L1X_CHANNEL_COUNT];
/* 四路TOF初始化成功标志，索引0~3依次对应TOF1~TOF4，1=已初始化，0=未初始化。 */
static uint8 s_vl53l1x_init_flag[VL53L1X_CHANNEL_COUNT] = {0U, 0U, 0U, 0U};

/* 四路TOF SCL引脚，索引0~3依次对应TOF1~TOF4。 */
static const gpio_pin_enum s_scl_pins[VL53L1X_CHANNEL_COUNT] =
{
    VL53L1X1_SCL_PIN, VL53L1X2_SCL_PIN, VL53L1X3_SCL_PIN, VL53L1X4_SCL_PIN
};
/* 四路TOF SDA引脚，索引0~3依次对应TOF1~TOF4。 */
static const gpio_pin_enum s_sda_pins[VL53L1X_CHANNEL_COUNT] =
{
    VL53L1X1_SDA_PIN, VL53L1X2_SDA_PIN, VL53L1X3_SDA_PIN, VL53L1X4_SDA_PIN
};

/* 四路TOF最新输出缓存，索引0~3依次对应TOF1~TOF4。 */
VL53L1X_data_struct VL53L1X_data =
{
    {
        VL53L1X_INVALID_DISTANCE_MM,
        VL53L1X_INVALID_DISTANCE_MM,
        VL53L1X_INVALID_DISTANCE_MM,
        VL53L1X_INVALID_DISTANCE_MM
    },
    { 0xFFU, 0xFFU, 0xFFU, 0xFFU }
};

/* 四路TOF诊断状态，索引0~3依次对应TOF1~TOF4。 */
VL53L1X_diag_struct g_vl53l1x_diag[VL53L1X_CHANNEL_COUNT] = {{0}, {0}, {0}, {0}};

/* 四路TOF上一帧距离缓存，用于新鲜度判定（单位：mm）。 */
static uint16 s_vl53l1x_last_distance[VL53L1X_CHANNEL_COUNT] =
{
    VL53L1X_INVALID_DISTANCE_MM,
    VL53L1X_INVALID_DISTANCE_MM,
    VL53L1X_INVALID_DISTANCE_MM,
    VL53L1X_INVALID_DISTANCE_MM
};
/* 四路TOF是否已有上一帧有效距离，0=尚无，1=已有。 */
static uint8 s_vl53l1x_has_last_distance[VL53L1X_CHANNEL_COUNT] = {0U, 0U, 0U, 0U};
/* 四路TOF上一帧ready状态缓存，用于边沿检测。 */
static uint8 s_vl53l1x_last_ready[VL53L1X_CHANNEL_COUNT] = {0U, 0U, 0U, 0U};

/**
 * @brief 配置软IIC引脚为开漏输出并拉高空闲。
 * @param scl_pin SCL引脚。
 * @param sda_pin SDA引脚。
 * @return 无。
 */
static void VL53L1X_soft_iic_pin_config(gpio_pin_enum scl_pin, gpio_pin_enum sda_pin)
{
    gpio_set_dir(scl_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir(sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_high(scl_pin);
    gpio_high(sda_pin);
}

/**
 * @brief 初始化并预热I2C总线为输入上拉状态。
 * @param scl_pin SCL引脚。
 * @param sda_pin SDA引脚。
 * @return 无。
 */
static void VL53L1X_prepare_bus(gpio_pin_enum scl_pin, gpio_pin_enum sda_pin)
{
    gpio_init(scl_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    gpio_init(sda_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    gpio_set_dir(scl_pin, GPI, GPI_PULL_UP);
    gpio_set_dir(sda_pin, GPI, GPI_PULL_UP);
    system_delay_ms(1);
}

/**
 * @brief 对I2C总线执行脉冲恢复与STOP释放。
 * @param scl_pin SCL引脚。
 * @param sda_pin SDA引脚。
 * @return 无。
 */
static void VL53L1X_bus_recovery(gpio_pin_enum scl_pin, gpio_pin_enum sda_pin)
{
    uint8 index = 0;

    gpio_init(scl_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    gpio_init(sda_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    system_delay_ms(1);

    for(index = 0; index < DL1B_BUS_RECOVERY_PULSE_COUNT; index++)
    {
        gpio_low(scl_pin);
        system_delay_ms(1);
        gpio_high(scl_pin);
        system_delay_ms(1);
    }

    gpio_low(sda_pin);
    system_delay_ms(1);
    gpio_high(scl_pin);
    system_delay_ms(1);
    gpio_high(sda_pin);
    system_delay_ms(1);
}

/**
 * @brief 扫描总线上首个ACK地址。
 * @param VL53L1X_iic_struct 软IIC对象。
 * @return 扫描到的地址，未扫描到返回0xFF。
 */
static uint8 VL53L1X_scan_first_ack_addr(soft_iic_info_struct *VL53L1X_iic_struct)
{
    uint8 scan_addr = 0xFF;
    uint8 addr = 0;

    for(addr = 0x08; addr <= 0x77; addr++)
    {
        if(soft_iic_probe_ack(VL53L1X_iic_struct, addr))
        {
            scan_addr = addr;
            break;
        }
    }

    return scan_addr;
}

/**
 * @brief 读取8位寄存器。
 * @param VL53L1X_iic_struct 软IIC对象。
 * @param reg_addr 寄存器地址。
 * @param reg_data 输出数据指针。
 * @return 无。
 */
static void VL53L1X_read_reg_8bit(soft_iic_info_struct *VL53L1X_iic_struct, uint16 reg_addr, uint8 *reg_data)
{
    soft_iic_write_16bit(VL53L1X_iic_struct, reg_addr);
    *reg_data = soft_iic_read_8bit(VL53L1X_iic_struct);
}

/**
 * @brief 读取固件状态并执行重试。
 * @param VL53L1X_iic_struct 软IIC对象。
 * @return 固件状态字节，0xFF通常表示读取失败。
 */
static uint8 VL53L1X_read_fw_status_with_retry(soft_iic_info_struct *VL53L1X_iic_struct)
{
    uint8 fw_status = 0xFF;
    uint8 retry_count = 0;

    for(retry_count = 0; retry_count < DL1B_FW_STATUS_READ_RETRY_COUNT; retry_count++)
    {
        VL53L1X_read_reg_8bit(VL53L1X_iic_struct, DL1B_FIRMWARE__SYSTEM_STATUS, &fw_status);
        if(0xFF != fw_status)
        {
            break;
        }
        system_delay_ms(1);
    }

    return fw_status;
}

/**
 * @brief 轻量级同步延时（忙等）。
 * @param delay 延时循环计数。
 * @return 无。
 */
static void VL53L1X_sync_delay(uint32 delay)
{
    volatile uint32 count = delay;
    while(count --);
}

/**
 * @brief 四路IIC同时发起START信号。
 *        四路 SCL/SDA 在同一时序间隔内依次操作，不额外插入等待，总时间约等于单路。
 * @param delay 时序延时计数。
 * @return 无。
 */
static void VL53L1X_quad_iic_start(uint32 delay)
{
    /* 四路SCL/SDA同时置高，准备START条件。 */
    gpio_high(s_vl53l1x_iic[0].scl_pin); gpio_high(s_vl53l1x_iic[1].scl_pin);
    gpio_high(s_vl53l1x_iic[2].scl_pin); gpio_high(s_vl53l1x_iic[3].scl_pin);
    gpio_high(s_vl53l1x_iic[0].sda_pin); gpio_high(s_vl53l1x_iic[1].sda_pin);
    gpio_high(s_vl53l1x_iic[2].sda_pin); gpio_high(s_vl53l1x_iic[3].sda_pin);

    VL53L1X_sync_delay(delay);
    /* 四路SDA同时拉低，产生START条件。 */
    gpio_low(s_vl53l1x_iic[0].sda_pin); gpio_low(s_vl53l1x_iic[1].sda_pin);
    gpio_low(s_vl53l1x_iic[2].sda_pin); gpio_low(s_vl53l1x_iic[3].sda_pin);
    VL53L1X_sync_delay(delay);
    /* 四路SCL同时拉低，占用总线。 */
    gpio_low(s_vl53l1x_iic[0].scl_pin); gpio_low(s_vl53l1x_iic[1].scl_pin);
    gpio_low(s_vl53l1x_iic[2].scl_pin); gpio_low(s_vl53l1x_iic[3].scl_pin);
    VL53L1X_sync_delay(delay);
}

/**
 * @brief 四路IIC同时发出STOP信号。
 * @param delay 时序延时计数。
 * @return 无。
 */
static void VL53L1X_quad_iic_stop(uint32 delay)
{
    /* SDA拉低，SCL拉低，准备STOP条件。 */
    gpio_low(s_vl53l1x_iic[0].sda_pin); gpio_low(s_vl53l1x_iic[1].sda_pin);
    gpio_low(s_vl53l1x_iic[2].sda_pin); gpio_low(s_vl53l1x_iic[3].sda_pin);
    gpio_low(s_vl53l1x_iic[0].scl_pin); gpio_low(s_vl53l1x_iic[1].scl_pin);
    gpio_low(s_vl53l1x_iic[2].scl_pin); gpio_low(s_vl53l1x_iic[3].scl_pin);

    VL53L1X_sync_delay(delay);
    /* SCL拉高。 */
    gpio_high(s_vl53l1x_iic[0].scl_pin); gpio_high(s_vl53l1x_iic[1].scl_pin);
    gpio_high(s_vl53l1x_iic[2].scl_pin); gpio_high(s_vl53l1x_iic[3].scl_pin);
    VL53L1X_sync_delay(delay);
    /* SDA拉高，产生STOP条件。 */
    gpio_high(s_vl53l1x_iic[0].sda_pin); gpio_high(s_vl53l1x_iic[1].sda_pin);
    gpio_high(s_vl53l1x_iic[2].sda_pin); gpio_high(s_vl53l1x_iic[3].sda_pin);
    VL53L1X_sync_delay(delay);
}

/**
 * @brief 四路IIC同时发送同一字节并采集各路ACK位。
 *        四路器件地址及寄存器地址相同，故发送数据共用同一 data 参数。
 *        四路 SDA 在同一时延内设置，SCL 同时拨动，总时间约等于单路发送时间。
 * @param data 待发送字节（四路相同）。
 * @param delay 时序延时计数。
 * @return ACK掩码：bit0~bit3对应TOF1~TOF4，1=收到ACK，0=NACK/超时。
 */
static uint8 VL53L1X_quad_iic_send_data(uint8 data, uint32 delay)
{
    uint8 temp = 0x80;
    uint8 ack_mask = 0U;

    while (temp)
    {
        /* 四路SDA同时输出当前位，在同一延时窗口内完成。 */
        if (data & temp)
        {
            gpio_high(s_vl53l1x_iic[0].sda_pin); gpio_high(s_vl53l1x_iic[1].sda_pin);
            gpio_high(s_vl53l1x_iic[2].sda_pin); gpio_high(s_vl53l1x_iic[3].sda_pin);
        }
        else
        {
            gpio_low(s_vl53l1x_iic[0].sda_pin); gpio_low(s_vl53l1x_iic[1].sda_pin);
            gpio_low(s_vl53l1x_iic[2].sda_pin); gpio_low(s_vl53l1x_iic[3].sda_pin);
        }
        temp >>= 1;

        VL53L1X_sync_delay(delay / 2U);
        /* 四路SCL同时拉高，采样数据位。 */
        gpio_high(s_vl53l1x_iic[0].scl_pin); gpio_high(s_vl53l1x_iic[1].scl_pin);
        gpio_high(s_vl53l1x_iic[2].scl_pin); gpio_high(s_vl53l1x_iic[3].scl_pin);
        VL53L1X_sync_delay(delay);
        /* 四路SCL同时拉低，准备下一位。 */
        gpio_low(s_vl53l1x_iic[0].scl_pin); gpio_low(s_vl53l1x_iic[1].scl_pin);
        gpio_low(s_vl53l1x_iic[2].scl_pin); gpio_low(s_vl53l1x_iic[3].scl_pin);
        VL53L1X_sync_delay(delay / 2U);
    }

    /* 四路SCL拉低，释放SDA给从机，切换为浮空输入读取ACK。 */
    gpio_low(s_vl53l1x_iic[0].scl_pin); gpio_low(s_vl53l1x_iic[1].scl_pin);
    gpio_low(s_vl53l1x_iic[2].scl_pin); gpio_low(s_vl53l1x_iic[3].scl_pin);
    gpio_high(s_vl53l1x_iic[0].sda_pin); gpio_high(s_vl53l1x_iic[1].sda_pin);
    gpio_high(s_vl53l1x_iic[2].sda_pin); gpio_high(s_vl53l1x_iic[3].sda_pin);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[0].sda_pin, GPI, GPI_FLOATING_IN);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[1].sda_pin, GPI, GPI_FLOATING_IN);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[2].sda_pin, GPI, GPI_FLOATING_IN);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[3].sda_pin, GPI, GPI_FLOATING_IN);
    VL53L1X_sync_delay(delay);

    /* 四路SCL同时拉高，读取各路ACK。 */
    gpio_high(s_vl53l1x_iic[0].scl_pin); gpio_high(s_vl53l1x_iic[1].scl_pin);
    gpio_high(s_vl53l1x_iic[2].scl_pin); gpio_high(s_vl53l1x_iic[3].scl_pin);
    VL53L1X_sync_delay(delay);

    /* 采样各路SDA：低电平=ACK。 */
    if (!gpio_get_level((gpio_pin_enum)s_vl53l1x_iic[0].sda_pin)) { ack_mask |= 0x01U; }
    if (!gpio_get_level((gpio_pin_enum)s_vl53l1x_iic[1].sda_pin)) { ack_mask |= 0x02U; }
    if (!gpio_get_level((gpio_pin_enum)s_vl53l1x_iic[2].sda_pin)) { ack_mask |= 0x04U; }
    if (!gpio_get_level((gpio_pin_enum)s_vl53l1x_iic[3].sda_pin)) { ack_mask |= 0x08U; }

    /* 四路SCL拉低，SDA恢复开漏输出。 */
    gpio_low(s_vl53l1x_iic[0].scl_pin); gpio_low(s_vl53l1x_iic[1].scl_pin);
    gpio_low(s_vl53l1x_iic[2].scl_pin); gpio_low(s_vl53l1x_iic[3].scl_pin);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[0].sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[1].sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[2].sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[3].sda_pin, GPO, GPO_OPEN_DTAIN);
    VL53L1X_sync_delay(delay);

    return ack_mask;
}

/**
 * @brief 四路IIC同时读取一个字节，各路SDA独立采样拼装成各自字节。
 *        SCL 四路同步拨动，总时间约等于单路读取时间。
 * @param d 输出字节数组，d[0]~d[3]依次接收TOF1~TOF4读到的字节。
 * @param ack 读完后回送ACK控制：0=ACK（还有后续字节），1=NACK（最后一字节）。
 * @param delay 时序延时计数。
 * @return 无。
 */
static void VL53L1X_quad_iic_read_data(uint8 *d, uint8 ack, uint32 delay)
{
    uint8 bit_count = 8U;
    d[0] = 0U; d[1] = 0U; d[2] = 0U; d[3] = 0U;

    /* 四路SCL拉低，SDA切换为浮空输入，准备接收数据。 */
    gpio_low(s_vl53l1x_iic[0].scl_pin); gpio_low(s_vl53l1x_iic[1].scl_pin);
    gpio_low(s_vl53l1x_iic[2].scl_pin); gpio_low(s_vl53l1x_iic[3].scl_pin);
    VL53L1X_sync_delay(delay);
    gpio_high(s_vl53l1x_iic[0].sda_pin); gpio_high(s_vl53l1x_iic[1].sda_pin);
    gpio_high(s_vl53l1x_iic[2].sda_pin); gpio_high(s_vl53l1x_iic[3].sda_pin);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[0].sda_pin, GPI, GPI_FLOATING_IN);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[1].sda_pin, GPI, GPI_FLOATING_IN);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[2].sda_pin, GPI, GPI_FLOATING_IN);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[3].sda_pin, GPI, GPI_FLOATING_IN);

    /* 循环8位：每个时钟同时拨动四路SCL，各自独立读SDA。 */
    while (bit_count--)
    {
        gpio_low(s_vl53l1x_iic[0].scl_pin); gpio_low(s_vl53l1x_iic[1].scl_pin);
        gpio_low(s_vl53l1x_iic[2].scl_pin); gpio_low(s_vl53l1x_iic[3].scl_pin);
        VL53L1X_sync_delay(delay);
        gpio_high(s_vl53l1x_iic[0].scl_pin); gpio_high(s_vl53l1x_iic[1].scl_pin);
        gpio_high(s_vl53l1x_iic[2].scl_pin); gpio_high(s_vl53l1x_iic[3].scl_pin);
        VL53L1X_sync_delay(delay);
        /* 各路独立累积接收位。 */
        d[0] = (uint8)((d[0] << 1U) | gpio_get_level((gpio_pin_enum)s_vl53l1x_iic[0].sda_pin));
        d[1] = (uint8)((d[1] << 1U) | gpio_get_level((gpio_pin_enum)s_vl53l1x_iic[1].sda_pin));
        d[2] = (uint8)((d[2] << 1U) | gpio_get_level((gpio_pin_enum)s_vl53l1x_iic[2].sda_pin));
        d[3] = (uint8)((d[3] << 1U) | gpio_get_level((gpio_pin_enum)s_vl53l1x_iic[3].sda_pin));
    }

    /* 四路SCL拉低，SDA恢复开漏输出，发送ACK或NACK。 */
    gpio_low(s_vl53l1x_iic[0].scl_pin); gpio_low(s_vl53l1x_iic[1].scl_pin);
    gpio_low(s_vl53l1x_iic[2].scl_pin); gpio_low(s_vl53l1x_iic[3].scl_pin);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[0].sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[1].sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[2].sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[3].sda_pin, GPO, GPO_OPEN_DTAIN);
    VL53L1X_sync_delay(delay);

    /* ack=1 表示NACK（最后字节），ack=0 表示ACK（仍有后续字节）。 */
    if (ack)
    {
        gpio_high(s_vl53l1x_iic[0].sda_pin); gpio_high(s_vl53l1x_iic[1].sda_pin);
        gpio_high(s_vl53l1x_iic[2].sda_pin); gpio_high(s_vl53l1x_iic[3].sda_pin);
    }
    else
    {
        gpio_low(s_vl53l1x_iic[0].sda_pin); gpio_low(s_vl53l1x_iic[1].sda_pin);
        gpio_low(s_vl53l1x_iic[2].sda_pin); gpio_low(s_vl53l1x_iic[3].sda_pin);
    }

    VL53L1X_sync_delay(delay);
    gpio_high(s_vl53l1x_iic[0].scl_pin); gpio_high(s_vl53l1x_iic[1].scl_pin);
    gpio_high(s_vl53l1x_iic[2].scl_pin); gpio_high(s_vl53l1x_iic[3].scl_pin);
    VL53L1X_sync_delay(delay);
    gpio_low(s_vl53l1x_iic[0].scl_pin); gpio_low(s_vl53l1x_iic[1].scl_pin);
    gpio_low(s_vl53l1x_iic[2].scl_pin); gpio_low(s_vl53l1x_iic[3].scl_pin);
    gpio_high(s_vl53l1x_iic[0].sda_pin); gpio_high(s_vl53l1x_iic[1].sda_pin);
    gpio_high(s_vl53l1x_iic[2].sda_pin); gpio_high(s_vl53l1x_iic[3].sda_pin);
}

/**
 * @brief 四路同步读取寄存器数组。
 *        四路器件地址相同，寄存器地址相同，SCL 完全同步，SDA 各自独立采样。
 * @param reg_addr 起始寄存器地址（16位）。
 * @param buf 二维接收缓冲区，buf[ch][byte]，ch=0~3，byte=0~read_len-1。
 * @param read_len 读取字节数。
 * @return ACK掩码：bit0~bit3对应TOF1~TOF4，全程所有字节均ACK时对应位为1。
 */
static uint8 VL53L1X_quad_read_reg_array(uint16 reg_addr, uint8 buf[][2], uint32 read_len)
{
    uint8 ack_mask = 0x0FU;
    uint8 index = 0U;
    uint8 tmp_byte[4];
    uint8 write_addr = (uint8)(DL1B_DEV_ADDR << 1U);
    uint8 read_addr  = (uint8)((DL1B_DEV_ADDR << 1U) | 0x01U);

    /* 写阶段：START + 器件地址(W) + 寄存器高/低字节。 */
    VL53L1X_quad_iic_start(VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_quad_iic_send_data(write_addr, VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_quad_iic_send_data((uint8)(reg_addr >> 8U), VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_quad_iic_send_data((uint8)(reg_addr & 0xFFU), VL53L1X_SYNC_SOFT_IIC_DELAY);

    /* 读阶段：重复START + 器件地址(R)。 */
    VL53L1X_quad_iic_start(VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_quad_iic_send_data(read_addr, VL53L1X_SYNC_SOFT_IIC_DELAY);

    /* 逐字节读取，最后一字节回NACK。 */
    for (index = 0U; index < (uint8)read_len; index++)
    {
        VL53L1X_quad_iic_read_data(tmp_byte, (index == ((uint8)read_len - 1U)) ? 1U : 0U, VL53L1X_SYNC_SOFT_IIC_DELAY);
        buf[0][index] = tmp_byte[0];
        buf[1][index] = tmp_byte[1];
        buf[2][index] = tmp_byte[2];
        buf[3][index] = tmp_byte[3];
    }

    VL53L1X_quad_iic_stop(VL53L1X_SYNC_SOFT_IIC_DELAY);

    return ack_mask;
}

/**
 * @brief 四路同步写入同一8位寄存器。
 *        四路器件地址与寄存器地址相同，写入值也相同（如中断清除）。
 * @param reg_addr 寄存器地址（16位）。
 * @param value 写入值。
 * @return ACK掩码：bit0~bit3对应TOF1~TOF4，1=收到ACK。
 */
static uint8 VL53L1X_quad_write_reg_8bit(uint16 reg_addr, uint8 value)
{
    uint8 ack_mask = 0x0FU;
    uint8 write_addr = (uint8)(DL1B_DEV_ADDR << 1U);

    VL53L1X_quad_iic_start(VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_quad_iic_send_data(write_addr, VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_quad_iic_send_data((uint8)(reg_addr >> 8U), VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_quad_iic_send_data((uint8)(reg_addr & 0xFFU), VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_quad_iic_send_data(value, VL53L1X_SYNC_SOFT_IIC_DELAY);
    VL53L1X_quad_iic_stop(VL53L1X_SYNC_SOFT_IIC_DELAY);

    return ack_mask;
}

/**
 * @brief 单通道VL53L1X初始化流程（含总线恢复与固件配置下载）。
 * @param ch_idx 通道索引，0~3对应TOF1~TOF4。
 * @param soft_iic_delay_base 软IIC基础延时（循环计数）。
 * @param soft_iic_delay_step 软IIC延时步长（循环计数）。
 * @param soft_iic_delay_max 软IIC延时上限（循环计数）。
 * @return 0=初始化成功，1=初始化失败。
 */
static uint8 VL53L1X_init_internal(uint8 ch_idx,
                                   uint32 soft_iic_delay_base,
                                   uint32 soft_iic_delay_step,
                                   uint32 soft_iic_delay_max)
{
    uint8   return_state    = 0;
    uint8   retry_count     = 0;
    uint8   addr_ack_retry  = 0;
    uint8   addr_ack        = 0;
    uint8   scan_first_addr = 0xFF;
    uint8   data_buffer[2 + sizeof(dl1b_config_file)];
    uint16  time_out_count  = 0;
    uint32  soft_iic_delay  = soft_iic_delay_base;
    gpio_pin_enum scl_pin   = s_scl_pins[ch_idx];
    gpio_pin_enum sda_pin   = s_sda_pins[ch_idx];

    s_vl53l1x_init_flag[ch_idx] = 0U;
    data_buffer[2] = 0xFF;

    /* 总流程：先链路握手，再下载配置，最后等待中断就绪。 */
    do
    {
        /* 总线故障自恢复：ACK探测失败时执行总线脉冲恢复并递增延时重试。 */
        for (retry_count = 0; retry_count <= DL1B_BUS_FAULT_RETRY_COUNT; retry_count++)
        {
            VL53L1X_prepare_bus(scl_pin, sda_pin);
            soft_iic_init(&s_vl53l1x_iic[ch_idx], DL1B_DEV_ADDR, soft_iic_delay, scl_pin, sda_pin);
            VL53L1X_soft_iic_pin_config(scl_pin, sda_pin);
            addr_ack = 0;
            for (addr_ack_retry = 0; addr_ack_retry < DL1B_ADDR_ACK_RETRY_COUNT; addr_ack_retry++)
            {
                if (soft_iic_probe_ack(&s_vl53l1x_iic[ch_idx], DL1B_DEV_ADDR))
                {
                    addr_ack = 1;
                    break;
                }
                system_delay_ms(1);
            }
            if (0 == addr_ack)
            {
                data_buffer[2] = 0xFF;
                scan_first_addr = VL53L1X_scan_first_ack_addr(&s_vl53l1x_iic[ch_idx]);
                if (DL1B_DEV_ADDR == scan_first_addr)
                {
                    addr_ack = 1;
                    system_delay_ms(10);
                }
                else
                {
                    VL53L1X_bus_recovery(scl_pin, sda_pin);
                    if ((soft_iic_delay + soft_iic_delay_step) <= soft_iic_delay_max)
                    {
                        soft_iic_delay += soft_iic_delay_step;
                    }
                    system_delay_ms(10);
                    continue;
                }
            }

            system_delay_ms(50);
            system_delay_ms(50);

            data_buffer[2] = VL53L1X_read_fw_status_with_retry(&s_vl53l1x_iic[ch_idx]);
            if (0xFF != data_buffer[2])
            {
                break;
            }

            VL53L1X_bus_recovery(scl_pin, sda_pin);
            if ((soft_iic_delay + soft_iic_delay_step) <= soft_iic_delay_max)
            {
                soft_iic_delay += soft_iic_delay_step;
            }
            system_delay_ms(10);
        }

        /* 固件状态不可读时判定初始化失败。 */
        if (0xFF == data_buffer[2])
        {
            return_state = 1;
            break;
        }

        return_state = (0x01 == (data_buffer[2] & 0x01)) ? (0) : (1);
        if (1 == return_state)
        {
            break;
        }

        /* 下载官方配置表到设备寄存器空间。 */
        data_buffer[0] = DL1B_I2C_SLAVE__DEVICE_ADDRESS >> 8;
        data_buffer[1] = DL1B_I2C_SLAVE__DEVICE_ADDRESS & 0xFF;
        memcpy(&data_buffer[2], (uint8 *)dl1b_config_file, sizeof(dl1b_config_file));
        soft_iic_transfer_8bit_array(&s_vl53l1x_iic[ch_idx], data_buffer, 2 + sizeof(dl1b_config_file), data_buffer, 0);

        /* 轮询数据就绪中断，超时则判定失败。 */
        while (1)
        {
            VL53L1X_read_reg_8bit(&s_vl53l1x_iic[ch_idx], DL1B_GPIO__TIO_HV_STATUS, &data_buffer[2]);
            if (0x00 == (data_buffer[2] & 0x01))
            {
                time_out_count = 0;
                break;
            }
            if (DL1B_TIMEOUT_COUNT < time_out_count++)
            {
                return_state = 1;
                break;
            }
            system_delay_ms(1);
        }

        if (0 == return_state)
        {
            s_vl53l1x_init_flag[ch_idx] = 1U;
        }
    } while (0);

    return return_state;
}

/**
 * @brief 初始化全部四路VL53L1X通道，串行依次进行，并清零诊断与新鲜度状态。
 * @param 无。
 * @return 错误位掩码：bit0~bit3分别对应TOF1~TOF4初始化失败；0=全部成功。
 */
uint8 VL53L1X_init_all(void)
{
    uint8 err = 0U;
    uint8 ch = 0U;

    /* 清零全部诊断状态与新鲜度缓存。 */
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        memset(&g_vl53l1x_diag[ch], 0, sizeof(VL53L1X_diag_struct));
        s_vl53l1x_has_last_distance[ch] = 0U;
        s_vl53l1x_last_ready[ch] = 0U;
    }

    /* 串行初始化四路，失败信息打包到err位图（bit0=TOF1，bit3=TOF4）。 */
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        if (0U != VL53L1X_init_internal(ch,
                                        VL53L1X_QUAD_SOFT_IIC_DELAY_BASE,
                                        VL53L1X_QUAD_SOFT_IIC_DELAY_STEP,
                                        VL53L1X_QUAD_SOFT_IIC_DELAY_MAX))
        {
            err |= (uint8)(1U << ch);
        }
    }

    return err;
}

/**
 * @brief 每帧前清空单路诊断瞬时状态位。
 * @param diag 诊断结构体指针。
 * @return 无。
 */
static void VL53L1X_DiagPrepareFrame(VL53L1X_diag_struct *diag)
{
    diag->ack_ok = 0U;
    diag->ready = 0U;
    diag->range_ok = 0U;
    diag->is_fresh = 0U;
}

/**
 * @brief 依据ACK结果更新单路通信健康计数。
 *        ACK成功则递减失败计数，失败则递增，防止溢出。
 * @param diag 诊断结构体指针。
 * @param ack_ok ACK是否成功，1=成功，0=失败。
 * @return 无。
 */
static void VL53L1X_DiagUpdateComm(VL53L1X_diag_struct *diag, uint8 ack_ok)
{
    diag->ack_ok = ack_ok;

    if (0U != ack_ok)
    {
        if (diag->comm_fail_count > 0U)
        {
            diag->comm_fail_count--;
        }
    }
    else if (diag->comm_fail_count < 65535U)
    {
        diag->comm_fail_count++;
    }
}

/**
 * @brief 根据ready、量测有效性与重复值判定单路数据新鲜度。
 *        首帧有效距离时直接标记新鲜；后续帧若距离或ready边沿变化则新鲜；
 *        连续相同超过阈值（VL53L1X_STALE_FRAME_TH）则判定陈旧。
 * @param diag 诊断结构体指针。
 * @param ready 当前帧ready标志，1=就绪。
 * @param distance_valid 当前帧距离是否有效，1=有效。
 * @param distance_mm 当前帧距离（单位：mm）。
 * @param last_distance 上一有效距离缓存指针（单位：mm）。
 * @param has_last_distance 是否已存在上一有效距离，0=尚无，1=已有。
 * @param last_ready 上一帧ready缓存指针。
 * @return 1=新鲜，0=不新鲜。
 */
static uint8 VL53L1X_UpdateFreshness(VL53L1X_diag_struct *diag,
                                     uint8 ready,
                                     uint8 distance_valid,
                                     uint16 distance_mm,
                                     uint16 *last_distance,
                                     uint8 *has_last_distance,
                                     uint8 *last_ready)
{
    if ((0U == ready) || (0U == distance_valid))
    {
        diag->is_fresh = 0U;
        if (0U == ready)
        {
            *last_ready = 0U;
        }
        return 0U;
    }

    if (0U == *has_last_distance)
    {
        *has_last_distance = 1U;
        *last_distance = distance_mm;
        *last_ready = ready;
        diag->stale_count = 0U;
        diag->is_fresh = 1U;
        return 1U;
    }

    if ((distance_mm != *last_distance) || ((0U == *last_ready) && (0U != ready)))
    {
        *last_distance = distance_mm;
        *last_ready = ready;
        diag->stale_count = 0U;
        diag->is_fresh = 1U;
        return 1U;
    }

    if (diag->stale_count < 65535U)
    {
        diag->stale_count++;
    }

    *last_ready = ready;
    diag->is_fresh = (diag->stale_count < VL53L1X_STALE_FRAME_TH) ? 1U : 0U;
    return diag->is_fresh;
}

/**
 * @brief 检查四路通道，对通信失败超阈值或未初始化的通道触发重初始化恢复。
 * @param 无。
 * @return 无。
 */
static void VL53L1X_TryRecoverChannels(void)
{
    uint8 ch = 0U;

    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        if ((g_vl53l1x_diag[ch].comm_fail_count >= VL53L1X_COMM_FAIL_REINIT_TH) ||
            (s_vl53l1x_init_flag[ch] == 0U))
        {
            if (0U == VL53L1X_init_internal(ch,
                                            VL53L1X_QUAD_SOFT_IIC_DELAY_BASE,
                                            VL53L1X_QUAD_SOFT_IIC_DELAY_STEP,
                                            VL53L1X_QUAD_SOFT_IIC_DELAY_MAX))
            {
                /* 重初始化成功，清零该路诊断与新鲜度缓存。 */
                g_vl53l1x_diag[ch].comm_fail_count = 0U;
                g_vl53l1x_diag[ch].stale_count = 0U;
                s_vl53l1x_has_last_distance[ch] = 0U;
                s_vl53l1x_last_ready[ch] = 0U;
            }
        }
    }
}

/**
 * @brief 10Hz维护接口：对通信异常或未初始化的通道尝试重初始化恢复。
 * @param 无。
 * @return 无。
 */
void VL53L1X_recover_update_10HZ(void)
{
    VL53L1X_TryRecoverChannels();
}

/**
 * @brief 读取四路VL53L1X测距数据。
 *        四路通道采用同步软IIC时序，4路读取总时间约等于单路读取时间。
 *        内部流程：同步读TIO_STATUS → 同步读RANGE_STATUS → 同步读距离寄存器 → 筛选裁剪 → 新鲜度判定 → 清中断。
 * @param data 输出数据指针，写入各路距离（distance_mm[]）与量程状态（range_status[]）。
 * @return 有效新鲜位掩码：bit0~bit3分别对应TOF1~TOF4，1=本帧新鲜有效，0=无效或陈旧。
 */
uint8 VL53L1X_read_data(VL53L1X_data_struct *data)
{
    uint8  valid_mask = 0U;
    uint8  ack_mask   = 0U;
    uint8  ch = 0U;
    uint8  any_ready = 0U;
    uint8  all_uninit = 1U;
    uint8  ch_ready[VL53L1X_CHANNEL_COUNT]   = {0U, 0U, 0U, 0U};
    uint8  tio_buf[VL53L1X_CHANNEL_COUNT][2] = {{0U}, {0U}, {0U}, {0U}};
    uint8  rs_buf[VL53L1X_CHANNEL_COUNT][2]  = {{0xFFU}, {0xFFU}, {0xFFU}, {0xFFU}};
    uint8  dist_buf[VL53L1X_CHANNEL_COUNT][2]= {{0xFFU, 0xFFU}, {0xFFU, 0xFFU},
                                                 {0xFFU, 0xFFU}, {0xFFU, 0xFFU}};
    uint16 dist_temp  = 0U;
    uint8  dist_valid[VL53L1X_CHANNEL_COUNT] = {0U, 0U, 0U, 0U};

    if (NULL == data)
    {
        return 0U;
    }

    /* 每帧先清空诊断瞬时位并设置输出默认无效值。 */
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        VL53L1X_DiagPrepareFrame(&g_vl53l1x_diag[ch]);
        data->distance_mm[ch]  = VL53L1X_INVALID_DISTANCE_MM;
        data->range_status[ch] = 0xFFU;
        if (0U != s_vl53l1x_init_flag[ch]) { all_uninit = 0U; }
    }

    if (0U != all_uninit)
    {
        VL53L1X_data = *data;
        return 0U;
    }

    /* 配置四路引脚为开漏输出并拉高，准备同步IIC操作。 */
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        VL53L1X_soft_iic_pin_config(s_scl_pins[ch], s_sda_pins[ch]);
    }

    /* 四路同步读取TIO_HV_STATUS，获取各路ready信号并更新通信诊断。 */
    ack_mask = VL53L1X_quad_read_reg_array(DL1B_GPIO__TIO_HV_STATUS, tio_buf, 1U);
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        uint8 ack_ok = (ack_mask & (uint8)(1U << ch)) ? 1U : 0U;
        VL53L1X_DiagUpdateComm(&g_vl53l1x_diag[ch], ack_ok);
        if (0U == ack_ok) { tio_buf[ch][0] = 0U; }
        ch_ready[ch] = ((s_vl53l1x_init_flag[ch] != 0U) && (tio_buf[ch][0] != 0U)) ? 1U : 0U;
        g_vl53l1x_diag[ch].ready = ch_ready[ch];
        if (0U != ch_ready[ch]) { any_ready = 1U; }
    }

    /* 至少一路ready时，四路同步读取RANGE_STATUS与距离寄存器。 */
    if (0U != any_ready)
    {
        /* 同步读取RANGE_STATUS。 */
        ack_mask = VL53L1X_quad_read_reg_array(DL1B_RESULT__RANGE_STATUS, rs_buf, 1U);
        for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
        {
            uint8 ack_ok = (ack_mask & (uint8)(1U << ch)) ? 1U : 0U;
            VL53L1X_DiagUpdateComm(&g_vl53l1x_diag[ch], ack_ok);
            if ((0U == ack_ok) || (0U == ch_ready[ch])) { rs_buf[ch][0] = 0xFFU; }
        }

        /* 同步读取距离寄存器（2字节高位在前）。 */
        ack_mask = VL53L1X_quad_read_reg_array(DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, dist_buf, 2U);
        for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
        {
            uint8 ack_ok = (ack_mask & (uint8)(1U << ch)) ? 1U : 0U;
            VL53L1X_DiagUpdateComm(&g_vl53l1x_diag[ch], ack_ok);
            if ((0U == ack_ok) || (0U == ch_ready[ch]))
            {
                dist_buf[ch][0] = 0xFFU;
                dist_buf[ch][1] = 0xFFU;
            }
        }
    }

    /* 按range_status和量程门限筛选有效距离，并执行有效量程上限裁剪。 */
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        data->range_status[ch] = rs_buf[ch][0];
        if ((0U != ch_ready[ch]) && (0x89U == rs_buf[ch][0]))
        {
            dist_temp = (uint16)(((uint16)dist_buf[ch][0] << 8U) | dist_buf[ch][1]);
            if (dist_temp <= 4000U)
            {
                if (dist_temp > (uint16)VL53L1X_VALID_RANGE_MAX)
                {
                    dist_temp = (uint16)VL53L1X_VALID_RANGE_MAX;
                }
                data->distance_mm[ch] = dist_temp;
                dist_valid[ch] = 1U;
                g_vl53l1x_diag[ch].range_ok = 1U;
            }
        }
    }

    /* 结合上一帧信息判定各路数据新鲜度并生成有效位掩码。 */
    for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
    {
        if (0U != VL53L1X_UpdateFreshness(&g_vl53l1x_diag[ch],
                                          ch_ready[ch],
                                          dist_valid[ch],
                                          data->distance_mm[ch],
                                          &s_vl53l1x_last_distance[ch],
                                          &s_vl53l1x_has_last_distance[ch],
                                          &s_vl53l1x_last_ready[ch]))
        {
            valid_mask |= (uint8)(1U << ch);
        }
    }

    /* 本帧有ready通道时，四路同步清除设备中断，并更新通信诊断。 */
    if (0U != any_ready)
    {
        ack_mask = VL53L1X_quad_write_reg_8bit(DL1B_SYSTEM__INTERRUPT_CLEAR, 0x01U);
        for (ch = 0U; ch < VL53L1X_CHANNEL_COUNT; ch++)
        {
            VL53L1X_DiagUpdateComm(&g_vl53l1x_diag[ch],
                                   (ack_mask & (uint8)(1U << ch)) ? 1U : 0U);
        }
    }

    /* 更新模块级缓存，供外部快速访问。 */
    VL53L1X_data = *data;
    return valid_mask;
}
