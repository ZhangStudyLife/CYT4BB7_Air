#include "VL53L1X.h"
#include "zf_device_config.h"
#include <string.h>

#define VL53L1X_IIC_ADDR                 (0x52 >> 1) /* VL53L1X 的 7 位 IIC 地址 */
#define VL53L1X_IIC_DELAY                (100U)      /* 软 IIC 延时参数 */
#define VL53L1X_BUS_RECOVERY_PULSE       (9U)        /* 总线恢复脉冲数 */
#define VL53L1X_PROBE_RETRY              (3U)        /* 地址探测重试次数 */
#define VL53L1X_FW_RETRY                 (5U)        /* 固件状态读取重试次数 */
#define VL53L1X_BUS_RETRY                (5U)        /* 总线恢复重试次数 */
#define VL53L1X_READY_TIMEOUT            (1000U)     /* 初始化时等待 ready 超时计数 */

#define VL53L1X_REG_DEVICE_ADDRESS       (0x0001U)   /* 设备地址寄存器 */
#define VL53L1X_REG_GPIO_STATUS          (0x0031U)   /* 数据 ready 状态寄存器 */
#define VL53L1X_REG_INTERRUPT_CLEAR      (0x0086U)   /* 中断清除寄存器 */
#define VL53L1X_REG_RANGE_STATUS         (0x0089U)   /* 测距状态寄存器 */
#define VL53L1X_REG_DISTANCE_MM          (0x0096U)   /* 距离结果寄存器 */
#define VL53L1X_REG_FW_STATUS            (0x00E5U)   /* 固件状态寄存器 */
#define VL53L1X_ACK_ALL_MASK             ((uint8)((1UL << VL53L1X_SENSOR_COUNT) - 1UL)) /* 四路 ACK 掩码 */

/* 四路 TOF 的软 IIC 句柄 */
static soft_iic_info_struct s_vl53l1x_iic[VL53L1X_SENSOR_COUNT];
/* 四路 TOF 的初始化成功标志 */
static uint8 s_vl53l1x_init_ok[VL53L1X_SENSOR_COUNT] = {0U, 0U, 0U, 0U};
/* 四路 TOF 的最新缓存数据 */
static VL53L1X_data_struct s_vl53l1x_data =
{
    {
        VL53L1X_INVALID_DISTANCE_MM,
        VL53L1X_INVALID_DISTANCE_MM,
        VL53L1X_INVALID_DISTANCE_MM,
        VL53L1X_INVALID_DISTANCE_MM
    },
    {0U, 0U, 0U, 0U}
};

/* 四路 TOF 的 SCL 引脚表，顺序为 TOF1、TOF2、TOF3、TOF4 */
static const gpio_pin_enum s_vl53l1x_scl_pins[VL53L1X_SENSOR_COUNT] =
{
    VL53L1X_TOF1_SCL_PIN,
    VL53L1X_TOF2_SCL_PIN,
    VL53L1X_TOF3_SCL_PIN,
    VL53L1X_TOF4_SCL_PIN
};

/* 四路 TOF 的 SDA 引脚表，顺序为 TOF1、TOF2、TOF3、TOF4 */
static const gpio_pin_enum s_vl53l1x_sda_pins[VL53L1X_SENSOR_COUNT] =
{
    VL53L1X_TOF1_SDA_PIN,
    VL53L1X_TOF2_SDA_PIN,
    VL53L1X_TOF3_SDA_PIN,
    VL53L1X_TOF4_SDA_PIN
};

/*
 * 函数功能：将软 IIC 引脚配置为开漏输出并拉高空闲。
 * 输入参数：
 *   scl_pin：SCL 引脚
 *   sda_pin：SDA 引脚
 * 返回值：
 *   无
 */
static void VL53L1X_PinConfig(gpio_pin_enum scl_pin, gpio_pin_enum sda_pin)
{
    gpio_set_dir(scl_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir(sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_high(scl_pin);
    gpio_high(sda_pin);
}

/*
 * 函数功能：准备总线为上拉输入状态。
 * 输入参数：
 *   scl_pin：SCL 引脚
 *   sda_pin：SDA 引脚
 * 返回值：
 *   无
 */
static void VL53L1X_PrepareBus(gpio_pin_enum scl_pin, gpio_pin_enum sda_pin)
{
    gpio_init(scl_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    gpio_init(sda_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    gpio_set_dir(scl_pin, GPI, GPI_PULL_UP);
    gpio_set_dir(sda_pin, GPI, GPI_PULL_UP);
    system_delay_ms(1);
}

/*
 * 函数功能：在指定总线上执行恢复脉冲。
 * 输入参数：
 *   scl_pin：SCL 引脚
 *   sda_pin：SDA 引脚
 * 返回值：
 *   无
 */
static void VL53L1X_BusRecovery(gpio_pin_enum scl_pin, gpio_pin_enum sda_pin)
{
    uint8 index = 0U;

    gpio_init(scl_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    gpio_init(sda_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    system_delay_ms(1);

    for (index = 0U; index < VL53L1X_BUS_RECOVERY_PULSE; index++)
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

/*
 * 函数功能：扫描当前总线第一个有 ACK 的地址。
 * 输入参数：
 *   iic_obj：软 IIC 句柄
 * 返回值：
 *   扫描到的地址，失败返回 0xFF
 */
static uint8 VL53L1X_ScanFirstAckAddr(soft_iic_info_struct *iic_obj)
{
    uint8 addr = 0U;

    for (addr = 0x08U; addr <= 0x77U; addr++)
    {
        if (0U != soft_iic_probe_ack(iic_obj, addr))
        {
            return addr;
        }
    }

    return 0xFFU;
}

/*
 * 函数功能：读取 8 位寄存器。
 * 输入参数：
 *   iic_obj：软 IIC 句柄
 *   reg_addr：16 位寄存器地址
 *   reg_data：输出寄存器值指针
 * 返回值：
 *   无
 */
static void VL53L1X_ReadReg8(soft_iic_info_struct *iic_obj, uint16 reg_addr, uint8 *reg_data)
{
    soft_iic_write_16bit(iic_obj, reg_addr);
    *reg_data = soft_iic_read_8bit(iic_obj);
}

/*
 * 函数功能：重试读取固件状态寄存器。
 * 输入参数：
 *   iic_obj：软 IIC 句柄
 * 返回值：
 *   固件状态值，失败返回 0xFF
 */
static uint8 VL53L1X_ReadFwStatus(soft_iic_info_struct *iic_obj)
{
    uint8 fw_status = 0xFFU;
    uint8 retry = 0U;

    for (retry = 0U; retry < VL53L1X_FW_RETRY; retry++)
    {
        VL53L1X_ReadReg8(iic_obj, VL53L1X_REG_FW_STATUS, &fw_status);
        if (0xFFU != fw_status)
        {
            break;
        }
        system_delay_ms(1);
    }

    return fw_status;
}

/*
 * 函数功能：轻量同步延时。
 * 输入参数：
 *   delay：延时计数
 * 返回值：
 *   无
 */
static void VL53L1X_SyncDelay(uint32 delay)
{
    volatile uint32 count = delay;

    while (count--)
    {
    }
}

/*
 * 函数功能：同步设置四路 SCL 电平。
 * 输入参数：
 *   level：目标电平，0=低电平，非 0=高电平
 * 返回值：
 *   无
 */
static void VL53L1X_SetAllScl(uint8 level)
{
    uint8 index = 0U;

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        if (0U != level)
        {
            gpio_high((gpio_pin_enum)s_vl53l1x_iic[index].scl_pin);
        }
        else
        {
            gpio_low((gpio_pin_enum)s_vl53l1x_iic[index].scl_pin);
        }
    }
}

/*
 * 函数功能：同步设置四路 SDA 电平。
 * 输入参数：
 *   level：目标电平，0=低电平，非 0=高电平
 * 返回值：
 *   无
 */
static void VL53L1X_SetAllSda(uint8 level)
{
    uint8 index = 0U;

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        if (0U != level)
        {
            gpio_high((gpio_pin_enum)s_vl53l1x_iic[index].sda_pin);
        }
        else
        {
            gpio_low((gpio_pin_enum)s_vl53l1x_iic[index].sda_pin);
        }
    }
}

/*
 * 函数功能：同步设置四路 SDA 输入输出方向。
 * 输入参数：
 *   dir：GPIO 方向
 *   mode：GPIO 模式
 * 返回值：
 *   无
 */
static void VL53L1X_SetAllSdaDir(gpio_dir_enum dir, gpio_mode_enum mode)
{
    uint8 index = 0U;

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        gpio_set_dir((gpio_pin_enum)s_vl53l1x_iic[index].sda_pin, dir, mode);
    }
}

/*
 * 函数功能：四路 IIC 同时发起 START。
 * 输入参数：
 *   delay：时序延时参数
 * 返回值：
 *   无
 */
static void VL53L1X_SyncStart(uint32 delay)
{
    VL53L1X_SetAllScl(1U);
    VL53L1X_SetAllSda(1U);

    VL53L1X_SyncDelay(delay);
    VL53L1X_SetAllSda(0U);
    VL53L1X_SyncDelay(delay);
    VL53L1X_SetAllScl(0U);
    VL53L1X_SyncDelay(delay);
}

/*
 * 函数功能：四路 IIC 同时发起 STOP。
 * 输入参数：
 *   delay：时序延时参数
 * 返回值：
 *   无
 */
static void VL53L1X_SyncStop(uint32 delay)
{
    VL53L1X_SetAllSda(0U);
    VL53L1X_SetAllScl(0U);

    VL53L1X_SyncDelay(delay);
    VL53L1X_SetAllScl(1U);
    VL53L1X_SyncDelay(delay);
    VL53L1X_SetAllSda(1U);
    VL53L1X_SyncDelay(delay);
}

/*
 * 函数功能：四路 IIC 同时发送同一个字节，并返回 ACK 掩码。
 * 输入参数：
 *   data：待发送字节
 *   delay：时序延时参数
 * 返回值：
 *   ACK 掩码，bit0~bit3 对应 TOF1~TOF4
 */
static uint8 VL53L1X_SyncSendByte(uint8 data, uint32 delay)
{
    uint8 index = 0U;
    uint8 mask = 0x80U;
    uint8 ack_mask = 0U;

    while (0U != mask)
    {
        if (0U != (data & mask))
        {
            VL53L1X_SetAllSda(1U);
        }
        else
        {
            VL53L1X_SetAllSda(0U);
        }

        mask >>= 1U;
        VL53L1X_SyncDelay(delay / 2U);
        VL53L1X_SetAllScl(1U);
        VL53L1X_SyncDelay(delay);
        VL53L1X_SetAllScl(0U);
        VL53L1X_SyncDelay(delay / 2U);
    }

    VL53L1X_SetAllScl(0U);
    VL53L1X_SetAllSda(1U);
    VL53L1X_SetAllSdaDir(GPI, GPI_FLOATING_IN);
    VL53L1X_SyncDelay(delay);

    VL53L1X_SetAllScl(1U);
    VL53L1X_SyncDelay(delay);

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        if (!gpio_get_level((gpio_pin_enum)s_vl53l1x_iic[index].sda_pin))
        {
            ack_mask |= (uint8)(1U << index);
        }
    }

    VL53L1X_SetAllScl(0U);
    VL53L1X_SetAllSdaDir(GPO, GPO_OPEN_DTAIN);
    VL53L1X_SyncDelay(delay);

    return ack_mask;
}

/*
 * 函数功能：四路 IIC 同时读取一个字节。
 * 输入参数：
 *   out_data：输出数组，长度为 VL53L1X_SENSOR_COUNT
 *   nack：1 表示最后一个字节发送 NACK，0 表示发送 ACK
 *   delay：时序延时参数
 * 返回值：
 *   无
 */
static void VL53L1X_SyncReadByte(uint8 out_data[VL53L1X_SENSOR_COUNT], uint8 nack, uint32 delay)
{
    uint8 index = 0U;
    uint8 bit_count = 8U;

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        out_data[index] = 0U;
    }

    VL53L1X_SetAllScl(0U);
    VL53L1X_SyncDelay(delay);
    VL53L1X_SetAllSda(1U);
    VL53L1X_SetAllSdaDir(GPI, GPI_FLOATING_IN);

    while (0U != bit_count--)
    {
        VL53L1X_SetAllScl(0U);
        VL53L1X_SyncDelay(delay);
        VL53L1X_SetAllScl(1U);
        VL53L1X_SyncDelay(delay);
        for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
        {
            out_data[index] = (uint8)((out_data[index] << 1U) | gpio_get_level((gpio_pin_enum)s_vl53l1x_iic[index].sda_pin));
        }
    }

    VL53L1X_SetAllScl(0U);
    VL53L1X_SetAllSdaDir(GPO, GPO_OPEN_DTAIN);
    VL53L1X_SyncDelay(delay);

    if (0U != nack)
    {
        VL53L1X_SetAllSda(1U);
    }
    else
    {
        VL53L1X_SetAllSda(0U);
    }

    VL53L1X_SyncDelay(delay);
    VL53L1X_SetAllScl(1U);
    VL53L1X_SyncDelay(delay);
    VL53L1X_SetAllScl(0U);
    VL53L1X_SetAllSda(1U);
}

/*
 * 函数功能：四路 TOF 同时读取同一个寄存器块。
 * 输入参数：
 *   reg_addr：16 位寄存器地址
 *   buf：输出缓冲区，第一维为四路 TOF
 *   len：读取字节数
 * 返回值：
 *   ACK 掩码，bit0~bit3 对应 TOF1~TOF4
 */
static uint8 VL53L1X_SyncReadRegArray(uint16 reg_addr,
                                      uint8 buf[VL53L1X_SENSOR_COUNT][2],
                                      uint32 len)
{
    uint32 index = 0U;
    uint8 tof_index = 0U;
    uint8 ack_mask = VL53L1X_ACK_ALL_MASK;
    uint8 read_byte[VL53L1X_SENSOR_COUNT];
    uint8 write_addr = (uint8)(VL53L1X_IIC_ADDR << 1U);
    uint8 read_addr = (uint8)((VL53L1X_IIC_ADDR << 1U) | 0x01U);

    VL53L1X_SyncStart(VL53L1X_IIC_DELAY);
    ack_mask &= VL53L1X_SyncSendByte(write_addr, VL53L1X_IIC_DELAY);
    ack_mask &= VL53L1X_SyncSendByte((uint8)(reg_addr >> 8U), VL53L1X_IIC_DELAY);
    ack_mask &= VL53L1X_SyncSendByte((uint8)(reg_addr & 0xFFU), VL53L1X_IIC_DELAY);
    VL53L1X_SyncStart(VL53L1X_IIC_DELAY);
    ack_mask &= VL53L1X_SyncSendByte(read_addr, VL53L1X_IIC_DELAY);

    for (index = 0U; index < len; index++)
    {
        VL53L1X_SyncReadByte(read_byte, (index == (len - 1U)) ? 1U : 0U, VL53L1X_IIC_DELAY);
        for (tof_index = 0U; tof_index < VL53L1X_SENSOR_COUNT; tof_index++)
        {
            buf[tof_index][index] = read_byte[tof_index];
        }
    }

    VL53L1X_SyncStop(VL53L1X_IIC_DELAY);
    return ack_mask;
}

/*
 * 函数功能：四路 TOF 同时写入同一个 8 位寄存器。
 * 输入参数：
 *   reg_addr：16 位寄存器地址
 *   value：待写入的 8 位值
 * 返回值：
 *   ACK 掩码，bit0~bit3 对应 TOF1~TOF4
 */
static uint8 VL53L1X_SyncWriteReg8(uint16 reg_addr, uint8 value)
{
    uint8 ack_mask = VL53L1X_ACK_ALL_MASK;
    uint8 write_addr = (uint8)(VL53L1X_IIC_ADDR << 1U);

    VL53L1X_SyncStart(VL53L1X_IIC_DELAY);
    ack_mask &= VL53L1X_SyncSendByte(write_addr, VL53L1X_IIC_DELAY);
    ack_mask &= VL53L1X_SyncSendByte((uint8)(reg_addr >> 8U), VL53L1X_IIC_DELAY);
    ack_mask &= VL53L1X_SyncSendByte((uint8)(reg_addr & 0xFFU), VL53L1X_IIC_DELAY);
    ack_mask &= VL53L1X_SyncSendByte(value, VL53L1X_IIC_DELAY);
    VL53L1X_SyncStop(VL53L1X_IIC_DELAY);

    return ack_mask;
}

/*
 * 函数功能：初始化指定一路 TOF。
 * 输入参数：
 *   index：通道索引，0=TOF1，1=TOF2，2=TOF3，3=TOF4
 * 返回值：
 *   1：初始化成功
 *   0：初始化失败
 */
static uint8 VL53L1X_InitSingle(uint8 index)
{
    uint8 retry = 0U;
    uint8 addr_retry = 0U;
    uint8 addr_ok = 0U;
    uint8 fw_status = 0xFFU;
    uint8 scan_addr = 0xFFU;
    uint8 data_buffer[2 + sizeof(dl1b_config_file)];
    uint16 timeout_count = 0U;
    gpio_pin_enum scl_pin = s_vl53l1x_scl_pins[index];
    gpio_pin_enum sda_pin = s_vl53l1x_sda_pins[index];

    s_vl53l1x_init_ok[index] = 0U;

    for (retry = 0U; retry <= VL53L1X_BUS_RETRY; retry++)
    {
        VL53L1X_PrepareBus(scl_pin, sda_pin);
        soft_iic_init(&s_vl53l1x_iic[index], VL53L1X_IIC_ADDR, VL53L1X_IIC_DELAY, scl_pin, sda_pin);
        VL53L1X_PinConfig(scl_pin, sda_pin);

        addr_ok = 0U;
        for (addr_retry = 0U; addr_retry < VL53L1X_PROBE_RETRY; addr_retry++)
        {
            if (0U != soft_iic_probe_ack(&s_vl53l1x_iic[index], VL53L1X_IIC_ADDR))
            {
                addr_ok = 1U;
                break;
            }
            system_delay_ms(1);
        }

        if (0U == addr_ok)
        {
            scan_addr = VL53L1X_ScanFirstAckAddr(&s_vl53l1x_iic[index]);
            if (VL53L1X_IIC_ADDR == scan_addr)
            {
                addr_ok = 1U;
                system_delay_ms(10);
            }
            else
            {
                VL53L1X_BusRecovery(scl_pin, sda_pin);
                system_delay_ms(10);
                continue;
            }
        }

        fw_status = VL53L1X_ReadFwStatus(&s_vl53l1x_iic[index]);
        if (0xFFU != fw_status)
        {
            break;
        }

        VL53L1X_BusRecovery(scl_pin, sda_pin);
        system_delay_ms(10);
    }

    if ((0xFFU == fw_status) || (0U == (fw_status & 0x01U)))
    {
        return 0U;
    }

    data_buffer[0] = (uint8)(VL53L1X_REG_DEVICE_ADDRESS >> 8U);
    data_buffer[1] = (uint8)(VL53L1X_REG_DEVICE_ADDRESS & 0xFFU);
    memcpy(&data_buffer[2], (const uint8 *)dl1b_config_file, sizeof(dl1b_config_file));
    soft_iic_transfer_8bit_array(&s_vl53l1x_iic[index],
                                 data_buffer,
                                 2U + sizeof(dl1b_config_file),
                                 data_buffer,
                                 0U);

    while (1)
    {
        VL53L1X_ReadReg8(&s_vl53l1x_iic[index], VL53L1X_REG_GPIO_STATUS, &data_buffer[2]);
        if (0x00U == (data_buffer[2] & 0x01U))
        {
            break;
        }

        timeout_count++;
        if (timeout_count > VL53L1X_READY_TIMEOUT)
        {
            return 0U;
        }
        system_delay_ms(1);
    }

    s_vl53l1x_init_ok[index] = 1U;
    return 1U;
}

/*
 * 函数功能：初始化四路 VL53L1X。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void VL53L1X_Init(void)
{
    uint8 index = 0U;

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        s_vl53l1x_data.distance_mm[index] = VL53L1X_INVALID_DISTANCE_MM;
        s_vl53l1x_data.valid[index] = 0U;
        (void)VL53L1X_InitSingle(index);
    }
}

/*
 * 函数功能：非堵塞更新四路 VL53L1X 最新测距结果。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void VL53L1X_Update(void)
{
    uint8 index = 0U;
    uint8 ack_ready = 0U;
    uint8 ack_status = 0U;
    uint8 ack_distance = 0U;
    uint8 ready_mask = 0U;
    uint8 ready_buf[VL53L1X_SENSOR_COUNT][2] = {0U};
    uint8 status_buf[VL53L1X_SENSOR_COUNT][2] = {0U};
    uint8 distance_buf[VL53L1X_SENSOR_COUNT][2] = {0U};
    uint16 distance_mm = 0U;

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        status_buf[index][0] = 0xFFU;
        distance_buf[index][0] = 0xFFU;
        distance_buf[index][1] = 0xFFU;

        if (0U != s_vl53l1x_init_ok[index])
        {
            VL53L1X_PinConfig(s_vl53l1x_scl_pins[index], s_vl53l1x_sda_pins[index]);
        }
        else
        {
            s_vl53l1x_data.valid[index] = 0U;
            s_vl53l1x_data.distance_mm[index] = VL53L1X_INVALID_DISTANCE_MM;
        }
    }

    ack_ready = VL53L1X_SyncReadRegArray(VL53L1X_REG_GPIO_STATUS, ready_buf, 1U);

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        if (0U == s_vl53l1x_init_ok[index])
        {
            continue;
        }

        if (0U == (ack_ready & (1U << index)))
        {
            s_vl53l1x_data.valid[index] = 0U;
            s_vl53l1x_data.distance_mm[index] = VL53L1X_INVALID_DISTANCE_MM;
            continue;
        }

        if (0U != ready_buf[index][0])
        {
            ready_mask |= (uint8)(1U << index);
        }
    }

    if (0U == ready_mask)
    {
        return;
    }

    ack_status = VL53L1X_SyncReadRegArray(VL53L1X_REG_RANGE_STATUS, status_buf, 1U);
    ack_distance = VL53L1X_SyncReadRegArray(VL53L1X_REG_DISTANCE_MM, distance_buf, 2U);
    (void)VL53L1X_SyncWriteReg8(VL53L1X_REG_INTERRUPT_CLEAR, 0x01U);

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        if (0U == (ready_mask & (1U << index)))
        {
            continue;
        }

        if (((ack_status & (1U << index)) == 0U) ||
            ((ack_distance & (1U << index)) == 0U) ||
            (0x89U != status_buf[index][0]))
        {
            s_vl53l1x_data.valid[index] = 0U;
            s_vl53l1x_data.distance_mm[index] = VL53L1X_INVALID_DISTANCE_MM;
            continue;
        }

        distance_mm = (uint16)(((uint16)distance_buf[index][0] << 8U) | distance_buf[index][1]);
        if (distance_mm > (uint16)VL53L1X_VALID_RANGE_MAX)
        {
            distance_mm = (uint16)VL53L1X_VALID_RANGE_MAX;
        }

        s_vl53l1x_data.distance_mm[index] = distance_mm;
        s_vl53l1x_data.valid[index] = 1U;
    }
}

/*
 * 函数功能：获取四路 VL53L1X 最新缓存数据。
 * 输入参数：
 *   无
 * 返回值：
 *   指向内部缓存的只读指针
 */
const VL53L1X_data_struct *VL53L1X_GetData(void)
{
    return &s_vl53l1x_data;
}
