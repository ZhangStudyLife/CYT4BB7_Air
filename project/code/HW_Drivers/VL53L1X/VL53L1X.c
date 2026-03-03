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
#define DL1B_FW_STATUS_READ_RETRY_COUNT                          ( 5 )
/* 双路TOF软IIC基础延时。 */
#define VL53L1X23_SOFT_IIC_DELAY_BASE                               ( DL1B_SOFT_IIC_DELAY )
/* 双路TOF软IIC延时步长。 */
#define VL53L1X23_SOFT_IIC_DELAY_STEP                               ( DL1B_SOFT_IIC_DELAY_STEP )
/* 双路TOF软IIC延时上限。 */
#define VL53L1X23_SOFT_IIC_DELAY_MAX                                ( DL1B_SOFT_IIC_DELAY_MAX )
/* 双路同步IIC操作延时。 */
#define VL53L1X_SYNC_SOFT_IIC_DELAY                                 ( VL53L1X23_SOFT_IIC_DELAY_BASE )

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

/* 2号TOF软IIC句柄。 */
static soft_iic_info_struct VL53L1X2_iic_struct;
/* 3号TOF软IIC句柄。 */
static soft_iic_info_struct VL53L1X3_iic_struct;

/* 2号TOF初始化成功标志。 */
static uint8 VL53L1X2_init_flag = 0;
/* 3号TOF初始化成功标志。 */
static uint8 VL53L1X3_init_flag = 0;

/* 双路TOF最新输出缓存。 */
VL53L1X_data_struct VL53L1X_data =
{
    VL53L1X_INVALID_DISTANCE_MM,
    VL53L1X_INVALID_DISTANCE_MM,
    0xFF,
    0xFF
};

/* 2号TOF诊断状态。 */
VL53L1X_diag_struct g_vl53l1x2_diag = {0};
/* 3号TOF诊断状态。 */
VL53L1X_diag_struct g_vl53l1x3_diag = {0};

/* 2号TOF上一帧距离（用于新鲜度判定，单位：mm）。 */
static uint16 s_vl53l1x2_last_distance = VL53L1X_INVALID_DISTANCE_MM;
/* 3号TOF上一帧距离（用于新鲜度判定，单位：mm）。 */
static uint16 s_vl53l1x3_last_distance = VL53L1X_INVALID_DISTANCE_MM;
/* 2号TOF是否已有上一帧距离。 */
static uint8 s_vl53l1x2_has_last_distance = 0U;
/* 3号TOF是否已有上一帧距离。 */
static uint8 s_vl53l1x3_has_last_distance = 0U;
/* 2号TOF上一帧ready状态。 */
static uint8 s_vl53l1x2_last_ready = 0U;
/* 3号TOF上一帧ready状态。 */
static uint8 s_vl53l1x3_last_ready = 0U;

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
 * @brief 双路IIC同时发起START信号。
 * @param VL53L1X2_obj 2号软IIC对象。
 * @param VL53L1X3_obj 3号软IIC对象。
 * @param delay 时序延时计数。
 * @return 无。
 */
static void VL53L1X_dual_iic_start(soft_iic_info_struct *VL53L1X2_obj, soft_iic_info_struct *VL53L1X3_obj, uint32 delay)
{
    gpio_high(VL53L1X2_obj->scl_pin);
    gpio_high(VL53L1X3_obj->scl_pin);
    gpio_high(VL53L1X2_obj->sda_pin);
    gpio_high(VL53L1X3_obj->sda_pin);

    VL53L1X_sync_delay(delay);
    gpio_low(VL53L1X2_obj->sda_pin);
    gpio_low(VL53L1X3_obj->sda_pin);
    VL53L1X_sync_delay(delay);
    gpio_low(VL53L1X2_obj->scl_pin);
    gpio_low(VL53L1X3_obj->scl_pin);
    VL53L1X_sync_delay(delay);
}

/**
 * @brief 双路IIC同时发出STOP信号。
 * @param VL53L1X2_obj 2号软IIC对象。
 * @param VL53L1X3_obj 3号软IIC对象。
 * @param delay 时序延时计数。
 * @return 无。
 */
static void VL53L1X_dual_iic_stop(soft_iic_info_struct *VL53L1X2_obj, soft_iic_info_struct *VL53L1X3_obj, uint32 delay)
{
    gpio_low(VL53L1X2_obj->sda_pin);
    gpio_low(VL53L1X3_obj->sda_pin);
    gpio_low(VL53L1X2_obj->scl_pin);
    gpio_low(VL53L1X3_obj->scl_pin);

    VL53L1X_sync_delay(delay);
    gpio_high(VL53L1X2_obj->scl_pin);
    gpio_high(VL53L1X3_obj->scl_pin);
    VL53L1X_sync_delay(delay);
    gpio_high(VL53L1X2_obj->sda_pin);
    gpio_high(VL53L1X3_obj->sda_pin);
    VL53L1X_sync_delay(delay);
}

/**
 * @brief 双路IIC同时发送一个字节并采集ACK位。
 * @param VL53L1X2_obj 2号软IIC对象。
 * @param VL53L1X3_obj 3号软IIC对象。
 * @param data 待发送字节。
 * @param delay 时序延时计数。
 * @return ACK掩码：bit0对应2号，bit1对应3号，1表示收到ACK。
 */
static uint8 VL53L1X_dual_iic_send_data(soft_iic_info_struct *VL53L1X2_obj, soft_iic_info_struct *VL53L1X3_obj, uint8 data, uint32 delay)
{
    uint8 temp = 0x80;
    uint8 ack_mask = 0;

    while(temp)
    {
        if(data & temp)
        {
            gpio_high(VL53L1X2_obj->sda_pin);
            gpio_high(VL53L1X3_obj->sda_pin);
        }
        else
        {
            gpio_low(VL53L1X2_obj->sda_pin);
            gpio_low(VL53L1X3_obj->sda_pin);
        }
        temp >>= 1;

        VL53L1X_sync_delay(delay / 2);
        gpio_high(VL53L1X2_obj->scl_pin);
        gpio_high(VL53L1X3_obj->scl_pin);
        VL53L1X_sync_delay(delay);
        gpio_low(VL53L1X2_obj->scl_pin);
        gpio_low(VL53L1X3_obj->scl_pin);
        VL53L1X_sync_delay(delay / 2);
    }

    gpio_low(VL53L1X2_obj->scl_pin);
    gpio_low(VL53L1X3_obj->scl_pin);
    gpio_high(VL53L1X2_obj->sda_pin);
    gpio_high(VL53L1X3_obj->sda_pin);
    gpio_set_dir((gpio_pin_enum)VL53L1X2_obj->sda_pin, GPI, GPI_FLOATING_IN);
    gpio_set_dir((gpio_pin_enum)VL53L1X3_obj->sda_pin, GPI, GPI_FLOATING_IN);
    VL53L1X_sync_delay(delay);

    gpio_high(VL53L1X2_obj->scl_pin);
    gpio_high(VL53L1X3_obj->scl_pin);
    VL53L1X_sync_delay(delay);

    if(!gpio_get_level((gpio_pin_enum)VL53L1X2_obj->sda_pin))
    {
        ack_mask |= 0x01;
    }
    if(!gpio_get_level((gpio_pin_enum)VL53L1X3_obj->sda_pin))
    {
        ack_mask |= 0x02;
    }

    gpio_low(VL53L1X2_obj->scl_pin);
    gpio_low(VL53L1X3_obj->scl_pin);
    gpio_set_dir((gpio_pin_enum)VL53L1X2_obj->sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir((gpio_pin_enum)VL53L1X3_obj->sda_pin, GPO, GPO_OPEN_DTAIN);
    VL53L1X_sync_delay(delay);

    return ack_mask;
}

/**
 * @brief 双路IIC同时读取一个字节。
 * @param VL53L1X2_obj 2号软IIC对象。
 * @param VL53L1X3_obj 3号软IIC对象。
 * @param VL53L1X2_data 输出2号读到的数据。
 * @param VL53L1X3_data 输出3号读到的数据。
 * @param ack 读完后回送ACK控制，0=ACK，1=NACK。
 * @param delay 时序延时计数。
 * @return 无。
 */
static void VL53L1X_dual_iic_read_data(soft_iic_info_struct *VL53L1X2_obj, soft_iic_info_struct *VL53L1X3_obj, uint8 *VL53L1X2_data, uint8 *VL53L1X3_data, uint8 ack, uint32 delay)
{
    uint8 bit_count = 8;
    *VL53L1X2_data = 0;
    *VL53L1X3_data = 0;

    gpio_low(VL53L1X2_obj->scl_pin);
    gpio_low(VL53L1X3_obj->scl_pin);
    VL53L1X_sync_delay(delay);
    gpio_high(VL53L1X2_obj->sda_pin);
    gpio_high(VL53L1X3_obj->sda_pin);
    gpio_set_dir((gpio_pin_enum)VL53L1X2_obj->sda_pin, GPI, GPI_FLOATING_IN);
    gpio_set_dir((gpio_pin_enum)VL53L1X3_obj->sda_pin, GPI, GPI_FLOATING_IN);

    while(bit_count --)
    {
        gpio_low(VL53L1X2_obj->scl_pin);
        gpio_low(VL53L1X3_obj->scl_pin);
        VL53L1X_sync_delay(delay);
        gpio_high(VL53L1X2_obj->scl_pin);
        gpio_high(VL53L1X3_obj->scl_pin);
        VL53L1X_sync_delay(delay);
        *VL53L1X2_data = (uint8)((*VL53L1X2_data << 1) | gpio_get_level((gpio_pin_enum)VL53L1X2_obj->sda_pin));
        *VL53L1X3_data = (uint8)((*VL53L1X3_data << 1) | gpio_get_level((gpio_pin_enum)VL53L1X3_obj->sda_pin));
    }

    gpio_low(VL53L1X2_obj->scl_pin);
    gpio_low(VL53L1X3_obj->scl_pin);
    gpio_set_dir((gpio_pin_enum)VL53L1X2_obj->sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir((gpio_pin_enum)VL53L1X3_obj->sda_pin, GPO, GPO_OPEN_DTAIN);
    VL53L1X_sync_delay(delay);

    if(ack)
    {
        gpio_high(VL53L1X2_obj->sda_pin);
        gpio_high(VL53L1X3_obj->sda_pin);
    }
    else
    {
        gpio_low(VL53L1X2_obj->sda_pin);
        gpio_low(VL53L1X3_obj->sda_pin);
    }

    VL53L1X_sync_delay(delay);
    gpio_high(VL53L1X2_obj->scl_pin);
    gpio_high(VL53L1X3_obj->scl_pin);
    VL53L1X_sync_delay(delay);
    gpio_low(VL53L1X2_obj->scl_pin);
    gpio_low(VL53L1X3_obj->scl_pin);
    gpio_high(VL53L1X2_obj->sda_pin);
    gpio_high(VL53L1X3_obj->sda_pin);
}

/**
 * @brief 双路同步读取寄存器数组。
 * @param reg_addr 起始寄存器地址。
 * @param VL53L1X2_data 2号通道接收缓冲区。
 * @param VL53L1X3_data 3号通道接收缓冲区。
 * @param read_len 读取字节数。
 * @return ACK掩码：bit0对应2号，bit1对应3号。
 */
static uint8 VL53L1X_dual_read_reg_8bit_array(uint16 reg_addr, uint8 *VL53L1X2_data, uint8 *VL53L1X3_data, uint32 read_len)
{
    uint8 ack_mask = 0x03;
    uint8 index = 0;
    uint8 write_addr = (uint8)(DL1B_DEV_ADDR << 1);
    uint8 read_addr = (uint8)((DL1B_DEV_ADDR << 1) | 0x01);

    VL53L1X_dual_iic_start(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_dual_iic_send_data(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, write_addr, VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_dual_iic_send_data(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, (uint8)(reg_addr >> 8), VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_dual_iic_send_data(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, (uint8)(reg_addr & 0xFF), VL53L1X_SYNC_SOFT_IIC_DELAY);
    VL53L1X_dual_iic_start(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_dual_iic_send_data(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, read_addr, VL53L1X_SYNC_SOFT_IIC_DELAY);

    for(index = 0; index < read_len; index++)
    {
        VL53L1X_dual_iic_read_data(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, &VL53L1X2_data[index], &VL53L1X3_data[index], (index == (read_len - 1)), VL53L1X_SYNC_SOFT_IIC_DELAY);
    }

    VL53L1X_dual_iic_stop(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, VL53L1X_SYNC_SOFT_IIC_DELAY);

    return ack_mask;
}

/**
 * @brief 双路同步写入8位寄存器。
 * @param reg_addr 寄存器地址。
 * @param value 写入值。
 * @return ACK掩码：bit0对应2号，bit1对应3号。
 */
static uint8 VL53L1X_dual_write_reg_8bit(uint16 reg_addr, uint8 value)
{
    uint8 ack_mask = 0x03;
    uint8 write_addr = (uint8)(DL1B_DEV_ADDR << 1);

    VL53L1X_dual_iic_start(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_dual_iic_send_data(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, write_addr, VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_dual_iic_send_data(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, (uint8)(reg_addr >> 8), VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_dual_iic_send_data(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, (uint8)(reg_addr & 0xFF), VL53L1X_SYNC_SOFT_IIC_DELAY);
    ack_mask &= VL53L1X_dual_iic_send_data(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, value, VL53L1X_SYNC_SOFT_IIC_DELAY);
    VL53L1X_dual_iic_stop(&VL53L1X2_iic_struct, &VL53L1X3_iic_struct, VL53L1X_SYNC_SOFT_IIC_DELAY);

    return ack_mask;
}

/**
 * @brief 单通道VL53L1X初始化流程（含总线恢复与固件配置下载）。
 * @param VL53L1X_iic_struct 软IIC对象。
 * @param init_flag 初始化结果标志输出指针。
 * @param soft_iic_delay_base 软IIC基础延时。
 * @param soft_iic_delay_step 软IIC延时步长。
 * @param soft_iic_delay_max 软IIC延时上限。
 * @param scl_pin SCL引脚。
 * @param sda_pin SDA引脚。
 * @return 0=初始化成功，1=初始化失败。
 */
static uint8 VL53L1X_init_internal(soft_iic_info_struct *VL53L1X_iic_struct,
                               uint8 *init_flag,
                               uint32 soft_iic_delay_base,
                               uint32 soft_iic_delay_step,
                               uint32 soft_iic_delay_max,
                               gpio_pin_enum scl_pin,
                               gpio_pin_enum sda_pin)
{
    uint8   return_state    = 0;
    uint8   retry_count     = 0;
    uint8   addr_ack_retry  = 0;
    uint8   addr_ack        = 0;
    uint8   scan_first_addr = 0xFF;
    uint8   data_buffer[2 + sizeof(dl1b_config_file)];
    uint16  time_out_count  = 0;
    uint32  soft_iic_delay  = soft_iic_delay_base;

    *init_flag = 0;
    data_buffer[2] = 0xFF;

    /* 总流程：先链路握手，再下载配置，最后等待中断就绪。 */
    do
    {
        /* 总线故障自恢复：ACK探测失败时执行总线脉冲恢复并递增延时重试。 */
        for(retry_count = 0; retry_count <= DL1B_BUS_FAULT_RETRY_COUNT; retry_count++)
        {
            VL53L1X_prepare_bus(scl_pin, sda_pin);
            soft_iic_init(VL53L1X_iic_struct, DL1B_DEV_ADDR, soft_iic_delay, scl_pin, sda_pin);
            VL53L1X_soft_iic_pin_config(scl_pin, sda_pin);
            addr_ack = 0;
            for(addr_ack_retry = 0; addr_ack_retry < DL1B_ADDR_ACK_RETRY_COUNT; addr_ack_retry++)
            {
                if(soft_iic_probe_ack(VL53L1X_iic_struct, DL1B_DEV_ADDR))
                {
                    addr_ack = 1;
                    break;
                }
                system_delay_ms(1);
            }
            if(0 == addr_ack)
            {
                data_buffer[2] = 0xFF;
                scan_first_addr = VL53L1X_scan_first_ack_addr(VL53L1X_iic_struct);
                if(DL1B_DEV_ADDR == scan_first_addr)
                {
                    addr_ack = 1;
                    system_delay_ms(10);
                }
                else
                {
                    VL53L1X_bus_recovery(scl_pin, sda_pin);
                    if((soft_iic_delay + soft_iic_delay_step) <= soft_iic_delay_max)
                    {
                        soft_iic_delay += soft_iic_delay_step;
                    }
                    system_delay_ms(10);
                    continue;
                }
            }

            system_delay_ms(50);
            system_delay_ms(50);

            data_buffer[2] = VL53L1X_read_fw_status_with_retry(VL53L1X_iic_struct);
            if(0xFF != data_buffer[2])
            {
                break;
            }

            VL53L1X_bus_recovery(scl_pin, sda_pin);
            if((soft_iic_delay + soft_iic_delay_step) <= soft_iic_delay_max)
            {
                soft_iic_delay += soft_iic_delay_step;
            }
            system_delay_ms(10);
        }

        /* 固件状态不可读时判定初始化失败。 */
        if(0xFF == data_buffer[2])
        {
            return_state = 1;
            break;
        }

        return_state = (0x01 == (data_buffer[2] & 0x01)) ? (0) : (1);
        if(1 == return_state)
        {
            break;
        }

        /* 下载官方配置表到设备寄存器空间。 */
        data_buffer[0] = DL1B_I2C_SLAVE__DEVICE_ADDRESS >> 8;
        data_buffer[1] = DL1B_I2C_SLAVE__DEVICE_ADDRESS & 0xFF;
        memcpy(&data_buffer[2], (uint8 *)dl1b_config_file, sizeof(dl1b_config_file));
        soft_iic_transfer_8bit_array(VL53L1X_iic_struct, data_buffer, 2 + sizeof(dl1b_config_file), data_buffer, 0);

        /* 轮询数据就绪中断，超时则判定失败。 */
        while(1)
        {
            VL53L1X_read_reg_8bit(VL53L1X_iic_struct, DL1B_GPIO__TIO_HV_STATUS, &data_buffer[2]);
            if(0x00 == (data_buffer[2] & 0x01))
            {
                time_out_count = 0;
                break;
            }
            if(DL1B_TIMEOUT_COUNT < time_out_count ++)
            {
                return_state = 1;
                break;
            }
            system_delay_ms(1);
        }

        if(0 == return_state)
        {
            *init_flag = 1;
        }
    }while(0);

    return return_state;
}

/**
 * @brief 初始化2号VL53L1X通道。
 * @param 无。
 * @return 0=成功，1=失败。
 */
static uint8 VL53L1X2_init(void)
{
    return VL53L1X_init_internal(&VL53L1X2_iic_struct,
                             &VL53L1X2_init_flag,
                             VL53L1X23_SOFT_IIC_DELAY_BASE,
                             VL53L1X23_SOFT_IIC_DELAY_STEP,
                             VL53L1X23_SOFT_IIC_DELAY_MAX,
                             VL53L1X2_SCL_PIN,
                             VL53L1X2_SDA_PIN);
}

/**
 * @brief 初始化3号VL53L1X通道。
 * @param 无。
 * @return 0=成功，1=失败。
 */
static uint8 VL53L1X3_init(void)
{
    return VL53L1X_init_internal(&VL53L1X3_iic_struct,
                             &VL53L1X3_init_flag,
                             VL53L1X23_SOFT_IIC_DELAY_BASE,
                             VL53L1X23_SOFT_IIC_DELAY_STEP,
                             VL53L1X23_SOFT_IIC_DELAY_MAX,
                             VL53L1X3_SCL_PIN,
                             VL53L1X3_SDA_PIN);
}

/**
 * @brief 初始化双路VL53L1X并清理诊断状态。
 * @param 无。
 * @return 错误位掩码：bit0=2号失败，bit1=3号失败；0表示全部成功。
 */
uint8 VL53L1X_init_all(void)
{
    uint8 err = 0;

    memset(&g_vl53l1x2_diag, 0, sizeof(g_vl53l1x2_diag));
    memset(&g_vl53l1x3_diag, 0, sizeof(g_vl53l1x3_diag));
    s_vl53l1x2_has_last_distance = 0U;
    s_vl53l1x3_has_last_distance = 0U;
    s_vl53l1x2_last_ready = 0U;
    s_vl53l1x3_last_ready = 0U;

    /* 分别初始化2号与3号通道，失败信息打包到err位图。 */
    if (VL53L1X2_init())
    {
        err |= (1U << 0);
    }

    if (VL53L1X3_init())
    {
        err |= (1U << 1);
    }

    return err;
}

/**
 * @brief 每帧前清空诊断瞬时状态位。
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
 * @brief 依据ACK结果更新通信健康计数。
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
 * @brief 根据ready、量测有效性与重复值判定数据新鲜度。
 * @param diag 诊断结构体指针。
 * @param ready 当前帧ready标志。
 * @param distance_valid 当前帧距离是否有效。
 * @param distance_mm 当前帧距离（单位：mm）。
 * @param last_distance 上一有效距离缓存指针（单位：mm）。
 * @param has_last_distance 是否已存在上一有效距离。
 * @param last_ready 上一帧ready缓存。
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
 * @brief 在通信失败或未初始化时尝试重建双路通道。
 * @param 无。
 * @return 无。
 */
static void VL53L1X_TryRecoverChannels(void)
{
    if ((g_vl53l1x2_diag.comm_fail_count >= VL53L1X_COMM_FAIL_REINIT_TH) || (VL53L1X2_init_flag == 0U))
    {
        if (0U == VL53L1X2_init())
        {
            g_vl53l1x2_diag.comm_fail_count = 0U;
            g_vl53l1x2_diag.stale_count = 0U;
            s_vl53l1x2_has_last_distance = 0U;
            s_vl53l1x2_last_ready = 0U;
        }
    }

    if ((g_vl53l1x3_diag.comm_fail_count >= VL53L1X_COMM_FAIL_REINIT_TH) || (VL53L1X3_init_flag == 0U))
    {
        if (0U == VL53L1X3_init())
        {
            g_vl53l1x3_diag.comm_fail_count = 0U;
            g_vl53l1x3_diag.stale_count = 0U;
            s_vl53l1x3_has_last_distance = 0U;
            s_vl53l1x3_last_ready = 0U;
        }
    }
}

/**
 * @brief 10Hz维护接口：尝试链路恢复。
 * @param 无。
 * @return 无。
 */
void VL53L1X_recover_update_10HZ(void)
{
    VL53L1X_TryRecoverChannels();
}

/**
 * @brief 读取双路VL53L1X数据并输出有效位掩码。
 * @param data 输出数据指针。
 * @return 有效新鲜位掩码：bit0=2号有效，bit1=3号有效。
 */
uint8 VL53L1X_read_data(VL53L1X_data_struct *data)
{
    uint8 valid_mask = 0U;
    uint8 ack_mask;
    uint8 ack2;
    uint8 ack3;
    uint8 VL53L1X2_ready;
    uint8 VL53L1X3_ready;
    uint8 VL53L1X2_tio_status = 0U;
    uint8 VL53L1X3_tio_status = 0U;
    uint8 VL53L1X2_range_status = 0xFFU;
    uint8 VL53L1X3_range_status = 0xFFU;
    uint8 VL53L1X2_distance_buffer[2] = {0xFFU, 0xFFU};
    uint8 VL53L1X3_distance_buffer[2] = {0xFFU, 0xFFU};
    uint16 VL53L1X2_distance_temp = VL53L1X_INVALID_DISTANCE_MM;
    uint16 VL53L1X3_distance_temp = VL53L1X_INVALID_DISTANCE_MM;
    uint8 ch2_distance_valid = 0U;
    uint8 ch3_distance_valid = 0U;

    if (NULL == data)
    {
        return 0U;
    }

    /* 每帧先清空诊断瞬时位并设置输出默认无效值。 */
    VL53L1X_DiagPrepareFrame(&g_vl53l1x2_diag);
    VL53L1X_DiagPrepareFrame(&g_vl53l1x3_diag);

    data->VL53L1X2_distance_mm = VL53L1X_INVALID_DISTANCE_MM;
    data->VL53L1X3_distance_mm = VL53L1X_INVALID_DISTANCE_MM;
    data->VL53L1X2_range_status = 0xFFU;
    data->VL53L1X3_range_status = 0xFFU;

    if ((VL53L1X2_init_flag == 0U) && (VL53L1X3_init_flag == 0U))
    {
        VL53L1X_data = *data;
        return 0U;
    }

    /* 双路同步读取ready信号并更新通信诊断。 */
    VL53L1X_soft_iic_pin_config(VL53L1X2_SCL_PIN, VL53L1X2_SDA_PIN);
    VL53L1X_soft_iic_pin_config(VL53L1X3_SCL_PIN, VL53L1X3_SDA_PIN);

    ack_mask = VL53L1X_dual_read_reg_8bit_array(DL1B_GPIO__TIO_HV_STATUS, &VL53L1X2_tio_status, &VL53L1X3_tio_status, 1U);
    ack2 = (ack_mask & 0x01U) ? 1U : 0U;
    ack3 = (ack_mask & 0x02U) ? 1U : 0U;
    VL53L1X_DiagUpdateComm(&g_vl53l1x2_diag, ack2);
    VL53L1X_DiagUpdateComm(&g_vl53l1x3_diag, ack3);

    if (0U == ack2)
    {
        VL53L1X2_tio_status = 0U;
    }

    if (0U == ack3)
    {
        VL53L1X3_tio_status = 0U;
    }

    VL53L1X2_ready = ((VL53L1X2_init_flag != 0U) && (VL53L1X2_tio_status != 0U)) ? 1U : 0U;
    VL53L1X3_ready = ((VL53L1X3_init_flag != 0U) && (VL53L1X3_tio_status != 0U)) ? 1U : 0U;
    g_vl53l1x2_diag.ready = VL53L1X2_ready;
    g_vl53l1x3_diag.ready = VL53L1X3_ready;

    /* 至少一路ready时，读取状态与距离寄存器。 */
    if ((VL53L1X2_ready != 0U) || (VL53L1X3_ready != 0U))
    {
        ack_mask = VL53L1X_dual_read_reg_8bit_array(DL1B_RESULT__RANGE_STATUS, &VL53L1X2_range_status, &VL53L1X3_range_status, 1U);
        ack2 = (ack_mask & 0x01U) ? 1U : 0U;
        ack3 = (ack_mask & 0x02U) ? 1U : 0U;
        VL53L1X_DiagUpdateComm(&g_vl53l1x2_diag, ack2);
        VL53L1X_DiagUpdateComm(&g_vl53l1x3_diag, ack3);

        if ((0U == ack2) || (0U == VL53L1X2_ready))
        {
            VL53L1X2_range_status = 0xFFU;
        }

        if ((0U == ack3) || (0U == VL53L1X3_ready))
        {
            VL53L1X3_range_status = 0xFFU;
        }

        ack_mask = VL53L1X_dual_read_reg_8bit_array(DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,
                                                    VL53L1X2_distance_buffer,
                                                    VL53L1X3_distance_buffer,
                                                    2U);
        ack2 = (ack_mask & 0x01U) ? 1U : 0U;
        ack3 = (ack_mask & 0x02U) ? 1U : 0U;
        VL53L1X_DiagUpdateComm(&g_vl53l1x2_diag, ack2);
        VL53L1X_DiagUpdateComm(&g_vl53l1x3_diag, ack3);

        if ((0U == ack2) || (0U == VL53L1X2_ready))
        {
            VL53L1X2_distance_buffer[0] = 0xFFU;
            VL53L1X2_distance_buffer[1] = 0xFFU;
        }

        if ((0U == ack3) || (0U == VL53L1X3_ready))
        {
            VL53L1X3_distance_buffer[0] = 0xFFU;
            VL53L1X3_distance_buffer[1] = 0xFFU;
        }
    }

    /* 按range_status和量程门限筛选有效距离，并执行上限裁剪。 */
    if ((VL53L1X2_ready != 0U) && (0x89U == VL53L1X2_range_status))
    {
        VL53L1X2_distance_temp = (uint16)(((uint16)VL53L1X2_distance_buffer[0] << 8) | VL53L1X2_distance_buffer[1]);
        if (VL53L1X2_distance_temp <= 4000U)
        {
            if (VL53L1X2_distance_temp > VL53L1X_VALID_RANGE_MAX)
            {
                VL53L1X2_distance_temp = (uint16)VL53L1X_VALID_RANGE_MAX;
            }

            data->VL53L1X2_distance_mm = VL53L1X2_distance_temp;
            ch2_distance_valid = 1U;
            g_vl53l1x2_diag.range_ok = 1U;
        }
    }

    if ((VL53L1X3_ready != 0U) && (0x89U == VL53L1X3_range_status))
    {
        VL53L1X3_distance_temp = (uint16)(((uint16)VL53L1X3_distance_buffer[0] << 8) | VL53L1X3_distance_buffer[1]);
        if (VL53L1X3_distance_temp <= 4000U)
        {
            if (VL53L1X3_distance_temp > VL53L1X_VALID_RANGE_MAX)
            {
                VL53L1X3_distance_temp = (uint16)VL53L1X_VALID_RANGE_MAX;
            }

            data->VL53L1X3_distance_mm = VL53L1X3_distance_temp;
            ch3_distance_valid = 1U;
            g_vl53l1x3_diag.range_ok = 1U;
        }
    }

    /* 结合上一帧信息判定数据新鲜度并生成有效位。 */
    if (0U != VL53L1X_UpdateFreshness(&g_vl53l1x2_diag,
                                      VL53L1X2_ready,
                                      ch2_distance_valid,
                                      data->VL53L1X2_distance_mm,
                                      &s_vl53l1x2_last_distance,
                                      &s_vl53l1x2_has_last_distance,
                                      &s_vl53l1x2_last_ready))
    {
        valid_mask |= (1U << 0);
    }

    if (0U != VL53L1X_UpdateFreshness(&g_vl53l1x3_diag,
                                      VL53L1X3_ready,
                                      ch3_distance_valid,
                                      data->VL53L1X3_distance_mm,
                                      &s_vl53l1x3_last_distance,
                                      &s_vl53l1x3_has_last_distance,
                                      &s_vl53l1x3_last_ready))
    {
        valid_mask |= (1U << 1);
    }

    data->VL53L1X2_range_status = VL53L1X2_range_status;
    data->VL53L1X3_range_status = VL53L1X3_range_status;

    /* 本帧有ready通道时，统一清除设备中断。 */
    if ((VL53L1X2_ready != 0U) || (VL53L1X3_ready != 0U))
    {
        ack_mask = VL53L1X_dual_write_reg_8bit(DL1B_SYSTEM__INTERRUPT_CLEAR, 0x01U);
        VL53L1X_DiagUpdateComm(&g_vl53l1x2_diag, (ack_mask & 0x01U) ? 1U : 0U);
        VL53L1X_DiagUpdateComm(&g_vl53l1x3_diag, (ack_mask & 0x02U) ? 1U : 0U);
    }

    /* 更新模块级缓存，供外部快速访问。 */
    VL53L1X_data = *data;
    return valid_mask;
}
