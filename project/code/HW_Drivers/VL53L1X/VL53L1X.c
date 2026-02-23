#include "VL53L1X.h"
#include "zf_driver_delay.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_gpio.h"
#include "zf_device_config.h"
#include <string.h>

#define DL1B_SOFT_IIC_DELAY                                     ( 100 )
#define DL1B_TIMEOUT_COUNT                                      ( 1000 )
#define DL1B_DEV_ADDR                                           ( 0x52 >> 1 )
#define DL1B_BUS_FAULT_RETRY_COUNT                              ( 5 )
#define DL1B_BUS_RECOVERY_PULSE_COUNT                           ( 9 )
#define DL1B_SOFT_IIC_DELAY_STEP                                ( 100 )
#define DL1B_SOFT_IIC_DELAY_MAX                                 ( 500 )
#define DL1B_ADDR_ACK_RETRY_COUNT                               ( 3 )
#define DL1B_FW_STATUS_READ_RETRY_COUNT                          ( 5 )
#define TOF23_SOFT_IIC_DELAY_BASE                               ( DL1B_SOFT_IIC_DELAY )
#define TOF23_SOFT_IIC_DELAY_STEP                               ( DL1B_SOFT_IIC_DELAY_STEP )
#define TOF23_SOFT_IIC_DELAY_MAX                                ( DL1B_SOFT_IIC_DELAY_MAX )
#define TOF_SYNC_SOFT_IIC_DELAY                                 ( TOF23_SOFT_IIC_DELAY_BASE )

#define DL1B_I2C_SLAVE__DEVICE_ADDRESS                          ( 0x0001 )
#define DL1B_GPIO__TIO_HV_STATUS                                ( 0x0031 )
#define DL1B_SYSTEM__INTERRUPT_CLEAR                            ( 0x0086 )
#define DL1B_RESULT__RANGE_STATUS                               ( 0x0089 )
#define DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0     ( 0x0096 )
#define DL1B_FIRMWARE__SYSTEM_STATUS                            ( 0x00E5 )

static soft_iic_info_struct tof2_iic_struct;
static soft_iic_info_struct tof3_iic_struct;

static uint8 tof2_init_flag = 0;
static uint8 tof3_init_flag = 0;

tof_data_struct tof_data =
{
    TOF_INVALID_DISTANCE_MM,
    TOF_INVALID_DISTANCE_MM,
    0xFF,
    0xFF
};

static void tof_soft_iic_pin_config(gpio_pin_enum scl_pin, gpio_pin_enum sda_pin)
{
    gpio_set_dir(scl_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir(sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_high(scl_pin);
    gpio_high(sda_pin);
}

static void tof_prepare_bus(gpio_pin_enum scl_pin, gpio_pin_enum sda_pin)
{
    gpio_init(scl_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    gpio_init(sda_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    gpio_set_dir(scl_pin, GPI, GPI_PULL_UP);
    gpio_set_dir(sda_pin, GPI, GPI_PULL_UP);
    system_delay_ms(1);
}

static void tof_bus_recovery(gpio_pin_enum scl_pin, gpio_pin_enum sda_pin)
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

static uint8 tof_scan_first_ack_addr(soft_iic_info_struct *tof_iic_struct)
{
    uint8 scan_addr = 0xFF;
    uint8 addr = 0;

    for(addr = 0x08; addr <= 0x77; addr++)
    {
        if(soft_iic_probe_ack(tof_iic_struct, addr))
        {
            scan_addr = addr;
            break;
        }
    }

    return scan_addr;
}

static void tof_read_reg_8bit(soft_iic_info_struct *tof_iic_struct, uint16 reg_addr, uint8 *reg_data)
{
    soft_iic_write_16bit(tof_iic_struct, reg_addr);
    *reg_data = soft_iic_read_8bit(tof_iic_struct);
}

static uint8 tof_read_fw_status_with_retry(soft_iic_info_struct *tof_iic_struct)
{
    uint8 fw_status = 0xFF;
    uint8 retry_count = 0;

    for(retry_count = 0; retry_count < DL1B_FW_STATUS_READ_RETRY_COUNT; retry_count++)
    {
        tof_read_reg_8bit(tof_iic_struct, DL1B_FIRMWARE__SYSTEM_STATUS, &fw_status);
        if(0xFF != fw_status)
        {
            break;
        }
        system_delay_ms(1);
    }

    return fw_status;
}

static void tof_sync_delay(uint32 delay)
{
    volatile uint32 count = delay;
    while(count --);
}

static void tof_dual_iic_start(soft_iic_info_struct *tof2_obj, soft_iic_info_struct *tof3_obj, uint32 delay)
{
    gpio_high(tof2_obj->scl_pin);
    gpio_high(tof3_obj->scl_pin);
    gpio_high(tof2_obj->sda_pin);
    gpio_high(tof3_obj->sda_pin);

    tof_sync_delay(delay);
    gpio_low(tof2_obj->sda_pin);
    gpio_low(tof3_obj->sda_pin);
    tof_sync_delay(delay);
    gpio_low(tof2_obj->scl_pin);
    gpio_low(tof3_obj->scl_pin);
    tof_sync_delay(delay);
}

static void tof_dual_iic_stop(soft_iic_info_struct *tof2_obj, soft_iic_info_struct *tof3_obj, uint32 delay)
{
    gpio_low(tof2_obj->sda_pin);
    gpio_low(tof3_obj->sda_pin);
    gpio_low(tof2_obj->scl_pin);
    gpio_low(tof3_obj->scl_pin);

    tof_sync_delay(delay);
    gpio_high(tof2_obj->scl_pin);
    gpio_high(tof3_obj->scl_pin);
    tof_sync_delay(delay);
    gpio_high(tof2_obj->sda_pin);
    gpio_high(tof3_obj->sda_pin);
    tof_sync_delay(delay);
}

static uint8 tof_dual_iic_send_data(soft_iic_info_struct *tof2_obj, soft_iic_info_struct *tof3_obj, uint8 data, uint32 delay)
{
    uint8 temp = 0x80;
    uint8 ack_mask = 0;

    while(temp)
    {
        if(data & temp)
        {
            gpio_high(tof2_obj->sda_pin);
            gpio_high(tof3_obj->sda_pin);
        }
        else
        {
            gpio_low(tof2_obj->sda_pin);
            gpio_low(tof3_obj->sda_pin);
        }
        temp >>= 1;

        tof_sync_delay(delay / 2);
        gpio_high(tof2_obj->scl_pin);
        gpio_high(tof3_obj->scl_pin);
        tof_sync_delay(delay);
        gpio_low(tof2_obj->scl_pin);
        gpio_low(tof3_obj->scl_pin);
        tof_sync_delay(delay / 2);
    }

    gpio_low(tof2_obj->scl_pin);
    gpio_low(tof3_obj->scl_pin);
    gpio_high(tof2_obj->sda_pin);
    gpio_high(tof3_obj->sda_pin);
    gpio_set_dir((gpio_pin_enum)tof2_obj->sda_pin, GPI, GPI_FLOATING_IN);
    gpio_set_dir((gpio_pin_enum)tof3_obj->sda_pin, GPI, GPI_FLOATING_IN);
    tof_sync_delay(delay);

    gpio_high(tof2_obj->scl_pin);
    gpio_high(tof3_obj->scl_pin);
    tof_sync_delay(delay);

    if(!gpio_get_level((gpio_pin_enum)tof2_obj->sda_pin))
    {
        ack_mask |= 0x01;
    }
    if(!gpio_get_level((gpio_pin_enum)tof3_obj->sda_pin))
    {
        ack_mask |= 0x02;
    }

    gpio_low(tof2_obj->scl_pin);
    gpio_low(tof3_obj->scl_pin);
    gpio_set_dir((gpio_pin_enum)tof2_obj->sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir((gpio_pin_enum)tof3_obj->sda_pin, GPO, GPO_OPEN_DTAIN);
    tof_sync_delay(delay);

    return ack_mask;
}

static void tof_dual_iic_read_data(soft_iic_info_struct *tof2_obj, soft_iic_info_struct *tof3_obj, uint8 *tof2_data, uint8 *tof3_data, uint8 ack, uint32 delay)
{
    uint8 bit_count = 8;
    *tof2_data = 0;
    *tof3_data = 0;

    gpio_low(tof2_obj->scl_pin);
    gpio_low(tof3_obj->scl_pin);
    tof_sync_delay(delay);
    gpio_high(tof2_obj->sda_pin);
    gpio_high(tof3_obj->sda_pin);
    gpio_set_dir((gpio_pin_enum)tof2_obj->sda_pin, GPI, GPI_FLOATING_IN);
    gpio_set_dir((gpio_pin_enum)tof3_obj->sda_pin, GPI, GPI_FLOATING_IN);

    while(bit_count --)
    {
        gpio_low(tof2_obj->scl_pin);
        gpio_low(tof3_obj->scl_pin);
        tof_sync_delay(delay);
        gpio_high(tof2_obj->scl_pin);
        gpio_high(tof3_obj->scl_pin);
        tof_sync_delay(delay);
        *tof2_data = (uint8)((*tof2_data << 1) | gpio_get_level((gpio_pin_enum)tof2_obj->sda_pin));
        *tof3_data = (uint8)((*tof3_data << 1) | gpio_get_level((gpio_pin_enum)tof3_obj->sda_pin));
    }

    gpio_low(tof2_obj->scl_pin);
    gpio_low(tof3_obj->scl_pin);
    gpio_set_dir((gpio_pin_enum)tof2_obj->sda_pin, GPO, GPO_OPEN_DTAIN);
    gpio_set_dir((gpio_pin_enum)tof3_obj->sda_pin, GPO, GPO_OPEN_DTAIN);
    tof_sync_delay(delay);

    if(ack)
    {
        gpio_high(tof2_obj->sda_pin);
        gpio_high(tof3_obj->sda_pin);
    }
    else
    {
        gpio_low(tof2_obj->sda_pin);
        gpio_low(tof3_obj->sda_pin);
    }

    tof_sync_delay(delay);
    gpio_high(tof2_obj->scl_pin);
    gpio_high(tof3_obj->scl_pin);
    tof_sync_delay(delay);
    gpio_low(tof2_obj->scl_pin);
    gpio_low(tof3_obj->scl_pin);
    gpio_high(tof2_obj->sda_pin);
    gpio_high(tof3_obj->sda_pin);
}

static uint8 tof_dual_read_reg_8bit_array(uint16 reg_addr, uint8 *tof2_data, uint8 *tof3_data, uint32 read_len)
{
    uint8 ack_mask = 0x03;
    uint8 index = 0;
    uint8 write_addr = (uint8)(DL1B_DEV_ADDR << 1);
    uint8 read_addr = (uint8)((DL1B_DEV_ADDR << 1) | 0x01);

    tof_dual_iic_start(&tof2_iic_struct, &tof3_iic_struct, TOF_SYNC_SOFT_IIC_DELAY);
    ack_mask &= tof_dual_iic_send_data(&tof2_iic_struct, &tof3_iic_struct, write_addr, TOF_SYNC_SOFT_IIC_DELAY);
    ack_mask &= tof_dual_iic_send_data(&tof2_iic_struct, &tof3_iic_struct, (uint8)(reg_addr >> 8), TOF_SYNC_SOFT_IIC_DELAY);
    ack_mask &= tof_dual_iic_send_data(&tof2_iic_struct, &tof3_iic_struct, (uint8)(reg_addr & 0xFF), TOF_SYNC_SOFT_IIC_DELAY);
    tof_dual_iic_start(&tof2_iic_struct, &tof3_iic_struct, TOF_SYNC_SOFT_IIC_DELAY);
    ack_mask &= tof_dual_iic_send_data(&tof2_iic_struct, &tof3_iic_struct, read_addr, TOF_SYNC_SOFT_IIC_DELAY);

    for(index = 0; index < read_len; index++)
    {
        tof_dual_iic_read_data(&tof2_iic_struct, &tof3_iic_struct, &tof2_data[index], &tof3_data[index], (index == (read_len - 1)), TOF_SYNC_SOFT_IIC_DELAY);
    }

    tof_dual_iic_stop(&tof2_iic_struct, &tof3_iic_struct, TOF_SYNC_SOFT_IIC_DELAY);

    return ack_mask;
}

static uint8 tof_dual_write_reg_8bit(uint16 reg_addr, uint8 value)
{
    uint8 ack_mask = 0x03;
    uint8 write_addr = (uint8)(DL1B_DEV_ADDR << 1);

    tof_dual_iic_start(&tof2_iic_struct, &tof3_iic_struct, TOF_SYNC_SOFT_IIC_DELAY);
    ack_mask &= tof_dual_iic_send_data(&tof2_iic_struct, &tof3_iic_struct, write_addr, TOF_SYNC_SOFT_IIC_DELAY);
    ack_mask &= tof_dual_iic_send_data(&tof2_iic_struct, &tof3_iic_struct, (uint8)(reg_addr >> 8), TOF_SYNC_SOFT_IIC_DELAY);
    ack_mask &= tof_dual_iic_send_data(&tof2_iic_struct, &tof3_iic_struct, (uint8)(reg_addr & 0xFF), TOF_SYNC_SOFT_IIC_DELAY);
    ack_mask &= tof_dual_iic_send_data(&tof2_iic_struct, &tof3_iic_struct, value, TOF_SYNC_SOFT_IIC_DELAY);
    tof_dual_iic_stop(&tof2_iic_struct, &tof3_iic_struct, TOF_SYNC_SOFT_IIC_DELAY);

    return ack_mask;
}

static uint8 tof_init_internal(soft_iic_info_struct *tof_iic_struct,
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

    do
    {
        for(retry_count = 0; retry_count <= DL1B_BUS_FAULT_RETRY_COUNT; retry_count++)
        {
            tof_prepare_bus(scl_pin, sda_pin);
            soft_iic_init(tof_iic_struct, DL1B_DEV_ADDR, soft_iic_delay, scl_pin, sda_pin);
            tof_soft_iic_pin_config(scl_pin, sda_pin);
            addr_ack = 0;
            for(addr_ack_retry = 0; addr_ack_retry < DL1B_ADDR_ACK_RETRY_COUNT; addr_ack_retry++)
            {
                if(soft_iic_probe_ack(tof_iic_struct, DL1B_DEV_ADDR))
                {
                    addr_ack = 1;
                    break;
                }
                system_delay_ms(1);
            }
            if(0 == addr_ack)
            {
                data_buffer[2] = 0xFF;
                scan_first_addr = tof_scan_first_ack_addr(tof_iic_struct);
                if(DL1B_DEV_ADDR == scan_first_addr)
                {
                    addr_ack = 1;
                    system_delay_ms(10);
                }
                else
                {
                    tof_bus_recovery(scl_pin, sda_pin);
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

            data_buffer[2] = tof_read_fw_status_with_retry(tof_iic_struct);
            if(0xFF != data_buffer[2])
            {
                break;
            }

            tof_bus_recovery(scl_pin, sda_pin);
            if((soft_iic_delay + soft_iic_delay_step) <= soft_iic_delay_max)
            {
                soft_iic_delay += soft_iic_delay_step;
            }
            system_delay_ms(10);
        }

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

        data_buffer[0] = DL1B_I2C_SLAVE__DEVICE_ADDRESS >> 8;
        data_buffer[1] = DL1B_I2C_SLAVE__DEVICE_ADDRESS & 0xFF;
        memcpy(&data_buffer[2], (uint8 *)dl1b_config_file, sizeof(dl1b_config_file));
        soft_iic_transfer_8bit_array(tof_iic_struct, data_buffer, 2 + sizeof(dl1b_config_file), data_buffer, 0);

        while(1)
        {
            tof_read_reg_8bit(tof_iic_struct, DL1B_GPIO__TIO_HV_STATUS, &data_buffer[2]);
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

static uint8 tof2_init(void)
{
    return tof_init_internal(&tof2_iic_struct,
                             &tof2_init_flag,
                             TOF23_SOFT_IIC_DELAY_BASE,
                             TOF23_SOFT_IIC_DELAY_STEP,
                             TOF23_SOFT_IIC_DELAY_MAX,
                             TOF2_SCL_PIN,
                             TOF2_SDA_PIN);
}

static uint8 tof3_init(void)
{
    return tof_init_internal(&tof3_iic_struct,
                             &tof3_init_flag,
                             TOF23_SOFT_IIC_DELAY_BASE,
                             TOF23_SOFT_IIC_DELAY_STEP,
                             TOF23_SOFT_IIC_DELAY_MAX,
                             TOF3_SCL_PIN,
                             TOF3_SDA_PIN);
}

uint8 tof_init_all(void)
{
    uint8 err = 0;
    if(tof2_init()) { err |= (1U << 0); }
    if(tof3_init()) { err |= (1U << 1); }
    return err;
}

uint8 tof_read_data(tof_data_struct *data)
{
    uint8 valid_mask = 0;
    uint8 ack_mask = 0;
    uint8 tof2_ready = 0;
    uint8 tof3_ready = 0;
    uint8 tof2_tio_status = 0;
    uint8 tof3_tio_status = 0;
    uint8 tof2_range_status = 0xFF;
    uint8 tof3_range_status = 0xFF;
    uint8 tof2_distance_buffer[2] = {0xFF, 0xFF};
    uint8 tof3_distance_buffer[2] = {0xFF, 0xFF};
    uint16 tof2_distance_temp = TOF_INVALID_DISTANCE_MM;
    uint16 tof3_distance_temp = TOF_INVALID_DISTANCE_MM;

    if(NULL == data)
    {
        return 0;
    }

    data->tof2_distance_mm = TOF_INVALID_DISTANCE_MM;
    data->tof3_distance_mm = TOF_INVALID_DISTANCE_MM;
    data->tof2_range_status = 0xFF;
    data->tof3_range_status = 0xFF;

    if(!(tof2_init_flag || tof3_init_flag))
    {
        tof_data = *data;
        return 0;
    }

    tof_soft_iic_pin_config(TOF2_SCL_PIN, TOF2_SDA_PIN);
    tof_soft_iic_pin_config(TOF3_SCL_PIN, TOF3_SDA_PIN);

    ack_mask = tof_dual_read_reg_8bit_array(DL1B_GPIO__TIO_HV_STATUS, &tof2_tio_status, &tof3_tio_status, 1);
    if(!(ack_mask & 0x01))
    {
        tof2_tio_status = 0;
    }
    if(!(ack_mask & 0x02))
    {
        tof3_tio_status = 0;
    }

    tof2_ready = (tof2_init_flag && (0 != tof2_tio_status)) ? 1 : 0;
    tof3_ready = (tof3_init_flag && (0 != tof3_tio_status)) ? 1 : 0;

    if(!(tof2_ready || tof3_ready))
    {
        tof_data = *data;
        return 0;
    }

    (void)tof_dual_write_reg_8bit(DL1B_SYSTEM__INTERRUPT_CLEAR, 0x01);

    ack_mask = tof_dual_read_reg_8bit_array(DL1B_RESULT__RANGE_STATUS, &tof2_range_status, &tof3_range_status, 1);
    if(!(ack_mask & 0x01))
    {
        tof2_range_status = 0xFF;
    }
    if(!(ack_mask & 0x02))
    {
        tof3_range_status = 0xFF;
    }

    ack_mask = tof_dual_read_reg_8bit_array(DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, tof2_distance_buffer, tof3_distance_buffer, 2);
    if(!(ack_mask & 0x01))
    {
        tof2_distance_buffer[0] = 0xFF;
        tof2_distance_buffer[1] = 0xFF;
    }
    if(!(ack_mask & 0x02))
    {
        tof3_distance_buffer[0] = 0xFF;
        tof3_distance_buffer[1] = 0xFF;
    }

    if(tof2_ready && (0x89 == tof2_range_status))
    {
        tof2_distance_temp = (uint16)(((uint16)tof2_distance_buffer[0] << 8) | tof2_distance_buffer[1]);
        if(tof2_distance_temp <= 4000)
        {
            if(tof2_distance_temp > TOF_VALID_RANGE_MAX)
            {
                tof2_distance_temp = (uint16)TOF_VALID_RANGE_MAX;
            }
            data->tof2_distance_mm = tof2_distance_temp;
            valid_mask |= (1U << 0);
        }
        else
        {
            data->tof2_distance_mm = TOF_INVALID_DISTANCE_MM;
        }
    }
    else
    {
        data->tof2_distance_mm = TOF_INVALID_DISTANCE_MM;
        tof2_range_status = 0xFF;
    }

    if(tof3_ready && (0x89 == tof3_range_status))
    {
        tof3_distance_temp = (uint16)(((uint16)tof3_distance_buffer[0] << 8) | tof3_distance_buffer[1]);
        if(tof3_distance_temp <= 4000)
        {
            if(tof3_distance_temp > TOF_VALID_RANGE_MAX)
            {
                tof3_distance_temp = (uint16)TOF_VALID_RANGE_MAX;
            }
            data->tof3_distance_mm = tof3_distance_temp;
            valid_mask |= (1U << 1);
        }
        else
        {
            data->tof3_distance_mm = TOF_INVALID_DISTANCE_MM;
        }
    }
    else
    {
        data->tof3_distance_mm = TOF_INVALID_DISTANCE_MM;
        tof3_range_status = 0xFF;
    }

    data->tof2_range_status = tof2_range_status;
    data->tof3_range_status = tof3_range_status;

    tof_data = *data;

    return valid_mask;
}
