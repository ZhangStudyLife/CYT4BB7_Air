#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART                       (UART_4        )

#define SMALL_DRIVER_BAUDRATE                   (460800        )

#define SMALL_DRIVER_RX                         (UART4_TX_P14_1)

#define SMALL_DRIVER_TX                         (UART4_RX_P14_0)

typedef struct
{
    uint8 send_data_buffer[11];                 // 发送缓冲数组

    uint8 receive_data_buffer[11];              // 接收缓冲数组

    uint8 receive_data_count;                   // 接收计数

    uint8 sum_check_data;                       // 校验位

    int16 receive_speed_data[4];                // 接收到的电机速度数据

}small_device_value_struct;

extern small_device_value_struct motor_value;



void uart_control_callback(void);                                   // 无刷驱动 串口接收回调函数

void small_driver_set_duty(int16 motor_duty_1, int16 motor_duty_2, int16 motor_duty_3, int16 motor_duty_4);      // 无刷驱动 设置电机占空比

void small_driver_get_speed(void);                                  // 无刷驱动 获取速度信息

void small_driver_uart_init(void);                                  // 无刷驱动 串口通讯初始化


#endif
