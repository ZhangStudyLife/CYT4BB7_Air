/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#include "small_driver_uart_control.h"

small_device_value_struct motor_value;      // 定义通讯参数结构体


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口接收回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     uart_control_callback(1000, -1000);
// 备注信息     用于解析接收到的速度数据  该函数需要在对应的串口接收中断中调用
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback(void)
{
    uint8 receive_data;                                                                     // 定义临时变量

    if(uart_query_byte(SMALL_DRIVER_UART, &receive_data))                                   // 接收串口数据
    {
        if(receive_data == 0xA5 && motor_value.receive_data_buffer[0] != 0xA5)              // 判断是否收到帧头 并且 当前接收内容中是否正确包含帧头
        {
            motor_value.receive_data_count = 0;                                             // 未收到帧头或者未正确包含帧头则重新接收
        }

        motor_value.receive_data_buffer[motor_value.receive_data_count ++] = receive_data;  // 保存串口数据

        if(motor_value.receive_data_count >= 11)                                            // 判断是否接收到指定数量的数据
        {
            if(motor_value.receive_data_buffer[0] == 0xA5)                                  // 判断帧头是否正确
            {
                motor_value.sum_check_data = 0;                                             // 清除校验位数据

                for(int i = 0; i < 10; i ++)
                {
                    motor_value.sum_check_data += motor_value.receive_data_buffer[i];       // 重新计算校验位
                }

                if(motor_value.sum_check_data == motor_value.receive_data_buffer[10])       // 校验数据准确性
                {
                    for(int i = 0; i < 4; i ++)
                    {
                        motor_value.receive_speed_data[i]   = (((int)motor_value.receive_data_buffer[i * 2 + 2] << 8) | (int)motor_value.receive_data_buffer[i * 2 + 3]);
                    }

                    motor_value.receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value.receive_data_buffer, 0, 11);                         // 清除缓冲区数据
                }
                else
                {
                    motor_value.receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value.receive_data_buffer, 0, 11);                         // 清除缓冲区数据
                }
            }
            else
            {
                motor_value.receive_data_count = 0;                                         // 清除缓冲区计数值

                memset(motor_value.receive_data_buffer, 0, 11);                             // 清除缓冲区数据
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 设置电机占空比
// 参数说明     left_duty       左侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 参数说明     right_duty      右侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 返回参数     void
// 使用示例     small_driver_set_duty(1000, -1000, 1000, -1000);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty(int16 motor_duty_1, int16 motor_duty_2, int16 motor_duty_3, int16 motor_duty_4)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value.send_data_buffer[1] = 0X01;                                         // 配置功能字

    motor_value.send_data_buffer[2] = (uint8)((motor_duty_1 & 0xFF00) >> 8);        // 拆分 电机1占空比 的高八位

    motor_value.send_data_buffer[3] = (uint8)(motor_duty_1 & 0x00FF);               // 拆分 电机1占空比 的低八位

    motor_value.send_data_buffer[4] = (uint8)((motor_duty_2 & 0xFF00) >> 8);        // 拆分 电机2占空比 的高八位

    motor_value.send_data_buffer[5] = (uint8)(motor_duty_2 & 0x00FF);               // 拆分 电机2占空比 的低八位

    motor_value.send_data_buffer[6] = (uint8)((motor_duty_3 & 0xFF00) >> 8);        // 拆分 电机3占空比 的高八位

    motor_value.send_data_buffer[7] = (uint8)(motor_duty_3 & 0x00FF);               // 拆分 电机3占空比 的低八位

    motor_value.send_data_buffer[8] = (uint8)((motor_duty_4 & 0xFF00) >> 8);        // 拆分 电机4占空比 的高八位

    motor_value.send_data_buffer[9] = (uint8)(motor_duty_4 & 0x00FF);               // 拆分 电机4占空比 的低八位

    motor_value.send_data_buffer[10] = 0;                                           // 和校验清除

    for(int i = 0; i < 10; i ++)
    {
        motor_value.send_data_buffer[10] += motor_value.send_data_buffer[i];        // 计算校验位
    }

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);                     // 发送设置占空比的 字节包 数据
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取速度信息
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_get_speed();
// 备注信息     仅需发送一次 驱动将周期发出速度信息(默认10ms)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(void)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value.send_data_buffer[1] = 0X02;                                         // 配置功能字

    motor_value.send_data_buffer[2] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[3] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[4] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[5] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[6] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[7] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[8] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[9] = 0x00;                                         // 数据位清空

    motor_value.send_data_buffer[10] = 0xA7;                                        // 配置校验位

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 11);         // 发送获取转速数据的 字节包 数据
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 参数初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init(void)
{
    memset(motor_value.send_data_buffer, 0, 11);                             // 清除缓冲区数据

    memset(motor_value.receive_data_buffer, 0, 11);                          // 清除缓冲区数据

    memset(motor_value.receive_speed_data, 0, 8);                            // 清除接收数据

    motor_value.receive_data_count          = 0;

    motor_value.sum_check_data              = 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口通讯初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_uart_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // 串口初始化

    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // 使能串口接收中断

    small_driver_init();                                                                        // 结构体参数初始化

    small_driver_set_duty(0, 0, 0, 0);                                                          // 设置0占空比

    small_driver_get_speed();                                                                   // 获取实时速度数据
}














