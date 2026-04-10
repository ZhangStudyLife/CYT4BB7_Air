/*********************************************************************************************************************
 * LC302 UPixels 光流模块驱动头文件
 * 帧格式: 0xFE 0x0A [10字节 payload] [XOR校验] 0x55
 * 使用 UART_1，波特率 19200
 ********************************************************************************************************************/

#ifndef LC302_H
#define LC302_H

#include "zf_common_headfile.h"

// -------------------- 硬件配置 --------------------
#define LC302_UART      UART_1
#define LC302_BAUD      19200
#define LC302_TX_PIN    UART1_TX_P04_1
#define LC302_RX_PIN    UART1_RX_P04_0

// -------------------- 数据结构 --------------------
typedef struct
{
    int16   flow_x_integral;        // X 轴光流积分值
    int16   flow_y_integral;        // Y 轴光流积分值
    uint16  integration_timespan;   // 积分时间 (us)，当前模组实测固定约 20800
    uint16  ground_distance;        // 地面距离 (mm)
    uint8   valid;                  // 数据有效标志
    uint8   version;                // 固件版本号
} OpticalFlowData;

// -------------------- 全局变量声明 --------------------
extern OpticalFlowData  lc302_data;     // 最新有效帧数据


// -------------------- 函数声明 --------------------
void LC302_Init(void);              // 初始化 UART，需在系统初始化阶段调用
void LC302_Update_50HZ(void);            // 主循环中调用，轮询排空 UART 缓冲
void LC302_uart_handler(void);      // UART ISR 中调用，逐字节喂入状态机

#endif // LC302_H
