/*********************************************************************************************************************
 * LC302 UPixels 光流模块驱动实现
 *
 * 帧格式 (共 14 字节):
 *   [0]  0xFE  帧头 1
 *   [1]  0x0A  帧头 2
 *   [2]  flow_x_integral    低字节
 *   [3]  flow_x_integral    高字节
 *   [4]  flow_y_integral    低字节
 *   [5]  flow_y_integral    高字节
 *   [6]  integration_timespan 低字节
 *   [7]  integration_timespan 高字节
 *   [8]  ground_distance    低字节
 *   [9]  ground_distance    高字节
 *   [10] valid
 *   [11] version
 *   [12] XOR 校验（字节 2~11 异或）
 *   [13] 0x55  帧尾
 ********************************************************************************************************************/

#include "LC302.h"

// -------------------- 全局变量 --------------------
OpticalFlowData lc302_data    = {0};


// -------------------- 内部常量 --------------------
#define LC302_HEADER1      0xFEu    // 帧头 1
#define LC302_HEADER2      0x0Au    // 帧头 2
#define LC302_TAIL         0x55u    // 帧尾
#define LC302_FRAME_LEN    14u      // 完整帧字节数
#define LC302_CRC_INDEX    12u      // XOR 校验字节下标
#define LC302_TAIL_INDEX   13u      // 帧尾字节下标

// -------------------- 内部状态变量 --------------------
static uint8        s_receiver_data[LC302_FRAME_LEN] = {0};   // 接收完整原始帧
static uint8        s_receiver_len = 0;                       // 当前已接收字节数

static OpticalFlowData s_isr_data  = {0};   // ISR 影子缓冲
static volatile uint8  s_isr_ready = 0;     // ISR 新帧标志

// -------------------- 串口接收核心（单字节处理）--------------------
static void lc302_feed_byte(uint8 byte)
{
    uint8 parity_bit_sum = 0;
    uint8 i;

    s_receiver_data[s_receiver_len++] = byte;

    if ((s_receiver_len == 1u) && (s_receiver_data[0] != LC302_HEADER1))
    {
        s_receiver_len = 0;
        return;
    }

    if ((s_receiver_len == 2u) && (s_receiver_data[1] != LC302_HEADER2))
    {
        s_receiver_len = (byte == LC302_HEADER1) ? 1u : 0u;
        s_receiver_data[0] = byte;
        return;
    }

    if (s_receiver_len >= LC302_FRAME_LEN)
    {
        for (i = 2u; i < LC302_CRC_INDEX; i++)
        {
            parity_bit_sum ^= s_receiver_data[i];
        }

        if ((s_receiver_data[LC302_TAIL_INDEX] == LC302_TAIL) &&
            (parity_bit_sum == s_receiver_data[LC302_CRC_INDEX]))
        {
            // 校验通过，解析完整 receiver_data[14] 中的 payload
            s_isr_data.flow_x_integral      = (int16)((uint16)s_receiver_data[2] | ((uint16)s_receiver_data[3] << 8));
            s_isr_data.flow_y_integral      = (int16)((uint16)s_receiver_data[4] | ((uint16)s_receiver_data[5] << 8));
            s_isr_data.integration_timespan = (uint16)s_receiver_data[6] | ((uint16)s_receiver_data[7] << 8);
            s_isr_data.ground_distance      = (uint16)s_receiver_data[8] | ((uint16)s_receiver_data[9] << 8);
            s_isr_data.valid                = s_receiver_data[10];
            s_isr_data.version              = s_receiver_data[11];
            s_isr_ready = 1;
        }

        s_receiver_len = 0;
    }
}

// -------------------- 对外接口 --------------------

/**
 * @brief  初始化 LC302 驱动（UART 初始化）
 *         需在系统初始化阶段调用，调用前确保时钟已就绪
 */
void LC302_Init(void)
{
    lc302_data.flow_x_integral      = 0;
    lc302_data.flow_y_integral      = 0;
    lc302_data.integration_timespan = 0;
    lc302_data.ground_distance      = 0;
    lc302_data.valid                = 0;
    lc302_data.version              = 0;
    s_isr_data                      = lc302_data;
    s_isr_ready                     = 0;
    s_receiver_len                  = 0;

    system_delay_ms(100);
    uart_init(LC302_UART, LC302_BAUD, LC302_TX_PIN, LC302_RX_PIN);
    uart_rx_interrupt(LC302_UART, 1);   // 1=使能 RX_TRIGGER 中断，0=禁用；uart_init 默认传0禁用，此处必须显式传1
}

/**
 * @brief  主循环中调用，将 ISR 影子缓冲原子拷贝到 lc302_data
 *         ISR 独占状态机，此函数仅负责发布数据
 *
 * @note ZYZ亲自测试,几乎不花费任何性能,就是Copy
 */
void LC302_Update_50HZ(void)
{
    uint32 primask = interrupt_global_disable();

    if (s_isr_ready)
    {
        lc302_data  = s_isr_data;
        s_isr_ready = 0;
    }

    interrupt_global_enable(primask);
}

/**
 * @brief  UART ISR 中调用，从硬件取一字节并喂入状态机
 *         保持 ISR 中工作量最小
 */
void LC302_uart_handler(void)
{
    uint8 byte;
    while (uart_query_byte(LC302_UART, &byte))
    {
        lc302_feed_byte(byte);
    }
}
