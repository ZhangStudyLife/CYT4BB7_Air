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
#define LC302_HEADER1   0xFE
#define LC302_HEADER2   0x0A
#define LC302_TAIL      0x55
#define LC302_PAYLOAD   10u      // payload 字节数

// -------------------- 状态机定义 --------------------
typedef enum
{
    WAIT_FE   = 0,  // 等待帧头 0xFE
    WAIT_0A   = 1,  // 等待帧头 0x0A
    RECV_DATA = 2,  // 接收 10 字节 payload
    RECV_CRC  = 3,  // 接收 XOR 校验字节
    RECV_TAIL = 4   // 接收帧尾 0x55
} Lc302State;

// -------------------- 内部状态变量 --------------------
static Lc302State   s_state  = WAIT_FE;
static uint8        s_buf[LC302_PAYLOAD];   // payload 缓冲区
static uint8        s_idx    = 0;           // 已接收 payload 字节数
static uint8        s_crc    = 0;           // 滚动 XOR 值
static uint8        s_rx_crc = 0;           // 从帧中读到的校验字节

static OpticalFlowData s_isr_data  = {0};   // ISR 影子缓冲
static volatile uint8  s_isr_ready = 0;     // ISR 新帧标志

// -------------------- 状态机核心（单字节处理）--------------------
static void lc302_feed_byte(uint8 byte)
{
    switch (s_state)
    {
        case WAIT_FE:
            if (byte == LC302_HEADER1)
                s_state = WAIT_0A;
            break;

        case WAIT_0A:
            if (byte == LC302_HEADER2)
            {
                s_idx   = 0;
                s_crc   = 0;
                s_state = RECV_DATA;
            }
            else
            {
                // 非 0x0A：检查是否是新的 0xFE（鲁棒性）
                s_state = (byte == LC302_HEADER1) ? WAIT_0A : WAIT_FE;
            }
            break;

        case RECV_DATA:
            s_buf[s_idx] = byte;
            s_crc       ^= byte;
            s_idx++;
            if (s_idx >= LC302_PAYLOAD)
                s_state = RECV_CRC;
            break;

        case RECV_CRC:
            s_rx_crc = byte;
            s_state  = RECV_TAIL;
            break;

        case RECV_TAIL:
            if (byte == LC302_TAIL && s_rx_crc == s_crc)
            {
                // 校验通过，解析 payload 到 ISR 影子缓冲（小端序，与 ARM Cortex-M7 一致）
                s_isr_data.flow_x_integral      = (int16)((uint16)s_buf[0] | ((uint16)s_buf[1] << 8));
                s_isr_data.flow_y_integral      = (int16)((uint16)s_buf[2] | ((uint16)s_buf[3] << 8));
                s_isr_data.integration_timespan = (uint16)s_buf[4] | ((uint16)s_buf[5] << 8);
                s_isr_data.ground_distance      = (uint16)s_buf[6] | ((uint16)s_buf[7] << 8);
                s_isr_data.valid                = s_buf[8];
                s_isr_data.version              = s_buf[9];
                s_isr_ready                     = 1;
                if (s_isr_data.valid == 0)
                {
                    // 数据无效时，清零关键字段以避免误用
                    s_isr_data.flow_x_integral = 0;
                    s_isr_data.flow_y_integral = 0;
                }
            }
            // 无论成功与否，回到等待帧头状态
            s_state = WAIT_FE;
            break;

        default:
            s_state = WAIT_FE;
            break;
    }
}

// -------------------- 对外接口 --------------------

/**
 * @brief  初始化 LC302 驱动（UART 初始化）
 *         需在系统初始化阶段调用，调用前确保时钟已就绪
 */
void LC302_Init(void)
{
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
    if (s_isr_ready)
    {
        uint32 primask = interrupt_global_disable();
        lc302_data  = s_isr_data;
        s_isr_ready = 0;
        interrupt_global_enable(primask);
    }
}

/**
 * @brief  UART ISR 中调用，从硬件取一字节并喂入状态机
 *         保持 ISR 中工作量最小
 */
void LC302_uart_handler(void)
{
    uint8 byte;
    if (uart_query_byte(LC302_UART, &byte))
    {
        lc302_feed_byte(byte);
    }
}
