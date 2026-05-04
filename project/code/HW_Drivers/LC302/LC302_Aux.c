/*********************************************************************************************************************
 * LC302 Aux UPixels 光流模块驱动实现
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

#include "LC302_Aux.h"

// -------------------- 全局变量 --------------------
OpticalFlowData_Aux lc302_data_Aux    = {0};


// -------------------- 内部常量 --------------------
#define LC302_HEADER1_Aux   0xFE
#define LC302_HEADER2_Aux   0x0A
#define LC302_TAIL_Aux      0x55
#define LC302_PAYLOAD_Aux   10u      // payload 字节数
/* LC302 Aux 模块有效帧标志值 */
#define LC302_VALID_VALUE_Aux             245u
/* LC302 Aux 连续 50Hz 无新帧超时次数，5 次约 100ms */
#define LC302_TIMEOUT_50HZ_COUNT_Aux      5u

// -------------------- 状态机定义 --------------------
typedef enum
{
    WAIT_FE_Aux   = 0,  // 等待帧头 0xFE
    WAIT_0A_Aux   = 1,  // 等待帧头 0x0A
    RECV_DATA_Aux = 2,  // 接收 10 字节 payload
    RECV_CRC_Aux  = 3,  // 接收 XOR 校验字节
    RECV_TAIL_Aux = 4   // 接收帧尾 0x55
} Lc302State_Aux;

// -------------------- 内部状态变量 --------------------
static Lc302State_Aux   s_state_Aux  = WAIT_FE_Aux;
static uint8            s_buf_Aux[LC302_PAYLOAD_Aux];   // payload 缓冲区
static uint8            s_idx_Aux    = 0;                // 已接收 payload 字节数
static uint8            s_crc_Aux    = 0;                // 滚动 XOR 值
static uint8            s_rx_crc_Aux = 0;                // 从帧中读到的校验字节

static OpticalFlowData_Aux s_isr_data_Aux  = {0};        // ISR 影子缓冲
static volatile uint8      s_isr_ready_Aux = 0;          // ISR 新帧标志
/* 连续无新帧计数，单位为 LC302_Update_50HZ_Aux 调用次数 */
static uint8               s_no_frame_count_Aux = 0;

// -------------------- 状态机核心（单字节处理）--------------------
static void lc302_feed_byte_Aux(uint8 byte)
{
    switch (s_state_Aux)
    {
        case WAIT_FE_Aux:
            if (byte == LC302_HEADER1_Aux)
                s_state_Aux = WAIT_0A_Aux;
            break;

        case WAIT_0A_Aux:
            if (byte == LC302_HEADER2_Aux)
            {
                s_idx_Aux   = 0;
                s_crc_Aux   = 0;
                s_state_Aux = RECV_DATA_Aux;
            }
            else
            {
                // 非 0x0A：检查是否是新的 0xFE（鲁棒性）
                s_state_Aux = (byte == LC302_HEADER1_Aux) ? WAIT_0A_Aux : WAIT_FE_Aux;
            }
            break;

        case RECV_DATA_Aux:
            s_buf_Aux[s_idx_Aux] = byte;
            s_crc_Aux           ^= byte;
            s_idx_Aux++;
            if (s_idx_Aux >= LC302_PAYLOAD_Aux)
                s_state_Aux = RECV_CRC_Aux;
            break;

        case RECV_CRC_Aux:
            s_rx_crc_Aux = byte;
            s_state_Aux  = RECV_TAIL_Aux;
            break;

        case RECV_TAIL_Aux:
            if (byte == LC302_TAIL_Aux && s_rx_crc_Aux == s_crc_Aux)
            {
                // 校验通过，解析 payload 到 ISR 影子缓冲（小端序，与 ARM Cortex-M7 一致）
                s_isr_data_Aux.flow_x_integral      = (int16)((uint16)s_buf_Aux[0] | ((uint16)s_buf_Aux[1] << 8));
                s_isr_data_Aux.flow_y_integral      = (int16)((uint16)s_buf_Aux[2] | ((uint16)s_buf_Aux[3] << 8));
                s_isr_data_Aux.integration_timespan = (uint16)s_buf_Aux[4] | ((uint16)s_buf_Aux[5] << 8);
                s_isr_data_Aux.ground_distance      = (uint16)s_buf_Aux[6] | ((uint16)s_buf_Aux[7] << 8);
                s_isr_data_Aux.valid                = s_buf_Aux[8];
                s_isr_data_Aux.version              = s_buf_Aux[9];
                if (s_isr_data_Aux.valid != LC302_VALID_VALUE_Aux)
                {
                    // 数据无效时清零发布，避免上层误用上一帧
                    s_isr_data_Aux.flow_x_integral      = 0;
                    s_isr_data_Aux.flow_y_integral      = 0;
                    s_isr_data_Aux.integration_timespan = 0;
                    s_isr_data_Aux.ground_distance      = 0;
                    s_isr_data_Aux.valid                = 0;
                    s_isr_data_Aux.version              = 0;
                }
                s_isr_ready_Aux = 1;
            }
            // 无论成功与否，回到等待帧头状态
            s_state_Aux = WAIT_FE_Aux;
            break;

        default:
            s_state_Aux = WAIT_FE_Aux;
            break;
    }
}

// -------------------- 对外接口 --------------------

/**
 * @brief  初始化 LC302 Aux 驱动（UART 初始化）
 *         需在系统初始化阶段调用，调用前确保时钟已就绪
 */
void LC302_Init_Aux(void)
{
    lc302_data_Aux.flow_x_integral      = 0;
    lc302_data_Aux.flow_y_integral      = 0;
    lc302_data_Aux.integration_timespan = 0;
    lc302_data_Aux.ground_distance      = 0;
    lc302_data_Aux.valid                = 0;
    lc302_data_Aux.version              = 0;
    s_isr_data_Aux                      = lc302_data_Aux;
    s_isr_ready_Aux                     = 0;
    s_no_frame_count_Aux                = 0;

    system_delay_ms(100);
    uart_init(LC302_UART_Aux, LC302_BAUD_Aux, LC302_TX_PIN_Aux, LC302_RX_PIN_Aux);
    uart_rx_interrupt(LC302_UART_Aux, 1);   // 1=使能 RX_TRIGGER 中断，0=禁用；uart_init 默认传0禁用，此处必须显式传1
}

/**
 * @brief  主循环中调用，将 ISR 影子缓冲原子拷贝到 lc302_data_Aux
 *         ISR 独占状态机，此函数仅负责发布数据
 * 
 * @note ZYZ亲自测试,几乎不花费任何性能,就是Copy
 */
void LC302_Update_50HZ_Aux(void)
{
    uint8 has_new_frame = 0;
    uint32 primask = interrupt_global_disable();

    if (s_isr_ready_Aux)
    {
        lc302_data_Aux  = s_isr_data_Aux;
        s_isr_ready_Aux = 0;
        has_new_frame = 1;
    }

    interrupt_global_enable(primask);

    if (has_new_frame)
    {
        s_no_frame_count_Aux = 0;
    }
    else
    {
        if (s_no_frame_count_Aux < LC302_TIMEOUT_50HZ_COUNT_Aux)
        {
            s_no_frame_count_Aux++;
        }

        if (s_no_frame_count_Aux >= LC302_TIMEOUT_50HZ_COUNT_Aux)
        {
            lc302_data_Aux.flow_x_integral      = 0;
            lc302_data_Aux.flow_y_integral      = 0;
            lc302_data_Aux.integration_timespan = 0;
            lc302_data_Aux.ground_distance      = 0;
            lc302_data_Aux.valid                = 0;
            lc302_data_Aux.version              = 0;
        }
    }
}

/**
 * @brief  UART ISR 中调用，从硬件取一字节并喂入状态机
 *         保持 ISR 中工作量最小
 */
void LC302_uart_handler_Aux(void)
{
    uint8 byte;
    if (uart_query_byte(LC302_UART_Aux, &byte))
    {
        lc302_feed_byte_Aux(byte);
    }
}
