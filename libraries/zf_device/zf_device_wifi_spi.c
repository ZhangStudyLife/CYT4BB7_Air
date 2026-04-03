/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_device_wifi_spi
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2024-01-18        pudding            first version
* 2025-06-23        pudding            修复部分情况下握手异常的问题
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   RST                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_RST_PIN 宏定义
*                   INT                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_INT_PIN 宏定义
*                   CS                  查看 zf_device_wifi_spi.h 中 WIFI_SPI_CS_PIN 宏定义
*                   MISO                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MISO_PIN 宏定义
*                   SCK                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_SCK_PIN 宏定义
*                   MOSI                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MOSI_PIN 宏定义
*                   5V                  5V 电源
*                   GND                 电源地
*                   其余引脚悬空
*                   ------------------------------------
*********************************************************************************************************************/
#include "stdio.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"
#include "zf_device_type.h"
#include "scb/cy_scb_spi.h"
#include "dma/cy_pdma.h"
#include "trigmux/cy_trigmux.h"

#include "zf_device_wifi_spi.h"

#define WIFI_CONNECT_TIME_OUT       10000       // 单位毫秒
#define SOCKET_CONNECT_TIME_OUT     50000       // 单位毫秒
#define OTHER_TIME_OUT              1000        // 单位毫秒
#define WIFI_SPI_TX_DMA_CHANNEL         (30u)   // WiFi SPI(SCB7) TX请求对应 DW1 通道号
#define WIFI_SPI_TX_DMA_TRIGGER_1TO1    (TRIG_OUT_1TO1_2_SCB_TX_TO_PDMA17) // SCB7 TX -> DW1_TR_IN[30]
#define WIFI_SPI_TX_DMA_USE_SW_TRIGGER  (1u)    // DMA触发方式：0-硬件触发 1-软件触发

char wifi_spi_version[12];                      // 保存模块固件版本信息
char wifi_spi_mac_addr[20];                     // 保存模块MAC地址信息
char wifi_spi_ip_addr_port[25];                 // 保存模块IP地址与端口信息

static fifo_struct  wifi_spi_fifo;
static uint8        wifi_spi_buffer[WIFI_SPI_RECVIVE_FIFO_SIZE];
static volatile     wifi_spi_state_enum wifi_spi_mutex;
/* 非阻塞发送缓存：保存待发送数据，避免上层临时缓冲区失效 */
static uint8        wifi_spi_tx_cache[WIFI_SPI_TRANSFER_SIZE];
/* 非阻塞发送缓存有效长度，单位字节 */
static uint16       wifi_spi_tx_length = 0;
/* 非阻塞发送是否已提交且待处理：1-有待发任务，0-无待发任务 */
static uint8        wifi_spi_tx_pending = 0;
/* WiFi SPI TX DMA 是否初始化成功 */
static uint8        wifi_spi_tx_dma_ready = 0;
/* WiFi SPI TX DMA 是否正在传输 */
static uint8        wifi_spi_tx_dma_busy = 0;
/* WiFi SPI TX DMA 描述符 */
static cy_stc_pdma_descr_t wifi_spi_tx_dma_descr;

/* 发送轮询内部步骤状态 */
typedef enum
{
    WIFI_SPI_TX_STEP_IDLE = 0,
    WIFI_SPI_TX_STEP_WAIT_SEND,
    WIFI_SPI_TX_STEP_SEND,
    WIFI_SPI_TX_STEP_SEND_DMA_WAIT,
    WIFI_SPI_TX_STEP_SEND_TX_WAIT,
    WIFI_SPI_TX_STEP_DRAIN_REPLY,
    WIFI_SPI_TX_STEP_DONE,
    WIFI_SPI_TX_STEP_ERROR,
}wifi_spi_tx_step_enum;

/* 当前发送轮询步骤 */
static wifi_spi_tx_step_enum wifi_spi_tx_step = WIFI_SPI_TX_STEP_IDLE;
/* WiFi SPI 对应 SCB 基地址（WIFI_SPI_INDEX 固定为 SPI_0，对应 SCB7） */
static volatile stc_SCB_t * const s_wifi_spi_scb = SCB7;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 WIFI SPI 发送是否完成
// 参数说明     void
// 返回参数     uint8           状态 1-发送完成 0-发送未完成
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_tx_is_complete (void)
{
    return (0 != Cy_SCB_IsTxComplete(s_wifi_spi_scb)) ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空 WIFI SPI RX FIFO
// 参数说明     void
// 返回参数     void
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static void wifi_spi_rx_fifo_drain (void)
{
    while(0 != Cy_SCB_GetNumInRxFifo(s_wifi_spi_scb))
    {
        (void)Cy_SCB_ReadRxFifo(s_wifi_spi_scb);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WiFi SPI TX DMA 初始化
// 参数说明     void
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_tx_dma_init (void)
{
    uint8 return_state = 1;
    cy_stc_pdma_chnl_config_t tx_chnl_config;

    do
    {
        memset(&tx_chnl_config, 0, sizeof(tx_chnl_config));
        tx_chnl_config.PDMA_Descriptor    = &wifi_spi_tx_dma_descr;
        tx_chnl_config.preemptable        = 0;
        tx_chnl_config.priority           = 0;
        tx_chnl_config.enable             = 1;
        tx_chnl_config.priviledge         = 0;
        tx_chnl_config.non_secure         = 0;
        tx_chnl_config.bufferable         = 0;
        tx_chnl_config.protection_context = 0;

        if(CY_PDMA_SUCCESS != Cy_PDMA_Chnl_Init(DW1, WIFI_SPI_TX_DMA_CHANNEL, &tx_chnl_config))
        {
            break;
        }

#if (0u == WIFI_SPI_TX_DMA_USE_SW_TRIGGER)
        if(CY_TRIGMUX_SUCCESS != Cy_TrigMux_Connect1To1(WIFI_SPI_TX_DMA_TRIGGER_1TO1, CY_TR_MUX_TR_INV_DISABLE, TRIGGER_TYPE_LEVEL, 0))
        {
            break;
        }
#endif

        Cy_PDMA_Chnl_SetInterruptMask(DW1, WIFI_SPI_TX_DMA_CHANNEL);
        Cy_PDMA_Enable(DW1);
        return_state = 0;
    }while(0);

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动一次 WiFi SPI TX DMA 发送
// 参数说明     *data           数据地址
// 参数说明     len             数据长度
// 返回参数     uint8           状态 0-启动成功 1-启动失败
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_tx_dma_start (const uint8 *data, uint16 len)
{
    uint8 return_state = 1;
    cy_stc_pdma_descr_config_t tx_descr_config;

    do
    {
        if((NULL == data) || (0 == len))
        {
            break;
        }

        if(0 == wifi_spi_tx_dma_ready)
        {
            if(0 != wifi_spi_tx_dma_init())
            {
                break;
            }
            wifi_spi_tx_dma_ready = 1;
        }

        if(0 != wifi_spi_tx_dma_busy)
        {
            break;
        }

        memset(&tx_descr_config, 0, sizeof(tx_descr_config));
        tx_descr_config.deact          = CY_PDMA_TRIG_DEACT_NO_WAIT;
        tx_descr_config.intrType       = CY_PDMA_INTR_X_LOOP_CMPLT;
        tx_descr_config.trigoutType    = CY_PDMA_TRIGOUT_DESCR_CMPLT;
        tx_descr_config.chStateAtCmplt = CY_PDMA_CH_DISABLED;
        tx_descr_config.triginType     = (0u == WIFI_SPI_TX_DMA_USE_SW_TRIGGER) ? CY_PDMA_TRIGIN_1ELEMENT : CY_PDMA_TRIGIN_DESCR;
        tx_descr_config.dataSize       = CY_PDMA_BYTE;
        tx_descr_config.srcTxfrSize    = CY_PDMA_TXFR_SIZE_DATA_SIZE;
        tx_descr_config.destTxfrSize   = CY_PDMA_TXFR_SIZE_WORD;
        tx_descr_config.descrType      = CY_PDMA_1D_TRANSFER;
        tx_descr_config.srcAddr        = (void *)data;
        tx_descr_config.destAddr       = (void *)&(s_wifi_spi_scb->unTX_FIFO_WR.u32Register);
        tx_descr_config.srcXincr       = 1;
        tx_descr_config.destXincr      = 0;
        tx_descr_config.xCount         = len;
        tx_descr_config.srcYincr       = 0;
        tx_descr_config.destYincr      = 0;
        tx_descr_config.yCount         = 0;
        tx_descr_config.descrNext      = NULL;

        if(CY_PDMA_SUCCESS != Cy_PDMA_Descr_Init(&wifi_spi_tx_dma_descr, &tx_descr_config))
        {
            break;
        }

        Cy_PDMA_Chnl_ClearInterrupt(DW1, WIFI_SPI_TX_DMA_CHANNEL);
        Cy_PDMA_Chnl_SetDescr(DW1, WIFI_SPI_TX_DMA_CHANNEL, &wifi_spi_tx_dma_descr);
        Cy_PDMA_Chnl_Enable(DW1, WIFI_SPI_TX_DMA_CHANNEL);
#if defined(CPUSS_SW_TR_PRESENT) && (CPUSS_SW_TR_PRESENT == 1)
        if(0u != WIFI_SPI_TX_DMA_USE_SW_TRIGGER)
        {
            Cy_PDMA_Chnl_SetSwTrigger(DW1, WIFI_SPI_TX_DMA_CHANNEL);
        }
#endif
        wifi_spi_tx_dma_busy = 1;
        return_state = 0;
    }while(0);

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 WiFi SPI TX DMA 是否完成
// 参数说明     void
// 返回参数     uint8           状态 1-完成 0-未完成
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_tx_dma_is_done (void)
{
    uint32 dma_cause;

    if(0 == wifi_spi_tx_dma_busy)
    {
        return 1;
    }

    if(0 == Cy_PDMA_Chnl_GetInterruptStatus(DW1, WIFI_SPI_TX_DMA_CHANNEL))
    {
        return 0;
    }

    dma_cause = Cy_PDMA_Chnl_GetInterruptCause(DW1, WIFI_SPI_TX_DMA_CHANNEL);
    Cy_PDMA_Chnl_ClearInterrupt(DW1, WIFI_SPI_TX_DMA_CHANNEL);

    if(CY_PDMA_INTRCAUSE_COMPLETION != dma_cause)
    {
        wifi_spi_tx_dma_busy = 0;
        return 1;
    }

    wifi_spi_tx_dma_busy = 0;
    return 1;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     等待WIFI SPI就绪
// 参数说明     wait_time       最大等待时间 单位毫秒
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_wait_idle (uint32 wait_time)
{
    uint32 time = 0;
    
    wait_time = wait_time*100;
    while(0 == gpio_get_level(WIFI_SPI_INT_PIN))
    {
        system_delay_us(10);
        time++;
        if(wait_time <= time)
        {
            break;
        }
    }
    return (wait_time <= time);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入数据到WIFI SPI
// 参数说明     *buffer1        第一组需要发送的数据缓冲区地址
// 参数说明     length1         第一组数据长度
// 参数说明     *buffer2        第二组需要发送的数据缓冲区地址
// 参数说明     length2         第二组数据长度
// 返回参数     void
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static void wifi_spi_write (const uint8 *buffer1, uint16 length1, const uint8 *buffer2, uint16 length2)
{
    gpio_low(WIFI_SPI_CS_PIN);
    if(NULL != buffer1)
    {
        spi_write_8bit_array(WIFI_SPI_INDEX, buffer1, length1);
    }
    if(NULL != buffer2)
    {
        spi_write_8bit_array(WIFI_SPI_INDEX, buffer2, length2);
    }
    gpio_high(WIFI_SPI_CS_PIN);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 发送与接收同时进行（命令收发）
// 参数说明     *packets        发送与接收的地址
// 参数说明     length          需要接收的长度
// 返回参数     void
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static void wifi_spi_transfer_command (wifi_spi_packets_struct *packets, uint16 length)
{
    gpio_low(WIFI_SPI_CS_PIN);
    
    spi_transfer_8bit(WIFI_SPI_INDEX, (uint8 *)&(packets->head), (uint8 *)&(packets->head), sizeof(wifi_spi_head_struct));
    
    if(length)
    {
        spi_transfer_8bit(WIFI_SPI_INDEX, (const uint8 *)(packets->buffer), packets->buffer, length);
    }
    
    gpio_high(WIFI_SPI_CS_PIN);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 发送与接收同时进行(数据收发)
// 参数说明     *write_data     发送的数据缓冲区地址
// 参数说明     *read_data      接收到的数据的存储地址
// 参数说明     length          需要接收的长度
// 返回参数     void
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static void wifi_spi_transfer_data (const uint8 *write_data, wifi_spi_packets_struct *read_data, uint16 length)
{
    gpio_low(WIFI_SPI_CS_PIN);
    
    read_data->head.command = WIFI_SPI_DATA;
    read_data->head.length  = length;
    
    spi_transfer_8bit(WIFI_SPI_INDEX, (uint8 *)&(read_data->head), (uint8 *)&(read_data->head), sizeof(wifi_spi_head_struct));
    
    if(WIFI_SPI_RECVIVE_SIZE < length)
    {
        spi_transfer_8bit(WIFI_SPI_INDEX, write_data, read_data->buffer, WIFI_SPI_RECVIVE_SIZE);
        spi_write_8bit_array(WIFI_SPI_INDEX, &write_data[WIFI_SPI_RECVIVE_SIZE], length - WIFI_SPI_RECVIVE_SIZE);
    }
    else
    {
        // 将需要发送的数据拷贝到读取缓冲区，避免出现write_data越界访问
        memcpy(read_data->buffer, write_data, length);
        spi_transfer_8bit(WIFI_SPI_INDEX, read_data->buffer, read_data->buffer, WIFI_SPI_RECVIVE_SIZE);
    }
    gpio_high(WIFI_SPI_CS_PIN);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 参数设置
// 参数说明     command         命令类型
// 参数说明     *buffer         参数地址
// 参数说明     length          参数长度
// 参数说明     wait_time       最大等待时间 单位100微妙
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_set_parameter (wifi_spi_packets_command_enum command, uint8 *buffer, uint16 length, uint32 wait_time)
{
    uint8 return_state;
    wifi_spi_head_struct head;
    return_state = 1;
    do
    {
        head.command = command;
        head.length  = length;
        
        // 等待从机准备就绪
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }

        wifi_spi_write(&head.command, sizeof(wifi_spi_head_struct), buffer, length);
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }
        // 接收应答信号

        head.command = WIFI_SPI_DATA;
        head.length = 0;
        wifi_spi_transfer_command((wifi_spi_packets_struct *)&head, head.length);
        system_delay_us(20);
        if(WIFI_SPI_REPLY_OK == head.command)
        {
            return_state = 0;
        }
    }while(0);
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 模块信息获取
// 参数说明     command         命令类型
// 参数说明     *buffer         保存接收到的参数地址
// 参数说明     wait_time       最大等待时间 单位100微妙
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_parameter (wifi_spi_packets_command_enum command, wifi_spi_packets_struct *read_data, uint32 wait_time)
{
    uint8 return_state;

    return_state = 1;
    do
    {
        // 等待从机准备就绪
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }
        read_data->head.command = command;
        wifi_spi_write(&(read_data->head.command), WIFI_SPI_RECVIVE_SIZE, NULL, 0);

        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }
        read_data->head.command = WIFI_SPI_DATA;
        read_data->head.length = 0;
        wifi_spi_transfer_command(read_data, WIFI_SPI_RECVIVE_SIZE);
        return_state = 0;
    }while(0);
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 固件版本获取
// 参数说明     void            端口号
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例
// 备注信息     调用函数之后，固件版本信息以字符串形式保存在wifi_spi_version数组中
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_version (void)
{
    uint8 return_state;
    wifi_spi_packets_struct temp_packets;

    return_state = wifi_spi_get_parameter(WIFI_SPI_GET_VERSION, &temp_packets, OTHER_TIME_OUT);
    if((0 == return_state) && (WIFI_SPI_REPLY_VERSION == temp_packets.head.command))
    {
        memcpy(wifi_spi_version, temp_packets.buffer, temp_packets.head.length);
    }
    return_state = (return_state == 0) ? (WIFI_SPI_REPLY_VERSION != temp_packets.head.command) : 1;

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI MAC地址获取
// 参数说明     void            端口号
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例
// 备注信息     调用函数之后，MAC地址信息以字符串形式保存在wifi_spi_mac_addr数组中
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_mac_addr (void)
{
    uint8 return_state;
    wifi_spi_packets_struct temp_packets;

    return_state = wifi_spi_get_parameter(WIFI_SPI_GET_MAC_ADDR, &temp_packets, OTHER_TIME_OUT);
    if((0 == return_state) && (WIFI_SPI_REPLY_MAC_ADDR == temp_packets.head.command))
    {
        memcpy(wifi_spi_mac_addr, temp_packets.buffer, temp_packets.head.length);
    }
    return_state = (return_state == 0) ? (WIFI_SPI_REPLY_MAC_ADDR != temp_packets.head.command) : 1;

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI IP地址与端口号获取
// 参数说明     void            端口号
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例
// 备注信息     调用函数之后，IP地址与端口号信息以字符串形式保存在wifi_spi_ip_addr_port数组中
//              需要在连接Socket之后调用此函数才能正常获取信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_ip_addr_port (void)
{
    uint8 return_state;
    wifi_spi_packets_struct temp_packets;

    return_state = wifi_spi_get_parameter(WIFI_SPI_GET_IP_ADDR, &temp_packets, OTHER_TIME_OUT);
    if((0 == return_state) && (WIFI_SPI_REPLY_IP_ADDR == temp_packets.head.command))
    {
        memcpy(wifi_spi_ip_addr_port, temp_packets.buffer, temp_packets.head.length);
    }
    return_state = (return_state == 0) ? (WIFI_SPI_REPLY_IP_ADDR != temp_packets.head.command) : 1;

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 设置连接的WiFi信息并尝试连接WiFi
// 参数说明     *wifi_ssid      WIFI名称
// 参数说明     *pass_word      WIFI密码
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     wifi_spi_wifi_connect("SEEKFREE", "SEEKFREE123");
// 备注信息     wifi_spi_wifi_connect("SEEKFREE", NULL); // 连接没有密码的WIFI热点
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_wifi_connect (char *wifi_ssid, char *pass_word)
{
    uint8 return_state;
    uint8 temp_buffer[64];
    uint16 length;
    
    if(NULL != pass_word)
    {
        // WIFI热点有密码发送热点名称与密码
        length = (uint16)sprintf((char *)temp_buffer, "%s\r\n%s\r\n", wifi_ssid, pass_word);
    }
    else
    {
        // WIFI热点没有密码只需要发送热点名称
        length = (uint16)sprintf((char *)temp_buffer, "%s\r\n", wifi_ssid);
    }

    return_state = wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_INFORMATION, temp_buffer, length, WIFI_CONNECT_TIME_OUT);

    // 本机IP地址与端口号信息以字符串形式保存在wifi_spi_ip_addr_port数组中
    wifi_spi_get_ip_addr_port();

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 设置连接的Socket信息并尝试连接Socket
// 参数说明     *transport_type 传输类型
// 参数说明     *ip_addr        IP地址
// 参数说明     *port           目标端口号
// 参数说明     *local_port     本机端口号
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     wifi_spi_socket_connect("TCP", "192.168.2.5", "8080", "6060");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_socket_connect (char *transport_type, char *ip_addr, char *port, char *local_port)
{
    uint8 return_state;
    uint8 temp_buffer[41];
    uint16 length;
    
    length = (uint16)sprintf((char *)temp_buffer, "%s\r\n%s\r\n%s\r\n%s\r\n", transport_type, ip_addr, port, local_port);

    return_state = wifi_spi_set_parameter(WIFI_SPI_SET_SOCKET_INFORMATION, temp_buffer, length, SOCKET_CONNECT_TIME_OUT);

    // 本机IP地址与端口号信息以字符串形式保存在wifi_spi_ip_addr_port数组中
    wifi_spi_get_ip_addr_port();

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 断开Socket连接
// 参数说明     void
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     wifi_spi_socket_disconnect();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_socket_disconnect (void)
{
    wifi_spi_packets_struct temp_packets;

    return wifi_spi_get_parameter(WIFI_SPI_CLOSE_SOCKET, &temp_packets, OTHER_TIME_OUT);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 软复位
// 参数说明     void
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_reset (void)
{
    uint8 return_state;
    wifi_spi_head_struct head;
    return_state = 1;
    do
    {
        head.command = WIFI_SPI_RESET;
        head.length  = 0xA5A5;
        return_state = wifi_spi_wait_idle(OTHER_TIME_OUT);
        if(return_state)
        {
            break;
        }
        wifi_spi_write(&head.command, sizeof(wifi_spi_head_struct), NULL, 0);
    }while(0);
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI UDP模式时立即发送函数
// 参数说明     void
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例
// 备注信息     在UDP模式下模块收到数据后会等待2毫秒，2毫秒后未收到数据则将数据通过socket发送到网络，如果希望立即发送则在数据传输完毕后调用此函数
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_udp_send_now (void)
{
    uint8 return_state = 1;
    wifi_spi_packets_struct temp_packets;
    
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        // 将通讯状态设置为忙
        wifi_spi_mutex = WIFI_SPI_BUSY;
        do
        {
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }

            // 立即开始socket发送
            temp_packets.head.command = WIFI_SPI_UDP_SEND;
            temp_packets.head.length = 0;
            wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE);
            
            // 检查收到的包中是否有数据
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                // 保存接收到的数据
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
            
            // 等待应答信号
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            
            // 接收应答信号
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length = 0;
            wifi_spi_transfer_command(&temp_packets, temp_packets.head.length);
            
            if(WIFI_SPI_REPLY_OK == temp_packets.head.command)
            {
                return_state = 0;
            }
            
        }while(0);
        
        // 将通讯状态设置为空闲
        wifi_spi_mutex = WIFI_SPI_IDLE;
    } 
    
    return return_state;
}

/*
 * 函数功能：查询 WiFi SPI 当前是否仍有发送事务在执行。
 * 输入参数：无。
 * 返回值：1-正在忙；0-空闲。
 */
uint8 wifi_spi_is_busy (void)
{
    return (WIFI_SPI_BUSY == wifi_spi_mutex) ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 数据块发送函数（提交模式，需配合 wifi_spi_send_poll 推进）
// 参数说明     *buff           需要发送的数据地址
// 参数说明     length          发送长度
// 返回参数     uint32          未提交长度
// 使用示例     wifi_spi_send_buffer(buffer, 100);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32 wifi_spi_send_buffer (const uint8 *buffer, uint32 length)
{
    uint16 submit_length;
    if((NULL == buffer) || (0 == length))
    {
        return length;
    }

    if((WIFI_SPI_IDLE != wifi_spi_mutex) || (0 != wifi_spi_tx_pending))
    {
        return length;
    }

    submit_length = (length > WIFI_SPI_TRANSFER_SIZE) ? (uint16)WIFI_SPI_TRANSFER_SIZE : (uint16)length;
    memcpy(wifi_spi_tx_cache, buffer, submit_length);
    wifi_spi_tx_length = submit_length;
    wifi_spi_tx_pending = 1;
    wifi_spi_tx_step = WIFI_SPI_TX_STEP_WAIT_SEND;
    wifi_spi_mutex = WIFI_SPI_BUSY;
    return (uint32)(length - submit_length);
}

/*
 * 函数功能：推进一次 WiFi SPI 非阻塞发送状态机。
 * 输入参数：无。
 * 返回值：无。
 */
void wifi_spi_send_poll (void)
{
    wifi_spi_packets_struct temp_packets;
    if(WIFI_SPI_BUSY != wifi_spi_mutex)
    {
        return;
    }

    if(0 == wifi_spi_tx_pending)
    {
        wifi_spi_tx_step = WIFI_SPI_TX_STEP_IDLE;
        wifi_spi_mutex = WIFI_SPI_IDLE;
        return;
    }

    switch(wifi_spi_tx_step)
    {
        case WIFI_SPI_TX_STEP_WAIT_SEND:
        {
            if(wifi_spi_wait_idle(1))
            {
                return;
            }
            wifi_spi_tx_step = WIFI_SPI_TX_STEP_SEND;
        }break;

        case WIFI_SPI_TX_STEP_SEND:
        {
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length  = wifi_spi_tx_length;
            gpio_low(WIFI_SPI_CS_PIN);
            spi_write_8bit_array(WIFI_SPI_INDEX, &temp_packets.head.command, sizeof(wifi_spi_head_struct));
            /* SPI 全双工下先排空头部发送产生的RX回灌，避免后续DMA过程RX FIFO顶满 */
            wifi_spi_rx_fifo_drain();

            if(0 == wifi_spi_tx_length)
            {
                gpio_high(WIFI_SPI_CS_PIN);
                wifi_spi_tx_step = WIFI_SPI_TX_STEP_DRAIN_REPLY;
                break;
            }

            if(0 == wifi_spi_tx_dma_start(wifi_spi_tx_cache, wifi_spi_tx_length))
            {
                wifi_spi_tx_step = WIFI_SPI_TX_STEP_SEND_DMA_WAIT;
            }
            else
            {
                spi_write_8bit_array(WIFI_SPI_INDEX, wifi_spi_tx_cache, wifi_spi_tx_length);
                gpio_high(WIFI_SPI_CS_PIN);
                wifi_spi_tx_step = WIFI_SPI_TX_STEP_DRAIN_REPLY;
            }
        }break;

        case WIFI_SPI_TX_STEP_SEND_DMA_WAIT:
        {
            if(0 == wifi_spi_tx_dma_is_done())
            {
                /* DMA发送期间持续排空RX FIFO，防止全双工回灌撑满导致发送停滞 */
                wifi_spi_rx_fifo_drain();
                return;
            }

            wifi_spi_tx_step = WIFI_SPI_TX_STEP_SEND_TX_WAIT;
        }break;

        case WIFI_SPI_TX_STEP_SEND_TX_WAIT:
        {
            if(0 == wifi_spi_tx_is_complete())
            {
                return;
            }

            wifi_spi_rx_fifo_drain();
            gpio_high(WIFI_SPI_CS_PIN);
            wifi_spi_tx_step = WIFI_SPI_TX_STEP_DRAIN_REPLY;
        }break;

        case WIFI_SPI_TX_STEP_DRAIN_REPLY:
        {
            if(wifi_spi_wait_idle(1))
            {
                return;
            }
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length  = 0;
            wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE);
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
            if((WIFI_SPI_REPLY_DATA_END == temp_packets.head.command) || (WIFI_SPI_REPLY_OK == temp_packets.head.command))
            {
                wifi_spi_tx_step = WIFI_SPI_TX_STEP_DONE;
            }
        }break;

        case WIFI_SPI_TX_STEP_DONE:
        {
            wifi_spi_tx_pending = 0;
            wifi_spi_tx_length = 0;
            wifi_spi_tx_step = WIFI_SPI_TX_STEP_IDLE;
            wifi_spi_mutex = WIFI_SPI_IDLE;
        }break;

        default:
        {
            wifi_spi_tx_pending = 0;
            wifi_spi_tx_length = 0;
            wifi_spi_tx_step = WIFI_SPI_TX_STEP_ERROR;
            wifi_spi_mutex = WIFI_SPI_IDLE;
        }break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 读取缓冲区
// 参数说明     *buff           接收缓冲区
// 参数说明     length          读取数据长度
// 返回参数     uint32          实际读取数据长度
// 使用示例     wifi_spi_read_buffer(buffer, 100);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32 wifi_spi_read_buffer (uint8 *buffer, uint32 length)
{
    zf_assert(NULL != buffer);
    uint32 data_len = length;
    
#if(1 == WIFI_SPI_READ_TRANSFER)
    
    wifi_spi_packets_struct temp_packets;
    // 检查WIFI SPI状态，如果在其他中断或者线程中已经发起了通讯，则本次不能发送数据
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        // 将通讯状态设置为忙
        wifi_spi_mutex = WIFI_SPI_BUSY;
        
        // 发起通讯查看模块内是否有数据未读取
        do
        {
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length  = 0;
            wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE);
            // 检查收到的包中是否有数据
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                // 保存接收到的数据
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
        }while(WIFI_SPI_REPLY_DATA_START == temp_packets.head.command);
        wifi_spi_mutex = WIFI_SPI_IDLE;
    }
#endif 
    
    fifo_read_buffer(&wifi_spi_fifo, buffer, &data_len, FIFO_READ_AND_CLEAN);
    return data_len;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WiFi 模块初始化
// 参数说明     *wifi_ssid      目标连接的 WiFi 的名称 字符串形式
// 参数说明     *pass_word      目标连接的 WiFi 的密码 字符串形式
// 返回参数     uint8           模块初始化状态 0-成功 1-错误
// 使用示例     wifi_spi_init("SEEKFREE", "SEEKFREE123");
// 备注信息     wifi_spi_init("SEEKFREE", NULL); // 连接没有密码的WIFI热点
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_init (char *wifi_ssid, char *pass_word)
{
    uint8 return_state = 0;
    
    fifo_init(&wifi_spi_fifo, FIFO_DATA_8BIT, wifi_spi_buffer, WIFI_SPI_RECVIVE_FIFO_SIZE);
    spi_init(WIFI_SPI_INDEX, SPI_MODE0, WIFI_SPI_SPEED, WIFI_SPI_SCK_PIN, WIFI_SPI_MOSI_PIN, WIFI_SPI_MISO_PIN, SPI_CS_NULL);//硬件SPI初始化
    gpio_init(WIFI_SPI_CS_PIN,  GPO, 1, GPO_PUSH_PULL);
    gpio_init(WIFI_SPI_RST_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(WIFI_SPI_INT_PIN, GPI, 0, GPI_PULL_DOWN);
    
    // 复位
    gpio_set_level(WIFI_SPI_RST_PIN, 0);
    system_delay_ms(10);
    gpio_set_level(WIFI_SPI_RST_PIN, 1);
    
    // 等待模块初始化
    system_delay_ms(100);
    wifi_spi_mutex = WIFI_SPI_IDLE;
    wifi_spi_tx_length = 0;
    wifi_spi_tx_pending = 0;
    wifi_spi_tx_dma_ready = 0;
    wifi_spi_tx_dma_busy = 0;
    wifi_spi_tx_step = WIFI_SPI_TX_STEP_IDLE;

    do
    {
        // 固件版本信息以字符串形式保存在wifi_spi_version数组中
        return_state = wifi_spi_get_version();
        if(return_state)
        {
            break;
        }

        // MAC地址信息以字符串形式保存在wifi_spi_mac_addr数组中
        wifi_spi_get_mac_addr();


        return_state = wifi_spi_wifi_connect(wifi_ssid, pass_word);
        if(return_state)
        {
            break;
        }
        
    #if(1 == WIFI_SPI_AUTO_CONNECT)
        return_state = wifi_spi_socket_connect("TCP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT);
        if(return_state)
        {
            break;
        }
    #endif
        
    #if(2 == WIFI_SPI_AUTO_CONNECT)
        return_state = wifi_spi_socket_connect("UDP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT);
        if(return_state)
        {
            break;
        }
    #endif
    }while(0);

    return return_state;
}
