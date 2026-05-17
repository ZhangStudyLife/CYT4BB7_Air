/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          zf_device_wifi_spi
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2024-01-18        pudding            first version
* 2025-06-23        pudding            �޸���������������쳣������
********************************************************************************************************************/
/*********************************************************************************************************************
* ���߶��壺
*                   ------------------------------------
*                   ģ��ܽ�            ��Ƭ���ܽ�
*                   RST                 �鿴 zf_device_wifi_spi.h �� WIFI_SPI_RST_PIN �궨��
*                   INT                 �鿴 zf_device_wifi_spi.h �� WIFI_SPI_INT_PIN �궨��
*                   CS                  �鿴 zf_device_wifi_spi.h �� WIFI_SPI_CS_PIN �궨��
*                   MISO                �鿴 zf_device_wifi_spi.h �� WIFI_SPI_MISO_PIN �궨��
*                   SCK                 �鿴 zf_device_wifi_spi.h �� WIFI_SPI_SCK_PIN �궨��
*                   MOSI                �鿴 zf_device_wifi_spi.h �� WIFI_SPI_MOSI_PIN �궨��
*                   5V                  5V ��Դ
*                   GND                 ��Դ��
*                   ������������
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

#define WIFI_CONNECT_TIME_OUT       10000       // ��λ����
#define SOCKET_CONNECT_TIME_OUT     50000       // ��λ����
#define OTHER_TIME_OUT              1000        // ��λ����
#define WIFI_SPI_TX_DMA_CHANNEL         (30u)   // WiFi SPI(SCB7) TX�����Ӧ DW1 ͨ����
#define WIFI_SPI_TX_DMA_TRIGGER_1TO1    (TRIG_OUT_1TO1_2_SCB_TX_TO_PDMA17) // SCB7 TX -> DW1_TR_IN[30]
#define WIFI_SPI_TX_DMA_USE_SW_TRIGGER  (1u)    // DMA������ʽ��0-Ӳ������ 1-��������

char wifi_spi_version[12];                      // ����ģ��̼��汾��Ϣ
char wifi_spi_mac_addr[20];                     // ����ģ��MAC��ַ��Ϣ
char wifi_spi_ip_addr_port[25];                 // ����ģ��IP��ַ��˿���Ϣ

static fifo_struct  wifi_spi_fifo;
static uint8        wifi_spi_buffer[WIFI_SPI_RECVIVE_FIFO_SIZE];
static volatile     wifi_spi_state_enum wifi_spi_mutex;
/* ���������ͻ��棺������������ݣ������ϲ���ʱ������ʧЧ */
static uint8        wifi_spi_tx_cache[WIFI_SPI_TRANSFER_SIZE];
/* ���������ͻ�����Ч���ȣ���λ�ֽ� */
static uint16       wifi_spi_tx_length = 0;
/* �����������Ƿ����ύ�Ҵ�������1-�д�������0-�޴������� */
static uint8        wifi_spi_tx_pending = 0;
/* WiFi SPI TX DMA �Ƿ��ʼ���ɹ� */
static uint8        wifi_spi_tx_dma_ready = 0;
/* WiFi SPI TX DMA �Ƿ����ڴ��� */
static uint8        wifi_spi_tx_dma_busy = 0;
/* WiFi SPI TX DMA ������ */
static cy_stc_pdma_descr_t wifi_spi_tx_dma_descr;

/* ������ѯ�ڲ�����״̬ */
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

/* ��ǰ������ѯ���� */
static wifi_spi_tx_step_enum wifi_spi_tx_step = WIFI_SPI_TX_STEP_IDLE;
/* WiFi SPI ��Ӧ SCB ����ַ��WIFI_SPI_INDEX �̶�Ϊ SPI_0����Ӧ SCB7�� */
static volatile stc_SCB_t * const s_wifi_spi_scb_lut[4] = {SCB7, SCB8, SCB9, SCB6};
#define WIFI_SPI_SCB (s_wifi_spi_scb_lut[WIFI_SPI_INDEX])

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ѯ WIFI SPI �����Ƿ����
// ����˵��     void
// ���ز���     uint8           ״̬ 1-������� 0-����δ���
// ʹ��ʾ��     �ڲ�ʹ�ã��û��������
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_tx_is_complete (void)
{
    return (0 != Cy_SCB_IsTxComplete(WIFI_SPI_SCB)) ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��� WIFI SPI RX FIFO
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     �ڲ�ʹ�ã��û��������
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
static void wifi_spi_rx_fifo_drain (void)
{
    while(0 != Cy_SCB_GetNumInRxFifo(WIFI_SPI_SCB))
    {
        (void)Cy_SCB_ReadRxFifo(WIFI_SPI_SCB);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     WiFi SPI TX DMA ��ʼ��
// ����˵��     void
// ���ز���     uint8           ״̬ 0-�ɹ� 1-����
// ʹ��ʾ��     �ڲ�ʹ�ã��û��������
// ��ע��Ϣ
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
// �������     ����һ�� WiFi SPI TX DMA ����
// ����˵��     *data           ���ݵ�ַ
// ����˵��     len             ���ݳ���
// ���ز���     uint8           ״̬ 0-�����ɹ� 1-����ʧ��
// ʹ��ʾ��     �ڲ�ʹ�ã��û��������
// ��ע��Ϣ
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
        tx_descr_config.destAddr       = (void *)&(WIFI_SPI_SCB->unTX_FIFO_WR.u32Register);
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
// �������     ��ѯ WiFi SPI TX DMA �Ƿ����
// ����˵��     void
// ���ز���     uint8           ״̬ 1-��� 0-δ���
// ʹ��ʾ��     �ڲ�ʹ�ã��û��������
// ��ע��Ϣ
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
// �������     �ȴ�WIFI SPI����
// ����˵��     wait_time       ���ȴ�ʱ�� ��λ����
// ���ز���     uint8           ״̬ 0-�ɹ� 1-����
// ʹ��ʾ��     �ڲ�ʹ�ã��û��������
// ��ע��Ϣ
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
// �������     д�����ݵ�WIFI SPI
// ����˵��     *buffer1        ��һ����Ҫ���͵����ݻ�������ַ
// ����˵��     length1         ��һ�����ݳ���
// ����˵��     *buffer2        �ڶ�����Ҫ���͵����ݻ�������ַ
// ����˵��     length2         �ڶ������ݳ���
// ���ز���     void
// ʹ��ʾ��     �ڲ�ʹ�ã��û��������
// ��ע��Ϣ
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
// �������     WIFI SPI ���������ͬʱ���У������շ���
// ����˵��     *packets        ��������յĵ�ַ
// ����˵��     length          ��Ҫ���յĳ���
// ���ز���     void
// ʹ��ʾ��     �ڲ�ʹ�ã��û��������
// ��ע��Ϣ
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
// �������     WIFI SPI ���������ͬʱ����(�����շ�)
// ����˵��     *write_data     ���͵����ݻ�������ַ
// ����˵��     *read_data      ���յ������ݵĴ洢��ַ
// ����˵��     length          ��Ҫ���յĳ���
// ���ز���     void
// ʹ��ʾ��     �ڲ�ʹ�ã��û��������
// ��ע��Ϣ
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
        // ����Ҫ���͵����ݿ�������ȡ���������������write_dataԽ�����
        memcpy(read_data->buffer, write_data, length);
        spi_transfer_8bit(WIFI_SPI_INDEX, read_data->buffer, read_data->buffer, WIFI_SPI_RECVIVE_SIZE);
    }
    gpio_high(WIFI_SPI_CS_PIN);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     WIFI SPI ��������
// ����˵��     command         ��������
// ����˵��     *buffer         ������ַ
// ����˵��     length          ��������
// ����˵��     wait_time       ���ȴ�ʱ�� ��λ100΢��
// ���ز���     uint8           ״̬ 0-�ɹ� 1-����
// ʹ��ʾ��     �ڲ�ʹ�ã��û��������
// ��ע��Ϣ
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
        
        // �ȴ��ӻ�׼������
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }

        wifi_spi_write(&head.command, sizeof(wifi_spi_head_struct), buffer, length);
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }
        // ����Ӧ���ź�

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
// �������     WIFI SPI ģ����Ϣ��ȡ
// ����˵��     command         ��������
// ����˵��     *buffer         ������յ��Ĳ�����ַ
// ����˵��     wait_time       ���ȴ�ʱ�� ��λ100΢��
// ���ز���     uint8           ״̬ 0-�ɹ� 1-����
// ʹ��ʾ��     �ڲ�ʹ�ã��û��������
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_parameter (wifi_spi_packets_command_enum command, wifi_spi_packets_struct *read_data, uint32 wait_time)
{
    uint8 return_state;

    return_state = 1;
    do
    {
        // �ȴ��ӻ�׼������
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
// �������     WIFI SPI �̼��汾��ȡ
// ����˵��     void            �˿ں�
// ���ز���     uint8           ״̬ 0-�ɹ� 1-����
// ʹ��ʾ��
// ��ע��Ϣ     ���ú���֮�󣬹̼��汾��Ϣ���ַ�����ʽ������wifi_spi_version������
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
// �������     WIFI SPI MAC��ַ��ȡ
// ����˵��     void            �˿ں�
// ���ز���     uint8           ״̬ 0-�ɹ� 1-����
// ʹ��ʾ��
// ��ע��Ϣ     ���ú���֮��MAC��ַ��Ϣ���ַ�����ʽ������wifi_spi_mac_addr������
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
// �������     WIFI SPI IP��ַ��˿ںŻ�ȡ
// ����˵��     void            �˿ں�
// ���ز���     uint8           ״̬ 0-�ɹ� 1-����
// ʹ��ʾ��
// ��ע��Ϣ     ���ú���֮��IP��ַ��˿ں���Ϣ���ַ�����ʽ������wifi_spi_ip_addr_port������
//              ��Ҫ������Socket֮����ô˺�������������ȡ��Ϣ
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
// �������     WIFI SPI �������ӵ�WiFi��Ϣ����������WiFi
// ����˵��     *wifi_ssid      WIFI����
// ����˵��     *pass_word      WIFI����
// ���ز���     uint8           ״̬ 0-�ɹ� 1-����
// ʹ��ʾ��     wifi_spi_wifi_connect("SEEKFREE", "SEEKFREE123");
// ��ע��Ϣ     wifi_spi_wifi_connect("SEEKFREE", NULL); // ����û�������WIFI�ȵ�
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_wifi_connect (char *wifi_ssid, char *pass_word)
{
    uint8 return_state;
    uint8 temp_buffer[64];
    uint16 length;
    
    if(NULL != pass_word)
    {
        // WIFI�ȵ������뷢���ȵ�����������
        length = (uint16)sprintf((char *)temp_buffer, "%s\r\n%s\r\n", wifi_ssid, pass_word);
    }
    else
    {
        // WIFI�ȵ�û������ֻ��Ҫ�����ȵ�����
        length = (uint16)sprintf((char *)temp_buffer, "%s\r\n", wifi_ssid);
    }

    return_state = wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_INFORMATION, temp_buffer, length, WIFI_CONNECT_TIME_OUT);

    // ����IP��ַ��˿ں���Ϣ���ַ�����ʽ������wifi_spi_ip_addr_port������
    wifi_spi_get_ip_addr_port();

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     WIFI SPI �������ӵ�Socket��Ϣ����������Socket
// ����˵��     *transport_type ��������
// ����˵��     *ip_addr        IP��ַ
// ����˵��     *port           Ŀ��˿ں�
// ����˵��     *local_port     �����˿ں�
// ���ز���     uint8           ״̬ 0-�ɹ� 1-����
// ʹ��ʾ��     wifi_spi_socket_connect("TCP", "192.168.2.5", "8080", "6060");
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_socket_connect (char *transport_type, char *ip_addr, char *port, char *local_port)
{
    uint8 return_state;
    uint8 temp_buffer[41];
    uint16 length;
    
    length = (uint16)sprintf((char *)temp_buffer, "%s\r\n%s\r\n%s\r\n%s\r\n", transport_type, ip_addr, port, local_port);

    return_state = wifi_spi_set_parameter(WIFI_SPI_SET_SOCKET_INFORMATION, temp_buffer, length, SOCKET_CONNECT_TIME_OUT);

    // ����IP��ַ��˿ں���Ϣ���ַ�����ʽ������wifi_spi_ip_addr_port������
    wifi_spi_get_ip_addr_port();

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     WIFI SPI �Ͽ�Socket����
// ����˵��     void
// ���ز���     uint8           ״̬ 0-�ɹ� 1-����
// ʹ��ʾ��     wifi_spi_socket_disconnect();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_socket_disconnect (void)
{
    wifi_spi_packets_struct temp_packets;

    return wifi_spi_get_parameter(WIFI_SPI_CLOSE_SOCKET, &temp_packets, OTHER_TIME_OUT);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     WIFI SPI ����λ
// ����˵��     void
// ���ز���     uint8           ״̬ 0-�ɹ� 1-����
// ʹ��ʾ��
// ��ע��Ϣ
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
// �������     WIFI SPI UDPģʽʱ�������ͺ���
// ����˵��     void
// ���ز���     uint8           ״̬ 0-�ɹ� 1-����
// ʹ��ʾ��
// ��ע��Ϣ     ��UDPģʽ��ģ���յ����ݺ��ȴ�2���룬2�����δ�յ�����������ͨ��socket���͵����磬���ϣ�����������������ݴ�����Ϻ���ô˺���
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_udp_send_now (void)
{
    uint8 return_state = 1;
    wifi_spi_packets_struct temp_packets;
    
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        // ��ͨѶ״̬����Ϊæ
        wifi_spi_mutex = WIFI_SPI_BUSY;
        do
        {
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }

            // ������ʼsocket����
            temp_packets.head.command = WIFI_SPI_UDP_SEND;
            temp_packets.head.length = 0;
            wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE);
            
            // ����յ��İ����Ƿ�������
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                // ������յ�������
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
            
            // �ȴ�Ӧ���ź�
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            
            // ����Ӧ���ź�
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length = 0;
            wifi_spi_transfer_command(&temp_packets, temp_packets.head.length);
            
            if(WIFI_SPI_REPLY_OK == temp_packets.head.command)
            {
                return_state = 0;
            }
            
        }while(0);
        
        // ��ͨѶ״̬����Ϊ����
        wifi_spi_mutex = WIFI_SPI_IDLE;
    } 
    
    return return_state;
}

/*
 * �������ܣ���ѯ WiFi SPI ��ǰ�Ƿ����з���������ִ�С�
 * ����������ޡ�
 * ����ֵ��1-����æ��0-���С�
 */
uint8 wifi_spi_is_busy (void)
{
    return (WIFI_SPI_BUSY == wifi_spi_mutex) ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     WIFI SPI ���ݿ鷢�ͺ������ύģʽ������� wifi_spi_send_poll �ƽ���
// ����˵��     *buff           ��Ҫ���͵����ݵ�ַ
// ����˵��     length          ���ͳ���
// ���ز���     uint32          δ�ύ����
// ʹ��ʾ��     wifi_spi_send_buffer(buffer, 100);
// ��ע��Ϣ
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
 * �������ܣ��ƽ�һ�� WiFi SPI ����������״̬����
 * ����������ޡ�
 * ����ֵ���ޡ�
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
            if (0 == gpio_get_level(WIFI_SPI_INT_PIN))
            {
                wifi_spi_tx_pending = 0;
                wifi_spi_tx_length = 0;
                wifi_spi_tx_step = WIFI_SPI_TX_STEP_IDLE;
                wifi_spi_mutex = WIFI_SPI_IDLE;
                return;
            }
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length  = wifi_spi_tx_length;
            gpio_low(WIFI_SPI_CS_PIN);
            spi_write_8bit_array(WIFI_SPI_INDEX, &temp_packets.head.command, sizeof(wifi_spi_head_struct));
            /* SPI ȫ˫�������ſ�ͷ�����Ͳ�����RX�ع࣬�������DMA����RX FIFO���� */
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
                /* DMA�����ڼ�����ſ�RX FIFO����ֹȫ˫���ع�������·���ͣ�� */
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
// �������     WIFI SPI ��ȡ������
// ����˵��     *buff           ���ջ�����
// ����˵��     length          ��ȡ���ݳ���
// ���ز���     uint32          ʵ�ʶ�ȡ���ݳ���
// ʹ��ʾ��     wifi_spi_read_buffer(buffer, 100);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint32 wifi_spi_read_buffer (uint8 *buffer, uint32 length)
{
    zf_assert(NULL != buffer);
    uint32 data_len = length;
    
#if(1 == WIFI_SPI_READ_TRANSFER)
    
    wifi_spi_packets_struct temp_packets;
    // ���WIFI SPI״̬������������жϻ����߳����Ѿ�������ͨѶ���򱾴β��ܷ�������
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        // ��ͨѶ״̬����Ϊæ
        wifi_spi_mutex = WIFI_SPI_BUSY;
        
        // ����ͨѶ�鿴ģ�����Ƿ�������δ��ȡ
        do
        {
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length  = 0;
            wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE);
            // ����յ��İ����Ƿ�������
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                // ������յ�������
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
// �������     WiFi ģ���ʼ��
// ����˵��     *wifi_ssid      Ŀ�����ӵ� WiFi ������ �ַ�����ʽ
// ����˵��     *pass_word      Ŀ�����ӵ� WiFi ������ �ַ�����ʽ
// ���ز���     uint8           ģ���ʼ��״̬ 0-�ɹ� 1-����
// ʹ��ʾ��     wifi_spi_init("SEEKFREE", "SEEKFREE123");
// ��ע��Ϣ     wifi_spi_init("SEEKFREE", NULL); // ����û�������WIFI�ȵ�
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_init (char *wifi_ssid, char *pass_word)
{
    uint8 return_state = 0;
    
    fifo_init(&wifi_spi_fifo, FIFO_DATA_8BIT, wifi_spi_buffer, WIFI_SPI_RECVIVE_FIFO_SIZE);
    spi_init(WIFI_SPI_INDEX, SPI_MODE0, WIFI_SPI_SPEED, WIFI_SPI_SCK_PIN, WIFI_SPI_MOSI_PIN, WIFI_SPI_MISO_PIN, SPI_CS_NULL);//Ӳ��SPI��ʼ��
    gpio_init(WIFI_SPI_CS_PIN,  GPO, 1, GPO_PUSH_PULL);
    gpio_init(WIFI_SPI_RST_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(WIFI_SPI_INT_PIN, GPI, 0, GPI_PULL_DOWN);
    
    // ��λ
    gpio_set_level(WIFI_SPI_RST_PIN, 0);
    system_delay_ms(10);
    gpio_set_level(WIFI_SPI_RST_PIN, 1);
    
    // �ȴ�ģ���ʼ��
    system_delay_ms(100);
    wifi_spi_mutex = WIFI_SPI_IDLE;
    wifi_spi_tx_length = 0;
    wifi_spi_tx_pending = 0;
    wifi_spi_tx_dma_ready = 0;
    wifi_spi_tx_dma_busy = 0;
    wifi_spi_tx_step = WIFI_SPI_TX_STEP_IDLE;

    do
    {
        // �̼��汾��Ϣ���ַ�����ʽ������wifi_spi_version������
        return_state = wifi_spi_get_version();
        if(return_state)
        {
            break;
        }

        // MAC��ַ��Ϣ���ַ�����ʽ������wifi_spi_mac_addr������
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
