#include "wifi_vofa.h"
#include <stdarg.h>

#define WIFI_VOFA_JUSTFLOAT_TAIL_0    (0x00U)
#define WIFI_VOFA_JUSTFLOAT_TAIL_1    (0x00U)
#define WIFI_VOFA_JUSTFLOAT_TAIL_2    (0x80U)
#define WIFI_VOFA_JUSTFLOAT_TAIL_3    (0x7FU)

static uint8 s_wifi_vofa_ready = 0U;

void wifi_vofa_Init(void)
{
    uint8 ret;

    s_wifi_vofa_ready = 0U;

    /* 阻塞式初始化：先连WiFi，再连TCP */
    ret = wifi_spi_init((char *)WIFI_SSID_TEST, (char *)WIFI_PASSWORD_TEST);
    if (0U == ret)
    {
        ret = wifi_spi_socket_connect("TCP", (char *)TCP_TARGET_IP, (char *)TCP_TARGET_PORT, (char *)WIFI__LOCAL_PORT);
    }

    if (0U == ret)
    {
        s_wifi_vofa_ready = 1U;
    }
    else
    {
        /* 初始化失败提示：50%占空比，0.5s周期，10次 */
        Beep_Stop();
        Beep_Play(50U, 0.5f, 10U);
    }
}



uint8 wifi_vofa_IsReady(void)
{
    return s_wifi_vofa_ready;
}

uint8 wifi_vofa_JustFloat(uint8 data_num, ...)
{
    uint8 i;
    uint16 payload_len;
    uint16 frame_len;
    uint32 sent_len;
    uint8 frame[WIFI_VOFA_MAX_FLOAT_NUM * 4U + 4U];
    va_list ap;

    if ((0U == data_num) || (data_num > WIFI_VOFA_MAX_FLOAT_NUM))
    {
        return 1U;
    }

    if (0U == s_wifi_vofa_ready)
    {
        return 1U;
    }

    va_start(ap, data_num);
    for (i = 0U; i < data_num; i++)
    {
        float value_f = (float)va_arg(ap, double);
        memcpy(&frame[i * 4U], &value_f, sizeof(float));
    }
    va_end(ap);

    payload_len = (uint16)data_num * 4U;
    frame[payload_len + 0U] = WIFI_VOFA_JUSTFLOAT_TAIL_0;
    frame[payload_len + 1U] = WIFI_VOFA_JUSTFLOAT_TAIL_1;
    frame[payload_len + 2U] = WIFI_VOFA_JUSTFLOAT_TAIL_2;
    frame[payload_len + 3U] = WIFI_VOFA_JUSTFLOAT_TAIL_3;

    frame_len = payload_len + 4U;
    sent_len = wifi_spi_send_buffer(frame, frame_len);
    return (sent_len == frame_len) ? 0U : 1U;
}
