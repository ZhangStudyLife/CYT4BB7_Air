#ifndef WIFI_VOFA_H
#define WIFI_VOFA_H

#include "zf_common_headfile.h"
#include "../../HW_Drivers/Beep/Beep.h"

#ifndef WIFI_SSID_TEST
#define WIFI_SSID_TEST      "HDUASC_saidao"
#endif

#ifndef WIFI_PASSWORD_TEST
#define WIFI_PASSWORD_TEST  "zyz520520"
#endif

#ifndef TCP_TARGET_IP
#define TCP_TARGET_IP       "192.168.110.183"
#endif

#ifndef TCP_TARGET_PORT
#define TCP_TARGET_PORT     "8086"
#endif

#ifndef WIFI__LOCAL_PORT
#define WIFI__LOCAL_PORT    "6666"
#endif

#define WIFI_VOFA_CONNECT_WINDOW_MS   (10000U)
#define WIFI_VOFA_MAX_FLOAT_NUM       (16U)

/* 非阻塞初始化入口 */
void wifi_vofa_Init(void);

/* 连接状态查询：1-已连接可发送 0-未连接 */
uint8 wifi_vofa_IsReady(void);

/* JustFloat协议发送，首参数为数据个数，后续参数按double传入 */
uint8 wifi_vofa_JustFloat(uint8 data_num, ...);

#endif /* WIFI_VOFA_H */
