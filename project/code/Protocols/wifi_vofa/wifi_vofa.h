#ifndef WIFI_VOFA_H
#define WIFI_VOFA_H

#include "zf_common_headfile.h"
#include "../../HW_Drivers/Beep/Beep.h"

/*
 * wifi_vofa 模块：通过 WiFi-UDP 向 VOFA+ 发送 JustFloat 浮点数据
 * 说明：
 * - 传输协议：UDP（低时延，适合实时遥测）
 * - 当前目标上位机IP：192.168.110.183
 * - wifispi模块在路由器显示的ip是192.168.110.218
 */

#ifndef WIFI_SSID_TEST
#define WIFI_SSID_TEST      "HDUASC_saidao"
#endif

#ifndef WIFI_PASSWORD_TEST
#define WIFI_PASSWORD_TEST  "zyz520520"
#endif

#ifndef UDP_REMOTE_IP
/* UDP远端IP（VOFA+接收端） */
#define UDP_REMOTE_IP       "192.168.110.183"
#endif

#ifndef UDP_REMOTE_PORT
/* UDP远端端口（VOFA+接收端端口） */
#define UDP_REMOTE_PORT     "1347"
#endif

#ifndef UDP_LOCAL_PORT
/* 本机UDP本地端口 */
#define UDP_LOCAL_PORT      "1346"
#endif

#define WIFI_VOFA_CONNECT_WINDOW_MS   (10000U)
#define WIFI_VOFA_MAX_FLOAT_NUM       (16U)

typedef struct
{
    uint32 last_us;
    uint32 min_us;
    uint32 max_us;
    uint32 avg_us;
    uint32 ok_count;
    uint32 fail_count;
} wifi_vofa_tx_stats_t;

/* 阻塞初始化：连接WiFi并建立UDP socket */
void wifi_vofa_Init(void);

/* 连接状态查询：1-已连接可发送 0-未连接 */
uint8 wifi_vofa_IsReady(void);

/* JustFloat发送：首参数为通道数，后续参数按double传入（内部转float） */
uint8 wifi_vofa_JustFloat(uint8 data_num, ...);

/* 清空发送耗时统计 */
void wifi_vofa_ResetTxStats(void);

/* 读取发送耗时统计（最近/最小/最大/平均/成功/失败） */
void wifi_vofa_GetTxStats(wifi_vofa_tx_stats_t *stats);

#endif /* WIFI_VOFA_H */
