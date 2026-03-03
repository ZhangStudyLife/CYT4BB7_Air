#ifndef WIFI_VOFA_H
#define WIFI_VOFA_H

#include "zf_common_headfile.h"
#include "../../HW_Drivers/Beep/Beep.h"

/*
 * wifi_vofa 模块：通过 WiFi-UDP �?VOFA+ 发�?JustFloat 浮点数据
 * 说明�? * - 传输协议：UDP（低时延，适合实时遥测�? * - 当前目标上位机IP�?92.168.110.183
 * - wifispi模块在路由器显示的ip�?92.168.110.218
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

#define WIFI_VOFA_CAST_1(a1) (double)(a1)
#define WIFI_VOFA_CAST_2(a1,a2) (double)(a1), (double)(a2)
#define WIFI_VOFA_CAST_3(a1,a2,a3) (double)(a1), (double)(a2), (double)(a3)
#define WIFI_VOFA_CAST_4(a1,a2,a3,a4) (double)(a1), (double)(a2), (double)(a3), (double)(a4)
#define WIFI_VOFA_CAST_5(a1,a2,a3,a4,a5) (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5)
#define WIFI_VOFA_CAST_6(a1,a2,a3,a4,a5,a6) (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6)
#define WIFI_VOFA_CAST_7(a1,a2,a3,a4,a5,a6,a7) (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7)
#define WIFI_VOFA_CAST_8(a1,a2,a3,a4,a5,a6,a7,a8) (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8)
#define WIFI_VOFA_CAST_9(a1,a2,a3,a4,a5,a6,a7,a8,a9) (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9)
#define WIFI_VOFA_CAST_10(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10) (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10)
#define WIFI_VOFA_CAST_11(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11) (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11)
#define WIFI_VOFA_CAST_12(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12) (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12)
#define WIFI_VOFA_CAST_13(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13) (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13)
#define WIFI_VOFA_CAST_14(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14) (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14)
#define WIFI_VOFA_CAST_15(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15) (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15)
#define WIFI_VOFA_CAST_16(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16) (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16)

#define WIFI_VOFA_PP_ARG_N(_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,_11,_12,_13,_14,_15,_16,N,...) N
#define WIFI_VOFA_PP_RSEQ_N() 16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0
#define WIFI_VOFA_PP_NARG_(...) WIFI_VOFA_PP_ARG_N(__VA_ARGS__)
#define WIFI_VOFA_PP_NARG(...) WIFI_VOFA_PP_NARG_(__VA_ARGS__, WIFI_VOFA_PP_RSEQ_N())

#define WIFI_VOFA_CAST_DISPATCH_(n, ...) WIFI_VOFA_CAST_##n(__VA_ARGS__)
#define WIFI_VOFA_CAST_DISPATCH(n, ...) WIFI_VOFA_CAST_DISPATCH_(n, __VA_ARGS__)

/* 阻塞初始化：连接WiFi并建立UDP socket */
void wifi_vofa_Init(void);

/* 连接状态查询：1-已连接可发�?0-未连�?*/
uint8 wifi_vofa_IsReady(void);

/* JustFloat发送：首参数为通道数，后续参数按double传入（内部转float�?*/
uint8 wifi_vofa_JustFloat_Impl(uint8 data_num, ...);

#define wifi_vofa_JustFloat(data_num, ...) \
    wifi_vofa_JustFloat_Impl((data_num), \
        WIFI_VOFA_CAST_DISPATCH(WIFI_VOFA_PP_NARG(__VA_ARGS__), __VA_ARGS__))

/* 清空发送耗时统计 */
void wifi_vofa_ResetTxStats(void);

/* 读取发送耗时统计（最�?最�?最�?平均/成功/失败�?*/
void wifi_vofa_GetTxStats(wifi_vofa_tx_stats_t *stats);

#endif /* WIFI_VOFA_H */
