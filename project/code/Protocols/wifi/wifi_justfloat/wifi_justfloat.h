/*****************************************************************************
 * 文件: wifi_justfloat.h
 * 模块: WiFi JustFloat 遥测
 * 职责: 通过 wifi_cmd 提供的发送接口输出 VOFA JustFloat 二进制遥测
 *****************************************************************************/

#ifndef WIFI_JUSTFLOAT_H
#define WIFI_JUSTFLOAT_H

#include "zf_common_headfile.h"

#define WIFI_JUSTFLOAT_MAX_FLOAT_NUM       (16U)  /* JustFloat 最大通道数 */

/* JustFloat 发送耗时统计结构体 */
typedef struct
{
    uint32_t last_us;     /* 最近一次发送耗时，单位 us */
    uint32_t min_us;      /* 最小发送耗时，单位 us */
    uint32_t max_us;      /* 最大发送耗时，单位 us */
    uint32_t avg_us;      /* 平均发送耗时，单位 us，仅统计成功发送 */
    uint32_t ok_count;    /* 成功发送次数 */
    uint32_t fail_count;  /* 发送失败次数 */
    uint32_t skip_count;  /* 被待机态 stop 抑制的次数 */
} wifi_justfloat_tx_stats_t;

/*
 * 函数名: wifi_justfloat_Init
 * 功能: 初始化 JustFloat 模块内部状态与统计信息
 * 输入参数: 无
 * 返回值: 无
 */
void wifi_justfloat_Init(void);

/*
 * 函数名: wifi_justfloat_IsReady
 * 功能: 查询当前 WiFi 遥测链路是否可发送
 * 输入参数: 无
 * 返回值:
 *   1 - 可发送
 *   0 - 不可发送
 */
uint8_t wifi_justfloat_IsReady(void);

/*
 * 函数名: wifi_justfloat_SetStandbyContext
 * 功能: 设置当前是否处于待机态上下文
 * 输入参数:
 *   is_standby - 1 表示待机态，0 表示非待机态
 * 返回值: 无
 */
void wifi_justfloat_SetStandbyContext(uint8_t is_standby);

/*
 * 函数名: wifi_justfloat_SetStandbyUserEnable
 * 功能: 设置用户是否允许待机态发送遥测
 * 输入参数:
 *   enable - 1 允许发送，0 禁止发送
 * 返回值: 无
 */
void wifi_justfloat_SetStandbyUserEnable(uint8_t enable);

/*
 * 函数名: wifi_justfloat_GetStandbyUserEnable
 * 功能: 查询用户当前是否允许待机态发送遥测
 * 输入参数: 无
 * 返回值:
 *   1 - 允许发送
 *   0 - 禁止发送
 */
uint8_t wifi_justfloat_GetStandbyUserEnable(void);

/*
 * 函数名: wifi_justfloat_ResetTxStats
 * 功能: 清空遥测发送耗时统计
 * 输入参数: 无
 * 返回值: 无
 */
void wifi_justfloat_ResetTxStats(void);

/*
 * 函数名: wifi_justfloat_GetTxStats
 * 功能: 读取遥测发送耗时统计
 * 输入参数:
 *   stats - 输出统计结构体指针
 * 返回值: 无
 */
void wifi_justfloat_GetTxStats(wifi_justfloat_tx_stats_t *stats);

/*
 * 函数名: wifi_justfloat_Impl
 * 功能: 实际发送 JustFloat 数据帧
 * 输入参数:
 *   declared_num - 调用者声明的通道数
 *   actual_num   - 宏展开后计算出的实际通道数
 *   ...          - 逐通道数据，按 double 传入
 * 返回值:
 *   0 - 发送成功
 *   1 - 发送失败
 */
uint8_t wifi_justfloat_Impl(uint8_t declared_num, uint8_t actual_num, ...);

#define WIFI_JUSTFLOAT_CALL_1(a1) \
    wifi_justfloat_Impl(1U, 1U, (double)(a1))
#define WIFI_JUSTFLOAT_CALL_2(a1, a2) \
    wifi_justfloat_Impl(2U, 2U, (double)(a1), (double)(a2))
#define WIFI_JUSTFLOAT_CALL_3(a1, a2, a3) \
    wifi_justfloat_Impl(3U, 3U, (double)(a1), (double)(a2), (double)(a3))
#define WIFI_JUSTFLOAT_CALL_4(a1, a2, a3, a4) \
    wifi_justfloat_Impl(4U, 4U, (double)(a1), (double)(a2), (double)(a3), (double)(a4))
#define WIFI_JUSTFLOAT_CALL_5(a1, a2, a3, a4, a5) \
    wifi_justfloat_Impl(5U, 5U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5))
#define WIFI_JUSTFLOAT_CALL_6(a1, a2, a3, a4, a5, a6) \
    wifi_justfloat_Impl(6U, 6U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6))
#define WIFI_JUSTFLOAT_CALL_7(a1, a2, a3, a4, a5, a6, a7) \
    wifi_justfloat_Impl(7U, 7U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7))
#define WIFI_JUSTFLOAT_CALL_8(a1, a2, a3, a4, a5, a6, a7, a8) \
    wifi_justfloat_Impl(8U, 8U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8))
#define WIFI_JUSTFLOAT_CALL_9(a1, a2, a3, a4, a5, a6, a7, a8, a9) \
    wifi_justfloat_Impl(9U, 9U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9))
#define WIFI_JUSTFLOAT_CALL_10(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10) \
    wifi_justfloat_Impl(10U, 10U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10))
#define WIFI_JUSTFLOAT_CALL_11(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11) \
    wifi_justfloat_Impl(11U, 11U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11))
#define WIFI_JUSTFLOAT_CALL_12(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12) \
    wifi_justfloat_Impl(12U, 12U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12))
#define WIFI_JUSTFLOAT_CALL_13(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13) \
    wifi_justfloat_Impl(13U, 13U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13))
#define WIFI_JUSTFLOAT_CALL_14(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14) \
    wifi_justfloat_Impl(14U, 14U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14))
#define WIFI_JUSTFLOAT_CALL_15(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15) \
    wifi_justfloat_Impl(15U, 15U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15))
#define WIFI_JUSTFLOAT_CALL_16(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16) \
    wifi_justfloat_Impl(16U, 16U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16))

#define WIFI_JUSTFLOAT_SELECT(_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,_11,_12,_13,_14,_15,_16,NAME,...) NAME


// 每次调用wifi_justfloat差不多花费10us
#define wifi_justfloat(...) \
    WIFI_JUSTFLOAT_SELECT(__VA_ARGS__, \
                          WIFI_JUSTFLOAT_CALL_16, WIFI_JUSTFLOAT_CALL_15, WIFI_JUSTFLOAT_CALL_14, WIFI_JUSTFLOAT_CALL_13, \
                          WIFI_JUSTFLOAT_CALL_12, WIFI_JUSTFLOAT_CALL_11, WIFI_JUSTFLOAT_CALL_10, WIFI_JUSTFLOAT_CALL_9, \
                          WIFI_JUSTFLOAT_CALL_8, WIFI_JUSTFLOAT_CALL_7, WIFI_JUSTFLOAT_CALL_6, WIFI_JUSTFLOAT_CALL_5, \
                          WIFI_JUSTFLOAT_CALL_4, WIFI_JUSTFLOAT_CALL_3, WIFI_JUSTFLOAT_CALL_2, WIFI_JUSTFLOAT_CALL_1)(__VA_ARGS__)

#endif /* WIFI_JUSTFLOAT_H */
