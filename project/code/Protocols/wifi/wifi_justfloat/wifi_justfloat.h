/*****************************************************************************
 * File: wifi_justfloat.h
 * Module: WiFi JustFloat telemetry
 * Purpose: output VOFA JustFloat binary frames through wifi_cmd
 *****************************************************************************/

#ifndef WIFI_JUSTFLOAT_H
#define WIFI_JUSTFLOAT_H

#include "zf_common_headfile.h"

#define WIFI_JUSTFLOAT_MAX_FLOAT_NUM       (41U)  /* Auto timestamp + up to 40 user channels */

/* JustFloat transmit time statistics */
typedef struct
{
    uint32_t last_us;     /* Last transmit cost, unit: us */
    uint32_t min_us;      /* Minimum transmit cost, unit: us */
    uint32_t max_us;      /* Maximum transmit cost, unit: us */
    uint32_t avg_us;      /* Average cost for successful transmissions, unit: us */
    uint32_t ok_count;    /* Successful transmit count */
    uint32_t fail_count;  /* Failed transmit count */
    uint32_t skip_count;  /* Skip count while standby sending is disabled */
    uint32_t queued_count; /* Successfully queued JustFloat frame count */
    uint32_t overflow_count; /* Dropped frame count when the RAM queue is full */
    uint32_t pending_bytes;  /* Bytes currently waiting in the RAM queue */
} wifi_justfloat_tx_stats_t;

void wifi_justfloat_Init(void);
uint8_t wifi_justfloat_IsReady(void);
void wifi_justfloat_SetStandbyContext(uint8_t is_standby);
void wifi_justfloat_SetStandbyUserEnable(uint8_t enable);
uint8_t wifi_justfloat_GetStandbyUserEnable(void);
void wifi_justfloat_ResetTxStats(void);
void wifi_justfloat_GetTxStats(wifi_justfloat_tx_stats_t *stats);
uint8_t wifi_justfloat_Poll(void);
uint8_t wifi_justfloat_Impl(uint8_t declared_num, uint8_t actual_num, ...);
uint8_t wifi_justfloat_Array(const float *data, uint8_t num);

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
#define WIFI_JUSTFLOAT_CALL_17(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17) \
    wifi_justfloat_Impl(17U, 17U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17))
#define WIFI_JUSTFLOAT_CALL_18(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18) \
    wifi_justfloat_Impl(18U, 18U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18))
#define WIFI_JUSTFLOAT_CALL_19(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19) \
    wifi_justfloat_Impl(19U, 19U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19))
#define WIFI_JUSTFLOAT_CALL_20(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20) \
    wifi_justfloat_Impl(20U, 20U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20))
#define WIFI_JUSTFLOAT_CALL_21(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21) \
    wifi_justfloat_Impl(21U, 21U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21))
#define WIFI_JUSTFLOAT_CALL_22(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22) \
    wifi_justfloat_Impl(22U, 22U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22))
#define WIFI_JUSTFLOAT_CALL_23(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23) \
    wifi_justfloat_Impl(23U, 23U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23))
#define WIFI_JUSTFLOAT_CALL_24(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24) \
    wifi_justfloat_Impl(24U, 24U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24))
#define WIFI_JUSTFLOAT_CALL_25(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25) \
    wifi_justfloat_Impl(25U, 25U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25))
#define WIFI_JUSTFLOAT_CALL_26(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26) \
    wifi_justfloat_Impl(26U, 26U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26))
#define WIFI_JUSTFLOAT_CALL_27(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27) \
    wifi_justfloat_Impl(27U, 27U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27))
#define WIFI_JUSTFLOAT_CALL_28(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28) \
    wifi_justfloat_Impl(28U, 28U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28))
#define WIFI_JUSTFLOAT_CALL_29(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29) \
    wifi_justfloat_Impl(29U, 29U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28), (double)(a29))
#define WIFI_JUSTFLOAT_CALL_30(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30) \
    wifi_justfloat_Impl(30U, 30U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28), (double)(a29), (double)(a30))
#define WIFI_JUSTFLOAT_CALL_31(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31) \
    wifi_justfloat_Impl(31U, 31U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28), (double)(a29), (double)(a30), (double)(a31))
#define WIFI_JUSTFLOAT_CALL_32(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32) \
    wifi_justfloat_Impl(32U, 32U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28), (double)(a29), (double)(a30), (double)(a31), (double)(a32))
#define WIFI_JUSTFLOAT_CALL_33(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33) \
    wifi_justfloat_Impl(33U, 33U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28), (double)(a29), (double)(a30), (double)(a31), (double)(a32), (double)(a33))
#define WIFI_JUSTFLOAT_CALL_34(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34) \
    wifi_justfloat_Impl(34U, 34U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28), (double)(a29), (double)(a30), (double)(a31), (double)(a32), (double)(a33), (double)(a34))
#define WIFI_JUSTFLOAT_CALL_35(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35) \
    wifi_justfloat_Impl(35U, 35U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28), (double)(a29), (double)(a30), (double)(a31), (double)(a32), (double)(a33), (double)(a34), (double)(a35))
#define WIFI_JUSTFLOAT_CALL_36(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36) \
    wifi_justfloat_Impl(36U, 36U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28), (double)(a29), (double)(a30), (double)(a31), (double)(a32), (double)(a33), (double)(a34), (double)(a35), (double)(a36))
#define WIFI_JUSTFLOAT_CALL_37(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37) \
    wifi_justfloat_Impl(37U, 37U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28), (double)(a29), (double)(a30), (double)(a31), (double)(a32), (double)(a33), (double)(a34), (double)(a35), (double)(a36), (double)(a37))
#define WIFI_JUSTFLOAT_CALL_38(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38) \
    wifi_justfloat_Impl(38U, 38U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28), (double)(a29), (double)(a30), (double)(a31), (double)(a32), (double)(a33), (double)(a34), (double)(a35), (double)(a36), (double)(a37), (double)(a38))
#define WIFI_JUSTFLOAT_CALL_39(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39) \
    wifi_justfloat_Impl(39U, 39U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28), (double)(a29), (double)(a30), (double)(a31), (double)(a32), (double)(a33), (double)(a34), (double)(a35), (double)(a36), (double)(a37), (double)(a38), (double)(a39))
#define WIFI_JUSTFLOAT_CALL_40(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40) \
    wifi_justfloat_Impl(40U, 40U, (double)(a1), (double)(a2), (double)(a3), (double)(a4), (double)(a5), (double)(a6), (double)(a7), (double)(a8), (double)(a9), (double)(a10), (double)(a11), (double)(a12), (double)(a13), (double)(a14), (double)(a15), (double)(a16), (double)(a17), (double)(a18), (double)(a19), (double)(a20), (double)(a21), (double)(a22), (double)(a23), (double)(a24), (double)(a25), (double)(a26), (double)(a27), (double)(a28), (double)(a29), (double)(a30), (double)(a31), (double)(a32), (double)(a33), (double)(a34), (double)(a35), (double)(a36), (double)(a37), (double)(a38), (double)(a39), (double)(a40))

#define WIFI_JUSTFLOAT_SELECT(_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,_11,_12,_13,_14,_15,_16,_17,_18,_19,_20,_21,_22,_23,_24,_25,_26,_27,_28,_29,_30,_31,_32,_33,_34,_35,_36,_37,_38,_39,_40,NAME,...) NAME


/* Each wifi_justfloat call costs about 10 us. */
#define wifi_justfloat(...) \
    WIFI_JUSTFLOAT_SELECT(__VA_ARGS__, \
                          WIFI_JUSTFLOAT_CALL_40, WIFI_JUSTFLOAT_CALL_39, WIFI_JUSTFLOAT_CALL_38, WIFI_JUSTFLOAT_CALL_37, \
                          WIFI_JUSTFLOAT_CALL_36, WIFI_JUSTFLOAT_CALL_35, WIFI_JUSTFLOAT_CALL_34, WIFI_JUSTFLOAT_CALL_33, \
                          WIFI_JUSTFLOAT_CALL_32, WIFI_JUSTFLOAT_CALL_31, WIFI_JUSTFLOAT_CALL_30, WIFI_JUSTFLOAT_CALL_29, \
                          WIFI_JUSTFLOAT_CALL_28, WIFI_JUSTFLOAT_CALL_27, WIFI_JUSTFLOAT_CALL_26, WIFI_JUSTFLOAT_CALL_25, \
                          WIFI_JUSTFLOAT_CALL_24, WIFI_JUSTFLOAT_CALL_23, WIFI_JUSTFLOAT_CALL_22, WIFI_JUSTFLOAT_CALL_21, \
                          WIFI_JUSTFLOAT_CALL_20, WIFI_JUSTFLOAT_CALL_19, WIFI_JUSTFLOAT_CALL_18, WIFI_JUSTFLOAT_CALL_17, \
                          WIFI_JUSTFLOAT_CALL_16, WIFI_JUSTFLOAT_CALL_15, WIFI_JUSTFLOAT_CALL_14, WIFI_JUSTFLOAT_CALL_13, \
                          WIFI_JUSTFLOAT_CALL_12, WIFI_JUSTFLOAT_CALL_11, WIFI_JUSTFLOAT_CALL_10, WIFI_JUSTFLOAT_CALL_9, \
                          WIFI_JUSTFLOAT_CALL_8, WIFI_JUSTFLOAT_CALL_7, WIFI_JUSTFLOAT_CALL_6, WIFI_JUSTFLOAT_CALL_5, \
                          WIFI_JUSTFLOAT_CALL_4, WIFI_JUSTFLOAT_CALL_3, WIFI_JUSTFLOAT_CALL_2, WIFI_JUSTFLOAT_CALL_1)(__VA_ARGS__)

#endif /* WIFI_JUSTFLOAT_H */
