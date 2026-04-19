#ifndef CRSF_H
#define CRSF_H

#include "zf_common_headfile.h"
#include <stdint.h>
#define CRSF_CH_COUNT 10

typedef enum
{
	CRSF_CH_TYPE_AXIS_CENTER = 0,
	CRSF_CH_TYPE_THROTTLE,
	CRSF_CH_TYPE_SWITCH_2POS,
	CRSF_CH_TYPE_SWITCH_3POS,
	CRSF_CH_TYPE_BUTTON,
	CRSF_CH_TYPE_NONE
} CRSF_ChannelType;

typedef struct
{
	CRSF_ChannelType type;
	uint16_t min;
	uint16_t mid;
	uint16_t max;
	uint16_t low_th;
	uint16_t high_th;
} CRSF_ChannelConfig;

#define CRSF_MID(min, max)        ((uint16_t)(((min) + (max)) / 2U))
#define CRSF_THRESH_LOW(min, mid) ((uint16_t)(((min) + (mid)) / 2U))
#define CRSF_THRESH_HIGH(mid, max) ((uint16_t)(((mid) + (max)) / 2U))

static const CRSF_ChannelConfig CRSF_CHANNEL_CONFIG[CRSF_CH_COUNT] =
{
    // ROll -1000 ~ 1000
	{CRSF_CH_TYPE_AXIS_CENTER, 172, 992, 1810, CRSF_THRESH_LOW(172, 992), CRSF_THRESH_HIGH(992, 1810)},
    // Pitch -1000 ~ 1000
	{CRSF_CH_TYPE_AXIS_CENTER, 172, 992, 1810, CRSF_THRESH_LOW(172, 992), CRSF_THRESH_HIGH(992, 1810)},
    // Throttle 0 ~ 1000
	{CRSF_CH_TYPE_THROTTLE,    172, 992, 1810, CRSF_THRESH_LOW(172, 992), CRSF_THRESH_HIGH(992, 1810)},
	{CRSF_CH_TYPE_AXIS_CENTER, 172, 992, 1810, CRSF_THRESH_LOW(172, 992), CRSF_THRESH_HIGH(992, 1810)},  // CH3 Yaw
	{CRSF_CH_TYPE_SWITCH_2POS, 191, CRSF_MID(191, 1792), 1792, CRSF_THRESH_LOW(191, CRSF_MID(191, 1792)), CRSF_THRESH_HIGH(CRSF_MID(191, 1792), 1792)}, // CH4 2-pos
	{CRSF_CH_TYPE_SWITCH_3POS, 172, 992, 1810, CRSF_THRESH_LOW(172, 992), CRSF_THRESH_HIGH(992, 1810)},  // CH5 3-pos
	{CRSF_CH_TYPE_SWITCH_3POS, 172, 992, 1810, CRSF_THRESH_LOW(172, 992), CRSF_THRESH_HIGH(992, 1810)},  // CH6 3-pos
	{CRSF_CH_TYPE_SWITCH_2POS, 191, CRSF_MID(191, 1792), 1792, CRSF_THRESH_LOW(191, CRSF_MID(191, 1792)), CRSF_THRESH_HIGH(CRSF_MID(191, 1792), 1792)}, // CH7 2-pos
	{CRSF_CH_TYPE_BUTTON,      172, CRSF_MID(172, 1810), 1810, CRSF_THRESH_LOW(172, CRSF_MID(172, 1810)), CRSF_THRESH_HIGH(CRSF_MID(172, 1810), 1810)}, // CH8 Button
	{CRSF_CH_TYPE_NONE,        0,   0,   0,   0, 0}                                                           // CH9 Unused
};

extern volatile uint16_t CRSF_CH[CRSF_CH_COUNT];            // 接收的通道数据 0-2047 对应 11 位有效数�?
extern volatile int16_t CRSF_STD[CRSF_CH_COUNT];             // 
extern volatile uint32_t CRSF_LAST_UPDATE_TIME;             // 上次更新通道数据的时�?用于判断接收超时
extern volatile uint8_t CRSF_LINK_UP;                       // 接收链接状�?0表示链接断开 1表示链接正常

void crsf_init(void);

void CRSF_Update_100HZ(void);

void crsf_send_50hz(void);







#endif

