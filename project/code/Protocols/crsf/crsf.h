/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
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

extern volatile uint16_t CRSF_CH[CRSF_CH_COUNT];
extern volatile int16_t CRSF_STD[CRSF_CH_COUNT];
extern volatile uint32_t CRSF_LAST_UPDATE_TIME;
extern volatile uint8_t CRSF_LINK_UP;

void crsf_init(void);

void CRSF_Update_100HZ(void);

/* 生成并尝试提交10Hz姿态回传帧，硬件FIFO忙时直接丢弃。 */
void crsf_send_10hz(void);







#endif

