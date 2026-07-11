#ifndef LIGHT_SEQUENCE_CONFIG_H
#define LIGHT_SEQUENCE_CONFIG_H

#include "zf_common_headfile.h"

#define LIGHT_SEQUENCE_BEACON_COUNT       (7U)    /* 场地信标灯数量 */
#define LIGHT_SEQUENCE_PRESET_COUNT       (10U)   /* 预设亮灯序列数量 */
#define LIGHT_SEQUENCE_MAX_ROUNDS         (40U)   /* 单套序列最多保存的轮数 */
#define LIGHT_SEQUENCE_MATCH_RADIUS_M     (0.5f)  /* 熄灯事件坐标匹配半径，单位m */
#define LIGHT_SEQUENCE_DUPLICATE_WINDOW_MS (1000U) /* 同一灯号的时间去重窗口，单位ms */

/**
 * @brief 二维场地坐标。
 */
typedef struct
{
    float x;  /* 全局X坐标，单位m */
    float y;  /* 全局Y坐标，单位m */
} light_sequence_point_t;

/**
 * @brief 一套预设亮灯序列。
 */
typedef struct
{
    uint8 round_count;                                  /* 有效轮数 */
    uint8 round_mask[LIGHT_SEQUENCE_MAX_ROUNDS];        /* 各轮亮灯位掩码，bit0对应1号灯 */
} light_sequence_preset_t;

/* 7个信标灯中心的场地坐标，数组下标0对应1号灯。 */
extern const light_sequence_point_t g_light_sequence_beacon_positions[LIGHT_SEQUENCE_BEACON_COUNT];

/* 车辆发车位置的场地坐标，单位m。 */
extern const light_sequence_point_t g_light_sequence_initial_position;

/* 10套预设亮灯序列配置。 */
extern const light_sequence_preset_t g_light_sequence_presets[LIGHT_SEQUENCE_PRESET_COUNT];

#endif /* LIGHT_SEQUENCE_CONFIG_H */
