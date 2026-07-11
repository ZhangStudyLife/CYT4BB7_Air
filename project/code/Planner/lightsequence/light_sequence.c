#include "light_sequence.h"

#include <string.h>

#include "light_sequence_config.h"

#define LIGHT_SEQUENCE_VALID_BEACON_MASK       (0x7FU)  /* 7盏信标灯对应的有效位掩码 */
#define LIGHT_SEQUENCE_MAX_LIGHTS_PER_ROUND    (3U)     /* 每轮允许同时亮起的最大灯数 */

typedef struct
{
    uint8 valid;
    uint8 round_index;
    uint8 remaining_mask;
} light_sequence_candidate_t;

/* 10套序列各自的当前筛选进度。 */
static light_sequence_candidate_t s_candidates[LIGHT_SEQUENCE_PRESET_COUNT];

/* 当前灯号与序列识别结果。 */
static light_sequence_result_t s_result;

/* 上一次100Hz更新时的熄灯标志，用于检测0到1上升沿。 */
static uint8 s_last_beacon_lost_flag;

/**
 * @brief 统计一个灯位掩码中置位的数量。
 * @param mask 待统计的灯位掩码。
 * @return 置位数量。
 */
static uint8 LightSequence_CountBits(uint8 mask)
{
    uint8 count = 0U;

    while(mask != 0U)
    {
        count = (uint8)(count + (mask & 1U));
        mask >>= 1U;
    }

    return count;
}

/**
 * @brief 检查一套预设序列的轮数和各轮灯位掩码是否合法。
 * @param preset 待检查的预设序列指针。
 * @return 1表示配置有效，0表示配置无效。
 */
static uint8 LightSequence_PresetValid(const light_sequence_preset_t *preset)
{
    uint8 round_index;

    if((preset == 0) ||
       (preset->round_count == 0U) ||
       (preset->round_count > LIGHT_SEQUENCE_MAX_ROUNDS))
    {
        return 0U;
    }

    for(round_index = 0U; round_index < preset->round_count; round_index++)
    {
        uint8 round_mask = preset->round_mask[round_index];
        uint8 light_count = LightSequence_CountBits(round_mask);

        if((round_mask == 0U) ||
           ((round_mask & (uint8)(~LIGHT_SEQUENCE_VALID_BEACON_MASK)) != 0U) ||
           (light_count > LIGHT_SEQUENCE_MAX_LIGHTS_PER_ROUND))
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief 根据车辆坐标查找匹配半径内最近的信标灯。
 * @param car_position_x 车辆最新全局X坐标，单位m。
 * @param car_position_y 车辆最新全局Y坐标，单位m。
 * @return 匹配成功返回1至7号灯，匹配失败返回0。
 */
static uint8 LightSequence_FindNearestBeacon(float car_position_x,
                                             float car_position_y)
{
    const float match_radius_sq =
        LIGHT_SEQUENCE_MATCH_RADIUS_M * LIGHT_SEQUENCE_MATCH_RADIUS_M;
    float nearest_distance_sq = 0.0f;
    uint8 nearest_beacon_id = LIGHT_SEQUENCE_BEACON_ID_NONE;
    uint8 beacon_index;

    for(beacon_index = 0U; beacon_index < LIGHT_SEQUENCE_BEACON_COUNT; beacon_index++)
    {
        float dx = car_position_x - g_light_sequence_beacon_positions[beacon_index].x;
        float dy = car_position_y - g_light_sequence_beacon_positions[beacon_index].y;
        float distance_sq = (dx * dx) + (dy * dy);

        if((nearest_beacon_id == LIGHT_SEQUENCE_BEACON_ID_NONE) ||
           (distance_sq < nearest_distance_sq))
        {
            nearest_distance_sq = distance_sq;
            nearest_beacon_id = (uint8)(beacon_index + 1U);
        }
    }

    if((nearest_beacon_id == LIGHT_SEQUENCE_BEACON_ID_NONE) ||
       (nearest_distance_sq > match_radius_sq))
    {
        return LIGHT_SEQUENCE_BEACON_ID_NONE;
    }

    return nearest_beacon_id;
}

/**
 * @brief 根据有效候选数量刷新序列编号和识别状态。
 * @param 无。
 * @return 无。
 */
static void LightSequence_RefreshResult(void)
{
    uint16 candidate_mask = 0U;
    uint8 candidate_count = 0U;
    uint8 sequence_id = LIGHT_SEQUENCE_SEQUENCE_ID_UNKNOWN;
    uint8 sequence_index;

    for(sequence_index = 0U; sequence_index < LIGHT_SEQUENCE_PRESET_COUNT; sequence_index++)
    {
        if(s_candidates[sequence_index].valid != 0U)
        {
            candidate_mask =
                (uint16)(candidate_mask | (uint16)(1U << sequence_index));
            candidate_count++;
            sequence_id = (uint8)(sequence_index + 1U);
        }
    }

    s_result.candidate_mask = candidate_mask;
    s_result.candidate_count = candidate_count;
    if(candidate_count == 0U)
    {
        s_result.status = LIGHT_SEQUENCE_STATUS_FAILED;
        s_result.sequence_id = LIGHT_SEQUENCE_SEQUENCE_ID_UNKNOWN;
    }
    else if(candidate_count == 1U)
    {
        s_result.status = LIGHT_SEQUENCE_STATUS_IDENTIFIED;
        s_result.sequence_id = sequence_id;
    }
    else
    {
        s_result.status = LIGHT_SEQUENCE_STATUS_IDENTIFYING;
        s_result.sequence_id = LIGHT_SEQUENCE_SEQUENCE_ID_UNKNOWN;
    }
}

/**
 * @brief 复位灯号与序列识别状态，重新加载全部10套候选。
 * @param 无。
 * @return 无。
 */
void LightSequence_Reset(void)
{
    uint8 valid_count = 0U;
    uint8 sequence_index;

    memset(s_candidates, 0, sizeof(s_candidates));
    memset(&s_result, 0, sizeof(s_result));
    s_last_beacon_lost_flag = 0U;

    for(sequence_index = 0U; sequence_index < LIGHT_SEQUENCE_PRESET_COUNT; sequence_index++)
    {
        if(LightSequence_PresetValid(&g_light_sequence_presets[sequence_index]) != 0U)
        {
            s_candidates[sequence_index].valid = 1U;
            s_candidates[sequence_index].round_index = 0U;
            s_candidates[sequence_index].remaining_mask =
                g_light_sequence_presets[sequence_index].round_mask[0];
            valid_count++;
        }
    }

    if(valid_count != LIGHT_SEQUENCE_PRESET_COUNT)
    {
        memset(s_candidates, 0, sizeof(s_candidates));
        s_result.status = LIGHT_SEQUENCE_STATUS_CONFIG_ERROR;
        return;
    }

    LightSequence_RefreshResult();
}

/**
 * @brief 在熄灯标志上升沿匹配灯号，并筛选预设亮灯序列。
 * @param beacon_lost_flag 当前熄灯标志，0表示未熄灯，非0表示检测到熄灯。
 * @param car_position_x 车辆最新全局X坐标，单位m。
 * @param car_position_y 车辆最新全局Y坐标，单位m。
 * @return 无。
 */
void LightSequence_Update(uint8 beacon_lost_flag,
                          float car_position_x,
                          float car_position_y)
{
    uint8 beacon_id;
    uint8 beacon_bit;
    uint8 sequence_index;

    beacon_lost_flag = (beacon_lost_flag != 0U) ? 1U : 0U;
    if((beacon_lost_flag == 0U) || (s_last_beacon_lost_flag != 0U))
    {
        s_last_beacon_lost_flag = beacon_lost_flag;
        return;
    }
    s_last_beacon_lost_flag = beacon_lost_flag;

    s_result.last_beacon_id = LIGHT_SEQUENCE_BEACON_ID_NONE;
    beacon_id = LightSequence_FindNearestBeacon(car_position_x, car_position_y);
    if(beacon_id == LIGHT_SEQUENCE_BEACON_ID_NONE)
    {
        return;
    }
    s_result.last_beacon_id = beacon_id;

    if(s_result.status == LIGHT_SEQUENCE_STATUS_CONFIG_ERROR)
    {
        return;
    }

    beacon_bit = (uint8)(1U << (beacon_id - 1U));
    for(sequence_index = 0U; sequence_index < LIGHT_SEQUENCE_PRESET_COUNT; sequence_index++)
    {
        light_sequence_candidate_t *candidate = &s_candidates[sequence_index];
        const light_sequence_preset_t *preset = &g_light_sequence_presets[sequence_index];

        if(candidate->valid == 0U)
        {
            continue;
        }

        if((candidate->round_index >= preset->round_count) ||
           ((candidate->remaining_mask & beacon_bit) == 0U))
        {
            candidate->valid = 0U;
            continue;
        }

        candidate->remaining_mask =
            (uint8)(candidate->remaining_mask & (uint8)(~beacon_bit));
        if(candidate->remaining_mask == 0U)
        {
            candidate->round_index++;
            if(candidate->round_index < preset->round_count)
            {
                candidate->remaining_mask = preset->round_mask[candidate->round_index];
            }
        }
    }

    LightSequence_RefreshResult();
}

/**
 * @brief 获取当前灯号与序列识别结果。
 * @param result 输出结果指针，传入空指针时不执行复制。
 * @return 无。
 */
void LightSequence_GetResult(light_sequence_result_t *result)
{
    if(result == 0)
    {
        return;
    }

    *result = s_result;
}
