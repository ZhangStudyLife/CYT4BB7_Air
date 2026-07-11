#ifndef LIGHT_SEQUENCE_H
#define LIGHT_SEQUENCE_H

#include "zf_common_headfile.h"

#define LIGHT_SEQUENCE_BEACON_ID_NONE       (0U)    /* 本次熄灯事件未匹配到信标 */
#define LIGHT_SEQUENCE_SEQUENCE_ID_UNKNOWN  (0U)    /* 当前序列尚未唯一确定 */

/**
 * @brief 信标灯序列识别状态。
 */
typedef enum
{
    LIGHT_SEQUENCE_STATUS_IDENTIFYING = 0U,
    LIGHT_SEQUENCE_STATUS_IDENTIFIED,
    LIGHT_SEQUENCE_STATUS_FAILED,
    LIGHT_SEQUENCE_STATUS_CONFIG_ERROR
} light_sequence_status_e;

/**
 * @brief 信标灯序列识别结果快照。
 */
typedef struct
{
    uint8 status;              /* light_sequence_status_e识别状态 */
    uint8 last_beacon_id;      /* 最近一次熄灯上升沿匹配的灯号，范围1至7，0表示未匹配 */
    uint8 sequence_id;         /* 唯一确定的序列编号，范围1至10，0表示未确定 */
    uint8 candidate_count;     /* 当前剩余候选序列数量 */
    uint16 candidate_mask;     /* 当前候选位掩码，bit0至bit9分别对应序列1至10 */
} light_sequence_result_t;

/**
 * @brief 复位灯号与序列识别状态，重新加载全部10套候选。
 * @param 无。
 * @return 无。
 */
void LightSequence_Reset(void);

/**
 * @brief 在熄灯标志上升沿匹配灯号，并筛选预设亮灯序列。
 * @param beacon_lost_flag 当前熄灯标志，0表示未熄灯，非0表示检测到熄灯。
 * @param car_position_x 车辆最新全局X坐标，单位m。
 * @param car_position_y 车辆最新全局Y坐标，单位m。
 * @return 无。
 */
void LightSequence_Update(uint8 beacon_lost_flag,
                          float car_position_x,
                          float car_position_y);

/**
 * @brief 获取当前灯号与序列识别结果。
 * @param result 输出结果指针，传入空指针时不执行复制。
 * @return 无。
 */
void LightSequence_GetResult(light_sequence_result_t *result);

#endif /* LIGHT_SEQUENCE_H */
