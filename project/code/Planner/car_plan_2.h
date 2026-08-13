#ifndef CAR_PLAN_2_H
#define CAR_PLAN_2_H

#include "zf_common_headfile.h"

typedef struct
{
    uint8 valid;
    uint8 camera_mask;
    float target_strafe_mps;
    float target_forward_mps;
    float target_center_x;
    float target_center_y;
} car_plan_2_result_t;

/* 原始候选来源位：Front[0..1]=0..1、Center[0..1]=2..3、Back[0..1]=4..5。 */
typedef enum
{
    CAR_PLAN_2_DEBUG_REASON_IDLE = 0U,
    CAR_PLAN_2_DEBUG_REASON_ACQUIRE = 1U,
    CAR_PLAN_2_DEBUG_REASON_TRACK = 2U,
    CAR_PLAN_2_DEBUG_REASON_VELOCITY_PENDING = 3U,
    CAR_PLAN_2_DEBUG_REASON_NEARER_PENDING = 4U,
    CAR_PLAN_2_DEBUG_REASON_SWITCH_VELOCITY = 5U,
    CAR_PLAN_2_DEBUG_REASON_SWITCH_NEARER = 6U,
    CAR_PLAN_2_DEBUG_REASON_LOST_HOLD = 7U,
    CAR_PLAN_2_DEBUG_REASON_REACQUIRE = 8U,
    CAR_PLAN_2_DEBUG_REASON_NO_TARGET = 9U,
    CAR_PLAN_2_DEBUG_REASON_INVALID_LAMP = 10U,
    CAR_PLAN_2_DEBUG_REASON_TARGET_TOO_CLOSE = 11U,
    CAR_PLAN_2_DEBUG_REASON_LOCK_NO_MATCH = 12U
} car_plan_2_debug_reason_e;

typedef struct
{
    uint8 lock_valid;
    uint8 lock_camera_mask;
    uint8 lock_source_mask;
    uint8 previous_lock_source_mask;
    uint8 matched_source_mask;
    uint8 velocity_challenger_source_mask;
    uint8 nearer_source_mask;
    uint8 reason;
    uint8 lock_changed;
    uint8 lock_change_reason;
    uint8 lost_ticks;
    uint8 velocity_conflict_ticks;
    uint8 nearer_target_ticks;
    uint8 cluster_count;
    uint8 failure_reason;
    uint32 lock_change_count;
    float lock_center_x;
    float lock_center_y;
    float predicted_x;
    float predicted_y;
    float locked_velocity_cos;
    float target_strafe_mps;
    float target_forward_mps;
} car_plan_2_debug_t;

/**
 * @brief 重置影子车模规划器的目标锁定状态和输出结果。
 * @param 无。
 * @return 无。
 */
void CarPlan_2_Reset(void);

/**
 * @brief 扫描三摄信标候选并更新影子车模速度规划结果。
 * @param result 输出规划结果；允许传入空指针。
 * @return 规划结果有效时返回1，否则返回0。
 */
uint8 CarPlan_2_Update(car_plan_2_result_t *result);

/**
 * @brief 获取最近一次影子车模规划结果的快照。
 * @param result 输出规划结果；允许传入空指针。
 * @return 无。
 */
void CarPlan_2_GetResult(car_plan_2_result_t *result);

/** 读取car_plan_2最新的内部诊断快照。 */
void CarPlan_2_GetDebug(car_plan_2_debug_t *debug);

#endif /* CAR_PLAN_2_H */
