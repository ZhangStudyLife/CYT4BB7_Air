#ifndef YAW_ALIGN_H
#define YAW_ALIGN_H

#include "zf_common_headfile.h"

typedef struct
{
    uint8 valid;
    uint8 camera;
    float x;
    float y;
    float area;
} yaw_align_debug_beacon_t;

typedef struct
{
    uint8 locked;
    uint8 candidate_frames;
    uint8 lost_frames;
    uint8 action;
    float yaw_delta_deg;
    yaw_align_debug_beacon_t active_beacon;
    yaw_align_debug_beacon_t locked_beacon;
    yaw_align_debug_beacon_t candidate_beacon;
} yaw_align_debug_t;

typedef enum
{
    YAW_ALIGN_ACTION_IDLE = 0U,
    YAW_ALIGN_ACTION_CANDIDATE,
    YAW_ALIGN_ACTION_CENTER_TURN,
    YAW_ALIGN_ACTION_LOST_HOLD,
    YAW_ALIGN_ACTION_DEADBAND_HOLD,
    YAW_ALIGN_ACTION_TRACK,
    YAW_ALIGN_ACTION_SEARCH
} yaw_align_action_e;

void YawAlign_Reset(void);
/*
 * 函数功能: 根据航向控制模式更新 yaw 目标
 * 输入参数:
 *   yaw_change_mode - 航向控制模式，0=固定0度，1=信标对准，2=步进搜索
 * 输出参数或返回值: 无
 */
void YawAlign_Update(float yaw_change_mode);
void YawAlign_GetDebug(yaw_align_debug_t *out);

#endif /* YAW_ALIGN_H */
