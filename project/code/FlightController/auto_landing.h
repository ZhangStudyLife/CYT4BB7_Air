#ifndef AUTO_LANDING_H
#define AUTO_LANDING_H

#include "zf_common_headfile.h"

typedef struct
{
    uint16 initial_wait_ticks; /* 自动降落前置等待计数，单位10ms。 */
    uint16 no_beacon_ticks; /* 连续无信标计数，单位10ms。 */
    uint8 beacon_visible; /* 当前是否发现合格信标。 */
    uint8 rotation_ready; /* 本轮实际定向搜索是否达到360度。 */
    uint8 triggered; /* 自动降落触发锁存标志。 */
} auto_landing_debug_t;

/**
 * @brief 以100Hz更新Mode4自动降落检测。
 * @param 无。
 * @return 无。
 */
void AutoLanding_Update100Hz(void);

/**
 * @brief 查询自动降落触发锁存状态。
 * @param 无。
 * @return 1表示自动降落已触发，0表示未触发。
 */
uint8 AutoLanding_IsTriggered(void);

/**
 * @brief 获取自动降落检测的只读调试快照。
 * @param debug 调试快照输出地址，不可为空。
 * @return 无。
 */
void AutoLanding_GetDebug(auto_landing_debug_t *debug);

#endif /* AUTO_LANDING_H */
