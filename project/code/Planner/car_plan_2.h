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
#ifndef CAR_PLAN_2_H
#define CAR_PLAN_2_H

#include "car_plan_entry.h"

#define CAR_PLAN_2_DEBUG_BEACON_COUNT (3U) /* 调试上报的融合信标槽位数量。 */

typedef struct
{
    uint8 valid;
    uint8 camera_mask;
    float center_x;
    float center_y;
    float area;
} car_plan_2_debug_beacon_t;

typedef struct
{
    car_plan_2_debug_beacon_t beacon[CAR_PLAN_2_DEBUG_BEACON_COUNT];
    int8 selected_target_id;
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
uint8 CarPlan_2_Update(car_plan_result_t *result);

/**
 * @brief 获取最近一次影子车模规划结果的快照。
 * @param result 输出规划结果；允许传入空指针。
 * @return 无。
 */
void CarPlan_2_GetResult(car_plan_result_t *result);

/**
 * @brief 获取最近一次三摄融合信标和当前选定目标的调试快照。
 * @param debug 输出调试快照；允许传入空指针。
 * @return 无。
 */
void CarPlan_2_GetDebug(car_plan_2_debug_t *debug);

#endif /* CAR_PLAN_2_H */
