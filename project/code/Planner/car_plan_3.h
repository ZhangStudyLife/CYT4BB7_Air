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
#ifndef CAR_PLAN_3_H
#define CAR_PLAN_3_H

#include "car_plan_entry.h"

#define CAR_PLAN_3_DEBUG_BEACON_COUNT (4U) /* 调试输出的全局融合信标最大数量。 */

typedef struct
{
    uint8 valid;
    uint8 camera_mask;
    float center_x; /* 世界对齐局部 X 坐标，单位 m。 */
    float center_y; /* 世界对齐局部 Y 坐标，单位 m。 */
    float area;
} car_plan_3_debug_beacon_t;

typedef struct
{
    uint8 valid;
    uint8 camera_mask;
    float center_x; /* 世界对齐局部 X 坐标，单位 m。 */
    float center_y; /* 世界对齐局部 Y 坐标，单位 m。 */
    float angle_deg; /* 车灯长轴在水平全局坐标系中的无向角度，单位 deg。 */
} car_plan_3_debug_lamp_t;

typedef struct
{
    car_plan_3_debug_beacon_t beacon[CAR_PLAN_3_DEBUG_BEACON_COUNT];
    car_plan_3_debug_lamp_t car_lamp;
    int8 selected_target_id;
} car_plan_3_debug_t;

void CarPlan_3_Reset(void);

/**
 * @brief 使用三摄 Double Sphere 几何确定车灯到最近信标的目标速度，同摄组合优先于跨摄组合。
 * @param result 输出车体系横向和前向目标速度；允许传入空指针。
 * @return 成功得到可信目标方向时返回 1，否则清空输出并返回 0。
 * @note g_car_yaw 只用于消除所选车灯长轴的 180 度方向歧义，不替代图像目标方向。
 */
uint8 CarPlan_3_Update(car_plan_result_t *result);
void CarPlan_3_GetResult(car_plan_result_t *result);
void CarPlan_3_GetDebug(car_plan_3_debug_t *debug);

#endif /* CAR_PLAN_3_H */
