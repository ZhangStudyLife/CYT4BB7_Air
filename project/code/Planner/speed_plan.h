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
#ifndef SPEED_PLAN_H
#define SPEED_PLAN_H

#include "zf_common_typedef.h"

void SpeedPlan_Reset(void);

/**
 * @brief 根据目标距离、方向和当前跟随状态选择车模速度模长。
 * @param target_valid 目标方向有效时为1，否则为0。
 * @param target_distance_m 车灯到目标信标的水平距离，单位m。
 * @param target_angle_deg 目标方向与车体正前方的夹角，范围0至180deg。
 * @return Car_Speed或Car_Speed_Fast。
 */
float SpeedPlan_Update(uint8 target_valid,
                       float target_distance_m,
                       float target_angle_deg);

#endif /* SPEED_PLAN_H */
