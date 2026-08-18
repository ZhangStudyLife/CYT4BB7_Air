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
