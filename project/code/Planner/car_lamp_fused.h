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
#ifndef CAR_LAMP_FUSED_H
#define CAR_LAMP_FUSED_H

#include "zf_common_typedef.h"

typedef struct
{
    uint8 valid;
    float cx;
    float cy;
    float angle;
    float width;
    float length;
} car_lamp_fused_result_t;

extern car_lamp_fused_result_t g_car_lamp_fused;
extern uint8 only_front_see_car_lamp;
extern uint8 only_back_see_car_lamp;

void CarLampFused_Init(void);
uint8 CarLampFused_Update100Hz(void);

#endif /* CAR_LAMP_FUSED_H */
