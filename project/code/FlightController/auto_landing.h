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
#ifndef AUTO_LANDING_H
#define AUTO_LANDING_H

#include "zf_common_headfile.h"

typedef struct
{
    uint16 initial_wait_ticks; /* 自动降落前置等待计数，单位10ms。 */
    uint16 no_target_ticks; /* 无有效规划目标累计计数，单位10ms。 */
    uint16 valid_target_ticks; /* 速度下发连续有效计数，单位10ms。 */
    uint8 target_valid; /* plan_result.valid 是否已连续有效200ms。 */
    uint8 rotation_ready; /* 实际定向搜索是否达到360度，仅用于观测。 */
    uint8 triggered; /* 自动降落触发锁存标志。 */
    uint8 state; /* 自动降落状态机状态，0=IDLE 1=DETECT 2=ROTATE 3=TRIGGERED。 */
    uint8 car_started; /* 车模启动判定：最近200ms内车端时间戳推进。 */
    uint8 rotate_active; /* 双旋转阶段是否进行中。 */
    float rotate_air_deg; /* 双旋转阶段飞机累计旋转角，单位度。 */
    float rotate_car_deg; /* 双旋转阶段车模累计旋转角，单位度。 */
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
 * @brief 查询双旋转阶段是否进行中。
 * @param 无。
 * @return 1表示飞机与车模正在各旋转一圈，0表示未在旋转。
 */
uint8 AutoLanding_IsRotationActive(void);

/**
 * @brief 获取双旋转阶段下发车模的旋转指令速度。
 * @param strafe_mps 车体右向速度输出地址，不可为空。
 * @param forward_mps 车体前向速度输出地址，不可为空。
 * @return 无。
 */
void AutoLanding_GetRotateCommand(float *strafe_mps, float *forward_mps);

/**
 * @brief 获取自动降落检测的只读调试快照。
 * @param debug 调试快照输出地址，不可为空。
 * @return 无。
 */
void AutoLanding_GetDebug(auto_landing_debug_t *debug);

#endif /* AUTO_LANDING_H */
