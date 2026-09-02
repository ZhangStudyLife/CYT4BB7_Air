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
    uint8 beacon_visible; /* plan_result.valid 是否已连续有效100ms。 */
    uint8 plan_valid; /* 当前 plan_result.valid 原始状态。 */
    uint8 search_active; /* 当前是否正在执行无信标旋转搜索。 */
    int8 search_direction; /* 搜索方向，1为yaw正方向，-1为yaw负方向。 */
    float yaw_delta_deg;
    float search_rotation_deg; /* 本轮实际定向搜索角，单位度。 */
    float cable_twist_deg; /* 飞机相对车辆的累计线缆扭转角，单位度。 */
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

/**
 * @brief 清空航向对准、搜索和线缆扭转跟踪状态。
 * @param 无。
 * @return 无。
 */
void YawAlign_Reset(void);
/**
 * @brief 根据航向控制模式更新yaw目标和搜索状态。
 * @param yaw_change_mode 航向控制模式，0=固定0度，1=信标对准，2=步进搜索。
 * @return 无。
 */
void YawAlign_Update(float yaw_change_mode);

/**
 * @brief 查询mode=2是否正在执行旋转搜索。
 * @param 无。
 * @return 1表示正在搜索，0表示未搜索。
 */
uint8 YawAlign_IsSearchActive(void);

/**
 * @brief 设置强制搜索，供自动降落双旋转阶段使用。
 * @param forced 1为强制持续旋转搜索，0为恢复正常搜索。
 * @return 无。
 * @note 强制期间忽略plan确认对搜索的停止，并跳过首个目标速度等待。
 */
void YawAlign_SetSearchForced(uint8 forced);

/**
 * @brief 获取航向对准和搜索状态的只读调试快照。
 * @param out 调试快照输出地址，不可为空。
 * @return 无。
 */
void YawAlign_GetDebug(yaw_align_debug_t *out);

#endif /* YAW_ALIGN_H */
