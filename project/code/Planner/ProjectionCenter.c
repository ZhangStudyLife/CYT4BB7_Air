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
#include "ProjectionCenter.h"
#include "../Estimation/Attitude/IMU_TOP.h"

#define PROJECTION_CENTER_X_BIAS                 (-0.20f) /* x轴零姿态偏置，单位px。 */
#define PROJECTION_CENTER_X_ROLL_K               (1.325f) /* x轴对roll的灵敏度，单位px/deg。 */
#define PROJECTION_CENTER_Y_BIAS                 (-4.63f) /* y轴零姿态偏置，单位px。 */
#define PROJECTION_CENTER_Y_PITCH_K              (1.334f) /* y轴对pitch的灵敏度，单位px/deg。 */

static float s_projection_roll_history[4]; /* 最近约40ms的100Hz周期roll，单位deg。 */
static float s_projection_pitch_history[4]; /* 最近约40ms的100Hz周期pitch，单位deg。 */

projection_center_result_t g_projection_center;

void ProjectionCenter_Init(void)
{
    g_projection_center.valid = 0U;
    g_projection_center.cx = 0.0f;
    g_projection_center.cy = 0.0f;
    s_projection_roll_history[0] = g_euler.roll;
    s_projection_roll_history[1] = g_euler.roll;
    s_projection_roll_history[2] = g_euler.roll;
    s_projection_roll_history[3] = g_euler.roll;
    s_projection_pitch_history[0] = g_euler.pitch;
    s_projection_pitch_history[1] = g_euler.pitch;
    s_projection_pitch_history[2] = g_euler.pitch;
    s_projection_pitch_history[3] = g_euler.pitch;
}

uint8 ProjectionCenter_Update100Hz(void)
{
    float delayed_roll = s_projection_roll_history[3];
    float delayed_pitch = s_projection_pitch_history[3];

    s_projection_roll_history[3] = s_projection_roll_history[2];
    s_projection_roll_history[2] = s_projection_roll_history[1];
    s_projection_roll_history[1] = s_projection_roll_history[0];
    s_projection_pitch_history[3] = s_projection_pitch_history[2];
    s_projection_pitch_history[2] = s_projection_pitch_history[1];
    s_projection_pitch_history[1] = s_projection_pitch_history[0];
    s_projection_roll_history[0] = g_euler.roll;
    s_projection_pitch_history[0] = g_euler.pitch;

    g_projection_center.cx = PROJECTION_CENTER_X_BIAS +
                             PROJECTION_CENTER_X_ROLL_K * delayed_roll;
    g_projection_center.cy = PROJECTION_CENTER_Y_BIAS +
                             PROJECTION_CENTER_Y_PITCH_K * delayed_pitch;
    g_projection_center.valid = 1U;

    return 1U;
}
