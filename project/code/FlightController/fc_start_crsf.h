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
#ifndef FC_START_CRSF_H
#define FC_START_CRSF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    FC_START_CRSF_STATE_INIT = 0,
    FC_START_CRSF_STATE_STANDBY,
    FC_START_CRSF_STATE_TAKEOFF,
    FC_START_CRSF_STATE_FLYING,
    FC_START_CRSF_STATE_LANDING
} FC_START_CRSF_state_e;

typedef enum
{
    FC_START_CRSF_FLIGHT_MODE_0 = 0,
    FC_START_CRSF_FLIGHT_MODE_1,
    FC_START_CRSF_FLIGHT_MODE_2,
    FC_START_CRSF_FLIGHT_MODE_3,
    FC_START_CRSF_FLIGHT_MODE_4,
    FC_START_CRSF_FLIGHT_MODE_5,
    FC_START_CRSF_FLIGHT_MODE_6,
    FC_START_CRSF_FLIGHT_MODE_7,
    FC_START_CRSF_FLIGHT_MODE_8
} FC_START_CRSF_flight_mode_e;

#define FC_START_CRSF_UNLOCK_HOLD_TIME_MS (1000U)
#define FC_START_CRSF_RC_RAW_LOW_THR      (-200)
#define FC_START_CRSF_RC_RAW_HIGH_THR     (200)

extern FC_START_CRSF_state_e s_fc_start_state;

void FC_START_CRSF_Init(void);
void FC_START_CRSF_Update(void);
void FC_START_CRSF_UpdateLandingButton100Hz(void);

FC_START_CRSF_state_e FC_START_CRSF_Get_State(void);
FC_START_CRSF_flight_mode_e FC_START_CRSF_Get_Flight_Mode(void);
uint8_t FC_START_CRSF_Is_Armed(void);

void FC_START_CRSF_Trigger_Emergency_Stop(void);
void FC_START_CRSF_Request_Landing(void);

uint16_t FC_START_CRSF_Get_Unlock_Timer(void);
uint16_t FC_START_CRSF_Get_Arm_Delay_Timer(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_START_CRSF_H */
