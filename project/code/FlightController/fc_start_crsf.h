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
    FC_START_CRSF_FLIGHT_MODE_2
} FC_START_CRSF_flight_mode_e;

#define FC_START_CRSF_UNLOCK_HOLD_TIME_MS (1000U)
#define FC_START_CRSF_RC_RAW_LOW_THR      (-200)
#define FC_START_CRSF_RC_RAW_HIGH_THR     (200)

void FC_START_CRSF_Init(void);
void FC_START_CRSF_Update(void);

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
