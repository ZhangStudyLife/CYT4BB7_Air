#include "fc_mode.h"

/* Mode4 manual attitude mapping scale, deg per RC normalized unit. */
static const float s_mode4_rc_to_angle_scale = 0.04f;
/* Mode4 manual attitude limit, deg. */
static const float s_mode4_manual_angle_limit_deg = 20.0f;

void FC_Mode4_Init(void)
{
    FC_Mode4_Reset();
}

void FC_Mode4_Reset(void)
{
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

void FC_Mode4_100Hz(void)
{
    float ch0;
    float ch1;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        return;
    }

    ch0 = FC_Mode_Clamp((float)CRSF_STD[0], -1000.0f, 1000.0f);
    ch1 = FC_Mode_Clamp((float)CRSF_STD[1], -1000.0f, 1000.0f);

    roll_angle_target = FC_Mode_Clamp(ch0 * s_mode4_rc_to_angle_scale + FC_Mode_Get_Roll_Mech_Trim_Deg(),
                                      -s_mode4_manual_angle_limit_deg, s_mode4_manual_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(-ch1 * s_mode4_rc_to_angle_scale + FC_Mode_Get_Pitch_Mech_Trim_Deg(),
                                       -s_mode4_manual_angle_limit_deg, s_mode4_manual_angle_limit_deg);
}

void FC_Mode4_50Hz(float dt)
{
    (void)dt;
}
