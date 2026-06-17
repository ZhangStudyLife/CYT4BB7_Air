#include "fc_mode.h"

void FC_Mode2_Init(void)
{
    FC_Mode2_Reset();
}

void FC_Mode2_Reset(void)
{
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
}

void FC_Mode2_100Hz(void)
{
}

void FC_Mode2_50Hz(float dt)
{
    (void)dt;
}
