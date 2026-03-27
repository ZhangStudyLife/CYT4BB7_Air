#include "fc_mode.h"

void FC_Mode1_Init(void)     { FC_Mode0_Init(); }
void FC_Mode1_Reset(void)    { FC_Mode0_Reset(); }
void FC_Mode1_100Hz(void)    { FC_Mode0_100Hz(); }
void FC_Mode1_50Hz(float dt) { FC_Mode0_50Hz(dt); }
