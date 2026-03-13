#include "Pos_Est.h"

extern volatile uint32 tick_1000us_cnt;

float Pos_Est_vel_x = 0.0f;
float Pos_Est_vel_y = 0.0f;

void Pos_Est_Init(void)
{
    PMW3901_Init();
    FlowGyroDecoupler_Init();
}

void Pos_Est_Reinit(void)
{
    PMW3901_ReInit();
    FlowGyroDecoupler_Reinit();
}

void Pos_Est_Update_1000HZ(void)
{
    FlowGyroDecoupler_Push1000Hz(tick_1000us_cnt, g_imudata_250hz.gyrox, g_imudata_250hz.gyroy);
}

void Pos_Est_Update_50HZ(void)
{
    PMW3901_Update_50HZ();
    FlowGyroDecoupler_Update50Hz(tick_1000us_cnt, g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY);
    float dec_x = FlowGyroDecoupler_GetDecX();
    float dec_y = FlowGyroDecoupler_GetDecY();
    float height = g_tof_fused_height_mm / 1000.0f;         // 高度单位 M

    float coeff = 0.2131946f * (height - 0.05);     // 0.2131946f是 1m 高度下 1 像素对应位移，单位 cm 
    if (height < 0.2f)
    {
        coeff = 0.0f;
    }
    Pos_Est_vel_x = dec_x * coeff * 50; // CM/S        50HZ调用
    Pos_Est_vel_y = dec_y * coeff * 50; // CM/S
}
