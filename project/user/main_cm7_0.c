#include "zf_common_headfile.h"

volatile uint32 tick_1000us_cnt = 0U;
volatile uint16 g_tick_1000HZ = 0U;
volatile uint8 g_tick_100HZ = 0U;

static uint8 s_tick_div_pos_250hz = 0U;
static uint8 s_tick_div_fc_loop_500hz = 0U;
static uint8 s_tick_div_fc_start_10hz = 0U;
static uint8 s_tick_div_fc_start_50hz = 0U;
static uint8 s_tick_div_fc_start_1hz = 0U;

int main(void)
{
    wifi_params_diag_t wifi_params_diag = {0};

    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    Beep_Init();
    pit_ms_init(PIT_CH2, 10);
    wifi_cmd_Init();
    image_init();
    TOF_Init();
    PMW3901_Init();
    IMU_Init_All();
    crsf_init();
    AccelCalibration_Init();
    IMUCalib_Init();
    FC_Params_Init();
    (void)FC_Params_LoadFromFlash();
    Pos_Est_Init();
    FC_Loop_Init();
    wifi_justfloat_Init();
    wifi_params_Init();
    wifi_cal_imu_Init();
    Motor_Init();
    FC_START_CRSF_Init();
    wifi_justfloat_SetStandbyContext((FC_START_CRSF_STATE_STANDBY == FC_START_CRSF_Get_State()) && (0U == FC_START_CRSF_Is_Armed()));
    pit_us_init(PIT_CH0, 1000);
    pit_ms_init(PIT_CH1, 10);

    while (true)
    {
        uint16 tick_1000_guard = 0U;

        while ((g_tick_1000HZ > 0U) && (tick_1000_guard < 100U))
        {
            g_tick_1000HZ--;

            IMU_Update_1000HZ();
            Pos_Est_Update_1000HZ();
            // wifi_justfloat(g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy, g_imufilter_1000hz.gyroz,
            //                g_euler.roll, g_euler.pitch, g_euler.yaw);
            
            s_tick_div_fc_loop_500hz++;
            if (s_tick_div_fc_loop_500hz >= 2U)
            {
                s_tick_div_fc_loop_500hz = 0U;
                FC_Loop_500Hz();
            }

            FC_Loop_1000Hz();

            s_tick_div_pos_250hz++;
            if (s_tick_div_pos_250hz >= 4U)
            {
                s_tick_div_pos_250hz = 0U;
            }

            tick_1000_guard++;
        }

        if (g_tick_100HZ > 0U)
        {
            g_tick_100HZ--;

            crsf_send_25hz();
            CRSF_Update_100HZ();
            FC_Loop_100Hz();
            wifi_justfloat_SetStandbyContext((FC_START_CRSF_STATE_STANDBY == FC_START_CRSF_Get_State()) && (0U == FC_START_CRSF_Is_Armed()));

            s_tick_div_fc_start_50hz++;
            if (s_tick_div_fc_start_50hz >= 2U)
            {
                Pos_Est_Update_50HZ();
                s_tick_div_fc_start_50hz = 0U;
                image_update();
                FC_Loop_50Hz();

                wifi_params_GetDiag(&wifi_params_diag);
                #if (0U == WIFI_IMAGE_ENABLE)
                    wifi_cmd_Poll();
                #endif
            }

            s_tick_div_fc_start_1hz++;
            if (s_tick_div_fc_start_1hz >= 2U)
            {
                s_tick_div_fc_start_1hz = 0U;
            }
            

            s_tick_div_fc_start_10hz++;
            if (s_tick_div_fc_start_10hz >= 10U)
            {
                s_tick_div_fc_start_10hz = 0U;


                FC_START_CRSF_Update();

                if (g_euler.roll > 35.0f || g_euler.roll < -35.0f ||
                    g_euler.pitch > 35.0f || g_euler.pitch < -35.0f)
                {
                    FC_START_CRSF_Trigger_Emergency_Stop();
                }
            }
        }


    }
}
