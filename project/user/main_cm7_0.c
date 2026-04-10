#include "zf_common_headfile.h"

volatile uint32 tick_1000us_cnt = 0U;
volatile uint16 g_tick_1000HZ = 0U;
volatile uint8 g_tick_100HZ = 0U;

static uint8 div500 = 0U;
static uint8 div50 = 0U;
static uint8 slot50 = 0U;
static uint8 div10 = 0U;

int main(void)
{
    wifi_params_diag_t wifi_params_diag = {0};

    clock_init(SYSTEM_CLOCK_250M);
    Beep_Init();
    pit_ms_init(PIT_CH2, 10);
    wifi_cmd_Init();
    TOF_Init();
    PMW3901_Init();
    LC302_Init();
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
        uint16 guard = 0U;

        while ((g_tick_1000HZ > 0U) && (guard < 100U))
        {
            g_tick_1000HZ--;

            IMU_Update_1000HZ();
            Pos_Est_Update_1000HZ();
            // wifi_justfloat(g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy, g_imufilter_1000hz.gyroz,
            //                g_euler.roll, g_euler.pitch, g_euler.yaw);

            div500++;
            if (div500 >= 2U)
            {
                div500 = 0U;
                FC_Loop_500Hz();
                wifi_justfloat(tick_1000us_cnt, lc302_data.flow_x_integral, lc302_data.flow_y_integral, lc302_data.integration_timespan, lc302_data.ground_distance, lc302_data.valid, lc302_data.version);
            }

            FC_Loop_1000Hz();
            guard++;
        }

        if (g_tick_100HZ > 0U)
        {
            g_tick_100HZ--;

            CRSF_Update_100HZ();
            FC_Loop_100Hz();
            wifi_justfloat_SetStandbyContext((FC_START_CRSF_STATE_STANDBY == FC_START_CRSF_Get_State()) && (0U == FC_START_CRSF_Is_Armed()));

            slot50 = div50;
            if (slot50 == 0U)
            {
                LC302_Update_50HZ();
                Pos_Est_Update_50HZ();
                crsf_send_50hz();
            }
            else
            {
                FC_Loop_50Hz();

                wifi_params_GetDiag(&wifi_params_diag);
            }

            div50++;
            if (div50 >= 2U)
            {
                div50 = 0U;
            }

            div10++;
            if (div10 >= 10U)
            {
                div10 = 0U;
                FC_START_CRSF_Update();

                if (g_euler.roll > 35.0f || g_euler.roll < -35.0f ||
                    g_euler.pitch > 35.0f || g_euler.pitch < -35.0f)
                {
                    FC_START_CRSF_Trigger_Emergency_Stop();
                }
            }
        }
        //         #if (0U == WIFI_IMAGE_ENABLE)
        //     wifi_cmd_Poll();
        // #endif
    }
}
