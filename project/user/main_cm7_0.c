#include "zf_common_headfile.h"
#include "../code/HW_Drivers/ICM42688_Aux/ICM42688_Aux.h"
#include "../code/HW_Drivers/BMI088/BMI088.h"

volatile uint32 tick_1000us_cnt = 0U;
volatile uint16 g_tick_1000HZ = 0U;
volatile uint8 g_tick_100HZ = 0U;

static uint8 div500 = 0U;
static uint8 div50 = 0U;
static uint8 slot50 = 0U;
static uint8 div10 = 0U;

void ips114_show_debug()
{
    const VL53L1X_data_struct *tof = VL53L1X_GetData();

    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_set_font(IPS114_6X8_FONT);

    ips114_show_string(0, 0, "A");
    ips114_show_float(12, 0, (double)ICM42688.acc_x, 3, 2);
    ips114_show_float(60, 0, (double)ICM42688.acc_y, 3, 2);
    ips114_show_float(108, 0, (double)ICM42688.acc_z, 3, 2);

    ips114_show_string(0, 8, "G");
    ips114_show_float(12, 8, (double)ICM42688.gyro_x, 4, 2);
    ips114_show_float(60, 8, (double)ICM42688.gyro_y, 4, 2);
    ips114_show_float(108, 8, (double)ICM42688.gyro_z, 4, 2);

    ips114_show_string(0, 16, "R");
    ips114_show_float(12, 16, (double)g_euler.roll, 3, 2);
    ips114_show_string(54, 16, "P");
    ips114_show_float(66, 16, (double)g_euler.pitch, 3, 2);
    ips114_show_string(108, 16, "Y");
    ips114_show_float(120, 16, (double)g_euler.yaw, 4, 2);

    ips114_show_string(0, 24, "1");
    ips114_show_uint(12, 24, tof->distance_mm[0], 4);
    ips114_show_string(42, 24, "2");
    ips114_show_uint(54, 24, tof->distance_mm[1], 4);
    ips114_show_string(84, 24, "3");
    ips114_show_uint(96, 24, tof->distance_mm[2], 4);
    ips114_show_string(126, 24, "4");
    ips114_show_uint(138, 24, tof->distance_mm[3], 4);

    ips114_show_string(0, 32, "X");
    ips114_show_int(12, 32, lc302_data.flow_x_integral, 5);
    ips114_show_string(48, 32, "Y");
    ips114_show_int(60, 32, lc302_data.flow_y_integral, 5);
}

int main(void)
{
    wifi_params_diag_t wifi_params_diag = {0};

    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();
    ips114_init();
    Beep_Init();
    pit_ms_init(PIT_CH2, 10);
    /* 副 IMU 初始化日志走 WiFi 文本链路，所以 wifi_cmd_Init 必须早于 ICM42688_Aux_Init。 */
    // wifi_cmd_Init();
    TOF_Init();
    // PMW3901_Init();
    // LC302_Init();
    IMU_Init_All();
    // (void)ICM42688_Aux_Init();           //对比用的陀螺仪关掉
    // (void)BMI088_Init();
    crsf_init();
    AccelCalibration_Init();
    IMUCalib_Init();
    FC_Params_Init();
    (void)FC_Params_LoadFromFlash();
    Pos_Est_Init();
    FC_Loop_Init();
    // wifi_justfloat_Init();
    // wifi_params_Init();
    // wifi_cal_imu_Init();
    Motor_Init();
    ipc_communicate_init(IPC_PORT_1, ipc_image_callback);
    FC_START_CRSF_Init();
    // wifi_justfloat_SetStandbyContext((FC_START_CRSF_STATE_STANDBY == FC_START_CRSF_Get_State()) && (0U == FC_START_CRSF_Is_Armed()));
    pit_us_init(PIT_CH0, 1000);
    pit_ms_init(PIT_CH1, 10);

    Motor_Enable();
    Motor_SetThrottleAll((int32[]){2000, 0, 0, 0});
    system_delay_ms(500);
    Motor_SetThrottleAll((int32[]){0, 2000, 0, 0});
    system_delay_ms(500);
    Motor_SetThrottleAll((int32[]){0, 0, 2000, 0});
    system_delay_ms(500);
    Motor_SetThrottleAll((int32[]){0, 0, 0, 2000});
    system_delay_ms(500);
    Motor_SetThrottleAll((int32[]){0, 0, 0, 0});

    while (true)
    {
        uint16 guard = 0U;

        while ((g_tick_1000HZ > 0U) && (guard < 100U))
        {
            g_tick_1000HZ--;

            IMU_Update_1000HZ();
            // ICM42688_Aux_Update_1000Hz(tick_1000us_cnt);         //对比用的陀螺仪关掉
            // BMI088_Update_1000Hz(tick_1000us_cnt);
            Pos_Est_Update_1000HZ();

            div500++;
            if (div500 >= 2U)
            {
                div500 = 0U;
                FC_Loop_500Hz();
                // wifi_justfloat(tick_1000us_cnt, lc302_data.flow_x_integral, lc302_data.flow_y_integral, lc302_data.integration_timespan, g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY,g_pmw3901_raw.squal);
            }

            FC_Loop_1000Hz();
            guard++;
        }

        if (g_tick_100HZ > 0U)
        {
            g_tick_100HZ--;
            Height_Est_update_100HZ();
            CRSF_Update_100HZ();
            FC_Loop_100Hz();
            // wifi_justfloat_SetStandbyContext((FC_START_CRSF_STATE_STANDBY == FC_START_CRSF_Get_State()) && (0U == FC_START_CRSF_Is_Armed()));

            slot50 = div50;
            if (slot50 == 0U)
            {
                if (ipc_image_is_new())
                {
                    ipc_image_payload_t img;
                    ipc_image_get(&img);
                    // wifi_justfloat(img.circles->valid, img.circles->x, img.circles->y);
                }
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
                if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
                {
                    ips114_show_debug();
                }
                
                div10 = 0U;
                FC_START_CRSF_Update();

                if (g_euler.roll > 35.0f || g_euler.roll < -35.0f ||
                    g_euler.pitch > 35.0f || g_euler.pitch < -35.0f)
                {
                    FC_START_CRSF_Trigger_Emergency_Stop();
                }
            }
        }
        // #if (0U == WIFI_IMAGE_ENABLE)
        //         wifi_cmd_Poll();
        // #endif
    }
}
