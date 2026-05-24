#include "zf_common_headfile.h"

volatile uint32 tick_1000us_cnt = 0U;
volatile uint16 g_tick_1000HZ = 0U;
volatile uint8 g_tick_100HZ = 0U;

static uint8 div500 = 0U;
static uint8 div50 = 0U;
static uint8 slot50 = 0U;
static uint8 div10 = 0U;
static uint8 s_ipc_last_flying = 0U;      /* 上一次成功通知给核1的飞行状态 */
static uint8 s_ipc_flying_retry_div = 0U; /* 飞行状态 IPC 通知失败后的 100Hz 重试分频 */

/*
 * 函数功能: 在 IPS114 上显示飞控调试信息。
 * 输入参数: 无。
 * 返回值: 无。
 * 说明: 采用 8x16 字体显示7行数据，分别按类别分行。
 *       使用 sprintf 定宽带符号格式化，避免正负号导致数值跳动。
 */
void ips114_show_debug(void)
{
    const VL53L1X_data_struct *tof = VL53L1X_GetData();
    static uint8 screen_ready = 0U;
    char buf[64];

    ips114_set_font(IPS114_8X16_FONT);

    if (0U == screen_ready)
    {
        screen_ready = 1U;
        ips114_set_color(RGB565_WHITE, RGB565_BLACK);
        ips114_clear(); // 首次进入清屏并填充满黑色背景
    }

    ips114_set_color(RGB565_GREEN, RGB565_BLACK);

    // R1: ACC (X, Y, Z)
    snprintf(buf, sizeof(buf), "ACC % 7.2f% 7.2f% 7.2f",
             (double)g_imufilter_1000hz.accx, (double)g_imufilter_1000hz.accy, (double)g_imufilter_1000hz.accz);
    ips114_show_string(0U, 0U, buf);

    // R2: GYRO (X, Y, Z)
    snprintf(buf, sizeof(buf), "GYR % 6.1f % 6.1f % 6.1f",
             (double)g_imufilter_1000hz.gyrox, (double)g_imufilter_1000hz.gyroy, (double)g_imufilter_1000hz.gyroz);
    ips114_show_string(0U, 16U, buf);

    // R3: EULER (Roll, Pitch, Yaw)
    snprintf(buf, sizeof(buf), "EUL % 6.1f % 6.1f % 6.1f",
             (double)g_euler.roll, (double)g_euler.pitch, (double)g_euler.yaw);
    ips114_show_string(0U, 32U, buf);

    // R4: TOF 1-4 (Height_1, Height_2, Height_3, Height_4)
    snprintf(buf, sizeof(buf), "TOF %4u %4u %4u %4u       ",
             tof->distance_mm[0], tof->distance_mm[1], tof->distance_mm[2], tof->distance_mm[3]);
    ips114_show_string(0U, 48U, buf);

    // R5: Fused Height & Fused Vz
    snprintf(buf, sizeof(buf), "FUS H:% 6.1f Vz:% 7.3f  ",
             (double)g_tof_fused_height_mm, (double)g_height_fused_vz_mps);
    ips114_show_string(0U, 64U, buf);

    // R6: RC Status & CH1-CH4 (STD)
    snprintf(buf, sizeof(buf), "RC %1d % 5d % 5d % 5d % 5d",
             FC_START_CRSF_Get_State(), CRSF_STD[0], CRSF_STD[1], CRSF_STD[2], CRSF_STD[3]);
    ips114_show_string(0U, 80U, buf);

    // R7: Horizontal XY velocity (LC302 flow_x/y)
    snprintf(buf, sizeof(buf), "FLO % 5d % 5d          ",
             lc302_data.flow_x_integral, lc302_data.flow_y_integral);
    ips114_show_string(0U, 96U, buf);
}

volatile float g_car_encoder_left_front;
volatile float g_car_encoder_right_front;
volatile float g_car_encoder_left_rear;
volatile float g_car_encoder_right_rear;
volatile float g_car_imufilter_1000hz_accx;
volatile float g_car_imufilter_1000hz_accy;
volatile float g_car_imufilter_1000hz_accz;
volatile float g_car_imufilter_1000hz_gyrox;
volatile float g_car_imufilter_1000hz_gyroy;
volatile float g_car_imufilter_1000hz_gyroz;

static void on_car_data(const float *data, uint8 count)
{
    if (count >= 10)
    {
        g_car_encoder_left_front = data[0];
        g_car_encoder_right_front = data[1];
        g_car_encoder_left_rear = data[2];
        g_car_encoder_right_rear = data[3];
        g_car_imufilter_1000hz_accx = data[4];
        g_car_imufilter_1000hz_accy = data[5];
        g_car_imufilter_1000hz_accz = data[6];
        g_car_imufilter_1000hz_gyrox = data[7];
        g_car_imufilter_1000hz_gyroy = data[8];
        g_car_imufilter_1000hz_gyroz = data[9];
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();
    ips114_init();
    Beep_Init();
    pit_ms_init(PIT_CH2, 10);
    /* 副 IMU 初始化日志走 WiFi 文本链路，所以 wifi_cmd_Init 必须早于 ICM42688_Aux_Init。 */
    wifi_cmd_Init();
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
    wifi_justfloat_Init();
    wifi_params_Init();
    wifi_cal_imu_Init();
    Motor_Init();
    ipc_communicate_init(IPC_PORT_1, ipc_image_callback);
    FC_START_CRSF_Init();
    air_comm_air_init();
    wifi_justfloat_SetStandbyContext((0U == FC_START_CRSF_Get_State()) && (0U == FC_START_CRSF_Is_Armed()));
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
    air_comm_set_run_data_callback(on_car_data);
    while (true)
    {
        uint16 guard = 0U;

        while ((g_tick_1000HZ > 0U) && (guard < 100U))
        {
            g_tick_1000HZ--;

            IMU_Update_1000HZ();
            Height_Est_update_1000HZ();
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
            FC_START_CRSF_UpdateLandingButton100Hz();
            FC_Loop_100Hz();
            air_comm_air_update_100HZ();

            float air_data[15];
            air_data[0] = g_tof_fused_height_mm;
            air_data[1] = g_euler.roll;
            air_data[2] = g_euler.pitch;
            air_data[3] = g_euler.yaw;
            air_data[4] = Pos_Est_vel_x;
            air_data[5] = Pos_Est_vel_y;
            air_data[6] = (float)FC_START_CRSF_Get_State();
            air_data[7] = (float)CRSF_STD[0];
            air_data[8] = (float)CRSF_STD[1];
            air_data[9] = (float)CRSF_STD[2];
            air_data[10] = (float)CRSF_STD[3];
            air_data[11] = (float)CRSF_STD[4];
            air_data[12] = (float)CRSF_STD[5];
            air_data[13] = (float)CRSF_STD[6];
            air_data[14] = (float)CRSF_STD[7];
            air_comm_send_run_data(air_data, 15);

            wifi_justfloat(g_tof_fused_height_mm,
                           g_euler.roll,
                           g_euler.pitch,
                           g_euler.yaw,
                           Pos_Est_vel_x,
                           Pos_Est_vel_y,
                           (float)FC_START_CRSF_Get_State());
            wifi_justfloat_SetStandbyContext((FC_START_CRSF_STATE_STANDBY == FC_START_CRSF_Get_State()) && (0U == FC_START_CRSF_Is_Armed()));
            {
                uint8 flying = (FC_START_CRSF_STATE_FLYING == FC_START_CRSF_Get_State()) ? 1U : 0U;

                if (flying != s_ipc_last_flying)
                {
                    if (0U == s_ipc_flying_retry_div)
                    {
                        if (0U == ipc_flight_state_send(flying))
                        {
                            s_ipc_last_flying = flying;
                        }
                        s_ipc_flying_retry_div = 10U;
                    }
                    else
                    {
                        s_ipc_flying_retry_div--;
                    }
                }
                else
                {
                    s_ipc_flying_retry_div = 0U;
                }
            }
            if (FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_STANDBY)
            {
                // ips114_show_debug();
            }
            slot50 = div50;
            if (slot50 == 0U)
            {
                Pos_Est_Update_50HZ();
                crsf_send_50hz();
            }
            else
            {
                FC_Loop_50Hz();
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
#if (0U == WIFI_IMAGE_ENABLE)
        wifi_cmd_Poll();
#endif
        air_comm_air_poll();
    }
}
