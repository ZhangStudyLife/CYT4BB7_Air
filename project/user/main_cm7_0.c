#include "zf_common_headfile.h"
#include "../code/Planner/beacon_lost_detector.h"

volatile uint32 tick_1000us_cnt = 0U;
volatile uint16 g_tick_1000HZ = 0U;
volatile uint8 g_tick_100HZ = 0U;

static uint8 div500 = 0U;
static uint8 div50 = 0U;
static uint8 slot50 = 0U;
static uint8 div10 = 0U;
static uint8 s_ipc_last_flying = 0xFFU;   /* 上一次成功通知给核1的飞行状态 */
/* 上一次成功通知给核1的 2BL3 图传发送模式 */
static uint8 s_ipc_last_image_send_enable = 0xFFU;
static uint8 s_ipc_flying_retry_div = 0U; /* 飞行状态 IPC 通知失败后的 100Hz 重试分频 */

float g_car_vel_x = 0.0f; // 这个是车的速度 这个变量大于0 , 车往右
float g_car_vel_y = 0.0f; // 这个是车的速度 这个变量大于0 , 车往前

float g_car_sync_time_ms = 0.0f; /* Last car-side sync timestamp, unit: ms */

static void on_car_data(const float *data, uint8 count)
{
    if (count == 11U)
    {
        g_car_vel_x = data[0];
        g_car_vel_y = data[1];
        g_car_sync_time_ms = data[10];
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();
    Beep_Init();
    pit_ms_init(PIT_CH2, 10);
    /* 副 IMU 初始化日志走 WiFi 文本链路，所以 wifi_cmd_Init 必须早于 ICM42688_Aux_Init。 */
    wifi_cmd_Init();
    TOF_Init();
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
    BeaconLostDetector_Init();
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
    Motor_SetThrottleAll((int32[]){1200, 0, 0, 0});
    system_delay_ms(500);
    Motor_SetThrottleAll((int32[]){0, 1200, 0, 0});
    system_delay_ms(500);
    Motor_SetThrottleAll((int32[]){0, 0, 1200, 0});
    system_delay_ms(500);
    Motor_SetThrottleAll((int32[]){0, 0, 0, 1200});
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
            // ICM42688_Aux_Update_1000Hz(tick_1000us_cnt);         //对比用的陀螺仪关掉
            // BMI088_Update_1000Hz(tick_1000us_cnt);
            Pos_Est_Update_1000HZ();
            // wifi_justfloat();
            // {
            //     const VL53L1X_data_struct *tof = VL53L1X_GetData();
            //     ipc_camera_spi_log_t spi_log;
            //     ipc_camera_spi_log_get(&spi_log);
            //     wifi_justfloat(ICM42688.gyro_x,
            //                    ICM42688.gyro_y,
            //                    ICM42688.gyro_z,
            //                    ICM42688.acc_x,
            //                    ICM42688.acc_y,
            //                    ICM42688.acc_z,
            //                    tof->distance_mm[0],
            //                    tof->distance_mm[1],
            //                    tof->distance_mm[2],
            //                    tof->distance_mm[3],
            //                    lc302_data.flow_x_integral,
            //                    lc302_data.flow_y_integral,
            //                    lc302_data.integration_timespan,
            //                    lc302_data.ground_distance,
            //                    lc302_data.valid,
            //                    lc302_data.version,
            //                    CRSF_CH[0],
            //                    spi_log.board[0].online,
            //                    spi_log.board[0].first_beacon_x,
            //                    spi_log.board[0].first_beacon_y,
            //                    spi_log.board[1].online,
            //                    spi_log.board[1].first_beacon_x,
            //                    spi_log.board[1].first_beacon_y,
            //                    spi_log.seq,
            //                    spi_log.board[0].last_error,
            //                    spi_log.board[0].rx_ok_count,
            //                    spi_log.board[0].rx_error_count,
            //                    spi_log.board[0].last_rx_head0,
            //                    spi_log.board[0].last_rx_head1,
            //                    spi_log.board[1].last_error,
            //                    spi_log.board[1].rx_ok_count,
            //                    spi_log.board[1].rx_error_count,
            //                    spi_log.board[1].last_rx_head0,
            //                    spi_log.board[1].last_rx_head1);
            // }

            // wifi_justfloat(g_euler.roll, g_euler.pitch, g_euler.yaw, Pos_Est_vel_x, Pos_Est_vel_y);

                //                    tof->distance_mm[0],
                //                    tof->distance_mm[1],
                //                    tof->distance_mm[2],
                //                    tof->distance_mm[3],
                div500++;
            if (div500 >= 2U)
            {
                div500 = 0U;
                FC_Loop_500Hz();
            }

            FC_Loop_1000Hz();
            guard++;
        }

#if (0U == WIFI_IMAGE_ENABLE)
        wifi_cmd_Poll();
#endif

        if (g_tick_100HZ > 0U)
        {
            g_tick_100HZ--;
            Height_Est_update_100HZ();
            CRSF_Update_100HZ();
            FC_START_CRSF_UpdateLandingButton100Hz();
            FC_Loop_100Hz();
            air_comm_air_update_100HZ();
            ipc_image_poll();
            (void)BeaconLostDetector_Update();

            car_plan_result_t car_plan;
            uint8 car_plan_send_valid;
            (void)CarPlan_Update(&car_plan);
            car_plan_send_valid = ((car_plan.valid != 0U) && (g_tof_fused_height_mm > 500.0f)) ? 1U : 0U;

            float air_data[24];
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
            air_data[15] = yaw_angle_target;
            air_data[16] = (float)tick_1000us_cnt;
            air_data[17] = (float)car_plan_send_valid;
            air_data[18] = (car_plan_send_valid != 0U) ? car_plan.target_strafe_mps : 0.0f;
            air_data[19] = (car_plan_send_valid != 0U) ? car_plan.target_forward_mps : 0.0f;
            air_data[20] = (float)car_plan.camera;
            air_data[21] = (float)car_plan.beacon_index;
            air_data[22] = car_plan.dist_px;
            air_data[23] = (float)BeaconLostDetector_GetFlag();
            air_comm_send_run_data(air_data, 24);

            // wifi_justfloat(image_data[Front].car_lamp_data[0].cx,
            //                 image_data[Front].car_lamp_data[0].cy,
            //                 image_data[Center].car_lamp_data[0].cx,
            //                 image_data[Center].car_lamp_data[0].cy,
            //                 image_data[Back].car_lamp_data[0].cx,
            //                 image_data[Back].car_lamp_data[0].cy,
            //                 g_euler.roll,
            //                 g_euler.pitch,
            //                 g_euler.yaw,
            //                 Pos_Est_vel_x,
            //                 Pos_Est_vel_y,
            //                 g_tof_fused_height_mm
            //             );

            wifi_justfloat_SetStandbyContext((FC_START_CRSF_STATE_STANDBY == FC_START_CRSF_Get_State()) && (0U == FC_START_CRSF_Is_Armed()));
            {
                uint8 flying = (FC_START_CRSF_STATE_FLYING == FC_START_CRSF_Get_State()) ? 1U : 0U;
                uint8 image_send_enable = g_2bl3_image_send_enable;

                if(image_send_enable > 2U)
                {
                    image_send_enable = 0U;
                }

                if ((flying != s_ipc_last_flying) || (image_send_enable != s_ipc_last_image_send_enable))
                {
                    if (0U == s_ipc_flying_retry_div)
                    {
                        if (0U == ipc_flight_state_send(flying, image_send_enable))
                        {
                            s_ipc_last_flying = flying;
                            s_ipc_last_image_send_enable = image_send_enable;
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
            slot50 = div50;
            if (slot50 == 0U)
            {
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

                if (g_euler.roll > 45.0f || g_euler.roll < -45.0f ||
                    g_euler.pitch > 45.0f || g_euler.pitch < -45.0f)
                {
                    FC_START_CRSF_Trigger_Emergency_Stop();
                }
            }
        }
    }
}
