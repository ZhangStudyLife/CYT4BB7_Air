#include "zf_common_headfile.h"
#include "../code/Planner/beacon_lost_detector.h"
#include "../code/Estimation/Pos_Est/FlowGyroDecoupler_LC302.h"

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
static uint8 s_ipc_last_screen_refresh_enable = 0xFFU;
static uint8 s_ipc_flying_retry_div = 0U; /* 飞行状态 IPC 通知失败后的 100Hz 重试分频 */
static uint8 s_ipc_state_periodic_div = 0U;

#define AIR_RUN_DATA_CRITICAL_COUNT       (15U) /* 飞行期间下发的关键数据数量 */
#define AIR_RUN_DATA_DIAGNOSTIC_COUNT     (52U) /* 常态下发的完整诊断数据数量 */
#define AIR_RUN_CRITICAL_STATE            (0U)  /* 飞机运行状态 */
#define AIR_RUN_CRITICAL_CRSF_CH0         (1U)  /* CRSF标准化通道0 */
#define AIR_RUN_CRITICAL_CRSF_CH1         (2U)  /* CRSF标准化通道1 */
#define AIR_RUN_CRITICAL_CRSF_CH2         (3U)  /* CRSF标准化通道2 */
#define AIR_RUN_CRITICAL_CRSF_CH3         (4U)  /* CRSF标准化通道3 */
#define AIR_RUN_CRITICAL_CRSF_CH4         (5U)  /* CRSF标准化通道4 */
#define AIR_RUN_CRITICAL_CRSF_CH5         (6U)  /* CRSF标准化通道5 */
#define AIR_RUN_CRITICAL_CRSF_CH6         (7U)  /* CRSF标准化通道6 */
#define AIR_RUN_CRITICAL_CRSF_CH7         (8U)  /* CRSF标准化通道7 */
#define AIR_RUN_CRITICAL_CRSF_CH8         (9U)  /* CRSF标准化通道8 */
#define AIR_RUN_CRITICAL_YAW_TARGET       (10U) /* 飞机yaw目标角，单位deg */
#define AIR_RUN_CRITICAL_PLAN_VALID       (11U) /* 车模规划结果有效标志 */
#define AIR_RUN_CRITICAL_PLAN_STRAFE      (12U) /* 车模规划横移速度，单位m/s */
#define AIR_RUN_CRITICAL_PLAN_FORWARD     (13U) /* 车模规划前进速度，单位m/s */
#define AIR_RUN_CRITICAL_BEACON_LOST      (14U) /* 信标丢失标志 */

float g_car_vel_x = 0.0f; // 这个是车的速度 这个变量大于0 , 车往右
float g_car_vel_y = 0.0f; // 这个是车的速度 这个变量大于0 , 车往前
float g_car_yaw = 0.0f; /* Car yaw angle, unit: deg */
float g_car_yaw_rate_dps = 0.0f; /* 10Hz low-pass car yaw rate, unit: deg/s */
float g_car_sync_time_ms = 0.0f; /* Last car-side sync timestamp, unit: ms */
uint32 g_car_last_update_time_ms = 0U; /* 最近一次收到新车端时间戳的飞机本机时刻，单位ms */

/**
 * @brief 接收并保存小车实时运行数据。
 * @param data 小车发送的float数据数组。
 * @param count 数组中的float数量，当前协议固定为11。
 * @return 无。
 */
static void on_car_data(const float *data, uint8 count)
{
    if (count == 11U)
    {
        g_car_vel_x = data[0];
        g_car_vel_y = data[1];
        g_car_yaw = data[3];
        g_car_yaw_rate_dps = data[4];
        /* 仅在车端时间戳推进时刷新数据新鲜时刻。 */
        if (data[10] != g_car_sync_time_ms)
        {
            g_car_sync_time_ms = data[10];
            g_car_last_update_time_ms = tick_1000us_cnt;
        }
    }
}

/**
 * @brief 按飞机状态向车端发送关键运行数据或完整诊断数据。
 * @param car_plan 当前车模规划结果。
 * @param car_plan_send_valid 允许下发规划结果时为1，否则为0。
 * @return 无。
 */
static void send_air_run_data_100hz(const car_plan_result_t *car_plan,
                                    uint8 car_plan_send_valid)
{
    FC_START_CRSF_state_e state = FC_START_CRSF_Get_State();
    float plan_strafe_mps = (car_plan_send_valid != 0U) ? car_plan->target_strafe_mps : 0.0f;
    float plan_forward_mps = (car_plan_send_valid != 0U) ? car_plan->target_forward_mps : 0.0f;
    float beacon_lost = (float)BeaconLostDetector_GetFlag();

    if ((state == FC_START_CRSF_STATE_TAKEOFF) ||
        (state == FC_START_CRSF_STATE_FLYING) ||
        (state == FC_START_CRSF_STATE_LANDING))
    {
        float air_data[AIR_RUN_DATA_CRITICAL_COUNT];

        air_data[AIR_RUN_CRITICAL_STATE] = (float)state;
        air_data[AIR_RUN_CRITICAL_CRSF_CH0] = (float)CRSF_STD[0];
        air_data[AIR_RUN_CRITICAL_CRSF_CH1] = (float)CRSF_STD[1];
        air_data[AIR_RUN_CRITICAL_CRSF_CH2] = (float)CRSF_STD[2];
        air_data[AIR_RUN_CRITICAL_CRSF_CH3] = (float)CRSF_STD[3];
        air_data[AIR_RUN_CRITICAL_CRSF_CH4] = (float)CRSF_STD[4];
        air_data[AIR_RUN_CRITICAL_CRSF_CH5] = (float)CRSF_STD[5];
        air_data[AIR_RUN_CRITICAL_CRSF_CH6] = (float)CRSF_STD[6];
        air_data[AIR_RUN_CRITICAL_CRSF_CH7] = (float)CRSF_STD[7];
        air_data[AIR_RUN_CRITICAL_CRSF_CH8] = (float)CRSF_STD[8];
        air_data[AIR_RUN_CRITICAL_YAW_TARGET] = yaw_angle_target;
        air_data[AIR_RUN_CRITICAL_PLAN_VALID] = (float)car_plan_send_valid;
        air_data[AIR_RUN_CRITICAL_PLAN_STRAFE] = plan_strafe_mps;
        air_data[AIR_RUN_CRITICAL_PLAN_FORWARD] = plan_forward_mps;
        air_data[AIR_RUN_CRITICAL_BEACON_LOST] = beacon_lost;
        (void)air_comm_send_run_data(air_data, AIR_RUN_DATA_CRITICAL_COUNT);
        return;
    }

    {
        ipc_camera_spi_log_t camera_spi_log;
        float air_data[AIR_RUN_DATA_DIAGNOSTIC_COUNT];

        ipc_camera_spi_log_get(&camera_spi_log);

        air_data[0] = g_tof_fused_height_mm;
        air_data[1] = g_euler.roll;
        air_data[2] = g_euler.pitch;
        air_data[3] = g_euler.yaw;
        air_data[4] = Pos_Est_vel_x;
        air_data[5] = Pos_Est_vel_y;
        air_data[6] = (float)state;
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
        air_data[18] = plan_strafe_mps;
        air_data[19] = plan_forward_mps;
        air_data[20] = (float)car_plan->camera;
        air_data[21] = (float)car_plan->beacon_index;
        air_data[22] = car_plan->dist_px;
        air_data[23] = beacon_lost;
        air_data[24] = (float)CRSF_STD[8];
        air_data[25] = g_tof1_height_mm;
        air_data[26] = g_tof2_height_mm;
        air_data[27] = g_tof3_height_mm;
        air_data[28] = g_tof4_height_mm;
        air_data[29] = (float)lc302_data.flow_x_integral;
        air_data[30] = (float)lc302_data.flow_y_integral;
        air_data[31] = FlowGyroDecoupler_LC302_GetDecX();
        air_data[32] = FlowGyroDecoupler_LC302_GetDecY();
        IMU_GetRawSampleForCalibration(&air_data[33], &air_data[34], &air_data[35],
                                       &air_data[36], &air_data[37], &air_data[38]);
        air_data[39] = g_imufilter_1000hz.gyrox;
        air_data[40] = g_imufilter_1000hz.gyroy;
        air_data[41] = g_imufilter_1000hz.gyroz;
        air_data[42] = g_imufilter_1000hz.accx;
        air_data[43] = g_imufilter_1000hz.accy;
        air_data[44] = g_imufilter_1000hz.accz;
        air_data[45] = (float)(camera_spi_log.board[0].online |
                               (gpio_get_level(P01_0) << 1));
        air_data[46] = (float)(camera_spi_log.board[1].online |
                               (gpio_get_level(P01_1) << 1));
        air_data[47] = (float)(((uint16)camera_spi_log.board[0].last_error << 8) |
                               (uint16)camera_spi_log.board[1].last_error);
        air_data[48] = (float)camera_spi_log.board[0].last_rx_head0;
        air_data[49] = (float)camera_spi_log.board[0].last_rx_head1;
        air_data[50] = (float)camera_spi_log.board[1].last_rx_head0;
        air_data[51] = (float)camera_spi_log.board[1].last_rx_head1;
        (void)air_comm_send_run_data(air_data, AIR_RUN_DATA_DIAGNOSTIC_COUNT);
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
    Pos_Est_Init_2();
    FC_Loop_Init();
    BeaconLostDetector_Init();
    wifi_justfloat_Init();
    wifi_params_Init();
    wifi_cal_imu_Init();
    Motor_Init();
    ipc_communicate_init(IPC_PORT_1, ipc_image_callback);
    ipc_remote_param_core0_init();
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
            Pos_Est_Update_1000HZ_2();
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
            ipc_attitude_publish(g_euler.roll,
                                 g_euler.pitch,
                                 g_tof_fused_height_mm,
                                 g_tof_fused_valid);
            ipc_image_poll();
            {
                ipc_camera_spi_log_t camera_spi_log;

                ipc_camera_spi_log_get(&camera_spi_log);
                wifi_justfloat((float)g_image_data_seq,
                               (float)camera_spi_log.seq,
                               (float)camera_spi_log.board[0].rx_ok_count,
                               (float)camera_spi_log.board[1].rx_ok_count,
                               (float)camera_spi_log.board[0].rx_error_count,
                               (float)camera_spi_log.board[1].rx_error_count,
                               (float)camera_spi_log.board[0].online,
                               (float)camera_spi_log.board[1].online,
                               (float)image_data[Front].car_lamp_data[0].valid,
                               image_data[Front].car_lamp_data[0].cx,
                               image_data[Front].car_lamp_data[0].cy,
                               (float)image_data[Center].car_lamp_data[0].valid,
                               image_data[Center].car_lamp_data[0].cx,
                               image_data[Center].car_lamp_data[0].cy,
                               (float)image_data[Back].car_lamp_data[0].valid,
                               image_data[Back].car_lamp_data[0].cx,
                               image_data[Back].car_lamp_data[0].cy);
            }
            (void)BeaconLostDetector_Update();

            car_plan_result_t car_plan = {0};
            car_plan_2_result_t car_plan_2 = {0};
            uint8 car_plan_send_valid;
            if (FC_START_CRSF_Get_Flight_Mode() != FC_START_CRSF_FLIGHT_MODE_8)
            {
                (void)CarPlan_Update(&car_plan);
                (void)CarPlan_2_Update(&car_plan_2);
                if (Car_Plan_Mode >= 1.5f)
                {
                    car_plan.valid = car_plan_2.valid;
                    car_plan.target_strafe_mps = car_plan_2.target_strafe_mps;
                    car_plan.target_forward_mps = car_plan_2.target_forward_mps;
                }
            }
            car_plan_send_valid = ((car_plan.valid != 0U) && (g_tof_fused_height_mm > 500.0f)) ? 1U : 0U;


    //    wifi_justfloat(  image_data[Front].beacon_data[0].x,          /* I1 */
    //                     image_data[Front].beacon_data[0].y,          /* I2 */
    //                     image_data[Front].beacon_data[0].area,          /* I3 */
                        
    //                     image_data[Front].beacon_data[1].x,          /* I4 */
    //                     image_data[Front].beacon_data[1].y,          /* I5 */
    //                     image_data[Front].beacon_data[1].area,          /* I6 */
                        
    //                     image_data[Center].beacon_data[0].x,         /* I7 */
    //                     image_data[Center].beacon_data[0].y,         /* I8 */
    //                     image_data[Center].beacon_data[0].area,        /* I9 */

    //                     image_data[Center].beacon_data[1].x,         /* I10 */
    //                     image_data[Center].beacon_data[1].y,         /* I11 */
    //                     image_data[Center].beacon_data[1].area,        /* I12 */

    //                     image_data[Back].beacon_data[0].x,           /* I13 */
    //                     image_data[Back].beacon_data[0].y,           /* I14 */
    //                     image_data[Back].beacon_data[0].area,           /* I15 */

    //                     image_data[Back].beacon_data[1].x,           /* I16 */
    //                     image_data[Back].beacon_data[1].y,           /* I17 */
    //                     image_data[Back].beacon_data[1].area,           /* I18 */

    //                     image_data[Front].car_lamp_data[0].cx,       /* I19 */
    //                     image_data[Front].car_lamp_data[0].cy,       /* I20 */
    //                     image_data[Front].car_lamp_data[0].angle,    /* I21 */
    //                     image_data[Front].car_lamp_data[0].width,    /* I22 */
    //                     image_data[Front].car_lamp_data[0].length,   /* I23 */
    //                     image_data[Center].car_lamp_data[0].cx,      /* I24 */
    //                     image_data[Center].car_lamp_data[0].cy,      /* I25 */
    //                     image_data[Center].car_lamp_data[0].angle,   /* I26 */
    //                     image_data[Center].car_lamp_data[0].width,   /* I27 */
    //                     image_data[Center].car_lamp_data[0].length,  /* I28 */
    //                     image_data[Back].car_lamp_data[0].cx,        /* I29 */
    //                     image_data[Back].car_lamp_data[0].cy,        /* I30 */
    //                     image_data[Back].car_lamp_data[0].angle,     /* I31 */
    //                     image_data[Back].car_lamp_data[0].width,     /* I32 */
    //                     image_data[Back].car_lamp_data[0].length,    /* I33 */
    //                     g_euler.pitch,                              /* I34 */
    //                     g_euler.roll,                               /* I35 */
    //                     g_euler.yaw,                                /* I36 */
    //                     g_car_sync_time_ms,                         /* I37 */
    //                     g_car_vel_x, g_car_vel_y, g_car_yaw,
    //                     car_plan.target_forward_mps,car_plan.target_strafe_mps
    //                     // g_car_lamp_fused.cx,                        /* I38 */
    //                     // g_car_lamp_fused.cy,                        /* I39 */
    //                     // g_car_lamp_fused_distance_projectioncenter_2.x_cm,
    //                     // g_car_lamp_fused_distance_projectioncenter_2.y_cm
    //                     // g_tof_fused_height_mm,
    //                     // g_height_fused_vz_mps,
    //                     // CRSF_STD[8]
    //                     );
            send_air_run_data_100hz(&car_plan, car_plan_send_valid);

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
                uint8 screen_refresh_enable =
                    ((FC_START_CRSF_STATE_STANDBY == FC_START_CRSF_Get_State()) &&
                     (0U == FC_START_CRSF_Is_Armed())) ? 1U : 0U;

                if(image_send_enable > 2U)
                {
                    image_send_enable = 0U;
                }

                if ((flying != s_ipc_last_flying) ||
                    (image_send_enable != s_ipc_last_image_send_enable) ||
                    (screen_refresh_enable != s_ipc_last_screen_refresh_enable) ||
                    (0U == s_ipc_state_periodic_div))
                {
                    if (0U == s_ipc_flying_retry_div)
                    {
                        if (0U == ipc_flight_state_send(flying,
                                                       image_send_enable,
                                                       screen_refresh_enable))
                        {
                            s_ipc_last_flying = flying;
                            s_ipc_last_image_send_enable = image_send_enable;
                            s_ipc_last_screen_refresh_enable = screen_refresh_enable;
                            s_ipc_state_periodic_div = 100U;
                            s_ipc_flying_retry_div = 0U;
                        }
                        else
                        {
                            s_ipc_flying_retry_div = 10U;
                        }
                    }
                    else
                    {
                        s_ipc_flying_retry_div--;
                    }
                }
                else
                {
                    s_ipc_flying_retry_div = 0U;
                    s_ipc_state_periodic_div--;
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
