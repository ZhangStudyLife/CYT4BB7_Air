#include "zf_common_headfile.h"
#include "../code/FlightController/fc_mode.h"
#include "../code/Planner/beacon_lost_detector.h"
#include "../code/Planner/car_lamp_fused.h"
#include "../code/Estimation/Pos_Est/FlowGyroDecoupler_LC302.h"

volatile uint32 tick_1000us_cnt = 0U;
volatile uint16 g_tick_1000HZ = 0U;
static uint16 s_air_comm_beep_tick = 200U; /* 空地串口断联蜂鸣器的100Hz节拍计数 */

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
float g_car_vel_target_x = 0.0f;
float g_car_vel_target_y = 0.0f;
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
        g_car_vel_target_x = data[5];
        g_car_vel_target_y = data[6];
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

/**
 * @brief 执行一次Core 0高速任务，并在内部完成500Hz飞控分频。
 * @param 无。
 * @return 无。
 */
static void core0_run_fast_loop_step(void)
{
    static uint8 div500 = 0U;

    (void)ipc_image_poll();
    IMU_Update_1000HZ();
    Pos_Est_Update_1000HZ();
    Pos_Est_Update_1000HZ_2();

    div500++;
    if (div500 >= 2U)
    {
        div500 = 0U;
        FC_Loop_500Hz();
    }

    FC_Loop_1000Hz();
    air_comm_air_poll();
}

/**
 * @brief 在100Hz槽位维护核1飞行状态、图传发送模式和屏幕刷新状态。
 * @param 无。
 * @return 无。
 */
static void core0_update_ipc_state_100hz(void)
{
    static uint8 last_flying = 0xFFU;
    static uint8 last_image_send_enable = 0xFFU;
    static uint8 last_screen_refresh_enable = 0xFFU;
    static uint8 flying_retry_div = 0U;
    static uint8 state_periodic_div = 0U;
    uint8 flying = (FC_START_CRSF_STATE_FLYING == FC_START_CRSF_Get_State()) ? 1U : 0U;
    uint8 image_send_enable = g_2bl3_image_send_enable;
    uint8 screen_refresh_enable =
        ((FC_START_CRSF_STATE_STANDBY == FC_START_CRSF_Get_State()) &&
         (0U == FC_START_CRSF_Is_Armed())) ? 1U : 0U;

    wifi_justfloat_SetStandbyContext(screen_refresh_enable);

    if(image_send_enable > 2U)
    {
        image_send_enable = 0U;
    }

    if ((flying != last_flying) ||
        (image_send_enable != last_image_send_enable) ||
        (screen_refresh_enable != last_screen_refresh_enable) ||
        (0U == state_periodic_div))
    {
        if (0U == flying_retry_div)
        {
            if (0U == ipc_flight_state_send(flying,
                                            image_send_enable,
                                            screen_refresh_enable))
            {
                last_flying = flying;
                last_image_send_enable = image_send_enable;
                last_screen_refresh_enable = screen_refresh_enable;
                state_periodic_div = 100U;
                flying_retry_div = 0U;
            }
            else
            {
                flying_retry_div = 10U;
            }
        }
        else
        {
            flying_retry_div--;
        }
    }
    else
    {
        flying_retry_div = 0U;
        state_periodic_div--;
    }
}

/**
 * @brief 更新信标与车模规划，并按高度条件发送本周期空地数据。
 * @param 无。
 * @return 无。
 */
static void core0_plan_and_send_100hz(void)
{
    car_plan_result_t car_plan = {0};
    car_plan_2_result_t car_plan_2 = {0};
    uint8 car_plan_send_valid;

    (void)BeaconLostDetector_Update();
    if (FC_START_CRSF_Get_Flight_Mode() != FC_START_CRSF_FLIGHT_MODE_8)
    {
        (void)CarPlan_Update(&car_plan);
        (void)CarPlan_2_Update(&car_plan_2);
        if ((FC_START_CRSF_Get_Flight_Mode() == FC_START_CRSF_FLIGHT_MODE_1) ||
            (Car_Plan_Mode >= 1.5f))
        {
            car_plan.valid = car_plan_2.valid;
            car_plan.target_strafe_mps = car_plan_2.target_strafe_mps;
            car_plan.target_forward_mps = car_plan_2.target_forward_mps;
        }
    }

    car_plan_send_valid = ((car_plan.valid != 0U) && (g_tof_fused_height_mm > 500.0f)) ? 1U : 0U;
    send_air_run_data_100hz(&car_plan, car_plan_send_valid);

    (void)wifi_justfloat((float)FC_START_CRSF_Get_Flight_Mode(),             /* I1  flight mode */
                             (float)FC_START_CRSF_Get_State(),                   /* I2  flight state */
                             (float)g_image_data_seq,                            /* I3  image sequence */
                             (float)g_car_lamp_fused.valid,                      /* I4  image valid */
                             (float)g_tof_fused_valid,                           /* I5  height valid */
                             g_tof_fused_height_mm,                              /* I6  height, mm */
                             g_car_sync_time_ms,                                 /* I7  car time, ms */
                             (g_car_sync_time_ms > 0.0f)
                                 ? (float)(tick_1000us_cnt - g_car_last_update_time_ms)
                                 : -1.0f,                                        /* I8  car data age, ms */
                             roll_angle_target, pitch_angle_target, yaw_angle_target, /* I9-I11 target Euler, deg */
                             g_euler.roll, g_euler.pitch, g_euler.yaw,           /* I12-I14 actual Euler, deg */
                             g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy, /* I15-I16 gyro, deg/s */
                             g_car_lamp_fused.cx,                                /* I17 car center X, px */
                             g_car_lamp_fused.cy,                                /* I18 car center Y, px */
                             g_car_lamp_fused.angle,                             /* I19 car image angle, deg */
                             (float)car_plan_2.valid,                             /* I20 CarPlan_2 valid */
                             car_plan_2.target_center_x,                         /* I21 target center X, px */
                             car_plan_2.target_center_y,                         /* I22 target center Y, px */
                             (float)car_plan_2.camera_mask,                       /* I23 target camera mask */
                             g_mode2_imgx_pid.error, g_mode2_imgy_pid.error,      /* I24-I25 image error, px */
                             (float)car_plan_send_valid,                          /* I26 sent plan valid */
                             (car_plan_send_valid != 0U) ? car_plan.target_strafe_mps : 0.0f, /* I27 sent strafe, m/s */
                             (car_plan_send_valid != 0U) ? car_plan.target_forward_mps : 0.0f, /* I28 sent forward, m/s */
                             g_car_yaw, g_car_yaw_rate_dps,                       /* I29-I30 car yaw/rate */
                             g_car_vel_x, g_car_vel_y,                            /* I31-I32 car velocity, m/s */
                             image_data[Front].beacon_data[0].x,                 /* I33 Front beacon0 X, px */
                             image_data[Front].beacon_data[0].y,                 /* I34 Front beacon0 Y, px */
                             image_data[Front].beacon_data[0].area,              /* I35 Front beacon0 area */
                             image_data[Center].beacon_data[0].x,                /* I36 Center beacon0 X, px */
                             image_data[Center].beacon_data[0].y,                /* I37 Center beacon0 Y, px */
                             image_data[Center].beacon_data[0].area,             /* I38 Center beacon0 area */
                             image_data[Back].beacon_data[0].x,                  /* I39 Back beacon0 X, px */
                             image_data[Back].beacon_data[0].y,                  /* I40 Back beacon0 Y, px */
                             image_data[Back].beacon_data[0].area);              /* I41 Back beacon0 area */
}

/**
 * @brief 执行当前1ms墙钟对应的低频任务槽位。
 * @param slot 当前槽位编号，范围为0至9。
 * @return 无。
 */
static void core0_run_slow_slot(uint8 slot)
{
    static uint8 div50 = 0U;
    static uint8 div10 = 0U;

    switch (slot)
    {
    case 1U:
        Height_Est_update_100HZ();
        CRSF_Update_100HZ();
        FC_START_CRSF_UpdateLandingButton100Hz();
        (void)ipc_image_poll();
        FC_Loop_100Hz();
        break;

    case 2U:
    case 4U:
    case 6U:
    case 8U:
        VL53L1X_RequestUpdate();
        VL53L1X_TaskStep();
        break;

    case 3U:
        air_comm_air_update_100HZ();
        if (air_comm_air_is_car_online() == 0U)
        {
            if (s_air_comm_beep_tick >= 200U)
            {
                s_air_comm_beep_tick = 0U;
                Beep_SetAlarm(BEEP_ALARM_CAR_DATA_LOST, 1U);
            }
            else if (s_air_comm_beep_tick == 100U)
            {
                Beep_SetAlarm(BEEP_ALARM_CAR_DATA_LOST, 0U);
            }
            s_air_comm_beep_tick++;
        }
        else if (s_air_comm_beep_tick != 200U)
        {
            s_air_comm_beep_tick = 200U;
            Beep_SetAlarm(BEEP_ALARM_CAR_DATA_LOST, 0U);
        }
        ipc_attitude_publish(g_euler.roll,
                             g_euler.pitch,
                             g_tof_fused_height_mm,
                             g_tof_fused_valid);
        break;

    case 5U:
        core0_plan_and_send_100hz();
        break;

    case 7U:
        core0_update_ipc_state_100hz();
        if (div50 != 0U)
        {
            FC_Loop_50Hz();
        }
        div50++;
        if (div50 >= 2U)
        {
            div50 = 0U;
        }
        break;

    case 9U:
        div10++;
        if (div10 >= 10U)
        {
            div10 = 0U;
            FC_START_CRSF_Update();

            if (g_euler.roll > 75.0f || g_euler.roll < -75.0f ||
                g_euler.pitch > 75.0f || g_euler.pitch < -75.0f)
            {
                FC_START_CRSF_Trigger_Emergency_Stop();
            }

            /* 1kHz出现新积压时直接放弃低优先级CRSF姿态回传。 */
            if (g_tick_1000HZ == 0U)
            {
                crsf_send_10hz();
            }
        }
        break;

    default:
        break;
    }
}

int main(void)
{
    uint32 slow_tick_last;

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
    ipc_image_init();
    FC_START_CRSF_Init();
    air_comm_air_init();
    wifi_justfloat_SetStandbyContext((0U == FC_START_CRSF_Get_State()) &&
                                     (0U == FC_START_CRSF_Is_Armed()));

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
    pit_us_init(PIT_CH0, 1000);
    slow_tick_last = tick_1000us_cnt;

    while (true)
    {
        uint16 guard = 0U;

        while ((g_tick_1000HZ > 0U) && (guard < 100U))
        {
            g_tick_1000HZ--;
            core0_run_fast_loop_step();
            guard++;
        }

#if (0U == WIFI_IMAGE_ENABLE)
        wifi_cmd_Poll();
#endif

        {
            uint32 slow_tick_now = tick_1000us_cnt;

            if ((g_tick_1000HZ == 0U) && (slow_tick_now != slow_tick_last))
            {
                slow_tick_last = slow_tick_now;
                core0_run_slow_slot((uint8)(slow_tick_now % 10U));
            }
        }
    }
}
