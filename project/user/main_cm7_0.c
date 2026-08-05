#include "zf_common_headfile.h"
#include "../code/Planner/beacon_lost_detector.h"
#include "../code/Estimation/Pos_Est/FlowGyroDecoupler_LC302.h"

#define CORE0_PROFILE_DWT_UNLOCK_KEY       (0xC5ACCE55UL) /* Core 0性能计时使用的DWT解锁键 */
#define CORE0_PROFILE_UPDATE_MAX(target, value)          /* 更新本次100Hz日志周期内的最大耗时 */ \
    do                                                     \
    {                                                      \
        if ((value) > (target))                            \
        {                                                  \
            (target) = (value);                            \
        }                                                  \
    } while (0)
#define CORE0_PROFILE_TO_US(cycles, cycles_per_us)       ((float)(cycles) / (cycles_per_us)) /* 将DWT周期数转换为微秒 */
#define CORE0_PROFILE_CRSF10_UPDATED       (1U << 0) /* 本日志窗口已执行CRSF 10Hz发送 */
#define CORE0_PROFILE_FC50_UPDATED         (1U << 1) /* 本日志窗口已执行飞控50Hz任务 */
#define CORE0_PROFILE_STATE10_UPDATED      (1U << 2) /* 本日志窗口已执行起降10Hz状态机 */

typedef struct
{
    uint32 imu_max_cycles;
    uint32 pos1_max_cycles;
    uint32 pos2_max_cycles;
    uint32 fc500_max_cycles;
    uint32 fc1000_max_cycles;
    uint32 air_poll_max_cycles;
    uint32 loop1000_max_cycles;
    uint32 tof_max_cycles;
    uint32 height_max_cycles;
    uint32 crsf_max_cycles;
    uint32 fc100_max_cycles;
    uint32 air_update_max_cycles;
    uint32 ipc_max_cycles;
    uint32 planner_max_cycles;
    uint32 air_send_max_cycles;
    uint32 ipc_state_max_cycles;
    uint32 crsf10_max_cycles;
    uint32 fc50_max_cycles;
    uint32 state10_max_cycles;
    uint32 loop100_max_cycles;
    uint32 task100_count;
    uint32 slow_tick_gap_max;
    uint32 slow_slot_skip_count;
    uint32 loop100_abort_count;
    uint16 backlog_max;
    uint16 guard_max;
    uint8 tof_max_step;
    uint8 tail_update_mask;
    uint8 send_pending;
} core0_profile_t;

volatile uint32 tick_1000us_cnt = 0U;
volatile uint16 g_tick_1000HZ = 0U;
volatile uint8 g_tick_100HZ = 0U;

static uint8 div500 = 0U;
static uint8 div50 = 0U;
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
    core0_profile_t profile = {0};
    float profile_cycles_per_us;
    uint32 profile_start_cycles;
    uint32 profile_period_start_cycles;
    uint32 profile_elapsed_cycles;
    uint32 profile_air_raw_rx_last = 0U;
    uint32 profile_air_overflow_last = 0U;
    uint32 profile_air_tx_overflow_last = 0U;
    uint32 profile_wifi_overflow_last = 0U;
    uint32 profile_tof_sample_seq_last = 0U;
    uint32 profile_log_last_cycles = 0U;
    uint32 profile_loop100_accum_cycles = 0U;
    uint32 profile_loop100_cycle = 0U;
    uint32 slow_tick_last = 0U;
    uint8 profile_loop100_active = 0U;
    uint8 profile_loop100_phase_mask = 0U;
    uint8 profile_tof_step;

    clock_init(SYSTEM_CLOCK_250M);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = CORE0_PROFILE_DWT_UNLOCK_KEY;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    profile_cycles_per_us = (float)SystemCoreClock / 1000000.0f;
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
    profile_log_last_cycles = DWT->CYCCNT;
    slow_tick_last = tick_1000us_cnt;
    while (true)
    {
        uint16 guard = 0U;

        while ((g_tick_1000HZ > 0U) && (guard < 100U))
        {
            profile_period_start_cycles = DWT->CYCCNT;
            if (g_tick_1000HZ > profile.backlog_max)
            {
                profile.backlog_max = g_tick_1000HZ;
            }
            g_tick_1000HZ--;

            profile_start_cycles = DWT->CYCCNT;
            IMU_Update_1000HZ();
            profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
            CORE0_PROFILE_UPDATE_MAX(profile.imu_max_cycles, profile_elapsed_cycles);
            // ICM42688_Aux_Update_1000Hz(tick_1000us_cnt);         //对比用的陀螺仪关掉
            // BMI088_Update_1000Hz(tick_1000us_cnt);
            profile_start_cycles = DWT->CYCCNT;
            Pos_Est_Update_1000HZ();
            profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
            CORE0_PROFILE_UPDATE_MAX(profile.pos1_max_cycles, profile_elapsed_cycles);
            profile_start_cycles = DWT->CYCCNT;
            Pos_Est_Update_1000HZ_2();
            profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
            CORE0_PROFILE_UPDATE_MAX(profile.pos2_max_cycles, profile_elapsed_cycles);
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
                profile_start_cycles = DWT->CYCCNT;
                FC_Loop_500Hz();
                profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                CORE0_PROFILE_UPDATE_MAX(profile.fc500_max_cycles, profile_elapsed_cycles);
            }

            profile_start_cycles = DWT->CYCCNT;
            FC_Loop_1000Hz();
            profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
            CORE0_PROFILE_UPDATE_MAX(profile.fc1000_max_cycles, profile_elapsed_cycles);

            profile_start_cycles = DWT->CYCCNT;
            air_comm_air_poll();
            profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
            CORE0_PROFILE_UPDATE_MAX(profile.air_poll_max_cycles, profile_elapsed_cycles);

            profile_elapsed_cycles = DWT->CYCCNT - profile_period_start_cycles;
            CORE0_PROFILE_UPDATE_MAX(profile.loop1000_max_cycles, profile_elapsed_cycles);
            guard++;
        }

        if (guard > profile.guard_max)
        {
            profile.guard_max = guard;
        }

#if (0U == WIFI_IMAGE_ENABLE)
        wifi_cmd_Poll();
#endif

        {
            uint32 slow_tick_now = tick_1000us_cnt;

            if ((g_tick_1000HZ == 0U) && (slow_tick_now != slow_tick_last))
            {
                uint32 slow_tick_gap = slow_tick_now - slow_tick_last;

                CORE0_PROFILE_UPDATE_MAX(profile.slow_tick_gap_max, slow_tick_gap);
                if (slow_tick_gap > 1U)
                {
                    profile.slow_slot_skip_count += slow_tick_gap - 1U;
                }
                slow_tick_last = slow_tick_now;

                if ((profile_loop100_active != 0U) &&
                    (profile_loop100_cycle != (slow_tick_now / 10U)))
                {
                    profile.loop100_abort_count++;
                    profile_loop100_active = 0U;
                    profile_loop100_phase_mask = 0U;
                }

                switch ((uint8)(slow_tick_now % 10U))
                {
                case 0U:
                    /* 槽0保留为空，100Hz统计在日志成功入队后统一清零。 */
                    break;

                case 1U:
                    /* 槽1：按高度、遥控、降落判定、100Hz飞控的依赖顺序执行。 */
                    g_tick_100HZ = 0U;
                    profile_period_start_cycles = DWT->CYCCNT;
                    profile_loop100_accum_cycles = 0U;
                    profile_loop100_cycle = slow_tick_now / 10U;
                    profile_loop100_active = 1U;
                    profile_loop100_phase_mask = 1U;

                    profile_start_cycles = DWT->CYCCNT;
                    Height_Est_update_100HZ();
                    profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                    CORE0_PROFILE_UPDATE_MAX(profile.height_max_cycles, profile_elapsed_cycles);

                    profile_start_cycles = DWT->CYCCNT;
                    CRSF_Update_100HZ();
                    FC_START_CRSF_UpdateLandingButton100Hz();
                    profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                    CORE0_PROFILE_UPDATE_MAX(profile.crsf_max_cycles, profile_elapsed_cycles);

                    profile_start_cycles = DWT->CYCCNT;
                    FC_Loop_100Hz();
                    profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                    CORE0_PROFILE_UPDATE_MAX(profile.fc100_max_cycles, profile_elapsed_cycles);

                    profile_loop100_accum_cycles += DWT->CYCCNT - profile_period_start_cycles;
                    break;

                case 2U:
                case 4U:
                case 6U:
                case 8U:
                    /* 偶数槽：先轮询新的READY请求，再推进一个完整ToF软件IIC事务。 */
                    VL53L1X_RequestUpdate();
                    profile_start_cycles = DWT->CYCCNT;
                    profile_tof_step = VL53L1X_TaskStep();
                    profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                    if (profile_elapsed_cycles > profile.tof_max_cycles)
                    {
                        profile.tof_max_cycles = profile_elapsed_cycles;
                        profile.tof_max_step = profile_tof_step;
                    }
                    break;

                case 3U:
                    /* 槽3：维护空地协议，并在规划前刷新IPC数据。 */
                    profile_period_start_cycles = DWT->CYCCNT;

                    profile_start_cycles = DWT->CYCCNT;
                    air_comm_air_update_100HZ();
                    profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                    CORE0_PROFILE_UPDATE_MAX(profile.air_update_max_cycles, profile_elapsed_cycles);

                    profile_start_cycles = DWT->CYCCNT;
                    ipc_attitude_publish(g_euler.roll,
                                         g_euler.pitch,
                                         g_tof_fused_height_mm,
                                         g_tof_fused_valid);
                    ipc_image_poll();
                    profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                    CORE0_PROFILE_UPDATE_MAX(profile.ipc_max_cycles, profile_elapsed_cycles);

                    if (profile_loop100_active != 0U)
                    {
                        profile_loop100_accum_cycles += DWT->CYCCNT - profile_period_start_cycles;
                        profile_loop100_phase_mask |= (1U << 1);
                    }
                    break;

                case 5U:
                    /* 槽5：使用已刷新的图像数据规划，并提交本周期空地数据。 */
                    profile_period_start_cycles = DWT->CYCCNT;

                    profile_start_cycles = DWT->CYCCNT;
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
                    profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                    CORE0_PROFILE_UPDATE_MAX(profile.planner_max_cycles, profile_elapsed_cycles);


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
                    profile_start_cycles = DWT->CYCCNT;
                    send_air_run_data_100hz(&car_plan, car_plan_send_valid);
                    profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                    CORE0_PROFILE_UPDATE_MAX(profile.air_send_max_cycles, profile_elapsed_cycles);

                    if (profile_loop100_active != 0U)
                    {
                        profile_loop100_accum_cycles += DWT->CYCCNT - profile_period_start_cycles;
                        profile_loop100_phase_mask |= (1U << 2);
                    }
                    break;

                case 7U:
                    /* 槽7：维护IPC状态并执行飞控50Hz任务。 */
                    profile_period_start_cycles = DWT->CYCCNT;

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

                    profile_start_cycles = DWT->CYCCNT;
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
                    profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                    CORE0_PROFILE_UPDATE_MAX(profile.ipc_state_max_cycles, profile_elapsed_cycles);

                    if (div50 != 0U)
                    {
                        profile_start_cycles = DWT->CYCCNT;
                        FC_Loop_50Hz();
                        profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                        if ((profile.tail_update_mask & CORE0_PROFILE_FC50_UPDATED) == 0U)
                        {
                            profile.fc50_max_cycles = profile_elapsed_cycles;
                            profile.tail_update_mask |= CORE0_PROFILE_FC50_UPDATED;
                        }
                        else
                        {
                            CORE0_PROFILE_UPDATE_MAX(profile.fc50_max_cycles, profile_elapsed_cycles);
                        }
                    }

                    div50++;
                    if (div50 >= 2U)
                    {
                        div50 = 0U;
                    }

                    if (profile_loop100_active != 0U)
                    {
                        profile_loop100_accum_cycles += DWT->CYCCNT - profile_period_start_cycles;
                        profile_loop100_phase_mask |= (1U << 3);
                    }
                    break;

                case 9U:
                    /* 槽9：完成100Hz周期，先执行起降状态机，最后尝试低优先级CRSF回传。 */
                    profile_period_start_cycles = DWT->CYCCNT;
                    div10++;
                    if (div10 >= 10U)
                    {
                        div10 = 0U;
                        profile_start_cycles = DWT->CYCCNT;
                        FC_START_CRSF_Update();

                        if (g_euler.roll > 45.0f || g_euler.roll < -45.0f ||
                            g_euler.pitch > 45.0f || g_euler.pitch < -45.0f)
                        {
                            FC_START_CRSF_Trigger_Emergency_Stop();
                        }
                        profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                        if ((profile.tail_update_mask & CORE0_PROFILE_STATE10_UPDATED) == 0U)
                        {
                            profile.state10_max_cycles = profile_elapsed_cycles;
                            profile.tail_update_mask |= CORE0_PROFILE_STATE10_UPDATED;
                        }
                        else
                        {
                            CORE0_PROFILE_UPDATE_MAX(profile.state10_max_cycles, profile_elapsed_cycles);
                        }

                        /* 1kHz出现新积压时直接放弃本次姿态回传。 */
                        if (g_tick_1000HZ == 0U)
                        {
                            profile_start_cycles = DWT->CYCCNT;
                            crsf_send_10hz();
                            profile_elapsed_cycles = DWT->CYCCNT - profile_start_cycles;
                            if ((profile.tail_update_mask & CORE0_PROFILE_CRSF10_UPDATED) == 0U)
                            {
                                profile.crsf10_max_cycles = profile_elapsed_cycles;
                                profile.tail_update_mask |= CORE0_PROFILE_CRSF10_UPDATED;
                            }
                            else
                            {
                                CORE0_PROFILE_UPDATE_MAX(profile.crsf10_max_cycles, profile_elapsed_cycles);
                            }
                        }
                    }
                    if (profile_loop100_active != 0U)
                    {
                        profile_loop100_accum_cycles += DWT->CYCCNT - profile_period_start_cycles;
                        profile_loop100_phase_mask |= (1U << 4);
                        if (profile_loop100_phase_mask == 0x1FU)
                        {
                            CORE0_PROFILE_UPDATE_MAX(profile.loop100_max_cycles, profile_loop100_accum_cycles);
                            profile.task100_count++;
                        }
                        else
                        {
                            profile.loop100_abort_count++;
                        }
                        profile_loop100_active = 0U;
                        profile_loop100_phase_mask = 0U;
                    }
                    profile.send_pending = 1U;
                    break;

                default:
                    break;
                }
            }
        }

        if (profile.send_pending != 0U)
        {
            wifi_justfloat_tx_stats_t tx_before;
            wifi_justfloat_tx_stats_t tx_after;
            air_comm_air_stats_t air_stats;
            const VL53L1X_data_struct *tof_data;
            uint32 air_raw_rx_delta;
            uint32 air_overflow_delta;
            uint32 air_tx_overflow_delta;
            uint32 wifi_overflow_delta;
            uint32 tof_sample_seq_delta;
            uint32 log_interval_cycles;
            uint8 tof_valid_mask;

            wifi_justfloat_GetTxStats(&tx_before);
            air_comm_air_get_stats(&air_stats);
            tof_data = VL53L1X_GetData();
            air_raw_rx_delta = air_stats.raw_rx_byte_count - profile_air_raw_rx_last;
            air_overflow_delta = air_stats.rx_queue_overflow_count - profile_air_overflow_last;
            air_tx_overflow_delta = air_stats.tx_queue_overflow_count - profile_air_tx_overflow_last;
            wifi_overflow_delta = tx_before.overflow_count - profile_wifi_overflow_last;
            tof_sample_seq_delta = tof_data->sample_seq - profile_tof_sample_seq_last;
            tof_valid_mask = (uint8)((tof_data->valid[0] << 0U) |
                                     (tof_data->valid[1] << 1U) |
                                     (tof_data->valid[2] << 2U) |
                                     (tof_data->valid[3] << 3U));

            /* 固定42通道的100Hz性能日志，P27单位为毫秒，其余耗时通道单位均为微秒。 */
            profile_start_cycles = DWT->CYCCNT;
            log_interval_cycles = profile_start_cycles - profile_log_last_cycles;
            (void)wifi_justfloat(
                CORE0_PROFILE_TO_US(profile.imu_max_cycles, profile_cycles_per_us),       /* P01 IMU更新最大耗时 */
                CORE0_PROFILE_TO_US(profile.pos1_max_cycles, profile_cycles_per_us),      /* P02 位置估计1最大耗时 */
                CORE0_PROFILE_TO_US(profile.pos2_max_cycles, profile_cycles_per_us),      /* P03 位置估计2最大耗时 */
                CORE0_PROFILE_TO_US(profile.fc500_max_cycles, profile_cycles_per_us),     /* P04 500Hz控制最大耗时 */
                CORE0_PROFILE_TO_US(profile.fc1000_max_cycles, profile_cycles_per_us),    /* P05 1000Hz控制最大耗时，不含空地解析 */
                CORE0_PROFILE_TO_US(profile.loop1000_max_cycles, profile_cycles_per_us),  /* P06 完整1kHz任务最大耗时 */
                CORE0_PROFILE_TO_US(profile.tof_max_cycles, profile_cycles_per_us),       /* P07 ToF单步最大耗时 */
                CORE0_PROFILE_TO_US(profile.crsf_max_cycles, profile_cycles_per_us),      /* P08 CRSF接收更新最大耗时 */
                CORE0_PROFILE_TO_US(profile.fc100_max_cycles, profile_cycles_per_us),     /* P09 100Hz控制最大耗时 */
                CORE0_PROFILE_TO_US(profile.air_update_max_cycles, profile_cycles_per_us), /* P10 空地100Hz维护最大耗时 */
                CORE0_PROFILE_TO_US(profile.ipc_max_cycles, profile_cycles_per_us),       /* P11 IPC处理最大耗时 */
                CORE0_PROFILE_TO_US(profile.planner_max_cycles, profile_cycles_per_us),   /* P12 规划器最大耗时 */
                CORE0_PROFILE_TO_US(profile.air_send_max_cycles, profile_cycles_per_us),  /* P13 空地发送最大耗时 */
                CORE0_PROFILE_TO_US(profile.ipc_state_max_cycles, profile_cycles_per_us), /* P14 IPC飞行状态维护最大耗时 */
                CORE0_PROFILE_TO_US(profile.loop100_max_cycles, profile_cycles_per_us),   /* P15 完整100Hz任务最大耗时 */
                (float)profile.guard_max,                                                  /* P16 单轮1kHz补跑次数 */
                (float)profile.backlog_max,                                                /* P17 1kHz最大积压 */
                (float)g_tick_1000HZ,                                                      /* P18 发送时剩余1kHz积压 */
                (float)profile.task100_count,                                              /* P19 本窗口完整100Hz周期数 */
                (float)air_raw_rx_delta,                                                   /* P20 空地串口新增原始字节数 */
                (float)air_overflow_delta,                                                 /* P21 空地接收队列新增溢出次数 */
                (float)FC_START_CRSF_Get_State(),                                          /* P22 当前飞行状态 */
                CORE0_PROFILE_TO_US(profile.air_poll_max_cycles, profile_cycles_per_us),  /* P23 空地串口解析最大耗时 */
                ((profile.tail_update_mask & CORE0_PROFILE_CRSF10_UPDATED) != 0U)
                    ? CORE0_PROFILE_TO_US(profile.crsf10_max_cycles, profile_cycles_per_us) : 0.0f, /* P24 CRSF 10Hz发送耗时 */
                ((profile.tail_update_mask & CORE0_PROFILE_FC50_UPDATED) != 0U)
                    ? CORE0_PROFILE_TO_US(profile.fc50_max_cycles, profile_cycles_per_us) : 0.0f,   /* P25 飞控50Hz任务耗时 */
                ((profile.tail_update_mask & CORE0_PROFILE_STATE10_UPDATED) != 0U)
                    ? CORE0_PROFILE_TO_US(profile.state10_max_cycles, profile_cycles_per_us) : 0.0f, /* P26 起降10Hz状态机耗时 */
                CORE0_PROFILE_TO_US(log_interval_cycles, profile_cycles_per_us) / 1000.0f, /* P27 成功日志帧间隔，单位毫秒 */
                CORE0_PROFILE_TO_US(profile.height_max_cycles, profile_cycles_per_us),    /* P28 高度融合最大耗时 */
                (float)profile.tof_max_step,                                               /* P29 ToF最大耗时步骤编号 */
                (float)tof_sample_seq_delta,                                               /* P30 ToF快照序号增量 */
                (float)tof_data->fresh_mask,                                               /* P31 ToF最新快照新鲜通道掩码 */
                (float)tof_valid_mask,                                                     /* P32 四路ToF有效通道掩码 */
                (float)g_tof_fused_valid,                                                  /* P33 ToF融合高度有效标志 */
                g_height_meas_health,                                                      /* P34 高度测量健康度 */
                (float)air_stats.tx_pending_frames,                                        /* P35 AirComm当前待发帧数 */
                (float)air_tx_overflow_delta,                                              /* P36 AirComm发送队列新增溢出次数 */
                (float)profile.slow_tick_gap_max,                                          /* P37 慢任务相邻执行最大间隔，单位毫秒 */
                (float)profile.slow_slot_skip_count,                                       /* P38 慢任务跳过slot数量 */
                (float)profile.loop100_abort_count,                                        /* P39 未完整执行的100Hz周期数 */
                (float)profile.tail_update_mask,                                           /* P40 低频任务本帧更新掩码 */
                (float)tx_before.pending_bytes,                                            /* P41 WiFi待发送字节数 */
                (float)wifi_overflow_delta);                                               /* P42 WiFi队列新增溢出次数 */
            wifi_justfloat_GetTxStats(&tx_after);
            profile.send_pending = 0U;

            if (tx_after.queued_count != tx_before.queued_count)
            {
                profile.imu_max_cycles = 0U;
                profile.pos1_max_cycles = 0U;
                profile.pos2_max_cycles = 0U;
                profile.fc500_max_cycles = 0U;
                profile.fc1000_max_cycles = 0U;
                profile.air_poll_max_cycles = 0U;
                profile.loop1000_max_cycles = 0U;
                profile.tof_max_cycles = 0U;
                profile.height_max_cycles = 0U;
                profile.crsf_max_cycles = 0U;
                profile.fc100_max_cycles = 0U;
                profile.air_update_max_cycles = 0U;
                profile.ipc_max_cycles = 0U;
                profile.planner_max_cycles = 0U;
                profile.air_send_max_cycles = 0U;
                profile.ipc_state_max_cycles = 0U;
                profile.crsf10_max_cycles = 0U;
                profile.fc50_max_cycles = 0U;
                profile.state10_max_cycles = 0U;
                profile.loop100_max_cycles = 0U;
                profile.task100_count = 0U;
                profile.slow_tick_gap_max = 0U;
                profile.slow_slot_skip_count = 0U;
                profile.loop100_abort_count = 0U;
                profile.backlog_max = 0U;
                profile.guard_max = 0U;
                profile.tof_max_step = 0U;
                profile.tail_update_mask = 0U;
                profile_air_raw_rx_last = air_stats.raw_rx_byte_count;
                profile_air_overflow_last = air_stats.rx_queue_overflow_count;
                profile_air_tx_overflow_last = air_stats.tx_queue_overflow_count;
                profile_wifi_overflow_last = tx_before.overflow_count;
                profile_tof_sample_seq_last = tof_data->sample_seq;
                profile_log_last_cycles = profile_start_cycles;
            }
        }
    }
}
