#include "zf_common_headfile.h"
#include "../code/FlightController/fc_mode.h"
#include "../code/Image/image_data.h"
#include "../code/Planner/beacon_lost_detector.h"
#include "../code/Planner/car_lamp_fused.h"
#include "../code/Planner/car_plan_2.h"
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
float g_car_yaw_target_deg = 0.0f; /* 车端yaw控制目标角(经SPI data[2])，单位deg */
float g_car_large_turn_state = 0.0f; /* 车端大角度状态机(经SPI data[7]): 0正常 1刹车 2原地转 3退出 */
static car_plan_result_t s_air_run_car_plan;
static uint8 s_air_run_car_plan_valid = 0U;

#define MODE12_WIFI_DEBUG_PERIOD_MS      (5U)

typedef struct
{
    float data[42];
} CarPlanWifiJustFloatPacket;

_Static_assert(sizeof(CarPlanWifiJustFloatPacket) ==
                   (42U * sizeof(float)),
               "JustFloat packet layout error");

#if 0
static uint32 mode12_debug_state_pack(void)
{
    uint32 state = (uint32)FC_START_CRSF_Get_Flight_Mode() |
                   ((uint32)FC_START_CRSF_Get_State() << 4U);
    uint32 lamp_mask = 0U;

    if (image_data_car_lamp_valid(&image_data[Front].car_lamp_data[0]) != 0U)
    {
        lamp_mask |= 1U;
    }
    if (image_data_car_lamp_valid(&image_data[Center].car_lamp_data[0]) != 0U)
    {
        lamp_mask |= 2U;
    }
    if (image_data_car_lamp_valid(&image_data[Back].car_lamp_data[0]) != 0U)
    {
        lamp_mask |= 4U;
    }
    if (g_car_lamp_fused.valid != 0U)
    {
        state |= 1UL << 7U;
    }
    if (g_tof_fused_valid != 0U)
    {
        state |= 1UL << 8U;
    }
    if ((g_car_sync_time_ms > 0.0f) &&
        ((tick_1000us_cnt - g_car_last_update_time_ms) < FC_MODE_CAR_RUN_DATA_TIMEOUT_MS))
    {
        state |= 1UL << 9U;
    }

    return state | (lamp_mask << 10U);
}

static uint32 mode12_debug_yaw_pack(const yaw_align_debug_t *debug)
{
    uint32 camera = (debug->active_beacon.valid != 0U)
                        ? (uint32)debug->active_beacon.camera
                        : 3U;

    return ((uint32)debug->action & 0x7U) |
           ((camera & 0x3U) << 3U) |
           (((uint32)debug->locked & 0x1U) << 5U) |
           (((uint32)debug->candidate_frames & 0x1FU) << 6U) |
           (((uint32)debug->lost_frames & 0x1FU) << 11U);
}

static void mode12_wifi_debug_legacy_200hz(void)
{
    static uint32 last_tick_ms = 0U;
    FC_START_CRSF_flight_mode_e mode = FC_START_CRSF_Get_Flight_Mode();
    yaw_align_debug_t yaw_debug;
    car_plan_2_result_t plan2_dbg;
    float cmd_ang_deg = 0.0f;
    float dir_cmd_deg = 0.0f;
    uint32 tick_now = tick_1000us_cnt;

    if ((FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING) ||
        ((mode != FC_START_CRSF_FLIGHT_MODE_1) &&
         (mode != FC_START_CRSF_FLIGHT_MODE_2)))
    {
        last_tick_ms = tick_now;
        return;
    }
    if ((tick_now - last_tick_ms) < MODE12_WIFI_DEBUG_PERIOD_MS)
    {
        return;
    }
    last_tick_ms = tick_now;
    YawAlign_GetDebug(&yaw_debug);
    CarPlan_2_GetResult(&plan2_dbg);
    if ((s_air_run_car_plan_valid != 0U) &&
        ((fabsf(s_air_run_car_plan.target_strafe_mps) > 0.01f) ||
         (fabsf(s_air_run_car_plan.target_forward_mps) > 0.01f)))
    {
        /* 车端控制律: yaw目标 = 车yaw + atan2(strafe, forward)，飞机可据此预判车旋转方向 */
        cmd_ang_deg = atan2f(s_air_run_car_plan.target_strafe_mps,
                             s_air_run_car_plan.target_forward_mps) * 57.29577951f;
        dir_cmd_deg = g_car_yaw + cmd_ang_deg;
        while (dir_cmd_deg > 180.0f)
        {
            dir_cmd_deg -= 360.0f;
        }
        while (dir_cmd_deg < -180.0f)
        {
            dir_cmd_deg += 360.0f;
        }
    }

    (void)wifi_justfloat(
        (float)MODE12_WIFI_RECORD_DYNAMIC,                  /* I1 记录类型 */
        (float)mode12_debug_state_pack(),                   /* I2 状态位 */
        (float)g_mode2_control_seq,                         /* I3 100Hz控制序号 */
        (float)g_image_data_seq,                            /* I4 图像序号 */
        g_car_sync_time_ms,                                 /* I5 车端时间戳 */
        g_euler.roll, g_euler.pitch, g_euler.yaw,           /* I6-I8 实际欧拉角 */
        roll_angle_target, pitch_angle_target, yaw_angle_target, /* I9-I11 目标欧拉角 */
        g_tof_fused_height_mm,                              /* I12 融合高度 */
        g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy,
        g_imufilter_1000hz.gyroz,                           /* I13-I15 实际角速度 */
        roll_gyro_target, pitch_gyro_target, yaw_gyro_target, /* I16-I18 目标角速度 */
        (float)g_motor_cmd.roll, (float)g_motor_cmd.pitch,
        (float)g_motor_cmd.yaw,                             /* I19-I21 电机控制量 */
        g_car_lamp_fused.cx, g_car_lamp_fused.cy,
        g_car_lamp_fused.angle,                             /* I22-I24 融合车灯坐标及角度 */
        g_mode2_imgx_pid.error, g_mode2_imgy_pid.error,     /* I25-I26 图像误差 */
        g_mode2_img_error_rate_x_pxps,
        g_mode2_img_error_rate_y_pxps,                      /* I27-I28 滤波误差变化率 */
        g_car_vel_x, g_car_vel_y,                           /* I29-I30 实际车速 */
        g_car_vel_target_x, g_car_vel_target_y,             /* I31-I32 目标车速 */
        g_mode2_car_vel_error_x_mps,
        g_mode2_car_vel_error_y_mps,                        /* I33-I34 滤波车速误差 */
        g_mode2_car_body_accel_x_mps2,
        g_mode2_car_body_accel_y_mps2,                      /* I35-I36 旋转前车体加速度 */
        g_mode2_car_turn_accel_mps2,                        /* I37 滤波向心加速度 */
        g_car_yaw, g_car_yaw_rate_dps,                      /* I38-I39 车Yaw及角速度 */
        g_mode2_yaw_diff_deg,                               /* I40 控制航向差 */
        g_mode2_raw_roll_correction_deg,
        g_mode2_raw_pitch_correction_deg,                   /* I41-I42 限幅前修正角 */
        yaw_debug.active_beacon.x, yaw_debug.active_beacon.y,
        yaw_debug.active_beacon.area,                       /* I43-I45 Yaw使用信标 */
        yaw_debug.yaw_delta_deg,                            /* I46 Yaw目标增量 */
        (float)mode12_debug_yaw_pack(&yaw_debug),           /* I47 Yaw状态 */
        (float)s_air_run_car_plan_valid,                    /* I48 实际下发plan有效 */
        s_air_run_car_plan.target_strafe_mps,               /* I49 实际下发横移速度m/s */
        s_air_run_car_plan.target_forward_mps,              /* I50 实际下发前进速度m/s */
        (float)plan2_dbg.camera_mask,                       /* I51 plan2目标相机mask 1前2中4后 */
        plan2_dbg.target_center_x,                          /* I52 plan2目标信标x px */
        plan2_dbg.target_center_y,                          /* I53 plan2目标信标y px */
        (float)plan2_dbg.valid,                             /* I54 plan2结果有效 */
        cmd_ang_deg,                                        /* I55 指令方向角(车身系)deg */
        dir_cmd_deg,                                        /* I56 预测车航向(世界系)deg */
        g_car_yaw_target_deg,                               /* I57 车端yaw目标(经SPI)deg */
        g_car_large_turn_state);                            /* I58 车端大角度状态 0正常1刹车2原地转3退出 */
}
#endif

static void mode12_wifi_debug_200hz(void)
{
    static uint32 last_tick_ms = 0U;
    FC_START_CRSF_flight_mode_e mode = FC_START_CRSF_Get_Flight_Mode();
    uint32 tick_now = tick_1000us_cnt;

    if ((FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING) ||
        ((mode != FC_START_CRSF_FLIGHT_MODE_1) &&
         (mode != FC_START_CRSF_FLIGHT_MODE_2)))
    {
        last_tick_ms = tick_now;
        return;
    }
    if ((tick_now - last_tick_ms) < MODE12_WIFI_DEBUG_PERIOD_MS)
    {
        return;
    }
    last_tick_ms = tick_now;

    {
        CarPlanWifiJustFloatPacket packet;
        float *data = packet.data;
        uint8 camera;
        uint8 beacon;
        uint8 index = 0U;

        for (camera = 0U; camera < (uint8)IMAGE_CAMERA_COUNT; camera++)
        {
            for (beacon = 0U; beacon < 2U; beacon++)
            {
                const beacon_data *item = &image_data[camera].beacon_data[beacon];
                if ((item->valid != 0U) && (item->x > -900.0f) &&
                    (item->y > -900.0f) && (item->area > 0.0f))
                {
                    data[index++] = item->x;
                    data[index++] = item->y;
                    data[index++] = item->area;
                }
                else
                {
                    data[index++] = -999.0f;
                    data[index++] = -999.0f;
                    data[index++] = 0.0f;
                }
            }
        }

        for (camera = 0U; camera < (uint8)IMAGE_CAMERA_COUNT; camera++)
        {
            const car_lamp_data *item = &image_data[camera].car_lamp_data[0];
            if ((item->valid != 0U) && (item->cx > -900.0f) &&
                (item->cy > -900.0f) && (item->angle > -900.0f) &&
                (item->width > 0.0f) && (item->length > 0.0f))
            {
                data[index++] = item->cx;
                data[index++] = item->cy;
                data[index++] = item->angle;
                data[index++] = item->width;
                data[index++] = item->length;
            }
            else
            {
                data[index++] = -999.0f;
                data[index++] = -999.0f;
                data[index++] = -999.0f;
                data[index++] = 0.0f;
                data[index++] = 0.0f;
            }
        }

        data[index++] = g_euler.pitch;
        data[index++] = g_euler.roll;
        data[index++] = g_euler.yaw;
        data[index++] = (float)tick_now;
        data[index++] = 0.0f;
        data[index++] = g_car_vel_y;
        data[index++] = g_car_yaw;
        data[index++] = (s_air_run_car_plan_valid != 0U) ?
                            s_air_run_car_plan.target_forward_mps : 0.0f;
        data[index++] = (s_air_run_car_plan_valid != 0U) ?
                            s_air_run_car_plan.target_strafe_mps : 0.0f;

        (void)wifi_justfloat(
            data[0], data[1], data[2], data[3], data[4], data[5],
            data[6], data[7], data[8], data[9], data[10], data[11],
            data[12], data[13], data[14], data[15], data[16], data[17],
            data[18], data[19], data[20], data[21], data[22], data[23],
            data[24], data[25], data[26], data[27], data[28], data[29],
            data[30], data[31], data[32], data[33], data[34], data[35],
            data[36], data[37], data[38], data[39], data[40], data[41]);
    }
}

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
        g_car_yaw_target_deg = data[2];
        g_car_yaw = data[3];
        g_car_yaw_rate_dps = data[4];
        g_car_vel_target_x = data[5];
        g_car_vel_target_y = data[6];
        g_car_large_turn_state = data[7];
        /* 仅在车端时间戳推进时刷新数据新鲜时刻。 */
        if (data[10] != g_car_sync_time_ms)
        {
            g_car_sync_time_ms = data[10];
            g_car_last_update_time_ms = tick_1000us_cnt;
        }
    }
}

/**
 * @brief 按飞机状态以200Hz发送缓存的规划结果或完整诊断数据。
 * @param 无。
 * @return 无。
 */
static void send_air_run_data_200hz(void)
{
    FC_START_CRSF_state_e state = FC_START_CRSF_Get_State();
    const car_plan_result_t *car_plan = &s_air_run_car_plan;
    uint8 car_plan_send_valid = s_air_run_car_plan_valid;
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
    mode12_wifi_debug_200hz();
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
 * @brief 以100Hz更新信标与车模规划，并缓存供200Hz发送任务使用。
 * @param 无。
 * @return 无。
 */
static void core0_plan_update_100hz(void)
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
    s_air_run_car_plan = car_plan;
    s_air_run_car_plan_valid = car_plan_send_valid;

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

    case 0U:
        air_comm_air_update_200HZ();
        send_air_run_data_200hz();
        break;

    case 3U:
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
        core0_plan_update_100hz();
        air_comm_air_update_200HZ();
        send_air_run_data_200hz();
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
