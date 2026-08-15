#include "zf_common_headfile.h"
#include "../code/FlightController/fc_mode.h"
#include "../code/Image/image_data.h"
#include "../code/Planner/beacon_lost_detector.h"
#include "../code/Planner/car_plan_3.h"
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

#define CAR_PLAN_DEBUG_PERIOD_MS          (5U)  /* 规划调试JustFloat发送周期，单位ms。 */
#define CAR_PLAN_DEBUG_FLOAT_COUNT        (63U) /* CarPlan3 V2调试协议用户float数量。 */
#define CAR_PLAN_DEBUG_PROTOCOL_VERSION   (3.0f) /* CarPlan3全局坐标调试协议版本。 */
#define MODE12_DEBUG_FLOAT_COUNT          (48U) /* Mode1/2跟车控制调试数据数量。 */

typedef struct
{
    float data[CAR_PLAN_DEBUG_FLOAT_COUNT];
} CarPlanWifiJustFloatPacket;

_Static_assert(sizeof(CarPlanWifiJustFloatPacket) ==
                   (CAR_PLAN_DEBUG_FLOAT_COUNT * sizeof(float)),
               "JustFloat packet layout error");

/**
 * @brief 以200Hz组装并发送车模规划调试JustFloat数据。
 * @param 无。
 * @return 无。
 */
static void car_plan_debug_200hz(void)
{
    static uint32 last_tick_ms = 0U;
    uint32 tick_now = tick_1000us_cnt;
    CarPlanWifiJustFloatPacket packet;
    car_plan_3_debug_t plan_debug;
    car_plan_result_t plan_result;
    float *data = packet.data;
    uint8 camera;
    uint8 beacon;
    uint8 index = 0U;

    if ((tick_now - last_tick_ms) < CAR_PLAN_DEBUG_PERIOD_MS)
    {
        return;
    }
    last_tick_ms = tick_now;
    CarPlan_3_GetDebug(&plan_debug);
    CarPlan_3_GetResult(&plan_result);

    /* I1-I30: Front/Center/Back，各两个信标(x,y,area)和一个车灯(x,y,angle,length)。 */
    for (camera = 0U; camera < (uint8)IMAGE_CAMERA_COUNT; camera++)
    {
        for (beacon = 0U; beacon < 2U; beacon++)
        {
            const beacon_data *item = &image_data[camera].beacon_data[beacon];
            if (image_data_beacon_valid(item) != 0U)
            {
                data[index++] = item->x;
                data[index++] = item->y;
                data[index++] = item->area;
            }
            else
            {
                data[index++] = IMAGE_DATA_INVALID_VALUE;
                data[index++] = IMAGE_DATA_INVALID_VALUE;
                data[index++] = 0.0f;
            }
        }

        {
            const car_lamp_data *item = &image_data[camera].car_lamp_data[0];
            if ((image_data_car_lamp_valid(item) != 0U) && (item->length > 0.0f))
            {
                data[index++] = item->cx;
                data[index++] = item->cy;
                data[index++] = item->angle;
                data[index++] = item->length;
            }
            else
            {
                data[index++] = IMAGE_DATA_INVALID_VALUE;
                data[index++] = IMAGE_DATA_INVALID_VALUE;
                data[index++] = IMAGE_DATA_INVALID_VALUE;
                data[index++] = 0.0f;
            }
        }
    }

    /* I31-I46: 四个CarPlan3全局融合信标(x_m,y_m,area,camera_mask)。 */
    for (beacon = 0U; beacon < CAR_PLAN_3_DEBUG_BEACON_COUNT; beacon++)
    {
        const car_plan_3_debug_beacon_t *item = &plan_debug.beacon[beacon];
        data[index++] = item->center_x;
        data[index++] = item->center_y;
        data[index++] = item->area;
        data[index++] = (float)item->camera_mask;
    }
    /* I47-I50: CarPlan3全局融合车灯(x_m,y_m,angle,camera_mask)。 */
    if (plan_debug.car_lamp.valid != 0U)
    {
        data[index++] = plan_debug.car_lamp.center_x;
        data[index++] = plan_debug.car_lamp.center_y;
        data[index++] = plan_debug.car_lamp.angle_deg;
        data[index++] = (float)plan_debug.car_lamp.camera_mask;
    }
    else
    {
        data[index++] = IMAGE_DATA_INVALID_VALUE;
        data[index++] = IMAGE_DATA_INVALID_VALUE;
        data[index++] = IMAGE_DATA_INVALID_VALUE;
        data[index++] = 0.0f;
    }
    /* I51-I63: CarPlan3有效状态、车状态、飞机状态、选中槽位、标记和协议版本。 */
    data[index++] = (plan_result.valid != 0U) ? 1.0f : 0.0f;
    data[index++] = g_car_yaw;
    data[index++] = g_car_vel_x;
    data[index++] = g_car_vel_y;
    data[index++] = (plan_result.valid != 0U) ? plan_result.target_strafe_mps : 0.0f;
    data[index++] = (plan_result.valid != 0U) ? plan_result.target_forward_mps : 0.0f;
    data[index++] = g_tof_fused_height_mm;
    data[index++] = g_euler.roll;
    data[index++] = g_euler.pitch;
    data[index++] = g_euler.yaw;
    data[index++] = (float)plan_debug.selected_target_id;
    data[index++] = (CRSF_STD[8] > 0) ? 1.0f : 0.0f;
    data[index++] = CAR_PLAN_DEBUG_PROTOCOL_VERSION;

    (void)wifi_justfloat_Array(data, CAR_PLAN_DEBUG_FLOAT_COUNT);
}

/**
 * @brief 以200Hz发送当前Mode1/2无人机跟车控制调试数据。
 * @param 无。
 * @return 无。
 */
static void mode12_wifi_debug_200hz(void)
{
    static uint32 last_tick_ms = 0U;
    FC_START_CRSF_flight_mode_e mode = FC_START_CRSF_Get_Flight_Mode();
    uint32 tick_now = tick_1000us_cnt;
    float data[MODE12_DEBUG_FLOAT_COUNT];

    if ((FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING) ||
        ((mode != FC_START_CRSF_FLIGHT_MODE_1) &&
         (mode != FC_START_CRSF_FLIGHT_MODE_2)))
    {
        last_tick_ms = tick_now;
        return;
    }
    if ((tick_now - last_tick_ms) < CAR_PLAN_DEBUG_PERIOD_MS)
    {
        return;
    }
    last_tick_ms = tick_now;

    data[0] = (float)mode;                         /* I1 飞行模式。 */
    data[1] = (float)((mode == FC_START_CRSF_FLIGHT_MODE_1) ?
                          g_mode1_control_seq : g_mode2_control_seq); /* I2 100Hz控制序号。 */
    data[2] = (float)tick_now;                     /* I3 飞机时间，ms。 */
    data[3] = g_car_sync_time_ms;                  /* I4 车端时间，ms。 */
    data[4] = g_car_vel_x;                         /* I5 车实际横移速度，m/s。 */
    data[5] = g_car_vel_y;                         /* I6 车实际前进速度，m/s。 */
    data[6] = g_car_yaw;                           /* I7 车Yaw，deg。 */
    data[7] = g_car_yaw_rate_dps;                  /* I8 车Yaw角速度，deg/s。 */
    data[8] = g_car_vel_target_x;                  /* I9 车目标横移速度，m/s。 */
    data[9] = g_car_vel_target_y;                  /* I10 车目标前进速度，m/s。 */
    data[10] = g_euler.roll;                       /* I11 飞机实际Roll，deg。 */
    data[11] = g_euler.pitch;                      /* I12 飞机实际Pitch，deg。 */
    data[12] = g_euler.yaw;                        /* I13 飞机实际Yaw，deg。 */
    data[13] = roll_angle_target;                  /* I14 飞机目标Roll，deg。 */
    data[14] = pitch_angle_target;                 /* I15 飞机目标Pitch，deg。 */
    data[15] = yaw_angle_target;                   /* I16 飞机目标Yaw，deg。 */
    data[16] = g_imufilter_1000hz.gyrox;           /* I17 实际Roll角速度，deg/s。 */
    data[17] = g_imufilter_1000hz.gyroy;           /* I18 实际Pitch角速度，deg/s。 */
    data[18] = g_imufilter_1000hz.gyroz;           /* I19 实际Yaw角速度，deg/s。 */
    data[19] = roll_gyro_target;                   /* I20 目标Roll角速度，deg/s。 */
    data[20] = pitch_gyro_target;                  /* I21 目标Pitch角速度，deg/s。 */
    data[21] = yaw_gyro_target;                    /* I22 目标Yaw角速度，deg/s。 */
    data[30] = g_car_vel_target_x - g_car_vel_x;   /* I31 车X速度误差滤波前，m/s。 */
    data[31] = g_car_vel_target_y - g_car_vel_y;   /* I32 车Y速度误差滤波前，m/s。 */
    data[38] = g_car_vel_y * g_car_yaw_rate_dps * 0.017453292519943295f; /* I39 向心加速度滤波前，m/s^2。 */

    if (mode == FC_START_CRSF_FLIGHT_MODE_1)
    {
        data[22] = g_mode1_imgx_pid.error; data[23] = g_mode1_imgy_pid.error;
        data[24] = g_mode1_imgx_pid.p_term; data[25] = g_mode1_imgy_pid.p_term;
        data[26] = g_mode1_imgx_pid.d_term; data[27] = g_mode1_imgy_pid.d_term;
        data[28] = g_mode1_img_error_rate_x_pxps; data[29] = g_mode1_img_error_rate_y_pxps;
        data[32] = g_mode1_car_vel_error_x_mps; data[33] = g_mode1_car_vel_error_y_mps;
        data[34] = g_mode1_car_body_accel_x_mps2; data[35] = g_mode1_car_body_accel_y_mps2;
        data[36] = g_mode1_car_accel_x_mps2; data[37] = g_mode1_car_accel_y_mps2;
        data[39] = g_mode1_car_turn_accel_mps2;
        data[40] = g_mode1_car_accel_angle_ff_x_deg; data[41] = g_mode1_car_accel_angle_ff_y_deg;
        data[42] = g_mode1_imgx_pid.output; data[43] = g_mode1_imgy_pid.output;
        data[44] = g_mode1_raw_roll_correction_deg; data[45] = g_mode1_raw_pitch_correction_deg;
        data[46] = g_mode1_yaw_diff_deg; data[47] = g_mode1_car_dt_ms;
    }
    else
    {
        data[22] = g_mode2_imgx_pid.error; data[23] = g_mode2_imgy_pid.error;
        data[24] = g_mode2_imgx_pid.p_term; data[25] = g_mode2_imgy_pid.p_term;
        data[26] = g_mode2_imgx_pid.d_term; data[27] = g_mode2_imgy_pid.d_term;
        data[28] = g_mode2_img_error_rate_x_pxps; data[29] = g_mode2_img_error_rate_y_pxps;
        data[32] = g_mode2_car_vel_error_x_mps; data[33] = g_mode2_car_vel_error_y_mps;
        data[34] = g_mode2_car_body_accel_x_mps2; data[35] = g_mode2_car_body_accel_y_mps2;
        data[36] = g_mode2_car_accel_x_mps2; data[37] = g_mode2_car_accel_y_mps2;
        data[39] = g_mode2_car_turn_accel_mps2;
        data[40] = g_mode2_car_accel_angle_ff_x_deg; data[41] = g_mode2_car_accel_angle_ff_y_deg;
        data[42] = g_mode2_imgx_pid.output; data[43] = g_mode2_imgy_pid.output;
        data[44] = g_mode2_raw_roll_correction_deg; data[45] = g_mode2_raw_pitch_correction_deg;
        data[46] = g_mode2_yaw_diff_deg; data[47] = g_mode2_car_dt_ms;
    }

    (void)wifi_justfloat_Array(data, MODE12_DEBUG_FLOAT_COUNT);
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
    uint8 car_plan_send_valid = s_air_run_car_plan.valid;
    float plan_strafe_mps = (car_plan_send_valid != 0U) ? s_air_run_car_plan.target_strafe_mps : 0.0f;
    float plan_forward_mps = (car_plan_send_valid != 0U) ? s_air_run_car_plan.target_forward_mps : 0.0f;
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
        air_data[20] = 0.0f;
        air_data[21] = 0.0f;
        air_data[22] = 0.0f;
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
    // car_plan_debug_200hz(); /* 临时关闭CarPlan3调试，改发Mode1/2跟车控制数据。 */
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
    (void)BeaconLostDetector_Update();
    CarPlanEntry_Update(&s_air_run_car_plan);
    if(g_tof_fused_height_mm <= 500.0f)
    {
        s_air_run_car_plan.valid = 0U;
    }
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
