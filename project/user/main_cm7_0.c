/*********************************************************************************************************************
 * 文件名称          main_cm7_0
 * 说明              CM7_0 主程序入口，负责系统初始化与多频率任务调度
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "Protocols/wifi/wifi_cmd/wifi_cmd.h"
#include "Protocols/wifi/wifi_params/wifi_params.h"
#include "Protocols/wifi/wifi_cal_imu/wifi_cal_imu.h"
#include "Protocols/wifi/wifi_justfloat/wifi_justfloat.h"

/* 1kHz 基准节拍计数器，单位 us tick */
volatile uint32 tick_1000us_cnt = 0U;
/* 1kHz 主循环待处理节拍数 */
volatile uint16 g_tick_1000HZ = 0U;
/* 100Hz 主循环待处理节拍数 */
volatile uint8 g_tick_100HZ = 0U;

/* 250Hz 位置估计分频计数器 */
static uint8 s_tick_div_pos_250hz = 0U;
/* 500Hz 飞控分频计数器 */
static uint8 s_tick_div_fc_loop_500hz = 0U;
/* 10Hz 启动状态机分频计数器 */
static uint8 s_tick_div_fc_start_10hz = 0U;
/* 50Hz 位置估计与高度环分频计数器 */
static uint8 s_tick_div_fc_start_50hz = 0U;
static uint8 s_tick_div_fc_start_1hz = 0U;

int main(void)
{
    wifi_params_diag_t wifi_params_diag = {0}; /* WiFi 参数调节诊断信息，用于 VOFA 回传最近一次文本命令处理状态 */

    /* ===== 系统与外设初始化 ===== */
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    Beep_Init();
    pit_ms_init(PIT_CH2, 10);
    wifi_cmd_Init();
    TOF_Init();
    PMW3901_Init();
    IMU_Init_All();
    crsf_init();
    AccelCalibration_Init();
    IMUCalib_Init();
    FC_Params_Init();
    (void)FC_Params_LoadFromFlash(); /* 优先加载掉电保存参数，无有效数据时保持默认值 */
    Pos_Est_Init();                  /* 确保参数装载后重新清理位置估计状态 */
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
        uint16 tick_1000_guard = 0U; /* 单次 while 内 1kHz 任务最多处理帧数，防止异常积压 */

        /* ===== 1kHz 高频任务 ===== */
        while ((g_tick_1000HZ > 0U) && (tick_1000_guard < 100U))
        {
            g_tick_1000HZ--;

            IMU_Update_1000HZ();
            Pos_Est_Update_1000HZ();
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
                /* Pos_Est_Update_250HZ(); */
            }

            tick_1000_guard++;
        }

        /* ===== 100Hz 低频任务 ===== */
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
                FC_Loop_50Hz();

                wifi_params_GetDiag(&wifi_params_diag);
            }

            s_tick_div_fc_start_1hz++;
            if (s_tick_div_fc_start_1hz >= 2U)
            {
                s_tick_div_fc_start_1hz = 0U;
                /* 1Hz 定时保存参数到 Flash，确保掉电保存 */
            }

            s_tick_div_fc_start_10hz++;
            if (s_tick_div_fc_start_10hz >= 10U)
            {
                s_tick_div_fc_start_10hz = 0U;
                FC_START_CRSF_Update();

                /* 姿态超限保护：roll 或 pitch 超过 ±55 度时触发紧急停桨 */
                if (g_euler.roll > 55.0f || g_euler.roll < -55.0f ||
                    g_euler.pitch > 55.0f || g_euler.pitch < -55.0f)
                {
                    FC_START_CRSF_Trigger_Emergency_Stop();
                }
            }
        }

        /* ===== 后台任务 ===== */
        wifi_cmd_Poll();
    }
}
