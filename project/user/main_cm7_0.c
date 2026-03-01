/*********************************************************************************************************************
 * CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 CYT4BB 开源库的一部分
 *
 * CYT4BB 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          main_cm7_0
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 9.40.1
 * 适用平台          CYT4BB
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2024-1-4       pudding            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
volatile uint32 tick_500us_cnt = 0U;
volatile uint16 g_tick_2000HZ = 0U;
volatile uint8 g_tick_100HZ = 0U;
static uint8 s_tick_div_pos_250hz = 0U;
static uint8 s_tick_div_fc_start_10hz = 0U;


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
    debug_init();                  // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等

    Height_Est_Init(); // 高度估计初始化（TOF+Baro）
    PMW3901_Init();    // PMW3901 光流传感器初始化
    IMU_Init_All();    // ICM42688 IMU 初始化
    Pos_Est_Init();    // 位置估计初始化
    crsf_init();       // CRSF 遥控协议初始化
    AccelCalibration_Init();   // 加速度标定模块初始化
    IMUCalib_Init();           // 读取Flash中的IMU校准参数并应用
    FC_Params_Init();          // 飞控参数初始化
    FC_Loop_Init();            // 飞控主循环相关资源初始化
    Motor_Init();              // 电机驱动初始化
    FC_START_CRSF_Init();     // 起飞流程状态机初始化
    pit_us_init(PIT_CH0, 500); // PIT 定时器初始化 500us 中断周期
    pit_ms_init(PIT_CH1, 10);  // 100Hz 节拍
 

    while (true)
    {
        uint8 run_100hz = 0U;
        uint16 tick_2000_guard = 0U;

        while ((g_tick_2000HZ > 0U) && (tick_2000_guard < 200U))
        {
            g_tick_2000HZ--;
            IMU_Update_2000HZ();
            AccelCalibration_Update_2000HZ();
            IMUCalib_Update_2000HZ();
            FC_Loop_2000Hz();
            s_tick_div_pos_250hz++;
            if (s_tick_div_pos_250hz >= 8U)
            {
                s_tick_div_pos_250hz = 0U;
                Pos_Est_Update_250HZ();
            }

            tick_2000_guard++;
        }

        if (g_tick_100HZ > 0U)
        {
            g_tick_100HZ--;
            Height_Est_Update_100HZ();
            Pos_Est_Update_100HZ();
            PMW3901_Update();
            crsf_send_25hz(); // CRSF 25Hz发送函数（100Hz调用一次）
            CRSF_Update_100HZ();
            s_tick_div_fc_start_10hz++;
            if (s_tick_div_fc_start_10hz >= 10U)
            {
                s_tick_div_fc_start_10hz = 0U;
                FC_START_CRSF_Update();
            }
            run_100hz = 1U;
        }

        IMUCalib_CommandPoll();

        if ((run_100hz != 0U))
        {
            static uint8 s_pos_print_div = 0U;
            s_pos_print_div++;
            if (s_pos_print_div >= 2U)
            {
                s_pos_print_div = 0U;
                FC_START_CRSF_state_e state = FC_START_CRSF_Get_State();
                uint8 prop_spinning = ((state == FC_START_CRSF_STATE_TAKEOFF) ||
                                       (state == FC_START_CRSF_STATE_FLYING)) ? 1U : 0U;
                // ch0~ch15: tick500us,state,prop_spin,raw_p_adc,raw_t_adc,press_raw_pa,press_lpf_pa,ref_pa,alt_raw_m,alt_lpf_m,tof_fused_mm,tof_fused_valid,height_m,height_valid,height_src,vz_mps
                // printf("%lu,%u,%u,%lu,%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%u,%u,%.4f,%u,%u,%.4f\r\n",
                //        (unsigned long)tick_500us_cnt,
                //        (unsigned int)state,
                //        (unsigned int)prop_spinning,
                //        (unsigned long)g_BMP388_data.raw_pressure,
                //        (unsigned long)g_BMP388_data.raw_temperature,
                //        g_baro_pressure_raw_pa,
                //        g_baro_pressure_filt_pa,
                //        g_baro_ref_pressure,
                //        g_baro_altitude_raw_m,
                //        g_baro_altitude,
                //        (unsigned int)g_tof_fused_height_mm,
                //        (unsigned int)g_tof_fused_valid,
                //        g_height_est_m,
                //        (unsigned int)g_height_est_valid,
                //        (unsigned int)g_height_est_source,
                //        g_height_vz_mps);

                printf("%d,%d\r\n",g_pmw3901_raw.deltaX,g_pmw3901_raw.deltaY);
            }
        }
    }
}

// **************************** 代码区域 ****************************


