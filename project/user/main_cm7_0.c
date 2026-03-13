/*********************************************************************************************************************
 * CYT4BB 开源库（CYT4BB Open Source Library）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 CYT4BB 开源库的一部分
 *
 * CYT4BB 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，GNU 通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅 <https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议，禁止添加附加其为闭源软件的条款
 * 详细内容英文版参见 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件
 * 许可证副本参见 libraries 文件夹下 该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序，修改内容时请保留逐飞科技的版权声明及以下声明
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

// 如果工程中的工具或工程名中存在非法字符则会导致编译失败
// 第一步 关闭所有打开的文件
// 第二步 project->clean 等待清理完成重新编译即可

// 以下为开源扩展示例，开发者移植或者测试可以参考以下内容
// 以下为开源扩展示例，开发者移植或者测试可以参考以下内容
// 以下为开源扩展示例，开发者移植或者测试可以参考以下内容

/* ======================== 全局变量定义 ======================== */
volatile uint32 tick_1000us_cnt = 0U;       /* 1kHz 基准节拍计数器 */
volatile uint16 g_tick_1000HZ = 0U;         /* 1kHz 主循环待处理节拍数 */
volatile uint8  g_tick_100HZ  = 0U;         /* 100Hz 主循环待处理节拍数 */
static uint8 s_tick_div_pos_250hz     = 0U; /* 250Hz 位置估计分频计数器 */
static uint8 s_tick_div_fc_loop_500hz = 0U; /* 500Hz 飞控循环分频计数器 */
static uint8 s_tick_div_fc_start_10hz = 0U; /* 10Hz 飞控启动状态机分频计数器 */
static uint8 s_tick_div_fc_start_50hz = 0U; /* 50Hz 位置估计与飞控循环分频计数器 */

int main(void)
{
    /* ===== 系统与外设初始化 ===== */
    clock_init(SYSTEM_CLOCK_250M); /* 时钟配置及系统初始化（最优先执行） */
    debug_init();                  /* 调试串口信息初始化 */

    Beep_Init();                   /* 蜂鸣器初始化 */
    pit_ms_init(PIT_CH2, 10);      /* 10ms 周期 PIT 定时器，用于蜂鸣器节拍驱动 */
    wifi_vofa_Init();              /* WiFi VOFA 协议模块初始化 */
    TOF_Init();                    /* TOF 激光测距模块初始化 */
    PMW3901_Init();                /* PMW3901 光流模块初始化 */
    IMU_Init_All();                /* ICM42688 IMU 初始化（含自检与暖机） */
    Pos_Est_Init();                /* 位置估计器初始化 */
    crsf_init();                   /* CRSF 遥控协议初始化 */
    AccelCalibration_Init();       /* 加速度计标定模块初始化 */
    IMUCalib_Init();               /* 从 Flash 读取 IMU 校准参数并应用 */
    FC_Params_Init();              /* 飞控参数初始化 */
    Pos_Est_Init();                /* 位置估计模块二次初始化（确保参数加载后重置状态） */
    FC_Loop_Init();                /* 飞控主循环资源初始化 */
    Motor_Init();                  /* 电机驱动初始化 */
    FC_START_CRSF_Init();          /* 飞控启动状态机初始化 */
    pit_us_init(PIT_CH0, 1000);    /* PIT 定时器初始化，1000us 中断节拍（1kHz 基准） */
    pit_ms_init(PIT_CH1, 10);      /* PIT 定时器初始化，10ms 中断节拍（100Hz 基准） */

    while (true)
    {
        uint16 tick_1000_guard = 0U; /* 单次 while 循环内 1kHz 任务最大处理帧数防护 */

        /* ===== 1kHz 高频任务处理块 ===== */
        while ((g_tick_1000HZ > 0U) && (tick_1000_guard < 100U))
        {
            g_tick_1000HZ--;

            /* 1kHz：IMU 数据更新与位置估计预处理 */
            IMU_Update_1000HZ();
            Pos_Est_Update_1000HZ();

            /* 500Hz：飞控内环（姿态环）更新 */
            s_tick_div_fc_loop_500hz++;
            if (s_tick_div_fc_loop_500hz >= 2U)
            {
                s_tick_div_fc_loop_500hz = 0U;
                FC_Loop_500Hz();
            }

            /* 1kHz：飞控电机输出更新 */
            FC_Loop_1000Hz();

            /* 250Hz：位置估计主更新（光流 + TOF 融合） */
            s_tick_div_pos_250hz++;
            if (s_tick_div_pos_250hz >= 4U)
            {
                s_tick_div_pos_250hz = 0U;
                //Pos_Est_Update_250HZ();
                // wifi_vofa_JustFloat(3, g_euler.roll, g_euler.pitch, g_euler.yaw);

            }

            tick_1000_guard++;
        }
        
        /* ===== 100Hz 低频任务处理块 ===== */
        if (g_tick_100HZ > 0U)
        {
            g_tick_100HZ--;

            /* 100Hz：CRSF 遥控数据收发与解析 */
            crsf_send_25hz();       /* CRSF 25Hz 发送函数，每 100Hz 调用一次 */
            CRSF_Update_100HZ();
            FC_Loop_100Hz();        /* 飞控外环（速度环/位置环）更新 */
            /* 50Hz：位置估计辅助更新与飞控定高环 */
            s_tick_div_fc_start_50hz++;
            if (s_tick_div_fc_start_50hz >= 2)
            {
                Pos_Est_Update_50HZ();
                s_tick_div_fc_start_50hz = 0U;
                FC_Loop_50Hz();
                wifi_vofa_JustFloat(8u,g_pmw3901_raw.deltaX,g_pmw3901_raw.deltaY,g_imudata_250hz.gyrox,g_imudata_250hz.gyroy,FlowGyroDecoupler_GetDecX(), FlowGyroDecoupler_GetDecY(),Pos_Est_vel_x, Pos_Est_vel_y);
            }

            /* 10Hz：飞控启动状态机更新与紧急停桨检测 */
            s_tick_div_fc_start_10hz++;
            if (s_tick_div_fc_start_10hz >= 10U)
            {
                s_tick_div_fc_start_10hz = 0U;
                FC_START_CRSF_Update();

                /* 姿态超限保护：roll 或 pitch 超过 ±55° 时触发紧急停桨 */
                if (g_euler.roll > 55.0f || g_euler.roll < -55.0f ||
                    g_euler.pitch > 55.0f || g_euler.pitch < -55.0f)
                {
                    FC_START_CRSF_Trigger_Emergency_Stop();
                }
            }
        }

        /* ===== 后台任务：IMU 校准命令轮询（串口解析） ===== */
        IMUCalib_CommandPoll();
    }
}