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
#include "../code/HW_Drivers/ICM42688/ICM42688.h"
#include "../code/HW_Drivers/PMW3901/PMW3901.h"
#include "../code/Estimation/Height_Est/Height_Est.h"
#include "../code/Estimation/Attitude/IMU_TOP.h"
#include "../code/Protocols/crsf/crsf.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
volatile uint8 g_height_est_tick_100hz = 0U;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
    debug_init();                  // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等

    Height_Est_Init();         // 高度估计初始化（TOF+Baro）
    PMW3901_Init();            // PMW3901 光流传感器初始化
    IMU_Init_All();            // ICM42688 IMU 初始化
    crsf_init();               // CRSF 遥控协议初始化
    pit_us_init(PIT_CH0, 500); // PIT 定时器初始化 500us 中断周期 用于 IMU 2kHz 更新
    pit_ms_init(PIT_CH1, 10);  // 100Hz 节拍

    printf("1");
    for (int i = 0; i < 100; i++)
    {
        Height_Est_update_100HZ(); // 100Hz 更新一次高度估计
        system_delay_ms(10);
    }
    printf("2");

    while (true)
    {
        if (g_height_est_tick_100hz > 0U)
        {
            g_height_est_tick_100hz = 0U;
            Height_Est_update_100HZ();
        }
        // printf("%d,%f\r\n",g_height_est_mm, g_height_vz_mps); // 打印高度估计结果
        // VL53L1X_read_data(&VL53L1X_data); // 读取 VL53L1X 传感器数据
        // PMW3901_Update(); // 更新 PMW3901 光流传感器数据
        // printf("%d,%d,%d,%d,%d,%d\r\n",
        //        VL53L1X_data.VL53L1X2_distance_mm,
        //        VL53L1X_data.VL53L1X3_distance_mm,
        //        g_tof2_height_mm,
        //        g_tof3_height_mm,
        //        g_tof_fused_height_mm,
        //        g_tof_fused_source);
        // crsf_send_25hz(); // 25Hz 发送一次遥控数据
        // printf("%d,%d,%d,%d\r\n", CRSF_STD[0], CRSF_STD[1], CRSF_STD[2], CRSF_STD[3]);
        // printf("%d,%d,%f,%f\r\n",g_BMP388_data.raw_pressure, g_BMP388_data.raw_temperature, g_BMP388_data.pressure_pa, g_BMP388_data.temperature_c); // 打印 BMP388 气压和温度数据
        // printf("%f,%f,%f,%f,%f,%f\r\n", ICM42688.gyro_x, ICM42688.gyro_y, ICM42688.gyro_z, ICM42688.acc_x, ICM42688.acc_y, ICM42688.acc_z); // 打印 IMU 滤波后的陀螺和加速度数据
        // printf("%f,%f,%f,%f\r\n", g_euler.roll, g_euler.pitch, g_euler.yaw, g_baro_altitude); // 打印欧拉角和气压高度数据
        // printf("%d,%d,%d,%d\r\n", VL53L1X_data.VL53L1X2_distance_mm, VL53L1X_data.VL53L1X3_distance_mm, VL53L1X_data.VL53L1X2_range_status, VL53L1X_data.VL53L1X3_range_status);
        // printf("%d,%d,%d,%d\r\n", g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY, g_pmw3901_raw.squal, g_pmw3901_raw.observation); // 打印 PMW3901 光流数据
        system_delay_ms(1);
    }
}

// **************************** 代码区域 ****************************
