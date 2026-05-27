#ifndef FC_LOOP_H
#define FC_LOOP_H

#include "fc_params.h"
#include "pid_core.h"
#include "zf_common_headfile.h"

extern pid_t roll_gyro_pid;
extern pid_t pitch_gyro_pid;
extern pid_t yaw_gyro_pid;
extern pid_t roll_angle_pid;
extern pid_t pitch_angle_pid;
extern pid_t yaw_angle_pid;
extern float roll_gyro_target;
extern float pitch_gyro_target;
extern float yaw_gyro_target;
extern float roll_angle_target;
extern float pitch_angle_target;
extern float yaw_angle_target;

/*
 * 主要是PID循环控制函数的声明，分别对应不同频率的控制环
 */
void FC_Loop_Init(void);  /* PID循环相关资源初始化 */
void FC_Loop_Reset(void); /* PID循环状态重置，通常在解锁或模式切换时调用 */
void FC_Loop_50Hz(void);  /* 50Hz */
void FC_Loop_100Hz(void);  /* 100Hz*/
void FC_Loop_500Hz(void); /* 500Hz角度外环，输出角速度目标 */
void FC_Loop_1000Hz(void);  /* 1kHz主循环，处理陀螺仪数据和角速度控制 */

#endif /* FC_LOOP_H */
