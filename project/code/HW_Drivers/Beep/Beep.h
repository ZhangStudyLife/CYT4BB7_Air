#ifndef BEEP_H
#define BEEP_H

#include "zf_common_headfile.h"

/* 主板蜂鸣器引脚（有源蜂鸣器） */
#ifndef BUZZER_PIN
#define BUZZER_PIN (P19_4)
#endif

/* 初始化蜂鸣器（默认静音） */
void Beep_Init(void);

/* 100Hz周期调用：非阻塞刷新蜂鸣器输出 */
void Beep_Update_100HZ(void);

/* 立即停止鸣叫并清空播放状态 */
void Beep_Stop(void);

/* 使能蜂鸣器（持续响） */
void Beep_Enable(void);

/* 失能蜂鸣器（立即停） */
void Beep_Disable(void);

/*
 * 触发鸣叫
 * duty_percent : 占空比(0~100)
 * cycle_time_s : 单周期时长(秒)
 * cycle_count  : 循环次数
 */
void Beep_Play(uint8 duty_percent, float cycle_time_s, uint16 cycle_count);

#endif /* BEEP_H */
