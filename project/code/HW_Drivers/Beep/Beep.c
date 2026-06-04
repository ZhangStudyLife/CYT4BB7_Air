#include "Beep.h"

/*
 * 蜂鸣器非阻塞驱动（100Hz节拍）
 * - 通过占空比控制“响/静”时间比例
 * - 通过周期时长 + 周期次数控制总播放时长
 */
#define BEEP_TICK_HZ            (100U)
#define BEEP_MAX_CYCLE_TIME_S   (600.0f)
#define BEEP_MAX_CYCLE_COUNT    (600U)

/* 运行状态 */
static uint8  s_beep_active = 0U;
/* 当前占空比（0~100） */
static uint8  s_beep_duty_percent = 0U;
/* 单个周期总tick数（cycle_time_s * 100Hz） */
static uint32 s_beep_cycle_ticks = 0U;
/* 当前周期内已运行tick */
static uint32 s_beep_tick_in_cycle = 0U;
/* 目标播放周期数 */
static uint16 s_beep_cycle_count = 0U;
/* 已完成周期数 */
static uint16 s_beep_cycle_done = 0U;

/* 占空比限幅到0~100 */
static uint8 Beep_ClampDuty(uint8 duty_percent)
{
    if (duty_percent > 100U)
    {
        return 100U;
    }
    return duty_percent;
}

void Beep_Init(void)
{
    /* 初始化为推挽输出，默认低电平（静音） */
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    Beep_Stop();
}

void Beep_Stop(void)
{
    /* 清空播放状态并拉低蜂鸣器引脚 */
    s_beep_active = 0U;
    s_beep_duty_percent = 0U;
    s_beep_cycle_ticks = 0U;
    s_beep_tick_in_cycle = 0U;
    s_beep_cycle_count = 0U;
    s_beep_cycle_done = 0U;
    gpio_low(BUZZER_PIN);
}

void Beep_Enable(void)
{
    s_beep_active = 0U;
    s_beep_duty_percent = 0U;
    s_beep_cycle_ticks = 0U;
    s_beep_tick_in_cycle = 0U;
    s_beep_cycle_count = 0U;
    s_beep_cycle_done = 0U;
    gpio_high(BUZZER_PIN);
}

void Beep_Disable(void)
{
    s_beep_active = 0U;
    s_beep_duty_percent = 0U;
    s_beep_cycle_ticks = 0U;
    s_beep_tick_in_cycle = 0U;
    s_beep_cycle_count = 0U;
    s_beep_cycle_done = 0U;
    gpio_low(BUZZER_PIN);
}

void Beep_Play(uint8 duty_percent, float cycle_time_s, uint16 cycle_count)
{
    /* 参数保护：限制最大播放时长与次数，防止异常长任务 */
    if (cycle_time_s > BEEP_MAX_CYCLE_TIME_S)
    {
        cycle_time_s = BEEP_MAX_CYCLE_TIME_S;
    }
    if (cycle_count > BEEP_MAX_CYCLE_COUNT)
    {
        cycle_count = BEEP_MAX_CYCLE_COUNT;
    }

    Beep_Stop();

    /* 将秒转换为100Hz节拍下的tick */
    s_beep_duty_percent = Beep_ClampDuty(duty_percent);
    if (cycle_time_s <= 0.0f)
    {
        return;
    }

    s_beep_cycle_ticks = (uint32)(cycle_time_s * (float)BEEP_TICK_HZ + 0.5f);
    s_beep_cycle_count = cycle_count;

    if (0U == s_beep_cycle_ticks)
    {
        s_beep_cycle_ticks = 1U;
    }

    /* 任一参数为0表示不播放 */
    if (0U == s_beep_cycle_count)
    {
        return;
    }

    s_beep_active = 1U;
}

void Beep_Update_100HZ(void)
{
    uint32 on_ticks;

    /* 空闲时直接返回，保持非阻塞 */
    if (0U == s_beep_active)
    {
        return;
    }

    /* 按占空比计算本周期应响tick数 */
    on_ticks = ((uint32)s_beep_duty_percent * s_beep_cycle_ticks) / 100U;

    /* 周期前半段(占空比部分)拉高，后半段拉低 */
    if ((on_ticks > 0U) && (s_beep_tick_in_cycle < on_ticks))
    {
        gpio_high(BUZZER_PIN);
    }
    else
    {
        gpio_low(BUZZER_PIN);
    }

    s_beep_tick_in_cycle++;
    if (s_beep_tick_in_cycle >= s_beep_cycle_ticks)
    {
        /* 进入下一个周期并统计完成次数 */
        s_beep_tick_in_cycle = 0U;
        s_beep_cycle_done++;
        if (s_beep_cycle_done >= s_beep_cycle_count)
        {
            /* 达到目标次数后自动停止 */
            Beep_Stop();
        }
    }
}
