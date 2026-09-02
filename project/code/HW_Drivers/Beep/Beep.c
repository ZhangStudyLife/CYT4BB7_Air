/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#include "Beep.h"

/*
 * 蜂鸣器非阻塞驱动（100Hz节拍）
 * - 通过占空比控制“响/静”时间比例
 * - 通过周期时长 + 周期次数控制总播放时长
 */
#define BEEP_TICK_HZ            (100U)
#define BEEP_MAX_CYCLE_TIME_S   (600.0f)
#define BEEP_MAX_CYCLE_COUNT    (600U)
/* 兼容Beep_Enable/Beep_Disable的持续报警位 */
#define BEEP_ALARM_LEGACY       (1U << 7)

/* 运行状态 */
static volatile uint8  s_beep_active = 0U;
/* 当前占空比（0~100） */
static volatile uint8  s_beep_duty_percent = 0U;
/* 单个周期总tick数（cycle_time_s * 100Hz） */
static volatile uint32 s_beep_cycle_ticks = 0U;
/* 当前周期内已运行tick */
static volatile uint32 s_beep_tick_in_cycle = 0U;
/* 目标播放周期数 */
static volatile uint16 s_beep_cycle_count = 0U;
/* 已完成周期数 */
static volatile uint16 s_beep_cycle_done = 0U;
/* 各持续报警源状态位，任一位置位时蜂鸣器保持高电平 */
static volatile uint8 s_beep_alarm_mask = 0U;

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
    s_beep_alarm_mask = 0U;
    Beep_Stop();
}

void Beep_Stop(void)
{
    /* 清空循环播放状态，输出仍服从持续报警位 */
    s_beep_active = 0U;
    s_beep_duty_percent = 0U;
    s_beep_cycle_ticks = 0U;
    s_beep_tick_in_cycle = 0U;
    s_beep_cycle_count = 0U;
    s_beep_cycle_done = 0U;
    if (s_beep_alarm_mask != 0U)
    {
        gpio_high(BUZZER_PIN);
    }
    else
    {
        gpio_low(BUZZER_PIN);
    }
}

void Beep_Enable(void)
{
    Beep_SetAlarm(BEEP_ALARM_LEGACY, 1U);
}

void Beep_Disable(void)
{
    Beep_SetAlarm(BEEP_ALARM_LEGACY, 0U);
}

/*
 * 设置指定持续报警源
 * alarm  : BEEP_ALARM_*报警位
 * active : 非0置位报警，0清除报警
 * 返回值 : 无
 */
void Beep_SetAlarm(uint8 alarm, uint8 active)
{
    if (active != 0U)
    {
        s_beep_alarm_mask |= alarm;
        gpio_high(BUZZER_PIN);
    }
    else
    {
        s_beep_alarm_mask &= (uint8)(~alarm);
        if ((s_beep_alarm_mask == 0U) && (s_beep_active == 0U))
        {
            gpio_low(BUZZER_PIN);
        }
    }
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
        if (s_beep_alarm_mask != 0U)
        {
            gpio_high(BUZZER_PIN);
        }
        else
        {
            gpio_low(BUZZER_PIN);
        }
        return;
    }

    /* 按占空比计算本周期应响tick数 */
    on_ticks = ((uint32)s_beep_duty_percent * s_beep_cycle_ticks) / 100U;

    /* 周期前半段(占空比部分)拉高，后半段拉低 */
    if ((s_beep_alarm_mask != 0U) ||
        ((on_ticks > 0U) && (s_beep_tick_in_cycle < on_ticks)))
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
