#include "zf_common_headfile.h"
#include "Estimation/Pos_Est/image_down.h"
#include "Display/image_debug_screen.h"
#include "Image/image_down_horizon.h"
#include "Protocols/CameraSpi/camera_spi.h"

#define IMAGE_PIT  (PIT_CH10)
#define IMAGE_CORE1_TASK_PERIOD_US          CAMERA_SPI_UPDATE_PERIOD_US
#define IMAGE_ATTITUDE_TIMEOUT_US           (50000U)
#define IMAGE_ATTITUDE_TIMEOUT_TICKS \
    ((IMAGE_ATTITUDE_TIMEOUT_US + IMAGE_CORE1_TASK_PERIOD_US - 1U) / \
     IMAGE_CORE1_TASK_PERIOD_US)
#define IMAGE_SCREEN_TICK_PERIOD_US         (10000U)
#define IMAGE_SCREEN_TICK_DIVIDER \
    ((IMAGE_SCREEN_TICK_PERIOD_US + IMAGE_CORE1_TASK_PERIOD_US - 1U) / \
     IMAGE_CORE1_TASK_PERIOD_US)
#define IMAGE_CAMERA_ALL_MASK                ((1U << IMAGE_CAMERA_COUNT) - 1U) /* 三路摄像头全部产生新结果的位掩码。 */
#define IMAGE_CORE1_PERF_WINDOW_FRAMES       (5000U)  /* 核1性能统计窗口的真实图像帧数。 */
#define IMAGE_CORE1_PERF_AUTO_BREAK_DEFAULT  (0U)     /* 默认仅冻结结果，不主动暂停核心。 */

volatile uint8 g_image_tick_100hz = 0U;              /* ISR累计、主循环逐个消费的图像任务待处理节拍数。 */
volatile uint32 g_image_core1_tick_generated;         /* PIT累计生成的图像任务节拍数。 */
volatile uint32 g_image_core1_tick_consumed;          /* 主循环累计消费的图像任务节拍数。 */
volatile uint32 g_image_core1_tick_overflow_count;    /* 待处理计数器饱和时无法排队的节拍数。 */

static ipc_attitude_data_t s_image_attitude;
static uint32 s_image_attitude_sequence;
static uint8 s_image_attitude_age = IMAGE_ATTITUDE_TIMEOUT_TICKS;
static uint32 s_image_core1_cycle_start_cycles;       /* 当前图像任务从节拍消费开始的DWT周期值。 */
static uint32 s_image_core1_spi_start_cycles;         /* 当前图像任务启动Camera SPI轮次的DWT周期值。 */
static uint32 s_image_core1_pre_spi_ipc_cycles;       /* Camera SPI启动前的IPC轮询周期值。 */
static uint32 s_image_core1_pre_spi_screen_cycles;    /* Camera SPI启动前的屏幕节拍周期值。 */
static uint8 s_image_core1_perf_started;              /* 是否已经收到统计窗口第一帧真实下摄图像。 */
static uint64 s_image_core1_perf_image_cycles_sum;    /* 下摄任务周期累计值。 */
static uint64 s_image_core1_perf_ipc_cycles_sum;      /* IPC轮询与发布周期累计值。 */
static uint64 s_image_core1_perf_screen_cycles_sum;   /* 屏幕更新周期累计值。 */
static uint64 s_image_core1_perf_spi_cycles_sum;      /* 两板Camera SPI轮次墙钟周期累计值。 */
static uint64 s_image_core1_perf_total_cycles_sum;    /* 图像任务完整墙钟周期累计值。 */
static uint64 s_image_core1_perf_elapsed_cycles;      /* 首末真实图像任务起点之间的DWT周期累计值。 */
static uint32 s_image_core1_perf_max_image_cycles;    /* 下摄任务最大周期值。 */
static uint32 s_image_core1_perf_max_ipc_cycles;      /* IPC轮询与发布最大周期值。 */
static uint32 s_image_core1_perf_max_screen_cycles;   /* 屏幕更新最大周期值。 */
static uint32 s_image_core1_perf_max_spi_cycles;      /* 两板Camera SPI轮次最大墙钟周期值。 */
static uint32 s_image_core1_perf_max_total_cycles;    /* 图像任务最大墙钟周期值。 */
static uint32 s_image_core1_perf_previous_cycle_start_cycles; /* 上一完整任务起点的DWT周期值。 */
static uint64 s_image_core1_perf_publish_elapsed_cycles;      /* 真实新结果发布首末间隔累计周期。 */
static uint64 s_image_core1_perf_bundle_elapsed_cycles;       /* 三路完整刷新首末间隔累计周期。 */
static uint64 s_image_core1_perf_camera_publish_elapsed_cycles[IMAGE_CAMERA_COUNT]; /* 各路最终IPC新结果发布首末间隔累计周期。 */
static uint32 s_image_core1_perf_publish_previous_cycles;     /* 上一次真实新结果发布的DWT周期值。 */
static uint32 s_image_core1_perf_bundle_previous_cycles;      /* 上一次三路完整刷新的DWT周期值。 */
static uint32 s_image_core1_perf_camera_publish_previous_cycles[IMAGE_CAMERA_COUNT]; /* 各路上一次最终IPC新结果发布的DWT周期值。 */
static uint32 s_image_core1_perf_publish_max_gap_cycles;      /* 真实新结果发布最大间隔周期。 */
static uint32 s_image_core1_perf_bundle_max_gap_cycles;       /* 三路完整刷新最大间隔周期。 */
static uint32 s_image_core1_perf_camera_publish_max_gap_cycles[IMAGE_CAMERA_COUNT]; /* 各路最终IPC新结果发布最大间隔周期。 */
static uint8 s_image_core1_perf_bundle_pending_mask;          /* 尚未组成三路完整刷新的新结果掩码。 */

volatile uint8 g_image_core1_perf_auto_break_enabled = IMAGE_CORE1_PERF_AUTO_BREAK_DEFAULT; /* 是否允许统计完成后在调试器中自动暂停。 */
volatile uint8 g_image_core1_perf_window_done;             /* 核1性能统计窗口是否已经冻结。 */
volatile uint32 g_image_core1_perf_auto_break_count;       /* 核1自动暂停断点累计触发次数。 */
volatile uint32 g_image_core1_perf_processed_frames;       /* 窗口内完成真实下摄图像处理的帧数。 */
volatile uint32 g_image_core1_perf_task_count;              /* 窗口内完成的图像任务数。 */
volatile uint32 g_image_core1_perf_no_image_cycles;        /* 窗口内图像任务未拿到新下摄图像的次数。 */
volatile uint32 g_image_core1_perf_tick_backlog_count;     /* 窗口内任务结束时仍有下一节拍待消费的次数。 */
volatile uint32 g_image_core1_perf_max_tick_backlog;       /* 窗口内任务结束时待消费节拍数的最大值。 */
volatile uint32 g_image_core1_perf_elapsed_ms;             /* 统计窗口完整任务的DWT时间跨度，单位毫秒。 */
volatile float g_image_core1_perf_processed_fps;           /* 核1完整流水线实际处理帧率，单位帧每秒。 */
volatile float g_image_core1_perf_task_hz;                  /* 核1完整流水线实际任务频率，单位赫兹。 */
volatile uint32 g_image_core1_perf_fresh_publish_updates;   /* 至少一路产生真实新结果的发布次数。 */
volatile uint32 g_image_core1_perf_all_camera_bundle_updates; /* 前中后三路均至少更新一次的完整刷新次数。 */
volatile uint32 g_image_core1_perf_camera_publish_updates[IMAGE_CAMERA_COUNT]; /* 各路产生真实新结果的最终IPC发布次数。 */
volatile float g_image_core1_perf_fresh_publish_hz;         /* 最终image_data真实新结果发布频率，单位赫兹。 */
volatile float g_image_core1_perf_all_camera_bundle_hz;     /* 三路完整刷新频率，单位赫兹。 */
volatile float g_image_core1_perf_camera_publish_hz[IMAGE_CAMERA_COUNT]; /* 各路最终image_data真实新结果发布频率，单位赫兹。 */
volatile uint32 g_image_core1_perf_publish_max_gap_us;      /* 最终image_data真实新结果发布最大间隔，单位微秒。 */
volatile uint32 g_image_core1_perf_bundle_max_gap_us;       /* 三路完整刷新最大间隔，单位微秒。 */
volatile uint32 g_image_core1_perf_camera_publish_max_gap_us[IMAGE_CAMERA_COUNT]; /* 各路最终image_data真实新结果发布最大间隔，单位微秒。 */
volatile uint32 g_image_core1_perf_publish_over_10ms_gap_count; /* 最终发布间隔超过10毫秒的次数。 */
volatile uint32 g_image_core1_perf_bundle_over_10ms_gap_count; /* 三路完整刷新间隔超过10毫秒的次数。 */
volatile uint32 g_image_core1_perf_camera_publish_over_10ms_gap_count[IMAGE_CAMERA_COUNT]; /* 各路最终发布间隔超过10毫秒的次数。 */
volatile uint32 g_image_core1_perf_last_image_us;          /* 最近一次下摄任务耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_last_ipc_us;            /* 最近一次IPC轮询与发布耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_last_screen_us;         /* 最近一次屏幕更新耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_last_spi_cycle_us;      /* 最近一轮两板Camera SPI墙钟耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_last_total_us;          /* 最近一次图像任务完整耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_average_image_us;       /* 下摄任务平均耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_average_ipc_us;         /* IPC轮询与发布平均耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_average_screen_us;      /* 屏幕更新平均耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_average_spi_cycle_us;   /* 两板Camera SPI轮次平均墙钟耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_average_total_us;       /* 图像任务完整平均墙钟耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_max_image_us;           /* 下摄任务最大耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_max_ipc_us;             /* IPC轮询与发布最大耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_max_screen_us;          /* 屏幕更新最大耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_max_spi_cycle_us;       /* 两板Camera SPI轮次最大墙钟耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_max_total_us;           /* 图像任务完整最大墙钟耗时，单位微秒。 */
volatile uint32 g_image_core1_perf_over_period_count;      /* 完整图像任务超过目标周期的次数。 */
volatile uint32 g_image_core1_perf_over_10ms_count;        /* 完整图像任务超过10毫秒的次数。 */

/*
 * 函数功能: 在短临界区内消费一个图像任务节拍，避免主循环读改写与PIT中断竞争。
 * 输入参数: 无。
 * 返回值: 成功消费返回1，否则返回0。
 */
static uint8 image_tick_take_one(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    uint8 tick_taken = 0U;

    if(g_image_tick_100hz > 0U)
    {
        g_image_tick_100hz--;
        g_image_core1_tick_consumed++;
        tick_taken = 1U;
    }
    interrupt_global_enable(interrupt_state);
    return tick_taken;
}

/*
 * 函数功能: 在短临界区内读取当前待消费的图像任务节拍数。
 * 输入参数: 无。
 * 返回值: 当前待消费节拍数，范围0至255。
 */
static uint8 image_tick_pending_count(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    uint8 pending_count = g_image_tick_100hz;

    interrupt_global_enable(interrupt_state);
    return pending_count;
}

static uint8 image_attitude_poll(void)
{
    ipc_attitude_data_t attitude;

    ipc_attitude_get(&attitude);
    if((attitude.sequence != 0U) &&
       (attitude.sequence != s_image_attitude_sequence))
    {
        s_image_attitude = attitude;
        s_image_attitude_sequence = attitude.sequence;
        s_image_attitude_age = 0U;
    }
    else if(s_image_attitude_age < IMAGE_ATTITUDE_TIMEOUT_TICKS)
    {
        s_image_attitude_age++;
    }
    return (s_image_attitude_age < IMAGE_ATTITUDE_TIMEOUT_TICKS) ? 1U : 0U;
}

/*
 * 函数功能: 更新三路图像结果并区分真实新结果与超时清空等状态变化。
 * 输入参数: fresh_mask输出本轮真实新算法结果对应的摄像头位掩码。
 * 返回值: 本轮image_data内容发生变化的摄像头位掩码。
 */
static uint8 Get_Image_data(uint8 *fresh_mask)
{
    uint8 image_frame_updated;
    uint8 camera_spi_fresh_mask = 0U;
    uint8 changed_mask;
    uint8 image_frame_ready = (mt9v03x_finish_flag != 0U) ? 1U : 0U;
    uint8 attitude_valid = image_attitude_poll();
    uint8 height_valid =
        (((s_image_attitude.flags & IPC_ATTITUDE_FLAG_HEIGHT_VALID) != 0U) &&
         (attitude_valid != 0U)) ? 1U : 0U;

    if(image_frame_ready != 0U)
    {
        if((attitude_valid != 0U) && (height_valid != 0U))
        {
            image_down_horizon_update(
                s_image_attitude.roll_deg,
                s_image_attitude.pitch_deg,
                s_image_attitude.height_mm,
                attitude_valid,
                height_valid);
        }
        else
        {
            image_down_horizon_invalidate();
        }
    }
    else if((attitude_valid == 0U) || (height_valid == 0U))
    {
        image_down_horizon_invalidate();
    }
    image_frame_updated = image_down_update();
    changed_mask = CameraSpi_GetSnapshot(image_data, &camera_spi_fresh_mask);
    if(image_frame_updated != 0U)
    {
        camera_spi_fresh_mask |= (uint8)(1U << Center);
        changed_mask |= (uint8)(1U << Center);
    }
    if(fresh_mask != NULL)
    {
        *fresh_mask = camera_spi_fresh_mask;
    }
    return changed_mask;
}

/*
 * 函数功能: 记录核1一次完整图像任务的分段耗时，并在5000个真实图像帧后冻结结果。
 * 输入参数: fresh_mask表示本轮真实新结果；pending_ticks表示任务结束时的待消费节拍数；publish_cycles为实际发布完成时刻；其余参数为下摄、IPC、屏幕、Camera SPI轮次和完整任务的DWT周期数。
 * 返回值: 无。
 */
static void image_core1_perf_record(uint8 fresh_mask,
                                    uint8 pending_ticks,
                                    uint32 publish_cycles,
                                    uint32 image_cycles,
                                    uint32 ipc_cycles,
                                    uint32 screen_cycles,
                                    uint32 spi_cycles,
                                    uint32 total_cycles)
{
    uint8 camera_id;
    uint32 cycles_per_us;
    uint32 gap_cycles;

    if(g_image_core1_perf_window_done != 0U)
    {
        return;
    }

    if(s_image_core1_perf_started == 0U)
    {
        if((fresh_mask & (uint8)(1U << Center)) == 0U)
        {
            return;
        }
        s_image_core1_perf_started = 1U;
        s_image_core1_perf_previous_cycle_start_cycles =
            s_image_core1_cycle_start_cycles;
    }
    else
    {
        s_image_core1_perf_elapsed_cycles +=
            (uint64)(uint32)(s_image_core1_cycle_start_cycles -
                             s_image_core1_perf_previous_cycle_start_cycles);
        s_image_core1_perf_previous_cycle_start_cycles =
            s_image_core1_cycle_start_cycles;
    }

    g_image_core1_perf_task_count++;
    if((fresh_mask & (uint8)(1U << Center)) != 0U)
    {
        g_image_core1_perf_processed_frames++;
    }
    else
    {
        g_image_core1_perf_no_image_cycles++;
    }

    cycles_per_us = SystemCoreClock / 1000000U;
    if(cycles_per_us == 0U)
    {
        cycles_per_us = 1U;
    }

    if(fresh_mask != 0U)
    {
        for(camera_id = 0U; camera_id < IMAGE_CAMERA_COUNT; camera_id++)
        {
            if((fresh_mask & (uint8)(1U << camera_id)) == 0U)
            {
                continue;
            }

            if(g_image_core1_perf_camera_publish_updates[camera_id] != 0U)
            {
                gap_cycles = publish_cycles -
                             s_image_core1_perf_camera_publish_previous_cycles[camera_id];
                s_image_core1_perf_camera_publish_elapsed_cycles[camera_id] +=
                    (uint64)gap_cycles;
                if(gap_cycles >
                   s_image_core1_perf_camera_publish_max_gap_cycles[camera_id])
                {
                    s_image_core1_perf_camera_publish_max_gap_cycles[camera_id] =
                        gap_cycles;
                }
                if(gap_cycles > (SystemCoreClock / 100U))
                {
                    g_image_core1_perf_camera_publish_over_10ms_gap_count[camera_id]++;
                }
            }
            s_image_core1_perf_camera_publish_previous_cycles[camera_id] =
                publish_cycles;
            g_image_core1_perf_camera_publish_updates[camera_id]++;
        }

        if(g_image_core1_perf_fresh_publish_updates != 0U)
        {
            gap_cycles = publish_cycles -
                         s_image_core1_perf_publish_previous_cycles;
            s_image_core1_perf_publish_elapsed_cycles += (uint64)gap_cycles;
            if(gap_cycles > s_image_core1_perf_publish_max_gap_cycles)
            {
                s_image_core1_perf_publish_max_gap_cycles = gap_cycles;
            }
            if(gap_cycles > (SystemCoreClock / 100U))
            {
                g_image_core1_perf_publish_over_10ms_gap_count++;
            }
        }
        s_image_core1_perf_publish_previous_cycles =
            publish_cycles;
        g_image_core1_perf_fresh_publish_updates++;

        s_image_core1_perf_bundle_pending_mask |= fresh_mask;
        if((s_image_core1_perf_bundle_pending_mask & IMAGE_CAMERA_ALL_MASK) ==
           IMAGE_CAMERA_ALL_MASK)
        {
            if(g_image_core1_perf_all_camera_bundle_updates != 0U)
            {
                gap_cycles = publish_cycles -
                             s_image_core1_perf_bundle_previous_cycles;
                s_image_core1_perf_bundle_elapsed_cycles += (uint64)gap_cycles;
                if(gap_cycles > s_image_core1_perf_bundle_max_gap_cycles)
                {
                    s_image_core1_perf_bundle_max_gap_cycles = gap_cycles;
                }
                if(gap_cycles > (SystemCoreClock / 100U))
                {
                    g_image_core1_perf_bundle_over_10ms_gap_count++;
                }
            }
            s_image_core1_perf_bundle_previous_cycles =
                publish_cycles;
            g_image_core1_perf_all_camera_bundle_updates++;
            s_image_core1_perf_bundle_pending_mask = 0U;
        }
    }

    g_image_core1_perf_last_image_us =
        (image_cycles + cycles_per_us / 2U) / cycles_per_us;
    g_image_core1_perf_last_ipc_us =
        (ipc_cycles + cycles_per_us / 2U) / cycles_per_us;
    g_image_core1_perf_last_screen_us =
        (screen_cycles + cycles_per_us / 2U) / cycles_per_us;
    g_image_core1_perf_last_spi_cycle_us =
        (spi_cycles + cycles_per_us / 2U) / cycles_per_us;
    g_image_core1_perf_last_total_us =
        (total_cycles + cycles_per_us / 2U) / cycles_per_us;

    s_image_core1_perf_image_cycles_sum += (uint64)image_cycles;
    s_image_core1_perf_ipc_cycles_sum += (uint64)ipc_cycles;
    s_image_core1_perf_screen_cycles_sum += (uint64)screen_cycles;
    s_image_core1_perf_spi_cycles_sum += (uint64)spi_cycles;
    s_image_core1_perf_total_cycles_sum += (uint64)total_cycles;
    if(image_cycles > s_image_core1_perf_max_image_cycles)
    {
        s_image_core1_perf_max_image_cycles = image_cycles;
    }
    if(ipc_cycles > s_image_core1_perf_max_ipc_cycles)
    {
        s_image_core1_perf_max_ipc_cycles = ipc_cycles;
    }
    if(screen_cycles > s_image_core1_perf_max_screen_cycles)
    {
        s_image_core1_perf_max_screen_cycles = screen_cycles;
    }
    if(spi_cycles > s_image_core1_perf_max_spi_cycles)
    {
        s_image_core1_perf_max_spi_cycles = spi_cycles;
    }
    if(total_cycles > s_image_core1_perf_max_total_cycles)
    {
        s_image_core1_perf_max_total_cycles = total_cycles;
    }
    if(pending_ticks > 0U)
    {
        g_image_core1_perf_tick_backlog_count++;
    }
    if((uint32)pending_ticks > g_image_core1_perf_max_tick_backlog)
    {
        g_image_core1_perf_max_tick_backlog = (uint32)pending_ticks;
    }
    if(total_cycles > (cycles_per_us * IMAGE_CORE1_TASK_PERIOD_US))
    {
        g_image_core1_perf_over_period_count++;
    }
    if(total_cycles > (cycles_per_us * 10000U))
    {
        g_image_core1_perf_over_10ms_count++;
    }

    if(g_image_core1_perf_processed_frames >= IMAGE_CORE1_PERF_WINDOW_FRAMES)
    {
        if(s_image_core1_perf_elapsed_cycles == 0U)
        {
            s_image_core1_perf_elapsed_cycles = 1U;
        }
        g_image_core1_perf_elapsed_ms = (uint32)(
            (s_image_core1_perf_elapsed_cycles * 1000ULL +
             (uint64)(SystemCoreClock / 2U)) /
            (uint64)SystemCoreClock);
        g_image_core1_perf_processed_fps =
            ((float)(g_image_core1_perf_processed_frames - 1U) *
             (float)SystemCoreClock) /
            (float)s_image_core1_perf_elapsed_cycles;
        g_image_core1_perf_task_hz =
            ((float)(g_image_core1_perf_task_count - 1U) *
             (float)SystemCoreClock) /
            (float)s_image_core1_perf_elapsed_cycles;
        if((g_image_core1_perf_fresh_publish_updates > 1U) &&
           (s_image_core1_perf_publish_elapsed_cycles != 0U))
        {
            g_image_core1_perf_fresh_publish_hz =
                ((float)(g_image_core1_perf_fresh_publish_updates - 1U) *
                 (float)SystemCoreClock) /
                (float)s_image_core1_perf_publish_elapsed_cycles;
        }
        if((g_image_core1_perf_all_camera_bundle_updates > 1U) &&
           (s_image_core1_perf_bundle_elapsed_cycles != 0U))
        {
            g_image_core1_perf_all_camera_bundle_hz =
                ((float)(g_image_core1_perf_all_camera_bundle_updates - 1U) *
                 (float)SystemCoreClock) /
                (float)s_image_core1_perf_bundle_elapsed_cycles;
        }
        for(camera_id = 0U; camera_id < IMAGE_CAMERA_COUNT; camera_id++)
        {
            if((g_image_core1_perf_camera_publish_updates[camera_id] > 1U) &&
               (s_image_core1_perf_camera_publish_elapsed_cycles[camera_id] != 0U))
            {
                g_image_core1_perf_camera_publish_hz[camera_id] =
                    ((float)(g_image_core1_perf_camera_publish_updates[camera_id] - 1U) *
                     (float)SystemCoreClock) /
                    (float)s_image_core1_perf_camera_publish_elapsed_cycles[camera_id];
            }
        }
        g_image_core1_perf_publish_max_gap_us =
            (s_image_core1_perf_publish_max_gap_cycles + cycles_per_us / 2U) /
            cycles_per_us;
        g_image_core1_perf_bundle_max_gap_us =
            (s_image_core1_perf_bundle_max_gap_cycles + cycles_per_us / 2U) /
            cycles_per_us;
        for(camera_id = 0U; camera_id < IMAGE_CAMERA_COUNT; camera_id++)
        {
            g_image_core1_perf_camera_publish_max_gap_us[camera_id] =
                (s_image_core1_perf_camera_publish_max_gap_cycles[camera_id] +
                 cycles_per_us / 2U) /
                cycles_per_us;
        }
        g_image_core1_perf_average_image_us = (uint32)(
            ((s_image_core1_perf_image_cycles_sum /
              (uint64)g_image_core1_perf_task_count) +
             (uint64)(cycles_per_us / 2U)) /
            (uint64)cycles_per_us);
        g_image_core1_perf_average_ipc_us = (uint32)(
            ((s_image_core1_perf_ipc_cycles_sum /
              (uint64)g_image_core1_perf_task_count) +
             (uint64)(cycles_per_us / 2U)) /
            (uint64)cycles_per_us);
        g_image_core1_perf_average_screen_us = (uint32)(
            ((s_image_core1_perf_screen_cycles_sum /
              (uint64)g_image_core1_perf_task_count) +
             (uint64)(cycles_per_us / 2U)) /
            (uint64)cycles_per_us);
        g_image_core1_perf_average_spi_cycle_us = (uint32)(
            ((s_image_core1_perf_spi_cycles_sum /
              (uint64)g_image_core1_perf_task_count) +
             (uint64)(cycles_per_us / 2U)) /
            (uint64)cycles_per_us);
        g_image_core1_perf_average_total_us = (uint32)(
            ((s_image_core1_perf_total_cycles_sum /
              (uint64)g_image_core1_perf_task_count) +
             (uint64)(cycles_per_us / 2U)) /
            (uint64)cycles_per_us);
        g_image_core1_perf_max_image_us =
            (s_image_core1_perf_max_image_cycles + cycles_per_us / 2U) /
            cycles_per_us;
        g_image_core1_perf_max_ipc_us =
            (s_image_core1_perf_max_ipc_cycles + cycles_per_us / 2U) /
            cycles_per_us;
        g_image_core1_perf_max_screen_us =
            (s_image_core1_perf_max_screen_cycles + cycles_per_us / 2U) /
            cycles_per_us;
        g_image_core1_perf_max_spi_cycle_us =
            (s_image_core1_perf_max_spi_cycles + cycles_per_us / 2U) /
            cycles_per_us;
        g_image_core1_perf_max_total_us =
            (s_image_core1_perf_max_total_cycles + cycles_per_us / 2U) /
            cycles_per_us;
        __DMB();
        g_image_core1_perf_window_done = 1U;

        if((g_image_core1_perf_auto_break_enabled != 0U) &&
           ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U))
        {
            g_image_core1_perf_auto_break_count++;
            __BKPT(0);
        }
    }
}

int main(void)
{
    uint8 image_fresh_mask;
    uint8 image_changed_mask;
    uint8 image_task_pending = 0U;
    uint8 image_screen_tick_divider = 0U;
    uint8 pending_ticks;
    uint32 spi_cycle_cycles;
    uint32 image_start_cycles;
    uint32 image_publish_cycles;
    uint32 ipc_start_cycles;
    uint32 screen_start_cycles;
    uint32 image_cycles;
    uint32 ipc_cycles;
    uint32 screen_cycles;
    uint32 total_cycles;

    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();

    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);
    ipc_remote_param_core1_init();
    ipc_image_init();
    image_down_init();
    image_down_horizon_init();
    CameraSpi_Init();
    g_image_tick_100hz = 0U;
    g_image_core1_tick_generated = 0U;
    g_image_core1_tick_consumed = 0U;
    g_image_core1_tick_overflow_count = 0U;
    g_image_core1_perf_auto_break_enabled = IMAGE_CORE1_PERF_AUTO_BREAK_DEFAULT;
    g_image_core1_perf_window_done = 0U;
    g_image_core1_perf_auto_break_count = 0U;
    g_image_core1_perf_processed_frames = 0U;
    g_image_core1_perf_task_count = 0U;
    g_image_core1_perf_no_image_cycles = 0U;
    g_image_core1_perf_tick_backlog_count = 0U;
    g_image_core1_perf_max_tick_backlog = 0U;
    g_image_core1_perf_elapsed_ms = 0U;
    g_image_core1_perf_processed_fps = 0.0f;
    g_image_core1_perf_task_hz = 0.0f;
    g_image_core1_perf_fresh_publish_updates = 0U;
    g_image_core1_perf_all_camera_bundle_updates = 0U;
    memset((void *)g_image_core1_perf_camera_publish_updates,
           0,
           sizeof(g_image_core1_perf_camera_publish_updates));
    g_image_core1_perf_fresh_publish_hz = 0.0f;
    g_image_core1_perf_all_camera_bundle_hz = 0.0f;
    memset((void *)g_image_core1_perf_camera_publish_hz,
           0,
           sizeof(g_image_core1_perf_camera_publish_hz));
    g_image_core1_perf_publish_max_gap_us = 0U;
    g_image_core1_perf_bundle_max_gap_us = 0U;
    memset((void *)g_image_core1_perf_camera_publish_max_gap_us,
           0,
           sizeof(g_image_core1_perf_camera_publish_max_gap_us));
    g_image_core1_perf_publish_over_10ms_gap_count = 0U;
    g_image_core1_perf_bundle_over_10ms_gap_count = 0U;
    memset((void *)g_image_core1_perf_camera_publish_over_10ms_gap_count,
           0,
           sizeof(g_image_core1_perf_camera_publish_over_10ms_gap_count));
    g_image_core1_perf_last_image_us = 0U;
    g_image_core1_perf_last_ipc_us = 0U;
    g_image_core1_perf_last_screen_us = 0U;
    g_image_core1_perf_last_spi_cycle_us = 0U;
    g_image_core1_perf_last_total_us = 0U;
    g_image_core1_perf_average_image_us = 0U;
    g_image_core1_perf_average_ipc_us = 0U;
    g_image_core1_perf_average_screen_us = 0U;
    g_image_core1_perf_average_spi_cycle_us = 0U;
    g_image_core1_perf_average_total_us = 0U;
    g_image_core1_perf_max_image_us = 0U;
    g_image_core1_perf_max_ipc_us = 0U;
    g_image_core1_perf_max_screen_us = 0U;
    g_image_core1_perf_max_spi_cycle_us = 0U;
    g_image_core1_perf_max_total_us = 0U;
    g_image_core1_perf_over_period_count = 0U;
    g_image_core1_perf_over_10ms_count = 0U;
    s_image_core1_cycle_start_cycles = 0U;
    s_image_core1_spi_start_cycles = 0U;
    s_image_core1_pre_spi_ipc_cycles = 0U;
    s_image_core1_pre_spi_screen_cycles = 0U;
    s_image_core1_perf_started = 0U;
    s_image_core1_perf_image_cycles_sum = 0U;
    s_image_core1_perf_ipc_cycles_sum = 0U;
    s_image_core1_perf_screen_cycles_sum = 0U;
    s_image_core1_perf_spi_cycles_sum = 0U;
    s_image_core1_perf_total_cycles_sum = 0U;
    s_image_core1_perf_elapsed_cycles = 0U;
    s_image_core1_perf_max_image_cycles = 0U;
    s_image_core1_perf_max_ipc_cycles = 0U;
    s_image_core1_perf_max_screen_cycles = 0U;
    s_image_core1_perf_max_spi_cycles = 0U;
    s_image_core1_perf_max_total_cycles = 0U;
    s_image_core1_perf_previous_cycle_start_cycles = 0U;
    s_image_core1_perf_publish_elapsed_cycles = 0U;
    s_image_core1_perf_bundle_elapsed_cycles = 0U;
    memset(s_image_core1_perf_camera_publish_elapsed_cycles,
           0,
           sizeof(s_image_core1_perf_camera_publish_elapsed_cycles));
    s_image_core1_perf_publish_previous_cycles = 0U;
    s_image_core1_perf_bundle_previous_cycles = 0U;
    memset(s_image_core1_perf_camera_publish_previous_cycles,
           0,
           sizeof(s_image_core1_perf_camera_publish_previous_cycles));
    s_image_core1_perf_publish_max_gap_cycles = 0U;
    s_image_core1_perf_bundle_max_gap_cycles = 0U;
    memset(s_image_core1_perf_camera_publish_max_gap_cycles,
           0,
           sizeof(s_image_core1_perf_camera_publish_max_gap_cycles));
    s_image_core1_perf_bundle_pending_mask = 0U;
    ImageDebugScreen_Init();
    pit_us_init(IMAGE_PIT, IMAGE_CORE1_TASK_PERIOD_US);
    while(true)
    {
        if(image_task_pending != 0U)
        {
            if(CameraSpi_Service() != 0U)
            {
                continue;
            }

            spi_cycle_cycles = DWT->CYCCNT - s_image_core1_spi_start_cycles;
            image_start_cycles = DWT->CYCCNT;
            image_changed_mask = Get_Image_data(&image_fresh_mask);
            image_cycles = DWT->CYCCNT - image_start_cycles;
            ipc_start_cycles = DWT->CYCCNT;
            ipc_remote_param_core1_poll();
            image_publish_cycles = 0U;
            if(image_changed_mask != 0U)
            {
                ipc_image_publish(image_fresh_mask);
                if(image_fresh_mask != 0U)
                {
                    image_publish_cycles = DWT->CYCCNT;
                }
            }
            ipc_cycles = s_image_core1_pre_spi_ipc_cycles +
                         (DWT->CYCCNT - ipc_start_cycles);
            screen_start_cycles = DWT->CYCCNT;
            ImageDebugScreen_Update(
                ((image_fresh_mask & (uint8)(1U << Center)) != 0U) ? 1U : 0U);
            screen_cycles = s_image_core1_pre_spi_screen_cycles +
                            (DWT->CYCCNT - screen_start_cycles);
            total_cycles = DWT->CYCCNT - s_image_core1_cycle_start_cycles;
            pending_ticks = image_tick_pending_count();
            image_core1_perf_record(image_fresh_mask,
                                    pending_ticks,
                                    image_publish_cycles,
                                    image_cycles,
                                    ipc_cycles,
                                    screen_cycles,
                                    spi_cycle_cycles,
                                    total_cycles);
            image_task_pending = 0U;
            continue;
        }

        if(CameraSpi_Service() != 0U)
        {
            continue;
        }

        if(image_tick_take_one() != 0U)
        {
            s_image_core1_cycle_start_cycles = DWT->CYCCNT;
            s_image_core1_pre_spi_screen_cycles = 0U;
            image_screen_tick_divider++;
            if(image_screen_tick_divider >= IMAGE_SCREEN_TICK_DIVIDER)
            {
                image_screen_tick_divider = 0U;
                screen_start_cycles = DWT->CYCCNT;
                ImageDebugScreen_Tick10ms();
                s_image_core1_pre_spi_screen_cycles =
                    DWT->CYCCNT - screen_start_cycles;
            }
            ipc_start_cycles = DWT->CYCCNT;
            ipc_remote_param_core1_poll();
            s_image_core1_pre_spi_ipc_cycles =
                DWT->CYCCNT - ipc_start_cycles;
            s_image_core1_spi_start_cycles = DWT->CYCCNT;
            CameraSpi_Update();
            image_task_pending = 1U;
        }
    }
}
