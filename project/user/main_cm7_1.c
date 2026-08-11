#include "zf_common_headfile.h"
#include "Estimation/Pos_Est/image_down.h"
#include "Display/image_debug_screen.h"
#include "Image/image_down_horizon.h"
#include "Protocols/CameraSpi/camera_spi.h"

#define IMAGE_PIT                          (PIT_CH10)
#define IMAGE_CORE1_TASK_PERIOD_US         CAMERA_SPI_UPDATE_PERIOD_US
#define IMAGE_ATTITUDE_TIMEOUT_US          (50000U)
#define IMAGE_ATTITUDE_TIMEOUT_TICKS \
    ((IMAGE_ATTITUDE_TIMEOUT_US + IMAGE_CORE1_TASK_PERIOD_US - 1U) / \
     IMAGE_CORE1_TASK_PERIOD_US)
#define IMAGE_SCREEN_TICK_PERIOD_US        (10000U)
#define IMAGE_SCREEN_TICK_DIVIDER \
    ((IMAGE_SCREEN_TICK_PERIOD_US + IMAGE_CORE1_TASK_PERIOD_US - 1U) / \
     IMAGE_CORE1_TASK_PERIOD_US)

volatile uint8 g_image_tick_100hz = 0U;
volatile uint32 g_image_core1_tick_generated;
volatile uint32 g_image_core1_tick_overflow_count;

static ipc_attitude_data_t s_image_attitude;
static uint32 s_image_attitude_sequence;
static uint8 s_image_attitude_age = IMAGE_ATTITUDE_TIMEOUT_TICKS;

static uint8 image_tick_take_one(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    uint8 tick_taken = 0U;

    if(g_image_tick_100hz > 0U)
    {
        g_image_tick_100hz--;
        tick_taken = 1U;
    }
    interrupt_global_enable(interrupt_state);
    return tick_taken;
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

/* 更新三路图像，并分别返回内容变化和真实新结果掩码。 */
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
            image_down_horizon_update(s_image_attitude.roll_deg,
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

int main(void)
{
    uint8 image_fresh_mask;
    uint8 image_changed_mask;
    uint8 image_task_pending = 0U;
    uint8 image_screen_tick_divider = 0U;

    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();

    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);
    ipc_remote_param_core1_init();
    ipc_image_init();
    image_down_init();
    image_down_horizon_init();
    CameraSpi_Init();
    g_image_tick_100hz = 0U;
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

            image_changed_mask = Get_Image_data(&image_fresh_mask);
            ipc_remote_param_core1_poll();
            if(image_changed_mask != 0U)
            {
                ipc_image_publish(image_fresh_mask);
            }
            ImageDebugScreen_Update(
                ((image_fresh_mask & (uint8)(1U << Center)) != 0U) ? 1U : 0U);
            image_task_pending = 0U;
            continue;
        }

        if(CameraSpi_Service() != 0U)
        {
            continue;
        }

        if(image_tick_take_one() != 0U)
        {
            image_screen_tick_divider++;
            if(image_screen_tick_divider >= IMAGE_SCREEN_TICK_DIVIDER)
            {
                image_screen_tick_divider = 0U;
                ImageDebugScreen_Tick10ms();
            }
            ipc_remote_param_core1_poll();
            CameraSpi_Update();
            image_task_pending = 1U;
        }
    }
}
