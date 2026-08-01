#include "zf_common_headfile.h"
#include "Estimation/Pos_Est/image_down.h"
#include "Display/image_debug_screen.h"
#include "Image/image_down_horizon.h"

#define IMAGE_PIT  (PIT_CH10)
#define IMAGE_ATTITUDE_TIMEOUT_TICKS  (5U)

volatile uint8 g_image_tick_100hz = 0U;

static ipc_attitude_data_t s_image_attitude;
static uint32 s_image_attitude_sequence;
static uint8 s_image_attitude_age = IMAGE_ATTITUDE_TIMEOUT_TICKS;

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

static uint8 Get_Image_data(void)
{
    uint8 image_frame_updated;
    uint8 attitude_valid = image_attitude_poll();
    uint8 height_valid =
        (((s_image_attitude.flags & IPC_ATTITUDE_FLAG_HEIGHT_VALID) != 0U) &&
         (attitude_valid != 0U)) ? 1U : 0U;

    image_frame_updated = image_down_update();
    if(image_frame_updated != 0U)
    {
        image_down_horizon_update(
            s_image_attitude.roll_deg,
            s_image_attitude.pitch_deg,
            s_image_attitude.height_mm,
            attitude_valid,
            height_valid);
    }
    else if((attitude_valid == 0U) || (height_valid == 0U))
    {
        image_down_horizon_invalidate();
    }
    CameraSpi_GetSnapshot(image_data);
    return image_frame_updated;
}

int main(void)
{
    uint8 image_frame_updated;
    uint8 image_task_pending = 0U;

    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();

    ImageDebugScreen_Init();
    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);
    ipc_remote_param_core1_init();
    image_data_clear(&image_data[Front]);
    image_data_clear(&image_data[Center]);
    image_data_clear(&image_data[Back]);
    image_down_init();
    image_down_horizon_init();
    CameraSpi_Init();
    pit_ms_init(IMAGE_PIT, 10U);
    while(true)
    {
        if(image_task_pending != 0U)
        {
            if(CameraSpi_Service() != 0U)
            {
                continue;
            }

            image_frame_updated = Get_Image_data();
            ipc_remote_param_core1_poll();
            ipc_image_publish();
            ImageDebugScreen_Update(image_frame_updated);
            image_task_pending = 0U;
            continue;
        }

        if(CameraSpi_Service() != 0U)
        {
            continue;
        }

        if(g_image_tick_100hz > 0U)
        {
            g_image_tick_100hz--;
            ImageDebugScreen_Tick10ms();
            ipc_remote_param_core1_poll();
            CameraSpi_Update();
            image_task_pending = 1U;
        }
    }
}
