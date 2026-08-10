#include "zf_common_headfile.h"
#include "Estimation/Pos_Est/image_down.h"
#include "Display/image_debug_screen.h"
#include "Image/image_down_horizon.h"
#include "Image/car_lamp_cross_check.h"

#define IMAGE_PIT  (PIT_CH10)
#define IMAGE_TIME_PIT  (PIT_CH11)
#define IMAGE_ATTITUDE_TIMEOUT_TICKS  (5U)

volatile uint8 g_image_tick_100hz = 0U;
volatile uint32 g_image_master_time_ms = 0U; /* 核心1统一毫秒时间。 */

static ipc_attitude_data_t s_image_attitude;
static uint32 s_image_attitude_sequence;
static uint8 s_image_attitude_age = IMAGE_ATTITUDE_TIMEOUT_TICKS;
static uint32 s_roi_diag_frame_sequence[IMAGE_CAMERA_COUNT];

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

static void image_cross_check_apply_runtime_diag(void)
{
    uint8 roi_hit_mask = 0U;
    uint8 fallback_mask = 0U;
    uint8 conflict_mask = 0U;
    uint8 sample_mask = 0U;
    uint8 camera;

    g_camera_spi_roi_diag[Center] = g_image_down_car_lamp_roi_diag;
    for(camera = 0U; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        uint8 flags = g_camera_spi_roi_diag[camera];
        uint8 camera_bit = CAR_LAMP_CAMERA_BIT(camera);
        const image_frame_meta_t *meta = &image_frame_meta[camera];

        if((meta->frame_valid != 0U) &&
           (meta->frame_sequence != 0U) &&
           (meta->frame_sequence != s_roi_diag_frame_sequence[camera]))
        {
            s_roi_diag_frame_sequence[camera] = meta->frame_sequence;
            sample_mask |= camera_bit;
        }

        if((flags & CAMERA_SPI_ROI_DIAG_HIT) != 0U)
        {
            roi_hit_mask |= camera_bit;
        }
        if((flags & CAMERA_SPI_ROI_DIAG_FALLBACK) != 0U)
        {
            fallback_mask |= camera_bit;
        }
        if((flags & CAMERA_SPI_ROI_DIAG_CONFLICT) != 0U)
        {
            conflict_mask |= camera_bit;
        }
    }
    CarLampCrossCheck_ApplyRuntimeDiag(
        roi_hit_mask, fallback_mask, conflict_mask, sample_mask);
}

static uint8 Get_Image_data(void)
{
    uint8 image_frame_updated;
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
    image_down_set_roi_context(s_image_attitude.roll_deg,
                               s_image_attitude.pitch_deg,
                               s_image_attitude.height_mm,
                               attitude_valid,
                               height_valid);
    image_frame_updated = image_down_update();
    if(image_frame_updated != 0U)
    {
        CameraSpi_SubmitLocalFrame(&image_data[Center],
                                   &image_frame_meta[Center]);
    }
    CameraSpi_GetSnapshot(image_data);
    {
        image_sync_set_t latest_set;

        (void)CameraSpi_GetLatestSet(&latest_set);
        (void)CarLampCrossCheck_UpdateAt(
            &latest_set,
            g_image_master_time_ms,
            s_image_attitude.roll_deg,
            s_image_attitude.pitch_deg,
            s_image_attitude.height_mm,
            attitude_valid,
            height_valid);
        image_cross_check_apply_runtime_diag();
    }
    return image_frame_updated;
}

int main(void)
{
    uint8 image_frame_updated;
    uint8 image_task_pending = 0U;

    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();
    pit_ms_init(IMAGE_TIME_PIT, 1U);

    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);
    ipc_remote_param_core1_init();
    image_data_clear(&image_data[Front]);
    image_data_clear(&image_data[Center]);
    image_data_clear(&image_data[Back]);
    image_frame_meta_clear(&image_frame_meta[Front], Front);
    image_frame_meta_clear(&image_frame_meta[Center], Center);
    image_frame_meta_clear(&image_frame_meta[Back], Back);
    memset((void *)&g_image_sync_diag, 0, sizeof(g_image_sync_diag));
    image_down_init();
    image_down_horizon_init();
    CameraSpi_Init();
    CarLampCrossCheck_Init(Center);
    ImageDebugScreen_Init();
    pit_ms_init(IMAGE_PIT, 10U);
    while(true)
    {
        /* 已启动的SPI硬件事务优先完成，禁止下摄整帧处理把前摄事务拖延一帧。 */
        if(CameraSpi_Service() != 0U)
        {
            continue;
        }

        if(image_task_pending != 0U)
        {
            image_frame_updated = Get_Image_data();
            ipc_remote_param_core1_poll();
            ipc_image_publish();
            ImageDebugScreen_Update(image_frame_updated);
            image_task_pending = 0U;
            continue;
        }

        if(g_image_tick_100hz > 0U)
        {
            g_image_tick_100hz = 0U;
            ImageDebugScreen_Tick10ms();
            ipc_remote_param_core1_poll();
            CameraSpi_Update();
            image_task_pending = 1U;
        }
    }
}
