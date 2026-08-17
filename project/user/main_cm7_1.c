#include "zf_common_headfile.h"
#include "Estimation/Pos_Est/image_down.h"
#include "Display/image_debug_screen.h"
#include "Image/image_down_horizon.h"
#include "Protocols/CameraSpi/camera_spi.h"
#include "Protocols/AirComm/air_comm_air.h"

#define IMAGE_PIT                          (PIT_CH10)
#define IMAGE_TIME_PIT                     (PIT_CH11)
#define IMAGE_CORE1_TASK_PERIOD_US         CAMERA_SPI_UPDATE_PERIOD_US
#define IMAGE_ATTITUDE_TIMEOUT_US          (50000U)
#define IMAGE_ATTITUDE_TIMEOUT_TICKS \
    ((IMAGE_ATTITUDE_TIMEOUT_US + IMAGE_CORE1_TASK_PERIOD_US - 1U) / \
     IMAGE_CORE1_TASK_PERIOD_US)
#define IMAGE_SCREEN_TICK_PERIOD_US        (10000U)
#define IMAGE_SCREEN_TICK_DIVIDER \
    ((IMAGE_SCREEN_TICK_PERIOD_US + IMAGE_CORE1_TASK_PERIOD_US - 1U) / \
     IMAGE_CORE1_TASK_PERIOD_US)

/* Air侧仅负责候选关联和输出槽位重排，不参与图像检测。 */
#define AIR_BEACON0_MATCH_DISTANCE_SQ       (18.0f * 18.0f)
#define AIR_BEACON0_NEW_TARGET_DISTANCE_SQ  (36.0f * 36.0f)
#define AIR_BEACON0_SWITCH_AREA_RATIO       1.45f
#define AIR_BEACON0_CONFIRM_FRAMES          2U
#define AIR_BEACON0_MAX_MISSES              3U

typedef struct
{
    uint8 active;
    uint8 confirmed;
    uint8 hits;
    uint8 misses;
    float x;
    float y;
    float vx;
    float vy;
    float area;
} air_beacon0_track_t;

static air_beacon0_track_t s_air_beacon0_track[IMAGE_CAMERA_COUNT];

static float air_beacon0_distance_sq(float ax, float ay, float bx, float by)
{
    float dx = ax - bx;
    float dy = ay - by;
    return dx * dx + dy * dy;
}

static void air_beacon0_reset_track(air_beacon0_track_t *track)
{
    memset(track, 0, sizeof(*track));
}

static void air_beacon0_start_track(air_beacon0_track_t *track,
                                    const beacon_data *beacon)
{
    air_beacon0_reset_track(track);
    track->active = 1U;
    track->hits = 1U;
    track->x = beacon->x;
    track->y = beacon->y;
    track->area = beacon->area;
}

static int air_beacon0_nearest(const struct image_data *data,
                               float x,
                               float y)
{
    uint8 i;
    int best = -1;
    float best_distance = AIR_BEACON0_MATCH_DISTANCE_SQ;

    for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
    {
        const beacon_data *beacon = &data->beacon_data[i];
        float distance;

        if(image_data_beacon_valid(beacon) == 0U)
        {
            continue;
        }
        distance = air_beacon0_distance_sq(beacon->x, beacon->y, x, y);
        if(distance <= best_distance)
        {
            best_distance = distance;
            best = (int)i;
        }
    }
    return best;
}

static void air_beacon0_output_track(struct image_data *data,
                                     air_beacon0_track_t *track,
                                     int matched)
{
    beacon_data output;
    uint8 i;

    output.valid = 1U;
    output.x = track->x;
    output.y = track->y;
    output.area = track->area;

    if(matched > 0)
    {
        /* 先移除关联候选，再把历史轨迹插入0号槽，保持其余候选相对顺序。 */
        for(i = (uint8)matched; i + 1U < IMAGE_MAX_BEACON_COUNT; i++)
        {
            data->beacon_data[i] = data->beacon_data[i + 1U];
        }
        for(i = IMAGE_MAX_BEACON_COUNT - 1U; i > 0U; i--)
        {
            data->beacon_data[i] = data->beacon_data[i - 1U];
        }
        data->beacon_data[0] = output;
    }
    else
    {
        data->beacon_data[0] = output;
    }
}

static void air_beacon0_stabilize_camera(struct image_data *data,
                                         air_beacon0_track_t *track)
{
    int selected = -1;
    uint8 i;

    if(data == NULL || track == NULL)
    {
        return;
    }

    if(track->confirmed != 0U)
    {
        float predict_x = track->x + track->vx;
        float predict_y = track->y + track->vy;
        selected = air_beacon0_nearest(data, predict_x, predict_y);
        if(selected > 0 &&
           data->beacon_data[0].area >
               data->beacon_data[selected].area * AIR_BEACON0_SWITCH_AREA_RATIO)
        {
            selected = 0;
        }
        if(selected < 0)
        {
            if(image_data_beacon_valid(&data->beacon_data[0]) != 0U &&
               air_beacon0_distance_sq(predict_x, predict_y,
                                       data->beacon_data[0].x,
                                       data->beacon_data[0].y) >
                   AIR_BEACON0_NEW_TARGET_DISTANCE_SQ)
            {
                air_beacon0_start_track(track, &data->beacon_data[0]);
                return;
            }
        }
    }
    else
    {
        for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
        {
            if(image_data_beacon_valid(&data->beacon_data[i]) != 0U)
            {
                selected = (int)i;
                break;
            }
        }
    }

    if(selected < 0)
    {
        if(track->confirmed != 0U && track->misses < AIR_BEACON0_MAX_MISSES)
        {
            track->x += track->vx;
            track->y += track->vy;
            track->misses++;
            air_beacon0_output_track(data, track, -1);
        }
        else
        {
            air_beacon0_reset_track(track);
        }
        return;
    }

    {
        beacon_data *measurement = &data->beacon_data[selected];
        float old_x = track->x;
        float old_y = track->y;

        if(track->active == 0U)
        {
            air_beacon0_start_track(track, measurement);
            return;
        }
        if(air_beacon0_distance_sq(track->x + track->vx,
                                   track->y + track->vy,
                                   measurement->x,
                                   measurement->y) >
           AIR_BEACON0_NEW_TARGET_DISTANCE_SQ)
        {
            air_beacon0_start_track(track, measurement);
            return;
        }

        track->vx = 0.70f * track->vx + 0.30f * (measurement->x - old_x);
        track->vy = 0.70f * track->vy + 0.30f * (measurement->y - old_y);
        track->x = 0.65f * measurement->x + 0.35f * (track->x + track->vx);
        track->y = 0.65f * measurement->y + 0.35f * (track->y + track->vy);
        track->area = measurement->area;
        track->misses = 0U;
        if(track->confirmed == 0U)
        {
            if(track->hits < 255U)
            {
                track->hits++;
            }
            if(track->hits >= AIR_BEACON0_CONFIRM_FRAMES)
            {
                track->confirmed = 1U;
            }
        }
        if(track->confirmed != 0U)
        {
            air_beacon0_output_track(data, track, selected);
        }
    }
}

static void air_beacon0_stabilize(uint8 fresh_mask)
{
    uint8 camera;

    for(camera = Front; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        if((fresh_mask & (uint8)(1U << camera)) != 0U)
        {
            air_beacon0_stabilize_camera(&image_data[camera],
                                         &s_air_beacon0_track[camera]);
        }
    }
}

volatile uint8 g_image_tick_100hz = 0U;
volatile uint32 g_image_core1_tick_generated;
volatile uint32 g_image_core1_tick_overflow_count;
volatile uint32 g_image_master_time_ms = 0U; /* 核心1统一毫秒时间。 */

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
        if((c1_horizon_enable != 0) &&
           (attitude_valid != 0U) && (height_valid != 0U))
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
    air_beacon0_stabilize((uint8)(camera_spi_fresh_mask | changed_mask));
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
    pit_ms_init(IMAGE_TIME_PIT, 1U);

    ipc_communicate_init(IPC_PORT_2, ipc_image_callback);
    ipc_remote_param_core1_init();
    ipc_image_init();
    image_frame_meta_clear(&image_frame_meta[Front], Front);
    image_frame_meta_clear(&image_frame_meta[Center], Center);
    image_frame_meta_clear(&image_frame_meta[Back], Back);
    memset((void *)&g_image_sync_diag, 0, sizeof(g_image_sync_diag));
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
