#include "beacon_lost_detector.h"

#include <string.h>

#include "../Estimation/Height_Est/Height_Est.h"
#include "../HW_Drivers/Beep/Beep.h"

#define BEACON_LOST_HISTORY_SIZE       (20U)
#define BEACON_LOST_NORMAL_HAS_COUNT   (5U)
#define BEACON_LOST_NORMAL_LOST_COUNT  (5U)
#define BEACON_LOST_NORMAL_PATTERN_SIZE \
    (BEACON_LOST_NORMAL_HAS_COUNT + BEACON_LOST_NORMAL_LOST_COUNT)
#define BEACON_LOST_LAMP_LOST_MIN_HISTORY (6U)
#define BEACON_LOST_MIN_TOF_HEIGHT_MM  (700.0f)
#define BEACON_LOST_DISAPPEAR_RADIUS_SQ \
    (BEACON_LOST_DISAPPEAR_RADIUS_PX * BEACON_LOST_DISAPPEAR_RADIUS_PX)
#define BEACON_LOST_BEEP_DUTY_PERCENT  (100U)
#define BEACON_LOST_BEEP_TIME_S        (0.50f)
#define BEACON_LOST_BEEP_COUNT         (1U)
#define BEACON_LOST_SCAN_COUNT         (2U)
#define BEACON_LOST_CENTER_MIN_AREA    (15.0f)

typedef struct
{
    uint8 valid;
    float x;
    float y;
} beacon_lost_point_t;

typedef struct
{
    uint8 lamp_valid;
    uint8 in_range;
    float lost_x;
    float lost_y;
    uint8 beacon_count;
    beacon_lost_point_t beacon[IMAGE_MAX_BEACON_COUNT];
} beacon_lost_frame_t;

uint8 g_beacon_lost_flag;

static beacon_lost_frame_t s_history[IMAGE_CAMERA_COUNT][BEACON_LOST_HISTORY_SIZE];
static uint8 s_write_index;
static uint8 s_history_count;
static uint8 s_last_lost_flag;

static uint8 BeaconLostDetector_HeightAllowed(void)
{
    return (g_tof_fused_height_mm >= BEACON_LOST_MIN_TOF_HEIGHT_MM) ? 1U : 0U;
}

static uint8 BeaconLostDetector_BeaconAllowed(image_camera_e camera,
                                              const beacon_data *beacon)
{
    if(image_data_beacon_valid(beacon) == 0U)
    {
        return 0U;
    }

    if((camera == Center) && (beacon->area <= BEACON_LOST_CENTER_MIN_AREA))
    {
        return 0U;
    }

    return 1U;
}

static uint8 BeaconLostDetector_BeaconInSquare(const beacon_data *beacon,
                                               const car_lamp_data *lamp,
                                               image_camera_e camera)
{
    float up_px = BEACON_LOST_SQUARE_UP_PX;
    float down_px = BEACON_LOST_SQUARE_DOWN_PX;

    if((beacon == 0) || (lamp == 0) ||
       (BeaconLostDetector_BeaconAllowed(camera, beacon) == 0U) ||
       (image_data_car_lamp_valid(lamp) == 0U))
    {
        return 0U;
    }

    if(camera == Back)
    {
        up_px = BEACON_LOST_SQUARE_DOWN_PX;
        down_px = BEACON_LOST_SQUARE_UP_PX;
    }

    return ((beacon->x >= (lamp->cx - BEACON_LOST_SQUARE_LEFT_PX)) &&
            (beacon->x <= (lamp->cx + BEACON_LOST_SQUARE_RIGHT_PX)) &&
            (beacon->y >= (lamp->cy - up_px)) &&
            (beacon->y <= (lamp->cy + down_px))) ? 1U : 0U;
}

static void BeaconLostDetector_CaptureFrame(image_camera_e camera,
                                            beacon_lost_frame_t *out)
{
    const struct image_data *frame = &image_data[camera];
    const car_lamp_data *lamp = &frame->car_lamp_data[0];
    uint8 i;

    memset(out, 0, sizeof(*out));
    out->lamp_valid = image_data_car_lamp_valid(lamp);

    for(i = 0U; (i < IMAGE_MAX_BEACON_COUNT) && (i < BEACON_LOST_SCAN_COUNT); i++)
    {
        const beacon_data *beacon = &frame->beacon_data[i];

        if(BeaconLostDetector_BeaconAllowed(camera, beacon) == 0U)
        {
            continue;
        }

        if(out->beacon_count < IMAGE_MAX_BEACON_COUNT)
        {
            beacon_lost_point_t *point = &out->beacon[out->beacon_count];
            point->valid = 1U;
            point->x = beacon->x;
            point->y = beacon->y;
            out->beacon_count++;
        }

        if((out->in_range == 0U) &&
           (BeaconLostDetector_BeaconInSquare(beacon, lamp, camera) != 0U))
        {
            out->in_range = 1U;
            out->lost_x = beacon->x;
            out->lost_y = beacon->y;
        }
    }
}

static uint8 BeaconLostDetector_FrameHasBeaconNearPoint(
    const beacon_lost_frame_t *frame,
    float x,
    float y)
{
    uint8 i;

    if(frame == 0)
    {
        return 0U;
    }

    for(i = 0U; i < frame->beacon_count; i++)
    {
        const beacon_lost_point_t *point = &frame->beacon[i];
        float dx;
        float dy;

        if(point->valid == 0U)
        {
            continue;
        }

        dx = point->x - x;
        dy = point->y - y;
        if((dx * dx + dy * dy) <= BEACON_LOST_DISAPPEAR_RADIUS_SQ)
        {
            return 1U;
        }
    }

    return 0U;
}

static const beacon_lost_frame_t *BeaconLostDetector_HistoryAt(
    image_camera_e camera,
    uint8 age)
{
    uint8 index;

    if((camera >= IMAGE_CAMERA_COUNT) ||
       (age >= s_history_count) ||
       (age >= BEACON_LOST_HISTORY_SIZE))
    {
        return 0;
    }

    index = (uint8)((s_write_index + BEACON_LOST_HISTORY_SIZE - age) %
                    BEACON_LOST_HISTORY_SIZE);
    return &s_history[camera][index];
}

static uint8 BeaconLostDetector_FrameHasDisappearedPoint(
    const beacon_lost_frame_t *lost_frame,
    const beacon_lost_frame_t *first_lamp_frame,
    const beacon_lost_frame_t *second_lamp_frame,
    const beacon_lost_frame_t *third_lamp_frame)
{
    uint8 i;

    if((lost_frame == 0) || (first_lamp_frame == 0) ||
       (second_lamp_frame == 0) || (third_lamp_frame == 0))
    {
        return 0U;
    }

    for(i = 0U; i < lost_frame->beacon_count; i++)
    {
        const beacon_lost_point_t *point = &lost_frame->beacon[i];

        if(point->valid == 0U)
        {
            continue;
        }

        if((BeaconLostDetector_FrameHasBeaconNearPoint(first_lamp_frame,
                                                       point->x,
                                                       point->y) == 0U) &&
           (BeaconLostDetector_FrameHasBeaconNearPoint(second_lamp_frame,
                                                       point->x,
                                                       point->y) == 0U) &&
           (BeaconLostDetector_FrameHasBeaconNearPoint(third_lamp_frame,
                                                       point->x,
                                                       point->y) == 0U))
        {
            return 1U;
        }
    }

    return 0U;
}

static uint8 BeaconLostDetector_MatchNormalLostPattern(image_camera_e camera)
{
    const beacon_lost_frame_t *frame[BEACON_LOST_NORMAL_PATTERN_SIZE];
    const beacon_lost_frame_t *lost_frame;
    float lost_x;
    float lost_y;
    uint8 i;

    if(s_history_count < BEACON_LOST_NORMAL_PATTERN_SIZE)
    {
        return 0U;
    }

    for(i = 0U; i < BEACON_LOST_NORMAL_PATTERN_SIZE; i++)
    {
        frame[i] = BeaconLostDetector_HistoryAt(
            camera,
            (uint8)(BEACON_LOST_NORMAL_PATTERN_SIZE - 1U - i));
        if(frame[i] == 0)
        {
            return 0U;
        }
    }

    for(i = 0U; i < BEACON_LOST_NORMAL_HAS_COUNT; i++)
    {
        if(frame[i]->in_range == 0U)
        {
            return 0U;
        }
    }

    for(i = BEACON_LOST_NORMAL_HAS_COUNT; i < BEACON_LOST_NORMAL_PATTERN_SIZE; i++)
    {
        if((frame[i]->lamp_valid == 0U) || (frame[i]->in_range != 0U))
        {
            return 0U;
        }
    }

    lost_frame = frame[BEACON_LOST_NORMAL_HAS_COUNT - 1U];
    lost_x = lost_frame->lost_x;
    lost_y = lost_frame->lost_y;

    for(i = BEACON_LOST_NORMAL_HAS_COUNT; i < BEACON_LOST_NORMAL_PATTERN_SIZE; i++)
    {
        if(BeaconLostDetector_FrameHasBeaconNearPoint(frame[i], lost_x, lost_y) != 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8 BeaconLostDetector_MatchLampLostPattern(image_camera_e camera)
{
    uint8 lost_age;

    if(s_history_count <= BEACON_LOST_LAMP_LOST_MIN_HISTORY)
    {
        return 0U;
    }

    for(lost_age = 2U; lost_age < s_history_count; lost_age++)
    {
        const beacon_lost_frame_t *lost_frame;
        const beacon_lost_frame_t *before_1;
        const beacon_lost_frame_t *before_2;
        const beacon_lost_frame_t *before_3;
        const beacon_lost_frame_t *first_lamp_frame = 0;
        const beacon_lost_frame_t *second_lamp_frame = 0;
        const beacon_lost_frame_t *third_lamp_frame = 0;
        uint8 continuous_lamp_count = 0U;
        uint8 recovery_has_beacon = 0U;
        uint8 age;

        if((uint8)(lost_age + 3U) >= s_history_count)
        {
            break;
        }

        lost_frame = BeaconLostDetector_HistoryAt(camera, lost_age);
        before_1 = BeaconLostDetector_HistoryAt(camera, (uint8)(lost_age + 1U));
        before_2 = BeaconLostDetector_HistoryAt(camera, (uint8)(lost_age + 2U));
        before_3 = BeaconLostDetector_HistoryAt(camera, (uint8)(lost_age + 3U));

        if((lost_frame == 0) || (before_1 == 0) ||
           (before_2 == 0) || (before_3 == 0))
        {
            return 0U;
        }

        if((before_1->in_range == 0U) ||
           (before_2->in_range == 0U) ||
           (before_3->in_range == 0U) ||
           (lost_frame->lamp_valid != 0U) ||
           (lost_frame->beacon_count == 0U))
        {
            continue;
        }

        for(age = lost_age; age > 0U; age--)
        {
            const beacon_lost_frame_t *frame =
                BeaconLostDetector_HistoryAt(camera, (uint8)(age - 1U));

            if(frame == 0)
            {
                return 0U;
            }

            if(frame->lamp_valid == 0U)
            {
                continuous_lamp_count = 0U;
                first_lamp_frame = 0;
                second_lamp_frame = 0;
                third_lamp_frame = 0;
                continue;
            }

            if(frame->in_range != 0U)
            {
                recovery_has_beacon = 1U;
                continue;
            }

            if(continuous_lamp_count == 0U)
            {
                first_lamp_frame = frame;
                continuous_lamp_count = 1U;
            }
            else if(continuous_lamp_count == 1U)
            {
                second_lamp_frame = frame;
                continuous_lamp_count = 2U;
            }
            else
            {
                third_lamp_frame = frame;
                continuous_lamp_count = 3U;
                break;
            }
        }

        if((recovery_has_beacon != 0U) ||
           (first_lamp_frame == 0) || (second_lamp_frame == 0) ||
           (third_lamp_frame == 0) || (continuous_lamp_count < 3U))
        {
            continue;
        }

        if(BeaconLostDetector_FrameHasDisappearedPoint(lost_frame,
                                                       first_lamp_frame,
                                                       second_lamp_frame,
                                                       third_lamp_frame) != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

static uint8 BeaconLostDetector_MatchLostPattern(image_camera_e camera)
{
    if(BeaconLostDetector_MatchNormalLostPattern(camera) != 0U)
    {
        return 1U;
    }

    return BeaconLostDetector_MatchLampLostPattern(camera);
}

void BeaconLostDetector_Init(void)
{
    memset(s_history, 0, sizeof(s_history));
    g_beacon_lost_flag = 0U;
    s_write_index = 0U;
    s_history_count = 0U;
    s_last_lost_flag = 0U;
}

uint8 BeaconLostDetector_Update(void)
{
    image_camera_e camera;

    for(camera = Front; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        BeaconLostDetector_CaptureFrame(camera, &s_history[camera][s_write_index]);
    }

    if(s_history_count < BEACON_LOST_HISTORY_SIZE)
    {
        s_history_count++;
    }

    g_beacon_lost_flag = 0U;
    for(camera = Front; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        if(BeaconLostDetector_MatchLostPattern(camera) != 0U)
        {
            g_beacon_lost_flag = 1U;
        }
    }

    if(BeaconLostDetector_HeightAllowed() == 0U)
    {
        g_beacon_lost_flag = 0U;
    }

    if((g_beacon_lost_flag != 0U) && (s_last_lost_flag == 0U))
    {
        Beep_Play(BEACON_LOST_BEEP_DUTY_PERCENT,
                  BEACON_LOST_BEEP_TIME_S,
                  BEACON_LOST_BEEP_COUNT);
    }
    s_last_lost_flag = g_beacon_lost_flag;

    s_write_index++;
    if(s_write_index >= BEACON_LOST_HISTORY_SIZE)
    {
        s_write_index = 0U;
    }

    return g_beacon_lost_flag;
}

uint8 BeaconLostDetector_GetFlag(void)
{
    if(BeaconLostDetector_HeightAllowed() == 0U)
    {
        return 0U;
    }

    return g_beacon_lost_flag;
}
