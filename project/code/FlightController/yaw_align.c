#include "yaw_align.h"
#include "fc_loop.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../Image/image_data.h"
#include <math.h>

typedef struct
{
    uint8 valid;
    image_camera_e camera;
    uint8 mode;
    float x;
    float y;
    float area;
    float error_px;
    float yaw_delta_deg;
    float canonical_yaw;
    float score_deg;
} yaw_align_beacon_t;

static const float s_yaw_align_deadband_px = 5.0f;
static const float s_yaw_align_x_to_deg = 0.5f;
static const float s_yaw_align_center_y_to_deg = 0.5f;
static const float s_yaw_align_max_delta_deg = 25.0f;
static const float s_yaw_align_center_side_x_px = 20.0f;
static const float s_yaw_align_same_beacon_gate_deg = 25.0f;
static const float s_yaw_align_switch_margin_deg = 25.0f;
static const float s_yaw_align_lamp_gate_y_px = 30.0f;
static const float s_yaw_align_lamp_gate_dist_px = 38.0f;
static const float s_yaw_align_lamp_gate_min_dx_px = 10.0f;
static const float s_yaw_align_lamp_gate_min_dy_px = -5.0f;
static const uint8 s_yaw_align_stable_frames = 5U;
static const uint8 s_yaw_align_switch_frames = 8U;
static const uint8 s_yaw_align_lost_reset_frames = 80U;

#define YAW_ALIGN_MODE_FRONT        (0U)
#define YAW_ALIGN_MODE_CENTER_LEFT  (1U)
#define YAW_ALIGN_MODE_BACK         (2U)
#define YAW_ALIGN_MODE_CENTER_RIGHT (3U)

static yaw_align_beacon_t s_locked_beacon;
static yaw_align_beacon_t s_candidate_beacon;
static float s_locked_canonical_yaw = 0.0f;
static uint8 s_candidate_frames = 0U;
static uint8 s_locked = 0U;
static uint8 s_lost_frames = 0U;

static float YawAlign_Clamp(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}

static float YawAlign_DistanceSq(float x0, float y0, float x1, float y1)
{
    float dx = x0 - x1;
    float dy = y0 - y1;

    return dx * dx + dy * dy;
}

static float YawAlign_Wrap180Deg(float angle_deg)
{
    while(angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while(angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float YawAlign_AbsAngleDeltaDeg(float angle0_deg, float angle1_deg)
{
    return fabsf(YawAlign_Wrap180Deg(angle0_deg - angle1_deg));
}

static void YawAlign_HoldCurrentYaw(void)
{
    yaw_angle_target = g_euler.yaw;
    yaw_gyro_target = 0.0f;
    PID_Reset(&yaw_angle_pid);
    PID_Reset(&yaw_gyro_pid);
}

static uint8 YawAlign_IsNearCarLamp(image_camera_e camera, const beacon_data *beacon)
{
    uint8 i;

    if(beacon->y <= s_yaw_align_lamp_gate_y_px)
    {
        return 0U;
    }

    for(i = 0U; i < IMAGE_MAX_CAR_LAMP_COUNT; i++)
    {
        const car_lamp_data *lamp = &image_data[camera].car_lamp_data[i];
        float dx;
        float dy;

        if(lamp->valid == 0U)
        {
            continue;
        }

        dx = beacon->x - lamp->cx;
        dy = beacon->y - lamp->cy;

        if((fabsf(dx) > s_yaw_align_lamp_gate_min_dx_px) &&
           (dy > s_yaw_align_lamp_gate_min_dy_px) &&
           (YawAlign_DistanceSq(beacon->x, beacon->y,
                                lamp->cx, lamp->cy) <
            (s_yaw_align_lamp_gate_dist_px *
             s_yaw_align_lamp_gate_dist_px)))
        {
            return 1U;
        }
    }

    return 0U;
}

static uint8 YawAlign_BuildCandidate(image_camera_e camera,
                                     const yaw_align_beacon_t *raw,
                                     yaw_align_beacon_t *out)
{
    float canonical_offset_deg = 0.0f;
    float raw_yaw_delta_deg;

    *out = *raw;

    if(camera == Front)
    {
        out->mode = YAW_ALIGN_MODE_FRONT;
        out->error_px = raw->x;
        raw_yaw_delta_deg = raw->x * s_yaw_align_x_to_deg;
        canonical_offset_deg = 0.0f;
    }
    else if(camera == Back)
    {
        out->mode = YAW_ALIGN_MODE_BACK;
        out->error_px = raw->x;
        raw_yaw_delta_deg = raw->x * s_yaw_align_x_to_deg;
        canonical_offset_deg = -180.0f;
    }
    else if(camera == Center)
    {
        if(raw->x > s_yaw_align_center_side_x_px)
        {
            out->mode = YAW_ALIGN_MODE_CENTER_RIGHT;
            out->error_px = raw->y;
            raw_yaw_delta_deg = raw->y * s_yaw_align_center_y_to_deg;
            canonical_offset_deg = 90.0f;
        }
        else if(raw->x < -s_yaw_align_center_side_x_px)
        {
            out->mode = YAW_ALIGN_MODE_CENTER_LEFT;
            out->error_px = raw->y;
            raw_yaw_delta_deg = -raw->y * s_yaw_align_center_y_to_deg;
            canonical_offset_deg = -90.0f;
        }
        else
        {
            return 0U;
        }
    }
    else
    {
        return 0U;
    }

    out->yaw_delta_deg = YawAlign_Clamp(raw_yaw_delta_deg,
                                        -s_yaw_align_max_delta_deg,
                                        s_yaw_align_max_delta_deg);
    out->canonical_yaw = YawAlign_Wrap180Deg(g_euler.yaw +
                                             raw_yaw_delta_deg +
                                             canonical_offset_deg);
    out->score_deg = fabsf(raw_yaw_delta_deg);
    return 1U;
}

static uint8 YawAlign_BuildImageCandidate(image_camera_e camera,
                                          const beacon_data *beacon,
                                          yaw_align_beacon_t *candidate)
{
    yaw_align_beacon_t raw;

    if((beacon->valid == 0U) || (beacon->area <= 0.0f))
    {
        return 0U;
    }

    if(YawAlign_IsNearCarLamp(camera, beacon) != 0U)
    {
        return 0U;
    }

    raw.valid = 1U;
    raw.camera = camera;
    raw.x = beacon->x;
    raw.y = beacon->y;
    raw.area = beacon->area;

    return YawAlign_BuildCandidate(camera, &raw, candidate);
}

static void YawAlign_ChooseBetterCandidate(const yaw_align_beacon_t *candidate,
                                           yaw_align_beacon_t *best,
                                           uint8 *found)
{
    if((*found == 0U) || (candidate->score_deg < best->score_deg))
    {
        *best = *candidate;
        *found = 1U;
    }
}

static uint8 YawAlign_FindBestCandidate(yaw_align_beacon_t *best)
{
    image_camera_e cameras[3] = {Front, Center, Back};
    uint8 camera_index;
    uint8 found = 0U;

    for(camera_index = 0U; camera_index < 3U; camera_index++)
    {
        yaw_align_beacon_t candidate;
        image_camera_e camera = cameras[camera_index];
        uint8 i;

        for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
        {
            const beacon_data *beacon = &image_data[camera].beacon_data[i];

            if(YawAlign_BuildImageCandidate(camera, beacon, &candidate) == 0U)
            {
                continue;
            }

            YawAlign_ChooseBetterCandidate(&candidate, best, &found);
        }
    }

    return found;
}

static uint8 YawAlign_FindLockedCandidates(yaw_align_beacon_t *best,
                                           yaw_align_beacon_t *current)
{
    image_camera_e cameras[3] = {Front, Center, Back};
    uint8 camera_index;
    uint8 best_found = 0U;
    uint8 current_found = 0U;

    for(camera_index = 0U; camera_index < 3U; camera_index++)
    {
        yaw_align_beacon_t candidate;
        image_camera_e camera = cameras[camera_index];
        uint8 i;

        for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
        {
            const beacon_data *beacon = &image_data[camera].beacon_data[i];

            if(YawAlign_BuildImageCandidate(camera, beacon, &candidate) == 0U)
            {
                continue;
            }

            if(YawAlign_AbsAngleDeltaDeg(candidate.canonical_yaw,
                                         s_locked_canonical_yaw) >
               s_yaw_align_same_beacon_gate_deg)
            {
                continue;
            }

            YawAlign_ChooseBetterCandidate(&candidate, best, &best_found);
            if(candidate.mode == s_locked_beacon.mode)
            {
                YawAlign_ChooseBetterCandidate(&candidate, current, &current_found);
            }
        }
    }

    return (best_found != 0U) ? (uint8)(1U + current_found) : 0U;
}

static uint8 YawAlign_UpdateCandidate(const yaw_align_beacon_t *beacon,
                                      uint8 stable_frames,
                                      uint8 require_same_mode)
{
    if((s_candidate_frames == 0U) ||
       ((require_same_mode != 0U) &&
        (s_candidate_beacon.mode != beacon->mode)) ||
       (YawAlign_AbsAngleDeltaDeg(s_candidate_beacon.canonical_yaw,
                                  beacon->canonical_yaw) >
        s_yaw_align_same_beacon_gate_deg))
    {
        s_candidate_beacon = *beacon;
        s_candidate_frames = 1U;
        return 0U;
    }

    s_candidate_beacon = *beacon;
    if(s_candidate_frames < stable_frames)
    {
        s_candidate_frames++;
    }

    return (s_candidate_frames >= stable_frames) ? 1U : 0U;
}

static void YawAlign_AcceptLockedBeacon(const yaw_align_beacon_t *beacon)
{
    s_locked_beacon = *beacon;
    s_locked = 1U;
    s_lost_frames = 0U;
}

static void YawAlign_AcceptInitialBeacon(const yaw_align_beacon_t *beacon)
{
    s_locked_canonical_yaw = beacon->canonical_yaw;
    YawAlign_AcceptLockedBeacon(beacon);
}

static void YawAlign_ResetCandidate(void)
{
    s_candidate_beacon.valid = 0U;
    s_candidate_frames = 0U;
}

static void YawAlign_ApplyYawTarget(const yaw_align_beacon_t *beacon)
{
    if(fabsf(beacon->error_px) <= s_yaw_align_deadband_px)
    {
        YawAlign_HoldCurrentYaw();
        return;
    }

    yaw_angle_target = g_euler.yaw + beacon->yaw_delta_deg;
}

void YawAlign_Reset(void)
{
    s_locked_beacon.valid = 0U;
    s_candidate_beacon.valid = 0U;
    s_locked_canonical_yaw = 0.0f;
    s_candidate_frames = 0U;
    s_locked = 0U;
    s_lost_frames = 0U;
}

static void YawAlign_FillDebugBeacon(const yaw_align_beacon_t *src,
                                     yaw_align_debug_beacon_t *dst)
{
    dst->valid = src->valid;
    dst->camera = (uint8)src->camera;
    dst->x = src->x;
    dst->y = src->y;
    dst->area = src->area;
}

void YawAlign_GetDebug(yaw_align_debug_t *out)
{
    if(out == 0)
    {
        return;
    }

    out->locked = s_locked;
    out->candidate_frames = s_candidate_frames;
    out->lost_frames = s_lost_frames;
    YawAlign_FillDebugBeacon(&s_locked_beacon, &out->locked_beacon);
    YawAlign_FillDebugBeacon(&s_candidate_beacon, &out->candidate_beacon);
}

uint8 YawAlign_Update(void)
{
    yaw_align_beacon_t beacon;
    yaw_align_beacon_t current_beacon;
    uint8 locked_find_result;

    beacon.valid = 0U;
    current_beacon.valid = 0U;

    if(s_locked == 0U)
    {
        if(YawAlign_FindBestCandidate(&beacon) == 0U)
        {
            YawAlign_Reset();
            YawAlign_HoldCurrentYaw();
            return 0U;
        }

        if(YawAlign_UpdateCandidate(&beacon,
                                    s_yaw_align_stable_frames,
                                    0U) != 0U)
        {
            YawAlign_AcceptInitialBeacon(&s_candidate_beacon);
        }

        YawAlign_HoldCurrentYaw();
        return 0U;
    }

    locked_find_result = YawAlign_FindLockedCandidates(&beacon,
                                                       &current_beacon);
    if(locked_find_result == 0U)
    {
        if(s_lost_frames < s_yaw_align_lost_reset_frames)
        {
            s_lost_frames++;
        }
        else
        {
            YawAlign_Reset();
        }

        YawAlign_HoldCurrentYaw();
        return 0U;
    }

    if((current_beacon.valid != 0U) &&
       ((beacon.mode == current_beacon.mode) ||
        (beacon.score_deg + s_yaw_align_switch_margin_deg >=
         current_beacon.score_deg)))
    {
        YawAlign_AcceptLockedBeacon(&current_beacon);
        YawAlign_ResetCandidate();
        YawAlign_ApplyYawTarget(&current_beacon);
        return 1U;
    }

    if(YawAlign_UpdateCandidate(&beacon,
                                s_yaw_align_switch_frames,
                                1U) == 0U)
    {
        if(current_beacon.valid != 0U)
        {
            YawAlign_AcceptLockedBeacon(&current_beacon);
            YawAlign_ApplyYawTarget(&current_beacon);
            return 1U;
        }

        YawAlign_HoldCurrentYaw();
        return 0U;
    }

    YawAlign_AcceptLockedBeacon(&s_candidate_beacon);
    YawAlign_ApplyYawTarget(&s_candidate_beacon);
    return 1U;
}
