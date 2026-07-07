#include "yaw_align.h"
#include "fc_loop.h"
#include "../Estimation/Attitude/IMU_TOP.h"
#include "../Image/image_data.h"
#include <math.h>

typedef struct
{
    uint8 valid;
    image_camera_e camera;
    float x;
    float y;
    float area;
} yaw_align_beacon_t;

static const float s_yaw_align_deadband_px = 3.0f;
static const float s_yaw_align_center_turn_x_px = 50.0f;
static const float s_yaw_align_x_to_deg = 0.5f;
static const float s_yaw_align_near_x_to_deg = 0.25f;
static const float s_yaw_align_near_y_px = 20.0f;
static const float s_yaw_align_max_delta_deg = 45.0f;
static const float s_yaw_align_near_max_delta_deg = 25.0f;
static const float s_yaw_align_jump_gate_px = 20.0f;
static const float s_yaw_align_yaw_to_x_px_per_deg = 1.0f;
static const float s_yaw_align_lamp_gate_y_px = 30.0f;
static const float s_yaw_align_lamp_gate_dist_px = 38.0f;
static const float s_yaw_align_lamp_gate_min_dx_px = 10.0f;
static const float s_yaw_align_lamp_gate_min_dy_px = -5.0f;
static const uint8 s_yaw_align_stable_frames = 5U;
static const uint8 s_yaw_align_lost_reset_frames = 20U;

static yaw_align_beacon_t s_locked_beacon;
static yaw_align_beacon_t s_candidate_beacon;
static float s_locked_yaw = 0.0f;
static float s_candidate_yaw = 0.0f;
static uint8 s_candidate_frames = 0U;
static uint8 s_locked = 0U;
static uint8 s_lost_frames = 0U;
static uint8 s_center_turn_active = 0U;
static float s_center_turn_target_yaw = 0.0f;

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

static float YawAlign_CompensatedDistanceSq(const yaw_align_beacon_t *ref_beacon,
                                            float ref_yaw,
                                            float x,
                                            float y)
{
    float yaw_delta = YawAlign_Wrap180Deg(g_euler.yaw - ref_yaw);
    float predicted_x = ref_beacon->x -
                        yaw_delta * s_yaw_align_yaw_to_x_px_per_deg;

    return YawAlign_DistanceSq(x, y, predicted_x, ref_beacon->y);
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

static uint8 YawAlign_FindLargestBeacon(yaw_align_beacon_t *out)
{
    image_camera_e cameras[2] = {Front, Back};
    uint8 found = 0U;
    uint8 camera_index;
    uint8 i;

    for(camera_index = 0U; camera_index < 2U; camera_index++)
    {
        image_camera_e camera = cameras[camera_index];

        for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
        {
            const beacon_data *beacon = &image_data[camera].beacon_data[i];

            if((beacon->valid == 0U) || (beacon->area <= 0.0f))
            {
                continue;
            }

            if(YawAlign_IsNearCarLamp(camera, beacon) != 0U)
            {
                continue;
            }

            if((found == 0U) || (beacon->area > out->area))
            {
                out->valid = 1U;
                out->camera = camera;
                out->x = beacon->x;
                out->y = beacon->y;
                out->area = beacon->area;
                found = 1U;
            }
        }
    }

    return found;
}

static uint8 YawAlign_FindLargestCenterBeacon(yaw_align_beacon_t *out)
{
    uint8 found = 0U;
    uint8 i;

    for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
    {
        const beacon_data *beacon = &image_data[Center].beacon_data[i];

        if((beacon->valid == 0U) || (beacon->area <= 0.0f))
        {
            continue;
        }

        if((found == 0U) || (beacon->area > out->area))
        {
            out->valid = 1U;
            out->camera = Center;
            out->x = beacon->x;
            out->y = beacon->y;
            out->area = beacon->area;
            found = 1U;
        }
    }

    return found;
}

static uint8 YawAlign_FindNearestLockedBeacon(yaw_align_beacon_t *out)
{
    uint8 found = 0U;
    uint8 i;
    float best_dist_sq = s_yaw_align_jump_gate_px * s_yaw_align_jump_gate_px;

    for(i = 0U; i < IMAGE_MAX_BEACON_COUNT; i++)
    {
        const beacon_data *beacon = &image_data[s_locked_beacon.camera].beacon_data[i];
        float dist_sq;

        if((beacon->valid == 0U) || (beacon->area <= 0.0f))
        {
            continue;
        }

        dist_sq = YawAlign_CompensatedDistanceSq(&s_locked_beacon,
                                                 s_locked_yaw,
                                                 beacon->x,
                                                 beacon->y);
        if(dist_sq > best_dist_sq)
        {
            continue;
        }

        if((found == 0U) || (dist_sq < best_dist_sq))
        {
            out->valid = 1U;
            out->camera = s_locked_beacon.camera;
            out->x = beacon->x;
            out->y = beacon->y;
            out->area = beacon->area;
            best_dist_sq = dist_sq;
            found = 1U;
        }
    }

    return found;
}

static void YawAlign_UpdateCandidate(const yaw_align_beacon_t *beacon)
{
    float jump_gate_sq = s_yaw_align_jump_gate_px * s_yaw_align_jump_gate_px;

    if((s_candidate_frames == 0U) ||
       (s_candidate_beacon.camera != beacon->camera) ||
       (YawAlign_CompensatedDistanceSq(&s_candidate_beacon,
                                       s_candidate_yaw,
                                       beacon->x,
                                       beacon->y) > jump_gate_sq))
    {
        s_candidate_beacon = *beacon;
        s_candidate_yaw = g_euler.yaw;
        s_candidate_frames = 1U;
        return;
    }

    s_candidate_beacon = *beacon;
    s_candidate_yaw = g_euler.yaw;
    if(s_candidate_frames < s_yaw_align_stable_frames)
    {
        s_candidate_frames++;
    }

    if(s_candidate_frames >= s_yaw_align_stable_frames)
    {
        s_locked_beacon = s_candidate_beacon;
        s_locked_yaw = s_candidate_yaw;
        s_locked = 1U;
        s_lost_frames = 0U;
    }
}

void YawAlign_Reset(void)
{
    s_locked_beacon.valid = 0U;
    s_candidate_beacon.valid = 0U;
    s_locked_yaw = 0.0f;
    s_candidate_yaw = 0.0f;
    s_candidate_frames = 0U;
    s_locked = 0U;
    s_lost_frames = 0U;
    s_center_turn_active = 0U;
    s_center_turn_target_yaw = 0.0f;
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

static float YawAlign_GetYawDelta(const yaw_align_beacon_t *beacon)
{
    float x_to_deg = s_yaw_align_x_to_deg;
    float max_delta_deg = s_yaw_align_max_delta_deg;

    if(beacon->y > s_yaw_align_near_y_px)
    {
        x_to_deg = s_yaw_align_near_x_to_deg;
        max_delta_deg = s_yaw_align_near_max_delta_deg;
    }

    return YawAlign_Clamp(beacon->x * x_to_deg,
                          -max_delta_deg,
                          max_delta_deg);
}

uint8 YawAlign_Update(void)
{
    yaw_align_beacon_t beacon;
    float yaw_delta;

    beacon.valid = 0U;

    if(s_locked == 0U)
    {
        if(YawAlign_FindLargestBeacon(&beacon) != 0U)
        {
            s_center_turn_active = 0U;
            s_center_turn_target_yaw = 0.0f;
            YawAlign_UpdateCandidate(&beacon);
            YawAlign_HoldCurrentYaw();
            return 0U;
        }

        if(s_center_turn_active == 0U)
        {
            if((YawAlign_FindLargestCenterBeacon(&beacon) == 0U) ||
               (fabsf(beacon.x) <= s_yaw_align_center_turn_x_px))
            {
                YawAlign_Reset();
                YawAlign_HoldCurrentYaw();
                return 0U;
            }

            YawAlign_Reset();
            s_center_turn_target_yaw = g_euler.yaw +
                                       ((beacon.x > 0.0f) ? 90.0f : -90.0f);
            s_center_turn_active = 1U;
        }

        yaw_angle_target = s_center_turn_target_yaw;
        return 0U;
    }

    if(YawAlign_FindNearestLockedBeacon(&beacon) == 0U)
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

    s_locked_beacon = beacon;
    s_locked_yaw = g_euler.yaw;
    s_lost_frames = 0U;

    if(fabsf(beacon.x) <= s_yaw_align_deadband_px)
    {
        YawAlign_HoldCurrentYaw();
        return 1U;
    }

    yaw_delta = YawAlign_GetYawDelta(&beacon);
    yaw_angle_target = g_euler.yaw + yaw_delta;
    return 1U;
}
