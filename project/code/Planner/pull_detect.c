#include "pull_detect.h"

#include <math.h>
#include <string.h>

#define PULL_DETECT_DISTANCE_LPF_ALPHA       (0.105573f)
#define PULL_DETECT_SPEED_LPF_ALPHA          (0.105573f)
#define PULL_DETECT_HISTORY_COUNT             (20U)
#define PULL_DETECT_HISTORY_DT_S              (0.2f)
#define PULL_DETECT_PREDICT_TIME_S            (0.3f)
#define PULL_DETECT_WARN_SCORE_CM            (66.0f)
#define PULL_DETECT_HYSTERESIS_CM             (5.0f)
#define PULL_DETECT_ON_TICKS                   (4U)
#define PULL_DETECT_OFF_TICKS                  (10U)

typedef struct
{
    float distance_lpf_cm;
    float separation_speed_lpf_cmps;
    float distance_history_cm[PULL_DETECT_HISTORY_COUNT];
    uint8 history_index;
    uint8 initialized;
    uint8 danger_ticks;
    uint8 safe_ticks;
} pull_detect_state_t;

pull_detect_result_t g_pull_detect_result;

static pull_detect_state_t s_pull_detect_state;

void PullDetect_Init(void)
{
    memset(&s_pull_detect_state, 0, sizeof(s_pull_detect_state));
    g_pull_detect_result.danger = 0U;
}

void PullDetect_Update100Hz(uint8 image_valid, float x_cm, float y_cm)
{
    float distance_cm;
    float delayed_distance_cm;
    float separation_speed_cmps;
    float positive_separation_speed_cmps;
    float risk_score_cm;
    uint8 risk;
    uint8 safe;
    uint8 i;

    if (image_valid == 0U)
    {
        PullDetect_Init();
        return;
    }

    distance_cm = sqrtf(x_cm * x_cm + y_cm * y_cm);
    if (s_pull_detect_state.initialized == 0U)
    {
        s_pull_detect_state.distance_lpf_cm = distance_cm;
        for (i = 0U; i < PULL_DETECT_HISTORY_COUNT; i++)
        {
            s_pull_detect_state.distance_history_cm[i] = distance_cm;
        }
        s_pull_detect_state.initialized = 1U;
    }
    else
    {
        s_pull_detect_state.distance_lpf_cm +=
            PULL_DETECT_DISTANCE_LPF_ALPHA *
            (distance_cm - s_pull_detect_state.distance_lpf_cm);
        delayed_distance_cm =
            s_pull_detect_state.distance_history_cm[s_pull_detect_state.history_index];
        s_pull_detect_state.distance_history_cm[s_pull_detect_state.history_index] =
            s_pull_detect_state.distance_lpf_cm;
        s_pull_detect_state.history_index++;
        if (s_pull_detect_state.history_index >= PULL_DETECT_HISTORY_COUNT)
        {
            s_pull_detect_state.history_index = 0U;
        }

        separation_speed_cmps =
            (s_pull_detect_state.distance_lpf_cm - delayed_distance_cm) /
            PULL_DETECT_HISTORY_DT_S;
        s_pull_detect_state.separation_speed_lpf_cmps +=
            PULL_DETECT_SPEED_LPF_ALPHA *
            (separation_speed_cmps - s_pull_detect_state.separation_speed_lpf_cmps);
    }

    positive_separation_speed_cmps = s_pull_detect_state.separation_speed_lpf_cmps;
    if (positive_separation_speed_cmps < 0.0f)
    {
        positive_separation_speed_cmps = 0.0f;
    }
    risk_score_cm =
        s_pull_detect_state.distance_lpf_cm +
        PULL_DETECT_PREDICT_TIME_S * positive_separation_speed_cmps;
    risk = (risk_score_cm >= PULL_DETECT_WARN_SCORE_CM) ? 1U : 0U;
    safe = (risk_score_cm <
            (PULL_DETECT_WARN_SCORE_CM - PULL_DETECT_HYSTERESIS_CM)) ? 1U : 0U;

    if (g_pull_detect_result.danger == 0U)
    {
        s_pull_detect_state.danger_ticks =
            (risk != 0U) ? (uint8)(s_pull_detect_state.danger_ticks + 1U) : 0U;
        if (s_pull_detect_state.danger_ticks >= PULL_DETECT_ON_TICKS)
        {
            g_pull_detect_result.danger = 1U;
            s_pull_detect_state.safe_ticks = 0U;
        }
    }
    else
    {
        s_pull_detect_state.safe_ticks =
            (safe != 0U) ? (uint8)(s_pull_detect_state.safe_ticks + 1U) : 0U;
        if (s_pull_detect_state.safe_ticks >= PULL_DETECT_OFF_TICKS)
        {
            g_pull_detect_result.danger = 0U;
            s_pull_detect_state.danger_ticks = 0U;
        }
    }
}
