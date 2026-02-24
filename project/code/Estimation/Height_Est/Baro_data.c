#include "Baro_data.h"

#define BARO_PRESS_LPF_ALPHA            (0.26f)
#define BARO_ALT_LPF_ALPHA_SLOW         (0.14f)
#define BARO_ALT_LPF_ALPHA_FAST         (0.35f)
#define BARO_ALT_FAST_SWITCH_M          (0.08f)
#define BARO_ALT_STEP_LIMIT_M           (0.04f)
#define BARO_ZERO_DEADBAND_M            (0.006f)
#define BARO_GROUND_ENTER_ABS_H_M       (0.20f)
#define BARO_AIRBORNE_LATCH_ABS_H_M     (0.35f)
#define BARO_GROUND_LOCK_FRAMES         (50U)
#define BARO_REF_ADAPT_ALPHA            (0.002f)

float g_baro_ref_pressure = 0.0f;
float g_baro_altitude = 0.0f;

static uint8 s_baro_inited = 0U;
static uint8 s_baro_ref_valid = 0U;

static float s_pressure_med_buf[3] = {0.0f, 0.0f, 0.0f};
static uint8 s_pressure_med_count = 0U;
static float s_pressure_lpf_pa = 0.0f;
static uint8 s_pressure_lpf_inited = 0U;

static float s_alt_lpf_m = 0.0f;
static float s_prev_alt_raw_m = 0.0f;
static uint16 s_ground_lock_cnt = 0U;
static uint8 s_airborne_latched = 0U;

static float Baro_AbsFloat(float value)
{
    if (value >= 0.0f)
    {
        return value;
    }
    return -value;
}

static float Baro_Median3(float a, float b, float c)
{
    float temp;

    if (a > b)
    {
        temp = a;
        a = b;
        b = temp;
    }

    if (b > c)
    {
        temp = b;
        b = c;
        c = temp;
    }

    if (a > b)
    {
        temp = a;
        a = b;
        b = temp;
    }

    return b;
}

static float Baro_ClampStep(float current, float previous, float max_step)
{
    float delta = current - previous;

    if (delta > max_step)
    {
        return previous + max_step;
    }
    if (delta < -max_step)
    {
        return previous - max_step;
    }

    return current;
}

static void Baro_ClearFilterState(void)
{
    s_pressure_med_buf[0] = 0.0f;
    s_pressure_med_buf[1] = 0.0f;
    s_pressure_med_buf[2] = 0.0f;
    s_pressure_med_count = 0U;
    s_pressure_lpf_pa = 0.0f;
    s_pressure_lpf_inited = 0U;

    s_alt_lpf_m = 0.0f;
    s_prev_alt_raw_m = 0.0f;
    s_ground_lock_cnt = 0U;
    s_airborne_latched = 0U;
}

static void Baro_SeedFilterState(float pressure_pa)
{
    s_pressure_med_buf[0] = pressure_pa;
    s_pressure_med_buf[1] = pressure_pa;
    s_pressure_med_buf[2] = pressure_pa;
    s_pressure_med_count = 3U;
    s_pressure_lpf_pa = pressure_pa;
    s_pressure_lpf_inited = 1U;

    s_alt_lpf_m = 0.0f;
    s_prev_alt_raw_m = 0.0f;
    s_ground_lock_cnt = 0U;
    s_airborne_latched = 0U;
}

static float Baro_FilterPressure(float pressure_pa)
{
    float pressure_med;

    if (s_pressure_med_count < 3U)
    {
        s_pressure_med_buf[s_pressure_med_count] = pressure_pa;
        s_pressure_med_count++;
    }
    else
    {
        s_pressure_med_buf[0] = s_pressure_med_buf[1];
        s_pressure_med_buf[1] = s_pressure_med_buf[2];
        s_pressure_med_buf[2] = pressure_pa;
    }

    if (s_pressure_med_count >= 3U)
    {
        pressure_med = Baro_Median3(s_pressure_med_buf[0], s_pressure_med_buf[1], s_pressure_med_buf[2]);
    }
    else if (s_pressure_med_count == 2U)
    {
        pressure_med = 0.5f * (s_pressure_med_buf[0] + s_pressure_med_buf[1]);
    }
    else
    {
        pressure_med = s_pressure_med_buf[0];
    }

    if (0U == s_pressure_lpf_inited)
    {
        s_pressure_lpf_pa = pressure_med;
        s_pressure_lpf_inited = 1U;
    }
    else
    {
        s_pressure_lpf_pa += BARO_PRESS_LPF_ALPHA * (pressure_med - s_pressure_lpf_pa);
    }

    return s_pressure_lpf_pa;
}

void Baro_Init(void)
{
    uint8 ret = BMP388_init();

    g_baro_ref_pressure = 0.0f;
    g_baro_altitude = 0.0f;
    s_baro_ref_valid = 0U;
    Baro_ClearFilterState();

    if (BMP388_RET_OK != ret)
    {
        s_baro_inited = 0U;
        return;
    }

    s_baro_inited = 1U;
    Baro_Calibrate();
}

void Baro_Calibrate(void)
{
    uint32 i;
    uint32 valid_samples = 0U;
    float pressure_sum = 0.0f;
    float pressure_filtered_pa;
    const uint32 sample_dt_ms = (uint32)(BARO_UPDATE_DT_SEC * 1000.0f + 0.5f);

    if (0U == s_baro_inited)
    {
        return;
    }

    s_baro_ref_valid = 0U;
    Baro_ClearFilterState();

    for (i = 0U; i < BARO_CALIBRATION_SAMPLES; ++i)
    {
        if (BMP388_RET_OK == BMP388_update())
        {
            pressure_filtered_pa = Baro_FilterPressure(g_BMP388_data.pressure_pa);
            pressure_sum += pressure_filtered_pa;
            ++valid_samples;
        }

        if (sample_dt_ms > 0U)
        {
            system_delay_ms(sample_dt_ms);
        }
    }

    if (valid_samples > 0U)
    {
        g_baro_ref_pressure = pressure_sum / (float)valid_samples;
        g_baro_altitude = 0.0f;
        s_baro_ref_valid = 1U;
        Baro_SeedFilterState(g_baro_ref_pressure);
    }
}

void Baro_Update(void)
{
    float pressure_filtered_pa;
    float altitude_raw_m;
    float abs_alt_m;
    float altitude_innov_m;
    float altitude_alpha;

    if (0U == s_baro_inited)
    {
        return;
    }

    if (BMP388_RET_OK != BMP388_update())
    {
        return;
    }

    pressure_filtered_pa = Baro_FilterPressure(g_BMP388_data.pressure_pa);

    if (0U == s_baro_ref_valid)
    {
        g_baro_ref_pressure = pressure_filtered_pa;
        g_baro_altitude = 0.0f;
        s_baro_ref_valid = 1U;
        Baro_SeedFilterState(g_baro_ref_pressure);
        return;
    }

    altitude_raw_m = (g_baro_ref_pressure - pressure_filtered_pa) * BARO_TO_HEIGHT_SCALE_FACTOR * 0.01f;
    altitude_raw_m = Baro_ClampStep(altitude_raw_m, s_prev_alt_raw_m, BARO_ALT_STEP_LIMIT_M);
    s_prev_alt_raw_m = altitude_raw_m;

    altitude_innov_m = altitude_raw_m - s_alt_lpf_m;
    if (Baro_AbsFloat(altitude_innov_m) > BARO_ALT_FAST_SWITCH_M)
    {
        altitude_alpha = BARO_ALT_LPF_ALPHA_FAST;
    }
    else
    {
        altitude_alpha = BARO_ALT_LPF_ALPHA_SLOW;
    }
    s_alt_lpf_m += altitude_alpha * altitude_innov_m;

    abs_alt_m = Baro_AbsFloat(s_alt_lpf_m);
    if (abs_alt_m > BARO_AIRBORNE_LATCH_ABS_H_M)
    {
        s_airborne_latched = 1U;
    }

    if (abs_alt_m < BARO_GROUND_ENTER_ABS_H_M)
    {
        if (s_ground_lock_cnt < 65535U)
        {
            s_ground_lock_cnt++;
        }
    }
    else
    {
        s_ground_lock_cnt = 0U;
    }

    if ((0U == s_airborne_latched) && (s_ground_lock_cnt >= BARO_GROUND_LOCK_FRAMES))
    {
        g_baro_ref_pressure += BARO_REF_ADAPT_ALPHA * (pressure_filtered_pa - g_baro_ref_pressure);
    }

    if (Baro_AbsFloat(s_alt_lpf_m) < BARO_ZERO_DEADBAND_M)
    {
        s_alt_lpf_m = 0.0f;
    }

    g_baro_altitude = s_alt_lpf_m;
}
