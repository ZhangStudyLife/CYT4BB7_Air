#include "FlowGyroDecoupler.h"

/* ===================== 内部类型 ===================== */
typedef struct
{
    uint32 t_ms;
    float  gx;
    float  gy;
} FlowGyroSample_t;

typedef struct
{
    FlowGyroSample_t buf[FLOW_RING_SIZE];
    uint16 head;
} FlowGyroRing_t;

/* ===================== 静态变量 ===================== */
static FlowGyroRing_t s_ring       = {0};
static uint32         s_prev_t_eff = 0U;
static uint8          s_has_prev   = 0U;

static float s_dec_x = 0.0f;
static float s_dec_y = 0.0f;

static float s_kf_hat_x = 0.0f;
static float s_kf_hat_y = 0.0f;
static float s_kf_p_x   = FLOW_KF_P0;
static float s_kf_p_y   = FLOW_KF_P0;

/* ===================== 内部：卡尔曼更新 ===================== */
static float KF_Update(float z, float *hat, float *p)
{
    float k;
    *p  = *p + FLOW_KF_Q;
    k   = *p / (*p + FLOW_KF_R);
    *hat = *hat + k * (z - *hat);
    *p  = (1.0f - k) * (*p);
    return *hat;
}

/* ===================== 内部：区间积分 ===================== */
static uint8 Integrate(uint32 t0, uint32 t1, float *dtheta_x, float *dtheta_y)
{
    uint16 i;
    uint16 cnt   = 0U;
    float  sum_x = 0.0f;
    float  sum_y = 0.0f;

    *dtheta_x = 0.0f;
    *dtheta_y = 0.0f;

    if (t1 <= t0) { return 0U; }

    for (i = 0U; i < FLOW_RING_SIZE; i++)
    {
        const FlowGyroSample_t *s = &s_ring.buf[i];
        if ((s->t_ms >= t0) && (s->t_ms < t1))
        {
            sum_x += s->gx * 0.001f;
            sum_y += s->gy * 0.001f;
            cnt++;
        }
    }

    if (cnt == 0U) { return 0U; }

    *dtheta_x = sum_x;
    *dtheta_y = sum_y;
    return 1U;
}

/* ===================== 公开接口 ===================== */
void FlowGyroDecoupler_Init(void)
{
    uint16 i;
    s_ring.head = 0U;
    for (i = 0U; i < FLOW_RING_SIZE; i++)
    {
        s_ring.buf[i].t_ms = 0U;
        s_ring.buf[i].gx   = 0.0f;
        s_ring.buf[i].gy   = 0.0f;
    }
    s_prev_t_eff = 0U;
    s_has_prev   = 0U;
    s_dec_x      = 0.0f;
    s_dec_y      = 0.0f;
    s_kf_hat_x   = 0.0f;
    s_kf_hat_y   = 0.0f;
    s_kf_p_x     = FLOW_KF_P0;
    s_kf_p_y     = FLOW_KF_P0;
}

void FlowGyroDecoupler_Reinit(void)
{
    FlowGyroDecoupler_Init();
}

void FlowGyroDecoupler_Push1000Hz(uint32 t_ms, float gyro_x, float gyro_y)
{
    s_ring.head = (uint16)((s_ring.head + 1U) % FLOW_RING_SIZE);
    s_ring.buf[s_ring.head].t_ms = t_ms;
    s_ring.buf[s_ring.head].gx   = gyro_x;
    s_ring.buf[s_ring.head].gy   = gyro_y;
}

uint8 FlowGyroDecoupler_Update50Hz(uint32 t_read_ms, int16_t delta_x, int16_t delta_y)
{
    uint32 t_eff;
    float  dtheta_x, dtheta_y;

    if (t_read_ms < FLOW_TAU_MS) { return 0U; }
    t_eff = t_read_ms - FLOW_TAU_MS;

    if (s_has_prev == 0U)
    {
        s_prev_t_eff = t_eff;
        s_has_prev   = 1U;
        return 0U;
    }

    if ((t_eff <= s_prev_t_eff) ||
        ((t_eff - s_prev_t_eff) > FLOW_MAX_WINDOW_MS))
    {
        s_prev_t_eff = t_eff;
        return 0U;
    }

    if (Integrate(s_prev_t_eff, t_eff, &dtheta_x, &dtheta_y) == 0U)
    {
        s_prev_t_eff = t_eff;
        return 0U;
    }

    s_prev_t_eff = t_eff;

    s_dec_x = (float)delta_x - (FLOW_KX * dtheta_x + FLOW_BX);
    s_dec_y = (float)delta_y - (FLOW_KY * dtheta_y + FLOW_BY);

    /* 卡尔曼平滑 */
    s_dec_x = KF_Update(s_dec_x, &s_kf_hat_x, &s_kf_p_x);
    s_dec_y = KF_Update(s_dec_y, &s_kf_hat_y, &s_kf_p_y);

    return 1U;
}

float FlowGyroDecoupler_GetDecX(void) { return s_dec_x; }
float FlowGyroDecoupler_GetDecY(void) { return s_dec_y; }
