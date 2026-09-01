#include "filter.h"

/* === LPF1 实现 === */

void LPF1_Init(LPF1_t *f, float alpha)
{
    f->alpha = alpha;
    f->state = 0.0f;
}

float LPF1_Update(LPF1_t *f, float z)
{
    f->state += f->alpha * (z - f->state);
    return f->state;
}

void LPF1_Reset(LPF1_t *f)
{
    f->state = 0.0f;
}

/* === Kalman1D 实现 === */

void Kalman1D_Init(Kalman1D_t *f, float q, float r, float p0, float x0)
{
    f->q  = q;
    f->r  = r;
    f->p0 = p0;
    f->x0 = x0;
    f->p  = p0;
    f->x  = x0;
}

float Kalman1D_Update(Kalman1D_t *f, float z)
{
    float k;
    f->p += f->q;
    k     = f->p / (f->p + f->r);
    f->x += k * (z - f->x);
    f->p  = (1.0f - k) * f->p;
    return f->x;
}

void Kalman1D_Reset(Kalman1D_t *f)
{
    f->x = f->x0;
    f->p = f->p0;
}

/* === MovAvg 实现 === */

void MovAvg_Init(MovAvg_t *f, uint8 window)
{
    uint8 i;
    if (window == 0U || window > FILT_MA_WIN_MAX)
    {
        window = FILT_MA_WIN_MAX;
    }
    f->win   = window;
    f->idx   = 0U;
    f->count = 0U;
    f->sum   = 0.0f;
    for (i = 0U; i < FILT_MA_WIN_MAX; i++)
    {
        f->buf[i] = 0.0f;
    }
}

float MovAvg_Update(MovAvg_t *f, float z)
{
    f->sum -= f->buf[f->idx];
    f->buf[f->idx] = z;
    f->sum += z;
    f->idx = (uint8)((f->idx + 1U) % f->win);
    if (f->count < f->win)
    {
        f->count++;
    }
    return f->sum / (float)f->count;
}

void MovAvg_Reset(MovAvg_t *f)
{
    uint8 i;
    f->idx   = 0U;
    f->count = 0U;
    f->sum   = 0.0f;
    for (i = 0U; i < FILT_MA_WIN_MAX; i++)
    {
        f->buf[i] = 0.0f;
    }
}

/* === Median 实现 === */

void Median_Init(Median_t *f, uint8 window)
{
    uint8 i;
    if (window == 0U || window > FILT_MEDIAN_WIN_MAX)
    {
        window = FILT_MEDIAN_WIN_MAX;
    }
    f->win   = window;
    f->idx   = 0U;
    f->count = 0U;
    for (i = 0U; i < FILT_MEDIAN_WIN_MAX; i++)
    {
        f->buf[i] = 0.0f;
        f->tmp[i] = 0.0f;
    }
}

float Median_Update(Median_t *f, float z)
{
    uint8 n, i, j;
    float key;

    f->buf[f->idx] = z;
    f->idx = (uint8)((f->idx + 1U) % f->win);
    if (f->count < f->win)
    {
        f->count++;
    }
    n = f->count;

    /* 复制到临时数组后插入排序 */
    for (i = 0U; i < n; i++)
    {
        f->tmp[i] = f->buf[i];
    }
    for (i = 1U; i < n; i++)
    {
        key = f->tmp[i];
        j   = i;
        while (j > 0U && f->tmp[j - 1U] > key)
        {
            f->tmp[j] = f->tmp[j - 1U];
            j--;
        }
        f->tmp[j] = key;
    }

    return f->tmp[n / 2U];
}

void Median_Reset(Median_t *f)
{
    uint8 i;
    f->idx   = 0U;
    f->count = 0U;
    for (i = 0U; i < FILT_MEDIAN_WIN_MAX; i++)
    {
        f->buf[i] = 0.0f;
        f->tmp[i] = 0.0f;
    }
}

/* === StepLim 实现 === */

void StepLim_Init(StepLim_t *f, float step)
{
    f->step   = step;
    f->state  = 0.0f;
    f->seeded = 0U;
}

float StepLim_Update(StepLim_t *f, float z)
{
    float delta;
    if (f->seeded == 0U)
    {
        f->state  = z;
        f->seeded = 1U;
        return f->state;
    }
    delta = z - f->state;
    if (delta > f->step)
    {
        delta = f->step;
    }
    else if (delta < -f->step)
    {
        delta = -f->step;
    }
    f->state += delta;
    return f->state;
}

void StepLim_Reset(StepLim_t *f)
{
    f->state  = 0.0f;
    f->seeded = 0U;
}

/* === Biquad 实现 === */
/* 双线性变换，无外部依赖，仅用 cosf/sinf/sqrtf */

void Biquad_Init(Biquad_t *f, BiquadType_e type, float fs, float fc, float q)
{
    float omega, sn, cs, alpha, a0_inv;

    omega = 2.0f * 3.14159265358979323846f * fc / fs;
    sn    = sinf(omega);
    cs    = cosf(omega);
    alpha = sn / (2.0f * q);

    switch (type)
    {
        case BIQUAD_LPF:
            f->b0 = (1.0f - cs) * 0.5f;
            f->b1 =  1.0f - cs;
            f->b2 = (1.0f - cs) * 0.5f;
            f->a1 = -2.0f * cs;
            f->a2 =  1.0f - alpha;
            a0_inv = 1.0f / (1.0f + alpha);
            break;

        case BIQUAD_HPF:
            f->b0 =  (1.0f + cs) * 0.5f;
            f->b1 = -(1.0f + cs);
            f->b2 =  (1.0f + cs) * 0.5f;
            f->a1 = -2.0f * cs;
            f->a2 =  1.0f - alpha;
            a0_inv = 1.0f / (1.0f + alpha);
            break;

        case BIQUAD_NOTCH:
        default:
            f->b0 =  1.0f;
            f->b1 = -2.0f * cs;
            f->b2 =  1.0f;
            f->a1 = -2.0f * cs;
            f->a2 =  1.0f - alpha;
            a0_inv = 1.0f / (1.0f + alpha);
            break;
    }

    /* 归一化：除以 a0 */
    f->b0 *= a0_inv;
    f->b1 *= a0_inv;
    f->b2 *= a0_inv;
    f->a1 *= a0_inv;
    f->a2 *= a0_inv;

    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

float Biquad_Update(Biquad_t *f, float z)
{
    /* Direct Form II Transposed */
    float out = f->b0 * z + f->d1;
    f->d1 = f->b1 * z - f->a1 * out + f->d2;
    f->d2 = f->b2 * z - f->a2 * out;
    return out;
}

void Biquad_Reset(Biquad_t *f)
{
    f->d1 = 0.0f;
    f->d2 = 0.0f;
}
