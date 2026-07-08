#include "pix_to_distance.h"
#include "car_lamp_fused.h"

#define PIX_TO_DISTANCE_X_STRETCH_START   (40.0f)
#define PIX_TO_DISTANCE_X_STRETCH_END     (80.0f)
#define PIX_TO_DISTANCE_X_STRETCH_GAIN    (0.20f)
#define PIX_TO_DISTANCE_Y_STRETCH_START   (40.0f)
#define PIX_TO_DISTANCE_Y_STRETCH_END     (80.0f)
#define PIX_TO_DISTANCE_Y_STRETCH_GAIN    (0.60f)
#define PIX_TO_DISTANCE_OUTPUT_LIMIT_CM   (200.0f)

pix_to_distance_result_t g_car_lamp_fused_distance;

static float PixToDistance_AbsF(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float PixToDistance_ClampF(float x, float min_val, float max_val)
{
    if(x < min_val)
    {
        return min_val;
    }

    if(x > max_val)
    {
        return max_val;
    }

    return x;
}

static float PixToDistance_SmoothStep01(float t)
{
    t = PixToDistance_ClampF(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static void PixToDistance_Clear(void)
{
    g_car_lamp_fused_distance.valid = 0U;
    g_car_lamp_fused_distance.x_cm = 0.0f;
    g_car_lamp_fused_distance.y_cm = 0.0f;
}

static void PixToDistance_Calc5thStretchXY(float cx, float cy, float *x_cm, float *y_cm)
{
    float u = cx * 0.02f;
    float v = cy * 0.02f;
    float u2 = u * u;
    float u3 = u2 * u;
    float u4 = u2 * u2;
    float u5 = u4 * u;
    float v2 = v * v;
    float v3 = v2 * v;
    float v4 = v2 * v2;
    float v5 = v4 * v;
    float x;
    float y;
    float cx_abs;
    float cy_abs;
    float tx;
    float ty;
    float sx;
    float sy;
    float x_stretch;
    float y_stretch;

    x = 44.01069065f * u
      - 27.61256553f * u * v2
      + 17.90590983f * u * v4
      + 20.56672332f * u3
      + 36.47132093f * u3 * v2
      - 2.96596734f * u5;

    y = 65.36820230f * v
      + 15.76715691f * v3
      - 4.84363232f * v5
      + 1.97772301f * u2 * v
      - 5.97205674f * u2 * v3
      - 22.11071927f * u4 * v;

    cx_abs = PixToDistance_AbsF(cx);
    tx = (cx_abs - PIX_TO_DISTANCE_X_STRETCH_START) * 0.025f;
    sx = PixToDistance_SmoothStep01(tx);
    x_stretch = 1.0f + PIX_TO_DISTANCE_X_STRETCH_GAIN * sx;
    x = x * x_stretch;

    cy_abs = PixToDistance_AbsF(cy);
    ty = (cy_abs - PIX_TO_DISTANCE_Y_STRETCH_START) * 0.025f;
    sy = PixToDistance_SmoothStep01(ty);
    y_stretch = 1.0f + PIX_TO_DISTANCE_Y_STRETCH_GAIN * sy;
    y = y * y_stretch;

    x = PixToDistance_ClampF(x,
                             -PIX_TO_DISTANCE_OUTPUT_LIMIT_CM,
                             PIX_TO_DISTANCE_OUTPUT_LIMIT_CM);
    y = PixToDistance_ClampF(y,
                             -PIX_TO_DISTANCE_OUTPUT_LIMIT_CM,
                             PIX_TO_DISTANCE_OUTPUT_LIMIT_CM);

    *x_cm = x;
    *y_cm = y;
}

void PixToDistance_Init(void)
{
    PixToDistance_Clear();
}

uint8 PixToDistance_Update(void)
{
    float cx;
    float cy;
    float x_cm;
    float y_cm;

    if(g_car_lamp_fused.valid == 0U)
    {
        PixToDistance_Clear();
        return 0U;
    }

    cx = g_car_lamp_fused.cx;
    cy = g_car_lamp_fused.cy;
    PixToDistance_Calc5thStretchXY(cx, cy, &x_cm, &y_cm);

    g_car_lamp_fused_distance.valid = 1U;
    g_car_lamp_fused_distance.x_cm = x_cm;
    g_car_lamp_fused_distance.y_cm = y_cm;
    return 1U;
}
