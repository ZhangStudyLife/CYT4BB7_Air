/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#include "pix_to_distance.h"
#include "car_lamp_fused.h"
#include "ProjectionCenter.h"

#define PIX_TO_DISTANCE_OUTPUT_LIMIT_CM   (200.0f)

pix_to_distance_result_t g_car_lamp_fused_distance;
pix_to_distance_result_t g_car_lamp_fused_distance_projectioncenter;
pix_to_distance_result_t g_car_lamp_fused_distance_projectioncenter_2;

static float PixToDistance_AbsF(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float PixToDistance_SignF(float x)
{
    return (x >= 0.0f) ? 1.0f : -1.0f;
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

static float PixToDistance_CalcStrongGain(float pixel_abs)
{
    if(pixel_abs <= 30.0f)
    {
        return 1.0f;
    }

    if(pixel_abs < 40.0f)
    {
        return 1.0f + 0.3f * PixToDistance_SmoothStep01((pixel_abs - 30.0f) * 0.1f);
    }

    if(pixel_abs < 50.0f)
    {
        return 1.3f + 0.3f * PixToDistance_SmoothStep01((pixel_abs - 40.0f) * 0.1f);
    }

    if(pixel_abs < 60.0f)
    {
        return 1.6f + 0.4f * PixToDistance_SmoothStep01((pixel_abs - 50.0f) * 0.1f);
    }

    if(pixel_abs < 80.0f)
    {
        return 2.0f + 0.5f * PixToDistance_SmoothStep01((pixel_abs - 60.0f) * 0.05f);
    }

    return 2.5f;
}

static void PixToDistance_Clear(void)
{
    g_car_lamp_fused_distance.valid = 0U;
    g_car_lamp_fused_distance.x_cm = 0.0f;
    g_car_lamp_fused_distance.y_cm = 0.0f;
}

static void PixToDistance_ClearProjectionCenter(void)
{
    g_car_lamp_fused_distance_projectioncenter.valid = 0U;
    g_car_lamp_fused_distance_projectioncenter.x_cm = 0.0f;
    g_car_lamp_fused_distance_projectioncenter.y_cm = 0.0f;
}

static void PixToDistance_ClearProjectionCenter2(void)
{
    g_car_lamp_fused_distance_projectioncenter_2.valid = 0U;
    g_car_lamp_fused_distance_projectioncenter_2.x_cm = 0.0f;
    g_car_lamp_fused_distance_projectioncenter_2.y_cm = 0.0f;
}

static void PixToDistance_Calc5thStretchXY(float cx, float cy, float *x_cm, float *y_cm)
{
    float cx_abs;
    float cy_abs;
    float sx;
    float sy;
    float u;
    float v;
    float u2;
    float u3;
    float u5;
    float v2;
    float v3;
    float v5;
    float x;
    float y;
    float gain_x;
    float gain_y;

    cx_abs = PixToDistance_AbsF(cx);
    cy_abs = PixToDistance_AbsF(cy);
    sx = PixToDistance_SignF(cx);
    sy = PixToDistance_SignF(cy);

    u = cx_abs * 0.02f;
    v = cy_abs * 0.02f;

    u2 = u * u;
    u3 = u2 * u;
    u5 = u3 * u2;

    v2 = v * v;
    v3 = v2 * v;
    v5 = v3 * v2;

    x = 42.61037587f * u
      + 18.26989440f * u3
      - 1.67580239f * u5;
    x = sx * x;

    y = 74.91278180f * v
      - 11.85777924f * v3
      + 10.88236607f * v5;
    y = sy * y;

    gain_x = PixToDistance_CalcStrongGain(cx_abs);
    gain_y = PixToDistance_CalcStrongGain(cy_abs);
    // x = x * gain_x;
    // y = y * gain_y;

    x = PixToDistance_ClampF(x,
                             -PIX_TO_DISTANCE_OUTPUT_LIMIT_CM,
                             PIX_TO_DISTANCE_OUTPUT_LIMIT_CM);
    y = PixToDistance_ClampF(y,
                             -PIX_TO_DISTANCE_OUTPUT_LIMIT_CM,
                             PIX_TO_DISTANCE_OUTPUT_LIMIT_CM);

    if(x_cm != 0)
    {
        *x_cm = x;
    }

    if(y_cm != 0)
    {
        *y_cm = y;
    }
}

void PixToDistance_Init(void)
{
    PixToDistance_Clear();
    PixToDistance_ClearProjectionCenter();
    PixToDistance_ClearProjectionCenter2();
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

uint8 PixToDistance_Update_ProjectionCenter(void)
{
    float lamp_x_cm;
    float lamp_y_cm;
    float projection_x_cm;
    float projection_y_cm;

    if((g_car_lamp_fused.valid == 0U) || (g_projection_center.valid == 0U))
    {
        PixToDistance_ClearProjectionCenter();
        return 0U;
    }

    PixToDistance_Calc5thStretchXY(g_car_lamp_fused.cx,
                                   g_car_lamp_fused.cy,
                                   &lamp_x_cm,
                                   &lamp_y_cm);
    PixToDistance_Calc5thStretchXY(g_projection_center.cx,
                                   g_projection_center.cy,
                                   &projection_x_cm,
                                   &projection_y_cm);

    g_car_lamp_fused_distance_projectioncenter.valid = 1U;
    g_car_lamp_fused_distance_projectioncenter.x_cm = lamp_x_cm - projection_x_cm;
    g_car_lamp_fused_distance_projectioncenter.y_cm = lamp_y_cm - projection_y_cm;
    return 1U;
}

uint8 PixToDistance_UpdateProjectionCenter2_100Hz(void)
{
    float delta_cx;
    float delta_cy;
    float x_cm;
    float y_cm;

    if((g_car_lamp_fused.valid == 0U) || (g_projection_center.valid == 0U))
    {
        PixToDistance_ClearProjectionCenter2();
        return 0U;
    }

    delta_cx = g_car_lamp_fused.cx - g_projection_center.cx;
    delta_cy = g_car_lamp_fused.cy - g_projection_center.cy;
    PixToDistance_Calc5thStretchXY(delta_cx, delta_cy, &x_cm, &y_cm);

    g_car_lamp_fused_distance_projectioncenter_2.valid = 1U;
    g_car_lamp_fused_distance_projectioncenter_2.x_cm = x_cm;
    g_car_lamp_fused_distance_projectioncenter_2.y_cm = y_cm;
    return 1U;
}
