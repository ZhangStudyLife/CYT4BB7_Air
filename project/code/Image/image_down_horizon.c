#include "image_down_horizon.h"

#include <math.h>
#include <string.h>

#define IMAGE_DOWN_HORIZON_PI                 (3.14159265358979323846f)
#define IMAGE_DOWN_HORIZON_DEG_TO_RAD         (IMAGE_DOWN_HORIZON_PI / 180.0f)
#define IMAGE_DOWN_HORIZON_CENTER_X           (86.7802501f)
#define IMAGE_DOWN_HORIZON_CENTER_Y           (55.0531910f)
#define IMAGE_DOWN_HORIZON_SCALE              (93.5f)
#define IMAGE_DOWN_HORIZON_THETA_K1           (1.26656119f)
#define IMAGE_DOWN_HORIZON_THETA_K3           (-0.0392416268f)
#define IMAGE_DOWN_HORIZON_THETA_K5           (0.143348613f)
#define IMAGE_DOWN_HORIZON_RANGE_MM           (7000.0f)
#define IMAGE_DOWN_HORIZON_HEIGHT_BIAS_MM     (239.727462f)
#define IMAGE_DOWN_HORIZON_EDGE_RADIUS        (1.27147706f)
#define IMAGE_DOWN_HORIZON_EDGE_THETA         (2.00610176f)
#define IMAGE_DOWN_HORIZON_EDGE_SLOPE         (2.94949888f)
#define IMAGE_DOWN_HORIZON_ROLL_MIN_DEG       (-37.6750984f)
#define IMAGE_DOWN_HORIZON_ROLL_MAX_DEG       (34.1501808f)
#define IMAGE_DOWN_HORIZON_PITCH_MIN_DEG      (-34.9680252f)
#define IMAGE_DOWN_HORIZON_PITCH_MAX_DEG      (37.8621559f)
#define IMAGE_DOWN_HORIZON_HEIGHT_MIN_MM      (515.624451f)
#define IMAGE_DOWN_HORIZON_HEIGHT_MAX_MM      (1308.21106f)
#define IMAGE_DOWN_HORIZON_STEP_COS           (0.999847695f)
#define IMAGE_DOWN_HORIZON_STEP_SIN           (0.0174524064f)

static const float s_body_to_camera[3][3] =
{
    {-0.0156029708f, 0.999826714f, 0.0101532740f},
    {-0.999776778f, -0.0157452587f, 0.0140883038f},
    {0.0142457284f, -0.00993118815f, 0.999849204f}
};

float g_image_down_horizon_x[IMAGE_DOWN_HORIZON_POINT_COUNT];
float g_image_down_horizon_y[IMAGE_DOWN_HORIZON_POINT_COUNT];
float g_image_down_horizon_top_y[IMAGE_DOWN_HORIZON_WIDTH];
float g_image_down_horizon_bottom_y[IMAGE_DOWN_HORIZON_WIDTH];
uint8 g_image_down_horizon_column_valid[IMAGE_DOWN_HORIZON_WIDTH];
uint8 g_image_down_horizon_valid;
uint8 g_image_down_horizon_extrapolated;

static uint8 s_initialized;

static float image_down_horizon_abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float image_down_horizon_radius(float theta)
{
    float low = 0.0f;
    float high = IMAGE_DOWN_HORIZON_EDGE_RADIUS;
    uint8 iteration;

    if(theta > IMAGE_DOWN_HORIZON_EDGE_THETA)
    {
        return IMAGE_DOWN_HORIZON_EDGE_RADIUS +
               (theta - IMAGE_DOWN_HORIZON_EDGE_THETA) /
               IMAGE_DOWN_HORIZON_EDGE_SLOPE;
    }
    for(iteration = 0U; iteration < 28U; iteration++)
    {
        float radius = (low + high) * 0.5f;
        float radius2 = radius * radius;
        float mapped = radius *
                       (IMAGE_DOWN_HORIZON_THETA_K1 +
                        radius2 * (IMAGE_DOWN_HORIZON_THETA_K3 +
                                   radius2 * IMAGE_DOWN_HORIZON_THETA_K5));

        if(mapped < theta)
        {
            low = radius;
        }
        else
        {
            high = radius;
        }
    }
    return (low + high) * 0.5f;
}

static void image_down_horizon_add_column(uint16 x, float y)
{
    if(g_image_down_horizon_column_valid[x] == 0U)
    {
        g_image_down_horizon_top_y[x] = y;
        g_image_down_horizon_bottom_y[x] = y;
        g_image_down_horizon_column_valid[x] = 1U;
    }
    else
    {
        if(y < g_image_down_horizon_top_y[x])
        {
            g_image_down_horizon_top_y[x] = y;
        }
        if(y > g_image_down_horizon_bottom_y[x])
        {
            g_image_down_horizon_bottom_y[x] = y;
        }
    }
}

static void image_down_horizon_build_columns(void)
{
    uint16 index;

    memset(g_image_down_horizon_column_valid, 0,
           sizeof(g_image_down_horizon_column_valid));
    for(index = 0U; index < IMAGE_DOWN_HORIZON_POINT_COUNT; index++)
    {
        uint16 next = (uint16)((index + 1U) % IMAGE_DOWN_HORIZON_POINT_COUNT);
        float x0 = g_image_down_horizon_x[index];
        float y0 = g_image_down_horizon_y[index];
        float x1 = g_image_down_horizon_x[next];
        float y1 = g_image_down_horizon_y[next];
        float dx = x1 - x0;
        int first;
        int last;
        int x;

        if(image_down_horizon_abs(dx) < 1.0e-6f)
        {
            x = (int)(x0 + ((x0 >= 0.0f) ? 0.5f : -0.5f));
            if((x >= 0) && (x < (int)IMAGE_DOWN_HORIZON_WIDTH))
            {
                image_down_horizon_add_column((uint16)x, y0);
                image_down_horizon_add_column((uint16)x, y1);
            }
            continue;
        }

        first = (int)ceilf((x0 < x1) ? x0 : x1);
        last = (int)floorf((x0 > x1) ? x0 : x1);
        if(first < 0)
        {
            first = 0;
        }
        if(last >= (int)IMAGE_DOWN_HORIZON_WIDTH)
        {
            last = (int)IMAGE_DOWN_HORIZON_WIDTH - 1;
        }
        for(x = first; x <= last; x++)
        {
            float ratio = ((float)x - x0) / dx;
            if((ratio >= 0.0f) && (ratio <= 1.0f))
            {
                image_down_horizon_add_column((uint16)x,
                                              y0 + ratio * (y1 - y0));
            }
        }
    }
}

void image_down_horizon_init(void)
{
    memset(g_image_down_horizon_x, 0, sizeof(g_image_down_horizon_x));
    memset(g_image_down_horizon_y, 0, sizeof(g_image_down_horizon_y));
    memset(g_image_down_horizon_top_y, 0, sizeof(g_image_down_horizon_top_y));
    memset(g_image_down_horizon_bottom_y, 0, sizeof(g_image_down_horizon_bottom_y));
    memset(g_image_down_horizon_column_valid, 0,
           sizeof(g_image_down_horizon_column_valid));
    g_image_down_horizon_valid = 0U;
    g_image_down_horizon_extrapolated = 0U;

    s_initialized = 1U;
}

void image_down_horizon_invalidate(void)
{
    g_image_down_horizon_valid = 0U;
    g_image_down_horizon_extrapolated = 0U;
    memset(g_image_down_horizon_column_valid, 0,
           sizeof(g_image_down_horizon_column_valid));
}

void image_down_horizon_update(float roll_deg,
                               float pitch_deg,
                               float height_mm,
                               uint8 attitude_valid,
                               uint8 height_valid)
{
    float roll;
    float pitch;
    float g[3];
    float u[3];
    float v[3];
    float u_norm;
    float height;
    float boundary_cos = 1.0f;
    float boundary_sin = 0.0f;
    uint16 index;

    if(s_initialized == 0U)
    {
        image_down_horizon_init();
    }
    if((attitude_valid == 0U) || (height_valid == 0U) ||
       (roll_deg != roll_deg) || (pitch_deg != pitch_deg) ||
       (height_mm != height_mm))
    {
        image_down_horizon_invalidate();
        return;
    }

    height = height_mm + IMAGE_DOWN_HORIZON_HEIGHT_BIAS_MM;
    if(height <= 0.0f)
    {
        image_down_horizon_invalidate();
        return;
    }
    g_image_down_horizon_extrapolated =
        ((roll_deg < IMAGE_DOWN_HORIZON_ROLL_MIN_DEG) ||
         (roll_deg > IMAGE_DOWN_HORIZON_ROLL_MAX_DEG) ||
         (pitch_deg < IMAGE_DOWN_HORIZON_PITCH_MIN_DEG) ||
         (pitch_deg > IMAGE_DOWN_HORIZON_PITCH_MAX_DEG) ||
         (height_mm < IMAGE_DOWN_HORIZON_HEIGHT_MIN_MM) ||
         (height_mm > IMAGE_DOWN_HORIZON_HEIGHT_MAX_MM)) ? 1U : 0U;

    roll = roll_deg * IMAGE_DOWN_HORIZON_DEG_TO_RAD;
    pitch = pitch_deg * IMAGE_DOWN_HORIZON_DEG_TO_RAD;
    g[0] = -sinf(pitch);
    g[1] = sinf(roll) * cosf(pitch);
    g[2] = cosf(roll) * cosf(pitch);
    u[0] = 0.0f;
    u[1] = g[2];
    u[2] = -g[1];
    u_norm = sqrtf(u[1] * u[1] + u[2] * u[2]);
    if(u_norm < 1.0e-6f)
    {
        u[0] = -g[2];
        u[1] = 0.0f;
        u[2] = g[0];
        u_norm = sqrtf(u[0] * u[0] + u[2] * u[2]);
    }
    u[0] /= u_norm;
    u[1] /= u_norm;
    u[2] /= u_norm;
    v[0] = g[1] * u[2] - g[2] * u[1];
    v[1] = g[2] * u[0] - g[0] * u[2];
    v[2] = g[0] * u[1] - g[1] * u[0];

    for(index = 0U; index < IMAGE_DOWN_HORIZON_POINT_COUNT; index++)
    {
        float d[3];
        float camera[3];
        float norm;
        float lateral;
        float theta;
        float radius;
        float factor;
        float next_cos;
        uint8 row;

        d[0] = height * g[0] + IMAGE_DOWN_HORIZON_RANGE_MM *
               (boundary_cos * u[0] + boundary_sin * v[0]);
        d[1] = height * g[1] + IMAGE_DOWN_HORIZON_RANGE_MM *
               (boundary_cos * u[1] + boundary_sin * v[1]);
        d[2] = height * g[2] + IMAGE_DOWN_HORIZON_RANGE_MM *
               (boundary_cos * u[2] + boundary_sin * v[2]);
        norm = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if(norm <= 1.0e-6f)
        {
            image_down_horizon_invalidate();
            return;
        }
        d[0] /= norm;
        d[1] /= norm;
        d[2] /= norm;
        for(row = 0U; row < 3U; row++)
        {
            camera[row] = s_body_to_camera[row][0] * d[0] +
                          s_body_to_camera[row][1] * d[1] +
                          s_body_to_camera[row][2] * d[2];
        }
        if(camera[2] > 1.0f)
        {
            camera[2] = 1.0f;
        }
        else if(camera[2] < -1.0f)
        {
            camera[2] = -1.0f;
        }
        theta = acosf(camera[2]);
        lateral = sqrtf(camera[0] * camera[0] + camera[1] * camera[1]);
        radius = image_down_horizon_radius(theta);
        if(lateral <= 1.0e-6f)
        {
            g_image_down_horizon_x[index] = IMAGE_DOWN_HORIZON_CENTER_X;
            g_image_down_horizon_y[index] = IMAGE_DOWN_HORIZON_CENTER_Y;
        }
        else
        {
            factor = IMAGE_DOWN_HORIZON_SCALE * radius / lateral;
            g_image_down_horizon_x[index] = IMAGE_DOWN_HORIZON_CENTER_X +
                                            camera[0] * factor;
            g_image_down_horizon_y[index] = IMAGE_DOWN_HORIZON_CENTER_Y +
                                            camera[1] * factor;
        }
        next_cos = boundary_cos * IMAGE_DOWN_HORIZON_STEP_COS -
                   boundary_sin * IMAGE_DOWN_HORIZON_STEP_SIN;
        boundary_sin = boundary_sin * IMAGE_DOWN_HORIZON_STEP_COS +
                       boundary_cos * IMAGE_DOWN_HORIZON_STEP_SIN;
        boundary_cos = next_cos;
    }

    image_down_horizon_build_columns();
    g_image_down_horizon_valid = 1U;
}

uint8 image_down_horizon_get_point(uint16 index, float *x, float *y)
{
    if((g_image_down_horizon_valid == 0U) ||
       (index >= IMAGE_DOWN_HORIZON_POINT_COUNT) ||
       (x == 0) || (y == 0))
    {
        return 0U;
    }
    *x = g_image_down_horizon_x[index];
    *y = g_image_down_horizon_y[index];
    return 1U;
}

uint8 image_down_horizon_get_column(uint16 x, float *top_y, float *bottom_y)
{
    if((g_image_down_horizon_valid == 0U) ||
       (x >= IMAGE_DOWN_HORIZON_WIDTH) ||
       (g_image_down_horizon_column_valid[x] == 0U) ||
       (top_y == 0) || (bottom_y == 0))
    {
        return 0U;
    }
    *top_y = g_image_down_horizon_top_y[x];
    *bottom_y = g_image_down_horizon_bottom_y[x];
    return 1U;
}
