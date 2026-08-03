#include "image_down_horizon.h"

#include <math.h>
#include <string.h>

#define DOWN_HORIZON_PI                  (3.14159265358979323846f)
#define DOWN_HORIZON_DEG_TO_RAD          (DOWN_HORIZON_PI / 180.0f)
#define DOWN_HORIZON_CENTER_X            (86.7802501f)
#define DOWN_HORIZON_CENTER_Y            (55.0531910f)
#define DOWN_HORIZON_SCALE               (93.5f)
#define DOWN_HORIZON_THETA_K1            (1.26656119f)
#define DOWN_HORIZON_THETA_K3            (-0.0392416268f)
#define DOWN_HORIZON_THETA_K5            (0.143348613f)
#define DOWN_HORIZON_RANGE_MM            (7000.0f)
#define DOWN_HORIZON_HEIGHT_BIAS_MM      (239.727462f)
#define DOWN_HORIZON_EDGE_RADIUS         (1.27147706f)
#define DOWN_HORIZON_EDGE_THETA          (2.00610176f)
#define DOWN_HORIZON_EDGE_SLOPE          (2.94949888f)
#define DOWN_HORIZON_ROLL_MIN_DEG        (-37.6750984f)
#define DOWN_HORIZON_ROLL_MAX_DEG        (34.1501808f)
#define DOWN_HORIZON_PITCH_MIN_DEG       (-34.9680252f)
#define DOWN_HORIZON_PITCH_MAX_DEG       (37.8621559f)
#define DOWN_HORIZON_HEIGHT_MIN_MM       (515.624451f)
#define DOWN_HORIZON_HEIGHT_MAX_MM       (1308.21106f)
#define DOWN_HORIZON_STEP_COS            (0.999847695f)
#define DOWN_HORIZON_STEP_SIN            (0.0174524064f)
#define DOWN_HORIZON_TABLE_INTERVALS     (256U)
#define DOWN_HORIZON_TABLE_SIZE          (DOWN_HORIZON_TABLE_INTERVALS + 1U)
#define DOWN_HORIZON_ZERO_EPSILON        (1.0e-7f)
#define DOWN_HORIZON_ROOT_EPSILON        (1.0e-4f)
#define DOWN_HORIZON_TANGENT_PROBE       (0.002f)
#define DOWN_HORIZON_TOP_SENTINEL        (-0.01f)
#define DOWN_HORIZON_BOTTOM_SENTINEL     ((float)(IMAGE_DOWN_HORIZON_HEIGHT - 1U) + 0.01f)
#define DOWN_HORIZON_COLUMN_OUTSIDE      (0U)
#define DOWN_HORIZON_COLUMN_PARTIAL      (1U)
#define DOWN_HORIZON_COLUMN_INSIDE       (2U)
#define DOWN_HORIZON_TRACK_RADIUS        (6)
#define DOWN_HORIZON_TRACK_TANGENT_GUARD (3.0f)

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

static float s_normalized_x[IMAGE_DOWN_HORIZON_WIDTH];
static float s_normalized_y[IMAGE_DOWN_HORIZON_HEIGHT];
static float s_radius2_table_x[IMAGE_DOWN_HORIZON_WIDTH];
static float s_radius2_table_y[IMAGE_DOWN_HORIZON_HEIGHT];
static float s_gravity_y[IMAGE_DOWN_HORIZON_HEIGHT];
static float s_radial_factor[DOWN_HORIZON_TABLE_SIZE];
static float s_ray_z[DOWN_HORIZON_TABLE_SIZE];
static float s_radius2_to_table;
static float s_last_gravity_body[3];
static float s_last_height;
static uint8 s_column_state[IMAGE_DOWN_HORIZON_WIDTH];
static uint8 s_column_root_count[IMAGE_DOWN_HORIZON_WIDTH];
static uint8 s_column_peak_y[IMAGE_DOWN_HORIZON_WIDTH];
static uint8 s_points_valid;
static uint8 s_initialized;

static float down_horizon_abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float down_horizon_max(float first, float second)
{
    return (first > second) ? first : second;
}

static float down_horizon_theta(float radius, float radius2)
{
    if (radius > DOWN_HORIZON_EDGE_RADIUS)
    {
        return DOWN_HORIZON_EDGE_THETA +
               (radius - DOWN_HORIZON_EDGE_RADIUS) * DOWN_HORIZON_EDGE_SLOPE;
    }
    return radius *
           (DOWN_HORIZON_THETA_K1 +
            radius2 * (DOWN_HORIZON_THETA_K3 +
                       radius2 * DOWN_HORIZON_THETA_K5));
}

static float down_horizon_radius(float theta)
{
    float low = 0.0f;
    float high = DOWN_HORIZON_EDGE_RADIUS;
    uint8 iteration;

    if (theta > DOWN_HORIZON_EDGE_THETA)
    {
        return DOWN_HORIZON_EDGE_RADIUS +
               (theta - DOWN_HORIZON_EDGE_THETA) / DOWN_HORIZON_EDGE_SLOPE;
    }
    for (iteration = 0U; iteration < 28U; iteration++)
    {
        float radius = (low + high) * 0.5f;
        float radius2 = radius * radius;
        float mapped = down_horizon_theta(radius, radius2);

        if (mapped < theta)
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

static float down_horizon_value(float table_position,
                                float planar_dot,
                                const float gravity_camera[3],
                                float cone_cos)
{
    uint16 index;
    float fraction;
    float radial;
    float ray_z;

    if (table_position <= 0.0f)
    {
        index = 0U;
        fraction = 0.0f;
    }
    else if (table_position >= (float)DOWN_HORIZON_TABLE_INTERVALS)
    {
        index = DOWN_HORIZON_TABLE_INTERVALS - 1U;
        fraction = 1.0f;
    }
    else
    {
        index = (uint16)table_position;
        fraction = table_position - (float)index;
    }

    radial = s_radial_factor[index] +
             fraction * (s_radial_factor[index + 1U] - s_radial_factor[index]);
    ray_z = s_ray_z[index] +
            fraction * (s_ray_z[index + 1U] - s_ray_z[index]);
    return planar_dot * radial +
           ray_z * gravity_camera[2] - cone_cos;
}

static void down_horizon_add_root(float root,
                                  uint8 *root_count,
                                  float *top_root,
                                  float *bottom_root)
{
    if (*root_count != 0U &&
        down_horizon_abs(root - *bottom_root) <= DOWN_HORIZON_ROOT_EPSILON)
    {
        return;
    }
    if (*root_count == 0U)
    {
        *top_root = root;
    }
    *bottom_root = root;
    if (*root_count < 255U)
    {
        (*root_count)++;
    }
}

static float down_horizon_column_value(uint16 x,
                                       uint16 y,
                                       float gravity_x,
                                       const float gravity_camera[3],
                                       float cone_cos)
{
    return down_horizon_value(
        s_radius2_table_x[x] + s_radius2_table_y[y],
        gravity_x + s_gravity_y[y],
        gravity_camera,
        cone_cos);
}

static void down_horizon_store_column(uint16 x,
                                      float first_value,
                                      float last_value,
                                      uint8 root_count,
                                      float top_root,
                                      float bottom_root)
{
    s_column_root_count[x] = root_count;
    if (root_count >= 2U)
    {
        g_image_down_horizon_top_y[x] = top_root;
        g_image_down_horizon_bottom_y[x] = bottom_root;
        g_image_down_horizon_column_valid[x] = 1U;
        s_column_state[x] = DOWN_HORIZON_COLUMN_PARTIAL;
    }
    else if (root_count == 1U &&
             ((first_value >= 0.0f) != (last_value >= 0.0f)))
    {
        if (first_value < 0.0f)
        {
            g_image_down_horizon_top_y[x] = top_root;
            g_image_down_horizon_bottom_y[x] = DOWN_HORIZON_BOTTOM_SENTINEL;
        }
        else
        {
            g_image_down_horizon_top_y[x] = DOWN_HORIZON_TOP_SENTINEL;
            g_image_down_horizon_bottom_y[x] = top_root;
        }
        g_image_down_horizon_column_valid[x] = 1U;
        s_column_state[x] = DOWN_HORIZON_COLUMN_PARTIAL;
    }
    else if (first_value >= 0.0f)
    {
        g_image_down_horizon_top_y[x] = DOWN_HORIZON_TOP_SENTINEL;
        g_image_down_horizon_bottom_y[x] = DOWN_HORIZON_BOTTOM_SENTINEL;
        s_column_state[x] = DOWN_HORIZON_COLUMN_INSIDE;
    }
    else
    {
        g_image_down_horizon_top_y[x] = 0.0f;
        g_image_down_horizon_bottom_y[x] = 0.0f;
        s_column_state[x] = DOWN_HORIZON_COLUMN_OUTSIDE;
    }
}

static void down_horizon_scan_column(uint16 x,
                                     const float gravity_camera[3],
                                     float cone_cos)
{
    float gravity_x = s_normalized_x[x] * gravity_camera[0];
    float first_value = down_horizon_column_value(
        x, 0U, gravity_x, gravity_camera, cone_cos);
    float previous_value = first_value;
    float last_value = first_value;
    float peak_value = first_value;
    float top_root = 0.0f;
    float bottom_root = 0.0f;
    uint8 root_count = 0U;
    uint8 peak_y = 0U;
    uint16 previous_y = 0U;
    uint16 y;

    if (down_horizon_abs(first_value) <= DOWN_HORIZON_ZERO_EPSILON)
    {
        down_horizon_add_root(0.0f, &root_count, &top_root, &bottom_root);
    }
    for (y = 2U; y < IMAGE_DOWN_HORIZON_HEIGHT; y += 2U)
    {
        float current_value = down_horizon_column_value(
            x, y, gravity_x, gravity_camera, cone_cos);

        if (current_value > peak_value)
        {
            peak_value = current_value;
            peak_y = (uint8)y;
        }

        if (down_horizon_abs(current_value) <= DOWN_HORIZON_ZERO_EPSILON)
        {
            down_horizon_add_root((float)y,
                                  &root_count,
                                  &top_root,
                                  &bottom_root);
        }
        else if (down_horizon_abs(previous_value) > DOWN_HORIZON_ZERO_EPSILON &&
                 ((previous_value > 0.0f) != (current_value > 0.0f)))
        {
            uint16 lower_y = previous_y;
            uint16 upper_y = y;
            float lower_value = previous_value;
            float upper_value = current_value;
            uint16 middle_y = (uint16)(previous_y + 1U);
            float middle_value = down_horizon_column_value(
                x, middle_y, gravity_x, gravity_camera, cone_cos);
            float magnitude;
            float root;

            if (down_horizon_abs(middle_value) <= DOWN_HORIZON_ZERO_EPSILON)
            {
                down_horizon_add_root((float)middle_y,
                                      &root_count,
                                      &top_root,
                                      &bottom_root);
                previous_value = current_value;
                last_value = current_value;
                previous_y = y;
                continue;
            }
            if ((lower_value > 0.0f) != (middle_value > 0.0f))
            {
                upper_y = middle_y;
                upper_value = middle_value;
            }
            else
            {
                lower_y = middle_y;
                lower_value = middle_value;
            }
            magnitude = down_horizon_abs(lower_value) +
                        down_horizon_abs(upper_value);
            root = (float)lower_y +
                   down_horizon_abs(lower_value) / magnitude *
                   (float)(upper_y - lower_y);

            down_horizon_add_root(root,
                                  &root_count,
                                  &top_root,
                                  &bottom_root);
        }
        else if (previous_value < 0.0f && current_value < 0.0f &&
                 (down_horizon_abs(previous_value) <= DOWN_HORIZON_TANGENT_PROBE ||
                  down_horizon_abs(current_value) <= DOWN_HORIZON_TANGENT_PROBE))
        {
            uint16 middle_y = (uint16)(previous_y + 1U);
            float middle_value = down_horizon_column_value(
                x, middle_y, gravity_x, gravity_camera, cone_cos);

            if (middle_value > DOWN_HORIZON_ZERO_EPSILON)
            {
                float first_magnitude = down_horizon_abs(previous_value) +
                                        middle_value;
                float second_magnitude = middle_value +
                                         down_horizon_abs(current_value);
                float first_root = (float)previous_y +
                                   down_horizon_abs(previous_value) /
                                   first_magnitude;
                float second_root = (float)middle_y +
                                    middle_value / second_magnitude;

                down_horizon_add_root(first_root,
                                      &root_count,
                                      &top_root,
                                      &bottom_root);
                down_horizon_add_root(second_root,
                                      &root_count,
                                      &top_root,
                                      &bottom_root);
            }
            else
            {
                float quadratic_a =
                    (previous_value + current_value) * 0.5f - middle_value;
                float quadratic_b = middle_value - previous_value - quadratic_a;
                float discriminant = quadratic_b * quadratic_b -
                                     4.0f * quadratic_a * previous_value;

                if (quadratic_a < 0.0f && discriminant > 0.0f)
                {
                    float root_offset = sqrtf(discriminant);
                    float first_offset = (-quadratic_b + root_offset) /
                                         (2.0f * quadratic_a);
                    float second_offset = (-quadratic_b - root_offset) /
                                          (2.0f * quadratic_a);

                    if (first_offset > second_offset)
                    {
                        float temporary = first_offset;
                        first_offset = second_offset;
                        second_offset = temporary;
                    }
                    if (first_offset >= 0.0f && second_offset <= 2.0f)
                    {
                        down_horizon_add_root((float)previous_y + first_offset,
                                              &root_count,
                                              &top_root,
                                              &bottom_root);
                        down_horizon_add_root((float)previous_y + second_offset,
                                              &root_count,
                                              &top_root,
                                              &bottom_root);
                    }
                }
            }
        }
        previous_value = current_value;
        last_value = current_value;
        previous_y = y;
    }
    if (previous_y != IMAGE_DOWN_HORIZON_HEIGHT - 1U)
    {
        y = IMAGE_DOWN_HORIZON_HEIGHT - 1U;
        last_value = down_horizon_column_value(
            x, y, gravity_x, gravity_camera, cone_cos);
        if (last_value > peak_value)
        {
            peak_y = (uint8)y;
        }
        if (down_horizon_abs(last_value) <= DOWN_HORIZON_ZERO_EPSILON)
        {
            down_horizon_add_root((float)y,
                                  &root_count,
                                  &top_root,
                                  &bottom_root);
        }
        else if (down_horizon_abs(previous_value) > DOWN_HORIZON_ZERO_EPSILON &&
                 ((previous_value > 0.0f) != (last_value > 0.0f)))
        {
            float magnitude = down_horizon_abs(previous_value) +
                              down_horizon_abs(last_value);
            float root = (float)previous_y +
                         down_horizon_abs(previous_value) / magnitude *
                         (float)(y - previous_y);

            down_horizon_add_root(root,
                                  &root_count,
                                  &top_root,
                                  &bottom_root);
        }
    }

    down_horizon_store_column(x,
                              first_value,
                              last_value,
                              root_count,
                              top_root,
                              bottom_root);
    s_column_peak_y[x] = peak_y;
}

static uint8 down_horizon_find_root_near(uint16 x,
                                         float gravity_x,
                                         float reference_y,
                                         const float gravity_camera[3],
                                         float cone_cos,
                                         float *root)
{
    int center = (int)(reference_y + 0.5f);
    int first_y = center - DOWN_HORIZON_TRACK_RADIUS;
    int last_y = center + DOWN_HORIZON_TRACK_RADIUS;
    float best_distance = 1.0e9f;
    float previous_value;
    int previous_y;
    int y;
    uint8 found = 0U;

    if (first_y < 0)
    {
        first_y = 0;
    }
    if (last_y >= (int)IMAGE_DOWN_HORIZON_HEIGHT)
    {
        last_y = (int)IMAGE_DOWN_HORIZON_HEIGHT - 1;
    }
    previous_y = first_y;
    previous_value = down_horizon_column_value(
        x, (uint16)previous_y, gravity_x, gravity_camera, cone_cos);
    if (down_horizon_abs(previous_value) <= DOWN_HORIZON_ZERO_EPSILON)
    {
        best_distance = down_horizon_abs((float)previous_y - reference_y);
        *root = (float)previous_y;
        found = 1U;
    }

    for (y = first_y + 1; y <= last_y; y++)
    {
        float current_value = down_horizon_column_value(
            x, (uint16)y, gravity_x, gravity_camera, cone_cos);
        float candidate = 0.0f;
        uint8 candidate_valid = 0U;

        if (down_horizon_abs(current_value) <= DOWN_HORIZON_ZERO_EPSILON)
        {
            candidate = (float)y;
            candidate_valid = 1U;
        }
        else if (down_horizon_abs(previous_value) > DOWN_HORIZON_ZERO_EPSILON &&
                 ((previous_value > 0.0f) != (current_value > 0.0f)))
        {
            float magnitude = down_horizon_abs(previous_value) +
                              down_horizon_abs(current_value);
            candidate = (float)previous_y +
                        down_horizon_abs(previous_value) / magnitude;
            candidate_valid = 1U;
        }
        if (candidate_valid != 0U)
        {
            float distance = down_horizon_abs(candidate - reference_y);
            if (distance < best_distance)
            {
                best_distance = distance;
                *root = candidate;
                found = 1U;
            }
        }
        previous_y = y;
        previous_value = current_value;
    }
    return found;
}

static uint8 down_horizon_track_outside_peak(uint16 x,
                                              uint16 previous_x,
                                              float gravity_x,
                                              float first_value,
                                              float last_value,
                                              const float gravity_camera[3],
                                              float cone_cos)
{
    int center = (int)s_column_peak_y[previous_x];
    int first_y = center - DOWN_HORIZON_TRACK_RADIUS;
    int last_y = center + DOWN_HORIZON_TRACK_RADIUS;
    int best_y;
    float best_value;
    int y;

    if (first_value >= 0.0f || last_value >= 0.0f)
    {
        return 0U;
    }
    if (first_y < 0)
    {
        first_y = 0;
    }
    if (last_y >= (int)IMAGE_DOWN_HORIZON_HEIGHT)
    {
        last_y = (int)IMAGE_DOWN_HORIZON_HEIGHT - 1;
    }
    best_y = first_y;
    best_value = down_horizon_column_value(
        x, (uint16)first_y, gravity_x, gravity_camera, cone_cos);
    for (y = first_y + 1; y <= last_y; y++)
    {
        float value = down_horizon_column_value(
            x, (uint16)y, gravity_x, gravity_camera, cone_cos);

        if (value > best_value)
        {
            best_value = value;
            best_y = y;
        }
    }
    if ((best_y == first_y && first_y != 0) ||
        (best_y == last_y &&
         last_y != (int)IMAGE_DOWN_HORIZON_HEIGHT - 1) ||
        best_value > -DOWN_HORIZON_TANGENT_PROBE)
    {
        return 0U;
    }

    down_horizon_store_column(x,
                              first_value,
                              last_value,
                              0U,
                              0.0f,
                              0.0f);
    s_column_peak_y[x] = (uint8)best_y;
    return 1U;
}

static uint8 down_horizon_track_column(uint16 x,
                                       const float gravity_camera[3],
                                       float cone_cos)
{
    uint16 previous_x;
    float gravity_x;
    float first_value;
    float last_value;
    float top_root;
    float bottom_root;

    if (x == 0U)
    {
        return 0U;
    }
    previous_x = (uint16)(x - 1U);

    gravity_x = s_normalized_x[x] * gravity_camera[0];
    first_value = down_horizon_column_value(
        x, 0U, gravity_x, gravity_camera, cone_cos);
    last_value = down_horizon_column_value(
        x, IMAGE_DOWN_HORIZON_HEIGHT - 1U,
        gravity_x, gravity_camera, cone_cos);

    if (s_column_state[previous_x] == DOWN_HORIZON_COLUMN_INSIDE)
    {
        if (first_value >= 0.0f && last_value >= 0.0f)
        {
            down_horizon_store_column(x,
                                      first_value,
                                      last_value,
                                      0U,
                                      0.0f,
                                      0.0f);
            return 1U;
        }
        return 0U;
    }
    if (s_column_state[previous_x] == DOWN_HORIZON_COLUMN_OUTSIDE)
    {
        return down_horizon_track_outside_peak(
            x,
            previous_x,
            gravity_x,
            first_value,
            last_value,
            gravity_camera,
            cone_cos);
    }

    if (s_column_root_count[previous_x] >= 2U)
    {
        float previous_top = g_image_down_horizon_top_y[previous_x];
        float previous_bottom = g_image_down_horizon_bottom_y[previous_x];

        if (previous_bottom - previous_top < DOWN_HORIZON_TRACK_TANGENT_GUARD ||
            ((first_value >= 0.0f) != (last_value >= 0.0f)) ||
            down_horizon_find_root_near(
                x, gravity_x, previous_top,
                gravity_camera, cone_cos, &top_root) == 0U ||
            down_horizon_find_root_near(
                x, gravity_x, previous_bottom,
                gravity_camera, cone_cos, &bottom_root) == 0U ||
            bottom_root - top_root <= DOWN_HORIZON_ROOT_EPSILON)
        {
            return 0U;
        }
        down_horizon_store_column(x,
                                  first_value,
                                  last_value,
                                  2U,
                                  top_root,
                                  bottom_root);
        return 1U;
    }
    if (s_column_root_count[previous_x] == 1U &&
        ((first_value >= 0.0f) != (last_value >= 0.0f)))
    {
        float previous_root =
            (g_image_down_horizon_top_y[previous_x] >= 0.0f &&
             g_image_down_horizon_top_y[previous_x] <=
                 (float)(IMAGE_DOWN_HORIZON_HEIGHT - 1U)) ?
                g_image_down_horizon_top_y[previous_x] :
                g_image_down_horizon_bottom_y[previous_x];

        if (down_horizon_find_root_near(
                x, gravity_x, previous_root,
                gravity_camera, cone_cos, &top_root) != 0U)
        {
            down_horizon_store_column(x,
                                      first_value,
                                      last_value,
                                      1U,
                                      top_root,
                                      top_root);
            return 1U;
        }
    }
    return 0U;
}

static void down_horizon_build_columns(const float gravity_camera[3],
                                       float cone_cos)
{
    uint16 x;

    memset(g_image_down_horizon_column_valid, 0,
           sizeof(g_image_down_horizon_column_valid));
    memset(s_column_state, DOWN_HORIZON_COLUMN_OUTSIDE,
           sizeof(s_column_state));
    memset(s_column_root_count, 0, sizeof(s_column_root_count));
    memset(s_column_peak_y, 0, sizeof(s_column_peak_y));

    for (x = 0U; x < IMAGE_DOWN_HORIZON_WIDTH; x++)
    {
        if (down_horizon_track_column(x, gravity_camera, cone_cos) != 0U)
        {
            continue;
        }
        down_horizon_scan_column(x, gravity_camera, cone_cos);
    }
}

static uint8 down_horizon_build_points(void)
{
    float u[3];
    float v[3];
    float u_norm;
    float boundary_cos = 1.0f;
    float boundary_sin = 0.0f;
    uint16 index;

    if (g_image_down_horizon_valid == 0U)
    {
        return 0U;
    }

    u[0] = 0.0f;
    u[1] = s_last_gravity_body[2];
    u[2] = -s_last_gravity_body[1];
    u_norm = sqrtf(u[1] * u[1] + u[2] * u[2]);
    if (u_norm < 1.0e-6f)
    {
        u[0] = -s_last_gravity_body[2];
        u[1] = 0.0f;
        u[2] = s_last_gravity_body[0];
        u_norm = sqrtf(u[0] * u[0] + u[2] * u[2]);
    }
    u[0] /= u_norm;
    u[1] /= u_norm;
    u[2] /= u_norm;
    v[0] = s_last_gravity_body[1] * u[2] - s_last_gravity_body[2] * u[1];
    v[1] = s_last_gravity_body[2] * u[0] - s_last_gravity_body[0] * u[2];
    v[2] = s_last_gravity_body[0] * u[1] - s_last_gravity_body[1] * u[0];

    for (index = 0U; index < IMAGE_DOWN_HORIZON_POINT_COUNT; index++)
    {
        float direction[3];
        float camera[3];
        float norm;
        float lateral;
        float theta;
        float radius;
        float factor;
        float next_cos;
        uint8 row;

        direction[0] = s_last_height * s_last_gravity_body[0] +
                       DOWN_HORIZON_RANGE_MM *
                       (boundary_cos * u[0] + boundary_sin * v[0]);
        direction[1] = s_last_height * s_last_gravity_body[1] +
                       DOWN_HORIZON_RANGE_MM *
                       (boundary_cos * u[1] + boundary_sin * v[1]);
        direction[2] = s_last_height * s_last_gravity_body[2] +
                       DOWN_HORIZON_RANGE_MM *
                       (boundary_cos * u[2] + boundary_sin * v[2]);
        norm = sqrtf(direction[0] * direction[0] +
                     direction[1] * direction[1] +
                     direction[2] * direction[2]);
        if (norm <= 1.0e-6f)
        {
            return 0U;
        }
        direction[0] /= norm;
        direction[1] /= norm;
        direction[2] /= norm;
        for (row = 0U; row < 3U; row++)
        {
            camera[row] = s_body_to_camera[row][0] * direction[0] +
                          s_body_to_camera[row][1] * direction[1] +
                          s_body_to_camera[row][2] * direction[2];
        }
        if (camera[2] > 1.0f)
        {
            camera[2] = 1.0f;
        }
        else if (camera[2] < -1.0f)
        {
            camera[2] = -1.0f;
        }
        theta = acosf(camera[2]);
        lateral = sqrtf(camera[0] * camera[0] + camera[1] * camera[1]);
        radius = down_horizon_radius(theta);
        if (lateral <= 1.0e-6f)
        {
            g_image_down_horizon_x[index] = DOWN_HORIZON_CENTER_X;
            g_image_down_horizon_y[index] = DOWN_HORIZON_CENTER_Y;
        }
        else
        {
            factor = DOWN_HORIZON_SCALE * radius / lateral;
            g_image_down_horizon_x[index] = DOWN_HORIZON_CENTER_X +
                                             camera[0] * factor;
            g_image_down_horizon_y[index] = DOWN_HORIZON_CENTER_Y +
                                             camera[1] * factor;
        }
        next_cos = boundary_cos * DOWN_HORIZON_STEP_COS -
                   boundary_sin * DOWN_HORIZON_STEP_SIN;
        boundary_sin = boundary_sin * DOWN_HORIZON_STEP_COS +
                       boundary_cos * DOWN_HORIZON_STEP_SIN;
        boundary_cos = next_cos;
    }
    s_points_valid = 1U;
    return 1U;
}

void image_down_horizon_init(void)
{
    float max_x;
    float max_y;
    float radius2_max;
    uint16 index;

    memset(g_image_down_horizon_x, 0, sizeof(g_image_down_horizon_x));
    memset(g_image_down_horizon_y, 0, sizeof(g_image_down_horizon_y));
    memset(g_image_down_horizon_top_y, 0, sizeof(g_image_down_horizon_top_y));
    memset(g_image_down_horizon_bottom_y, 0, sizeof(g_image_down_horizon_bottom_y));
    memset(g_image_down_horizon_column_valid, 0,
           sizeof(g_image_down_horizon_column_valid));
    memset(s_column_state, DOWN_HORIZON_COLUMN_OUTSIDE,
           sizeof(s_column_state));
    memset(s_column_root_count, 0, sizeof(s_column_root_count));
    memset(s_column_peak_y, 0, sizeof(s_column_peak_y));

    for (index = 0U; index < IMAGE_DOWN_HORIZON_WIDTH; index++)
    {
        s_normalized_x[index] = ((float)index - DOWN_HORIZON_CENTER_X) /
                                DOWN_HORIZON_SCALE;
    }
    for (index = 0U; index < IMAGE_DOWN_HORIZON_HEIGHT; index++)
    {
        s_normalized_y[index] = ((float)index - DOWN_HORIZON_CENTER_Y) /
                                DOWN_HORIZON_SCALE;
    }
    max_x = down_horizon_max(down_horizon_abs(s_normalized_x[0]),
                             down_horizon_abs(
                                 s_normalized_x[IMAGE_DOWN_HORIZON_WIDTH - 1U]));
    max_y = down_horizon_max(down_horizon_abs(s_normalized_y[0]),
                             down_horizon_abs(
                                 s_normalized_y[IMAGE_DOWN_HORIZON_HEIGHT - 1U]));
    radius2_max = max_x * max_x + max_y * max_y;
    s_radius2_to_table = (float)DOWN_HORIZON_TABLE_INTERVALS / radius2_max;
    for (index = 0U; index < IMAGE_DOWN_HORIZON_WIDTH; index++)
    {
        s_radius2_table_x[index] = s_normalized_x[index] * s_normalized_x[index] *
                                   s_radius2_to_table;
    }
    for (index = 0U; index < IMAGE_DOWN_HORIZON_HEIGHT; index++)
    {
        s_radius2_table_y[index] = s_normalized_y[index] * s_normalized_y[index] *
                                   s_radius2_to_table;
    }

    for (index = 0U; index < DOWN_HORIZON_TABLE_SIZE; index++)
    {
        float radius2 = radius2_max * (float)index /
                        (float)DOWN_HORIZON_TABLE_INTERVALS;
        float radius = sqrtf(radius2);
        float theta = down_horizon_theta(radius, radius2);

        s_radial_factor[index] = (radius > 1.0e-8f) ?
                                 (sinf(theta) / radius) : DOWN_HORIZON_THETA_K1;
        s_ray_z[index] = cosf(theta);
    }

    g_image_down_horizon_valid = 0U;
    g_image_down_horizon_extrapolated = 0U;
    s_points_valid = 0U;
    s_initialized = 1U;
}

void image_down_horizon_invalidate(void)
{
    g_image_down_horizon_valid = 0U;
    g_image_down_horizon_extrapolated = 0U;
    memset(g_image_down_horizon_column_valid, 0,
           sizeof(g_image_down_horizon_column_valid));
    memset(s_column_state, DOWN_HORIZON_COLUMN_OUTSIDE,
           sizeof(s_column_state));
    memset(s_column_root_count, 0, sizeof(s_column_root_count));
    memset(s_column_peak_y, 0, sizeof(s_column_peak_y));
    s_points_valid = 0U;
}

void image_down_horizon_update(float roll_deg,
                               float pitch_deg,
                               float height_mm,
                               uint8 attitude_valid,
                               uint8 height_valid)
{
    float roll;
    float pitch;
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;
    float gravity_camera[3];
    float cone_cos;
    uint8 row;

    if (s_initialized == 0U)
    {
        image_down_horizon_init();
    }
    if (attitude_valid == 0U || height_valid == 0U ||
        roll_deg != roll_deg || pitch_deg != pitch_deg || height_mm != height_mm)
    {
        image_down_horizon_invalidate();
        return;
    }
    s_last_height = height_mm + DOWN_HORIZON_HEIGHT_BIAS_MM;
    if (s_last_height <= 0.0f)
    {
        image_down_horizon_invalidate();
        return;
    }

    g_image_down_horizon_extrapolated =
        (roll_deg < DOWN_HORIZON_ROLL_MIN_DEG ||
         roll_deg > DOWN_HORIZON_ROLL_MAX_DEG ||
         pitch_deg < DOWN_HORIZON_PITCH_MIN_DEG ||
         pitch_deg > DOWN_HORIZON_PITCH_MAX_DEG ||
         height_mm < DOWN_HORIZON_HEIGHT_MIN_MM ||
         height_mm > DOWN_HORIZON_HEIGHT_MAX_MM) ? 1U : 0U;

    roll = roll_deg * DOWN_HORIZON_DEG_TO_RAD;
    pitch = pitch_deg * DOWN_HORIZON_DEG_TO_RAD;
    sin_roll = sinf(roll);
    cos_roll = cosf(roll);
    sin_pitch = sinf(pitch);
    cos_pitch = cosf(pitch);
    s_last_gravity_body[0] = -sin_pitch;
    s_last_gravity_body[1] = sin_roll * cos_pitch;
    s_last_gravity_body[2] = cos_roll * cos_pitch;

    for (row = 0U; row < 3U; row++)
    {
        gravity_camera[row] = s_body_to_camera[row][0] * s_last_gravity_body[0] +
                              s_body_to_camera[row][1] * s_last_gravity_body[1] +
                              s_body_to_camera[row][2] * s_last_gravity_body[2];
    }
    for (row = 0U; row < IMAGE_DOWN_HORIZON_HEIGHT; row++)
    {
        s_gravity_y[row] = s_normalized_y[row] * gravity_camera[1];
    }
    cone_cos = s_last_height /
               sqrtf(s_last_height * s_last_height +
                     DOWN_HORIZON_RANGE_MM * DOWN_HORIZON_RANGE_MM);
    down_horizon_build_columns(gravity_camera, cone_cos);
    s_points_valid = 0U;
    g_image_down_horizon_valid = 1U;
}

uint8 image_down_horizon_get_point(uint16 index, float *x, float *y)
{
    if (g_image_down_horizon_valid == 0U ||
        index >= IMAGE_DOWN_HORIZON_POINT_COUNT || x == 0 || y == 0)
    {
        return 0U;
    }
    if (s_points_valid == 0U && down_horizon_build_points() == 0U)
    {
        return 0U;
    }
    *x = g_image_down_horizon_x[index];
    *y = g_image_down_horizon_y[index];
    return 1U;
}

uint8 image_down_horizon_get_column(uint16 x, float *top_y, float *bottom_y)
{
    if (g_image_down_horizon_valid == 0U ||
        x >= IMAGE_DOWN_HORIZON_WIDTH ||
        g_image_down_horizon_column_valid[x] == 0U ||
        top_y == 0 || bottom_y == 0)
    {
        return 0U;
    }
    *top_y = g_image_down_horizon_top_y[x];
    *bottom_y = g_image_down_horizon_bottom_y[x];
    return 1U;
}

uint8 image_down_horizon_contains(float x, float y, float margin)
{
    int column;

    if (g_image_down_horizon_valid == 0U ||
        g_image_down_horizon_extrapolated != 0U)
    {
        return 1U;
    }
    column = (int)(x + 0.5f);
    if (column < 0 || column >= (int)IMAGE_DOWN_HORIZON_WIDTH ||
        y < -margin || y > (float)(IMAGE_DOWN_HORIZON_HEIGHT - 1U) + margin)
    {
        return 0U;
    }
    if (s_column_state[column] == DOWN_HORIZON_COLUMN_INSIDE)
    {
        return 1U;
    }
    if (s_column_state[column] != DOWN_HORIZON_COLUMN_PARTIAL)
    {
        return 0U;
    }
    return (y >= g_image_down_horizon_top_y[column] - margin &&
            y <= g_image_down_horizon_bottom_y[column] + margin) ? 1U : 0U;
}
