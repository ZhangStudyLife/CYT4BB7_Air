#include "image_down.h"

#include <math.h>
#include <string.h>

#include "zf_device_mt9v03x.h"
#include "Display/image_debug_screen.h"

#define BEACON_IMAGE_W 188
#define BEACON_IMAGE_H 120
#define BEACON_MAX_CIRCLE_COUNT 8
#define BEACON_MAX_BEACON_COUNT 8
#define BEACON_MAX_CAR_LAMP_COUNT IMAGE_MAX_CAR_LAMP_COUNT

typedef struct
{
    beacon_circle_t circles[BEACON_MAX_CIRCLE_COUNT];
    unsigned char count;
    beacon_circle_t beacons[BEACON_MAX_BEACON_COUNT];
    unsigned char beacon_count;
    beacon_rect_t car_lamps[BEACON_MAX_CAR_LAMP_COUNT];
    unsigned char car_lamp_count;
} beacon_result_t;

#if (MT9V03X_W != BEACON_IMAGE_W) || (MT9V03X_H != BEACON_IMAGE_H)
#error "Beacon image algorithm is tuned for MT9V03X 188x120 frames."
#endif

#define BEACON_THRESHOLD_MIN_VALUE      24
#define BEACON_MIN_COMPONENT_AREA       10
#define BEACON_MAX_COMPONENT_AREA       5000
#define CAR_LAMP_BINARY_THRESHOLD       200
#define LAMP_MASK_PAD                   2
#define LAMP_NEAR_BEACON_PAD            8
#define LAMP_NEAR_BEACON_MIN_AREA       45
#define LAMP_NEAR_BEACON_ISOLATED_MIN_AREA 18
#define LAMP_NEAR_BEACON_BACKGROUND_MAX 40
#define BEACON_SIDE_EDGE_MARGIN         25
#define BEACON_SIDE_EDGE_MIN_AREA       10
#define BEACON_SIDE_EDGE_THRESHOLD      100
#define BEACON_TOP_BOTTOM_EDGE_REJECT_MARGIN 8
#define CLOSE_LAMP_SPLIT_THRESHOLD      250
#define CLOSE_LAMP_MIN_AREA             120
#define CLOSE_LAMP_MAX_AREA             260
#define CLOSE_LAMP_MIN_BBOX_W           14
#define CLOSE_LAMP_MAX_BBOX_W           28
#define CLOSE_LAMP_MIN_BBOX_H           12
#define CLOSE_LAMP_MAX_BBOX_H           28
#define CLOSE_LAMP_MIN_MEAN             235
#define CLOSE_LAMP_MAX_ELONGATION       1.35f
#define CLOSE_LAMP_CORE_MIN_AREA        45
#define CLOSE_LAMP_CORE_MIN_BBOX_W      14
#define CLOSE_LAMP_CORE_MAX_BBOX_H      10
#define CLOSE_BEACON_CORE_MIN_AREA      20
#define CLOSE_BEACON_CORE_MAX_BBOX_W    12
#define CLOSE_BEACON_CORE_MAX_BBOX_H    12
#define CLOSE_CORE_MAX_DISTANCE_SQ      400.0f
#define CLOSE_MERGED_CORE_MIN_AREA      115
#define CLOSE_MERGED_CORE_MAX_AREA      150
#define CLOSE_MERGED_CORE_MIN_BBOX_W    16

/* 核1信标二值化运行时阈值，车端远程设置后从下一帧开始使用。 */
int32 g_image_down_beacon_binary_threshold = 120;
static uint8 s_mt9v03x_initialized;
#define CLOSE_MERGED_CORE_MIN_BBOX_H    14
#define CAR_LAMP_MIN_AREA               24
#define CAR_LAMP_MAX_AREA               1200
#define CAR_LAMP_MIN_ELONGATION         1.6f
#define CAR_LAMP_MIN_LENGTH             8.0f
#define CAR_LAMP_TRACK_START_AREA       30
#define CAR_LAMP_TRACK_START_ELONGATION 1.6f
#define CAR_LAMP_TRACK_START_SCORE      80.0f
#define CAR_LAMP_LARGE_AREA             90
#define CAR_LAMP_LARGE_ELONGATION       1.2f
#define CAR_LAMP_STRONG_AREA            45
#define CAR_LAMP_SMALL_MIN_SPAN         12
#define CAR_LAMP_SMALL_BACKGROUND_MAX   60
#define CAR_LAMP_EDGE_BACKGROUND_MAX    80
#define CAR_LAMP_SIDE_SUN_BACKGROUND_MAX 70
#define CAR_LAMP_SIDE_SUN_MARGIN        16
#define CAR_LAMP_SIDE_SUN_Y             36
#define CAR_LAMP_EDGE_MARGIN            2
#define CAR_LAMP_LOCAL_RING_PAD         3
#define B0_MATCH_DISTANCE               18.0f
#define B0_SWITCH_AREA_RATIO            1.45f
#define B0_INIT_CONFIRM_FRAMES          2
#define BEACON_MAX_MISSES               3
#define CAR_LAMP_EDGE_MAX_MISSES        3
#define CAR_LAMP_CENTER_MAX_MISSES      24
#define CAR_LAMP_TEMPORAL_EDGE_MARGIN   8
#define CAR_LAMP_TEMPORAL_MASK_PAD      5
#define CAR_LAMP_TEMPORAL_CORE_PAD      2
#define CAR_LAMP_TEMPORAL_TAKEOVER_PAD  10
#define CAR_LAMP_TEMPORAL_MIN_BRIGHT_AREA 3
#define KALMAN_GATE_DISTANCE            24.0f
#define KALMAN_NEW_TARGET_DISTANCE      36.0f
#define FILTER_POS_ALPHA                0.65f
#define FILTER_VEL_ALPHA                0.30f

#define IMAGE_QUEUE_SIZE                (BEACON_IMAGE_W * BEACON_IMAGE_H)
#define PI_F                            3.1415926f

typedef struct
{
    int area;
    int min_x;
    int min_y;
    int max_x;
    int max_y;
    float cx;
    float cy;
    float major;
    float minor;
    float elongation;
    float angle;
    unsigned char valid;
} component_t;

typedef struct
{
    unsigned char active;
    unsigned char confirmed;
    unsigned char hits;
    unsigned char misses;
    float x;
    float y;
    float vx;
    float vy;
    float radius;
    float width;
    float length;
    float angle;
} temporal_track_t;

uint8 g_image_frame[MT9V03X_H][MT9V03X_W];

static unsigned char g_binary[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_visit_stamp[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_current_stamp = 0;
static unsigned char g_queue_x[IMAGE_QUEUE_SIZE];
static unsigned char g_queue_y[IMAGE_QUEUE_SIZE];
static const unsigned char (*g_current_image)[BEACON_IMAGE_W] = 0;
static unsigned char g_has_lamp_track = 0;
static temporal_track_t g_b0_track;
static temporal_track_t g_car_track;

static void beacon_image_reset_temporal(void);

static void beacon_image_init(void)
{
    memset(g_binary, 0, sizeof(g_binary));
    memset(g_visit_stamp, 0, sizeof(g_visit_stamp));
    g_current_image = 0;
    g_has_lamp_track = 0;
    g_current_stamp = 0;
    beacon_image_reset_temporal();
}

static void beacon_image_reset_temporal(void)
{
    memset(&g_b0_track, 0, sizeof(g_b0_track));
    memset(&g_car_track, 0, sizeof(g_car_track));
}

static void clear_result(beacon_result_t *result)
{
    if (result != 0)
    {
        memset(result, 0, sizeof(*result));
    }
}

static void begin_visit_pass(void)
{
    g_current_stamp++;
    if (g_current_stamp == 0)
    {
        memset(g_visit_stamp, 0, sizeof(g_visit_stamp));
        g_current_stamp = 1;
    }
}

static unsigned char is_visited(unsigned char x, unsigned char y)
{
    return (g_visit_stamp[y][x] == g_current_stamp) ? 1 : 0;
}

static void mark_visited(unsigned char x, unsigned char y)
{
    g_visit_stamp[y][x] = g_current_stamp;
}

static void threshold_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char threshold)
{
    const unsigned char *src = &image[0][0];
    unsigned char *dst = &g_binary[0][0];
    int i;

    for (i = 0; i < BEACON_IMAGE_W * BEACON_IMAGE_H; i++)
    {
        dst[i] = (src[i] >= threshold) ? 255 : 0;
    }
}

static void threshold_beacon_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    unsigned char x;
    unsigned char y;

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            unsigned char threshold =
                (x < BEACON_SIDE_EDGE_MARGIN ||
                 x >= BEACON_IMAGE_W - BEACON_SIDE_EDGE_MARGIN)
                    ? BEACON_SIDE_EDGE_THRESHOLD
                    : (unsigned char)g_image_down_beacon_binary_threshold;
            g_binary[y][x] = (image[y][x] >= threshold) ? 255 : 0;
        }
    }
}

static component_t grow_component(unsigned char start_x, unsigned char start_y)
{
    static const signed char dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    static const signed char dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
    unsigned short head = 0;
    unsigned short tail = 0;
    int sum_x = 0;
    int sum_y = 0;
    float sum_xx = 0.0f;
    float sum_yy = 0.0f;
    float sum_xy = 0.0f;
    component_t comp;

    memset(&comp, 0, sizeof(comp));
    comp.min_x = start_x;
    comp.max_x = start_x;
    comp.min_y = start_y;
    comp.max_y = start_y;

    g_queue_x[tail] = start_x;
    g_queue_y[tail] = start_y;
    tail++;
    mark_visited(start_x, start_y);

    while (head < tail)
    {
        unsigned char i;
        unsigned char x = g_queue_x[head];
        unsigned char y = g_queue_y[head];
        head++;

        comp.area++;
        sum_x += x;
        sum_y += y;
        sum_xx += (float)x * (float)x;
        sum_yy += (float)y * (float)y;
        sum_xy += (float)x * (float)y;

        if ((int)x < comp.min_x) comp.min_x = x;
        if ((int)x > comp.max_x) comp.max_x = x;
        if ((int)y < comp.min_y) comp.min_y = y;
        if ((int)y > comp.max_y) comp.max_y = y;

        for (i = 0; i < 8; i++)
        {
            int nx = (int)x + dx[i];
            int ny = (int)y + dy[i];

            if (nx < 0 || nx >= BEACON_IMAGE_W ||
                ny < 0 || ny >= BEACON_IMAGE_H)
            {
                continue;
            }
            if (g_binary[ny][nx] == 0 || is_visited((unsigned char)nx, (unsigned char)ny))
            {
                continue;
            }
            if (tail >= IMAGE_QUEUE_SIZE)
            {
                continue;
            }

            mark_visited((unsigned char)nx, (unsigned char)ny);
            g_queue_x[tail] = (unsigned char)nx;
            g_queue_y[tail] = (unsigned char)ny;
            tail++;
        }
    }

    if (comp.area > 0)
    {
        float inv_area = 1.0f / (float)comp.area;
        float var_x;
        float var_y;
        float cov_xy;
        float trace;
        float det;
        float disc;
        float eig_major;
        float eig_minor;

        comp.cx = (float)sum_x * inv_area;
        comp.cy = (float)sum_y * inv_area;
        var_x = sum_xx * inv_area - comp.cx * comp.cx;
        var_y = sum_yy * inv_area - comp.cy * comp.cy;
        cov_xy = sum_xy * inv_area - comp.cx * comp.cy;
        trace = var_x + var_y;
        det = var_x * var_y - cov_xy * cov_xy;
        disc = trace * trace * 0.25f - det;
        if (disc < 0.0f)
        {
            disc = 0.0f;
        }

        eig_major = trace * 0.5f + sqrtf(disc);
        eig_minor = trace * 0.5f - sqrtf(disc);
        if (eig_minor < 0.0f)
        {
            eig_minor = 0.0f;
        }

        comp.major = 4.0f * sqrtf(eig_major + 0.0001f);
        comp.minor = 4.0f * sqrtf(eig_minor + 0.0001f);
        if (comp.minor < 1.0f)
        {
            comp.minor = 1.0f;
        }
        comp.elongation = comp.major / comp.minor;
        comp.angle = 0.5f * atan2f(2.0f * cov_xy, var_x - var_y) * 180.0f / PI_F;
        comp.valid = 1;
    }

    return comp;
}

static int local_background_average(const component_t *comp, int pad)
{
    int min_x = comp->min_x - pad;
    int max_x = comp->max_x + pad;
    int min_y = comp->min_y - pad;
    int max_y = comp->max_y + pad;
    int x;
    int y;
    int sum = 0;
    int count = 0;

    if (g_current_image == 0)
    {
        return 0;
    }
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
        {
            if (x >= comp->min_x && x <= comp->max_x &&
                y >= comp->min_y && y <= comp->max_y)
            {
                continue;
            }
            sum += g_current_image[y][x];
            count++;
        }
    }

    if (count == 0)
    {
        return 0;
    }
    return sum / count;
}

static unsigned char is_lamp_candidate(const component_t *comp)
{
    unsigned char touches_top_or_bottom;
    unsigned char touches_left_or_right;
    int bbox_w = comp->max_x - comp->min_x + 1;
    int bbox_h = comp->max_y - comp->min_y + 1;
    int bbox_span = bbox_w > bbox_h ? bbox_w : bbox_h;

    if (comp->area > CAR_LAMP_MAX_AREA)
    {
        return 0;
    }
    if (g_has_lamp_track == 0 &&
        (comp->area < CAR_LAMP_TRACK_START_AREA ||
         comp->elongation < CAR_LAMP_TRACK_START_ELONGATION ||
         (float)comp->area * comp->elongation < CAR_LAMP_TRACK_START_SCORE))
    {
        return 0;
    }

    touches_top_or_bottom =
        (comp->min_y <= CAR_LAMP_EDGE_MARGIN ||
         comp->max_y >= BEACON_IMAGE_H - 1 - CAR_LAMP_EDGE_MARGIN) ? 1 : 0;
    touches_left_or_right =
        (comp->min_x <= CAR_LAMP_EDGE_MARGIN ||
         comp->max_x >= BEACON_IMAGE_W - 1 - CAR_LAMP_EDGE_MARGIN) ? 1 : 0;

    if (touches_top_or_bottom != 0 && comp->area < CAR_LAMP_MIN_AREA)
    {
        return 0;
    }
    if (comp->area < CAR_LAMP_MIN_AREA)
    {
        return 0;
    }
    if (comp->elongation < CAR_LAMP_MIN_ELONGATION &&
        (comp->area < CAR_LAMP_LARGE_AREA ||
         comp->elongation < CAR_LAMP_LARGE_ELONGATION))
    {
        return 0;
    }
    if (comp->major < CAR_LAMP_MIN_LENGTH)
    {
        return 0;
    }
    if (touches_top_or_bottom != 0 &&
        comp->area < CAR_LAMP_LARGE_AREA &&
        local_background_average(comp, CAR_LAMP_LOCAL_RING_PAD) >
            CAR_LAMP_EDGE_BACKGROUND_MAX)
    {
        return 0;
    }
    if (touches_left_or_right != 0 &&
        comp->cy < (float)BEACON_IMAGE_H * 0.5f &&
        local_background_average(comp, CAR_LAMP_LOCAL_RING_PAD) >
            CAR_LAMP_EDGE_BACKGROUND_MAX)
    {
        return 0;
    }
    if (touches_left_or_right != 0 &&
        comp->cy < CAR_LAMP_SIDE_SUN_Y &&
        local_background_average(comp, CAR_LAMP_LOCAL_RING_PAD) >
            CAR_LAMP_SIDE_SUN_BACKGROUND_MAX)
    {
        return 0;
    }
    if ((comp->min_x <= CAR_LAMP_SIDE_SUN_MARGIN ||
         comp->max_x >= BEACON_IMAGE_W - 1 - CAR_LAMP_SIDE_SUN_MARGIN) &&
        comp->cy < CAR_LAMP_SIDE_SUN_Y &&
        local_background_average(comp, CAR_LAMP_LOCAL_RING_PAD) >
            CAR_LAMP_SIDE_SUN_BACKGROUND_MAX)
    {
        return 0;
    }
    if (comp->area < CAR_LAMP_STRONG_AREA &&
        (bbox_span < CAR_LAMP_SMALL_MIN_SPAN ||
         local_background_average(comp, CAR_LAMP_LOCAL_RING_PAD) >
             CAR_LAMP_SMALL_BACKGROUND_MAX))
    {
        return 0;
    }
    return 1;
}

static float lamp_score(const component_t *comp)
{
    return (float)comp->area * comp->elongation;
}

static unsigned char is_edge_lamp_candidate(const component_t *comp)
{
    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    if (comp->min_y > 2 && comp->max_y < BEACON_IMAGE_H - 3)
    {
        return 0;
    }
    return (comp->elongation >= 2.6f &&
            comp->major >= 16.0f &&
            comp->minor <= 6.5f) ? 1 : 0;
}

static unsigned char find_car_lamp(component_t *best_lamp)
{
    unsigned char x;
    unsigned char y;
    float best_score = 0.0f;
    unsigned char found = 0;

    memset(best_lamp, 0, sizeof(*best_lamp));
    begin_visit_pass();

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;
            float score;

            if (g_binary[y][x] == 0 || is_visited(x, y))
            {
                continue;
            }

            comp = grow_component(x, y);
            if (!is_lamp_candidate(&comp))
            {
                continue;
            }

            score = lamp_score(&comp);
            if (found == 0 || score > best_score)
            {
                *best_lamp = comp;
                best_score = score;
                found = 1;
            }
        }
    }

    return found;
}

static unsigned char find_edge_car_lamp(component_t *best_lamp)
{
    unsigned char x;
    unsigned char y;
    float best_score = 0.0f;
    unsigned char found = 0;

    memset(best_lamp, 0, sizeof(*best_lamp));
    begin_visit_pass();

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;
            float score;

            if (g_binary[y][x] == 0 || is_visited(x, y))
            {
                continue;
            }

            comp = grow_component(x, y);
            if (!is_edge_lamp_candidate(&comp))
            {
                continue;
            }

            score = lamp_score(&comp);
            if (found == 0 || score > best_score)
            {
                *best_lamp = comp;
                best_score = score;
                found = 1;
            }
        }
    }

    return found;
}

static int component_mean_gray(const component_t *comp)
{
    int x;
    int y;
    int sum = 0;
    int count = 0;

    if (comp == 0 || comp->valid == 0 || g_current_image == 0)
    {
        return 0;
    }

    for (y = comp->min_y; y <= comp->max_y; y++)
    {
        for (x = comp->min_x; x <= comp->max_x; x++)
        {
            if (g_current_image[y][x] >= CAR_LAMP_BINARY_THRESHOLD)
            {
                sum += g_current_image[y][x];
                count++;
            }
        }
    }

    if (count == 0)
    {
        return 0;
    }
    return sum / count;
}

static unsigned char is_close_compound_candidate(const component_t *comp)
{
    int bbox_w;
    int bbox_h;

    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }

    bbox_w = comp->max_x - comp->min_x + 1;
    bbox_h = comp->max_y - comp->min_y + 1;
    if (comp->area < CLOSE_LAMP_MIN_AREA ||
        comp->area > CLOSE_LAMP_MAX_AREA ||
        bbox_w < CLOSE_LAMP_MIN_BBOX_W ||
        bbox_w > CLOSE_LAMP_MAX_BBOX_W ||
        bbox_h < CLOSE_LAMP_MIN_BBOX_H ||
        bbox_h > CLOSE_LAMP_MAX_BBOX_H ||
        comp->elongation > CLOSE_LAMP_MAX_ELONGATION)
    {
        return 0;
    }

    return component_mean_gray(comp) >= CLOSE_LAMP_MIN_MEAN ? 1 : 0;
}

static component_t grow_local_threshold_component(
    unsigned char start_x,
    unsigned char start_y,
    const component_t *bounds)
{
    static const signed char dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    static const signed char dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
    unsigned short head = 0;
    unsigned short tail = 0;
    int sum_x = 0;
    int sum_y = 0;
    float sum_xx = 0.0f;
    float sum_yy = 0.0f;
    float sum_xy = 0.0f;
    component_t comp;

    memset(&comp, 0, sizeof(comp));
    comp.min_x = start_x;
    comp.max_x = start_x;
    comp.min_y = start_y;
    comp.max_y = start_y;

    g_queue_x[tail] = start_x;
    g_queue_y[tail] = start_y;
    tail++;
    mark_visited(start_x, start_y);

    while (head < tail)
    {
        unsigned char i;
        unsigned char x = g_queue_x[head];
        unsigned char y = g_queue_y[head];
        head++;

        comp.area++;
        sum_x += x;
        sum_y += y;
        sum_xx += (float)x * (float)x;
        sum_yy += (float)y * (float)y;
        sum_xy += (float)x * (float)y;

        if ((int)x < comp.min_x) comp.min_x = x;
        if ((int)x > comp.max_x) comp.max_x = x;
        if ((int)y < comp.min_y) comp.min_y = y;
        if ((int)y > comp.max_y) comp.max_y = y;

        for (i = 0; i < 8; i++)
        {
            int nx = (int)x + dx[i];
            int ny = (int)y + dy[i];

            if (nx < bounds->min_x || nx > bounds->max_x ||
                ny < bounds->min_y || ny > bounds->max_y)
            {
                continue;
            }
            if (g_current_image[ny][nx] < CLOSE_LAMP_SPLIT_THRESHOLD ||
                is_visited((unsigned char)nx, (unsigned char)ny))
            {
                continue;
            }
            if (tail >= IMAGE_QUEUE_SIZE)
            {
                continue;
            }

            mark_visited((unsigned char)nx, (unsigned char)ny);
            g_queue_x[tail] = (unsigned char)nx;
            g_queue_y[tail] = (unsigned char)ny;
            tail++;
        }
    }

    if (comp.area > 0)
    {
        float inv_area = 1.0f / (float)comp.area;
        float var_x;
        float var_y;
        float cov_xy;
        float trace;
        float det;
        float disc;
        float eig_major;
        float eig_minor;

        comp.cx = (float)sum_x * inv_area;
        comp.cy = (float)sum_y * inv_area;
        var_x = sum_xx * inv_area - comp.cx * comp.cx;
        var_y = sum_yy * inv_area - comp.cy * comp.cy;
        cov_xy = sum_xy * inv_area - comp.cx * comp.cy;
        trace = var_x + var_y;
        det = var_x * var_y - cov_xy * cov_xy;
        disc = trace * trace * 0.25f - det;
        if (disc < 0.0f)
        {
            disc = 0.0f;
        }

        eig_major = trace * 0.5f + sqrtf(disc);
        eig_minor = trace * 0.5f - sqrtf(disc);
        if (eig_minor < 0.0f)
        {
            eig_minor = 0.0f;
        }

        comp.major = 4.0f * sqrtf(eig_major + 0.0001f);
        comp.minor = 4.0f * sqrtf(eig_minor + 0.0001f);
        if (comp.minor < 1.0f)
        {
            comp.minor = 1.0f;
        }
        comp.elongation = comp.major / comp.minor;
        comp.angle = 0.5f * atan2f(2.0f * cov_xy, var_x - var_y) * 180.0f / PI_F;
        comp.valid = 1;
    }

    return comp;
}

static unsigned char is_close_lamp_core(const component_t *comp)
{
    int bbox_w;
    int bbox_h;

    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }

    bbox_w = comp->max_x - comp->min_x + 1;
    bbox_h = comp->max_y - comp->min_y + 1;
    return (comp->area >= CLOSE_LAMP_CORE_MIN_AREA &&
            bbox_w >= CLOSE_LAMP_CORE_MIN_BBOX_W &&
            bbox_h <= CLOSE_LAMP_CORE_MAX_BBOX_H) ? 1 : 0;
}

static unsigned char is_close_beacon_core(const component_t *comp)
{
    int bbox_w;
    int bbox_h;

    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }

    bbox_w = comp->max_x - comp->min_x + 1;
    bbox_h = comp->max_y - comp->min_y + 1;
    return (comp->area >= CLOSE_BEACON_CORE_MIN_AREA &&
            bbox_w <= CLOSE_BEACON_CORE_MAX_BBOX_W &&
            bbox_h <= CLOSE_BEACON_CORE_MAX_BBOX_H) ? 1 : 0;
}

static unsigned char is_close_merged_core(const component_t *comp)
{
    int bbox_w;
    int bbox_h;

    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }

    bbox_w = comp->max_x - comp->min_x + 1;
    bbox_h = comp->max_y - comp->min_y + 1;
    return (comp->area >= CLOSE_MERGED_CORE_MIN_AREA &&
            comp->area <= CLOSE_MERGED_CORE_MAX_AREA &&
            bbox_w >= CLOSE_MERGED_CORE_MIN_BBOX_W &&
            bbox_h >= CLOSE_MERGED_CORE_MIN_BBOX_H) ? 1 : 0;
}

static unsigned char make_lower_lamp_core(
    const component_t *merged_core,
    component_t *lamp_core)
{
    int y_split;
    int x;
    int y;
    int sum_x = 0;
    int sum_y = 0;
    float sum_xx = 0.0f;
    float sum_yy = 0.0f;
    float sum_xy = 0.0f;

    if (merged_core == 0 || merged_core->valid == 0 || lamp_core == 0)
    {
        return 0;
    }

    memset(lamp_core, 0, sizeof(*lamp_core));
    y_split = (merged_core->min_y + merged_core->max_y + 1) / 2;
    lamp_core->min_x = BEACON_IMAGE_W - 1;
    lamp_core->min_y = BEACON_IMAGE_H - 1;

    for (y = y_split; y <= merged_core->max_y; y++)
    {
        for (x = merged_core->min_x; x <= merged_core->max_x; x++)
        {
            if (g_current_image[y][x] < CLOSE_LAMP_SPLIT_THRESHOLD)
            {
                continue;
            }

            lamp_core->area++;
            sum_x += x;
            sum_y += y;
            sum_xx += (float)x * (float)x;
            sum_yy += (float)y * (float)y;
            sum_xy += (float)x * (float)y;

            if (x < lamp_core->min_x) lamp_core->min_x = x;
            if (x > lamp_core->max_x) lamp_core->max_x = x;
            if (y < lamp_core->min_y) lamp_core->min_y = y;
            if (y > lamp_core->max_y) lamp_core->max_y = y;
        }
    }

    if (lamp_core->area > 0)
    {
        float inv_area = 1.0f / (float)lamp_core->area;
        float var_x;
        float var_y;
        float cov_xy;
        float trace;
        float det;
        float disc;
        float eig_major;
        float eig_minor;

        lamp_core->cx = (float)sum_x * inv_area;
        lamp_core->cy = (float)sum_y * inv_area;
        var_x = sum_xx * inv_area - lamp_core->cx * lamp_core->cx;
        var_y = sum_yy * inv_area - lamp_core->cy * lamp_core->cy;
        cov_xy = sum_xy * inv_area - lamp_core->cx * lamp_core->cy;
        trace = var_x + var_y;
        det = var_x * var_y - cov_xy * cov_xy;
        disc = trace * trace * 0.25f - det;
        if (disc < 0.0f)
        {
            disc = 0.0f;
        }

        eig_major = trace * 0.5f + sqrtf(disc);
        eig_minor = trace * 0.5f - sqrtf(disc);
        if (eig_minor < 0.0f)
        {
            eig_minor = 0.0f;
        }

        lamp_core->major = 4.0f * sqrtf(eig_major + 0.0001f);
        lamp_core->minor = 4.0f * sqrtf(eig_minor + 0.0001f);
        if (lamp_core->minor < 1.0f)
        {
            lamp_core->minor = 1.0f;
        }
        lamp_core->elongation = lamp_core->major / lamp_core->minor;
        lamp_core->angle =
            0.5f * atan2f(2.0f * cov_xy, var_x - var_y) * 180.0f / PI_F;
        lamp_core->valid = 1;
    }

    return is_close_lamp_core(lamp_core);
}

static unsigned char split_close_compound_lamp(
    const component_t *compound,
    component_t *best_lamp)
{
    unsigned char x;
    unsigned char y;
    component_t lamp_core;
    component_t beacon_core;
    component_t merged_core;
    unsigned char has_lamp_core = 0;
    unsigned char has_beacon_core = 0;
    unsigned char has_merged_core = 0;

    memset(&lamp_core, 0, sizeof(lamp_core));
    memset(&beacon_core, 0, sizeof(beacon_core));
    memset(&merged_core, 0, sizeof(merged_core));
    begin_visit_pass();

    for (y = (unsigned char)compound->min_y; y <= (unsigned char)compound->max_y; y++)
    {
        for (x = (unsigned char)compound->min_x; x <= (unsigned char)compound->max_x; x++)
        {
            component_t comp;

            if (g_current_image[y][x] < CLOSE_LAMP_SPLIT_THRESHOLD || is_visited(x, y))
            {
                continue;
            }

            comp = grow_local_threshold_component(x, y, compound);
            if (is_close_lamp_core(&comp) != 0 &&
                (has_lamp_core == 0 || comp.area > lamp_core.area))
            {
                lamp_core = comp;
                has_lamp_core = 1;
            }
            if (is_close_beacon_core(&comp) != 0 &&
                (has_beacon_core == 0 || comp.area > beacon_core.area))
            {
                beacon_core = comp;
                has_beacon_core = 1;
            }
            if (is_close_merged_core(&comp) != 0 &&
                (has_merged_core == 0 || comp.area > merged_core.area))
            {
                merged_core = comp;
                has_merged_core = 1;
            }
        }
    }

    if (has_lamp_core != 0 && has_beacon_core != 0)
    {
        float dx = lamp_core.cx - beacon_core.cx;
        float dy = lamp_core.cy - beacon_core.cy;
        if (dx * dx + dy * dy <= CLOSE_CORE_MAX_DISTANCE_SQ)
        {
            *best_lamp = lamp_core;
            return 1;
        }
    }
    if (has_merged_core != 0 && make_lower_lamp_core(&merged_core, best_lamp) != 0)
    {
        return 1;
    }

    return 0;
}

static unsigned char find_compound_close_lamp(component_t *best_lamp)
{
    unsigned char x;
    unsigned char y;
    float best_score = 0.0f;
    unsigned char found = 0;

    memset(best_lamp, 0, sizeof(*best_lamp));
    begin_visit_pass();

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;
            component_t lamp_core;
            float score;

            if (g_binary[y][x] == 0 || is_visited(x, y))
            {
                continue;
            }

            comp = grow_component(x, y);
            if (is_close_compound_candidate(&comp) == 0)
            {
                continue;
            }
            if (split_close_compound_lamp(&comp, &lamp_core) == 0)
            {
                continue;
            }

            score = lamp_score(&lamp_core);
            if (found == 0 || score > best_score)
            {
                *best_lamp = lamp_core;
                best_score = score;
                found = 1;
            }
        }
    }

    return found;
}

static void fill_car_lamp_rect(const component_t *lamp, beacon_rect_t *rect)
{
    rect->cx = lamp->cx - (float)BEACON_IMAGE_W * 0.5f;
    rect->cy = lamp->cy - (float)BEACON_IMAGE_H * 0.5f;
    rect->length = lamp->major;
    rect->width = lamp->minor;
    rect->angle = lamp->angle;
    rect->valid = 1;
}

static void write_car_lamp(const component_t *lamp, beacon_result_t *result)
{
    beacon_rect_t *rect;

    if (lamp->valid == 0)
    {
        result->car_lamp_count = 0;
        return;
    }

    rect = &result->car_lamps[0];
    fill_car_lamp_rect(lamp, rect);
    result->car_lamp_count = 1;
}

static void erase_lamp_rect_from_binary(const component_t *lamp)
{
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int x;
    int y;

    if (lamp == 0 || lamp->valid == 0)
    {
        return;
    }

    min_x = lamp->min_x - LAMP_MASK_PAD;
    max_x = lamp->max_x + LAMP_MASK_PAD;
    min_y = lamp->min_y - LAMP_MASK_PAD;
    max_y = lamp->max_y + LAMP_MASK_PAD;

    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
        {
            g_binary[y][x] = 0;
        }
    }
}

static unsigned char component_from_temporal_car(
    const temporal_track_t *track,
    component_t *lamp,
    unsigned char predicted)
{
    float image_cx;
    float image_cy;
    float half_len;
    float half_wid;
    float radius;

    if (track == 0 || lamp == 0 || track->confirmed == 0 ||
        track->length <= 0.0f || track->width <= 0.0f)
    {
        return 0;
    }

    memset(lamp, 0, sizeof(*lamp));
    image_cx = track->x + (float)BEACON_IMAGE_W * 0.5f;
    image_cy = track->y + (float)BEACON_IMAGE_H * 0.5f;
    if (predicted != 0)
    {
        image_cx += track->vx;
        image_cy += track->vy;
    }

    half_len = track->length * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;
    half_wid = track->width * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;
    radius = sqrtf(half_len * half_len + half_wid * half_wid);

    lamp->cx = image_cx;
    lamp->cy = image_cy;
    lamp->major = track->length;
    lamp->minor = track->width;
    lamp->angle = track->angle;
    lamp->area = (int)(track->length * track->width + 0.5f);
    lamp->min_x = (int)(image_cx - radius);
    lamp->max_x = (int)(image_cx + radius);
    lamp->min_y = (int)(image_cy - radius);
    lamp->max_y = (int)(image_cy + radius);
    if (lamp->min_x < 0) lamp->min_x = 0;
    if (lamp->min_y < 0) lamp->min_y = 0;
    if (lamp->max_x >= BEACON_IMAGE_W) lamp->max_x = BEACON_IMAGE_W - 1;
    if (lamp->max_y >= BEACON_IMAGE_H) lamp->max_y = BEACON_IMAGE_H - 1;
    lamp->valid = 1;
    return 1;
}

static void erase_temporal_lamp_from_binary(const component_t *lamp)
{
    int x;
    int y;
    float angle;
    float cos_a;
    float sin_a;
    float half_len;
    float half_wid;

    if (lamp == 0 || lamp->valid == 0)
    {
        return;
    }

    angle = lamp->angle * (PI_F / 180.0f);
    cos_a = cosf(angle);
    sin_a = sinf(angle);
    half_len = lamp->major * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;
    half_wid = lamp->minor * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;

    for (y = lamp->min_y; y <= lamp->max_y; y++)
    {
        for (x = lamp->min_x; x <= lamp->max_x; x++)
        {
            float dx = (float)x - lamp->cx;
            float dy = (float)y - lamp->cy;
            float major = dx * cos_a + dy * sin_a;
            float minor = -dx * sin_a + dy * cos_a;
            if (fabsf(major) <= half_len && fabsf(minor) <= half_wid)
            {
                g_binary[y][x] = 0;
            }
        }
    }
}

static unsigned char is_component_in_lamp_core(const component_t *comp, const component_t *lamp)
{
    float angle;
    float cos_a;
    float sin_a;
    float dx;
    float dy;
    float major;
    float minor;

    if (comp == 0 || comp->valid == 0 || lamp == 0 || lamp->valid == 0)
    {
        return 0;
    }

    angle = lamp->angle * (PI_F / 180.0f);
    cos_a = cosf(angle);
    sin_a = sinf(angle);
    dx = comp->cx - lamp->cx;
    dy = comp->cy - lamp->cy;
    major = dx * cos_a + dy * sin_a;
    minor = -dx * sin_a + dy * cos_a;

    return (fabsf(major) <= lamp->major * 0.5f + (float)CAR_LAMP_TEMPORAL_CORE_PAD &&
            fabsf(minor) <= lamp->minor * 0.5f + (float)CAR_LAMP_TEMPORAL_CORE_PAD) ? 1 : 0;
}

static unsigned char is_near_lamp(const component_t *comp, const component_t *lamp)
{
    if (comp == 0 || lamp == 0 || lamp->valid == 0)
    {
        return 0;
    }
    return (comp->max_x >= lamp->min_x - LAMP_NEAR_BEACON_PAD &&
            comp->min_x <= lamp->max_x + LAMP_NEAR_BEACON_PAD &&
            comp->max_y >= lamp->min_y - LAMP_NEAR_BEACON_PAD &&
            comp->min_y <= lamp->max_y + LAMP_NEAR_BEACON_PAD) ? 1 : 0;
}

static unsigned char is_side_edge_beacon(const component_t *comp)
{
    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    return (comp->min_x < BEACON_SIDE_EDGE_MARGIN ||
            comp->max_x >= BEACON_IMAGE_W - BEACON_SIDE_EDGE_MARGIN) ? 1 : 0;
}

static unsigned char is_isolated_near_lamp_beacon(const component_t *comp)
{
    if (comp == 0 || comp->valid == 0 || comp->area < LAMP_NEAR_BEACON_ISOLATED_MIN_AREA)
    {
        return 0;
    }

    return local_background_average(comp, CAR_LAMP_LOCAL_RING_PAD) <=
        LAMP_NEAR_BEACON_BACKGROUND_MAX ? 1 : 0;
}

static void insert_beacon_by_area(
    const component_t *comp,
    const component_t *lamp,
    const component_t *temporal_lamp,
    beacon_result_t *result)
{
    int i;
    int slot;
    int min_area;
    beacon_circle_t circle;

    if (comp == 0 || comp->valid == 0)
    {
        return;
    }
    if ((lamp == 0 || lamp->valid == 0) &&
        (temporal_lamp == 0 || temporal_lamp->valid == 0) &&
        (comp->max_y <= BEACON_TOP_BOTTOM_EDGE_REJECT_MARGIN ||
         comp->min_y >= BEACON_IMAGE_H - 1 - BEACON_TOP_BOTTOM_EDGE_REJECT_MARGIN))
    {
        return;
    }
    min_area = is_side_edge_beacon(comp) != 0
        ? BEACON_SIDE_EDGE_MIN_AREA
        : BEACON_MIN_COMPONENT_AREA;
    if (comp->area < min_area)
    {
        return;
    }
    if (is_component_in_lamp_core(comp, lamp) != 0 ||
        is_component_in_lamp_core(comp, temporal_lamp) != 0)
    {
        return;
    }
    if ((is_near_lamp(comp, lamp) != 0 ||
         is_near_lamp(comp, temporal_lamp) != 0) &&
        comp->area < LAMP_NEAR_BEACON_MIN_AREA &&
        is_isolated_near_lamp_beacon(comp) == 0)
    {
        return;
    }

    slot = result->beacon_count;
    if (slot >= BEACON_MAX_BEACON_COUNT)
    {
        slot = BEACON_MAX_BEACON_COUNT - 1;
        if ((float)comp->area <= result->beacons[slot].radius * result->beacons[slot].radius * PI_F)
        {
            return;
        }
    }
    else
    {
        result->beacon_count++;
    }

    for (i = slot - 1; i >= 0; i--)
    {
        if ((float)comp->area <= result->beacons[i].radius * result->beacons[i].radius * PI_F)
        {
            break;
        }
        result->beacons[i + 1] = result->beacons[i];
    }

    circle.x = comp->cx - (float)BEACON_IMAGE_W * 0.5f;
    circle.y = comp->cy - (float)BEACON_IMAGE_H * 0.5f;
    circle.radius = sqrtf((float)comp->area / PI_F);
    circle.area = (float)comp->area;
    circle.valid = 1;
    result->beacons[i + 1] = circle;
}

static void sync_legacy_beacons(beacon_result_t *result)
{
    int i;
    int count = result->beacon_count;

    if (count > BEACON_MAX_CIRCLE_COUNT)
    {
        count = BEACON_MAX_CIRCLE_COUNT;
    }

    result->count = (unsigned char)count;
    for (i = 0; i < count; i++)
    {
        result->circles[i] = result->beacons[i];
    }
}

static void find_beacons(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *lamp,
    const component_t *temporal_lamp,
    beacon_result_t *result)
{
    unsigned char x;
    unsigned char y;

    result->beacon_count = 0;
    threshold_beacon_image(image);
    erase_lamp_rect_from_binary(lamp);
    erase_temporal_lamp_from_binary(temporal_lamp);
    begin_visit_pass();

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;

            if (g_binary[y][x] == 0 || is_visited(x, y))
            {
                continue;
            }

            comp = grow_component(x, y);
            insert_beacon_by_area(&comp, lamp, temporal_lamp, result);
        }
    }

    sync_legacy_beacons(result);
}

static float square_distance(float ax, float ay, float bx, float by)
{
    float dx = ax - bx;
    float dy = ay - by;

    return dx * dx + dy * dy;
}

static float beacon_area(const beacon_circle_t *beacon)
{
    return beacon->radius * beacon->radius * PI_F;
}

static void reset_track(temporal_track_t *track)
{
    memset(track, 0, sizeof(*track));
}

static unsigned char predict_missed_track(temporal_track_t *track, unsigned char max_misses)
{
    if (track->confirmed != 0 && track->misses < max_misses)
    {
        track->x += track->vx;
        track->y += track->vy;
        track->misses++;
        return 1;
    }

    reset_track(track);
    return 0;
}

static void start_pending_track(temporal_track_t *track, float x, float y)
{
    reset_track(track);
    track->active = 1;
    track->hits = 1;
    track->x = x;
    track->y = y;
}

static void update_track_position(temporal_track_t *track, float x, float y)
{
    float old_x = track->x;
    float old_y = track->y;
    float predict_x = track->x + track->vx;
    float predict_y = track->y + track->vy;

    track->vx = (1.0f - FILTER_VEL_ALPHA) * track->vx + FILTER_VEL_ALPHA * (x - old_x);
    track->vy = (1.0f - FILTER_VEL_ALPHA) * track->vy + FILTER_VEL_ALPHA * (y - old_y);
    track->x = FILTER_POS_ALPHA * x + (1.0f - FILTER_POS_ALPHA) * predict_x;
    track->y = FILTER_POS_ALPHA * y + (1.0f - FILTER_POS_ALPHA) * predict_y;
    track->misses = 0;
}

static void set_beacon_track_shape(temporal_track_t *track, const beacon_circle_t *beacon)
{
    track->radius = beacon->radius;
}

static void set_car_track_shape(temporal_track_t *track, const beacon_rect_t *car)
{
    track->width = car->width;
    track->length = car->length;
    track->angle = car->angle;
}

static void output_temporal_beacon(const temporal_track_t *track, beacon_result_t *result)
{
    beacon_circle_t beacon;
    int i;
    int matched = -1;
    float best_d2 = B0_MATCH_DISTANCE * B0_MATCH_DISTANCE;

    memset(&beacon, 0, sizeof(beacon));
    beacon.x = track->x;
    beacon.y = track->y;
    beacon.radius = track->radius;
    beacon.area = track->radius * track->radius * PI_F;
    beacon.valid = 1;

    for (i = 0; i < result->beacon_count; i++)
    {
        float d2 = square_distance(result->beacons[i].x, result->beacons[i].y,
                                   beacon.x, beacon.y);
        if (d2 <= best_d2)
        {
            best_d2 = d2;
            matched = i;
        }
    }

    if (matched == 0)
    {
        result->beacons[0] = beacon;
        sync_legacy_beacons(result);
        return;
    }

    if (matched > 0)
    {
        for (i = matched; i < result->beacon_count - 1; i++)
        {
            result->beacons[i] = result->beacons[i + 1];
        }
        result->beacon_count--;
    }

    if (result->beacon_count >= BEACON_MAX_BEACON_COUNT)
    {
        result->beacon_count = BEACON_MAX_BEACON_COUNT - 1;
    }
    for (i = result->beacon_count; i > 0; i--)
    {
        result->beacons[i] = result->beacons[i - 1];
    }
    result->beacons[0] = beacon;
    result->beacon_count++;
    sync_legacy_beacons(result);
}

static void output_temporal_car(const temporal_track_t *track, beacon_result_t *result)
{
    beacon_rect_t *car = &result->car_lamps[0];

    car->cx = track->x;
    car->cy = track->y;
    car->width = track->width;
    car->length = track->length;
    car->angle = track->angle;
    car->valid = 1;
    if (result->car_lamp_count == 0)
    {
        result->car_lamp_count = 1;
    }
}

static unsigned char temporal_car_max_misses(const temporal_track_t *track)
{
    component_t lamp;

    if (component_from_temporal_car(track, &lamp, 1) == 0)
    {
        return CAR_LAMP_EDGE_MAX_MISSES;
    }

    if (lamp.min_x <= CAR_LAMP_TEMPORAL_EDGE_MARGIN ||
        lamp.min_y <= CAR_LAMP_TEMPORAL_EDGE_MARGIN ||
        lamp.max_x >= BEACON_IMAGE_W - 1 - CAR_LAMP_TEMPORAL_EDGE_MARGIN ||
        lamp.max_y >= BEACON_IMAGE_H - 1 - CAR_LAMP_TEMPORAL_EDGE_MARGIN)
    {
        return CAR_LAMP_EDGE_MAX_MISSES;
    }

    return CAR_LAMP_CENTER_MAX_MISSES;
}

static unsigned char can_use_temporal_car_mask(const temporal_track_t *track)
{
    return (track != 0 &&
            track->confirmed != 0 &&
            track->misses < temporal_car_max_misses(track)) ? 1 : 0;
}

static unsigned char find_temporal_car_lamp(component_t *best_lamp)
{
    component_t temporal_lamp;
    component_t best_comp;
    unsigned char found = 0;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    unsigned char x;
    unsigned char y;

    if (best_lamp == 0 || g_current_image == 0 ||
        component_from_temporal_car(&g_car_track, &temporal_lamp, 1) == 0)
    {
        return 0;
    }

    memset(&best_comp, 0, sizeof(best_comp));

    min_x = temporal_lamp.min_x - CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    max_x = temporal_lamp.max_x + CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    min_y = temporal_lamp.min_y - CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    max_y = temporal_lamp.max_y + CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    threshold_image(g_current_image, (unsigned char)g_image_down_beacon_binary_threshold);
    begin_visit_pass();
    for (y = (unsigned char)min_y; y <= (unsigned char)max_y; y++)
    {
        for (x = (unsigned char)min_x; x <= (unsigned char)max_x; x++)
        {
            component_t comp;

            if (g_binary[y][x] == 0 || is_visited(x, y))
            {
                continue;
            }

            comp = grow_component(x, y);
            if (comp.area >= CAR_LAMP_TEMPORAL_MIN_BRIGHT_AREA &&
                is_component_in_lamp_core(&comp, &temporal_lamp) != 0)
            {
                if (found == 0 || comp.area > best_comp.area)
                {
                    best_comp = comp;
                    found = 1;
                }
            }
        }
    }

    if (found != 0)
    {
        *best_lamp = best_comp;
    }

    return found;
}

static int nearest_beacon_index(const beacon_result_t *result, float x, float y)
{
    int i;
    int best = -1;
    float best_d2 = B0_MATCH_DISTANCE * B0_MATCH_DISTANCE;

    for (i = 0; i < result->beacon_count; i++)
    {
        float d2 = square_distance(result->beacons[i].x, result->beacons[i].y, x, y);
        if (d2 <= best_d2)
        {
            best_d2 = d2;
            best = i;
        }
    }

    return best;
}

static unsigned char update_temporal_beacon(beacon_result_t *result)
{
    int selected = -1;
    beacon_circle_t *measurement;

    if (result->beacon_count == 0)
    {
        return predict_missed_track(&g_b0_track, BEACON_MAX_MISSES);
    }

    if (g_b0_track.confirmed != 0)
    {
        float predict_x = g_b0_track.x + g_b0_track.vx;
        float predict_y = g_b0_track.y + g_b0_track.vy;

        selected = nearest_beacon_index(result, predict_x, predict_y);
        if (selected > 0 &&
            beacon_area(&result->beacons[0]) > beacon_area(&result->beacons[selected]) * B0_SWITCH_AREA_RATIO)
        {
            selected = 0;
        }
        if (selected < 0 &&
            square_distance(predict_x, predict_y, result->beacons[0].x, result->beacons[0].y) >
                KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE)
        {
            start_pending_track(&g_b0_track, result->beacons[0].x, result->beacons[0].y);
            set_beacon_track_shape(&g_b0_track, &result->beacons[0]);
            return 0;
        }
    }
    else
    {
        selected = 0;
    }

    if (selected < 0)
    {
        return predict_missed_track(&g_b0_track, BEACON_MAX_MISSES);
    }

    measurement = &result->beacons[selected];
    if (g_b0_track.active == 0)
    {
        start_pending_track(&g_b0_track, measurement->x, measurement->y);
        set_beacon_track_shape(&g_b0_track, measurement);
        return 0;
    }

    if (g_b0_track.confirmed == 0)
    {
        if (square_distance(g_b0_track.x, g_b0_track.y, measurement->x, measurement->y) >
            KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE)
        {
            start_pending_track(&g_b0_track, measurement->x, measurement->y);
            set_beacon_track_shape(&g_b0_track, measurement);
            return 0;
        }
        update_track_position(&g_b0_track, measurement->x, measurement->y);
        set_beacon_track_shape(&g_b0_track, measurement);
        g_b0_track.hits++;
        if (g_b0_track.hits >= B0_INIT_CONFIRM_FRAMES)
        {
            g_b0_track.confirmed = 1;
            return 1;
        }
        return 0;
    }

    if (square_distance(g_b0_track.x + g_b0_track.vx, g_b0_track.y + g_b0_track.vy,
                        measurement->x, measurement->y) >
        KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE)
    {
        start_pending_track(&g_b0_track, measurement->x, measurement->y);
        set_beacon_track_shape(&g_b0_track, measurement);
        return 0;
    }

    update_track_position(&g_b0_track, measurement->x, measurement->y);
    set_beacon_track_shape(&g_b0_track, measurement);
    return 1;
}

static unsigned char update_temporal_car(beacon_result_t *result)
{
    beacon_rect_t *measurement = 0;

    if (result->car_lamp_count > 0 && result->car_lamps[0].valid != 0)
    {
        measurement = &result->car_lamps[0];
    }
    if (measurement == 0)
    {
        return predict_missed_track(&g_car_track,
                                    temporal_car_max_misses(&g_car_track));
    }

    if (g_car_track.active == 0)
    {
        start_pending_track(&g_car_track, measurement->cx, measurement->cy);
        set_car_track_shape(&g_car_track, measurement);
        return 0;
    }
    if (g_car_track.confirmed != 0 &&
        square_distance(g_car_track.x + g_car_track.vx, g_car_track.y + g_car_track.vy,
                        measurement->cx, measurement->cy) >
            KALMAN_GATE_DISTANCE * KALMAN_GATE_DISTANCE)
    {
        start_pending_track(&g_car_track, measurement->cx, measurement->cy);
        set_car_track_shape(&g_car_track, measurement);
        return 0;
    }
    if (g_car_track.confirmed == 0 &&
        square_distance(g_car_track.x, g_car_track.y, measurement->cx, measurement->cy) >
            KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE)
    {
        start_pending_track(&g_car_track, measurement->cx, measurement->cy);
        set_car_track_shape(&g_car_track, measurement);
        return 0;
    }

    update_track_position(&g_car_track, measurement->cx, measurement->cy);
    set_car_track_shape(&g_car_track, measurement);
    g_car_track.hits++;
    if (g_car_track.confirmed == 0 && g_car_track.hits >= B0_INIT_CONFIRM_FRAMES)
    {
        g_car_track.confirmed = 1;
    }

    return g_car_track.confirmed;
}

static void apply_temporal_beacon(beacon_result_t *result)
{
    if (update_temporal_beacon(result) == 0)
    {
        return;
    }

    output_temporal_beacon(&g_b0_track, result);
}

static void apply_temporal_car(beacon_result_t *result)
{
    if (update_temporal_car(result) == 0)
    {
        return;
    }

    output_temporal_car(&g_car_track, result);
}

static void update_temporal_result(beacon_result_t *result)
{
    apply_temporal_beacon(result);
    apply_temporal_car(result);
}

static void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    component_t lamp;
    component_t temporal_lamp;
    unsigned char has_lamp;
    int i;

    if (result == 0)
    {
        return;
    }

    clear_result(result);
    if (image == 0)
    {
        return;
    }

    g_current_image = image;
    memset(&temporal_lamp, 0, sizeof(temporal_lamp));
    threshold_image(image, CAR_LAMP_BINARY_THRESHOLD);
    has_lamp = find_car_lamp(&lamp);
    if (has_lamp == 0)
    {
        has_lamp = find_edge_car_lamp(&lamp);
    }
    if (has_lamp == 0)
    {
        has_lamp = find_compound_close_lamp(&lamp);
    }
    if (has_lamp == 0)
    {
        has_lamp = find_temporal_car_lamp(&lamp);
    }

    if (has_lamp == 0)
    {
        memset(&lamp, 0, sizeof(lamp));
        g_has_lamp_track = 0;
    }
    else
    {
        g_has_lamp_track = 1;
    }

    if (can_use_temporal_car_mask(&g_car_track) != 0)
    {
        (void)component_from_temporal_car(&g_car_track, &temporal_lamp, 1);
    }

    write_car_lamp(&lamp, result);
    find_beacons(image, &lamp, &temporal_lamp, result);
    update_temporal_result(result);

    for (i = result->car_lamp_count; i < BEACON_MAX_CAR_LAMP_COUNT; i++)
    {
        result->car_lamps[i].valid = 0;
    }
}

static uint8 image_down_latch_frame(void)
{
    if(0U == mt9v03x_finish_flag)
    {
        return 0U;
    }

    memcpy(g_image_frame[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    mt9v03x_finish_flag = 0U;
    return 1U;
}

static void image_down_clear_results(void)
{
    image_data_clear(&image_data[Center]);
}

static void image_down_store_result(const beacon_result_t *result)
{
    uint8 i;
    uint8 beacon_count = result->beacon_count;
    uint8 car_lamp_count = result->car_lamp_count;

    image_down_clear_results();

    if(beacon_count > IMAGE_MAX_BEACON_COUNT)
    {
        beacon_count = IMAGE_MAX_BEACON_COUNT;
    }
    for(i = 0U; i < beacon_count; i++)
    {
        image_data[Center].beacon_data[i].valid = result->beacons[i].valid;
        image_data[Center].beacon_data[i].x = result->beacons[i].x;
        image_data[Center].beacon_data[i].y = result->beacons[i].y;
        image_data[Center].beacon_data[i].area = result->beacons[i].area;
    }

    if(car_lamp_count > IMAGE_MAX_CAR_LAMP_COUNT)
    {
        car_lamp_count = IMAGE_MAX_CAR_LAMP_COUNT;
    }
    for(i = 0U; i < car_lamp_count; i++)
    {
        image_data[Center].car_lamp_data[i].valid = result->car_lamps[i].valid;
        image_data[Center].car_lamp_data[i].cx = result->car_lamps[i].cx;
        image_data[Center].car_lamp_data[i].cy = result->car_lamps[i].cy;
        image_data[Center].car_lamp_data[i].width = result->car_lamps[i].width;
        image_data[Center].car_lamp_data[i].length = result->car_lamps[i].length;
        image_data[Center].car_lamp_data[i].angle = result->car_lamps[i].angle;
    }
}

void image_down_init(void)
{
    memset(g_image_frame, 0, MT9V03X_IMAGE_SIZE);
    image_down_clear_results();
    beacon_image_init();
    mt9v03x_finish_flag = 0U;
    s_mt9v03x_initialized = (mt9v03x_init() == 0U) ? 1U : 0U;
}

uint8 image_down_update(void)
{
    beacon_result_t result;

    if(0U == image_down_latch_frame())
    {
        return 0U;
    }

    beacon_image_process(g_image_frame, &result);
    image_down_store_result(&result);
    return 1U;
}

uint8 *image_down_get_frame_buffer(void)
{
    return g_image_frame[0];
}

const uint8 *image_down_get_binary_buffer(void)
{
    return g_binary[0];
}

/*
 * 函数功能: 在核1图像帧边界执行图像参数SET/GET，并返回实际读回值。
 * 输入参数: op操作码；type数值类型；param_id参数ID；value_bits目标值位模式；actual_bits实际值位模式输出。
 * 返回值: IPC_REMOTE_PARAM_STATUS_*统一状态码。
 */
uint8 image_down_remote_param_execute(uint8 op,
                                      uint8 type,
                                      uint16 param_id,
                                      uint32 value_bits,
                                      uint32 *actual_bits)
{
    int32 value;

    if(actual_bits == NULL)
    {
        return IPC_REMOTE_PARAM_STATUS_ERROR;
    }
    if(type != IPC_REMOTE_PARAM_TYPE_INT32)
    {
        *actual_bits = 0U;
        return IPC_REMOTE_PARAM_STATUS_NOT_FOUND;
    }

    if(param_id == IPC_REMOTE_PARAM_ID_BEACON_THRESHOLD)
    {
        if(op == IPC_REMOTE_PARAM_OP_SET)
        {
            value = (int32)value_bits;
            if((value < 0) || (value > 255))
            {
                *actual_bits = (uint32)g_image_down_beacon_binary_threshold;
                return IPC_REMOTE_PARAM_STATUS_OUT_OF_RANGE;
            }
            g_image_down_beacon_binary_threshold = value;
        }
        else if(op != IPC_REMOTE_PARAM_OP_GET)
        {
            *actual_bits = (uint32)g_image_down_beacon_binary_threshold;
            return IPC_REMOTE_PARAM_STATUS_ERROR;
        }

        *actual_bits = (uint32)g_image_down_beacon_binary_threshold;
        return IPC_REMOTE_PARAM_STATUS_OK;
    }

    if(param_id == IPC_REMOTE_PARAM_ID_EXP_TIME)
    {
        uint16 old_value = g_mt9v03x_exp_time;

        if(op == IPC_REMOTE_PARAM_OP_SET)
        {
            value = (int32)value_bits;
            if((value < 0) || (value > 636))
            {
                *actual_bits = (uint32)g_mt9v03x_exp_time;
                return IPC_REMOTE_PARAM_STATUS_OUT_OF_RANGE;
            }
            if(((uint16)value == old_value) &&
               (s_mt9v03x_initialized != 0U))
            {
                *actual_bits = (uint32)g_mt9v03x_exp_time;
                return IPC_REMOTE_PARAM_STATUS_OK;
            }

            if(((uint16)value != old_value) ||
               (s_mt9v03x_initialized == 0U))
            {
                g_mt9v03x_exp_time = (uint16)value;
                s_mt9v03x_initialized = 0U;
                if(mt9v03x_init() != 0U)
                {
                    g_mt9v03x_exp_time = old_value;
                    if(mt9v03x_init() != 0U)
                    {
                        *actual_bits = (uint32)g_mt9v03x_exp_time;
                        return IPC_REMOTE_PARAM_STATUS_ROLLBACK_FAIL;
                    }
                    s_mt9v03x_initialized = 1U;
                    *actual_bits = (uint32)g_mt9v03x_exp_time;
                    return IPC_REMOTE_PARAM_STATUS_ERROR;
                }
                s_mt9v03x_initialized = 1U;
            }
        }
        else if(op != IPC_REMOTE_PARAM_OP_GET)
        {
            *actual_bits = (uint32)g_mt9v03x_exp_time;
            return IPC_REMOTE_PARAM_STATUS_ERROR;
        }

        *actual_bits = (uint32)g_mt9v03x_exp_time;
        return (s_mt9v03x_initialized != 0U) ?
               IPC_REMOTE_PARAM_STATUS_OK : IPC_REMOTE_PARAM_STATUS_ERROR;
    }

    if(param_id == IPC_REMOTE_PARAM_ID_SCREEN_MODE)
    {
        if(op == IPC_REMOTE_PARAM_OP_SET)
        {
            value = (int32)value_bits;
            if((value < (int32)IMAGE_DEBUG_SCREEN_MODE_DATA) ||
               (value > (int32)IMAGE_DEBUG_SCREEN_MODE_BINARY))
            {
                *actual_bits = (uint32)ImageDebugScreen_GetMode();
                return IPC_REMOTE_PARAM_STATUS_OUT_OF_RANGE;
            }
            (void)ImageDebugScreen_SetMode((uint8)value);
        }
        else if(op != IPC_REMOTE_PARAM_OP_GET)
        {
            *actual_bits = (uint32)ImageDebugScreen_GetMode();
            return IPC_REMOTE_PARAM_STATUS_ERROR;
        }

        *actual_bits = (uint32)ImageDebugScreen_GetMode();
        return IPC_REMOTE_PARAM_STATUS_OK;
    }

    *actual_bits = 0U;
    return IPC_REMOTE_PARAM_STATUS_NOT_FOUND;
}
