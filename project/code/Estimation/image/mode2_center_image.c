#include "mode2_center_image.h"

#include <math.h>
#include <string.h>

#include "zf_device_mt9v03x.h"

#define BEACON_IMAGE_W 188
#define BEACON_IMAGE_H 120
#define BEACON_MAX_CIRCLE_COUNT 8
#define BEACON_MAX_BEACON_COUNT 8
#define BEACON_MAX_CAR_LAMP_COUNT MODE2_CENTER_IMAGE_MAX_CAR_LAMP_COUNT

typedef struct
{
    mode2_center_image_beacon_t circles[BEACON_MAX_CIRCLE_COUNT];
    unsigned char count;
    mode2_center_image_beacon_t beacons[BEACON_MAX_BEACON_COUNT];
    unsigned char beacon_count;
    mode2_center_image_car_lamp_t car_lamps[BEACON_MAX_CAR_LAMP_COUNT];
    unsigned char car_lamp_count;
} beacon_result_t;

#if (MT9V03X_W != BEACON_IMAGE_W) || (MT9V03X_H != BEACON_IMAGE_H)
#error "Beacon image algorithm is tuned for MT9V03X 188x120 frames."
#endif

/* Normal threshold used for beacon segmentation after car-lamp masking. */
#define BEACON_BINARY_THRESHOLD       200
#define BEACON_BEACON_LOW_THRESHOLD   150
#define BEACON_BEACON_TINY_THRESHOLD  130
#define BEACON_BEACON_MICRO_THRESHOLD 100

/* High threshold used to isolate the very bright car lamp strip. */
#define BEACON_CAR_LAMP_THRESHOLD     200

/* Connected-component area filters. */
#define BEACON_MIN_COMPONENT_AREA     3
#define BEACON_MAX_COMPONENT_AREA     5000
#define BEACON_MIN_LAMP_AREA          24
#define BEACON_MAX_LAMP_AREA          1200
#define BEACON_MIN_LAMP_ELONGATION    1.6f
#define BEACON_MIN_LAMP_LENGTH        8.0f
#define BEACON_LAMP_TRACK_START_AREA  45
#define BEACON_LAMP_TRACK_START_ELONGATION 2.0f
#define BEACON_LAMP_TRACK_START_SCORE 120.0f
#define BEACON_STRONG_LAMP_AREA       45
#define BEACON_SMALL_LAMP_MIN_SPAN    12
#define BEACON_SMALL_LAMP_BACKGROUND_MAX 60
#define BEACON_LAMP_MASK_PAD          4
#define BEACON_LAMP_COMPANION_ELONGATION 1.6f
#define BEACON_LAMP_COMPANION_DISTANCE 36.0f
#define BEACON_EDGE_LAMP_BACKGROUND_MAX 80
#define BEACON_SIDE_SUN_LAMP_BACKGROUND_MAX 70
#define BEACON_SIDE_SUN_LAMP_MARGIN  16
#define BEACON_SIDE_SUN_LAMP_Y       36
#define BEACON_TOP_SUN_LAMP_BACKGROUND_MAX 84
#define BEACON_TOP_SUN_LAMP_Y        18
#define BEACON_EDGE_LAMP_MIN_AREA     18
#define BEACON_EDGE_LAMP_MARGIN       2
#define BEACON_EDGE_LAMP_MIN_SPAN     5
#define BEACON_MAX_BEACON_AREA        260
#define BEACON_LOW_BEACON_MAX_AREA    200
#define BEACON_MAX_BEACON_ELONGATION  5.0f
#define BEACON_NORMAL_BEACON_MIN_AREA 3
#define BEACON_NORMAL_BEACON_MAX_AREA 260
#define BEACON_TOP_BEACON_Y           24.0f
#define BEACON_TOP_BEACON_MIN_AREA    3
#define BEACON_TOP_BEACON_MAX_AREA    260
#define BEACON_TOP_BEACON_MAX_ELONGATION 5.0f
#define BEACON_MID_BEACON_Y           56.0f
#define BEACON_MID_BEACON_MIN_AREA    3
#define BEACON_MID_BEACON_MAX_AREA    260
#define BEACON_LAMP_RESIDUE_ELONGATION 1.65f
#define BEACON_LAMP_RESIDUE_DISTANCE  24.0f
#define BEACON_EDGE_BEACON_MIN_AREA   3
#define BEACON_EDGE_BEACON_MAX_AREA   200
#define BEACON_EDGE_BEACON_MAX_SPAN   30
#define BEACON_EDGE_BEACON_MARGIN     1
#define BEACON_SIDE_EDGE_TINY_MARGIN  64.0f
#define BEACON_LOWER_SIDE_TINY_MARGIN 64.0f
#define BEACON_CORNER_EDGE_TINY_MARGIN 56.0f
#define BEACON_SIDE_EDGE_TINY_MIN_AREA 3
#define BEACON_SIDE_EDGE_TINY_MAX_AREA 30
#define BEACON_SIDE_EDGE_TINY_MIN_Y   24.0f
#define BEACON_SIDE_EDGE_TINY_MAX_Y   100.0f
#define BEACON_CORNER_EDGE_TINY_MAX_Y 24.0f
#define BEACON_LOWER_SIDE_TINY_MIN_Y  100.0f
#define BEACON_SIDE_EDGE_TINY_MAX_ELONGATION 3.7f
#define BEACON_SIDE_EDGE_TINY_MIN_RADIUS 0.9f
#define BEACON_SIDE_EDGE_TINY_BACKGROUND_MAX 45
#define BEACON_TOP_TINY_MAX_Y         20.0f
#define BEACON_TOP_TINY_MIN_AREA      3
#define BEACON_TOP_TINY_MAX_AREA      18
#define BEACON_TOP_TINY_MAX_SPAN      7
#define BEACON_TOP_TINY_MAX_ELONGATION 3.6f
#define BEACON_TOP_TINY_LINE_MAX_AREA 6
#define BEACON_TOP_TINY_LINE_MAX_SPAN 5
#define BEACON_TOP_TINY_LINE_MAX_ELONGATION 4.8f
#define BEACON_TOP_TINY_BACKGROUND_MAX 40
#define BEACON_TOP_TINY_MIN_PEAK      230
#define BEACON_TOP_TINY_LOW_PEAK      220
#define BEACON_TOP_TINY_LOW_PEAK_BACKGROUND_MAX 10
#define BEACON_TOP_WIDE_LAMP_MAX_Y    36.0f
#define BEACON_TOP_WIDE_LAMP_MIN_AREA 160
#define BEACON_TOP_WIDE_LAMP_MIN_WIDTH 18
#define BEACON_TOP_WIDE_LAMP_MIN_ELONGATION 1.25f
#define BEACON_TOP_WIDE_LAMP_BACKGROUND_MAX 130
#define BEACON_NO_LAMP_SMALL_AREA     80
#define BEACON_NO_LAMP_SMALL_ELONGATION 1.5f
#define BEACON_DARK_SMALL_BACKGROUND_MAX 30
#define BEACON_LOCAL_CONTRAST_AREA    40
#define BEACON_LAMP_TINY_Y            40.0f
#define BEACON_LAMP_TINY_MAX_AREA     200
#define BEACON_LAMP_TINY_BACKGROUND_MAX 30
#define BEACON_LAMP_TINY_FAR_MAX_AREA 12
#define BEACON_LAMP_TINY_FAR_MAX_SPAN 6
#define BEACON_LAMP_TINY_FAR_DISTANCE 48.0f
#define BEACON_LAMP_TINY_FAR_MAX_Y    100.0f
#define BEACON_LAMP_MICRO_MAX_AREA    200
#define BEACON_LAMP_MICRO_BACKGROUND_MAX 25
#define BEACON_LOCAL_RING_PAD         3
#define BEACON_LOCAL_BACKGROUND_MAX   50
#define BEACON_NEW_SLOT_MIN_AREA      3
#define BEACON_OUTPUT_MIN_AREA        18
#define BEACON_NEW_SLOT_MAX_ELONGATION 5.0f
#define BEACON_NEW_SLOT_EDGE_MARGIN   1
#define BEACON_LAMP_CONTEXT_FRAMES    12
#define BEACON_NEW_SLOT_FRAMES_AFTER_LAMP 120
#define BEACON_LAMP_LOST_BEACON_MIN_Y 100.0f
#define BEACON_LAMP_NEW_SLOT_NEAR_AREA 28
#define BEACON_LAMP_NEW_SLOT_DISTANCE 32.0f
#define BEACON_LAMP_NEW_SLOT_EDGE_MARGIN 4
#define BEACON_LAMP_SPLIT_MIN_SOURCE_ELONGATION 2.4f
#define BEACON_LAMP_SPLIT_MIN_SOURCE_AREA 180
#define BEACON_LAMP_SPLIT_MIN_SOURCE_Y   24.0f
#define BEACON_LAMP_SPLIT_MIN_SIDE_WIDTH 6
#define BEACON_LAMP_SPLIT_MIN_SIDE_AREA 18
#define BEACON_LAMP_SPLIT_BEACON_MAX_SPAN 18
#define BEACON_LAMP_SPLIT_BEACON_MAX_ELONGATION 1.9f
#define BEACON_LAMP_SPLIT_RANK_EPS 0.5f
#define BEACON_TOP_EDGE_Y             12.0f
#define BEACON_TOP_EDGE_MIN_AREA      40
#define BEACON_TOP_EDGE_MAX_AREA      150
#define BEACON_TOP_EDGE_MAX_SPAN      16
#define BEACON_TOP_EDGE_MAX_ELONGATION 1.6f
#define BEACON_TOP_EDGE_BACKGROUND_MAX 80
#define BEACON_BOTTOM_EDGE_Y          107.0f
#define BEACON_BOTTOM_EDGE_MIN_AREA   40
#define BEACON_BOTTOM_EDGE_MAX_AREA   150
#define BEACON_BOTTOM_EDGE_MAX_SPAN   16
#define BEACON_BOTTOM_EDGE_MAX_ELONGATION 1.6f
#define BEACON_BOTTOM_EDGE_BACKGROUND_MAX 80
#define BEACON_REACQUIRE_MIN_Y        BEACON_MID_BEACON_Y
#define BEACON_REACQUIRE_MIN_AREA     80
#define BEACON_REACQUIRE_MAX_AREA     260
#define BEACON_REACQUIRE_MAX_ELONGATION 1.45f
#define BEACON_REACQUIRE_MIN_RADIUS   4.5f
#define BEACON_STACKED_SPLIT_MIN_Y    80.0f
#define BEACON_STACKED_SPLIT_MIN_AREA 160
#define BEACON_STACKED_SPLIT_MAX_AREA 360
#define BEACON_STACKED_SPLIT_MIN_HEIGHT 15
#define BEACON_STACKED_SPLIT_BEACON_MIN_AREA 45
#define BEACON_STACKED_SPLIT_BEACON_MAX_SPAN 18
#define BEACON_STACKED_SPLIT_BEACON_MAX_ELONGATION 1.8f
#define BEACON_STACKED_SPLIT_LAMP_MIN_AREA 45
#define BEACON_STACKED_SPLIT_LAMP_MIN_WIDTH 14
#define BEACON_STACKED_SPLIT_LAMP_MIN_ELONGATION 1.6f
#define BEACON_STACKED_SPLIT_OUTPUT_MAX_SPAN 18
#define BEACON_LAMP_SPLIT_TRACK_PENALTY 16.0f
#define BEACON_TRACKED_SLOT_LIMIT     4
#define BEACON_DUPLICATE_DISTANCE     5.0f
#define BEACON_TRACK_MATCH_DISTANCE   36.0f
#define BEACON_CANDIDATE_SCORE_EPS    0.0001f
#define BEACON_CANDIDATE_CENTER_EPS   0.01f
#define BEACON_EDGE_DISTANCE_EPS      0.01f

/* Queue size = image pixel count. */
#define BEACON_QUEUE_SIZE             (BEACON_IMAGE_W * BEACON_IMAGE_H)

#define PI_F 3.1415926f
#define MAX_INTERNAL_BEACONS 16

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
    float radius;
    unsigned char max_value;
    unsigned char valid;
    unsigned char used;
} component_t;

typedef struct
{
    mode2_center_image_beacon_t circle;
    int area;
    float score;
    float elongation;
    float image_x;
    float image_y;
    unsigned char max_value;
    unsigned char background;
    unsigned char from_lamp_split;
    unsigned char valid;
    unsigned char used;
} beacon_candidate_t;

static unsigned char g_binary[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_mask[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_visit_stamp[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_current_stamp = 0;
static unsigned char g_queue_x[BEACON_QUEUE_SIZE];
static unsigned char g_queue_y[BEACON_QUEUE_SIZE];
static beacon_candidate_t g_beacon_candidates[MAX_INTERNAL_BEACONS];
static unsigned char g_beacon_candidate_count = 0;
static component_t g_lamp_mask_components[BEACON_MAX_CAR_LAMP_COUNT + 3];
static unsigned char g_lamp_mask_component_count = 0;
static component_t g_lamp_split_beacons[BEACON_MAX_BEACON_COUNT];
static unsigned char g_lamp_split_beacon_count = 0;
static mode2_center_image_beacon_t g_last_beacons[BEACON_MAX_BEACON_COUNT];
static unsigned char g_has_last_beacons = 0;
static const unsigned char (*g_current_image)[BEACON_IMAGE_W] = 0;
static unsigned char g_has_current_lamp = 0;
static unsigned char g_had_lamp_previous = 0;
static unsigned char g_has_lamp_track = 0;
static unsigned char g_seen_lamp = 0;
static unsigned char g_new_slot_frames = 0;
static unsigned char g_lamp_context_frames = 0;
static unsigned char g_beacon_pass_threshold = BEACON_BINARY_THRESHOLD;
static component_t g_current_lamp;

static float squaref_local(float value)
{
    return value * value;
}

static float vertical_edge_distance(float image_y)
{
    float bottom_distance = (float)(BEACON_IMAGE_H - 1) - image_y;
    return image_y < bottom_distance ? image_y : bottom_distance;
}

static float horizontal_edge_distance(float image_x)
{
    float right_distance = (float)(BEACON_IMAGE_W - 1) - image_x;
    return image_x < right_distance ? image_x : right_distance;
}

static unsigned char is_near_vertical_edge(float image_y, float margin)
{
    return (vertical_edge_distance(image_y) <=
        margin + BEACON_EDGE_DISTANCE_EPS) ? 1 : 0;
}

static unsigned char is_near_horizontal_edge(float image_x, float margin)
{
    return (horizontal_edge_distance(image_x) <=
        margin + BEACON_EDGE_DISTANCE_EPS) ? 1 : 0;
}

static unsigned char is_in_mirrored_lower_edge_band(float image_y, float lower_y)
{
    return is_near_vertical_edge(image_y, (float)(BEACON_IMAGE_H - 1) - lower_y);
}

static void clear_result(beacon_result_t *result)
{
    memset(result, 0, sizeof(*result));
}

static void beacon_image_init(void)
{
    memset(g_binary, 0, sizeof(g_binary));
    memset(g_mask, 0, sizeof(g_mask));
    memset(g_visit_stamp, 0, sizeof(g_visit_stamp));
    memset(g_beacon_candidates, 0, sizeof(g_beacon_candidates));
    memset(g_lamp_mask_components, 0, sizeof(g_lamp_mask_components));
    memset(g_lamp_split_beacons, 0, sizeof(g_lamp_split_beacons));
    memset(g_last_beacons, 0, sizeof(g_last_beacons));
    memset(&g_current_lamp, 0, sizeof(g_current_lamp));
    g_beacon_candidate_count = 0;
    g_lamp_mask_component_count = 0;
    g_lamp_split_beacon_count = 0;
    g_has_last_beacons = 0;
    g_had_lamp_previous = 0;
    g_has_lamp_track = 0;
    g_seen_lamp = 0;
    g_new_slot_frames = 0;
    g_lamp_context_frames = 0;
    g_current_stamp = 0;
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
    unsigned char threshold,
    unsigned char use_mask)
{
    const unsigned char *src = &image[0][0];
    unsigned char *dst = &g_binary[0][0];
    const unsigned char *mask = &g_mask[0][0];
    int i;

    for (i = 0; i < BEACON_IMAGE_W * BEACON_IMAGE_H; i++)
    {
        dst[i] = (src[i] >= threshold && (use_mask == 0 || mask[i] == 0)) ? 255 : 0;
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
    comp.max_value = g_current_image != 0 ? g_current_image[start_y][start_x] : 0U;

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
        if (g_current_image != 0 && g_current_image[y][x] > comp.max_value)
        {
            comp.max_value = g_current_image[y][x];
        }
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

            if (nx < 0 || nx >= BEACON_IMAGE_W || ny < 0 || ny >= BEACON_IMAGE_H)
            {
                continue;
            }
            if (g_binary[ny][nx] == 0)
            {
                continue;
            }
            if (is_visited((unsigned char)nx, (unsigned char)ny))
            {
                continue;
            }
            if (tail >= BEACON_QUEUE_SIZE)
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
        comp.radius = sqrtf((float)comp.area / PI_F);
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

static float beacon_candidate_score(const component_t *comp)
{
    float roundness = 1.0f / comp->elongation;
    if (roundness > 1.0f)
    {
        roundness = 1.0f;
    }
    return (float)comp->area * roundness;
}

static float image_center_distance2(float image_x, float image_y)
{
    float dx = image_x - ((float)(BEACON_IMAGE_W - 1) * 0.5f);
    float dy = image_y - ((float)(BEACON_IMAGE_H - 1) * 0.5f);
    return dx * dx + dy * dy;
}

static unsigned char component_has_higher_rank(
    const component_t *comp,
    int background,
    const beacon_candidate_t *candidate)
{
    float score;
    float comp_center_distance;
    float candidate_center_distance;

    if (comp == 0 || candidate == 0 || candidate->valid == 0)
    {
        return 0;
    }

    score = beacon_candidate_score(comp);
    if (score > candidate->score + BEACON_CANDIDATE_SCORE_EPS)
    {
        return 1;
    }
    if (score + BEACON_CANDIDATE_SCORE_EPS < candidate->score)
    {
        return 0;
    }
    if (comp->area > candidate->area)
    {
        return 1;
    }
    if (comp->area < candidate->area)
    {
        return 0;
    }
    if (comp->max_value > candidate->max_value)
    {
        return 1;
    }
    if (comp->max_value < candidate->max_value)
    {
        return 0;
    }
    if (background < (int)candidate->background)
    {
        return 1;
    }
    if (background > (int)candidate->background)
    {
        return 0;
    }

    comp_center_distance = image_center_distance2(comp->cx, comp->cy);
    candidate_center_distance =
        image_center_distance2(candidate->image_x, candidate->image_y);
    if (comp_center_distance + BEACON_CANDIDATE_CENTER_EPS <
        candidate_center_distance)
    {
        return 1;
    }
    if (candidate_center_distance + BEACON_CANDIDATE_CENTER_EPS <
        comp_center_distance)
    {
        return 0;
    }

    return (comp->elongation + BEACON_CANDIDATE_SCORE_EPS <
        candidate->elongation) ? 1 : 0;
}

static unsigned char duplicate_component_replaces_candidate(
    const component_t *comp,
    int background,
    const beacon_candidate_t *candidate)
{
    if (comp == 0 || candidate == 0 || candidate->valid == 0)
    {
        return 0;
    }
    if (comp->area > candidate->area)
    {
        return 1;
    }
    if (comp->area < candidate->area)
    {
        return 0;
    }
    return component_has_higher_rank(comp, background, candidate);
}

static unsigned char candidate_has_higher_rank(
    const beacon_candidate_t *candidate,
    const beacon_candidate_t *other)
{
    float candidate_center_distance;
    float other_center_distance;

    if (candidate == 0 || other == 0 ||
        candidate->valid == 0 || other->valid == 0)
    {
        return 0;
    }
    if (candidate->score > other->score + BEACON_CANDIDATE_SCORE_EPS)
    {
        return 1;
    }
    if (candidate->score + BEACON_CANDIDATE_SCORE_EPS < other->score)
    {
        return 0;
    }
    if (candidate->area > other->area)
    {
        return 1;
    }
    if (candidate->area < other->area)
    {
        return 0;
    }
    if (candidate->max_value > other->max_value)
    {
        return 1;
    }
    if (candidate->max_value < other->max_value)
    {
        return 0;
    }
    if (candidate->background < other->background)
    {
        return 1;
    }
    if (candidate->background > other->background)
    {
        return 0;
    }

    candidate_center_distance =
        image_center_distance2(candidate->image_x, candidate->image_y);
    other_center_distance =
        image_center_distance2(other->image_x, other->image_y);
    if (candidate_center_distance + BEACON_CANDIDATE_CENTER_EPS <
        other_center_distance)
    {
        return 1;
    }
    if (other_center_distance + BEACON_CANDIDATE_CENTER_EPS <
        candidate_center_distance)
    {
        return 0;
    }

    return (candidate->elongation + BEACON_CANDIDATE_SCORE_EPS <
        other->elongation) ? 1 : 0;
}

static void sort_beacon_candidates(void)
{
    int i;

    for (i = 1; i < g_beacon_candidate_count; i++)
    {
        int j = i;
        beacon_candidate_t candidate = g_beacon_candidates[i];
        while (j > 0 &&
               candidate_has_higher_rank(
                   &candidate,
                   &g_beacon_candidates[j - 1]) != 0)
        {
            g_beacon_candidates[j] = g_beacon_candidates[j - 1];
            j--;
        }
        g_beacon_candidates[j] = candidate;
    }
}

static unsigned char is_vertical_edge_wide_lamp_shape(
    const component_t *comp,
    int background)
{
    int width;

    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    width = comp->max_x - comp->min_x + 1;
    if (is_near_vertical_edge(comp->cy, BEACON_TOP_WIDE_LAMP_MAX_Y) == 0 ||
        comp->area < BEACON_TOP_WIDE_LAMP_MIN_AREA ||
        width < BEACON_TOP_WIDE_LAMP_MIN_WIDTH)
    {
        return 0;
    }
    if (comp->elongation < BEACON_TOP_WIDE_LAMP_MIN_ELONGATION)
    {
        return 0;
    }
    return (background <= BEACON_TOP_WIDE_LAMP_BACKGROUND_MAX) ? 1 : 0;
}

static unsigned char is_lamp_candidate(const component_t *comp)
{
    unsigned char touches_top_or_bottom;
    unsigned char touches_left_or_right;
    int bbox_w = comp->max_x - comp->min_x + 1;
    int bbox_h = comp->max_y - comp->min_y + 1;
    int bbox_span = bbox_w > bbox_h ? bbox_w : bbox_h;
    int background = local_background_average(comp, BEACON_LOCAL_RING_PAD);

    if (comp->area > BEACON_MAX_LAMP_AREA)
    {
        return 0;
    }
    if (is_vertical_edge_wide_lamp_shape(comp, background) != 0)
    {
        return 1;
    }
    if (g_has_lamp_track == 0 &&
        (comp->area < BEACON_LAMP_TRACK_START_AREA ||
         comp->elongation < BEACON_LAMP_TRACK_START_ELONGATION ||
         (float)comp->area * comp->elongation < BEACON_LAMP_TRACK_START_SCORE))
    {
        return 0;
    }
    touches_top_or_bottom =
        (comp->min_y <= BEACON_EDGE_LAMP_MARGIN ||
         comp->max_y >= BEACON_IMAGE_H - 1 - BEACON_EDGE_LAMP_MARGIN) ? 1 : 0;
    touches_left_or_right =
        (comp->min_x <= BEACON_EDGE_LAMP_MARGIN ||
         comp->max_x >= BEACON_IMAGE_W - 1 - BEACON_EDGE_LAMP_MARGIN) ? 1 : 0;
    if (touches_top_or_bottom != 0 && comp->area < BEACON_MIN_LAMP_AREA)
    {
        return 0;
    }
    if (comp->area < BEACON_MIN_LAMP_AREA)
    {
        return 0;
    }
    if (comp->elongation < BEACON_MIN_LAMP_ELONGATION)
    {
        return 0;
    }
    if (comp->major < BEACON_MIN_LAMP_LENGTH)
    {
        return 0;
    }
    if (touches_top_or_bottom != 0 &&
        background > BEACON_EDGE_LAMP_BACKGROUND_MAX)
    {
        return 0;
    }
    if (touches_left_or_right != 0 &&
        background > BEACON_EDGE_LAMP_BACKGROUND_MAX)
    {
        return 0;
    }
    if (touches_left_or_right != 0 &&
        is_near_vertical_edge(comp->cy, BEACON_SIDE_SUN_LAMP_Y) != 0 &&
        background > BEACON_SIDE_SUN_LAMP_BACKGROUND_MAX)
    {
        return 0;
    }
    if ((comp->min_x <= BEACON_SIDE_SUN_LAMP_MARGIN ||
         comp->max_x >= BEACON_IMAGE_W - 1 - BEACON_SIDE_SUN_LAMP_MARGIN) &&
        is_near_vertical_edge(comp->cy, BEACON_SIDE_SUN_LAMP_Y) != 0 &&
        background > BEACON_SIDE_SUN_LAMP_BACKGROUND_MAX)
    {
        return 0;
    }
    if (g_has_lamp_track == 0 &&
        is_near_vertical_edge(comp->cy, BEACON_TOP_SUN_LAMP_Y) != 0 &&
        background > BEACON_TOP_SUN_LAMP_BACKGROUND_MAX)
    {
        return 0;
    }
    if (comp->area < BEACON_STRONG_LAMP_AREA &&
        (bbox_span < BEACON_SMALL_LAMP_MIN_SPAN ||
         background > BEACON_SMALL_LAMP_BACKGROUND_MAX))
    {
        return 0;
    }
    return 1;
}

static float lamp_score(const component_t *comp)
{
    return (float)comp->area * comp->elongation;
}

static unsigned char split_stacked_lamp_from_beacon(
    const component_t *source,
    component_t *beacon);

static component_t component_from_threshold_region(
    const component_t *source,
    int min_x,
    int max_x,
    int min_y,
    int max_y,
    unsigned char threshold)
{
    int x;
    int y;
    int sum_x = 0;
    int sum_y = 0;
    float sum_xx = 0.0f;
    float sum_yy = 0.0f;
    float sum_xy = 0.0f;
    component_t comp;

    memset(&comp, 0, sizeof(comp));
    comp.min_x = BEACON_IMAGE_W;
    comp.min_y = BEACON_IMAGE_H;
    if (source == 0 || g_current_image == 0)
    {
        return comp;
    }

    if (min_x < source->min_x) min_x = source->min_x;
    if (max_x > source->max_x) max_x = source->max_x;
    if (min_y < source->min_y) min_y = source->min_y;
    if (max_y > source->max_y) max_y = source->max_y;
    if (min_x > max_x || min_y > max_y)
    {
        return comp;
    }

    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
        {
            if (g_current_image[y][x] < threshold)
            {
                continue;
            }
            comp.area++;
            if (g_current_image[y][x] > comp.max_value)
            {
                comp.max_value = g_current_image[y][x];
            }
            sum_x += x;
            sum_y += y;
            sum_xx += (float)x * (float)x;
            sum_yy += (float)y * (float)y;
            sum_xy += (float)x * (float)y;
            if (x < comp.min_x) comp.min_x = x;
            if (x > comp.max_x) comp.max_x = x;
            if (y < comp.min_y) comp.min_y = y;
            if (y > comp.max_y) comp.max_y = y;
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
        comp.radius = sqrtf((float)comp.area / PI_F);
        comp.valid = 1;
    }

    return comp;
}

static void add_lamp_split_beacon(const component_t *comp)
{
    if (comp == 0 || comp->valid == 0 ||
        g_lamp_split_beacon_count >= BEACON_MAX_BEACON_COUNT)
    {
        return;
    }
    g_lamp_split_beacons[g_lamp_split_beacon_count] = *comp;
    g_lamp_split_beacon_count++;
}

static unsigned char component_is_split_beacon_shape(const component_t *comp)
{
    int width;
    int height;

    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    if (comp->area < BEACON_LAMP_SPLIT_MIN_SIDE_AREA)
    {
        return 0;
    }
    if (width > BEACON_LAMP_SPLIT_BEACON_MAX_SPAN ||
        height > BEACON_LAMP_SPLIT_BEACON_MAX_SPAN)
    {
        return 0;
    }
    return (comp->elongation <= BEACON_LAMP_SPLIT_BEACON_MAX_ELONGATION) ? 1 : 0;
}

static unsigned char component_is_split_lamp_shape(const component_t *comp)
{
    int width;

    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    width = comp->max_x - comp->min_x + 1;
    if (comp->area < BEACON_MIN_LAMP_AREA || width < BEACON_MIN_LAMP_LENGTH)
    {
        return 0;
    }
    return (comp->elongation >= BEACON_MIN_LAMP_ELONGATION) ? 1 : 0;
}

static float split_beacon_rank(const component_t *comp)
{
    float width;
    float height;
    float balance;

    if (comp == 0 || comp->valid == 0)
    {
        return 0.0f;
    }
    width = (float)(comp->max_x - comp->min_x + 1);
    height = (float)(comp->max_y - comp->min_y + 1);
    balance = width > height ? height / width : width / height;
    return (float)comp->area * balance / comp->elongation;
}

static void update_best_lamp_split_beacon(
    const component_t *candidate,
    float rank,
    component_t *best_beacon,
    float *best_rank,
    unsigned char *ambiguous)
{
    float dx;

    if (candidate == 0 || candidate->valid == 0 ||
        best_beacon == 0 || best_rank == 0 || ambiguous == 0)
    {
        return;
    }
    if (best_beacon->valid == 0 || rank > *best_rank + BEACON_LAMP_SPLIT_RANK_EPS)
    {
        *best_rank = rank;
        *best_beacon = *candidate;
        *ambiguous = 0U;
        return;
    }
    if (rank + BEACON_LAMP_SPLIT_RANK_EPS < *best_rank)
    {
        return;
    }

    dx = candidate->cx - best_beacon->cx;
    if (dx * dx > BEACON_DUPLICATE_DISTANCE * BEACON_DUPLICATE_DISTANCE)
    {
        *ambiguous = 1U;
    }
}

static unsigned char split_beacon_from_lamp_component(const component_t *source)
{
    int x;
    float best_rank = 0.0f;
    component_t best_beacon;
    unsigned char ambiguous = 0U;
    int source_w;

    memset(&best_beacon, 0, sizeof(best_beacon));
    if (source == 0 ||
        source->valid == 0 ||
        g_current_image == 0 ||
        source->elongation < BEACON_LAMP_SPLIT_MIN_SOURCE_ELONGATION ||
        source->area < BEACON_LAMP_SPLIT_MIN_SOURCE_AREA ||
        vertical_edge_distance(source->cy) < BEACON_LAMP_SPLIT_MIN_SOURCE_Y)
    {
        return 0;
    }

    source_w = source->max_x - source->min_x + 1;
    if (source_w < BEACON_LAMP_SPLIT_MIN_SIDE_WIDTH * 2 + 1)
    {
        return 0;
    }

    for (x = source->min_x + BEACON_LAMP_SPLIT_MIN_SIDE_WIDTH;
         x <= source->max_x - BEACON_LAMP_SPLIT_MIN_SIDE_WIDTH;
         x++)
    {
        component_t left = component_from_threshold_region(
            source,
            source->min_x,
            x,
            source->min_y,
            source->max_y,
            BEACON_CAR_LAMP_THRESHOLD);
        component_t right = component_from_threshold_region(
            source,
            x + 1,
            source->max_x,
            source->min_y,
            source->max_y,
            BEACON_CAR_LAMP_THRESHOLD);
        if (component_is_split_beacon_shape(&left) != 0 &&
            component_is_split_lamp_shape(&right) != 0)
        {
            float rank = split_beacon_rank(&left);
            update_best_lamp_split_beacon(
                &left,
                rank,
                &best_beacon,
                &best_rank,
                &ambiguous);
        }
        if (component_is_split_beacon_shape(&right) != 0 &&
            component_is_split_lamp_shape(&left) != 0)
        {
            float rank = split_beacon_rank(&right);
            update_best_lamp_split_beacon(
                &right,
                rank,
                &best_beacon,
                &best_rank,
                &ambiguous);
        }
    }

    if (best_beacon.valid == 0 || ambiguous != 0U)
    {
        return 0;
    }
    add_lamp_split_beacon(&best_beacon);
    return 1;
}

static unsigned char find_car_lamp(component_t *best_lamp)
{
    unsigned char x;
    unsigned char y;
    float best_score = 0.0f;
    unsigned char found = 0;

    memset(best_lamp, 0, sizeof(*best_lamp));
    memset(g_lamp_mask_components, 0, sizeof(g_lamp_mask_components));
    memset(g_lamp_split_beacons, 0, sizeof(g_lamp_split_beacons));
    g_lamp_mask_component_count = 0;
    g_lamp_split_beacon_count = 0;
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
            if (comp.area >= BEACON_STACKED_SPLIT_MIN_AREA)
            {
                component_t split_beacon;
                if (split_stacked_lamp_from_beacon(&comp, &split_beacon) != 0)
                {
                    add_lamp_split_beacon(&split_beacon);
                }
            }
            (void)split_beacon_from_lamp_component(&comp);
            if (g_lamp_mask_component_count <
                (unsigned char)(BEACON_MAX_CAR_LAMP_COUNT + 3))
            {
                g_lamp_mask_components[g_lamp_mask_component_count] = comp;
                g_lamp_mask_component_count++;
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

static void build_lamp_mask(const component_t *lamp)
{
    int seed_min_x = lamp->min_x - BEACON_LAMP_MASK_PAD;
    int seed_max_x = lamp->max_x + BEACON_LAMP_MASK_PAD;
    int seed_min_y = lamp->min_y - BEACON_LAMP_MASK_PAD;
    int seed_max_y = lamp->max_y + BEACON_LAMP_MASK_PAD;
    int x;
    int y;

    if (lamp->valid == 0)
    {
        return;
    }

    if (seed_min_x < 0) seed_min_x = 0;
    if (seed_min_y < 0) seed_min_y = 0;
    if (seed_max_x >= BEACON_IMAGE_W) seed_max_x = BEACON_IMAGE_W - 1;
    if (seed_max_y >= BEACON_IMAGE_H) seed_max_y = BEACON_IMAGE_H - 1;

    for (y = seed_min_y; y <= seed_max_y; y++)
    {
        for (x = seed_min_x; x <= seed_max_x; x++)
        {
            g_mask[y][x] = 1;
        }
    }
}

static void build_all_lamp_masks(const component_t *lamp)
{
    unsigned char i;

    memset(g_mask, 0, sizeof(g_mask));
    if (lamp->valid == 0)
    {
        return;
    }

    build_lamp_mask(lamp);
    for (i = 0; i < g_lamp_mask_component_count; i++)
    {
        float dx = g_lamp_mask_components[i].cx - lamp->cx;
        float dy = g_lamp_mask_components[i].cy - lamp->cy;

        if (g_lamp_mask_components[i].elongation < BEACON_LAMP_COMPANION_ELONGATION)
        {
            continue;
        }
        if (dx * dx + dy * dy >
            BEACON_LAMP_COMPANION_DISTANCE * BEACON_LAMP_COMPANION_DISTANCE)
        {
            continue;
        }
        build_lamp_mask(&g_lamp_mask_components[i]);
    }
}

static void write_car_lamp(const component_t *lamp, beacon_result_t *result)
{
    mode2_center_image_car_lamp_t *rect;

    if (lamp->valid == 0)
    {
        result->car_lamp_count = 0;
        return;
    }

    rect = &result->car_lamps[0];
    rect->cx = lamp->cx;
    rect->cy = lamp->cy;
    rect->length = lamp->major;
    rect->width = lamp->minor;
    rect->angle = lamp->angle;
    rect->valid = 1;
    result->car_lamp_count = 1;
}

static unsigned char is_far_tiny_component_from_lamp(
    const component_t *comp,
    int background,
    int bbox_span)
{
    float dx;
    float dy;

    if (comp == 0 || g_has_current_lamp == 0)
    {
        return 0;
    }
    if (comp->area > BEACON_LAMP_TINY_FAR_MAX_AREA ||
        bbox_span > BEACON_LAMP_TINY_FAR_MAX_SPAN ||
        is_in_mirrored_lower_edge_band(comp->cy, BEACON_LAMP_TINY_FAR_MAX_Y) != 0 ||
        background > BEACON_LAMP_TINY_BACKGROUND_MAX)
    {
        return 0;
    }

    dx = comp->cx - g_current_lamp.cx;
    dy = comp->cy - g_current_lamp.cy;
    return (dx * dx + dy * dy >=
            BEACON_LAMP_TINY_FAR_DISTANCE * BEACON_LAMP_TINY_FAR_DISTANCE) ? 1 : 0;
}

static unsigned char is_far_tiny_candidate_from_lamp(
    const beacon_candidate_t *candidate,
    int bbox_span)
{
    float dx;
    float dy;

    if (candidate == 0 || g_has_current_lamp == 0)
    {
        return 0;
    }
    if (candidate->area > BEACON_LAMP_TINY_FAR_MAX_AREA ||
        bbox_span > BEACON_LAMP_TINY_FAR_MAX_SPAN ||
        is_in_mirrored_lower_edge_band(candidate->image_y, BEACON_LAMP_TINY_FAR_MAX_Y) != 0)
    {
        return 0;
    }

    dx = candidate->image_x - g_current_lamp.cx;
    dy = candidate->image_y - g_current_lamp.cy;
    return (dx * dx + dy * dy >=
            BEACON_LAMP_TINY_FAR_DISTANCE * BEACON_LAMP_TINY_FAR_DISTANCE) ? 1 : 0;
}

static unsigned char is_bottom_edge_beacon_shape(
    const component_t *comp,
    int background,
    int bbox_span)
{
    if (comp == 0)
    {
        return 0;
    }
    if (comp->cy < BEACON_BOTTOM_EDGE_Y ||
        comp->max_y < BEACON_IMAGE_H - 1 - BEACON_EDGE_BEACON_MARGIN)
    {
        return 0;
    }
    if (comp->area < BEACON_BOTTOM_EDGE_MIN_AREA ||
        comp->area > BEACON_BOTTOM_EDGE_MAX_AREA)
    {
        return 0;
    }
    if (bbox_span > BEACON_BOTTOM_EDGE_MAX_SPAN ||
        comp->elongation > BEACON_BOTTOM_EDGE_MAX_ELONGATION)
    {
        return 0;
    }
    return (background <= BEACON_BOTTOM_EDGE_BACKGROUND_MAX) ? 1 : 0;
}

static unsigned char is_top_edge_beacon_shape(
    const component_t *comp,
    int background,
    int bbox_span)
{
    if (comp == 0)
    {
        return 0;
    }
    if (comp->cy > BEACON_TOP_EDGE_Y ||
        comp->min_y > BEACON_EDGE_BEACON_MARGIN)
    {
        return 0;
    }
    if (comp->area < BEACON_TOP_EDGE_MIN_AREA ||
        comp->area > BEACON_TOP_EDGE_MAX_AREA)
    {
        return 0;
    }
    if (bbox_span > BEACON_TOP_EDGE_MAX_SPAN ||
        comp->elongation > BEACON_TOP_EDGE_MAX_ELONGATION)
    {
        return 0;
    }
    return (background <= BEACON_TOP_EDGE_BACKGROUND_MAX) ? 1 : 0;
}

static unsigned char is_edge_tiny_location(float image_x, float image_y)
{
    unsigned char near_side =
        is_near_horizontal_edge(image_x, BEACON_SIDE_EDGE_TINY_MARGIN);
    unsigned char near_lower_side =
        is_near_horizontal_edge(image_x, BEACON_LOWER_SIDE_TINY_MARGIN);
    unsigned char near_corner_side =
        is_near_horizontal_edge(image_x, BEACON_CORNER_EDGE_TINY_MARGIN);
    float edge_y = vertical_edge_distance(image_y);

    if (near_side != 0 &&
        edge_y >= BEACON_SIDE_EDGE_TINY_MIN_Y &&
        edge_y <= BEACON_SIDE_EDGE_TINY_MAX_Y)
    {
        return 1;
    }
    if (near_corner_side != 0 &&
        is_near_vertical_edge(image_y, BEACON_CORNER_EDGE_TINY_MAX_Y) != 0)
    {
        return 1;
    }
    if (near_lower_side != 0 &&
        is_in_mirrored_lower_edge_band(image_y, BEACON_LOWER_SIDE_TINY_MIN_Y) != 0)
    {
        return 1;
    }
    return 0;
}

static unsigned char is_side_edge_tiny_component(
    const component_t *comp,
    int background)
{
    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    if (is_edge_tiny_location(comp->cx, comp->cy) == 0)
    {
        return 0;
    }
    if (comp->area < BEACON_SIDE_EDGE_TINY_MIN_AREA ||
        comp->area > BEACON_SIDE_EDGE_TINY_MAX_AREA)
    {
        return 0;
    }
    if (comp->elongation > BEACON_SIDE_EDGE_TINY_MAX_ELONGATION ||
        comp->radius < BEACON_SIDE_EDGE_TINY_MIN_RADIUS)
    {
        return 0;
    }
    return (background <= BEACON_SIDE_EDGE_TINY_BACKGROUND_MAX) ? 1 : 0;
}

static unsigned char is_vertical_edge_tiny_component(
    const component_t *comp,
    int background,
    int bbox_span)
{
    unsigned char low_background =
        (background <= BEACON_TOP_TINY_LOW_PEAK_BACKGROUND_MAX) ? 1 : 0;
    unsigned char low_peak =
        (comp != 0 &&
         comp->max_value >= BEACON_TOP_TINY_LOW_PEAK &&
         comp->max_value < BEACON_TOP_TINY_MIN_PEAK) ? 1 : 0;

    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    if (is_near_vertical_edge(comp->cy, BEACON_TOP_TINY_MAX_Y) == 0)
    {
        return 0;
    }
    if (comp->area < BEACON_TOP_TINY_MIN_AREA ||
        comp->area > BEACON_TOP_TINY_MAX_AREA)
    {
        return 0;
    }
    if (low_peak != 0 &&
        low_background != 0 &&
        comp->area <= BEACON_TOP_TINY_LINE_MAX_AREA &&
        bbox_span <= BEACON_TOP_TINY_LINE_MAX_SPAN)
    {
        return (comp->elongation <= BEACON_TOP_TINY_LINE_MAX_ELONGATION) ? 1 : 0;
    }
    if (bbox_span > BEACON_TOP_TINY_MAX_SPAN ||
        comp->elongation > BEACON_TOP_TINY_MAX_ELONGATION)
    {
        return 0;
    }
    if (comp->max_value < BEACON_TOP_TINY_MIN_PEAK &&
        (comp->max_value < BEACON_TOP_TINY_LOW_PEAK ||
         background > BEACON_TOP_TINY_LOW_PEAK_BACKGROUND_MAX))
    {
        return 0;
    }
    return (background <= BEACON_TOP_TINY_BACKGROUND_MAX) ? 1 : 0;
}

static unsigned char is_beacon_candidate(const component_t *comp)
{
    int bbox_w = comp->max_x - comp->min_x + 1;
    int bbox_h = comp->max_y - comp->min_y + 1;
    int bbox_span = bbox_w > bbox_h ? bbox_w : bbox_h;
    int min_area = BEACON_MIN_COMPONENT_AREA;
    int max_area = BEACON_LOW_BEACON_MAX_AREA;
    float max_elongation = BEACON_MAX_BEACON_ELONGATION;
    unsigned char touches_edge =
        (comp->min_x <= BEACON_EDGE_BEACON_MARGIN ||
         comp->max_x >= BEACON_IMAGE_W - 1 - BEACON_EDGE_BEACON_MARGIN ||
         comp->min_y <= BEACON_EDGE_BEACON_MARGIN ||
         comp->max_y >= BEACON_IMAGE_H - 1 - BEACON_EDGE_BEACON_MARGIN) ? 1 : 0;
    int background = local_background_average(comp, BEACON_LOCAL_RING_PAD);
    unsigned char bottom_edge_shape;
    unsigned char top_edge_shape;
    unsigned char side_edge_tiny_shape;
    unsigned char vertical_edge_tiny_shape;
    unsigned char vertical_edge_wide_lamp_shape;
    float edge_distance = vertical_edge_distance(comp->cy);

    if (edge_distance < BEACON_TOP_BEACON_Y)
    {
        min_area = BEACON_TOP_BEACON_MIN_AREA;
        max_area = BEACON_TOP_BEACON_MAX_AREA;
        max_elongation = BEACON_TOP_BEACON_MAX_ELONGATION;
    }
    else if (edge_distance < BEACON_MID_BEACON_Y)
    {
        min_area = BEACON_MID_BEACON_MIN_AREA;
        max_area = BEACON_MID_BEACON_MAX_AREA;
    }
    else
    {
        min_area = BEACON_NORMAL_BEACON_MIN_AREA;
        max_area = BEACON_NORMAL_BEACON_MAX_AREA;
    }
    if (g_beacon_pass_threshold < BEACON_BINARY_THRESHOLD &&
        max_area > BEACON_LOW_BEACON_MAX_AREA)
    {
        max_area = BEACON_LOW_BEACON_MAX_AREA;
    }
    bottom_edge_shape = is_bottom_edge_beacon_shape(comp, background, bbox_span);
    top_edge_shape = is_top_edge_beacon_shape(comp, background, bbox_span);
    side_edge_tiny_shape = is_side_edge_tiny_component(comp, background);
    vertical_edge_tiny_shape =
        is_vertical_edge_tiny_component(comp, background, bbox_span);
    vertical_edge_wide_lamp_shape =
        is_vertical_edge_wide_lamp_shape(comp, background);
    if (vertical_edge_wide_lamp_shape != 0)
    {
        return 0;
    }
    if (g_beacon_pass_threshold < BEACON_BEACON_LOW_THRESHOLD)
    {
        if (g_has_current_lamp != 0 &&
            (vertical_edge_distance(comp->cy) >= BEACON_LAMP_TINY_Y ||
             comp->area > BEACON_LAMP_TINY_MAX_AREA ||
             background > BEACON_LAMP_TINY_BACKGROUND_MAX) &&
            is_far_tiny_component_from_lamp(comp, background, bbox_span) == 0 &&
            bottom_edge_shape == 0 &&
            top_edge_shape == 0 &&
            side_edge_tiny_shape == 0 &&
            vertical_edge_tiny_shape == 0)
        {
            return 0;
        }
        if (g_beacon_pass_threshold < BEACON_BEACON_TINY_THRESHOLD &&
             (comp->area > BEACON_LAMP_MICRO_MAX_AREA ||
             background > BEACON_LAMP_MICRO_BACKGROUND_MAX) &&
            side_edge_tiny_shape == 0 &&
            vertical_edge_tiny_shape == 0)
        {
            return 0;
        }
        min_area = BEACON_TOP_BEACON_MIN_AREA;
    }

    if (comp->area < min_area || comp->area > max_area)
    {
        return 0;
    }
    if (g_beacon_pass_threshold < BEACON_BEACON_LOW_THRESHOLD &&
        local_background_average(comp, BEACON_LOCAL_RING_PAD) >
            BEACON_LOCAL_BACKGROUND_MAX)
    {
        return 0;
    }
    if (touches_edge != 0 &&
        g_beacon_pass_threshold >= BEACON_BEACON_LOW_THRESHOLD &&
        comp->area < BEACON_EDGE_BEACON_MIN_AREA)
    {
        return 0;
    }
    if (touches_edge != 0 &&
        g_has_current_lamp == 0 &&
        background > BEACON_LOCAL_BACKGROUND_MAX)
    {
        return 0;
    }
    if (g_has_current_lamp == 0 &&
        comp->area < BEACON_NO_LAMP_SMALL_AREA &&
        comp->elongation > BEACON_NO_LAMP_SMALL_ELONGATION &&
        side_edge_tiny_shape == 0 &&
        vertical_edge_tiny_shape == 0 &&
        (g_lamp_context_frames > 0 || background > BEACON_DARK_SMALL_BACKGROUND_MAX))
    {
        return 0;
    }
    if (touches_edge != 0 &&
        (comp->area > BEACON_EDGE_BEACON_MAX_AREA ||
         bbox_span > BEACON_EDGE_BEACON_MAX_SPAN))
    {
        return 0;
    }
    if (comp->elongation > max_elongation)
    {
        return 0;
    }
    if (g_has_current_lamp != 0 &&
        comp->elongation > BEACON_LAMP_RESIDUE_ELONGATION)
    {
        float dx = comp->cx - g_current_lamp.cx;
        float dy = comp->cy - g_current_lamp.cy;
        if (dx * dx + dy * dy <=
            BEACON_LAMP_RESIDUE_DISTANCE * BEACON_LAMP_RESIDUE_DISTANCE)
        {
            return 0;
        }
    }
    if ((touches_edge != 0 || comp->area <= BEACON_LOCAL_CONTRAST_AREA) &&
        background > BEACON_LOCAL_BACKGROUND_MAX &&
        bottom_edge_shape == 0 &&
        top_edge_shape == 0 &&
        side_edge_tiny_shape == 0 &&
        vertical_edge_tiny_shape == 0)
    {
        return 0;
    }
    return 1;
}

static void insert_beacon_candidate(const component_t *comp)
{
    int i;
    int slot;
    float comp_x;
    float comp_y;
    int background;

    if (!is_beacon_candidate(comp))
    {
        return;
    }

    background = local_background_average(comp, BEACON_LOCAL_RING_PAD);
    comp_x = comp->cx;
    comp_y = comp->cy;
    for (i = 0; i < g_beacon_candidate_count; i++)
    {
        float dx = g_beacon_candidates[i].circle.x - comp_x;
        float dy = g_beacon_candidates[i].circle.y - comp_y;
        if (dx * dx + dy * dy <= BEACON_DUPLICATE_DISTANCE * BEACON_DUPLICATE_DISTANCE)
        {
            if (duplicate_component_replaces_candidate(
                    comp,
                    background,
                    &g_beacon_candidates[i]) != 0)
            {
                g_beacon_candidates[i].circle.x = comp_x;
                g_beacon_candidates[i].circle.y = comp_y;
                g_beacon_candidates[i].circle.radius = comp->radius;
                g_beacon_candidates[i].circle.area = (float)comp->area;
                g_beacon_candidates[i].circle.valid = 1;
                g_beacon_candidates[i].area = comp->area;
                g_beacon_candidates[i].score = beacon_candidate_score(comp);
                g_beacon_candidates[i].elongation = comp->elongation;
                g_beacon_candidates[i].image_x = comp->cx;
                g_beacon_candidates[i].image_y = comp->cy;
                g_beacon_candidates[i].max_value = comp->max_value;
                g_beacon_candidates[i].background = (unsigned char)background;
                g_beacon_candidates[i].valid = 1;
            }
            return;
        }
    }

    slot = g_beacon_candidate_count;
    if (slot >= MAX_INTERNAL_BEACONS)
    {
        slot = MAX_INTERNAL_BEACONS - 1;
        if (component_has_higher_rank(
                comp,
                background,
                &g_beacon_candidates[slot]) == 0)
        {
            return;
        }
    }
    else
    {
        g_beacon_candidate_count++;
    }

    for (i = slot - 1; i >= 0; i--)
    {
        if (component_has_higher_rank(
                comp,
                background,
                &g_beacon_candidates[i]) == 0)
        {
            break;
        }
        g_beacon_candidates[i + 1] = g_beacon_candidates[i];
    }

    g_beacon_candidates[i + 1].circle.x = comp_x;
    g_beacon_candidates[i + 1].circle.y = comp_y;
    g_beacon_candidates[i + 1].circle.radius = comp->radius;
    g_beacon_candidates[i + 1].circle.area = (float)comp->area;
    g_beacon_candidates[i + 1].circle.valid = 1;
    g_beacon_candidates[i + 1].area = comp->area;
    g_beacon_candidates[i + 1].score = beacon_candidate_score(comp);
    g_beacon_candidates[i + 1].elongation = comp->elongation;
    g_beacon_candidates[i + 1].image_x = comp->cx;
    g_beacon_candidates[i + 1].image_y = comp->cy;
    g_beacon_candidates[i + 1].max_value = comp->max_value;
    g_beacon_candidates[i + 1].background = (unsigned char)background;
    g_beacon_candidates[i + 1].from_lamp_split = 0;
    g_beacon_candidates[i + 1].valid = 1;
    g_beacon_candidates[i + 1].used = 0;
}

static void insert_lamp_split_beacon_candidate(const component_t *comp)
{
    unsigned char old_threshold = g_beacon_pass_threshold;
    unsigned char old_has_lamp = g_has_current_lamp;
    int i;
    float comp_x;
    float comp_y;

    g_beacon_pass_threshold = BEACON_BINARY_THRESHOLD;
    g_has_current_lamp = 0;
    insert_beacon_candidate(comp);
    g_has_current_lamp = old_has_lamp;
    g_beacon_pass_threshold = old_threshold;

    comp_x = comp->cx;
    comp_y = comp->cy;
    for (i = 0; i < g_beacon_candidate_count; i++)
    {
        float dx = g_beacon_candidates[i].circle.x - comp_x;
        float dy = g_beacon_candidates[i].circle.y - comp_y;
        if (dx * dx + dy * dy <=
            BEACON_DUPLICATE_DISTANCE * BEACON_DUPLICATE_DISTANCE)
        {
            g_beacon_candidates[i].from_lamp_split = 1;
            return;
        }
    }
}

static unsigned char component_is_stacked_beacon_shape(const component_t *comp)
{
    int width;
    int height;

    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    if (comp->area < BEACON_STACKED_SPLIT_BEACON_MIN_AREA)
    {
        return 0;
    }
    if (width > BEACON_STACKED_SPLIT_OUTPUT_MAX_SPAN ||
        height > BEACON_STACKED_SPLIT_OUTPUT_MAX_SPAN)
    {
        return 0;
    }
    return (comp->elongation <= BEACON_STACKED_SPLIT_BEACON_MAX_ELONGATION) ? 1 : 0;
}

static unsigned char component_is_stacked_lamp_shape(const component_t *comp)
{
    int width;

    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    width = comp->max_x - comp->min_x + 1;
    if (comp->area < BEACON_STACKED_SPLIT_LAMP_MIN_AREA ||
        width < BEACON_STACKED_SPLIT_LAMP_MIN_WIDTH)
    {
        return 0;
    }
    return (comp->elongation >= BEACON_STACKED_SPLIT_LAMP_MIN_ELONGATION) ? 1 : 0;
}

static unsigned char split_stacked_lamp_from_beacon(
    const component_t *source,
    component_t *beacon)
{
    int y;
    float best_rank = 0.0f;
    unsigned char found = 0U;

    if (source == 0 || beacon == 0 || source->valid == 0)
    {
        return 0;
    }
    memset(beacon, 0, sizeof(*beacon));
    if (source->area < BEACON_STACKED_SPLIT_MIN_AREA ||
        source->area > BEACON_STACKED_SPLIT_MAX_AREA ||
        (source->max_y - source->min_y + 1) < BEACON_STACKED_SPLIT_MIN_HEIGHT)
    {
        return 0;
    }

    for (y = source->min_y + 4; y <= source->max_y - 4; y++)
    {
        component_t top = component_from_threshold_region(
            source,
            source->min_x,
            source->max_x,
            source->min_y,
            y,
            BEACON_CAR_LAMP_THRESHOLD);
        component_t bottom = component_from_threshold_region(
            source,
            source->min_x,
            source->max_x,
            y + 1,
            source->max_y,
            BEACON_CAR_LAMP_THRESHOLD);
        if (component_is_stacked_beacon_shape(&top) != 0 &&
            component_is_stacked_lamp_shape(&bottom) != 0)
        {
            float rank = split_beacon_rank(&top) +
                vertical_edge_distance(top.cy) * 24.0f;
            if (found == 0U || rank > best_rank)
            {
                best_rank = rank;
                *beacon = top;
                found = 1U;
            }
        }
        if (component_is_stacked_lamp_shape(&top) != 0 &&
            component_is_stacked_beacon_shape(&bottom) != 0)
        {
            float rank = split_beacon_rank(&bottom) +
                vertical_edge_distance(bottom.cy) * 24.0f;
            if (found == 0U || rank > best_rank)
            {
                best_rank = rank;
                *beacon = bottom;
                found = 1U;
            }
        }
    }

    return (beacon->valid != 0) ? 1 : 0;
}

static void find_beacon_candidates_pass(void)
{
    unsigned char x;
    unsigned char y;

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
            if (comp.area >= BEACON_STACKED_SPLIT_MIN_AREA)
            {
                component_t split_beacon;
                if (split_stacked_lamp_from_beacon(&comp, &split_beacon) != 0)
                {
                    add_lamp_split_beacon(&split_beacon);
                    continue;
                }
            }
            insert_beacon_candidate(&comp);
        }
    }
}

static void find_beacon_candidates(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    unsigned char split_index;

    memset(g_beacon_candidates, 0, sizeof(g_beacon_candidates));
    g_beacon_candidate_count = 0;

    g_beacon_pass_threshold = BEACON_BINARY_THRESHOLD;
    threshold_image(image, BEACON_BINARY_THRESHOLD, 1);
    find_beacon_candidates_pass();

    g_beacon_pass_threshold = BEACON_BEACON_LOW_THRESHOLD;
    threshold_image(image, BEACON_BEACON_LOW_THRESHOLD, 1);
    find_beacon_candidates_pass();

    g_beacon_pass_threshold = BEACON_BEACON_TINY_THRESHOLD;
    threshold_image(image, BEACON_BEACON_TINY_THRESHOLD, 1);
    find_beacon_candidates_pass();

    g_beacon_pass_threshold = BEACON_BEACON_MICRO_THRESHOLD;
    threshold_image(image, BEACON_BEACON_MICRO_THRESHOLD, 1);
    find_beacon_candidates_pass();

    for (split_index = 0U; split_index < g_lamp_split_beacon_count; split_index++)
    {
        insert_lamp_split_beacon_candidate(&g_lamp_split_beacons[split_index]);
    }
    sort_beacon_candidates();
}

static void copy_beacons_to_legacy(beacon_result_t *result)
{
    int i;

    result->count = result->beacon_count;
    for (i = 0; i < BEACON_MAX_CIRCLE_COUNT; i++)
    {
        if (i < BEACON_MAX_BEACON_COUNT)
        {
            result->circles[i] = result->beacons[i];
        }
        else
        {
            result->circles[i].valid = 0;
        }
    }
}

static unsigned char is_reacquirable_round_beacon(const beacon_candidate_t *candidate)
{
    if (candidate == 0 || candidate->valid == 0)
    {
        return 0;
    }
    if (candidate->area < BEACON_REACQUIRE_MIN_AREA ||
        candidate->area > BEACON_REACQUIRE_MAX_AREA)
    {
        return 0;
    }
    if (candidate->elongation > BEACON_REACQUIRE_MAX_ELONGATION ||
        candidate->circle.radius < BEACON_REACQUIRE_MIN_RADIUS)
    {
        return 0;
    }
    return 1;
}

static unsigned char is_side_edge_tiny_beacon(const beacon_candidate_t *candidate)
{
    if (candidate == 0 || candidate->valid == 0)
    {
        return 0;
    }
    if (is_edge_tiny_location(candidate->image_x, candidate->image_y) == 0)
    {
        return 0;
    }
    if (candidate->area < BEACON_SIDE_EDGE_TINY_MIN_AREA ||
        candidate->area > BEACON_SIDE_EDGE_TINY_MAX_AREA)
    {
        return 0;
    }
    if (candidate->elongation > BEACON_SIDE_EDGE_TINY_MAX_ELONGATION ||
        candidate->circle.radius < BEACON_SIDE_EDGE_TINY_MIN_RADIUS)
    {
        return 0;
    }
    return 1;
}

static unsigned char is_vertical_edge_tiny_beacon(const beacon_candidate_t *candidate)
{
    int bbox_span;
    unsigned char low_background;
    unsigned char low_peak;

    if (candidate == 0 || candidate->valid == 0)
    {
        return 0;
    }
    if (is_near_vertical_edge(candidate->image_y, BEACON_TOP_TINY_MAX_Y) == 0)
    {
        return 0;
    }
    if (candidate->area < BEACON_TOP_TINY_MIN_AREA ||
        candidate->area > BEACON_TOP_TINY_MAX_AREA)
    {
        return 0;
    }
    bbox_span = (int)(candidate->circle.radius * 2.0f + 1.0f);
    low_background =
        (candidate->background <= BEACON_TOP_TINY_LOW_PEAK_BACKGROUND_MAX) ?
            1 : 0;
    low_peak =
        (candidate->max_value >= BEACON_TOP_TINY_LOW_PEAK &&
         candidate->max_value < BEACON_TOP_TINY_MIN_PEAK) ? 1 : 0;
    if (low_peak != 0 &&
        low_background != 0 &&
        candidate->area <= BEACON_TOP_TINY_LINE_MAX_AREA &&
        bbox_span <= BEACON_TOP_TINY_LINE_MAX_SPAN)
    {
        return (candidate->elongation <=
            BEACON_TOP_TINY_LINE_MAX_ELONGATION) ? 1 : 0;
    }
    if (bbox_span > BEACON_TOP_TINY_MAX_SPAN ||
        candidate->elongation > BEACON_TOP_TINY_MAX_ELONGATION)
    {
        return 0;
    }
    if (candidate->max_value < BEACON_TOP_TINY_MIN_PEAK &&
        candidate->max_value < BEACON_TOP_TINY_LOW_PEAK)
    {
        return 0;
    }
    return 1;
}

static unsigned char can_output_beacon_candidate(const beacon_candidate_t *candidate)
{
    if (candidate == 0 || candidate->valid == 0)
    {
        return 0;
    }
    if (candidate->from_lamp_split != 0)
    {
        return 1;
    }
    if (is_side_edge_tiny_beacon(candidate) != 0)
    {
        return 1;
    }
    if (is_vertical_edge_tiny_beacon(candidate) != 0)
    {
        return 1;
    }
    return (candidate->area >= BEACON_OUTPUT_MIN_AREA) ? 1 : 0;
}

static unsigned char can_fill_new_beacon_slot(const beacon_candidate_t *candidate)
{
    float dx;
    float dy;
    int bbox_span;
    unsigned char reacquirable_round;

    if (candidate == 0 || candidate->valid == 0)
    {
        return 0;
    }
    if (candidate->area < BEACON_NEW_SLOT_MIN_AREA ||
        can_output_beacon_candidate(candidate) == 0)
    {
        return 0;
    }
    if (candidate->elongation > BEACON_NEW_SLOT_MAX_ELONGATION)
    {
        return 0;
    }
    if (candidate->image_x <= BEACON_NEW_SLOT_EDGE_MARGIN ||
        candidate->image_x >= (float)(BEACON_IMAGE_W - 1 - BEACON_NEW_SLOT_EDGE_MARGIN) ||
        candidate->image_y <= BEACON_NEW_SLOT_EDGE_MARGIN ||
        candidate->image_y >= (float)(BEACON_IMAGE_H - 1 - BEACON_NEW_SLOT_EDGE_MARGIN))
    {
        return 0;
    }
    if (candidate->circle.radius <= 0.0f)
    {
        return 0;
    }
    reacquirable_round = is_reacquirable_round_beacon(candidate);
    if (g_has_current_lamp == 0 &&
        g_seen_lamp != 0 &&
        g_has_last_beacons == 0 &&
        is_in_mirrored_lower_edge_band(candidate->image_y, BEACON_LAMP_LOST_BEACON_MIN_Y) != 0)
    {
        return 0;
    }
    if (g_has_current_lamp != 0 || g_lamp_context_frames > 0)
    {
        dx = candidate->image_x - g_current_lamp.cx;
        dy = candidate->image_y - g_current_lamp.cy;
        bbox_span = (int)(candidate->circle.radius * 2.0f + 1.0f);
        if (is_side_edge_tiny_beacon(candidate) == 0 &&
            is_near_horizontal_edge(candidate->image_x,
                BEACON_LAMP_NEW_SLOT_EDGE_MARGIN) != 0)
        {
            return 0;
        }
        if (candidate->from_lamp_split != 0)
        {
            return 1;
        }
        if (g_new_slot_frames == 0 &&
            is_near_vertical_edge(candidate->image_y, BEACON_TOP_BEACON_Y) == 0 &&
            is_far_tiny_candidate_from_lamp(candidate, bbox_span) == 0 &&
            reacquirable_round == 0 &&
            is_side_edge_tiny_beacon(candidate) == 0)
        {
            return 0;
        }
        if (g_has_current_lamp != 0 &&
            is_near_vertical_edge(candidate->image_y, BEACON_TOP_BEACON_Y) == 0 &&
            candidate->area > BEACON_LAMP_NEW_SLOT_NEAR_AREA &&
            dx * dx + dy * dy <
                BEACON_LAMP_NEW_SLOT_DISTANCE * BEACON_LAMP_NEW_SLOT_DISTANCE &&
            reacquirable_round == 0)
        {
            return 0;
        }
    }
    return 1;
}

static void write_beacons(beacon_result_t *result)
{
    int i;
    int out_index;
    int highest_slot = 0;

    for (i = 0; i < BEACON_MAX_BEACON_COUNT; i++)
    {
        result->beacons[i].valid = 0;
    }

    if (g_has_last_beacons != 0)
    {
        for (out_index = 0; out_index < BEACON_MAX_BEACON_COUNT; out_index++)
        {
            int best = -1;
            float best_distance = 0.0f;

            if (g_last_beacons[out_index].valid == 0)
            {
                continue;
            }

            for (i = 0; i < g_beacon_candidate_count; i++)
            {
                float dx;
                float dy;
                float dist2;

                if (g_beacon_candidates[i].valid == 0 ||
                    g_beacon_candidates[i].used != 0 ||
                    can_output_beacon_candidate(&g_beacon_candidates[i]) == 0)
                {
                    continue;
                }

                dx = g_beacon_candidates[i].circle.x - g_last_beacons[out_index].x;
                dy = g_beacon_candidates[i].circle.y - g_last_beacons[out_index].y;
                dist2 = dx * dx + dy * dy;
                if (g_has_current_lamp != 0 &&
                    g_beacon_candidates[i].from_lamp_split != 0)
                {
                    dist2 += squaref_local(BEACON_LAMP_SPLIT_TRACK_PENALTY);
                }
                if (dist2 > squaref_local(BEACON_TRACK_MATCH_DISTANCE))
                {
                    continue;
                }
                if (best < 0 || dist2 < best_distance)
                {
                    best = i;
                    best_distance = dist2;
                }
            }

            if (best >= 0)
            {
                result->beacons[out_index] = g_beacon_candidates[best].circle;
                g_beacon_candidates[best].used = 1;
            }
            if (g_last_beacons[out_index].valid != 0 &&
                highest_slot < out_index + 1)
            {
                highest_slot = out_index + 1;
            }
        }
    }

    out_index = (g_has_last_beacons != 0 && g_has_current_lamp != 0) ? 1 : 0;
    for (i = 0; i < g_beacon_candidate_count; i++)
    {
        if (g_beacon_candidates[i].valid == 0 ||
            g_beacon_candidates[i].used != 0 ||
            can_output_beacon_candidate(&g_beacon_candidates[i]) == 0)
        {
            continue;
        }
        if ((g_has_last_beacons != 0 ||
             (g_seen_lamp != 0 && g_has_current_lamp == 0)) &&
            can_fill_new_beacon_slot(&g_beacon_candidates[i]) == 0)
        {
            continue;
        }
        while (out_index < BEACON_MAX_BEACON_COUNT &&
               result->beacons[out_index].valid != 0)
        {
            out_index++;
        }
        if (g_has_last_beacons != 0 &&
            out_index >= BEACON_TRACKED_SLOT_LIMIT)
        {
            break;
        }
        if (out_index >= BEACON_MAX_BEACON_COUNT)
        {
            break;
        }

        result->beacons[out_index] = g_beacon_candidates[i].circle;
        highest_slot = out_index + 1;
        out_index++;
    }

    for (i = 0; i < BEACON_MAX_BEACON_COUNT; i++)
    {
        if (result->beacons[i].valid != 0)
        {
            g_last_beacons[i] = result->beacons[i];
        }
    }
    if (highest_slot > 0 || g_has_last_beacons != 0)
    {
        g_has_last_beacons = 1;
    }

    result->beacon_count = (unsigned char)highest_slot;
    copy_beacons_to_legacy(result);
    if (g_new_slot_frames > 0)
    {
        g_new_slot_frames--;
    }
}

static void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    component_t lamp;
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
    threshold_image(image, BEACON_CAR_LAMP_THRESHOLD, 0);
    has_lamp = find_car_lamp(&lamp);
    g_has_current_lamp = has_lamp;
    if (has_lamp != 0)
    {
        g_seen_lamp = 1;
        g_lamp_context_frames = BEACON_LAMP_CONTEXT_FRAMES;
    }
    else if (g_lamp_context_frames > 0)
    {
        g_lamp_context_frames--;
    }
    if (has_lamp != 0 && g_had_lamp_previous == 0)
    {
        g_has_last_beacons = 0;
        memset(g_last_beacons, 0, sizeof(g_last_beacons));
        g_new_slot_frames = BEACON_NEW_SLOT_FRAMES_AFTER_LAMP;
    }
    g_had_lamp_previous = has_lamp;
    if (has_lamp == 0)
    {
        memset(&lamp, 0, sizeof(lamp));
        g_has_lamp_track = 0;
    }
    else
    {
        g_has_lamp_track = 1;
    }
    g_current_lamp = lamp;

    build_all_lamp_masks(&lamp);
    write_car_lamp(&lamp, result);

    find_beacon_candidates(image);
    write_beacons(result);

    for (i = result->car_lamp_count; i < BEACON_MAX_CAR_LAMP_COUNT; i++)
    {
        result->car_lamps[i].valid = 0;
    }
}

static uint8 s_image_frame[MT9V03X_H][MT9V03X_W];

mode2_center_image_beacon_t g_mode2_center_image_beacons[MODE2_CENTER_IMAGE_MAX_BEACON_COUNT] = {0};
uint8 g_mode2_center_image_beacon_count = 0U;
mode2_center_image_car_lamp_t g_mode2_center_image_car_lamps[MODE2_CENTER_IMAGE_MAX_CAR_LAMP_COUNT] = {0};
uint8 g_mode2_center_image_car_lamp_count = 0U;

static uint8 image_latch_frame(void)
{
    if(0U == mt9v03x_finish_flag)
    {
        return 0U;
    }

    memcpy(s_image_frame[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    mt9v03x_finish_flag = 0U;
    return 1U;
}

static void image_clear_results(void)
{
    memset(g_mode2_center_image_beacons, 0, sizeof(g_mode2_center_image_beacons));
    memset(g_mode2_center_image_car_lamps, 0, sizeof(g_mode2_center_image_car_lamps));
    g_mode2_center_image_beacon_count = 0U;
    g_mode2_center_image_car_lamp_count = 0U;
}
static void image_store_result(const beacon_result_t *result)
{
    uint8 i;
    uint8 beacon_count = result->beacon_count;
    uint8 car_lamp_count = result->car_lamp_count;

    image_clear_results();

    if(beacon_count > MODE2_CENTER_IMAGE_MAX_BEACON_COUNT)
    {
        beacon_count = MODE2_CENTER_IMAGE_MAX_BEACON_COUNT;
    }
    for(i = 0U; i < beacon_count; i++)
    {
        g_mode2_center_image_beacons[i] = result->beacons[i];
    }
    g_mode2_center_image_beacon_count = beacon_count;

    if(car_lamp_count > MODE2_CENTER_IMAGE_MAX_CAR_LAMP_COUNT)
    {
        car_lamp_count = MODE2_CENTER_IMAGE_MAX_CAR_LAMP_COUNT;
    }
    for(i = 0U; i < car_lamp_count; i++)
    {
        g_mode2_center_image_car_lamps[i] = result->car_lamps[i];
    }
    g_mode2_center_image_car_lamp_count = car_lamp_count;
}

void mode2_center_image_init(void)
{
    memset(s_image_frame, 0, sizeof(s_image_frame));
    image_clear_results();
    beacon_image_init();

    mt9v03x_finish_flag = 0U;
    (void)mt9v03x_init();
}

void mode2_center_image_update(void)
{
    beacon_result_t result;

    if(0U == image_latch_frame())
    {
        return;
    }

    beacon_image_process(s_image_frame, &result);
    image_store_result(&result);
}

uint8 *mode2_center_image_get_frame_buffer(void)
{
    return s_image_frame[0];
}
