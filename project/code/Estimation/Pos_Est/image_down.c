#include "image_down.h"

#include <math.h>
#include <string.h>

#include "zf_device_mt9v03x.h"
#include "Display/image_debug_screen.h"
#include "Image/image_down_horizon.h"

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
    unsigned char car_lamp_measured_mask;
    unsigned char car_lamp_classification;
} beacon_result_t;

#if (MT9V03X_W != BEACON_IMAGE_W) || (MT9V03X_H != BEACON_IMAGE_H)
#error "Beacon image algorithm is tuned for MT9V03X 188x120 frames."
#endif

#define LAMP_MASK_PAD                   7

/* Core1图像算法运行时参数。 */
int32 g_image_down_beacon_binary_threshold = 120;
int32 g_image_down_beacon_min_area = 10;
int32 g_image_down_side_edge_min_area = 10;
int32 g_image_down_side_edge_threshold = 100;
int32 g_image_down_car_lamp_binary_threshold = 200;
int32 g_image_down_car_lamp_min_area = 24;
int32 g_image_down_car_lamp_max_area = 1200;
float g_image_down_car_lamp_min_elongation = 1.6f;
float g_image_down_car_lamp_min_length = 8.0f;
int32 g_image_down_near_lamp_pad = 8;
int32 g_image_down_near_lamp_min_area = 45;
int32 g_image_down_near_lamp_isolated_min_area = 18;
int32 g_image_down_near_lamp_background_max = 40;
float g_image_down_match_distance = 18.0f;
float g_image_down_gate_distance = 24.0f;
float g_image_down_new_target_distance = 36.0f;
int32 g_image_down_confirm_frames = 2;
int32 g_image_down_max_misses = 3;
float g_image_down_filter_pos_alpha = 0.65f;
float g_image_down_filter_vel_alpha = 0.30f;
int32 g_image_down_car_lamp_edge_max_misses = 3;              /* 边缘车灯最大丢失帧数。 */
int32 g_image_down_car_lamp_center_max_misses = 24;           /* 中部车灯最大丢失帧数。 */
int32 g_image_down_car_lamp_temporal_core_pad = 2;            /* 时序核心外扩距离，单位像素。 */
int32 g_image_down_car_lamp_temporal_takeover_pad = 10;       /* 时序接管外扩距离，单位像素。 */
int32 g_image_down_car_lamp_temporal_min_bright_area = 3;     /* 时序车灯最小亮区面积。 */
int32 g_image_down_car_output_envelope_pad = 10;              /* 车灯输出包络外扩距离，单位像素。 */
float g_image_down_car_group_angle_max = 25.0f;               /* 车灯分组最大角度差，单位度。 */
float g_image_down_car_group_minor_pad = 2.0f;                /* 车灯分组短轴外扩距离，单位像素。 */
float g_image_down_car_group_major_gap = 4.0f;                /* 车灯分组长轴最大间隙，单位像素。 */
int32 g_image_down_car_recover_min_core_area = 3;             /* 车灯恢复核心最小面积。 */
int32 g_image_down_car_recover_max_core_area = 18;            /* 车灯恢复核心最大面积。 */
int32 g_image_down_car_recover_connect_threshold = 150;      /* 车灯连接恢复灰度阈值。 */
int32 g_image_down_car_recover_track_threshold = 120;        /* 车灯跟踪恢复灰度阈值。 */
int32 g_image_down_car_recover_bridge_threshold = 80;        /* 车灯桥接恢复灰度阈值。 */
float g_image_down_car_recover_support_pad = 3.0f;            /* 车灯恢复支持区外扩距离，单位像素。 */
int32 g_image_down_car_support_pad = 1;                       /* 车灯支持区外扩距离，单位像素。 */
int32 g_image_down_gray_support_threshold = 120;             /* 车灯支持区灰度阈值。 */
int32 g_image_down_car_compact_edge_margin = 2;              /* 紧凑车灯边缘余量，单位像素。 */
float g_image_down_car_compact_edge_max_major = 10.0f;        /* 边缘紧凑车灯最大长轴，单位像素。 */
int32 g_image_down_beacon_coast_frames = 4;                  /* 信标无测量惯性跟踪帧数。 */
int32 g_image_down_beacon_corner_margin = 12;                /* 信标角落判定边距，单位像素。 */
int32 g_image_down_near_car_beacon_confirm = 3;              /* 近车灯信标确认帧数。 */
int32 g_image_down_near_car_beacon_max_misses = 2;           /* 近车灯信标最大丢失帧数。 */
int32 g_image_down_edge_beacon_confirm = 2;                  /* 边缘信标确认帧数。 */
int32 g_image_down_edge_beacon_recover_radius = 6;           /* 边缘信标恢复搜索半径，单位像素。 */
int32 g_image_down_edge_beacon_recover_peak = 170;           /* 边缘信标恢复最小峰值灰度。 */

static uint8 s_mt9v03x_initialized;
static uint32 s_image_down_latched_frame_sequence;                 /* 最近成功锁存并处理的摄像头来源帧号。 */

#define CAR_LAMP_EDGE_MAX_MISSES        g_image_down_car_lamp_edge_max_misses
#define CAR_LAMP_CENTER_MAX_MISSES      g_image_down_car_lamp_center_max_misses
#define CAR_LAMP_TEMPORAL_EDGE_MARGIN   8
#define CAR_LAMP_TEMPORAL_MASK_PAD      5
#define CAR_LAMP_TEMPORAL_CORE_PAD      g_image_down_car_lamp_temporal_core_pad
#define CAR_LAMP_TEMPORAL_TAKEOVER_PAD  g_image_down_car_lamp_temporal_takeover_pad
#define CAR_LAMP_TEMPORAL_MIN_BRIGHT_AREA g_image_down_car_lamp_temporal_min_bright_area

#define IMAGE_QUEUE_SIZE                (BEACON_IMAGE_W * BEACON_IMAGE_H)
#define PI_F                            3.1415926f
#define DOWN_BEACON_COAST_FRAMES        g_image_down_beacon_coast_frames
#define DOWN_GRAY_MAX_PEAKS             20U
#define DOWN_GRAY_MAX_CANDIDATES        12U
#define DOWN_OBJECT_BOUNDARY_MARGIN     3.0f
#define DOWN_BEACON_BOUNDARY_CLEARANCE  9.0f
#define DOWN_GRAY_DEDUP_DISTANCE_SQ     25.0f
#define DOWN_GRAY_PATCH_RADIUS          6
#define DOWN_GRAY_BACKGROUND_INNER_SQ  49
#define DOWN_GRAY_BACKGROUND_OUTER_SQ  100
#define DOWN_BEACON_LEFT_EDGE_MARGIN    24
#define DOWN_BEACON_RIGHT_EDGE_MARGIN   16
#define DOWN_BEACON_CORNER_MARGIN       g_image_down_beacon_corner_margin
#define DOWN_GRAY_EDGE_MIN_PEAK         200U
#define DOWN_GRAY_EDGE_MAX_OCCUPANCY    0.25f
#define DOWN_EDGE_SUPPORT_RADIUS        12
#define DOWN_EDGE_SUPPORT_MAX_AREA      48
#define DOWN_LOCAL_SHAPE_SIZE           (DOWN_EDGE_SUPPORT_RADIUS * 2 + 1)
#define DOWN_LOCAL_SHAPE_CAPACITY       (DOWN_LOCAL_SHAPE_SIZE * DOWN_LOCAL_SHAPE_SIZE)
#define DOWN_CAR_MAX_MASKS              12U
#define DOWN_CAR_MAX_COMPONENTS         32U
#define DOWN_CAR_ENVELOPE_MIN_PAD       4
#define DOWN_CAR_ENVELOPE_PAD_SCALE     0.75f
#define DOWN_CAR_OUTPUT_ENVELOPE_PAD    g_image_down_car_output_envelope_pad
#define DOWN_CAR_GROUP_MAX_CORES        3U
#define DOWN_CAR_GROUP_ANGLE_MAX        g_image_down_car_group_angle_max
#define DOWN_CAR_GROUP_MINOR_PAD        g_image_down_car_group_minor_pad
#define DOWN_CAR_GROUP_MAJOR_GAP        g_image_down_car_group_major_gap
#define DOWN_CAR_RECOVER_MIN_CORE_AREA  g_image_down_car_recover_min_core_area
#define DOWN_CAR_RECOVER_MAX_CORE_AREA  g_image_down_car_recover_max_core_area
#define DOWN_CAR_RECOVER_CONNECT_THRESHOLD g_image_down_car_recover_connect_threshold
#define DOWN_CAR_RECOVER_TRACK_THRESHOLD   g_image_down_car_recover_track_threshold
#define DOWN_CAR_RECOVER_BRIDGE_THRESHOLD  g_image_down_car_recover_bridge_threshold
#define DOWN_CAR_RECOVER_SUPPORT_PAD        g_image_down_car_recover_support_pad
#define DOWN_CAR_SUPPORT_PAD            g_image_down_car_support_pad
#define DOWN_CAR_SUPPORT_MASK_SIZE      ((IMAGE_QUEUE_SIZE + 7U) / 8U)
#define DOWN_NEAR_CAR_BEACON_TRACKS     2U
#define DOWN_NEAR_CAR_BEACON_CONFIRM    g_image_down_near_car_beacon_confirm
#define DOWN_NEAR_CAR_BEACON_MAX_MISSES g_image_down_near_car_beacon_max_misses
#define DOWN_EDGE_BEACON_TRACKS         2U
#define DOWN_EDGE_BEACON_CONFIRM        g_image_down_edge_beacon_confirm
#define DOWN_EDGE_BEACON_RECOVER_RADIUS g_image_down_edge_beacon_recover_radius
#define DOWN_EDGE_BEACON_RECOVER_PEAK   g_image_down_edge_beacon_recover_peak
#define DOWN_CAR_STRONG_SCORE           0.53f
#define DOWN_CAR_WEAK_SCORE             0.50f
#define DOWN_CAR_TRACK_SCORE            0.47f
#define DOWN_CAR_TRACK_SCORE_MARGIN     0.02f
#define DOWN_CAR_SCORE_ELONGATION_BASE  1.35f
#define DOWN_CAR_SCORE_ELONGATION_RANGE 2.00f
#define DOWN_CAR_SCORE_VARIANCE_RANGE   700.0f
#define DOWN_CAR_SCORE_MEAN_BASE        205.0f
#define DOWN_CAR_SCORE_MEAN_RANGE       35.0f
#define DOWN_CAR_SCORE_CONTRAST_BASE    18.0f
#define DOWN_CAR_SCORE_CONTRAST_RANGE   80.0f
#define DOWN_CAR_SCORE_AREA_BASE        10.0f
#define DOWN_CAR_SCORE_AREA_RANGE       70.0f
#define DOWN_CAR_COMPACT_EDGE_MARGIN     g_image_down_car_compact_edge_margin
#define DOWN_CAR_COMPACT_EDGE_MAX_MAJOR g_image_down_car_compact_edge_max_major
#define DOWN_GRAY_SUPPORT_THRESHOLD     g_image_down_gray_support_threshold

#define DOWN_CAR_CLASS_NONE             0U
#define DOWN_CAR_CLASS_TRACK            1U
#define DOWN_CAR_CLASS_WEAK             2U
#define DOWN_CAR_CLASS_STRONG           3U
#define DOWN_CAR_CLASS_CENTER_RECOVERED 4U

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
    float orientation_numerator;
    float orientation_denominator;
    unsigned long gray_sum;
    unsigned long gray_sum_sq;
    unsigned char peak;
    unsigned char seed_x;
    unsigned char seed_y;
    unsigned char envelope_threshold;
    unsigned char angle_valid;
    unsigned char valid;
} component_t;

typedef struct
{
    float cx;
    float cy;
    float cos_angle;
    float sin_angle;
    float half_length;
    float half_width;
    int min_x;
    int min_y;
    int max_x;
    int max_y;
    unsigned char valid;
} down_lamp_geometry_t;

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

typedef struct
{
    float x;
    float y;
    unsigned char active;
    unsigned char confirmed;
    unsigned char hits;
    unsigned char misses;
} down_near_car_beacon_track_t;

typedef struct
{
    float x;
    float y;
    float vx;
    float vy;
    float area;
    unsigned char active;
    unsigned char confirmed;
    unsigned char hits;
    unsigned char misses;
} down_edge_beacon_track_t;

typedef struct
{
    signed short response;
    unsigned char x;
    unsigned char y;
} down_gray_peak_t;

typedef struct
{
    float x;
    float y;
    float area;
    float score;
    float classification_x;
    float classification_y;
    unsigned char seed_x;
    unsigned char seed_y;
} down_gray_candidate_t;

typedef struct
{
    float background;
    float inner_contrast;
    float radial_drop;
    float concentration;
    float elongation;
    float offset;
    float outer_occupancy;
    float centroid_x;
    float centroid_y;
    unsigned char peak;
    unsigned char half_area;
} down_gray_features_t;

typedef struct
{
    int area;
    float major;
    float minor;
    float elongation;
    unsigned char touches_artificial_boundary;
    unsigned char valid;
} down_local_shape_t;

typedef struct
{
    float mean;
    float variance;
    float contrast;
    float score;
    component_t envelope;
    unsigned char peak;
    unsigned char classification;
} down_car_features_t;

typedef struct
{
    component_t component;
    component_t core;
    float score;
    unsigned char classification;
    unsigned char output_eligible;
} down_car_candidate_t;

uint8 g_image_frame[MT9V03X_H][MT9V03X_W];

static unsigned char g_binary[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_beacon_binary_snapshot[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_car_lamp_binary_snapshot[BEACON_IMAGE_H][BEACON_IMAGE_W];
/* 最终车灯的实际支持像素及一像素保护带，每像素占一位。 */
static unsigned char g_car_support_mask[DOWN_CAR_SUPPORT_MASK_SIZE];
static unsigned char g_car_support_valid;
static component_t g_car_output_override;
static unsigned char g_car_output_override_valid;
static unsigned char g_visit_stamp[IMAGE_QUEUE_SIZE];
static unsigned char g_current_stamp = 0;
/* 连通域队列：低8位为X，高8位为Y。 */
static unsigned short g_queue[IMAGE_QUEUE_SIZE];
static const unsigned char (*g_current_image)[BEACON_IMAGE_W] = 0;
static temporal_track_t g_b0_track;
static temporal_track_t g_car_track;
static down_near_car_beacon_track_t
    g_near_car_beacon_tracks[DOWN_NEAR_CAR_BEACON_TRACKS];
static down_edge_beacon_track_t
    g_edge_beacon_tracks[DOWN_EDGE_BEACON_TRACKS];
static float g_current_beacon_classification_x[BEACON_MAX_BEACON_COUNT];
static float g_current_beacon_classification_y[BEACON_MAX_BEACON_COUNT];
static unsigned char
    g_current_beacon_classification_valid[BEACON_MAX_BEACON_COUNT];
static component_t g_weak_car_pending;
static unsigned char g_weak_car_pending_hits;
static unsigned char g_weak_car_pending_misses;
/* Box3列方向滑动和，前后预留镜像边界槽。 */
static unsigned short g_gray_box3_storage[BEACON_IMAGE_W + 3];
/* Box9列方向滑动和，前后预留镜像边界槽。 */
static unsigned short g_gray_box9_storage[BEACON_IMAGE_W + 8];
static signed short g_gray_response_rows[3][BEACON_IMAGE_W];

static void beacon_image_reset_temporal(void);
static void begin_visit_pass(void);
static unsigned char down_gray_point_in_current_lamps(int x, int y);
static unsigned char down_local_shape_measure(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_x,
    int center_y,
    int radius,
    int threshold,
    int maximum_area,
    unsigned char reject_artificial_boundary,
    down_local_shape_t *shape);
/* 候选分类完成后，仅在局部恢复阈值120的真实信标区域。 */
static unsigned char down_gray_refine_candidate_region(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    down_gray_candidate_t *candidate)
{
    unsigned short *head = g_queue;
    unsigned short *tail = g_queue;
    unsigned char stamp;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int seed_x;
    int seed_y;
    int sum_x = 0;
    int sum_y = 0;
    int area = 0;

    if(image == 0 || candidate == 0)
    {
        return 0U;
    }
    seed_x = (int)candidate->seed_x;
    seed_y = (int)candidate->seed_y;
    if(seed_x < 0 || seed_x >= BEACON_IMAGE_W ||
       seed_y < 0 || seed_y >= BEACON_IMAGE_H ||
       image[seed_y][seed_x] < DOWN_GRAY_SUPPORT_THRESHOLD ||
       down_gray_point_in_current_lamps(seed_x, seed_y) != 0U)
    {
        return 0U;
    }
    min_x = seed_x - DOWN_EDGE_SUPPORT_RADIUS;
    max_x = seed_x + DOWN_EDGE_SUPPORT_RADIUS;
    min_y = seed_y - DOWN_EDGE_SUPPORT_RADIUS;
    max_y = seed_y + DOWN_EDGE_SUPPORT_RADIUS;
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    begin_visit_pass();
    stamp = g_current_stamp;
    g_visit_stamp[seed_y * BEACON_IMAGE_W + seed_x] = stamp;
    *tail++ = (unsigned short)(seed_x | (seed_y << 8));
    while(head < tail)
    {
        unsigned short coordinate = *head++;
        unsigned char x = (unsigned char)(coordinate & 0xFFU);
        unsigned char y = (unsigned char)(coordinate >> 8);
        unsigned short pixel_index =
            (unsigned short)((unsigned short)y * BEACON_IMAGE_W + x);

        sum_x += (int)x;
        sum_y += (int)y;
        area++;
#define ENQUEUE_REFINE_IF(condition, neighbor_index, neighbor_coordinate) \
        do { \
            if ((condition) && \
                g_visit_stamp[(neighbor_index)] != stamp && \
                 down_gray_point_in_current_lamps( \
                     (unsigned char)((neighbor_coordinate) & 0xFFU), \
                     (unsigned char)((neighbor_coordinate) >> 8)) == 0U && \
                 image[(unsigned char)((neighbor_coordinate) >> 8)] \
                     [(unsigned char)((neighbor_coordinate) & 0xFFU)] >= \
                    DOWN_GRAY_SUPPORT_THRESHOLD) \
            { \
                g_visit_stamp[(neighbor_index)] = stamp; \
                *tail++ = (neighbor_coordinate); \
            } \
        } while (0)
        ENQUEUE_REFINE_IF((int)x < max_x,
            pixel_index + 1U, (unsigned short)(coordinate + 1U));
        ENQUEUE_REFINE_IF((int)x > min_x,
            pixel_index - 1U, (unsigned short)(coordinate - 1U));
        ENQUEUE_REFINE_IF((int)y < max_y,
            pixel_index + BEACON_IMAGE_W,
            (unsigned short)(coordinate + 0x0100U));
        ENQUEUE_REFINE_IF((int)y > min_y,
            pixel_index - BEACON_IMAGE_W,
            (unsigned short)(coordinate - 0x0100U));
        ENQUEUE_REFINE_IF((int)x < max_x && (int)y < max_y,
            pixel_index + BEACON_IMAGE_W + 1U,
            (unsigned short)(coordinate + 0x0101U));
        ENQUEUE_REFINE_IF((int)x < max_x && (int)y > min_y,
            pixel_index - BEACON_IMAGE_W + 1U,
            (unsigned short)(coordinate - 0x00FFU));
        ENQUEUE_REFINE_IF((int)x > min_x && (int)y < max_y,
            pixel_index + BEACON_IMAGE_W - 1U,
            (unsigned short)(coordinate + 0x00FFU));
        ENQUEUE_REFINE_IF((int)x > min_x && (int)y > min_y,
            pixel_index - BEACON_IMAGE_W - 1U,
            (unsigned short)(coordinate - 0x0101U));
#undef ENQUEUE_REFINE_IF
    }
    if(area <= 0)
    {
        return 0U;
    }
    candidate->x = (float)sum_x / (float)area;
    candidate->y = (float)sum_y / (float)area;
    candidate->area = (float)area;
    return 1U;
}

static unsigned char down_gray_edge_support_valid(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_x,
    int center_y,
    float background,
    unsigned char peak,
    int left_margin,
    int right_margin);
static unsigned char down_gray_side_edge_beacon_like(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *component);
static void down_gray_lamp_geometry_init(
    const component_t *lamp,
    down_lamp_geometry_t *geometry);
static unsigned char down_gray_point_in_lamp_geometry(
    int x,
    int y,
    const down_lamp_geometry_t *geometry);
static void component_resolve_angle(component_t *comp);
static void clear_car_result(beacon_result_t *result);

static void beacon_image_init(void)
{
    memset(g_binary, 0, sizeof(g_binary));
    memset(g_beacon_binary_snapshot, 0, sizeof(g_beacon_binary_snapshot));
    memset(g_car_lamp_binary_snapshot, 0, sizeof(g_car_lamp_binary_snapshot));
    memset(g_car_support_mask, 0, sizeof(g_car_support_mask));
    g_car_support_valid = 0U;
    memset(&g_car_output_override, 0, sizeof(g_car_output_override));
    g_car_output_override_valid = 0U;
    memset(g_visit_stamp, 0, sizeof(g_visit_stamp));
    memset(&g_weak_car_pending, 0, sizeof(g_weak_car_pending));
    g_current_image = 0;
    g_weak_car_pending_hits = 0U;
    g_weak_car_pending_misses = 0U;
    g_current_stamp = 0;
    beacon_image_reset_temporal();
}

static void beacon_image_reset_temporal(void)
{
    memset(&g_b0_track, 0, sizeof(g_b0_track));
    memset(&g_car_track, 0, sizeof(g_car_track));
    memset(g_near_car_beacon_tracks, 0,
           sizeof(g_near_car_beacon_tracks));
    memset(g_edge_beacon_tracks, 0,
           sizeof(g_edge_beacon_tracks));
    memset(&g_weak_car_pending, 0, sizeof(g_weak_car_pending));
    g_weak_car_pending_hits = 0U;
    g_weak_car_pending_misses = 0U;
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
    return (g_visit_stamp[(unsigned int)y * BEACON_IMAGE_W + x] ==
            g_current_stamp) ? 1 : 0;
}

static unsigned char car_support_get(unsigned int pixel_index)
{
    return (g_car_support_mask[pixel_index >> 3U] &
            (unsigned char)(1U << (pixel_index & 7U))) != 0U ? 1U : 0U;
}

static void car_support_set(unsigned int pixel_index, unsigned char value)
{
    unsigned char bit = (unsigned char)(1U << (pixel_index & 7U));

    if(value != 0U)
    {
        g_car_support_mask[pixel_index >> 3U] |= bit;
    }
    else
    {
        g_car_support_mask[pixel_index >> 3U] &= (unsigned char)~bit;
    }
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static float threshold_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char threshold)
{
    const unsigned char *src = &image[0][0];
    unsigned char *dst = &g_binary[0][0];
    unsigned long scene_sum = 0U;
    unsigned int scene_count = 0U;
    int x;
    int y;

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        const unsigned char *src_row = src + y * BEACON_IMAGE_W;
        unsigned char *dst_row = dst + y * BEACON_IMAGE_W;

#if defined(__ICCARM__)
        const uint32 threshold_word = (uint32)threshold * 0x01010101UL;
        for (x = 0; x < BEACON_IMAGE_W; x += 4)
        {
            uint32 gray_word = __UNALIGNED_UINT32_READ(&src_row[x]);
            uint32 binary_word;

            (void)__USUB8(gray_word, threshold_word);
            binary_word = __SEL(0xFFFFFFFFUL, 0U);
            __UNALIGNED_UINT32_WRITE(&dst_row[x], binary_word);
            if ((y & 3) == 0)
            {
                scene_sum += gray_word & 0xFFU;
                scene_count++;
            }
        }
#else
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            dst_row[x] = (src_row[x] >= threshold) ? 255U : 0U;
            if (((y & 3) == 0) && ((x & 3) == 0))
            {
                scene_sum += src_row[x];
                scene_count++;
            }
        }
#endif
    }
    return (scene_count > 0U) ?
               (float)scene_sum / (float)scene_count : 0.0f;
}

static unsigned short *grow_component_enqueue(
    const unsigned char *binary,
    unsigned char *visit,
    unsigned int index,
    unsigned short coordinate,
    unsigned char stamp,
    unsigned short *tail,
    unsigned short *end)
{
    if (tail >= end || binary[index] == 0U || visit[index] == stamp)
    {
        return tail;
    }
    visit[index] = stamp;
    *tail++ = coordinate;
    return tail;
}

static void finish_component(
    component_t *comp,
    int sum_x,
    int sum_y,
    unsigned short coordinate_count);

#if defined(__ICCARM__)
#pragma inline=never
#endif
static component_t grow_component(unsigned char start_x, unsigned char start_y)
{
    const unsigned char *binary = &g_binary[0][0];
    unsigned char *visit = g_visit_stamp;
    unsigned short *head = g_queue;
    unsigned short *tail = g_queue;
    unsigned short *end = g_queue + IMAGE_QUEUE_SIZE;
    unsigned char stamp = g_current_stamp;
    int sum_x = 0;
    int sum_y = 0;
    component_t comp;

    memset(&comp, 0, sizeof(comp));
    comp.min_x = start_x;
    comp.max_x = start_x;
    comp.min_y = start_y;
    comp.max_y = start_y;
    comp.seed_x = start_x;
    comp.seed_y = start_y;

    *tail++ = (unsigned short)start_x | ((unsigned short)start_y << 8U);
    visit[(unsigned int)start_y * BEACON_IMAGE_W + start_x] = stamp;

    while (head < tail)
    {
        unsigned short coordinate = *head++;
        unsigned char x = (unsigned char)coordinate;
        unsigned char y = (unsigned char)(coordinate >> 8U);
        unsigned int pixel_index = (unsigned int)y * BEACON_IMAGE_W + x;
        unsigned int gray = (g_current_image != 0) ? g_current_image[y][x] : 0U;

        comp.area++;
        comp.gray_sum += gray;
        comp.gray_sum_sq += gray * gray;
        if (gray > comp.peak)
        {
            comp.peak = (unsigned char)gray;
        }
        sum_x += x;
        sum_y += y;

        if ((int)x < comp.min_x) comp.min_x = x;
        if ((int)x > comp.max_x) comp.max_x = x;
        if ((int)y < comp.min_y) comp.min_y = y;
        if ((int)y > comp.max_y) comp.max_y = y;

        if (x > 0U && x < BEACON_IMAGE_W - 1 &&
            y > 0U && y < BEACON_IMAGE_H - 1)
        {
            tail = grow_component_enqueue(
                binary, visit, pixel_index + 1U,
                (unsigned short)(coordinate + 1U), stamp, tail, end);
            tail = grow_component_enqueue(
                binary, visit, pixel_index - 1U,
                (unsigned short)(coordinate - 1U), stamp, tail, end);
            tail = grow_component_enqueue(
                binary, visit, pixel_index + BEACON_IMAGE_W,
                (unsigned short)(coordinate + 0x0100U), stamp, tail, end);
            tail = grow_component_enqueue(
                binary, visit, pixel_index - BEACON_IMAGE_W,
                (unsigned short)(coordinate - 0x0100U), stamp, tail, end);
            tail = grow_component_enqueue(
                binary, visit, pixel_index + BEACON_IMAGE_W + 1U,
                (unsigned short)(coordinate + 0x0101U), stamp, tail, end);
            tail = grow_component_enqueue(
                binary, visit, pixel_index - BEACON_IMAGE_W + 1U,
                (unsigned short)(coordinate - 0x00FFU), stamp, tail, end);
            tail = grow_component_enqueue(
                binary, visit, pixel_index + BEACON_IMAGE_W - 1U,
                (unsigned short)(coordinate + 0x00FFU), stamp, tail, end);
            tail = grow_component_enqueue(
                binary, visit, pixel_index - BEACON_IMAGE_W - 1U,
                (unsigned short)(coordinate - 0x0101U), stamp, tail, end);
        }
        else
        {
#define ENQUEUE_COMPONENT_IF(valid_, index_, coordinate_) \
            do { if (valid_) { tail = grow_component_enqueue( \
                binary, visit, (index_), (coordinate_), stamp, tail, end); } } while (0)
            ENQUEUE_COMPONENT_IF(x + 1U < BEACON_IMAGE_W,
                pixel_index + 1U, (unsigned short)(coordinate + 1U));
            ENQUEUE_COMPONENT_IF(x > 0U,
                pixel_index - 1U, (unsigned short)(coordinate - 1U));
            ENQUEUE_COMPONENT_IF(y + 1U < BEACON_IMAGE_H,
                pixel_index + BEACON_IMAGE_W,
                (unsigned short)(coordinate + 0x0100U));
            ENQUEUE_COMPONENT_IF(y > 0U,
                pixel_index - BEACON_IMAGE_W,
                (unsigned short)(coordinate - 0x0100U));
            ENQUEUE_COMPONENT_IF(x + 1U < BEACON_IMAGE_W && y + 1U < BEACON_IMAGE_H,
                pixel_index + BEACON_IMAGE_W + 1U,
                (unsigned short)(coordinate + 0x0101U));
            ENQUEUE_COMPONENT_IF(x + 1U < BEACON_IMAGE_W && y > 0U,
                pixel_index - BEACON_IMAGE_W + 1U,
                (unsigned short)(coordinate - 0x00FFU));
            ENQUEUE_COMPONENT_IF(x > 0U && y + 1U < BEACON_IMAGE_H,
                pixel_index + BEACON_IMAGE_W - 1U,
                (unsigned short)(coordinate + 0x00FFU));
            ENQUEUE_COMPONENT_IF(x > 0U && y > 0U,
                pixel_index - BEACON_IMAGE_W - 1U,
                (unsigned short)(coordinate - 0x0101U));
#undef ENQUEUE_COMPONENT_IF
        }
    }

    finish_component(
        &comp, sum_x, sum_y, (unsigned short)(tail - g_queue));
    return comp;
}

static float car_score_term(float value, float base, float range)
{
    value = (value - base) / range;
    if (value < 0.0f)
    {
        return 0.0f;
    }
    if (value > 1.0f)
    {
        return 1.0f;
    }
    return value;
}

static int car_envelope_pad(const component_t *comp)
{
    int pad = (int)(comp->minor * DOWN_CAR_ENVELOPE_PAD_SCALE + 0.5f);
    return (pad < DOWN_CAR_ENVELOPE_MIN_PAD) ?
               DOWN_CAR_ENVELOPE_MIN_PAD : pad;
}

static int car_local_background_median(const component_t *comp, int pad)
{
    unsigned short histogram[256];
    int min_x = comp->min_x - pad;
    int max_x = comp->max_x + pad;
    int min_y = comp->min_y - pad;
    int max_y = comp->max_y + pad;
    int sample_count = 0;
    int cumulative = 0;
    int target;
    int x;
    int y;

    memset(histogram, 0, sizeof(histogram));
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
        {
            unsigned char gray;
            if (x >= comp->min_x && x <= comp->max_x &&
                y >= comp->min_y && y <= comp->max_y)
            {
                continue;
            }
            gray = g_current_image[y][x];
            histogram[gray]++;
            sample_count++;
        }
    }
    if (sample_count == 0)
    {
        return 0;
    }
    target = (sample_count - 1) / 2;
    for (x = 0; x < 256; x++)
    {
        cumulative += histogram[x];
        if (cumulative > target)
        {
            return x;
        }
    }
    return 0;
}

static unsigned char car_find_core_seed(const component_t *comp,
                                        unsigned char *seed_x,
                                        unsigned char *seed_y)
{
    float best_distance = 1.0e9f;
    int x;
    int y;

    for (y = comp->min_y; y <= comp->max_y; y++)
    {
        for (x = comp->min_x; x <= comp->max_x; x++)
        {
            float dx;
            float dy;
            float distance;
            if (g_binary[y][x] == 0U)
            {
                continue;
            }
            dx = (float)x - comp->cx;
            dy = (float)y - comp->cy;
            distance = dx * dx + dy * dy;
            if (distance < best_distance)
            {
                best_distance = distance;
                *seed_x = (unsigned char)x;
                *seed_y = (unsigned char)y;
            }
        }
    }
    return (best_distance < 1.0e9f) ? 1U : 0U;
}

static unsigned short *grow_car_envelope_enqueue(
    const unsigned char *image,
    unsigned char *visit,
    unsigned int index,
    unsigned short coordinate,
    unsigned char threshold,
    unsigned char stamp,
    unsigned short *tail,
    unsigned short *end)
{
    if (tail >= end || image[index] < threshold || visit[index] == stamp)
    {
        return tail;
    }
    visit[index] = stamp;
    *tail++ = coordinate;
    return tail;
}

static component_t grow_car_envelope(const component_t *core,
                                     int threshold,
                                     int pad)
{
    const unsigned char *image = &g_current_image[0][0];
    unsigned char *visit = g_visit_stamp;
    unsigned short *head = g_queue;
    unsigned short *tail = g_queue;
    unsigned short *end = g_queue + IMAGE_QUEUE_SIZE;
    unsigned char stamp;
    component_t envelope;
    unsigned char seed_x = 0U;
    unsigned char seed_y = 0U;
    int min_x = core->min_x - pad;
    int max_x = core->max_x + pad;
    int min_y = core->min_y - pad;
    int max_y = core->max_y + pad;
    int sum_x = 0;
    int sum_y = 0;

    memset(&envelope, 0, sizeof(envelope));
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    if (car_find_core_seed(core, &seed_x, &seed_y) == 0U)
    {
        return envelope;
    }

    begin_visit_pass();
    stamp = g_current_stamp;
    envelope.min_x = seed_x;
    envelope.max_x = seed_x;
    envelope.min_y = seed_y;
    envelope.max_y = seed_y;
    *tail++ = (unsigned short)seed_x | ((unsigned short)seed_y << 8U);
    visit[(unsigned int)seed_y * BEACON_IMAGE_W + seed_x] = stamp;
    while (head < tail)
    {
        unsigned short coordinate = *head++;
        unsigned char x = (unsigned char)coordinate;
        unsigned char y = (unsigned char)(coordinate >> 8U);
        unsigned int pixel_index = (unsigned int)y * BEACON_IMAGE_W + x;
        unsigned int gray = g_current_image[y][x];
        envelope.area++;
        envelope.gray_sum += gray;
        envelope.gray_sum_sq += gray * gray;
        if (gray > envelope.peak)
        {
            envelope.peak = (unsigned char)gray;
        }
        sum_x += x;
        sum_y += y;
        if ((int)x < envelope.min_x) envelope.min_x = x;
        if ((int)x > envelope.max_x) envelope.max_x = x;
        if ((int)y < envelope.min_y) envelope.min_y = y;
        if ((int)y > envelope.max_y) envelope.max_y = y;
        if ((int)x > min_x && (int)x < max_x &&
            (int)y > min_y && (int)y < max_y)
        {
            tail = grow_car_envelope_enqueue(
                image, visit, pixel_index + 1U,
                (unsigned short)(coordinate + 1U),
                (unsigned char)threshold, stamp, tail, end);
            tail = grow_car_envelope_enqueue(
                image, visit, pixel_index - 1U,
                (unsigned short)(coordinate - 1U),
                (unsigned char)threshold, stamp, tail, end);
            tail = grow_car_envelope_enqueue(
                image, visit, pixel_index + BEACON_IMAGE_W,
                (unsigned short)(coordinate + 0x0100U),
                (unsigned char)threshold, stamp, tail, end);
            tail = grow_car_envelope_enqueue(
                image, visit, pixel_index - BEACON_IMAGE_W,
                (unsigned short)(coordinate - 0x0100U),
                (unsigned char)threshold, stamp, tail, end);
            tail = grow_car_envelope_enqueue(
                image, visit, pixel_index + BEACON_IMAGE_W + 1U,
                (unsigned short)(coordinate + 0x0101U),
                (unsigned char)threshold, stamp, tail, end);
            tail = grow_car_envelope_enqueue(
                image, visit, pixel_index - BEACON_IMAGE_W + 1U,
                (unsigned short)(coordinate - 0x00FFU),
                (unsigned char)threshold, stamp, tail, end);
            tail = grow_car_envelope_enqueue(
                image, visit, pixel_index + BEACON_IMAGE_W - 1U,
                (unsigned short)(coordinate + 0x00FFU),
                (unsigned char)threshold, stamp, tail, end);
            tail = grow_car_envelope_enqueue(
                image, visit, pixel_index - BEACON_IMAGE_W - 1U,
                (unsigned short)(coordinate - 0x0101U),
                (unsigned char)threshold, stamp, tail, end);
        }
        else
        {
#define ENQUEUE_ENVELOPE_IF(valid_, index_, coordinate_) \
            do { if (valid_) { tail = grow_car_envelope_enqueue( \
                image, visit, (index_), (coordinate_), \
                (unsigned char)threshold, stamp, tail, end); } } while (0)
            ENQUEUE_ENVELOPE_IF((int)x < max_x,
                pixel_index + 1U, (unsigned short)(coordinate + 1U));
            ENQUEUE_ENVELOPE_IF((int)x > min_x,
                pixel_index - 1U, (unsigned short)(coordinate - 1U));
            ENQUEUE_ENVELOPE_IF((int)y < max_y,
                pixel_index + BEACON_IMAGE_W,
                (unsigned short)(coordinate + 0x0100U));
            ENQUEUE_ENVELOPE_IF((int)y > min_y,
                pixel_index - BEACON_IMAGE_W,
                (unsigned short)(coordinate - 0x0100U));
            ENQUEUE_ENVELOPE_IF((int)x < max_x && (int)y < max_y,
                pixel_index + BEACON_IMAGE_W + 1U,
                (unsigned short)(coordinate + 0x0101U));
            ENQUEUE_ENVELOPE_IF((int)x < max_x && (int)y > min_y,
                pixel_index - BEACON_IMAGE_W + 1U,
                (unsigned short)(coordinate - 0x00FFU));
            ENQUEUE_ENVELOPE_IF((int)x > min_x && (int)y < max_y,
                pixel_index + BEACON_IMAGE_W - 1U,
                (unsigned short)(coordinate + 0x00FFU));
            ENQUEUE_ENVELOPE_IF((int)x > min_x && (int)y > min_y,
                pixel_index - BEACON_IMAGE_W - 1U,
                (unsigned short)(coordinate - 0x0101U));
#undef ENQUEUE_ENVELOPE_IF
        }
    }
    finish_component(
        &envelope, sum_x, sum_y, (unsigned short)(tail - g_queue));
    envelope.seed_x = seed_x;
    envelope.seed_y = seed_y;
    return envelope;
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static unsigned char car_component_features(
    const component_t *comp,
    down_car_features_t *features)
{
    int pad;
    int background;
    int envelope_threshold;
    float delta;
    float core_elongation_score;
    float envelope_elongation_score;
    float geometry_score;
    float uniformity_score;
    float brightness_score;
    float contrast_score;
    float area_score;
    float photometric_score;

    if (comp == 0 || comp->valid == 0U || features == 0 ||
        g_current_image == 0 || comp->area <= 0 ||
        comp->area > g_image_down_car_lamp_max_area ||
        image_down_horizon_contains(comp->cx, comp->cy,
                                    DOWN_OBJECT_BOUNDARY_MARGIN) == 0U)
    {
        return 0U;
    }
    memset(features, 0, sizeof(*features));
    features->peak = comp->peak;
    features->mean = (float)comp->gray_sum / (float)comp->area;
    features->variance = (float)comp->gray_sum_sq / (float)comp->area -
                         features->mean * features->mean;
    if (features->variance < 0.0f)
    {
        features->variance = 0.0f;
    }
    pad = car_envelope_pad(comp);
    background = car_local_background_median(comp, pad);
    features->contrast = features->mean - (float)background;
    delta = ((float)features->peak - (float)background) * 0.22f;
    if (delta < 20.0f)
    {
        delta = 20.0f;
    }
    envelope_threshold = (int)((float)background + delta + 0.5f);
    if (envelope_threshold < 90) envelope_threshold = 90;
    if (envelope_threshold > 255) envelope_threshold = 255;
    features->envelope = grow_car_envelope(comp, envelope_threshold, pad);
    features->envelope.envelope_threshold =
        (unsigned char)envelope_threshold;
    if (features->envelope.valid == 0U ||
        features->envelope.area > g_image_down_car_lamp_max_area)
    {
        return 0U;
    }

    core_elongation_score = car_score_term(
        comp->elongation,
        DOWN_CAR_SCORE_ELONGATION_BASE,
        DOWN_CAR_SCORE_ELONGATION_RANGE);
    envelope_elongation_score = car_score_term(
        features->envelope.elongation,
        DOWN_CAR_SCORE_ELONGATION_BASE,
        DOWN_CAR_SCORE_ELONGATION_RANGE);
    geometry_score = (core_elongation_score < envelope_elongation_score) ?
                         core_elongation_score : envelope_elongation_score;
    uniformity_score = 1.0f - car_score_term(
        features->variance, 0.0f, DOWN_CAR_SCORE_VARIANCE_RANGE);
    brightness_score = car_score_term(
        features->mean,
        DOWN_CAR_SCORE_MEAN_BASE,
        DOWN_CAR_SCORE_MEAN_RANGE);
    contrast_score = car_score_term(
        features->contrast,
        DOWN_CAR_SCORE_CONTRAST_BASE,
        DOWN_CAR_SCORE_CONTRAST_RANGE);
    area_score = car_score_term(
        (float)features->envelope.area,
        DOWN_CAR_SCORE_AREA_BASE,
        DOWN_CAR_SCORE_AREA_RANGE);
    photometric_score = uniformity_score * 0.30f +
                        brightness_score * 0.25f +
                        contrast_score * 0.25f +
                        area_score * 0.20f;
    features->score = geometry_score * 0.55f +
                      photometric_score * 0.45f;
    if (features->score >= DOWN_CAR_STRONG_SCORE)
    {
        features->classification = DOWN_CAR_CLASS_STRONG;
    }
    else if (features->score >= DOWN_CAR_WEAK_SCORE)
    {
        features->classification = DOWN_CAR_CLASS_WEAK;
    }
    else if (features->score >= DOWN_CAR_TRACK_SCORE)
    {
        features->classification = DOWN_CAR_CLASS_TRACK;
    }
    if(comp->major < DOWN_CAR_COMPACT_EDGE_MAX_MAJOR &&
       (comp->min_x <= DOWN_CAR_COMPACT_EDGE_MARGIN ||
        comp->min_y <= DOWN_CAR_COMPACT_EDGE_MARGIN ||
        comp->max_x >= BEACON_IMAGE_W - 1 - DOWN_CAR_COMPACT_EDGE_MARGIN ||
        comp->max_y >= BEACON_IMAGE_H - 1 - DOWN_CAR_COMPACT_EDGE_MARGIN))
    {
        features->classification = DOWN_CAR_CLASS_TRACK;
    }
    return (features->classification != DOWN_CAR_CLASS_NONE) ? 1U : 0U;
}

static void finish_component(
    component_t *comp,
    int sum_x,
    int sum_y,
    unsigned short coordinate_count)
{
    float inv_area;
    float sum_xx = 0.0f;
    float sum_yy = 0.0f;
    float sum_xy = 0.0f;
    float var_x;
    float var_y;
    float cov_xy;
    float trace;
    float det;
    float disc;
    float eig_delta;
    float eig_major;
    float eig_minor;
    unsigned short *head;
    unsigned short *end;

    if (comp == 0 || comp->area <= 0)
    {
        return;
    }
    inv_area = 1.0f / (float)comp->area;
    comp->cx = (float)sum_x * inv_area;
    comp->cy = (float)sum_y * inv_area;
    head = g_queue;
    end = g_queue + coordinate_count;
    if (coordinate_count <= 479U)
    {
        unsigned int int_sum_xx = 0U;
        unsigned int int_sum_yy = 0U;
        unsigned int int_sum_xy = 0U;
#if defined(__ICCARM__)
        unsigned short *pair_end = g_queue + (coordinate_count & ~1U);
        while (head < pair_end)
        {
            uint32 coordinates = __UNALIGNED_UINT32_READ(head);
            uint32 x_pair = __UXTB16(coordinates);
            uint32 y_pair = __UXTB16(__ROR(coordinates, 8U));
            head += 2;
            int_sum_xx = (unsigned int)__SMLAD(x_pair, x_pair, int_sum_xx);
            int_sum_yy = (unsigned int)__SMLAD(y_pair, y_pair, int_sum_yy);
            int_sum_xy = (unsigned int)__SMLAD(x_pair, y_pair, int_sum_xy);
        }
#endif
        while (head < end)
        {
            unsigned short coordinate = *head++;
            unsigned int x = (unsigned char)coordinate;
            unsigned int y = (unsigned char)(coordinate >> 8U);
            int_sum_xx += x * x;
            int_sum_yy += y * y;
            int_sum_xy += x * y;
        }
        sum_xx = (float)int_sum_xx;
        sum_yy = (float)int_sum_yy;
        sum_xy = (float)int_sum_xy;
    }
    else
    {
        while (head < end)
        {
            unsigned short coordinate = *head++;
            unsigned char x = (unsigned char)coordinate;
            unsigned char y = (unsigned char)(coordinate >> 8U);
            sum_xx += (float)x * (float)x;
            sum_yy += (float)y * (float)y;
            sum_xy += (float)x * (float)y;
        }
    }
    var_x = sum_xx * inv_area - comp->cx * comp->cx;
    var_y = sum_yy * inv_area - comp->cy * comp->cy;
    cov_xy = sum_xy * inv_area - comp->cx * comp->cy;
    trace = var_x + var_y;
    det = var_x * var_y - cov_xy * cov_xy;
    disc = trace * trace * 0.25f - det;
    if (disc < 0.0f)
    {
        disc = 0.0f;
    }
    eig_delta = sqrtf(disc);
    eig_major = trace * 0.5f + eig_delta;
    eig_minor = trace * 0.5f - eig_delta;
    if (eig_minor < 0.0f)
    {
        eig_minor = 0.0f;
    }
    comp->major = 4.0f * sqrtf(eig_major + 0.0001f);
    comp->minor = 4.0f * sqrtf(eig_minor + 0.0001f);
    if (comp->minor < 1.0f)
    {
        comp->minor = 1.0f;
    }
    comp->elongation = comp->major / comp->minor;
    comp->orientation_numerator = 2.0f * cov_xy;
    comp->orientation_denominator = var_x - var_y;
    comp->angle_valid = 0U;
    comp->valid = 1U;
}

static void component_resolve_angle(component_t *comp)
{
    if (comp != 0 && comp->valid != 0U && comp->angle_valid == 0U)
    {
        comp->angle = 0.5f * atan2f(
            comp->orientation_numerator,
            comp->orientation_denominator) * 180.0f / PI_F;
        comp->angle_valid = 1U;
    }
}


static void reset_weak_car_pending(void)
{
    memset(&g_weak_car_pending, 0, sizeof(g_weak_car_pending));
    g_weak_car_pending_hits = 0U;
    g_weak_car_pending_misses = 0U;
}

static void age_weak_car_pending(void)
{
    if (g_weak_car_pending_hits == 0U)
    {
        return;
    }
    if (g_weak_car_pending_misses >= 2U)
    {
        reset_weak_car_pending();
        return;
    }
    g_weak_car_pending_misses++;
}

static unsigned char weak_car_matches(const component_t *left,
                                       const component_t *right)
{
    float dx;
    float dy;
    float major_ratio;
    float minor_ratio;

    if (left == 0 || right == 0 || left->valid == 0 || right->valid == 0 ||
        left->major <= 0.0f || right->major <= 0.0f ||
        left->minor <= 0.0f || right->minor <= 0.0f)
    {
        return 0U;
    }
    dx = left->cx - right->cx;
    dy = left->cy - right->cy;
    major_ratio = left->major / right->major;
    minor_ratio = left->minor / right->minor;
    return (dx * dx + dy * dy <=
                g_image_down_match_distance * g_image_down_match_distance &&
            major_ratio >= 0.5f && major_ratio <= 2.0f &&
            minor_ratio >= 0.5f && minor_ratio <= 2.0f) ? 1U : 0U;
}

static unsigned char weak_car_confirmed(
    const component_t *candidate,
    unsigned char can_start)
{
    unsigned char required_hits = 2U;

    if (candidate == 0 || candidate->valid == 0)
    {
        return 0U;
    }
    if (g_car_track.confirmed != 0U)
    {
        float predicted_x = g_car_track.x + (float)BEACON_IMAGE_W * 0.5f +
                            g_car_track.vx;
        float predicted_y = g_car_track.y + (float)BEACON_IMAGE_H * 0.5f +
                            g_car_track.vy;
        float dx = candidate->cx - predicted_x;
        float dy = candidate->cy - predicted_y;
        if (dx * dx + dy * dy <=
            g_image_down_gate_distance * g_image_down_gate_distance)
        {
            return 1U;
        }
    }
    if (can_start == 0U)
    {
        return 0U;
    }
    if (g_weak_car_pending_hits != 0U &&
        g_weak_car_pending_misses <= 2U &&
        weak_car_matches(candidate, &g_weak_car_pending) != 0U)
    {
        if (g_weak_car_pending_hits < 255U)
        {
            g_weak_car_pending_hits++;
        }
    }
    else
    {
        reset_weak_car_pending();
        g_weak_car_pending_hits = 1U;
    }
    g_weak_car_pending = *candidate;
    g_weak_car_pending_misses = 0U;
    if (g_image_down_confirm_frames > 2)
    {
        required_hits = (g_image_down_confirm_frames > 255) ?
                            255U : (unsigned char)g_image_down_confirm_frames;
    }
    return (g_weak_car_pending_hits >= required_hits) ? 1U : 0U;
}

static void store_car_core(component_t cores[DOWN_CAR_MAX_COMPONENTS],
                           unsigned char *core_count,
                           const component_t *component)
{
    unsigned char index;
    unsigned char smallest = 0U;

    if (*core_count < DOWN_CAR_MAX_COMPONENTS)
    {
        cores[*core_count] = *component;
        (*core_count)++;
        return;
    }
    for (index = 1U; index < *core_count; index++)
    {
        if (cores[index].area < cores[smallest].area)
        {
            smallest = index;
        }
    }
    if (component->area > cores[smallest].area)
    {
        cores[smallest] = *component;
    }
}

static unsigned char same_car_candidate(const component_t *left,
                                        const component_t *right)
{
    float dx = left->cx - right->cx;
    float dy = left->cy - right->cy;
    return (dx * dx + dy * dy <= 9.0f) ? 1U : 0U;
}

static void store_car_candidate(
    down_car_candidate_t candidates[DOWN_CAR_MAX_MASKS],
    unsigned char *candidate_count,
    const down_car_features_t *features,
    const component_t *core,
    unsigned char output_eligible)
{
    unsigned char index;

    for (index = 0U; index < *candidate_count; index++)
    {
        if (same_car_candidate(&candidates[index].component,
                               &features->envelope) == 0U)
        {
            continue;
        }
        if (features->score > candidates[index].score)
        {
            candidates[index].component = features->envelope;
            candidates[index].core = *core;
            component_resolve_angle(&candidates[index].core);
            candidates[index].score = features->score;
            candidates[index].classification = features->classification;
            candidates[index].output_eligible = output_eligible;
        }
        else if(output_eligible != 0U)
        {
            candidates[index].output_eligible = 1U;
        }
        return;
    }
    if (*candidate_count < DOWN_CAR_MAX_MASKS)
    {
        candidates[*candidate_count].component = features->envelope;
        candidates[*candidate_count].core = *core;
        component_resolve_angle(&candidates[*candidate_count].core);
        candidates[*candidate_count].score = features->score;
        candidates[*candidate_count].classification = features->classification;
        candidates[*candidate_count].output_eligible = output_eligible;
        (*candidate_count)++;
    }
}

static unsigned char select_car_candidate(
    const down_car_candidate_t candidates[DOWN_CAR_MAX_MASKS],
    unsigned char candidate_count,
    unsigned char classification)
{
    unsigned char best = 255U;
    unsigned char tracked = 255U;
    unsigned char index;
    float predicted_x = g_car_track.x + (float)BEACON_IMAGE_W * 0.5f +
                        g_car_track.vx;
    float predicted_y = g_car_track.y + (float)BEACON_IMAGE_H * 0.5f +
                        g_car_track.vy;

    for (index = 0U; index < candidate_count; index++)
    {
        if (candidates[index].classification != classification ||
            candidates[index].output_eligible == 0U)
        {
            continue;
        }
        if (best >= candidate_count ||
            candidates[index].score > candidates[best].score)
        {
            best = index;
        }
    }
    if (best >= candidate_count || g_car_track.confirmed == 0U)
    {
        return best;
    }
    {
        float best_distance = g_image_down_gate_distance *
                              g_image_down_gate_distance;
        for (index = 0U; index < candidate_count; index++)
        {
            float dx;
            float dy;
            float distance;
            if (candidates[index].classification != classification ||
                candidates[index].output_eligible == 0U)
            {
                continue;
            }
            dx = candidates[index].component.cx - predicted_x;
            dy = candidates[index].component.cy - predicted_y;
            distance = dx * dx + dy * dy;
            if (distance <= best_distance)
            {
                best_distance = distance;
                tracked = index;
            }
        }
    }
    if (tracked < candidate_count &&
        candidates[tracked].score + DOWN_CAR_TRACK_SCORE_MARGIN >=
            candidates[best].score)
    {
        return tracked;
    }
    return best;
}

static float down_car_axis_angle_difference(float left, float right)
{
    float difference = fabsf(left - right);

    while(difference >= 180.0f)
    {
        difference -= 180.0f;
    }
    return (difference > 90.0f) ? 180.0f - difference : difference;
}

static unsigned char down_car_select_track_group_primary(
    const down_car_candidate_t candidates[DOWN_CAR_MAX_MASKS],
    unsigned char candidate_count)
{
    unsigned char best = 255U;
    unsigned char index;
    float predicted_x;
    float predicted_y;
    float gate_sq;

    if(g_car_track.confirmed == 0U)
    {
        return best;
    }
    predicted_x = g_car_track.x + (float)BEACON_IMAGE_W * 0.5f +
                  g_car_track.vx;
    predicted_y = g_car_track.y + (float)BEACON_IMAGE_H * 0.5f +
                  g_car_track.vy;
    gate_sq = g_image_down_gate_distance * g_image_down_gate_distance;
    if(gate_sq < 1.0f) gate_sq = 1.0f;

    for(index = 0U; index < candidate_count; index++)
    {
        float dx;
        float dy;

        if(candidates[index].output_eligible != 0U ||
           candidates[index].classification < DOWN_CAR_CLASS_WEAK ||
           candidates[index].core.valid == 0U ||
           down_car_axis_angle_difference(
               candidates[index].core.angle, g_car_track.angle) >
               DOWN_CAR_GROUP_ANGLE_MAX)
        {
            continue;
        }
        dx = candidates[index].core.cx - predicted_x;
        dy = candidates[index].core.cy - predicted_y;
        if(dx * dx + dy * dy > gate_sq)
        {
            continue;
        }
        if(best >= candidate_count ||
           candidates[index].classification >
               candidates[best].classification ||
           (candidates[index].classification ==
                candidates[best].classification &&
            candidates[index].score > candidates[best].score))
        {
            best = index;
        }
    }
    return best;
}

static unsigned char down_car_group_cores_match(
    const component_t *primary,
    const component_t *candidate,
    float predicted_x,
    float predicted_y,
    unsigned char allow_compact_axis,
    float *quality)
{
    float angle;
    float axis_x;
    float axis_y;
    float dx;
    float dy;
    float major_offset;
    float minor_offset;
    float major_gap;
    float minor_limit;
    float distance_sq;
    float gate_sq;
    float angle_difference;

    if(primary == 0 || candidate == 0 || quality == 0 ||
       primary->valid == 0U || candidate->valid == 0U)
    {
        return 0U;
    }
    dx = candidate->cx - primary->cx;
    dy = candidate->cy - primary->cy;
    distance_sq = dx * dx + dy * dy;
    gate_sq = g_image_down_gate_distance * g_image_down_gate_distance;
    if(gate_sq < 1.0f) gate_sq = 1.0f;
    if(distance_sq > gate_sq)
    {
        return 0U;
    }
    if(allow_compact_axis != 0U &&
       candidate->elongation < g_image_down_car_lamp_min_elongation)
    {
        float connection_angle = atan2f(dy, dx) * (180.0f / PI_F);
        angle_difference = down_car_axis_angle_difference(
            primary->angle, connection_angle);
    }
    else
    {
        angle_difference = down_car_axis_angle_difference(
            primary->angle, candidate->angle);
    }
    if(angle_difference > DOWN_CAR_GROUP_ANGLE_MAX)
    {
        return 0U;
    }
    dx = candidate->cx - predicted_x;
    dy = candidate->cy - predicted_y;
    if(dx * dx + dy * dy > gate_sq)
    {
        return 0U;
    }

    angle = primary->angle * (PI_F / 180.0f);
    axis_x = cosf(angle);
    axis_y = sinf(angle);
    dx = candidate->cx - primary->cx;
    dy = candidate->cy - primary->cy;
    major_offset = fabsf(dx * axis_x + dy * axis_y);
    minor_offset = fabsf(-dx * axis_y + dy * axis_x);
    major_gap = major_offset -
                (primary->major + candidate->major) * 0.5f;
    minor_limit = (primary->minor + candidate->minor) * 0.5f +
                  DOWN_CAR_GROUP_MINOR_PAD;
    if(major_gap > DOWN_CAR_GROUP_MAJOR_GAP ||
       minor_offset > minor_limit)
    {
        return 0U;
    }
    if(major_gap < 0.0f) major_gap = 0.0f;
    *quality = distance_sq / gate_sq +
               angle_difference / DOWN_CAR_GROUP_ANGLE_MAX +
               minor_offset / ((minor_limit > 1.0f) ? minor_limit : 1.0f) +
               major_gap / DOWN_CAR_GROUP_MAJOR_GAP;
    return 1U;
}

static unsigned char down_car_group_candidate_matches(
    const down_car_candidate_t *primary,
    const down_car_candidate_t *candidate,
    float predicted_x,
    float predicted_y,
    float *quality)
{
    if(primary == 0 || candidate == 0 ||
       (candidate->classification != DOWN_CAR_CLASS_WEAK &&
        candidate->classification != DOWN_CAR_CLASS_STRONG))
    {
        return 0U;
    }
    return down_car_group_cores_match(
        &primary->core, &candidate->core,
        predicted_x, predicted_y, 0U, quality);
}

static unsigned char down_car_select_group(
    const down_car_candidate_t candidates[DOWN_CAR_MAX_MASKS],
    unsigned char candidate_count,
    unsigned char selected,
    unsigned char group[DOWN_CAR_GROUP_MAX_CORES])
{
    unsigned char group_count = 1U;
    unsigned char slot;
    float predicted_x;
    float predicted_y;

    group[0] = selected;
    if(selected >= candidate_count || g_car_track.confirmed == 0U)
    {
        return group_count;
    }
    predicted_x = g_car_track.x + (float)BEACON_IMAGE_W * 0.5f +
                  g_car_track.vx;
    predicted_y = g_car_track.y + (float)BEACON_IMAGE_H * 0.5f +
                  g_car_track.vy;
    for(slot = 1U; slot < DOWN_CAR_GROUP_MAX_CORES; slot++)
    {
        unsigned char index;
        unsigned char best = 255U;
        float best_quality = 1.0e9f;

        for(index = 0U; index < candidate_count; index++)
        {
            unsigned char used = 0U;
            unsigned char matched = 0U;
            unsigned char group_index;
            float quality = 1.0e9f;

            for(group_index = 0U; group_index < group_count; group_index++)
            {
                if(group[group_index] == index)
                {
                    used = 1U;
                    break;
                }
            }
            if(used != 0U ||
               down_car_axis_angle_difference(
                   candidates[selected].core.angle,
                   candidates[index].core.angle) > DOWN_CAR_GROUP_ANGLE_MAX)
            {
                continue;
            }
            for(group_index = 0U; group_index < group_count; group_index++)
            {
                float pair_quality;

                if(down_car_group_candidate_matches(
                       &candidates[group[group_index]], &candidates[index],
                       predicted_x, predicted_y, &pair_quality) != 0U)
                {
                    matched = 1U;
                    if(pair_quality < quality) quality = pair_quality;
                }
            }
            if(matched == 0U) continue;
            if(quality < best_quality)
            {
                best = index;
                best_quality = quality;
            }
        }
        if(best >= candidate_count)
        {
            break;
        }
        group[group_count++] = best;
    }
    return group_count;
}

static unsigned char down_car_point_in_recovery_roi(
    float x,
    float y,
    float predicted_x,
    float predicted_y,
    float axis_x,
    float axis_y,
    float major_limit,
    float minor_limit)
{
    float dx = x - predicted_x;
    float dy = y - predicted_y;
    float major = dx * axis_x + dy * axis_y;
    float minor = -dx * axis_y + dy * axis_x;

    return (fabsf(major) <= major_limit &&
            fabsf(minor) <= minor_limit) ? 1U : 0U;
}

/* 已确认信标只保护与车灯主核心可分离的当前亮核，避免历史误轨迹阻断车灯。 */
static unsigned char down_car_core_near_confirmed_beacon(
    const component_t *primary,
    const component_t *core,
    unsigned char allow_track_roi_override)
{
    float match_distance_sq =
        g_image_down_match_distance * g_image_down_match_distance;
    float primary_angle;
    float primary_axis_x;
    float primary_axis_y;
    float predicted_x;
    float predicted_y;
    float track_angle;
    float track_axis_x;
    float track_axis_y;
    float track_major_limit;
    float track_minor_limit;
    unsigned char index;

    if(primary == 0 || core == 0 ||
       primary->valid == 0U || core->valid == 0U)
    {
        return 0U;
    }
    if(match_distance_sq < 1.0f) match_distance_sq = 1.0f;
    primary_angle = primary->angle * (PI_F / 180.0f);
    primary_axis_x = cosf(primary_angle);
    primary_axis_y = sinf(primary_angle);
    predicted_x = g_car_track.x + (float)BEACON_IMAGE_W * 0.5f +
                  g_car_track.vx;
    predicted_y = g_car_track.y + (float)BEACON_IMAGE_H * 0.5f +
                  g_car_track.vy;
    track_angle = g_car_track.angle * (PI_F / 180.0f);
    track_axis_x = cosf(track_angle);
    track_axis_y = sinf(track_angle);
    track_major_limit = g_car_track.length * 0.5f +
                        DOWN_CAR_GROUP_MAJOR_GAP +
                        DOWN_CAR_RECOVER_SUPPORT_PAD;
    track_minor_limit = g_car_track.width * 0.5f +
                        DOWN_CAR_GROUP_MINOR_PAD;

#define DOWN_CAR_BEACON_TRACK_PROTECT(track_x, track_y)                 \
    do                                                                  \
    {                                                                   \
        float protect_x = (track_x);                                    \
        float protect_y = (track_y);                                    \
        float core_dx = core->cx - protect_x;                           \
        float core_dy = core->cy - protect_y;                           \
                                                                        \
        if(core_dx * core_dx + core_dy * core_dy <= match_distance_sq &&\
           down_car_point_in_recovery_roi(                              \
               protect_x, protect_y, primary->cx, primary->cy,          \
               primary_axis_x, primary_axis_y,                          \
               primary->major * 0.5f + DOWN_CAR_SUPPORT_PAD,            \
               primary->minor * 0.5f + DOWN_CAR_SUPPORT_PAD) == 0U &&   \
           (allow_track_roi_override == 0U ||                           \
            down_car_point_in_recovery_roi(                             \
                protect_x, protect_y, predicted_x, predicted_y,         \
                track_axis_x, track_axis_y,                             \
                track_major_limit, track_minor_limit) == 0U))           \
        {                                                               \
            return 1U;                                                  \
        }                                                               \
    } while(0)

    if(g_b0_track.confirmed != 0U)
    {
        DOWN_CAR_BEACON_TRACK_PROTECT(
            g_b0_track.x + (float)BEACON_IMAGE_W * 0.5f + g_b0_track.vx,
            g_b0_track.y + (float)BEACON_IMAGE_H * 0.5f + g_b0_track.vy);
    }
    for(index = 0U; index < DOWN_NEAR_CAR_BEACON_TRACKS; index++)
    {
        const down_near_car_beacon_track_t *track =
            &g_near_car_beacon_tracks[index];

        if(track->active == 0U || track->confirmed == 0U)
        {
            continue;
        }
        DOWN_CAR_BEACON_TRACK_PROTECT(
            track->x + (float)BEACON_IMAGE_W * 0.5f,
            track->y + (float)BEACON_IMAGE_H * 0.5f);
    }
    for(index = 0U; index < DOWN_EDGE_BEACON_TRACKS; index++)
    {
        const down_edge_beacon_track_t *track =
            &g_edge_beacon_tracks[index];

        if(track->active == 0U || track->confirmed == 0U)
        {
            continue;
        }
        DOWN_CAR_BEACON_TRACK_PROTECT(
            track->x + track->vx,
            track->y + track->vy);
    }
#undef DOWN_CAR_BEACON_TRACK_PROTECT
    return 0U;
}

/* 仅在已确认车灯的严格预测范围内，回收与正式车灯组同包络的小亮核。 */
static unsigned char down_car_select_recoverable_cores(
    const down_car_candidate_t candidates[DOWN_CAR_MAX_MASKS],
    const unsigned char group[DOWN_CAR_GROUP_MAX_CORES],
    unsigned char group_count,
    const component_t cores[DOWN_CAR_MAX_COMPONENTS],
    const unsigned char core_classifications[DOWN_CAR_MAX_COMPONENTS],
    unsigned char core_count,
    unsigned char recovered_indices[DOWN_CAR_GROUP_MAX_CORES],
    unsigned char connect_threshold,
    unsigned char allow_track_roi_override,
    unsigned char use_strict_classification)
{
    static const signed char neighbor_x[8] =
        {1, -1, 0, 0, 1, 1, -1, -1};
    static const signed char neighbor_y[8] =
        {0, 0, 1, -1, 1, -1, 1, -1};
    unsigned short *head = g_queue;
    unsigned short *tail = g_queue;
    unsigned short *queue_end = g_queue + IMAGE_QUEUE_SIZE;
    unsigned char recovered_count = 0U;
    unsigned char selected;
    unsigned char stamp;
    unsigned char index;
    float predicted_x;
    float predicted_y;
    float gate_sq;
    float angle;
    float axis_x;
    float axis_y;
    float major_limit;
    float minor_limit;

    if(candidates == 0 || group == 0 || cores == 0 ||
       core_classifications == 0 || recovered_indices == 0 ||
       group_count == 0U || group_count >= DOWN_CAR_GROUP_MAX_CORES ||
       g_car_track.confirmed == 0U || g_current_image == 0)
    {
        return 0U;
    }
    selected = group[0];
    if(selected >= DOWN_CAR_MAX_MASKS ||
       candidates[selected].core.valid == 0U ||
       candidates[selected].classification < DOWN_CAR_CLASS_TRACK ||
       g_car_track.length <= 0.0f || g_car_track.width <= 0.0f)
    {
        return 0U;
    }
    predicted_x = g_car_track.x + (float)BEACON_IMAGE_W * 0.5f +
                  g_car_track.vx;
    predicted_y = g_car_track.y + (float)BEACON_IMAGE_H * 0.5f +
                  g_car_track.vy;
    gate_sq = g_image_down_gate_distance * g_image_down_gate_distance;
    if(gate_sq < 1.0f) gate_sq = 1.0f;
    {
        float dx = candidates[selected].core.cx - predicted_x;
        float dy = candidates[selected].core.cy - predicted_y;

        if(dx * dx + dy * dy > gate_sq ||
           down_car_axis_angle_difference(
               candidates[selected].core.angle, g_car_track.angle) >
               DOWN_CAR_GROUP_ANGLE_MAX)
        {
            return 0U;
        }
    }
    angle = g_car_track.angle * (PI_F / 180.0f);
    axis_x = cosf(angle);
    axis_y = sinf(angle);
    major_limit = g_car_track.length * 0.5f + DOWN_CAR_GROUP_MAJOR_GAP +
                  DOWN_CAR_RECOVER_SUPPORT_PAD;
    minor_limit = g_car_track.width * 0.5f + DOWN_CAR_GROUP_MINOR_PAD;
    if(connect_threshold == DOWN_CAR_RECOVER_BRIDGE_THRESHOLD)
    {
        major_limit = g_image_down_gate_distance;
        minor_limit = g_image_down_gate_distance;
        if(major_limit < 1.0f) major_limit = 1.0f;
        if(minor_limit < 1.0f) minor_limit = 1.0f;
    }

    begin_visit_pass();
    stamp = g_current_stamp;
    for(index = 0U; index < group_count; index++)
    {
        const component_t *core = &candidates[group[index]].core;
        unsigned int pixel_index;

        if(core->valid == 0U || core->seed_x >= BEACON_IMAGE_W ||
           core->seed_y >= BEACON_IMAGE_H ||
           g_current_image[core->seed_y][core->seed_x] <
               connect_threshold ||
           down_car_point_in_recovery_roi(
               core->seed_x, core->seed_y, predicted_x, predicted_y,
               axis_x, axis_y, major_limit, minor_limit) == 0U)
        {
            continue;
        }
        pixel_index = (unsigned int)core->seed_y * BEACON_IMAGE_W +
                      core->seed_x;
        if(g_visit_stamp[pixel_index] == stamp)
        {
            continue;
        }
        if(tail >= queue_end)
        {
            return 0U;
        }
        g_visit_stamp[pixel_index] = stamp;
        *tail++ = (unsigned short)(core->seed_x |
                                  ((unsigned short)core->seed_y << 8U));
    }
    if(head == tail)
    {
        return 0U;
    }
    while(head < tail)
    {
        unsigned short coordinate = *head++;
        int center_x = (unsigned char)coordinate;
        int center_y = (unsigned char)(coordinate >> 8U);
        unsigned char neighbor;

        for(neighbor = 0U; neighbor < 8U; neighbor++)
        {
            int next_x = center_x + neighbor_x[neighbor];
            int next_y = center_y + neighbor_y[neighbor];
            unsigned int next_index;

            if(next_x < 0 || next_x >= BEACON_IMAGE_W ||
               next_y < 0 || next_y >= BEACON_IMAGE_H ||
               g_current_image[next_y][next_x] <
                   connect_threshold ||
               down_car_point_in_recovery_roi(
                   next_x, next_y, predicted_x, predicted_y,
                   axis_x, axis_y, major_limit, minor_limit) == 0U)
            {
                continue;
            }
            next_index = (unsigned int)next_y * BEACON_IMAGE_W + next_x;
            if(g_visit_stamp[next_index] == stamp)
            {
                continue;
            }
            if(tail >= queue_end)
            {
                return 0U;
            }
            g_visit_stamp[next_index] = stamp;
            *tail++ = (unsigned short)(next_x | (next_y << 8));
        }
    }

    while(group_count + recovered_count < DOWN_CAR_GROUP_MAX_CORES)
    {
        unsigned char best = 255U;
        float best_quality = 1.0e9f;

        for(index = 0U; index < core_count; index++)
        {
            const component_t *core = &cores[index];
            unsigned char used = 0U;
            unsigned char matched = 0U;
            unsigned char member;
            float quality = 1.0e9f;
            unsigned int seed_index;

            if(core->valid == 0U ||
               (use_strict_classification != 0U &&
                !((core_classifications[index] < DOWN_CAR_CLASS_WEAK) ||
                  (core_classifications[index] >= DOWN_CAR_CLASS_STRONG &&
                   core->elongation >=
                       g_image_down_car_lamp_min_elongation))) ||
               (use_strict_classification == 0U &&
                allow_track_roi_override == 0U &&
                (core_classifications[index] == DOWN_CAR_CLASS_WEAK ||
                 (core_classifications[index] == DOWN_CAR_CLASS_NONE &&
                  core->elongation >= g_image_down_car_lamp_min_elongation) ||
                 (core_classifications[index] >= DOWN_CAR_CLASS_STRONG &&
                  core->elongation < g_image_down_car_lamp_min_elongation))) ||
               core->area < DOWN_CAR_RECOVER_MIN_CORE_AREA ||
               core->area > DOWN_CAR_RECOVER_MAX_CORE_AREA ||
               core->seed_x >= BEACON_IMAGE_W ||
               core->seed_y >= BEACON_IMAGE_H ||
               down_car_point_in_recovery_roi(
                   core->cx, core->cy, predicted_x, predicted_y,
                   axis_x, axis_y, major_limit, minor_limit) == 0U)
            {
                continue;
            }
            if(down_car_core_near_confirmed_beacon(
                   &candidates[selected].core, core,
                   allow_track_roi_override) != 0U)
            {
                continue;
            }
            seed_index = (unsigned int)core->seed_y * BEACON_IMAGE_W +
                         core->seed_x;
            if(g_visit_stamp[seed_index] != stamp)
            {
                continue;
            }
            for(member = 0U; member < group_count; member++)
            {
                const component_t *group_core =
                    &candidates[group[member]].core;
                float pair_quality;

                if(group_core->seed_x == core->seed_x &&
                   group_core->seed_y == core->seed_y)
                {
                    used = 1U;
                    break;
                }
                if(down_car_group_cores_match(
                       group_core, core, predicted_x, predicted_y,
                       1U, &pair_quality) != 0U)
                {
                    matched = 1U;
                    if(pair_quality < quality) quality = pair_quality;
                }
            }
            if(used != 0U)
            {
                continue;
            }
            for(member = 0U; member < recovered_count; member++)
            {
                const component_t *group_core =
                    &cores[recovered_indices[member]];
                float pair_quality;

                if(recovered_indices[member] == index)
                {
                    used = 1U;
                    break;
                }
                if(down_car_group_cores_match(
                       group_core, core, predicted_x, predicted_y,
                       1U, &pair_quality) != 0U)
                {
                    matched = 1U;
                    if(pair_quality < quality) quality = pair_quality;
                }
            }
            if(used != 0U || matched == 0U)
            {
                continue;
            }
            if(quality < best_quality)
            {
                best = index;
                best_quality = quality;
            }
        }
        if(best >= core_count)
        {
            break;
        }
        recovered_indices[recovered_count++] = best;
    }
    return recovered_count;
}

static void down_car_mark_core_pixels(const component_t *core)
{
    component_t connected;
    unsigned short *coordinate;
    unsigned short *end;

    if(core == 0 || core->valid == 0U ||
       core->seed_x >= BEACON_IMAGE_W || core->seed_y >= BEACON_IMAGE_H ||
       g_binary[core->seed_y][core->seed_x] == 0U)
    {
        return;
    }
    begin_visit_pass();
    connected = grow_component(core->seed_x, core->seed_y);
    coordinate = g_queue;
    end = g_queue + connected.area;
    while(coordinate < end)
    {
        unsigned short value = *coordinate++;
        unsigned int pixel_index =
            (unsigned int)(unsigned char)(value >> 8U) * BEACON_IMAGE_W +
            (unsigned char)value;
        car_support_set(pixel_index, 1U);
    }
}

static unsigned char down_car_fit_support_geometry(
    component_t *lamp,
    int min_x,
    int max_x,
    int min_y,
    int max_y,
    unsigned char envelope_threshold)
{
    component_t fitted;
    unsigned short *tail = g_queue;
    unsigned short *head;
    unsigned short *end;
    int sum_x = 0;
    int sum_y = 0;
    float axis_x;
    float axis_y;
    float orientation_norm;
    float cos_double_angle;
    float pixel_extent;
    float min_major = 1.0e9f;
    float max_major = -1.0e9f;
    float min_minor = 1.0e9f;
    float max_minor = -1.0e9f;
    int x;
    int y;

    memset(&fitted, 0, sizeof(fitted));
    fitted.min_x = BEACON_IMAGE_W - 1;
    fitted.min_y = BEACON_IMAGE_H - 1;
    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            unsigned int pixel_index = (unsigned int)y * BEACON_IMAGE_W + x;
            unsigned int gray;

            if(car_support_get(pixel_index) == 0U)
            {
                continue;
            }
            if(tail >= g_queue + IMAGE_QUEUE_SIZE)
            {
                return 0U;
            }
            *tail++ = (unsigned short)(x | (y << 8));
            fitted.area++;
            sum_x += x;
            sum_y += y;
            gray = g_current_image[y][x];
            fitted.gray_sum += gray;
            fitted.gray_sum_sq += gray * gray;
            if(gray > fitted.peak) fitted.peak = (unsigned char)gray;
            if(x < fitted.min_x) fitted.min_x = x;
            if(x > fitted.max_x) fitted.max_x = x;
            if(y < fitted.min_y) fitted.min_y = y;
            if(y > fitted.max_y) fitted.max_y = y;
        }
    }
    if(fitted.area <= 0 || fitted.area > g_image_down_car_lamp_max_area)
    {
        return 0U;
    }
    finish_component(&fitted, sum_x, sum_y, (unsigned short)fitted.area);
    fitted.seed_x = lamp->seed_x;
    fitted.seed_y = lamp->seed_y;
    fitted.envelope_threshold = envelope_threshold;
    component_resolve_angle(&fitted);
    orientation_norm = sqrtf(
        fitted.orientation_numerator * fitted.orientation_numerator +
        fitted.orientation_denominator * fitted.orientation_denominator);
    if(orientation_norm > 0.0001f)
    {
        cos_double_angle =
            fitted.orientation_denominator / orientation_norm;
        if(cos_double_angle < -1.0f) cos_double_angle = -1.0f;
        if(cos_double_angle > 1.0f) cos_double_angle = 1.0f;
        axis_x = sqrtf((1.0f + cos_double_angle) * 0.5f);
        axis_y = sqrtf((1.0f - cos_double_angle) * 0.5f);
        if(fitted.orientation_numerator < 0.0f) axis_y = -axis_y;
    }
    else
    {
        axis_x = 1.0f;
        axis_y = 0.0f;
    }
    head = g_queue;
    end = tail;
    while(head < end)
    {
        unsigned short coordinate = *head++;
        float pixel_x = (float)(unsigned char)coordinate;
        float pixel_y = (float)(unsigned char)(coordinate >> 8U);
        float major = pixel_x * axis_x + pixel_y * axis_y;
        float minor = -pixel_x * axis_y + pixel_y * axis_x;

        if(major < min_major) min_major = major;
        if(major > max_major) max_major = major;
        if(minor < min_minor) min_minor = minor;
        if(minor > max_minor) max_minor = minor;
    }
    pixel_extent = fabsf(axis_x) + fabsf(axis_y);
    fitted.major = max_major - min_major + pixel_extent;
    fitted.minor = max_minor - min_minor + pixel_extent;
    fitted.cx = ((min_major + max_major) * axis_x -
                 (min_minor + max_minor) * axis_y) * 0.5f;
    fitted.cy = ((min_major + max_major) * axis_y +
                 (min_minor + max_minor) * axis_x) * 0.5f;
    if(fitted.minor < 1.0f) fitted.minor = 1.0f;
    fitted.elongation = fitted.major / fitted.minor;
    *lamp = fitted;

    head = g_queue;
    while(head < end)
    {
        unsigned short coordinate = *head++;
        int center_x = (unsigned char)coordinate;
        int center_y = (unsigned char)(coordinate >> 8U);
        int pad_x;
        int pad_y;

        for(pad_y = -DOWN_CAR_SUPPORT_PAD;
            pad_y <= DOWN_CAR_SUPPORT_PAD; pad_y++)
        {
            int support_y = center_y + pad_y;
            if(support_y < 0 || support_y >= BEACON_IMAGE_H) continue;
            for(pad_x = -DOWN_CAR_SUPPORT_PAD;
                pad_x <= DOWN_CAR_SUPPORT_PAD; pad_x++)
            {
                int support_x = center_x + pad_x;
                if(support_x < 0 || support_x >= BEACON_IMAGE_W) continue;
                car_support_set(
                    (unsigned int)support_y * BEACON_IMAGE_W + support_x, 1U);
            }
        }
    }
    g_car_support_valid = 1U;
    return 1U;
}

/* 分类和轨迹继续使用动态包络，最终输出只拟合阈值120的实际亮核。 */
static unsigned char down_car_fit_output_geometry(
    const component_t *identity,
    component_t *output)
{
    component_t fitted;

    if(identity == 0 || output == 0 || identity->valid == 0U ||
       identity->seed_x >= BEACON_IMAGE_W ||
       identity->seed_y >= BEACON_IMAGE_H || g_current_image == 0)
    {
        return 0U;
    }
    fitted = grow_car_envelope(
        identity, DOWN_GRAY_SUPPORT_THRESHOLD,
        DOWN_CAR_OUTPUT_ENVELOPE_PAD);
    if(fitted.valid == 0U ||
       fitted.area < g_image_down_car_lamp_min_area)
    {
        return 0U;
    }
    fitted.seed_x = identity->seed_x;
    fitted.seed_y = identity->seed_y;
    fitted.envelope_threshold = DOWN_GRAY_SUPPORT_THRESHOLD;
    component_resolve_angle(&fitted);
    *output = fitted;
    return 1U;
}

/* 低阈值只证明连通性，最终支持掩码和几何统一收紧到阈值120。 */
static unsigned char down_car_refit_bridge_output(
    component_t *merged,
    component_t *output)
{
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int x;
    int y;

    if(merged == 0 || output == 0 || merged->valid == 0U ||
       g_car_support_valid == 0U || g_current_image == 0)
    {
        return 0U;
    }
    min_x = merged->min_x - DOWN_CAR_SUPPORT_PAD;
    max_x = merged->max_x + DOWN_CAR_SUPPORT_PAD;
    min_y = merged->min_y - DOWN_CAR_SUPPORT_PAD;
    max_y = merged->max_y + DOWN_CAR_SUPPORT_PAD;
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            unsigned int pixel_index = (unsigned int)y * BEACON_IMAGE_W + x;

            if(car_support_get(pixel_index) != 0U &&
               g_current_image[y][x] < DOWN_GRAY_SUPPORT_THRESHOLD)
            {
                car_support_set(pixel_index, 0U);
            }
        }
    }
    *output = *merged;
    if(down_car_fit_support_geometry(
           output, min_x, max_x, min_y, max_y,
           DOWN_GRAY_SUPPORT_THRESHOLD) == 0U ||
       output->area < g_image_down_car_lamp_min_area)
    {
        return 0U;
    }
    return 1U;
}

static unsigned char down_car_assign_support(
    component_t *lamp,
    const down_car_candidate_t *candidates,
    const unsigned char *group,
    unsigned char group_count,
    const component_t *extra_cores,
    const unsigned char *extra_indices,
    unsigned char extra_count)
{
    static const signed char neighbor_x[8] =
        {1, -1, 0, 0, 1, 1, -1, -1};
    static const signed char neighbor_y[8] =
        {0, 0, 1, -1, 1, -1, 1, -1};
    unsigned short *head = g_queue;
    unsigned short *tail = g_queue;
    unsigned short *queue_end = g_queue + IMAGE_QUEUE_SIZE;
    unsigned char stamp;
    unsigned char threshold = 255U;
    unsigned char index;
    int min_x = BEACON_IMAGE_W - 1;
    int max_x = 0;
    int min_y = BEACON_IMAGE_H - 1;
    int max_y = 0;
    int x;
    int y;

    if(lamp == 0 || candidates == 0 || group == 0 || group_count == 0U ||
       (extra_count != 0U &&
        (extra_cores == 0 || extra_indices == 0)))
    {
        return 0U;
    }
    memset(g_car_support_mask, 0, sizeof(g_car_support_mask));
    g_car_support_valid = 0U;
    for(index = 0U; index < group_count; index++)
    {
        const down_car_candidate_t *candidate = &candidates[group[index]];

        if(candidate->component.min_x < min_x) min_x = candidate->component.min_x;
        if(candidate->component.max_x > max_x) max_x = candidate->component.max_x;
        if(candidate->component.min_y < min_y) min_y = candidate->component.min_y;
        if(candidate->component.max_y > max_y) max_y = candidate->component.max_y;
        if(candidate->component.envelope_threshold != 0U &&
           candidate->component.envelope_threshold < threshold)
        {
            threshold = candidate->component.envelope_threshold;
        }
        down_car_mark_core_pixels(&candidate->core);
    }
    for(index = 0U; index < extra_count; index++)
    {
        const component_t *core = &extra_cores[extra_indices[index]];

        if(core->valid == 0U)
        {
            continue;
        }
        if(core->min_x < min_x) min_x = core->min_x;
        if(core->max_x > max_x) max_x = core->max_x;
        if(core->min_y < min_y) min_y = core->min_y;
        if(core->max_y > max_y) max_y = core->max_y;
        down_car_mark_core_pixels(core);
    }
    if(threshold == 255U) threshold = DOWN_GRAY_SUPPORT_THRESHOLD;
    min_x -= DOWN_CAR_OUTPUT_ENVELOPE_PAD;
    max_x += DOWN_CAR_OUTPUT_ENVELOPE_PAD;
    min_y -= DOWN_CAR_OUTPUT_ENVELOPE_PAD;
    max_y += DOWN_CAR_OUTPUT_ENVELOPE_PAD;
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    begin_visit_pass();
    stamp = g_current_stamp;
    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            unsigned int pixel_index = (unsigned int)y * BEACON_IMAGE_W + x;

            if(g_binary[y][x] == 0U)
            {
                continue;
            }
            if(tail >= queue_end)
            {
                return 0U;
            }
            g_visit_stamp[pixel_index] = stamp;
            *tail++ = (unsigned short)(x | (y << 8));
        }
    }
    if(head == tail)
    {
        return 0U;
    }
    while(head < tail)
    {
        unsigned short coordinate = *head++;
        int center_x = (unsigned char)coordinate;
        int center_y = (unsigned char)(coordinate >> 8U);
        unsigned int center_index =
            (unsigned int)center_y * BEACON_IMAGE_W + center_x;
        unsigned char owner = car_support_get(center_index);

        for(index = 0U; index < 8U; index++)
        {
            int next_x = center_x + neighbor_x[index];
            int next_y = center_y + neighbor_y[index];
            unsigned int next_index;

            if(next_x < min_x || next_x > max_x ||
               next_y < min_y || next_y > max_y)
            {
                continue;
            }
            next_index = (unsigned int)next_y * BEACON_IMAGE_W + next_x;
            if(g_visit_stamp[next_index] == stamp ||
               g_current_image[next_y][next_x] < threshold)
            {
                continue;
            }
            if(tail >= queue_end)
            {
                return 0U;
            }
            g_visit_stamp[next_index] = stamp;
            car_support_set(next_index, owner);
            *tail++ = (unsigned short)(next_x | (next_y << 8));
        }
    }
    return down_car_fit_support_geometry(
        lamp, min_x, max_x, min_y, max_y, threshold);
}

static unsigned char down_car_recovered_support_matches_track(
    const component_t *lamp,
    float maximum_major_ratio)
{
    float predicted_x;
    float predicted_y;
    float dx;
    float dy;
    float gate_sq;
    float major_ratio;
    float minor_ratio;

    if(lamp == 0 || lamp->valid == 0U ||
       lamp->area < g_image_down_car_lamp_min_area ||
       g_car_track.confirmed == 0U ||
       g_car_track.length <= 0.0f || g_car_track.width <= 0.0f)
    {
        return 0U;
    }
    predicted_x = g_car_track.x + (float)BEACON_IMAGE_W * 0.5f +
                  g_car_track.vx;
    predicted_y = g_car_track.y + (float)BEACON_IMAGE_H * 0.5f +
                  g_car_track.vy;
    dx = lamp->cx - predicted_x;
    dy = lamp->cy - predicted_y;
    gate_sq = g_image_down_gate_distance * g_image_down_gate_distance;
    if(gate_sq < 1.0f) gate_sq = 1.0f;
    major_ratio = lamp->major / g_car_track.length;
    minor_ratio = lamp->minor / g_car_track.width;
    return (dx * dx + dy * dy <= gate_sq &&
            down_car_axis_angle_difference(
                lamp->angle, g_car_track.angle) <=
                DOWN_CAR_GROUP_ANGLE_MAX &&
            major_ratio >= 0.5f && major_ratio <= maximum_major_ratio &&
            minor_ratio >= 0.5f && minor_ratio <= 2.0f) ? 1U : 0U;
}

static unsigned char down_car_recovered_support_improves_fragment(
    const component_t *original,
    const component_t *recovered)
{
    if(original == 0 || recovered == 0 ||
       original->valid == 0U || recovered->valid == 0U)
    {
        return 0U;
    }
    return (recovered->major > original->major + 1.0f &&
            recovered->area > original->area) ? 1U : 0U;
}

/*
 * 阈值80仅用于确认碎片与车灯同属一片光晕；最终输出和互斥掩码
 * 均重新收紧到阈值120。失败时由调用方重建原有支持区域。
 */
static unsigned char down_car_try_bridge_output(
    const component_t *identity,
    const down_car_candidate_t candidates[DOWN_CAR_MAX_MASKS],
    const unsigned char group[DOWN_CAR_GROUP_MAX_CORES],
    unsigned char group_count,
    const component_t cores[DOWN_CAR_MAX_COMPONENTS],
    const unsigned char core_classifications[DOWN_CAR_MAX_COMPONENTS],
    unsigned char core_count,
    const unsigned char *base_extra_indices,
    unsigned char base_extra_count)
{
    unsigned char recovered_indices[DOWN_CAR_GROUP_MAX_CORES];
    unsigned char combined_indices[DOWN_CAR_GROUP_MAX_CORES];
    unsigned char recovered_count;
    unsigned char combined_count = 0U;
    unsigned char added_count = 0U;
    unsigned char index;
    component_t merged;
    component_t output;

    if(identity == 0 || identity->valid == 0U ||
       g_car_track.confirmed == 0U)
    {
        return 0U;
    }
    recovered_count = down_car_select_recoverable_cores(
        candidates, group, group_count,
        cores, core_classifications, core_count, recovered_indices,
        DOWN_CAR_RECOVER_BRIDGE_THRESHOLD, 1U, 1U);
    if(recovered_count == 0U)
    {
        return 0U;
    }
    for(index = 0U;
        index < base_extra_count &&
        group_count + combined_count < DOWN_CAR_GROUP_MAX_CORES;
        index++)
    {
        combined_indices[combined_count++] = base_extra_indices[index];
    }
    for(index = 0U;
        index < recovered_count &&
        group_count + combined_count < DOWN_CAR_GROUP_MAX_CORES;
        index++)
    {
        unsigned char duplicate = 0U;
        unsigned char existing;

        for(existing = 0U; existing < combined_count; existing++)
        {
            if(combined_indices[existing] == recovered_indices[index])
            {
                duplicate = 1U;
                break;
            }
        }
        if(duplicate == 0U)
        {
            combined_indices[combined_count++] = recovered_indices[index];
            added_count++;
        }
    }
    if(added_count == 0U)
    {
        return 0U;
    }
    merged = *identity;
    if(down_car_assign_support(
           &merged, candidates, group, group_count,
           cores, combined_indices, combined_count) == 0U)
    {
        return 0U;
    }
    if(down_car_refit_bridge_output(&merged, &output) == 0U)
    {
        return 0U;
    }
    if(down_car_recovered_support_matches_track(&output, 2.2f) == 0U)
    {
        return 0U;
    }
    if(down_car_recovered_support_improves_fragment(identity, &output) == 0U)
    {
        return 0U;
    }
    g_car_output_override = output;
    g_car_output_override_valid = 1U;
    return 1U;
}

static void down_car_clear_support(void)
{
    memset(g_car_support_mask, 0, sizeof(g_car_support_mask));
    g_car_support_valid = 0U;
}

static unsigned char down_car_finalize_candidate(
    component_t *lamp,
    const down_car_candidate_t candidates[DOWN_CAR_MAX_MASKS],
    unsigned char candidate_count,
    unsigned char selected,
    const component_t cores[DOWN_CAR_MAX_COMPONENTS],
    const unsigned char core_classifications[DOWN_CAR_MAX_COMPONENTS],
    unsigned char core_count)
{
    unsigned char group[DOWN_CAR_GROUP_MAX_CORES];
    unsigned char recovered_indices[DOWN_CAR_GROUP_MAX_CORES];
    unsigned char group_count;
    unsigned char recovered_count;
    component_t original_lamp;

    if(lamp == 0 || selected >= candidate_count)
    {
        return 0U;
    }
    original_lamp = candidates[selected].component;
    *lamp = original_lamp;
    component_resolve_angle(lamp);
    group_count = down_car_select_group(
        candidates, candidate_count, selected, group);
    recovered_count = down_car_select_recoverable_cores(
        candidates, group, group_count,
        cores, core_classifications, core_count, recovered_indices,
        DOWN_CAR_RECOVER_CONNECT_THRESHOLD, 0U, 0U);
    if(recovered_count != 0U)
    {
        if(down_car_assign_support(
               lamp, candidates, group, group_count,
               cores, recovered_indices, recovered_count) != 0U &&
           down_car_recovered_support_matches_track(lamp, 2.0f) != 0U)
        {
            component_t identity_lamp = *lamp;

            if(down_car_try_bridge_output(
                   &identity_lamp, candidates, group, group_count,
                   cores, core_classifications, core_count,
                   recovered_indices, recovered_count) == 0U)
            {
                (void)down_car_assign_support(
                    lamp, candidates, group, group_count,
                    cores, recovered_indices, recovered_count);
            }
            *lamp = identity_lamp;
            component_resolve_angle(lamp);
            return 1U;
        }
        /* 合并失败时完整回退，不能删除原候选车灯输出。 */
        down_car_clear_support();
        *lamp = original_lamp;
        component_resolve_angle(lamp);
    }
    recovered_count = down_car_select_recoverable_cores(
        candidates, group, group_count,
        cores, core_classifications, core_count, recovered_indices,
        DOWN_CAR_RECOVER_TRACK_THRESHOLD, 1U, 0U);
    if(recovered_count != 0U)
    {
        if(down_car_assign_support(
               lamp, candidates, group, group_count,
               cores, recovered_indices, recovered_count) != 0U &&
           down_car_recovered_support_matches_track(lamp, 2.0f) != 0U &&
           down_car_recovered_support_improves_fragment(
               &candidates[selected].core, lamp) != 0U)
        {
            component_t identity_lamp = *lamp;

            if(down_car_try_bridge_output(
                   &identity_lamp, candidates, group, group_count,
                   cores, core_classifications, core_count,
                   recovered_indices, recovered_count) == 0U)
            {
                (void)down_car_assign_support(
                    lamp, candidates, group, group_count,
                    cores, recovered_indices, recovered_count);
            }
            *lamp = identity_lamp;
            component_resolve_angle(lamp);
            return 1U;
        }
        down_car_clear_support();
        *lamp = original_lamp;
        component_resolve_angle(lamp);
    }
    if(down_car_assign_support(
           lamp, candidates, group, group_count, 0, 0, 0U) != 0U)
    {
        component_t identity_lamp = *lamp;

        if(down_car_try_bridge_output(
               &identity_lamp, candidates, group, group_count,
               cores, core_classifications, core_count, 0, 0U) == 0U)
        {
            (void)down_car_assign_support(
                lamp, candidates, group, group_count, 0, 0, 0U);
        }
        *lamp = identity_lamp;
        component_resolve_angle(lamp);
        return 1U;
    }
    if(group_count <= 1U)
    {
        down_car_clear_support();
        return 0U;
    }

    *lamp = original_lamp;
    component_resolve_angle(lamp);
    group[0] = selected;
    if(down_car_assign_support(
           lamp, candidates, group, 1U, 0, 0, 0U) != 0U)
    {
        component_t identity_lamp = *lamp;

        if(down_car_try_bridge_output(
               &identity_lamp, candidates, group, 1U,
               cores, core_classifications, core_count, 0, 0U) == 0U)
        {
            (void)down_car_assign_support(
                lamp, candidates, group, 1U, 0, 0, 0U);
        }
        *lamp = identity_lamp;
        component_resolve_angle(lamp);
        return 1U;
    }
    down_car_clear_support();
    return 0U;
}

static void down_car_mark_unreported_priority_core(
    const down_car_candidate_t *candidates,
    unsigned char candidate_count)
{
    unsigned char group[DOWN_CAR_GROUP_MAX_CORES] = {0U, 0U, 0U};
    unsigned char best = 255U;
    unsigned char index;
    component_t ignored_geometry;

    for(index = 0U; index < candidate_count; index++)
    {
        if(candidates[index].output_eligible != 0U ||
           candidates[index].classification < DOWN_CAR_CLASS_WEAK ||
           down_gray_side_edge_beacon_like(
               g_current_image, &candidates[index].component) != 0U)
        {
            continue;
        }
        if(best >= candidate_count ||
           candidates[index].classification > candidates[best].classification ||
           (candidates[index].classification == candidates[best].classification &&
            candidates[index].score > candidates[best].score))
        {
            best = index;
        }
    }
    if(best >= candidate_count)
    {
        return;
    }
    group[0] = best;
    ignored_geometry = candidates[best].component;
    (void)down_car_assign_support(
        &ignored_geometry, candidates, group, 1U, 0, 0, 0U);
}

static unsigned char down_car_build_low_track_support(
    component_t *lamp,
    int min_x,
    int max_x,
    int min_y,
    int max_y,
    float axis_x,
    float axis_y)
{
    static const signed char neighbor_x[8] =
        {1, -1, 0, 0, 1, 1, -1, -1};
    static const signed char neighbor_y[8] =
        {0, 0, 1, -1, 1, -1, 1, -1};
    component_t component;
    unsigned short *head = g_queue;
    unsigned short *tail = g_queue;
    unsigned char stamp;
    float best_distance = 1.0e9f;
    int seed_x = 0;
    int seed_y = 0;
    int sum_x = 0;
    int sum_y = 0;
    int x;
    int y;

    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            float dx;
            float dy;
            float major;
            float minor;
            float distance;

            if(g_current_image[y][x] < DOWN_GRAY_SUPPORT_THRESHOLD)
            {
                continue;
            }
            dx = (float)x - lamp->cx;
            dy = (float)y - lamp->cy;
            major = dx * axis_x + dy * axis_y;
            minor = -dx * axis_y + dy * axis_x;
            if(fabsf(major) > lamp->major * 0.5f + DOWN_CAR_SUPPORT_PAD ||
               fabsf(minor) > lamp->minor * 0.5f + DOWN_CAR_SUPPORT_PAD)
            {
                continue;
            }
            distance = dx * dx + dy * dy;
            if(distance < best_distance)
            {
                best_distance = distance;
                seed_x = x;
                seed_y = y;
            }
        }
    }
    if(best_distance >= 1.0e9f)
    {
        return 0U;
    }

    memset(&component, 0, sizeof(component));
    component.min_x = seed_x;
    component.max_x = seed_x;
    component.min_y = seed_y;
    component.max_y = seed_y;
    component.seed_x = (unsigned char)seed_x;
    component.seed_y = (unsigned char)seed_y;
    begin_visit_pass();
    stamp = g_current_stamp;
    g_visit_stamp[(unsigned int)seed_y * BEACON_IMAGE_W + seed_x] = stamp;
    *tail++ = (unsigned short)(seed_x | (seed_y << 8));
    while(head < tail)
    {
        unsigned short coordinate = *head++;
        int center_x = (unsigned char)coordinate;
        int center_y = (unsigned char)(coordinate >> 8U);
        unsigned int gray = g_current_image[center_y][center_x];
        unsigned char index;

        component.area++;
        component.gray_sum += gray;
        component.gray_sum_sq += gray * gray;
        if(gray > component.peak) component.peak = (unsigned char)gray;
        sum_x += center_x;
        sum_y += center_y;
        if(center_x < component.min_x) component.min_x = center_x;
        if(center_x > component.max_x) component.max_x = center_x;
        if(center_y < component.min_y) component.min_y = center_y;
        if(center_y > component.max_y) component.max_y = center_y;
        for(index = 0U; index < 8U; index++)
        {
            int next_x = center_x + neighbor_x[index];
            int next_y = center_y + neighbor_y[index];
            unsigned int next_pixel;
            float dx;
            float dy;
            float major;
            float minor;

            if(next_x < min_x || next_x > max_x ||
               next_y < min_y || next_y > max_y)
            {
                continue;
            }
            next_pixel = (unsigned int)next_y * BEACON_IMAGE_W + next_x;
            if(g_visit_stamp[next_pixel] == stamp ||
               g_current_image[next_y][next_x] < DOWN_GRAY_SUPPORT_THRESHOLD)
            {
                continue;
            }
            dx = (float)next_x - lamp->cx;
            dy = (float)next_y - lamp->cy;
            major = dx * axis_x + dy * axis_y;
            minor = -dx * axis_y + dy * axis_x;
            if(fabsf(major) > lamp->major * 0.5f + DOWN_CAR_SUPPORT_PAD ||
               fabsf(minor) > lamp->minor * 0.5f + DOWN_CAR_SUPPORT_PAD)
            {
                continue;
            }
            g_visit_stamp[next_pixel] = stamp;
            *tail++ = (unsigned short)(next_x | (next_y << 8));
        }
    }
    if(component.area < CAR_LAMP_TEMPORAL_MIN_BRIGHT_AREA)
    {
        return 0U;
    }
    finish_component(
        &component, sum_x, sum_y, (unsigned short)component.area);
    component_resolve_angle(&component);
    if(component.elongation < DOWN_CAR_SCORE_ELONGATION_BASE ||
       down_car_axis_angle_difference(component.angle, lamp->angle) >
           DOWN_CAR_GROUP_ANGLE_MAX)
    {
        return 0U;
    }
    memset(g_car_support_mask, 0, sizeof(g_car_support_mask));
    head = g_queue;
    while(head < tail)
    {
        unsigned short coordinate = *head++;
        car_support_set(
            (unsigned int)(unsigned char)(coordinate >> 8U) * BEACON_IMAGE_W +
            (unsigned char)coordinate,
            1U);
    }
    lamp->seed_x = component.seed_x;
    lamp->seed_y = component.seed_y;
    return down_car_fit_support_geometry(
        lamp, min_x, max_x, min_y, max_y, DOWN_GRAY_SUPPORT_THRESHOLD);
}

static unsigned char down_car_build_single_support(
    component_t *lamp,
    unsigned char require_track_shape)
{
    down_car_candidate_t candidate;
    component_t core;
    unsigned char group[DOWN_CAR_GROUP_MAX_CORES] = {0U, 0U, 0U};
    float angle;
    float axis_x;
    float axis_y;
    float best_distance = 1.0e9f;
    unsigned char seed_x = 0U;
    unsigned char seed_y = 0U;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int x;
    int y;

    if(lamp == 0 || lamp->valid == 0U || g_current_image == 0)
    {
        return 0U;
    }
    angle = lamp->angle * (PI_F / 180.0f);
    axis_x = cosf(angle);
    axis_y = sinf(angle);
    min_x = lamp->min_x - DOWN_CAR_SUPPORT_PAD;
    max_x = lamp->max_x + DOWN_CAR_SUPPORT_PAD;
    min_y = lamp->min_y - DOWN_CAR_SUPPORT_PAD;
    max_y = lamp->max_y + DOWN_CAR_SUPPORT_PAD;
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            float dx;
            float dy;
            float major;
            float minor;
            float distance;

            if(g_binary[y][x] == 0U)
            {
                continue;
            }
            dx = (float)x - lamp->cx;
            dy = (float)y - lamp->cy;
            major = dx * axis_x + dy * axis_y;
            minor = -dx * axis_y + dy * axis_x;
            if(fabsf(major) > lamp->major * 0.5f + DOWN_CAR_SUPPORT_PAD ||
               fabsf(minor) > lamp->minor * 0.5f + DOWN_CAR_SUPPORT_PAD)
            {
                continue;
            }
            distance = dx * dx + dy * dy;
            if(distance < best_distance)
            {
                best_distance = distance;
                seed_x = (unsigned char)x;
                seed_y = (unsigned char)y;
            }
        }
    }
    if(best_distance >= 1.0e9f)
    {
        return (require_track_shape != 0U) ?
            down_car_build_low_track_support(
                lamp, min_x, max_x, min_y, max_y, axis_x, axis_y) : 0U;
    }
    begin_visit_pass();
    core = grow_component(seed_x, seed_y);
    if(core.valid == 0U)
    {
        return 0U;
    }
    memset(&candidate, 0, sizeof(candidate));
    candidate.core = core;
    component_resolve_angle(&candidate.core);
    if(require_track_shape != 0U)
    {
        if(core.area < CAR_LAMP_TEMPORAL_MIN_BRIGHT_AREA ||
           candidate.core.elongation < DOWN_CAR_SCORE_ELONGATION_BASE ||
           down_car_axis_angle_difference(
               candidate.core.angle, lamp->angle) > DOWN_CAR_GROUP_ANGLE_MAX)
        {
            return 0U;
        }
        candidate.component = *lamp;
        candidate.classification = DOWN_CAR_CLASS_TRACK;
    }
    else
    {
        candidate.component = *lamp;
        candidate.classification = DOWN_CAR_CLASS_CENTER_RECOVERED;
    }
    candidate.output_eligible = 1U;
    return down_car_assign_support(
        lamp, &candidate, group, 1U, 0, 0, 0U);
}

static unsigned char down_car_merge_temporal_support(component_t *lamp)
{
    unsigned char *saved_support = &g_beacon_binary_snapshot[0][0];
    unsigned char had_support = g_car_support_valid;
    unsigned int index;

    if(had_support != 0U)
    {
        memcpy(saved_support, g_car_support_mask,
               sizeof(g_car_support_mask));
    }
    if(down_car_build_single_support(lamp, 1U) != 0U)
    {
        if(had_support != 0U)
        {
            for(index = 0U; index < DOWN_CAR_SUPPORT_MASK_SIZE; index++)
            {
                g_car_support_mask[index] |= saved_support[index];
            }
        }
        g_car_support_valid = 1U;
        return 1U;
    }

    if(had_support != 0U)
    {
        memcpy(g_car_support_mask, saved_support,
               sizeof(g_car_support_mask));
        g_car_support_valid = 1U;
    }
    else
    {
        memset(g_car_support_mask, 0, sizeof(g_car_support_mask));
        g_car_support_valid = 0U;
    }
    return 0U;
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static unsigned char find_car_lamp(
    component_t *best_lamp,
    unsigned char *selected_classification)
{
    component_t cores[DOWN_CAR_MAX_COMPONENTS];
    unsigned char core_classifications[DOWN_CAR_MAX_COMPONENTS];
    down_car_candidate_t candidates[DOWN_CAR_MAX_MASKS];
    unsigned char core_count = 0U;
    unsigned char candidate_count = 0U;
    unsigned char x;
    unsigned char y;
    unsigned char index;
    unsigned char best;
    unsigned char weak_best;

    memset(best_lamp, 0, sizeof(*best_lamp));
    if(selected_classification != 0)
    {
        *selected_classification = DOWN_CAR_CLASS_NONE;
    }
    memset(cores, 0, sizeof(cores));
    memset(core_classifications, 0, sizeof(core_classifications));
    memset(candidates, 0, sizeof(candidates));
    begin_visit_pass();
    for (y = 0U; y < BEACON_IMAGE_H; y++)
    {
        x = 0U;
        while (x < BEACON_IMAGE_W)
        {
            component_t component;
#if defined(__ICCARM__)
            if ((x <= BEACON_IMAGE_W - 4U) &&
                (__UNALIGNED_UINT32_READ(&g_binary[y][x]) == 0U))
            {
                x = (unsigned char)(x + 4U);
                continue;
            }
#endif
            if (g_binary[y][x] == 0U || is_visited(x, y) != 0U)
            {
                x++;
                continue;
            }
            component = grow_component(x, y);
            if (component.valid != 0U)
            {
                store_car_core(cores, &core_count, &component);
            }
            x++;
        }
    }

    for (index = 0U; index < core_count; index++)
    {
        down_car_features_t features;

        memset(&features, 0, sizeof(features));
        component_resolve_angle(&cores[index]);
        if (car_component_features(&cores[index], &features) != 0U)
        {
            core_classifications[index] = features.classification;
            store_car_candidate(
                candidates, &candidate_count, &features, &cores[index],
                (features.envelope.area >=
                 g_image_down_car_lamp_min_area) ? 1U : 0U);
        }
    }

    best = select_car_candidate(candidates, candidate_count,
                                DOWN_CAR_CLASS_STRONG);
    if (best < candidate_count &&
        down_car_finalize_candidate(
            best_lamp, candidates, candidate_count, best,
            cores, core_classifications, core_count) != 0U)
    {
        reset_weak_car_pending();
        if(selected_classification != 0)
        {
            *selected_classification = DOWN_CAR_CLASS_STRONG;
        }
        return 1U;
    }

    weak_best = select_car_candidate(candidates, candidate_count,
                                     DOWN_CAR_CLASS_WEAK);
    if (weak_best < candidate_count &&
        weak_car_confirmed(&candidates[weak_best].component, 1U) != 0U &&
        down_car_finalize_candidate(
            best_lamp, candidates, candidate_count, weak_best,
            cores, core_classifications, core_count) != 0U)
    {
        if(selected_classification != 0)
        {
            *selected_classification = DOWN_CAR_CLASS_WEAK;
        }
        return 1U;
    }
    best = down_car_select_track_group_primary(
        candidates, candidate_count);
    if(best < candidate_count &&
       down_car_finalize_candidate(
           best_lamp, candidates, candidate_count, best,
           cores, core_classifications, core_count) != 0U &&
       best_lamp->area >= g_image_down_car_lamp_min_area)
    {
        reset_weak_car_pending();
        if(selected_classification != 0)
        {
            *selected_classification = candidates[best].classification;
        }
        return 1U;
    }
    best = select_car_candidate(candidates, candidate_count,
                                DOWN_CAR_CLASS_TRACK);
    if (best < candidate_count && g_car_track.confirmed != 0U)
    {
        float predicted_x = g_car_track.x + (float)BEACON_IMAGE_W * 0.5f +
                            g_car_track.vx;
        float predicted_y = g_car_track.y + (float)BEACON_IMAGE_H * 0.5f +
                            g_car_track.vy;
        float dx = candidates[best].component.cx - predicted_x;
        float dy = candidates[best].component.cy - predicted_y;
        if (dx * dx + dy * dy <=
                g_image_down_gate_distance * g_image_down_gate_distance &&
            down_car_finalize_candidate(
                best_lamp, candidates, candidate_count, best,
                cores, core_classifications, core_count) != 0U)
        {
            reset_weak_car_pending();
            if(selected_classification != 0)
            {
                *selected_classification = DOWN_CAR_CLASS_TRACK;
            }
            return 1U;
        }
    }
    if (weak_best >= candidate_count)
    {
        age_weak_car_pending();
    }
    down_car_mark_unreported_priority_core(candidates, candidate_count);
    return 0U;
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

static void write_car_lamp(const component_t *lamp,
                           unsigned char classification,
                           beacon_result_t *result)
{
    beacon_rect_t *rect;

    if (lamp->valid == 0 ||
        classification == DOWN_CAR_CLASS_NONE ||
        (classification == DOWN_CAR_CLASS_TRACK &&
         g_car_track.confirmed == 0U))
    {
        result->car_lamp_count = 0;
        result->car_lamp_measured_mask = 0U;
        result->car_lamp_classification = DOWN_CAR_CLASS_NONE;
        return;
    }

    rect = &result->car_lamps[0];
    fill_car_lamp_rect(lamp, rect);
    result->car_lamp_count = 1;
    result->car_lamp_measured_mask = 0x01U;
    result->car_lamp_classification = classification;
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
    if (image_down_horizon_contains(image_cx, image_cy,
                                    DOWN_OBJECT_BOUNDARY_MARGIN) == 0U)
    {
        return 0;
    }

    half_len = track->length * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;
    half_wid = track->width * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;
    radius = sqrtf(half_len * half_len + half_wid * half_wid);
    /* 预测框完全离开画面时立即失效，避免负边界进入无符号像素循环。 */
    if ((isfinite(image_cx) == 0) ||
        (isfinite(image_cy) == 0) ||
        (isfinite(radius) == 0) ||
        (isfinite(track->angle) == 0) ||
        (image_cx + radius < 0.0f) ||
        (image_cy + radius < 0.0f) ||
        (image_cx - radius >= (float)BEACON_IMAGE_W) ||
        (image_cy - radius >= (float)BEACON_IMAGE_H))
    {
        return 0;
    }

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

static unsigned char down_car_component_in_temporal_core(
    const component_t *component,
    const component_t *predicted_lamp)
{
    float angle;
    float cos_angle;
    float sin_angle;
    float dx;
    float dy;
    float major;
    float minor;
    float distance_limit;

    if(component == 0 || predicted_lamp == 0 ||
       component->valid == 0U || predicted_lamp->valid == 0U)
    {
        return 0U;
    }
    angle = predicted_lamp->angle * (PI_F / 180.0f);
    cos_angle = cosf(angle);
    sin_angle = sinf(angle);
    dx = component->cx - predicted_lamp->cx;
    dy = component->cy - predicted_lamp->cy;
    major = dx * cos_angle + dy * sin_angle;
    minor = -dx * sin_angle + dy * cos_angle;
    if(fabsf(major) <= predicted_lamp->major * 0.5f +
                         (float)CAR_LAMP_TEMPORAL_CORE_PAD &&
       fabsf(minor) <= predicted_lamp->minor * 0.5f +
                         (float)CAR_LAMP_TEMPORAL_CORE_PAD)
    {
        return 1U;
    }
    distance_limit =
        g_image_down_gate_distance * g_image_down_gate_distance;
    if(distance_limit < 1.0f) distance_limit = 1.0f;
    return (dx * dx + dy * dy <= distance_limit) ? 1U : 0U;
}

static void down_car_store_recovery_component(
    component_t candidates[DOWN_CAR_MAX_MASKS],
    unsigned char *count,
    const component_t *component)
{
    unsigned char index;
    unsigned char smallest = 0U;

    if(*count < DOWN_CAR_MAX_MASKS)
    {
        candidates[*count] = *component;
        (*count)++;
        return;
    }
    for(index = 1U; index < *count; index++)
    {
        if(candidates[index].area < candidates[smallest].area)
        {
            smallest = index;
        }
    }
    if(component->area > candidates[smallest].area)
    {
        candidates[smallest] = *component;
    }
}

static unsigned char down_car_center_prediction(
    component_t *predicted_lamp)
{
    if(predicted_lamp == 0 || g_car_track.confirmed == 0U ||
       component_from_temporal_car(
           &g_car_track, predicted_lamp, 1U) == 0U ||
       predicted_lamp->min_x <= CAR_LAMP_TEMPORAL_EDGE_MARGIN ||
       predicted_lamp->min_y <= CAR_LAMP_TEMPORAL_EDGE_MARGIN ||
       predicted_lamp->max_x >=
           BEACON_IMAGE_W - 1 - CAR_LAMP_TEMPORAL_EDGE_MARGIN ||
       predicted_lamp->max_y >=
           BEACON_IMAGE_H - 1 - CAR_LAMP_TEMPORAL_EDGE_MARGIN)
    {
        return 0U;
    }
    return 1U;
}

static unsigned char down_car_center_track_mismatch(
    const component_t *lamp)
{
    component_t predicted_lamp;
    float dx;
    float dy;
    float distance_limit;

    if(lamp == 0 || lamp->valid == 0U ||
       (lamp->min_x > CAR_LAMP_TEMPORAL_EDGE_MARGIN &&
        lamp->min_y > CAR_LAMP_TEMPORAL_EDGE_MARGIN &&
        lamp->max_x <
            BEACON_IMAGE_W - 1 - CAR_LAMP_TEMPORAL_EDGE_MARGIN &&
        lamp->max_y <
            BEACON_IMAGE_H - 1 - CAR_LAMP_TEMPORAL_EDGE_MARGIN) ||
       down_car_center_prediction(&predicted_lamp) == 0U)
    {
        return 0U;
    }
    dx = lamp->cx - predicted_lamp.cx;
    dy = lamp->cy - predicted_lamp.cy;
    distance_limit =
        g_image_down_gate_distance * g_image_down_gate_distance;
    if(distance_limit < 1.0f) distance_limit = 1.0f;
    return (dx * dx + dy * dy > distance_limit) ? 1U : 0U;
}

static unsigned char down_car_try_center_recovery_bridge(
    const component_t *identity,
    int min_x,
    int max_x,
    int min_y,
    int max_y)
{
    component_t cores[DOWN_CAR_MAX_COMPONENTS];
    unsigned char core_classifications[DOWN_CAR_MAX_COMPONENTS];
    down_car_candidate_t primary[DOWN_CAR_MAX_MASKS];
    unsigned char group[DOWN_CAR_GROUP_MAX_CORES] = {0U, 0U, 0U};
    unsigned char core_count = 0U;
    unsigned char primary_index = DOWN_CAR_MAX_COMPONENTS;
    float best_distance_sq = 1.0e9f;
    int x;
    int y;
    unsigned char index;

    if(identity == 0 || identity->valid == 0U ||
       g_car_track.confirmed == 0U)
    {
        return 0U;
    }
    memset(cores, 0, sizeof(cores));
    memset(core_classifications, 0, sizeof(core_classifications));
    memset(primary, 0, sizeof(primary));
    begin_visit_pass();
    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            component_t component;

            if(g_binary[y][x] == 0U ||
               is_visited((unsigned char)x, (unsigned char)y) != 0U)
            {
                continue;
            }
            component = grow_component((unsigned char)x, (unsigned char)y);
            if(component.valid != 0U)
            {
                store_car_core(cores, &core_count, &component);
            }
        }
    }
    for(index = 0U; index < core_count; index++)
    {
        down_car_features_t features;
        float dx;
        float dy;
        float distance_sq;

        component_resolve_angle(&cores[index]);
        memset(&features, 0, sizeof(features));
        (void)car_component_features(&cores[index], &features);
        core_classifications[index] = features.classification;
        if(down_car_component_in_temporal_core(
               &cores[index], identity) == 0U)
        {
            continue;
        }
        dx = cores[index].cx - identity->cx;
        dy = cores[index].cy - identity->cy;
        distance_sq = dx * dx + dy * dy;
        if(distance_sq < best_distance_sq)
        {
            best_distance_sq = distance_sq;
            primary_index = index;
        }
    }
    if(primary_index >= core_count)
    {
        return 0U;
    }
    primary[0].component = *identity;
    primary[0].core = cores[primary_index];
    primary[0].classification = DOWN_CAR_CLASS_CENTER_RECOVERED;
    primary[0].output_eligible = 1U;
    if(down_car_try_bridge_output(
           identity, primary, group, 1U,
           cores, core_classifications, core_count, 0, 0U) != 0U)
    {
        return 1U;
    }
    down_car_clear_support();
    return 0U;
}

/* 已确认车灯在画面中心漏检时，以轨迹先验重新接管当前真实亮域。 */
static unsigned char down_car_recover_center_track(
    component_t *best_lamp,
    unsigned char *selected_classification)
{
    component_t predicted_lamp;
    down_lamp_geometry_t recovery_geometry;
    component_t candidates[DOWN_CAR_MAX_MASKS];
    float candidate_morphology_scores[DOWN_CAR_MAX_MASKS];
    unsigned char candidate_classifications[DOWN_CAR_MAX_MASKS];
    unsigned char candidate_count = 0U;
    unsigned char best_classification = DOWN_CAR_CLASS_NONE;
    unsigned char best = 255U;
    float best_score = -1.0f;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int x;
    int y;
    unsigned char index;

    if(best_lamp == 0 || selected_classification == 0 ||
       g_current_image == 0 ||
       down_car_center_prediction(&predicted_lamp) == 0U)
    {
        return 0U;
    }
    down_gray_lamp_geometry_init(&predicted_lamp, &recovery_geometry);

    min_x = predicted_lamp.min_x - CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    max_x = predicted_lamp.max_x + CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    min_y = predicted_lamp.min_y - CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    max_y = predicted_lamp.max_y + CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    memset(candidates, 0, sizeof(candidates));
    memset(candidate_morphology_scores, 0,
           sizeof(candidate_morphology_scores));
    memset(candidate_classifications, 0,
           sizeof(candidate_classifications));
    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            g_binary[y][x] =
                (g_current_image[y][x] >= DOWN_GRAY_SUPPORT_THRESHOLD &&
                 down_gray_point_in_lamp_geometry(
                     x, y, &recovery_geometry) != 0U) ? 1U : 0U;
        }
    }
    begin_visit_pass();
    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            component_t component;

            if(g_binary[y][x] == 0U ||
               is_visited((unsigned char)x, (unsigned char)y) != 0U)
            {
                continue;
            }
            component = grow_component((unsigned char)x, (unsigned char)y);
            if(component.valid != 0U &&
               component.area >= CAR_LAMP_TEMPORAL_MIN_BRIGHT_AREA &&
               down_car_component_in_temporal_core(
                   &component, &predicted_lamp) != 0U)
            {
                down_car_store_recovery_component(
                    candidates, &candidate_count, &component);
            }
        }
    }
    for(y = min_y; y <= max_y; y++)
    {
        memcpy(&g_binary[y][min_x],
               &g_car_lamp_binary_snapshot[y][min_x],
               (size_t)(max_x - min_x + 1));
    }

    for(index = 0U; index < candidate_count; index++)
    {
        down_car_features_t features;

        if(car_component_features(&candidates[index], &features) == 0U)
        {
            candidates[index].valid = 0U;
            continue;
        }
        candidates[index] = features.envelope;
        candidate_classifications[index] = features.classification;
        candidate_morphology_scores[index] = features.score;
        if(features.envelope.area < g_image_down_car_lamp_min_area)
        {
            candidates[index].valid = 0U;
            continue;
        }
        if(features.classification >= DOWN_CAR_CLASS_WEAK)
        {
            if(features.classification > best_classification)
            {
                best_classification = features.classification;
            }
        }
    }
    for(index = 0U; index < candidate_count; index++)
    {
        float dx = candidates[index].cx - predicted_lamp.cx;
        float dy = candidates[index].cy - predicted_lamp.cy;
        float distance_limit =
            g_image_down_gate_distance * g_image_down_gate_distance;
        float proximity;
        float major_ratio = candidates[index].major / predicted_lamp.major;
        float minor_ratio = candidates[index].minor / predicted_lamp.minor;
        float shape_score;
        float morphology_score = candidate_morphology_scores[index];
        float score;

        if(candidates[index].valid == 0U)
        {
            continue;
        }
        if(best_classification >= DOWN_CAR_CLASS_WEAK &&
           candidate_classifications[index] < best_classification)
        {
            continue;
        }
        if(distance_limit < 1.0f) distance_limit = 1.0f;
        proximity = 1.0f - (dx * dx + dy * dy) / distance_limit;
        if(proximity < 0.0f) proximity = 0.0f;
        if(major_ratio > 1.0f) major_ratio = 1.0f / major_ratio;
        if(minor_ratio > 1.0f) minor_ratio = 1.0f / minor_ratio;
        shape_score = (major_ratio + minor_ratio) * 0.5f;
        score = proximity * 0.50f + shape_score * 0.25f +
                morphology_score * 0.25f;
        if(best >= candidate_count || score > best_score)
        {
            best = index;
            best_score = score;
        }
    }
    if(best >= candidate_count)
    {
        return 0U;
    }

    *best_lamp = candidates[best];
    component_resolve_angle(best_lamp);
    *selected_classification = DOWN_CAR_CLASS_CENTER_RECOVERED;
    (void)down_car_try_center_recovery_bridge(
        best_lamp, min_x, max_x, min_y, max_y);
    reset_weak_car_pending();
    return 1U;
}

static int down_gray_reflect_index(int index, int limit)
{
    if (index < 0)
    {
        return -index;
    }
    if (index >= limit)
    {
        return limit * 2 - 2 - index;
    }
    return index;
}

static unsigned char down_gray_in_bounds(int x, int y)
{
    return (x >= 0 && x < BEACON_IMAGE_W &&
            y >= 0 && y < BEACON_IMAGE_H) ? 1U : 0U;
}

static void down_gray_lamp_geometry_init(
    const component_t *lamp,
    down_lamp_geometry_t *geometry)
{
    float angle;

    memset(geometry, 0, sizeof(*geometry));
    if (lamp == 0 || lamp->valid == 0U)
    {
        return;
    }
    angle = lamp->angle * (PI_F / 180.0f);
    geometry->cx = lamp->cx;
    geometry->cy = lamp->cy;
    geometry->cos_angle = cosf(angle);
    geometry->sin_angle = sinf(angle);
    geometry->half_length = lamp->major * 0.5f + (float)LAMP_MASK_PAD;
    geometry->half_width = lamp->minor * 0.5f + (float)LAMP_MASK_PAD;
    geometry->min_x = lamp->min_x - LAMP_MASK_PAD;
    geometry->max_x = lamp->max_x + LAMP_MASK_PAD;
    geometry->min_y = lamp->min_y - LAMP_MASK_PAD;
    geometry->max_y = lamp->max_y + LAMP_MASK_PAD;
    geometry->valid = 1U;
}

static unsigned char down_gray_point_in_lamp_geometry(
    int x,
    int y,
    const down_lamp_geometry_t *geometry)
{
    float dx;
    float dy;
    float major;
    float minor;

    if (geometry == 0 || geometry->valid == 0U)
    {
        return 0U;
    }
    if (x < geometry->min_x || x > geometry->max_x ||
        y < geometry->min_y || y > geometry->max_y)
    {
        return 0U;
    }
    dx = (float)x - geometry->cx;
    dy = (float)y - geometry->cy;
    major = dx * geometry->cos_angle + dy * geometry->sin_angle;
    minor = -dx * geometry->sin_angle + dy * geometry->cos_angle;
    return (fabsf(major) <= geometry->half_length &&
            fabsf(minor) <= geometry->half_width) ? 1U : 0U;
}

static unsigned char down_gray_point_in_current_lamps(int x, int y)
{
    if((x < 0) || (x >= BEACON_IMAGE_W) ||
       (y < 0) || (y >= BEACON_IMAGE_H))
    {
        return 0U;
    }
    return (g_car_support_valid != 0U) ?
        car_support_get((unsigned int)y * BEACON_IMAGE_W + x) : 0U;
}

static void down_car_clear_supported_beacon_tracks(void)
{
    unsigned char index;
    int x;
    int y;

    if(g_car_support_valid == 0U || g_car_track.confirmed == 0U)
    {
        return;
    }
    if(g_b0_track.active != 0U)
    {
        x = (int)(g_b0_track.x + (float)BEACON_IMAGE_W * 0.5f + 0.5f);
        y = (int)(g_b0_track.y + (float)BEACON_IMAGE_H * 0.5f + 0.5f);
        if(down_gray_point_in_current_lamps(x, y) != 0U)
        {
            memset(&g_b0_track, 0, sizeof(g_b0_track));
        }
    }
    for(index = 0U; index < DOWN_NEAR_CAR_BEACON_TRACKS; index++)
    {
        down_near_car_beacon_track_t *track =
            &g_near_car_beacon_tracks[index];

        if(track->active == 0U)
        {
            continue;
        }
        x = (int)(track->x + (float)BEACON_IMAGE_W * 0.5f + 0.5f);
        y = (int)(track->y + (float)BEACON_IMAGE_H * 0.5f + 0.5f);
        if(down_gray_point_in_current_lamps(x, y) != 0U)
        {
            memset(track, 0, sizeof(*track));
        }
    }
}

static unsigned char down_gray_point_in_range(int x, int y)
{
    float top_y;
    float bottom_y;

    if (g_image_down_horizon_valid == 0U ||
        g_image_down_horizon_extrapolated != 0U)
    {
        return 1U;
    }
    if (image_down_horizon_contains((float)x, (float)y, 0.0f) == 0U)
    {
        return 0U;
    }
    if (image_down_horizon_get_column(
            (uint16)x, &top_y, &bottom_y) == 0U)
    {
        return 1U;
    }
    if (top_y >= 0.0f && top_y <= (float)(BEACON_IMAGE_H - 1) &&
        (float)y < top_y + DOWN_BEACON_BOUNDARY_CLEARANCE)
    {
        return 0U;
    }
    if (bottom_y >= 0.0f && bottom_y <= (float)(BEACON_IMAGE_H - 1) &&
        (float)y > bottom_y - DOWN_BEACON_BOUNDARY_CLEARANCE)
    {
        return 0U;
    }
    return 1U;
}

/* 刷新列滑动和的镜像边界槽。输入和输出均为当前行的Box列和缓存。 */
static void down_gray_vertical_box_sum_reflect_edges(void)
{
    unsigned short *box3_sum = &g_gray_box3_storage[2];
    unsigned short *box9_sum = &g_gray_box9_storage[4];
    int offset;

    box3_sum[-1] = box3_sum[1];
    box3_sum[BEACON_IMAGE_W] = box3_sum[BEACON_IMAGE_W - 2];
    for (offset = 1; offset <= 4; offset++)
    {
        box9_sum[-offset] = box9_sum[offset];
        box9_sum[BEACON_IMAGE_W - 1 + offset] =
            box9_sum[BEACON_IMAGE_W - 1 - offset];
    }
}

/* 初始化指定中心行的Box3和Box9列方向滑动和。 */
static void down_gray_vertical_box_sum_init_at(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_y)
{
    unsigned short *box3_sum = &g_gray_box3_storage[2];
    unsigned short *box9_sum = &g_gray_box9_storage[4];
    int x;

#if defined(__ICCARM__)
    if (center_y >= 4 && center_y <= BEACON_IMAGE_H - 5)
    {
        const unsigned char *row_m4 = &image[center_y - 4][0];

        for (x = 0; x < BEACON_IMAGE_W; x += 4)
        {
            uint32 word = __UNALIGNED_UINT32_READ(&row_m4[x]);
            uint32 box9_even = __UXTB16(word);
            uint32 box9_odd = __UXTB16(__ROR(word, 8U));
            uint32 box3_even;
            uint32 box3_odd;
            uint32 even;
            uint32 odd;

            word = __UNALIGNED_UINT32_READ(&row_m4[BEACON_IMAGE_W + x]);
            box9_even = __UADD16(box9_even, __UXTB16(word));
            box9_odd = __UADD16(box9_odd, __UXTB16(__ROR(word, 8U)));
            word = __UNALIGNED_UINT32_READ(&row_m4[2 * BEACON_IMAGE_W + x]);
            box9_even = __UADD16(box9_even, __UXTB16(word));
            box9_odd = __UADD16(box9_odd, __UXTB16(__ROR(word, 8U)));
            word = __UNALIGNED_UINT32_READ(&row_m4[3 * BEACON_IMAGE_W + x]);
            box3_even = __UXTB16(word);
            box3_odd = __UXTB16(__ROR(word, 8U));
            box9_even = __UADD16(box9_even, box3_even);
            box9_odd = __UADD16(box9_odd, box3_odd);
            word = __UNALIGNED_UINT32_READ(&row_m4[4 * BEACON_IMAGE_W + x]);
            even = __UXTB16(word);
            odd = __UXTB16(__ROR(word, 8U));
            box3_even = __UADD16(box3_even, even);
            box3_odd = __UADD16(box3_odd, odd);
            box9_even = __UADD16(box9_even, even);
            box9_odd = __UADD16(box9_odd, odd);
            word = __UNALIGNED_UINT32_READ(&row_m4[5 * BEACON_IMAGE_W + x]);
            even = __UXTB16(word);
            odd = __UXTB16(__ROR(word, 8U));
            box3_even = __UADD16(box3_even, even);
            box3_odd = __UADD16(box3_odd, odd);
            box9_even = __UADD16(box9_even, even);
            box9_odd = __UADD16(box9_odd, odd);
            __UNALIGNED_UINT32_WRITE(
                &box3_sum[x], __PKHBT(box3_even, box3_odd, 16));
            __UNALIGNED_UINT32_WRITE(
                &box3_sum[x + 2], __PKHTB(box3_odd, box3_even, 16));
            word = __UNALIGNED_UINT32_READ(&row_m4[6 * BEACON_IMAGE_W + x]);
            box9_even = __UADD16(box9_even, __UXTB16(word));
            box9_odd = __UADD16(box9_odd, __UXTB16(__ROR(word, 8U)));
            word = __UNALIGNED_UINT32_READ(&row_m4[7 * BEACON_IMAGE_W + x]);
            box9_even = __UADD16(box9_even, __UXTB16(word));
            box9_odd = __UADD16(box9_odd, __UXTB16(__ROR(word, 8U)));
            word = __UNALIGNED_UINT32_READ(&row_m4[8 * BEACON_IMAGE_W + x]);
            box9_even = __UADD16(box9_even, __UXTB16(word));
            box9_odd = __UADD16(box9_odd, __UXTB16(__ROR(word, 8U)));
            __UNALIGNED_UINT32_WRITE(
                &box9_sum[x], __PKHBT(box9_even, box9_odd, 16));
            __UNALIGNED_UINT32_WRITE(
                &box9_sum[x + 2], __PKHTB(box9_odd, box9_even, 16));
        }
    }
    else
#endif
    {
        const unsigned char *row_m4 =
            image[down_gray_reflect_index(center_y - 4, BEACON_IMAGE_H)];
        const unsigned char *row_m3 =
            image[down_gray_reflect_index(center_y - 3, BEACON_IMAGE_H)];
        const unsigned char *row_m2 =
            image[down_gray_reflect_index(center_y - 2, BEACON_IMAGE_H)];
        const unsigned char *row_m1 =
            image[down_gray_reflect_index(center_y - 1, BEACON_IMAGE_H)];
        const unsigned char *row_0 = image[center_y];
        const unsigned char *row_p1 =
            image[down_gray_reflect_index(center_y + 1, BEACON_IMAGE_H)];
        const unsigned char *row_p2 =
            image[down_gray_reflect_index(center_y + 2, BEACON_IMAGE_H)];
        const unsigned char *row_p3 =
            image[down_gray_reflect_index(center_y + 3, BEACON_IMAGE_H)];
        const unsigned char *row_p4 =
            image[down_gray_reflect_index(center_y + 4, BEACON_IMAGE_H)];

        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            box3_sum[x] = (unsigned short)(
                (unsigned int)row_m1[x] + (unsigned int)row_0[x] +
                (unsigned int)row_p1[x]);
            box9_sum[x] = (unsigned short)(
                (unsigned int)row_m4[x] + (unsigned int)row_m3[x] +
                (unsigned int)row_m2[x] + (unsigned int)row_m1[x] +
                (unsigned int)row_0[x] + (unsigned int)row_p1[x] +
                (unsigned int)row_p2[x] + (unsigned int)row_p3[x] +
                (unsigned int)row_p4[x]);
        }
    }
    down_gray_vertical_box_sum_reflect_edges();
}

/* 将四列Box列和推进一行；M7路径使用双半字并行加减。 */
static void down_gray_vertical_box_sum_update4(
    unsigned short *sum,
    const unsigned char *sub_row,
    const unsigned char *add_row,
    int x)
{
#if defined(__ICCARM__)
    uint32 current01 = __UNALIGNED_UINT32_READ(&sum[x]);
    uint32 current23 = __UNALIGNED_UINT32_READ(&sum[x + 2]);
    uint32 sub_word = __UNALIGNED_UINT32_READ(&sub_row[x]);
    uint32 add_word = __UNALIGNED_UINT32_READ(&add_row[x]);
    uint32 even = __PKHBT(current01, current23, 16);
    uint32 odd = __PKHTB(current23, current01, 16);

    even = __USUB16(even, __UXTB16(sub_word));
    even = __UADD16(even, __UXTB16(add_word));
    odd = __USUB16(odd, __UXTB16(__ROR(sub_word, 8U)));
    odd = __UADD16(odd, __UXTB16(__ROR(add_word, 8U)));
    current01 = __PKHBT(even, odd, 16);
    current23 = __PKHTB(odd, even, 16);
    __UNALIGNED_UINT32_WRITE(&sum[x], current01);
    __UNALIGNED_UINT32_WRITE(&sum[x + 2], current23);
#else
    int offset;
    for (offset = 0; offset < 4; offset++)
    {
        sum[x + offset] = (unsigned short)(
            sum[x + offset] - sub_row[x + offset] + add_row[x + offset]);
    }
#endif
}

/* 缓存闭合边界内每列的整数行区间，并生成各行的候选扫描跨度。 */
static void down_gray_build_range_cache(
    unsigned char minimum_y[BEACON_IMAGE_W],
    unsigned char maximum_y[BEACON_IMAGE_W],
    unsigned char row_minimum_x[BEACON_IMAGE_H],
    unsigned char row_maximum_x[BEACON_IMAGE_H],
    unsigned char row_valid[BEACON_IMAGE_H])
{
    int x;
    int y;

    memset(row_minimum_x, BEACON_IMAGE_W, BEACON_IMAGE_H);
    memset(row_maximum_x, 0, BEACON_IMAGE_H);
    memset(row_valid, 0, BEACON_IMAGE_H);
    for (x = 0; x < BEACON_IMAGE_W; x++)
    {
        int min_y = 0;
        int max_y = BEACON_IMAGE_H - 1;

        if (g_image_down_horizon_valid != 0U &&
            g_image_down_horizon_extrapolated == 0U)
        {
            float top_y;
            float bottom_y;
            if (image_down_horizon_get_column(
                    (uint16)x, &top_y, &bottom_y) != 0U)
            {
                float lower = top_y;
                float upper = bottom_y;
                if (top_y >= 0.0f && top_y <= (float)(BEACON_IMAGE_H - 1))
                {
                    lower = top_y + DOWN_BEACON_BOUNDARY_CLEARANCE;
                }
                if (bottom_y >= 0.0f &&
                    bottom_y <= (float)(BEACON_IMAGE_H - 1))
                {
                    upper = bottom_y - DOWN_BEACON_BOUNDARY_CLEARANCE;
                }
                min_y = (int)lower;
                if ((float)min_y < lower)
                {
                    min_y++;
                }
                max_y = (int)upper;
                if ((float)max_y > upper)
                {
                    max_y--;
                }
                if (min_y < 0) min_y = 0;
                if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
            }
            else if (image_down_horizon_contains((float)x, 0.0f, 0.0f) == 0U)
            {
                min_y = BEACON_IMAGE_H;
                max_y = -1;
            }
        }
        minimum_y[x] = (unsigned char)min_y;
        maximum_y[x] = (max_y >= 0) ? (unsigned char)max_y : 0U;
        if (min_y > max_y)
        {
            continue;
        }
        for (y = min_y; y <= max_y; y++)
        {
            if (row_valid[y] == 0U)
            {
                row_minimum_x[y] = (unsigned char)x;
                row_valid[y] = 1U;
            }
            row_maximum_x[y] = (unsigned char)x;
        }
    }
}

static void down_gray_insert_peak(
    down_gray_peak_t peaks[DOWN_GRAY_MAX_PEAKS],
    unsigned char *count,
    signed short response,
    int x,
    int y)
{
    int position;
    int last;

    if (count == 0 || response <= 0)
    {
        return;
    }
    last = *count;
    if (last >= (int)DOWN_GRAY_MAX_PEAKS)
    {
        last = (int)DOWN_GRAY_MAX_PEAKS - 1;
        if (response <= peaks[last].response)
        {
            return;
        }
    }
    else
    {
        (*count)++;
    }
    position = last;
    while (position > 0 && response > peaks[position - 1].response)
    {
        peaks[position] = peaks[position - 1];
        position--;
    }
    peaks[position].response = response;
    peaks[position].x = (unsigned char)x;
    peaks[position].y = (unsigned char)y;
}

static void down_gray_collect_response_row(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const down_lamp_geometry_t *lamp_geometry,
    const down_lamp_geometry_t *temporal_lamp_geometry,
    float scene_mean,
    int center_y,
    int top_slot,
    int center_slot,
    int bottom_slot,
    const unsigned char minimum_y[BEACON_IMAGE_W],
    const unsigned char maximum_y[BEACON_IMAGE_W],
    const unsigned char row_minimum_x[BEACON_IMAGE_H],
    const unsigned char row_maximum_x[BEACON_IMAGE_H],
    const unsigned char row_valid[BEACON_IMAGE_H],
    down_gray_peak_t peaks[DOWN_GRAY_MAX_PEAKS],
    unsigned char *count)
{
    int minimum_peak = (int)(scene_mean + 25.0f);
    int x;

    if (minimum_peak < 90)
    {
        minimum_peak = 90;
    }
    if (row_valid[center_y] == 0U)
    {
        return;
    }
    for (x = row_minimum_x[center_y]; x <= row_maximum_x[center_y]; x++)
    {
        signed short response = g_gray_response_rows[center_slot][x];
        int dx;
        unsigned char local_maximum = 1U;

#if defined(__ICCARM__)
        if (((x & 3) == 0) && x + 3 <= row_maximum_x[center_y])
        {
            uint32 gray_word = __UNALIGNED_UINT32_READ(&image[center_y][x]);
            uint32 threshold_word = (uint32)minimum_peak * 0x01010101UL;
            (void)__USUB8(gray_word, threshold_word);
            if (__SEL(0xFFFFFFFFUL, 0U) == 0U)
            {
                x += 3;
                continue;
            }
        }
#endif

        if (response <= 0 ||
            center_y < minimum_y[x] || center_y > maximum_y[x])
        {
            continue;
        }
        if (image[center_y][x] < minimum_peak)
        {
            int local_peak = 0;
            int weak_peak = (int)(scene_mean + 70.0f);
            int dy;

            if (x < DOWN_BEACON_LEFT_EDGE_MARGIN ||
                x >= BEACON_IMAGE_W - DOWN_BEACON_RIGHT_EDGE_MARGIN)
            {
                continue;
            }

            if (weak_peak < 140)
            {
                weak_peak = 140;
            }
            for (dy = -1; dy <= 1; dy++)
            {
                int yy = down_gray_reflect_index(
                    center_y + dy, BEACON_IMAGE_H);
                for (dx = -1; dx <= 1; dx++)
                {
                    int xx = down_gray_reflect_index(x + dx, BEACON_IMAGE_W);
                    if (image[yy][xx] > local_peak)
                    {
                        local_peak = image[yy][xx];
                    }
                }
            }
            if (local_peak < weak_peak)
            {
                continue;
            }
        }
        if (response < g_gray_response_rows[center_slot]
                         [down_gray_reflect_index(x - 1, BEACON_IMAGE_W)] ||
            response < g_gray_response_rows[center_slot]
                         [down_gray_reflect_index(x + 1, BEACON_IMAGE_W)])
        {
            continue;
        }
        for (dx = -1; dx <= 1; dx++)
        {
            int xx = down_gray_reflect_index(x + dx, BEACON_IMAGE_W);
            if (response < g_gray_response_rows[top_slot][xx] ||
                response < g_gray_response_rows[bottom_slot][xx])
            {
                local_maximum = 0U;
                break;
            }
        }
        if (local_maximum != 0U)
        {
            if ((*count < DOWN_GRAY_MAX_PEAKS ||
                 response > peaks[DOWN_GRAY_MAX_PEAKS - 1U].response) &&
                down_gray_point_in_current_lamps(x, center_y) == 0U &&
                down_gray_point_in_lamp_geometry(
                    x, center_y, lamp_geometry) == 0U &&
                down_gray_point_in_lamp_geometry(
                    x, center_y, temporal_lamp_geometry) == 0U)
            {
                down_gray_insert_peak(peaks, count, response, x, center_y);
            }
        }
    }
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static unsigned char down_gray_find_peaks(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const down_lamp_geometry_t *lamp_geometry,
    const down_lamp_geometry_t *temporal_lamp_geometry,
    float scene_mean,
    down_gray_peak_t peaks[DOWN_GRAY_MAX_PEAKS])
{
    unsigned char count = 0U;
    unsigned char minimum_y[BEACON_IMAGE_W];
    unsigned char maximum_y[BEACON_IMAGE_W];
    unsigned char row_minimum_x[BEACON_IMAGE_H];
    unsigned char row_maximum_x[BEACON_IMAGE_H];
    unsigned char row_valid[BEACON_IMAGE_H];
    unsigned short *box3_vertical = &g_gray_box3_storage[2];
    unsigned short *box9_vertical = &g_gray_box9_storage[4];
    int x;
    int y;

    memset(peaks, 0, sizeof(down_gray_peak_t) * DOWN_GRAY_MAX_PEAKS);
    down_gray_build_range_cache(
        minimum_y, maximum_y,
        row_minimum_x, row_maximum_x, row_valid);
    down_gray_vertical_box_sum_init_at(image, 0);
    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        int response_slot = y % 3;
        int box_response =
            9 * ((int)box3_vertical[0] + 2 * (int)box3_vertical[1]) -
            ((int)box9_vertical[0] +
             2 * ((int)box9_vertical[1] + (int)box9_vertical[2] +
                  (int)box9_vertical[3] + (int)box9_vertical[4]));

        g_gray_response_rows[response_slot][0] = (signed short)box_response;
        for (x = 1; x < BEACON_IMAGE_W; x++)
        {
#if defined(__ICCARM__)
            uint32 entering = __PKHBT(
                (uint32)box3_vertical[x + 1],
                (uint32)box9_vertical[x + 4], 16);
            uint32 leaving = __PKHBT(
                (uint32)box3_vertical[x - 2],
                (uint32)box9_vertical[x - 5], 16);
            uint32 delta = __SSUB16(entering, leaving);
            box_response = (int)__SMLAD(
                delta, 0xFFFF0009UL, (uint32)box_response);
#else
            box_response +=
                9 * ((int)box3_vertical[x + 1] -
                     (int)box3_vertical[x - 2]) -
                ((int)box9_vertical[x + 4] -
                 (int)box9_vertical[x - 5]);
#endif
            g_gray_response_rows[response_slot][x] = (signed short)box_response;
        }
        if (y == 1)
        {
            down_gray_collect_response_row(
                image, lamp_geometry, temporal_lamp_geometry, scene_mean,
                0, 1, 0, 1,
                minimum_y, maximum_y,
                row_minimum_x, row_maximum_x, row_valid,
                peaks, &count);
        }
        else if (y >= 2)
        {
            down_gray_collect_response_row(
                image, lamp_geometry, temporal_lamp_geometry, scene_mean,
                y - 1, (y - 2) % 3, (y - 1) % 3, y % 3,
                minimum_y, maximum_y,
                row_minimum_x, row_maximum_x, row_valid,
                peaks, &count);
        }
        if (y == BEACON_IMAGE_H - 1)
        {
            break;
        }
        {
            const unsigned char *box3_sub_row =
                image[down_gray_reflect_index(y - 1, BEACON_IMAGE_H)];
            const unsigned char *box3_add_row =
                image[down_gray_reflect_index(y + 2, BEACON_IMAGE_H)];
            const unsigned char *box9_sub_row =
                image[down_gray_reflect_index(y - 4, BEACON_IMAGE_H)];
            const unsigned char *box9_add_row =
                image[down_gray_reflect_index(y + 5, BEACON_IMAGE_H)];
            for (x = 0; x < BEACON_IMAGE_W; x += 4)
            {
                down_gray_vertical_box_sum_update4(
                    box3_vertical, box3_sub_row, box3_add_row, x);
                down_gray_vertical_box_sum_update4(
                    box9_vertical, box9_sub_row, box9_add_row, x);
            }
        }
        down_gray_vertical_box_sum_reflect_edges();
    }
    down_gray_collect_response_row(
        image, lamp_geometry, temporal_lamp_geometry, scene_mean,
        BEACON_IMAGE_H - 1,
        (BEACON_IMAGE_H - 2) % 3,
        (BEACON_IMAGE_H - 1) % 3,
        (BEACON_IMAGE_H - 2) % 3,
        minimum_y, maximum_y,
        row_minimum_x, row_maximum_x, row_valid,
        peaks, &count);
    return count;
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static unsigned char down_local_shape_measure(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_x,
    int center_y,
    int radius,
    int threshold,
    int maximum_area,
    unsigned char reject_artificial_boundary,
    down_local_shape_t *shape)
{
    static const signed char neighbor_x[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const signed char neighbor_y[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    unsigned char visited[DOWN_LOCAL_SHAPE_SIZE][DOWN_LOCAL_SHAPE_SIZE];
    unsigned char queue_x[DOWN_LOCAL_SHAPE_CAPACITY];
    unsigned char queue_y[DOWN_LOCAL_SHAPE_CAPACITY];
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int seed_x = center_x;
    int seed_y = center_y;
    int best = -1;
    int head = 0;
    int tail = 0;
    int sum_x = 0;
    int sum_y = 0;
    float sum_xx = 0.0f;
    float sum_yy = 0.0f;
    float sum_xy = 0.0f;
    int dx;
    int dy;

    if (image == 0 || shape == 0 || radius < 1 ||
        radius > DOWN_EDGE_SUPPORT_RADIUS ||
        center_x < 0 || center_x >= BEACON_IMAGE_W ||
        center_y < 0 || center_y >= BEACON_IMAGE_H)
    {
        return 0U;
    }
    memset(shape, 0, sizeof(*shape));
    min_x = center_x - radius;
    max_x = center_x + radius;
    min_y = center_y - radius;
    max_y = center_y + radius;
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    for (dy = -2; dy <= 2; dy++)
    {
        int y = center_y + dy;
        if (y < min_y || y > max_y)
        {
            continue;
        }
        for (dx = -2; dx <= 2; dx++)
        {
            int x = center_x + dx;
            int value;
            if (x < min_x || x > max_x)
            {
                continue;
            }
            value = image[y][x];
            if (value > best)
            {
                best = value;
                seed_x = x;
                seed_y = y;
            }
        }
    }
    if (best < threshold)
    {
        return 0U;
    }
    memset(visited, 0, sizeof(visited));
    visited[seed_y - min_y][seed_x - min_x] = 1U;
    queue_x[tail] = (unsigned char)seed_x;
    queue_y[tail] = (unsigned char)seed_y;
    tail++;
    while (head < tail)
    {
        int index;
        int x = queue_x[head];
        int y = queue_y[head];
        float relative_x = (float)(x - center_x);
        float relative_y = (float)(y - center_y);
        head++;
        shape->area++;
        sum_x += x - center_x;
        sum_y += y - center_y;
        sum_xx += relative_x * relative_x;
        sum_yy += relative_y * relative_y;
        sum_xy += relative_x * relative_y;
        if ((x == min_x && min_x > 0) ||
            (x == max_x && max_x < BEACON_IMAGE_W - 1) ||
            (y == min_y && min_y > 0) ||
            (y == max_y && max_y < BEACON_IMAGE_H - 1))
        {
            shape->touches_artificial_boundary = 1U;
            if (reject_artificial_boundary != 0U)
            {
                return 0U;
            }
        }
        if (maximum_area > 0 && shape->area > maximum_area)
        {
            return 0U;
        }
        for (index = 0; index < 8; index++)
        {
            int next_x = x + neighbor_x[index];
            int next_y = y + neighbor_y[index];
            int visit_x;
            int visit_y;
            if (next_x < min_x || next_x > max_x ||
                next_y < min_y || next_y > max_y)
            {
                continue;
            }
            visit_x = next_x - min_x;
            visit_y = next_y - min_y;
            if (visited[visit_y][visit_x] != 0U ||
                image[next_y][next_x] < threshold ||
                tail >= DOWN_LOCAL_SHAPE_CAPACITY)
            {
                continue;
            }
            visited[visit_y][visit_x] = 1U;
            queue_x[tail] = (unsigned char)next_x;
            queue_y[tail] = (unsigned char)next_y;
            tail++;
        }
    }
    if (shape->area > 0)
    {
        float inv_area = 1.0f / (float)shape->area;
        float centroid_x = (float)sum_x * inv_area;
        float centroid_y = (float)sum_y * inv_area;
        float covariance_xx = sum_xx * inv_area - centroid_x * centroid_x;
        float covariance_yy = sum_yy * inv_area - centroid_y * centroid_y;
        float covariance_xy = sum_xy * inv_area - centroid_x * centroid_y;
        float trace = covariance_xx + covariance_yy;
        float discriminant = (covariance_xx - covariance_yy) *
                             (covariance_xx - covariance_yy) +
                             4.0f * covariance_xy * covariance_xy;
        float eigen_major;
        float eigen_minor;
        if (discriminant < 0.0f)
        {
            discriminant = 0.0f;
        }
        discriminant = sqrtf(discriminant);
        eigen_major = (trace + discriminant) * 0.5f;
        eigen_minor = (trace - discriminant) * 0.5f;
        if (eigen_major < 0.001f) eigen_major = 0.001f;
        if (eigen_minor < 0.001f) eigen_minor = 0.001f;
        shape->major = 4.0f * sqrtf(eigen_major);
        shape->minor = 4.0f * sqrtf(eigen_minor);
        shape->elongation = sqrtf(eigen_major / eigen_minor);
        shape->valid = 1U;
    }
    return shape->valid;
}

static unsigned char down_gray_edge_support_valid(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_x,
    int center_y,
    float background,
    unsigned char peak,
    int left_margin,
    int right_margin)
{
    float delta;
    int threshold;
    down_local_shape_t support;

    if (center_x >= left_margin &&
        center_x < BEACON_IMAGE_W - right_margin)
    {
        return 1U;
    }
    delta = ((float)peak - background) * 0.10f;
    if (delta < 12.0f)
    {
        delta = 12.0f;
    }
    threshold = (int)(background + delta + 0.5f);
    if (threshold < 0) threshold = 0;
    if (threshold > 255) threshold = 255;
    memset(&support, 0, sizeof(support));
    return down_local_shape_measure(
        image, center_x, center_y,
        DOWN_EDGE_SUPPORT_RADIUS, threshold,
        DOWN_EDGE_SUPPORT_MAX_AREA, 1U, &support);
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static unsigned char down_gray_local_features(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_x,
    int center_y,
    down_gray_features_t *features)
{
    float inner_sum = 0.0f;
    float middle_sum = 0.0f;
    float weight_sum = 0.0f;
    float inner_weight = 0.0f;
    float weight_x = 0.0f;
    float weight_y = 0.0f;
    float weight_xx = 0.0f;
    float weight_yy = 0.0f;
    float weight_xy = 0.0f;
    float centroid_x;
    float centroid_y;
    float covariance_xx;
    float covariance_yy;
    float covariance_xy;
    float trace;
    float discriminant;
    float eigen_major;
    float eigen_minor;
    float background;
    unsigned short background_histogram[256];
    unsigned short background_count = 0U;
    unsigned short background_cumulative = 0U;
    int inner_count = 0;
    int middle_count = 0;
    int outer_count = 0;
    int outer_bright = 0;
    int min_dx;
    int max_dx;
    int min_dy;
    int max_dy;
    int dx;
    int dy;
    int value;
    unsigned char peak = 0U;
    unsigned char half_area = 0U;

    if (features == 0)
    {
        return 0U;
    }
    memset(features, 0, sizeof(*features));
    memset(background_histogram, 0, sizeof(background_histogram));
    min_dx = (center_x < 10) ? -center_x : -10;
    max_dx = (center_x + 10 >= BEACON_IMAGE_W) ?
                 BEACON_IMAGE_W - 1 - center_x : 10;
    min_dy = (center_y < 10) ? -center_y : -10;
    max_dy = (center_y + 10 >= BEACON_IMAGE_H) ?
                 BEACON_IMAGE_H - 1 - center_y : 10;
    for (dy = min_dy; dy <= max_dy; dy++)
    {
        for (dx = min_dx; dx <= max_dx; dx++)
        {
            int radius2 = dx * dx + dy * dy;
            int x = center_x + dx;
            int y = center_y + dy;
            unsigned char pixel = image[y][x];
            if (radius2 >= DOWN_GRAY_BACKGROUND_INNER_SQ &&
                radius2 <= DOWN_GRAY_BACKGROUND_OUTER_SQ)
            {
                background_histogram[pixel]++;
                background_count++;
                outer_count++;
            }
            if (radius2 > DOWN_GRAY_PATCH_RADIUS * DOWN_GRAY_PATCH_RADIUS)
            {
                continue;
            }
            if (pixel > peak)
            {
                peak = pixel;
            }
            if (radius2 <= 4)
            {
                inner_sum += pixel;
                inner_count++;
            }
            else if (radius2 <= 16)
            {
                middle_sum += pixel;
                middle_count++;
            }
        }
    }
    background = 0.0f;
    if (background_count != 0U)
    {
        for (value = 0; value < 256; value++)
        {
            background_cumulative += background_histogram[value];
            if (background_cumulative * 2U >= background_count)
            {
                background = (float)value;
                break;
            }
        }
    }
    for (dy = min_dy; dy <= max_dy; dy++)
    {
        for (dx = min_dx; dx <= max_dx; dx++)
        {
            int radius2 = dx * dx + dy * dy;
            int x = center_x + dx;
            int y = center_y + dy;
            unsigned char pixel = image[y][x];
            if (radius2 >= DOWN_GRAY_BACKGROUND_INNER_SQ &&
                radius2 <= DOWN_GRAY_BACKGROUND_OUTER_SQ &&
                (float)pixel >= background + 20.0f)
            {
                outer_bright++;
            }
            if (radius2 > DOWN_GRAY_PATCH_RADIUS * DOWN_GRAY_PATCH_RADIUS)
            {
                continue;
            }
            {
                float weight = (float)pixel - background;
                if (weight < 0.0f)
                {
                    weight = 0.0f;
                }
                weight_sum += weight;
                weight_x += weight * (float)dx;
                weight_y += weight * (float)dy;
                weight_xx += weight * (float)(dx * dx);
                weight_yy += weight * (float)(dy * dy);
                weight_xy += weight * (float)(dx * dy);
                if (radius2 <= 4)
                {
                    inner_weight += weight;
                }
            }
            if ((unsigned int)pixel * 2U >= peak)
            {
                half_area++;
            }
        }
    }
    if (weight_sum <= 0.0001f || inner_count == 0 || middle_count == 0)
    {
        return 0U;
    }
    centroid_x = weight_x / weight_sum;
    centroid_y = weight_y / weight_sum;
    covariance_xx = weight_xx / weight_sum - centroid_x * centroid_x;
    covariance_yy = weight_yy / weight_sum - centroid_y * centroid_y;
    covariance_xy = weight_xy / weight_sum - centroid_x * centroid_y;
    trace = covariance_xx + covariance_yy;
    discriminant = (covariance_xx - covariance_yy) *
                   (covariance_xx - covariance_yy) +
                   4.0f * covariance_xy * covariance_xy;
    if (discriminant < 0.0f)
    {
        discriminant = 0.0f;
    }
    discriminant = sqrtf(discriminant);
    eigen_major = (trace + discriminant) * 0.5f;
    eigen_minor = (trace - discriminant) * 0.5f;
    if (eigen_minor < 0.001f)
    {
        eigen_minor = 0.001f;
    }
    features->background = background;
    features->inner_contrast = inner_sum / (float)inner_count - background;
    features->radial_drop = inner_sum / (float)inner_count -
                            middle_sum / (float)middle_count;
    features->concentration = inner_weight / weight_sum;
    features->elongation = sqrtf(eigen_major / eigen_minor);
    features->offset = sqrtf(centroid_x * centroid_x + centroid_y * centroid_y);
    features->outer_occupancy = (outer_count > 0) ?
        (float)outer_bright / (float)outer_count : 0.0f;
    features->centroid_x = centroid_x;
    features->centroid_y = centroid_y;
    features->peak = peak;
    features->half_area = half_area;
    return 1U;
}

static unsigned char down_gray_side_edge_beacon_like(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *component)
{
    down_gray_features_t features;
    int x;
    int y;

    if (image == 0 || component == 0 || component->valid == 0U ||
        (component->min_x > DOWN_CAR_COMPACT_EDGE_MARGIN &&
         component->max_x < BEACON_IMAGE_W - 1 -
                                DOWN_CAR_COMPACT_EDGE_MARGIN) ||
        component->major > 9.5f)
    {
        return 0U;
    }
    x = (int)(component->cx + 0.5f);
    y = (int)(component->cy + 0.5f);
    {
        int best_x = x;
        int best_y = y;
        int best = -1;
        int dx;
        int dy;

        for (dy = -3; dy <= 3; dy++)
        {
            for (dx = -3; dx <= 3; dx++)
            {
                int sample_x = x + dx;
                int sample_y = y + dy;
                if (down_gray_in_bounds(sample_x, sample_y) != 0U &&
                    image[sample_y][sample_x] > best)
                {
                    best = image[sample_y][sample_x];
                    best_x = sample_x;
                    best_y = sample_y;
                }
            }
        }
        x = best_x;
        y = best_y;
    }
    if (down_gray_local_features(image, x, y, &features) == 0U)
    {
        return 0U;
    }
    x = (int)((float)x + features.centroid_x + 0.5f);
    y = (int)((float)y + features.centroid_y + 0.5f);
    return ((features.peak >= 235U) &&
            (features.half_area >= 8U) &&
            (features.half_area <= 36U) &&
            (features.inner_contrast >= 150.0f) &&
            (features.radial_drop >= 55.0f) &&
            (features.concentration >= 0.28f) &&
            (features.elongation <= 1.95f) &&
            (features.outer_occupancy <= 0.25f) &&
            (down_gray_edge_support_valid(
                 image, x, y, features.background, features.peak,
                 DOWN_BEACON_LEFT_EDGE_MARGIN,
                 DOWN_BEACON_RIGHT_EDGE_MARGIN) != 0U)) ? 1U : 0U;
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static unsigned char down_gray_large_shape_valid(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_x,
    int center_y,
    const down_gray_features_t *features)
{
    enum { RADIUS = 10, SIZE = RADIUS * 2 + 1, CAPACITY = SIZE * SIZE };
    static const signed char neighbor_x[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const signed char neighbor_y[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    unsigned char visited[SIZE][SIZE];
    signed char queue_x[CAPACITY];
    signed char queue_y[CAPACITY];
    int head = 0;
    int tail = 0;
    int count = 0;
    int seed_x = 0;
    int seed_y = 0;
    int threshold;
    int best = -1;
    int dx;
    int dy;
    int sum_x = 0;
    int sum_y = 0;
    int sum_xx = 0;
    int sum_yy = 0;
    int sum_xy = 0;
    float cx;
    float cy;
    float var_x;
    float var_y;
    float cov_xy;
    float trace;
    float discriminant;
    float major;
    float minor;

    if (features == 0)
    {
        return 0U;
    }
    threshold = (int)(features->background +
                      ((float)features->peak - features->background) * 0.45f);
    if (threshold < 120)
    {
        threshold = 120;
    }
    for (dy = -2; dy <= 2; dy++)
    {
        for (dx = -2; dx <= 2; dx++)
        {
            int image_x = center_x + dx;
            int image_y = center_y + dy;
            int pixel;
            if (down_gray_in_bounds(image_x, image_y) == 0U)
            {
                continue;
            }
            pixel = image[image_y][image_x];
            if (pixel > best)
            {
                best = pixel;
                seed_x = dx;
                seed_y = dy;
            }
        }
    }
    if (best < threshold)
    {
        return 0U;
    }
    memset(visited, 0, sizeof(visited));
    visited[seed_y + RADIUS][seed_x + RADIUS] = 1U;
    queue_x[tail] = (signed char)seed_x;
    queue_y[tail] = (signed char)seed_y;
    tail++;
    while (head < tail)
    {
        int index;
        int x = queue_x[head];
        int y = queue_y[head];
        head++;
        if (x == -RADIUS || x == RADIUS || y == -RADIUS || y == RADIUS)
        {
            return 0U;
        }
        count++;
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_yy += y * y;
        sum_xy += x * y;
        for (index = 0; index < 8; index++)
        {
            int next_x = x + neighbor_x[index];
            int next_y = y + neighbor_y[index];
            int visit_x = next_x + RADIUS;
            int visit_y = next_y + RADIUS;
            int image_x = center_x + next_x;
            int image_y = center_y + next_y;
            if (visit_x < 0 || visit_x >= SIZE ||
                visit_y < 0 || visit_y >= SIZE ||
                visited[visit_y][visit_x] != 0U ||
                down_gray_in_bounds(image_x, image_y) == 0U ||
                image[image_y][image_x] < threshold)
            {
                continue;
            }
            visited[visit_y][visit_x] = 1U;
            queue_x[tail] = (signed char)next_x;
            queue_y[tail] = (signed char)next_y;
            tail++;
        }
    }
    if (count < 20 || count > 140)
    {
        return 0U;
    }
    cx = (float)sum_x / (float)count;
    cy = (float)sum_y / (float)count;
    var_x = (float)sum_xx / (float)count - cx * cx;
    var_y = (float)sum_yy / (float)count - cy * cy;
    cov_xy = (float)sum_xy / (float)count - cx * cy;
    trace = var_x + var_y;
    discriminant = (var_x - var_y) * (var_x - var_y) + 4.0f * cov_xy * cov_xy;
    discriminant = sqrtf((discriminant > 0.0f) ? discriminant : 0.0f);
    major = (trace + discriminant) * 0.5f;
    minor = (trace - discriminant) * 0.5f;
    if (minor < 0.001f)
    {
        minor = 0.001f;
    }
    return (sqrtf(major / minor) <= 2.0f) ? 1U : 0U;
}

static unsigned char down_gray_candidate_valid(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int x,
    int y,
    float scene_mean,
    const down_gray_features_t *features,
    unsigned char peak_clipped)
{
    int weak_peak = (int)(scene_mean + 70.0f);
    unsigned char weak;
    unsigned char normal;
    unsigned char medium;
    unsigned char large;
    unsigned char saturated_large;

    if (features == 0 || down_gray_point_in_range(x, y) == 0U)
    {
        return 0U;
    }
    if ((x < DOWN_BEACON_LEFT_EDGE_MARGIN ||
         x >= BEACON_IMAGE_W - DOWN_BEACON_RIGHT_EDGE_MARGIN) &&
        (features->peak < DOWN_GRAY_EDGE_MIN_PEAK ||
         (features->outer_occupancy > DOWN_GRAY_EDGE_MAX_OCCUPANCY &&
          (features->half_area > 20U ||
           features->concentration < 0.45f))))
    {
        return 0U;
    }
    if (weak_peak < 140)
    {
        weak_peak = 140;
    }
    weak = ((features->half_area >= 3U) &&
            (features->half_area <= 20U) &&
            (features->peak >= weak_peak) &&
            (features->inner_contrast >= 42.0f) &&
            (features->radial_drop >= 38.0f) &&
            (features->concentration >= 0.58f) &&
            (features->elongation <= 1.75f) &&
            (features->offset <= 1.70f) &&
            (features->outer_occupancy <= 0.42f)) ? 1U : 0U;
    normal = ((features->half_area >= 4U) &&
              (features->half_area <= 32U) &&
              (features->peak >= 200U) &&
              (features->inner_contrast >= 80.0f) &&
              (features->radial_drop >= 65.0f) &&
              (features->concentration >= 0.42f) &&
              (features->elongation <= 1.90f) &&
              (features->offset <= 2.00f) &&
              (features->outer_occupancy <= 0.50f)) ? 1U : 0U;
    medium = ((features->half_area >= 15U) &&
              (features->half_area <= 42U) &&
              (features->peak >= 235U) &&
              (features->inner_contrast >= 105.0f) &&
              (features->radial_drop >= 58.0f) &&
              (features->concentration >= 0.36f) &&
              (features->elongation <= 1.90f) &&
              (features->offset <= 2.10f) &&
              (features->outer_occupancy <= 0.56f)) ? 1U : 0U;
    large = ((features->half_area >= 24U) &&
             (features->half_area <= 84U) &&
             (features->peak >= 248U) &&
             (features->inner_contrast >= 135.0f) &&
             (features->radial_drop >= 38.0f) &&
              (features->concentration >= 0.18f) &&
             (features->elongation <= 2.00f) &&
             (features->offset <= 2.40f) &&
             (features->outer_occupancy <= 0.65f) &&
             (down_gray_large_shape_valid(image, x, y, features) != 0U)) ? 1U : 0U;
    saturated_large = ((features->half_area >= 60U) &&
                       (features->half_area <= 112U) &&
                       (features->peak >= 248U) &&
                       (features->inner_contrast >= 170.0f) &&
                       (features->concentration >= 0.12f) &&
                       (features->elongation <= 1.50f) &&
                       (features->offset <= 2.10f) &&
                       (features->outer_occupancy <= 0.48f)) ? 1U : 0U;
    if (peak_clipped != 0U && weak == 0U && large == 0U)
    {
        return 0U;
    }
    if (large == 0U &&
        down_gray_edge_support_valid(
            image, x, y, features->background, features->peak,
            DOWN_BEACON_LEFT_EDGE_MARGIN,
            DOWN_BEACON_RIGHT_EDGE_MARGIN) == 0U)
    {
        return 0U;
    }
    return (weak != 0U || normal != 0U || medium != 0U ||
            large != 0U || saturated_large != 0U) ? 1U : 0U;
}

static unsigned char down_gray_companion_edge_candidate_valid(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int x,
    int y,
    float scene_mean,
    const down_gray_features_t *features,
    const down_gray_candidate_t candidates[DOWN_GRAY_MAX_CANDIDATES],
    unsigned char candidate_count)
{
    int weak_peak = (int)(scene_mean + 70.0f);
    unsigned char index;

    if (features == 0 || candidate_count != 1U ||
        (x >= DOWN_BEACON_LEFT_EDGE_MARGIN &&
         x < BEACON_IMAGE_W - DOWN_BEACON_RIGHT_EDGE_MARGIN) ||
        features->peak < 170U || features->peak >= DOWN_GRAY_EDGE_MIN_PEAK)
    {
        return 0U;
    }
    if (weak_peak < 140)
    {
        weak_peak = 140;
    }
    if (features->half_area < 3U || features->half_area > 20U ||
        features->peak < weak_peak || features->inner_contrast < 42.0f ||
        features->radial_drop < 38.0f || features->concentration < 0.58f ||
        features->elongation > 1.75f || features->offset > 1.70f ||
        features->outer_occupancy > 0.42f ||
        down_gray_edge_support_valid(
            image, x, y, features->background, features->peak,
            DOWN_BEACON_LEFT_EDGE_MARGIN,
            DOWN_BEACON_RIGHT_EDGE_MARGIN) == 0U)
    {
        return 0U;
    }
    for (index = 0U; index < candidate_count; index++)
    {
        float dx = (float)x - candidates[index].x;
        float dy = (float)y - candidates[index].y;
        float distance_sq = dx * dx + dy * dy;
        if (distance_sq <= 144.0f || distance_sq > 1296.0f)
        {
            return 0U;
        }
    }
    return 1U;
}

static unsigned char down_gray_bind_candidate_source(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int response_x,
    int response_y,
    down_gray_candidate_t *candidate)
{
    int best_gray = -1;
    int best_distance_sq = 3;
    int dx;
    int dy;

    if(image == 0 || candidate == 0)
    {
        return 0U;
    }
    if(response_x < 0 || response_x >= BEACON_IMAGE_W ||
       response_y < 0 || response_y >= BEACON_IMAGE_H)
    {
        return 0U;
    }
    if(image[response_y][response_x] >= DOWN_GRAY_SUPPORT_THRESHOLD)
    {
        if(down_gray_point_in_current_lamps(response_x, response_y) != 0U)
        {
            return 0U;
        }
        candidate->seed_x = (unsigned char)response_x;
        candidate->seed_y = (unsigned char)response_y;
        return 1U;
    }
    if(response_x < DOWN_BEACON_LEFT_EDGE_MARGIN ||
       response_x >= BEACON_IMAGE_W - DOWN_BEACON_RIGHT_EDGE_MARGIN)
    {
        return 0U;
    }
    for(dy = -1; dy <= 1; dy++)
    {
        int y = response_y + dy;

        if(y < 0 || y >= BEACON_IMAGE_H)
        {
            continue;
        }
        for(dx = -1; dx <= 1; dx++)
        {
            int x = response_x + dx;
            int distance_sq = dx * dx + dy * dy;
            int gray;

            if(x < 0 || x >= BEACON_IMAGE_W)
            {
                continue;
            }
            gray = image[y][x];
            if(gray < DOWN_GRAY_SUPPORT_THRESHOLD ||
               down_gray_point_in_current_lamps(x, y) != 0U)
            {
                continue;
            }
            if(distance_sq < best_distance_sq ||
               (distance_sq == best_distance_sq && gray > best_gray))
            {
                best_gray = gray;
                best_distance_sq = distance_sq;
                candidate->seed_x = (unsigned char)x;
                candidate->seed_y = (unsigned char)y;
            }
        }
    }
    return (best_gray >= DOWN_GRAY_SUPPORT_THRESHOLD) ? 1U : 0U;
}

/* 同一高阈值连通域只能归属车灯或信标，低阈值光晕连接不参与该互斥。 */
static unsigned char down_gray_candidate_shares_car_core(
    const down_gray_candidate_t *candidate)
{
    component_t component;
    unsigned short *coordinate;
    unsigned short *end;
    unsigned int seed_index;

    if(candidate == 0 || g_car_support_valid == 0U)
    {
        return 0U;
    }
    seed_index = (unsigned int)candidate->seed_y * BEACON_IMAGE_W +
                 candidate->seed_x;
    if(candidate->seed_x >= BEACON_IMAGE_W ||
       candidate->seed_y >= BEACON_IMAGE_H ||
       g_binary[candidate->seed_y][candidate->seed_x] == 0U)
    {
        return 0U;
    }
    if(car_support_get(seed_index) != 0U)
    {
        return 1U;
    }
    begin_visit_pass();
    component = grow_component(candidate->seed_x, candidate->seed_y);
    coordinate = g_queue;
    end = g_queue + component.area;
    while(coordinate < end)
    {
        unsigned short value = *coordinate++;
        unsigned int pixel_index =
            (unsigned int)(unsigned char)(value >> 8U) * BEACON_IMAGE_W +
            (unsigned char)value;

        if(car_support_get(pixel_index) != 0U)
        {
            return 1U;
        }
    }
    return 0U;
}

static void down_gray_add_candidate(
    down_gray_candidate_t candidates[DOWN_GRAY_MAX_CANDIDATES],
    unsigned char *count,
    const down_gray_candidate_t *candidate)
{
    unsigned char index;

    if (count == 0 || candidate == 0)
    {
        return;
    }
    for (index = 0U; index < *count; index++)
    {
        float dx = candidate->x - candidates[index].x;
        float dy = candidate->y - candidates[index].y;
        float dedup_distance_sq = DOWN_GRAY_DEDUP_DISTANCE_SQ;
        if((candidate->area >= 60.0f) ||
           (candidates[index].area >= 60.0f))
        {
            dedup_distance_sq = 144.0f;
        }
        if (dx * dx + dy * dy <= dedup_distance_sq)
        {
            if (candidate->score > candidates[index].score)
            {
                candidates[index] = *candidate;
            }
            return;
        }
    }
    if (*count < DOWN_GRAY_MAX_CANDIDATES)
    {
        candidates[*count] = *candidate;
        (*count)++;
    }
}

static void down_beacon_copy_slot(beacon_result_t *result,
                                  unsigned char destination,
                                  unsigned char source)
{
    result->beacons[destination] = result->beacons[source];
    g_current_beacon_classification_x[destination] =
        g_current_beacon_classification_x[source];
    g_current_beacon_classification_y[destination] =
        g_current_beacon_classification_y[source];
    g_current_beacon_classification_valid[destination] =
        g_current_beacon_classification_valid[source];
}

static void down_beacon_clear_slot(beacon_result_t *result,
                                   unsigned char index)
{
    memset(&result->beacons[index], 0, sizeof(result->beacons[index]));
    g_current_beacon_classification_valid[index] = 0U;
}

static unsigned char down_beacon_corner_clipped(
    const beacon_circle_t *beacon)
{
    float x;
    float y;

    if(beacon == 0 || beacon->valid == 0U)
    {
        return 0U;
    }
    x = beacon->x + (float)BEACON_IMAGE_W * 0.5f;
    y = beacon->y + (float)BEACON_IMAGE_H * 0.5f;
    return ((x < (float)DOWN_BEACON_LEFT_EDGE_MARGIN ||
             x >= (float)(BEACON_IMAGE_W - DOWN_BEACON_RIGHT_EDGE_MARGIN)) &&
            (y < (float)DOWN_BEACON_CORNER_MARGIN ||
             y >= (float)(BEACON_IMAGE_H - DOWN_BEACON_CORNER_MARGIN))) ?
        1U : 0U;
}

/* 候选超额时优先舍弃同时被两个边界裁切的目标，角点目标本身不做硬过滤。 */
static void down_beacon_limit_result(beacon_result_t *result)
{
    while(result != 0 && result->beacon_count > 2U)
    {
        unsigned char remove = (unsigned char)(result->beacon_count - 1U);
        unsigned char index = result->beacon_count;

        while(index > 0U)
        {
            index--;
            if(down_beacon_corner_clipped(&result->beacons[index]) != 0U)
            {
                remove = index;
                break;
            }
        }
        for(index = remove; index + 1U < result->beacon_count; index++)
        {
            down_beacon_copy_slot(result, index, (unsigned char)(index + 1U));
        }
        result->beacon_count--;
        down_beacon_clear_slot(result, result->beacon_count);
    }
}

static void down_gray_insert_result(
    const down_gray_candidate_t *candidate,
    beacon_result_t *result)
{
    beacon_circle_t circle;
    int slot;
    int index;

    if (candidate == 0 || result == 0)
    {
        return;
    }
    slot = result->beacon_count;
    if (slot >= BEACON_MAX_BEACON_COUNT)
    {
        slot = BEACON_MAX_BEACON_COUNT - 1;
        if (candidate->area <= result->beacons[slot].area)
        {
            return;
        }
    }
    else
    {
        result->beacon_count++;
    }
    for (index = slot; index > 0; index--)
    {
        if (candidate->area <= result->beacons[index - 1].area)
        {
            break;
        }
        down_beacon_copy_slot(result, (unsigned char)index,
                              (unsigned char)(index - 1));
    }
    memset(&circle, 0, sizeof(circle));
    circle.x = candidate->x - (float)BEACON_IMAGE_W * 0.5f;
    circle.y = candidate->y - (float)BEACON_IMAGE_H * 0.5f;
    circle.area = candidate->area;
    circle.radius = sqrtf(candidate->area / PI_F);
    circle.valid = 1U;
    result->beacons[index] = circle;
    g_current_beacon_classification_x[index] = candidate->classification_x;
    g_current_beacon_classification_y[index] = candidate->classification_y;
    g_current_beacon_classification_valid[index] = 1U;
}

static void down_gray_build_snapshot(const beacon_result_t *result)
{
    unsigned char index;

    memset(g_beacon_binary_snapshot, 0, sizeof(g_beacon_binary_snapshot));
    if (result == 0)
    {
        return;
    }
    for (index = 0U; index < result->beacon_count; index++)
    {
        int center_x = (int)(result->beacons[index].x +
                             (float)BEACON_IMAGE_W * 0.5f + 0.5f);
        int center_y = (int)(result->beacons[index].y +
                             (float)BEACON_IMAGE_H * 0.5f + 0.5f);
        int radius = (int)(result->beacons[index].radius + 1.5f);
        int x;
        int y;

        if (radius < 2)
        {
            radius = 2;
        }
        for (y = center_y - radius; y <= center_y + radius; y++)
        {
            for (x = center_x - radius; x <= center_x + radius; x++)
            {
                if (x >= 0 && x < BEACON_IMAGE_W &&
                    y >= 0 && y < BEACON_IMAGE_H &&
                    (x - center_x) * (x - center_x) +
                    (y - center_y) * (y - center_y) <= radius * radius)
                {
                    g_beacon_binary_snapshot[y][x] = 255U;
                }
            }
        }
    }
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
    float scene_mean,
    beacon_result_t *result)
{
    down_gray_peak_t peaks[DOWN_GRAY_MAX_PEAKS];
    down_gray_candidate_t candidates[DOWN_GRAY_MAX_CANDIDATES];
    down_lamp_geometry_t lamp_geometry;
    down_lamp_geometry_t temporal_lamp_geometry;
    unsigned char peak_count;
    unsigned char candidate_count = 0U;
    unsigned char index;

    result->beacon_count = 0U;
    memset(candidates, 0, sizeof(candidates));
    memset(g_current_beacon_classification_valid, 0,
           sizeof(g_current_beacon_classification_valid));
    (void)lamp;
    (void)temporal_lamp;
    memset(&lamp_geometry, 0, sizeof(lamp_geometry));
    memset(&temporal_lamp_geometry, 0, sizeof(temporal_lamp_geometry));
    peak_count = down_gray_find_peaks(
        image, &lamp_geometry, &temporal_lamp_geometry, scene_mean, peaks);
    for (index = 0U; index < peak_count; index++)
    {
        down_gray_features_t features;
        down_gray_candidate_t candidate;
        int x = peaks[index].x;
        int y = peaks[index].y;
        unsigned char peak_clipped =
            (x == 0 || x == BEACON_IMAGE_W - 1 ||
             y == 0 || y == BEACON_IMAGE_H - 1) ? 1U : 0U;

        memset(&candidate, 0, sizeof(candidate));
        if (down_gray_local_features(image, x, y, &features) == 0U)
        {
            continue;
        }
        candidate.x = (float)x + features.centroid_x;
        candidate.y = (float)y + features.centroid_y;
        if (candidate.x < 0.0f || candidate.x > (float)(BEACON_IMAGE_W - 1) ||
            candidate.y < 0.0f || candidate.y > (float)(BEACON_IMAGE_H - 1) ||
            down_gray_point_in_current_lamps(
                (int)(candidate.x + 0.5f),
                (int)(candidate.y + 0.5f)) != 0U ||
            down_gray_point_in_lamp_geometry(
                (int)(candidate.x + 0.5f),
                (int)(candidate.y + 0.5f), &lamp_geometry) != 0U)
        {
            continue;
        }
        if (down_gray_candidate_valid(
                image, (int)(candidate.x + 0.5f),
                (int)(candidate.y + 0.5f), scene_mean, &features,
                peak_clipped) == 0U &&
            down_gray_companion_edge_candidate_valid(
                image, (int)(candidate.x + 0.5f),
                (int)(candidate.y + 0.5f), scene_mean, &features,
                candidates, candidate_count) == 0U)
        {
            continue;
        }
        if(down_gray_bind_candidate_source(image, x, y, &candidate) == 0U)
        {
            continue;
        }
        candidate.classification_x = candidate.x;
        candidate.classification_y = candidate.y;
        if(down_gray_refine_candidate_region(image, &candidate) == 0U)
        {
            continue;
        }
        if(down_gray_candidate_shares_car_core(&candidate) != 0U)
        {
            continue;
        }
        candidate.score = (float)peaks[index].response +
                          features.inner_contrast * 8.0f +
                          features.concentration * 500.0f;
        down_gray_add_candidate(candidates, &candidate_count, &candidate);
    }
    for (index = 0U; index < candidate_count; index++)
    {
        down_gray_insert_result(&candidates[index], result);
    }
    down_beacon_limit_result(result);
    sync_legacy_beacons(result);
    down_gray_build_snapshot(result);
}

static float square_distance(float ax, float ay, float bx, float by)
{
    float dx = ax - bx;
    float dy = ay - by;

    return dx * dx + dy * dy;
}

static void reset_track(temporal_track_t *track)
{
    memset(track, 0, sizeof(*track));
}

static unsigned char down_beacon_in_near_car_axis(
    const beacon_circle_t *beacon,
    const component_t *lamp)
{
    component_t core;
    float angle;
    float axis_x;
    float axis_y;
    float dx;
    float dy;
    float minor;
    int center_x;
    int center_y;
    int seed_x = -1;
    int seed_y = -1;
    int best_distance_sq = 0x7FFFFFFF;
    int offset_x;
    int offset_y;

    if(beacon == 0 || beacon->valid == 0U ||
       lamp == 0 || lamp->valid == 0U)
    {
        return 0U;
    }
    dx = beacon->x + (float)BEACON_IMAGE_W * 0.5f - lamp->cx;
    dy = beacon->y + (float)BEACON_IMAGE_H * 0.5f - lamp->cy;
    if(dx * dx + dy * dy >
       g_image_down_gate_distance * g_image_down_gate_distance)
    {
        return 0U;
    }
    angle = lamp->angle * (PI_F / 180.0f);
    axis_x = cosf(angle);
    axis_y = sinf(angle);
    minor = -dx * axis_y + dy * axis_x;
    if(fabsf(minor) >
       lamp->minor * 0.5f + DOWN_CAR_GROUP_MINOR_PAD)
    {
        return 0U;
    }
    center_x = (int)(beacon->x + (float)BEACON_IMAGE_W * 0.5f + 0.5f);
    center_y = (int)(beacon->y + (float)BEACON_IMAGE_H * 0.5f + 0.5f);
    for(offset_y = -DOWN_GRAY_PATCH_RADIUS;
        offset_y <= DOWN_GRAY_PATCH_RADIUS; offset_y++)
    {
        int y = center_y + offset_y;

        if(y < 0 || y >= BEACON_IMAGE_H)
        {
            continue;
        }
        for(offset_x = -DOWN_GRAY_PATCH_RADIUS;
            offset_x <= DOWN_GRAY_PATCH_RADIUS; offset_x++)
        {
            int x = center_x + offset_x;
            int distance_sq = offset_x * offset_x + offset_y * offset_y;

            if(x < 0 || x >= BEACON_IMAGE_W ||
               distance_sq > DOWN_GRAY_PATCH_RADIUS * DOWN_GRAY_PATCH_RADIUS ||
               g_binary[y][x] == 0U || distance_sq >= best_distance_sq)
            {
                continue;
            }
            seed_x = x;
            seed_y = y;
            best_distance_sq = distance_sq;
        }
    }
    if(seed_x < 0 || seed_y < 0)
    {
        return 1U;
    }
    begin_visit_pass();
    core = grow_component((unsigned char)seed_x, (unsigned char)seed_y);
    if(core.valid == 0U ||
       core.area <= g_image_down_near_lamp_isolated_min_area)
    {
        return 1U;
    }
    component_resolve_angle(&core);
    return (core.angle_valid != 0U &&
            down_car_axis_angle_difference(core.angle, lamp->angle) <=
                DOWN_CAR_GROUP_ANGLE_MAX) ? 1U : 0U;
}

static unsigned char down_near_car_beacon_track_update(
    const beacon_circle_t *beacon,
    unsigned char matched[DOWN_NEAR_CAR_BEACON_TRACKS])
{
    float best_distance_sq =
        g_image_down_match_distance * g_image_down_match_distance;
    unsigned char best = DOWN_NEAR_CAR_BEACON_TRACKS;
    unsigned char index;

    for(index = 0U; index < DOWN_NEAR_CAR_BEACON_TRACKS; index++)
    {
        down_near_car_beacon_track_t *track =
            &g_near_car_beacon_tracks[index];
        float distance_sq;

        if(track->active == 0U || matched[index] != 0U)
        {
            continue;
        }
        distance_sq = square_distance(
            track->x, track->y, beacon->x, beacon->y);
        if(distance_sq <= best_distance_sq)
        {
            best = index;
            best_distance_sq = distance_sq;
        }
    }
    if(best >= DOWN_NEAR_CAR_BEACON_TRACKS)
    {
        for(index = 0U; index < DOWN_NEAR_CAR_BEACON_TRACKS; index++)
        {
            if(g_near_car_beacon_tracks[index].active == 0U)
            {
                best = index;
                break;
            }
        }
        if(best >= DOWN_NEAR_CAR_BEACON_TRACKS)
        {
            return 0U;
        }
        memset(&g_near_car_beacon_tracks[best], 0,
               sizeof(g_near_car_beacon_tracks[best]));
        g_near_car_beacon_tracks[best].active = 1U;
        g_near_car_beacon_tracks[best].hits = 1U;
    }
    else if(g_near_car_beacon_tracks[best].hits < 255U)
    {
        g_near_car_beacon_tracks[best].hits++;
    }
    g_near_car_beacon_tracks[best].x = beacon->x;
    g_near_car_beacon_tracks[best].y = beacon->y;
    g_near_car_beacon_tracks[best].misses = 0U;
    matched[best] = 1U;
    if(g_near_car_beacon_tracks[best].hits >=
       DOWN_NEAR_CAR_BEACON_CONFIRM)
    {
        g_near_car_beacon_tracks[best].confirmed = 1U;
    }
    return g_near_car_beacon_tracks[best].confirmed;
}

/* 车灯主轴窄带内的紧凑独立核心需连续出现，避免断裂车灯单帧冒充信标。 */
static void down_filter_near_car_beacons(
    beacon_result_t *result,
    const component_t *lamp)
{
    unsigned char matched[DOWN_NEAR_CAR_BEACON_TRACKS] = {0U, 0U};
    unsigned char index = 0U;
    unsigned char removed = 0U;

    if(result == 0)
    {
        return;
    }
    while(index < result->beacon_count)
    {
        unsigned char keep = 1U;

        if(down_beacon_in_near_car_axis(
               &result->beacons[index], lamp) != 0U)
        {
            float match_distance_sq =
                g_image_down_match_distance * g_image_down_match_distance;
            if(g_b0_track.confirmed != 0U &&
               square_distance(
                   g_b0_track.x + g_b0_track.vx,
                   g_b0_track.y + g_b0_track.vy,
                   result->beacons[index].x,
                   result->beacons[index].y) <= match_distance_sq)
            {
                keep = 1U;
            }
            else
            {
                keep = down_near_car_beacon_track_update(
                    &result->beacons[index], matched);
            }
        }
        if(keep != 0U)
        {
            index++;
            continue;
        }
        {
            unsigned char remove = index;

            for(; remove + 1U < result->beacon_count; remove++)
            {
                down_beacon_copy_slot(
                    result, remove, (unsigned char)(remove + 1U));
            }
        }
        result->beacon_count--;
        down_beacon_clear_slot(result, result->beacon_count);
        removed = 1U;
    }
    for(index = 0U; index < DOWN_NEAR_CAR_BEACON_TRACKS; index++)
    {
        down_near_car_beacon_track_t *track =
            &g_near_car_beacon_tracks[index];

        if(track->active == 0U || matched[index] != 0U)
        {
            continue;
        }
        if(track->confirmed == 0U ||
           track->misses >= DOWN_NEAR_CAR_BEACON_MAX_MISSES)
        {
            memset(track, 0, sizeof(*track));
        }
        else
        {
            track->misses++;
        }
    }
    if(removed != 0U)
    {
        sync_legacy_beacons(result);
        down_gray_build_snapshot(result);
    }
}

static int down_edge_beacon_track_slot(float image_x)
{
    if(image_x < (float)DOWN_BEACON_LEFT_EDGE_MARGIN)
    {
        return 0;
    }
    if(image_x >= (float)(BEACON_IMAGE_W - DOWN_BEACON_RIGHT_EDGE_MARGIN))
    {
        return 1;
    }
    return -1;
}

static void down_edge_beacon_update_strict(
    down_edge_beacon_track_t *track,
    const beacon_circle_t *beacon,
    float image_x,
    float image_y)
{
    float gate = g_image_down_match_distance;
    float dx;
    float dy;

    if(track == 0 || beacon == 0)
    {
        return;
    }
    if(gate < 1.0f) gate = 1.0f;
    dx = image_x - track->x;
    dy = image_y - track->y;
    if(track->active == 0U || dx * dx + dy * dy > gate * gate)
    {
        memset(track, 0, sizeof(*track));
        track->active = 1U;
        track->hits = 1U;
    }
    else
    {
        track->vx = dx;
        track->vy = dy;
        if(track->hits < 255U) track->hits++;
    }
    track->x = image_x;
    track->y = image_y;
    track->area = beacon->area;
    track->misses = 0U;
    if(track->hits >= DOWN_EDGE_BEACON_CONFIRM)
    {
        track->confirmed = 1U;
    }
}

/* 只在已确认边缘轨迹的13x13预测窗口内恢复当前帧真实亮点。 */
static unsigned char down_edge_beacon_recover(
    down_edge_beacon_track_t *track,
    unsigned char slot,
    beacon_result_t *result)
{
    down_gray_features_t features;
    down_gray_candidate_t candidate;
    float predicted_x;
    float predicted_y;
    int center_x;
    int center_y;
    int best_x = -1;
    int best_y = -1;
    int best_gray = -1;
    int x;
    int y;
    unsigned char index;

    if(track == 0 || result == 0 || track->confirmed == 0U ||
       g_current_image == 0 || result->beacon_count != 1U)
    {
        return 0U;
    }
    predicted_x = track->x + track->vx;
    predicted_y = track->y + track->vy;
    if(down_edge_beacon_track_slot(predicted_x) != (int)slot)
    {
        return 0U;
    }
    center_x = (int)(predicted_x + 0.5f);
    center_y = (int)(predicted_y + 0.5f);
    for(y = center_y - DOWN_EDGE_BEACON_RECOVER_RADIUS;
        y <= center_y + DOWN_EDGE_BEACON_RECOVER_RADIUS; y++)
    {
        if(y < 0 || y >= BEACON_IMAGE_H) continue;
        for(x = center_x - DOWN_EDGE_BEACON_RECOVER_RADIUS;
            x <= center_x + DOWN_EDGE_BEACON_RECOVER_RADIUS; x++)
        {
            int gray;

            if(x < 0 || x >= BEACON_IMAGE_W ||
               down_edge_beacon_track_slot((float)x) != (int)slot ||
               down_gray_point_in_range(x, y) == 0U ||
               down_gray_point_in_current_lamps(x, y) != 0U)
            {
                continue;
            }
            gray = g_current_image[y][x];
            if(gray > best_gray)
            {
                best_gray = gray;
                best_x = x;
                best_y = y;
            }
        }
    }
    if(best_gray < (int)DOWN_EDGE_BEACON_RECOVER_PEAK ||
       down_gray_local_features(
           g_current_image, best_x, best_y, &features) == 0U ||
       features.half_area < 3U || features.half_area > 32U ||
       features.inner_contrast < 80.0f ||
       features.radial_drop < 65.0f ||
       features.concentration < 0.42f ||
       features.elongation > 2.20f ||
       features.offset > 2.00f ||
       features.outer_occupancy > 0.30f)
    {
        return 0U;
    }
    memset(&candidate, 0, sizeof(candidate));
    candidate.x = (float)best_x + features.centroid_x;
    candidate.y = (float)best_y + features.centroid_y;
    candidate.classification_x = candidate.x;
    candidate.classification_y = candidate.y;
    candidate.seed_x = (unsigned char)best_x;
    candidate.seed_y = (unsigned char)best_y;
    if(down_gray_refine_candidate_region(
           g_current_image, &candidate) == 0U ||
       candidate.area < 3.0f || candidate.area > 84.0f ||
       down_gray_candidate_shares_car_core(&candidate) != 0U)
    {
        return 0U;
    }
    for(index = 0U; index < result->beacon_count; index++)
    {
        float existing_x = result->beacons[index].x +
                           (float)BEACON_IMAGE_W * 0.5f;
        float existing_y = result->beacons[index].y +
                           (float)BEACON_IMAGE_H * 0.5f;

        if(square_distance(
               existing_x, existing_y,
               candidate.x, candidate.y) <= DOWN_GRAY_DEDUP_DISTANCE_SQ)
        {
            return 0U;
        }
    }
    index = result->beacon_count;
    memset(&result->beacons[index], 0, sizeof(result->beacons[index]));
    result->beacons[index].x = candidate.x -
                               (float)BEACON_IMAGE_W * 0.5f;
    result->beacons[index].y = candidate.y -
                               (float)BEACON_IMAGE_H * 0.5f;
    result->beacons[index].area = candidate.area;
    result->beacons[index].radius = sqrtf(candidate.area / PI_F);
    result->beacons[index].valid = 1U;
    g_current_beacon_classification_x[index] = candidate.classification_x;
    g_current_beacon_classification_y[index] = candidate.classification_y;
    g_current_beacon_classification_valid[index] = 1U;
    result->beacon_count++;
    track->x = candidate.x;
    track->y = candidate.y;
    track->area = candidate.area;
    return 1U;
}

static void down_edge_beacon_continue(beacon_result_t *result)
{
    unsigned char matched[DOWN_EDGE_BEACON_TRACKS] = {0U, 0U};
    unsigned char changed = 0U;
    unsigned char index;
    unsigned char max_misses = 3U;

    if(result == 0)
    {
        return;
    }
    if(g_image_down_max_misses >= 0 &&
       g_image_down_max_misses < (int32)max_misses)
    {
        max_misses = (unsigned char)g_image_down_max_misses;
    }
    for(index = 0U; index < result->beacon_count; index++)
    {
        float image_x = result->beacons[index].x +
                        (float)BEACON_IMAGE_W * 0.5f;
        float image_y = result->beacons[index].y +
                        (float)BEACON_IMAGE_H * 0.5f;
        int slot = down_edge_beacon_track_slot(image_x);

        if(slot < 0 || matched[slot] != 0U)
        {
            continue;
        }
        down_edge_beacon_update_strict(
            &g_edge_beacon_tracks[slot],
            &result->beacons[index], image_x, image_y);
        matched[slot] = 1U;
    }
    for(index = 0U; index < DOWN_EDGE_BEACON_TRACKS; index++)
    {
        down_edge_beacon_track_t *track = &g_edge_beacon_tracks[index];

        if(matched[index] != 0U || track->active == 0U)
        {
            continue;
        }
        if(track->confirmed == 0U || track->misses >= max_misses)
        {
            memset(track, 0, sizeof(*track));
            continue;
        }
        if(down_edge_beacon_recover(track, index, result) != 0U)
        {
            changed = 1U;
        }
        track->misses++;
    }
    if(changed != 0U)
    {
        sync_legacy_beacons(result);
        down_gray_build_snapshot(result);
    }
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

    track->vx = (1.0f - g_image_down_filter_vel_alpha) * track->vx +
                g_image_down_filter_vel_alpha * (x - old_x);
    track->vy = (1.0f - g_image_down_filter_vel_alpha) * track->vy +
                g_image_down_filter_vel_alpha * (y - old_y);
    track->x = g_image_down_filter_pos_alpha * x +
               (1.0f - g_image_down_filter_pos_alpha) * predict_x;
    track->y = g_image_down_filter_pos_alpha * y +
               (1.0f - g_image_down_filter_pos_alpha) * predict_y;
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
    float best_d2 = g_image_down_match_distance * g_image_down_match_distance;

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
            down_beacon_copy_slot(result, (unsigned char)i,
                                  (unsigned char)(i + 1));
        }
        down_beacon_clear_slot(
            result, (unsigned char)(result->beacon_count - 1));
        result->beacon_count--;
    }

    if (result->beacon_count >= BEACON_MAX_BEACON_COUNT)
    {
        result->beacon_count = BEACON_MAX_BEACON_COUNT - 1;
    }
    for (i = result->beacon_count; i > 0; i--)
    {
        down_beacon_copy_slot(result, (unsigned char)i,
                              (unsigned char)(i - 1));
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

static void clear_car_result(beacon_result_t *result)
{
    memset(result->car_lamps, 0, sizeof(result->car_lamps));
    result->car_lamp_count = 0U;
    result->car_lamp_measured_mask = 0U;
    result->car_lamp_classification = DOWN_CAR_CLASS_NONE;
    reset_track(&g_car_track);
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

static void update_current_beacon_track(const beacon_circle_t *measurement)
{
    float distance_limit =
        g_image_down_new_target_distance * g_image_down_new_target_distance;

    if (measurement == 0)
    {
        return;
    }
    if (g_b0_track.active == 0 ||
        square_distance(g_b0_track.x, g_b0_track.y,
                        measurement->x, measurement->y) > distance_limit)
    {
        start_pending_track(&g_b0_track, measurement->x, measurement->y);
    }
    else
    {
        float dx = measurement->x - g_b0_track.x;
        float dy = measurement->y - g_b0_track.y;
        g_b0_track.vx = (1.0f - g_image_down_filter_vel_alpha) * g_b0_track.vx +
                        g_image_down_filter_vel_alpha * dx;
        g_b0_track.vy = (1.0f - g_image_down_filter_vel_alpha) * g_b0_track.vy +
                        g_image_down_filter_vel_alpha * dy;
        g_b0_track.x = measurement->x;
        g_b0_track.y = measurement->y;
        g_b0_track.misses = 0U;
        if (g_b0_track.confirmed == 0U)
        {
            if (g_b0_track.hits < 255U)
            {
                g_b0_track.hits++;
            }
            if (g_b0_track.hits >= g_image_down_confirm_frames)
            {
                g_b0_track.confirmed = 1U;
            }
        }
    }
    set_beacon_track_shape(&g_b0_track, measurement);
}

static unsigned char update_temporal_car(
    beacon_result_t *result,
    const component_t *identity_lamp,
    unsigned char identity_classification)
{
    beacon_rect_t *measurement = 0;
    beacon_rect_t identity_measurement;

    if (result->car_lamp_count > 0 && result->car_lamps[0].valid != 0)
    {
        if(result->car_lamp_classification == DOWN_CAR_CLASS_NONE)
        {
            clear_car_result(result);
            return 0U;
        }
        if(result->car_lamp_classification == DOWN_CAR_CLASS_TRACK &&
           g_car_track.confirmed == 0U)
        {
            clear_car_result(result);
            return 0U;
        }
        if(identity_lamp != 0 && identity_lamp->valid != 0U &&
           identity_classification != DOWN_CAR_CLASS_NONE)
        {
            memset(&identity_measurement, 0, sizeof(identity_measurement));
            fill_car_lamp_rect(identity_lamp, &identity_measurement);
            measurement = &identity_measurement;
        }
        else
        {
            measurement = &result->car_lamps[0];
        }
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
            g_image_down_gate_distance * g_image_down_gate_distance)
    {
        if(result->car_lamp_classification < DOWN_CAR_CLASS_WEAK)
        {
            clear_car_result(result);
            return 0U;
        }
        g_car_track.x = measurement->cx;
        g_car_track.y = measurement->cy;
        g_car_track.vx = 0.0f;
        g_car_track.vy = 0.0f;
        g_car_track.misses = 0U;
        set_car_track_shape(&g_car_track, measurement);
        return 1U;
    }
    if (g_car_track.confirmed == 0 &&
        square_distance(g_car_track.x, g_car_track.y,
                        measurement->cx, measurement->cy) >
            g_image_down_new_target_distance * g_image_down_new_target_distance)
    {
        start_pending_track(&g_car_track, measurement->cx, measurement->cy);
        set_car_track_shape(&g_car_track, measurement);
        return 0;
    }

    update_track_position(&g_car_track, measurement->cx, measurement->cy);
    set_car_track_shape(&g_car_track, measurement);
    if (g_car_track.confirmed == 0U)
    {
        if (g_car_track.hits < 255U)
        {
            g_car_track.hits++;
        }
        if (g_car_track.hits >= g_image_down_confirm_frames)
        {
            g_car_track.confirmed = 1U;
        }
    }
    return g_car_track.confirmed;
}

static unsigned char down_gray_pending_car_fragment(
    const beacon_circle_t *beacon,
    unsigned char index)
{
    down_gray_features_t features;
    int x;
    int y;

    if(beacon == 0 || beacon->valid == 0U || g_current_image == 0)
    {
        return 0U;
    }
    if(index < BEACON_MAX_BEACON_COUNT &&
       g_current_beacon_classification_valid[index] != 0U)
    {
        x = (int)(g_current_beacon_classification_x[index] + 0.5f);
        y = (int)(g_current_beacon_classification_y[index] + 0.5f);
    }
    else
    {
        x = (int)(beacon->x + (float)BEACON_IMAGE_W * 0.5f + 0.5f);
        y = (int)(beacon->y + (float)BEACON_IMAGE_H * 0.5f + 0.5f);
    }
    if(down_gray_local_features(g_current_image, x, y, &features) == 0U)
    {
        return 0U;
    }
    return (features.peak < 235U &&
            features.half_area <= 20U &&
            features.elongation >= 1.70f) ? 1U : 0U;
}

static unsigned char temporal_track_has_current_support(
    const temporal_track_t *track)
{
    int center_x;
    int center_y;

    if(track == 0 || g_current_image == 0 ||
       isfinite(track->x) == 0 || isfinite(track->y) == 0)
    {
        return 0U;
    }
    center_x = (int)(track->x + (float)BEACON_IMAGE_W * 0.5f + 0.5f);
    center_y = (int)(track->y + (float)BEACON_IMAGE_H * 0.5f + 0.5f);
    return (center_x >= 0 && center_x < BEACON_IMAGE_W &&
            center_y >= 0 && center_y < BEACON_IMAGE_H &&
            g_current_image[center_y][center_x] >=
                DOWN_GRAY_SUPPORT_THRESHOLD) ? 1U : 0U;
}

static void apply_temporal_beacon(beacon_result_t *result)
{
    unsigned char max_misses = DOWN_BEACON_COAST_FRAMES;

    if (result->beacon_count > 0U)
    {
        unsigned char pending_car_fragment =
            down_gray_pending_car_fragment(&result->beacons[0], 0U);

        if(pending_car_fragment != 0U && g_b0_track.confirmed == 0U)
        {
            if(result->beacon_count == 1U)
            {
                /* 单个拉长小亮域只积累轨迹，确认前不对外输出。 */
                update_current_beacon_track(&result->beacons[0]);
                result->beacon_count--;
                down_beacon_clear_slot(result, result->beacon_count);
            }
            else
            {
                unsigned char index;

                /* 多目标时先移除可疑项，避免它污染真正 B0 的主轨迹。 */
                for(index = 0U; index + 1U < result->beacon_count; index++)
                {
                    down_beacon_copy_slot(
                        result, index, (unsigned char)(index + 1U));
                }
                down_beacon_clear_slot(
                    result, (unsigned char)(result->beacon_count - 1U));
                result->beacon_count--;
                update_current_beacon_track(&result->beacons[0]);
            }
            sync_legacy_beacons(result);
            down_gray_build_snapshot(result);
            return;
        }
        update_current_beacon_track(&result->beacons[0]);
        return;
    }
    if (g_b0_track.confirmed != 0U &&
        down_gray_point_in_current_lamps(
            (int)(g_b0_track.x + (float)BEACON_IMAGE_W * 0.5f + 0.5f),
            (int)(g_b0_track.y + (float)BEACON_IMAGE_H * 0.5f + 0.5f)) != 0U)
    {
        reset_track(&g_b0_track);
        return;
    }
    if (g_image_down_max_misses >= 0 &&
        g_image_down_max_misses < (int32)max_misses)
    {
        max_misses = (unsigned char)g_image_down_max_misses;
    }
    if (predict_missed_track(&g_b0_track, max_misses) != 0U)
    {
        if(temporal_track_has_current_support(&g_b0_track) == 0U)
        {
            reset_track(&g_b0_track);
            return;
        }
        output_temporal_beacon(&g_b0_track, result);
    }
}

static void apply_temporal_car(
    beacon_result_t *result,
    unsigned char current_track_support,
    const component_t *identity_lamp,
    unsigned char identity_classification)
{
    unsigned char has_measurement =
        (result->car_lamp_count > 0U &&
         result->car_lamps[0].valid != 0U) ? 1U : 0U;

    if (update_temporal_car(
            result, identity_lamp, identity_classification) == 0)
    {
        return;
    }
    if (has_measurement == 0U)
    {
        if(current_track_support == 0U)
        {
            return;
        }
        output_temporal_car(&g_car_track, result);
    }
}

static void update_temporal_result(
    beacon_result_t *result,
    unsigned char current_car_track_support,
    const component_t *identity_lamp,
    unsigned char identity_classification)
{
    apply_temporal_beacon(result);
    apply_temporal_car(
        result, current_car_track_support,
        identity_lamp, identity_classification);
}

static void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    component_t lamp;
    component_t output_lamp;
    component_t temporal_lamp;
    const component_t *temporal_lamp_ptr = 0;
    float scene_mean;
    unsigned char has_lamp;
    unsigned char center_track_mismatch;
    unsigned char lamp_classification = DOWN_CAR_CLASS_NONE;
    unsigned char i;

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
    scene_mean = threshold_image(
        image, (unsigned char)g_image_down_car_lamp_binary_threshold);
    memcpy(g_car_lamp_binary_snapshot, g_binary,
           sizeof(g_car_lamp_binary_snapshot));
    memset(g_car_support_mask, 0, sizeof(g_car_support_mask));
    g_car_support_valid = 0U;
    memset(&g_car_output_override, 0, sizeof(g_car_output_override));
    g_car_output_override_valid = 0U;
    has_lamp = find_car_lamp(&lamp, &lamp_classification);
    center_track_mismatch =
        (has_lamp != 0U) ? down_car_center_track_mismatch(&lamp) : 0U;
    if(has_lamp == 0U ||
       lamp_classification == DOWN_CAR_CLASS_TRACK ||
       center_track_mismatch != 0U)
    {
        if(down_car_recover_center_track(
               &lamp, &lamp_classification) != 0U)
        {
            has_lamp = 1U;
            memset(g_car_support_mask, 0, sizeof(g_car_support_mask));
            g_car_support_valid = 0U;
        }
    }
    if (has_lamp == 0)
    {
        memset(&lamp, 0, sizeof(lamp));
    }
    else
    {
        if(g_car_support_valid == 0U &&
           lamp_classification != DOWN_CAR_CLASS_NONE)
        {
            if(down_car_build_single_support(&lamp, 0U) == 0U)
            {
                down_car_clear_support();
                memset(&lamp, 0, sizeof(lamp));
                has_lamp = 0U;
                lamp_classification = DOWN_CAR_CLASS_NONE;
            }
        }
    }

    output_lamp = lamp;
    if(has_lamp != 0U && lamp_classification != DOWN_CAR_CLASS_NONE)
    {
        if(g_car_output_override_valid != 0U)
        {
            output_lamp = g_car_output_override;
        }
        else
        {
            (void)down_car_fit_output_geometry(&lamp, &output_lamp);
        }
    }
    write_car_lamp(&output_lamp, lamp_classification, result);
    if(lamp_classification >= DOWN_CAR_CLASS_WEAK)
    {
        down_car_clear_supported_beacon_tracks();
    }
    memset(&temporal_lamp, 0, sizeof(temporal_lamp));
    if((has_lamp == 0U ||
        lamp_classification < DOWN_CAR_CLASS_WEAK) &&
       g_car_track.confirmed != 0U &&
       component_from_temporal_car(
           &g_car_track, &temporal_lamp, 1U) != 0U)
    {
        if(down_car_merge_temporal_support(&temporal_lamp) != 0U)
        {
            int bridge_min_x = temporal_lamp.min_x -
                               CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
            int bridge_max_x = temporal_lamp.max_x +
                               CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
            int bridge_min_y = temporal_lamp.min_y -
                               CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
            int bridge_max_y = temporal_lamp.max_y +
                               CAR_LAMP_TEMPORAL_TAKEOVER_PAD;

            if(bridge_min_x < 0) bridge_min_x = 0;
            if(bridge_min_y < 0) bridge_min_y = 0;
            if(bridge_max_x >= BEACON_IMAGE_W)
            {
                bridge_max_x = BEACON_IMAGE_W - 1;
            }
            if(bridge_max_y >= BEACON_IMAGE_H)
            {
                bridge_max_y = BEACON_IMAGE_H - 1;
            }
            if(down_car_try_center_recovery_bridge(
                   &temporal_lamp,
                   bridge_min_x, bridge_max_x,
                   bridge_min_y, bridge_max_y) == 0U)
            {
                (void)down_car_merge_temporal_support(&temporal_lamp);
            }
            temporal_lamp_ptr = &temporal_lamp;
        }
    }
    find_beacons(
        image,
        (lamp_classification >= DOWN_CAR_CLASS_WEAK) ? &lamp : 0,
        temporal_lamp_ptr, scene_mean, result);
    down_filter_near_car_beacons(
        result,
        (result->car_lamp_measured_mask != 0U) ? &lamp :
        ((temporal_lamp.valid != 0U) ? &temporal_lamp : 0));
    down_edge_beacon_continue(result);
    update_temporal_result(
        result, (temporal_lamp_ptr != 0) ? 1U : 0U,
        &lamp, lamp_classification);
    if(result->car_lamp_measured_mask == 0U &&
       result->car_lamp_count > 0U &&
       g_car_output_override_valid != 0U)
    {
        fill_car_lamp_rect(
            &g_car_output_override, &result->car_lamps[0]);
    }

    for (i = result->car_lamp_count; i < BEACON_MAX_CAR_LAMP_COUNT; i++)
    {
        result->car_lamps[i].valid = 0;
    }
}

/*
 * 函数功能: 基于摄像头来源帧号锁存一帧稳定图像，并跳过已处理的旧帧。
 * 输入参数: 无。
 * 返回值: 1表示锁存到真实新帧；0表示当前没有尚未处理的新帧。
 */
static uint8 image_down_latch_frame(void)
{
    uint32 frame_sequence;
    uint32 frame_timestamp_ms;
    uint32 interrupt_state;

    if((mt9v03x_frame_sequence == 0U) ||
       (mt9v03x_frame_sequence == s_image_down_latched_frame_sequence))
    {
        return 0U;
    }

    /* 若帧完成中断在复制期间更新源缓冲区，则重拷一次最新完整帧。 */
    do
    {
        frame_sequence = mt9v03x_frame_sequence;
        frame_timestamp_ms = mt9v03x_frame_timestamp_ms;
        memcpy(g_image_frame[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    }
    while(frame_sequence != mt9v03x_frame_sequence);

    s_image_down_latched_frame_sequence = frame_sequence;
    image_frame_meta[Center].frame_sequence = frame_sequence;
    image_frame_meta[Center].capture_time_ms = frame_timestamp_ms;
    image_frame_meta[Center].source_camera = (uint8)Center;
    image_frame_meta[Center].frame_valid = 1U;
    image_frame_meta[Center].timestamp_valid = 1U;
    image_frame_meta[Center].reserved = 0U;
    interrupt_state = interrupt_global_disable();
    if(mt9v03x_frame_sequence == frame_sequence)
    {
        mt9v03x_finish_flag = 0U;
    }
    interrupt_global_enable(interrupt_state);
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
    image_data[Center].car_lamp_measured_mask =
        result->car_lamp_measured_mask & 0x01U;
}

/* 初始化下摄图像算法和摄像头接口。 */
void image_down_init(void)
{
    memset(g_image_frame, 0, MT9V03X_IMAGE_SIZE);
    image_down_clear_results();
    image_frame_meta_clear(&image_frame_meta[Center], Center);
    beacon_image_init();
    s_image_down_latched_frame_sequence = 0U;

    mt9v03x_finish_flag = 0U;
    s_mt9v03x_initialized = (mt9v03x_init() == 0U) ? 1U : 0U;
}

/*
 * 函数功能: 仅在摄像头发布真实新帧时锁存图像并执行算法。
 * 输入参数: 无。
 * 返回值: 1表示本次完成了一帧处理；0表示没有可处理的新帧。
 */
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
    return g_beacon_binary_snapshot[0];
}

const uint8 *image_down_get_car_lamp_binary_buffer(void)
{
    return g_car_lamp_binary_snapshot[0];
}

/*
 * 函数功能: 在核1图像帧边界执行图像参数SET/GET，并返回实际读回值。
 * 输入参数: op操作码；type数值类型；param_id参数ID；value_bits目标值位模式；actual_bits实际值位模式输出。
 * 返回值: IPC_REMOTE_PARAM_STATUS_*统一状态码。
 */
typedef uint8 (*image_down_param_handler_t)(uint8 op,
                                            uint32 requested_bits,
                                            uint32 *actual_bits);

typedef struct
{
    uint16 id;
    uint8 type;
    void *value_ptr;
    float minimum;
    float maximum;
    image_down_param_handler_t handler;
} image_down_param_descriptor_t;

#define IMAGE_DOWN_PARAM_I(id_, value_, min_, max_) \
    {(id_), IPC_REMOTE_PARAM_TYPE_INT32, &(value_), (min_), (max_), NULL}
#define IMAGE_DOWN_PARAM_F(id_, value_, min_, max_) \
    {(id_), IPC_REMOTE_PARAM_TYPE_FLOAT, &(value_), (min_), (max_), NULL}
#define IMAGE_DOWN_PARAM_CUSTOM_I(id_, handler_) \
    {(id_), IPC_REMOTE_PARAM_TYPE_INT32, NULL, 0.0f, 0.0f, (handler_)}

static uint32 image_down_param_value_bits(
    const image_down_param_descriptor_t *param)
{
    uint32 bits = 0U;
    memcpy(&bits, param->value_ptr, sizeof(bits));
    return bits;
}

static int32 image_down_param_bits_to_int32(uint32 bits)
{
    int32 value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint8 image_down_exposure_param_execute(uint8 op,
                                               uint32 requested_bits,
                                               uint32 *actual_bits)
{
    int32 value;
    uint16 old_value = g_mt9v03x_exp_time;

    if(op == IPC_REMOTE_PARAM_OP_SET)
    {
        value = image_down_param_bits_to_int32(requested_bits);
        if((value < 0) || (value > 636))
        {
            *actual_bits = (uint32)g_mt9v03x_exp_time;
            return IPC_REMOTE_PARAM_STATUS_OUT_OF_RANGE;
        }
        if(((uint16)value == old_value) && (s_mt9v03x_initialized != 0U))
        {
            *actual_bits = (uint32)g_mt9v03x_exp_time;
            return IPC_REMOTE_PARAM_STATUS_OK;
        }

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
    else if(op != IPC_REMOTE_PARAM_OP_GET)
    {
        *actual_bits = (uint32)g_mt9v03x_exp_time;
        return IPC_REMOTE_PARAM_STATUS_ERROR;
    }

    *actual_bits = (uint32)g_mt9v03x_exp_time;
    return (s_mt9v03x_initialized != 0U) ?
           IPC_REMOTE_PARAM_STATUS_OK : IPC_REMOTE_PARAM_STATUS_ERROR;
}

static uint8 image_down_screen_param_execute(uint8 op,
                                             uint32 requested_bits,
                                             uint32 *actual_bits)
{
    int32 value;

    if(op == IPC_REMOTE_PARAM_OP_SET)
    {
        value = image_down_param_bits_to_int32(requested_bits);
        if((value < (int32)IMAGE_DEBUG_SCREEN_MODE_DATA) ||
           (value > (int32)IMAGE_DEBUG_SCREEN_MODE_OVERLAY))
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

static const image_down_param_descriptor_t s_image_down_params[] =
{
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_BEACON_THR,
                       g_image_down_beacon_binary_threshold, 0, 255),
    IMAGE_DOWN_PARAM_CUSTOM_I(IPC_REMOTE_PARAM_ID_C1_EXP_TIME,
                              image_down_exposure_param_execute),
    IMAGE_DOWN_PARAM_CUSTOM_I(IPC_REMOTE_PARAM_ID_C1_SCREEN_MODE,
                              image_down_screen_param_execute),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_BEACON_MIN,
                       g_image_down_beacon_min_area, 0, 22560),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_EDGE_MIN,
                       g_image_down_side_edge_min_area, 0, 22560),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_EDGE_THR,
                       g_image_down_side_edge_threshold, 0, 255),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_LAMP_THR,
                       g_image_down_car_lamp_binary_threshold, 0, 255),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_LAMP_MIN,
                       g_image_down_car_lamp_min_area, 0, 22560),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_LAMP_MAX,
                       g_image_down_car_lamp_max_area, 0, 22560),
    IMAGE_DOWN_PARAM_F(IPC_REMOTE_PARAM_ID_C1_LAMP_ELONG,
                       g_image_down_car_lamp_min_elongation, 0.0f, 224.0f),
    IMAGE_DOWN_PARAM_F(IPC_REMOTE_PARAM_ID_C1_LAMP_LEN,
                       g_image_down_car_lamp_min_length, 0.0f, 224.0f),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_NEAR_PAD,
                       g_image_down_near_lamp_pad, 0, 224),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_NEAR_MIN,
                       g_image_down_near_lamp_min_area, 0, 22560),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_NEAR_ISO_MIN,
                       g_image_down_near_lamp_isolated_min_area, 0, 22560),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_NEAR_BG,
                       g_image_down_near_lamp_background_max, 0, 255),
    IMAGE_DOWN_PARAM_F(IPC_REMOTE_PARAM_ID_C1_MATCH_DIST,
                       g_image_down_match_distance, 0.0f, 224.0f),
    IMAGE_DOWN_PARAM_F(IPC_REMOTE_PARAM_ID_C1_GATE_DIST,
                       g_image_down_gate_distance, 0.0f, 224.0f),
    IMAGE_DOWN_PARAM_F(IPC_REMOTE_PARAM_ID_C1_NEW_DIST,
                       g_image_down_new_target_distance, 0.0f, 224.0f),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_CONFIRM,
                       g_image_down_confirm_frames, 1, 255),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_MISSES,
                       g_image_down_max_misses, 0, 255),
    IMAGE_DOWN_PARAM_F(IPC_REMOTE_PARAM_ID_C1_POS_ALPHA,
                       g_image_down_filter_pos_alpha, 0.0f, 1.0f),
    IMAGE_DOWN_PARAM_F(IPC_REMOTE_PARAM_ID_C1_VEL_ALPHA,
                       g_image_down_filter_vel_alpha, 0.0f, 1.0f),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_LAMP_EDGE_MISSES,
                       g_image_down_car_lamp_edge_max_misses, 0, 255),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_LAMP_CENTER_MISSES,
                       g_image_down_car_lamp_center_max_misses, 0, 255),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_TEMP_CORE_PAD,
                       g_image_down_car_lamp_temporal_core_pad, 0, 224),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_TEMP_TAKEOVER_PAD,
                       g_image_down_car_lamp_temporal_takeover_pad, 0, 224),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_TEMP_MIN_BRIGHT,
                       g_image_down_car_lamp_temporal_min_bright_area, 0, 22560),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_OUTPUT_ENV_PAD,
                       g_image_down_car_output_envelope_pad, 0, 224),
    IMAGE_DOWN_PARAM_F(IPC_REMOTE_PARAM_ID_C1_GROUP_ANGLE_MAX,
                       g_image_down_car_group_angle_max, 0.1f, 180.0f),
    IMAGE_DOWN_PARAM_F(IPC_REMOTE_PARAM_ID_C1_GROUP_MINOR_PAD,
                       g_image_down_car_group_minor_pad, 0.0f, 224.0f),
    IMAGE_DOWN_PARAM_F(IPC_REMOTE_PARAM_ID_C1_GROUP_MAJOR_GAP,
                       g_image_down_car_group_major_gap, 0.1f, 224.0f),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_RECOVER_MIN_CORE,
                       g_image_down_car_recover_min_core_area, 0, 22560),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_RECOVER_MAX_CORE,
                       g_image_down_car_recover_max_core_area, 0, 22560),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_RECOVER_CONNECT_THR,
                       g_image_down_car_recover_connect_threshold, 0, 255),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_RECOVER_TRACK_THR,
                       g_image_down_car_recover_track_threshold, 0, 255),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_RECOVER_BRIDGE_THR,
                       g_image_down_car_recover_bridge_threshold, 0, 255),
    IMAGE_DOWN_PARAM_F(IPC_REMOTE_PARAM_ID_C1_RECOVER_SUPPORT_PAD,
                       g_image_down_car_recover_support_pad, 0.0f, 224.0f),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_SUPPORT_PAD,
                       g_image_down_car_support_pad, 0, 224),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_GRAY_SUPPORT_THR,
                       g_image_down_gray_support_threshold, 0, 255),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_COMPACT_EDGE_MARGIN,
                       g_image_down_car_compact_edge_margin, 0, 120),
    IMAGE_DOWN_PARAM_F(IPC_REMOTE_PARAM_ID_C1_COMPACT_MAX_MAJOR,
                       g_image_down_car_compact_edge_max_major, 0.0f, 224.0f),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_BEACON_COAST,
                       g_image_down_beacon_coast_frames, 0, 255),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_BEACON_CORNER_MARGIN,
                       g_image_down_beacon_corner_margin, 0, 120),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_NEAR_BEACON_CONFIRM,
                       g_image_down_near_car_beacon_confirm, 1, 255),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_NEAR_BEACON_MISSES,
                       g_image_down_near_car_beacon_max_misses, 0, 255),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_EDGE_BEACON_CONFIRM,
                       g_image_down_edge_beacon_confirm, 1, 255),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_EDGE_BEACON_RADIUS,
                       g_image_down_edge_beacon_recover_radius, 0, 224),
    IMAGE_DOWN_PARAM_I(IPC_REMOTE_PARAM_ID_C1_EDGE_BEACON_PEAK,
                       g_image_down_edge_beacon_recover_peak, 0, 255)
};

typedef char image_down_param_count_must_match[
    (sizeof(s_image_down_params) / sizeof(s_image_down_params[0]) == 48U) ?
    1 : -1];

static const image_down_param_descriptor_t *image_down_find_param(uint16 id)
{
    uint16 index;
    for(index = 0U;
        index < (uint16)(sizeof(s_image_down_params) /
                         sizeof(s_image_down_params[0]));
        index++)
    {
        if(s_image_down_params[index].id == id)
        {
            return &s_image_down_params[index];
        }
    }
    return NULL;
}

static uint8 image_down_param_values_valid(
    const image_down_param_descriptor_t *param)
{
    float float_value;
    int32 int_value;

    if(param->type == IPC_REMOTE_PARAM_TYPE_FLOAT)
    {
        memcpy(&float_value, param->value_ptr, sizeof(float_value));
        if((isfinite(float_value) == 0) ||
           (float_value < param->minimum) ||
           (float_value > param->maximum))
        {
            return 0U;
        }
    }
    else
    {
        memcpy(&int_value, param->value_ptr, sizeof(int_value));
        if(((float)int_value < param->minimum) ||
           ((float)int_value > param->maximum))
        {
            return 0U;
        }
    }

    return ((g_image_down_car_lamp_min_area <=
             g_image_down_car_lamp_max_area) &&
            (g_image_down_car_recover_min_core_area <=
             g_image_down_car_recover_max_core_area)) ? 1U : 0U;
}

uint8 image_down_remote_param_execute(uint8 op,
                                      uint8 type,
                                      uint16 param_id,
                                      uint32 requested_bits,
                                      uint32 *actual_bits)
{
    const image_down_param_descriptor_t *param;
    uint32 previous_bits;

    if(actual_bits == NULL)
    {
        return IPC_REMOTE_PARAM_STATUS_ERROR;
    }
    *actual_bits = 0U;

    param = image_down_find_param(param_id);
    if(param == NULL)
    {
        return IPC_REMOTE_PARAM_STATUS_NOT_FOUND;
    }
    if(type != param->type)
    {
        return IPC_REMOTE_PARAM_STATUS_MISMATCH;
    }
    if(param->handler != NULL)
    {
        return param->handler(op, requested_bits, actual_bits);
    }

    *actual_bits = image_down_param_value_bits(param);
    if(op == IPC_REMOTE_PARAM_OP_GET)
    {
        return IPC_REMOTE_PARAM_STATUS_OK;
    }
    if(op != IPC_REMOTE_PARAM_OP_SET)
    {
        return IPC_REMOTE_PARAM_STATUS_ERROR;
    }
    if(*actual_bits == requested_bits)
    {
        return IPC_REMOTE_PARAM_STATUS_OK;
    }

    previous_bits = *actual_bits;
    memcpy(param->value_ptr, &requested_bits, sizeof(requested_bits));
    if(image_down_param_values_valid(param) == 0U)
    {
        memcpy(param->value_ptr, &previous_bits, sizeof(previous_bits));
        return IPC_REMOTE_PARAM_STATUS_OUT_OF_RANGE;
    }

    beacon_image_reset_temporal();
    *actual_bits = image_down_param_value_bits(param);
    return (*actual_bits == requested_bits) ?
           IPC_REMOTE_PARAM_STATUS_OK : IPC_REMOTE_PARAM_STATUS_ERROR;
}
