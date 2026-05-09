#include "image.h"

#include <math.h>
#include <string.h>

#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"

#define IMAGE_DEFAULT_THRESHOLD (50U) /* 默认固定阈值二值化阈值 */
#define IMAGE_ADAPTIVE_THRESHOLD_BLOCK (41U) /* 自适应阈值窗口边长（需为奇数） */
#define IMAGE_ADAPTIVE_THRESHOLD_BRIGHT (245U) /* 极亮像素直接置白阈值 */
#define IMAGE_ADAPTIVE_THRESHOLD_TOP_LOW (50U) /* 阈值下限（防止过暗） */
#define IMAGE_ADAPTIVE_THRESHOLD_TOP_HIGH (200U) /* 阈值上限（防止过亮） */
#define IMAGE_ADAPTIVE_THRESHOLD_TOP_Y (0) /* 预留参数：上部区域分界行 */
#define IMAGE_TARGET_PIXEL_X (94.0f) /* 目标投影点X坐标，左上角为原点 */
#define IMAGE_TARGET_PIXEL_Y (100.0f) /* 目标投影点Y坐标，左上角为原点 */
#define IMAGE_TARGET_CENTER_X (((float)MT9V03X_W * 0.5f) - IMAGE_TARGET_PIXEL_X) /* 目标投影点X中心系偏移 */
#define IMAGE_TARGET_CENTER_Y (IMAGE_TARGET_PIXEL_Y - ((float)MT9V03X_H * 0.5f)) /* 目标投影点Y中心系偏移 */

/* 内部完整单帧灰度图缓存，单位像素灰度值 */
static uint8 s_image_frame[MT9V03X_H][MT9V03X_W];
/* 内部默认二值化图像缓存，单位像素灰度值 */
static uint8 s_image_binary[MT9V03X_H][MT9V03X_W];
/* 内部默认固定二值化阈值偏移，单位灰度级 */
static int16 s_image_threshold = 0;
/* 积分图缓存，用于加速自适应二值化 */
static uint32 integral_image[MT9V03X_H + 1][MT9V03X_W + 1];
/* 当前帧白色圆形目标检测结果数组，数组按照连通域面积从大到小排序，坐标原点位于图像中心 */
image_circle g_image_circles[IMAGE_MAX_CIRCLE_COUNT] = {0};
/* 当前帧有效圆形目标数量 */
uint8 g_image_circle_count = 0U;

/*
 * 函数功能：尝试抓取最新完整图像并锁存到内部缓存。
 * 输入参数：无。
 * 返回值：1-本次成功获取到新图像；0-当前没有新的完整图像。
 */
static uint8 image_latch_frame(void)
{
    if (0U == mt9v03x_finish_flag)
    {
        return 0U;
    }

    /* 锁存完整帧，避免后续处理直接读取DMA工作缓冲导致撕裂 */
    memcpy(s_image_frame[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    mt9v03x_finish_flag = 0U;
    return 1U;
}

/*
 * 函数功能：使用快速滑动窗口局部阈值算法完成二值化，适用于高亮目标提取。
 * 输入参数：无。
 * 返回值：无。
 */
static void image_fast_adaptive_threshold(void)
{
    const uint8 *img_data = s_image_frame[0];
    uint8 *output_data = s_image_binary[0];
    const uint16 width = MT9V03X_W;
    const uint16 height = MT9V03X_H;
    const uint8 block = IMAGE_ADAPTIVE_THRESHOLD_BLOCK;
    const int16 clip_value = s_image_threshold;
    const uint8 clip_value2 = IMAGE_ADAPTIVE_THRESHOLD_TOP_LOW;
    const uint8 clip_value3 = IMAGE_ADAPTIVE_THRESHOLD_TOP_HIGH;
    const int16 upper_limit_y = IMAGE_ADAPTIVE_THRESHOLD_TOP_Y;
    int32 half_block;
    int32 x;
    int32 y;
    int32 sum;
    int32 threshold;
    int32 pixel_threshold;
    uint32 pixel_index;
    int32 x1;
    int32 y1;
    int32 x2;
    int32 y2;
    int32 block_area;

    if ((0 == img_data) || (0 == output_data))
    {
        return;
    }
    if ((0U == block) || (0U == (block & 1U)))
    {
        return;
    }
    if (((int32)width > MT9V03X_W) || ((int32)block > (int32)width) || ((int32)block > (int32)height))
    {
        return;
    }
    if ((clip_value < -127) || (clip_value > 127))
    {
        return;
    }
    if (clip_value2 > clip_value3)
    {
        return;
    }
    (void)upper_limit_y;

    half_block = (int32)block / 2;

    /* 第一步：构建积分图，便于O(1)求任意窗口和 */
    for (y = 0; y <= (int32)height; y++)
    {
        integral_image[y][0] = 0U;
    }
    for (x = 0; x <= (int32)width; x++)
    {
        integral_image[0][x] = 0U;
    }
    for (y = 1; y <= (int32)height; y++)
    {
        for (x = 1; x <= (int32)width; x++)
        {
            integral_image[y][x] = (uint32)img_data[((y - 1) * (int32)width) + (x - 1)] +
                                   integral_image[y - 1][x] +
                                   integral_image[y][x - 1] -
                                   integral_image[y - 1][x - 1];
        }
    }

    /* 第二步：积分图窗口均值阈值二值化 */
    for (y = 0; y < (int32)height; y++)
    {
        for (x = 0; x < (int32)width; x++)
        {
            x1 = x - half_block;
            y1 = y - half_block;
            x2 = x + half_block;
            y2 = y + half_block;

            if (x1 < 0)
            {
                x1 = 0;
            }
            if (y1 < 0)
            {
                y1 = 0;
            }
            if (x2 >= (int32)width)
            {
                x2 = (int32)width - 1;
            }
            if (y2 >= (int32)height)
            {
                y2 = (int32)height - 1;
            }

            sum = (int32)integral_image[y2 + 1][x2 + 1] -
                  (int32)integral_image[y1][x2 + 1] -
                  (int32)integral_image[y2 + 1][x1] +
                  (int32)integral_image[y1][x1];

            block_area = ((x2 - x1) + 1) * ((y2 - y1) + 1);
            threshold = sum / block_area;
            pixel_threshold = threshold - clip_value;
            if (pixel_threshold < (int32)clip_value2)
            {
                pixel_threshold = (int32)clip_value2;
            }
            else if (pixel_threshold > (int32)clip_value3)
            {
                pixel_threshold = (int32)clip_value3;
            }

            pixel_index = (uint32)((y * (int32)width) + x);
            if (img_data[pixel_index] >= IMAGE_ADAPTIVE_THRESHOLD_BRIGHT)
            {
                output_data[pixel_index] = 255U;
            }
            else
            {
                output_data[pixel_index] = (img_data[pixel_index] > (uint8)(pixel_threshold + 10)) ? 255U : 0U;
            }
        }
    }
}

/*
 * 函数功能：对内部灰度图执行固定阈值二值化，生成默认二值图缓存。
 * 输入参数：无。
 * 返回值：无。
 */
static void image_binary_threshold_fixed(void)
{
    uint32 i;

    /* 单次遍历完成固定阈值二值化，优先保证实时性 */
    for (i = 0U; i < MT9V03X_IMAGE_SIZE; i++)
    {
        s_image_binary[0][i] = (s_image_frame[0][i] > s_image_threshold) ? 255U : 0U;
    }
}

/*
 * 函数功能：对内部灰度图执行二值化，生成默认二值图缓存。
 * 输入参数：无。
 * 返回值：无。
 */
static void image_binary_threshold(void)
{
    image_fast_adaptive_threshold(); /* 积分图加速的局部自适应阈值二值化 */

    //image_binary_threshold_fixed();

}
/*
 * 函数功能：基于二值化图像计算白色圆形目标的圆心与半径。
 * 输入参数：无。
 * 返回值：无。
 */
static void image_clear_circle_results(void)
{
    memset(g_image_circles, 0, sizeof(g_image_circles));
    g_image_circle_count = 0U;
}

/**
 * @brief 两遍扫描法寻找4面连通域
 * */
static void image_find_connected_components(void)
{
    enum { IMAGE_MAX_LABELS = (MT9V03X_IMAGE_SIZE / 2U) + 2U };
    /* 并查集父节点表：用于合并等价标签 */
    static uint16 s_parent[IMAGE_MAX_LABELS];
    /* 连通域横坐标累计和：用于后续计算质心 */
    static uint32 s_sum_x[IMAGE_MAX_LABELS];
    /* 连通域纵坐标累计和：用于后续计算质心 */
    static uint32 s_sum_y[IMAGE_MAX_LABELS];
    /* 连通域像素计数：用于面积排序与半径估计 */
    static uint16 s_count[IMAGE_MAX_LABELS];
    uint16 prev_row[MT9V03X_W];
    uint16 curr_row[MT9V03X_W];
    uint16 top_labels[IMAGE_MAX_CIRCLE_COUNT];
    uint16 top_counts[IMAGE_MAX_CIRCLE_COUNT];
    uint16 next_label = 1U;
    uint16 repro_label = 1U;
    uint16 left_label;
    uint16 up_label;
    uint16 root_left;
    uint16 root_up;
    uint16 current_label;
    uint16 temp_label;
    uint16 next_parent;
    uint16 label;
    uint16 insert_slot;
    uint16 move_slot;
    uint32 row;
    uint32 col;

    /* 清空上一帧检测结果并初始化行标签缓存 */
    image_clear_circle_results();
    memset(prev_row, 0, sizeof(prev_row));
    memset(curr_row, 0, sizeof(curr_row));
    /* 第一遍扫描，为每个连通域分配标签 */
    for (row = 0U; row < MT9V03X_H; row++)
    {
        for (col = 0U; col < MT9V03X_W; col++)
        {
            if (0U == s_image_binary[row][col])
            {
                curr_row[col] = 0U;
                continue;
            }
            left_label = (0U == col) ? 0U : curr_row[col - 1U];
            up_label = prev_row[col];
            if ((0U == left_label) && (0U == up_label))
            {
                if (next_label >= IMAGE_MAX_LABELS)
                {
                    return;
                }
                s_parent[next_label] = next_label;
                curr_row[col] = next_label;
                next_label++;
            }
            else if (0U == left_label)
            {
                curr_row[col] = up_label;
            }
            else if (0U == up_label)
            {
                curr_row[col] = left_label;
            }
            else
            {
                /* 查找 left 标签根并进行路径压缩 */
                root_left = left_label;
                while (s_parent[root_left] != root_left)
                {
                    root_left = s_parent[root_left];
                }
                temp_label = left_label;
                while (s_parent[temp_label] != temp_label)
                {
                    next_parent = s_parent[temp_label];
                    s_parent[temp_label] = root_left;
                    temp_label = next_parent;
                }

                /* 查找 up 标签根并进行路径压缩 */
                root_up = up_label;
                while (s_parent[root_up] != root_up)
                {
                    root_up = s_parent[root_up];
                }
                temp_label = up_label;
                while (s_parent[temp_label] != temp_label)
                {
                    next_parent = s_parent[temp_label];
                    s_parent[temp_label] = root_up;
                    temp_label = next_parent;
                }

                /* 合并两棵等价树，使用较小根作为统一标签 */
                if (root_left < root_up)
                {
                    s_parent[root_up] = root_left;
                    curr_row[col] = root_left;
                }
                else
                {
                    s_parent[root_left] = root_up;
                    curr_row[col] = root_up;
                }
            }
        }
        memcpy(prev_row, curr_row, sizeof(prev_row));
    }
    /* 第二遍扫描，为每个连通域分配标签 */
    for (label = 1U; label < next_label; label++)
    {
        /* 将每个标签直接压缩到最终根，便于后续快速统计 */
        current_label = label;
        while (s_parent[current_label] != current_label)
        {
            current_label = s_parent[current_label];
        }
        temp_label = label;
        while (s_parent[temp_label] != temp_label)
        {
            next_parent = s_parent[temp_label];
            s_parent[temp_label] = current_label;
            temp_label = next_parent;
        }
    }
    memset(s_sum_x, 0, (uint32)next_label * sizeof(s_sum_x[0]));
    memset(s_sum_y, 0, (uint32)next_label * sizeof(s_sum_y[0]));
    memset(s_count, 0, (uint32)next_label * sizeof(s_count[0]));
    memset(top_labels, 0, sizeof(top_labels));
    memset(top_counts, 0, sizeof(top_counts));
    memset(prev_row, 0, sizeof(prev_row));
    memset(curr_row, 0, sizeof(curr_row));
    /* 重新扫描图像，按最终根标签累计面积与质心统计量 */
    for (row = 0U; row < MT9V03X_H; row++)
    {
        for (col = 0U; col < MT9V03X_W; col++)
        {
            if (0U == s_image_binary[row][col])
            {
                curr_row[col] = 0U;
                continue;
            }
            left_label = (0U == col) ? 0U : curr_row[col - 1U];
            up_label = prev_row[col];
            if ((0U == left_label) && (0U == up_label))
            {
                current_label = s_parent[repro_label];
                repro_label++;
            }
            else if (0U == left_label)
            {
                current_label = up_label;
            }
            else if (0U == up_label)
            {
                current_label = left_label;
            }
            else
            {
                current_label = (left_label < up_label) ? left_label : up_label;
            }
            curr_row[col] = current_label;
            s_count[current_label]++;
            s_sum_x[current_label] += col;
            s_sum_y[current_label] += row;
        }
        memcpy(prev_row, curr_row, sizeof(prev_row));
    }
    /* 将连通域按像素数量降序插入，保留前 IMAGE_MAX_CIRCLE_COUNT 个 */
    for (label = 1U; label < next_label; label++)
    {
        /* 过滤掉面积小于阈值的连通域 */
        if ((0U == s_count[label]) || (s_count[label] < IMAGE_MIN_COMPONENT_AREA))
        {
            continue;
        }
        for (insert_slot = 0U; insert_slot < IMAGE_MAX_CIRCLE_COUNT; insert_slot++)
        {
            if (s_count[label] > top_counts[insert_slot])
            {
                for (move_slot = IMAGE_MAX_CIRCLE_COUNT - 1U; move_slot > insert_slot; move_slot--)
                {
                    top_counts[move_slot] = top_counts[move_slot - 1U];
                    top_labels[move_slot] = top_labels[move_slot - 1U];
                }
                top_counts[insert_slot] = s_count[label];
                top_labels[insert_slot] = label;
                break;
            }
        }
    }

    /* 输出结果：坐标换算到图像中心坐标系，并由面积近似半径 */
    for (insert_slot = 0U; insert_slot < IMAGE_MAX_CIRCLE_COUNT; insert_slot++)
    {
        if (0U == top_counts[insert_slot])
        {
            break;
        }
        g_image_circles[insert_slot].x =
            (((float)MT9V03X_W * 0.5f) - ((float)s_sum_x[top_labels[insert_slot]] / (float)top_counts[insert_slot])) - IMAGE_TARGET_CENTER_X;
        g_image_circles[insert_slot].y =
            (((float)s_sum_y[top_labels[insert_slot]] / (float)top_counts[insert_slot]) - ((float)MT9V03X_H * 0.5f)) - IMAGE_TARGET_CENTER_Y;
        g_image_circles[insert_slot].radius = sqrtf((float)top_counts[insert_slot] / 3.1415926f);
        g_image_circles[insert_slot].valid = 1U;
    }
    g_image_circle_count = (uint8)insert_slot;
}


/*
 * 函数功能：初始化图像模块，完成摄像头、内部缓存和默认阈值初始化。
 * 输入参数：无。
 * 返回值：无。
 */
void image_init(void)
{
    memset(s_image_frame, 0, sizeof(s_image_frame));
    memset(s_image_binary, 0, sizeof(s_image_binary));
    memset(g_image_circles, 0, sizeof(g_image_circles));
    g_image_circle_count = 0U;

    s_image_threshold = IMAGE_DEFAULT_THRESHOLD;
    mt9v03x_finish_flag = 0U;

    (void)mt9v03x_init();
}

/*
 * 函数功能：更新图像模块，获取最新完整图像，缓存到内部缓冲区，完成二值化，并发送原始图像到上位机。
 * 输入参数：无。
 * 返回值：无。
 */
void image_update(void)
{

    if (0U == image_latch_frame())
    {
        return;
    }

    image_binary_threshold();
    image_find_connected_components(); /* 利用两遍扫描法寻找4面连通域 */


}
