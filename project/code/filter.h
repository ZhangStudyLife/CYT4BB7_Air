#ifndef FILTER_H
#define FILTER_H

#include "zf_common_headfile.h"
#include <math.h>

/* === 窗口大小宏 === */
#define FILT_MA_WIN_MAX     (32U)
#define FILT_MEDIAN_WIN_MAX (9U)

/* === LPF1 一阶低通 === */
typedef struct {
    float alpha;  /* 平滑系数 0,1，越小越平滑 */
    float state;  /* 上一帧输出 */
} LPF1_t;

void  LPF1_Init(LPF1_t *f, float alpha);
float LPF1_Update(LPF1_t *f, float z);
void  LPF1_Reset(LPF1_t *f);

/* === Kalman1D 一维卡尔曼 === */
typedef struct {
    float q;   /* 过程噪声方差 */
    float r;   /* 测量噪声方差 */
    float x;   /* 状态估计 */
    float p;   /* 估计误差协方差 */
    float p0;  /* 初始协方差（Reset用） */
    float x0;  /* 初始状态（Reset用） */
} Kalman1D_t;

void  Kalman1D_Init(Kalman1D_t *f, float q, float r, float p0, float x0);
float Kalman1D_Update(Kalman1D_t *f, float z);
void  Kalman1D_Reset(Kalman1D_t *f);

/* === MovAvg 滑动均值 === */
typedef struct {
    float buf[FILT_MA_WIN_MAX];
    float sum;
    uint8 win;    /* 实际窗口大小 */
    uint8 idx;    /* 写指针 */
    uint8 count;  /* 已填充样本数 */
} MovAvg_t;

void  MovAvg_Init(MovAvg_t *f, uint8 window);
float MovAvg_Update(MovAvg_t *f, float z);
void  MovAvg_Reset(MovAvg_t *f);

/* === Median 中值滤波 === */
typedef struct {
    float buf[FILT_MEDIAN_WIN_MAX];
    float tmp[FILT_MEDIAN_WIN_MAX];  /* 排序临时数组 */
    uint8 win;
    uint8 idx;
    uint8 count;
} Median_t;

void  Median_Init(Median_t *f, uint8 window);
float Median_Update(Median_t *f, float z);
void  Median_Reset(Median_t *f);

/* === StepLim 步进限幅 === */
typedef struct {
    float step;   /* 每帧最大允许变化量（绝对值） */
    float state;  /* 上一帧输出 */
    uint8 seeded; /* 是否已有第一次有效输出，0时直通第一个值 */
} StepLim_t;

void  StepLim_Init(StepLim_t *f, float step);
float StepLim_Update(StepLim_t *f, float z);
void  StepLim_Reset(StepLim_t *f);

/* === Biquad 二阶IIR === */
typedef enum {
    BIQUAD_LPF   = 0,
    BIQUAD_HPF   = 1,
    BIQUAD_NOTCH = 2,
} BiquadType_e;

typedef struct {
    float b0, b1, b2;  /* 分子系数 */
    float a1, a2;       /* 分母系数（已归一化，符号：y = b*x - a*y_prev） */
    float d1, d2;       /* Direct Form II Transposed 内部状态 */
} Biquad_t;

void  Biquad_Init(Biquad_t *f, BiquadType_e type, float fs, float fc, float q);
float Biquad_Update(Biquad_t *f, float z);
void  Biquad_Reset(Biquad_t *f);

#endif /* FILTER_H */
