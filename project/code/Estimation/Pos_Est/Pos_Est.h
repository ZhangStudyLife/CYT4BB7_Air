#ifndef POS_EST_H
#define POS_EST_H

#include <stdint.h>
#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEG2RAD                (0.017453292519943295f)  /* 角度转弧度系数 */
#define POS_EST_RAD2DEG        (57.29577951308232f)     /* 弧度转角度系数 */
#define POS_EST_SQUAL_MIN      (25U)                    /* 旧光流链路可用性的最小 SQUAL */
#define RESOLUTION             (0.2131946f)             /* 1m 高度下 1 像素对应位移，单位 cm */
#define POS_EST_100HZ_DT       (0.01f)                  /* 旧光流链路默认慢环周期，单位 s */
#define POS_EST_250HZ_DT       (0.004f)                 /* 默认 250Hz 周期，单位 s */
#define POS_EST_VEL_LIMIT      (200.0f)                 /* 速度限幅，单位 cm/s */

/**
 * @brief 并行融合模式枚举
 *        0=纯惯导死推，1=光流恢复阶段，2=光流稳定融合阶段。
 */
typedef enum
{
    POS_EST_FUSION_MODE_DEAD_RECKONING = 0,
    POS_EST_FUSION_MODE_RECOVERING     = 1,
    POS_EST_FUSION_MODE_FLOW_HOLD      = 2
} PosEstFusionMode_e;

typedef struct opFlow_s
{
    float pixSum[2];             /* 像素累计量 */
    float pixComp[2];            /* 像素补偿量 */
    float pixValid[2];           /* 当前有效像素量 */
    float pixValidLast[2];       /* 上一帧有效像素量 */

    float deltaPos[2];           /* 单帧位移，单位 cm */
    float deltaVel[2];           /* 单帧速度，单位 cm/s */
    float posSum[2];             /* 光流累计位移，单位 cm */
    float velLpf[2];             /* 旧链路输出速度，单位 cm/s */

    uint8 isOpFlowOk;            /* 旧链路光流可用标志 */
    uint8 isDataValid;           /* 数据有效标志 */
} opFlow_t;

typedef struct
{
    float vAccDeadband;          /* 加速度死区，单位 cm/s^2 */
    float accBias[3];            /* 加速度偏置，单位 cm/s^2 */
    float acc[3];                /* 融合使用加速度，单位 cm/s^2 */
    float vel[3];                /* 融合速度，单位 cm/s */
    float pos[3];                /* 融合位置，单位 cm */
} estimator_t;

/**
 * @brief 并行水平速度融合输出
 *        索引 0 对应机体右向速度通道（roll 控制通道），
 *        索引 1 对应机体前向速度通道（pitch 控制通道）。
 */
typedef struct
{
    float vel_flow_cmps[2];      /* 光流速度观测，单位 cm/s */
    float vel_fused_cmps[2];     /* 融合速度输出，单位 cm/s */
    float innovation_cmps[2];    /* 光流速度新息，单位 cm/s */
    float acc_bias_cmpss[2];     /* 在线估计加速度偏置，单位 cm/s^2 */
    uint8 flow_valid;            /* 当前光流是否可用于融合 */
    uint8 fusion_mode;           /* PosEstFusionMode_e */
    float dead_reckon_time_s;    /* 连续死推时长，单位 s */
} PosEstXYOutput_t;

extern volatile opFlow_t opFlow;                   /* 旧链路光流输出 */
extern estimator_t estimator;                      /* 当前融合状态 */
extern volatile PosEstXYOutput_t g_pos_est_xy_output; /* 并行融合输出 */

/**
 * @brief  位置估计模块初始化
 *         清零旧光流链路、并行融合状态以及调试输出。
 *
 * @param  无
 * @return 无
 */
void Pos_Est_Init(void);

/**
 * @brief  1000Hz 姿态补偿辅助更新
 *         累积陀螺数据，供旧光流链路做慢环姿态补偿。
 *
 * @param  无
 * @return 无
 */
void Pos_Est_Update_1000HZ(void);

/**
 * @brief  250Hz 水平加速度预测与并行融合更新
 *         基于滤波后的水平线性加速度执行惯导预测，
 *         再使用最新光流观测做 INAV 风格速度/位置校正。
 *
 * @param  无
 * @return 无
 */
void Pos_Est_Update_250HZ(void);

/**
 * @brief  慢环光流更新
 *         保持旧光流速度链路不变，同时刷新并行融合所需的
 *         光流观测、可用性、恢复增益与调试状态。
 *
 * @param  无
 * @return 无
 */
void Pos_Est_Update_100HZ(void);

#ifdef __cplusplus
}
#endif

#endif
