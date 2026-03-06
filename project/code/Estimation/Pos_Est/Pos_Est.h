#ifndef POS_EST_H
#define POS_EST_H

#include <stdint.h>
#include "zf_common_headfile.h"
#ifdef __cplusplus
extern "C" {
#endif


#define DEG2RAD (0.017453292519943295f)  /* 角度转弧度的常数 */
#define POS_EST_RAD2DEG (57.29577951308232f)    /* 弧度转角度的常数 */
#define POS_EST_SQUAL_MIN 25U              /* 光流有效的最小表面质量指标 */
#define RESOLUTION			(0.2131946f)/*1m高度下 1个像素对应的位移，单位cm*/
#define POS_EST_100HZ_DT (0.01f) /* 100Hz周期的时间间隔，单位秒 */
#define POS_EST_250HZ_DT (0.004f) /* 250Hz周期的时间间隔，单位秒 */
#define POS_EST_VEL_LIMIT (100.0f) /* 速度限幅，单位cm/s */

typedef struct opFlow_s 
{
	float pixSum[2];		/*累积像素*/
	float pixComp[2];		/*像素补偿*/
	float pixValid[2];		/*有效像素*/
	float pixValidLast[2];	/*上一次有效像素*/
	
	float deltaPos[2];		/*2帧之间的位移 单位cm*/
	float deltaVel[2];		/*速度 单位cm/s*/
	float posSum[2];		/*累积位移 单位cm*/
	float velLpf[2];		/*速度低通 单位cm/s*/
	
	uint8 isOpFlowOk;		/*光流状态*/
	uint8 isDataValid;		/*数据有效*/

} opFlow_t;


typedef struct
{
	float vAccDeadband; /* 加速度死区 */
	float accBias[3];	/* 加速度 偏置(cm/s/s)*/
	float acc[3];		/* 估测加速度 单位(cm/s/s)*/
	float vel[3];		/* 估测速度 单位(cm/s)*/
	float pos[3]; 		/* 估测位移 单位(cm)*/
} estimator_t;


extern volatile opFlow_t opFlow; /* 光流全局状态，跨周期更新需使用volatile */
extern estimator_t estimator;


/**
 * @brief  位置估计模块初始化
 *         清零所有状态变量、输出和调试结构体
 */
void Pos_Est_Init(void);

/**
 * @brief  2000Hz陀螺仪角度积分更新
 *         在每个2000Hz周期累积 pitch/roll 轴角位移(deg)，
 *         等效于对角速度做矩形窗平均滤波(20点@2000Hz)，
 *         在100Hz周期中读取并重置累积值，供光流姿态解耦补偿使用
 */
void Pos_Est_Update_2000HZ(void);

/**
 * @brief  250Hz加速度计积分更新
 *         执行加速度偏置估计、振动加权、速度/位置积分
 */
void Pos_Est_Update_250HZ(void);

/**
 * @brief  100Hz光流融合更新
 *         读取光流传感器，进行姿态解耦补偿，与加速度计积分融合
 */
void Pos_Est_Update_100HZ(void);

#ifdef __cplusplus
}
#endif

#endif
