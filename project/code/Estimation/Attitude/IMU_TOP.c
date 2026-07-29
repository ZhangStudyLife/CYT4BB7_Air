#include "IMU_TOP.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <math.h>
#include "../code/HW_Drivers/ICM42688_Aux/ICM42688_Aux.h"
#include "../Height_Est/Height_Est.h"
/* ======================== IMU 全局状�?======================== */
/* IMU滤波数据通过 g_imufilter_1000hz (IMU_Filtter.h) 访问 */
MahonyAhrs_t g_mahony_ahrs;					   /* Mahony 姿态解算器状�?*/
MahonyAhrs_Euler_t g_euler;					   /* 当前姿态欧拉角（度�?*/
uint8 g_imu_ready = 0U;						   /* IMU 是否完成初始化与自检 */
static imudata_t s_imu_raw_calib_1000hz = {0}; /* 当前 1kHz 原始 IMU 快照，供校准链读取 */
static uint8 s_imu_initializing = 0U;
extern uint32 tick_1000us_cnt;
/* ======================== 本地工具函数 ======================== */
static uint8 IMU_IsFiniteFloat(float value)
{
	if (value != value)
	{
		return 0U;
	}

	if ((value > 1000000.0f) || (value < -1000000.0f))
	{
		return 0U;
	}

	return 1U;
}

/*
 * 函数功能: 读取当前 1kHz 周期内供校准使用的原始 IMU 物理量快照。
 * 输入参数:
 *   gx, gy, gz - 输出陀螺仪原始角速度，单位 dps；已做符号映射并扣除陀螺仪零偏
 *   ax, ay, az - 输出加速度计原始比力，单位 g；仅做量程换算与符号映射
 * 输出参数/返回值:
 *   通过指针返回当前帧原始 IMU 快照；空指针会被忽略
 */
void IMU_GetRawSampleForCalibration(float *gx, float *gy, float *gz,
									float *ax, float *ay, float *az)
{
	if (gx != NULL)
	{
		*gx = s_imu_raw_calib_1000hz.gyrox;
	}
	if (gy != NULL)
	{
		*gy = s_imu_raw_calib_1000hz.gyroy;
	}
	if (gz != NULL)
	{
		*gz = s_imu_raw_calib_1000hz.gyroz;
	}
	if (ax != NULL)
	{
		*ax = s_imu_raw_calib_1000hz.accx;
	}
	if (ay != NULL)
	{
		*ay = s_imu_raw_calib_1000hz.accy;
	}
	if (az != NULL)
	{
		*az = s_imu_raw_calib_1000hz.accz;
	}
}

/*
 * 函数功能: 上电读取短窗口数据做基础健康检�? * 检查项  :
 *   1) 数据是否有效（无 NaN/异常值）
 *   2) 静止时平均角速度是否在合理范�? *   3) 加速度模长均值是否接�?1g
 * 返回�? : 1=通过�?=失败
 */
static uint8 IMU_Startup_SelfCheck(void)
{
	uint32 i;
	float gyro_abs_sum = 0.0f;
	float acc_mag_sum = 0.0f;

	for (i = 0U; i < IMU_SELFTEST_SAMPLE_COUNT; i++)
	{
		float gyro_abs;
		float acc_mag;

		ICM42688_Get_Data();

		if ((0U == IMU_IsFiniteFloat(ICM42688.gyro_x)) ||
			(0U == IMU_IsFiniteFloat(ICM42688.gyro_y)) ||
			(0U == IMU_IsFiniteFloat(ICM42688.gyro_z)) ||
			(0U == IMU_IsFiniteFloat(ICM42688.acc_x)) ||
			(0U == IMU_IsFiniteFloat(ICM42688.acc_y)) ||
			(0U == IMU_IsFiniteFloat(ICM42688.acc_z)))
		{
			return 0U;
		}

		gyro_abs = sqrtf(ICM42688.gyro_x * ICM42688.gyro_x +
						 ICM42688.gyro_y * ICM42688.gyro_y +
						 ICM42688.gyro_z * ICM42688.gyro_z);

		acc_mag = sqrtf(ICM42688.acc_x * ICM42688.acc_x +
						ICM42688.acc_y * ICM42688.acc_y +
						ICM42688.acc_z * ICM42688.acc_z);

		gyro_abs_sum += gyro_abs;
		acc_mag_sum += acc_mag;

		system_delay_us(ICM42688_SAMPLE_INTERVAL_US);
	}

	gyro_abs_sum /= (float)IMU_SELFTEST_SAMPLE_COUNT;
	acc_mag_sum /= (float)IMU_SELFTEST_SAMPLE_COUNT;

	if (gyro_abs_sum > IMU_SELFTEST_GYRO_MEAN_MAX_DPS)
	{
		return 0U;
	}

	if ((acc_mag_sum < IMU_SELFTEST_ACC_MIN_G) || (acc_mag_sum > IMU_SELFTEST_ACC_MAX_G))
	{
		return 0U;
	}

	return 1U;
}

/* ======================== IMU 初始�?======================== */
void IMU_Init_All(void)
{
	uint32 i;

	g_imu_ready = 0U;
	s_imu_initializing = 1U;
	s_imu_raw_calib_1000hz = (imudata_t){0};

	/* 步骤1: 上电初始�?ICM42688 驱动 */
	ICM42688_Init(&ICM42688_CONFIG);

	/* 步骤2: 上电自检（必须静止放置） */
	if (0U == IMU_Startup_SelfCheck())
	{
		printf("IMU startup self-check failed.\r\n");
	}

	/* 步骤3: 初始化上层滤波与姿态解算器 */
	IMUFilter_Init();
	MahonyAhrs_Init(&g_mahony_ahrs);
	g_euler.roll = 0.0f;
	g_euler.pitch = 0.0f;
	g_euler.yaw = 0.0f;
	g_euler.sin_roll = 0.0f;
	g_euler.cos_roll = 1.0f;
	g_euler.sin_pitch = 0.0f;
	g_euler.cos_pitch = 1.0f;

	/* 步骤4: 暖机，丢弃前若干帧用于稳定滤波器内部状�?*/
	for (i = 0U; i < IMU_WARMUP_DISCARD_SAMPLES; i++)
	{
		IMU_Update_1000HZ();
		system_delay_us(ICM42688_SAMPLE_INTERVAL_US);
	}

	g_imu_ready = 1U;
	s_imu_initializing = 0U;
}

/* IMU 1kHz*/
// 04110147 zyz实际测试执行一次IMU_Update_1000HZ花费55us
void IMU_Update_1000HZ(void)
{
	/* 步骤1: �?ICM42688 读取一帧原始传感器数据 */
	const float dt_s = IMU_UPDATE_DT_SEC;
	float ahrs_gx;
	float ahrs_gy;
	float ahrs_gz;
	float ahrs_ax;
	float ahrs_ay;
	float ahrs_az;
	if ((0U == g_imu_ready) && (0U == s_imu_initializing))
	{
		return;
	}

	/* 1. 原始数据 (gyro已去零偏) */
	ICM42688_Get_Data();

	/* 缓存当前帧原始 IMU 物理量，供校准流程直接读取 */
	s_imu_raw_calib_1000hz.gyrox = ICM42688.gyro_x;
	s_imu_raw_calib_1000hz.gyroy = ICM42688.gyro_y;
	s_imu_raw_calib_1000hz.gyroz = ICM42688.gyro_z;
	s_imu_raw_calib_1000hz.accx = ICM42688.acc_x;
	s_imu_raw_calib_1000hz.accy = ICM42688.acc_y;
	s_imu_raw_calib_1000hz.accz = ICM42688.acc_z;

	/* 2. 加速度计校准前置（传感器坐标系） */
	float cal_ax = ICM42688.acc_x;
	float cal_ay = ICM42688.acc_y;
	float cal_az = ICM42688.acc_z;
	AccelCalibration_ApplySensorCorrection(&cal_ax, &cal_ay, &cal_az);

	/* 3. 校准后数据送入滤波器 */
	IMUFilter_Update(ICM42688.gyro_x, ICM42688.gyro_y, ICM42688.gyro_z,
					 cal_ax, cal_ay, cal_az);

	/* 4. 校准状态机 + 高级处理（当前帧） */
	IMUCalib_Update_1000HZ();

	/* 5. 姿态解算统一使用 1kHz 滤波 IMU 输出 */
	ahrs_gx = g_imufilter_1000hz.gyrox;
	ahrs_gy = g_imufilter_1000hz.gyroy;
	ahrs_gz = g_imufilter_1000hz.gyroz;
	ahrs_ax = g_imufilter_1000hz.accx;
	ahrs_ay = g_imufilter_1000hz.accy;
	ahrs_az = g_imufilter_1000hz.accz;

	if ((0U != IMU_IsFiniteFloat(ahrs_gx)) &&
		(0U != IMU_IsFiniteFloat(ahrs_gy)) &&
		(0U != IMU_IsFiniteFloat(ahrs_gz)) &&
		(0U != IMU_IsFiniteFloat(ahrs_ax)) &&
		(0U != IMU_IsFiniteFloat(ahrs_ay)) &&
		(0U != IMU_IsFiniteFloat(ahrs_az)))
	{
		MahonyAhrs_Update(
			&g_mahony_ahrs,
			ahrs_gx, ahrs_gy, ahrs_gz,
			ahrs_ax, ahrs_ay, ahrs_az,
			dt_s);
	}

	/* 步骤5: 计算欧拉角（单位: 度）并缓�?*/
	g_euler = MahonyAhrs_GetEulerDegrees(&g_mahony_ahrs);
	AccelCalibration_Update_1000HZ();
	Height_Est_predict_1000HZ();

	wifi_justfloat(g_euler.roll, g_euler.pitch, g_euler.yaw,
		g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy, g_imufilter_1000hz.gyroz,
		g_imufilter_1000hz.accx, g_imufilter_1000hz.accy, g_imufilter_1000hz.accz,
		ICM42688.gyro_x, ICM42688.gyro_y, ICM42688.gyro_z,
		ICM42688.acc_x, ICM42688.acc_y, ICM42688.acc_z);
}

uint8 IMU_Is_Ready(void)
{
	return g_imu_ready;
}
