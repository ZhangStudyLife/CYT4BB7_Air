#include "IMU_TOP.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <math.h>

/* ======================== IMU 全局状�?======================== */
/* IMU滤波数据通过 g_imufilter_1000hz (IMU_Filtter.h) 访问 */
MahonyAhrs_t g_mahony_ahrs;       /* Mahony 姿态解算器状�?*/
MahonyAhrs_Euler_t g_euler;       /* 当前姿态欧拉角（度�?*/
uint8 g_imu_ready = 0U;           /* IMU 是否完成初始化与自检 */
uint32 g_imu_update_count = 0U;   /* 1kHz ���¼��� */
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
	g_imu_update_count = 0U;

	/* 步骤1: 上电初始�?ICM42688 驱动 */
	ICM42688_Init(&ICM42688_CONFIG);

	/* 步骤2: 上电自检（必须静止放置） */
	if (0U == IMU_Startup_SelfCheck())
	{
		printf("IMU startup self-check failed.\r\n");
		while (1)
		{
		}
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

void IMU_SelectAhrsInput(float *gx, float *gy, float *gz,
								float *ax, float *ay, float *az)
{
	float cal_gx = 0.0f;
	float cal_gy = 0.0f;
	float cal_gz = 0.0f;
	float cal_ax = 0.0f;
	float cal_ay = 0.0f;
	float cal_az = 0.0f;
	float accel_norm_g;

	if ((gx == 0) || (gy == 0) || (gz == 0) ||
		(ax == 0) || (ay == 0) || (az == 0))
	{
		return;
	}

	/* 默认使用500Hz滤波值，保证启动与未校准阶段正常工作 */
	*gx = g_imudata_500hz.gyrox;
	*gy = g_imudata_500hz.gyroy;
	*gz = g_imudata_500hz.gyroz;
	*ax = g_imudata_500hz.accx;
	*ay = g_imudata_500hz.accy;
	*az = g_imudata_500hz.accz;

	/* 校准未完成、校准忙、或本帧校准数据无效时，不切换姿态输入源 */
	if ((0U == AccelCalibration_IsCalibrated()) ||
		(0U == AccelCalibration_IsRealtimeDataValid()) ||
		(0U != IMUCalib_IsBusy()))
	{
		return;
	}

	AccelCalibration_GetBodyGyroDps(&cal_gx, &cal_gy, &cal_gz);
	AccelCalibration_GetCorrectedSpecificForceG(&cal_ax, &cal_ay, &cal_az);

	if ((0U == IMU_IsFiniteFloat(cal_gx)) || (0U == IMU_IsFiniteFloat(cal_gy)) || (0U == IMU_IsFiniteFloat(cal_gz)) ||
		(0U == IMU_IsFiniteFloat(cal_ax)) || (0U == IMU_IsFiniteFloat(cal_ay)) || (0U == IMU_IsFiniteFloat(cal_az)))
	{
		return;
	}

	accel_norm_g = sqrtf(cal_ax * cal_ax + cal_ay * cal_ay + cal_az * cal_az);
	if ((accel_norm_g < 0.30f) || (accel_norm_g > 1.70f))
	{
		return;
	}

	/* 姿态解算优先使用校准后机体系数据，减小长时静止漂移 */
	*gx = cal_gx;
	*gy = cal_gy;
	*gz = cal_gz;
	*ax = cal_ax;
	*ay = cal_ay;
	*az = cal_az;
}

/* �������ܣ�IMU 1kHz ������ڣ���ɲ������˲�����̬���¡�����ֵ���ޡ� */
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
	AccelCalibration_Update_1000HZ();

	/* 5. 姿态解算 */
	IMU_SelectAhrsInput(&ahrs_gx, &ahrs_gy, &ahrs_gz, &ahrs_ax, &ahrs_ay, &ahrs_az);
	MahonyAhrs_Update(
		&g_mahony_ahrs,
		ahrs_gx, ahrs_gy, ahrs_gz,
		ahrs_ax, ahrs_ay, ahrs_az,
		dt_s);

	/* 步骤5: 计算欧拉角（单位: 度）并缓�?*/
	g_euler = MahonyAhrs_GetEulerDegrees(&g_mahony_ahrs);
	g_imu_update_count++;
	            wifi_justfloat(tick_1000us_cnt,ICM42688.gyro_x, ICM42688.gyro_y, ICM42688.gyro_z,ICM42688.acc_x, ICM42688.acc_y, ICM42688.acc_z);
}

uint8 IMU_Is_Ready(void)
{
	return g_imu_ready;
}

