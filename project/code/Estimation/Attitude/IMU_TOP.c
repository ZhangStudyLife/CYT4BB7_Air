#include "IMU_TOP.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <math.h>

/* ======================== IMU 全局状态 ======================== */
IMUFilter_t g_imu_filter;         /* IMU 滤波器状态 */
MahonyAhrs_t g_mahony_ahrs;       /* Mahony 姿态解算器状态 */
MahonyAhrs_Euler_t g_euler;       /* 当前姿态欧拉角（度） */
uint8 g_imu_ready = 0U;           /* IMU 是否完成初始化与自检 */
uint32 g_imu_update_count = 0U;   /* 2kHz 更新计数 */
static uint8 s_imu_initializing = 0U;

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
 * 函数功能: 上电读取短窗口数据做基础健康检查
 * 检查项  :
 *   1) 数据是否有效（无 NaN/异常值）
 *   2) 静止时平均角速度是否在合理范围
 *   3) 加速度模长均值是否接近 1g
 * 返回值  : 1=通过，0=失败
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

		system_delay_us(500);
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

/* ======================== IMU 初始化 ======================== */
void IMU_Init_All(void)
{
	uint32 i;

	g_imu_ready = 0U;
	s_imu_initializing = 1U;
	g_imu_update_count = 0U;

	/* 步骤1: 上电初始化 ICM42688 驱动 */
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
	IMUFilter_Init(&g_imu_filter);
	MahonyAhrs_Init(&g_mahony_ahrs);
	g_euler.roll = 0.0f;
	g_euler.pitch = 0.0f;
	g_euler.yaw = 0.0f;
	g_euler.sin_roll = 0.0f;
	g_euler.cos_roll = 1.0f;
	g_euler.sin_pitch = 0.0f;
	g_euler.cos_pitch = 1.0f;

	/* 步骤4: 暖机，丢弃前若干帧用于稳定滤波器内部状态 */
	for (i = 0U; i < IMU_WARMUP_DISCARD_SAMPLES; i++)
	{
		IMU_Update_2kHz();
	}

	g_imu_ready = 1U;
	s_imu_initializing = 0U;
}

void IMU_Update_2kHz(void)
{
	/* 步骤1: 从 ICM42688 读取一帧原始传感器数据 */
	const float dt_s = IMU_UPDATE_DT_SEC;
	if ((0U == g_imu_ready) && (0U == s_imu_initializing))
	{
		return;
	}

	ICM42688_Get_Data();

	/* 步骤2: 将原始数据写入滤波器输入缓存 */
	g_imu_filter.gyro_raw_x = ICM42688.gyro_x;
	g_imu_filter.gyro_raw_y = ICM42688.gyro_y;
	g_imu_filter.gyro_raw_z = ICM42688.gyro_z;
	g_imu_filter.acc_raw_x = ICM42688.acc_x;
	g_imu_filter.acc_raw_y = ICM42688.acc_y;
	g_imu_filter.acc_raw_z = ICM42688.acc_z;

	/* 步骤3: 更新 IMU 滤波器（低通/陷波等） */
	IMUFilter_Update(&g_imu_filter);

	/* 步骤4: 使用滤波后的陀螺和加速度数据进行 Mahony 姿态更新 */
	MahonyAhrs_Update(
		&g_mahony_ahrs,
		g_imu_filter.gyro_filt_x, g_imu_filter.gyro_filt_y, g_imu_filter.gyro_filt_z,
		g_imu_filter.acc_filt_x, g_imu_filter.acc_filt_y, g_imu_filter.acc_filt_z,
		dt_s);

	/* 步骤5: 计算欧拉角（单位: 度）并缓存 */
	g_euler = MahonyAhrs_GetEulerDegrees(&g_mahony_ahrs);
	g_imu_update_count++;
}

uint8 IMU_Is_Ready(void)
{
	return g_imu_ready;
}
