#include "Pos_Est.h"
#include "zf_common_headfile.h"
#include <math.h>

#ifndef sq
#define sq(x) ((x) * (x))
#endif
#define GRAVITY_CMSS (980.f)                                  /*重力加速度 单位cm/s/s*/
#define INAV_ACC_BIAS_ACCEPTANCE_VALUE (GRAVITY_CMSS * 0.25f) // Max accepted bias correction of 0.25G - unlikely we are going to be that much off anyway
#define POS_EST_DEG_TO_RAD (3.14159265359f / 180.0f)                  /*角度转弧度转换系数*/

volatile opFlow_t opFlow = {0};
estimator_t estimator =
    {
        .vAccDeadband = 4.0f,
        .accBias[0] = 0.0f,
        .accBias[1] = 0.0f,
        .accBias[2] = 0.0f,
        .acc[0] = 0.0f,
        .acc[1] = 0.0f,
        .acc[2] = 0.0f,
        .vel[0] = 0.0f,
        .vel[1] = 0.0f,
        .vel[2] = 0.0f,
        .pos[0] = 0.0f,
        .pos[1] = 0.0f,
        .pos[2] = 0.0f,
};

static float Pos_Est_Clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

/**
 * @brief 计算角度的正切值
 *
 * @param angle_rad 输入角度，单位：rad（弧度）
 *                  建议避免接近 ±pi/2 的角度，以防正切值数值发散。
 *
 * @return float 输出正切值 tan(angle_rad)，单位：无量纲
 */
static float Pos_Est_Tan(float angle_rad)
{
    return tanf(angle_rad);
}

static float Pos_Est_Absf(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }
    return value;
}

/* Inertial filter, implementation taken from PX4 implementation by Anton Babushkin <rk3dov@gmail.com> */
static void inavFilterPredict(int axis, float dt, float acc)
{
    estimator.pos[axis] += estimator.vel[axis] * dt + acc * dt * dt / 2.0f;
    estimator.vel[axis] += acc * dt;
}
/*位置校正*/
static void inavFilterCorrectPos(int axis, float dt, float e, float w)
{
    float ewdt = e * w * dt;
    estimator.pos[axis] += ewdt;
    estimator.vel[axis] += w * ewdt;
}
/*速度校正*/
static void inavFilterCorrectVel(int axis, float dt, float e, float w)
{
    estimator.vel[axis] += e * w * dt;
}

void Pos_Est_Init(void)
{
    estimator.vAccDeadband = 4.0f;
    estimator.accBias[0] = 0.0f;
    estimator.accBias[1] = 0.0f;
    estimator.accBias[2] = 0.0f;
    estimator.acc[0] = 0.0f;
    estimator.acc[1] = 0.0f;
    estimator.acc[2] = 0.0f;
    estimator.vel[0] = 0.0f;
    estimator.vel[1] = 0.0f;
    estimator.vel[2] = 0.0f;
    estimator.pos[0] = 0.0f;
    estimator.pos[1] = 0.0f;
    estimator.pos[2] = 0.0f;

    opFlow.pixSum[0] = 0.0f;
    opFlow.pixSum[1] = 0.0f;
    opFlow.pixComp[0] = 0.0f;
    opFlow.pixComp[1] = 0.0f;
    opFlow.pixValid[0] = 0.0f;
    opFlow.pixValid[1] = 0.0f;
    opFlow.pixValidLast[0] = 0.0f;
    opFlow.pixValidLast[1] = 0.0f;

    opFlow.deltaPos[0] = 0.0f;
    opFlow.deltaPos[1] = 0.0f;
    opFlow.deltaVel[0] = 0.0f;
    opFlow.deltaVel[1] = 0.0f;
    opFlow.posSum[0] = 0.0f;
    opFlow.posSum[1] = 0.0f;
    opFlow.velLpf[0] = 0.0f;
    opFlow.velLpf[1] = 0.0f;

    opFlow.isOpFlowOk = false;
    opFlow.isDataValid = false;
}

float gx_sum = 0.0f;
float gy_sum = 0.0f;
uint16_t gyro_count = 0U;
void Pos_Est_Update_1000HZ(void)
{
    gx_sum += g_imudata_250hz.gyrox;
    gy_sum += g_imudata_250hz.gyroy;
    gyro_count += 1;
}

void Pos_Est_Update_250HZ(void)
{

    static float accLpf[3] = {0.f};                                              /*加速度低通*/
                                                                                 // float weight = wBaro;				//气压权重，范围0-1，0表示完全信任气压高度，1表示完全信任激光高度       // 不用气压计试试
    static float fusedHeightLpf = 0.f;                                           // 融合高度低通
    fusedHeightLpf += (g_tof_fused_height_mm / 1000.0f - fusedHeightLpf) * 0.1f; // 单位m，融合高度低通

    float ax, ay, az;                                  /* 水平系线性加速度，单位m/s^2；+ax=机头前方，+ay=机体右侧，+az=Down，忽略yaw，仅去除roll/pitch影响 */
    float ax_raw_g, ay_raw_g, az_raw_g;               /* 250Hz滤波后的机体系比力，单位g；+ax前，+ay右，+az下，静止平放约为0/0/-1g */
    float az_up;                                       /* 竖直Up方向线性加速度，单位m/s^2；静止时应接近0 */

    /* 原始输入坐标系：
     * g_imudata_250hz.acc* 为机体系FRD比力，单位g，包含重力项。
     * +X前，+Y右，+Z下；静止平放时 accz≈-1g。
     */
    ax_raw_g = g_imudata_250hz.accx;
    ay_raw_g = g_imudata_250hz.accy;
    az_raw_g = g_imudata_250hz.accz;

    /* 输出坐标系：
     * ax/ay/az 为“水平系线性加速度”，由校准模块完成去重力与姿态解耦。
     * 仅消除 roll/pitch 对前后/左右加速度的影响，不引入 yaw 旋转。
     * 因此飞机朝向变化不会改变前后/左右轴定义：
     * +ax 始终表示机头前方加速度，+ay 始终表示机体右侧加速度，+az 为 Down。
     */
    AccelCalibration_GetLevelAccelMps2(&ax, &ay, &az);
    az_up = AccelCalibration_GetVerticalAccelUpMps2();

    wifi_vofa_JustFloat(11, ax_raw_g, ay_raw_g, az_raw_g, 
        ax, ay, az,
        g_euler.roll, g_euler.pitch,g_euler.yaw,
        opFlow.velLpf[0],opFlow.velLpf[1]
    );
    ax *= 100.0f; /*转换为cm/s^2*/
    ay *= 100.0f;
    az *= 100.0f;
    az_up *= 1000.0f; /* Z轴统一使用mm/s^2 */
    if (ax > 6.0f)
    {
        ax -= 6.0f;
    }
    else if (ax < -6.0f)
    {
        ax += 6.0f;
    }
    else
    {
        ax = 0.0f;
    }
    if (ay > 6.0f)
    {
        ay -= 6.0f;
    }
    else if (ay < -6.0f)
    {
        ay += 6.0f;
    }
    else
    {
        ay = 0.0f;
    }

    if (az_up > 70.0f)
    {
        az_up -= 70.0f;
    }
    else if (az_up < -70.0f)
    {
        az_up += 70.0f;
    }
    else
    {
        az_up = 0.0f;
    }
    accLpf[0] += (ax - accLpf[0]) * 0.1f;    /*加速度低通*/
    accLpf[1] += (ay - accLpf[1]) * 0.1f;    /*加速度低通*/
    accLpf[2] += (az_up - accLpf[2]) * 0.1f; /*加速度低通*/
    accLpf[0] = (accLpf[0] > 1000.0f) ? 1000.0f : accLpf[0] < -1000.0f ? -1000.0f
                                                                       : accLpf[0]; /*加速度限幅*/
    accLpf[1] = (accLpf[1] > 1000.0f) ? 1000.0f : accLpf[1] < -1000.0f ? -1000.0f
                                                                       : accLpf[1]; /*加速度限幅*/
    accLpf[2] = (accLpf[2] > 10000.0f) ? 10000.0f : accLpf[2] < -10000.0f ? -10000.0f
                                                                          : accLpf[2]; /*加速度限幅*/

    // 实际测试
    // 飞机向正前方水平加速 estimator.acc[0]为正
    // 飞机水平向右方加速   estimator.acc[1]是正
    // 飞机垂直向上加速    estimator.acc[2]是正
    estimator.acc[0] = accLpf[1]; /*更新估测加速度，单位cm/s^2*/
    estimator.acc[1] = accLpf[0];
    estimator.acc[2] = accLpf[2];                             /* Z轴单位:mm/s^2 */
    float errPosZ = (float)fusedHeightLpf - estimator.pos[2]; /* Z轴误差统一:mm */

    /* 位置校正: Z-axis，权重0.35与正点原子wBaro一致，避免过度校正导致速度振荡 */
    inavFilterCorrectPos(2, POS_EST_250HZ_DT, errPosZ, 0.35f);
    /* 位置预估: Z-axis */
    inavFilterPredict(2, POS_EST_250HZ_DT, estimator.acc[2]);

    float opflowDt = POS_EST_250HZ_DT;

    float opResidualX = opFlow.posSum[0] - estimator.pos[0];
    float opResidualY = opFlow.posSum[1] - estimator.pos[1];
    float opResidualXVel = opFlow.velLpf[0] - estimator.vel[0];
    float opResidualYVel = opFlow.velLpf[1] - estimator.vel[1];

    float opWeightScaler = 1.0f;

    float wXYPos = 1 * opWeightScaler;     // 1表示完全信任光流位置，0表示完全不信任光流位置
    float wXYVel = 2 * sq(opWeightScaler); // 二次关系，增加权重区分度，2表示完全信任光流速度，0表示完全不信任光流速度

    /* 位置预估: XY-axis */
    inavFilterPredict(0, opflowDt, estimator.acc[0]);
    inavFilterPredict(1, opflowDt, estimator.acc[1]);
    /* 位置校正: XY-axis */
    inavFilterCorrectPos(0, opflowDt, opResidualX, wXYPos);
    inavFilterCorrectPos(1, opflowDt, opResidualY, wXYPos);
    /* 速度校正: XY-axis */
    inavFilterCorrectVel(0, opflowDt, opResidualXVel, wXYVel);
    inavFilterCorrectVel(1, opflowDt, opResidualYVel, wXYVel);
}

void Pos_Est_Update_100HZ(void)
{
    PMW3901_Update_100HZ(); /* 先更新光流数据，确保读取到最新的像素增量和质量指标 */

    float pixelDx = (float)g_pmw3901_raw.deltaX;
    float pixelDy = (float)g_pmw3901_raw.deltaY;
    int16_t spual_pmw = g_pmw3901_raw.squal;
    float height = g_tof_fused_height_mm / 1000.0f; /*读取高度信息 单位m*/

    /* 异常值剔除：阈值35像素，过滤PMW3901周期性突发噪声（实测静止时突发可达±20像素） */
    if (Pos_Est_Absf(pixelDx) > 40)
    {
        pixelDx = 0;
    }
    if (Pos_Est_Absf(pixelDx) > 40)
    {
        pixelDx = 0;
    }

    float gyroRollDps = 0.0f;
    float gyroPitchDps = 0.0f;
    if (gyro_count != 0)
    {
        gyroRollDps = gx_sum / gyro_count;
        gyroPitchDps = gy_sum / gyro_count;
        gx_sum = 0;
        gy_sum = 0;
        gyro_count = 0;
    }
    gyroRollDps = Pos_Est_Clampf(g_imudata_250hz.gyrox, -68.0f,
                                 68.0f);
    gyroPitchDps = Pos_Est_Clampf(g_imudata_250hz.gyroy, -68.0f,
                                  68.0f);

    /* 对光流像素增量做一阶低通，进一步抑制随机噪声 */
    static float pixelDxLpf = 0.0f;
    static float pixelDyLpf = 0.0f;
    pixelDxLpf += (pixelDx - pixelDxLpf) * 0.4f;
    pixelDyLpf += (pixelDy - pixelDyLpf) * 0.4f;
    pixelDx = pixelDxLpf;
    pixelDy = pixelDyLpf;
    /* 对陀螺仪角速度做 5 点因果均值，使用环形缓冲降低计算开销 */
    static float gyroRollHist[5] = {0.0f};
    static float gyroPitchHist[5] = {0.0f};
    static float gyroRollSum = 0.0f;
    static float gyroPitchSum = 0.0f;
    static uint8_t gyroHistIndex = 0U;
    gyroRollSum += gyroRollDps - gyroRollHist[gyroHistIndex];
    gyroPitchSum += gyroPitchDps - gyroPitchHist[gyroHistIndex];
    gyroRollHist[gyroHistIndex] = gyroRollDps;
    gyroPitchHist[gyroHistIndex] = gyroPitchDps;
    gyroHistIndex++;
    if (gyroHistIndex >= 5U)
    {
        gyroHistIndex = 0U;
    }
    gyroRollDps = gyroRollSum * 0.2f;
    gyroPitchDps = gyroPitchSum * 0.2f;

    /* 位移换算系数：1m 标定值 * 当前高度(m) */
    static uint8_t heightWasLow = 1U;
    float coeff = RESOLUTION * (height - 0.05f); // 光流安装位置实际比TOF低 5CM
    if (height < 0.2f)                           /*高度过低时不更新位置，避免地面效应干扰*/
    {
        coeff = 0.0f;
        heightWasLow = 1U;
    }

    // 进行姿态解耦 , 补偿数据是上位机测量出来的拟合的
    opFlow.deltaPos[0] = (pixelDx + 0.1719264517 * gyroRollDps)*coeff;   // 1m高度下 1像素对应的位移cm * 当前高度m，得到当前高度下1像素对应的位移cm，乘以像素增量得到位移cm
    opFlow.deltaPos[1] = (pixelDy - 0.1664168828 * gyroPitchDps)*coeff; // 1m高度下 1像素对应的位移cm * 当前高度m，得到当前高度下1像素对应的位移cm，乘以像素增量得到位移cm

    /* 速度 = 位移 / dt，单位 cm/s */
    opFlow.deltaVel[0] = opFlow.deltaPos[0] / POS_EST_100HZ_DT; /*速度 cm/s*/
    opFlow.deltaVel[1] = opFlow.deltaPos[1] / POS_EST_100HZ_DT;

    {
        static float velKalmanP[2] = {1.0f, 1.0f};     /*速度卡尔曼误差协方差*/
        const float velKalmanQ = 0.05f;                 /*过程噪声协方差Q*/
        const float velKalmanR = 0.3f;                 /*测量噪声协方差R*/
        float velKalmanK;                              /*卡尔曼增益*/

        /* X轴速度一阶卡尔曼滤波 */
        velKalmanP[0] += velKalmanQ;
        velKalmanK = velKalmanP[0] / (velKalmanP[0] + velKalmanR);
        opFlow.velLpf[0] += velKalmanK * (opFlow.deltaVel[0] - opFlow.velLpf[0]);
        velKalmanP[0] = (1.0f - velKalmanK) * velKalmanP[0];

        /* Y轴速度一阶卡尔曼滤波 */
        velKalmanP[1] += velKalmanQ;
        velKalmanK = velKalmanP[1] / (velKalmanP[1] + velKalmanR);
        opFlow.velLpf[1] += velKalmanK * (opFlow.deltaVel[1] - opFlow.velLpf[1]);
        velKalmanP[1] = (1.0f - velKalmanK) * velKalmanP[1];
    }

    /* 速度限幅，防止异常输入影响下游位置控制 */
    opFlow.velLpf[0] = Pos_Est_Clampf(opFlow.velLpf[0], -POS_EST_VEL_LIMIT, POS_EST_VEL_LIMIT); /*速度限幅 cm/s*/
    opFlow.velLpf[1] = Pos_Est_Clampf(opFlow.velLpf[1], -POS_EST_VEL_LIMIT, POS_EST_VEL_LIMIT); /*速度限幅 cm/s*/
    // wifi_vofa_JustFloat(8u, opFlow.deltaPos[0], opFlow.deltaPos[1], opFlow.deltaVel[0], opFlow.deltaVel[1], height, g_euler.pitch,opFlow.velLpf[0],opFlow.velLpf[1]);
    /* 位移死区：0.2cm≈1.6像素@0.6m，过滤单像素噪声对posSum的累积漂移 */
    if (Pos_Est_Absf(opFlow.deltaPos[0]) < 0.2f)
    {
        opFlow.deltaPos[0] = 0.0f;
    }
    if (Pos_Est_Absf(opFlow.deltaPos[1]) < 0.2f)
    {
        opFlow.deltaPos[1] = 0.0f;
    }

    /* 累加位移，用于定点控制和调试观测 */
    opFlow.posSum[0] += opFlow.deltaPos[0]; /*累积位移 cm*/
    opFlow.posSum[1] += opFlow.deltaPos[1]; /*累积位移 cm*/

    opFlow.isOpFlowOk = (g_pmw3901_raw.squal >= POS_EST_SQUAL_MIN) ? 1U : 0U; /*光流状态*/

    // wifi_vofa_JustFloat(13,g_pmw3901_raw.deltaX,g_pmw3901_raw.deltaY,pixelDx, pixelDy, spual_pmw,
    //                     gyroRollDps, gyroPitchDps,
    //                     opFlow.deltaPos[0], opFlow.deltaPos[1],
    //                 opFlow.deltaVel[0],opFlow.deltaVel[1],
    //             opFlow.velLpf[0],opFlow.velLpf[1]);

}
