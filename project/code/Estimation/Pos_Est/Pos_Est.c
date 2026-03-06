#include "Pos_Est.h"
#include "zf_common_headfile.h"
#include <math.h>

#ifndef sq
#define sq(x) ((x) * (x))
#endif
#define GRAVITY_CMSS (980.f)                                  /*重力加速度 单位cm/s/s*/
#define INAV_ACC_BIAS_ACCEPTANCE_VALUE (GRAVITY_CMSS * 0.25f) // Max accepted bias correction of 0.25G - unlikely we are going to be that much off anyway

volatile opFlow_t opFlow = {0};
static estimator_t estimator =
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
uint16_t count = 0U;
void Pos_Est_Update_2000HZ(void)
{
}

void Pos_Est_Update_250HZ(void)
{

    static float rangeLpf = 0.f;                                                 // 激光测距低通
    static float accLpf[3] = {0.f};                                              /*加速度低通*/
                                                                                 // float weight = wBaro;				//气压权重，范围0-1，0表示完全信任气压高度，1表示完全信任激光高度       // 不用气压计试试
    static float fusedHeightLpf = 0.f;                                           // 融合高度低通
    fusedHeightLpf += (g_tof_fused_height_mm / 1000.0f - fusedHeightLpf) * 0.1f; // 单位m，融合高度低通

    float ax, ay, az;
    float az_up;
    AccelCalibration_GetLevelAccelMps2(&ax, &ay, &az); /*获取水平系线加速度，单位m/s^2*/
    az_up = AccelCalibration_GetVerticalAccelUpMps2();
    ax *= 100.0f; /*转换为cm/s^2*/
    ay *= 100.0f;
    az *= 100.0f;
    az_up *= 1000.0f; /* Z轴统一使用mm/s^2 */
    if (ax > 4.0f) { ax -= 4.0f; }
    else if (ax < -4.0f) { ax += 4.0f; }
    else { ax = 0.0f; }
    if (ay > 4.0f) { ay -= 4.0f; }
    else if (ay < -4.0f) { ay += 4.0f; }
    else { ay = 0.0f; }
    if (az > 4.0f) { az -= 4.0f; }
    else if (az < -4.0f) { az += 4.0f; }
    else { az = 0.0f; }
    if (az_up > 40.0f) { az_up -= 40.0f; }
    else if (az_up < -40.0f) { az_up += 40.0f; }
    else { az_up = 0.0f; }
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
    estimator.acc[2] = accLpf[2];                                    /* Z轴单位:mm/s^2 */
    float errPosZ = (float)g_tof_fused_height_mm - estimator.pos[2]; /* Z轴误差统一:mm */

    /* 位置预估: Z-axis */
    inavFilterPredict(2, POS_EST_250HZ_DT, estimator.acc[2]);
    /* 位置校正: Z-axis，权重0.35与正点原子wBaro一致，避免过度校正导致速度振荡 */
    inavFilterCorrectPos(2, POS_EST_250HZ_DT, errPosZ, 0.35f);

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

    // wifi_vofa_JustFloat(9u, estimator.pos[0], estimator.pos[1], estimator.pos[2], g_tof_fused_height_mm,
    //     accLpf[0],accLpf[1],accLpf[2],opFlow.posSum[0],opFlow.posSum[1]);
}

void Pos_Est_Update_100HZ(void)
{
    PMW3901_Update_100HZ(); /* 先更新光流数据，确保读取到最新的像素增量和质量指标 */

    int16_t pixelDx = g_pmw3901_raw.deltaX;
    int16_t pixelDy = g_pmw3901_raw.deltaY;

    /* 异常值剔除：阈值35像素，过滤PMW3901周期性突发噪声（实测静止时突发可达±20像素） */
    if (Pos_Est_Absf(pixelDx) < 40 && Pos_Est_Absf(pixelDy) < 40)
    {
        opFlow.pixSum[0] += pixelDx;
        opFlow.pixSum[1] += pixelDy;
    }

    float height = g_tof_fused_height_mm / 1000.0f; /*读取高度信息 单位m*/
    /*
     * 姿态补偿：
     * 俯仰/横滚会导致图像平面出现“伪位移”，
     * 使用 tan(角度) 进行一阶几何补偿。
     */
    float tanRoll = Pos_Est_Tan(g_euler.roll * DEG2RAD);
    float tanPitch = Pos_Est_Tan(g_euler.pitch * DEG2RAD);

    opFlow.pixComp[0] = 480.f * tanRoll;  /*右向轴由横滚补偿：右倾时光流看到地面左移，补偿抵消*/
    opFlow.pixComp[1] = -480.f * tanPitch; /*前向轴由俯仰补偿：符号取反以正确抵消姿态耦合*/
    opFlow.pixValid[0] = (opFlow.pixSum[0] + opFlow.pixComp[0]); /*实际输出像素*/
    opFlow.pixValid[1] = (opFlow.pixSum[1] + opFlow.pixComp[1]);

    /* 位移换算系数：1m 标定值 * 当前高度(m) */
    static uint8_t heightWasLow = 1U;
    float coeff = RESOLUTION * height;
    if (height < 0.15f) /*高度过低时不更新位置，避免地面效应干扰*/
    {
        coeff = 0.0f;
        heightWasLow = 1U;
    }

    /* 首帧跳过：从低高度过渡到有效高度时，同步pixValidLast防止
     * pixValid-pixValidLast(=0)产生巨大跳变（如pitch=20°时可跳22cm） */
    if (heightWasLow && coeff > 0.0f)
    {
        heightWasLow = 0U;
        opFlow.pixValidLast[0] = opFlow.pixValid[0];
        opFlow.pixValidLast[1] = opFlow.pixValid[1];
    }

    /*
     * 位移增量 = 系数 * 两次有效像素差
     * 注意：此处使用 pixValid 与上次 pixValid 的差，
     * 不是直接使用当前帧 deltaX/deltaY，可抑制短时噪声。
     */
    opFlow.deltaPos[0] = coeff * (opFlow.pixValid[0] - opFlow.pixValidLast[0]); /*2帧之间位移变化量，单位cm*/
    opFlow.deltaPos[1] = coeff * (opFlow.pixValid[1] - opFlow.pixValidLast[1]);
    opFlow.pixValidLast[0] = opFlow.pixValid[0]; /*上一次实际输出像素*/
    opFlow.pixValidLast[1] = opFlow.pixValid[1];

    /* 速度 = 位移 / dt，单位 cm/s */
    opFlow.deltaVel[0] = opFlow.deltaPos[0] / POS_EST_100HZ_DT; /*速度 cm/s*/
    opFlow.deltaVel[1] = opFlow.deltaPos[1] / POS_EST_100HZ_DT;

    opFlow.velLpf[0] += (opFlow.deltaVel[0] - opFlow.velLpf[0]) * 0.08f; /*速度低通 cm/s，alpha=0.08降噪*/
    opFlow.velLpf[1] += (opFlow.deltaVel[1] - opFlow.velLpf[1]) * 0.08f; /*速度低通 cm/s，alpha=0.08降噪*/

    /* 速度限幅，防止异常输入影响下游位置控制 */
    opFlow.velLpf[0] = Pos_Est_Clampf(opFlow.velLpf[0], -POS_EST_VEL_LIMIT, POS_EST_VEL_LIMIT); /*速度限幅 cm/s*/
    opFlow.velLpf[1] = Pos_Est_Clampf(opFlow.velLpf[1], -POS_EST_VEL_LIMIT, POS_EST_VEL_LIMIT); /*速度限幅 cm/s*/

    /* 位移死区：0.2cm≈1.6像素@0.6m，过滤单像素噪声对posSum的累积漂移 */
    if (Pos_Est_Absf(opFlow.deltaPos[0]) < 0.2f) { opFlow.deltaPos[0] = 0.0f; }
    if (Pos_Est_Absf(opFlow.deltaPos[1]) < 0.2f) { opFlow.deltaPos[1] = 0.0f; }

    /* 累加位移，用于定点控制和调试观测 */
    opFlow.posSum[0] += opFlow.deltaPos[0]; /*累积位移 cm*/
    opFlow.posSum[1] += opFlow.deltaPos[1]; /*累积位移 cm*/

    opFlow.isOpFlowOk = (g_pmw3901_raw.squal >= POS_EST_SQUAL_MIN) ? 1U : 0U; /*光流状态*/

    /* 16通道VOFA调试输出：完整光流处理链路诊断 */
    wifi_vofa_JustFloat(16u,
        (float)pixelDx,           /* CH1:  原始像素增量X(右) */
        (float)pixelDy,           /* CH2:  原始像素增量Y(前) */
        opFlow.pixComp[0],        /* CH3:  像素补偿X(右) */
        opFlow.pixComp[1],        /* CH4:  像素补偿Y(前) */
        opFlow.deltaPos[0],       /* CH5:  位移增量X cm */
        opFlow.deltaPos[1],       /* CH6:  位移增量Y cm */
        opFlow.deltaVel[0],       /* CH7:  原始速度X cm/s */
        opFlow.deltaVel[1],       /* CH8:  原始速度Y cm/s */
        opFlow.velLpf[0],         /* CH9:  滤波速度X cm/s */
        opFlow.velLpf[1],         /* CH10: 滤波速度Y cm/s */
        opFlow.posSum[0],         /* CH11: 累积位移X cm */
        opFlow.posSum[1],         /* CH12: 累积位移Y cm */
        (float)g_pmw3901_raw.squal, /* CH13: 光流质量 */
        (float)g_tof_fused_height_mm, /* CH14: 融合高度 mm */
        g_euler.roll,             /* CH15: 横滚角 deg */
        g_euler.pitch             /* CH16: 俯仰角 deg */
    );
}
