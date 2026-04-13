#include "Pos_Est.h"
#include "FlightController/fc_params.h"

extern volatile uint32 tick_1000us_cnt;

/* X 轴加速度一阶低通输出，单位 cm/s^2，飞机往前加速为正，往后加速为负 */
float acc_x_lp = 0.0f;
/* Y 轴加速度一阶低通输出，单位 cm/s^2，飞机往右加速为正，往左加速为负 */
float acc_y_lp = 0.0f;

float vel_x_pred = 0.0f;
float vel_y_pred = 0.0f;

/* 光流解算得到的 X 轴速度，单位 cm/s，往左飞为正，往右飞为负 */
float opflow_vel_x = 0.0f;
/* 光流解算得到的 Y 轴速度，单位 cm/s，往前飞为正，往后飞为负 */
float opflow_vel_y = 0.0f;

/* 位置估计的 X 轴速度，单位 cm/s */
float Pos_Est_vel_x = 0.0f;
/* 位置估计的 Y 轴速度，单位 cm/s */
float Pos_Est_vel_y = 0.0f;



/*
 * 函数名: Pos_Est_Init
 * 功能: 初始化光流位置估计模块和相关滤波器
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Init(void)
{
    PMW3901_Init();
    FlowGyroDecoupler_Init();
    acc_x_lp = 0.0f;
    acc_y_lp = 0.0f;
    opflow_vel_x = 0.0f;
    opflow_vel_y = 0.0f;
    Pos_Est_vel_x = 0.0f;
    Pos_Est_vel_y = 0.0f;
    vel_x_pred = 0.0f;
    vel_y_pred = 0.0f;
}

/*
 * 函数名: Pos_Est_Reinit
 * 功能: 重置光流和滤波器内部状态
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Reinit(void)
{
    PMW3901_ReInit();
    FlowGyroDecoupler_Reinit();

    acc_x_lp = 0.0f;
    acc_y_lp = 0.0f;
    opflow_vel_x = 0.0f;
    opflow_vel_y = 0.0f;
    Pos_Est_vel_x = 0.0f;
    Pos_Est_vel_y = 0.0f;
    vel_x_pred = 0.0f;
    vel_y_pred = 0.0f;
}



/*
 * 函数名: Pos_Est_Update_1000HZ
 * 功能: 推送光流去旋转解耦所需陀螺数据，并更新水平加速度低通输出
 * 输入参数: 无
 * 返回值: 无
 *
 * 加速度数据路径说明：
 *   使用 g_imufilter_1000hz（已经过陷波161Hz/320Hz + 12Hz低通滤波）作为输入，
 *   通过 AccelCalibration_RotateImuToBody 旋转到机体系后，利用旋转矩阵投影到水平系。
 *   重力分量在旋转投影中自动消除，无需额外去重力步骤（数学推导验证）。
 *   相比原 AccelCalibration_GetLevelAccelMps2 路径（原始未滤波acc），
 *   可消除飞行中电机振动导致的 ±100~300 cm/s² 污染。
 */
void Pos_Est_Update_1000HZ(void)
{
    float acc_sensor[3];
    float acc_body[3];
    float sp;
    float cp;
    float sr;
    float cr;

    FlowGyroDecoupler_Push1000Hz(tick_1000us_cnt, g_imudata_250hz.gyrox, g_imudata_250hz.gyroy);

    /* 取陷波+低通滤波后的传感器系加速度（单位 g，已校准零偏/尺度） */
    acc_sensor[0] = g_imufilter_1000hz.accx;
    acc_sensor[1] = g_imufilter_1000hz.accy;
    acc_sensor[2] = g_imufilter_1000hz.accz;
    // AccelCalibration_RotateImuToBody(acc_sensor, acc_body);
    // 不用上面那个函数脱裤子放屁了，直接当传感器系就是机体系，毕竟没有安装误差
    acc_body[0] = acc_sensor[0];
    acc_body[1] = acc_sensor[1];
    acc_body[2] = acc_sensor[2];

    sp = g_euler.sin_pitch;
    cp = g_euler.cos_pitch;
    sr = g_euler.sin_roll;
    cr = g_euler.cos_roll;

    /* 旋转到水平系（yaw=0近似），重力自动消除。
     * level_x = cp*body_x + sp*sr*body_y + sp*cr*body_z  (前进为正)
     * level_y = cr*body_y - sr*body_z                     (向右为正)
     * 单位 g → cm/s^2 */
    acc_x_lp = (cp * acc_body[0] + sp * sr * acc_body[1] + sp * cr * acc_body[2]) * 9.80665f * 100.0f;
    acc_y_lp = (cr * acc_body[1] - sr * acc_body[2]) * 9.80665f * 100.0f;

    // 需要注意的是acc_y_lp右正左负，acc_x_lp前正后负
    vel_x_pred -= acc_y_lp * 0.001f;
    vel_y_pred += acc_x_lp * 0.001f;


}

/*
 * 函数名: Pos_Est_Update_50HZ
 * 功能: 更新 PMW3901 光流速度，并执行速度融合与位置积分
 * 输入参数: 无
 * 返回值: 无
 */
void Pos_Est_Update_50HZ(void)
{
    float dec_x;
    float dec_y;
    float height;
    float coeff;

    PMW3901_Update_50HZ();
    FlowGyroDecoupler_Update50Hz(tick_1000us_cnt, g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY);
    dec_x = FlowGyroDecoupler_GetDecX();
    dec_y = FlowGyroDecoupler_GetDecY();
    height = g_tof_fused_height_mm * 0.001f;

    /* 1m 高度下 1 像素约对应 0.2131946 cm 位移，同时对过低高度做保护 */
    coeff = 0.2131946f * (height - 0.05f);
    if (height < 0.2f)
    {
        coeff = 0.0f;dec_x = 0.0f;dec_y = 0.0f;
    }

    /* 计算光流速度，此刻加入高度补偿，单位 cm/s */
    opflow_vel_x = dec_x * coeff * 50.0f;
    opflow_vel_y = dec_y * coeff * 50.0f;

    /* squal 有效性门控：光流质量过低时不参与融合 */
    uint8 opflow_valid = (g_pmw3901_raw.squal >= 25U) && (height >= 0.2f);
    float k_use = opflow_valid ? g_fc_params.pos_est_k_flow : 0.0f;

    /* 用光流测量对惯性预测做互补校正 */
    Pos_Est_vel_x = vel_x_pred + k_use * (opflow_vel_x - vel_x_pred);
    Pos_Est_vel_y = vel_y_pred + k_use * (opflow_vel_y - vel_y_pred);

    /* 光流失效时对融合速度施加摩擦衰减，防止加速度偏置积分漂移 */
    if (!opflow_valid)
    {
        Pos_Est_vel_x *= 0.96f;
        Pos_Est_vel_y *= 0.96f;
    }

    vel_x_pred = Pos_Est_vel_x;
    vel_y_pred = Pos_Est_vel_y;

    
}                  
