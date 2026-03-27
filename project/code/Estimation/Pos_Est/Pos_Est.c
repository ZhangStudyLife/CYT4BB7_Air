#include "Pos_Est.h"
#include "filter.h"
#include "FlightController/fc_params.h"

extern volatile uint32 tick_1000us_cnt;

/* 50Hz 末端速度一阶低通截止频率 10Hz，对应 alpha = 1 - exp(-2πfc/fs) */
#define POS_EST_VEL_OUT_LPF_ALPHA (0.71539046f)

/* 1000Hz 水平加速度一阶低通 alpha
 * 上游 g_imufilter_1000hz 已经过 12Hz Butterworth LPF（群延迟约 19ms），
 * 此处额外 LPF 仅用于轻微平滑，alpha 取大值以避免引入过多相位延迟。
 * alpha=0.20 → fc≈31.8Hz，群延迟≈5ms，总 acc 延迟≈24ms
 * 实飞数据实测（acc vs d(vel)/dt 互相关）表明原 alpha=0.04（总延迟44ms）
 * 导致 acc 相位滞后速度导数约 40~60ms，修改后可缩短至约 12ms。 */
#define POS_EST_ACC_LPF_ALPHA (0.20f)

/* 光流解算得到的 X 轴速度，单位 cm/s，往左飞为正，往右飞为负 */
float opflow_vel_x = 0.0f;
/* 光流解算得到的 Y 轴速度，单位 cm/s，往前飞为正，往后飞为负 */
float opflow_vel_y = 0.0f;

/* 位置估计的 X 轴速度，单位 cm/s */
float Pos_Est_vel_x = 0.0f;
/* 位置估计的上一拍 X 轴速度，单位 cm/s */
float Pos_Est_vel_x_last = 0.0f;
/* 位置估计的 Y 轴速度，单位 cm/s */
float Pos_Est_vel_y = 0.0f;
/* 位置估计的上一拍 Y 轴速度，单位 cm/s */
float Pos_Est_vel_y_last = 0.0f;

/* 位置估计的 X 轴位置，单位 cm，往左飞为正，往右飞为负 */
float Pos_Est_pos_x = 0.0f;
/* 位置估计的上一拍 X 轴位置，单位 cm */
float Pos_Est_pos_x_last = 0.0f;
/* 位置估计的 Y 轴位置，单位 cm，往前飞为正，往后飞为负 */
float Pos_Est_pos_y = 0.0f;
/* 位置估计的上一拍 Y 轴位置，单位 cm */
float Pos_Est_pos_y_last = 0.0f;

/* X 轴速度 Kalman 输出，单位 cm/s */
float Pos_Est_vel_x_kf = 0.0f;
/* Y 轴速度 Kalman 输出，单位 cm/s */
float Pos_Est_vel_y_kf = 0.0f;

/* X 轴加速度一阶低通状态 */
static LPF1_t s_acc_lp_x;
/* Y 轴加速度一阶低通状态 */
static LPF1_t s_acc_lp_y;
/* X 轴末端速度一阶低通状态 */
static LPF1_t s_vel_out_lp_x;
/* Y 轴末端速度一阶低通状态 */
static LPF1_t s_vel_out_lp_y;

/* X 轴加速度一阶低通输出，单位 cm/s^2，飞机往前加速为正，往后加速为负 */
float acc_x_lp = 0.0f;
/* Y 轴加速度一阶低通输出，单位 cm/s^2，飞机往右加速为正，往左加速为负 */
float acc_y_lp = 0.0f;

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
    LPF1_Init(&s_acc_lp_x, POS_EST_ACC_LPF_ALPHA);
    LPF1_Init(&s_acc_lp_y, POS_EST_ACC_LPF_ALPHA);
    LPF1_Init(&s_vel_out_lp_x, POS_EST_VEL_OUT_LPF_ALPHA);
    LPF1_Init(&s_vel_out_lp_y, POS_EST_VEL_OUT_LPF_ALPHA);

    Pos_Est_pos_x = 0.0f;
    Pos_Est_pos_y = 0.0f;
    Pos_Est_pos_x_last = 0.0f;
    Pos_Est_pos_y_last = 0.0f;
    Pos_Est_vel_x_kf = 0.0f;
    Pos_Est_vel_y_kf = 0.0f;
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
    LPF1_Reset(&s_acc_lp_x);
    LPF1_Reset(&s_acc_lp_y);
    LPF1_Reset(&s_vel_out_lp_x);
    LPF1_Reset(&s_vel_out_lp_y);

    acc_x_lp = 0.0f;
    acc_y_lp = 0.0f;
    Pos_Est_vel_x_kf = 0.0f;
    Pos_Est_vel_y_kf = 0.0f;
    Pos_Est_pos_x = 0.0f;
    Pos_Est_pos_y = 0.0f;
    Pos_Est_pos_x_last = 0.0f;
    Pos_Est_pos_y_last = 0.0f;
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
    float acc_x_temp;
    float acc_y_temp;
    float sp;
    float cp;
    float sr;
    float cr;

    FlowGyroDecoupler_Push1000Hz(tick_1000us_cnt, g_imudata_250hz.gyrox, g_imudata_250hz.gyroy);

    /* 取陷波+低通滤波后的传感器系加速度（单位 g，已校准零偏/尺度） */
    acc_sensor[0] = g_imufilter_1000hz.accx;
    acc_sensor[1] = g_imufilter_1000hz.accy;
    acc_sensor[2] = g_imufilter_1000hz.accz;
    AccelCalibration_RotateImuToBody(acc_sensor, acc_body);

    sp = g_euler.sin_pitch;
    cp = g_euler.cos_pitch;
    sr = g_euler.sin_roll;
    cr = g_euler.cos_roll;

    /* 旋转到水平系（yaw=0近似），重力自动消除。
     * level_x = cp*body_x + sp*sr*body_y + sp*cr*body_z  (前进为正)
     * level_y = cr*body_y - sr*body_z                     (向右为正)
     * 单位 g → cm/s^2 */
    acc_x_temp = (cp * acc_body[0] + sp * sr * acc_body[1] + sp * cr * acc_body[2]) * 9.80665f * 100.0f;
    acc_y_temp = (cr * acc_body[1] - sr * acc_body[2]) * 9.80665f * 100.0f;

    acc_x_lp = LPF1_Update(&s_acc_lp_x, acc_x_temp);
    acc_y_lp = LPF1_Update(&s_acc_lp_y, acc_y_temp);
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
    float vel_x_pred;
    float vel_y_pred;
    const float dt = 0.02f;

    PMW3901_Update_50HZ();
    FlowGyroDecoupler_Update50Hz(tick_1000us_cnt, g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY);
    dec_x = FlowGyroDecoupler_GetDecX();
    dec_y = FlowGyroDecoupler_GetDecY();
    height = g_tof_fused_height_mm / 1000.0f;

    /* 1m 高度下 1 像素约对应 0.2131946 cm 位移，同时对过低高度做保护 */
    coeff = 0.2131946f * (height - 0.05f);
    if (height < 0.2f)
    {
        coeff = 0.0f;
    }

    /* 计算光流速度，此刻加入高度补偿，单位 cm/s */
    opflow_vel_x = dec_x * coeff * 50.0f;
    opflow_vel_y = dec_y * coeff * 50.0f;

    /* X: 光流左正右负，acc_y 右正左负，所以预测项取负 */
    vel_x_pred = Pos_Est_vel_x_kf - acc_y_lp * dt;
    /* Y: 光流前正后负，acc_x 前正后负，所以预测项同号 */
    vel_y_pred = Pos_Est_vel_y_kf + acc_x_lp * dt;

    Pos_Est_vel_x_last = Pos_Est_vel_x;
    Pos_Est_vel_y_last = Pos_Est_vel_y;

    /* squal 有效性门控：光流质量过低时不参与融合 */
    {
        uint8_t opflow_valid = (g_pmw3901_raw.squal >= 25U) && (height >= 0.2f);
        float k_use = opflow_valid ? g_fc_params.pos_est_k_flow : 0.0f;

        /* 用光流测量对惯性预测做互补校正 */
        Pos_Est_vel_x = vel_x_pred + k_use * (opflow_vel_x - vel_x_pred);
        Pos_Est_vel_y = vel_y_pred + k_use * (opflow_vel_y - vel_y_pred);

        /* 光流失效时对融合速度施加摩擦衰减，防止加速度偏置积分漂移 */
        if (!opflow_valid)
        {
            Pos_Est_vel_x *= 0.98f;
            Pos_Est_vel_y *= 0.98f;
        }
    }

    /* 速度末端改为一阶低通，50Hz 下截止频率 10Hz */
    Pos_Est_vel_x_kf = LPF1_Update(&s_vel_out_lp_x, Pos_Est_vel_x);
    Pos_Est_vel_y_kf = LPF1_Update(&s_vel_out_lp_y, Pos_Est_vel_y);

    Pos_Est_pos_x_last = Pos_Est_pos_x;
    Pos_Est_pos_y_last = Pos_Est_pos_y;

    Pos_Est_pos_x = Pos_Est_pos_x_last + 0.5f * (Pos_Est_vel_x_last + Pos_Est_vel_x) * dt;
    Pos_Est_pos_y = Pos_Est_pos_y_last + 0.5f * (Pos_Est_vel_y_last + Pos_Est_vel_y) * dt;

    // wifi_justfloat(tick_1000us_cnt,
    //                g_pmw3901_raw.deltaX, g_pmw3901_raw.deltaY, g_pmw3901_raw.squal,
    //                dec_x, dec_y,
    //                opflow_vel_x, opflow_vel_y,
    //                acc_x_lp, acc_y_lp,
    //                Pos_Est_vel_x, Pos_Est_vel_y,
    //                g_euler.roll, g_euler.pitch,height);
}
