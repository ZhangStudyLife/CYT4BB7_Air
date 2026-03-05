#include "Pos_Est.h"
#include "zf_common_headfile.h"
#include <math.h>

typedef struct
{
    float accel_bias_x_mps2;
    float accel_bias_y_mps2;
    float accel_lpf_x_mps2;
    float accel_lpf_y_mps2;
    float accel_vibe_metric;
    float acc_weight_xy;
    float flow_vx_lpf_mps;
    float flow_vy_lpf_mps;
    float flow_pos_x_m;
    float flow_pos_y_m;
    float flow_dead_time_s;
    uint8_t flow_ref_ready;
} PosEstState_t;

volatile PosEstOutput_t g_pos_est_output = {0};
volatile PosEstDebug_t g_pos_est_debug = {0};
static PosEstState_t s_pos_est_state = {0};

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

static float Pos_Est_Absf(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }
    return value;
}

void Pos_Est_Init(void)
{
}

float gx_sum = 0.0f;
float gy_sum = 0.0f;
uint16_t count = 0U;
void Pos_Est_Update_2000HZ(void)
{
    gx_sum+=g_imu_filter.gyro_filt_x;
    gy_sum+=g_imu_filter.gyro_filt_y;
    count++;
}

void Pos_Est_Update_250HZ(void)
{
}

void Pos_Est_Update_50HZ(void)
{
    PMW3901_Update_50HZ();
    g_pos_est_debug.deltaX = g_pmw3901_raw.deltaX;
    g_pos_est_debug.deltaY = g_pmw3901_raw.deltaY;
    g_pos_est_debug.squal = g_pmw3901_raw.squal;

    g_pos_est_debug.pixel_flow_X = g_pos_est_debug.deltaX / 470.0f / 0.02f; /* 470像素视场，20ms周期 */
    g_pos_est_debug.pixel_flow_Y = g_pos_est_debug.deltaY / 470.0f / 0.02f;

    /*   获取机体陀螺仪角速率 (rad/s)
     *   roll右倾→图像左移→deltaX为负, 需 +gyro_roll 补偿
     *   pitch抬头→图像后移→deltaY为负, 需 -gyro_pitch 补偿
     *   符号与拟合K值一致: K_X=+8.40, K_Y=-8.85*/


    float body_rate_x = -(gx_sum/count) * POS_EST_DEG2RAD; // = -gx_rad
    float body_rate_y = (gy_sum/count) * POS_EST_DEG2RAD;  // = +gy_rad
    count = 0U;
    gx_sum = 0.0f;
    gy_sum = 0.0f;

    float flow_comp_x = g_pos_est_debug.pixel_flow_X - body_rate_x; // = flow_rate_x − (−gx_rad) = flow_rate_x + gx_rad
    float flow_comp_y = g_pos_est_debug.pixel_flow_Y - body_rate_y; // = flow_rate_y − (+gy_rad) = flow_rate_y − gy_rad


    wifi_vofa_JustFloat(7U, g_pos_est_debug.pixel_flow_X, g_pos_est_debug.pixel_flow_Y, body_rate_x, body_rate_y,flow_comp_x, flow_comp_y, g_pos_est_debug.squal);
    
}
