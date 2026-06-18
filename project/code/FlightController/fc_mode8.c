#include "fc_mode.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../Image/image_data.h"
#include <math.h>

extern float g_car_vel_x;
extern float g_car_vel_y;

static pid_t s_mode8_imgx_pid;
static pid_t s_mode8_imgy_pid;
pid_t g_mode8_velx_pid;
pid_t g_mode8_vely_pid;
float g_mode8_velx_target = 0.0f;
float g_mode8_vely_target = 0.0f;

static const float s_mode8_img_fb_limit_cmps = 200.0f;
static const float s_mode8_vel_limit_cmps = 200.0f;
static const float s_mode8_vel_accel_cmps2 = 250.0f;
static const float s_mode8_vel_jerk_cmps3 = 1800.0f;
static const float s_mode8_angle_limit_deg = 15.0f;
static const float s_mode8_img_y_offset = 0.0f;
static const float s_mode8_center_weight = 1.0f;
static const float s_mode8_side_weight_cap = 0.25f;
static const float s_mode8_front_x_weight = 0.227180f;
static const float s_mode8_front_y_weight = 0.135893f;
static const float s_mode8_back_x_weight = 0.194550f;
static const float s_mode8_back_y_weight = 0.134159f;
static float s_mode8_accel_x = 0.0f;
static float s_mode8_accel_y = 0.0f;
static uint16_t s_mode8_yaw_tick = 0U;
static uint8_t s_mode8_yaw_index = 0U;

static void FC_Mode8_LimitVector(float *x, float *y, float limit)
{
    float mag = sqrtf((*x) * (*x) + (*y) * (*y));
    if (mag > limit)
    {
        float scale = limit / mag;
        *x *= scale;
        *y *= scale;
    }
}

static void FC_Mode8_AddLampEstimate(float cx, float cy,
                                     float x_weight, float y_weight,
                                     float *x_sum, float *y_sum,
                                     float *weight_x_sum, float *weight_y_sum)
{
    *x_sum += cx * x_weight;
    *y_sum += cy * y_weight;
    *weight_x_sum += x_weight;
    *weight_y_sum += y_weight;
}

static uint8_t FC_Mode8_FuseCarLamp(float *cx, float *cy)
{
    const car_lamp_data *front_lamp = &image_data[Front].car_lamp_data[0];
    const car_lamp_data *center_lamp = &image_data[Center].car_lamp_data[0];
    const car_lamp_data *back_lamp = &image_data[Back].car_lamp_data[0];
    const uint8_t center_valid = (center_lamp->valid != 0U) ? 1U : 0U;
    float x_sum = 0.0f;
    float y_sum = 0.0f;
    float weight_x_sum = 0.0f;
    float weight_y_sum = 0.0f;
    float side_x_sum = 0.0f;
    float side_y_sum = 0.0f;
    float side_weight_x_sum = 0.0f;
    float side_weight_y_sum = 0.0f;

    if (center_valid != 0U)
    {
        FC_Mode8_AddLampEstimate(center_lamp->cx, center_lamp->cy,
                                 s_mode8_center_weight, s_mode8_center_weight,
                                 &x_sum, &y_sum, &weight_x_sum, &weight_y_sum);
    }

    if (front_lamp->valid != 0U)
    {
        float front_center_cx = -8.902509f + 1.016973f * front_lamp->cx + 0.031254f * front_lamp->cy;
        float front_center_cy = -51.990433f - 0.020303f * front_lamp->cx + 1.003250f * front_lamp->cy;

        FC_Mode8_AddLampEstimate(front_center_cx, front_center_cy,
                                 s_mode8_front_x_weight, s_mode8_front_y_weight,
                                 &side_x_sum, &side_y_sum, &side_weight_x_sum, &side_weight_y_sum);
    }

    if (back_lamp->valid != 0U)
    {
        float back_center_cx = 3.636157f - 0.996270f * back_lamp->cx - 0.085060f * back_lamp->cy;
        float back_center_cy = 25.907502f + 0.111016f * back_lamp->cx - 1.074126f * back_lamp->cy;

        FC_Mode8_AddLampEstimate(back_center_cx, back_center_cy,
                                 s_mode8_back_x_weight, s_mode8_back_y_weight,
                                 &side_x_sum, &side_y_sum, &side_weight_x_sum, &side_weight_y_sum);
    }

    if (center_valid != 0U)
    {
        float side_x_scale = 1.0f;
        float side_y_scale = 1.0f;

        if (side_weight_x_sum > s_mode8_side_weight_cap)
        {
            side_x_scale = s_mode8_side_weight_cap / side_weight_x_sum;
        }
        if (side_weight_y_sum > s_mode8_side_weight_cap)
        {
            side_y_scale = s_mode8_side_weight_cap / side_weight_y_sum;
        }

        x_sum += side_x_sum * side_x_scale;
        y_sum += side_y_sum * side_y_scale;
        weight_x_sum += side_weight_x_sum * side_x_scale;
        weight_y_sum += side_weight_y_sum * side_y_scale;
    }
    else
    {
        x_sum = side_x_sum;
        y_sum = side_y_sum;
        weight_x_sum = side_weight_x_sum;
        weight_y_sum = side_weight_y_sum;
    }

    if ((weight_x_sum <= 0.0f) || (weight_y_sum <= 0.0f))
    {
        return 0U;
    }

    *cx = x_sum / weight_x_sum;
    *cy = y_sum / weight_y_sum;
    return 1U;
}

static void FC_Mode8_UpdateYawTarget(void)
{
    static const float yaw_targets[] = {
        0.0f, 30.0f, 60.0f, 90.0f, 45.0f,
        0.0f, -45.0f, -90.0f, -60.0f, -30.0f};

    yaw_angle_target = yaw_targets[s_mode8_yaw_index];
    if (++s_mode8_yaw_tick >= 400U)
    {
        s_mode8_yaw_tick = 0U;
        s_mode8_yaw_index++;
        if (s_mode8_yaw_index >= (sizeof(yaw_targets) / sizeof(yaw_targets[0])))
        {
            s_mode8_yaw_index = 0U;
        }
    }
}

void FC_Mode8_Init(void)
{
    PID_Init(&s_mode8_imgx_pid,
             g_fc_params.mode8_img_x_kp, g_fc_params.mode8_img_x_ki, g_fc_params.mode8_img_x_kd,
             g_fc_params.mode8_img_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.mode8_img_x_i_limit, g_fc_params.mode8_img_x_d_lpf);
    PID_Init(&s_mode8_imgy_pid,
             g_fc_params.mode8_img_y_kp, g_fc_params.mode8_img_y_ki, g_fc_params.mode8_img_y_kd,
             g_fc_params.mode8_img_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.mode8_img_y_i_limit, g_fc_params.mode8_img_y_d_lpf);
    PID_Init(&g_mode8_velx_pid,
             g_fc_params.mode8_vel_x_kp, g_fc_params.mode8_vel_x_ki, g_fc_params.mode8_vel_x_kd,
             0.0f, g_fc_params.vel_xy_dt,
             g_fc_params.mode8_vel_x_i_limit, g_fc_params.mode8_vel_x_d_lpf);
    PID_Init(&g_mode8_vely_pid,
             g_fc_params.mode8_vel_y_kp, g_fc_params.mode8_vel_y_ki, g_fc_params.mode8_vel_y_kd,
             0.0f, g_fc_params.vel_xy_dt,
             g_fc_params.mode8_vel_y_i_limit, g_fc_params.mode8_vel_y_d_lpf);
    g_mode8_velx_pid.aw_enable = 1U;
    g_mode8_velx_pid.aw_gain = 0.15f;
    g_mode8_vely_pid.aw_enable = 1U;
    g_mode8_vely_pid.aw_gain = 0.15f;
    FC_Mode8_Reset();
}

void FC_Mode8_Reset(void)
{
    PID_Reset(&s_mode8_imgx_pid);
    PID_Reset(&s_mode8_imgy_pid);
    PID_Reset(&g_mode8_velx_pid);
    PID_Reset(&g_mode8_vely_pid);
    g_mode8_velx_target = 0.0f;
    g_mode8_vely_target = 0.0f;
    s_mode8_accel_x = 0.0f;
    s_mode8_accel_y = 0.0f;
    s_mode8_yaw_tick = 0U;
    s_mode8_yaw_index = 0U;
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    yaw_angle_target = 0.0f;
}

void FC_Mode8_100Hz(void)
{
    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        s_mode8_yaw_tick = 0U;
        s_mode8_yaw_index = 0U;
        yaw_angle_target = 0.0f;
        return;
    }

    // 这个是自动修改yaw的目标,是为了调节yaw,所以临时写的自动修改yaw目标,注释掉以后yaw目标就是0! 不允许对这部分代码修改,就这么保留注释!
    // FC_Mode8_UpdateYawTarget();
}

void FC_Mode8_50Hz(float dt)
{
    float velx_sp = 0.0f;
    float vely_sp = 0.0f;
    float velx_sp_raw = 0.0f;
    float vely_sp_raw = 0.0f;
    float accx_sp;
    float accy_sp;
    float velx_ff;
    float vely_ff;
    float velx_out;
    float vely_out;
    float img_err_x = 0.0f;
    float img_err_y = 0.0f;
    float img_fb_x = 0.0f;
    float img_fb_y = 0.0f;
    float roll_trim;
    float pitch_trim;
    float fused_lamp_cx = 0.0f;
    float fused_lamp_cy = 0.0f;
    uint8_t fused_lamp_valid;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        Beep_Disable();
        FC_Mode8_Reset();
        return;
    }

    fused_lamp_valid = FC_Mode8_FuseCarLamp(&fused_lamp_cx, &fused_lamp_cy);

    if (fused_lamp_valid != 0U)
    {
        Beep_Disable();
    }
    else
    {
        Beep_Enable();
    }

    if ((fused_lamp_valid != 0U) &&
        (0U != g_tof_fused_valid) &&
        (g_tof_fused_height_mm > 500.0f))
    {
        img_err_x = fused_lamp_cx;
        img_err_y = fused_lamp_cy + s_mode8_img_y_offset;
        img_fb_x = PID_Update(&s_mode8_imgx_pid, 0.0f, -img_err_x, dt);
        img_fb_y = PID_Update(&s_mode8_imgy_pid, 0.0f, -img_err_y, dt);
        img_fb_x = FC_Mode_Clamp(img_fb_x, -s_mode8_img_fb_limit_cmps, s_mode8_img_fb_limit_cmps);
        img_fb_y = FC_Mode_Clamp(img_fb_y, -s_mode8_img_fb_limit_cmps, s_mode8_img_fb_limit_cmps);

        velx_sp = img_fb_x + g_car_vel_x * 1.0f * g_fc_params.mode8_kp_car_x;
        vely_sp = img_fb_y + g_car_vel_y * -1.0f * g_fc_params.mode8_kp_car_y;
    }
    else
    {
        PID_Reset(&s_mode8_imgx_pid);
        PID_Reset(&s_mode8_imgy_pid);
    }
    // wifi_justfloat(g_car_vel_x, g_car_vel_y,
    //                img_fb_x, img_fb_y,
    //                velx_sp, vely_sp,
    //                g_mode8_velx_target, g_mode8_vely_target,
    //                roll_angle_target, pitch_angle_target);
    velx_sp_raw = velx_sp;
    vely_sp_raw = vely_sp;
    FC_Mode8_LimitVector(&velx_sp, &vely_sp, s_mode8_vel_limit_cmps);

    accx_sp = (velx_sp - g_mode8_velx_target) / dt;
    accy_sp = (vely_sp - g_mode8_vely_target) / dt;
    FC_Mode8_LimitVector(&accx_sp, &accy_sp, s_mode8_vel_accel_cmps2);
    accx_sp -= s_mode8_accel_x;
    accy_sp -= s_mode8_accel_y;
    FC_Mode8_LimitVector(&accx_sp, &accy_sp, s_mode8_vel_jerk_cmps3 * dt);
    s_mode8_accel_x += accx_sp;
    s_mode8_accel_y += accy_sp;
    g_mode8_velx_target += s_mode8_accel_x * dt;
    g_mode8_vely_target += s_mode8_accel_y * dt;

    if (((velx_sp - (g_mode8_velx_target - s_mode8_accel_x * dt)) * (velx_sp - g_mode8_velx_target) +
         (vely_sp - (g_mode8_vely_target - s_mode8_accel_y * dt)) * (vely_sp - g_mode8_vely_target)) <= 0.0f)
    {
        g_mode8_velx_target = velx_sp;
        g_mode8_vely_target = vely_sp;
        s_mode8_accel_x = 0.0f;
        s_mode8_accel_y = 0.0f;
    }

    roll_trim = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_trim = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    velx_ff = FC_Mode_Clamp(g_fc_params.mode8_vel_x_kff * s_mode8_accel_x,
                            -s_mode8_angle_limit_deg, s_mode8_angle_limit_deg);
    vely_ff = FC_Mode_Clamp(g_fc_params.mode8_vel_y_kff * s_mode8_accel_y,
                            -s_mode8_angle_limit_deg, s_mode8_angle_limit_deg);

    g_mode8_velx_pid.output_min = -s_mode8_angle_limit_deg - roll_trim - velx_ff;
    g_mode8_velx_pid.output_max = s_mode8_angle_limit_deg - roll_trim - velx_ff;
    g_mode8_vely_pid.output_min = -s_mode8_angle_limit_deg - pitch_trim - vely_ff;
    g_mode8_vely_pid.output_max = s_mode8_angle_limit_deg - pitch_trim - vely_ff;

    velx_out = PID_Update(&g_mode8_velx_pid, g_mode8_velx_target, -Pos_Est_vel_x, dt) + velx_ff;
    vely_out = PID_Update(&g_mode8_vely_pid, g_mode8_vely_target, -Pos_Est_vel_y, dt) + vely_ff;
    g_mode8_velx_pid.ff_term = velx_ff;
    g_mode8_vely_pid.ff_term = vely_ff;

    roll_angle_target = FC_Mode_Clamp(velx_out + roll_trim, -s_mode8_angle_limit_deg, s_mode8_angle_limit_deg);
    pitch_angle_target = FC_Mode_Clamp(vely_out + pitch_trim, -s_mode8_angle_limit_deg, s_mode8_angle_limit_deg);

    wifi_justfloat(g_car_vel_x, g_car_vel_y,
                 Pos_Est_vel_x, Pos_Est_vel_y,
                 fused_lamp_cx, fused_lamp_cy,
                 img_err_x, img_err_y,
                 img_fb_x, img_fb_y,
                 velx_sp_raw, vely_sp_raw,
                 velx_sp, vely_sp,
                 g_mode8_velx_target, g_mode8_vely_target,
                 s_mode8_accel_x, s_mode8_accel_y,
                 roll_angle_target, pitch_angle_target,
                 g_euler.roll, g_euler.pitch,
                 opflow_vel_x, opflow_vel_y,
                 opflow_vel_x_lpf, opflow_vel_y_lpf,
                 g_mode8_velx_pid.p_term, g_mode8_velx_pid.i_term,
                 g_mode8_velx_pid.d_term, g_mode8_velx_pid.output,
                 g_mode8_vely_pid.p_term, g_mode8_vely_pid.i_term,
                 (float)g_tof_fused_valid,
                 g_tof_fused_height_mm,
                 velx_ff, vely_ff);
}
