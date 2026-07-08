#include "fc_mode.h"
#include "yaw_align.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../Image/image_data.h"
#include "../Planner/car_lamp_fused.h"
#include "../Protocols/wifi/wifi_justfloat/wifi_justfloat.h"
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
static const float s_mode8_down_proj_x_bias = 0.0f;
static const float s_mode8_down_proj_x_roll_k = 1.408988f;
static const float s_mode8_down_proj_x_pitch_k = 0.015998f;
static const float s_mode8_down_proj_y_bias = -19.863429f;
static const float s_mode8_down_proj_y_roll_k = -0.064165f;
static const float s_mode8_down_proj_y_pitch_k = 1.377711f;
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

static void FC_Mode8_GetDownProjectionCenter(float *cx, float *cy)
{
    *cx = s_mode8_down_proj_x_bias +
          s_mode8_down_proj_x_roll_k * g_euler.roll +
          s_mode8_down_proj_x_pitch_k * g_euler.pitch;
    *cy = s_mode8_down_proj_y_bias +
          s_mode8_down_proj_y_roll_k * g_euler.roll +
          s_mode8_down_proj_y_pitch_k * g_euler.pitch;
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
    YawAlign_Reset();
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    yaw_angle_target = g_euler.yaw;
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
    float down_proj_cx = 0.0f;
    float down_proj_cy = 0.0f;
    float car_ff_x = 0.0f;
    float car_ff_y = 0.0f;
    uint8_t fused_lamp_valid;
    uint8_t tof_height_valid;
    uint8_t yaw_align_active;
    yaw_align_debug_t yaw_debug;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        Beep_Disable();
        FC_Mode8_Reset();
        return;
    }

    yaw_align_active = YawAlign_Update();

    fused_lamp_valid = g_car_lamp_fused.valid;
    fused_lamp_cx = g_car_lamp_fused.cx;
    fused_lamp_cy = g_car_lamp_fused.cy;

    if (fused_lamp_valid != 0U)
    {
        Beep_Disable();
    }
    else
    {
        Beep_Enable();
    }

    tof_height_valid = ((0U != g_tof_fused_valid) && (g_tof_fused_height_mm > 500.0f)) ? 1U : 0U;

    if ((fused_lamp_valid != 0U) && (tof_height_valid != 0U))
    {
        FC_Mode8_GetDownProjectionCenter(&down_proj_cx, &down_proj_cy);
        img_err_x = fused_lamp_cx - down_proj_cx;
        img_err_y = fused_lamp_cy - down_proj_cy;
        img_fb_x = PID_Update(&s_mode8_imgx_pid, 0.0f, -img_err_x, dt);
        img_fb_y = PID_Update(&s_mode8_imgy_pid, 0.0f, -img_err_y, dt);
        img_fb_x = FC_Mode_Clamp(img_fb_x, -s_mode8_img_fb_limit_cmps, s_mode8_img_fb_limit_cmps);
        img_fb_y = FC_Mode_Clamp(img_fb_y, -s_mode8_img_fb_limit_cmps, s_mode8_img_fb_limit_cmps);
    }
    else
    {
        PID_Reset(&s_mode8_imgx_pid);
        PID_Reset(&s_mode8_imgy_pid);
    }

    car_ff_x = g_car_vel_x * g_fc_params.mode8_kp_car_x;
    car_ff_y = -g_car_vel_y * g_fc_params.mode8_kp_car_y;
    velx_sp = img_fb_x + car_ff_x;
    vely_sp = img_fb_y + car_ff_y;
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
    YawAlign_GetDebug(&yaw_debug);

    // wifi_justfloat(g_car_vel_x,                 /* I1 */
    //              g_car_vel_y,                   /* I2 */
    //              Pos_Est_vel_x,                 /* I3 */
    //              Pos_Est_vel_y,                 /* I4 */
    //              fused_lamp_cx,                 /* I5 */
    //              fused_lamp_cy,                 /* I6 */
    //              img_err_x,                     /* I7 */
    //              img_err_y,                     /* I8 */
    //              img_fb_x,                      /* I9 */
    //              img_fb_y,                      /* I10 */
    //              velx_sp_raw,                   /* I11 */
    //              vely_sp_raw,                   /* I12 */
    //              velx_sp,                       /* I13 */
    //              vely_sp,                       /* I14 */
    //              g_mode8_velx_target,           /* I15 */
    //              g_mode8_vely_target,           /* I16 */
    //              s_mode8_accel_x,               /* I17 */
    //              s_mode8_accel_y,               /* I18 */
    //              roll_angle_target,             /* I19 */
    //              pitch_angle_target,            /* I20 */
    //              g_euler.roll,                  /* I21 */
    //              g_euler.pitch,                 /* I22 */
    //              opflow_vel_x,                  /* I23 */
    //              opflow_vel_y,                  /* I24 */
    //              opflow_vel_x_lpf,              /* I25 */
    //              opflow_vel_y_lpf,              /* I26 */
    //              g_mode8_velx_pid.p_term,       /* I27 */
    //              g_mode8_velx_pid.i_term,       /* I28 */
    //              g_mode8_velx_pid.d_term,       /* I29 */
    //              g_mode8_velx_pid.output,       /* I30 */
    //              g_mode8_vely_pid.p_term,       /* I31 */
    //              g_mode8_vely_pid.i_term,       /* I32 */
    //              (float)g_tof_fused_valid,      /* I33 */
    //              g_tof_fused_height_mm,         /* I34 */
    //              velx_ff,                       /* I35 */
    //              vely_ff);                      /* I36 */
    // wifi_justfloat(image_data[Front].beacon_data[0].x,     /* I1 */
    //                image_data[Front].beacon_data[0].y,     /* I2 */
    //                image_data[Front].beacon_data[1].x,     /* I3 */
    //                image_data[Front].beacon_data[1].y,     /* I4 */
    //                image_data[Front].beacon_data[2].x,     /* I5 */
    //                image_data[Front].beacon_data[2].y,     /* I6 */
    //                image_data[Front].car_lamp_data[0].cx,  /* I7 */
    //                image_data[Front].car_lamp_data[0].cy,  /* I8 */
    //                image_data[Front].car_lamp_data[1].cx,  /* I9 */
    //                image_data[Front].car_lamp_data[1].cy,  /* I10 */
    //                image_data[Center].beacon_data[0].x,    /* I11 */
    //                image_data[Center].beacon_data[0].y,    /* I12 */
    //                image_data[Center].beacon_data[1].x,    /* I13 */
    //                image_data[Center].beacon_data[1].y,    /* I14 */
    //                image_data[Center].beacon_data[2].x,    /* I15 */
    //                image_data[Center].beacon_data[2].y,    /* I16 */
    //                image_data[Center].car_lamp_data[0].cx, /* I17 */
    //                image_data[Center].car_lamp_data[0].cy, /* I18 */
    //                image_data[Center].car_lamp_data[1].cx, /* I19 */
    //                image_data[Center].car_lamp_data[1].cy, /* I20 */
    //                image_data[Back].beacon_data[0].x,      /* I21 */
    //                image_data[Back].beacon_data[0].y,      /* I22 */
    //                image_data[Back].beacon_data[1].x,      /* I23 */
    //                image_data[Back].beacon_data[1].y,      /* I24 */
    //                image_data[Back].beacon_data[2].x,      /* I25 */
    //                image_data[Back].beacon_data[2].y,      /* I26 */
    //                image_data[Back].car_lamp_data[0].cx,   /* I27 */
    //                image_data[Back].car_lamp_data[0].cy,   /* I28 */
    //                image_data[Back].car_lamp_data[1].cx,   /* I29 */
    //                image_data[Back].car_lamp_data[1].cy,   /* I30 */
    //                (float)yaw_align_active,                /* I31 */
    //                (float)yaw_debug.locked,                /* I32 */
    //                (float)yaw_debug.locked_beacon.camera,  /* I33 */
    //                yaw_debug.locked_beacon.x,              /* I34 */
    //                yaw_debug.locked_beacon.y,              /* I35 */
    //                g_euler.yaw,                            /* I36 */
    //                yaw_angle_target,                       /* I37 */
    //                yaw_gyro_target,                        /* I38 */
    //                yaw_gyro_pid.output);                   /* I39 */
}
