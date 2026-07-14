#include "fc_mode.h"
#include "yaw_align.h"
#include "../Estimation/Pos_Est/Pos_Est.h"
#include "../Estimation/Height_Est/Height_Est.h"
#include "../Image/image_data.h"
#include "../Planner/pix_to_distance.h"
#include "../Protocols/wifi/wifi_justfloat/wifi_justfloat.h"
#include <math.h>

extern float g_car_vel_x;
extern float g_car_vel_y;
extern float g_car_yaw;
extern float g_car_sync_time_ms;

pid_t g_mode2_imgx_pid; /* 模式2图像X轴位置环PID状态，供控制与调试访问。 */
pid_t g_mode2_imgy_pid; /* 模式2图像Y轴位置环PID状态，供控制与调试访问。 */
pid_t g_mode2_velx_pid;
pid_t g_mode2_vely_pid;
float g_mode2_velx_target = 0.0f;
float g_mode2_vely_target = 0.0f;

static float s_mode2_prev_velx_target = 0.0f;
static float s_mode2_prev_vely_target = 0.0f;
static float s_mode2_velx_ff_lpf = 0.0f;
static float s_mode2_vely_ff_lpf = 0.0f;
static uint16_t s_mode2_yaw_tick = 0U;
static uint8_t s_mode2_yaw_index = 0U;

static void FC_Mode2_UpdateImgYKp(float dt, uint8_t reset)
{
    static float hold_time_s = 0.0f;
    const float kp_base = g_fc_params.mode2_img_y_kp;
    const float kp_max = 2.31948906f;
    float speed_y;
    float kp_target;

    if (reset != 0U)
    {
        hold_time_s = 0.0f;
        g_mode2_imgy_pid.kp = kp_base;
        return;
    }

    speed_y = fabsf(g_car_vel_y);
    if (speed_y <= 0.6f)
    {
        kp_target = kp_base;
    }
    else if (speed_y < 1.4f)
    {
        kp_target = kp_base + (kp_max - kp_base) * 0.5f * (speed_y - 0.6f) / 0.8f;
    }
    else if (speed_y < 1.6f)
    {
        kp_target = kp_base + (kp_max - kp_base) *
                                  (0.5f + 0.5f * (speed_y - 1.4f) / 0.2f);
    }
    else
    {
        kp_target = kp_max;
    }

    if (kp_target >= g_mode2_imgy_pid.kp)
    {
        g_mode2_imgy_pid.kp = kp_target;
        hold_time_s = 0.6f;
    }
    else if (hold_time_s > 0.0f)
    {
        hold_time_s -= dt;
    }
    else
    {
        g_mode2_imgy_pid.kp -= 0.5f * dt;
        if (g_mode2_imgy_pid.kp < kp_target)
        {
            g_mode2_imgy_pid.kp = kp_target;
        }
    }
}

static void FC_Mode2_UpdateYawTarget(void)
{
    static const float yaw_targets[] = {
        0.0f, 30.0f, 60.0f, 90.0f, 45.0f,
        0.0f, -45.0f, -90.0f, -60.0f, -30.0f};

    yaw_angle_target = yaw_targets[s_mode2_yaw_index];
    if (++s_mode2_yaw_tick >= 400U)
    {
        s_mode2_yaw_tick = 0U;
        s_mode2_yaw_index++;
        if (s_mode2_yaw_index >= (sizeof(yaw_targets) / sizeof(yaw_targets[0])))
        {
            s_mode2_yaw_index = 0U;
        }
    }
}

void FC_Mode2_Init(void)
{
    PID_Init(&g_mode2_imgx_pid,
             g_fc_params.mode2_img_x_kp, g_fc_params.mode2_img_x_ki, g_fc_params.mode2_img_x_kd,
             g_fc_params.mode2_img_x_kff, g_fc_params.vel_xy_dt,
             g_fc_params.mode2_img_x_i_limit, g_fc_params.mode2_img_x_d_lpf);
    PID_Init(&g_mode2_imgy_pid,
             g_fc_params.mode2_img_y_kp, g_fc_params.mode2_img_y_ki, g_fc_params.mode2_img_y_kd,
             g_fc_params.mode2_img_y_kff, g_fc_params.vel_xy_dt,
             g_fc_params.mode2_img_y_i_limit, g_fc_params.mode2_img_y_d_lpf);
    PID_Init(&g_mode2_velx_pid,
             g_fc_params.mode2_vel_x_kp, g_fc_params.mode2_vel_x_ki, g_fc_params.mode2_vel_x_kd,
             0.0f, g_fc_params.vel_xy_dt,
             g_fc_params.mode2_vel_x_i_limit, g_fc_params.mode2_vel_x_d_lpf);
    PID_Init(&g_mode2_vely_pid,
             g_fc_params.mode2_vel_y_kp, g_fc_params.mode2_vel_y_ki, g_fc_params.mode2_vel_y_kd,
             0.0f, g_fc_params.vel_xy_dt,
             g_fc_params.mode2_vel_y_i_limit, g_fc_params.mode2_vel_y_d_lpf);
    g_mode2_velx_pid.aw_enable = 1U;
    g_mode2_velx_pid.aw_gain = 0.15f;
    g_mode2_vely_pid.aw_enable = 1U;
    g_mode2_vely_pid.aw_gain = 0.15f;
    FC_Mode2_Reset();
}

void FC_Mode2_Reset(void)
{
    PID_Reset(&g_mode2_imgx_pid);
    PID_Reset(&g_mode2_imgy_pid);
    FC_Mode2_UpdateImgYKp(0.0f, 1U);
    PID_Reset(&g_mode2_velx_pid);
    PID_Reset(&g_mode2_vely_pid);
    g_mode2_velx_target = 0.0f;
    g_mode2_vely_target = 0.0f;
    s_mode2_prev_velx_target = 0.0f;
    s_mode2_prev_vely_target = 0.0f;
    s_mode2_velx_ff_lpf = 0.0f;
    s_mode2_vely_ff_lpf = 0.0f;
    s_mode2_yaw_tick = 0U;
    s_mode2_yaw_index = 0U;
    YawAlign_Reset();
    roll_angle_target = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_angle_target = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    yaw_angle_target = g_euler.yaw;
}

void FC_Mode2_100Hz(void)
{
    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        s_mode2_yaw_tick = 0U;
        s_mode2_yaw_index = 0U;
        yaw_angle_target = 0.0f;
        return;
    }

    // 这个是自动修改yaw的目标,是为了调节yaw,所以临时写的自动修改yaw目标,注释掉以后yaw目标就是0! 不允许对这部分代码修改,就这么保留注释!
    // FC_Mode2_UpdateYawTarget();
}

void FC_Mode2_50Hz(float dt)
{
    float velx_sp = 0.0f;
    float vely_sp = 0.0f;
    float velx_ff;
    float vely_ff;
    float velx_target_rate;
    float vely_target_rate;
    float velx_out;
    float vely_out;
    float img_err_x = 0.0f;
    float img_err_y = 0.0f;
    float img_fb_x = 0.0f;
    float img_fb_y = 0.0f;
    float roll_trim;
    float pitch_trim;
    float car_ff_x = 0.0f;
    float car_ff_y = 0.0f;
    uint8_t fused_lamp_valid;
    uint8_t tof_height_valid;
    uint8_t yaw_align_active;
    yaw_align_debug_t yaw_debug;

    if (FC_START_CRSF_Get_State() != FC_START_CRSF_STATE_FLYING)
    {
        Beep_Disable();
        FC_Mode2_Reset();
        return;
    }

    yaw_align_active = YawAlign_Update();

    fused_lamp_valid = g_car_lamp_fused_distance_projectioncenter_2.valid;

    if (fused_lamp_valid != 0U)
    {
        Beep_Disable();
    }
    else
    {
        Beep_Enable();
    }

    tof_height_valid = ((0U != g_tof_fused_valid) &&
                        (g_tof_fused_height_mm > FC_MODE_IMAGE_MIN_HEIGHT_MM)) ? 1U : 0U;

    FC_Mode2_UpdateImgYKp(dt, 0U);
    if ((fused_lamp_valid != 0U) && (tof_height_valid != 0U))
    {
        img_err_x = g_car_lamp_fused_distance_projectioncenter_2.x_cm;
        img_err_y = g_car_lamp_fused_distance_projectioncenter_2.y_cm;
        img_fb_x = PID_Update(&g_mode2_imgx_pid, 0.0f, -img_err_x, dt);
        img_fb_y = PID_Update(&g_mode2_imgy_pid, 0.0f, -img_err_y, dt);
        img_fb_x = FC_Mode_Clamp(img_fb_x, -FC_MODE_IMAGE_VEL_LIMIT_CMPS, FC_MODE_IMAGE_VEL_LIMIT_CMPS);
        img_fb_y = FC_Mode_Clamp(img_fb_y, -FC_MODE_IMAGE_VEL_LIMIT_CMPS, FC_MODE_IMAGE_VEL_LIMIT_CMPS);
    }
    else
    {
        PID_Reset(&g_mode2_imgx_pid);
        PID_Reset(&g_mode2_imgy_pid);
    }

    car_ff_x = g_car_vel_x * g_fc_params.mode2_kp_car_x;
    car_ff_y = FC_Mode_Clamp(-g_car_vel_y * g_fc_params.mode2_kp_car_y,
                             -87.670937f, 87.670937f);
    velx_sp = img_fb_x + car_ff_x;
    vely_sp = img_fb_y + car_ff_y;
    // wifi_justfloat(g_car_vel_x, g_car_vel_y,
    //                img_fb_x, img_fb_y,
    //                velx_sp, vely_sp,
    //                g_mode2_velx_target, g_mode2_vely_target,
    //                roll_angle_target, pitch_angle_target);
    velx_target_rate = (velx_sp - s_mode2_prev_velx_target) / dt;
    vely_target_rate = (vely_sp - s_mode2_prev_vely_target) / dt;
    g_mode2_velx_target = velx_sp;
    g_mode2_vely_target = vely_sp;
    s_mode2_prev_velx_target = g_mode2_velx_target;
    s_mode2_prev_vely_target = g_mode2_vely_target;

    roll_trim = FC_Mode_Get_Roll_Mech_Trim_Deg();
    pitch_trim = FC_Mode_Get_Pitch_Mech_Trim_Deg();
    velx_ff = FC_Mode_Clamp(g_fc_params.mode2_vel_x_kff * velx_target_rate,
                            -FC_MODE_XY_ANGLE_LIMIT_DEG, FC_MODE_XY_ANGLE_LIMIT_DEG);
    vely_ff = FC_Mode_Clamp(g_fc_params.mode2_vel_y_kff * vely_target_rate,
                            -FC_MODE_XY_ANGLE_LIMIT_DEG, FC_MODE_XY_ANGLE_LIMIT_DEG);
    s_mode2_velx_ff_lpf += FC_MODE_VEL_KFF_LPF_ALPHA * (velx_ff - s_mode2_velx_ff_lpf);
    s_mode2_vely_ff_lpf += FC_MODE_VEL_KFF_LPF_ALPHA * (vely_ff - s_mode2_vely_ff_lpf);
    velx_ff = s_mode2_velx_ff_lpf;
    vely_ff = s_mode2_vely_ff_lpf;

    g_mode2_velx_pid.output_min = -FC_MODE_XY_ANGLE_LIMIT_DEG - roll_trim - velx_ff;
    g_mode2_velx_pid.output_max = FC_MODE_XY_ANGLE_LIMIT_DEG - roll_trim - velx_ff;
    g_mode2_vely_pid.output_min = -FC_MODE_XY_ANGLE_LIMIT_DEG - pitch_trim - vely_ff;
    g_mode2_vely_pid.output_max = FC_MODE_XY_ANGLE_LIMIT_DEG - pitch_trim - vely_ff;

    velx_out = PID_Update(&g_mode2_velx_pid, g_mode2_velx_target, -Pos_Est_vel_x, dt) + velx_ff;
    vely_out = PID_Update(&g_mode2_vely_pid, g_mode2_vely_target, -Pos_Est_vel_y, dt) + vely_ff;
    g_mode2_velx_pid.ff_term = velx_ff;
    g_mode2_vely_pid.ff_term = vely_ff;

    roll_angle_target = FC_Mode_Clamp(velx_out + roll_trim, -FC_MODE_XY_ANGLE_LIMIT_DEG, FC_MODE_XY_ANGLE_LIMIT_DEG);
    pitch_angle_target = FC_Mode_Clamp(vely_out + pitch_trim, -FC_MODE_XY_ANGLE_LIMIT_DEG, FC_MODE_XY_ANGLE_LIMIT_DEG);
    YawAlign_GetDebug(&yaw_debug);

    // wifi_justfloat(g_car_vel_x,                 /* I1 */
    //              g_car_vel_y,                   /* I2 */
    //              Pos_Est_vel_x,                 /* I3 */
    //              Pos_Est_vel_y,                 /* I4 */
    //              g_car_lamp_fused_distance_projectioncenter_2.x_cm,/* I5 */
    //              g_car_lamp_fused_distance_projectioncenter_2.y_cm,/* I6 */
    //              img_err_x,                     /* I7 */
    //              img_err_y,                     /* I8 */
    //              img_fb_x,                      /* I9 */
    //              img_fb_y,                      /* I10 */
    //              velx_sp,                       /* I11 */
    //              vely_sp,                       /* I12 */
    //              g_mode2_velx_target,           /* I13 */
    //              g_mode2_vely_target,           /* I14 */
    //              roll_angle_target,             /* I15 */
    //              pitch_angle_target,            /* I16 */
    //              g_euler.roll,                  /* I17 */
    //              g_euler.pitch,                 /* I18 */
    //              opflow_vel_x,                  /* I19 */
    //              opflow_vel_y,                  /* I20 */
    //              opflow_vel_x_lpf,              /* I21 */
    //              opflow_vel_y_lpf,              /* I22 */
    //              g_mode2_velx_pid.p_term,       /* I23 */
    //              g_mode2_velx_pid.i_term,       /* I24 */
    //              g_mode2_velx_pid.d_term,       /* I25 */
    //              g_mode2_velx_pid.output,       /* I26 */
    //              g_mode2_vely_pid.p_term,       /* I27 */
    //              g_mode2_vely_pid.i_term,       /* I28 */
    //              (float)g_tof_fused_valid,      /* I29 */
    //              g_tof_fused_height_mm,         /* I30 */
    //              velx_ff,                       /* I31 */
    //              vely_ff);                      /* I32 */
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
//     wifi_justfloat(g_euler.roll, g_euler.pitch, g_euler.yaw,
//                    roll_angle_target, pitch_angle_target, yaw_angle_target,
//                    g_tof_fused_height_mm,g_height_fused_vz_mps,
//                    g_car_vel_x, g_car_vel_y,g_car_yaw,
//                    Pos_Est_vel_x, Pos_Est_vel_y,
//                    g_mode2_velx_target, g_mode2_vely_target,
//                    g_car_lamp_fused_distance_projectioncenter_2.x_cm,
//                    g_car_lamp_fused_distance_projectioncenter_2.y_cm,
//                    velx_ff, vely_ff,
//                    g_mode2_velx_pid.p_term, g_mode2_velx_pid.i_term,
//                    g_mode2_vely_pid.p_term, g_mode2_vely_pid.i_term,
//                    g_mode2_imgx_pid.p_term, g_mode2_imgx_pid.i_term,g_mode2_imgx_pid.d_term,
//                    g_mode2_imgy_pid.p_term, g_mode2_imgy_pid.i_term,g_mode2_imgy_pid.d_term,
//                    car_ff_x, car_ff_y,
//                    (float)g_car_lamp_fused_distance_projectioncenter_2.valid,
//                    g_mode2_imgy_pid.kp,
//                    g_mode2_velx_pid.d_term, g_mode2_vely_pid.d_term,
//                    g_car_sync_time_ms
//                    );
}

float FC_Mode2_Get_Fixed_Height_M(void)
{
    return 1.1f;
}
