/*****************************************************************************
 * 文件: fc_params.c
 * 模块: 飞控 - 参数管理实现
 * 职责: 管理默认参数、运行时参数以及 Flash 持久化
 *****************************************************************************/

#include "fc_params.h"

#include <stddef.h>
#include <string.h>
#include "zf_common_headfile.h"

#define FC_PARAMS_FLASH_MAGIC                  (0x46504346UL)
#define FC_PARAMS_FLASH_VERSION                (8U)
#define FC_PARAMS_FLASH_MIN_COMPAT_VERSION     (5U)

#define FC_PARAMS_FLASH_V5_V6_SIZE_BYTES       (340U)
#define FC_PARAMS_FLASH_V7_SIZE_BYTES          (388U)
#define FC_PARAMS_FLASH_V8_SIZE_BYTES          (596U)

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t checksum;
    fc_params_t params;
} fc_params_flash_blob_t;

/* 固化旧版参数前缀布局，并确保完整参数块不超过单个 Flash 页。 */
typedef char fc_params_v5_v6_layout_must_match[
    (offsetof(fc_params_t, mode8_img_x_kp) == FC_PARAMS_FLASH_V5_V6_SIZE_BYTES) ? 1 : -1];
typedef char fc_params_v7_layout_must_match[
    (offsetof(fc_params_t, mode5_img_x_kp) == FC_PARAMS_FLASH_V7_SIZE_BYTES) ? 1 : -1];
typedef char fc_params_v8_layout_must_match[
    (sizeof(fc_params_t) == FC_PARAMS_FLASH_V8_SIZE_BYTES) ? 1 : -1];
typedef char fc_params_blob_must_fit_flash_page[
    (sizeof(fc_params_flash_blob_t) <= FLASH_PAGE_SIZE) ? 1 : -1];
typedef char fc_params_flash_page_must_exist[
    (FC_PARAMS_FLASH_PAGE < FLASH_PAGE_NUM) ? 1 : -1];

static uint32_t fc_params_checksum_calc(const void *data, uint32_t size)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t hash = 2166136261UL;
    uint32_t i;

    for (i = 0U; i < size; i++)
    {
        hash ^= (uint32_t)bytes[i];
        hash *= 16777619UL;
    }

    return hash;
}

static uint8_t fc_params_flash_size_is_valid(uint16_t version, uint16_t size)
{
    switch (version)
    {
        case 5U:
        case 6U:
            return (size == FC_PARAMS_FLASH_V5_V6_SIZE_BYTES) ? 1U : 0U;

        case 7U:
            return (size == FC_PARAMS_FLASH_V7_SIZE_BYTES) ? 1U : 0U;

        case FC_PARAMS_FLASH_VERSION:
            return (size == FC_PARAMS_FLASH_V8_SIZE_BYTES) ? 1U : 0U;

        default:
            return 0U;
    }
}

static uint8_t fc_params_blob_is_valid(const fc_params_flash_blob_t *blob)
{
    uint32_t checksum;

    if (NULL == blob)
    {
        return 0U;
    }

    if (blob->magic != FC_PARAMS_FLASH_MAGIC)
    {
        return 0U;
    }

    if ((blob->version < FC_PARAMS_FLASH_MIN_COMPAT_VERSION) ||
        (blob->version > FC_PARAMS_FLASH_VERSION))
    {
        return 0U;
    }

    if (0U == fc_params_flash_size_is_valid(blob->version, blob->size))
    {
        return 0U;
    }

    checksum = fc_params_checksum_calc(&blob->params, (uint32_t)blob->size);
    return (checksum == blob->checksum) ? 1U : 0U;
}

static void fc_params_migrate_loaded(fc_params_t *params, uint16_t version)
{
    if (NULL == params)
    {
        return;
    }

    /* 版本 5 已保留 Yaw 角度字段，但当时这些字段尚无有效默认值。 */
    if ((version < 6U) &&
        (params->yaw_angle_kp == 0.0f) &&
        (params->yaw_angle_ki == 0.0f) &&
        (params->yaw_angle_kd == 0.0f) &&
        (params->yaw_angle_kff == 0.0f))
    {
        params->yaw_angle_kp = 1.5f;
        params->yaw_angle_i_limit = 0.0f;
        params->yaw_angle_d_lpf = 0.0f;
    }
}

/* 飞控参数全局实例：集中保存控制周期、油门基准和各控制环 PID 参数 */
fc_params_t g_fc_params;
/* 2BL3 图传发送开关：0=关闭，1=仅非飞行发送，2=始终发送 */
volatile uint8_t g_2bl3_image_send_enable = 1U;
/*
 * 函数名: fc_params_fill_defaults
 * 功能: 向目标参数结构体写入编译期默认值
 * 输入参数:
 *   params - 目标参数结构体指针
 * 返回值: 无
 */
static void fc_params_fill_defaults(fc_params_t *params)
{
    if (NULL == params)
    {
        return;
    }

    memset(params, 0, sizeof(*params));

    /* ===== 控制周期参数 ===== */
    params->gyro_dt = 0.001f;   /* 1kHz */
    params->angle_dt = 0.002f;  /* 500Hz */
    params->pos_xy_dt = 0.02f;  /* 50Hz */
    params->pos_z_dt = 0.02f;   /* 50Hz */
    params->vel_xy_dt = 0.02f;  /* 50Hz */
    params->vel_z_dt = 0.01f;   /* 100Hz */

    /* ===== 油门与机械配平参数 ===== */
    params->base_throttle = 3200;         /* 悬停油门 */
    params->roll_mech_trim_deg = 1.3f;    /* Roll 机械配平角 */
    params->pitch_mech_trim_deg = 1.0f;   /* Pitch 机械配平角 */

    /* ===== Roll 轴角速度环参数 ===== */
    params->roll_gyro_kp = 5.4f;
    params->roll_gyro_ki = 0.18f;
    params->roll_gyro_kd = 0.010f;
    params->roll_gyro_kff = 0.0f;
    params->roll_gyro_i_limit = 180.0f;
    params->roll_gyro_d_lpf = 60.0f;

    /* ===== Pitch 轴角速度环参数 ===== */
    params->pitch_gyro_kp = 5.3f;
    params->pitch_gyro_ki = 0.14f;
    params->pitch_gyro_kd = 0.010f;
    params->pitch_gyro_kff = 0.0f;
    params->pitch_gyro_i_limit = 140.0f;
    params->pitch_gyro_d_lpf = 60.0f;

    /* ===== Yaw 轴角速度环参数 ===== */
    params->yaw_gyro_kp = 15.0f;
    params->yaw_gyro_ki = 5.0f;
    params->yaw_gyro_kd = 0.0f;
    params->yaw_gyro_kff = 0.0f;
    params->yaw_gyro_i_limit = 700.0f;
    params->yaw_gyro_d_lpf = 30.0f;

    /* ===== Roll 轴角度环参数 ===== */
    params->roll_angle_kp = 6.0f;
    params->roll_angle_ki = 0.0f;
    params->roll_angle_kd = 0.0f;
    params->roll_angle_kff = 0.02f;
    params->roll_angle_i_limit = 80.0f;
    params->roll_angle_d_lpf = 15.0f;

    /* ===== Pitch 轴角度环参数 ===== */
    params->pitch_angle_kp = 6.2f;
    params->pitch_angle_ki = 0.0f;
    params->pitch_angle_kd = 0.0f;
    params->pitch_angle_kff = 0.04f;
    params->pitch_angle_i_limit = 80.0f;
    params->pitch_angle_d_lpf = 15.0f;

    /* ===== Yaw 轴角度环参数 ===== */
    params->yaw_angle_kp = 6.0f;
    params->yaw_angle_ki = 0.0f;
    params->yaw_angle_kd = 0.0f;
    params->yaw_angle_kff = 0.0f;
    params->yaw_angle_i_limit = 0.0f;
    params->yaw_angle_d_lpf = 0.0f;

    /* ===== X 轴位置环参数 ===== */
    params->pos_x_kp = 0.90f;
    params->pos_x_ki = 0.0f;
    params->pos_x_kd = 0.0f;
    params->pos_x_kff = 0.0f;
    params->pos_x_i_limit = 0.0f;
    params->pos_x_d_lpf = 0.0f;

    /* ===== Y 轴位置环参数 ===== */
    params->pos_y_kp = 0.90f;
    params->pos_y_ki = 0.0f;
    params->pos_y_kd = 0.0f;
    params->pos_y_kff = 0.0f;
    params->pos_y_i_limit = 0.0f;
    params->pos_y_d_lpf = 0.0f;

    /* ===== Z 轴位置环参数 ===== */
    params->pos_z_kp = 1.3f;
    params->pos_z_ki = 0.0f;
    params->pos_z_kd = 0.0f;
    params->pos_z_kff = 0.0f;
    params->pos_z_i_limit = 0.0f;
    params->pos_z_d_lpf = 0.0f;

    /* ===== X 轴速度环参数 ===== */
    params->vel_x_kp = 0.14f;
    params->vel_x_ki = 0.02f;
    params->vel_x_kd = 0.0f;
    params->vel_x_kff = 0.0f;
    params->vel_x_i_limit = 3.0f;
    params->vel_x_d_lpf = 0.0f;

    /* ===== Y 轴速度环参数 ===== */
    params->vel_y_kp = 0.14f;
    params->vel_y_ki = 0.02f;
    params->vel_y_kd = 0.0f;
    params->vel_y_kff = 0.0f;
    params->vel_y_i_limit = 3.0f;
    params->vel_y_d_lpf = 0.0f;

    /* ===== Z 轴速度环参数 ===== */
    params->vel_z_kp = 30.0f;
    params->vel_z_ki = 80.0f;
    params->vel_z_kd =0.0f;
    params->vel_z_kff = 0.0f;
    params->vel_z_i_limit = 450.0f;
    params->vel_z_d_lpf = 0.0f;

    /* ===== 模式 1 跟杆前馈与刹车参数 ===== */
    params->mode1_track_ff_deg_per_cmps = 0.06f;
    params->mode1_brake_kp = 0.18f;
    params->mode1_brake_exit_vel_cmps = 10.0f;

    /* ===== 模式 7 速度环参数 ===== */
    params->mode7_vel_x_kp = 0.16f;
    params->mode7_vel_x_ki = 0.0f;
    params->mode7_vel_x_kd = 0.0f;
    params->mode7_vel_x_kff = 0.010f;
    params->mode7_vel_x_i_limit = 3.0f;
    params->mode7_vel_x_d_lpf = 0.0f;
    params->mode7_vel_y_kp = 0.16f;
    params->mode7_vel_y_ki = 0.0f;
    params->mode7_vel_y_kd = 0.0f;
    params->mode7_vel_y_kff = 0.010f;
    params->mode7_vel_y_i_limit = 3.0f;
    params->mode7_vel_y_d_lpf = 0.0f;

    /* ===== 位置估计参数 ===== */
    params->pos_est_k_flow = 0.04f;

    /* ===== 模式 8 图像位置环参数 ===== */
    params->mode8_img_x_kp = 1.8f;
    params->mode8_img_x_ki = 0.0f;
    params->mode8_img_x_kd = 0.0f;
    params->mode8_img_x_kff = 0.0f;
    params->mode8_img_x_i_limit = 0.0f;
    params->mode8_img_x_d_lpf = 0.0f;
    params->mode8_img_y_kp = 1.8f;
    params->mode8_img_y_ki = 0.0f;
    params->mode8_img_y_kd = 0.0f;
    params->mode8_img_y_kff = 0.0f;
    params->mode8_img_y_i_limit = 0.0f;
    params->mode8_img_y_d_lpf = 0.0f;

    params->mode8_vel_x_kp = 0.18f;
    params->mode8_vel_x_ki = 0.0f;
    params->mode8_vel_x_kd = 0.0f;
    params->mode8_vel_x_kff = 0.015f;
    params->mode8_vel_x_i_limit = 3.0f;
    params->mode8_vel_x_d_lpf = 0.0f;
    params->mode8_vel_y_kp = 0.18f;
    params->mode8_vel_y_ki = 0.0f;
    params->mode8_vel_y_kd = 0.0f;
    params->mode8_vel_y_kff = 0.015f;
    params->mode8_vel_y_i_limit = 3.0f;
    params->mode8_vel_y_d_lpf = 0.0f;
    params->mode8_kp_car_x = 30.0f;
    params->mode8_kp_car_y = 30.0f;

    /* ===== Mode 5 image and velocity params ===== */
    params->mode5_img_x_kp = 1.8f;
    params->mode5_img_x_ki = 0.0f;
    params->mode5_img_x_kd = 0.0f;
    params->mode5_img_x_kff = 0.0f;
    params->mode5_img_x_i_limit = 0.0f;
    params->mode5_img_x_d_lpf = 0.0f;
    params->mode5_img_y_kp = 1.8f;
    params->mode5_img_y_ki = 0.0f;
    params->mode5_img_y_kd = 0.0f;
    params->mode5_img_y_kff = 0.0f;
    params->mode5_img_y_i_limit = 0.0f;
    params->mode5_img_y_d_lpf = 0.0f;
    params->mode5_vel_x_kp = 0.18f;
    params->mode5_vel_x_ki = 0.0f;
    params->mode5_vel_x_kd = 0.0f;
    params->mode5_vel_x_kff = 0.015f;
    params->mode5_vel_x_i_limit = 3.0f;
    params->mode5_vel_x_d_lpf = 0.0f;
    params->mode5_vel_y_kp = 0.18f;
    params->mode5_vel_y_ki = 0.0f;
    params->mode5_vel_y_kd = 0.0f;
    params->mode5_vel_y_kff = 0.015f;
    params->mode5_vel_y_i_limit = 3.0f;
    params->mode5_vel_y_d_lpf = 0.0f;
    params->mode5_kp_car_x = 30.0f;
    params->mode5_kp_car_y = 30.0f;

}

/*
 * 函数名: FC_Params_Init
 * 功能: 初始化飞控参数默认值
 * 输入参数: 无
 * 返回值: 无
 */
void FC_Params_Init(void)
{
    fc_params_fill_defaults(&g_fc_params);
}

uint8_t FC_Params_LoadFromFlash(void)
{
    fc_params_flash_blob_t blob;
    const uint32_t words =
        (uint32_t)((sizeof(blob) + sizeof(uint32_t) - 1U) / sizeof(uint32_t));

    flash_init();

    memset(&blob, 0, sizeof(blob));
    flash_read_page(0U, FC_PARAMS_FLASH_PAGE, (uint32 *)&blob, words);
    if (0U == fc_params_blob_is_valid(&blob))
    {
        return 0U;
    }

    fc_params_fill_defaults(&g_fc_params);
    memcpy(&g_fc_params, &blob.params, (size_t)blob.size);
    fc_params_migrate_loaded(&g_fc_params, blob.version);
    return 1U;
}

uint8_t FC_Params_SaveToFlash(void)
{
    fc_params_flash_blob_t blob;
    const uint32_t words =
        (uint32_t)((sizeof(blob) + sizeof(uint32_t) - 1U) / sizeof(uint32_t));

    flash_init();

    memset(&blob, 0, sizeof(blob));
    blob.magic = FC_PARAMS_FLASH_MAGIC;
    blob.version = FC_PARAMS_FLASH_VERSION;
    blob.size = (uint16_t)sizeof(fc_params_t);
    memcpy(&blob.params, &g_fc_params, sizeof(blob.params));
    blob.checksum = fc_params_checksum_calc(&blob.params, (uint32_t)sizeof(blob.params));

    flash_write_page(0U, FC_PARAMS_FLASH_PAGE, (const uint32 *)&blob, words);

    memset(&blob, 0, sizeof(blob));
    flash_read_page(0U, FC_PARAMS_FLASH_PAGE, (uint32 *)&blob, words);
    if ((0U == fc_params_blob_is_valid(&blob)) ||
        (blob.version != FC_PARAMS_FLASH_VERSION) ||
        (blob.size != (uint16_t)sizeof(fc_params_t)))
    {
        return 0U;
    }

    return (0 == memcmp(&blob.params, &g_fc_params, sizeof(g_fc_params))) ? 1U : 0U;
}

uint8_t FC_Params_ClearFlash(void)
{
    flash_init();
    flash_erase_page(0U, FC_PARAMS_FLASH_PAGE);
    return (0U == flash_check(0U, FC_PARAMS_FLASH_PAGE)) ? 1U : 0U;
}
