/*****************************************************************************
 * 文件: fc_params.c
 * 模块: 飞控 - 参数管理实现
 * 职责: 管理默认参数、运行时参数以及 Flash 持久化
 *****************************************************************************/

#include "fc_params.h"

#include <string.h>
#include "zf_common_headfile.h"

/* 飞控参数 Flash 魔数：用于识别有效参数块 */
#define FC_PARAMS_FLASH_MAGIC   (0x46504346UL)
/* 飞控参数 Flash 版本号：结构变化时递增 */
#define FC_PARAMS_FLASH_VERSION (15U)
/* 飞控参数 Flash 可兼容读取的最小版本号 */
#define FC_PARAMS_FLASH_MIN_COMPAT_VERSION (5U)

/* 飞控参数 Flash 数据块：保存头信息和完整参数结构体 */
typedef struct
{
    uint32 magic;      /* 魔数 */
    uint16 version;    /* 参数版本 */
    uint16 size;       /* 参数结构体大小 */
    uint32 checksum;   /* 参数校验值 */
    fc_params_t params;  /* 参数内容 */
} fc_params_flash_blob_t;

/* 飞控参数全局实例：集中保存控制周期、油门基准和各控制环 PID 参数 */
fc_params_t g_fc_params;

/*
 * 函数名: fc_params_checksum_calc
 * 功能: 计算参数块的 FNV-1a 32 位校验值
 * 输入参数:
 *   data - 待校验数据首地址
 *   size - 数据长度，单位字节
 * 返回值:
 *   32 位校验值
 */
static uint32 fc_params_checksum_calc(const void *data, uint32 size)
{
    const uint8 *bytes = (const uint8 *)data;
    uint32 hash = 2166136261UL;
    uint32 i;

    for (i = 0U; i < size; i++)
    {
        hash ^= (uint32)bytes[i];
        hash *= 16777619UL;
    }

    return hash;
}

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
    params->base_throttle = 4200;         /* 悬停油门 */
    params->roll_mech_trim_deg = 0.22f;    /* Roll 机械配平角 */
    params->pitch_mech_trim_deg = -1.80f;  /* Pitch 机械配平角 */

    /* ===== Roll 轴角速度环参数 ===== */
    params->roll_gyro_kp = 5.0f;
    params->roll_gyro_ki = 0.0f;
    params->roll_gyro_kd = 0.0f;
    params->roll_gyro_kff = 0.0f;
    params->roll_gyro_i_limit = 180.0f;
    params->roll_gyro_d_lpf = 30.0f;

    /* ===== Pitch 轴角速度环参数 ===== */
    params->pitch_gyro_kp = 6.0f;
    params->pitch_gyro_ki = 0.10f;
    params->pitch_gyro_kd = 0.0f;
    params->pitch_gyro_kff = 0.0f;
    params->pitch_gyro_i_limit = 100.0f;
    params->pitch_gyro_d_lpf = 30.0f;

    /* ===== Yaw 轴角速度环参数 ===== */
    params->yaw_gyro_kp = 14.0f;
    params->yaw_gyro_ki = 4.0f;
    params->yaw_gyro_kd = 0.0f;
    params->yaw_gyro_kff = 0.0f;
    params->yaw_gyro_i_limit = 700.0f;
    params->yaw_gyro_d_lpf = 30.0f;

    /* ===== Roll 轴角度环参数 ===== */
    params->roll_angle_kp = 6.5f;
    params->roll_angle_ki = 0.0f;
    params->roll_angle_kd = 0.0f;
    params->roll_angle_kff = 0.12f;
    params->roll_angle_i_limit = 80.0f;
    params->roll_angle_d_lpf = 15.0f;

    /* ===== Pitch 轴角度环参数 ===== */
    params->pitch_angle_kp = 6.4f;
    params->pitch_angle_ki = 0.0f;
    params->pitch_angle_kd = 0.0f;
    params->pitch_angle_kff = 0.12f;
    params->pitch_angle_i_limit = 80.0f;
    params->pitch_angle_d_lpf = 15.0f;

    /* ===== Yaw 轴角度环参数 ===== */
    params->yaw_angle_kp = 1.5f;
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
    params->pos_z_kp = 1.40f;
    params->pos_z_ki = 0.0f;
    params->pos_z_kd = 0.0f;
    params->pos_z_kff = 0.0f;
    params->pos_z_i_limit = 0.0f;
    params->pos_z_d_lpf = 0.0f;

    /* ===== X 轴速度环参数 ===== */
    params->vel_x_kp = 0.14f;
    params->vel_x_ki = 0.010f;
    params->vel_x_kd = 0.0f;
    params->vel_x_kff = 0.0f;
    params->vel_x_i_limit = 3.0f;
    params->vel_x_d_lpf = 0.0f;

    /* ===== Y 轴速度环参数 ===== */
    params->vel_y_kp = 0.14f;
    params->vel_y_ki = 0.010f;
    params->vel_y_kd = 0.0f;
    params->vel_y_kff = 0.0f;
    params->vel_y_i_limit = 3.0f;
    params->vel_y_d_lpf = 0.0f;

    /* ===== Z 轴速度环参数 ===== */
    params->vel_z_kp = 700.0f;
    params->vel_z_ki = 55.0f;
    params->vel_z_kd = 0.0f;
    params->vel_z_kff = 0.0f;
    params->vel_z_i_limit = 380.0f;
    params->vel_z_d_lpf = 0.0f;

    /* ===== 模式 1 跟杆前馈与刹车参数 ===== */
    params->mode1_track_ff_deg_per_cmps = 0.06f;
    params->mode1_brake_kp = 0.18f;
    params->mode1_brake_exit_vel_cmps = 10.0f;

    /* ===== 模式 2 速度环前馈参数 ===== */
    params->mode2_vel_x_ff_deg_per_cmps = 0.010f;
    params->mode2_vel_y_ff_deg_per_cmps = 0.010f;
    params->mode2_adrc_enable = 1.0f;
    params->mode2_adrc_log_enable = 1.0f;
    params->mode2_adrc_b0_x = 16.0f;
    params->mode2_adrc_b0_y = 14.5f;
    params->mode2_adrc_td_acc_limit_cmss = 100.0f;
    params->mode2_adrc_td_jerk_limit_cmsss = 450.0f;
    params->mode2_adrc_td_brake_acc_limit_cmss = 160.0f;
    params->mode2_adrc_td_brake_jerk_limit_cmsss = 900.0f;
    params->mode2_adrc_eso_beta1 = 12.0f;
    params->mode2_adrc_eso_beta2 = 36.0f;
    params->mode2_adrc_eso_alpha1 = 0.50f;
    params->mode2_adrc_eso_alpha2 = 0.25f;
    params->mode2_adrc_eso_delta_cmps = 15.0f;
    params->mode2_adrc_nl_kp = 4.2f;
    params->mode2_adrc_nl_alpha = 0.75f;
    params->mode2_adrc_nl_delta_cmps = 12.0f;
    params->mode2_adrc_comp_limit_cmss = 45.0f;
    params->mode2_adrc_angle_limit_deg = 10.0f;
    params->mode2_adrc_output_rate_limit_degps = 35.0f;
    params->mode2_adrc_zero_quiet_cmps = 8.0f;

    /* ===== 位置估计参数 ===== */
    params->pos_est_k_flow = 0.04f;

    /* ===== 模式 8 图像位置环参数 ===== */
    params->mode8_img_x_kp = 1.1f;
    params->mode8_img_x_ki = 0.0f;
    params->mode8_img_x_kd = 0.10f;
    params->mode8_img_x_kff = 0.0f;
    params->mode8_img_x_i_limit = 0.0f;
    params->mode8_img_x_d_lpf = 0.0f;
    params->mode8_img_y_kp = 0.9f;
    params->mode8_img_y_ki = 0.0f;
    params->mode8_img_y_kd = 0.10f;
    params->mode8_img_y_kff = 0.0f;
    params->mode8_img_y_i_limit = 0.0f;
    params->mode8_img_y_d_lpf = 0.0f;
}

/*
 * 函数名: fc_params_blob_is_valid
 * 功能: 检查 Flash 参数块是否合法
 * 输入参数:
 *   blob - 待读取的 Flash 参数块指针
 * 返回值:
 *   1=合法，0=非法
 */
static uint8 fc_params_blob_is_valid(const fc_params_flash_blob_t *blob)
{
    uint32 checksum;
    uint32 payload_size;

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

    if ((blob->size == 0U) || (blob->size > (uint16)sizeof(fc_params_t)))
    {
        return 0U;
    }

    payload_size = (uint32)blob->size;
    checksum = fc_params_checksum_calc(&blob->params, payload_size);
    return (checksum == blob->checksum) ? 1U : 0U;
}

/*
 * 函数名: fc_params_migrate_loaded
 * 功能: 对旧版 Flash 参数做最小兼容迁移，避免新增默认值被旧参数清零
 * 输入参数:
 *   params  - 已加载到 RAM 的参数结构体指针
 *   version - Flash 参数版本号
 * 返回值: 无
 */
static void fc_params_migrate_loaded(fc_params_t *params, uint16 version)
{
    if (NULL == params)
    {
        return;
    }

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

    if (version < 9U)
    {
        params->mode8_img_x_kp = 1.1f;
        params->mode8_img_x_ki = 0.0f;
        params->mode8_img_x_kd = 0.10f;
        params->mode8_img_x_kff = 0.0f;
        params->mode8_img_x_i_limit = 0.0f;
        params->mode8_img_x_d_lpf = 0.0f;
        params->mode8_img_y_kp = 0.9f;
        params->mode8_img_y_ki = 0.0f;
        params->mode8_img_y_kd = 0.10f;
        params->mode8_img_y_kff = 0.0f;
        params->mode8_img_y_i_limit = 0.0f;
        params->mode8_img_y_d_lpf = 0.0f;
    }

    if (version < 10U)
    {
        params->roll_mech_trim_deg = 0.22f;
        params->pitch_mech_trim_deg = -1.80f;
        params->roll_angle_kp = 6.5f;
        params->roll_angle_kff = 0.12f;
        params->pitch_angle_kp = 6.4f;
        params->pitch_angle_kff = 0.12f;
        params->vel_x_kp = 0.16f;
        params->vel_y_kp = 0.16f;
        params->mode2_vel_x_ff_deg_per_cmps = 0.015f;
        params->mode2_vel_y_ff_deg_per_cmps = 0.015f;
    }

    if (version < 11U)
    {
        params->vel_x_kp = 0.14f;
        params->vel_x_ki = 0.015f;
        params->vel_y_kp = 0.14f;
        params->vel_y_ki = 0.015f;
        params->mode2_vel_x_ff_deg_per_cmps = 0.010f;
        params->mode2_vel_y_ff_deg_per_cmps = 0.010f;
    }

    if (version < 12U)
    {
        params->vel_x_ki = 0.010f;
        params->vel_y_ki = 0.010f;
    }

    if (version < 13U)
    {
        params->mode2_adrc_enable = 1.0f;
        params->mode2_adrc_log_enable = 1.0f;
        params->mode2_adrc_b0_x = 16.0f;
        params->mode2_adrc_b0_y = 14.5f;
        params->mode2_adrc_td_acc_limit_cmss = 100.0f;
        params->mode2_adrc_td_jerk_limit_cmsss = 450.0f;
        params->mode2_adrc_td_brake_acc_limit_cmss = 160.0f;
        params->mode2_adrc_td_brake_jerk_limit_cmsss = 900.0f;
        params->mode2_adrc_eso_beta1 = 12.0f;
        params->mode2_adrc_eso_beta2 = 36.0f;
        params->mode2_adrc_eso_alpha1 = 0.50f;
        params->mode2_adrc_eso_alpha2 = 0.25f;
        params->mode2_adrc_eso_delta_cmps = 15.0f;
        params->mode2_adrc_nl_kp = 4.2f;
        params->mode2_adrc_nl_alpha = 0.75f;
        params->mode2_adrc_nl_delta_cmps = 12.0f;
        params->mode2_adrc_comp_limit_cmss = 45.0f;
        params->mode2_adrc_angle_limit_deg = 10.0f;
    }

    if (version < 14U)
    {
        params->mode2_adrc_output_rate_limit_degps = 35.0f;
        params->mode2_adrc_td_acc_limit_cmss = 100.0f;
        params->mode2_adrc_td_jerk_limit_cmsss = 450.0f;
        params->mode2_adrc_nl_kp = 4.2f;
        params->mode2_adrc_comp_limit_cmss = 45.0f;
    }

    if (version < 15U)
    {
        params->mode2_adrc_b0_x = 16.0f;
        params->mode2_adrc_b0_y = 14.5f;
        params->mode2_adrc_td_brake_acc_limit_cmss = 160.0f;
        params->mode2_adrc_td_brake_jerk_limit_cmsss = 900.0f;
        params->mode2_adrc_zero_quiet_cmps = 8.0f;
    }
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

/*
 * 函数名: FC_Params_LoadFromFlash
 * 功能: 从 Flash 加载飞控参数到运行时实例
 * 输入参数: 无
 * 返回值: 1=加载成功，0=加载失败
 */
uint8 FC_Params_LoadFromFlash(void)
{
    fc_params_flash_blob_t blob;
    uint32 words;

    flash_init();

    words = (uint32)((sizeof(blob) + sizeof(uint32) - 1U) / sizeof(uint32));
    flash_read_page(0U, FC_PARAMS_FLASH_PAGE, (uint32 *)&blob, words);
    if (0U == fc_params_blob_is_valid(&blob))
    {
        return 0U;
    }

    fc_params_fill_defaults(&g_fc_params);
    memcpy(&g_fc_params, &blob.params, (uint32)blob.size);
    fc_params_migrate_loaded(&g_fc_params, blob.version);
    return 1U;
}

/*
 * 函数名: FC_Params_SaveToFlash
 * 功能: 将当前运行时参数保存到 Flash
 * 输入参数: 无
 * 返回值: 1=保存成功，0=保存失败
 */
uint8 FC_Params_SaveToFlash(void)
{
    fc_params_flash_blob_t blob;
    uint32 words;

    flash_init();

    memset(&blob, 0, sizeof(blob));
    blob.magic = FC_PARAMS_FLASH_MAGIC;
    blob.version = FC_PARAMS_FLASH_VERSION;
    blob.size = (uint16)sizeof(fc_params_t);
    memcpy(&blob.params, &g_fc_params, sizeof(blob.params));
    blob.checksum = fc_params_checksum_calc(&blob.params, (uint32)sizeof(blob.params));

    words = (uint32)((sizeof(blob) + sizeof(uint32) - 1U) / sizeof(uint32));
    flash_write_page(0U, FC_PARAMS_FLASH_PAGE, (const uint32 *)&blob, words);

    return FC_Params_LoadFromFlash();
}

/*
 * 函数名: FC_Params_ClearFlash
 * 功能: 擦除飞控参数 Flash 页
 * 输入参数: 无
 * 返回值: 1=擦除成功，0=擦除失败
 */
uint8 FC_Params_ClearFlash(void)
{
    flash_init();
    flash_erase_page(0U, FC_PARAMS_FLASH_PAGE);
    return (0U == flash_check(0U, FC_PARAMS_FLASH_PAGE)) ? 1U : 0U;
}
