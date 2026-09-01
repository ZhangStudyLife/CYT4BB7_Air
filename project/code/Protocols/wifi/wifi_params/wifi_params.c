/*****************************************************************************
 * 文件: wifi_params.c
 * 模块: WiFi 参数调节
 * 职责: 处理飞控参数查询、修改、保存、加载与待机态遥测开关命令
 *****************************************************************************/

#include "wifi_params.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FlightController/fc_loop.h"
#include "FlightController/fc_params.h"
#include "FlightController/fc_start_crsf.h"
#include "Protocols/wifi/wifi_cmd/wifi_cmd.h"
#include "Protocols/wifi/wifi_justfloat/wifi_justfloat.h"

/* WiFi 参数值类型：区分 float 参数与 int32 参数 */
typedef enum
{
    WIFI_PARAMS_VALUE_FLOAT = 0,  /* float 参数 */
    WIFI_PARAMS_VALUE_INT32 = 1   /* int32 参数 */
} wifi_params_value_type_e;

/* WiFi 参数白名单表项：描述参数名、目标地址、类型和安全范围 */
typedef struct
{
    const char *name;                    /* 参数名 */
    void *value_ptr;                     /* 参数地址 */
    wifi_params_value_type_e value_type; /* 参数类型 */
    float min_value;                     /* 最小允许值 */
    float max_value;                     /* 最大允许值 */
} wifi_params_entry_t;

#define WIFI_PARAMS_FLOAT_ITEM(param_name, member, min_v, max_v) \
    { (param_name), (void *)&g_fc_params.member, WIFI_PARAMS_VALUE_FLOAT, (min_v), (max_v) }

#define WIFI_PARAMS_INT_ITEM(param_name, member, min_v, max_v) \
    { (param_name), (void *)&g_fc_params.member, WIFI_PARAMS_VALUE_INT32, (float)(min_v), (float)(max_v) }

/* WiFi 参数白名单表：只允许修改显式开放的飞控参数 */
static const wifi_params_entry_t s_wifi_params_table[] =
{
    WIFI_PARAMS_INT_ITEM("base_throttle", base_throttle, 0, 6000),
    WIFI_PARAMS_FLOAT_ITEM("roll_mech_trim_deg", roll_mech_trim_deg, -30.0f, 30.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_mech_trim_deg", pitch_mech_trim_deg, -30.0f, 30.0f),

    WIFI_PARAMS_FLOAT_ITEM("roll_gyro_kp", roll_gyro_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_gyro_ki", roll_gyro_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_gyro_kd", roll_gyro_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_gyro_kff", roll_gyro_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_gyro_i_limit", roll_gyro_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_gyro_d_lpf", roll_gyro_d_lpf, 0.0f, 500.0f),

    WIFI_PARAMS_FLOAT_ITEM("pitch_gyro_kp", pitch_gyro_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_gyro_ki", pitch_gyro_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_gyro_kd", pitch_gyro_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_gyro_kff", pitch_gyro_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_gyro_i_limit", pitch_gyro_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_gyro_d_lpf", pitch_gyro_d_lpf, 0.0f, 500.0f),

    WIFI_PARAMS_FLOAT_ITEM("yaw_gyro_kp", yaw_gyro_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_gyro_ki", yaw_gyro_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_gyro_kd", yaw_gyro_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_gyro_kff", yaw_gyro_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_gyro_i_limit", yaw_gyro_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_gyro_d_lpf", yaw_gyro_d_lpf, 0.0f, 500.0f),

    WIFI_PARAMS_FLOAT_ITEM("roll_angle_kp", roll_angle_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_angle_ki", roll_angle_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_angle_kd", roll_angle_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_angle_kff", roll_angle_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_angle_i_limit", roll_angle_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_angle_d_lpf", roll_angle_d_lpf, 0.0f, 500.0f),

    WIFI_PARAMS_FLOAT_ITEM("pitch_angle_kp", pitch_angle_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_angle_ki", pitch_angle_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_angle_kd", pitch_angle_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_angle_kff", pitch_angle_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_angle_i_limit", pitch_angle_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_angle_d_lpf", pitch_angle_d_lpf, 0.0f, 500.0f),

    WIFI_PARAMS_FLOAT_ITEM("yaw_angle_kp", yaw_angle_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_angle_ki", yaw_angle_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_angle_kd", yaw_angle_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_angle_kff", yaw_angle_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_angle_i_limit", yaw_angle_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_angle_d_lpf", yaw_angle_d_lpf, 0.0f, 500.0f),

    WIFI_PARAMS_FLOAT_ITEM("pos_z_kp", pos_z_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_z_ki", pos_z_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_z_kd", pos_z_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_z_kff", pos_z_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_z_i_limit", pos_z_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_z_d_lpf", pos_z_d_lpf, 0.0f, 500.0f),

    WIFI_PARAMS_FLOAT_ITEM("vel_z_kp", vel_z_kp, 0.0f, 1000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_z_ki", vel_z_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_z_kd", vel_z_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_z_kff", vel_z_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_z_i_limit", vel_z_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_z_d_lpf", vel_z_d_lpf, 0.0f, 500.0f),

    WIFI_PARAMS_FLOAT_ITEM("mode7_vel_x_kp", mode7_vel_x_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode7_vel_x_ki", mode7_vel_x_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode7_vel_x_kd", mode7_vel_x_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode7_vel_x_kff", mode7_vel_x_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode7_vel_x_i_limit", mode7_vel_x_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode7_vel_x_d_lpf", mode7_vel_x_d_lpf, 0.0f, 500.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode7_vel_y_kp", mode7_vel_y_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode7_vel_y_ki", mode7_vel_y_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode7_vel_y_kd", mode7_vel_y_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode7_vel_y_kff", mode7_vel_y_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode7_vel_y_i_limit", mode7_vel_y_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode7_vel_y_d_lpf", mode7_vel_y_d_lpf, 0.0f, 500.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_est_k_flow", pos_est_k_flow, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("mode8_img_x_kp", mode8_img_x_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_img_x_ki", mode8_img_x_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_img_x_kd", mode8_img_x_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_img_x_kff", mode8_img_x_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_img_x_i_limit", mode8_img_x_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_img_x_d_lpf", mode8_img_x_d_lpf, 0.0f, 500.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_img_y_kp", mode8_img_y_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_img_y_ki", mode8_img_y_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_img_y_kd", mode8_img_y_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_img_y_kff", mode8_img_y_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_img_y_i_limit", mode8_img_y_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_img_y_d_lpf", mode8_img_y_d_lpf, 0.0f, 500.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_vel_x_kp", mode8_vel_x_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_vel_x_ki", mode8_vel_x_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_vel_x_kd", mode8_vel_x_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_vel_x_kff", mode8_vel_x_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_vel_x_i_limit", mode8_vel_x_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_vel_x_d_lpf", mode8_vel_x_d_lpf, 0.0f, 500.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_vel_y_kp", mode8_vel_y_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_vel_y_ki", mode8_vel_y_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_vel_y_kd", mode8_vel_y_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_vel_y_kff", mode8_vel_y_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_vel_y_i_limit", mode8_vel_y_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode8_vel_y_d_lpf", mode8_vel_y_d_lpf, 0.0f, 500.0f)
};

static wifi_params_diag_t s_wifi_params_diag = {0}; /* 最近一次参数命令处理诊断 */

/*
 * 函数名: wifi_params_set_diag
 * 功能: 更新最近一次命令处理诊断
 * 输入参数:
 *   command_code - 命令码
 *   result_code  - 结果码
 *   param_index  - 参数索引
 *   value        - 相关参数值
 * 返回值: 无
 */
static void wifi_params_set_diag(uint8_t command_code, uint8_t result_code, uint16_t param_index, float value)
{
    s_wifi_params_diag.last_command_code = command_code;
    s_wifi_params_diag.last_result_code = result_code;
    s_wifi_params_diag.last_param_index = param_index;
    s_wifi_params_diag.last_value = value;
}

/*
 * 函数名: wifi_params_is_edit_allowed
 * 功能: 判断当前是否允许执行参数读写和待机态遥测开关操作
 * 输入参数: 无
 * 返回值:
 *   1 - 允许
 *   0 - 不允许
 */
static uint8_t wifi_params_is_edit_allowed(void)
{
    if (FC_START_CRSF_STATE_STANDBY != FC_START_CRSF_Get_State())
    {
        return 0U;
    }

    return (0U == FC_START_CRSF_Is_Armed()) ? 1U : 0U;
}

/*
 * 函数名: wifi_params_parse_float
 * 功能: 解析浮点字符串并过滤 NaN/Inf
 * 输入参数:
 *   text  - 待解析字符串
 *   value - 输出浮点值指针
 * 返回值:
 *   1 - 解析成功
 *   0 - 解析失败
 */
static uint8_t wifi_params_parse_float(const char *text, float *value)
{
    char *endptr = NULL;
    double parsed;

    if ((NULL == text) || (NULL == value) || ('\0' == text[0]))
    {
        return 0U;
    }

    parsed = strtod(text, &endptr);
    if ((NULL == endptr) || ('\0' != *endptr))
    {
        return 0U;
    }

    if ((parsed != parsed) || (parsed > 1000000000.0) || (parsed < -1000000000.0))
    {
        return 0U;
    }

    *value = (float)parsed;
    return 1U;
}

/*
 * 函数名: wifi_params_parse_int32
 * 功能: 解析整数字符串并过滤非法尾部
 * 输入参数:
 *   text  - 待解析字符串
 *   value - 输出整数值指针
 * 返回值:
 *   1 - 解析成功
 *   0 - 解析失败
 */
static uint8_t wifi_params_parse_int32(const char *text, int32_t *value)
{
    char *endptr = NULL;
    long parsed;

    if ((NULL == text) || (NULL == value) || ('\0' == text[0]))
    {
        return 0U;
    }

    parsed = strtol(text, &endptr, 10);
    if ((NULL == endptr) || ('\0' != *endptr))
    {
        return 0U;
    }

    *value = (int32_t)parsed;
    return 1U;
}

/*
 * 函数名: wifi_params_find_entry
 * 功能: 根据参数名查找白名单表项
 * 输入参数:
 *   name - 参数名
 * 返回值:
 *   匹配到的表项指针，未找到时返回 NULL
 */
static const wifi_params_entry_t *wifi_params_find_entry(const char *name)
{
    uint16_t i;

    if (NULL == name)
    {
        return NULL;
    }

    for (i = 0U; i < (uint16_t)(sizeof(s_wifi_params_table) / sizeof(s_wifi_params_table[0])); i++)
    {
        if (0 == wifi_cmd_ascii_stricmp(name, s_wifi_params_table[i].name))
        {
            return &s_wifi_params_table[i];
        }
    }

    return NULL;
}

/*
 * 函数名: wifi_params_entry_index
 * 功能: 计算白名单表项的 1 基参数索引
 * 输入参数:
 *   entry - 白名单表项指针
 * 返回值:
 *   参数索引，失败时返回 0
 */
static uint16_t wifi_params_entry_index(const wifi_params_entry_t *entry)
{
    uint16_t i;

    if (NULL == entry)
    {
        return 0U;
    }

    for (i = 0U; i < (uint16_t)(sizeof(s_wifi_params_table) / sizeof(s_wifi_params_table[0])); i++)
    {
        if (entry == &s_wifi_params_table[i])
        {
            return (uint16_t)(i + 1U);
        }
    }

    return 0U;
}

/*
 * 函数名: wifi_params_entry_count
 * 功能: 获取白名单参数总数
 * 输入参数: 无
 * 返回值:
 *   白名单参数总数
 */
static uint16_t wifi_params_entry_count(void)
{
    return (uint16_t)(sizeof(s_wifi_params_table) / sizeof(s_wifi_params_table[0]));
}

/*
 * 函数名: wifi_params_entry_read_as_float
 * 功能: 将表项当前值统一读取为 float
 * 输入参数:
 *   entry - 白名单表项指针
 * 返回值:
 *   当前参数值，失败时返回 0
 */
static float wifi_params_entry_read_as_float(const wifi_params_entry_t *entry)
{
    if (NULL == entry)
    {
        return 0.0f;
    }

    if (WIFI_PARAMS_VALUE_INT32 == entry->value_type)
    {
        return (float)(*(const int32_t *)entry->value_ptr);
    }

    return *(const float *)entry->value_ptr;
}

/*
 * 函数名: wifi_params_entry_write_value
 * 功能: 对表项执行范围检查并写入运行时参数
 * 输入参数:
 *   entry - 白名单表项指针
 *   value - 待写入值
 * 返回值:
 *   1 - 写入成功
 *   0 - 写入失败
 */
static uint8_t wifi_params_entry_write_value(const wifi_params_entry_t *entry, float value)
{
    if (NULL == entry)
    {
        return 0U;
    }

    if ((value < entry->min_value) || (value > entry->max_value))
    {
        return 0U;
    }

    if (WIFI_PARAMS_VALUE_INT32 == entry->value_type)
    {
        *(int32_t *)entry->value_ptr = (int32_t)value;
    }
    else
    {
        *(float *)entry->value_ptr = value;
    }

    return 1U;
}

/*
 * 函数名: wifi_params_send_value_line
 * 功能: 按统一格式回传一条参数值文本
 * 输入参数:
 *   entry - 需要回传的参数表项
 * 返回值:
 *   1 - 发送成功
 *   0 - 发送失败
 */
static uint8_t wifi_params_send_value_line(const wifi_params_entry_t *entry)
{
    if (NULL == entry)
    {
        return 0U;
    }

    if (WIFI_PARAMS_VALUE_INT32 == entry->value_type)
    {
        return wifi_cmd_SendLine("%s=%ld", entry->name, (long)(*(const int32_t *)entry->value_ptr));
    }

    return wifi_cmd_SendLine("%s=%.6g", entry->name, (double)(*(const float *)entry->value_ptr));
}

/*
 * 函数名: wifi_params_reply_error
 * 功能: 发送参数命令错误回包并更新诊断信息
 * 输入参数:
 *   command_code - 命令码
 *   result_code  - 结果码
 *   param_index  - 参数索引
 *   value        - 相关值
 *   reason       - 错误原因文本
 * 返回值: 无
 */
static void wifi_params_reply_error(uint8_t command_code, uint8_t result_code, uint16_t param_index, float value, const char *reason)
{
    wifi_params_set_diag(command_code, result_code, param_index, value);
    (void)wifi_cmd_SendLine("ERR %s", (NULL != reason) ? reason : "format");
}

/*
 * 函数名: wifi_params_process_ping
 * 功能: 处理 ping 命令
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_params_process_ping(void)
{
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_PING, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);
    (void)wifi_cmd_SendLine("OK ping");
}

/*
 * 函数名: wifi_params_process_help_topic
 * 功能: 输出指定主题帮助
 * 输入参数:
 *   topic - 帮助主题
 * 返回值:
 *   1 - 已处理
 *   0 - 未知主题
 */
static uint8_t wifi_params_process_help_topic(const char *topic)
{
    if (NULL == topic)
    {
        return 0U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "ping"))
    {
        (void)wifi_cmd_SendLine("主题: ping");
        (void)wifi_cmd_SendLine("用法: ping");
        (void)wifi_cmd_SendLine("功能: 检查当前 UDP 链路是否畅通");
        (void)wifi_cmd_SendLine("限制: 任意状态都允许执行");
        (void)wifi_cmd_SendLine("示例: ping");
        (void)wifi_cmd_SendLine("成功回包: OK ping");
        (void)wifi_cmd_SendLine("OK help ping");
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "help"))
    {
        (void)wifi_cmd_SendLine("主题: help");
        (void)wifi_cmd_SendLine("用法1: help");
        (void)wifi_cmd_SendLine("用法2: help <命令>");
        (void)wifi_cmd_SendLine("用法3: <命令> --help");
        (void)wifi_cmd_SendLine("功能: 输出总帮助或命令的详细帮助");
        (void)wifi_cmd_SendLine("示例1: help imu");
        (void)wifi_cmd_SendLine("示例2: imu help");
        (void)wifi_cmd_SendLine("OK help help");
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "imu"))
    {
        (void)wifi_cmd_SendLine("主题: imu");
        (void)wifi_cmd_SendLine("用法1: imu help");
        (void)wifi_cmd_SendLine("用法2: imu status");
        (void)wifi_cmd_SendLine("用法3: imu start gyro");
        (void)wifi_cmd_SendLine("用法4: imu start accel");
        (void)wifi_cmd_SendLine("用法5: imu start accel_man");
        (void)wifi_cmd_SendLine("用法6: imu acc collect / imu acc stop");
        (void)wifi_cmd_SendLine("用法7: imu flash");
        (void)wifi_cmd_SendLine("功能: 访问 IMU 校准命令集");
        (void)wifi_cmd_SendLine("说明: 更详细说明请执行 imu help");
        (void)wifi_cmd_SendLine("OK help imu");
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "get"))
    {
        (void)wifi_cmd_SendLine("主题: get");
        (void)wifi_cmd_SendLine("用法: get <参数名>");
        (void)wifi_cmd_SendLine("功能: 读取白名单参数的当前值");
        (void)wifi_cmd_SendLine("限制: 仅待机且未解锁时允许执行");
        (void)wifi_cmd_SendLine("说明: 参数名不区分大小写");
        (void)wifi_cmd_SendLine("示例: get base_throttle");
        (void)wifi_cmd_SendLine("成功回包: OK base_throttle=3500");
        (void)wifi_cmd_SendLine("OK help get");
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "set"))
    {
        (void)wifi_cmd_SendLine("主题: set");
        (void)wifi_cmd_SendLine("用法: set <参数名> <数值>");
        (void)wifi_cmd_SendLine("功能: 修改 RAM 参数并立即重载控制器");
        (void)wifi_cmd_SendLine("限制: 仅待机且未解锁时允许执行");
        (void)wifi_cmd_SendLine("说明1: 参数名不区分大小写");
        (void)wifi_cmd_SendLine("说明2: 断电不会保存，需要再执行 save");
        (void)wifi_cmd_SendLine("示例: set roll_angle_kp 5.5");
        (void)wifi_cmd_SendLine("成功回包: OK set roll_angle_kp=5.5");
        (void)wifi_cmd_SendLine("OK help set");
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "save"))
    {
        (void)wifi_cmd_SendLine("主题: save");
        (void)wifi_cmd_SendLine("用法: save");
        (void)wifi_cmd_SendLine("功能: 将当前参数永久保存到 Flash");
        (void)wifi_cmd_SendLine("限制: 仅待机且未解锁时允许执行");
        (void)wifi_cmd_SendLine("说明: save 保存的是当前 RAM 参数");
        (void)wifi_cmd_SendLine("示例: save");
        (void)wifi_cmd_SendLine("成功回包: OK save");
        (void)wifi_cmd_SendLine("OK help save");
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "load"))
    {
        (void)wifi_cmd_SendLine("主题: load");
        (void)wifi_cmd_SendLine("用法: load");
        (void)wifi_cmd_SendLine("功能: 从 Flash 读取上次保存的参数");
        (void)wifi_cmd_SendLine("限制: 仅待机且未解锁时允许执行");
        (void)wifi_cmd_SendLine("说明: 成功后会重新装载控制器参数");
        (void)wifi_cmd_SendLine("示例: load");
        (void)wifi_cmd_SendLine("成功回包: OK load");
        (void)wifi_cmd_SendLine("OK help load");
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "start"))
    {
        (void)wifi_cmd_SendLine("主题: start");
        (void)wifi_cmd_SendLine("用法: start");
        (void)wifi_cmd_SendLine("功能: 恢复待机状态下的 JustFloat 遥测发送");
        (void)wifi_cmd_SendLine("限制: 仅待机且未解锁时允许执行");
        (void)wifi_cmd_SendLine("说明: 飞行中遥测本来就始终允许发送");
        (void)wifi_cmd_SendLine("示例: start");
        (void)wifi_cmd_SendLine("成功回包: OK start telemetry=on");
        (void)wifi_cmd_SendLine("OK help start");
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "stop"))
    {
        (void)wifi_cmd_SendLine("主题: stop");
        (void)wifi_cmd_SendLine("用法: stop");
        (void)wifi_cmd_SendLine("功能: 停止待机状态下的 JustFloat 遥测发送");
        (void)wifi_cmd_SendLine("限制: 仅待机且未解锁时允许执行");
        (void)wifi_cmd_SendLine("说明: 不影响 imu/list/get/set 等文本命令回包");
        (void)wifi_cmd_SendLine("示例: stop");
        (void)wifi_cmd_SendLine("成功回包: OK stop telemetry=off");
        (void)wifi_cmd_SendLine("OK help stop");
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "list"))
    {
        (void)wifi_cmd_SendLine("主题: list");
        (void)wifi_cmd_SendLine("用法: list");
        (void)wifi_cmd_SendLine("功能: 输出全部白名单参数和当前值");
        (void)wifi_cmd_SendLine("限制: 仅待机且未解锁时允许执行");
        (void)wifi_cmd_SendLine("说明: 第一行会先返回参数总数量");
        (void)wifi_cmd_SendLine("示例: list");
        (void)wifi_cmd_SendLine("回包1: OK list begin count=N");
        (void)wifi_cmd_SendLine("回包2: 参数名=数值");
        (void)wifi_cmd_SendLine("回包3: OK list end");
        (void)wifi_cmd_SendLine("OK help list");
        return 1U;
    }

    return 0U;
}

/*
 * 函数名: wifi_params_process_help
 * 功能: 输出总帮助或指定主题帮助
 * 输入参数:
 *   topic - 为空时输出总帮助，非空时输出指定主题帮助
 * 返回值: 无
 */
static void wifi_params_process_help(const char *topic)
{
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_HELP, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);

    if ((NULL != topic) && ('\0' != topic[0]) && (0U == wifi_cmd_is_help_flag(topic)))
    {
        if (0U != wifi_params_process_help_topic(topic))
        {
            return;
        }

        wifi_params_reply_error(WIFI_PARAMS_COMMAND_HELP, WIFI_PARAMS_RESULT_ERR_UNKNOWN_CMD, 0U, 0.0f, "unknown command");
        return;
    }

    (void)wifi_cmd_SendLine("主题: 总帮助");
    (void)wifi_cmd_SendLine("用法1: help");
    (void)wifi_cmd_SendLine("用法2: help <命令>");
    (void)wifi_cmd_SendLine("用法3: <命令> --help");
    (void)wifi_cmd_SendLine("命令1: ping help get set save load");
    (void)wifi_cmd_SendLine("命令2: start stop list");
    (void)wifi_cmd_SendLine("命令3: imu help / imu status / imu start ...");
    (void)wifi_cmd_SendLine("说明1: 命令字和参数名不区分大小写");
    (void)wifi_cmd_SendLine("说明2: 文本命令必须以 CRLF 结束");
    (void)wifi_cmd_SendLine("说明3: start/stop 只控制待机遥测");
    (void)wifi_cmd_SendLine("示例1: help imu");
    (void)wifi_cmd_SendLine("示例2: imu help");
    (void)wifi_cmd_SendLine("OK help");
}

/*
 * 函数名: wifi_params_process_get
 * 功能: 处理单参数查询命令
 * 输入参数:
 *   name - 参数名
 * 返回值: 无
 */
static void wifi_params_process_get(const char *name)
{
    const wifi_params_entry_t *entry;
    uint16_t param_index;
    float value;

    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_GET, WIFI_PARAMS_RESULT_ERR_STATE, 0U, 0.0f, "state");
        return;
    }

    entry = wifi_params_find_entry(name);
    if (NULL == entry)
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_GET, WIFI_PARAMS_RESULT_ERR_UNKNOWN_PARAM, 0U, 0.0f, "unknown param");
        return;
    }

    param_index = wifi_params_entry_index(entry);
    value = wifi_params_entry_read_as_float(entry);
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_GET, WIFI_PARAMS_RESULT_OK, param_index, value);
    (void)wifi_cmd_SendLine("OK %s=%.6g", entry->name, (double)value);
}

/*
 * 函数名: wifi_params_process_set
 * 功能: 处理参数修改命令
 * 输入参数:
 *   name       - 参数名
 *   value_text - 参数值文本
 * 返回值: 无
 */
static void wifi_params_process_set(const char *name, const char *value_text)
{
    const wifi_params_entry_t *entry;
    uint16_t param_index;
    float value;
    int32_t int_value;
    float applied_value;

    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_STATE, 0U, 0.0f, "set 仅允许待机且未解锁时执行");
        return;
    }

    entry = wifi_params_find_entry(name);
    if (NULL == entry)
    {
        wifi_params_set_diag(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_UNKNOWN_PARAM, 0U, 0.0f);
        (void)wifi_cmd_SendLine("ERR set 未知参数: %s", (NULL != name) ? name : "(null)");
        return;
    }

    if (WIFI_PARAMS_VALUE_INT32 == entry->value_type)
    {
        if (0U == wifi_params_parse_int32(value_text, &int_value))
        {
            wifi_params_set_diag(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_FORMAT, wifi_params_entry_index(entry), 0.0f);
            (void)wifi_cmd_SendLine("ERR set 格式错误: set %s <整数>", entry->name);
            return;
        }

        value = (float)int_value;
    }
    else if (0U == wifi_params_parse_float(value_text, &value))
    {
        wifi_params_set_diag(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_FORMAT, wifi_params_entry_index(entry), 0.0f);
        (void)wifi_cmd_SendLine("ERR set 格式错误: set %s <数值>", entry->name);
        return;
    }

    if (0U == wifi_params_entry_write_value(entry, value))
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_RANGE, wifi_params_entry_index(entry), value, "set 数值超出允许范围");
        return;
    }

    FC_Loop_Init();
    param_index = wifi_params_entry_index(entry);
    applied_value = wifi_params_entry_read_as_float(entry);
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_OK, param_index, applied_value);
    (void)wifi_cmd_SendLine("OK set %s=%.6g", entry->name, (double)applied_value);
}

/*
 * 函数名: wifi_params_process_save
 * 功能: 处理参数保存命令
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_params_process_save(void)
{
    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_SAVE, WIFI_PARAMS_RESULT_ERR_STATE, 0U, 0.0f, "state");
        return;
    }

    if (0U == FC_Params_SaveToFlash())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_SAVE, WIFI_PARAMS_RESULT_ERR_FLASH, 0U, 0.0f, "flash");
        return;
    }

    wifi_params_set_diag(WIFI_PARAMS_COMMAND_SAVE, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);
    (void)wifi_cmd_SendLine("OK save");
}

/*
 * 函数名: wifi_params_process_load
 * 功能: 处理参数加载命令
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_params_process_load(void)
{
    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_LOAD, WIFI_PARAMS_RESULT_ERR_STATE, 0U, 0.0f, "state");
        return;
    }

    if (0U == FC_Params_LoadFromFlash())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_LOAD, WIFI_PARAMS_RESULT_ERR_FLASH, 0U, 0.0f, "flash");
        return;
    }

    FC_Loop_Init();
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_LOAD, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);
    (void)wifi_cmd_SendLine("OK load");
}

/*
 * 函数名: wifi_params_process_start
 * 功能: 恢复待机态遥测
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_params_process_start(void)
{
    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_START, WIFI_PARAMS_RESULT_ERR_STATE, 0U, 0.0f, "state");
        return;
    }

    wifi_justfloat_SetStandbyUserEnable(1U);
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_START, WIFI_PARAMS_RESULT_OK, 0U, 1.0f);
    (void)wifi_cmd_SendLine("OK start telemetry=on");
}

/*
 * 函数名: wifi_params_process_stop
 * 功能: 停止待机态遥测
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_params_process_stop(void)
{
    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_STOP, WIFI_PARAMS_RESULT_ERR_STATE, 0U, 0.0f, "state");
        return;
    }

    wifi_justfloat_SetStandbyUserEnable(0U);
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_STOP, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);
    (void)wifi_cmd_SendLine("OK stop telemetry=off");
}

/*
 * 函数名: wifi_params_process_list
 * 功能: 输出全部白名单参数
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_params_process_list(void)
{
    uint16_t i;
    uint16_t count;

    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_LIST, WIFI_PARAMS_RESULT_ERR_STATE, 0U, 0.0f, "state");
        return;
    }

    count = wifi_params_entry_count();
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_LIST, WIFI_PARAMS_RESULT_OK, 0U, (float)count);
    (void)wifi_cmd_SendLine("OK list begin count=%u", (unsigned int)count);
    for (i = 0U; i < count; i++)
    {
        (void)wifi_params_send_value_line(&s_wifi_params_table[i]);
    }
    (void)wifi_cmd_SendLine("OK list end");
}

void wifi_params_Init(void)
{
    memset(&s_wifi_params_diag, 0, sizeof(s_wifi_params_diag));
}

void wifi_params_ProcessLine(char *line)
{
    char *trimmed_line;
    char *cursor;
    char *token_cmd;
    char *token_arg1;
    char *token_arg2;
    char *token_arg3;

    if (NULL == line)
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_NONE, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
        return;
    }

    trimmed_line = wifi_cmd_trim_line(line);
    if ((NULL == trimmed_line) || ('\0' == trimmed_line[0]))
    {
        return;
    }

    cursor = trimmed_line;
    token_cmd = wifi_cmd_next_token(&cursor);
    token_arg1 = wifi_cmd_next_token(&cursor);
    token_arg2 = wifi_cmd_next_token(&cursor);
    token_arg3 = wifi_cmd_next_token(&cursor);

    if ((NULL == token_cmd) || (NULL != token_arg3))
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_NONE, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
        return;
    }

    wifi_cmd_ascii_strtolower(token_cmd);
    if (NULL != token_arg1)
    {
        wifi_cmd_ascii_strtolower(token_arg1);
    }

    if (0 == strcmp(token_cmd, "ping"))
    {
        if (NULL == token_arg1)
        {
            wifi_params_process_ping();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_params_process_help("ping");
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "help"))
    {
        if (NULL == token_arg2)
        {
            if ((NULL != token_arg1) && (0U != wifi_cmd_is_help_flag(token_arg1)))
            {
                wifi_params_process_help("help");
            }
            else
            {
                wifi_params_process_help(token_arg1);
            }
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "get"))
    {
        if ((NULL != token_arg1) && (0U == wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_params_process_get(token_arg1);
            return;
        }

        if ((NULL != token_arg1) && (0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_params_process_help("get");
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "set"))
    {
        if ((NULL != token_arg1) && (0U == wifi_cmd_is_help_flag(token_arg1)) && (NULL != token_arg2))
        {
            wifi_params_process_set(token_arg1, token_arg2);
            return;
        }

        if ((NULL != token_arg1) && (0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_params_process_help("set");
            return;
        }

        if ((NULL != token_arg1) && (0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL != token_arg2))
        {
            wifi_params_reply_error(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "save"))
    {
        if (NULL == token_arg1)
        {
            wifi_params_process_save();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_params_process_help("save");
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "load"))
    {
        if (NULL == token_arg1)
        {
            wifi_params_process_load();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_params_process_help("load");
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "start"))
    {
        if (NULL == token_arg1)
        {
            wifi_params_process_start();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_params_process_help("start");
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "stop"))
    {
        if (NULL == token_arg1)
        {
            wifi_params_process_stop();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_params_process_help("stop");
            return;
        }
    }
    else if (0 == strcmp(token_cmd, "list"))
    {
        if (NULL == token_arg1)
        {
            wifi_params_process_list();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_params_process_help("list");
            return;
        }
    }

    if ((NULL != token_arg1) && (0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
    {
        wifi_params_process_help(token_cmd);
        return;
    }

    if ((0 == strcmp(token_cmd, "get")) || (0 == strcmp(token_cmd, "set")) || (0 == strcmp(token_cmd, "save")) ||
        (0 == strcmp(token_cmd, "load")) || (0 == strcmp(token_cmd, "start")) || (0 == strcmp(token_cmd, "stop")) ||
        (0 == strcmp(token_cmd, "list")) || (0 == strcmp(token_cmd, "ping")) || (0 == strcmp(token_cmd, "help")))
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_NONE, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
        return;
    }

    wifi_params_reply_error(WIFI_PARAMS_COMMAND_NONE, WIFI_PARAMS_RESULT_ERR_UNKNOWN_CMD, 0U, 0.0f, "unknown command");
}

void wifi_params_GetDiag(wifi_params_diag_t *diag)
{
    if (NULL == diag)
    {
        return;
    }

    *diag = s_wifi_params_diag;
}
