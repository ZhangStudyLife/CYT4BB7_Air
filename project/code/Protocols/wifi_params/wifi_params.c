/*****************************************************************************
 * 文件: wifi_params.c
 * 模块: WiFi 参数调节
 * 职责: 通过现有 WiFi SPI-UDP 链路接收人工可读文本命令，并在待机状态安全调节飞控参数
 *****************************************************************************/

#include "wifi_params.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FlightController/fc_params.h"
#include "../../FlightController/fc_loop.h"
#include "../../FlightController/fc_start_crsf.h"
#include "../wifi_vofa/wifi_vofa.h"
#include "zf_device_wifi_spi.h"

/* WiFi 参数接收缓存长度，单位字节 */
#define WIFI_PARAMS_RX_BUFFER_SIZE (128U)
/* WiFi 参数单行文本回包最大长度，单位字节 */
#define WIFI_PARAMS_TX_LINE_MAX    (128U)

/* WiFi 参数值类型：区分 float 与 int32_t 参数 */
typedef enum
{
    WIFI_PARAMS_VALUE_FLOAT = 0,
    WIFI_PARAMS_VALUE_INT32 = 1
} wifi_params_value_type_e;

/* WiFi 参数白名单表项：描述名称、目标地址、类型与安全范围 */
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

/* WiFi 参数白名单表：只允许修改 g_fc_params 中显式开放的控制参数 */
static const wifi_params_entry_t s_wifi_params_table[] =
{
    WIFI_PARAMS_INT_ITEM("base_throttle", base_throttle, 0, 6000),

    WIFI_PARAMS_FLOAT_ITEM("roll_gyro_kp", roll_gyro_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_gyro_ki", roll_gyro_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_gyro_kd", roll_gyro_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_gyro_kff", roll_gyro_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_gyro_i_limit", roll_gyro_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_gyro_d_lpf", roll_gyro_d_lpf, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("pitch_gyro_kp", pitch_gyro_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_gyro_ki", pitch_gyro_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_gyro_kd", pitch_gyro_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_gyro_kff", pitch_gyro_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_gyro_i_limit", pitch_gyro_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_gyro_d_lpf", pitch_gyro_d_lpf, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("yaw_gyro_kp", yaw_gyro_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_gyro_ki", yaw_gyro_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_gyro_kd", yaw_gyro_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_gyro_kff", yaw_gyro_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_gyro_i_limit", yaw_gyro_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_gyro_d_lpf", yaw_gyro_d_lpf, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("roll_angle_kp", roll_angle_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_angle_ki", roll_angle_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_angle_kd", roll_angle_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_angle_kff", roll_angle_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_angle_i_limit", roll_angle_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("roll_angle_d_lpf", roll_angle_d_lpf, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("pitch_angle_kp", pitch_angle_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_angle_ki", pitch_angle_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_angle_kd", pitch_angle_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_angle_kff", pitch_angle_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_angle_i_limit", pitch_angle_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pitch_angle_d_lpf", pitch_angle_d_lpf, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("yaw_angle_kp", yaw_angle_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_angle_ki", yaw_angle_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_angle_kd", yaw_angle_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_angle_kff", yaw_angle_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_angle_i_limit", yaw_angle_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("yaw_angle_d_lpf", yaw_angle_d_lpf, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("pos_x_kp", pos_x_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_x_ki", pos_x_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_x_kd", pos_x_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_x_kff", pos_x_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_x_i_limit", pos_x_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_x_d_lpf", pos_x_d_lpf, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("pos_y_kp", pos_y_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_y_ki", pos_y_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_y_kd", pos_y_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_y_kff", pos_y_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_y_i_limit", pos_y_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_y_d_lpf", pos_y_d_lpf, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("pos_z_kp", pos_z_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_z_ki", pos_z_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_z_kd", pos_z_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_z_kff", pos_z_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_z_i_limit", pos_z_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("pos_z_d_lpf", pos_z_d_lpf, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("vel_x_kp", vel_x_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_x_ki", vel_x_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_x_kd", vel_x_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_x_kff", vel_x_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_x_i_limit", vel_x_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_x_d_lpf", vel_x_d_lpf, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("vel_y_kp", vel_y_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_y_ki", vel_y_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_y_kd", vel_y_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_y_kff", vel_y_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_y_i_limit", vel_y_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_y_d_lpf", vel_y_d_lpf, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("vel_z_kp", vel_z_kp, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_z_ki", vel_z_ki, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_z_kd", vel_z_kd, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_z_kff", vel_z_kff, 0.0f, 3000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_z_i_limit", vel_z_i_limit, 0.0f, 5000.0f),
    WIFI_PARAMS_FLOAT_ITEM("vel_z_d_lpf", vel_z_d_lpf, 0.0f, 1.0f),

    WIFI_PARAMS_FLOAT_ITEM("mode1_track_ff_deg_per_cmps", mode1_track_ff_deg_per_cmps, 0.0f, 1.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode1_brake_kp", mode1_brake_kp, 0.0f, 50.0f),
    WIFI_PARAMS_FLOAT_ITEM("mode1_brake_exit_vel_cmps", mode1_brake_exit_vel_cmps, 0.0f, 300.0f)
};

/* WiFi 参数最近一次处理诊断信息 */
static wifi_params_diag_t s_wifi_params_diag = {0};
/* WiFi 参数当前文本行缓存 */
static char s_wifi_params_line[WIFI_PARAMS_LINE_MAX] = {0};
/* WiFi 参数当前文本行长度 */
static uint16_t s_wifi_params_line_len = 0U;
/* WiFi 参数当前文本行是否溢出 */
static uint8_t s_wifi_params_line_overflow = 0U;
/* WiFi 参数当前文本行是否含非法字符 */
static uint8_t s_wifi_params_line_invalid = 0U;
static uint8_t s_wifi_params_line_expect_lf = 0U;
/* WiFi 参数是否存在待应用的新参数 */
static uint8_t s_wifi_params_pending_apply = 0U;

/*
 * 函数名: wifi_params_set_diag
 * 功能: 更新最近一次处理诊断状态
 * 输入参数:
 *   command_code - 命令码
 *   result_code  - 结果码
 *   param_index  - 参数索引，0 表示无参数
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

/* 重置当前文本行解析状态，供严格 CRLF 状态机复用 */
static void wifi_params_reset_line_state(void)
{
    s_wifi_params_line_len = 0U;
    s_wifi_params_line_overflow = 0U;
    s_wifi_params_line_invalid = 0U;
    s_wifi_params_line_expect_lf = 0U;
}

/*
 * 函数名: wifi_params_is_edit_allowed
 * 功能: 判断当前是否允许参数读写与保存
 * 输入参数: 无
 * 返回值: 1=允许，0=不允许
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
 * 函数名: wifi_params_is_space_char
 * 功能: 判断字符是否为空白分隔符
 * 输入参数:
 *   ch - 待判断字符
 * 返回值: 1=空白，0=非空白
 */
static uint8_t wifi_params_is_space_char(char ch)
{
    return ((' ' == ch) || ('\t' == ch)) ? 1U : 0U;
}

/*
 * 函数名: wifi_params_ascii_tolower
 * 功能: 将单个 ASCII 字符转换为小写
 * 输入参数:
 *   ch - 待转换字符
 * 返回值:
 *   转换后的字符
 */
static char wifi_params_ascii_tolower(char ch)
{
    if ((ch >= 'A') && (ch <= 'Z'))
    {
        return (char)(ch - 'A' + 'a');
    }

    return ch;
}

/*
 * 函数名: wifi_params_ascii_stricmp
 * 功能: 进行 ASCII 字符串大小写无关比较
 * 输入参数:
 *   lhs - 左字符串
 *   rhs - 右字符串
 * 返回值:
 *   0=相等，非0=不相等
 */
static int wifi_params_ascii_stricmp(const char *lhs, const char *rhs)
{
    char left_ch;
    char right_ch;

    if ((NULL == lhs) || (NULL == rhs))
    {
        return (lhs == rhs) ? 0 : 1;
    }

    while (('\0' != *lhs) && ('\0' != *rhs))
    {
        left_ch = wifi_params_ascii_tolower(*lhs);
        right_ch = wifi_params_ascii_tolower(*rhs);
        if (left_ch != right_ch)
        {
            return ((int)(unsigned char)left_ch - (int)(unsigned char)right_ch);
        }

        lhs++;
        rhs++;
    }

    left_ch = wifi_params_ascii_tolower(*lhs);
    right_ch = wifi_params_ascii_tolower(*rhs);
    return ((int)(unsigned char)left_ch - (int)(unsigned char)right_ch);
}

/*
 * 函数名: wifi_params_ascii_strtolower
 * 功能: 将 ASCII 字符串原地转换为小写
 * 输入参数:
 *   text - 待转换字符串
 * 返回值: 无
 */
static void wifi_params_ascii_strtolower(char *text)
{
    if (NULL == text)
    {
        return;
    }

    while ('\0' != *text)
    {
        *text = wifi_params_ascii_tolower(*text);
        text++;
    }
}

/*
 * 鍑芥暟鍚? wifi_params_is_help_flag
 * 鍔熻兘: 鍒ゆ柇瀛楃涓叉槸鍚︿负甯姪鏍囪
 * 杈撳叆鍙傛暟:
 *   text - 寰呭垽鏂瓧绗︿覆
 * 杩斿洖鍊?
 *   1=鏄府鍔╂爣璁帮紝0=涓嶆槸
 */
static uint8_t wifi_params_is_help_flag(const char *text)
{
    if (NULL == text)
    {
        return 0U;
    }

    if (0 == wifi_params_ascii_stricmp(text, "--help"))
    {
        return 1U;
    }

    return (0 == wifi_params_ascii_stricmp(text, "-h")) ? 1U : 0U;
}

/*
 * 函数名: wifi_params_trim_line
 * 功能: 去掉文本行首尾空白
 * 输入参数:
 *   text - 文本行首地址
 * 返回值:
 *   去掉首尾空白后的首地址
 */
static char *wifi_params_trim_line(char *text)
{
    char *end;

    if (NULL == text)
    {
        return NULL;
    }

    while (wifi_params_is_space_char(*text))
    {
        text++;
    }

    if ('\0' == *text)
    {
        return text;
    }

    end = text + strlen(text) - 1;
    while ((end >= text) && wifi_params_is_space_char(*end))
    {
        *end = '\0';
        end--;
    }

    return text;
}

/*
 * 函数名: wifi_params_next_token
 * 功能: 依次提取空白分隔字段
 * 输入参数:
 *   cursor - 当前解析游标地址
 * 返回值:
 *   当前字段首地址；无字段时返回 NULL
 */
static char *wifi_params_next_token(char **cursor)
{
    char *token;
    char *end;

    if ((NULL == cursor) || (NULL == *cursor))
    {
        return NULL;
    }

    while (wifi_params_is_space_char(**cursor))
    {
        (*cursor)++;
    }

    if ('\0' == **cursor)
    {
        *cursor = NULL;
        return NULL;
    }

    token = *cursor;
    end = token;
    while ((*end != '\0') && (0U == wifi_params_is_space_char(*end)))
    {
        end++;
    }

    if ('\0' == *end)
    {
        *cursor = NULL;
    }
    else
    {
        *end = '\0';
        *cursor = end + 1;
    }

    return token;
}

/*
 * 函数名: wifi_params_parse_float
 * 功能: 解析 float 字符串并过滤 NaN/Inf
 * 输入参数:
 *   text  - 待解析字符串
 *   value - 输出值指针
 * 返回值:
 *   1=解析成功，0=解析失败
 */
static uint8_t wifi_params_parse_float(const char *text, float *value)
{
    char *endptr = NULL;
    float parsed;

    if ((NULL == text) || (NULL == value) || (text[0] == '\0'))
    {
        return 0U;
    }

    parsed = strtof(text, &endptr);
    if ((NULL == endptr) || (*endptr != '\0'))
    {
        return 0U;
    }

    if ((parsed != parsed) || (parsed > 1000000000.0f) || (parsed < -1000000000.0f))
    {
        return 0U;
    }

    *value = parsed;
    return 1U;
}

/*
 * 函数名: wifi_params_find_entry
 * 功能: 根据参数名查找白名单表项
 * 输入参数:
 *   name - 参数名字字符串
 * 返回值:
 *   匹配表项指针；未找到时返回 NULL
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
        if (0 == wifi_params_ascii_stricmp(name, s_wifi_params_table[i].name))
        {
            return &s_wifi_params_table[i];
        }
    }

    return NULL;
}

/*
 * 函数名: wifi_params_entry_index
 * 功能: 计算表项的 1 基参数索引
 * 输入参数:
 *   entry - 白名单表项指针
 * 返回值:
 *   1 基参数索引，失败时返回 0
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
 * 功能: 获取白名单参数数量
 * 输入参数: 无
 * 返回值:
 *   白名单参数数量
 */
static uint16_t wifi_params_entry_count(void)
{
    return (uint16_t)(sizeof(s_wifi_params_table) / sizeof(s_wifi_params_table[0]));
}

/*
 * 函数名: wifi_params_entry_read_as_float
 * 功能: 将白名单表项当前值统一转换为 float
 * 输入参数:
 *   entry - 白名单表项指针
 * 返回值:
 *   当前参数值的 float 表示
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
 * 功能: 向白名单表项写入新值
 * 输入参数:
 *   entry - 白名单表项指针
 *   value - 待写入值
 * 返回值:
 *   1=写入成功，0=写入失败
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
        int32_t int_value = (int32_t)value;
        if ((float)int_value != value)
        {
            return 0U;
        }
        *(int32_t *)entry->value_ptr = int_value;
    }
    else
    {
        *(float *)entry->value_ptr = value;
    }

    return 1U;
}

/*
 * 函数名: wifi_params_send_text
 * 功能: 发送一条文本数据并在 UDP 模式下立即发出
 * 输入参数:
 *   text - 待发送文本
 * 返回值:
 *   1=发送成功，0=发送失败
 */
static uint8_t wifi_params_send_text(const char *text)
{
    uint32_t text_len;
    uint32_t remain_len;

    if (NULL == text)
    {
        return 0U;
    }

    text_len = (uint32_t)strlen(text);
    if (0U == text_len)
    {
        return 0U;
    }

    remain_len = wifi_spi_send_buffer((const uint8_t *)text, text_len);
    (void)wifi_spi_udp_send_now();
    return (0U == remain_len) ? 1U : 0U;
}

/*
 * 函数名: wifi_params_send_line
 * 功能: 按格式发送一行文本回包，自动补齐 CRLF
 * 输入参数:
 *   format - 格式化字符串
 *   ...    - 可变参数
 * 返回值:
 *   1=发送成功，0=发送失败
 */
static uint8_t wifi_params_send_line(const char *format, ...)
{
    char line[WIFI_PARAMS_TX_LINE_MAX];
    int write_len;
    va_list ap;

    if (NULL == format)
    {
        return 0U;
    }

    va_start(ap, format);
    write_len = vsnprintf(line, (int)(sizeof(line) - 3U), format, ap);
    va_end(ap);

    if (write_len < 0)
    {
        return 0U;
    }

    if ((uint32_t)write_len > (sizeof(line) - 3U))
    {
        write_len = (int)(sizeof(line) - 3U);
    }

    line[write_len + 0] = '\r';
    line[write_len + 1] = '\n';
    line[write_len + 2] = '\0';
    return wifi_params_send_text(line);
}

/*
 * 函数名: wifi_params_send_value_line
 * 功能: 发送单个参数的 name=value 文本行
 * 输入参数:
 *   entry - 白名单表项指针
 * 返回值:
 *   1=发送成功，0=发送失败
 */
static uint8_t wifi_params_send_value_line(const wifi_params_entry_t *entry)
{
    if (NULL == entry)
    {
        return 0U;
    }

    if (WIFI_PARAMS_VALUE_INT32 == entry->value_type)
    {
        return wifi_params_send_line("%s=%ld", entry->name, (long)(*(const int32_t *)entry->value_ptr));
    }

    return wifi_params_send_line("%s=%.6g", entry->name, (double)(*(const float *)entry->value_ptr));
}

/*
 * 函数名: wifi_params_reply_error
 * 功能: 发送错误回包并更新诊断
 * 输入参数:
 *   command_code - 命令码
 *   result_code  - 结果码
 *   param_index  - 参数索引
 *   value        - 相关数值
 *   reason       - 错误原因文本
 * 返回值: 无
 */
static void wifi_params_reply_error(uint8_t command_code, uint8_t result_code, uint16_t param_index, float value, const char *reason)
{
    wifi_params_set_diag(command_code, result_code, param_index, value);
    (void)wifi_params_send_line("ERR %s", (NULL != reason) ? reason : "format");
}

/*
 * 函数名: wifi_params_process_ping
 * 功能: 处理链路探测命令
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_params_process_ping(void)
{
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_PING, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);
    (void)wifi_params_send_line("OK ping");
}

/*
 * 函数名: wifi_params_process_help
 * 功能: 处理帮助命令
 * 输入参数: 无
 * 返回值: 无
 */
#if 0
static uint8_t wifi_params_process_help_topic(const char *topic)
{
    if (NULL == topic)
    {
        return 0U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "ping"))
    {
        (void)wifi_params_send_line("主题: ping");
        (void)wifi_params_send_line("用法: ping");
        (void)wifi_params_send_line("说明: 检查链路是否畅通");
        (void)wifi_params_send_line("示例: ping");
        (void)wifi_params_send_line("回包: OK ping");
        (void)wifi_params_send_line("OK help ping");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "help"))
    {
        (void)wifi_params_send_line("主题: help");
        (void)wifi_params_send_line("用法1: help");
        (void)wifi_params_send_line("用法2: help <命令>");
        (void)wifi_params_send_line("用法3: <命令> --help");
        (void)wifi_params_send_line("说明: 输出总帮助或子命令帮助");
        (void)wifi_params_send_line("示例1: help list");
        (void)wifi_params_send_line("示例2: list --help");
        (void)wifi_params_send_line("OK help help");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "get"))
    {
        (void)wifi_params_send_line("主题: get");
        (void)wifi_params_send_line("用法: get <参数名>");
        (void)wifi_params_send_line("说明: 读取当前参数值");
        (void)wifi_params_send_line("限制: 仅待机且未解锁");
        (void)wifi_params_send_line("示例: get base_throttle");
        (void)wifi_params_send_line("回包: OK <参数名>=<值>");
        (void)wifi_params_send_line("OK help get");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "set"))
    {
        (void)wifi_params_send_line("主题: set");
        (void)wifi_params_send_line("用法: set <参数名> <数值>");
        (void)wifi_params_send_line("说明: 修改 RAM 参数并立即应用");
        (void)wifi_params_send_line("限制: 仅待机且未解锁");
        (void)wifi_params_send_line("注意: 断电不保存, 需 save");
        (void)wifi_params_send_line("示例: set roll_angle_kp 5.5");
        (void)wifi_params_send_line("回包: OK set <参数名>=<值>");
        (void)wifi_params_send_line("OK help set");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "save"))
    {
        (void)wifi_params_send_line("主题: save");
        (void)wifi_params_send_line("用法: save");
        (void)wifi_params_send_line("说明: 将当前RAM参数写入 Flash");
        (void)wifi_params_send_line("限制: 仅待机且未解锁");
        (void)wifi_params_send_line("示例: save");
        (void)wifi_params_send_line("回包: OK save");
        (void)wifi_params_send_line("OK help save");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "load"))
    {
        (void)wifi_params_send_line("主题: load");
        (void)wifi_params_send_line("用法: load");
        (void)wifi_params_send_line("说明: 从 Flash 读回参数");
        (void)wifi_params_send_line("限制: 仅待机且未解锁");
        (void)wifi_params_send_line("注意: 成功后会重新应用 PID");
        (void)wifi_params_send_line("示例: load");
        (void)wifi_params_send_line("回包: OK load");
        (void)wifi_params_send_line("OK help load");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "start"))
    {
        (void)wifi_params_send_line("主题: start");
        (void)wifi_params_send_line("用法: start");
        (void)wifi_params_send_line("说明: 恢复待机态遥测发送");
        (void)wifi_params_send_line("限制: 仅待机且未解锁");
        (void)wifi_params_send_line("注意: 飞行时遥测始终发送");
        (void)wifi_params_send_line("示例: start");
        (void)wifi_params_send_line("回包: OK start telemetry=on");
        (void)wifi_params_send_line("OK help start");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "stop"))
    {
        (void)wifi_params_send_line("主题: stop");
        (void)wifi_params_send_line("用法: stop");
        (void)wifi_params_send_line("说明: 停止待机态遥测发送");
        (void)wifi_params_send_line("限制: 仅待机且未解锁");
        (void)wifi_params_send_line("注意: 文本命令回包不受影响");
        (void)wifi_params_send_line("示例: stop");
        (void)wifi_params_send_line("回包: OK stop telemetry=off");
        (void)wifi_params_send_line("OK help stop");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "list"))
    {
        (void)wifi_params_send_line("主题: list");
        (void)wifi_params_send_line("用法: list");
        (void)wifi_params_send_line("说明: 输出全部白名单参数");
        (void)wifi_params_send_line("限制: 仅待机且未解锁");
        (void)wifi_params_send_line("示例: list");
        (void)wifi_params_send_line("回包1: OK list begin count=N");
        (void)wifi_params_send_line("回包2: <参数名>=<值>");
        (void)wifi_params_send_line("回包3: OK list end");
        (void)wifi_params_send_line("OK help list");
        return 1U;
    }

    return 0U;
}

static void wifi_params_process_help(const char *topic)
{
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_HELP, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);

    if ((NULL != topic) && ('\0' != topic[0]) && (0U == wifi_params_is_help_flag(topic)))
    {
        if (0U != wifi_params_process_help_topic(topic))
        {
            return;
        }

        wifi_params_reply_error(WIFI_PARAMS_COMMAND_HELP, WIFI_PARAMS_RESULT_ERR_UNKNOWN_CMD, 0U, 0.0f, "unknown command");
        return;
    }

    (void)wifi_params_send_line("主题: 总帮助");
    (void)wifi_params_send_line("用法1: help");
    (void)wifi_params_send_line("用法2: help <命令>");
    (void)wifi_params_send_line("用法3: <命令> --help");
    (void)wifi_params_send_line("命令1: ping help get set save load");
    (void)wifi_params_send_line("命令2: start stop list");
    (void)wifi_params_send_line("说明: 命令和参数名不区分大小写");
    (void)wifi_params_send_line("说明: 文本命令必须以 CRLF 结束");
    (void)wifi_params_send_line("示例: help list / list --help");
    (void)wifi_params_send_line("OK help");
}

/*
 * 函数名: wifi_params_process_get
 * 功能: 处理参数查询命令
 * 输入参数:
 *   name - 参数名
 * 返回值: 无
 */
#endif

/*
 * 函数名: wifi_params_process_help_topic
 * 功能: 按命令主题输出详细中文帮助
 * 输入参数:
 *   topic - 帮助主题名称，例如 ping、set、list
 * 返回值:
 *   1=主题存在且已输出帮助，0=主题不存在
 */
static uint8_t wifi_params_process_help_topic(const char *topic)
{
    if (NULL == topic)
    {
        return 0U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "ping"))
    {
        (void)wifi_params_send_line("主题: ping");
        (void)wifi_params_send_line("用法: ping");
        (void)wifi_params_send_line("功能: 检查当前 UDP 链路是否畅通");
        (void)wifi_params_send_line("状态: 任意状态都允许执行");
        (void)wifi_params_send_line("示例: ping");
        (void)wifi_params_send_line("成功回包: OK ping");
        (void)wifi_params_send_line("OK help ping");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "help"))
    {
        (void)wifi_params_send_line("主题: help");
        (void)wifi_params_send_line("用法1: help");
        (void)wifi_params_send_line("用法2: help <命令>");
        (void)wifi_params_send_line("用法3: <命令> --help");
        (void)wifi_params_send_line("功能: 输出总帮助或命令的详细帮助");
        (void)wifi_params_send_line("示例1: help list");
        (void)wifi_params_send_line("示例2: list --help");
        (void)wifi_params_send_line("OK help help");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "get"))
    {
        (void)wifi_params_send_line("主题: get");
        (void)wifi_params_send_line("用法: get <参数名>");
        (void)wifi_params_send_line("功能: 读取白名单参数的当前值");
        (void)wifi_params_send_line("限制: 仅待机且未解锁时允许执行");
        (void)wifi_params_send_line("说明: 参数名不区分大小写");
        (void)wifi_params_send_line("示例: get base_throttle");
        (void)wifi_params_send_line("成功回包: OK base_throttle=3500");
        (void)wifi_params_send_line("OK help get");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "set"))
    {
        (void)wifi_params_send_line("主题: set");
        (void)wifi_params_send_line("用法: set <参数名> <数值>");
        (void)wifi_params_send_line("功能: 修改 RAM 参数并立即应用");
        (void)wifi_params_send_line("限制: 仅待机且未解锁时允许执行");
        (void)wifi_params_send_line("说明1: 参数名不区分大小写");
        (void)wifi_params_send_line("说明2: 断电不会保存，需要再执行 save");
        (void)wifi_params_send_line("示例: set roll_angle_kp 5.5");
        (void)wifi_params_send_line("成功回包: OK set roll_angle_kp=5.5");
        (void)wifi_params_send_line("OK help set");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "save"))
    {
        (void)wifi_params_send_line("主题: save");
        (void)wifi_params_send_line("用法: save");
        (void)wifi_params_send_line("功能: 将当前参数永久保存到 Flash");
        (void)wifi_params_send_line("限制: 仅待机且未解锁时允许执行");
        (void)wifi_params_send_line("说明: save 保存的是当前 RAM 参数");
        (void)wifi_params_send_line("示例: save");
        (void)wifi_params_send_line("成功回包: OK save");
        (void)wifi_params_send_line("OK help save");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "load"))
    {
        (void)wifi_params_send_line("主题: load");
        (void)wifi_params_send_line("用法: load");
        (void)wifi_params_send_line("功能: 从 Flash 读取上次保存的参数");
        (void)wifi_params_send_line("限制: 仅待机且未解锁时允许执行");
        (void)wifi_params_send_line("说明: 成功后会重新应用控制器参数");
        (void)wifi_params_send_line("示例: load");
        (void)wifi_params_send_line("成功回包: OK load");
        (void)wifi_params_send_line("OK help load");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "start"))
    {
        (void)wifi_params_send_line("主题: start");
        (void)wifi_params_send_line("用法: start");
        (void)wifi_params_send_line("功能: 恢复待机状态下的遥测发送");
        (void)wifi_params_send_line("限制: 仅待机且未解锁时允许执行");
        (void)wifi_params_send_line("说明: 飞行中遥测本来就始终发送");
        (void)wifi_params_send_line("示例: start");
        (void)wifi_params_send_line("成功回包: OK start telemetry=on");
        (void)wifi_params_send_line("OK help start");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "stop"))
    {
        (void)wifi_params_send_line("主题: stop");
        (void)wifi_params_send_line("用法: stop");
        (void)wifi_params_send_line("功能: 停止待机状态下的遥测发送");
        (void)wifi_params_send_line("限制: 仅待机且未解锁时允许执行");
        (void)wifi_params_send_line("说明: 不影响 help/list/get/set 等文本回包");
        (void)wifi_params_send_line("示例: stop");
        (void)wifi_params_send_line("成功回包: OK stop telemetry=off");
        (void)wifi_params_send_line("OK help stop");
        return 1U;
    }

    if (0 == wifi_params_ascii_stricmp(topic, "list"))
    {
        (void)wifi_params_send_line("主题: list");
        (void)wifi_params_send_line("用法: list");
        (void)wifi_params_send_line("功能: 输出全部白名单参数和当前值");
        (void)wifi_params_send_line("限制: 仅待机且未解锁时允许执行");
        (void)wifi_params_send_line("说明: 第一行会先返回参数总数量");
        (void)wifi_params_send_line("示例: list");
        (void)wifi_params_send_line("回包1: OK list begin count=N");
        (void)wifi_params_send_line("回包2: 参数名=数值");
        (void)wifi_params_send_line("回包3: OK list end");
        (void)wifi_params_send_line("OK help list");
        return 1U;
    }

    return 0U;
}

/*
 * 函数名: wifi_params_process_help
 * 功能: 输出总帮助或指定命令的详细帮助
 * 输入参数:
 *   topic - 为空时输出总帮助，非空时输出对应命令帮助
 * 返回值: 无
 */
static void wifi_params_process_help(const char *topic)
{
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_HELP, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);

    if ((NULL != topic) && ('\0' != topic[0]) && (0U == wifi_params_is_help_flag(topic)))
    {
        if (0U != wifi_params_process_help_topic(topic))
        {
            return;
        }

        wifi_params_reply_error(WIFI_PARAMS_COMMAND_HELP, WIFI_PARAMS_RESULT_ERR_UNKNOWN_CMD, 0U, 0.0f, "unknown command");
        return;
    }

    (void)wifi_params_send_line("主题: 总帮助");
    (void)wifi_params_send_line("用法1: help");
    (void)wifi_params_send_line("用法2: help <命令>");
    (void)wifi_params_send_line("用法3: <命令> --help");
    (void)wifi_params_send_line("命令1: ping help get set save load");
    (void)wifi_params_send_line("命令2: start stop list");
    (void)wifi_params_send_line("说明1: 命令字和参数名不区分大小写");
    (void)wifi_params_send_line("说明2: 文本命令必须以 CRLF 结束");
    (void)wifi_params_send_line("说明3: start/stop 只控制待机遥测");
    (void)wifi_params_send_line("示例1: help list");
    (void)wifi_params_send_line("示例2: list --help");
    (void)wifi_params_send_line("OK help");
}

static void wifi_params_process_get(const char *name)
{
    const wifi_params_entry_t *entry = wifi_params_find_entry(name);
    uint16_t param_index;
    float value;

    if (NULL == entry)
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_GET, WIFI_PARAMS_RESULT_ERR_UNKNOWN_PARAM, 0U, 0.0f, "unknown param");
        return;
    }

    param_index = wifi_params_entry_index(entry);
    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_GET, WIFI_PARAMS_RESULT_ERR_STATE, param_index, 0.0f, "state");
        return;
    }

    value = wifi_params_entry_read_as_float(entry);
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_GET, WIFI_PARAMS_RESULT_OK, param_index, value);

    if (WIFI_PARAMS_VALUE_INT32 == entry->value_type)
    {
        (void)wifi_params_send_line("OK %s=%ld", entry->name, (long)(*(const int32_t *)entry->value_ptr));
    }
    else
    {
        (void)wifi_params_send_line("OK %s=%.6g", entry->name, (double)(*(const float *)entry->value_ptr));
    }
}

/*
 * 函数名: wifi_params_process_set
 * 功能: 处理参数设置命令
 * 输入参数:
 *   name  - 参数名
 *   value - 参数值
 * 返回值: 无
 */
static void wifi_params_process_set(const char *name, float value)
{
    const wifi_params_entry_t *entry = wifi_params_find_entry(name);
    uint16_t param_index;

    if (NULL == entry)
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_UNKNOWN_PARAM, 0U, value, "unknown param");
        return;
    }

    param_index = wifi_params_entry_index(entry);
    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_STATE, param_index, value, "state");
        return;
    }

    if (0U == wifi_params_entry_write_value(entry, value))
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_RANGE, param_index, value, "range");
        return;
    }

    s_wifi_params_pending_apply = 1U;
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_OK, param_index, value);

    if (WIFI_PARAMS_VALUE_INT32 == entry->value_type)
    {
        (void)wifi_params_send_line("OK set %s=%ld", entry->name, (long)(*(const int32_t *)entry->value_ptr));
    }
    else
    {
        (void)wifi_params_send_line("OK set %s=%.6g", entry->name, (double)(*(const float *)entry->value_ptr));
    }
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
    (void)wifi_params_send_line("OK save");
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

    s_wifi_params_pending_apply = 1U;
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_LOAD, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);
    (void)wifi_params_send_line("OK load");
}

/* 处理 start 命令：恢复待机态 VOFA 遥测发送 */
static void wifi_params_process_start(void)
{
    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_START, WIFI_PARAMS_RESULT_ERR_STATE, 0U, 1.0f, "state");
        return;
    }

    wifi_vofa_SetStandbyUserEnable(1U);
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_START, WIFI_PARAMS_RESULT_OK, 0U, 1.0f);
    (void)wifi_params_send_line("OK start telemetry=on");
}

/* 处理 stop 命令：停止待机态 VOFA 遥测发送 */
static void wifi_params_process_stop(void)
{
    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_STOP, WIFI_PARAMS_RESULT_ERR_STATE, 0U, 0.0f, "state");
        return;
    }

    wifi_vofa_SetStandbyUserEnable(0U);
    wifi_params_set_diag(WIFI_PARAMS_COMMAND_STOP, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);
    (void)wifi_params_send_line("OK stop telemetry=off");
}

/*
 * 函数名: wifi_params_process_list
 * 功能: 处理全参数列表命令
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_params_process_list(void)
{
    uint16_t i;

    if (0U == wifi_params_is_edit_allowed())
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_LIST, WIFI_PARAMS_RESULT_ERR_STATE, 0U, 0.0f, "state");
        return;
    }

    wifi_params_set_diag(WIFI_PARAMS_COMMAND_LIST, WIFI_PARAMS_RESULT_OK, 0U, 0.0f);
    (void)wifi_params_send_line("OK list begin count=%u", (unsigned int)wifi_params_entry_count());
    for (i = 0U; i < wifi_params_entry_count(); i++)
    {
        (void)wifi_params_send_value_line(&s_wifi_params_table[i]);
    }
    (void)wifi_params_send_line("OK list end");
}

/*
 * 函数名: wifi_params_process_line
 * 功能: 解析并处理一条完整文本命令
 * 输入参数:
 *   line - 文本命令缓冲区，函数内部会原地切分字段
 * 返回值: 无
 */
static void wifi_params_process_line(char *line)
{
    char *cursor;
    char *token_cmd;
    char *token_name;
    char *token_value;
    char *token_extra;
    char *token_more;
    float value;

    if (NULL == line)
    {
        return;
    }

    line = wifi_params_trim_line(line);
    if ((NULL == line) || ('\0' == line[0]))
    {
        return;
    }

    cursor = line;
    token_cmd = wifi_params_next_token(&cursor);
    if (NULL == token_cmd)
    {
        return;
    }
    wifi_params_ascii_strtolower(token_cmd);

    if (0 == strcmp(token_cmd, "ping"))
    {
        token_extra = wifi_params_next_token(&cursor);
        if (NULL != token_extra)
        {
            if (0U != wifi_params_is_help_flag(token_extra))
            {
                token_more = wifi_params_next_token(&cursor);
                if (NULL == token_more)
                {
                    wifi_params_process_help("ping");
                    return;
                }
            }

            wifi_params_reply_error(WIFI_PARAMS_COMMAND_PING, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }
        wifi_params_process_ping();
        return;
    }

    if (0 == strcmp(token_cmd, "help"))
    {
        token_name = wifi_params_next_token(&cursor);
        token_extra = wifi_params_next_token(&cursor);
        if (NULL != token_extra)
        {
            wifi_params_reply_error(WIFI_PARAMS_COMMAND_HELP, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }
        if (NULL != token_name)
        {
            wifi_params_ascii_strtolower(token_name);
        }
        wifi_params_process_help(token_name);
        return;
    }

    if (0 == strcmp(token_cmd, "get"))
    {
        token_name = wifi_params_next_token(&cursor);
        token_extra = wifi_params_next_token(&cursor);
        if (NULL == token_name)
        {
            wifi_params_reply_error(WIFI_PARAMS_COMMAND_GET, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }
        if (NULL != token_extra)
        {
            wifi_params_reply_error(WIFI_PARAMS_COMMAND_GET, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }
        if (0U != wifi_params_is_help_flag(token_name))
        {
            wifi_params_process_help("get");
            return;
        }
        wifi_params_ascii_strtolower(token_name);
        wifi_params_process_get(token_name);
        return;
    }

    if (0 == strcmp(token_cmd, "set"))
    {
        token_name = wifi_params_next_token(&cursor);
        token_value = wifi_params_next_token(&cursor);
        token_extra = wifi_params_next_token(&cursor);
        if (NULL == token_name)
        {
            wifi_params_reply_error(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }
        if (0U != wifi_params_is_help_flag(token_name))
        {
            if ((NULL == token_value) && (NULL == token_extra))
            {
                wifi_params_process_help("set");
                return;
            }

            wifi_params_reply_error(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }
        if ((NULL == token_value) || (NULL != token_extra))
        {
            wifi_params_reply_error(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }
        wifi_params_ascii_strtolower(token_name);

        if (0U == wifi_params_parse_float(token_value, &value))
        {
            wifi_params_reply_error(WIFI_PARAMS_COMMAND_SET, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }

        wifi_params_process_set(token_name, value);
        return;
    }

    if (0 == strcmp(token_cmd, "save"))
    {
        token_extra = wifi_params_next_token(&cursor);
        if (NULL != token_extra)
        {
            if (0U != wifi_params_is_help_flag(token_extra))
            {
                token_more = wifi_params_next_token(&cursor);
                if (NULL == token_more)
                {
                    wifi_params_process_help("save");
                    return;
                }
            }

            wifi_params_reply_error(WIFI_PARAMS_COMMAND_SAVE, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }
        wifi_params_process_save();
        return;
    }

    if (0 == strcmp(token_cmd, "load"))
    {
        token_extra = wifi_params_next_token(&cursor);
        if (NULL != token_extra)
        {
            if (0U != wifi_params_is_help_flag(token_extra))
            {
                token_more = wifi_params_next_token(&cursor);
                if (NULL == token_more)
                {
                    wifi_params_process_help("load");
                    return;
                }
            }

            wifi_params_reply_error(WIFI_PARAMS_COMMAND_LOAD, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }
        wifi_params_process_load();
        return;
    }

    if (0 == strcmp(token_cmd, "start"))
    {
        token_extra = wifi_params_next_token(&cursor);
        if (NULL != token_extra)
        {
            if (0U != wifi_params_is_help_flag(token_extra))
            {
                token_more = wifi_params_next_token(&cursor);
                if (NULL == token_more)
                {
                    wifi_params_process_help("start");
                    return;
                }
            }

            wifi_params_reply_error(WIFI_PARAMS_COMMAND_START, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 1.0f, "format");
            return;
        }
        wifi_params_process_start();
        return;
    }

    if (0 == strcmp(token_cmd, "stop"))
    {
        token_extra = wifi_params_next_token(&cursor);
        if (NULL != token_extra)
        {
            if (0U != wifi_params_is_help_flag(token_extra))
            {
                token_more = wifi_params_next_token(&cursor);
                if (NULL == token_more)
                {
                    wifi_params_process_help("stop");
                    return;
                }
            }

            wifi_params_reply_error(WIFI_PARAMS_COMMAND_STOP, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }
        wifi_params_process_stop();
        return;
    }

    if (0 == strcmp(token_cmd, "list"))
    {
        token_extra = wifi_params_next_token(&cursor);
        if (NULL != token_extra)
        {
            if (0U != wifi_params_is_help_flag(token_extra))
            {
                token_more = wifi_params_next_token(&cursor);
                if (NULL == token_more)
                {
                    wifi_params_process_help("list");
                    return;
                }
            }

            wifi_params_reply_error(WIFI_PARAMS_COMMAND_LIST, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            return;
        }
        wifi_params_process_list();
        return;
    }

    wifi_params_reply_error(WIFI_PARAMS_COMMAND_NONE, WIFI_PARAMS_RESULT_ERR_UNKNOWN_CMD, 0U, 0.0f, "unknown command");
}

/*
 * 函数名: wifi_params_feed_byte
 * 功能: 向文本行解析器喂入单个字符
 * 输入参数:
 *   ch - 新接收到的字符
 * 返回值: 无
 */
static void wifi_params_feed_byte(char ch)
{
    if (0U != s_wifi_params_line_expect_lf)
    {
        if ('\n' == ch)
        {
            if ((0U != s_wifi_params_line_overflow) || (0U != s_wifi_params_line_invalid))
            {
                wifi_params_reply_error(WIFI_PARAMS_COMMAND_NONE, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
            }
            else if (s_wifi_params_line_len > 0U)
            {
                s_wifi_params_line[s_wifi_params_line_len] = '\0';
                wifi_params_process_line(s_wifi_params_line);
            }

            wifi_params_reset_line_state();
            return;
        }

        wifi_params_reply_error(WIFI_PARAMS_COMMAND_NONE, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
        wifi_params_reset_line_state();
        return;
    }

    if ('\n' == ch)
    {
        wifi_params_reply_error(WIFI_PARAMS_COMMAND_NONE, WIFI_PARAMS_RESULT_ERR_FORMAT, 0U, 0.0f, "format");
        wifi_params_reset_line_state();
        return;
    }

    if ('\r' == ch)
    {
        s_wifi_params_line_expect_lf = 1U;
        return;
    }

    if (((unsigned char)ch < 32U) || ((unsigned char)ch > 126U))
    {
        if ('\t' != ch)
        {
            s_wifi_params_line_invalid = 1U;
            return;
        }
    }

    if (0U != s_wifi_params_line_overflow)
    {
        return;
    }

    if (s_wifi_params_line_len >= (WIFI_PARAMS_LINE_MAX - 1U))
    {
        s_wifi_params_line_overflow = 1U;
        return;
    }

    s_wifi_params_line[s_wifi_params_line_len++] = ch;
}

/*
 * 函数名: wifi_params_Init
 * 功能: 初始化 WiFi 参数模块内部状态
 * 输入参数: 无
 * 返回值: 无
 */
void wifi_params_Init(void)
{
    memset(&s_wifi_params_diag, 0, sizeof(s_wifi_params_diag));
    memset(s_wifi_params_line, 0, sizeof(s_wifi_params_line));
    wifi_params_reset_line_state();
    s_wifi_params_pending_apply = 0U;
}

/*
 * 函数名: wifi_params_Poll
 * 功能: 轮询读取并解析 WiFi 参数命令，必要时在待机态刷新 PID 参数
 * 输入参数: 无
 * 返回值: 无
 */
void wifi_params_Poll(void)
{
    uint8_t rx_buffer[WIFI_PARAMS_RX_BUFFER_SIZE];
    uint32_t read_len;
    uint32_t i;

    read_len = wifi_spi_read_buffer(rx_buffer, (uint32_t)sizeof(rx_buffer));
    for (i = 0U; i < read_len; i++)
    {
        wifi_params_feed_byte((char)rx_buffer[i]);
    }

    if ((0U != s_wifi_params_pending_apply) && (0U != wifi_params_is_edit_allowed()))
    {
        FC_Loop_Init();
        s_wifi_params_pending_apply = 0U;
    }
}

/*
 * 函数名: wifi_params_GetDiag
 * 功能: 读取最近一次命令处理诊断状态
 * 输入参数:
 *   diag - 输出诊断结构体指针
 * 返回值: 无
 */
void wifi_params_GetDiag(wifi_params_diag_t *diag)
{
    if (NULL == diag)
    {
        return;
    }

    *diag = s_wifi_params_diag;
}
