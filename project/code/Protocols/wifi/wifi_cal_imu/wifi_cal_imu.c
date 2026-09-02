/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
/*****************************************************************************
 * 文件: wifi_cal_imu.c
 * 模块: WiFi IMU 校准命令转接
 * 职责: 负责 imu 命令解析、状态约束检查、中文帮助输出与 IMU 校准接口转接
 *****************************************************************************/

#include "wifi_cal_imu.h"

#include <string.h>

#include "Estimation/Attitude/Accel_Calibration.h"
#include "FlightController/fc_start_crsf.h"
#include "Protocols/wifi/wifi_cmd/wifi_cmd.h"

/*
 * 函数功能: 判断当前是否允许执行 IMU 校准写操作
 * 输入参数: 无
 * 返回值:
 *   1 - 允许
 *   0 - 不允许
 */
static uint8_t wifi_cal_imu_is_edit_allowed(void)
{
    if (FC_START_CRSF_STATE_STANDBY != FC_START_CRSF_Get_State())
    {
        return 0U;
    }

    return (0U == FC_START_CRSF_Is_Armed()) ? 1U : 0U;
}

/*
 * 函数功能: 将 IMU 校准过程文本转发到 WiFi 文本回包通道
 * 输入参数:
 *   text - 已格式化好的单行文本
 * 返回值: 无
 */
static void wifi_cal_imu_text_sink(const char *text)
{
    if ((NULL == text) || ('\0' == text[0]))
    {
        return;
    }

    (void)wifi_cmd_SendLine("%s", text);
}

/*
 * 函数功能: 统一发送 IMU 命令错误回包
 * 输入参数:
 *   reason - 错误原因关键字
 * 返回值: 无
 */
static void wifi_cal_imu_reply_error(const char *reason)
{
    (void)wifi_cmd_SendLine("ERR imu %s", (NULL != reason) ? reason : "format");
}

/*
 * 函数功能: 将内部模式码转成可读文本
 * 输入参数:
 *   mode - IMU 校准模式码
 * 返回值: 中文模式名
 */
static const char *wifi_cal_imu_mode_name(uint8_t mode)
{
    if (IMU_CALIB_STATUS_MODE_GYRO == mode)
    {
        return "角速度校准";
    }

    if (IMU_CALIB_STATUS_MODE_ELLIP == mode)
    {
        return "加速度自动校准";
    }

    if (IMU_CALIB_STATUS_MODE_ELLIP_MANUAL == mode)
    {
        return "加速度手动校准";
    }

    if ((IMU_CALIB_STATUS_MODE_ACC6 == mode) || (IMU_CALIB_STATUS_MODE_ALL == mode))
    {
        return "内部保留模式";
    }

    return "空闲";
}

/*
 * 函数功能: 将手动校准子状态码转成可读文本
 * 输入参数:
 *   substate - 手动校准子状态码
 * 返回值: 中文子状态名
 */
static const char *wifi_cal_imu_manual_substate_name(uint8_t substate)
{
    if (IMU_CALIB_MANUAL_SUBSTATE_READY == substate)
    {
        return "准备";
    }

    if (IMU_CALIB_MANUAL_SUBSTATE_WAIT_STATIC == substate)
    {
        return "等待静止";
    }

    if (IMU_CALIB_MANUAL_SUBSTATE_COLLECTING == substate)
    {
        return "采集中";
    }

    if (IMU_CALIB_MANUAL_SUBSTATE_SOLVING == substate)
    {
        return "求解中";
    }

    return "无";
}

/*
 * 函数功能: 输出 imu 总帮助
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_send_help_overview(void)
{
    (void)wifi_cmd_SendLine("主题: imu");
    (void)wifi_cmd_SendLine("用法1: imu help");
    (void)wifi_cmd_SendLine("用法2: imu status");
    (void)wifi_cmd_SendLine("用法3: imu start gyro");
    (void)wifi_cmd_SendLine("用法4: imu start accel");
    (void)wifi_cmd_SendLine("用法5: imu start accel_man");
    (void)wifi_cmd_SendLine("用法6: imu acc collect / imu acc stop");
    (void)wifi_cmd_SendLine("用法7: imu load / imu save / imu default / imu clear / imu flash");
    (void)wifi_cmd_SendLine("说明1: imu status 与 imu flash 任意状态都允许执行");
    (void)wifi_cmd_SendLine("说明2: 其他 imu 写命令仅待机且未解锁时允许执行");
    (void)wifi_cmd_SendLine("说明3: 手动校准请先执行 imu start accel_man，再按 imu acc collect / imu acc stop");
    (void)wifi_cmd_SendLine("示例1: imu help acc");
    (void)wifi_cmd_SendLine("示例2: imu start accel_man");
    (void)wifi_cmd_SendLine("说明4: accel 与 accel_man 均基于未校准原始加速度采样");
    (void)wifi_cmd_SendLine("OK imu help");
}

/*
 * 函数功能: 输出 status 子命令帮助
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_send_help_status(void)
{
    (void)wifi_cmd_SendLine("主题: imu status");
    (void)wifi_cmd_SendLine("用法: imu status");
    (void)wifi_cmd_SendLine("功能: 查询当前 IMU 校准忙闲、模式、进度、姿态点与手动子状态");
    (void)wifi_cmd_SendLine("限制: 任意状态都允许执行");
    (void)wifi_cmd_SendLine("示例: imu status");
    (void)wifi_cmd_SendLine("成功回包示例: OK imu status 状态=空闲 模式=空闲 进度=0%% 姿态点=0");
    (void)wifi_cmd_SendLine("OK imu help status");
}

/*
 * 函数功能: 输出 load 子命令帮助
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_send_help_load(void)
{
    (void)wifi_cmd_SendLine("主题: imu load");
    (void)wifi_cmd_SendLine("用法: imu load");
    (void)wifi_cmd_SendLine("功能: 从 Flash 读取 IMU 校准参数并立即应用");
    (void)wifi_cmd_SendLine("限制: 仅待机且未解锁时允许执行");
    (void)wifi_cmd_SendLine("说明: 校准进行中会返回 ERR imu busy");
    (void)wifi_cmd_SendLine("成功回包: OK imu load");
    (void)wifi_cmd_SendLine("OK imu help load");
}

/*
 * 函数功能: 输出 save 子命令帮助
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_send_help_save(void)
{
    (void)wifi_cmd_SendLine("主题: imu save");
    (void)wifi_cmd_SendLine("用法: imu save");
    (void)wifi_cmd_SendLine("功能: 将当前 IMU 校准参数保存到 Flash");
    (void)wifi_cmd_SendLine("限制: 仅待机且未解锁时允许执行");
    (void)wifi_cmd_SendLine("说明: 校准进行中会返回 ERR imu busy");
    (void)wifi_cmd_SendLine("成功回包: OK imu save");
    (void)wifi_cmd_SendLine("OK imu help save");
}

/*
 * 函数功能: 输出 default 子命令帮助
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_send_help_default(void)
{
    (void)wifi_cmd_SendLine("主题: imu default");
    (void)wifi_cmd_SendLine("用法: imu default");
    (void)wifi_cmd_SendLine("功能: 将代码内置默认加速度校准参数立即写入 Flash");
    (void)wifi_cmd_SendLine("限制: 仅待机且未解锁时允许执行");
    (void)wifi_cmd_SendLine("说明: 用于强制覆盖 Flash 中已有的旧加速度校准参数");
    (void)wifi_cmd_SendLine("成功回包: OK imu default");
    (void)wifi_cmd_SendLine("OK imu help default");
}

/*
 * 函数功能: 输出 clear 子命令帮助
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_send_help_clear(void)
{
    (void)wifi_cmd_SendLine("主题: imu clear");
    (void)wifi_cmd_SendLine("用法: imu clear");
    (void)wifi_cmd_SendLine("功能: 擦除 Flash 中保存的 IMU 校准参数");
    (void)wifi_cmd_SendLine("限制: 仅待机且未解锁时允许执行");
    (void)wifi_cmd_SendLine("说明: 校准进行中会返回 ERR imu busy");
    (void)wifi_cmd_SendLine("成功回包: OK imu clear");
    (void)wifi_cmd_SendLine("OK imu help clear");
}

/*
 * 函数功能: 输出 start 子命令帮助
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_send_help_start(void)
{
    (void)wifi_cmd_SendLine("主题: imu start");
    (void)wifi_cmd_SendLine("用法1: imu start gyro");
    (void)wifi_cmd_SendLine("用法2: imu start accel");
    (void)wifi_cmd_SendLine("用法3: imu start accel_man");
    (void)wifi_cmd_SendLine("功能: 启动角速度校准、加速度自动校准或加速度手动校准");
    (void)wifi_cmd_SendLine("限制: 仅待机且未解锁时允许执行");
    (void)wifi_cmd_SendLine("说明1: gyro 要求飞机全程静止");
    (void)wifi_cmd_SendLine("说明2: accel 为自动椭球校准，自动检测静止并自动收点");
    (void)wifi_cmd_SendLine("说明3: accel_man 为手动椭球校准，进入准备态后请发 imu acc collect / imu acc stop");
    (void)wifi_cmd_SendLine("示例1: imu start accel");
    (void)wifi_cmd_SendLine("示例2: imu start accel_man");
    (void)wifi_cmd_SendLine("OK imu help start");
}

/*
 * 函数功能: 输出 acc 子命令帮助
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_send_help_acc(void)
{
    (void)wifi_cmd_SendLine("主题: imu acc");
    (void)wifi_cmd_SendLine("用法1: imu acc collect");
    (void)wifi_cmd_SendLine("用法2: imu acc stop");
    (void)wifi_cmd_SendLine("功能: 手动加速度椭球校准的采点与停止求解命令");
    (void)wifi_cmd_SendLine("限制1: 仅在 imu start accel_man 后可用");
    (void)wifi_cmd_SendLine("限制2: 仅待机且未解锁时允许执行");
    (void)wifi_cmd_SendLine("说明1: imu acc collect 会先检测静止，再采集 5000 个样本形成一个姿态点");
    (void)wifi_cmd_SendLine("说明2: imu acc stop 会结束手动会话，并对已完成姿态点做椭球拟合");
    (void)wifi_cmd_SendLine("说明3: 若 stop 时当前点未采满，该未完成点会被丢弃");
    (void)wifi_cmd_SendLine("示例1: imu acc collect");
    (void)wifi_cmd_SendLine("示例2: imu acc stop");
    (void)wifi_cmd_SendLine("说明4: accel_man 采样源为未校准原始加速度，不走旧矩阵和控制链滤波");
    (void)wifi_cmd_SendLine("OK imu help acc");
}

/*
 * 函数功能: 输出 flash 子命令帮助
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_send_help_flash(void)
{
    (void)wifi_cmd_SendLine("主题: imu flash");
    (void)wifi_cmd_SendLine("用法: imu flash");
    (void)wifi_cmd_SendLine("功能: 读取 Flash 中保存的 IMU 校准参数并按人类可读形式输出");
    (void)wifi_cmd_SendLine("限制: 任意状态都允许执行");
    (void)wifi_cmd_SendLine("说明1: 无有效数据时返回 OK imu flash empty");
    (void)wifi_cmd_SendLine("说明2: 有效数据时会输出陀螺仪零偏、加速度偏置、校正矩阵和安装矩阵");
    (void)wifi_cmd_SendLine("示例: imu flash");
    (void)wifi_cmd_SendLine("成功回包: OK imu flash begin version=2 full_matrix=1");
    (void)wifi_cmd_SendLine("OK imu help flash");
}

/*
 * 函数功能: 按帮助主题输出详细帮助
 * 输入参数:
 *   topic - 帮助主题
 * 返回值:
 *   1 - 已处理
 *   0 - 未知主题
 */
static uint8_t wifi_cal_imu_process_help_topic(const char *topic)
{
    if ((NULL == topic) || ('\0' == topic[0]))
    {
        return 0U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "status"))
    {
        wifi_cal_imu_send_help_status();
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "load"))
    {
        wifi_cal_imu_send_help_load();
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "save"))
    {
        wifi_cal_imu_send_help_save();
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "default"))
    {
        wifi_cal_imu_send_help_default();
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "clear"))
    {
        wifi_cal_imu_send_help_clear();
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "start"))
    {
        wifi_cal_imu_send_help_start();
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "acc"))
    {
        wifi_cal_imu_send_help_acc();
        return 1U;
    }

    if (0 == wifi_cmd_ascii_stricmp(topic, "flash"))
    {
        wifi_cal_imu_send_help_flash();
        return 1U;
    }

    return 0U;
}

/*
 * 函数功能: 处理 imu help 或 <cmd> --help
 * 输入参数:
 *   topic - 为空时输出总帮助，不为空时输出对应子主题帮助
 * 返回值: 无
 */
static void wifi_cal_imu_process_help(const char *topic)
{
    if ((NULL == topic) || ('\0' == topic[0]))
    {
        wifi_cal_imu_send_help_overview();
        return;
    }

    if (0U != wifi_cal_imu_process_help_topic(topic))
    {
        return;
    }

    wifi_cal_imu_reply_error("unknown");
}

/*
 * 函数功能: 输出当前 IMU 校准状态
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_process_status(void)
{
    IMUCalibStatus_t status = {0};

    IMUCalib_GetStatus(&status);
    if (status.mode == IMU_CALIB_STATUS_MODE_ELLIP_MANUAL)
    {
        (void)wifi_cmd_SendLine("OK imu status 状态=%s 模式=%s 子状态=%s 进度=%lu%% 姿态点=%u 当前样本=%lu/%lu",
                                (0U != status.busy) ? "忙" : "空闲",
                                wifi_cal_imu_mode_name(status.mode),
                                wifi_cal_imu_manual_substate_name(status.substate),
                                (unsigned long)status.progress_percent,
                                (unsigned int)status.pose_count,
                                (unsigned long)status.current_samples,
                                (unsigned long)status.target_samples);
        return;
    }

    (void)wifi_cmd_SendLine("OK imu status 状态=%s 模式=%s 进度=%lu%% 姿态点=%u",
                            (0U != status.busy) ? "忙" : "空闲",
                            wifi_cal_imu_mode_name(status.mode),
                            (unsigned long)status.progress_percent,
                            (unsigned int)status.pose_count);
}

/*
 * 函数功能: 处理 imu load 命令
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_process_load(void)
{
    if (0U == wifi_cal_imu_is_edit_allowed())
    {
        wifi_cal_imu_reply_error("state");
        return;
    }

    if (0U != IMUCalib_IsBusy())
    {
        wifi_cal_imu_reply_error("busy");
        return;
    }

    if (0U == IMUCalib_LoadFromFlashAndApply())
    {
        wifi_cal_imu_reply_error("flash");
        return;
    }

    (void)wifi_cmd_SendLine("OK imu load");
}

/*
 * 函数功能: 处理 imu save 命令
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_process_save(void)
{
    if (0U == wifi_cal_imu_is_edit_allowed())
    {
        wifi_cal_imu_reply_error("state");
        return;
    }

    if (0U != IMUCalib_IsBusy())
    {
        wifi_cal_imu_reply_error("busy");
        return;
    }

    if (0U == IMUCalib_SaveCurrentToFlash())
    {
        wifi_cal_imu_reply_error("flash");
        return;
    }

    (void)wifi_cmd_SendLine("OK imu save");
}

/*
 * 函数功能: 处理 imu default 命令
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_process_default(void)
{
    if (0U == wifi_cal_imu_is_edit_allowed())
    {
        wifi_cal_imu_reply_error("state");
        return;
    }

    if (0U != IMUCalib_IsBusy())
    {
        wifi_cal_imu_reply_error("busy");
        return;
    }

    if (0U == IMUCalib_ApplyDefaultToFlash())
    {
        wifi_cal_imu_reply_error("flash");
        return;
    }

    (void)wifi_cmd_SendLine("OK imu default");
}

/*
 * 函数功能: 处理 imu clear 命令
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_process_clear(void)
{
    if (0U == wifi_cal_imu_is_edit_allowed())
    {
        wifi_cal_imu_reply_error("state");
        return;
    }

    if (0U != IMUCalib_IsBusy())
    {
        wifi_cal_imu_reply_error("busy");
        return;
    }

    if (0U == IMUCalib_ClearFlash())
    {
        wifi_cal_imu_reply_error("flash");
        return;
    }

    (void)wifi_cmd_SendLine("OK imu clear");
}

/*
 * 函数功能: 处理 imu flash 命令
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_process_flash(void)
{
    IMUCalibFlashInfo_t info = {0};

    if (0U == IMUCalib_ReadFlashInfo(&info))
    {
        wifi_cal_imu_reply_error("flash");
        return;
    }

    if (0U == info.valid)
    {
        (void)wifi_cmd_SendLine("OK imu flash empty");
        return;
    }

    (void)wifi_cmd_SendLine("OK imu flash begin version=%u full_matrix=%u",
                            (unsigned int)info.version,
                            (unsigned int)info.use_full_matrix);
    (void)wifi_cmd_SendLine("OK imu flash gyro_bias_dps x=%f y=%f z=%f",
                            (double)info.gyro_bias_dps[0],
                            (double)info.gyro_bias_dps[1],
                            (double)info.gyro_bias_dps[2]);
    (void)wifi_cmd_SendLine("OK imu flash accel_bias_g x=%f y=%f z=%f",
                            (double)info.accel_bias_g[0],
                            (double)info.accel_bias_g[1],
                            (double)info.accel_bias_g[2]);
    (void)wifi_cmd_SendLine("OK imu flash accel_matrix r0=%f,%f,%f",
                            (double)info.accel_corr_matrix[0][0],
                            (double)info.accel_corr_matrix[0][1],
                            (double)info.accel_corr_matrix[0][2]);
    (void)wifi_cmd_SendLine("OK imu flash accel_matrix r1=%f,%f,%f",
                            (double)info.accel_corr_matrix[1][0],
                            (double)info.accel_corr_matrix[1][1],
                            (double)info.accel_corr_matrix[1][2]);
    (void)wifi_cmd_SendLine("OK imu flash accel_matrix r2=%f,%f,%f",
                            (double)info.accel_corr_matrix[2][0],
                            (double)info.accel_corr_matrix[2][1],
                            (double)info.accel_corr_matrix[2][2]);
    (void)wifi_cmd_SendLine("OK imu flash imu_to_body r0=%f,%f,%f",
                            (double)info.imu_to_body[0][0],
                            (double)info.imu_to_body[0][1],
                            (double)info.imu_to_body[0][2]);
    (void)wifi_cmd_SendLine("OK imu flash imu_to_body r1=%f,%f,%f",
                            (double)info.imu_to_body[1][0],
                            (double)info.imu_to_body[1][1],
                            (double)info.imu_to_body[1][2]);
    (void)wifi_cmd_SendLine("OK imu flash imu_to_body r2=%f,%f,%f",
                            (double)info.imu_to_body[2][0],
                            (double)info.imu_to_body[2][1],
                            (double)info.imu_to_body[2][2]);
    (void)wifi_cmd_SendLine("OK imu flash end");
}

/*
 * 函数功能: 按模式启动 IMU 校准
 * 输入参数:
 *   mode - 校准模式字符串
 * 返回值: 无
 */
static void wifi_cal_imu_process_start(const char *mode)
{
    uint8_t ok = 0U;

    if (0U == wifi_cal_imu_is_edit_allowed())
    {
        wifi_cal_imu_reply_error("state");
        return;
    }

    if (0U != IMUCalib_IsBusy())
    {
        wifi_cal_imu_reply_error("busy");
        return;
    }

    if (0 == wifi_cmd_ascii_stricmp(mode, "gyro"))
    {
        ok = IMUCalib_StartGyro();
    }
    else if (0 == wifi_cmd_ascii_stricmp(mode, "accel"))
    {
        ok = IMUCalib_StartAccel();
    }
    else if (0 == wifi_cmd_ascii_stricmp(mode, "accel_man"))
    {
        ok = IMUCalib_StartAccelManual();
    }
    else
    {
        wifi_cal_imu_reply_error("unknown");
        return;
    }

    if (0U == ok)
    {
        wifi_cal_imu_reply_error("busy");
    }
}

/*
 * 函数功能: 处理手动加速度采点命令
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_process_acc_collect(void)
{
    IMUCalibStatus_t status = {0};

    if (0U == wifi_cal_imu_is_edit_allowed())
    {
        wifi_cal_imu_reply_error("state");
        return;
    }

    IMUCalib_GetStatus(&status);
    if ((status.busy == 0U) || (status.mode != IMU_CALIB_STATUS_MODE_ELLIP_MANUAL))
    {
        wifi_cal_imu_reply_error("state");
        return;
    }

    if (status.pose_count >= 128U)
    {
        wifi_cal_imu_reply_error("limit");
        return;
    }

    if (status.substate != IMU_CALIB_MANUAL_SUBSTATE_READY)
    {
        wifi_cal_imu_reply_error("busy");
        return;
    }

    if (0U == IMUCalib_ManualCollect())
    {
        wifi_cal_imu_reply_error("busy");
    }
}

/*
 * 函数功能: 处理手动加速度停止求解命令
 * 输入参数: 无
 * 返回值: 无
 */
static void wifi_cal_imu_process_acc_stop(void)
{
    IMUCalibStatus_t status = {0};

    if (0U == wifi_cal_imu_is_edit_allowed())
    {
        wifi_cal_imu_reply_error("state");
        return;
    }

    IMUCalib_GetStatus(&status);
    if ((status.busy == 0U) || (status.mode != IMU_CALIB_STATUS_MODE_ELLIP_MANUAL))
    {
        wifi_cal_imu_reply_error("state");
        return;
    }

    if (status.substate == IMU_CALIB_MANUAL_SUBSTATE_SOLVING)
    {
        wifi_cal_imu_reply_error("busy");
        return;
    }

    if (0U == IMUCalib_ManualStop())
    {
        wifi_cal_imu_reply_error("busy");
    }
}

/*
 * 函数功能: 初始化 WiFi IMU 校准模块
 * 输入参数: 无
 * 返回值: 无
 */
void wifi_cal_imu_Init(void)
{
    IMUCalib_SetTextSink(wifi_cal_imu_text_sink);
}

/*
 * 函数功能: 处理一条 imu 文本命令
 * 输入参数:
 *   line - 完整命令文本，函数内部允许原地切分
 * 返回值: 无
 */
void wifi_cal_imu_ProcessLine(char *line)
{
    char *trimmed_line;
    char *cursor;
    char *token_root;
    char *token_cmd;
    char *token_arg1;
    char *token_arg2;
    char *token_arg3;

    if (NULL == line)
    {
        wifi_cal_imu_reply_error("format");
        return;
    }

    trimmed_line = wifi_cmd_trim_line(line);
    if ((NULL == trimmed_line) || ('\0' == trimmed_line[0]))
    {
        wifi_cal_imu_reply_error("format");
        return;
    }

    cursor = trimmed_line;
    token_root = wifi_cmd_next_token(&cursor);
    token_cmd = wifi_cmd_next_token(&cursor);
    token_arg1 = wifi_cmd_next_token(&cursor);
    token_arg2 = wifi_cmd_next_token(&cursor);
    token_arg3 = wifi_cmd_next_token(&cursor);

    if ((NULL == token_root) || (NULL != token_arg3))
    {
        wifi_cal_imu_reply_error("format");
        return;
    }

    wifi_cmd_ascii_strtolower(token_root);
    if (0 != strcmp(token_root, "imu"))
    {
        wifi_cal_imu_reply_error("unknown");
        return;
    }

    if (NULL == token_cmd)
    {
        wifi_cal_imu_process_help(NULL);
        return;
    }

    wifi_cmd_ascii_strtolower(token_cmd);
    if (NULL != token_arg1)
    {
        wifi_cmd_ascii_strtolower(token_arg1);
    }
    if (NULL != token_arg2)
    {
        wifi_cmd_ascii_strtolower(token_arg2);
    }

    if ((0 == strcmp(token_cmd, "help")) || (0U != wifi_cmd_is_help_flag(token_cmd)))
    {
        if (NULL == token_arg2)
        {
            wifi_cal_imu_process_help(token_arg1);
            return;
        }

        wifi_cal_imu_reply_error("format");
        return;
    }

    if (0 == strcmp(token_cmd, "status"))
    {
        if (NULL == token_arg1)
        {
            wifi_cal_imu_process_status();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_cal_imu_send_help_status();
            return;
        }

        wifi_cal_imu_reply_error("format");
        return;
    }

    if (0 == strcmp(token_cmd, "load"))
    {
        if (NULL == token_arg1)
        {
            wifi_cal_imu_process_load();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_cal_imu_send_help_load();
            return;
        }

        wifi_cal_imu_reply_error("format");
        return;
    }

    if (0 == strcmp(token_cmd, "save"))
    {
        if (NULL == token_arg1)
        {
            wifi_cal_imu_process_save();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_cal_imu_send_help_save();
            return;
        }

        wifi_cal_imu_reply_error("format");
        return;
    }

    if (0 == strcmp(token_cmd, "default"))
    {
        if (NULL == token_arg1)
        {
            wifi_cal_imu_process_default();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_cal_imu_send_help_default();
            return;
        }

        wifi_cal_imu_reply_error("format");
        return;
    }

    if (0 == strcmp(token_cmd, "clear"))
    {
        if (NULL == token_arg1)
        {
            wifi_cal_imu_process_clear();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_cal_imu_send_help_clear();
            return;
        }

        wifi_cal_imu_reply_error("format");
        return;
    }

    if (0 == strcmp(token_cmd, "flash"))
    {
        if (NULL == token_arg1)
        {
            wifi_cal_imu_process_flash();
            return;
        }

        if ((0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_cal_imu_send_help_flash();
            return;
        }

        wifi_cal_imu_reply_error("format");
        return;
    }

    if (0 == strcmp(token_cmd, "start"))
    {
        if ((NULL != token_arg1) && (0U == wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_cal_imu_process_start(token_arg1);
            return;
        }

        if ((NULL != token_arg1) && (0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2))
        {
            wifi_cal_imu_send_help_start();
            return;
        }

        wifi_cal_imu_reply_error("format");
        return;
    }

    if (0 == strcmp(token_cmd, "acc"))
    {
        if ((NULL != token_arg1) && (0 == strcmp(token_arg1, "collect")) && (NULL == token_arg2))
        {
            wifi_cal_imu_process_acc_collect();
            return;
        }

        if ((NULL != token_arg1) && (0 == strcmp(token_arg1, "stop")) && (NULL == token_arg2))
        {
            wifi_cal_imu_process_acc_stop();
            return;
        }

        if (((NULL != token_arg1) && (0U != wifi_cmd_is_help_flag(token_arg1)) && (NULL == token_arg2)) ||
            ((NULL == token_arg1) && (NULL == token_arg2)))
        {
            wifi_cal_imu_send_help_acc();
            return;
        }

        wifi_cal_imu_reply_error("format");
        return;
    }

    wifi_cal_imu_reply_error("unknown");
}
