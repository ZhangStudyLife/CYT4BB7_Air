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
 * File: wifi_justfloat.h
 * Module: WiFi JustFloat telemetry
 * Purpose: output VOFA JustFloat binary frames through wifi_cmd
 *****************************************************************************/

#ifndef WIFI_JUSTFLOAT_H
#define WIFI_JUSTFLOAT_H

#include "zf_common_headfile.h"

#define WIFI_JUSTFLOAT_MAX_FLOAT_NUM       (101U)  /* Auto timestamp + up to 100 user channels */

void wifi_justfloat_Init(void);
uint8_t wifi_justfloat_IsReady(void);
void wifi_justfloat_SetStandbyContext(uint8_t is_standby);
void wifi_justfloat_SetStandbyUserEnable(uint8_t enable);
uint8_t wifi_justfloat_GetStandbyUserEnable(void);
uint8_t wifi_justfloat_Poll(void);
uint8_t wifi_justfloat_Array(const float *data, uint8_t num);

/* Each wifi_justfloat call costs about 10 us. */
#define wifi_justfloat(...) \
    wifi_justfloat_Array((const float[]){__VA_ARGS__}, \
                         (uint8_t)(sizeof((const float[]){__VA_ARGS__}) / sizeof(float)))

#endif /* WIFI_JUSTFLOAT_H */
