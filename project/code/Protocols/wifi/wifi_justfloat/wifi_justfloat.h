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
