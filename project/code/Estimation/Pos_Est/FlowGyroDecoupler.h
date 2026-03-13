#ifndef FLOW_GYRO_DECOUPLER_H
#define FLOW_GYRO_DECOUPLER_H

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 参数宏定义 ===================== */
#define FLOW_RING_SIZE        (64U)
#define FLOW_TAU_MS           (13U)   /* PMW3901 pipeline延迟补偿，单位ms */
#define FLOW_KX               (8.50f) /* counts/deg */
#define FLOW_KY               (8.46f)
#define FLOW_BX               (0.0f)  /* 每窗口bias */
#define FLOW_BY               (0.0f)
#define FLOW_MAX_WINDOW_MS    (30U)   /* 超过此窗口视为异常 */

/* ===================== 接口函数 ===================== */
void  FlowGyroDecoupler_Init(void);
void  FlowGyroDecoupler_Reinit(void);
void  FlowGyroDecoupler_Push1000Hz(uint32 t_ms, float gyro_x, float gyro_y);
uint8 FlowGyroDecoupler_Update50Hz(uint32 t_read_ms, int16_t delta_x, int16_t delta_y);
float FlowGyroDecoupler_GetDecX(void);
float FlowGyroDecoupler_GetDecY(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_GYRO_DECOUPLER_H */
