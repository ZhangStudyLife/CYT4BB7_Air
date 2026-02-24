#ifndef BARO_DATA_H_
#define BARO_DATA_H_

#include "../../HW_Drivers/BMP388/BMP388.h"


#ifdef __cplusplus
extern "C" {
#endif

#define BARO_UPDATE_DT_SEC            (1.0f / 50.0f) /* 50Hz 更新频率 */
#define BARO_CALIBRATION_SAMPLES          (200U)         /* 校准时采样数量 */
#define BARO_TO_HEIGHT_SCALE_FACTOR          (8.3f)          /* 气压每变化 1 Pa，高度约变 8.3 CM */

extern float g_baro_ref_pressure; /* 气压计上电的气压参考 */
extern float g_baro_altitude;     /* 当前气压计高度 */

void Baro_Init(void);
void Baro_Update(void);
void Baro_Calibrate(void);


#ifdef __cplusplus
}
#endif

#endif /* BARO_DATA_H_ */
