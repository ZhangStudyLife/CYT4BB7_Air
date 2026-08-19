#ifndef AUTO_LANDING_H
#define AUTO_LANDING_H

#include "zf_common_headfile.h"

/* 以100Hz更新Mode4自动降落检测。 */
void AutoLanding_Update100Hz(void);

/* 返回自动降落是否已经触发并锁存。 */
uint8 AutoLanding_IsTriggered(void);

#endif /* AUTO_LANDING_H */
