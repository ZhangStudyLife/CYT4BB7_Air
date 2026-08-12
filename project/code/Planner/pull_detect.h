#ifndef PULL_DETECT_H
#define PULL_DETECT_H

#include "zf_common_typedef.h"

typedef struct
{
    uint8 danger;
} pull_detect_result_t;

extern pull_detect_result_t g_pull_detect_result;

void PullDetect_Init(void);

/* 固定50 Hz调用，danger为1表示车辆与飞机的相对距离存在快速增大风险。 */
void PullDetect_Update100Hz(uint8 image_valid, float x_cm, float y_cm);

#endif /* PULL_DETECT_H */
