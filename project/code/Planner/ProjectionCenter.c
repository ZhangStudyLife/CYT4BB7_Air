#include "ProjectionCenter.h"
#include "../Estimation/Attitude/IMU_TOP.h"

#define PROJECTION_CENTER_X_BIAS      (0.0f)
#define PROJECTION_CENTER_X_ROLL_K    (1.4f)
#define PROJECTION_CENTER_X_PITCH_K   (0.015f)
#define PROJECTION_CENTER_Y_BIAS      (-25.0f)
#define PROJECTION_CENTER_Y_ROLL_K    (-0.015f)
#define PROJECTION_CENTER_Y_PITCH_K   (1.3f)

projection_center_result_t g_projection_center;

void ProjectionCenter_Init(void)
{
    g_projection_center.valid = 0U;
    g_projection_center.cx = 0.0f;
    g_projection_center.cy = 0.0f;
}

uint8 ProjectionCenter_Update(void)
{
    g_projection_center.cx = PROJECTION_CENTER_X_BIAS +
                             PROJECTION_CENTER_X_ROLL_K * g_euler.roll +
                             PROJECTION_CENTER_X_PITCH_K * g_euler.pitch;
    g_projection_center.cy = PROJECTION_CENTER_Y_BIAS +
                             PROJECTION_CENTER_Y_ROLL_K * g_euler.roll +
                             PROJECTION_CENTER_Y_PITCH_K * g_euler.pitch;
    g_projection_center.valid = 1U;

    return 1U;
}
