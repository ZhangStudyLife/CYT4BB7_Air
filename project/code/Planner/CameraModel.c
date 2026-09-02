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
#include "CameraModel.h"

#define CAMERA_MODEL_RADIAL_K4                   (2.40f)
#define CAMERA_MODEL_CENTER_SCALE                (0.3333333333f)
#define CAMERA_MODEL_CENTER_X_BIAS               (-0.20f)
#define CAMERA_MODEL_CENTER_X_ROLL_K             (1.325f)
#define CAMERA_MODEL_CENTER_Y_BIAS               (-4.63f)
#define CAMERA_MODEL_CENTER_Y_PITCH_K            (1.334f)

void CameraModel_MapPoint(float x, float y,
                          float roll, float pitch,
                          float *out_x, float *out_y)
{
    float radius;
    float gain;

    x -= CAMERA_MODEL_CENTER_SCALE *
         (CAMERA_MODEL_CENTER_X_BIAS + CAMERA_MODEL_CENTER_X_ROLL_K * roll);
    y -= CAMERA_MODEL_CENTER_SCALE *
         (CAMERA_MODEL_CENTER_Y_BIAS + CAMERA_MODEL_CENTER_Y_PITCH_K * pitch);
    radius = (x * x + y * y) * 0.0001f;
    gain = 1.0f + CAMERA_MODEL_RADIAL_K4 * radius * radius;
    *out_x = x * gain;
    *out_y = y * gain;
}

void CameraModel_MapVector(float x, float y,
                           float vx, float vy,
                           float roll, float pitch,
                           float *out_vx, float *out_vy)
{
    float radius;
    float gain;
    float radial;

    x -= CAMERA_MODEL_CENTER_SCALE *
         (CAMERA_MODEL_CENTER_X_BIAS + CAMERA_MODEL_CENTER_X_ROLL_K * roll);
    y -= CAMERA_MODEL_CENTER_SCALE *
         (CAMERA_MODEL_CENTER_Y_BIAS + CAMERA_MODEL_CENTER_Y_PITCH_K * pitch);
    radius = (x * x + y * y) * 0.0001f;
    gain = 1.0f + CAMERA_MODEL_RADIAL_K4 * radius * radius;
    radial = 4.0f * CAMERA_MODEL_RADIAL_K4 * radius *
             (x * vx + y * vy) * 0.0001f;
    *out_vx = gain * vx + radial * x;
    *out_vy = gain * vy + radial * y;
}
