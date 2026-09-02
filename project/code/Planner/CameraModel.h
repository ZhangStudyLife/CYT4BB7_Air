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
#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

void CameraModel_MapPoint(float x, float y,
                          float roll, float pitch,
                          float *out_x, float *out_y);

void CameraModel_MapVector(float x, float y,
                           float vx, float vy,
                           float roll, float pitch,
                           float *out_vx, float *out_vy);

#endif /* CAMERA_MODEL_H */
