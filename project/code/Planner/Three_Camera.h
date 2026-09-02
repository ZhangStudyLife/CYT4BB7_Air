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
#ifndef THREE_CAMERA_H
#define THREE_CAMERA_H

#include "../Image/image_data.h"

#define THREE_CAMERA_MAX_BEACON_COUNT   (IMAGE_MAX_BEACON_COUNT) /* 三摄融合后的物理信标最大数量。 */

/*
 * 输出坐标系说明:
 * 1. 原点位于飞机的相机/机体参考点，不是场地固定原点。
 * 2. X、Y 轴方向与经过飞机欧拉角解耦后的水平全局坐标系对齐，不随飞机机头方向旋转。
 * 3. x_m、y_m 表示目标相对飞机原点的水平位移，单位 m。
 * 4. 输出未叠加飞机的绝对位置，因此不是场地中的绝对全局坐标。
 * 5. 如需绝对全局坐标，应在外部计算:
 *      target_global_x = aircraft_global_x + x_m;
 *      target_global_y = aircraft_global_y + y_m;
 */
typedef struct
{
    uint8 valid;
    uint8 camera_mask;
    uint8 pair_valid;       /* 车灯到该信标的优选相对向量有效标志。 */
    uint8 pair_same_camera; /* 优选相对向量是否来自同一摄像头。 */
    float x_m; /* 信标相对飞机原点、沿水平全局 X 轴方向的位移，单位 m。 */
    float y_m; /* 信标相对飞机原点、沿水平全局 Y 轴方向的位移，单位 m。 */
    float area;
    float pair_dx_m;        /* 优选信标坐标减车灯坐标的全局 X 分量，单位 m。 */
    float pair_dy_m;        /* 优选信标坐标减车灯坐标的全局 Y 分量，单位 m。 */
    float pair_lamp_angle_deg; /* 优选车灯长轴的水平全局无向角度，单位 deg。 */
} three_camera_beacon_t;

typedef struct
{
    uint8 valid;
    uint8 camera_mask;
    float x_m;      /* 车灯相对飞机原点、沿水平全局 X 轴方向的位移，单位 m。 */
    float y_m;      /* 车灯相对飞机原点、沿水平全局 Y 轴方向的位移，单位 m。 */
    float angle_deg; /* 车灯长轴在水平全局坐标系中的无向角度，单位 deg。 */
} three_camera_lamp_t;

typedef struct
{
    three_camera_lamp_t car_lamp;
    three_camera_beacon_t beacon[THREE_CAMERA_MAX_BEACON_COUNT];
    uint8 beacon_count;
} three_camera_result_t;

/*
 * 函数功能: 使用三台相机独立 Double Sphere 模型，将当前图像检测结果投影到以飞机参考点为原点、
 *           轴向与水平全局坐标系对齐的局部坐标系，并完成跨摄融合和同摄优先配对。
 * 输入参数:
 *   image - 三路原始图像检测结果；x/y 为相机算法中心化像素坐标。
 *   roll_deg/pitch_deg/yaw_deg - 飞机欧拉角，单位 deg。
 *   height_mm - 飞机到地面的垂直高度，单位 mm。
 *   height_valid - 高度有效标志，非零时允许地面求交。
 *   result - 输出融合车灯、物理信标及优选车灯-信标相对向量，单位 m；不得为空。
 *            该结果已使用欧拉角解耦飞机姿态，但未叠加飞机绝对位置；存在同摄组合时优先使用同摄相对向量。
 * 输出参数/返回值:
 *   返回 1 表示完成有效高度下的投影，返回 0 表示输入高度无效或输出指针为空。
 */
uint8 Three_Camera_Update(const struct image_data image[IMAGE_CAMERA_COUNT],
                          float roll_deg,
                          float pitch_deg,
                          float yaw_deg,
                          float height_mm,
                          uint8 height_valid,
                          three_camera_result_t *result);

#endif /* THREE_CAMERA_H */
