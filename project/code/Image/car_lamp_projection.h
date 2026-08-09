#ifndef CAR_LAMP_PROJECTION_H_
#define CAR_LAMP_PROJECTION_H_

#include "Image/image_data.h"

#define CAR_LAMP_IMAGE_HALF_WIDTH   (94.0f) /* 188像素图像的横向半尺寸。 */
#define CAR_LAMP_IMAGE_HALF_HEIGHT  (60.0f) /* 120像素图像的纵向半尺寸。 */
#define CAR_LAMP_CENTER_HALF_WIDTH  (140.0f) /* 下摄公共标定坐标允许的横向半范围。 */
#define CAR_LAMP_CENTER_HALF_HEIGHT (110.0f) /* 下摄公共标定坐标允许的纵向半范围。 */

typedef struct
{
    float x;
    float y;
} car_lamp_projection_point_t;

/**
 * @brief 将指定摄像头的车灯中心映射到下摄公共坐标系。
 * @param camera 来源摄像头编号。
 * @param source 来源摄像头内的中心坐标，单位像素。
 * @param center 输出下摄公共坐标，单位像素。
 * @return 1表示输入输出均位于标定图像范围，0表示参数或坐标无效。
 */
uint8 CarLampProjection_ToCenter(
    image_camera_e camera,
    const car_lamp_projection_point_t *source,
    car_lamp_projection_point_t *center);

/**
 * @brief 将下摄公共坐标通过离线固定系数反向映射到指定摄像头。
 * @param camera 目标摄像头编号。
 * @param center 下摄公共坐标，单位像素。
 * @param source 输出目标摄像头内的预计坐标，单位像素。
 * @return 1表示输入输出均位于标定图像范围，0表示参数或坐标无效。
 */
uint8 CarLampProjection_FromCenter(
    image_camera_e camera,
    const car_lamp_projection_point_t *center,
    car_lamp_projection_point_t *source);

#endif /* CAR_LAMP_PROJECTION_H_ */
