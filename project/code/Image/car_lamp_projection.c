#include "car_lamp_projection.h"

#define CAR_LAMP_PROJECTION_COEFFICIENT_COUNT (6U) /* 二次投影多项式的固定系数数量。 */

/* 前摄坐标正向映射到下摄公共坐标的离线标定系数。 */
static const float s_front_to_center_x[CAR_LAMP_PROJECTION_COEFFICIENT_COUNT] =
{
    -3.224193f, 1.123975f, 0.003353f,
    0.000073f, -0.004078f, -0.000302f
};
/* 前摄纵坐标正向映射到下摄公共坐标的离线标定系数。 */
static const float s_front_to_center_y[CAR_LAMP_PROJECTION_COEFFICIENT_COUNT] =
{
    -60.512112f, 0.030475f, 0.772429f,
    0.004336f, -0.000232f, 0.004678f
};
/* 后摄坐标正向映射到下摄公共坐标的离线标定系数。 */
static const float s_back_to_center_x[CAR_LAMP_PROJECTION_COEFFICIENT_COUNT] =
{
    -10.828701f, -1.119896f, 0.059751f,
    -0.000063f, 0.004186f, -0.000850f
};
/* 后摄纵坐标正向映射到下摄公共坐标的离线标定系数。 */
static const float s_back_to_center_y[CAR_LAMP_PROJECTION_COEFFICIENT_COUNT] =
{
    58.428997f, -0.026951f, -0.718077f,
    -0.004166f, 0.000106f, -0.004593f
};
/* 下摄公共坐标反向映射到前摄横坐标的离线拟合系数。 */
static const float s_center_to_front_x[CAR_LAMP_PROJECTION_COEFFICIENT_COUNT] =
{
    4.598873713f, 1.112662896f, 0.03961500727f,
    -0.0009593892413f, 0.003638791921f, 0.0001953756646f
};
/* 下摄公共坐标反向映射到前摄纵坐标的离线拟合系数。 */
static const float s_center_to_front_y[CAR_LAMP_PROJECTION_COEFFICIENT_COUNT] =
{
    57.76555674f, -0.04623574104f, 0.7418767472f,
    -0.003921243931f, 0.00009845932112f, -0.003075629357f
};
/* 下摄公共坐标反向映射到后摄横坐标的离线拟合系数。 */
static const float s_center_to_back_x[CAR_LAMP_PROJECTION_COEFFICIENT_COUNT] =
{
    -11.82017882f, -1.112148871f, 0.05965474006f,
    0.001312940857f, 0.003580014712f, -0.0005696456455f
};
/* 下摄公共坐标反向映射到后摄纵坐标的离线拟合系数。 */
static const float s_center_to_back_y[CAR_LAMP_PROJECTION_COEFFICIENT_COUNT] =
{
    58.42283122f, -0.04773654331f, -0.7173750546f,
    -0.003842275563f, -0.0004617931f, -0.005021741498f
};

/**
 * @brief 检查中心化图像坐标是否位于188乘120的标定范围。
 * @param point 待检查坐标。
 * @return 1表示坐标有限且位于范围内，0表示无效。
 */
static uint8 car_lamp_projection_point_valid(
    const car_lamp_projection_point_t *point)
{
    if((point == 0) || (point->x != point->x) || (point->y != point->y))
    {
        return 0U;
    }

    return ((point->x >= -CAR_LAMP_IMAGE_HALF_WIDTH) &&
            (point->x <= CAR_LAMP_IMAGE_HALF_WIDTH) &&
            (point->y >= -CAR_LAMP_IMAGE_HALF_HEIGHT) &&
            (point->y <= CAR_LAMP_IMAGE_HALF_HEIGHT)) ? 1U : 0U;
}

/**
 * @brief 检查下摄公共坐标是否位于三摄联合标定范围。
 * @param point 待检查公共坐标。
 * @return 1表示坐标有限且位于联合标定范围，0表示无效。
 */
static uint8 car_lamp_projection_center_valid(
    const car_lamp_projection_point_t *point)
{
    if((point == 0) || (point->x != point->x) || (point->y != point->y))
    {
        return 0U;
    }

    return ((point->x >= -CAR_LAMP_CENTER_HALF_WIDTH) &&
            (point->x <= CAR_LAMP_CENTER_HALF_WIDTH) &&
            (point->y >= -CAR_LAMP_CENTER_HALF_HEIGHT) &&
            (point->y <= CAR_LAMP_CENTER_HALF_HEIGHT)) ? 1U : 0U;
}

/**
 * @brief 执行固定六系数二次多项式乘加。
 * @param coefficients 常数项、x、y、x平方、xy、y平方系数。
 * @param point 输入二维坐标。
 * @return 多项式计算结果。
 */
static float car_lamp_projection_evaluate(
    const float coefficients[CAR_LAMP_PROJECTION_COEFFICIENT_COUNT],
    const car_lamp_projection_point_t *point)
{
    return coefficients[0] +
           coefficients[1] * point->x +
           coefficients[2] * point->y +
           coefficients[3] * point->x * point->x +
           coefficients[4] * point->x * point->y +
           coefficients[5] * point->y * point->y;
}

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
    car_lamp_projection_point_t *center)
{
    if((center == 0) || (car_lamp_projection_point_valid(source) == 0U))
    {
        return 0U;
    }

    if(camera == Center)
    {
        *center = *source;
    }
    else if(camera == Front)
    {
        center->x = car_lamp_projection_evaluate(s_front_to_center_x, source);
        center->y = car_lamp_projection_evaluate(s_front_to_center_y, source);
    }
    else if(camera == Back)
    {
        center->x = car_lamp_projection_evaluate(s_back_to_center_x, source);
        center->y = car_lamp_projection_evaluate(s_back_to_center_y, source);
    }
    else
    {
        return 0U;
    }

    return car_lamp_projection_center_valid(center);
}

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
    car_lamp_projection_point_t *source)
{
    if((source == 0) || (car_lamp_projection_center_valid(center) == 0U))
    {
        return 0U;
    }

    if(camera == Center)
    {
        *source = *center;
    }
    else if(camera == Front)
    {
        source->x = car_lamp_projection_evaluate(s_center_to_front_x, center);
        source->y = car_lamp_projection_evaluate(s_center_to_front_y, center);
    }
    else if(camera == Back)
    {
        source->x = car_lamp_projection_evaluate(s_center_to_back_x, center);
        source->y = car_lamp_projection_evaluate(s_center_to_back_y, center);
    }
    else
    {
        return 0U;
    }

    return car_lamp_projection_point_valid(source);
}
