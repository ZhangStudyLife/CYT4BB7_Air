#include "Three_Camera.h"
#include <math.h>

#define THREE_CAMERA_DEG_TO_RAD              (0.017453292519943295f) /* 角度转弧度系数。 */
#define THREE_CAMERA_BEACON_HEIGHT_M          (0.0233670966f) /* 标定信标有效高度，单位 m。 */
#define THREE_CAMERA_LAMP_HEIGHT_M            (0.2490776486f) /* 标定车灯有效高度，单位 m。 */
#define THREE_CAMERA_YAW_BIAS_RAD             (0.4068566800f) /* 标定全局航向偏置，单位 rad。 */
#define THREE_CAMERA_BEACON_MERGE_DIST_M      (0.35f) /* 跨摄相同信标的Center锚点合并半径，单位 m。 */
#define THREE_CAMERA_MAX_GROUND_DISTANCE_M    (15.0f) /* 射线地面求交允许的最大距离，单位 m。 */

typedef struct
{
    float f;
    float cx;
    float cy;
    float k1;
    float camera_to_body[3][3];
} three_camera_model_t;

typedef struct
{
    uint8 camera;
    float x_m;
    float y_m;
    float area;
} three_camera_beacon_candidate_t;

static const three_camera_model_t s_camera_model[IMAGE_CAMERA_COUNT] = /* 三摄经验内参与相机到机体安装旋转。 */
{
    {
        77.2632115f, 10.0431456f, 7.1929172f, 0.7000000f,
        {
            { -0.053257901f, -0.823901336f, 0.564225295f },
            {  0.997565442f, -0.018423355f, 0.067258975f },
            { -0.045019836f,  0.566433728f, 0.822876690f }
        }
    },
    {
        81.0801880f, 5.4973460f, 1.2174406f, 0.7000000f,
        {
            { -0.032443015f, -0.995479870f, -0.089259618f },
            {  0.996951194f, -0.038572645f,  0.067826744f },
            { -0.070963138f, -0.086786978f,  0.993696258f }
        }
    },
    {
        82.3499934f, -1.1395418f, -5.2939230f, 0.7000000f,
        {
            {  0.156584158f, 0.727257310f, -0.668265072f },
            { -0.985382340f, 0.069061905f, -0.155730851f },
            { -0.067104741f, 0.682881584f,  0.727440510f }
        }
    }
};

/*
 * 函数功能: 清零三摄融合输出。
 * 输入参数: result - 待清零输出结构体。
 * 输出参数/返回值: 无。
 */
static void Three_Camera_ClearResult(three_camera_result_t *result)
{
    uint8 i;

    result->car_lamp.valid = 0U;
    result->car_lamp.camera_mask = 0U;
    result->car_lamp.x_m = 0.0f;
    result->car_lamp.y_m = 0.0f;
    result->car_lamp.angle_deg = 0.0f;
    result->beacon_count = 0U;
    for(i = 0U; i < THREE_CAMERA_MAX_BEACON_COUNT; i++)
    {
        result->beacon[i].valid = 0U;
        result->beacon[i].camera_mask = 0U;
        result->beacon[i].x_m = 0.0f;
        result->beacon[i].y_m = 0.0f;
        result->beacon[i].area = 0.0f;
    }
}

/*
 * 函数功能: 生成机体系 FRD 到水平全局坐标系的旋转矩阵。
 * 输入参数: roll_rad/pitch_rad/yaw_rad - 飞机欧拉角，单位 rad。
 * 输出参数/返回值: out - 输出 3x3 旋转矩阵。
 */
static void Three_Camera_BuildWorldRotation(float roll_rad,
                                             float pitch_rad,
                                             float yaw_rad,
                                             float out[3][3])
{
    float cr = cosf(roll_rad);
    float sr = sinf(roll_rad);
    float cp = cosf(pitch_rad);
    float sp = sinf(pitch_rad);
    float cy = cosf(yaw_rad);
    float sy = sinf(yaw_rad);

    out[0][0] = cp * cy;
    out[0][1] = sr * sp * cy - cr * sy;
    out[0][2] = cr * sp * cy + sr * sy;
    out[1][0] = cp * sy;
    out[1][1] = sr * sp * sy + cr * cy;
    out[1][2] = cr * sp * sy - sr * cy;
    out[2][0] = -sp;
    out[2][1] = sr * cp;
    out[2][2] = cr * cp;
}

/*
 * 函数功能: 将单个相机像素射线与目标高度平面求交，得到飞机光心原点的水平全局坐标。
 * 输入参数: camera - 相机编号；x/y - 中心化像素坐标；height_m - 目标高度；air_height_m - 飞机高度；world - 姿态旋转矩阵。
 * 输出参数/返回值: 成功时通过 out_x/out_y 返回坐标，返回 1；射线不朝向地面或超量程时返回 0。
 */
static uint8 Three_Camera_ProjectPoint(uint8 camera,
                                       float x,
                                       float y,
                                       float height_m,
                                       float air_height_m,
                                       const float world[3][3],
                                       float *out_x,
                                       float *out_y)
{
    const three_camera_model_t *model = &s_camera_model[camera];
    float ray_x = (x - model->cx) / model->f;
    float ray_y = (y - model->cy) / model->f;
    float gain = 1.0f + model->k1 * (ray_x * ray_x + ray_y * ray_y);
    float body_x;
    float body_y;
    float body_z;
    float world_x;
    float world_y;
    float world_z;
    float distance;

    ray_x *= gain;
    ray_y *= gain;
    body_x = model->camera_to_body[0][0] * ray_x +
             model->camera_to_body[0][1] * ray_y + model->camera_to_body[0][2];
    body_y = model->camera_to_body[1][0] * ray_x +
             model->camera_to_body[1][1] * ray_y + model->camera_to_body[1][2];
    body_z = model->camera_to_body[2][0] * ray_x +
             model->camera_to_body[2][1] * ray_y + model->camera_to_body[2][2];
    world_x = world[0][0] * body_x + world[0][1] * body_y + world[0][2] * body_z;
    world_y = world[1][0] * body_x + world[1][1] * body_y + world[1][2] * body_z;
    world_z = world[2][0] * body_x + world[2][1] * body_y + world[2][2] * body_z;
    if(world_z <= 0.0001f)
    {
        return 0U;
    }

    distance = (air_height_m - height_m) / world_z;
    if((distance <= 0.0f) || (distance > THREE_CAMERA_MAX_GROUND_DISTANCE_M))
    {
        return 0U;
    }

    *out_x = distance * world_x;
    *out_y = distance * world_y;
    return 1U;
}

/*
 * 函数功能: 将指定相机车灯长轴投影为水平全局坐标系中的无向角度。
 * 输入参数: camera - 相机编号；lamp - 原始车灯；air_height_m - 飞机高度；world - 姿态旋转矩阵。
 * 输出参数/返回值: 成功时通过 out_angle_deg 返回角度并返回 1，投影失败时返回 0。
 */
static uint8 Three_Camera_ProjectLampAngle(uint8 camera,
                                           const car_lamp_data *lamp,
                                           float air_height_m,
                                           const float world[3][3],
                                           float *out_angle_deg)
{
    float angle_rad = lamp->angle * THREE_CAMERA_DEG_TO_RAD;
    float half_length = 0.5f * lamp->length;
    float x1;
    float y1;
    float x2;
    float y2;

    if((half_length <= 0.0f) ||
       (Three_Camera_ProjectPoint(camera,
                                  lamp->cx - half_length * cosf(angle_rad),
                                  lamp->cy - half_length * sinf(angle_rad),
                                  THREE_CAMERA_LAMP_HEIGHT_M,
                                  air_height_m,
                                  world,
                                  &x1,
                                  &y1) == 0U) ||
       (Three_Camera_ProjectPoint(camera,
                                  lamp->cx + half_length * cosf(angle_rad),
                                  lamp->cy + half_length * sinf(angle_rad),
                                  THREE_CAMERA_LAMP_HEIGHT_M,
                                  air_height_m,
                                  world,
                                  &x2,
                                  &y2) == 0U))
    {
        return 0U;
    }

    *out_angle_deg = atan2f(y2 - y1, x2 - x1) / THREE_CAMERA_DEG_TO_RAD;
    return 1U;
}

/*
 * 函数功能: 融合三路车灯的世界坐标与无向长轴角度。
 * 输入参数: image - 三路原始图像检测结果；air_height_m - 飞机高度；world - 姿态旋转矩阵；result - 融合输出。
 * 输出参数/返回值: 无。
 */
static void Three_Camera_BuildLamp(const struct image_data image[IMAGE_CAMERA_COUNT],
                                   float air_height_m,
                                   const float world[3][3],
                                   three_camera_result_t *result)
{
    uint8 camera;
    uint8 count = 0U;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_cos2 = 0.0f;
    float sum_sin2 = 0.0f;

    for(camera = Front; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        const car_lamp_data *lamp = &image[camera].car_lamp_data[0];
        float x_m;
        float y_m;
        float angle_deg;
        if((image_data_car_lamp_valid(lamp) == 0U) ||
           (Three_Camera_ProjectPoint(camera, lamp->cx, lamp->cy,
                                      THREE_CAMERA_LAMP_HEIGHT_M, air_height_m,
                                      world, &x_m, &y_m) == 0U) ||
           (Three_Camera_ProjectLampAngle(camera, lamp, air_height_m,
                                          world, &angle_deg) == 0U))
        {
            continue;
        }
        sum_x += x_m;
        sum_y += y_m;
        sum_cos2 += cosf(2.0f * angle_deg * THREE_CAMERA_DEG_TO_RAD);
        sum_sin2 += sinf(2.0f * angle_deg * THREE_CAMERA_DEG_TO_RAD);
        result->car_lamp.camera_mask |= (uint8)(1U << camera);
        count++;
    }

    if(count != 0U)
    {
        result->car_lamp.valid = 1U;
        result->car_lamp.x_m = sum_x / (float)count;
        result->car_lamp.y_m = sum_y / (float)count;
        result->car_lamp.angle_deg = 0.5f * atan2f(sum_sin2, sum_cos2) / THREE_CAMERA_DEG_TO_RAD;
    }
}

static void Three_Camera_AddBeacon(const three_camera_beacon_candidate_t *candidate,
                                   uint8 camera,
                                   three_camera_result_t *result,
                                   uint8 result_index,
                                   uint8 *member_count)
{
    float count = (float)member_count[result_index];

    result->beacon[result_index].valid = 1U;
    result->beacon[result_index].x_m =
        (result->beacon[result_index].x_m * count + candidate->x_m) / (count + 1.0f);
    result->beacon[result_index].y_m =
        (result->beacon[result_index].y_m * count + candidate->y_m) / (count + 1.0f);
    if(candidate->area > result->beacon[result_index].area)
    {
        result->beacon[result_index].area = candidate->area;
    }
    result->beacon[result_index].camera_mask |= (uint8)(1U << camera);
    member_count[result_index]++;
}

/*
 * 函数功能: 将三摄信标候选按世界坐标合并为物理信标。
 * 输入参数: image - 三路原始图像检测结果；air_height_m - 飞机高度；world - 姿态旋转矩阵；result - 融合输出。
 * 输出参数/返回值: 无。
 */
static void Three_Camera_BuildBeacons(const struct image_data image[IMAGE_CAMERA_COUNT],
                                      float air_height_m,
                                      const float world[3][3],
                                      three_camera_result_t *result)
{
    three_camera_beacon_candidate_t candidate[IMAGE_CAMERA_COUNT * IMAGE_MAX_BEACON_COUNT];
    float member_x[THREE_CAMERA_MAX_BEACON_COUNT][IMAGE_CAMERA_COUNT];
    float member_y[THREE_CAMERA_MAX_BEACON_COUNT][IMAGE_CAMERA_COUNT];
    uint8 member_count[THREE_CAMERA_MAX_BEACON_COUNT] = {0U};
    uint8 candidate_count = 0U;
    uint8 camera;
    uint8 index;
    uint8 i;

    for(camera = Front; camera < IMAGE_CAMERA_COUNT; camera++)
    {
        for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
        {
            const beacon_data *beacon = &image[camera].beacon_data[index];
            if((image_data_beacon_valid(beacon) == 0U) ||
               (Three_Camera_ProjectPoint(camera, beacon->x, beacon->y,
                                           THREE_CAMERA_BEACON_HEIGHT_M,
                                           air_height_m, world,
                                           &candidate[candidate_count].x_m,
                                           &candidate[candidate_count].y_m) == 0U))
            {
                continue;
            }
            candidate[candidate_count].camera = camera;
            candidate[candidate_count].area = beacon->area;
            candidate_count++;
        }
    }

    for(i = 0U; i < candidate_count; i++)
    {
        uint8 best = 0xFFU;
        float best_distance_sq = THREE_CAMERA_BEACON_MERGE_DIST_M *
                                 THREE_CAMERA_BEACON_MERGE_DIST_M;
        uint8 j;

        for(j = 0U; j < result->beacon_count; j++)
        {
            uint8 camera_bit = (uint8)(1U << candidate[i].camera);
            float match_distance_sq = 0.0f;

            if(((result->beacon[j].camera_mask & camera_bit) != 0U) ||
               ((candidate[i].camera != Center) &&
                ((result->beacon[j].camera_mask & (uint8)(1U << Center)) == 0U)))
            {
                continue;
            }

            if(candidate[i].camera != Center)
            {
                float dx = candidate[i].x_m - member_x[j][Center];
                float dy = candidate[i].y_m - member_y[j][Center];
                match_distance_sq = dx * dx + dy * dy;
            }
            else
            {
                uint8 member_camera;
                for(member_camera = Front; member_camera < IMAGE_CAMERA_COUNT; member_camera++)
                {
                    if((result->beacon[j].camera_mask & (uint8)(1U << member_camera)) != 0U)
                    {
                        float dx = candidate[i].x_m - member_x[j][member_camera];
                        float dy = candidate[i].y_m - member_y[j][member_camera];
                        match_distance_sq = dx * dx + dy * dy;
                        break;
                    }
                }
            }

            if(match_distance_sq < best_distance_sq)
            {
                best = j;
                best_distance_sq = match_distance_sq;
            }
        }

        if(best == 0xFFU)
        {
            if(result->beacon_count >= THREE_CAMERA_MAX_BEACON_COUNT)
            {
                continue;
            }
            best = result->beacon_count++;
        }

        Three_Camera_AddBeacon(&candidate[i], candidate[i].camera, result,
                               best, member_count);
        member_x[best][candidate[i].camera] = candidate[i].x_m;
        member_y[best][candidate[i].camera] = candidate[i].y_m;
    }
}

/*
 * 函数功能: 将三摄原始检测直接投影到水平全局坐标系，并融合车灯与信标。
 * 输入参数: image - 三路原始检测；roll_deg/pitch_deg/yaw_deg - 飞机欧拉角，单位 deg；
 *           height_mm - 飞机离地高度，单位 mm；height_valid - 高度有效标志；result - 融合输出。
 * 输出参数/返回值: 投影输入有效时返回 1；高度无效、输入为空或输出为空时返回 0。
 */
uint8 Three_Camera_Update(const struct image_data image[IMAGE_CAMERA_COUNT],
                          float roll_deg,
                          float pitch_deg,
                          float yaw_deg,
                          float height_mm,
                          uint8 height_valid,
                          three_camera_result_t *result)
{
    float world[3][3];
    float air_height_m;

    if(result == 0)
    {
        return 0U;
    }
    Three_Camera_ClearResult(result);
    if((image == 0) || (height_valid == 0U) || (height_mm <= 0.0f))
    {
        return 0U;
    }

    air_height_m = height_mm * 0.001f;
    Three_Camera_BuildWorldRotation(roll_deg * THREE_CAMERA_DEG_TO_RAD,
                                    pitch_deg * THREE_CAMERA_DEG_TO_RAD,
                                    yaw_deg * THREE_CAMERA_DEG_TO_RAD + THREE_CAMERA_YAW_BIAS_RAD,
                                    world);
    Three_Camera_BuildLamp(image, air_height_m, world, result);
    Three_Camera_BuildBeacons(image, air_height_m, world, result);
    return 1U;
}
