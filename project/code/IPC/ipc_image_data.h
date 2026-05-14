#ifndef IPC_IMAGE_DATA_H_
#define IPC_IMAGE_DATA_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IPC_IMAGE_MAX_CIRCLES  5

#define IPC_IMAGE_DEFAULT_X        (0.0f)  /* 默认视觉X偏差，主控去视觉化后固定为0 */
#define IPC_IMAGE_DEFAULT_Y        (0.0f)  /* 默认视觉Y偏差，主控去视觉化后固定为0 */
#define IPC_IMAGE_DEFAULT_RADIUS   (0.0f)  /* 默认视觉半径，主控去视觉化后固定为0 */
#define IPC_IMAGE_DEFAULT_VALID    (1U)    /* 默认视觉有效标志，1=使用固定兜底坐标 */

typedef struct {
    float    x;
    float    y;
    float    radius;
    uint8    valid;
    uint8    _pad[3];
} ipc_image_circle_t;

typedef struct {
    volatile uint32        seq;
    uint8                  count;
    uint8                  _pad[3];
    ipc_image_circle_t     circles[IPC_IMAGE_MAX_CIRCLES];
} ipc_image_payload_t;

/* CM7_1: 将 g_image_circles 写入共享内存并通过IPC通知CM7_0 */
void ipc_image_send(void);

/* CM7_0: IPC回调函数，传给 ipc_communicate_init */
void ipc_image_callback(uint32 ipc_data);

/* CM7_0: 是否有新数据到达（调用后自动清除标志） */
uint8 ipc_image_is_new(void);

/* CM7_0: 拷贝最新共享数据到 out */
void ipc_image_get(ipc_image_payload_t *out);

/* CM7_0: 读取最新图像结果中的第一个有效圆，返回1表示有效 */
uint8 ipc_image_get_first_valid_circle(ipc_image_circle_t *out);

uint8 ipc_flight_state_send(uint8 flying);
uint8 ipc_core0_is_flying(void);

#ifdef __cplusplus
}
#endif

#endif /* IPC_IMAGE_DATA_H_ */
