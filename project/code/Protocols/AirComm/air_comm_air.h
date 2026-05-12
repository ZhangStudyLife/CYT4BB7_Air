#ifndef AIR_COMM_AIR_H
#define AIR_COMM_AIR_H

#include "zf_common_headfile.h"

/*
 * 模块作用：
 * 无人机端车空串口通信模块。无人机 CYT4BB7 通过 UART_2 与小车 CYT4BB7 通信，
 * 支持参数远程设置、函数远程调用、心跳保活和实时数据双向收发。
 *
 * 角色关系：
 * 小车是"主机"，主动发 SET_PARAM / EXEC_FUNC / HEARTBEAT 给无人机。
 * 无人机是"从机"，收到后执行并回 ACK，同时周期发心跳告诉小车自己还活着。
 *
 * 帧格式：
 * header(4B) + type(1B) + seq(1B) + len(1B) + payload(len B) + crc16(2B)
 * header = 0xAA 0xAA 0x55 0x55
 * len 只表示 payload 长度，crc16 覆盖 header 到 payload 的全部字节。
 *
 * 消息类型：
 * 0x01 SET_PARAM  小车→无人机  设置参数（按名字查找，带 min/max 限幅）
 * 0x02 ACK_PARAM  无人机→小车  参数设置结果回执
 * 0x03 EXEC_FUNC  小车→无人机  远程调用函数
 * 0x04 ACK_FUNC   无人机→小车  函数调用结果回执
 * 0x05 HEARTBEAT  双向          心跳包，payload 带 tick_ms 时间戳
 * 0x06 RUN_DATA   双向          实时数据（float 数组，不需要 ACK）
 *
 * 在线判断：
 * 无人机每 200ms 发一次心跳，小车也每 200ms 发一次。
 * 收到小车心跳就标记在线，超过 600ms 没收到就标记离线（online_status=2）。
 *
 * 硬件引脚：
 * UART_2, TX=P10_1, RX=P10_0, 波特率 1152000
 * RX 中断收到字节后放进环形队列，poll 函数消费队列并解析。
 */

#define AIR_COMM_AIR_PARAM_NAME_MAX          (32U)   /* 参数名最大长度（不含 '\0'） */
#define AIR_COMM_AIR_RUN_DATA_MAX_FLOATS     (32U)   /* 单次实时数据最多 float 个数 */
#define AIR_COMM_AIR_BAUDRATE                (1152000U) /* UART 波特率 */

/* 参数/函数操作返回状态 */
#define AIR_COMM_AIR_STATUS_OK               (0U)    /* 成功 */
#define AIR_COMM_AIR_STATUS_NOT_FOUND        (1U)    /* 参数名或 func_id 未注册 */
#define AIR_COMM_AIR_STATUS_OUT_OF_RANGE     (2U)    /* 值超出 [min, max]，已限幅 */
#define AIR_COMM_AIR_STATUS_ERROR            (3U)    /* 通用错误（payload 长度不对等） */

#define AIR_COMM_AIR_PARAM_TYPE_FLOAT        (0U)
#define AIR_COMM_AIR_PARAM_TYPE_INT32        (1U)

typedef void (*air_comm_run_data_fn)(const float *data, uint8 count);

/*
 * 通信统计结构体。
 * 用于排查丢帧、CRC 错误、队列溢出、在线状态等问题。
 * online_status: 0=从未收到心跳, 1=在线, 2=离线（超时）
 */
typedef struct
{
    uint32 tick_ms;                 /* 模块运行时间（ms），由 tick_1MS 累加 */
    uint32 tx_frame_count;          /* 已发送帧数 */
    uint32 rx_frame_count;          /* 已接收并校验通过的帧数 */
    uint32 tx_byte_count;          /* 已发送字节数（含帧头和 CRC） */
    uint32 rx_byte_count;          /* 已接收字节数（校验通过的帧） */
    uint32 raw_rx_byte_count;      /* 原始接收字节数（中断收到的全部字节） */
    uint32 crc_error_count;        /* CRC 校验失败次数 */
    uint32 rx_oversize_count;      /* 接收帧 payload 超长次数 */
    uint32 rx_queue_overflow_count; /* 接收环形队列溢出次数 */
    uint32 heartbeat_tx_count;     /* 心跳发送次数 */
    uint32 heartbeat_rx_count;     /* 心跳接收次数 */
    uint32 set_param_ok_count;     /* 参数设置成功次数 */
    uint32 set_param_fail_count;   /* 参数设置失败次数 */
    uint32 exec_func_ok_count;     /* 函数调用成功次数 */
    uint32 exec_func_fail_count;   /* 函数调用失败次数 */
    uint8 online_status;           /* 0=未连接 1=在线 2=离线 */
} air_comm_air_stats_t;

/* 可被小车远程设置的无人机参数，注册后可通过 SET_PARAM 消息修改 */
extern float air_min_area;  /* 信标检测最小面积阈值，小于此面积的目标丢弃 */
extern float air_hold_ms;   /* 信标跟踪保持时间（ms），丢失目标后继续维持 */
extern float air_x_bias;    /* 信标 X 方向偏差补偿（像素） */
extern float air_y_bias;    /* 信标 Y 方向偏差补偿（像素） */

/*
 * 初始化模块，清零所有状态，注册默认参数，配置 UART_2。
 * 在系统启动时调用一次。
 */
void air_comm_air_init(void);

/*
 * 1ms 心跳计数器，每 1ms 调一次（放在 SysTick 或定时器中断里）。
 * 只做 tick_ms++，不做其他事，中断安全。
 */
void air_comm_air_tick_1MS(void);

/*
 * 轮询消费接收队列，解析收到的字节并处理完整帧。
 * 应在主循环中尽可能频繁调用（建议 >= 1000Hz）。
 * 同时检查小车在线状态。
 */
void air_comm_air_poll(void);

/*
 * 100Hz 周期更新，负责定时发送心跳（每 200ms 一次）和检查在线超时。
 */
void air_comm_air_update_100HZ(void);

/*
 * UART 中断回调入口。收到一个字节就调这个函数，字节会存入环形队列。
 * 注意：这个函数在中断上下文调用，只做入队，不做解析。
 */
void air_comm_air_rx_byte(uint8 byte);

/* 查询小车是否在线。返回 1=在线，0=离线或从未连接。 */
uint8 air_comm_air_is_car_online(void);

/*
 * 注册一个可被远程设置的参数。
 * name: 参数名字符串（小车通过这个名字查找）
 * var:  指向 float 变量的指针（SET_PARAM 时直接写入）
 * min/max: 允许的范围，超出会限幅并返回 OUT_OF_RANGE
 * 返回 1=成功，0=失败（表满、名字太长等）
 */
uint8 air_comm_air_register_param(const char *name, void *var, uint8 type, float min, float max);

/*
 * 注册一个可被远程调用的函数。
 * func_id: 函数编号（小车通过这个 ID 调用）
 * func:    函数指针，无参无返回值
 * 返回 1=成功，0=失败（表满或 func 为 NULL）
 */
uint8 air_comm_air_register_func(uint8 func_id, void (*func)(void));

uint8 air_comm_send_run_data(const float *data, uint8 count);
void air_comm_set_run_data_callback(air_comm_run_data_fn callback);
uint8 air_comm_get_last_run_data(float *data, uint8 max_count, uint8 *count);

/*
 * 发送实时数据（float 数组）。
 * data: float 数组首地址
 * count: float 个数（<= 32）
 * 返回 1=发送成功，0=失败
 */
uint8 air_comm_air_send_run_data(const float *data, uint8 count);

/* 获取当前通信统计快照，用于菜单显示或调试。 */
void air_comm_air_get_stats(air_comm_air_stats_t *stats);

#endif
