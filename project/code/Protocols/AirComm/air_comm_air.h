#ifndef AIR_COMM_AIR_H
#define AIR_COMM_AIR_H

#include "zf_common_headfile.h"

/*
 * 模块作用：
 * 无人机端车空串口通信模块。无人机 CYT4BB7 通过 UART_2 与小车 CYT4BB7 通信，
 * 支持参数远程设置、远程命令、心跳保活和实时数据双向收发。
 *
 * 角色关系：
 * 小车是"主机"，主动发 SET_PARAM / EXEC_COMMAND / HEARTBEAT 给无人机。
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
 * 0x03 EXEC_COMMAND 小车→无人机  执行远程命令
 * 0x04 ACK_COMMAND  无人机→小车  远程命令结果回执
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
#define AIR_COMM_AIR_COMMAND_NAME_MAX        (32U)   /* 远程命令名最大长度，不含 '\0' */
#define AIR_COMM_AIR_ACK_TEXT_MAX            (96U)   /* 远程命令 ACK 文本最大长度，不含 '\0' */
#define AIR_COMM_AIR_RUN_DATA_MAX_FLOATS     (52U)   /* 单次实时数据最多 float 个数 */
#define AIR_COMM_AIR_BAUDRATE                (1152000U) /* UART 波特率 */

/* 参数/远程命令操作返回状态 */
#define AIR_COMM_AIR_STATUS_OK               (0U)    /* 成功 */
#define AIR_COMM_AIR_STATUS_NOT_FOUND        (1U)    /* 参数名或远程命令未注册 */
#define AIR_COMM_AIR_STATUS_OUT_OF_RANGE     (2U)    /* 值超出 [min, max]，已限幅 */
#define AIR_COMM_AIR_STATUS_ERROR            (3U)    /* 通用错误（payload 长度不对等） */
#define AIR_COMM_AIR_STATUS_BUSY             (4U)    /* 远程命令忙，已有命令正在执行 */

#define AIR_COMM_AIR_STATUS_TIMEOUT          (5U)    /* 远端参数事务超时 */
#define AIR_COMM_AIR_STATUS_MISMATCH         (6U)    /* 两颗2BL3读回值不一致 */
#define AIR_COMM_AIR_STATUS_PARTIAL          (7U)    /* 两板部分成功，已完成回滚 */
#define AIR_COMM_AIR_STATUS_ROLLBACK_FAIL    (8U)    /* 两板部分成功且回滚失败 */

#define AIR_COMM_AIR_PARAM_TYPE_FLOAT        (0U)
#define AIR_COMM_AIR_PARAM_TYPE_INT32        (1U)

typedef void (*air_comm_run_data_fn)(const float *data, uint8 count);

typedef enum
{
    AIR_COMM_AIR_COMMAND_MODE_POLLING = 0,    /* 轮询型远程命令：持续执行，直到收到 NONE */
    AIR_COMM_AIR_COMMAND_MODE_INSTANT         /* 立即型远程命令：完成后自动退出 */
} air_comm_air_command_mode_t;

typedef void (*air_comm_air_command_fn)(void);

/*
 * 通信统计结构体。
 * 用于排查丢帧、CRC 错误、队列溢出、在线状态等问题。
 * online_status: 0=从未收到心跳, 1=在线, 2=离线（超时）
 */
typedef struct
{
    uint32 tick_ms;                 /* 模块运行时间（ms），由 tick_1MS 累加 */
    uint32 tx_frame_count;          /* 成功进入发送队列的帧数 */
    uint32 rx_frame_count;          /* 已接收并校验通过的帧数 */
    uint32 tx_byte_count;          /* 成功进入发送队列的字节数（含帧头和 CRC） */
    uint32 rx_byte_count;          /* 已接收字节数（校验通过的帧） */
    uint32 raw_rx_byte_count;      /* 原始接收字节数（中断收到的全部字节） */
    uint32 crc_error_count;        /* CRC 校验失败次数 */
    uint32 rx_oversize_count;      /* 接收帧 payload 超长次数 */
    uint32 rx_queue_overflow_count; /* 接收环形队列溢出次数 */
    uint32 tx_queue_overflow_count; /* 发送队列满或RUN_DATA限额导致的拒绝次数 */
    uint32 heartbeat_tx_count;     /* 心跳发送次数 */
    uint32 heartbeat_rx_count;     /* 心跳接收次数 */
    uint32 set_param_ok_count;     /* 参数设置成功次数 */
    uint32 set_param_fail_count;   /* 参数设置失败次数 */
    uint32 command_ok_count;       /* 远程命令成功次数 */
    uint32 command_fail_count;     /* 远程命令失败次数 */
    uint8 tx_pending_frames;       /* 当前等待写入硬件FIFO的帧数 */
    uint8 tx_queue_high_water;     /* 发送队列历史最高占用帧数 */
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
extern int32 c1_beacon_thr;  /* 核1信标二值化阈值的核0菜单镜像，不参与核0飞控计算 */
extern int32 bl3_beacon_thr; /* 两颗2BL3信标二值化阈值的核0菜单镜像，不参与核0飞控计算 */
extern int32 c1_exp_time;    /* 核1摄像头曝光时间的核0菜单镜像，不参与核0飞控计算 */
extern int32 bl3_exp_time;   /* 两颗2BL3曝光时间的核0菜单镜像，不参与核0飞控计算 */
extern int32 c1_screen_mode; /* 核1屏幕显示模式的核0菜单镜像，不参与核0飞控计算 */
extern int32 c1_beacon_min;
extern int32 c1_edge_min;
extern int32 c1_edge_thr;
extern int32 c1_lamp_thr;
extern int32 c1_lamp_min;
extern int32 c1_lamp_max;
extern float c1_lamp_elong;
extern float c1_lamp_len;
extern int32 c1_near_pad;
extern int32 c1_near_min;
extern int32 c1_near_iso_min;
extern int32 c1_near_bg;
extern float c1_match_dist;
extern float c1_gate_dist;
extern float c1_new_dist;
extern int32 c1_confirm;
extern int32 c1_misses;
extern float c1_pos_alpha;
extern float c1_vel_alpha;
extern int32 bl3_edge_thr;   /* 两颗2BL3边缘区域二值化阈值镜像 */
extern int32 bl3_track_thr;  /* 两颗2BL3跟踪补强二值化阈值镜像 */
extern int32 bl3_lamp_thr;   /* 两颗2BL3车灯普通区域二值化阈值镜像 */
extern int32 bl3_lamp_up_thr; /* 两颗2BL3车灯上部区域二值化阈值镜像 */
extern float bl3_lamp_up_y;  /* 两颗2BL3车灯上部区域纵向边界镜像，单位像素 */
extern float bl3_bridge_gap; /* 两颗2BL3车灯横向暗缝最大连接宽度镜像，单位像素 */
extern int32 bl3_beacon_min; /* 两颗2BL3普通信标最小面积镜像 */
extern int32 bl3_edge_min;   /* 两颗2BL3边缘信标最小面积镜像 */
extern int32 bl3_top_max;    /* 两颗2BL3顶部信标最大面积镜像 */
extern int32 bl3_edge_max;   /* 两颗2BL3侧边信标最大面积镜像 */
extern int32 bl3_lamp_min;   /* 两颗2BL3车灯最小面积镜像 */
extern int32 bl3_lamp_max;   /* 两颗2BL3车灯普通最大面积镜像 */
extern float bl3_lamp_elong; /* 两颗2BL3车灯最小长宽比镜像 */
extern float bl3_back_len;   /* 两颗2BL3后摄车灯最小长轴镜像，单位像素 */
extern int32 bl3_iso_gray;   /* 两颗2BL3孤立小信标最小峰值灰度镜像 */
extern int32 bl3_iso_bg;     /* 两颗2BL3孤立小信标最大背景灰度镜像 */
extern float bl3_ring_in;    /* 两颗2BL3局部背景方环内半径镜像，单位像素 */
extern float bl3_ring_out;   /* 两颗2BL3局部背景方环外半径镜像，单位像素 */
extern float bl3_near_pad;   /* 两颗2BL3车灯附近信标外扩距离镜像，单位像素 */
extern int32 bl3_near_min;   /* 两颗2BL3车灯附近信标最小面积镜像 */
extern int32 bl3_near_gray;  /* 两颗2BL3车灯附近孤立信标最小峰值灰度镜像 */
extern int32 bl3_near_bg;    /* 两颗2BL3车灯附近孤立信标最大背景灰度镜像 */
extern float bl3_match_dist; /* 两颗2BL3信标跟踪匹配距离镜像，单位像素 */
extern float bl3_gate_dist;  /* 两颗2BL3已确认目标匹配门距离镜像，单位像素 */
extern float bl3_new_dist;   /* 两颗2BL3新目标重建距离镜像，单位像素 */
extern int32 bl3_confirm;    /* 两颗2BL3目标初始化确认帧数镜像 */
extern int32 bl3_misses;     /* 两颗2BL3信标最大连续丢失帧数镜像 */
extern float bl3_pos_alpha;  /* 两颗2BL3位置滤波当前测量权重镜像 */
extern float bl3_vel_alpha;  /* 两颗2BL3速度滤波当前测量权重镜像 */
extern int32 bl3_stream_mode; /* 两颗2BL3图传内容模式镜像 */
extern float bl3_lamp_width;
extern float bl3_narrow_width;
extern float bl3_narrow_elong;
extern int32 bl3_upper_area;
extern float bl3_upper_len;
extern float bl3_upper_width;
extern float bl3_compact_y;
extern int32 bl3_compact_area;
extern float bl3_compact_len;
extern float bl3_compact_width;
extern float bl3_compact_elong;
extern float bl3_vglare_elong;
extern int32 bl3_vglare_gray;
extern float bl3_linear_elong;
extern int32 bl3_weak_c_thr;
extern int32 bl3_weak_c_min;
extern int32 bl3_weak_c_max;
extern int32 bl3_weak_c_gray;
extern int32 bl3_weak_c_bg;
extern int32 bl3_shape_min;
extern float bl3_shape_ratio;
extern int32 bl3_shape_fill;
extern int32 bl3_shape_s_fill;
extern float bl3_top_v_elong;
extern int32 bl3_sat_t_gray;

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

/* 远端参数设置或取消回滚尚未收敛时返回1，用于阻止进入起飞流程。 */
uint8 air_comm_air_remote_param_busy(void);

/*
 * 100Hz 周期更新，负责定时发送心跳（每 200ms 一次）和检查在线超时。
 */
void air_comm_air_update_100HZ(void);

/*
 * UART 中断回调入口。收到一个字节就调这个函数，字节会存入环形队列。
 * 注意：这个函数在中断上下文调用，只做入队，不做解析。
 */
void air_comm_air_rx_byte(uint8 byte);

/*
 * 函数功能：在UART2中断中批量填充AirComm发送FIFO。
 * 输入参数：
 *   无
 * 返回值：
 *   无
 */
void air_comm_air_uart_tx_isr(void);

/* 查询小车是否在线。返回 1=在线，0=离线或从未连接。 */
uint8 air_comm_air_is_car_online(void);

/**
 * @brief 查询小车运行数据是否仍在指定超时范围内。
 * @param timeout_ms 最大允许未更新时间，单位 ms。
 * @return 1 表示运行数据有效且未超时，0 表示从未收到或已经超时。
 */
uint8 air_comm_air_is_run_data_fresh(uint32 timeout_ms);

/*
 * 注册一个可被远程设置的参数。
 * name: 参数名字符串（小车通过这个名字查找）
 * var:  指向 float 变量的指针（SET_PARAM 时直接写入）
 * min/max: 允许的范围，超出会限幅并返回 OUT_OF_RANGE
 * 返回 1=成功，0=失败（表满、名字太长等）
 */
uint8 air_comm_air_register_param(const char *name, void *var, uint8 type, float min, float max);

/*
 * 注册轮询型 Air 远程命令。
 * name: 远程命令名，0x03 payload 中传输的字符串。
 * run:  100Hz 周期调用，只写命令需要持续运行的内容。
 */
uint8 air_comm_air_register_polling_command(const char *name, air_comm_air_command_fn run);

/*
 * 注册立即退出型 Air 远程命令。
 * name: 远程命令名，0x03 payload 中传输的字符串。
 * run:  ACK_OK 之后在 100Hz 调度中执行一次，完成后框架自动发送 ACK_EXIT_OK。
 */
uint8 air_comm_air_register_instant_command(const char *name, air_comm_air_command_fn run);
uint8 air_comm_air_send_command_ack_text(uint8 seq, const char *text);

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
