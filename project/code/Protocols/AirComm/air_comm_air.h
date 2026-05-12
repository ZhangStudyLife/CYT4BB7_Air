#ifndef AIR_COMM_AIR_H
#define AIR_COMM_AIR_H

#include "zf_common_headfile.h"

#define AIR_COMM_AIR_PARAM_NAME_MAX          (16U)
#define AIR_COMM_AIR_RUN_DATA_MAX_FLOATS     (32U)
#define AIR_COMM_AIR_BAUDRATE                (1152000U)

#define AIR_COMM_AIR_STATUS_OK               (0U)
#define AIR_COMM_AIR_STATUS_NOT_FOUND        (1U)
#define AIR_COMM_AIR_STATUS_OUT_OF_RANGE     (2U)
#define AIR_COMM_AIR_STATUS_ERROR            (3U)

typedef struct
{
    uint32 tick_ms;
    uint32 tx_frame_count;
    uint32 rx_frame_count;
    uint32 tx_byte_count;
    uint32 rx_byte_count;
    uint32 raw_rx_byte_count;
    uint32 crc_error_count;
    uint32 rx_oversize_count;
    uint32 rx_queue_overflow_count;
    uint32 heartbeat_tx_count;
    uint32 heartbeat_rx_count;
    uint32 set_param_ok_count;
    uint32 set_param_fail_count;
    uint32 exec_func_ok_count;
    uint32 exec_func_fail_count;
    uint8 online_status;
} air_comm_air_stats_t;

extern float air_min_area;
extern float air_hold_ms;
extern float air_x_bias;
extern float air_y_bias;

void air_comm_air_init(void);
void air_comm_air_tick_1MS(void);
void air_comm_air_poll(void);
void air_comm_air_update_100HZ(void);
void air_comm_air_rx_byte(uint8 byte);
uint8 air_comm_air_is_car_online(void);
uint8 air_comm_air_register_param(const char *name, float *var, float min, float max);
uint8 air_comm_air_register_func(uint8 func_id, void (*func)(void));
uint8 air_comm_air_send_run_data(const float *data, uint8 count);
void air_comm_air_get_stats(air_comm_air_stats_t *stats);

#endif
