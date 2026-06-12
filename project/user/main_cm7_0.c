#include "zf_common_headfile.h"
#include "IPC/ipc_image_data.h"
#include "Protocols/wifi/wifi_cmd/wifi_cmd.h"
#include "Protocols/wifi/wifi_justfloat/wifi_justfloat.h"

volatile uint32 tick_1000us_cnt = 0U;
volatile uint16 g_tick_1000HZ = 0U;
volatile uint8 g_tick_100HZ = 0U;

#define AIR_LOG_PERIOD_MS      (10U)

static void air_log_tick_1ms(void)
{
    tick_1000us_cnt++;
    if(g_tick_1000HZ < 60000U)
    {
        g_tick_1000HZ++;
    }
    if((tick_1000us_cnt % 10U) == 0U)
    {
        g_tick_100HZ++;
        if(g_tick_100HZ >= 100U)
        {
            g_tick_100HZ = 0U;
        }
    }
}

static void air_log_send_justfloat(void)
{
    static uint32 alive_counter;
    ipc_camera_spi_log_t log;
    float data[40];

    ipc_camera_spi_log_get(&log);

    data[0] = (float)alive_counter++;
    data[1] = (float)log.ready_mask;
    data[2] = (float)log.last_polled_board;
    data[3] = (float)wifi_justfloat_IsReady();
    data[4] = (float)log.board[0].online;
    data[5] = (float)log.board[0].rx_ok_count;
    data[6] = (float)log.board[0].rx_error_count;
    data[7] = (float)log.board[0].last_error;
    data[8] = (float)log.board[0].peer_last_error;
    data[9] = (float)log.board[0].flags;
    data[10] = (float)log.board[0].last_rx_head0;
    data[11] = (float)log.board[0].last_rx_head1;
    data[12] = (float)log.board[1].online;
    data[13] = (float)log.board[1].rx_ok_count;
    data[14] = (float)log.board[1].rx_error_count;
    data[15] = (float)log.board[1].last_error;
    data[16] = (float)log.board[1].peer_last_error;
    data[17] = (float)log.board[1].flags;
    data[18] = (float)log.board[1].last_rx_head0;
    data[19] = (float)log.board[1].last_rx_head1;
    data[20] = (float)log.board[0].beacon_count;
    data[21] = (float)log.board[0].first_beacon_valid;
    data[22] = log.board[0].first_beacon_x;
    data[23] = log.board[0].first_beacon_y;
    data[24] = log.board[0].first_beacon_radius;
    data[25] = (float)log.board[0].car_lamp_count;
    data[26] = (float)log.board[0].first_lamp_valid;
    data[27] = log.board[0].first_lamp_cx;
    data[28] = log.board[0].first_lamp_cy;
    data[29] = log.board[0].first_lamp_angle;
    data[30] = (float)log.board[1].beacon_count;
    data[31] = (float)log.board[1].first_beacon_valid;
    data[32] = log.board[1].first_beacon_x;
    data[33] = log.board[1].first_beacon_y;
    data[34] = log.board[1].first_beacon_radius;
    data[35] = (float)log.board[1].car_lamp_count;
    data[36] = (float)log.board[1].first_lamp_valid;
    data[37] = log.board[1].first_lamp_cx;
    data[38] = log.board[1].first_lamp_cy;
    data[39] = log.board[1].first_lamp_angle;

    (void)wifi_justfloat_Array(data, (uint8_t)(sizeof(data) / sizeof(data[0])));
}

int main(void)
{
    uint32 last_log_ms = 0U;

    clock_init(SYSTEM_CLOCK_250M);
    SCB_DisableDCache();
    wifi_cmd_Init();
    wifi_justfloat_Init();

    while(true)
    {
        system_delay_ms(1U);
        air_log_tick_1ms();

        wifi_cmd_Poll();
        if((uint32)(tick_1000us_cnt - last_log_ms) >= AIR_LOG_PERIOD_MS)
        {
            last_log_ms = tick_1000us_cnt;
            air_log_send_justfloat();
        }
    }
}
