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
    float data[26];

    ipc_camera_spi_log_get(&log);

    data[0] = (float)alive_counter++;
    data[1] = (float)wifi_justfloat_IsReady();
    data[2] = (float)log.board[0].online;
    data[3] = (float)log.board[0].beacon_count;
    data[4] = (float)log.board[0].first_beacon_valid;
    data[5] = log.board[0].first_beacon_x;
    data[6] = log.board[0].first_beacon_y;
    data[7] = log.board[0].first_beacon_radius;
    data[8] = (float)log.board[0].car_lamp_count;
    data[9] = (float)log.board[0].first_lamp_valid;
    data[10] = log.board[0].first_lamp_cx;
    data[11] = log.board[0].first_lamp_cy;
    data[12] = log.board[0].first_lamp_angle;
    data[13] = (float)log.board[1].online;
    data[14] = (float)log.board[1].beacon_count;
    data[15] = (float)log.board[1].first_beacon_valid;
    data[16] = log.board[1].first_beacon_x;
    data[17] = log.board[1].first_beacon_y;
    data[18] = log.board[1].first_beacon_radius;
    data[19] = (float)log.board[1].car_lamp_count;
    data[20] = (float)log.board[1].first_lamp_valid;
    data[21] = log.board[1].first_lamp_cx;
    data[22] = log.board[1].first_lamp_cy;
    data[23] = log.board[1].first_lamp_angle;
    data[24] = 0.0f;
    data[25] = 0.0f;

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
