#include "air_remote_cmd.h"

#define AIR_REMOTE_CMD_MAX_COUNT             (8U)
#define AIR_REMOTE_CMD_NONE                  "NONE"
#define AIR_REMOTE_CMD_MOTOR_PWM             (500)
#define AIR_REMOTE_CMD_MOTOR_HOLD_TICKS      (10U)
#define AIR_REMOTE_CMD_SCREEN_LINE_COUNT     (8U)
#define AIR_REMOTE_CMD_SCREEN_LINE_LEN       (31U)

typedef struct
{
    const char *name;
    air_remote_cmd_mode_t mode;
    air_remote_cmd_start_fn start;
    air_remote_cmd_poll_fn poll;
    air_remote_cmd_stop_fn stop;
} air_remote_cmd_entry_t;

typedef struct
{
    uint8 active;
    uint8 seq;
    uint8 step;
    uint8 tick;
    uint8 motor_was_enabled;
} air_remote_motor_test_t;

static air_remote_cmd_entry_t s_cmd_table[AIR_REMOTE_CMD_MAX_COUNT];
static uint8 s_cmd_count;
static const air_remote_cmd_entry_t *s_active_cmd;
static uint8 s_active_seq;
static air_remote_motor_test_t s_motor_test;
static char s_screen_cache[AIR_REMOTE_CMD_SCREEN_LINE_COUNT][AIR_REMOTE_CMD_SCREEN_LINE_LEN + 1U];
static uint8 s_screen_ready;
static uint8 s_last_done_valid;
static uint8 s_last_done_seq;
static char s_last_done_name[AIR_COMM_AIR_FUNC_NAME_MAX + 1U];

uint8 air_remote_cmd_register(const char *name,
                              air_remote_cmd_mode_t mode,
                              air_remote_cmd_start_fn start,
                              air_remote_cmd_poll_fn poll,
                              air_remote_cmd_stop_fn stop)
{
    if((name == NULL) || (s_cmd_count >= AIR_REMOTE_CMD_MAX_COUNT))
    {
        return 0U;
    }

    s_cmd_table[s_cmd_count].name = name;
    s_cmd_table[s_cmd_count].mode = mode;
    s_cmd_table[s_cmd_count].start = start;
    s_cmd_table[s_cmd_count].poll = poll;
    s_cmd_table[s_cmd_count].stop = stop;
    s_cmd_count++;

    return 1U;
}

static const air_remote_cmd_entry_t *air_remote_cmd_find(const char *name)
{
    uint8 i;

    if(name == NULL)
    {
        return NULL;
    }

    for(i = 0U; i < s_cmd_count; i++)
    {
        if(strcmp(s_cmd_table[i].name, name) == 0)
        {
            return &s_cmd_table[i];
        }
    }

    return NULL;
}

static void air_remote_cmd_screen_reset(void)
{
    uint8 i;

    ips114_set_font(IPS114_8X16_FONT);
    ips114_set_color(RGB565_GREEN, RGB565_BLACK);
    ips114_clear();
    for(i = 0U; i < AIR_REMOTE_CMD_SCREEN_LINE_COUNT; i++)
    {
        s_screen_cache[i][0] = '\0';
    }
    s_screen_ready = 1U;
}

static void air_remote_cmd_screen_line(uint8 line, const char *text)
{
    char padded[AIR_REMOTE_CMD_SCREEN_LINE_LEN + 1U];
    uint8 len;
    uint8 i;

    if((line >= AIR_REMOTE_CMD_SCREEN_LINE_COUNT) || (text == NULL))
    {
        return;
    }

    len = (uint8)strlen(text);
    if(len > AIR_REMOTE_CMD_SCREEN_LINE_LEN)
    {
        len = AIR_REMOTE_CMD_SCREEN_LINE_LEN;
    }

    for(i = 0U; i < AIR_REMOTE_CMD_SCREEN_LINE_LEN; i++)
    {
        padded[i] = (i < len) ? text[i] : ' ';
    }
    padded[AIR_REMOTE_CMD_SCREEN_LINE_LEN] = '\0';

    if(strcmp(s_screen_cache[line], padded) != 0)
    {
        strcpy(s_screen_cache[line], padded);
        ips114_show_string(0U, (uint16)(line * 16U), padded);
    }
}

static uint8 air_remote_cmd_show_imu_start(void)
{
    air_remote_cmd_screen_reset();
    return 1U;
}

static void air_remote_cmd_show_imu_poll(void)
{
    char line[40];

    if(s_screen_ready == 0U)
    {
        air_remote_cmd_screen_reset();
    }

    air_remote_cmd_screen_line(0U, "IMU RAW");
    snprintf(line, sizeof(line), "ACC X:%6d Y:%6d",
             (int)ICM42688_RAW.acc_x_lsb,
             (int)ICM42688_RAW.acc_y_lsb);
    air_remote_cmd_screen_line(1U, line);
    snprintf(line, sizeof(line), "ACC Z:%6d T:%6d",
             (int)ICM42688_RAW.acc_z_lsb,
             (int)ICM42688_RAW.temp_lsb);
    air_remote_cmd_screen_line(2U, line);
    snprintf(line, sizeof(line), "GYR X:%6d Y:%6d",
             (int)ICM42688_RAW.gyro_x_lsb,
             (int)ICM42688_RAW.gyro_y_lsb);
    air_remote_cmd_screen_line(3U, line);
    snprintf(line, sizeof(line), "GYR Z:%6d", (int)ICM42688_RAW.gyro_z_lsb);
    air_remote_cmd_screen_line(4U, line);
    snprintf(line, sizeof(line), "ACCg %5.2f %5.2f",
             (double)g_imufilter_1000hz.accx,
             (double)g_imufilter_1000hz.accy);
    air_remote_cmd_screen_line(5U, line);
    snprintf(line, sizeof(line), "GYRd %5.1f %5.1f",
             (double)g_imufilter_1000hz.gyrox,
             (double)g_imufilter_1000hz.gyroy);
    air_remote_cmd_screen_line(6U, line);
    snprintf(line, sizeof(line), "RPY %4.1f %4.1f %4.1f",
             (double)g_euler.roll,
             (double)g_euler.pitch,
             (double)g_euler.yaw);
    air_remote_cmd_screen_line(7U, line);
}

static uint8 air_remote_cmd_show_flow_start(void)
{
    air_remote_cmd_screen_reset();
    return 1U;
}

static void air_remote_cmd_show_flow_poll(void)
{
    char line[40];

    if(s_screen_ready == 0U)
    {
        air_remote_cmd_screen_reset();
    }

    air_remote_cmd_screen_line(0U, "OPTICAL FLOW");
    snprintf(line, sizeof(line), "LC X:%6d Y:%6d",
             (int)lc302_data.flow_x_integral,
             (int)lc302_data.flow_y_integral);
    air_remote_cmd_screen_line(1U, line);
    snprintf(line, sizeof(line), "DT:%6u DIS:%5u",
             (unsigned int)lc302_data.integration_timespan,
             (unsigned int)lc302_data.ground_distance);
    air_remote_cmd_screen_line(2U, line);
    snprintf(line, sizeof(line), "VALID:%u VER:%u",
             (unsigned int)lc302_data.valid,
             (unsigned int)lc302_data.version);
    air_remote_cmd_screen_line(3U, line);
    snprintf(line, sizeof(line), "PMW dX:%5d dY:%5d",
             (int)g_pmw3901_raw.deltaX,
             (int)g_pmw3901_raw.deltaY);
    air_remote_cmd_screen_line(4U, line);
    snprintf(line, sizeof(line), "SQUAL:%3u OBS:%3u",
             (unsigned int)g_pmw3901_raw.squal,
             (unsigned int)g_pmw3901_raw.observation);
    air_remote_cmd_screen_line(5U, line);
    snprintf(line, sizeof(line), "RAW %3u %3u %3u",
             (unsigned int)g_pmw3901_raw.rawDataSum,
             (unsigned int)g_pmw3901_raw.maxRawData,
             (unsigned int)g_pmw3901_raw.minRawData);
    air_remote_cmd_screen_line(6U, line);
    snprintf(line, sizeof(line), "SHUT:%5u MOT:%3u",
             (unsigned int)g_pmw3901_raw.shutter,
             (unsigned int)g_pmw3901_raw.motion);
    air_remote_cmd_screen_line(7U, line);
}

static void air_remote_cmd_screen_stop(void)
{
    s_screen_ready = 0U;
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_clear();
}

static void air_remote_cmd_motor_output(uint8 motor_index)
{
    int32 throttle[MOTOR_NUM] = {0, 0, 0, 0};

    if(motor_index < MOTOR_NUM)
    {
        throttle[motor_index] = AIR_REMOTE_CMD_MOTOR_PWM;
    }
    Motor_SetThrottleAll(throttle);
}

static void air_remote_cmd_motor_finish(uint8 send_ack)
{
    int32 throttle[MOTOR_NUM] = {0, 0, 0, 0};

    Motor_SetThrottleAll(throttle);
    if(s_motor_test.motor_was_enabled == 0U)
    {
        Motor_Disable();
    }
    s_motor_test.active = 0U;
    if(s_active_cmd != NULL)
    {
        s_last_done_valid = 1U;
        s_last_done_seq = s_motor_test.seq;
        strncpy(s_last_done_name, s_active_cmd->name, AIR_COMM_AIR_FUNC_NAME_MAX);
        s_last_done_name[AIR_COMM_AIR_FUNC_NAME_MAX] = '\0';
    }
    s_active_cmd = NULL;
    if(send_ack != 0U)
    {
        (void)air_comm_air_send_func_ack_text(s_motor_test.seq, "ACK_EXIT_OK");
    }
}

static uint8 air_remote_cmd_motor_start(void)
{
    int32 throttle[MOTOR_NUM] = {0, 0, 0, 0};

    s_motor_test.active = 1U;
    s_motor_test.seq = s_active_seq;
    s_motor_test.step = 0U;
    s_motor_test.tick = 0U;
    s_motor_test.motor_was_enabled = Motor_IsEnabled();
    if(s_motor_test.motor_was_enabled == 0U)
    {
        Motor_Enable();
    }
    Motor_SetThrottleAll(throttle);
    air_remote_cmd_motor_output(0U);

    return 1U;
}

static void air_remote_cmd_motor_poll(void)
{
    if(s_motor_test.active == 0U)
    {
        return;
    }

    s_motor_test.tick++;
    if(s_motor_test.tick < AIR_REMOTE_CMD_MOTOR_HOLD_TICKS)
    {
        return;
    }

    s_motor_test.tick = 0U;
    s_motor_test.step++;

    if(s_motor_test.step < MOTOR_NUM)
    {
        air_remote_cmd_motor_output(s_motor_test.step);
    }
    else
    {
        air_remote_cmd_motor_finish(1U);
    }
}

static void air_remote_cmd_motor_stop(void)
{
    if(s_motor_test.active != 0U)
    {
        air_remote_cmd_motor_finish(0U);
    }
}

static void air_remote_cmd_stop_active(void)
{
    if((s_active_cmd != NULL) && (s_active_cmd->stop != NULL))
    {
        s_active_cmd->stop();
    }

    s_active_cmd = NULL;
    s_motor_test.active = 0U;
}

static uint8 air_remote_cmd_handle(uint8 seq, const char *name)
{
    const air_remote_cmd_entry_t *cmd;
    char ack[AIR_COMM_AIR_ACK_TEXT_MAX + 1U];

    if(name == NULL)
    {
        (void)air_comm_air_send_func_ack_text(seq, "ACK_ERROR 3 bad_name");
        return 0U;
    }

    if(strcmp(name, AIR_REMOTE_CMD_NONE) == 0)
    {
        air_remote_cmd_stop_active();
        (void)air_comm_air_send_func_ack_text(seq, "ACK_EXIT_OK");
        return 1U;
    }

    if((s_last_done_valid != 0U) &&
       (seq == s_last_done_seq) &&
       (strcmp(s_last_done_name, name) == 0))
    {
        (void)air_comm_air_send_func_ack_text(seq, "ACK_EXIT_OK");
        return 1U;
    }

    if(s_active_cmd != NULL)
    {
        if((seq == s_active_seq) && (strcmp(s_active_cmd->name, name) == 0))
        {
            snprintf(ack, sizeof(ack), "ACK_OK %s", s_active_cmd->name);
            (void)air_comm_air_send_func_ack_text(seq, ack);
            return 1U;
        }
        (void)air_comm_air_send_func_ack_text(seq, "ACK_ERROR 4 busy");
        return 0U;
    }

    cmd = air_remote_cmd_find(name);
    if(cmd == NULL)
    {
        (void)air_comm_air_send_func_ack_text(seq, "ACK_ERROR 1 not_found");
        return 0U;
    }

    s_active_cmd = cmd;
    s_active_seq = seq;
    s_last_done_valid = 0U;
    if((cmd->start != NULL) && (cmd->start() == 0U))
    {
        s_active_cmd = NULL;
        (void)air_comm_air_send_func_ack_text(seq, "ACK_ERROR 3 start_fail");
        return 0U;
    }

    snprintf(ack, sizeof(ack), "ACK_OK %s", cmd->name);
    (void)air_comm_air_send_func_ack_text(seq, ack);

    return 1U;
}

void air_remote_cmd_init(void)
{
    s_cmd_count = 0U;
    s_active_cmd = NULL;
    s_active_seq = 0U;
    memset(&s_motor_test, 0, sizeof(s_motor_test));
    memset(s_screen_cache, 0, sizeof(s_screen_cache));
    s_screen_ready = 0U;
    s_last_done_valid = 0U;
    s_last_done_seq = 0U;
    s_last_done_name[0] = '\0';

    (void)air_remote_cmd_register("show_imu_data",
                                  AIR_REMOTE_CMD_MODE_POLLING,
                                  air_remote_cmd_show_imu_start,
                                  air_remote_cmd_show_imu_poll,
                                  air_remote_cmd_screen_stop);
    (void)air_remote_cmd_register("show_optical_flow_data",
                                  AIR_REMOTE_CMD_MODE_POLLING,
                                  air_remote_cmd_show_flow_start,
                                  air_remote_cmd_show_flow_poll,
                                  air_remote_cmd_screen_stop);
    (void)air_remote_cmd_register("test_motors_pwm",
                                  AIR_REMOTE_CMD_MODE_INSTANT,
                                  air_remote_cmd_motor_start,
                                  air_remote_cmd_motor_poll,
                                  air_remote_cmd_motor_stop);

    air_comm_air_set_exec_command_callback(air_remote_cmd_handle);
}

void air_remote_cmd_update_100HZ(void)
{
    if((s_active_cmd != NULL) && (s_active_cmd->poll != NULL))
    {
        s_active_cmd->poll();
    }
}
