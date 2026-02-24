#ifndef BMP388_H_
#define BMP388_H_

#include "zf_common_headfile.h"

// 固定引脚 4线软件SPI
#define BMP388_PIN_SCK               (P13_2)
#define BMP388_PIN_MISO              (P13_3)
#define BMP388_PIN_MOSI              (P13_4)
#define BMP388_PIN_CS                (P13_5)

// 返回状态
#define BMP388_RET_OK                (0U)
#define BMP388_RET_ERR_NOT_INIT      (1U)
#define BMP388_RET_ERR_PARAM         (2U)
#define BMP388_RET_ERR_TIMEOUT       (3U)
#define BMP388_RET_ERR_CHIP_ID       (4U)
#define BMP388_RET_ERR_RESET         (5U)
#define BMP388_RET_ERR_CRC           (6U)
#define BMP388_RET_ERR_CONFIG        (7U)

// 校准参数结构体
// 仅保留温度和气压补偿所需参数
typedef struct
{
    uint16 par_t1;
    uint16 par_t2;
    int8   par_t3;

    int16  par_p1;
    int16  par_p2;
    int8   par_p3;
    int8   par_p4;
    uint16 par_p5;
    uint16 par_p6;
    int8   par_p7;
    int8   par_p8;
    int16  par_p9;
    int8   par_p10;
    int8   par_p11;

    int64  t_lin;
} BMP388_calib_t;

// 设备实例结构体
// 固定引脚 默认配置 运行状态
typedef struct
{
    gpio_pin_enum sck_pin;
    gpio_pin_enum miso_pin;
    gpio_pin_enum mosi_pin;
    gpio_pin_enum cs_pin;

    uint8  osr_t;
    uint8  osr_p;
    uint8  odr;
    uint8  iir_coef;

    uint32 measure_time_us;
    uint8  inited;
} BMP388_device_t;

// 实时数据结构体
// 原始值和补偿值
typedef struct
{
    uint32 raw_pressure;        // 原始气压值
    uint32 raw_temperature;     // 原始温度值
    float  pressure_pa;         // 补偿后气压值 单位Pa/
    float  temperature_c;       // 补偿后温度值 单位℃/
} BMP388_data_t;

extern volatile BMP388_calib_t  g_BMP388_calib;
extern volatile BMP388_device_t g_BMP388_dev;
extern volatile BMP388_data_t   g_BMP388_data;

// 初始化 GPIO 软件SPI 复位 ID校验 校准读取 配置写入
uint8 BMP388_init(void);

// 更新 触发测量 读取原始值 补偿计算 输出温度和气压
uint8 BMP388_update(void);
uint8 BMP388_update_nonblocking(uint8 *is_new_sample);

#endif /* BMP388_H_ */
