#ifndef PMW3901_H_
#define PMW3901_H_

#include "zf_common_headfile.h"

#ifndef __packed
#define __packed __PACKED                     /* 字节对齐封装宏，兼容不同编译器 */
#endif

/* ======================== SPI硬件引脚配置 ======================== */
#ifndef PMW3901_SPI
#define PMW3901_SPI              (SPI_3)             /* 使用SPI3外设 */
#endif
#ifndef PMW3901_MOSI_PIN
#define PMW3901_MOSI_PIN         (SPI3_MOSI_P03_1)   /* 主出从入引脚 P03_1 */
#endif
#ifndef PMW3901_MISO_PIN
#define PMW3901_MISO_PIN         (SPI3_MISO_P03_0)   /* 主入从出引脚 P03_0 */
#endif
#define PMW3901_SCK_PIN          (SPI3_CLK_P03_2)    /* 时钟引脚 P03_2 */
#define PMW3901_CS_Pin           (P03_3)             /* 片选引脚 P03_3，软件控制 */
#ifndef PMW3901_SPI_SPEED
#define PMW3901_SPI_SPEED        (2 * 1000 * 1000)   /* SPI通信速率 2MHz */
#endif

/* ======================== 光流极性映射 ======================== */
#define PMW3901_SIGN_X           (1)                 /* X轴极性：芯片X方向与机体X方向同向 */
#define PMW3901_SIGN_Y           (1)                /* Y轴极性：芯片Y方向与机体Y方向反向 */

/* 寄存器写入校验开关：0=直接写入不验证（内部寄存器多为非公开寄存器，回读值可能与写入值不同） */
#define PMW3901_VERIFY_WRITES    (0)








/**
 * @brief PMW3901 Motion Burst 原始数据结构体
 *        对应芯片 Burst Read 返回的 12 字节数据帧，按字节紧凑排列
 */
typedef __packed struct motionBurst_s
{
    /* 运动检测寄存器，包含运动标志和工作模式位域 */
    __packed union
    {
        uint8 motion;                       /* 运动寄存器原始值 */
        __packed struct
        {
            uint8 frameFrom0    : 1;        /* 帧数据是否来自复位后首帧 */
            uint8 runMode       : 2;        /* 运行模式(00=正常) */
            uint8 reserved1     : 1;        /* 保留位 */
            uint8 rawFrom0      : 1;        /* 原始数据是否来自首帧 */
            uint8 reserved2     : 2;        /* 保留位 */
            uint8 motionOccured : 1;        /* 运动发生标志，1=检测到运动 */
        };
    };

    uint8 observation;                      /* 帧观测累计值，用于内部诊断 */

    int16 deltaX;                           /* X方向像素位移增量（芯片坐标系） */
    int16 deltaY;                           /* Y方向像素位移增量（芯片坐标系） */

    uint8 squal;                            /* 表面质量指标，0~255，越大质量越好 */
    uint8 rawDataSum;                       /* 原始图像数据和，用于表面评估 */
    uint8 maxRawData;                       /* 原始图像最大值 */
    uint8 minRawData;                       /* 原始图像最小值 */

    uint16 shutter;                         /* 曝光时间（大端序传输，驱动内做字节交换） */
} pmw3901_raw_t;

/* PMW3901光流传感器原始数据全局变量，由PMW3901_Update_50HZ()更新，已做极性映射 */
extern volatile pmw3901_raw_t g_pmw3901_raw;

/**
 * @brief  PMW3901光流传感器初始化
 *         执行SPI初始化、芯片复位、ID校验、两阶段寄存器配置
 *
 * @param  无
 * @return 0=初始化成功，1=初始化失败（ID校验或寄存器配置失败）
 */
uint8 PMW3901_Init(void);

/**
 * @brief  PMW3901重新初始化
 *         先清零原始数据和初始化标志，再调用PMW3901_Init()完成完整初始化
 *
 * @param  无
 * @return 0=重新初始化成功，1=失败
 */
uint8 PMW3901_ReInit(void);

/**
 * @brief  PMW3901数据更新（需周期性调用，典型50Hz）
 *         执行Burst Read读取运动数据，并对deltaX/deltaY施加极性映射
 *         结果写入全局变量 g_pmw3901_raw
 *
 * @param  无
 * @return 无
 */
void PMW3901_Update_50HZ(void);

#endif /* PMW3901_H_ */
