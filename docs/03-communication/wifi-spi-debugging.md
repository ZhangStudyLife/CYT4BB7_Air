# WiFi SPI 调试

> 状态：待补充正文。

## 文档目标

说明 WiFi SPI 调试链路的组成、数据类型和实际使用顺序。

## 内容提纲

- WiFi 模块连接与 SPI 配置
- 调试链路总体数据流
- 参数读取与在线修改
- 浮点日志上传
- 图像数据传输
- 命令通道
- IMU 标定辅助功能
- 上位机连接和典型调试流程
- 带宽、阻塞和实时性注意事项
- 常见连接问题

## 为什么需要wifispi?

wifispi不是必须的,但是你必须得有某种手段,可以获取无人机运行时的中间变量,而且越高频越原始越好,同时尽量得做到非侵入式,尽可能少的堵占cpu,堵占时间片!!! 要知道我们调的飞机是底层的控制相关的,直接开发mcu的,而不是基于成品飞控二开的,所以更加需要注意实时性和时刻谨记cpu的性能是有限的 , 调的时候就要注意 1.各个通信耗时多少,驱动程序是否还可以优化 2. 算法耗时多少,是否会对其他实时性更高的产生堵塞呢 3.如果没有手段可以知道他内部拿到了什么原始传感器数据和控制器的输入输出,那么你的算法只是一个盲盒系统 , 从结果的现象去推算算法本身是否合理 这个是非常低级的. 前期一定得准备好 高效 , 方便 , 快捷的调试手段 , 后面才方便进行调试


## 针对于高频数据非堵塞,驱动相比较原生逐飞库,优化了哪些

参考代码  CYT4BB7_Air\project\code\Protocols\wifi 以及 **CYT4BB7_Air\libraries\zf_device\zf_device_wifi_spi.c**

做了很多的魔改 


## 走什么协议

我使用的是udp协议 , 为什么不选择tcp协议,你们可以搜索一下这两个协议有什么区别 再结合上述我的需求描述 , udp初始化的时候简单 , 不管我电脑开没开机,有没有打开上位机,他只管发,不管我收没收到,就不会产生堵塞问题 也没有握手阶段,适合发送1khz的高频数据

平常发送日志,我只需要管 调用 `wifi_justfloat` 即可 , 类似于你可以在我的代码随处可见(也许发布版本那些散落注释的 `wifi_justfloat` 我都给删除了),类似于

```
   wifi_justfloat(image_data[Front].beacon_data[0].x,     /* I1 */
                   image_data[Front].beacon_data[0].y,     /* I2 */
                   image_data[Front].beacon_data[1].x,     /* I3 */
                   image_data[Front].beacon_data[1].y,     /* I4 */
                   image_data[Front].beacon_data[2].x,     /* I5 */
                   image_data[Front].beacon_data[2].y,     /* I6 */
                   image_data[Front].car_lamp_data[0].cx,  /* I7 */
                   image_data[Front].car_lamp_data[0].cy,  /* I8 */
                   image_data[Front].car_lamp_data[1].cx,  /* I9 */
                   image_data[Front].car_lamp_data[1].cy,  /* I10 */
                   image_data[Center].beacon_data[0].x,    /* I11 */
                   image_data[Center].beacon_data[0].y,    /* I12 */
                   image_data[Center].beacon_data[1].x,    /* I13 */
                   image_data[Center].beacon_data[1].y,    /* I14 */
                   image_data[Center].beacon_data[2].x,    /* I15 */
                   image_data[Center].beacon_data[2].y,    /* I16 */
                   image_data[Center].car_lamp_data[0].cx, /* I17 */
                   image_data[Center].car_lamp_data[0].cy, /* I18 */
                   image_data[Center].car_lamp_data[1].cx, /* I19 */
                   image_data[Center].car_lamp_data[1].cy, /* I20 */
                   image_data[Back].beacon_data[0].x,      /* I21 */
                   image_data[Back].beacon_data[0].y,      /* I22 */
                   image_data[Back].beacon_data[1].x,      /* I23 */
                   image_data[Back].beacon_data[1].y,      /* I24 */
                   image_data[Back].beacon_data[2].x,      /* I25 */
                   image_data[Back].beacon_data[2].y,      /* I26 */
                   image_data[Back].car_lamp_data[0].cx,   /* I27 */
                   image_data[Back].car_lamp_data[0].cy,   /* I28 */
                   image_data[Back].car_lamp_data[1].cx,   /* I29 */
                   image_data[Back].car_lamp_data[1].cy,   /* I30 */
                   (float)yaw_align_active,                /* I31 */
                   (float)yaw_debug.locked,                /* I32 */
                   (float)yaw_debug.locked_beacon.camera,  /* I33 */
                   yaw_debug.locked_beacon.x,              /* I34 */
                   yaw_debug.locked_beacon.y,              /* I35 */
                   g_euler.yaw,                            /* I36 */
                   yaw_angle_target,                       /* I37 */
                   yaw_gyro_target,                        /* I38 */
                   yaw_gyro_pid.output);                   /* I39 */
```

这样就可以实现不管多少变量的发送,上位机选择[www.vofa.plus](https://www.vofa.plus/) 或者自己写上位机 然后协议兼容 justfloat 这样自定义更方便

像类似于原始imu的频域分析什么的,car_plan的路径规划,需要发送3个摄像头的信标坐标和车灯坐标角度等 都离不开这个wifispi

[返回总览](../../README.md)
