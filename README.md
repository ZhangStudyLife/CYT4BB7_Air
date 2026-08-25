# CYT4BB7 Air 飞控

本仓库为第二十一届全国大学生智能汽车竞赛“飞越雷区”项目的空中控制端代码。

> 文档状态：目录框架已初始化，技术正文将按实际代码、实验记录和比赛经验逐步补充。

## 按目标阅读

| 我想了解 | 建议入口 |
| --- | --- |
| 硬件、引脚和外设 | [硬件与引脚分配](docs/01-hardware/hardware-and-pinout.md) |
| 编译和烧录工程 | [编译与烧录](docs/01-hardware/build-and-flash.md) |
| 双核飞控软件如何运行 | [软件架构](docs/02-flight-control/software-architecture.md) |
| IMU、姿态解算和滤波 | [IMU 与姿态估计](docs/02-flight-control/imu-and-attitude.md) |
| 四路 TOF 定高 | [高度估计与控制](docs/02-flight-control/height-estimation-and-control.md) |
| 遥控器和飞行模式 | [遥控器与飞行模式](docs/02-flight-control/rc-and-flight-modes.md) |
| Air、Image、Car 如何通信 | [通信总览](docs/03-communication/communication-overview.md) |
| WiFi SPI 如何调试 | [WiFi SPI 调试](docs/03-communication/wifi-spi-debugging.md) |
| 多摄像头和相机模型 | [多摄像头系统](docs/04-competition/multi-camera-system.md) |
| CarPlan3 和上位机联调 | [CarPlan3 调试流程](docs/04-competition/car-plan3-debug-workflow.md) |
| 备赛方案如何演进 | [方案迭代](docs/04-competition/solution-evolution.md) |
| 代码目录和模块边界 | [仓库结构](docs/05-engineering/repository-structure.md) |

## 推荐阅读路线

### 通用飞控

硬件与引脚 -> 软件架构 -> IMU 与姿态估计 -> 高度估计与控制 -> 遥控器与飞行模式 -> 通信总览

### 国赛系统

软件架构 -> 通信总览 -> 多摄像头系统 -> 相机模型标定 -> CarPlan3 -> 上位机调试 -> 方案迭代

## 文档目录

- [硬件与工程](docs/01-hardware/)
- [飞控核心](docs/02-flight-control/)
- [通信与调试](docs/03-communication/)
- [国赛专项](docs/04-competition/)
- [工程实践](docs/05-engineering/)

## 当前说明

- 文档中的参数、波形和分析结论应与实际锁存代码对应。
- IMU、定高和相机标定文档将保留最终频域分析图、对比图和测试结论，不恢复全量临时日志与中间产物。
- 已知协议设计和故障记录暂保留在 [`doc/`](doc/) 目录，后续由新文档统一建立入口。
