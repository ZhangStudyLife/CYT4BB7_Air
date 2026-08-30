# CYT4BB7 Air 飞控

> 这是第 21 届全国大学生智能汽车竞赛“飞跃雷区”项目的空中控制端子仓库。

> **仓库关系先说清楚：**本仓库的母仓库是 [HDUASC-SmartCar-21st-FlyOverMinefield](https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield)。总仓库固定的 Air 版本可以从母仓库进入；想看 Air 子仓库自己的最新文档和提交，请以本仓库当前分支为准。

PCB 源文件、板卡照片和硬件迭代记录不在这里维护，请直接前往母仓库的[硬件 PCB 总文档](https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield/blob/national-2026/hardware/README.md)。这里最多保留硬件引脚和软件使用方式，不复制一份 PCB 文件。

## 先看这里

- [待填写：Air 飞控或比赛系统演示视频]
- [母仓库总 README](https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield/blob/national-2026/README.md)
- [CarPlan3 上位机调试视频](https://www.bilibili.com/video/BV1Rm4m6fEMv/)

如果第一次接触这个项目，建议先看母仓库的比赛背景和整体方案，再回到这里按下面的阅读路线理解 Air。不要一上来就从某个 `.c` 文件开始看，很容易只看见局部实现，却不知道它在空地协同系统里负责什么。

## 按目标阅读

| 我想了解 | 建议入口 |
| --- | --- |
| PCB、板卡和硬件迭代 | 母仓库的[硬件 PCB 总文档](https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield/blob/national-2026/hardware/README.md) |
| 双核飞控如何运行 | [软件架构](docs/01-flight-control/software-architecture.md) |
| IMU、姿态解算和滤波 | [IMU 与姿态估计](docs/01-flight-control/imu-and-attitude.md) |
| 四路 TOF 定高 | [高度估计与控制](docs/01-flight-control/height-estimation-and-control.md) |
| 遥控器和 CRSF 飞行模式 | [遥控器与飞行模式](docs/01-flight-control/rc-and-flight-modes.md) |
| Air 和 Car 怎么通信 | [AirComm 空地通信](docs/02-communication/aircomm.md) |
| 图像板和双核 IPC | [Camera SPI 与双核 IPC](docs/02-communication/camera-spi-and-ipc.md) |
| WiFi SPI 怎么发日志 | [WiFi SPI 调试](docs/02-communication/wifi-spi-debugging.md) |
| 多摄像头和相机模型 | [相机模型标定](docs/03-competition/camera-model-calibration.md) |
| CarPlan3 和上位机联调 | [CarPlan3 调试流程](docs/03-competition/car-plan3-debug-workflow.md) |
| 代码目录和模块边界 | [仓库结构](docs/04-engineering/repository-structure.md) |
| 已知问题和排查记录 | [故障排查索引](docs/04-engineering/troubleshooting.md) |

## 推荐阅读路线

### 只想理解飞控

[软件架构](docs/01-flight-control/software-architecture.md) -> [IMU 与姿态估计](docs/01-flight-control/imu-and-attitude.md) -> [高度估计与控制](docs/01-flight-control/height-estimation-and-control.md) -> [遥控器与飞行模式](docs/01-flight-control/rc-and-flight-modes.md)

### 想理解空地协同

[软件架构](docs/01-flight-control/software-architecture.md) -> [AirComm](docs/02-communication/aircomm.md) -> [Camera SPI 与双核 IPC](docs/02-communication/camera-spi-and-ipc.md) -> [WiFi SPI 调试](docs/02-communication/wifi-spi-debugging.md)

### 想理解图像到车模速度

[Camera SPI 与双核 IPC](docs/02-communication/camera-spi-and-ipc.md) -> [相机模型标定](docs/03-competition/camera-model-calibration.md) -> [CarPlan3 调试流程](docs/03-competition/car-plan3-debug-workflow.md) -> 母仓库的 [Car 端工程](https://github.com/choumouing/CYT4bb7_Car/)

## 代码目录地图

```text
project/user/                 两个核心的入口和主循环调度
project/code/Estimation/      IMU、姿态、高度和位置估计
project/code/FlightController/飞行模式、姿态/位置控制和自动降落
project/code/Planner/         相机模型、三摄融合和 CarPlan3/4
project/code/Protocols/       AirComm、Camera SPI、WiFi SPI 等通信
project/code/IPC/             双核之间的 image_data 和参数快照
project/code/Image/           图像结果结构体及有效性判断
libraries/                    芯片、逐飞库和外设驱动
doc/                          早期设计稿和具体故障原始记录
docs/                         面向开源阅读整理后的正式文档
```

代码阅读时，建议从 [`project/user/main_cm7_0.c`](project/user/main_cm7_0.c) 开始，再根据目标进入 `Estimation`、`FlightController`、`Planner` 或 `Protocols`。`docs/` 是解释思路和实验过程的入口，`doc/` 里的内容更偏历史记录；两者不要混为同一套最终规范。

## 文档状态

- `docs/01-flight-control` 到 `docs/04-engineering` 是面向开源阅读的正式入口，硬件相关内容统一跳转母仓库。
- `doc/` 保留早期通信设计和故障原始记录，后续只通过正式文档建立跳转，不随意删除历史材料。
- 文档中的参数、波形和分析结论应与对应 Git 提交和源码版本一起理解。
- [待填写：哪些模块是最终比赛版本，哪些模块只是实验版本。]

## 返回路径

- [返回母仓库总 README](https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield/blob/national-2026/README.md)
- [返回母仓库硬件 PCB 总文档](https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield/blob/national-2026/hardware/README.md)
- [查看 Air 子仓库最新代码](https://github.com/ZhangStudyLife/CYT4BB7_Air/tree/national-2026)
