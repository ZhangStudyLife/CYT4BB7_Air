# 三摄图像与核心0日志回放工具

这个工具用于录制2BL3输出的 `BIMG v3` 原始灰度图，并按来源帧号与核心0的 `I0..I35` CSV逐帧配准。它不会修改图像算法、SPI通信或飞控输出。

## 采集前准备

- Python 3.10或更高版本，并安装 `Pillow`：`python -m pip install Pillow`
- 电脑有线网卡固定为 `192.168.110.30`
- 前摄WiFi图传连接电脑TCP端口 `8086`；后摄使用 `8087`
- 2BL3图传内容模式设为 `RAW(0)`，这是算法实际使用的锁存原图
- Air侧图传允许值设为 `2` 时会持续发送，设为 `1` 时只在非飞行状态发送

必须保留 `.bimg` 原始文件。MP4经过编码且不包含完整来源帧号，不能用于与核心0 CSV做可靠的逐帧对应。

图传约为1.13 MB/s，并由2BL3同步发送。正式采集前先录10至20秒，确认图像来源帧率仍不低于47 FPS；飞行时只连接需要录制的那一路图传。

## 启动图形界面

在PowerShell中运行：

```powershell
& "CYT4BB7_Air/tools/image_log_review/run_tool.ps1"
```

也可以进入 `CYT4BB7_Air/tools` 后运行：

```powershell
python -m image_log_review
```

录制页选择保存路径并开始监听，然后给2BL3和Air上电。工具会原样保存TCP字节到 `.bimg`，同时生成 `.bimg.jsonl` 接收索引。停止录制后，再到回放页依次打开 `.bimg` 和同一时段的核心0 CSV。

配准成功后可逐帧查看：

- 原始188×120灰度图
- 本摄车灯位置、宽度、长度和角度
- 本摄前两个信标的位置与面积
- 对应日志时刻的公共轨迹投影点及8/12像素核心门
- `I32`中的轨迹、支持来源、ROI命中、冲突、回退和实际模式

低置信度配准会明确警告。此时不能直接根据叠加画面修改算法，应检查画面姿态是否对应；必要时填写128整数倍的“周期偏移”后重新配准。

## 命令行批处理

```powershell
# 录制前摄原始流，按Ctrl+C停止
python -m image_log_review capture "E:/Desktop/front_run.bimg" --port 8086

# 导出BIMG帧索引
python -m image_log_review index "E:/Desktop/front_run.bimg"

# 自动配准并导出包含全部I0..I35的逐帧CSV
python -m image_log_review align "E:/Desktop/front_run.bimg" "E:/Desktop/core0.csv"

# 自动配准不唯一时手动指定一个128帧周期偏移
python -m image_log_review align "E:/Desktop/front_run.bimg" "E:/Desktop/core0.csv" --offset 896
```

## 推荐同步采集步骤

1. 启动BIMG录制，确认界面已经显示“图像板已连接”且FPS稳定。
2. 启动核心0 CSV记录，静止保持约5秒。
3. 开始计划场景，并记下明显事件的大致时刻，例如开始转Yaw或信标熄灭。
4. 场景结束后再静止保持约5秒，先停止核心0 CSV，再停止BIMG录制。
5. 保存原始 `.bimg`、`.bimg.jsonl` 和CSV；视频只作为人工场景参考，可额外提供但不能替代BIMG。

目前核心0 CSV只记录公共轨迹坐标与下摄ROI半尺寸，因此工具不会虚构前摄或后摄的完整动态ROI矩形。`I31`是核心1在对应日志时刻发布的公共轨迹，不等于这张BIMG在识别前实际读取的ROI快照；界面只绘制该日志时刻的公共预计点及固件现有几何匹配核心门。
