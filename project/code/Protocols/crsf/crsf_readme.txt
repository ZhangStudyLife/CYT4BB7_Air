CRSF (ELRS) 最简说明

1) 代码功能
- �?UART2 (P10.0 RX, P10.1 TX) 上以 420000 波特率接�?CRSF RC 帧�?- 解析 CH1~CH10 �?rc_ch[10]�?~2047 原始值）�?
- 更新 rc_last_update_time（单�?us）�?
- 包含帧同步、长度与 CRC 校验�?
- 姿态角回传（roll/pitch/yaw），25Hz 发送�?
- BARO 高度回传接口预留（dm = 0.1m）�?

2) 代码使用方式
- 启动时调用一�?crsf_init()�?
- 10ms 定时器中调用 CRSF_Update_100HZ()（轮询硬�?FIFO，不开中断）�?
- 25Hz 调用 crsf_send_25hz()：内部使�?g_euler，自动将角度转换�?rad * 10000 并回传标准姿态�?
  - 标准单位：rad * 10000�?.0001 rad/bit），payload 为大端序�?
  - 参考：180deg = pi rad，pi * 10000 �?31416�?
- BARO 高度：crsf_send_height(height_dm) 为预留接口，单位 dm(0.1m)，当前在 crsf_send_25hz 中调用，默认发�?0�?
- 失控示例（用户逻辑）：if (timer_get(TC_TIME2_CH0) - rc_last_update_time > 100000) 判定信号丢失�?

3) ELRS 接收机配置（不使�?Betaflight�?
- 上电让接收机进入 WiFi 模式（通常快速上�?3 次）�?
- 手机/电脑连接 WiFi "ExpressLRS RX"，打开 10.0.0.1�?
- 设置 UART 波特率为 420000，协议选择 CRSF�?
- 保存并重启�?
- 若需要回传，需自行实现 CRSF TX 帧�?

4) Radiomaster Pocket（ELRS）配�?
- 新建模型，RF 协议�?CRSF�?
- Packet Rate �?150Hz �?250Hz�?
- ELRS 模式�?Internal（如用外置模块则�?External）�?
- Telemetry 设为 On�?
- 绑定方式�?
  - 使用绑定短语时，TX/RX �?Web 配置中填同一短语�?
  - 使用按键绑定时，先让 RX 进入绑定（LED 快闪），再在 TX 端绑定�?
- 在固件中观察 rc_ch[] 是否有变化�?

5) 遥测脚本与屏幕显示（EdgeTX / OpenTX 常见菜单�?
- 标准姿态（rad）：�?Telemetry �?Discover sensors，会出现 Roll/Pitch/Yaw，Nums/Bars 页面可直接显示�?
- BARO 高度：若接收机识�?BARO 类型，将会出现对应高度传感器条目；遥控器 ratio 设为 0.1 可直接显示米�?

6) 适配与优化建议（Plan�?
- 硬件外设：使用同一 UART 420000 波特率，RX 轮询 FIFO，TX 阻塞发送，确保不与其他高速外设争用�?
- 发送机制：25Hz 固定周期发送姿态，避免高频占用；必要时与控制回路分离�?
- 处理逻辑：仅发送必要帧类型（Attitude），减少 CRC 计算与串口带宽占用�?
- 兼容性：遵循 CRSF 帧格式与 CRC8 规则，保持与主流飞控一致�?
- 扩展：若需更多遥测，再增加电池、电流、GPS 等帧类型，保持低速发送�?

