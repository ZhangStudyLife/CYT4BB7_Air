# wifi_params 使用说明

## 1. 模块定位

`wifi_params` 是当前工程里的 WiFi 文本调参模块，目录位置：

- `project/code/Protocols/wifi_params`

它的职责只有一个：

- 通过现有 `wifi_spi + UDP` 链路，接收**人工可读文本命令**
- 在**待机且未解锁**时安全地读取、修改、保存飞控参数

当前版本已经**完全取消**旧的 `$WP,...*校验` 协议。

现在只支持人工可读命令：

- `ping`
- `help`
- `get base_throttle`
- `set base_throttle 3600`
- `set roll_angle_kp 5.5`
- `start`
- `stop`
- `save`
- `load`
- `list`

也就是说，你不再需要：

- 自己写命令序号
- 自己计算校验和
- 自己拼接 `$WP,...*xx`

---

## 2. 当前系统里的关系

当前工程中：

- `wifi_vofa` 继续负责下行 VOFA `JustFloat` 浮点数据回传
- `wifi_params` 负责上行文本命令接收与文本回包
- 二者共用同一个底层 `wifi_spi` UDP socket
- `stop` 只停止待机态下的 `wifi_vofa_JustFloat` 遥测，不影响文本命令回包
- 飞行态下 `wifi_vofa_JustFloat` 始终允许发送，不受 `start / stop` 控制

这意味着：

- 文本回包和 VOFA 浮点帧会在同一个 UDP 通道里混发

这是当前版本的**明确设计**，不是 bug。

### 2.1 这会带来什么影响

如果你同时开着：

- VOFA 监听这个 UDP 端口
- 文本调试助手也监听这个 UDP 端口

那么：

- VOFA 在收到文本回包时，可能会把这些文本当成脏帧
- 文本工具会看到普通文本回包

所以当前推荐的使用方式是：

- 把 `wifi_params` 当成“人工调参控制台”
- 允许 VOFA 在你发命令时偶发出现一两帧异常解析

当前版本**没有**做以下隔离：

- 不暂停 VOFA
- 不分第二个 socket
- 不单独开第二个端口

这样做的原因是保持实现简单、稳定、易维护。

---

## 3. 当前代码结构

### 3.1 相关文件

- `project/code/Protocols/wifi_params/wifi_params.h`
- `project/code/Protocols/wifi_params/wifi_params.c`
- `project/code/FlightController/fc_params.h`
- `project/code/FlightController/fc_params.c`
- `project/user/main_cm7_0.c`

### 3.2 各文件职责

#### `wifi_params.h / wifi_params.c`

负责：

- 从 `wifi_spi_read_buffer()` 读取上行文本
- 按行解析命令
- 白名单匹配参数名
- 范围检查
- 修改 `g_fc_params`
- 必要时调用 `FC_Loop_Init()` 刷新 PID
- 发送文本回包
- 维护最近一次命令诊断信息

#### `fc_params.h / fc_params.c`

负责：

- 定义参数结构 `fc_params_t`
- 提供默认参数
- 提供 Flash 持久化加载/保存

#### `main_cm7_0.c`

负责：

- 上电时先装默认参数，再尝试加载 Flash 参数
- 初始化 `wifi_params`
- 后台循环持续调用 `wifi_params_Poll()`
- 通过 VOFA 多回传 4 个诊断通道

---

## 4. 上电启动流程

当前关键启动顺序如下：

1. `FC_Params_Init()`
2. `FC_Params_LoadFromFlash()`
3. `Pos_Est_Init()`
4. `FC_Loop_Init()`
5. `wifi_params_Init()`

### 4.1 `FC_Params_Init()` 做什么

作用：

- 把编译期默认值写入 `g_fc_params`

这一步保证：

- 即使 Flash 没保存过参数
- 即使 Flash 数据损坏
- 即使版本不兼容

系统仍然有一套可用的默认参数。

### 4.2 `FC_Params_LoadFromFlash()` 做什么

作用：

- 从 Flash 固定页读取上次保存的参数块

当前使用页号：

- `FC_PARAMS_FLASH_PAGE = 94`

它会校验：

- `magic`
- `version`
- `size`
- `checksum`

如果这些检查全部通过：

- Flash 里的参数会覆盖默认参数

如果任何一项失败：

- 加载失败
- 保留默认参数
- 系统继续正常启动

### 4.3 `FC_Loop_Init()` 为什么重要

`g_fc_params` 只是参数存储区。

很多 PID 和模式控制器在初始化时，会把参数从 `g_fc_params` 拷进运行态结构体。

所以单纯修改 `g_fc_params` 不够。

当前实现规定：

- `set` 成功后置位 `pending_apply`
- `load` 成功后置位 `pending_apply`
- 在 `wifi_params_Poll()` 尾部，如果当前仍处于允许状态，则统一调用一次 `FC_Loop_Init()`

这能保证：

- 主控制环 PID 刷新
- `fc_mode1` 刷新
- `fc_mode2` 刷新

---

## 5. 参数是临时的还是永久的

这个必须分清。

### 5.1 `set` 是临时修改

例如：

```text
set base_throttle 3600
```

它的效果是：

- 修改 RAM 中的 `g_fc_params`
- 成功后重新应用控制器参数
- 当前运行立即生效

但是：

- **不会自动写 Flash**
- **断电后不会保留**

结论：

- `set` = 临时调参

### 5.2 `save` 才是永久保存

例如：

```text
save
```

它的效果是：

- 把当前 RAM 中的全部参数写入 Flash

结论：

- `save` = 永久保存

### 5.3 `load` 是恢复已保存参数

例如：

```text
load
```

它的效果是：

- 从 Flash 读回最后一次成功 `save` 的参数
- 成功后重新应用到控制器

结论：

- `load` = 恢复最后一次保存值

### 5.4 断电后保留的到底是什么

断电后保留的是：

- **最后一次 `save` 成功时的参数**

不是：

- 最后一次 `set` 的参数

---

## 6. 从未保存过参数时会发生什么

这是第一次上电时最常见的情况。

### 6.1 上电加载会怎样

如果这块板子从未执行过 `save`：

- `FC_Params_LoadFromFlash()` 会读取第 94 页
- 但那里面没有合法参数块
- 校验不会通过
- 函数返回失败

### 6.2 系统会不会异常

不会。

因为默认值已经先写进了 `g_fc_params`，所以加载失败后：

- 保持默认参数
- 继续正常启动
- 不会串位
- 不会随机读垃圾数据

### 6.3 这时候手工发 `load` 会怎样

如果从未保存过参数，此时发送：

```text
load
```

当前系统会文本回包：

```text
ERR flash
```

含义是：

- Flash 中没有合法参数块可读

同时：

- 当前 RAM 里的参数保持不变

---

## 7. 如果读取错误会怎样

这里分两类。

### 7.1 Flash 读取失败或数据非法

包括：

- 从未保存过参数
- Flash 被擦除
- Flash 区域被污染
- 旧参数版本不兼容
- 校验和错误

#### 上电阶段的行为

- `FC_Params_LoadFromFlash()` 失败
- 保留默认参数
- 系统继续运行

#### `load` 命令阶段的行为

- 文本回包 `ERR flash`
- 不修改当前 RAM 参数
- 不刷新控制器

### 7.2 文本命令格式错误

包括：

- 命令字错误
- 参数个数不对
- 参数名不存在
- 数值解析失败
- 行过长
- 行中有非法字符

行为：

- 文本回包 `ERR format` / `ERR unknown command` / `ERR unknown param`
- 不修改任何参数
- 不写 Flash

---

## 8. `magic / version / size / checksum` 是干什么的

当前 Flash 里保存的不是裸 `fc_params_t`，而是一个带头部的参数块。

### 8.1 `magic`

作用：

- 判断这页 Flash 里存的是不是飞控参数块

如果不匹配：

- 说明这里不是本模块写入的数据
- 本次加载直接判无效

### 8.2 `version`

作用：

- 区分参数结构版本

当前版本号：

- `FC_PARAMS_FLASH_VERSION = 1`

这个值不是程序版本，而是：

- **Flash 参数布局版本**

### 8.3 `size`

作用：

- 记录当前 `fc_params_t` 结构体大小

如果你以后增删字段导致结构体大小变化：

- 旧数据会因为 `size` 不匹配而失效
- 系统回退默认值

### 8.4 `checksum`

作用：

- 检查参数内容有没有损坏

当前使用：

- FNV-1a 32 位校验

### 8.5 什么时候要改版本号

只要你改了 `fc_params_t` 的布局或语义，建议递增版本号。

包括：

- 新增字段
- 删除字段
- 调整字段顺序
- 改字段类型
- 改字段含义

这样旧 Flash 参数会自动失效，避免“旧值被错误解释”。

---

## 9. 当前支持的命令

### 9.1 `ping`

作用：

- 检查链路是否通

命令：

```text
ping
```

成功回包：

```text
OK ping
```

### 9.2 `help`

作用：

- 输出当前支持的命令帮助

命令：

```text
help
```

典型回包：

```text
topic: general
usage: help [cmd]
usage: <cmd> --help
cmds: ping help get set save load start stop list
state: get/set/save/load/start/stop/list standby
note: commands must end with CRLF
OK help
```

也支持：
```text
help list
list --help
set --help
```

### 9.3 `get <name>`

作用：

- 读取某个参数当前值

示例：

```text
get base_throttle
```

成功回包示例：

```text
OK base_throttle=3600
```

### 9.4 `set <name> <value>`

作用：

- 修改某个参数当前值

示例：

```text
set base_throttle 3600
set roll_angle_kp 5.5
```

成功回包示例：

```text
OK set base_throttle=3600
OK set roll_angle_kp=5.5
```

### 9.5 `save`

作用：

- 把当前参数写入 Flash

命令：

```text
save
```

成功回包：

```text
OK save
```

### 9.6 `load`

作用：

- 从 Flash 读回最后一次保存的参数

命令：

```text
load
```

成功回包：

```text
OK load
```

### 9.7 `list`

作用：

- 一次性输出全部白名单参数及当前值

命令：

```text
list
```

回包格式固定为：

```text
OK list begin count=76
base_throttle=3600
roll_gyro_kp=3
roll_gyro_ki=0.8
...
mode1_brake_exit_vel_cmps=10
OK list end
```

说明：

- 第一行表示开始和总数
- 中间每行一个 `name=value`
- 最后一行表示结束

这样人工看最直接，也能让上位机脚本知道何时接收完成。

---

### 9.8 `start`

作用：
- 恢复待机态下的 `wifi_vofa_JustFloat` 遥测

命令：
```text
start\r\n
```

成功回包：
```text
OK start telemetry=on\r\n
```

### 9.9 `stop`

作用：
- 停止待机态下的 `wifi_vofa_JustFloat` 遥测
- 继续保留文本 ACK / list / get / set / save / load 回包

命令：
```text
stop\r\n
```

成功回包：
```text
OK stop telemetry=off\r\n
```

说明：
- 仅在 `STANDBY` 且未解锁时允许执行
- 掉电后不保留，重新上电默认恢复允许发送

## 10. 命令输入规则

当前输入规则固定如下：

- 命令字与参数名大小写不敏感
- 以一整行作为一条命令
- 必须以 `\r\n` 结束
- 行首行尾空格会被忽略
- 参数之间允许一个或多个空格或制表符

### 10.1 合法示例

```text
ping\r\n
help\r\n
get base_throttle\r\n
set base_throttle 3600\r\n
set roll_angle_kp 5.5\r\n
start\r\n
stop\r\n
save\r\n
load\r\n
list\r\n
```

### 10.2 非法示例

```text
PING\r\n
get\r\n
set base_throttle\r\n
set base_throttle abc\r\n
set unknown_param 1\r\n
list now\r\n
ping\n
stop\r
```

这些会收到错误回包。

---

## 11. 回包格式规范

### 11.1 普通成功回包

风格固定为：

- `OK ...`

示例：

```text
OK ping
OK save
OK load
OK base_throttle=3600
OK set roll_angle_kp=5.5
```

### 11.2 错误回包

风格固定为：

- `ERR ...`

当前可能出现：

```text
ERR format
ERR unknown command
ERR unknown param
ERR range
ERR state
ERR flash
```

### 11.3 `list` 回包

固定是三段式：

1. `OK list begin count=<N>`
2. 多行 `name=value`
3. `OK list end`

---

## 12. 只有待机时才能改，飞行中为什么不允许

当前门控规则：

- `set`
- `save`
- `load`
- `start`
- `stop`
- `get`
- `list`

只有在下面条件同时满足时才允许：

- `FC_START_CRSF_Get_State() == FC_START_CRSF_STATE_STANDBY`
- `FC_START_CRSF_Is_Armed() == 0`

### 12.1 飞行中还能用什么

当前只有这两个命令在任何状态都允许：

- `ping`
- `help`

### 12.2 飞行中发改参命令会怎样

例如飞行中发：

```text
set base_throttle 3600
```

会收到：

```text
ERR state
```

同时：

- 参数不会变化
- 不会保存
- 不会落地后补执行

### 12.3 为什么这么设计

因为 `set` 和 `load` 成功后，会触发一次 `FC_Loop_Init()`，这会重装 PID。

这种行为在飞行中不可接受，所以必须强制只允许待机态执行。

---

## 13. 当前开放的参数

当前不是所有 `fc_params_t` 成员都开放。

系统采用的是：

- **静态白名单**

只有白名单里的参数名才能被 `get / set / list` 访问。

### 13.1 已开放参数

#### 基础油门

- `base_throttle`

#### 角速度环

- `roll_gyro_kp`
- `roll_gyro_ki`
- `roll_gyro_kd`
- `roll_gyro_kff`
- `roll_gyro_i_limit`
- `roll_gyro_d_lpf`
- `pitch_gyro_kp`
- `pitch_gyro_ki`
- `pitch_gyro_kd`
- `pitch_gyro_kff`
- `pitch_gyro_i_limit`
- `pitch_gyro_d_lpf`
- `yaw_gyro_kp`
- `yaw_gyro_ki`
- `yaw_gyro_kd`
- `yaw_gyro_kff`
- `yaw_gyro_i_limit`
- `yaw_gyro_d_lpf`

#### 角度环

- `roll_angle_kp`
- `roll_angle_ki`
- `roll_angle_kd`
- `roll_angle_kff`
- `roll_angle_i_limit`
- `roll_angle_d_lpf`
- `pitch_angle_kp`
- `pitch_angle_ki`
- `pitch_angle_kd`
- `pitch_angle_kff`
- `pitch_angle_i_limit`
- `pitch_angle_d_lpf`
- `yaw_angle_kp`
- `yaw_angle_ki`
- `yaw_angle_kd`
- `yaw_angle_kff`
- `yaw_angle_i_limit`
- `yaw_angle_d_lpf`

#### 位置环

- `pos_x_kp`
- `pos_x_ki`
- `pos_x_kd`
- `pos_x_kff`
- `pos_x_i_limit`
- `pos_x_d_lpf`
- `pos_y_kp`
- `pos_y_ki`
- `pos_y_kd`
- `pos_y_kff`
- `pos_y_i_limit`
- `pos_y_d_lpf`
- `pos_z_kp`
- `pos_z_ki`
- `pos_z_kd`
- `pos_z_kff`
- `pos_z_i_limit`
- `pos_z_d_lpf`

#### 速度环

- `vel_x_kp`
- `vel_x_ki`
- `vel_x_kd`
- `vel_x_kff`
- `vel_x_i_limit`
- `vel_x_d_lpf`
- `vel_y_kp`
- `vel_y_ki`
- `vel_y_kd`
- `vel_y_kff`
- `vel_y_i_limit`
- `vel_y_d_lpf`
- `vel_z_kp`
- `vel_z_ki`
- `vel_z_kd`
- `vel_z_kff`
- `vel_z_i_limit`
- `vel_z_d_lpf`

#### 模式 1 参数

- `mode1_track_ff_deg_per_cmps`
- `mode1_brake_kp`
- `mode1_brake_exit_vel_cmps`

### 13.2 当前没有开放的参数

当前没有开放控制周期参数：

- `gyro_dt`
- `angle_dt`
- `pos_xy_dt`
- `pos_z_dt`
- `vel_xy_dt`
- `vel_z_dt`

原因：

- 这类参数一旦在线误改，风险高于普通 PID 增益

---

## 14. 范围检查规则

当前内置了范围检查。

### 14.1 整数参数

- `base_throttle`：`0 ~ 6000`

### 14.2 大部分普通增益

- 大部分 `kp / ki / kd / kff`：`0.0 ~ 3000.0`

### 14.3 大部分积分限幅

- 大部分 `*_i_limit`：`0.0 ~ 5000.0`

### 14.4 大部分微分低通

- 大部分 `*_d_lpf`：`0.0 ~ 1.0`

### 14.5 模式 1 特殊参数

- `mode1_track_ff_deg_per_cmps`：`0.0 ~ 1.0`
- `mode1_brake_kp`：`0.0 ~ 50.0`
- `mode1_brake_exit_vel_cmps`：`0.0 ~ 300.0`

### 14.6 超范围会怎样

例如：

```text
set base_throttle 99999
```

会回：

```text
ERR range
```

并且：

- 不会偷偷裁剪成上限
- 不会写入错误值

---

## 15. 初始化没过会怎么样

这里分两种。

### 15.1 `wifi_params_Init()` 没执行

正常主程序里已经执行了 `wifi_params_Init()`。

如果你以后改启动流程把它漏掉，风险主要是：

- 文本解析状态和诊断状态可能不是你预期的干净初值

虽然静态区上电默认会清零，但仍然不建议依赖这个副作用。

结论：

- `wifi_params_Init()` 应当保留，不能删

### 15.2 底层 WiFi/UDP 没建好

`wifi_params` 自己不负责建链，它依赖前面的：

- `wifi_vofa_Init()`

如果底层 WiFi 或 UDP socket 没建成功，通常表现为：

- 你发过去的文本命令没有任何效果
- 收不到文本回包
- 参数不会变化

但不会导致：

- 飞控参数乱改
- 系统启动异常

---

## 16. VOFA 诊断通道含义

当前主循环还是通过 VOFA 回传 13 个通道。

前 9 个通道保持原来的 IMU/姿态含义。

最后 4 个通道现在改成：

10. `last_command_code`
11. `last_result_code`
12. `last_param_index`
13. `last_value`

### 16.1 `last_command_code`

当前定义：

- `0`：无命令
- `1`：`ping`
- `2`：`help`
- `3`：`get`
- `4`：`set`
- `5`：`save`
- `6`：`load`
- `7`：`list`
- `8`：`start`
- `9`：`stop`

### 16.2 `last_result_code`

当前定义：

- `0`：成功
- `1`：格式错误
- `2`：未知命令
- `3`：未知参数
- `4`：超范围
- `5`：状态不允许
- `6`：Flash 错误

### 16.3 `last_param_index`

含义：

- 最近一次命中的白名单参数索引

约定：

- `0` 表示没有命中具体参数

### 16.4 `last_value`

含义：

- 最近一次处理关联的数值

例如：

- `get` 成功时，这里是当前参数值
- `set` 成功时，这里是新值

---

## 17. 当前推荐使用流程

### 17.1 第一次联调

建议先这样做：

1. 保持飞控待机且未解锁
2. 文本工具向 UDP 端口发送 `ping`
3. 确认收到 `OK ping`
4. 再发送 `get base_throttle`
5. 确认收到当前值

### 17.2 临时调参

例如：

```text
set base_throttle 3600
```

如果收到：

```text
OK set base_throttle=3600
```

说明：

- RAM 参数已生效
- 控制器已重新装载
- 但还没有永久保存

### 17.3 永久保存

调好后发送：

```text
save
```

如果收到：

```text
OK save
```

说明：

- 当前参数已经写入 Flash

### 17.4 恢复上次保存值

如果你临时改乱了参数，可以发送：

```text
load
```

如果收到：

```text
OK load
```

说明：

- 当前参数已被 Flash 覆盖
- 控制器已重新应用

### 17.5 导出全部参数

发送：

```text
list
```

然后等待：

- 先收到 `OK list begin ...`
- 中间收到很多行 `name=value`
- 最后收到 `OK list end`

看到结束行后，说明本次全量输出完成。

---

## 18. 典型异常场景

### 18.1 命令字写错

输入：

```text
abc
```

回包：

```text
ERR unknown command
```

### 18.2 参数名不存在

输入：

```text
get abc
```

回包：

```text
ERR unknown param
```

### 18.3 数值不是合法数字

输入：

```text
set base_throttle hello
```

回包：

```text
ERR format
```

### 18.4 数值超范围

输入：

```text
set base_throttle 100000
```

回包：

```text
ERR range
```

### 18.5 飞行中改参

输入：

```text
set roll_angle_kp 5.5
```

回包：

```text
ERR state
```

### 18.6 从未保存过就执行 `load`

输入：

```text
load
```

回包：

```text
ERR flash
```

---

## 19. 当前实现边界

当前版本是第一版人工文本协议，目标是：

- 简单
- 稳定
- 容易手工使用

### 19.1 已实现

- 人工可读文本命令
- 文本回包
- 白名单参数
- 范围检查
- 显式 `save / load`
- 全量 `list`
- 待机态门控
- PID 重新装载

### 19.2 没有实现

- 第二个独立端口
- 文本和 VOFA 通道隔离
- JSON
- 批量事务
- 自动保存
- 飞行中热修改
- 参数版本自动迁移

---

## 20. 一句话结论

当前 `wifi_params` 的实际行为可以概括成：

- 上电先装默认值，再尝试读 Flash
- 没保存过或读失败时自动回默认值
- 只支持人工可读文本命令
- `set` 只改当前运行参数
- `save` 才会永久保存
- `load` 会恢复上次保存值
- `list` 会一口气输出全部白名单参数
- 只有待机且未解锁时才能改参
- 文本回包会和 VOFA 浮点帧共用同一个 UDP 通道
