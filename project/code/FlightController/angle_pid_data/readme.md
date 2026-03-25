## 第一次飞行

project\code\FlightController\angle_pid_data\03251623_angle.csv

以上这个飞行日志是手动操控遥控器,给目标角度,收集的

飞行参数如下:"

[16:11:54.860] roll_mech_trim_deg=-0.5
[16:11:54.860] pitch_mech_trim_deg=0
[16:11:54.860] roll_gyro_kp=3
[16:11:54.860] roll_gyro_ki=0.8
[16:11:54.864] roll_gyro_kd=0.03
[16:11:54.864] roll_gyro_kff=0
[16:11:54.864] roll_gyro_i_limit=300
[16:11:54.864] roll_gyro_d_lpf=30
[16:11:54.866] pitch_gyro_kp=3
[16:11:54.866] pitch_gyro_ki=0.8
[16:11:54.866] pitch_gyro_kd=0.03
[16:11:54.866] pitch_gyro_kff=0
[16:11:54.866] pitch_gyro_i_limit=300
[16:11:54.866] pitch_gyro_d_lpf=30
[16:11:54.866] yaw_gyro_kp=14
[16:11:54.879] yaw_gyro_ki=8
[16:11:54.879] yaw_gyro_kd=0
[16:11:54.879] yaw_gyro_kff=0
[16:11:54.879] yaw_gyro_i_limit=1800
[16:11:54.879] yaw_gyro_d_lpf=30
[16:11:54.879] roll_angle_kp=5
[16:11:54.879] roll_angle_ki=0
[16:11:54.879] roll_angle_kd=0
[16:11:54.879] roll_angle_kff=0
[16:11:54.879] roll_angle_i_limit=110
[16:11:54.879] roll_angle_d_lpf=0
[16:11:54.879] pitch_angle_kp=5
[16:11:54.879] pitch_angle_ki=0
[16:11:54.879] pitch_angle_kd=0
[16:11:54.879] pitch_angle_kff=0
[16:11:54.879] pitch_angle_i_limit=110
[16:11:54.883] pitch_angle_d_lpf=0
[16:11:54.883] yaw_angle_kp=0
[16:11:54.883] yaw_angle_ki=0
[16:11:54.884] yaw_angle_kd=0
[16:11:54.884] yaw_angle_kff=0
[16:11:54.884] yaw_angle_i_limit=0
[16:11:54.884] yaw_angle_d_lpf=0"

2MS发送一次数据:

    wifi_justfloat(tick_1000us_cnt,

    roll_angle_target, roll_angle_meas, roll_angle_pid.p_term, roll_angle_pid.i_term, roll_angle_pid.d_term,

    roll_gyro_target, g_imufilter_1000hz.gyrox,

    pitch_angle_target, pitch_angle_meas, pitch_angle_pid.p_term, pitch_angle_pid.i_term, pitch_angle_pid.d_term,

    pitch_gyro_target, g_imufilter_1000hz.gyroy);

## 第二次飞行

project\code\FlightController\angle_pid_data\03251809_angle.csv

- 在 /D:/Car_Air_Protocol/CYT4BB7_Air/project/code/FlightController/fc_loop.c:174 给 roll/pitch 姿态外环启用了 anti-windup、±260 deg/s 输出限幅、30deg/s 积分松弛阈值。
- 在 /D:/Car_Air_Protocol/CYT4BB7_Air/project/code/FlightController/fc_loop.c:415 把 FC_Loop_500Hz 的外环输出保持为 float，不再先截成 int32_t 再喂给roll_gyro_target/pitch_gyro_target。
- 在 /D:/Car_Air_Protocol/CYT4BB7_Air/project/code/FlightController/fc_params.c:108 把默认姿态外环参数更新成两轴同构：kp=7.0、ki=0.10、kd=0.10、kff=0、
  i_limit=80、d_lpf=15Hz。

[17:58:50.481] roll_gyro_kp=3

[17:58:50.481] roll_gyro_ki=0.8

[17:58:50.481] roll_gyro_kd=0.03

[17:58:50.481] roll_gyro_kff=0

[17:58:50.482] roll_gyro_i_limit=300

[17:58:50.482] roll_gyro_d_lpf=30

[17:58:50.483] pitch_gyro_kp=3

[17:58:50.483] pitch_gyro_ki=0.8

[17:58:50.484] pitch_gyro_kd=0.03

[17:58:50.484] pitch_gyro_kff=0

[17:58:50.484] pitch_gyro_i_limit=300

[17:58:50.485] pitch_gyro_d_lpf=30

[17:58:50.485] yaw_gyro_kp=14

[17:58:50.486] yaw_gyro_ki=8

[17:58:50.487] yaw_gyro_kd=0

[17:58:50.487] yaw_gyro_kff=0

[17:58:50.487] yaw_gyro_i_limit=1800

[17:58:50.488] yaw_gyro_d_lpf=30

[17:58:50.488] roll_angle_kp=7

[17:58:50.488] roll_angle_ki=0.1

[17:58:50.489] roll_angle_kd=0.1

[17:58:50.489] roll_angle_kff=0

[17:58:50.489] roll_angle_i_limit=80

[17:58:50.490] roll_angle_d_lpf=15

[17:58:50.490] pitch_angle_kp=7

[17:58:50.491] pitch_angle_ki=0.1

[17:58:50.491] pitch_angle_kd=0.1

[17:58:50.492] pitch_angle_kff=0

[17:58:50.492] pitch_angle_i_limit=80

[17:58:50.492] pitch_angle_d_lpf=15



## 第三次飞行

project\code\FlightController\angle_pid_data\03251820_angle.csv

在第二次的飞行基础上修改了以下参数:

[18:11:03.640] SET pitch_angle_kp 10

[18:11:03.643] OK set pitch_angle_kp=10

[18:11:10.340] SET roll_angle_kp 10

[18:11:10.344] OK set roll_angle_kp=10



## 第四次飞行

project\code\FlightController\angle_pid_data\03251834_angle.csv

[18:27:14.012] roll_angle_kp=5

[18:27:14.012] roll_angle_ki=0.1

[18:27:14.012] roll_angle_kd=0.1

[18:27:14.012] roll_angle_kff=0

[18:27:14.012] roll_angle_i_limit=80

[18:27:14.012] roll_angle_d_lpf=15

[18:27:14.012] pitch_angle_kp=5

[18:27:14.012] pitch_angle_ki=0.1

[18:27:14.012] pitch_angle_kd=0.1

[18:27:14.012] pitch_angle_kff=0

[18:27:14.012] pitch_angle_i_limit=80

[18:27:14.012] pitch_angle_d_lpf=15



## 第五次飞行


project\code\FlightController\angle_pid_data\03251839_angle.csv

[18:36:43.955] roll_gyro_kp=3

[18:36:43.955] roll_gyro_ki=0.8

[18:36:43.955] roll_gyro_kd=0.03

[18:36:43.955] roll_gyro_kff=0

[18:36:43.955] roll_gyro_i_limit=300

[18:36:43.955] roll_gyro_d_lpf=30

[18:36:43.955] pitch_gyro_kp=3

[18:36:43.955] pitch_gyro_ki=0.8

[18:36:43.955] pitch_gyro_kd=0.03

[18:36:43.955] pitch_gyro_kff=0

[18:36:43.955] pitch_gyro_i_limit=300

[18:36:43.955] pitch_gyro_d_lpf=30

[18:36:43.955] yaw_gyro_kp=14

[18:36:43.955] yaw_gyro_ki=8

[18:36:43.955] yaw_gyro_kd=0

[18:36:43.955] yaw_gyro_kff=0

[18:36:43.956] yaw_gyro_i_limit=1800

[18:36:43.956] yaw_gyro_d_lpf=30

[18:36:43.957] roll_angle_kp=5

[18:36:43.957] roll_angle_ki=0.1

[18:36:43.957] roll_angle_kd=0.3

[18:36:43.959] roll_angle_kff=0

[18:36:43.959] roll_angle_i_limit=80

[18:36:43.959] roll_angle_d_lpf=15

[18:36:43.959] pitch_angle_kp=5

[18:36:43.959] pitch_angle_ki=0.1

[18:36:43.959] pitch_angle_kd=0.3

[18:36:43.961] pitch_angle_kff=0

[18:36:43.963] pitch_angle_i_limit=80

[18:36:43.963] pitch_angle_d_lpf=15


# AI分析(应当辩证看待)


我并行拆了 4 条线，把 [angle_pid_data](/D:/Car_Air_Protocol/CYT4BB7_Air/project/code/FlightController/angle_pid_data) 里 5 份日志和 [readme.md](/D:/Car_Air_Protocol/CYT4BB7_Air/project/code/FlightController/angle_pid_data/readme.md) 全部对齐分析了。代码和参数我都没动，只做了离线数据分析。

**结论**

- 对你这个“室内 `1m` 高度、稳定定点悬停定高”的目标，综合推荐姿态外环角度环参数是：
- `roll_angle_kp = 7.0`
- `roll_angle_ki = 0.10`
- `roll_angle_kd = 0.10`
- `roll_angle_kff = 0`
- `roll_angle_i_limit = 80`
- `roll_angle_d_lpf = 15`
- `pitch_angle_kp = 7.0`
- `pitch_angle_ki = 0.10`
- `pitch_angle_kd = 0.10`
- `pitch_angle_kff = 0`
- `pitch_angle_i_limit = 80`
- `pitch_angle_d_lpf = 15`

这套就是 [03251809_angle.csv](/D:/Car_Air_Protocol/CYT4BB7_Air/project/code/FlightController/angle_pid_data/03251809_angle.csv) 对应那档。它不是绝对最快，但在“悬停误差、动态滞后、内环压力、两轴对称性”这几项里最平衡，最像你要的定点悬停档。

**核心证据**

- 5 份日志总时长约 `1548s`，采样整体都还能用，`2ms` 占比大约 `93%~96%`。
- 最干净的参数对比是：
- `03251809` 对 `03251820`：看 `Kp 7 -> 10`
- `03251834` 对 `03251839`：看 `Kd 0.1 -> 0.3`

综合看，`7/0.1/0.1` 比 `5/0.1/0.1` 明显更值钱：

- 平均近悬停 `p95`：`3.73deg` 对 `4.17deg`
- 平均活动段滞后：`3.85ms` 对 `5.52ms`
- 响应更快，但角速度目标强度还没暴躁到离谱

`10/0.1/0.1` 虽然更快，但不适合你的主目标：

- 平均近悬停 `p95` 只比 `7/0.1/0.1` 好了大约 `1.9%`
- 平均活动段滞后大约快了 `8.8%`
- 但平均 `gyro_target p99` 从 `68.2` 直接抬到 `126.1`，内环跟踪压力也上升约 `14%`
- 这更像“追响应”的档，不像“1 米稳悬停”的档

`5/0.1/0.3` 这档不推荐：

- 平均近悬停 `p95` 比 `5/0.1/0.1` 还差一点：`4.25deg` 对 `4.17deg`
- 平均活动段滞后更慢：`6.84ms` 对 `5.52ms`
- 说明 `Kd=0.3` 在你这套机体上已经偏重了，没换来更快，反而更拖

`5/0/0` 只能当旧基线，不适合最终定点：

- 纯 P 明显偏软
- `pitch` 悬停附近有持续静差
- 而且它和后 4 份还混了外环实现链变化，不能当纯参数 A/B

**建议排序**

- 最推荐：`7 / 0.1 / 0.1 / kff=0 / i_limit=80 / d_lpf=15`
- 次推荐偏保守：`6.5 / 0.1 / 0.08~0.10 / kff=0 / i_limit=80 / d_lpf=15`
- 不推荐用于 1m 稳悬停主档：`10 / 0.1 / 0.1`
- 不推荐：`5 / 0.1 / 0.3`
- 不推荐：`5 / 0 / 0`

**边界**

- 第一份 [03251623_angle.csv](/D:/Car_Air_Protocol/CYT4BB7_Air/project/code/FlightController/angle_pid_data/03251623_angle.csv) 和后四份不是纯参数对比，因为控制链实现也变了。
- 这 5 份日志足够做“趋势筛选”和“候选参数收敛”，但还不足以把“1m 定点悬停最终定参”钉死。
- 缺 `flight_mode`、位置/速度、高度、TOF 有效性、光流质量这些量，所以这次结论是“姿态外环最优候选”，不是整个定点闭环的最终证明。

一句话压缩：你这 5 份数据综合下来，最适合 `1m` 室内稳定定点悬停的姿态外环，不是最快的 `10/0.1/0.1`，也不是更重 D 的 `5/0.1/0.3`，而是中间这档 `7.0/0.10/0.10`。
