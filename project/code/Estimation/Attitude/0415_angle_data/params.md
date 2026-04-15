## 0415 angle data clean index

### 1. 日志字段对应关系

当前 CSV 来自下面这段发送代码：

```c
wifi_justfloat(tick_1000us_cnt,
               roll_angle_target, roll_angle_meas, roll_angle_pid.p_term, roll_angle_pid.i_term, roll_angle_pid.d_term,
               roll_gyro_target, g_imufilter_1000hz.gyrox,
               pitch_angle_target, pitch_angle_meas, pitch_angle_pid.p_term, pitch_angle_pid.i_term, pitch_angle_pid.d_term,
               pitch_gyro_target, g_imufilter_1000hz.gyroy);
```

列含义如下：

| 列 | 含义 | 单位 |
| --- | --- | --- |
| `I0` | `tick_1000us_cnt` | `ms` |
| `I1` | `roll_angle_target` | `deg` |
| `I2` | `roll_angle_meas` | `deg` |
| `I3` | `roll_angle_pid.p_term` | `deg/s` |
| `I4` | `roll_angle_pid.i_term` | `deg/s` |
| `I5` | `roll_angle_pid.d_term` | `deg/s` |
| `I6` | `roll_gyro_target` | `deg/s` |
| `I7` | `g_imufilter_1000hz.gyrox` | `deg/s` |
| `I8` | `pitch_angle_target` | `deg` |
| `I9` | `pitch_angle_meas` | `deg` |
| `I10` | `pitch_angle_pid.p_term` | `deg/s` |
| `I11` | `pitch_angle_pid.i_term` | `deg/s` |
| `I12` | `pitch_angle_pid.d_term` | `deg/s` |
| `I13` | `pitch_gyro_target` | `deg/s` |
| `I14` | `g_imufilter_1000hz.gyroy` | `deg/s` |

### 2. 本目录日志与参数总览

下面这张表是整理后的快速索引，优先看这张表就行。

| 序号 | 日志文件 | Roll angle PID | Pitch angle PID | 说明 |
| --- | --- | --- | --- | --- |
| 1 | `04151231.csv` | `7.0 / 0.0 / 0.0` | `7.0 / 0.0 / 0.0` | 纯 `P=7` 基线 |
| 2 | `04151238.csv` | `5.0 / 0.0 / 0.0` | `5.0 / 0.0 / 0.0` | 纯 `P=5` 基线 |
| 3 | `04151250.csv` | `5.0 / 0.1 / 0.0` | `5.0 / 0.1 / 0.0` | 在 `P=5` 基线上只加 `I` |
| 4 | `04151324.csv` | `5.0 / 0.0 / 0.1` | `5.0 / 0.0 / 0.1` | 在 `P=5` 基线上只加 `D` |
| 5 | `04151329.csv` | `5.0 / 0.1 / 0.1` | `5.0 / 0.1 / 0.1` | 在 `P=5` 基线上同时加 `I + D` |
| 6 | `04151558.csv` | `6.0 / 0.0 / 0.0` | `6.0 / 0.0 / 0.0` | 新增试飞，原文档里漏记了这次 |
| 7 | `04151604.csv` | `6.0 / 0.0 / 0.0` | `6.0 / 0.08 / 0.0` | 新增试飞，按建议只给 Pitch 留一点 `I` |
| 8 | `04151620.csv` | `6.0 / 0.0 / 0.0` | `6.0 / 0.08 / 0.0` | 与 `04151604.csv` 内容完全相同，是重复导出，不算新的独立飞行 |

### 3. 读表规则

这里的 `Roll angle PID` 和 `Pitch angle PID` 都按下面顺序写：

```text
KP / KI / KD
```

也就是说：

- `6.0 / 0.0 / 0.0` 表示 `KP=6.0, KI=0.0, KD=0.0`
- `6.0 / 0.08 / 0.0` 表示 `KP=6.0, KI=0.08, KD=0.0`

### 4. 当前文档注意事项

当前 `params.md` 后面原始记录里有两个需要注意的地方：

1. `04151558.csv` 这次飞行原文档没有写进去，但它是独立有效日志。
2. `04151620.csv` 和 `04151604.csv` 内容完全一致，是重复文件，不应当当成新的独立试飞样本。

---

    wifi_justfloat(tick_1000us_cnt,

    roll_angle_target, roll_angle_meas, roll_angle_pid.p_term, roll_angle_pid.i_term, roll_angle_pid.d_term,

    roll_gyro_target, g_imufilter_1000hz.gyrox,

    pitch_angle_target, pitch_angle_meas, pitch_angle_pid.p_term, pitch_angle_pid.i_term, pitch_angle_pid.d_term,

    pitch_gyro_target, g_imufilter_1000hz.gyroy);

### 第一次飞行

project\code\Estimation\Attitude\0415_angle_data\04151231.csv

/* ===== Roll 轴角速度环参数 ===== */

    params->roll_gyro_kp=2.6f;

    params->roll_gyro_ki=0.75f;

    params->roll_gyro_kd=0.040f;

    params->roll_gyro_kff=0.0f;

    params->roll_gyro_i_limit=300.0f;

    params->roll_gyro_d_lpf=30.0f;

    /* ===== Pitch 轴角速度环参数 ===== */

    params->pitch_gyro_kp=2.8f;

    params->pitch_gyro_ki=0.75f;

    params->pitch_gyro_kd=0.045f;

    params->pitch_gyro_kff=0.0f;

    params->pitch_gyro_i_limit=300.0f;

    params->pitch_gyro_d_lpf=30.0f;

    /* ===== Yaw 轴角速度环参数 ===== */

    params->yaw_gyro_kp=14.0f;

    params->yaw_gyro_ki=4.0f;

    params->yaw_gyro_kd=0.0f;

    params->yaw_gyro_kff=0.0f;

    params->yaw_gyro_i_limit=700.0f;

    params->yaw_gyro_d_lpf=30.0f;

    /* ===== Roll 轴角度环参数 ===== */

    params->roll_angle_kp=7.0f;

    params->roll_angle_ki=0.0f;

    params->roll_angle_kd=0.0f;

    params->roll_angle_kff=0.0f;

    params->roll_angle_i_limit=80.0f;

    params->roll_angle_d_lpf=15.0f;

    /* ===== Pitch 轴角度环参数 ===== */

    params->pitch_angle_kp=7.0f;

    params->pitch_angle_ki=0.0f;

    params->pitch_angle_kd=0.0f;

    params->pitch_angle_kff=0.0f;

    params->pitch_angle_i_limit=80.0f;

    params->pitch_angle_d_lpf=15.0f;

    /* ===== Yaw 轴角度环参数 ===== */

    params->yaw_angle_kp=0.0f;

    params->yaw_angle_ki=0.0f;

    params->yaw_angle_kd=0.0f;

    params->yaw_angle_kff=0.0f;

    params->yaw_angle_i_limit=0.0f;

    params->yaw_angle_d_lpf=0.0f;

### 第二次飞行

project\code\Estimation\Attitude\0415_angle_data\04151238.csv

/* ===== Roll 轴角速度环参数 ===== */

    params->roll_gyro_kp=2.6f;

    params->roll_gyro_ki=0.75f;

    params->roll_gyro_kd=0.040f;

    params->roll_gyro_kff=0.0f;

    params->roll_gyro_i_limit=300.0f;

    params->roll_gyro_d_lpf=30.0f;

    /* ===== Pitch 轴角速度环参数 ===== */

    params->pitch_gyro_kp=2.8f;

    params->pitch_gyro_ki=0.75f;

    params->pitch_gyro_kd=0.045f;

    params->pitch_gyro_kff=0.0f;

    params->pitch_gyro_i_limit=300.0f;

    params->pitch_gyro_d_lpf=30.0f;

    /* ===== Yaw 轴角速度环参数 ===== */

    params->yaw_gyro_kp=14.0f;

    params->yaw_gyro_ki=4.0f;

    params->yaw_gyro_kd=0.0f;

    params->yaw_gyro_kff=0.0f;

    params->yaw_gyro_i_limit=700.0f;

    params->yaw_gyro_d_lpf=30.0f;

    /* ===== Roll 轴角度环参数 ===== */

    params->roll_angle_kp=5.0f;

    params->roll_angle_ki=0.0f;

    params->roll_angle_kd=0.0f;

    params->roll_angle_kff=0.0f;

    params->roll_angle_i_limit=80.0f;

    params->roll_angle_d_lpf=15.0f;

    /* ===== Pitch 轴角度环参数 ===== */

    params->pitch_angle_kp=5.0f;

    params->pitch_angle_ki=0.0f;

    params->pitch_angle_kd=0.0f;

    params->pitch_angle_kff=0.0f;

    params->pitch_angle_i_limit=80.0f;

    params->pitch_angle_d_lpf=15.0f;

    /* ===== Yaw 轴角度环参数 ===== */

    params->yaw_angle_kp=0.0f;

    params->yaw_angle_ki=0.0f;

    params->yaw_angle_kd=0.0f;

    params->yaw_angle_kff=0.0f;

    params->yaw_angle_i_limit=0.0f;

    params->yaw_angle_d_lpf=0.0f;

### 第三次飞行

project\code\Estimation\Attitude\0415_angle_data\04151250.csv

/* ===== Roll 轴角速度环参数 ===== */

    params->roll_gyro_kp=2.6f;

    params->roll_gyro_ki=0.75f;

    params->roll_gyro_kd=0.040f;

    params->roll_gyro_kff=0.0f;

    params->roll_gyro_i_limit=300.0f;

    params->roll_gyro_d_lpf=30.0f;

    /* ===== Pitch 轴角速度环参数 ===== */

    params->pitch_gyro_kp=2.8f;

    params->pitch_gyro_ki=0.75f;

    params->pitch_gyro_kd=0.045f;

    params->pitch_gyro_kff=0.0f;

    params->pitch_gyro_i_limit=300.0f;

    params->pitch_gyro_d_lpf=30.0f;

    /* ===== Yaw 轴角速度环参数 ===== */

    params->yaw_gyro_kp=14.0f;

    params->yaw_gyro_ki=4.0f;

    params->yaw_gyro_kd=0.0f;

    params->yaw_gyro_kff=0.0f;

    params->yaw_gyro_i_limit=700.0f;

    params->yaw_gyro_d_lpf=30.0f;

    /* ===== Roll 轴角度环参数 ===== */

    params->roll_angle_kp=5.0f;

    params->roll_angle_ki=0.1f;

    params->roll_angle_kd=0.0f;

    params->roll_angle_kff=0.0f;

    params->roll_angle_i_limit=80.0f;

    params->roll_angle_d_lpf=15.0f;

    /* ===== Pitch 轴角度环参数 ===== */

    params->pitch_angle_kp=5.0f;

    params->pitch_angle_ki=0.1f;

    params->pitch_angle_kd=0.0f;

    params->pitch_angle_kff=0.0f;

    params->pitch_angle_i_limit=80.0f;

    params->pitch_angle_d_lpf=15.0f;

    /* ===== Yaw 轴角度环参数 ===== */

    params->yaw_angle_kp=0.0f;

    params->yaw_angle_ki=0.0f;

    params->yaw_angle_kd=0.0f;

    params->yaw_angle_kff=0.0f;

    params->yaw_angle_i_limit=0.0f;

    params->yaw_angle_d_lpf=0.0f;

### 第四次飞行

project\code\Estimation\Attitude\0415_angle_data\04151324.csv

/* ===== Roll 轴角速度环参数 ===== */

    params->roll_gyro_kp=2.6f;

    params->roll_gyro_ki=0.75f;

    params->roll_gyro_kd=0.040f;

    params->roll_gyro_kff=0.0f;

    params->roll_gyro_i_limit=300.0f;

    params->roll_gyro_d_lpf=30.0f;

    /* ===== Pitch 轴角速度环参数 ===== */

    params->pitch_gyro_kp=2.8f;

    params->pitch_gyro_ki=0.75f;

    params->pitch_gyro_kd=0.045f;

    params->pitch_gyro_kff=0.0f;

    params->pitch_gyro_i_limit=300.0f;

    params->pitch_gyro_d_lpf=30.0f;

    /* ===== Yaw 轴角速度环参数 ===== */

    params->yaw_gyro_kp=14.0f;

    params->yaw_gyro_ki=4.0f;

    params->yaw_gyro_kd=0.0f;

    params->yaw_gyro_kff=0.0f;

    params->yaw_gyro_i_limit=700.0f;

    params->yaw_gyro_d_lpf=30.0f;

    /* ===== Roll 轴角度环参数 ===== */

    params->roll_angle_kp=5.0f;

    params->roll_angle_ki=0.0f;

    params->roll_angle_kd=0.1f;

    params->roll_angle_kff=0.0f;

    params->roll_angle_i_limit=80.0f;

    params->roll_angle_d_lpf=15.0f;

    /* ===== Pitch 轴角度环参数 ===== */

    params->pitch_angle_kp=5.0f;

    params->pitch_angle_ki=0.0f;

    params->pitch_angle_kd=0.1f;

    params->pitch_angle_kff=0.0f;

    params->pitch_angle_i_limit=80.0f;

    params->pitch_angle_d_lpf=15.0f;

    /* ===== Yaw 轴角度环参数 ===== */

    params->yaw_angle_kp=0.0f;

    params->yaw_angle_ki=0.0f;

    params->yaw_angle_kd=0.0f;

    params->yaw_angle_kff=0.0f;

    params->yaw_angle_i_limit=0.0f;

    params->yaw_angle_d_lpf=0.0f;

### 第五次飞行

project\code\Estimation\Attitude\0415_angle_data\04151329.csv

/* ===== Roll 轴角速度环参数 ===== */

    params->roll_gyro_kp=2.6f;

    params->roll_gyro_ki=0.75f;

    params->roll_gyro_kd=0.040f;

    params->roll_gyro_kff=0.0f;

    params->roll_gyro_i_limit=300.0f;

    params->roll_gyro_d_lpf=30.0f;

    /* ===== Pitch 轴角速度环参数 ===== */

    params->pitch_gyro_kp=2.8f;

    params->pitch_gyro_ki=0.75f;

    params->pitch_gyro_kd=0.045f;

    params->pitch_gyro_kff=0.0f;

    params->pitch_gyro_i_limit=300.0f;

    params->pitch_gyro_d_lpf=30.0f;

    /* ===== Yaw 轴角速度环参数 ===== */

    params->yaw_gyro_kp=14.0f;

    params->yaw_gyro_ki=4.0f;

    params->yaw_gyro_kd=0.0f;

    params->yaw_gyro_kff=0.0f;

    params->yaw_gyro_i_limit=700.0f;

    params->yaw_gyro_d_lpf=30.0f;

    /* ===== Roll 轴角度环参数 ===== */

    params->roll_angle_kp=5.0f;

    params->roll_angle_ki=0.1f;

    params->roll_angle_kd=0.1f;

    params->roll_angle_kff=0.0f;

    params->roll_angle_i_limit=80.0f;

    params->roll_angle_d_lpf=15.0f;

    /* ===== Pitch 轴角度环参数 ===== */

    params->pitch_angle_kp=5.0f;

    params->pitch_angle_ki=0.1f;

    params->pitch_angle_kd=0.1f;

    params->pitch_angle_kff=0.0f;

    params->pitch_angle_i_limit=80.0f;

    params->pitch_angle_d_lpf=15.0f;

    /* ===== Yaw 轴角度环参数 ===== */

    params->yaw_angle_kp=0.0f;

    params->yaw_angle_ki=0.0f;

    params->yaw_angle_kd=0.0f;

    params->yaw_angle_kff=0.0f;

    params->yaw_angle_i_limit=0.0f;

    params->yaw_angle_d_lpf=0.0f;

### 第六次飞行

project\code\Estimation\Attitude\0415_angle_data\04151604.csv

    /* ===== Roll 轴角速度环参数 ===== */

    params->roll_gyro_kp=2.6f;

    params->roll_gyro_ki=0.75f;

    params->roll_gyro_kd=0.040f;

    params->roll_gyro_kff=0.0f;

    params->roll_gyro_i_limit=300.0f;

    params->roll_gyro_d_lpf=30.0f;

    /* ===== Pitch 轴角速度环参数 ===== */

    params->pitch_gyro_kp=2.8f;

    params->pitch_gyro_ki=0.75f;

    params->pitch_gyro_kd=0.045f;

    params->pitch_gyro_kff=0.0f;

    params->pitch_gyro_i_limit=300.0f;

    params->pitch_gyro_d_lpf=30.0f;

    /* ===== Yaw 轴角速度环参数 ===== */

    params->yaw_gyro_kp=14.0f;

    params->yaw_gyro_ki=4.0f;

    params->yaw_gyro_kd=0.0f;

    params->yaw_gyro_kff=0.0f;

    params->yaw_gyro_i_limit=700.0f;

    params->yaw_gyro_d_lpf=30.0f;

    /* ===== Roll 轴角度环参数 ===== */

    params->roll_angle_kp=6.0f;

    params->roll_angle_ki=0.0f;

    params->roll_angle_kd=0.0f;

    params->roll_angle_kff=0.0f;

    params->roll_angle_i_limit=80.0f;

    params->roll_angle_d_lpf=15.0f;

    /* ===== Pitch 轴角度环参数 ===== */

    params->pitch_angle_kp=6.0f;

    params->pitch_angle_ki=0.0f;

    params->pitch_angle_kd=0.0f;

    params->pitch_angle_kff=0.0f;

    params->pitch_angle_i_limit=80.0f;

    params->pitch_angle_d_lpf=15.0f;

    /* ===== Yaw 轴角度环参数 ===== */

    params->yaw_angle_kp=0.0f;

    params->yaw_angle_ki=0.0f;

    params->yaw_angle_kd=0.0f;

    params->yaw_angle_kff=0.0f;

    params->yaw_angle_i_limit=0.0f;

    params->yaw_angle_d_lpf=0.0f;

### 第七次飞行

project\code\Estimation\Attitude\0415_angle_data\04151620.csv

/* ===== Roll 轴角速度环参数 ===== */

    params->roll_gyro_kp=2.6f;

    params->roll_gyro_ki=0.75f;

    params->roll_gyro_kd=0.040f;

    params->roll_gyro_kff=0.0f;

    params->roll_gyro_i_limit=300.0f;

    params->roll_gyro_d_lpf=30.0f;

    /* ===== Pitch 轴角速度环参数 ===== */

    params->pitch_gyro_kp=2.8f;

    params->pitch_gyro_ki=0.75f;

    params->pitch_gyro_kd=0.045f;

    params->pitch_gyro_kff=0.0f;

    params->pitch_gyro_i_limit=300.0f;

    params->pitch_gyro_d_lpf=30.0f;

    /* ===== Yaw 轴角速度环参数 ===== */

    params->yaw_gyro_kp=14.0f;

    params->yaw_gyro_ki=4.0f;

    params->yaw_gyro_kd=0.0f;

    params->yaw_gyro_kff=0.0f;

    params->yaw_gyro_i_limit=700.0f;

    params->yaw_gyro_d_lpf=30.0f;

    /* ===== Roll 轴角度环参数 ===== */

    params->roll_angle_kp=6.0f;

    params->roll_angle_ki=0.0f;

    params->roll_angle_kd=0.0f;

    params->roll_angle_kff=0.0f;

    params->roll_angle_i_limit=80.0f;

    params->roll_angle_d_lpf=15.0f;

    /* ===== Pitch 轴角度环参数 ===== */

    params->pitch_angle_kp=6.0f;

    params->pitch_angle_ki=0.08f;

    params->pitch_angle_kd=0.0f;

    params->pitch_angle_kff=0.0f;

    params->pitch_angle_i_limit=80.0f;

    params->pitch_angle_d_lpf=15.0f;

    /* ===== Yaw 轴角度环参数 ===== */

    params->yaw_angle_kp=0.0f;

    params->yaw_angle_ki=0.0f;

    params->yaw_angle_kd=0.0f;

    params->yaw_angle_kff=0.0f;

    params->yaw_angle_i_limit=0.0f;

    params->yaw_angle_d_lpf=0.0f;



第八次飞行

project\code\Estimation\Attitude\0415_angle_data\04151643.csv


- Roll = 6.4 / 0 / 0
  - Pitch = 6.0 / 0 / 0
