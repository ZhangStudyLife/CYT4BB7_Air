# 0423 TOF 双路复现与四路候选方案分析

## 1. 当前双 TOF 离线复现结果

### 1.1 数据清洗与切段

- 日志文件：`0423_tof_fused.csv`
- 先剔除 1 行非有限值脏数据：
  - `nan,nan,nan,nan,0,0,0,0,-0.802481,685.501221`
- 剔除后按时间回跳切成 4 段：
  - `Segment 0`: `33098..73573 ms`
  - `Segment 1`: `7101..12680 ms`
  - `Segment 2`: `7601..12480 ms`
  - `Segment 3`: `8501..228356 ms`

### 1.2 双 TOF 回放与日志对齐

离线脚本：
- `replay_dual_tof_height_est.py`
- `analyze_four_tof_candidates.py`

回放结果：

| Segment | rows | 高度误差均值 mm | 高度误差最大 mm | 速度误差均值 m/s | 速度误差最大 m/s |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 | 4049 | 0.000579 | 0.280876 | 0.000014 | 0.003983 |
| 1 | 559 | 0.004111 | 0.318134 | 0.000080 | 0.003245 |
| 2 | 489 | 0.006943 | 0.258460 | 0.000130 | 0.008220 |
| 3 | 21989 | 0.000228 | 0.328045 | 0.000006 | 0.012097 |

结论：
- 双 TOF 融合逻辑已经被高精度复现。
- 不能做到首样本完全逐点一致，但已经“足够接近”，而且段尾能对齐。
- 首帧差异的主要原因不是公式错，而是日志只在 `FC_START_CRSF_STATE_FLYING` 时开始发，融合器在日志开始前可能已经运行过若干帧，因此日志首帧 `vz` 可能非零。

## 2. 当前算法漏洞

### 2.1 TOF2/TOF3 并不是摆设

基于同一份日志统计：

- `ready_rows = 27082`
- `single_14_plus_23_valid = 80`
  - `TOF1/TOF4` 只有一路有效，但 `TOF2/TOF3` 还能补额外有效量测
- `single_14_accept_plus_23_accept = 333`
  - 在预测门控后，`TOF1/TOF4` 只有一路能进门，但 `TOF2/TOF3` 还能提供额外可信量测
- `both_14_invalid_23_valid = 3`
  - 虽然不多，但确实存在 `TOF1/TOF4` 都失效、而 `TOF2/TOF3` 仍能活着的帧
- `outlier_14_supported_by_23 = 396`
  - 存在 396 帧是 `TOF1/TOF4` 某一路明显偏离，而 `TOF2/TOF3` 贴近其余共识

这说明当前只用 `TOF1 + TOF4`，会丢掉两类信息：
- 额外观测覆盖率
- 单路异常时的群体反证能力

### 2.2 只用 TOF1/TOF4 时，单路异常很容易把高度拉偏

定义异常事件：
- `TOF1` 或 `TOF4` 相对四路中值偏离 `>= 60 mm`
- 同时 `TOF2` 或 `TOF3` 至少一路距离中值 `<= 30 mm`

在这 396 帧里：

- 当前双 TOF：
  - `outlier_pull_p95 = 107.37 mm`
  - `outlier_pull_max = 124.91 mm`
- 这不是小偏差，是会直接污染高度闭环的量级

### 2.3 当前 AB 更新会把错误高度注入成错误速度

仍看上面那 396 帧异常事件：

- 双 TOF 事件期 `|vz|` 的 `p95 ≈ 0.492 m/s`
- 双 TOF 事件期 `|Δvz|` 的 `p95 ≈ 0.0876 m/s/帧`

这说明一旦错误高度通过残差门控混进来，`beta` 更新会把它转成速度尖峰，对速度闭环尤其不友好。

### 2.4 双 TOF 的整体指标不占优

全日志、不剔尾时，当前双 TOF 基线：

- `height_step_std = 5.5796 mm`
- `vz_step_std = 0.0120 m/s`
- `ref_err_p50 = 7.56 mm`
- `ref_err_p95 = 14.40 mm`
- `quiet_vz_p95 = 0.1205 m/s`

这里 `ref` 是“至少 3 路有效且四路 spread <= 120 mm 时的四路中值参考”。它不是绝对真值，但足够用来比较方案鲁棒性和闭环可用性。

## 3. 四 TOF 候选方案

## 方案 A：四路预测门控加权平均 + 原 AB

### 公式

对每路：
- `q_i = 0`，若该路无效
- `q_i = 0`，若 `|h_i - h_pred| >= 120 mm`
- `q_i = 1 - |h_i - h_pred| / 120`

量测融合：
- `z_meas = sum(q_i * h_i) / sum(q_i)`

状态更新仍保持：
- `h = h_pred + alpha * (z_meas - h_pred)`
- `vz = v_pred + (beta / dt) * ((z_meas - h_pred) * 0.001)`

### 优点

- 改动最小，直接从双 TOF 扩展到四 TOF
- 速度更平滑，`vz_step_std = 0.00828 m/s`
- 对异常帧已经明显比双 TOF 强

### 缺点

- 单路大偏差如果没被预测门控挡住，仍会参与平均
- 抗遮挡能力比纯双 TOF 强，但不算最稳

### 离线指标

- `ref_err_p95 = 6.17 mm`
- `quiet_vz_p95 = 0.1144 m/s`
- `outlier_pull_p95 = 14.41 mm`

## 方案 B：四路预测门控中值/截尾平均 + 原 AB

### 公式

先做预测门控：
- `S = { i | h_i valid and |h_i - h_pred| < 120 }`

然后：
- 若 `|S| >= 3`，`z_meas = median({h_i | i in S})`
- 若 `|S| == 2`，`z_meas = mean({h_i | i in S})`
- 若 `|S| == 1`，可选单路使用或继续 predict-only

更新仍保持原 AB。

### 优点

- 对单路遮挡和单路跳变最直接，逻辑简单
- 高度偏差指标最好

### 缺点

- 中值切换会有阶梯感
- 速度平滑性不如鲁棒加权均值

### 离线指标

- `ref_err_p95 = 4.38 mm`
- `quiet_vz_p95 = 0.1140 m/s`
- `outlier_pull_p95 = 14.52 mm`

## 方案 C：四路预测门控 + Huber 鲁棒加权平均 + 原 AB

### 公式

先做预测门控：
- `S = { i | h_i valid and |h_i - h_pred| < 120 }`

取门控集合中值：
- `m = median({h_i | i in S})`

定义鲁棒权重：
- `w_cons_i = 1`，若 `|h_i - m| <= k`
- `w_cons_i = k / |h_i - m|`，若 `|h_i - m| > k`
- 当前离线评估用 `k = 25 mm`

预测权重：
- `w_pred_i = 1 - |h_i - h_pred| / 120`

总权重：
- `q_i = w_pred_i * w_cons_i`

量测融合：
- `z_meas = sum(q_i * h_i) / sum(q_i)`

更新仍保持原 AB。

### 优点

- 比纯加权平均更能压单路异常
- 比纯中值更平滑，速度更友好
- 数学形式简单，嵌入式实现成本可控

### 缺点

- 比方案 A/B 多一个 `k` 参数
- 需要注意 `k` 过小会过度抑制正常动态 spread

### 离线指标

- `ref_err_p95 = 5.94 mm`
- `quiet_vz_p95 = 0.1134 m/s`
- `outlier_pull_p95 = 12.79 mm`
- `vz_step_std = 0.00852 m/s`

### 评价

- 这是当前最平衡的方案。

## 方案 D：可信子集选择后融合

### 公式

先做预测门控，再在有效路里找最大一致子集：
- `S = { i | h_i valid and |h_i - h_pred| < 120 }`
- 从 `S` 中选择最大子集 `C`
- 满足 `max(C) - min(C) <= spread_gate`
- 当前离线评估用 `spread_gate = 45 mm`

然后：
- 若 `|C| >= 3`，取中值
- 若 `|C| == 2`，取均值

### 优点

- 逻辑可解释性强
- 很适合做“先筛选，再更新”

### 缺点

- 参数更多，行为分段感更强
- 离线指标没压过方案 C
- 实现时容易因为门限抖动引入切换噪声

### 评价

- 可以作为备选实验方案，不建议作为第一落地方案

## 4. 统一离线评估口径

主判分标准：
- 闭环可用性优先

统一指标：

- 高度输出平滑性：
  - `height_step_std`
- 速度输出平滑性：
  - `vz_step_std`
  - `quiet_vz_p95`
- 高度跟随一致性：
  - `ref_err_p50`
  - `ref_err_p95`
  - `ref_err_max`
- 抗单路异常能力：
  - `outlier_pull_p95`
  - `outlier_pull_max`
- 相位：
  - 与四路共识参考的相关峰值滞后 `lag_frames`
- 脏尾敏感性：
  - 保留一份全日志结果
  - 再保留一份“尾段剔除后”结果

尾段剔除规则：
- 每段只保留到最后一个满足以下条件的时刻：
  - 至少 3 路有效
  - 四路有效值 spread `<= 120 mm`
- 该时刻之后的连续尾段全部视为脏尾，单独做 sensitivity check

当前日志上，剔尾前后结论基本一致，说明推荐方案不是靠“删脏数据”刷出来的。

## 5. 首选方案

### 推荐：方案 C，四路预测门控 + Huber 鲁棒加权平均 + 原 AB

推荐理由：

- 比当前双 TOF 明显更稳，不是小修小补
- 比纯四路加权平均更抗单路遮挡
- 比纯中值更适合速度闭环
- 公式简单，能直接嵌进当前 `Height_Est.c`

和双 TOF 基线相比：

- `ref_err_p95`: `14.40 -> 5.94 mm`
- `outlier_pull_p95`: `107.37 -> 12.79 mm`
- `vz_step_std`: `0.01196 -> 0.00852 m/s`
- `quiet_vz_p95`: `0.1205 -> 0.1134 m/s`

### 不推荐其他方案作为第一落地方案的原因

- 方案 A：
  - 太容易被“门内异常值”继续拉偏
- 方案 B：
  - 高度最稳，但速度连续性略差
- 方案 D：
  - 参数更多、切换感更强、实现风险更大

## 6. 伪代码

```c
/* 四路 TOF 鲁棒融合：先做预测门控，再做鲁棒加权平均 */
valid_idx = [];
for (i = 0; i < 4; i++)
{
    if (tof_h[i] < 1300.0f)
    {
        valid_idx.push(i);
    }
}

if (ready)
{
    h_pred = clamp(h + vz * dt * 1000.0f);
    gate_idx = [];
    for (i in valid_idx)
    {
        if (fabsf(tof_h[i] - h_pred) < 120.0f)
        {
            gate_idx.push(i);
        }
    }
}
else
{
    gate_idx = valid_idx;
}

if (!gate_idx.empty())
{
    m = median(tof_h[gate_idx]);
    weighted_sum = 0.0f;
    weight_sum = 0.0f;

    for (i in gate_idx)
    {
        float pred_w = 1.0f;
        float cons_w;
        float dev = fabsf(tof_h[i] - m);

        if (ready)
        {
            pred_w = 1.0f - fabsf(tof_h[i] - h_pred) / 120.0f;
        }

        if (dev <= 25.0f)
        {
            cons_w = 1.0f;
        }
        else
        {
            cons_w = 25.0f / dev;
        }

        q = pred_w * cons_w;
        if (q > 0.0f)
        {
            weighted_sum += q * tof_h[i];
            weight_sum += q;
        }
    }

    if (weight_sum > eps)
    {
        z_meas = weighted_sum / weight_sum;
        meas_valid = 1U;
    }
}

if (!ready)
{
    if (meas_valid)
    {
        h = clamp(z_meas);
        vz = 0.0f;
        ready = 1U;
        hold_cnt = 0U;
    }
}
else if (meas_valid)
{
    residual = z_meas - h_pred;
    h = clamp(h_pred + alpha * residual);
    vz = v_pred + (beta / dt) * (residual * 0.001f);
    hold_cnt = 0U;
}
else
{
    h = clamp(h_pred);
    vz = v_pred * 0.95f;
    if (hold_cnt < 15U)
    {
        hold_cnt++;
    }
}
```

## 7. 对后续实机落地的建议

- 第一版不要同时改太多参数
- 先只把观测融合从双 TOF 换成方案 C
- `alpha/beta` 先保持不变
- 实飞后再看是否要把 `beta` 略微下调，进一步压速度尖峰
- 落地后必须继续保留四路解耦高度日志，方便重新离线复盘
