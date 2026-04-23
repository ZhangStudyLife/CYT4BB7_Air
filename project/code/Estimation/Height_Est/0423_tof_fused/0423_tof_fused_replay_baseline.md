# 0423 TOF 融合日志离线回放基线

## 用途

这份文件记录当前 `Height_Est.c` 双 TOF 融合器对 `0423_tof_fused.csv` 的离线回放基线，供后续清理上下文后继续分析或与云端 AI 方案对比。

## 对应文件

- 提示词：`0423_tof_fused_cloud_ai_prompt.md`
- 回放脚本：`replay_dual_tof_height_est.py`
- 日志文件：`0423_tof_fused.csv`

## 当前离线回放前提

1. 日志列定义：
   - `I0`: 时间戳 ms
   - `I1..I4`: 四路姿态解耦后高度，`1300 mm` 代表无效
   - `I5`: roll_deg
   - `I6`: pitch_deg
   - `I7`: acc_z_dyn_mps2
   - `I8`: 当前固件输出的 `g_tof_fused_vz_mps`
   - `I9`: 当前固件输出的 `g_tof_fused_height_mm`
2. 日志中存在 1 行非有限值脏记录，必须先剔除：
   - `nan,nan,nan,nan,0,0,0,0,-0.802481,685.501221`
3. 剔除该脏行后，再按时间回跳切成 4 段单调时间序列：
   - `Segment 0`: `33098..73573 ms`
   - `Segment 1`: `7101..12680 ms`
   - `Segment 2`: `7601..12480 ms`
   - `Segment 3`: `8501..228356 ms`
4. 每段回放时按 `HeightEst_ResetAll()` 等价初始状态重新开始。

## 当前双 TOF 回放结果

脚本运行命令：

```bash
python "project/code/Estimation/Height_Est/0423_tof_fused/replay_dual_tof_height_est.py"
```

脚本输出摘要：

```text
rows=27086 dropped_rows=1 sessions=4
session=0 rows=4049 t_start=33098 t_end=73573 height_err: mean=0.000579 max=0.280876 vz_err: mean=0.000014 max=0.003983
session=1 rows=559  t_start=7101  t_end=12680 height_err: mean=0.004111 max=0.318134 vz_err: mean=0.000080 max=0.003245
session=2 rows=489  t_start=7601  t_end=12480 height_err: mean=0.006943 max=0.258460 vz_err: mean=0.000130 max=0.008220
session=3 rows=21989 t_start=8501 t_end=228356 height_err: mean=0.000228 max=0.328045 vz_err: mean=0.000006 max=0.012097
```

## 结论

- 当前离线脚本已经能高精度复现现有双 TOF 融合逻辑。
- 之前出现的大误差不是融合算法本身先天对不上，而是由于日志里混入了 `nan` 脏行，导致时间切段错误。
- 后续无论是本地分析还是云端 AI 分析，都必须沿用这份清洗和切段规则，否则结论会被脏数据带偏。
