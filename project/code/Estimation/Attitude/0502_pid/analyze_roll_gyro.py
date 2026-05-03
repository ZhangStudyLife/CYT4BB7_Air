#!/usr/bin/env python3
"""分析 Roll 角速度环 PID - 通过时间戳跳跃分段"""

import numpy as np
import warnings
warnings.filterwarnings('ignore')

data = np.genfromtxt('D:/Car_Air_Protocol/CYT4BB7_Air/project/code/Estimation/Attitude/0502_pid/05022358.csv',
                     delimiter=',', skip_header=1, dtype=np.float32)

I_TIME = 0; I_ROLL_TGT = 1; I_ROLL_EULER = 2; I_GYRO_TGT = 3
I_GYRO_FILT = 4; I_GYRO_RAW = 5; I_P_TERM = 6; I_I_TERM = 7
I_D_TERM = 8; I_PID_OUT = 9; I_MOTOR_CMD = 10; I_THROTTLE = 11

dt_ms = np.mean(np.diff(data[:, 0]))
print(f"总数据: {len(data)}行, 间隔≈{dt_ms:.1f}ms ({1000/dt_ms:.0f}Hz)")

# 分段
times = data[:, I_TIME]
diffs = np.diff(times)
jumps = np.where(diffs > 500)[0]
bounds = [0] + [j+1 for j in jumps] + [len(data)]

# 跳过地面段和太短的段
valid_segs = []
for i in range(len(bounds)-1):
    s, e = bounds[i], bounds[i+1]
    seg = data[s:e]
    if len(seg) < 100 or np.mean(seg[:, I_THROTTLE]) < 100:
        continue
    valid_segs.append((i+1, s, e))

print(f"有效飞行段: {len(valid_segs)}个\n")

# 收集所有段的摘要
all_summary = []

for seg_id, s, e in valid_segs:
    seg = data[s:e]
    t_start = seg[0, I_TIME] / 1000.0
    t_end = seg[-1, I_TIME] / 1000.0
    dur = t_end - t_start
    thr = np.mean(seg[:, I_THROTTLE])

    roll_tgt = seg[:, I_ROLL_TGT]
    gyro_tgt = seg[:, I_GYRO_TGT]
    gyro_filt = seg[:, I_GYRO_FILT]
    gyro_raw = seg[:, I_GYRO_RAW]
    p_term = seg[:, I_P_TERM]
    i_term = seg[:, I_I_TERM]
    d_term = seg[:, I_D_TERM]
    pid_out = seg[:, I_PID_OUT]
    motor = seg[:, I_MOTOR_CMD]
    gyro_err = gyro_tgt - gyro_filt

    # 估计 KP
    act = np.abs(pid_out) > 1.0
    n_act = np.sum(act)
    ratios = p_term[act] / (gyro_err[act] + 1e-4)
    valid_kp = ratios[(ratios > 0) & (ratios < 50)]
    kp = np.median(valid_kp) if len(valid_kp) > 10 else 0

    # 估计 KI: I项变化率
    i_diff = np.diff(i_term)
    ki_ratios = np.abs(i_diff) / (np.abs(gyro_err[:-1]) * dt_ms / 1000.0 + 1e-4)
    valid_ki = ki_ratios[np.abs(ki_ratios) < 10]
    ki = np.median(valid_ki) if len(valid_ki) > 10 else 0

    print(f"段{seg_id}: 行[{s}-{e}], 时长{dur:.0f}s, 油门{thr:.0f}, KP≈{kp:.3f}, KI≈{ki:.3f}")

    # 动静分类
    static_mask = (np.abs(roll_tgt) < 0.3) & (np.abs(gyro_tgt) < 2.0)
    small_mask  = (np.abs(roll_tgt) >= 0.3) & (np.abs(roll_tgt) < 10) & (np.abs(gyro_tgt) >= 2.0)
    large_mask  = np.abs(roll_tgt) >= 10

    row_data = {'mae_static': 0, 'mae_small': 0, 'mae_large': 0,
                'std_static': 0, 'std_small': 0, 'n_static': 0, 'n_small': 0, 'n_large': 0,
                'p_pct': 0, 'i_pct': 0, 'd_pct': 0, 'pid_mag': 0, 'osc': 0, 'resp': 0}
    print(f"  {'工况':<15s} {'N':>6s} {'MAE':>8s} {'RMSE':>8s} {'Std':>8s} {'MaxErr':>8s} {'P%':>6s} {'I%':>6s} {'D%':>6s} {'|P|':>8s} {'|PID|':>8s}")
    print(f"  {'─'*15} {'─'*6} {'─'*8} {'─'*8} {'─'*8} {'─'*8} {'─'*6} {'─'*6} {'─'*6} {'─'*8} {'─'*8}")

    for name, mask in [("静态", static_mask), ("轻微(±10°)", small_mask), ("大幅(>10°)", large_mask)]:
        n = np.sum(mask)
        if n < 10:
            print(f"  {name:<15s} {n:>6d} (样本不足)")
            continue

        err = gyro_err[mask]
        p = p_term[mask]
        i = i_term[mask]
        d = d_term[mask]
        o = pid_out[mask]

        mae = np.mean(np.abs(err))
        rmse = np.sqrt(np.mean(err**2))
        std_e = np.std(err)
        max_e = np.max(np.abs(err))
        p_abs = np.mean(np.abs(p))
        i_abs = np.mean(np.abs(i))
        d_abs = np.mean(np.abs(d))
        total = p_abs + i_abs + d_abs
        p_pct = p_abs/total*100 if total>0.001 else 0
        i_pct = i_abs/total*100 if total>0.001 else 0
        d_pct = d_abs/total*100 if total>0.001 else 0
        pid_mag = np.mean(np.abs(o))

        # 震荡指数
        osc = np.std(o) / (np.mean(np.abs(o)) + 1e-6)

        # 响应延迟
        if name == "轻微(±10°)":
            chg = np.abs(np.diff(gyro_tgt[mask], prepend=gyro_tgt[mask][0])) > 3
            if np.sum(chg) > 5:
                actual_chg = np.abs(np.diff(gyro_filt[mask], prepend=gyro_filt[mask][0]))
                tgt_chg = np.abs(np.diff(gyro_tgt[mask], prepend=gyro_tgt[mask][0]))
                resp_ratio = np.mean(actual_chg[chg]) / (np.mean(tgt_chg[chg]) + 1e-6)
            else:
                resp_ratio = 1.0
        else:
            resp_ratio = 0

        print(f"  {name:<15s} {n:>6d} {mae:>8.2f} {rmse:>8.2f} {std_e:>8.2f} {max_e:>8.2f} "
              f"{p_pct:>5.1f}% {i_pct:>5.1f}% {d_pct:>5.1f}% {p_abs:>8.2f} {pid_mag:>8.2f}")

        if name == "轻微(±10°)":
            row_data.update({
                'n_small': n, 'mae_small': mae, 'rmse_small': rmse, 'std_small': std_e,
                'max_small': max_e, 'p_pct': p_pct, 'i_pct': i_pct, 'd_pct': d_pct,
                'pid_mag': pid_mag, 'osc': osc, 'resp': resp_ratio,
            })
        if name == "静态":
            row_data['mae_static'] = mae
            row_data['std_static'] = std_e
            row_data['n_static'] = n
        if name == "大幅(>10°)":
            row_data['mae_large'] = mae
            row_data['n_large'] = n

    row_data.update({'seg_id': seg_id, 'dur': dur, 'kp': kp, 'ki': ki, 'thr': thr})
    all_summary.append(row_data)
    print()

# === 综合对比 ===
print(f"\n{'='*100}")
print(f"{'段':>4s} {'时长':>5s} {'KP':>7s} {'KI':>7s} {'静态MAE':>9s} {'轻微MAE':>9s} {'大幅MAE':>9s} {'轻微Std':>9s} {'P%':>6s} {'I%':>6s} {'PID幅':>8s} {'振荡':>6s}")
print(f"{'─'*100}")
for s in all_summary:
    print(f"  {s['seg_id']:>3d} {s['dur']:>4.0f}s {s['kp']:>7.2f} {s['ki']:>7.3f} "
          f"{s['mae_static']:>9.2f} {s['mae_small']:>9.2f} {s['mae_large']:>9.2f} "
          f"{s['std_small']:>9.2f} {s['p_pct']:>5.1f}% {s['i_pct']:>5.1f}% {s['pid_mag']:>8.2f} {s['osc']:>5.2f}")

# === 关键洞察 ===
print(f"\n{'='*100}")
print("=== 关键分析 ===")
print(f"{'='*100}")

# 按轻微MAE排序
by_small_mae = sorted(all_summary, key=lambda x: x['mae_small'])
print(f"\n> 轻微姿态(±10°)跟踪 MAE 排名:")
for i, s in enumerate(by_small_mae):
    print(f"  {i+1}. 段{s['seg_id']}: KP={s['kp']:.2f}, KI={s['ki']:.3f}, MAE={s['mae_small']:.2f}°/s, "
          f"P占{s['p_pct']:.0f}%, PID幅={s['pid_mag']:.1f}, 振荡={s['osc']:.2f}")

# 按静态MAE排序
by_static_mae = sorted(all_summary, key=lambda x: x['mae_static'])
print(f"\n> 静态悬停 MAE 排名:")
for i, s in enumerate(by_static_mae):
    print(f"  {i+1}. 段{s['seg_id']}: KP={s['kp']:.2f}, KI={s['ki']:.3f}, MAE={s['mae_static']:.2f}°/s")

# KP vs MAE关系
print(f"\n> KP对跟踪效果的影响:")
print(f"  {'KP范围':>10s} {'静态MAE':>9s} {'轻微MAE':>9s} {'大幅MAE':>9s} {'PID幅值':>8s}")
kps = np.array([s['kp'] for s in all_summary])
for lo, hi in [(3.5, 3.7), (4.4, 4.6), (4.9, 5.1)]:
    group = [s for s in all_summary if lo <= s['kp'] <= hi]
    if group:
        avg_static = np.mean([s['mae_static'] for s in group])
        avg_small = np.mean([s['mae_small'] for s in group])
        avg_large = np.mean([s['mae_large'] for s in group])
        avg_pid = np.mean([s['pid_mag'] for s in group])
        print(f"  KP≈{np.mean([s['kp'] for s in group]):.1f}      {avg_static:>9.2f} {avg_small:>9.2f} {avg_large:>9.2f} {avg_pid:>8.1f}")

# KI分析
print(f"\n> KI对跟踪效果的影响 (KP≈4.5):")
kp45 = [s for s in all_summary if 4.4 <= s['kp'] <= 4.6]
for s in kp45:
    print(f"  KI={s['ki']:.3f}: 静态MAE={s['mae_static']:.2f}, 轻微MAE={s['mae_small']:.2f}, "
          f"大幅MAE={s['mae_large']:.2f}, PID幅={s['pid_mag']:.1f}, 振荡={s['osc']:.2f}")

# 纯P vs P+I 对比
print(f"\n> 纯P(无I) vs P+I:")
pure_p = [s for s in all_summary if s['ki'] < 0.01]
with_i = [s for s in all_summary if s['ki'] > 0.1]
if pure_p:
    avg = {k: np.mean([s[k] for s in pure_p]) for k in ['mae_static','mae_small','mae_large','osc']}
    print(f"  纯P({len(pure_p)}段): 静态MAE={avg['mae_static']:.2f}, 轻微MAE={avg['mae_small']:.2f}, "
          f"大幅MAE={avg['mae_large']:.2f}, 振荡={avg['osc']:.2f}")
if with_i:
    avg = {k: np.mean([s[k] for s in with_i]) for k in ['mae_static','mae_small','mae_large','osc']}
    print(f"  P+I({len(with_i)}段): 静态MAE={avg['mae_static']:.2f}, 轻微MAE={avg['mae_small']:.2f}, "
          f"大幅MAE={avg['mae_large']:.2f}, 振荡={avg['osc']:.2f}")
