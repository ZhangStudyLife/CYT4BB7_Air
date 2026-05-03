#!/usr/bin/env python3
"""Pitch轴角速度环+角度环 深度分析"""

import numpy as np
import warnings
warnings.filterwarnings('ignore')

data = np.genfromtxt('D:/Car_Air_Protocol/CYT4BB7_Air/project/code/Estimation/Attitude/0502_pid/05030040.csv',
                     delimiter=',', skip_header=1, dtype=np.float32)

I_TIME = 0; I_ANG_TGT = 1; I_ANG_EULER = 2; I_GYRO_TGT = 3
I_GYRO_FILT = 4; I_GYRO_RAW = 5; I_P_TERM = 6; I_I_TERM = 7
I_D_TERM = 8; I_PID_OUT = 9; I_MOTOR_CMD = 10; I_THROTTLE = 11

N = len(data)
dt_ms = np.mean(np.diff(data[:, 0]))
print(f"总数据: {N}行, 间隔≈{dt_ms:.1f}ms ({1000/dt_ms:.0f}Hz)")

# === 时间戳跳跃分段 ===
times = data[:, I_TIME]
diffs = np.diff(times)
jump_threshold = 500
jumps = np.where(diffs > jump_threshold)[0]
print(f"\n时间跳跃(>{jump_threshold}ms): {len(jumps)}个")
for j in jumps:
    print(f"  行{j}: gap={diffs[j]:.0f}ms, t {times[j]:.0f}->{times[j+1]:.0f}, thr {data[j,I_THROTTLE]:.0f}->{data[j+1,I_THROTTLE]:.0f}")

bounds = [0] + [j+1 for j in jumps] + [N]

valid_segs = []
for i in range(len(bounds)-1):
    s, e = bounds[i], bounds[i+1]
    seg = data[s:e]
    if len(seg) < 100 or np.mean(seg[:, I_THROTTLE]) < 100:
        continue
    valid_segs.append((i+1, s, e))

print(f"\n有效飞行段: {len(valid_segs)}个")

# ============================================
# === 深度分析函数 ===
# ============================================

def estimate_angle_kp(seg):
    """估计角度环KP: gyro_target / (angle_target - euler_angle)"""
    ang_err = seg[:, I_ANG_TGT] - seg[:, I_ANG_EULER]
    gyro_tgt = seg[:, I_GYRO_TGT]
    # 只在有显著角度误差且角速度目标非零时估计
    valid = (np.abs(ang_err) > 0.5) & (np.abs(gyro_tgt) > 2)
    if np.sum(valid) > 20:
        ratios = gyro_tgt[valid] / (ang_err[valid] + 1e-4)
        valid_r = ratios[(ratios > 0) & (ratios < 20)]
        return np.median(valid_r) if len(valid_r) > 5 else 0
    return 0

def estimate_rate_kp_ki(seg):
    """估计角速度环KP和KI"""
    gyro_tgt = seg[:, I_GYRO_TGT]
    gyro_filt = seg[:, I_GYRO_FILT]
    gyro_err = gyro_tgt - gyro_filt
    p_term = seg[:, I_P_TERM]
    i_term = seg[:, I_I_TERM]
    pid_out = seg[:, I_PID_OUT]
    act = np.abs(pid_out) > 1.0

    if np.sum(act) < 20:
        return 0, 0

    # KP
    ratios = p_term[act] / (gyro_err[act] + 1e-4)
    valid = ratios[(ratios > 0) & (ratios < 50)]
    kp = np.median(valid) if len(valid) > 10 else 0

    # KI
    i_diff = np.diff(i_term)
    ki_vals = np.abs(i_diff) / (np.abs(gyro_err[:-1]) * dt_ms / 1000.0 + 1e-4)
    ki_vals = ki_vals[np.abs(ki_vals) < 10]
    ki = np.median(ki_vals) if len(ki_vals) > 10 else 0
    return kp, ki

def cross_corr_with_lag(x, y, max_lag=10):
    """计算互相关，返回最佳滞后和相关系数"""
    x = x - np.mean(x)
    y = y - np.mean(y)
    best_lag, best_corr = 0, 0
    for lag in range(-max_lag, max_lag+1):
        if lag < 0:
            corr = np.corrcoef(x[-lag:], y[:lag])[0,1] if abs(lag) < len(x)//2 else 0
        elif lag > 0:
            corr = np.corrcoef(x[:-lag], y[lag:])[0,1] if lag < len(x)//2 else 0
        else:
            corr = np.corrcoef(x, y)[0,1]
        if abs(corr) > abs(best_corr):
            best_corr = corr
            best_lag = lag
    return best_lag, best_corr

def analyze_phase_lag(seg, mask):
    """分析角速度环相位滞后：gyro_target → gyro_filt 的延迟"""
    gt = seg[mask, I_GYRO_TGT]
    gf = seg[mask, I_GYRO_FILT]
    if len(gt) < 50:
        return 0, 0, 0

    lag, corr = cross_corr_with_lag(gt, gf, max_lag=15)
    phase_lag_ms = abs(lag) * dt_ms  # 正lag表示实际滞后于目标

    # 在变化点的响应延迟
    d_tgt = np.abs(np.diff(gt, prepend=gt[0]))
    chg = d_tgt > 3
    if np.sum(chg) > 5:
        # 计算变化发生后实际到达目标90%的时间
        rise_times = []
        for idx in np.where(chg)[0]:
            if idx > 0 and idx < len(gt) - 20:
                tgt_val = gt[idx]
                for j in range(idx, min(idx+30, len(gt))):
                    if abs(gf[j] - tgt_val) < abs(tgt_val - gf[idx-1]) * 0.3:
                        rise_times.append((j-idx) * dt_ms)
                        break
        avg_rise = np.mean(rise_times) if rise_times else 0
    else:
        avg_rise = 0

    return lag * dt_ms, avg_rise, corr

def analyze_pi_cooperation(p_term, i_term, gyro_err, mask):
    """分析P和I是否在合作而非互相抵消"""
    p = p_term[mask]
    i = i_term[mask]
    e = gyro_err[mask]

    if len(p) < 10:
        return {}

    # P和I同号比例 (同号=合作，异号=打架)
    same_sign = np.sum(np.sign(p) == np.sign(i)) / len(p) if np.sum(np.abs(i) > 0.01) > 5 else 0

    # P和error的相关系数 (应该强正相关)
    p_e_corr = np.corrcoef(p, e)[0,1] if len(p) > 5 else 0

    # I和error积分的关系
    e_cumsum = np.cumsum(e) * dt_ms / 1000
    i_eint_corr = np.corrcoef(i, e_cumsum)[0,1] if len(i) > 5 else 0

    # I项是否在缩小误差 (看I变化与误差的关系)
    i_diff = np.diff(i, prepend=i[0])
    # I增长方向应该抵消误差：当error>0时I应该负向增长(如果I是反馈)
    i_correct_dir = np.sum(np.sign(i_diff) == -np.sign(e)) / len(e) if len(e) > 5 else 0

    # 最大P/最小P 比值（动态范围使用率）
    p_abs = np.abs(p)
    p_dynamic = np.percentile(p_abs, 95) / (np.percentile(p_abs, 5) + 1e-6)

    return {
        'pi_same_sign_pct': same_sign * 100,
        'p_error_corr': p_e_corr,
        'i_eint_corr': i_eint_corr,
        'i_correct_dir': i_correct_dir * 100,
        'p_dynamic_range': p_dynamic,
    }

def analyze_stick_quality(seg, mask):
    """分析打杆质量：是否平滑、是否有过多的急停急起"""
    ang_tgt = seg[mask, I_ANG_TGT]
    if len(ang_tgt) < 10:
        return {}

    # 角速度变化分布
    d_ang = np.diff(ang_tgt)
    d_ang = d_ang[np.abs(d_ang) > 1e-4]

    if len(d_ang) < 5:
        return {}

    # 急打杆比例 (角度变化 > 0.3°/sample 视为急打)
    jerk_ratio = np.sum(np.abs(d_ang) > 0.3) / len(d_ang) * 100

    # 打杆平滑度：连续同向 vs 频繁换向
    direction = np.sign(d_ang)
    dir_change = np.sum(np.diff(direction) != 0) / len(d_ang) * 100

    # 有效打杆路径：角速度目标峰值/角度目标峰值的比值
    ang_range = np.max(ang_tgt) - np.min(ang_tgt)
    gyro_range = np.max(np.abs(seg[mask, I_GYRO_TGT]))
    if ang_range > 1:
        effective_rate = gyro_range / ang_range
    else:
        effective_rate = 0

    return {
        'jerk_ratio': jerk_ratio,
        'dir_change_pct': dir_change,
        'ang_range': ang_range,
        'gyro_range': gyro_range,
        'effective_rate_ratio': effective_rate,
    }

# ============================================
# === 逐段深度分析 ===
# ============================================

all_summary = []

for seg_id, s, e in valid_segs:
    seg = data[s:e]
    dur = (seg[-1, I_TIME] - seg[0, I_TIME]) / 1000.0
    thr_mean = np.mean(seg[:, I_THROTTLE])

    # 基本估计
    ang_kp = estimate_angle_kp(seg)
    rate_kp, rate_ki = estimate_rate_kp_ki(seg)

    print(f"\n{'='*100}")
    print(f"段{seg_id}: 行[{s}-{e}], 时长{dur:.0f}s, 油门均{thr_mean:.0f}, 角度KP≈{ang_kp:.2f}, 角速度KP≈{rate_kp:.2f}, KI≈{rate_ki:.3f}")
    print(f"{'='*100}")

    # 工况划分
    ang_tgt = seg[:, I_ANG_TGT]
    gyro_tgt = seg[:, I_GYRO_TGT]
    static = (np.abs(ang_tgt) < 0.3) & (np.abs(gyro_tgt) < 2.0)
    small = (np.abs(ang_tgt) >= 0.3) & (np.abs(ang_tgt) < 10) & (np.abs(gyro_tgt) >= 2.0)
    large = np.abs(ang_tgt) >= 10

    # ---- 角度环分析 ----
    print(f"\n  [角度环] 角度跟踪分析")
    print(f"  {'工况':<15s} {'N':>6s} {'目标范围':>9s} {'实际范围':>9s} {'MAE':>8s} {'RMSE':>8s} {'Std':>8s} {'滞后ms':>8s}")
    print(f"  {'─'*15} {'─'*6} {'─'*9} {'─'*9} {'─'*8} {'─'*8} {'─'*8} {'─'*8}")

    for name, mask in [("静态", static), ("轻微(±10°)", small), ("大幅(>10°)", large)]:
        n = np.sum(mask)
        if n < 10:
            print(f"  {name:<15s} {n:>6d} (样本不足)")
            continue

        ang_err = seg[mask, I_ANG_TGT] - seg[mask, I_ANG_EULER]
        mae_ang = np.mean(np.abs(ang_err))
        rmse_ang = np.sqrt(np.mean(ang_err**2))
        std_ang = np.std(ang_err)
        tgt_range = np.max(seg[mask, I_ANG_TGT]) - np.min(seg[mask, I_ANG_TGT])
        act_range = np.max(seg[mask, I_ANG_EULER]) - np.min(seg[mask, I_ANG_EULER])
        ang_lag, _, ang_corr = analyze_phase_lag(seg, mask)

        print(f"  {name:<15s} {n:>6d} {tgt_range:>9.2f} {act_range:>9.2f} "
              f"{mae_ang:>8.2f} {rmse_ang:>8.2f} {std_ang:>8.2f} {ang_lag:>7.1f}ms")

    # ---- 角速度环分析 ----
    print(f"\n  [角速度环] 跟踪与PID贡献分析")
    print(f"  {'工况':<15s} {'N':>6s} {'MAE':>8s} {'RMSE':>8s} {'Std':>8s} {'Max':>8s} {'|P|':>8s} {'|I|':>8s} {'|D|':>8s} {'P%':>6s} {'I%':>6s}")

    gyro_filt = seg[:, I_GYRO_FILT]
    gyro_err_all = gyro_tgt - gyro_filt
    p_term = seg[:, I_P_TERM]
    i_term = seg[:, I_I_TERM]
    d_term = seg[:, I_D_TERM]
    pid_out = seg[:, I_PID_OUT]

    seg_results = {'seg_id': seg_id, 'dur': dur, 'ang_kp': ang_kp, 'rate_kp': rate_kp, 'rate_ki': rate_ki}

    for name, mask in [("静态", static), ("轻微(±10°)", small), ("大幅(>10°)", large)]:
        n = np.sum(mask)
        if n < 10:
            print(f"  {name:<15s} {n:>6d} (样本不足)")
            continue

        e_gyro = gyro_err_all[mask]
        mae_gyro = np.mean(np.abs(e_gyro))
        rmse_gyro = np.sqrt(np.mean(e_gyro**2))
        std_gyro = np.std(e_gyro)
        max_gyro = np.max(np.abs(e_gyro))

        p_abs = np.mean(np.abs(p_term[mask]))
        i_abs = np.mean(np.abs(i_term[mask]))
        d_abs = np.mean(np.abs(d_term[mask]))
        total = p_abs + i_abs + d_abs
        p_pct = p_abs/total*100 if total>0.001 else 0
        i_pct = i_abs/total*100 if total>0.001 else 0

        # 相位滞后
        gyro_lag, rise_t, gyro_corr = analyze_phase_lag(seg, mask)

        print(f"  {name:<15s} {n:>6d} {mae_gyro:>8.2f} {rmse_gyro:>8.2f} {std_gyro:>8.2f} "
              f"{max_gyro:>8.2f} {p_abs:>8.2f} {i_abs:>8.2f} {d_abs:>8.2f} {p_pct:>5.1f}% {i_pct:>5.1f}%")

        if name == "轻微(±10°)":
            # 详细P/I合作分析
            coop = analyze_pi_cooperation(p_term, i_term, gyro_err_all, mask)
            stick = analyze_stick_quality(seg, mask)

            print(f"\n  >>> 轻微(±10°)工况深度分析 <<<")
            print(f"  相位滞后: 互相关滞后={gyro_lag:.0f}ms, 阶跃响应上升时间≈{rise_t:.0f}ms, 相关系数={gyro_corr:.3f}")
            print(f"  P项动态范围: P95/P05={coop.get('p_dynamic_range',0):.1f}x")
            print(f"  P与误差相关性: r={coop.get('p_error_corr',0):.3f} (应≈1.0, P真正在对抗error)")
            print(f"  P与I同号比例: {coop.get('pi_same_sign_pct',0):.0f}% (应>80%, <50%说明P和I在打架)")
            print(f"  I与误差积分相关: r={coop.get('i_eint_corr',0):.3f} (I跟踪积分误差)")
            print(f"  I矫正方向正确率: {coop.get('i_correct_dir',0):.0f}% (应>80%)")
            print(f"  打杆急转比例: {stick.get('jerk_ratio',0):.0f}% (急打杆,低<20%更好)")
            print(f"  打杆方向切换率: {stick.get('dir_change_pct',0):.0f}% (频繁换向率)")
            print(f"  角度幅度: {stick.get('ang_range',0):.1f}°, 角速度峰值: {stick.get('gyro_range',0):.0f}°/s")

            seg_results.update({
                'mae_ang_small': np.mean(np.abs(seg[mask, I_ANG_TGT] - seg[mask, I_ANG_EULER])),
                'mae_gyro_small': mae_gyro,
                'std_gyro_small': std_gyro,
                'gyro_lag_ms': gyro_lag,
                'rise_time_ms': rise_t,
                'gyro_corr': gyro_corr,
                'p_pct': p_pct, 'i_pct': i_pct,
                'pi_same': coop.get('pi_same_sign_pct', 0),
                'p_err_corr': coop.get('p_error_corr', 0),
                'i_correct': coop.get('i_correct_dir', 0),
                'jerk': stick.get('jerk_ratio', 0),
                'dir_chg': stick.get('dir_change_pct', 0),
                'ang_range': stick.get('ang_range', 0),
            })

    all_summary.append(seg_results)

# ============================================
# === 跨段综合对比 ===
# ============================================
print(f"\n{'='*100}")
print(f"=== 跨段综合对比 (轻微(±10°)工况) ===")
print(f"{'='*100}")
print(f"  {'段':>3s} {'角度KP':>7s} {'速率KP':>7s} {'速率KI':>7s} {'角度MAE':>8s} {'角速MAE':>8s} "
      f"{'相位滞后':>8s} {'上升时间':>8s} {'P%':>6s} {'I%':>6s} {'P-I同号':>8s} {'P-Err相关':>9s} {'I-矫正%':>7s} {'急打%':>6s}")
print(f"  {'─'*3} {'─'*7} {'─'*7} {'─'*7} {'─'*8} {'─'*8} {'─'*8} {'─'*8} {'─'*6} {'─'*6} {'─'*8} {'─'*9} {'─'*7} {'─'*6}")

for s in all_summary:
    if s.get('mae_gyro_small') is None:
        continue
    print(f"  {s['seg_id']:>3d} {s['ang_kp']:>7.2f} {s['rate_kp']:>7.2f} {s['rate_ki']:>7.3f} "
          f"{s['mae_ang_small']:>8.2f} {s['mae_gyro_small']:>8.2f} "
          f"{s['gyro_lag_ms']:>7.0f}ms {s['rise_time_ms']:>6.0f}ms "
          f"{s['p_pct']:>5.1f}% {s['i_pct']:>5.1f}% {s['pi_same']:>7.1f}% {s['p_err_corr']:>8.3f} "
          f"{s['i_correct']:>6.1f}% {s['jerk']:>5.1f}%")

# ============================================
# === I项有效性专项分析 ===
# ============================================
print(f"\n{'='*100}")
print(f"=== I项(积分项)有效性专项分析 ===")
print(f"{'='*100}")

for s in all_summary:
    sid = s['seg_id']
    if s.get('rate_ki', 0) < 0.01:
        print(f"\n段{sid}: KI≈0 纯P/PD控制 — 无I项参与")
        continue

    print(f"\n段{sid}: KI≈{s['rate_ki']:.3f}")
    print(f"  P-I同号比例: {s.get('pi_same',0):.1f}% {'[OK] P和I方向一致,合作良好' if s.get('pi_same',0)>80 else '[!!] P和I方向经常相反,存在内耗!'}")
    print(f"  P与误差相关性: {s.get('p_err_corr',0):.3f} {'[OK] P项积极响应误差' if s.get('p_err_corr',0)>0.7 else '[!!] P项与误差脱节'}")
    print(f"  I矫正方向正确率: {s.get('i_correct',0):.1f}% {'[OK] I在正确方向消除误差' if s.get('i_correct',0)>80 else '[!!] I项方向混乱,可能是噪声导致'}")
    print(f"  相位滞后: {s.get('gyro_lag_ms',0):.0f}ms {'(可接受)' if s.get('gyro_lag_ms',0)<30 else '(偏大)'}")

# ============================================
# === 打杆分析 ===
# ============================================
print(f"\n{'='*100}")
print(f"=== 打杆质量分析 ===")
print(f"{'='*100}")

for s in all_summary:
    print(f"\n段{s['seg_id']}: 急打杆{s.get('jerk',0):.0f}%, 方向切换{s.get('dir_chg',0):.0f}%, "
          f"角度幅{s.get('ang_range',0):.1f}°")

# ============================================
# === 关键洞察与建议 ===
# ============================================
print(f"\n{'='*100}")
print(f"=== 关键发现与建议 ===")
print(f"{'='*100}")

# 找出最佳段
if all_summary:
    best_gyro = min(all_summary, key=lambda x: x.get('mae_gyro_small', 999))
    best_ang = min(all_summary, key=lambda x: x.get('mae_ang_small', 999))
    lowest_lag = min(all_summary, key=lambda x: abs(x.get('gyro_lag_ms', 999)))

    print(f"\n  最佳角速度跟踪: 段{best_gyro['seg_id']} (KP={best_gyro['rate_kp']:.2f}, KI={best_gyro['rate_ki']:.3f}, MAE={best_gyro.get('mae_gyro_small',0):.2f}°/s)")
    print(f"  最佳角度跟踪: 段{best_ang['seg_id']} (角度KP={best_ang['ang_kp']:.2f}, MAE={best_ang.get('mae_ang_small',0):.2f}°)")
    print(f"  最小相位滞后: 段{lowest_lag['seg_id']} ({lowest_lag.get('gyro_lag_ms',0):.0f}ms)")
