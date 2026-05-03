#!/usr/bin/env python3
"""Roll轴深度回顾分析 - 检查P/I是否真正起正向作用"""

import numpy as np
import warnings
warnings.filterwarnings('ignore')

data = np.genfromtxt('D:/Car_Air_Protocol/CYT4BB7_Air/project/code/Estimation/Attitude/0502_pid/05022358.csv',
                     delimiter=',', skip_header=1, dtype=np.float32)

I_TIME = 0; I_ANG_TGT = 1; I_ANG_EULER = 2; I_GYRO_TGT = 3
I_GYRO_FILT = 4; I_GYRO_RAW = 5; I_P_TERM = 6; I_I_TERM = 7
I_D_TERM = 8; I_PID_OUT = 9; I_MOTOR_CMD = 10; I_THROTTLE = 11

N = len(data)
dt_ms = np.mean(np.diff(data[:, 0]))
print(f"Roll日志: {N}行, 采样率{1000/dt_ms:.0f}Hz")

# 分段
times = data[:, I_TIME]
diffs = np.diff(times)
jumps = np.where(diffs > 500)[0]
bounds = [0] + [j+1 for j in jumps] + [N]

print(f"时间跳跃>{500}ms: {len(jumps)}个")

valid_segs = []
for i in range(len(bounds)-1):
    s, e = bounds[i], bounds[i+1]
    seg = data[s:e]
    if len(seg) >= 100 and np.mean(seg[:, I_THROTTLE]) >= 100:
        valid_segs.append((i+1, s, e))

print(f"有效段: {len(valid_segs)}个\n")

def estimate_angle_kp(seg):
    ang_err = seg[:, I_ANG_TGT] - seg[:, I_ANG_EULER]
    gyro_tgt = seg[:, I_GYRO_TGT]
    valid = (np.abs(ang_err) > 0.5) & (np.abs(gyro_tgt) > 2)
    if np.sum(valid) > 20:
        ratios = gyro_tgt[valid] / (ang_err[valid] + 1e-4)
        valid_r = ratios[(ratios > 0) & (ratios < 20)]
        return np.median(valid_r) if len(valid_r) > 5 else 0
    return 0

def estimate_rate_kp_ki(seg):
    gt, gf = seg[:, I_GYRO_TGT], seg[:, I_GYRO_FILT]
    err = gt - gf
    pt, it, po = seg[:, I_P_TERM], seg[:, I_I_TERM], seg[:, I_PID_OUT]
    act = np.abs(po) > 1.0
    if np.sum(act) < 20:
        return 0, 0
    ratios = pt[act] / (err[act] + 1e-4)
    valid = ratios[(ratios > 0) & (ratios < 50)]
    kp = np.median(valid) if len(valid) > 10 else 0
    idiff = np.diff(it)
    kiv = np.abs(idiff) / (np.abs(err[:-1]) * dt_ms / 1000.0 + 1e-4)
    kiv = kiv[np.abs(kiv) < 10]
    ki = np.median(kiv) if len(kiv) > 10 else 0
    return kp, ki

def cross_corr_with_lag(x, y, max_lag=15):
    x, y = x - np.mean(x), y - np.mean(y)
    best_lag, best_corr = 0, 0
    for lag in range(-max_lag, max_lag+1):
        if lag < 0: corr = np.corrcoef(x[-lag:], y[:lag])[0,1] if abs(lag) < len(x)//2 else 0
        elif lag > 0: corr = np.corrcoef(x[:-lag], y[lag:])[0,1] if lag < len(x)//2 else 0
        else: corr = np.corrcoef(x, y)[0,1]
        if abs(corr) > abs(best_corr): best_corr, best_lag = corr, lag
    return best_lag * dt_ms, best_corr

all_summary = []

for seg_id, s, e in valid_segs:
    seg = data[s:e]
    dur = (seg[-1, I_TIME] - seg[0, I_TIME]) / 1000.0
    thr_mean = np.mean(seg[:, I_THROTTLE])

    ang_kp = estimate_angle_kp(seg)
    rate_kp, rate_ki = estimate_rate_kp_ki(seg)

    ang_tgt = seg[:, I_ANG_TGT]
    gyro_tgt = seg[:, I_GYRO_TGT]
    gyro_filt = seg[:, I_GYRO_FILT]
    p_term = seg[:, I_P_TERM]
    i_term = seg[:, I_I_TERM]
    d_term = seg[:, I_D_TERM]
    pid_out = seg[:, I_PID_OUT]
    gyro_err = gyro_tgt - gyro_filt
    motor_cmd = seg[:, I_MOTOR_CMD]

    # 工况
    static = (np.abs(ang_tgt) < 0.3) & (np.abs(gyro_tgt) < 2.0)
    small  = (np.abs(ang_tgt) >= 0.3) & (np.abs(ang_tgt) < 10) & (np.abs(gyro_tgt) >= 2.0)
    large  = np.abs(ang_tgt) >= 10

    print(f"{'='*100}")
    print(f"段{seg_id}: [{s}-{e}], {dur:.0f}s, 油门{thr_mean:.0f}, 角度KP={ang_kp:.2f}, 速率KP={rate_kp:.2f}, KI={rate_ki:.3f}")
    print(f"{'='*100}")

    print(f"\n  [角度环]")
    print(f"  {'工况':<15s} {'N':>6s} {'MAE':>8s} {'RMSE':>8s} {'Std':>8s} {'Max':>8s} {'滞后':>7s}")
    for name, mask in [("静态", static), ("轻微+-10", small), ("大幅>10", large)]:
        n = np.sum(mask)
        if n < 10: continue
        ae = ang_tgt[mask] - seg[mask, I_ANG_EULER]
        lag, _ = cross_corr_with_lag(ang_tgt[mask], seg[mask, I_ANG_EULER], 15)
        print(f"  {name:<15s} {n:>6d} {np.mean(np.abs(ae)):>8.2f} {np.sqrt(np.mean(ae**2)):>8.2f} "
              f"{np.std(ae):>8.2f} {np.max(np.abs(ae)):>8.2f} {abs(lag):>6.0f}ms")

    print(f"\n  [角速度环]")
    print(f"  {'工况':<15s} {'N':>6s} {'MAE':>8s} {'RMSE':>8s} {'Std':>8s} {'|P|':>8s} {'|I|':>8s} {'|D|':>8s} {'P%':>6s} {'I%':>6s} {'PID幅':>8s}")
    seg_info = {'seg_id': seg_id, 'dur': dur, 'ang_kp': ang_kp, 'rate_kp': rate_kp, 'rate_ki': rate_ki}

    for name, mask in [("静态", static), ("轻微+-10", small), ("大幅>10", large)]:
        n = np.sum(mask)
        if n < 20: continue
        e = gyro_err[mask]
        p = p_term[mask]; i = i_term[mask]; d = d_term[mask]; o = pid_out[mask]
        mae = np.mean(np.abs(e)); rmse = np.sqrt(np.mean(e**2))
        p_abs = np.mean(np.abs(p)); i_abs = np.mean(np.abs(i)); d_abs = np.mean(np.abs(d))
        total = p_abs + i_abs + d_abs
        p_pct = p_abs/total*100 if total>0.001 else 0
        i_pct = i_abs/total*100 if total>0.001 else 0
        print(f"  {name:<15s} {n:>6d} {mae:>8.2f} {rmse:>8.2f} {np.std(e):>8.2f} "
              f"{p_abs:>8.2f} {i_abs:>8.2f} {d_abs:>8.2f} {p_pct:>5.1f}% {i_pct:>5.1f}% {np.mean(np.abs(o)):>8.2f}")

        if name == "轻微+-10":
            # P-I合作分析
            act_mask = np.abs(p) > 0.1
            if np.sum(act_mask) > 10:
                # P和I同号比例
                same_sign = np.sum(np.sign(p[act_mask]) == np.sign(i[act_mask])) / np.sum(act_mask) if np.sum(np.abs(i[act_mask]) > 0.01) > 5 else 0
                # I矫正方向正确率: dI/dt的符号应该和误差符号相反(如果I是负反馈)
                i_full = i_term[mask]
                i_diff = np.diff(i_full, prepend=i_full[0])
                i_correct = np.sum(np.sign(i_diff) == -np.sign(e)) / len(e)
                # P与error相关
                p_err_corr = np.corrcoef(p, e)[0,1]

                # 相位滞后
                gyro_lag, gyro_corr = cross_corr_with_lag(gyro_tgt[mask], gyro_filt[mask], 15)

                # I-ERR滞后分析: I项变化 vs 误差 的相位关系
                i_lag, i_corr = cross_corr_with_lag(np.abs(i_diff), np.abs(e), 15)

                # D项分析
                d_err_corr = np.corrcoef(d, np.diff(e, prepend=e[0]))[0,1] if np.sum(np.abs(d)) > 0.01 else 0

                # 电机cmd与PID输出差异 (电机混控非线性)
                motor_pid_corr = np.corrcoef(o, motor_cmd[mask])[0,1]
                motor_pid_ratio = np.mean(np.abs(motor_cmd[mask])) / (np.mean(np.abs(o)) + 1e-6)

                print(f"\n  >>> P-I-D合作深度分析 <<<")
                print(f"  P-误差相关系数: r={p_err_corr:.4f} {'[OK] P积极响应' if p_err_corr>0.8 else '[!!] P与误差脱节'}")
                print(f"  P-I同号比例: {same_sign*100:.1f}% {'[OK] 合作' if same_sign>0.8 else '[!!] 经常打架' if same_sign<0.55 else '[~] 临界'}")
                print(f"  I矫正方向: {i_correct*100:.1f}% {'[OK]' if i_correct>0.8 else '[!!] 近乎随机' if i_correct<0.1 else '[~] 部分有效'}")
                print(f"  I变化-误差相关系数: r={i_corr:.4f} (应为正，I滞后误差)")
                print(f"  I变化相对误差的相位滞后: {abs(i_lag):.0f}ms (应为正值，I在误差之后反应)")
                print(f"  D-误差微分的相关系数: r={d_err_corr:.4f} (应为正，D预测误差变化)")
                print(f"  角速度相位滞后: {abs(gyro_lag):.0f}ms, 互相关r={gyro_corr:.3f}")
                print(f"  电机/PID幅值比: {motor_pid_ratio:.2f} (混控缩放)")
                print(f"  电机-PID相关系数: r={motor_pid_corr:.4f} (应~1.0)")

                # 检查I项是否在"帮倒忙"：把I项置零后理论上MAE的变化
                # I项帮助 = (|P+I| - |P|) 的平均值
                help_metric = np.mean(np.abs(p + i) - np.abs(p))
                print(f"  I项净贡献: {help_metric:+.2f} (正=I增加控制力度, 负=I抵消P)")

                seg_info.update({
                    'mae_gyro': mae, 'mae_ang': np.mean(np.abs(ang_tgt[mask] - seg[mask, I_ANG_EULER])),
                    'p_pct': p_pct, 'i_pct': i_pct,
                    'same_sign': same_sign*100, 'i_correct': i_correct*100,
                    'p_err_corr': p_err_corr, 'i_lag_ms': abs(i_lag),
                    'gyro_lag_ms': abs(gyro_lag), 'gyro_corr': gyro_corr,
                    'help_metric': help_metric,
                    'd_err_corr': d_err_corr,
                    'motor_pid_ratio': motor_pid_ratio,
                })

    all_summary.append(seg_info)

# === 综合对比 ===
print(f"\n{'='*100}")
print(f"=== ROLL轴综合对比 ===")
print(f"{'='*100}")
print(f"  {'段':>3s} {'角度KP':>7s} {'速率KP':>7s} {'速率KI':>7s} {'角度MAE':>8s} {'角速MAE':>8s} "
      f"{'P%':>6s} {'I%':>6s} {'P-Err':>7s} {'P-I同号':>8s} {'I矫正':>7s} {'I帮助':>7s} {'相位':>6s}")
print(f"  {'─'*3} {'─'*7} {'─'*7} {'─'*7} {'─'*8} {'─'*8} {'─'*6} {'─'*6} {'─'*7} {'─'*8} {'─'*7} {'─'*7} {'─'*6}")

for s in all_summary:
    if s.get('mae_gyro') is None: continue
    print(f"  {s['seg_id']:>3d} {s['ang_kp']:>7.2f} {s['rate_kp']:>7.2f} {s['rate_ki']:>7.3f} "
          f"{s['mae_ang']:>8.2f} {s['mae_gyro']:>8.2f} "
          f"{s['p_pct']:>5.1f}% {s['i_pct']:>5.1f}% {s['p_err_corr']:>6.3f} "
          f"{s['same_sign']:>7.1f}% {s['i_correct']:>6.1f}% {s['help_metric']:>+6.1f} {s['gyro_lag_ms']:>5.0f}ms")

# === 按KI分组对比 ===
print(f"\n{'='*100}")
print(f"=== KI效应专项 ===")
print(f"{'='*100}")

no_i = [s for s in all_summary if s.get('rate_ki', 0) < 0.01]
with_i = [s for s in all_summary if s.get('rate_ki', 0) >= 0.01]

for group, name in [(no_i, "纯P(无I)"), (with_i, "P+I")]:
    if group:
        avg_mae = np.mean([s['mae_gyro'] for s in group])
        avg_ang = np.mean([s['mae_ang'] for s in group])
        print(f"\n  {name} ({len(group)}段):")
        print(f"    角速度MAE: {avg_mae:.2f} deg/s")
        print(f"    角度MAE:   {avg_ang:.2f} deg")
        if name == "P+I":
            avg_same = np.mean([s['same_sign'] for s in group])
            avg_icorr = np.mean([s['i_correct'] for s in group])
            avg_help = np.mean([s['help_metric'] for s in group])
            print(f"    P-I同号:   {avg_same:.1f}%")
            print(f"    I矫正率:   {avg_icorr:.1f}% {'[!!] 几乎无效' if avg_icorr < 10 else ''}")
            print(f"    I净贡献:   {avg_help:+.1f}")
