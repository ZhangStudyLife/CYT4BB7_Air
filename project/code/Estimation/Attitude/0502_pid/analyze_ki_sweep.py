#!/usr/bin/env python3
"""KI 扫描分析 - 系统性评估不同 KI 对 Pitch 角速度环的影响"""

import numpy as np
import warnings
warnings.filterwarnings('ignore')

data = np.genfromtxt('D:/Car_Air_Protocol/CYT4BB7_Air/project/code/Estimation/Attitude/0502_pid/05030147.csv',
                     delimiter=',', skip_header=1, dtype=np.float32)

I_TIME = 0; I_ANG_TGT = 1; I_ANG_EULER = 2; I_GYRO_TGT = 3
I_GYRO_FILT = 4; I_GYRO_RAW = 5; I_P_TERM = 6; I_I_TERM = 7
I_D_TERM = 8; I_PID_OUT = 9; I_MOTOR_CMD = 10; I_THROTTLE = 11

dt_ms = np.mean(np.diff(data[:, 0]))
print(f"总行数: {len(data)}, 采样率: {1000/dt_ms:.0f}Hz")

# 分段
times = data[:, 0]
diffs = np.diff(times)
jumps = np.where(diffs > 500)[0]
bounds = [0] + [j+1 for j in jumps] + [len(data)]

valid_segs = []
for i in range(len(bounds)-1):
    s, e = bounds[i], bounds[i+1]
    seg = data[s:e]
    if len(seg) >= 100 and np.mean(seg[:, I_THROTTLE]) >= 100:
        # Estimate KI
        act = np.abs(seg[:, I_PID_OUT]) > 1.0
        if np.sum(act) < 20: continue
        gt, gf = seg[:, I_GYRO_TGT], seg[:, I_GYRO_FILT]
        err = gt - gf
        i_term = seg[:, I_I_TERM]
        i_diff = np.diff(i_term)
        ki_vals = np.abs(i_diff) / (np.abs(err[:-1]) * dt_ms / 1000.0 + 1e-4)
        ki_vals = ki_vals[np.abs(ki_vals) < 10]
        ki = np.median(ki_vals) if len(ki_vals) > 10 else 0
        ratio = np.sum(np.abs(i_term[act])) / (np.sum(np.abs(seg[act, I_P_TERM])) + 1e-6)
        valid_segs.append((i+1, s, e, ki, ratio))

for sid, s, e, ki, ir in valid_segs:
    dur = (data[e-1, I_TIME] - data[s, I_TIME]) / 1000.0
    print(f"段{sid}: [{s}-{e}], {dur:.0f}s, KI={ki:.3f}, I/P幅值比={ir:.2f}")

def cross_corr_lag(x, y, max_lag=15):
    x, y = x - np.mean(x), y - np.mean(y)
    best = (0, 0)
    for lag in range(-max_lag, max_lag+1):
        if lag < 0: c = np.corrcoef(x[-lag:], y[:lag])[0,1] if abs(lag)<len(x)//2 else 0
        elif lag > 0: c = np.corrcoef(x[:-lag], y[lag:])[0,1] if lag<len(x)//2 else 0
        else: c = np.corrcoef(x, y)[0,1]
        if abs(c) > abs(best[1]): best = (lag*dt_ms, c)
    return best

all_summary = []

for sid, s, e, est_ki, i_p_ratio in valid_segs:
    seg = data[s:e]
    dur = (seg[-1, I_TIME] - seg[0, I_TIME]) / 1000.0
    thr = np.mean(seg[:, I_THROTTLE])

    ang_tgt = seg[:, I_ANG_TGT]; ang_euler = seg[:, I_ANG_EULER]
    gyro_tgt = seg[:, I_GYRO_TGT]; gyro_filt = seg[:, I_GYRO_FILT]; gyro_raw = seg[:, I_GYRO_RAW]
    p_term = seg[:, I_P_TERM]; i_term = seg[:, I_I_TERM]; d_term = seg[:, I_D_TERM]
    pid_out = seg[:, I_PID_OUT]; motor = seg[:, I_MOTOR_CMD]

    ang_err = ang_tgt - ang_euler
    gyro_err = gyro_tgt - gyro_filt

    # KP estimate
    act = np.abs(pid_out) > 1.0
    ratios = p_term[act] / (gyro_err[act] + 1e-4)
    valid = ratios[(ratios > 0) & (ratios < 50)]
    kp = np.median(valid) if len(valid) > 10 else 0

    # 工况
    static = (np.abs(ang_tgt) < 0.3) & (np.abs(gyro_tgt) < 2.0)
    small  = (np.abs(ang_tgt) >= 0.3) & (np.abs(ang_tgt) < 10) & (np.abs(gyro_tgt) >= 2.0)
    large  = np.abs(ang_tgt) >= 10
    # 新增: moderate工况 (2-5度，最典型的巡航打杆)
    moderate = (np.abs(ang_tgt) >= 0.3) & (np.abs(ang_tgt) < 5) & (np.abs(gyro_tgt) >= 2.0)

    print(f"\n{'='*100}")
    name_map = {0:'0', 0.043:'0.05', 0.088:'0.10', 0.131:'0.13', 0.173:'0.17', 0.220:'0.22'}
    ki_label = name_map.get(round(est_ki, 2), f'{est_ki:.2f}')
    print(f"段{sid}: KI={ki_label}, KP={kp:.2f}, I/P幅比={i_p_ratio:.2f}, {dur:.0f}s, 油门{thr:.0f}")
    print(f"{'='*100}")

    # ---- 角度环 ----
    print(f"\n  [角度环]")
    print(f"  {'工况':<14s} {'N':>6s} {'MAE°':>8s} {'RMSE':>8s} {'Std':>8s} {'Max':>8s} {'滞后':>7s}")
    for nm, mk in [("静态",static), ("中度+-5°",moderate), ("轻微+-10°",small), ("大幅>10°",large)]:
        n = np.sum(mk)
        if n < 10: continue
        ae = ang_err[mk]
        alag, _ = cross_corr_lag(ang_tgt[mk], ang_euler[mk], 15)
        print(f"  {nm:<14s} {n:>6d} {np.mean(np.abs(ae)):>8.2f} {np.sqrt(np.mean(ae**2)):>8.2f} "
              f"{np.std(ae):>8.2f} {np.max(np.abs(ae)):>8.2f} {abs(alag):>6.0f}ms")

    # ---- 角速度环 ----
    print(f"\n  [角速度环]")
    print(f"  {'工况':<14s} {'N':>6s} {'MAE':>8s} {'RMSE':>8s} {'Std':>8s} {'|P|':>8s} {'|I|':>8s} {'P%':>6s} {'I%':>6s} {'PID幅':>7s}")

    seg_info = {'seg_id': sid, 'ki': est_ki, 'kp': kp, 'i_p_ratio': i_p_ratio, 'dur': dur}

    for nm, mk in [("静态",static), ("中度+-5°",moderate), ("轻微+-10°",small), ("大幅>10°",large)]:
        n = np.sum(mk)
        if n < 20: continue
        e = gyro_err[mk]; p = p_term[mk]; i = i_term[mk]; o = pid_out[mk]
        mae = np.mean(np.abs(e)); rmse = np.sqrt(np.mean(e**2))
        p_abs = np.mean(np.abs(p)); i_abs = np.mean(np.abs(i))
        total = p_abs + i_abs + 1e-6
        p_pct = p_abs/total*100; i_pct = i_abs/total*100
        print(f"  {nm:<14s} {n:>6d} {mae:>8.2f} {rmse:>8.2f} {np.std(e):>8.2f} "
              f"{p_abs:>8.2f} {i_abs:>8.2f} {p_pct:>5.1f}% {i_pct:>5.1f}% {np.mean(np.abs(o)):>7.2f}")

    # ---- 深度P-I分析 (用小角度) ----
    nm, mk = ("中度+-5°", moderate) if np.sum(moderate) > 100 else ("轻微+-10°", small)
    if np.sum(mk) < 50: continue

    e = gyro_err[mk]; p = p_term[mk]; i = i_term[mk]; o = pid_out[mk]
    gt_mk = gyro_tgt[mk]; gf_mk = gyro_filt[mk]

    # 1. 基础指标
    mae = np.mean(np.abs(e)); rmse = np.sqrt(np.mean(e**2))

    # 2. P与error相关性
    p_err_r = np.corrcoef(p, e)[0,1]

    # 3. I矫正分析
    i_diff = np.diff(i, prepend=i[0])
    # I增长方向 vs error符号: I应该增长来减小error (如果I是正反馈加到PID输出)
    i_correct = np.sum(np.sign(i_diff) == -np.sign(e)) / len(e) * 100

    # 4. P-I合作
    pi_same = np.sum(np.sign(p) == np.sign(i)) / len(p) * 100 if np.sum(np.abs(i)>0.01)>5 else 0

    # 5. I净贡献
    i_net = np.mean(np.abs(p + i) - np.abs(p))

    # 6. 相位分析
    gyro_lag, gyro_corr = cross_corr_lag(gt_mk, gf_mk, 15)
    i_lag, i_corr = cross_corr_lag(np.abs(i_diff[:len(e)]), np.abs(e), 15)
    p_lag, p_corr = cross_corr_lag(np.abs(p), np.abs(e), 15)  # P相对err的延迟(应为0)

    # 7. I对角度闭环的真实贡献
    # 计算PID输出中I项能量的占比，以及在误差持续时I的增长率
    i_power = np.std(i) / (np.std(p) + 1e-6)  # I的波动幅度 vs P的波动幅度
    # I项在误差持续同号时的平均增长率
    err_sign = np.sign(e)
    sign_change = np.diff(err_sign, prepend=err_sign[0]) != 0
    # 找持续同号的段
    persistent = []
    run_start = 0
    for j in range(1, len(err_sign)):
        if sign_change[j]:
            run_len = j - run_start
            if run_len > 5 and np.abs(np.mean(e[run_start:j])) > 1:
                persistent.append((run_start, j))
            run_start = j
    if persistent:
        i_growth_rates = []
        for ps, pe in persistent:
            if pe < len(i_diff):
                di = i[pe-1] - i[ps]
                dt = (pe - ps) * dt_ms / 1000.0
                mean_err = np.mean(e[ps:pe])
                if abs(mean_err) > 0.5:
                    i_growth_rates.append(di / (mean_err * dt))
        if i_growth_rates:
            avg_i_growth = np.median(i_growth_rates)
        else:
            avg_i_growth = 0
    else:
        avg_i_growth = 0

    # 8. 阶跃响应: 在gyro_target突变时的I项行为
    d_gt = np.abs(np.diff(gt_mk, prepend=gt_mk[0]))
    steps = np.where(d_gt > 10)[0]  # gyro目标突变>10deg/s
    if len(steps) > 3:
        step_responses = []
        for st in steps:
            if st > 2 and st < len(e) - 30:
                # 阶跃前稳态误差
                pre_err = np.mean(np.abs(e[max(0,st-3):st]))
                # 阶跃后30个采样点内误差的衰减
                post_err = np.mean(np.abs(e[st+10:st+25])) if st+25 < len(e) else pre_err
                # I在阶跃前后的变化
                i_before = np.mean(i[max(0,st-3):st])
                i_after_15 = np.mean(i[st+10:st+20]) if st+20 < len(e) else i_before
                i_after_30 = np.mean(i[st+25:st+35]) if st+35 < len(e) else i_before
                step_responses.append({
                    'pre_err': pre_err, 'post_err': post_err,
                    'i_before': i_before, 'i_after_15': i_after_15, 'i_after_30': i_after_30,
                    'i_delta_15': i_after_15 - i_before,
                })
        if step_responses:
            avg_i_response = np.mean([r['i_delta_15'] for r in step_responses])
            avg_err_reduction = np.mean([r['post_err'] - r['pre_err'] for r in step_responses])
        else:
            avg_i_response = 0; avg_err_reduction = 0
    else:
        avg_i_response = 0; avg_err_reduction = 0

    print(f"\n  >>> P-I 合作深度分析 <<<")
    print(f"  P-Err相关系数: r={p_err_r:.4f}  (应>0.95)")
    print(f"  P-I同号比例: {pi_same:.1f}%  (理想>80%, <55%说明打架)")
    print(f"  I矫正方向正确率: {i_correct:.1f}%  (理想>70%)")
    print(f"  I净贡献: {i_net:+.2f}  (正=I增加总输出, 负=I抵消P)")
    print(f"  I功率比(I/P std): {i_power:.3f}  (应<0.5, 过大=I主导)")
    print(f"  角速度相位滞后: {abs(gyro_lag):.0f}ms, 相关系数={gyro_corr:.3f}")
    print(f"  I项相对误差相位滞后: {abs(i_lag):.0f}ms  (应为正值, 但过大会失效)")
    print(f"  P项相对误差相位滞后: {abs(p_lag):.0f}ms  (应≈0, P是同步的)")
    print(f"  I在持续误差下的增长率系数: {avg_i_growth:.4f}  (≈KI, 用于判断I是否过快/过慢)")
    if avg_err_reduction != 0:
        print(f"  阶跃后I对误差减小的贡献方向: {'正向帮助' if avg_err_reduction * avg_i_response < 0 else '可能反向阻碍'}")

    seg_info.update({
        'mae_ang_mod': np.mean(np.abs(ang_err[moderate])) if np.sum(moderate) > 20 else np.mean(np.abs(ang_err[small])),
        'mae_ang_small': np.mean(np.abs(ang_err[small])),
        'mae_gyro_mod': np.mean(np.abs(gyro_err[moderate])) if np.sum(moderate) > 20 else np.mean(np.abs(gyro_err[small])),
        'mae_gyro_small': np.mean(np.abs(gyro_err[small])),
        'mae_gyro_static': np.mean(np.abs(gyro_err[static])),
        'mae_gyro_large': np.mean(np.abs(gyro_err[large])) if np.sum(large) > 20 else 0,
        'mae_ang_static': np.mean(np.abs(ang_err[static])),
        'p_err_r': p_err_r, 'pi_same': pi_same, 'i_correct': i_correct,
        'i_net': i_net, 'i_power': i_power,
        'gyro_lag': abs(gyro_lag), 'gyro_corr': gyro_corr,
        'i_lag': abs(i_lag), 'p_lag': abs(p_lag),
        'i_growth': avg_i_growth,
    })
    all_summary.append(seg_info)

# ============ 综合对比 ============
print(f"\n{'='*100}")
print(f"=== KI 扫描综合对比 ===")
print(f"{'='*100}")
print(f"  {'KI':>6s} {'角度MAE':>9s} {'角速MAE':>9s} {'静态MAE':>9s} {'大幅MAE':>9s} "
      f"{'P-Err r':>8s} {'P-I同号':>8s} {'I矫正%':>7s} {'I净贡献':>8s} {'I功率比':>8s} "
      f"{'相位滞后':>8s} {'I滞后':>7s}")
print(f"  {'─'*6} {'─'*9} {'─'*9} {'─'*9} {'─'*9} {'─'*8} {'─'*8} {'─'*7} {'─'*8} {'─'*8} {'─'*8} {'─'*7}")
for s in all_summary:
    print(f"  {s['ki']:>6.3f} {s['mae_ang_mod']:>9.2f} {s['mae_gyro_mod']:>9.2f} "
          f"{s['mae_gyro_static']:>9.2f} {s.get('mae_gyro_large',0):>9.2f} "
          f"{s['p_err_r']:>8.3f} {s['pi_same']:>7.1f}% {s['i_correct']:>6.1f}% "
          f"{s['i_net']:>+7.2f} {s['i_power']:>8.3f} {s['gyro_lag']:>7.0f}ms {s['i_lag']:>6.0f}ms")

# ============ 最优KI分析 ============
print(f"\n{'='*100}")
print(f"=== 最优KI推荐 ===")
print(f"{'='*100}")

if all_summary:
    # 按角速MAE排序
    by_gyro = sorted(all_summary, key=lambda x: x['mae_gyro_mod'])
    by_ang = sorted(all_summary, key=lambda x: x['mae_ang_mod'])
    by_i_ok = sorted(all_summary, key=lambda x: x['i_correct'], reverse=True)  # I矫正率最高
    by_pi_coop = sorted(all_summary, key=lambda x: x['pi_same'], reverse=True)  # P-I合作最好

    ranks = []
    for s in by_gyro: ranks.append((s['ki'], s['mae_gyro_mod']))
    print(f"\n  按角速度MAE排名: {[(round(k,3), round(m,2)) for k,m in ranks]}")
    ranks = []
    for s in by_ang: ranks.append((s['ki'], s['mae_ang_mod']))
    print(f"  按角度MAE排名:   {[(round(k,3), round(m,2)) for k,m in ranks]}")
    ranks = []
    for s in by_i_ok: ranks.append((s['ki'], s['i_correct']))
    print(f"  按I矫正率排名:   {[(round(k,3), round(m,1)) for k,m in ranks]}")
    ranks = []
    for s in by_pi_coop: ranks.append((s['ki'], s['pi_same']))
    print(f"  按P-I合作排名:   {[(round(k,3), round(m,1)) for k,m in ranks]}")

    best_gyro = by_gyro[0]
    worst_gyro = by_gyro[-1]
    print(f"\n  最佳KI={best_gyro['ki']:.3f}: 角速MAE={best_gyro['mae_gyro_mod']:.2f} deg/s, "
          f"I矫正={best_gyro['i_correct']:.1f}%, P-I同号={best_gyro['pi_same']:.1f}%")
    print(f"  最差KI={worst_gyro['ki']:.3f}: 角速MAE={worst_gyro['mae_gyro_mod']:.2f} deg/s")
