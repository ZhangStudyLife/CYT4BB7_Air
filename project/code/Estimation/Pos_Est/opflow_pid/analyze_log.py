#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Comprehensive analysis of quadcopter speed-loop flight log 05061440.csv"""
import numpy as np
import sys, os

# Force UTF-8 output
if sys.platform == 'win32':
    os.system('')

data = np.loadtxt(r"D:\Car_Air_Protocol\CYT4BB7_Air\project\code\Estimation\Pos_Est\opflow_pid\05061440.csv",
                  delimiter=',', skiprows=1)
N = len(data)
dt = 0.02  # 50Hz
t = np.arange(N) * dt

cols = 'I0 I1 I2 I3 I4 I5 I6 I7 I8 I9 I10 I11 I12 I13'.split()
I = {c: data[:, i] for i, c in enumerate(cols)}

I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13 = [I[c] for c in cols]

velx_target = -I7   # desired x velocity (cm/s), sign reversed for analysis
vely_target = -I9   # desired y velocity (cm/s)
Pos_Est_vel_x = I6  # fused velocity estimate x (cm/s)
Pos_Est_vel_y = I8  # fused velocity estimate y (cm/s)
opflow_vel_x = I2   # raw optical flow velocity x (cm/s)
opflow_vel_y = I3   # raw optical flow velocity y (cm/s)
acc_x = I4          # horizontal accel x (cm/s^2)
acc_y = I5          # horizontal accel y (cm/s^2)
roll_target = I10   # roll angle target (deg)
roll_actual = I11   # roll angle actual (deg)
pitch_target = I12  # pitch angle target (deg)
pitch_actual = I13  # pitch angle actual (deg)

# =====================================================
# Helper: cross-correlation for delay estimation
# =====================================================
def estimate_delay(sig1, sig2, dt):
    """Estimate delay between two signals via cross-correlation."""
    s1 = sig1 - sig1.mean()
    s2 = sig2 - sig2.mean()
    n = len(s1)
    # FFT-based cross-correlation
    f1 = np.fft.rfft(s1, n=2*n)
    f2 = np.fft.rfft(s2, n=2*n)
    corr = np.fft.irfft(f1 * np.conj(f2), n=2*n)[:n]
    # corr = np.correlate(s1, s2, mode='same')
    # Use direct for simplicity
    corr = np.correlate(s1, s2, mode='full')
    mid = len(corr) // 2
    corr_centered = corr[mid - n//2: mid + n//2 + 1]
    lag = np.argmax(corr_centered) - n//2
    return lag * dt * 1000  # ms

# =====================================================
# Print helper
# =====================================================
sq = '^2'  # superscript 2 replacement

# =====================================================
# 0. BASIC DATA CHECK
# =====================================================
print("=" * 70)
print("0. BASIC DATA CHECK")
print("=" * 70)
print(f"Total frames: {N}, duration: {N*dt:.1f}s")

# Check data sanity
for c in cols:
    d = I[c]
    n_nan = np.isnan(d).sum()
    if n_nan > 0:
        print(f"  {c}: {n_nan} NaN values!")
print(f"Roll target limit: [{roll_target.min():.1f}, {roll_target.max():.1f}] deg")
print(f"Pitch target limit: [{pitch_target.min():.1f}, {pitch_target.max():.1f}] deg")
print(f"Roll actual range: [{roll_actual.min():.1f}, {roll_actual.max():.1f}] deg")
print(f"Pitch actual range: [{pitch_actual.min():.1f}, {pitch_actual.max():.1f}] deg")

# =====================================================
# 1. OPTICAL FLOW RAW STATISTICS
# =====================================================
print("\n" + "=" * 70)
print("1. OPTICAL FLOW RAW DATA (I0,I1) - 像素积分值")
print("=" * 70)

flow_valid = (np.abs(I0) > 1e-6) | (np.abs(I1) > 1e-6)
flow_zero = ~flow_valid
print(f"有效光流帧: {flow_valid.sum()} ({100*flow_valid.sum()/N:.1f}%)")
print(f"零值光流帧: {flow_zero.sum()} ({100*flow_zero.sum()/N:.1f}%)")

# Dropout analysis
dropout_starts, dropout_ends = [], []
in_dropout = False
start_idx = 0
for i in range(N):
    if flow_zero[i] and not in_dropout:
        in_dropout = True
        start_idx = i
    elif (not flow_zero[i]) and in_dropout:
        in_dropout = False
        dropout_starts.append(start_idx)
        dropout_ends.append(i-1)
if in_dropout:
    dropout_starts.append(start_idx)
    dropout_ends.append(N-1)

durations = [(e-s+1)*dt for s, e in zip(dropout_starts, dropout_ends)]
short_drops = sum(1 for d in durations if d < 0.1)
med_drops = sum(1 for d in durations if 0.1 <= d < 1.0)
long_drops = sum(1 for d in durations if d >= 1.0)

print(f"掉线次数: {len(dropout_starts)}")
print(f"  极短掉线 (<0.1s): {short_drops} 次")
print(f"  中等掉线 (0.1-1s): {med_drops} 次")
print(f"  长掉线 (>=1s): {long_drops} 次")

if long_drops > 0:
    print(f"\n  长掉线详情:")
    count = 0
    for (s, e), dur in zip(zip(dropout_starts, dropout_ends), durations):
        if dur >= 1.0:
            print(f"    t=[{s*dt:.1f}-{e*dt:.1f}s], dur={dur:.2f}s ({e-s+1} frames)")
            count += 1
            if count >= 8:
                break

# Valid flow statistics
vI0, vI1 = I0[flow_valid], I1[flow_valid]
print(f"\n有效I0: mean={vI0.mean():.1f}, std={vI0.std():.1f}, max={vI0.max():.0f}, min={vI0.min():.0f}")
print(f"有效I1: mean={vI1.mean():.1f}, std={vI1.std():.1f}, max={vI1.max():.0f}, min={vI1.min():.0f}")

# Consecutive valid frames
cons = []
cnt = 0
for i in range(N):
    if flow_valid[i]:
        cnt += 1
    else:
        if cnt > 0:
            cons.append(cnt)
        cnt = 0
if cnt > 0:
    cons.append(cnt)
if cons:
    cons = np.array(cons)
    print(f"连续有效帧: mean={cons.mean():.0f}, median={np.median(cons):.0f}, max={cons.max()}")

# Max pixel integral per frame
print(f"\nI0 像素积分范围: [{vI0.min():.0f}, {vI0.max():.0f}]")
print(f"I1 像素积分范围: [{vI1.min():.0f}, {vI1.max():.0f}]")

# =====================================================
# 2. OPTICAL FLOW VELOCITY vs FUSED ESTIMATE
# =====================================================
print("\n" + "=" * 70)
print("2. 光流速度 vs 融合估计速度")
print("=" * 70)

valid_f = flow_valid & (np.abs(opflow_vel_x) < 500) & (np.abs(opflow_vel_y) < 500)

print(f"--- X轴 (I2=opflow_vel_x, I6=Pos_Est_vel_x) ---")
print(f"  opflow_vel_x  均值={opflow_vel_x[valid_f].mean():.2f}, 标准差={opflow_vel_x[valid_f].std():.2f} cm/s")
print(f"  Pos_Est_vel_x 均值={Pos_Est_vel_x[valid_f].mean():.2f}, 标准差={Pos_Est_vel_x[valid_f].std():.2f} cm/s")
print(f"  光流速度噪声(std): {opflow_vel_x[valid_f].std():.2f} cm/s")
print(f"  融合速度平滑度(std): {Pos_Est_vel_x[valid_f].std():.2f} cm/s")
print(f"  平滑比率: {Pos_Est_vel_x[valid_f].std()/max(opflow_vel_x[valid_f].std(),0.1):.2f}x")

print(f"\n--- Y轴 (I3=opflow_vel_y, I8=Pos_Est_vel_y) ---")
print(f"  opflow_vel_y  均值={opflow_vel_y[valid_f].mean():.2f}, 标准差={opflow_vel_y[valid_f].std():.2f} cm/s")
print(f"  Pos_Est_vel_y 均值={Pos_Est_vel_y[valid_f].mean():.2f}, 标准差={Pos_Est_vel_y[valid_f].std():.2f} cm/s")
print(f"  光流速度噪声(std): {opflow_vel_y[valid_f].std():.2f} cm/s")
print(f"  融合速度平滑度(std): {Pos_Est_vel_y[valid_f].std():.2f} cm/s")

# Frame-to-frame noise
flow_dx = np.diff(opflow_vel_x)
flow_dy = np.diff(opflow_vel_y)
est_dx = np.diff(Pos_Est_vel_x)
est_dy = np.diff(Pos_Est_vel_y)
valid_diff = flow_valid[1:] & flow_valid[:-1]

print(f"\n--- 帧间变化(std) ---")
print(f"  光流X帧差(std): {flow_dx[valid_diff].std():.2f} cm/s")
print(f"  融合X帧差(std): {est_dx.std():.2f} cm/s")
print(f"  光流Y帧差(std): {flow_dy[valid_diff].std():.2f} cm/s")
print(f"  融合Y帧差(std): {est_dy.std():.2f} cm/s")
print(f"  融合/光流噪声比 X: {est_dx.std()/max(flow_dx[valid_diff].std(),0.01):.2f}")
print(f"  融合/光流噪声比 Y: {est_dy.std()/max(flow_dy[valid_diff].std(),0.01):.2f}")

# =====================================================
# 3. ACCELERATION ANALYSIS
# =====================================================
print("\n" + "=" * 70)
print("3. 加速度数据 (I4=acc_x, I5=acc_y)")
print("=" * 70)
print(f"acc_x: mean={acc_x.mean():.1f}, std={acc_x.std():.1f}, max={acc_x.max():.0f}, min={acc_x.min():.0f} cm/s{sq}")
print(f"acc_y: mean={acc_y.mean():.1f}, std={acc_y.std():.1f}, max={acc_y.max():.0f}, min={acc_y.min():.0f} cm/s{sq}")

acc_noise = max(acc_x.std(), acc_y.std())
print(f"\n加速度噪声(std): {acc_noise:.1f} cm/s{sq}")
print(f"单帧(20ms)纯惯性积分漂移: ~{acc_noise*0.02:.1f} cm/s")
print(f"10帧(200ms)随机游走漂移: ~{acc_noise*0.02*np.sqrt(10):.1f} cm/s")
print(f"50帧(1s)随机游走漂移: ~{acc_noise*0.02*np.sqrt(50):.1f} cm/s")

# Mean bias in acceleration
print(f"\n加速度均值(偏置): X={acc_x.mean():.1f}, Y={acc_y.mean():.1f} cm/s{sq}")
print(f"如果5Hz低通滤波未能完全消除偏置，1s积分漂移可达: X={abs(acc_x.mean())*1.0:.1f} cm/s")

# =====================================================
# 4. HOVER PERFORMANCE
# =====================================================
print("\n" + "=" * 70)
print("4. 悬停性能分析 (目标速度=0)")
print("=" * 70)

target_zero_thresh = 6  # cm/s
hover_x = np.abs(velx_target) < target_zero_thresh
hover_y = np.abs(vely_target) < target_zero_thresh
hover_both = hover_x & hover_y

# Continuous hover segments
h_starts, h_ends = [], []
in_h = False
h_s = 0
for i in range(N):
    if hover_both[i] and not in_h:
        in_h = True
        h_s = i
    elif (not hover_both[i]) and in_h:
        in_h = False
        h_starts.append(h_s)
        h_ends.append(i-1)
if in_h:
    h_ends.append(N-1)
    h_starts.append(h_s)

long_h = [(s, e) for s, e in zip(h_starts, h_ends) if (e-s) >= 50]

print(f"悬停段总数(>1s): {len(long_h)}")
for i, (s, e) in enumerate(long_h[:12]):
    dur = (e - s) * dt
    vx_m = Pos_Est_vel_x[s:e+1].mean()
    vy_m = Pos_Est_vel_y[s:e+1].mean()
    vx_s = Pos_Est_vel_x[s:e+1].std()
    vy_s = Pos_Est_vel_y[s:e+1].std()
    # Position drift in this segment
    px_drift = Pos_Est_vel_x[s:e+1].sum() * dt * 0.01  # m
    py_drift = Pos_Est_vel_y[s:e+1].sum() * dt * 0.01
    # Check target angle directions
    rt_m = roll_target[s:e+1].mean()
    pt_m = pitch_target[s:e+1].mean()
    print(f"  H{i+1}: [{s*dt:.1f}-{e*dt:.1f}s] {dur:.1f}s | "
          f"vx={vx_m:+.1f}±{vx_s:.1f} vy={vy_m:+.1f}±{vy_s:.1f} cm/s | "
          f"dX={px_drift:+.1f}m dY={py_drift:+.1f}m | "
          f"rt={rt_m:+.1f} pt={pt_m:+.1f}")

total_hover_frames = hover_both.sum()
print(f"\n总悬停帧数: {total_hover_frames} ({100*total_hover_frames/N:.1f}% of flight, {total_hover_frames*dt:.1f}s)")
if total_hover_frames > 10:
    hvx = Pos_Est_vel_x[hover_both]
    hvy = Pos_Est_vel_y[hover_both]
    print(f"悬停期间 Pos_Est_vel_x: mean={hvx.mean():.2f}, std={hvx.std():.2f} cm/s")
    print(f"悬停期间 Pos_Est_vel_y: mean={hvy.mean():.2f}, std={hvy.std():.2f} cm/s")

    # Position drift: integrate residual velocity
    px = np.cumsum(hvx) * dt * 0.01  # m
    py = np.cumsum(hvy) * dt * 0.01
    print(f"累计位置漂移 X: {px[-1]:.2f}m (over {total_hover_frames*dt:.1f}s)")
    print(f"累计位置漂移 Y: {py[-1]:.2f}m")
    print(f"平均漂移速度 X: {px[-1]/(total_hover_frames*dt)*100:.1f} cm/s")
    print(f"平均漂移速度 Y: {py[-1]/(total_hover_frames*dt)*100:.1f} cm/s")

    # Breakdown: is velocity biased in one direction consistently?
    pos_x_pct = (hvx > 0).mean() * 100
    pos_y_pct = (hvy > 0).mean() * 100
    print(f"正向速度占比: X={pos_x_pct:.0f}%, Y={pos_y_pct:.0f}% (50%=无偏)")

# =====================================================
# 5. ANGLE TRACKING
# =====================================================
print("\n" + "=" * 70)
print("5. 角度跟踪质量")
print("=" * 70)

roll_err = roll_target - roll_actual
pitch_err = pitch_target - pitch_actual
r_active = np.abs(roll_target) > 0.5
p_active = np.abs(pitch_target) > 0.5

print(f"--- ROLL ---")
print(f"  目标范围: [{roll_target.min():.1f}, {roll_target.max():.1f}] deg")
print(f"  实测范围: [{roll_actual.min():.1f}, {roll_actual.max():.1f}] deg")
print(f"  跟踪误差(全部): mean={roll_err.mean():.2f}, std={roll_err.std():.2f} deg")
print(f"  跟踪误差(active): mean={roll_err[r_active].mean():.2f}, std={roll_err[r_active].std():.2f} deg")
# Over/undershoot analysis
if r_active.sum() > 50:
    roll_overshoot = (np.abs(roll_actual) - np.abs(roll_target))[r_active]
    roll_over_pct = (roll_overshoot > 1.0).mean() * 100
    roll_under_pct = (roll_overshoot < -1.0).mean() * 100
    print(f"  超调(>1deg): {roll_over_pct:.0f}%, 欠调(<-1deg): {roll_under_pct:.0f}%")

# Cross-correlation delay
if len(roll_target) > 100:
    seg_s, seg_e = N//4, N//4 + 500
    delay_r = estimate_delay(roll_target[seg_s:seg_e], roll_actual[seg_s:seg_e], dt)
    print(f"  估计延迟(互相关): {delay_r:.0f}ms")

print(f"\n--- PITCH ---")
print(f"  目标范围: [{pitch_target.min():.1f}, {pitch_target.max():.1f}] deg")
print(f"  实测范围: [{pitch_actual.min():.1f}, {pitch_actual.max():.1f}] deg")
print(f"  跟踪误差(全部): mean={pitch_err.mean():.2f}, std={pitch_err.std():.2f} deg")
print(f"  跟踪误差(active): mean={pitch_err[p_active].mean():.2f}, std={pitch_err[p_active].std():.2f} deg")
if p_active.sum() > 50:
    pitch_overshoot = (np.abs(pitch_actual) - np.abs(pitch_target))[p_active]
    pitch_over_pct = (pitch_overshoot > 1.0).mean() * 100
    pitch_under_pct = (pitch_overshoot < -1.0).mean() * 100
    print(f"  超调(>1deg): {pitch_over_pct:.0f}%, 欠调(<-1deg): {pitch_under_pct:.0f}%")
if len(pitch_target) > 100:
    delay_p = estimate_delay(pitch_target[seg_s:seg_e], pitch_actual[seg_s:seg_e], dt)
    print(f"  估计延迟(互相关): {delay_p:.0f}ms")

# =====================================================
# 6. SPEED PI ANALYSIS
# =====================================================
print("\n" + "=" * 70)
print("6. 速度环PI分析")
print("=" * 70)

Kp, Ki = 0.12, 0.030
err_x = velx_target + Pos_Est_vel_x  # error = target_positive + est_velocity
err_y = vely_target + Pos_Est_vel_y

# Reconstruct integral term: target_angle = Kp*error + Ki*integral(error)dt
integral_x = roll_target - Kp * err_x
integral_y = pitch_target - Kp * err_y

print(f"--- X轴 ---")
print(f"  速度误差(目标+估计): mean={err_x.mean():.2f}, std={err_x.std():.2f} cm/s")
print(f"  重构积分项: mean={integral_x.mean():.2f}, std={integral_x.std():.2f} deg")
print(f"  积分范围: [{integral_x.min():.2f}, {integral_x.max():.2f}] deg")
print(f"  积分饱和帧(>=4.4 deg): {(np.abs(integral_x) >= 4.4).sum()} / {N} ({(np.abs(integral_x)>=4.4).sum()/N*100:.1f}%)")
# When specifically hovering and velocity err is small, is integral building up?
hover_int_x = integral_x[hover_both]
print(f"  悬停时积分项: mean={hover_int_x.mean():.2f}, std={hover_int_x.std():.2f} deg")

print(f"\n--- Y轴 ---")
print(f"  速度误差(目标+估计): mean={err_y.mean():.2f}, std={err_y.std():.2f} cm/s")
print(f"  重构积分项: mean={integral_y.mean():.2f}, std={integral_y.std():.2f} deg")
print(f"  积分范围: [{integral_y.min():.2f}, {integral_y.max():.2f}] deg")
print(f"  积分饱和帧(>=4.4 deg): {(np.abs(integral_y) >= 4.4).sum()} / {N} ({(np.abs(integral_y)>=4.4).sum()/N*100:.1f}%)")
hover_int_y = integral_y[hover_both]
print(f"  悬停时积分项: mean={hover_int_y.mean():.2f}, std={hover_int_y.std():.2f} deg")

# =====================================================
# 7. DIRECTION CORRECTNESS
# =====================================================
print("\n" + "=" * 70)
print("7. 控制方向正确性检查")
print("=" * 70)

# Forward velocity (vx>0, 机头向前) should cause negative pitch (nose-up to brake)
mask = (Pos_Est_vel_x > 5) & (np.abs(pitch_target) > 0.1)
if mask.sum() > 0:
    pct = (pitch_target[mask] < 0).mean() * 100
    print(f"  vx>5cm/s 时 pitch_target<0 (刹车正确): {pct:.0f}% ({mask.sum()} frames)")

# Right velocity (vy>0) should cause positive roll (roll right to brake)
mask = (Pos_Est_vel_y > 5) & (np.abs(roll_target) > 0.1)
if mask.sum() > 0:
    pct = (roll_target[mask] > 0).mean() * 100
    print(f"  vy>5cm/s 时 roll_target>0 (刹车正确): {pct:.0f}% ({mask.sum()} frames)")

# Reverse check: negative velocity
mask = (Pos_Est_vel_x < -5) & (np.abs(pitch_target) > 0.1)
if mask.sum() > 0:
    pct = (pitch_target[mask] > 0).mean() * 100
    print(f"  vx<-5cm/s 时 pitch_target>0 (刹车正确): {pct:.0f}% ({mask.sum()} frames)")

mask = (Pos_Est_vel_y < -5) & (np.abs(roll_target) > 0.1)
if mask.sum() > 0:
    pct = (roll_target[mask] < 0).mean() * 100
    print(f"  vy<-5cm/s 时 roll_target<0 (刹车正确): {pct:.0f}% ({mask.sum()} frames)")

# =====================================================
# 8. DROPOUT IMPACT - PURE INERTIAL
# =====================================================
print("\n" + "=" * 70)
print("8. 光流掉线时纯惯性估计表现")
print("=" * 70)

# Find significant dropouts
sig_drops = [(s, e) for s, e in zip(dropout_starts, dropout_ends) if (e-s) >= 5]
print(f"显著掉线(>=5帧, >100ms): {len(sig_drops)} 次")

for i, (s, e) in enumerate(sig_drops[:8]):
    dur = (e - s) * dt
    vx_pre = Pos_Est_vel_x[max(0,s-1)]
    vx_post = Pos_Est_vel_x[min(N-1,e+1)]
    vy_pre = Pos_Est_vel_y[max(0,s-1)]
    vy_post = Pos_Est_vel_y[min(N-1,e+1)]
    # Velocity evolution during dropout
    vx_dur = Pos_Est_vel_x[s:e+1]
    vy_dur = Pos_Est_vel_y[s:e+1]
    print(f"  D{i+1}: [{s*dt:.1f}-{e*dt:.1f}s] {dur:.2f}s | "
          f"vx: {vx_pre:+.1f} -> {vx_post:+.1f} (drift={vx_post-vx_pre:+.1f}) | "
          f"vy: {vy_pre:+.1f} -> {vy_post:+.1f} (drift={vy_post-vy_pre:+.1f}) | "
          f"accX: {acc_x[s:e+1].mean():.0f}, accY: {acc_y[s:e+1].mean():.0f}")

# =====================================================
# 9. STICK RESPONSE
# =====================================================
print("\n" + "=" * 70)
print("9. 打杆响应分析")
print("=" * 70)

stick_x = np.abs(velx_target) > 30
stick_y = np.abs(vely_target) > 30

if stick_x.sum() > 50:
    print(f"--- X轴 (velx_target>30cm/s, {stick_x.sum()} frames) ---")
    print(f"  目标速度 mean: {velx_target[stick_x].mean():.1f} cm/s")
    print(f"  估计速度 mean: {Pos_Est_vel_x[stick_x].mean():.1f} cm/s")
    print(f"  跟踪误差 mean: {(velx_target[stick_x] + Pos_Est_vel_x[stick_x]).mean():.1f} cm/s")

if stick_y.sum() > 50:
    print(f"--- Y轴 (vely_target>30cm/s, {stick_y.sum()} frames) ---")
    print(f"  目标速度 mean: {vely_target[stick_y].mean():.1f} cm/s")
    print(f"  估计速度 mean: {Pos_Est_vel_y[stick_y].mean():.1f} cm/s")
    print(f"  跟踪误差 mean: {(vely_target[stick_y] + Pos_Est_vel_y[stick_y]).mean():.1f} cm/s")

# =====================================================
# 10. PER-QUARTER ANALYSIS
# =====================================================
print("\n" + "=" * 70)
print("10. 分时段分析 (按时间四等分)")
print("=" * 70)

for q in range(4):
    s = q * (N//4)
    e = min((q+1)*(N//4), N)
    print(f"\n  Q{q+1}: [{s*dt:.1f}-{e*dt:.1f}s]")
    qh = hover_both[s:e]
    if qh.sum() > 10:
        print(f"    悬停: {qh.sum()} frames, vx={Pos_Est_vel_x[s:e][qh].mean():.1f}+/-{Pos_Est_vel_x[s:e][qh].std():.1f} cm/s, vy={Pos_Est_vel_y[s:e][qh].mean():.1f}+/-{Pos_Est_vel_y[s:e][qh].std():.1f} cm/s")
    print(f"    积分项X: {integral_x[s:e].mean():.2f} deg, 积分项Y: {integral_y[s:e].mean():.2f} deg")
    print(f"    Roll跟踪误差: {roll_err[s:e].std():.2f} deg, Pitch跟踪误差: {pitch_err[s:e].std():.2f} deg")

# =====================================================
# 11. K=0.06 FUSION GAIN ANALYSIS
# =====================================================
print("\n" + "=" * 70)
print("11. 融合增益 K=0.06 分析")
print("=" * 70)

K = 0.06
# When flow is valid, estimate how much correction is being applied
div_x = Pos_Est_vel_x[valid_f] - opflow_vel_x[valid_f]
div_y = Pos_Est_vel_y[valid_f] - opflow_vel_y[valid_f]

# Note: we see Pos_Est_vel, not vel_pred separately.
# But the correction term is K * (opflow_lpf - vel_pred)
# Pos_Est = vel_pred + K*(opflow_lpf - vel_pred)
# So opflow_lpf - vel_pred = (Pos_Est - vel_pred) / K
# This means the correction term per frame = K * (opflow_lpf - vel_pred)
# The effective correction is limited by K

# The key insight: K=0.06 means <94% inertial, 6% optical flow>
# Time constant of convergence: tau = dt / K = 0.02 / 0.06 = 0.333s
tau = dt / K
print(f"融合时间常数: tau = dt/K = {dt}/{K} = {tau:.2f}s")
print(f"这意味着光流修正需要约 {tau*3:.1f}s (3*tau) 才能修正63%的误差")
print(f"")

# For different error magnitudes, convergence time
for err_mag in [10, 20, 50, 100]:
    # Time to reduce error to 37% (one time constant)
    # Correction per frame = K * err_mag
    # Number of frames to converge = err_mag / (K * err_mag) * some_factor
    # Actually: error decays as e^(-K * n), where n is number of frames
    # Time to 90% convergence: n = -ln(0.1)/K = 2.3/0.06 = 38 frames = 0.77s
    n_90 = -np.log(0.1) / K
    t_90 = n_90 * dt
    correction_per_frame = K * err_mag
    print(f"  {err_mag}cm/s 速度误差 -> 每帧修正{correction_per_frame:.1f}cm/s -> 90%收敛需{n_90:.0f}帧({t_90:.1f}s)")

print(f"\n光流噪声(std) X: {opflow_vel_x[valid_f].std():.1f} cm/s, Y: {opflow_vel_y[valid_f].std():.1f} cm/s")
print(f"融合后噪声(std) X: {Pos_Est_vel_x[valid_f].std():.1f} cm/s, Y: {Pos_Est_vel_y[valid_f].std():.1f} cm/s")

# If we double K to 0.12:
for test_k in [0.08, 0.12, 0.2, 0.4]:
    tau_test = dt / test_k
    t90_test = -np.log(0.1) / test_k * dt
    print(f"  K={test_k:.2f}: tau={tau_test:.2f}s, 90%收敛={t90_test:.1f}s ({-np.log(0.1)/test_k:.0f}帧)")

# =====================================================
# 12. OPTICAL FLOW SPIKE / JUMP ANALYSIS
# =====================================================
print("\n" + "=" * 70)
print("12. 光流跳变检测")
print("=" * 70)

# Large frame-to-frame changes
for thresh in [50, 100, 200]:
    n_x = (np.abs(flow_dx) > thresh).sum()
    n_y = (np.abs(flow_dy) > thresh).sum()
    print(f"  帧间变化>{thresh}cm/s: X={n_x}次, Y={n_y}次")

# Check if opflow_vel stays at same value (stale data)
# When I0/I1=0 but opflow_vel stays at previous value (4Hz LPF holding)
stale = []
for i in range(1, N):
    if flow_zero[i] and flow_valid[i-1]:
        if np.abs(opflow_vel_x[i] - opflow_vel_x[i-1]) < 0.01:
            stale.append(i)
print(f"\n  光流掉线但输出值不变(旧值保持)的帧: {len(stale)}")

# =====================================================
# 13. OSCILLATION / SPECTRAL ANALYSIS
# =====================================================
print("\n" + "=" * 70)
print("13. 振荡频谱分析 (最长悬停段)")
print("=" * 70)

# Find longest hover segment
best_len = 0
best_seg = None
for s, e in zip(h_starts, h_ends):
    if e - s > best_len:
        best_len = e - s
        best_seg = (s, e)

if best_seg and best_len > 200:
    s, e = best_seg
    print(f"最长悬停段: [{s*dt:.1f}-{e*dt:.1f}s], {best_len}帧 ({best_len*dt:.1f}s)")

    for name, sig in [("Roll实测", roll_actual[s:e+1]), ("Pitch实测", pitch_actual[s:e+1]),
                       ("Roll目标", roll_target[s:e+1]), ("Pitch目标", pitch_target[s:e+1]),
                       ("Vel_X", Pos_Est_vel_x[s:e+1]), ("Vel_Y", Pos_Est_vel_y[s:e+1])]:
        sig_d = sig - sig.mean()
        fft = np.abs(np.fft.rfft(sig_d))
        freqs = np.fft.rfftfreq(len(sig_d), dt)
        valid = freqs > 0.05  # ignore DC and very low freq
        if valid.sum() > 0:
            peak_idx = np.argmax(fft[valid])
            peak_f = freqs[valid][peak_idx]
            peak_amp = fft[valid][peak_idx] / len(sig_d) * 2
            # RMS
            rms = np.sqrt(np.mean(sig_d**2))
            print(f"  {name:12s}: 主频={peak_f:.2f}Hz, 振幅={peak_amp:.2f}, RMS={rms:.2f}")
else:
    print("无足够长的悬停段用于频谱分析")

# =====================================================
# 14. SUMMARY AND RECOMMENDATIONS
# =====================================================
print("\n" + "=" * 70)
print("14. 综合诊断总结")
print("=" * 70)

# Velocity bias during hover
hvx_bias = Pos_Est_vel_x[hover_both].mean()
hvy_bias = Pos_Est_vel_y[hover_both].mean()
hvx_std = Pos_Est_vel_x[hover_both].std()
hvy_std = Pos_Est_vel_y[hover_both].std()

# Integral term at hover
h_int_x = integral_x[hover_both].mean()
h_int_y = integral_y[hover_both].mean()

print(f"""
【速度估计质量】
  光流有效率: {100*flow_valid.sum()/N:.1f}%
  光流噪声(1-sigma): X={opflow_vel_x[valid_f].std():.1f}, Y={opflow_vel_y[valid_f].std():.1f} cm/s
  融合后噪声(1-sigma): X={Pos_Est_vel_x[valid_f].std():.1f}, Y={Pos_Est_vel_y[valid_f].std():.1f} cm/s
  加速度噪声(1-sigma): {acc_noise:.0f} cm/s^2
  融合时间常数: {tau:.2f}s (K=0.06)

【悬停残留速度】
  均值偏置: X={hvx_bias:.2f}, Y={hvy_bias:.2f} cm/s
  波动(std): X={hvx_std:.2f}, Y={hvy_std:.2f} cm/s
  位置漂移速率: X={Pos_Est_vel_x[hover_both].sum()*dt*0.01/(total_hover_frames*dt)*100:.1f}, Y={Pos_Est_vel_y[hover_both].sum()*dt*0.01/(total_hover_frames*dt)*100:.1f} cm/s

【PI积分项】
  悬停时均值: X={h_int_x:.2f}, Y={h_int_y:.2f} deg (限幅±4.5 deg)
  饱和比例: X={(np.abs(integral_x)>=4.4).sum()/N*100:.1f}%, Y={(np.abs(integral_y)>=4.4).sum()/N*100:.1f}%

【角度跟踪】
  Roll 误差(std): {roll_err.std():.2f} deg
  Pitch 误差(std): {pitch_err.std():.2f} deg

【控制方向】
  已在Section 7中验证
""")
