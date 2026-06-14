import csv
import math
from pathlib import Path

import numpy as np
from scipy import signal


BASE = Path(__file__).resolve().parent
FILES = [
    ("F1", "第一次飞行.csv", 0.12, 0.02),
    ("F2", "第二次飞行.csv", 0.12, 0.00),
    ("F3", "第三次飞行.csv", 0.18, 0.00),
    ("F4", "第四次飞行.csv", 0.12, 0.04),
    ("F5", "第五次飞行.csv", 0.12, 0.06),
    ("F6", "第六次飞行.csv", 0.16, 0.00),
    ("F7", "第七次飞行.csv", 0.16, 0.03),
]


def pct(a, p):
    a = np.asarray(a)
    if a.size == 0:
        return float("nan")
    return float(np.percentile(a, p))


def rms(a):
    a = np.asarray(a)
    if a.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(a * a)))


def segments(mask, t, min_dur=0.0):
    out = []
    start = None
    for i, ok in enumerate(mask):
        if ok and start is None:
            start = i
        if ((not ok) or (i == len(mask) - 1)) and start is not None:
            end = i if not ok else i + 1
            if end > start and (t[end - 1] - t[start]) >= min_dur:
                out.append((start, end))
            start = None
    return out


def load_mode7(path):
    rows = []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            if len(row) < 20:
                continue
            v = [float(x) for x in row[:20]]
            if int(round(v[1])) == 7:
                rows.append(v)
    a = np.asarray(rows, dtype=float)
    a[:, 0] = (a[:, 0] - a[0, 0]) * 0.001
    return a


def resample(t, y, fs=100.0):
    if len(t) < 4 or t[-1] <= t[0]:
        return None, None
    tu = np.arange(t[0], t[-1], 1.0 / fs)
    if len(tu) < 8:
        return None, None
    return tu, np.interp(tu, t, y)


def highpass_rms(t, y, fs=100.0, cutoff=4.0):
    vals = []
    dom = []
    for s, e in segments(np.ones_like(t, dtype=bool), t, 2.0):
        tu, yu = resample(t[s:e], y[s:e], fs)
        if yu is None or len(yu) < fs * 2:
            continue
        sos = signal.butter(2, cutoff, "hp", fs=fs, output="sos")
        hp = signal.sosfiltfilt(sos, yu)
        vals.append(rms(hp))
        f, pxx = signal.welch(yu - np.mean(yu), fs=fs, nperseg=min(len(yu), 512))
        band = (f >= 4.0) & (f <= 30.0)
        if np.any(band):
            dom.append(float(f[band][np.argmax(pxx[band])]))
    return (float(np.mean(vals)) if vals else float("nan"),
            float(np.median(dom)) if dom else float("nan"))


def highpass_rms_parts(parts, fs=100.0, cutoff=4.0):
    vals, doms = [], []
    for t, y in parts:
        hp, dom = highpass_rms(t, y, fs, cutoff)
        if not math.isnan(hp):
            vals.append(hp)
        if not math.isnan(dom):
            doms.append(dom)
    return (float(np.mean(vals)) if vals else float("nan"),
            float(np.median(doms)) if doms else float("nan"))


def lag_seconds(t, target, meas, fs=50.0):
    tu, tar = resample(t, target, fs)
    _, mea = resample(t, meas, fs)
    if tar is None or len(tar) < fs * 5:
        return float("nan"), float("nan")
    tar = tar - np.mean(tar)
    mea = mea - np.mean(mea)
    max_lag = int(fs * 2.0)
    best = (0, -1.0)
    for lag in range(-max_lag, max_lag + 1):
        if lag > 0:
            x, y = tar[:-lag], mea[lag:]
        elif lag < 0:
            x, y = tar[-lag:], mea[:lag]
        else:
            x, y = tar, mea
        den = np.sqrt(np.sum(x * x) * np.sum(y * y))
        if den <= 0:
            continue
        c = float(np.sum(x * y) / den)
        if c > best[1]:
            best = (lag, c)
    return best[0] / fs, best[1]


def zero_metrics(a):
    t = a[:, 0]
    roll, pitch = a[:, 2], a[:, 3]
    rt, pt = a[:, 5], a[:, 6]
    vx, vy = a[:, 10], a[:, 11]
    tx, ty = a[:, 12], a[:, 13]
    xi, yi = a[:, 15], a[:, 18]
    tmag = np.hypot(tx, ty)
    zero = tmag <= 6.0
    keep = np.zeros_like(zero)
    zero_parts_roll = []
    zero_parts_pitch = []
    for s, e in segments(zero, t, 1.5):
        ss = np.searchsorted(t, t[s] + 0.8)
        if ss < e:
            keep[ss:e] = True
            zero_parts_roll.append((t[ss:e], roll[ss:e]))
            zero_parts_pitch.append((t[ss:e], pitch[ss:e]))
    spd = np.hypot(vx[keep], vy[keep])
    rhp, rdom = highpass_rms_parts(zero_parts_roll)
    php, pdom = highpass_rms_parts(zero_parts_pitch)
    return {
        "zero_dur_s": float(np.sum(np.diff(t, prepend=t[0]) * keep)),
        "zero_speed_mean": float(np.mean(spd)) if spd.size else float("nan"),
        "zero_speed_p95": pct(spd, 95),
        "zero_vx_mean": float(np.mean(vx[keep])) if np.any(keep) else float("nan"),
        "zero_vy_mean": float(np.mean(vy[keep])) if np.any(keep) else float("nan"),
        "roll_std": float(np.std(roll[keep])) if np.any(keep) else float("nan"),
        "pitch_std": float(np.std(pitch[keep])) if np.any(keep) else float("nan"),
        "roll_hp_rms": rhp,
        "pitch_hp_rms": php,
        "roll_dom_hz": rdom,
        "pitch_dom_hz": pdom,
        "roll_err_std": float(np.std((rt - roll)[keep])) if np.any(keep) else float("nan"),
        "pitch_err_std": float(np.std((pt - pitch)[keep])) if np.any(keep) else float("nan"),
        "xi_abs_p95": pct(np.abs(xi[keep]), 95),
        "yi_abs_p95": pct(np.abs(yi[keep]), 95),
        "i_sat_pct": float(np.mean((np.abs(xi[keep]) > 2.8) | (np.abs(yi[keep]) > 2.8)) * 100.0) if np.any(keep) else float("nan"),
    }


def tracking_metrics(a):
    t = a[:, 0]
    vx, vy = a[:, 10], a[:, 11]
    tx, ty = a[:, 12], a[:, 13]
    tmag = np.hypot(tx, ty)
    nonzero = tmag >= 25.0
    good = nonzero & (tmag > 1.0)
    along = (vx * tx + vy * ty) / np.maximum(tmag, 1.0)
    lateral = (vx * ty - vy * tx) / np.maximum(tmag, 1.0)
    err = tmag - along
    lx, cx = lag_seconds(t, tx, vx)
    ly, cy = lag_seconds(t, ty, vy)
    return {
        "track_dur_s": float(np.sum(np.diff(t, prepend=t[0]) * nonzero)),
        "track_err_mean": float(np.mean(np.abs(err[good]))) if np.any(good) else float("nan"),
        "track_err_p95": pct(np.abs(err[good]), 95),
        "track_ratio_med": float(np.median(along[good] / np.maximum(tmag[good], 1.0))) if np.any(good) else float("nan"),
        "lateral_p95": pct(np.abs(lateral[good]), 95),
        "lag_x_s": lx,
        "corr_x": cx,
        "lag_y_s": ly,
        "corr_y": cy,
    }


def brake_metrics(a):
    t = a[:, 0]
    vx, vy = a[:, 10], a[:, 11]
    tx, ty = a[:, 12], a[:, 13]
    tmag = np.hypot(tx, ty)
    zero_segs = segments(tmag <= 6.0, t, 1.0)
    times, residual_1s, residual_2s, initv = [], [], [], []
    hard_times, hard_residual_2s = [], []
    for s, e in zero_segs:
        pre = (t >= t[s] - 0.7) & (t < t[s] - 0.1) & (tmag > 25.0)
        if np.sum(pre) < 5:
            continue
        ux, uy = np.median(tx[pre]), np.median(ty[pre])
        norm = math.hypot(ux, uy)
        if norm < 20.0:
            continue
        ux, uy = ux / norm, uy / norm
        valong = vx[s:e] * ux + vy[s:e] * uy
        start_v = float(np.median(valong[:min(len(valong), 10)]))
        if abs(start_v) < 15.0:
            continue
        sign = 1.0 if start_v >= 0.0 else -1.0
        same = valong * sign
        hit = np.where(same <= 10.0)[0]
        times.append(float(t[s + hit[0]] - t[s]) if hit.size else float(t[e - 1] - t[s]))
        initv.append(abs(start_v))
        for sec, out in [(1.0, residual_1s), (2.0, residual_2s)]:
            idx = np.searchsorted(t[s:e], t[s] + sec)
            idx = min(max(idx, 0), len(same) - 1)
            out.append(float(same[idx]))
        if abs(start_v) >= 35.0:
            hard_times.append(times[-1])
            hard_residual_2s.append(residual_2s[-1])
    return {
        "brake_events": len(times),
        "hard_brake_events": len(hard_times),
        "brake_init_v_med": float(np.median(initv)) if initv else float("nan"),
        "brake_time_med": float(np.median(times)) if times else float("nan"),
        "brake_time_p80": pct(times, 80),
        "brake_residual_1s": float(np.mean(residual_1s)) if residual_1s else float("nan"),
        "brake_residual_2s": float(np.mean(residual_2s)) if residual_2s else float("nan"),
        "brake_fail_2s_pct": float(np.mean(np.asarray(times) > 2.0) * 100.0) if times else float("nan"),
        "hard_brake_time_med": float(np.median(hard_times)) if hard_times else float("nan"),
        "hard_brake_residual_2s": float(np.mean(hard_residual_2s)) if hard_residual_2s else float("nan"),
    }


def angle_accel_fit(a):
    t = a[:, 0]
    if t[-1] - t[0] < 5.0:
        return {}
    fs = 50.0
    tu, vx = resample(t, a[:, 10], fs)
    _, vy = resample(t, a[:, 11], fs)
    _, roll_t = resample(t, a[:, 5], fs)
    _, pitch_t = resample(t, a[:, 6], fs)
    _, tx = resample(t, a[:, 12], fs)
    _, ty = resample(t, a[:, 13], fs)
    if vx is None:
        return {}
    win = max(7, int(fs * 0.25) | 1)
    vx_s = signal.savgol_filter(vx, win, 2)
    vy_s = signal.savgol_filter(vy, win, 2)
    ax = np.gradient(vx_s, 1.0 / fs)
    ay = np.gradient(vy_s, 1.0 / fs)
    dyn = np.hypot(tx, ty) > 15.0
    out = {}
    for axis, acc, ang, vel, trim in [
        ("x", ax, roll_t, vx_s, 2.3),
        ("y", ay, pitch_t, vy_s, 3.0),
    ]:
        best = None
        for lag in range(0, int(fs * 0.4) + 1):
            aa = acc[lag:]
            ag = ang[:len(ang) - lag] - trim
            vv = vel[:len(vel) - lag]
            mm = dyn[:len(dyn) - lag]
            if np.sum(mm) < 100:
                continue
            X = np.column_stack([ag[mm], vv[mm], np.ones(np.sum(mm))])
            coef, *_ = np.linalg.lstsq(X, aa[mm], rcond=None)
            pred = X @ coef
            ssr = float(np.sum((aa[mm] - pred) ** 2))
            sst = float(np.sum((aa[mm] - np.mean(aa[mm])) ** 2))
            r2 = 1.0 - ssr / sst if sst > 0 else 0.0
            if best is None or r2 > best[0]:
                best = (r2, lag / fs, coef[0], coef[1])
        if best and abs(best[2]) > 1e-6:
            out[f"{axis}_acc_per_deg"] = float(best[2])
            out[f"{axis}_deg_per_acc"] = float(1.0 / best[2])
            out[f"{axis}_fit_lag_s"] = float(best[1])
            out[f"{axis}_fit_r2"] = float(best[0])
    return out


def main():
    summary = []
    for label, name, kp, ki in FILES:
        a = load_mode7(BASE / name)
        m = {"flight": label, "file": name, "kp": kp, "ki": ki, "rows": len(a), "duration_s": float(a[-1, 0] - a[0, 0])}
        m.update(zero_metrics(a))
        m.update(tracking_metrics(a))
        m.update(brake_metrics(a))
        m.update(angle_accel_fit(a))
        summary.append(m)

    keys = sorted({k for row in summary for k in row.keys()})
    first = ["flight", "file", "kp", "ki", "rows", "duration_s"]
    keys = first + [k for k in keys if k not in first]
    with (BASE / "velocity_loop_analysis_summary.csv").open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for row in summary:
            w.writerow(row)

    lines = ["# Velocity loop analysis", ""]
    lines.append("|flight|Kp|Ki|zero speed p95|roll hp rms|pitch hp rms|track err p95|brake med|brake residual 2s|I sat %|")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for r in summary:
        lines.append("|{flight}|{kp:.2f}|{ki:.2f}|{zero_speed_p95:.1f}|{roll_hp_rms:.3f}|{pitch_hp_rms:.3f}|{track_err_p95:.1f}|{brake_time_med:.2f}|{brake_residual_2s:.1f}|{i_sat_pct:.1f}|".format(**r))
    lines.extend(["", "Generated by analyze_velocity_loop.py."])
    (BASE / "velocity_loop_analysis_report.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
