"""
对比分析: 变软过后 (新参数) vs Flight 11 (旧参数) vs Flight 6 (最佳参考)
"""
from __future__ import annotations
import json, math, warnings
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import signal as scipy_signal

warnings.filterwarnings("ignore")
matplotlib.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

# ---- Paths ----
BASE_DIR = Path(r"D:\HDUASC-SmartCar-21st-FlyOverMinefield\CYT4BB7_Air\project\code\Estimation\Attitude\0531_PID")
NEW_LOG = Path(r"D:\Downloads\姿态环日志-变软过后.csv")
OUT_DIR = BASE_DIR / "analysis_变软过后对比"
PLOT_DIR = OUT_DIR / "plots"
for d in [OUT_DIR, PLOT_DIR]: d.mkdir(parents=True, exist_ok=True)

TRIM = {"roll": -1.8, "pitch": 3.5}

# ---- Reuse data structures ----
FLIGHT_ORDER = {"一": 1, "二": 2, "三": 3, "四": 4, "五": 5, "六": 6,
                "七": 7, "八": 8, "九": 9, "十": 10}
FLIGHT_NAME_ORDER = {"第十一次": 11, "第十二次": 12, "第十次": 10}

PARAMS = {
    6: {"name": "F6-重载平滑(参考)", "roll_gyro_kp": 4.3, "roll_gyro_ki": 0.08, "roll_gyro_kd": 0.008,
        "pitch_gyro_kp": 5.1, "pitch_gyro_ki": 0.14, "pitch_gyro_kd": 0.010,
        "roll_angle_kp": 6.2, "roll_angle_kff": 0.04, "pitch_angle_kp": 6.1, "pitch_angle_kff": 0.04},
    11: {"name": "F11-强内环(旧)", "roll_gyro_kp": 5.4, "roll_gyro_ki": 0.18, "roll_gyro_kd": 0.010,
         "pitch_gyro_kp": 6.0, "pitch_gyro_ki": 0.24, "pitch_gyro_kd": 0.012,
         "roll_angle_kp": 6.0, "roll_angle_kff": 0.02, "pitch_angle_kp": 6.2, "pitch_angle_kff": 0.02},
    13: {"name": "F13-变软后(新)", "roll_gyro_kp": 5.0, "roll_gyro_ki": 0.12, "roll_gyro_kd": 0.008,
         "pitch_gyro_kp": 5.1, "pitch_gyro_ki": 0.14, "pitch_gyro_kd": 0.010,
         "roll_angle_kp": 6.0, "roll_angle_kff": 0.02, "pitch_angle_kp": 6.2, "pitch_angle_kff": 0.02},
}

COLUMNS = {
    "t_ms": "I0", "roll": "I13", "pitch": "I14",
    "roll_angle_target": "I15", "roll_angle_measured": "I16", "roll_angle_out": "I17",
    "roll_angle_p": "I18", "roll_angle_i": "I19", "roll_angle_d": "I20", "roll_angle_error": "I21",
    "pitch_angle_target": "I22", "pitch_angle_measured": "I23", "pitch_angle_out": "I24",
    "pitch_angle_p": "I25", "pitch_angle_i": "I26", "pitch_angle_d": "I27", "pitch_angle_error": "I28",
    "roll_gyro_target": "I29", "roll_gyro_measured": "I30", "roll_gyro_out": "I31",
    "roll_gyro_p": "I32", "roll_gyro_i": "I33", "roll_gyro_d": "I34",
    "pitch_gyro_target": "I35", "pitch_gyro_measured": "I36", "pitch_gyro_out": "I37",
    "pitch_gyro_p": "I38", "pitch_gyro_i": "I39", "pitch_gyro_d": "I40",
}

@dataclass(frozen=True)
class AxisCols:
    angle_target: str; angle_measured: str; angle_out: str
    angle_p: str; angle_i: str; angle_d: str; angle_error: str
    gyro_target: str; gyro_measured: str; gyro_out: str
    gyro_p: str; gyro_i: str; gyro_d: str

AXES = {
    "roll": AxisCols("roll_angle_target", "roll_angle_measured", "roll_angle_out",
                      "roll_angle_p", "roll_angle_i", "roll_angle_d", "roll_angle_error",
                      "roll_gyro_target", "roll_gyro_measured", "roll_gyro_out",
                      "roll_gyro_p", "roll_gyro_i", "roll_gyro_d"),
    "pitch": AxisCols("pitch_angle_target", "pitch_angle_measured", "pitch_angle_out",
                       "pitch_angle_p", "pitch_angle_i", "pitch_angle_d", "pitch_angle_error",
                       "pitch_gyro_target", "pitch_gyro_measured", "pitch_gyro_out",
                       "pitch_gyro_p", "pitch_gyro_i", "pitch_gyro_d"),
}

BANDS = {"low": (0.5, 5.0), "mid": (5.0, 20.0), "high": (20.0, 80.0), "ultra": (80.0, 200.0)}
COLORS = {6: "#2ecc71", 11: "#e74c3c", 13: "#3498db"}

# ---- Utility functions (copied) ----
def rms(x): x = np.asarray(x, dtype=float); return float(np.sqrt(np.mean(x*x))) if x.size else math.nan

def derivative(t_s, x):
    t_s=np.asarray(t_s,float); x=np.asarray(x,float); out=np.zeros_like(x)
    if len(x)<2: return out
    dt=np.diff(t_s); dx=np.diff(x); safe=np.where(dt>1e-6,dt,np.nan)
    first=np.nan_to_num(dx/safe,nan=0,posinf=0,neginf=0); out[1:]=first; out[0]=out[1]; return out

def rolling_abs_rate(rate,samples=21):
    return pd.Series(np.abs(rate)).rolling(samples,center=True,min_periods=1).median().to_numpy(dtype=float)

def highpass_filter(signal, fs_hz, cutoff=15.0, order=4):
    s=np.asarray(signal,float).copy(); nan_mask=np.isnan(s)
    if nan_mask.any(): s=np.interp(np.flatnonzero(nan_mask),np.flatnonzero(~nan_mask),s[~nan_mask])
    nyq=fs_hz/2.0
    if cutoff>=nyq*0.95: return np.zeros_like(s)
    b,a=scipy_signal.butter(order,cutoff/nyq,btype="high")
    return scipy_signal.filtfilt(b,a,s)

def compute_psd(signal, fs_hz, nperseg=2048):
    s=np.asarray(signal,float); s=s[~np.isnan(s)]
    if len(s)<nperseg: return np.array([]),np.array([])
    nperseg=min(nperseg,len(s)//4)
    if nperseg<32: return np.array([]),np.array([])
    f,p=scipy_signal.welch(s,fs=fs_hz,window="hann",nperseg=nperseg,noverlap=nperseg//2,detrend="constant")
    return f,p

def band_limited_rms(freqs, psd):
    result={}
    for name,(flo,fhi) in BANDS.items():
        m=(freqs>=flo)&(freqs<fhi)
        result[name]=float(np.sqrt(max(np.trapz(psd[m],freqs[m]),0))) if m.sum()>=2 else np.nan
    m_total=freqs>=0.5
    result["total"]=float(np.sqrt(max(np.trapz(psd[m_total],freqs[m_total]),0))) if m_total.sum()>=2 else np.nan
    return result

def add_derived(df, flight):
    out=df.rename(columns={v:k for k,v in COLUMNS.items()}).copy()
    out["active"]=(out[["roll_angle_out","pitch_angle_out","roll_gyro_out","pitch_gyro_out"]].abs().sum(axis=1)>1e-6)
    t0=float(out.loc[out["active"],"t_ms"].iloc[0]) if out["active"].any() else float(out["t_ms"].iloc[0])
    out["t_active_s"]=(out["t_ms"]-t0)*0.001
    out["t_s"]=(out["t_ms"]-float(out["t_ms"].iloc[0]))*0.001
    out["analysis_mask"]=out["active"]&(out["t_ms"]>=t0+1000.0)
    for axis,cols in AXES.items():
        out[f"{axis}_gyro_error"]=out[cols.gyro_target]-out[cols.gyro_measured]
        out[f"{axis}_angle_target_rate"]=derivative(out["t_s"].to_numpy(),out[cols.angle_target].to_numpy())
        out[f"{axis}_gyro_target_rate"]=derivative(out["t_s"].to_numpy(),out[cols.gyro_target].to_numpy())
        out[f"{axis}_gyro_measured_rate"]=derivative(out["t_s"].to_numpy(),out[cols.gyro_measured].to_numpy())
        out[f"{axis}_angle_target_rate_abs_med"]=rolling_abs_rate(out[f"{axis}_angle_target_rate"].to_numpy())
    return out

def get_fs_hz(df):
    t=df["t_ms"].to_numpy(float); dt=np.diff(t)
    return float(1000.0/np.median(dt[dt>0.1]))

# ---- Main analysis ----
def analyze_one(df, flight, fs_hz):
    """Compute all metrics for one flight."""
    result = {"flight": flight, "params": PARAMS[flight], "fs_hz": fs_hz}

    for axis in ["roll", "pitch"]:
        cols = AXES[axis]
        mask_all = df["analysis_mask"].to_numpy(bool)

        # Quiet center mask
        target_offset = (df[cols.angle_target] - TRIM[axis]).to_numpy(float)
        target_rate_abs = df[f"{axis}_angle_target_rate_abs_med"].to_numpy(float)
        gyro_t = df[cols.gyro_target].to_numpy(float)
        gyro_m = df[cols.gyro_measured].to_numpy(float)
        mask_quiet = (mask_all & (np.abs(target_offset) <= 4.0) & (target_rate_abs <= 25.0)
                      & (np.abs(gyro_t) <= 35.0) & (np.abs(gyro_m) <= 35.0))

        ax_res = {}

        # --- Jitter (quiet center) ---
        if mask_quiet.sum() >= 100:
            gyro_raw = gyro_m[mask_quiet]
            gyro_hf = highpass_filter(gyro_raw, fs_hz, 15.0)
            gyro_err = df[f"{axis}_gyro_error"].to_numpy(float)[mask_quiet]
            gyro_err_hf = highpass_filter(gyro_err, fs_hz, 15.0)
            motor_out = df[cols.gyro_out].to_numpy(float)[mask_quiet]
            motor_deriv = derivative(df["t_s"].to_numpy()[mask_quiet], motor_out)

            comp_jitter = {}
            for comp, col in [("P", cols.gyro_p), ("I", cols.gyro_i), ("D", cols.gyro_d)]:
                comp_raw = df[col].to_numpy(float)[mask_quiet]
                comp_hf = highpass_filter(comp_raw, fs_hz, 15.0)
                comp_jitter[comp] = rms(comp_hf)

            ax_res["jitter"] = {
                "gyro_jitter_hf_rms": rms(gyro_hf),
                "gyro_jitter_err_hf_rms": rms(gyro_err_hf),
                "motor_roughness": rms(motor_deriv),
                "comp_jitter": comp_jitter,
                "motor_out_std": float(np.std(motor_out)),
                "motor_out_mean_abs": float(np.mean(np.abs(motor_out))),
            }

        # --- Hover precision (quiet center) ---
        if mask_quiet.sum() >= 100:
            angle_err = df[cols.angle_error].to_numpy(float)[mask_quiet]
            t_q = df["t_s"].to_numpy(float)[mask_quiet]
            slope = np.polyfit(t_q - t_q[0], angle_err, 1)[0] if len(t_q) > 2 else np.nan
            ax_res["hover"] = {
                "angle_hold_std_deg": float(np.std(angle_err)),
                "angle_hold_rms_deg": rms(angle_err),
                "angle_drift_deg_s": float(slope),
                "gyro_noise_std_dps": float(np.std(gyro_raw)),
                "motor_out_std": float(np.std(motor_out)),
                "duration_s": float(t_q[-1] - t_q[0]) if len(t_q) > 1 else 0.0,
                "n_samples": int(mask_quiet.sum()),
            }

        # --- Spectral (all active) ---
        if mask_all.sum() >= 100:
            ax_res["spectral"] = {}
            for sig_name, col_data in [
                ("gyro_measured", gyro_m[mask_all]),
                ("gyro_error", df[f"{axis}_gyro_error"].to_numpy(float)[mask_all]),
                ("gyro_out", df[cols.gyro_out].to_numpy(float)[mask_all]),
                ("angle_error", df[cols.angle_error].to_numpy(float)[mask_all]),
            ]:
                freqs, psd = compute_psd(col_data, fs_hz)
                if len(freqs) > 0:
                    ax_res["spectral"][sig_name] = {
                        "freqs": freqs, "psd": psd,
                        "band_rms": band_limited_rms(freqs, psd),
                    }

        result[axis] = ax_res

    return result


def plot_comparison(results):
    """Generate side-by-side comparison plots."""
    flights = [6, 11, 13]
    labels = {6: "F6 重载平滑\n(最佳参考)", 11: "F11 强内环\n(旧参数)", 13: "F13 变软后\n(新参数)"}

    # 1. PSD overlay - Pitch gyro measured (the key problem axis)
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    for ai, axis in enumerate(["roll", "pitch"]):
        ax = axes[ai]
        for fnum in flights:
            r = results.get(fnum, {}); ax_r = r.get(axis, {})
            sp = ax_r.get("spectral", {}).get("gyro_measured", {})
            if "freqs" not in sp: continue
            f = sp["freqs"]; p = sp["psd"]
            lw = 3 if fnum == 13 else (2.5 if fnum == 11 else 1.5)
            ax.loglog(f[f <= 150], p[f <= 150], color=COLORS[fnum], lw=lw, alpha=0.85, label=labels[fnum])
        for bname, (flo, fhi) in BANDS.items():
            ax.axvspan(flo, fhi, alpha=0.04, color="gray")
        ax.set_xlabel("频率 (Hz)"); ax.set_ylabel("PSD")
        ax.set_title(f"{axis.upper()} Gyro Measured PSD 对比"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "psd_comparison.png", dpi=160)
    plt.close(fig)

    # 2. Jitter bar chart
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    metric_groups = [
        ("gyro_jitter_hf_rms", "Gyro高频抖动RMS\n(>15Hz, deg/s)", "jitter"),
        ("motor_roughness", "电机输出粗糙度\n(输出导数RMS)", "jitter"),
        ("motor_out_std", "电机输出波动\n(std)", "jitter"),
    ]
    metric_groups2 = [
        ("angle_hold_std_deg", "角度保持精度\n(std deg, 越小越好)", "hover"),
        ("angle_drift_deg_s", "角度漂移率\n(deg/s)", "hover"),
        ("gyro_noise_std_dps", "陀螺噪声底\n(std deg/s)", "hover"),
    ]

    for row_i, axis in enumerate(["roll", "pitch"]):
        for col_j, (metric, title, source) in enumerate(metric_groups):
            ax = axes[row_i, col_j]
            vals = []
            for fnum in flights:
                r = results.get(fnum, {}); ax_r = r.get(axis, {})
                d = ax_r.get(source, {})
                if metric == "motor_out_std":
                    v = d.get("motor_out_std") or ax_r.get("hover", {}).get("motor_out_std", np.nan)
                else:
                    v = d.get(metric, np.nan)
                vals.append(v)
            colors_list = [COLORS[f] for f in flights]
            bars = ax.bar(range(3), vals, color=colors_list, alpha=0.85, width=0.5)
            ax.set_xticks(range(3))
            ax.set_xticklabels([labels[f] for f in flights], fontsize=7)
            ax.set_title(f"{axis.upper()} {title}", fontsize=9)
            ax.grid(True, alpha=0.3)
            for i, v in enumerate(vals):
                if not np.isnan(v):
                    ax.text(i, v + max(vals)*0.02, f"{v:.1f}", ha="center", fontsize=8, fontweight="bold")

    for row_i, axis in enumerate(["roll", "pitch"]):
        for col_j, (metric, title, source) in enumerate(metric_groups2):
            ax = axes[row_i, col_j]
            vals = []
            for fnum in flights:
                r = results.get(fnum, {}); ax_r = r.get(axis, {})
                d = ax_r.get(source, {})
                vals.append(d.get(metric, np.nan))
            colors_list = [COLORS[f] for f in flights]
            ax.bar(range(3), vals, color=colors_list, alpha=0.85, width=0.5)
            ax.set_xticks(range(3))
            ax.set_xticklabels([labels[f] for f in flights], fontsize=7)
            ax.set_title(f"{axis.upper()} {title}", fontsize=9)
            ax.grid(True, alpha=0.3)

    # But the axes grid is 2x3 which doesn't match - let me redo with separate figures

    # Let me simplify - create 2 figures: one for pitch (key), one for roll
    for main_axis in ["pitch", "roll"]:
        fig, axes = plt.subplots(2, 3, figsize=(18, 10))
        all_metrics = [
            ("gyro_jitter_hf_rms", "jitter", "Gyro高频抖动\n(>15Hz RMS, deg/s)"),
            ("motor_roughness", "jitter", "电机粗糙度\n(输出导数RMS)"),
            ("angle_hold_std_deg", "hover", "角度保持精度\n(std deg)"),
            ("motor_out_std", "jitter", "电机输出波动\n(std)"),
            ("angle_drift_deg_s", "hover", "角度漂移率\n(deg/s)"),
            ("gyro_noise_std_dps", "hover", "陀螺噪声底\n(std deg/s)"),
        ]
        for (metric, source, title), ax in zip(all_metrics, axes.flatten()):
            vals = []
            for fnum in flights:
                r = results.get(fnum, {}); ax_r = r.get(main_axis, {})
                d = ax_r.get(source, {})
                if metric == "motor_out_std":
                    v = d.get("motor_out_std") or ax_r.get("hover", {}).get("motor_out_std", np.nan)
                else:
                    v = d.get(metric, np.nan)
                vals.append(v)
            colors_list = [COLORS[f] for f in flights]
            bars = ax.bar(range(3), vals, color=colors_list, alpha=0.85, width=0.5)
            ax.set_xticks(range(3))
            ax.set_xticklabels(["F6\n最佳参考", "F11\n强内环(旧)", "F13\n变软后(新)"], fontsize=8)
            ax.set_title(title, fontsize=10)
            ax.grid(True, alpha=0.3)
            # Annotate with values
            for i, v in enumerate(vals):
                if not np.isnan(v) and max(vals) > 0:
                    offset = max(vals) * 0.03
                    ax.text(i, v + offset, f"{v:.1f}", ha="center", fontsize=9 if fnum == flights[i] else 8,
                            fontweight="bold" if flights[i] == 13 else "normal")
        fig.suptitle(f"{main_axis.upper()} 轴 — 三组参数横向对比", fontsize=13, fontweight="bold")
        fig.tight_layout()
        fig.savefig(PLOT_DIR / f"{main_axis}_comparison.png", dpi=160)
        plt.close(fig)

    # 3. PID component jitter breakdown - Pitch only (the problem axis)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for ai, axis in enumerate(["pitch", "roll"]):
        ax = axes[ai]
        p_vals, i_vals, d_vals = [], [], []
        for fnum in flights:
            r = results.get(fnum, {}); ax_r = r.get(axis, {})
            cj = ax_r.get("jitter", {}).get("comp_jitter", {})
            p_vals.append(cj.get("P", 0)); i_vals.append(cj.get("I", 0)); d_vals.append(cj.get("D", 0))
        x = np.arange(3); w = 0.5
        ax.bar(x, p_vals, w, label="P项抖振", color="#e74c3c", alpha=0.85)
        ax.bar(x, i_vals, w, bottom=p_vals, label="I项抖振", color="#f39c12", alpha=0.85)
        bottoms = [p_vals[i] + i_vals[i] for i in range(3)]
        ax.bar(x, d_vals, w, bottom=bottoms, label="D项抖振", color="#3498db", alpha=0.85)
        ax.set_xticks(x)
        ax.set_xticklabels(["F6\n最佳参考", "F11\n强内环(旧)", "F13\n变软后(新)"], fontsize=8)
        ax.set_title(f"{axis.upper()} PID分量高频抖振归因"); ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "pid_component_jitter.png", dpi=160)
    plt.close(fig)

    # 4. Spectral band RMS comparison - Pitch
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for ai, axis in enumerate(["pitch", "roll"]):
        ax = axes[ai]
        band_names = ["low", "mid", "high", "ultra"]
        x = np.arange(len(band_names)); w = 0.25
        for fi, fnum in enumerate(flights):
            r = results.get(fnum, {}); ax_r = r.get(axis, {})
            sp = ax_r.get("spectral", {}).get("gyro_measured", {})
            br = sp.get("band_rms", {})
            vals = [br.get(b, 0) for b in band_names]
            ax.bar(x + fi * w, vals, w, color=COLORS[fnum], alpha=0.85, label=labels[fnum])
        ax.set_xticks(x + w); ax.set_xticklabels(["0.5-5Hz\n操纵", "5-20Hz\n控制振荡", "20-80Hz\n高频抖动", "80-200Hz\n谐波"])
        ax.set_title(f"{axis.upper()} Gyro 分频带 RMS"); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "band_rms_comparison.png", dpi=160)
    plt.close(fig)

    # 5. Time-domain: angle tracking for a representative window
    fig, axes = plt.subplots(2, 2, figsize=(18, 10))
    for row_i, axis in enumerate(["roll", "pitch"]):
        for col_j, fnum in enumerate([11, 13]):
            ax = axes[row_i, col_j]
            df = loaded[fnum]
            cols = AXES[axis]
            # Find the longest active region
            mask = df["analysis_mask"].to_numpy(bool)
            if mask.sum() < 50: continue
            # Take a representative segment (middle 5 seconds of active flight)
            active_idx = np.flatnonzero(mask)
            mid = active_idx[len(active_idx)//2]
            t = df["t_active_s"].to_numpy(float)
            t_mid = t[mid]
            window = (t >= t_mid - 2.5) & (t <= t_mid + 2.5) & mask
            if window.sum() < 50: continue
            sub = df[window]
            t_w = sub["t_active_s"].to_numpy(float)
            ax.plot(t_w, sub[cols.angle_target], "k--", label="Target", lw=1.5, alpha=0.7)
            ax.plot(t_w, sub[cols.angle_measured], color=COLORS[fnum], label="Measured", lw=1.8)
            ax.set_title(f"{axis.upper()} — {PARAMS[fnum]['name']}"); ax.legend(); ax.grid(True, alpha=0.3)
            ax.set_ylabel("deg")
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "angle_tracking_time_domain.png", dpi=160)
    plt.close(fig)


def generate_report(results):
    """Generate comparison markdown report."""
    lines = [
        "# 变软过后参数横向对比分析",
        "",
        "## 对比对象",
        "",
        "| 飞行 | 参数组 | pitch_gyro_kp | pitch_gyro_ki | pitch_gyro_kd | roll_gyro_kp | roll_gyro_ki |",
        "|------|--------|--------------|--------------|--------------|-------------|-------------|",
    ]
    for fnum in [6, 11, 13]:
        p = PARAMS[fnum]
        lines.append(f"| F{fnum} | {p['name']} | {p['pitch_gyro_kp']} | {p['pitch_gyro_ki']} | {p['pitch_gyro_kd']} | {p['roll_gyro_kp']} | {p['roll_gyro_ki']} |")

    lines.extend([
        "",
        "---",
        "",
        "## 核心指标对比 — Pitch 轴 (问题轴)",
        "",
        "| 指标 | F6 最佳参考 | F11 强内环(旧) | F13 变软后(新) | F13 vs F11 改善 | 结论 |",
        "|------|-----------|---------------|---------------|----------------|------|",
    ])

    def get_val(fnum, axis, metric, source="jitter"):
        r = results.get(fnum, {}); ax_r = r.get(axis, {})
        d = ax_r.get(source, {})
        return d.get(metric, np.nan)

    pitch_metrics = [
        ("gyro_jitter_hf_rms", "jitter", "Gyro高频抖动RMS", "deg/s", True),
        ("motor_roughness", "jitter", "电机粗糙度", "units/s", True),
        ("motor_out_std", "jitter", "电机输出波动std", "units", True),
        ("angle_hold_std_deg", "hover", "角度保持精度std", "deg", True),
        ("angle_drift_deg_s", "hover", "角度漂移率", "deg/s", True),
        ("gyro_noise_std_dps", "hover", "陀螺噪声底", "deg/s", False),
    ]

    improvements = []
    for metric, source, label, unit, lower_better in pitch_metrics:
        f6_v = get_val(6, "pitch", metric, source)
        f11_v = get_val(11, "pitch", metric, source)
        f13_v = get_val(13, "pitch", metric, source)

        if lower_better:
            change = ((f11_v - f13_v) / f11_v * 100) if f11_v > 0 else 0
            better = "[改善]" if change > 1 else ("[持平]" if abs(change) <= 1 else "[变差]")
        else:
            change = 0
            better = "—"

        lines.append(f"| {label} | {f6_v:.1f} | {f11_v:.1f} | {f13_v:.1f} | {change:+.0f}% | {better} |")
        improvements.append((label, change, better))

    # Also compare Roll axis
    lines.extend([
        "",
        "---",
        "",
        "## 核心指标对比 — Roll 轴",
        "",
        "| 指标 | F6 最佳参考 | F11 强内环(旧) | F13 变软后(新) | F13 vs F11 改善 | 结论 |",
        "|------|-----------|---------------|---------------|----------------|------|",
    ])
    for metric, source, label, unit, lower_better in pitch_metrics:
        f6_v = get_val(6, "roll", metric, source)
        f11_v = get_val(11, "roll", metric, source)
        f13_v = get_val(13, "roll", metric, source)
        if lower_better:
            change = ((f11_v - f13_v) / f11_v * 100) if f11_v > 0 else 0
            better = "[改善]" if change > 1 else ("[持平]" if abs(change) <= 1 else "[变差]")
        else:
            change = 0; better = "—"
        lines.append(f"| {label} | {f6_v:.1f} | {f11_v:.1f} | {f13_v:.1f} | {change:+.0f}% | {better} |")

    # PID component breakdown for Pitch
    lines.extend([
        "",
        "---",
        "",
        "## Pitch 轴 PID 分量高频抖振归因",
        "",
        "| 分量 | F6 | F11(旧) | F13(新) | 变化 |",
        "|------|-----|---------|---------|------|",
    ])
    for comp in ["P", "I", "D"]:
        f6_v = get_val(6, "pitch", comp, "jitter") if comp != "P" else results[6]["pitch"]["jitter"]["comp_jitter"]["P"]
        f11_v = get_val(11, "pitch", comp, "jitter") if comp != "P" else results[11]["pitch"]["jitter"]["comp_jitter"]["P"]
        f13_v = get_val(13, "pitch", comp, "jitter") if comp != "P" else results[13]["pitch"]["jitter"]["comp_jitter"]["P"]
        # Actually let me get these properly from comp_jitter
        f6_cj = results[6]["pitch"]["jitter"]["comp_jitter"]
        f11_cj = results[11]["pitch"]["jitter"]["comp_jitter"]
        f13_cj = results[13]["pitch"]["jitter"]["comp_jitter"]
        f6_v = f6_cj.get(comp, 0)
        f11_v = f11_cj.get(comp, 0)
        f13_v = f13_cj.get(comp, 0)
        change = ((f11_v - f13_v) / f11_v * 100) if f11_v > 0 else 0
        better = "[改善]" if change > 1 else ("[持平]" if abs(change) <= 1 else "[变差]")
        lines.append(f"| {comp} | {f6_v:.2f} | {f11_v:.2f} | {f13_v:.2f} | {change:+.0f}% {better} |")

    # Spectral band RMS for Pitch
    lines.extend([
        "",
        "---",
        "",
        "## Pitch 轴 Gyro 分频带 RMS 对比 (deg/s)",
        "",
        "| 频带 | F6 | F11(旧) | F13(新) | F13 vs F11 变化 |",
        "|------|-----|---------|---------|----------------|",
    ])
    for band in ["low", "mid", "high", "ultra"]:
        br_f6 = results[6]["pitch"]["spectral"]["gyro_measured"]["band_rms"]
        br_f11 = results[11]["pitch"]["spectral"]["gyro_measured"]["band_rms"]
        br_f13 = results[13]["pitch"]["spectral"]["gyro_measured"]["band_rms"]
        f6_v = br_f6.get(band, 0)
        f11_v = br_f11.get(band, 0)
        f13_v = br_f13.get(band, 0)
        change = ((f11_v - f13_v) / f11_v * 100) if f11_v > 0 else 0
        band_label = f"{BANDS[band][0]}-{BANDS[band][1]}Hz {band}"
        lines.append(f"| {band_label} | {f6_v:.2f} | {f11_v:.2f} | {f13_v:.2f} | {change:+.0f}% |")

    # Overall conclusion
    lines.extend([
        "",
        "---",
        "",
        "## 总体结论",
        "",
    ])

    # Calculate key improvements for pitch
    pitch_gyro_jit_change = (get_val(11, "pitch", "gyro_jitter_hf_rms", "jitter") - get_val(13, "pitch", "gyro_jitter_hf_rms", "jitter")) / get_val(11, "pitch", "gyro_jitter_hf_rms", "jitter") * 100
    pitch_rough_change = (get_val(11, "pitch", "motor_roughness", "jitter") - get_val(13, "pitch", "motor_roughness", "jitter")) / get_val(11, "pitch", "motor_roughness", "jitter") * 100
    pitch_p_change = (results[11]["pitch"]["jitter"]["comp_jitter"]["P"] - results[13]["pitch"]["jitter"]["comp_jitter"]["P"]) / results[11]["pitch"]["jitter"]["comp_jitter"]["P"] * 100

    lines.append(f"1. **Pitch 轴高频抖动**: F13 比 F11 {abs(pitch_gyro_jit_change):.0f}% {'降低' if pitch_gyro_jit_change > 0 else '升高'} ({get_val(13, 'pitch', 'gyro_jitter_hf_rms', 'jitter'):.2f} vs {get_val(11, 'pitch', 'gyro_jitter_hf_rms', 'jitter'):.2f} deg/s)")
    lines.append(f"2. **Pitch 电机粗糙度**: F13 比 F11 {abs(pitch_rough_change):.0f}% {'降低' if pitch_rough_change > 0 else '升高'} ({get_val(13, 'pitch', 'motor_roughness', 'jitter'):.0f} vs {get_val(11, 'pitch', 'motor_roughness', 'jitter'):.0f})")
    lines.append(f"3. **Pitch P分量抖振**: F13 比 F11 {abs(pitch_p_change):.0f}% {'降低' if pitch_p_change > 0 else '升高'} ({results[13]['pitch']['jitter']['comp_jitter']['P']:.1f} vs {results[11]['pitch']['jitter']['comp_jitter']['P']:.1f})")

    if pitch_rough_change > 5 and pitch_gyro_jit_change > 3:
        lines.append("")
        lines.append("**结论: 变软参数改进有效!** Pitch 轴的电机粗糙度和高频抖动均有明显降低，参数调整方向正确。")
    elif pitch_rough_change > 0 and pitch_gyro_jit_change > 0:
        lines.append("")
        lines.append("**结论: 变软参数有改善。** 建议继续飞行验证手感是否'不抖了'。")
    else:
        lines.append("")
        lines.append("**结论: 变软参数效果不明显或变差。** 建议检查飞行条件是否一致（风速、电池电压、挂载重量等）。")

    lines.append("")
    report = "\n".join(lines)
    (OUT_DIR / "comparison_report.md").write_text(report, encoding="utf-8")
    return report


# ---- Main ----
print("=" * 60)
print("变软过后参数横向对比分析")
print("=" * 60)

# Load flights
loaded = {}
print("\n加载飞行日志...")
for fnum, path in [
    (6, BASE_DIR / "第六次飞行.csv"),
    (11, BASE_DIR / "第十一次飞行.csv"),
    (13, NEW_LOG),
]:
    df = pd.read_csv(path)
    df = add_derived(df, fnum)
    loaded[fnum] = df
    n = len(df)
    dur = (df["t_ms"].iloc[-1] - df["t_ms"].iloc[0]) / 1000.0 if len(df) > 1 else 0
    print(f"  F{fnum} ({PARAMS[fnum]['name']}): {n} 行, {dur:.1f}s")

fs_hz = get_fs_hz(loaded[6])
print(f"  采样率: {fs_hz:.0f} Hz")

# Analyze each
results = {}
for fnum in [6, 11, 13]:
    print(f"\n分析 F{fnum} ({PARAMS[fnum]['name']})...")
    results[fnum] = analyze_one(loaded[fnum], fnum, fs_hz)

    # Print key metrics
    for axis in ["pitch", "roll"]:
        j = results[fnum][axis].get("jitter", {})
        h = results[fnum][axis].get("hover", {})
        cj = j.get("comp_jitter", {})
        print(f"  {axis}: 抖动RMS={j.get('gyro_jitter_hf_rms',0):.2f} deg/s, "
              f"粗糙度={j.get('motor_roughness',0):.0f}, "
              f"P抖振={cj.get('P',0):.1f}, "
              f"角度std={h.get('angle_hold_std_deg',0):.3f} deg")

# Generate plots and report
print("\n生成对比图表...")
plot_comparison(results)

print("生成报告...")
report = generate_report(results)
print(report)
print(f"\n图表: {PLOT_DIR}/")
print(f"报告: {OUT_DIR / 'comparison_report.md'}")
print("\n分析完成!")
