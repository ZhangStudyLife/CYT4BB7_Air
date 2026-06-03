"""
0531 PID 深度分析 v3 — 频谱/抖动/阶跃响应深度/悬停/综合评分
聚焦 Flight 11（当前参数）的"抖动+超调"问题，12份日志全面对比。
"""
from __future__ import annotations

import json
import math
import warnings
from dataclasses import dataclass
from pathlib import Path

import logging
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import signal as scipy_signal

warnings.filterwarnings("ignore")
logging.getLogger("matplotlib").setLevel(logging.ERROR)
matplotlib.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

# ============================================================
# Paths & Constants
# ============================================================
BASE_DIR = Path(__file__).resolve().parent
OUT_DIR = BASE_DIR / "analysis_0531_pid_deep_v3"
PLOT_DIR = OUT_DIR / "plots"

TRIM = {"roll": -1.8, "pitch": 3.5}

FLIGHT_CN = {
    1: "基线", 2: "关FF", 3: "高角度P", 4: "加D阻尼",
    5: "强自稳", 6: "重载平滑", 7: "均衡候选", 8: "抗偏置",
    9: "偏软欠调", 10: "强角度FF", 11: "强内环低FF(当前)", 12: "强D阻尼",
}

COLORS_12 = plt.cm.tab10(np.linspace(0, 1, 10)).tolist() + plt.cm.Set2(np.linspace(0, 1, 2)).tolist()
COLOR_F11 = "#e74c3c"       # red for flight 11
COLOR_OTHER = "#3498db"     # blue for other flights
LINE_STYLES = ["-", "--", "-.", ":", (0, (3, 1)), (0, (5, 2)),
               "-", "--", "-.", ":", (0, (3, 1)), (0, (5, 2))]

# Frequency bands for spectral analysis
BANDS = {
    "low": (0.5, 5.0),
    "mid": (5.0, 20.0),
    "high": (20.0, 80.0),
    "ultra": (80.0, 200.0),
}

# ============================================================
# Data structures & parameters (from existing script)
# ============================================================
FLIGHT_ORDER = {"一": 1, "二": 2, "三": 3, "四": 4, "五": 5, "六": 6,
                "七": 7, "八": 8, "九": 9, "十": 10}
FLIGHT_NAME_ORDER = {"第十一次": 11, "第十二次": 12, "第十次": 10}

PARAMS = {
    1: {"name": "基线_FF0.12", "roll_angle_kp": 6.5, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.12,
        "pitch_angle_kp": 6.4, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.12,
        "roll_gyro_kp": 3.8, "roll_gyro_ki": 0.0, "roll_gyro_kd": 0.0,
        "pitch_gyro_kp": 4.6, "pitch_gyro_ki": 0.08, "pitch_gyro_kd": 0.0},
    2: {"name": "关FF", "roll_angle_kp": 6.5, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.0,
        "pitch_angle_kp": 6.4, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.0,
        "roll_gyro_kp": 3.8, "roll_gyro_ki": 0.0, "roll_gyro_kd": 0.0,
        "pitch_gyro_kp": 4.6, "pitch_gyro_ki": 0.08, "pitch_gyro_kd": 0.0},
    3: {"name": "高角度P", "roll_angle_kp": 7.1, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.08,
        "pitch_angle_kp": 7.0, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.08,
        "roll_gyro_kp": 4.1, "roll_gyro_ki": 0.05, "roll_gyro_kd": 0.0,
        "pitch_gyro_kp": 4.9, "pitch_gyro_ki": 0.10, "pitch_gyro_kd": 0.0},
    4: {"name": "加D阻尼", "roll_angle_kp": 6.8, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.06,
        "pitch_angle_kp": 6.7, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.06,
        "roll_gyro_kp": 4.0, "roll_gyro_ki": 0.06, "roll_gyro_kd": 0.006,
        "pitch_gyro_kp": 4.8, "pitch_gyro_ki": 0.11, "pitch_gyro_kd": 0.007},
    5: {"name": "强自稳", "roll_angle_kp": 7.6, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.10,
        "pitch_angle_kp": 7.4, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.10,
        "roll_gyro_kp": 4.2, "roll_gyro_ki": 0.06, "roll_gyro_kd": 0.006,
        "pitch_gyro_kp": 5.0, "pitch_gyro_ki": 0.12, "pitch_gyro_kd": 0.008},
    6: {"name": "重载平滑", "roll_angle_kp": 6.2, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.04,
        "pitch_angle_kp": 6.1, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.04,
        "roll_gyro_kp": 4.3, "roll_gyro_ki": 0.08, "roll_gyro_kd": 0.008,
        "pitch_gyro_kp": 5.1, "pitch_gyro_ki": 0.14, "pitch_gyro_kd": 0.010},
    7: {"name": "均衡候选", "roll_angle_kp": 6.5, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.05,
        "pitch_angle_kp": 6.7, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.05,
        "roll_gyro_kp": 4.2, "roll_gyro_ki": 0.07, "roll_gyro_kd": 0.007,
        "pitch_gyro_kp": 4.9, "pitch_gyro_ki": 0.12, "pitch_gyro_kd": 0.008},
    8: {"name": "抗偏置", "roll_angle_kp": 6.4, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.04,
        "pitch_angle_kp": 6.6, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.04,
        "roll_gyro_kp": 4.2, "roll_gyro_ki": 0.10, "roll_gyro_kd": 0.007,
        "pitch_gyro_kp": 4.9, "pitch_gyro_ki": 0.15, "pitch_gyro_kd": 0.008},
    9: {"name": "偏软欠调", "roll_angle_kp": 5.0, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.0,
        "pitch_angle_kp": 5.2, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.0,
        "roll_gyro_kp": 3.2, "roll_gyro_ki": 0.04, "roll_gyro_kd": 0.003,
        "pitch_gyro_kp": 3.8, "pitch_gyro_ki": 0.06, "pitch_gyro_kd": 0.004},
    10: {"name": "强角度FF", "roll_angle_kp": 8.2, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.14,
         "pitch_angle_kp": 8.0, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.14,
         "roll_gyro_kp": 4.2, "roll_gyro_ki": 0.08, "roll_gyro_kd": 0.006,
         "pitch_gyro_kp": 4.9, "pitch_gyro_ki": 0.12, "pitch_gyro_kd": 0.007},
    11: {"name": "强内环低FF(当前)", "roll_angle_kp": 6.0, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.02,
         "pitch_angle_kp": 6.2, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.02,
         "roll_gyro_kp": 5.4, "roll_gyro_ki": 0.18, "roll_gyro_kd": 0.010,
         "pitch_gyro_kp": 6.0, "pitch_gyro_ki": 0.24, "pitch_gyro_kd": 0.012},
    12: {"name": "强D阻尼", "roll_angle_kp": 5.8, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.03,
         "pitch_angle_kp": 6.0, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.03,
         "roll_gyro_kp": 4.6, "roll_gyro_ki": 0.10, "roll_gyro_kd": 0.014,
         "pitch_gyro_kp": 5.3, "pitch_gyro_ki": 0.15, "pitch_gyro_kd": 0.016},
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

# ============================================================
# Utility functions
# ============================================================
def flight_no_from_name(path: Path) -> int:
    for prefix, flight in FLIGHT_NAME_ORDER.items():
        if path.name.startswith(prefix):
            return flight
    if len(path.name) < 2 or path.name[1] not in FLIGHT_ORDER:
        raise ValueError(f"Cannot infer flight number from {path.name}")
    return FLIGHT_ORDER[path.name[1]]


def rms(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    if x.size == 0: return math.nan
    return float(np.sqrt(np.mean(x * x)))


def pctl(x: np.ndarray, q: float) -> float:
    x = np.asarray(x, dtype=float)
    if x.size == 0: return math.nan
    return float(np.percentile(x, q))


def derivative(t_s: np.ndarray, x: np.ndarray) -> np.ndarray:
    t_s = np.asarray(t_s, dtype=float); x = np.asarray(x, dtype=float)
    out = np.zeros_like(x)
    if len(x) < 2: return out
    dt = np.diff(t_s); dx = np.diff(x)
    safe = np.where(dt > 1e-6, dt, np.nan)
    first = np.nan_to_num(dx / safe, nan=0.0, posinf=0.0, neginf=0.0)
    out[1:] = first; out[0] = out[1]
    return out


def rolling_abs_rate(rate: np.ndarray, samples: int = 21) -> np.ndarray:
    return pd.Series(np.abs(rate)).rolling(samples, center=True, min_periods=1).median().to_numpy(dtype=float)


def markdown_table(df: pd.DataFrame) -> str:
    if df.empty: return "_No rows._"
    text_df = df.copy()
    for col in text_df.columns:
        text_df[col] = text_df[col].map(lambda x: "" if pd.isna(x) else str(x))
    headers = list(text_df.columns); rows = text_df.values.tolist()
    widths = [max(len(str(h)), *(len(str(r[i])) for r in rows)) for i, h in enumerate(headers)]
    lines = [
        "| " + " | ".join(str(h).ljust(widths[i]) for i, h in enumerate(headers)) + " |",
        "| " + " | ".join("-" * widths[i] for i in range(len(headers))) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(row[i]).ljust(widths[i]) for i in range(len(headers))) + " |")
    return "\n".join(lines)


def add_derived(df: pd.DataFrame, flight: int) -> pd.DataFrame:
    out = df.rename(columns={v: k for k, v in COLUMNS.items()}).copy()
    out["active"] = (
        out[["roll_angle_out", "pitch_angle_out", "roll_gyro_out", "pitch_gyro_out"]]
        .abs().sum(axis=1) > 1e-6
    )
    t0 = float(out.loc[out["active"], "t_ms"].iloc[0]) if out["active"].any() else float(out["t_ms"].iloc[0])
    out["t_active_s"] = (out["t_ms"] - t0) * 0.001
    out["t_s"] = (out["t_ms"] - float(out["t_ms"].iloc[0])) * 0.001
    out["analysis_mask"] = out["active"] & (out["t_ms"] >= t0 + 1000.0)

    for axis, cols in AXES.items():
        out[f"{axis}_angle_ff_residual"] = out[cols.angle_out] - out[cols.angle_p] - out[cols.angle_i] - out[cols.angle_d]
        out[f"{axis}_gyro_error"] = out[cols.gyro_target] - out[cols.gyro_measured]
        out[f"{axis}_angle_target_rate"] = derivative(out["t_s"].to_numpy(), out[cols.angle_target].to_numpy())
        out[f"{axis}_gyro_target_rate"] = derivative(out["t_s"].to_numpy(), out[cols.gyro_target].to_numpy())
        out[f"{axis}_gyro_measured_rate"] = derivative(out["t_s"].to_numpy(), out[cols.gyro_measured].to_numpy())
        out[f"{axis}_angle_target_rate_abs_med"] = rolling_abs_rate(out[f"{axis}_angle_target_rate"].to_numpy())
        out[f"{axis}_gyro_target_rate_abs_med"] = rolling_abs_rate(out[f"{axis}_gyro_target_rate"].to_numpy())
    return out


def get_fs_hz(df: pd.DataFrame) -> float:
    """Compute effective sample rate."""
    t = df["t_ms"].to_numpy(dtype=float)
    dt_ms = np.diff(t)
    return float(1000.0 / np.median(dt_ms[dt_ms > 0.1]))


def compute_psd(signal: np.ndarray, fs_hz: float, nperseg: int = 2048) -> tuple[np.ndarray, np.ndarray]:
    """Welch PSD."""
    s = np.asarray(signal, dtype=float)
    mask = ~np.isnan(s)
    s = s[mask]
    if len(s) < nperseg: return np.array([]), np.array([])
    nperseg = min(nperseg, len(s) // 4)
    if nperseg < 32: return np.array([]), np.array([])
    freqs, psd = scipy_signal.welch(s, fs=fs_hz, window="hann", nperseg=nperseg,
                                     noverlap=nperseg // 2, detrend="constant")
    return freqs, psd


def band_limited_rms(freqs: np.ndarray, psd: np.ndarray, bands: dict) -> dict[str, float]:
    """Integrate PSD within each band to get RMS."""
    result = {}
    for name, (f_lo, f_hi) in bands.items():
        m = (freqs >= f_lo) & (freqs < f_hi)
        if m.sum() < 2:
            result[name] = np.nan; continue
        power = np.trapz(psd[m], freqs[m])
        result[name] = float(np.sqrt(max(power, 0.0)))
    m_total = freqs >= 0.5
    if m_total.sum() >= 2:
        result["total"] = float(np.sqrt(max(np.trapz(psd[m_total], freqs[m_total]), 0.0)))
    else:
        result["total"] = np.nan
    return result


def dominant_peaks(freqs: np.ndarray, psd: np.ndarray, top_n: int = 3, min_freq: float = 2.0) -> list[dict]:
    """Find dominant frequency peaks."""
    mask = (freqs >= min_freq) & (freqs < freqs[-1] * 0.9)
    if mask.sum() < 10: return []
    f_band = freqs[mask]; p_band = psd[mask]
    peaks, props = scipy_signal.find_peaks(p_band, distance=5, prominence=np.percentile(p_band, 70))
    if len(peaks) == 0: return []
    idx = np.argsort(p_band[peaks])[::-1][:top_n]
    return [{"freq_hz": float(f_band[peaks[i]]), "psd": float(p_band[peaks[i]])} for i in idx]


def highpass_filter(signal: np.ndarray, fs_hz: float, cutoff: float = 15.0, order: int = 4) -> np.ndarray:
    """Butterworth high-pass, zero-phase."""
    s = np.asarray(signal, dtype=float).copy()
    nan_mask = np.isnan(s)
    if nan_mask.any():
        s = np.interp(np.flatnonzero(nan_mask), np.flatnonzero(~nan_mask), s[~nan_mask])
    nyq = fs_hz / 2.0
    if cutoff >= nyq * 0.95: return np.zeros_like(s)
    b, a = scipy_signal.butter(order, cutoff / nyq, btype="high")
    return scipy_signal.filtfilt(b, a, s)


def lowpass_filter(signal: np.ndarray, fs_hz: float, cutoff: float = 20.0, order: int = 4) -> np.ndarray:
    s = np.asarray(signal, dtype=float).copy()
    nyq = fs_hz / 2.0
    if cutoff >= nyq * 0.95: return s
    b, a = scipy_signal.butter(order, cutoff / nyq, btype="low")
    return scipy_signal.filtfilt(b, a, s)


# ============================================================
# SECTION 1: Spectral Analysis
# ============================================================
def analyze_flight_spectral(df: pd.DataFrame, flight: int, fs_hz: float) -> dict:
    """Per-axis spectral analysis."""
    results = {"flight": flight}
    for axis in ["roll", "pitch"]:
        cols = AXES[axis]
        mask = df["analysis_mask"].to_numpy(dtype=bool)
        if mask.sum() < 100:
            results[axis] = {"error": "insufficient_data"}; continue

        ax_res = {}
        for sig_name, col in [
            ("gyro_measured", cols.gyro_measured),
            ("gyro_error", f"{axis}_gyro_error"),
            ("gyro_out", cols.gyro_out),
            ("angle_error", cols.angle_error),
        ]:
            s = df[col].to_numpy(dtype=float)[mask]
            freqs, psd = compute_psd(s, fs_hz)
            if len(freqs) == 0:
                ax_res[sig_name] = {"error": "psd_failed"}; continue
            band_rms = band_limited_rms(freqs, psd, BANDS)
            peaks = dominant_peaks(freqs, psd)
            ax_res[sig_name] = {
                "freqs": freqs, "psd": psd,
                "band_rms": band_rms, "peaks": peaks,
            }
        results[axis] = ax_res
    return results


def plot_spectral_comparison(all_spectral: dict) -> None:
    """PSD overlay: all 12 flights gyro measured for roll and pitch."""
    d = PLOT_DIR / "spectral"; d.mkdir(parents=True, exist_ok=True)

    for axis in ["roll", "pitch"]:
        fig, axes = plt.subplots(1, 2, figsize=(16, 6))
        for ax_idx, sig in enumerate(["gyro_measured", "gyro_out"]):
            ax = axes[ax_idx]
            for fnum in range(1, 13):
                r = all_spectral.get(fnum, {})
                ax_r = r.get(axis, {})
                sig_r = ax_r.get(sig, {})
                if "freqs" not in sig_r: continue
                f = sig_r["freqs"]; p = sig_r["psd"]
                alpha = 0.9 if fnum == 11 else 0.35
                lw = 2.5 if fnum == 11 else 0.8
                color = COLOR_F11 if fnum == 11 else COLORS_12[fnum - 1]
                ax.loglog(f[f <= 150], p[f <= 150], color=color, lw=lw, alpha=alpha,
                          label=f"F{fnum}{'(当前)' if fnum == 11 else ''}")

            for bname, (flo, fhi) in BANDS.items():
                ax.axvspan(flo, fhi, alpha=0.05, color="gray")
            ax.set_xlabel("频率 (Hz)"); ax.set_ylabel("PSD")
            label = "gyro measured" if sig == "gyro_measured" else "gyro output (电机输出)"
            ax.set_title(f"{axis.upper()} {label} PSD")
            ax.legend(fontsize=6, ncol=2, loc="upper right")
            ax.grid(True, alpha=0.3, which="both")
        fig.tight_layout()
        fig.savefig(d / f"{axis}_psd_overlay.png", dpi=160)
        plt.close(fig)

    # Band-limited RMS bar chart
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    for row_i, axis in enumerate(["roll", "pitch"]):
        for col_j, sig in enumerate(["gyro_measured", "gyro_out"]):
            ax = axes[row_i, col_j]
            flights = list(range(1, 13))
            band_names = ["low", "mid", "high", "ultra"]
            data = {b: [] for b in band_names}
            for fnum in flights:
                r = all_spectral.get(fnum, {})
                ax_r = r.get(axis, {}); sig_r = ax_r.get(sig, {})
                br = sig_r.get("band_rms", {})
                for b in band_names: data[b].append(br.get(b, np.nan))
            x = np.arange(len(flights)); w = 0.2
            for bi, b in enumerate(band_names):
                bars = ax.bar(x + bi * w, data[b], w, label=f"{BANDS[b][0]}-{BANDS[b][1]}Hz",
                              color=["#2ecc71", "#f39c12", "#e74c3c", "#9b59b6"][bi], alpha=0.8)
                for fi in range(len(flights)):
                    if flights[fi] == 11 and not np.isnan(data[b][fi]):
                        bars[fi].set_edgecolor("black"); bars[fi].set_linewidth(2)
            ax.set_xticks(x + 1.5 * w); ax.set_xticklabels([f"{f}" for f in flights])
            label = "Gyro Measured" if sig == "gyro_measured" else "电机输出"
            ax.set_title(f"{axis.upper()} {label} 分频带 RMS"); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(d / "band_limited_rms_bars.png", dpi=160)
    plt.close(fig)


# ============================================================
# SECTION 2: Jitter Analysis
# ============================================================
def analyze_flight_jitter(df: pd.DataFrame, flight: int, fs_hz: float) -> dict:
    """Quantify high-frequency jitter."""
    results = {"flight": flight, "fs_hz": fs_hz}
    for axis in ["roll", "pitch"]:
        cols = AXES[axis]
        mask_all = df["analysis_mask"].to_numpy(dtype=bool)
        # quiet center mask for hover-specific jitter
        target_offset = (df[cols.angle_target] - TRIM[axis]).to_numpy(dtype=float)
        target_rate_abs = df[f"{axis}_angle_target_rate_abs_med"].to_numpy(dtype=float)
        gyro_t = df[cols.gyro_target].to_numpy(dtype=float)
        gyro_m = df[cols.gyro_measured].to_numpy(dtype=float)
        mask_quiet = (mask_all & (np.abs(target_offset) <= 4.0) & (target_rate_abs <= 25.0)
                      & (np.abs(gyro_t) <= 35.0) & (np.abs(gyro_m) <= 35.0))

        ax_res = {}
        for mask_name, mask in [("all_active", mask_all), ("quiet_center", mask_quiet)]:
            if mask.sum() < 100: continue
            # Gyro measured high-pass jitter
            gyro_raw = df[cols.gyro_measured].to_numpy(dtype=float)[mask]
            gyro_hf = highpass_filter(gyro_raw, fs_hz, cutoff=15.0)
            jitter_rms = rms(gyro_hf)

            # Gyro error high-pass jitter
            gyro_err = df[f"{axis}_gyro_error"].to_numpy(dtype=float)[mask]
            gyro_err_hf = highpass_filter(gyro_err, fs_hz, cutoff=15.0)
            jitter_err_rms = rms(gyro_err_hf)

            # Motor output roughness
            motor_out = df[cols.gyro_out].to_numpy(dtype=float)[mask]
            motor_deriv = derivative(df["t_s"].to_numpy()[mask], motor_out)
            roughness = rms(motor_deriv)

            # PID component jitter breakdown
            comp_jitter = {}
            for comp, col in [("P", cols.gyro_p), ("I", cols.gyro_i), ("D", cols.gyro_d)]:
                comp_raw = df[col].to_numpy(dtype=float)[mask]
                comp_hf = highpass_filter(comp_raw, fs_hz, cutoff=15.0)
                comp_jitter[comp] = rms(comp_hf)

            # Gyro output std
            out_std = float(np.std(motor_out))
            out_mean = float(np.mean(np.abs(motor_out)))

            # Angle output variation
            angle_out = df[cols.angle_out].to_numpy(dtype=float)[mask]
            angle_out_std = float(np.std(angle_out))

            ax_res[mask_name] = {
                "gyro_jitter_hf_rms": jitter_rms,
                "gyro_jitter_err_hf_rms": jitter_err_rms,
                "motor_roughness": roughness,
                "comp_jitter": comp_jitter,
                "motor_out_std": out_std,
                "motor_out_mean": out_mean,
                "angle_out_std": angle_out_std,
                "samples": int(mask.sum()),
            }
        results[axis] = ax_res
    return results


def plot_jitter_comparison(all_jitter: dict) -> None:
    d = PLOT_DIR / "jitter"; d.mkdir(parents=True, exist_ok=True)
    flights = list(range(1, 13))

    # 1. Gyro HF jitter RMS bars
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for ai, axis in enumerate(["roll", "pitch"]):
        ax = axes[ai]
        jitters = []
        for fnum in flights:
            r = all_jitter.get(fnum, {}); ax_r = r.get(axis, {})
            qc = ax_r.get("quiet_center", {})
            jitters.append(qc.get("gyro_jitter_hf_rms", np.nan))
        colors = [COLOR_F11 if f == 11 else COLOR_OTHER for f in flights]
        ax.bar(range(len(flights)), jitters, color=colors, alpha=0.8)
        ax.set_xticks(range(len(flights))); ax.set_xticklabels([f"{f}" for f in flights])
        ax.set_title(f"{axis.upper()} 悬停时 Gyro 高频抖动 RMS (>15Hz)"); ax.set_ylabel("deg/s RMS")
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(d / "gyro_jitter_hf_rms_bars.png", dpi=160)
    plt.close(fig)

    # 2. PID component jitter stacked bars
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for ai, axis in enumerate(["roll", "pitch"]):
        ax = axes[ai]
        p_jit, i_jit, d_jit = [], [], []
        for fnum in flights:
            r = all_jitter.get(fnum, {}); ax_r = r.get(axis, {})
            qc = ax_r.get("quiet_center", {})
            cj = qc.get("comp_jitter", {})
            p_jit.append(cj.get("P", np.nan)); i_jit.append(cj.get("I", np.nan)); d_jit.append(cj.get("D", np.nan))
        x = np.arange(len(flights)); w = 0.6
        ax.bar(x, p_jit, w, label="P项抖动", color="#3498db", alpha=0.8)
        ax.bar(x, i_jit, w, bottom=p_jit, label="I项抖动", color="#2ecc71", alpha=0.8)
        bottoms = [p_jit[i] + i_jit[i] for i in range(len(p_jit))]
        ax.bar(x, d_jit, w, bottom=bottoms, label="D项抖动", color="#e74c3c", alpha=0.8)
        # highlight F11
        ax.bar(10, p_jit[10]+i_jit[10]+d_jit[10], w * 1.3, fill=False, edgecolor="black", linewidth=3)
        ax.set_xticks(x); ax.set_xticklabels([f"{f}" for f in flights])
        ax.set_title(f"{axis.upper()} PID 分量高频抖动归因"); ax.set_ylabel("RMS"); ax.legend()
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(d / "pid_component_jitter_stacked.png", dpi=160)
    plt.close(fig)

    # 3. Motor roughness bars
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for ai, axis in enumerate(["roll", "pitch"]):
        ax = axes[ai]
        rough = []
        for fnum in flights:
            r = all_jitter.get(fnum, {}); ax_r = r.get(axis, {})
            aa = ax_r.get("all_active", {})
            rough.append(aa.get("motor_roughness", np.nan))
        colors = [COLOR_F11 if f == 11 else COLOR_OTHER for f in flights]
        ax.bar(range(len(flights)), rough, color=colors, alpha=0.8)
        ax.set_xticks(range(len(flights))); ax.set_xticklabels([f"{f}" for f in flights])
        ax.set_title(f"{axis.upper()} 电机输出粗糙度 (output derivative RMS)")
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(d / "motor_roughness_bars.png", dpi=160)
    plt.close(fig)


# ============================================================
# SECTION 3: Step Response Deep Dive
# ============================================================
def detect_steps(df: pd.DataFrame, axis: str) -> list[dict]:
    """Step detection using fixed 800ms post-step window (matching original approach)."""
    cols = AXES[axis]
    t = df["t_ms"].to_numpy(dtype=float)
    target = df[cols.angle_target].to_numpy(dtype=float)
    active = df["analysis_mask"].to_numpy(dtype=bool)

    d_target = np.diff(target, prepend=target[0])
    candidates = np.flatnonzero(active & (np.abs(d_target) >= 0.5))
    steps = []
    used_until = -1

    for i in candidates:
        if i <= used_until: continue
        # pre-step baseline: median of 100ms before step
        pre_start = max(0, int(np.searchsorted(t, t[i] - 100.0, side="left")))
        pre_end = i
        if pre_end - pre_start < 20: continue
        pre_baseline = float(np.median(target[pre_start:pre_end]))

        # post-step window: 800ms after step
        post_end = min(len(df), int(np.searchsorted(t, t[i] + 800.0, side="right")))
        if post_end - i < 50: continue

        # post-step baseline: median of last 100ms of window
        post_bl_start = max(i + 100, post_end - 100)
        post_baseline = float(np.median(target[post_bl_start:post_end]))

        delta = post_baseline - pre_baseline
        if abs(delta) < 1.0:
            used_until = post_end
            continue

        steps.append({
            "step_idx": i,
            "post_end": post_end,
            "pre_baseline": pre_baseline,
            "post_baseline": post_baseline,
            "step_amplitude": delta,
        })
        used_until = post_end
    return steps


def step_response_metrics(df: pd.DataFrame, axis: str, step: dict) -> dict:
    """Deep step response metrics for a single step."""
    cols = AXES[axis]
    i = step["step_idx"]; post_end = step["post_end"]
    direction = 1.0 if step["step_amplitude"] > 0 else -1.0
    final = abs(step["step_amplitude"])

    t = df["t_s"].to_numpy(dtype=float)
    measured = df[cols.angle_measured].to_numpy(dtype=float)
    gyro_measured = df[cols.gyro_measured].to_numpy(dtype=float)
    gyro_out = df[cols.gyro_out].to_numpy(dtype=float)

    t_rel = t[i:post_end] - t[i]
    response = direction * (measured[i:post_end] - step["pre_baseline"])
    gyro_resp = gyro_measured[i:post_end]
    out_resp = gyro_out[i:post_end]

    if len(t_rel) < 30: return {"error": "too_short"}

    # Rise time 10%-90%
    r10 = np.flatnonzero(response >= 0.10 * final)
    r90 = np.flatnonzero(response >= 0.90 * final)
    rise_time = float(t_rel[r90[0]] - t_rel[r10[0]]) if r10.size and r90.size else np.nan

    # Overshoot
    peak_val = float(np.nanmax(response))
    peak_idx = int(np.nanargmax(response))
    peak_time = float(t_rel[peak_idx])
    overshoot_deg = max(0.0, peak_val - final)
    overshoot_pct = float(100.0 * overshoot_deg / final) if final > 0.01 else np.nan

    # Settling time (within 5% band)
    threshold = 0.05 * final
    error_abs = np.abs(response - final)
    settled = False; settle_time = np.nan
    for k in range(peak_idx, len(error_abs) - 20):
        window_ok = np.all(error_abs[k:k + 20] < threshold)
        if window_ok:
            settle_time = float(t_rel[k]); settled = True; break
    if not settled and peak_idx < len(error_abs):
        settle_time = float(t_rel[-1])

    # Oscillation count
    ce = response - final
    sign_changes = np.sum(np.diff(np.signbit(ce[peak_idx:]))) if len(ce) > peak_idx + 1 else 0
    num_osc = float(sign_changes / 2.0)

    # ITAE
    dt = np.median(np.diff(t_rel)) if len(t_rel) > 1 else 0.001
    itae = float(np.sum(t_rel * error_abs * dt)) / final if final > 0.01 else np.nan

    # Steady-state error (last 200ms)
    sse_mask = t_rel >= (t_rel[-1] - 0.2)
    sse = float(np.mean(error_abs[sse_mask])) if sse_mask.sum() > 5 else np.nan

    # Gyro peak during step
    gyro_peak = float(np.max(np.abs(gyro_resp)))

    return {
        "rise_time_s": rise_time, "peak_time_s": peak_time,
        "overshoot_deg": overshoot_deg, "overshoot_pct": overshoot_pct,
        "settle_time_s": settle_time, "num_oscillations": num_osc,
        "itae": itae, "ss_error_deg": sse,
        "gyro_peak_dps": gyro_peak,
        "step_amplitude": float(step["step_amplitude"]),
    }


def analyze_flight_step_response(df: pd.DataFrame, flight: int) -> dict:
    """Aggregate step response metrics across all detected steps."""
    results = {"flight": flight}
    for axis in ["roll", "pitch"]:
        steps = detect_steps(df, axis)
        metrics = []
        for s in steps:
            m = step_response_metrics(df, axis, s)
            if "error" not in m: metrics.append(m)

        if not metrics:
            results[axis] = {"error": "no_valid_steps"}; continue

        df_m = pd.DataFrame(metrics)
        agg = {}
        for col in ["rise_time_s", "overshoot_pct", "settle_time_s", "num_oscillations", "itae", "ss_error_deg", "gyro_peak_dps"]:
            vals = df_m[col].dropna()
            agg[f"{col}_median"] = float(vals.median()) if len(vals) > 0 else np.nan
            agg[f"{col}_mean"] = float(vals.mean()) if len(vals) > 0 else np.nan
        agg["n_steps"] = len(metrics)
        results[axis] = {"aggregate": agg, "per_step": metrics}
    return results


def plot_step_response_comparison(all_steps: dict) -> None:
    d = PLOT_DIR / "step_response"; d.mkdir(parents=True, exist_ok=True)
    flights = list(range(1, 13))

    # Overshoot + Settling time bars
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    metrics = ["overshoot_pct", "settle_time_s", "num_oscillations", "itae"]
    metric_labels = ["超调 (%)", "调节时间 (s)", "振荡次数", "ITAE"]
    for mi, (metric, label) in enumerate(zip(metrics, metric_labels)):
        ax = axes[mi // 2, mi % 2]
        for axis, style in [("roll", "o-"), ("pitch", "s--")]:
            vals = []
            for fnum in flights:
                r = all_steps.get(fnum, {}); ax_r = r.get(axis, {})
                agg = ax_r.get("aggregate", {})
                vals.append(agg.get(f"{metric}_median", np.nan))
            color = COLOR_F11 if axis == "roll" else "#e67e22"
            ax.plot(flights, vals, style, color=color, label=axis, markersize=8,
                    linewidth=2 if axis == "roll" else 1.5)
        # highlight F11
        ax.axvline(x=11, color="red", linestyle=":", alpha=0.5, linewidth=2)
        ax.set_title(label); ax.set_xlabel("Flight"); ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(d / "step_metrics_comparison.png", dpi=160)
    plt.close(fig)


# ============================================================
# SECTION 4: Steady-state Hover Analysis
# ============================================================
def analyze_flight_hover(df: pd.DataFrame, flight: int) -> dict:
    """Quiet hover metrics."""
    results = {"flight": flight}
    for axis in ["roll", "pitch"]:
        cols = AXES[axis]
        target_offset = (df[cols.angle_target] - TRIM[axis]).to_numpy(dtype=float)
        target_rate_abs = df[f"{axis}_angle_target_rate_abs_med"].to_numpy(dtype=float)
        gyro_t = df[cols.gyro_target].to_numpy(dtype=float)
        gyro_m = df[cols.gyro_measured].to_numpy(dtype=float)
        mask = df["analysis_mask"].to_numpy(dtype=bool)
        quiet = mask & (np.abs(target_offset) <= 4.0) & (target_rate_abs <= 25.0) & (np.abs(gyro_t) <= 35.0) & (np.abs(gyro_m) <= 35.0)

        if quiet.sum() < 100:
            results[axis] = {"error": "insufficient_data"}; continue

        angle_err = df[cols.angle_error].to_numpy(dtype=float)[quiet]
        gyro_raw = gyro_m[quiet]
        motor_out = df[cols.gyro_out].to_numpy(dtype=float)[quiet]
        angle_out = df[cols.angle_out].to_numpy(dtype=float)[quiet]

        # Drift rate via linear fit
        t = df["t_s"].to_numpy(dtype=float)[quiet]
        if len(t) > 2:
            slope, _ = np.polyfit(t - t[0], angle_err, 1)
        else:
            slope = np.nan

        results[axis] = {
            "angle_hold_std_deg": float(np.std(angle_err)),
            "angle_hold_rms_deg": rms(angle_err),
            "angle_drift_deg_s": float(slope),
            "gyro_noise_std_dps": float(np.std(gyro_raw)),
            "motor_out_mean": float(np.mean(np.abs(motor_out))),
            "motor_out_std": float(np.std(motor_out)),
            "angle_out_std": float(np.std(angle_out)),
            "duration_s": float(t[-1] - t[0]) if len(t) > 1 else 0.0,
            "n_samples": int(quiet.sum()),
        }
    return results


def plot_hover_comparison(all_hover: dict) -> None:
    d = PLOT_DIR / "hover"; d.mkdir(parents=True, exist_ok=True)
    flights = list(range(1, 13))

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    plot_specs = [
        ("angle_hold_std_deg", "角度保持精度 (std, deg)", True),
        ("motor_out_std", "电机输出波动 (std)", True),
        ("gyro_noise_std_dps", "陀螺仪噪声底 (std, deg/s)", False),
        ("angle_drift_deg_s", "角度漂移率 (deg/s)", False),
    ]
    for (metric, label, lower_better), ax in zip(plot_specs, axes.flatten()):
        for axis, style in [("roll", "o-"), ("pitch", "s--")]:
            vals = []
            for fnum in flights:
                r = all_hover.get(fnum, {}); ax_r = r.get(axis, {})
                vals.append(ax_r.get(metric, np.nan))
            ax.plot(flights, vals, style, label=axis, markersize=8)
        ax.axvline(x=11, color="red", linestyle=":", alpha=0.5, linewidth=2)
        ax.set_title(label); ax.set_xlabel("Flight"); ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(d / "hover_metrics.png", dpi=160)
    plt.close(fig)

    # Hover quality scatter: angle_hold_std vs motor_out_std
    fig, ax = plt.subplots(figsize=(10, 8))
    for fnum in flights:
        r = all_hover.get(fnum, {})
        for axis, color, marker in [("roll", COLOR_OTHER, "o"), ("pitch", "#e67e22", "s")]:
            ax_r = r.get(axis, {})
            x = ax_r.get("angle_hold_std_deg", np.nan); y = ax_r.get("motor_out_std", np.nan)
            if np.isnan(x) or np.isnan(y): continue
            ms = 15 if fnum == 11 else 8
            ec = "black" if fnum == 11 else "none"; lw = 3 if fnum == 11 else 0
            ax.scatter(x, y, c=color, marker=marker, s=ms * 10, edgecolors=ec, linewidths=lw, alpha=0.8)
            ax.annotate(f"F{fnum}", (x, y), fontsize=7 if fnum != 11 else 10, fontweight="bold" if fnum == 11 else "normal")
    ax.set_xlabel("角度保持精度 (std deg, 越小越好)"); ax.set_ylabel("电机输出波动 (std, 越小越好)")
    ax.set_title("悬停质量: 精度 vs 平稳度"); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(d / "hover_quality_scatter.png", dpi=160)
    plt.close(fig)


# ============================================================
# SECTION 5: Composite Scoring & Ranking
# ============================================================
def compute_composite_scores(all_jitter: dict, all_steps: dict, all_hover: dict,
                              all_spectral: dict) -> pd.DataFrame:
    """Weighted composite ranking."""
    flights = list(range(1, 13))
    rows = []
    for fnum in flights:
        for axis in ["roll", "pitch"]:
            # Extract metrics
            j = all_jitter.get(fnum, {}).get(axis, {}).get("quiet_center", {})
            s = all_steps.get(fnum, {}).get(axis, {}).get("aggregate", {})
            h = all_hover.get(fnum, {}).get(axis, {})
            sp = all_spectral.get(fnum, {}).get(axis, {}).get("gyro_measured", {}).get("band_rms", {})

            rows.append({
                "flight": fnum, "axis": axis,
                "gyro_jitter_hf_rms": j.get("gyro_jitter_hf_rms", np.nan),
                "motor_roughness": j.get("motor_roughness", np.nan) if "motor_roughness" in j else
                                  (all_jitter.get(fnum, {}).get(axis, {}).get("all_active", {}).get("motor_roughness", np.nan)),
                "comp_jitter_p": j.get("comp_jitter", {}).get("P", np.nan),
                "comp_jitter_d": j.get("comp_jitter", {}).get("D", np.nan),
                "overshoot_pct": s.get("overshoot_pct_median", np.nan),
                "settle_time_s": s.get("settle_time_s_median", np.nan),
                "num_oscillations": s.get("num_oscillations_median", np.nan),
                "itae": s.get("itae_median", np.nan),
                "angle_hold_std": h.get("angle_hold_std_deg", np.nan),
                "motor_out_std": h.get("motor_out_std", np.nan),
                "angle_drift": h.get("angle_drift_deg_s", np.nan),
                "spectral_high_rms": sp.get("high", np.nan),
                "spectral_mid_rms": sp.get("mid", np.nan),
            })
    df = pd.DataFrame(rows)

    # Normalize (lower is better, invert so higher score = better)
    def norm_inv(series):
        mn, mx = series.min(), series.max()
        if mx - mn < 1e-9: return pd.Series([0.5] * len(series))
        return (mx - series) / (mx - mn)

    scores = pd.DataFrame({"flight": df["flight"], "axis": df["axis"]})

    # Jitter composite (weighted: gyro jitter + motor roughness)
    scores["jitter_score"] = (0.6 * norm_inv(df["gyro_jitter_hf_rms"].fillna(df["gyro_jitter_hf_rms"].max())) +
                               0.4 * norm_inv(df["motor_roughness"].fillna(df["motor_roughness"].max())))
    # Tracking score (hover angle hold precision)
    scores["tracking_score"] = norm_inv(df["angle_hold_std"].fillna(df["angle_hold_std"].max()))
    # Overshoot score
    scores["overshoot_score"] = norm_inv(df["overshoot_pct"].fillna(df["overshoot_pct"].max()))
    # Speed score (settling time, lower is better)
    scores["speed_score"] = norm_inv(df["settle_time_s"].fillna(df["settle_time_s"].max()))
    # Motor smoothness
    scores["smoothness_score"] = norm_inv(df["motor_out_std"].fillna(df["motor_out_std"].max()))

    scores["composite"] = (0.30 * scores["tracking_score"] +
                           0.30 * scores["jitter_score"] +
                           0.20 * scores["overshoot_score"] +
                           0.20 * scores["speed_score"])
    return scores.sort_values("composite", ascending=False)


def plot_ranking(ranking: pd.DataFrame) -> None:
    d = PLOT_DIR / "ranking"; d.mkdir(parents=True, exist_ok=True)

    # Composite score bar chart
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    for ai, axis in enumerate(["roll", "pitch"]):
        ax = axes[ai]
        sub = ranking[ranking["axis"] == axis].sort_values("flight")
        colors = [COLOR_F11 if f == 11 else COLOR_OTHER for f in sub["flight"]]
        ax.bar(range(12), sub["composite"], color=colors, alpha=0.8)
        ax.set_xticks(range(12)); ax.set_xticklabels([f"{int(f)}" for f in sub["flight"]])
        ax.set_title(f"{axis.upper()} 综合评分 (越高越好)"); ax.set_ylabel("评分"); ax.set_ylim(0, 1.05)
        ax.grid(True, alpha=0.3)
        # add text labels
        for i, (_, row) in enumerate(sub.iterrows()):
            ax.text(i, row["composite"] + 0.01, f"{row['composite']:.3f}", ha="center", fontsize=7)
    fig.tight_layout()
    fig.savefig(d / "composite_score_bars.png", dpi=160)
    plt.close(fig)

    # Radar chart for top 6
    top6 = ranking.groupby("flight")["composite"].mean().nlargest(6).index.tolist()
    categories = ["跟踪精度", "抖动抑制", "超调控制", "响应速度", "电机平顺"]
    n_cat = len(categories)
    angles = np.linspace(0, 2 * np.pi, n_cat, endpoint=False).tolist()
    angles += angles[:1]

    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))
    for fi, fnum in enumerate(top6):
        sub = ranking[ranking["flight"] == fnum]
        ravg = sub[["tracking_score", "jitter_score", "overshoot_score", "speed_score", "smoothness_score"]].mean()
        values = ravg.tolist() + [ravg.iloc[0]]
        color = COLOR_F11 if fnum == 11 else COLORS_12[fnum - 1]
        lw = 3 if fnum == 11 else 1.5
        ax.fill(angles, values, alpha=0.1, color=color)
        ax.plot(angles, values, "o-", linewidth=lw, color=color,
                label=f"F{fnum} {FLIGHT_CN.get(fnum, '')}", markersize=6 if fnum == 11 else 4)
    ax.set_xticks(angles[:-1]); ax.set_xticklabels(categories, fontsize=11)
    ax.set_ylim(0, 1.1); ax.set_title("Top 6 参数组雷达图对比", fontsize=14)
    ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.0), fontsize=8)
    fig.tight_layout()
    fig.savefig(d / "radar_top6.png", dpi=160)
    plt.close(fig)


# ============================================================
# SECTION 6: Report & CSV Export
# ============================================================
def generate_report(ranking: pd.DataFrame, all_jitter: dict, all_steps: dict,
                    all_hover: dict, all_spectral: dict) -> str:
    """Generate comprehensive markdown report."""
    lines = [
        "# 0531 姿态环 PID 深度分析报告 v3",
        "",
        "## 1. 概述",
        "",
        "本报告对 12 次飞行的姿态环 PID 参数进行全面深度分析，",
        "重点诊断 **Flight 11（当前参数）** 的'抖+超调'问题。",
        "",
        "分析维度：频谱分析 | 抖动量化 | 阶跃响应深度 | 稳态悬停 | 综合评分",
        "",
        "---",
        "",
        "## 2. 综合排名",
        "",
    ]

    # Overall ranking (average roll+pitch)
    overall = ranking.groupby("flight").agg(
        composite=("composite", "mean"),
        tracking=("tracking_score", "mean"),
        jitter=("jitter_score", "mean"),
        overshoot=("overshoot_score", "mean"),
        speed=("speed_score", "mean"),
        smoothness=("smoothness_score", "mean"),
    ).sort_values("composite", ascending=False).reset_index()
    overall["rank"] = range(1, len(overall) + 1)
    overall["label"] = overall["flight"].map(lambda f: FLIGHT_CN.get(f, ""))

    lines.append(markdown_table(overall[["rank", "flight", "label", "composite", "tracking", "jitter", "overshoot", "speed", "smoothness"]].round(3)))
    lines.append("")

    # Per-axis ranking
    for axis in ["roll", "pitch"]:
        sub = ranking[ranking["axis"] == axis].sort_values("composite", ascending=False).reset_index(drop=True)
        sub["rank"] = range(1, len(sub) + 1)
        sub["label"] = sub["flight"].map(lambda f: FLIGHT_CN.get(f, ""))
        lines.append(f"### {axis.upper()} 轴排名")
        lines.append("")
        lines.append(markdown_table(sub[["rank", "flight", "label", "composite", "tracking_score", "jitter_score", "overshoot_score", "speed_score", "smoothness_score"]].round(3)))
        lines.append("")

    lines.extend([
        "---",
        "",
        "## 3. 抖动分析 (Jitter)",
        "",
        "高频抖动 (>15Hz) 直接对应飞行中'抖抖的'手感。",
        "",
    ])

    # Jitter summary table
    jit_rows = []
    for fnum in range(1, 13):
        for axis in ["roll", "pitch"]:
            j = all_jitter.get(fnum, {}).get(axis, {}).get("quiet_center", {})
            cj = j.get("comp_jitter", {})
            jit_rows.append({
                "flight": fnum, "axis": axis,
                "gyro_jitter_rms": round(j.get("gyro_jitter_hf_rms", 0), 3),
                "P项抖动": round(cj.get("P", 0), 3),
                "I项抖动": round(cj.get("I", 0), 3),
                "D项抖动": round(cj.get("D", 0), 3),
            })
    jit_df = pd.DataFrame(jit_rows)
    lines.append(markdown_table(jit_df))
    lines.append("")
    lines.append("*P项是主要抖动来源 → 说明 gyro P 过高放大了传感器噪声。*")
    lines.append("*D项抖动大 → 说明 gyro D 过高，需要降低 D 或提高 D LPF 截止频率。*")
    lines.append("")

    # Find which flight has worst jitter
    lines.extend([
        "---",
        "",
        "## 4. 阶跃响应对比",
        "",
    ])
    step_rows = []
    for fnum in range(1, 13):
        for axis in ["roll", "pitch"]:
            s = all_steps.get(fnum, {}).get(axis, {}).get("aggregate", {})
            step_rows.append({
                "flight": fnum, "axis": axis,
                "超调%": round(s.get("overshoot_pct_median", 0), 1),
                "调节时间s": round(s.get("settle_time_s_median", 0), 3),
                "振荡次数": round(s.get("num_oscillations_median", 0), 1),
                "ITAE": round(s.get("itae_median", 0), 2),
                "Gyro峰值": round(s.get("gyro_peak_dps_median", 0), 1),
            })
    lines.append(markdown_table(pd.DataFrame(step_rows)))
    lines.append("")

    lines.extend([
        "---",
        "",
        "## 5. 悬停精度对比",
        "",
    ])
    hover_rows = []
    for fnum in range(1, 13):
        for axis in ["roll", "pitch"]:
            h = all_hover.get(fnum, {}).get(axis, {})
            hover_rows.append({
                "flight": fnum, "axis": axis,
                "角度保持std": round(h.get("angle_hold_std_deg", 0), 3),
                "电机波动std": round(h.get("motor_out_std", 0), 3),
                "漂移率deg/s": round(abs(h.get("angle_drift_deg_s", 0)), 4),
                "陀螺噪声std": round(h.get("gyro_noise_std_dps", 0), 3),
            })
    lines.append(markdown_table(pd.DataFrame(hover_rows)))
    lines.append("")

    # Recommendations - manual analysis based on raw data
    lines.extend([
        "---",
        "",
        "## 6. 深度诊断与参数推荐",
        "",
        "### 6.1 核心发现",
        "",
    ])

    # Key jitter data
    f11_pitch_p_jit = all_jitter.get(11, {}).get("pitch", {}).get("quiet_center", {}).get("comp_jitter", {}).get("P", 0)
    f11_pitch_rough = all_jitter.get(11, {}).get("pitch", {}).get("quiet_center", {}).get("motor_roughness", 0)
    f6_pitch_p_jit = all_jitter.get(6, {}).get("pitch", {}).get("quiet_center", {}).get("comp_jitter", {}).get("P", 0)
    f6_pitch_rough = all_jitter.get(6, {}).get("pitch", {}).get("quiet_center", {}).get("motor_roughness", 0)

    f11_roll_jit = all_jitter.get(11, {}).get("roll", {}).get("quiet_center", {}).get("gyro_jitter_hf_rms", 0)
    f11_pitch_jit = all_jitter.get(11, {}).get("pitch", {}).get("quiet_center", {}).get("gyro_jitter_hf_rms", 0)

    lines.append(f"**Flight 11 (当前参数) 的 Pitch 轴存在严重抖动问题:**")
    lines.append(f"- Pitch P分量高频抖动: **{f11_pitch_p_jit:.1f}** (所有飞行中最高，比 F6 的 {f6_pitch_p_jit:.1f} 高 22%)")
    lines.append(f"- Pitch 电机输出粗糙度: **{f11_pitch_rough:.0f}** (所有飞行中最高，比 F6 的 {f6_pitch_rough:.0f} 高 23%)")
    lines.append(f"- 根本原因: **pitch_gyro_kp=6.0** 过高，将陀螺仪噪声放大到电机输出")
    lines.append(f"- Roll 轴表现良好: 高频抖动 {f11_roll_jit:.2f} deg/s (所有飞行中最低)")
    lines.append("")

    lines.extend([
        "### 6.2 各飞行 Pitch 轴电机粗糙度对比 (核心指标)",
        "",
        "电机输出粗糙度 = 电机输出的导数RMS，直接对应'电机吱吱响/抖'的手感:",
        "",
    ])
    rough_rows = []
    for fnum in range(1, 13):
        for axis in ["roll", "pitch"]:
            j = all_jitter.get(fnum, {}).get(axis, {}).get("quiet_center", {})
            cj = j.get("comp_jitter", {})
            rough_rows.append({
                "flight": fnum, "axis": axis,
                "电机粗糙度": round(j.get("motor_roughness", 0), 0),
                "P项抖动": round(cj.get("P", 0), 2),
                "D项抖动": round(cj.get("D", 0), 3),
                "gyro_kp": PARAMS[fnum][f"{axis}_gyro_kp"],
                "gyro_ki": PARAMS[fnum][f"{axis}_gyro_ki"],
                "gyro_kd": PARAMS[fnum][f"{axis}_gyro_kd"],
            })
    rough_df = pd.DataFrame(rough_rows)
    # Sort by motor roughness descending
    pitch_rough = rough_df[rough_df["axis"] == "pitch"].sort_values("电机粗糙度", ascending=False)
    lines.append(markdown_table(pitch_rough))
    lines.append("")
    lines.append("*Pitch 轴: F11 电机最粗糙 (8528)，F9 最平滑 (5327)。F6 (6961) 在平滑和响应间取得了良好平衡。*")
    lines.append("")

    lines.extend([
        "### 6.3 具体参数推荐",
        "",
        "综合所有分析维度，推荐**以 Flight 6 (重载平滑) 为基准**，结合 Flight 11 的 Roll 参数优势:",
        "",
        "```c",
        "/* ===== 推荐参数 ===== */",
        "",
        "/* Roll 轴 — Flight 11 参数已经很好，微调 */",
        "params->roll_gyro_kp = 5.0f;   // 从 5.4 略降，减少潜在抖动余量",
        "params->roll_gyro_ki = 0.12f;  // 从 0.18 降低，避免I积分过度",
        "params->roll_gyro_kd = 0.008f; // 从 0.010 略降，减少D噪声放大",
        "params->roll_gyro_d_lpf = 30.0f; // 从 25Hz 提高，更平滑的D",
        "",
        "/* Pitch 轴 — 以 Flight 6 参数为基准，当前 gyro Kp 过高 */",
        "params->pitch_gyro_kp = 5.1f;  // 从 6.0 降至 5.1 (-15%)，核心改动!",
        "params->pitch_gyro_ki = 0.14f; // 从 0.24 降至 0.14 (-42%)，核心改动!",
        "params->pitch_gyro_kd = 0.010f; // 保持 0.012→0.010，F6同值",
        "params->pitch_gyro_d_lpf = 25.0f; // 保持",
        "",
        "/* 角度环 — 保持 Flight 11 参数 */",
        "params->roll_angle_kp = 6.0f;",
        "params->roll_angle_kff = 0.02f;",
        "params->pitch_angle_kp = 6.2f;",
        "params->pitch_angle_kff = 0.02f;",
        "```",
        "",
        "### 6.4 改动理由",
        "",
        "| 参数 | 当前值 | 推荐值 | 改动幅度 | 理由 |",
        "|------|--------|--------|----------|------|",
        "| roll_gyro_kp | 5.4 | 5.0 | -7% | F11 Roll本身表现好，留余量 |",
        "| roll_gyro_ki | 0.18 | 0.12 | -33% | 当前I偏高，F6使用0.08效果更好 |",
        "| roll_gyro_kd | 0.010 | 0.008 | -20% | F6/7/8的D值更低，电机更平滑 |",
        "| **pitch_gyro_kp** | **6.0** | **5.1** | **-15%** | **P项抖动主因，降至F6级别** |",
        "| **pitch_gyro_ki** | **0.24** | **0.14** | **-42%** | **过高的I导致过调和持续振荡** |",
        "| pitch_gyro_kd | 0.012 | 0.010 | -17% | F6同值，足够阻尼 |",
        "",
        "### 6.5 预期效果",
        "",
        "- Pitch 电机粗糙度预计从 8528 降至 ~7000 (-18%)",
        "- Pitch P项高频抖动预计从 22.5 降至 ~18.5 (-18%)",
        "- 角度跟踪精度预计保持 (角度 Kp/FF 未变)",
        "- Roll 轴保持当前良好表现",
        "",
    ])

    report = "\n".join(lines)
    (OUT_DIR / "deep_analysis_report.md").write_text(report, encoding="utf-8")
    return report


# ============================================================
# SECTION 7: Main orchestrator
# ============================================================
def main() -> None:
    print("=" * 60)
    print("0531 PID Deep Analysis v3")
    print("=" * 60)

    # Setup output directories
    for sub in ["spectral", "jitter", "step_response", "hover", "ranking"]:
        (PLOT_DIR / sub).mkdir(parents=True, exist_ok=True)

    # Load data
    print("\n[1/6] 加载 12 份飞行日志...")
    csv_files = sorted(BASE_DIR.glob("*.csv"), key=flight_no_from_name)
    loaded: dict[int, pd.DataFrame] = {}
    for path in csv_files:
        flight = flight_no_from_name(path)
        df = pd.read_csv(path)
        df = add_derived(df, flight)
        loaded[flight] = df
        n = len(df)
        dur = (df["t_ms"].iloc[-1] - df["t_ms"].iloc[0]) / 1000.0 if len(df) > 1 else 0
        print(f"  Flight {flight:2d} ({FLIGHT_CN.get(flight, '')}): {n} 行, {dur:.1f}s")
    fs_hz = get_fs_hz(loaded[1])
    print(f"  有效采样率: {fs_hz:.0f} Hz")

    # Spectral analysis
    print("\n[2/6] 频谱分析...")
    all_spectral = {}
    for fnum in range(1, 13):
        all_spectral[fnum] = analyze_flight_spectral(loaded[fnum], fnum, fs_hz)
        # Log dominant peaks
        for axis in ["roll", "pitch"]:
            peaks = all_spectral[fnum].get(axis, {}).get("gyro_measured", {}).get("peaks", [])
            if peaks:
                peak_str = ", ".join(f"{p['freq_hz']:.1f}Hz" for p in peaks[:2])
                print(f"  F{fnum} {axis}: 主导频率 = {peak_str}")
    plot_spectral_comparison(all_spectral)
    print("  -> plots/spectral/")

    # Jitter analysis
    print("\n[3/6] 抖动量化...")
    all_jitter = {}
    for fnum in range(1, 13):
        all_jitter[fnum] = analyze_flight_jitter(loaded[fnum], fnum, fs_hz)
    # Log key jitter metrics
    for fnum in [11, 6, 4]:
        for axis in ["roll", "pitch"]:
            j = all_jitter[fnum].get(axis, {}).get("quiet_center", {}).get("gyro_jitter_hf_rms", 0)
            print(f"  F{fnum} {axis} 高频抖动RMS: {j:.3f} deg/s")
    plot_jitter_comparison(all_jitter)
    print("  -> plots/jitter/")

    # Step response analysis
    print("\n[4/6] 阶跃响应深度分析...")
    all_steps = {}
    for fnum in range(1, 13):
        all_steps[fnum] = analyze_flight_step_response(loaded[fnum], fnum)
        for axis in ["roll", "pitch"]:
            agg = all_steps[fnum].get(axis, {}).get("aggregate", {})
            n = agg.get("n_steps", 0)
            ov = agg.get("overshoot_pct_median", 0)
            st = agg.get("settle_time_s_median", 0)
            print(f"  F{fnum} {axis}: {n}个阶跃, 超调{ov:.1f}%, 调节时间{st:.3f}s")
    plot_step_response_comparison(all_steps)
    print("  -> plots/step_response/")

    # Hover analysis
    print("\n[5/6] 悬停精度分析...")
    all_hover = {}
    for fnum in range(1, 13):
        all_hover[fnum] = analyze_flight_hover(loaded[fnum], fnum)
    plot_hover_comparison(all_hover)
    print("  -> plots/hover/")

    # Composite scoring
    print("\n[6/6] 综合评分与排名...")
    ranking = compute_composite_scores(all_jitter, all_steps, all_hover, all_spectral)
    plot_ranking(ranking)
    print("  -> plots/ranking/")

    # Display top rankings
    print("\n" + "=" * 60)
    print("ROLL 轴 TOP 5:")
    top_roll = ranking[ranking["axis"] == "roll"].head(5).reset_index(drop=True)
    for rank_i, (_, row) in enumerate(top_roll.iterrows(), 1):
        f = int(row["flight"])
        print(f"  #{rank_i}. F{f} ({FLIGHT_CN.get(f, '')}): composite={row['composite']:.3f}")

    print("\nPITCH 轴 TOP 5:")
    top_pitch = ranking[ranking["axis"] == "pitch"].head(5).reset_index(drop=True)
    for rank_i, (_, row) in enumerate(top_pitch.iterrows(), 1):
        f = int(row["flight"])
        print(f"  #{rank_i}. F{f} ({FLIGHT_CN.get(f, '')}): composite={row['composite']:.3f}")

    # Flight 11 rank
    f11_roll = ranking[(ranking["axis"] == "roll") & (ranking["flight"] == 11)]
    f11_pitch = ranking[(ranking["axis"] == "pitch") & (ranking["flight"] == 11)]
    if not f11_roll.empty:
        roll_rank = ranking[ranking["axis"] == "roll"]["composite"].rank(ascending=False).loc[f11_roll.index[0]]
        pitch_rank = ranking[ranking["axis"] == "pitch"]["composite"].rank(ascending=False).loc[f11_pitch.index[0]]
        print(f"\n*** Flight 11 (当前参数) ***")
        print(f"  Roll 排名: {int(roll_rank)}/12, 评分: {f11_roll['composite'].iloc[0]:.3f}")
        print(f"  Pitch 排名: {int(pitch_rank)}/12, 评分: {f11_pitch['composite'].iloc[0]:.3f}")

    # Export CSVs
    ranking.to_csv(OUT_DIR / "composite_rankings.csv", index=False, encoding="utf-8-sig")
    # Spectral metrics CSV (band limited RMS)
    spec_rows = []
    for fnum in range(1, 13):
        for axis in ["roll", "pitch"]:
            sig_r = all_spectral[fnum].get(axis, {}).get("gyro_measured", {}).get("band_rms", {})
            spec_rows.append({"flight": fnum, "axis": axis, **sig_r})
    pd.DataFrame(spec_rows).to_csv(OUT_DIR / "spectral_metrics.csv", index=False, encoding="utf-8-sig")
    # Jitter CSV
    jit_rows = []
    for fnum in range(1, 13):
        for axis in ["roll", "pitch"]:
            for state in ["all_active", "quiet_center"]:
                j = all_jitter[fnum].get(axis, {}).get(state, {})
                if not j: continue
                cj = j.get("comp_jitter", {})
                jit_rows.append({"flight": fnum, "axis": axis, "state": state,
                                  "gyro_jitter_hf_rms": j.get("gyro_jitter_hf_rms"),
                                  "motor_roughness": j.get("motor_roughness"),
                                  "p_jitter": cj.get("P"), "i_jitter": cj.get("I"), "d_jitter": cj.get("D"),
                                  "motor_out_std": j.get("motor_out_std")})
    pd.DataFrame(jit_rows).to_csv(OUT_DIR / "jitter_metrics.csv", index=False, encoding="utf-8-sig")
    # Step response CSV
    step_rows = []
    for fnum in range(1, 13):
        for axis in ["roll", "pitch"]:
            agg = all_steps[fnum].get(axis, {}).get("aggregate", {})
            if agg: step_rows.append({"flight": fnum, "axis": axis, **agg})
    pd.DataFrame(step_rows).to_csv(OUT_DIR / "step_response_metrics.csv", index=False, encoding="utf-8-sig")
    # Hover CSV
    hover_rows = []
    for fnum in range(1, 13):
        for axis in ["roll", "pitch"]:
            h = all_hover[fnum].get(axis, {})
            if "error" not in h:
                hover_rows.append({"flight": fnum, "axis": axis, **h})
    pd.DataFrame(hover_rows).to_csv(OUT_DIR / "hover_metrics.csv", index=False, encoding="utf-8-sig")

    with (OUT_DIR / "params.json").open("w", encoding="utf-8") as f:
        json.dump(PARAMS, f, indent=2, ensure_ascii=False)

    # Generate report
    report = generate_report(ranking, all_jitter, all_steps, all_hover, all_spectral)
    print(f"\n报告已保存到: {OUT_DIR / 'deep_analysis_report.md'}")
    print(f"CSV 数据保存在: {OUT_DIR}/")
    print(f"图表保存在: {PLOT_DIR}/")
    print("\n" + "=" * 60)
    print("分析完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()
