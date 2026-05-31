from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import signal


FS_HZ = 1000.0
LOG_GLOB = "*1000hz.csv"

COLUMNS = {
    "t_ms": "I0",
    "diff_ms": "I1",
    "gyro_x": "I2",
    "gyro_y": "I3",
    "gyro_z": "I4",
    "acc_x": "I5",
    "acc_y": "I6",
    "acc_z": "I7",
    "roll": "I8",
    "pitch": "I9",
    "yaw": "I10",
    "roll_gyro_target": "I11",
    "pitch_gyro_target": "I12",
    "yaw_gyro_target": "I13",
    "roll_angle_target": "I14",
    "pitch_angle_target": "I15",
    "yaw_angle_target": "I16",
    "flow_x": "I17",
    "flow_y": "I18",
}

GYRO_COLS = ["gyro_x", "gyro_y", "gyro_z"]
ACC_COLS = ["acc_x", "acc_y", "acc_z"]
EULER_COLS = ["roll", "pitch", "yaw"]
GYRO_TARGET_COLS = ["roll_gyro_target", "pitch_gyro_target", "yaw_gyro_target"]
ANGLE_TARGET_COLS = ["roll_angle_target", "pitch_angle_target", "yaw_angle_target"]
FLOW_COLS = ["flow_x", "flow_y"]


def load_log(base: Path) -> pd.DataFrame:
    files = sorted(base.glob(LOG_GLOB), key=lambda p: p.stat().st_mtime, reverse=True)
    if not files:
        raise FileNotFoundError(f"No log matching {LOG_GLOB} under {base}")

    df = pd.read_csv(files[0])
    reverse_columns = {value: key for key, value in COLUMNS.items()}
    df = df.rename(columns=reverse_columns)
    df.attrs["source_file"] = str(files[0])
    return df


def describe_values(df: pd.DataFrame, cols: list[str]) -> dict[str, dict[str, float]]:
    out: dict[str, dict[str, float]] = {}
    for col in cols:
        values = df[col].to_numpy(dtype=float)
        out[col] = {
            "mean": float(np.mean(values)),
            "median": float(np.median(values)),
            "std": float(np.std(values)),
            "p01": float(np.percentile(values, 1)),
            "p99": float(np.percentile(values, 99)),
            "min": float(np.min(values)),
            "max": float(np.max(values)),
            "p2p_p01_p99": float(np.percentile(values, 99) - np.percentile(values, 1)),
        }
    return out


def contiguous_slices(df: pd.DataFrame, max_gap_ms: float = 10.0, min_duration_s: float = 8.0) -> list[tuple[int, int]]:
    t = df["t_ms"].to_numpy(dtype=float)
    breaks = np.where(np.diff(t) > max_gap_ms)[0] + 1
    starts = np.r_[0, breaks]
    ends = np.r_[breaks, len(df)]
    slices: list[tuple[int, int]] = []
    for start, end in zip(starts, ends):
        if end - start < 2:
            continue
        duration_s = (t[end - 1] - t[start]) / 1000.0
        if duration_s >= min_duration_s:
            slices.append((int(start), int(end)))
    return slices


def uniform_resample(seg: pd.DataFrame, cols: list[str], fs_hz: float = FS_HZ) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    t_s = (seg["t_ms"].to_numpy(dtype=float) - float(seg["t_ms"].iloc[0])) / 1000.0
    t_uniform = np.arange(0.0, t_s[-1], 1.0 / fs_hz)
    values: dict[str, np.ndarray] = {}
    for col in cols:
        values[col] = np.interp(t_uniform, t_s, seg[col].to_numpy(dtype=float))
    return t_uniform, values


def welch_psd(values: np.ndarray, fs_hz: float = FS_HZ) -> tuple[np.ndarray, np.ndarray]:
    nperseg = min(8192, max(1024, len(values) // 4))
    if nperseg >= len(values):
        nperseg = max(256, len(values) // 2)
    freq, psd = signal.welch(
        signal.detrend(values, type="constant"),
        fs=fs_hz,
        window="hann",
        nperseg=nperseg,
        noverlap=nperseg // 2,
        scaling="density",
    )
    return freq, psd


def band_rms_from_psd(freq: np.ndarray, psd: np.ndarray, lo_hz: float, hi_hz: float) -> float:
    mask = (freq >= lo_hz) & (freq < hi_hz)
    if np.count_nonzero(mask) < 2:
        return 0.0
    return float(np.sqrt(np.trapezoid(psd[mask], freq[mask])))


def top_peaks(freq: np.ndarray, psd: np.ndarray, lo_hz: float, hi_hz: float, count: int = 8) -> list[dict[str, float]]:
    mask = (freq >= lo_hz) & (freq <= hi_hz)
    f = freq[mask]
    p = psd[mask]
    if len(f) < 3:
        return []

    peak_idx, props = signal.find_peaks(p, prominence=np.max(p) * 0.01 if np.max(p) > 0 else 0.0)
    if len(peak_idx) == 0:
        top = np.argsort(p)[-count:][::-1]
    else:
        order = np.argsort(p[peak_idx])[-count:][::-1]
        top = peak_idx[order]

    peaks = []
    total_power = float(np.trapezoid(p, f)) if len(f) > 1 else 0.0
    for idx in top[:count]:
        peaks.append(
            {
                "hz": float(f[idx]),
                "sqrt_psd": float(np.sqrt(p[idx])),
                "power_share_pct": float(100.0 * p[idx] / np.sum(p)) if np.sum(p) > 0 else 0.0,
                "band_rms": float(np.sqrt(total_power)) if total_power > 0 else 0.0,
            }
        )
    return peaks


def biquad_lpf(fs: float, fc: float) -> tuple[np.ndarray, np.ndarray]:
    w0 = 2.0 * np.pi * fc / fs
    sw0 = np.sin(w0)
    cw0 = np.cos(w0)
    alpha = sw0 / (2.0 * 0.70710678)
    a0 = 1.0 + alpha
    b = np.array([(1.0 - cw0) * 0.5 / a0, (1.0 - cw0) / a0, (1.0 - cw0) * 0.5 / a0])
    a = np.array([1.0, (-2.0 * cw0) / a0, (1.0 - alpha) / a0])
    return b, a


def biquad_notch(fs: float, fc: float, q: float) -> tuple[np.ndarray, np.ndarray]:
    w0 = 2.0 * np.pi * fc / fs
    sw0 = np.sin(w0)
    cw0 = np.cos(w0)
    alpha = sw0 / (2.0 * q)
    a0 = 1.0 + alpha
    b = np.array([1.0 / a0, (-2.0 * cw0) / a0, 1.0 / a0])
    a = np.array([1.0, (-2.0 * cw0) / a0, (1.0 - alpha) / a0])
    return b, a


def filter_chain_response(freq_hz: np.ndarray, spec: dict[str, float | int]) -> np.ndarray:
    response = np.ones_like(freq_hz, dtype=complex)
    stages: list[tuple[np.ndarray, np.ndarray]] = []

    if float(spec.get("gyro_aa_lpf_hz", 0.0)) > 0.0:
        stages.append(biquad_lpf(FS_HZ, float(spec["gyro_aa_lpf_hz"])))
    if float(spec.get("notch0_hz", 0.0)) > 0.0:
        stages.append(biquad_notch(FS_HZ, float(spec["notch0_hz"]), float(spec["notch0_q"])))
    if int(spec.get("notch1_enable", 0)) != 0 and float(spec.get("notch1_hz", 0.0)) > 0.0:
        stages.append(biquad_notch(FS_HZ, float(spec["notch1_hz"]), float(spec["notch1_q"])))
    if float(spec.get("gyro_lpf_hz", 0.0)) > 0.0:
        stages.append(biquad_lpf(FS_HZ, float(spec["gyro_lpf_hz"])))

    w = 2.0 * np.pi * freq_hz / FS_HZ
    for b, a in stages:
        _, h = signal.freqz(b, a, worN=w)
        response *= h
    return np.abs(response)


def filter_band_rms(freq: np.ndarray, psd: np.ndarray, spec: dict[str, float | int], lo_hz: float, hi_hz: float) -> float:
    gain = filter_chain_response(freq, spec)
    return band_rms_from_psd(freq, psd * gain * gain, lo_hz, hi_hz)


def rolling_quiet_windows(df: pd.DataFrame, window_s: float = 2.0) -> pd.DataFrame:
    samples = max(50, int(window_s * 1000))
    # The dataframe has missing log rows, so use a row window as a conservative proxy and rank windows by several quiet metrics.
    work = df.copy()
    gyro_norm = np.sqrt(np.sum(np.square(work[GYRO_COLS].to_numpy(dtype=float)), axis=1))
    gyro_target_norm = np.sqrt(np.sum(np.square(work[GYRO_TARGET_COLS].to_numpy(dtype=float)), axis=1))
    flow_norm = np.sqrt(np.sum(np.square(work[FLOW_COLS].to_numpy(dtype=float)), axis=1))
    angle_err_roll = work["roll_angle_target"].to_numpy(dtype=float) - work["roll"].to_numpy(dtype=float)
    angle_err_pitch = work["pitch_angle_target"].to_numpy(dtype=float) - work["pitch"].to_numpy(dtype=float)

    work["gyro_norm_abs"] = np.abs(gyro_norm)
    work["gyro_target_norm_abs"] = np.abs(gyro_target_norm)
    work["flow_norm_abs"] = np.abs(flow_norm)
    work["roll_angle_err"] = angle_err_roll
    work["pitch_angle_err"] = angle_err_pitch

    roll = work[
        [
            "t_ms",
            "gyro_norm_abs",
            "gyro_target_norm_abs",
            "flow_norm_abs",
            "roll",
            "pitch",
            "roll_angle_target",
            "pitch_angle_target",
            "roll_angle_err",
            "pitch_angle_err",
        ]
    ].rolling(samples, min_periods=samples)

    out = pd.DataFrame(
        {
            "t_end_ms": work["t_ms"],
            "gyro_norm_median": roll["gyro_norm_abs"].median(),
            "gyro_target_norm_median": roll["gyro_target_norm_abs"].median(),
            "flow_norm_median": roll["flow_norm_abs"].median(),
            "roll_mean": roll["roll"].mean(),
            "pitch_mean": roll["pitch"].mean(),
            "roll_target_mean": roll["roll_angle_target"].mean(),
            "pitch_target_mean": roll["pitch_angle_target"].mean(),
            "roll_err_mean": roll["roll_angle_err"].mean(),
            "pitch_err_mean": roll["pitch_angle_err"].mean(),
            "roll_std": roll["roll"].std(),
            "pitch_std": roll["pitch"].std(),
        }
    )
    out["quiet_score"] = (
        out["gyro_norm_median"]
        + 0.08 * out["gyro_target_norm_median"]
        + 0.02 * out["flow_norm_median"]
        + 2.0 * out["roll_std"].fillna(9999.0)
        + 2.0 * out["pitch_std"].fillna(9999.0)
    )
    return out.dropna().sort_values("quiet_score")


def control_metrics(df: pd.DataFrame) -> dict[str, dict[str, float]]:
    metrics: dict[str, dict[str, float]] = {}
    pairs = [
        ("roll", "roll_angle_target", "roll"),
        ("pitch", "pitch_angle_target", "pitch"),
        ("yaw", "yaw_angle_target", "yaw"),
        ("gyro_roll", "roll_gyro_target", "gyro_x"),
        ("gyro_pitch", "pitch_gyro_target", "gyro_y"),
        ("gyro_yaw", "yaw_gyro_target", "gyro_z"),
    ]
    for name, target, meas in pairs:
        err = df[target].to_numpy(dtype=float) - df[meas].to_numpy(dtype=float)
        active = np.abs(df[target].to_numpy(dtype=float)) > (0.5 if name.startswith("gyro") else 0.2)
        use = err[active] if np.count_nonzero(active) > 100 else err
        metrics[name] = {
            "mean_err": float(np.mean(use)),
            "mae": float(np.mean(np.abs(use))),
            "rmse": float(np.sqrt(np.mean(np.square(use)))),
            "p95_abs": float(np.percentile(np.abs(use), 95)),
            "target_std": float(np.std(df[target].to_numpy(dtype=float))),
            "meas_std": float(np.std(df[meas].to_numpy(dtype=float))),
        }
    return metrics


def dominant_lag_ms(df: pd.DataFrame, target: str, meas: str, max_lag_ms: int = 500) -> dict[str, float]:
    x = df[target].to_numpy(dtype=float)
    y = df[meas].to_numpy(dtype=float)
    x = signal.detrend(x)
    y = signal.detrend(y)
    if np.std(x) < 1e-6 or np.std(y) < 1e-6:
        return {"lag_ms": 0.0, "corr": 0.0}

    max_lag = min(max_lag_ms, len(x) // 4)
    lags = np.arange(-max_lag, max_lag + 1)
    cors = []
    for lag in lags:
        if lag < 0:
            a = x[-lag:]
            b = y[: len(a)]
        elif lag > 0:
            a = x[:-lag]
            b = y[lag:]
        else:
            a = x
            b = y
        if len(a) < 50:
            cors.append(0.0)
        else:
            cors.append(float(np.corrcoef(a, b)[0, 1]))
    best = int(np.nanargmax(np.abs(cors)))
    return {"lag_ms": float(lags[best]), "corr": float(cors[best])}


def make_plots(out_dir: Path, psd_results: dict[str, dict[str, np.ndarray]], quiet: pd.DataFrame, df: pd.DataFrame) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=(12, 8))
    for col in GYRO_COLS:
        freq = psd_results[col]["freq"]
        psd = psd_results[col]["psd"]
        plt.semilogy(freq, np.sqrt(psd), label=col)
    plt.xlim(0, 500)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("sqrt(PSD), dps/sqrt(Hz)")
    plt.title("Gyro frequency content")
    plt.grid(True, which="both", alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "gyro_psd.png", dpi=160)
    plt.close()

    plt.figure(figsize=(12, 8))
    for col in ACC_COLS:
        freq = psd_results[col]["freq"]
        psd = psd_results[col]["psd"]
        plt.semilogy(freq, np.sqrt(psd), label=col)
    plt.xlim(0, 500)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("sqrt(PSD), g/sqrt(Hz)")
    plt.title("Accel frequency content")
    plt.grid(True, which="both", alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "accel_psd.png", dpi=160)
    plt.close()

    best = quiet.head(1)
    if len(best) == 1:
        t_end = float(best["t_end_ms"].iloc[0])
        mask = (df["t_ms"] >= t_end - 5000.0) & (df["t_ms"] <= t_end + 1000.0)
        seg = df[mask]
        plt.figure(figsize=(12, 8))
        plt.plot((seg["t_ms"] - seg["t_ms"].iloc[0]) / 1000.0, seg["roll"], label="roll")
        plt.plot((seg["t_ms"] - seg["t_ms"].iloc[0]) / 1000.0, seg["pitch"], label="pitch")
        plt.plot((seg["t_ms"] - seg["t_ms"].iloc[0]) / 1000.0, seg["roll_angle_target"], "--", label="roll target")
        plt.plot((seg["t_ms"] - seg["t_ms"].iloc[0]) / 1000.0, seg["pitch_angle_target"], "--", label="pitch target")
        plt.xlabel("Time around quiet window (s)")
        plt.ylabel("deg")
        plt.title("Best quiet window attitude and targets")
        plt.grid(True, alpha=0.25)
        plt.legend()
        plt.tight_layout()
        plt.savefig(out_dir / "quiet_window_attitude.png", dpi=160)
        plt.close()


def markdown_table(rows: list[list[object]], headers: list[str]) -> str:
    table = ["| " + " | ".join(headers) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    for row in rows:
        table.append("| " + " | ".join(str(item) for item in row) + " |")
    return "\n".join(table)


def main() -> None:
    base = Path(__file__).resolve().parent
    df = load_log(base)
    out_dir = base / "analysis_imu_noise_0531"
    out_dir.mkdir(exist_ok=True)

    slices = contiguous_slices(df)
    longest_start, longest_end = max(slices, key=lambda s: df["t_ms"].iloc[s[1] - 1] - df["t_ms"].iloc[s[0]])
    spectral_seg = df.iloc[longest_start:longest_end].copy()

    _, resampled = uniform_resample(spectral_seg, GYRO_COLS + ACC_COLS)
    psd_results: dict[str, dict[str, np.ndarray]] = {}
    spectral_summary: dict[str, object] = {}
    for col, values in resampled.items():
        freq, psd = welch_psd(values)
        psd_results[col] = {"freq": freq, "psd": psd}
        spectral_summary[col] = {
            "band_rms": {
                "0_30_hz": band_rms_from_psd(freq, psd, 0.0, 30.0),
                "30_80_hz": band_rms_from_psd(freq, psd, 30.0, 80.0),
                "80_250_hz": band_rms_from_psd(freq, psd, 80.0, 250.0),
                "250_500_hz": band_rms_from_psd(freq, psd, 250.0, 500.0),
            },
            "peaks_20_500_hz": top_peaks(freq, psd, 20.0, 500.0, count=8),
        }

    old_filter = {
        "gyro_aa_lpf_hz": 250.0,
        "notch0_hz": 160.0,
        "notch0_q": 160.0 / 65.0,
        "notch1_enable": 0,
        "notch1_hz": 450.0,
        "notch1_q": 450.0 / 70.0,
        "gyro_lpf_hz": 60.0,
    }
    new_filter = {
        "gyro_aa_lpf_hz": 250.0,
        "notch0_hz": 185.0,
        "notch0_q": 185.0 / 70.0,
        "notch1_enable": 0,
        "notch1_hz": 450.0,
        "notch1_q": 450.0 / 70.0,
        "gyro_lpf_hz": 50.0,
    }
    filter_summary: dict[str, object] = {}
    for col in GYRO_COLS:
        freq = psd_results[col]["freq"]
        psd = psd_results[col]["psd"]
        filter_summary[col] = {
            "raw_80_250_rms": band_rms_from_psd(freq, psd, 80.0, 250.0),
            "old_80_250_rms": filter_band_rms(freq, psd, old_filter, 80.0, 250.0),
            "new_80_250_rms": filter_band_rms(freq, psd, new_filter, 80.0, 250.0),
            "raw_250_500_rms": band_rms_from_psd(freq, psd, 250.0, 500.0),
            "old_250_500_rms": filter_band_rms(freq, psd, old_filter, 250.0, 500.0),
            "new_250_500_rms": filter_band_rms(freq, psd, new_filter, 250.0, 500.0),
            "signal_0_30_rms": band_rms_from_psd(freq, psd, 0.0, 30.0),
            "old_0_30_rms": filter_band_rms(freq, psd, old_filter, 0.0, 30.0),
            "new_0_30_rms": filter_band_rms(freq, psd, new_filter, 0.0, 30.0),
        }

    quiet = rolling_quiet_windows(df)
    quiet.head(20).to_csv(out_dir / "quiet_windows.csv", index=False)

    metrics = {
        "source_file": df.attrs["source_file"],
        "rows": int(len(df)),
        "timestamp_start_ms": float(df["t_ms"].iloc[0]),
        "timestamp_end_ms": float(df["t_ms"].iloc[-1]),
        "timestamp_span_s": float((df["t_ms"].iloc[-1] - df["t_ms"].iloc[0]) / 1000.0),
        "diff_counts": {str(k): int(v) for k, v in df["diff_ms"].value_counts().sort_index().items()},
        "continuous_slices": [
            {
                "start_row": int(start),
                "end_row": int(end),
                "duration_s": float((df["t_ms"].iloc[end - 1] - df["t_ms"].iloc[start]) / 1000.0),
            }
            for start, end in slices
        ],
        "value_stats": describe_values(df, GYRO_COLS + ACC_COLS + EULER_COLS + GYRO_TARGET_COLS + ANGLE_TARGET_COLS + FLOW_COLS),
        "spectral_window": {
            "start_row": int(longest_start),
            "end_row": int(longest_end),
            "start_ms": float(df["t_ms"].iloc[longest_start]),
            "end_ms": float(df["t_ms"].iloc[longest_end - 1]),
            "duration_s": float((df["t_ms"].iloc[longest_end - 1] - df["t_ms"].iloc[longest_start]) / 1000.0),
        },
        "spectral_summary": spectral_summary,
        "old_filter": old_filter,
        "new_filter": new_filter,
        "filter_summary": filter_summary,
        "control_metrics": control_metrics(df),
        "lag_metrics": {
            "roll_angle": dominant_lag_ms(df, "roll_angle_target", "roll", 400),
            "pitch_angle": dominant_lag_ms(df, "pitch_angle_target", "pitch", 400),
            "roll_gyro": dominant_lag_ms(df, "roll_gyro_target", "gyro_x", 300),
            "pitch_gyro": dominant_lag_ms(df, "pitch_gyro_target", "gyro_y", 300),
            "yaw_gyro": dominant_lag_ms(df, "yaw_gyro_target", "gyro_z", 300),
        },
        "quiet_windows_top": quiet.head(10).to_dict(orient="records"),
    }

    with (out_dir / "imu_noise_analysis.json").open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)

    make_plots(out_dir, psd_results, quiet, df)

    gyro_rows = []
    for axis in GYRO_COLS:
        bands = metrics["spectral_summary"][axis]["band_rms"]
        peaks = metrics["spectral_summary"][axis]["peaks_20_500_hz"][:4]
        gyro_rows.append(
            [
                axis,
                f"{bands['0_30_hz']:.3f}",
                f"{bands['30_80_hz']:.3f}",
                f"{bands['80_250_hz']:.3f}",
                f"{bands['250_500_hz']:.3f}",
                ", ".join(f"{p['hz']:.1f}" for p in peaks),
            ]
        )

    acc_rows = []
    for axis in ACC_COLS:
        bands = metrics["spectral_summary"][axis]["band_rms"]
        peaks = metrics["spectral_summary"][axis]["peaks_20_500_hz"][:4]
        acc_rows.append(
            [
                axis,
                f"{bands['0_30_hz']:.4f}",
                f"{bands['30_80_hz']:.4f}",
                f"{bands['80_250_hz']:.4f}",
                f"{bands['250_500_hz']:.4f}",
                ", ".join(f"{p['hz']:.1f}" for p in peaks),
            ]
        )

    filt_rows = []
    for axis in GYRO_COLS:
        row = metrics["filter_summary"][axis]
        filt_rows.append(
            [
                axis,
                f"{row['raw_80_250_rms']:.3f}",
                f"{row['old_80_250_rms']:.3f}",
                f"{row['new_80_250_rms']:.3f}",
                f"{row['raw_250_500_rms']:.3f}",
                f"{row['old_250_500_rms']:.3f}",
                f"{row['new_250_500_rms']:.3f}",
            ]
        )

    quiet_rows = []
    for row in metrics["quiet_windows_top"][:5]:
        quiet_rows.append(
            [
                f"{row['t_end_ms'] / 1000.0:.2f}",
                f"{row['quiet_score']:.2f}",
                f"{row['roll_mean']:.2f}",
                f"{row['pitch_mean']:.2f}",
                f"{row['roll_target_mean']:.2f}",
                f"{row['pitch_target_mean']:.2f}",
                f"{row['flow_norm_median']:.1f}",
            ]
        )

    md = []
    md.append("# IMU noise analysis 2026-05-31")
    md.append("")
    md.append(f"Source: `{metrics['source_file']}`")
    md.append(f"Rows: {metrics['rows']}, timestamp span: {metrics['timestamp_span_s']:.1f}s")
    md.append(f"Diff counts: `{metrics['diff_counts']}`")
    md.append("")
    md.append("## Gyro PSD band RMS")
    md.append(markdown_table(gyro_rows, ["axis", "0-30Hz dps", "30-80Hz dps", "80-250Hz dps", "250-500Hz dps", "top peaks Hz"]))
    md.append("")
    md.append("## Accel PSD band RMS")
    md.append(markdown_table(acc_rows, ["axis", "0-30Hz g", "30-80Hz g", "80-250Hz g", "250-500Hz g", "top peaks Hz"]))
    md.append("")
    md.append("## Gyro filter simulation")
    md.append(
        "Old filter is AA LPF 250Hz + notch 160Hz/BW65 + gyro LPF 60Hz. "
        "New filter is AA LPF 250Hz + notch 185Hz/BW70 + gyro LPF 50Hz."
    )
    md.append(markdown_table(filt_rows, ["axis", "raw 80-250", "old 80-250", "new 80-250", "raw 250-500", "old 250-500", "new 250-500"]))
    md.append("")
    md.append("## Quiet windows for temporary mechanical trim")
    md.append(markdown_table(quiet_rows, ["t_end_s", "score", "roll mean", "pitch mean", "roll target", "pitch target", "flow median"]))
    md.append("")
    md.append("## Control metrics")
    control_rows = []
    for name, row in metrics["control_metrics"].items():
        control_rows.append([name, f"{row['mean_err']:.2f}", f"{row['mae']:.2f}", f"{row['rmse']:.2f}", f"{row['p95_abs']:.2f}"])
    md.append(markdown_table(control_rows, ["loop", "mean err", "MAE", "RMSE", "p95 abs"]))
    md.append("")
    md.append("## Lag metrics")
    lag_rows = []
    for name, row in metrics["lag_metrics"].items():
        lag_rows.append([name, f"{row['lag_ms']:.0f}", f"{row['corr']:.3f}"])
    md.append(markdown_table(lag_rows, ["pair", "best lag ms", "corr"]))
    md.append("")
    md.append("Plots: `gyro_psd.png`, `accel_psd.png`, `quiet_window_attitude.png`")
    md.append("")
    (out_dir / "imu_noise_analysis.md").write_text("\n".join(md), encoding="utf-8")

    print(out_dir)


if __name__ == "__main__":
    main()
