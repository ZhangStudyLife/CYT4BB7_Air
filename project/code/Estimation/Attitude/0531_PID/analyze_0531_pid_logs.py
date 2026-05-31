from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


BASE_DIR = Path(__file__).resolve().parent
OUT_DIR = BASE_DIR / "analysis_0531_pid_12"

TRIM = {
    "roll": -1.8,
    "pitch": 3.5,
}

FLIGHT_ORDER = {
    "\u4e00": 1,
    "\u4e8c": 2,
    "\u4e09": 3,
    "\u56db": 4,
    "\u4e94": 5,
    "\u516d": 6,
    "\u4e03": 7,
    "\u516b": 8,
    "\u4e5d": 9,
    "\u5341": 10,
}

FLIGHT_NAME_ORDER = {
    "\u7b2c\u5341\u4e00\u6b21": 11,
    "\u7b2c\u5341\u4e8c\u6b21": 12,
    "\u7b2c\u5341\u6b21": 10,
}

PARAMS = {
    1: {
        "name": "first_baseline_ff012_no_roll_i_d",
        "roll_angle_kp": 6.5, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.12,
        "pitch_angle_kp": 6.4, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.12,
        "roll_gyro_kp": 3.8, "roll_gyro_ki": 0.0, "roll_gyro_kd": 0.0, "roll_gyro_kff": 0.0,
        "pitch_gyro_kp": 4.6, "pitch_gyro_ki": 0.08, "pitch_gyro_kd": 0.0, "pitch_gyro_kff": 0.0,
    },
    2: {
        "name": "second_angle_ff_off",
        "roll_angle_kp": 6.5, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.0,
        "pitch_angle_kp": 6.4, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.0,
        "roll_gyro_kp": 3.8, "roll_gyro_ki": 0.0, "roll_gyro_kd": 0.0, "roll_gyro_kff": 0.0,
        "pitch_gyro_kp": 4.6, "pitch_gyro_ki": 0.08, "pitch_gyro_kd": 0.0, "pitch_gyro_kff": 0.0,
    },
    3: {
        "name": "third_higher_angle_p_low_ff_inner_i",
        "roll_angle_kp": 7.1, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.08,
        "pitch_angle_kp": 7.0, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.08,
        "roll_gyro_kp": 4.1, "roll_gyro_ki": 0.05, "roll_gyro_kd": 0.0, "roll_gyro_kff": 0.0,
        "pitch_gyro_kp": 4.9, "pitch_gyro_ki": 0.10, "pitch_gyro_kd": 0.0, "pitch_gyro_kff": 0.0,
    },
    4: {
        "name": "fourth_small_gyro_d",
        "roll_angle_kp": 6.8, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.06,
        "pitch_angle_kp": 6.7, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.06,
        "roll_gyro_kp": 4.0, "roll_gyro_ki": 0.06, "roll_gyro_kd": 0.006, "roll_gyro_kff": 0.0,
        "pitch_gyro_kp": 4.8, "pitch_gyro_ki": 0.11, "pitch_gyro_kd": 0.007, "pitch_gyro_kff": 0.0,
    },
    5: {
        "name": "fifth_strong_self_level",
        "roll_angle_kp": 7.6, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.10,
        "pitch_angle_kp": 7.4, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.10,
        "roll_gyro_kp": 4.2, "roll_gyro_ki": 0.06, "roll_gyro_kd": 0.006, "roll_gyro_kff": 0.0,
        "pitch_gyro_kp": 5.0, "pitch_gyro_ki": 0.12, "pitch_gyro_kd": 0.008, "pitch_gyro_kff": 0.0,
    },
    6: {
        "name": "sixth_heavy_smooth",
        "roll_angle_kp": 6.2, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.04,
        "pitch_angle_kp": 6.1, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.04,
        "roll_gyro_kp": 4.3, "roll_gyro_ki": 0.08, "roll_gyro_kd": 0.008, "roll_gyro_kff": 0.0,
        "pitch_gyro_kp": 5.1, "pitch_gyro_ki": 0.14, "pitch_gyro_kd": 0.010, "pitch_gyro_kff": 0.0,
    },
    7: {
        "name": "seventh_balanced_candidate",
        "roll_angle_kp": 6.5, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.05,
        "pitch_angle_kp": 6.7, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.05,
        "roll_gyro_kp": 4.2, "roll_gyro_ki": 0.07, "roll_gyro_kd": 0.007, "roll_gyro_kff": 0.0,
        "pitch_gyro_kp": 4.9, "pitch_gyro_ki": 0.12, "pitch_gyro_kd": 0.008, "pitch_gyro_kff": 0.0,
    },
    8: {
        "name": "eighth_more_i_less_ff",
        "roll_angle_kp": 6.4, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.04,
        "pitch_angle_kp": 6.6, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.04,
        "roll_gyro_kp": 4.2, "roll_gyro_ki": 0.10, "roll_gyro_kd": 0.007, "roll_gyro_kff": 0.0,
        "pitch_gyro_kp": 4.9, "pitch_gyro_ki": 0.15, "pitch_gyro_kd": 0.008, "pitch_gyro_kff": 0.0,
    },
    9: {
        "name": "ninth_soft_undertuned",
        "roll_angle_kp": 5.0, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.0,
        "pitch_angle_kp": 5.2, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.0,
        "roll_gyro_kp": 3.2, "roll_gyro_ki": 0.04, "roll_gyro_kd": 0.003, "roll_gyro_kff": 0.0,
        "pitch_gyro_kp": 3.8, "pitch_gyro_ki": 0.06, "pitch_gyro_kd": 0.004, "pitch_gyro_kff": 0.0,
    },
    10: {
        "name": "tenth_strong_angle_ff",
        "roll_angle_kp": 8.2, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.14,
        "pitch_angle_kp": 8.0, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.14,
        "roll_gyro_kp": 4.2, "roll_gyro_ki": 0.08, "roll_gyro_kd": 0.006, "roll_gyro_kff": 0.0,
        "pitch_gyro_kp": 4.9, "pitch_gyro_ki": 0.12, "pitch_gyro_kd": 0.007, "pitch_gyro_kff": 0.0,
    },
    11: {
        "name": "eleventh_strong_inner_low_ff",
        "roll_angle_kp": 6.0, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.02,
        "pitch_angle_kp": 6.2, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.02,
        "roll_gyro_kp": 5.4, "roll_gyro_ki": 0.18, "roll_gyro_kd": 0.010, "roll_gyro_kff": 0.0,
        "pitch_gyro_kp": 6.0, "pitch_gyro_ki": 0.24, "pitch_gyro_kd": 0.012, "pitch_gyro_kff": 0.0,
    },
    12: {
        "name": "twelfth_strong_d_damping",
        "roll_angle_kp": 5.8, "roll_angle_ki": 0.0, "roll_angle_kd": 0.0, "roll_angle_kff": 0.03,
        "pitch_angle_kp": 6.0, "pitch_angle_ki": 0.0, "pitch_angle_kd": 0.0, "pitch_angle_kff": 0.03,
        "roll_gyro_kp": 4.6, "roll_gyro_ki": 0.10, "roll_gyro_kd": 0.014, "roll_gyro_kff": 0.0,
        "pitch_gyro_kp": 5.3, "pitch_gyro_ki": 0.15, "pitch_gyro_kd": 0.016, "pitch_gyro_kff": 0.0,
    },
}

COLUMNS = {
    "t_ms": "I0",
    "roll": "I13",
    "pitch": "I14",
    "roll_angle_target": "I15",
    "roll_angle_measured": "I16",
    "roll_angle_out": "I17",
    "roll_angle_p": "I18",
    "roll_angle_i": "I19",
    "roll_angle_d": "I20",
    "roll_angle_error": "I21",
    "pitch_angle_target": "I22",
    "pitch_angle_measured": "I23",
    "pitch_angle_out": "I24",
    "pitch_angle_p": "I25",
    "pitch_angle_i": "I26",
    "pitch_angle_d": "I27",
    "pitch_angle_error": "I28",
    "roll_gyro_target": "I29",
    "roll_gyro_measured": "I30",
    "roll_gyro_out": "I31",
    "roll_gyro_p": "I32",
    "roll_gyro_i": "I33",
    "roll_gyro_d": "I34",
    "pitch_gyro_target": "I35",
    "pitch_gyro_measured": "I36",
    "pitch_gyro_out": "I37",
    "pitch_gyro_p": "I38",
    "pitch_gyro_i": "I39",
    "pitch_gyro_d": "I40",
}


@dataclass(frozen=True)
class AxisCols:
    angle_target: str
    angle_measured: str
    angle_out: str
    angle_p: str
    angle_i: str
    angle_d: str
    angle_error: str
    gyro_target: str
    gyro_measured: str
    gyro_out: str
    gyro_p: str
    gyro_i: str
    gyro_d: str


AXES = {
    "roll": AxisCols(
        "roll_angle_target", "roll_angle_measured", "roll_angle_out",
        "roll_angle_p", "roll_angle_i", "roll_angle_d", "roll_angle_error",
        "roll_gyro_target", "roll_gyro_measured", "roll_gyro_out",
        "roll_gyro_p", "roll_gyro_i", "roll_gyro_d",
    ),
    "pitch": AxisCols(
        "pitch_angle_target", "pitch_angle_measured", "pitch_angle_out",
        "pitch_angle_p", "pitch_angle_i", "pitch_angle_d", "pitch_angle_error",
        "pitch_gyro_target", "pitch_gyro_measured", "pitch_gyro_out",
        "pitch_gyro_p", "pitch_gyro_i", "pitch_gyro_d",
    ),
}


def flight_no_from_name(path: Path) -> int:
    for prefix, flight in FLIGHT_NAME_ORDER.items():
        if path.name.startswith(prefix):
            return flight
    if len(path.name) < 2 or path.name[1] not in FLIGHT_ORDER:
        raise ValueError(f"Cannot infer flight number from {path.name}")
    return FLIGHT_ORDER[path.name[1]]


def rms(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return math.nan
    return float(np.sqrt(np.mean(values * values)))


def pctl(values: np.ndarray, q: float) -> float:
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return math.nan
    return float(np.percentile(values, q))


def markdown_table(df: pd.DataFrame) -> str:
    if df.empty:
        return "_No rows._"
    text_df = df.copy()
    for col in text_df.columns:
        text_df[col] = text_df[col].map(lambda x: "" if pd.isna(x) else str(x))
    headers = list(text_df.columns)
    rows = text_df.values.tolist()
    widths = [
        max(len(str(header)), *(len(str(row[i])) for row in rows))
        for i, header in enumerate(headers)
    ]
    lines = [
        "| " + " | ".join(str(header).ljust(widths[i]) for i, header in enumerate(headers)) + " |",
        "| " + " | ".join("-" * widths[i] for i in range(len(headers))) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(row[i]).ljust(widths[i]) for i in range(len(headers))) + " |")
    return "\n".join(lines)


def pct(mask: np.ndarray) -> float:
    mask = np.asarray(mask)
    if mask.size == 0:
        return math.nan
    return float(100.0 * np.mean(mask))


def derivative(t_s: np.ndarray, x: np.ndarray) -> np.ndarray:
    t_s = np.asarray(t_s, dtype=float)
    x = np.asarray(x, dtype=float)
    out = np.zeros_like(x)
    if len(x) < 2:
        return out
    dt = np.diff(t_s)
    dx = np.diff(x)
    safe = np.where(dt > 1.0e-6, dt, np.nan)
    first = dx / safe
    first = np.nan_to_num(first, nan=0.0, posinf=0.0, neginf=0.0)
    out[1:] = first
    out[0] = out[1]
    return out


def rolling_abs_rate(rate: np.ndarray, samples: int = 21) -> np.ndarray:
    return (
        pd.Series(np.abs(rate))
        .rolling(samples, center=True, min_periods=1)
        .median()
        .to_numpy(dtype=float)
    )


def dominant_lag_ms(t_ms: np.ndarray, target: np.ndarray, measured: np.ndarray, max_lag_ms: int) -> tuple[float, float]:
    target = np.asarray(target, dtype=float)
    measured = np.asarray(measured, dtype=float)
    if target.size < 20 or measured.size < 20:
        return math.nan, math.nan

    x = target - np.nanmean(target)
    y = measured - np.nanmean(measured)
    x_std = np.nanstd(x)
    y_std = np.nanstd(y)
    if x_std < 1.0e-6 or y_std < 1.0e-6:
        return math.nan, math.nan

    x = x / x_std
    y = y / y_std
    best_lag = 0
    best_corr = -2.0
    for lag in range(-max_lag_ms, max_lag_ms + 1):
        if lag < 0:
            xs = x[-lag:]
            ys = y[:lag]
        elif lag > 0:
            xs = x[:-lag]
            ys = y[lag:]
        else:
            xs = x
            ys = y
        if xs.size < 20:
            continue
        corr = float(np.mean(xs * ys))
        if corr > best_corr:
            best_corr = corr
            best_lag = lag
    return float(best_lag), best_corr


def contiguous_windows(mask: np.ndarray, t_ms: np.ndarray, min_duration_ms: float) -> list[tuple[int, int]]:
    mask = np.asarray(mask, dtype=bool)
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    breaks = np.where(np.diff(idx) > 1)[0] + 1
    groups = np.split(idx, breaks)
    windows: list[tuple[int, int]] = []
    for g in groups:
        if g.size == 0:
            continue
        start = int(g[0])
        end = int(g[-1]) + 1
        if t_ms[end - 1] - t_ms[start] >= min_duration_ms:
            windows.append((start, end))
    return windows


def expanded_event_mask(t_ms: np.ndarray, signal: np.ndarray, threshold: float, before_ms: int, after_ms: int) -> np.ndarray:
    mask = np.zeros(len(signal), dtype=bool)
    event_idx = np.flatnonzero(np.abs(signal) >= threshold)
    for i in event_idx:
        start = int(np.searchsorted(t_ms, t_ms[i] - before_ms, side="left"))
        end = int(np.searchsorted(t_ms, t_ms[i] + after_ms, side="right"))
        mask[start:end] = True
    return mask


def longest_window(mask: np.ndarray, t_ms: np.ndarray, min_duration_ms: float) -> tuple[int, int] | None:
    windows = contiguous_windows(mask, t_ms, min_duration_ms)
    if not windows:
        return None
    return max(windows, key=lambda w: t_ms[w[1] - 1] - t_ms[w[0]])


def event_windows(signal: np.ndarray, t_ms: np.ndarray, threshold: float, before_ms: int, after_ms: int, limit: int = 4) -> list[tuple[int, int]]:
    idx = np.flatnonzero(np.abs(signal) >= threshold)
    if idx.size == 0:
        return []
    candidates = []
    used_until = -1
    for i in idx:
        if i <= used_until:
            continue
        local_end = min(len(signal), i + after_ms)
        local = np.arange(i, local_end)
        if local.size == 0:
            continue
        peak_i = int(local[np.argmax(np.abs(signal[local]))])
        start_t = t_ms[peak_i] - before_ms
        end_t = t_ms[peak_i] + after_ms
        start = int(np.searchsorted(t_ms, start_t, side="left"))
        end = int(np.searchsorted(t_ms, end_t, side="right"))
        candidates.append((start, end, float(np.max(np.abs(signal[start:end])))))
        used_until = int(np.searchsorted(t_ms, end_t, side="right"))
    candidates.sort(key=lambda x: x[2], reverse=True)
    return [(s, e) for s, e, _ in candidates[:limit]]


def add_derived(df: pd.DataFrame, flight: int) -> pd.DataFrame:
    out = df.rename(columns={v: k for k, v in COLUMNS.items()}).copy()
    out["active"] = (
        out[["roll_angle_out", "pitch_angle_out", "roll_gyro_out", "pitch_gyro_out"]]
        .abs()
        .sum(axis=1)
        > 1.0e-6
    )
    t0 = float(out.loc[out["active"], "t_ms"].iloc[0]) if out["active"].any() else float(out["t_ms"].iloc[0])
    out["t_active_s"] = (out["t_ms"] - t0) * 0.001
    out["t_s"] = (out["t_ms"] - float(out["t_ms"].iloc[0])) * 0.001
    out["analysis_mask"] = out["active"] & (out["t_ms"] >= t0 + 1000.0)

    for axis, cols in AXES.items():
        out[f"{axis}_angle_ff_residual"] = out[cols.angle_out] - out[cols.angle_p] - out[cols.angle_i] - out[cols.angle_d]
        out[f"{axis}_gyro_error"] = out[cols.gyro_target] - out[cols.gyro_measured]
        out[f"{axis}_gyro_ff_residual"] = out[cols.gyro_out] - out[cols.gyro_p] - out[cols.gyro_i] - out[cols.gyro_d]
        out[f"{axis}_angle_target_rate"] = derivative(out["t_s"].to_numpy(), out[cols.angle_target].to_numpy())
        out[f"{axis}_gyro_target_rate"] = derivative(out["t_s"].to_numpy(), out[cols.gyro_target].to_numpy())
        out[f"{axis}_gyro_measured_rate"] = derivative(out["t_s"].to_numpy(), out[cols.gyro_measured].to_numpy())
        out[f"{axis}_angle_target_rate_abs_med"] = rolling_abs_rate(out[f"{axis}_angle_target_rate"].to_numpy())
        out[f"{axis}_gyro_target_rate_abs_med"] = rolling_abs_rate(out[f"{axis}_gyro_target_rate"].to_numpy())

    out["flight"] = flight
    out["flight_name"] = PARAMS[flight]["name"]
    return out


def state_masks(df: pd.DataFrame, axis: str, cols: AxisCols) -> dict[str, np.ndarray]:
    mask = df["analysis_mask"].to_numpy(dtype=bool)
    target_offset = (df[cols.angle_target] - TRIM[axis]).to_numpy(dtype=float)
    measured_offset = (df[cols.angle_measured] - TRIM[axis]).to_numpy(dtype=float)
    target_rate_raw = np.abs(df[f"{axis}_angle_target_rate"].to_numpy(dtype=float))
    target_rate_abs = df[f"{axis}_angle_target_rate_abs_med"].to_numpy(dtype=float)
    gyro_target = df[cols.gyro_target].to_numpy(dtype=float)
    gyro_measured = df[cols.gyro_measured].to_numpy(dtype=float)
    t_ms = df["t_ms"].to_numpy(dtype=float)

    quiet = (
        mask
        & (np.abs(target_offset) <= 4.0)
        & (target_rate_abs <= 25.0)
        & (np.abs(gyro_target) <= 35.0)
        & (np.abs(gyro_measured) <= 35.0)
    )
    step = mask & expanded_event_mask(t_ms, target_rate_raw, 120.0, 250, 850)
    large_angle = mask & ((np.abs(target_offset) >= 12.0) | (np.abs(measured_offset) >= 12.0))
    stable = (
        mask
        & (target_rate_abs <= 20.0)
        & (np.abs(gyro_target) <= 35.0)
        & (np.abs(gyro_measured) <= 35.0)
    )
    return {
        "all_active": mask,
        "quiet_center": quiet,
        "rapid_step": step,
        "large_angle": large_angle,
        "stable_hold": stable,
    }


def summarize_axis(df: pd.DataFrame, flight: int, axis: str, state: str, mask: np.ndarray) -> dict[str, float | int | str]:
    cols = AXES[axis]
    sub = df.loc[mask]
    row: dict[str, float | int | str] = {
        "flight": flight,
        "flight_name": PARAMS[flight]["name"],
        "axis": axis,
        "state": state,
        "samples": int(len(sub)),
        "duration_s": float((sub["t_ms"].iloc[-1] - sub["t_ms"].iloc[0]) * 0.001) if len(sub) > 1 else 0.0,
    }
    if len(sub) < 20:
        return row

    err = sub[cols.angle_error].to_numpy(dtype=float)
    gyro_err = sub[f"{axis}_gyro_error"].to_numpy(dtype=float)
    angle_lag, angle_corr = dominant_lag_ms(
        sub["t_ms"].to_numpy(dtype=float),
        sub[cols.angle_target].to_numpy(dtype=float),
        sub[cols.angle_measured].to_numpy(dtype=float),
        400,
    )
    gyro_lag, gyro_corr = dominant_lag_ms(
        sub["t_ms"].to_numpy(dtype=float),
        sub[cols.gyro_target].to_numpy(dtype=float),
        sub[cols.gyro_measured].to_numpy(dtype=float),
        250,
    )
    valid_angle_err = np.abs(err) > 0.05
    valid_gyro_err = np.abs(gyro_err) > 0.5
    sp_rate = sub[f"{axis}_angle_target_rate"].to_numpy(dtype=float)
    valid_sp_rate = np.abs(sp_rate) > 20.0
    gyro_meas_accel = sub[f"{axis}_gyro_measured_rate"].to_numpy(dtype=float)
    valid_accel = np.abs(gyro_meas_accel) > 200.0

    row.update({
        "angle_err_mean_deg": float(np.mean(err)),
        "angle_err_rms_deg": rms(err),
        "angle_abs_err_p95_deg": pctl(np.abs(err), 95),
        "angle_abs_err_p99_deg": pctl(np.abs(err), 99),
        "angle_target_min_deg": float(sub[cols.angle_target].min()),
        "angle_target_max_deg": float(sub[cols.angle_target].max()),
        "angle_meas_min_deg": float(sub[cols.angle_measured].min()),
        "angle_meas_max_deg": float(sub[cols.angle_measured].max()),
        "angle_lag_ms": angle_lag,
        "angle_corr": angle_corr,
        "angle_output_rms_dps": rms(sub[cols.angle_out].to_numpy(dtype=float)),
        "angle_p_rms_dps": rms(sub[cols.angle_p].to_numpy(dtype=float)),
        "angle_i_rms_dps": rms(sub[cols.angle_i].to_numpy(dtype=float)),
        "angle_d_rms_dps": rms(sub[cols.angle_d].to_numpy(dtype=float)),
        "angle_ff_residual_rms_dps": rms(sub[f"{axis}_angle_ff_residual"].to_numpy(dtype=float)),
        "angle_output_help_pct": pct((sub[cols.angle_out].to_numpy(dtype=float)[valid_angle_err] * err[valid_angle_err]) > 0.0),
        "angle_p_help_pct": pct((sub[cols.angle_p].to_numpy(dtype=float)[valid_angle_err] * err[valid_angle_err]) > 0.0),
        "angle_ff_with_sp_rate_pct": pct((sub[f"{axis}_angle_ff_residual"].to_numpy(dtype=float)[valid_sp_rate] * sp_rate[valid_sp_rate]) > 0.0),
        "gyro_err_mean_dps": float(np.mean(gyro_err)),
        "gyro_err_rms_dps": rms(gyro_err),
        "gyro_abs_err_p95_dps": pctl(np.abs(gyro_err), 95),
        "gyro_abs_err_p99_dps": pctl(np.abs(gyro_err), 99),
        "gyro_lag_ms": gyro_lag,
        "gyro_corr": gyro_corr,
        "gyro_output_rms": rms(sub[cols.gyro_out].to_numpy(dtype=float)),
        "gyro_p_rms": rms(sub[cols.gyro_p].to_numpy(dtype=float)),
        "gyro_i_rms": rms(sub[cols.gyro_i].to_numpy(dtype=float)),
        "gyro_d_rms": rms(sub[cols.gyro_d].to_numpy(dtype=float)),
        "gyro_output_help_pct": pct((sub[cols.gyro_out].to_numpy(dtype=float)[valid_gyro_err] * gyro_err[valid_gyro_err]) > 0.0),
        "gyro_p_help_pct": pct((sub[cols.gyro_p].to_numpy(dtype=float)[valid_gyro_err] * gyro_err[valid_gyro_err]) > 0.0),
        "gyro_i_help_pct": pct((sub[cols.gyro_i].to_numpy(dtype=float)[valid_gyro_err] * gyro_err[valid_gyro_err]) > 0.0),
        "gyro_d_damping_pct": pct((sub[cols.gyro_d].to_numpy(dtype=float)[valid_accel] * gyro_meas_accel[valid_accel]) < 0.0),
        "gyro_d_opposes_p_pct": pct((sub[cols.gyro_d].to_numpy(dtype=float) * sub[cols.gyro_p].to_numpy(dtype=float)) < 0.0),
        "gyro_ff_residual_rms": rms(sub[f"{axis}_gyro_ff_residual"].to_numpy(dtype=float)),
    })
    return row


def summarize_windows(df: pd.DataFrame, flight: int, axis: str) -> list[dict[str, float | int | str]]:
    cols = AXES[axis]
    masks = state_masks(df, axis, cols)
    t_ms = df["t_ms"].to_numpy(dtype=float)
    rows: list[dict[str, float | int | str]] = []

    for state, min_ms in [("quiet_center", 800), ("stable_hold", 800), ("large_angle", 500)]:
        win = longest_window(masks[state], t_ms, min_ms)
        if win is None:
            continue
        s, e = win
        local = np.zeros(len(df), dtype=bool)
        local[s:e] = True
        local &= masks[state]
        row = summarize_axis(df, flight, axis, f"example_{state}", local)
        row.update({
            "start_active_s": float(df["t_active_s"].iloc[s]),
            "end_active_s": float(df["t_active_s"].iloc[e - 1]),
        })
        rows.append(row)

    step_signal = np.abs(df[f"{axis}_angle_target_rate"].to_numpy(dtype=float))
    for n, (s, e) in enumerate(event_windows(step_signal, t_ms, 120.0, 250, 850, limit=3), start=1):
        local = np.zeros(len(df), dtype=bool)
        local[s:e] = True
        local &= df["analysis_mask"].to_numpy(dtype=bool)
        if local.sum() < 50:
            continue
        row = summarize_axis(df, flight, axis, f"example_rapid_step_{n}", local)
        row.update({
            "start_active_s": float(df["t_active_s"].iloc[s]),
            "end_active_s": float(df["t_active_s"].iloc[e - 1]),
        })
        rows.append(row)
    return rows


def plot_summary(overall: pd.DataFrame) -> None:
    active = overall[overall["state"] == "all_active"].copy()
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    for axis_name, ax in zip(["roll", "pitch"], axes[0]):
        data = active[active["axis"] == axis_name]
        ax.plot(data["flight"], data["angle_err_rms_deg"], marker="o", label="angle error RMS")
        ax.plot(data["flight"], data["angle_abs_err_p95_deg"], marker="s", label="angle abs error P95")
        ax.set_title(f"{axis_name} angle tracking")
        ax.set_ylabel("deg")
        ax.grid(True, alpha=0.3)
        ax.legend()
    for axis_name, ax in zip(["roll", "pitch"], axes[1]):
        data = active[active["axis"] == axis_name]
        ax.plot(data["flight"], data["gyro_err_rms_dps"], marker="o", label="gyro error RMS")
        ax.plot(data["flight"], data["gyro_abs_err_p95_dps"], marker="s", label="gyro abs error P95")
        ax.set_title(f"{axis_name} gyro tracking")
        ax.set_xlabel("flight")
        ax.set_ylabel("deg/s")
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(OUT_DIR / "overall_tracking.png", dpi=160)
    plt.close(fig)


def plot_representative(df: pd.DataFrame, flight: int, axis: str, start_s: float, end_s: float, label: str) -> None:
    cols = AXES[axis]
    sub = df[(df["t_active_s"] >= start_s) & (df["t_active_s"] <= end_s)].copy()
    if len(sub) < 20:
        return
    t = sub["t_active_s"].to_numpy(dtype=float)
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axes[0].plot(t, sub[cols.angle_target], label="angle target", linewidth=1.2)
    axes[0].plot(t, sub[cols.angle_measured], label="angle measured", linewidth=1.2)
    axes[0].plot(t, sub[cols.angle_error], label="angle error", linewidth=0.9)
    axes[0].set_ylabel("deg")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, sub[cols.gyro_target], label="gyro target", linewidth=1.2)
    axes[1].plot(t, sub[cols.gyro_measured], label="gyro measured", linewidth=1.2)
    axes[1].plot(t, sub[f"{axis}_gyro_error"], label="gyro error", linewidth=0.9)
    axes[1].set_ylabel("deg/s")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, sub[cols.gyro_out], label="gyro output", linewidth=1.2)
    axes[2].plot(t, sub[cols.gyro_p], label="P", linewidth=0.9)
    axes[2].plot(t, sub[cols.gyro_i], label="I", linewidth=0.9)
    axes[2].plot(t, sub[cols.gyro_d], label="D", linewidth=0.9)
    axes[2].set_xlabel("active time (s)")
    axes[2].set_ylabel("mixer units")
    axes[2].legend(loc="upper right", ncol=4)
    axes[2].grid(True, alpha=0.3)

    fig.suptitle(f"flight {flight} {axis} {label}")
    fig.tight_layout()
    safe_label = label.replace("example_", "").replace("rapid_step_", "step")
    fig.savefig(OUT_DIR / f"flight{flight}_{axis}_{safe_label}.png", dpi=160)
    plt.close(fig)


def step_event_metrics(df: pd.DataFrame, flight: int, axis: str, limit: int = 30) -> list[dict[str, float | int | str]]:
    cols = AXES[axis]
    t = df["t_ms"].to_numpy(dtype=float)
    target = df[cols.angle_target].to_numpy(dtype=float)
    measured = df[cols.angle_measured].to_numpy(dtype=float)
    gyro_target = df[cols.gyro_target].to_numpy(dtype=float)
    gyro_measured = df[cols.gyro_measured].to_numpy(dtype=float)
    active = df["analysis_mask"].to_numpy(dtype=bool)
    d_target = np.diff(target, prepend=target[0])
    idx = np.flatnonzero(active & (np.abs(d_target) >= 0.5))
    rows: list[dict[str, float | int | str]] = []
    used_until = -1

    for i in idx:
        if i <= used_until:
            continue
        start = max(0, int(np.searchsorted(t, t[i] - 100.0, side="left")))
        end = min(len(df), int(np.searchsorted(t, t[i] + 800.0, side="right")))
        if end - start < 50:
            continue

        target_before = float(np.median(target[start:i])) if i > start else float(target[i - 1])
        target_after = float(np.median(target[min(end, i + 100):end])) if end - i > 120 else float(target[end - 1])
        delta = target_after - target_before
        if abs(delta) < 1.0:
            used_until = end
            continue

        direction = 1.0 if delta > 0.0 else -1.0
        response = direction * (measured[i:end] - target_before)
        final = abs(delta)
        reached_idx = np.flatnonzero(response >= 0.63 * final)
        t63_ms = float(t[i + reached_idx[0]] - t[i]) if reached_idx.size else math.nan
        overshoot = max(0.0, float(np.nanmax(response) - final))

        post_error = target[i:end] - measured[i:end]
        gyro_err = gyro_target[i:end] - gyro_measured[i:end]
        sp_rate = df[f"{axis}_angle_target_rate"].to_numpy(dtype=float)[i:end]
        ff = df[f"{axis}_angle_ff_residual"].to_numpy(dtype=float)[i:end]
        p_term = df[cols.gyro_p].to_numpy(dtype=float)[i:end]
        i_term = df[cols.gyro_i].to_numpy(dtype=float)[i:end]
        d_term = df[cols.gyro_d].to_numpy(dtype=float)[i:end]
        gyro_out = df[cols.gyro_out].to_numpy(dtype=float)[i:end]
        gyro_acc = df[f"{axis}_gyro_measured_rate"].to_numpy(dtype=float)[i:end]
        valid_sp = np.abs(sp_rate) > 20.0
        valid_gyro_err = np.abs(gyro_err) > 0.5
        valid_acc = np.abs(gyro_acc) > 200.0

        rows.append({
            "flight": flight,
            "axis": axis,
            "event_time_active_s": float(df["t_active_s"].iloc[i]),
            "delta_deg": float(delta),
            "pre_error_deg": float(target_before - measured[i - 1]),
            "post_angle_err_rms_deg": rms(post_error),
            "post_angle_abs_err_p95_deg": pctl(np.abs(post_error), 95),
            "t63_ms": t63_ms,
            "overshoot_deg": overshoot,
            "overshoot_pct_of_step": float(100.0 * overshoot / final) if final > 1.0e-6 else math.nan,
            "gyro_err_rms_dps": rms(gyro_err),
            "gyro_abs_err_p95_dps": pctl(np.abs(gyro_err), 95),
            "angle_ff_with_sp_rate_pct": pct((ff[valid_sp] * sp_rate[valid_sp]) > 0.0),
            "gyro_output_help_pct": pct((gyro_out[valid_gyro_err] * gyro_err[valid_gyro_err]) > 0.0),
            "gyro_p_help_pct": pct((p_term[valid_gyro_err] * gyro_err[valid_gyro_err]) > 0.0),
            "gyro_i_help_pct": pct((i_term[valid_gyro_err] * gyro_err[valid_gyro_err]) > 0.0),
            "gyro_d_damping_pct": pct((d_term[valid_acc] * gyro_acc[valid_acc]) < 0.0),
            "gyro_d_opposes_p_pct": pct((d_term * p_term) < 0.0),
            "gyro_p_rms": rms(p_term),
            "gyro_i_rms": rms(i_term),
            "gyro_d_rms": rms(d_term),
        })
        used_until = end
        if len(rows) >= limit:
            break
    return rows


def main() -> None:
    OUT_DIR.mkdir(exist_ok=True)
    csv_files = sorted(BASE_DIR.glob("*.csv"), key=flight_no_from_name)

    overall_rows = []
    window_rows = []
    step_rows = []
    loaded: dict[int, pd.DataFrame] = {}
    for path in csv_files:
        flight = flight_no_from_name(path)
        df = pd.read_csv(path)
        df = add_derived(df, flight)
        loaded[flight] = df
        for axis, cols in AXES.items():
            for state, mask in state_masks(df, axis, cols).items():
                overall_rows.append(summarize_axis(df, flight, axis, state, mask))
            window_rows.extend(summarize_windows(df, flight, axis))
            step_rows.extend(step_event_metrics(df, flight, axis))

    overall = pd.DataFrame(overall_rows)
    windows = pd.DataFrame(window_rows)
    step_events = pd.DataFrame(step_rows)
    overall.to_csv(OUT_DIR / "summary_by_state.csv", index=False, encoding="utf-8-sig")
    windows.to_csv(OUT_DIR / "representative_windows.csv", index=False, encoding="utf-8-sig")
    step_events.to_csv(OUT_DIR / "step_events.csv", index=False, encoding="utf-8-sig")
    with (OUT_DIR / "params.json").open("w", encoding="utf-8") as f:
        json.dump(PARAMS, f, indent=2, ensure_ascii=False)

    plot_summary(overall)

    # Keep figure count bounded: plot the strongest representative step plus stable window for key flights.
    for flight in [2, 4, 5, 6]:
        for axis in ["roll", "pitch"]:
            sub = windows[(windows["flight"] == flight) & (windows["axis"] == axis)]
            for prefix in ["example_rapid_step", "example_stable_hold"]:
                candidates = sub[sub["state"].astype(str).str.startswith(prefix)]
                if candidates.empty:
                    continue
                row = candidates.iloc[0]
                plot_representative(
                    loaded[flight],
                    flight,
                    axis,
                    float(row["start_active_s"]),
                    float(row["end_active_s"]),
                    str(row["state"]),
                )

    active = overall[overall["state"] == "all_active"].copy()
    lines = [
        "# 0531 self-level PID log analysis",
        "",
        "Generated from six CSV logs in this directory.",
        "",
        "## Overall active tracking",
        "",
        markdown_table(active[[
            "flight", "axis", "angle_err_rms_deg", "angle_abs_err_p95_deg",
            "angle_lag_ms", "gyro_err_rms_dps", "gyro_abs_err_p95_dps",
            "gyro_lag_ms", "gyro_i_rms", "gyro_d_rms",
        ]].round(3)),
        "",
        "## State summaries",
        "",
        markdown_table(overall[overall["state"].isin(["quiet_center", "rapid_step", "large_angle", "stable_hold"])][[
            "flight", "axis", "state", "duration_s", "angle_err_rms_deg",
            "angle_abs_err_p95_deg", "gyro_err_rms_dps", "gyro_abs_err_p95_dps",
            "angle_lag_ms", "gyro_lag_ms", "angle_ff_residual_rms_dps",
            "gyro_i_rms", "gyro_d_rms",
        ]].round(3)),
        "",
        "## Representative windows",
        "",
        markdown_table(windows[[
            "flight", "axis", "state", "start_active_s", "end_active_s",
            "angle_err_rms_deg", "angle_abs_err_p95_deg",
            "gyro_err_rms_dps", "gyro_abs_err_p95_dps",
            "angle_lag_ms", "gyro_lag_ms",
        ]].round(3)),
        "",
        "## Step events",
        "",
        markdown_table(step_events.groupby(["flight", "axis"], as_index=False).agg({
            "post_angle_err_rms_deg": "mean",
            "post_angle_abs_err_p95_deg": "mean",
            "t63_ms": "median",
            "overshoot_deg": "mean",
            "overshoot_pct_of_step": "mean",
            "gyro_err_rms_dps": "mean",
            "gyro_abs_err_p95_dps": "mean",
            "gyro_i_rms": "mean",
            "gyro_d_rms": "mean",
        }).round(3)),
        "",
        "## Notes",
        "",
        "- angle_ff_residual = logged angle output - P - I - D. In this firmware the angle output has a PT3 low-pass after summing, so this residual is FF plus output-filter lag/residue, not a pure FF log.",
        "- gyro_ff_residual should be near zero because gyro KFF is zero in all six flights.",
    ]
    (OUT_DIR / "analysis_summary.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
