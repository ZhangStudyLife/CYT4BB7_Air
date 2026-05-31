from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import compare_offline_attitude_estimators as base


BASE_DIR = Path(__file__).resolve().parent
OUT_DIR = BASE_DIR / "mahony_param_sweep_vs_ekf"


@dataclass(frozen=True)
class EkfParams:
    name: str
    gyro_noise_dps: float
    accel_noise_deg: float
    accel_gate_g: float
    accel_gate_soft_g: float


EKF_DEFAULT = EkfParams(
    name="ekf_rp",
    gyro_noise_dps=0.9,
    accel_noise_deg=4.0,
    accel_gate_g=0.35,
    accel_gate_soft_g=0.60,
)


MAHONY_VARIANTS = [
    base.MahonyParams("bf_gate_kp025_ki0", 0.25, 0.0, 0.90, 1.10, 0.10),
    base.MahonyParams("current_kp10_ki002", 1.0, 0.02, 0.30, 3.00, 0.35),
    base.MahonyParams("current_kp06_ki002", 0.6, 0.02, 0.30, 3.00, 0.35),
    base.MahonyParams("current_kp15_ki002", 1.5, 0.02, 0.30, 3.00, 0.35),
    base.MahonyParams("current_kp10_ki0", 1.0, 0.0, 0.30, 3.00, 0.35),
    base.MahonyParams("wide_kp10_band020", 1.0, 0.02, 0.30, 3.00, 0.20),
    base.MahonyParams("wide_kp10_band050", 1.0, 0.02, 0.30, 3.00, 0.50),
    base.MahonyParams("mid_gate_kp10", 1.0, 0.02, 0.70, 1.50, 0.35),
]


def accel_roll_pitch(ax: np.ndarray, ay: np.ndarray, az: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    gx = -ax
    gy = -ay
    gz = -az
    mag = np.sqrt(gx * gx + gy * gy + gz * gz)
    safe = np.where(mag > 1.0e-9, mag, 1.0)
    gx = gx / safe
    gy = gy / safe
    gz = gz / safe
    roll = np.degrees(np.arctan2(gy, gz))
    pitch = np.degrees(np.arctan2(-gx, np.sqrt(gy * gy + gz * gz)))
    return roll, pitch, mag


def run_rp_ekf(df: pd.DataFrame, params: EkfParams = EKF_DEFAULT) -> tuple[pd.DataFrame, pd.DataFrame]:
    gx = df["filt_gx"].to_numpy(float)
    gy = df["filt_gy"].to_numpy(float)
    ax = df["filt_ax"].to_numpy(float)
    ay = df["filt_ay"].to_numpy(float)
    az = df["filt_az"].to_numpy(float)
    t = df["t_ms"].to_numpy(float)
    acc_roll, acc_pitch, acc_mag = accel_roll_pitch(ax, ay, az)

    n = len(df)
    x = np.array([acc_roll[0], acc_pitch[0]], dtype=float)
    p = np.eye(2, dtype=float) * 4.0
    roll = np.zeros(n, dtype=float)
    pitch = np.zeros(n, dtype=float)
    gain_roll = np.zeros(n, dtype=float)
    gain_pitch = np.zeros(n, dtype=float)
    acc_weight = np.zeros(n, dtype=float)

    for i in range(n):
        dt = 0.001 if i == 0 else max((t[i] - t[i - 1]) * 0.001, 1.0e-6)
        q = (params.gyro_noise_dps * max(dt, 1.0e-3)) ** 2
        x[0] += gx[i] * dt
        x[1] += gy[i] * dt
        p = p + np.eye(2, dtype=float) * q

        acc_err = abs(acc_mag[i] - 1.0)
        if acc_err <= params.accel_gate_soft_g:
            if acc_err <= params.accel_gate_g:
                weight = 1.0
            else:
                span = max(params.accel_gate_soft_g - params.accel_gate_g, 1.0e-6)
                weight = 1.0 - (acc_err - params.accel_gate_g) / span
            weight = max(0.0, min(1.0, weight))
            r = (params.accel_noise_deg / max(weight, 0.05)) ** 2
            for axis, z in enumerate((acc_roll[i], acc_pitch[i])):
                y = base.wrap_to_180(float(z - x[axis]))
                s = p[axis, axis] + r
                k = p[axis, axis] / s
                x[axis] += k * y
                p[axis, axis] = (1.0 - k) * p[axis, axis]
                if axis == 0:
                    gain_roll[i] = k
                else:
                    gain_pitch[i] = k
            acc_weight[i] = weight

        roll[i] = x[0]
        pitch[i] = x[1]

    angles = pd.DataFrame({f"{params.name}_roll": roll, f"{params.name}_pitch": pitch})
    diag = pd.DataFrame({
        f"{params.name}_acc_weight": acc_weight,
        f"{params.name}_gain_roll": gain_roll,
        f"{params.name}_gain_pitch": gain_pitch,
    })
    return angles, diag


def align(series: np.ndarray, ref: np.ndarray, align_mask: np.ndarray) -> np.ndarray:
    offset = float(np.nanmedian(ref[align_mask] - series[align_mask]))
    return series + offset


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.nanmean(values * values))) if values.size else math.nan


def summarize_pair(df: pd.DataFrame, flight: int, left: str, right: str, align_mask: np.ndarray) -> list[dict[str, float | int | str]]:
    rows: list[dict[str, float | int | str]] = []
    t = (df["t_ms"] - df["t_ms"].iloc[0]).to_numpy(float) * 0.001
    mask = t >= 1.0
    for axis in ["roll", "pitch"]:
        online = df[f"online_{axis}"].to_numpy(float)
        a = align(df[f"{left}_{axis}"].to_numpy(float), online, align_mask)
        b = align(df[f"{right}_{axis}"].to_numpy(float), online, align_mask)
        diff = a[mask] - b[mask]
        rows.append({
            "flight": flight,
            "axis": axis,
            "left": left,
            "right": right,
            "rms_diff_deg": rms(diff),
            "mean_diff_deg": float(np.nanmean(diff)),
            "p95_abs_diff_deg": float(np.nanpercentile(np.abs(diff), 95)),
            "max_abs_diff_deg": float(np.nanmax(np.abs(diff))),
            "final_diff_deg": float(diff[-1]),
        })
    return rows


def summarize_vs_online(df: pd.DataFrame, flight: int, estimator: str, align_mask: np.ndarray) -> list[dict[str, float | int | str]]:
    rows: list[dict[str, float | int | str]] = []
    t = (df["t_ms"] - df["t_ms"].iloc[0]).to_numpy(float) * 0.001
    mask = t >= 1.0
    for axis in ["roll", "pitch"]:
        online = df[f"online_{axis}"].to_numpy(float)
        est = align(df[f"{estimator}_{axis}"].to_numpy(float), online, align_mask)
        err = est[mask] - online[mask]
        rows.append({
            "flight": flight,
            "axis": axis,
            "estimator": estimator,
            "rms_vs_online_deg": rms(err),
            "mean_vs_online_deg": float(np.nanmean(err)),
            "p95_abs_vs_online_deg": float(np.nanpercentile(np.abs(err), 95)),
            "final_vs_online_deg": float(err[-1]),
        })
    return rows


def summarize_variant_diag(df: pd.DataFrame, flight: int, variant: base.MahonyParams) -> dict[str, float | int | str]:
    weight = df[f"{variant.name}_acc_weight"].to_numpy(float)
    used = weight > 0.0
    return {
        "flight": flight,
        "variant": variant.name,
        "kp": variant.kp,
        "ki": variant.ki,
        "accel_min_g": variant.accel_min_g,
        "accel_max_g": variant.accel_max_g,
        "trust_band_g": variant.trust_band_g,
        "acc_used_pct": float(np.mean(used) * 100.0),
        "acc_weight_mean": float(np.nanmean(weight)),
        "acc_weight_p95": float(np.nanpercentile(weight, 95)),
    }


def plot_flight(df: pd.DataFrame, flight: int, align_mask: np.ndarray) -> None:
    t = (df["t_ms"] - df["t_ms"].iloc[0]).to_numpy(float) * 0.001
    estimators = ["current_kp10_ki002", "madgwick_imu", "ekf_rp", "bf_gate_kp025_ki0", "current_kp06_ki002", "current_kp15_ki002"]
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    for idx, axis in enumerate(["roll", "pitch"]):
        online = df[f"online_{axis}"].to_numpy(float)
        axes[idx].plot(t, online, color="black", linewidth=1.0, label="online")
        for estimator in estimators:
            y = align(df[f"{estimator}_{axis}"].to_numpy(float), online, align_mask)
            axes[idx].plot(t, y, linewidth=0.85, label=estimator)
        axes[idx].set_ylabel(f"{axis} deg")
        axes[idx].grid(True, alpha=0.25)
        axes[idx].legend(ncol=3, fontsize=8, loc="upper right")

    axes[2].plot(t, df["current_kp10_ki002_acc_weight"], label="current acc weight", linewidth=0.9)
    axes[2].plot(t, df["bf_gate_kp025_ki0_acc_weight"], label="bf-gate acc weight", linewidth=0.9)
    axes[2].plot(t, df["ekf_rp_acc_weight"], label="ekf acc weight", linewidth=0.9)
    axes[2].set_ylabel("weight")
    axes[2].set_xlabel("time s")
    axes[2].grid(True, alpha=0.25)
    axes[2].legend(fontsize=8, loc="upper right")
    fig.suptitle(f"flight {flight}: Mahony parameter sweep vs Madgwick/EKF")
    fig.tight_layout()
    fig.savefig(OUT_DIR / f"flight{flight}_param_sweep.png", dpi=150)
    plt.close(fig)


def markdown_table(df: pd.DataFrame) -> str:
    if df.empty:
        return "_No rows._"
    text_df = df.copy()
    for col in text_df.columns:
        text_df[col] = text_df[col].map(lambda x: "" if pd.isna(x) else str(x))
    values = text_df.values.tolist()
    widths = [max(len(str(col)), *(len(str(row[i])) for row in values)) for i, col in enumerate(text_df.columns)]
    lines = [
        "| " + " | ".join(str(col).ljust(widths[i]) for i, col in enumerate(text_df.columns)) + " |",
        "| " + " | ".join("-" * widths[i] for i in range(len(text_df.columns))) + " |",
    ]
    for row in values:
        lines.append("| " + " | ".join(str(row[i]).ljust(widths[i]) for i in range(len(row))) + " |")
    return "\n".join(lines)


def process_flight(path: Path) -> tuple[list[dict[str, float | int | str]], list[dict[str, float | int | str]], list[dict[str, float | int | str]], list[dict[str, float | int | str]]]:
    flight = base.flight_no_from_name(path)
    df = base.load_flight(path)

    frames = []
    diag_frames = []
    variant_diag = []
    for variant in MAHONY_VARIANTS:
        angles, diag = base.run_mahony(df, variant)
        frames.append(angles)
        diag_frames.append(diag)
    frames.append(base.run_madgwick_imu(df))
    ekf_angles, ekf_diag = run_rp_ekf(df)
    frames.append(ekf_angles)
    diag_frames.append(ekf_diag)
    df = pd.concat([df, *frames, *diag_frames], axis=1)

    t = df["t_ms"].to_numpy(float)
    align_mask = t <= t[0] + 500.0
    if int(np.sum(align_mask)) < 10:
        align_mask = np.arange(len(df)) < min(500, len(df))

    for variant in MAHONY_VARIANTS:
        variant_diag.append(summarize_variant_diag(df, flight, variant))

    pair_rows = []
    for left, right in [
        ("current_kp10_ki002", "madgwick_imu"),
        ("current_kp10_ki002", "ekf_rp"),
        ("madgwick_imu", "ekf_rp"),
        ("bf_gate_kp025_ki0", "current_kp10_ki002"),
        ("current_kp06_ki002", "current_kp10_ki002"),
        ("current_kp15_ki002", "current_kp10_ki002"),
        ("current_kp10_ki0", "current_kp10_ki002"),
        ("wide_kp10_band020", "current_kp10_ki002"),
        ("wide_kp10_band050", "current_kp10_ki002"),
    ]:
        pair_rows.extend(summarize_pair(df, flight, left, right, align_mask))

    vs_online = []
    for estimator in [v.name for v in MAHONY_VARIANTS] + ["madgwick_imu", "ekf_rp"]:
        vs_online.extend(summarize_vs_online(df, flight, estimator, align_mask))

    plot_flight(df, flight, align_mask)
    return pair_rows, vs_online, variant_diag, []


def main() -> None:
    OUT_DIR.mkdir(exist_ok=True)
    csv_files = sorted(BASE_DIR.glob("*.csv"), key=base.flight_no_from_name)
    pair_rows: list[dict[str, float | int | str]] = []
    online_rows: list[dict[str, float | int | str]] = []
    diag_rows: list[dict[str, float | int | str]] = []

    for path in csv_files:
        pairs, online, diag, _ = process_flight(path)
        pair_rows.extend(pairs)
        online_rows.extend(online)
        diag_rows.extend(diag)

    pairwise = pd.DataFrame(pair_rows)
    online = pd.DataFrame(online_rows)
    diag = pd.DataFrame(diag_rows)
    pairwise.to_csv(OUT_DIR / "pairwise_summary.csv", index=False, encoding="utf-8-sig")
    online.to_csv(OUT_DIR / "vs_online_summary.csv", index=False, encoding="utf-8-sig")
    diag.to_csv(OUT_DIR / "mahony_variant_gate_summary.csv", index=False, encoding="utf-8-sig")

    pair_key = pairwise.pivot_table(
        index=["left", "right", "axis"],
        values=["rms_diff_deg", "p95_abs_diff_deg", "final_diff_deg"],
        aggfunc="mean",
    ).reset_index().round(3)
    online_key = online.pivot_table(
        index=["estimator", "axis"],
        values=["rms_vs_online_deg", "p95_abs_vs_online_deg", "final_vs_online_deg"],
        aggfunc="mean",
    ).reset_index().round(3)
    diag_key = diag.pivot_table(
        index=["variant", "kp", "ki", "accel_min_g", "accel_max_g", "trust_band_g"],
        values=["acc_used_pct", "acc_weight_mean"],
        aggfunc="mean",
    ).reset_index().round(3)

    lines = [
        "# Mahony parameter sweep vs Madgwick and roll/pitch EKF",
        "",
        "This report uses the same six 0531_PID logs and the filtered I7-I12 6-axis data.",
        "All curves are offset-aligned to the first 0.5 s before statistics. Online Euler is a firmware reference, not ground truth.",
        "",
        "## Pairwise mean differences across six flights",
        "",
        markdown_table(pair_key),
        "",
        "## Mean difference vs online Euler across six flights",
        "",
        markdown_table(online_key),
        "",
        "## Mahony accelerometer usage by variant",
        "",
        markdown_table(diag_key),
        "",
        "## Parameter interpretation",
        "",
        "- `Kp` controls how strongly the gravity-vector error corrects gyro integration. Higher `Kp` reduces long gyro-only drift faster, but follows acceleration-induced false gravity more aggressively.",
        "- `Ki` only matters in this firmware when the static detector is locked. It slowly learns roll/pitch gyro bias; it has little effect during active flight in these logs.",
        "- The `0.9g-1.1g` hard gate removes many flight samples from correction. It can make the angle look smooth while the estimate is mostly gyro-integrated.",
        "- Wider accel magnitude limits plus a trust band do not mean blindly trusting acceleration; the trust band still reduces correction when `|acc|-1g` grows.",
        "- Madgwick and the simple roll/pitch EKF are independent references here. They are not ground truth, but agreement between them is a useful warning when Mahony differs strongly.",
        "",
        "## Files",
        "",
        "- `pairwise_summary.csv`: per-flight pairwise differences.",
        "- `vs_online_summary.csv`: per-flight differences against online Euler.",
        "- `mahony_variant_gate_summary.csv`: accelerometer correction usage by Mahony variant.",
        "- `flightN_param_sweep.png`: plotted curves for each flight.",
    ]
    (OUT_DIR / "mahony_param_sweep_report.md").write_text("\n".join(lines), encoding="utf-8")
    print(OUT_DIR)


if __name__ == "__main__":
    main()
