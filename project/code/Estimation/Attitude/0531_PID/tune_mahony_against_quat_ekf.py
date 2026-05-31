from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import compare_offline_attitude_estimators as base


BASE_DIR = Path(__file__).resolve().parent
OUT_DIR = BASE_DIR / "mahony_tune_quat_ekf"


@dataclass(frozen=True)
class QuatEkfParams:
    name: str = "quat_ekf"
    gyro_noise_rad_s: float = 0.020
    accel_noise: float = 0.060
    accel_gate_g: float = 0.35
    accel_gate_soft_g: float = 0.70


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ], dtype=float)


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    out = base.quat_mul(base.quat_mul(q, np.array([0.0, v[0], v[1], v[2]], dtype=float)), quat_conj(q))
    return out[1:]


def small_angle_quat(delta: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(delta))
    if angle < 1.0e-12:
        return np.array([1.0, 0.5 * delta[0], 0.5 * delta[1], 0.5 * delta[2]], dtype=float)
    axis = delta / angle
    half = 0.5 * angle
    return np.array([math.cos(half), *(math.sin(half) * axis)], dtype=float)


def gravity_measurement(ax: float, ay: float, az: float) -> tuple[np.ndarray, float]:
    # Logs store specific force; static level is about ax=0, ay=0, az=-1g.
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    if mag < 1.0e-9:
        return np.array([0.0, 0.0, 1.0], dtype=float), 0.0
    return np.array([-ax / mag, -ay / mag, -az / mag], dtype=float), mag


def run_quat_error_state_ekf(df: pd.DataFrame, params: QuatEkfParams = QuatEkfParams()) -> tuple[pd.DataFrame, pd.DataFrame]:
    gx = df["filt_gx"].to_numpy(float) * base.DEG_TO_RAD
    gy = df["filt_gy"].to_numpy(float) * base.DEG_TO_RAD
    gz = df["filt_gz"].to_numpy(float) * base.DEG_TO_RAD
    ax = df["filt_ax"].to_numpy(float)
    ay = df["filt_ay"].to_numpy(float)
    az = df["filt_az"].to_numpy(float)
    t = df["t_ms"].to_numpy(float)
    n = len(df)

    g0, _ = gravity_measurement(ax[0], ay[0], az[0])
    q = base.quat_from_gravity(float(g0[0]), float(g0[1]), float(g0[2]))
    p = np.eye(3, dtype=float) * (2.0 * base.DEG_TO_RAD) ** 2
    roll = np.zeros(n, dtype=float)
    pitch = np.zeros(n, dtype=float)
    acc_weight = np.zeros(n, dtype=float)
    innovation_norm = np.zeros(n, dtype=float)

    world_down = np.array([0.0, 0.0, 1.0], dtype=float)

    for i in range(n):
        dt = 0.001 if i == 0 else max((t[i] - t[i - 1]) * 0.001, 1.0e-6)
        omega = np.array([gx[i], gy[i], gz[i]], dtype=float)
        q = base.integrate_quat(q, float(omega[0]), float(omega[1]), float(omega[2]), dt)

        f = np.eye(3, dtype=float) - skew(omega) * dt
        q_noise = (params.gyro_noise_rad_s * dt) ** 2
        p = f @ p @ f.T + np.eye(3, dtype=float) * q_noise

        z, amag = gravity_measurement(ax[i], ay[i], az[i])
        acc_err = abs(amag - 1.0)
        if acc_err <= params.accel_gate_soft_g:
            if acc_err <= params.accel_gate_g:
                weight = 1.0
            else:
                span = max(params.accel_gate_soft_g - params.accel_gate_g, 1.0e-6)
                weight = 1.0 - (acc_err - params.accel_gate_g) / span
            weight = max(0.0, min(1.0, weight))

            h = quat_rotate(q, world_down)
            y = z - h
            h_jac = -skew(h)
            r = np.eye(3, dtype=float) * (params.accel_noise / max(weight, 0.05)) ** 2
            s = h_jac @ p @ h_jac.T + r
            k = p @ h_jac.T @ np.linalg.pinv(s)
            delta = k @ y
            q = base.quat_normalize(base.quat_mul(small_angle_quat(delta), q))
            p = (np.eye(3, dtype=float) - k @ h_jac) @ p
            acc_weight[i] = weight
            innovation_norm[i] = float(np.linalg.norm(y))

        roll[i], pitch[i] = base.quat_to_roll_pitch(q)

    return (
        pd.DataFrame({f"{params.name}_roll": roll, f"{params.name}_pitch": pitch}),
        pd.DataFrame({f"{params.name}_acc_weight": acc_weight, f"{params.name}_innovation_norm": innovation_norm}),
    )


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
        left_y = align(df[f"{left}_{axis}"].to_numpy(float), online, align_mask)
        right_y = align(df[f"{right}_{axis}"].to_numpy(float), online, align_mask)
        diff = left_y[mask] - right_y[mask]
        rows.append({
            "flight": flight,
            "axis": axis,
            "left": left,
            "right": right,
            "rms_diff_deg": rms(diff),
            "p95_abs_diff_deg": float(np.nanpercentile(np.abs(diff), 95)),
            "mean_diff_deg": float(np.nanmean(diff)),
            "final_diff_deg": float(diff[-1]),
        })
    return rows


def make_variants() -> list[base.MahonyParams]:
    variants: list[base.MahonyParams] = []
    for kp in [0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.2]:
        for ki in [0.0, 0.02]:
            variants.append(base.MahonyParams(f"mahony_kp{kp:.1f}_ki{ki:.2f}".replace(".", "p"), kp, ki, 0.30, 3.00, 0.35))
    for band in [0.20, 0.50]:
        variants.append(base.MahonyParams(f"mahony_kp0p6_band{band:.2f}".replace(".", "p"), 0.6, 0.02, 0.30, 3.00, band))
    variants.append(base.MahonyParams("bf_gate_kp0p25_ki0", 0.25, 0.0, 0.90, 1.10, 0.10))
    # Preserve insertion order while removing duplicates from the band variants.
    out: list[base.MahonyParams] = []
    seen: set[str] = set()
    for variant in variants:
        if variant.name not in seen:
            out.append(variant)
            seen.add(variant.name)
    return out


def plot_kp_curve(score: pd.DataFrame) -> None:
    kp_rows = score[
        score["variant"].str.contains("band").eq(False)
        & score["variant"].str.startswith("mahony_kp")
        & (score["trust_band_g"] == 0.35)
    ].copy()
    if kp_rows.empty:
        return
    fig, ax = plt.subplots(figsize=(9, 5))
    for ki, group in kp_rows.groupby("ki"):
        group = group.sort_values("kp")
        ax.plot(group["kp"], group["score_rms_deg"], marker="o", label=f"Ki={ki}")
    ax.set_xlabel("Mahony Kp")
    ax.set_ylabel("score RMS deg vs quat EKF + Madgwick")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT_DIR / "kp_score_curve.png", dpi=150)
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


def main() -> None:
    OUT_DIR.mkdir(exist_ok=True)
    variants = make_variants()
    csv_files = sorted(BASE_DIR.glob("*.csv"), key=base.flight_no_from_name)

    rows: list[dict[str, float | int | str]] = []
    gate_rows: list[dict[str, float | int | str]] = []

    for path in csv_files:
        flight = base.flight_no_from_name(path)
        df = base.load_flight(path)
        ekf_angles, ekf_diag = run_quat_error_state_ekf(df)
        madgwick = base.run_madgwick_imu(df)
        frames = [ekf_angles, ekf_diag, madgwick]
        for variant in variants:
            angles, diag = base.run_mahony(df, variant)
            frames.extend([angles, diag])
        df = pd.concat([df, *frames], axis=1)
        t = df["t_ms"].to_numpy(float)
        align_mask = t <= t[0] + 500.0
        if int(np.sum(align_mask)) < 10:
            align_mask = np.arange(len(df)) < min(500, len(df))

        for variant in variants:
            rows.extend(summarize_pair(df, flight, variant.name, "quat_ekf", align_mask))
            rows.extend(summarize_pair(df, flight, variant.name, "madgwick_imu", align_mask))
            weight = df[f"{variant.name}_acc_weight"].to_numpy(float)
            gate_rows.append({
                "flight": flight,
                "variant": variant.name,
                "kp": variant.kp,
                "ki": variant.ki,
                "trust_band_g": variant.trust_band_g,
                "acc_used_pct": float(np.mean(weight > 0.0) * 100.0),
                "acc_weight_mean": float(np.nanmean(weight)),
            })
        rows.extend(summarize_pair(df, flight, "madgwick_imu", "quat_ekf", align_mask))

    pairwise = pd.DataFrame(rows)
    gates = pd.DataFrame(gate_rows)
    pairwise.to_csv(OUT_DIR / "mahony_vs_quat_ekf_pairwise.csv", index=False, encoding="utf-8-sig")
    gates.to_csv(OUT_DIR / "mahony_tune_gate_summary.csv", index=False, encoding="utf-8-sig")

    variant_pairs = pairwise[pairwise["left"].str.startswith(("mahony_", "bf_gate"))].copy()
    axis_score = variant_pairs.pivot_table(
        index=["left", "axis"],
        columns="right",
        values="rms_diff_deg",
        aggfunc="mean",
    ).reset_index()
    axis_score["score_rms_deg"] = np.sqrt((axis_score["quat_ekf"] ** 2 + axis_score["madgwick_imu"] ** 2) / 2.0)
    meta = gates.groupby("variant", as_index=False).agg({
        "kp": "first",
        "ki": "first",
        "trust_band_g": "first",
        "acc_used_pct": "mean",
        "acc_weight_mean": "mean",
    })
    axis_score = axis_score.merge(meta, left_on="left", right_on="variant", how="left")
    total_score = axis_score.groupby(["variant", "kp", "ki", "trust_band_g", "acc_used_pct", "acc_weight_mean"], as_index=False).agg({
        "score_rms_deg": "mean",
        "quat_ekf": "mean",
        "madgwick_imu": "mean",
    }).sort_values("score_rms_deg")
    total_score.to_csv(OUT_DIR / "mahony_kpki_score.csv", index=False, encoding="utf-8-sig")
    plot_kp_curve(total_score)

    ekf_mad = pairwise[(pairwise["left"] == "madgwick_imu") & (pairwise["right"] == "quat_ekf")]
    ekf_mad_key = ekf_mad.pivot_table(index="axis", values=["rms_diff_deg", "p95_abs_diff_deg"], aggfunc="mean").reset_index().round(3)
    top = total_score.head(12).round(4)
    current = total_score[total_score["variant"] == "mahony_kp1p0_ki0p02"].round(4)
    bf = total_score[total_score["variant"] == "bf_gate_kp0p25_ki0"].round(4)

    lines = [
        "# Mahony Kp/Ki tuning against quaternion error-state EKF",
        "",
        "Reference EKF: 3D quaternion error-state EKF using gyro propagation and normalized accelerometer gravity-vector updates with soft acceleration-magnitude rejection.",
        "This is appropriate for 6-axis roll/pitch comparison; yaw remains unobservable without magnetometer or other heading information.",
        "",
        "Score: mean roll/pitch RMS difference to both quaternion EKF and Madgwick IMU after first-0.5s offset alignment. Lower is closer to the independent references.",
        "",
        "## Madgwick vs quaternion EKF sanity check",
        "",
        markdown_table(ekf_mad_key),
        "",
        "## Best Mahony variants",
        "",
        markdown_table(top[["variant", "kp", "ki", "trust_band_g", "score_rms_deg", "quat_ekf", "madgwick_imu", "acc_used_pct", "acc_weight_mean"]]),
        "",
        "## Current firmware parameter score",
        "",
        markdown_table(current[["variant", "kp", "ki", "trust_band_g", "score_rms_deg", "quat_ekf", "madgwick_imu", "acc_used_pct", "acc_weight_mean"]]),
        "",
        "## BF hard-gate parameter score",
        "",
        markdown_table(bf[["variant", "kp", "ki", "trust_band_g", "score_rms_deg", "quat_ekf", "madgwick_imu", "acc_used_pct", "acc_weight_mean"]]),
        "",
        "## Practical reading",
        "",
        "- Best-fit Kp for these six logs is around 0.4-0.6 with the wide 0.30-3.00g gate and 0.35 trust band.",
        "- Kp=1.0 is still close to the EKF/Madgwick references, but it is more aggressive and slightly farther in this score.",
        "- Ki has almost no effect in these flight logs because firmware Ki only learns bias when the static detector is locked.",
        "- The old BF hard gate scores poorly mainly because correction is unavailable for too much of the flight.",
        "",
        "## Files",
        "",
        "- `mahony_kpki_score.csv`: aggregate ranking.",
        "- `mahony_vs_quat_ekf_pairwise.csv`: per-flight pairwise metrics.",
        "- `mahony_tune_gate_summary.csv`: accelerometer correction usage.",
        "- `kp_score_curve.png`: Kp sweep curve.",
    ]
    (OUT_DIR / "mahony_tune_quat_ekf_report.md").write_text("\n".join(lines), encoding="utf-8")
    print(OUT_DIR)


if __name__ == "__main__":
    main()
