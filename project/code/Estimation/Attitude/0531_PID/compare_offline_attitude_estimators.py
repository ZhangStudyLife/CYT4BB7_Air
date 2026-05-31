from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


BASE_DIR = Path(__file__).resolve().parent
OUT_DIR = BASE_DIR / "offline_attitude_estimators"

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi

FLIGHT_ORDER = {
    "一": 1,
    "二": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    "六": 6,
}

COLUMNS = {
    "t_ms": "I0",
    "raw_gx": "I1",
    "raw_gy": "I2",
    "raw_gz": "I3",
    "raw_ax": "I4",
    "raw_ay": "I5",
    "raw_az": "I6",
    "filt_gx": "I7",
    "filt_gy": "I8",
    "filt_gz": "I9",
    "filt_ax": "I10",
    "filt_ay": "I11",
    "filt_az": "I12",
    "online_roll": "I13",
    "online_pitch": "I14",
}


@dataclass(frozen=True)
class MahonyParams:
    name: str
    kp: float
    ki: float
    accel_min_g: float
    accel_max_g: float
    trust_band_g: float
    rate_start_dps: float = 12.0
    rate_slope_dps: float = 18.0
    gyro_lpf_hz: float = 3.0
    accel_weight_min: float = 0.0
    integral_limit_deg: float = 2.0


REPO_MAHONY = MahonyParams(
    name="repo_mahony",
    kp=0.25,
    ki=0.0,
    accel_min_g=0.90,
    accel_max_g=1.10,
    trust_band_g=0.10,
)

SOFT_MAHONY = MahonyParams(
    name="soft_mahony",
    kp=1.0,
    ki=0.02,
    accel_min_g=0.30,
    accel_max_g=3.00,
    trust_band_g=0.35,
)


def flight_no_from_name(path: Path) -> int:
    if len(path.name) < 2 or path.name[1] not in FLIGHT_ORDER:
        raise ValueError(f"Cannot infer flight number from {path.name}")
    return FLIGHT_ORDER[path.name[1]]


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def quat_normalize(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / norm


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=float)


def quat_from_gravity(gx: float, gy: float, gz: float) -> np.ndarray:
    norm = math.sqrt(gx * gx + gy * gy + gz * gz)
    if norm < 1.0e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    q = np.array([gz + norm, gy, -gx, 0.0], dtype=float)
    return quat_normalize(q)


def gravity_from_specific_force(ax: float, ay: float, az: float) -> tuple[float, float, float, float]:
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    if mag < 1.0e-9:
        return 0.0, 0.0, 1.0, 0.0
    return -ax / mag, -ay / mag, -az / mag, mag


def estimated_gravity_body(q: np.ndarray) -> tuple[float, float, float]:
    q0, q1, q2, q3 = q
    return (
        2.0 * (q1 * q3 - q0 * q2),
        2.0 * (q0 * q1 + q2 * q3),
        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3,
    )


def quat_to_roll_pitch(q: np.ndarray) -> tuple[float, float]:
    q0, q1, q2, q3 = q
    roll = math.atan2(
        2.0 * (q0 * q1 + q2 * q3),
        1.0 - 2.0 * (q1 * q1 + q2 * q2),
    )
    sin_pitch = 2.0 * (q0 * q2 - q3 * q1)
    pitch = math.asin(clamp(sin_pitch, -1.0, 1.0))
    return roll * RAD_TO_DEG, pitch * RAD_TO_DEG


def integrate_quat(q: np.ndarray, wx: float, wy: float, wz: float, dt: float) -> np.ndarray:
    theta = np.array([wx, wy, wz], dtype=float) * (0.5 * dt)
    theta_mag_sq = float(np.dot(theta, theta))
    if theta_mag_sq < 1.0e-20:
        return q
    if theta_mag_sq < 0.00489898:
        scale = 1.0 - theta_mag_sq / 6.0
        dq = np.array([
            1.0 - theta_mag_sq / 2.0,
            theta[0] * scale,
            theta[1] * scale,
            theta[2] * scale,
        ], dtype=float)
    else:
        theta_mag = math.sqrt(theta_mag_sq)
        scale = math.sin(theta_mag) / theta_mag
        dq = np.array([
            math.cos(theta_mag),
            theta[0] * scale,
            theta[1] * scale,
            theta[2] * scale,
        ], dtype=float)
    return quat_normalize(quat_mul(q, dq))


def pt1_update(state: float, value: float, cutoff_hz: float, dt: float) -> float:
    if cutoff_hz <= 0.0 or dt <= 0.0:
        return value
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    alpha = clamp(dt / (rc + dt), 0.0, 1.0)
    return state + alpha * (value - state)


def accel_weight_nearness(accel_mag: float, trust_band: float) -> float:
    if trust_band <= 0.0:
        return 1.0
    return clamp(1.0 - abs(accel_mag - 1.0) / trust_band, 0.0, 1.0)


def accel_weight_rate(gyro_lpf: np.ndarray, params: MahonyParams) -> float:
    if params.rate_start_dps <= 0.0 or params.rate_slope_dps <= 0.0:
        return 1.0
    rate_dps = float(np.linalg.norm(gyro_lpf)) * RAD_TO_DEG
    gate_end = params.rate_start_dps + params.rate_slope_dps
    if rate_dps <= params.rate_start_dps:
        return 1.0
    if rate_dps >= gate_end:
        return 0.0
    return 1.0 - (rate_dps - params.rate_start_dps) / params.rate_slope_dps


def run_mahony(df: pd.DataFrame, params: MahonyParams) -> tuple[pd.DataFrame, pd.DataFrame]:
    gx = df["filt_gx"].to_numpy(float)
    gy = df["filt_gy"].to_numpy(float)
    gz = df["filt_gz"].to_numpy(float)
    ax = df["filt_ax"].to_numpy(float)
    ay = df["filt_ay"].to_numpy(float)
    az = df["filt_az"].to_numpy(float)
    t = df["t_ms"].to_numpy(float)
    n = len(df)

    gravity0 = gravity_from_specific_force(ax[0], ay[0], az[0])
    q = quat_from_gravity(gravity0[0], gravity0[1], gravity0[2])
    gyro_lpf = np.zeros(3, dtype=float)
    gyro_lpf_ready = False
    integral = np.zeros(2, dtype=float)
    integral_limit = params.integral_limit_deg * DEG_TO_RAD
    static_count = 0

    roll = np.zeros(n, dtype=float)
    pitch = np.zeros(n, dtype=float)
    accel_mag = np.zeros(n, dtype=float)
    acc_valid = np.zeros(n, dtype=bool)
    acc_weight = np.zeros(n, dtype=float)
    acc_near = np.zeros(n, dtype=float)
    acc_rate = np.zeros(n, dtype=float)

    for i in range(n):
        dt = 0.001 if i == 0 else max((t[i] - t[i - 1]) * 0.001, 1.0e-6)
        wx = gx[i] * DEG_TO_RAD
        wy = gy[i] * DEG_TO_RAD
        wz = gz[i] * DEG_TO_RAD
        g_meas_x, g_meas_y, g_meas_z, amag = gravity_from_specific_force(ax[i], ay[i], az[i])
        accel_mag[i] = amag

        gyro_abs_dps = math.sqrt(gx[i] * gx[i] + gy[i] * gy[i] + gz[i] * gz[i])
        if gyro_abs_dps < 1.5 and params.accel_min_g < amag < params.accel_max_g and abs(amag - 1.0) < 0.08:
            static_count = min(static_count + 1, 65535)
        else:
            static_count = 0
        is_static = static_count >= 100

        if not gyro_lpf_ready:
            gyro_lpf[:] = (wx, wy, wz)
            gyro_lpf_ready = True
        else:
            gyro_lpf[0] = pt1_update(float(gyro_lpf[0]), wx, params.gyro_lpf_hz, dt)
            gyro_lpf[1] = pt1_update(float(gyro_lpf[1]), wy, params.gyro_lpf_hz, dt)
            gyro_lpf[2] = pt1_update(float(gyro_lpf[2]), wz, params.gyro_lpf_hz, dt)

        valid = params.accel_min_g < amag < params.accel_max_g
        weight = 0.0
        if valid:
            near = accel_weight_nearness(amag, params.trust_band_g)
            rate = accel_weight_rate(gyro_lpf, params)
            weight = near * rate
            acc_valid[i] = True
            acc_near[i] = near
            acc_rate[i] = rate
            acc_weight[i] = weight

        correction = np.zeros(3, dtype=float)
        if valid and weight > params.accel_weight_min:
            est_x, est_y, est_z = estimated_gravity_body(q)
            err = np.array([
                g_meas_y * est_z - g_meas_z * est_y,
                g_meas_z * est_x - g_meas_x * est_z,
                g_meas_x * est_y - g_meas_y * est_x,
            ], dtype=float)
            if params.ki > 0.0 and is_static:
                integral += params.ki * err[:2] * weight * dt
                integral[0] = clamp(float(integral[0]), -integral_limit, integral_limit)
                integral[1] = clamp(float(integral[1]), -integral_limit, integral_limit)
            correction = params.kp * weight * err

        q = integrate_quat(
            q,
            wx + integral[0] + correction[0],
            wy + integral[1] + correction[1],
            wz + correction[2],
            dt,
        )
        roll[i], pitch[i] = quat_to_roll_pitch(q)

    angles = pd.DataFrame({f"{params.name}_roll": roll, f"{params.name}_pitch": pitch})
    diag = pd.DataFrame({
        f"{params.name}_accel_mag": accel_mag,
        f"{params.name}_acc_valid": acc_valid,
        f"{params.name}_acc_weight": acc_weight,
        f"{params.name}_acc_near": acc_near,
        f"{params.name}_acc_rate": acc_rate,
    })
    return angles, diag


def run_gyro_only(df: pd.DataFrame) -> pd.DataFrame:
    gx = df["filt_gx"].to_numpy(float)
    gy = df["filt_gy"].to_numpy(float)
    gz = df["filt_gz"].to_numpy(float)
    ax = df["filt_ax"].to_numpy(float)
    ay = df["filt_ay"].to_numpy(float)
    az = df["filt_az"].to_numpy(float)
    t = df["t_ms"].to_numpy(float)
    n = len(df)

    gravity0 = gravity_from_specific_force(ax[0], ay[0], az[0])
    q = quat_from_gravity(gravity0[0], gravity0[1], gravity0[2])
    roll = np.zeros(n, dtype=float)
    pitch = np.zeros(n, dtype=float)
    for i in range(n):
        dt = 0.001 if i == 0 else max((t[i] - t[i - 1]) * 0.001, 1.0e-6)
        q = integrate_quat(q, gx[i] * DEG_TO_RAD, gy[i] * DEG_TO_RAD, gz[i] * DEG_TO_RAD, dt)
        roll[i], pitch[i] = quat_to_roll_pitch(q)
    return pd.DataFrame({"gyro_only_roll": roll, "gyro_only_pitch": pitch})


def run_madgwick_imu(df: pd.DataFrame, beta: float = 0.04) -> pd.DataFrame:
    gx = df["filt_gx"].to_numpy(float) * DEG_TO_RAD
    gy = df["filt_gy"].to_numpy(float) * DEG_TO_RAD
    gz = df["filt_gz"].to_numpy(float) * DEG_TO_RAD
    ax = df["filt_ax"].to_numpy(float)
    ay = df["filt_ay"].to_numpy(float)
    az = df["filt_az"].to_numpy(float)
    t = df["t_ms"].to_numpy(float)
    n = len(df)

    gravity0 = gravity_from_specific_force(ax[0], ay[0], az[0])
    q = quat_from_gravity(gravity0[0], gravity0[1], gravity0[2])
    roll = np.zeros(n, dtype=float)
    pitch = np.zeros(n, dtype=float)

    for i in range(n):
        dt = 0.001 if i == 0 else max((t[i] - t[i - 1]) * 0.001, 1.0e-6)
        q1, q2, q3, q4 = q
        grav_x, grav_y, grav_z, amag = gravity_from_specific_force(ax[i], ay[i], az[i])

        q_dot = 0.5 * quat_mul(q, np.array([0.0, gx[i], gy[i], gz[i]], dtype=float))
        if amag > 1.0e-9:
            f1 = 2.0 * (q2 * q4 - q1 * q3) - grav_x
            f2 = 2.0 * (q1 * q2 + q3 * q4) - grav_y
            f3 = 2.0 * (0.5 - q2 * q2 - q3 * q3) - grav_z
            step = np.array([
                -2.0 * q3 * f1 + 2.0 * q2 * f2,
                2.0 * q4 * f1 + 2.0 * q1 * f2 - 4.0 * q2 * f3,
                -2.0 * q1 * f1 + 2.0 * q4 * f2 - 4.0 * q3 * f3,
                2.0 * q2 * f1 + 2.0 * q3 * f2,
            ], dtype=float)
            step_norm = float(np.linalg.norm(step))
            if step_norm > 1.0e-9:
                q_dot -= beta * step / step_norm

        q = quat_normalize(q + q_dot * dt)
        roll[i], pitch[i] = quat_to_roll_pitch(q)

    return pd.DataFrame({"madgwick_imu_roll": roll, "madgwick_imu_pitch": pitch})


def accel_roll_pitch(ax: float, ay: float, az: float) -> tuple[float, float]:
    gx, gy, gz, _ = gravity_from_specific_force(ax, ay, az)
    roll = math.atan2(gy, gz) * RAD_TO_DEG
    pitch = math.atan2(-gx, math.sqrt(gy * gy + gz * gz)) * RAD_TO_DEG
    return roll, pitch


def wrap_to_180(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def run_complementary_rp(df: pd.DataFrame, tau_s: float = 0.50) -> pd.DataFrame:
    gx = df["filt_gx"].to_numpy(float)
    gy = df["filt_gy"].to_numpy(float)
    ax = df["filt_ax"].to_numpy(float)
    ay = df["filt_ay"].to_numpy(float)
    az = df["filt_az"].to_numpy(float)
    t = df["t_ms"].to_numpy(float)
    n = len(df)

    roll = np.zeros(n, dtype=float)
    pitch = np.zeros(n, dtype=float)
    roll[0], pitch[0] = accel_roll_pitch(ax[0], ay[0], az[0])

    for i in range(1, n):
        dt = max((t[i] - t[i - 1]) * 0.001, 1.0e-6)
        alpha = tau_s / (tau_s + dt)
        acc_roll, acc_pitch = accel_roll_pitch(ax[i], ay[i], az[i])
        pred_roll = roll[i - 1] + gx[i] * dt
        pred_pitch = pitch[i - 1] + gy[i] * dt
        roll[i] = pred_roll + (1.0 - alpha) * wrap_to_180(acc_roll - pred_roll)
        pitch[i] = pred_pitch + (1.0 - alpha) * wrap_to_180(acc_pitch - pred_pitch)

    return pd.DataFrame({"complementary_rp_roll": roll, "complementary_rp_pitch": pitch})


def align_to_online(df: pd.DataFrame, estimator: str, axis: str, align_mask: np.ndarray) -> np.ndarray:
    online = df[f"online_{axis}"].to_numpy(float)
    estimate = df[f"{estimator}_{axis}"].to_numpy(float)
    offset = float(np.nanmedian(online[align_mask] - estimate[align_mask]))
    return estimate + offset


def longest_false_run_ms(mask: np.ndarray, t_ms: np.ndarray) -> float:
    false_idx = np.flatnonzero(~mask)
    if false_idx.size == 0:
        return 0.0
    groups = np.split(false_idx, np.where(np.diff(false_idx) > 1)[0] + 1)
    best = 0.0
    for group in groups:
        if group.size == 0:
            continue
        duration = float(t_ms[group[-1]] - t_ms[group[0]])
        best = max(best, duration)
    return best


def rms(values: np.ndarray) -> float:
    if values.size == 0:
        return math.nan
    return float(np.sqrt(np.nanmean(values * values)))


def summarize_estimator(df: pd.DataFrame, flight: int, estimator: str, align_mask: np.ndarray) -> list[dict[str, float | int | str]]:
    rows: list[dict[str, float | int | str]] = []
    t_active = (df["t_ms"] - df["t_ms"].iloc[0]).to_numpy(float) * 0.001
    analysis_mask = t_active >= 1.0
    for axis in ["roll", "pitch"]:
        online = df[f"online_{axis}"].to_numpy(float)
        aligned = align_to_online(df, estimator, axis, align_mask)
        err = aligned[analysis_mask] - online[analysis_mask]
        rows.append({
            "flight": flight,
            "estimator": estimator,
            "axis": axis,
            "rms_vs_online_deg": rms(err),
            "mean_vs_online_deg": float(np.nanmean(err)),
            "p95_abs_vs_online_deg": float(np.nanpercentile(np.abs(err), 95)),
            "max_abs_vs_online_deg": float(np.nanmax(np.abs(err))),
            "final_vs_online_deg": float(err[-1]),
            "range_deg": float(np.nanmax(aligned[analysis_mask]) - np.nanmin(aligned[analysis_mask])),
        })
    return rows


def summarize_disagreement(df: pd.DataFrame, flight: int, estimators: list[str], align_mask: np.ndarray) -> list[dict[str, float | int | str]]:
    rows: list[dict[str, float | int | str]] = []
    t_active = (df["t_ms"] - df["t_ms"].iloc[0]).to_numpy(float) * 0.001
    analysis_mask = t_active >= 1.0
    for axis in ["roll", "pitch"]:
        aligned = np.vstack([align_to_online(df, est, axis, align_mask) for est in estimators])
        median = np.nanmedian(aligned, axis=0)
        for idx, estimator in enumerate(estimators):
            err = aligned[idx, analysis_mask] - median[analysis_mask]
            rows.append({
                "flight": flight,
                "estimator": estimator,
                "axis": axis,
                "rms_vs_algo_median_deg": rms(err),
                "mean_vs_algo_median_deg": float(np.nanmean(err)),
                "p95_abs_vs_algo_median_deg": float(np.nanpercentile(np.abs(err), 95)),
                "max_abs_vs_algo_median_deg": float(np.nanmax(np.abs(err))),
            })
    return rows


def summarize_pairwise(df: pd.DataFrame, flight: int, pairs: list[tuple[str, str]], align_mask: np.ndarray) -> list[dict[str, float | int | str]]:
    rows: list[dict[str, float | int | str]] = []
    t_active = (df["t_ms"] - df["t_ms"].iloc[0]).to_numpy(float) * 0.001
    analysis_mask = t_active >= 1.0
    for axis in ["roll", "pitch"]:
        aligned = {
            estimator: align_to_online(df, estimator, axis, align_mask)
            for pair in pairs
            for estimator in pair
        }
        for left, right in pairs:
            diff = aligned[left][analysis_mask] - aligned[right][analysis_mask]
            rows.append({
                "flight": flight,
                "axis": axis,
                "left": left,
                "right": right,
                "rms_diff_deg": rms(diff),
                "mean_diff_deg": float(np.nanmean(diff)),
                "p95_abs_diff_deg": float(np.nanpercentile(np.abs(diff), 95)),
                "max_abs_diff_deg": float(np.nanmax(np.abs(diff))),
            })
    return rows


def plot_flight(df: pd.DataFrame, flight: int, estimators: list[str], align_mask: np.ndarray) -> None:
    t = (df["t_ms"] - df["t_ms"].iloc[0]).to_numpy(float) * 0.001
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    for ax_i, axis in enumerate(["roll", "pitch"]):
        axes[ax_i].plot(t, df[f"online_{axis}"], color="black", linewidth=1.1, label=f"online {axis}")
        for estimator in estimators:
            axes[ax_i].plot(t, align_to_online(df, estimator, axis, align_mask), linewidth=0.85, label=estimator)
        axes[ax_i].set_ylabel(f"{axis} deg")
        axes[ax_i].grid(True, alpha=0.25)
        axes[ax_i].legend(loc="upper right", ncol=3, fontsize=8)

    axes[2].plot(t, df["repo_mahony_accel_mag"], linewidth=0.8, label="filtered accel norm")
    axes[2].plot(t, df["repo_mahony_acc_weight"], linewidth=0.8, label="repo accel weight")
    axes[2].plot(t, df["soft_mahony_acc_weight"], linewidth=0.8, label="soft accel weight")
    axes[2].axhline(0.9, color="gray", linewidth=0.6, linestyle="--")
    axes[2].axhline(1.1, color="gray", linewidth=0.6, linestyle="--")
    axes[2].set_ylabel("g / weight")
    axes[2].set_xlabel("time s")
    axes[2].grid(True, alpha=0.25)
    axes[2].legend(loc="upper right", fontsize=8)

    fig.suptitle(f"flight {flight} offline attitude estimators")
    fig.tight_layout()
    fig.savefig(OUT_DIR / f"flight{flight}_estimators.png", dpi=150)
    plt.close(fig)


def markdown_table(df: pd.DataFrame) -> str:
    if df.empty:
        return "_No rows._"
    text_df = df.copy()
    for col in text_df.columns:
        text_df[col] = text_df[col].map(lambda x: "" if pd.isna(x) else str(x))
    widths = [
        max(len(str(col)), *(len(str(row[i])) for row in text_df.values.tolist()))
        for i, col in enumerate(text_df.columns)
    ]
    lines = [
        "| " + " | ".join(str(col).ljust(widths[i]) for i, col in enumerate(text_df.columns)) + " |",
        "| " + " | ".join("-" * widths[i] for i in range(len(text_df.columns))) + " |",
    ]
    for row in text_df.values.tolist():
        lines.append("| " + " | ".join(str(row[i]).ljust(widths[i]) for i in range(len(row))) + " |")
    return "\n".join(lines)


def load_flight(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df = df.rename(columns={v: k for k, v in COLUMNS.items()})
    return df[list(COLUMNS.keys())].copy()


def process_flight(path: Path) -> tuple[list[dict[str, float | int | str]], list[dict[str, float | int | str]], list[dict[str, float | int | str]], dict[str, float | int]]:
    flight = flight_no_from_name(path)
    df = load_flight(path)

    parts = [
        run_mahony(df, REPO_MAHONY),
        run_mahony(df, SOFT_MAHONY),
    ]
    angle_frames = [parts[0][0], parts[1][0], run_madgwick_imu(df), run_complementary_rp(df), run_gyro_only(df)]
    diag_frames = [parts[0][1], parts[1][1]]
    df = pd.concat([df, *angle_frames, *diag_frames], axis=1)

    t = df["t_ms"].to_numpy(float)
    align_mask = t <= t[0] + 500.0
    if int(np.sum(align_mask)) < 10:
        align_mask = np.arange(len(df)) < min(500, len(df))

    estimators = ["repo_mahony", "soft_mahony", "madgwick_imu", "complementary_rp", "gyro_only"]
    rows = []
    for estimator in estimators:
        rows.extend(summarize_estimator(df, flight, estimator, align_mask))
    disagreement = summarize_disagreement(df, flight, estimators, align_mask)
    pairwise = summarize_pairwise(
        df,
        flight,
        [
            ("repo_mahony", "soft_mahony"),
            ("repo_mahony", "madgwick_imu"),
            ("soft_mahony", "madgwick_imu"),
        ],
        align_mask,
    )

    repo_valid = df["repo_mahony_acc_valid"].to_numpy(bool) & (df["repo_mahony_acc_weight"].to_numpy(float) > 0.0)
    soft_valid = df["soft_mahony_acc_valid"].to_numpy(bool) & (df["soft_mahony_acc_weight"].to_numpy(float) > 0.0)
    accel_mag = df["repo_mahony_accel_mag"].to_numpy(float)
    diag = {
        "flight": flight,
        "samples": int(len(df)),
        "duration_s": float((df["t_ms"].iloc[-1] - df["t_ms"].iloc[0]) * 0.001),
        "acc_mag_mean_g": float(np.nanmean(accel_mag)),
        "acc_mag_p05_g": float(np.nanpercentile(accel_mag, 5)),
        "acc_mag_p95_g": float(np.nanpercentile(accel_mag, 95)),
        "repo_acc_used_pct": float(np.mean(repo_valid) * 100.0),
        "soft_acc_used_pct": float(np.mean(soft_valid) * 100.0),
        "repo_longest_no_acc_ms": longest_false_run_ms(repo_valid, t),
        "soft_longest_no_acc_ms": longest_false_run_ms(soft_valid, t),
    }

    compact_cols = ["t_ms", "online_roll", "online_pitch"]
    for estimator in estimators:
        for axis in ["roll", "pitch"]:
            df[f"{estimator}_{axis}_aligned"] = align_to_online(df, estimator, axis, align_mask)
            compact_cols.append(f"{estimator}_{axis}_aligned")
    compact_cols += ["repo_mahony_accel_mag", "repo_mahony_acc_weight", "soft_mahony_acc_weight"]
    df[compact_cols].to_csv(OUT_DIR / f"flight{flight}_offline_angles.csv", index=False, encoding="utf-8-sig")
    plot_flight(df, flight, estimators, align_mask)
    return rows, disagreement, pairwise, diag


def main() -> None:
    OUT_DIR.mkdir(exist_ok=True)
    csv_files = sorted(BASE_DIR.glob("*.csv"), key=flight_no_from_name)
    all_rows: list[dict[str, float | int | str]] = []
    all_disagreement: list[dict[str, float | int | str]] = []
    all_pairwise: list[dict[str, float | int | str]] = []
    all_diag: list[dict[str, float | int]] = []

    for path in csv_files:
        rows, disagreement, pairwise, diag = process_flight(path)
        all_rows.extend(rows)
        all_disagreement.extend(disagreement)
        all_pairwise.extend(pairwise)
        all_diag.append(diag)

    summary = pd.DataFrame(all_rows)
    disagreement = pd.DataFrame(all_disagreement)
    pairwise = pd.DataFrame(all_pairwise)
    diag = pd.DataFrame(all_diag)
    summary.to_csv(OUT_DIR / "estimator_vs_online_summary.csv", index=False, encoding="utf-8-sig")
    disagreement.to_csv(OUT_DIR / "estimator_disagreement_summary.csv", index=False, encoding="utf-8-sig")
    pairwise.to_csv(OUT_DIR / "trusted_estimator_pairwise_summary.csv", index=False, encoding="utf-8-sig")
    diag.to_csv(OUT_DIR / "accel_gate_summary.csv", index=False, encoding="utf-8-sig")

    key = summary.pivot_table(
        index=["estimator", "axis"],
        values=["rms_vs_online_deg", "p95_abs_vs_online_deg", "final_vs_online_deg"],
        aggfunc="mean",
    ).reset_index().round(3)
    key_disagreement = disagreement.pivot_table(
        index=["estimator", "axis"],
        values=["rms_vs_algo_median_deg", "p95_abs_vs_algo_median_deg"],
        aggfunc="mean",
    ).reset_index().round(3)
    key_pairwise = pairwise.pivot_table(
        index=["left", "right", "axis"],
        values=["rms_diff_deg", "p95_abs_diff_deg"],
        aggfunc="mean",
    ).reset_index().round(3)
    diag_round = diag.round(3)
    repo_acc_mean = float(diag["repo_acc_used_pct"].mean())
    soft_acc_mean = float(diag["soft_acc_used_pct"].mean())
    repo_soft_pitch = key_pairwise[
        (key_pairwise["left"] == "repo_mahony")
        & (key_pairwise["right"] == "soft_mahony")
        & (key_pairwise["axis"] == "pitch")
    ].iloc[0]
    soft_madgwick_pitch = key_pairwise[
        (key_pairwise["left"] == "soft_mahony")
        & (key_pairwise["right"] == "madgwick_imu")
        & (key_pairwise["axis"] == "pitch")
    ].iloc[0]

    lines = [
        "# Offline 6-axis attitude estimator comparison",
        "",
        "Input: I7-I12 filtered gyro/accelerometer channels from the six 0531_PID CSV logs.",
        "Reference: I13/I14 online roll/pitch. This is only the firmware output, not ground truth.",
        "All offline estimator curves are offset-aligned to the first 0.5 s of the online roll/pitch before error statistics.",
        "",
        "## Accelerometer gate diagnostics",
        "",
        markdown_table(diag_round),
        "",
        "## Mean error vs online Euler angle across six flights",
        "",
        markdown_table(key),
        "",
        "## Mean disagreement vs estimator median across six flights",
        "",
        markdown_table(key_disagreement),
        "",
        "## Pairwise differences among the three fused quaternion estimators",
        "",
        markdown_table(key_pairwise),
        "",
        "## Findings from these logs",
        "",
        f"- The current repo Mahony gate uses accelerometer correction on average {repo_acc_mean:.1f}% of samples; the soft Mahony variant uses it on {soft_acc_mean:.1f}% of samples.",
        f"- On pitch, repo Mahony differs from soft Mahony by RMS {repo_soft_pitch['rms_diff_deg']:.3f} deg and P95 {repo_soft_pitch['p95_abs_diff_deg']:.3f} deg.",
        f"- On pitch, soft Mahony differs from Madgwick IMU by RMS {soft_madgwick_pitch['rms_diff_deg']:.3f} deg and P95 {soft_madgwick_pitch['p95_abs_diff_deg']:.3f} deg.",
        "- This supports the hypothesis that the hard 0.9g-1.1g gate is too restrictive for these flights; it makes the estimator depend on gyro integration during too much of the flight segment.",
        "- The online Euler angle is produced by the current firmware, so matching it is not proof of correctness. Agreement between independent offline filters is the stronger signal here.",
        "",
        "## Algorithm references",
        "",
        "- Mahony, Hamel, Pflimlin, Nonlinear Complementary Filters on SO(3): https://doi.org/10.1109/TAC.2008.923738",
        "- Madgwick IMU gradient-descent formulation: https://ahrs.readthedocs.io/en/latest/filters/madgwick.html",
        "- x-io Fusion acceleration rejection/recovery notes: https://github.com/xioTechnologies/Fusion",
        "",
        "## Files",
        "",
        "- `estimator_vs_online_summary.csv`: per-flight, per-axis error against online Euler.",
        "- `estimator_disagreement_summary.csv`: per-flight disagreement against the estimator median.",
        "- `trusted_estimator_pairwise_summary.csv`: pairwise differences between repo Mahony, soft Mahony, and Madgwick IMU.",
        "- `accel_gate_summary.csv`: acceleration magnitude and correction gate statistics.",
        "- `flightN_estimators.png`: aligned roll/pitch curves plus acceleration gate traces.",
        "- `flightN_offline_angles.csv`: compact aligned angle traces for external plotting.",
        "",
        "## Interpretation rules",
        "",
        "- If `repo_mahony` diverges from most other algorithms while its accel weight is near zero for long windows, suspect the hard acceleration/rotation gate.",
        "- If all algorithms diverge in the same direction, suspect input axes, calibration, filter delay, or real vehicle attitude rather than only the Mahony correction gate.",
        "- With 6-axis data, yaw is not observable; this report intentionally limits conclusions to roll and pitch.",
    ]
    (OUT_DIR / "offline_attitude_report.md").write_text("\n".join(lines), encoding="utf-8")
    print(OUT_DIR)


if __name__ == "__main__":
    main()
