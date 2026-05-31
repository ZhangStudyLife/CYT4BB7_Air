from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


BASE_DIR = Path(__file__).resolve().parent
OUT_DIR = BASE_DIR / "mahony_tune_quat_ekf"


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
    pairwise = pd.read_csv(OUT_DIR / "cpp_grid_pairwise.csv")
    variants = pairwise[pairwise["variant"].ne("madgwick_vs_ekf")].copy()
    score = variants.pivot_table(
        index=["variant", "kp", "ki", "min_g", "max_g", "band_g", "axis"],
        columns="ref",
        values="rms_diff_deg",
        aggfunc="mean",
    ).reset_index()
    score["axis_score_rms_deg"] = np.sqrt((score["quat_ekf"] ** 2 + score["madgwick_imu"] ** 2) / 2.0)
    total = score.groupby(["variant", "kp", "ki", "min_g", "max_g", "band_g"], as_index=False).agg({
        "axis_score_rms_deg": "mean",
        "quat_ekf": "mean",
        "madgwick_imu": "mean",
    })
    gates = variants.groupby(["variant"], as_index=False).agg({
        "acc_used_pct": "mean",
        "acc_weight_mean": "mean",
    })
    total = total.merge(gates, on="variant", how="left").sort_values("axis_score_rms_deg")
    total.to_csv(OUT_DIR / "cpp_mahony_kpki_score.csv", index=False, encoding="utf-8-sig")

    mad_ekf = pairwise[pairwise["variant"].eq("madgwick_vs_ekf")]
    mad_ekf_key = mad_ekf.pivot_table(
        index="axis",
        values=["rms_diff_deg", "p95_abs_diff_deg"],
        aggfunc="mean",
    ).reset_index().round(4)
    current = total[
        (total["kp"].eq(1.0))
        & (total["ki"].eq(0.02))
        & (total["band_g"].eq(0.35))
        & (total["min_g"].eq(0.30))
        & (total["max_g"].eq(3.00))
    ].head(1)
    bf = total[total["variant"].eq("bf_gate_kp0.25_ki0.00_band0.10")]

    kp_band035 = total[
        total["variant"].str.contains("band0.35")
        & total["min_g"].eq(0.30)
        & total["max_g"].eq(3.00)
    ].copy()
    fig, ax = plt.subplots(figsize=(9, 5))
    for ki, group in kp_band035.groupby("ki"):
        group = group.sort_values("kp")
        ax.plot(group["kp"], group["axis_score_rms_deg"], marker="o", linewidth=1.2, label=f"Ki={ki:g}")
    ax.set_xlabel("Mahony Kp")
    ax.set_ylabel("mean RMS score vs quat EKF + Madgwick (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT_DIR / "cpp_kp_score_curve.png", dpi=150)
    plt.close(fig)

    best_same_gate = total[
        total["min_g"].eq(0.30)
        & total["max_g"].eq(3.00)
        & total["band_g"].eq(0.35)
    ].head(10)

    lines = [
        "# Full Mahony grid search against quaternion EKF",
        "",
        "The grid search is computed by `mahony_ekf_grid_search.cpp` for speed. It runs all six 0531_PID logs at full sample rate.",
        "",
        "Reference filters:",
        "- Quaternion error-state EKF: gyro propagation plus normalized accelerometer gravity-vector correction with soft acceleration rejection.",
        "- Madgwick IMU: independent gradient-descent 6-axis attitude filter.",
        "",
        "Score is the mean roll/pitch RMS difference to both references after first-0.5s offset alignment. Lower is closer to the independent references.",
        "",
        "## Madgwick vs quaternion EKF baseline",
        "",
        markdown_table(mad_ekf_key),
        "",
        "## Best variants overall",
        "",
        markdown_table(total.head(15).round(5)[["variant", "kp", "ki", "min_g", "max_g", "band_g", "axis_score_rms_deg", "quat_ekf", "madgwick_imu", "acc_used_pct", "acc_weight_mean"]]),
        "",
        "## Best variants with current gate shape",
        "",
        markdown_table(best_same_gate.round(5)[["variant", "kp", "ki", "band_g", "axis_score_rms_deg", "quat_ekf", "madgwick_imu", "acc_used_pct", "acc_weight_mean"]]),
        "",
        "## Current firmware parameters",
        "",
        markdown_table(current.round(5)[["variant", "kp", "ki", "band_g", "axis_score_rms_deg", "quat_ekf", "madgwick_imu", "acc_used_pct", "acc_weight_mean"]]),
        "",
        "## Old BF hard-gate parameters",
        "",
        markdown_table(bf.round(5)[["variant", "kp", "ki", "min_g", "max_g", "band_g", "axis_score_rms_deg", "quat_ekf", "madgwick_imu", "acc_used_pct", "acc_weight_mean"]]),
        "",
        "## Recommendation",
        "",
        "- For these six logs, the best Mahony `Kp` is around `0.30-0.45` when using the current wide gate `0.30-3.00g` and `band=0.35g`.",
        "- `Ki` does not materially change the score in flight because the firmware only integrates bias while static-locked. Keep `Ki=0.02` for slow static bias learning or set `0` if you want fewer hidden state changes.",
        "- `Kp=1.0` is usable but more aggressive than the EKF/Madgwick consensus; it scores worse than `0.3-0.5` on these logs.",
        "- Do not return to the old `0.9-1.1g` hard gate. Its score is worse and its accelerometer correction participation is much lower.",
    ]
    (OUT_DIR / "cpp_mahony_tune_report.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
