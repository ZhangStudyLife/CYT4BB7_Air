#!/usr/bin/env python3
"""对比互补滤波与轻量 EKF 的离线回放表现。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from vel_acc_common import (
    SAMPLE_RATE_HZ,
    FilterSpec,
    load_better_vel_acc,
    replay_complementary_filter,
    replay_ekf,
    rms,
    settle_time_s,
)


SCRIPT_DIR = Path(__file__).resolve().parent


def resolve_script_path(path_text: str) -> Path:
    """将默认相对路径解析到脚本目录，避免依赖当前工作目录。"""
    path = Path(path_text)
    if path.is_absolute():
        return path
    return SCRIPT_DIR / path


@dataclass(frozen=True)
class Scenario:
    """单个故障注入场景。"""

    name: str
    kind: str
    duration_s: float


@dataclass(frozen=True)
class ScenarioMetric:
    """算法在单场景下的指标。"""

    algorithm: str
    scenario: str
    peak_deviation_cmps: float
    settle_time_s: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="比较互补滤波与轻量 EKF 的离线回放表现。")
    parser.add_argument("--input", default="better_vel_acc.csv", help="输入 CSV 文件路径。")
    parser.add_argument("--sample-rate", type=float, default=SAMPLE_RATE_HZ, help="采样频率，默认 250Hz。")
    parser.add_argument("--output-dir", default="cf_ekf_compare_better_vel_acc", help="输出目录。")
    parser.add_argument("--lpf-hz", type=float, default=15.0, help="低通截止频率，默认 15Hz。")
    parser.add_argument("--notch-hz", type=float, default=85.0, help="陷波中心频率，默认 85Hz。")
    parser.add_argument("--notch-q", type=float, default=3.0, help="陷波 Q 值，默认 3。")
    return parser.parse_args()


def build_fault_mask(sample_count: int, sample_rate_hz: float, duration_s: float, center_ratio: float = 0.5) -> tuple[np.ndarray, slice]:
    duration_samples = max(1, int(round(duration_s * sample_rate_hz)))
    start = max(0, int(round(sample_count * center_ratio)) - duration_samples // 2)
    end = min(sample_count, start + duration_samples)
    valid_mask = np.ones(sample_count, dtype=bool)
    valid_mask[start:end] = False
    return valid_mask, slice(start, end)


def inject_fault(
    flow_right_cmps: np.ndarray,
    flow_forward_cmps: np.ndarray,
    sample_rate_hz: float,
    scenario: Scenario,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, slice]:
    right = np.asarray(flow_right_cmps, dtype=np.float64).copy()
    forward = np.asarray(flow_forward_cmps, dtype=np.float64).copy()
    valid_mask = np.ones_like(right, dtype=bool)

    if scenario.kind == "dropout":
        valid_mask, fault_window = build_fault_mask(right.size, sample_rate_hz, scenario.duration_s)
        return right, forward, valid_mask, fault_window

    if scenario.kind == "freeze":
        _, fault_window = build_fault_mask(right.size, sample_rate_hz, scenario.duration_s)
        start = fault_window.start
        end = fault_window.stop
        valid_mask = np.ones_like(right, dtype=bool)
        if start > 0:
            right[start:end] = right[start - 1]
            forward[start:end] = forward[start - 1]
        return right, forward, valid_mask, fault_window

    if scenario.kind == "spike":
        valid_mask = np.ones_like(right, dtype=bool)
        center = right.size // 2
        right[center] += 150.0
        forward[center] -= 150.0
        return right, forward, valid_mask, slice(center, center + 1)

    raise ValueError(f"未知场景类型: {scenario.kind}")


def compare_against_baseline(
    baseline_right_cmps: np.ndarray,
    baseline_forward_cmps: np.ndarray,
    fault_right_cmps: np.ndarray,
    fault_forward_cmps: np.ndarray,
    sample_rate_hz: float,
    fault_window: slice,
) -> tuple[float, float]:
    diff_right = fault_right_cmps - baseline_right_cmps
    diff_forward = fault_forward_cmps - baseline_forward_cmps
    peak_deviation = float(np.max(np.sqrt(diff_right * diff_right + diff_forward * diff_forward)))
    settle = settle_time_s(
        diff_right_cmps=diff_right,
        diff_forward_cmps=diff_forward,
        sample_rate_hz=sample_rate_hz,
        start_index=fault_window.stop,
    )
    return peak_deviation, settle


def write_report(
    output_dir: Path,
    innovation_cf_cmps: float,
    innovation_ekf_cmps: float,
    scenario_metrics: list[ScenarioMetric],
) -> None:
    lines = [
        "# 互补滤波 / EKF 回放对比报告",
        "",
        "## 无故障基线",
        "",
        f"- 互补滤波速度新息 RMS: {innovation_cf_cmps:.4f} cm/s",
        f"- EKF 速度新息 RMS: {innovation_ekf_cmps:.4f} cm/s",
        "",
        "## 故障场景",
        "",
        "| 算法 | 场景 | 峰值偏差(cm/s) | 恢复时间(s) |",
        "| --- | --- | ---: | ---: |",
    ]

    for metric in scenario_metrics:
        lines.append(
            f"| {metric.algorithm} | {metric.scenario} | {metric.peak_deviation_cmps:.4f} | {metric.settle_time_s:.4f} |"
        )

    cf_metrics = [metric for metric in scenario_metrics if metric.algorithm == "CF"]
    ekf_metrics = [metric for metric in scenario_metrics if metric.algorithm == "EKF"]

    cf_peak_mean = float(np.mean([metric.peak_deviation_cmps for metric in cf_metrics]))
    ekf_peak_mean = float(np.mean([metric.peak_deviation_cmps for metric in ekf_metrics]))
    cf_settle_mean = float(np.mean([metric.settle_time_s for metric in cf_metrics]))
    ekf_settle_mean = float(np.mean([metric.settle_time_s for metric in ekf_metrics]))

    lines.extend(
        [
            "",
            "## 结论",
            "",
            f"- 互补滤波平均峰值偏差: {cf_peak_mean:.4f} cm/s，EKF 平均峰值偏差: {ekf_peak_mean:.4f} cm/s。",
            f"- 互补滤波平均恢复时间: {cf_settle_mean:.4f} s，EKF 平均恢复时间: {ekf_settle_mean:.4f} s。",
            "- 本轮回放优先关注低纹理退化下的可恢复性，不将 EKF 直接上机，只保留为离线对比基线。",
        ]
    )

    (output_dir / "cf_ekf_compare_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def render_plot(
    output_dir: Path,
    time_s: np.ndarray,
    baseline_cf_right: np.ndarray,
    fault_cf_right: np.ndarray,
    baseline_ekf_right: np.ndarray,
    fault_ekf_right: np.ndarray,
    fault_window: slice,
    title: str,
    filename: str,
) -> None:
    figure, axis = plt.subplots(figsize=(12, 5))
    axis.plot(time_s, baseline_cf_right, label="CF baseline", linewidth=1.0)
    axis.plot(time_s, fault_cf_right, label="CF fault", linewidth=1.0)
    axis.plot(time_s, baseline_ekf_right, label="EKF baseline", linewidth=1.0, alpha=0.8)
    axis.plot(time_s, fault_ekf_right, label="EKF fault", linewidth=1.0, alpha=0.8)
    axis.axvspan(time_s[fault_window.start], time_s[fault_window.stop - 1], color="tab:red", alpha=0.15, label="fault window")
    axis.set_title(title)
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Velocity (cm/s)")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_dir / filename, dpi=180)
    plt.close(figure)


def main() -> int:
    args = parse_args()
    output_dir = resolve_script_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    dataset = load_better_vel_acc(resolve_script_path(args.input), sample_rate_hz=args.sample_rate)
    filter_spec = FilterSpec(
        label=f"Notch{args.notch_hz:.0f}Q{args.notch_q:.0f}+LPF{args.lpf_hz:.0f}",
        notch_hz=args.notch_hz,
        notch_q=args.notch_q,
        lpf_hz=args.lpf_hz,
    )

    baseline_cf = replay_complementary_filter(
        acc_right_mps2=dataset.acc_right_mps2,
        acc_forward_mps2=dataset.acc_forward_mps2,
        flow_right_cmps=dataset.flow_right_cmps,
        flow_forward_cmps=dataset.flow_forward_cmps,
        filter_spec=filter_spec,
        sample_rate_hz=args.sample_rate,
    )
    baseline_ekf = replay_ekf(
        acc_right_mps2=dataset.acc_right_mps2,
        acc_forward_mps2=dataset.acc_forward_mps2,
        flow_right_cmps=dataset.flow_right_cmps,
        flow_forward_cmps=dataset.flow_forward_cmps,
        filter_spec=filter_spec,
        sample_rate_hz=args.sample_rate,
    )

    innovation_cf_cmps = 0.5 * (
        rms(baseline_cf.innovation_right_cmps) + rms(baseline_cf.innovation_forward_cmps)
    )
    innovation_ekf_cmps = 0.5 * (
        rms(baseline_ekf.innovation_right_cmps) + rms(baseline_ekf.innovation_forward_cmps)
    )

    scenarios = [
        Scenario(name="dropout_100ms", kind="dropout", duration_s=0.10),
        Scenario(name="dropout_200ms", kind="dropout", duration_s=0.20),
        Scenario(name="dropout_300ms", kind="dropout", duration_s=0.30),
        Scenario(name="freeze_200ms", kind="freeze", duration_s=0.20),
        Scenario(name="single_spike", kind="spike", duration_s=0.004),
    ]

    scenario_metrics: list[ScenarioMetric] = []

    for scenario in scenarios:
        fault_right, fault_forward, valid_mask, fault_window = inject_fault(
            dataset.flow_right_cmps,
            dataset.flow_forward_cmps,
            args.sample_rate,
            scenario,
        )

        fault_cf = replay_complementary_filter(
            acc_right_mps2=dataset.acc_right_mps2,
            acc_forward_mps2=dataset.acc_forward_mps2,
            flow_right_cmps=fault_right,
            flow_forward_cmps=fault_forward,
            filter_spec=filter_spec,
            sample_rate_hz=args.sample_rate,
            flow_valid_mask=valid_mask,
        )
        fault_ekf = replay_ekf(
            acc_right_mps2=dataset.acc_right_mps2,
            acc_forward_mps2=dataset.acc_forward_mps2,
            flow_right_cmps=fault_right,
            flow_forward_cmps=fault_forward,
            filter_spec=filter_spec,
            sample_rate_hz=args.sample_rate,
            flow_valid_mask=valid_mask,
        )

        cf_peak, cf_settle = compare_against_baseline(
            baseline_cf.vel_right_cmps,
            baseline_cf.vel_forward_cmps,
            fault_cf.vel_right_cmps,
            fault_cf.vel_forward_cmps,
            args.sample_rate,
            fault_window,
        )
        ekf_peak, ekf_settle = compare_against_baseline(
            baseline_ekf.vel_right_cmps,
            baseline_ekf.vel_forward_cmps,
            fault_ekf.vel_right_cmps,
            fault_ekf.vel_forward_cmps,
            args.sample_rate,
            fault_window,
        )

        scenario_metrics.append(
            ScenarioMetric("CF", scenario.name, cf_peak, cf_settle)
        )
        scenario_metrics.append(
            ScenarioMetric("EKF", scenario.name, ekf_peak, ekf_settle)
        )

        if scenario.name in {"dropout_300ms", "freeze_200ms"}:
            render_plot(
                output_dir=output_dir,
                time_s=dataset.time_s,
                baseline_cf_right=baseline_cf.vel_right_cmps,
                fault_cf_right=fault_cf.vel_right_cmps,
                baseline_ekf_right=baseline_ekf.vel_right_cmps,
                fault_ekf_right=fault_ekf.vel_right_cmps,
                fault_window=fault_window,
                title=f"{scenario.name} right-axis replay",
                filename=f"{scenario.name}_right_axis.png",
            )

    write_report(output_dir, innovation_cf_cmps, innovation_ekf_cmps, scenario_metrics)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
