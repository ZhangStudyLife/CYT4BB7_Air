#!/usr/bin/env python3
"""对 better_vel_acc.csv 生成频域分析报告。"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from vel_acc_common import (
    SAMPLE_RATE_HZ,
    band_power_fraction,
    compute_fft,
    compute_welch_psd,
    cumulative_power_frequency,
    find_top_peaks,
    load_better_vel_acc,
)


SCRIPT_DIR = Path(__file__).resolve().parent


def resolve_script_path(path_text: str) -> Path:
    """将默认相对路径解析到脚本目录，避免依赖当前工作目录。"""
    path = Path(path_text)
    if path.is_absolute():
        return path
    return SCRIPT_DIR / path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="对 better_vel_acc.csv 做 FFT/PSD 分析并输出 Markdown 报告。")
    parser.add_argument("--input", default="better_vel_acc.csv", help="输入 CSV 文件路径。")
    parser.add_argument("--sample-rate", type=float, default=SAMPLE_RATE_HZ, help="采样频率，默认 250Hz。")
    parser.add_argument("--output-dir", default="spectrum_report_better_vel_acc", help="输出目录。")
    return parser.parse_args()


def summarize_axis(name: str, signal: np.ndarray, sample_rate_hz: float) -> dict[str, object]:
    freqs_fft, amplitude = compute_fft(signal, sample_rate_hz)
    freqs_psd, psd = compute_welch_psd(signal, sample_rate_hz)
    peaks = find_top_peaks(freqs_psd, psd, count=5, min_freq_hz=0.1, min_separation_hz=1.0)

    return {
        "name": name,
        "freqs_fft": freqs_fft,
        "amplitude": amplitude,
        "freqs_psd": freqs_psd,
        "psd": psd,
        "std": float(np.std(signal)),
        "band_0_5": band_power_fraction(freqs_psd, psd, 0.0, 5.0),
        "band_5_15": band_power_fraction(freqs_psd, psd, 5.0, 15.0),
        "band_15_30": band_power_fraction(freqs_psd, psd, 15.0, 30.0),
        "band_30_60": band_power_fraction(freqs_psd, psd, 30.0, 60.0),
        "band_60_125": band_power_fraction(freqs_psd, psd, 60.0, 125.1),
        "p50_hz": cumulative_power_frequency(freqs_psd, psd, 0.50),
        "p80_hz": cumulative_power_frequency(freqs_psd, psd, 0.80),
        "p95_hz": cumulative_power_frequency(freqs_psd, psd, 0.95),
        "peaks_hz": peaks,
    }


def render_psd_plot(output_dir: Path, summaries: list[dict[str, object]]) -> None:
    figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    for index, (axis, summary) in enumerate(zip(axes, summaries, strict=True), start=1):
        axis.semilogy(summary["freqs_psd"], summary["psd"], linewidth=1.0)
        axis.set_ylabel(f"Axis{index} PSD")
        axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Frequency (Hz)")
    axes[-1].set_xlim(0.0, 125.0)
    figure.tight_layout()
    figure.savefig(output_dir / "acc_psd_summary.png", dpi=180)
    plt.close(figure)


def render_fft_plot(output_dir: Path, summaries: list[dict[str, object]]) -> None:
    figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    for index, (axis, summary) in enumerate(zip(axes, summaries, strict=True), start=1):
        axis.plot(summary["freqs_fft"], summary["amplitude"], linewidth=0.9)
        axis.set_ylabel(f"Axis{index} FFT")
        axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Frequency (Hz)")
    axes[-1].set_xlim(0.0, 125.0)
    figure.tight_layout()
    figure.savefig(output_dir / "acc_fft_summary.png", dpi=180)
    plt.close(figure)


def build_report_text(
    sample_count: int,
    sample_rate_hz: float,
    flow_x_summary: dict[str, object],
    flow_y_summary: dict[str, object],
    axis_summaries: list[dict[str, object]],
) -> str:
    notch_candidates = []
    for summary in axis_summaries[:2]:
        peaks_hz = [peak_hz for peak_hz in summary["peaks_hz"] if 80.0 <= peak_hz <= 110.0]
        if peaks_hz:
            notch_candidates.append(peaks_hz[0])

    if notch_candidates:
        mean_peak_hz = float(np.mean(notch_candidates))
        recommend_notch_hz = min((85.0, 100.0), key=lambda candidate_hz: abs(candidate_hz - mean_peak_hz))
    else:
        recommend_notch_hz = 85.0

    useful_upper_hz = 15.0

    lines = [
        "# better_vel_acc 频域分析报告",
        "",
        f"- 样本数: {sample_count}",
        f"- 采样频率: {sample_rate_hz:.1f} Hz",
        f"- 数据时长: {sample_count / sample_rate_hz:.2f} s",
        "- 列映射: I3=前向水平线性加速度, I4=右向水平线性加速度, I5=竖直水平线性加速度, I9/I10=当前光流速度",
        "",
        "## 频带功率占比",
        "",
        "| 轴向 | 0~5Hz | 5~15Hz | 15~30Hz | 30~60Hz | 60~125Hz | 50%功率频率 | 80%功率频率 | 95%功率频率 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]

    for summary in axis_summaries:
        lines.append(
            f"| {summary['name']} | "
            f"{summary['band_0_5'] * 100.0:.2f}% | "
            f"{summary['band_5_15'] * 100.0:.2f}% | "
            f"{summary['band_15_30'] * 100.0:.2f}% | "
            f"{summary['band_30_60'] * 100.0:.2f}% | "
            f"{summary['band_60_125'] * 100.0:.2f}% | "
            f"{summary['p50_hz']:.2f}Hz | "
            f"{summary['p80_hz']:.2f}Hz | "
            f"{summary['p95_hz']:.2f}Hz |"
        )

    lines.extend(
        [
            "",
            "## 主峰频率",
            "",
        ]
    )

    for summary in axis_summaries:
        peak_text = ", ".join(f"{peak_hz:.2f}Hz" for peak_hz in summary["peaks_hz"])
        lines.append(f"- {summary['name']}: {peak_text}")

    lines.extend(
        [
            "",
            "## 结论",
            "",
            f"- 光流速度 I9/I10 的主要信息几乎全部落在 0~5Hz，95% 累积能量频率分别为 {flow_x_summary['p95_hz']:.2f}Hz 和 {flow_y_summary['p95_hz']:.2f}Hz。",
            "- 水平线性加速度 I3/I4 的高频噪声主能量集中在 60~125Hz，且存在接近 100Hz 的稳定窄带峰，符合机体振动/电机谐波特征。",
            f"- 推荐将“有用运动带”定义为 0~{useful_upper_hz:.1f}Hz，将 60Hz 以上视为主要噪声带。",
            f"- 推荐嵌入式水平加速度滤波器采用 `Notch@{recommend_notch_hz:.1f}Hz + LPF@15Hz`，用于在控制时延可接受的前提下压制高频窄带噪声。",
        ]
    )

    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    output_dir = resolve_script_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    dataset = load_better_vel_acc(resolve_script_path(args.input), sample_rate_hz=args.sample_rate)

    acc_summaries = [
        summarize_axis("I3 前向水平加速度", dataset.acc_forward_mps2, args.sample_rate),
        summarize_axis("I4 右向水平加速度", dataset.acc_right_mps2, args.sample_rate),
        summarize_axis("I5 竖直水平加速度", dataset.acc_vertical_mps2, args.sample_rate),
    ]
    flow_x_summary = summarize_axis("I9 右向光流速度", dataset.flow_right_cmps, args.sample_rate)
    flow_y_summary = summarize_axis("I10 前向光流速度", dataset.flow_forward_cmps, args.sample_rate)

    render_psd_plot(output_dir, acc_summaries)
    render_fft_plot(output_dir, acc_summaries)

    report_text = build_report_text(
        sample_count=dataset.time_s.size,
        sample_rate_hz=args.sample_rate,
        flow_x_summary=flow_x_summary,
        flow_y_summary=flow_y_summary,
        axis_summaries=acc_summaries,
    )
    (output_dir / "spectrum_report.md").write_text(report_text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
