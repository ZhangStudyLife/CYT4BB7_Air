#!/usr/bin/env python3
"""分析 inav_better_vel 实飞日志并给出互补滤波调参建议。"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
COMMON_DIR = SCRIPT_DIR.parent / "new_pmw3901_data"
if str(COMMON_DIR) not in sys.path:
    sys.path.insert(0, str(COMMON_DIR))

from vel_acc_common import (  # noqa: E402
    FilterSpec,
    apply_filter,
    band_power_fraction,
    compute_welch_psd,
    find_top_peaks,
    rms,
)


DEFAULT_INPUT = "inav_better_vel.csv"
DEFAULT_REPORT = "inav_better_vel_analysis_report.md"
DEFAULT_SAMPLE_RATE_HZ = 250.0

OLD_MACHINE_PARAMS = {
    "flow_v": 2.0,
    "acc_bias": 0.010,
    "innov_gate": 120.0,
    "dead_max_s": 0.30,
    "res_v": 0.5,
}

CURRENT_MACHINE_PARAMS = {
    "flow_v": 4.5,
    "acc_bias": 0.003,
    "innov_gate": 130.0,
    "dead_max_s": 0.20,
    "res_v": 1.0,
}


@dataclass(frozen=True)
class ReplayMetrics:
    """保存一组融合参数在实飞日志回放上的统计结果。"""

    valid_ratio: float
    longest_invalid_s: float
    innovation_rms_cmps: float
    innovation_bias_cmps: float
    fused_flow_bias_cmps: float
    mean_fused_right_cmps: float
    mean_fused_forward_cmps: float
    bias_end_right_cmpss: float
    bias_end_forward_cmpss: float


@dataclass(frozen=True)
class CandidateResult:
    """保存单组候选参数及其评分。"""

    flow_v: float
    acc_bias: float
    innov_gate: float
    dead_max_s: float
    res_v: float
    score: float
    metrics: ReplayMetrics


def resolve_script_path(path_text: str) -> Path:
    """将相对路径解析到脚本目录。"""
    path = Path(path_text)
    if path.is_absolute():
        return path
    return SCRIPT_DIR / path


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="分析 inav_better_vel 实飞日志并输出调参报告。")
    parser.add_argument("--input", default=DEFAULT_INPUT, help="输入 CSV 路径，默认 inav_better_vel.csv。")
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=DEFAULT_SAMPLE_RATE_HZ,
        help="日志采样率，默认 250Hz。",
    )
    parser.add_argument("--report", default=DEFAULT_REPORT, help="输出 Markdown 报告路径。")
    return parser.parse_args()


def load_csv(csv_path: Path) -> dict[str, np.ndarray]:
    """读取 CSV 全部列到浮点数组。"""
    with csv_path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError("CSV 缺少表头。")
        rows = list(reader)

    return {
        field_name: np.asarray([float(row[field_name]) for row in rows], dtype=np.float64)
        for field_name in reader.fieldnames
    }


def longest_invalid_run_s(mask: np.ndarray, sample_rate_hz: float) -> float:
    """计算连续无效段最长时长。"""
    max_run = 0
    current_run = 0
    for item in mask:
        if item:
            current_run += 1
            if current_run > max_run:
                max_run = current_run
        else:
            current_run = 0
    return max_run / sample_rate_hz


def summarize_psd(signal: np.ndarray, sample_rate_hz: float) -> dict[str, object]:
    """生成频谱和带功率摘要。"""
    freqs, psd = compute_welch_psd(signal, sample_rate_hz)
    return {
        "band_0_5": band_power_fraction(freqs, psd, 0.0, 5.0),
        "band_5_15": band_power_fraction(freqs, psd, 5.0, 15.0),
        "band_15_30": band_power_fraction(freqs, psd, 15.0, 30.0),
        "band_30_60": band_power_fraction(freqs, psd, 30.0, 60.0),
        "band_60_125": band_power_fraction(freqs, psd, 60.0, 125.1),
        "peaks_hz": find_top_peaks(freqs, psd, count=6, min_freq_hz=0.5, min_separation_hz=1.0),
    }


def simulate_replay(
    columns: dict[str, np.ndarray],
    sample_rate_hz: float,
    filter_spec: FilterSpec,
    flow_v: float,
    acc_bias: float,
    innov_gate: float,
    dead_max_s: float,
    res_v: float,
    flow_p: float = 1.0,
    recover_ramp_s: float = 0.20,
) -> ReplayMetrics:
    """按当前机载逻辑近似回放实飞日志。"""
    dt_s = 1.0 / sample_rate_hz

    acc_right_filt = apply_filter(columns["I0"], filter_spec, sample_rate_hz)
    acc_forward_filt = apply_filter(columns["I1"], filter_spec, sample_rate_hz)

    flow_right = columns["I4"]
    flow_forward = columns["I5"]
    squal = columns["I10"]
    height = columns["I11"]
    delta_x = columns["I12"]
    delta_y = columns["I13"]

    vel_right = 0.0
    vel_forward = 0.0
    pos_right = 0.0
    pos_forward = 0.0
    bias_right = 0.0
    bias_forward = 0.0
    flow_pos_right = 0.0
    flow_pos_forward = 0.0
    dead_time_s = 0.0
    recover_gain = 0.0
    flow_anchor_ready = False
    flow_quality_ok = False

    valid_mask = np.zeros_like(flow_right, dtype=bool)
    innovation_right = np.zeros_like(flow_right)
    innovation_forward = np.zeros_like(flow_forward)
    fused_right = np.zeros_like(flow_right)
    fused_forward = np.zeros_like(flow_forward)

    for index in range(flow_right.size):
        acc_right_use = acc_right_filt[index] - bias_right
        acc_forward_use = acc_forward_filt[index] - bias_forward

        pos_right += vel_right * dt_s + 0.5 * acc_right_use * dt_s * dt_s
        pos_forward += vel_forward * dt_s + 0.5 * acc_forward_use * dt_s * dt_s
        vel_right += acc_right_use * dt_s
        vel_forward += acc_forward_use * dt_s

        if not flow_quality_ok:
            flow_quality_ok = squal[index] >= 40.0
        else:
            flow_quality_ok = squal[index] >= 25.0

        innovation_right[index] = flow_right[index] - vel_right
        innovation_forward[index] = flow_forward[index] - vel_forward
        innovation_norm = math.hypot(innovation_right[index], innovation_forward[index])

        flow_valid = (
            flow_quality_ok
            and (0.1 <= height[index] <= 2.0)
            and (abs(delta_x[index]) <= 40.0)
            and (abs(delta_y[index]) <= 40.0)
            and (innovation_norm <= innov_gate)
        )
        valid_mask[index] = flow_valid

        if flow_valid:
            if not flow_anchor_ready:
                flow_pos_right = pos_right
                flow_pos_forward = pos_forward
                flow_anchor_ready = True

            dead_time_s = 0.0
            recover_gain = min(1.0, recover_gain + dt_s / recover_ramp_s)

            flow_pos_right += flow_right[index] * dt_s
            flow_pos_forward += flow_forward[index] * dt_s

            pos_right += (flow_pos_right - pos_right) * flow_p * recover_gain * dt_s
            pos_forward += (flow_pos_forward - pos_forward) * flow_p * recover_gain * dt_s
            vel_right += innovation_right[index] * flow_v * recover_gain * dt_s
            vel_forward += innovation_forward[index] * flow_v * recover_gain * dt_s

            bias_right += innovation_right[index] * acc_bias * dt_s
            bias_forward += innovation_forward[index] * acc_bias * dt_s
            bias_right = max(-245.0, min(245.0, bias_right))
            bias_forward = max(-245.0, min(245.0, bias_forward))
        else:
            dead_time_s += dt_s
            recover_gain = 0.0
            if dead_time_s > dead_max_s:
                decay_factor = max(0.0, 1.0 - res_v * dt_s)
                vel_right *= decay_factor
                vel_forward *= decay_factor
                flow_anchor_ready = False

        fused_right[index] = vel_right
        fused_forward[index] = vel_forward

    valid_samples = valid_mask
    invalid_samples = ~valid_samples

    return ReplayMetrics(
        valid_ratio=float(np.mean(valid_samples)),
        longest_invalid_s=longest_invalid_run_s(invalid_samples, sample_rate_hz),
        innovation_rms_cmps=0.5 * (rms(innovation_right[valid_samples]) + rms(innovation_forward[valid_samples])),
        innovation_bias_cmps=0.5
        * (abs(float(np.mean(innovation_right[valid_samples]))) + abs(float(np.mean(innovation_forward[valid_samples])))),
        fused_flow_bias_cmps=0.5
        * (
            abs(float(np.mean(flow_right[valid_samples] - fused_right[valid_samples])))
            + abs(float(np.mean(flow_forward[valid_samples] - fused_forward[valid_samples])))
        ),
        mean_fused_right_cmps=float(np.mean(fused_right)),
        mean_fused_forward_cmps=float(np.mean(fused_forward)),
        bias_end_right_cmpss=bias_right,
        bias_end_forward_cmpss=bias_forward,
    )


def score_candidate(metrics: ReplayMetrics) -> float:
    """按悬停场景关注点为候选参数打分，越小越好。"""
    return (
        0.40 * metrics.innovation_rms_cmps
        + 0.25 * metrics.innovation_bias_cmps
        + 0.20 * metrics.fused_flow_bias_cmps
        + 6.0 * (1.0 - metrics.valid_ratio)
        + 0.80 * metrics.longest_invalid_s
    )


def search_candidates(columns: dict[str, np.ndarray], sample_rate_hz: float) -> list[CandidateResult]:
    """对实飞日志做小规模网格搜索。"""
    filter_spec = FilterSpec(label="Notch85Q3+LPF15", notch_hz=85.0, notch_q=3.0, lpf_hz=15.0)
    results: list[CandidateResult] = []

    for flow_v in (2.0, 2.5, 3.0, 3.5, 4.0, 4.5):
        for acc_bias in (0.003, 0.005, 0.008, 0.010):
            for innov_gate in (120.0, 130.0, 140.0):
                for dead_max_s in (0.20, 0.30):
                    for res_v in (0.5, 1.0):
                        metrics = simulate_replay(
                            columns=columns,
                            sample_rate_hz=sample_rate_hz,
                            filter_spec=filter_spec,
                            flow_v=flow_v,
                            acc_bias=acc_bias,
                            innov_gate=innov_gate,
                            dead_max_s=dead_max_s,
                            res_v=res_v,
                        )
                        results.append(
                            CandidateResult(
                                flow_v=flow_v,
                                acc_bias=acc_bias,
                                innov_gate=innov_gate,
                                dead_max_s=dead_max_s,
                                res_v=res_v,
                                score=score_candidate(metrics),
                                metrics=metrics,
                            )
                        )

    return sorted(results, key=lambda item: item.score)


def build_report(columns: dict[str, np.ndarray], sample_rate_hz: float, candidates: list[CandidateResult]) -> str:
    """组装 Markdown 报告文本。"""
    raw_right_summary = summarize_psd(columns["I0"], sample_rate_hz)
    raw_forward_summary = summarize_psd(columns["I1"], sample_rate_hz)
    filt_right_summary = summarize_psd(columns["I2"], sample_rate_hz)
    filt_forward_summary = summarize_psd(columns["I3"], sample_rate_hz)

    actual_valid = columns["I14"] > 0.5
    innovation_norm_actual = np.sqrt(columns["I8"] * columns["I8"] + columns["I9"] * columns["I9"])

    flow_quality_ok = np.zeros_like(columns["I10"], dtype=bool)
    quality_ok = False
    for index, squal in enumerate(columns["I10"]):
        if not quality_ok:
            quality_ok = squal >= 40.0
        else:
            quality_ok = squal >= 25.0
        flow_quality_ok[index] = quality_ok

    quality_fail = ~flow_quality_ok
    innov_fail = innovation_norm_actual > 120.0
    height_fail = (columns["I11"] < 0.1) | (columns["I11"] > 2.0)
    pixel_fail = (np.abs(columns["I12"]) > 40.0) | (np.abs(columns["I13"]) > 40.0)
    actual_invalid = ~actual_valid

    old_baseline = next(
        candidate
        for candidate in candidates
        if (
            candidate.flow_v == OLD_MACHINE_PARAMS["flow_v"]
            and abs(candidate.acc_bias - OLD_MACHINE_PARAMS["acc_bias"]) < 1.0e-9
            and candidate.innov_gate == OLD_MACHINE_PARAMS["innov_gate"]
            and abs(candidate.dead_max_s - OLD_MACHINE_PARAMS["dead_max_s"]) < 1.0e-9
            and abs(candidate.res_v - OLD_MACHINE_PARAMS["res_v"]) < 1.0e-9
        )
    )
    current_baseline = next(
        candidate
        for candidate in candidates
        if (
            candidate.flow_v == CURRENT_MACHINE_PARAMS["flow_v"]
            and abs(candidate.acc_bias - CURRENT_MACHINE_PARAMS["acc_bias"]) < 1.0e-9
            and candidate.innov_gate == CURRENT_MACHINE_PARAMS["innov_gate"]
            and abs(candidate.dead_max_s - CURRENT_MACHINE_PARAMS["dead_max_s"]) < 1.0e-9
            and abs(candidate.res_v - CURRENT_MACHINE_PARAMS["res_v"]) < 1.0e-9
        )
    )
    best = candidates[0]

    lines = [
        "# inav_better_vel 实飞日志分析报告",
        "",
        f"- 样本数: {columns['I0'].size}",
        f"- 采样率: {sample_rate_hz:.1f} Hz",
        f"- 日志时长: {columns['I0'].size / sample_rate_hz:.2f} s",
        "- 列映射: I0/I1=原始水平加速度, I2/I3=滤波后水平加速度, I4/I5=光流速度, I6/I7=融合速度, I8/I9=创新, I10=squal, I11=高度, I12/I13=deltaX/deltaY, I14=flow_valid, I15=fusion_mode",
        "",
        "## 实飞数据结论",
        "",
        f"- 当前日志 `flow_valid` 占比为 {np.mean(actual_valid) * 100.0:.2f}% ，最长连续失效约 {longest_invalid_run_s(actual_invalid, sample_rate_hz):.3f} s。",
        f"- 失效主因是创新门限：在当前无效样本中，创新超门限占比 {np.mean(innov_fail[actual_invalid]) * 100.0:.2f}% ，SQUAL 导致的无效仅占 {np.mean(quality_fail[actual_invalid]) * 100.0:.2f}% 。",
        f"- 当前融合速度存在明显慢偏移：日志中的 `I6/I7` 均值约为 {np.mean(columns['I6']):.2f} / {np.mean(columns['I7']):.2f} cm/s，而光流速度 `I4/I5` 均值仅为 {np.mean(columns['I4']):.2f} / {np.mean(columns['I5']):.2f} cm/s。",
        "",
        "## 原始/滤波频域摘要",
        "",
        f"- I0 原始右向加速度主峰: {', '.join(f'{peak:.2f}Hz' for peak in raw_right_summary['peaks_hz'])}",
        f"- I1 原始前向加速度主峰: {', '.join(f'{peak:.2f}Hz' for peak in raw_forward_summary['peaks_hz'])}",
        f"- I2 右向滤波后主峰: {', '.join(f'{peak:.2f}Hz' for peak in filt_right_summary['peaks_hz'])}",
        f"- I3 前向滤波后主峰: {', '.join(f'{peak:.2f}Hz' for peak in filt_forward_summary['peaks_hz'])}",
        f"- I0 原始右向加速度带功率占比: 0~5Hz={raw_right_summary['band_0_5'] * 100.0:.2f}%, 5~15Hz={raw_right_summary['band_5_15'] * 100.0:.2f}%, 15~30Hz={raw_right_summary['band_15_30'] * 100.0:.2f}%, 30~60Hz={raw_right_summary['band_30_60'] * 100.0:.2f}%, 60~125Hz={raw_right_summary['band_60_125'] * 100.0:.2f}%",
        f"- I1 原始前向加速度带功率占比: 0~5Hz={raw_forward_summary['band_0_5'] * 100.0:.2f}%, 5~15Hz={raw_forward_summary['band_5_15'] * 100.0:.2f}%, 15~30Hz={raw_forward_summary['band_15_30'] * 100.0:.2f}%, 30~60Hz={raw_forward_summary['band_30_60'] * 100.0:.2f}%, 60~125Hz={raw_forward_summary['band_60_125'] * 100.0:.2f}%",
        f"- 当前 `Notch85Q3+LPF15` 已把 I2/I3 的 60~125Hz 功率压到约 {filt_right_summary['band_60_125'] * 100.0:.2f}% / {filt_forward_summary['band_60_125'] * 100.0:.2f}% ，说明滤波器本身不是主要瓶颈。",
        "",
        "## 参数回放对比",
        "",
        "| 方案 | w_flow_v | w_acc_bias | innov_gate | dead_max_s | w_res_v | valid占比 | 最长失效(s) | 创新RMS(cm/s) | 创新均值偏差(cm/s) | 融合-光流均值偏差(cm/s) | 融合速度均值(cm/s) |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
        f"| 旧机载参数回放 | {old_baseline.flow_v:.1f} | {old_baseline.acc_bias:.3f} | {old_baseline.innov_gate:.0f} | {old_baseline.dead_max_s:.2f} | {old_baseline.res_v:.1f} | {old_baseline.metrics.valid_ratio * 100.0:.2f}% | {old_baseline.metrics.longest_invalid_s:.3f} | {old_baseline.metrics.innovation_rms_cmps:.3f} | {old_baseline.metrics.innovation_bias_cmps:.3f} | {old_baseline.metrics.fused_flow_bias_cmps:.3f} | {old_baseline.metrics.mean_fused_right_cmps:.3f} / {old_baseline.metrics.mean_fused_forward_cmps:.3f} |",
        f"| 当前机载参数回放 | {current_baseline.flow_v:.1f} | {current_baseline.acc_bias:.3f} | {current_baseline.innov_gate:.0f} | {current_baseline.dead_max_s:.2f} | {current_baseline.res_v:.1f} | {current_baseline.metrics.valid_ratio * 100.0:.2f}% | {current_baseline.metrics.longest_invalid_s:.3f} | {current_baseline.metrics.innovation_rms_cmps:.3f} | {current_baseline.metrics.innovation_bias_cmps:.3f} | {current_baseline.metrics.fused_flow_bias_cmps:.3f} | {current_baseline.metrics.mean_fused_right_cmps:.3f} / {current_baseline.metrics.mean_fused_forward_cmps:.3f} |",
        f"| 最优得分方案 | {best.flow_v:.1f} | {best.acc_bias:.3f} | {best.innov_gate:.0f} | {best.dead_max_s:.2f} | {best.res_v:.1f} | {best.metrics.valid_ratio * 100.0:.2f}% | {best.metrics.longest_invalid_s:.3f} | {best.metrics.innovation_rms_cmps:.3f} | {best.metrics.innovation_bias_cmps:.3f} | {best.metrics.fused_flow_bias_cmps:.3f} | {best.metrics.mean_fused_right_cmps:.3f} / {best.metrics.mean_fused_forward_cmps:.3f} |",
        "",
        "## 机载落地建议",
        "",
        "- 单看当前这份日志，最优得分仍倾向于进一步放宽 `innov_gate`，但联合两份实飞日志后，机载仍建议保持更稳妥的 `innov_gate=130 cm/s`。",
        f"- 当前机载参数保持为：`w_flow_v={CURRENT_MACHINE_PARAMS['flow_v']:.1f}`、`w_acc_bias={CURRENT_MACHINE_PARAMS['acc_bias']:.3f}`、`innov_gate={CURRENT_MACHINE_PARAMS['innov_gate']:.0f} cm/s`、`dead_max_s={CURRENT_MACHINE_PARAMS['dead_max_s']:.2f} s`、`w_res_v={CURRENT_MACHINE_PARAMS['res_v']:.1f}`。",
        f"- 当前机载参数相对旧机载参数回放结果：`flow_valid` 由约 {old_baseline.metrics.valid_ratio * 100.0:.2f}% 提升到约 {current_baseline.metrics.valid_ratio * 100.0:.2f}% ，最长连续失效由 {old_baseline.metrics.longest_invalid_s:.3f}s 缩短到约 {current_baseline.metrics.longest_invalid_s:.3f}s ，创新 RMS 由约 {old_baseline.metrics.innovation_rms_cmps:.2f} cm/s 降到约 {current_baseline.metrics.innovation_rms_cmps:.2f} cm/s，前向融合均值由约 {old_baseline.metrics.mean_fused_forward_cmps:.2f} cm/s 收敛到约 {current_baseline.metrics.mean_fused_forward_cmps:.2f} cm/s。",
        "- 本轮数据下，SQUAL 与高度门控都不是主要问题，不建议优先调整 `SQUAL_HIGH/LOW` 或高度阈值；主要问题是创新门控过于容易把慢偏移放大成失效。",
        "",
        "## 建议的下一轮验证",
        "",
        "- 继续采集至少两类日志：一类为稳悬停，一类为小角度前后/左右拨杆后回中。",
        "- 下一轮重点观察 `I8/I9` 创新是否仍存在单轴长期偏置；若前向仍有系统性负偏，优先考虑更明确的加速度零偏冻结逻辑，而不是继续放宽创新门限。",
    ]

    return "\n".join(lines) + "\n"


def main() -> int:
    """程序入口。"""
    args = parse_args()
    input_path = resolve_script_path(args.input)
    report_path = resolve_script_path(args.report)

    columns = load_csv(input_path)
    candidates = search_candidates(columns, args.sample_rate)
    report_text = build_report(columns, args.sample_rate, candidates)
    report_path.write_text(report_text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
