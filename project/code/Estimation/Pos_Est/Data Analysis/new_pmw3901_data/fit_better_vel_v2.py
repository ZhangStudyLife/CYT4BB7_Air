#!/usr/bin/env python3
"""对 better_vel_V2.csv 执行低延迟消抖拟合并输出融合效果图。"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 默认输入文件名。
DEFAULT_INPUT_FILE = "better_vel_V2.csv"
# 默认有效段门控列。
DEFAULT_GATE_COLUMN = "I2"
# 默认有效段阈值。
DEFAULT_GATE_THRESHOLD = 40.0
# 默认桥接短缺口长度，单位采样点。
DEFAULT_MAX_GAP_SAMPLES = 3
# 默认最小有效段长度，单位采样点。
DEFAULT_MIN_SEGMENT_SAMPLES = 50
# 默认采样频率，单位 Hz。
DEFAULT_SAMPLE_RATE_HZ = 100.0
# 默认允许的最大等效滤波延迟，单位采样点。
DEFAULT_MAX_FILTER_DELAY = 2.0
# 默认输出目录。
DEFAULT_OUTPUT_DIR = "fit_plots_better_vel_v2"


@dataclass(frozen=True)
class FilterCandidate:
    """保存低延迟滤波候选。

    参数:
        name: 候选名称。
        filter_func: 过滤单段数据的一维滤波函数。
        delay_samples: 该候选的等效延迟，单位采样点。

    返回:
        FilterCandidate: 低延迟滤波候选对象。
    """

    name: str
    filter_func: Callable[[np.ndarray], np.ndarray]
    delay_samples: float


@dataclass(frozen=True)
class ChannelFitResult:
    """保存单通道拟合结果。

    参数:
        target_name: 目标列名。
        reference_name: 参考列名。
        scale: 最优缩放系数。
        residual_rms: 残差均方根。
        residual_mean_abs: 残差平均绝对值。
        filtered_target: 目标滤波后的有效段拼接结果。
        filtered_reference: 参考滤波后的有效段拼接结果。
        raw_target: 目标原始有效段拼接结果。
        raw_reference: 参考原始有效段拼接结果。
        raw_scale: 原始未滤波情况下的最小二乘缩放系数。

    返回:
        ChannelFitResult: 单通道拟合结果对象。
    """

    target_name: str
    reference_name: str
    scale: float
    residual_rms: float
    residual_mean_abs: float
    filtered_target: np.ndarray
    filtered_reference: np.ndarray
    raw_target: np.ndarray
    raw_reference: np.ndarray
    raw_scale: float


@dataclass(frozen=True)
class GlobalFitResult:
    """保存两路联合拟合结果。

    参数:
        target_filter: 目标信号统一滤波候选。
        reference_filter: 参考信号统一滤波候选。
        channel_x: I0/I5 通道拟合结果。
        channel_y: I1/I6 通道拟合结果。
        total_score: 联合评分，越小越优。
        total_samples: 总有效样本数。
        segment_count: 有效段数量。

    返回:
        GlobalFitResult: 两路联合拟合结果对象。
    """

    target_filter: FilterCandidate
    reference_filter: FilterCandidate
    channel_x: ChannelFitResult
    channel_y: ChannelFitResult
    total_score: float
    total_samples: int
    segment_count: int


def parse_args() -> argparse.Namespace:
    """解析命令行参数。

    参数:
        无。

    返回:
        argparse.Namespace: 已完成类型转换的命令行参数对象。
    """

    parser = argparse.ArgumentParser(
        description="对 better_vel_V2.csv 执行低延迟 K/M 联合拟合，并保存融合效果图。"
    )
    parser.add_argument(
        "--input",
        default=DEFAULT_INPUT_FILE,
        help="输入 CSV 文件路径，默认 better_vel_V2.csv。",
    )
    parser.add_argument(
        "--gate-column",
        default=DEFAULT_GATE_COLUMN,
        help="有效段门控列名，默认 I2。",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=DEFAULT_GATE_THRESHOLD,
        help="有效段阈值，默认 40。",
    )
    parser.add_argument(
        "--max-gap-samples",
        type=int,
        default=DEFAULT_MAX_GAP_SAMPLES,
        help="允许桥接的短时跌破阈值长度，默认 3 点。",
    )
    parser.add_argument(
        "--min-segment-samples",
        type=int,
        default=DEFAULT_MIN_SEGMENT_SAMPLES,
        help="最小有效段长度，默认 50 点。",
    )
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=DEFAULT_SAMPLE_RATE_HZ,
        help="采样频率，单位 Hz，默认 100。",
    )
    parser.add_argument(
        "--max-filter-delay",
        type=float,
        default=DEFAULT_MAX_FILTER_DELAY,
        help="允许的最大等效滤波延迟，单位采样点，默认 2。",
    )
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help="输出目录，默认 fit_plots_better_vel_v2。",
    )
    return parser.parse_args()


def load_csv_columns(csv_path: Path, required_columns: tuple[str, ...]) -> dict[str, np.ndarray]:
    """读取指定列并转换为浮点数组。

    参数:
        csv_path: CSV 文件路径。
        required_columns: 必须存在的列名集合。

    返回:
        dict[str, np.ndarray]: 以列名为键、浮点数组为值的列数据字典。
    """

    with csv_path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError("CSV 文件缺少表头。")

        missing_columns = [column for column in required_columns if column not in reader.fieldnames]
        if missing_columns:
            raise ValueError(f"CSV 文件缺少必要列: {', '.join(missing_columns)}")

        column_buffers = {column: [] for column in required_columns}
        for row_index, row in enumerate(reader, start=2):
            try:
                for column in required_columns:
                    column_buffers[column].append(float(row[column]))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"第 {row_index} 行列 {column} 不是合法浮点数。") from exc

    return {column: np.asarray(values, dtype=np.float64) for column, values in column_buffers.items()}


def bridge_short_gaps(mask: np.ndarray, max_gap_samples: int) -> np.ndarray:
    """桥接被有效段夹住的短时无效缺口。

    参数:
        mask: 原始有效布尔掩码。
        max_gap_samples: 允许桥接的最大缺口长度，单位采样点。

    返回:
        np.ndarray: 桥接后的有效布尔掩码。
    """

    if max_gap_samples < 0:
        raise ValueError("max_gap_samples 不能小于 0。")

    bridged_mask = mask.copy()
    index = 0
    while index < bridged_mask.size:
        if bridged_mask[index]:
            index += 1
            continue

        gap_end = index
        while gap_end < bridged_mask.size and not bridged_mask[gap_end]:
            gap_end += 1

        is_internal_gap = index > 0 and gap_end < bridged_mask.size
        is_short_gap = gap_end - index <= max_gap_samples
        if is_internal_gap and is_short_gap and bridged_mask[index - 1] and bridged_mask[gap_end]:
            bridged_mask[index:gap_end] = True

        index = gap_end

    return bridged_mask


def extract_segments(mask: np.ndarray, min_segment_samples: int) -> list[tuple[int, int]]:
    """从布尔掩码中提取有效段。

    参数:
        mask: 有效布尔掩码。
        min_segment_samples: 最小有效段长度，单位采样点。

    返回:
        list[tuple[int, int]]: 有效段列表，每项为 [start, end) 半开区间。
    """

    if min_segment_samples <= 0:
        raise ValueError("min_segment_samples 必须大于 0。")

    segments: list[tuple[int, int]] = []
    segment_start: int | None = None

    for index, flag in enumerate(mask):
        if flag:
            if segment_start is None:
                segment_start = index
            continue

        if segment_start is not None and index - segment_start >= min_segment_samples:
            segments.append((segment_start, index))
        segment_start = None

    if segment_start is not None and mask.size - segment_start >= min_segment_samples:
        segments.append((segment_start, mask.size))

    return segments


def concatenate_segments(values: np.ndarray, segments: list[tuple[int, int]]) -> np.ndarray:
    """按有效段拼接一维信号。

    参数:
        values: 原始一维信号。
        segments: 有效段列表，每项为 [start, end) 半开区间。

    返回:
        np.ndarray: 按有效段首尾拼接后的信号。
    """

    return np.concatenate([values[start:end] for start, end in segments], axis=0)


def causal_moving_average(values: np.ndarray, window_size: int) -> np.ndarray:
    """执行因果滑动平均滤波。

    参数:
        values: 待滤波的一维信号。
        window_size: 滑动窗口长度，单位采样点。

    返回:
        np.ndarray: 与输入同长度的滤波结果。
    """

    if values.size == 0 or window_size <= 1:
        return values.copy()

    filtered = np.empty_like(values, dtype=np.float64)
    accumulator = 0.0
    for index in range(values.size):
        accumulator += values[index]
        if index >= window_size:
            accumulator -= values[index - window_size]
            filtered[index] = accumulator / float(window_size)
        else:
            filtered[index] = accumulator / float(index + 1)
    return filtered


def exponential_moving_average(values: np.ndarray, alpha: float) -> np.ndarray:
    """执行指数滑动平均滤波。

    参数:
        values: 待滤波的一维信号。
        alpha: 当前样本权重，取值范围 (0, 1]。

    返回:
        np.ndarray: 与输入同长度的滤波结果。
    """

    if values.size == 0:
        return values.copy()
    if not 0.0 < alpha <= 1.0:
        raise ValueError("alpha 必须在 (0, 1] 范围内。")

    filtered = np.empty_like(values, dtype=np.float64)
    filtered[0] = values[0]
    for index in range(1, values.size):
        filtered[index] = alpha * values[index] + (1.0 - alpha) * filtered[index - 1]
    return filtered


def scalar_kalman_filter(values: np.ndarray, process_noise: float, measure_noise: float) -> np.ndarray:
    """执行一维随机游走卡尔曼滤波。

    参数:
        values: 待滤波的一维信号。
        process_noise: 过程噪声方差 q。
        measure_noise: 观测噪声方差 r。

    返回:
        np.ndarray: 与输入同长度的滤波结果。
    """

    if values.size == 0:
        return values.copy()
    if process_noise <= 0.0 or measure_noise <= 0.0:
        raise ValueError("process_noise 与 measure_noise 必须大于 0。")

    filtered = np.empty_like(values, dtype=np.float64)
    estimate = values[0]
    covariance = 1.0
    filtered[0] = estimate

    for index in range(1, values.size):
        covariance += process_noise
        kalman_gain = covariance / (covariance + measure_noise)
        estimate = estimate + kalman_gain * (values[index] - estimate)
        covariance = (1.0 - kalman_gain) * covariance
        filtered[index] = estimate

    return filtered


def estimate_kalman_delay(process_noise: float, measure_noise: float, warmup_steps: int = 256) -> float:
    """估计一维卡尔曼滤波的等效延迟。

    参数:
        process_noise: 过程噪声方差 q。
        measure_noise: 观测噪声方差 r。
        warmup_steps: 用于逼近稳态增益的迭代步数。

    返回:
        float: 估计得到的等效延迟，单位采样点。
    """

    covariance = 1.0
    gains: list[float] = []
    for _ in range(warmup_steps):
        covariance += process_noise
        kalman_gain = covariance / (covariance + measure_noise)
        covariance = (1.0 - kalman_gain) * covariance
        gains.append(kalman_gain)

    steady_gain = float(np.mean(gains[-32:]))
    return (1.0 - steady_gain) / max(steady_gain, 1e-9)


def build_filter_candidates(max_filter_delay: float) -> list[FilterCandidate]:
    """构建满足低延迟约束的候选滤波器。

    参数:
        max_filter_delay: 允许的最大等效延迟，单位采样点。

    返回:
        list[FilterCandidate]: 低延迟滤波候选列表。
    """

    candidates: list[FilterCandidate] = [
        FilterCandidate(name="raw", filter_func=lambda values: values.copy(), delay_samples=0.0),
    ]

    for window_size in (2, 3, 4, 5):
        delay_samples = (window_size - 1) / 2.0
        if delay_samples <= max_filter_delay:
            candidates.append(
                FilterCandidate(
                    name=f"cma_{window_size}",
                    filter_func=lambda values, w=window_size: causal_moving_average(values, w),
                    delay_samples=delay_samples,
                )
            )

    for alpha in (0.90, 0.80, 0.70, 0.60, 0.50, 0.40):
        delay_samples = (1.0 - alpha) / alpha
        if delay_samples <= max_filter_delay:
            candidates.append(
                FilterCandidate(
                    name=f"ema_{alpha:.2f}",
                    filter_func=lambda values, a=alpha: exponential_moving_average(values, a),
                    delay_samples=delay_samples,
                )
            )

    for process_noise, measure_noise in ((0.1, 0.3), (0.1, 0.5), (0.2, 0.5), (0.3, 0.5), (0.5, 1.0), (1.0, 1.0)):
        delay_samples = estimate_kalman_delay(process_noise, measure_noise)
        if delay_samples <= max_filter_delay:
            candidates.append(
                FilterCandidate(
                    name=f"kf_q{process_noise:.2f}_r{measure_noise:.2f}",
                    filter_func=lambda values, q=process_noise, r=measure_noise: scalar_kalman_filter(values, q, r),
                    delay_samples=delay_samples,
                )
            )

    return candidates


def apply_filter_on_segments(
    values: np.ndarray,
    segments: list[tuple[int, int]],
    filter_func: Callable[[np.ndarray], np.ndarray],
) -> np.ndarray:
    """在每个有效段内独立执行滤波并拼接结果。

    参数:
        values: 原始一维信号。
        segments: 有效段列表，每项为 [start, end) 半开区间。
        filter_func: 单段滤波函数。

    返回:
        np.ndarray: 拼接后的滤波结果。
    """

    return np.concatenate([filter_func(values[start:end]) for start, end in segments], axis=0)


def solve_scale(target_values: np.ndarray, reference_values: np.ndarray) -> tuple[float, float, float]:
    """求解无截距最小二乘缩放系数及残差指标。

    参数:
        target_values: 目标信号。
        reference_values: 参考信号。

    返回:
        tuple[float, float, float]:
            - scale: 最优缩放系数。
            - residual_rms: 残差均方根。
            - residual_mean_abs: 残差平均绝对值。
    """

    denominator = float(np.dot(reference_values, reference_values))
    if denominator <= 0.0:
        raise ValueError("参考信号能量为 0，无法进行最小二乘拟合。")

    scale = float(np.dot(reference_values, target_values) / denominator)
    residual = target_values - scale * reference_values
    residual_rms = float(np.sqrt(np.mean(residual * residual)))
    residual_mean_abs = float(np.mean(np.abs(residual)))
    return scale, residual_rms, residual_mean_abs


def fit_single_channel(
    target_name: str,
    reference_name: str,
    target_values: np.ndarray,
    reference_values: np.ndarray,
    segments: list[tuple[int, int]],
    target_filter: FilterCandidate,
    reference_filter: FilterCandidate,
) -> ChannelFitResult:
    """拟合单通道最优缩放系数。

    参数:
        target_name: 目标列名。
        reference_name: 参考列名。
        target_values: 目标原始信号。
        reference_values: 参考原始信号。
        segments: 有效段列表，每项为 [start, end) 半开区间。
        target_filter: 目标滤波候选。
        reference_filter: 参考滤波候选。

    返回:
        ChannelFitResult: 单通道拟合结果对象。
    """

    raw_target = concatenate_segments(target_values, segments)
    raw_reference = concatenate_segments(reference_values, segments)
    filtered_target = apply_filter_on_segments(target_values, segments, target_filter.filter_func)
    filtered_reference = apply_filter_on_segments(reference_values, segments, reference_filter.filter_func)

    raw_scale, _, _ = solve_scale(raw_target, raw_reference)
    scale, residual_rms, residual_mean_abs = solve_scale(filtered_target, filtered_reference)
    return ChannelFitResult(
        target_name=target_name,
        reference_name=reference_name,
        scale=scale,
        residual_rms=residual_rms,
        residual_mean_abs=residual_mean_abs,
        filtered_target=filtered_target,
        filtered_reference=filtered_reference,
        raw_target=raw_target,
        raw_reference=raw_reference,
        raw_scale=raw_scale,
    )


def evaluate_best_scheme(
    columns: dict[str, np.ndarray],
    segments: list[tuple[int, int]],
    filter_candidates: list[FilterCandidate],
) -> GlobalFitResult:
    """搜索两路通用的最优低延迟滤波与缩放方案。

    参数:
        columns: 必要列数据字典。
        segments: 有效段列表，每项为 [start, end) 半开区间。
        filter_candidates: 低延迟滤波候选列表。

    返回:
        GlobalFitResult: 联合拟合最优结果对象。
    """

    best_result: GlobalFitResult | None = None
    total_samples = sum(end - start for start, end in segments)

    for target_filter in filter_candidates:
        for reference_filter in filter_candidates:
            channel_x = fit_single_channel(
                target_name="I0",
                reference_name="I5",
                target_values=columns["I0"],
                reference_values=columns["I5"],
                segments=segments,
                target_filter=target_filter,
                reference_filter=reference_filter,
            )
            channel_y = fit_single_channel(
                target_name="I1",
                reference_name="I6",
                target_values=columns["I1"],
                reference_values=columns["I6"],
                segments=segments,
                target_filter=target_filter,
                reference_filter=reference_filter,
            )

            latency_penalty = 0.05 * max(target_filter.delay_samples, reference_filter.delay_samples)
            total_score = channel_x.residual_rms + channel_y.residual_rms + latency_penalty
            candidate_result = GlobalFitResult(
                target_filter=target_filter,
                reference_filter=reference_filter,
                channel_x=channel_x,
                channel_y=channel_y,
                total_score=total_score,
                total_samples=total_samples,
                segment_count=len(segments),
            )

            if best_result is None or candidate_result.total_score < best_result.total_score:
                best_result = candidate_result

    if best_result is None:
        raise ValueError("没有可用的低延迟滤波候选。")

    return best_result


def save_fusion_plot(
    output_path: Path,
    fit_result: GlobalFitResult,
    sample_rate_hz: float,
) -> None:
    """保存最终融合效果图。

    参数:
        output_path: 输出图片路径。
        fit_result: 联合拟合结果对象。
        sample_rate_hz: 采样频率，单位 Hz。

    返回:
        无。
    """

    figure, axes = plt.subplots(2, 2, figsize=(15, 9), sharex="col")

    for axis_group, channel_result, gain_label in (
        ((axes[0, 0], axes[1, 0]), fit_result.channel_x, "K"),
        ((axes[0, 1], axes[1, 1]), fit_result.channel_y, "M"),
    ):
        top_axis, bottom_axis = axis_group
        point_count = channel_result.filtered_target.size
        time_axis = np.arange(point_count, dtype=np.float64) / sample_rate_hz

        fitted_projection = channel_result.scale * channel_result.filtered_reference
        fused_output = channel_result.filtered_target - fitted_projection
        raw_projection = channel_result.raw_scale * channel_result.raw_reference
        raw_output = channel_result.raw_target - raw_projection

        top_axis.plot(time_axis, channel_result.filtered_target, linewidth=0.9, color="#0b3954", label="filtered target")
        top_axis.plot(time_axis, fitted_projection, linewidth=0.9, color="#087e8b", label="scaled reference")
        top_axis.set_title(
            f"{channel_result.target_name} vs {channel_result.reference_name} | "
            f"{gain_label}={channel_result.scale:.10f}"
        )
        top_axis.set_ylabel("amplitude")
        top_axis.grid(True, alpha=0.25)
        top_axis.legend(loc="upper right")

        bottom_axis.plot(time_axis, raw_output, linewidth=0.8, color="#f77f00", label="raw residual")
        bottom_axis.plot(time_axis, fused_output, linewidth=0.9, color="#1d3557", label="fused residual")
        bottom_axis.axhline(0.0, linewidth=0.8, linestyle="--", color="#6c757d")
        bottom_axis.set_xlabel("time / s")
        bottom_axis.set_ylabel("residual")
        bottom_axis.grid(True, alpha=0.25)
        bottom_axis.legend(loc="upper right")

    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def main() -> int:
    """执行主流程并输出最优 K、M 及处理逻辑。

    参数:
        无。

    返回:
        int: 进程退出码，0 表示成功，非 0 表示失败。
    """

    args = parse_args()
    csv_path = Path(args.input).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    required_columns = ("I0", "I1", args.gate_column, "I5", "I6")
    try:
        columns = load_csv_columns(csv_path, required_columns)
    except (OSError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1

    mask = columns[args.gate_column] > args.threshold
    bridged_mask = bridge_short_gaps(mask, args.max_gap_samples)
    segments = extract_segments(bridged_mask, args.min_segment_samples)
    if not segments:
        print("未找到满足条件的有效段。")
        print(
            f"规则: {args.gate_column} > {args.threshold:.3f}, "
            f"桥接缺口 <= {args.max_gap_samples} 点, 最小段长 {args.min_segment_samples} 点"
        )
        return 2

    filter_candidates = build_filter_candidates(args.max_filter_delay)
    fit_result = evaluate_best_scheme(columns, segments, filter_candidates)

    plot_path = output_dir / f"{csv_path.stem}_fusion_effect.png"
    save_fusion_plot(plot_path, fit_result, args.sample_rate)

    total_duration_s = fit_result.total_samples / args.sample_rate
    print(f"文件: {csv_path.name}")
    print(f"最优 K = {fit_result.channel_x.scale:.10f}")
    print(f"最优 M = {fit_result.channel_y.scale:.10f}")
    print(
        "处理逻辑: "
        f"先按 {args.gate_column} > {args.threshold:.3f} 提取有效点，"
        f"并桥接被有效点夹住且长度 <= {args.max_gap_samples} 点的短时失效缺口；"
        f"仅保留长度 >= {args.min_segment_samples} 点的稳定有效段。"
    )
    print(
        "最优统一滤波链: "
        f"I0/I1 使用 {fit_result.target_filter.name}，"
        f"I5/I6 使用 {fit_result.reference_filter.name}；"
        f"最大等效延迟约 {max(fit_result.target_filter.delay_samples, fit_result.reference_filter.delay_samples):.3f} 点。"
    )
    print(
        "有效段统计: "
        f"{fit_result.segment_count} 段，{fit_result.total_samples} 点，约 {total_duration_s:.3f} 秒。"
    )
    print(
        "X 轴拟合: "
        f"I0_f - K*I5_f -> RMS={fit_result.channel_x.residual_rms:.10f}, "
        f"MAE={fit_result.channel_x.residual_mean_abs:.10f}。"
    )
    print(
        "Y 轴拟合: "
        f"I1_f - M*I6_f -> RMS={fit_result.channel_y.residual_rms:.10f}, "
        f"MAE={fit_result.channel_y.residual_mean_abs:.10f}。"
    )
    print(f"融合效果图: {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
