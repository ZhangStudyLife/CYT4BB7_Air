#!/usr/bin/env python3
"""拟合消抖系数并输出拟合前后曲线。"""

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

# 默认输入文件路径。
DEFAULT_INPUT_FILE = "new_pmw3901_data.csv"
# 默认有效段判定列名。
DEFAULT_GATE_COLUMN = "I2"
# 默认有效段阈值。
DEFAULT_GATE_THRESHOLD = 60.0
# 默认长期有效段最小长度，单位为采样点。
DEFAULT_MIN_SEGMENT_SAMPLES = 50
# 默认采样频率，单位 Hz。
DEFAULT_SAMPLE_RATE_HZ = 100.0
# 默认时延搜索范围，单位采样点。
DEFAULT_MAX_DELAY_SAMPLES = 5


@dataclass(frozen=True)
class FitResult:
    """保存单通道拟合结果。

    参数:
        filter_name: 最优滤波器名称。
        delay_samples: 最优时延，正值表示参考信号向后对齐。
        scale: 最优缩放系数。
        residual_rms: 残差均方根。
        residual_mean_abs: 残差平均绝对值。
        target_used: 有效段拼接后的目标信号。
        reference_used: 有效段拼接后的原始参考信号。
        filtered_reference_used: 最优滤波和时延处理后的参考信号。

    返回:
        FitResult: 单通道拟合结果对象。
    """

    filter_name: str
    delay_samples: int
    scale: float
    residual_rms: float
    residual_mean_abs: float
    target_used: np.ndarray
    reference_used: np.ndarray
    filtered_reference_used: np.ndarray


def parse_args() -> argparse.Namespace:
    """解析命令行参数。

    参数:
        无。

    返回:
        argparse.Namespace: 已完成类型转换的命令行参数对象。
    """

    parser = argparse.ArgumentParser(
        description="按连续有效段拟合 I0-K*I5 和 I1-M*I6，并保存拟合前后曲线。"
    )
    parser.add_argument(
        "--input",
        default=DEFAULT_INPUT_FILE,
        help="待处理的 CSV 文件路径，默认读取当前目录下的 new_pmw3901_data.csv。",
    )
    parser.add_argument(
        "--gate-column",
        default=DEFAULT_GATE_COLUMN,
        help="用于筛选有效段的列名，默认 I2。",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=DEFAULT_GATE_THRESHOLD,
        help="有效段阈值，默认 60。",
    )
    parser.add_argument(
        "--min-segment-samples",
        type=int,
        default=DEFAULT_MIN_SEGMENT_SAMPLES,
        help="长期有效段最小长度，单位采样点，默认 50。",
    )
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=DEFAULT_SAMPLE_RATE_HZ,
        help="采样频率，单位 Hz，默认 100。",
    )
    parser.add_argument(
        "--max-delay-samples",
        type=int,
        default=DEFAULT_MAX_DELAY_SAMPLES,
        help="深入拟合时参考信号允许搜索的最大时延，单位采样点，默认 5。",
    )
    parser.add_argument(
        "--output-dir",
        default="fit_plots",
        help="输出图片目录，默认 fit_plots。",
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


def find_valid_segments(
    gate_values: np.ndarray,
    threshold: float,
    min_segment_samples: int,
) -> list[tuple[int, int]]:
    """按阈值提取连续有效段。

    参数:
        gate_values: 用于判定有效段的门控列数据。
        threshold: 判定阈值，满足 gate_values > threshold 视为有效。
        min_segment_samples: 长期有效段最小长度，单位采样点。

    返回:
        list[tuple[int, int]]: 有效段列表，每项为 [start, end) 半开区间。
    """

    if min_segment_samples <= 0:
        raise ValueError("min_segment_samples 必须大于 0。")

    segments: list[tuple[int, int]] = []
    segment_start: int | None = None

    for index, value in enumerate(gate_values):
        if value > threshold:
            if segment_start is None:
                segment_start = index
            continue

        if segment_start is not None and index - segment_start >= min_segment_samples:
            segments.append((segment_start, index))
        segment_start = None

    if segment_start is not None and len(gate_values) - segment_start >= min_segment_samples:
        segments.append((segment_start, len(gate_values)))

    return segments


def moving_average(signal: np.ndarray, window_size: int) -> np.ndarray:
    """执行对称滑动平均 FIR 滤波。

    参数:
        signal: 待滤波的一维信号。
        window_size: 滑动窗口长度，单位采样点。

    返回:
        np.ndarray: 与输入同长度的滤波结果。
    """

    if signal.size == 0 or window_size <= 1:
        return signal.copy()

    pad_left = window_size // 2
    pad_right = window_size - 1 - pad_left
    padded = np.pad(signal, (pad_left, pad_right), mode="edge")
    kernel = np.ones(window_size, dtype=np.float64) / float(window_size)
    return np.convolve(padded, kernel, mode="valid")


def cascaded_moving_average(signal: np.ndarray, first_window: int, second_window: int) -> np.ndarray:
    """执行两级滑动平均滤波。

    参数:
        signal: 待滤波的一维信号。
        first_window: 第一级 FIR 窗口长度。
        second_window: 第二级 FIR 窗口长度。

    返回:
        np.ndarray: 与输入同长度的滤波结果。
    """

    return moving_average(moving_average(signal, first_window), second_window)


def single_pole_iir(signal: np.ndarray, alpha: float) -> np.ndarray:
    """执行单极点 IIR 低通滤波。

    参数:
        signal: 待滤波的一维信号。
        alpha: 当前样本权重，取值范围应在 0 到 1 之间。

    返回:
        np.ndarray: 与输入同长度的滤波结果。
    """

    if signal.size == 0:
        return signal.copy()
    if not 0.0 < alpha <= 1.0:
        raise ValueError("alpha 必须在 (0, 1] 范围内。")

    filtered = np.empty_like(signal, dtype=np.float64)
    filtered[0] = signal[0]
    for index in range(1, signal.size):
        filtered[index] = alpha * signal[index] + (1.0 - alpha) * filtered[index - 1]
    return filtered


def zero_phase_iir(signal: np.ndarray, alpha: float) -> np.ndarray:
    """执行前后向 IIR 滤波以减小时延。

    参数:
        signal: 待滤波的一维信号。
        alpha: 单极点 IIR 的当前样本权重。

    返回:
        np.ndarray: 与输入同长度的近似零相位滤波结果。
    """

    forward = single_pole_iir(signal, alpha)
    backward = single_pole_iir(forward[::-1], alpha)
    return backward[::-1]


def build_filter_candidates() -> list[tuple[str, Callable[[np.ndarray], np.ndarray]]]:
    """构建待评估的滤波候选集合。

    参数:
        无。

    返回:
        list[tuple[str, Callable[[np.ndarray], np.ndarray]]]: 候选名称与执行函数列表。
    """

    candidates: list[tuple[str, Callable[[np.ndarray], np.ndarray]]] = [("raw", lambda values: values.copy())]

    for window_size in (3, 5, 7, 9, 11, 15, 21):
        candidates.append(
            (f"fir_ma_{window_size}", lambda values, w=window_size: moving_average(values, w))
        )

    for first_window, second_window in ((3, 3), (5, 3), (5, 5), (7, 5)):
        candidates.append(
            (
                f"fir_cascade_{first_window}_{second_window}",
                lambda values, w1=first_window, w2=second_window: cascaded_moving_average(values, w1, w2),
            )
        )

    for alpha in (0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.45, 0.60, 0.75, 0.85):
        candidates.append(
            (f"iir_alpha_{alpha:.2f}", lambda values, a=alpha: single_pole_iir(values, a))
        )
        candidates.append(
            (f"iir_zero_phase_{alpha:.2f}", lambda values, a=alpha: zero_phase_iir(values, a))
        )

    return candidates


def concatenate_segments(values: np.ndarray, segments: list[tuple[int, int]]) -> np.ndarray:
    """按有效段拼接信号。

    参数:
        values: 原始一维信号。
        segments: 有效段区间列表，每项为 [start, end)。

    返回:
        np.ndarray: 将多个有效段首尾拼接后的结果。
    """

    return np.concatenate([values[start:end] for start, end in segments], axis=0)


def shift_signal(signal: np.ndarray, delay_samples: int) -> np.ndarray:
    """对信号执行整数采样点时延平移。

    参数:
        signal: 待平移的一维信号。
        delay_samples: 平移量，正值表示整体向后移动。

    返回:
        np.ndarray: 与输入同长度的平移结果。
    """

    if signal.size == 0 or delay_samples == 0:
        return signal.copy()

    shifted = np.empty_like(signal, dtype=np.float64)
    if delay_samples > 0:
        shifted[:delay_samples] = signal[0]
        shifted[delay_samples:] = signal[:-delay_samples]
        return shifted

    shift_count = -delay_samples
    shifted[-shift_count:] = signal[-1]
    shifted[:-shift_count] = signal[shift_count:]
    return shifted


def apply_filter_on_segments(
    values: np.ndarray,
    segments: list[tuple[int, int]],
    filter_func: Callable[[np.ndarray], np.ndarray],
    delay_samples: int,
) -> np.ndarray:
    """在每个有效段内独立应用滤波和时延对齐，再拼接结果。

    参数:
        values: 原始一维信号。
        segments: 有效段区间列表，每项为 [start, end)。
        filter_func: 单段滤波函数。
        delay_samples: 整数采样点时延。

    返回:
        np.ndarray: 拼接后的处理结果。
    """

    processed_segments = []
    for start, end in segments:
        filtered = filter_func(values[start:end])
        aligned = shift_signal(filtered, delay_samples)
        processed_segments.append(aligned)
    return np.concatenate(processed_segments, axis=0)


def solve_scale(target_values: np.ndarray, reference_values: np.ndarray) -> tuple[float, float, float]:
    """求解无截距最小二乘缩放系数及残差指标。

    参数:
        target_values: 目标信号，例如 I0 或 I1。
        reference_values: 参考信号，例如处理后的 I5 或 I6。

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


def evaluate_best_fit(
    target_values: np.ndarray,
    reference_values: np.ndarray,
    segments: list[tuple[int, int]],
    max_delay_samples: int,
) -> FitResult:
    """枚举滤波器和小范围时延，选择残差 RMS 最小的方案。

    参数:
        target_values: 目标信号，例如 I0 或 I1。
        reference_values: 参考信号，例如 I5 或 I6。
        segments: 有效段区间列表，每项为 [start, end)。
        max_delay_samples: 允许搜索的最大时延，单位采样点。

    返回:
        FitResult: 最优拟合结果对象。
    """

    target_segment_values = concatenate_segments(target_values, segments)
    reference_segment_values = concatenate_segments(reference_values, segments)
    best_result: FitResult | None = None

    for filter_name, filter_func in build_filter_candidates():
        for delay_samples in range(-max_delay_samples, max_delay_samples + 1):
            filtered_reference = apply_filter_on_segments(
                values=reference_values,
                segments=segments,
                filter_func=filter_func,
                delay_samples=delay_samples,
            )
            scale, residual_rms, residual_mean_abs = solve_scale(target_segment_values, filtered_reference)
            candidate_result = FitResult(
                filter_name=filter_name,
                delay_samples=delay_samples,
                scale=scale,
                residual_rms=residual_rms,
                residual_mean_abs=residual_mean_abs,
                target_used=target_segment_values,
                reference_used=reference_segment_values,
                filtered_reference_used=filtered_reference,
            )

            if best_result is None or candidate_result.residual_rms < best_result.residual_rms:
                best_result = candidate_result

    if best_result is None:
        raise ValueError("没有可用的滤波候选。")

    return best_result


def format_segment_summary(
    segments: list[tuple[int, int]],
    sample_rate_hz: float,
) -> tuple[int, int, float]:
    """汇总有效段数量、样本数和总时长。

    参数:
        segments: 有效段区间列表，每项为 [start, end)。
        sample_rate_hz: 采样频率，单位 Hz。

    返回:
        tuple[int, int, float]:
            - segment_count: 有效段数量。
            - total_samples: 有效样本总数。
            - total_duration_s: 有效段总时长，单位秒。
    """

    total_samples = sum(end - start for start, end in segments)
    total_duration_s = total_samples / sample_rate_hz
    return len(segments), total_samples, total_duration_s


def save_fit_plot(
    output_path: Path,
    title: str,
    sample_rate_hz: float,
    target_values: np.ndarray,
    reference_values: np.ndarray,
    filtered_reference_values: np.ndarray,
    scale: float,
) -> None:
    """保存拟合前后曲线图。

    参数:
        output_path: 输出图片路径。
        title: 图标题。
        sample_rate_hz: 采样频率，单位 Hz。
        target_values: 有效段拼接后的目标信号。
        reference_values: 有效段拼接后的原始参考信号。
        filtered_reference_values: 有效段拼接后的处理后参考信号。
        scale: 最优缩放系数。

    返回:
        无。
    """

    time_axis = np.arange(target_values.size, dtype=np.float64) / sample_rate_hz
    fitted_reference = scale * filtered_reference_values
    fitted_residual = target_values - fitted_reference
    raw_projection = scale * reference_values

    figure, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    # 上图对比拟合前目标曲线、原始参考投影和拟合后参考投影。
    axes[0].plot(time_axis, target_values, label="target", linewidth=1.1, color="#003049")
    axes[0].plot(time_axis, raw_projection, label="raw projection", linewidth=0.9, color="#d62828", alpha=0.75)
    axes[0].plot(time_axis, fitted_reference, label="fitted projection", linewidth=1.0, color="#2a9d8f")
    axes[0].set_ylabel("amplitude")
    axes[0].set_title(title)
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="upper right")

    # 下图对比拟合前残差和拟合后残差。
    axes[1].plot(
        time_axis,
        target_values - raw_projection,
        label="raw residual",
        linewidth=0.9,
        color="#f77f00",
    )
    axes[1].plot(
        time_axis,
        fitted_residual,
        label="fitted residual",
        linewidth=1.0,
        color="#1d3557",
    )
    axes[1].set_xlabel("time / s")
    axes[1].set_ylabel("residual")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="upper right")

    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def save_gate_plot(
    output_path: Path,
    gate_values: np.ndarray,
    threshold: float,
    sample_rate_hz: float,
    gate_column: str,
) -> None:
    """保存门控列与阈值的诊断曲线图。

    参数:
        output_path: 输出图片路径。
        gate_values: 门控列数据。
        threshold: 判定阈值。
        sample_rate_hz: 采样频率，单位 Hz。
        gate_column: 门控列名称。

    返回:
        无。
    """

    time_axis = np.arange(gate_values.size, dtype=np.float64) / sample_rate_hz
    figure, axis = plt.subplots(1, 1, figsize=(14, 4.5))
    axis.plot(time_axis, gate_values, linewidth=0.9, color="#264653", label=gate_column)
    axis.axhline(threshold, linewidth=1.0, color="#e63946", linestyle="--", label=f"threshold {threshold:.3f}")
    axis.set_title(f"{gate_column} gate diagnostic")
    axis.set_xlabel("time / s")
    axis.set_ylabel("amplitude")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="upper right")
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def main() -> int:
    """执行主流程并输出拟合结果。

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

    segments = find_valid_segments(
        gate_values=columns[args.gate_column],
        threshold=args.threshold,
        min_segment_samples=args.min_segment_samples,
    )

    if not segments:
        gate_plot_path = output_dir / f"{csv_path.stem}_{args.gate_column}_threshold_{int(args.threshold)}_diagnostic.png"
        save_gate_plot(
            output_path=gate_plot_path,
            gate_values=columns[args.gate_column],
            threshold=args.threshold,
            sample_rate_hz=args.sample_rate,
            gate_column=args.gate_column,
        )

        gate_min = float(np.min(columns[args.gate_column]))
        gate_max = float(np.max(columns[args.gate_column]))
        print(f"文件: {csv_path.name}")
        print("未找到满足条件的长期有效段。")
        print(f"规则: {args.gate_column} > {args.threshold:.3f}, 最小长度 {args.min_segment_samples} 点")
        print(f"实际 {args.gate_column} 范围: [{gate_min:.3f}, {gate_max:.3f}]")
        print("处理逻辑: 先按阈值提取连续段，再过滤过短段；当前文件在该规则下没有可拟合数据。")
        print(f"诊断曲线: {gate_plot_path}")
        return 2

    segment_count, total_samples, total_duration_s = format_segment_summary(
        segments=segments,
        sample_rate_hz=args.sample_rate,
    )

    best_k = evaluate_best_fit(
        target_values=columns["I0"],
        reference_values=columns["I5"],
        segments=segments,
        max_delay_samples=args.max_delay_samples,
    )
    best_m = evaluate_best_fit(
        target_values=columns["I1"],
        reference_values=columns["I6"],
        segments=segments,
        max_delay_samples=args.max_delay_samples,
    )

    k_plot_path = output_dir / f"{csv_path.stem}_K_fit.png"
    m_plot_path = output_dir / f"{csv_path.stem}_M_fit.png"
    save_fit_plot(
        output_path=k_plot_path,
        title=(
            f"I0 vs I5 | K={best_k.scale:.10f}, "
            f"filter={best_k.filter_name}, delay={best_k.delay_samples} samples"
        ),
        sample_rate_hz=args.sample_rate,
        target_values=best_k.target_used,
        reference_values=best_k.reference_used,
        filtered_reference_values=best_k.filtered_reference_used,
        scale=best_k.scale,
    )
    save_fit_plot(
        output_path=m_plot_path,
        title=(
            f"I1 vs I6 | M={best_m.scale:.10f}, "
            f"filter={best_m.filter_name}, delay={best_m.delay_samples} samples"
        ),
        sample_rate_hz=args.sample_rate,
        target_values=best_m.target_used,
        reference_values=best_m.reference_used,
        filtered_reference_values=best_m.filtered_reference_used,
        scale=best_m.scale,
    )

    print(f"文件: {csv_path.name}")
    print(f"最优 K = {best_k.scale:.10f}")
    print(f"最优 M = {best_m.scale:.10f}")
    print(
        "处理逻辑: "
        f"先按 {args.gate_column} > {args.threshold:.3f} 提取连续段，并保留长度 >= "
        f"{args.min_segment_samples} 点的长期有效段；共保留 {segment_count} 段，"
        f"{total_samples} 点，约 {total_duration_s:.3f} 秒。"
    )
    print(
        "K 深入拟合: "
        f"在有效段内对 I5 枚举 raw、单级 FIR、级联 FIR、单极点 IIR、前后向 IIR，"
        f"并搜索 ±{args.max_delay_samples} 点时延；最终选择 {best_k.filter_name}，"
        f"时延 {best_k.delay_samples} 点，最小二乘得到 K={best_k.scale:.10f}，"
        f"残差 RMS={best_k.residual_rms:.10f}，残差平均绝对值={best_k.residual_mean_abs:.10f}。"
    )
    print(
        "M 深入拟合: "
        f"在有效段内对 I6 枚举 raw、单级 FIR、级联 FIR、单极点 IIR、前后向 IIR，"
        f"并搜索 ±{args.max_delay_samples} 点时延；最终选择 {best_m.filter_name}，"
        f"时延 {best_m.delay_samples} 点，最小二乘得到 M={best_m.scale:.10f}，"
        f"残差 RMS={best_m.residual_rms:.10f}，残差平均绝对值={best_m.residual_mean_abs:.10f}。"
    )
    print(f"K 曲线图: {k_plot_path}")
    print(f"M 曲线图: {m_plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
