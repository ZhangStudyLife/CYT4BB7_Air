#!/usr/bin/env python3
"""拟合光流消抖系数的独立脚本。"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Callable

import numpy as np

# 默认输入文件路径。
DEFAULT_INPUT_FILE = "new_pmw3901_data.csv"
# 默认有效段判定列名。
DEFAULT_GATE_COLUMN = "I2"
# 默认有效段阈值。
DEFAULT_GATE_THRESHOLD = 70.0
# 默认长期有效段最小长度，单位为采样点。
DEFAULT_MIN_SEGMENT_SAMPLES = 50
# 默认采样频率，单位 Hz。
DEFAULT_SAMPLE_RATE_HZ = 100.0


def parse_args() -> argparse.Namespace:
    """解析命令行参数。

    参数:
        无。

    返回:
        argparse.Namespace: 已完成类型转换的命令行参数对象。
    """

    parser = argparse.ArgumentParser(
        description="按连续有效段拟合 I0-K*I5 和 I1-M*I6 的最优消抖系数。"
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
        help="有效段阈值，默认 70。",
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

    if window_size <= 1 or signal.size == 0:
        return signal.copy()

    pad_left = window_size // 2
    pad_right = window_size - 1 - pad_left
    padded = np.pad(signal, (pad_left, pad_right), mode="edge")
    kernel = np.ones(window_size, dtype=np.float64) / float(window_size)
    return np.convolve(padded, kernel, mode="valid")


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


def build_filter_candidates() -> list[tuple[str, Callable[[np.ndarray], np.ndarray]]]:
    """构建待评估的滤波候选集合。

    参数:
        无。

    返回:
        list[tuple[str, Callable[[np.ndarray], np.ndarray]]]: 候选名称与执行函数列表。
    """

    return [
        ("raw", lambda values: values.copy()),
        ("fir_ma_3", lambda values: moving_average(values, 3)),
        ("fir_ma_5", lambda values: moving_average(values, 5)),
        ("fir_ma_9", lambda values: moving_average(values, 9)),
        ("fir_ma_15", lambda values: moving_average(values, 15)),
        ("iir_alpha_0.20", lambda values: single_pole_iir(values, 0.20)),
        ("iir_alpha_0.35", lambda values: single_pole_iir(values, 0.35)),
        ("iir_alpha_0.50", lambda values: single_pole_iir(values, 0.50)),
        ("iir_alpha_0.70", lambda values: single_pole_iir(values, 0.70)),
    ]


def concatenate_segments(values: np.ndarray, segments: list[tuple[int, int]]) -> np.ndarray:
    """按有效段拼接信号。

    参数:
        values: 原始一维信号。
        segments: 有效段区间列表，每项为 [start, end)。

    返回:
        np.ndarray: 将多个有效段首尾拼接后的结果。
    """

    return np.concatenate([values[start:end] for start, end in segments], axis=0)


def apply_filter_on_segments(
    values: np.ndarray,
    segments: list[tuple[int, int]],
    filter_func: Callable[[np.ndarray], np.ndarray],
) -> np.ndarray:
    """在每个有效段内独立应用滤波，再拼接结果。

    参数:
        values: 原始一维信号。
        segments: 有效段区间列表，每项为 [start, end)。
        filter_func: 单段滤波函数。

    返回:
        np.ndarray: 拼接后的滤波结果。
    """

    filtered_segments = [filter_func(values[start:end]) for start, end in segments]
    return np.concatenate(filtered_segments, axis=0)


def solve_scale(target_values: np.ndarray, reference_values: np.ndarray) -> tuple[float, float, float]:
    """求解无截距最小二乘缩放系数及残差指标。

    参数:
        target_values: 目标信号，例如 I0 或 I1。
        reference_values: 参考信号，例如滤波后的 I5 或 I6。

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
) -> dict[str, float | str]:
    """枚举滤波候选并选择残差 RMS 最小的方案。

    参数:
        target_values: 目标信号，例如 I0 或 I1。
        reference_values: 参考信号，例如 I5 或 I6。
        segments: 有效段区间列表，每项为 [start, end)。

    返回:
        dict[str, float | str]: 包含最优滤波名称、系数和残差指标的结果字典。
    """

    target_segment_values = concatenate_segments(target_values, segments)
    best_result: dict[str, float | str] | None = None

    for filter_name, filter_func in build_filter_candidates():
        filtered_reference = apply_filter_on_segments(reference_values, segments, filter_func)
        scale, residual_rms, residual_mean_abs = solve_scale(target_segment_values, filtered_reference)
        candidate_result: dict[str, float | str] = {
            "filter_name": filter_name,
            "scale": scale,
            "residual_rms": residual_rms,
            "residual_mean_abs": residual_mean_abs,
        }

        if best_result is None or residual_rms < float(best_result["residual_rms"]):
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


def main() -> int:
    """执行主流程并输出拟合结果。

    参数:
        无。

    返回:
        int: 进程退出码，0 表示成功，非 0 表示失败。
    """

    args = parse_args()
    csv_path = Path(args.input).resolve()

    required_columns = (
        "I0",
        "I1",
        args.gate_column,
        "I5",
        "I6",
    )

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
        gate_min = float(np.min(columns[args.gate_column]))
        gate_max = float(np.max(columns[args.gate_column]))
        print(f"文件: {csv_path.name}")
        print("未找到满足条件的长期有效段。")
        print(
            f"规则: {args.gate_column} > {args.threshold:.3f}, "
            f"最小长度 {args.min_segment_samples} 点"
        )
        print(f"实际 {args.gate_column} 范围: [{gate_min:.3f}, {gate_max:.3f}]")
        print("处理逻辑: 先按阈值提取连续段，再过滤过短段；当前文件在该规则下没有可拟合数据。")
        return 2

    segment_count, total_samples, total_duration_s = format_segment_summary(
        segments=segments,
        sample_rate_hz=args.sample_rate,
    )

    best_k = evaluate_best_fit(
        target_values=columns["I0"],
        reference_values=columns["I5"],
        segments=segments,
    )
    best_m = evaluate_best_fit(
        target_values=columns["I1"],
        reference_values=columns["I6"],
        segments=segments,
    )

    print(f"文件: {csv_path.name}")
    print(f"最优 K = {float(best_k['scale']):.10f}")
    print(f"最优 M = {float(best_m['scale']):.10f}")
    print(
        "处理逻辑: "
        f"先按 {args.gate_column} > {args.threshold:.3f} 提取连续段，并保留长度 >= "
        f"{args.min_segment_samples} 点的长期有效段；共保留 {segment_count} 段，"
        f"{total_samples} 点，约 {total_duration_s:.3f} 秒。"
    )
    print(
        "K 拟合: "
        f"在有效段内对 I5 尝试 raw/FIR/IIR 候选滤波，最终选择 {best_k['filter_name']}，"
        f"以无截距最小二乘最小化 I0 - K*I5_filtered 的残差 RMS；"
        f"残差 RMS={float(best_k['residual_rms']):.10f}，"
        f"残差平均绝对值={float(best_k['residual_mean_abs']):.10f}。"
    )
    print(
        "M 拟合: "
        f"在有效段内对 I6 尝试 raw/FIR/IIR 候选滤波，最终选择 {best_m['filter_name']}，"
        f"以无截距最小二乘最小化 I1 - M*I6_filtered 的残差 RMS；"
        f"残差 RMS={float(best_m['residual_rms']):.10f}，"
        f"残差平均绝对值={float(best_m['residual_mean_abs']):.10f}。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
