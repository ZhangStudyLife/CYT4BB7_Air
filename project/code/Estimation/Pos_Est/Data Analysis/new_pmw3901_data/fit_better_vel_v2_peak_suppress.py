#!/usr/bin/env python3
"""研究 better_vel_V2.csv 的峰值抑制补偿模型。"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 默认输入文件名。
DEFAULT_INPUT_FILE = "better_vel_V2.csv"
# 默认门控列。
DEFAULT_GATE_COLUMN = "I2"
# 默认有效阈值。
DEFAULT_GATE_THRESHOLD = 40.0
# 默认桥接短缺口长度，单位采样点。
DEFAULT_MAX_GAP_SAMPLES = 3
# 默认最小稳定段长度，单位采样点。
DEFAULT_MIN_SEGMENT_SAMPLES = 50
# 默认 EMA 系数。
DEFAULT_EMA_ALPHA = 0.40
# 默认因果均值窗口长度。
DEFAULT_CMA_WINDOW = 5
# 默认采样频率，单位 Hz。
DEFAULT_SAMPLE_RATE_HZ = 100.0
# 默认输出目录。
DEFAULT_OUTPUT_DIR = "fit_plots_better_vel_v2_peak"


@dataclass(frozen=True)
class LinearFitResult:
    """保存线性拟合结果。

    参数:
        scale: 线性补偿系数。
        residual: 残差序列。
        rms_all: 全部样本残差 RMS。
        rms_q95: 高峰区残差 RMS，按 |reference| 的 95 分位阈值统计。
        rms_q98: 更强高峰区残差 RMS，按 |reference| 的 98 分位阈值统计。

    返回:
        LinearFitResult: 线性拟合结果对象。
    """

    scale: float
    residual: np.ndarray
    rms_all: float
    rms_q95: float
    rms_q98: float


@dataclass(frozen=True)
class NonlinearFitResult:
    """保存峰值抑制非线性拟合结果。

    参数:
        linear_gain: 一阶项系数。
        peak_gain: 峰值抑制二阶项系数，对应 reference*abs(reference)。
        residual: 残差序列。
        rms_all: 全部样本残差 RMS。
        rms_q95: 高峰区残差 RMS，按 |reference| 的 95 分位阈值统计。
        rms_q98: 更强高峰区残差 RMS，按 |reference| 的 98 分位阈值统计。

    返回:
        NonlinearFitResult: 峰值抑制非线性拟合结果对象。
    """

    linear_gain: float
    peak_gain: float
    residual: np.ndarray
    rms_all: float
    rms_q95: float
    rms_q98: float


def parse_args() -> argparse.Namespace:
    """解析命令行参数。

    参数:
        无。

    返回:
        argparse.Namespace: 命令行参数对象。
    """

    parser = argparse.ArgumentParser(
        description="对 better_vel_V2.csv 研究线性补偿与峰值抑制非线性补偿。"
    )
    parser.add_argument("--input", default=DEFAULT_INPUT_FILE, help="输入 CSV 文件路径。")
    parser.add_argument("--gate-column", default=DEFAULT_GATE_COLUMN, help="门控列名，默认 I2。")
    parser.add_argument("--threshold", type=float, default=DEFAULT_GATE_THRESHOLD, help="有效阈值，默认 40。")
    parser.add_argument(
        "--max-gap-samples",
        type=int,
        default=DEFAULT_MAX_GAP_SAMPLES,
        help="允许桥接的短缺口长度，默认 3 点。",
    )
    parser.add_argument(
        "--min-segment-samples",
        type=int,
        default=DEFAULT_MIN_SEGMENT_SAMPLES,
        help="最小稳定段长度，默认 50 点。",
    )
    parser.add_argument("--ema-alpha", type=float, default=DEFAULT_EMA_ALPHA, help="EMA 系数，默认 0.40。")
    parser.add_argument("--cma-window", type=int, default=DEFAULT_CMA_WINDOW, help="CMA 窗口长度，默认 5。")
    parser.add_argument("--sample-rate", type=float, default=DEFAULT_SAMPLE_RATE_HZ, help="采样频率，默认 100Hz。")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="输出目录。")
    return parser.parse_args()


def load_csv_columns(csv_path: Path, required_columns: tuple[str, ...]) -> dict[str, np.ndarray]:
    """读取指定列并转换为浮点数组。

    参数:
        csv_path: CSV 文件路径。
        required_columns: 必须存在的列名集合。

    返回:
        dict[str, np.ndarray]: 以列名为键、数据数组为值的字典。
    """

    with csv_path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError("CSV 文件缺少表头。")

        missing_columns = [column for column in required_columns if column not in reader.fieldnames]
        if missing_columns:
            raise ValueError(f"CSV 文件缺少必要列: {', '.join(missing_columns)}")

        buffers = {column: [] for column in required_columns}
        for row_index, row in enumerate(reader, start=2):
            try:
                for column in required_columns:
                    buffers[column].append(float(row[column]))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"第 {row_index} 行列 {column} 不是合法浮点数。") from exc

    return {column: np.asarray(values, dtype=np.float64) for column, values in buffers.items()}


def bridge_short_gaps(mask: np.ndarray, max_gap_samples: int) -> np.ndarray:
    """桥接被有效段夹住的短时失效缺口。

    参数:
        mask: 原始有效掩码。
        max_gap_samples: 允许桥接的最大缺口长度。

    返回:
        np.ndarray: 桥接后的有效掩码。
    """

    bridged_mask = mask.copy()
    index = 0
    while index < bridged_mask.size:
        if bridged_mask[index]:
            index += 1
            continue

        gap_end = index
        while gap_end < bridged_mask.size and not bridged_mask[gap_end]:
            gap_end += 1

        if (
            index > 0
            and gap_end < bridged_mask.size
            and bridged_mask[index - 1]
            and bridged_mask[gap_end]
            and gap_end - index <= max_gap_samples
        ):
            bridged_mask[index:gap_end] = True

        index = gap_end

    return bridged_mask


def extract_segments(mask: np.ndarray, min_segment_samples: int) -> list[tuple[int, int]]:
    """从有效掩码中提取稳定段。

    参数:
        mask: 有效掩码。
        min_segment_samples: 最小稳定段长度。

    返回:
        list[tuple[int, int]]: 稳定段列表，每项为 [start, end)。
    """

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


def apply_ema_by_segments(values: np.ndarray, segments: list[tuple[int, int]], alpha: float) -> np.ndarray:
    """在每个稳定段内执行 EMA。

    参数:
        values: 原始信号。
        segments: 稳定段列表。
        alpha: EMA 系数。

    返回:
        np.ndarray: 拼接后的 EMA 结果。
    """

    filtered_segments = []
    for start, end in segments:
        segment = values[start:end]
        filtered = np.empty_like(segment, dtype=np.float64)
        filtered[0] = segment[0]
        for index in range(1, segment.size):
            filtered[index] = alpha * segment[index] + (1.0 - alpha) * filtered[index - 1]
        filtered_segments.append(filtered)
    return np.concatenate(filtered_segments, axis=0)


def apply_cma_by_segments(values: np.ndarray, segments: list[tuple[int, int]], window_size: int) -> np.ndarray:
    """在每个稳定段内执行因果滑动平均。

    参数:
        values: 原始信号。
        segments: 稳定段列表。
        window_size: 均值窗口长度。

    返回:
        np.ndarray: 拼接后的 CMA 结果。
    """

    filtered_segments = []
    for start, end in segments:
        segment = values[start:end]
        filtered = np.empty_like(segment, dtype=np.float64)
        accumulator = 0.0
        for index in range(segment.size):
            accumulator += segment[index]
            if index >= window_size:
                accumulator -= segment[index - window_size]
                filtered[index] = accumulator / float(window_size)
            else:
                filtered[index] = accumulator / float(index + 1)
        filtered_segments.append(filtered)
    return np.concatenate(filtered_segments, axis=0)


def concatenate_segments(values: np.ndarray, segments: list[tuple[int, int]]) -> np.ndarray:
    """按稳定段拼接原始信号。

    参数:
        values: 原始信号。
        segments: 稳定段列表。

    返回:
        np.ndarray: 拼接后的信号。
    """

    return np.concatenate([values[start:end] for start, end in segments], axis=0)


def compute_peak_metrics(residual: np.ndarray, reference: np.ndarray) -> tuple[float, float, float]:
    """计算整体与高峰区残差 RMS。

    参数:
        residual: 残差序列。
        reference: 参考序列，用于判定高峰区。

    返回:
        tuple[float, float, float]:
            - rms_all: 全样本残差 RMS。
            - rms_q95: 95 分位高峰区残差 RMS。
            - rms_q98: 98 分位更强高峰区残差 RMS。
    """

    abs_reference = np.abs(reference)
    q95 = float(np.quantile(abs_reference, 0.95))
    q98 = float(np.quantile(abs_reference, 0.98))
    rms_all = float(np.sqrt(np.mean(residual * residual)))
    rms_q95 = float(np.sqrt(np.mean(residual[abs_reference >= q95] ** 2)))
    rms_q98 = float(np.sqrt(np.mean(residual[abs_reference >= q98] ** 2)))
    return rms_all, rms_q95, rms_q98


def fit_linear(target: np.ndarray, reference: np.ndarray) -> LinearFitResult:
    """拟合单系数线性补偿模型。

    参数:
        target: 目标序列。
        reference: 参考序列。

    返回:
        LinearFitResult: 线性拟合结果。
    """

    scale = float(np.dot(reference, target) / np.dot(reference, reference))
    residual = target - scale * reference
    rms_all, rms_q95, rms_q98 = compute_peak_metrics(residual, reference)
    return LinearFitResult(
        scale=scale,
        residual=residual,
        rms_all=rms_all,
        rms_q95=rms_q95,
        rms_q98=rms_q98,
    )


def fit_peak_suppress(target: np.ndarray, reference: np.ndarray) -> NonlinearFitResult:
    """拟合带峰值抑制项的非线性补偿模型。

    参数:
        target: 目标序列。
        reference: 参考序列。

    返回:
        NonlinearFitResult: 非线性峰值抑制拟合结果。
    """

    design_matrix = np.column_stack([reference, reference * np.abs(reference)])
    coefficients, *_ = np.linalg.lstsq(design_matrix, target, rcond=None)
    prediction = design_matrix @ coefficients
    residual = target - prediction
    rms_all, rms_q95, rms_q98 = compute_peak_metrics(residual, reference)
    return NonlinearFitResult(
        linear_gain=float(coefficients[0]),
        peak_gain=float(coefficients[1]),
        residual=residual,
        rms_all=rms_all,
        rms_q95=rms_q95,
        rms_q98=rms_q98,
    )


def save_compare_plot(
    output_path: Path,
    sample_rate_hz: float,
    axis_x_reference: np.ndarray,
    axis_y_reference: np.ndarray,
    linear_x: LinearFitResult,
    nonlinear_x: NonlinearFitResult,
    linear_y: LinearFitResult,
    nonlinear_y: NonlinearFitResult,
) -> None:
    """保存线性方案与峰值抑制方案的残差对比图。

    参数:
        output_path: 输出图片路径。
        sample_rate_hz: 采样频率。
        axis_x_reference: X 轴参考序列。
        axis_y_reference: Y 轴参考序列。
        linear_x: X 轴线性拟合结果。
        nonlinear_x: X 轴非线性拟合结果。
        linear_y: Y 轴线性拟合结果。
        nonlinear_y: Y 轴非线性拟合结果。

    返回:
        无。
    """

    time_axis_x = np.arange(linear_x.residual.size, dtype=np.float64) / sample_rate_hz
    time_axis_y = np.arange(linear_y.residual.size, dtype=np.float64) / sample_rate_hz
    figure, axes = plt.subplots(2, 2, figsize=(15, 9))

    axes[0, 0].plot(time_axis_x, linear_x.residual, linewidth=0.8, color="#f77f00", label="linear residual")
    axes[0, 0].plot(time_axis_x, nonlinear_x.residual, linewidth=0.9, color="#1d3557", label="peak-suppress residual")
    axes[0, 0].set_title("X axis residual over time")
    axes[0, 0].set_xlabel("time / s")
    axes[0, 0].set_ylabel("residual")
    axes[0, 0].grid(True, alpha=0.25)
    axes[0, 0].legend(loc="upper right")

    axes[0, 1].plot(time_axis_y, linear_y.residual, linewidth=0.8, color="#f77f00", label="linear residual")
    axes[0, 1].plot(time_axis_y, nonlinear_y.residual, linewidth=0.9, color="#1d3557", label="peak-suppress residual")
    axes[0, 1].set_title("Y axis residual over time")
    axes[0, 1].set_xlabel("time / s")
    axes[0, 1].set_ylabel("residual")
    axes[0, 1].grid(True, alpha=0.25)
    axes[0, 1].legend(loc="upper right")

    abs_x = np.abs(axis_x_reference)
    x_peak_order = np.argsort(abs_x)
    axes[1, 0].plot(abs_x[x_peak_order], np.abs(linear_x.residual[x_peak_order]), linewidth=0.8, color="#f77f00", label="linear |residual|")
    axes[1, 0].plot(abs_x[x_peak_order], np.abs(nonlinear_x.residual[x_peak_order]), linewidth=0.9, color="#1d3557", label="peak-suppress |residual|")
    axes[1, 0].set_title("X axis peak-zone residual")
    axes[1, 0].set_xlabel("|reference|")
    axes[1, 0].set_ylabel("|residual|")
    axes[1, 0].grid(True, alpha=0.25)
    axes[1, 0].legend(loc="upper right")

    abs_y = np.abs(axis_y_reference)
    y_peak_order = np.argsort(abs_y)
    axes[1, 1].plot(abs_y[y_peak_order], np.abs(linear_y.residual[y_peak_order]), linewidth=0.8, color="#f77f00", label="linear |residual|")
    axes[1, 1].plot(abs_y[y_peak_order], np.abs(nonlinear_y.residual[y_peak_order]), linewidth=0.9, color="#1d3557", label="peak-suppress |residual|")
    axes[1, 1].set_title("Y axis peak-zone residual")
    axes[1, 1].set_xlabel("|reference|")
    axes[1, 1].set_ylabel("|residual|")
    axes[1, 1].grid(True, alpha=0.25)
    axes[1, 1].legend(loc="upper right")

    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def main() -> int:
    """执行峰值抑制模型研究并输出结果。

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
        print("未找到满足条件的稳定有效段。")
        return 2

    filtered_i0 = apply_ema_by_segments(columns["I0"], segments, args.ema_alpha)
    filtered_i1 = apply_ema_by_segments(columns["I1"], segments, args.ema_alpha)
    filtered_i5 = apply_cma_by_segments(columns["I5"], segments, args.cma_window)
    filtered_i6 = apply_cma_by_segments(columns["I6"], segments, args.cma_window)

    linear_x = fit_linear(filtered_i0, filtered_i5)
    linear_y = fit_linear(filtered_i1, filtered_i6)
    nonlinear_x = fit_peak_suppress(filtered_i0, filtered_i5)
    nonlinear_y = fit_peak_suppress(filtered_i1, filtered_i6)

    plot_path = output_dir / f"{csv_path.stem}_peak_suppress_compare.png"
    save_compare_plot(
        output_path=plot_path,
        sample_rate_hz=args.sample_rate,
        axis_x_reference=filtered_i5,
        axis_y_reference=filtered_i6,
        linear_x=linear_x,
        nonlinear_x=nonlinear_x,
        linear_y=linear_y,
        nonlinear_y=nonlinear_y,
    )

    print(f"文件: {csv_path.name}")
    print(
        "预处理逻辑: "
        f"I0/I1 使用 EMA(alpha={args.ema_alpha:.2f})，"
        f"I5/I6 使用 CMA{args.cma_window}；"
        f"有效段规则为 {args.gate_column} > {args.threshold:.3f}，桥接 <= {args.max_gap_samples} 点短缺口，保留 >= {args.min_segment_samples} 点稳定段。"
    )
    print(
        "X 轴线性模型: "
        f"I0_f - ({linear_x.scale:.10f}) * I5_f，"
        f"RMS(all/q95/q98) = {linear_x.rms_all:.10f} / {linear_x.rms_q95:.10f} / {linear_x.rms_q98:.10f}。"
    )
    print(
        "X 轴峰值抑制模型: "
        f"I0_f - ({nonlinear_x.linear_gain:.10f} * I5_f + {nonlinear_x.peak_gain:.10f} * I5_f * abs(I5_f))，"
        f"RMS(all/q95/q98) = {nonlinear_x.rms_all:.10f} / {nonlinear_x.rms_q95:.10f} / {nonlinear_x.rms_q98:.10f}。"
    )
    print(
        "Y 轴线性模型: "
        f"I1_f - ({linear_y.scale:.10f}) * I6_f，"
        f"RMS(all/q95/q98) = {linear_y.rms_all:.10f} / {linear_y.rms_q95:.10f} / {linear_y.rms_q98:.10f}。"
    )
    print(
        "Y 轴峰值抑制模型: "
        f"I1_f - ({nonlinear_y.linear_gain:.10f} * I6_f + {nonlinear_y.peak_gain:.10f} * I6_f * abs(I6_f))，"
        f"RMS(all/q95/q98) = {nonlinear_y.rms_all:.10f} / {nonlinear_y.rms_q95:.10f} / {nonlinear_y.rms_q98:.10f}。"
    )
    print(f"对比图: {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
