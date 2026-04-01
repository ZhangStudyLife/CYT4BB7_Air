#!/usr/bin/env python3
"""研究 better_vel_V2.csv 的限幅防过补偿方案。"""

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
# 默认门控列名。
DEFAULT_GATE_COLUMN = "I2"
# 默认门控阈值。
DEFAULT_GATE_THRESHOLD = 40.0
# 默认短缺口桥接长度，单位采样点。
DEFAULT_MAX_GAP_SAMPLES = 3
# 默认最小稳定段长度，单位采样点。
DEFAULT_MIN_SEGMENT_SAMPLES = 50
# 默认 EMA 系数。
DEFAULT_EMA_ALPHA = 0.40
# 默认 CMA 窗口长度。
DEFAULT_CMA_WINDOW = 5
# 默认采样频率，单位 Hz。
DEFAULT_SAMPLE_RATE_HZ = 100.0
# 默认输出目录。
DEFAULT_OUTPUT_DIR = "fit_plots_better_vel_v2_clip"


@dataclass(frozen=True)
class ClipFitResult:
    """保存单通道限幅拟合结果。

    参数:
        limit_value: 最优限幅值。
        scale: 最优线性系数。
        residual: 残差序列。
        rms_all: 全体样本残差 RMS。
        rms_q95: 高峰区残差 RMS，按 |reference| 的 95 分位阈值统计。
        rms_q98: 更强高峰区残差 RMS，按 |reference| 的 98 分位阈值统计。
        clipped_reference: 限幅后的参考序列。

    返回:
        ClipFitResult: 单通道限幅拟合结果对象。
    """

    limit_value: float
    scale: float
    residual: np.ndarray
    rms_all: float
    rms_q95: float
    rms_q98: float
    clipped_reference: np.ndarray


def parse_args() -> argparse.Namespace:
    """解析命令行参数。

    参数:
        无。

    返回:
        argparse.Namespace: 命令行参数对象。
    """

    parser = argparse.ArgumentParser(
        description="对 better_vel_V2.csv 搜索低复杂度限幅防过补偿参数，并保存对比图。"
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


def compute_rms_metrics(residual: np.ndarray, reference: np.ndarray) -> tuple[float, float, float]:
    """计算整体与高峰区残差 RMS。

    参数:
        residual: 残差序列。
        reference: 参考序列，用于判定高峰区。

    返回:
        tuple[float, float, float]:
            - rms_all: 全样本残差 RMS。
            - rms_q95: 95 分位高峰区残差 RMS。
            - rms_q98: 98 分位高峰区残差 RMS。
    """

    abs_reference = np.abs(reference)
    q95 = float(np.quantile(abs_reference, 0.95))
    q98 = float(np.quantile(abs_reference, 0.98))
    rms_all = float(np.sqrt(np.mean(residual * residual)))
    rms_q95 = float(np.sqrt(np.mean(residual[abs_reference >= q95] ** 2)))
    rms_q98 = float(np.sqrt(np.mean(residual[abs_reference >= q98] ** 2)))
    return rms_all, rms_q95, rms_q98


def fit_linear(target: np.ndarray, reference: np.ndarray) -> tuple[float, np.ndarray, float, float, float]:
    """拟合基础线性补偿。

    参数:
        target: 目标序列。
        reference: 参考序列。

    返回:
        tuple[float, np.ndarray, float, float, float]:
            - scale: 线性系数。
            - residual: 残差序列。
            - rms_all: 全样本残差 RMS。
            - rms_q95: 高峰区残差 RMS。
            - rms_q98: 更强高峰区残差 RMS。
    """

    scale = float(np.dot(reference, target) / np.dot(reference, reference))
    residual = target - scale * reference
    rms_all, rms_q95, rms_q98 = compute_rms_metrics(residual, reference)
    return scale, residual, rms_all, rms_q95, rms_q98


def fit_best_hard_clip(target: np.ndarray, reference: np.ndarray) -> ClipFitResult:
    """搜索最优硬限幅防过补偿方案。

    参数:
        target: 目标序列。
        reference: 参考序列。

    返回:
        ClipFitResult: 最优硬限幅拟合结果。
    """

    abs_reference = np.abs(reference)
    limit_candidates = sorted({float(value) for value in np.quantile(abs_reference, [0.85, 0.90, 0.92, 0.95, 0.97, 0.98])})
    best_result: ClipFitResult | None = None
    best_score: float | None = None

    for limit_value in limit_candidates:
        clipped_reference = np.clip(reference, -limit_value, limit_value)
        scale = float(np.dot(clipped_reference, target) / np.dot(clipped_reference, clipped_reference))
        residual = target - scale * clipped_reference
        rms_all, rms_q95, rms_q98 = compute_rms_metrics(residual, reference)

        # 联合考虑整体与高峰区残差，偏向高峰区压制。
        score = rms_all + 0.04 * rms_q95 + 0.07 * rms_q98
        candidate_result = ClipFitResult(
            limit_value=limit_value,
            scale=scale,
            residual=residual,
            rms_all=rms_all,
            rms_q95=rms_q95,
            rms_q98=rms_q98,
            clipped_reference=clipped_reference,
        )

        if best_score is None or score < best_score:
            best_score = score
            best_result = candidate_result

    if best_result is None:
        raise ValueError("没有可用的限幅候选。")

    return best_result


def save_compare_plot(
    output_path: Path,
    sample_rate_hz: float,
    reference_x: np.ndarray,
    reference_y: np.ndarray,
    residual_x_linear: np.ndarray,
    residual_x_clip: np.ndarray,
    residual_y_linear: np.ndarray,
    residual_y_clip: np.ndarray,
) -> None:
    """保存基础线性方案与限幅防过补偿方案的对比图。

    参数:
        output_path: 输出图片路径。
        sample_rate_hz: 采样频率。
        reference_x: X 轴参考序列。
        reference_y: Y 轴参考序列。
        residual_x_linear: X 轴基础线性残差。
        residual_x_clip: X 轴限幅残差。
        residual_y_linear: Y 轴基础线性残差。
        residual_y_clip: Y 轴限幅残差。

    返回:
        无。
    """

    figure, axes = plt.subplots(2, 2, figsize=(15, 9))
    time_axis_x = np.arange(residual_x_linear.size, dtype=np.float64) / sample_rate_hz
    time_axis_y = np.arange(residual_y_linear.size, dtype=np.float64) / sample_rate_hz

    axes[0, 0].plot(time_axis_x, residual_x_linear, linewidth=0.8, color="#f77f00", label="linear residual")
    axes[0, 0].plot(time_axis_x, residual_x_clip, linewidth=0.9, color="#1d3557", label="clip-guard residual")
    axes[0, 0].set_title("X axis residual over time")
    axes[0, 0].set_xlabel("time / s")
    axes[0, 0].set_ylabel("residual")
    axes[0, 0].grid(True, alpha=0.25)
    axes[0, 0].legend(loc="upper right")

    axes[0, 1].plot(time_axis_y, residual_y_linear, linewidth=0.8, color="#f77f00", label="linear residual")
    axes[0, 1].plot(time_axis_y, residual_y_clip, linewidth=0.9, color="#1d3557", label="clip-guard residual")
    axes[0, 1].set_title("Y axis residual over time")
    axes[0, 1].set_xlabel("time / s")
    axes[0, 1].set_ylabel("residual")
    axes[0, 1].grid(True, alpha=0.25)
    axes[0, 1].legend(loc="upper right")

    abs_reference_x = np.abs(reference_x)
    sort_index_x = np.argsort(abs_reference_x)
    axes[1, 0].plot(abs_reference_x[sort_index_x], np.abs(residual_x_linear[sort_index_x]), linewidth=0.8, color="#f77f00", label="linear |residual|")
    axes[1, 0].plot(abs_reference_x[sort_index_x], np.abs(residual_x_clip[sort_index_x]), linewidth=0.9, color="#1d3557", label="clip-guard |residual|")
    axes[1, 0].set_title("X axis peak-zone residual")
    axes[1, 0].set_xlabel("|reference|")
    axes[1, 0].set_ylabel("|residual|")
    axes[1, 0].grid(True, alpha=0.25)
    axes[1, 0].legend(loc="upper right")

    abs_reference_y = np.abs(reference_y)
    sort_index_y = np.argsort(abs_reference_y)
    axes[1, 1].plot(abs_reference_y[sort_index_y], np.abs(residual_y_linear[sort_index_y]), linewidth=0.8, color="#f77f00", label="linear |residual|")
    axes[1, 1].plot(abs_reference_y[sort_index_y], np.abs(residual_y_clip[sort_index_y]), linewidth=0.9, color="#1d3557", label="clip-guard |residual|")
    axes[1, 1].set_title("Y axis peak-zone residual")
    axes[1, 1].set_xlabel("|reference|")
    axes[1, 1].set_ylabel("|residual|")
    axes[1, 1].grid(True, alpha=0.25)
    axes[1, 1].legend(loc="upper right")

    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def main() -> int:
    """执行限幅防过补偿研究并输出结果。

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

    linear_k, residual_x_linear, rms_x_linear, rms_x_linear_q95, rms_x_linear_q98 = fit_linear(filtered_i0, filtered_i5)
    linear_m, residual_y_linear, rms_y_linear, rms_y_linear_q95, rms_y_linear_q98 = fit_linear(filtered_i1, filtered_i6)
    clip_x = fit_best_hard_clip(filtered_i0, filtered_i5)
    clip_y = fit_best_hard_clip(filtered_i1, filtered_i6)

    plot_path = output_dir / f"{csv_path.stem}_clip_guard_compare.png"
    save_compare_plot(
        output_path=plot_path,
        sample_rate_hz=args.sample_rate,
        reference_x=filtered_i5,
        reference_y=filtered_i6,
        residual_x_linear=residual_x_linear,
        residual_x_clip=clip_x.residual,
        residual_y_linear=residual_y_linear,
        residual_y_clip=clip_y.residual,
    )

    print(f"文件: {csv_path.name}")
    print(
        "预处理逻辑: "
        f"I0/I1 使用 EMA(alpha={args.ema_alpha:.2f})，"
        f"I5/I6 使用 CMA{args.cma_window}；"
        f"有效段规则为 {args.gate_column} > {args.threshold:.3f}，桥接 <= {args.max_gap_samples} 点短缺口，保留 >= {args.min_segment_samples} 点稳定段。"
    )
    print(
        "X 轴基础线性: "
        f"I0_f - ({linear_k:.10f}) * I5_f，"
        f"RMS(all/q95/q98) = {rms_x_linear:.10f} / {rms_x_linear_q95:.10f} / {rms_x_linear_q98:.10f}。"
    )
    print(
        "X 轴限幅方案: "
        f"I0_f - ({clip_x.scale:.10f}) * clip(I5_f, -{clip_x.limit_value:.10f}, {clip_x.limit_value:.10f})，"
        f"RMS(all/q95/q98) = {clip_x.rms_all:.10f} / {clip_x.rms_q95:.10f} / {clip_x.rms_q98:.10f}。"
    )
    print(
        "Y 轴基础线性: "
        f"I1_f - ({linear_m:.10f}) * I6_f，"
        f"RMS(all/q95/q98) = {rms_y_linear:.10f} / {rms_y_linear_q95:.10f} / {rms_y_linear_q98:.10f}。"
    )
    print(
        "Y 轴限幅方案: "
        f"I1_f - ({clip_y.scale:.10f}) * clip(I6_f, -{clip_y.limit_value:.10f}, {clip_y.limit_value:.10f})，"
        f"RMS(all/q95/q98) = {clip_y.rms_all:.10f} / {clip_y.rms_q95:.10f} / {clip_y.rms_q98:.10f}。"
    )
    print(f"对比图: {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
