from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np


# 输入文件路径：原始采样数据 CSV。
INPUT_CSV_PATH = Path(__file__).with_name("04152218.csv")
# 输出文件路径：保留原文件不动，生成补齐并追加新列后的备份文件。
OUTPUT_CSV_PATH = Path(__file__).with_name("04152218_backup.csv")

# 列索引定义：和原始 wifi_justfloat 输出顺序保持一致。
COL_TICK = 0
COL_HEIGHT_MM = 1
COL_GYRO_X = 2
COL_GYRO_Y = 3
COL_GYRO_Z = 4
COL_PMW_X = 5
COL_PMW_Y = 6
COL_PMW_SQUAL = 7
COL_LC_X = 8
COL_LC_Y = 9
COL_ACC_X = 10
COL_ACC_Y = 11

# 1kHz 列：这些信号按用户要求在丢包时做线性插值补齐。
INTERP_1KHZ_COLUMNS = (
    COL_GYRO_X,
    COL_GYRO_Y,
    COL_GYRO_Z,
    COL_ACC_X,
    COL_ACC_Y,
)
# 非 1kHz 列：扩展到完整时间轴时采用最近一次有效值保持。
HOLD_COLUMNS = (
    COL_HEIGHT_MM,
    COL_PMW_X,
    COL_PMW_Y,
    COL_PMW_SQUAL,
    COL_LC_X,
    COL_LC_Y,
)

# 50Hz 窗口参数：本文件中有效光流窗口的最佳相位起点和容忍延迟。
FLOW_SAMPLE_START_TICK = 57283
FLOW_SAMPLE_PERIOD_MS = 20
FLOW_SAMPLE_MAX_LAG_MS = 8
PMW_SQUAL_MIN = 20.0

# PMW3901 解耦参数：一阶低通截止频率和对应的积分消除倍率。
PMW_GYRO_LP_CUTOFF_HZ = 10.95
PMW_X_GAIN = 0.007282257614242992
PMW_Y_GAIN = 0.008605754110750803

# LC302 解耦参数：一阶低通截止频率和对应的积分消除倍率。
LC_GYRO_LP_CUTOFF_HZ = 3.675
LC_X_GAIN = 0.16008104771702067
LC_Y_GAIN = 0.2026204371214993


def load_csv_rows(csv_path: Path) -> tuple[list[str], np.ndarray]:
    """读取原始 CSV。

    参数:
    - csv_path: 输入 CSV 文件路径。

    返回:
    - header: 表头列表。
    - rows: 浮点型二维数组，形状为 [N, 12]。
    """

    with csv_path.open("r", encoding="utf-8", newline="") as csv_file:
        reader = csv.reader(csv_file)
        header = next(reader)
        rows = [[float(cell) for cell in row] for row in reader]
    return header, np.asarray(rows, dtype=float)


def first_order_lowpass(signal: np.ndarray, cutoff_hz: float, dt_s: float) -> tuple[np.ndarray, float]:
    """对角速度序列执行一阶低通滤波。

    参数:
    - signal: 输入角速度序列。
    - cutoff_hz: 截止频率，单位 Hz。
    - dt_s: 采样周期，单位秒。

    返回:
    - filtered: 低通后的角速度序列。
    - alpha: 低通离散系数。
    """

    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    alpha = dt_s / (rc + dt_s)
    filtered = np.empty_like(signal)
    filtered[0] = signal[0]

    # 递推执行一阶低通，强行把陀螺相位往光流后面拖。
    for index in range(1, signal.shape[0]):
        filtered[index] = filtered[index - 1] + alpha * (signal[index] - filtered[index - 1])

    return filtered, alpha


def expand_to_full_tick(rows: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """把原始 CSV 扩成完整 1ms 时间轴。

    参数:
    - rows: 原始数据数组。

    返回:
    - full_ticks: 完整毫秒时间轴。
    - full_rows: 补齐后的完整数据数组。
    """

    original_ticks = np.rint(rows[:, COL_TICK]).astype(np.int64)
    full_ticks = np.arange(original_ticks[0], original_ticks[-1] + 1, dtype=np.int64)
    full_rows = np.zeros((full_ticks.shape[0], rows.shape[1]), dtype=float)
    full_rows[:, COL_TICK] = full_ticks

    # 1kHz 信号按用户要求在线性插值补齐。
    for column in INTERP_1KHZ_COLUMNS:
        full_rows[:, column] = np.interp(full_ticks, original_ticks, rows[:, column])

    # 其他列沿用最近一次接收到的值，避免擅自捏造额外动态。
    tick_pos = np.searchsorted(original_ticks, full_ticks, side="right") - 1
    tick_pos = np.clip(tick_pos, 0, rows.shape[0] - 1)
    for column in HOLD_COLUMNS:
        full_rows[:, column] = rows[tick_pos, column]

    return full_ticks, full_rows


def build_sample_schedule(original_ticks: np.ndarray) -> list[tuple[int, int]]:
    """构建 50Hz 光流有效窗口和对应观测行。

    参数:
    - original_ticks: 原始 CSV 中存在的 tick 序列。

    返回:
    - sample_pairs: 每个元素为 (窗口基准 tick, 实际观测 tick)。
    """

    sample_pairs: list[tuple[int, int]] = []
    for base_tick in range(FLOW_SAMPLE_START_TICK, int(original_ticks[-1]) + 1, FLOW_SAMPLE_PERIOD_MS):
        row_index = int(np.searchsorted(original_ticks, base_tick, side="left"))
        if row_index >= original_ticks.shape[0]:
            break

        observed_tick = int(original_ticks[row_index])
        if observed_tick - base_tick > FLOW_SAMPLE_MAX_LAG_MS:
            continue

        sample_pairs.append((base_tick, observed_tick))

    return sample_pairs


def assign_held_values(full_ticks: np.ndarray, sample_ticks: list[int], sample_values: list[str]) -> list[str]:
    """把 50Hz 结果按保持方式铺到完整时间轴。

    参数:
    - full_ticks: 完整毫秒时间轴。
    - sample_ticks: 有效样本写入 tick。
    - sample_values: 对应样本值，空字符串代表该窗口无效。

    返回:
    - held_values: 与 full_ticks 同长度的字符串列表。
    """

    held_values = [""] * full_ticks.shape[0]
    if not sample_ticks:
        return held_values

    full_start_tick = int(full_ticks[0])
    full_end_tick = int(full_ticks[-1])

    for sample_index, sample_tick in enumerate(sample_ticks):
        if sample_tick < full_start_tick or sample_tick > full_end_tick:
            continue

        value_text = sample_values[sample_index]
        next_tick = sample_ticks[sample_index + 1] if sample_index + 1 < len(sample_ticks) else full_end_tick + 1
        start_index = sample_tick - full_start_tick
        stop_index = min(next_tick - full_start_tick, held_values.__len__())

        # 当前窗口无效时整段留空，下一次有效窗口再恢复。
        if value_text == "":
            continue

        for full_index in range(start_index, stop_index):
            held_values[full_index] = value_text

    return held_values


def format_float(value: float) -> str:
    """统一 CSV 浮点输出格式。

    参数:
    - value: 需要写入的浮点值。

    返回:
    - 格式化后的字符串。
    """

    return f"{value:.6f}"


def compute_decoupled_columns(rows: np.ndarray, full_ticks: np.ndarray, full_rows: np.ndarray) -> dict[str, list[str]]:
    """按指定算法计算四路姿态解耦后的光流结果。

    参数:
    - rows: 原始数据数组。
    - full_ticks: 完整毫秒时间轴。
    - full_rows: 补齐后的完整数据数组。

    返回:
    - decoupled_columns: 四列输出，键为 I12/I13/I14/I15。
    """

    original_ticks = np.rint(rows[:, COL_TICK]).astype(np.int64)
    sample_pairs = build_sample_schedule(original_ticks)
    sample_base_ticks = [base_tick for base_tick, _ in sample_pairs]
    sample_observed_ticks = [observed_tick for _, observed_tick in sample_pairs]
    sample_row_indices = np.searchsorted(original_ticks, sample_observed_ticks, side="left")

    pmw_gyro_x_lp, pmw_alpha = first_order_lowpass(full_rows[:, COL_GYRO_X], PMW_GYRO_LP_CUTOFF_HZ, 0.001)
    pmw_gyro_y_lp, _ = first_order_lowpass(full_rows[:, COL_GYRO_Y], PMW_GYRO_LP_CUTOFF_HZ, 0.001)
    lc_gyro_x_lp, lc_alpha = first_order_lowpass(full_rows[:, COL_GYRO_X], LC_GYRO_LP_CUTOFF_HZ, 0.001)
    lc_gyro_y_lp, _ = first_order_lowpass(full_rows[:, COL_GYRO_Y], LC_GYRO_LP_CUTOFF_HZ, 0.001)

    pmw_gyro_x_sum = np.concatenate(([0.0], np.cumsum(pmw_gyro_x_lp)))
    pmw_gyro_y_sum = np.concatenate(([0.0], np.cumsum(pmw_gyro_y_lp)))
    lc_gyro_x_sum = np.concatenate(([0.0], np.cumsum(lc_gyro_x_lp)))
    lc_gyro_y_sum = np.concatenate(([0.0], np.cumsum(lc_gyro_y_lp)))

    pmw_x_samples: list[str] = []
    pmw_y_samples: list[str] = []
    lc_x_samples: list[str] = []
    lc_y_samples: list[str] = []

    full_start_tick = int(full_ticks[0])

    # 每个 20ms 窗口都减掉同窗口内低通角速度积分，倍率固定为拟合最优值。
    for sample_index, base_tick in enumerate(sample_base_ticks):
        observed_row = rows[sample_row_indices[sample_index]]
        end_index = base_tick - full_start_tick + 1
        start_index = end_index - FLOW_SAMPLE_PERIOD_MS
        if start_index < 0:
            pmw_x_samples.append("")
            pmw_y_samples.append("")
            lc_x_samples.append("")
            lc_y_samples.append("")
            continue

        pmw_gyro_x_integral = pmw_gyro_x_sum[end_index] - pmw_gyro_x_sum[start_index]
        pmw_gyro_y_integral = pmw_gyro_y_sum[end_index] - pmw_gyro_y_sum[start_index]
        lc_gyro_x_integral = lc_gyro_x_sum[end_index] - lc_gyro_x_sum[start_index]
        lc_gyro_y_integral = lc_gyro_y_sum[end_index] - lc_gyro_y_sum[start_index]

        lc_x_value = observed_row[COL_LC_X] - LC_X_GAIN * lc_gyro_x_integral
        lc_y_value = observed_row[COL_LC_Y] - LC_Y_GAIN * lc_gyro_y_integral
        lc_x_samples.append(format_float(lc_x_value))
        lc_y_samples.append(format_float(lc_y_value))

        # PMW 的 squal 低于门限直接丢弃这个窗口，别让脏数据混进来。
        if observed_row[COL_PMW_SQUAL] < PMW_SQUAL_MIN:
            pmw_x_samples.append("")
            pmw_y_samples.append("")
            continue

        pmw_x_value = observed_row[COL_PMW_X] - PMW_X_GAIN * pmw_gyro_x_integral
        pmw_y_value = observed_row[COL_PMW_Y] - PMW_Y_GAIN * pmw_gyro_y_integral
        pmw_x_samples.append(format_float(pmw_x_value))
        pmw_y_samples.append(format_float(pmw_y_value))

    print(f"PMW 一阶低通 alpha={pmw_alpha:.6f}, 截止频率={PMW_GYRO_LP_CUTOFF_HZ:.3f}Hz")
    print(f"LC302 一阶低通 alpha={lc_alpha:.6f}, 截止频率={LC_GYRO_LP_CUTOFF_HZ:.3f}Hz")
    print(f"50Hz 有效窗口数量={len(sample_pairs)}")
    print(
        "解耦倍率: "
        f"PMW_X={PMW_X_GAIN:.12f}, PMW_Y={PMW_Y_GAIN:.12f}, "
        f"LC_X={LC_X_GAIN:.12f}, LC_Y={LC_Y_GAIN:.12f}"
    )

    return {
        "I12": assign_held_values(full_ticks, sample_observed_ticks, pmw_x_samples),
        "I13": assign_held_values(full_ticks, sample_observed_ticks, pmw_y_samples),
        "I14": assign_held_values(full_ticks, sample_observed_ticks, lc_x_samples),
        "I15": assign_held_values(full_ticks, sample_observed_ticks, lc_y_samples),
    }


def write_output_csv(header: list[str], full_rows: np.ndarray, decoupled_columns: dict[str, list[str]], output_path: Path) -> None:
    """写出补齐后的备份 CSV。

    参数:
    - header: 原始表头。
    - full_rows: 补齐后的完整数据数组。
    - decoupled_columns: 追加的四列结果。
    - output_path: 输出文件路径。
    """

    output_header = [*header, "I12", "I13", "I14", "I15"]
    with output_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(output_header)
        for row_index in range(full_rows.shape[0]):
            base_row = [format_float(value) for value in full_rows[row_index]]
            writer.writerow(
                [
                    *base_row,
                    decoupled_columns["I12"][row_index],
                    decoupled_columns["I13"][row_index],
                    decoupled_columns["I14"][row_index],
                    decoupled_columns["I15"][row_index],
                ]
            )


def main() -> None:
    """执行 04152218.csv 的补齐与光流姿态解耦处理。"""

    header, rows = load_csv_rows(INPUT_CSV_PATH)
    full_ticks, full_rows = expand_to_full_tick(rows)
    decoupled_columns = compute_decoupled_columns(rows, full_ticks, full_rows)
    write_output_csv(header, full_rows, decoupled_columns, OUTPUT_CSV_PATH)
    print(f"输出完成: {OUTPUT_CSV_PATH}")


if __name__ == "__main__":
    main()
