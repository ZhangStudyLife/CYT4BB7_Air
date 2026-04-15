from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np


INPUT_CSV_PATH = Path(__file__).with_name("04152218.csv")
OUTPUT_CSV_PATH = Path(__file__).with_name("04152218_compare.csv")

COL_TICK = 0
COL_GYRO_X = 2
COL_GYRO_Y = 3
COL_PMW_X = 5
COL_PMW_Y = 6
COL_PMW_SQUAL = 7
COL_LC_X = 8
COL_LC_Y = 9
COL_ACC_X = 10
COL_ACC_Y = 11

INTERP_1KHZ_COLUMNS = (COL_GYRO_X, COL_GYRO_Y, 4, COL_ACC_X, COL_ACC_Y)
HOLD_COLUMNS = (1, COL_PMW_X, COL_PMW_Y, COL_PMW_SQUAL, COL_LC_X, COL_LC_Y)

FLOW_SAMPLE_START_TICK = 57283
FLOW_SAMPLE_PERIOD_MS = 20
FLOW_SAMPLE_MAX_LAG_MS = 8
PMW_SQUAL_MIN = 20.0

PMW_X_CUTOFF_HZ = 10.0
PMW_Y_CUTOFF_HZ = 7.0
LC_X_CUTOFF_HZ = 4.0
LC_Y_CUTOFF_HZ = 3.0

PMW_X_GAIN = 8.009662
PMW_Y_GAIN = 9.120377
LC_X_GAIN = 170.882894
LC_Y_GAIN = 206.472807


def load_rows(csv_path: Path) -> np.ndarray:
    with csv_path.open("r", encoding="utf-8", newline="") as csv_file:
        reader = csv.reader(csv_file)
        next(reader)
        return np.asarray([[float(cell) for cell in row] for row in reader], dtype=float)


def first_order_lowpass(signal: np.ndarray, cutoff_hz: float, dt_s: float) -> tuple[np.ndarray, float]:
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    alpha = dt_s / (rc + dt_s)
    filtered = np.empty_like(signal)
    filtered[0] = signal[0]
    for index in range(1, signal.shape[0]):
        filtered[index] = filtered[index - 1] + alpha * (signal[index] - filtered[index - 1])
    return filtered, alpha


def expand_to_full_tick(rows: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    original_ticks = np.rint(rows[:, COL_TICK]).astype(np.int64)
    full_ticks = np.arange(original_ticks[0], original_ticks[-1] + 1, dtype=np.int64)
    full_rows = np.zeros((full_ticks.shape[0], rows.shape[1]), dtype=float)
    full_rows[:, COL_TICK] = full_ticks

    for column in INTERP_1KHZ_COLUMNS:
        full_rows[:, column] = np.interp(full_ticks, original_ticks, rows[:, column])

    tick_pos = np.searchsorted(original_ticks, full_ticks, side="right") - 1
    tick_pos = np.clip(tick_pos, 0, rows.shape[0] - 1)
    for column in HOLD_COLUMNS:
        full_rows[:, column] = rows[tick_pos, column]

    return full_ticks, full_rows


def build_sample_pairs(original_ticks: np.ndarray) -> list[tuple[int, int, int]]:
    sample_pairs: list[tuple[int, int, int]] = []
    for base_tick in range(FLOW_SAMPLE_START_TICK, int(original_ticks[-1]) + 1, FLOW_SAMPLE_PERIOD_MS):
        row_index = int(np.searchsorted(original_ticks, base_tick, side="left"))
        if row_index >= original_ticks.shape[0]:
            break
        observed_tick = int(original_ticks[row_index])
        if observed_tick - base_tick > FLOW_SAMPLE_MAX_LAG_MS:
            continue
        sample_pairs.append((base_tick, observed_tick, row_index))
    return sample_pairs


def format_float(value: float) -> str:
    return f"{value:.6f}"


def assign_held_values(full_ticks: np.ndarray, sample_ticks: list[int], sample_values: list[str]) -> list[str]:
    held_values = [""] * full_ticks.shape[0]
    if not sample_ticks:
        return held_values

    full_start_tick = int(full_ticks[0])
    full_end_tick = int(full_ticks[-1])
    for sample_index, sample_tick in enumerate(sample_ticks):
        value_text = sample_values[sample_index]
        next_tick = sample_ticks[sample_index + 1] if sample_index + 1 < len(sample_ticks) else full_end_tick + 1
        start_index = sample_tick - full_start_tick
        stop_index = min(next_tick - full_start_tick, len(held_values))
        if value_text == "":
            continue
        for full_index in range(start_index, stop_index):
            held_values[full_index] = value_text
    return held_values


def make_compare_columns(rows: np.ndarray, full_ticks: np.ndarray, full_rows: np.ndarray) -> list[list[str]]:
    original_ticks = np.rint(rows[:, COL_TICK]).astype(np.int64)
    sample_pairs = build_sample_pairs(original_ticks)
    sample_base_ticks = [item[0] for item in sample_pairs]
    sample_observed_ticks = [item[1] for item in sample_pairs]
    sample_row_indices = [item[2] for item in sample_pairs]
    full_start_tick = int(full_ticks[0])

    pmw_x_lp, pmw_x_alpha = first_order_lowpass(full_rows[:, COL_GYRO_X], PMW_X_CUTOFF_HZ, 0.001)
    pmw_y_lp, pmw_y_alpha = first_order_lowpass(full_rows[:, COL_GYRO_Y], PMW_Y_CUTOFF_HZ, 0.001)
    lc_x_lp, lc_x_alpha = first_order_lowpass(full_rows[:, COL_GYRO_X], LC_X_CUTOFF_HZ, 0.001)
    lc_y_lp, lc_y_alpha = first_order_lowpass(full_rows[:, COL_GYRO_Y], LC_Y_CUTOFF_HZ, 0.001)

    pmw_x_sum = np.concatenate(([0.0], np.cumsum(pmw_x_lp) * 0.001))
    pmw_y_sum = np.concatenate(([0.0], np.cumsum(pmw_y_lp) * 0.001))
    lc_x_sum = np.concatenate(([0.0], np.cumsum(lc_x_lp) * 0.001))
    lc_y_sum = np.concatenate(([0.0], np.cumsum(lc_y_lp) * 0.001))

    pmw_x_raw_samples: list[str] = []
    pmw_y_raw_samples: list[str] = []
    lc_x_raw_samples: list[str] = []
    lc_y_raw_samples: list[str] = []
    pmw_x_dec_samples: list[str] = []
    pmw_y_dec_samples: list[str] = []
    lc_x_dec_samples: list[str] = []
    lc_y_dec_samples: list[str] = []

    for sample_index, base_tick in enumerate(sample_base_ticks):
        observed_row = rows[sample_row_indices[sample_index]]
        end_index = base_tick - full_start_tick + 1
        start_index = end_index - FLOW_SAMPLE_PERIOD_MS
        if start_index < 0:
            pmw_x_raw_samples.append("")
            pmw_y_raw_samples.append("")
            lc_x_raw_samples.append("")
            lc_y_raw_samples.append("")
            pmw_x_dec_samples.append("")
            pmw_y_dec_samples.append("")
            lc_x_dec_samples.append("")
            lc_y_dec_samples.append("")
            continue

        pmw_x_integral = pmw_x_sum[end_index] - pmw_x_sum[start_index]
        pmw_y_integral = pmw_y_sum[end_index] - pmw_y_sum[start_index]
        lc_x_integral = lc_x_sum[end_index] - lc_x_sum[start_index]
        lc_y_integral = lc_y_sum[end_index] - lc_y_sum[start_index]

        pmw_x_raw_samples.append(format_float(observed_row[COL_PMW_X]))
        pmw_y_raw_samples.append(format_float(observed_row[COL_PMW_Y]))
        lc_x_raw_samples.append(format_float(observed_row[COL_LC_X]))
        lc_y_raw_samples.append(format_float(observed_row[COL_LC_Y]))

        if observed_row[COL_PMW_SQUAL] < PMW_SQUAL_MIN:
            pmw_x_dec_samples.append("")
            pmw_y_dec_samples.append("")
        else:
            pmw_x_dec_samples.append(format_float(observed_row[COL_PMW_X] - PMW_X_GAIN * pmw_x_integral))
            pmw_y_dec_samples.append(format_float(observed_row[COL_PMW_Y] - PMW_Y_GAIN * pmw_y_integral))

        lc_x_dec_samples.append(format_float(observed_row[COL_LC_X] - LC_X_GAIN * lc_x_integral))
        lc_y_dec_samples.append(format_float(observed_row[COL_LC_Y] - LC_Y_GAIN * lc_y_integral))

    print(f"PMW_X alpha={pmw_x_alpha:.6f}, PMW_Y alpha={pmw_y_alpha:.6f}")
    print(f"LC_X alpha={lc_x_alpha:.6f}, LC_Y alpha={lc_y_alpha:.6f}")
    print(f"sample_count={len(sample_pairs)}")

    return [
        [format_float(value) for value in full_rows[:, COL_GYRO_X]],
        [format_float(value) for value in full_rows[:, COL_GYRO_Y]],
        assign_held_values(full_ticks, sample_observed_ticks, pmw_x_raw_samples),
        assign_held_values(full_ticks, sample_observed_ticks, pmw_y_raw_samples),
        assign_held_values(full_ticks, sample_observed_ticks, lc_x_raw_samples),
        assign_held_values(full_ticks, sample_observed_ticks, lc_y_raw_samples),
        assign_held_values(full_ticks, sample_observed_ticks, pmw_x_dec_samples),
        assign_held_values(full_ticks, sample_observed_ticks, pmw_y_dec_samples),
        assign_held_values(full_ticks, sample_observed_ticks, lc_x_dec_samples),
        assign_held_values(full_ticks, sample_observed_ticks, lc_y_dec_samples),
    ]


def write_compare_csv(compare_columns: list[list[str]], output_path: Path) -> None:
    header = [f"I{index}" for index in range(10)]
    row_count = len(compare_columns[0])
    with output_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(header)
        for row_index in range(row_count):
            writer.writerow([column[row_index] for column in compare_columns])


def main() -> None:
    rows = load_rows(INPUT_CSV_PATH)
    full_ticks, full_rows = expand_to_full_tick(rows)
    compare_columns = make_compare_columns(rows, full_ticks, full_rows)
    write_compare_csv(compare_columns, OUTPUT_CSV_PATH)
    print(f"output={OUTPUT_CSV_PATH}")


if __name__ == "__main__":
    main()
