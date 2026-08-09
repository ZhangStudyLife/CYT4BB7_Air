#!/usr/bin/env python3
"""Analyze legacy and compact single-lamp three-camera CSV logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import Counter
from pathlib import Path


CAMERAS = ("front", "center", "back")
INVALID_VALUE = -999.0
FRAME_GAP_LIMIT_MS = 30.0


def _number(row: dict[str, str], column: str, default: float = math.nan) -> float:
    try:
        return float(row[column])
    except (KeyError, TypeError, ValueError):
        return default


def _longest_false_run(values: list[bool]) -> int:
    longest = 0
    current = 0
    for value in values:
        current = 0 if value else current + 1
        longest = max(longest, current)
    return longest


def _unwrap_residues(
    residues: list[int], valid: list[bool], modulus: int
) -> list[float]:
    result: list[float] = []
    accumulated = 0
    previous: int | None = None
    for residue, is_valid in zip(residues, valid):
        if not is_valid:
            result.append(math.nan)
            continue
        if previous is None:
            accumulated = residue
        else:
            delta = (residue - previous) % modulus
            if delta >= modulus // 2:
                result.append(math.nan)
                continue
            accumulated += delta
        previous = residue
        result.append(float(accumulated))
    return result


def _frame_stats(
    sequences: list[int],
    valid: list[bool],
    times_ms: list[float],
    modulus: int,
) -> dict[str, float | int]:
    previous_sequence: int | None = None
    unique_times: list[float] = []
    repeated_rows = 0
    longest_repeated_rows = 0
    repeated_run = 0
    dropped_frames = 0
    out_of_order_frames = 0

    for sequence, is_valid, timestamp in zip(sequences, valid, times_ms):
        if not is_valid or not math.isfinite(timestamp):
            continue
        if previous_sequence is None:
            previous_sequence = sequence
            unique_times.append(timestamp)
            continue
        if sequence == previous_sequence:
            repeated_rows += 1
            repeated_run += 1
            longest_repeated_rows = max(longest_repeated_rows, repeated_run)
            continue

        delta = (sequence - previous_sequence) % modulus
        if 0 < delta < modulus // 2:
            dropped_frames += max(0, delta - 1)
        else:
            out_of_order_frames += 1
        previous_sequence = sequence
        unique_times.append(timestamp)
        repeated_run = 0

    intervals = [
        right - left
        for left, right in zip(unique_times, unique_times[1:])
        if right > left
    ]
    duration_ms = (
        unique_times[-1] - unique_times[0] if len(unique_times) >= 2 else 0.0
    )
    return {
        "valid_rows": sum(valid),
        "unique_frames": len(unique_times),
        "repeated_rows": repeated_rows,
        "longest_repeated_rows": longest_repeated_rows,
        "dropped_frames": dropped_frames,
        "out_of_order_frames": out_of_order_frames,
        "frame_gap_count": sum(value > FRAME_GAP_LIMIT_MS for value in intervals),
        "max_frame_interval_ms": max(intervals, default=0.0),
        "frame_rate_hz": (
            (len(unique_times) - 1) * 1000.0 / duration_ms
            if duration_ms > 0.0
            else 0.0
        ),
        "longest_invalid_rows": _longest_false_run(valid),
    }


def _compact_v12(rows: list[dict[str, str]]) -> dict[str, object]:
    statuses = [int(round(_number(row, "I10", 0.0))) for row in rows]
    sequence_words = [int(round(_number(row, "I11", 0.0))) for row in rows]
    center_time_valid = [((word >> (1 * 8 + 7)) & 1) != 0 for word in sequence_words]
    center_residues = [(status >> 15) & 0x1FF for status in statuses]
    times_ms = _unwrap_residues(center_residues, center_time_valid, 1 << 9)

    frames: dict[str, object] = {}
    for camera_index, camera in enumerate(CAMERAS):
        values = [(word >> (camera_index * 8)) & 0xFF for word in sequence_words]
        valid = [((value >> 7) & 1) != 0 for value in values]
        sequences = [value & 0x7F for value in values]
        frames[camera] = _frame_stats(
            sequences, valid, times_ms, modulus=1 << 7
        )

    time_ready = [((status >> 14) & 1) != 0 for status in statuses]
    sync_10_valid = [((status >> 13) & 1) != 0 for status in statuses]
    skews = [_number(row, "I9", math.nan) for row in rows]
    ready_skews = [
        skew
        for skew, ready in zip(skews, time_ready)
        if ready and math.isfinite(skew)
    ]
    threshold_coverage = {
        f"{threshold}_ms": (
            sum(skew <= threshold for skew in ready_skews) / len(ready_skews)
            if ready_skews
            else 0.0
        )
        for threshold in (10, 15, 20)
    }

    detection: dict[str, float] = {}
    for camera_index, camera in enumerate(CAMERAS):
        x_column = f"I{camera_index * 2}"
        y_column = f"I{camera_index * 2 + 1}"
        detected = [
            _number(row, x_column) != INVALID_VALUE
            and _number(row, y_column) != INVALID_VALUE
            for row in rows
        ]
        detection[camera] = sum(detected) / len(detected) if detected else 0.0

    states = [status & 0x07 for status in statuses]
    support = [(status >> 3) & 0x07 for status in statuses]
    roi_hits = [(status >> 6) & 0x07 for status in statuses]
    conflicts = [(status >> 9) & 0x07 for status in statuses]
    source_switches = sum(
        left != right and left != 0 and right != 0
        for left, right in zip(support, support[1:])
    )
    return {
        "format": "single_lamp_shadow_v12",
        "rows": len(rows),
        "frames": frames,
        "sync": {
            "time_ready_rate": sum(time_ready) / len(rows) if rows else 0.0,
            "reported_10ms_valid_rate": (
                sum(sync_10_valid) / len(rows) if rows else 0.0
            ),
            "threshold_coverage": threshold_coverage,
            "max_skew_ms": max(ready_skews, default=0.0),
        },
        "track": {
            "state_rows": dict(Counter(states)),
            "source_switches": source_switches,
            "roi_hit_rate": {
                camera: (
                    sum((mask & (1 << index)) != 0 for mask in roi_hits)
                    / len(rows)
                    if rows
                    else 0.0
                )
                for index, camera in enumerate(CAMERAS)
            },
            "conflict_rate": {
                camera: (
                    sum((mask & (1 << index)) != 0 for mask in conflicts)
                    / len(rows)
                    if rows
                    else 0.0
                )
                for index, camera in enumerate(CAMERAS)
            },
        },
        "raw_lamp_detection_rate": detection,
    }


def _sync_v57(rows: list[dict[str, str]]) -> dict[str, object]:
    frames: dict[str, object] = {}
    for camera_index, camera in enumerate(CAMERAS):
        sequence_column = f"I{50 + camera_index * 2}"
        time_column = f"I{51 + camera_index * 2}"
        sequences = [int(round(_number(row, sequence_column, 0.0))) for row in rows]
        valid = [sequence > 0 for sequence in sequences]
        times = [_number(row, time_column) for row in rows]
        frames[camera] = _frame_stats(sequences, valid, times, 1 << 32)

    skews = [_number(row, "I56") for row in rows]
    available = [value for value in skews if math.isfinite(value)]
    return {
        "format": "sync_v57",
        "rows": len(rows),
        "frames": frames,
        "sync": {
            "reported_10ms_valid_rate": (
                sum(_number(row, "I57", 0.0) >= 0.5 for row in rows) / len(rows)
                if rows
                else 0.0
            ),
            "threshold_coverage": {
                f"{threshold}_ms": (
                    sum(value <= threshold for value in available) / len(available)
                    if available
                    else 0.0
                )
                for threshold in (10, 15, 20)
            },
            "max_skew_ms": max(available, default=0.0),
        },
    }


def analyze_csv(path: Path) -> dict[str, object]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        columns = set(reader.fieldnames or [])

    if not rows or "I0" not in columns:
        raise ValueError(f"{path}: missing I0 header or data rows")
    if all(f"I{index}" in columns for index in range(12)) and "I12" not in columns:
        result = _compact_v12(rows)
    elif "I57" in columns:
        result = _sync_v57(rows)
    else:
        result = {
            "format": "legacy_without_frame_metadata",
            "rows": len(rows),
            "frames": None,
            "sync": None,
        }
    result["file"] = str(path)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze_csv(args.csv), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
