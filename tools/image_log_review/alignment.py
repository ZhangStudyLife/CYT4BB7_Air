"""核心0 CSV与BIMG来源帧的序号配准和诊断字段解码。"""

from __future__ import annotations

import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from .bimg import BimgIndexEntry


INVALID_VALUE = -999.0
CAMERA_NAMES = ("front", "center", "back")
CAMERA_LOG_COLUMNS = (("I1", "I2"), ("I3", "I4"), ("I5", "I6"))
CAMERA_SHAPE_COLUMNS = ("I10", "I11", "I12")
CAMERA_BEACON_START_COLUMNS = (13, 19, 25)


@dataclass(frozen=True)
class CoreLogRow:
    """保存核心0 CSV的一行数值及解绕后的摄像头帧号。"""

    row_index: int
    values: Mapping[str, float]
    frame_valid: tuple[bool, bool, bool]
    frame_low7: tuple[int, int, int]
    frame_unwrapped: tuple[int | None, int | None, int | None]

    def value(self, column: str, default: float = math.nan) -> float:
        """读取指定通道，缺失时返回默认值。"""

        return self.values.get(column, default)


@dataclass(frozen=True)
class AlignmentCandidate:
    """保存一个128帧周期偏移候选的评分。"""

    sequence_offset: int
    matched_frames: int
    match_rate: float
    pose_sample_count: int
    roll_median_error_deg: float
    pitch_median_error_deg: float
    height_median_error_mm: float
    pose_error: float
    clock_delta_median_ms: float
    clock_delta_mad_ms: float


@dataclass(frozen=True)
class FrameMatch:
    """保存一帧BIMG与核心0日志行的匹配关系。"""

    bimg_frame_index: int
    source_frame_sequence: int
    csv_row_index: int | None
    csv_time_ms: float | None
    sequence_offset: int


@dataclass(frozen=True)
class AlignmentResult:
    """保存自动配准结果、置信度和逐帧映射。"""

    camera_index: int
    sequence_offset: int
    confidence: str
    matches: tuple[FrameMatch, ...]
    candidates: tuple[AlignmentCandidate, ...]

    @property
    def matched_count(self) -> int:
        """返回存在CSV对应行的BIMG帧数。"""

        return sum(match.csv_row_index is not None for match in self.matches)


@dataclass(frozen=True)
class TrackGeometry:
    """保存I31中的公共轨迹位置和下摄ROI半尺寸。"""

    valid: bool
    center_x: float
    center_y: float
    center_roi_half_size: float


@dataclass(frozen=True)
class CrossCheckStatus:
    """保存I32中的公共轨迹、ROI和实测状态。"""

    state: int
    support_mask: int
    roi_valid_mask: int
    roi_hit_mask: int
    conflict_mask: int
    projection_enabled: bool
    fallback_mask: int
    button_marker: bool
    measured_mask: int
    actual_roi_mode: bool


@dataclass(frozen=True)
class LampShape:
    """保存I10至I12中的车灯形态字段。"""

    valid: bool
    width_px: float
    length_px: float
    angle_deg: float
    nearest_beacon_distance_px: float | None


def _number(value: object, default: float = math.nan) -> float:
    """把CSV字段转换为浮点数，失败时返回默认值。"""

    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _decode_frame_word(value: float) -> tuple[tuple[bool, bool, bool], tuple[int, int, int]]:
    """解码I33中的三摄7位帧号和有效位。"""

    packed = int(round(value)) if math.isfinite(value) else 0
    raw = tuple((packed >> (index * 8)) & 0xFF for index in range(3))
    valid = tuple(bool(item & 0x80) for item in raw)
    low7 = tuple(item & 0x7F for item in raw)
    return valid, low7


def _unwrap_camera_sequences(
    residues: Sequence[int], valid: Sequence[bool], times_ms: Sequence[float]
) -> list[int | None]:
    """结合100Hz日志时间把7位帧号解绕为单调递增的会话内帧号。"""

    result: list[int | None] = []
    previous_residue: int | None = None
    previous_time_ms: float | None = None
    unwrapped = 0
    for residue, is_valid, time_ms in zip(residues, valid, times_ms):
        if not is_valid:
            result.append(None)
            continue
        if previous_residue is None:
            unwrapped = residue
        else:
            residue_delta = (residue - previous_residue) & 0x7F
            delta = residue_delta
            if (
                previous_time_ms is not None
                and math.isfinite(previous_time_ms)
                and math.isfinite(time_ms)
                and time_ms >= previous_time_ms
            ):
                expected_delta = max(0, round((time_ms - previous_time_ms) / 20.0))
                wrap_count = round((expected_delta - residue_delta) / 128.0)
                delta = residue_delta + 128 * wrap_count
            if delta < 0 or (delta >= 64 and previous_time_ms is None):
                result.append(None)
                continue
            unwrapped += delta
        previous_residue = residue
        previous_time_ms = time_ms if math.isfinite(time_ms) else previous_time_ms
        result.append(unwrapped)
    return result


def read_core_log(path: str | Path) -> list[CoreLogRow]:
    """读取I0至I35核心0 CSV并解绕三摄帧号。"""

    raw_rows: list[dict[str, float]] = []
    with Path(path).open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or "I0" not in reader.fieldnames or "I33" not in reader.fieldnames:
            raise ValueError("CSV must contain I0 and I33 columns")
        for source in reader:
            raw_rows.append({key: _number(value) for key, value in source.items() if key})

    valid_by_camera: list[list[bool]] = [[], [], []]
    low_by_camera: list[list[int]] = [[], [], []]
    decoded: list[tuple[tuple[bool, bool, bool], tuple[int, int, int]]] = []
    for row in raw_rows:
        valid, low7 = _decode_frame_word(row.get("I33", 0.0))
        decoded.append((valid, low7))
        for camera in range(3):
            valid_by_camera[camera].append(valid[camera])
            low_by_camera[camera].append(low7[camera])

    times_ms = [row.get("I0", math.nan) for row in raw_rows]
    unwrapped = [
        _unwrap_camera_sequences(low_by_camera[camera], valid_by_camera[camera], times_ms)
        for camera in range(3)
    ]
    return [
        CoreLogRow(
            row_index=index,
            values=row,
            frame_valid=decoded[index][0],
            frame_low7=decoded[index][1],
            frame_unwrapped=tuple(unwrapped[camera][index] for camera in range(3)),
        )
        for index, row in enumerate(raw_rows)
    ]


def decode_track_geometry(value: float) -> TrackGeometry:
    """解码I31公共轨迹几何字段。"""

    packed = int(round(value)) if math.isfinite(value) else 0
    return TrackGeometry(
        valid=bool((packed >> 23) & 0x01),
        center_x=float((packed & 0x1FF) - 140),
        center_y=float(((packed >> 9) & 0xFF) - 110),
        center_roi_half_size=float((packed >> 17) & 0x3F),
    )


def decode_cross_check_status(value: float) -> CrossCheckStatus:
    """解码I32公共轨迹和ROI状态字段。"""

    packed = int(round(value)) if math.isfinite(value) else 0
    return CrossCheckStatus(
        state=packed & 0x07,
        support_mask=(packed >> 3) & 0x07,
        roi_valid_mask=(packed >> 6) & 0x07,
        roi_hit_mask=(packed >> 9) & 0x07,
        conflict_mask=(packed >> 12) & 0x07,
        projection_enabled=bool((packed >> 15) & 0x01),
        fallback_mask=(packed >> 16) & 0x07,
        button_marker=bool((packed >> 19) & 0x01),
        measured_mask=(packed >> 20) & 0x07,
        actual_roi_mode=bool((packed >> 23) & 0x01),
    )


def decode_lamp_shape(value: float) -> LampShape:
    """解码单摄车灯宽度、长度、角度和最近信标距离。"""

    packed = int(round(value)) if math.isfinite(value) else 0
    if packed == 0:
        return LampShape(False, 0.0, 0.0, 0.0, None)
    distance_code = (packed >> 20) & 0x0F
    return LampShape(
        valid=True,
        width_px=(packed & 0x3F) / 4.0,
        length_px=((packed >> 6) & 0x7F) / 2.0,
        angle_deg=((packed >> 13) & 0x7F) * 2.0 - 90.0,
        nearest_beacon_distance_px=None if distance_code == 15 else distance_code * 4.0,
    )


def camera_lamp(row: CoreLogRow, camera_index: int) -> tuple[float, float] | None:
    """读取指定摄像头最终车灯中心，无效时返回None。"""

    x_column, y_column = CAMERA_LOG_COLUMNS[camera_index]
    x = row.value(x_column)
    y = row.value(y_column)
    if not math.isfinite(x) or not math.isfinite(y) or x == INVALID_VALUE or y == INVALID_VALUE:
        return None
    return x, y


def camera_beacons(
    row: CoreLogRow, camera_index: int
) -> list[tuple[int, float, float, float]]:
    """读取指定摄像头前两个有效信标的槽号、中心和面积。"""

    result: list[tuple[int, float, float, float]] = []
    start = CAMERA_BEACON_START_COLUMNS[camera_index]
    for slot in range(2):
        column = start + slot * 3
        x = row.value(f"I{column}")
        y = row.value(f"I{column + 1}")
        area = row.value(f"I{column + 2}", 0.0)
        if (
            math.isfinite(x)
            and math.isfinite(y)
            and x != INVALID_VALUE
            and y != INVALID_VALUE
            and area > 0.0
        ):
            result.append((slot, x, y, area))
    return result


def _median(values: Sequence[float], default: float = math.inf) -> float:
    """返回有限数值中位数，没有样本时返回默认值。"""

    finite = [value for value in values if math.isfinite(value)]
    return statistics.median(finite) if finite else default


def _score_candidate(
    entries: Sequence[BimgIndexEntry],
    csv_by_sequence: Mapping[int, CoreLogRow],
    sequence_offset: int,
    camera_index: int,
) -> AlignmentCandidate:
    """计算一个帧号周期偏移的覆盖率、姿态误差和时钟稳定性。"""

    roll_errors: list[float] = []
    pitch_errors: list[float] = []
    height_errors: list[float] = []
    clock_deltas: list[float] = []
    matched = 0
    for entry in entries:
        row = csv_by_sequence.get(entry.frame_sequence - sequence_offset)
        if row is None:
            continue
        matched += 1
        if entry.timestamp_valid:
            csv_time = row.value("I0")
            if math.isfinite(csv_time):
                clock_deltas.append(csv_time - float(entry.capture_time_ms))
        roll = row.value("I7")
        pitch = row.value("I8")
        height = row.value("I9")
        if entry.roll_deg is not None and math.isfinite(entry.roll_deg) and math.isfinite(roll):
            roll_errors.append(abs(roll - entry.roll_deg))
        if entry.pitch_deg is not None and math.isfinite(entry.pitch_deg) and math.isfinite(pitch):
            pitch_errors.append(abs(pitch - entry.pitch_deg))
        if entry.height_mm is not None and math.isfinite(entry.height_mm) and math.isfinite(height):
            height_errors.append(abs(height - entry.height_mm))

    roll_error = _median(roll_errors)
    pitch_error = _median(pitch_errors)
    height_error = _median(height_errors)
    pose_parts = [
        value
        for value in (roll_error, pitch_error, height_error / 100.0)
        if math.isfinite(value)
    ]
    pose_error = sum(pose_parts) if pose_parts else math.inf
    clock_median = _median(clock_deltas)
    clock_mad = (
        _median([abs(value - clock_median) for value in clock_deltas])
        if math.isfinite(clock_median)
        else math.inf
    )
    return AlignmentCandidate(
        sequence_offset=sequence_offset,
        matched_frames=matched,
        match_rate=matched / len(entries) if entries else 0.0,
        pose_sample_count=max(len(roll_errors), len(pitch_errors), len(height_errors)),
        roll_median_error_deg=roll_error,
        pitch_median_error_deg=pitch_error,
        height_median_error_mm=height_error,
        pose_error=pose_error,
        clock_delta_median_ms=clock_median,
        clock_delta_mad_ms=clock_mad,
    )


def align_bimg_to_core_log(
    entries: Sequence[BimgIndexEntry],
    rows: Sequence[CoreLogRow],
    camera_index: int = 0,
    manual_sequence_offset: int | None = None,
) -> AlignmentResult:
    """按来源帧号、姿态签名和时钟偏移把BIMG帧对应到核心0日志。"""

    if camera_index not in range(3):
        raise ValueError("camera_index must be 0, 1 or 2")
    usable_entries = [
        entry
        for entry in entries
        if (
            entry.frame_valid
            and entry.crc_ok
            and entry.stream_mode == 0
            and entry.source_camera == camera_index
        )
    ]
    if not usable_entries:
        raise ValueError("BIMG contains no valid RAW frame for the selected camera")
    if any(
        current.frame_sequence < previous.frame_sequence
        for previous, current in zip(usable_entries, usable_entries[1:])
    ):
        raise ValueError("BIMG source frame sequence was reset; split the recording at the reset")

    csv_by_sequence: dict[int, CoreLogRow] = {}
    for row in rows:
        sequence = row.frame_unwrapped[camera_index]
        if sequence is not None and sequence not in csv_by_sequence:
            csv_by_sequence[sequence] = row
    if not csv_by_sequence:
        raise ValueError("CSV contains no valid frame sequence for the selected camera")

    if manual_sequence_offset is not None:
        if manual_sequence_offset % 128:
            raise ValueError("manual sequence offset must be a multiple of 128")
        offsets = [manual_sequence_offset]
    else:
        minimum_csv = min(csv_by_sequence)
        maximum_csv = max(csv_by_sequence)
        minimum_bimg = min(entry.frame_sequence for entry in usable_entries)
        maximum_bimg = max(entry.frame_sequence for entry in usable_entries)
        minimum_k = math.floor((minimum_bimg - maximum_csv) / 128) - 1
        maximum_k = math.ceil((maximum_bimg - minimum_csv) / 128) + 1
        offsets = [128 * k for k in range(minimum_k, maximum_k + 1)]

    candidates = [
        _score_candidate(usable_entries, csv_by_sequence, offset, camera_index)
        for offset in offsets
    ]
    maximum_match = max(candidate.matched_frames for candidate in candidates)
    if maximum_match == 0:
        raise ValueError("BIMG source frames do not overlap the selected camera in the CSV")
    match_tolerance = max(2, math.ceil(len(usable_entries) * 0.01))
    finalists = [
        candidate
        for candidate in candidates
        if candidate.matched_frames >= maximum_match - match_tolerance
    ]
    finalists.sort(
        key=lambda candidate: (
            candidate.pose_error,
            candidate.clock_delta_mad_ms,
            -candidate.matched_frames,
        )
    )
    best = finalists[0]
    sorted_candidates = tuple(
        sorted(
            candidates,
            key=lambda candidate: (
                -candidate.matched_frames,
                candidate.pose_error,
                candidate.clock_delta_mad_ms,
            ),
        )
    )

    confidence = "high"
    if best.match_rate < 0.90 or best.pose_sample_count == 0 or not math.isfinite(best.pose_error):
        confidence = "low"
    elif len(finalists) > 1:
        runner_up = finalists[1]
        if abs(runner_up.pose_error - best.pose_error) < 0.5:
            confidence = "medium"
        if abs(runner_up.pose_error - best.pose_error) < 0.1:
            confidence = "low"

    row_lookup = {
        sequence + best.sequence_offset: row for sequence, row in csv_by_sequence.items()
    }
    matches: list[FrameMatch] = []
    for entry in entries:
        eligible = (
            entry.frame_valid
            and entry.crc_ok
            and entry.stream_mode == 0
            and entry.source_camera == camera_index
        )
        row = row_lookup.get(entry.frame_sequence) if eligible else None
        matches.append(
            FrameMatch(
                bimg_frame_index=entry.frame_index,
                source_frame_sequence=entry.frame_sequence,
                csv_row_index=row.row_index if row is not None else None,
                csv_time_ms=row.value("I0") if row is not None else None,
                sequence_offset=best.sequence_offset,
            )
        )
    return AlignmentResult(
        camera_index=camera_index,
        sequence_offset=best.sequence_offset,
        confidence=confidence,
        matches=tuple(matches),
        candidates=sorted_candidates,
    )


def export_alignment_csv(
    path: str | Path,
    entries: Sequence[BimgIndexEntry],
    rows: Sequence[CoreLogRow],
    alignment: AlignmentResult,
) -> None:
    """导出包含BIMG索引和完整核心0通道的逐帧配准表。"""

    rows_by_index = {row.row_index: row for row in rows}
    matches_by_frame = {match.bimg_frame_index: match for match in alignment.matches}
    log_columns = sorted(
        {key for row in rows for key in row.values},
        key=lambda name: int(name[1:]) if name.startswith("I") and name[1:].isdigit() else 999,
    )
    fieldnames = [
        "bimg_frame_index",
        "file_offset",
        "stream_sequence",
        "source_frame_sequence",
        "capture_time_ms",
        "crc_ok",
        "csv_row_index",
        "sequence_offset",
    ] + log_columns
    with Path(path).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for entry in entries:
            match = matches_by_frame.get(entry.frame_index)
            row = rows_by_index.get(match.csv_row_index) if match and match.csv_row_index is not None else None
            record: dict[str, object] = {
                "bimg_frame_index": entry.frame_index,
                "file_offset": entry.file_offset,
                "stream_sequence": entry.stream_sequence,
                "source_frame_sequence": entry.frame_sequence,
                "capture_time_ms": entry.capture_time_ms,
                "crc_ok": int(entry.crc_ok),
                "csv_row_index": "" if match is None or match.csv_row_index is None else match.csv_row_index,
                "sequence_offset": alignment.sequence_offset,
            }
            if row is not None:
                record.update(row.values)
            writer.writerow(record)
