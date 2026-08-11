"""核心0 CSV与BIMG帧号配准测试。"""

from __future__ import annotations

import csv
import math
import tempfile
import unittest
from pathlib import Path

from image_log_review.alignment import (
    align_bimg_to_core_log,
    export_alignment_csv,
    read_core_log,
)
from image_log_review.bimg import BimgIndexEntry


def make_entry(index: int, sequence: int, pose_index: int) -> BimgIndexEntry:
    """生成包含姿态签名的前摄BIMG索引。"""

    return BimgIndexEntry(
        frame_index=index,
        file_offset=index * 22628,
        packet_size=22628,
        stream_sequence=index,
        frame_sequence=sequence,
        capture_time_ms=5000 + pose_index * 20,
        source_camera=0,
        board_id=0,
        width=188,
        height=120,
        stream_mode=0,
        frame_valid=True,
        timestamp_valid=True,
        crc_ok=True,
        roll_deg=pose_index * 0.25,
        pitch_deg=-pose_index * 0.1,
        height_mm=900.0 + pose_index,
    )


class AlignmentTest(unittest.TestCase):
    """验证7位帧号回绕、周期选择和逐帧导出主键。"""

    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.csv_path = Path(self.temp_dir.name) / "core.csv"
        fieldnames = [f"I{index}" for index in range(36)]
        with self.csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for pose_index in range(260):
                low7 = (100 + pose_index) & 0x7F
                packed_i33 = 0x80 | low7
                for duplicate in range(2):
                    row = {name: 0.0 for name in fieldnames}
                    row.update(
                        {
                            "I0": 5005 + pose_index * 20 + duplicate * 10,
                            "I7": pose_index * 0.25,
                            "I8": -pose_index * 0.1,
                            "I9": 900.0 + pose_index,
                            "I33": packed_i33,
                        }
                    )
                    writer.writerow(row)

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def test_automatic_alignment_uses_pose_to_choose_wrap(self) -> None:
        rows = read_core_log(self.csv_path)
        expected_offset = 896
        entries = [
            make_entry(index, expected_offset + 100 + pose_index, pose_index)
            for index, pose_index in enumerate(range(30, 220, 3))
        ]
        result = align_bimg_to_core_log(entries, rows)
        self.assertEqual(result.sequence_offset, expected_offset)
        self.assertEqual(result.matched_count, len(entries))
        self.assertEqual(result.confidence, "high")
        self.assertEqual(result.matches[0].csv_row_index, 30 * 2)

    def test_manual_offset_must_follow_seven_bit_period(self) -> None:
        rows = read_core_log(self.csv_path)
        entries = [make_entry(0, 996, 0)]
        with self.assertRaises(ValueError):
            align_bimg_to_core_log(entries, rows, manual_sequence_offset=1)

    def test_zero_overlap_is_rejected(self) -> None:
        short_path = Path(self.temp_dir.name) / "short.csv"
        fieldnames = [f"I{index}" for index in range(36)]
        with short_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            row = {name: 0.0 for name in fieldnames}
            row["I33"] = 0x80
            writer.writerow(row)
        rows = read_core_log(short_path)
        entries = [make_entry(0, 1, 0)]
        with self.assertRaisesRegex(ValueError, "do not overlap"):
            align_bimg_to_core_log(entries, rows)

    def test_invalid_and_non_raw_entries_are_not_mapped(self) -> None:
        rows = read_core_log(self.csv_path)
        valid = make_entry(0, 996, 0)
        invalid = BimgIndexEntry(**{**valid.__dict__, "frame_index": 1, "crc_ok": False})
        non_raw = BimgIndexEntry(**{**valid.__dict__, "frame_index": 2, "stream_mode": 3})
        result = align_bimg_to_core_log(
            [valid, invalid, non_raw], rows, manual_sequence_offset=896
        )
        self.assertIsNotNone(result.matches[0].csv_row_index)
        self.assertIsNone(result.matches[1].csv_row_index)
        self.assertIsNone(result.matches[2].csv_row_index)

    def test_long_gap_uses_log_time_to_unwrap_sequence(self) -> None:
        gap_path = Path(self.temp_dir.name) / "gap.csv"
        fieldnames = [f"I{index}" for index in range(36)]
        samples = ((0.0, 0), (1400.0, 70), (2560.0, 0), (2580.0, 1))
        with gap_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for time_ms, residue in samples:
                row = {name: 0.0 for name in fieldnames}
                row["I0"] = time_ms
                row["I33"] = 0x80 | residue
                writer.writerow(row)
        rows = read_core_log(gap_path)
        self.assertEqual([row.frame_unwrapped[0] for row in rows], [0, 70, 128, 129])

    def test_nan_pose_cannot_produce_high_confidence(self) -> None:
        rows = read_core_log(self.csv_path)
        entries = [
            BimgIndexEntry(
                **{
                    **make_entry(index, 996 + index, index).__dict__,
                    "roll_deg": math.nan,
                    "pitch_deg": math.nan,
                    "height_mm": math.nan,
                }
            )
            for index in range(10)
        ]
        result = align_bimg_to_core_log(entries, rows, manual_sequence_offset=896)
        self.assertNotEqual(result.confidence, "high")

    def test_static_pose_and_independent_clock_origin_stays_low_confidence(self) -> None:
        static_path = Path(self.temp_dir.name) / "static.csv"
        fieldnames = [f"I{index}" for index in range(36)]
        with static_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for index in range(260):
                row = {name: 0.0 for name in fieldnames}
                row.update(
                    {
                        "I0": 500000.0 + index * 20.0,
                        "I7": 0.0,
                        "I8": 0.0,
                        "I9": 1000.0,
                        "I33": 0x80 | ((100 + index) & 0x7F),
                    }
                )
                writer.writerow(row)
        rows = read_core_log(static_path)
        entries = []
        for frame_index, pose_index in enumerate(range(128, 201, 3)):
            entry = make_entry(frame_index, 996 + pose_index, pose_index)
            entries.append(
                BimgIndexEntry(
                    **{
                        **entry.__dict__,
                        "roll_deg": 0.0,
                        "pitch_deg": 0.0,
                        "height_mm": 1000.0,
                    }
                )
            )
        result = align_bimg_to_core_log(entries, rows)
        self.assertEqual(result.confidence, "low")

    def test_export_contains_matched_and_unmatched_rows(self) -> None:
        rows = read_core_log(self.csv_path)
        valid = make_entry(0, 996, 0)
        invalid = BimgIndexEntry(**{**valid.__dict__, "frame_index": 1, "crc_ok": False})
        result = align_bimg_to_core_log(
            [valid, invalid], rows, manual_sequence_offset=896
        )
        output = Path(self.temp_dir.name) / "aligned.csv"
        export_alignment_csv(output, [valid, invalid], rows, result)
        with output.open("r", encoding="utf-8-sig", newline="") as stream:
            exported = list(csv.DictReader(stream))
        self.assertEqual(len(exported), 2)
        self.assertNotEqual(exported[0]["csv_row_index"], "")
        self.assertEqual(exported[1]["csv_row_index"], "")


if __name__ == "__main__":
    unittest.main()
