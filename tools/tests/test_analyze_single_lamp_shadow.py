import csv
import sys
import tempfile
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

from analyze_single_lamp_shadow import analyze_csv  # noqa: E402


class AnalyzeSingleLampShadowTest(unittest.TestCase):
    def _write(self, column_count, rows):
        handle = tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", newline="", suffix=".csv", delete=False
        )
        path = Path(handle.name)
        with handle:
            names = [f"I{index}" for index in range(column_count)]
            writer = csv.DictWriter(handle, fieldnames=names)
            writer.writeheader()
            for values in rows:
                row = {name: 0.0 for name in names}
                row.update(values)
                writer.writerow(row)
        self.addCleanup(path.unlink, missing_ok=True)
        return path

    @staticmethod
    def _status(time_ms, skew):
        return (
            2
            | (0x07 << 3)
            | (0x07 << 6)
            | (1 << 12)
            | ((1 if skew <= 10 else 0) << 13)
            | (1 << 14)
            | ((time_ms & 0x1FF) << 15)
        )

    @staticmethod
    def _sequences(front, center, back):
        return (
            ((front & 0x7F) | 0x80)
            | (((center & 0x7F) | 0x80) << 8)
            | (((back & 0x7F) | 0x80) << 16)
        )

    def test_compact_frame_and_sync_stats(self):
        rows = []
        for index, skew in enumerate((9, 10, 11, 20)):
            time_ms = 500 + index * 20
            rows.append(
                {
                    "I0": 1.0,
                    "I1": 2.0,
                    "I2": 3.0,
                    "I3": 4.0,
                    "I4": 5.0,
                    "I5": 6.0,
                    "I9": skew,
                    "I10": self._status(time_ms, skew),
                    "I11": self._sequences(
                        126 + index, 10 + index, 30 + index
                    ),
                }
            )
        result = analyze_csv(self._write(12, rows))
        self.assertEqual(result["format"], "single_lamp_shadow_v12")
        self.assertAlmostEqual(result["frames"]["front"]["frame_rate_hz"], 50.0)
        self.assertEqual(result["frames"]["front"]["dropped_frames"], 0)
        self.assertEqual(result["frames"]["front"]["out_of_order_frames"], 0)
        self.assertAlmostEqual(
            result["sync"]["threshold_coverage"]["10_ms"], 0.5
        )
        self.assertAlmostEqual(
            result["sync"]["threshold_coverage"]["15_ms"], 0.75
        )
        self.assertAlmostEqual(
            result["sync"]["threshold_coverage"]["20_ms"], 1.0
        )
        self.assertEqual(result["track"]["state_rows"], {2: 4})

    def test_legacy_is_accepted_without_inventing_metadata(self):
        result = analyze_csv(self._write(6, [{"I0": 0.0}, {"I0": 1.0}]))
        self.assertEqual(result["format"], "legacy_without_frame_metadata")
        self.assertIsNone(result["frames"])
        self.assertIsNone(result["sync"])


if __name__ == "__main__":
    unittest.main()
