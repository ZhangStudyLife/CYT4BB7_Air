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
    def _status(time_ms, latest_group_valid=True):
        return (
            2
            | (0x07 << 3)
            | (0x07 << 6)
            | (1 << 12)
            | ((1 if latest_group_valid else 0) << 13)
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

    @staticmethod
    def _shape(width, length, angle, beacon_distance_code):
        return (
            (round(width * 4) & 0x3F)
            | ((round(length * 2) & 0x7F) << 6)
            | ((round((angle + 90) / 2) & 0x7F) << 13)
            | ((beacon_distance_code & 0x0F) << 20)
        )

    def test_v35_decodes_lamps_beacons_track_and_frames(self):
        status = (
            2
            | (0x03 << 3)
            | (0x07 << 6)
            | (0x02 << 9)
            | (0x04 << 12)
            | (1 << 15)
            | (0x01 << 16)
            | (1 << 19)
            | (0x03 << 20)
            | (1 << 23)
        )
        geometry = (
            ((12 + 140) & 0x1FF)
            | (((-8 + 110) & 0xFF) << 9)
            | (20 << 17)
            | (1 << 23)
        )
        rows = []
        for index, skew in enumerate((9, 10, 11, 20)):
            rows.append(
                {
                    "I0": 1000 + index * 20,
                    "I1": 10.0,
                    "I2": 20.0,
                    "I3": 30.0,
                    "I4": 40.0,
                    "I5": -999.0,
                    "I6": -999.0,
                    "I10": self._shape(4.0, 12.0, 30.0, 3),
                    "I11": self._shape(5.0, 14.0, -20.0, 15),
                    "I12": 0.0,
                    "I13": 10.0,
                    "I14": 20.0,
                    "I15": 64.0,
                    "I16": 100.0,
                    "I17": 80.0,
                    "I18": 128.0,
                    "I19": 11.0,
                    "I20": 21.0,
                    "I21": 65.0,
                    "I22": 101.0,
                    "I23": 81.0,
                    "I24": 129.0,
                    "I25": 12.0,
                    "I26": 22.0,
                    "I27": 66.0,
                    "I28": -999.0,
                    "I29": -999.0,
                    "I30": 0.0,
                    "I31": geometry,
                    "I32": status,
                    "I33": self._sequences(126 + index, 10 + index, 30 + index),
                    "I34": 15.0,
                    "I35": skew,
                }
            )

        result = analyze_csv(self._write(36, rows))
        self.assertEqual(result["format"], "single_lamp_shadow_v35")
        self.assertAlmostEqual(result["frames"]["front"]["frame_rate_hz"], 50.0)
        self.assertAlmostEqual(result["sync"]["threshold_coverage"]["10_ms"], 0.5)
        self.assertEqual(result["track"]["state_rows"], {2: 4})
        self.assertEqual(result["track"]["geometry_valid_rate"], 1.0)
        self.assertEqual(result["track"]["actual_roi_mode_rate"], 1.0)
        self.assertEqual(result["button_marker"]["pressed_rows"], 4)
        self.assertEqual(result["raw_lamp_detection_rate"]["back"], 0.0)
        self.assertEqual(result["lamp_shapes"]["front"]["mean_angle_deg"], 30.0)
        self.assertEqual(result["beacons"]["front"][0]["valid_rate"], 1.0)
        self.assertEqual(result["beacons"]["front"][0]["mean_area"], 64.0)
        self.assertEqual(result["beacons"]["back"][1]["valid_rate"], 0.0)
        self.assertEqual(result["relative_yaw"]["mean_deg"], 15.0)

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
                    "I10": self._status(time_ms),
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

    def test_roi_debug_status_and_button_marker(self):
        status = (
            2
            | (0x03 << 3)
            | (0x07 << 6)
            | (0x02 << 9)
            | (0x04 << 12)
            | (1 << 15)
            | (1 << 16)
            | (1 << 17)
            | (0x02 << 18)
            | (0x07 << 21)
        )
        rows = [
            {
                "I0": 1.0,
                "I1": 2.0,
                "I2": 3.0,
                "I3": 4.0,
                "I4": -999.0,
                "I5": -999.0,
                "I9": status,
                "I10": 20.0,
                "I11": 0.0,
            },
            {
                "I0": 1.0,
                "I1": 2.0,
                "I2": 3.0,
                "I3": 4.0,
                "I4": -999.0,
                "I5": -999.0,
                "I9": status,
                "I10": 20.0,
                "I11": 1.0,
            },
        ]
        result = analyze_csv(self._write(12, rows))
        self.assertEqual(result["format"], "single_lamp_roi_debug_v12")
        self.assertEqual(result["track"]["state_rows"], {2: 2})
        self.assertEqual(result["track"]["projection_enabled_rate"], 1.0)
        self.assertEqual(result["track"]["roi_valid_rate"]["center"], 1.0)
        self.assertEqual(result["track"]["measured_rate"]["center"], 1.0)
        self.assertEqual(result["button_marker"]["pressed_rows"], 1)
        self.assertEqual(result["lamp_output_valid_rate"]["back"], 0.0)

    def test_legacy_is_accepted_without_inventing_metadata(self):
        result = analyze_csv(self._write(6, [{"I0": 0.0}, {"I0": 1.0}]))
        self.assertEqual(result["format"], "legacy_without_frame_metadata")
        self.assertIsNone(result["frames"])
        self.assertIsNone(result["sync"])


if __name__ == "__main__":
    unittest.main()
