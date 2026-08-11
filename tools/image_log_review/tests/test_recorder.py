"""BIMG本地TCP录制和文件一致性测试。"""

from __future__ import annotations

import json
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path

from image_log_review.recorder import BimgRecorder
from image_log_review.tests.test_bimg import make_packet


def reserve_local_port() -> int:
    """向操作系统申请一个当前可用的本地TCP端口。"""

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class RecorderTest(unittest.TestCase):
    """验证录制器保存原始字节、索引偏移和首帧FPS。"""

    def test_local_socket_recording_preserves_stream(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "capture.bimg"
            port = reserve_local_port()
            listening = threading.Event()
            recorder = BimgRecorder(
                output,
                host="127.0.0.1",
                port=port,
                event_callback=lambda event, _payload: listening.set()
                if event == "listening"
                else None,
            )
            worker = threading.Thread(target=recorder.run)
            worker.start()
            self.assertTrue(listening.wait(2.0))
            time.sleep(0.2)

            first = make_packet(stream_sequence=0, frame_sequence=100)
            second = make_packet(stream_sequence=1, frame_sequence=101)
            with socket.create_connection(("127.0.0.1", port), timeout=2.0) as client:
                client.sendall(first)
                time.sleep(0.02)
                client.sendall(second)
            time.sleep(0.05)
            recorder.stop()
            worker.join(2.0)

            self.assertFalse(worker.is_alive())
            self.assertEqual(output.read_bytes(), first + second)
            self.assertEqual(recorder.stats.decoded_frames, 2)
            self.assertGreater(recorder.stats.frame_rate_hz, 20.0)
            records = [
                json.loads(line)
                for line in output.with_suffix(".bimg.jsonl").read_text(encoding="utf-8").splitlines()
            ]
            frames = [record for record in records if record["type"] == "frame"]
            self.assertEqual([record["file_offset"] for record in frames], [0, len(first)])

    def test_bind_failure_does_not_create_output_files(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "capture.bimg"
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as occupied:
                occupied.bind(("127.0.0.1", 0))
                occupied.listen(1)
                recorder = BimgRecorder(
                    output, host="127.0.0.1", port=occupied.getsockname()[1]
                )
                with self.assertRaises(OSError):
                    recorder.run()
            self.assertFalse(output.exists())
            self.assertFalse(output.with_suffix(".bimg.jsonl").exists())


if __name__ == "__main__":
    unittest.main()
