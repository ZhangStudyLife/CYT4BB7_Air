"""BIMG TCP原始流录制器。"""

from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable

from .bimg import BimgFrame, BimgStreamDecoder


@dataclass
class RecorderStats:
    """保存本次TCP录制的运行统计。"""

    connection_count: int = 0
    received_bytes: int = 0
    decoded_frames: int = 0
    connection_frames: int = 0
    crc_error_count: int = 0
    format_error_count: int = 0
    trailing_bytes: int = 0
    last_stream_sequence: int = 0
    last_frame_sequence: int = 0
    started_ns: int = 0
    first_frame_ns: int = 0
    last_frame_ns: int = 0

    @property
    def frame_rate_hz(self) -> float:
        """返回从首包到当前的平均BIMG接收帧率。"""

        if self.connection_frames < 2 or self.last_frame_ns <= self.first_frame_ns:
            return 0.0
        return (self.connection_frames - 1) * 1_000_000_000.0 / (
            self.last_frame_ns - self.first_frame_ns
        )


class BimgRecorder:
    """监听一个TCP端口并原样保存设备发送的BIMG字节流。"""

    def __init__(
        self,
        output_path: str | Path,
        host: str = "0.0.0.0",
        port: int = 8086,
        event_callback: Callable[[str, object], None] | None = None,
    ) -> None:
        self.output_path = Path(output_path)
        self.index_path = self.output_path.with_suffix(self.output_path.suffix + ".jsonl")
        self.host = host
        self.port = port
        self.event_callback = event_callback
        self.stats = RecorderStats()
        self._stop_event = threading.Event()
        self._server: socket.socket | None = None
        self._client: socket.socket | None = None

    def stop(self) -> None:
        """请求结束监听，已收到的原始字节不会被删除。"""

        self._stop_event.set()
        for sock in (self._client, self._server):
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    sock.close()
                except OSError:
                    pass

    def run(self) -> RecorderStats:
        """阻塞运行录制循环，直到stop被调用或发生不可恢复错误。"""

        if not 1 <= self.port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.stats.started_ns = time.time_ns()

        if self.output_path.exists() or self.index_path.exists():
            raise FileExistsError("BIMG output or JSONL index already exists")

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._server.bind((self.host, self.port))
            self._server.listen(1)
            self._server.settimeout(0.5)
        except Exception:
            self._server.close()
            self._server = None
            raise

        with self.output_path.open("xb") as raw_stream, self.index_path.open(
            "x", encoding="utf-8", newline="\n"
        ) as index_stream:
            self._emit("listening", {"host": self.host, "port": self.port})

            while not self._stop_event.is_set():
                try:
                    client, address = self._server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._stop_event.is_set():
                        break
                    raise
                self.stats.connection_count += 1
                self.stats.connection_frames = 0
                self.stats.first_frame_ns = 0
                self.stats.last_frame_ns = 0
                self._client = client
                self._emit("connected", {"address": address, "connection": self.stats.connection_count})
                self._record_connection(raw_stream, index_stream, client)
                self._client = None

        self._emit("stopped", asdict(self.stats))
        return self.stats

    def _record_connection(self, raw_stream, index_stream, client: socket.socket) -> None:
        """录制一个TCP连接并建立帧级JSONL索引。"""

        decoder = BimgStreamDecoder()
        connection_start = raw_stream.tell()
        client.settimeout(0.5)
        try:
            while not self._stop_event.is_set():
                try:
                    chunk = client.recv(64 * 1024)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break

                raw_stream.write(chunk)
                raw_stream.flush()
                self.stats.received_bytes += len(chunk)
                received_ns = time.time_ns()
                received_monotonic_ns = time.monotonic_ns()
                for frame in decoder.feed(chunk):
                    self._record_frame(
                        index_stream,
                        frame,
                        connection_start,
                        received_ns,
                        received_monotonic_ns,
                    )
        finally:
            received_ns = time.time_ns()
            received_monotonic_ns = time.monotonic_ns()
            for frame in decoder.flush():
                self._record_frame(
                    index_stream,
                    frame,
                    connection_start,
                    received_ns,
                    received_monotonic_ns,
                )
            trailing_bytes = decoder.finish()
            self.stats.trailing_bytes += trailing_bytes
            self.stats.format_error_count += decoder.format_error_count
            index_stream.write(
                json.dumps(
                    {
                        "type": "connection_end",
                        "connection": self.stats.connection_count,
                        "trailing_bytes": trailing_bytes,
                        "bytes_discarded": decoder.bytes_discarded,
                        "format_errors": decoder.format_error_count,
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )
            index_stream.flush()
            try:
                client.close()
            except OSError:
                pass
            self._emit("disconnected", asdict(self.stats))

    def _record_frame(
        self,
        index_stream,
        frame: BimgFrame,
        connection_start: int,
        received_ns: int,
        received_monotonic_ns: int,
    ) -> None:
        """记录一帧索引并更新实时统计。"""

        self.stats.decoded_frames += 1
        self.stats.connection_frames += 1
        if not frame.crc_ok:
            self.stats.crc_error_count += 1
        self.stats.last_stream_sequence = frame.header.stream_sequence
        self.stats.last_frame_sequence = frame.header.frame_sequence
        if self.stats.first_frame_ns == 0:
            self.stats.first_frame_ns = received_monotonic_ns
        self.stats.last_frame_ns = received_monotonic_ns
        record = {
            "type": "frame",
            "connection": self.stats.connection_count,
            "file_offset": connection_start + frame.stream_offset,
            "packet_size": frame.header.packet_size,
            "host_received_ns": received_ns,
            "stream_sequence": frame.header.stream_sequence,
            "source_frame_sequence": frame.header.frame_sequence,
            "capture_time_ms": frame.header.capture_time_ms,
            "flags": frame.header.flags,
            "source_camera": frame.header.source_camera,
            "board_id": frame.header.board_id,
            "stream_mode": frame.header.stream_mode,
            "crc_ok": frame.crc_ok,
        }
        index_stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        index_stream.flush()
        self._emit("frame", {**record, "frame_rate_hz": self.stats.frame_rate_hz})

    def _emit(self, event: str, payload: object) -> None:
        """向界面或命令行报告录制事件。"""

        if self.event_callback is not None:
            self.event_callback(event, payload)
