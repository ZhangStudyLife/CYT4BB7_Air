"""BIMG v3图像流的解析、校验与文件索引。"""

from __future__ import annotations

import csv
import struct
import zlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import BinaryIO, Iterable, Iterator


BIMG_MAGIC = b"BIMG"
BIMG_VERSION = 3
BIMG_HEADER_SIZE = 40
BIMG_CRC_SIZE = 4
BIMG_MAX_PACKET_SIZE = 4 * 1024 * 1024
BIMG_FRAME_VALID = 0x01
BIMG_TIMESTAMP_VALID = 0x02
BIMG_IMAGE_WIDTH = 188
BIMG_IMAGE_HEIGHT = 120
BIMG_MARKER_SIZE = 6
BIMG_DEBUG_COUNT = 3
BIMG_DEBUG_SIZE = 8


class BimgFormatError(ValueError):
    """表示BIMG帧头或负载结构不合法。"""


@dataclass(frozen=True)
class BimgHeader:
    """保存BIMG v3固定帧头字段。"""

    version: int
    header_size: int
    stream_mode: int
    board_id: int
    width: int
    height: int
    stream_sequence: int
    image_size: int
    marker_count: int
    marker_size: int
    debug_count: int
    debug_size: int
    payload_size: int
    frame_sequence: int
    capture_time_ms: int
    flags: int
    source_camera: int
    reported_board_id: int

    @property
    def frame_valid(self) -> bool:
        """返回来源帧号是否有效。"""

        return bool(self.flags & BIMG_FRAME_VALID)

    @property
    def timestamp_valid(self) -> bool:
        """返回统一采集时间是否有效。"""

        return bool(self.flags & BIMG_TIMESTAMP_VALID)

    @property
    def packet_size(self) -> int:
        """返回包含CRC的完整BIMG包长度，单位为字节。"""

        return self.header_size + self.payload_size + BIMG_CRC_SIZE


@dataclass(frozen=True)
class BimgMarker:
    """保存固件附带的一个识别结果标记。"""

    marker_type: int
    index: int
    x: int
    y: int


@dataclass(frozen=True)
class BimgDebugRecord:
    """保存固件附带的一个浮点调试字段。"""

    record_id: int
    valid: bool
    value: float


@dataclass(frozen=True)
class BimgFrame:
    """保存一帧已完成结构解析的BIMG数据。"""

    header: BimgHeader
    image: bytes
    markers: tuple[BimgMarker, ...]
    debug_records: tuple[BimgDebugRecord, ...]
    crc_expected: int
    crc_actual: int
    raw_packet: bytes = field(repr=False)
    stream_offset: int = 0

    @property
    def crc_ok(self) -> bool:
        """返回整帧CRC32是否正确。"""

        return self.crc_expected == self.crc_actual

    def debug_value(self, record_id: int) -> float | None:
        """按字段编号读取有效调试浮点值，不存在时返回None。"""

        for record in self.debug_records:
            if record.record_id == record_id and record.valid:
                return record.value
        return None


@dataclass(frozen=True)
class BimgIndexEntry:
    """保存BIMG文件内一帧的轻量索引。"""

    frame_index: int
    file_offset: int
    packet_size: int
    stream_sequence: int
    frame_sequence: int
    capture_time_ms: int
    source_camera: int
    board_id: int
    width: int
    height: int
    stream_mode: int
    frame_valid: bool
    timestamp_valid: bool
    crc_ok: bool
    roll_deg: float | None
    pitch_deg: float | None
    height_mm: float | None


def _parse_header(packet: bytes | bytearray | memoryview) -> BimgHeader:
    """解析并校验BIMG固定帧头。"""

    if len(packet) < BIMG_HEADER_SIZE:
        raise BimgFormatError("BIMG header is incomplete")
    if bytes(packet[:4]) != BIMG_MAGIC:
        raise BimgFormatError("BIMG magic does not match")

    version = packet[4]
    header_size = packet[5]
    if version != BIMG_VERSION:
        raise BimgFormatError(f"unsupported BIMG version: {version}")
    if header_size != BIMG_HEADER_SIZE:
        raise BimgFormatError(f"invalid BIMG header size: {header_size}")

    width, height = struct.unpack_from("<HH", packet, 8)
    stream_sequence, image_size = struct.unpack_from("<II", packet, 12)
    payload_size = struct.unpack_from("<I", packet, 24)[0]
    frame_sequence, capture_time_ms = struct.unpack_from("<II", packet, 28)
    marker_count = packet[20]
    marker_size = packet[21]
    debug_count = packet[22]
    debug_size = packet[23]

    if width != BIMG_IMAGE_WIDTH or height != BIMG_IMAGE_HEIGHT:
        raise BimgFormatError("BIMG image dimensions are not 188x120")
    if width == 0 or height == 0 or image_size != width * height:
        raise BimgFormatError("BIMG image dimensions do not match image_size")
    if payload_size < image_size:
        raise BimgFormatError("BIMG payload is smaller than the image")
    metadata_size = marker_count * marker_size + debug_count * debug_size
    if marker_count and marker_size < 6:
        raise BimgFormatError("BIMG marker record is too short")
    if debug_count and debug_size < 8:
        raise BimgFormatError("BIMG debug record is too short")
    if marker_size != BIMG_MARKER_SIZE or debug_count != BIMG_DEBUG_COUNT or debug_size != BIMG_DEBUG_SIZE:
        raise BimgFormatError("BIMG metadata layout does not match firmware v3")
    if packet[6] > 3:
        raise BimgFormatError("BIMG stream mode is outside firmware range")
    if packet[7] not in (0, 1) or packet[38] != packet[7]:
        raise BimgFormatError("BIMG board id is invalid or inconsistent")
    expected_camera = 0 if packet[7] == 0 else 2
    frame_valid = bool(packet[36] & BIMG_FRAME_VALID)
    if (frame_valid and packet[37] != expected_camera) or (
        not frame_valid and packet[37] not in (expected_camera, 0xFF)
    ):
        raise BimgFormatError("BIMG camera source does not match its physical board")
    if image_size + metadata_size != payload_size:
        raise BimgFormatError("BIMG payload_size does not match its records")

    packet_size = header_size + payload_size + BIMG_CRC_SIZE
    if packet_size > BIMG_MAX_PACKET_SIZE:
        raise BimgFormatError("BIMG packet exceeds the safety limit")

    return BimgHeader(
        version=version,
        header_size=header_size,
        stream_mode=packet[6],
        board_id=packet[7],
        width=width,
        height=height,
        stream_sequence=stream_sequence,
        image_size=image_size,
        marker_count=marker_count,
        marker_size=marker_size,
        debug_count=debug_count,
        debug_size=debug_size,
        payload_size=payload_size,
        frame_sequence=frame_sequence,
        capture_time_ms=capture_time_ms,
        flags=packet[36],
        source_camera=packet[37],
        reported_board_id=packet[38],
    )


def parse_bimg_packet(packet: bytes, stream_offset: int = 0) -> BimgFrame:
    """解析一帧完整BIMG包并计算CRC，不因CRC错误丢弃结构信息。"""

    header = _parse_header(packet)
    if len(packet) != header.packet_size:
        raise BimgFormatError(
            f"BIMG packet length {len(packet)} does not match {header.packet_size}"
        )

    payload_start = header.header_size
    image_end = payload_start + header.image_size
    metadata = memoryview(packet)[image_end : header.header_size + header.payload_size]
    markers: list[BimgMarker] = []
    debug_records: list[BimgDebugRecord] = []

    for index in range(header.marker_count):
        offset = index * header.marker_size
        marker = metadata[offset : offset + header.marker_size]
        marker_x, marker_y = struct.unpack_from("<HH", marker, 2)
        markers.append(BimgMarker(marker[0], marker[1], marker_x, marker_y))

    debug_start = header.marker_count * header.marker_size
    for index in range(header.debug_count):
        offset = debug_start + index * header.debug_size
        record = metadata[offset : offset + header.debug_size]
        record_id = struct.unpack_from("<H", record, 0)[0]
        value = struct.unpack_from("<f", record, 4)[0]
        debug_records.append(BimgDebugRecord(record_id, bool(record[2] & 0x01), value))

    crc_expected = struct.unpack_from("<I", packet, len(packet) - BIMG_CRC_SIZE)[0]
    crc_actual = zlib.crc32(packet[:-BIMG_CRC_SIZE]) & 0xFFFFFFFF
    return BimgFrame(
        header=header,
        image=packet[payload_start:image_end],
        markers=tuple(markers),
        debug_records=tuple(debug_records),
        crc_expected=crc_expected,
        crc_actual=crc_actual,
        raw_packet=packet,
        stream_offset=stream_offset,
    )


class BimgStreamDecoder:
    """把任意TCP分包形式的字节流恢复成完整BIMG帧。"""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._buffer_offset = 0
        self.bytes_discarded = 0
        self.format_error_count = 0

    def feed(self, data: bytes) -> list[BimgFrame]:
        """追加一段字节并返回本次新解出的全部完整帧。"""

        if data:
            self._buffer.extend(data)
        return self._decode(final=False)

    def flush(self) -> list[BimgFrame]:
        """在输入结束时输出最后一帧结构完整但CRC错误的诊断帧。"""

        return self._decode(final=True)

    def _decode(self, final: bool) -> list[BimgFrame]:
        """解析当前缓冲区；final表示后续不会再有输入字节。"""

        frames: list[BimgFrame] = []

        while True:
            magic_index = self._buffer.find(BIMG_MAGIC)
            if magic_index < 0:
                keep = min(len(self._buffer), len(BIMG_MAGIC) - 1)
                discard = len(self._buffer) - keep
                if discard:
                    del self._buffer[:discard]
                    self._buffer_offset += discard
                    self.bytes_discarded += discard
                break
            if magic_index:
                del self._buffer[:magic_index]
                self._buffer_offset += magic_index
                self.bytes_discarded += magic_index
            if len(self._buffer) < BIMG_HEADER_SIZE:
                break

            try:
                header = _parse_header(self._buffer)
            except BimgFormatError:
                del self._buffer[0]
                self._buffer_offset += 1
                self.bytes_discarded += 1
                self.format_error_count += 1
                continue
            if len(self._buffer) < header.packet_size:
                candidate, incomplete = self._find_valid_frame_start(header.packet_size)
                if candidate is not None:
                    self._discard_to(candidate)
                    self.format_error_count += 1
                    continue
                if incomplete and not final:
                    break
                break

            packet_offset = self._buffer_offset
            packet = bytes(self._buffer[: header.packet_size])
            try:
                frame = parse_bimg_packet(packet, packet_offset)
            except BimgFormatError:
                del self._buffer[0]
                self._buffer_offset += 1
                self.bytes_discarded += 1
                self.format_error_count += 1
                continue

            if not frame.crc_ok:
                candidate, incomplete = self._find_valid_frame_start(header.packet_size)
                if candidate is not None:
                    self._discard_to(candidate)
                    self.format_error_count += 1
                    continue
                if incomplete and not final:
                    break

            del self._buffer[: header.packet_size]
            self._buffer_offset += header.packet_size
            frames.append(frame)

        return frames

    def _find_valid_frame_start(self, current_packet_size: int) -> tuple[int | None, bool]:
        """在当前包内部查找下一份CRC正确的完整帧或尚未收全的候选帧头。"""

        search_from = 1
        incomplete = False
        while True:
            magic_index = self._buffer.find(BIMG_MAGIC, search_from)
            if magic_index < 0 or magic_index >= current_packet_size:
                return None, incomplete
            if len(self._buffer) - magic_index < BIMG_HEADER_SIZE:
                return None, True
            try:
                header = _parse_header(memoryview(self._buffer)[magic_index:])
            except BimgFormatError:
                search_from = magic_index + 1
                continue
            if len(self._buffer) - magic_index < header.packet_size:
                incomplete = True
                search_from = magic_index + 1
                continue
            packet = bytes(self._buffer[magic_index : magic_index + header.packet_size])
            try:
                if parse_bimg_packet(packet).crc_ok:
                    return magic_index, incomplete
            except BimgFormatError:
                pass
            search_from = magic_index + 1

    def _discard_to(self, offset: int) -> None:
        """丢弃损坏前缀并同步维护文件绝对偏移和诊断计数。"""

        del self._buffer[:offset]
        self._buffer_offset += offset
        self.bytes_discarded += offset

    def finish(self) -> int:
        """结束输入并返回尚未组成完整帧的尾部字节数。"""

        return len(self._buffer)


def iter_bimg_stream(stream: BinaryIO, chunk_size: int = 64 * 1024) -> Iterator[BimgFrame]:
    """从二进制流中顺序产生BIMG帧。"""

    decoder = BimgStreamDecoder()
    while True:
        chunk = stream.read(chunk_size)
        if not chunk:
            break
        yield from decoder.feed(chunk)
    yield from decoder.flush()


def iter_bimg_file(path: str | Path) -> Iterator[BimgFrame]:
    """从指定文件中顺序产生BIMG帧。"""

    with Path(path).open("rb") as stream:
        yield from iter_bimg_stream(stream)


def frame_to_index(frame: BimgFrame, frame_index: int) -> BimgIndexEntry:
    """把完整BIMG帧转换成不含图像数据的索引项。"""

    return BimgIndexEntry(
        frame_index=frame_index,
        file_offset=frame.stream_offset,
        packet_size=frame.header.packet_size,
        stream_sequence=frame.header.stream_sequence,
        frame_sequence=frame.header.frame_sequence,
        capture_time_ms=frame.header.capture_time_ms,
        source_camera=frame.header.source_camera,
        board_id=frame.header.board_id,
        width=frame.header.width,
        height=frame.header.height,
        stream_mode=frame.header.stream_mode,
        frame_valid=frame.header.frame_valid,
        timestamp_valid=frame.header.timestamp_valid,
        crc_ok=frame.crc_ok,
        roll_deg=frame.debug_value(0x0001),
        pitch_deg=frame.debug_value(0x0002),
        height_mm=frame.debug_value(0x0003),
    )


def scan_bimg_file(path: str | Path) -> list[BimgIndexEntry]:
    """扫描BIMG文件并返回可随机读取的帧索引。"""

    return [frame_to_index(frame, index) for index, frame in enumerate(iter_bimg_file(path))]


def read_bimg_frame(path: str | Path, entry: BimgIndexEntry) -> BimgFrame:
    """根据索引从BIMG文件随机读取一帧。"""

    with Path(path).open("rb") as stream:
        stream.seek(entry.file_offset)
        packet = stream.read(entry.packet_size)
    return parse_bimg_packet(packet, entry.file_offset)


def write_bimg_index(path: str | Path, entries: Iterable[BimgIndexEntry]) -> None:
    """将BIMG帧索引写成便于人工检查的CSV文件。"""

    fieldnames = list(BimgIndexEntry.__dataclass_fields__)
    with Path(path).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for entry in entries:
            writer.writerow({name: getattr(entry, name) for name in fieldnames})
