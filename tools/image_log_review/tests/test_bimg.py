"""BIMG v3解析和流恢复测试。"""

from __future__ import annotations

import struct
import unittest
import zlib

from image_log_review.bimg import BimgStreamDecoder, parse_bimg_packet


def make_packet(
    stream_sequence: int = 7,
    frame_sequence: int = 1234,
    capture_time_ms: int = 5678,
    board_id: int = 0,
    source_camera: int = 0,
    flags: int = 3,
    image: bytes | None = None,
) -> bytes:
    """生成与固件BIMG v3完全一致的测试包。"""

    width, height = 188, 120
    image_data = image if image is not None else bytes(width * height)
    debug = b"".join(
        (
            struct.pack("<HBBf", 1, 1, 0, 1.25),
            struct.pack("<HBBf", 2, 1, 0, -2.5),
            struct.pack("<HBBf", 3, 1, 0, 1000.0),
        )
    )
    payload_size = len(image_data) + len(debug)
    header = bytearray(40)
    header[:4] = b"BIMG"
    header[4:8] = bytes((3, 40, 0, board_id))
    struct.pack_into("<HHII", header, 8, width, height, stream_sequence, len(image_data))
    header[20:24] = bytes((0, 6, 3, 8))
    struct.pack_into("<III", header, 24, payload_size, frame_sequence, capture_time_ms)
    header[36:40] = bytes((flags, source_camera, board_id, 0))
    body = bytes(header) + image_data + debug
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


class BimgProtocolTest(unittest.TestCase):
    """验证协议边界、CRC和TCP重同步行为。"""

    def test_golden_vector_crc_and_fields(self) -> None:
        packet = make_packet()
        self.assertEqual(packet[-4:].hex(), "783141d7")
        frame = parse_bimg_packet(packet)
        self.assertTrue(frame.crc_ok)
        self.assertEqual(frame.header.width, 188)
        self.assertEqual(frame.header.height, 120)
        self.assertEqual(frame.header.frame_sequence, 1234)
        self.assertEqual(frame.debug_value(1), 1.25)
        self.assertEqual(frame.debug_value(2), -2.5)
        self.assertEqual(frame.debug_value(3), 1000.0)

    def test_decoder_accepts_every_byte_boundary(self) -> None:
        packet = make_packet()
        decoder = BimgStreamDecoder()
        frames = []
        for value in packet:
            frames.extend(decoder.feed(bytes((value,))))
        self.assertEqual(len(frames), 1)
        self.assertEqual(decoder.finish(), 0)
        self.assertEqual(frames[0].raw_packet, packet)

    def test_decoder_handles_noise_and_multiple_frames(self) -> None:
        first = make_packet(stream_sequence=1, frame_sequence=100)
        second = make_packet(stream_sequence=2, frame_sequence=101)
        decoder = BimgStreamDecoder()
        frames = decoder.feed(b"noise" + first + second)
        self.assertEqual([frame.header.frame_sequence for frame in frames], [100, 101])
        self.assertEqual(frames[0].stream_offset, 5)
        self.assertEqual(frames[1].stream_offset, 5 + len(first))

    def test_crc_failure_resyncs_to_following_frame(self) -> None:
        damaged = bytearray(make_packet(frame_sequence=200))
        damaged[100] ^= 0x01
        good = make_packet(stream_sequence=2, frame_sequence=201)
        decoder = BimgStreamDecoder()
        frames = decoder.feed(bytes(damaged) + good)
        self.assertEqual(
            [(frame.header.frame_sequence, frame.crc_ok) for frame in frames],
            [(200, False), (201, True)],
        )

    def test_crc_result_does_not_depend_on_tcp_chunking(self) -> None:
        damaged = bytearray(make_packet(frame_sequence=200))
        damaged[100] ^= 0x01
        stream = bytes(damaged) + make_packet(stream_sequence=2, frame_sequence=201)
        whole = BimgStreamDecoder().feed(stream)
        split_decoder = BimgStreamDecoder()
        split = []
        for value in stream:
            split.extend(split_decoder.feed(bytes((value,))))
        expected = [(200, False), (201, True)]
        self.assertEqual([(frame.header.frame_sequence, frame.crc_ok) for frame in whole], expected)
        self.assertEqual([(frame.header.frame_sequence, frame.crc_ok) for frame in split], expected)

    def test_truncated_frame_recovers_when_next_header_is_split(self) -> None:
        first = make_packet(frame_sequence=300)
        second = make_packet(stream_sequence=2, frame_sequence=301)
        decoder = BimgStreamDecoder()
        frames = decoder.feed(first[:-10] + second[:10])
        self.assertEqual(frames, [])
        frames.extend(decoder.feed(second[10:]))
        self.assertEqual([frame.header.frame_sequence for frame in frames], [301])
        self.assertGreater(decoder.bytes_discarded, 0)

    def test_raw_packet_has_three_debug_records(self) -> None:
        packet = make_packet()
        frame = parse_bimg_packet(packet)
        self.assertEqual(len(packet), 22628)
        self.assertEqual(frame.header.marker_count, 0)
        self.assertEqual(frame.header.debug_count, 3)
        self.assertEqual(frame.header.payload_size - frame.header.image_size, 24)

    def test_image_magic_does_not_break_valid_packet(self) -> None:
        image = bytearray(188 * 120)
        image[500:504] = b"BIMG"
        frame = BimgStreamDecoder().feed(make_packet(image=bytes(image)))[0]
        self.assertTrue(frame.crc_ok)
        self.assertEqual(frame.image[500:504], b"BIMG")

    def test_back_board_maps_to_back_camera(self) -> None:
        frame = parse_bimg_packet(make_packet(board_id=1, source_camera=2))
        self.assertEqual(frame.header.board_id, 1)
        self.assertEqual(frame.header.source_camera, 2)

    def test_invalid_frame_metadata_keeps_raw_image_parseable(self) -> None:
        frame = parse_bimg_packet(make_packet(source_camera=0xFF, flags=0))
        self.assertFalse(frame.header.frame_valid)
        self.assertEqual(frame.header.source_camera, 0xFF)


if __name__ == "__main__":
    unittest.main()
