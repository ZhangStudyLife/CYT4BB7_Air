"""三摄图像日志工具的图形界面和命令行入口。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .alignment import align_bimg_to_core_log, export_alignment_csv, read_core_log
from .bimg import scan_bimg_file, write_bimg_index
from .recorder import BimgRecorder


CAMERA_ARGUMENTS = {"front": 0, "center": 1, "back": 2}
CAMERA_NAMES_CN = ("前摄", "下摄", "后摄")


def _default_output(source: Path, suffix: str) -> Path:
    """根据输入文件生成不覆盖原文件的默认输出路径。"""

    return source.with_name(source.stem + suffix)


def _camera_from_entries(entries, requested: str) -> int:
    """返回指定或由有效BIMG帧自动推断的摄像头编号。"""

    if requested != "auto":
        return CAMERA_ARGUMENTS[requested]
    cameras = [
        entry.source_camera
        for entry in entries
        if (
            entry.frame_valid
            and entry.crc_ok
            and entry.stream_mode == 0
            and entry.source_camera in range(3)
        )
    ]
    if not cameras:
        raise ValueError("BIMG中没有有效的摄像头来源编号")
    return max(set(cameras), key=cameras.count)


def _capture_event(event: str, payload: object) -> None:
    """把录制器事件输出为便于保存和排查的单行文本。"""

    if event == "frame":
        data = payload if isinstance(payload, dict) else {}
        print(
            f"\r流序号 {data.get('stream_sequence', 0)} | "
            f"来源帧 {data.get('source_frame_sequence', 0)} | "
            f"{data.get('frame_rate_hz', 0.0):.2f} FPS",
            end="",
            flush=True,
        )
        return
    if event in {"listening", "connected", "disconnected", "stopped"}:
        print(f"\n[{event}] {json.dumps(payload, ensure_ascii=False)}")


def _run_capture(args: argparse.Namespace) -> int:
    """运行TCP监听并把收到的原始BIMG字节流保存到文件。"""

    recorder = BimgRecorder(args.output, host=args.host, port=args.port, event_callback=_capture_event)
    try:
        stats = recorder.run()
    except KeyboardInterrupt:
        recorder.stop()
        stats = recorder.stats
    print(
        f"\n录制结束：{stats.decoded_frames}帧，{stats.received_bytes}字节，"
        f"CRC错误{stats.crc_error_count}，格式错误{stats.format_error_count}。"
    )
    return 0


def _run_index(args: argparse.Namespace) -> int:
    """扫描BIMG并导出帧级索引。"""

    source = Path(args.bimg)
    output = Path(args.output) if args.output else _default_output(source, ".index.csv")
    entries = scan_bimg_file(source)
    write_bimg_index(output, entries)
    crc_errors = sum(not entry.crc_ok for entry in entries)
    print(f"已索引{len(entries)}帧，CRC错误{crc_errors}帧：{output}")
    return 0


def _run_align(args: argparse.Namespace) -> int:
    """配准BIMG与核心0日志并导出逐帧结果。"""

    source = Path(args.bimg)
    output = Path(args.output) if args.output else _default_output(source, ".aligned.csv")
    entries = scan_bimg_file(source)
    rows = read_core_log(args.csv)
    camera_index = _camera_from_entries(entries, args.camera)
    result = align_bimg_to_core_log(
        entries,
        rows,
        camera_index=camera_index,
        manual_sequence_offset=args.offset,
    )
    best = next(
        candidate
        for candidate in result.candidates
        if candidate.sequence_offset == result.sequence_offset
    )
    if result.confidence == "low" and args.offset is None:
        print(
            f"自动配准置信度低，未导出结果。首选偏移{result.sequence_offset:+d}，"
            f"匹配{result.matched_count}/{len(entries)}帧。",
            file=sys.stderr,
        )
        for candidate in result.candidates[:3]:
            print(
                f"候选偏移{candidate.sequence_offset:+d}：匹配{candidate.matched_frames}帧，"
                f"姿态误差{candidate.pose_error:.2f}",
                file=sys.stderr,
            )
        print("请用--offset指定128的整数倍后重试。", file=sys.stderr)
        return 2
    export_alignment_csv(output, entries, rows, result)
    print(
        f"{CAMERA_NAMES_CN[camera_index]}配准完成：{result.matched_count}/{len(entries)}帧，"
        f"偏移{result.sequence_offset:+d}，置信度{result.confidence.upper()}，"
        f"姿态误差{best.pose_error:.2f}：{output}"
    )
    return 0


def _build_parser() -> argparse.ArgumentParser:
    """创建图形界面与三个批处理命令的参数解析器。"""

    parser = argparse.ArgumentParser(description="BIMG录制、核心0日志配准与逐帧回放工具")
    commands = parser.add_subparsers(dest="command")
    commands.add_parser("gui", help="启动图形界面")

    capture = commands.add_parser("capture", help="监听TCP并保存原始BIMG流")
    capture.add_argument("output", help="输出.bimg文件；已存在时拒绝覆盖")
    capture.add_argument("--host", default="0.0.0.0", help="监听地址，默认0.0.0.0")
    capture.add_argument("--port", type=int, default=8086, help="前摄8086，后摄8087")

    index = commands.add_parser("index", help="扫描BIMG并导出帧索引CSV")
    index.add_argument("bimg", help="输入.bimg文件")
    index.add_argument("-o", "--output", help="输出CSV，默认与BIMG同目录")

    align = commands.add_parser("align", help="按来源帧号配准BIMG和核心0 CSV")
    align.add_argument("bimg", help="输入.bimg文件")
    align.add_argument("csv", help="核心0 I0至I35 CSV")
    align.add_argument("-o", "--output", help="输出CSV，默认与BIMG同目录")
    align.add_argument(
        "--camera",
        choices=("auto", "front", "center", "back"),
        default="auto",
        help="摄像头来源，默认根据BIMG自动识别",
    )
    align.add_argument("--offset", type=int, help="手动帧号偏移，必须为128的整数倍")
    return parser


def main(argv: list[str] | None = None) -> int:
    """根据命令行参数启动GUI或执行一个批处理命令。"""

    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command in {None, "gui"}:
            from .app import launch_gui

            launch_gui()
            return 0
        if args.command == "capture":
            return _run_capture(args)
        if args.command == "index":
            return _run_index(args)
        if args.command == "align":
            return _run_align(args)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
