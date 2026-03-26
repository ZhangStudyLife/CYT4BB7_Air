#!/usr/bin/env python3
"""离线分析 PMW3901 光流与陀螺解耦效果。"""

from __future__ import annotations

import argparse
import csv
import html
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ALPHA = 0.089955
FIT_K = 10.0
DT_S = 0.001
WINDOW_SAMPLES = 20
TRACE_POINTS = 1400
HIST_BINS = 48


@dataclass(frozen=True)
class LogData:
    name: str
    tick: np.ndarray
    delta_x: np.ndarray
    delta_y: np.ndarray
    squal: np.ndarray
    gyro_x: np.ndarray
    gyro_y: np.ndarray
    gyro_z: np.ndarray
    pitch: np.ndarray
    roll: np.ndarray
    yaw: np.ndarray


@dataclass(frozen=True)
class AxisStats:
    mean: float
    rms: float
    mae: float
    p95_abs: float


@dataclass(frozen=True)
class OffsetMetrics:
    offset: int
    dec_x_rms: float
    dec_y_rms: float
    score: float


@dataclass(frozen=True)
class AnalysisResult:
    log: LogData
    offset: int
    time_50hz: np.ndarray
    raw_x: np.ndarray
    raw_y: np.ndarray
    dec_x: np.ndarray
    dec_y: np.ndarray
    sum_x: np.ndarray
    sum_y: np.ndarray
    raw_x_stats: AxisStats
    raw_y_stats: AxisStats
    dec_x_stats: AxisStats
    dec_y_stats: AxisStats
    squal_mean: float
    squal_median: float
    squal_min: float
    offset_scan: list[OffsetMetrics]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="分析 PMW 光流与陀螺解耦效果")
    parser.add_argument("--input-dir", type=Path, default=Path("."), help="输入目录，默认当前目录")
    parser.add_argument("--output-dir", type=Path, default=Path("."), help="输出目录，默认当前目录")
    return parser.parse_args()


def load_csv(path: Path) -> LogData:
    buffers: dict[str, list[float]] = {f"I{i}": [] for i in range(13)}

    with path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError(f"{path.name} 缺少表头")
        for column in buffers:
            if column not in reader.fieldnames:
                raise ValueError(f"{path.name} 缺少列 {column}")
        for row_index, row in enumerate(reader, start=2):
            try:
                for column in buffers:
                    buffers[column].append(float(row[column]))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path.name} 第 {row_index} 行存在非法浮点值") from exc

    return LogData(
        name=path.stem,
        tick=np.asarray(buffers["I0"], dtype=np.float64),
        delta_x=np.asarray(buffers["I1"], dtype=np.float64),
        delta_y=np.asarray(buffers["I2"], dtype=np.float64),
        squal=np.asarray(buffers["I3"], dtype=np.float64),
        gyro_x=np.asarray(buffers["I4"], dtype=np.float64),
        gyro_y=np.asarray(buffers["I5"], dtype=np.float64),
        gyro_z=np.asarray(buffers["I6"], dtype=np.float64),
        pitch=np.asarray(buffers["I10"], dtype=np.float64),
        roll=np.asarray(buffers["I11"], dtype=np.float64),
        yaw=np.asarray(buffers["I12"], dtype=np.float64),
    )


def calc_stats(values: np.ndarray) -> AxisStats:
    arr = np.asarray(values, dtype=np.float64)
    return AxisStats(
        mean=float(np.mean(arr)),
        rms=float(np.sqrt(np.mean(arr * arr))),
        mae=float(np.mean(np.abs(arr))),
        p95_abs=float(np.percentile(np.abs(arr), 95.0)),
    )


def replay_with_offset(log: LogData, offset: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    lpf_x = 0.0
    lpf_y = 0.0
    sum_x = 0.0
    sum_y = 0.0
    raw_x: list[float] = []
    raw_y: list[float] = []
    dec_x: list[float] = []
    dec_y: list[float] = []
    comp_x: list[float] = []
    comp_y: list[float] = []

    for idx in range(log.tick.size):
        lpf_x += ALPHA * (float(log.gyro_x[idx]) - lpf_x)
        lpf_y += ALPHA * (float(log.gyro_y[idx]) - lpf_y)
        sum_x += FIT_K * lpf_x * DT_S
        sum_y += FIT_K * lpf_y * DT_S

        if idx >= offset and ((idx - offset) % WINDOW_SAMPLES) == 0:
            dx = float(log.delta_x[idx])
            dy = float(log.delta_y[idx])
            raw_x.append(dx)
            raw_y.append(dy)
            dec_x.append(dx - sum_x)
            dec_y.append(dy - sum_y)
            comp_x.append(sum_x)
            comp_y.append(sum_y)
            sum_x = 0.0
            sum_y = 0.0

    return (
        np.asarray(raw_x, dtype=np.float64),
        np.asarray(raw_y, dtype=np.float64),
        np.asarray(dec_x, dtype=np.float64),
        np.asarray(dec_y, dtype=np.float64),
        np.asarray(comp_x, dtype=np.float64),
        np.asarray(comp_y, dtype=np.float64),
    )


def analyze_log(log: LogData) -> AnalysisResult:
    offset_scan: list[OffsetMetrics] = []
    best_offset = 0
    best_score = float("inf")
    best_payload: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None

    for offset in range(WINDOW_SAMPLES):
        raw_x, raw_y, dec_x, dec_y, comp_x, comp_y = replay_with_offset(log, offset)
        score = float(np.sqrt(np.mean(dec_x * dec_x)) + np.sqrt(np.mean(dec_y * dec_y)))
        offset_scan.append(
            OffsetMetrics(
                offset=offset,
                dec_x_rms=float(np.sqrt(np.mean(dec_x * dec_x))),
                dec_y_rms=float(np.sqrt(np.mean(dec_y * dec_y))),
                score=score,
            )
        )
        if score < best_score:
            best_score = score
            best_offset = offset
            best_payload = (raw_x, raw_y, dec_x, dec_y, comp_x, comp_y)

    assert best_payload is not None
    raw_x, raw_y, dec_x, dec_y, comp_x, comp_y = best_payload
    time_50hz = ((np.arange(raw_x.size, dtype=np.float64) * WINDOW_SAMPLES) + best_offset) * DT_S

    return AnalysisResult(
        log=log,
        offset=best_offset,
        time_50hz=time_50hz,
        raw_x=raw_x,
        raw_y=raw_y,
        dec_x=dec_x,
        dec_y=dec_y,
        sum_x=comp_x,
        sum_y=comp_y,
        raw_x_stats=calc_stats(raw_x),
        raw_y_stats=calc_stats(raw_y),
        dec_x_stats=calc_stats(dec_x),
        dec_y_stats=calc_stats(dec_y),
        squal_mean=float(np.mean(log.squal)),
        squal_median=float(np.median(log.squal)),
        squal_min=float(np.min(log.squal)),
        offset_scan=offset_scan,
    )


def axis_judgement(raw_stats: AxisStats, dec_stats: AxisStats) -> str:
    mean_ok = abs(dec_stats.mean) <= 0.1
    rms_ok = dec_stats.rms <= raw_stats.rms * 0.35
    if mean_ok and rms_ok:
        return "解耦后已接近 0"
    if mean_ok:
        return "均值贴近 0，但波动还不够小"
    return "没有稳定贴近 0"


def fmt_stats(stats: AxisStats) -> str:
    return (
        f"mean={stats.mean:.4f}, rms={stats.rms:.4f}, "
        f"mae={stats.mae:.4f}, p95_abs={stats.p95_abs:.4f}"
    )


def escape(text: str) -> str:
    return html.escape(text, quote=True)


def polyline_points(x: np.ndarray, y: np.ndarray, width: int, height: int, x_min: float, x_max: float, y_min: float, y_max: float) -> str:
    if x.size == 0:
        return ""
    x_span = max(1e-9, x_max - x_min)
    y_span = max(1e-9, y_max - y_min)
    points: list[str] = []
    for xv, yv in zip(x, y):
        px = 60.0 + (float(xv) - x_min) / x_span * (width - 90.0)
        py = 20.0 + (y_max - float(yv)) / y_span * (height - 60.0)
        points.append(f"{px:.2f},{py:.2f}")
    return " ".join(points)


def polyline_points_offset(
    x: np.ndarray,
    y: np.ndarray,
    width: int,
    height: int,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
) -> str:
    if x.size == 0:
        return ""
    x_span = max(1e-9, x_max - x_min)
    y_span = max(1e-9, y_max - y_min)
    points: list[str] = []
    for xv, yv in zip(x, y):
        px = left + (float(xv) - x_min) / x_span * width
        py = top + (y_max - float(yv)) / y_span * height
        points.append(f"{px:.2f},{py:.2f}")
    return " ".join(points)


def choose_trace_points(time_s: np.ndarray, *signals: np.ndarray) -> tuple[np.ndarray, list[np.ndarray]]:
    if time_s.size <= TRACE_POINTS:
        return time_s, [np.asarray(sig, dtype=np.float64) for sig in signals]
    indices = np.linspace(0, time_s.size - 1, TRACE_POINTS).astype(np.int64)
    return time_s[indices], [np.asarray(sig, dtype=np.float64)[indices] for sig in signals]


def make_trace_svg(result: AnalysisResult) -> str:
    width = 1200
    height = 680
    time_ds, signals = choose_trace_points(
        result.time_50hz, result.raw_x, result.dec_x, result.raw_y, result.dec_y
    )
    raw_x_ds, dec_x_ds, raw_y_ds, dec_y_ds = signals
    y_max = max(np.max(np.abs(raw_x_ds)), np.max(np.abs(dec_x_ds)), np.max(np.abs(raw_y_ds)), np.max(np.abs(dec_y_ds)), 1.0)
    y_min = -y_max
    x_min = float(time_ds[0])
    x_max = float(time_ds[-1])
    zero_y = 20.0 + (y_max - 0.0) / max(1e-9, y_max - y_min) * (height - 60.0)

    lines = [
        f'<line x1="60" y1="{zero_y:.2f}" x2="{width-30}" y2="{zero_y:.2f}" stroke="#666" stroke-dasharray="6 4" stroke-width="1"/>',
        f'<polyline fill="none" stroke="#b33a3a" stroke-width="1.5" points="{polyline_points(time_ds, raw_x_ds, width, height, x_min, x_max, y_min, y_max)}"/>',
        f'<polyline fill="none" stroke="#1d78b5" stroke-width="1.5" points="{polyline_points(time_ds, dec_x_ds, width, height, x_min, x_max, y_min, y_max)}"/>',
        f'<polyline fill="none" stroke="#d28b25" stroke-width="1.5" points="{polyline_points(time_ds, raw_y_ds, width, height, x_min, x_max, y_min, y_max)}"/>',
        f'<polyline fill="none" stroke="#2e9b54" stroke-width="1.5" points="{polyline_points(time_ds, dec_y_ds, width, height, x_min, x_max, y_min, y_max)}"/>',
    ]

    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#fffdf8"/>
<text x="60" y="18" font-size="20" font-family="Arial" fill="#222">原始光流与解耦后光流时序对比（50Hz窗口）</text>
<text x="60" y="{height-10}" font-size="12" font-family="Arial" fill="#444">时间 / s</text>
<text x="8" y="{height/2:.0f}" font-size="12" font-family="Arial" fill="#444" transform="rotate(-90 8,{height/2:.0f})">delta / count</text>
<line x1="60" y1="20" x2="60" y2="{height-40}" stroke="#222" stroke-width="1.2"/>
<line x1="60" y1="{height-40}" x2="{width-30}" y2="{height-40}" stroke="#222" stroke-width="1.2"/>
{''.join(lines)}
<rect x="{width-320}" y="40" width="260" height="98" fill="#fff" stroke="#ddd"/>
<line x1="{width-300}" y1="64" x2="{width-265}" y2="64" stroke="#b33a3a" stroke-width="2"/><text x="{width-255}" y="68" font-size="13" font-family="Arial">raw_x</text>
<line x1="{width-300}" y1="88" x2="{width-265}" y2="88" stroke="#1d78b5" stroke-width="2"/><text x="{width-255}" y="92" font-size="13" font-family="Arial">dec_x</text>
<line x1="{width-300}" y1="112" x2="{width-265}" y2="112" stroke="#d28b25" stroke-width="2"/><text x="{width-255}" y="116" font-size="13" font-family="Arial">raw_y</text>
<line x1="{width-300}" y1="136" x2="{width-265}" y2="136" stroke="#2e9b54" stroke-width="2"/><text x="{width-255}" y="140" font-size="13" font-family="Arial">dec_y</text>
</svg>"""


def make_offset_compare_svg(log: LogData, offsets: list[int]) -> str:
    width = 1200
    panel_height = 180
    height = 50 + panel_height * len(offsets)
    blocks: list[str] = [
        f'<rect width="100%" height="100%" fill="#fffdf8"/>',
        '<text x="60" y="24" font-size="20" font-family="Arial" fill="#222">03252307 不同 offset 的 dec_x 对齐对比</text>',
        '<text x="60" y="44" font-size="12" font-family="Arial" fill="#555">同一份日志，同一套 15Hz低通 + 10*gyro*dt，仅改变 20ms 窗口起点。蓝线越贴近 0，说明这个 offset 越对齐。</text>',
    ]

    for panel_idx, offset in enumerate(offsets):
        raw_x, raw_y, dec_x, dec_y, comp_x, comp_y = replay_with_offset(log, offset)
        time_50hz = ((np.arange(raw_x.size, dtype=np.float64) * WINDOW_SAMPLES) + offset) * DT_S
        time_ds, signals = choose_trace_points(time_50hz, raw_x, dec_x)
        raw_x_ds, dec_x_ds = signals
        y_max = max(float(np.max(np.abs(raw_x_ds))), float(np.max(np.abs(dec_x_ds))), 1.0)
        y_min = -y_max
        x_min = float(time_ds[0])
        x_max = float(time_ds[-1])
        top = 50 + panel_idx * panel_height
        plot_top = top + 10
        plot_height = panel_height - 60
        bottom = plot_top + plot_height
        plot_width = width - 90
        zero_y = plot_top + (y_max - 0.0) / max(1e-9, y_max - y_min) * plot_height
        blocks.extend(
            [
                f'<text x="60" y="{top-6}" font-size="15" font-family="Arial" fill="#222">offset={offset}  raw_x_rms={calc_stats(raw_x).rms:.3f}  dec_x_rms={calc_stats(dec_x).rms:.3f}</text>',
                f'<line x1="60" y1="{plot_top}" x2="60" y2="{bottom}" stroke="#222" stroke-width="1.1"/>',
                f'<line x1="60" y1="{bottom}" x2="{width-30}" y2="{bottom}" stroke="#222" stroke-width="1.1"/>',
                f'<line x1="60" y1="{zero_y:.2f}" x2="{width-30}" y2="{zero_y:.2f}" stroke="#666" stroke-dasharray="6 4" stroke-width="1"/>',
                f'<polyline fill="none" stroke="#c35a42" stroke-width="1.3" points="{polyline_points_offset(time_ds, raw_x_ds, plot_width, plot_height, x_min, x_max, y_min, y_max, 60.0, plot_top)}"/>',
                f'<polyline fill="none" stroke="#1d78b5" stroke-width="1.5" points="{polyline_points_offset(time_ds, dec_x_ds, plot_width, plot_height, x_min, x_max, y_min, y_max, 60.0, plot_top)}"/>',
            ]
        )

    return f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">{"".join(blocks)}</svg>'


def make_offset_svg(result: AnalysisResult) -> str:
    width = 980
    height = 520
    offsets = np.asarray([item.offset for item in result.offset_scan], dtype=np.float64)
    dec_x_rms = np.asarray([item.dec_x_rms for item in result.offset_scan], dtype=np.float64)
    dec_y_rms = np.asarray([item.dec_y_rms for item in result.offset_scan], dtype=np.float64)
    y_max = max(float(np.max(dec_x_rms)), float(np.max(dec_y_rms)), result.raw_x_stats.rms, result.raw_y_stats.rms, 1.0)
    y_min = 0.0
    x_min = 0.0
    x_max = float(WINDOW_SAMPLES - 1)
    raw_x_line_y = 20.0 + (y_max - result.raw_x_stats.rms) / max(1e-9, y_max - y_min) * (height - 60.0)
    raw_y_line_y = 20.0 + (y_max - result.raw_y_stats.rms) / max(1e-9, y_max - y_min) * (height - 60.0)
    best_x = 60.0 + result.offset / max(1e-9, x_max - x_min) * (width - 90.0)

    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#f9fbff"/>
<text x="60" y="18" font-size="20" font-family="Arial" fill="#222">20个窗口相位 offset 扫描</text>
<line x1="60" y1="20" x2="60" y2="{height-40}" stroke="#222" stroke-width="1.2"/>
<line x1="60" y1="{height-40}" x2="{width-30}" y2="{height-40}" stroke="#222" stroke-width="1.2"/>
<line x1="60" y1="{raw_x_line_y:.2f}" x2="{width-30}" y2="{raw_x_line_y:.2f}" stroke="#b33a3a" stroke-dasharray="7 5" stroke-width="1.3"/>
<line x1="60" y1="{raw_y_line_y:.2f}" x2="{width-30}" y2="{raw_y_line_y:.2f}" stroke="#d28b25" stroke-dasharray="7 5" stroke-width="1.3"/>
<line x1="{best_x:.2f}" y1="20" x2="{best_x:.2f}" y2="{height-40}" stroke="#444" stroke-dasharray="6 6" stroke-width="1.2"/>
<polyline fill="none" stroke="#1d78b5" stroke-width="2" points="{polyline_points(offsets, dec_x_rms, width, height, x_min, x_max, y_min, y_max)}"/>
<polyline fill="none" stroke="#2e9b54" stroke-width="2" points="{polyline_points(offsets, dec_y_rms, width, height, x_min, x_max, y_min, y_max)}"/>
<rect x="{width-290}" y="38" width="230" height="116" fill="#fff" stroke="#ddd"/>
<line x1="{width-270}" y1="62" x2="{width-235}" y2="62" stroke="#b33a3a" stroke-width="2" stroke-dasharray="7 5"/><text x="{width-225}" y="66" font-size="13" font-family="Arial">raw_x rms</text>
<line x1="{width-270}" y1="86" x2="{width-235}" y2="86" stroke="#d28b25" stroke-width="2" stroke-dasharray="7 5"/><text x="{width-225}" y="90" font-size="13" font-family="Arial">raw_y rms</text>
<line x1="{width-270}" y1="110" x2="{width-235}" y2="110" stroke="#1d78b5" stroke-width="2"/><text x="{width-225}" y="114" font-size="13" font-family="Arial">dec_x rms</text>
<line x1="{width-270}" y1="134" x2="{width-235}" y2="134" stroke="#2e9b54" stroke-width="2"/><text x="{width-225}" y="138" font-size="13" font-family="Arial">dec_y rms</text>
<text x="60" y="{height-10}" font-size="12" font-family="Arial" fill="#444">offset / sample</text>
<text x="10" y="{height/2:.0f}" font-size="12" font-family="Arial" fill="#444" transform="rotate(-90 10,{height/2:.0f})">rms / count</text>
</svg>"""


def make_histogram_svg(result: AnalysisResult) -> str:
    width = 980
    height = 520
    hist_x, edges_x = np.histogram(result.dec_x, bins=HIST_BINS)
    hist_y, edges_y = np.histogram(result.dec_y, bins=HIST_BINS)
    x_min = float(min(edges_x[0], edges_y[0]))
    x_max = float(max(edges_x[-1], edges_y[-1]))
    max_count = float(max(np.max(hist_x), np.max(hist_y), 1))
    plot_width = width - 90
    bar_width = plot_width / HIST_BINS

    bars: list[str] = []
    for idx in range(HIST_BINS):
        left = 60.0 + idx * bar_width
        x_h = float(hist_x[idx]) / max_count * (height - 80.0)
        y_h = float(hist_y[idx]) / max_count * (height - 80.0)
        bars.append(f'<rect x="{left:.2f}" y="{height-40-x_h:.2f}" width="{max(1.0, bar_width-1.5):.2f}" height="{x_h:.2f}" fill="#1d78b5" fill-opacity="0.45"/>')
        bars.append(f'<rect x="{left:.2f}" y="{height-40-y_h:.2f}" width="{max(1.0, bar_width-1.5):.2f}" height="{y_h:.2f}" fill="#2e9b54" fill-opacity="0.45"/>')

    zero_x = 60.0 + (0.0 - x_min) / max(1e-9, x_max - x_min) * (width - 90.0)

    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#fff"/>
<text x="60" y="18" font-size="20" font-family="Arial" fill="#222">解耦后光流分布</text>
<line x1="60" y1="20" x2="60" y2="{height-40}" stroke="#222" stroke-width="1.2"/>
<line x1="60" y1="{height-40}" x2="{width-30}" y2="{height-40}" stroke="#222" stroke-width="1.2"/>
<line x1="{zero_x:.2f}" y1="20" x2="{zero_x:.2f}" y2="{height-40}" stroke="#666" stroke-dasharray="6 4" stroke-width="1.1"/>
{''.join(bars)}
<rect x="{width-280}" y="38" width="210" height="72" fill="#fff" stroke="#ddd"/>
<rect x="{width-260}" y="58" width="22" height="12" fill="#1d78b5" fill-opacity="0.45"/><text x="{width-230}" y="68" font-size="13" font-family="Arial">dec_x</text>
<rect x="{width-260}" y="86" width="22" height="12" fill="#2e9b54" fill-opacity="0.45"/><text x="{width-230}" y="96" font-size="13" font-family="Arial">dec_y</text>
<text x="60" y="{height-10}" font-size="12" font-family="Arial" fill="#444">decoupled delta / count</text>
<text x="12" y="{height/2:.0f}" font-size="12" font-family="Arial" fill="#444" transform="rotate(-90 12,{height/2:.0f})">count</text>
</svg>"""


def write_summary_csv(path: Path, result: AnalysisResult) -> None:
    with path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["axis", "raw_mean", "raw_rms", "raw_mae", "raw_p95_abs", "dec_mean", "dec_rms", "dec_mae", "dec_p95_abs", "rms_ratio", "mae_ratio", "judgement"])
        for axis, raw_stats, dec_stats in (
            ("x", result.raw_x_stats, result.dec_x_stats),
            ("y", result.raw_y_stats, result.dec_y_stats),
        ):
            writer.writerow(
                [
                    axis,
                    f"{raw_stats.mean:.6f}",
                    f"{raw_stats.rms:.6f}",
                    f"{raw_stats.mae:.6f}",
                    f"{raw_stats.p95_abs:.6f}",
                    f"{dec_stats.mean:.6f}",
                    f"{dec_stats.rms:.6f}",
                    f"{dec_stats.mae:.6f}",
                    f"{dec_stats.p95_abs:.6f}",
                    f"{dec_stats.rms / max(1e-9, raw_stats.rms):.6f}",
                    f"{dec_stats.mae / max(1e-9, raw_stats.mae):.6f}",
                    axis_judgement(raw_stats, dec_stats),
                ]
            )


def result_section(result: AnalysisResult) -> str:
    x_judge = axis_judgement(result.raw_x_stats, result.dec_x_stats)
    y_judge = axis_judgement(result.raw_y_stats, result.dec_y_stats)
    improvement_x = 1.0 - result.dec_x_stats.rms / max(1e-9, result.raw_x_stats.rms)
    improvement_y = 1.0 - result.dec_y_stats.rms / max(1e-9, result.raw_y_stats.rms)

    return f"""
<h2>{escape(result.log.name)}</h2>
<p>最佳窗口相位 offset = <strong>{result.offset}</strong>。X 轴 RMS 下降 <strong>{improvement_x * 100.0:.1f}%</strong>，Y 轴 RMS 下降 <strong>{improvement_y * 100.0:.1f}%</strong>。</p>
<ul>
  <li>X 轴原始: {escape(fmt_stats(result.raw_x_stats))}</li>
  <li>X 轴解耦: {escape(fmt_stats(result.dec_x_stats))}</li>
  <li>X 轴结论: <strong>{escape(x_judge)}</strong></li>
  <li>Y 轴原始: {escape(fmt_stats(result.raw_y_stats))}</li>
  <li>Y 轴解耦: {escape(fmt_stats(result.dec_y_stats))}</li>
  <li>Y 轴结论: <strong>{escape(y_judge)}</strong></li>
  <li>SQUAL: mean={result.squal_mean:.2f}, median={result.squal_median:.2f}, min={result.squal_min:.2f}</li>
</ul>
"""


def write_html_report(path: Path, result: AnalysisResult, trace_name: str, offset_name: str, hist_name: str, summary_name: str) -> None:
    x_judge = axis_judgement(result.raw_x_stats, result.dec_x_stats)
    y_judge = axis_judgement(result.raw_y_stats, result.dec_y_stats)
    html_text = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8"/>
  <title>{escape(result.log.name)} 解耦分析报告</title>
  <style>
    body {{ font-family: "Segoe UI", Arial, sans-serif; margin: 24px auto; max-width: 1200px; color: #222; background: #fcfbf7; }}
    h1, h2 {{ margin-bottom: 8px; }}
    .card {{ background: #fff; border: 1px solid #e6dfd3; padding: 18px 20px; margin-bottom: 20px; box-shadow: 0 2px 10px rgba(0,0,0,0.04); }}
    ul {{ margin-top: 10px; }}
    code {{ background: #f3efe7; padding: 1px 4px; }}
    .links a {{ margin-right: 18px; }}
    .good {{ color: #196c2e; font-weight: 700; }}
  </style>
</head>
<body>
  <h1>{escape(result.log.name)} PMW/Gyro 解耦报告</h1>
  <div class="card">
    <p>本报告按固件当前逻辑回放：<code>15Hz 一阶低通 + 10 * gyro_lpf * 0.001</code>，每 20 个样本作为一个 50Hz 窗口做一次 <code>delta - compensation</code>。</p>
    <p>X 轴判定：<span class="good">{escape(x_judge)}</span>；Y 轴判定：<span class="good">{escape(y_judge)}</span>。</p>
    <div class="links">
      <a href="{escape(trace_name)}">{escape(trace_name)}</a>
      <a href="{escape(offset_name)}">{escape(offset_name)}</a>
      <a href="{escape(hist_name)}">{escape(hist_name)}</a>
      <a href="{escape(summary_name)}">{escape(summary_name)}</a>
    </div>
  </div>
  <div class="card">{result_section(result)}</div>
  <div class="card">{make_trace_svg(result)}</div>
  <div class="card">{make_offset_svg(result)}</div>
  <div class="card">{make_histogram_svg(result)}</div>
</body>
</html>"""
    path.write_text(html_text, encoding="utf-8")


def write_index(path: Path, results: list[AnalysisResult]) -> None:
    sections = []
    for result in results:
        report_name = f"{result.log.name}_decouple_report.html"
        sections.append(
            f'<div class="card">{result_section(result)}<p><a href="{escape(report_name)}">打开详细报告</a></p></div>'
        )

    html_text = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8"/>
  <title>0325 PMW 解耦汇总</title>
  <style>
    body {{ font-family: "Segoe UI", Arial, sans-serif; margin: 24px auto; max-width: 1200px; color: #222; background: #f7f8fa; }}
    .card {{ background: #fff; border: 1px solid #d9dde4; padding: 18px 20px; margin-bottom: 20px; box-shadow: 0 2px 8px rgba(0,0,0,0.04); }}
  </style>
</head>
<body>
  <h1>0325 PMW/Gyro 解耦汇总</h1>
  <div class="card">
    <p>分析对象：两份 <code>*_pmwgyro.csv</code> 日志。判定规则：<code>|mean| &lt;= 0.1</code> 且 <code>dec_rms &lt;= 0.35 * raw_rms</code> 时认为该轴“解耦后已接近 0”。</p>
  </div>
  {''.join(sections)}
</body>
</html>"""
    path.write_text(html_text, encoding="utf-8")


def save_svg(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")


def main() -> None:
    args = parse_args()
    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    csv_paths = sorted(input_dir.glob("*_pmwgyro.csv"))
    if not csv_paths:
        raise SystemExit("没有找到 *_pmwgyro.csv 日志文件")

    results: list[AnalysisResult] = []
    for csv_path in csv_paths:
        log = load_csv(csv_path)
        result = analyze_log(log)
        results.append(result)

        trace_name = f"{log.name}_trace.svg"
        offset_name = f"{log.name}_offset_scan.svg"
        hist_name = f"{log.name}_hist.svg"
        summary_name = f"{log.name}_summary.csv"
        report_name = f"{log.name}_decouple_report.html"

        save_svg(output_dir / trace_name, make_trace_svg(result))
        save_svg(output_dir / offset_name, make_offset_svg(result))
        save_svg(output_dir / hist_name, make_histogram_svg(result))
        write_summary_csv(output_dir / summary_name, result)
        write_html_report(output_dir / report_name, result, trace_name, offset_name, hist_name, summary_name)

        print(f"{log.name}: offset={result.offset}, raw_x_rms={result.raw_x_stats.rms:.4f}, dec_x_rms={result.dec_x_stats.rms:.4f}, raw_y_rms={result.raw_y_stats.rms:.4f}, dec_y_rms={result.dec_y_stats.rms:.4f}")

    write_index(output_dir / "index.html", results)

    for result in results:
        if result.log.name == "03252307_pmwgyro":
            save_svg(output_dir / "03252307_offset_compare.svg", make_offset_compare_svg(result.log, [0, 5, 10, 15]))
            break


if __name__ == "__main__":
    main()
