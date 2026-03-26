#!/usr/bin/env python3
"""固定 offset=0、alpha=0.05 时分析解耦后光流频域。"""

from __future__ import annotations

import argparse
import csv
import html
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

GYRO_LPF_ALPHA = 0.05
FS_1000HZ = 1000.0
WINDOW_SAMPLES = 20
FS_DEC_HZ = FS_1000HZ / WINDOW_SAMPLES
FIT_K = 10.0
DT_S = 0.001
FFT_POINTS_LIMIT = 2048


@dataclass(frozen=True)
class LogData:
    name: str
    delta_x: np.ndarray
    delta_y: np.ndarray
    gyro_x: np.ndarray
    gyro_y: np.ndarray


@dataclass(frozen=True)
class SpectrumSummary:
    peak_hz: float
    peak_amp: float
    p95_hz: float
    band_0_5: float
    band_5_10: float
    band_10_20: float
    band_20_25: float


@dataclass(frozen=True)
class LogResult:
    log: LogData
    dec_x: np.ndarray
    dec_y: np.ndarray
    raw_x: np.ndarray
    raw_y: np.ndarray
    freqs: np.ndarray
    amp_dec_x: np.ndarray
    amp_dec_y: np.ndarray
    amp_raw_x: np.ndarray
    amp_raw_y: np.ndarray
    x_summary: SpectrumSummary
    y_summary: SpectrumSummary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="分析 offset=0 alpha=0.05 时 dec_x/dec_y 的频域")
    parser.add_argument("--input-dir", type=Path, default=Path("."), help="输入目录，默认当前目录")
    parser.add_argument("--output-dir", type=Path, default=Path("."), help="输出目录，默认当前目录")
    return parser.parse_args()


def load_log(path: Path) -> LogData:
    dx: list[float] = []
    dy: list[float] = []
    gx: list[float] = []
    gy: list[float] = []
    with path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError(f"{path.name} 缺少表头")
        for column in ("I1", "I2", "I4", "I5"):
            if column not in reader.fieldnames:
                raise ValueError(f"{path.name} 缺少列 {column}")
        for row_index, row in enumerate(reader, start=2):
            try:
                dx.append(float(row["I1"]))
                dy.append(float(row["I2"]))
                gx.append(float(row["I4"]))
                gy.append(float(row["I5"]))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path.name} 第 {row_index} 行存在非法浮点值") from exc
    return LogData(
        name=path.stem,
        delta_x=np.asarray(dx, dtype=np.float64),
        delta_y=np.asarray(dy, dtype=np.float64),
        gyro_x=np.asarray(gx, dtype=np.float64),
        gyro_y=np.asarray(gy, dtype=np.float64),
    )


def replay_decoupled(log: LogData) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    lpf_x = 0.0
    lpf_y = 0.0
    sum_x = 0.0
    sum_y = 0.0
    raw_x: list[float] = []
    raw_y: list[float] = []
    dec_x: list[float] = []
    dec_y: list[float] = []

    for idx in range(log.delta_x.size):
        lpf_x += GYRO_LPF_ALPHA * (float(log.gyro_x[idx]) - lpf_x)
        lpf_y += GYRO_LPF_ALPHA * (float(log.gyro_y[idx]) - lpf_y)
        sum_x += FIT_K * lpf_x * DT_S
        sum_y += FIT_K * lpf_y * DT_S

        if (idx % WINDOW_SAMPLES) == 0:
            dx = float(log.delta_x[idx])
            dy = float(log.delta_y[idx])
            raw_x.append(dx)
            raw_y.append(dy)
            dec_x.append(dx - sum_x)
            dec_y.append(dy - sum_y)
            sum_x = 0.0
            sum_y = 0.0

    return (
        np.asarray(raw_x, dtype=np.float64),
        np.asarray(raw_y, dtype=np.float64),
        np.asarray(dec_x, dtype=np.float64),
        np.asarray(dec_y, dtype=np.float64),
    )


def compute_fft(values: np.ndarray, sample_rate_hz: float) -> tuple[np.ndarray, np.ndarray]:
    centered = np.asarray(values, dtype=np.float64) - float(np.mean(values))
    if centered.size > FFT_POINTS_LIMIT:
        idx = np.linspace(0, centered.size - 1, FFT_POINTS_LIMIT).astype(np.int64)
        centered = centered[idx]
    window = np.hanning(centered.size)
    fft_values = np.fft.rfft(centered * window)
    freqs = np.fft.rfftfreq(centered.size, d=1.0 / sample_rate_hz)
    amp = np.abs(fft_values) / max(1.0, np.sum(window) * 0.5)
    return freqs, amp


def summarize_spectrum(freqs: np.ndarray, amp: np.ndarray) -> SpectrumSummary:
    power = amp * amp
    positive = freqs > 0.1
    if np.any(positive):
        peak_idx = int(np.argmax(power[positive]))
        peak_hz = float(freqs[positive][peak_idx])
        peak_amp = float(amp[positive][peak_idx])
    else:
        peak_hz = 0.0
        peak_amp = 0.0

    total_power = float(np.sum(power))
    if total_power <= 0.0:
        p95_hz = 0.0
    else:
        cumulative = np.cumsum(power)
        p95_idx = int(np.searchsorted(cumulative, total_power * 0.95))
        p95_idx = min(max(p95_idx, 0), len(freqs) - 1)
        p95_hz = float(freqs[p95_idx])

    def band_fraction(low: float, high: float) -> float:
        if total_power <= 0.0:
            return 0.0
        mask = (freqs >= low) & (freqs < high)
        return float(np.sum(power[mask]) / total_power)

    return SpectrumSummary(
        peak_hz=peak_hz,
        peak_amp=peak_amp,
        p95_hz=p95_hz,
        band_0_5=band_fraction(0.0, 5.0),
        band_5_10=band_fraction(5.0, 10.0),
        band_10_20=band_fraction(10.0, 20.0),
        band_20_25=band_fraction(20.0, 25.0),
    )


def analyze_log(log: LogData) -> LogResult:
    raw_x, raw_y, dec_x, dec_y = replay_decoupled(log)
    freqs, amp_dec_x = compute_fft(dec_x, FS_DEC_HZ)
    _, amp_dec_y = compute_fft(dec_y, FS_DEC_HZ)
    _, amp_raw_x = compute_fft(raw_x, FS_DEC_HZ)
    _, amp_raw_y = compute_fft(raw_y, FS_DEC_HZ)
    return LogResult(
        log=log,
        dec_x=dec_x,
        dec_y=dec_y,
        raw_x=raw_x,
        raw_y=raw_y,
        freqs=freqs,
        amp_dec_x=amp_dec_x,
        amp_dec_y=amp_dec_y,
        amp_raw_x=amp_raw_x,
        amp_raw_y=amp_raw_y,
        x_summary=summarize_spectrum(freqs, amp_dec_x),
        y_summary=summarize_spectrum(freqs, amp_dec_y),
    )


def polyline_points(x: np.ndarray, y: np.ndarray, left: float, top: float, width: float, height: float, x_min: float, x_max: float, y_min: float, y_max: float) -> str:
    x_span = max(1e-9, x_max - x_min)
    y_span = max(1e-9, y_max - y_min)
    points: list[str] = []
    for xv, yv in zip(x, y):
        px = left + (float(xv) - x_min) / x_span * width
        py = top + (y_max - float(yv)) / y_span * height
        points.append(f"{px:.2f},{py:.2f}")
    return " ".join(points)


def make_fft_svg(result: LogResult) -> str:
    width = 1180
    height = 720
    left = 70.0
    plot_width = 1050.0
    panel_height = 260.0
    x_min = 0.0
    x_max = min(25.0, float(np.max(result.freqs)))

    def clipped(values: np.ndarray) -> np.ndarray:
        mask = result.freqs <= x_max
        return values[mask]

    freqs = result.freqs[result.freqs <= x_max]
    amp_sets = [
        clipped(result.amp_raw_x),
        clipped(result.amp_dec_x),
        clipped(result.amp_raw_y),
        clipped(result.amp_dec_y),
    ]
    y_max = max(float(np.max(arr)) for arr in amp_sets if arr.size > 0)
    y_max = max(y_max, 0.1)

    top1 = 50.0
    top2 = 400.0

    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#fffdf8"/>
<text x="70" y="24" font-size="22" font-family="Arial" fill="#222">{html.escape(result.log.name)} offset=0 alpha=0.05 频域图</text>
<text x="70" y="44" font-size="12" font-family="Arial" fill="#555">采样频率 50Hz，对 dec_x / dec_y 与原始 raw_x / raw_y 分别做单边 FFT 幅值谱。</text>

<line x1="{left}" y1="{top1}" x2="{left}" y2="{top1 + panel_height}" stroke="#222" stroke-width="1.2"/>
<line x1="{left}" y1="{top1 + panel_height}" x2="{left + plot_width}" y2="{top1 + panel_height}" stroke="#222" stroke-width="1.2"/>
<polyline fill="none" stroke="#c35a42" stroke-width="1.8" points="{polyline_points(freqs, clipped(result.amp_raw_x), left, top1, plot_width, panel_height, x_min, x_max, 0.0, y_max)}"/>
<polyline fill="none" stroke="#1d78b5" stroke-width="1.8" points="{polyline_points(freqs, clipped(result.amp_dec_x), left, top1, plot_width, panel_height, x_min, x_max, 0.0, y_max)}"/>
<text x="70" y="{top1 - 10}" font-size="16" font-family="Arial" fill="#222">X轴：raw_x vs dec_x</text>

<line x1="{left}" y1="{top2}" x2="{left}" y2="{top2 + panel_height}" stroke="#222" stroke-width="1.2"/>
<line x1="{left}" y1="{top2 + panel_height}" x2="{left + plot_width}" y2="{top2 + panel_height}" stroke="#222" stroke-width="1.2"/>
<polyline fill="none" stroke="#d28b25" stroke-width="1.8" points="{polyline_points(freqs, clipped(result.amp_raw_y), left, top2, plot_width, panel_height, x_min, x_max, 0.0, y_max)}"/>
<polyline fill="none" stroke="#2e9b54" stroke-width="1.8" points="{polyline_points(freqs, clipped(result.amp_dec_y), left, top2, plot_width, panel_height, x_min, x_max, 0.0, y_max)}"/>
<text x="70" y="{top2 - 10}" font-size="16" font-family="Arial" fill="#222">Y轴：raw_y vs dec_y</text>

<rect x="840" y="68" width="260" height="90" fill="#fff" stroke="#ddd"/>
<line x1="860" y1="92" x2="900" y2="92" stroke="#c35a42" stroke-width="2"/><text x="910" y="96" font-size="13" font-family="Arial">raw_x</text>
<line x1="860" y1="116" x2="900" y2="116" stroke="#1d78b5" stroke-width="2"/><text x="910" y="120" font-size="13" font-family="Arial">dec_x</text>
<line x1="860" y1="140" x2="900" y2="140" stroke="#d28b25" stroke-width="2"/><text x="910" y="144" font-size="13" font-family="Arial">raw_y</text>
<line x1="860" y1="164" x2="900" y2="164" stroke="#2e9b54" stroke-width="2"/><text x="910" y="168" font-size="13" font-family="Arial">dec_y</text>

<text x="70" y="692" font-size="12" font-family="Arial" fill="#444">frequency / Hz</text>
<text x="16" y="210" font-size="12" font-family="Arial" fill="#444" transform="rotate(-90 16,210)">amplitude</text>
<text x="16" y="560" font-size="12" font-family="Arial" fill="#444" transform="rotate(-90 16,560)">amplitude</text>
</svg>"""


def write_summary_csv(path: Path, results: list[LogResult]) -> None:
    with path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "log_name",
                "axis",
                "peak_hz",
                "peak_amp",
                "p95_hz",
                "band_0_5",
                "band_5_10",
                "band_10_20",
                "band_20_25",
            ]
        )
        for result in results:
            for axis, summary in (("x", result.x_summary), ("y", result.y_summary)):
                writer.writerow(
                    [
                        result.log.name,
                        axis,
                        f"{summary.peak_hz:.6f}",
                        f"{summary.peak_amp:.6f}",
                        f"{summary.p95_hz:.6f}",
                        f"{summary.band_0_5:.6f}",
                        f"{summary.band_5_10:.6f}",
                        f"{summary.band_10_20:.6f}",
                        f"{summary.band_20_25:.6f}",
                    ]
                )


def write_html(path: Path, results: list[LogResult]) -> None:
    fc_hz = -FS_1000HZ * math.log(1.0 - GYRO_LPF_ALPHA) / (2.0 * math.pi)
    cards: list[str] = []
    for result in results:
        svg_name = f"{result.log.name}_offset0_alpha005_fft.svg"
        cards.append(
            f"""
<div class="card">
  <h2>{html.escape(result.log.name)}</h2>
  <ul>
    <li>X轴主峰：{result.x_summary.peak_hz:.2f}Hz，峰值幅度 {result.x_summary.peak_amp:.4f}</li>
    <li>X轴95%累计能量频率：{result.x_summary.p95_hz:.2f}Hz</li>
    <li>X轴能量分布：0~5Hz={result.x_summary.band_0_5*100:.1f}%，5~10Hz={result.x_summary.band_5_10*100:.1f}%，10~20Hz={result.x_summary.band_10_20*100:.1f}%，20~25Hz={result.x_summary.band_20_25*100:.1f}%</li>
    <li>Y轴主峰：{result.y_summary.peak_hz:.2f}Hz，峰值幅度 {result.y_summary.peak_amp:.4f}</li>
    <li>Y轴95%累计能量频率：{result.y_summary.p95_hz:.2f}Hz</li>
    <li>Y轴能量分布：0~5Hz={result.y_summary.band_0_5*100:.1f}%，5~10Hz={result.y_summary.band_5_10*100:.1f}%，10~20Hz={result.y_summary.band_10_20*100:.1f}%，20~25Hz={result.y_summary.band_20_25*100:.1f}%</li>
    <li><a href="{html.escape(svg_name)}">打开频谱图 {html.escape(svg_name)}</a></li>
  </ul>
</div>"""
        )

    html_text = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8"/>
  <title>offset=0 alpha=0.05 频域分析</title>
  <style>
    body {{ font-family: "Segoe UI", Arial, sans-serif; margin: 24px auto; max-width: 1240px; color: #222; background: #fbfaf7; }}
    .card {{ background: #fff; border: 1px solid #e1dbcf; padding: 18px 20px; margin-bottom: 20px; box-shadow: 0 2px 8px rgba(0,0,0,0.04); }}
    code {{ background: #f1ede5; padding: 1px 4px; }}
  </style>
</head>
<body>
  <h1>offset=0, alpha=0.05 的 dec_x / dec_y 频域分析</h1>
  <div class="card">
    <p>本分析固定 <code>offset=0</code>，固定 <code>FLOW_GYRO_LPF_ALPHA = {GYRO_LPF_ALPHA:.2f}</code>。它对应的一阶低通等效截止频率约为 <code>{fc_hz:.2f}Hz</code>。</p>
    <p>对最终解耦后的 <code>s_dec_x / s_dec_y</code> 做 50Hz 采样序列 FFT，并与原始 <code>deltaX / deltaY</code> 的频谱对比。</p>
  </div>
  {''.join(cards)}
</body>
</html>"""
    path.write_text(html_text, encoding="utf-8")


def main() -> None:
    args = parse_args()
    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    csv_paths = sorted(input_dir.glob("*_pmwgyro.csv"))
    if not csv_paths:
        raise SystemExit("没有找到 *_pmwgyro.csv")

    results = [analyze_log(load_log(path)) for path in csv_paths]
    for result in results:
        svg_name = f"{result.log.name}_offset0_alpha005_fft.svg"
        (output_dir / svg_name).write_text(make_fft_svg(result), encoding="utf-8")

    write_summary_csv(output_dir / "offset0_alpha005_fft_summary.csv", results)
    write_html(output_dir / "offset0_alpha005_fft_report.html", results)

    for result in results:
        print(
            "{}: x_peak={:.2f}Hz y_peak={:.2f}Hz x_p95={:.2f}Hz y_p95={:.2f}Hz".format(
                result.log.name,
                result.x_summary.peak_hz,
                result.y_summary.peak_hz,
                result.x_summary.p95_hz,
                result.y_summary.p95_hz,
            )
        )


if __name__ == "__main__":
    main()
