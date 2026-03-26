#!/usr/bin/env python3
"""固定 offset=0 时扫一阶低通截止频率。"""

from __future__ import annotations

import argparse
import csv
import html
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

FS_HZ = 1000.0
DT_S = 0.001
FIT_K = 10.0
WINDOW_SAMPLES = 20
CURRENT_ALPHA = 0.089955
CURRENT_FC_HZ = -FS_HZ * math.log(1.0 - CURRENT_ALPHA) / (2.0 * math.pi)
SWEEP_START_HZ = 1.0
SWEEP_STOP_HZ = 20.0
SWEEP_STEP_HZ = 0.25


@dataclass(frozen=True)
class LogData:
    name: str
    delta_x: np.ndarray
    delta_y: np.ndarray
    gyro_x: np.ndarray
    gyro_y: np.ndarray


@dataclass(frozen=True)
class SweepPoint:
    cutoff_hz: float
    alpha: float
    total_score: float
    x_rms_sum: float
    y_rms_sum: float
    per_log: list[tuple[str, float, float]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="固定 offset=0 扫 PMW/gyro 解耦低通截止频率")
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


def alpha_from_cutoff(cutoff_hz: float) -> float:
    return 1.0 - math.exp(-2.0 * math.pi * cutoff_hz / FS_HZ)


def replay_offset0(log: LogData, alpha: float) -> tuple[float, float]:
    lpf_x = 0.0
    lpf_y = 0.0
    sum_x = 0.0
    sum_y = 0.0
    dec_x: list[float] = []
    dec_y: list[float] = []

    for idx in range(log.delta_x.size):
        lpf_x += alpha * (float(log.gyro_x[idx]) - lpf_x)
        lpf_y += alpha * (float(log.gyro_y[idx]) - lpf_y)
        sum_x += FIT_K * lpf_x * DT_S
        sum_y += FIT_K * lpf_y * DT_S

        if (idx % WINDOW_SAMPLES) == 0:
            dec_x.append(float(log.delta_x[idx]) - sum_x)
            dec_y.append(float(log.delta_y[idx]) - sum_y)
            sum_x = 0.0
            sum_y = 0.0

    dec_x_arr = np.asarray(dec_x, dtype=np.float64)
    dec_y_arr = np.asarray(dec_y, dtype=np.float64)
    rms_x = float(np.sqrt(np.mean(dec_x_arr * dec_x_arr)))
    rms_y = float(np.sqrt(np.mean(dec_y_arr * dec_y_arr)))
    return rms_x, rms_y


def sweep_logs(logs: list[LogData]) -> list[SweepPoint]:
    points: list[SweepPoint] = []
    cutoff_values = np.arange(SWEEP_START_HZ, SWEEP_STOP_HZ + 0.0001, SWEEP_STEP_HZ)

    for cutoff_hz in cutoff_values:
        alpha = alpha_from_cutoff(float(cutoff_hz))
        total_score = 0.0
        x_rms_sum = 0.0
        y_rms_sum = 0.0
        per_log: list[tuple[str, float, float]] = []

        for log in logs:
            rms_x, rms_y = replay_offset0(log, alpha)
            per_log.append((log.name, rms_x, rms_y))
            total_score += rms_x + rms_y
            x_rms_sum += rms_x
            y_rms_sum += rms_y

        points.append(
            SweepPoint(
                cutoff_hz=float(cutoff_hz),
                alpha=alpha,
                total_score=total_score,
                x_rms_sum=x_rms_sum,
                y_rms_sum=y_rms_sum,
                per_log=per_log,
            )
        )

    return points


def best_point(points: list[SweepPoint]) -> SweepPoint:
    return min(points, key=lambda item: item.total_score)


def current_point(points: list[SweepPoint]) -> SweepPoint:
    return min(points, key=lambda item: abs(item.cutoff_hz - CURRENT_FC_HZ))


def polyline_points(x: np.ndarray, y: np.ndarray, width: int, height: int, left: float, top: float, x_min: float, x_max: float, y_min: float, y_max: float) -> str:
    x_span = max(1e-9, x_max - x_min)
    y_span = max(1e-9, y_max - y_min)
    points: list[str] = []
    for xv, yv in zip(x, y):
        px = left + (float(xv) - x_min) / x_span * width
        py = top + (y_max - float(yv)) / y_span * height
        points.append(f"{px:.2f},{py:.2f}")
    return " ".join(points)


def make_sweep_svg(points: list[SweepPoint], best: SweepPoint, current: SweepPoint) -> str:
    width = 1180
    height = 620
    left = 70.0
    top = 40.0
    plot_width = 1070.0
    plot_height = 500.0
    x = np.asarray([item.cutoff_hz for item in points], dtype=np.float64)
    y_total = np.asarray([item.total_score for item in points], dtype=np.float64)
    y_x = np.asarray([item.x_rms_sum for item in points], dtype=np.float64)
    y_y = np.asarray([item.y_rms_sum for item in points], dtype=np.float64)
    x_min = float(np.min(x))
    x_max = float(np.max(x))
    y_min = float(min(np.min(y_total), np.min(y_x), np.min(y_y)))
    y_max = float(max(np.max(y_total), np.max(y_x), np.max(y_y)))
    best_x = left + (best.cutoff_hz - x_min) / (x_max - x_min) * plot_width
    cur_x = left + (current.cutoff_hz - x_min) / (x_max - x_min) * plot_width

    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#fffdf8"/>
<text x="70" y="24" font-size="22" font-family="Arial" fill="#222">offset=0 固定时的截止频率扫参</text>
<text x="70" y="44" font-size="12" font-family="Arial" fill="#555">目标函数 = 两份日志的 dec_x_rms + dec_y_rms 总和，越小越好。</text>
<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_height}" stroke="#222" stroke-width="1.2"/>
<line x1="{left}" y1="{top + plot_height}" x2="{left + plot_width}" y2="{top + plot_height}" stroke="#222" stroke-width="1.2"/>
<line x1="{best_x:.2f}" y1="{top}" x2="{best_x:.2f}" y2="{top + plot_height}" stroke="#1b6f34" stroke-width="1.2" stroke-dasharray="7 5"/>
<line x1="{cur_x:.2f}" y1="{top}" x2="{cur_x:.2f}" y2="{top + plot_height}" stroke="#b33a3a" stroke-width="1.2" stroke-dasharray="7 5"/>
<polyline fill="none" stroke="#2f6db3" stroke-width="2.2" points="{polyline_points(x, y_total, plot_width, plot_height, left, top, x_min, x_max, y_min, y_max)}"/>
<polyline fill="none" stroke="#2e9b54" stroke-width="1.8" points="{polyline_points(x, y_x, plot_width, plot_height, left, top, x_min, x_max, y_min, y_max)}"/>
<polyline fill="none" stroke="#d28b25" stroke-width="1.8" points="{polyline_points(x, y_y, plot_width, plot_height, left, top, x_min, x_max, y_min, y_max)}"/>
<circle cx="{best_x:.2f}" cy="{top + (y_max - best.total_score) / max(1e-9, y_max - y_min) * plot_height:.2f}" r="5" fill="#1b6f34"/>
<circle cx="{cur_x:.2f}" cy="{top + (y_max - current.total_score) / max(1e-9, y_max - y_min) * plot_height:.2f}" r="5" fill="#b33a3a"/>
<rect x="780" y="64" width="320" height="118" fill="#fff" stroke="#ddd"/>
<line x1="802" y1="92" x2="840" y2="92" stroke="#2f6db3" stroke-width="2.2"/><text x="850" y="96" font-size="13" font-family="Arial">total score</text>
<line x1="802" y1="118" x2="840" y2="118" stroke="#2e9b54" stroke-width="1.8"/><text x="850" y="122" font-size="13" font-family="Arial">sum(dec_x_rms)</text>
<line x1="802" y1="144" x2="840" y2="144" stroke="#d28b25" stroke-width="1.8"/><text x="850" y="148" font-size="13" font-family="Arial">sum(dec_y_rms)</text>
<text x="70" y="592" font-size="12" font-family="Arial" fill="#444">cutoff / Hz</text>
<text x="18" y="310" font-size="12" font-family="Arial" fill="#444" transform="rotate(-90 18,310)">score / rms sum</text>
<text x="780" y="204" font-size="13" font-family="Arial" fill="#1b6f34">最佳: {best.cutoff_hz:.2f}Hz, alpha={best.alpha:.6f}, score={best.total_score:.4f}</text>
<text x="780" y="226" font-size="13" font-family="Arial" fill="#b33a3a">当前: {CURRENT_FC_HZ:.2f}Hz, alpha={CURRENT_ALPHA:.6f}, score={current.total_score:.4f}</text>
</svg>"""


def write_csv(path: Path, points: list[SweepPoint]) -> None:
    with path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        header = ["cutoff_hz", "alpha", "total_score", "x_rms_sum", "y_rms_sum"]
        log_names = [name for name, _, _ in points[0].per_log]
        for log_name in log_names:
            header.extend([f"{log_name}_dec_x_rms", f"{log_name}_dec_y_rms"])
        writer.writerow(header)
        for point in points:
            row = [
                f"{point.cutoff_hz:.2f}",
                f"{point.alpha:.6f}",
                f"{point.total_score:.6f}",
                f"{point.x_rms_sum:.6f}",
                f"{point.y_rms_sum:.6f}",
            ]
            for _, rms_x, rms_y in point.per_log:
                row.extend([f"{rms_x:.6f}", f"{rms_y:.6f}"])
            writer.writerow(row)


def write_html(path: Path, points: list[SweepPoint], best: SweepPoint, current: SweepPoint) -> None:
    improve = 1.0 - best.total_score / max(1e-9, current.total_score)
    rows = []
    for name, rms_x, rms_y in best.per_log:
        rows.append(f"<li>{html.escape(name)}: dec_x_rms={rms_x:.4f}, dec_y_rms={rms_y:.4f}</li>")

    html_text = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8"/>
  <title>offset=0 低通扫参报告</title>
  <style>
    body {{ font-family: "Segoe UI", Arial, sans-serif; margin: 24px auto; max-width: 1240px; background: #fbfaf7; color: #222; }}
    .card {{ background: #fff; border: 1px solid #e1dbcf; padding: 18px 20px; margin-bottom: 20px; box-shadow: 0 2px 8px rgba(0,0,0,0.04); }}
    code {{ background: #f1ede5; padding: 1px 4px; }}
  </style>
</head>
<body>
  <h1>offset=0 固定的 LPF 截止频率离线扫参</h1>
  <div class="card">
    <p>本报告强制 <code>offset=0</code>，只调 <code>FlowGyroDecoupler_Push1000Hz</code> 里的陀螺一阶低通截止频率。拟合公式保持不变：<code>sum += 10 * gyro_lpf * 0.001</code>。</p>
    <p>当前固件参数：<code>FLOW_GYRO_LPF_ALPHA = {CURRENT_ALPHA:.6f}</code>，等效截止频率约 <code>{CURRENT_FC_HZ:.2f}Hz</code>。</p>
    <p>离线最优结果：<strong>{best.cutoff_hz:.2f}Hz</strong>，对应 <strong>alpha={best.alpha:.6f}</strong>。相对当前 15Hz，总 score 下降 <strong>{improve * 100.0:.1f}%</strong>。</p>
    <ul>
      {''.join(rows)}
    </ul>
  </div>
  <div class="card">{make_sweep_svg(points, best, current)}</div>
</body>
</html>"""
    path.write_text(html_text, encoding="utf-8")


def save_svg(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8")


def main() -> None:
    args = parse_args()
    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    csv_paths = sorted(input_dir.glob("*_pmwgyro.csv"))
    if not csv_paths:
        raise SystemExit("没有找到 *_pmwgyro.csv")

    logs = [load_log(path) for path in csv_paths]
    points = sweep_logs(logs)
    best = best_point(points)
    current = current_point(points)

    csv_path = output_dir / "offset0_lpf_sweep.csv"
    svg_path = output_dir / "offset0_lpf_sweep.svg"
    html_path = output_dir / "offset0_lpf_sweep_report.html"

    write_csv(csv_path, points)
    save_svg(svg_path, make_sweep_svg(points, best, current))
    write_html(html_path, points, best, current)

    print(
        "best_cutoff_hz={:.2f} alpha={:.6f} current_cutoff_hz={:.2f} current_alpha={:.6f}".format(
            best.cutoff_hz, best.alpha, CURRENT_FC_HZ, CURRENT_ALPHA
        )
    )


if __name__ == "__main__":
    main()
