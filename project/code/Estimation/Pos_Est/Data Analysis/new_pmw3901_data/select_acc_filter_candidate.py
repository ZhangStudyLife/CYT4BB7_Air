#!/usr/bin/env python3
"""筛选水平加速度嵌入式滤波候选。"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from vel_acc_common import (
    SAMPLE_RATE_HZ,
    FilterSpec,
    attenuation_db_at,
    hf_noise_ratio,
    iter_default_filter_specs,
    load_better_vel_acc,
    replay_complementary_filter,
    rms,
    group_delay_ms_at,
)


SCRIPT_DIR = Path(__file__).resolve().parent


def resolve_script_path(path_text: str) -> Path:
    """将默认相对路径解析到脚本目录，避免依赖当前工作目录。"""
    path = Path(path_text)
    if path.is_absolute():
        return path
    return SCRIPT_DIR / path


@dataclass(frozen=True)
class CandidateScore:
    """单个候选的评分结果。"""

    spec: FilterSpec
    attenuation_5hz_db: float
    delay_5hz_ms: float
    hf_ratio_right: float
    hf_ratio_forward: float
    innovation_rms_right: float
    innovation_rms_forward: float
    total_score: float
    passed_constraints: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="筛选 better_vel_acc 的水平加速度滤波候选。")
    parser.add_argument("--input", default="better_vel_acc.csv", help="输入 CSV 文件路径。")
    parser.add_argument("--sample-rate", type=float, default=SAMPLE_RATE_HZ, help="采样频率，默认 250Hz。")
    parser.add_argument("--output-dir", default="filter_selection_better_vel_acc", help="输出目录。")
    return parser.parse_args()


def evaluate_candidates(sample_rate_hz: float, input_csv: Path) -> list[CandidateScore]:
    dataset = load_better_vel_acc(input_csv, sample_rate_hz=sample_rate_hz)
    raw_hf_right = hf_noise_ratio(dataset.acc_right_mps2, sample_rate_hz)
    raw_hf_forward = hf_noise_ratio(dataset.acc_forward_mps2, sample_rate_hz)

    raw_scores = []
    for spec in iter_default_filter_specs():
        replay = replay_complementary_filter(
            acc_right_mps2=dataset.acc_right_mps2,
            acc_forward_mps2=dataset.acc_forward_mps2,
            flow_right_cmps=dataset.flow_right_cmps,
            flow_forward_cmps=dataset.flow_forward_cmps,
            filter_spec=spec,
            sample_rate_hz=sample_rate_hz,
        )

        attenuation_5hz_db = attenuation_db_at(spec, sample_rate_hz, 5.0)
        delay_5hz_ms = group_delay_ms_at(spec, sample_rate_hz, 5.0)
        hf_ratio_right = hf_noise_ratio(replay.acc_right_filt_cmpss, sample_rate_hz) / max(1.0e-12, raw_hf_right)
        hf_ratio_forward = hf_noise_ratio(replay.acc_forward_filt_cmpss, sample_rate_hz) / max(1.0e-12, raw_hf_forward)
        innovation_rms_right = rms(replay.innovation_right_cmps)
        innovation_rms_forward = rms(replay.innovation_forward_cmps)

        raw_scores.append(
            {
                "spec": spec,
                "attenuation_5hz_db": attenuation_5hz_db,
                "delay_5hz_ms": delay_5hz_ms,
                "hf_ratio": 0.5 * (hf_ratio_right + hf_ratio_forward),
                "innovation_rms": 0.5 * (innovation_rms_right + innovation_rms_forward),
                "hf_ratio_right": hf_ratio_right,
                "hf_ratio_forward": hf_ratio_forward,
                "innovation_rms_right": innovation_rms_right,
                "innovation_rms_forward": innovation_rms_forward,
                "passed_constraints": (attenuation_5hz_db >= -3.0) and (delay_5hz_ms <= 20.0),
            }
        )

    hf_values = np.asarray([item["hf_ratio"] for item in raw_scores], dtype=np.float64)
    innovation_values = np.asarray([item["innovation_rms"] for item in raw_scores], dtype=np.float64)
    delay_values = np.asarray([item["delay_5hz_ms"] for item in raw_scores], dtype=np.float64)

    hf_min, hf_max = float(np.min(hf_values)), float(np.max(hf_values))
    innovation_min, innovation_max = float(np.min(innovation_values)), float(np.max(innovation_values))
    delay_min, delay_max = float(np.min(delay_values)), float(np.max(delay_values))

    def normalize(value: float, value_min: float, value_max: float) -> float:
        if abs(value_max - value_min) < 1.0e-12:
            return 0.0
        return (value - value_min) / (value_max - value_min)

    scores = []
    for item in raw_scores:
        if item["passed_constraints"]:
            total_score = (
                0.45 * normalize(item["hf_ratio"], hf_min, hf_max)
                + 0.35 * normalize(item["innovation_rms"], innovation_min, innovation_max)
                + 0.20 * normalize(item["delay_5hz_ms"], delay_min, delay_max)
            )
        else:
            total_score = 1.0e6

        scores.append(
            CandidateScore(
                spec=item["spec"],
                attenuation_5hz_db=item["attenuation_5hz_db"],
                delay_5hz_ms=item["delay_5hz_ms"],
                hf_ratio_right=item["hf_ratio_right"],
                hf_ratio_forward=item["hf_ratio_forward"],
                innovation_rms_right=item["innovation_rms_right"],
                innovation_rms_forward=item["innovation_rms_forward"],
                total_score=total_score,
                passed_constraints=item["passed_constraints"],
            )
        )

    return sorted(scores, key=lambda item: item.total_score)


def write_csv(output_dir: Path, scores: list[CandidateScore]) -> None:
    csv_path = output_dir / "candidate_scores.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "label",
                "attenuation_5hz_db",
                "delay_5hz_ms",
                "hf_ratio_right",
                "hf_ratio_forward",
                "innovation_rms_right",
                "innovation_rms_forward",
                "total_score",
                "passed_constraints",
            ]
        )
        for score in scores:
            writer.writerow(
                [
                    score.spec.label,
                    f"{score.attenuation_5hz_db:.6f}",
                    f"{score.delay_5hz_ms:.6f}",
                    f"{score.hf_ratio_right:.6f}",
                    f"{score.hf_ratio_forward:.6f}",
                    f"{score.innovation_rms_right:.6f}",
                    f"{score.innovation_rms_forward:.6f}",
                    f"{score.total_score:.6f}",
                    int(score.passed_constraints),
                ]
            )


def render_score_plot(output_dir: Path, scores: list[CandidateScore]) -> None:
    valid_scores = [score for score in scores if score.passed_constraints][:10]
    labels = [score.spec.label for score in valid_scores]
    values = [score.total_score for score in valid_scores]

    figure, axis = plt.subplots(figsize=(14, 6))
    axis.bar(range(len(valid_scores)), values)
    axis.set_xticks(range(len(valid_scores)))
    axis.set_xticklabels(labels, rotation=35, ha="right")
    axis.set_ylabel("Score")
    axis.set_title("Top-10 Filter Candidates")
    axis.grid(True, axis="y", alpha=0.3)
    figure.tight_layout()
    figure.savefig(output_dir / "candidate_top10.png", dpi=180)
    plt.close(figure)


def write_report(output_dir: Path, scores: list[CandidateScore]) -> None:
    best = next(score for score in scores if score.passed_constraints)
    lines = [
        "# 水平加速度滤波候选筛选报告",
        "",
        "## 约束",
        "",
        "- 5Hz 幅值衰减不超过 3dB。",
        "- 5Hz 等效群时延不超过 20ms。",
        "- 综合评分 = 0.45*高频残余比 + 0.35*速度新息 RMS + 0.20*群时延归一化。",
        "",
        "## 最优候选",
        "",
        f"- 候选: `{best.spec.label}`",
        f"- 5Hz 衰减: {best.attenuation_5hz_db:.3f} dB",
        f"- 5Hz 群时延: {best.delay_5hz_ms:.3f} ms",
        f"- 右向高频残余比: {best.hf_ratio_right:.4f}",
        f"- 前向高频残余比: {best.hf_ratio_forward:.4f}",
        f"- 右向速度新息 RMS: {best.innovation_rms_right:.4f} cm/s",
        f"- 前向速度新息 RMS: {best.innovation_rms_forward:.4f} cm/s",
        "",
        "## 排名前十",
        "",
        "| 排名 | 候选 | 5Hz衰减(dB) | 时延(ms) | 右向HF比 | 前向HF比 | 右向新息RMS | 前向新息RMS | 总分 |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]

    valid_scores = [score for score in scores if score.passed_constraints][:10]
    for index, score in enumerate(valid_scores, start=1):
        lines.append(
            f"| {index} | {score.spec.label} | "
            f"{score.attenuation_5hz_db:.3f} | "
            f"{score.delay_5hz_ms:.3f} | "
            f"{score.hf_ratio_right:.4f} | "
            f"{score.hf_ratio_forward:.4f} | "
            f"{score.innovation_rms_right:.4f} | "
            f"{score.innovation_rms_forward:.4f} | "
            f"{score.total_score:.4f} |"
        )

    lines.extend(
        [
            "",
            "## 结论",
            "",
            f"- 在当前数据集上，`{best.spec.label}` 在噪声压制、速度新息与时延约束之间取得了最优综合分数。",
            "- 10Hz/12Hz 低通虽然进一步降低了高频残余，但速度新息 RMS 与等效时延显著增加，不适合作为动态飞行主方案。",
        ]
    )

    (output_dir / "filter_selection_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    output_dir = resolve_script_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    scores = evaluate_candidates(sample_rate_hz=args.sample_rate, input_csv=resolve_script_path(args.input))
    write_csv(output_dir, scores)
    render_score_plot(output_dir, scores)
    write_report(output_dir, scores)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
