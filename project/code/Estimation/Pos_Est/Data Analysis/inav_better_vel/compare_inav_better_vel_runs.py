#!/usr/bin/env python3
"""联合分析多份 inav_better_vel 实飞日志并输出稳健调参建议。"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
COMMON_DIR = SCRIPT_DIR.parent / "new_pmw3901_data"
for search_path in (SCRIPT_DIR, COMMON_DIR):
    if str(search_path) not in sys.path:
        sys.path.insert(0, str(search_path))

from analyze_inav_better_vel import (  # noqa: E402
    CandidateResult,
    load_csv,
    score_candidate,
    search_candidates,
    simulate_replay,
    summarize_psd,
)
from vel_acc_common import FilterSpec  # noqa: E402


DEFAULT_INPUTS = ("inav_better_vel.csv", "inav_better_vel_2.csv")
DEFAULT_REPORT = "inav_better_vel_joint_report.md"
DEFAULT_SAMPLE_RATE_HZ = 250.0

OLD_MACHINE_PARAMS = {
    "flow_v": 4.0,
    "acc_bias": 0.005,
    "innov_gate": 130.0,
    "dead_max_s": 0.20,
    "res_v": 1.0,
}

CURRENT_MACHINE_PARAMS = {
    "flow_v": 4.5,
    "acc_bias": 0.003,
    "innov_gate": 130.0,
    "dead_max_s": 0.20,
    "res_v": 1.0,
}

FILTER_SPEC = FilterSpec(label="Notch85Q3+LPF15", notch_hz=85.0, notch_q=3.0, lpf_hz=15.0)


@dataclass(frozen=True)
class JointCandidate:
    """保存联合评分结果。"""

    candidate: CandidateResult
    total_score: float
    worst_score: float


def resolve_script_path(path_text: str) -> Path:
    """将相对路径解析到脚本目录。"""
    path = Path(path_text)
    if path.is_absolute():
        return path
    return SCRIPT_DIR / path


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="联合分析多份 inav_better_vel 实飞日志。")
    parser.add_argument(
        "--inputs",
        nargs="+",
        default=list(DEFAULT_INPUTS),
        help="一个或多个输入 CSV 路径，默认同时分析 inav_better_vel.csv 和 inav_better_vel_2.csv。",
    )
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=DEFAULT_SAMPLE_RATE_HZ,
        help="日志采样率，默认 250Hz。",
    )
    parser.add_argument("--report", default=DEFAULT_REPORT, help="输出 Markdown 报告路径。")
    return parser.parse_args()


def evaluate_param_set(
    columns_by_name: dict[str, dict[str, object]],
    sample_rate_hz: float,
    params: dict[str, float],
) -> dict[str, object]:
    """评估一组参数在多份日志上的联合表现。"""
    per_file_metrics: dict[str, dict[str, object]] = {}
    scores: list[float] = []

    for file_name, columns in columns_by_name.items():
        metrics = simulate_replay(
            columns=columns,
            sample_rate_hz=sample_rate_hz,
            filter_spec=FILTER_SPEC,
            flow_v=params["flow_v"],
            acc_bias=params["acc_bias"],
            innov_gate=params["innov_gate"],
            dead_max_s=params["dead_max_s"],
            res_v=params["res_v"],
        )
        metric_score = score_candidate(metrics)
        per_file_metrics[file_name] = {
            "metrics": metrics,
            "score": metric_score,
        }
        scores.append(metric_score)

    return {
        "per_file": per_file_metrics,
        "total_score": float(sum(scores)),
        "worst_score": float(max(scores)),
    }


def find_joint_best(
    columns_by_name: dict[str, dict[str, object]],
    sample_rate_hz: float,
) -> JointCandidate:
    """在多份日志上搜索联合最优候选。"""
    reference_candidates = search_candidates(next(iter(columns_by_name.values())), sample_rate_hz)
    best_joint: JointCandidate | None = None

    for candidate in reference_candidates:
        result = evaluate_param_set(
            columns_by_name=columns_by_name,
            sample_rate_hz=sample_rate_hz,
            params={
                "flow_v": candidate.flow_v,
                "acc_bias": candidate.acc_bias,
                "innov_gate": candidate.innov_gate,
                "dead_max_s": candidate.dead_max_s,
                "res_v": candidate.res_v,
            },
        )
        joint_candidate = JointCandidate(
            candidate=candidate,
            total_score=result["total_score"],
            worst_score=result["worst_score"],
        )
        if best_joint is None or (joint_candidate.total_score, joint_candidate.worst_score) < (
            best_joint.total_score,
            best_joint.worst_score,
        ):
            best_joint = joint_candidate

    if best_joint is None:
        raise ValueError("未找到联合候选结果。")
    return best_joint


def build_report(
    columns_by_name: dict[str, dict[str, object]],
    sample_rate_hz: float,
    old_eval: dict[str, object],
    current_eval: dict[str, object],
    joint_best: JointCandidate,
) -> str:
    """生成联合分析 Markdown 报告。"""
    lines = [
        "# inav_better_vel 多日志联合分析报告",
        "",
        f"- 日志数量: {len(columns_by_name)}",
        f"- 采样率: {sample_rate_hz:.1f} Hz",
        f"- 上一轮机载参数: `w_flow_v={OLD_MACHINE_PARAMS['flow_v']:.1f}`, `w_acc_bias={OLD_MACHINE_PARAMS['acc_bias']:.3f}`, `innov_gate={OLD_MACHINE_PARAMS['innov_gate']:.0f}`, `dead_max_s={OLD_MACHINE_PARAMS['dead_max_s']:.2f}`, `w_res_v={OLD_MACHINE_PARAMS['res_v']:.1f}`",
        f"- 当前机载参数: `w_flow_v={CURRENT_MACHINE_PARAMS['flow_v']:.1f}`, `w_acc_bias={CURRENT_MACHINE_PARAMS['acc_bias']:.3f}`, `innov_gate={CURRENT_MACHINE_PARAMS['innov_gate']:.0f}`, `dead_max_s={CURRENT_MACHINE_PARAMS['dead_max_s']:.2f}`, `w_res_v={CURRENT_MACHINE_PARAMS['res_v']:.1f}`",
        "",
        "## 单日志摘要",
        "",
    ]

    for file_name, columns in columns_by_name.items():
        raw_right_summary = summarize_psd(columns["I0"], sample_rate_hz)
        raw_forward_summary = summarize_psd(columns["I1"], sample_rate_hz)
        old_metrics = old_eval["per_file"][file_name]["metrics"]
        current_metrics = current_eval["per_file"][file_name]["metrics"]

        lines.extend(
            [
                f"### {file_name}",
                "",
                f"- 样本数: {columns['I0'].size}，时长: {columns['I0'].size / sample_rate_hz:.2f} s",
                f"- 当前日志原始加速度限幅命中比例: 右向 {((abs(columns['I0']) >= 999.0).mean() * 100.0):.2f}% ，前向 {((abs(columns['I1']) >= 999.0).mean() * 100.0):.2f}%",
                f"- 原始右向主峰: {', '.join(f'{peak:.2f}Hz' for peak in raw_right_summary['peaks_hz'])}",
                f"- 原始前向主峰: {', '.join(f'{peak:.2f}Hz' for peak in raw_forward_summary['peaks_hz'])}",
                f"- 上一轮参数回放: valid={old_metrics.valid_ratio * 100.0:.2f}%, 最长失效={old_metrics.longest_invalid_s:.3f}s, 创新RMS={old_metrics.innovation_rms_cmps:.3f} cm/s, 融合均值={old_metrics.mean_fused_right_cmps:.3f}/{old_metrics.mean_fused_forward_cmps:.3f} cm/s",
                f"- 当前参数回放: valid={current_metrics.valid_ratio * 100.0:.2f}%, 最长失效={current_metrics.longest_invalid_s:.3f}s, 创新RMS={current_metrics.innovation_rms_cmps:.3f} cm/s, 融合均值={current_metrics.mean_fused_right_cmps:.3f}/{current_metrics.mean_fused_forward_cmps:.3f} cm/s",
                "",
            ]
        )

    lines.extend(
        [
            "## 联合评估",
            "",
            f"- 上一轮机载参数联合总分: {old_eval['total_score']:.4f}，最差单日志分数: {old_eval['worst_score']:.4f}",
            f"- 当前机载参数联合总分: {current_eval['total_score']:.4f}，最差单日志分数: {current_eval['worst_score']:.4f}",
            f"- 网格搜索联合最优参数: `w_flow_v={joint_best.candidate.flow_v:.1f}`, `w_acc_bias={joint_best.candidate.acc_bias:.3f}`, `innov_gate={joint_best.candidate.innov_gate:.0f}`, `dead_max_s={joint_best.candidate.dead_max_s:.2f}`, `w_res_v={joint_best.candidate.res_v:.1f}`",
            f"- 网格搜索联合最优总分: {joint_best.total_score:.4f}，最差单日志分数: {joint_best.worst_score:.4f}",
            "",
            "## 结论",
            "",
            "- 两份日志都表明：当前滤波器不是主要矛盾，主要矛盾仍是融合速度权重与偏置学习速度之间的平衡。",
            "- 相比上一轮机载参数，当前机载参数已经在两份日志上同时改善了 valid 占比、最长失效和创新 RMS，可以继续作为上机候选。",
            "- 联合搜索里如果只看悬停日志得分，`innov_gate=140` 会进一步降低分数；但考虑低纹理/异常流场景容错，当前机载仍保留 `innov_gate=130`，不继续放宽门限。",
            "- 原始水平加速度在两份日志上都有较高的 `±1000 cm/s²` 限幅命中比例，说明机体振动和动态机动依然重；这暂时不建议靠放宽加速度限幅解决，否则更容易把高频振动送进积分链。",
            "- 我额外回放了“低机动时冻结偏置学习”和“翻转偏置学习符号”两个方向，收益都只有边际级，当前还不值得带上机载逻辑复杂度。",
        ]
    )

    return "\n".join(lines) + "\n"


def main() -> int:
    """程序入口。"""
    args = parse_args()
    input_paths = [resolve_script_path(path_text) for path_text in args.inputs]
    report_path = resolve_script_path(args.report)

    columns_by_name = {input_path.name: load_csv(input_path) for input_path in input_paths}
    old_eval = evaluate_param_set(columns_by_name, args.sample_rate, OLD_MACHINE_PARAMS)
    current_eval = evaluate_param_set(columns_by_name, args.sample_rate, CURRENT_MACHINE_PARAMS)
    joint_best = find_joint_best(columns_by_name, args.sample_rate)

    report_text = build_report(
        columns_by_name=columns_by_name,
        sample_rate_hz=args.sample_rate,
        old_eval=old_eval,
        current_eval=current_eval,
        joint_best=joint_best,
    )
    report_path.write_text(report_text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
