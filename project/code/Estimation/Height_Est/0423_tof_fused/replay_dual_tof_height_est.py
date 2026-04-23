import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable


HEIGHT_EST_DT_S = 0.01
HEIGHT_EST_RESIDUAL_GATE_MM = 120.0
HEIGHT_EST_AB_ALPHA = 0.18
HEIGHT_EST_AB_BETA = 0.03
HEIGHT_EST_PREDICT_HOLD_CNT = 15
HEIGHT_EST_VEL_DECAY = 0.95
HEIGHT_EST_WEIGHT_EPS = 0.001
HEIGHT_EST_INVALID_MM = 1300.0


@dataclass
class ReplayState:
    height_mm: float = HEIGHT_EST_INVALID_MM
    vz_mps: float = 0.0
    ready: int = 0
    hold_cnt: int = 0


def clamp_height(height_mm: float) -> float:
    if height_mm < 0.0:
        return 0.0
    if height_mm > HEIGHT_EST_INVALID_MM:
        return HEIGHT_EST_INVALID_MM
    return height_mm


def residual_weight(valid: int, residual_mm: float) -> float:
    if not valid:
        return 0.0
    abs_residual = abs(residual_mm)
    if abs_residual >= HEIGHT_EST_RESIDUAL_GATE_MM:
        return 0.0
    return 1.0 - abs_residual / HEIGHT_EST_RESIDUAL_GATE_MM


def split_sessions(rows: list[dict[str, float]]) -> list[list[dict[str, float]]]:
    if not rows:
        return []
    sessions: list[list[dict[str, float]]] = [[rows[0]]]
    for row in rows[1:]:
        if row["I0"] < sessions[-1][-1]["I0"]:
            sessions.append([row])
        else:
            sessions[-1].append(row)
    return sessions


def replay_row(state: ReplayState, row: dict[str, float]) -> dict[str, float]:
    tof1_height_mm = row["I1"]
    tof4_height_mm = row["I4"]
    tof1_valid = 1 if tof1_height_mm < HEIGHT_EST_INVALID_MM else 0
    tof4_valid = 1 if tof4_height_mm < HEIGHT_EST_INVALID_MM else 0

    h_pred_mm = HEIGHT_EST_INVALID_MM
    v_pred_mps = 0.0
    if state.ready:
        v_pred_mps = state.vz_mps
        h_pred_mm = clamp_height(state.height_mm + v_pred_mps * HEIGHT_EST_DT_S * 1000.0)

    weighted_sum = 0.0
    weight_sum = 0.0
    q1 = 0.0
    q4 = 0.0
    hard_relock = 0
    meas_valid = 0
    predict_only = 0
    z_meas_mm = h_pred_mm

    if state.ready:
        q1 = residual_weight(tof1_valid, tof1_height_mm - h_pred_mm)
        q4 = residual_weight(tof4_valid, tof4_height_mm - h_pred_mm)
        if q1 > 0.0:
            weighted_sum += q1 * tof1_height_mm
            weight_sum += q1
        if q4 > 0.0:
            weighted_sum += q4 * tof4_height_mm
            weight_sum += q4
    else:
        if tof1_valid:
            weighted_sum += tof1_height_mm
            weight_sum += 1.0
        if tof4_valid:
            weighted_sum += tof4_height_mm
            weight_sum += 1.0

    if (
        weight_sum <= HEIGHT_EST_WEIGHT_EPS
        and (tof1_valid or tof4_valid)
        and state.hold_cnt >= HEIGHT_EST_PREDICT_HOLD_CNT
    ):
        hard_relock = 1
        weighted_sum = 0.0
        weight_sum = 0.0
        if tof1_valid:
            weighted_sum += tof1_height_mm
            weight_sum += 1.0
        if tof4_valid:
            weighted_sum += tof4_height_mm
            weight_sum += 1.0

    if weight_sum > HEIGHT_EST_WEIGHT_EPS:
        z_meas_mm = weighted_sum / weight_sum
        meas_valid = 1

    replay_valid = 0
    if not state.ready:
        if meas_valid:
            state.height_mm = clamp_height(z_meas_mm)
            state.vz_mps = 0.0
            state.ready = 1
            state.hold_cnt = 0
            replay_valid = 1
    elif meas_valid:
        if hard_relock:
            state.height_mm = clamp_height(z_meas_mm)
            state.vz_mps = 0.0
        else:
            residual_mm = z_meas_mm - h_pred_mm
            state.height_mm = clamp_height(h_pred_mm + HEIGHT_EST_AB_ALPHA * residual_mm)
            state.vz_mps = v_pred_mps + (HEIGHT_EST_AB_BETA / HEIGHT_EST_DT_S) * (residual_mm * 0.001)
        state.hold_cnt = 0
        replay_valid = 1
    else:
        state.height_mm = clamp_height(h_pred_mm)
        state.vz_mps = v_pred_mps * HEIGHT_EST_VEL_DECAY
        if state.hold_cnt < HEIGHT_EST_PREDICT_HOLD_CNT:
            state.hold_cnt += 1
            replay_valid = 1
            predict_only = 1
        else:
            replay_valid = 0

    replay_height_mm = state.height_mm if state.ready else HEIGHT_EST_INVALID_MM
    replay_vz_mps = state.vz_mps if state.ready else 0.0

    return {
        "t_ms": row["I0"],
        "tof1_mm": tof1_height_mm,
        "tof2_mm": row["I2"],
        "tof3_mm": row["I3"],
        "tof4_mm": tof4_height_mm,
        "roll_deg": row["I5"],
        "pitch_deg": row["I6"],
        "acc_z_dyn_mps2": row["I7"],
        "log_vz_mps": row["I8"],
        "log_height_mm": row["I9"],
        "tof1_valid": tof1_valid,
        "tof4_valid": tof4_valid,
        "h_pred_mm": h_pred_mm,
        "q1": q1,
        "q4": q4,
        "weight_sum": weight_sum,
        "z_meas_mm": z_meas_mm,
        "meas_valid": meas_valid,
        "hard_relock": hard_relock,
        "predict_only": predict_only,
        "replay_valid": replay_valid,
        "replay_vz_mps": replay_vz_mps,
        "replay_height_mm": replay_height_mm,
        "abs_err_vz": abs(replay_vz_mps - row["I8"]),
        "abs_err_height": abs(replay_height_mm - row["I9"]),
        "hold_cnt": state.hold_cnt,
    }


def load_rows(csv_path: Path) -> tuple[list[dict[str, float]], int]:
    rows: list[dict[str, float]] = []
    dropped_rows = 0
    with csv_path.open("r", encoding="utf-8-sig", newline="") as fp:
        reader = csv.DictReader(fp)
        for raw in reader:
            row = {key: float(value) for key, value in raw.items()}
            if not all(math.isfinite(value) for value in row.values()):
                dropped_rows += 1
                continue
            rows.append(row)
    return rows, dropped_rows


def summarize(name: str, values: Iterable[float]) -> str:
    values = list(values)
    if not values:
        return f"{name}: n=0"
    mean_value = sum(values) / len(values)
    max_value = max(values)
    return f"{name}: n={len(values)}, mean={mean_value:.6f}, max={max_value:.6f}"


def write_csv(output_path: Path, rows: list[dict[str, float]]) -> None:
    if not rows:
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with output_path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay current Height_Est dual-TOF fusion offline.")
    parser.add_argument(
        "--csv",
        default="project/code/Estimation/Height_Est/0423_tof_fused/0423_tof_fused.csv",
        help="Path to the logged fused TOF CSV.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional output CSV for replayed intermediate variables. Leave empty to skip writing.",
    )
    args = parser.parse_args()

    rows, dropped_rows = load_rows(Path(args.csv))
    sessions = split_sessions(rows)
    print(f"rows={len(rows)} dropped_rows={dropped_rows} sessions={len(sessions)}")

    all_rows: list[dict[str, float]] = []
    for session_id, session_rows in enumerate(sessions):
        state = ReplayState()
        replayed: list[dict[str, float]] = []
        for row_idx, row in enumerate(session_rows):
            replay = replay_row(state, row)
            replay["session_id"] = session_id
            replay["row_in_session"] = row_idx
            replayed.append(replay)

        all_rows.extend(replayed)
        print(
            f"session={session_id} rows={len(replayed)} "
            f"t_start={replayed[0]['t_ms']:.0f} t_end={replayed[-1]['t_ms']:.0f} "
            f"{summarize('height_err', (r['abs_err_height'] for r in replayed))} "
            f"{summarize('vz_err', (r['abs_err_vz'] for r in replayed))}"
        )
        print(
            f"session={session_id} start_replay_h={replayed[0]['replay_height_mm']:.6f} "
            f"start_log_h={replayed[0]['log_height_mm']:.6f} "
            f"end_replay_h={replayed[-1]['replay_height_mm']:.6f} "
            f"end_log_h={replayed[-1]['log_height_mm']:.6f}"
        )
        print(
            f"session={session_id} start_replay_v={replayed[0]['replay_vz_mps']:.6f} "
            f"start_log_v={replayed[0]['log_vz_mps']:.6f} "
            f"end_replay_v={replayed[-1]['replay_vz_mps']:.6f} "
            f"end_log_v={replayed[-1]['log_vz_mps']:.6f}"
        )

    if args.output:
        write_csv(Path(args.output), all_rows)
        print(f"replay_csv={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
