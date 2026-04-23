import csv
from dataclasses import dataclass
import math
from itertools import combinations
from pathlib import Path
import statistics


HEIGHT_EST_DT_S = 0.01
HEIGHT_EST_RESIDUAL_GATE_MM = 120.0
HEIGHT_EST_AB_ALPHA = 0.18
HEIGHT_EST_AB_BETA = 0.03
HEIGHT_EST_PREDICT_HOLD_CNT = 15
HEIGHT_EST_VEL_DECAY = 0.95
HEIGHT_EST_WEIGHT_EPS = 0.001
HEIGHT_EST_INVALID_MM = 1300.0
CONSENSUS_SPREAD_GATE_MM = 120.0
PAIR_SPREAD_GATE_MM = 45.0
HUBER_K_MM = 25.0


@dataclass
class EstState:
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
    abs_residual_mm = abs(residual_mm)
    if abs_residual_mm >= HEIGHT_EST_RESIDUAL_GATE_MM:
        return 0.0
    return 1.0 - abs_residual_mm / HEIGHT_EST_RESIDUAL_GATE_MM


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    values = sorted(values)
    position = (len(values) - 1) * p
    low = int(position)
    high = min(low + 1, len(values) - 1)
    frac = position - low
    return values[low] * (1.0 - frac) + values[high] * frac


def correlation(a: list[float], b: list[float]) -> float | None:
    if len(a) != len(b) or len(a) < 3:
        return None
    mean_a = sum(a) / len(a)
    mean_b = sum(b) / len(b)
    diff_a = [x - mean_a for x in a]
    diff_b = [y - mean_b for y in b]
    var_a = sum(x * x for x in diff_a)
    var_b = sum(y * y for y in diff_b)
    if var_a <= 1.0e-12 or var_b <= 1.0e-12:
        return None
    return sum(x * y for x, y in zip(diff_a, diff_b)) / math.sqrt(var_a * var_b)


def estimate_lag_frames(rows: list[dict[str, float]]) -> tuple[int | None, float | None]:
    pairs = [(row["ref_mm"], row["height_mm"]) for row in rows if row["ref_mm"] is not None]
    if len(pairs) < 200:
        return None, None

    ref = [pair[0] for pair in pairs]
    est = [pair[1] for pair in pairs]
    best_lag = None
    best_corr = -10.0

    for lag in range(-12, 13):
        if lag < 0:
            ref_slice = ref[-lag:]
            est_slice = est[: len(ref_slice)]
        elif lag > 0:
            ref_slice = ref[:-lag]
            est_slice = est[lag:]
        else:
            ref_slice = ref
            est_slice = est

        corr_value = correlation(ref_slice, est_slice)
        if corr_value is not None and corr_value > best_corr:
            best_corr = corr_value
            best_lag = lag

    return best_lag, best_corr if best_lag is not None else None


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


def last_good_index(session_rows: list[dict[str, float]]) -> int:
    last_index = len(session_rows) - 1
    for index, row in enumerate(session_rows):
        heights = [row[f"I{i}"] for i in range(1, 5)]
        valid_values = [height for height in heights if height < HEIGHT_EST_INVALID_MM]
        if len(valid_values) < 3:
            continue
        if max(valid_values) - min(valid_values) <= CONSENSUS_SPREAD_GATE_MM:
            last_index = index
    return last_index


def consensus_reference_mm(row: dict[str, float]) -> float | None:
    heights = [row[f"I{i}"] for i in range(1, 5)]
    valid_values = [height for height in heights if height < HEIGHT_EST_INVALID_MM]
    if len(valid_values) < 3:
        return None
    if max(valid_values) - min(valid_values) > CONSENSUS_SPREAD_GATE_MM:
        return None
    return float(statistics.median(valid_values))


def outlier_support_reference_mm(row: dict[str, float]) -> float | None:
    heights = [row[f"I{i}"] for i in range(1, 5)]
    valid_values = [height for height in heights if height < HEIGHT_EST_INVALID_MM]
    if len(valid_values) < 3:
        return None

    ref_mm = float(statistics.median(valid_values))
    far_14 = (
        (heights[0] < HEIGHT_EST_INVALID_MM and abs(heights[0] - ref_mm) >= 60.0)
        or (heights[3] < HEIGHT_EST_INVALID_MM and abs(heights[3] - ref_mm) >= 60.0)
    )
    near_23 = (
        (heights[1] < HEIGHT_EST_INVALID_MM and abs(heights[1] - ref_mm) <= 30.0)
        or (heights[2] < HEIGHT_EST_INVALID_MM and abs(heights[2] - ref_mm) <= 30.0)
    )
    return ref_mm if far_14 and near_23 else None


def choose_consensus_cluster(valid_indices: list[int], heights_mm: list[float]) -> list[int]:
    for subset_size in range(len(valid_indices), 0, -1):
        for subset in combinations(valid_indices, subset_size):
            subset_values = [heights_mm[index] for index in subset]
            if max(subset_values) - min(subset_values) <= PAIR_SPREAD_GATE_MM:
                return list(subset)

    if len(valid_indices) >= 2:
        best_pair: list[int] | None = None
        best_spread = float("inf")
        for subset in combinations(valid_indices, 2):
            subset_values = [heights_mm[index] for index in subset]
            spread_mm = max(subset_values) - min(subset_values)
            if spread_mm < best_spread:
                best_spread = spread_mm
                best_pair = list(subset)
        if best_pair is not None:
            return best_pair

    return valid_indices[:1]


def weighted_average(heights_mm: list[float], indices: list[int], weights: list[float]) -> tuple[float | None, float]:
    weighted_sum = 0.0
    weight_sum = 0.0
    for index in indices:
        if weights[index] > 0.0:
            weighted_sum += weights[index] * heights_mm[index]
            weight_sum += weights[index]
    if weight_sum <= HEIGHT_EST_WEIGHT_EPS:
        return None, 0.0
    return weighted_sum / weight_sum, weight_sum


def observe_dual(state: EstState, heights_mm: list[float]) -> dict[str, object]:
    valid = [height < HEIGHT_EST_INVALID_MM for height in heights_mm]
    h_pred_mm = HEIGHT_EST_INVALID_MM
    v_pred_mps = 0.0
    if state.ready:
        v_pred_mps = state.vz_mps
        h_pred_mm = clamp_height(state.height_mm + v_pred_mps * HEIGHT_EST_DT_S * 1000.0)

    q = [0.0, 0.0, 0.0, 0.0]
    used_indices: list[int] = []
    z_meas_mm = h_pred_mm
    weight_sum = 0.0
    hard_relock = 0
    meas_valid = 0

    if state.ready:
        for index in (0, 3):
            if valid[index]:
                q[index] = residual_weight(1, heights_mm[index] - h_pred_mm)
        z_value, weight_sum = weighted_average(heights_mm, [0, 3], q)
        if z_value is not None:
            z_meas_mm = z_value
            meas_valid = 1
            used_indices = [index for index in (0, 3) if q[index] > 0.0]
    else:
        init_indices = [index for index in (0, 3) if valid[index]]
        if init_indices:
            z_meas_mm = sum(heights_mm[index] for index in init_indices) / len(init_indices)
            weight_sum = float(len(init_indices))
            meas_valid = 1
            used_indices = init_indices

    if (not meas_valid) and state.ready and state.hold_cnt >= HEIGHT_EST_PREDICT_HOLD_CNT:
        relock_indices = [index for index in (0, 3) if valid[index]]
        if relock_indices:
            z_meas_mm = sum(heights_mm[index] for index in relock_indices) / len(relock_indices)
            weight_sum = float(len(relock_indices))
            meas_valid = 1
            hard_relock = 1
            used_indices = relock_indices

    return {
        "valid": valid,
        "h_pred_mm": h_pred_mm,
        "v_pred_mps": v_pred_mps,
        "q": q,
        "z_meas_mm": z_meas_mm,
        "weight_sum": weight_sum,
        "meas_valid": meas_valid,
        "hard_relock": hard_relock,
        "used_indices": used_indices,
    }


def observe_all4_weighted(state: EstState, heights_mm: list[float]) -> dict[str, object]:
    valid = [height < HEIGHT_EST_INVALID_MM for height in heights_mm]
    valid_indices = [index for index, flag in enumerate(valid) if flag]
    h_pred_mm = HEIGHT_EST_INVALID_MM
    v_pred_mps = 0.0
    if state.ready:
        v_pred_mps = state.vz_mps
        h_pred_mm = clamp_height(state.height_mm + v_pred_mps * HEIGHT_EST_DT_S * 1000.0)

    q = [0.0, 0.0, 0.0, 0.0]
    used_indices: list[int] = []
    z_meas_mm = h_pred_mm
    weight_sum = 0.0
    hard_relock = 0
    meas_valid = 0

    if state.ready:
        for index in valid_indices:
            q[index] = residual_weight(1, heights_mm[index] - h_pred_mm)
        z_value, weight_sum = weighted_average(heights_mm, valid_indices, q)
        if z_value is not None:
            z_meas_mm = z_value
            meas_valid = 1
            used_indices = [index for index in valid_indices if q[index] > 0.0]
    elif valid_indices:
        z_meas_mm = sum(heights_mm[index] for index in valid_indices) / len(valid_indices)
        weight_sum = float(len(valid_indices))
        meas_valid = 1
        used_indices = valid_indices

    if (not meas_valid) and state.ready and state.hold_cnt >= HEIGHT_EST_PREDICT_HOLD_CNT and valid_indices:
        z_meas_mm = sum(heights_mm[index] for index in valid_indices) / len(valid_indices)
        weight_sum = float(len(valid_indices))
        meas_valid = 1
        hard_relock = 1
        used_indices = valid_indices

    return {
        "valid": valid,
        "h_pred_mm": h_pred_mm,
        "v_pred_mps": v_pred_mps,
        "q": q,
        "z_meas_mm": z_meas_mm,
        "weight_sum": weight_sum,
        "meas_valid": meas_valid,
        "hard_relock": hard_relock,
        "used_indices": used_indices,
    }


def observe_median4_gate(state: EstState, heights_mm: list[float]) -> dict[str, object]:
    valid = [height < HEIGHT_EST_INVALID_MM for height in heights_mm]
    valid_indices = [index for index, flag in enumerate(valid) if flag]
    h_pred_mm = HEIGHT_EST_INVALID_MM
    v_pred_mps = 0.0
    if state.ready:
        v_pred_mps = state.vz_mps
        h_pred_mm = clamp_height(state.height_mm + v_pred_mps * HEIGHT_EST_DT_S * 1000.0)

    if state.ready:
        gated_indices = [index for index in valid_indices if abs(heights_mm[index] - h_pred_mm) < HEIGHT_EST_RESIDUAL_GATE_MM]
    else:
        gated_indices = valid_indices[:]

    z_meas_mm = h_pred_mm
    weight_sum = 0.0
    hard_relock = 0
    meas_valid = 0

    if gated_indices:
        gated_values = [heights_mm[index] for index in gated_indices]
        if len(gated_values) >= 3:
            z_meas_mm = float(statistics.median(gated_values))
        else:
            z_meas_mm = sum(gated_values) / len(gated_values)
        weight_sum = float(len(gated_indices))
        meas_valid = 1
    elif state.ready and state.hold_cnt >= HEIGHT_EST_PREDICT_HOLD_CNT and len(valid_indices) >= 2:
        relock_values = [heights_mm[index] for index in valid_indices]
        if len(relock_values) >= 3:
            z_meas_mm = float(statistics.median(relock_values))
        else:
            z_meas_mm = sum(relock_values) / len(relock_values)
        weight_sum = float(len(valid_indices))
        meas_valid = 1
        hard_relock = 1

    return {
        "valid": valid,
        "h_pred_mm": h_pred_mm,
        "v_pred_mps": v_pred_mps,
        "q": [0.0, 0.0, 0.0, 0.0],
        "z_meas_mm": z_meas_mm,
        "weight_sum": weight_sum,
        "meas_valid": meas_valid,
        "hard_relock": hard_relock,
        "used_indices": gated_indices if meas_valid and not hard_relock else valid_indices if hard_relock else [],
    }


def observe_huber4(state: EstState, heights_mm: list[float]) -> dict[str, object]:
    valid = [height < HEIGHT_EST_INVALID_MM for height in heights_mm]
    valid_indices = [index for index, flag in enumerate(valid) if flag]
    h_pred_mm = HEIGHT_EST_INVALID_MM
    v_pred_mps = 0.0
    if state.ready:
        v_pred_mps = state.vz_mps
        h_pred_mm = clamp_height(state.height_mm + v_pred_mps * HEIGHT_EST_DT_S * 1000.0)

    q = [0.0, 0.0, 0.0, 0.0]
    z_meas_mm = h_pred_mm
    weight_sum = 0.0
    hard_relock = 0
    meas_valid = 0
    used_indices: list[int] = []

    if state.ready:
        gated_indices = [index for index in valid_indices if abs(heights_mm[index] - h_pred_mm) < HEIGHT_EST_RESIDUAL_GATE_MM]
    else:
        gated_indices = valid_indices[:]

    if gated_indices:
        center_mm = float(statistics.median(heights_mm[index] for index in gated_indices))
        for index in gated_indices:
            deviation_mm = abs(heights_mm[index] - center_mm)
            robust_weight = 1.0 if deviation_mm <= HUBER_K_MM else HUBER_K_MM / deviation_mm
            predict_weight = 1.0 if not state.ready else residual_weight(1, heights_mm[index] - h_pred_mm)
            q[index] = robust_weight * predict_weight
        z_value, weight_sum = weighted_average(heights_mm, gated_indices, q)
        if z_value is not None:
            z_meas_mm = z_value
            meas_valid = 1
            used_indices = [index for index in gated_indices if q[index] > 0.0]
    elif state.ready and state.hold_cnt >= HEIGHT_EST_PREDICT_HOLD_CNT and len(valid_indices) >= 2:
        center_mm = float(statistics.median(heights_mm[index] for index in valid_indices))
        for index in valid_indices:
            deviation_mm = abs(heights_mm[index] - center_mm)
            q[index] = 1.0 if deviation_mm <= HUBER_K_MM else HUBER_K_MM / deviation_mm
        z_value, weight_sum = weighted_average(heights_mm, valid_indices, q)
        if z_value is not None:
            z_meas_mm = z_value
            meas_valid = 1
            hard_relock = 1
            used_indices = [index for index in valid_indices if q[index] > 0.0]

    return {
        "valid": valid,
        "h_pred_mm": h_pred_mm,
        "v_pred_mps": v_pred_mps,
        "q": q,
        "z_meas_mm": z_meas_mm,
        "weight_sum": weight_sum,
        "meas_valid": meas_valid,
        "hard_relock": hard_relock,
        "used_indices": used_indices,
    }


def observe_subset_consensus(state: EstState, heights_mm: list[float]) -> dict[str, object]:
    valid = [height < HEIGHT_EST_INVALID_MM for height in heights_mm]
    valid_indices = [index for index, flag in enumerate(valid) if flag]
    h_pred_mm = HEIGHT_EST_INVALID_MM
    v_pred_mps = 0.0
    if state.ready:
        v_pred_mps = state.vz_mps
        h_pred_mm = clamp_height(state.height_mm + v_pred_mps * HEIGHT_EST_DT_S * 1000.0)

    q = [0.0, 0.0, 0.0, 0.0]
    z_meas_mm = h_pred_mm
    weight_sum = 0.0
    hard_relock = 0
    meas_valid = 0
    used_indices: list[int] = []

    if state.ready:
        gated_indices = [index for index in valid_indices if abs(heights_mm[index] - h_pred_mm) < HEIGHT_EST_RESIDUAL_GATE_MM]
    else:
        gated_indices = valid_indices[:]

    if gated_indices:
        cluster_indices = choose_consensus_cluster(gated_indices, heights_mm)
        cluster_values = [heights_mm[index] for index in cluster_indices]
        if len(cluster_values) >= 3:
            z_meas_mm = float(statistics.median(cluster_values))
        else:
            z_meas_mm = sum(cluster_values) / len(cluster_values)
        weight_sum = float(len(cluster_indices))
        meas_valid = 1
        used_indices = cluster_indices
    elif state.ready and state.hold_cnt >= HEIGHT_EST_PREDICT_HOLD_CNT and len(valid_indices) >= 2:
        cluster_indices = choose_consensus_cluster(valid_indices, heights_mm)
        cluster_values = [heights_mm[index] for index in cluster_indices]
        if len(cluster_values) >= 3:
            z_meas_mm = float(statistics.median(cluster_values))
        else:
            z_meas_mm = sum(cluster_values) / len(cluster_values)
        weight_sum = float(len(cluster_indices))
        meas_valid = 1
        hard_relock = 1
        used_indices = cluster_indices

    return {
        "valid": valid,
        "h_pred_mm": h_pred_mm,
        "v_pred_mps": v_pred_mps,
        "q": q,
        "z_meas_mm": z_meas_mm,
        "weight_sum": weight_sum,
        "meas_valid": meas_valid,
        "hard_relock": hard_relock,
        "used_indices": used_indices,
    }


OBSERVERS = {
    "dual": observe_dual,
    "all4_weighted": observe_all4_weighted,
    "median4_gate": observe_median4_gate,
    "huber4": observe_huber4,
    "subset_consensus": observe_subset_consensus,
}


def update_state(state: EstState, obs: dict[str, object]) -> tuple[int, int]:
    predict_only = 0
    replay_valid = 0

    if not state.ready:
        if obs["meas_valid"]:
            state.height_mm = clamp_height(float(obs["z_meas_mm"]))
            state.vz_mps = 0.0
            state.ready = 1
            state.hold_cnt = 0
            replay_valid = 1
    elif obs["meas_valid"]:
        if obs["hard_relock"]:
            state.height_mm = clamp_height(float(obs["z_meas_mm"]))
            state.vz_mps = 0.0
        else:
            residual_mm = float(obs["z_meas_mm"]) - float(obs["h_pred_mm"])
            state.height_mm = clamp_height(float(obs["h_pred_mm"]) + HEIGHT_EST_AB_ALPHA * residual_mm)
            state.vz_mps = float(obs["v_pred_mps"]) + (HEIGHT_EST_AB_BETA / HEIGHT_EST_DT_S) * (residual_mm * 0.001)
        state.hold_cnt = 0
        replay_valid = 1
    else:
        state.height_mm = clamp_height(float(obs["h_pred_mm"]))
        state.vz_mps = float(obs["v_pred_mps"]) * HEIGHT_EST_VEL_DECAY
        if state.hold_cnt < HEIGHT_EST_PREDICT_HOLD_CNT:
            state.hold_cnt += 1
            replay_valid = 1
            predict_only = 1

    return replay_valid, predict_only


def run_mode(mode: str, sessions: list[list[dict[str, float]]], trim_tail: bool) -> tuple[list[dict[str, float]], list[dict[str, int]]]:
    observer = OBSERVERS[mode]
    all_rows: list[dict[str, float]] = []
    session_summary: list[dict[str, int]] = []

    for session_id, session_rows in enumerate(sessions):
        end_index = last_good_index(session_rows) + 1 if trim_tail else len(session_rows)
        state = EstState()
        hard_relock_count = 0
        predict_only_count = 0

        for row_index, row in enumerate(session_rows[:end_index]):
            heights_mm = [row[f"I{i}"] for i in range(1, 5)]
            obs = observer(state, heights_mm)
            replay_valid, predict_only = update_state(state, obs)
            hard_relock_count += int(obs["hard_relock"])
            predict_only_count += predict_only

            all_rows.append(
                {
                    "mode": mode,
                    "session_id": session_id,
                    "row_in_session": row_index,
                    "t_ms": row["I0"],
                    "height_mm": state.height_mm if state.ready else HEIGHT_EST_INVALID_MM,
                    "vz_mps": state.vz_mps if state.ready else 0.0,
                    "log_height_mm": row["I9"],
                    "log_vz_mps": row["I8"],
                    "ref_mm": consensus_reference_mm(row),
                    "outlier_ref_mm": outlier_support_reference_mm(row),
                    "predict_only": predict_only,
                    "replay_valid": replay_valid,
                    "hard_relock": int(obs["hard_relock"]),
                    "used_cnt": len(obs["used_indices"]),
                    "valid_cnt": sum(1 for height in heights_mm if height < HEIGHT_EST_INVALID_MM),
                }
            )

        session_summary.append(
            {
                "session_id": session_id,
                "rows": end_index,
                "hard_relock_count": hard_relock_count,
                "predict_only_count": predict_only_count,
            }
        )

    return all_rows, session_summary


def summarize_mode(rows: list[dict[str, float]], session_summary: list[dict[str, int]]) -> dict[str, float | int | None]:
    heights = [row["height_mm"] for row in rows]
    vz_list = [row["vz_mps"] for row in rows]
    height_steps = [heights[index] - heights[index - 1] for index in range(1, len(heights))]
    vz_steps = [vz_list[index] - vz_list[index - 1] for index in range(1, len(vz_list))]
    ref_errors = [abs(row["height_mm"] - row["ref_mm"]) for row in rows if row["ref_mm"] is not None]
    outlier_errors = [abs(row["height_mm"] - row["outlier_ref_mm"]) for row in rows if row["outlier_ref_mm"] is not None]

    quiet_vz = []
    for index in range(1, len(rows)):
        if rows[index]["ref_mm"] is None or rows[index - 1]["ref_mm"] is None:
            continue
        ref_speed_mps = abs((rows[index]["ref_mm"] - rows[index - 1]["ref_mm"]) * 0.001 / HEIGHT_EST_DT_S)
        if ref_speed_mps < 0.15:
            quiet_vz.append(abs(rows[index]["vz_mps"]))

    lag_frames, lag_corr = estimate_lag_frames(rows)

    return {
        "rows": len(rows),
        "valid_ratio": sum(row["replay_valid"] for row in rows) / len(rows),
        "predict_ratio": sum(row["predict_only"] for row in rows) / len(rows),
        "hard_relock_count": sum(item["hard_relock_count"] for item in session_summary),
        "height_step_std_mm": statistics.pstdev(height_steps) if len(height_steps) > 1 else 0.0,
        "vz_step_std_mps": statistics.pstdev(vz_steps) if len(vz_steps) > 1 else 0.0,
        "ref_err_p50_mm": percentile(ref_errors, 0.50),
        "ref_err_p95_mm": percentile(ref_errors, 0.95),
        "ref_err_max_mm": max(ref_errors) if ref_errors else None,
        "quiet_vz_p95_mps": percentile(quiet_vz, 0.95),
        "outlier_pull_p95_mm": percentile(outlier_errors, 0.95),
        "outlier_pull_max_mm": max(outlier_errors) if outlier_errors else None,
        "lag_frames": lag_frames,
        "lag_corr": lag_corr,
    }


def summarize_dual_replay_against_log(sessions: list[list[dict[str, float]]]) -> list[dict[str, float]]:
    replay_rows, _ = run_mode("dual", sessions, trim_tail=False)
    summary: list[dict[str, float]] = []

    for session_id in range(len(sessions)):
        session_rows = [row for row in replay_rows if row["session_id"] == session_id]
        height_errors = [abs(row["height_mm"] - row["log_height_mm"]) for row in session_rows]
        vz_errors = [abs(row["vz_mps"] - row["log_vz_mps"]) for row in session_rows]
        summary.append(
            {
                "session_id": session_id,
                "rows": len(session_rows),
                "t_start_ms": session_rows[0]["t_ms"],
                "t_end_ms": session_rows[-1]["t_ms"],
                "height_err_mean_mm": sum(height_errors) / len(height_errors),
                "height_err_max_mm": max(height_errors),
                "vz_err_mean_mps": sum(vz_errors) / len(vz_errors),
                "vz_err_max_mps": max(vz_errors),
                "start_replay_h_mm": session_rows[0]["height_mm"],
                "start_log_h_mm": session_rows[0]["log_height_mm"],
                "end_replay_h_mm": session_rows[-1]["height_mm"],
                "end_log_h_mm": session_rows[-1]["log_height_mm"],
                "start_replay_vz_mps": session_rows[0]["vz_mps"],
                "start_log_vz_mps": session_rows[0]["log_vz_mps"],
                "end_replay_vz_mps": session_rows[-1]["vz_mps"],
                "end_log_vz_mps": session_rows[-1]["log_vz_mps"],
            }
        )

    return summary


def summarize_tof23_value(sessions: list[list[dict[str, float]]]) -> dict[str, int]:
    dual_rows, _ = run_mode("dual", sessions, trim_tail=False)
    by_session_row = {(row["session_id"], row["row_in_session"]): row for row in dual_rows}

    stats = {
        "ready_rows": 0,
        "single_14_plus_23_valid": 0,
        "single_14_accept_plus_23_accept": 0,
        "both_14_invalid_23_valid": 0,
        "both_14_reject_23_accept": 0,
        "outlier_14_supported_by_23": 0,
    }

    for session_id, session_rows in enumerate(sessions):
        for row_index, row in enumerate(session_rows):
            dual_row = by_session_row[(session_id, row_index)]
            if dual_row["row_in_session"] == 0:
                continue

            heights = [row[f"I{i}"] for i in range(1, 5)]
            valid = [height < HEIGHT_EST_INVALID_MM for height in heights]
            stats["ready_rows"] += 1

            if sum((valid[0], valid[3])) == 1 and (valid[1] or valid[2]):
                stats["single_14_plus_23_valid"] += 1

            h_pred_mm = dual_row["height_mm"] - dual_row["vz_mps"] * HEIGHT_EST_DT_S * 1000.0
            accept = [valid[index] and residual_weight(1, heights[index] - h_pred_mm) > 0.0 for index in range(4)]

            if sum((accept[0], accept[3])) == 1 and (accept[1] or accept[2]):
                stats["single_14_accept_plus_23_accept"] += 1
            if (not valid[0]) and (not valid[3]) and (valid[1] or valid[2]):
                stats["both_14_invalid_23_valid"] += 1
            if (not accept[0]) and (not accept[3]) and (accept[1] or accept[2]):
                stats["both_14_reject_23_accept"] += 1
            if outlier_support_reference_mm(row) is not None:
                stats["outlier_14_supported_by_23"] += 1

    return stats


def format_value(value: float | int | None) -> str:
    if value is None:
        return "None"
    if isinstance(value, int):
        return str(value)
    return f"{value:.6f}"


def main() -> int:
    csv_path = Path("project/code/Estimation/Height_Est/0423_tof_fused/0423_tof_fused.csv")
    rows, dropped_rows = load_rows(csv_path)
    sessions = split_sessions(rows)

    print(f"rows={len(rows)} dropped_rows={dropped_rows} sessions={len(sessions)}")
    print("session_ranges:")
    for session_id, session_rows in enumerate(sessions):
        print(
            f"  session={session_id} rows={len(session_rows)} "
            f"t_start={int(session_rows[0]['I0'])} t_end={int(session_rows[-1]['I0'])}"
        )

    print("\ndual_replay_vs_log:")
    for item in summarize_dual_replay_against_log(sessions):
        print(
            f"  session={item['session_id']} rows={item['rows']} "
            f"height_err_mean={item['height_err_mean_mm']:.6f} "
            f"height_err_max={item['height_err_max_mm']:.6f} "
            f"vz_err_mean={item['vz_err_mean_mps']:.6f} "
            f"vz_err_max={item['vz_err_max_mps']:.6f} "
            f"start_h={item['start_replay_h_mm']:.6f}/{item['start_log_h_mm']:.6f} "
            f"end_h={item['end_replay_h_mm']:.6f}/{item['end_log_h_mm']:.6f} "
            f"start_vz={item['start_replay_vz_mps']:.6f}/{item['start_log_vz_mps']:.6f} "
            f"end_vz={item['end_replay_vz_mps']:.6f}/{item['end_log_vz_mps']:.6f}"
        )

    tof23_stats = summarize_tof23_value(sessions)
    print("\ntof23_value:")
    for key, value in tof23_stats.items():
        print(f"  {key}={value}")

    for trim_tail in (False, True):
        print(f"\nmode_metrics trim_tail={int(trim_tail)}")
        for mode in OBSERVERS:
            mode_rows, session_summary = run_mode(mode, sessions, trim_tail=trim_tail)
            summary = summarize_mode(mode_rows, session_summary)
            print(
                f"  mode={mode} "
                f"rows={summary['rows']} "
                f"valid_ratio={format_value(summary['valid_ratio'])} "
                f"predict_ratio={format_value(summary['predict_ratio'])} "
                f"hard_relock={format_value(summary['hard_relock_count'])} "
                f"height_step_std={format_value(summary['height_step_std_mm'])} "
                f"vz_step_std={format_value(summary['vz_step_std_mps'])} "
                f"ref_p50={format_value(summary['ref_err_p50_mm'])} "
                f"ref_p95={format_value(summary['ref_err_p95_mm'])} "
                f"ref_max={format_value(summary['ref_err_max_mm'])} "
                f"quiet_vz_p95={format_value(summary['quiet_vz_p95_mps'])} "
                f"outlier_p95={format_value(summary['outlier_pull_p95_mm'])} "
                f"outlier_max={format_value(summary['outlier_pull_max_mm'])} "
                f"lag_frames={format_value(summary['lag_frames'])} "
                f"lag_corr={format_value(summary['lag_corr'])}"
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
