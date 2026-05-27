import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parent
LOG_PATH = next(ROOT.glob("*-2.csv"))

DT_S = 0.01
TOF_BIAS_MM = np.array([-14.18, 14.33, -37.02, 55.94], dtype=float)


def clamp(value, lo, hi):
    return min(max(value, lo), hi)


def lpf_alpha(fc_hz, dt_s=DT_S):
    tau = 1.0 / (2.0 * math.pi * fc_hz)
    return dt_s / (tau + dt_s)


def load_log():
    df = pd.read_csv(LOG_PATH)
    df["t_s"] = df["I0"] / 1000.0
    return df


def build_tof_measure(df):
    corrected = df[[f"I{i}" for i in range(7, 11)]].to_numpy(dtype=float)
    sample = corrected - TOF_BIAS_MM.reshape(1, 4)
    obs = np.full(len(df), np.nan)
    spread = np.full(len(df), np.nan)
    count = np.zeros(len(df), dtype=int)
    accept_count = np.zeros(len(df), dtype=int)

    for i, row in enumerate(sample):
        valid = row[(row > 40.0) & (row < 2000.0)]
        count[i] = len(valid)
        if len(valid) < 3:
            continue
        valid.sort()
        center = float(np.median(valid))
        spread[i] = float(valid[-1] - valid[0])
        if spread[i] > 240.0:
            continue
        trimmed = valid[np.abs(valid - center) <= 110.0]
        if len(trimmed) >= 2:
            obs[i] = float(np.mean(trimmed))
            accept_count[i] = len(trimmed)
        else:
            obs[i] = center
            accept_count[i] = len(trimmed)

    return obs, spread, count, accept_count


def estimate_vz_from_height(z_mm, valid):
    vz = np.zeros(len(z_mm))
    alpha = 0.2212
    ready = False
    prev = 0.0
    filt = 0.0
    for i, z in enumerate(z_mm):
        if not valid[i] or not np.isfinite(z):
            ready = False
            vz[i] = 0.0
            continue
        if not ready:
            prev = z
            filt = 0.0
            ready = True
            vz[i] = 0.0
            continue
        raw = clamp((z - prev) * 0.001 / DT_S, -1.5, 1.5)
        prev = z
        filt += alpha * (raw - filt)
        vz[i] = filt
    return vz


@dataclass
class ReplayResult:
    name: str
    z_mm: np.ndarray
    vz_mps: np.ndarray
    accepted: np.ndarray
    polluted: np.ndarray


def replay_trimmed_lpf(df, obs):
    z = np.full(len(df), np.nan)
    accepted = np.zeros(len(df), dtype=bool)
    polluted = np.zeros(len(df), dtype=bool)
    alpha = 0.2212
    ready = False
    state = 0.0
    for i, meas in enumerate(obs):
        if np.isfinite(meas):
            if not ready:
                state = meas
                ready = True
            else:
                state += alpha * (meas - state)
            accepted[i] = True
        z[i] = state if ready else np.nan
    return ReplayResult("A_trimmed_lpf", z, estimate_vz_from_height(z, np.isfinite(z)), accepted, polluted)


def replay_huber_ab(df, obs):
    z = np.full(len(df), np.nan)
    v = np.zeros(len(df))
    accepted = np.zeros(len(df), dtype=bool)
    polluted = np.zeros(len(df), dtype=bool)
    alpha = 0.18
    beta = 0.025
    huber_mm = 120.0
    ready = False
    zh = 0.0
    vh = 0.0
    for i, meas in enumerate(obs):
        acc = clamp(float(df.iloc[i]["I12"]), -4.0, 4.0)
        if ready:
            vh = clamp(0.985 * vh + 0.15 * acc * DT_S, -1.5, 1.5)
            zh = clamp(zh + vh * DT_S * 1000.0, 0.0, 1400.0)
        if np.isfinite(meas):
            if not ready:
                zh = meas
                vh = 0.0
                ready = True
                accepted[i] = True
            else:
                r = meas - zh
                r_sat = clamp(r, -huber_mm, huber_mm)
                zh = clamp(zh + alpha * r_sat, 0.0, 1400.0)
                vh = clamp(vh + beta * r_sat * 0.001 / DT_S, -1.5, 1.5)
                accepted[i] = True
        z[i] = zh if ready else np.nan
        v[i] = vh if ready else 0.0
    return ReplayResult("B_huber_ab", z, v, accepted, polluted)


def replay_physical_gate_ab(df, obs, spread):
    z = np.full(len(df), np.nan)
    v = np.zeros(len(df))
    accepted = np.zeros(len(df), dtype=bool)
    polluted = np.zeros(len(df), dtype=bool)
    ready = False
    zh = 0.0
    vh = 0.0
    alpha = 0.20
    beta = 0.028
    reject_cnt = 0
    for i, meas in enumerate(obs):
        acc = clamp(float(df.iloc[i]["I12"]), -4.0, 4.0)
        target = float(df.iloc[i]["I1"])
        if ready:
            vh = clamp(0.990 * vh + 0.20 * acc * DT_S, -1.5, 1.5)
            zh = clamp(zh + vh * DT_S * 1000.0, 0.0, 1400.0)
        if np.isfinite(meas):
            if not ready:
                zh = meas
                vh = 0.0
                ready = True
                accepted[i] = True
            else:
                r = meas - zh
                suspicious_low = (
                    target > 500.0
                    and r < -180.0
                    and meas < 850.0
                    and (not np.isfinite(spread[i]) or spread[i] <= 240.0)
                    and acc > -2.5
                    and vh > -1.2
                )
                if suspicious_low:
                    reject_cnt = min(reject_cnt + 1, 100)
                    polluted[i] = True
                else:
                    reject_cnt = max(reject_cnt - 1, 0)
                    step_r = clamp(r, -160.0, 160.0)
                    zh = clamp(zh + alpha * step_r, 0.0, 1400.0)
                    vh = clamp(vh + beta * step_r * 0.001 / DT_S, -1.5, 1.5)
                    accepted[i] = True
        else:
            reject_cnt = max(reject_cnt - 1, 0)
        z[i] = zh if ready else np.nan
        v[i] = vh if ready else 0.0
    return ReplayResult("C_physical_gate_ab", z, v, accepted, polluted)


def replay_pollution_fsm(df, obs, spread):
    z = np.full(len(df), np.nan)
    v = np.zeros(len(df))
    accepted = np.zeros(len(df), dtype=bool)
    polluted = np.zeros(len(df), dtype=bool)
    ready = False
    zh = 0.0
    vh = 0.0
    alpha = 0.16
    beta = 0.020
    in_pollution = False
    enter_cnt = 0
    exit_cnt = 0
    blend_cnt = 0

    for i, meas in enumerate(obs):
        acc = clamp(float(df.iloc[i]["I12"]), -4.0, 4.0)
        target = float(df.iloc[i]["I1"])
        throttle = float(df.iloc[i]["I16"])

        if ready:
            if in_pollution:
                vh = clamp(0.94 * vh + 0.08 * acc * DT_S, -0.6, 0.6)
            else:
                vh = clamp(0.990 * vh + 0.18 * acc * DT_S, -1.5, 1.5)
            zh = clamp(zh + vh * DT_S * 1000.0, 0.0, 1400.0)

        if np.isfinite(meas):
            if not ready:
                zh = meas
                vh = 0.0
                ready = True
                accepted[i] = True
            else:
                r = meas - zh
                low_consensus = (
                    target > 500.0
                    and meas < 850.0
                    and r < -180.0
                    and np.isfinite(spread[i])
                    and spread[i] <= 240.0
                )
                physics_not_drop = (acc > -2.5 and vh > -1.2) or (throttle > 4300.0 and acc > -4.0)

                if in_pollution:
                    polluted[i] = True
                    if abs(r) < 120.0 or (meas > zh - 90.0 and meas > 850.0):
                        exit_cnt += 1
                    else:
                        exit_cnt = 0
                    if exit_cnt >= 8:
                        in_pollution = False
                        enter_cnt = 0
                        exit_cnt = 0
                        blend_cnt = 20
                    else:
                        z[i] = zh if ready else np.nan
                        v[i] = vh if ready else 0.0
                        continue

                if low_consensus and physics_not_drop:
                    enter_cnt += 1
                    if enter_cnt >= 2:
                        in_pollution = True
                        polluted[i] = True
                        exit_cnt = 0
                        z[i] = zh if ready else np.nan
                        v[i] = vh if ready else 0.0
                        continue
                else:
                    enter_cnt = 0

                r = meas - zh
                if blend_cnt > 0:
                    a = 0.06
                    b = 0.008
                    max_r = 80.0
                    blend_cnt -= 1
                else:
                    a = alpha
                    b = beta
                    max_r = 140.0
                step_r = clamp(r, -max_r, max_r)
                zh = clamp(zh + a * step_r, 0.0, 1400.0)
                vh = clamp(vh + b * step_r * 0.001 / DT_S, -1.5, 1.5)
                accepted[i] = True
        else:
            if in_pollution:
                polluted[i] = True
            enter_cnt = max(enter_cnt - 1, 0)

        z[i] = zh if ready else np.nan
        v[i] = vh if ready else 0.0

    return ReplayResult("D_pollution_fsm", z, v, accepted, polluted)


def replay_recommended_mcu(df, obs, spread):
    z = np.full(len(df), np.nan)
    v = np.zeros(len(df))
    accepted = np.zeros(len(df), dtype=bool)
    polluted = np.zeros(len(df), dtype=bool)

    ready = False
    zh = 0.0
    vh = 0.0
    pollution = False
    susp_cnt = 0
    clear_cnt = 0
    soft_rejoin = 0

    for i, meas in enumerate(obs):
        target = float(df.iloc[i]["I1"])
        pos_out = float(df.iloc[i]["I13"])
        acc_up = clamp(float(df.iloc[i]["I12"]), -4.0, 4.0)

        if ready:
            # IMU 只做短时桥接，不让积分漂移当老大。
            acc_gain = 0.08 if pollution else 0.15
            vel_decay = 0.94 if pollution else 0.985
            vh = clamp(vh * vel_decay + acc_gain * acc_up * DT_S, -1.2, 1.2)
            if pollution:
                vh = clamp(vh, -0.35, 0.35)
            zh = clamp(zh + vh * DT_S * 1000.0, 0.0, 1400.0)

        if not np.isfinite(meas):
            if pollution:
                polluted[i] = True
            z[i] = zh if ready else np.nan
            v[i] = vh if ready else 0.0
            continue

        if not ready:
            zh = meas
            vh = 0.0
            ready = True
            accepted[i] = True
            z[i] = zh
            v[i] = vh
            continue

        residual = meas - zh
        consensus = np.isfinite(spread[i]) and spread[i] <= 240.0
        low_against_state = residual < -160.0
        low_against_target = (target > 500.0) and (meas < target - 180.0)
        strong_low = (meas < 850.0) and (low_against_state or low_against_target)

        real_drop_evidence = (
            (target < 500.0)
            or ((acc_up < -2.8) and (vh < -0.55))
            or ((pos_out < -0.35) and (acc_up < -1.4) and (vh < -0.35))
        )
        suspicious_low = consensus and strong_low and (not real_drop_evidence)

        if pollution:
            polluted[i] = True
            # 必须连续多帧回到预测附近，才允许重接入，避免刚恢复就反向冲高。
            if (not suspicious_low) and ((abs(residual) < 120.0) or ((meas > 850.0) and (meas > zh - 80.0))):
                clear_cnt += 1
            else:
                clear_cnt = 0
            if clear_cnt < 8:
                z[i] = zh
                v[i] = vh
                continue
            pollution = False
            susp_cnt = 0
            clear_cnt = 0
            soft_rejoin = 20

        if suspicious_low:
            susp_cnt += 1
            polluted[i] = True
            if susp_cnt >= 1:
                pollution = True
                clear_cnt = 0
                z[i] = zh
                v[i] = vh
                continue
        else:
            susp_cnt = 0

        residual = meas - zh
        if soft_rejoin > 0:
            alpha = 0.05
            beta = 0.006
            r_limit = 70.0
            soft_rejoin -= 1
        elif real_drop_evidence:
            alpha = 0.18
            beta = 0.020
            r_limit = 150.0
        else:
            alpha = 0.16
            beta = 0.018
            r_limit = 110.0

        r_clip = clamp(residual, -r_limit, r_limit)
        zh = clamp(zh + alpha * r_clip, 0.0, 1400.0)
        vh = clamp(vh + beta * r_clip * 0.001 / DT_S, -1.2, 1.2)
        accepted[i] = True
        z[i] = zh
        v[i] = vh

    return ReplayResult("E_recommended_mcu", z, v, accepted, polluted)


def replay_ported_c_logic(df, obs, spread):
    z = np.full(len(df), np.nan)
    v = np.zeros(len(df))
    accepted = np.zeros(len(df), dtype=bool)
    polluted_rows = np.zeros(len(df), dtype=bool)

    est = 0.0
    out = 0.0
    prev_out = 0.0
    state_v = 0.0
    vz_lpf = 0.0
    ready = False
    out_ready = False
    vz_ready = False
    polluted = False
    exit_cnt = 0
    soft_rejoin = 0
    miss_cnt = 0

    for i, meas in enumerate(obs):
        raw = df.iloc[i][["I3", "I4", "I5", "I6"]].to_numpy(dtype=float)
        raw_high_count = int(np.sum((raw >= 1390.0) | (raw >= 8192.0)))
        range_high_active = bool(ready and raw_high_count >= 3 and est >= 900.0)
        meas_valid = bool(np.isfinite(meas) or range_high_active)
        if range_high_active:
            meas = 1400.0

        target = float(df.iloc[i]["I1"])
        pos_out = float(df.iloc[i]["I13"])
        acc = clamp(float(df.iloc[i]["I12"]), -4.0, 4.0)

        if not ready:
            if not meas_valid:
                continue
            est = float(meas)
            out = est
            prev_out = out
            ready = True
            out_ready = True
            accepted[i] = True
            z[i] = out
            v[i] = 0.0
            continue

        if polluted:
            state_v = clamp(0.94 * state_v + 0.08 * acc * DT_S, -0.35, 0.35)
        else:
            state_v = clamp(0.985 * state_v + 0.15 * acc * DT_S, -1.2, 1.2)
        est = clamp(est + state_v * DT_S * 1000.0, 0.0, 1400.0)

        if meas_valid:
            if range_high_active:
                residual = 1400.0 - est
                # C 移植版：高量程不作为 1400mm 观测，只保持短时预测。
                accepted[i] = False
            else:
                residual = float(meas) - est
                real_descent = (
                    target < 500.0
                    or (acc < -2.8 and state_v < -0.55)
                    or (pos_out < -0.35 and acc < -1.4 and state_v < -0.35)
                )
                low_consensus = np.isfinite(spread[i]) and spread[i] <= 240.0
                low_state = est >= 650.0 and residual < -160.0
                low_target = target > 500.0 and meas < target - 180.0
                low_polluted = (
                    low_consensus
                    and meas < 850.0
                    and (low_state or low_target)
                    and (not real_descent)
                )

                if polluted:
                    if abs(residual) <= 140.0 or (meas >= 850.0 and residual >= -220.0):
                        exit_cnt += 1
                    else:
                        exit_cnt = 0
                    if exit_cnt < 8:
                        low_polluted = True
                    else:
                        polluted = False
                        exit_cnt = 0
                        soft_rejoin = 20
                elif low_polluted:
                    polluted = True
                    exit_cnt = 0

                if low_polluted:
                    polluted_rows[i] = True
                    miss_cnt = 0
                else:
                    before = est
                    soft_rejoin_active = False
                    step_limit = 90.0 if abs(residual) > 250.0 else 35.0
                    if soft_rejoin:
                        soft_rejoin_active = True
                        step_limit = 8.0
                        soft_rejoin -= 1
                    if residual < 0.0:
                        if residual < -220.0 and not real_descent:
                            step_limit = 30.0
                        elif step_limit > 55.0:
                            step_limit = 55.0
                    delta = clamp(residual, -step_limit, step_limit)
                    est = clamp(est + delta, 0.0, 1400.0)
                    state_v = state_v + 0.20 * (est - before) * 0.001 / DT_S
                    if soft_rejoin_active:
                        state_v = clamp(state_v, -0.35, 0.35)
                    else:
                        state_v = clamp(state_v, -1.2, 1.2)
                    accepted[i] = True
                    miss_cnt = 0
        else:
            miss_cnt += 1

        if out_ready:
            out += 0.22120 * (est - out)
        else:
            out = est
            out_ready = True

        if miss_cnt > 15:
            z[i] = out
            v[i] = 0.0
            vz_ready = False
            continue

        z[i] = out
        if polluted:
            prev_out = out
            v[i] = clamp(state_v, -0.35, 0.35)
            vz_lpf = state_v
            vz_ready = True
        elif soft_rejoin:
            prev_out = out
            v[i] = clamp(state_v, -0.35, 0.35)
            vz_lpf = v[i]
            vz_ready = True
        elif not vz_ready:
            prev_out = out
            vz_lpf = 0.0
            vz_ready = True
            v[i] = 0.0
        else:
            raw_v = clamp((out - prev_out) * 0.001 / DT_S, -1.5, 1.5)
            prev_out = out
            vz_lpf += 0.22120 * (raw_v - vz_lpf)
            v[i] = vz_lpf

        if polluted:
            polluted_rows[i] = True

    return ReplayResult("F_ported_c_logic", z, v, accepted, polluted_rows)


def contiguous_segments(mask, min_len=1, merge_gap=0):
    raw = []
    s = None
    for i, m in enumerate(mask):
        if m and s is None:
            s = i
        if (not m or i == len(mask) - 1) and s is not None:
            e = i - 1 if not m else i
            if e - s + 1 >= min_len:
                raw.append([s, e])
            s = None
    if merge_gap <= 0 or not raw:
        return [tuple(x) for x in raw]
    merged = [raw[0]]
    for s, e in raw[1:]:
        if s - merged[-1][1] - 1 <= merge_gap:
            merged[-1][1] = e
        else:
            merged.append([s, e])
    return [tuple(x) for x in merged]


def detect_pollution_events(df, obs, spread):
    logged = df["I2"].to_numpy(dtype=float)
    flight = (df["I1"].to_numpy(dtype=float) > 500.0) & (df["I16"].to_numpy(dtype=float) > 2500.0)
    mask = (
        flight
        & np.isfinite(obs)
        & np.isfinite(spread)
        & (spread <= 240.0)
        & (obs < 850.0)
        & ((logged - obs) > 180.0)
    )
    # Merge nearby short TOF flickers into one interference pass.
    segs = contiguous_segments(mask, min_len=2, merge_gap=15)
    return [(s, e) for s, e in segs if e - s + 1 >= 4]


def metric_summary(df, obs, spread, events, results):
    t = df["t_s"].to_numpy(dtype=float)
    flight = (df["I1"].to_numpy(dtype=float) > 500.0) & (df["I16"].to_numpy(dtype=float) > 2500.0)
    landing = df["I1"].to_numpy(dtype=float) < 0.0
    rows = []

    logged = ReplayResult(
        "logged_current",
        df["I2"].to_numpy(dtype=float),
        df["I14"].to_numpy(dtype=float),
        df["I17"].to_numpy(dtype=float) > 0.5,
        df["I21"].to_numpy(dtype=float) > 0.5,
    )

    for res in [logged] + results:
        z = res.z_mm
        v = res.vz_mps
        valid = flight & np.isfinite(z)
        dz1 = np.diff(z)
        dz10 = z[10:] - z[:-10]
        flight10 = flight[10:] & flight[:-10] & np.isfinite(dz10)

        recover_times = []
        event_min = []
        event_max_drop = []
        for s, e in events:
            pre_s = max(0, s - 10)
            pre_e = max(0, s - 1)
            pre = float(np.nanmedian(z[pre_s : pre_e + 1]))
            if not np.isfinite(pre):
                continue
            seg_z = z[s : e + 1]
            event_min.append(float(np.nanmin(seg_z)))
            event_max_drop.append(float(pre - np.nanmin(seg_z)))
            rec = math.nan
            for j in range(e + 1, min(len(z), e + 401)):
                if np.isfinite(z[j]) and abs(z[j] - pre) <= 80.0 and abs(v[j]) <= 0.6:
                    rec = float(t[j] - t[e])
                    break
            recover_times.append(rec)

        land_valid = landing & np.isfinite(z)
        land_reject = landing & np.isfinite(obs) & (~res.accepted)

        rows.append(
            {
                "candidate": res.name,
                "flight_min_h_mm": float(np.nanmin(z[valid])),
                "flight_p01_h_mm": float(np.nanpercentile(z[valid], 1)),
                "cruise_min_h_mm": float(np.nanmin(z[valid & (df["t_s"].to_numpy(dtype=float) > 20.0) & (df["t_s"].to_numpy(dtype=float) < 280.0)])),
                "cruise_p01_h_mm": float(np.nanpercentile(z[valid & (df["t_s"].to_numpy(dtype=float) > 20.0) & (df["t_s"].to_numpy(dtype=float) < 280.0)], 1)),
                "max_1frame_jump_mm": float(np.nanmax(np.abs(dz1[flight[1:] & flight[:-1] & np.isfinite(dz1)]))),
                "max_100ms_drop_mm": float(-np.nanmin(dz10[flight10])),
                "vz_min_mps": float(np.nanmin(v[valid])),
                "vz_max_mps": float(np.nanmax(v[valid])),
                "vz_abs_p99_mps": float(np.nanpercentile(np.abs(v[valid]), 99)),
                "event_min_h_mm": float(np.nanmin(event_min)) if event_min else math.nan,
                "event_max_drop_mm": float(np.nanmax(event_max_drop)) if event_max_drop else math.nan,
                "event_recovery_p50_s": float(np.nanmedian(recover_times)) if recover_times else math.nan,
                "event_recovery_p90_s": float(np.nanpercentile(recover_times, 90)) if recover_times else math.nan,
                "event_unrecovered_count": int(sum(1 for x in recover_times if not np.isfinite(x))),
                "event_polluted_rows": int(sum(int(np.sum(res.polluted[s : e + 1])) for s, e in events)),
                "landing_final_h_mm": float(z[-1]) if np.isfinite(z[-1]) else math.nan,
                "landing_min_h_mm": float(np.nanmin(z[land_valid])) if np.any(land_valid) else math.nan,
                "landing_obs_reject_pct": float(np.sum(land_reject) * 100.0 / max(1, np.sum(landing & np.isfinite(obs)))),
            }
        )
    return pd.DataFrame(rows)


def event_summary(df, obs, spread, events):
    t = df["t_s"].to_numpy(dtype=float)
    rows = []
    for s, e in events:
        pre = max(0, s - 10)
        post = min(len(df) - 1, e + 10)
        seg = df.iloc[s : e + 1]
        rows.append(
            {
                "start_s": float(t[s]),
                "end_s": float(t[e]),
                "duration_s": float(t[e] - t[s] + DT_S),
                "pre_h_mm": float(np.median(df.iloc[pre:s]["I2"])) if s > pre else float(df.iloc[s]["I2"]),
                "min_logged_h_mm": float(seg["I2"].min()),
                "min_tof_obs_mm": float(np.nanmin(obs[s : e + 1])),
                "tof_spread_max_mm": float(np.nanmax(spread[s : e + 1])),
                "acc_up_min_mps2": float(seg["I12"].min()),
                "acc_up_max_mps2": float(seg["I12"].max()),
                "vz_min_mps": float(seg["I14"].min()),
                "pos_out_max_mps": float(seg["I13"].max()),
                "vel_out_max": float(seg["I15"].max()),
                "throttle_max": float(seg["I16"].max()),
                "current_polluted_rows": int((seg["I21"] > 0.5).sum()),
                "gate_reasons": "|".join(str(int(x)) for x in sorted(seg["I23"].unique())),
                "post_h_mm": float(df.iloc[post]["I2"]),
            }
        )
    return pd.DataFrame(rows)


def write_detail(df, obs, spread, results):
    out = pd.DataFrame(
        {
            "t_s": df["t_s"],
            "target_mm": df["I1"],
            "logged_h_mm": df["I2"],
            "tof_obs_mm": obs,
            "tof_spread_mm": spread,
            "acc_up_mps2": df["I12"],
            "logged_vz_mps": df["I14"],
            "logged_throttle": df["I16"],
            "logged_polluted": df["I21"],
            "logged_gate_reason": df["I23"],
        }
    )
    for res in results:
        out[f"{res.name}_h_mm"] = res.z_mm
        out[f"{res.name}_vz_mps"] = res.vz_mps
        out[f"{res.name}_accepted"] = res.accepted.astype(int)
        out[f"{res.name}_polluted"] = res.polluted.astype(int)
    out.to_csv(ROOT / "height_est_replay_0527_detail.csv", index=False)


def main():
    df = load_log()
    obs, spread, count, accept_count = build_tof_measure(df)
    events = detect_pollution_events(df, obs, spread)
    results = [
        replay_trimmed_lpf(df, obs),
        replay_huber_ab(df, obs),
        replay_physical_gate_ab(df, obs, spread),
        replay_pollution_fsm(df, obs, spread),
        replay_recommended_mcu(df, obs, spread),
        replay_ported_c_logic(df, obs, spread),
    ]
    events_df = event_summary(df, obs, spread, events)
    metrics_df = metric_summary(df, obs, spread, events, results)
    events_df.to_csv(ROOT / "height_est_replay_0527_events.csv", index=False)
    metrics_df.to_csv(ROOT / "height_est_replay_0527_metrics.csv", index=False)
    write_detail(df, obs, spread, results)

    print(f"log={LOG_PATH.name} rows={len(df)} events={len(events)}")
    print("events:")
    print(events_df.to_string(index=False, max_rows=50))
    print()
    print("metrics:")
    print(metrics_df.to_string(index=False))


if __name__ == "__main__":
    main()
