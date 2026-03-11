#!/usr/bin/env python3
"""better_vel_acc 数据集通用分析工具。"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

SAMPLE_RATE_HZ = 250.0

RIGHT_ACCEL_COLUMN = "I4"
FORWARD_ACCEL_COLUMN = "I3"
VERTICAL_ACCEL_COLUMN = "I5"
ROLL_COLUMN = "I6"
PITCH_COLUMN = "I7"
YAW_COLUMN = "I8"
FLOW_RIGHT_COLUMN = "I9"
FLOW_FORWARD_COLUMN = "I10"


@dataclass(frozen=True)
class Dataset:
    """better_vel_acc 解析结果。"""

    time_s: np.ndarray
    acc_forward_mps2: np.ndarray
    acc_right_mps2: np.ndarray
    acc_vertical_mps2: np.ndarray
    roll_deg: np.ndarray
    pitch_deg: np.ndarray
    yaw_deg: np.ndarray
    flow_right_cmps: np.ndarray
    flow_forward_cmps: np.ndarray


@dataclass(frozen=True)
class FilterSpec:
    """候选滤波器参数。"""

    label: str
    lpf_hz: float
    notch_hz: float | None = None
    notch_q: float = 0.0


@dataclass(frozen=True)
class ReplayResult:
    """融合回放结果。"""

    vel_right_cmps: np.ndarray
    vel_forward_cmps: np.ndarray
    innovation_right_cmps: np.ndarray
    innovation_forward_cmps: np.ndarray
    bias_right_cmpss: np.ndarray
    bias_forward_cmpss: np.ndarray
    acc_right_filt_cmpss: np.ndarray
    acc_forward_filt_cmpss: np.ndarray


def load_better_vel_acc(csv_path: Path, sample_rate_hz: float = SAMPLE_RATE_HZ) -> Dataset:
    """读取 better_vel_acc.csv。"""

    required_columns = (
        RIGHT_ACCEL_COLUMN,
        FORWARD_ACCEL_COLUMN,
        VERTICAL_ACCEL_COLUMN,
        ROLL_COLUMN,
        PITCH_COLUMN,
        YAW_COLUMN,
        FLOW_RIGHT_COLUMN,
        FLOW_FORWARD_COLUMN,
    )

    with csv_path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError("CSV 文件缺少表头。")

        missing_columns = [column for column in required_columns if column not in reader.fieldnames]
        if missing_columns:
            raise ValueError(f"CSV 文件缺少必要列: {', '.join(missing_columns)}")

        buffers = {column: [] for column in required_columns}
        for row_index, row in enumerate(reader, start=2):
            try:
                for column in required_columns:
                    buffers[column].append(float(row[column]))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"第 {row_index} 行存在非法浮点值。") from exc

    sample_count = len(buffers[RIGHT_ACCEL_COLUMN])
    time_s = np.arange(sample_count, dtype=np.float64) / float(sample_rate_hz)

    return Dataset(
        time_s=time_s,
        acc_forward_mps2=np.asarray(buffers[FORWARD_ACCEL_COLUMN], dtype=np.float64),
        acc_right_mps2=np.asarray(buffers[RIGHT_ACCEL_COLUMN], dtype=np.float64),
        acc_vertical_mps2=np.asarray(buffers[VERTICAL_ACCEL_COLUMN], dtype=np.float64),
        roll_deg=np.asarray(buffers[ROLL_COLUMN], dtype=np.float64),
        pitch_deg=np.asarray(buffers[PITCH_COLUMN], dtype=np.float64),
        yaw_deg=np.asarray(buffers[YAW_COLUMN], dtype=np.float64),
        flow_right_cmps=np.asarray(buffers[FLOW_RIGHT_COLUMN], dtype=np.float64),
        flow_forward_cmps=np.asarray(buffers[FLOW_FORWARD_COLUMN], dtype=np.float64),
    )


def compute_fft(signal: np.ndarray, sample_rate_hz: float) -> tuple[np.ndarray, np.ndarray]:
    """计算单边 FFT 幅值谱。"""

    centered = np.asarray(signal, dtype=np.float64) - float(np.mean(signal))
    window = np.hanning(centered.size)
    fft_values = np.fft.rfft(centered * window)
    freqs = np.fft.rfftfreq(centered.size, d=1.0 / sample_rate_hz)
    amplitude = np.abs(fft_values) / max(1.0, np.sum(window) * 0.5)
    return freqs, amplitude


def compute_welch_psd(
    signal: np.ndarray,
    sample_rate_hz: float,
    segment_length: int = 2048,
    overlap_ratio: float = 0.5,
) -> tuple[np.ndarray, np.ndarray]:
    """使用 Welch 方法估计功率谱密度。"""

    values = np.asarray(signal, dtype=np.float64)
    if values.size < 8:
        raise ValueError("信号长度过短，无法估计 PSD。")

    segment_length = max(256, min(segment_length, values.size))
    step = max(1, int(segment_length * (1.0 - overlap_ratio)))
    window = np.hanning(segment_length)
    window_power = np.sum(window * window)

    psd_accumulator: np.ndarray | None = None
    segment_count = 0

    for start in range(0, values.size - segment_length + 1, step):
        segment = values[start : start + segment_length]
        segment = segment - float(np.mean(segment))
        fft_values = np.fft.rfft(segment * window)
        psd = (np.abs(fft_values) ** 2) / (sample_rate_hz * window_power)
        if psd_accumulator is None:
            psd_accumulator = psd
        else:
            psd_accumulator += psd
        segment_count += 1

    if psd_accumulator is None:
        raise ValueError("Welch PSD 计算失败。")

    psd_accumulator /= float(segment_count)
    freqs = np.fft.rfftfreq(segment_length, d=1.0 / sample_rate_hz)
    return freqs, psd_accumulator


def band_power_fraction(freqs: np.ndarray, psd: np.ndarray, low_hz: float, high_hz: float) -> float:
    """计算指定频段的功率占比。"""

    band_mask = (freqs >= low_hz) & (freqs < high_hz)
    total_power = float(np.sum(psd))
    if total_power <= 0.0:
        return 0.0
    return float(np.sum(psd[band_mask])) / total_power


def cumulative_power_frequency(freqs: np.ndarray, psd: np.ndarray, percentile: float) -> float:
    """计算累计功率分位点频率。"""

    total_power = float(np.sum(psd))
    if total_power <= 0.0:
        return 0.0
    cumulative = np.cumsum(psd)
    index = int(np.searchsorted(cumulative, total_power * percentile))
    index = min(max(index, 0), len(freqs) - 1)
    return float(freqs[index])


def find_top_peaks(
    freqs: np.ndarray,
    psd: np.ndarray,
    count: int = 5,
    min_freq_hz: float = 0.1,
    min_separation_hz: float = 1.0,
) -> list[float]:
    """提取互相分离的主峰频率。"""

    candidate_indices = np.argsort(psd)[::-1]
    peaks: list[float] = []

    for index in candidate_indices:
        freq_hz = float(freqs[index])
        if freq_hz < min_freq_hz:
            continue
        if any(abs(freq_hz - peak_hz) < min_separation_hz for peak_hz in peaks):
            continue
        peaks.append(freq_hz)
        if len(peaks) >= count:
            break

    return peaks


def _init_lpf(sample_rate_hz: float, cutoff_hz: float) -> tuple[float, float, float, float, float]:
    w0 = 2.0 * math.pi * cutoff_hz / sample_rate_hz
    sw0 = math.sin(w0)
    cw0 = math.cos(w0)
    alpha = sw0 / (2.0 * 0.70710678)
    a0 = 1.0 + alpha
    b0 = (1.0 - cw0) * 0.5 / a0
    b1 = (1.0 - cw0) / a0
    b2 = (1.0 - cw0) * 0.5 / a0
    a1 = (-2.0 * cw0) / a0
    a2 = (1.0 - alpha) / a0
    return b0, b1, b2, a1, a2


def _init_notch(sample_rate_hz: float, center_hz: float, q: float) -> tuple[float, float, float, float, float]:
    w0 = 2.0 * math.pi * center_hz / sample_rate_hz
    sw0 = math.sin(w0)
    cw0 = math.cos(w0)
    alpha = sw0 / (2.0 * q)
    a0 = 1.0 + alpha
    b0 = 1.0 / a0
    b1 = (-2.0 * cw0) / a0
    b2 = 1.0 / a0
    a1 = (-2.0 * cw0) / a0
    a2 = (1.0 - alpha) / a0
    return b0, b1, b2, a1, a2


def _apply_biquad(signal: np.ndarray, coeffs: tuple[float, float, float, float, float]) -> np.ndarray:
    b0, b1, b2, a1, a2 = coeffs
    d1 = 0.0
    d2 = 0.0
    output = np.empty_like(signal, dtype=np.float64)

    for index, value in enumerate(signal):
        sample = b0 * value + d1
        d1 = b1 * value - a1 * sample + d2
        d2 = b2 * value - a2 * sample
        output[index] = sample

    return output


def apply_filter(signal: np.ndarray, spec: FilterSpec, sample_rate_hz: float = SAMPLE_RATE_HZ) -> np.ndarray:
    """按候选参数执行单通道实时滤波。"""

    filtered = np.asarray(signal, dtype=np.float64)
    if spec.notch_hz is not None:
        filtered = _apply_biquad(filtered, _init_notch(sample_rate_hz, spec.notch_hz, spec.notch_q))
    filtered = _apply_biquad(filtered, _init_lpf(sample_rate_hz, spec.lpf_hz))
    return filtered


def filter_frequency_response(spec: FilterSpec, sample_rate_hz: float, freqs_hz: np.ndarray) -> np.ndarray:
    """计算候选滤波器频响。"""

    z_inv = np.exp(-1j * 2.0 * np.pi * freqs_hz / sample_rate_hz)
    response = np.ones_like(freqs_hz, dtype=np.complex128)

    coeff_list: list[tuple[float, float, float, float, float]] = []
    if spec.notch_hz is not None:
        coeff_list.append(_init_notch(sample_rate_hz, spec.notch_hz, spec.notch_q))
    coeff_list.append(_init_lpf(sample_rate_hz, spec.lpf_hz))

    for b0, b1, b2, a1, a2 in coeff_list:
        numerator = b0 + b1 * z_inv + b2 * (z_inv ** 2)
        denominator = 1.0 + a1 * z_inv + a2 * (z_inv ** 2)
        response *= numerator / denominator

    return response


def attenuation_db_at(spec: FilterSpec, sample_rate_hz: float, freq_hz: float) -> float:
    """计算指定频率的幅值衰减。"""

    response = filter_frequency_response(spec, sample_rate_hz, np.asarray([freq_hz], dtype=np.float64))[0]
    magnitude = max(1.0e-12, abs(response))
    return 20.0 * math.log10(magnitude)


def group_delay_ms_at(spec: FilterSpec, sample_rate_hz: float, freq_hz: float) -> float:
    """数值估计指定频率附近的群时延。"""

    delta_hz = 0.05
    probe_freqs = np.asarray(
        [
            max(0.01, freq_hz - delta_hz),
            freq_hz,
            min(sample_rate_hz * 0.49, freq_hz + delta_hz),
        ],
        dtype=np.float64,
    )
    response = filter_frequency_response(spec, sample_rate_hz, probe_freqs)
    phase = np.unwrap(np.angle(response))
    omega = 2.0 * np.pi * probe_freqs / sample_rate_hz
    dphi = phase[2] - phase[0]
    domega = omega[2] - omega[0]
    if abs(domega) < 1.0e-12:
        return 0.0
    delay_samples = -dphi / domega
    delay_samples = max(0.0, delay_samples)
    return delay_samples * 1000.0 / sample_rate_hz


def hf_noise_ratio(signal: np.ndarray, sample_rate_hz: float = SAMPLE_RATE_HZ) -> float:
    """计算 60~125Hz / 0~10Hz 功率比。"""

    freqs, psd = compute_welch_psd(signal, sample_rate_hz)
    low_power = max(1.0e-12, float(np.sum(psd[(freqs >= 0.0) & (freqs < 10.0)])))
    high_power = float(np.sum(psd[(freqs >= 60.0) & (freqs <= 125.0)]))
    return high_power / low_power


def replay_complementary_filter(
    acc_right_mps2: np.ndarray,
    acc_forward_mps2: np.ndarray,
    flow_right_cmps: np.ndarray,
    flow_forward_cmps: np.ndarray,
    filter_spec: FilterSpec,
    sample_rate_hz: float = SAMPLE_RATE_HZ,
    flow_valid_mask: np.ndarray | None = None,
    w_flow_v: float = 2.0,
    w_flow_p: float = 1.0,
    w_acc_bias: float = 0.01,
    w_res_v: float = 0.5,
    dead_max_s: float = 0.30,
    recover_ramp_s: float = 0.20,
) -> ReplayResult:
    """回放 INAV 风格互补滤波。"""

    dt_s = 1.0 / sample_rate_hz
    if flow_valid_mask is None:
        flow_valid_mask = np.ones_like(flow_right_cmps, dtype=bool)

    acc_right_cmpss = apply_filter(acc_right_mps2 * 100.0, filter_spec, sample_rate_hz)
    acc_forward_cmpss = apply_filter(acc_forward_mps2 * 100.0, filter_spec, sample_rate_hz)

    vel_right = np.zeros_like(flow_right_cmps, dtype=np.float64)
    vel_forward = np.zeros_like(flow_forward_cmps, dtype=np.float64)
    pos_right = np.zeros_like(flow_right_cmps, dtype=np.float64)
    pos_forward = np.zeros_like(flow_forward_cmps, dtype=np.float64)
    innov_right = np.zeros_like(flow_right_cmps, dtype=np.float64)
    innov_forward = np.zeros_like(flow_forward_cmps, dtype=np.float64)
    bias_right = np.zeros_like(flow_right_cmps, dtype=np.float64)
    bias_forward = np.zeros_like(flow_forward_cmps, dtype=np.float64)

    flow_pos_right = 0.0
    flow_pos_forward = 0.0
    dead_time_s = 0.0
    recover_gain = 0.0
    flow_anchor_ready = False

    for index in range(len(flow_right_cmps)):
        acc_right_use = acc_right_cmpss[index] - (bias_right[index - 1] if index > 0 else 0.0)
        acc_forward_use = acc_forward_cmpss[index] - (bias_forward[index - 1] if index > 0 else 0.0)

        if index > 0:
            vel_right[index] = vel_right[index - 1] + acc_right_use * dt_s
            vel_forward[index] = vel_forward[index - 1] + acc_forward_use * dt_s
            pos_right[index] = pos_right[index - 1] + vel_right[index - 1] * dt_s + 0.5 * acc_right_use * dt_s * dt_s
            pos_forward[index] = pos_forward[index - 1] + vel_forward[index - 1] * dt_s + 0.5 * acc_forward_use * dt_s * dt_s

            bias_right[index] = bias_right[index - 1]
            bias_forward[index] = bias_forward[index - 1]

        if flow_valid_mask[index]:
            if not flow_anchor_ready:
                flow_pos_right = pos_right[index]
                flow_pos_forward = pos_forward[index]
                flow_anchor_ready = True

            dead_time_s = 0.0
            recover_gain = min(1.0, recover_gain + dt_s / recover_ramp_s)

            flow_pos_right += flow_right_cmps[index] * dt_s
            flow_pos_forward += flow_forward_cmps[index] * dt_s

            innov_right[index] = flow_right_cmps[index] - vel_right[index]
            innov_forward[index] = flow_forward_cmps[index] - vel_forward[index]

            pos_right[index] += (flow_pos_right - pos_right[index]) * w_flow_p * recover_gain * dt_s
            pos_forward[index] += (flow_pos_forward - pos_forward[index]) * w_flow_p * recover_gain * dt_s
            vel_right[index] += innov_right[index] * w_flow_v * recover_gain * dt_s
            vel_forward[index] += innov_forward[index] * w_flow_v * recover_gain * dt_s

            bias_right[index] += innov_right[index] * w_acc_bias * dt_s
            bias_forward[index] += innov_forward[index] * w_acc_bias * dt_s
        else:
            dead_time_s += dt_s
            recover_gain = 0.0
            flow_anchor_ready = dead_time_s <= dead_max_s
            if dead_time_s > dead_max_s:
                decay_factor = max(0.0, 1.0 - w_res_v * dt_s)
                vel_right[index] *= decay_factor
                vel_forward[index] *= decay_factor

    return ReplayResult(
        vel_right_cmps=vel_right,
        vel_forward_cmps=vel_forward,
        innovation_right_cmps=innov_right,
        innovation_forward_cmps=innov_forward,
        bias_right_cmpss=bias_right,
        bias_forward_cmpss=bias_forward,
        acc_right_filt_cmpss=acc_right_cmpss,
        acc_forward_filt_cmpss=acc_forward_cmpss,
    )


def replay_ekf(
    acc_right_mps2: np.ndarray,
    acc_forward_mps2: np.ndarray,
    flow_right_cmps: np.ndarray,
    flow_forward_cmps: np.ndarray,
    filter_spec: FilterSpec,
    sample_rate_hz: float = SAMPLE_RATE_HZ,
    flow_valid_mask: np.ndarray | None = None,
) -> ReplayResult:
    """回放轻量 4 状态 EKF。"""

    dt_s = 1.0 / sample_rate_hz
    if flow_valid_mask is None:
        flow_valid_mask = np.ones_like(flow_right_cmps, dtype=bool)

    acc_right_cmpss = apply_filter(acc_right_mps2 * 100.0, filter_spec, sample_rate_hz)
    acc_forward_cmpss = apply_filter(acc_forward_mps2 * 100.0, filter_spec, sample_rate_hz)

    vel_right = np.zeros_like(flow_right_cmps, dtype=np.float64)
    vel_forward = np.zeros_like(flow_forward_cmps, dtype=np.float64)
    innov_right = np.zeros_like(flow_right_cmps, dtype=np.float64)
    innov_forward = np.zeros_like(flow_forward_cmps, dtype=np.float64)
    bias_right = np.zeros_like(flow_right_cmps, dtype=np.float64)
    bias_forward = np.zeros_like(flow_forward_cmps, dtype=np.float64)

    state = np.zeros((4, 1), dtype=np.float64)
    covariance = np.diag([200.0, 200.0, 25.0, 25.0]).astype(np.float64)

    accel_var = float(np.var(np.concatenate([acc_right_cmpss, acc_forward_cmpss])))
    q_v = max(1.0, accel_var * dt_s * dt_s * 0.05)
    q_b = 0.02
    process_noise = np.diag([q_v, q_v, q_b, q_b]).astype(np.float64)

    flow_var = float(
        np.var(
            np.concatenate(
                [
                    flow_right_cmps - np.mean(flow_right_cmps),
                    flow_forward_cmps - np.mean(flow_forward_cmps),
                ]
            )
        )
    )
    measurement_noise = np.diag([max(4.0, flow_var * 0.15), max(4.0, flow_var * 0.15)]).astype(np.float64)

    transition = np.array(
        [
            [1.0, 0.0, -dt_s, 0.0],
            [0.0, 1.0, 0.0, -dt_s],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    control = np.array(
        [
            [dt_s, 0.0],
            [0.0, dt_s],
            [0.0, 0.0],
            [0.0, 0.0],
        ],
        dtype=np.float64,
    )
    observe = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )

    for index in range(len(flow_right_cmps)):
        control_input = np.array([[acc_right_cmpss[index]], [acc_forward_cmpss[index]]], dtype=np.float64)
        state = transition @ state + control @ control_input
        covariance = transition @ covariance @ transition.T + process_noise

        if flow_valid_mask[index]:
            measurement = np.array([[flow_right_cmps[index]], [flow_forward_cmps[index]]], dtype=np.float64)
            innovation = measurement - observe @ state
            innovation_cov = observe @ covariance @ observe.T + measurement_noise
            kalman_gain = covariance @ observe.T @ np.linalg.inv(innovation_cov)
            state = state + kalman_gain @ innovation
            covariance = (np.eye(4, dtype=np.float64) - kalman_gain @ observe) @ covariance
            innov_right[index] = innovation[0, 0]
            innov_forward[index] = innovation[1, 0]
        else:
            innov_right[index] = 0.0
            innov_forward[index] = 0.0

        vel_right[index] = state[0, 0]
        vel_forward[index] = state[1, 0]
        bias_right[index] = state[2, 0]
        bias_forward[index] = state[3, 0]

    return ReplayResult(
        vel_right_cmps=vel_right,
        vel_forward_cmps=vel_forward,
        innovation_right_cmps=innov_right,
        innovation_forward_cmps=innov_forward,
        bias_right_cmpss=bias_right,
        bias_forward_cmpss=bias_forward,
        acc_right_filt_cmpss=acc_right_cmpss,
        acc_forward_filt_cmpss=acc_forward_cmpss,
    )


def rms(signal: np.ndarray) -> float:
    """计算 RMS。"""

    values = np.asarray(signal, dtype=np.float64)
    return float(np.sqrt(np.mean(values * values)))


def settle_time_s(
    diff_right_cmps: np.ndarray,
    diff_forward_cmps: np.ndarray,
    sample_rate_hz: float,
    start_index: int,
    tolerance_cmps: float = 5.0,
    stable_window_s: float = 0.20,
) -> float:
    """计算恢复到容差范围内的稳定时间。"""

    stable_samples = max(1, int(round(stable_window_s * sample_rate_hz)))
    right_abs = np.abs(diff_right_cmps)
    forward_abs = np.abs(diff_forward_cmps)

    for index in range(start_index, len(diff_right_cmps) - stable_samples + 1):
        if (
            np.all(right_abs[index : index + stable_samples] <= tolerance_cmps)
            and np.all(forward_abs[index : index + stable_samples] <= tolerance_cmps)
        ):
            return (index - start_index) / sample_rate_hz

    return (len(diff_right_cmps) - start_index) / sample_rate_hz


def iter_default_filter_specs() -> Iterable[FilterSpec]:
    """枚举计划中约定的候选滤波器。"""

    for cutoff_hz in (10.0, 12.0, 15.0, 18.0):
        yield FilterSpec(label=f"LPF{cutoff_hz:.0f}", lpf_hz=cutoff_hz)
        for notch_hz in (85.0, 100.0):
            for notch_q in (3.0, 4.0, 5.0):
                yield FilterSpec(
                    label=f"Notch{notch_hz:.0f}Q{notch_q:.0f}+LPF{cutoff_hz:.0f}",
                    notch_hz=notch_hz,
                    notch_q=notch_q,
                    lpf_hz=cutoff_hz,
                )
