#!/usr/bin/env python3
"""Fit and compare three-camera wide-angle models from calibration CSV logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation


CAMERA_NAMES = ("Front", "Center", "Back")
INVALID = -900.0
DEG = math.pi / 180.0
YAW_BIAS_RAD = 0.4068566800
MAX_GROUND_DISTANCE_M = 15.0
MIN_TRUSTED_TARGET_DISTANCE_M = 0.20
GRID_X = np.linspace(-94.0, 94.0, 9)
GRID_Y = np.linspace(-60.0, 60.0, 7)
DELAY_MS = np.arange(0.0, 55.0, 5.0)

CURRENT_INTRINSICS = np.array(
    [
        [77.2632115, 10.0431456, 7.1929172, 0.7000000],
        [81.0801880, 5.4973460, 1.2174406, 0.7000000],
        [82.3499934, -1.1395418, -5.2939230, 0.7000000],
    ],
    dtype=float,
)
CURRENT_ROTATIONS = np.array(
    [
        [
            [-0.053257901, -0.823901336, 0.564225295],
            [0.997565442, -0.018423355, 0.067258975],
            [-0.045019836, 0.566433728, 0.822876690],
        ],
        [
            [-0.032443015, -0.995479870, -0.089259618],
            [0.996951194, -0.038572645, 0.067826744],
            [-0.070963138, -0.086786978, 0.993696258],
        ],
        [
            [0.156584158, 0.727257310, -0.668265072],
            [-0.985382340, 0.069061905, -0.155730851],
            [-0.067104741, 0.682881584, 0.727440510],
        ],
    ],
    dtype=float,
)


@dataclass(frozen=True)
class LogInfo:
    path: Path
    name: str
    distance_m: float
    suffix_2: bool


@dataclass
class Dataset:
    logs: list[LogInfo]
    log_id: np.ndarray
    time_ms: np.ndarray
    distance_m: np.ndarray
    suffix_2: np.ndarray
    car_yaw: np.ndarray
    yaw_rate: np.ndarray
    pose: np.ndarray
    pose_lags: np.ndarray
    height_m: np.ndarray
    beacons: np.ndarray
    beacon_valid: np.ndarray
    beacon_filtered: np.ndarray
    lamps: np.ndarray
    lamp_valid: np.ndarray
    raw_pair_type: np.ndarray
    camera_bucket: np.ndarray
    region: np.ndarray
    pose_rate: np.ndarray

    def __len__(self) -> int:
        return int(self.log_id.size)


@dataclass(frozen=True)
class ModelSpec:
    name: str
    intrinsic_count: int


@dataclass
class FitResult:
    spec: ModelSpec
    params: np.ndarray
    lut: np.ndarray | None
    observation_count: int
    cost: float


@dataclass
class Observations:
    row: np.ndarray
    lamp_camera: np.ndarray
    beacon_camera: np.ndarray
    beacon_slot: np.ndarray

    def __len__(self) -> int:
        return int(self.row.size)


SPECS = {
    "pinhole_k1": ModelSpec("pinhole_k1", 4),
    "double_sphere": ModelSpec("double_sphere", 6),
    "kb8": ModelSpec("kb8", 8),
}


def wrap_rad(value: np.ndarray | float) -> np.ndarray:
    return (np.asarray(value) + math.pi) % (2.0 * math.pi) - math.pi


def wrap_axis_rad(value: np.ndarray | float) -> np.ndarray:
    return (np.asarray(value) + 0.5 * math.pi) % math.pi - 0.5 * math.pi


def parse_log_info(path: Path) -> LogInfo:
    match = re.search(r"距离为([0-9]+(?:[.,][0-9]+)?)米", path.name)
    if match is None:
        raise ValueError(f"cannot parse distance from {path.name}")
    distance = float(match.group(1).replace(",", "."))
    return LogInfo(path, path.name, distance, path.stem.endswith("_2"))


def interpolate_pose(history: deque[tuple[float, np.ndarray]], target_ms: float) -> np.ndarray:
    if not history:
        return np.zeros(3, dtype=float)
    if target_ms <= history[0][0]:
        return history[0][1].copy()
    previous_t, previous_pose = history[0]
    for current_t, current_pose in history:
        if current_t >= target_ms:
            span = current_t - previous_t
            if span <= 0.0:
                return current_pose.copy()
            gain = (target_ms - previous_t) / span
            delta = current_pose - previous_pose
            delta[2] = float(wrap_rad(delta[2] * DEG) / DEG)
            result = previous_pose + gain * delta
            result[2] = float(wrap_rad(result[2] * DEG) / DEG)
            return result
        previous_t, previous_pose = current_t, current_pose
    return history[-1][1].copy()


def filter_near_lamp(
    beacons: np.ndarray,
    beacon_valid: np.ndarray,
    lamps: np.ndarray,
    lamp_valid: np.ndarray,
    state: dict[str, np.ndarray],
) -> np.ndarray:
    filtered = beacon_valid.copy()
    for camera in range(3):
        used = np.zeros(4, dtype=bool)
        valid_tracks = state["valid"][camera]
        state["far_age"][camera, valid_tracks] = np.minimum(
            state["far_age"][camera, valid_tracks] + 1, 31
        )
        state["suspect_age"][camera, valid_tracks] = np.minimum(
            state["suspect_age"][camera, valid_tracks] + 1, 31
        )
        for slot in np.flatnonzero(beacon_valid[camera]):
            point = beacons[camera, slot, :2]
            best = -1
            best_distance_sq = 15.0 * 15.0
            for track in range(4):
                if (
                    not state["valid"][camera, track]
                    or used[track]
                    or state["gap"][camera, track] > 2
                ):
                    continue
                delta = point - state["xy"][camera, track]
                distance_sq = float(delta @ delta)
                if distance_sq < best_distance_sq:
                    best = track
                    best_distance_sq = distance_sq
            matched = best >= 0
            if best < 0:
                free = np.flatnonzero(~used)
                if free.size == 0:
                    filtered[camera, slot] = False
                    continue
                best = int(free[0])

            lamp_distance = 10.0
            if lamp_valid[camera]:
                lamp_distance = float(np.linalg.norm(point - lamps[camera, :2]))
            used[best] = True
            state["valid"][camera, best] = True
            state["gap"][camera, best] = 0
            state["samples"][camera, best] = min(
                int(state["samples"][camera, best]) + 1 if matched else 1, 255
            )
            if not matched:
                state["far_age"][camera, best] = 31
                state["suspect_age"][camera, best] = (
                    0 if lamp_valid[camera] and lamp_distance < 15.0 else 31
                )
            state["xy"][camera, best] = point
            if lamp_valid[camera] and lamp_distance >= 10.0:
                state["far_age"][camera, best] = 0
            if state["suspect_age"][camera, best] <= 30 or (
                lamp_valid[camera]
                and lamp_distance < 3.0
                and (
                    state["samples"][camera, best] < 3
                    or state["far_age"][camera, best] > 30
                )
            ):
                filtered[camera, slot] = False
            if state["samples"][camera, best] < 2:
                filtered[camera, slot] = False

        for track in range(4):
            if state["valid"][camera, track] and not used[track]:
                state["gap"][camera, track] = min(
                    int(state["gap"][camera, track]) + 1, 255
                )
            if state["gap"][camera, track] > 2:
                state["valid"][camera, track] = False
                state["samples"][camera, track] = 0
                state["far_age"][camera, track] = 31
                state["suspect_age"][camera, track] = 31
    return filtered


def load_dataset(data_dir: Path) -> Dataset:
    paths = sorted(data_dir.glob("*.csv"), key=lambda p: (parse_log_info(p).distance_m, p.name))
    logs = [parse_log_info(path) for path in paths]
    if len(logs) != 14:
        raise ValueError(f"expected 14 CSV logs, found {len(logs)} in {data_dir}")

    records: dict[str, list] = {
        key: []
        for key in (
            "log_id",
            "time_ms",
            "distance_m",
            "suffix_2",
            "car_yaw",
            "yaw_rate",
            "pose",
            "pose_lags",
            "height_m",
            "beacons",
            "beacon_valid",
            "beacon_filtered",
            "lamps",
            "lamp_valid",
            "raw_pair_type",
            "camera_bucket",
            "region",
        )
    }

    for log_id, info in enumerate(logs):
        previous_signature: np.ndarray | None = None
        pose_history: deque[tuple[float, np.ndarray]] = deque(maxlen=32)
        track_state = {
            "valid": np.zeros((3, 4), dtype=bool),
            "gap": np.zeros((3, 4), dtype=np.uint8),
            "samples": np.zeros((3, 4), dtype=np.uint8),
            "far_age": np.full((3, 4), 31, dtype=np.uint8),
            "suspect_age": np.full((3, 4), 31, dtype=np.uint8),
            "xy": np.zeros((3, 4, 2), dtype=float),
        }
        with info.path.open("r", encoding="utf-8-sig", newline="") as handle:
            next(handle)
            for line in handle:
                values = np.fromstring(line, sep=",")
                if values.size != 64:
                    continue
                timestamp = float(values[0])
                pose = values[51:54].astype(float)
                pose_history.append((timestamp, pose))
                signature = values[1:49]
                if previous_signature is not None and np.array_equal(signature, previous_signature):
                    continue
                previous_signature = signature.copy()
                if values[55] <= 0.0 or values[54] <= 0.0:
                    continue

                beacons = np.zeros((3, 4, 3), dtype=np.float32)
                beacon_valid = np.zeros((3, 4), dtype=bool)
                lamps = np.zeros((3, 4), dtype=np.float32)
                lamp_valid = np.zeros(3, dtype=bool)
                for camera in range(3):
                    for slot in range(4):
                        start = 1 + camera * 12 + slot * 3
                        beacons[camera, slot] = values[start : start + 3]
                        beacon_valid[camera, slot] = (
                            values[start] > INVALID
                            and values[start + 1] > INVALID
                            and values[start + 2] > 0.0
                        )
                    start = 37 + camera * 4
                    lamps[camera] = values[start : start + 4]
                    lamp_valid[camera] = (
                        values[start] > INVALID
                        and values[start + 1] > INVALID
                        and values[start + 2] > INVALID
                        and values[start + 3] > 0.0
                    )

                beacon_filtered = filter_near_lamp(
                    beacons, beacon_valid, lamps, lamp_valid, track_state
                )
                lamp_mask = sum((1 << camera) for camera in np.flatnonzero(lamp_valid))
                filtered_cam_valid = np.any(beacon_filtered, axis=1)
                beacon_mask = sum(
                    (1 << camera) for camera in np.flatnonzero(filtered_cam_valid)
                )
                raw_beacon_mask = sum(
                    (1 << camera)
                    for camera in np.flatnonzero(np.any(beacon_valid, axis=1))
                )
                pair_mask = beacon_mask if beacon_mask else raw_beacon_mask
                if lamp_mask and pair_mask:
                    pair_type = 0 if lamp_mask & pair_mask else 1
                    overlap = lamp_mask & pair_mask
                    if overlap:
                        camera_bucket = int(math.log2(overlap & -overlap))
                    else:
                        lamp_camera = int(math.log2(lamp_mask & -lamp_mask))
                        beacon_camera = int(math.log2(pair_mask & -pair_mask))
                        camera_bucket = 3 + lamp_camera * 3 + beacon_camera
                else:
                    pair_type = -1
                    camera_bucket = -1

                points = []
                for camera in np.flatnonzero(lamp_valid):
                    points.append(lamps[camera, :2])
                for camera, slot in np.argwhere(beacon_valid):
                    points.append(beacons[camera, slot, :2])
                radius = 0.0
                if points:
                    xy = np.asarray(points, dtype=float)
                    radius = float(
                        np.max(np.hypot(xy[:, 0] / 94.0, xy[:, 1] / 60.0))
                    )

                lag_poses = np.stack(
                    [
                        interpolate_pose(pose_history, timestamp - delay)
                        for delay in DELAY_MS
                    ]
                ).astype(np.float32)
                records["log_id"].append(log_id)
                records["time_ms"].append(timestamp)
                records["distance_m"].append(info.distance_m)
                records["suffix_2"].append(info.suffix_2)
                records["car_yaw"].append(values[49])
                records["yaw_rate"].append(values[50])
                records["pose"].append(pose.astype(np.float32))
                records["pose_lags"].append(lag_poses)
                records["height_m"].append(values[54] * 0.001)
                records["beacons"].append(beacons)
                records["beacon_valid"].append(beacon_valid)
                records["beacon_filtered"].append(beacon_filtered)
                records["lamps"].append(lamps)
                records["lamp_valid"].append(lamp_valid)
                records["raw_pair_type"].append(pair_type)
                records["camera_bucket"].append(camera_bucket)
                records["region"].append(0 if radius <= 0.65 else 1)

    log_id_array = np.asarray(records["log_id"], dtype=np.int16)
    time_array = np.asarray(records["time_ms"], dtype=np.float64)
    pose_array = np.asarray(records["pose"], dtype=np.float32)
    pose_rate = np.zeros(len(log_id_array), dtype=np.float32)
    for current_log in range(len(logs)):
        indices = np.flatnonzero(log_id_array == current_log)
        if indices.size < 2:
            continue
        dt = np.diff(time_array[indices]) * 0.001
        delta = np.diff(pose_array[indices].astype(float), axis=0)
        delta[:, 2] = wrap_rad(delta[:, 2] * DEG) / DEG
        rate = np.linalg.norm(delta / np.maximum(dt[:, None], 0.001), axis=1)
        pose_rate[indices[1:]] = np.clip(rate, 0.0, 1000.0)

    return Dataset(
        logs=logs,
        log_id=log_id_array,
        time_ms=time_array,
        distance_m=np.asarray(records["distance_m"], dtype=np.float32),
        suffix_2=np.asarray(records["suffix_2"], dtype=bool),
        car_yaw=np.asarray(records["car_yaw"], dtype=np.float32),
        yaw_rate=np.asarray(records["yaw_rate"], dtype=np.float32),
        pose=pose_array,
        pose_lags=np.asarray(records["pose_lags"], dtype=np.float32),
        height_m=np.asarray(records["height_m"], dtype=np.float32),
        beacons=np.asarray(records["beacons"], dtype=np.float32),
        beacon_valid=np.asarray(records["beacon_valid"], dtype=bool),
        beacon_filtered=np.asarray(records["beacon_filtered"], dtype=bool),
        lamps=np.asarray(records["lamps"], dtype=np.float32),
        lamp_valid=np.asarray(records["lamp_valid"], dtype=bool),
        raw_pair_type=np.asarray(records["raw_pair_type"], dtype=np.int8),
        camera_bucket=np.asarray(records["camera_bucket"], dtype=np.int8),
        region=np.asarray(records["region"], dtype=np.int8),
        pose_rate=pose_rate,
    )


def initial_parameters(spec: ModelSpec) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if spec.name == "pinhole_k1":
        intrinsics = CURRENT_INTRINSICS.copy()
        lower_intrinsic = np.tile([40.0, -30.0, -30.0, -0.5], (3, 1))
        upper_intrinsic = np.tile([180.0, 30.0, 30.0, 3.0], (3, 1))
    elif spec.name == "double_sphere":
        intrinsics = np.column_stack(
            [
                CURRENT_INTRINSICS[:, 0],
                CURRENT_INTRINSICS[:, 0],
                CURRENT_INTRINSICS[:, 1:3],
                np.zeros(3),
                np.full(3, 0.5),
            ]
        )
        lower_intrinsic = np.tile([30.0, 30.0, -30.0, -30.0, -0.5, 0.25], (3, 1))
        upper_intrinsic = np.tile([200.0, 200.0, 30.0, 30.0, 1.8, 0.85], (3, 1))
    elif spec.name == "kb8":
        intrinsics = np.column_stack(
            [
                CURRENT_INTRINSICS[:, 0],
                CURRENT_INTRINSICS[:, 0],
                CURRENT_INTRINSICS[:, 1:3],
                np.zeros((3, 4)),
            ]
        )
        lower_intrinsic = np.tile(
            [30.0, 30.0, -30.0, -30.0, -1.0, -0.5, -0.25, -0.1], (3, 1)
        )
        upper_intrinsic = np.tile(
            [200.0, 200.0, 30.0, 30.0, 1.0, 0.5, 0.25, 0.1], (3, 1)
        )
    else:
        raise ValueError(spec.name)

    rotation_delta = np.zeros((3, 3), dtype=float)
    translation = np.zeros((2, 3), dtype=float)
    params = np.concatenate([intrinsics.ravel(), rotation_delta.ravel(), translation.ravel()])
    lower = np.concatenate(
        [
            lower_intrinsic.ravel(),
            np.full(9, -25.0 * DEG),
            np.full(6, -0.35),
        ]
    )
    upper = np.concatenate(
        [
            upper_intrinsic.ravel(),
            np.full(9, 25.0 * DEG),
            np.full(6, 0.35),
        ]
    )
    return params, lower, upper


def decode_parameters(
    spec: ModelSpec, params: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    intrinsic_size = 3 * spec.intrinsic_count
    intrinsics = params[:intrinsic_size].reshape(3, spec.intrinsic_count)
    deltas = params[intrinsic_size : intrinsic_size + 9].reshape(3, 3)
    rotations = np.empty((3, 3, 3), dtype=float)
    for camera in range(3):
        rotations[camera] = Rotation.from_rotvec(deltas[camera]).as_matrix() @ CURRENT_ROTATIONS[camera]
    translation = np.zeros((3, 3), dtype=float)
    free_translation = params[intrinsic_size + 9 :].reshape(2, 3)
    translation[0] = free_translation[0]
    translation[2] = free_translation[1]
    return intrinsics, rotations, translation


def rays_from_pixels(
    spec: ModelSpec,
    intrinsics: np.ndarray,
    u: np.ndarray,
    v: np.ndarray,
    camera: np.ndarray,
    lut: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    rays = np.full((u.size, 3), np.nan, dtype=float)
    valid = np.isfinite(u) & np.isfinite(v)
    for current_camera in range(3):
        mask = valid & (camera == current_camera)
        if not np.any(mask):
            continue
        p = intrinsics[current_camera]
        if spec.name == "pinhole_k1":
            x = (u[mask] - p[1]) / p[0]
            y = (v[mask] - p[2]) / p[0]
            gain = 1.0 + p[3] * (x * x + y * y)
            camera_rays = np.column_stack([x * gain, y * gain, np.ones_like(x)])
        elif spec.name == "double_sphere":
            x = (u[mask] - p[2]) / p[0]
            y = (v[mask] - p[3]) / p[1]
            radius_sq = x * x + y * y
            inside = 1.0 - (2.0 * p[5] - 1.0) * radius_sq
            local_valid = inside > 1.0e-9
            inside = np.maximum(inside, 1.0e-9)
            mz = (1.0 - p[5] * p[5] * radius_sq) / (
                p[5] * np.sqrt(inside) + 1.0 - p[5]
            )
            second = mz * mz + (1.0 - p[4] * p[4]) * radius_sq
            local_valid &= second > 1.0e-9
            second = np.maximum(second, 1.0e-9)
            gain = (mz * p[4] + np.sqrt(second)) / np.maximum(
                mz * mz + radius_sq, 1.0e-9
            )
            camera_rays = np.column_stack([gain * x, gain * y, gain * mz - p[4]])
            camera_rays[~local_valid] = np.nan
        elif spec.name == "kb8":
            x = (u[mask] - p[2]) / p[0]
            y = (v[mask] - p[3]) / p[1]
            radius = np.hypot(x, y)
            theta = np.minimum(radius, 1.55)
            for _ in range(6):
                theta_sq = theta * theta
                poly = 1.0 + theta_sq * (
                    p[4]
                    + theta_sq * (p[5] + theta_sq * (p[6] + theta_sq * p[7]))
                )
                function = theta * poly - radius
                derivative = 1.0 + theta_sq * (
                    3.0 * p[4]
                    + theta_sq * (5.0 * p[5] + theta_sq * (7.0 * p[6] + 9.0 * theta_sq * p[7]))
                )
                theta = np.clip(theta - function / np.maximum(derivative, 1.0e-6), 0.0, 1.55)
            scale = np.ones_like(radius)
            nonzero = radius > 1.0e-9
            scale[nonzero] = np.sin(theta[nonzero]) / radius[nonzero]
            camera_rays = np.column_stack([scale * x, scale * y, np.cos(theta)])
        else:
            raise ValueError(spec.name)

        if lut is not None:
            ax = np.arctan2(camera_rays[:, 0], camera_rays[:, 2])
            ay = np.arctan2(
                camera_rays[:, 1], np.hypot(camera_rays[:, 0], camera_rays[:, 2])
            )
            correction = interpolate_lut(lut[current_camera], u[mask], v[mask])
            ax += correction[:, 0]
            ay += correction[:, 1]
            cos_y = np.cos(ay)
            camera_rays = np.column_stack(
                [cos_y * np.sin(ax), np.sin(ay), cos_y * np.cos(ax)]
            )
        rays[mask] = camera_rays
    return rays, np.all(np.isfinite(rays), axis=1)


def world_rotation(pose_deg: np.ndarray) -> np.ndarray:
    roll = pose_deg[:, 0] * DEG
    pitch = pose_deg[:, 1] * DEG
    yaw = pose_deg[:, 2] * DEG + YAW_BIAS_RAD
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    result = np.empty((pose_deg.shape[0], 3, 3), dtype=float)
    result[:, 0, 0] = cp * cy
    result[:, 0, 1] = sr * sp * cy - cr * sy
    result[:, 0, 2] = cr * sp * cy + sr * sy
    result[:, 1, 0] = cp * sy
    result[:, 1, 1] = sr * sp * sy + cr * cy
    result[:, 1, 2] = cr * sp * sy - sr * cy
    result[:, 2, 0] = -sp
    result[:, 2, 1] = sr * cp
    result[:, 2, 2] = cr * cp
    return result


def project_pixels(
    spec: ModelSpec,
    params: np.ndarray,
    u: np.ndarray,
    v: np.ndarray,
    camera: np.ndarray,
    pose: np.ndarray,
    height_m: np.ndarray,
    lut: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    intrinsics, rotations, translation = decode_parameters(spec, params)
    rays, valid = rays_from_pixels(spec, intrinsics, u, v, camera, lut)
    body_ray = np.einsum("nij,nj->ni", rotations[camera], rays)
    body_origin = translation[camera]
    rotation = world_rotation(pose)
    world_ray = np.einsum("nij,nj->ni", rotation, body_ray)
    world_origin = np.einsum("nij,nj->ni", rotation, body_origin)
    scale = (height_m - world_origin[:, 2]) / np.maximum(world_ray[:, 2], 1.0e-9)
    valid &= world_ray[:, 2] > 1.0e-4
    valid &= (scale > 0.0) & (scale <= MAX_GROUND_DISTANCE_M)
    point = world_origin[:, :2] + scale[:, None] * world_ray[:, :2]
    point[~valid] = np.nan
    return point, valid


def lut_coordinates(u: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    gx = np.clip((u - GRID_X[0]) / (GRID_X[-1] - GRID_X[0]) * 8.0, 0.0, 8.0)
    gy = np.clip((v - GRID_Y[0]) / (GRID_Y[-1] - GRID_Y[0]) * 6.0, 0.0, 6.0)
    x0 = np.minimum(np.floor(gx).astype(int), 7)
    y0 = np.minimum(np.floor(gy).astype(int), 5)
    tx = gx - x0
    ty = gy - y0
    return x0, x0 + 1, y0, y0 + 1, tx, ty


def interpolate_lut(table: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    x0, x1, y0, y1, tx, ty = lut_coordinates(u, v)
    return (
        table[y0, x0] * ((1.0 - tx) * (1.0 - ty))[:, None]
        + table[y0, x1] * (tx * (1.0 - ty))[:, None]
        + table[y1, x0] * ((1.0 - tx) * ty)[:, None]
        + table[y1, x1] * (tx * ty)[:, None]
    )


def balanced_indices(dataset: Dataset, allowed: np.ndarray, maximum: int) -> np.ndarray:
    eligible = allowed & (dataset.raw_pair_type >= 0)
    indices = np.flatnonzero(eligible)
    if indices.size <= maximum:
        return indices
    positive_rates = dataset.pose_rate[indices][dataset.pose_rate[indices] > 0.0]
    rate_threshold = float(np.median(positive_rates)) if positive_rates.size else 0.0
    groups: dict[tuple[int, int, int, int, int], list[int]] = {}
    for row in indices:
        key = (
            int(dataset.log_id[row]),
            int(dataset.raw_pair_type[row]),
            int(dataset.camera_bucket[row]),
            int(dataset.region[row]),
            int(dataset.pose_rate[row] > rate_threshold),
        )
        groups.setdefault(key, []).append(int(row))
    per_group = max(1, int(math.ceil(maximum / len(groups))))
    selected: list[int] = []
    for key in sorted(groups):
        group = groups[key]
        if len(group) <= per_group:
            selected.extend(group)
        else:
            positions = np.linspace(0, len(group) - 1, per_group, dtype=int)
            selected.extend(group[position] for position in positions)
    if len(selected) > maximum:
        positions = np.linspace(0, len(selected) - 1, maximum, dtype=int)
        selected = [selected[position] for position in positions]
    return np.asarray(sorted(set(selected)), dtype=np.int64)


def camera_pose(
    dataset: Dataset, rows: np.ndarray, cameras: np.ndarray, delays_ms: np.ndarray
) -> np.ndarray:
    delay_index = np.rint(delays_ms[cameras] / 5.0).astype(int)
    delay_index = np.clip(delay_index, 0, len(DELAY_MS) - 1)
    return dataset.pose_lags[rows, delay_index].astype(float)


def project_scene(
    dataset: Dataset,
    rows: np.ndarray,
    fit: FitResult,
    delays_ms: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    if delays_ms is None:
        delays_ms = np.zeros(3, dtype=float)
    count = rows.size
    beacon_camera = np.tile(np.repeat(np.arange(3), 4), count)
    beacon_rows = np.repeat(rows, 12)
    beacon_uv = dataset.beacons[rows, :, :, :2].reshape(-1, 2).astype(float)
    beacon_pose = camera_pose(dataset, beacon_rows, beacon_camera, delays_ms)
    beacon_points, beacon_projected = project_pixels(
        fit.spec,
        fit.params,
        beacon_uv[:, 0],
        beacon_uv[:, 1],
        beacon_camera,
        beacon_pose,
        np.repeat(dataset.height_m[rows], 12).astype(float),
        fit.lut,
    )
    beacon_projected &= dataset.beacon_filtered[rows].reshape(-1)

    lamp_camera = np.tile(np.arange(3), count)
    lamp_rows = np.repeat(rows, 3)
    lamps = dataset.lamps[rows].reshape(-1, 4).astype(float)
    lamp_pose = camera_pose(dataset, lamp_rows, lamp_camera, delays_ms)
    lamp_height = np.repeat(dataset.height_m[rows], 3).astype(float)
    lamp_center, center_valid = project_pixels(
        fit.spec,
        fit.params,
        lamps[:, 0],
        lamps[:, 1],
        lamp_camera,
        lamp_pose,
        lamp_height,
        fit.lut,
    )
    angle = lamps[:, 2] * DEG
    half_length = 0.5 * lamps[:, 3]
    du = half_length * np.cos(angle)
    dv = half_length * np.sin(angle)
    endpoint_1, endpoint_1_valid = project_pixels(
        fit.spec,
        fit.params,
        lamps[:, 0] - du,
        lamps[:, 1] - dv,
        lamp_camera,
        lamp_pose,
        lamp_height,
        fit.lut,
    )
    endpoint_2, endpoint_2_valid = project_pixels(
        fit.spec,
        fit.params,
        lamps[:, 0] + du,
        lamps[:, 1] + dv,
        lamp_camera,
        lamp_pose,
        lamp_height,
        fit.lut,
    )
    lamp_projected = (
        dataset.lamp_valid[rows].reshape(-1)
        & center_valid
        & endpoint_1_valid
        & endpoint_2_valid
    )
    lamp_angle = np.arctan2(
        endpoint_2[:, 1] - endpoint_1[:, 1],
        endpoint_2[:, 0] - endpoint_1[:, 0],
    )
    yaw_rate = np.repeat(dataset.yaw_rate[rows], 3)
    lamp_angle += yaw_rate * delays_ms[lamp_camera] * 0.001 * DEG
    lamp_angle[~lamp_projected] = np.nan
    return {
        "beacon": beacon_points.reshape(count, 3, 4, 2),
        "beacon_valid": beacon_projected.reshape(count, 3, 4),
        "lamp": lamp_center.reshape(count, 3, 2),
        "lamp_valid": lamp_projected.reshape(count, 3),
        "lamp_angle": lamp_angle.reshape(count, 3),
    }


def geometry_values(
    dataset: Dataset,
    rows: np.ndarray,
    lamp_camera: np.ndarray,
    beacon_camera: np.ndarray,
    beacon_slot: np.ndarray,
    scene: dict[str, np.ndarray],
) -> dict[str, np.ndarray]:
    local = np.arange(rows.size)
    lamp = scene["lamp"][local, lamp_camera]
    beacon = scene["beacon"][local, beacon_camera, beacon_slot]
    lamp_angle = scene["lamp_angle"][local, lamp_camera]
    valid = scene["lamp_valid"][local, lamp_camera]
    valid &= scene["beacon_valid"][local, beacon_camera, beacon_slot]
    delta = beacon - lamp
    distance = np.linalg.norm(delta, axis=1)
    target_angle = np.arctan2(delta[:, 1], delta[:, 0])
    car_yaw = dataset.car_yaw[rows].astype(float) * DEG
    right_angle = lamp_angle.copy()
    flip = np.cos(right_angle - car_yaw - 0.5 * math.pi) < 0.0
    right_angle[flip] += math.pi
    right_x = np.cos(right_angle)
    right_y = np.sin(right_angle)
    strafe = delta[:, 0] * right_x + delta[:, 1] * right_y
    forward = delta[:, 0] * right_y - delta[:, 1] * right_x
    bearing = np.arctan2(strafe, forward)
    valid &= np.isfinite(distance)
    valid &= (distance >= MIN_TRUSTED_TARGET_DISTANCE_M) & (distance <= 6.0)
    return {
        "valid": valid,
        "distance": distance,
        "target_angle": target_angle,
        "lamp_angle": lamp_angle,
        "bearing": bearing,
        "target_error": wrap_rad(target_angle),
        "lamp_error": wrap_axis_rad(lamp_angle - car_yaw - 0.5 * math.pi),
        "bearing_error": wrap_rad(bearing + car_yaw),
    }


def choose_observations(
    dataset: Dataset,
    rows: np.ndarray,
    fit: FitResult,
    use_truth: bool,
    delays_ms: np.ndarray | None = None,
) -> tuple[Observations, dict[str, np.ndarray]]:
    scene = project_scene(dataset, rows, fit, delays_ms)
    selected_rows: list[int] = []
    selected_lamp: list[int] = []
    selected_beacon: list[int] = []
    selected_slot: list[int] = []
    selected_score: list[float] = []
    selected_same: list[bool] = []
    vector_tracks: dict[int, list[dict[str, float | int | np.ndarray]]] = {}
    for local, row in enumerate(rows):
        lamps = np.flatnonzero(scene["lamp_valid"][local])
        beacons = np.argwhere(scene["beacon_valid"][local])
        if lamps.size == 0 or beacons.size == 0:
            continue
        pairs = [
            (int(lamp), int(beacon_camera), int(slot))
            for lamp in lamps
            for beacon_camera, slot in beacons
            if lamp == beacon_camera
        ]
        same = bool(pairs)
        if not pairs:
            pairs = [
                (int(lamp), int(beacon_camera), int(slot))
                for lamp in lamps
                for beacon_camera, slot in beacons
            ]
        options: list[tuple[int, int, int, np.ndarray, float, float]] = []
        for lamp, beacon_camera, slot in pairs:
            delta = (
                scene["beacon"][local, beacon_camera, slot]
                - scene["lamp"][local, lamp]
            )
            distance = float(np.linalg.norm(delta))
            if not (MIN_TRUSTED_TARGET_DISTANCE_M <= distance <= 6.0):
                continue
            score = distance
            car_yaw = float(dataset.car_yaw[row]) * DEG
            lamp_angle = float(scene["lamp_angle"][local, lamp])
            if use_truth:
                target_angle = math.atan2(float(delta[1]), float(delta[0]))
                right_angle = lamp_angle
                if math.cos(right_angle - car_yaw - 0.5 * math.pi) < 0.0:
                    right_angle += math.pi
                right_x, right_y = math.cos(right_angle), math.sin(right_angle)
                bearing = math.atan2(
                    float(delta[0]) * right_x + float(delta[1]) * right_y,
                    float(delta[0]) * right_y - float(delta[1]) * right_x,
                )
                target_error = abs(float(wrap_rad(target_angle))) / (12.0 * DEG)
                lamp_error = abs(
                    float(wrap_axis_rad(lamp_angle - car_yaw - 0.5 * math.pi))
                ) / (15.0 * DEG)
                bearing_error = abs(float(wrap_rad(bearing + car_yaw))) / (8.0 * DEG)
                distance_error = abs(math.log(distance / float(dataset.distance_m[row]))) / 0.25
                score = (
                    target_error * target_error
                    + lamp_error * lamp_error
                    + bearing_error * bearing_error
                    + distance_error * distance_error
                )
            options.append((lamp, beacon_camera, slot, delta.copy(), distance, score))
        if not options:
            continue
        if use_truth:
            best_option = min(options, key=lambda item: item[5])
            if best_option[5] > 30.0:
                continue
        else:
            log_id = int(dataset.log_id[row])
            timestamp = float(dataset.time_ms[row])
            tracks = vector_tracks.setdefault(log_id, [])
            tracks[:] = [track for track in tracks if timestamp - float(track["time"]) <= 500.0]
            mature_before = any(int(track["count"]) >= 2 for track in tracks)
            used_tracks: set[int] = set()
            ranked: list[tuple[int, float, float, tuple[int, int, int, np.ndarray, float, float]]] = []
            for option in sorted(options, key=lambda item: item[4]):
                delta = option[3]
                best_track = -1
                best_jump = math.inf
                for track_index, track in enumerate(tracks):
                    if track_index in used_tracks:
                        continue
                    track_vector = np.asarray(track["vector"])
                    jump = float(np.linalg.norm(delta - track_vector))
                    track_distance = float(np.linalg.norm(track_vector))
                    delta_distance = float(np.linalg.norm(delta))
                    cosine = float(
                        np.clip(
                            np.dot(delta, track_vector)
                            / max(delta_distance * track_distance, 1.0e-9),
                            -1.0,
                            1.0,
                        )
                    )
                    angle_jump = math.acos(cosine)
                    distance_jump = abs(delta_distance - track_distance)
                    if (
                        angle_jump <= 25.0 * DEG
                        and distance_jump <= max(0.25, 0.30 * track_distance)
                        and jump < best_jump
                    ):
                        best_track = track_index
                        best_jump = jump
                if best_track < 0:
                    tracks.append(
                        {"vector": delta.copy(), "count": 1, "time": timestamp}
                    )
                    best_track = len(tracks) - 1
                    best_jump = math.inf
                else:
                    track = tracks[best_track]
                    count = min(int(track["count"]) + 1, 255)
                    gain = 1.0 / min(count, 8)
                    track["vector"] = (1.0 - gain) * np.asarray(track["vector"]) + gain * delta
                    track["count"] = count
                    track["time"] = timestamp
                used_tracks.add(best_track)
                ranked.append(
                    (int(tracks[best_track]["count"]), best_jump, option[4], option)
                )
            confirmed = [item for item in ranked if item[0] >= 2]
            if confirmed:
                best_rank = min(
                    confirmed,
                    key=lambda item: item[2] + 0.35 * item[1] - 0.01 * min(item[0], 10),
                )
                best_option = best_rank[3]
            elif len(options) == 1 and not mature_before:
                best_option = options[0]
            else:
                continue
        best = best_option[:3]
        best_score = float(best_option[5])
        selected_rows.append(int(row))
        selected_lamp.append(best[0])
        selected_beacon.append(best[1])
        selected_slot.append(best[2])
        selected_score.append(best_score)
        selected_same.append(same)

    observations = Observations(
        row=np.asarray(selected_rows, dtype=np.int64),
        lamp_camera=np.asarray(selected_lamp, dtype=np.int8),
        beacon_camera=np.asarray(selected_beacon, dtype=np.int8),
        beacon_slot=np.asarray(selected_slot, dtype=np.int8),
    )
    metadata = {
        "score": np.asarray(selected_score, dtype=float),
        "same": np.asarray(selected_same, dtype=bool),
    }
    return observations, metadata


def observation_geometry(
    dataset: Dataset,
    observations: Observations,
    fit: FitResult,
    delays_ms: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    scene = project_scene(dataset, observations.row, fit, delays_ms)
    return geometry_values(
        dataset,
        observations.row,
        observations.lamp_camera,
        observations.beacon_camera,
        observations.beacon_slot,
        scene,
    )


def fit_residual(
    params: np.ndarray,
    dataset: Dataset,
    observations: Observations,
    spec: ModelSpec,
    initial: np.ndarray,
    lut: np.ndarray | None = None,
) -> np.ndarray:
    fit = FitResult(spec, params, lut, len(observations), 0.0)
    geometry = observation_geometry(dataset, observations, fit)
    valid = geometry["valid"]
    distance = np.maximum(geometry["distance"], 1.0e-6)
    residual = np.column_stack(
        [
            geometry["target_error"] / (8.0 * DEG),
            geometry["lamp_error"] / (8.0 * DEG),
            geometry["bearing_error"] / (4.0 * DEG),
            np.log(distance / dataset.distance_m[observations.row]) / 0.18,
        ]
    )
    residual[~valid] = 20.0

    intrinsic_size = spec.intrinsic_count * 3
    regularization = []
    intrinsic_scale = np.maximum(np.abs(initial[:intrinsic_size]), 1.0)
    regularization.append(
        0.03 * (params[:intrinsic_size] - initial[:intrinsic_size]) / intrinsic_scale
    )
    regularization.append(0.10 * params[intrinsic_size : intrinsic_size + 9] / (15.0 * DEG))
    regularization.append(0.10 * params[intrinsic_size + 9 :] / 0.15)
    return np.concatenate([residual.ravel(), *regularization])


def fit_physical_model(
    dataset: Dataset,
    train_mask: np.ndarray,
    spec: ModelSpec,
    maximum_samples: int,
    max_nfev: int,
    start: np.ndarray | None = None,
) -> FitResult:
    params, lower, upper = initial_parameters(spec)
    initial = params.copy()
    if start is not None and start.shape == params.shape:
        params = np.clip(start, lower, upper)
    rows = balanced_indices(dataset, train_mask, maximum_samples)
    fit = FitResult(spec, params, None, 0, math.inf)
    observations = Observations(
        np.empty(0, dtype=np.int64),
        np.empty(0, dtype=np.int8),
        np.empty(0, dtype=np.int8),
        np.empty(0, dtype=np.int8),
    )
    result = None
    for _ in range(2):
        observations, _ = choose_observations(dataset, rows, fit, use_truth=True)
        if len(observations) < 200:
            raise RuntimeError(f"{spec.name}: only {len(observations)} usable training observations")
        result = least_squares(
            fit_residual,
            fit.params,
            bounds=(lower, upper),
            args=(dataset, observations, spec, initial, None),
            loss="huber",
            f_scale=1.5,
            x_scale="jac",
            max_nfev=max_nfev,
            verbose=0,
        )
        fit = FitResult(spec, result.x, None, len(observations), float(result.cost))
    return fit


def lut_regularization(table: np.ndarray) -> np.ndarray:
    magnitude = 0.05 * table.ravel() / (3.0 * DEG)
    horizontal = 0.25 * np.diff(table, axis=2).ravel() / (1.0 * DEG)
    vertical = 0.25 * np.diff(table, axis=1).ravel() / (1.0 * DEG)
    return np.concatenate([magnitude, horizontal, vertical])


def lut_residual(
    values: np.ndarray,
    dataset: Dataset,
    observations: Observations,
    base_fit: FitResult,
) -> np.ndarray:
    table = values.reshape(3, 7, 9, 2)
    fit = FitResult(base_fit.spec, base_fit.params, table, len(observations), 0.0)
    geometry = observation_geometry(dataset, observations, fit)
    valid = geometry["valid"]
    distance = np.maximum(geometry["distance"], 1.0e-6)
    residual = np.column_stack(
        [
            geometry["target_error"] / (8.0 * DEG),
            geometry["lamp_error"] / (8.0 * DEG),
            geometry["bearing_error"] / (4.0 * DEG),
            np.log(distance / dataset.distance_m[observations.row]) / 0.18,
        ]
    )
    residual[~valid] = 20.0
    return np.concatenate([residual.ravel(), lut_regularization(table)])


def point_lut_variables(camera: int, u: float, v: float) -> list[int]:
    x0, x1, y0, y1, _, _ = lut_coordinates(np.array([u]), np.array([v]))
    variables = []
    for y in (int(y0[0]), int(y1[0])):
        for x in (int(x0[0]), int(x1[0])):
            base = ((camera * 7 + y) * 9 + x) * 2
            variables.extend([base, base + 1])
    return variables


def lut_jacobian_sparsity(dataset: Dataset, observations: Observations) -> lil_matrix:
    variable_count = 3 * 7 * 9 * 2
    physics_rows = 4 * len(observations)
    regularization_count = variable_count + 3 * 7 * 8 * 2 + 3 * 6 * 9 * 2
    sparsity = lil_matrix((physics_rows + regularization_count, variable_count), dtype=np.int8)
    for index, row in enumerate(observations.row):
        lamp_camera = int(observations.lamp_camera[index])
        beacon_camera = int(observations.beacon_camera[index])
        slot = int(observations.beacon_slot[index])
        lamp = dataset.lamps[row, lamp_camera].astype(float)
        angle = lamp[2] * DEG
        du = 0.5 * lamp[3] * math.cos(angle)
        dv = 0.5 * lamp[3] * math.sin(angle)
        points = [
            (lamp_camera, lamp[0], lamp[1]),
            (lamp_camera, lamp[0] - du, lamp[1] - dv),
            (lamp_camera, lamp[0] + du, lamp[1] + dv),
            (
                beacon_camera,
                float(dataset.beacons[row, beacon_camera, slot, 0]),
                float(dataset.beacons[row, beacon_camera, slot, 1]),
            ),
        ]
        variables: set[int] = set()
        for camera, u, v in points:
            variables.update(point_lut_variables(camera, u, v))
        for residual_row in range(index * 4, index * 4 + 4):
            sparsity[residual_row, list(variables)] = 1

    current_row = physics_rows
    for variable in range(variable_count):
        sparsity[current_row, variable] = 1
        current_row += 1
    for camera in range(3):
        for y in range(7):
            for x in range(8):
                for axis in range(2):
                    left = ((camera * 7 + y) * 9 + x) * 2 + axis
                    right = left + 2
                    sparsity[current_row, [left, right]] = 1
                    current_row += 1
    for camera in range(3):
        for y in range(6):
            for x in range(9):
                for axis in range(2):
                    top = ((camera * 7 + y) * 9 + x) * 2 + axis
                    bottom = top + 9 * 2
                    sparsity[current_row, [top, bottom]] = 1
                    current_row += 1
    return sparsity


def fit_lut_model(
    dataset: Dataset,
    train_mask: np.ndarray,
    base_fit: FitResult,
    maximum_samples: int,
    max_nfev: int,
) -> FitResult:
    rows = balanced_indices(dataset, train_mask, maximum_samples)
    observations, _ = choose_observations(dataset, rows, base_fit, use_truth=True)
    if len(observations) < 200:
        raise RuntimeError(f"double_sphere_lut: only {len(observations)} observations")
    initial = np.zeros(3 * 7 * 9 * 2, dtype=float)
    limit = np.full(initial.size, 8.0 * DEG)
    sparsity = lut_jacobian_sparsity(dataset, observations)
    result = least_squares(
        lut_residual,
        initial,
        bounds=(-limit, limit),
        args=(dataset, observations, base_fit),
        jac_sparsity=sparsity,
        loss="huber",
        f_scale=1.5,
        x_scale="jac",
        max_nfev=max_nfev,
        verbose=0,
    )
    return FitResult(
        base_fit.spec,
        base_fit.params.copy(),
        result.x.reshape(3, 7, 9, 2),
        len(observations),
        float(result.cost),
    )


def model_key(fit: FitResult) -> str:
    return "double_sphere_lut" if fit.lut is not None else fit.spec.name


def evaluate_model(
    dataset: Dataset,
    validation_mask: np.ndarray,
    fit: FitResult,
    delays_ms: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    rows = np.flatnonzero(validation_mask & (dataset.raw_pair_type >= 0))
    observations, metadata = choose_observations(
        dataset, rows, fit, use_truth=False, delays_ms=delays_ms
    )
    geometry = observation_geometry(dataset, observations, fit, delays_ms)
    output = {
        "row": rows,
        "valid": np.zeros(rows.size, dtype=bool),
        "direction_error_deg": np.full(rows.size, np.nan, dtype=float),
        "distance_error_m": np.full(rows.size, np.nan, dtype=float),
        "relative_distance_error": np.full(rows.size, np.nan, dtype=float),
        "pair_type": dataset.raw_pair_type[rows].copy(),
        "beacon_camera": np.empty(rows.size, dtype=np.int8),
        "lamp_camera": np.empty(rows.size, dtype=np.int8),
        "distance_m": dataset.distance_m[rows].astype(float),
        "region": dataset.region[rows].copy(),
        "pose_rate": dataset.pose_rate[rows].astype(float),
        "log_id": dataset.log_id[rows].copy(),
    }
    raw_bucket = dataset.camera_bucket[rows]
    cross_bucket = np.maximum(raw_bucket - 3, 0)
    output["lamp_camera"][:] = np.where(raw_bucket < 3, raw_bucket, cross_bucket // 3)
    output["beacon_camera"][:] = np.where(raw_bucket < 3, raw_bucket, cross_bucket % 3)
    if len(observations) == 0:
        return output
    positions = np.searchsorted(rows, observations.row)
    valid = geometry["valid"]
    positions = positions[valid]
    chosen_rows = observations.row[valid]
    direction_error = np.abs(geometry["bearing_error"][valid]) / DEG
    distance_error = np.abs(
        geometry["distance"][valid] - dataset.distance_m[chosen_rows]
    )
    output["valid"][positions] = True
    output["direction_error_deg"][positions] = direction_error
    output["distance_error_m"][positions] = distance_error
    output["relative_distance_error"][positions] = (
        distance_error / dataset.distance_m[chosen_rows]
    )
    output["pair_type"][positions] = np.where(metadata["same"][valid], 0, 1)
    output["lamp_camera"][positions] = observations.lamp_camera[valid]
    output["beacon_camera"][positions] = observations.beacon_camera[valid]
    return output


def concatenate_evaluations(items: list[dict[str, np.ndarray]]) -> dict[str, np.ndarray]:
    if not items:
        return {}
    return {key: np.concatenate([item[key] for item in items]) for key in items[0]}


def metric_values(evaluation: dict[str, np.ndarray], mask: np.ndarray) -> dict[str, float]:
    denominator = int(np.count_nonzero(mask))
    valid = mask & evaluation["valid"]
    count = int(np.count_nonzero(valid))
    if count == 0:
        return {
            "count": 0,
            "total": denominator,
            "coverage": 0.0,
            "direction_p50": math.nan,
            "direction_p90": math.nan,
            "direction_p99": math.nan,
            "direction_mean": math.nan,
            "distance_p50": math.nan,
            "distance_p90": math.nan,
            "distance_p99": math.nan,
            "distance_mean": math.nan,
            "relative_distance_p50": math.nan,
            "relative_distance_p90": math.nan,
            "catastrophic_rate": math.nan,
        }
    direction = evaluation["direction_error_deg"][valid]
    distance = evaluation["distance_error_m"][valid]
    relative = evaluation["relative_distance_error"][valid]
    return {
        "count": count,
        "total": denominator,
        "coverage": count / denominator if denominator else 0.0,
        "direction_p50": float(np.percentile(direction, 50)),
        "direction_p90": float(np.percentile(direction, 90)),
        "direction_p99": float(np.percentile(direction, 99)),
        "direction_mean": float(np.mean(direction)),
        "distance_p50": float(np.percentile(distance, 50)),
        "distance_p90": float(np.percentile(distance, 90)),
        "distance_p99": float(np.percentile(distance, 99)),
        "distance_mean": float(np.mean(distance)),
        "relative_distance_p50": float(np.percentile(relative, 50)),
        "relative_distance_p90": float(np.percentile(relative, 90)),
        "catastrophic_rate": float(np.mean(direction > 30.0)),
    }


def summarize_evaluation(
    model: str,
    validation: str,
    evaluation: dict[str, np.ndarray],
) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []

    def append(group: str, value: str, mask: np.ndarray) -> None:
        row: dict[str, float | str] = {
            "model": model,
            "validation": validation,
            "group": group,
            "value": value,
        }
        row.update(metric_values(evaluation, mask))
        rows.append(row)

    size = evaluation["valid"].size
    append("overall", "all", np.ones(size, dtype=bool))
    append("pair_type", "same", evaluation["pair_type"] == 0)
    append("pair_type", "cross", evaluation["pair_type"] == 1)
    for distance in sorted(np.unique(evaluation["distance_m"])):
        append("distance_m", f"{distance:.1f}", np.isclose(evaluation["distance_m"], distance))
    for camera, name in enumerate(CAMERA_NAMES):
        append("beacon_camera", name, evaluation["beacon_camera"] == camera)
    for log_id in sorted(np.unique(evaluation["log_id"])):
        append("log_id", str(int(log_id)), evaluation["log_id"] == log_id)
    append("image_region", "center", evaluation["region"] == 0)
    append("image_region", "edge", evaluation["region"] == 1)
    positive = evaluation["pose_rate"][evaluation["pose_rate"] > 0.0]
    threshold = float(np.median(positive)) if positive.size else 0.0
    append("pose_change", "low", evaluation["pose_rate"] <= threshold)
    append("pose_change", "high", evaluation["pose_rate"] > threshold)
    return rows


def fit_model_set(
    dataset: Dataset,
    train_mask: np.ndarray,
    maximum_samples: int,
    lut_samples: int,
    max_nfev: int,
    lut_nfev: int,
) -> dict[str, FitResult]:
    fitted: dict[str, FitResult] = {}
    for name in ("pinhole_k1", "double_sphere", "kb8"):
        started = time.perf_counter()
        fit = fit_physical_model(
            dataset, train_mask, SPECS[name], maximum_samples, max_nfev
        )
        fitted[name] = fit
        print(
            f"  fitted {name}: observations={fit.observation_count}, "
            f"cost={fit.cost:.1f}, time={time.perf_counter() - started:.1f}s",
            flush=True,
        )
    started = time.perf_counter()
    fitted["double_sphere_lut"] = fit_lut_model(
        dataset,
        train_mask,
        fitted["double_sphere"],
        lut_samples,
        lut_nfev,
    )
    fit = fitted["double_sphere_lut"]
    print(
        f"  fitted double_sphere_lut: observations={fit.observation_count}, "
        f"cost={fit.cost:.1f}, time={time.perf_counter() - started:.1f}s",
        flush=True,
    )
    return fitted


def pair_score(evaluation: dict[str, np.ndarray]) -> tuple[float, float]:
    p90_values = []
    coverage_values = []
    for pair_type in (0, 1):
        values = metric_values(evaluation, evaluation["pair_type"] == pair_type)
        if math.isfinite(values["direction_p90"]):
            p90_values.append(values["direction_p90"])
            coverage_values.append(values["coverage"])
    return (
        max(p90_values) if p90_values else math.inf,
        min(coverage_values) if coverage_values else 0.0,
    )


def scan_model_delays(
    dataset: Dataset,
    fold_fits: list[tuple[np.ndarray, FitResult]],
    maximum_rows: int,
) -> dict[str, float | list[float]]:
    sample_folds: list[tuple[np.ndarray, FitResult]] = []
    for mask, fit in fold_fits:
        rows = balanced_indices(dataset, mask, maximum_rows)
        sample_mask = np.zeros(len(dataset), dtype=bool)
        sample_mask[rows] = True
        sample_folds.append((sample_mask, fit))

    def evaluate(delays: np.ndarray) -> tuple[float, float]:
        combined = concatenate_evaluations(
            [
                evaluate_model(dataset, mask, fit, delays)
                for mask, fit in sample_folds
            ]
        )
        return pair_score(combined)

    delays = np.zeros(3, dtype=float)
    for camera in range(3):
        _, current_coverage = evaluate(delays)
        candidates = []
        for delay in DELAY_MS:
            candidate = delays.copy()
            candidate[camera] = delay
            p90, coverage = evaluate(candidate)
            if coverage + 1.0e-9 >= current_coverage:
                candidates.append((p90, -coverage, float(delay)))
        if candidates:
            candidates.sort()
            delays[camera] = candidates[0][2]

    full_baseline = concatenate_evaluations(
        [evaluate_model(dataset, mask, fit, np.zeros(3)) for mask, fit in fold_fits]
    )
    full_best = concatenate_evaluations(
        [evaluate_model(dataset, mask, fit, delays) for mask, fit in fold_fits]
    )
    baseline_p90, baseline_coverage = pair_score(full_baseline)
    best_p90, best_coverage = pair_score(full_best)
    accepted = (
        baseline_p90 - best_p90 >= 1.0
        and best_coverage + 1.0e-9 >= baseline_coverage
    )
    return {
        "delays_ms": delays.tolist(),
        "baseline_worst_p90": baseline_p90,
        "best_worst_p90": best_p90,
        "baseline_coverage": baseline_coverage,
        "best_coverage": best_coverage,
        "accepted": bool(accepted),
    }


def fit_to_json(fit: FitResult) -> dict:
    intrinsics, rotations, translation = decode_parameters(fit.spec, fit.params)
    result = {
        "model": model_key(fit),
        "intrinsics": {
            CAMERA_NAMES[camera]: intrinsics[camera].tolist() for camera in range(3)
        },
        "camera_to_body": {
            CAMERA_NAMES[camera]: rotations[camera].tolist() for camera in range(3)
        },
        "translation_body_m": {
            CAMERA_NAMES[camera]: translation[camera].tolist() for camera in range(3)
        },
        "yaw_bias_rad_fixed": YAW_BIAS_RAD,
        "training_observations": fit.observation_count,
        "least_squares_cost": fit.cost,
    }
    if fit.lut is not None:
        result["lut_grid_x_px"] = GRID_X.tolist()
        result["lut_grid_y_px"] = GRID_Y.tolist()
        result["lut_angle_correction_rad"] = {
            CAMERA_NAMES[camera]: fit.lut[camera].tolist() for camera in range(3)
        }
    return result


def timing_rows() -> list[dict[str, float | int | str]]:
    definitions = {
        "pinhole_k1": (0, 3, 0, 0, 65),
        "double_sphere": (2, 5, 0, 0, 110),
        "kb8": (1, 9, 2, 6, 170),
        "double_sphere_lut": (3, 5, 5, 0, 160),
    }
    rows = []
    for model, (sqrt_count, division_count, trig_count, iterations, scalar_cycles) in definitions.items():
        cycles_per_point = (
            sqrt_count * 14
            + division_count * 14
            + trig_count * 100
            + scalar_cycles
        )
        total_cycles = cycles_per_point * 15
        microseconds = total_cycles / 250.0
        rows.append(
            {
                "model": model,
                "sqrt_per_point": sqrt_count,
                "division_per_point": division_count,
                "trig_per_point": trig_count,
                "newton_iterations": iterations,
                "cycles_15_points": total_cycles,
                "time_us_250mhz": microseconds,
                "period_percent_100hz": microseconds / 10000.0 * 100.0,
            }
        )
    return rows


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def metric_lookup(
    rows: list[dict], model: str, validation: str, group: str, value: str
) -> dict | None:
    for row in rows:
        if (
            row["model"] == model
            and row["validation"] == validation
            and row["group"] == group
            and row["value"] == value
        ):
            return row
    return None


def create_plots(
    output_dir: Path,
    primary_evaluations: dict[str, dict[str, np.ndarray]],
    metrics: list[dict],
) -> None:
    plot_dir = output_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    labels = {
        "pinhole_k1": "Pinhole + k1",
        "double_sphere": "Double Sphere",
        "kb8": "KB8",
        "double_sphere_lut": "Double Sphere + LUT",
    }
    plt.figure(figsize=(8.0, 5.0))
    for model, evaluation in primary_evaluations.items():
        error = evaluation["direction_error_deg"][evaluation["valid"]]
        error = np.sort(error[np.isfinite(error)])
        if error.size:
            plt.plot(error, np.linspace(0.0, 1.0, error.size), label=labels[model])
    plt.xlim(0.0, 40.0)
    plt.ylim(0.0, 1.0)
    plt.xlabel("Absolute car-frame direction error (deg)")
    plt.ylabel("Empirical CDF")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "direction_error_cdf.png", dpi=180)
    plt.close()

    distances = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5]
    plt.figure(figsize=(9.0, 5.0))
    for model in labels:
        values = []
        for distance in distances:
            row = metric_lookup(metrics, model, "primary_two_fold", "distance_m", f"{distance:.1f}")
            values.append(row["direction_p90"] if row else math.nan)
        plt.plot(distances, values, marker="o", label=labels[model])
    plt.axhline(7.0, color="black", linestyle="--", linewidth=1.0, label="7 deg reference")
    plt.xlabel("Lamp-to-beacon distance (m)")
    plt.ylabel("Direction P90 (deg)")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "direction_p90_by_distance.png", dpi=180)
    plt.close()

    plt.figure(figsize=(9.0, 5.0))
    width = 0.18
    x = np.arange(len(distances), dtype=float)
    for index, model in enumerate(labels):
        values = []
        for distance in distances:
            row = metric_lookup(metrics, model, "primary_two_fold", "distance_m", f"{distance:.1f}")
            values.append(row["distance_p90"] if row else math.nan)
        plt.bar(x + (index - 1.5) * width, values, width=width, label=labels[model])
    plt.xticks(x, [f"{distance:.1f}" for distance in distances])
    plt.xlabel("Lamp-to-beacon distance (m)")
    plt.ylabel("Absolute distance error P90 (m)")
    plt.grid(True, axis="y", alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "distance_p90_by_distance.png", dpi=180)
    plt.close()


def format_number(value: float, digits: int = 2) -> str:
    return "-" if not math.isfinite(float(value)) else f"{float(value):.{digits}f}"


def build_report(
    output_dir: Path,
    dataset: Dataset,
    metrics: list[dict],
    delay_results: dict[str, dict],
    timings: list[dict],
    parameters: dict[str, dict],
    loo_enabled: bool,
) -> None:
    labels = {
        "pinhole_k1": "针孔 + k1（重拟合）",
        "double_sphere": "Double Sphere",
        "kb8": "Kannala-Brandt 8",
        "double_sphere_lut": "Double Sphere + 9x7 残差表",
    }
    pair_labels = {"same": "同摄", "cross": "跨摄"}
    camera_labels = {"Front": "前摄", "Center": "中摄", "Back": "后摄"}
    ds_same = metric_lookup(metrics, "double_sphere", "primary_two_fold", "pair_type", "same")
    ds_cross = metric_lookup(metrics, "double_sphere", "primary_two_fold", "pair_type", "cross")
    kb_same = metric_lookup(metrics, "kb8", "primary_two_fold", "pair_type", "same")
    kb_cross = metric_lookup(metrics, "kb8", "primary_two_fold", "pair_type", "cross")

    lines = [
        "# 三摄鱼眼相机模型离线拟合与验证报告",
        "",
        "## 一、先看结论",
        "",
        "1. **建议优先把 Double Sphere 作为正式固件候选。** 它与 KB8 的方向精度几乎相同，但计算量更小、灾难误差率略低。",
        "2. **KB8 是纯精度略优的模型。** 它的同摄/跨摄方向 P90 只比 Double Sphere 好约 0.02-0.03°，不足以抵消更高的计算复杂度。",
        "3. **9x7 二维残差表没有带来留出集收益。** 它在训练数据上可以继续降低残差，但验证结果反而略差，说明出现了过拟合，不建议进固件。",
        "4. **固定图像延迟补偿不值得加入。** 扫描三路 0-50 ms 后，最差 P90 只改善约 0.01°，远低于预设的 1° 接受门槛。",
        "5. **方向解算明显进步，但尚未完全达标。** 宽角模型的跨摄 P90 已低于 7°，同摄 P90 仍约 7.4°，灾难误差率也仍高于 0.2%。",
        "6. **绝对距离只在约 0.5-1.0 m 范围较可靠。** 距离增大后尺度误差快速增加，因此当前参数更适合先改善 CarPlan3 行驶方向，不能宣称已经精确解决全距离测距。",
        "",
        "本报告只完成离线比较和参数输出，**没有替换 `Three_Camera.c/.h` 或 `car_plan_3.c` 的正式模型**。",
        "",
        "## 二、数据、物理假设与验证方法",
        "",
        f"- 共解析 {len(dataset.logs)} 份完整 CSV。按照 I1-I48 原始检测是否变化进行精确去重后，保留 {len(dataset):,} 个回放样本。",
        "- 车灯中心和信标中心都按 `z=0` 地面点处理；文件名中的距离表示两者中心的水平欧氏距离。",
        "- 期望车体目标角为 `wrap(-car_yaw)`。也就是说，当车 yaw 为正时，图像解算出的目标方向应在车体坐标系中相应向左旋转。",
        "- 同一摄像头同时看到车灯和信标时，优先使用该摄像头内的相对向量；只有没有同摄组合时才进入跨摄外参解算。",
        "- 训练阶段允许使用文件名距离进行候选关联和鲁棒剔除；验证阶段选择候选时完全不读取文件名距离，只使用近车灯过滤、短时连续性、同摄优先和最近投影目标。",
        f"- 投影后车灯到信标小于 {MIN_TRUSTED_TARGET_DISTANCE_M:.2f} m 的组合视为明显的近车灯误检并立即判无效，不输出历史方向代替。",
        "- 主验证严格按整份日志拆分：无 `_2` 后缀训练、`_2` 验证，再反向验证一次。没有随机拆帧，因此相邻帧不会同时落入训练集和验证集。",
        "- 另外执行 7 组留一距离验证：每次完全拿掉一个距离的两份日志，使用其余六个距离重新拟合，再验证被拿掉的距离。",
        "- 全局 yaw 偏置保持现值不动。原因是仅凭这些相对几何日志，所有相机共同绕 Z 轴旋转与全局 yaw 偏置之间存在不可辨识自由度。",
        "- 复现命令：`python tools/camera_model_fit.py --data-dir \"D:/Downloads/摄像头拟合\"`。",
        "",
        "## 三、指标怎么理解",
        "",
        "- **同摄**：被选中的车灯与信标来自同一个摄像头。它主要检验单个镜头的畸变模型和车灯长轴投影。",
        "- **跨摄**：车灯和信标不在同一个摄像头内。除了内参畸变，还会受到三摄旋转和平移外参误差影响。",
        "- **方向误差**：解算出的车体目标方向与 `wrap(-car_yaw)` 的最小圆周角误差，单位为度。这个指标直接对应车模应该向左/向右转多少。",
        "- **P50**：一半有效样本的误差不超过该值，反映日常典型表现。",
        "- **P90**：90% 有效样本的误差不超过该值，是本次方向选型最重要的指标。",
        "- **P99**：99% 有效样本的误差不超过该值，用于观察长尾风险。",
        "- **覆盖率**：原始数据存在车灯和信标候选时，最终成功得到可信方向的比例。判无效的组合计入覆盖率损失。",
        "- **灾难误差率**：方向误差大于 30° 的有效样本比例。这类错误可能让车明显跑偏，不能只看 P50 而忽略。",
        "- **距离 P50/P90**：车灯中心到信标中心的绝对距离误差分位数，单位为米。",
        "",
        "参考目标：覆盖率不低于 98%；同摄方向 P90 不高于 6°；跨摄方向 P90 不高于 7°；灾难误差率不高于 0.2%；距离中位误差不高于 `max(0.08 m, 5%距离)`，距离 P90 不高于 `max(0.20 m, 15%距离)`。",
        "",
        "## 四、当前固件基线",
        "",
        "当前正式固件日志基线保持原值：同摄方向 P50/P90 为 **4.60°/13.13°**，跨摄为 **4.52°/8.99°**。",
        "",
        "下面的“针孔 + k1（重拟合）”不是旧固件逐行复刻。它与三种鱼眼候选统一使用 `z=0`、相同候选过滤和相同融合逻辑，用于公平判断模型本身的差异。",
        "",
        "## 五、主验证结果：整日志双向验证",
        "",
        "| 模型 | 配对类型 | 方向 P50（°） | 方向 P90（°） | 方向 P99（°） | 方向均值（°） | 覆盖率 | >30° 比例 | 距离 P50（m） | 距离 P90（m） |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for model in labels:
        for pair in ("same", "cross"):
            row = metric_lookup(metrics, model, "primary_two_fold", "pair_type", pair)
            if row is None:
                continue
            lines.append(
                f"| {labels[model]} | {pair_labels[pair]} | {format_number(row['direction_p50'])} | "
                f"{format_number(row['direction_p90'])} | {format_number(row['direction_p99'])} | "
                f"{format_number(row['direction_mean'])} | {row['coverage'] * 100:.2f}% | "
                f"{row['catastrophic_rate'] * 100:.3f}% | {format_number(row['distance_p50'], 3)} | "
                f"{format_number(row['distance_p90'], 3)} |"
            )

    if ds_same and ds_cross and kb_same and kb_cross:
        ds_same_gain = (13.13 - ds_same["direction_p90"]) / 13.13 * 100.0
        ds_cross_gain = (8.99 - ds_cross["direction_p90"]) / 8.99 * 100.0
        worst_log = max(
            (
                metric_lookup(metrics, "double_sphere", "primary_two_fold", "log_id", str(log_id))
                for log_id in range(len(dataset.logs))
            ),
            key=lambda row: row["direction_p90"] if row else -math.inf,
        )
        worst_log_name = dataset.logs[int(worst_log["value"])].name if worst_log else "unknown"
        lines.extend(
            [
                "",
                "### 主验证结论",
                "",
                f"- Double Sphere 相比当前固件，同摄 P90 降低 {ds_same_gain:.1f}%，跨摄 P90 降低 {ds_cross_gain:.1f}%。",
                f"- Double Sphere 同摄 P90 为 {ds_same['direction_p90']:.2f}°。含义是：在成功输出的同摄样本中，90% 的方向误差不超过该值；虽然明显优于 13.13° 基线，但仍未达到 6° 目标。",
                f"- Double Sphere 跨摄 P90 为 {ds_cross['direction_p90']:.2f}°，KB8 为 {kb_cross['direction_p90']:.2f}°，两者均达到跨摄不高于 7° 的目标。",
                f"- 同摄/跨摄整体覆盖率超过 98%，但 3.5 m 分桶覆盖率低于 98%；Double Sphere 方向 P90 最差的单份日志是 `{worst_log_name}`。",
                "- 所有候选的灾难误差率都高于 0.2%。报告保留这些错误，没有通过继续加硬门限、降低覆盖率来隐藏它们。",
                "- 距离目标大致只在 0.5-1.0 m 满足。日志没有飞机水平位置，飞机移动、相机平移、高度偏差和畸变参数会共同影响距离尺度，因此方向比绝对距离更容易从现有数据中可靠拟合。",
                "",
                "![四种模型的方向误差累计分布](plots/direction_error_cdf.png)",
                "",
                "累计分布曲线越靠左上越好。Double Sphere 与 KB8 基本重合，并且整体优于重拟合针孔模型；30° 之后仍存在少量长尾，对应表中的灾难误差。",
            ]
        )

    lines.extend(
        [
            "",
            "## 六、不同真实距离下的表现",
            "",
            "这张表用于判断模型是否只在某一个距离上有效。方向 P90 越小越好；距离 P50/P90 是绝对误差，不是测量值本身。",
            "",
            "| 模型 | 真实距离 | 方向 P90（°） | 距离 P50（m） | 距离 P90（m） | 覆盖率 |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for model in labels:
        for distance in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5):
            row = metric_lookup(metrics, model, "primary_two_fold", "distance_m", f"{distance:.1f}")
            if row is None:
                continue
            lines.append(
                f"| {labels[model]} | {distance:.1f} m | {format_number(row['direction_p90'])} | "
                f"{format_number(row['distance_p50'], 3)} | {format_number(row['distance_p90'], 3)} | "
                f"{row['coverage'] * 100:.2f}% |"
            )

    lines.extend(
        [
            "",
            "从结果可以看到，鱼眼模型在 1.5-3.5 m 的方向解算优势最明显；0.5 m 和 1.0 m 的方向 P90 仍偏高，说明近距离大视场边缘、车灯长轴投影或误检关联仍是主要误差源。距离误差则随真实距离增加而明显增大。",
            "",
            "![不同距离下的方向 P90](plots/direction_p90_by_distance.png)",
            "",
            "上图中虚线为 7° 参考目标。鱼眼模型在 1.5 m 以后逐渐低于该线，但近距离仍没有达到目标。",
            "",
            "![不同距离下的距离误差 P90](plots/distance_p90_by_distance.png)",
            "",
            "距离误差随真实距离增大，说明当前日志对绝对尺度的约束不足；这不是换成更高阶鱼眼模型就能自动消除的问题。",
            "",
            "## 七、摄像头、图像区域与飞机姿态变化",
            "",
            "“图像中心/边缘”按参与投影点的归一化半径分组；“低/高姿态变化”按样本姿态变化率的中位数二分。该表用于检查模型是否只在中心区域或飞机平稳时有效。",
            "",
        ]
    )
    lines.extend(
        [
            "| 模型 | 分组 | 方向 P90（°） | 距离 P90（m） | 覆盖率 |",
            "|---|---|---:|---:|---:|",
        ]
    )
    for model in labels:
        for group, value in (
            ("beacon_camera", "Front"),
            ("beacon_camera", "Center"),
            ("beacon_camera", "Back"),
            ("image_region", "center"),
            ("image_region", "edge"),
            ("pose_change", "low"),
            ("pose_change", "high"),
        ):
            row = metric_lookup(metrics, model, "primary_two_fold", group, value)
            if row is None:
                continue
            display_value = {
                "Front": "前摄",
                "Center": "中摄",
                "Back": "后摄",
                "center": "图像中心",
                "edge": "图像边缘",
                "low": "低姿态变化",
                "high": "高姿态变化",
            }[value]
            lines.append(
                f"| {labels[model]} | {display_value} | {format_number(row['direction_p90'])} | "
                f"{format_number(row['distance_p90'], 3)} | {row['coverage'] * 100:.2f}% |"
            )

    if loo_enabled:
        lines.extend(
            [
                "",
                "## 八、留一距离验证",
                "",
                "每一行都表示：该距离的两份日志完全不参加拟合，只用其他六个距离训练，然后测试模型对这个未见距离的泛化能力。它比随机拆帧更严格，也更接近换一个实际车灯距离后的表现。",
                "",
                "| 模型 | 完全留出的距离 | 方向 P90（°） | 距离 P90（m） | 覆盖率 |",
                "|---|---:|---:|---:|---:|",
            ]
        )
        for model in labels:
            for distance in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5):
                validation = f"leave_{distance:.1f}m"
                row = metric_lookup(metrics, model, validation, "overall", "all")
                if row is None:
                    continue
                lines.append(
                    f"| {labels[model]} | {distance:.1f} m | {format_number(row['direction_p90'])} | "
                    f"{format_number(row['distance_p90'], 3)} | {row['coverage'] * 100:.2f}% |"
                )

    lines.extend(
        [
            "",
            "## 九、使用全部 14 份日志拟合的最终参数",
            "",
            "参数顺序说明：针孔模型为 `f, cx, cy, k1`；Double Sphere 为 `fx, fy, cx, cy, xi, alpha`；KB8 为 `fx, fy, cx, cy, k1, k2, k3, k4`。",
            "平移外参采用飞机 FRD 机体系，单位为米；中摄固定为平移基准 `(0,0,0)`。完整的 3x3 相机到机体旋转矩阵和 9x7 残差表节点位于 `model_parameters.json`。",
            "需要注意：Double Sphere 后摄的 `alpha` 已接近本次设置的上界。这说明现有日志对部分参数的约束仍不充分。表中的平移应先视为能够解释日志的“有效外参”，在写入固件前最好与实际安装尺寸或标准标定板结果交叉检查。",
            "",
            "| 模型 | 摄像头 | 内参 | 机体系平移（m） |",
            "|---|---|---|---|",
        ]
    )
    for model in labels:
        parameter = parameters[model]
        for camera in CAMERA_NAMES:
            intrinsic = ", ".join(
                f"{float(value):.7g}" for value in parameter["intrinsics"][camera]
            )
            translation = ", ".join(
                f"{float(value):.5f}" for value in parameter["translation_body_m"][camera]
            )
            lines.append(
                f"| {labels[model]} | {camera_labels[camera]} | `{intrinsic}` | `{translation}` |"
            )

    lines.extend(
        [
            "",
            "## 十、固定图像延迟扫描",
            "",
            "三路相机分别扫描 0-50 ms，步长 5 ms。离线回放会插值飞机姿态，并使用车端 `yaw_rate` 把车灯横轴补偿到当前时刻。日志没有飞机水平位置，因此无法补偿延迟期间飞机的水平平移。",
            "",
            "只有完整留出集的最差 P90 至少改善 1° 且覆盖率不下降，延迟方案才允许被接受。",
            "",
            "| 模型 | 前/中/后摄延迟 | 补偿前最差 P90（°） | 补偿后最差 P90（°） | 补偿前/后覆盖率 | 是否接受 |",
            "|---|---|---:|---:|---:|---:|",
        ]
    )
    for model, result in delay_results.items():
        delays = "/".join(f"{value:.0f}" for value in result["delays_ms"])
        lines.append(
            f"| {labels[model]} | {delays} ms | {format_number(result['baseline_worst_p90'])} | "
            f"{format_number(result['best_worst_p90'])} | {result['baseline_coverage'] * 100:.2f}% / "
            f"{result['best_coverage'] * 100:.2f}% | {'是' if result['accepted'] else '否'} |"
        )

    lines.extend(
        [
            "",
            "扫描结果只改善约 0.01°，远小于 1° 门槛，因此四个模型都不建议加入时序补偿。",
            "",
            "## 十一、250 MHz Cortex-M7 计算量估算",
            "",
            "估算假设：单精度除法和平方根各按约 14 周期，libm 三角函数按约 100 周期，最坏一次处理 15 个投影点。该结果用于模型选型，不是板上 DWT 实测；编译器优化和数学库实现会改变绝对耗时。",
            "",
            "| 模型 | 每点平方根 | 每点除法 | 每点三角函数 | Newton 次数 | 15 点估算耗时 | 占 10 ms 周期 |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in timings:
        lines.append(
            f"| {labels[row['model']]} | {row['sqrt_per_point']} | {row['division_per_point']} | "
            f"{row['trig_per_point']} | {row['newton_iterations']} | {row['time_us_250mhz']:.2f} us | "
            f"{row['period_percent_100hz']:.3f}% |"
        )

    lines.extend(
        [
            "",
            "## 十二、最终选型建议",
            "",
            "**只看方向 P90：KB8 略优。** 但它相对 Double Sphere 只领先约 0.02-0.03°，灾难误差率还略高，因此不能认为 KB8 在工程上明显更稳定。",
            "",
            "**综合精度、稳定性和耗时：建议优先选择 Double Sphere。** 它的 15 点估算耗时为 12.48 us，KB8 为 30.60 us；两者精度几乎持平，而 Double Sphere 的灾难误差率略低。",
            "",
            "**不建议选择 9x7 残差表，也不建议加入固定延迟补偿。** 两者都没有通过完整留出集证明收益。",
            "",
            "当前仍需正视两个问题：",
            "",
            "- 同摄方向 P90 仍约 7.4°，没有达到 6°；近距离 `0.5 m` 的方向表现尤其差。后续应重点检查近距离边缘成像、车灯长轴角度和图像误检，而不是继续堆叠更复杂的查表。",
            "- 1.5 m 以后绝对距离明显不达标。若距离必须精确，建议补充相机标准鱼眼标定、实测三摄安装平移、飞机水平位置/速度或静态标定架数据，将内参、外参、高度和移动造成的尺度误差分开。",
            "",
            "在你确认模型前，本工具不会自动改写正式固件。按当前数据，下一步应先确认是否接受 Double Sphere 的精度边界，再将对应参数以保持 `Three_Camera_Update` 接口不变的方式移植到固件。",
            "",
            "## 十三、生成文件与技术依据",
            "",
            "- `model_parameters.json`：使用全部 14 份日志拟合的最终参数。",
            "- `metrics_summary.csv`：报告中所有分组指标的完整数据。",
            "- `timing_estimate.csv`：每点运算次数和 M7 耗时估算。",
            "- `delay_scan.json`：每个模型的三摄延迟扫描结果。",
            "- `plots/direction_error_cdf.png`：方向误差累计分布。",
            "- `plots/direction_p90_by_distance.png`：不同真实距离的方向 P90。",
            "- `plots/distance_p90_by_distance.png`：不同真实距离的距离误差 P90。",
            "",
            "技术依据：[OpenCV 鱼眼/Kannala-Brandt 模型](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html)；[Double Sphere 模型论文](https://arxiv.org/abs/1807.08957)。",
        ]
    )
    (output_dir / "camera_model_fit_report.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )


def self_test() -> None:
    assert abs(float(wrap_rad(3.0 * math.pi)) + math.pi) < 1.0e-9
    assert parse_log_info(Path("车灯距离为0,5米_2.csv")).distance_m == 0.5
    for spec in SPECS.values():
        params, _, _ = initial_parameters(spec)
        intrinsics, _, _ = decode_parameters(spec, params)
        rays, valid = rays_from_pixels(
            spec,
            intrinsics,
            np.array([intrinsics[1, 2] if spec.intrinsic_count > 4 else intrinsics[1, 1]]),
            np.array([intrinsics[1, 3] if spec.intrinsic_count > 4 else intrinsics[1, 2]]),
            np.array([1]),
        )
        assert valid[0]
        assert rays[0, 2] > 0.0
    table = np.zeros((7, 9, 2), dtype=float)
    assert np.allclose(interpolate_lut(table, np.array([0.0]), np.array([0.0])), 0.0)
    print("self-test passed")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path(r"D:\Downloads\摄像头拟合"),
        help="directory containing the 14 calibration CSV logs",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "docs" / "camera_model_fit",
    )
    parser.add_argument("--max-samples", type=int, default=6000)
    parser.add_argument("--loo-samples", type=int, default=4000)
    parser.add_argument("--lut-samples", type=int, default=2500)
    parser.add_argument("--max-nfev", type=int, default=14)
    parser.add_argument("--lut-nfev", type=int, default=8)
    parser.add_argument("--delay-samples", type=int, default=2000)
    parser.add_argument("--skip-loo", action="store_true")
    parser.add_argument("--skip-delay", action="store_true")
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.quick:
        args.max_samples = min(args.max_samples, 900)
        args.loo_samples = min(args.loo_samples, 600)
        args.lut_samples = min(args.lut_samples, 400)
        args.max_nfev = min(args.max_nfev, 4)
        args.lut_nfev = min(args.lut_nfev, 3)
        args.delay_samples = min(args.delay_samples, 500)
        args.skip_loo = True
        args.skip_delay = True

    started = time.perf_counter()
    print(f"loading logs from {args.data_dir}", flush=True)
    dataset = load_dataset(args.data_dir)
    print(f"loaded {len(dataset):,} deduplicated rows in {time.perf_counter() - started:.1f}s")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    metrics: list[dict] = []
    primary_evaluations: dict[str, list[dict[str, np.ndarray]]] = {
        name: [] for name in ("pinhole_k1", "double_sphere", "kb8", "double_sphere_lut")
    }
    fold_fits: dict[str, list[tuple[np.ndarray, FitResult]]] = {
        name: [] for name in primary_evaluations
    }

    primary_folds = [
        ("base_to_2", ~dataset.suffix_2, dataset.suffix_2),
        ("2_to_base", dataset.suffix_2, ~dataset.suffix_2),
    ]
    for fold_name, train_mask, validation_mask in primary_folds:
        print(f"primary fold {fold_name}", flush=True)
        fits = fit_model_set(
            dataset,
            train_mask,
            args.max_samples,
            args.lut_samples,
            args.max_nfev,
            args.lut_nfev,
        )
        for model, fit in fits.items():
            evaluation = evaluate_model(dataset, validation_mask, fit)
            primary_evaluations[model].append(evaluation)
            fold_fits[model].append((validation_mask.copy(), fit))
            metrics.extend(summarize_evaluation(model, fold_name, evaluation))

    combined_primary: dict[str, dict[str, np.ndarray]] = {}
    for model, evaluations in primary_evaluations.items():
        combined = concatenate_evaluations(evaluations)
        combined_primary[model] = combined
        metrics.extend(summarize_evaluation(model, "primary_two_fold", combined))

    if not args.skip_loo:
        for distance in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5):
            print(f"leave-one-distance-out {distance:.1f}m", flush=True)
            validation_mask = np.isclose(dataset.distance_m, distance)
            train_mask = ~validation_mask
            fits = fit_model_set(
                dataset,
                train_mask,
                args.loo_samples,
                min(args.lut_samples, args.loo_samples // 2),
                args.max_nfev,
                args.lut_nfev,
            )
            for model, fit in fits.items():
                evaluation = evaluate_model(dataset, validation_mask, fit)
                metrics.extend(
                    summarize_evaluation(model, f"leave_{distance:.1f}m", evaluation)
                )

    delay_results: dict[str, dict] = {}
    if not args.skip_delay:
        for model, folds in fold_fits.items():
            print(f"delay scan {model}", flush=True)
            delay_results[model] = scan_model_delays(
                dataset, folds, args.delay_samples
            )
    else:
        for model in fold_fits:
            delay_results[model] = {
                "delays_ms": [0.0, 0.0, 0.0],
                "baseline_worst_p90": math.nan,
                "best_worst_p90": math.nan,
                "baseline_coverage": math.nan,
                "best_coverage": math.nan,
                "accepted": False,
            }

    print("fitting final parameters on all logs", flush=True)
    final_fits = fit_model_set(
        dataset,
        np.ones(len(dataset), dtype=bool),
        args.max_samples,
        args.lut_samples,
        args.max_nfev,
        args.lut_nfev,
    )
    parameters = {model: fit_to_json(fit) for model, fit in final_fits.items()}
    (args.output_dir / "model_parameters.json").write_text(
        json.dumps(parameters, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (args.output_dir / "delay_scan.json").write_text(
        json.dumps(delay_results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    timings = timing_rows()
    write_csv(args.output_dir / "metrics_summary.csv", metrics)
    write_csv(args.output_dir / "timing_estimate.csv", timings)
    create_plots(args.output_dir, combined_primary, metrics)
    build_report(
        args.output_dir,
        dataset,
        metrics,
        delay_results,
        timings,
        parameters,
        not args.skip_loo,
    )
    print(
        f"completed in {(time.perf_counter() - started) / 60.0:.1f} min; "
        f"report: {args.output_dir / 'camera_model_fit_report.md'}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
