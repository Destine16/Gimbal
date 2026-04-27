#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


GYRO_COLUMNS = ("imu_gyro_x_rad_s", "imu_gyro_y_rad_s", "imu_gyro_z_rad_s")
ACCEL_COLUMNS = ("imu_accel_x_m_s2", "imu_accel_y_m_s2", "imu_accel_z_m_s2")

AXES = {
    "x": np.array([1.0, 0.0, 0.0]),
    "y": np.array([0.0, 1.0, 0.0]),
    "z": np.array([0.0, 0.0, 1.0]),
}


def load_vectors(path: Path, columns: tuple[str, str, str]) -> np.ndarray:
    vectors: list[list[float]] = []
    with path.open(newline="") as fp:
        reader = csv.DictReader(fp)
        missing = [name for name in columns if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{path} missing columns: {', '.join(missing)}")
        for row in reader:
            try:
                vectors.append([float(row[name]) for name in columns])
            except ValueError:
                continue
    if not vectors:
        raise ValueError(f"{path} has no usable rows")
    return np.asarray(vectors, dtype=float)


def normalize(v: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(v))
    if norm <= 1.0e-12:
        raise ValueError("zero-length vector")
    return v / norm


def mean_static_accel(path: Path) -> tuple[np.ndarray, int, float]:
    accel = load_vectors(path, ACCEL_COLUMNS)
    norms = np.linalg.norm(accel, axis=1)
    mask = (norms > 5.0) & (norms < 15.0)
    if int(np.count_nonzero(mask)) < 20:
        raise ValueError(f"{path} has too few static accel samples")
    unit = accel[mask] / norms[mask, None]
    mean_vec = normalize(np.mean(unit, axis=0))
    residual_deg = float(np.std(np.rad2deg(np.arccos(np.clip(unit @ mean_vec, -1.0, 1.0)))))
    return mean_vec, int(np.count_nonzero(mask)), residual_deg


def mean_rotation_axis(path: Path, nominal_axis: np.ndarray, min_gyro_norm: float) -> tuple[np.ndarray, int, float]:
    gyro = load_vectors(path, GYRO_COLUMNS)
    norms = np.linalg.norm(gyro, axis=1)
    mask = norms > min_gyro_norm
    if int(np.count_nonzero(mask)) < 20:
        raise ValueError(f"{path} has too few moving gyro samples; lower --min-gyro-norm or recapture")

    vectors = gyro[mask]
    unit = vectors / norms[mask, None]
    signed_unit = unit.copy()
    signs = np.sign(signed_unit @ nominal_axis)
    signs[signs == 0.0] = 1.0
    signed_unit *= signs[:, None]
    mean_vec = normalize(np.mean(signed_unit, axis=0))
    residual_deg = float(np.std(np.rad2deg(np.arccos(np.clip(signed_unit @ mean_vec, -1.0, 1.0)))))
    return mean_vec, int(np.count_nonzero(mask)), residual_deg


def wahba_rotation(measured: list[np.ndarray], desired: list[np.ndarray], weights: list[float]) -> np.ndarray:
    b = np.zeros((3, 3), dtype=float)
    for m, d, w in zip(measured, desired, weights):
        b += w * np.outer(normalize(d), normalize(m))
    u, _, vt = np.linalg.svd(b)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u @ vt)
    return u @ correction @ vt


def matrix_to_imu_param_euler(r: np.ndarray) -> tuple[float, float, float]:
    # Match the matrix used by IMU_Param_Correction(): yaw(alpha), pitch(beta), roll(gamma).
    pitch = math.asin(max(-1.0, min(1.0, float(r[2, 1]))))
    yaw = math.atan2(float(r[0, 1]), float(r[1, 1]))
    roll = math.atan2(float(-r[2, 0]), float(r[2, 2]))
    return yaw, pitch, roll


def angle_error_deg(r: np.ndarray, measured: np.ndarray, desired: np.ndarray) -> float:
    corrected = normalize(r @ normalize(measured))
    return math.degrees(math.acos(max(-1.0, min(1.0, float(corrected @ normalize(desired))))))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fit small IMU mount correction from RTT CSV logs.")
    parser.add_argument("--static", type=Path, required=True, help="CSV captured while the gimbal is mechanically level and still")
    parser.add_argument("--yaw", type=Path, required=True, help="CSV captured while yaw axis moves back and forth")
    parser.add_argument("--pitch", type=Path, required=True, help="CSV captured while pitch axis moves back and forth")
    parser.add_argument("--yaw-axis", choices=AXES.keys(), default="z", help="Expected yaw gyro axis in current firmware frame")
    parser.add_argument("--pitch-axis", choices=AXES.keys(), default="x", help="Expected pitch gyro axis in current firmware frame")
    parser.add_argument("--min-gyro-norm", type=float, default=0.05, help="Minimum gyro norm used for rotation-axis samples")
    parser.add_argument("--output", type=Path, default=None, help="Optional JSON output path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    yaw_axis = AXES[args.yaw_axis]
    pitch_axis = AXES[args.pitch_axis]
    gravity_axis = AXES["z"]

    static_vec, static_n, static_scatter = mean_static_accel(args.static)
    yaw_vec, yaw_n, yaw_scatter = mean_rotation_axis(args.yaw, yaw_axis, args.min_gyro_norm)
    pitch_vec, pitch_n, pitch_scatter = mean_rotation_axis(args.pitch, pitch_axis, args.min_gyro_norm)

    measured = [static_vec, yaw_vec, pitch_vec]
    desired = [gravity_axis, yaw_axis, pitch_axis]
    weights = [1.0, 1.0, 1.0]
    r = wahba_rotation(measured, desired, weights)
    yaw, pitch, roll = matrix_to_imu_param_euler(r)

    residuals = {
        "static_gravity_deg": angle_error_deg(r, static_vec, gravity_axis),
        "yaw_axis_deg": angle_error_deg(r, yaw_vec, yaw_axis),
        "pitch_axis_deg": angle_error_deg(r, pitch_vec, pitch_axis),
    }
    result = {
        "samples": {
            "static": static_n,
            "yaw": yaw_n,
            "pitch": pitch_n,
        },
        "input_scatter_deg": {
            "static": static_scatter,
            "yaw": yaw_scatter,
            "pitch": pitch_scatter,
        },
        "measured_unit_vectors": {
            "static_gravity": static_vec.tolist(),
            "yaw_axis": yaw_vec.tolist(),
            "pitch_axis": pitch_vec.tolist(),
        },
        "correction_matrix": r.tolist(),
        "imu_param_offsets_rad": {
            "Yaw": yaw,
            "Pitch": pitch,
            "Roll": roll,
        },
        "imu_param_offsets_deg": {
            "Yaw": math.degrees(yaw),
            "Pitch": math.degrees(pitch),
            "Roll": math.degrees(roll),
        },
        "residual_after_correction_deg": residuals,
    }

    print("samples:", result["samples"])
    print("input scatter deg:", result["input_scatter_deg"])
    print("correction matrix:")
    for row in r:
        print("  {" + ", ".join(f"{value:+.9f}f" for value in row) + "}")
    print("IMU_Param offsets:")
    print(f"  Yaw   = {yaw:+.9f} rad ({math.degrees(yaw):+.5f} deg)")
    print(f"  Pitch = {pitch:+.9f} rad ({math.degrees(pitch):+.5f} deg)")
    print(f"  Roll  = {roll:+.9f} rad ({math.degrees(roll):+.5f} deg)")
    print("residual after correction deg:", residuals)

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
