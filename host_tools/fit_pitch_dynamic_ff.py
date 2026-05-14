#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


PITCH_FAST_MULTISINE_MODE = 11
MULTISINE_PHASE = 7


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit pitch dynamic feedforward from fast multisine sysid data."
    )
    parser.add_argument("csv", type=Path, help="CSV captured by gimbal_sysid_rtt_capture.py")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--gravity-sin", type=float, default=-1115.8459)
    parser.add_argument("--gravity-offset", type=float, default=-97.4462)
    parser.add_argument("--pitch-output-sign", type=float, default=-1.0)
    parser.add_argument("--min-speed-ref", type=float, default=0.05)
    parser.add_argument("--output-limit", type=float, default=5000.0)
    parser.add_argument("--sat-margin", type=float, default=0.98)
    parser.add_argument("--speed-ff-limit", type=float, default=None)
    return parser.parse_args()


def finite_array(series: pd.Series) -> np.ndarray:
    return series.to_numpy(dtype=float)


def r2_score(y: np.ndarray, y_hat: np.ndarray) -> float:
    residual = y - y_hat
    ss_res = float(np.sum(residual * residual))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    if ss_tot <= 1e-12:
        return float("nan")
    return 1.0 - ss_res / ss_tot


def regression_metrics(y: np.ndarray, y_hat: np.ndarray) -> dict[str, float]:
    residual = y - y_hat
    return {
        "rmse_raw": float(math.sqrt(np.mean(residual * residual))),
        "mae_raw": float(np.mean(np.abs(residual))),
        "max_abs_raw": float(np.max(np.abs(residual))),
        "r2": r2_score(y, y_hat),
    }


def default_output_dir(csv_path: Path) -> Path:
    return csv_path.parent / "analysis" / f"{csv_path.stem}_pitch_dynamic_ff"


def main() -> int:
    args = parse_args()
    df = pd.read_csv(args.csv)
    required = {
        "mode",
        "phase",
        "angle_feedback_rad",
        "speed_ref_rad_s",
        "voltage_ref_raw",
        "output_ff_raw",
        "output_cmd",
    }
    missing = required.difference(df.columns)
    if missing:
        raise SystemExit(f"missing required columns: {sorted(missing)}")

    active = df[(df["mode"] == PITCH_FAST_MULTISINE_MODE) & (df["phase"] == MULTISINE_PHASE)].copy()
    if active.empty:
        raise SystemExit("no pitch fast multisine active samples found")

    output_cmd = finite_array(active["output_cmd"])
    not_saturated = np.abs(output_cmd) < args.output_limit * args.sat_margin
    speed_ref = finite_array(active["speed_ref_rad_s"])
    enough_speed = np.abs(speed_ref) >= args.min_speed_ref
    valid = not_saturated & enough_speed
    if int(np.sum(valid)) < 50:
        raise SystemExit(f"not enough fit samples: {int(np.sum(valid))}")

    theta = finite_array(active["angle_feedback_rad"])
    voltage_ref = finite_array(active["voltage_ref_raw"])
    current_ff = finite_array(active["output_ff_raw"])

    total_required = current_ff + args.pitch_output_sign * voltage_ref
    gravity_model = args.gravity_sin * np.sin(theta) + args.gravity_offset
    residual = total_required - gravity_model

    v = speed_ref[valid]
    y = residual[valid]
    speed_gain = float(np.dot(v, y) / np.dot(v, v))
    speed_ff = speed_gain * speed_ref
    if args.speed_ff_limit is None:
        observed_limit = float(np.percentile(np.abs(speed_ff[valid]), 95.0))
        speed_ff_limit = max(500.0, min(2000.0, observed_limit * 1.15))
    else:
        speed_ff_limit = args.speed_ff_limit
    speed_ff_limited = np.clip(speed_ff, -speed_ff_limit, speed_ff_limit)

    x_full = np.column_stack([np.sin(theta[valid]), np.ones(int(np.sum(valid))), speed_ref[valid]])
    full_gain = np.linalg.lstsq(x_full, total_required[valid], rcond=None)[0]
    full_pred = (
        full_gain[0] * np.sin(theta)
        + full_gain[1]
        + full_gain[2] * speed_ref
    )

    base_pred = gravity_model
    speed_pred = gravity_model + speed_gain * speed_ref
    speed_limited_pred = gravity_model + speed_ff_limited

    result = {
        "source_csv": str(args.csv),
        "fit_samples": int(np.sum(valid)),
        "active_samples": int(len(active)),
        "gravity_sin_raw": args.gravity_sin,
        "gravity_offset_raw": args.gravity_offset,
        "speed_gain_raw_per_rad_s": speed_gain,
        "speed_ff_limit_raw": speed_ff_limit,
        "full_fit": {
            "sin_raw": float(full_gain[0]),
            "offset_raw": float(full_gain[1]),
            "speed_gain_raw_per_rad_s": float(full_gain[2]),
        },
        "metrics_fit_window": {
            "gravity_only": regression_metrics(total_required[valid], base_pred[valid]),
            "gravity_plus_speed": regression_metrics(total_required[valid], speed_pred[valid]),
            "gravity_plus_speed_limited": regression_metrics(total_required[valid], speed_limited_pred[valid]),
            "full_fit": regression_metrics(total_required[valid], full_pred[valid]),
        },
        "speed_ref_range_rad_s": {
            "min": float(np.min(speed_ref[valid])),
            "max": float(np.max(speed_ref[valid])),
            "p95_abs": float(np.percentile(np.abs(speed_ref[valid]), 95.0)),
        },
        "recommended_macros": {
            "GIMBAL_PITCH_OUTPUT_FF_SPEED_RAW": speed_gain,
            "GIMBAL_PITCH_OUTPUT_FF_SPEED_MAX_RAW": speed_ff_limit,
        },
    }

    output_dir = args.output_dir or default_output_dir(args.csv)
    output_dir.mkdir(parents=True, exist_ok=True)
    result_path = output_dir / "pitch_dynamic_ff_result.json"
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    t = (finite_array(active["tick_ms"]) - float(active["tick_ms"].iloc[0])) / 1000.0
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    axes[0].plot(t, np.degrees(theta), label="pitch actual", linewidth=0.8)
    axes[0].plot(t, np.degrees(finite_array(active["angle_ref_rad"])), label="pitch target", linewidth=0.8)
    axes[0].set_ylabel("Angle (deg)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, residual, label="required residual", linewidth=0.7)
    axes[1].plot(t, speed_ff_limited, label="speed ff limited", linewidth=0.7)
    axes[1].set_ylabel("Raw output")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
    axes[2].scatter(speed_ref[valid], residual[valid], s=5, alpha=0.35, label="samples")
    v_line = np.linspace(float(np.min(speed_ref[valid])), float(np.max(speed_ref[valid])), 200)
    ff_line = np.clip(speed_gain * v_line, -speed_ff_limit, speed_ff_limit)
    axes[2].plot(v_line, ff_line, color="tab:red", label="fit")
    axes[2].set_xlabel("speed_ref (rad/s)")
    axes[2].set_ylabel("Residual raw")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)
    fig.suptitle("Pitch dynamic feedforward fit")
    fig.tight_layout()
    plot_path = output_dir / "pitch_dynamic_ff_fit.png"
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

    print(f"fit samples: {result['fit_samples']}/{result['active_samples']}")
    print(f"speed gain: {speed_gain:+.4f} raw/(rad/s)")
    print(f"speed ff limit: {speed_ff_limit:.1f} raw")
    print("fit-window metrics:")
    for name, metrics in result["metrics_fit_window"].items():
        print(
            f"  {name}: rmse={metrics['rmse_raw']:.2f}, "
            f"mae={metrics['mae_raw']:.2f}, r2={metrics['r2']:.4f}"
        )
    print("recommended:")
    print(f"#define GIMBAL_PITCH_OUTPUT_FF_SPEED_RAW     ({speed_gain:+.4f}f)")
    print(f"#define GIMBAL_PITCH_OUTPUT_FF_SPEED_MAX_RAW {speed_ff_limit:.1f}f")
    print(f"wrote {result_path}")
    print(f"wrote {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
