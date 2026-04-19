#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from analyze_yaw_prbs_sysid import compute_step_metrics, load_numeric_csv, response_metrics, uniform_resample  # noqa: E402


YAW_STEP_MODE = 3
STEP_PHASE = 5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze small yaw step validation RTT CSV.")
    parser.add_argument("csv", type=Path, help="CSV captured by gimbal_sysid_rtt_capture.py")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--output-limit", type=float, default=5000.0)
    parser.add_argument("--sat-margin", type=float, default=0.98)
    parser.add_argument("--min-step-deg", type=float, default=5.0)
    parser.add_argument("--settle-band-ratio", type=float, default=0.02)
    parser.add_argument("--settle-band-min-deg", type=float, default=0.3)
    parser.add_argument("--no-plots", action="store_true")
    return parser.parse_args()


def analysis_dir_for(csv_path: Path, output_dir: Path | None) -> Path:
    if output_dir is not None:
        return output_dir
    return csv_path.parent / "analysis" / f"{csv_path.stem}_yaw_step"


def plot_step(mode_df: pd.DataFrame, step_df: pd.DataFrame, output_dir: Path) -> list[str]:
    paths: list[str] = []
    t = (mode_df["tick_ms"] - mode_df["tick_ms"].iloc[0]) / 1000.0
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    axes[0].plot(t, np.degrees(mode_df["angle_ref_rad"]), label="target", linewidth=1.0)
    axes[0].plot(t, np.degrees(mode_df["angle_feedback_rad"]), label="actual", linewidth=1.0)
    axes[0].set_ylabel("Yaw angle (deg)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, np.degrees(mode_df["angle_ref_rad"] - mode_df["angle_feedback_rad"]), linewidth=0.9)
    axes[1].set_ylabel("Error (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(t, mode_df["speed_feedback_rad_s"], label="speed", linewidth=0.9)
    axes[2].plot(t, mode_df["speed_ref_rad_s"], label="speed_ref", linewidth=0.8)
    axes[2].set_ylabel("Speed (rad/s)")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)
    axes[3].plot(t, mode_df["output_cmd"], label="output_cmd", linewidth=0.9)
    axes[3].plot(t, mode_df["voltage_ref_raw"], label="voltage_ref", linewidth=0.8)
    axes[3].set_ylabel("Raw output")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="best")
    axes[3].grid(True, alpha=0.3)
    fig.suptitle("Yaw small-step validation")
    fig.tight_layout()
    path = output_dir / "yaw_step_time_series.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    if not step_df.empty:
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axes[0].bar(step_df["seq_index"], step_df["overshoot_ratio"] * 100.0)
        axes[0].set_ylabel("Overshoot (%)")
        axes[0].grid(True, alpha=0.3)
        axes[1].bar(step_df["seq_index"], step_df["steady_error_deg"])
        axes[1].set_ylabel("Steady error (deg)")
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(step_df["seq_index"], step_df["rise_time_s"], marker="o", label="rise")
        axes[2].plot(step_df["seq_index"], step_df["settling_time_s"], marker="s", label="settling")
        axes[2].set_xlabel("Step seq_index")
        axes[2].set_ylabel("Time (s)")
        axes[2].legend(loc="best")
        axes[2].grid(True, alpha=0.3)
        fig.suptitle("Yaw small-step metrics")
        fig.tight_layout()
        path = output_dir / "yaw_step_metrics.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths.append(str(path))

    return paths


def finite_median(series: pd.Series) -> float:
    values = series.to_numpy(dtype=float)
    values = values[np.isfinite(values)]
    return float(np.median(values)) if len(values) else float("nan")


def main() -> int:
    args = parse_args()
    output_dir = analysis_dir_for(args.csv, args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    df = load_numeric_csv(args.csv)
    mode_df = df[df["mode"].astype(int) == YAW_STEP_MODE].copy()
    step_phase_df = mode_df[mode_df["phase"].astype(int) == STEP_PHASE].copy()
    if len(mode_df) < 64 or len(step_phase_df) < 32:
        print("not enough yaw step samples found")
        return 2

    required = [
        "angle_ref_rad",
        "angle_feedback_rad",
        "speed_ref_rad_s",
        "speed_feedback_rad_s",
        "output_cmd",
        "voltage_ref_raw",
        "seq_index",
    ]
    mode_df = mode_df.dropna(subset=required)
    step_phase_df = step_phase_df.dropna(subset=required)
    _, values, dt_s = uniform_resample(step_phase_df, required)
    overall = response_metrics(
        values["angle_ref_rad"],
        values["angle_feedback_rad"],
        values["output_cmd"],
        args.output_limit,
        args.sat_margin,
    )
    step_df = compute_step_metrics(step_phase_df, args)
    step_csv = output_dir / "yaw_step_metrics.csv"
    result_json = output_dir / "yaw_step_result.json"
    step_df.to_csv(step_csv, index=False)

    plot_paths: list[str] = []
    if not args.no_plots:
        plot_paths = plot_step(mode_df, step_df, output_dir)

    result = {
        "input_csv": str(args.csv),
        "output_dir": str(output_dir),
        "sample_period_s": float(dt_s),
        "sample_rate_hz": float(1.0 / dt_s),
        "overall_metrics": overall,
        "step_summary": {
            "step_count": int(len(step_df)),
            "median_overshoot_ratio": finite_median(step_df["overshoot_ratio"]) if not step_df.empty else float("nan"),
            "median_rise_time_s": finite_median(step_df["rise_time_s"]) if not step_df.empty else float("nan"),
            "median_settling_time_s": finite_median(step_df["settling_time_s"]) if not step_df.empty else float("nan"),
            "median_steady_error_deg": finite_median(step_df["steady_error_deg"]) if not step_df.empty else float("nan"),
        },
        "step_metrics_csv": str(step_csv),
        "plot_paths": plot_paths,
        "warnings": [],
    }
    if overall["output_saturation_ratio"] > 0.02:
        result["warnings"].append("output saturation is visible during small-step validation")
    if not step_df.empty and finite_median(step_df["overshoot_ratio"]) > 0.15:
        result["warnings"].append("median overshoot exceeds 15%; PID may be too aggressive")

    result_json.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"output_dir={output_dir}")
    print(f"tracking_rmse={overall['tracking_rmse_deg']:.4f} deg")
    print(f"tracking_max_abs={overall['tracking_max_abs_deg']:.4f} deg")
    print(f"output_saturation={100.0 * overall['output_saturation_ratio']:.2f}%")
    print(f"step_count={len(step_df)}")
    if not step_df.empty:
        print(f"median_overshoot={100.0 * finite_median(step_df['overshoot_ratio']):.2f}%")
        print(f"median_rise_time={finite_median(step_df['rise_time_s']):.4f}s")
        print(f"median_settling_time={finite_median(step_df['settling_time_s']):.4f}s")
        print(f"median_steady_error={finite_median(step_df['steady_error_deg']):.4f} deg")
    if result["warnings"]:
        for warning in result["warnings"]:
            print(f"warning: {warning}")
    print(f"json={result_json}")
    print(f"step_csv={step_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
