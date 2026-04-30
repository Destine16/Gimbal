#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd


RAD_TO_DEG = 180.0 / math.pi


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot gimbal scan target/actual tracking curves.")
    parser.add_argument("csv", type=Path, help="CSV captured by vision_debug_rtt_capture.py")
    parser.add_argument("--output", type=Path, default=None, help="Output PNG path")
    return parser.parse_args()


def require_columns(df: pd.DataFrame, columns: list[str], csv_path: Path) -> None:
    missing = [column for column in columns if column not in df.columns]
    if missing:
        raise ValueError(f"{csv_path} missing columns: {', '.join(missing)}")


def main() -> None:
    args = parse_args()
    df = pd.read_csv(args.csv)
    columns = [
        "tick_ms",
        "cmd_yaw_rad",
        "actual_yaw_rad",
        "cmd_pitch_rad",
        "actual_pitch_rad",
        "yaw_output_cmd",
        "pitch_output_cmd",
        "yaw_speed_ref_rad_s",
        "pitch_speed_ref_rad_s",
    ]
    require_columns(df, columns, args.csv)

    output = args.output
    if output is None:
        output = args.csv.with_name(f"{args.csv.stem}_tracking.png")
    output.parent.mkdir(parents=True, exist_ok=True)

    t = (df["tick_ms"] - df["tick_ms"].iloc[0]) / 1000.0
    yaw_ref_deg = df["cmd_yaw_rad"] * RAD_TO_DEG
    yaw_actual_deg = df["actual_yaw_rad"] * RAD_TO_DEG
    pitch_ref_deg = df["cmd_pitch_rad"] * RAD_TO_DEG
    pitch_actual_deg = df["actual_pitch_rad"] * RAD_TO_DEG
    yaw_error_deg = yaw_ref_deg - yaw_actual_deg
    pitch_error_deg = pitch_ref_deg - pitch_actual_deg

    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)

    axes[0].plot(t, yaw_ref_deg, label="yaw target", linewidth=1.0)
    axes[0].plot(t, yaw_actual_deg, label="yaw actual", linewidth=1.0)
    axes[0].set_ylabel("Yaw (deg)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(t, pitch_ref_deg, label="pitch target", linewidth=1.0)
    axes[1].plot(t, pitch_actual_deg, label="pitch actual", linewidth=1.0)
    axes[1].set_ylabel("Pitch (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].plot(t, yaw_error_deg, label="yaw error", linewidth=1.0)
    axes[2].plot(t, pitch_error_deg, label="pitch error", linewidth=1.0)
    axes[2].axhline(0.0, color="black", linewidth=0.6)
    axes[2].set_ylabel("Error (deg)")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right")

    axes[3].plot(t, df["yaw_output_cmd"], label="yaw output", linewidth=1.0)
    axes[3].plot(t, df["pitch_output_cmd"], label="pitch output", linewidth=1.0)
    axes[3].plot(t, df["yaw_speed_ref_rad_s"] * 1000.0, label="yaw speed ref x1000", linewidth=0.8, alpha=0.7)
    axes[3].plot(t, df["pitch_speed_ref_rad_s"] * 1000.0, label="pitch speed ref x1000", linewidth=0.8, alpha=0.7)
    axes[3].axhline(0.0, color="black", linewidth=0.6)
    axes[3].set_ylabel("Output raw")
    axes[3].set_xlabel("Time (s)")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="upper right", ncols=2)

    fig.suptitle(args.csv.name)
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)

    print(f"plot={output}")


if __name__ == "__main__":
    main()
