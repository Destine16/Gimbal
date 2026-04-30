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

from analyze_yaw_prbs_sysid import load_numeric_csv  # noqa: E402


YAW_PERF_SINE_MODE = 7
PITCH_PERF_SINE_MODE = 8
SINE_PHASE = 6
SINE_CASES = [
    {"label": "A=5deg T=1s", "amplitude_deg": 5.0, "period_s": 1.0},
    {"label": "A=20deg T=2s", "amplitude_deg": 20.0, "period_s": 2.0},
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze gimbal sine tracking performance RTT CSV.")
    parser.add_argument("csv", type=Path, help="CSV captured by gimbal_sysid_rtt_capture.py")
    parser.add_argument("--axis", choices=["auto", "yaw", "pitch"], default="auto")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--skip-cycles", type=float, default=1.0)
    parser.add_argument("--no-plots", action="store_true")
    return parser.parse_args()


def analysis_dir_for(csv_path: Path, output_dir: Path | None, axis: str) -> Path:
    if output_dir is not None:
        return output_dir
    return csv_path.parent / "analysis" / f"{csv_path.stem}_{axis}_perf_sine"


def finite_median(values: np.ndarray | pd.Series) -> float:
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    return float(np.median(arr)) if arr.size else float("nan")


def infer_axis(df: pd.DataFrame, requested: str) -> str:
    if requested != "auto":
        return requested
    modes = set(df["mode"].dropna().astype(int).unique().tolist())
    if PITCH_PERF_SINE_MODE in modes:
        return "pitch"
    if YAW_PERF_SINE_MODE in modes:
        return "yaw"
    raise ValueError("CSV does not contain yaw/pitch performance sine mode")


def fit_sine(t: np.ndarray, y: np.ndarray, period_s: float) -> tuple[float, float, float]:
    omega = 2.0 * math.pi / period_s
    matrix = np.column_stack([np.ones_like(t), np.sin(omega * t), np.cos(omega * t)])
    coeff, *_ = np.linalg.lstsq(matrix, y, rcond=None)
    offset = float(coeff[0])
    sin_coeff = float(coeff[1])
    cos_coeff = float(coeff[2])
    amplitude = math.sqrt(sin_coeff * sin_coeff + cos_coeff * cos_coeff)
    phase_rad = math.atan2(cos_coeff, sin_coeff)
    return offset, amplitude, phase_rad


def analyze_stage(stage: pd.DataFrame, axis: str, case: dict[str, float | str], args: argparse.Namespace) -> dict[str, float | int | str]:
    stage = stage.sort_values("tick_ms").copy()
    t_all = (stage["tick_ms"].to_numpy(dtype=float) - float(stage["tick_ms"].iloc[0])) / 1000.0
    period_s = float(case["period_s"])
    keep = t_all >= args.skip_cycles * period_s
    if np.count_nonzero(keep) < 16:
        keep = np.ones_like(t_all, dtype=bool)

    t = t_all[keep] - float(t_all[keep][0])
    ref = stage["angle_ref_rad"].to_numpy(dtype=float)[keep]
    actual = stage["angle_feedback_rad"].to_numpy(dtype=float)[keep]
    output = stage["output_cmd"].to_numpy(dtype=float)[keep]
    current = stage["current_feedback_raw"].to_numpy(dtype=float)[keep]
    error = ref - actual

    ref_offset, ref_amp, ref_phase = fit_sine(t, ref, period_s)
    actual_offset, actual_amp, actual_phase = fit_sine(t, actual, period_s)
    phase_lag_rad = actual_phase - ref_phase
    while phase_lag_rad > math.pi:
        phase_lag_rad -= 2.0 * math.pi
    while phase_lag_rad < -math.pi:
        phase_lag_rad += 2.0 * math.pi

    rmse_rad = math.sqrt(float(np.mean(error * error)))
    mae_rad = float(np.mean(np.abs(error)))
    max_error_rad = float(np.max(np.abs(error)))
    return {
        "axis": axis,
        "seq_index": int(finite_median(stage["seq_index"])),
        "label": str(case["label"]),
        "amplitude_deg": float(case["amplitude_deg"]),
        "period_s": period_s,
        "sample_count": int(len(t)),
        "mae_deg": math.degrees(mae_rad),
        "rmse_deg": math.degrees(rmse_rad),
        "max_error_deg": math.degrees(max_error_rad),
        "ref_fit_amp_deg": math.degrees(ref_amp),
        "actual_fit_amp_deg": math.degrees(actual_amp),
        "amplitude_ratio": actual_amp / ref_amp if ref_amp > 1e-9 else float("nan"),
        "phase_lag_deg": math.degrees(phase_lag_rad),
        "ref_offset_deg": math.degrees(ref_offset),
        "actual_offset_deg": math.degrees(actual_offset),
        "output_abs_max_raw": float(np.max(np.abs(output))),
        "current_abs_max_raw": float(np.max(np.abs(current))),
    }


def plot_stage(stage: pd.DataFrame, row: dict[str, float | int | str], output_dir: Path, args: argparse.Namespace) -> str:
    label = str(row["label"]).replace("=", "").replace(" ", "_").replace("/", "_")
    t_all = (stage["tick_ms"].to_numpy(dtype=float) - float(stage["tick_ms"].iloc[0])) / 1000.0
    period_s = float(row["period_s"])
    keep = t_all >= args.skip_cycles * period_s
    if np.count_nonzero(keep) < 16:
        keep = np.ones_like(t_all, dtype=bool)
    t = t_all[keep] - float(t_all[keep][0])
    target_deg = np.degrees(stage["angle_ref_rad"].to_numpy(dtype=float)[keep])
    actual_deg = np.degrees(stage["angle_feedback_rad"].to_numpy(dtype=float)[keep])
    error_deg = target_deg - actual_deg

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(t, target_deg, label="target", linewidth=1.0)
    axes[0].plot(t, actual_deg, label="actual", linewidth=1.0)
    axes[0].set_ylabel(f"{row['axis']} angle (deg)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, error_deg, linewidth=0.9)
    axes[1].set_ylabel("error (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(t, stage["output_cmd"].to_numpy(dtype=float)[keep], label="output_cmd", linewidth=0.8)
    axes[2].plot(t, stage["current_feedback_raw"].to_numpy(dtype=float)[keep], label="current_feedback", linewidth=0.8)
    axes[2].set_ylabel("raw")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)
    title = (
        f"{row['axis']} {row['label']}: MAE={row['mae_deg']:.3f}deg, "
        f"RMSE={row['rmse_deg']:.3f}deg, Max={row['max_error_deg']:.3f}deg, "
        f"gain={row['amplitude_ratio']:.3f}, lag={row['phase_lag_deg']:.1f}deg"
    )
    fig.suptitle(title)
    fig.tight_layout()
    path = output_dir / f"sine_{label}.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return str(path)


def make_markdown_table(axis: str, rows: list[dict[str, float | int | str]], plot_paths: dict[str, str]) -> str:
    row_by_label = {str(row["label"]): row for row in rows}
    lines = [
        f"# {axis} sine tracking performance",
        "",
        "| case | MAE (deg) | RMSE (deg) | Max Error (deg) | amplitude ratio | phase lag (deg) | curve |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    for case in SINE_CASES:
        label = str(case["label"])
        row = row_by_label.get(label)
        if row is None:
            lines.append(f"| {label} | - | - | - | - | - | - |")
            continue
        curve = Path(plot_paths.get(label, "")).name or "-"
        lines.append(
            f"| {label} | {row['mae_deg']:.4f} | {row['rmse_deg']:.4f} | "
            f"{row['max_error_deg']:.4f} | {row['amplitude_ratio']:.4f} | "
            f"{row['phase_lag_deg']:.2f} | {curve} |"
        )
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    df = load_numeric_csv(args.csv)
    axis = infer_axis(df, args.axis)
    mode = YAW_PERF_SINE_MODE if axis == "yaw" else PITCH_PERF_SINE_MODE
    output_dir = analysis_dir_for(args.csv, args.output_dir, axis)
    output_dir.mkdir(parents=True, exist_ok=True)

    mode_df = df[df["mode"].astype(int) == mode].copy()
    sine_df = mode_df[mode_df["phase"].astype(int) == SINE_PHASE].copy()
    if len(sine_df) < 32:
        print("not enough performance sine samples found")
        return 2

    required = ["tick_ms", "seq_index", "angle_ref_rad", "angle_feedback_rad", "output_cmd", "current_feedback_raw"]
    sine_df = sine_df.dropna(subset=required)
    groups = [(int(seq), stage.sort_values("tick_ms")) for seq, stage in sine_df.groupby("seq_index")]
    groups.sort(key=lambda item: item[0])

    rows: list[dict[str, float | int | str]] = []
    stage_by_label: dict[str, pd.DataFrame] = {}
    for idx, (_, stage) in enumerate(groups[: len(SINE_CASES)]):
        case = SINE_CASES[idx]
        row = analyze_stage(stage, axis, case, args)
        rows.append(row)
        stage_by_label[str(case["label"])] = stage

    metrics_df = pd.DataFrame(rows)
    metrics_csv = output_dir / f"{axis}_perf_sine_metrics.csv"
    result_json = output_dir / f"{axis}_perf_sine_result.json"
    table_md = output_dir / f"{axis}_perf_sine_table.md"
    metrics_df.to_csv(metrics_csv, index=False)

    plot_paths: dict[str, str] = {}
    if not args.no_plots:
        for row in rows:
            label = str(row["label"])
            plot_paths[label] = plot_stage(stage_by_label[label], row, output_dir, args)

    table_md.write_text(make_markdown_table(axis, rows, plot_paths), encoding="utf-8")
    result = {
        "input_csv": str(args.csv),
        "axis": axis,
        "mode": mode,
        "output_dir": str(output_dir),
        "metrics_csv": str(metrics_csv),
        "table_md": str(table_md),
        "plot_paths": list(plot_paths.values()),
        "case_count": len(rows),
    }
    result_json.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"axis={axis}")
    print(f"output_dir={output_dir}")
    print(f"metrics_csv={metrics_csv}")
    print(f"table_md={table_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
