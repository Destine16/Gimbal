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


YAW_PERF_STEP_MODE = 5
PITCH_PERF_STEP_MODE = 6
STEP_PHASE = 5

EXPECTED_TARGETS_DEG = {
    "yaw": [3.0, 5.0, 10.0, 20.0],
    "pitch": [3.0, -3.0, -10.0, 10.0],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze gimbal performance step-test RTT CSV.")
    parser.add_argument("csv", type=Path, help="CSV captured by gimbal_sysid_rtt_capture.py")
    parser.add_argument("--axis", choices=["auto", "yaw", "pitch"], default="auto")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--settle-band-ratio", type=float, default=0.02)
    parser.add_argument("--settle-band-min-deg", type=float, default=0.3)
    parser.add_argument("--target-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--no-plots", action="store_true")
    return parser.parse_args()


def analysis_dir_for(csv_path: Path, output_dir: Path | None, axis: str) -> Path:
    if output_dir is not None:
        return output_dir
    return csv_path.parent / "analysis" / f"{csv_path.stem}_{axis}_perf_step"


def finite_mean(values: np.ndarray) -> float:
    values = values[np.isfinite(values)]
    return float(np.mean(values)) if values.size else float("nan")


def finite_median(values: np.ndarray | pd.Series) -> float:
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    return float(np.median(arr)) if arr.size else float("nan")


def crossing_time(t: np.ndarray, value: np.ndarray, threshold: float) -> float:
    hits = np.flatnonzero(value >= threshold)
    if hits.size == 0:
        return float("nan")
    idx = int(hits[0])
    if idx == 0:
        return float(t[0])
    x0 = float(value[idx - 1])
    x1 = float(value[idx])
    t0 = float(t[idx - 1])
    t1 = float(t[idx])
    if abs(x1 - x0) < 1e-12:
        return t1
    return t0 + (threshold - x0) * (t1 - t0) / (x1 - x0)


def infer_axis(df: pd.DataFrame, requested: str) -> str:
    if requested != "auto":
        return requested
    modes = set(df["mode"].dropna().astype(int).unique().tolist())
    if PITCH_PERF_STEP_MODE in modes:
        return "pitch"
    if YAW_PERF_STEP_MODE in modes:
        return "yaw"
    raise ValueError("CSV does not contain yaw/pitch performance step mode")


def nearest_expected(axis: str, target_offset_deg: float, tolerance_deg: float) -> float | None:
    expected = EXPECTED_TARGETS_DEG[axis]
    best = min(expected, key=lambda item: abs(item - target_offset_deg))
    if abs(best - target_offset_deg) <= tolerance_deg:
        return best
    return None


def baseline_ref(df: pd.DataFrame, mode: int) -> float:
    mode_df = df[df["mode"].astype(int) == mode]
    baseline = mode_df[mode_df["phase"].astype(int) == 1]
    if not baseline.empty:
        return finite_median(baseline["angle_ref_rad"])
    zero_stage = mode_df[np.abs(mode_df["target_offset_rad"]) < math.radians(0.5)] if "target_offset_rad" in mode_df else pd.DataFrame()
    if not zero_stage.empty:
        return finite_median(zero_stage["angle_ref_rad"])
    return finite_median(mode_df["angle_ref_rad"])


def analyze_stage(stage: pd.DataFrame, axis: str, target_label_deg: float, args: argparse.Namespace) -> dict[str, float | int | str]:
    stage = stage.sort_values("tick_ms").copy()
    t = (stage["tick_ms"].to_numpy(dtype=float) - float(stage["tick_ms"].iloc[0])) / 1000.0
    target = finite_median(stage["angle_ref_rad"])
    actual = stage["angle_feedback_rad"].to_numpy(dtype=float)
    speed = stage["speed_feedback_rad_s"].to_numpy(dtype=float)
    output = stage["output_cmd"].to_numpy(dtype=float)
    current = stage["current_feedback_raw"].to_numpy(dtype=float)

    first_n = max(3, int(0.05 * len(actual)))
    last_n = max(3, int(0.20 * len(actual)))
    start_actual = finite_mean(actual[:first_n])
    steady_actual = finite_mean(actual[-last_n:])
    target_delta = target - start_actual
    direction = 1.0 if target_delta >= 0.0 else -1.0
    response = direction * (actual - start_actual)
    final_response = max(abs(target_delta), math.radians(0.1))

    peak_idx = int(np.argmax(response)) if response.size else 0
    peak_response = float(response[peak_idx]) if response.size else float("nan")
    peak_time = float(t[peak_idx]) if response.size else float("nan")
    overshoot = max(0.0, peak_response - final_response) / final_response

    rise10 = crossing_time(t, response, 0.10 * final_response)
    rise90 = crossing_time(t, response, 0.90 * final_response)
    rise_time = rise90 - rise10 if math.isfinite(rise10) and math.isfinite(rise90) else float("nan")

    settle_band = max(args.settle_band_ratio * final_response, math.radians(args.settle_band_min_deg))
    abs_error = np.abs(actual - target)
    settling_time = float("nan")
    for idx in range(len(t)):
        if np.all(abs_error[idx:] <= settle_band):
            settling_time = float(t[idx])
            break

    steady_error = target - steady_actual
    return {
        "axis": axis,
        "seq_index": int(finite_median(stage["seq_index"])),
        "target_label_deg": float(target_label_deg),
        "duration_s": float(t[-1] - t[0]) if len(t) else float("nan"),
        "target_deg": math.degrees(target),
        "start_actual_deg": math.degrees(start_actual),
        "steady_actual_deg": math.degrees(steady_actual),
        "rise_time_s": float(rise_time),
        "peak_time_s": float(peak_time),
        "settling_time_s": float(settling_time),
        "overshoot_percent": float(100.0 * overshoot),
        "steady_error_deg": math.degrees(steady_error),
        "rmse_deg": math.degrees(float(math.sqrt(np.mean((stage["angle_ref_rad"].to_numpy(dtype=float) - actual) ** 2)))),
        "max_error_deg": math.degrees(float(np.max(np.abs(stage["angle_ref_rad"].to_numpy(dtype=float) - actual)))),
        "peak_speed_rad_s": float(np.max(np.abs(speed))),
        "output_abs_max_raw": float(np.max(np.abs(output))),
        "current_abs_max_raw": float(np.max(np.abs(current))),
    }


def plot_stage(stage: pd.DataFrame, row: dict[str, float | int | str], output_dir: Path) -> str:
    label = f"{row['target_label_deg']:+.0f}deg"
    safe_label = label.replace("+", "p").replace("-", "m")
    t = (stage["tick_ms"].to_numpy(dtype=float) - float(stage["tick_ms"].iloc[0])) / 1000.0
    target_deg = np.degrees(stage["angle_ref_rad"].to_numpy(dtype=float))
    actual_deg = np.degrees(stage["angle_feedback_rad"].to_numpy(dtype=float))
    error_deg = target_deg - actual_deg

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(t, target_deg, label="target", linewidth=1.0)
    axes[0].plot(t, actual_deg, label="actual", linewidth=1.0)
    axes[0].set_ylabel(f"{row['axis']} angle (deg)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, error_deg, linewidth=0.9)
    axes[1].set_ylabel("error (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(t, stage["speed_feedback_rad_s"], label="speed", linewidth=0.9)
    axes[2].plot(t, stage["output_cmd"], label="output_cmd", linewidth=0.8)
    axes[2].set_ylabel("speed/output")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)
    title = (
        f"{row['axis']} step {label}: rise={row['rise_time_s']:.3f}s, "
        f"peak={row['peak_time_s']:.3f}s, settle={row['settling_time_s']:.3f}s, "
        f"overshoot={row['overshoot_percent']:.1f}%, steady={row['steady_error_deg']:.3f}deg"
    )
    fig.suptitle(title)
    fig.tight_layout()
    path = output_dir / f"step_{safe_label}.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return str(path)


def make_markdown_table(axis: str, rows: list[dict[str, float | int | str]], plot_paths: dict[float, str]) -> str:
    order = EXPECTED_TARGETS_DEG[axis]
    row_by_target = {float(row["target_label_deg"]): row for row in rows}
    lines = [
        f"# {axis} step performance",
        "",
        "| step | rise time tr (s) | peak time tp (s) | settling time ts (s) | overshoot (%) | steady error (deg) | curve |",
        "|---:|---:|---:|---:|---:|---:|---|",
    ]
    for target in order:
        row = row_by_target.get(float(target))
        if row is None:
            lines.append(f"| {target:+.0f}deg | - | - | - | - | - | - |")
            continue
        plot = plot_paths.get(float(target), "")
        curve = Path(plot).name if plot else "-"
        lines.append(
            f"| {target:+.0f}deg | {row['rise_time_s']:.4f} | {row['peak_time_s']:.4f} | "
            f"{row['settling_time_s']:.4f} | {row['overshoot_percent']:.2f} | "
            f"{row['steady_error_deg']:.4f} | {curve} |"
        )
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    df = load_numeric_csv(args.csv)
    axis = infer_axis(df, args.axis)
    mode = YAW_PERF_STEP_MODE if axis == "yaw" else PITCH_PERF_STEP_MODE
    output_dir = analysis_dir_for(args.csv, args.output_dir, axis)
    output_dir.mkdir(parents=True, exist_ok=True)

    mode_df = df[df["mode"].astype(int) == mode].copy()
    step_df = mode_df[mode_df["phase"].astype(int) == STEP_PHASE].copy()
    if len(step_df) < 32:
        print("not enough performance step samples found")
        return 2

    required = ["tick_ms", "seq_index", "angle_ref_rad", "angle_feedback_rad", "speed_feedback_rad_s", "output_cmd", "current_feedback_raw"]
    step_df = step_df.dropna(subset=required)
    base_ref = baseline_ref(df, mode)

    rows: list[dict[str, float | int | str]] = []
    stage_by_target: dict[float, pd.DataFrame] = {}
    for _, stage in step_df.groupby("seq_index"):
        target_offset_deg = math.degrees(finite_median(stage["angle_ref_rad"]) - base_ref)
        target_label = nearest_expected(axis, target_offset_deg, args.target_tolerance_deg)
        if target_label is None:
            continue
        rows.append(analyze_stage(stage, axis, target_label, args))
        stage_by_target[float(target_label)] = stage

    rows.sort(key=lambda row: EXPECTED_TARGETS_DEG[axis].index(float(row["target_label_deg"])))
    metrics_df = pd.DataFrame(rows)
    metrics_csv = output_dir / f"{axis}_perf_step_metrics.csv"
    result_json = output_dir / f"{axis}_perf_step_result.json"
    table_md = output_dir / f"{axis}_perf_step_table.md"
    metrics_df.to_csv(metrics_csv, index=False)

    plot_paths: dict[float, str] = {}
    if not args.no_plots:
        for row in rows:
            target = float(row["target_label_deg"])
            plot_paths[target] = plot_stage(stage_by_target[target], row, output_dir)

    table_md.write_text(make_markdown_table(axis, rows, plot_paths), encoding="utf-8")
    result = {
        "input_csv": str(args.csv),
        "axis": axis,
        "mode": mode,
        "output_dir": str(output_dir),
        "metrics_csv": str(metrics_csv),
        "table_md": str(table_md),
        "plot_paths": list(plot_paths.values()),
        "missing_targets_deg": [target for target in EXPECTED_TARGETS_DEG[axis] if target not in {float(row["target_label_deg"]) for row in rows}],
    }
    result_json.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"axis={axis}")
    print(f"output_dir={output_dir}")
    print(f"metrics_csv={metrics_csv}")
    print(f"table_md={table_md}")
    if result["missing_targets_deg"]:
        print(f"missing_targets_deg={result['missing_targets_deg']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
