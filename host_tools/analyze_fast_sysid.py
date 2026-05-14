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

from analyze_yaw_prbs_sysid import (  # noqa: E402
    compute_step_metrics,
    estimate_frequency_response,
    fit_arx,
    load_numeric_csv,
    save_frequency_csv,
    scalar_frequency_metrics,
    uniform_resample,
)


YAW_PRBS_MODE = 1
PITCH_PRBS_MODE = 9
YAW_FAST_MULTISINE_MODE = 10
PITCH_FAST_MULTISINE_MODE = 11
PRBS_PHASE = 2
MULTISINE_PHASE = 7
DEFAULT_OUTPUT_LIMIT_RAW = 5000.0

MODE_INFO = {
    YAW_PRBS_MODE: ("yaw", "prbs", PRBS_PHASE),
    PITCH_PRBS_MODE: ("pitch", "prbs", PRBS_PHASE),
    YAW_FAST_MULTISINE_MODE: ("yaw", "fast_multisine", MULTISINE_PHASE),
    PITCH_FAST_MULTISINE_MODE: ("pitch", "fast_multisine", MULTISINE_PHASE),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze yaw/pitch fast-response PRBS or multisine RTT sysid data."
    )
    parser.add_argument("csv", type=Path, help="CSV captured by gimbal_sysid_rtt_capture.py")
    parser.add_argument("--axis", choices=["auto", "yaw", "pitch"], default="auto")
    parser.add_argument("--mode", type=int, default=None, help="Force GIMBAL_SYSID_MODE")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--output-limit", type=float, default=DEFAULT_OUTPUT_LIMIT_RAW)
    parser.add_argument("--sat-margin", type=float, default=0.98)
    parser.add_argument("--min-step-deg", type=float, default=3.0)
    parser.add_argument("--settle-band-ratio", type=float, default=0.02)
    parser.add_argument("--settle-band-min-deg", type=float, default=0.35)
    parser.add_argument("--fast-target-rmse-deg", type=float, default=0.8)
    parser.add_argument("--fast-target-max-error-deg", type=float, default=4.0)
    parser.add_argument("--no-plots", action="store_true")
    return parser.parse_args()


def infer_mode(df: pd.DataFrame, axis: str, forced_mode: int | None) -> int:
    if forced_mode is not None:
        if forced_mode not in MODE_INFO:
            raise ValueError(f"unsupported mode {forced_mode}; expected one of {sorted(MODE_INFO)}")
        return forced_mode

    modes = set(int(v) for v in df["mode"].dropna().astype(int).unique())
    candidates = [mode for mode in (YAW_FAST_MULTISINE_MODE, PITCH_FAST_MULTISINE_MODE, YAW_PRBS_MODE, PITCH_PRBS_MODE)
                  if mode in modes]
    if axis != "auto":
        candidates = [mode for mode in candidates if MODE_INFO[mode][0] == axis]
    if not candidates:
        raise ValueError(f"no supported fast sysid mode found; present modes={sorted(modes)}")
    return candidates[0]


def analysis_dir_for(csv_path: Path, output_dir: Path | None, axis: str, kind: str) -> Path:
    if output_dir is not None:
        return output_dir
    return csv_path.parent / "analysis" / f"{csv_path.stem}_{axis}_{kind}_fast"


def finite_mean(values: np.ndarray | pd.Series) -> float:
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    return float(np.mean(arr)) if arr.size else float("nan")


def finite_median(values: np.ndarray | pd.Series) -> float:
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    return float(np.median(arr)) if arr.size else float("nan")


def tracking_metrics(ref: np.ndarray, actual: np.ndarray, speed_ref: np.ndarray,
                     speed_feedback: np.ndarray, output_cmd: np.ndarray,
                     output_limit: float, sat_margin: float) -> dict[str, float]:
    err = ref - actual
    speed_err = speed_ref - speed_feedback
    output_rate = np.diff(output_cmd, prepend=output_cmd[0])
    sat_threshold = output_limit * sat_margin
    return {
        "tracking_rmse_deg": math.degrees(float(math.sqrt(np.mean(err * err)))),
        "tracking_mae_deg": math.degrees(float(np.mean(np.abs(err)))),
        "tracking_max_abs_deg": math.degrees(float(np.max(np.abs(err)))),
        "speed_rmse_rad_s": float(math.sqrt(np.mean(speed_err * speed_err))),
        "speed_max_abs_error_rad_s": float(np.max(np.abs(speed_err))),
        "output_saturation_ratio": float(np.mean(np.abs(output_cmd) >= sat_threshold)),
        "output_abs_max_raw": float(np.max(np.abs(output_cmd))),
        "output_rms_raw": float(math.sqrt(np.mean(output_cmd * output_cmd))),
        "output_rate_rms_raw": float(math.sqrt(np.mean(output_rate * output_rate))),
    }


def fast_score(metrics: dict[str, float], args: argparse.Namespace) -> float:
    rmse = metrics["tracking_rmse_deg"] / max(args.fast_target_rmse_deg, 1e-6)
    max_err = metrics["tracking_max_abs_deg"] / max(args.fast_target_max_error_deg, 1e-6)
    speed = metrics["speed_rmse_rad_s"] / 2.0
    sat = metrics["output_saturation_ratio"]
    output_rate = metrics["output_rate_rms_raw"] / max(args.output_limit, 1.0)
    return float(
        3.0 * rmse * rmse
        + 0.8 * max_err * max_err
        + 0.7 * speed * speed
        + 12.0 * sat * sat
        + 0.4 * output_rate * output_rate
    )


def phase_at(freq_result: dict[str, object], freq_hz: float) -> float:
    if not freq_result.get("ok"):
        return float("nan")
    freq = np.asarray(freq_result["freq_hz"], dtype=float)
    phase = np.asarray(freq_result["phase_rad"], dtype=float)
    coh = np.asarray(freq_result["coherence"], dtype=float)
    valid = np.isfinite(freq) & np.isfinite(phase) & (coh > 0.45)
    if not np.any(valid):
        return float("nan")
    idx = int(np.argmin(np.abs(freq[valid] - freq_hz)))
    phase_deg = float(np.degrees(phase[valid][idx]))
    return float((phase_deg + 180.0) % 360.0 - 180.0)


def extra_frequency_metrics(freq_result: dict[str, object]) -> dict[str, float]:
    if not freq_result.get("ok"):
        return {}
    freq = np.asarray(freq_result["freq_hz"], dtype=float)
    mag = np.asarray(freq_result["magnitude"], dtype=float)
    coh = np.asarray(freq_result["coherence"], dtype=float)
    valid = np.isfinite(freq) & np.isfinite(mag) & np.isfinite(coh)
    bands = {
        "mean_coherence_0p1_3hz": (0.1, 3.0),
        "mean_coherence_3_10hz": (3.0, 10.0),
        "mean_gain_0p1_3hz": (0.1, 3.0),
        "mean_gain_3_10hz": (3.0, 10.0),
    }
    result: dict[str, float] = {}
    for key, (lo, hi) in bands.items():
        mask = valid & (freq >= lo) & (freq <= hi)
        if key.startswith("mean_coherence"):
            result[key] = float(np.mean(coh[mask])) if np.any(mask) else float("nan")
        else:
            coherent = mask & (coh > 0.45)
            result[key] = float(np.mean(mag[coherent])) if np.any(coherent) else float("nan")
    result["phase_deg_at_1hz"] = phase_at(freq_result, 1.0)
    result["phase_deg_at_3hz"] = phase_at(freq_result, 3.0)
    result["phase_deg_at_5hz"] = phase_at(freq_result, 5.0)
    return result


def plot_results(mode_df: pd.DataFrame, active_df: pd.DataFrame, t_grid: np.ndarray,
                 values: dict[str, np.ndarray], axis: str,
                 angle_freq: dict[str, object], speed_freq: dict[str, object],
                 step_df: pd.DataFrame, result: dict[str, object],
                 output_dir: Path) -> list[str]:
    paths: list[str] = []
    t = (mode_df["tick_ms"] - mode_df["tick_ms"].iloc[0]) / 1000.0
    angle_label = f"{axis} angle"

    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    axes[0].plot(t, np.degrees(mode_df["angle_ref_rad"]), label="target", linewidth=0.9)
    axes[0].plot(t, np.degrees(mode_df["angle_feedback_rad"]), label="actual", linewidth=0.9)
    axes[0].set_ylabel(f"{angle_label} (deg)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, np.degrees(mode_df["angle_ref_rad"] - mode_df["angle_feedback_rad"]), linewidth=0.8)
    axes[1].set_ylabel("Error (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(t, mode_df["speed_ref_rad_s"], label="speed_ref", linewidth=0.8)
    axes[2].plot(t, mode_df["speed_feedback_rad_s"], label="speed_actual", linewidth=0.8)
    axes[2].set_ylabel("Speed (rad/s)")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)
    axes[3].plot(t, mode_df["output_cmd"], label="output_cmd", linewidth=0.8)
    axes[3].plot(t, mode_df["voltage_ref_raw"], label="voltage_ref", linewidth=0.8)
    axes[3].set_ylabel("Raw output")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="best")
    axes[3].grid(True, alpha=0.3)
    fig.suptitle(f"{axis} fast sysid time series")
    fig.tight_layout()
    path = output_dir / f"{axis}_fast_time_series.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    if angle_freq.get("ok"):
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        axes[0].semilogx(angle_freq["freq_hz"], 20.0 * np.log10(np.maximum(angle_freq["magnitude"], 1e-12)), label="angle ref -> actual")
        if speed_freq.get("ok"):
            axes[0].semilogx(speed_freq["freq_hz"], 20.0 * np.log10(np.maximum(speed_freq["magnitude"], 1e-12)), label="speed ref -> actual")
        axes[0].set_ylabel("Magnitude (dB)")
        axes[0].legend(loc="best")
        axes[0].grid(True, which="both", alpha=0.3)
        axes[1].semilogx(angle_freq["freq_hz"], np.degrees(angle_freq["phase_rad"]), label="angle")
        if speed_freq.get("ok"):
            axes[1].semilogx(speed_freq["freq_hz"], np.degrees(speed_freq["phase_rad"]), label="speed")
        axes[1].set_ylabel("Phase (deg)")
        axes[1].grid(True, which="both", alpha=0.3)
        axes[2].semilogx(angle_freq["freq_hz"], angle_freq["coherence"], label="angle")
        if speed_freq.get("ok"):
            axes[2].semilogx(speed_freq["freq_hz"], speed_freq["coherence"], label="speed")
        axes[2].set_xlabel("Frequency (Hz)")
        axes[2].set_ylabel("Coherence")
        axes[2].set_ylim(-0.05, 1.05)
        axes[2].legend(loc="best")
        axes[2].grid(True, which="both", alpha=0.3)
        fig.suptitle(f"{axis} empirical fast-response frequency response")
        fig.tight_layout()
        path = output_dir / f"{axis}_fast_frequency_response.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths.append(str(path))

    if not step_df.empty:
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        axes[0].bar(step_df["seq_index"], step_df["overshoot_ratio"] * 100.0)
        axes[0].set_ylabel("Overshoot (%)")
        axes[0].grid(True, alpha=0.3)
        axes[1].bar(step_df["seq_index"], step_df["steady_error_deg"])
        axes[1].set_ylabel("Steady err (deg)")
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(step_df["seq_index"], step_df["rise_time_s"], marker="o", label="rise")
        axes[2].plot(step_df["seq_index"], step_df["settling_time_s"], marker="s", label="settling")
        axes[2].set_xlabel("Stage seq_index")
        axes[2].set_ylabel("Time (s)")
        axes[2].legend(loc="best")
        axes[2].grid(True, alpha=0.3)
        fig.suptitle(f"{axis} PRBS step-derived metrics")
        fig.tight_layout()
        path = output_dir / f"{axis}_fast_step_metrics.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths.append(str(path))

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axis("off")
    metrics = result["tracking_metrics"]
    freq_metrics = result["frequency_response"]["angle_ref_to_angle_actual"]
    summary = [
        f"{axis} fast-response sysid summary",
        f"mode: {result['mode']} ({result['kind']})",
        f"score: {result['fast_score']:.4f} (lower is better)",
        f"tracking RMSE: {metrics['tracking_rmse_deg']:.4f} deg",
        f"max abs error: {metrics['tracking_max_abs_deg']:.4f} deg",
        f"output saturation: {100.0 * metrics['output_saturation_ratio']:.2f}%",
        f"angle bandwidth: {freq_metrics.get('bandwidth_hz', float('nan')):.4f} Hz",
        f"coherence 0.1-3Hz: {result['extra_frequency_metrics'].get('mean_coherence_0p1_3hz', float('nan')):.4f}",
        f"phase @3Hz: {result['extra_frequency_metrics'].get('phase_deg_at_3hz', float('nan')):.2f} deg",
    ]
    ax.text(0.02, 0.98, "\n".join(summary), va="top", family="monospace", fontsize=12)
    fig.tight_layout()
    path = output_dir / f"{axis}_fast_summary.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))
    return paths


def main() -> int:
    args = parse_args()
    df = load_numeric_csv(args.csv)
    mode = infer_mode(df, args.axis, args.mode)
    axis, kind, active_phase = MODE_INFO[mode]
    output_dir = analysis_dir_for(args.csv, args.output_dir, axis, kind)
    output_dir.mkdir(parents=True, exist_ok=True)

    mode_df = df[df["mode"].astype(int) == mode].copy()
    active_df = mode_df[mode_df["phase"].astype(int) == active_phase].copy()
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
    active_df = active_df.dropna(subset=required)
    if len(active_df) < 256:
        print(f"not enough active samples for mode {mode}")
        return 2

    t_grid, values, dt_s = uniform_resample(active_df, required)
    metrics = tracking_metrics(
        values["angle_ref_rad"],
        values["angle_feedback_rad"],
        values["speed_ref_rad_s"],
        values["speed_feedback_rad_s"],
        values["output_cmd"],
        args.output_limit,
        args.sat_margin,
    )
    score = fast_score(metrics, args)
    angle_freq = estimate_frequency_response(values["angle_ref_rad"], values["angle_feedback_rad"], dt_s, "angle")
    speed_freq = estimate_frequency_response(values["speed_ref_rad_s"], values["speed_feedback_rad_s"], dt_s, "speed")
    angle_arx = fit_arx(values["angle_ref_rad"], values["angle_feedback_rad"], na=4, nb=4, nk=1)
    speed_arx = fit_arx(values["speed_ref_rad_s"], values["speed_feedback_rad_s"], na=4, nb=4, nk=1)
    step_df = compute_step_metrics(active_df, args) if kind == "prbs" else pd.DataFrame()

    freq_csv = output_dir / f"{axis}_{kind}_frequency_response.csv"
    save_frequency_csv(freq_csv, angle_freq, speed_freq)
    step_csv = output_dir / f"{axis}_{kind}_step_metrics.csv"
    if not step_df.empty:
        step_df.to_csv(step_csv, index=False)

    result = {
        "input_csv": str(args.csv),
        "output_dir": str(output_dir),
        "axis": axis,
        "kind": kind,
        "mode": int(mode),
        "active_phase": int(active_phase),
        "sample_period_s": float(dt_s),
        "sample_rate_hz": float(1.0 / dt_s),
        "sample_count": int(len(active_df)),
        "fast_score": float(score),
        "tracking_metrics": metrics,
        "frequency_response_csv": str(freq_csv),
        "frequency_response": {
            "angle_ref_to_angle_actual": scalar_frequency_metrics(angle_freq),
            "speed_ref_to_speed_actual": scalar_frequency_metrics(speed_freq),
        },
        "extra_frequency_metrics": extra_frequency_metrics(angle_freq),
        "arx_model": {
            "angle_ref_to_angle_actual": {key: value for key, value in angle_arx.items() if key != "simulation"},
            "speed_ref_to_speed_actual": {key: value for key, value in speed_arx.items() if key != "simulation"},
        },
        "warnings": [],
    }
    if not step_df.empty:
        result["step_metrics_csv"] = str(step_csv)
        result["step_metrics_summary"] = {
            "step_count": int(len(step_df)),
            "median_rise_time_s": finite_median(step_df["rise_time_s"]),
            "median_settling_time_s": finite_median(step_df["settling_time_s"]),
            "median_overshoot_ratio": finite_median(step_df["overshoot_ratio"]),
            "median_steady_error_deg": finite_median(step_df["steady_error_deg"]),
            "mean_stage_error_rmse_deg": finite_mean(step_df["stage_error_rmse_deg"]),
        }

    angle_scalar = result["frequency_response"]["angle_ref_to_angle_actual"]
    if angle_scalar.get("ok") and float(result["extra_frequency_metrics"].get("mean_coherence_0p1_3hz", 0.0)) < 0.45:
        result["warnings"].append("angle coherence is low; increase excitation amplitude or reduce external disturbances")
    if metrics["output_saturation_ratio"] > 0.10:
        result["warnings"].append("output saturation is high; optimization will be affected by clipping")
    if angle_arx.get("ok") and float(angle_arx["simulation_r2"]) < 0.60:
        result["warnings"].append("angle ARX simulation R2 is low; model-based conclusions need another dataset")

    plot_paths: list[str] = []
    if not args.no_plots:
        plot_paths = plot_results(mode_df, active_df, t_grid, values, axis, angle_freq, speed_freq,
                                  step_df, result, output_dir)
    result["plot_paths"] = plot_paths

    result_json = output_dir / f"{axis}_{kind}_fast_result.json"
    result_json.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"output_dir={output_dir}")
    print(f"axis={axis}, kind={kind}, mode={mode}, samples={len(active_df)}, fs={1.0 / dt_s:.2f}Hz")
    print(f"fast_score={score:.6f}")
    print(f"tracking_rmse={metrics['tracking_rmse_deg']:.4f}deg")
    print(f"tracking_max_abs={metrics['tracking_max_abs_deg']:.4f}deg")
    print(f"output_saturation={100.0 * metrics['output_saturation_ratio']:.2f}%")
    if angle_scalar.get("ok"):
        print(f"angle_bandwidth={angle_scalar['bandwidth_hz']:.6f}Hz")
    for warning in result["warnings"]:
        print(f"warning: {warning}")
    print(f"json={result_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
