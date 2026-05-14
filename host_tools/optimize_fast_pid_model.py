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
from scipy import optimize

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from analyze_fast_sysid import MODE_INFO, infer_mode, load_numeric_csv, uniform_resample  # noqa: E402
from optimize_yaw_pid_model import (  # noqa: E402
    estimate_current_to_output_gain,
    identify_output_to_speed_plant,
    rmse,
    serializable_optimizer_result,
    simulate_arx,
    simulate_cascade_pid,
    simulation_metrics,
    split_arrays,
)


DEFAULT_TRAIN_RATIO = 0.70
DEFAULT_OUTPUT_LIMIT = 5000.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fast-response model-based PID optimization from yaw/pitch RTT sysid data."
    )
    parser.add_argument("csv", type=Path, help="CSV captured from RTT")
    parser.add_argument("--axis", choices=["auto", "yaw", "pitch"], default="auto")
    parser.add_argument("--mode", type=int, default=None, help="Force GIMBAL_SYSID_MODE")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--current-angle-kp", type=float, required=True)
    parser.add_argument("--current-speed-kp", type=float, required=True)
    parser.add_argument("--current-speed-ki", type=float, required=True)
    parser.add_argument("--speed-ref-limit", type=float, required=True)
    parser.add_argument("--max-speed-ref-limit", type=float, default=None)
    parser.add_argument("--current-limit", type=float, default=3800.0)
    parser.add_argument("--output-limit", type=float, default=DEFAULT_OUTPUT_LIMIT)
    parser.add_argument("--train-ratio", type=float, default=DEFAULT_TRAIN_RATIO)
    parser.add_argument("--maxiter", type=int, default=36)
    parser.add_argument("--popsize", type=int, default=9)
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--no-plots", action="store_true")
    return parser.parse_args()


def analysis_dir_for(csv_path: Path, output_dir: Path | None, axis: str, kind: str) -> Path:
    if output_dir is not None:
        return output_dir
    return csv_path.parent / "analysis" / f"{csv_path.stem}_{axis}_{kind}_fast_pid"


def objective_from_metrics(metrics: dict[str, float]) -> float:
    if not math.isfinite(metrics["tracking_rmse_rad"]):
        return 1e12
    rmse_term = metrics["tracking_rmse_rad"] / math.radians(1.2)
    mae_term = math.radians(metrics["tracking_mae_deg"]) / math.radians(0.9)
    max_term = math.radians(metrics["tracking_max_abs_deg"]) / math.radians(6.0)
    sat_term = metrics["output_saturation_ratio"]
    speed_sat_term = metrics["speed_ref_saturation_ratio"]
    output_term = metrics["output_rms_ratio"]
    rate_term = metrics["output_rate_rms_ratio"]
    return float(
        4.0 * rmse_term * rmse_term
        + 1.2 * mae_term * mae_term
        + 0.4 * max_term * max_term
        + 18.0 * sat_term * sat_term
        + 0.35 * speed_sat_term * speed_sat_term
        + 0.20 * output_term * output_term
        + 0.08 * rate_term * rate_term
    )


def simulate_candidate(args: argparse.Namespace, angle_ref: np.ndarray, dt_s: float,
                       plant: dict[str, object], current_to_output_gain: float,
                       gains: np.ndarray, initial_angle: float, initial_speed: float) -> dict[str, np.ndarray]:
    return simulate_cascade_pid(
        angle_ref,
        dt_s,
        plant,
        current_to_output_gain,
        angle_kp=float(gains[0]),
        speed_kp=float(gains[1]),
        speed_ki=float(gains[2]),
        speed_ref_limit=float(gains[3]),
        current_limit=args.current_limit,
        output_limit=args.output_limit,
        initial_angle=initial_angle,
        initial_speed=initial_speed,
    )


def optimize_pid(args: argparse.Namespace, axis: str, angle_ref: np.ndarray, dt_s: float,
                 plant: dict[str, object], current_to_output_gain: float,
                 initial_angle: float, initial_speed: float) -> dict[str, object]:
    current = np.asarray([
        args.current_angle_kp,
        args.current_speed_kp,
        args.current_speed_ki,
        args.speed_ref_limit,
    ], dtype=float)
    max_speed_ref = args.max_speed_ref_limit
    if max_speed_ref is None:
        max_speed_ref = 8.0 if axis == "yaw" else 6.0

    bounds = [
        (max(1.0, current[0] * 0.60), min(120.0, current[0] * 2.40)),
        (max(100.0, current[1] * 0.50), min(7000.0, current[1] * 2.50)),
        (max(0.0, current[2] * 0.05), min(5000.0, max(200.0, current[2] * 1.60))),
        (max(0.5, current[3] * 0.90), min(max_speed_ref, current[3] * 2.00)),
    ]
    cache: dict[tuple[float, float, float, float], float] = {}

    def evaluate(gains: np.ndarray) -> float:
        key = tuple(float(f"{v:.6f}") for v in gains)
        if key in cache:
            return cache[key]
        sim = simulate_candidate(args, angle_ref, dt_s, plant, current_to_output_gain,
                                 gains, initial_angle, initial_speed)
        metrics = simulation_metrics(angle_ref, sim, args.output_limit, float(gains[3]))
        value = objective_from_metrics(metrics)
        log_ratio = np.log(np.maximum(gains, 1e-6) / np.maximum(current, 1e-6))
        value += float(0.015 * np.sum(log_ratio * log_ratio))
        cache[key] = value
        return value

    current_cost = evaluate(current)
    de_result = optimize.differential_evolution(
        evaluate,
        bounds=bounds,
        maxiter=args.maxiter,
        popsize=args.popsize,
        seed=args.seed,
        polish=False,
        updating="immediate",
        workers=1,
        tol=0.01,
    )
    local_result = optimize.minimize(
        evaluate,
        de_result.x,
        method="L-BFGS-B",
        bounds=bounds,
        options={"maxiter": 140, "ftol": 1e-9},
    )
    best_x = local_result.x if local_result.success else de_result.x
    best_cost = evaluate(best_x)
    current_sim = simulate_candidate(args, angle_ref, dt_s, plant, current_to_output_gain,
                                     current, initial_angle, initial_speed)
    best_sim = simulate_candidate(args, angle_ref, dt_s, plant, current_to_output_gain,
                                  best_x, initial_angle, initial_speed)
    return {
        "current": {
            "angle_kp": float(current[0]),
            "speed_kp": float(current[1]),
            "speed_ki": float(current[2]),
            "speed_ref_limit": float(current[3]),
            "cost": float(current_cost),
            "metrics": simulation_metrics(angle_ref, current_sim, args.output_limit, float(current[3])),
        },
        "optimized": {
            "angle_kp": float(best_x[0]),
            "speed_kp": float(best_x[1]),
            "speed_ki": float(best_x[2]),
            "speed_ref_limit": float(best_x[3]),
            "cost": float(best_cost),
            "metrics": simulation_metrics(angle_ref, best_sim, args.output_limit, float(best_x[3])),
        },
        "bounds": bounds,
        "eval_count": int(len(cache)),
        "de_success": bool(de_result.success),
        "local_success": bool(local_result.success),
        "current_sim": current_sim,
        "optimized_sim": best_sim,
    }


def plot_results(t_grid: np.ndarray, values: dict[str, np.ndarray], train_count: int,
                 axis: str, plant: dict[str, object], opt_valid: dict[str, object],
                 output_dir: Path) -> list[str]:
    paths: list[str] = []
    measured_output_speed = simulate_arx(
        values["output_cmd"],
        plant,
        initial_y=float(values["speed_feedback_rad_s"][0]),
    )
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    axes[0].plot(t_grid, values["speed_feedback_rad_s"], label="measured speed", linewidth=0.9)
    axes[0].plot(t_grid, measured_output_speed, label="ARX output->speed", linewidth=0.9)
    axes[0].axvline(t_grid[train_count], color="gray", linestyle="--", linewidth=0.9, label="train/valid split")
    axes[0].set_ylabel("Speed (rad/s)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t_grid, values["output_cmd"], label="measured output_cmd", linewidth=0.8)
    axes[1].set_ylabel("Output raw")
    axes[1].set_xlabel("Time (s)")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
    fig.suptitle(
        f"{axis} plant validation, valid R2={plant['validation_simulation_r2']:.3f}, "
        f"valid RMSE={plant['validation_simulation_rmse']:.3f} rad/s"
    )
    fig.tight_layout()
    path = output_dir / f"{axis}_fast_pid_plant_validation.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    valid_start = train_count
    valid_t = t_grid[valid_start:] - t_grid[valid_start]
    ref = values["angle_ref_rad"][valid_start:]
    actual = values["angle_feedback_rad"][valid_start:]
    current_sim = opt_valid["current_sim"]
    optimized_sim = opt_valid["optimized_sim"]

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    axes[0].plot(valid_t, np.degrees(ref), label="target", linewidth=0.9)
    axes[0].plot(valid_t, np.degrees(actual), label="measured current PID", linewidth=0.8, alpha=0.7)
    axes[0].plot(valid_t, np.degrees(current_sim["angle"]), label="model current PID", linewidth=0.9)
    axes[0].plot(valid_t, np.degrees(optimized_sim["angle"]), label="model fast PID", linewidth=0.9)
    axes[0].set_ylabel(f"{axis} angle (deg)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(valid_t, np.degrees(ref - current_sim["angle"]), label="current model error", linewidth=0.9)
    axes[1].plot(valid_t, np.degrees(ref - optimized_sim["angle"]), label="fast model error", linewidth=0.9)
    axes[1].set_ylabel("Error (deg)")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(valid_t, current_sim["output_cmd"], label="current output", linewidth=0.9)
    axes[2].plot(valid_t, optimized_sim["output_cmd"], label="fast output", linewidth=0.9)
    axes[2].set_ylabel("Output raw")
    axes[2].set_xlabel("Validation time (s)")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)
    fig.suptitle(f"{axis} fast PID model validation")
    fig.tight_layout()
    path = output_dir / f"{axis}_fast_pid_validation.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axis("off")
    current = opt_valid["current"]
    optimized = opt_valid["optimized"]
    text = "\n".join([
        f"Fast-response {axis} PID optimization",
        f"plant validation R2: {plant['validation_simulation_r2']:.4f}",
        f"current cost: {current['cost']:.4f}",
        f"fast cost: {optimized['cost']:.4f}",
        "",
        "current:",
        f"  angle Kp = {current['angle_kp']:.6f}",
        f"  speed Kp = {current['speed_kp']:.6f}",
        f"  speed Ki = {current['speed_ki']:.6f}",
        f"  speed ref max = {current['speed_ref_limit']:.6f}",
        f"  RMSE = {current['metrics']['tracking_rmse_deg']:.3f} deg",
        "",
        "fast candidate:",
        f"  angle Kp = {optimized['angle_kp']:.6f}",
        f"  speed Kp = {optimized['speed_kp']:.6f}",
        f"  speed Ki = {optimized['speed_ki']:.6f}",
        f"  speed ref max = {optimized['speed_ref_limit']:.6f}",
        f"  RMSE = {optimized['metrics']['tracking_rmse_deg']:.3f} deg",
    ])
    ax.text(0.02, 0.98, text, va="top", family="monospace", fontsize=12)
    fig.tight_layout()
    path = output_dir / f"{axis}_fast_pid_summary.png"
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

    active_df = df[(df["mode"].astype(int) == mode) &
                   (df["phase"].astype(int) == active_phase)].copy()
    required = [
        "angle_ref_rad",
        "angle_feedback_rad",
        "speed_ref_rad_s",
        "speed_feedback_rad_s",
        "current_ref_raw",
        "output_cmd",
    ]
    active_df = active_df.dropna(subset=required)
    if len(active_df) < 768:
        print(f"not enough active samples for mode {mode}")
        return 2

    t_grid, values, dt_s = uniform_resample(active_df, required)
    train, valid = split_arrays(values, args.train_ratio)
    train_count = len(train["angle_ref_rad"])
    plant = identify_output_to_speed_plant(
        train["output_cmd"],
        train["speed_feedback_rad_s"],
        train_ratio=0.75,
    )
    if not plant.get("ok"):
        print(f"plant identification failed: {plant.get('reason')}")
        return 2
    current_gain = estimate_current_to_output_gain(train["current_ref_raw"], train["output_cmd"])

    opt_train = optimize_pid(
        args,
        axis,
        train["angle_ref_rad"],
        dt_s,
        plant,
        current_gain["gain"],
        initial_angle=float(train["angle_feedback_rad"][0]),
        initial_speed=float(train["speed_feedback_rad_s"][0]),
    )
    train_candidate = opt_train["optimized"]
    current_gains = np.asarray([
        args.current_angle_kp,
        args.current_speed_kp,
        args.current_speed_ki,
        args.speed_ref_limit,
    ])
    candidate_gains = np.asarray([
        train_candidate["angle_kp"],
        train_candidate["speed_kp"],
        train_candidate["speed_ki"],
        train_candidate["speed_ref_limit"],
    ])
    valid_current_sim = simulate_candidate(
        args,
        valid["angle_ref_rad"],
        dt_s,
        plant,
        current_gain["gain"],
        current_gains,
        initial_angle=float(valid["angle_feedback_rad"][0]),
        initial_speed=float(valid["speed_feedback_rad_s"][0]),
    )
    valid_candidate_sim = simulate_candidate(
        args,
        valid["angle_ref_rad"],
        dt_s,
        plant,
        current_gain["gain"],
        candidate_gains,
        initial_angle=float(valid["angle_feedback_rad"][0]),
        initial_speed=float(valid["speed_feedback_rad_s"][0]),
    )
    current_metrics = simulation_metrics(valid["angle_ref_rad"], valid_current_sim,
                                         args.output_limit, args.speed_ref_limit)
    candidate_metrics = simulation_metrics(valid["angle_ref_rad"], valid_candidate_sim,
                                           args.output_limit, train_candidate["speed_ref_limit"])
    opt_valid = {
        "current": {
            "angle_kp": args.current_angle_kp,
            "speed_kp": args.current_speed_kp,
            "speed_ki": args.current_speed_ki,
            "speed_ref_limit": args.speed_ref_limit,
            "metrics": current_metrics,
            "cost": objective_from_metrics(current_metrics),
        },
        "optimized": {
            "angle_kp": train_candidate["angle_kp"],
            "speed_kp": train_candidate["speed_kp"],
            "speed_ki": train_candidate["speed_ki"],
            "speed_ref_limit": train_candidate["speed_ref_limit"],
            "metrics": candidate_metrics,
            "cost": objective_from_metrics(candidate_metrics),
        },
        "current_sim": valid_current_sim,
        "optimized_sim": valid_candidate_sim,
    }

    warnings: list[str] = []
    if float(plant["validation_simulation_r2"]) < 0.65:
        warnings.append("plant validation R2 is low; do not apply optimized PID without another dataset")
    if current_gain["r2"] < 0.65:
        warnings.append("current_ref to output_cmd gain fit is weak; speed PID scaling is approximate")
    if opt_valid["optimized"]["cost"] >= opt_valid["current"]["cost"]:
        warnings.append("fast candidate does not improve validation cost over current PID")
    if opt_valid["optimized"]["metrics"]["output_saturation_ratio"] > 0.10:
        warnings.append("fast candidate saturates output often; verify on hardware with smaller step first")

    result = {
        "input_csv": str(args.csv),
        "output_dir": str(output_dir),
        "axis": axis,
        "kind": kind,
        "mode": int(mode),
        "active_phase": int(active_phase),
        "sample_period_s": float(dt_s),
        "sample_rate_hz": float(1.0 / dt_s),
        "sample_count": int(len(t_grid)),
        "train_sample_count": int(train_count),
        "validation_sample_count": int(len(t_grid) - train_count),
        "plant_model": plant,
        "current_ref_to_output_cmd": current_gain,
        "optimization": {
            "method": "ARX output_cmd->speed plant + cascaded PID simulation + fast-response objective",
            "objective": (
                "tracking RMSE/MAE/max error with output saturation, speed-ref saturation, "
                "output RMS and output-rate penalties"
            ),
            "train": serializable_optimizer_result(opt_train),
            "validation": serializable_optimizer_result(opt_valid),
        },
        "recommended_pid": {
            "angle_kp": opt_valid["optimized"]["angle_kp"],
            "speed_kp": opt_valid["optimized"]["speed_kp"],
            "speed_ki": opt_valid["optimized"]["speed_ki"],
            "speed_ref_limit": opt_valid["optimized"]["speed_ref_limit"],
        },
        "warnings": warnings,
    }

    plot_paths: list[str] = []
    if not args.no_plots:
        plot_paths = plot_results(t_grid, values, train_count, axis, plant, opt_valid, output_dir)
    result["plot_paths"] = plot_paths
    result_json = output_dir / f"{axis}_{kind}_fast_pid_result.json"
    result_json.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"output_dir={output_dir}")
    print(f"axis={axis}, kind={kind}, mode={mode}, fs={1.0 / dt_s:.2f}Hz")
    print(f"plant: na={plant['na']} nb={plant['nb']} nk={plant['nk']} "
          f"valid_r2={plant['validation_simulation_r2']:.4f} "
          f"valid_rmse={plant['validation_simulation_rmse']:.4f} rad/s")
    print(f"current_ref_to_output_gain={current_gain['gain']:.6f}, r2={current_gain['r2']:.4f}")
    print("")
    print("validation model cost:")
    print(f"  current={opt_valid['current']['cost']:.6f}, fast={opt_valid['optimized']['cost']:.6f}")
    print("")
    print("fast PID candidate:")
    print(f"  angle Kp: {args.current_angle_kp:.6f} -> {opt_valid['optimized']['angle_kp']:.6f}")
    print(f"  speed Kp: {args.current_speed_kp:.6f} -> {opt_valid['optimized']['speed_kp']:.6f}")
    print(f"  speed Ki: {args.current_speed_ki:.6f} -> {opt_valid['optimized']['speed_ki']:.6f}")
    print(f"  speed ref max: {args.speed_ref_limit:.6f} -> {opt_valid['optimized']['speed_ref_limit']:.6f}")
    for warning in warnings:
        print(f"warning: {warning}")
    print(f"json={result_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
