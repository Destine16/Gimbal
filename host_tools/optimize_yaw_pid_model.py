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
from scipy import optimize

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from analyze_yaw_prbs_sysid import (  # noqa: E402
    PRBS_PHASE,
    YAW_PRBS_MODE,
    load_numeric_csv,
    uniform_resample,
)


DEFAULT_ANGLE_KP = 12.0
DEFAULT_SPEED_KP = 600.0
DEFAULT_SPEED_KI = 1600.0
DEFAULT_SPEED_REF_LIMIT = 2.0
DEFAULT_CURRENT_LIMIT = 3000.0
DEFAULT_OUTPUT_LIMIT = 5000.0
DEFAULT_TRAIN_RATIO = 0.70


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Model-based yaw PID optimization from PRBS telemetry."
    )
    parser.add_argument("csv", type=Path, help="Yaw PRBS CSV captured from RTT")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--current-angle-kp", type=float, default=DEFAULT_ANGLE_KP)
    parser.add_argument("--current-speed-kp", type=float, default=DEFAULT_SPEED_KP)
    parser.add_argument("--current-speed-ki", type=float, default=DEFAULT_SPEED_KI)
    parser.add_argument("--speed-ref-limit", type=float, default=DEFAULT_SPEED_REF_LIMIT)
    parser.add_argument("--current-limit", type=float, default=DEFAULT_CURRENT_LIMIT)
    parser.add_argument("--output-limit", type=float, default=DEFAULT_OUTPUT_LIMIT)
    parser.add_argument("--train-ratio", type=float, default=DEFAULT_TRAIN_RATIO)
    parser.add_argument("--maxiter", type=int, default=28, help="Differential evolution iterations")
    parser.add_argument("--popsize", type=int, default=8, help="Differential evolution population size")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--no-plots", action="store_true")
    return parser.parse_args()


def analysis_dir_for(csv_path: Path, output_dir: Path | None) -> Path:
    if output_dir is not None:
        return output_dir
    return csv_path.parent / "analysis" / f"{csv_path.stem}_yaw_model_pid"


def clamp(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def finite_r2(actual: np.ndarray, pred: np.ndarray) -> float:
    actual = np.asarray(actual, dtype=float)
    pred = np.asarray(pred, dtype=float)
    mask = np.isfinite(actual) & np.isfinite(pred)
    if np.count_nonzero(mask) < 2:
        return float("nan")
    y = actual[mask]
    y_hat = pred[mask]
    sst = float(np.sum((y - np.mean(y)) ** 2))
    if sst <= 1e-12:
        return float("nan")
    return float(1.0 - np.sum((y - y_hat) ** 2) / sst)


def rmse(actual: np.ndarray, pred: np.ndarray) -> float:
    actual = np.asarray(actual, dtype=float)
    pred = np.asarray(pred, dtype=float)
    mask = np.isfinite(actual) & np.isfinite(pred)
    if np.count_nonzero(mask) == 0:
        return float("nan")
    err = actual[mask] - pred[mask]
    return float(math.sqrt(np.mean(err * err)))


def fit_arx_model(u: np.ndarray, y: np.ndarray, na: int, nb: int, nk: int) -> dict[str, object]:
    u = np.asarray(u, dtype=float)
    y = np.asarray(y, dtype=float)
    mask = np.isfinite(u) & np.isfinite(y)
    u = u[mask]
    y = y[mask]
    max_lag = max(na, nk + nb - 1)
    if len(y) <= max_lag + 20:
        return {"ok": False, "reason": "not_enough_samples"}

    u_mean = float(np.mean(u))
    y_mean = float(np.mean(y))
    uu = u - u_mean
    yy = y - y_mean
    rows: list[list[float]] = []
    target: list[float] = []

    for k in range(max_lag, len(y)):
        row = [yy[k - i] for i in range(1, na + 1)]
        row += [uu[k - nk - j] for j in range(nb)]
        row += [1.0]
        rows.append(row)
        target.append(yy[k])

    x = np.asarray(rows, dtype=float)
    z = np.asarray(target, dtype=float)
    coeff, *_ = np.linalg.lstsq(x, z, rcond=None)
    one_step = x @ coeff
    one_step_rmse = rmse(z, one_step)
    one_step_r2 = finite_r2(z, one_step)

    return {
        "ok": True,
        "na": int(na),
        "nb": int(nb),
        "nk": int(nk),
        "max_lag": int(max_lag),
        "coefficients": [float(v) for v in coeff],
        "u_mean": u_mean,
        "y_mean": y_mean,
        "one_step_rmse": float(one_step_rmse),
        "one_step_r2": float(one_step_r2),
    }


def simulate_arx(u: np.ndarray, model: dict[str, object], initial_y: float | None = None) -> np.ndarray:
    u = np.asarray(u, dtype=float)
    na = int(model["na"])
    nb = int(model["nb"])
    nk = int(model["nk"])
    coeff = np.asarray(model["coefficients"], dtype=float)
    u_mean = float(model["u_mean"])
    y_mean = float(model["y_mean"])
    max_lag = int(model["max_lag"])
    result_center = np.zeros(len(u), dtype=float)
    initial_center = (float(initial_y) - y_mean) if initial_y is not None else 0.0
    result_center[:max_lag] = initial_center
    uu = u - u_mean

    for k in range(max_lag, len(u)):
        row = [result_center[k - i] for i in range(1, na + 1)]
        row += [uu[k - nk - j] for j in range(nb)]
        row += [1.0]
        result_center[k] = float(np.dot(row, coeff))
        if not math.isfinite(result_center[k]) or abs(result_center[k]) > 200.0:
            result_center[k:] = np.nan
            break

    return result_center + y_mean


def identify_output_to_speed_plant(u: np.ndarray, y: np.ndarray, train_ratio: float) -> dict[str, object]:
    split = int(len(u) * train_ratio)
    split = min(max(split, 256), len(u) - 128)
    train_u = u[:split]
    train_y = y[:split]
    valid_u = u[split:]
    valid_y = y[split:]
    candidates: list[dict[str, object]] = []

    for na in range(2, 8):
        for nb in range(2, 8):
            for nk in (1, 2, 3):
                model = fit_arx_model(train_u, train_y, na, nb, nk)
                if not model.get("ok"):
                    continue
                train_sim = simulate_arx(train_u, model, initial_y=float(train_y[0]))
                valid_sim = simulate_arx(valid_u, model, initial_y=float(valid_y[0]))
                model["train_simulation_rmse"] = rmse(train_y, train_sim)
                model["train_simulation_r2"] = finite_r2(train_y, train_sim)
                model["validation_simulation_rmse"] = rmse(valid_y, valid_sim)
                model["validation_simulation_r2"] = finite_r2(valid_y, valid_sim)
                candidates.append(model)

    if not candidates:
        return {"ok": False, "reason": "no_arx_candidate"}

    candidates.sort(
        key=lambda item: (
            -float(item.get("validation_simulation_r2", -999.0)),
            float(item.get("validation_simulation_rmse", 999.0)),
            int(item["na"]) + int(item["nb"]),
        )
    )
    best = candidates[0]
    best["candidate_count"] = len(candidates)
    best["train_sample_count"] = int(split)
    best["validation_sample_count"] = int(len(u) - split)
    return best


def estimate_current_to_output_gain(current_ref: np.ndarray, output_cmd: np.ndarray) -> dict[str, float]:
    current_ref = np.asarray(current_ref, dtype=float)
    output_cmd = np.asarray(output_cmd, dtype=float)
    mask = np.isfinite(current_ref) & np.isfinite(output_cmd) & (np.abs(current_ref) > 1e-6)
    x = current_ref[mask]
    y = output_cmd[mask]
    gain = float(np.dot(x, y) / max(np.dot(x, x), 1e-9))
    pred = gain * current_ref
    return {
        "gain": gain,
        "r2": finite_r2(output_cmd, pred),
        "rmse_raw": rmse(output_cmd, pred),
    }


def simulate_cascade_pid(
    angle_ref: np.ndarray,
    dt_s: float,
    plant: dict[str, object],
    current_to_output_gain: float,
    angle_kp: float,
    speed_kp: float,
    speed_ki: float,
    speed_ref_limit: float,
    current_limit: float,
    output_limit: float,
    initial_angle: float,
    initial_speed: float,
) -> dict[str, np.ndarray]:
    n = len(angle_ref)
    angle = np.zeros(n, dtype=float)
    speed = np.zeros(n, dtype=float)
    speed_ref = np.zeros(n, dtype=float)
    current_ref = np.zeros(n, dtype=float)
    output_cmd = np.zeros(n, dtype=float)
    angle[0] = initial_angle
    speed[0] = initial_speed

    na = int(plant["na"])
    nb = int(plant["nb"])
    nk = int(plant["nk"])
    coeff = np.asarray(plant["coefficients"], dtype=float)
    u_mean = float(plant["u_mean"])
    y_mean = float(plant["y_mean"])
    y_hist = [initial_speed - y_mean for _ in range(max(na, 1))]
    u_hist = [0.0 - u_mean for _ in range(max(nb + nk, 1))]
    speed_iout = 0.0

    for k in range(1, n):
        angle_error = float(angle_ref[k - 1] - angle[k - 1])
        speed_ref[k - 1] = clamp(angle_kp * angle_error, speed_ref_limit)

        speed_error = speed_ref[k - 1] - speed[k - 1]
        speed_iout = clamp(speed_iout + speed_ki * speed_error * dt_s, current_limit)
        current_ref[k - 1] = clamp(speed_kp * speed_error + speed_iout, current_limit)
        output_cmd[k - 1] = clamp(current_to_output_gain * current_ref[k - 1], output_limit)

        u_hist.insert(0, output_cmd[k - 1] - u_mean)
        del u_hist[-1]
        row = [y_hist[i - 1] for i in range(1, na + 1)]
        row += [u_hist[nk - 1 + j] for j in range(nb)]
        row += [1.0]
        y_center = float(np.dot(row, coeff))
        if not math.isfinite(y_center) or abs(y_center) > 200.0:
            angle[k:] = np.nan
            speed[k:] = np.nan
            speed_ref[k:] = np.nan
            current_ref[k:] = np.nan
            output_cmd[k:] = np.nan
            break
        speed[k] = y_center + y_mean
        angle[k] = angle[k - 1] + speed[k] * dt_s
        y_hist.insert(0, y_center)
        del y_hist[-1]

    speed_ref[-1] = clamp(angle_kp * float(angle_ref[-1] - angle[-1]), speed_ref_limit)
    speed_error = speed_ref[-1] - speed[-1]
    current_ref[-1] = clamp(speed_kp * speed_error + speed_iout, current_limit)
    output_cmd[-1] = clamp(current_to_output_gain * current_ref[-1], output_limit)
    return {
        "angle": angle,
        "speed": speed,
        "speed_ref": speed_ref,
        "current_ref": current_ref,
        "output_cmd": output_cmd,
    }


def simulation_metrics(angle_ref: np.ndarray, sim: dict[str, np.ndarray],
                       output_limit: float, speed_ref_limit: float) -> dict[str, float]:
    angle = sim["angle"]
    output_cmd = sim["output_cmd"]
    speed_ref = sim["speed_ref"]
    if not np.all(np.isfinite(angle)):
        return {
            "tracking_rmse_rad": float("inf"),
            "tracking_rmse_deg": float("inf"),
            "tracking_mae_deg": float("inf"),
            "tracking_max_abs_deg": float("inf"),
            "output_saturation_ratio": 1.0,
            "speed_ref_saturation_ratio": 1.0,
            "output_rms_ratio": float("inf"),
            "output_rate_rms_ratio": float("inf"),
        }

    error = angle_ref - angle
    output_rate = np.diff(output_cmd, prepend=output_cmd[0])
    return {
        "tracking_rmse_rad": float(math.sqrt(np.mean(error * error))),
        "tracking_rmse_deg": float(math.degrees(math.sqrt(np.mean(error * error)))),
        "tracking_mae_deg": float(math.degrees(np.mean(np.abs(error)))),
        "tracking_max_abs_deg": float(math.degrees(np.max(np.abs(error)))),
        "output_saturation_ratio": float(np.mean(np.abs(output_cmd) >= 0.98 * output_limit)),
        "speed_ref_saturation_ratio": float(np.mean(np.abs(speed_ref) >= 0.98 * speed_ref_limit)),
        "output_rms_ratio": float(math.sqrt(np.mean(output_cmd * output_cmd)) / output_limit),
        "output_rate_rms_ratio": float(math.sqrt(np.mean(output_rate * output_rate)) / output_limit),
    }


def objective_from_metrics(metrics: dict[str, float]) -> float:
    if not math.isfinite(metrics["tracking_rmse_rad"]):
        return 1e9
    rmse_term = metrics["tracking_rmse_rad"] / 0.18
    mae_term = math.radians(metrics["tracking_mae_deg"]) / 0.12
    max_term = math.radians(metrics["tracking_max_abs_deg"]) / 1.20
    sat_term = metrics["output_saturation_ratio"]
    speed_sat_term = metrics["speed_ref_saturation_ratio"]
    output_term = metrics["output_rms_ratio"]
    rate_term = metrics["output_rate_rms_ratio"]
    return float(
        rmse_term * rmse_term
        + 0.35 * mae_term * mae_term
        + 0.05 * max_term * max_term
        + 40.0 * sat_term * sat_term
        # Outer-loop speed reference clipping is expected during large PRBS steps.
        # Treat it as a weak smoothness penalty, not as a failure condition.
        + 0.10 * speed_sat_term * speed_sat_term
        + 0.08 * output_term * output_term
        + 0.03 * rate_term * rate_term
    )


def optimize_pid(args: argparse.Namespace, angle_ref: np.ndarray, dt_s: float,
                 plant: dict[str, object], current_to_output_gain: float,
                 initial_angle: float, initial_speed: float) -> dict[str, object]:
    current = np.asarray([
        args.current_angle_kp,
        args.current_speed_kp,
        args.current_speed_ki,
    ], dtype=float)
    bounds = [
        (max(2.0, current[0] * 0.45), min(30.0, current[0] * 2.20)),
        (max(120.0, current[1] * 0.35), min(1800.0, current[1] * 2.20)),
        (max(200.0, current[2] * 0.25), min(4200.0, current[2] * 2.50)),
    ]

    cache: dict[tuple[float, float, float], float] = {}

    def evaluate(gains: np.ndarray) -> float:
        key = tuple(float(f"{v:.6f}") for v in gains)
        if key in cache:
            return cache[key]
        sim = simulate_cascade_pid(
            angle_ref,
            dt_s,
            plant,
            current_to_output_gain,
            angle_kp=float(gains[0]),
            speed_kp=float(gains[1]),
            speed_ki=float(gains[2]),
            speed_ref_limit=args.speed_ref_limit,
            current_limit=args.current_limit,
            output_limit=args.output_limit,
            initial_angle=initial_angle,
            initial_speed=initial_speed,
        )
        value = objective_from_metrics(
            simulation_metrics(angle_ref, sim, args.output_limit, args.speed_ref_limit)
        )
        # Soft regularization keeps the optimizer from exploiting model errors far away from the tested region.
        log_ratio = np.log(np.maximum(gains, 1e-6) / current)
        value += float(0.03 * np.sum(log_ratio * log_ratio))
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
        options={"maxiter": 120, "ftol": 1e-9},
    )
    best_x = local_result.x if local_result.success else de_result.x
    best_cost = evaluate(best_x)

    current_sim = simulate_cascade_pid(
        angle_ref,
        dt_s,
        plant,
        current_to_output_gain,
        args.current_angle_kp,
        args.current_speed_kp,
        args.current_speed_ki,
        args.speed_ref_limit,
        args.current_limit,
        args.output_limit,
        initial_angle,
        initial_speed,
    )
    best_sim = simulate_cascade_pid(
        angle_ref,
        dt_s,
        plant,
        current_to_output_gain,
        float(best_x[0]),
        float(best_x[1]),
        float(best_x[2]),
        args.speed_ref_limit,
        args.current_limit,
        args.output_limit,
        initial_angle,
        initial_speed,
    )
    return {
        "current": {
            "angle_kp": float(current[0]),
            "speed_kp": float(current[1]),
            "speed_ki": float(current[2]),
            "cost": float(current_cost),
            "metrics": simulation_metrics(angle_ref, current_sim, args.output_limit, args.speed_ref_limit),
        },
        "optimized": {
            "angle_kp": float(best_x[0]),
            "speed_kp": float(best_x[1]),
            "speed_ki": float(best_x[2]),
            "cost": float(best_cost),
            "metrics": simulation_metrics(angle_ref, best_sim, args.output_limit, args.speed_ref_limit),
        },
        "bounds": bounds,
        "eval_count": int(len(cache)),
        "de_success": bool(de_result.success),
        "local_success": bool(local_result.success),
        "current_sim": current_sim,
        "optimized_sim": best_sim,
    }


def split_arrays(values: dict[str, np.ndarray], train_ratio: float) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    n = len(next(iter(values.values())))
    split = int(n * train_ratio)
    split = min(max(split, 256), n - 128)
    train = {key: value[:split] for key, value in values.items()}
    valid = {key: value[split:] for key, value in values.items()}
    return train, valid


def serializable_optimizer_result(result: dict[str, object]) -> dict[str, object]:
    return {
        key: value for key, value in result.items()
        if key not in {"current_sim", "optimized_sim"}
    }


def plot_results(t_grid: np.ndarray, values: dict[str, np.ndarray], train_count: int,
                 plant: dict[str, object], opt_train: dict[str, object], opt_valid: dict[str, object],
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
        f"Yaw plant validation, valid R2={plant['validation_simulation_r2']:.3f}, "
        f"valid RMSE={plant['validation_simulation_rmse']:.3f} rad/s"
    )
    fig.tight_layout()
    path = output_dir / "yaw_model_plant_validation.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    valid_start = train_count
    valid_t = t_grid[valid_start:] - t_grid[valid_start]
    current_sim = opt_valid["current_sim"]
    optimized_sim = opt_valid["optimized_sim"]
    ref = values["angle_ref_rad"][valid_start:]
    actual = values["angle_feedback_rad"][valid_start:]

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    axes[0].plot(valid_t, np.degrees(ref), label="target", linewidth=0.9)
    axes[0].plot(valid_t, np.degrees(actual), label="measured old PID", linewidth=0.8, alpha=0.7)
    axes[0].plot(valid_t, np.degrees(current_sim["angle"]), label="model current PID", linewidth=0.9)
    axes[0].plot(valid_t, np.degrees(optimized_sim["angle"]), label="model optimized PID", linewidth=0.9)
    axes[0].set_ylabel("Yaw angle (deg)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(valid_t, np.degrees(ref - current_sim["angle"]), label="current model error", linewidth=0.9)
    axes[1].plot(valid_t, np.degrees(ref - optimized_sim["angle"]), label="optimized model error", linewidth=0.9)
    axes[1].set_ylabel("Error (deg)")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(valid_t, current_sim["output_cmd"], label="current output", linewidth=0.9)
    axes[2].plot(valid_t, optimized_sim["output_cmd"], label="optimized output", linewidth=0.9)
    axes[2].set_ylabel("Output raw")
    axes[2].set_xlabel("Validation time (s)")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)
    fig.suptitle("Yaw model-based PID validation")
    fig.tight_layout()
    path = output_dir / "yaw_model_pid_validation.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axis("off")
    current = opt_valid["current"]
    optimized = opt_valid["optimized"]
    text = "\n".join([
        "Model-based yaw PID optimization",
        f"plant validation R2: {plant['validation_simulation_r2']:.4f}",
        f"current cost: {current['cost']:.4f}",
        f"optimized cost: {optimized['cost']:.4f}",
        "",
        "current PID:",
        f"  angle Kp = {current['angle_kp']:.6f}",
        f"  speed Kp = {current['speed_kp']:.6f}",
        f"  speed Ki = {current['speed_ki']:.6f}",
        f"  RMSE = {current['metrics']['tracking_rmse_deg']:.3f} deg",
        "",
        "optimized PID:",
        f"  angle Kp = {optimized['angle_kp']:.6f}",
        f"  speed Kp = {optimized['speed_kp']:.6f}",
        f"  speed Ki = {optimized['speed_ki']:.6f}",
        f"  RMSE = {optimized['metrics']['tracking_rmse_deg']:.3f} deg",
    ])
    ax.text(0.02, 0.98, text, va="top", family="monospace", fontsize=12)
    fig.tight_layout()
    path = output_dir / "yaw_model_pid_summary.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))
    return paths


def main() -> int:
    args = parse_args()
    output_dir = analysis_dir_for(args.csv, args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    df = load_numeric_csv(args.csv)
    prbs_df = df[(df["mode"].astype(int) == YAW_PRBS_MODE) &
                 (df["phase"].astype(int) == PRBS_PHASE)].copy()
    required = [
        "angle_ref_rad",
        "angle_feedback_rad",
        "speed_ref_rad_s",
        "speed_feedback_rad_s",
        "current_ref_raw",
        "output_cmd",
    ]
    prbs_df = prbs_df.dropna(subset=required)
    if len(prbs_df) < 512:
        print("not enough yaw PRBS samples")
        return 2

    t_grid, values, dt_s = uniform_resample(prbs_df, required)
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

    current_gain = estimate_current_to_output_gain(
        train["current_ref_raw"],
        train["output_cmd"],
    )

    opt_train = optimize_pid(
        args,
        train["angle_ref_rad"],
        dt_s,
        plant,
        current_gain["gain"],
        initial_angle=float(train["angle_feedback_rad"][0]),
        initial_speed=float(train["speed_feedback_rad_s"][0]),
    )
    opt_valid = optimize_pid(
        args,
        valid["angle_ref_rad"],
        dt_s,
        plant,
        current_gain["gain"],
        initial_angle=float(valid["angle_feedback_rad"][0]),
        initial_speed=float(valid["speed_feedback_rad_s"][0]),
    )
    # Keep the train-optimized candidate, and evaluate it on validation.
    train_candidate = opt_train["optimized"]
    valid_candidate_sim = simulate_cascade_pid(
        valid["angle_ref_rad"],
        dt_s,
        plant,
        current_gain["gain"],
        train_candidate["angle_kp"],
        train_candidate["speed_kp"],
        train_candidate["speed_ki"],
        args.speed_ref_limit,
        args.current_limit,
        args.output_limit,
        initial_angle=float(valid["angle_feedback_rad"][0]),
        initial_speed=float(valid["speed_feedback_rad_s"][0]),
    )
    valid_current_sim = simulate_cascade_pid(
        valid["angle_ref_rad"],
        dt_s,
        plant,
        current_gain["gain"],
        args.current_angle_kp,
        args.current_speed_kp,
        args.current_speed_ki,
        args.speed_ref_limit,
        args.current_limit,
        args.output_limit,
        initial_angle=float(valid["angle_feedback_rad"][0]),
        initial_speed=float(valid["speed_feedback_rad_s"][0]),
    )
    opt_valid = {
        "current": {
            "angle_kp": args.current_angle_kp,
            "speed_kp": args.current_speed_kp,
            "speed_ki": args.current_speed_ki,
            "metrics": simulation_metrics(valid["angle_ref_rad"], valid_current_sim,
                                          args.output_limit, args.speed_ref_limit),
            "cost": objective_from_metrics(
                simulation_metrics(valid["angle_ref_rad"], valid_current_sim,
                                   args.output_limit, args.speed_ref_limit)
            ),
        },
        "optimized": {
            "angle_kp": train_candidate["angle_kp"],
            "speed_kp": train_candidate["speed_kp"],
            "speed_ki": train_candidate["speed_ki"],
            "metrics": simulation_metrics(valid["angle_ref_rad"], valid_candidate_sim,
                                          args.output_limit, args.speed_ref_limit),
            "cost": objective_from_metrics(
                simulation_metrics(valid["angle_ref_rad"], valid_candidate_sim,
                                   args.output_limit, args.speed_ref_limit)
            ),
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
        warnings.append("optimized candidate does not improve validation cost over current PID")

    result = {
        "input_csv": str(args.csv),
        "output_dir": str(output_dir),
        "sample_period_s": dt_s,
        "sample_rate_hz": 1.0 / dt_s,
        "sample_count": int(len(t_grid)),
        "train_sample_count": int(train_count),
        "validation_sample_count": int(len(t_grid) - train_count),
        "plant_model": {
            key: value for key, value in plant.items()
            if key not in set()
        },
        "current_ref_to_output_cmd": current_gain,
        "optimization": {
            "method": "ARX output_cmd->speed plant + cascaded PID simulation + differential_evolution/L-BFGS-B",
            "objective": (
                "tracking RMSE + MAE + max error + output saturation + speed-ref saturation "
                "+ output RMS + output-rate RMS"
            ),
            "train": serializable_optimizer_result(opt_train),
            "validation": serializable_optimizer_result(opt_valid),
        },
        "recommended_pid": {
            "angle_kp": opt_valid["optimized"]["angle_kp"],
            "speed_kp": opt_valid["optimized"]["speed_kp"],
            "speed_ki": opt_valid["optimized"]["speed_ki"],
        },
        "warnings": warnings,
    }

    plot_paths: list[str] = []
    if not args.no_plots:
        plot_paths = plot_results(t_grid, values, train_count, plant, opt_train, opt_valid, output_dir)
    result["plot_paths"] = plot_paths

    result_json = output_dir / "yaw_model_pid_result.json"
    result_json.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"output_dir={output_dir}")
    print(f"plant: na={plant['na']} nb={plant['nb']} nk={plant['nk']} "
          f"valid_r2={plant['validation_simulation_r2']:.4f} "
          f"valid_rmse={plant['validation_simulation_rmse']:.4f} rad/s")
    print(f"current_ref_to_output_gain={current_gain['gain']:.6f}, r2={current_gain['r2']:.4f}")
    print("")
    print("validation model cost:")
    print(f"  current={opt_valid['current']['cost']:.6f}, optimized={opt_valid['optimized']['cost']:.6f}")
    print("")
    print("model-based PID candidate:")
    print(f"  angle Kp: {args.current_angle_kp:.6f} -> {opt_valid['optimized']['angle_kp']:.6f}")
    print(f"  speed Kp: {args.current_speed_kp:.6f} -> {opt_valid['optimized']['speed_kp']:.6f}")
    print(f"  speed Ki: {args.current_speed_ki:.6f} -> {opt_valid['optimized']['speed_ki']:.6f}")
    if warnings:
        print("")
        for warning in warnings:
            print(f"warning: {warning}")
    print(f"json={result_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
