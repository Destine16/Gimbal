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


PITCH_FF_SWEEP_MODE = 12
SWEEP_PHASE = 8


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit pitch physical feedforward from low-speed constant-rate sweep data."
    )
    parser.add_argument("csv", type=Path, help="CSV captured by gimbal_sysid_rtt_capture.py")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--pitch-output-sign", type=float, default=-1.0)
    parser.add_argument("--speed-source", choices=["speed_ref", "speed_feedback", "motor_speed"], default="speed_ref")
    parser.add_argument("--output-limit", type=float, default=5000.0)
    parser.add_argument("--sat-margin", type=float, default=0.98)
    parser.add_argument("--trim-edge-s", type=float, default=0.8)
    parser.add_argument("--min-speed-rad-s", type=float, default=0.04)
    parser.add_argument("--max-accel-rad-s2", type=float, default=1.2)
    parser.add_argument("--max-tracking-error-deg", type=float, default=2.5)
    parser.add_argument("--v0-min", type=float, default=0.04)
    parser.add_argument("--v0-max", type=float, default=0.60)
    parser.add_argument("--v0-count", type=int, default=80)
    parser.add_argument("--no-plots", action="store_true")
    return parser.parse_args()


def output_dir_for(csv_path: Path, output_dir: Path | None) -> Path:
    if output_dir is not None:
        return output_dir
    return csv_path.parent / "analysis" / f"{csv_path.stem}_pitch_ff_sweep"


def speed_column_name(speed_source: str) -> str:
    if speed_source == "speed_ref":
        return "speed_ref_rad_s"
    if speed_source == "speed_feedback":
        return "speed_feedback_rad_s"
    return "motor_speed_rad_s"


def metrics(y: np.ndarray, pred: np.ndarray, param_count: int) -> dict[str, float]:
    err = y - pred
    n = max(int(y.size), 1)
    rss = float(np.sum(err * err))
    tss = float(np.sum((y - np.mean(y)) ** 2))
    return {
        "rmse_raw": float(math.sqrt(np.mean(err * err))),
        "mae_raw": float(np.mean(np.abs(err))),
        "max_abs_raw": float(np.max(np.abs(err))),
        "r2": float("nan") if tss <= 1e-12 else 1.0 - rss / tss,
        "bic": n * math.log(max(rss / n, 1e-12)) + param_count * math.log(n),
    }


def fit_linear(features: np.ndarray, y: np.ndarray) -> np.ndarray:
    beta, *_ = np.linalg.lstsq(features, y, rcond=None)
    return beta


def segment_loocv_rmse(x_builder, theta: np.ndarray, speed: np.ndarray, y: np.ndarray,
                       seq: np.ndarray, v0: float | None = None) -> float:
    pred_all = np.full_like(y, np.nan, dtype=float)
    unique_seq = np.unique(seq)
    for held in unique_seq:
        train = seq != held
        test = seq == held
        if int(np.sum(train)) < 20 or int(np.sum(test)) < 5:
            continue
        x_train = x_builder(theta[train], speed[train], v0)
        beta = fit_linear(x_train, y[train])
        pred_all[test] = x_builder(theta[test], speed[test], v0) @ beta
    valid = np.isfinite(pred_all)
    if int(np.sum(valid)) < 20:
        return float("nan")
    err = y[valid] - pred_all[valid]
    return float(math.sqrt(np.mean(err * err)))


def x_gravity(theta: np.ndarray, speed: np.ndarray, v0: float | None = None) -> np.ndarray:
    del speed, v0
    return np.column_stack([np.sin(theta), np.ones_like(theta)])


def x_gravity_coulomb(theta: np.ndarray, speed: np.ndarray, v0: float | None) -> np.ndarray:
    if v0 is None or v0 <= 0.0:
        raise ValueError("v0 must be positive")
    return np.column_stack([np.sin(theta), np.ones_like(theta), np.tanh(speed / v0)])


def x_full(theta: np.ndarray, speed: np.ndarray, v0: float | None) -> np.ndarray:
    if v0 is None or v0 <= 0.0:
        raise ValueError("v0 must be positive")
    return np.column_stack([np.sin(theta), np.ones_like(theta), np.tanh(speed / v0), speed])


def select_samples(df: pd.DataFrame, args: argparse.Namespace) -> pd.DataFrame:
    speed_col = speed_column_name(args.speed_source)
    required = {
        "tick_ms",
        "seq_index",
        "mode",
        "phase",
        "angle_ref_rad",
        "angle_feedback_rad",
        speed_col,
        "voltage_ref_raw",
        "output_ff_raw",
        "output_cmd",
    }
    missing = required.difference(df.columns)
    if missing:
        raise SystemExit(f"missing required columns: {sorted(missing)}")

    active = df[(df["mode"] == PITCH_FF_SWEEP_MODE) & (df["phase"] == SWEEP_PHASE)].copy()
    if active.empty:
        raise SystemExit("no pitch feedforward sweep active samples found")

    t_s = (active["tick_ms"].to_numpy(dtype=float) - float(active["tick_ms"].iloc[0])) / 1000.0
    active["t_s"] = t_s
    speed = active[speed_col].to_numpy(dtype=float)
    dt_s = np.gradient(t_s)
    active["accel_rad_s2"] = np.gradient(speed) / np.maximum(np.abs(dt_s), 1e-3)
    active["tracking_error_deg"] = np.degrees(
        active["angle_ref_rad"].to_numpy(dtype=float) -
        active["angle_feedback_rad"].to_numpy(dtype=float)
    )

    in_segment_mid = np.ones(len(active), dtype=bool)
    for _, group in active.groupby("seq_index", sort=False):
        idx = group.index.to_numpy()
        local_t = active.loc[idx, "t_s"].to_numpy(dtype=float)
        if local_t.size == 0:
            continue
        in_segment_mid[active.index.get_indexer(idx)] = (
            (local_t - local_t[0] >= args.trim_edge_s) &
            (local_t[-1] - local_t >= args.trim_edge_s)
        )

    mask = (
        in_segment_mid &
        (np.abs(active["output_cmd"].to_numpy(dtype=float)) < args.output_limit * args.sat_margin) &
        (np.abs(speed) >= args.min_speed_rad_s) &
        (np.abs(active["accel_rad_s2"].to_numpy(dtype=float)) <= args.max_accel_rad_s2) &
        (np.abs(active["tracking_error_deg"].to_numpy(dtype=float)) <= args.max_tracking_error_deg)
    )
    selected = active[mask].copy()
    if len(selected) < 80:
        raise SystemExit(f"not enough selected samples: {len(selected)} / {len(active)}")
    return selected


def main() -> int:
    args = parse_args()
    df = pd.read_csv(args.csv)
    selected = select_samples(df, args)
    speed_col = speed_column_name(args.speed_source)

    theta = selected["angle_feedback_rad"].to_numpy(dtype=float)
    speed = selected[speed_col].to_numpy(dtype=float)
    y = (
        selected["output_ff_raw"].to_numpy(dtype=float) +
        args.pitch_output_sign * selected["voltage_ref_raw"].to_numpy(dtype=float)
    )
    seq = selected["seq_index"].to_numpy(dtype=int)

    gravity_beta = fit_linear(x_gravity(theta, speed), y)
    gravity_pred = x_gravity(theta, speed) @ gravity_beta
    gravity_metrics = metrics(y, gravity_pred, 2)
    gravity_loocv = segment_loocv_rmse(x_gravity, theta, speed, y, seq)

    candidates: list[dict[str, object]] = []
    v0_values = np.linspace(args.v0_min, args.v0_max, max(args.v0_count, 2))
    for v0 in v0_values:
        coulomb_x = x_gravity_coulomb(theta, speed, float(v0))
        coulomb_beta = fit_linear(coulomb_x, y)
        coulomb_pred = coulomb_x @ coulomb_beta
        coulomb_m = metrics(y, coulomb_pred, 3)
        coulomb_m["segment_loocv_rmse_raw"] = segment_loocv_rmse(
            x_gravity_coulomb, theta, speed, y, seq, float(v0)
        )

        full_x = x_full(theta, speed, float(v0))
        full_beta = fit_linear(full_x, y)
        full_pred = full_x @ full_beta
        full_m = metrics(y, full_pred, 4)
        full_m["segment_loocv_rmse_raw"] = segment_loocv_rmse(
            x_full, theta, speed, y, seq, float(v0)
        )
        candidates.append({
            "v0": float(v0),
            "coulomb_beta": coulomb_beta.tolist(),
            "coulomb_metrics": coulomb_m,
            "full_beta": full_beta.tolist(),
            "full_metrics": full_m,
        })

    def candidate_key(item: dict[str, object], model_key: str) -> tuple[float, float]:
        model_metrics = item[model_key]
        assert isinstance(model_metrics, dict)
        loocv = float(model_metrics["segment_loocv_rmse_raw"])
        bic = float(model_metrics["bic"])
        return (loocv if math.isfinite(loocv) else float("inf"), bic)

    best_coulomb = min(candidates, key=lambda item: candidate_key(item, "coulomb_metrics"))
    best_full = min(candidates, key=lambda item: candidate_key(item, "full_metrics"))

    full_beta = np.asarray(best_full["full_beta"], dtype=float)
    full_v0 = float(best_full["v0"])
    full_pred = x_full(theta, speed, full_v0) @ full_beta
    speed_term = full_beta[3] * speed
    speed_ff_max_raw = float(np.percentile(np.abs(speed_term), 99.0))
    if speed_ff_max_raw < 1.0:
        speed_ff_max_raw = 0.0

    result = {
        "source_csv": str(args.csv),
        "mode": PITCH_FF_SWEEP_MODE,
        "phase": SWEEP_PHASE,
        "speed_source": args.speed_source,
        "selected_samples": int(len(selected)),
        "selected_seq_count": int(selected["seq_index"].nunique()),
        "selection": {
            "trim_edge_s": args.trim_edge_s,
            "min_speed_rad_s": args.min_speed_rad_s,
            "max_accel_rad_s2": args.max_accel_rad_s2,
            "max_tracking_error_deg": args.max_tracking_error_deg,
        },
        "gravity_only": {
            "sin_raw": float(gravity_beta[0]),
            "offset_raw": float(gravity_beta[1]),
            "metrics": {**gravity_metrics, "segment_loocv_rmse_raw": gravity_loocv},
        },
        "best_coulomb": best_coulomb,
        "best_full": best_full,
        "recommended_macros": {
            "GIMBAL_PITCH_OUTPUT_FF_SIN_RAW": float(full_beta[0]),
            "GIMBAL_PITCH_OUTPUT_FF_OFFSET_RAW": float(full_beta[1]),
            "GIMBAL_PITCH_OUTPUT_FF_HYST_RAW": float(full_beta[2]),
            "GIMBAL_PITCH_OUTPUT_FF_HYST_TRANSITION_RAD_S": full_v0,
            "GIMBAL_PITCH_OUTPUT_FF_SPEED_RAW": float(full_beta[3]),
            "GIMBAL_PITCH_OUTPUT_FF_SPEED_MAX_RAW": speed_ff_max_raw,
        },
    }

    output_dir = output_dir_for(args.csv, args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    result_path = output_dir / "pitch_ff_sweep_result.json"
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if not args.no_plots:
        t = selected["t_s"].to_numpy(dtype=float)
        fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=False)
        axes[0].plot(t, np.degrees(selected["angle_ref_rad"]), label="target", linewidth=0.8)
        axes[0].plot(t, np.degrees(selected["angle_feedback_rad"]), label="actual", linewidth=0.8)
        axes[0].set_ylabel("Pitch (deg)")
        axes[0].legend(loc="best")
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(t, y, label="required output", linewidth=0.7)
        axes[1].plot(t, full_pred, label="model", linewidth=0.7)
        axes[1].set_ylabel("Raw output")
        axes[1].legend(loc="best")
        axes[1].grid(True, alpha=0.3)
        axes[2].scatter(np.degrees(theta), y - full_pred, c=speed, s=5, cmap="coolwarm", alpha=0.45)
        axes[2].set_xlabel("Pitch angle (deg)")
        axes[2].set_ylabel("Residual raw")
        axes[2].grid(True, alpha=0.3)
        axes[3].scatter(speed, y - (full_beta[0] * np.sin(theta) + full_beta[1]),
                        s=5, alpha=0.45, label="gravity residual")
        speed_line = np.linspace(float(np.min(speed)), float(np.max(speed)), 300)
        speed_model = full_beta[2] * np.tanh(speed_line / full_v0) + full_beta[3] * speed_line
        axes[3].plot(speed_line, speed_model, color="tab:red", label="friction+viscous")
        axes[3].set_xlabel(f"{args.speed_source} (rad/s)")
        axes[3].set_ylabel("Residual raw")
        axes[3].legend(loc="best")
        axes[3].grid(True, alpha=0.3)
        fig.suptitle("Pitch low-speed sweep feedforward fit")
        fig.tight_layout()
        plot_path = output_dir / "pitch_ff_sweep_fit.png"
        fig.savefig(plot_path, dpi=160)
        plt.close(fig)

    best_full_metrics = best_full["full_metrics"]
    assert isinstance(best_full_metrics, dict)
    print(f"selected samples: {len(selected)}")
    print(f"segments: {selected['seq_index'].nunique()}")
    print("gravity only:")
    print(
        f"  sin={gravity_beta[0]:+.4f}, offset={gravity_beta[1]:+.4f}, "
        f"rmse={gravity_metrics['rmse_raw']:.2f}, loocv={gravity_loocv:.2f}"
    )
    print("recommended full model:")
    print(
        f"  sin={full_beta[0]:+.4f}, offset={full_beta[1]:+.4f}, "
        f"hyst={full_beta[2]:+.4f}, v0={full_v0:.4f}, speed={full_beta[3]:+.4f}"
    )
    print(
        f"  rmse={best_full_metrics['rmse_raw']:.2f}, "
        f"loocv={best_full_metrics['segment_loocv_rmse_raw']:.2f}, "
        f"r2={best_full_metrics['r2']:.4f}"
    )
    print("macros:")
    for name, value in result["recommended_macros"].items():
        print(f"#define {name:<46} ({value:+.4f}f)")
    print(f"wrote {result_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
