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
from scipy import stats


PITCH_FF_MODE = 2
STEP_PHASE = 5
DEFAULT_TAIL_S = 1.0
DEFAULT_OUTPUT_SIGN = -1.0
DEFAULT_OUTPUT_LIMIT_RAW = 5000.0
DEFAULT_MAX_STABLE_SPEED_RAD_S = 0.05
DEFAULT_MAX_TRACKING_ERROR_RAD = 0.035
DEFAULT_MAX_SATURATION_RATIO = 0.02
DEFAULT_MIN_SAMPLES = 20
DEFAULT_TARGET_BIN_DEG = 0.5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze pitch gravity feedforward RTT sysid CSV.")
    parser.add_argument("csv", type=Path, help="CSV captured by gimbal_sysid_rtt_capture.py")
    parser.add_argument("--tail", type=float, default=DEFAULT_TAIL_S, help="Stable tail duration per stage in seconds")
    parser.add_argument("--output-sign", type=float, default=DEFAULT_OUTPUT_SIGN, help="Pitch output_sign used by firmware")
    parser.add_argument("--output-limit", type=float, default=DEFAULT_OUTPUT_LIMIT_RAW, help="Raw output clamp used by firmware")
    parser.add_argument("--sat-margin", type=float, default=0.98, help="Treat abs(output_cmd) above this limit ratio as saturated")
    parser.add_argument("--max-stable-speed", type=float, default=DEFAULT_MAX_STABLE_SPEED_RAD_S, help="Reject stage tail above this mean speed")
    parser.add_argument("--max-tracking-error", type=float, default=DEFAULT_MAX_TRACKING_ERROR_RAD, help="Reject stage tail above this mean abs error")
    parser.add_argument("--max-saturation-ratio", type=float, default=DEFAULT_MAX_SATURATION_RATIO, help="Reject stage tail above this saturation ratio")
    parser.add_argument("--min-samples", type=int, default=DEFAULT_MIN_SAMPLES, help="Minimum tail samples per stage")
    parser.add_argument("--fit-source", choices=["target-average", "stage"], default="target-average",
                        help="Fit target-averaged points by default to avoid overweighting repeated zero anchors")
    parser.add_argument("--target-bin-deg", type=float, default=DEFAULT_TARGET_BIN_DEG,
                        help="Angle bin size for target-averaged fitting")
    parser.add_argument("--output-dir", type=Path, default=None, help="Directory for CSV/JSON/plots")
    parser.add_argument("--no-plots", action="store_true", help="Skip PNG plot generation")
    return parser.parse_args()


def analysis_dir_for(csv_path: Path, output_dir: Path | None) -> Path:
    if output_dir is not None:
        return output_dir
    return csv_path.parent / "analysis" / f"{csv_path.stem}_pitch_ff"


def load_numeric_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    for column in df.columns:
        df[column] = pd.to_numeric(df[column], errors="coerce")
    return df.dropna(subset=["tick_ms", "mode", "phase"])


def fit_linear_model(x: np.ndarray, y: np.ndarray, names: list[str]) -> dict[str, object]:
    beta, *_ = np.linalg.lstsq(x, y, rcond=None)
    y_hat = x @ beta
    residual = y - y_hat
    n = int(len(y))
    k = int(x.shape[1])
    sse = float(np.sum(residual * residual))
    sst = float(np.sum((y - np.mean(y)) ** 2))
    rmse = math.sqrt(sse / max(n, 1))
    mae = float(np.mean(np.abs(residual)))
    max_abs = float(np.max(np.abs(residual)))
    r2 = 1.0 - sse / sst if sst > 1e-12 else float("nan")
    adj_r2 = 1.0 - (1.0 - r2) * (n - 1) / max(n - k, 1) if math.isfinite(r2) else float("nan")
    sigma2 = sse / max(n - k, 1)
    cov = sigma2 * np.linalg.pinv(x.T @ x)
    se = np.sqrt(np.maximum(np.diag(cov), 0.0))
    tcrit = stats.t.ppf(0.975, max(n - k, 1)) if n > k else 1.96
    t_value = np.divide(beta, se, out=np.full_like(beta, np.nan), where=se > 1e-12)
    p_value = 2.0 * (1.0 - stats.t.cdf(np.abs(t_value), max(n - k, 1))) if n > k else np.full_like(beta, np.nan)
    aic = n * math.log(max(sse / max(n, 1), 1e-12)) + 2 * k
    bic = n * math.log(max(sse / max(n, 1), 1e-12)) + k * math.log(max(n, 2))

    loo_errors: list[float] = []
    if n > k + 1:
        for idx in range(n):
            mask = np.ones(n, dtype=bool)
            mask[idx] = False
            loo_beta, *_ = np.linalg.lstsq(x[mask], y[mask], rcond=None)
            loo_errors.append(float(y[idx] - x[idx] @ loo_beta))
    loo_rmse = math.sqrt(float(np.mean(np.square(loo_errors)))) if loo_errors else float("nan")

    coefficients = {}
    for idx, name in enumerate(names):
        coefficients[name] = {
            "value": float(beta[idx]),
            "std_error": float(se[idx]),
            "ci95_low": float(beta[idx] - tcrit * se[idx]),
            "ci95_high": float(beta[idx] + tcrit * se[idx]),
            "t_value": float(t_value[idx]),
            "p_value": float(p_value[idx]),
        }

    return {
        "names": names,
        "coefficients": coefficients,
        "beta": beta,
        "prediction": y_hat,
        "residual": residual,
        "n": n,
        "k": k,
        "rmse": rmse,
        "mae": mae,
        "max_abs_error": max_abs,
        "r2": float(r2),
        "adjusted_r2": float(adj_r2),
        "aic": float(aic),
        "bic": float(bic),
        "loocv_rmse": float(loo_rmse),
    }


def summarize_stages(df: pd.DataFrame, args: argparse.Namespace) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    sat_threshold = args.output_limit * args.sat_margin

    for seq_index, group in df.groupby("seq_index"):
        stage = group.sort_values("tick_ms")
        if stage.empty:
            continue

        max_tick = float(stage["tick_ms"].iloc[-1])
        stable = stage[stage["tick_ms"] >= max_tick - args.tail * 1000.0]
        if stable.empty:
            continue

        angle_ref = stable["angle_ref_rad"].to_numpy()
        angle_feedback = stable["angle_feedback_rad"].to_numpy()
        speed = stable["speed_feedback_rad_s"].to_numpy()
        voltage_ref = stable["voltage_ref_raw"].to_numpy()
        output_cmd = stable["output_cmd"].to_numpy()
        output_ff = stable["output_ff_raw"].to_numpy()
        error = angle_ref - angle_feedback
        u_hold = output_ff + args.output_sign * voltage_ref
        sat_ratio = float(np.mean(np.abs(output_cmd) >= sat_threshold))

        reject_reasons: list[str] = []
        if len(stable) < args.min_samples:
            reject_reasons.append("too_few_samples")
        if float(np.mean(np.abs(speed))) > args.max_stable_speed:
            reject_reasons.append("not_static")
        if float(np.mean(np.abs(error))) > args.max_tracking_error:
            reject_reasons.append("tracking_error")
        if sat_ratio > args.max_saturation_ratio:
            reject_reasons.append("output_saturated")

        theta = float(np.mean(angle_feedback))
        rows.append({
            "seq_index": int(seq_index),
            "accepted": len(reject_reasons) == 0,
            "reject_reason": ",".join(reject_reasons),
            "sample_count": int(len(stable)),
            "duration_s": float((stage["tick_ms"].iloc[-1] - stage["tick_ms"].iloc[0]) / 1000.0),
            "theta_rad": theta,
            "theta_deg": math.degrees(theta),
            "angle_ref_rad": float(np.mean(angle_ref)),
            "angle_ref_deg": math.degrees(float(np.mean(angle_ref))),
            "tracking_error_mean_rad": float(np.mean(error)),
            "tracking_error_rms_rad": float(math.sqrt(np.mean(error * error))),
            "tracking_error_abs_mean_rad": float(np.mean(np.abs(error))),
            "u_hold_raw": float(np.mean(u_hold)),
            "u_hold_std_raw": float(np.std(u_hold, ddof=1)) if len(u_hold) > 1 else 0.0,
            "voltage_ref_raw": float(np.mean(voltage_ref)),
            "output_ff_raw": float(np.mean(output_ff)),
            "output_cmd_raw": float(np.mean(output_cmd)),
            "output_cmd_abs_max": float(np.max(np.abs(output_cmd))),
            "output_saturation_ratio": sat_ratio,
            "speed_abs_mean_rad_s": float(np.mean(np.abs(speed))),
            "speed_abs_max_rad_s": float(np.max(np.abs(speed))),
        })

    return pd.DataFrame(rows).sort_values("seq_index").reset_index(drop=True)


def summarize_targets(stage_df: pd.DataFrame, args: argparse.Namespace) -> pd.DataFrame:
    accepted = stage_df[stage_df["accepted"]].copy()
    if accepted.empty:
        return pd.DataFrame()

    bin_deg = max(float(args.target_bin_deg), 1e-6)
    accepted["target_bin_deg"] = np.round(accepted["angle_ref_deg"] / bin_deg) * bin_deg
    rows: list[dict[str, object]] = []

    for target_bin_deg, group in accepted.groupby("target_bin_deg"):
        group = group.sort_values("seq_index")
        u_hold = group["u_hold_raw"].to_numpy()
        rows.append({
            "target_bin_deg": float(target_bin_deg),
            "stage_count": int(len(group)),
            "seq_indices": ",".join(str(int(v)) for v in group["seq_index"].to_numpy()),
            "theta_rad": float(group["theta_rad"].mean()),
            "theta_deg": float(group["theta_deg"].mean()),
            "angle_ref_rad": float(group["angle_ref_rad"].mean()),
            "angle_ref_deg": float(group["angle_ref_deg"].mean()),
            "tracking_error_abs_mean_rad": float(group["tracking_error_abs_mean_rad"].mean()),
            "u_hold_raw": float(np.mean(u_hold)),
            "u_hold_std_raw": float(np.std(u_hold, ddof=1)) if len(u_hold) > 1 else 0.0,
            "u_hold_min_raw": float(np.min(u_hold)),
            "u_hold_max_raw": float(np.max(u_hold)),
            "u_hold_spread_raw": float(np.max(u_hold) - np.min(u_hold)),
            "speed_abs_mean_rad_s": float(group["speed_abs_mean_rad_s"].mean()),
            "output_saturation_ratio": float(group["output_saturation_ratio"].max()),
        })

    return pd.DataFrame(rows).sort_values("target_bin_deg").reset_index(drop=True)


def add_fit_columns(stage_df: pd.DataFrame, reduced: dict[str, object], full: dict[str, object]) -> pd.DataFrame:
    result = stage_df.copy()
    theta = result["theta_rad"].to_numpy()
    x_reduced = np.column_stack([np.sin(theta), np.ones_like(theta)])
    x_full = np.column_stack([np.sin(theta), np.cos(theta), np.ones_like(theta)])
    result["pred_reduced_raw"] = x_reduced @ reduced["beta"]
    result["residual_reduced_raw"] = result["u_hold_raw"] - result["pred_reduced_raw"]
    result["pred_full_raw"] = x_full @ full["beta"]
    result["residual_full_raw"] = result["u_hold_raw"] - result["pred_full_raw"]
    return result


def add_data_quality_warnings(target_df: pd.DataFrame, warnings: list[str]) -> None:
    if target_df.empty or "u_hold_spread_raw" not in target_df.columns:
        return

    repeated = target_df[target_df["stage_count"] > 1]
    if repeated.empty:
        return

    max_spread = float(repeated["u_hold_spread_raw"].max())
    median_spread = float(repeated["u_hold_spread_raw"].median())
    if max_spread > 800.0:
        warnings.append(f"large repeated-target hysteresis detected; max u_hold spread is {max_spread:.1f} raw")
    elif median_spread > 400.0:
        warnings.append(f"repeated-target hysteresis is still visible; median spread is {median_spread:.1f} raw")


def choose_model(reduced: dict[str, object], full: dict[str, object]) -> tuple[str, list[str]]:
    warnings: list[str] = []
    cos = full["coefficients"]["cos(theta)"]
    sin_abs = abs(full["coefficients"]["sin(theta)"]["value"])
    cos_abs = abs(cos["value"])
    rmse_gain = (reduced["rmse"] - full["rmse"]) / max(reduced["rmse"], 1e-9)

    if cos_abs > 0.15 * max(sin_abs, 1.0) and rmse_gain > 0.20 and cos["p_value"] < 0.05:
        warnings.append("cos term is significant; current firmware model may need sin + cos + offset")
        return "full", warnings

    if reduced["r2"] < 0.85:
        warnings.append("reduced model R2 is low; inspect residual plot before updating firmware")
    if reduced["loocv_rmse"] > 1.5 * max(reduced["rmse"], 1.0):
        warnings.append("leave-one-out error is much larger than fit RMSE; data may contain outliers")
    return "reduced", warnings


def plot_results(raw_df: pd.DataFrame, stage_df: pd.DataFrame, accepted_df: pd.DataFrame,
                 target_df: pd.DataFrame, fit_source: str,
                 reduced: dict[str, object], full: dict[str, object], output_dir: Path) -> list[str]:
    paths: list[str] = []
    t = (raw_df["tick_ms"] - raw_df["tick_ms"].iloc[0]) / 1000.0

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t, np.degrees(raw_df["angle_ref_rad"]), label="target", linewidth=1.0)
    axes[0].plot(t, np.degrees(raw_df["angle_feedback_rad"]), label="actual", linewidth=1.0)
    axes[0].set_ylabel("Pitch angle (deg)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, raw_df["speed_feedback_rad_s"], label="speed", linewidth=0.9)
    axes[1].set_ylabel("Speed (rad/s)")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(t, raw_df["output_cmd"], label="output_cmd", linewidth=0.9)
    axes[2].plot(t, raw_df["voltage_ref_raw"], label="voltage_ref", linewidth=0.9)
    axes[2].set_ylabel("Raw output")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)
    fig.suptitle("Pitch feedforward sysid time series")
    fig.tight_layout()
    path = output_dir / "pitch_ff_time_series.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    theta_grid = np.linspace(stage_df["theta_rad"].min(), stage_df["theta_rad"].max(), 300)
    x_reduced = np.column_stack([np.sin(theta_grid), np.ones_like(theta_grid)])
    x_full = np.column_stack([np.sin(theta_grid), np.cos(theta_grid), np.ones_like(theta_grid)])
    fig, ax = plt.subplots(figsize=(10, 6))
    rejected = stage_df[~stage_df["accepted"]]
    ax.scatter(np.degrees(accepted_df["theta_rad"]), accepted_df["u_hold_raw"],
               label="accepted stages", s=40, alpha=0.45)
    if not target_df.empty:
        ax.scatter(np.degrees(target_df["theta_rad"]), target_df["u_hold_raw"],
                   label="target averages", s=80, edgecolors="black", linewidths=0.8)
    if not rejected.empty:
        ax.scatter(np.degrees(rejected["theta_rad"]), rejected["u_hold_raw"], label="rejected stages", marker="x", s=70)
    ax.plot(np.degrees(theta_grid), x_reduced @ reduced["beta"], label="A*sin(theta)+C", linewidth=2)
    ax.plot(np.degrees(theta_grid), x_full @ full["beta"], label="A*sin(theta)+B*cos(theta)+C", linewidth=1.5)
    text = (
        f"fit source: {fit_source}\n"
        f"reduced: R2={reduced['r2']:.4f}, RMSE={reduced['rmse']:.1f}, LOOCV={reduced['loocv_rmse']:.1f}\n"
        f"full: R2={full['r2']:.4f}, RMSE={full['rmse']:.1f}, LOOCV={full['loocv_rmse']:.1f}"
    )
    ax.text(0.02, 0.98, text, transform=ax.transAxes, va="top", bbox={"facecolor": "white", "alpha": 0.85})
    ax.set_xlabel("Pitch angle theta (deg)")
    ax.set_ylabel("u_hold raw")
    ax.set_title("Pitch gravity feedforward fit")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = output_dir / "pitch_ff_fit.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].bar(stage_df["seq_index"], stage_df["residual_reduced_raw"], label="reduced residual")
    axes[0].axhline(0.0, color="black", linewidth=0.8)
    axes[0].set_ylabel("Residual raw")
    axes[0].set_title("Residual by stage")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(stage_df["seq_index"], stage_df["speed_abs_mean_rad_s"], marker="o", label="mean |speed|")
    axes[1].plot(stage_df["seq_index"], stage_df["output_saturation_ratio"], marker="s", label="saturation ratio")
    axes[1].set_xlabel("Stage seq_index")
    axes[1].set_ylabel("Diagnostic value")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    path = output_dir / "pitch_ff_residuals.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    if not target_df.empty:
        fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        axes[0].bar(target_df["target_bin_deg"], target_df["residual_reduced_raw"], width=3.0, label="target-average residual")
        axes[0].axhline(0.0, color="black", linewidth=0.8)
        axes[0].set_ylabel("Residual raw")
        axes[0].set_title("Residual by target angle")
        axes[0].grid(True, alpha=0.3)
        axes[1].bar(target_df["target_bin_deg"], target_df["u_hold_spread_raw"], width=3.0, label="same-target spread")
        axes[1].set_xlabel("Target angle bin (deg)")
        axes[1].set_ylabel("u_hold spread raw")
        axes[1].grid(True, alpha=0.3)
        fig.tight_layout()
        path = output_dir / "pitch_ff_target_residuals.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths.append(str(path))

    return paths


def serializable_model(model: dict[str, object]) -> dict[str, object]:
    return {
        "coefficients": model["coefficients"],
        "n": model["n"],
        "k": model["k"],
        "rmse": model["rmse"],
        "mae": model["mae"],
        "max_abs_error": model["max_abs_error"],
        "r2": model["r2"],
        "adjusted_r2": model["adjusted_r2"],
        "aic": model["aic"],
        "bic": model["bic"],
        "loocv_rmse": model["loocv_rmse"],
    }


def main() -> int:
    args = parse_args()
    output_dir = analysis_dir_for(args.csv, args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    df = load_numeric_csv(args.csv)
    raw_df = df[(df["mode"].astype(int) == PITCH_FF_MODE)].copy()
    step_df = raw_df[(raw_df["phase"].astype(int) == STEP_PHASE)].copy()
    if step_df.empty:
        print("not enough pitch feedforward stages found")
        return 2

    stage_df = summarize_stages(step_df, args)
    if stage_df.empty:
        print("no usable pitch feedforward stages found")
        return 2

    accepted_df = stage_df[stage_df["accepted"]].copy()
    if len(accepted_df) < 4:
        print("not enough accepted stages; check stage_summary.csv for reject reasons")
        stage_df.to_csv(output_dir / "pitch_ff_stage_summary.csv", index=False)
        return 2

    target_df = summarize_targets(stage_df, args)
    fit_df = target_df if args.fit_source == "target-average" else accepted_df
    if len(fit_df) < 4:
        print("not enough fit points; check pitch_ff_stage_summary.csv and pitch_ff_target_summary.csv")
        stage_df.to_csv(output_dir / "pitch_ff_stage_summary.csv", index=False)
        target_df.to_csv(output_dir / "pitch_ff_target_summary.csv", index=False)
        return 2

    theta = fit_df["theta_rad"].to_numpy()
    values = fit_df["u_hold_raw"].to_numpy()
    reduced_x = np.column_stack([np.sin(theta), np.ones_like(theta)])
    full_x = np.column_stack([np.sin(theta), np.cos(theta), np.ones_like(theta)])
    reduced = fit_linear_model(reduced_x, values, ["sin(theta)", "offset"])
    full = fit_linear_model(full_x, values, ["sin(theta)", "cos(theta)", "offset"])
    stage_df = add_fit_columns(stage_df, reduced, full)
    target_df = add_fit_columns(target_df, reduced, full) if not target_df.empty else target_df
    accepted_df = stage_df[stage_df["accepted"]].copy()
    selected_model, warnings = choose_model(reduced, full)
    add_data_quality_warnings(target_df, warnings)

    stage_csv = output_dir / "pitch_ff_stage_summary.csv"
    target_csv = output_dir / "pitch_ff_target_summary.csv"
    result_json = output_dir / "pitch_ff_result.json"
    stage_df.to_csv(stage_csv, index=False)
    target_df.to_csv(target_csv, index=False)

    plot_paths: list[str] = []
    if not args.no_plots:
        plot_paths = plot_results(raw_df, stage_df, accepted_df, target_df, args.fit_source, reduced, full, output_dir)

    reduced_a = reduced["coefficients"]["sin(theta)"]["value"]
    reduced_c = reduced["coefficients"]["offset"]["value"]
    result = {
        "input_csv": str(args.csv),
        "stage_summary_csv": str(stage_csv),
        "target_summary_csv": str(target_csv),
        "plot_paths": plot_paths,
        "total_stages": int(len(stage_df)),
        "accepted_stages": int(len(accepted_df)),
        "rejected_stages": int(len(stage_df) - len(accepted_df)),
        "fit_source": args.fit_source,
        "fit_points": int(len(fit_df)),
        "target_points": int(len(target_df)),
        "filters": {
            "tail_s": args.tail,
            "output_sign": args.output_sign,
            "output_limit": args.output_limit,
            "sat_margin": args.sat_margin,
            "max_stable_speed_rad_s": args.max_stable_speed,
            "max_tracking_error_rad": args.max_tracking_error,
            "max_saturation_ratio": args.max_saturation_ratio,
            "min_samples": args.min_samples,
            "target_bin_deg": args.target_bin_deg,
        },
        "selected_model": selected_model,
        "models": {
            "reduced_sin_offset": serializable_model(reduced),
            "full_sin_cos_offset": serializable_model(full),
        },
        "firmware_compatible_macros": {
            "GIMBAL_PITCH_OUTPUT_FF_SIN_RAW": reduced_a,
            "GIMBAL_PITCH_OUTPUT_FF_OFFSET_RAW": reduced_c,
        },
        "warnings": warnings,
    }
    result_json.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"output_dir={output_dir}")
    print(f"stages={len(stage_df)}, accepted={len(accepted_df)}, rejected={len(stage_df) - len(accepted_df)}")
    print(f"fit_source={args.fit_source}, fit_points={len(fit_df)}, target_points={len(target_df)}")
    if not target_df.empty and "u_hold_spread_raw" in target_df.columns:
        repeated = target_df[target_df["stage_count"] > 1]
        if not repeated.empty:
            print(f"same-target spread: median={repeated['u_hold_spread_raw'].median():.3f}, max={repeated['u_hold_spread_raw'].max():.3f} raw")
    print("")
    print("fit: u_hold = A*sin(theta) + C")
    print(f"  A = {reduced_a:+.6f}")
    print(f"  C = {reduced_c:+.6f}")
    print(f"  R2 = {reduced['r2']:.5f}, RMSE = {reduced['rmse']:.3f}, LOOCV_RMSE = {reduced['loocv_rmse']:.3f}")
    print("")
    print("fit: u_hold = A*sin(theta) + B*cos(theta) + C")
    print(f"  A = {full['coefficients']['sin(theta)']['value']:+.6f}")
    print(f"  B = {full['coefficients']['cos(theta)']['value']:+.6f}")
    print(f"  C = {full['coefficients']['offset']['value']:+.6f}")
    print(f"  R2 = {full['r2']:.5f}, RMSE = {full['rmse']:.3f}, LOOCV_RMSE = {full['loocv_rmse']:.3f}")
    print("")
    print("current firmware-compatible macros:")
    print(f"#define GIMBAL_PITCH_OUTPUT_FF_SIN_RAW       ({reduced_a:+.6f}f)")
    print(f"#define GIMBAL_PITCH_OUTPUT_FF_OFFSET_RAW    ({reduced_c:+.6f}f)")
    if warnings:
        print("")
        for warning in warnings:
            print(f"warning: {warning}")
    print(f"json={result_json}")
    print(f"stage_csv={stage_csv}")
    print(f"target_csv={target_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
