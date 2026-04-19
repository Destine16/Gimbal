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


PITCH_HYST_MODE = 4
STEP_PHASE = 5
DEFAULT_TAIL_S = 1.0
DEFAULT_OUTPUT_SIGN = -1.0
DEFAULT_OUTPUT_LIMIT_RAW = 5000.0
DEFAULT_GRAVITY_SIN_RAW = -1115.8459
DEFAULT_GRAVITY_OFFSET_RAW = -97.4462
DEFAULT_MAX_STABLE_SPEED_RAD_S = 0.06
DEFAULT_MAX_TRACKING_ERROR_RAD = 0.04
DEFAULT_MAX_SATURATION_RATIO = 0.02
DEFAULT_MIN_SAMPLES = 20
DEFAULT_TARGET_BIN_DEG = 0.5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze pitch hysteresis RTT sysid CSV.")
    parser.add_argument("csv", type=Path, help="CSV captured by gimbal_sysid_rtt_capture.py")
    parser.add_argument("--tail", type=float, default=DEFAULT_TAIL_S, help="Stable tail duration per stage in seconds")
    parser.add_argument("--output-sign", type=float, default=DEFAULT_OUTPUT_SIGN, help="Pitch output_sign used by firmware")
    parser.add_argument("--output-limit", type=float, default=DEFAULT_OUTPUT_LIMIT_RAW, help="Raw output clamp used by firmware")
    parser.add_argument("--gravity-sin", type=float, default=DEFAULT_GRAVITY_SIN_RAW, help="Current gravity feedforward sin coefficient")
    parser.add_argument("--gravity-offset", type=float, default=DEFAULT_GRAVITY_OFFSET_RAW, help="Current gravity feedforward offset")
    parser.add_argument("--sat-margin", type=float, default=0.98, help="Treat abs(output_cmd) above this limit ratio as saturated")
    parser.add_argument("--max-stable-speed", type=float, default=DEFAULT_MAX_STABLE_SPEED_RAD_S, help="Reject stage tail above this mean speed")
    parser.add_argument("--max-tracking-error", type=float, default=DEFAULT_MAX_TRACKING_ERROR_RAD, help="Reject stage tail above this mean abs error")
    parser.add_argument("--max-saturation-ratio", type=float, default=DEFAULT_MAX_SATURATION_RATIO, help="Reject stage tail above this saturation ratio")
    parser.add_argument("--min-samples", type=int, default=DEFAULT_MIN_SAMPLES, help="Minimum tail samples per stage")
    parser.add_argument("--target-bin-deg", type=float, default=DEFAULT_TARGET_BIN_DEG, help="Angle bin size for repeated-target pairing")
    parser.add_argument("--output-dir", type=Path, default=None, help="Directory for CSV/JSON/plots")
    parser.add_argument("--no-plots", action="store_true", help="Skip PNG plot generation")
    return parser.parse_args()


def analysis_dir_for(csv_path: Path, output_dir: Path | None) -> Path:
    if output_dir is not None:
        return output_dir
    return csv_path.parent / "analysis" / f"{csv_path.stem}_pitch_hysteresis"


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
    r2 = 1.0 - sse / sst if sst > 1e-12 else float("nan")
    sigma2 = sse / max(n - k, 1)
    cov = sigma2 * np.linalg.pinv(x.T @ x)
    se = np.sqrt(np.maximum(np.diag(cov), 0.0))
    dof = max(n - k, 1)
    tcrit = stats.t.ppf(0.975, dof) if n > k else 1.96
    t_value = np.divide(beta, se, out=np.full_like(beta, np.nan), where=se > 1e-12)
    p_value = 2.0 * (1.0 - stats.t.cdf(np.abs(t_value), dof)) if n > k else np.full_like(beta, np.nan)

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
        "mae": float(np.mean(np.abs(residual))),
        "max_abs_error": float(np.max(np.abs(residual))),
        "r2": float(r2),
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
        gravity = args.gravity_sin * math.sin(theta) + args.gravity_offset
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
            "tracking_error_abs_mean_rad": float(np.mean(np.abs(error))),
            "u_hold_raw": float(np.mean(u_hold)),
            "u_hold_std_raw": float(np.std(u_hold, ddof=1)) if len(u_hold) > 1 else 0.0,
            "firmware_gravity_raw": float(gravity),
            "gravity_residual_raw": float(np.mean(u_hold) - gravity),
            "voltage_ref_raw": float(np.mean(voltage_ref)),
            "output_ff_raw": float(np.mean(output_ff)),
            "output_cmd_raw": float(np.mean(output_cmd)),
            "output_cmd_abs_max": float(np.max(np.abs(output_cmd))),
            "output_saturation_ratio": sat_ratio,
            "speed_abs_mean_rad_s": float(np.mean(np.abs(speed))),
            "speed_abs_max_rad_s": float(np.max(np.abs(speed))),
        })

    stage_df = pd.DataFrame(rows).sort_values("seq_index").reset_index(drop=True)
    if stage_df.empty:
        return stage_df

    prev_ref = stage_df["angle_ref_rad"].shift(1)
    delta = stage_df["angle_ref_rad"] - prev_ref
    stage_df["approach_delta_rad"] = delta.fillna(0.0)
    stage_df["approach_sign"] = np.sign(stage_df["approach_delta_rad"]).astype(int)
    stage_df.loc[stage_df["approach_delta_rad"].abs() < 1e-4, "approach_sign"] = 0
    bin_deg = max(float(args.target_bin_deg), 1e-6)
    stage_df["target_bin_deg"] = np.round(stage_df["angle_ref_deg"] / bin_deg) * bin_deg
    return stage_df


def summarize_pairs(stage_df: pd.DataFrame) -> pd.DataFrame:
    accepted = stage_df[(stage_df["accepted"]) & (stage_df["approach_sign"] != 0)].copy()
    rows: list[dict[str, object]] = []

    for target_bin_deg, group in accepted.groupby("target_bin_deg"):
        pos = group[group["approach_sign"] > 0]
        neg = group[group["approach_sign"] < 0]
        if pos.empty or neg.empty:
            continue
        pos_mean = float(pos["u_hold_raw"].mean())
        neg_mean = float(neg["u_hold_raw"].mean())
        pos_residual = float(pos["gravity_residual_raw"].mean())
        neg_residual = float(neg["gravity_residual_raw"].mean())
        rows.append({
            "target_bin_deg": float(target_bin_deg),
            "pos_count": int(len(pos)),
            "neg_count": int(len(neg)),
            "theta_rad": float(group["theta_rad"].mean()),
            "theta_deg": float(group["theta_deg"].mean()),
            "u_hold_pos_raw": pos_mean,
            "u_hold_neg_raw": neg_mean,
            "u_hold_spread_raw": pos_mean - neg_mean,
            "hysteresis_half_raw": 0.5 * (pos_mean - neg_mean),
            "gravity_residual_pos_raw": pos_residual,
            "gravity_residual_neg_raw": neg_residual,
            "gravity_residual_spread_raw": pos_residual - neg_residual,
            "gravity_residual_half_raw": 0.5 * (pos_residual - neg_residual),
        })

    return pd.DataFrame(rows).sort_values("target_bin_deg").reset_index(drop=True)


def add_fit_columns(stage_df: pd.DataFrame, model: dict[str, object]) -> pd.DataFrame:
    result = stage_df.copy()
    theta = result["theta_rad"].to_numpy()
    approach = result["approach_sign"].to_numpy(dtype=float)
    x = np.column_stack([np.sin(theta), np.ones_like(theta), approach])
    result["pred_hyst_raw"] = x @ model["beta"]
    result["residual_hyst_raw"] = result["u_hold_raw"] - result["pred_hyst_raw"]
    return result


def serializable_model(model: dict[str, object]) -> dict[str, object]:
    return {
        "coefficients": model["coefficients"],
        "n": model["n"],
        "k": model["k"],
        "rmse": model["rmse"],
        "mae": model["mae"],
        "max_abs_error": model["max_abs_error"],
        "r2": model["r2"],
    }


def plot_results(raw_df: pd.DataFrame, stage_df: pd.DataFrame, pair_df: pd.DataFrame,
                 model: dict[str, object], output_dir: Path) -> list[str]:
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
    fig.suptitle("Pitch hysteresis sysid time series")
    fig.tight_layout()
    path = output_dir / "pitch_hysteresis_time_series.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    accepted = stage_df[(stage_df["accepted"]) & (stage_df["approach_sign"] != 0)]
    theta_grid = np.linspace(stage_df["theta_rad"].min(), stage_df["theta_rad"].max(), 300)
    fig, ax = plt.subplots(figsize=(10, 6))
    for sign, label, marker in [(1, "approach increasing", "o"), (-1, "approach decreasing", "s")]:
        subset = accepted[accepted["approach_sign"] == sign]
        ax.scatter(np.degrees(subset["theta_rad"]), subset["u_hold_raw"], label=label, marker=marker, s=60)
        x_grid = np.column_stack([np.sin(theta_grid), np.ones_like(theta_grid), np.full_like(theta_grid, sign)])
        ax.plot(np.degrees(theta_grid), x_grid @ model["beta"], linewidth=1.8)
    text = f"R2={model['r2']:.4f}, RMSE={model['rmse']:.1f}, H={model['coefficients']['approach_sign']['value']:.1f}"
    ax.text(0.02, 0.98, text, transform=ax.transAxes, va="top", bbox={"facecolor": "white", "alpha": 0.85})
    ax.set_xlabel("Pitch angle theta (deg)")
    ax.set_ylabel("u_hold raw")
    ax.set_title("Pitch hysteresis fit")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = output_dir / "pitch_hysteresis_fit.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    if not pair_df.empty:
        fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        axes[0].bar(pair_df["target_bin_deg"], pair_df["u_hold_spread_raw"], width=3.0)
        axes[0].axhline(0.0, color="black", linewidth=0.8)
        axes[0].set_ylabel("u_hold spread raw")
        axes[0].set_title("Same-target approach-direction spread")
        axes[0].grid(True, alpha=0.3)
        axes[1].bar(pair_df["target_bin_deg"], pair_df["gravity_residual_half_raw"], width=3.0)
        axes[1].axhline(0.0, color="black", linewidth=0.8)
        axes[1].set_xlabel("Target angle bin (deg)")
        axes[1].set_ylabel("Residual half spread raw")
        axes[1].grid(True, alpha=0.3)
        fig.tight_layout()
        path = output_dir / "pitch_hysteresis_pair_spread.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths.append(str(path))

    return paths


def main() -> int:
    args = parse_args()
    output_dir = analysis_dir_for(args.csv, args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    df = load_numeric_csv(args.csv)
    raw_df = df[df["mode"].astype(int) == PITCH_HYST_MODE].copy()
    step_df = raw_df[raw_df["phase"].astype(int) == STEP_PHASE].copy()
    if step_df.empty:
        print("not enough pitch hysteresis stages found")
        return 2

    stage_df = summarize_stages(step_df, args)
    if stage_df.empty:
        print("no usable pitch hysteresis stages found")
        return 2

    accepted_df = stage_df[(stage_df["accepted"]) & (stage_df["approach_sign"] != 0)].copy()
    if len(accepted_df) < 6:
        print("not enough accepted approach stages; check pitch_hysteresis_stage_summary.csv")
        stage_df.to_csv(output_dir / "pitch_hysteresis_stage_summary.csv", index=False)
        return 2

    x = np.column_stack([
        np.sin(accepted_df["theta_rad"].to_numpy()),
        np.ones(len(accepted_df)),
        accepted_df["approach_sign"].to_numpy(dtype=float),
    ])
    y = accepted_df["u_hold_raw"].to_numpy()
    model = fit_linear_model(x, y, ["sin(theta)", "offset", "approach_sign"])
    stage_df = add_fit_columns(stage_df, model)
    pair_df = summarize_pairs(stage_df)

    warnings: list[str] = []
    h = model["coefficients"]["approach_sign"]
    if abs(h["value"]) < 150.0:
        warnings.append("approach-direction term is small; software hysteresis compensation may not be worth adding")
    if h["p_value"] > 0.05:
        warnings.append("approach-direction term is not statistically significant")
    if not pair_df.empty and float(pair_df["hysteresis_half_raw"].std(ddof=0)) > max(abs(h["value"]) * 0.6, 150.0):
        warnings.append("hysteresis varies strongly by angle; a single constant H may be too crude")

    stage_csv = output_dir / "pitch_hysteresis_stage_summary.csv"
    pair_csv = output_dir / "pitch_hysteresis_pair_summary.csv"
    result_json = output_dir / "pitch_hysteresis_result.json"
    stage_df.to_csv(stage_csv, index=False)
    pair_df.to_csv(pair_csv, index=False)

    plot_paths: list[str] = []
    if not args.no_plots:
        plot_paths = plot_results(raw_df, stage_df, pair_df, model, output_dir)

    result = {
        "input_csv": str(args.csv),
        "stage_summary_csv": str(stage_csv),
        "pair_summary_csv": str(pair_csv),
        "plot_paths": plot_paths,
        "accepted_approach_stages": int(len(accepted_df)),
        "paired_targets": int(len(pair_df)),
        "filters": {
            "tail_s": args.tail,
            "output_sign": args.output_sign,
            "output_limit": args.output_limit,
            "gravity_sin": args.gravity_sin,
            "gravity_offset": args.gravity_offset,
            "max_stable_speed_rad_s": args.max_stable_speed,
            "max_tracking_error_rad": args.max_tracking_error,
            "max_saturation_ratio": args.max_saturation_ratio,
            "target_bin_deg": args.target_bin_deg,
        },
        "model": serializable_model(model),
        "firmware_candidate": {
            "gravity_sin_raw": model["coefficients"]["sin(theta)"]["value"],
            "gravity_offset_raw": model["coefficients"]["offset"]["value"],
            "hysteresis_raw": model["coefficients"]["approach_sign"]["value"],
        },
        "warnings": warnings,
    }
    result_json.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"output_dir={output_dir}")
    print(f"stages={len(stage_df)}, accepted_approach_stages={len(accepted_df)}, paired_targets={len(pair_df)}")
    print("")
    print("fit: u_hold = A*sin(theta) + C + H*approach_sign")
    print(f"  A = {model['coefficients']['sin(theta)']['value']:+.6f}")
    print(f"  C = {model['coefficients']['offset']['value']:+.6f}")
    print(f"  H = {model['coefficients']['approach_sign']['value']:+.6f}")
    print(f"  R2 = {model['r2']:.5f}, RMSE = {model['rmse']:.3f}")
    if not pair_df.empty:
        print(f"paired half-spread median = {pair_df['hysteresis_half_raw'].median():+.3f} raw")
        print(f"paired half-spread max_abs = {pair_df['hysteresis_half_raw'].abs().max():.3f} raw")
    if warnings:
        print("")
        for warning in warnings:
            print(f"warning: {warning}")
    print(f"json={result_json}")
    print(f"stage_csv={stage_csv}")
    print(f"pair_csv={pair_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
