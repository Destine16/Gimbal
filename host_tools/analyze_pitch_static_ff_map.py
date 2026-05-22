#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


PITCH_STATIC_FF_MAP_MODE = 13
STATIC_MAP_PHASE = 9
DEFAULT_OUTPUT_SIGN = -1.0
DEFAULT_OUTPUT_LIMIT_RAW = 5000.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit pitch static feedforward map from bidirectional hold data."
    )
    parser.add_argument("csv", type=Path, help="CSV captured by gimbal_sysid_rtt_capture.py")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--tail", type=float, default=0.9, help="Stable tail duration per stage in seconds")
    parser.add_argument("--output-sign", type=float, default=DEFAULT_OUTPUT_SIGN)
    parser.add_argument("--output-limit", type=float, default=DEFAULT_OUTPUT_LIMIT_RAW)
    parser.add_argument("--sat-margin", type=float, default=0.98)
    parser.add_argument("--target-bin-deg", type=float, default=0.5)
    parser.add_argument("--max-fit-target-abs-deg", type=float, default=36.0,
                        help="Exclude anchor points outside this absolute target angle from the fit")
    parser.add_argument("--max-stable-speed", type=float, default=0.05)
    parser.add_argument("--max-tracking-error-deg", type=float, default=2.0)
    parser.add_argument("--max-saturation-ratio", type=float, default=0.02)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--no-plots", action="store_true")
    return parser.parse_args()


def output_dir_for(csv_path: Path, output_dir: Path | None) -> Path:
    if output_dir is not None:
        return output_dir
    return csv_path.parent / "analysis" / f"{csv_path.stem}_pitch_static_ff_map"


def load_numeric_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    for column in df.columns:
        df[column] = pd.to_numeric(df[column], errors="coerce")
    return df.dropna(subset=["tick_ms", "mode", "phase"])


def linear_fit(x: np.ndarray, y: np.ndarray, names: list[str]) -> dict[str, Any]:
    beta, *_ = np.linalg.lstsq(x, y, rcond=None)
    pred = x @ beta
    residual = y - pred
    n = int(y.size)
    k = int(x.shape[1])
    rss = float(np.sum(residual * residual))
    tss = float(np.sum((y - float(np.mean(y))) ** 2))
    rmse = float(math.sqrt(np.mean(residual * residual)))
    mae = float(np.mean(np.abs(residual)))
    r2 = float("nan") if tss <= 1e-12 else float(1.0 - rss / tss)
    bic = float(n * math.log(max(rss / max(n, 1), 1e-12)) + k * math.log(max(n, 2)))
    return {
        "names": names,
        "beta": beta,
        "prediction": pred,
        "residual": residual,
        "n": n,
        "k": k,
        "rmse_raw": rmse,
        "mae_raw": mae,
        "max_abs_error_raw": float(np.max(np.abs(residual))),
        "r2": r2,
        "bic": bic,
        "coefficients": {name: float(beta[idx]) for idx, name in enumerate(names)},
    }


def summarize_stages(df: pd.DataFrame, args: argparse.Namespace) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    sat_threshold = args.output_limit * args.sat_margin
    active = df[(df["mode"] == PITCH_STATIC_FF_MAP_MODE) & (df["phase"] == STATIC_MAP_PHASE)].copy()
    if active.empty:
        raise SystemExit("no pitch static feedforward map samples found")

    for seq_index, group in active.groupby("seq_index"):
        stage = group.sort_values("tick_ms")
        if stage.empty:
            continue
        max_tick = float(stage["tick_ms"].iloc[-1])
        stable = stage[stage["tick_ms"] >= max_tick - args.tail * 1000.0]
        if stable.empty:
            continue

        angle_ref = stable["angle_ref_rad"].to_numpy(dtype=float)
        angle_feedback = stable["angle_feedback_rad"].to_numpy(dtype=float)
        speed_feedback = stable["speed_feedback_rad_s"].to_numpy(dtype=float)
        voltage_ref = stable["voltage_ref_raw"].to_numpy(dtype=float)
        output_ff = stable["output_ff_raw"].to_numpy(dtype=float)
        output_cmd = stable["output_cmd"].to_numpy(dtype=float)
        error = angle_ref - angle_feedback
        u_hold = output_ff + args.output_sign * voltage_ref
        sat_ratio = float(np.mean(np.abs(output_cmd) >= sat_threshold))

        reject_reasons: list[str] = []
        if stable.shape[0] < args.min_samples:
            reject_reasons.append("too_few_samples")
        if float(np.mean(np.abs(speed_feedback))) > args.max_stable_speed:
            reject_reasons.append("not_static")
        if math.degrees(float(np.mean(np.abs(error)))) > args.max_tracking_error_deg:
            reject_reasons.append("tracking_error")
        if sat_ratio > args.max_saturation_ratio:
            reject_reasons.append("output_saturated")

        theta = float(np.mean(angle_feedback))
        target = float(np.mean(angle_ref))
        rows.append({
            "seq_index": int(seq_index),
            "accepted": len(reject_reasons) == 0,
            "reject_reason": ",".join(reject_reasons),
            "sample_count": int(stable.shape[0]),
            "stage_duration_s": float((stage["tick_ms"].iloc[-1] - stage["tick_ms"].iloc[0]) / 1000.0),
            "theta_rad": theta,
            "theta_deg": math.degrees(theta),
            "target_rad": target,
            "target_deg": math.degrees(target),
            "tracking_error_mean_deg": math.degrees(float(np.mean(error))),
            "tracking_error_abs_mean_deg": math.degrees(float(np.mean(np.abs(error)))),
            "speed_abs_mean_rad_s": float(np.mean(np.abs(speed_feedback))),
            "u_hold_raw": float(np.mean(u_hold)),
            "u_hold_std_raw": float(np.std(u_hold, ddof=1)) if stable.shape[0] > 1 else 0.0,
            "voltage_ref_raw": float(np.mean(voltage_ref)),
            "output_ff_raw": float(np.mean(output_ff)),
            "output_cmd_raw": float(np.mean(output_cmd)),
            "output_cmd_abs_max": float(np.max(np.abs(output_cmd))),
            "output_saturation_ratio": sat_ratio,
        })

    stage_df = pd.DataFrame(rows).sort_values("seq_index").reset_index(drop=True)
    if stage_df.empty:
        raise SystemExit("no usable stages found")

    previous_target = stage_df["target_rad"].shift(1)
    delta = (stage_df["target_rad"] - previous_target).fillna(0.0)
    stage_df["approach_delta_rad"] = delta
    stage_df["approach_sign"] = np.sign(delta).astype(int)
    stage_df.loc[stage_df["approach_delta_rad"].abs() < 1e-4, "approach_sign"] = 0
    bin_deg = max(float(args.target_bin_deg), 1e-6)
    stage_df["target_bin_deg"] = np.round(stage_df["target_deg"] / bin_deg) * bin_deg
    stage_df["fit_enabled"] = (
        stage_df["accepted"] &
        (stage_df["approach_sign"] != 0) &
        (stage_df["target_deg"].abs() <= args.max_fit_target_abs_deg)
    )
    return stage_df


def summarize_targets(stage_df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    selected = stage_df[stage_df["fit_enabled"]].copy()
    for target_bin_deg, group in selected.groupby("target_bin_deg"):
        up = group[group["approach_sign"] > 0]
        down = group[group["approach_sign"] < 0]
        if up.empty or down.empty:
            direction_spread = float("nan")
            hyst_half = float("nan")
        else:
            direction_spread = float(up["u_hold_raw"].mean() - down["u_hold_raw"].mean())
            hyst_half = 0.5 * direction_spread
        rows.append({
            "target_bin_deg": float(target_bin_deg),
            "stage_count": int(group.shape[0]),
            "up_count": int(up.shape[0]),
            "down_count": int(down.shape[0]),
            "theta_deg": float(group["theta_deg"].mean()),
            "u_hold_mean_raw": float(group["u_hold_raw"].mean()),
            "u_hold_std_raw": float(group["u_hold_raw"].std(ddof=1)) if group.shape[0] > 1 else 0.0,
            "u_hold_up_raw": float(up["u_hold_raw"].mean()) if not up.empty else float("nan"),
            "u_hold_down_raw": float(down["u_hold_raw"].mean()) if not down.empty else float("nan"),
            "direction_spread_raw": direction_spread,
            "hysteresis_half_raw": hyst_half,
            "tracking_error_abs_mean_deg": float(group["tracking_error_abs_mean_deg"].mean()),
        })
    if not rows:
        return pd.DataFrame()
    return pd.DataFrame(rows).sort_values("target_bin_deg").reset_index(drop=True)


def fit_models(stage_df: pd.DataFrame) -> dict[str, Any]:
    selected = stage_df[stage_df["fit_enabled"]].copy()
    if selected.shape[0] < 8:
        raise SystemExit(f"not enough fit samples: {selected.shape[0]}")

    theta = selected["theta_rad"].to_numpy(dtype=float)
    sign = selected["approach_sign"].to_numpy(dtype=float)
    y = selected["u_hold_raw"].to_numpy(dtype=float)

    gravity2 = linear_fit(
        np.column_stack([np.sin(theta), np.ones_like(theta)]),
        y,
        ["sin(theta)", "offset"],
    )
    gravity3 = linear_fit(
        np.column_stack([np.sin(theta), np.cos(theta), np.ones_like(theta)]),
        y,
        ["sin(theta)", "cos(theta)", "offset"],
    )
    gravity_hyst = linear_fit(
        np.column_stack([np.sin(theta), np.cos(theta), np.ones_like(theta), sign]),
        y,
        ["sin(theta)", "cos(theta)", "offset", "approach_sign"],
    )
    return {
        "fit_samples": int(selected.shape[0]),
        "target_count": int(selected["target_bin_deg"].nunique()),
        "gravity_sin_offset": gravity2,
        "gravity_sin_cos_offset": gravity3,
        "gravity_sin_cos_offset_hyst": gravity_hyst,
    }


def serializable_model(model: dict[str, Any]) -> dict[str, Any]:
    return {
        "n": model["n"],
        "k": model["k"],
        "coefficients": model["coefficients"],
        "rmse_raw": model["rmse_raw"],
        "mae_raw": model["mae_raw"],
        "max_abs_error_raw": model["max_abs_error_raw"],
        "r2": model["r2"],
        "bic": model["bic"],
    }


def build_result(stage_df: pd.DataFrame, target_df: pd.DataFrame,
                 models: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    firmware_model = models["gravity_sin_offset"]
    firmware_coefficients = firmware_model["coefficients"]
    diagnostic_model = models["gravity_sin_cos_offset_hyst"]
    diagnostic_coefficients = diagnostic_model["coefficients"]
    warnings: list[str] = []
    accepted = stage_df[stage_df["accepted"]]
    rejected = stage_df[~stage_df["accepted"]]

    if accepted.shape[0] < stage_df.shape[0] * 0.7:
        warnings.append("many stages were rejected; inspect tracking and saturation before trusting coefficients")
    if not target_df.empty:
        paired = target_df.dropna(subset=["direction_spread_raw"])
        if not paired.empty:
            max_spread = float(paired["direction_spread_raw"].abs().max())
            median_spread = float(paired["direction_spread_raw"].abs().median())
            if max_spread > 600.0:
                warnings.append(f"large pitch direction hysteresis detected; max spread={max_spread:.1f} raw")
            if median_spread > 250.0:
                warnings.append(f"visible pitch direction hysteresis; median spread={median_spread:.1f} raw")

    return {
        "source_csv": str(args.csv),
        "mode": PITCH_STATIC_FF_MAP_MODE,
        "phase": STATIC_MAP_PHASE,
        "stage_count": int(stage_df.shape[0]),
        "accepted_stage_count": int(accepted.shape[0]),
        "rejected_stage_count": int(rejected.shape[0]),
        "fit_samples": models["fit_samples"],
        "fit_target_count": models["target_count"],
        "selection": {
            "tail_s": args.tail,
            "max_fit_target_abs_deg": args.max_fit_target_abs_deg,
            "max_stable_speed_rad_s": args.max_stable_speed,
            "max_tracking_error_deg": args.max_tracking_error_deg,
            "max_saturation_ratio": args.max_saturation_ratio,
        },
        "models": {
            "gravity_sin_offset": serializable_model(models["gravity_sin_offset"]),
            "gravity_sin_cos_offset": serializable_model(models["gravity_sin_cos_offset"]),
            "gravity_sin_cos_offset_hyst": serializable_model(models["gravity_sin_cos_offset_hyst"]),
        },
        "firmware_compatible_macros": {
            "GIMBAL_PITCH_OUTPUT_FF_SIN_RAW": firmware_coefficients["sin(theta)"],
            "GIMBAL_PITCH_OUTPUT_FF_OFFSET_RAW": firmware_coefficients["offset"],
            "GIMBAL_PITCH_OUTPUT_FF_HYST_RAW": 0.0,
            "GIMBAL_PITCH_OUTPUT_HYST_ENABLE": 0,
        },
        "diagnostic_coefficients": {
            "GIMBAL_PITCH_OUTPUT_FF_COS_RAW": diagnostic_coefficients["cos(theta)"],
            "approach_direction_hyst_raw": diagnostic_coefficients["approach_sign"],
        },
        "warnings": warnings,
    }


def write_macro_md(result: dict[str, Any], path: Path) -> None:
    macros = result["firmware_compatible_macros"]
    diag = result["diagnostic_coefficients"]
    lines = [
        "# Pitch Static Feedforward Map Result",
        "",
        "Firmware-compatible values:",
        "",
        "```c",
        f"#define GIMBAL_PITCH_OUTPUT_FF_SIN_RAW       ({macros['GIMBAL_PITCH_OUTPUT_FF_SIN_RAW']:.4f}f)",
        f"#define GIMBAL_PITCH_OUTPUT_FF_OFFSET_RAW    ({macros['GIMBAL_PITCH_OUTPUT_FF_OFFSET_RAW']:.4f}f)",
        f"#define GIMBAL_PITCH_OUTPUT_FF_HYST_RAW      ({macros['GIMBAL_PITCH_OUTPUT_FF_HYST_RAW']:.4f}f)",
        f"#define GIMBAL_PITCH_OUTPUT_HYST_ENABLE     {macros['GIMBAL_PITCH_OUTPUT_HYST_ENABLE']}",
        "```",
        "",
        "Diagnostic-only coefficients from the full static map model:",
        "",
        f"- cos(theta): {diag['GIMBAL_PITCH_OUTPUT_FF_COS_RAW']:.4f} raw",
        f"- approach-direction hysteresis: {diag['approach_direction_hyst_raw']:.4f} raw",
        "",
        "The current firmware does not directly implement cos(theta) or approach-direction hold hysteresis.",
        "Use these values to decide whether a new firmware feature is needed; do not paste them as-is.",
        "",
        "Model metrics:",
        "",
        "| Model | RMSE raw | MAE raw | Max error raw | R2 |",
        "|---|---:|---:|---:|---:|",
    ]
    for key, model in result["models"].items():
        lines.append(
            f"| {key} | {model['rmse_raw']:.2f} | {model['mae_raw']:.2f} | "
            f"{model['max_abs_error_raw']:.2f} | {model['r2']:.4f} |"
        )
    if result["warnings"]:
        lines.extend(["", "Warnings:"])
        lines.extend(f"- {item}" for item in result["warnings"])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_results(raw_df: pd.DataFrame, stage_df: pd.DataFrame, target_df: pd.DataFrame,
                 models: dict[str, Any], output_dir: Path) -> list[str]:
    paths: list[str] = []
    active = raw_df[(raw_df["mode"] == PITCH_STATIC_FF_MAP_MODE)].copy()
    t_s = (active["tick_ms"] - active["tick_ms"].iloc[0]) / 1000.0

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t_s, np.degrees(active["angle_ref_rad"]), label="target", linewidth=0.9)
    axes[0].plot(t_s, np.degrees(active["angle_feedback_rad"]), label="actual", linewidth=0.9)
    axes[0].set_ylabel("Pitch angle (deg)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t_s, active["speed_feedback_rad_s"], linewidth=0.9)
    axes[1].set_ylabel("Speed (rad/s)")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(t_s, active["output_cmd"], label="output_cmd", linewidth=0.9)
    axes[2].plot(t_s, active["voltage_ref_raw"], label="voltage_ref", linewidth=0.9)
    axes[2].set_ylabel("Raw output")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)
    fig.tight_layout()
    path = output_dir / "pitch_static_ff_map_time_series.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    selected = stage_df[stage_df["fit_enabled"]].copy()
    theta = selected["theta_rad"].to_numpy(dtype=float)
    sign = selected["approach_sign"].to_numpy(dtype=float)
    x = np.column_stack([np.sin(theta), np.cos(theta), np.ones_like(theta), sign])
    selected["pred_raw"] = x @ models["gravity_sin_cos_offset_hyst"]["beta"]
    selected["residual_raw"] = selected["u_hold_raw"] - selected["pred_raw"]

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    up = selected[selected["approach_sign"] > 0]
    down = selected[selected["approach_sign"] < 0]
    axes[0].scatter(up["theta_deg"], up["u_hold_raw"], label="up approach", marker="o")
    axes[0].scatter(down["theta_deg"], down["u_hold_raw"], label="down approach", marker="s")
    axes[0].scatter(selected["theta_deg"], selected["pred_raw"], label="model", marker="x")
    axes[0].set_ylabel("Required FF raw")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].scatter(selected["theta_deg"], selected["residual_raw"], c=selected["approach_sign"], cmap="coolwarm")
    axes[1].axhline(0.0, color="k", linewidth=0.8)
    axes[1].set_xlabel("Pitch angle (deg)")
    axes[1].set_ylabel("Residual raw")
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    path = output_dir / "pitch_static_ff_map_fit.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    if not target_df.empty:
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.bar(target_df["target_bin_deg"], target_df["direction_spread_raw"].fillna(0.0), width=1.8)
        ax.axhline(0.0, color="k", linewidth=0.8)
        ax.set_xlabel("Target angle bin (deg)")
        ax.set_ylabel("Up - down required FF raw")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        path = output_dir / "pitch_static_ff_map_direction_spread.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths.append(str(path))

    return paths


def main() -> int:
    args = parse_args()
    raw_df = load_numeric_csv(args.csv)
    output_dir = output_dir_for(args.csv, args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    stage_df = summarize_stages(raw_df, args)
    target_df = summarize_targets(stage_df)
    models = fit_models(stage_df)
    result = build_result(stage_df, target_df, models, args)

    stage_csv = output_dir / "pitch_static_ff_map_stage_summary.csv"
    target_csv = output_dir / "pitch_static_ff_map_target_summary.csv"
    result_json = output_dir / "pitch_static_ff_map_result.json"
    macro_md = output_dir / "pitch_static_ff_map_macros.md"

    stage_df.to_csv(stage_csv, index=False)
    target_df.to_csv(target_csv, index=False)
    result_json.write_text(json.dumps(result, indent=2), encoding="utf-8")
    write_macro_md(result, macro_md)
    if not args.no_plots:
        result["plot_paths"] = plot_results(raw_df, stage_df, target_df, models, output_dir)
        result_json.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"wrote {result_json}")
    print(f"wrote {macro_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
