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
from scipy import signal


YAW_PRBS_MODE = 1
PRBS_PHASE = 2
DEFAULT_OUTPUT_LIMIT_RAW = 5000.0
DEFAULT_ANGLE_KP = 12.0
DEFAULT_SPEED_KP = 600.0
DEFAULT_SPEED_KI = 1600.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze yaw PRBS RTT sysid CSV and recommend PID update candidates.")
    parser.add_argument("csv", type=Path, help="CSV captured by gimbal_sysid_rtt_capture.py")
    parser.add_argument("--output-dir", type=Path, default=None, help="Directory for CSV/JSON/plots")
    parser.add_argument("--output-limit", type=float, default=DEFAULT_OUTPUT_LIMIT_RAW, help="Raw output clamp used by firmware")
    parser.add_argument("--sat-margin", type=float, default=0.98, help="Treat abs(output_cmd) above this limit ratio as saturated")
    parser.add_argument("--min-step-deg", type=float, default=8.0, help="Minimum reference step used for step metrics")
    parser.add_argument("--settle-band-ratio", type=float, default=0.02, help="Settling band as step amplitude ratio")
    parser.add_argument("--settle-band-min-deg", type=float, default=0.5, help="Minimum settling band in degrees")
    parser.add_argument("--current-angle-kp", type=float, default=DEFAULT_ANGLE_KP)
    parser.add_argument("--current-speed-kp", type=float, default=DEFAULT_SPEED_KP)
    parser.add_argument("--current-speed-ki", type=float, default=DEFAULT_SPEED_KI)
    parser.add_argument("--target-overshoot", type=float, default=0.08, help="Desired median overshoot ratio")
    parser.add_argument("--min-target-angle-bw", type=float, default=0.6, help="Minimum target angle bandwidth in Hz")
    parser.add_argument("--max-target-angle-bw", type=float, default=2.2, help="Maximum target angle bandwidth in Hz")
    parser.add_argument("--no-plots", action="store_true", help="Skip PNG plot generation")
    return parser.parse_args()


def analysis_dir_for(csv_path: Path, output_dir: Path | None) -> Path:
    if output_dir is not None:
        return output_dir
    return csv_path.parent / "analysis" / f"{csv_path.stem}_yaw_prbs"


def load_numeric_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    for column in df.columns:
        df[column] = pd.to_numeric(df[column], errors="coerce")
    return df.dropna(subset=["tick_ms", "mode", "phase"])


def finite_median(values: pd.Series | np.ndarray) -> float:
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    return float(np.median(arr)) if arr.size else float("nan")


def finite_mean(values: pd.Series | np.ndarray) -> float:
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    return float(np.mean(arr)) if arr.size else float("nan")


def estimate_dt_s(df: pd.DataFrame) -> float:
    tick = df["tick_ms"].to_numpy(dtype=float)
    diff = np.diff(tick)
    diff = diff[(diff > 0.0) & np.isfinite(diff)]
    if diff.size == 0:
        return 0.005
    return float(np.median(diff) / 1000.0)


def uniform_resample(df: pd.DataFrame, columns: list[str]) -> tuple[np.ndarray, dict[str, np.ndarray], float]:
    source = df.sort_values("tick_ms").drop_duplicates(subset=["tick_ms"], keep="last")
    t = (source["tick_ms"].to_numpy(dtype=float) - float(source["tick_ms"].iloc[0])) / 1000.0
    dt_s = estimate_dt_s(source)
    if len(t) < 2:
        return t, {col: source[col].to_numpy(dtype=float) for col in columns}, dt_s
    t_grid = np.arange(t[0], t[-1] + 0.5 * dt_s, dt_s)
    values = {}
    for col in columns:
        values[col] = np.interp(t_grid, t, source[col].to_numpy(dtype=float))
    return t_grid, values, dt_s


def response_metrics(ref: np.ndarray, actual: np.ndarray, output_cmd: np.ndarray,
                     output_limit: float, sat_margin: float) -> dict[str, float]:
    error = ref - actual
    sat_threshold = output_limit * sat_margin
    return {
        "tracking_rmse_rad": float(math.sqrt(np.mean(error * error))),
        "tracking_rmse_deg": float(math.degrees(math.sqrt(np.mean(error * error)))),
        "tracking_mae_deg": float(math.degrees(np.mean(np.abs(error)))),
        "tracking_max_abs_deg": float(math.degrees(np.max(np.abs(error)))),
        "output_saturation_ratio": float(np.mean(np.abs(output_cmd) >= sat_threshold)),
        "output_abs_max_raw": float(np.max(np.abs(output_cmd))),
    }


def crossing_time(t: np.ndarray, value: np.ndarray, threshold: float) -> float:
    hits = np.flatnonzero(value >= threshold)
    if hits.size == 0:
        return float("nan")
    idx = int(hits[0])
    if idx == 0:
        return float(t[0])
    x0, x1 = value[idx - 1], value[idx]
    t0, t1 = t[idx - 1], t[idx]
    if abs(x1 - x0) < 1e-12:
        return float(t1)
    alpha = (threshold - x0) / (x1 - x0)
    return float(t0 + alpha * (t1 - t0))


def compute_step_metrics(prbs_df: pd.DataFrame, args: argparse.Namespace) -> pd.DataFrame:
    rows: list[dict[str, float | int]] = []
    groups = [(int(seq), group.sort_values("tick_ms")) for seq, group in prbs_df.groupby("seq_index")]
    groups.sort(key=lambda item: item[0])
    min_step = math.radians(args.min_step_deg)
    settle_min = math.radians(args.settle_band_min_deg)
    sat_threshold = args.output_limit * args.sat_margin
    previous_ref: float | None = None

    for seq_index, stage in groups:
        if len(stage) < 8:
            continue
        ref = stage["angle_ref_rad"].to_numpy(dtype=float)
        actual = stage["angle_feedback_rad"].to_numpy(dtype=float)
        speed = stage["speed_feedback_rad_s"].to_numpy(dtype=float)
        output = stage["output_cmd"].to_numpy(dtype=float)
        t = (stage["tick_ms"].to_numpy(dtype=float) - float(stage["tick_ms"].iloc[0])) / 1000.0
        target = finite_median(ref)
        if previous_ref is None:
            previous_ref = target
            continue

        ref_step = target - previous_ref
        previous_ref = target
        if not math.isfinite(ref_step) or abs(ref_step) < min_step:
            continue

        early_end = min(float(t[-1]), 0.05)
        early = actual[t <= early_end]
        y0 = float(np.mean(early)) if early.size else float(actual[0])
        target_delta = target - y0
        if abs(target_delta) < min_step * 0.5:
            continue

        direction = 1.0 if target_delta >= 0.0 else -1.0
        response = direction * (actual - y0)
        target_response = abs(target_delta)
        peak_response = float(np.max(response))
        overshoot = max(0.0, peak_response - target_response) / max(target_response, 1e-9)
        undershoot = max(0.0, -float(np.min(response))) / max(target_response, 1e-9)

        rise10 = crossing_time(t, response, 0.10 * target_response)
        rise90 = crossing_time(t, response, 0.90 * target_response)
        rise_time = rise90 - rise10 if math.isfinite(rise10) and math.isfinite(rise90) else float("nan")

        tail_start = max(float(t[0]), float(t[-1]) - min(0.2, max(0.05, 0.25 * float(t[-1]))))
        tail_mask = t >= tail_start
        y_tail = actual[tail_mask]
        steady_actual = float(np.mean(y_tail)) if y_tail.size else float(actual[-1])
        steady_error = target - steady_actual

        settle_band = max(args.settle_band_ratio * target_response, settle_min)
        abs_error = np.abs(actual - target)
        settling_time = float("nan")
        for idx in range(len(t)):
            if np.all(abs_error[idx:] <= settle_band):
                settling_time = float(t[idx])
                break

        rows.append({
            "seq_index": seq_index,
            "duration_s": float(t[-1] - t[0]),
            "ref_step_rad": float(ref_step),
            "ref_step_deg": math.degrees(float(ref_step)),
            "target_rad": float(target),
            "target_deg": math.degrees(float(target)),
            "start_actual_rad": y0,
            "steady_actual_rad": steady_actual,
            "steady_error_rad": float(steady_error),
            "steady_error_deg": math.degrees(float(steady_error)),
            "steady_error_ratio": abs(float(steady_error)) / max(abs(ref_step), 1e-9),
            "rise_time_s": float(rise_time),
            "settling_time_s": float(settling_time),
            "overshoot_ratio": float(overshoot),
            "undershoot_ratio": float(undershoot),
            "peak_speed_rad_s": float(np.max(np.abs(speed))),
            "stage_error_rmse_deg": math.degrees(float(math.sqrt(np.mean((ref - actual) ** 2)))),
            "output_saturation_ratio": float(np.mean(np.abs(output) >= sat_threshold)),
            "output_abs_max_raw": float(np.max(np.abs(output))),
        })

    return pd.DataFrame(rows)


def estimate_frequency_response(u: np.ndarray, y: np.ndarray, dt_s: float, label: str) -> dict[str, object]:
    u = np.asarray(u, dtype=float)
    y = np.asarray(y, dtype=float)
    mask = np.isfinite(u) & np.isfinite(y)
    u = u[mask]
    y = y[mask]
    if len(u) < 128:
        return {"label": label, "ok": False, "reason": "not_enough_samples"}

    u = signal.detrend(u - np.mean(u), type="constant")
    y = signal.detrend(y - np.mean(y), type="constant")
    fs = 1.0 / dt_s
    nperseg = min(4096, max(128, 2 ** int(math.floor(math.log2(max(len(u) // 4, 128))))))
    nperseg = min(nperseg, len(u))
    if nperseg < 64:
        return {"label": label, "ok": False, "reason": "not_enough_segment_length"}

    freq, pxx = signal.welch(u, fs=fs, nperseg=nperseg)
    _, pxy = signal.csd(u, y, fs=fs, nperseg=nperseg)
    _, coh = signal.coherence(u, y, fs=fs, nperseg=nperseg)
    h = np.divide(pxy, pxx, out=np.zeros_like(pxy), where=pxx > 1e-18)
    mag = np.abs(h)
    phase = np.unwrap(np.angle(h))
    input_power_threshold = float(np.max(pxx) * 1e-4) if len(pxx) else 0.0
    valid = (freq > 0.02) & (pxx > input_power_threshold) & np.isfinite(mag)
    coherent = valid & (coh > 0.45)

    low_band = coherent & (freq >= 0.05) & (freq <= 0.5)
    if np.any(low_band):
        low_gain = float(np.median(mag[low_band]))
    elif np.any(coherent):
        low_gain = float(np.median(mag[coherent][: min(5, np.count_nonzero(coherent))]))
    else:
        low_gain = float("nan")

    bandwidth_hz = float("nan")
    if math.isfinite(low_gain) and low_gain > 1e-9:
        below = np.flatnonzero(coherent & (mag <= low_gain / math.sqrt(2.0)))
        if below.size:
            bandwidth_hz = float(freq[int(below[0])])

    passband = valid & (freq >= 0.05) & (freq <= 3.0)
    mean_coherence = float(np.mean(coh[passband])) if np.any(passband) else float("nan")
    peak_gain = float(np.max(mag[coherent])) if np.any(coherent) else float("nan")
    peak_gain_freq_hz = float(freq[np.argmax(np.where(coherent, mag, -np.inf))]) if np.any(coherent) else float("nan")

    return {
        "label": label,
        "ok": True,
        "freq_hz": freq,
        "magnitude": mag,
        "phase_rad": phase,
        "coherence": coh,
        "low_frequency_gain": low_gain,
        "bandwidth_hz": bandwidth_hz,
        "mean_coherence_0p05_3hz": mean_coherence,
        "peak_gain": peak_gain,
        "peak_gain_freq_hz": peak_gain_freq_hz,
    }


def fit_arx(u: np.ndarray, y: np.ndarray, na: int = 4, nb: int = 4, nk: int = 1) -> dict[str, object]:
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
    phi = []
    target = []
    for k in range(max_lag, len(yy)):
        row = [yy[k - i] for i in range(1, na + 1)]
        row += [uu[k - nk - j] for j in range(nb)]
        row += [1.0]
        phi.append(row)
        target.append(yy[k])
    x = np.asarray(phi, dtype=float)
    z = np.asarray(target, dtype=float)
    theta, *_ = np.linalg.lstsq(x, z, rcond=None)
    one_step = x @ theta
    residual = z - one_step

    sim = np.zeros_like(yy)
    sim[:max_lag] = yy[:max_lag]
    for k in range(max_lag, len(yy)):
        row = [sim[k - i] for i in range(1, na + 1)]
        row += [uu[k - nk - j] for j in range(nb)]
        row += [1.0]
        sim[k] = float(np.dot(row, theta))

    y_valid = yy[max_lag:]
    sim_valid = sim[max_lag:]
    sst = float(np.sum((y_valid - np.mean(y_valid)) ** 2))
    sse_one = float(np.sum(residual * residual))
    sse_sim = float(np.sum((y_valid - sim_valid) ** 2))
    return {
        "ok": True,
        "na": na,
        "nb": nb,
        "nk": nk,
        "coefficients": [float(v) for v in theta],
        "one_step_rmse": float(math.sqrt(np.mean(residual * residual))),
        "one_step_r2": float(1.0 - sse_one / sst) if sst > 1e-12 else float("nan"),
        "simulation_rmse": float(math.sqrt(np.mean((y_valid - sim_valid) ** 2))),
        "simulation_r2": float(1.0 - sse_sim / sst) if sst > 1e-12 else float("nan"),
        "max_lag": int(max_lag),
        "simulation": sim + y_mean,
        "u_mean": u_mean,
        "y_mean": y_mean,
    }


def zeta_from_overshoot(overshoot: float) -> float:
    if not math.isfinite(overshoot) or overshoot <= 1e-4:
        return 1.0
    overshoot = min(max(overshoot, 1e-4), 0.95)
    log_os = math.log(overshoot)
    return float(-log_os / math.sqrt(math.pi * math.pi + log_os * log_os))


def build_pid_recommendation(args: argparse.Namespace, step_df: pd.DataFrame,
                             angle_freq: dict[str, object], speed_freq: dict[str, object],
                             overall: dict[str, float]) -> dict[str, object]:
    median_overshoot = finite_median(step_df["overshoot_ratio"]) if not step_df.empty else float("nan")
    median_rise = finite_median(step_df["rise_time_s"]) if not step_df.empty else float("nan")
    median_settle = finite_median(step_df["settling_time_s"]) if not step_df.empty else float("nan")
    median_steady_ratio = finite_median(step_df["steady_error_ratio"]) if not step_df.empty else float("nan")
    zeta = zeta_from_overshoot(median_overshoot)
    angle_bw = float(angle_freq.get("bandwidth_hz", float("nan"))) if angle_freq.get("ok") else float("nan")
    speed_bw = float(speed_freq.get("bandwidth_hz", float("nan"))) if speed_freq.get("ok") else float("nan")

    reasons: list[str] = []
    if math.isfinite(speed_bw):
        target_angle_bw = min(args.max_target_angle_bw, max(args.min_target_angle_bw, 0.30 * speed_bw))
        reasons.append(f"angle bandwidth target limited to 30% of measured speed-loop bandwidth ({speed_bw:.3f} Hz)")
    elif math.isfinite(angle_bw):
        target_angle_bw = min(args.max_target_angle_bw, max(args.min_target_angle_bw, 1.15 * angle_bw))
        reasons.append(f"angle bandwidth target based on measured closed-loop bandwidth ({angle_bw:.3f} Hz)")
    elif math.isfinite(median_rise) and median_rise > 1e-3:
        target_angle_bw = min(args.max_target_angle_bw, max(args.min_target_angle_bw, 0.35 / median_rise))
        reasons.append("angle bandwidth target estimated from median rise time")
    else:
        target_angle_bw = args.current_angle_kp / (2.0 * math.pi)
        reasons.append("insufficient bandwidth estimate; keeping current angle bandwidth")

    angle_kp_candidate = 2.0 * math.pi * target_angle_bw
    angle_kp_candidate = min(max(angle_kp_candidate, 0.60 * args.current_angle_kp), 1.40 * args.current_angle_kp)
    if math.isfinite(median_overshoot) and median_overshoot > max(0.15, 1.5 * args.target_overshoot):
        angle_kp_candidate = min(angle_kp_candidate, 0.90 * args.current_angle_kp)
        reasons.append("overshoot is high; angle Kp candidate is capped below current value")
    elif math.isfinite(median_overshoot) and median_overshoot < 0.5 * args.target_overshoot and overall["output_saturation_ratio"] < 0.03:
        angle_kp_candidate = max(angle_kp_candidate, 1.05 * args.current_angle_kp)
        reasons.append("overshoot and saturation are low; angle Kp can be tested slightly higher")

    speed_kp_candidate = args.current_speed_kp
    if math.isfinite(speed_bw) and speed_bw < 3.0 * target_angle_bw and overall["output_saturation_ratio"] < 0.05:
        speed_kp_candidate *= 1.15
        reasons.append("speed-loop bandwidth is close to angle target; speed Kp candidate increased")
    if math.isfinite(median_overshoot) and median_overshoot > 0.15 and overall["output_saturation_ratio"] < 0.10:
        speed_kp_candidate *= 1.08
        reasons.append("angle overshoot suggests insufficient damping; speed Kp candidate increased")
    if overall["output_saturation_ratio"] > 0.10:
        speed_kp_candidate *= 0.90
        angle_kp_candidate = min(angle_kp_candidate, args.current_angle_kp)
        reasons.append("output saturation is high; avoid increasing aggressiveness")

    speed_ki_candidate = args.current_speed_ki
    if math.isfinite(median_steady_ratio) and median_steady_ratio > 0.05 and overall["output_saturation_ratio"] < 0.05:
        speed_ki_candidate *= 1.12
        reasons.append("steady error ratio is high; speed Ki candidate increased")
    if math.isfinite(median_overshoot) and median_overshoot > 0.18:
        speed_ki_candidate *= 0.88
        reasons.append("overshoot is high; speed Ki candidate reduced")

    return {
        "method": "bandwidth-and-step-metric constrained candidate, not automatic firmware patch",
        "current": {
            "angle_kp": args.current_angle_kp,
            "speed_kp": args.current_speed_kp,
            "speed_ki": args.current_speed_ki,
        },
        "candidate": {
            "angle_kp": float(angle_kp_candidate),
            "speed_kp": float(speed_kp_candidate),
            "speed_ki": float(speed_ki_candidate),
        },
        "estimated": {
            "median_overshoot_ratio": float(median_overshoot),
            "estimated_zeta": float(zeta),
            "median_rise_time_s": float(median_rise),
            "median_settling_time_s": float(median_settle),
            "median_steady_error_ratio": float(median_steady_ratio),
            "angle_bandwidth_hz": float(angle_bw),
            "speed_bandwidth_hz": float(speed_bw),
            "target_angle_bandwidth_hz": float(target_angle_bw),
        },
        "reasons": reasons,
    }


def scalar_frequency_metrics(freq: dict[str, object]) -> dict[str, float | str | bool]:
    if not freq.get("ok"):
        return {"ok": False, "reason": str(freq.get("reason", "unknown"))}
    return {
        "ok": True,
        "low_frequency_gain": float(freq["low_frequency_gain"]),
        "bandwidth_hz": float(freq["bandwidth_hz"]),
        "mean_coherence_0p05_3hz": float(freq["mean_coherence_0p05_3hz"]),
        "peak_gain": float(freq["peak_gain"]),
        "peak_gain_freq_hz": float(freq["peak_gain_freq_hz"]),
    }


def save_frequency_csv(path: Path, angle_freq: dict[str, object], speed_freq: dict[str, object]) -> None:
    if not angle_freq.get("ok"):
        return
    df = pd.DataFrame({
        "freq_hz": angle_freq["freq_hz"],
        "angle_mag": angle_freq["magnitude"],
        "angle_mag_db": 20.0 * np.log10(np.maximum(angle_freq["magnitude"], 1e-12)),
        "angle_phase_deg": np.degrees(angle_freq["phase_rad"]),
        "angle_coherence": angle_freq["coherence"],
    })
    if speed_freq.get("ok"):
        speed_df = pd.DataFrame({
            "freq_hz": speed_freq["freq_hz"],
            "speed_mag": speed_freq["magnitude"],
            "speed_mag_db": 20.0 * np.log10(np.maximum(speed_freq["magnitude"], 1e-12)),
            "speed_phase_deg": np.degrees(speed_freq["phase_rad"]),
            "speed_coherence": speed_freq["coherence"],
        })
        df = pd.merge_asof(df.sort_values("freq_hz"), speed_df.sort_values("freq_hz"), on="freq_hz", direction="nearest")
    df.to_csv(path, index=False)


def plot_results(mode_df: pd.DataFrame, prbs_df: pd.DataFrame, step_df: pd.DataFrame,
                 angle_freq: dict[str, object], speed_freq: dict[str, object],
                 angle_arx: dict[str, object], t_grid: np.ndarray, values: dict[str, np.ndarray],
                 result: dict[str, object], output_dir: Path) -> list[str]:
    paths: list[str] = []
    t = (mode_df["tick_ms"] - mode_df["tick_ms"].iloc[0]) / 1000.0
    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    axes[0].plot(t, np.degrees(mode_df["angle_ref_rad"]), label="target", linewidth=0.9)
    axes[0].plot(t, np.degrees(mode_df["angle_feedback_rad"]), label="actual", linewidth=0.9)
    axes[0].set_ylabel("Yaw angle (deg)")
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
    fig.suptitle("Yaw PRBS time series")
    fig.tight_layout()
    path = output_dir / "yaw_prbs_time_series.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))

    if angle_freq.get("ok"):
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        axes[0].semilogx(angle_freq["freq_hz"], 20.0 * np.log10(np.maximum(angle_freq["magnitude"], 1e-12)), label="angle ref -> angle actual")
        if speed_freq.get("ok"):
            axes[0].semilogx(speed_freq["freq_hz"], 20.0 * np.log10(np.maximum(speed_freq["magnitude"], 1e-12)), label="speed ref -> speed actual")
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
        fig.suptitle("Empirical closed-loop frequency response")
        fig.tight_layout()
        path = output_dir / "yaw_prbs_frequency_response.png"
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
        fig.suptitle("Yaw PRBS step metrics")
        fig.tight_layout()
        path = output_dir / "yaw_prbs_step_metrics.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths.append(str(path))

    if angle_arx.get("ok"):
        max_lag = int(angle_arx["max_lag"])
        sim = angle_arx["simulation"]
        max_points = min(len(t_grid), int(20.0 / max(np.median(np.diff(t_grid)), 1e-6)))
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(t_grid[:max_points], np.degrees(values["angle_feedback_rad"][:max_points]), label="actual", linewidth=1.0)
        ax.plot(t_grid[:max_points], np.degrees(sim[:max_points]), label="ARX simulation", linewidth=1.0)
        ax.plot(t_grid[:max_points], np.degrees(values["angle_ref_rad"][:max_points]), label="target", linewidth=0.8, alpha=0.8)
        text = f"ARX sim R2={angle_arx['simulation_r2']:.4f}, RMSE={math.degrees(angle_arx['simulation_rmse']):.3f} deg"
        ax.text(0.02, 0.98, text, transform=ax.transAxes, va="top", bbox={"facecolor": "white", "alpha": 0.85})
        ax.axvline(t_grid[max_lag], color="gray", linewidth=0.8, linestyle="--")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Yaw angle (deg)")
        ax.set_title("Closed-loop ARX validation")
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        path = output_dir / "yaw_prbs_arx_validation.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths.append(str(path))

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axis("off")
    rec = result["pid_recommendation"]
    text_lines = [
        "Yaw PRBS analysis summary",
        f"tracking RMSE: {result['overall_metrics']['tracking_rmse_deg']:.3f} deg",
        f"max abs error: {result['overall_metrics']['tracking_max_abs_deg']:.3f} deg",
        f"output saturation: {100.0 * result['overall_metrics']['output_saturation_ratio']:.2f}%",
        f"angle BW: {rec['estimated']['angle_bandwidth_hz']:.3f} Hz",
        f"speed BW: {rec['estimated']['speed_bandwidth_hz']:.3f} Hz",
        f"median overshoot: {100.0 * rec['estimated']['median_overshoot_ratio']:.2f}%",
        "",
        "PID candidate:",
        f"angle Kp: {rec['candidate']['angle_kp']:.6f}",
        f"speed Kp: {rec['candidate']['speed_kp']:.6f}",
        f"speed Ki: {rec['candidate']['speed_ki']:.6f}",
    ]
    ax.text(0.02, 0.98, "\n".join(text_lines), va="top", family="monospace", fontsize=12)
    fig.tight_layout()
    path = output_dir / "yaw_prbs_summary.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(str(path))
    return paths


def main() -> int:
    args = parse_args()
    output_dir = analysis_dir_for(args.csv, args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    df = load_numeric_csv(args.csv)
    mode_df = df[df["mode"].astype(int) == YAW_PRBS_MODE].copy()
    prbs_df = mode_df[mode_df["phase"].astype(int) == PRBS_PHASE].copy()
    if len(prbs_df) < 128:
        print("not enough yaw PRBS samples found")
        return 2

    required = [
        "angle_ref_rad",
        "angle_feedback_rad",
        "speed_ref_rad_s",
        "speed_feedback_rad_s",
        "output_cmd",
        "voltage_ref_raw",
        "seq_index",
    ]
    prbs_df = prbs_df.dropna(subset=required)
    mode_df = mode_df.dropna(subset=required)
    t_grid, values, dt_s = uniform_resample(prbs_df, required)
    overall = response_metrics(
        values["angle_ref_rad"],
        values["angle_feedback_rad"],
        values["output_cmd"],
        args.output_limit,
        args.sat_margin,
    )
    step_df = compute_step_metrics(prbs_df, args)
    angle_freq = estimate_frequency_response(values["angle_ref_rad"], values["angle_feedback_rad"], dt_s, "angle")
    speed_freq = estimate_frequency_response(values["speed_ref_rad_s"], values["speed_feedback_rad_s"], dt_s, "speed")
    angle_arx = fit_arx(values["angle_ref_rad"], values["angle_feedback_rad"], na=4, nb=4, nk=1)
    speed_arx = fit_arx(values["speed_ref_rad_s"], values["speed_feedback_rad_s"], na=4, nb=4, nk=1)
    pid_recommendation = build_pid_recommendation(args, step_df, angle_freq, speed_freq, overall)

    step_csv = output_dir / "yaw_prbs_step_metrics.csv"
    freq_csv = output_dir / "yaw_prbs_frequency_response.csv"
    result_json = output_dir / "yaw_prbs_result.json"
    step_df.to_csv(step_csv, index=False)
    save_frequency_csv(freq_csv, angle_freq, speed_freq)

    result = {
        "input_csv": str(args.csv),
        "output_dir": str(output_dir),
        "step_metrics_csv": str(step_csv),
        "frequency_response_csv": str(freq_csv),
        "sample_period_s": float(dt_s),
        "sample_rate_hz": float(1.0 / dt_s),
        "sample_count": int(len(prbs_df)),
        "overall_metrics": overall,
        "step_metrics_summary": {
            "step_count": int(len(step_df)),
            "median_overshoot_ratio": finite_median(step_df["overshoot_ratio"]) if not step_df.empty else float("nan"),
            "median_rise_time_s": finite_median(step_df["rise_time_s"]) if not step_df.empty else float("nan"),
            "median_settling_time_s": finite_median(step_df["settling_time_s"]) if not step_df.empty else float("nan"),
            "median_steady_error_deg": finite_median(step_df["steady_error_deg"]) if not step_df.empty else float("nan"),
            "mean_stage_error_rmse_deg": finite_mean(step_df["stage_error_rmse_deg"]) if not step_df.empty else float("nan"),
        },
        "frequency_response": {
            "angle_ref_to_angle_actual": scalar_frequency_metrics(angle_freq),
            "speed_ref_to_speed_actual": scalar_frequency_metrics(speed_freq),
        },
        "arx_model": {
            "angle_ref_to_angle_actual": {
                key: value for key, value in angle_arx.items()
                if key not in {"simulation"}
            },
            "speed_ref_to_speed_actual": {
                key: value for key, value in speed_arx.items()
                if key not in {"simulation"}
            },
        },
        "pid_recommendation": pid_recommendation,
        "warnings": [],
    }

    if angle_freq.get("ok") and float(angle_freq["mean_coherence_0p05_3hz"]) < 0.45:
        result["warnings"].append("angle frequency coherence is low; PRBS excitation or data quality may be insufficient")
    if angle_arx.get("ok") and float(angle_arx["simulation_r2"]) < 0.70:
        result["warnings"].append("ARX simulation R2 is low; do not trust PID candidate without another dataset")
    if overall["output_saturation_ratio"] > 0.10:
        result["warnings"].append("output saturation is high; PID fit is affected by nonlinear clipping")

    plot_paths: list[str] = []
    if not args.no_plots:
        plot_paths = plot_results(mode_df, prbs_df, step_df, angle_freq, speed_freq, angle_arx, t_grid, values, result, output_dir)
    result["plot_paths"] = plot_paths
    result_json.write_text(json.dumps(result, indent=2), encoding="utf-8")

    rec = result["pid_recommendation"]
    print(f"output_dir={output_dir}")
    print(f"samples={len(prbs_df)}, dt={dt_s:.6f}s, fs={1.0 / dt_s:.2f}Hz")
    print("")
    print("overall:")
    print(f"  tracking_rmse={overall['tracking_rmse_deg']:.4f} deg")
    print(f"  tracking_max_abs={overall['tracking_max_abs_deg']:.4f} deg")
    print(f"  output_saturation={100.0 * overall['output_saturation_ratio']:.2f}%")
    print("")
    print("frequency:")
    print(f"  angle_bw={rec['estimated']['angle_bandwidth_hz']:.6f} Hz")
    print(f"  speed_bw={rec['estimated']['speed_bandwidth_hz']:.6f} Hz")
    print("")
    print("PID candidate:")
    print(f"  angle Kp: {rec['current']['angle_kp']:.6f} -> {rec['candidate']['angle_kp']:.6f}")
    print(f"  speed Kp: {rec['current']['speed_kp']:.6f} -> {rec['candidate']['speed_kp']:.6f}")
    print(f"  speed Ki: {rec['current']['speed_ki']:.6f} -> {rec['candidate']['speed_ki']:.6f}")
    if result["warnings"]:
        print("")
        for warning in result["warnings"]:
            print(f"warning: {warning}")
    print(f"json={result_json}")
    print(f"step_csv={step_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
