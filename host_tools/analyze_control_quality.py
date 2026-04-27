#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd


RAD_TO_DEG = 180.0 / math.pi


def has_columns(df: pd.DataFrame, columns: list[str]) -> bool:
    return all(column in df.columns for column in columns)


def pct(mask: pd.Series | np.ndarray) -> float:
    values = np.asarray(mask, dtype=bool)
    if values.size == 0:
        return 0.0
    return 100.0 * float(np.count_nonzero(values) / values.size)


def rms(values: pd.Series | np.ndarray) -> float:
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(arr * arr)))


def span_deg(series: pd.Series) -> float:
    return float((series.max() - series.min()) * RAD_TO_DEG)


def safe_percentile(values: pd.Series | np.ndarray, q: float) -> float:
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return 0.0
    return float(np.percentile(arr, q))


def fit_relation_deg(x_rad: pd.Series, y_rad: pd.Series) -> dict[str, float]:
    x = (x_rad - x_rad.iloc[0]).to_numpy(dtype=float) * RAD_TO_DEG
    y = (y_rad - y_rad.iloc[0]).to_numpy(dtype=float) * RAD_TO_DEG
    if x.size < 3 or np.ptp(x) < 1e-6 or np.ptp(y) < 1e-6:
        return {
            "gain": 0.0,
            "offset_deg": 0.0,
            "corr": 0.0,
            "residual_rms_deg": 0.0,
            "x_span_deg": float(np.ptp(x)) if x.size else 0.0,
            "y_span_deg": float(np.ptp(y)) if y.size else 0.0,
        }

    gain, offset = np.polyfit(x, y, 1)
    pred = gain * x + offset
    corr = float(np.corrcoef(x, y)[0, 1]) if x.size > 2 else 0.0
    return {
        "gain": float(gain),
        "offset_deg": float(offset),
        "corr": corr,
        "residual_rms_deg": rms(y - pred),
        "x_span_deg": float(np.ptp(x)),
        "y_span_deg": float(np.ptp(y)),
    }


def analyze_pid_axis(df: pd.DataFrame, axis: str) -> dict[str, float]:
    cmd = df[f"cmd_{axis}_rad"]
    actual = df[f"actual_{axis}_rad"]
    err_deg = (cmd - actual) * RAD_TO_DEG
    output = df[f"{axis}_output_cmd"]
    speed = df.get(f"{axis}_speed_feedback_rad_s")
    real_current = df.get(f"{axis}_real_current")
    current_ref = df.get(f"{axis}_current_ref_raw")

    result = {
        "cmd_span_deg": span_deg(cmd),
        "actual_span_deg": span_deg(actual),
        "error_mean_abs_deg": float(err_deg.abs().mean()),
        "error_rms_deg": rms(err_deg),
        "error_p95_abs_deg": safe_percentile(err_deg.abs(), 95),
        "error_max_abs_deg": float(err_deg.abs().max()),
        "output_abs_max": int(output.abs().max()),
        "output_sat_percent_4500": pct(output.abs() >= 4500),
    }
    if speed is not None:
        result["speed_abs_max_rad_s"] = float(speed.abs().max())
        result["speed_rms_rad_s"] = rms(speed)
    if real_current is not None:
        result["real_current_abs_max"] = int(real_current.abs().max())
        result["real_current_rms"] = rms(real_current)
    if current_ref is not None:
        result["current_ref_abs_max"] = float(current_ref.abs().max())
        result["current_ref_rms"] = rms(current_ref)
    return result


def analyze_encoder_consistency(df: pd.DataFrame, axis: str) -> dict[str, float] | None:
    encoder_col = f"{axis}_encoder_total_angle_rad"
    if encoder_col not in df.columns:
        return None
    actual = df[f"actual_{axis}_rad"]
    encoder = df[encoder_col]
    return fit_relation_deg(encoder, actual)


def analyze_ekf(df: pd.DataFrame) -> dict[str, float | int | None]:
    result: dict[str, float | int | None] = {}
    if has_columns(df, ["imu_accel_x_m_s2", "imu_accel_y_m_s2", "imu_accel_z_m_s2"]):
        accel_norm = np.sqrt(
            df["imu_accel_x_m_s2"] ** 2 +
            df["imu_accel_y_m_s2"] ** 2 +
            df["imu_accel_z_m_s2"] ** 2
        )
        result["accel_norm_mean_m_s2"] = float(accel_norm.mean())
        result["accel_norm_std_m_s2"] = float(accel_norm.std())
        result["accel_norm_p95_abs_error_m_s2"] = safe_percentile(np.abs(accel_norm - 9.81), 95)

    for axis in ["x", "y", "z"]:
        column = f"imu_gyro_{axis}_rad_s"
        if column in df.columns:
            result[f"gyro_{axis}_mean_rad_s"] = float(df[column].mean())
            result[f"gyro_{axis}_std_rad_s"] = float(df[column].std())
            result[f"gyro_{axis}_abs_max_rad_s"] = float(df[column].abs().max())

    if "imu_yaw_gyro_corrected_rad_s" in df.columns:
        result["yaw_gyro_corrected_mean_rad_s"] = float(df["imu_yaw_gyro_corrected_rad_s"].mean())
        result["yaw_gyro_corrected_std_rad_s"] = float(df["imu_yaw_gyro_corrected_rad_s"].std())
    if "imu_yaw_gyro_bias_rad_s" in df.columns:
        result["yaw_gyro_bias_last_rad_s"] = float(df["imu_yaw_gyro_bias_rad_s"].iloc[-1])
    if "imu_yaw_gyro_bias_ready" in df.columns:
        result["yaw_gyro_bias_ready_last"] = int(df["imu_yaw_gyro_bias_ready"].iloc[-1])
    if "ekf_stable_flag" in df.columns:
        result["stable_percent"] = pct(df["ekf_stable_flag"] == 1)
    if "ekf_error_count" in df.columns:
        result["error_count_delta"] = int(df["ekf_error_count"].iloc[-1] - df["ekf_error_count"].iloc[0])
    if "ekf_chi_square" in df.columns:
        result["chi_square_p95"] = safe_percentile(df["ekf_chi_square"], 95)
        result["chi_square_max"] = float(df["ekf_chi_square"].max())

    if has_columns(df, ["actual_yaw_rad", "yaw_encoder_total_angle_rad"]):
        result["yaw_imu_vs_encoder"] = fit_relation_deg(df["yaw_encoder_total_angle_rad"], df["actual_yaw_rad"])
    if has_columns(df, ["actual_pitch_rad", "pitch_encoder_total_angle_rad"]):
        result["pitch_imu_vs_encoder"] = fit_relation_deg(df["pitch_encoder_total_angle_rad"], df["actual_pitch_rad"])
    return result


def diagnose(result: dict[str, object]) -> list[str]:
    findings: list[str] = []
    pid = result["pid"]  # type: ignore[index]
    ekf = result["ekf"]  # type: ignore[index]
    state = result["state"]  # type: ignore[index]

    if state["gimbal_ready_percent"] < 99.0:  # type: ignore[index]
        findings.append("gimbal_ready 不稳定: 先排查 IMU/CAN 在线状态,否则 PID/EKF 评价不可靠。")
    if state["yaw_online_percent"] < 99.0 or state["pitch_online_percent"] < 99.0:  # type: ignore[index]
        findings.append("电机在线率不足: 优先排查 CAN、电机 ID、供电或接线。")

    for axis in ["yaw", "pitch"]:
        axis_pid = pid[axis]  # type: ignore[index]
        if axis_pid["output_sat_percent_4500"] > 5.0:
            findings.append(f"{axis} 输出接近限幅时间偏长: 可能目标过激、PID/方向错误、机械阻力或前馈不足。")
        if axis_pid["error_p95_abs_deg"] > 5.0 and axis_pid["output_sat_percent_4500"] < 1.0:
            findings.append(f"{axis} 跟踪误差大但输出未饱和: PID 可能偏软,或速度/电流环限幅偏保守。")
        if axis_pid["error_p95_abs_deg"] > 5.0 and axis_pid["output_sat_percent_4500"] >= 1.0:
            findings.append(f"{axis} 跟踪误差大且输出接近限幅: 不是单纯加大 Kp,要查限位、方向、负载和前馈。")

    for axis in ["yaw", "pitch"]:
        key = f"{axis}_imu_vs_encoder"
        relation = ekf.get(key) if isinstance(ekf, dict) else None
        if isinstance(relation, dict) and relation["x_span_deg"] > 3.0 and relation["y_span_deg"] > 3.0:
            if abs(relation["corr"]) < 0.8:
                findings.append(f"{axis} IMU 与编码器相关性差: 优先查 EKF、轴映射、安装矩阵或反馈符号。")
            if relation["residual_rms_deg"] > 2.0:
                findings.append(f"{axis} IMU/编码器拟合残差偏大: 可能有 EKF 抖动、结构松动或轴耦合。")

    if isinstance(ekf, dict):
        if ekf.get("stable_percent") is not None and ekf["stable_percent"] < 70.0:
            findings.append("EKF StableFlag 占比低: 振动/加速度异常较多,bias 修正和姿态修正可信度会下降。")
        if ekf.get("chi_square_p95") is not None and ekf["chi_square_p95"] > 12.0:
            findings.append("EKF ChiSquare 经常超过门限: 加速度量测与姿态预测不一致,需查振动、安装误差或 R/门限。")
        if ekf.get("yaw_gyro_bias_ready_last") == 0:
            findings.append("yaw gyro bias 尚未收敛: 上电后需要保持静止约 1 秒再评价 yaw 漂移。")
        if ekf.get("yaw_gyro_corrected_mean_rad_s") is not None and abs(ekf["yaw_gyro_corrected_mean_rad_s"]) > 0.01:
            findings.append("补偿后的 yaw gyro 均值仍偏大: yaw bias 估计不足,静止 yaw 仍会漂。")

    if not findings:
        findings.append("未发现明显 PID/EKF 级异常; 若实物异常,需要采集包含异常发生时刻的数据。")
    return findings


def analyze(csv_path: Path) -> dict[str, object]:
    df = pd.read_csv(csv_path)
    if df.empty:
        raise ValueError(f"{csv_path} is empty")
    required = [
        "tick_ms",
        "cmd_yaw_rad",
        "cmd_pitch_rad",
        "actual_yaw_rad",
        "actual_pitch_rad",
        "yaw_output_cmd",
        "pitch_output_cmd",
        "gimbal_ready",
        "imu_online",
        "yaw_motor_online",
        "pitch_motor_online",
    ]
    missing = [name for name in required if name not in df.columns]
    if missing:
        raise ValueError(f"{csv_path} missing columns: {', '.join(missing)}")

    duration_s = float((df["tick_ms"].iloc[-1] - df["tick_ms"].iloc[0]) / 1000.0)
    result: dict[str, object] = {
        "input_csv": str(csv_path),
        "rows": int(len(df)),
        "duration_s": duration_s,
        "sample_rate_hz": float(len(df) / duration_s) if duration_s > 0 else 0.0,
        "state": {
            "gimbal_ready_percent": pct(df["gimbal_ready"] == 1),
            "imu_online_percent": pct(df["imu_online"] == 1),
            "yaw_online_percent": pct(df["yaw_motor_online"] == 1),
            "pitch_online_percent": pct(df["pitch_motor_online"] == 1),
        },
        "pid": {
            "yaw": analyze_pid_axis(df, "yaw"),
            "pitch": analyze_pid_axis(df, "pitch"),
        },
        "encoder_consistency": {
            "yaw": analyze_encoder_consistency(df, "yaw"),
            "pitch": analyze_encoder_consistency(df, "pitch"),
        },
        "ekf": analyze_ekf(df),
    }
    result["findings"] = diagnose(result)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze PID and EKF quality from vision RTT CSV.")
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = analyze(args.csv)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
