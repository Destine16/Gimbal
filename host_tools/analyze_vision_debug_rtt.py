#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


RAD_TO_DEG = 57.29577951308232


def pct(value: float) -> float:
    return 100.0 * float(value)


def safe_span(series: pd.Series) -> float:
    if series.empty:
        return 0.0
    return float(series.max() - series.min())


def fraction(mask: pd.Series | np.ndarray) -> float:
    arr = np.asarray(mask, dtype=bool)
    if arr.size == 0:
        return 0.0
    return float(np.count_nonzero(arr) / arr.size)


def mode_counts(series: pd.Series) -> dict[str, int]:
    return {str(int(k)): int(v) for k, v in series.value_counts(dropna=False).sort_index().items()}


def count_delta(df: pd.DataFrame, column: str) -> int | None:
    if column not in df.columns:
        return None
    return int(df[column].iloc[-1] - df[column].iloc[0])


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    if df.empty:
        raise ValueError(f"{path} is empty")
    required = [
        "tick_ms",
        "seq",
        "valid_frame_count",
        "crc_error_count",
        "vision_rx_target_valid",
        "last_delta_yaw_rad",
        "last_delta_pitch_rad",
        "actual_yaw_rad",
        "actual_pitch_rad",
        "cmd_yaw_rad",
        "cmd_pitch_rad",
        "sentry_state",
        "stall_axis",
        "gimbal_ready",
        "imu_online",
        "yaw_motor_online",
        "pitch_motor_online",
        "can_tx_attempt_count",
        "can_tx_success_count",
        "yaw_output_cmd",
        "pitch_output_cmd",
        "yaw_angle_ref_rad",
        "yaw_angle_feedback_rad",
        "pitch_angle_ref_rad",
        "pitch_angle_feedback_rad",
    ]
    missing = [name for name in required if name not in df.columns]
    if missing:
        raise ValueError(f"{path} missing columns: {', '.join(missing)}")
    return df


def analyze(df: pd.DataFrame, csv_path: Path) -> dict[str, object]:
    duration_s = float((df["tick_ms"].iloc[-1] - df["tick_ms"].iloc[0]) / 1000.0)
    valid_count_delta = int(df["valid_frame_count"].iloc[-1] - df["valid_frame_count"].iloc[0])
    crc_count_delta = int(df["crc_error_count"].iloc[-1] - df["crc_error_count"].iloc[0])
    rx_total = valid_count_delta + crc_count_delta
    can_attempt_delta = int(df["can_tx_attempt_count"].iloc[-1] - df["can_tx_attempt_count"].iloc[0])
    can_success_delta = int(df["can_tx_success_count"].iloc[-1] - df["can_tx_success_count"].iloc[0])
    can_rx_total_delta = count_delta(df, "can_rx_total_count")
    can_rx_matched_delta = count_delta(df, "can_rx_matched_count")
    can_rx_unmatched_delta = count_delta(df, "can_rx_unmatched_count")
    can_rx_0x206_delta = count_delta(df, "can_rx_0x206_count")
    can_rx_0x208_delta = count_delta(df, "can_rx_0x208_count")

    yaw_cmd_err = df["cmd_yaw_rad"] - df["actual_yaw_rad"]
    pitch_cmd_err = df["cmd_pitch_rad"] - df["actual_pitch_rad"]
    yaw_motor_err = df["yaw_angle_ref_rad"] - df["yaw_angle_feedback_rad"]
    pitch_motor_err = df["pitch_angle_ref_rad"] - df["pitch_angle_feedback_rad"]

    summary: dict[str, object] = {
        "input_csv": str(csv_path),
        "rows": int(len(df)),
        "duration_s": duration_s,
        "sample_rate_hz": float(len(df) / duration_s) if duration_s > 0 else 0.0,
        "rx": {
            "valid_frames_delta": valid_count_delta,
            "crc_errors_delta": crc_count_delta,
            "crc_error_rate_percent": pct(crc_count_delta / rx_total) if rx_total > 0 else 0.0,
            "target_valid_percent": pct(fraction(df["vision_rx_target_valid"] == 1)),
            "delta_yaw_span_deg": safe_span(df["last_delta_yaw_rad"]) * RAD_TO_DEG,
            "delta_pitch_span_deg": safe_span(df["last_delta_pitch_rad"]) * RAD_TO_DEG,
            "delta_yaw_abs_max_deg": float(df["last_delta_yaw_rad"].abs().max() * RAD_TO_DEG),
            "delta_pitch_abs_max_deg": float(df["last_delta_pitch_rad"].abs().max() * RAD_TO_DEG),
        },
        "state": {
            "sentry_state_counts": mode_counts(df["sentry_state"]),
            "stall_axis_counts": mode_counts(df["stall_axis"]),
            "gimbal_ready_percent": pct(fraction(df["gimbal_ready"] == 1)),
            "imu_online_percent": pct(fraction(df["imu_online"] == 1)),
            "yaw_online_percent": pct(fraction(df["yaw_motor_online"] == 1)),
            "pitch_online_percent": pct(fraction(df["pitch_motor_online"] == 1)),
        },
        "command_tracking": {
            "cmd_yaw_span_deg": safe_span(df["cmd_yaw_rad"]) * RAD_TO_DEG,
            "cmd_pitch_span_deg": safe_span(df["cmd_pitch_rad"]) * RAD_TO_DEG,
            "actual_yaw_span_deg": safe_span(df["actual_yaw_rad"]) * RAD_TO_DEG,
            "actual_pitch_span_deg": safe_span(df["actual_pitch_rad"]) * RAD_TO_DEG,
            "yaw_cmd_error_abs_max_deg": float(yaw_cmd_err.abs().max() * RAD_TO_DEG),
            "pitch_cmd_error_abs_max_deg": float(pitch_cmd_err.abs().max() * RAD_TO_DEG),
            "yaw_motor_error_abs_max_deg": float(yaw_motor_err.abs().max() * RAD_TO_DEG),
            "pitch_motor_error_abs_max_deg": float(pitch_motor_err.abs().max() * RAD_TO_DEG),
        },
        "output": {
            "yaw_output_abs_max": int(df["yaw_output_cmd"].abs().max()),
            "pitch_output_abs_max": int(df["pitch_output_cmd"].abs().max()),
            "yaw_output_sat_percent_4500": pct(fraction(df["yaw_output_cmd"].abs() >= 4500)),
            "pitch_output_sat_percent_4500": pct(fraction(df["pitch_output_cmd"].abs() >= 4500)),
            "can_tx_attempt_delta": can_attempt_delta,
            "can_tx_success_delta": can_success_delta,
            "can_tx_success_rate_percent": pct(can_success_delta / can_attempt_delta) if can_attempt_delta > 0 else 0.0,
        },
    }
    if can_rx_total_delta is not None:
        summary["can_rx"] = {
            "rx_total_delta": can_rx_total_delta,
            "rx_matched_delta": can_rx_matched_delta,
            "rx_unmatched_delta": can_rx_unmatched_delta,
            "rx_0x206_yaw_delta": can_rx_0x206_delta,
            "rx_0x208_pitch_delta": can_rx_0x208_delta,
            "last_rx_std_id_hex": f"0x{int(df['can_last_rx_std_id'].iloc[-1]):03X}",
            "last_unmatched_rx_std_id_hex": f"0x{int(df['can_last_unmatched_rx_std_id'].iloc[-1]):03X}",
        }
    return summary


def diagnose(summary: dict[str, object]) -> list[str]:
    rx = summary["rx"]  # type: ignore[index]
    state = summary["state"]  # type: ignore[index]
    tracking = summary["command_tracking"]  # type: ignore[index]
    output = summary["output"]  # type: ignore[index]
    can_rx = summary.get("can_rx")
    findings: list[str] = []

    if rx["valid_frames_delta"] == 0:  # type: ignore[index]
        findings.append("没有解析到合法视觉帧: 优先查 USB/串口/协议/CRC。")
    if rx["crc_error_rate_percent"] > 1.0:  # type: ignore[index]
        findings.append("CRC 错误率偏高: 优先查视觉端串口写入是否拆包/并发、帧结构和线缆稳定性。")
    if state["gimbal_ready_percent"] < 99.0:  # type: ignore[index]
        findings.append("gimbal_ready 不是长期为 1: 优先查 IMU 或电机在线状态。")
    if state["imu_online_percent"] < 99.0:  # type: ignore[index]
        findings.append("IMU 在线率不足: 优先查 BMI088 数据更新/初始化/RTT 采集期间是否卡顿。")
    if state["yaw_online_percent"] < 99.0 or state["pitch_online_percent"] < 99.0:  # type: ignore[index]
        findings.append("电机在线率不足: 优先查 CAN 接线、电机 ID、反馈帧。")
    if isinstance(can_rx, dict):
        if state["yaw_online_percent"] >= 99.0 and state["pitch_online_percent"] < 99.0:  # type: ignore[index]
            if can_rx.get("rx_0x208_pitch_delta") == 0:
                findings.append("yaw 在线但没有收到 0x208: pitch 电机未反馈,优先查 pitch ID/供电/CAN 接线。")
            elif can_rx.get("rx_0x208_pitch_delta", 0) > 0:
                findings.append("收到了 0x208 但 pitch 仍离线: 优先查解析匹配、daemon 刷新或反馈间隔是否超过 50ms。")
        if can_rx.get("rx_unmatched_delta", 0) > 0:
            findings.append("存在未匹配 CAN 帧: 查看 last_unmatched_rx_std_id_hex,可能有电机 ID 与配置不一致。")
    if output["can_tx_success_rate_percent"] < 99.0 and output["can_tx_attempt_delta"] > 0:  # type: ignore[index]
        findings.append("CAN 发送成功率不足: 优先查 CAN 总线、邮箱拥塞或 HAL 错误码。")
    if state["gimbal_ready_percent"] < 99.0 and float(tracking["cmd_yaw_span_deg"]) < 0.2 and float(tracking["cmd_pitch_span_deg"]) < 0.2:  # type: ignore[index]
        findings.append("云台未 ready 时目标不会更新到控制器: 先解决离线模块,再判断视觉 delta 是否有效。")
    if state["gimbal_ready_percent"] >= 99.0 and float(tracking["cmd_yaw_span_deg"]) < 0.2 and float(tracking["cmd_pitch_span_deg"]) < 0.2 and rx["target_valid_percent"] > 50.0:  # type: ignore[index]
        findings.append("视觉有效但电控目标几乎不变: 视觉 delta 可能过小、重复旧值或 target_valid/发送节奏不对。")
    if float(tracking["cmd_yaw_span_deg"]) > 1.0 and float(tracking["actual_yaw_span_deg"]) < 0.2:  # type: ignore[index]
        findings.append("yaw 目标变化但实际不跟: 优先查 yaw PID/电机输出/方向/限幅。")
    if float(tracking["cmd_pitch_span_deg"]) > 1.0 and float(tracking["actual_pitch_span_deg"]) < 0.2:  # type: ignore[index]
        findings.append("pitch 目标变化但实际不跟: 优先查 pitch PID/前馈/限位/方向。")
    if output["yaw_output_sat_percent_4500"] > 5.0:  # type: ignore[index]
        findings.append("yaw 输出长时间接近限幅: 可能目标过大、PID/方向错误、堵转或电机力矩不足。")
    if output["pitch_output_sat_percent_4500"] > 5.0:  # type: ignore[index]
        findings.append("pitch 输出长时间接近限幅: 可能重力前馈不足、限位/线缆阻力、PID 或方向问题。")
    if not findings:
        findings.append("未发现明显链路级异常; 若实物仍异常,需要结合具体异常时间段放大查看。")
    return findings


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze normal vision-control RTT debug CSV.")
    parser.add_argument("csv", type=Path, help="CSV captured by vision_debug_rtt_capture.py")
    parser.add_argument("--output", type=Path, default=None, help="Optional JSON output path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    df = load_csv(args.csv)
    summary = analyze(df, args.csv)
    findings = diagnose(summary)
    result = {
        **summary,
        "findings": findings,
    }

    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
