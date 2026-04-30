#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Score gimbal step/sine performance metrics.")
    parser.add_argument("--yaw-step", type=Path, required=True)
    parser.add_argument("--pitch-step", type=Path, default=None)
    parser.add_argument("--yaw-sine", type=Path, required=True)
    parser.add_argument("--pitch-sine", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def finite(value: float, fallback: float) -> float:
    return float(value) if math.isfinite(float(value)) else fallback


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else float("nan")


def score_step(path: Path) -> dict[str, float]:
    df = pd.read_csv(path)
    rmse = mean([finite(v, 20.0) for v in df["rmse_deg"]])
    max_error = mean([finite(v, 30.0) for v in df["max_error_deg"]])
    steady = mean([abs(finite(v, 10.0)) for v in df["steady_error_deg"]])
    overshoot = mean([max(0.0, finite(v, 100.0)) for v in df["overshoot_percent"]])
    settle = mean([finite(v, 3.0) for v in df["settling_time_s"]])
    rise = mean([finite(v, 3.0) for v in df["rise_time_s"]])

    score = (
        2.0 * rmse +
        1.0 * max_error +
        3.0 * steady +
        0.20 * overshoot +
        2.0 * settle +
        1.0 * rise
    )
    return {
        "score": score,
        "rmse_deg": rmse,
        "max_error_deg": max_error,
        "steady_error_deg": steady,
        "overshoot_percent": overshoot,
        "settling_time_s": settle,
        "rise_time_s": rise,
    }


def score_sine(path: Path) -> dict[str, float]:
    df = pd.read_csv(path)
    rmse = mean([finite(v, 20.0) for v in df["rmse_deg"]])
    mae = mean([finite(v, 20.0) for v in df["mae_deg"]])
    max_error = mean([finite(v, 30.0) for v in df["max_error_deg"]])
    gain_error = mean([abs(1.0 - finite(v, 0.0)) for v in df["amplitude_ratio"]])
    phase_lag = mean([abs(finite(v, 180.0)) for v in df["phase_lag_deg"]])

    score = (
        2.0 * rmse +
        1.5 * mae +
        0.8 * max_error +
        20.0 * gain_error +
        0.20 * phase_lag
    )
    return {
        "score": score,
        "rmse_deg": rmse,
        "mae_deg": mae,
        "max_error_deg": max_error,
        "gain_error": gain_error,
        "phase_lag_deg_abs": phase_lag,
    }


def add_optional(result: dict[str, object], key: str, path: Path | None, scorer) -> None:
    if path is None:
        return
    result[key] = scorer(path)


def main() -> int:
    args = parse_args()
    result: dict[str, object] = {
        "yaw_step": score_step(args.yaw_step),
        "yaw_sine": score_sine(args.yaw_sine),
    }
    add_optional(result, "pitch_step", args.pitch_step, score_step)
    add_optional(result, "pitch_sine", args.pitch_sine, score_sine)

    total = 0.0
    for key, value in result.items():
        if isinstance(value, dict) and "score" in value:
            total += float(value["score"])
    result["total_score"] = total

    text = json.dumps(result, indent=2)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
