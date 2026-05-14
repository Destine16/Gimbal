#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a Markdown comparison report for fast-response PID experiments."
    )
    parser.add_argument("--baseline-analysis", type=Path, required=True,
                        help="JSON from analyze_fast_sysid.py for the baseline run")
    parser.add_argument("--candidate-analysis", type=Path, default=None,
                        help="Optional JSON from analyze_fast_sysid.py after applying candidate PID")
    parser.add_argument("--optimization", type=Path, default=None,
                        help="Optional JSON from optimize_fast_pid_model.py")
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def load_json(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def fmt(value: Any, digits: int = 4, suffix: str = "") -> str:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return "N/A"
    if not math.isfinite(f):
        return "N/A"
    return f"{f:.{digits}f}{suffix}"


def get_path(data: dict[str, Any] | None, path: str) -> Any:
    if data is None:
        return None
    cur: Any = data
    for item in path.split("."):
        if not isinstance(cur, dict) or item not in cur:
            return None
        cur = cur[item]
    return cur


def percent_change(old: Any, new: Any) -> str:
    try:
        old_f = float(old)
        new_f = float(new)
    except (TypeError, ValueError):
        return "N/A"
    if not math.isfinite(old_f) or not math.isfinite(new_f) or abs(old_f) < 1e-12:
        return "N/A"
    return f"{100.0 * (new_f - old_f) / old_f:+.2f}%"


def metric_row(name: str, key: str, baseline: dict[str, Any], candidate: dict[str, Any] | None,
               digits: int = 4, suffix: str = "") -> str:
    old = get_path(baseline, key)
    new = get_path(candidate, key)
    return (
        f"| {name} | {fmt(old, digits, suffix)} | "
        f"{fmt(new, digits, suffix) if candidate else '待复测'} | "
        f"{percent_change(old, new) if candidate else '待复测'} |"
    )


def pid_table(opt: dict[str, Any] | None) -> list[str]:
    if opt is None:
        return [
            "尚未提供优化结果 JSON。",
        ]
    current = get_path(opt, "optimization.validation.current") or {}
    fast = get_path(opt, "optimization.validation.optimized") or {}
    rows = [
        "| 参数 | 当前值 | 快响应候选值 | 变化 |",
        "|---|---:|---:|---:|",
    ]
    for label, key in [
        ("angle Kp", "angle_kp"),
        ("speed Kp", "speed_kp"),
        ("speed Ki", "speed_ki"),
        ("speed ref max", "speed_ref_limit"),
    ]:
        rows.append(
            f"| {label} | {fmt(current.get(key), 6)} | {fmt(fast.get(key), 6)} | "
            f"{percent_change(current.get(key), fast.get(key))} |"
        )
    rows.extend([
        "",
        "| 模型验证指标 | 当前值 | 快响应候选值 | 变化 |",
        "|---|---:|---:|---:|",
    ])
    for label, key in [
        ("模型 cost", "cost"),
        ("RMSE", "metrics.tracking_rmse_deg"),
        ("MAE", "metrics.tracking_mae_deg"),
        ("最大误差", "metrics.tracking_max_abs_deg"),
        ("输出饱和占比", "metrics.output_saturation_ratio"),
        ("速度参考饱和占比", "metrics.speed_ref_saturation_ratio"),
    ]:
        old = get_path(current, key)
        new = get_path(fast, key)
        rows.append(f"| {label} | {fmt(old)} | {fmt(new)} | {percent_change(old, new)} |")
    return rows


def main() -> int:
    args = parse_args()
    baseline = load_json(args.baseline_analysis)
    candidate = load_json(args.candidate_analysis)
    opt = load_json(args.optimization)
    assert baseline is not None

    axis = baseline.get("axis", "unknown")
    kind = baseline.get("kind", "unknown")
    lines = [
        f"# {axis} 快响应 PID 对比报告",
        "",
        "## 数据来源",
        "",
        f"- baseline analysis: `{args.baseline_analysis}`",
        f"- candidate analysis: `{args.candidate_analysis}`" if args.candidate_analysis else "- candidate analysis: 待复测",
        f"- optimization: `{args.optimization}`" if args.optimization else "- optimization: 未提供",
        "",
        "## 实验信息",
        "",
        "| 项目 | 值 |",
        "|---|---|",
        f"| 轴 | {axis} |",
        f"| 类型 | {kind} |",
        f"| baseline mode | {baseline.get('mode')} |",
        f"| baseline samples | {baseline.get('sample_count')} |",
        f"| baseline sample rate | {fmt(baseline.get('sample_rate_hz'), 2)} Hz |",
        "",
        "## 实测指标对比",
        "",
        "| 指标 | 当前参数 | 候选参数复测 | 变化 |",
        "|---|---:|---:|---:|",
        metric_row("fast score", "fast_score", baseline, candidate),
        metric_row("tracking RMSE", "tracking_metrics.tracking_rmse_deg", baseline, candidate, suffix=" deg"),
        metric_row("tracking MAE", "tracking_metrics.tracking_mae_deg", baseline, candidate, suffix=" deg"),
        metric_row("最大误差", "tracking_metrics.tracking_max_abs_deg", baseline, candidate, suffix=" deg"),
        metric_row("速度误差 RMSE", "tracking_metrics.speed_rmse_rad_s", baseline, candidate, suffix=" rad/s"),
        metric_row("输出饱和占比", "tracking_metrics.output_saturation_ratio", baseline, candidate),
        metric_row("角度带宽", "frequency_response.angle_ref_to_angle_actual.bandwidth_hz", baseline, candidate, suffix=" Hz"),
        metric_row("0.1-3Hz 相干度", "extra_frequency_metrics.mean_coherence_0p1_3hz", baseline, candidate),
        metric_row("3Hz 相位", "extra_frequency_metrics.phase_deg_at_3hz", baseline, candidate, suffix=" deg"),
        "",
        "## 模型优化建议",
        "",
        *pid_table(opt),
        "",
        "## 结论模板",
        "",
        "- 如果候选参数复测后 `fast score`、RMSE 和相位滞后下降，同时输出饱和占比没有明显上升，可以采用候选参数。",
        "- 如果模型预测变好但复测变差，说明模型外推不可靠，需要重新采一组更高相干度的数据。",
        "- 如果输出饱和占比显著上升，优先处理限幅或降低速度参考，而不是继续增大 PID。",
        "",
    ]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
