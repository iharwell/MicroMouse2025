#!/usr/bin/env python3
"""Iteratively search open-floor UKF replay tuning overrides."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = REPO_ROOT / "tooling" / "run_open_floor_ukf_replay.ps1"

# Keep the search focused on parameters that materially affect replay behavior on the open-floor corpus.
DEFAULT_TUNING: Dict[str, float] = {
    "general_encoder_linear_speed_sigma_mps": 0.004,
    "general_encoder_yaw_rate_sigma_radps": 0.1617,
    "stationary_encoder_velocity_sigma_mps": 1.76e-6,
    "imu_yaw_rate_sigma_radps": 0.0131,
    "stationary_gyro_bias_time_constant_s": 30.0,
    "yaw_consistency_low_pass_threshold_radps": 0.08,
    "yaw_window_mismatch_threshold_rad": 0.03,
    "nhc_base_sigma_mps": 0.005,
    "launch_sigma_u_sqrt_q": 0.020,
    "launch_sigma_r_sqrt_q": 0.050,
    "grip_sigma_u_sqrt_q": 0.012,
    "grip_sigma_r_sqrt_q": 0.025,
    "grip_sigma_wheel_speed_sqrt_q": 0.400,
    "inconsistent_sigma_u_sqrt_q": 0.030,
    "inconsistent_sigma_r_sqrt_q": 0.070,
}

PASS_FACTORS = (
    (0.67, 1.50),
    (0.85, 1.15),
)

METRIC_PATHS = {
    "encoder_linear_rmse_mps": ("prediction", "encoder_linear_rmse_mps"),
    "raw_gyro_rmse_radps": ("prediction", "raw_gyro_rmse_radps"),
    "encoder_yaw_rate_rmse_radps": ("prediction", "encoder_yaw_rate_rmse_radps"),
    "body_forward_speed_rmse_mps": ("prediction", "body_forward_speed_rmse_mps"),
    "accel_body_right_rmse_mps2": ("prediction", "accel_body_right_rmse_mps2"),
    "accel_body_forward_rmse_mps2": ("prediction", "accel_body_forward_rmse_mps2"),
    "post_position_rmse_mm": ("consistency", "post_position_rmse_mm"),
    "post_heading_rmse_deg": ("consistency", "post_heading_rmse_deg"),
}

METRIC_WEIGHTS = {
    "encoder_linear_rmse_mps": 3.0,
    "raw_gyro_rmse_radps": 3.0,
    "encoder_yaw_rate_rmse_radps": 1.5,
    "body_forward_speed_rmse_mps": 1.0,
    "accel_body_right_rmse_mps2": 0.5,
    "accel_body_forward_rmse_mps2": 0.5,
    "post_position_rmse_mm": 1.25,
    "post_heading_rmse_deg": 1.0,
}


@dataclass
class EvaluationResult:
    index: int
    label: str
    output_dir: Path
    tuning_path: Optional[Path]
    aggregate_path: Optional[Path]
    score: float
    metrics: Dict[str, float]
    completed_runs: int
    unique_runs: int
    replay_fault_runs: int
    returncode: int
    failure_reason: str = ""


def timestamp() -> str:
    return dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def write_tuning_file(path: Path, tuning: Dict[str, float]) -> None:
    lines = [f"{key}={tuning[key]:.12g}" for key in sorted(tuning)]
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def metric_values(aggregate: dict) -> Dict[str, float]:
    values: Dict[str, float] = {}
    for name, path in METRIC_PATHS.items():
        current = aggregate
        for part in path:
            current = current[part]
        values[name] = float(current)
    return values


def normalized_score(aggregate: dict, baseline_metrics: Dict[str, float]) -> float:
    score = 25.0 * float(aggregate.get("replay_fault_runs", 0))
    for name, weight in METRIC_WEIGHTS.items():
        value = metric_values(aggregate)[name]
        baseline = baseline_metrics[name]
        scale = baseline if baseline > 1.0e-12 else max(abs(value), 1.0)
        score += weight * (value / scale)
    return score


def tuning_signature(tuning: Dict[str, float]) -> tuple[tuple[str, float], ...]:
    return tuple(sorted((key, round(value, 12)) for key, value in tuning.items()))


def current_value(overrides: Dict[str, float], key: str) -> float:
    return overrides.get(key, DEFAULT_TUNING[key])


def candidate_tuning(base: Dict[str, float], key: str, factor: float) -> Dict[str, float]:
    tuning = dict(base)
    value = max(current_value(base, key) * factor, 1.0e-12)
    default_value = DEFAULT_TUNING[key]
    if math.isclose(value, default_value, rel_tol=1.0e-9, abs_tol=1.0e-12):
        tuning.pop(key, None)
    else:
        tuning[key] = value
    return tuning


def run_replay(
    root: Path,
    output_root: Path,
    run_id: str,
    tuning: Dict[str, float],
    label: str,
    index: int,
    skip_tool_build: bool,
    baseline_metrics: Optional[Dict[str, float]],
) -> EvaluationResult:
    output_dir = output_root / f"{index:03d}_{label}"
    output_dir.mkdir(parents=True, exist_ok=True)

    tuning_path: Optional[Path] = None
    if tuning:
        tuning_path = output_dir / "tuning.txt"
        write_tuning_file(tuning_path, tuning)

    command = [
        "powershell",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(RUNNER_PATH),
        "-Root",
        str(root),
        "-Output",
        str(output_dir),
        "-KnownStationarySeed",
    ]
    if run_id:
        command.extend(["-RunId", run_id])
    if tuning_path is not None:
        command.extend(["-Tuning", str(tuning_path)])
    if skip_tool_build:
        command.append("-SkipToolBuild")

    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    (output_dir / "stdout.txt").write_text(completed.stdout, encoding="utf-8", errors="replace")
    (output_dir / "stderr.txt").write_text(completed.stderr, encoding="utf-8", errors="replace")

    aggregate_path = output_dir / "aggregate_metrics.json"
    if completed.returncode != 0 or not aggregate_path.exists():
        return EvaluationResult(
            index=index,
            label=label,
            output_dir=output_dir,
            tuning_path=tuning_path,
            aggregate_path=aggregate_path if aggregate_path.exists() else None,
            score=float("inf"),
            metrics={},
            completed_runs=0,
            unique_runs=0,
            replay_fault_runs=0,
            returncode=completed.returncode,
            failure_reason=completed.stderr.strip() or completed.stdout.strip() or "Replay command failed",
        )

    aggregate = load_json(aggregate_path)
    metrics = metric_values(aggregate)
    if baseline_metrics is None:
        score = sum(METRIC_WEIGHTS.values())
    else:
        score = normalized_score(aggregate, baseline_metrics)

    return EvaluationResult(
        index=index,
        label=label,
        output_dir=output_dir,
        tuning_path=tuning_path,
        aggregate_path=aggregate_path,
        score=score,
        metrics=metrics,
        completed_runs=int(aggregate.get("completed_runs", 0)),
        unique_runs=int(aggregate.get("unique_runs", 0)),
        replay_fault_runs=int(aggregate.get("replay_fault_runs", 0)),
        returncode=completed.returncode,
    )


def append_evaluation_row(log_path: Path, result: EvaluationResult, tuning: Dict[str, float]) -> None:
    fieldnames = [
        "index",
        "label",
        "score",
        "completed_runs",
        "unique_runs",
        "replay_fault_runs",
        "returncode",
        "failure_reason",
        *METRIC_PATHS.keys(),
        "tuning_json",
        "output_dir",
    ]
    exists = log_path.exists()
    with log_path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        if not exists:
            writer.writeheader()
        row = {
            "index": result.index,
            "label": result.label,
            "score": f"{result.score:.12g}" if math.isfinite(result.score) else "inf",
            "completed_runs": result.completed_runs,
            "unique_runs": result.unique_runs,
            "replay_fault_runs": result.replay_fault_runs,
            "returncode": result.returncode,
            "failure_reason": result.failure_reason,
            "tuning_json": json.dumps(tuning, sort_keys=True),
            "output_dir": str(result.output_dir),
        }
        for metric in METRIC_PATHS:
            value = result.metrics.get(metric)
            row[metric] = "" if value is None else f"{value:.12g}"
        writer.writerow(row)


def write_summary(
    path: Path,
    baseline: EvaluationResult,
    best: EvaluationResult,
    best_tuning: Dict[str, float],
    evaluation_count: int,
) -> None:
    lines = [
        "# Open-Floor UKF Tuning Search Summary",
        "",
        f"- Generated: `{timestamp()}`",
        f"- Evaluations: {evaluation_count}",
        f"- Baseline score: {baseline.score:.6f}",
        f"- Best score: {best.score:.6f}",
        f"- Best output: `{best.output_dir}`",
        f"- Best tuning file: `{best.tuning_path if best.tuning_path is not None else '<default>'}`",
        "",
        "## Score Model",
        "",
        "- Score is a weighted sum of replay-fault penalty plus aggregate RMSE metrics normalized to the baseline known-stationary-seed replay.",
        "- Lower is better.",
        "",
        "## Best Overrides",
        "",
    ]
    if not best_tuning:
        lines.append("- `<none>`")
    else:
        for key in sorted(best_tuning):
            lines.append(f"- `{key} = {best_tuning[key]:.12g}`")
    lines.extend([
        "",
        "## Metric Comparison",
        "",
        "| Metric | Baseline | Best | Ratio |",
        "| --- | ---: | ---: | ---: |",
    ])
    for metric in METRIC_PATHS:
        base_value = baseline.metrics.get(metric, float("nan"))
        best_value = best.metrics.get(metric, float("nan"))
        ratio = (best_value / base_value) if base_value and math.isfinite(base_value) else float("nan")
        lines.append(
            f"| `{metric}` | {base_value:.6g} | {best_value:.6g} | {ratio:.4f} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=str(REPO_ROOT / "TestResults"))
    parser.add_argument("--output-root", default="")
    parser.add_argument("--run-id", default="")
    parser.add_argument("--max-passes", type=int, default=len(PASS_FACTORS))
    return parser.parse_args(list(argv))


def main(argv: Iterable[str]) -> int:
    args = parse_args(argv)
    root = Path(args.root).resolve()
    output_root = Path(args.output_root).resolve() if args.output_root else root / f"ukf_tuning_search_{timestamp()}"
    output_root.mkdir(parents=True, exist_ok=True)

    evaluation_log_path = output_root / "evaluation_log.csv"
    best_tuning_path = output_root / "best_tuning.txt"
    summary_path = output_root / "summary.md"

    seen = set()
    evaluation_index = 0

    baseline_tuning: Dict[str, float] = {}
    baseline = run_replay(
        root=root,
        output_root=output_root,
        run_id=args.run_id,
        tuning=baseline_tuning,
        label="baseline",
        index=evaluation_index,
        skip_tool_build=False,
        baseline_metrics=None,
    )
    append_evaluation_row(evaluation_log_path, baseline, baseline_tuning)
    if not math.isfinite(baseline.score):
        print(f"Baseline replay failed: {baseline.failure_reason}", file=sys.stderr)
        return 1

    baseline_metrics = baseline.metrics
    baseline_result = EvaluationResult(
        index=baseline.index,
        label=baseline.label,
        output_dir=baseline.output_dir,
        tuning_path=baseline.tuning_path,
        aggregate_path=baseline.aggregate_path,
        score=normalized_score(load_json(baseline.aggregate_path), baseline_metrics),  # type: ignore[arg-type]
        metrics=baseline.metrics,
        completed_runs=baseline.completed_runs,
        unique_runs=baseline.unique_runs,
        replay_fault_runs=baseline.replay_fault_runs,
        returncode=baseline.returncode,
        failure_reason=baseline.failure_reason,
    )
    best_tuning: Dict[str, float] = {}
    best_result = baseline_result
    seen.add(tuning_signature(best_tuning))
    best_tuning_path.write_text("", encoding="ascii")

    print(f"Baseline complete: score={best_result.score:.6f} output={best_result.output_dir}")

    evaluation_index = 1
    max_passes = max(1, min(args.max_passes, len(PASS_FACTORS)))
    for pass_index in range(max_passes):
        pass_improved = False
        factors = PASS_FACTORS[pass_index]
        for key in DEFAULT_TUNING:
            for factor in factors:
                candidate = candidate_tuning(best_tuning, key, factor)
                signature = tuning_signature(candidate)
                if signature in seen:
                    continue
                seen.add(signature)

                label = f"p{pass_index + 1}_{key}_{factor:.2f}".replace(".", "p")
                result = run_replay(
                    root=root,
                    output_root=output_root,
                    run_id=args.run_id,
                    tuning=candidate,
                    label=label,
                    index=evaluation_index,
                    skip_tool_build=True,
                    baseline_metrics=baseline_metrics,
                )
                append_evaluation_row(evaluation_log_path, result, candidate)
                evaluation_index += 1

                if result.score < best_result.score:
                    best_tuning = candidate
                    best_result = result
                    pass_improved = True
                    if best_result.tuning_path is not None:
                        best_tuning_path.write_text(best_result.tuning_path.read_text(encoding="ascii"), encoding="ascii")
                    else:
                        best_tuning_path.write_text("", encoding="ascii")
                    print(
                        f"Improved: score={best_result.score:.6f} param={key} factor={factor:.2f} output={best_result.output_dir}"
                    )
        if not pass_improved:
            break

    write_summary(
        summary_path,
        baseline=baseline_result,
        best=best_result,
        best_tuning=best_tuning,
        evaluation_count=evaluation_index,
    )

    print(f"Best score: {best_result.score:.6f}")
    print(f"Best output: {best_result.output_dir}")
    print(f"Best tuning: {best_tuning_path}")
    print(f"Evaluation log: {evaluation_log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
