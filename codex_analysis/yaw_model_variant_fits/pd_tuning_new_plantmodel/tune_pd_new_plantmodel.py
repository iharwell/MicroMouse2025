#!/usr/bin/env python3
"""Analysis-only PD sweep for the current yaw-residual PlantModel.

This intentionally does not build or run project tests.  It uses the current
source constants plus recent selected-log yaw artifacts to tune the DriveBase
state-feedback gains in a compact yaw-axis replay.
"""

from __future__ import annotations

import csv
import math
import re
from collections import deque
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
DT = 0.001


@dataclass(frozen=True)
class Gains:
    velocity_kp: float
    velocity_kd: float
    heading_kp: float
    heading_kd: float
    yawrate_kp: float
    yawrate_kd: float


@dataclass(frozen=True)
class Scenario:
    name: str
    duration_s: float
    initial_yaw_rate: float
    target_yaw_rate: float
    target_heading: float | None
    accel_limit: float
    tau_s: float
    delay_s: float
    weight: float


def load_named_float(path: Path, name: str) -> float:
    text = path.read_text(encoding="utf-8", errors="replace")
    pattern = re.compile(rf"\b{name}\s*=\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)[fF]?\b")
    match = pattern.search(text)
    if match is None:
        raise ValueError(f"missing {name} in {path}")
    return float(match.group(1))


def load_current_gains() -> Gains:
    path = ROOT / "MazeMap" / "MazeMap" / "CoreConfig.h"
    return Gains(
        velocity_kp=load_named_float(path, "kDriveBaseVelocityStateKp"),
        velocity_kd=load_named_float(path, "kDriveBaseVelocityStateKd"),
        heading_kp=load_named_float(path, "kDriveBaseHeadingStateKp"),
        heading_kd=load_named_float(path, "kDriveBaseHeadingStateKd"),
        yawrate_kp=load_named_float(path, "kDriveBaseYawRateStateKp"),
        yawrate_kd=load_named_float(path, "kDriveBaseYawRateStateKd"),
    )


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8", errors="replace") as handle:
        return list(csv.DictReader(handle))


def median(values: list[float]) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def load_evidence() -> dict[str, float]:
    launch = read_csv_rows(ROOT / "codex_analysis" / "yaw_launch_step_response" / "yaw_launch_aggregate.csv")
    sustained = [
        row for row in launch
        if float(row["amplitude"]) >= 0.65 and int(row["sustained_launch_steps"]) > 0
    ]
    launch_accels = [abs(float(row["median_initial_yaw_accel_plus2_radps2"])) for row in sustained]
    launch_delays = [float(row["median_delay_ms"]) for row in sustained]
    launch_tau_ms = [
        float(row["median_time_constant_63_ms"]) for row in sustained
        if row["median_time_constant_63_ms"].lower() != "nan"
    ]

    blend_rows = read_csv_rows(
        ROOT
        / "codex_analysis"
        / "yaw_model_variant_fits"
        / "transition_options"
        / "rational_speed_force_blend"
        / "candidate_scores.csv"
    )
    selected = next(row for row in blend_rows if row.get("selected") == "True")

    split_rows = read_csv_rows(
        ROOT
        / "codex_analysis"
        / "yaw_model_variant_fits"
        / "transition_options"
        / "rational_speed_force_blend"
        / "split_metrics.csv"
    )
    primary = next(row for row in split_rows if row["group"] == "primary_open_floor_fit_authoritative")
    validation = next(row for row in split_rows if row["group"] == "validation_non_authoritative")

    return {
        "launch_accel_median_radps2": median(launch_accels),
        "launch_delay_median_s": 0.001 * median(launch_delays),
        "launch_tau_median_s": 0.001 * median(launch_tau_ms),
        "selected_in_place_cmd": float(selected["in_place_max_abs_command"]),
        "selected_in_place_extra_nm": float(selected["in_place_extra_opposing_nm"]),
        "primary_baseline_rmse_nm": float(primary["baseline_rmse_nm"]),
        "primary_corrected_rmse_nm": float(primary["corrected_rmse_nm"]),
        "validation_baseline_rmse_nm": float(validation["baseline_rmse_nm"]),
        "validation_corrected_rmse_nm": float(validation["corrected_rmse_nm"]),
    }


def clamp(value: float, lo: float, hi: float) -> float:
    return min(hi, max(lo, value))


def sign_changes(values: list[float], deadband: float) -> int:
    last = 0
    changes = 0
    for value in values:
        current = 1 if value > deadband else (-1 if value < -deadband else 0)
        if current == 0:
            continue
        if last != 0 and current != last:
            changes += 1
        last = current
    return changes


def simulate(gains: Gains, scenario: Scenario) -> dict[str, float]:
    samples = max(1, int(round(scenario.duration_s / DT)))
    delay_samples = max(0, int(round(scenario.delay_s / DT)))
    delay_line: deque[float] = deque([0.0] * delay_samples, maxlen=max(1, delay_samples))
    yaw_rate = scenario.initial_yaw_rate
    heading = 0.0
    actual_accel = 0.0
    errors: list[float] = []
    heading_errors: list[float] = []
    accel_cmds: list[float] = []
    saturation_count = 0
    crossed = False
    first_cross_s = float("nan")

    for index in range(samples):
        yaw_error = scenario.target_yaw_rate - yaw_rate
        accel_cmd = gains.yawrate_kp * yaw_error
        if scenario.target_heading is not None:
            heading_error = scenario.target_heading - heading
            accel_cmd += gains.heading_kp * heading_error
            accel_cmd += gains.heading_kd * yaw_error
            heading_errors.append(heading_error)
        else:
            heading_errors.append(0.0)

        limited_accel = clamp(accel_cmd, -scenario.accel_limit, scenario.accel_limit)
        if abs(limited_accel - accel_cmd) > 1.0e-9:
            saturation_count += 1
        if delay_samples:
            delay_line.append(limited_accel)
            delayed = delay_line[0]
        else:
            delayed = limited_accel

        alpha = DT / max(DT, scenario.tau_s)
        actual_accel += alpha * (delayed - actual_accel)
        yaw_rate += actual_accel * DT
        heading += yaw_rate * DT

        error = scenario.target_yaw_rate - yaw_rate
        if not crossed and (scenario.target_yaw_rate - scenario.initial_yaw_rate) != 0.0:
            if math.copysign(1.0, scenario.target_yaw_rate - scenario.initial_yaw_rate) != math.copysign(1.0, error):
                crossed = True
                first_cross_s = index * DT
        errors.append(error)
        accel_cmds.append(accel_cmd)

    early = errors[: min(len(errors), 500)]
    late = errors[int(0.75 * len(errors)) :]
    heading_late = heading_errors[int(0.75 * len(heading_errors)) :]
    target_scale = max(1.0, abs(scenario.target_yaw_rate - scenario.initial_yaw_rate), abs(scenario.target_yaw_rate))
    rms_early = math.sqrt(sum(value * value for value in early) / len(early)) / target_scale
    rms_late = math.sqrt(sum(value * value for value in late) / len(late)) / target_scale
    p2p_late = (max(late) - min(late)) / target_scale
    heading_abs_late = sum(abs(value) for value in heading_late) / len(heading_late)
    changes = sign_changes(errors[int(0.1 * len(errors)) :], 0.01 * target_scale)
    saturation_fraction = saturation_count / samples
    accel_rms = math.sqrt(sum(value * value for value in accel_cmds) / len(accel_cmds)) / scenario.accel_limit
    score = (
        60.0 * rms_early
        + 90.0 * rms_late
        + 30.0 * p2p_late
        + 20.0 * saturation_fraction
        + 2.5 * changes
        + 25.0 * heading_abs_late
        + 0.5 * accel_rms
    )
    return {
        "score": scenario.weight * score,
        "rms_early_norm": rms_early,
        "rms_late_norm": rms_late,
        "late_p2p_norm": p2p_late,
        "heading_late_abs_rad": heading_abs_late,
        "sign_changes": float(changes),
        "saturation_fraction": saturation_fraction,
        "accel_cmd_rms_over_limit": accel_rms,
        "first_cross_s": first_cross_s,
        "final_yaw_rate_error": errors[-1],
        "final_heading_error": heading_errors[-1] if heading_errors else 0.0,
    }


def evaluate(gains: Gains, scenarios: list[Scenario]) -> tuple[float, list[dict[str, float | str]]]:
    rows: list[dict[str, float | str]] = []
    total = 0.0
    for scenario in scenarios:
        metrics = simulate(gains, scenario)
        total += float(metrics["score"])
        rows.append({"scenario": scenario.name, **metrics})
    return total, rows


def build_scenarios(evidence: dict[str, float]) -> list[Scenario]:
    launch_accel = max(150.0, evidence["launch_accel_median_radps2"])
    delay = evidence["launch_delay_median_s"]
    tau = max(0.012, evidence["launch_tau_median_s"])
    return [
        Scenario("launch_0_to_1_radps", 0.70, 0.0, 1.0, None, launch_accel, tau, delay, 1.35),
        Scenario("yaw_rate_0_to_9_radps", 1.10, 0.0, 9.0, None, 645.0, tau, delay, 1.00),
        Scenario("yaw_rate_9_to_minus9_radps", 1.25, 9.0, -9.0, None, 645.0, tau, delay, 1.15),
        Scenario("combined_3mps_5p5radps", 0.95, 0.0, 5.5, None, 645.0, tau, delay, 0.90),
        Scenario("heading_correction_15deg_at_3mps", 0.85, 0.0, 0.0, math.radians(15.0), 645.0, tau, delay, 1.25),
    ]


def gain_grid(current: Gains) -> list[Gains]:
    candidates: list[Gains] = [current]
    yaw_values = [48.0, 60.0, 72.0, 84.0, 96.0, 108.0, 120.0, 126.0, 138.0, 150.0, 162.0]
    heading_values = [450.0, 600.0, 750.0, 900.0, 1200.0, 1600.0, 2200.0, 3000.0, 3800.0, 4800.0, 6000.0, 7200.0, 8400.0, 9600.0, 9718.0]
    heading_d_values = [0.0, 8.0, 16.0, 24.0, 32.0, 48.0, 64.0, 80.0]
    for yaw_kp in yaw_values:
        for heading_kp in heading_values:
            for heading_kd in heading_d_values:
                candidates.append(
                    Gains(
                        velocity_kp=current.velocity_kp,
                        velocity_kd=current.velocity_kd,
                        heading_kp=heading_kp,
                        heading_kd=heading_kd,
                        yawrate_kp=yaw_kp,
                        yawrate_kd=current.yawrate_kd,
                    )
                )
    return candidates


def write_dict_csv(path: Path, rows: list[dict[str, float | str]]) -> None:
    if not rows:
        return
    keys = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    current = load_current_gains()
    evidence = load_evidence()
    scenarios = build_scenarios(evidence)

    scored: list[tuple[float, Gains]] = []
    summary_rows: list[dict[str, float | str]] = []
    for candidate in gain_grid(current):
        score, _ = evaluate(candidate, scenarios)
        scored.append((score, candidate))
    scored.sort(key=lambda item: item[0])

    seen: set[tuple[float, float, float]] = set()
    top: list[tuple[float, Gains]] = []
    for score, gains in scored:
        key = (gains.heading_kp, gains.heading_kd, gains.yawrate_kp)
        if key in seen:
            continue
        seen.add(key)
        top.append((score, gains))
        if len(top) >= 20:
            break

    baseline_score, baseline_scenarios = evaluate(current, scenarios)
    best_score, best = top[0]
    best_score_check, best_scenarios = evaluate(best, scenarios)
    assert abs(best_score - best_score_check) < 1.0e-9

    recommended = best
    recommended_score = best_score
    recommended_scenarios = best_scenarios
    for score, gains in top:
        _, rows = evaluate(gains, scenarios)
        max_abs_final_heading = max(abs(float(row["final_heading_error"])) for row in rows)
        if (
            score <= (1.25 * best_score)
            and max_abs_final_heading <= 0.005
            and gains.heading_kp >= 600.0
            and gains.yawrate_kp >= 60.0
        ):
            recommended = gains
            recommended_score = score
            recommended_scenarios = rows
            break

    for rank, (score, gains) in enumerate(top, start=1):
        summary_rows.append(
            {
                "rank": rank,
                "score": score,
                "score_delta_vs_current_pct": 100.0 * ((score / baseline_score) - 1.0),
                "velocity_kp": gains.velocity_kp,
                "velocity_kd": gains.velocity_kd,
                "heading_kp": gains.heading_kp,
                "heading_kd": gains.heading_kd,
                "yawrate_kp": gains.yawrate_kp,
                "yawrate_kd": gains.yawrate_kd,
            }
        )

    write_dict_csv(OUT / "candidate_summary.csv", summary_rows)

    scenario_rows: list[dict[str, float | str]] = []
    for label, total, rows in [
        ("current", baseline_score, baseline_scenarios),
        ("grid_best", best_score, best_scenarios),
        ("recommended_balanced", recommended_score, recommended_scenarios),
    ]:
        for row in rows:
            scenario_rows.append({"candidate": label, "total_score": total, **row})
    write_dict_csv(OUT / "scenario_metrics.csv", scenario_rows)

    with (OUT / "evidence_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["metric", "value"])
        for key in sorted(evidence):
            writer.writerow([key, evidence[key]])
        writer.writerow(["current_score", baseline_score])
        writer.writerow(["grid_best_score", best_score])
        writer.writerow(["recommended_balanced_score", recommended_score])

    print(f"current={current}")
    print(f"grid_best={best}")
    print(f"recommended_balanced={recommended}")
    print(f"current_score={baseline_score:.6f}")
    print(f"grid_best_score={best_score:.6f}")
    print(f"recommended_balanced_score={recommended_score:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
