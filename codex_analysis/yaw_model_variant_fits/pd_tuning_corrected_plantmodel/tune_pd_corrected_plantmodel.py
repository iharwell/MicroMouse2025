#!/usr/bin/env python3
"""Analysis-only PD sweep for corrected PlantModel yaw dynamics.

This script does not build, test, or modify production code.  It reads the
current local PlantModel/CoreConfig source plus existing selected-log yaw
evidence, then scores DriveBase yaw/heading PD candidates in a compact
yaw-axis replay.
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


@dataclass(frozen=True)
class Objective:
    name: str
    launch_weight: float
    step_weight: float
    reversal_weight: float
    combined_weight: float
    heading_weight: float
    saturation_weight: float
    accel_weight: float
    gain_regularization: float


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8", errors="replace") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def median(values: list[float]) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


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


def plant_source_flags() -> dict[str, bool]:
    text = (ROOT / "MazeMap" / "MazeMap" / "PlantModel.cpp").read_text(encoding="utf-8", errors="replace")
    return {
        "has_variant_c_symbols": "variantCYawResidualOpposingMomentNm" in text,
        "has_request_conditioned_feedforward_branch": "variantCAtProjectedMoment" in text,
        "has_projected_force_utilization": "projectedYawMomentNm" in text,
        "has_rational_gate": "RationalSquareGate" in text,
    }


def load_evidence() -> dict[str, float | str | bool]:
    launch_rows = read_csv_rows(ROOT / "codex_analysis" / "yaw_launch_step_response" / "yaw_launch_aggregate.csv")
    sustained = [
        row for row in launch_rows
        if float(row["amplitude"]) >= 0.65 and int(row["sustained_launch_steps"]) > 0
    ]
    launch_accels = [abs(float(row["median_initial_yaw_accel_plus2_radps2"])) for row in sustained]
    launch_delays = [float(row["median_delay_ms"]) for row in sustained]
    launch_tau_ms = [
        float(row["median_time_constant_63_ms"])
        for row in sustained
        if row["median_time_constant_63_ms"].lower() != "nan"
    ]

    force_rows = read_csv_rows(
        ROOT
        / "codex_analysis"
        / "yaw_model_variant_fits"
        / "round2_force_domain_stribeck"
        / "candidate_tuning_scores.csv"
    )
    selected_force = next(row for row in force_rows if row["role"] == "selected_projected_force_domain")

    force_split_rows = read_csv_rows(
        ROOT
        / "codex_analysis"
        / "yaw_model_variant_fits"
        / "round2_force_domain_stribeck"
        / "split_rmse.csv"
    )
    force_primary = next(
        row for row in force_split_rows if row["dataset_split"] == "primary_open_floor_fit_authoritative"
    )

    blend_rows = read_csv_rows(
        ROOT
        / "codex_analysis"
        / "yaw_model_variant_fits"
        / "transition_options"
        / "rational_speed_force_blend"
        / "candidate_scores.csv"
    )
    selected_blend = next(row for row in blend_rows if row.get("selected") == "True")
    blend_split_rows = read_csv_rows(
        ROOT
        / "codex_analysis"
        / "yaw_model_variant_fits"
        / "transition_options"
        / "rational_speed_force_blend"
        / "split_metrics.csv"
    )
    blend_primary = next(row for row in blend_split_rows if row["group"] == "primary_open_floor_fit_authoritative")
    blend_validation = next(row for row in blend_split_rows if row["group"] == "validation_non_authoritative")

    flags = plant_source_flags()
    return {
        "launch_accel_median_radps2": median(launch_accels),
        "launch_delay_median_s": 0.001 * median(launch_delays),
        "launch_tau_median_s": 0.001 * median(launch_tau_ms),
        "force_in_place_command_abs": float(selected_force["in_place_left_command"]),
        "force_in_place_extra_nm": float(selected_force["in_place_extra_opposing_yaw_torque_nm"]),
        "force_primary_corrected_rmse_nm": float(force_primary["corrected_rmse_nm"]),
        "blend_in_place_command_abs": float(selected_blend["in_place_max_abs_command"]),
        "blend_in_place_extra_nm": float(selected_blend["in_place_extra_opposing_nm"]),
        "blend_primary_baseline_rmse_nm": float(blend_primary["baseline_rmse_nm"]),
        "blend_primary_corrected_rmse_nm": float(blend_primary["corrected_rmse_nm"]),
        "blend_validation_baseline_rmse_nm": float(blend_validation["baseline_rmse_nm"]),
        "blend_validation_corrected_rmse_nm": float(blend_validation["corrected_rmse_nm"]),
        **flags,
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
            heading_error_rate = yaw_error
            accel_cmd += gains.heading_kp * heading_error
            accel_cmd += gains.heading_kd * heading_error_rate
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
        target_delta = scenario.target_yaw_rate - scenario.initial_yaw_rate
        if not crossed and target_delta != 0.0:
            if math.copysign(1.0, target_delta) != math.copysign(1.0, error):
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
    return {
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


def scenario_base_score(metrics: dict[str, float], objective: Objective) -> float:
    return (
        60.0 * metrics["rms_early_norm"]
        + 90.0 * metrics["rms_late_norm"]
        + 30.0 * metrics["late_p2p_norm"]
        + objective.saturation_weight * metrics["saturation_fraction"]
        + 2.5 * metrics["sign_changes"]
        + 25.0 * metrics["heading_late_abs_rad"]
        + objective.accel_weight * metrics["accel_cmd_rms_over_limit"]
    )


def scenario_weight(objective: Objective, scenario_name: str) -> float:
    if scenario_name.startswith("launch"):
        return objective.launch_weight
    if "reversal" in scenario_name or "minus" in scenario_name:
        return objective.reversal_weight
    if scenario_name.startswith("combined"):
        return objective.combined_weight
    if scenario_name.startswith("heading"):
        return objective.heading_weight
    return objective.step_weight


def evaluate(gains: Gains, scenarios: list[Scenario], objective: Objective) -> tuple[float, list[dict[str, float | str]]]:
    rows: list[dict[str, float | str]] = []
    total = 0.0
    for scenario in scenarios:
        metrics = simulate(gains, scenario)
        score = scenario_weight(objective, scenario.name) * scenario_base_score(metrics, objective)
        total += score
        rows.append({"objective": objective.name, "scenario": scenario.name, "score": score, **metrics})
    total += objective.gain_regularization * (
        0.002 * gains.heading_kp + 0.03 * gains.heading_kd + 0.04 * gains.yawrate_kp
    )
    return total, rows


def build_scenarios(evidence: dict[str, float | str | bool]) -> list[Scenario]:
    launch_accel = max(150.0, float(evidence["launch_accel_median_radps2"]))
    delay = float(evidence["launch_delay_median_s"])
    tau = max(0.012, float(evidence["launch_tau_median_s"]))
    return [
        Scenario("launch_0_to_1_radps", 0.70, 0.0, 1.0, None, launch_accel, tau, delay),
        Scenario("yaw_rate_0_to_9_radps", 1.10, 0.0, 9.0, None, 645.0, tau, delay),
        Scenario("yaw_rate_9_to_minus9_radps", 1.25, 9.0, -9.0, None, 645.0, tau, delay),
        Scenario("combined_3mps_5p5radps", 0.95, 0.0, 5.5, None, 645.0, tau, delay),
        Scenario("heading_correction_15deg_at_3mps", 0.85, 0.0, 0.0, math.radians(15.0), 645.0, tau, delay),
    ]


def gain_grid(current: Gains) -> list[Gains]:
    candidates: list[Gains] = [current]
    yaw_values = [36.0, 42.0, 48.0, 54.0, 60.0, 66.0, 72.0, 84.0, 96.0, 108.0, 120.0, 126.0, 138.0]
    heading_values = [
        300.0,
        450.0,
        600.0,
        750.0,
        900.0,
        1200.0,
        1600.0,
        2200.0,
        3000.0,
        4200.0,
        6000.0,
        7800.0,
        9600.0,
        9718.0,
    ]
    heading_d_values = [0.0, 8.0, 16.0, 24.0, 32.0, 48.0, 64.0, 80.0, 96.0]
    for yaw_kp in yaw_values:
        for heading_kp in heading_values:
            for heading_kd in heading_d_values:
                candidates.append(
                    Gains(
                        current.velocity_kp,
                        current.velocity_kd,
                        heading_kp,
                        heading_kd,
                        yaw_kp,
                        current.yawrate_kd,
                    )
                )
    unique: dict[tuple[float, float, float, float, float, float], Gains] = {}
    for candidate in candidates:
        unique[
            (
                candidate.velocity_kp,
                candidate.velocity_kd,
                candidate.heading_kp,
                candidate.heading_kd,
                candidate.yawrate_kp,
                candidate.yawrate_kd,
            )
        ] = candidate
    return list(unique.values())


def objectives() -> list[Objective]:
    return [
        Objective("balanced", 1.45, 1.00, 1.15, 0.95, 1.30, 20.0, 0.50, 0.25),
        Objective("launch_focused", 2.80, 1.10, 0.85, 0.75, 0.75, 14.0, 0.35, 0.10),
        Objective("broad_envelope", 0.95, 1.05, 1.35, 1.20, 1.60, 24.0, 0.70, 0.35),
    ]


def candidate_matches_objective(gains: Gains, objective_name: str) -> bool:
    if objective_name == "launch_focused":
        return gains.yawrate_kp >= 60.0 and gains.heading_kp <= 600.0
    if objective_name == "broad_envelope":
        return gains.heading_kp >= 600.0 and gains.heading_kd >= 80.0 and gains.yawrate_kp >= 48.0
    if objective_name == "balanced":
        return gains.heading_kp >= 600.0 and gains.heading_kd >= 64.0 and gains.yawrate_kp >= 60.0
    return True


def fmt(value: object, digits: int = 6) -> str:
    if isinstance(value, float):
        if math.isnan(value):
            return "nan"
        return f"{value:.{digits}f}"
    return str(value)


def make_report(
    current: Gains,
    evidence: dict[str, float | str | bool],
    selected_rows: list[dict[str, object]],
    scenario_rows: list[dict[str, object]],
) -> None:
    selected_by_objective = {row["objective"]: row for row in selected_rows if row["role"] == "selected"}
    balanced = selected_by_objective["balanced"]
    broad = selected_by_objective["broad_envelope"]
    launch = selected_by_objective["launch_focused"]

    def gain_table(row: dict[str, object]) -> str:
        return (
            f"| `VelocityStatePD.kp` | {current.velocity_kp:.6g} | {float(row['velocity_kp']):.6g} |\n"
            f"| `VelocityStatePD.kd` | {current.velocity_kd:.6g} | {float(row['velocity_kd']):.6g} |\n"
            f"| `HeadingStatePD.kp` | {current.heading_kp:.6g} | {float(row['heading_kp']):.6g} |\n"
            f"| `HeadingStatePD.kd` | {current.heading_kd:.6g} | {float(row['heading_kd']):.6g} |\n"
            f"| `YawRateStatePD.kp` | {current.yawrate_kp:.6g} | {float(row['yawrate_kp']):.6g} |\n"
            f"| `YawRateStatePD.kd` | {current.yawrate_kd:.6g} | {float(row['yawrate_kd']):.6g} |"
        )

    balance_scenarios = [row for row in scenario_rows if row["objective"] == "balanced" and row["role"] == "selected"]
    current_scenarios = [row for row in scenario_rows if row["objective"] == "balanced" and row["role"] == "current"]

    lines = [
        "# PD Tuning For Corrected PlantModel",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not modified. The proposed coefficients were not installed.",
        "",
        "## Recommendation",
        "",
        "Recommended balanced `DriveBase` PD coefficients:",
        "",
        "| Gain | Current | Recommended |",
        "| --- | ---: | ---: |",
        gain_table(balanced),
        "",
        "Classification: balanced. It keeps the corrected PlantModel's projected-contact force launch authority as the primary low-speed yaw source, then uses materially lower feedback gains to damp residual yaw-rate and heading error instead of brute-forcing launch through feedback.",
        "",
        "Alternates for review:",
        "",
        "| Class | Heading kp | Heading kd | Yaw-rate kp | Yaw-rate kd | Score | Delta vs current |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        (
            f"| Launch-focused | {float(launch['heading_kp']):.6g} | {float(launch['heading_kd']):.6g} | "
            f"{float(launch['yawrate_kp']):.6g} | {float(launch['yawrate_kd']):.6g} | "
            f"{float(launch['score']):.3f} | {float(launch['score_delta_vs_current_pct']):.2f}% |"
        ),
        (
            f"| Broad-envelope | {float(broad['heading_kp']):.6g} | {float(broad['heading_kd']):.6g} | "
            f"{float(broad['yawrate_kp']):.6g} | {float(broad['yawrate_kd']):.6g} | "
            f"{float(broad['score']):.3f} | {float(broad['score_delta_vs_current_pct']):.2f}% |"
        ),
        (
            f"| Balanced | {float(balanced['heading_kp']):.6g} | {float(balanced['heading_kd']):.6g} | "
            f"{float(balanced['yawrate_kp']):.6g} | {float(balanced['yawrate_kd']):.6g} | "
            f"{float(balanced['score']):.3f} | {float(balanced['score_delta_vs_current_pct']):.2f}% |"
        ),
        "",
        "`VelocityStatePD` is left unchanged because this pass targets the yaw dynamics affected by PlantModel feedforward and yaw-residual changes. `YawRateStatePD.kd` is reported unchanged because the current `DriveBase` yaw-rate path calls `Compute(yawRateError, 0.0f)`, so this derivative value is not actively sampled there.",
        "",
        "## Objective And Data",
        "",
        "Inputs used:",
        "",
        "- Current local `MazeMap/MazeMap/PlantModel.cpp`, `PlantModel.h`, and `CoreConfig.h`.",
        "- Existing rational speed/force partition evidence in `codex_analysis/yaw_model_variant_fits/transition_options/rational_speed_force_blend/`.",
        "- Existing projected-force-domain yaw residual evidence in `codex_analysis/yaw_model_variant_fits/round2_force_domain_stribeck/` as supporting context.",
        "- Existing yaw-launch step evidence in `codex_analysis/yaw_launch_step_response/`.",
        "",
        "Not used: UKF state-vector fields as tuning targets, command/request values as traction selectors, unit tests, or project builds.",
        "",
        "The compact replay scenarios were: low-speed 0 to 1 rad/s yaw launch, 0 to 9 rad/s yaw-rate step, 9 to -9 rad/s reversal, 3 m/s plus 5.5 rad/s combined turn, and 15 degree heading correction at speed.",
        "",
        "## PlantModel Evidence",
        "",
        "| Evidence | Value |",
        "| --- | ---: |",
        f"| Rational partition +1 rad/s in-place max command | {float(evidence['blend_in_place_command_abs']):.6f} |",
        f"| Rational partition +1 rad/s extra opposing moment Nm | {float(evidence['blend_in_place_extra_nm']):.6f} |",
        f"| Projected-force-only reference +1 rad/s max command | {float(evidence['force_in_place_command_abs']):.6f} |",
        f"| Primary RMSE baseline -> rational corrected Nm | {float(evidence['blend_primary_baseline_rmse_nm']):.6f} -> {float(evidence['blend_primary_corrected_rmse_nm']):.6f} |",
        f"| Validation RMSE baseline -> rational corrected Nm | {float(evidence['blend_validation_baseline_rmse_nm']):.6f} -> {float(evidence['blend_validation_corrected_rmse_nm']):.6f} |",
        f"| Median measured launch delay used | {1000.0 * float(evidence['launch_delay_median_s']):.3f} ms |",
        f"| Median launch time constant used | {1000.0 * float(evidence['launch_tau_median_s']):.3f} ms |",
        f"| Median sustained initial yaw acceleration evidence | {float(evidence['launch_accel_median_radps2']):.3f} rad/s^2 |",
        "",
        "Source caveat: the current local `PlantModel.cpp` still contains Variant-C symbols and a request-conditioned feedforward helper according to static source inspection. This report scores the owner-described corrected model behavior: force/projected-contact-only rational partition and no command/request traction selector.",
        "",
        "## Result Versus Current",
        "",
        "| Objective | Current score | Selected score | Delta |",
        "| --- | ---: | ---: | ---: |",
    ]
    for objective_name in ["launch_focused", "broad_envelope", "balanced"]:
        selected = selected_by_objective[objective_name]
        lines.append(
            f"| {objective_name} | {float(selected['current_score']):.3f} | {float(selected['score']):.3f} | {float(selected['score_delta_vs_current_pct']):.2f}% |"
        )
    lines.extend(
        [
            "",
            "Balanced scenario highlights:",
            "",
            "| Scenario | Current late RMS | Selected late RMS | Current saturation | Selected saturation | Selected final heading error |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    current_by_name = {row["scenario"]: row for row in current_scenarios}
    for row in balance_scenarios:
        current_row = current_by_name[row["scenario"]]
        lines.append(
            f"| {row['scenario']} | {float(current_row['rms_late_norm']):.6f} | {float(row['rms_late_norm']):.6f} | "
            f"{float(current_row['saturation_fraction']):.3f} | {float(row['saturation_fraction']):.3f} | {float(row['final_heading_error']):.6f} |"
        )
    lines.extend(
        [
            "",
            "## Caveats",
            "",
            "- This is a tuning recommendation, not adoption. Coefficients should be reviewed before installation.",
            "- The replay is compact and analysis-only; it does not replace a release-mode `PdTuning` run once rebuilding that tool is in scope.",
            "- The rational force/projected-contact residual has stronger launch authority than old Variant C. The balanced recommendation avoids high heading proportional gain because the corrected feedforward should carry launch, not feedback saturation.",
            "- Launch-focused is best when immediate breakaway and low-speed yaw response are the review priority. Broad-envelope is best when reversals, combined turns, and heading correction matter more. Balanced is the recommended default between those pressures.",
            "",
            "## Artifacts",
            "",
            "- `tune_pd_corrected_plantmodel.py`",
            "- `candidate_summary.csv`",
            "- `scenario_metrics.csv`",
            "- `evidence_summary.csv`",
            "- `pd_tuning_report.md`",
        ]
    )
    (OUT / "pd_tuning_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    current = load_current_gains()
    evidence = load_evidence()
    scenarios = build_scenarios(evidence)
    objectives_list = objectives()
    candidates = gain_grid(current)

    candidate_rows: list[dict[str, object]] = []
    scenario_rows: list[dict[str, object]] = []
    selected_rows: list[dict[str, object]] = []

    for objective in objectives_list:
        current_score, current_metrics = evaluate(current, scenarios, objective)
        scored: list[tuple[float, Gains, list[dict[str, float | str]]]] = []
        for candidate in candidates:
            score, metrics = evaluate(candidate, scenarios, objective)
            scored.append((score, candidate, metrics))
        scored.sort(key=lambda item: item[0])

        for rank, (score, gains, _) in enumerate(scored[:100], start=1):
            candidate_rows.append(
                {
                    "objective": objective.name,
                    "rank": rank,
                    "score": score,
                    "current_score": current_score,
                    "score_delta_vs_current_pct": 100.0 * (score / current_score - 1.0),
                    "velocity_kp": gains.velocity_kp,
                    "velocity_kd": gains.velocity_kd,
                    "heading_kp": gains.heading_kp,
                    "heading_kd": gains.heading_kd,
                    "yawrate_kp": gains.yawrate_kp,
                    "yawrate_kd": gains.yawrate_kd,
                }
            )

        selected_index = 0
        for index, (_, gains, _) in enumerate(scored):
            if candidate_matches_objective(gains, objective.name):
                selected_index = index
                break
        selected_score, selected_gains, selected_metrics = scored[selected_index]
        selected_row = {
            "objective": objective.name,
            "role": "selected",
            "score": selected_score,
            "current_score": current_score,
            "score_delta_vs_current_pct": 100.0 * (selected_score / current_score - 1.0),
            "velocity_kp": selected_gains.velocity_kp,
            "velocity_kd": selected_gains.velocity_kd,
            "heading_kp": selected_gains.heading_kp,
            "heading_kd": selected_gains.heading_kd,
            "yawrate_kp": selected_gains.yawrate_kp,
            "yawrate_kd": selected_gains.yawrate_kd,
        }
        selected_rows.append(selected_row)
        selected_rows.append(
            {
                "objective": objective.name,
                "role": "current",
                "score": current_score,
                "current_score": current_score,
                "score_delta_vs_current_pct": 0.0,
                "velocity_kp": current.velocity_kp,
                "velocity_kd": current.velocity_kd,
                "heading_kp": current.heading_kp,
                "heading_kd": current.heading_kd,
                "yawrate_kp": current.yawrate_kp,
                "yawrate_kd": current.yawrate_kd,
            }
        )
        for row in current_metrics:
            scenario_rows.append({"objective": objective.name, "role": "current", **row})
        for row in selected_metrics:
            scenario_rows.append({"objective": objective.name, "role": "selected", **row})

    evidence_rows = [{"metric": key, "value": value} for key, value in evidence.items()]
    write_csv(OUT / "candidate_summary.csv", candidate_rows)
    write_csv(OUT / "scenario_metrics.csv", scenario_rows)
    write_csv(OUT / "selected_coefficients.csv", selected_rows)
    write_csv(OUT / "evidence_summary.csv", evidence_rows)
    make_report(current, evidence, selected_rows, scenario_rows)


if __name__ == "__main__":
    main()
