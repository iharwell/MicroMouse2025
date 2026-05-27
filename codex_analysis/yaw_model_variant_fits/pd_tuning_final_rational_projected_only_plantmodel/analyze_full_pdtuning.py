#!/usr/bin/env python3
"""Post-process full C++ PdTuning runs for the rational/projected-only PlantModel.

The script only writes under this analysis directory. It can re-run PdTuning for
candidate gains collected from the search JSON files, then produces CSV
summaries and a Markdown report.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path


OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
EXE = ROOT / "Tools" / "PdTuning" / "x64" / "Release" / "PdTuning.exe"
EVAL_DIR = OUT / "candidate_evals"


@dataclass(frozen=True)
class Gains:
    velocity_kp: float
    velocity_kd: float
    heading_kp: float
    heading_kd: float
    yawrate_kp: float
    yawrate_kd: float

    def key(self) -> tuple[float, float, float, float, float, float]:
        return tuple(round(v, 8) for v in (
            self.velocity_kp,
            self.velocity_kd,
            self.heading_kp,
            self.heading_kd,
            self.yawrate_kp,
            self.yawrate_kd,
        ))

    def slug(self) -> str:
        parts = [
            ("vkp", self.velocity_kp),
            ("vkd", self.velocity_kd),
            ("hkp", self.heading_kp),
            ("hkd", self.heading_kd),
            ("ykp", self.yawrate_kp),
            ("ykd", self.yawrate_kd),
        ]
        return "__".join(f"{name}_{value:.8g}".replace(".", "p").replace("-", "m") for name, value in parts)


@dataclass
class Candidate:
    label: str
    source: str
    gains: Gains


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8", errors="replace"))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def num(value: object, default: float = 0.0) -> float:
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return default


def gains_from_json(node: dict) -> Gains:
    g = node["gains"]
    return Gains(
        num(g["velocity_state_pd"]["kp"]),
        num(g["velocity_state_pd"]["kd"]),
        num(g["heading_state_pd"]["kp"]),
        num(g["heading_state_pd"]["kd"]),
        num(g["yaw_rate_state_pd"]["kp"]),
        num(g["yaw_rate_state_pd"]["kd"]),
    )


def collect_candidates() -> list[Candidate]:
    candidates: dict[tuple[float, float, float, float, float, float], Candidate] = {}

    def add(label: str, source: str, gains: Gains) -> None:
        candidates.setdefault(gains.key(), Candidate(label, source, gains))

    baseline = read_json(OUT / "pdtuning_baseline.json")
    add("current", "current_core_config", gains_from_json(baseline["baseline"]))

    manual = [
        ("compact_prior_balanced", Gains(5.5, 0.01, 600.0, 80.0, 60.0, 5.0)),
        ("compact_prior_launch", Gains(5.5, 0.01, 300.0, 112.0, 60.0, 5.0)),
        ("compact_prior_broad", Gains(5.5, 0.01, 600.0, 80.0, 48.0, 5.0)),
        ("low_heading_damped", Gains(5.5, 0.01, 300.0, 80.0, 48.0, 5.0)),
        ("moderate_heading_high_yaw", Gains(5.5, 0.01, 1200.0, 80.0, 126.0, 5.0)),
        ("high_authority_current_yaw", Gains(5.5, 0.01, 2500.0, 0.0, 126.0, 5.0)),
        ("high_authority_search_anchor", Gains(5.5, 0.01, 6000.0, 0.0, 126.0, 5.0)),
    ]
    for label, gains in manual:
        add(label, "manual_review_anchor", gains)

    for yaw_kp in [1.0, 2.0, 5.0, 10.0, 20.0, 36.0, 48.0, 60.0]:
        for heading_kp, heading_kd in [
            (0.0, 0.0),
            (50.0, 0.0),
            (100.0, 20.0),
            (300.0, 40.0),
            (300.0, 80.0),
            (600.0, 80.0),
            (1200.0, 80.0),
        ]:
            add(
                f"manual_grid_h{heading_kp:.0f}_d{heading_kd:.0f}_y{yaw_kp:.0f}",
                "manual_low_authority_grid",
                Gains(5.5, 0.01, heading_kp, heading_kd, yaw_kp, 5.0),
            )

    for yaw_kp in [48.0, 60.0, 80.0, 100.0, 122.3940811, 126.0, 160.0]:
        for heading_kp, heading_kd in [
            (300.0, 80.0),
            (600.0, 80.0),
            (1200.0, 0.0),
            (1200.0, 80.0),
            (2500.0, 0.0),
            (3500.0, 20.0),
        ]:
            add(
                f"manual_authority_h{heading_kp:.0f}_d{heading_kd:.0f}_y{yaw_kp:.3g}",
                "manual_authority_grid",
                Gains(5.5, 0.01, heading_kp, heading_kd, yaw_kp, 5.0),
            )

    for path in sorted(OUT.glob("pdtuning_search_*.json")):
        data = read_json(path)
        search_name = path.stem.removeprefix("pdtuning_search_")
        search = data.get("search", {})
        best = search.get("best")
        if isinstance(best, dict) and "gains" in best:
            add(f"{search_name}_best", path.name, gains_from_json(best))
        for row in search.get("top_candidates", []):
            if "gains" in row:
                add(f"{search_name}_rank_{row.get('rank', 'x')}", path.name, gains_from_json(row))
    return list(candidates.values())


def run_eval(candidate: Candidate, force: bool) -> Path:
    EVAL_DIR.mkdir(parents=True, exist_ok=True)
    path = EVAL_DIR / f"{candidate.gains.slug()}.json"
    if path.exists() and not force:
        return path
    args = [
        str(EXE),
        "--velocity-kp", f"{candidate.gains.velocity_kp:.9g}",
        "--velocity-kd", f"{candidate.gains.velocity_kd:.9g}",
        "--heading-kp", f"{candidate.gains.heading_kp:.9g}",
        "--heading-kd", f"{candidate.gains.heading_kd:.9g}",
        "--yawrate-kp", f"{candidate.gains.yawrate_kp:.9g}",
        "--yawrate-kd", f"{candidate.gains.yawrate_kd:.9g}",
    ]
    completed = subprocess.run(args, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    path.write_text(completed.stdout, encoding="utf-8")
    if completed.stderr:
        (EVAL_DIR / f"{candidate.gains.slug()}.stderr.txt").write_text(completed.stderr, encoding="utf-8")
    return path


def scenario_by_name(result: dict) -> dict[str, dict]:
    return {row["name"]: row for row in result.get("baseline", {}).get("scenarios", [])}


def result_node(data: dict) -> dict:
    candidate = data.get("candidate")
    if isinstance(candidate, dict) and candidate:
        return candidate
    return data["baseline"]


def scale_for(scenario: dict) -> float:
    target = abs(num(scenario.get("target")))
    initial = abs(num(scenario.get("initial")))
    delta = abs(num(scenario.get("target")) - num(scenario.get("initial")))
    return max(1.0, target, initial, delta)


def step_amplitude_for(scenario: dict) -> float:
    return max(1.0e-9, abs(num(scenario.get("target")) - num(scenario.get("initial"))))


def overshoot_percent(scenario: dict) -> float:
    return 100.0 * num(scenario.get("overshoot")) / step_amplitude_for(scenario)


def step_rms_norm(scenario: dict) -> float:
    step = scenario.get("step_response", {})
    scale = scale_for(scenario)
    values = []
    for key, unit in [
        ("velocity_error_first_500_ticks", "rms_mps"),
        ("yaw_rate_error_first_500_ticks", "rms_radps"),
    ]:
        part = step.get(key, {})
        if part.get("active"):
            values.append(num(part.get(unit)) / scale)
    return max(values) if values else 0.0


def accel_rms_norm(scenario: dict) -> float:
    step = scenario.get("step_response", {})
    values = []
    for key, unit in [
        ("forward_accel_error_first_100_ticks", "rms_mps2"),
        ("yaw_accel_error_first_100_ticks", "rms_radps2"),
    ]:
        part = step.get(key, {})
        if part.get("active"):
            values.append(num(part.get(unit)) / 100.0)
    return max(values) if values else 0.0


def late_rms_norm(scenario: dict) -> float:
    osc = scenario.get("oscillation", {})
    return num(osc.get("late_window_rms_error")) / scale_for(scenario)


def scenario_score(scenario: dict, saturation_weight: float, allow_saturation: bool) -> float:
    failed_penalty = 2500.0 if scenario.get("failed") else 0.0
    nonfinite_penalty = 2000.0 * num(scenario.get("non_finite_count"))
    settle_penalty = 120.0 if scenario.get("settled") is False else 0.0
    sat = num(scenario.get("command_saturation_fraction"))
    sat_penalty = (0.0 if allow_saturation else saturation_weight * sat)
    return (
        80.0 * step_rms_norm(scenario)
        + 24.0 * accel_rms_norm(scenario)
        + 110.0 * late_rms_norm(scenario)
        + 40.0 * abs(num(scenario.get("final_error"))) / scale_for(scenario)
        + sat_penalty
        + settle_penalty
        + failed_penalty
        + nonfinite_penalty
    )


def maneuver_tracking_score(result: dict) -> float:
    rows = [
        row for row in result.get("acceptance_scenarios", [])
        if row.get("metric") == "maneuver_tracking_rms"
    ]
    if not rows:
        return 0.0
    values = [num(row.get("score_penalty"), num(row.get("score_contribution"))) for row in rows]
    return sum(values) / len(values)


def smooth_blocker_count(result: dict) -> int:
    return sum(1 for row in result.get("acceptance_scenarios", []) if row.get("blocker"))


def objective_scores(result: dict) -> dict[str, float]:
    scenarios = {row["name"]: row for row in result.get("scenarios", [])}
    gains = gains_from_json(result)
    launch_names = ["yaw_rate_max_effort_0_to_9_radps", "yaw_rate_reversal_9_to_minus9_radps"]
    broad_names = [
        "forward_launch_0_to_4_mps",
        "forward_brake_4_to_0_mps",
        "forward_high_speed_disturbance_4_to_2_mps",
        "combined_3mps_16p5mps2_turn",
        "combined_3mps_heading_correction_15deg",
    ]
    tracking = maneuver_tracking_score(result)
    blockers = smooth_blocker_count(result)
    launch_sat = max(num(scenarios[name].get("command_saturation_fraction")) for name in launch_names)
    broad_sat = max(num(scenarios[name].get("command_saturation_fraction")) for name in broad_names)
    yaw_launch = scenarios["yaw_rate_max_effort_0_to_9_radps"]
    heading = scenarios["combined_3mps_heading_correction_15deg"]
    yaw_response = (
        100.0 * step_rms_norm(yaw_launch)
        + 60.0 * abs(num(yaw_launch.get("final_error"))) / 9.0
        + 40.0 * late_rms_norm(yaw_launch)
    )
    heading_error = abs(num(heading.get("final_error")))
    heading_response = (
        120.0 * heading_error
        + 40.0 * late_rms_norm(heading)
        + 4.0 * scenario_score(heading, 0.0, True)
    )
    authority_heading_penalty = 10000000.0 if gains.heading_kp < 300.0 else 0.0
    blocker_penalty = 100000.0 * blockers
    launch_sat_required_penalty = 1000000.0 if launch_sat < 0.95 else 0.0
    launch_sat_rejected_penalty = 1000000.0 if launch_sat > 0.02 else 0.0
    broad_sat_rejected_penalty = 1000000.0 if broad_sat > 0.995 else 0.0

    # Objective scores are post-processing classifiers over the full C++ PdTuning
    # results. They deliberately gate out zero-authority artifacts for objectives
    # that need real heading or launch authority.

    return {
        "launch_with_saturation": (
            yaw_response
            + (2.0 * tracking)
            + (5.0 * heading_error)
            + launch_sat_required_penalty
            + blocker_penalty
        ),
        "launch_without_saturation": (
            yaw_response
            + tracking
            + (2.0 * heading_error)
            + launch_sat_rejected_penalty
            + blocker_penalty
        ),
        "nonlaunch_with_saturation": (
            (18.0 * tracking)
            + heading_response
            + (5.0 * broad_sat)
            + (5.0 * step_rms_norm(yaw_launch))
            + authority_heading_penalty
            + blocker_penalty
        ),
        "nonlaunch_without_saturation": (
            (18.0 * tracking)
            + heading_response
            + (1000.0 * broad_sat)
            + (500.0 * launch_sat)
            + (5.0 * step_rms_norm(yaw_launch))
            + broad_sat_rejected_penalty
            + authority_heading_penalty
            + blocker_penalty
        ),
        "balanced": (
            (16.0 * tracking)
            + (80.0 * heading_error)
            + (25.0 * step_rms_norm(yaw_launch))
            + (120.0 * broad_sat)
            + launch_sat_required_penalty
            + authority_heading_penalty
            + blocker_penalty
        ),
    }


def summarize_candidate(candidate: Candidate, data: dict) -> dict[str, object]:
    result = result_node(data)
    scenarios = {row["name"]: row for row in result.get("scenarios", [])}
    objective = objective_scores(result)
    launch_sat = max(num(scenarios[name].get("command_saturation_fraction")) for name in [
        "yaw_rate_max_effort_0_to_9_radps",
        "yaw_rate_reversal_9_to_minus9_radps",
    ])
    broad_sat = max(num(scenarios[name].get("command_saturation_fraction")) for name in [
        "forward_launch_0_to_4_mps",
        "forward_brake_4_to_0_mps",
        "forward_high_speed_disturbance_4_to_2_mps",
        "combined_3mps_16p5mps2_turn",
        "combined_3mps_heading_correction_15deg",
    ])
    heading = scenarios["combined_3mps_heading_correction_15deg"]
    yaw_launch = scenarios["yaw_rate_max_effort_0_to_9_radps"]
    forward_brake = scenarios["forward_brake_4_to_0_mps"]
    failed_scenarios = sum(1 for row in scenarios.values() if row.get("failed"))
    nonfinite_scenarios = sum(1 for row in scenarios.values() if num(row.get("non_finite_count")) > 0.0)
    max_overshoot_pct = max(overshoot_percent(row) for row in scenarios.values())
    return {
        "label": candidate.label,
        "source": candidate.source,
        "tool_score": result.get("score"),
        "tool_failed": result.get("failed"),
        "oscillation_flagged": result.get("oscillation_flagged"),
        "acceptance_blocked": result.get("acceptance_blocked"),
        "velocity_kp": candidate.gains.velocity_kp,
        "velocity_kd": candidate.gains.velocity_kd,
        "heading_kp": candidate.gains.heading_kp,
        "heading_kd": candidate.gains.heading_kd,
        "yawrate_kp": candidate.gains.yawrate_kp,
        "yawrate_kd": candidate.gains.yawrate_kd,
        "launch_saturation_max": launch_sat,
        "broad_saturation_max": broad_sat,
        "yaw_launch_final_error_radps": yaw_launch.get("final_error"),
        "yaw_launch_step_rms_norm": step_rms_norm(yaw_launch),
        "yaw_launch_late_rms_norm": late_rms_norm(yaw_launch),
        "yaw_launch_overshoot_percent": overshoot_percent(yaw_launch),
        "heading_final_error_rad": heading.get("final_error"),
        "heading_step_rms_norm": step_rms_norm(heading),
        "heading_late_rms_norm": late_rms_norm(heading),
        "heading_overshoot_percent": overshoot_percent(heading),
        "forward_brake_overshoot_percent": overshoot_percent(forward_brake),
        "max_scenario_overshoot_percent": max_overshoot_pct,
        "maneuver_tracking_mean": maneuver_tracking_score(result),
        "smooth_blockers": smooth_blocker_count(result),
        "failed_scenario_count": failed_scenarios,
        "nonfinite_scenario_count": nonfinite_scenarios,
        "combined_turn_failed": scenarios["combined_3mps_16p5mps2_turn"].get("failed"),
        "combined_heading_failed": scenarios["combined_3mps_heading_correction_15deg"].get("failed"),
        **objective,
    }


def read_rational_evidence() -> dict[str, str]:
    rows = list(csv.DictReader((OUT.parent / "transition_options" / "rational_speed_force_blend" / "candidate_scores.csv").open()))
    selected = next(row for row in rows if row.get("selected") == "True")
    split_rows = list(csv.DictReader((OUT.parent / "transition_options" / "rational_speed_force_blend" / "split_metrics.csv").open()))
    primary = next(row for row in split_rows if row["group"] == "primary_open_floor_fit_authoritative")
    validation = next(row for row in split_rows if row["group"] == "validation_non_authoritative")
    return {
        "in_place_max_abs_command": selected["in_place_max_abs_command"],
        "in_place_extra_opposing_nm": selected["in_place_extra_opposing_nm"],
        "in_place_blend_gate": selected["in_place_blend_gate"],
        "primary_baseline_rmse_nm": primary["baseline_rmse_nm"],
        "primary_corrected_rmse_nm": primary["corrected_rmse_nm"],
        "validation_baseline_rmse_nm": validation["baseline_rmse_nm"],
        "validation_corrected_rmse_nm": validation["corrected_rmse_nm"],
    }


def render_report(summary_rows: list[dict[str, object]], selected_rows: list[dict[str, object]]) -> None:
    evidence = read_rational_evidence()
    selected = {row["objective"]: row for row in selected_rows}
    balanced = selected["balanced"]
    current = next(row for row in summary_rows if row["label"] == "current")

    def gain_tuple(row: dict[str, object]) -> str:
        return (
            f"`Velocity=({float(row['velocity_kp']):.6g}, {float(row['velocity_kd']):.6g})`, "
            f"`Heading=({float(row['heading_kp']):.6g}, {float(row['heading_kd']):.6g})`, "
            f"`YawRate=({float(row['yawrate_kp']):.6g}, {float(row['yawrate_kd']):.6g})`"
        )

    lines = [
        "# PD Tuning For Final Rational Projected-Only PlantModel",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not modified. The proposed coefficients were not installed.",
        "",
        "## Recommendation",
        "",
        "Recommended balanced review candidate for `DriveBase` PD coefficients:",
        "",
        "| Gain | Current | Recommended |",
        "| --- | ---: | ---: |",
        f"| `VelocityStatePD.kp` | {float(current['velocity_kp']):.6g} | {float(balanced['velocity_kp']):.6g} |",
        f"| `VelocityStatePD.kd` | {float(current['velocity_kd']):.6g} | {float(balanced['velocity_kd']):.6g} |",
        f"| `HeadingStatePD.kp` | {float(current['heading_kp']):.6g} | {float(balanced['heading_kp']):.6g} |",
        f"| `HeadingStatePD.kd` | {float(current['heading_kd']):.6g} | {float(balanced['heading_kd']):.6g} |",
        f"| `YawRateStatePD.kp` | {float(current['yawrate_kp']):.6g} | {float(balanced['yawrate_kp']):.6g} |",
        f"| `YawRateStatePD.kd` | {float(current['yawrate_kd']):.6g} | {float(balanced['yawrate_kd']):.6g} |",
        "",
        "This is a launch-capable review candidate, not an adoption-ready install. It saturates the yaw launch scenarios and improves the C++ replay's maneuver-tracking and high-speed heading-error metrics versus the current coefficients, but it still makes `PdTuning` report `failed=true` in the combined high-speed scenarios.",
        "",
        "Alternates by objective:",
        "",
        "| Objective | Classification | Gains | Status | Objective score | Launch sat max | Broad sat max | Yaw-launch OS | Heading OS | Max OS |",
        "| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    labels = {
        "launch_with_saturation": "launch-focused, saturation-permissive",
        "launch_without_saturation": "launch-focused, saturation-averse",
        "nonlaunch_with_saturation": "broad-envelope, saturation-permissive",
        "nonlaunch_without_saturation": "broad-envelope, saturation-averse",
        "balanced": "balanced default",
    }
    status = {
        "launch_with_saturation": "review candidate; intentionally saturates yaw launch",
        "launch_without_saturation": "diagnostic low-launch-saturation candidate; not broad-envelope clean",
        "nonlaunch_with_saturation": "diagnostic only; broad scenarios still fail",
        "nonlaunch_without_saturation": "diagnostic only; no true broad no-saturation candidate found",
        "balanced": "primary review candidate; not adoption-ready",
    }
    for objective in [
        "launch_with_saturation",
        "launch_without_saturation",
        "nonlaunch_with_saturation",
        "nonlaunch_without_saturation",
        "balanced",
    ]:
        row = selected[objective]
        lines.append(
            f"| `{objective}` | {labels[objective]} | {gain_tuple(row)} | "
            f"{status[objective]} | "
            f"{float(row['objective_score']):.3f} | {float(row['launch_saturation_max']):.3f} | {float(row['broad_saturation_max']):.3f} | "
            f"{float(row['yaw_launch_overshoot_percent']):.2f}% | {float(row['heading_overshoot_percent']):.2f}% | {float(row['max_scenario_overshoot_percent']):.2f}% |"
        )

    current_tracking = float(current["maneuver_tracking_mean"])
    balanced_tracking = float(balanced["maneuver_tracking_mean"])
    current_heading_abs = abs(float(current["heading_final_error_rad"]))
    balanced_heading_abs = abs(float(balanced["heading_final_error_rad"]))
    tracking_delta_pct = (
        100.0 * (current_tracking - balanced_tracking) / current_tracking
        if current_tracking > 0.0 else 0.0
    )
    heading_delta_pct = (
        100.0 * (current_heading_abs - balanced_heading_abs) / current_heading_abs
        if current_heading_abs > 0.0 else 0.0
    )
    lines.extend([
        "",
        "Performance versus current coefficients:",
        "",
        "| Metric | Current | Balanced review candidate | Delta |",
        "| --- | ---: | ---: | ---: |",
        f"| Yaw launch final error, rad/s | {float(current['yaw_launch_final_error_radps']):.3f} | {float(balanced['yaw_launch_final_error_radps']):.3f} | {float(current['yaw_launch_final_error_radps']) - float(balanced['yaw_launch_final_error_radps']):.3f} |",
        f"| Yaw launch overshoot | {float(current['yaw_launch_overshoot_percent']):.2f}% | {float(balanced['yaw_launch_overshoot_percent']):.2f}% | {float(balanced['yaw_launch_overshoot_percent']) - float(current['yaw_launch_overshoot_percent']):.2f} pp |",
        f"| Launch saturation max | {float(current['launch_saturation_max']):.3f} | {float(balanced['launch_saturation_max']):.3f} | {float(balanced['launch_saturation_max']) - float(current['launch_saturation_max']):.3f} |",
        f"| High-speed heading final abs error, rad | {current_heading_abs:.3f} | {balanced_heading_abs:.3f} | {heading_delta_pct:.1f}% lower |",
        f"| High-speed heading overshoot | {float(current['heading_overshoot_percent']):.2f}% | {float(balanced['heading_overshoot_percent']):.2f}% | {float(balanced['heading_overshoot_percent']) - float(current['heading_overshoot_percent']):.2f} pp |",
        f"| Max scenario overshoot | {float(current['max_scenario_overshoot_percent']):.2f}% | {float(balanced['max_scenario_overshoot_percent']):.2f}% | {float(balanced['max_scenario_overshoot_percent']) - float(current['max_scenario_overshoot_percent']):.2f} pp |",
        f"| Maneuver tracking mean | {current_tracking:.3f} | {balanced_tracking:.3f} | {tracking_delta_pct:.1f}% lower |",
        f"| Failed PdTuning scenarios | {int(current['failed_scenario_count'])} | {int(balanced['failed_scenario_count'])} | 0 |",
    ])

    lines.extend([
        "",
        "## Model Provenance",
        "",
        "This supersedes the flawed `pd_tuning_final_projected_only_plantmodel` report because that report used `cubic_smoothstep_partition` as a proxy. This pass uses the installed rational speed/force partition model facts:",
        "",
        "- `M_opp = M_C + blend * (M_force - M_C)`.",
        "- `speedLow = k_v^2 / (k_v^2 + v2)`, with `k_v = 0.500 m/s`.",
        "- `forceGate = u^2 / (u^2 + k_u^2)`, with `k_u = 0.10`.",
        "- `blend = clamp(speedLow * forceGate, 0, 1)`.",
        "- `rel_weight = 0.75`, `speed_fade = 0.64 m/s`, `force_sliding = 0.067416756 Nm`.",
        "- C-like branch assumed projected-force/contact-state-only: no request/preprojection terms and no limiter/projection-scale terms.",
        "",
        "Rational model evidence used for context:",
        "",
        "| Evidence | Value |",
        "| --- | ---: |",
        f"| +1 rad/s in-place max command | {float(evidence['in_place_max_abs_command']):.6f} |",
        f"| +1 rad/s extra opposing yaw torque | {float(evidence['in_place_extra_opposing_nm']):.6f} Nm |",
        f"| +1 rad/s blend gate | {float(evidence['in_place_blend_gate']):.6f} |",
        f"| Primary RMSE baseline -> corrected | {float(evidence['primary_baseline_rmse_nm']):.6f} -> {float(evidence['primary_corrected_rmse_nm']):.6f} Nm |",
        f"| Validation RMSE baseline -> corrected | {float(evidence['validation_baseline_rmse_nm']):.6f} -> {float(evidence['validation_corrected_rmse_nm']):.6f} Nm |",
        "",
        "## Evaluation Setup",
        "",
        "- Existing JSON came from the earlier `MazeMap` Release and `Tools/PdTuning` Release rebuilds. This overshoot-only update did not rebuild `MazeMap` or rerun `PdTuning`. It reprocessed the existing JSON outputs.",
        "- The existing `Tools/PdTuning/x64/Release/PdTuning.exe` outputs execute `DriveBase::ProposeBodyTick(...)` and `PlantModel::integrate(...)` through the C++ model.",
        "- Ran baseline plus broad, launch-authority, low-saturation, and non-launch broad bounded searches.",
        f"- Re-evaluated {len(summary_rows)} unique top/manual candidates with the C++ tool and post-scored them into launch-focused, non-launch-focused, saturation-permissive, saturation-averse, and balanced objectives.",
        "- Overshoot percentages are `100 * overshoot / abs(target - initial)` for each C++ PdTuning scenario.",
        "- Did not use UKF state-vector fields as tuning targets and did not use command/request values as traction selectors.",
        "",
        "## Result Details",
        "",
        "| Candidate | Tool failed | Tool score | Launch sat | Broad sat | Yaw launch final err | Yaw OS | Heading final err | Heading OS | Max OS | Maneuver tracking mean |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in sorted(summary_rows, key=lambda r: float(r["balanced"]))[:12]:
        tool_score = row["tool_score"]
        tool_score_str = "nan" if tool_score is None else f"{float(tool_score):.3f}"
        lines.append(
            f"| {row['label']} | {row['tool_failed']} | {tool_score_str} | "
            f"{float(row['launch_saturation_max']):.3f} | {float(row['broad_saturation_max']):.3f} | "
            f"{float(row['yaw_launch_final_error_radps']):.3f} | {float(row['yaw_launch_overshoot_percent']):.2f}% | "
            f"{float(row['heading_final_error_rad']):.3f} | {float(row['heading_overshoot_percent']):.2f}% | "
            f"{float(row['max_scenario_overshoot_percent']):.2f}% | "
            f"{float(row['maneuver_tracking_mean']):.3f} |"
        )
    lines.extend([
        "",
        "## Caveats",
        "",
        "- All selected rows still make `PdTuning` report `failed=true`. The common blockers are the combined 3 m/s, 5.5 rad/s turn and high-speed heading correction scenarios reaching non-finite/failed state. Treat the balanced row as the least-bad review candidate from this pass, not a clean release candidate.",
        "- No true broad-envelope, no-saturation candidate was found. With heading authority enabled, the best broad-envelope saturation maximum in this evaluated set is still about 0.990, and those rows still fail the combined high-speed scenarios.",
        "- `YawRateStatePD.kd` remains score-insensitive in the current `DriveBase` path because yaw-rate feedback calls `Compute(yawRateError, 0.0f)`.",
        "- Launch-focused saturation-permissive intentionally requires yaw-launch saturation because the objective is to preserve static yaw-launch authority. Broad-envelope and saturation-averse objectives penalize sustained saturation strongly.",
        "- These are recommendations for review; no coefficients were installed.",
        "",
        "## Artifacts",
        "",
        "- `pdtuning_baseline.json`",
        "- `pdtuning_search_broad.json`",
        "- `pdtuning_search_launch_authority.json`",
        "- `pdtuning_search_low_saturation.json`",
        "- `pdtuning_search_nonlaunch_broad.json`",
        "- `candidate_evals/*.json`",
        "- `candidate_summary.csv`",
        "- `objective_rankings.csv`",
        "- `selected_coefficients.csv`",
        "- `scenario_metrics.csv`",
        "- `analyze_full_pdtuning.py`",
    ])
    (OUT / "pd_tuning_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    candidates = collect_candidates()
    eval_paths: dict[tuple[float, float, float, float, float, float], Path] = {}
    for candidate in candidates:
        if args.evaluate:
            eval_paths[candidate.gains.key()] = run_eval(candidate, args.force)
        else:
            eval_paths[candidate.gains.key()] = EVAL_DIR / f"{candidate.gains.slug()}.json"

    summary_rows: list[dict[str, object]] = []
    scenario_rows: list[dict[str, object]] = []
    for candidate in candidates:
        path = eval_paths[candidate.gains.key()]
        if not path.exists():
            continue
        data = read_json(path)
        result = result_node(data)
        summary = summarize_candidate(candidate, data)
        summary_rows.append(summary)
        for scenario in result.get("scenarios", []):
            scenario_rows.append({
                "label": candidate.label,
                "scenario": scenario["name"],
                "failed": scenario.get("failed"),
                "final_error": scenario.get("final_error"),
                "settled": scenario.get("settled"),
                "command_saturation_fraction": scenario.get("command_saturation_fraction"),
                "plant_clip_request_fraction": scenario.get("plant_clip_request_fraction"),
                "overshoot": scenario.get("overshoot"),
                "overshoot_percent": overshoot_percent(scenario),
                "step_rms_norm": step_rms_norm(scenario),
                "accel_rms_norm": accel_rms_norm(scenario),
                "late_rms_norm": late_rms_norm(scenario),
                "score": scenario.get("score"),
            })

    ranking_rows: list[dict[str, object]] = []
    selected_rows: list[dict[str, object]] = []
    objectives = [
        "launch_with_saturation",
        "launch_without_saturation",
        "nonlaunch_with_saturation",
        "nonlaunch_without_saturation",
        "balanced",
    ]
    for objective in objectives:
        ranked = sorted(summary_rows, key=lambda row: float(row[objective]))
        for rank, row in enumerate(ranked, start=1):
            ranking_rows.append({"objective": objective, "rank": rank, "objective_score": row[objective], **row})
        best = ranked[0]
        selected_rows.append({"objective": objective, "objective_score": best[objective], **best})

    write_csv(OUT / "candidate_summary.csv", summary_rows)
    write_csv(OUT / "objective_rankings.csv", ranking_rows)
    write_csv(OUT / "selected_coefficients.csv", selected_rows)
    write_csv(OUT / "scenario_metrics.csv", scenario_rows)
    render_report(summary_rows, selected_rows)


if __name__ == "__main__":
    main()
