#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

from competition_feedforward import CompetitionSweepSummary
from competition_feedforward import analyze_competition_feedforward
from competition_feedforward import discover_competition_diag_csvs
from competition_feedforward import estimate_competition_current_plant_parameters
from competition_feedforward import load_current_feedforward_setup


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Compare legacy competition diagnostic sweeps against the current repo feedforward setup."
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=repo_root / "TestResults" / "Competition Testing Data",
        help="Directory containing legacy competition diag*.csv logs.",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=repo_root,
        help="Repository root used to read the current authoritative setup.",
    )
    return parser.parse_args()


def print_sweep_summary(title: str, summary: CompetitionSweepSummary) -> None:
    print(title)
    print(
        f"  source_files={summary.source_file_count}, "
        f"traces={summary.trace_count}"
    )
    if summary.feedforward_alignment is None:
        print("  no qualifying traces found")
        return
    print(
        "  "
        f"command_bins={summary.feedforward_alignment.command_bin_count}, "
        f"samples={summary.feedforward_alignment.sample_count}, "
        f"overall_mean_command_error={summary.feedforward_alignment.overall_mean_command_error:+.6f}, "
        f"overall_rmse_command_error={summary.feedforward_alignment.overall_rmse_command_error:.6f}"
    )
    outcomes_by_command = {
        outcome.abs_command: outcome
        for outcome in summary.outcome_summaries
    }
    for command_summary in summary.feedforward_alignment.command_summaries:
        outcome = outcomes_by_command.get(command_summary.abs_command)
        success_text = "n/a"
        if outcome is not None and outcome.total_count > 0:
            success_text = f"{outcome.success_count}/{outcome.total_count}"
        steady_text = (
            f"{command_summary.steady_required_command_median:.4f}"
            if command_summary.steady_required_command_median is not None
            else "n/a"
        )
        print(
            "  "
            f"cmd={command_summary.abs_command:.2f}, "
            f"success={success_text}, "
            f"required_cmd_p10={command_summary.required_command_p10:.4f}, "
            f"required_cmd_median={command_summary.required_command_median:.4f}, "
            f"required_cmd_p90={command_summary.required_command_p90:.4f}, "
            f"steady_required_cmd_median={steady_text}, "
            f"mean_command_error={command_summary.mean_command_error:+.6f}, "
            f"rmse_command_error={command_summary.rmse_command_error:.6f}"
        )


def main() -> int:
    args = parse_args()
    setup = load_current_feedforward_setup(args.repo_root.resolve())
    csv_paths = discover_competition_diag_csvs(args.root.resolve())
    report = analyze_competition_feedforward(csv_paths, setup)
    estimate = estimate_competition_current_plant_parameters(csv_paths, setup)

    print("Competition feedforward alignment against current repo setup")
    print(
        "current plant: "
        f"effective_longitudinal_mass_kg={setup.launch_fit.effective_longitudinal_mass_kg:.6f}, "
        f"equivalent_wheel_inertia_kg_m2={setup.launch_fit.equivalent_wheel_inertia_kg_m2:.9f}, "
        f"rolling_friction_torque_nm={setup.launch_fit.rolling_friction_torque_nm:.6f}, "
        f"static_friction_torque_nm={setup.launch_fit.static_friction_torque_nm:.6f}, "
        f"static_friction_max_speed_mps={setup.launch_fit.static_friction_max_speed_mps:.6f}, "
        f"viscous_friction_nm_per_radps={setup.launch_fit.viscous_friction_nm_per_radps:.6f}"
    )
    print(
        "current feedforward: "
        f"kWheelStaticFeedforward={setup.wheel_static_feedforward:.6f}, "
        f"kWheelRestLaunchDriveCommand={setup.wheel_rest_launch_drive_command:.6f}, "
        f"plant_breakaway_drive_command={setup.plant_breakaway_drive_command:.6f}, "
        f"kWheelVelocityFeedforward={setup.wheel_velocity_feedforward:.6f}, "
        f"kWheelAccelerationResponseGainPerMps2={setup.wheel_acceleration_response_gain_per_mps2:.6f}, "
        f"kWheelVelocityKp={setup.wheel_velocity_kp:.6f}, "
        f"kWheelVelocityKi={setup.wheel_velocity_ki:.6f}, "
        f"kWheelIntegralLimit={setup.wheel_integral_limit:.6f}"
    )
    print(
        "estimated plant: "
        f"apparent_equivalent_wheel_inertia_kg_m2="
        f"{0.0 if estimate.apparent_equivalent_wheel_inertia_kg_m2 is None else estimate.apparent_equivalent_wheel_inertia_kg_m2:.9f}, "
        f"apparent_rolling_friction_torque_nm="
        f"{0.0 if estimate.apparent_rolling_friction_torque_nm is None else estimate.apparent_rolling_friction_torque_nm:.6f}, "
        f"apparent_viscous_friction_nm_per_radps="
        f"{0.0 if estimate.apparent_viscous_friction_nm_per_radps is None else estimate.apparent_viscous_friction_nm_per_radps:.6f}, "
        f"breakaway_threshold_command_range="
        f"{'n/a' if estimate.breakaway_threshold_lower_command is None else f'{estimate.breakaway_threshold_lower_command:.2f}'}"
        f".."
        f"{'n/a' if estimate.breakaway_threshold_upper_command is None else f'{estimate.breakaway_threshold_upper_command:.2f}'}, "
        f"breakaway_static_friction_midpoint_torque_nm="
        f"{0.0 if estimate.breakaway_static_friction_midpoint_torque_nm is None else estimate.breakaway_static_friction_midpoint_torque_nm:.6f}"
    )
    print(
        "recommended compromise plant: "
        f"equivalent_wheel_inertia_kg_m2={estimate.recommended_equivalent_wheel_inertia_kg_m2:.9f}, "
        f"rolling_friction_torque_nm={estimate.recommended_rolling_friction_torque_nm:.6f}, "
        f"static_friction_torque_nm={estimate.recommended_static_friction_torque_nm:.6f}, "
        f"viscous_friction_nm_per_radps={estimate.recommended_viscous_friction_nm_per_radps:.6f}, "
        f"kickoff_median_rmse={estimate.recommended_kickoff_median_rmse:.6f}, "
        f"forward_steady_median_rmse={estimate.recommended_forward_steady_median_rmse:.6f}"
    )
    print_sweep_summary("kickoff sweep:", report.kickoff)
    print_sweep_summary("forward hold sweep:", report.forward_hold)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
