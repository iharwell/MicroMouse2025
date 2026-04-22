#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean

from competition_feedforward import extract_initializer_entries
from competition_feedforward import evaluate_cpp_expr
from competition_feedforward import load_named_float
from open_floor_recovery import RunPlantParameters
from open_floor_recovery import drive_torque_from_command


@dataclass(frozen=True)
class PivotScrubSetup:
    track_width_m: float
    yaw_inertia_kg_m2: float
    wheel_radius_m: float
    drive: RunPlantParameters
    current_breakaway_yaw_moment_nm: float
    current_rolling_yaw_moment_nm: float
    current_max_forward_speed_mps: float
    current_min_command_yaw_rate_radps: float
    current_breakaway_yaw_rate_radps: float
    current_breakaway_yaw_rate_band_radps: float
    mapped_breakaway_yaw_moment_nm: float
    mapped_rolling_yaw_moment_nm: float


@dataclass(frozen=True)
class PivotScrubSample:
    target_forward_speed_mps: float
    current_forward_speed_mps: float
    target_yaw_rate_radps: float
    current_yaw_rate_radps: float
    resistive_yaw_moment_nm: float


@dataclass(frozen=True)
class PivotScrubFit:
    label: str
    sample_count: int
    rmse_nm: float
    max_forward_speed_mps: float
    min_command_yaw_rate_radps: float
    breakaway_yaw_rate_radps: float
    breakaway_yaw_rate_band_radps: float
    breakaway_yaw_moment_nm: float
    rolling_yaw_moment_nm: float


TURNAROUND_SUFFIXES = ("_turnaround", "_reset_heading")
DIRECT_PIVOT_PREFIXES = ("turn_", "square_")


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Fit PlantModel pivot-scrub feedforward constants from archived competition turn logs."
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=repo_root / "TestResults" / "Competition Testing Data",
        help="Directory containing archived competition diag*.csv logs.",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=repo_root,
        help="Repository root used to read the current authoritative setup.",
    )
    return parser.parse_args()


def smoothstep(edge0: float, edge1: float, value: float) -> float:
    if edge1 <= edge0:
        return 1.0 if value >= edge1 else 0.0
    ratio = max(0.0, min(1.0, (value - edge0) / (edge1 - edge0)))
    return ratio * ratio * (3.0 - (2.0 * ratio))


def load_setup(repo_root: Path) -> PivotScrubSetup:
    vehicle_entries = extract_initializer_entries(
        repo_root / "MazeMap" / "MazeMap" / "Vehicle.h",
        "inline static constexpr VehiclePhysicalModel kPhysicalModel",
    )
    drive_entries = extract_initializer_entries(
        repo_root / "MazeMap" / "MazeMap" / "MotorEncoderDrive.h",
        "inline static constexpr PhysicalModel kSharedPhysicalModel",
    )

    track_width_m = evaluate_cpp_expr(vehicle_entries[5])
    yaw_inertia_kg_m2 = evaluate_cpp_expr(vehicle_entries[3])
    supply_voltage_v = evaluate_cpp_expr(drive_entries[2])
    resistance_ohms = evaluate_cpp_expr(drive_entries[3])
    torque_constant_nm_per_a = evaluate_cpp_expr(drive_entries[4])
    no_load_current_a = evaluate_cpp_expr(drive_entries[5])
    speed_constant_radps_per_volt = evaluate_cpp_expr(drive_entries[6])
    gear_ratio = evaluate_cpp_expr(drive_entries[7])
    wheel_radius_m = 0.5 * evaluate_cpp_expr(drive_entries[8])

    plant_header = repo_root / "MazeMap" / "MazeMap" / "PlantModel.h"
    plant_cpp = repo_root / "MazeMap" / "MazeMap" / "PlantModel.cpp"
    reliable_launch_drive_command = load_named_float(plant_cpp, "kReliableLaunchDriveCommand")
    rolling_friction_torque_nm = load_named_float(plant_header, "rollingFrictionTorqueNm")

    motor_current_limit_a = supply_voltage_v / resistance_ohms if resistance_ohms > 0.0 else 0.0
    breakaway_current_a = reliable_launch_drive_command * motor_current_limit_a
    if motor_current_limit_a > 0.0:
        breakaway_current_a = max(-motor_current_limit_a, min(motor_current_limit_a, breakaway_current_a))
    breakaway_load_current_a = max(0.0, breakaway_current_a - no_load_current_a)
    mapped_breakaway_torque_nm = torque_constant_nm_per_a * breakaway_load_current_a * gear_ratio
    mapped_breakaway_yaw_moment_nm = (
        track_width_m * (mapped_breakaway_torque_nm / wheel_radius_m)
        if wheel_radius_m > 0.0
        else 0.0
    )
    mapped_rolling_yaw_moment_nm = (
        track_width_m * (rolling_friction_torque_nm / wheel_radius_m)
        if wheel_radius_m > 0.0
        else 0.0
    )

    return PivotScrubSetup(
        track_width_m=track_width_m,
        yaw_inertia_kg_m2=yaw_inertia_kg_m2,
        wheel_radius_m=wheel_radius_m,
        drive=RunPlantParameters(
            battery_voltage_v=supply_voltage_v,
            drive_resistance_ohms=resistance_ohms,
            torque_constant_nm_per_a=torque_constant_nm_per_a,
            speed_constant_radps_per_volt=speed_constant_radps_per_volt,
            no_load_current_a=no_load_current_a,
            gear_ratio=gear_ratio,
            wheel_radius_m=wheel_radius_m,
            nominal_track_width_m=track_width_m,
        ),
        current_breakaway_yaw_moment_nm=load_named_float(plant_header, "pivotScrubBreakawayYawMomentNm"),
        current_rolling_yaw_moment_nm=load_named_float(plant_header, "pivotScrubRollingYawMomentNm"),
        current_max_forward_speed_mps=load_named_float(plant_header, "pivotScrubMaxForwardSpeedMps"),
        current_min_command_yaw_rate_radps=load_named_float(plant_header, "pivotScrubMinCommandYawRateRadps"),
        current_breakaway_yaw_rate_radps=load_named_float(plant_header, "pivotScrubBreakawayYawRateRadps"),
        current_breakaway_yaw_rate_band_radps=load_named_float(plant_header, "pivotScrubBreakawayYawRateBandRadps"),
        mapped_breakaway_yaw_moment_nm=mapped_breakaway_yaw_moment_nm,
        mapped_rolling_yaw_moment_nm=mapped_rolling_yaw_moment_nm,
    )


def phase_matches(phase_name: str, include_turnarounds: bool) -> bool:
    lower = phase_name.lower()
    if lower.startswith(DIRECT_PIVOT_PREFIXES):
        return True
    return include_turnarounds and lower.endswith(TURNAROUND_SUFFIXES)


def collect_samples(root: Path, setup: PivotScrubSetup, include_turnarounds: bool) -> list[PivotScrubSample]:
    samples: list[PivotScrubSample] = []
    for csv_path in sorted(root.glob("diag*.csv")):
        phase_names: dict[int, str] = {}
        header: list[str] | None = None
        current_phase_key: tuple[str, str] | None = None
        current_rows: list[dict[str, str]] = []
        with csv_path.open(newline="", encoding="utf-8", errors="replace") as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
                if not row:
                    continue
                if row[0] == "# phase" and len(row) >= 4:
                    phase_names[int(row[1])] = row[3]
                    continue
                if row[0] == "sample":
                    header = row
                    continue
                if header is None or row[0].startswith("#"):
                    continue

                row_dict = dict(zip(header, row))
                phase_name = phase_names.get(int(row_dict["phase_id"]), "")
                if not phase_matches(phase_name, include_turnarounds):
                    continue

                target_forward_speed_mps = abs(float(row_dict["cmd_linear_mps"]))
                current_forward_speed_mps = abs(float(row_dict["linear_speed_mps"]))
                left_drive_cmd = float(row_dict["left_drive_cmd"])
                right_drive_cmd = float(row_dict["right_drive_cmd"])
                target_yaw_rate_radps = abs(float(row_dict["cmd_angular_radps"]))
                if (
                    (target_forward_speed_mps > 0.005) or
                    (left_drive_cmd * right_drive_cmd >= 0.0) or
                    (target_yaw_rate_radps < 0.8)
                ):
                    continue

                phase_key = (csv_path.name, phase_name)
                if current_phase_key != phase_key:
                    if len(current_rows) >= 3:
                        samples.extend(build_phase_samples(current_rows, setup))
                    current_phase_key = phase_key
                    current_rows = []
                current_rows.append(row_dict)
        if len(current_rows) >= 3:
            samples.extend(build_phase_samples(current_rows, setup))
    return samples


def build_phase_samples(rows: list[dict[str, str]], setup: PivotScrubSetup) -> list[PivotScrubSample]:
    phase_samples: list[PivotScrubSample] = []
    for index in range(1, len(rows) - 1):
        previous = rows[index - 1]
        row = rows[index]
        next_row = rows[index + 1]

        dt_seconds = max(
            1.0e-6,
            (float(previous["dt_us"]) + float(row["dt_us"])) * 1.0e-6,
        )
        previous_abs_yaw_rate_radps = abs(float(previous["angular_speed_radps"]))
        current_abs_yaw_rate_radps = abs(float(row["angular_speed_radps"]))
        next_abs_yaw_rate_radps = abs(float(next_row["angular_speed_radps"]))
        yaw_accel_mag_radps2 = max(
            0.0,
            (next_abs_yaw_rate_radps - previous_abs_yaw_rate_radps) / dt_seconds,
        )

        left_drive_cmd = float(row["left_drive_cmd"])
        right_drive_cmd = float(row["right_drive_cmd"])
        left_wheel_speed_radps = float(row["left_velocity_mps"]) / setup.wheel_radius_m
        right_wheel_speed_radps = float(row["right_velocity_mps"]) / setup.wheel_radius_m
        left_torque_nm = drive_torque_from_command(left_drive_cmd, left_wheel_speed_radps, setup.drive)
        right_torque_nm = drive_torque_from_command(right_drive_cmd, right_wheel_speed_radps, setup.drive)
        drive_yaw_moment_nm = abs(
            0.5
            * setup.track_width_m
            * ((left_torque_nm / setup.wheel_radius_m) - (right_torque_nm / setup.wheel_radius_m))
        )
        resistive_yaw_moment_nm = drive_yaw_moment_nm - (setup.yaw_inertia_kg_m2 * yaw_accel_mag_radps2)

        phase_samples.append(
            PivotScrubSample(
                target_forward_speed_mps=abs(float(row["cmd_linear_mps"])),
                current_forward_speed_mps=abs(float(row["linear_speed_mps"])),
                target_yaw_rate_radps=abs(float(row["cmd_angular_radps"])),
                current_yaw_rate_radps=current_abs_yaw_rate_radps,
                resistive_yaw_moment_nm=resistive_yaw_moment_nm,
            )
        )
    return phase_samples


def solve_moments(
    samples: list[PivotScrubSample],
    max_forward_speed_mps: float,
    min_command_yaw_rate_radps: float,
    breakaway_yaw_rate_radps: float,
    breakaway_yaw_rate_band_radps: float,
    track_width_m: float,
) -> tuple[float, float, float]:
    sum_x0x0 = 0.0
    sum_x0x1 = 0.0
    sum_x1x1 = 0.0
    sum_x0y = 0.0
    sum_x1y = 0.0
    gating_terms: list[tuple[float, float, float]] = []
    for sample in samples:
        command_yaw_gate = smoothstep(
            min_command_yaw_rate_radps,
            min_command_yaw_rate_radps + min_command_yaw_rate_radps,
            sample.target_yaw_rate_radps,
        )
        pivot_regime_gate = smoothstep(
            0.0,
            max_forward_speed_mps,
            (0.5 * track_width_m * sample.target_yaw_rate_radps) - sample.target_forward_speed_mps,
        )
        gate = command_yaw_gate * pivot_regime_gate
        rotating_blend = smoothstep(
            breakaway_yaw_rate_radps,
            breakaway_yaw_rate_radps + breakaway_yaw_rate_band_radps,
            sample.current_yaw_rate_radps,
        )
        x0 = gate * (1.0 - rotating_blend)
        x1 = gate * rotating_blend
        gating_terms.append((x0, x1, sample.resistive_yaw_moment_nm))
        sum_x0x0 += x0 * x0
        sum_x0x1 += x0 * x1
        sum_x1x1 += x1 * x1
        sum_x0y += x0 * sample.resistive_yaw_moment_nm
        sum_x1y += x1 * sample.resistive_yaw_moment_nm

    determinant = (sum_x0x0 * sum_x1x1) - (sum_x0x1 * sum_x0x1)
    if determinant <= 1.0e-12:
        return 0.0, 0.0, float("inf")

    breakaway_yaw_moment_nm = max(
        0.0,
        ((sum_x0y * sum_x1x1) - (sum_x1y * sum_x0x1)) / determinant,
    )
    rolling_yaw_moment_nm = max(
        0.0,
        ((sum_x0x0 * sum_x1y) - (sum_x0x1 * sum_x0y)) / determinant,
    )
    errors = [
        resistive_yaw_moment_nm - ((x0 * breakaway_yaw_moment_nm) + (x1 * rolling_yaw_moment_nm))
        for x0, x1, resistive_yaw_moment_nm in gating_terms
    ]
    rmse_nm = math.sqrt(sum(error * error for error in errors) / len(errors)) if errors else float("inf")
    return breakaway_yaw_moment_nm, rolling_yaw_moment_nm, rmse_nm


def fit_samples(label: str, samples: list[PivotScrubSample], setup: PivotScrubSetup) -> PivotScrubFit:
    best_fit: PivotScrubFit | None = None
    for min_command_yaw_rate_radps in (0.75, 1.0, 1.25):
        for breakaway_yaw_rate_radps in (1.5, 2.0, 2.5):
            for breakaway_yaw_rate_band_radps in (0.75, 1.0, 1.5):
                breakaway_yaw_moment_nm, rolling_yaw_moment_nm, rmse_nm = solve_moments(
                    samples,
                    setup.current_max_forward_speed_mps,
                    min_command_yaw_rate_radps,
                    breakaway_yaw_rate_radps,
                    breakaway_yaw_rate_band_radps,
                    setup.track_width_m,
                )
                candidate = PivotScrubFit(
                    label=label,
                    sample_count=len(samples),
                    rmse_nm=rmse_nm,
                    max_forward_speed_mps=setup.current_max_forward_speed_mps,
                    min_command_yaw_rate_radps=min_command_yaw_rate_radps,
                    breakaway_yaw_rate_radps=breakaway_yaw_rate_radps,
                    breakaway_yaw_rate_band_radps=breakaway_yaw_rate_band_radps,
                    breakaway_yaw_moment_nm=breakaway_yaw_moment_nm,
                    rolling_yaw_moment_nm=rolling_yaw_moment_nm,
                )
                if best_fit is None or (
                    candidate.rmse_nm,
                    abs(candidate.min_command_yaw_rate_radps - setup.current_min_command_yaw_rate_radps),
                    abs(candidate.breakaway_yaw_rate_radps - setup.current_breakaway_yaw_rate_radps),
                    abs(candidate.breakaway_yaw_rate_band_radps - setup.current_breakaway_yaw_rate_band_radps),
                ) < (
                    best_fit.rmse_nm,
                    abs(best_fit.min_command_yaw_rate_radps - setup.current_min_command_yaw_rate_radps),
                    abs(best_fit.breakaway_yaw_rate_radps - setup.current_breakaway_yaw_rate_radps),
                    abs(best_fit.breakaway_yaw_rate_band_radps - setup.current_breakaway_yaw_rate_band_radps),
                ):
                    best_fit = candidate
    assert best_fit is not None
    return best_fit


def print_fit(fit: PivotScrubFit) -> None:
    print(fit.label)
    print(
        "  "
        f"samples={fit.sample_count}, "
        f"rmse_nm={fit.rmse_nm:.6f}, "
        f"pivot_scrub_breakaway_yaw_moment_nm={fit.breakaway_yaw_moment_nm:.6f}, "
        f"pivot_scrub_rolling_yaw_moment_nm={fit.rolling_yaw_moment_nm:.6f}"
    )
    print(
        "  "
        f"pivot_scrub_max_forward_speed_mps={fit.max_forward_speed_mps:.6f}, "
        f"pivot_scrub_min_command_yaw_rate_radps={fit.min_command_yaw_rate_radps:.6f}, "
        f"pivot_scrub_breakaway_yaw_rate_radps={fit.breakaway_yaw_rate_radps:.6f}, "
        f"pivot_scrub_breakaway_yaw_rate_band_radps={fit.breakaway_yaw_rate_band_radps:.6f}"
    )


def main() -> int:
    args = parse_args()
    setup = load_setup(args.repo_root.resolve())
    dedicated_samples = collect_samples(args.root.resolve(), setup, include_turnarounds=False)
    all_pivot_samples = collect_samples(args.root.resolve(), setup, include_turnarounds=True)

    print("Competition pivot-scrub fit against current repo geometry and motor constants")
    print(
        "current defaults: "
        f"pivot_scrub_breakaway_yaw_moment_nm={setup.current_breakaway_yaw_moment_nm:.6f}, "
        f"pivot_scrub_rolling_yaw_moment_nm={setup.current_rolling_yaw_moment_nm:.6f}, "
        f"pivot_scrub_max_forward_speed_mps={setup.current_max_forward_speed_mps:.6f}, "
        f"pivot_scrub_min_command_yaw_rate_radps={setup.current_min_command_yaw_rate_radps:.6f}, "
        f"pivot_scrub_breakaway_yaw_rate_radps={setup.current_breakaway_yaw_rate_radps:.6f}, "
        f"pivot_scrub_breakaway_yaw_rate_band_radps={setup.current_breakaway_yaw_rate_band_radps:.6f}"
    )
    print(
        "legacy mapped moments: "
        f"pivot_scrub_breakaway_yaw_moment_nm={setup.mapped_breakaway_yaw_moment_nm:.6f}, "
        f"pivot_scrub_rolling_yaw_moment_nm={setup.mapped_rolling_yaw_moment_nm:.6f}"
    )
    print_fit(fit_samples("dedicated turn_* and square_* phases:", dedicated_samples, setup))
    all_pivot_fit = fit_samples("all pure-pivot phases including turnaround/reset:", all_pivot_samples, setup)
    print_fit(all_pivot_fit)
    persistent_scrub_yaw_moment_nm = 0.5 * (
        all_pivot_fit.breakaway_yaw_moment_nm + all_pivot_fit.rolling_yaw_moment_nm
    )
    print(
        "recommended constants-only approximation: "
        "if the archived data is better explained by persistent pivot scrub than by a distinct "
        "breakaway-vs-rolling split, collapse the two yaw moments toward a shared value near "
        f"{persistent_scrub_yaw_moment_nm:.6f} Nm and keep the current pivot-regime blend width."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
