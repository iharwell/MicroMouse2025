from __future__ import annotations

import csv
import math
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

from open_floor_plant_fit import FeedforwardAlignmentSummary
from open_floor_plant_fit import LaunchFitParameters
from open_floor_plant_fit import LaunchMeanTracePoint
from open_floor_plant_fit import fit_apparent_drive_drag
from open_floor_plant_fit import fit_apparent_equivalent_wheel_inertia
from open_floor_plant_fit import summarize_feedforward_alignment_trace_groups
from open_floor_recovery import RunPlantParameters
from open_floor_recovery import parse_key_value_fields


CPP_FLOAT_PATTERN = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[-+]?\d+)?"
FORWARD_PROBE_PATTERN = re.compile(r"^forward_(\d{3})_probe$")
KICKOFF_PROBE_PATTERN = re.compile(r"^kickoff_(\d{3})_probe$")


@dataclass
class CurrentFeedforwardSetup:
    launch_fit: LaunchFitParameters
    wheel_static_feedforward: float
    wheel_rest_launch_drive_command: float
    wheel_rest_launch_max_drive_command: float
    wheel_rest_launch_ramp_ms: int
    wheel_rest_launch_speed_threshold_mps: float
    wheel_rest_launch_drive_threshold: float
    wheel_velocity_feedforward: float
    wheel_acceleration_response_gain_per_mps2: float
    wheel_velocity_kp: float
    wheel_velocity_ki: float
    wheel_integral_limit: float
    plant_breakaway_drive_command: float


@dataclass
class CommandOutcomeSummary:
    abs_command: float
    success_count: int
    total_count: int


@dataclass
class CompetitionSweepSummary:
    source_file_count: int
    trace_count: int
    outcome_summaries: list[CommandOutcomeSummary]
    feedforward_alignment: FeedforwardAlignmentSummary | None


@dataclass
class CompetitionFeedforwardReport:
    current_setup: CurrentFeedforwardSetup
    kickoff: CompetitionSweepSummary
    forward_hold: CompetitionSweepSummary


@dataclass
class CompetitionTraceCollection:
    kickoff_trace_groups: dict[float, list[list[LaunchMeanTracePoint]]]
    forward_trace_groups: dict[float, list[list[LaunchMeanTracePoint]]]
    kickoff_outcomes: dict[float, list[int]]
    forward_outcomes: dict[float, list[int]]
    source_file_count: int


@dataclass
class CompetitionPlantParameterEstimate:
    apparent_equivalent_wheel_inertia_kg_m2: float | None
    apparent_equivalent_wheel_inertia_trace_count: int
    apparent_rolling_friction_torque_nm: float | None
    apparent_viscous_friction_nm_per_radps: float | None
    apparent_drag_fit_row_count: int
    apparent_drag_fit_residual_sigma_nm: float | None
    breakaway_threshold_lower_command: float | None
    breakaway_threshold_upper_command: float | None
    breakaway_threshold_midpoint_command: float | None
    breakaway_static_friction_lower_torque_nm: float | None
    breakaway_static_friction_midpoint_torque_nm: float | None
    breakaway_static_friction_upper_torque_nm: float | None
    recommended_equivalent_wheel_inertia_kg_m2: float
    recommended_rolling_friction_torque_nm: float
    recommended_static_friction_torque_nm: float
    recommended_viscous_friction_nm_per_radps: float
    recommended_objective: float
    recommended_kickoff_median_rmse: float
    recommended_forward_steady_median_rmse: float


def strip_cpp_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return re.sub(r"//.*", "", text)


def split_top_level_items(text: str) -> list[str]:
    items: list[str] = []
    current: list[str] = []
    paren_depth = 0
    brace_depth = 0
    for character in text:
        if character == "," and paren_depth == 0 and brace_depth == 0:
            item = "".join(current).strip()
            if item:
                items.append(item)
            current = []
            continue
        current.append(character)
        if character == "(":
            paren_depth += 1
        elif character == ")":
            paren_depth -= 1
        elif character == "{":
            brace_depth += 1
        elif character == "}":
            brace_depth -= 1
    item = "".join(current).strip()
    if item:
        items.append(item)
    return items


def extract_initializer_entries(path: Path, marker: str) -> list[str]:
    text = strip_cpp_comments(path.read_text(encoding="utf-8"))
    marker_index = text.find(marker)
    if marker_index < 0:
        raise ValueError(f"could not find initializer marker {marker!r} in {path}")
    brace_start = text.find("{", marker_index)
    if brace_start < 0:
        raise ValueError(f"could not find initializer start for {marker!r} in {path}")
    depth = 0
    brace_end = -1
    for index in range(brace_start, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                brace_end = index
                break
    if brace_end < 0:
        raise ValueError(f"could not find initializer end for {marker!r} in {path}")
    return split_top_level_items(text[brace_start + 1:brace_end])


def milli_amps_to_amps(milliamps: float) -> float:
    return milliamps * 1.0e-3


def milli_newton_meters_to_newton_meters(milli_newton_meters: float) -> float:
    return milli_newton_meters * 1.0e-3


def rpm_to_rad_per_second(rpm: float) -> float:
    return rpm * ((2.0 * math.pi) / 60.0)


def compute_motor_speed_constant_radps_per_volt(
    no_load_speed_rpm: float,
    nominal_voltage_v: float,
    no_load_current_a: float,
    terminal_resistance_ohms: float,
) -> float:
    effective_back_emf_voltage_v = nominal_voltage_v - (no_load_current_a * terminal_resistance_ohms)
    return (rpm_to_rad_per_second(no_load_speed_rpm) / effective_back_emf_voltage_v) if effective_back_emf_voltage_v > 0.0 else 0.0


def evaluate_cpp_expr(expression: str) -> float:
    sanitized = strip_cpp_comments(expression).strip()
    sanitized = re.sub(rf"(?<=\d)(?:f|UL|U|L)\b", "", sanitized)
    return float(
        eval(
            sanitized,
            {"__builtins__": {}},
            {
                "MilliAmpsToAmps": milli_amps_to_amps,
                "MilliNewtonMetersToNewtonMeters": milli_newton_meters_to_newton_meters,
                "ComputeMotorSpeedConstantRadpsPerVolt": compute_motor_speed_constant_radps_per_volt,
                "RpmToRadPerSecond": rpm_to_rad_per_second,
                "PI_F": math.pi,
            },
        )
    )


def load_named_float(path: Path, name: str) -> float:
    text = strip_cpp_comments(path.read_text(encoding="utf-8"))
    match = re.search(rf"\b{name}\b\s*=\s*({CPP_FLOAT_PATTERN})(?:f)?", text, flags=re.IGNORECASE)
    if match is None:
        raise ValueError(f"could not find {name} in {path}")
    return float(match.group(1))


def load_named_int(path: Path, name: str) -> int:
    text = strip_cpp_comments(path.read_text(encoding="utf-8"))
    match = re.search(rf"\b{name}\b\s*=\s*(\d+)(?:UL|U|L)?", text, flags=re.IGNORECASE)
    if match is None:
        raise ValueError(f"could not find {name} in {path}")
    return int(match.group(1))


def load_current_feedforward_setup(repo_root: Path) -> CurrentFeedforwardSetup:
    vehicle_header = repo_root / "MazeMap" / "MazeMap" / "Vehicle.h"
    drive_header = repo_root / "MazeMap" / "MazeMap" / "MotorEncoderDrive.h"
    plant_header = repo_root / "MazeMap" / "MazeMap" / "PlantModel.h"
    plant_cpp = repo_root / "MazeMap" / "MazeMap" / "PlantModel.cpp"
    runtime_core = repo_root / "MazeMap" / "MazeMap" / "MazeMapRuntimeCore.h"

    vehicle_entries = extract_initializer_entries(
        vehicle_header,
        "inline static constexpr VehiclePhysicalModel kPhysicalModel",
    )
    drive_entries = extract_initializer_entries(
        drive_header,
        "inline static constexpr MotorEncoderDrivePhysicalModel kSharedPhysicalModel",
    )

    mass_kg = evaluate_cpp_expr(vehicle_entries[0])
    track_width_m = evaluate_cpp_expr(vehicle_entries[5])
    supply_voltage_v = evaluate_cpp_expr(drive_entries[2])
    drive_resistance_ohms = evaluate_cpp_expr(drive_entries[3])
    torque_constant_nm_per_a = evaluate_cpp_expr(drive_entries[4])
    no_load_current_a = evaluate_cpp_expr(drive_entries[5])
    speed_constant_radps_per_volt = evaluate_cpp_expr(drive_entries[6])
    gear_ratio = evaluate_cpp_expr(drive_entries[7])
    wheel_diameter_m = evaluate_cpp_expr(drive_entries[8])

    wheel_static_feedforward = load_named_float(runtime_core, "kWheelStaticFeedforward")
    wheel_rest_launch_drive_command = load_named_float(runtime_core, "kWheelRestLaunchDriveCommand")
    wheel_rest_launch_max_drive_command = load_named_float(runtime_core, "kWheelRestLaunchMaxDriveCommand")
    wheel_rest_launch_ramp_ms = load_named_int(runtime_core, "kWheelRestLaunchRampMs")
    wheel_rest_launch_speed_threshold_mps = load_named_float(runtime_core, "kWheelRestLaunchSpeedThresholdMps")
    wheel_rest_launch_drive_threshold = load_named_float(runtime_core, "kWheelRestLaunchDriveThreshold")
    wheel_velocity_feedforward = load_named_float(runtime_core, "kWheelVelocityFeedforward")
    wheel_acceleration_response_gain_per_mps2 = load_named_float(runtime_core, "kWheelAccelerationResponseGainPerMps2")
    wheel_velocity_kp = load_named_float(runtime_core, "kWheelVelocityKp")
    wheel_velocity_ki = load_named_float(runtime_core, "kWheelVelocityKi")
    wheel_integral_limit = load_named_float(runtime_core, "kWheelIntegralLimit")

    equivalent_wheel_inertia_kg_m2 = load_named_float(plant_header, "equivalentWheelInertiaKgM2")
    drivetrain_efficiency = load_named_float(plant_header, "drivetrainEfficiency")
    rolling_friction_torque_nm = load_named_float(plant_header, "rollingFrictionTorqueNm")
    static_friction_max_speed_mps = load_named_float(plant_header, "staticFrictionMaxSpeedMps")
    viscous_friction_nm_per_radps = load_named_float(plant_header, "viscousFrictionNmPerRadps")
    plant_breakaway_drive_command = load_named_float(plant_cpp, "kReliableLaunchDriveCommand")

    motor_current_limit_a = supply_voltage_v / drive_resistance_ohms if drive_resistance_ohms > 0.0 else 0.0
    breakaway_current_a = plant_breakaway_drive_command * motor_current_limit_a
    if motor_current_limit_a > 0.0:
        breakaway_current_a = max(-motor_current_limit_a, min(motor_current_limit_a, breakaway_current_a))
    breakaway_load_current_a = max(0.0, breakaway_current_a - no_load_current_a)
    static_friction_torque_nm = (
        torque_constant_nm_per_a *
        breakaway_load_current_a *
        gear_ratio *
        drivetrain_efficiency
    )

    return CurrentFeedforwardSetup(
        launch_fit=LaunchFitParameters(
            drive=RunPlantParameters(
                battery_voltage_v=supply_voltage_v,
                drive_resistance_ohms=drive_resistance_ohms,
                torque_constant_nm_per_a=torque_constant_nm_per_a,
                speed_constant_radps_per_volt=speed_constant_radps_per_volt,
                no_load_current_a=no_load_current_a,
                gear_ratio=gear_ratio,
                wheel_radius_m=0.5 * wheel_diameter_m,
                nominal_track_width_m=track_width_m,
            ),
            effective_longitudinal_mass_kg=mass_kg,
            equivalent_wheel_inertia_kg_m2=equivalent_wheel_inertia_kg_m2,
            rolling_friction_torque_nm=rolling_friction_torque_nm,
            static_friction_torque_nm=static_friction_torque_nm,
            static_friction_max_speed_mps=static_friction_max_speed_mps,
            viscous_friction_nm_per_radps=viscous_friction_nm_per_radps,
            drivetrain_efficiency=drivetrain_efficiency,
            motor_current_limit_a=motor_current_limit_a,
        ),
        wheel_static_feedforward=wheel_static_feedforward,
        wheel_rest_launch_drive_command=wheel_rest_launch_drive_command,
        wheel_rest_launch_max_drive_command=wheel_rest_launch_max_drive_command,
        wheel_rest_launch_ramp_ms=wheel_rest_launch_ramp_ms,
        wheel_rest_launch_speed_threshold_mps=wheel_rest_launch_speed_threshold_mps,
        wheel_rest_launch_drive_threshold=wheel_rest_launch_drive_threshold,
        wheel_velocity_feedforward=wheel_velocity_feedforward,
        wheel_acceleration_response_gain_per_mps2=wheel_acceleration_response_gain_per_mps2,
        wheel_velocity_kp=wheel_velocity_kp,
        wheel_velocity_ki=wheel_velocity_ki,
        wheel_integral_limit=wheel_integral_limit,
        plant_breakaway_drive_command=plant_breakaway_drive_command,
    )


def build_trace_from_segment_rows(
    rows: list[dict[str, str]],
    abs_command: float,
    wheel_radius_m: float,
) -> list[LaunchMeanTracePoint]:
    trace: list[LaunchMeanTracePoint] = []
    for sample_index, row in enumerate(rows):
        left_drive_command = float(row["left_drive_cmd"])
        sign = 1.0 if left_drive_command >= 0.0 else -1.0
        average_encoder_velocity_mps = 0.5 * (float(row["left_velocity_mps"]) + float(row["right_velocity_mps"]))
        trace.append(
            LaunchMeanTracePoint(
                sample_index=sample_index,
                abs_command=abs_command,
                logged_command=abs(left_drive_command),
                measured_linear_speed_mps=sign * float(row["linear_speed_mps"]),
                average_encoder_omega_radps=(
                    (sign * average_encoder_velocity_mps) / wheel_radius_m
                    if wheel_radius_m > 0.0
                    else 0.0
                ),
                gyro_raw_radps=sign * float(row["gyro_raw_radps"]),
                accel_body_y_mps2=0.0,
                dt_seconds=1.0e-6 * int(row["dt_us"]),
                repeat_count=1,
            )
        )
    return trace


def extract_constant_command_segments(rows: list[dict[str, str]]) -> list[tuple[float, list[dict[str, str]]]]:
    segments: list[tuple[float, list[dict[str, str]]]] = []
    active_rows: list[dict[str, str]] = []
    active_command: float | None = None
    for row in rows:
        left_command = float(row["left_drive_cmd"])
        right_command = float(row["right_drive_cmd"])
        if abs(left_command) <= 1.0e-6 or abs(right_command) <= 1.0e-6 or abs(left_command - right_command) > 1.0e-6:
            if active_rows:
                assert active_command is not None
                segments.append((active_command, active_rows))
                active_rows = []
                active_command = None
            continue
        abs_command = round(abs(left_command), 2)
        if active_command is not None and abs(abs_command - active_command) > 1.0e-6:
            segments.append((active_command, active_rows))
            active_rows = []
        active_command = abs_command
        active_rows.append(row)
    if active_rows:
        assert active_command is not None
        segments.append((active_command, active_rows))
    return segments


def summarize_outcomes(outcomes: dict[float, list[int]]) -> list[CommandOutcomeSummary]:
    summaries: list[CommandOutcomeSummary] = []
    for abs_command in sorted(outcomes):
        values = outcomes[abs_command]
        summaries.append(
            CommandOutcomeSummary(
                abs_command=abs_command,
                success_count=sum(values),
                total_count=len(values),
            )
        )
    return summaries


def flatten_trace_groups(
    trace_groups: dict[float, list[list[LaunchMeanTracePoint]]],
) -> dict[float, list[LaunchMeanTracePoint]]:
    flattened: dict[float, list[LaunchMeanTracePoint]] = {}
    unique_index = 0
    for abs_command in sorted(trace_groups):
        for trace in trace_groups[abs_command]:
            flattened[abs_command + (unique_index * 1.0e-4)] = trace
            unique_index += 1
    return flattened


def breakaway_torque_nm_from_command(command: float, params: LaunchFitParameters) -> float:
    if command <= 0.0:
        return 0.0
    drive = params.drive
    if (
        drive.battery_voltage_v <= 0.0 or
        drive.drive_resistance_ohms <= 0.0 or
        drive.torque_constant_nm_per_a <= 0.0 or
        drive.gear_ratio <= 0.0
    ):
        return 0.0
    armature_current_a = (command * drive.battery_voltage_v) / drive.drive_resistance_ohms
    if params.motor_current_limit_a > 0.0:
        armature_current_a = min(params.motor_current_limit_a, armature_current_a)
    load_current_a = max(0.0, armature_current_a - drive.no_load_current_a)
    return (
        drive.torque_constant_nm_per_a *
        load_current_a *
        drive.gear_ratio *
        params.drivetrain_efficiency
    )


def fully_successful_command_bounds(outcomes: dict[float, list[int]]) -> tuple[float | None, float | None]:
    not_fully_successful = [
        abs_command
        for abs_command, values in outcomes.items()
        if values and sum(values) < len(values)
    ]
    fully_successful = [
        abs_command
        for abs_command, values in outcomes.items()
        if values and sum(values) == len(values)
    ]
    return (
        max(not_fully_successful) if not_fully_successful else None,
        min(fully_successful) if fully_successful else None,
    )


def alignment_median_rmse(summary: FeedforwardAlignmentSummary, steady: bool) -> float:
    squared_errors: list[float] = []
    for command_summary in summary.command_summaries:
        value = (
            command_summary.steady_required_command_median
            if steady
            else command_summary.required_command_median
        )
        if value is None:
            continue
        error = value - command_summary.abs_command
        squared_errors.append(error * error)
    return math.sqrt(sum(squared_errors) / len(squared_errors)) if squared_errors else 0.0


def collect_competition_trace_collection(
    csv_paths: list[Path],
    setup: CurrentFeedforwardSetup,
) -> CompetitionTraceCollection:
    kickoff_trace_groups: dict[float, list[list[LaunchMeanTracePoint]]] = defaultdict(list)
    forward_trace_groups: dict[float, list[list[LaunchMeanTracePoint]]] = defaultdict(list)
    kickoff_outcomes: dict[float, list[int]] = defaultdict(list)
    forward_outcomes: dict[float, list[int]] = defaultdict(list)
    source_files: set[Path] = set()

    for path in csv_paths:
        if not path.is_file():
            continue
        phase_names: dict[int, str] = {}
        rows_by_phase_id: dict[int, list[dict[str, str]]] = defaultdict(list)
        header: list[str] | None = None
        with path.open(newline="", encoding="utf-8", errors="replace") as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
                if not row:
                    continue
                if row[0] == "# phase":
                    if len(row) >= 4:
                        phase_names[int(row[1])] = row[3]
                    continue
                if row[0] == "# event" and len(row) >= 4:
                    if row[2] == "kickoff_result":
                        _, _, fields_text = row[3].partition(";")
                        fields = parse_key_value_fields(fields_text)
                        if "cmd" in fields and "moved" in fields:
                            kickoff_outcomes[round(fields["cmd"], 2)].append(int(fields["moved"]))
                    elif row[2] == "forward_result":
                        _, _, fields_text = row[3].partition(";")
                        fields = parse_key_value_fields(fields_text)
                        if "hold" in fields and "carried" in fields:
                            forward_outcomes[round(fields["hold"], 2)].append(int(fields["carried"]))
                    continue
                if row[0] == "sample":
                    header = row
                    continue
                if header is None:
                    continue
                row_dict = dict(zip(header, row))
                phase_id = int(row_dict["phase_id"])
                phase_name = phase_names.get(phase_id)
                if phase_name is None:
                    continue
                if KICKOFF_PROBE_PATTERN.match(phase_name) or FORWARD_PROBE_PATTERN.match(phase_name):
                    rows_by_phase_id[phase_id].append(row_dict)

        file_used = False
        for phase_id, rows in rows_by_phase_id.items():
            phase_name = phase_names[phase_id]
            kickoff_match = KICKOFF_PROBE_PATTERN.match(phase_name)
            forward_match = FORWARD_PROBE_PATTERN.match(phase_name)
            if kickoff_match is not None:
                nominal_command = round(0.01 * int(kickoff_match.group(1)), 2)
                for segment_command, segment_rows in extract_constant_command_segments(rows):
                    if abs(segment_command - nominal_command) > 1.0e-6:
                        continue
                    trace = build_trace_from_segment_rows(
                        segment_rows,
                        nominal_command,
                        setup.launch_fit.drive.wheel_radius_m,
                    )
                    if len(trace) >= 3:
                        kickoff_trace_groups[nominal_command].append(trace)
                        file_used = True
                continue
            if forward_match is not None:
                nominal_command = round(0.01 * int(forward_match.group(1)), 2)
                for segment_command, segment_rows in extract_constant_command_segments(rows):
                    if abs(segment_command - nominal_command) > 1.0e-6:
                        continue
                    trace = build_trace_from_segment_rows(
                        segment_rows,
                        nominal_command,
                        setup.launch_fit.drive.wheel_radius_m,
                    )
                    if len(trace) >= 3:
                        forward_trace_groups[nominal_command].append(trace)
                        file_used = True
        if file_used:
            source_files.add(path)

    return CompetitionTraceCollection(
        kickoff_trace_groups=kickoff_trace_groups,
        forward_trace_groups=forward_trace_groups,
        kickoff_outcomes=kickoff_outcomes,
        forward_outcomes=forward_outcomes,
        source_file_count=len(source_files),
    )


def summarize_competition_sweep(
    trace_groups: dict[float, list[list[LaunchMeanTracePoint]]],
    outcomes: dict[float, list[int]],
    source_file_count: int,
    setup: CurrentFeedforwardSetup,
) -> CompetitionSweepSummary:
    trace_count = sum(len(traces) for traces in trace_groups.values())
    return CompetitionSweepSummary(
        source_file_count=source_file_count,
        trace_count=trace_count,
        outcome_summaries=summarize_outcomes(outcomes),
        feedforward_alignment=summarize_feedforward_alignment_trace_groups(
            trace_groups,
            setup.launch_fit,
            run_id=None,
        ),
    )


def analyze_competition_feedforward(
    csv_paths: list[Path],
    setup: CurrentFeedforwardSetup,
) -> CompetitionFeedforwardReport:
    collection = collect_competition_trace_collection(csv_paths, setup)
    return CompetitionFeedforwardReport(
        current_setup=setup,
        kickoff=summarize_competition_sweep(
            collection.kickoff_trace_groups,
            collection.kickoff_outcomes,
            collection.source_file_count,
            setup,
        ),
        forward_hold=summarize_competition_sweep(
            collection.forward_trace_groups,
            collection.forward_outcomes,
            collection.source_file_count,
            setup,
        ),
    )


def estimate_competition_current_plant_parameters(
    csv_paths: list[Path],
    setup: CurrentFeedforwardSetup,
) -> CompetitionPlantParameterEstimate:
    collection = collect_competition_trace_collection(csv_paths, setup)
    kickoff_traces = flatten_trace_groups(collection.kickoff_trace_groups)
    forward_traces = flatten_trace_groups(collection.forward_trace_groups)

    apparent_inertia_kg_m2, inertia_trace_count = fit_apparent_equivalent_wheel_inertia(
        kickoff_traces,
        setup.launch_fit,
    )
    apparent_rolling_friction_torque_nm, apparent_viscous_friction_nm_per_radps, drag_fit_row_count, drag_sigma_nm = (
        fit_apparent_drive_drag(
            forward_traces,
            setup.launch_fit,
        )
    )

    lower_command, upper_command = fully_successful_command_bounds(collection.kickoff_outcomes)
    midpoint_command = None
    if lower_command is not None and upper_command is not None:
        midpoint_command = 0.5 * (lower_command + upper_command)
    lower_torque_nm = (
        breakaway_torque_nm_from_command(lower_command, setup.launch_fit)
        if lower_command is not None
        else None
    )
    upper_torque_nm = (
        breakaway_torque_nm_from_command(upper_command, setup.launch_fit)
        if upper_command is not None
        else None
    )
    midpoint_torque_nm = (
        breakaway_torque_nm_from_command(midpoint_command, setup.launch_fit)
        if midpoint_command is not None
        else None
    )

    inertia_candidates = sorted({
        round(0.75 * setup.launch_fit.equivalent_wheel_inertia_kg_m2, 10),
        round(setup.launch_fit.equivalent_wheel_inertia_kg_m2, 10),
        round(apparent_inertia_kg_m2, 10) if apparent_inertia_kg_m2 is not None else round(setup.launch_fit.equivalent_wheel_inertia_kg_m2, 10),
        round(1.25 * setup.launch_fit.equivalent_wheel_inertia_kg_m2, 10),
    })
    rolling_candidates = sorted({
        round(0.5 * setup.launch_fit.rolling_friction_torque_nm, 6),
        round(setup.launch_fit.rolling_friction_torque_nm, 6),
        round(apparent_rolling_friction_torque_nm, 6) if apparent_rolling_friction_torque_nm is not None else round(setup.launch_fit.rolling_friction_torque_nm, 6),
        round(1.1 * setup.launch_fit.rolling_friction_torque_nm, 6),
    })
    viscous_candidates = sorted({
        0.0,
        round(apparent_viscous_friction_nm_per_radps, 7) if apparent_viscous_friction_nm_per_radps is not None else 0.0,
        0.0002,
        0.0003,
    })
    static_candidates = sorted({
        round(lower_torque_nm, 6) if lower_torque_nm is not None else round(setup.launch_fit.static_friction_torque_nm, 6),
        round(midpoint_torque_nm, 6) if midpoint_torque_nm is not None else round(setup.launch_fit.static_friction_torque_nm, 6),
        round(upper_torque_nm, 6) if upper_torque_nm is not None else round(setup.launch_fit.static_friction_torque_nm, 6),
        round(setup.launch_fit.static_friction_torque_nm, 6),
    })

    best_result: tuple[float, float, float, float, float, float, float] | None = None
    for inertia_kg_m2 in inertia_candidates:
        for rolling_torque_nm in rolling_candidates:
            for viscous_nm_per_radps in viscous_candidates:
                for static_torque_nm in static_candidates:
                    params = LaunchFitParameters(
                        drive=setup.launch_fit.drive,
                        effective_longitudinal_mass_kg=setup.launch_fit.effective_longitudinal_mass_kg,
                        equivalent_wheel_inertia_kg_m2=inertia_kg_m2,
                        rolling_friction_torque_nm=rolling_torque_nm,
                        static_friction_torque_nm=static_torque_nm,
                        static_friction_max_speed_mps=setup.launch_fit.static_friction_max_speed_mps,
                        viscous_friction_nm_per_radps=viscous_nm_per_radps,
                        drivetrain_efficiency=setup.launch_fit.drivetrain_efficiency,
                        motor_current_limit_a=setup.launch_fit.motor_current_limit_a,
                    )
                    kickoff_summary = summarize_feedforward_alignment_trace_groups(
                        collection.kickoff_trace_groups,
                        params,
                        None,
                    )
                    forward_summary = summarize_feedforward_alignment_trace_groups(
                        collection.forward_trace_groups,
                        params,
                        None,
                    )
                    if kickoff_summary is None or forward_summary is None:
                        continue
                    kickoff_median_rmse = alignment_median_rmse(kickoff_summary, steady=False)
                    forward_steady_median_rmse = alignment_median_rmse(forward_summary, steady=True)
                    objective = 0.5 * kickoff_median_rmse + 0.5 * forward_steady_median_rmse
                    candidate = (
                        objective,
                        inertia_kg_m2,
                        rolling_torque_nm,
                        static_torque_nm,
                        viscous_nm_per_radps,
                        kickoff_median_rmse,
                        forward_steady_median_rmse,
                    )
                    if best_result is None or candidate[0] < best_result[0]:
                        best_result = candidate

    assert best_result is not None
    _, recommended_inertia_kg_m2, recommended_rolling_torque_nm, recommended_static_torque_nm, recommended_viscous_nm_per_radps, recommended_kickoff_median_rmse, recommended_forward_steady_median_rmse = best_result

    return CompetitionPlantParameterEstimate(
        apparent_equivalent_wheel_inertia_kg_m2=apparent_inertia_kg_m2,
        apparent_equivalent_wheel_inertia_trace_count=inertia_trace_count,
        apparent_rolling_friction_torque_nm=apparent_rolling_friction_torque_nm,
        apparent_viscous_friction_nm_per_radps=apparent_viscous_friction_nm_per_radps,
        apparent_drag_fit_row_count=drag_fit_row_count,
        apparent_drag_fit_residual_sigma_nm=drag_sigma_nm,
        breakaway_threshold_lower_command=lower_command,
        breakaway_threshold_upper_command=upper_command,
        breakaway_threshold_midpoint_command=midpoint_command,
        breakaway_static_friction_lower_torque_nm=lower_torque_nm,
        breakaway_static_friction_midpoint_torque_nm=midpoint_torque_nm,
        breakaway_static_friction_upper_torque_nm=upper_torque_nm,
        recommended_equivalent_wheel_inertia_kg_m2=recommended_inertia_kg_m2,
        recommended_rolling_friction_torque_nm=recommended_rolling_torque_nm,
        recommended_static_friction_torque_nm=recommended_static_torque_nm,
        recommended_viscous_friction_nm_per_radps=recommended_viscous_nm_per_radps,
        recommended_objective=best_result[0],
        recommended_kickoff_median_rmse=recommended_kickoff_median_rmse,
        recommended_forward_steady_median_rmse=recommended_forward_steady_median_rmse,
    )


def discover_competition_diag_csvs(root: Path) -> list[Path]:
    return sorted(path for path in root.glob("diag*.csv") if path.is_file())
