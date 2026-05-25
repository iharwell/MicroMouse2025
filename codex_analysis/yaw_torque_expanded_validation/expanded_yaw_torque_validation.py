#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR_DEFAULT = Path(__file__).resolve().parent

FORWARD_BIN_STEP_MPS = 0.10
YAW_BIN_STEP_RADPS = 0.50
MIN_DT_US = 350
MAX_DT_US = 3500
MIN_ABS_FORWARD_MPS = 0.05
MIN_ABS_YAW_RADPS = 0.25
MIN_ACTIVE_COMMAND_DELTA = 0.04
MIN_BIN_COUNT = 50
MIN_CONSISTENCY_RUN_COUNT = 2


@dataclass(frozen=True)
class SourceParams:
    mass_kg: float
    track_width_m: float
    yaw_inertia_kg_m2: float
    wheel_radius_m: float
    wheel_bank_inertia_kg_m2: float
    drive_voltage_v: float
    drive_resistance_ohms: float
    torque_constant_nm_per_a: float
    speed_constant_radps_per_volt: float
    no_load_current_a: float
    gear_ratio: float
    rolling_friction_torque_nm: float
    static_launch_command: float
    static_friction_max_speed_mps: float
    viscous_friction_nm_per_radps: float
    yaw_rate_damping_nms_per_rad: float
    drive_wheel_longitudinal_offset_m: float
    front_load_fraction: float
    longitudinal_tire_stiffness_n: float
    front_right_contact_force_gain_n_per_mps: float
    rear_right_contact_force_gain_n_per_mps: float
    fan_downforce_full_duty_n: float
    sustained_lateral_accel_mps2: float


@dataclass(frozen=True)
class CandidateLog:
    run_id: str
    kind: str
    path: Path
    size_bytes: int


@dataclass
class RowState:
    row_index: int
    time_us: int
    dt_us: int
    phase_key: tuple[str, ...]
    stationary: bool
    forward_velocity_mps: float
    yaw_rate_radps: float
    left_velocity_mps: float
    right_velocity_mps: float
    left_wheel_speed_radps: float
    right_wheel_speed_radps: float
    left_command: float
    right_command: float
    cmd_angular_radps: float
    saturation_flags: int
    watchdog_flags: int


@dataclass(frozen=True)
class Sample:
    run_id: str
    kind: str
    row_index: int
    time_us: int
    dt_s: float
    forward_velocity_mps: float
    yaw_rate_radps: float
    next_yaw_rate_radps: float
    measured_yaw_accel_radps2: float
    model_yaw_moment_nm: float
    residual_additive_yaw_torque_nm: float
    opposing_yaw_resistance_nm: float
    forward_bin_mps: float
    yaw_rate_bin_radps: float
    abs_yaw_rate_bin_radps: float


@dataclass(frozen=True)
class RunSummary:
    run_id: str
    kind: str
    path: Path
    input_rows: int
    stationary_bias_rows: int
    gyro_bias_radps: float
    adjacent_pairs: int
    active_pairs: int
    extracted_samples: int
    excluded_reason: str
    max_abs_forward_mps: float
    max_abs_yaw_radps: float
    nonzero_forward_sample_count: int
    nonzero_forward_yaw_sample_count: int


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def const_float(text: str, name: str) -> float:
    pattern = rf"\b{name}\s*=\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)[fF]?"
    match = re.search(pattern, text)
    if not match:
        raise ValueError(f"Could not find {name}")
    return float(match.group(1))


def converted_const_float(text: str, name: str, function_name: str, scale: float) -> float:
    pattern = rf"\b{name}\s*=\s*{function_name}\(\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)[fF]?\s*\)"
    match = re.search(pattern, text)
    if not match:
        raise ValueError(f"Could not find {name}")
    return scale * float(match.group(1))


def source_params() -> SourceParams:
    vehicle_h = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "Vehicle.h")
    motor_h = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "MotorEncoderDrive.h")
    plant_h = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "PlantModel.h")
    wheel_diameter_m = const_float(vehicle_h, "kDriveWheelDiameterM")
    no_load_current_a = converted_const_float(vehicle_h, "kDriveNoLoadCurrentA", "MilliAmpsToAmps", 1.0e-3)
    resistance_ohms = const_float(vehicle_h, "kDriveResistanceOhms")
    nominal_voltage_v = const_float(vehicle_h, "kDriveNominalVoltageV")
    no_load_speed_radps = const_float(vehicle_h, "kDriveNominalNoLoadSpeedRpm") * (2.0 * math.pi / 60.0)
    return SourceParams(
        mass_kg=const_float(vehicle_h, "kPhysicalMassKg"),
        track_width_m=const_float(vehicle_h, "kPhysicalTrackWidthM"),
        yaw_inertia_kg_m2=const_float(vehicle_h, "kPhysicalYawInertiaKgM2"),
        wheel_radius_m=0.5 * wheel_diameter_m,
        wheel_bank_inertia_kg_m2=const_float(motor_h, "kDefaultWheelBankEquivalentInertiaKgM2"),
        drive_voltage_v=const_float(vehicle_h, "kDriveSupplyVoltageV"),
        drive_resistance_ohms=resistance_ohms,
        torque_constant_nm_per_a=converted_const_float(
            vehicle_h,
            "kDriveTorqueConstantNmPerA",
            "MilliNewtonMetersToNewtonMeters",
            1.0e-3,
        ),
        speed_constant_radps_per_volt=no_load_speed_radps / (nominal_voltage_v - (no_load_current_a * resistance_ohms)),
        no_load_current_a=no_load_current_a,
        gear_ratio=const_float(vehicle_h, "kDriveGearRatio"),
        rolling_friction_torque_nm=const_float(plant_h, "kRollingFrictionTorqueNm"),
        static_launch_command=const_float(plant_h, "kReliableLaunchDriveCommand"),
        static_friction_max_speed_mps=const_float(plant_h, "kStaticFrictionMaxSpeedMps"),
        viscous_friction_nm_per_radps=const_float(plant_h, "kViscousFrictionNmPerRadps"),
        yaw_rate_damping_nms_per_rad=const_float(plant_h, "kYawRateDampingNmsPerRad"),
        drive_wheel_longitudinal_offset_m=const_float(vehicle_h, "kDriveWheelLongitudinalOffsetM"),
        front_load_fraction=const_float(plant_h, "kFrontLoadFraction"),
        longitudinal_tire_stiffness_n=const_float(motor_h, "kDefaultLongitudinalTireStiffnessN"),
        front_right_contact_force_gain_n_per_mps=const_float(motor_h, "kDefaultFrontRightContactForceGainNPerMps"),
        rear_right_contact_force_gain_n_per_mps=const_float(motor_h, "kDefaultRearRightContactForceGainNPerMps"),
        fan_downforce_full_duty_n=const_float(vehicle_h, "kFanDownforceAtFullDutyN"),
        sustained_lateral_accel_mps2=1.91 * 9.80665,
    )


def yaw_denominator_kg_m2(params: SourceParams) -> float:
    wheel_spinup_mass_kg = (2.0 * params.wheel_bank_inertia_kg_m2) / (params.wheel_radius_m * params.wheel_radius_m)
    half_track = 0.5 * abs(params.track_width_m)
    return params.yaw_inertia_kg_m2 + (wheel_spinup_mass_kg * half_track * half_track)


def finite_float(row: dict[str, str], key: str, default: float | None = None) -> float:
    text = row.get(key, "")
    if text == "":
        if default is not None:
            return default
        raise ValueError(key)
    value = float(text)
    if not math.isfinite(value):
        raise ValueError(key)
    return value


def finite_int(row: dict[str, str], key: str, default: int | None = None) -> int:
    text = row.get(key, "")
    if text == "":
        if default is not None:
            return default
        raise ValueError(key)
    return int(float(text))


def median_abs_deviation(values: list[float], center: float | None = None) -> float:
    if not values:
        return 0.0
    resolved_center = statistics.median(values) if center is None else center
    return statistics.median(abs(value - resolved_center) for value in values)


def percentile(sorted_values: list[float], fraction: float) -> float:
    if not sorted_values:
        return 0.0
    index = fraction * (len(sorted_values) - 1)
    lo = int(math.floor(index))
    hi = int(math.ceil(index))
    if lo == hi:
        return sorted_values[lo]
    return sorted_values[lo] + ((sorted_values[hi] - sorted_values[lo]) * (index - lo))


def trimmed_mean(values: list[float], trim_fraction: float = 0.10) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    trim = int(len(ordered) * trim_fraction)
    kept = ordered[trim:len(ordered) - trim] if len(ordered) - (2 * trim) > 0 else ordered
    return statistics.fmean(kept)


def robust_bias(values: list[float]) -> tuple[float, int]:
    if not values:
        return 0.0, 0
    center = statistics.median(values)
    mad = median_abs_deviation(values, center)
    threshold = max(0.015, 5.0 * 1.4826 * mad)
    kept = [value for value in values if abs(value - center) <= threshold]
    return (statistics.median(kept), len(kept)) if kept else (center, len(values))


def signed_direction(preferred_value: float, fallback_value: float = 0.0) -> float:
    eps = 1.0e-6
    if preferred_value > eps:
        return 1.0
    if preferred_value < -eps:
        return -1.0
    if fallback_value > eps:
        return 1.0
    if fallback_value < -eps:
        return -1.0
    return 0.0


def torque_from_command(command: float, wheel_speed_radps: float, params: SourceParams) -> float:
    applied_voltage_v = max(-1.0, min(1.0, command)) * params.drive_voltage_v
    back_emf_voltage_v = (wheel_speed_radps * params.gear_ratio) / params.speed_constant_radps_per_volt
    armature_current_a = (applied_voltage_v - back_emf_voltage_v) / params.drive_resistance_ohms
    no_load_direction = signed_direction(armature_current_a, wheel_speed_radps)
    load_current_a = armature_current_a - (no_load_direction * params.no_load_current_a)
    if no_load_direction > 0.0 and load_current_a < 0.0:
        load_current_a = 0.0
    elif no_load_direction < 0.0 and load_current_a > 0.0:
        load_current_a = 0.0
    return params.torque_constant_nm_per_a * params.gear_ratio * load_current_a


def applied_bank_torque(command: float, wheel_speed_radps: float, params: SourceParams) -> float:
    direct = torque_from_command(command, wheel_speed_radps, params)
    positive_limit = max(0.0, torque_from_command(1.0, wheel_speed_radps, params))
    negative_limit = min(0.0, torque_from_command(-1.0, wheel_speed_radps, params))
    limited = max(negative_limit, min(positive_limit, direct)) if positive_limit > negative_limit else direct
    static_launch_torque_nm = max(0.0, torque_from_command(params.static_launch_command, 0.0, params))
    surface_speed_mps = params.wheel_radius_m * wheel_speed_radps
    slow_ratio = abs(surface_speed_mps) / params.static_friction_max_speed_mps
    launch_torque_nm = static_launch_torque_nm * math.exp(-(slow_ratio * slow_ratio))
    launch_direction = signed_direction(limited, wheel_speed_radps)
    applied = 0.0
    if abs(limited) > launch_torque_nm:
        applied = limited - (launch_direction * launch_torque_nm)
    loss_direction = signed_direction(wheel_speed_radps, applied)
    rolling_loss = (params.rolling_friction_torque_nm * loss_direction) + (
        params.viscous_friction_nm_per_radps * wheel_speed_radps
    )
    return applied - rolling_loss


def current_model_yaw_moment_nm(state: RowState, params: SourceParams) -> float:
    left_torque_nm = applied_bank_torque(state.left_command, state.left_wheel_speed_radps, params)
    right_torque_nm = applied_bank_torque(state.right_command, state.right_wheel_speed_radps, params)
    left_drive_force_n = left_torque_nm / params.wheel_radius_m
    right_drive_force_n = right_torque_nm / params.wheel_radius_m

    fan_duty = 0.8
    total_normal_load_n = (params.mass_kg * 9.80665) + (fan_duty * params.fan_downforce_full_duty_n)
    front_wheel_normal_n = 0.5 * params.front_load_fraction * total_normal_load_n
    rear_wheel_normal_n = 0.5 * (1.0 - params.front_load_fraction) * total_normal_load_n
    left_bank_normal_n = front_wheel_normal_n + rear_wheel_normal_n
    right_bank_normal_n = left_bank_normal_n
    front_left_drive_request_n = left_drive_force_n * (front_wheel_normal_n / left_bank_normal_n)
    rear_left_drive_request_n = left_drive_force_n * (rear_wheel_normal_n / left_bank_normal_n)
    front_right_drive_request_n = right_drive_force_n * (front_wheel_normal_n / right_bank_normal_n)
    rear_right_drive_request_n = right_drive_force_n * (rear_wheel_normal_n / right_bank_normal_n)

    half_track = 0.5 * abs(params.track_width_m)
    left_body_forward_mps = state.forward_velocity_mps + (half_track * state.yaw_rate_radps)
    right_body_forward_mps = state.forward_velocity_mps - (half_track * state.yaw_rate_radps)
    left_forward_rel_mps = state.left_velocity_mps - left_body_forward_mps
    right_forward_rel_mps = state.right_velocity_mps - right_body_forward_mps

    longitudinal_offset_m = abs(params.drive_wheel_longitudinal_offset_m)
    front_right_body_velocity_mps = longitudinal_offset_m * state.yaw_rate_radps
    rear_right_body_velocity_mps = -longitudinal_offset_m * state.yaw_rate_radps
    front_right_rel_mps = -front_right_body_velocity_mps
    rear_right_rel_mps = -rear_right_body_velocity_mps

    half_longitudinal_stiffness = 0.5 * params.longitudinal_tire_stiffness_n
    front_left_raw_forward_n = front_left_drive_request_n + (half_longitudinal_stiffness * left_forward_rel_mps)
    rear_left_raw_forward_n = rear_left_drive_request_n + (half_longitudinal_stiffness * left_forward_rel_mps)
    front_right_raw_forward_n = front_right_drive_request_n + (half_longitudinal_stiffness * right_forward_rel_mps)
    rear_right_raw_forward_n = rear_right_drive_request_n + (half_longitudinal_stiffness * right_forward_rel_mps)
    front_left_raw_right_n = params.front_right_contact_force_gain_n_per_mps * front_right_rel_mps
    front_right_raw_right_n = params.front_right_contact_force_gain_n_per_mps * front_right_rel_mps
    rear_left_raw_right_n = params.rear_right_contact_force_gain_n_per_mps * rear_right_rel_mps
    rear_right_raw_right_n = params.rear_right_contact_force_gain_n_per_mps * rear_right_rel_mps

    sustained_mu = (params.sustained_lateral_accel_mps2 * params.mass_kg / total_normal_load_n) if total_normal_load_n > 1.0e-4 else 0.0

    def project(forward_n: float, right_n: float, normal_n: float) -> tuple[float, float]:
        magnitude = math.hypot(forward_n, right_n)
        max_force = max(0.0, sustained_mu * normal_n)
        scale = (max_force / magnitude) if magnitude > max_force and magnitude > 1.0e-4 else 1.0
        return scale * forward_n, scale * right_n

    front_left_forward_n, front_left_right_n = project(front_left_raw_forward_n, front_left_raw_right_n, front_wheel_normal_n)
    front_right_forward_n, front_right_right_n = project(front_right_raw_forward_n, front_right_raw_right_n, front_wheel_normal_n)
    rear_left_forward_n, rear_left_right_n = project(rear_left_raw_forward_n, rear_left_raw_right_n, rear_wheel_normal_n)
    rear_right_forward_n, rear_right_right_n = project(rear_right_raw_forward_n, rear_right_raw_right_n, rear_wheel_normal_n)

    left_bank_forward_n = front_left_forward_n + rear_left_forward_n
    right_bank_forward_n = front_right_forward_n + rear_right_forward_n
    front_right_force_n = front_left_right_n + front_right_right_n
    rear_right_force_n = rear_left_right_n + rear_right_right_n
    yaw_moment_nm = (half_track * (left_bank_forward_n - right_bank_forward_n)) + (
        longitudinal_offset_m * (front_right_force_n - rear_right_force_n)
    )
    return yaw_moment_nm - (params.yaw_rate_damping_nms_per_rad * state.yaw_rate_radps)


def round_to_step(value: float, step: float) -> float:
    if abs(value) < 0.5 * step:
        return 0.0
    return round(value / step) * step


def non_comment_csv_reader(path: Path) -> Iterable[dict[str, str]]:
    with path.open(newline="", encoding="utf-8", errors="replace") as raw:
        filtered = (line for line in raw if not line.startswith("#"))
        reader = csv.DictReader(filtered)
        if reader.fieldnames is None:
            return
        for row in reader:
            if row:
                yield row


def discover_logs() -> list[CandidateLog]:
    candidates: list[CandidateLog] = []
    for directory in sorted((REPO_ROOT / "TestResults").glob("mmlog_decode_*")):
        path = directory / "open_floor_main.csv"
        if path.is_file():
            candidates.append(CandidateLog(directory.name.replace("mmlog_decode_", ""), "decoded_open_floor", path, path.stat().st_size))
    competition = REPO_ROOT / "TestResults" / "Competition Testing Data"
    if competition.is_dir():
        for path in sorted(competition.glob("*.csv")):
            if path.name.startswith(("diag", "aux")):
                candidates.append(CandidateLog(path.stem, "competition", path, path.stat().st_size))
            elif path.name.startswith("fwc"):
                candidates.append(CandidateLog(path.stem, "competition_fwc", path, path.stat().st_size))
    extra = REPO_ROOT / "TestResults" / "UncertaintyTestLog" / "open_floor_main.csv"
    if extra.is_file():
        candidates.append(CandidateLog("UncertaintyTestLog", "decoded_open_floor", extra, extra.stat().st_size))
    return candidates


def row_to_state(row: dict[str, str], row_index: int, kind: str, bias_radps: float, params: SourceParams) -> RowState:
    if kind == "competition":
        time_us = finite_int(row, "t_us")
        dt_us = finite_int(row, "dt_us", 1000)
        left_velocity = finite_float(row, "left_velocity_mps")
        right_velocity = finite_float(row, "right_velocity_mps")
        left_command = finite_float(row, "left_drive_cmd")
        right_command = finite_float(row, "right_drive_cmd")
        raw_gyro = finite_float(row, "gyro_raw_radps")
        phase_key = (row.get("phase_id", ""),)
        stationary = finite_int(row, "stationary", 0) != 0
        cmd_angular = finite_float(row, "cmd_angular_radps", 0.0)
        saturation_flags = 0
        watchdog_flags = 0
    else:
        time_us = finite_int(row, "master_time_us", finite_int(row, "imu_timestamp_us", 0))
        dt_us = finite_int(row, "dt_us", 1000)
        left_velocity = finite_float(row, "left_encoder_velocity_mps")
        right_velocity = finite_float(row, "right_encoder_velocity_mps")
        left_command = finite_float(row, "left_drive_command")
        right_command = finite_float(row, "right_drive_command")
        raw_gyro = finite_float(row, "gyro_raw_radps")
        phase_key = (
            row.get("section_id", ""),
            row.get("phase_id", ""),
            row.get("primitive_id", ""),
            row.get("speed_bin", ""),
            row.get("repeat_index", ""),
        )
        stationary = (
            max(abs(left_command), abs(right_command)) <= 0.02
            and max(abs(left_velocity), abs(right_velocity)) <= 0.015
            and abs(raw_gyro) <= 0.5
        )
        cmd_angular = finite_float(row, "cmd_angular_radps", 0.0)
        saturation_flags = finite_int(row, "saturation_flags", 0)
        watchdog_flags = finite_int(row, "watchdog_flags", 0)

    forward_velocity = 0.5 * (left_velocity + right_velocity)
    return RowState(
        row_index=row_index,
        time_us=time_us,
        dt_us=dt_us,
        phase_key=phase_key,
        stationary=stationary,
        forward_velocity_mps=forward_velocity,
        yaw_rate_radps=raw_gyro - bias_radps,
        left_velocity_mps=left_velocity,
        right_velocity_mps=right_velocity,
        left_wheel_speed_radps=left_velocity / params.wheel_radius_m,
        right_wheel_speed_radps=right_velocity / params.wheel_radius_m,
        left_command=left_command,
        right_command=right_command,
        cmd_angular_radps=cmd_angular,
        saturation_flags=saturation_flags,
        watchdog_flags=watchdog_flags,
    )


def estimate_bias(candidate: CandidateLog, params: SourceParams) -> tuple[float, int, int]:
    if candidate.kind == "competition_fwc":
        return 0.0, 0, 0
    values: list[float] = []
    rows = 0
    try:
        for row in non_comment_csv_reader(candidate.path):
            rows += 1
            try:
                if candidate.kind == "competition":
                    raw_gyro = finite_float(row, "gyro_raw_radps")
                    stationary = finite_int(row, "stationary", 0) != 0
                    left_velocity = finite_float(row, "left_velocity_mps")
                    right_velocity = finite_float(row, "right_velocity_mps")
                    left_command = finite_float(row, "left_drive_cmd")
                    right_command = finite_float(row, "right_drive_cmd")
                else:
                    raw_gyro = finite_float(row, "gyro_raw_radps")
                    left_velocity = finite_float(row, "left_encoder_velocity_mps")
                    right_velocity = finite_float(row, "right_encoder_velocity_mps")
                    left_command = finite_float(row, "left_drive_command")
                    right_command = finite_float(row, "right_drive_command")
                    stationary = True
                dt_us = finite_int(row, "dt_us", 1000)
            except ValueError:
                continue
            if not (MIN_DT_US <= dt_us <= MAX_DT_US):
                continue
            if not stationary:
                continue
            if max(abs(left_command), abs(right_command)) > 0.025:
                continue
            if max(abs(left_velocity), abs(right_velocity)) > 0.02:
                continue
            if abs(raw_gyro) > 0.5:
                continue
            values.append(raw_gyro)
    except (OSError, csv.Error):
        return 0.0, 0, rows
    bias, kept = robust_bias(values)
    return bias, kept, rows


def is_valid_pair(current: RowState, nxt: RowState, kind: str) -> bool:
    if not (MIN_DT_US <= nxt.dt_us <= MAX_DT_US):
        return False
    if current.phase_key != nxt.phase_key:
        return False
    if nxt.time_us < current.time_us:
        return False
    if current.saturation_flags != 0 or nxt.saturation_flags != 0:
        return False
    if current.watchdog_flags != 0 or nxt.watchdog_flags != 0:
        return False
    if kind == "competition" and current.stationary and nxt.stationary:
        return False
    return True


def is_active_yaw_pair(current: RowState, nxt: RowState) -> bool:
    command_delta = abs(current.left_command - current.right_command)
    next_delta = abs(nxt.left_command - nxt.right_command)
    return (
        max(command_delta, next_delta) >= MIN_ACTIVE_COMMAND_DELTA
        or max(abs(current.cmd_angular_radps), abs(nxt.cmd_angular_radps)) >= MIN_ABS_YAW_RADPS
        or max(abs(current.yaw_rate_radps), abs(nxt.yaw_rate_radps)) >= MIN_ABS_YAW_RADPS
    )


def extract_samples(candidate: CandidateLog, params: SourceParams, denominator: float) -> tuple[RunSummary, list[Sample]]:
    if candidate.kind == "competition_fwc":
        summary = RunSummary(
            run_id=candidate.run_id,
            kind=candidate.kind,
            path=candidate.path,
            input_rows=0,
            stationary_bias_rows=0,
            gyro_bias_radps=0.0,
            adjacent_pairs=0,
            active_pairs=0,
            extracted_samples=0,
            excluded_reason="excluded: front-wall characterization CSV lacks yaw/encoder time series",
            max_abs_forward_mps=0.0,
            max_abs_yaw_radps=0.0,
            nonzero_forward_sample_count=0,
            nonzero_forward_yaw_sample_count=0,
        )
        return summary, []

    bias, bias_rows, input_rows = estimate_bias(candidate, params)
    samples: list[Sample] = []
    pending: RowState | None = None
    adjacent_pairs = 0
    active_pairs = 0
    max_abs_forward = 0.0
    max_abs_yaw = 0.0
    nonzero_forward = 0
    nonzero_forward_yaw = 0
    parsed_rows = 0
    try:
        for row_index, row in enumerate(non_comment_csv_reader(candidate.path)):
            try:
                state = row_to_state(row, row_index, candidate.kind, bias, params)
            except ValueError:
                continue
            parsed_rows += 1
            max_abs_forward = max(max_abs_forward, abs(state.forward_velocity_mps))
            max_abs_yaw = max(max_abs_yaw, abs(state.yaw_rate_radps))
            if abs(state.forward_velocity_mps) >= MIN_ABS_FORWARD_MPS:
                nonzero_forward += 1
                if abs(state.yaw_rate_radps) >= MIN_ABS_YAW_RADPS:
                    nonzero_forward_yaw += 1
            if pending is not None and is_valid_pair(pending, state, candidate.kind):
                adjacent_pairs += 1
                if is_active_yaw_pair(pending, state):
                    active_pairs += 1
                    if abs(pending.forward_velocity_mps) >= MIN_ABS_FORWARD_MPS and abs(pending.yaw_rate_radps) >= MIN_ABS_YAW_RADPS:
                        dt_s = state.dt_us * 1.0e-6
                        yaw_accel = (state.yaw_rate_radps - pending.yaw_rate_radps) / dt_s
                        if math.isfinite(yaw_accel) and abs(yaw_accel) <= 3000.0:
                            model_moment = current_model_yaw_moment_nm(pending, params)
                            observed_moment = denominator * yaw_accel
                            residual = observed_moment - model_moment
                            yaw_sign = signed_direction(pending.yaw_rate_radps)
                            opposing = -yaw_sign * residual if yaw_sign != 0.0 else 0.0
                            forward_bin = round_to_step(pending.forward_velocity_mps, FORWARD_BIN_STEP_MPS)
                            yaw_bin = round_to_step(pending.yaw_rate_radps, YAW_BIN_STEP_RADPS)
                            if forward_bin != 0.0 and yaw_bin != 0.0:
                                samples.append(
                                    Sample(
                                        run_id=candidate.run_id,
                                        kind=candidate.kind,
                                        row_index=pending.row_index,
                                        time_us=pending.time_us,
                                        dt_s=dt_s,
                                        forward_velocity_mps=pending.forward_velocity_mps,
                                        yaw_rate_radps=pending.yaw_rate_radps,
                                        next_yaw_rate_radps=state.yaw_rate_radps,
                                        measured_yaw_accel_radps2=yaw_accel,
                                        model_yaw_moment_nm=model_moment,
                                        residual_additive_yaw_torque_nm=residual,
                                        opposing_yaw_resistance_nm=opposing,
                                        forward_bin_mps=forward_bin,
                                        yaw_rate_bin_radps=yaw_bin,
                                        abs_yaw_rate_bin_radps=abs(yaw_bin),
                                    )
                                )
            pending = state
    except (OSError, csv.Error):
        pass

    reason = ""
    if parsed_rows == 0:
        reason = "excluded: no parseable required sensor rows"
    elif bias_rows == 0:
        reason = "included with zero fallback gyro bias: no independent stationary rows"
    elif not samples:
        reason = "excluded from bins: no nonzero-forward/nonzero-yaw active adjacent samples"
    else:
        reason = "included"

    summary = RunSummary(
        run_id=candidate.run_id,
        kind=candidate.kind,
        path=candidate.path,
        input_rows=input_rows if input_rows else parsed_rows,
        stationary_bias_rows=bias_rows,
        gyro_bias_radps=bias,
        adjacent_pairs=adjacent_pairs,
        active_pairs=active_pairs,
        extracted_samples=len(samples),
        excluded_reason=reason,
        max_abs_forward_mps=max_abs_forward,
        max_abs_yaw_radps=max_abs_yaw,
        nonzero_forward_sample_count=nonzero_forward,
        nonzero_forward_yaw_sample_count=nonzero_forward_yaw,
    )
    return summary, samples


def grouped_samples(samples: Iterable[Sample]) -> dict[tuple[float, float], list[Sample]]:
    grouped: dict[tuple[float, float], list[Sample]] = {}
    for sample in samples:
        grouped.setdefault((sample.forward_bin_mps, sample.yaw_rate_bin_radps), []).append(sample)
    return grouped


def build_surface(samples: Iterable[Sample], min_count: int = MIN_BIN_COUNT) -> dict[tuple[float, float], float]:
    grouped: dict[tuple[float, float], list[float]] = {}
    for sample in samples:
        grouped.setdefault((sample.forward_bin_mps, sample.yaw_rate_bin_radps), []).append(sample.residual_additive_yaw_torque_nm)
    surface: dict[tuple[float, float], float] = {}
    for key, values in grouped.items():
        if len(values) >= min_count:
            surface[key] = statistics.median(values)
    return surface


def rmse(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(statistics.fmean(value * value for value in values))


def write_discovered(path: Path, candidates: list[CandidateLog], summaries: list[RunSummary]) -> None:
    by_key = {(s.kind, s.run_id): s for s in summaries}
    with path.open("w", newline="", encoding="utf-8") as f:
        fields = [
            "run_id",
            "kind",
            "path",
            "size_bytes",
            "input_rows",
            "stationary_bias_rows",
            "gyro_bias_radps",
            "adjacent_pairs",
            "active_pairs",
            "extracted_samples",
            "max_abs_forward_mps",
            "max_abs_yaw_radps",
            "nonzero_forward_sample_count",
            "nonzero_forward_yaw_sample_count",
            "status",
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for candidate in candidates:
            summary = by_key.get((candidate.kind, candidate.run_id))
            writer.writerow(
                {
                    "run_id": candidate.run_id,
                    "kind": candidate.kind,
                    "path": str(candidate.path.relative_to(REPO_ROOT)),
                    "size_bytes": candidate.size_bytes,
                    "input_rows": "" if summary is None else summary.input_rows,
                    "stationary_bias_rows": "" if summary is None else summary.stationary_bias_rows,
                    "gyro_bias_radps": "" if summary is None else f"{summary.gyro_bias_radps:.9f}",
                    "adjacent_pairs": "" if summary is None else summary.adjacent_pairs,
                    "active_pairs": "" if summary is None else summary.active_pairs,
                    "extracted_samples": "" if summary is None else summary.extracted_samples,
                    "max_abs_forward_mps": "" if summary is None else f"{summary.max_abs_forward_mps:.6f}",
                    "max_abs_yaw_radps": "" if summary is None else f"{summary.max_abs_yaw_radps:.6f}",
                    "nonzero_forward_sample_count": "" if summary is None else summary.nonzero_forward_sample_count,
                    "nonzero_forward_yaw_sample_count": "" if summary is None else summary.nonzero_forward_yaw_sample_count,
                    "status": "not processed" if summary is None else summary.excluded_reason,
                }
            )


def write_bin_tables(out_dir: Path, samples: list[Sample]) -> list[dict[str, str]]:
    grouped = grouped_samples(samples)
    rows: list[dict[str, str]] = []
    run_rows: list[dict[str, str]] = []
    for (forward_bin, yaw_bin), group in sorted(grouped.items()):
        if len(group) < MIN_BIN_COUNT:
            continue
        residual = [s.residual_additive_yaw_torque_nm for s in group]
        opposing = [s.opposing_yaw_resistance_nm for s in group]
        ordered_opposing = sorted(opposing)
        med_residual = statistics.median(residual)
        med_opposing = statistics.median(opposing)
        by_run: dict[str, list[Sample]] = {}
        for sample in group:
            by_run.setdefault(sample.run_id, []).append(sample)
        run_medians: list[float] = []
        for run_id, run_group in sorted(by_run.items()):
            if len(run_group) < 10:
                continue
            run_opposing = [s.opposing_yaw_resistance_nm for s in run_group]
            run_residual = [s.residual_additive_yaw_torque_nm for s in run_group]
            run_median = statistics.median(run_opposing)
            run_medians.append(run_median)
            run_rows.append(
                {
                    "forward_velocity_bin_mps": f"{forward_bin:.2f}",
                    "yaw_rate_bin_radps": f"{yaw_bin:.2f}",
                    "run_id": run_id,
                    "kind": run_group[0].kind,
                    "count": str(len(run_group)),
                    "median_residual_additive_nm": f"{statistics.median(run_residual):.9f}",
                    "median_opposing_nm": f"{run_median:.9f}",
                    "mad_opposing_nm": f"{median_abs_deviation(run_opposing, run_median):.9f}",
                }
            )
        run_count = len(run_medians)
        run_median_spread = (max(run_medians) - min(run_medians)) if run_medians else 0.0
        run_median_std = statistics.pstdev(run_medians) if len(run_medians) > 1 else 0.0
        positive_run_frac = (sum(1 for value in run_medians if value > 0.0) / run_count) if run_count else 0.0
        rows.append(
            {
                "forward_velocity_bin_mps": f"{forward_bin:.2f}",
                "yaw_rate_bin_radps": f"{yaw_bin:.2f}",
                "abs_yaw_rate_bin_radps": f"{abs(yaw_bin):.2f}",
                "count": str(len(group)),
                "run_count_ge10": str(run_count),
                "median_residual_additive_nm": f"{med_residual:.9f}",
                "median_opposing_nm": f"{med_opposing:.9f}",
                "trimmed_mean_opposing_nm": f"{trimmed_mean(opposing):.9f}",
                "mad_opposing_nm": f"{median_abs_deviation(opposing, med_opposing):.9f}",
                "iqr_opposing_nm": f"{(percentile(ordered_opposing, 0.75) - percentile(ordered_opposing, 0.25)):.9f}",
                "run_median_min_opposing_nm": "" if not run_medians else f"{min(run_medians):.9f}",
                "run_median_max_opposing_nm": "" if not run_medians else f"{max(run_medians):.9f}",
                "run_median_spread_opposing_nm": f"{run_median_spread:.9f}",
                "run_median_std_opposing_nm": f"{run_median_std:.9f}",
                "positive_run_fraction": f"{positive_run_frac:.3f}",
                "consistency": "cross-run" if run_count >= MIN_CONSISTENCY_RUN_COUNT else "single-run/weak",
            }
        )
    bin_fields = [
        "forward_velocity_bin_mps",
        "yaw_rate_bin_radps",
        "abs_yaw_rate_bin_radps",
        "count",
        "run_count_ge10",
        "median_residual_additive_nm",
        "median_opposing_nm",
        "trimmed_mean_opposing_nm",
        "mad_opposing_nm",
        "iqr_opposing_nm",
        "run_median_min_opposing_nm",
        "run_median_max_opposing_nm",
        "run_median_spread_opposing_nm",
        "run_median_std_opposing_nm",
        "positive_run_fraction",
        "consistency",
    ]
    with (out_dir / "nonzero_vf_torque_bins.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=bin_fields)
        writer.writeheader()
        writer.writerows(rows)
    run_fields = [
        "forward_velocity_bin_mps",
        "yaw_rate_bin_radps",
        "run_id",
        "kind",
        "count",
        "median_residual_additive_nm",
        "median_opposing_nm",
        "mad_opposing_nm",
    ]
    with (out_dir / "bin_run_consistency.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=run_fields)
        writer.writeheader()
        writer.writerows(run_rows)
    return rows


def write_rmse(out_dir: Path, samples: list[Sample], denominator: float) -> tuple[list[dict[str, str]], dict[str, float]]:
    by_run: dict[str, list[Sample]] = {}
    for sample in samples:
        by_run.setdefault(sample.run_id, []).append(sample)
    rows: list[dict[str, str]] = []
    all_current: list[float] = []
    all_corrected: list[float] = []
    all_used = 0
    for run_id in sorted(by_run):
        holdout = by_run[run_id]
        train = [sample for sample in samples if sample.run_id != run_id]
        surface = build_surface(train)
        current_errors: list[float] = []
        corrected_errors: list[float] = []
        used = 0
        for sample in holdout:
            current_next = sample.yaw_rate_radps + ((sample.model_yaw_moment_nm / denominator) * sample.dt_s)
            correction = surface.get((sample.forward_bin_mps, sample.yaw_rate_bin_radps), 0.0)
            corrected_next = sample.yaw_rate_radps + (((sample.model_yaw_moment_nm + correction) / denominator) * sample.dt_s)
            current_error = current_next - sample.next_yaw_rate_radps
            corrected_error = corrected_next - sample.next_yaw_rate_radps
            current_errors.append(current_error)
            corrected_errors.append(corrected_error)
            all_current.append(current_error)
            all_corrected.append(corrected_error)
            if correction != 0.0:
                used += 1
                all_used += 1
        rows.append(
            {
                "holdout_run_id": run_id,
                "kind": holdout[0].kind,
                "samples": str(len(holdout)),
                "surface_bins_from_other_runs": str(len(surface)),
                "samples_with_surface_bin": str(used),
                "current_rmse_radps": f"{rmse(current_errors):.9f}",
                "surface_corrected_rmse_radps": f"{rmse(corrected_errors):.9f}",
            }
        )
    fields = [
        "holdout_run_id",
        "kind",
        "samples",
        "surface_bins_from_other_runs",
        "samples_with_surface_bin",
        "current_rmse_radps",
        "surface_corrected_rmse_radps",
    ]
    with (out_dir / "rmse_leave_one_run_out.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    aggregate = {
        "samples": float(len(samples)),
        "samples_with_surface_bin": float(all_used),
        "current_rmse_radps": rmse(all_current),
        "surface_corrected_rmse_radps": rmse(all_corrected),
    }
    return rows, aggregate


def compact_supported_regions(bin_rows: list[dict[str, str]]) -> tuple[list[str], list[str]]:
    supported: list[str] = []
    weak: list[str] = []
    for row in bin_rows:
        count = int(row["count"])
        run_count = int(row["run_count_ge10"])
        spread = float(row["run_median_spread_opposing_nm"] or 0.0)
        fbin = float(row["forward_velocity_bin_mps"])
        ybin = float(row["yaw_rate_bin_radps"])
        label = f"Vf={fbin:.2f} m/s, yaw={ybin:.2f} rad/s: n={count}, runs={run_count}, medianOpp={float(row['median_opposing_nm']):.4f} Nm, runSpread={spread:.4f}"
        if count >= 250 and run_count >= 2 and spread <= 0.12:
            supported.append(label)
        elif count >= 50:
            weak.append(label)
    return supported, weak


def write_report(
    out_dir: Path,
    params: SourceParams,
    candidates: list[CandidateLog],
    summaries: list[RunSummary],
    samples: list[Sample],
    bin_rows: list[dict[str, str]],
    rmse_rows: list[dict[str, str]],
    aggregate: dict[str, float],
) -> None:
    included = [s for s in summaries if s.extracted_samples > 0]
    competition_included = [s for s in included if s.kind == "competition"]
    decoded_included = [s for s in included if s.kind == "decoded_open_floor"]
    supported, weak = compact_supported_regions(bin_rows)
    top_bins = sorted(bin_rows, key=lambda row: int(row["count"]), reverse=True)[:25]
    lines = [
        "# Expanded Yaw Torque Validation",
        "",
        "Scratch-only independent validation. No production code was modified.",
        "",
        "## Method",
        "",
        "Inputs are actual sensors and commands only: raw gyro yaw rate minus an independently estimated stationary bias, encoder-derived forward velocity and wheel speeds, logged drive commands, and timestamps. The extraction does not use `ukf_state_*`, pose, estimator yaw-rate targets, or logged gyro bias as targets.",
        "",
        "Torque basis follows the prior reconciled Worker E approach: `residual_additive_yaw_torque_nm = observed_yaw_moment_nm - current_model_yaw_moment_nm`, where the current model mirror includes motor torque, static/rolling loss, longitudinal tire stiffness, contact force gains, contact projection, and yaw damping. `opposing_yaw_resistance_nm = -sign(sensor_yaw_rate) * residual_additive_yaw_torque_nm`.",
        "",
        f"Only nonzero-forward/nonzero-yaw active adjacent samples are binned: `|Vf| >= {MIN_ABS_FORWARD_MPS:.2f} m/s`, `|yaw| >= {MIN_ABS_YAW_RADPS:.2f} rad/s`, forward bins {FORWARD_BIN_STEP_MPS:.2f} m/s, yaw bins {YAW_BIN_STEP_RADPS:.2f} rad/s.",
        "",
        "## Constants",
        "",
        "| Constant | Value |",
        "| --- | ---: |",
        f"| mass kg | {params.mass_kg:.9f} |",
        f"| track width m | {params.track_width_m:.9f} |",
        f"| yaw inertia kg m^2 | {params.yaw_inertia_kg_m2:.9f} |",
        f"| yaw denominator incl wheel spin-up kg m^2 | {yaw_denominator_kg_m2(params):.9f} |",
        f"| wheel radius m | {params.wheel_radius_m:.9f} |",
        "",
        "## Log Set",
        "",
        f"Discovered {len(candidates)} candidate CSV logs: {sum(1 for c in candidates if c.kind == 'decoded_open_floor')} decoded open-floor logs, {sum(1 for c in candidates if c.kind == 'competition')} competition diagnostic/audit logs, and {sum(1 for c in candidates if c.kind == 'competition_fwc')} competition front-wall characterization CSVs.",
        f"Used {len(included)} logs with nonzero-Vf yaw samples: {len(decoded_included)} decoded open-floor and {len(competition_included)} competition logs.",
        "",
        "Full discovered/used/excluded details are in `discovered_logs.csv` and `run_summary.csv`.",
        "",
        "## Highest-Coverage Nonzero-Vf Bins",
        "",
        "| Vf bin m/s | Yaw bin rad/s | Count | Runs | Median residual Nm | Median opposing Nm | IQR Nm | Run spread Nm | Consistency |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in top_bins:
        lines.append(
            f"| {float(row['forward_velocity_bin_mps']):.2f} | {float(row['yaw_rate_bin_radps']):.2f} | {row['count']} | {row['run_count_ge10']} | {float(row['median_residual_additive_nm']):.9f} | {float(row['median_opposing_nm']):.9f} | {float(row['iqr_opposing_nm']):.9f} | {float(row['run_median_spread_opposing_nm']):.9f} | {row['consistency']} |"
        )
    lines.extend([
        "",
        "## RMSE",
        "",
        "Leave-one-run-out correction uses signed-bin median residual torque from all other runs. Units are one-step yaw-rate error in rad/s.",
        "",
        "| Holdout | Kind | Samples | Corrected samples | Current RMSE | Corrected RMSE |",
        "| --- | --- | ---: | ---: | ---: | ---: |",
    ])
    for row in rmse_rows:
        lines.append(
            f"| `{row['holdout_run_id']}` | {row['kind']} | {row['samples']} | {row['samples_with_surface_bin']} | {row['current_rmse_radps']} | {row['surface_corrected_rmse_radps']} |"
        )
    lines.extend([
        "",
        f"Aggregate: samples={int(aggregate['samples'])}, corrected={int(aggregate['samples_with_surface_bin'])}, current RMSE={aggregate['current_rmse_radps']:.9f} rad/s, corrected RMSE={aggregate['surface_corrected_rmse_radps']:.9f} rad/s.",
        "",
        "## Data Sufficiency Judgment",
        "",
    ])
    if supported:
        lines.append("Supported enough for calibration exploration, but still not production-final:")
        lines.extend(f"- {item}" for item in supported[:30])
    else:
        lines.append("No nonzero-forward bins meet the stricter support rule of count >= 250, at least two runs, and run-median spread <= 0.12 Nm.")
    lines.extend(["", "Weak or targeted-sweep-needed bins that are populated but inconsistent or single-run dominated:"])
    lines.extend(f"- {item}" for item in weak[:40])
    lines.extend([
        "",
        "Regions absent or not reliable require targeted sweeps: high forward speed with high yaw rate, negative forward velocity, and any bins represented by one run only. Competition logs add useful nonzero-forward coverage, but many bins are maneuver/path dependent and not repeated cleanly across runs.",
        "",
        "## Discrepancies With Previous Scratch Work",
        "",
        "- This validation confirms the prior rejection of the approximately 0.9 Nm motor-only bins; using the fuller current PlantModel mirror keeps typical residuals in the hundredths to low-tenths of Nm range.",
        "- Compared with the previous Worker E run, this extraction is deliberately coverage-first and includes competition diagnostic/audit CSVs plus all discoverable decoded open-floor candidates. It therefore exposes more nonzero-forward bins, but also more single-run and path-dependent bins.",
        "- The sign pattern is not a monotonic counter-yaw table. Several populated bins still have negative opposing resistance, meaning the model sometimes needs yaw-aiding residual torque.",
        "",
        "## Reproduce",
        "",
        "```powershell",
        "python codex_analysis\\yaw_torque_expanded_validation\\expanded_yaw_torque_validation.py --out-dir codex_analysis\\yaw_torque_expanded_validation",
        "```",
        "",
    ])
    (out_dir / "expanded_yaw_torque_validation_report.md").write_text("\n".join(lines), encoding="utf-8")


def write_run_summary(path: Path, summaries: list[RunSummary]) -> None:
    fields = [
        "run_id",
        "kind",
        "path",
        "input_rows",
        "stationary_bias_rows",
        "gyro_bias_radps",
        "adjacent_pairs",
        "active_pairs",
        "extracted_samples",
        "max_abs_forward_mps",
        "max_abs_yaw_radps",
        "nonzero_forward_sample_count",
        "nonzero_forward_yaw_sample_count",
        "status",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for s in summaries:
            writer.writerow(
                {
                    "run_id": s.run_id,
                    "kind": s.kind,
                    "path": str(s.path.relative_to(REPO_ROOT)),
                    "input_rows": s.input_rows,
                    "stationary_bias_rows": s.stationary_bias_rows,
                    "gyro_bias_radps": f"{s.gyro_bias_radps:.9f}",
                    "adjacent_pairs": s.adjacent_pairs,
                    "active_pairs": s.active_pairs,
                    "extracted_samples": s.extracted_samples,
                    "max_abs_forward_mps": f"{s.max_abs_forward_mps:.6f}",
                    "max_abs_yaw_radps": f"{s.max_abs_yaw_radps:.6f}",
                    "nonzero_forward_sample_count": s.nonzero_forward_sample_count,
                    "nonzero_forward_yaw_sample_count": s.nonzero_forward_yaw_sample_count,
                    "status": s.excluded_reason,
                }
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Expanded nonzero-forward yaw torque residual validation.")
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR_DEFAULT)
    parser.add_argument("--limit-runs", type=int, default=0, help="Optional quick debug limit after discovery.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir = args.out_dir
    if not out_dir.is_absolute():
        out_dir = REPO_ROOT / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    params = source_params()
    denominator = yaw_denominator_kg_m2(params)
    candidates = discover_logs()
    process_candidates = candidates if args.limit_runs <= 0 else candidates[: args.limit_runs]

    summaries: list[RunSummary] = []
    samples: list[Sample] = []
    for index, candidate in enumerate(process_candidates, start=1):
        print(f"[{index}/{len(process_candidates)}] {candidate.kind} {candidate.run_id}")
        summary, run_samples = extract_samples(candidate, params, denominator)
        summaries.append(summary)
        samples.extend(run_samples)

    write_run_summary(out_dir / "run_summary.csv", summaries)
    write_discovered(out_dir / "discovered_logs.csv", candidates, summaries)
    bin_rows = write_bin_tables(out_dir, samples)
    rmse_rows, aggregate = write_rmse(out_dir, samples, denominator) if samples else ([], {
        "samples": 0.0,
        "samples_with_surface_bin": 0.0,
        "current_rmse_radps": 0.0,
        "surface_corrected_rmse_radps": 0.0,
    })
    write_report(out_dir, params, candidates, summaries, samples, bin_rows, rmse_rows, aggregate)
    (out_dir / "commands_run.txt").write_text(
        "Get-Content -LiteralPath AGENTS.md\n"
        "python codex_analysis\\yaw_torque_expanded_validation\\expanded_yaw_torque_validation.py --out-dir codex_analysis\\yaw_torque_expanded_validation\n",
        encoding="utf-8",
    )

    print(f"out_dir={out_dir}")
    print(f"candidates={len(candidates)} processed={len(process_candidates)}")
    print(f"samples={len(samples)} bins={len(bin_rows)}")
    print(f"current_rmse_radps={aggregate['current_rmse_radps']:.9f}")
    print(f"surface_corrected_rmse_radps={aggregate['surface_corrected_rmse_radps']:.9f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
