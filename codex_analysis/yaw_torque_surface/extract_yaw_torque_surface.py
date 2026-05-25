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
DEFAULT_RUNS = [
    "2026-04-21_05-32-06",
    "2026-04-20_08-38-39",
    "2026-05-04_20-35-47",
    "2026-05-04_16-57-53",
    "2026-04-21_05-59-46",
    "2026-04-21_00-16-10",
    "2026-04-21_01-09-34",
    "2026-04-20_02-33-07",
    "2026-04-20_12-10-58",
    "2026-04-14_04-43-48",
]


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
class RunWindow:
    run_id: str
    csv_path: Path
    input_rows: int
    kept_rows: int
    gyro_bias_radps: float
    stationary_bias_rows: int
    extracted_samples: int
    cutoff_reason: str
    cutoff_first_tick: int | None
    cutoff_first_time_us: int | None


@dataclass(frozen=True)
class Sample:
    run_id: str
    run_kind: str
    row_index: int
    tick: int
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
class BinStats:
    forward_bin_mps: float
    yaw_rate_bin_radps: float
    abs_yaw_rate_bin_radps: float
    count: int
    median_residual_additive_nm: float
    median_opposing_nm: float
    trimmed_mean_opposing_nm: float
    mad_opposing_nm: float
    iqr_opposing_nm: float


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
    return int(text)


def median_abs_deviation(values: list[float], center: float | None = None) -> float:
    if not values:
        return 0.0
    resolved_center = statistics.median(values) if center is None else center
    return statistics.median(abs(value - resolved_center) for value in values)


def robust_bias(values: list[float]) -> tuple[float, int]:
    if not values:
        return 0.0, 0
    center = statistics.median(values)
    mad = median_abs_deviation(values, center)
    threshold = max(0.015, 5.0 * 1.4826 * mad)
    kept = [value for value in values if abs(value - center) <= threshold]
    if not kept:
        return center, len(values)
    return statistics.median(kept), len(kept)


def estimate_stationary_raw_gyro_bias(csv_path: Path) -> tuple[float, int, int]:
    stationary: list[float] = []
    rows = 0
    with csv_path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows += 1
            try:
                dt_us = finite_int(row, "dt_us", 1000)
                left_command = finite_float(row, "left_drive_command")
                right_command = finite_float(row, "right_drive_command")
                left_velocity = finite_float(row, "left_encoder_velocity_mps")
                right_velocity = finite_float(row, "right_encoder_velocity_mps")
                gyro_raw = finite_float(row, "gyro_raw_radps")
            except ValueError:
                continue
            if not (500 <= dt_us <= 3000):
                continue
            if max(abs(left_command), abs(right_command)) > 0.02:
                continue
            if max(abs(left_velocity), abs(right_velocity)) > 0.015:
                continue
            if abs(gyro_raw) > 0.5:
                continue
            stationary.append(gyro_raw)
    bias, kept = robust_bias(stationary)
    return bias, kept, rows


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
    applied = 0.0
    launch_direction = signed_direction(limited, wheel_speed_radps)
    if abs(limited) > launch_torque_nm:
        applied = limited - (launch_direction * launch_torque_nm)
    loss_direction = signed_direction(wheel_speed_radps, applied)
    rolling_loss = (
        params.rolling_friction_torque_nm * loss_direction
    ) + (params.viscous_friction_nm_per_radps * wheel_speed_radps)
    return applied - rolling_loss


def yaw_denominator_kg_m2(params: SourceParams) -> float:
    wheel_spinup_mass_kg = (2.0 * params.wheel_bank_inertia_kg_m2) / (params.wheel_radius_m * params.wheel_radius_m)
    half_track = 0.5 * abs(params.track_width_m)
    return params.yaw_inertia_kg_m2 + (wheel_spinup_mass_kg * half_track * half_track)


def current_model_yaw_moment_nm(
    row: dict[str, str],
    forward_velocity_mps: float,
    yaw_rate_radps: float,
    params: SourceParams,
) -> float:
    left_velocity_mps = finite_float(row, "left_encoder_velocity_mps")
    right_velocity_mps = finite_float(row, "right_encoder_velocity_mps")
    left_wheel_speed_radps = finite_float(row, "left_encoder_omega_radps")
    right_wheel_speed_radps = finite_float(row, "right_encoder_omega_radps")
    left_command = finite_float(row, "left_drive_command")
    right_command = finite_float(row, "right_drive_command")

    left_torque_nm = applied_bank_torque(left_command, left_wheel_speed_radps, params)
    right_torque_nm = applied_bank_torque(right_command, right_wheel_speed_radps, params)
    left_drive_force_n = left_torque_nm / params.wheel_radius_m
    right_drive_force_n = right_torque_nm / params.wheel_radius_m

    fan_duty = max(0.0, min(1.0, finite_float(row, "fan_duty_cycle", 0.8)))
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
    left_body_forward_mps = forward_velocity_mps + (half_track * yaw_rate_radps)
    right_body_forward_mps = forward_velocity_mps - (half_track * yaw_rate_radps)
    left_forward_rel_mps = left_velocity_mps - left_body_forward_mps
    right_forward_rel_mps = right_velocity_mps - right_body_forward_mps

    longitudinal_offset_m = abs(params.drive_wheel_longitudinal_offset_m)
    front_right_body_velocity_mps = longitudinal_offset_m * yaw_rate_radps
    rear_right_body_velocity_mps = -longitudinal_offset_m * yaw_rate_radps
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
    yaw_moment_nm = (
        half_track * (left_bank_forward_n - right_bank_forward_n)
    ) + (longitudinal_offset_m * (front_right_force_n - rear_right_force_n))
    return yaw_moment_nm - (params.yaw_rate_damping_nms_per_rad * yaw_rate_radps)


def row_key(row: dict[str, str]) -> tuple[str, ...]:
    return (
        row.get("section_id", ""),
        row.get("phase_id", ""),
        row.get("primitive_id", ""),
        row.get("speed_bin", ""),
        row.get("repeat_index", ""),
    )


def is_analysis_row(row: dict[str, str]) -> tuple[bool, str]:
    section = row.get("section_id")
    phase = row.get("phase_id", "")
    if section == "4" and phase in {"8", "9"}:
        return True, "in_place_yaw"
    if section == "5" and phase in {"10", "11", "12"}:
        return True, "moving_yaw"
    if section in {None, ""} and phase == "20":
        return True, "in_place_yaw"
    return False, ""


def round_to_step(value: float, step: float) -> float:
    if abs(value) < 0.5 * step:
        return 0.0
    return round(value / step) * step


def valid_adjacent(current: dict[str, str], nxt: dict[str, str]) -> bool:
    try:
        if finite_int(nxt, "control_tick_sequence") <= finite_int(current, "control_tick_sequence"):
            return False
        dt_us = finite_int(nxt, "dt_us")
        if not (500 <= dt_us <= 3000):
            return False
        if row_key(current) != row_key(nxt):
            return False
        if finite_int(current, "saturation_flags", 0) != 0 or finite_int(nxt, "saturation_flags", 0) != 0:
            return False
        if finite_int(current, "watchdog_flags", 0) != 0 or finite_int(nxt, "watchdog_flags", 0) != 0:
            return False
    except ValueError:
        return False
    return True


def is_tail_quiescent_or_invalid(row: dict[str, str], bias_radps: float) -> bool:
    try:
        tick = finite_int(row, "control_tick_sequence")
        dt_us = finite_int(row, "dt_us", 1000)
        left_command = finite_float(row, "left_drive_command")
        right_command = finite_float(row, "right_drive_command")
        left_velocity = finite_float(row, "left_encoder_velocity_mps")
        right_velocity = finite_float(row, "right_encoder_velocity_mps")
        yaw_rate = finite_float(row, "gyro_raw_radps") - bias_radps
    except ValueError:
        return True
    if tick < 0 or not (500 <= dt_us <= 3000):
        return True
    return (
        max(abs(left_command), abs(right_command)) <= 0.03 and
        max(abs(left_velocity), abs(right_velocity)) <= 0.03 and
        abs(yaw_rate) <= 0.08
    )


def extract_run_samples(csv_path: Path, bias_radps: float, params: SourceParams) -> tuple[RunWindow, list[Sample]]:
    pending: dict[str, str] | None = None
    pending_index = -1
    input_rows = 0
    samples: list[Sample] = []
    tail_start_index: int | None = None
    tail_start_tick: int | None = None
    tail_start_time_us: int | None = None
    tail_start_master_us: int | None = None
    last_time_us: int | None = None

    with csv_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            input_rows += 1
            row_index = input_rows - 1
            try:
                master_time_us = finite_int(row, "master_time_us")
            except ValueError:
                master_time_us = last_time_us if last_time_us is not None else 0
            if is_tail_quiescent_or_invalid(row, bias_radps):
                if tail_start_index is None:
                    tail_start_index = row_index
                    tail_start_tick = finite_int(row, "control_tick_sequence", -1)
                    tail_start_time_us = master_time_us
                    tail_start_master_us = master_time_us
            else:
                tail_start_index = None
                tail_start_tick = None
                tail_start_time_us = None
                tail_start_master_us = None
            last_time_us = master_time_us

            if pending is not None:
                use_row, run_kind = is_analysis_row(pending)
                if use_row and valid_adjacent(pending, row):
                    try:
                        dt_s = 1.0e-6 * finite_int(row, "dt_us")
                        left_velocity = finite_float(pending, "left_encoder_velocity_mps")
                        right_velocity = finite_float(pending, "right_encoder_velocity_mps")
                        forward_velocity = 0.5 * (left_velocity + right_velocity)
                        yaw_rate = finite_float(pending, "gyro_raw_radps") - bias_radps
                        next_yaw_rate = finite_float(row, "gyro_raw_radps") - bias_radps
                        measured_alpha = (next_yaw_rate - yaw_rate) / dt_s
                        model_yaw_moment = current_model_yaw_moment_nm(pending, forward_velocity, yaw_rate, params)
                    except ValueError:
                        pending = row
                        pending_index = row_index
                        continue
                    if all(math.isfinite(value) for value in (forward_velocity, yaw_rate, next_yaw_rate, measured_alpha, model_yaw_moment)):
                        denom = yaw_denominator_kg_m2(params)
                        observed_yaw_moment = denom * measured_alpha
                        residual = observed_yaw_moment - model_yaw_moment
                        yaw_sign = signed_direction(yaw_rate)
                        if yaw_sign != 0.0 and abs(yaw_rate) >= 0.05:
                            opposing = -yaw_sign * residual
                            samples.append(
                                Sample(
                                    run_id=csv_path.parent.name.replace("mmlog_decode_", ""),
                                    run_kind=run_kind,
                                    row_index=pending_index,
                                    tick=finite_int(pending, "control_tick_sequence"),
                                    time_us=finite_int(pending, "master_time_us"),
                                    dt_s=dt_s,
                                    forward_velocity_mps=forward_velocity,
                                    yaw_rate_radps=yaw_rate,
                                    next_yaw_rate_radps=next_yaw_rate,
                                    measured_yaw_accel_radps2=measured_alpha,
                                    model_yaw_moment_nm=model_yaw_moment,
                                    residual_additive_yaw_torque_nm=residual,
                                    opposing_yaw_resistance_nm=opposing,
                                    forward_bin_mps=round_to_step(forward_velocity, 0.10),
                                    yaw_rate_bin_radps=round_to_step(yaw_rate, 0.50),
                                    abs_yaw_rate_bin_radps=round_to_step(abs(yaw_rate), 0.50),
                                )
                            )
            pending = row
            pending_index = row_index

    cutoff_index = input_rows
    cutoff_reason = "sensor tail check kept all rows"
    cutoff_tick = None
    cutoff_time = None
    if tail_start_index is not None and tail_start_master_us is not None and last_time_us is not None:
        tail_duration_s = 1.0e-6 * max(0, last_time_us - tail_start_master_us)
        if tail_duration_s >= 0.25:
            cutoff_index = tail_start_index
            cutoff_reason = "dropped trailing sensor-quiescent/invalid tail >= 0.25 s"
            cutoff_tick = tail_start_tick
            cutoff_time = tail_start_time_us
            samples = [sample for sample in samples if sample.row_index < cutoff_index]

    return (
        RunWindow(
            run_id=csv_path.parent.name.replace("mmlog_decode_", ""),
            csv_path=csv_path,
            input_rows=input_rows,
            kept_rows=cutoff_index,
            gyro_bias_radps=bias_radps,
            stationary_bias_rows=0,
            extracted_samples=len(samples),
            cutoff_reason=cutoff_reason,
            cutoff_first_tick=cutoff_tick,
            cutoff_first_time_us=cutoff_time,
        ),
        samples,
    )


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


def make_bin_stats(samples: Iterable[Sample], min_count: int = 25) -> list[BinStats]:
    grouped: dict[tuple[float, float], list[Sample]] = {}
    for sample in samples:
        key = (sample.forward_bin_mps, sample.yaw_rate_bin_radps)
        grouped.setdefault(key, []).append(sample)
    stats: list[BinStats] = []
    for (forward_bin, yaw_bin), group in grouped.items():
        if len(group) < min_count:
            continue
        opposing = [sample.opposing_yaw_resistance_nm for sample in group]
        residual = [sample.residual_additive_yaw_torque_nm for sample in group]
        ordered = sorted(opposing)
        median_opposing = statistics.median(opposing)
        q1 = percentile(ordered, 0.25)
        q3 = percentile(ordered, 0.75)
        stats.append(
            BinStats(
                forward_bin_mps=forward_bin,
                yaw_rate_bin_radps=yaw_bin,
                abs_yaw_rate_bin_radps=abs(yaw_bin),
                count=len(group),
                median_residual_additive_nm=statistics.median(residual),
                median_opposing_nm=median_opposing,
                trimmed_mean_opposing_nm=trimmed_mean(opposing),
                mad_opposing_nm=median_abs_deviation(opposing, median_opposing),
                iqr_opposing_nm=q3 - q1,
            )
        )
    return sorted(stats, key=lambda item: (item.forward_bin_mps, item.yaw_rate_bin_radps))


def rmse(values: Iterable[float]) -> float:
    materialized = list(values)
    if not materialized:
        return 0.0
    return math.sqrt(statistics.fmean(value * value for value in materialized))


def build_signed_residual_surface(samples: Iterable[Sample], min_count: int = 25) -> dict[tuple[float, float], float]:
    grouped: dict[tuple[float, float], list[float]] = {}
    for sample in samples:
        key = (sample.forward_bin_mps, sample.yaw_rate_bin_radps)
        grouped.setdefault(key, []).append(sample.residual_additive_yaw_torque_nm)
    surface: dict[tuple[float, float], float] = {}
    for key, values in grouped.items():
        if len(values) >= min_count:
            surface[key] = statistics.median(values)
    return surface


def correction_for_sample(sample: Sample, surface: dict[tuple[float, float], float]) -> float:
    return surface.get((sample.forward_bin_mps, sample.yaw_rate_bin_radps), 0.0)


def evaluate(samples: list[Sample]) -> tuple[list[dict[str, str]], dict[str, float]]:
    denom = None
    by_run: dict[str, list[Sample]] = {}
    for sample in samples:
        by_run.setdefault(sample.run_id, []).append(sample)
    rows: list[dict[str, str]] = []
    aggregate_current_errors: list[float] = []
    aggregate_corrected_errors: list[float] = []
    aggregate_used = 0
    aggregate_total = 0
    for run_id in sorted(by_run):
        holdout = by_run[run_id]
        train = [sample for sample in samples if sample.run_id != run_id]
        surface = build_signed_residual_surface(train)
        current_errors: list[float] = []
        corrected_errors: list[float] = []
        used = 0
        for sample in holdout:
            if denom is None:
                denom = (sample.model_yaw_moment_nm + sample.residual_additive_yaw_torque_nm) / sample.measured_yaw_accel_radps2 if sample.measured_yaw_accel_radps2 else None
            current_next = sample.yaw_rate_radps + ((sample.model_yaw_moment_nm / YAW_DENOMINATOR_KG_M2) * sample.dt_s)
            correction = correction_for_sample(sample, surface)
            corrected_next = sample.yaw_rate_radps + (((sample.model_yaw_moment_nm + correction) / YAW_DENOMINATOR_KG_M2) * sample.dt_s)
            current_error = current_next - sample.next_yaw_rate_radps
            corrected_error = corrected_next - sample.next_yaw_rate_radps
            current_errors.append(current_error)
            corrected_errors.append(corrected_error)
            aggregate_current_errors.append(current_error)
            aggregate_corrected_errors.append(corrected_error)
            aggregate_total += 1
            if correction != 0.0:
                used += 1
                aggregate_used += 1
        rows.append(
            {
                "run_id": run_id,
                "samples": str(len(holdout)),
                "surface_bins_from_other_runs": str(len(surface)),
                "samples_with_surface_bin": str(used),
                "current_rmse_radps": f"{rmse(current_errors):.9f}",
                "surface_corrected_rmse_radps": f"{rmse(corrected_errors):.9f}",
            }
        )
    aggregate = {
        "samples": float(aggregate_total),
        "samples_with_surface_bin": float(aggregate_used),
        "current_rmse_radps": rmse(aggregate_current_errors),
        "surface_corrected_rmse_radps": rmse(aggregate_corrected_errors),
    }
    return rows, aggregate


def write_bin_csv(path: Path, stats: list[BinStats]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "forward_velocity_bin_mps",
                "yaw_rate_bin_radps",
                "abs_yaw_rate_bin_radps",
                "count",
                "median_residual_additive_yaw_torque_nm",
                "median_opposing_yaw_resistance_nm",
                "trimmed_mean_opposing_yaw_resistance_nm",
                "mad_opposing_yaw_resistance_nm",
                "iqr_opposing_yaw_resistance_nm",
            ],
        )
        writer.writeheader()
        for item in stats:
            writer.writerow(
                {
                    "forward_velocity_bin_mps": f"{item.forward_bin_mps:.2f}",
                    "yaw_rate_bin_radps": f"{item.yaw_rate_bin_radps:.2f}",
                    "abs_yaw_rate_bin_radps": f"{item.abs_yaw_rate_bin_radps:.2f}",
                    "count": item.count,
                    "median_residual_additive_yaw_torque_nm": f"{item.median_residual_additive_nm:.9f}",
                    "median_opposing_yaw_resistance_nm": f"{item.median_opposing_nm:.9f}",
                    "trimmed_mean_opposing_yaw_resistance_nm": f"{item.trimmed_mean_opposing_nm:.9f}",
                    "mad_opposing_yaw_resistance_nm": f"{item.mad_opposing_nm:.9f}",
                    "iqr_opposing_yaw_resistance_nm": f"{item.iqr_opposing_nm:.9f}",
                }
            )


def write_rmse_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "run_id",
                "samples",
                "surface_bins_from_other_runs",
                "samples_with_surface_bin",
                "current_rmse_radps",
                "surface_corrected_rmse_radps",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)


def write_window_csv(path: Path, windows: list[RunWindow]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "run_id",
                "csv_path",
                "input_rows",
                "kept_rows",
                "gyro_bias_radps",
                "stationary_bias_rows",
                "extracted_samples",
                "cutoff_reason",
                "cutoff_first_tick",
                "cutoff_first_time_us",
            ],
        )
        writer.writeheader()
        for window in windows:
            writer.writerow(
                {
                    "run_id": window.run_id,
                    "csv_path": str(window.csv_path.relative_to(REPO_ROOT)),
                    "input_rows": window.input_rows,
                    "kept_rows": window.kept_rows,
                    "gyro_bias_radps": f"{window.gyro_bias_radps:.9f}",
                    "stationary_bias_rows": window.stationary_bias_rows,
                    "extracted_samples": window.extracted_samples,
                    "cutoff_reason": window.cutoff_reason,
                    "cutoff_first_tick": "" if window.cutoff_first_tick is None else window.cutoff_first_tick,
                    "cutoff_first_time_us": "" if window.cutoff_first_time_us is None else window.cutoff_first_time_us,
                }
            )


def compact_symmetric_table(stats: list[BinStats]) -> list[tuple[float, float, int, float, float]]:
    grouped: dict[tuple[float, float], list[BinStats]] = {}
    for item in stats:
        grouped.setdefault((item.forward_bin_mps, item.abs_yaw_rate_bin_radps), []).append(item)
    rows = []
    for (forward_bin, abs_yaw_bin), items in grouped.items():
        count = sum(item.count for item in items)
        if count < 250 or abs_yaw_bin < 0.5:
            continue
        weighted_median = sum(item.median_opposing_nm * item.count for item in items) / count
        weighted_iqr = sum(item.iqr_opposing_nm * item.count for item in items) / count
        rows.append((forward_bin, abs_yaw_bin, count, weighted_median, weighted_iqr))
    return sorted(rows, key=lambda row: (abs(row[0]), row[0], row[1]))


def markdown_report(
    params: SourceParams,
    windows: list[RunWindow],
    stats: list[BinStats],
    rmse_rows: list[dict[str, str]],
    aggregate: dict[str, float],
) -> str:
    compact_rows = compact_symmetric_table(stats)
    lines = [
        "# Yaw Torque Resistance Surface",
        "",
        "## Method",
        "",
        "This scratch analysis uses decoded `open_floor_main.csv` logs and sensor-derived targets only: raw gyro yaw rate minus a per-run independently recomputed stationary bias, encoder-derived forward velocity, encoder wheel speeds, and logged drive commands. It does not use `gyro_bias_radps`, `gyro_bias_anchor_radps`, `ukf_state_*`, or estimator diagnostics as targets.",
        "",
        "Sign convention: `+Yaw` is clockwise. The scratch PlantModel mirror computes positive yaw moment as `0.5 * track_width * (left_forward_force - right_forward_force)` plus the front/rear right-force moment. Per sample, `residual_additive_yaw_torque_nm = observed_yaw_moment_nm - current_model_yaw_moment_nm`; this is the torque that must be added to the current model to match observed yaw acceleration. The reported opposing-yaw resistance is `-sign(sensor_yaw_rate) * residual_additive_yaw_torque_nm`, so positive values oppose the current yaw motion.",
        "",
        "Current-model yaw moment mirrors the yaw-relevant production equations from `PlantModel`: MotorEncoderDrive command-to-torque, static launch and rolling loss, longitudinal tire stiffness, front/rear right-contact force gains, and the single sustained-lateral-acceleration-derived contact `mu` projection. Lateral velocity is not independently observable in these logs, so the sensor-only replay assumes rightward body velocity is zero.",
        "",
        "Bins: forward velocity is rounded to 0.10 m/s; yaw rate is rounded to 0.50 rad/s. The correction evaluation uses the signed-bin median of `residual_additive_yaw_torque_nm` from all other runs, because that is the directly measured torque correction needed by the current model. The opposing-resistance columns are reported separately to show whether the residual acts against or with the current yaw motion.",
        "",
        "## Authoritative Constants Read",
        "",
        "| Constant | Value |",
        "| --- | ---: |",
        f"| mass kg | {params.mass_kg:.9f} |",
        f"| track width m | {params.track_width_m:.9f} |",
        f"| yaw inertia kg m^2 | {params.yaw_inertia_kg_m2:.9f} |",
        f"| yaw denominator incl wheel spin-up kg m^2 | {yaw_denominator_kg_m2(params):.9f} |",
        f"| wheel radius m | {params.wheel_radius_m:.9f} |",
        f"| sustained lateral accel used for current mu m/s^2 | {params.sustained_lateral_accel_mps2:.6f} |",
        "",
        "## Logs And Sensor Tail Cut",
        "",
        "| Run | Rows | Kept | Bias rows | Bias rad/s | Samples | Cutoff | First dropped tick/time |",
        "| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |",
    ]
    for window in windows:
        first_dropped = "" if window.cutoff_first_tick is None else f"{window.cutoff_first_tick} / {window.cutoff_first_time_us}"
        lines.append(
            f"| `{window.run_id}` | {window.input_rows} | {window.kept_rows} | {window.stationary_bias_rows} | {window.gyro_bias_radps:.9f} | {window.extracted_samples} | {window.cutoff_reason} | {first_dropped} |"
        )
    lines.extend([
        "",
        "Stationary bias rows are detected independently with near-zero drive command, near-zero encoder velocity, valid control timing, and robust median/MAD trimming of raw gyro. Bad-tail cutting is sensor-only: a final contiguous tail is dropped only when it is quiescent or invalid for at least 0.25 s. Many interrupted logs end while still moving; those are kept because the sensors do not prove a bad tail.",
        "",
        "## Compact Opposing Torque Table",
        "",
        "Full signed-bin table is in `yaw_torque_surface_bins.csv`. Values below combine positive and negative yaw-rate bins at the same absolute yaw-rate bin and show weighted median opposing resistance.",
        "",
        "| Forward bin m/s | Abs yaw-rate bin rad/s | Count | Median opposing Nm | IQR Nm |",
        "| ---: | ---: | ---: | ---: | ---: |",
    ])
    for forward_bin, abs_yaw_bin, count, median_opposing, iqr in compact_rows:
        lines.append(f"| {forward_bin:.2f} | {abs_yaw_bin:.2f} | {count} | {median_opposing:.9f} | {iqr:.9f} |")
    lines.extend([
        "",
        "## Leave-One-Run-Out RMSE",
        "",
        "| Holdout run | Samples | Surface bins | Samples corrected | Current RMSE rad/s | Surface RMSE rad/s |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in rmse_rows:
        lines.append(
            f"| `{row['run_id']}` | {row['samples']} | {row['surface_bins_from_other_runs']} | {row['samples_with_surface_bin']} | {row['current_rmse_radps']} | {row['surface_corrected_rmse_radps']} |"
        )
    lines.extend([
        "",
        f"Aggregate holdout samples: {int(aggregate['samples'])}; corrected by non-empty surface bins: {int(aggregate['samples_with_surface_bin'])}.",
        "",
        f"- Current one-step yaw-rate RMSE: {aggregate['current_rmse_radps']:.9f} rad/s",
        f"- Surface-corrected one-step yaw-rate RMSE: {aggregate['surface_corrected_rmse_radps']:.9f} rad/s",
        "",
        "## Recommendation",
        "",
        "The historical sensor data supports a velocity/yaw-rate-dependent residual yaw-torque structure, but not a clean production-ready counter-yaw resistance surface yet. The robust per-bin medians vary materially with yaw-rate and forward speed, but several high-count bins report negative opposing resistance, meaning the current model needs yaw-aiding torque rather than extra resistance in those bins. Treat this as evidence that the single-CoF model is incomplete, not as enough evidence to install a simple monotonic counter-yaw table.",
        "",
        "If this is implemented later, keep ownership in the existing authorities: `Vehicle` should continue to own physical construction facts and fixed hardware capabilities; `PlantModel` should own the plant equation that maps velocity/yaw-rate/contact state to tire-limited yaw resistance. Do not add a new generic CoF/safety-limit owner. A production change should replace the single sustained-acceleration-derived `mu` use inside `PlantModel` with a compact, directly testable tire/contact resistance model derived from authoritative Vehicle/MotorEncoderDrive facts and calibrated data.",
    ])
    return "\n".join(lines) + "\n"


YAW_DENOMINATOR_KG_M2 = 1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract a sensor-derived yaw torque resistance surface from open-floor logs.")
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--runs", nargs="*", default=DEFAULT_RUNS)
    return parser.parse_args()


def main() -> int:
    global YAW_DENOMINATOR_KG_M2
    args = parse_args()
    params = source_params()
    YAW_DENOMINATOR_KG_M2 = yaw_denominator_kg_m2(params)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    windows: list[RunWindow] = []
    samples: list[Sample] = []
    for run in args.runs:
        csv_path = REPO_ROOT / "TestResults" / f"mmlog_decode_{run}" / "open_floor_main.csv"
        if not csv_path.is_file():
            raise FileNotFoundError(csv_path)
        bias, bias_rows, _ = estimate_stationary_raw_gyro_bias(csv_path)
        window, run_samples = extract_run_samples(csv_path, bias, params)
        windows.append(
            RunWindow(
                run_id=window.run_id,
                csv_path=window.csv_path,
                input_rows=window.input_rows,
                kept_rows=window.kept_rows,
                gyro_bias_radps=bias,
                stationary_bias_rows=bias_rows,
                extracted_samples=len(run_samples),
                cutoff_reason=window.cutoff_reason,
                cutoff_first_tick=window.cutoff_first_tick,
                cutoff_first_time_us=window.cutoff_first_time_us,
            )
        )
        samples.extend(run_samples)
    if not samples:
        raise SystemExit("No yaw torque samples found.")

    stats = make_bin_stats(samples)
    rmse_rows, aggregate = evaluate(samples)

    write_window_csv(args.out_dir / "yaw_torque_surface_windows.csv", windows)
    write_bin_csv(args.out_dir / "yaw_torque_surface_bins.csv", stats)
    write_rmse_csv(args.out_dir / "yaw_torque_surface_rmse.csv", rmse_rows)
    report = markdown_report(params, windows, stats, rmse_rows, aggregate)
    (args.out_dir / "yaw_torque_surface_report.md").write_text(report, encoding="utf-8")

    print(f"out_dir={args.out_dir}")
    print(f"runs={len(windows)}")
    print(f"samples={len(samples)}")
    print(f"bins={len(stats)}")
    print(f"current_rmse_radps={aggregate['current_rmse_radps']:.9f}")
    print(f"surface_corrected_rmse_radps={aggregate['surface_corrected_rmse_radps']:.9f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
