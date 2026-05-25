#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import math
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[3]
OUT_DIR = Path(__file__).resolve().parent


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
class LogCandidate:
    run_id: str
    family: str
    schema: str
    path: Path


@dataclass(frozen=True)
class NormalizedRow:
    row_index: int
    raw: dict[str, str]
    time_us: int
    tick: int
    dt_us: int
    phase_id: str
    section_id: str
    primitive_id: str
    speed_bin: str
    repeat_index: str
    left_command: float
    right_command: float
    left_velocity_mps: float
    right_velocity_mps: float
    left_wheel_speed_radps: float
    right_wheel_speed_radps: float
    gyro_raw_radps: float
    stationary_flag: bool
    clipping_flags: int
    saturation_flags: int
    watchdog_flags: int
    fan_duty: float
    fan_source: str


@dataclass
class RunSummary:
    run_id: str
    family: str
    schema: str
    path: Path
    input_rows: int = 0
    normalized_rows: int = 0
    kept_rows: int = 0
    dropped_tail_rows: int = 0
    bias_radps: float = 0.0
    bias_rows: int = 0
    extracted_samples: int = 0
    sampled_rows_written: int = 0
    max_abs_forward_mps: float = 0.0
    max_abs_yaw_radps: float = 0.0
    max_vbar_rel_mps: float = 0.0
    max_force_utilization: float = 0.0
    saturated_rows: int = 0
    limiter_active_rows: int = 0
    cutoff_reason: str = ""
    cutoff_tick: int | None = None
    cutoff_time_us: int | None = None
    limitation: str = ""


@dataclass
class MomentFeatures:
    left_applied_torque_nm: float
    right_applied_torque_nm: float
    left_drive_force_n: float
    right_drive_force_n: float
    left_raw_current_a: float
    right_raw_current_a: float
    left_unit_command_clip: int
    right_unit_command_clip: int
    total_normal_load_n: float
    fl_normal_n: float
    fr_normal_n: float
    rl_normal_n: float
    rr_normal_n: float
    fl_v_rel_f_mps: float
    fr_v_rel_f_mps: float
    rl_v_rel_f_mps: float
    rr_v_rel_f_mps: float
    fl_v_rel_r_mps: float
    fr_v_rel_r_mps: float
    rl_v_rel_r_mps: float
    rr_v_rel_r_mps: float
    vbar_rel_mps: float
    vbar_lat_mps: float
    vbar_yaw_mps: float
    fl_req_f_n: float
    fr_req_f_n: float
    rl_req_f_n: float
    rr_req_f_n: float
    fl_req_r_n: float
    fr_req_r_n: float
    rl_req_r_n: float
    rr_req_r_n: float
    fl_force_f_n: float
    fr_force_f_n: float
    rl_force_f_n: float
    rr_force_f_n: float
    fl_force_r_n: float
    fr_force_r_n: float
    rl_force_r_n: float
    rr_force_r_n: float
    max_force_preprojection_utilization: float
    max_force_limiter_activity: float
    max_force_saturation_capped: float
    model_yaw_moment_nm: float
    diagnostic_encoder_yaw_rate_radps: float
    diagnostic_surface_delta_mps: float


@dataclass
class ValueAgg:
    count: int = 0
    sum_residual: float = 0.0
    sum_abs_residual: float = 0.0
    sum_sq_residual: float = 0.0
    sum_opposing: float = 0.0
    sum_vbar_rel: float = 0.0
    sum_vbar_lat: float = 0.0
    sum_vbar_yaw: float = 0.0
    sum_force_util: float = 0.0
    limiter_active: int = 0
    saturated: int = 0
    residuals: list[float] = field(default_factory=list)
    opposing: list[float] = field(default_factory=list)
    families: set[str] = field(default_factory=set)
    runs: set[str] = field(default_factory=set)

    def add(
        self,
        residual: float,
        opposing: float,
        vbar_rel: float,
        vbar_lat: float,
        vbar_yaw: float,
        force_util: float,
        limiter_active: bool,
        saturated: bool,
        family: str,
        run_id: str,
    ) -> None:
        self.count += 1
        self.sum_residual += residual
        self.sum_abs_residual += abs(residual)
        self.sum_sq_residual += residual * residual
        self.sum_opposing += opposing
        self.sum_vbar_rel += vbar_rel
        self.sum_vbar_lat += vbar_lat
        self.sum_vbar_yaw += vbar_yaw
        self.sum_force_util += force_util
        self.limiter_active += 1 if limiter_active else 0
        self.saturated += 1 if saturated else 0
        self.residuals.append(residual)
        self.opposing.append(opposing)
        self.families.add(family)
        self.runs.add(run_id)


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def const_float(text: str, name: str) -> float:
    import re

    pattern = rf"\b{name}\s*=\s*([^;]+);"
    match = re.search(pattern, text)
    if not match:
        raise ValueError(f"Could not find {name}")
    expression = match.group(1).strip().replace("f", "").replace("F", "")
    if not re.fullmatch(r"[-+*/().0-9eE\s]+", expression):
        raise ValueError(f"Unsupported constant expression for {name}: {expression}")
    return float(eval(expression, {"__builtins__": {}}, {}))


def converted_const_float(text: str, name: str, function_name: str, scale: float) -> float:
    import re

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
    value = int(float(text))
    return value


def csv_rows_with_metadata(path: Path) -> tuple[dict[str, str], list[dict[str, str]]]:
    metadata: dict[str, str] = {}
    data_lines: list[str] = []
    with path.open("r", encoding="utf-8", errors="replace", newline="") as f:
        for line in f:
            if line.startswith("# meta,"):
                parts = line.strip().split(",", 3)
                if len(parts) >= 4:
                    metadata[parts[2]] = parts[3]
                continue
            if line.startswith("#"):
                continue
            data_lines.append(line)
    if not data_lines:
        return metadata, []
    return metadata, list(csv.DictReader(data_lines))


def discover_logs(include_competition: bool, include_uncertainty: bool) -> list[LogCandidate]:
    candidates: list[LogCandidate] = []
    for path in sorted((REPO_ROOT / "TestResults").glob("mmlog_decode_*/open_floor_main.csv")):
        candidates.append(
            LogCandidate(
                run_id=path.parent.name.replace("mmlog_decode_", ""),
                family="open_floor",
                schema="decoded_open_floor_main",
                path=path,
            )
        )
    if include_uncertainty:
        uncertainty = REPO_ROOT / "TestResults" / "UncertaintyTestLog" / "open_floor_main.csv"
        if uncertainty.is_file():
            candidates.append(
                LogCandidate(
                    run_id="UncertaintyTestLog",
                    family="uncertainty_open_floor",
                    schema="decoded_open_floor_main",
                    path=uncertainty,
                )
            )
    if include_competition:
        comp_dir = REPO_ROOT / "TestResults" / "Competition Testing Data"
        for path in sorted(comp_dir.glob("diag*.csv")):
            candidates.append(LogCandidate(path.stem, "competition_diag", "legacy_competition", path))
        for path in sorted(comp_dir.glob("aux*.csv")):
            candidates.append(LogCandidate(path.stem, "competition_aux", "legacy_competition", path))
    return candidates


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


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


def torque_from_command_detail(command: float, wheel_speed_radps: float, params: SourceParams) -> tuple[float, float]:
    command = clamp(command, -1.0, 1.0)
    applied_voltage_v = command * params.drive_voltage_v
    back_emf_voltage_v = (wheel_speed_radps * params.gear_ratio) / params.speed_constant_radps_per_volt
    armature_current_a = (applied_voltage_v - back_emf_voltage_v) / params.drive_resistance_ohms
    no_load_direction = signed_direction(armature_current_a, wheel_speed_radps)
    load_current_a = armature_current_a - (no_load_direction * params.no_load_current_a)
    if no_load_direction > 0.0 and load_current_a < 0.0:
        load_current_a = 0.0
    elif no_load_direction < 0.0 and load_current_a > 0.0:
        load_current_a = 0.0
    return params.torque_constant_nm_per_a * params.gear_ratio * load_current_a, armature_current_a


def applied_bank_torque_detail(command: float, wheel_speed_radps: float, params: SourceParams) -> tuple[float, float, int]:
    direct, raw_current = torque_from_command_detail(command, wheel_speed_radps, params)
    positive_limit = max(0.0, torque_from_command_detail(1.0, wheel_speed_radps, params)[0])
    negative_limit = min(0.0, torque_from_command_detail(-1.0, wheel_speed_radps, params)[0])
    limited = clamp(direct, negative_limit, positive_limit) if positive_limit > negative_limit else direct
    clipped = 1 if abs(limited - direct) > 1.0e-9 else 0
    static_launch_torque_nm = max(0.0, torque_from_command_detail(params.static_launch_command, 0.0, params)[0])
    surface_speed_mps = params.wheel_radius_m * wheel_speed_radps
    slow_ratio = abs(surface_speed_mps) / params.static_friction_max_speed_mps if params.static_friction_max_speed_mps > 0.0 else 0.0
    launch_torque_nm = static_launch_torque_nm * math.exp(-(slow_ratio * slow_ratio))
    applied = 0.0
    launch_direction = signed_direction(limited, wheel_speed_radps)
    if abs(limited) > launch_torque_nm:
        applied = limited - (launch_direction * launch_torque_nm)
    loss_direction = signed_direction(wheel_speed_radps, applied)
    rolling_loss = (
        params.rolling_friction_torque_nm * loss_direction
    ) + (params.viscous_friction_nm_per_radps * wheel_speed_radps)
    return applied - rolling_loss, raw_current, clipped


def yaw_denominator_kg_m2(params: SourceParams) -> float:
    wheel_spinup_mass_kg = (2.0 * params.wheel_bank_inertia_kg_m2) / (params.wheel_radius_m * params.wheel_radius_m)
    half_track = 0.5 * abs(params.track_width_m)
    return params.yaw_inertia_kg_m2 + (wheel_spinup_mass_kg * half_track * half_track)


def compute_features(row: NormalizedRow, forward_velocity_mps: float, yaw_rate_radps: float, params: SourceParams) -> MomentFeatures:
    left_torque_nm, left_raw_current_a, left_clip = applied_bank_torque_detail(
        row.left_command,
        row.left_wheel_speed_radps,
        params,
    )
    right_torque_nm, right_raw_current_a, right_clip = applied_bank_torque_detail(
        row.right_command,
        row.right_wheel_speed_radps,
        params,
    )
    left_drive_force_n = left_torque_nm / params.wheel_radius_m
    right_drive_force_n = right_torque_nm / params.wheel_radius_m
    fan_duty = clamp(row.fan_duty, 0.0, 1.0)
    total_normal_load_n = (params.mass_kg * 9.80665) + (fan_duty * params.fan_downforce_full_duty_n)
    front_normal_n = 0.5 * params.front_load_fraction * total_normal_load_n
    rear_normal_n = 0.5 * (1.0 - params.front_load_fraction) * total_normal_load_n
    fl_normal_n = front_normal_n
    fr_normal_n = front_normal_n
    rl_normal_n = rear_normal_n
    rr_normal_n = rear_normal_n
    left_bank_normal_n = fl_normal_n + rl_normal_n
    right_bank_normal_n = fr_normal_n + rr_normal_n
    fl_drive_request_n = left_drive_force_n * (fl_normal_n / left_bank_normal_n) if left_bank_normal_n > 1.0e-4 else 0.5 * left_drive_force_n
    rl_drive_request_n = left_drive_force_n * (rl_normal_n / left_bank_normal_n) if left_bank_normal_n > 1.0e-4 else 0.5 * left_drive_force_n
    fr_drive_request_n = right_drive_force_n * (fr_normal_n / right_bank_normal_n) if right_bank_normal_n > 1.0e-4 else 0.5 * right_drive_force_n
    rr_drive_request_n = right_drive_force_n * (rr_normal_n / right_bank_normal_n) if right_bank_normal_n > 1.0e-4 else 0.5 * right_drive_force_n

    half_track_m = 0.5 * abs(params.track_width_m)
    f_offset_m = abs(params.drive_wheel_longitudinal_offset_m)
    assumed_right_velocity_mps = 0.0
    left_body_forward_mps = forward_velocity_mps + (half_track_m * yaw_rate_radps)
    right_body_forward_mps = forward_velocity_mps - (half_track_m * yaw_rate_radps)
    fl_v_rel_f = row.left_velocity_mps - left_body_forward_mps
    rl_v_rel_f = fl_v_rel_f
    fr_v_rel_f = row.right_velocity_mps - right_body_forward_mps
    rr_v_rel_f = fr_v_rel_f
    front_body_right_mps = assumed_right_velocity_mps + (f_offset_m * yaw_rate_radps)
    rear_body_right_mps = assumed_right_velocity_mps - (f_offset_m * yaw_rate_radps)
    fl_v_rel_r = -front_body_right_mps
    fr_v_rel_r = -front_body_right_mps
    rl_v_rel_r = -rear_body_right_mps
    rr_v_rel_r = -rear_body_right_mps

    normal_sum = fl_normal_n + fr_normal_n + rl_normal_n + rr_normal_n
    eps = 1.0e-9
    vbar_rel = math.sqrt(
        (
            fl_normal_n * (fl_v_rel_f * fl_v_rel_f + fl_v_rel_r * fl_v_rel_r)
            + fr_normal_n * (fr_v_rel_f * fr_v_rel_f + fr_v_rel_r * fr_v_rel_r)
            + rl_normal_n * (rl_v_rel_f * rl_v_rel_f + rl_v_rel_r * rl_v_rel_r)
            + rr_normal_n * (rr_v_rel_f * rr_v_rel_f + rr_v_rel_r * rr_v_rel_r)
        )
        / (normal_sum + eps)
    )
    vbar_lat = math.sqrt(
        (
            fl_normal_n * fl_v_rel_r * fl_v_rel_r
            + fr_normal_n * fr_v_rel_r * fr_v_rel_r
            + rl_normal_n * rl_v_rel_r * rl_v_rel_r
            + rr_normal_n * rr_v_rel_r * rr_v_rel_r
        )
        / (normal_sum + eps)
    )
    vbar_yaw = abs(yaw_rate_radps) * math.sqrt(
        (
            fl_normal_n * (half_track_m * half_track_m + f_offset_m * f_offset_m)
            + fr_normal_n * (half_track_m * half_track_m + f_offset_m * f_offset_m)
            + rl_normal_n * (half_track_m * half_track_m + f_offset_m * f_offset_m)
            + rr_normal_n * (half_track_m * half_track_m + f_offset_m * f_offset_m)
        )
        / (normal_sum + eps)
    )

    half_longitudinal_stiffness = 0.5 * params.longitudinal_tire_stiffness_n
    fl_req_f = fl_drive_request_n + (half_longitudinal_stiffness * fl_v_rel_f)
    rl_req_f = rl_drive_request_n + (half_longitudinal_stiffness * rl_v_rel_f)
    fr_req_f = fr_drive_request_n + (half_longitudinal_stiffness * fr_v_rel_f)
    rr_req_f = rr_drive_request_n + (half_longitudinal_stiffness * rr_v_rel_f)
    fl_req_r = params.front_right_contact_force_gain_n_per_mps * fl_v_rel_r
    fr_req_r = params.front_right_contact_force_gain_n_per_mps * fr_v_rel_r
    rl_req_r = params.rear_right_contact_force_gain_n_per_mps * rl_v_rel_r
    rr_req_r = params.rear_right_contact_force_gain_n_per_mps * rr_v_rel_r

    sustained_mu = (
        (params.sustained_lateral_accel_mps2 * params.mass_kg / total_normal_load_n)
        if total_normal_load_n > 1.0e-4
        else 0.0
    )

    def project(forward_n: float, right_n: float, normal_n: float) -> tuple[float, float, float, float, float]:
        raw_mag_n = math.hypot(forward_n, right_n)
        max_force_n = max(0.0, sustained_mu * normal_n)
        scale = (max_force_n / raw_mag_n) if raw_mag_n > max_force_n and raw_mag_n > 1.0e-4 else 1.0
        utilization = raw_mag_n / max(max_force_n, 1.0e-4)
        capped = min(utilization, 1.0)
        limiter_activity = max(0.0, 1.0 - scale)
        return scale * forward_n, scale * right_n, utilization, capped, limiter_activity

    fl_force_f, fl_force_r, fl_util, fl_cap, fl_lim = project(fl_req_f, fl_req_r, fl_normal_n)
    fr_force_f, fr_force_r, fr_util, fr_cap, fr_lim = project(fr_req_f, fr_req_r, fr_normal_n)
    rl_force_f, rl_force_r, rl_util, rl_cap, rl_lim = project(rl_req_f, rl_req_r, rl_normal_n)
    rr_force_f, rr_force_r, rr_util, rr_cap, rr_lim = project(rr_req_f, rr_req_r, rr_normal_n)

    left_bank_forward_n = fl_force_f + rl_force_f
    right_bank_forward_n = fr_force_f + rr_force_f
    front_right_force_n = fl_force_r + fr_force_r
    rear_right_force_n = rl_force_r + rr_force_r
    yaw_moment_nm = (
        half_track_m * (left_bank_forward_n - right_bank_forward_n)
    ) + (f_offset_m * (front_right_force_n - rear_right_force_n))
    model_yaw_moment_nm = yaw_moment_nm - (params.yaw_rate_damping_nms_per_rad * yaw_rate_radps)
    diagnostic_encoder_yaw_rate_radps = (
        params.wheel_radius_m * (row.left_wheel_speed_radps - row.right_wheel_speed_radps) / params.track_width_m
        if params.track_width_m > 0.0
        else 0.0
    )
    diagnostic_surface_delta_mps = row.left_velocity_mps - row.right_velocity_mps
    return MomentFeatures(
        left_torque_nm,
        right_torque_nm,
        left_drive_force_n,
        right_drive_force_n,
        left_raw_current_a,
        right_raw_current_a,
        left_clip,
        right_clip,
        total_normal_load_n,
        fl_normal_n,
        fr_normal_n,
        rl_normal_n,
        rr_normal_n,
        fl_v_rel_f,
        fr_v_rel_f,
        rl_v_rel_f,
        rr_v_rel_f,
        fl_v_rel_r,
        fr_v_rel_r,
        rl_v_rel_r,
        rr_v_rel_r,
        vbar_rel,
        vbar_lat,
        vbar_yaw,
        fl_req_f,
        fr_req_f,
        rl_req_f,
        rr_req_f,
        fl_req_r,
        fr_req_r,
        rl_req_r,
        rr_req_r,
        fl_force_f,
        fr_force_f,
        rl_force_f,
        rr_force_f,
        fl_force_r,
        fr_force_r,
        rl_force_r,
        rr_force_r,
        max(fl_util, fr_util, rl_util, rr_util),
        max(fl_lim, fr_lim, rl_lim, rr_lim),
        max(fl_cap, fr_cap, rl_cap, rr_cap),
        model_yaw_moment_nm,
        diagnostic_encoder_yaw_rate_radps,
        diagnostic_surface_delta_mps,
    )


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


def normalize_open_floor(row: dict[str, str], index: int) -> NormalizedRow:
    fan_text = row.get("fan_duty_cycle", "")
    fan_duty = finite_float(row, "fan_duty_cycle", 0.8)
    return NormalizedRow(
        row_index=index,
        raw=row,
        time_us=finite_int(row, "master_time_us", 0),
        tick=finite_int(row, "control_tick_sequence", index),
        dt_us=finite_int(row, "dt_us", 1000),
        phase_id=row.get("phase_id", ""),
        section_id=row.get("section_id", ""),
        primitive_id=row.get("primitive_id", ""),
        speed_bin=row.get("speed_bin", ""),
        repeat_index=row.get("repeat_index", ""),
        left_command=finite_float(row, "left_drive_command"),
        right_command=finite_float(row, "right_drive_command"),
        left_velocity_mps=finite_float(row, "left_encoder_velocity_mps"),
        right_velocity_mps=finite_float(row, "right_encoder_velocity_mps"),
        left_wheel_speed_radps=finite_float(row, "left_encoder_omega_radps"),
        right_wheel_speed_radps=finite_float(row, "right_encoder_omega_radps"),
        gyro_raw_radps=finite_float(row, "gyro_raw_radps"),
        stationary_flag=False,
        clipping_flags=finite_int(row, "clipping_flags", 0),
        saturation_flags=finite_int(row, "saturation_flags", 0),
        watchdog_flags=finite_int(row, "watchdog_flags", 0),
        fan_duty=fan_duty,
        fan_source="row_fan_duty_cycle" if fan_text != "" else "default_0p8",
    )


def normalize_competition(row: dict[str, str], index: int, metadata: dict[str, str], params: SourceParams) -> NormalizedRow:
    left_velocity = finite_float(row, "left_velocity_mps")
    right_velocity = finite_float(row, "right_velocity_mps")
    fan_text = metadata.get("kRacingFanDutyCycle", "")
    fan_duty = float(fan_text) if fan_text != "" else 0.8
    return NormalizedRow(
        row_index=index,
        raw=row,
        time_us=finite_int(row, "t_us", 0),
        tick=finite_int(row, "sample", index),
        dt_us=finite_int(row, "dt_us", 1000),
        phase_id=row.get("phase_id", ""),
        section_id="",
        primitive_id="",
        speed_bin="",
        repeat_index="",
        left_command=finite_float(row, "left_drive_cmd"),
        right_command=finite_float(row, "right_drive_cmd"),
        left_velocity_mps=left_velocity,
        right_velocity_mps=right_velocity,
        left_wheel_speed_radps=left_velocity / params.wheel_radius_m,
        right_wheel_speed_radps=right_velocity / params.wheel_radius_m,
        gyro_raw_radps=finite_float(row, "gyro_raw_radps"),
        stationary_flag=row.get("stationary", "0") in {"1", "true", "True"},
        clipping_flags=0,
        saturation_flags=0,
        watchdog_flags=0,
        fan_duty=fan_duty,
        fan_source="metadata_kRacingFanDutyCycle" if fan_text != "" else "default_0p8",
    )


def read_normalized_rows(candidate: LogCandidate, params: SourceParams) -> tuple[dict[str, str], list[NormalizedRow], str, int]:
    metadata, rows = csv_rows_with_metadata(candidate.path)
    if not rows:
        return metadata, [], "empty csv", 0
    headers = set(rows[0].keys())
    normalized: list[NormalizedRow] = []
    if {
        "left_drive_command",
        "right_drive_command",
        "left_encoder_velocity_mps",
        "right_encoder_velocity_mps",
        "left_encoder_omega_radps",
        "right_encoder_omega_radps",
        "gyro_raw_radps",
    }.issubset(headers):
        for index, row in enumerate(rows):
            try:
                normalized.append(normalize_open_floor(row, index))
            except ValueError:
                continue
        return metadata, normalized, "", len(rows)
    if {"left_drive_cmd", "right_drive_cmd", "left_velocity_mps", "right_velocity_mps", "gyro_raw_radps"}.issubset(headers):
        for index, row in enumerate(rows):
            try:
                normalized.append(normalize_competition(row, index, metadata, params))
            except ValueError:
                continue
        return metadata, normalized, "", len(rows)
    return metadata, [], "missing required command/encoder/raw gyro columns", len(rows)


def estimate_bias(rows: list[NormalizedRow]) -> tuple[float, int]:
    stationary: list[float] = []
    for row in rows:
        if not (250 <= row.dt_us <= 5000):
            continue
        near_zero_command = max(abs(row.left_command), abs(row.right_command)) <= 0.025
        near_zero_velocity = max(abs(row.left_velocity_mps), abs(row.right_velocity_mps)) <= 0.020
        if (row.stationary_flag or (near_zero_command and near_zero_velocity)) and abs(row.gyro_raw_radps) <= 0.6:
            stationary.append(row.gyro_raw_radps)
    return robust_bias(stationary)


def row_key(row: NormalizedRow, family: str) -> tuple[str, ...]:
    if family.startswith("competition"):
        return (row.phase_id,)
    return (row.section_id, row.phase_id, row.primitive_id, row.speed_bin, row.repeat_index)


def is_tail_quiescent_or_invalid(row: NormalizedRow, bias_radps: float) -> bool:
    if row.tick < 0 or not (250 <= row.dt_us <= 5000):
        return True
    yaw_rate = row.gyro_raw_radps - bias_radps
    return (
        max(abs(row.left_command), abs(row.right_command)) <= 0.03
        and max(abs(row.left_velocity_mps), abs(row.right_velocity_mps)) <= 0.03
        and abs(yaw_rate) <= 0.08
    )


def tail_cut_index(rows: list[NormalizedRow], bias_radps: float) -> tuple[int, str, int | None, int | None]:
    tail_start: int | None = None
    tail_tick: int | None = None
    tail_time: int | None = None
    for index, row in enumerate(rows):
        if is_tail_quiescent_or_invalid(row, bias_radps):
            if tail_start is None:
                tail_start = index
                tail_tick = row.tick
                tail_time = row.time_us
        else:
            tail_start = None
            tail_tick = None
            tail_time = None
    if tail_start is not None and rows:
        duration_s = max(0.0, (rows[-1].time_us - (tail_time or rows[-1].time_us)) * 1.0e-6)
        if duration_s >= 0.25:
            return tail_start, "dropped final sensor-quiescent/invalid tail >= 0.25 s", tail_tick, tail_time
    return len(rows), "kept through final sensor-active row", None, None


def valid_adjacent(current: NormalizedRow, nxt: NormalizedRow, family: str) -> bool:
    if nxt.tick <= current.tick:
        return False
    if not (250 <= nxt.dt_us <= 5000):
        return False
    if abs(current.gyro_raw_radps) > 40.0 or abs(nxt.gyro_raw_radps) > 40.0:
        return False
    if max(abs(current.left_command), abs(current.right_command), abs(nxt.left_command), abs(nxt.right_command)) > 1.05:
        return False
    if max(abs(current.left_velocity_mps), abs(current.right_velocity_mps), abs(nxt.left_velocity_mps), abs(nxt.right_velocity_mps)) > 4.0:
        return False
    return row_key(current, family) == row_key(nxt, family)


def active_contact_sample(row: NormalizedRow, yaw_rate_radps: float, forward_velocity_mps: float) -> bool:
    return (
        abs(yaw_rate_radps) >= 0.08
        or abs(forward_velocity_mps) >= 0.05
        or max(abs(row.left_command), abs(row.right_command)) >= 0.03
    )


def round_to_step(value: float, step: float) -> float:
    if abs(value) < 0.5 * step:
        return 0.0
    return round(value / step) * step


def stable_sample_keep(run_id: str, tick: int, sample_every: int) -> bool:
    if sample_every <= 1:
        return True
    digest = hashlib.blake2b(f"{run_id}:{tick}".encode("utf-8"), digest_size=8).digest()
    value = int.from_bytes(digest, "little")
    return (value % sample_every) == 0


def trimmed_mean(values: Iterable[float], trim_fraction: float = 0.10) -> float:
    materialized = sorted(values)
    if not materialized:
        return 0.0
    trim = int(len(materialized) * trim_fraction)
    if trim * 2 >= len(materialized):
        return statistics.fmean(materialized)
    return statistics.fmean(materialized[trim : len(materialized) - trim])


def percentile(sorted_values: list[float], q: float) -> float:
    if not sorted_values:
        return 0.0
    if len(sorted_values) == 1:
        return sorted_values[0]
    position = q * (len(sorted_values) - 1)
    lo = int(math.floor(position))
    hi = int(math.ceil(position))
    if lo == hi:
        return sorted_values[lo]
    weight = position - lo
    return sorted_values[lo] * (1.0 - weight) + sorted_values[hi] * weight


FEATURE_FIELDNAMES = [
    "run_id",
    "family",
    "schema",
    "row_index",
    "tick",
    "time_us",
    "dt_s",
    "section_id",
    "phase_id",
    "primitive_id",
    "speed_bin",
    "repeat_index",
    "valid_bad_tail_status",
    "left_command",
    "right_command",
    "left_encoder_velocity_mps",
    "right_encoder_velocity_mps",
    "left_wheel_speed_radps",
    "right_wheel_speed_radps",
    "forward_velocity_mps",
    "assumed_right_velocity_mps",
    "right_velocity_source",
    "yaw_rate_radps",
    "next_yaw_rate_radps",
    "measured_yaw_accel_radps2",
    "fl_v_rel_f_mps",
    "fr_v_rel_f_mps",
    "rl_v_rel_f_mps",
    "rr_v_rel_f_mps",
    "fl_v_rel_r_mps",
    "fr_v_rel_r_mps",
    "rl_v_rel_r_mps",
    "rr_v_rel_r_mps",
    "vbar_rel_mps",
    "vbar_lat_mps",
    "vbar_yaw_mps",
    "left_applied_torque_nm",
    "right_applied_torque_nm",
    "left_drive_force_n",
    "right_drive_force_n",
    "fl_req_f_n",
    "fr_req_f_n",
    "rl_req_f_n",
    "rr_req_f_n",
    "fl_req_r_n",
    "fr_req_r_n",
    "rl_req_r_n",
    "rr_req_r_n",
    "fl_force_f_n",
    "fr_force_f_n",
    "rl_force_f_n",
    "rr_force_f_n",
    "fl_force_r_n",
    "fr_force_r_n",
    "rl_force_r_n",
    "rr_force_r_n",
    "total_normal_load_n",
    "fl_normal_n",
    "fr_normal_n",
    "rl_normal_n",
    "rr_normal_n",
    "fan_duty",
    "fan_source",
    "max_force_preprojection_utilization",
    "max_force_limiter_activity",
    "max_force_saturation_capped",
    "left_raw_current_a",
    "right_raw_current_a",
    "current_proxy_abs_raw_over_unit_command_prior",
    "left_unit_command_torque_clip",
    "right_unit_command_torque_clip",
    "clipping_flags",
    "saturation_flags",
    "watchdog_flags",
    "observed_yaw_moment_nm",
    "model_yaw_moment_nm",
    "residual_additive_yaw_torque_nm",
    "opposing_yaw_resistance_nm",
    "diagnostic_encoder_yaw_rate_radps",
    "diagnostic_surface_delta_mps",
]


def feature_row(
    candidate: LogCandidate,
    current: NormalizedRow,
    nxt: NormalizedRow,
    yaw_rate: float,
    next_yaw_rate: float,
    measured_yaw_accel: float,
    observed_moment: float,
    residual: float,
    opposing: float,
    features: MomentFeatures,
) -> dict[str, str]:
    unit_current_prior = max(1.0e-9, candidate_current_prior_abs(features))
    return {
        "run_id": candidate.run_id,
        "family": candidate.family,
        "schema": candidate.schema,
        "row_index": str(current.row_index),
        "tick": str(current.tick),
        "time_us": str(current.time_us),
        "dt_s": f"{nxt.dt_us * 1.0e-6:.9f}",
        "section_id": current.section_id,
        "phase_id": current.phase_id,
        "primitive_id": current.primitive_id,
        "speed_bin": current.speed_bin,
        "repeat_index": current.repeat_index,
        "valid_bad_tail_status": "kept_before_tail",
        "left_command": f"{current.left_command:.9f}",
        "right_command": f"{current.right_command:.9f}",
        "left_encoder_velocity_mps": f"{current.left_velocity_mps:.9f}",
        "right_encoder_velocity_mps": f"{current.right_velocity_mps:.9f}",
        "left_wheel_speed_radps": f"{current.left_wheel_speed_radps:.9f}",
        "right_wheel_speed_radps": f"{current.right_wheel_speed_radps:.9f}",
        "forward_velocity_mps": f"{0.5 * (current.left_velocity_mps + current.right_velocity_mps):.9f}",
        "assumed_right_velocity_mps": "0.000000000",
        "right_velocity_source": "zero_lateral_sensor_unavailable",
        "yaw_rate_radps": f"{yaw_rate:.9f}",
        "next_yaw_rate_radps": f"{next_yaw_rate:.9f}",
        "measured_yaw_accel_radps2": f"{measured_yaw_accel:.9f}",
        "fl_v_rel_f_mps": f"{features.fl_v_rel_f_mps:.9f}",
        "fr_v_rel_f_mps": f"{features.fr_v_rel_f_mps:.9f}",
        "rl_v_rel_f_mps": f"{features.rl_v_rel_f_mps:.9f}",
        "rr_v_rel_f_mps": f"{features.rr_v_rel_f_mps:.9f}",
        "fl_v_rel_r_mps": f"{features.fl_v_rel_r_mps:.9f}",
        "fr_v_rel_r_mps": f"{features.fr_v_rel_r_mps:.9f}",
        "rl_v_rel_r_mps": f"{features.rl_v_rel_r_mps:.9f}",
        "rr_v_rel_r_mps": f"{features.rr_v_rel_r_mps:.9f}",
        "vbar_rel_mps": f"{features.vbar_rel_mps:.9f}",
        "vbar_lat_mps": f"{features.vbar_lat_mps:.9f}",
        "vbar_yaw_mps": f"{features.vbar_yaw_mps:.9f}",
        "left_applied_torque_nm": f"{features.left_applied_torque_nm:.9f}",
        "right_applied_torque_nm": f"{features.right_applied_torque_nm:.9f}",
        "left_drive_force_n": f"{features.left_drive_force_n:.9f}",
        "right_drive_force_n": f"{features.right_drive_force_n:.9f}",
        "fl_req_f_n": f"{features.fl_req_f_n:.9f}",
        "fr_req_f_n": f"{features.fr_req_f_n:.9f}",
        "rl_req_f_n": f"{features.rl_req_f_n:.9f}",
        "rr_req_f_n": f"{features.rr_req_f_n:.9f}",
        "fl_req_r_n": f"{features.fl_req_r_n:.9f}",
        "fr_req_r_n": f"{features.fr_req_r_n:.9f}",
        "rl_req_r_n": f"{features.rl_req_r_n:.9f}",
        "rr_req_r_n": f"{features.rr_req_r_n:.9f}",
        "fl_force_f_n": f"{features.fl_force_f_n:.9f}",
        "fr_force_f_n": f"{features.fr_force_f_n:.9f}",
        "rl_force_f_n": f"{features.rl_force_f_n:.9f}",
        "rr_force_f_n": f"{features.rr_force_f_n:.9f}",
        "fl_force_r_n": f"{features.fl_force_r_n:.9f}",
        "fr_force_r_n": f"{features.fr_force_r_n:.9f}",
        "rl_force_r_n": f"{features.rl_force_r_n:.9f}",
        "rr_force_r_n": f"{features.rr_force_r_n:.9f}",
        "total_normal_load_n": f"{features.total_normal_load_n:.9f}",
        "fl_normal_n": f"{features.fl_normal_n:.9f}",
        "fr_normal_n": f"{features.fr_normal_n:.9f}",
        "rl_normal_n": f"{features.rl_normal_n:.9f}",
        "rr_normal_n": f"{features.rr_normal_n:.9f}",
        "fan_duty": f"{current.fan_duty:.9f}",
        "fan_source": current.fan_source,
        "max_force_preprojection_utilization": f"{features.max_force_preprojection_utilization:.9f}",
        "max_force_limiter_activity": f"{features.max_force_limiter_activity:.9f}",
        "max_force_saturation_capped": f"{features.max_force_saturation_capped:.9f}",
        "left_raw_current_a": f"{features.left_raw_current_a:.9f}",
        "right_raw_current_a": f"{features.right_raw_current_a:.9f}",
        "current_proxy_abs_raw_over_unit_command_prior": f"{unit_current_prior:.9f}",
        "left_unit_command_torque_clip": str(features.left_unit_command_clip),
        "right_unit_command_torque_clip": str(features.right_unit_command_clip),
        "clipping_flags": str(current.clipping_flags),
        "saturation_flags": str(current.saturation_flags),
        "watchdog_flags": str(current.watchdog_flags),
        "observed_yaw_moment_nm": f"{observed_moment:.9f}",
        "model_yaw_moment_nm": f"{features.model_yaw_moment_nm:.9f}",
        "residual_additive_yaw_torque_nm": f"{residual:.9f}",
        "opposing_yaw_resistance_nm": f"{opposing:.9f}",
        "diagnostic_encoder_yaw_rate_radps": f"{features.diagnostic_encoder_yaw_rate_radps:.9f}",
        "diagnostic_surface_delta_mps": f"{features.diagnostic_surface_delta_mps:.9f}",
    }


def candidate_current_prior_abs(features: MomentFeatures) -> float:
    return max(abs(features.left_raw_current_a), abs(features.right_raw_current_a))


def write_run_summary(path: Path, runs: list[RunSummary]) -> None:
    fieldnames = [
        "run_id",
        "family",
        "schema",
        "path",
        "input_rows",
        "normalized_rows",
        "kept_rows",
        "dropped_tail_rows",
        "bias_radps",
        "bias_rows",
        "extracted_samples",
        "sampled_rows_written",
        "max_abs_forward_mps",
        "max_abs_yaw_radps",
        "max_vbar_rel_mps",
        "max_force_utilization",
        "saturated_rows",
        "limiter_active_rows",
        "cutoff_reason",
        "cutoff_tick",
        "cutoff_time_us",
        "limitation",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for run in runs:
            rel_path = str(run.path.relative_to(REPO_ROOT)) if run.path.is_relative_to(REPO_ROOT) else str(run.path)
            writer.writerow(
                {
                    "run_id": run.run_id,
                    "family": run.family,
                    "schema": run.schema,
                    "path": rel_path,
                    "input_rows": run.input_rows,
                    "normalized_rows": run.normalized_rows,
                    "kept_rows": run.kept_rows,
                    "dropped_tail_rows": run.dropped_tail_rows,
                    "bias_radps": f"{run.bias_radps:.9f}",
                    "bias_rows": run.bias_rows,
                    "extracted_samples": run.extracted_samples,
                    "sampled_rows_written": run.sampled_rows_written,
                    "max_abs_forward_mps": f"{run.max_abs_forward_mps:.9f}",
                    "max_abs_yaw_radps": f"{run.max_abs_yaw_radps:.9f}",
                    "max_vbar_rel_mps": f"{run.max_vbar_rel_mps:.9f}",
                    "max_force_utilization": f"{run.max_force_utilization:.9f}",
                    "saturated_rows": run.saturated_rows,
                    "limiter_active_rows": run.limiter_active_rows,
                    "cutoff_reason": run.cutoff_reason,
                    "cutoff_tick": "" if run.cutoff_tick is None else run.cutoff_tick,
                    "cutoff_time_us": "" if run.cutoff_time_us is None else run.cutoff_time_us,
                    "limitation": run.limitation,
                }
            )


def aggregate_row(label_fields: dict[str, str], agg: ValueAgg) -> dict[str, str]:
    residuals = sorted(agg.residuals)
    opposing = sorted(agg.opposing)
    median_residual = statistics.median(residuals) if residuals else 0.0
    median_opposing = statistics.median(opposing) if opposing else 0.0
    row = dict(label_fields)
    row.update(
        {
            "count": str(agg.count),
            "run_count": str(len(agg.runs)),
            "families": ";".join(sorted(agg.families)),
            "mean_residual_additive_yaw_torque_nm": f"{agg.sum_residual / agg.count:.9f}" if agg.count else "0.000000000",
            "median_residual_additive_yaw_torque_nm": f"{median_residual:.9f}",
            "trimmed_mean_residual_additive_yaw_torque_nm": f"{trimmed_mean(residuals):.9f}",
            "rmse_residual_additive_yaw_torque_nm": f"{math.sqrt(agg.sum_sq_residual / agg.count):.9f}" if agg.count else "0.000000000",
            "mad_residual_additive_yaw_torque_nm": f"{median_abs_deviation(residuals, median_residual):.9f}",
            "iqr_residual_additive_yaw_torque_nm": f"{percentile(residuals, 0.75) - percentile(residuals, 0.25):.9f}",
            "mean_opposing_yaw_resistance_nm": f"{agg.sum_opposing / agg.count:.9f}" if agg.count else "0.000000000",
            "median_opposing_yaw_resistance_nm": f"{median_opposing:.9f}",
            "trimmed_mean_opposing_yaw_resistance_nm": f"{trimmed_mean(opposing):.9f}",
            "mean_vbar_rel_mps": f"{agg.sum_vbar_rel / agg.count:.9f}" if agg.count else "0.000000000",
            "mean_vbar_lat_mps": f"{agg.sum_vbar_lat / agg.count:.9f}" if agg.count else "0.000000000",
            "mean_vbar_yaw_mps": f"{agg.sum_vbar_yaw / agg.count:.9f}" if agg.count else "0.000000000",
            "mean_max_force_preprojection_utilization": f"{agg.sum_force_util / agg.count:.9f}" if agg.count else "0.000000000",
            "limiter_active_fraction": f"{agg.limiter_active / agg.count:.9f}" if agg.count else "0.000000000",
            "hardware_saturation_fraction": f"{agg.saturated / agg.count:.9f}" if agg.count else "0.000000000",
        }
    )
    return row


def write_aggregate(path: Path, groups: dict[tuple, ValueAgg], labels: list[str], min_count: int) -> int:
    rows: list[dict[str, str]] = []
    for key, agg in groups.items():
        if agg.count < min_count:
            continue
        label_fields = {labels[index]: str(value) for index, value in enumerate(key)}
        rows.append(aggregate_row(label_fields, agg))
    rows.sort(key=lambda row: tuple(row[label] for label in labels))
    fieldnames = labels + [
        "count",
        "run_count",
        "families",
        "mean_residual_additive_yaw_torque_nm",
        "median_residual_additive_yaw_torque_nm",
        "trimmed_mean_residual_additive_yaw_torque_nm",
        "rmse_residual_additive_yaw_torque_nm",
        "mad_residual_additive_yaw_torque_nm",
        "iqr_residual_additive_yaw_torque_nm",
        "mean_opposing_yaw_resistance_nm",
        "median_opposing_yaw_resistance_nm",
        "trimmed_mean_opposing_yaw_resistance_nm",
        "mean_vbar_rel_mps",
        "mean_vbar_lat_mps",
        "mean_vbar_yaw_mps",
        "mean_max_force_preprojection_utilization",
        "limiter_active_fraction",
        "hardware_saturation_fraction",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def write_constants(path: Path, params: SourceParams) -> None:
    rows = [
        ("mass_kg", params.mass_kg),
        ("track_width_m", params.track_width_m),
        ("yaw_inertia_kg_m2", params.yaw_inertia_kg_m2),
        ("yaw_denominator_including_wheel_spinup_kg_m2", yaw_denominator_kg_m2(params)),
        ("wheel_radius_m", params.wheel_radius_m),
        ("drive_wheel_longitudinal_offset_m", params.drive_wheel_longitudinal_offset_m),
        ("wheel_bank_inertia_kg_m2", params.wheel_bank_inertia_kg_m2),
        ("drive_voltage_v", params.drive_voltage_v),
        ("drive_resistance_ohms", params.drive_resistance_ohms),
        ("torque_constant_nm_per_a", params.torque_constant_nm_per_a),
        ("speed_constant_radps_per_volt", params.speed_constant_radps_per_volt),
        ("no_load_current_a", params.no_load_current_a),
        ("gear_ratio", params.gear_ratio),
        ("rolling_friction_torque_nm", params.rolling_friction_torque_nm),
        ("static_launch_command", params.static_launch_command),
        ("static_friction_max_speed_mps", params.static_friction_max_speed_mps),
        ("front_load_fraction", params.front_load_fraction),
        ("longitudinal_tire_stiffness_n", params.longitudinal_tire_stiffness_n),
        ("front_right_contact_force_gain_n_per_mps", params.front_right_contact_force_gain_n_per_mps),
        ("rear_right_contact_force_gain_n_per_mps", params.rear_right_contact_force_gain_n_per_mps),
        ("fan_downforce_full_duty_n", params.fan_downforce_full_duty_n),
        ("sustained_lateral_accel_mps2", params.sustained_lateral_accel_mps2),
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["name", "value"])
        writer.writeheader()
        for name, value in rows:
            writer.writerow({"name": name, "value": f"{value:.12g}"})


def process_logs(args: argparse.Namespace) -> tuple[list[RunSummary], dict[str, ValueAgg], dict[tuple, ValueAgg], dict[tuple, ValueAgg], dict[tuple, ValueAgg], int]:
    params = source_params()
    write_constants(args.out_dir / "plant_mirror_constants.csv", params)
    candidates = discover_logs(include_competition=not args.no_competition, include_uncertainty=not args.no_uncertainty)
    by_family: dict[str, ValueAgg] = {}
    by_contact_bins: dict[tuple, ValueAgg] = {}
    by_phase: dict[tuple, ValueAgg] = {}
    by_force_bins: dict[tuple, ValueAgg] = {}
    runs: list[RunSummary] = []
    total_sampled_rows = 0
    denom = yaw_denominator_kg_m2(params)
    unit_current_prior = params.drive_voltage_v / params.drive_resistance_ohms if params.drive_resistance_ohms > 0.0 else 1.0

    sample_path = args.out_dir / "contact_continuum_feature_sample.csv"
    with sample_path.open("w", newline="", encoding="utf-8") as sample_file:
        sample_writer = csv.DictWriter(sample_file, fieldnames=FEATURE_FIELDNAMES)
        sample_writer.writeheader()
        for candidate in candidates:
            metadata, rows, limitation, input_rows = read_normalized_rows(candidate, params)
            run = RunSummary(
                run_id=candidate.run_id,
                family=candidate.family,
                schema=candidate.schema,
                path=candidate.path,
                input_rows=input_rows,
                normalized_rows=len(rows),
                limitation=limitation,
            )
            if candidate.schema == "legacy_competition":
                run.limitation = (
                    "legacy competition schema: wheel omega derived from encoder velocity/current radius; "
                    "saturation/watchdog fields unavailable; fan duty from metadata/default; pose fields ignored"
                )
            if not rows:
                run.cutoff_reason = "excluded"
                runs.append(run)
                print(f"{candidate.family}:{candidate.run_id}: rows={input_rows} samples=0 ({run.limitation})")
                continue
            bias, bias_rows = estimate_bias(rows)
            cutoff_index, cutoff_reason, cutoff_tick, cutoff_time = tail_cut_index(rows, bias)
            kept_rows = rows[:cutoff_index]
            run.kept_rows = len(kept_rows)
            run.dropped_tail_rows = max(0, len(rows) - len(kept_rows))
            run.bias_radps = bias
            run.bias_rows = bias_rows
            run.cutoff_reason = cutoff_reason
            run.cutoff_tick = cutoff_tick
            run.cutoff_time_us = cutoff_time
            for current, nxt in zip(kept_rows, kept_rows[1:]):
                if not valid_adjacent(current, nxt, candidate.family):
                    continue
                yaw_rate = current.gyro_raw_radps - bias
                next_yaw_rate = nxt.gyro_raw_radps - bias
                forward_velocity = 0.5 * (current.left_velocity_mps + current.right_velocity_mps)
                if not active_contact_sample(current, yaw_rate, forward_velocity):
                    continue
                dt_s = nxt.dt_us * 1.0e-6
                measured_yaw_accel = (next_yaw_rate - yaw_rate) / dt_s
                if not math.isfinite(measured_yaw_accel) or abs(measured_yaw_accel) > 4000.0:
                    continue
                features = compute_features(current, forward_velocity, yaw_rate, params)
                observed_moment = denom * measured_yaw_accel
                residual = observed_moment - features.model_yaw_moment_nm
                if not math.isfinite(residual) or abs(residual) > 2.0:
                    continue
                opposing = -math.copysign(1.0, yaw_rate) * residual if abs(yaw_rate) > 1.0e-9 else 0.0
                limiter_active = features.max_force_limiter_activity > 1.0e-6
                saturated = current.saturation_flags != 0 or current.clipping_flags != 0 or current.watchdog_flags != 0
                run.extracted_samples += 1
                run.max_abs_forward_mps = max(run.max_abs_forward_mps, abs(forward_velocity))
                run.max_abs_yaw_radps = max(run.max_abs_yaw_radps, abs(yaw_rate))
                run.max_vbar_rel_mps = max(run.max_vbar_rel_mps, features.vbar_rel_mps)
                run.max_force_utilization = max(run.max_force_utilization, features.max_force_preprojection_utilization)
                run.saturated_rows += 1 if saturated else 0
                run.limiter_active_rows += 1 if limiter_active else 0

                def add_group(groups: dict, key: tuple) -> None:
                    groups.setdefault(key, ValueAgg()).add(
                        residual,
                        opposing,
                        features.vbar_rel_mps,
                        features.vbar_lat_mps,
                        features.vbar_yaw_mps,
                        features.max_force_preprojection_utilization,
                        limiter_active,
                        saturated,
                        candidate.family,
                        candidate.run_id,
                    )

                add_group(by_family, (candidate.family,))
                add_group(
                    by_contact_bins,
                    (
                        f"{round_to_step(features.vbar_rel_mps, 0.05):.2f}",
                        f"{round_to_step(features.vbar_lat_mps, 0.05):.2f}",
                        f"{round_to_step(features.vbar_yaw_mps, 0.05):.2f}",
                        f"{round_to_step(yaw_rate, 0.50):.2f}",
                    ),
                )
                add_group(
                    by_phase,
                    (
                        candidate.family,
                        current.section_id,
                        current.phase_id,
                        current.primitive_id,
                        current.speed_bin,
                    ),
                )
                add_group(
                    by_force_bins,
                    (
                        f"{round_to_step(features.max_force_preprojection_utilization, 0.25):.2f}",
                        f"{round_to_step(features.vbar_rel_mps, 0.10):.2f}",
                    ),
                )

                if stable_sample_keep(candidate.run_id, current.tick, args.sample_every):
                    row = feature_row(
                        candidate,
                        current,
                        nxt,
                        yaw_rate,
                        next_yaw_rate,
                        measured_yaw_accel,
                        observed_moment,
                        residual,
                        opposing,
                        features,
                    )
                    row["current_proxy_abs_raw_over_unit_command_prior"] = f"{max(abs(features.left_raw_current_a), abs(features.right_raw_current_a)) / unit_current_prior:.9f}"
                    sample_writer.writerow(row)
                    run.sampled_rows_written += 1
                    total_sampled_rows += 1
            runs.append(run)
            print(f"{candidate.family}:{candidate.run_id}: rows={input_rows} samples={run.extracted_samples} sampled={run.sampled_rows_written}")
    write_run_summary(args.out_dir / "contact_continuum_run_summary.csv", runs)
    return runs, by_family, by_contact_bins, by_phase, by_force_bins, total_sampled_rows


def markdown_report(
    args: argparse.Namespace,
    runs: list[RunSummary],
    by_family: dict[str, ValueAgg],
    contact_bin_rows: int,
    phase_rows: int,
    force_rows: int,
    sampled_rows: int,
) -> str:
    included = [run for run in runs if run.extracted_samples > 0]
    total_samples = sum(run.extracted_samples for run in runs)
    total_input_rows = sum(run.input_rows for run in runs)
    total_tail = sum(run.dropped_tail_rows for run in runs)
    family_lines = []
    for (family,), agg in sorted(by_family.items()):
        family_lines.append(
            f"| {family} | {agg.count} | {len(agg.runs)} | {agg.limiter_active / agg.count:.6f} | {agg.saturated / agg.count:.6f} | {agg.sum_vbar_rel / agg.count:.6f} |"
        )
    command = (
        f"python codex_analysis\\contact_continuum_yaw_identification\\features\\extract_contact_continuum_features.py "
        f"--out-dir codex_analysis\\contact_continuum_yaw_identification\\features "
        f"--sample-every {args.sample_every} --min-bin-count {args.min_bin_count}"
    )
    if args.no_competition:
        command += " --no-competition"
    if args.no_uncertainty:
        command += " --no-uncertainty"
    lines = [
        "# Contact-Continuum Yaw Feature Extraction",
        "",
        "Scratch analysis only. No production code was modified.",
        "",
        "## Reproduce",
        "",
        "```powershell",
        command,
        "```",
        "",
        "## Source Basis",
        "",
        "The feature pass follows `micromouse_ukf_plant_measurement_noise_theory_only_spec.md`: contact-relative velocity is the primary contact primitive. It does not build a Vf/yaw residual table as the primary semantic object, and it does not compute slip angle, slip ratio, curvature, radius, or maneuver-mode branches.",
        "",
        "Targets use sensor data only: raw gyro yaw rate minus an independently estimated stationary bias where stationary rows exist, encoder-derived forward velocity and wheel-bank speeds, logged drive commands, and timestamps. Logged `ukf_state_*`, pose, estimator yaw-rate, and logged gyro-bias columns are not used as targets.",
        "",
        "The PlantModel mirror includes command-to-bank torque, no-load current, static launch loss, rolling loss, normal-load distribution, longitudinal/right contact force requests, force projection, yaw damping, and the wheel spin-up term in the yaw denominator. The residual convention is `residual_additive_yaw_torque_nm = observed_yaw_moment_nm - model_yaw_moment_nm`; positive yaw is clockwise.",
        "",
        "## Feature Definitions",
        "",
        "- Contact coordinates are `r>0` right and `f>0` forward: FL=(-half_track,+offset), FR=(+half_track,+offset), RL=(-half_track,-offset), RR=(+half_track,-offset).",
        "- `v_rel_f_i = wheel_surface_velocity_bank - (Vf - omega*r_i)`. The script uses encoder wheel surface velocities and sensor-derived `Vf`.",
        "- `v_rel_r_i = -(Vr + omega*f_i)`. No independent lateral velocity exists in these logs, so `Vr=0` is an explicit `zero_lateral_sensor_unavailable` source assumption.",
        "- `vbar_rel`, `vbar_lat`, and `vbar_yaw` are load-weighted RMS quantities from the root spec. Normal loads are static plus fan load only; load transfer is not reconstructed.",
        "- Force-request columns are the current PlantModel mirror's drive request plus longitudinal/right contact terms before projection. Force columns are after projection.",
        "- `max_force_preprojection_utilization` is the largest raw contact force magnitude divided by its sustained-lateral-acceleration envelope. `max_force_limiter_activity` is zero unless projection scales a contact force.",
        "- `current_proxy_abs_raw_over_unit_command_prior` is a drive-authority proxy against the unit-command current prior, not a measured DRV8871 trip current.",
        "",
        "## Output Scope",
        "",
        f"Discovered {len(runs)} candidate logs; {len(included)} produced at least one sample. Input rows scanned: {total_input_rows}. Extracted qualifying adjacent samples: {total_samples}. Final quiescent/bad-tail rows dropped: {total_tail}.",
        "",
        f"`contact_continuum_feature_sample.csv` keeps a deterministic 1-in-{args.sample_every} sample of qualifying rows, currently {sampled_rows} rows. Aggregated tables use all qualifying rows.",
        "",
        "| Family | Samples | Runs | Limiter-active fraction | Hardware saturation fraction | Mean vbar_rel m/s |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
        *family_lines,
        "",
        "## Outputs",
        "",
        "- `contact_continuum_feature_sample.csv`: compact per-sample feature rows.",
        "- `contact_continuum_contact_bins.csv`: aggregate residual/contact bins by `vbar_rel`, `vbar_lat`, `vbar_yaw`, and signed yaw-rate bin.",
        "- `contact_continuum_phase_summary.csv`: aggregate rows by family and phase/section labels.",
        "- `contact_continuum_force_bins.csv`: aggregate rows by force-utilization and `vbar_rel` bins.",
        "- `contact_continuum_run_summary.csv`: per-run inclusion, bias, tail, and limitation inventory.",
        "- `plant_mirror_constants.csv`: constants parsed from authoritative `Vehicle`, `PlantModel`, and `MotorEncoderDrive` code.",
        "",
        "## Limitations",
        "",
        "- Lateral body velocity is not independently measured in these logs; all right-relative contact features assume `Vr=0` and must be treated as reconstruction features, not measured lateral truth.",
        "- Normal-load transfer is not reconstructed; fan load uses row `fan_duty_cycle` when present, competition metadata when present, otherwise 0.8.",
        "- Legacy competition logs lack saturation/watchdog fields and derive wheel omega from encoder velocity and current wheel radius.",
        "- Residual yaw torque differentiates raw gyro, so timing jitter and gyro noise remain visible in single-sample targets; downstream ablation should prefer aggregate or filtered comparisons.",
        "- Saturated/limited rows are retained with evidence columns instead of removed, so consumers must decide whether to train on them.",
        "",
        f"Aggregate rows written: contact bins {contact_bin_rows}, phase rows {phase_rows}, force bins {force_rows}.",
    ]
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scratch contact-continuum yaw feature extraction.")
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--sample-every", type=int, default=25, help="Deterministic 1-in-N sampled per-sample CSV retention.")
    parser.add_argument("--min-bin-count", type=int, default=80)
    parser.add_argument("--no-competition", action="store_true")
    parser.add_argument("--no-uncertainty", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    params = source_params()
    runs, by_family, by_contact_bins, by_phase, by_force_bins, sampled_rows = process_logs(args)
    contact_bin_rows = write_aggregate(
        args.out_dir / "contact_continuum_contact_bins.csv",
        by_contact_bins,
        ["vbar_rel_bin_mps", "vbar_lat_bin_mps", "vbar_yaw_bin_mps", "yaw_rate_bin_radps"],
        args.min_bin_count,
    )
    phase_rows = write_aggregate(
        args.out_dir / "contact_continuum_phase_summary.csv",
        by_phase,
        ["family", "section_id", "phase_id", "primitive_id", "speed_bin"],
        args.min_bin_count,
    )
    force_rows = write_aggregate(
        args.out_dir / "contact_continuum_force_bins.csv",
        by_force_bins,
        ["max_force_preprojection_utilization_bin", "vbar_rel_bin_mps"],
        args.min_bin_count,
    )
    write_aggregate(
        args.out_dir / "contact_continuum_family_summary.csv",
        by_family,
        ["family"],
        1,
    )
    report = markdown_report(args, runs, by_family, contact_bin_rows, phase_rows, force_rows, sampled_rows)
    (args.out_dir / "contact_continuum_feature_report.md").write_text(report, encoding="utf-8")
    command = (
        f"python codex_analysis\\contact_continuum_yaw_identification\\features\\extract_contact_continuum_features.py "
        f"--out-dir codex_analysis\\contact_continuum_yaw_identification\\features "
        f"--sample-every {args.sample_every} --min-bin-count {args.min_bin_count}"
    )
    (args.out_dir / "commands_run.txt").write_text(command + "\n", encoding="utf-8")
    print(f"out_dir={args.out_dir}")
    print(f"logs={len(runs)}")
    print(f"samples={sum(run.extracted_samples for run in runs)}")
    print(f"sampled_rows={sampled_rows}")
    print(f"contact_bin_rows={contact_bin_rows}")
    print(f"phase_rows={phase_rows}")
    print(f"force_rows={force_rows}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
