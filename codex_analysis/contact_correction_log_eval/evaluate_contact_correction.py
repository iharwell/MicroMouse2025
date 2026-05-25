#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from codex_analysis.contact_continuum_yaw_identification.features import (  # noqa: E402
    extract_contact_continuum_features as prior,
)


@dataclass(frozen=True)
class Params:
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
    drive_wheel_longitudinal_offset_m: float
    front_load_fraction: float
    longitudinal_tire_stiffness_n: float
    front_right_contact_force_gain_n_per_mps: float
    rear_right_contact_force_gain_n_per_mps: float
    fan_downforce_full_duty_n: float
    sustained_lateral_accel_mps2: float
    contact_yaw_patch_force_gain_ns_per_m: float


@dataclass(frozen=True)
class ModelResult:
    model_yaw_moment_nm: float
    patch_delta_yaw_moment_nm: float
    patch_yaw_velocity_m2ps: float
    patch_radius_squared_m2: float
    contact_yaw_correction_scale_n: float
    max_preprojection_utilization: float
    max_limiter_activity: float


@dataclass(frozen=True)
class Sample:
    run_id: str
    family: str
    recommendation: str
    path: Path
    tick: int
    dt_s: float
    yaw_rate_radps: float
    next_yaw_rate_radps: float
    forward_velocity_mps: float
    old_model_yaw_moment_nm: float
    new_model_yaw_moment_nm: float
    observed_yaw_moment_nm: float
    patch_delta_yaw_moment_nm: float
    patch_yaw_velocity_m2ps: float
    patch_correction_scale_n: float
    old_error_radps: float
    new_error_radps: float
    max_preprojection_utilization: float
    max_limiter_activity: float
    saturated: bool
    motion_class: str


@dataclass
class RunSummary:
    run_id: str
    family: str
    recommendation: str
    path: Path
    input_rows: int = 0
    normalized_rows: int = 0
    kept_rows: int = 0
    bias_radps: float = 0.0
    bias_rows: int = 0
    samples: int = 0
    in_place_yaw_samples: int = 0
    moving_yaw_samples: int = 0
    old_rmse_radps: float = 0.0
    new_rmse_radps: float = 0.0
    delta_rmse_radps: float = 0.0
    mean_patch_delta_nm: float = 0.0
    limitation: str = ""
    cutoff_reason: str = ""


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def source_params() -> Params:
    vehicle_h = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "Vehicle.h")
    motor_h = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "MotorEncoderDrive.h")
    plant_h = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "PlantModel.h")
    wheel_diameter_m = prior.const_float(vehicle_h, "kDriveWheelDiameterM")
    no_load_current_a = prior.converted_const_float(
        vehicle_h,
        "kDriveNoLoadCurrentA",
        "MilliAmpsToAmps",
        1.0e-3,
    )
    resistance_ohms = prior.const_float(vehicle_h, "kDriveResistanceOhms")
    nominal_voltage_v = prior.const_float(vehicle_h, "kDriveNominalVoltageV")
    no_load_speed_radps = prior.const_float(vehicle_h, "kDriveNominalNoLoadSpeedRpm") * (2.0 * math.pi / 60.0)
    return Params(
        mass_kg=prior.const_float(vehicle_h, "kPhysicalMassKg"),
        track_width_m=prior.const_float(vehicle_h, "kPhysicalTrackWidthM"),
        yaw_inertia_kg_m2=prior.const_float(vehicle_h, "kPhysicalYawInertiaKgM2"),
        wheel_radius_m=0.5 * wheel_diameter_m,
        wheel_bank_inertia_kg_m2=prior.const_float(motor_h, "kDefaultWheelBankEquivalentInertiaKgM2"),
        drive_voltage_v=prior.const_float(vehicle_h, "kDriveSupplyVoltageV"),
        drive_resistance_ohms=resistance_ohms,
        torque_constant_nm_per_a=prior.converted_const_float(
            vehicle_h,
            "kDriveTorqueConstantNmPerA",
            "MilliNewtonMetersToNewtonMeters",
            1.0e-3,
        ),
        speed_constant_radps_per_volt=no_load_speed_radps / (nominal_voltage_v - (no_load_current_a * resistance_ohms)),
        no_load_current_a=no_load_current_a,
        gear_ratio=prior.const_float(vehicle_h, "kDriveGearRatio"),
        rolling_friction_torque_nm=prior.const_float(plant_h, "kRollingFrictionTorqueNm"),
        static_launch_command=prior.const_float(plant_h, "kReliableLaunchDriveCommand"),
        static_friction_max_speed_mps=prior.const_float(plant_h, "kStaticFrictionMaxSpeedMps"),
        viscous_friction_nm_per_radps=prior.const_float(plant_h, "kViscousFrictionNmPerRadps"),
        drive_wheel_longitudinal_offset_m=prior.const_float(vehicle_h, "kDriveWheelLongitudinalOffsetM"),
        front_load_fraction=prior.const_float(plant_h, "kFrontLoadFraction"),
        longitudinal_tire_stiffness_n=prior.const_float(motor_h, "kDefaultLongitudinalTireStiffnessN"),
        front_right_contact_force_gain_n_per_mps=prior.const_float(motor_h, "kDefaultFrontRightContactForceGainNPerMps"),
        rear_right_contact_force_gain_n_per_mps=prior.const_float(motor_h, "kDefaultRearRightContactForceGainNPerMps"),
        fan_downforce_full_duty_n=prior.const_float(vehicle_h, "kFanDownforceAtFullDutyN"),
        sustained_lateral_accel_mps2=1.91 * 9.80665,
        contact_yaw_patch_force_gain_ns_per_m=prior.const_float(plant_h, "kContactYawPatchForceGainNsPerM"),
    )


def rmse(values: Iterable[float]) -> float:
    materialized = list(values)
    if not materialized:
        return 0.0
    return math.sqrt(statistics.fmean(value * value for value in materialized))


def yaw_denominator_kg_m2(params: Params) -> float:
    wheel_spinup_mass_kg = (2.0 * params.wheel_bank_inertia_kg_m2) / (params.wheel_radius_m * params.wheel_radius_m)
    half_track = 0.5 * abs(params.track_width_m)
    return params.yaw_inertia_kg_m2 + (wheel_spinup_mass_kg * half_track * half_track)


def torque_from_command_detail(command: float, wheel_speed_radps: float, params: Params) -> tuple[float, float]:
    command = prior.clamp(command, -1.0, 1.0)
    applied_voltage_v = command * params.drive_voltage_v
    back_emf_voltage_v = (wheel_speed_radps * params.gear_ratio) / params.speed_constant_radps_per_volt
    armature_current_a = (applied_voltage_v - back_emf_voltage_v) / params.drive_resistance_ohms
    no_load_direction = prior.signed_direction(armature_current_a, wheel_speed_radps)
    load_current_a = armature_current_a - (no_load_direction * params.no_load_current_a)
    if no_load_direction > 0.0 and load_current_a < 0.0:
        load_current_a = 0.0
    elif no_load_direction < 0.0 and load_current_a > 0.0:
        load_current_a = 0.0
    return params.torque_constant_nm_per_a * params.gear_ratio * load_current_a, armature_current_a


def applied_bank_torque(command: float, wheel_speed_radps: float, params: Params) -> float:
    direct, _ = torque_from_command_detail(command, wheel_speed_radps, params)
    positive_limit = max(0.0, torque_from_command_detail(1.0, wheel_speed_radps, params)[0])
    negative_limit = min(0.0, torque_from_command_detail(-1.0, wheel_speed_radps, params)[0])
    limited = prior.clamp(direct, negative_limit, positive_limit) if positive_limit > negative_limit else direct
    static_launch_torque_nm = max(0.0, torque_from_command_detail(params.static_launch_command, 0.0, params)[0])
    surface_speed_mps = params.wheel_radius_m * wheel_speed_radps
    slow_ratio = abs(surface_speed_mps) / params.static_friction_max_speed_mps if params.static_friction_max_speed_mps > 0.0 else 0.0
    launch_torque_nm = static_launch_torque_nm * math.exp(-(slow_ratio * slow_ratio))
    applied = 0.0
    launch_direction = prior.signed_direction(limited, wheel_speed_radps)
    if abs(limited) > launch_torque_nm:
        applied = limited - (launch_direction * launch_torque_nm)
    loss_direction = prior.signed_direction(wheel_speed_radps, applied)
    rolling_loss = (params.rolling_friction_torque_nm * loss_direction) + (
        params.viscous_friction_nm_per_radps * wheel_speed_radps
    )
    return applied - rolling_loss


def model_result(
    row: prior.NormalizedRow,
    forward_velocity_mps: float,
    yaw_rate_radps: float,
    params: Params,
    include_contact_correction: bool,
) -> ModelResult:
    left_torque_nm = applied_bank_torque(row.left_command, row.left_wheel_speed_radps, params)
    right_torque_nm = applied_bank_torque(row.right_command, row.right_wheel_speed_radps, params)
    left_drive_force_n = left_torque_nm / params.wheel_radius_m
    right_drive_force_n = right_torque_nm / params.wheel_radius_m

    fan_duty = prior.clamp(row.fan_duty, 0.0, 1.0)
    total_normal_load_n = (params.mass_kg * 9.80665) + (fan_duty * params.fan_downforce_full_duty_n)
    front_normal_n = 0.5 * params.front_load_fraction * total_normal_load_n
    rear_normal_n = 0.5 * (1.0 - params.front_load_fraction) * total_normal_load_n
    normals = [front_normal_n, front_normal_n, rear_normal_n, rear_normal_n]

    left_bank_normal_n = normals[0] + normals[2]
    right_bank_normal_n = normals[1] + normals[3]
    drive_forward = [
        left_drive_force_n * (normals[0] / left_bank_normal_n) if left_bank_normal_n > 1.0e-4 else 0.5 * left_drive_force_n,
        right_drive_force_n * (normals[1] / right_bank_normal_n) if right_bank_normal_n > 1.0e-4 else 0.5 * right_drive_force_n,
        left_drive_force_n * (normals[2] / left_bank_normal_n) if left_bank_normal_n > 1.0e-4 else 0.5 * left_drive_force_n,
        right_drive_force_n * (normals[3] / right_bank_normal_n) if right_bank_normal_n > 1.0e-4 else 0.5 * right_drive_force_n,
    ]

    half_track_m = 0.5 * abs(params.track_width_m)
    forward_offset_m = abs(params.drive_wheel_longitudinal_offset_m)
    right_positions_m = [-half_track_m, half_track_m, -half_track_m, half_track_m]
    forward_positions_m = [forward_offset_m, forward_offset_m, -forward_offset_m, -forward_offset_m]

    left_body_forward_mps = forward_velocity_mps + (half_track_m * yaw_rate_radps)
    right_body_forward_mps = forward_velocity_mps - (half_track_m * yaw_rate_radps)
    v_rel_f = [
        row.left_velocity_mps - left_body_forward_mps,
        row.right_velocity_mps - right_body_forward_mps,
        row.left_velocity_mps - left_body_forward_mps,
        row.right_velocity_mps - right_body_forward_mps,
    ]
    front_body_right_mps = forward_offset_m * yaw_rate_radps
    rear_body_right_mps = -forward_offset_m * yaw_rate_radps
    v_rel_r = [-front_body_right_mps, -front_body_right_mps, -rear_body_right_mps, -rear_body_right_mps]

    half_longitudinal_stiffness = 0.5 * params.longitudinal_tire_stiffness_n
    raw_forward = [
        drive_forward[0] + (half_longitudinal_stiffness * v_rel_f[0]),
        drive_forward[1] + (half_longitudinal_stiffness * v_rel_f[1]),
        drive_forward[2] + (half_longitudinal_stiffness * v_rel_f[2]),
        drive_forward[3] + (half_longitudinal_stiffness * v_rel_f[3]),
    ]
    raw_right = [
        params.front_right_contact_force_gain_n_per_mps * v_rel_r[0],
        params.front_right_contact_force_gain_n_per_mps * v_rel_r[1],
        params.rear_right_contact_force_gain_n_per_mps * v_rel_r[2],
        params.rear_right_contact_force_gain_n_per_mps * v_rel_r[3],
    ]

    resolved_total_normal_load_n = total_normal_load_n if total_normal_load_n > 1.0e-4 else 1.0e-4
    load_weighted_patch_yaw_velocity = 0.0
    load_weighted_patch_radius_squared = 0.0
    for index in range(4):
        load_weighted_patch_yaw_velocity += normals[index] * (
            (forward_positions_m[index] * v_rel_r[index]) - (right_positions_m[index] * v_rel_f[index])
        )
        load_weighted_patch_radius_squared += normals[index] * (
            (right_positions_m[index] * right_positions_m[index])
            + (forward_positions_m[index] * forward_positions_m[index])
        )
    patch_yaw_velocity_m2ps = load_weighted_patch_yaw_velocity / resolved_total_normal_load_n
    patch_radius_squared_m2 = max(load_weighted_patch_radius_squared / resolved_total_normal_load_n, 1.0e-8)
    correction_scale_n = 0.0
    if include_contact_correction:
        correction_scale_n = (
            params.contact_yaw_patch_force_gain_ns_per_m
            * patch_yaw_velocity_m2ps
            / patch_radius_squared_m2
        )
        for index in range(4):
            load_fraction = normals[index] / resolved_total_normal_load_n
            raw_forward[index] -= load_fraction * right_positions_m[index] * correction_scale_n
            raw_right[index] += load_fraction * forward_positions_m[index] * correction_scale_n

    sustained_mu = (
        (params.sustained_lateral_accel_mps2 * params.mass_kg / total_normal_load_n)
        if total_normal_load_n > 1.0e-4
        else 0.0
    )

    projected_forward: list[float] = []
    projected_right: list[float] = []
    utilizations: list[float] = []
    limiter_activity: list[float] = []
    for index in range(4):
        raw_magnitude_n = math.hypot(raw_forward[index], raw_right[index])
        max_force_n = max(0.0, sustained_mu * normals[index])
        scale = (max_force_n / raw_magnitude_n) if raw_magnitude_n > max_force_n and raw_magnitude_n > 1.0e-4 else 1.0
        projected_forward.append(scale * raw_forward[index])
        projected_right.append(scale * raw_right[index])
        utilizations.append(raw_magnitude_n / max(max_force_n, 1.0e-4))
        limiter_activity.append(max(0.0, 1.0 - scale))

    left_bank_forward_n = projected_forward[0] + projected_forward[2]
    right_bank_forward_n = projected_forward[1] + projected_forward[3]
    front_right_force_n = projected_right[0] + projected_right[1]
    rear_right_force_n = projected_right[2] + projected_right[3]
    yaw_moment_nm = (half_track_m * (left_bank_forward_n - right_bank_forward_n)) + (
        forward_offset_m * (front_right_force_n - rear_right_force_n)
    )
    return ModelResult(
        model_yaw_moment_nm=yaw_moment_nm,
        patch_delta_yaw_moment_nm=0.0,
        patch_yaw_velocity_m2ps=patch_yaw_velocity_m2ps,
        patch_radius_squared_m2=patch_radius_squared_m2,
        contact_yaw_correction_scale_n=correction_scale_n,
        max_preprojection_utilization=max(utilizations),
        max_limiter_activity=max(limiter_activity),
    )


def evaluate_pair(
    current: prior.NormalizedRow,
    nxt: prior.NormalizedRow,
    candidate: prior.LogCandidate,
    recommendation: str,
    params: Params,
    denom: float,
    bias_radps: float,
) -> Sample | None:
    yaw_rate = current.gyro_raw_radps - bias_radps
    next_yaw_rate = nxt.gyro_raw_radps - bias_radps
    forward_velocity = 0.5 * (current.left_velocity_mps + current.right_velocity_mps)
    if not prior.active_contact_sample(current, yaw_rate, forward_velocity):
        return None
    dt_s = nxt.dt_us * 1.0e-6
    measured_yaw_accel = (next_yaw_rate - yaw_rate) / dt_s
    if not math.isfinite(measured_yaw_accel) or abs(measured_yaw_accel) > 4000.0:
        return None
    old_result = model_result(current, forward_velocity, yaw_rate, params, include_contact_correction=False)
    new_result = model_result(current, forward_velocity, yaw_rate, params, include_contact_correction=True)
    observed_yaw_moment_nm = denom * measured_yaw_accel
    old_residual_nm = observed_yaw_moment_nm - old_result.model_yaw_moment_nm
    new_residual_nm = observed_yaw_moment_nm - new_result.model_yaw_moment_nm
    if not math.isfinite(old_residual_nm) or not math.isfinite(new_residual_nm) or abs(old_residual_nm) > 2.0:
        return None
    old_pred_next = yaw_rate + ((old_result.model_yaw_moment_nm / denom) * dt_s)
    new_pred_next = yaw_rate + ((new_result.model_yaw_moment_nm / denom) * dt_s)
    saturated = current.saturation_flags != 0 or current.clipping_flags != 0 or current.watchdog_flags != 0
    if abs(yaw_rate) >= 0.20 and abs(forward_velocity) < 0.05:
        motion_class = "in_place_yaw"
    elif abs(yaw_rate) >= 0.20 and abs(forward_velocity) >= 0.05:
        motion_class = "moving_yaw"
    elif abs(forward_velocity) >= 0.05:
        motion_class = "mostly_forward"
    else:
        motion_class = "low_motion_commanded"
    return Sample(
        run_id=candidate.run_id,
        family=candidate.family,
        recommendation=recommendation,
        path=candidate.path,
        tick=current.tick,
        dt_s=dt_s,
        yaw_rate_radps=yaw_rate,
        next_yaw_rate_radps=next_yaw_rate,
        forward_velocity_mps=forward_velocity,
        old_model_yaw_moment_nm=old_result.model_yaw_moment_nm,
        new_model_yaw_moment_nm=new_result.model_yaw_moment_nm,
        observed_yaw_moment_nm=observed_yaw_moment_nm,
        patch_delta_yaw_moment_nm=new_result.model_yaw_moment_nm - old_result.model_yaw_moment_nm,
        patch_yaw_velocity_m2ps=new_result.patch_yaw_velocity_m2ps,
        patch_correction_scale_n=new_result.contact_yaw_correction_scale_n,
        old_error_radps=old_pred_next - next_yaw_rate,
        new_error_radps=new_pred_next - next_yaw_rate,
        max_preprojection_utilization=new_result.max_preprojection_utilization,
        max_limiter_activity=new_result.max_limiter_activity,
        saturated=saturated,
        motion_class=motion_class,
    )


def load_recommendations() -> dict[str, str]:
    path = REPO_ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "data_quality" / "data_quality_recommendations_by_run.csv"
    if not path.is_file():
        return {}
    with path.open("r", newline="", encoding="utf-8") as f:
        return {row["run_id"]: row["recommendation"] for row in csv.DictReader(f)}


def collect_samples(args: argparse.Namespace, params: Params) -> tuple[list[RunSummary], list[Sample]]:
    recommendations = load_recommendations()
    denom = yaw_denominator_kg_m2(params)
    candidates = prior.discover_logs(include_competition=not args.no_competition, include_uncertainty=args.include_uncertainty)
    runs: list[RunSummary] = []
    samples: list[Sample] = []
    for candidate in candidates:
        recommendation = recommendations.get(candidate.run_id, "not_in_data_quality")
        metadata, rows, limitation, input_rows = prior.read_normalized_rows(candidate, params)
        del metadata
        run = RunSummary(
            run_id=candidate.run_id,
            family=candidate.family,
            recommendation=recommendation,
            path=candidate.path,
            input_rows=input_rows,
            normalized_rows=len(rows),
            limitation=limitation,
        )
        if candidate.schema == "legacy_competition":
            run.limitation = (
                "legacy competition schema: wheel omega derived from velocity; saturation/watchdog unavailable; "
                "fan duty from metadata/default"
            )
        if rows:
            bias, bias_rows = prior.estimate_bias(rows)
            cutoff_index, cutoff_reason, _, _ = prior.tail_cut_index(rows, bias)
            kept_rows = rows[:cutoff_index]
            run.kept_rows = len(kept_rows)
            run.bias_radps = bias
            run.bias_rows = bias_rows
            run.cutoff_reason = cutoff_reason
            run_samples: list[Sample] = []
            for current, nxt in zip(kept_rows, kept_rows[1:]):
                if not prior.valid_adjacent(current, nxt, candidate.family):
                    continue
                sample = evaluate_pair(current, nxt, candidate, recommendation, params, denom, bias)
                if sample is not None:
                    run_samples.append(sample)
            run.samples = len(run_samples)
            run.in_place_yaw_samples = sum(1 for sample in run_samples if sample.motion_class == "in_place_yaw")
            run.moving_yaw_samples = sum(1 for sample in run_samples if sample.motion_class == "moving_yaw")
            run.old_rmse_radps = rmse(sample.old_error_radps for sample in run_samples)
            run.new_rmse_radps = rmse(sample.new_error_radps for sample in run_samples)
            run.delta_rmse_radps = run.new_rmse_radps - run.old_rmse_radps
            if run_samples:
                run.mean_patch_delta_nm = statistics.fmean(sample.patch_delta_yaw_moment_nm for sample in run_samples)
            samples.extend(run_samples)
        else:
            run.cutoff_reason = "excluded"
        runs.append(run)
        print(f"{candidate.family}:{candidate.run_id}: rows={input_rows} samples={run.samples} old={run.old_rmse_radps:.9f} new={run.new_rmse_radps:.9f}")
    return runs, samples


def aggregate_row(label: str, samples: list[Sample]) -> dict[str, str]:
    old = rmse(sample.old_error_radps for sample in samples)
    new = rmse(sample.new_error_radps for sample in samples)
    worsened = sum(1 for sample in samples if abs(sample.new_error_radps) > abs(sample.old_error_radps))
    return {
        "dataset": label,
        "samples": str(len(samples)),
        "old_rmse_radps": f"{old:.9f}",
        "new_rmse_radps": f"{new:.9f}",
        "delta_rmse_radps": f"{new - old:.9f}",
        "relative_delta_pct": f"{((new / old - 1.0) * 100.0) if old > 0.0 else 0.0:.6f}",
        "worsened_sample_fraction": f"{(worsened / len(samples)) if samples else 0.0:.9f}",
        "mean_patch_delta_yaw_moment_nm": f"{statistics.fmean(sample.patch_delta_yaw_moment_nm for sample in samples) if samples else 0.0:.9f}",
        "rmse_patch_delta_yaw_moment_nm": f"{rmse(sample.patch_delta_yaw_moment_nm for sample in samples):.9f}",
    }


def write_csv(path: Path, rows: list[dict[str, str]], fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_outputs(args: argparse.Namespace, params: Params, runs: list[RunSummary], samples: list[Sample]) -> None:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    constants = [
        ("mass_kg", params.mass_kg),
        ("track_width_m", params.track_width_m),
        ("yaw_inertia_kg_m2", params.yaw_inertia_kg_m2),
        ("yaw_denominator_including_wheel_spinup_kg_m2", yaw_denominator_kg_m2(params)),
        ("wheel_radius_m", params.wheel_radius_m),
        ("drive_wheel_longitudinal_offset_m", params.drive_wheel_longitudinal_offset_m),
        ("wheel_bank_inertia_kg_m2", params.wheel_bank_inertia_kg_m2),
        ("longitudinal_tire_stiffness_n", params.longitudinal_tire_stiffness_n),
        ("front_right_contact_force_gain_n_per_mps", params.front_right_contact_force_gain_n_per_mps),
        ("rear_right_contact_force_gain_n_per_mps", params.rear_right_contact_force_gain_n_per_mps),
        ("contact_yaw_patch_force_gain_ns_per_m", params.contact_yaw_patch_force_gain_ns_per_m),
    ]
    write_csv(
        args.out_dir / "plant_mirror_constants.csv",
        [{"name": name, "value": f"{value:.12g}"} for name, value in constants],
        ["name", "value"],
    )

    run_rows = []
    for run in runs:
        run_rows.append(
            {
                "run_id": run.run_id,
                "family": run.family,
                "recommendation": run.recommendation,
                "path": str(run.path.relative_to(REPO_ROOT)) if run.path.is_relative_to(REPO_ROOT) else str(run.path),
                "input_rows": str(run.input_rows),
                "normalized_rows": str(run.normalized_rows),
                "kept_rows": str(run.kept_rows),
                "bias_radps": f"{run.bias_radps:.9f}",
                "bias_rows": str(run.bias_rows),
                "samples": str(run.samples),
                "in_place_yaw_samples": str(run.in_place_yaw_samples),
                "moving_yaw_samples": str(run.moving_yaw_samples),
                "old_rmse_radps": f"{run.old_rmse_radps:.9f}",
                "new_rmse_radps": f"{run.new_rmse_radps:.9f}",
                "delta_rmse_radps": f"{run.delta_rmse_radps:.9f}",
                "mean_patch_delta_yaw_moment_nm": f"{run.mean_patch_delta_nm:.9f}",
                "cutoff_reason": run.cutoff_reason,
                "limitation": run.limitation,
            }
        )
    write_csv(args.out_dir / "per_run_rmse.csv", run_rows, list(run_rows[0].keys()) if run_rows else ["run_id"])

    families = sorted({sample.family for sample in samples})
    aggregate_specs: list[tuple[str, Callable[[Sample], bool]]] = [
        ("all_included", lambda sample: True),
        ("open_floor_only", lambda sample: sample.family == "open_floor"),
        ("competition_only", lambda sample: sample.family.startswith("competition")),
        ("fit_authoritative_open_floor", lambda sample: sample.family == "open_floor" and sample.recommendation == "fit_authoritative"),
        ("fit_downweighted_open_floor", lambda sample: sample.family == "open_floor" and sample.recommendation == "fit_downweighted"),
        ("validation_only_open_floor", lambda sample: sample.family == "open_floor" and sample.recommendation == "validation_only"),
    ]
    aggregate_rows = []
    for label, predicate in aggregate_specs:
        subset = [sample for sample in samples if predicate(sample)]
        if subset:
            aggregate_rows.append(aggregate_row(label, subset))
    for family in families:
        subset = [sample for sample in samples if sample.family == family]
        aggregate_rows.append(aggregate_row(f"family:{family}", subset))
    write_csv(args.out_dir / "aggregate_rmse.csv", aggregate_rows, list(aggregate_rows[0].keys()))

    motion_rows = []
    for family_label, family_predicate in [("all", lambda sample: True)] + [
        (family, lambda sample, family=family: sample.family == family) for family in families
    ]:
        for motion_class in ["in_place_yaw", "moving_yaw", "mostly_forward", "low_motion_commanded"]:
            subset = [sample for sample in samples if family_predicate(sample) and sample.motion_class == motion_class]
            if subset:
                row = aggregate_row(f"{family_label}:{motion_class}", subset)
                row["family"] = family_label
                row["motion_class"] = motion_class
                motion_rows.append(row)
    motion_fields = ["family", "motion_class"] + [key for key in motion_rows[0].keys() if key not in {"family", "motion_class"}]
    write_csv(args.out_dir / "family_motion_rmse.csv", motion_rows, motion_fields)

    bin_groups: dict[tuple[str, str, str], list[Sample]] = {}
    for sample in samples:
        vf_bin = f"{prior.round_to_step(sample.forward_velocity_mps, 0.10):.2f}"
        yaw_bin = f"{prior.round_to_step(sample.yaw_rate_radps, 0.50):.2f}"
        bin_groups.setdefault((sample.family, vf_bin, yaw_bin), []).append(sample)
    bin_rows = []
    for (family, vf_bin, yaw_bin), group in sorted(bin_groups.items()):
        if len(group) < args.min_bin_count:
            continue
        row = aggregate_row(f"{family}:{vf_bin}:{yaw_bin}", group)
        row.update({"family": family, "forward_velocity_bin_mps": vf_bin, "yaw_rate_bin_radps": yaw_bin})
        bin_rows.append(row)
    bin_fields = ["family", "forward_velocity_bin_mps", "yaw_rate_bin_radps"] + [
        key for key in bin_rows[0].keys() if key not in {"family", "forward_velocity_bin_mps", "yaw_rate_bin_radps"}
    ] if bin_rows else ["family", "forward_velocity_bin_mps", "yaw_rate_bin_radps"]
    write_csv(args.out_dir / "bin_rmse.csv", bin_rows, bin_fields)

    sample_rows = []
    for sample in samples[:: max(1, args.sample_every)]:
        sample_rows.append(
            {
                "run_id": sample.run_id,
                "family": sample.family,
                "recommendation": sample.recommendation,
                "tick": str(sample.tick),
                "dt_s": f"{sample.dt_s:.9f}",
                "yaw_rate_radps": f"{sample.yaw_rate_radps:.9f}",
                "forward_velocity_mps": f"{sample.forward_velocity_mps:.9f}",
                "old_error_radps": f"{sample.old_error_radps:.9f}",
                "new_error_radps": f"{sample.new_error_radps:.9f}",
                "patch_delta_yaw_moment_nm": f"{sample.patch_delta_yaw_moment_nm:.9f}",
                "patch_yaw_velocity_m2ps": f"{sample.patch_yaw_velocity_m2ps:.9f}",
                "patch_correction_scale_n": f"{sample.patch_correction_scale_n:.9f}",
                "motion_class": sample.motion_class,
            }
        )
    write_csv(args.out_dir / "sampled_predictions.csv", sample_rows, list(sample_rows[0].keys()) if sample_rows else ["run_id"])

    report = markdown_report(args, runs, samples, aggregate_rows, motion_rows, bin_rows)
    (args.out_dir / "evaluation_report.md").write_text(report, encoding="utf-8")


def markdown_report(
    args: argparse.Namespace,
    runs: list[RunSummary],
    samples: list[Sample],
    aggregate_rows: list[dict[str, str]],
    motion_rows: list[dict[str, str]],
    bin_rows: list[dict[str, str]],
) -> str:
    included_runs = [run for run in runs if run.samples > 0]
    old_all = next(row for row in aggregate_rows if row["dataset"] == "all_included")
    old_open = next(row for row in aggregate_rows if row["dataset"] == "open_floor_only")
    comp = next((row for row in aggregate_rows if row["dataset"] == "competition_only"), None)
    fit = next((row for row in aggregate_rows if row["dataset"] == "fit_authoritative_open_floor"), None)
    worsened_runs = sum(1 for run in included_runs if run.delta_rmse_radps > 0.0)
    improved_runs = sum(1 for run in included_runs if run.delta_rmse_radps < 0.0)
    worsened_bins = sum(1 for row in bin_rows if float(row["delta_rmse_radps"]) > 0.0)
    improved_bins = sum(1 for row in bin_rows if float(row["delta_rmse_radps"]) < 0.0)
    lines = [
        "# Contact Correction Log Evaluation",
        "",
        "Scratch analysis only. No production code or tests were modified.",
        "",
        "## Method",
        "",
        "Targets use actual sensor data only: raw gyro yaw rate minus an independently estimated stationary bias where rows allow it, encoder-derived forward velocity and wheel speeds, logged drive commands, and timestamps. UKF states and logged UKF gyro bias are not used as targets.",
        "",
        "The evaluator mirrors the yaw-relevant PlantModel path twice: old/pre-correction contact force requests, then the new contact-continuum patch-force couple before force projection. The mirror uses the current constants parsed from Vehicle, MotorEncoderDrive, and PlantModel. Fidelity assumptions: lateral velocity is unavailable in these logs and is set to zero, normal load transfer is not reconstructed, and legacy competition CSVs lack saturation/watchdog fields and derive wheel omega from linear velocity.",
        "",
        "Windows use the prior contact-continuum extractor gating: adjacent valid ticks, same phase/primitive key, raw-gyro differentiated yaw acceleration under 4000 rad/s^2, active command/motion rows, and old-model residual magnitude under 2 Nm to reject derivative spikes.",
        "",
        "## Scope",
        "",
        f"Candidate logs scanned: {len(runs)}. Runs with samples: {len(included_runs)}. Samples: {len(samples)}.",
        "",
        "## Aggregate RMSE",
        "",
        "| Dataset | Samples | Old RMSE | New RMSE | Delta | Relative delta | Worsened sample fraction |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in aggregate_rows:
        lines.append(
            f"| {row['dataset']} | {row['samples']} | {row['old_rmse_radps']} | {row['new_rmse_radps']} | {row['delta_rmse_radps']} | {row['relative_delta_pct']}% | {row['worsened_sample_fraction']} |"
        )
    lines.extend(
        [
            "",
            "## Motion Split",
            "",
            "| Family | Motion | Samples | Old RMSE | New RMSE | Delta | Patch RMSE Nm |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in motion_rows:
        if row["family"] in {"all", "open_floor", "competition_diag", "competition_aux"}:
            lines.append(
                f"| {row['family']} | {row['motion_class']} | {row['samples']} | {row['old_rmse_radps']} | {row['new_rmse_radps']} | {row['delta_rmse_radps']} | {row['rmse_patch_delta_yaw_moment_nm']} |"
            )
    lines.extend(
        [
            "",
            "## High-Level Per-Run Result",
            "",
            "| Run family | Improved runs | Worsened runs |",
            "| --- | ---: | ---: |",
            f"| all | {improved_runs} | {worsened_runs} |",
        ]
    )
    for family in sorted({run.family for run in included_runs}):
        family_runs = [run for run in included_runs if run.family == family]
        lines.append(
            f"| {family} | {sum(1 for run in family_runs if run.delta_rmse_radps < 0.0)} | {sum(1 for run in family_runs if run.delta_rmse_radps > 0.0)} |"
        )
    lines.extend(
        [
            "",
            "## Bin Direction",
            "",
            f"Signed velocity/yaw bins with at least {args.min_bin_count} samples: {len(bin_rows)}. Improved bins: {improved_bins}; worsened bins: {worsened_bins}. Full rows are in bin_rmse.csv.",
            "",
            "## Baseline Comparison",
            "",
            f"All-included old/current RMSE in this direct replay is {old_all['old_rmse_radps']} rad/s versus the prior expanded current baseline 0.272693052 rad/s. New corrected is {old_all['new_rmse_radps']} rad/s.",
            f"Open-floor old/current RMSE in this direct replay is {old_open['old_rmse_radps']} rad/s versus prior open-floor current 0.258347167 rad/s.",
        ]
    )
    if comp is not None:
        lines.append(
            f"Competition old/current RMSE in this direct replay is {comp['old_rmse_radps']} rad/s versus prior competition current 0.308298322 rad/s."
        )
    if fit is not None:
        lines.append(
            f"Fit-authoritative open-floor runs contribute {fit['samples']} samples: old {fit['old_rmse_radps']} rad/s, new {fit['new_rmse_radps']} rad/s."
        )
    lines.extend(
        [
            "",
            "The earlier fitted/scalar and surface-corrected baselines remain diagnostic comparisons, not production behavior: scalar four-run current 0.226296125 rad/s, scalar fit 0.161693351 rad/s, expanded surface-corrected all/open/competition 0.248282427/0.237744117/0.258216285 rad/s, and validation old/current to surface 0.302028502 -> 0.263993314 rad/s.",
            "",
            "## Recommendation",
            "",
            "Use the aggregate and motion-split deltas to decide whether the conservative gain should remain. A materially useful coefficient should reduce both in-place yaw and moving-yaw RMSE without broad run/bin regressions; otherwise it should be tuned or disabled pending cleaner targeted runs.",
        ]
    )
    return "\n".join(lines) + "\n"


def verify_outputs(out_dir: Path, runs: list[RunSummary], samples: list[Sample]) -> None:
    required = [
        "plant_mirror_constants.csv",
        "per_run_rmse.csv",
        "aggregate_rmse.csv",
        "family_motion_rmse.csv",
        "bin_rmse.csv",
        "sampled_predictions.csv",
        "evaluation_report.md",
        "commands_run.txt",
    ]
    missing = [name for name in required if not (out_dir / name).is_file()]
    if missing:
        raise RuntimeError(f"Missing expected outputs: {missing}")
    if not samples:
        raise RuntimeError("No evaluation samples produced.")
    if not any(run.recommendation == "fit_authoritative" and run.samples > 0 for run in runs):
        raise RuntimeError("No fit-authoritative open-floor run contributed samples.")
    for sample in samples:
        if not all(math.isfinite(value) for value in [sample.old_error_radps, sample.new_error_radps, sample.patch_delta_yaw_moment_nm]):
            raise RuntimeError("Non-finite sample metric found.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate PlantModel contact-continuum correction on historical logs.")
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--min-bin-count", type=int, default=80)
    parser.add_argument("--sample-every", type=int, default=200)
    parser.add_argument("--no-competition", action="store_true")
    parser.add_argument("--include-uncertainty", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    params = source_params()
    runs, samples = collect_samples(args, params)
    write_outputs(args, params, runs, samples)
    command = (
        "python codex_analysis\\contact_correction_log_eval\\evaluate_contact_correction.py "
        f"--out-dir {args.out_dir} --min-bin-count {args.min_bin_count} --sample-every {args.sample_every}"
    )
    if args.no_competition:
        command += " --no-competition"
    if args.include_uncertainty:
        command += " --include-uncertainty"
    (args.out_dir / "commands_run.txt").write_text(command + "\n", encoding="utf-8")
    verify_outputs(args.out_dir, runs, samples)
    aggregate_path = args.out_dir / "aggregate_rmse.csv"
    print(f"out_dir={args.out_dir}")
    print(f"runs={len(runs)}")
    print(f"samples={len(samples)}")
    print(f"aggregate={aggregate_path}")
    print("script_verification=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
