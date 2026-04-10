from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from pathlib import Path

from open_floor_recovery import IMU_POSITION_BODY_X_M
from open_floor_recovery import IMU_POSITION_BODY_Y_M
from open_floor_recovery import RunPlantParameters
from open_floor_recovery import drive_torque_from_command
from open_floor_recovery import independent_gyro_radps
from open_floor_recovery import load_control_log_parameters
from open_floor_recovery import parse_float
from open_floor_recovery import parse_key_value_fields
from open_floor_recovery import row_dt_seconds


EARLY_INERTIA_SAMPLE_INDEX = 2
DRAG_FIT_MAX_ABS_LONGITUDINAL_ACCEL_MPS2 = 0.12
DRAG_FIT_MAX_ABS_WHEEL_ACCEL_RADPS2 = 20.0
DRAG_FIT_MIN_ABS_WHEEL_SPEED_RADPS = 0.2
LONGITUDINAL_STIFFNESS_MIN_ABS_KAPPA = 0.01
LONGITUDINAL_STIFFNESS_STABILITY_RATIO_LIMIT = 2.5
MOTION_THRESHOLD_END_SPEED_MPS = 0.02


@dataclass
class LaunchMeanTracePoint:
    sample_index: int
    abs_command: float
    measured_linear_speed_mps: float
    average_encoder_omega_radps: float
    gyro_raw_radps: float
    accel_body_y_mps2: float
    dt_seconds: float
    repeat_count: int


@dataclass
class LongitudinalStiffnessCommandFit:
    abs_command: float
    sample_count: int
    end_speed_mps: float
    apparent_longitudinal_tire_stiffness_n: float | None


@dataclass
class TirePlantFitSummary:
    run_id: str | None
    launch_command_bin_count: int
    launch_motion_threshold_lower_command: float | None
    launch_motion_threshold_upper_command: float | None
    apparent_equivalent_wheel_inertia_kg_m2: float | None
    apparent_equivalent_wheel_inertia_bin_count: int
    apparent_equivalent_wheel_inertia_sample_index: int
    apparent_rolling_friction_torque_nm: float | None
    apparent_viscous_friction_nm_per_radps: float | None
    apparent_drag_fit_row_count: int
    apparent_drag_fit_residual_sigma_nm: float | None
    apparent_longitudinal_tire_stiffness_positive_bin_median_n: float | None
    apparent_longitudinal_tire_stiffness_stable: bool
    apparent_longitudinal_tire_stiffness_fits: list[LongitudinalStiffnessCommandFit]
    can_identify_cornering_stiffness: bool
    can_identify_lateral_damping: bool
    can_identify_peak_friction: bool
    lateral_identifiability_reason: str | None


@dataclass
class LaunchFitParameters:
    drive: RunPlantParameters
    effective_longitudinal_mass_kg: float


def load_run_id(path: Path | None) -> str | None:
    if path is None or not path.is_file():
        return None
    with path.open(encoding="utf-8", errors="replace") as control_log:
        for line in control_log:
            if "run_id=" not in line:
                continue
            for field in line.split(";"):
                field = field.strip()
                if "run_id=" in field:
                    value = field.split("run_id=", 1)[1].strip()
                    return value or None
    return None


def load_launch_fit_parameters(path: Path | None) -> LaunchFitParameters | None:
    if path is None or not path.is_file():
        return None
    drive = load_control_log_parameters(path)
    if drive is None:
        return None
    effective_longitudinal_mass_kg: float | None = None
    mass_kg: float | None = None
    with path.open(encoding="utf-8", errors="replace") as control_log:
        for line in control_log:
            if "ukf_dump_params_mass_geometry:" not in line:
                continue
            fields = parse_key_value_fields(
                line.split("ukf_dump_params_mass_geometry:", 1)[1].strip()
            )
            effective_longitudinal_mass_kg = fields.get("effective_longitudinal_mass_kg")
            mass_kg = fields.get("mass_kg")
            break
    chosen_mass_kg = effective_longitudinal_mass_kg if effective_longitudinal_mass_kg is not None else mass_kg
    if chosen_mass_kg is None or chosen_mass_kg <= 0.0:
        return None
    return LaunchFitParameters(
        drive=drive,
        effective_longitudinal_mass_kg=chosen_mass_kg,
    )


def mean_trace_point_from_rows(
    abs_command: float,
    sample_index: int,
    rows: list[dict[str, str]],
) -> LaunchMeanTracePoint:
    return LaunchMeanTracePoint(
        sample_index=sample_index,
        abs_command=abs_command,
        measured_linear_speed_mps=statistics.fmean(parse_float(row["measured_linear_speed_mps"]) for row in rows),
        average_encoder_omega_radps=statistics.fmean(
            0.5 * (parse_float(row["left_encoder_omega_radps"]) + parse_float(row["right_encoder_omega_radps"]))
            for row in rows
        ),
        gyro_raw_radps=statistics.fmean(parse_float(row["gyro_raw_radps"]) for row in rows),
        accel_body_y_mps2=statistics.fmean(parse_float(row["accel_body_y_mps2"]) for row in rows),
        dt_seconds=statistics.fmean(row_dt_seconds(row) for row in rows),
        repeat_count=len(rows),
    )


def build_launch_mean_traces(
    launch_rows_by_repeat: dict[int, list[dict[str, str]]],
) -> dict[float, list[LaunchMeanTracePoint]]:
    grouped_points: dict[tuple[float, int], list[dict[str, str]]] = {}
    for rows in launch_rows_by_repeat.values():
        active_index = 0
        for row in rows:
            left_command = parse_float(row["left_drive_command"])
            right_command = parse_float(row["right_drive_command"])
            if (
                abs(left_command) <= 1.0e-6 or
                abs(right_command) <= 1.0e-6 or
                abs(left_command - right_command) > 1.0e-6 or
                int(row["saturation_flags"]) != 0 or
                int(row["clipping_flags"]) != 0 or
                int(row["watchdog_flags"]) != 0
            ):
                continue
            sign = 1.0 if left_command >= 0.0 else -1.0
            abs_command = round(abs(left_command), 2)
            normalized_row = dict(row)
            normalized_row["measured_linear_speed_mps"] = f"{sign * parse_float(row['measured_linear_speed_mps'])}"
            normalized_row["left_encoder_omega_radps"] = f"{sign * parse_float(row['left_encoder_omega_radps'])}"
            normalized_row["right_encoder_omega_radps"] = f"{sign * parse_float(row['right_encoder_omega_radps'])}"
            normalized_row["gyro_raw_radps"] = f"{sign * parse_float(row['gyro_raw_radps'])}"
            normalized_row["accel_body_y_mps2"] = f"{sign * parse_float(row['accel_body_y_mps2'])}"
            grouped_points.setdefault((abs_command, active_index), []).append(normalized_row)
            active_index += 1
    traces: dict[float, list[LaunchMeanTracePoint]] = {}
    for abs_command in sorted({key[0] for key in grouped_points}):
        trace = [
            mean_trace_point_from_rows(abs_command, sample_index, grouped_points[(abs_command, sample_index)])
            for _, sample_index in sorted(
                (key for key in grouped_points if key[0] == abs_command),
                key=lambda item: item[1],
            )
        ]
        traces[abs_command] = trace
    return traces


def central_derivative(
    previous: LaunchMeanTracePoint,
    current: LaunchMeanTracePoint,
    next_point: LaunchMeanTracePoint,
    attribute: str,
) -> float:
    dt_seconds = max(previous.dt_seconds + current.dt_seconds, 1.0e-6)
    return (getattr(next_point, attribute) - getattr(previous, attribute)) / dt_seconds


def origin_longitudinal_accel_mps2(
    trace: list[LaunchMeanTracePoint],
    index: int,
    gyro_bias_radps: float,
    accel_bias_y_mps2: float,
) -> float:
    point = trace[index]
    gyro_radps = independent_gyro_radps({"gyro_raw_radps": f"{point.gyro_raw_radps}"}, gyro_bias_radps)
    yaw_accel_radps2 = 0.0
    if 0 < index < (len(trace) - 1):
        previous = trace[index - 1]
        next_point = trace[index + 1]
        dt_seconds = max(previous.dt_seconds + point.dt_seconds, 1.0e-6)
        yaw_accel_radps2 = (
            independent_gyro_radps({"gyro_raw_radps": f"{next_point.gyro_raw_radps}"}, gyro_bias_radps) -
            independent_gyro_radps({"gyro_raw_radps": f"{previous.gyro_raw_radps}"}, gyro_bias_radps)
        ) / dt_seconds
    imu_accel_y_mps2 = point.accel_body_y_mps2 - accel_bias_y_mps2
    return imu_accel_y_mps2 + (gyro_radps * gyro_radps * IMU_POSITION_BODY_Y_M) + (yaw_accel_radps2 * IMU_POSITION_BODY_X_M)


def fit_affine(x_values: list[float], y_values: list[float]) -> tuple[float, float] | None:
    if len(x_values) != len(y_values) or len(x_values) < 2:
        return None
    mean_x = statistics.fmean(x_values)
    mean_y = statistics.fmean(y_values)
    denominator = sum((x - mean_x) * (x - mean_x) for x in x_values)
    if denominator <= 0.0:
        return None
    slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(x_values, y_values)) / denominator
    intercept = mean_y - (slope * mean_x)
    return intercept, slope


def fit_apparent_drive_drag(
    traces: dict[float, list[LaunchMeanTracePoint]],
    params: LaunchFitParameters,
) -> tuple[float | None, float | None, int, float | None]:
    wheel_speed_samples: list[float] = []
    residual_torque_samples_nm: list[float] = []
    for trace in traces.values():
        for index in range(1, len(trace) - 1):
            previous = trace[index - 1]
            point = trace[index]
            next_point = trace[index + 1]
            longitudinal_accel_mps2 = central_derivative(previous, point, next_point, "measured_linear_speed_mps")
            wheel_accel_radps2 = central_derivative(previous, point, next_point, "average_encoder_omega_radps")
            if (
                abs(longitudinal_accel_mps2) > DRAG_FIT_MAX_ABS_LONGITUDINAL_ACCEL_MPS2 or
                abs(wheel_accel_radps2) > DRAG_FIT_MAX_ABS_WHEEL_ACCEL_RADPS2 or
                point.average_encoder_omega_radps < DRAG_FIT_MIN_ABS_WHEEL_SPEED_RADPS
            ):
                continue
            contact_force_bank_n = 0.5 * params.effective_longitudinal_mass_kg * longitudinal_accel_mps2
            drive_torque_nm = drive_torque_from_command(
                point.abs_command,
                point.average_encoder_omega_radps,
                params.drive,
            )
            residual_torque_samples_nm.append(drive_torque_nm - (params.drive.wheel_radius_m * contact_force_bank_n))
            wheel_speed_samples.append(point.average_encoder_omega_radps)
    fit = fit_affine(wheel_speed_samples, residual_torque_samples_nm)
    if fit is None:
        return None, None, 0, None
    intercept_nm, slope_nm_per_radps = fit
    residuals = [
        residual_torque_nm - (intercept_nm + (slope_nm_per_radps * wheel_speed_radps))
        for wheel_speed_radps, residual_torque_nm in zip(wheel_speed_samples, residual_torque_samples_nm)
    ]
    return intercept_nm, slope_nm_per_radps, len(wheel_speed_samples), statistics.pstdev(residuals)


def fit_apparent_equivalent_wheel_inertia(
    traces: dict[float, list[LaunchMeanTracePoint]],
    params: LaunchFitParameters,
) -> tuple[float | None, int]:
    values: list[float] = []
    for trace in traces.values():
        if len(trace) <= (EARLY_INERTIA_SAMPLE_INDEX + 1):
            continue
        previous = trace[EARLY_INERTIA_SAMPLE_INDEX - 1]
        point = trace[EARLY_INERTIA_SAMPLE_INDEX]
        next_point = trace[EARLY_INERTIA_SAMPLE_INDEX + 1]
        longitudinal_accel_mps2 = central_derivative(previous, point, next_point, "measured_linear_speed_mps")
        wheel_accel_radps2 = central_derivative(previous, point, next_point, "average_encoder_omega_radps")
        if abs(wheel_accel_radps2) < 1.0e-6:
            continue
        contact_force_bank_n = 0.5 * params.effective_longitudinal_mass_kg * longitudinal_accel_mps2
        drive_torque_nm = drive_torque_from_command(
            point.abs_command,
            point.average_encoder_omega_radps,
            params.drive,
        )
        inertia_kg_m2 = (
            drive_torque_nm - (params.drive.wheel_radius_m * contact_force_bank_n)
        ) / wheel_accel_radps2
        if math.isfinite(inertia_kg_m2) and 0.0 < inertia_kg_m2 < 1.0e-3:
            values.append(inertia_kg_m2)
    return (statistics.median(values), len(values)) if values else (None, 0)


def fit_apparent_longitudinal_stiffness(
    traces: dict[float, list[LaunchMeanTracePoint]],
    gyro_bias_radps: float,
    accel_bias_y_mps2: float,
    params: LaunchFitParameters,
) -> tuple[float | None, bool, list[LongitudinalStiffnessCommandFit]]:
    fits: list[LongitudinalStiffnessCommandFit] = []
    positive_values: list[float] = []
    for abs_command, trace in sorted(traces.items()):
        integrated_speed_mps = 0.0
        raw_speed_trace_mps: list[float] = []
        elapsed_time_s = 0.0
        elapsed_trace_s: list[float] = []
        longitudinal_accel_trace_mps2: list[float] = []
        for index, point in enumerate(trace):
            origin_accel_y_mps2 = origin_longitudinal_accel_mps2(trace, index, gyro_bias_radps, accel_bias_y_mps2)
            integrated_speed_mps += origin_accel_y_mps2 * point.dt_seconds
            elapsed_time_s += point.dt_seconds
            raw_speed_trace_mps.append(integrated_speed_mps)
            elapsed_trace_s.append(elapsed_time_s)
            longitudinal_accel_trace_mps2.append(origin_accel_y_mps2)
        drift_correction_mps = trace[-1].measured_linear_speed_mps - raw_speed_trace_mps[-1]
        total_elapsed_time_s = max(elapsed_trace_s[-1], 1.0e-6)
        kappa_values: list[float] = []
        force_values_n: list[float] = []
        for point, elapsed_s, raw_speed_mps, longitudinal_accel_mps2 in zip(
            trace,
            elapsed_trace_s,
            raw_speed_trace_mps,
            longitudinal_accel_trace_mps2,
        ):
            corrected_speed_mps = raw_speed_mps + (drift_correction_mps * (elapsed_s / total_elapsed_time_s))
            kappa = (
                (params.drive.wheel_radius_m * point.average_encoder_omega_radps) - corrected_speed_mps
            ) / max(abs(corrected_speed_mps), 0.05)
            if abs(kappa) < LONGITUDINAL_STIFFNESS_MIN_ABS_KAPPA:
                continue
            kappa_values.append(kappa)
            force_values_n.append(0.5 * params.effective_longitudinal_mass_kg * longitudinal_accel_mps2)
        apparent_stiffness_n: float | None = None
        if kappa_values:
            denominator = sum(kappa * kappa for kappa in kappa_values)
            if denominator > 0.0:
                apparent_stiffness_n = sum(
                    kappa * force_n
                    for kappa, force_n in zip(kappa_values, force_values_n)
                ) / denominator
                if apparent_stiffness_n > 0.0:
                    positive_values.append(apparent_stiffness_n)
        fits.append(
            LongitudinalStiffnessCommandFit(
                abs_command=abs_command,
                sample_count=len(kappa_values),
                end_speed_mps=trace[-1].measured_linear_speed_mps,
                apparent_longitudinal_tire_stiffness_n=apparent_stiffness_n,
            )
        )
    stable = False
    if positive_values and all(
        fit.apparent_longitudinal_tire_stiffness_n is not None and fit.apparent_longitudinal_tire_stiffness_n > 0.0
        for fit in fits
    ):
        stable = (max(positive_values) / min(positive_values)) <= LONGITUDINAL_STIFFNESS_STABILITY_RATIO_LIMIT
    return (
        statistics.median(positive_values) if positive_values else None,
        stable,
        fits,
    )


def launch_motion_threshold_command_range(
    fits: list[LongitudinalStiffnessCommandFit],
) -> tuple[float | None, float | None]:
    below_threshold = [
        fit.abs_command
        for fit in fits
        if fit.end_speed_mps < MOTION_THRESHOLD_END_SPEED_MPS
    ]
    above_threshold = [
        fit.abs_command
        for fit in fits
        if fit.end_speed_mps >= MOTION_THRESHOLD_END_SPEED_MPS
    ]
    return (
        max(below_threshold) if below_threshold else None,
        min(above_threshold) if above_threshold else None,
    )


def summarize_tire_plant_fit(
    launch_rows_by_repeat: dict[int, list[dict[str, str]]],
    gyro_bias_radps: float,
    accel_bias_y_mps2: float,
    control_log_path: Path | None,
    available_section_ids: set[int],
) -> TirePlantFitSummary | None:
    params = load_launch_fit_parameters(control_log_path)
    if params is None:
        return None
    traces = build_launch_mean_traces(launch_rows_by_repeat)
    if not traces:
        return None
    apparent_inertia_kg_m2, inertia_bin_count = fit_apparent_equivalent_wheel_inertia(traces, params)
    apparent_rolling_friction_torque_nm, apparent_viscous_friction_nm_per_radps, drag_row_count, drag_sigma_nm = (
        fit_apparent_drive_drag(traces, params)
    )
    apparent_longitudinal_tire_stiffness_positive_bin_median_n, longitudinal_stiffness_stable, stiffness_fits = (
        fit_apparent_longitudinal_stiffness(traces, gyro_bias_radps, accel_bias_y_mps2, params)
    )
    lower_command, upper_command = launch_motion_threshold_command_range(stiffness_fits)
    lateral_sections_present = any(section_id in {4, 5, 6, 7} for section_id in available_section_ids)
    lateral_identifiability_reason = None
    if not lateral_sections_present:
        lateral_identifiability_reason = (
            "current card has no completed SEC_40_YAW, SEC_50_SMOOTH, SEC_60_LOOP_CW, or SEC_70_LOOP_CCW sections"
        )
    return TirePlantFitSummary(
        run_id=load_run_id(control_log_path),
        launch_command_bin_count=len(traces),
        launch_motion_threshold_lower_command=lower_command,
        launch_motion_threshold_upper_command=upper_command,
        apparent_equivalent_wheel_inertia_kg_m2=apparent_inertia_kg_m2,
        apparent_equivalent_wheel_inertia_bin_count=inertia_bin_count,
        apparent_equivalent_wheel_inertia_sample_index=EARLY_INERTIA_SAMPLE_INDEX,
        apparent_rolling_friction_torque_nm=apparent_rolling_friction_torque_nm,
        apparent_viscous_friction_nm_per_radps=apparent_viscous_friction_nm_per_radps,
        apparent_drag_fit_row_count=drag_row_count,
        apparent_drag_fit_residual_sigma_nm=drag_sigma_nm,
        apparent_longitudinal_tire_stiffness_positive_bin_median_n=apparent_longitudinal_tire_stiffness_positive_bin_median_n,
        apparent_longitudinal_tire_stiffness_stable=longitudinal_stiffness_stable,
        apparent_longitudinal_tire_stiffness_fits=stiffness_fits,
        can_identify_cornering_stiffness=lateral_sections_present,
        can_identify_lateral_damping=lateral_sections_present,
        can_identify_peak_friction=lateral_sections_present,
        lateral_identifiability_reason=lateral_identifiability_reason,
    )
