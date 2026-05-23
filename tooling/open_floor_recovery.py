from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from pathlib import Path


SECTION_NAMES = {
    0: "SEC_00_TIMING",
    1: "SEC_10_STATIC",
    2: "SEC_20_LAUNCH",
    3: "SEC_30_STRAIGHT",
    4: "SEC_40_YAW",
    5: "SEC_50_SMOOTH",
    6: "SEC_60_LOOP_CW",
    7: "SEC_70_LOOP_CCW",
}

MARKER_NAMES = {
    0: "C",
    1: "N",
    2: "S",
    3: "CW",
    4: "CCW",
}

IMU_POSITION_BODY_X_M = -0.023
IMU_POSITION_BODY_Y_M = -0.011
DEFAULT_CONTROL_LOG_NAME = "logging.txt"
RECOVERY_PRIMITIVE_ID = 15
START_GYRO_RADPS = 0.6
START_ENCODER_DIFF_SPEED_MPS = 0.05
START_ACCEL_MPS2 = 0.5
END_GYRO_RADPS = 0.25
END_ENCODER_DIFF_SPEED_MPS = 0.025
END_ROTATION_SIGNATURE_MPS2 = 0.3
END_ACCEL_MPS2 = 0.45
SETTLE_SAMPLES = 30
MIN_TURN_ANGLE_RAD = 0.5
TRACK_WIDTH_SAMPLE_MIN_ABS_GYRO_RADPS = END_GYRO_RADPS
INERTIA_MIN_ACCEL_RADPS2 = 100.0
INERTIA_MIN_GYRO_RADPS = 1.0
INERTIA_MIN_DRIVE_COMMAND = 0.85


@dataclass
class DistributionSummary:
    count: int
    l5: float
    l10: float
    l25: float
    l50: float
    l75: float
    l90: float
    l95: float
    mean: float
    sigma: float


@dataclass
class RecoveryTurnSummary:
    section_id: int
    section_name: str
    repeat_index: int
    start_marker_name: str
    row_count: int
    duration_seconds: float
    angle_rad: float
    angle_deg: float
    differential_distance_m: float
    effective_track_width_m: float
    peak_abs_gyro_radps: float
    peak_abs_encoder_diff_speed_mps: float
    peak_abs_rotation_signature_mps2: float
    median_rotation_alignment: float
    saturation_flags: int
    watchdog_flags: int
    encoder_angle_at_logged_track_deg: float | None
    encoder_gyro_angle_ratio_at_logged_track: float | None
    likely_longitudinal_slip: bool
    sample_effective_track_width_stats: DistributionSummary | None
    apparent_yaw_inertia_torque_only_upper_bound_kg_m2: float | None


@dataclass
class RecoveryAggregateSummary:
    turn_count: int
    valid_turn_count: int
    likely_slip_turn_count: int
    mean_abs_angle_deg: float
    median_abs_angle_deg: float
    mean_effective_track_width_m: float
    median_effective_track_width_m: float
    sample_effective_track_width_stats: DistributionSummary | None
    mean_peak_abs_gyro_radps: float
    median_rotation_alignment: float
    apparent_yaw_inertia_torque_only_upper_bound_kg_m2: float | None


@dataclass
class RunPlantParameters:
    battery_voltage_v: float
    drive_resistance_ohms: float
    torque_constant_nm_per_a: float
    speed_constant_radps_per_volt: float
    no_load_current_a: float
    gear_ratio: float
    wheel_radius_m: float
    nominal_track_width_m: float


def parse_float(text: str) -> float:
    return float(text)


def parse_int(text: str) -> int:
    return int(text)


def row_dt_seconds(row: dict[str, str]) -> float:
    return 1.0e-6 * parse_int(row["dt_us"])


def row_watchdog_flags(row: dict[str, str]) -> int:
    return parse_int(row.get("watchdog_flags", "0"))


def encoder_diff_speed_mps(row: dict[str, str]) -> float:
    return parse_float(row["right_encoder_velocity_mps"]) - parse_float(row["left_encoder_velocity_mps"])


def independent_gyro_radps(row: dict[str, str], gyro_bias_radps: float) -> float:
    return parse_float(row["gyro_raw_radps"]) - gyro_bias_radps


def section_start_marker_name(section_id: int) -> str:
    if section_id == 6:
        return MARKER_NAMES[3]
    if section_id == 7:
        return MARKER_NAMES[4]
    return MARKER_NAMES[0]


def interpolated_percentile(sorted_values: list[float], fraction: float) -> float:
    if not sorted_values:
        raise ValueError("percentile requires at least one value")
    if len(sorted_values) == 1:
        return sorted_values[0]
    index = max(0.0, min(fraction, 1.0)) * (len(sorted_values) - 1)
    low_index = math.floor(index)
    high_index = math.ceil(index)
    if low_index == high_index:
        return sorted_values[low_index]
    high_fraction = index - low_index
    low_fraction = 1.0 - high_fraction
    return (
        sorted_values[low_index] * low_fraction +
        sorted_values[high_index] * high_fraction
    )


def summarize_distribution(values: list[float]) -> DistributionSummary | None:
    if not values:
        return None
    sorted_values = sorted(values)
    return DistributionSummary(
        count=len(sorted_values),
        l5=interpolated_percentile(sorted_values, 0.05),
        l10=interpolated_percentile(sorted_values, 0.10),
        l25=interpolated_percentile(sorted_values, 0.25),
        l50=interpolated_percentile(sorted_values, 0.50),
        l75=interpolated_percentile(sorted_values, 0.75),
        l90=interpolated_percentile(sorted_values, 0.90),
        l95=interpolated_percentile(sorted_values, 0.95),
        mean=statistics.fmean(sorted_values),
        sigma=statistics.pstdev(sorted_values),
    )


def corrected_accel_body(
    row: dict[str, str],
    accel_bias_x_mps2: float,
    accel_bias_y_mps2: float,
) -> tuple[float, float]:
    return (
        parse_float(row["accel_body_x_mps2"]) - accel_bias_x_mps2,
        parse_float(row["accel_body_y_mps2"]) - accel_bias_y_mps2,
    )


def rotation_signature(
    segment: list[dict[str, str]],
    index: int,
    gyro_bias_radps: float,
    accel_bias_x_mps2: float,
    accel_bias_y_mps2: float,
) -> tuple[float, float, float]:
    row = segment[index]
    prev_row = segment[index - 1]
    next_row = segment[index + 1]
    dt_seconds = max(row_dt_seconds(prev_row) + row_dt_seconds(row), 1.0e-6)
    gyro_radps = independent_gyro_radps(row, gyro_bias_radps)
    gyro_alpha_radps2 = (
        independent_gyro_radps(next_row, gyro_bias_radps) -
        independent_gyro_radps(prev_row, gyro_bias_radps)
    ) / dt_seconds
    predicted_x_mps2 = (
        -(gyro_radps * gyro_radps * IMU_POSITION_BODY_X_M) +
        (gyro_alpha_radps2 * IMU_POSITION_BODY_Y_M)
    )
    predicted_y_mps2 = (
        -(gyro_radps * gyro_radps * IMU_POSITION_BODY_Y_M) -
        (gyro_alpha_radps2 * IMU_POSITION_BODY_X_M)
    )
    measured_x_mps2, measured_y_mps2 = corrected_accel_body(
        row,
        accel_bias_x_mps2,
        accel_bias_y_mps2,
    )
    predicted_magnitude = math.hypot(predicted_x_mps2, predicted_y_mps2)
    measured_magnitude = math.hypot(measured_x_mps2, measured_y_mps2)
    alignment = 0.0
    if predicted_magnitude > 1.0e-6 and measured_magnitude > 1.0e-6:
        alignment = (
            (predicted_x_mps2 * measured_x_mps2) +
            (predicted_y_mps2 * measured_y_mps2)
        ) / (predicted_magnitude * measured_magnitude)
    return predicted_magnitude, measured_magnitude, alignment


def find_recovery_turn_window(
    segment: list[dict[str, str]],
    gyro_bias_radps: float,
    accel_bias_x_mps2: float,
    accel_bias_y_mps2: float,
) -> tuple[int, int] | None:
    if len(segment) < 3:
        return None
    started = False
    start_index = 0
    settle_count = 0
    end_index: int | None = None
    for index in range(1, len(segment) - 1):
        row = segment[index]
        gyro_abs_radps = abs(independent_gyro_radps(row, gyro_bias_radps))
        encoder_diff_abs_mps = abs(encoder_diff_speed_mps(row))
        predicted_magnitude, measured_magnitude, _ = rotation_signature(
            segment,
            index,
            gyro_bias_radps,
            accel_bias_x_mps2,
            accel_bias_y_mps2,
        )
        if not started:
            if (
                gyro_abs_radps >= START_GYRO_RADPS and
                encoder_diff_abs_mps >= START_ENCODER_DIFF_SPEED_MPS and
                measured_magnitude >= START_ACCEL_MPS2
            ):
                started = True
                start_index = index
            continue
        quiet = (
            gyro_abs_radps <= END_GYRO_RADPS and
            encoder_diff_abs_mps <= END_ENCODER_DIFF_SPEED_MPS and
            predicted_magnitude <= END_ROTATION_SIGNATURE_MPS2 and
            measured_magnitude <= END_ACCEL_MPS2
        )
        if quiet:
            settle_count += 1
            if settle_count >= SETTLE_SAMPLES:
                end_index = index - SETTLE_SAMPLES + 1
                break
        else:
            settle_count = 0
    if not started:
        return None
    if end_index is None:
        end_index = len(segment) - 2
    if end_index <= start_index:
        return None
    return start_index, end_index


def parse_key_value_fields(text: str) -> dict[str, float]:
    result: dict[str, float] = {}
    for field in text.split(";"):
        field = field.strip()
        if not field or "=" not in field:
            continue
        key, value = field.split("=", 1)
        try:
            result[key.strip()] = float(value.strip())
        except ValueError:
            continue
    return result


def load_control_log_parameters(path: Path) -> RunPlantParameters | None:
    if not path.is_file():
        return None
    geometry_fields: dict[str, float] | None = None
    drive_fields: dict[str, float] | None = None
    with path.open(encoding="utf-8", errors="replace") as control_log:
        for line in control_log:
            if "plant_dump_params_mass_geometry:" in line:
                geometry_fields = parse_key_value_fields(
                    line.split("plant_dump_params_mass_geometry:", 1)[1].strip()
                )
            elif "plant_dump_params_drive_electrical:" in line:
                drive_fields = parse_key_value_fields(
                    line.split("plant_dump_params_drive_electrical:", 1)[1].strip()
                )
    if geometry_fields is None or drive_fields is None:
        return None
    if "wheel_radius_m" not in geometry_fields or "track_width_m" not in geometry_fields:
        return None
    required_drive = {
        "supply_voltage_v",
        "drive_resistance_ohms",
        "torque_constant_nm_per_a",
        "speed_constant_radps_per_volt",
        "no_load_current_a",
        "gear_ratio",
    }
    if not required_drive.issubset(drive_fields):
        return None
    return RunPlantParameters(
        battery_voltage_v=drive_fields["supply_voltage_v"],
        drive_resistance_ohms=drive_fields["drive_resistance_ohms"],
        torque_constant_nm_per_a=drive_fields["torque_constant_nm_per_a"],
        speed_constant_radps_per_volt=drive_fields["speed_constant_radps_per_volt"],
        no_load_current_a=drive_fields["no_load_current_a"],
        gear_ratio=drive_fields["gear_ratio"],
        wheel_radius_m=geometry_fields["wheel_radius_m"],
        nominal_track_width_m=geometry_fields["track_width_m"],
    )


def drive_torque_from_command(motor_command: float, wheel_bank_speed_radps: float, params: RunPlantParameters) -> float:
    command = max(-1.0, min(1.0, motor_command))
    motor_speed_radps = wheel_bank_speed_radps * params.gear_ratio
    applied_voltage_v = command * params.battery_voltage_v
    back_emf_voltage_v = motor_speed_radps / params.speed_constant_radps_per_volt
    armature_current_a = (applied_voltage_v - back_emf_voltage_v) / params.drive_resistance_ohms
    motor_current_limit_a = params.battery_voltage_v / params.drive_resistance_ohms
    armature_current_a = max(-motor_current_limit_a, min(motor_current_limit_a, armature_current_a))
    if armature_current_a > 1.0e-6:
        no_load_direction = 1.0
    elif armature_current_a < -1.0e-6:
        no_load_direction = -1.0
    elif motor_speed_radps > 1.0e-6:
        no_load_direction = 1.0
    elif motor_speed_radps < -1.0e-6:
        no_load_direction = -1.0
    else:
        no_load_direction = 0.0
    load_current_a = armature_current_a - (no_load_direction * params.no_load_current_a)
    if no_load_direction > 0.0 and load_current_a < 0.0:
        load_current_a = 0.0
    elif no_load_direction < 0.0 and load_current_a > 0.0:
        load_current_a = 0.0
    return params.torque_constant_nm_per_a * load_current_a * params.gear_ratio


def per_sample_effective_track_width_m(row: dict[str, str], gyro_bias_radps: float) -> float | None:
    gyro_radps = independent_gyro_radps(row, gyro_bias_radps)
    if abs(gyro_radps) < TRACK_WIDTH_SAMPLE_MIN_ABS_GYRO_RADPS:
        return None
    return abs(encoder_diff_speed_mps(row) / gyro_radps)


def estimate_torque_only_yaw_inertia_upper_bound(
    window_rows: list[dict[str, str]],
    gyro_bias_radps: float,
    effective_track_width_m: float,
    params: RunPlantParameters | None,
) -> float | None:
    if params is None or len(window_rows) < 3 or effective_track_width_m <= 0.0 or params.wheel_radius_m <= 0.0:
        return None
    net_angle_rad = sum(independent_gyro_radps(row, gyro_bias_radps) * row_dt_seconds(row) for row in window_rows)
    if abs(net_angle_rad) < MIN_TURN_ANGLE_RAD:
        return None
    turn_sign = 1.0 if net_angle_rad >= 0.0 else -1.0
    values: list[float] = []
    for index in range(1, len(window_rows) - 1):
        row = window_rows[index]
        prev_row = window_rows[index - 1]
        next_row = window_rows[index + 1]
        dt_seconds = max(row_dt_seconds(prev_row) + row_dt_seconds(row), 1.0e-6)
        gyro_radps = independent_gyro_radps(row, gyro_bias_radps)
        gyro_alpha_radps2 = (
            independent_gyro_radps(next_row, gyro_bias_radps) -
            independent_gyro_radps(prev_row, gyro_bias_radps)
        ) / dt_seconds
        if turn_sign * gyro_alpha_radps2 < INERTIA_MIN_ACCEL_RADPS2 or abs(gyro_radps) < INERTIA_MIN_GYRO_RADPS:
            continue
        left_command = parse_float(row["left_drive_command"])
        right_command = parse_float(row["right_drive_command"])
        if abs(left_command) < INERTIA_MIN_DRIVE_COMMAND or abs(right_command) < INERTIA_MIN_DRIVE_COMMAND:
            continue
        if left_command * right_command > -0.25:
            continue
        left_torque_nm = drive_torque_from_command(left_command, parse_float(row["left_encoder_wheel_speed_radps"]), params)
        right_torque_nm = drive_torque_from_command(right_command, parse_float(row["right_encoder_wheel_speed_radps"]), params)
        differential_force_upper_bound_n = abs((left_torque_nm - right_torque_nm) / params.wheel_radius_m)
        yaw_moment_upper_bound_nm = 0.5 * effective_track_width_m * differential_force_upper_bound_n
        if yaw_moment_upper_bound_nm <= 0.0:
            continue
        inertia_kg_m2 = yaw_moment_upper_bound_nm / abs(gyro_alpha_radps2)
        if 0.0 < inertia_kg_m2 < 0.01:
            values.append(inertia_kg_m2)
    return statistics.median(values) if len(values) >= 3 else None


def summarize_recovery_segments(
    recovery_segments: list[list[dict[str, str]]],
    gyro_bias_radps: float,
    accel_bias_x_mps2: float,
    accel_bias_y_mps2: float,
    control_log_path: Path | None,
) -> tuple[list[RecoveryTurnSummary], RecoveryAggregateSummary | None]:
    params = (
        load_control_log_parameters(control_log_path)
        if control_log_path is not None
        else None
    )
    summaries: list[RecoveryTurnSummary] = []
    track_width_samples_by_watchdog: list[tuple[int, list[float]]] = []
    for segment in recovery_segments:
        window_indices = find_recovery_turn_window(
            segment,
            gyro_bias_radps,
            accel_bias_x_mps2,
            accel_bias_y_mps2,
        )
        if window_indices is None:
            continue
        window_rows = segment[window_indices[0] : window_indices[1] + 1]
        angle_rad = sum(independent_gyro_radps(row, gyro_bias_radps) * row_dt_seconds(row) for row in window_rows)
        if abs(angle_rad) < MIN_TURN_ANGLE_RAD:
            continue
        left_distance_change_m = parse_float(window_rows[-1]["left_encoder_distance_m"]) - parse_float(window_rows[0]["left_encoder_distance_m"])
        right_distance_change_m = parse_float(window_rows[-1]["right_encoder_distance_m"]) - parse_float(window_rows[0]["right_encoder_distance_m"])
        differential_distance_m = right_distance_change_m - left_distance_change_m
        effective_track_width_m = abs(differential_distance_m / angle_rad)
        sample_track_widths_m = [
            sample_width_m
            for row in window_rows
            for sample_width_m in [per_sample_effective_track_width_m(row, gyro_bias_radps)]
            if sample_width_m is not None
        ]
        sample_effective_track_width_stats = summarize_distribution(sample_track_widths_m)
        encoder_angle_at_logged_track_deg: float | None = None
        encoder_gyro_angle_ratio_at_logged_track: float | None = None
        alignments: list[float] = []
        predicted_magnitudes: list[float] = []
        peak_abs_gyro_radps = 0.0
        peak_abs_encoder_diff_speed_mps = 0.0
        saturation_flags = 0
        watchdog_flags = 0
        for row in segment:
            saturation_flags |= parse_int(row["saturation_flags"])
            watchdog_flags |= row_watchdog_flags(row)
        if params is not None and params.nominal_track_width_m > 0.0:
            encoder_angle_at_logged_track_rad = differential_distance_m / params.nominal_track_width_m
            encoder_angle_at_logged_track_deg = math.degrees(encoder_angle_at_logged_track_rad)
            encoder_gyro_angle_ratio_at_logged_track = (
                encoder_angle_at_logged_track_rad / angle_rad
                if abs(angle_rad) > 1.0e-9
                else None
            )
        for index in range(window_indices[0], window_indices[1] + 1):
            row = segment[index]
            peak_abs_gyro_radps = max(peak_abs_gyro_radps, abs(independent_gyro_radps(row, gyro_bias_radps)))
            peak_abs_encoder_diff_speed_mps = max(peak_abs_encoder_diff_speed_mps, abs(encoder_diff_speed_mps(row)))
            if 0 < index < len(segment) - 1:
                predicted_magnitude, _, alignment = rotation_signature(
                    segment,
                    index,
                    gyro_bias_radps,
                    accel_bias_x_mps2,
                    accel_bias_y_mps2,
                )
                predicted_magnitudes.append(predicted_magnitude)
                alignments.append(alignment)
        likely_longitudinal_slip = (
            saturation_flags != 0 or
            (
                encoder_gyro_angle_ratio_at_logged_track is not None and
                abs(abs(encoder_gyro_angle_ratio_at_logged_track) - 1.0) > 0.10
            )
        )
        track_width_samples_by_watchdog.append((watchdog_flags, sample_track_widths_m))
        summaries.append(
            RecoveryTurnSummary(
                section_id=parse_int(window_rows[0]["section_id"]),
                section_name=SECTION_NAMES.get(parse_int(window_rows[0]["section_id"]), "UNKNOWN"),
                repeat_index=parse_int(window_rows[0]["repeat_index"]),
                start_marker_name=section_start_marker_name(parse_int(window_rows[0]["section_id"])),
                row_count=len(window_rows),
                duration_seconds=sum(row_dt_seconds(row) for row in window_rows),
                angle_rad=angle_rad,
                angle_deg=math.degrees(angle_rad),
                differential_distance_m=differential_distance_m,
                effective_track_width_m=effective_track_width_m,
                peak_abs_gyro_radps=peak_abs_gyro_radps,
                peak_abs_encoder_diff_speed_mps=peak_abs_encoder_diff_speed_mps,
                peak_abs_rotation_signature_mps2=max(predicted_magnitudes, default=0.0),
                median_rotation_alignment=statistics.median(alignments) if alignments else 0.0,
                saturation_flags=saturation_flags,
                watchdog_flags=watchdog_flags,
                encoder_angle_at_logged_track_deg=encoder_angle_at_logged_track_deg,
                encoder_gyro_angle_ratio_at_logged_track=encoder_gyro_angle_ratio_at_logged_track,
                likely_longitudinal_slip=likely_longitudinal_slip,
                sample_effective_track_width_stats=sample_effective_track_width_stats,
                apparent_yaw_inertia_torque_only_upper_bound_kg_m2=estimate_torque_only_yaw_inertia_upper_bound(
                    window_rows,
                    gyro_bias_radps,
                    effective_track_width_m,
                    params,
                ),
            )
        )
    if not summaries:
        return [], None
    valid_summaries = [summary for summary in summaries if summary.watchdog_flags == 0]
    if not valid_summaries:
        valid_summaries = summaries
    if any(watchdog_flags == 0 for watchdog_flags, _ in track_width_samples_by_watchdog):
        aggregate_track_width_samples = [
            sample_track_width_m
            for watchdog_flags, sample_track_widths_m in track_width_samples_by_watchdog
            if watchdog_flags == 0
            for sample_track_width_m in sample_track_widths_m
        ]
    else:
        aggregate_track_width_samples = [
            sample_track_width_m
            for _, sample_track_widths_m in track_width_samples_by_watchdog
            for sample_track_width_m in sample_track_widths_m
        ]
    likely_slip_turn_count = sum(1 for summary in summaries if summary.likely_longitudinal_slip)
    inertia_values = [
        summary.apparent_yaw_inertia_torque_only_upper_bound_kg_m2
        for summary in valid_summaries
        if summary.apparent_yaw_inertia_torque_only_upper_bound_kg_m2 is not None
    ]
    aggregate = RecoveryAggregateSummary(
        turn_count=len(summaries),
        valid_turn_count=len(valid_summaries),
        likely_slip_turn_count=likely_slip_turn_count,
        mean_abs_angle_deg=statistics.fmean(abs(summary.angle_deg) for summary in valid_summaries),
        median_abs_angle_deg=statistics.median(abs(summary.angle_deg) for summary in valid_summaries),
        mean_effective_track_width_m=statistics.fmean(summary.effective_track_width_m for summary in valid_summaries),
        median_effective_track_width_m=statistics.median(summary.effective_track_width_m for summary in valid_summaries),
        sample_effective_track_width_stats=summarize_distribution(aggregate_track_width_samples),
        mean_peak_abs_gyro_radps=statistics.fmean(summary.peak_abs_gyro_radps for summary in valid_summaries),
        median_rotation_alignment=statistics.median(summary.median_rotation_alignment for summary in valid_summaries),
        apparent_yaw_inertia_torque_only_upper_bound_kg_m2=(
            statistics.median(inertia_values) if inertia_values else None
        ),
    )
    return summaries, aggregate
