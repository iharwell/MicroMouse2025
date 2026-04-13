#!/usr/bin/env python3
"""Summarize open-floor diagnostic logs for estimator and plant tuning.

The script intentionally uses only the Python standard library so the whole
team can run it without extra environment setup.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import DefaultDict
from typing import Iterable

from open_floor_plant_fit import TirePlantFitSummary
from open_floor_plant_fit import FeedforwardAlignmentSummary
from open_floor_plant_fit import summarize_feedforward_alignment
from open_floor_plant_fit import summarize_tire_plant_fit
from open_floor_launch_floor import LaunchFloorSummary
from open_floor_launch_floor import summarize_launch_floor
from open_floor_recovery import DEFAULT_CONTROL_LOG_NAME
from open_floor_recovery import DistributionSummary
from open_floor_recovery import IMU_POSITION_BODY_X_M
from open_floor_recovery import IMU_POSITION_BODY_Y_M
from open_floor_recovery import RecoveryAggregateSummary
from open_floor_recovery import RecoveryTurnSummary
from open_floor_recovery import RECOVERY_PRIMITIVE_ID
from open_floor_recovery import TRACK_WIDTH_SAMPLE_MIN_ABS_GYRO_RADPS
from open_floor_recovery import summarize_recovery_segments


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

PRIMITIVE_NAMES = {
    0: "NONE",
    1: "TIMING_NO_MOTION",
    2: "STATIC_HOLD",
    3: "OPEN_LOOP_LAUNCH",
    4: "STR2",
    5: "STR4",
    6: "IP90",
    7: "IP90_M",
    8: "IP180",
    9: "S45SS",
    10: "S45SS_M",
    11: "S90SS",
    12: "S90SS_M",
    13: "S135SS",
    14: "S135SS_M",
    15: "RECOVERY",
}

DIRECTION_NAMES = {
    0: "NONE",
    1: "POSITIVE",
    2: "NEGATIVE",
    3: "NORTHBOUND",
    4: "SOUTHBOUND",
    5: "CLOCKWISE",
    6: "COUNTERCLOCKWISE",
    7: "FLIP",
    8: "LEFT",
    9: "RIGHT",
}

MARKER_NAMES = {
    0: "C",
    1: "N",
    2: "S",
    3: "CW",
    4: "CCW",
}

STATIC_SECTION_ID = 1
STATIC_PRIMITIVE_ID = 2
LAUNCH_SECTION_ID = 2
LAUNCH_PRIMITIVE_ID = 3


@dataclass
class ScalarStats:
    mean: float
    sigma: float
    minimum: float
    maximum: float


@dataclass
class StationarySummary:
    row_count: int
    duration_seconds: float
    gyro_raw: ScalarStats
    gyro_logged_bias: ScalarStats
    gyro_logged_corrected: ScalarStats
    gyro_independent_bias_radps: float
    gyro_independent_corrected: ScalarStats
    accel_body_x: ScalarStats
    accel_body_y: ScalarStats
    planar_accel: ScalarStats


@dataclass
class LaunchMagnitudeSummary:
    abs_command: float
    repeat_count: int
    active_row_count: int
    nonzero_encoder_fraction: float
    peak_abs_linear_speed_mps: float
    peak_abs_wheel_omega_radps: float
    peak_abs_gyro_radps: float
    peak_abs_accel_mps2: float
    max_pose_drift_mm: float


@dataclass
class RepeatabilitySummary:
    linear_sigma_mps: float
    yaw_sigma_radps: float


@dataclass
class TimingSummary:
    row_count: int
    dt_mean_us: float
    dt_p95_us: float
    dt_max_us: int
    predict_mean_us: float
    predict_p95_us: float
    predict_max_us: int
    update_mean_us: float
    update_p95_us: float
    update_max_us: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize open-floor CSV logs for estimator tuning and repeatability analysis."
    )
    parser.add_argument(
        "--main",
        required=True,
        type=Path,
        help="Path to open_floor_main.csv",
    )
    parser.add_argument(
        "--timing",
        type=Path,
        help="Optional path to open_floor_timing.csv",
    )
    parser.add_argument(
        "--control-log",
        type=Path,
        help="Optional path to logging.txt so recovery-turn fits use the run's motor constants.",
    )
    return parser.parse_args()


def scalar_stats(values: Iterable[float]) -> ScalarStats:
    values = list(values)
    if not values:
        return ScalarStats(mean=0.0, sigma=0.0, minimum=0.0, maximum=0.0)
    return ScalarStats(
        mean=statistics.fmean(values),
        sigma=statistics.pstdev(values),
        minimum=min(values),
        maximum=max(values),
    )


def percentile(sorted_values: list[float], fraction: float) -> float:
    if not sorted_values:
        return 0.0
    index = min(int(fraction * len(sorted_values)), len(sorted_values) - 1)
    return sorted_values[index]


def analyze_main_csv(
    path: Path,
    control_log_path: Path | None,
) -> tuple[
    StationarySummary,
    list[LaunchMagnitudeSummary],
    LaunchFloorSummary | None,
    RepeatabilitySummary,
    dict[str, float],
    TirePlantFitSummary | None,
    FeedforwardAlignmentSummary | None,
    list[RecoveryTurnSummary],
    RecoveryAggregateSummary | None,
]:
    static_dt_seconds = 0.0
    static_gyro_raw: list[float] = []
    static_gyro_logged_bias: list[float] = []
    static_gyro_logged_corrected: list[float] = []
    static_accel_body_x: list[float] = []
    static_accel_body_y: list[float] = []
    static_planar_accel: list[float] = []

    launch_active_sample_index_by_repeat: dict[int, int] = {}
    launch_rows_by_repeat: DefaultDict[int, list[dict[str, str]]] = defaultdict(list)
    launch_series_by_command_and_index: DefaultDict[tuple[float, int], list[tuple[float, float]]] = defaultdict(list)
    launch_by_command: DefaultDict[float, dict[str, object]] = defaultdict(
        lambda: {
            "repeat_ids": set(),
            "row_count": 0,
            "nonzero_encoder_rows": 0,
            "peak_abs_linear_speed_mps": 0.0,
            "peak_abs_wheel_omega_radps": 0.0,
            "peak_abs_gyro_radps": 0.0,
            "peak_abs_accel_mps2": 0.0,
            "max_pose_drift_mm": 0.0,
        }
    )

    recovery_segments: list[list[dict[str, str]]] = []
    current_recovery_segment: list[dict[str, str]] = []
    current_recovery_key: tuple[int, int, int] | None = None
    available_section_ids: set[int] = set()

    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            section_id = int(row["section_id"])
            primitive_id = int(row["primitive_id"])
            available_section_ids.add(section_id)

            if primitive_id == RECOVERY_PRIMITIVE_ID:
                recovery_key = (
                    section_id,
                    int(row["repeat_index"]),
                    int(row["start_marker_id"]),
                )
                if current_recovery_segment and recovery_key != current_recovery_key:
                    recovery_segments.append(current_recovery_segment)
                    current_recovery_segment = []
                current_recovery_segment.append(dict(row))
                current_recovery_key = recovery_key
            elif current_recovery_segment:
                recovery_segments.append(current_recovery_segment)
                current_recovery_segment = []
                current_recovery_key = None

            if section_id == STATIC_SECTION_ID and primitive_id == STATIC_PRIMITIVE_ID:
                static_dt_seconds += 1.0e-6 * int(row["dt_us"])
                static_gyro_raw.append(float(row["gyro_raw_radps"]))
                static_gyro_logged_bias.append(float(row["gyro_bias_radps"]))
                static_gyro_logged_corrected.append(float(row["gyro_radps"]))
                static_accel_body_x.append(float(row["accel_body_x_mps2"]))
                static_accel_body_y.append(float(row["accel_body_y_mps2"]))
                static_planar_accel.append(float(row["planar_accel_mps2"]))

            if section_id != LAUNCH_SECTION_ID or primitive_id != LAUNCH_PRIMITIVE_ID:
                continue

            signed_command = float(row["left_drive_command"])
            abs_command = round(abs(signed_command), 2)
            if abs_command <= 0.0:
                continue

            repeat_index = int(row["repeat_index"])
            direction_id = int(row["direction_id"])
            sign = 1.0 if direction_id == 1 else -1.0 if direction_id == 2 else 1.0
            sample_index = launch_active_sample_index_by_repeat.get(repeat_index, 0)
            launch_active_sample_index_by_repeat[repeat_index] = sample_index + 1
            launch_rows_by_repeat[repeat_index].append(dict(row))

            normalized_linear_speed = sign * float(row["measured_linear_speed_mps"])
            normalized_yaw_rate = sign * float(row["measured_angular_speed_radps"])
            launch_series_by_command_and_index[(abs_command, sample_index)].append(
                (normalized_linear_speed, normalized_yaw_rate)
            )

            bucket = launch_by_command[abs_command]
            repeat_ids = bucket["repeat_ids"]
            assert isinstance(repeat_ids, set)
            repeat_ids.add(repeat_index)
            bucket["row_count"] = int(bucket["row_count"]) + 1

            left_omega = float(row["left_encoder_omega_radps"])
            right_omega = float(row["right_encoder_omega_radps"])
            peak_abs_wheel_omega = max(abs(left_omega), abs(right_omega))
            if peak_abs_wheel_omega > 0.0:
                bucket["nonzero_encoder_rows"] = int(bucket["nonzero_encoder_rows"]) + 1

            bucket["peak_abs_linear_speed_mps"] = max(
                float(bucket["peak_abs_linear_speed_mps"]),
                abs(float(row["measured_linear_speed_mps"])),
            )
            bucket["peak_abs_wheel_omega_radps"] = max(
                float(bucket["peak_abs_wheel_omega_radps"]),
                peak_abs_wheel_omega,
            )
            bucket["peak_abs_gyro_radps"] = max(
                float(bucket["peak_abs_gyro_radps"]),
                abs(float(row["gyro_raw_radps"])),
            )
            bucket["peak_abs_accel_mps2"] = max(
                float(bucket["peak_abs_accel_mps2"]),
                abs(float(row["accel_body_x_mps2"])),
                abs(float(row["accel_body_y_mps2"])),
            )

            dx_m = float(row["ukf_state_px_m"]) - 0.225
            dy_m = float(row["ukf_state_py_m"]) - 0.225
            bucket["max_pose_drift_mm"] = max(
                float(bucket["max_pose_drift_mm"]),
                1000.0 * math.hypot(dx_m, dy_m),
            )

    if current_recovery_segment:
        recovery_segments.append(current_recovery_segment)

    gyro_independent_bias_radps = statistics.fmean(static_gyro_raw) if static_gyro_raw else 0.0
    static_gyro_independent_corrected = [
        gyro_raw_radps - gyro_independent_bias_radps
        for gyro_raw_radps in static_gyro_raw
    ]
    stationary_summary = StationarySummary(
        row_count=len(static_gyro_logged_corrected),
        duration_seconds=static_dt_seconds,
        gyro_raw=scalar_stats(static_gyro_raw),
        gyro_logged_bias=scalar_stats(static_gyro_logged_bias),
        gyro_logged_corrected=scalar_stats(static_gyro_logged_corrected),
        gyro_independent_bias_radps=gyro_independent_bias_radps,
        gyro_independent_corrected=scalar_stats(static_gyro_independent_corrected),
        accel_body_x=scalar_stats(static_accel_body_x),
        accel_body_y=scalar_stats(static_accel_body_y),
        planar_accel=scalar_stats(static_planar_accel),
    )

    launch_summaries: list[LaunchMagnitudeSummary] = []
    for abs_command in sorted(launch_by_command):
        bucket = launch_by_command[abs_command]
        repeat_ids = bucket["repeat_ids"]
        assert isinstance(repeat_ids, set)
        row_count = int(bucket["row_count"])
        nonzero_encoder_rows = int(bucket["nonzero_encoder_rows"])
        launch_summaries.append(
            LaunchMagnitudeSummary(
                abs_command=abs_command,
                repeat_count=len(repeat_ids),
                active_row_count=row_count,
                nonzero_encoder_fraction=(nonzero_encoder_rows / row_count) if row_count else 0.0,
                peak_abs_linear_speed_mps=float(bucket["peak_abs_linear_speed_mps"]),
                peak_abs_wheel_omega_radps=float(bucket["peak_abs_wheel_omega_radps"]),
                peak_abs_gyro_radps=float(bucket["peak_abs_gyro_radps"]),
                peak_abs_accel_mps2=float(bucket["peak_abs_accel_mps2"]),
                max_pose_drift_mm=float(bucket["max_pose_drift_mm"]),
            )
        )

    launch_floor_summary = summarize_launch_floor(
        dict(launch_rows_by_repeat),
        stationary_summary.accel_body_y.mean,
    )

    series_means: dict[tuple[float, int], tuple[float, float]] = {}
    for key, values in launch_series_by_command_and_index.items():
        series_means[key] = (
            statistics.fmean(value[0] for value in values),
            statistics.fmean(value[1] for value in values),
        )

    linear_residuals: list[float] = []
    yaw_residuals: list[float] = []
    for key, values in launch_series_by_command_and_index.items():
        mean_linear, mean_yaw = series_means[key]
        for linear_value, yaw_value in values:
            linear_residuals.append(linear_value - mean_linear)
            yaw_residuals.append(yaw_value - mean_yaw)

    repeatability_summary = RepeatabilitySummary(
        linear_sigma_mps=statistics.pstdev(linear_residuals) if linear_residuals else 0.0,
        yaw_sigma_radps=statistics.pstdev(yaw_residuals) if yaw_residuals else 0.0,
    )

    suggestions = {
        "imu_yaw_sigma_radps": stationary_summary.gyro_independent_corrected.sigma,
        "imu_accel_sigma_mps2_conservative": max(
            stationary_summary.accel_body_x.sigma,
            stationary_summary.accel_body_y.sigma,
        ),
        "encoder_linear_sigma_mps": repeatability_summary.linear_sigma_mps,
        "encoder_yaw_sigma_radps": repeatability_summary.yaw_sigma_radps,
    }

    tire_plant_fit = summarize_tire_plant_fit(
        dict(launch_rows_by_repeat),
        stationary_summary.gyro_independent_bias_radps,
        stationary_summary.accel_body_y.mean,
        control_log_path,
        available_section_ids,
    )
    feedforward_alignment = summarize_feedforward_alignment(
        dict(launch_rows_by_repeat),
        control_log_path,
    )

    recovery_summaries, recovery_aggregate = summarize_recovery_segments(
        recovery_segments,
        stationary_summary.gyro_independent_bias_radps,
        stationary_summary.accel_body_x.mean,
        stationary_summary.accel_body_y.mean,
        control_log_path,
    )

    return (
        stationary_summary,
        launch_summaries,
        launch_floor_summary,
        repeatability_summary,
        suggestions,
        tire_plant_fit,
        feedforward_alignment,
        recovery_summaries,
        recovery_aggregate,
    )


def analyze_timing_csv(path: Path) -> TimingSummary:
    dt_values: list[int] = []
    predict_values: list[int] = []
    update_values: list[int] = []

    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            dt_values.append(int(row["dt_us"]))
            predict_values.append(int(row["ukf_predict_duration_us"]))
            update_values.append(int(row["ukf_update_duration_us"]))

    if not dt_values:
        return TimingSummary(
            row_count=0,
            dt_mean_us=0.0,
            dt_p95_us=0.0,
            dt_max_us=0,
            predict_mean_us=0.0,
            predict_p95_us=0.0,
            predict_max_us=0,
            update_mean_us=0.0,
            update_p95_us=0.0,
            update_max_us=0,
        )

    dt_sorted = sorted(dt_values)
    predict_sorted = sorted(predict_values)
    update_sorted = sorted(update_values)
    return TimingSummary(
        row_count=len(dt_values),
        dt_mean_us=statistics.fmean(dt_values),
        dt_p95_us=percentile(dt_sorted, 0.95),
        dt_max_us=max(dt_values),
        predict_mean_us=statistics.fmean(predict_values),
        predict_p95_us=percentile(predict_sorted, 0.95),
        predict_max_us=max(predict_values),
        update_mean_us=statistics.fmean(update_values),
        update_p95_us=percentile(update_sorted, 0.95),
        update_max_us=max(update_values),
    )


def print_scalar_summary(label: str, stats: ScalarStats, unit: str) -> None:
    print(
        f"{label}: mean={stats.mean:.6f} {unit}, sigma={stats.sigma:.6f} {unit}, "
        f"min={stats.minimum:.6f}, max={stats.maximum:.6f}"
    )


def print_distribution_summary(label: str, stats: DistributionSummary, unit: str) -> None:
    print(
        f"{label}: count={stats.count}, "
        f"L5={stats.l5:.6f} {unit}, L10={stats.l10:.6f} {unit}, L25={stats.l25:.6f} {unit}, "
        f"L50={stats.l50:.6f} {unit}, L75={stats.l75:.6f} {unit}, L90={stats.l90:.6f} {unit}, "
        f"L95={stats.l95:.6f} {unit}, mean={stats.mean:.6f} {unit}, sigma={stats.sigma:.6f} {unit}"
    )


def format_optional_float(value: float | None, precision: int) -> str:
    if value is None:
        return "none"
    return f"{value:.{precision}f}"


def main() -> int:
    args = parse_args()
    if not args.main.is_file():
        print(f"error: main CSV not found: {args.main}", file=sys.stderr)
        return 1
    if args.timing is not None and not args.timing.is_file():
        print(f"error: timing CSV not found: {args.timing}", file=sys.stderr)
        return 1

    control_log_path = args.control_log
    if control_log_path is None:
        inferred_control_log_path = args.main.parent / DEFAULT_CONTROL_LOG_NAME
        if inferred_control_log_path.is_file():
            control_log_path = inferred_control_log_path
    if control_log_path is not None and not control_log_path.is_file():
        print(f"error: control log not found: {control_log_path}", file=sys.stderr)
        return 1

    (
        stationary,
        launch_summaries,
        launch_floor_summary,
        repeatability,
        suggestions,
        tire_plant_fit,
        feedforward_alignment,
        recovery_summaries,
        recovery_aggregate,
    ) = analyze_main_csv(args.main, control_log_path)

    print(f"Open-floor main CSV: {args.main}")
    print()
    print("Static hold summary")
    print(
        f"rows={stationary.row_count}, duration_s={stationary.duration_seconds:.3f}, "
        f"section={SECTION_NAMES[STATIC_SECTION_ID]}, primitive={PRIMITIVE_NAMES[STATIC_PRIMITIVE_ID]}"
    )
    print_scalar_summary("gyro_raw", stationary.gyro_raw, "rad/s")
    print_scalar_summary("gyro_logged_bias", stationary.gyro_logged_bias, "rad/s")
    print_scalar_summary("gyro_logged_corrected", stationary.gyro_logged_corrected, "rad/s")
    print(f"gyro_independent_bias_radps={stationary.gyro_independent_bias_radps:.9f}")
    print_scalar_summary("gyro_independent_corrected", stationary.gyro_independent_corrected, "rad/s")
    print_scalar_summary("accel_body_x", stationary.accel_body_x, "m/s^2")
    print_scalar_summary("accel_body_y", stationary.accel_body_y, "m/s^2")
    print_scalar_summary("planar_accel", stationary.planar_accel, "m/s^2")
    print()

    print("Launch magnitude summary")
    print(
        f"section={SECTION_NAMES[LAUNCH_SECTION_ID]}, primitive={PRIMITIVE_NAMES[LAUNCH_PRIMITIVE_ID]}, "
        f"direction normalization={DIRECTION_NAMES[1]}/{DIRECTION_NAMES[2]}"
    )
    for summary in launch_summaries:
        print(
            f"abs_cmd={summary.abs_command:.2f}: repeats={summary.repeat_count}, active_rows={summary.active_row_count}, "
            f"nonzero_encoder_rows={summary.nonzero_encoder_fraction * 100.0:.1f}%, "
            f"peak_u={summary.peak_abs_linear_speed_mps:.4f} m/s, "
            f"peak_wheel_omega={summary.peak_abs_wheel_omega_radps:.4f} rad/s, "
            f"peak_gyro={summary.peak_abs_gyro_radps:.4f} rad/s, "
            f"peak_accel={summary.peak_abs_accel_mps2:.4f} m/s^2, "
            f"max_drift={summary.max_pose_drift_mm:.3f} mm"
        )
    print()

    if launch_floor_summary is not None:
        print("Launch floor summary")
        print(
            "method=LaunchPulse only; backlash candidates are repeats that never sustain encoder-derived body speed above the "
            "quantized-speed floor long enough to count as chassis motion; clear motion also requires inertial agreement"
        )
        print(
            f"speed_quantum_mps={launch_floor_summary.speed_quantum_mps:.6f}, "
            f"sustained_speed_threshold_mps={launch_floor_summary.sustained_speed_threshold_mps:.6f}, "
            f"clear_motion_min_time_s={launch_floor_summary.clear_motion_min_time_s:.3f}, "
            f"backlash_repeats={launch_floor_summary.backlash_repeat_count}"
        )
        if launch_floor_summary.backlash_net_encoder_counts_stats is not None:
            print_distribution_summary(
                "backlash_net_encoder_counts",
                launch_floor_summary.backlash_net_encoder_counts_stats,
                "counts",
            )
        if launch_floor_summary.backlash_net_pose_drift_mm_stats is not None:
            print_distribution_summary(
                "backlash_net_pose_drift_mm",
                launch_floor_summary.backlash_net_pose_drift_mm_stats,
                "mm",
            )
        if launch_floor_summary.backlash_peak_inertial_speed_mps_stats is not None:
            print_distribution_summary(
                "backlash_peak_inertial_speed_mps",
                launch_floor_summary.backlash_peak_inertial_speed_mps_stats,
                "m/s",
            )
        if launch_floor_summary.backlash_peak_inertial_displacement_mm_stats is not None:
            print_distribution_summary(
                "backlash_peak_inertial_displacement_mm",
                launch_floor_summary.backlash_peak_inertial_displacement_mm_stats,
                "mm",
            )
        print(
            f"observed_clear_breakaway_command={format_optional_float(launch_floor_summary.observed_clear_breakaway_command, 2)}, "
            f"effective_launch_floor_command={format_optional_float(launch_floor_summary.effective_launch_floor_command, 2)}, "
            f"nonmonotonic_clear_motion={int(launch_floor_summary.nonmonotonic_clear_motion)}"
        )
        for summary in launch_floor_summary.command_summaries:
            print(
                f"abs_cmd={summary.abs_command:.2f}: clear_motion={summary.clear_motion_count}/{summary.repeat_count}, "
                f"median_peak_u={summary.median_peak_abs_linear_speed_mps:.4f} m/s, "
                f"median_time_above_threshold_s={summary.median_time_above_sustained_speed_threshold_s:.4f}, "
                f"median_counts={summary.median_net_signed_encoder_counts:.1f}, "
                f"median_pose_drift={summary.median_net_signed_pose_drift_mm:.3f} mm, "
                f"median_peak_inertial_speed={summary.median_peak_signed_inertial_speed_mps:.4f} m/s, "
                f"median_peak_inertial_disp={summary.median_peak_signed_inertial_displacement_mm:.3f} mm"
            )
        print()

    print("Repeated-launch repeatability")
    print(
        f"encoder_linear_sigma={repeatability.linear_sigma_mps:.6f} m/s, "
        f"encoder_yaw_sigma={repeatability.yaw_sigma_radps:.6f} rad/s"
    )
    print()

    print("Suggested UKF measurement sigmas")
    print(f"imu_yaw_sigma_radps={suggestions['imu_yaw_sigma_radps']:.6f}")
    print(f"imu_accel_sigma_mps2_conservative={suggestions['imu_accel_sigma_mps2_conservative']:.6f}")
    print(f"encoder_linear_sigma_mps={suggestions['encoder_linear_sigma_mps']:.6f}")
    print(f"encoder_yaw_sigma_radps={suggestions['encoder_yaw_sigma_radps']:.6f}")

    if tire_plant_fit is not None:
        print()
        print("Tire plant fit summary")
        print(
            "method=current run only; launch fits use SEC_20_LAUNCH mean traces and the independently debiased stationary gyro; "
            "parameters labeled apparent absorb unmodeled drive efficiency and the lack of an external body-speed reference"
        )
        print(
            f"run_id={'unknown' if tire_plant_fit.run_id is None else tire_plant_fit.run_id}, "
            f"launch_command_bins={tire_plant_fit.launch_command_bin_count}, "
            f"motion_threshold_lower_command={format_optional_float(tire_plant_fit.launch_motion_threshold_lower_command, 2)}, "
            f"motion_threshold_upper_command={format_optional_float(tire_plant_fit.launch_motion_threshold_upper_command, 2)}"
        )
        if tire_plant_fit.apparent_equivalent_wheel_inertia_kg_m2 is not None:
            print(
                f"apparent_equivalent_wheel_inertia_kg_m2={tire_plant_fit.apparent_equivalent_wheel_inertia_kg_m2:.9f}, "
                f"inertia_bin_count={tire_plant_fit.apparent_equivalent_wheel_inertia_bin_count}, "
                f"inertia_sample_index={tire_plant_fit.apparent_equivalent_wheel_inertia_sample_index}"
            )
        if (
            tire_plant_fit.apparent_rolling_friction_torque_nm is not None and
            tire_plant_fit.apparent_viscous_friction_nm_per_radps is not None
        ):
            print(
                f"apparent_rolling_friction_torque_nm={tire_plant_fit.apparent_rolling_friction_torque_nm:.9f}, "
                f"apparent_viscous_friction_nm_per_radps={tire_plant_fit.apparent_viscous_friction_nm_per_radps:.9f}, "
                f"drag_fit_rows={tire_plant_fit.apparent_drag_fit_row_count}, "
                f"drag_fit_sigma_nm={0.0 if tire_plant_fit.apparent_drag_fit_residual_sigma_nm is None else tire_plant_fit.apparent_drag_fit_residual_sigma_nm:.9f}"
            )
        if tire_plant_fit.apparent_longitudinal_tire_stiffness_positive_bin_median_n is not None:
            print(
                f"apparent_longitudinal_tire_stiffness_positive_bin_median_n={tire_plant_fit.apparent_longitudinal_tire_stiffness_positive_bin_median_n:.9f}, "
                f"longitudinal_tire_stiffness_stable={int(tire_plant_fit.apparent_longitudinal_tire_stiffness_stable)}"
            )
            for fit in tire_plant_fit.apparent_longitudinal_tire_stiffness_fits:
                print(
                    f"longitudinal_stiffness_abs_cmd={fit.abs_command:.2f}, sample_count={fit.sample_count}, "
                    f"end_speed_mps={fit.end_speed_mps:.6f}, "
                    f"apparent_longitudinal_tire_stiffness_n={'none' if fit.apparent_longitudinal_tire_stiffness_n is None else f'{fit.apparent_longitudinal_tire_stiffness_n:.9f}'}"
                )
        print(
            f"identifiable_cornering_stiffness={int(tire_plant_fit.can_identify_cornering_stiffness)}, "
            f"identifiable_lateral_damping={int(tire_plant_fit.can_identify_lateral_damping)}, "
            f"identifiable_peak_friction={int(tire_plant_fit.can_identify_peak_friction)}"
        )
        if tire_plant_fit.lateral_identifiability_reason is not None:
            print(f"lateral_identifiability_reason={tire_plant_fit.lateral_identifiability_reason}")

    if feedforward_alignment is not None:
        print()
        print("Feedforward alignment summary")
        print(
            "method=invert the configured launch-region wheel command model on SEC_20_LAUNCH mean traces and "
            "compare the required normalized drive command against the logged launch command bin"
        )
        print(
            f"run_id={'unknown' if feedforward_alignment.run_id is None else feedforward_alignment.run_id}, "
            f"command_bins={feedforward_alignment.command_bin_count}, "
            f"samples={feedforward_alignment.sample_count}, "
            f"configured_effective_longitudinal_mass_kg={feedforward_alignment.configured_effective_longitudinal_mass_kg:.9f}, "
            f"configured_equivalent_wheel_inertia_kg_m2={feedforward_alignment.configured_equivalent_wheel_inertia_kg_m2:.9f}, "
            f"configured_rolling_friction_torque_nm={feedforward_alignment.configured_rolling_friction_torque_nm:.9f}, "
            f"configured_static_friction_torque_nm={feedforward_alignment.configured_static_friction_torque_nm:.9f}, "
            f"configured_static_friction_max_speed_mps={feedforward_alignment.configured_static_friction_max_speed_mps:.9f}, "
            f"configured_viscous_friction_nm_per_radps={feedforward_alignment.configured_viscous_friction_nm_per_radps:.9f}"
        )
        print(
            f"overall_mean_command_error={feedforward_alignment.overall_mean_command_error:+.6f}, "
            f"overall_rmse_command_error={feedforward_alignment.overall_rmse_command_error:.6f}"
        )
        for summary in feedforward_alignment.command_summaries:
            print(
                f"abs_cmd={summary.abs_command:.2f}: samples={summary.sample_count}, "
                f"required_cmd_p10={summary.required_command_p10:.4f}, "
                f"required_cmd_median={summary.required_command_median:.4f}, "
                f"required_cmd_p90={summary.required_command_p90:.4f}, "
                f"steady_required_cmd_median={format_optional_float(summary.steady_required_command_median, 4)}, "
                f"mean_command_error={summary.mean_command_error:+.4f}, "
                f"rmse_command_error={summary.rmse_command_error:.4f}"
            )

    if recovery_summaries:
        print()
        print("Recovery turn summary")
        print(
            "method=gyro integral over a recovery-turn window gated by encoder differential speed and "
            "IMU-offset rotational acceleration signature; gyro uses raw gyro minus independent stationary bias; UKF pose excluded"
        )
        print(
            f"imu_position_body_m=({IMU_POSITION_BODY_X_M:.3f},{IMU_POSITION_BODY_Y_M:.3f}), "
            f"control_log={'none' if control_log_path is None else control_log_path}, "
            f"per_sample_track_width_min_abs_gyro_radps={TRACK_WIDTH_SAMPLE_MIN_ABS_GYRO_RADPS:.3f}, "
            f"independent_gyro_bias_radps={stationary.gyro_independent_bias_radps:.9f}"
        )
        for summary in recovery_summaries:
            print(
                f"section={summary.section_name}, repeat={summary.repeat_index}, start_marker={summary.start_marker_name}, "
                f"rows={summary.row_count}, duration_s={summary.duration_seconds:.3f}, "
                f"angle_deg={summary.angle_deg:.3f}, angle_rad={summary.angle_rad:.6f}, "
                f"odometric_equivalent_track_width_m={summary.effective_track_width_m:.6f}, "
                f"delta_encoder_distance_m={summary.differential_distance_m:.6f}, "
                f"peak_gyro_radps={summary.peak_abs_gyro_radps:.3f}, "
                f"peak_encoder_diff_speed_mps={summary.peak_abs_encoder_diff_speed_mps:.3f}, "
                f"peak_rotation_signature_mps2={summary.peak_abs_rotation_signature_mps2:.3f}, "
                f"median_rotation_alignment={summary.median_rotation_alignment:.3f}, "
                f"saturation_flags={summary.saturation_flags}, watchdog_flags={summary.watchdog_flags}, "
                f"likely_longitudinal_slip={int(summary.likely_longitudinal_slip)}"
            )
            if summary.encoder_angle_at_logged_track_deg is not None:
                print(
                    f"encoder_angle_at_logged_track_deg={summary.encoder_angle_at_logged_track_deg:.3f}, "
                    f"encoder_gyro_angle_ratio_at_logged_track={summary.encoder_gyro_angle_ratio_at_logged_track:.3f}"
                )
            if summary.sample_effective_track_width_stats is not None:
                print_distribution_summary(
                    "per_sample_effective_track_width_m",
                    summary.sample_effective_track_width_stats,
                    "m",
                )
            if summary.apparent_yaw_inertia_torque_only_upper_bound_kg_m2 is not None:
                print(
                    "apparent_yaw_inertia_torque_only_upper_bound_kg_m2="
                    f"{summary.apparent_yaw_inertia_torque_only_upper_bound_kg_m2:.9f}"
                )
        if recovery_aggregate is not None:
            print("Recovery aggregate")
            print(
                f"turns={recovery_aggregate.turn_count}, valid_turns={recovery_aggregate.valid_turn_count}, "
                f"likely_slip_turns={recovery_aggregate.likely_slip_turn_count}, "
                f"mean_abs_angle_deg={recovery_aggregate.mean_abs_angle_deg:.3f}, "
                f"median_abs_angle_deg={recovery_aggregate.median_abs_angle_deg:.3f}, "
                f"mean_odometric_equivalent_track_width_m={recovery_aggregate.mean_effective_track_width_m:.6f}, "
                f"median_odometric_equivalent_track_width_m={recovery_aggregate.median_effective_track_width_m:.6f}, "
                f"mean_peak_gyro_radps={recovery_aggregate.mean_peak_abs_gyro_radps:.3f}, "
                f"median_rotation_alignment={recovery_aggregate.median_rotation_alignment:.3f}"
            )
            if recovery_aggregate.sample_effective_track_width_stats is not None:
                print_distribution_summary(
                    "aggregate_per_sample_effective_track_width_m",
                    recovery_aggregate.sample_effective_track_width_stats,
                    "m",
                )
            if recovery_aggregate.apparent_yaw_inertia_torque_only_upper_bound_kg_m2 is not None:
                print(
                    "apparent_yaw_inertia_torque_only_upper_bound_kg_m2="
                    f"{recovery_aggregate.apparent_yaw_inertia_torque_only_upper_bound_kg_m2:.9f}"
                )

    if args.timing is not None:
        timing = analyze_timing_csv(args.timing)
        print()
        print(f"Timing CSV: {args.timing}")
        print(
            f"rows={timing.row_count}, dt_mean_us={timing.dt_mean_us:.2f}, dt_p95_us={timing.dt_p95_us:.2f}, "
            f"dt_max_us={timing.dt_max_us}"
        )
        print(
            f"ukf_predict_mean_us={timing.predict_mean_us:.2f}, ukf_predict_p95_us={timing.predict_p95_us:.2f}, "
            f"ukf_predict_max_us={timing.predict_max_us}"
        )
        print(
            f"ukf_update_mean_us={timing.update_mean_us:.2f}, ukf_update_p95_us={timing.update_p95_us:.2f}, "
            f"ukf_update_max_us={timing.update_max_us}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
