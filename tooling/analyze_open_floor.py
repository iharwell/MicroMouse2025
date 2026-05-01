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
from open_floor_yaw_fft import YawFftSummary
from open_floor_yaw_fft import summarize_yaw_fft
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
    4: "STR1",
    5: "STR2",
    6: "STR4",
    7: "IP90",
    8: "IP90_M",
    9: "IP180",
    10: "S45SD",
    11: "S45SD_M",
    12: "S45SS",
    13: "S45SS_M",
    14: "S45LS",
    15: "S45LS_M",
    16: "S45LD",
    17: "S45LD_M",
    18: "S90SD",
    19: "S90SD_M",
    20: "S90SS",
    21: "S90SS_M",
    22: "S90LS",
    23: "S90LS_M",
    24: "S90LD",
    25: "S90LD_M",
    26: "S135SD",
    27: "S135SD_M",
    28: "S135SS",
    29: "S135SS_M",
    30: "S135LS",
    31: "S135LS_M",
    32: "S135LD",
    33: "S135LD_M",
    34: "S180SS",
    35: "S180SS_M",
    36: "S180LS",
    37: "S180LS_M",
    38: "RECOVERY",
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
MIRRORED_PRIMITIVE_IDS = {8, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37}
YAW_CLOCKWISE_PRIMITIVE_IDS = {7, 9}
YAW_COUNTERCLOCKWISE_PRIMITIVE_IDS = {8, 37}


def direction_id_for_row(row: dict[str, str]) -> int:
    section_id = int(row["section_id"])
    primitive_id = int(row["primitive_id"])
    if section_id in (2, 3):
        return 1 if float(row["left_drive_command"]) >= 0.0 else 2
    if section_id == 4:
        if primitive_id in YAW_CLOCKWISE_PRIMITIVE_IDS:
            return 5
        if primitive_id in YAW_COUNTERCLOCKWISE_PRIMITIVE_IDS:
            return 6
        return 0
    if section_id == 5:
        return 8 if primitive_id in MIRRORED_PRIMITIVE_IDS else 9
    if section_id == 6:
        return 5
    if section_id == 7:
        return 6
    return 0


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


@dataclass
class AllanPoint:
    tau_s: float
    gyro_allan_dev_radps: float
    accel_x_allan_dev_mps2: float
    accel_y_allan_dev_mps2: float


@dataclass
class AllanNoiseSummary:
    sample_period_s: float
    sample_count: int
    points: list[AllanPoint]
    gyro_angle_random_walk_rad_sqrt_s: float | None
    gyro_bias_instability_radps_lower_bound: float | None
    accel_x_velocity_random_walk_mps_sqrt_s: float | None
    accel_y_velocity_random_walk_mps_sqrt_s: float | None
    accel_x_bias_instability_mps2_lower_bound: float | None
    accel_y_bias_instability_mps2_lower_bound: float | None


@dataclass
class TailStats:
    mean: float
    sigma: float
    p95: float
    p99: float
    p999: float
    maximum: float


@dataclass
class TrackWidthSegmentSummary:
    section_label: str
    primitive_id: int
    primitive_name: str
    repeat_index: int
    direction_id: int
    direction_name: str
    speed_bin: int
    sample_count: int
    weighted_track_width_m: float | None
    robust_track_width_m: float | None
    track_width_distribution: DistributionSummary | None
    predicted_vs_gyro_correlation: float | None


@dataclass
class RobustTrackWidthSummary:
    label: str
    segment_count: int
    sample_count: int
    weighted_track_width_m: float | None
    robust_track_width_m: float | None
    bootstrap_l05_m: float | None
    bootstrap_l50_m: float | None
    bootstrap_l95_m: float | None
    track_width_distribution: DistributionSummary | None
    predicted_vs_gyro_correlation: float | None
    segment_summaries: list[TrackWidthSegmentSummary]


@dataclass
class RotationalTrackWidthSegment:
    section_label: str
    primitive_id: int
    repeat_index: int
    direction_id: int
    speed_bin: int
    samples: list[tuple[float, float]]


@dataclass
class SignalLagSummary:
    label: str
    sample_count: int
    best_lag_samples: int | None
    best_lag_ms: float | None
    zero_lag_correlation: float | None
    peak_correlation: float | None


@dataclass
class EstimatorEnvelopeSummary:
    sample_count: int
    yaw_consistency_abs: TailStats | None
    yaw_window_mismatch_abs: TailStats | None
    nhc_residual_sigma_abs: TailStats | None


@dataclass
class YawDecaySegmentSummary:
    label: str
    segment_id: int
    sample_count: int
    initial_abs_gyro_radps: float
    decay_time_constant_s: float
    equivalent_yaw_rate_damping_nms_per_rad: float | None
    fit_r2: float


@dataclass
class YawDecaySummary:
    segment_count: int
    decay_time_constant_distribution: DistributionSummary | None
    damping_distribution: DistributionSummary | None
    segment_summaries: list[YawDecaySegmentSummary]


@dataclass
class TimingBreakdownSummary:
    row_count: int
    control_duration_us: TailStats | None
    encoder_window_us: TailStats | None
    encoder_to_predict_us: TailStats | None
    imu_read_duration_us: TailStats | None
    imu_drdy_to_read_start_us: TailStats | None
    front_sensor_ready_us: TailStats | None
    left_sensor_ready_us: TailStats | None
    right_sensor_ready_us: TailStats | None
    sensor_ready_skew_us: TailStats | None


@dataclass
class SensorTimestampSummary:
    row_count: int
    encoder_age_us: TailStats | None
    imu_age_us: TailStats | None


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
    parser.add_argument(
        "--fft-fallback-control-log",
        type=Path,
        help="Optional fallback logging.txt when the latest card is missing drive electrical dumps but still has the latest mass geometry.",
    )
    parser.add_argument(
        "--max-control-tick-sequence",
        type=int,
        help="Optional inclusive trim point. Rows with larger control_tick_sequence values are ignored.",
    )
    parser.add_argument(
        "--max-master-time-us",
        type=int,
        help="Optional inclusive trim point. Rows with larger master_time_us values are ignored.",
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


def row_within_trim(
    row: dict[str, str],
    *,
    max_control_tick_sequence: int | None,
    max_master_time_us: int | None,
) -> bool:
    if max_control_tick_sequence is not None:
        tick_raw = row.get("control_tick_sequence")
        if tick_raw is not None and int(tick_raw) > max_control_tick_sequence:
            return False
    if max_master_time_us is not None:
        time_raw = row.get("master_time_us", row.get("mono_time_us"))
        if time_raw is not None and int(time_raw) > max_master_time_us:
            return False
    return True


def tail_stats(values: Iterable[float]) -> TailStats | None:
    samples = sorted(abs(float(value)) for value in values)
    if not samples:
        return None
    return TailStats(
        mean=statistics.fmean(samples),
        sigma=statistics.pstdev(samples),
        p95=percentile(samples, 0.95),
        p99=percentile(samples, 0.99),
        p999=percentile(samples, 0.999),
        maximum=samples[-1],
    )


def correlation_coefficient(values_x: list[float], values_y: list[float]) -> float | None:
    if len(values_x) != len(values_y) or len(values_x) < 2:
        return None
    mean_x = statistics.fmean(values_x)
    mean_y = statistics.fmean(values_y)
    centered_x = [value - mean_x for value in values_x]
    centered_y = [value - mean_y for value in values_y]
    denom_x = math.sqrt(sum(value * value for value in centered_x))
    denom_y = math.sqrt(sum(value * value for value in centered_y))
    if denom_x <= 0.0 or denom_y <= 0.0:
        return None
    numer = sum(x * y for x, y in zip(centered_x, centered_y))
    return numer / (denom_x * denom_y)


def compute_overlapping_allan_deviation(values: list[float], sample_period_s: float) -> list[tuple[float, float]]:
    sample_count = len(values)
    if sample_count < 4 or not (sample_period_s > 0.0):
        return []

    cumulative_sum = [0.0]
    for value in values:
        cumulative_sum.append(cumulative_sum[-1] + value)

    allan_points: list[tuple[float, float]] = []
    averaging_factor = 1
    while (2 * averaging_factor) < sample_count:
        diffs: list[float] = []
        limit = sample_count - (2 * averaging_factor)
        for start in range(limit + 1):
            avg_0 = (
                cumulative_sum[start + averaging_factor] -
                cumulative_sum[start]
            ) / averaging_factor
            avg_1 = (
                cumulative_sum[start + (2 * averaging_factor)] -
                cumulative_sum[start + averaging_factor]
            ) / averaging_factor
            diffs.append(avg_1 - avg_0)
        if diffs:
            allan_variance = 0.5 * statistics.fmean(diff * diff for diff in diffs)
            allan_points.append((averaging_factor * sample_period_s, math.sqrt(max(0.0, allan_variance))))
        averaging_factor *= 2
    return allan_points


def estimate_white_noise_coefficient(points: list[tuple[float, float]]) -> float | None:
    if not points:
        return None
    early_points = points[:min(3, len(points))]
    coefficients = [sigma * math.sqrt(tau_s) for tau_s, sigma in early_points if tau_s > 0.0]
    if not coefficients:
        return None
    return statistics.fmean(coefficients)


def estimate_bias_instability_lower_bound(points: list[tuple[float, float]]) -> float | None:
    if not points:
        return None
    min_sigma = min(point[1] for point in points)
    # Standard Allan-deviation bias-instability approximation for a flicker floor.
    return min_sigma / 0.664 if min_sigma > 0.0 else None


def build_allan_noise_summary(
    sample_period_s: float,
    gyro_values: list[float],
    accel_x_values: list[float],
    accel_y_values: list[float],
) -> AllanNoiseSummary | None:
    if not gyro_values or not accel_x_values or not accel_y_values:
        return None
    gyro_points = compute_overlapping_allan_deviation(gyro_values, sample_period_s)
    accel_x_points = compute_overlapping_allan_deviation(accel_x_values, sample_period_s)
    accel_y_points = compute_overlapping_allan_deviation(accel_y_values, sample_period_s)
    if not gyro_points:
        return None

    point_count = min(len(gyro_points), len(accel_x_points), len(accel_y_points))
    points = [
        AllanPoint(
            tau_s=gyro_points[index][0],
            gyro_allan_dev_radps=gyro_points[index][1],
            accel_x_allan_dev_mps2=accel_x_points[index][1],
            accel_y_allan_dev_mps2=accel_y_points[index][1],
        )
        for index in range(point_count)
    ]
    return AllanNoiseSummary(
        sample_period_s=sample_period_s,
        sample_count=len(gyro_values),
        points=points,
        gyro_angle_random_walk_rad_sqrt_s=estimate_white_noise_coefficient(gyro_points),
        gyro_bias_instability_radps_lower_bound=estimate_bias_instability_lower_bound(gyro_points),
        accel_x_velocity_random_walk_mps_sqrt_s=estimate_white_noise_coefficient(accel_x_points),
        accel_y_velocity_random_walk_mps_sqrt_s=estimate_white_noise_coefficient(accel_y_points),
        accel_x_bias_instability_mps2_lower_bound=estimate_bias_instability_lower_bound(accel_x_points),
        accel_y_bias_instability_mps2_lower_bound=estimate_bias_instability_lower_bound(accel_y_points),
    )


def huber_track_width_fit(samples: list[tuple[float, float]]) -> float | None:
    if len(samples) < 8:
        return None
    weighted_pairs = [
        (diff_speed, gyro_radps, max(abs(gyro_radps), 0.25))
        for diff_speed, gyro_radps in samples
    ]
    numerator = sum(weight * diff_speed * gyro_radps for diff_speed, gyro_radps, weight in weighted_pairs)
    denominator = sum(weight * diff_speed * diff_speed for diff_speed, _, weight in weighted_pairs)
    if denominator <= 0.0:
        return None
    beta = numerator / denominator
    for _ in range(4):
        residuals = [gyro_radps - (beta * diff_speed) for diff_speed, gyro_radps, _ in weighted_pairs]
        abs_residuals = sorted(abs(value) for value in residuals)
        if not abs_residuals:
            break
        mad = percentile(abs_residuals, 0.5)
        scale = max(1.0e-6, 1.4826 * mad)
        numerator = 0.0
        denominator = 0.0
        for (diff_speed, gyro_radps, base_weight), residual in zip(weighted_pairs, residuals):
            huber_weight = min(1.0, 1.5 * scale / max(abs(residual), 1.0e-9))
            weight = base_weight * huber_weight
            numerator += weight * diff_speed * gyro_radps
            denominator += weight * diff_speed * diff_speed
        if denominator <= 0.0:
            return None
        beta = numerator / denominator
    if beta <= 0.0:
        return None
    return 1.0 / beta


def summarize_track_width_fit(
    label: str,
    segments: list[RotationalTrackWidthSegment],
    gyro_bias_radps: float,
) -> RobustTrackWidthSummary | None:
    flat_samples = [
        (diff_speed, gyro_raw_radps - gyro_bias_radps)
        for segment in segments
        for diff_speed, gyro_raw_radps in segment.samples
    ]
    if len(flat_samples) < 8:
        return None
    robust_track_width = huber_track_width_fit(flat_samples)
    per_sample_track_widths = [
        abs(diff_speed / gyro_radps)
        for diff_speed, gyro_radps in flat_samples
        if abs(gyro_radps) >= 0.25
    ]
    predicted_yaw: list[float] = []
    measured_yaw: list[float] = []
    if robust_track_width is not None and robust_track_width > 0.0:
        for diff_speed, gyro_radps in flat_samples:
            predicted_yaw.append(diff_speed / robust_track_width)
            measured_yaw.append(gyro_radps)
    correlation = correlation_coefficient(predicted_yaw, measured_yaw)

    bootstrap_track_widths: list[float] = []
    if len(segments) >= 2:
        import random
        rng = random.Random(0)
        for _ in range(128):
            sampled_segments = [rng.choice(segments) for _ in range(len(segments))]
            bootstrap_samples = [
                (diff_speed, gyro_raw_radps - gyro_bias_radps)
                for segment in sampled_segments
                for diff_speed, gyro_raw_radps in segment.samples
            ]
            estimate = huber_track_width_fit(bootstrap_samples)
            if estimate is not None:
                bootstrap_track_widths.append(estimate)
    bootstrap_track_widths.sort()
    sorted_track_widths = sorted(per_sample_track_widths)
    distribution = (
        DistributionSummary(
            count=len(per_sample_track_widths),
            l5=percentile(sorted_track_widths, 0.05),
            l10=percentile(sorted_track_widths, 0.10),
            l25=percentile(sorted_track_widths, 0.25),
            l50=percentile(sorted_track_widths, 0.50),
            l75=percentile(sorted_track_widths, 0.75),
            l90=percentile(sorted_track_widths, 0.90),
            l95=percentile(sorted_track_widths, 0.95),
            mean=statistics.fmean(per_sample_track_widths),
            sigma=statistics.pstdev(per_sample_track_widths),
        )
        if per_sample_track_widths else None
    )
    segment_summaries: list[TrackWidthSegmentSummary] = []
    for segment in segments:
        segment_samples = [
            (diff_speed, gyro_raw_radps - gyro_bias_radps)
            for diff_speed, gyro_raw_radps in segment.samples
        ]
        segment_track_width = huber_track_width_fit(segment_samples)
        segment_per_sample_track_widths = [
            abs(diff_speed / gyro_radps)
            for diff_speed, gyro_radps in segment_samples
            if abs(gyro_radps) >= 0.25
        ]
        segment_distribution = None
        if segment_per_sample_track_widths:
            sorted_segment_track_widths = sorted(segment_per_sample_track_widths)
            segment_distribution = DistributionSummary(
                count=len(segment_per_sample_track_widths),
                l5=percentile(sorted_segment_track_widths, 0.05),
                l10=percentile(sorted_segment_track_widths, 0.10),
                l25=percentile(sorted_segment_track_widths, 0.25),
                l50=percentile(sorted_segment_track_widths, 0.50),
                l75=percentile(sorted_segment_track_widths, 0.75),
                l90=percentile(sorted_segment_track_widths, 0.90),
                l95=percentile(sorted_segment_track_widths, 0.95),
                mean=statistics.fmean(segment_per_sample_track_widths),
                sigma=statistics.pstdev(segment_per_sample_track_widths),
            )
        predicted_segment_yaw: list[float] = []
        measured_segment_yaw: list[float] = []
        if segment_track_width is not None and segment_track_width > 0.0:
            for diff_speed, gyro_radps in segment_samples:
                predicted_segment_yaw.append(diff_speed / segment_track_width)
                measured_segment_yaw.append(gyro_radps)
        segment_summaries.append(
            TrackWidthSegmentSummary(
                section_label=segment.section_label,
                primitive_id=segment.primitive_id,
                primitive_name=PRIMITIVE_NAMES.get(segment.primitive_id, str(segment.primitive_id)),
                repeat_index=segment.repeat_index,
                direction_id=segment.direction_id,
                direction_name=DIRECTION_NAMES.get(segment.direction_id, str(segment.direction_id)),
                speed_bin=segment.speed_bin,
                sample_count=len(segment.samples),
                weighted_track_width_m=segment_track_width,
                robust_track_width_m=(
                    statistics.median(segment_per_sample_track_widths)
                    if segment_per_sample_track_widths else None
                ),
                track_width_distribution=segment_distribution,
                predicted_vs_gyro_correlation=correlation_coefficient(
                    predicted_segment_yaw,
                    measured_segment_yaw,
                ),
            )
        )
    return RobustTrackWidthSummary(
        label=label,
        segment_count=len(segments),
        sample_count=len(flat_samples),
        weighted_track_width_m=robust_track_width,
        robust_track_width_m=(statistics.median(per_sample_track_widths) if per_sample_track_widths else None),
        bootstrap_l05_m=percentile(bootstrap_track_widths, 0.05) if bootstrap_track_widths else None,
        bootstrap_l50_m=percentile(bootstrap_track_widths, 0.50) if bootstrap_track_widths else None,
        bootstrap_l95_m=percentile(bootstrap_track_widths, 0.95) if bootstrap_track_widths else None,
        track_width_distribution=distribution,
        predicted_vs_gyro_correlation=correlation,
        segment_summaries=segment_summaries,
    )


def estimate_signal_lag(
    label: str,
    samples: list[tuple[float, float]],
    gyro_bias_radps: float,
    max_lag_samples: int,
) -> SignalLagSummary | None:
    if len(samples) < (2 * max_lag_samples) + 8:
        return None
    encoder_signal = [sample[0] for sample in samples]
    gyro_signal = [sample[1] - gyro_bias_radps for sample in samples]
    best_lag: int | None = None
    best_corr: float | None = None
    for lag in range(-max_lag_samples, max_lag_samples + 1):
        if lag < 0:
            aligned_encoder = encoder_signal[-lag:]
            aligned_gyro = gyro_signal[:len(aligned_encoder)]
        elif lag > 0:
            aligned_encoder = encoder_signal[:-lag]
            aligned_gyro = gyro_signal[lag:]
        else:
            aligned_encoder = encoder_signal
            aligned_gyro = gyro_signal
        corr = correlation_coefficient(aligned_encoder, aligned_gyro)
        if corr is None:
            continue
        if best_corr is None or abs(corr) > abs(best_corr):
            best_corr = corr
            best_lag = lag
    zero_lag_corr = correlation_coefficient(encoder_signal, gyro_signal)
    return SignalLagSummary(
        label=label,
        sample_count=len(samples),
        best_lag_samples=best_lag,
        best_lag_ms=(None if best_lag is None else float(best_lag)),
        zero_lag_correlation=zero_lag_corr,
        peak_correlation=best_corr,
    )


def estimate_yaw_decay_summary(
    segments: list[tuple[str, list[float], float]],
    stationary_gyro_sigma_radps: float,
    gyro_bias_radps: float,
    yaw_inertia_kg_m2: float | None,
) -> YawDecaySummary | None:
    segment_summaries: list[YawDecaySegmentSummary] = []
    damping_values: list[float] = []
    tau_values: list[float] = []
    threshold_floor = max(3.0 * stationary_gyro_sigma_radps, 0.10)
    for segment_id, (label, values, dt_seconds) in enumerate(segments, start=1):
        if len(values) < 20:
            continue
        corrected_values = [value - gyro_bias_radps for value in values]
        initial = abs(corrected_values[0])
        trimmed_values = [
            abs(value)
            for value in corrected_values
            if math.copysign(1.0, value) == math.copysign(1.0, corrected_values[0]) and abs(value) >= threshold_floor
        ]
        if len(trimmed_values) < 20:
            continue
        times = [index * dt_seconds for index in range(len(trimmed_values))]
        logs = [math.log(value) for value in trimmed_values if value > 0.0]
        if len(logs) != len(times) or len(logs) < 20:
            continue
        mean_t = statistics.fmean(times)
        mean_log = statistics.fmean(logs)
        numerator = sum((time - mean_t) * (log_value - mean_log) for time, log_value in zip(times, logs))
        denominator = sum((time - mean_t) * (time - mean_t) for time in times)
        if denominator <= 0.0:
            continue
        slope = numerator / denominator
        if slope >= 0.0:
            continue
        intercept = mean_log - (slope * mean_t)
        fitted = [intercept + (slope * time) for time in times]
        ss_tot = sum((log_value - mean_log) * (log_value - mean_log) for log_value in logs)
        ss_res = sum((log_value - fit_value) * (log_value - fit_value) for log_value, fit_value in zip(logs, fitted))
        tau_s = -1.0 / slope
        tau_values.append(tau_s)
        damping = None
        if yaw_inertia_kg_m2 is not None and tau_s > 0.0:
            damping = yaw_inertia_kg_m2 / tau_s
            damping_values.append(damping)
        fit_r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0.0 else 0.0
        segment_summaries.append(
            YawDecaySegmentSummary(
                label=label,
                segment_id=segment_id,
                sample_count=len(trimmed_values),
                initial_abs_gyro_radps=initial,
                decay_time_constant_s=tau_s,
                equivalent_yaw_rate_damping_nms_per_rad=damping,
                fit_r2=fit_r2,
            )
        )

    if not segment_summaries:
        return None

    tau_sorted = sorted(tau_values)
    damping_sorted = sorted(damping_values)
    return YawDecaySummary(
        segment_count=len(segment_summaries),
        decay_time_constant_distribution=DistributionSummary(
            count=len(tau_values),
            l5=percentile(tau_sorted, 0.05),
            l10=percentile(tau_sorted, 0.10),
            l25=percentile(tau_sorted, 0.25),
            l50=percentile(tau_sorted, 0.50),
            l75=percentile(tau_sorted, 0.75),
            l90=percentile(tau_sorted, 0.90),
            l95=percentile(tau_sorted, 0.95),
            mean=statistics.fmean(tau_values),
            sigma=statistics.pstdev(tau_values),
        ),
        damping_distribution=(
            DistributionSummary(
                count=len(damping_values),
                l5=percentile(damping_sorted, 0.05),
                l10=percentile(damping_sorted, 0.10),
                l25=percentile(damping_sorted, 0.25),
                l50=percentile(damping_sorted, 0.50),
                l75=percentile(damping_sorted, 0.75),
                l90=percentile(damping_sorted, 0.90),
                l95=percentile(damping_sorted, 0.95),
                mean=statistics.fmean(damping_values),
                sigma=statistics.pstdev(damping_values),
            )
            if damping_values else None
        ),
        segment_summaries=segment_summaries,
    )


def analyze_main_csv(
    path: Path,
    control_log_path: Path | None,
    yaw_fft_fallback_control_log_path: Path | None,
    *,
    max_control_tick_sequence: int | None,
    max_master_time_us: int | None,
) -> tuple[
    StationarySummary,
    AllanNoiseSummary | None,
    list[LaunchMagnitudeSummary],
    LaunchFloorSummary | None,
    RepeatabilitySummary,
    dict[str, float],
    TirePlantFitSummary | None,
    YawFftSummary | None,
    FeedforwardAlignmentSummary | None,
    list[RecoveryTurnSummary],
    RecoveryAggregateSummary | None,
    RobustTrackWidthSummary | None,
    RobustTrackWidthSummary | None,
    SignalLagSummary | None,
    EstimatorEnvelopeSummary | None,
    YawDecaySummary | None,
    SensorTimestampSummary | None,
]:
    static_dt_seconds = 0.0
    static_gyro_raw: list[float] = []
    static_gyro_logged_bias: list[float] = []
    static_gyro_logged_corrected: list[float] = []
    static_accel_body_x: list[float] = []
    static_accel_body_y: list[float] = []
    static_planar_accel: list[float] = []
    static_dt_us_values: list[int] = []

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
    current_recovery_key: tuple[int, int] | None = None
    yaw_rows_by_key: DefaultDict[tuple[int, int], list[dict[str, str]]] = defaultdict(list)
    available_section_ids: set[int] = set()
    rotational_track_width_segments_by_label: DefaultDict[str, list[RotationalTrackWidthSegment]] = defaultdict(list)
    rotational_lag_samples_by_label: DefaultDict[str, list[tuple[float, float]]] = defaultdict(list)
    current_rotational_segment_key: tuple[str, int, int, int, int] | None = None
    current_rotational_track_width_samples: list[tuple[float, float]] = []
    current_rotational_lag_samples: list[tuple[float, float]] = []
    estimator_yaw_consistency_values: list[float] = []
    estimator_yaw_window_mismatch_values: list[float] = []
    estimator_nhc_residual_sigma_values: list[float] = []
    decay_segments: list[tuple[str, list[float], float]] = []
    current_decay_key: tuple[str, int, int, int, int] | None = None
    current_decay_values: list[float] = []
    current_decay_dt_seconds = 0.0
    encoder_age_us_values: list[float] = []
    imu_age_us_values: list[float] = []

    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            if not row_within_trim(
                row,
                max_control_tick_sequence=max_control_tick_sequence,
                max_master_time_us=max_master_time_us,
            ):
                continue
            section_id = int(row["section_id"])
            primitive_id = int(row["primitive_id"])
            available_section_ids.add(section_id)

            if primitive_id == RECOVERY_PRIMITIVE_ID:
                recovery_key = (
                    section_id,
                    int(row["repeat_index"]),
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

            if section_id == 4:
                yaw_rows_by_key[(primitive_id, int(row["repeat_index"]))].append(dict(row))

            if section_id == STATIC_SECTION_ID and primitive_id == STATIC_PRIMITIVE_ID:
                dt_us = int(row["dt_us"])
                static_dt_seconds += 1.0e-6 * dt_us
                static_dt_us_values.append(dt_us)
                static_gyro_raw.append(float(row["gyro_raw_radps"]))
                static_gyro_logged_bias.append(float(row["gyro_bias_radps"]))
                static_gyro_logged_corrected.append(float(row["gyro_radps"]))
                static_accel_body_x.append(float(row["accel_body_x_mps2"]))
                static_accel_body_y.append(float(row["accel_body_y_mps2"]))
                static_planar_accel.append(float(row["planar_accel_mps2"]))

            if (
                (section_id == 3 and int(row["phase_id"]) == 11) or
                (section_id == 4 and int(row["phase_id"]) == 8) or
                (section_id == 5 and int(row["phase_id"]) == 11)
            ):
                estimator_yaw_consistency_values.append(abs(float(row["yaw_consistency_lp_radps"])))
                estimator_yaw_window_mismatch_values.append(abs(float(row["yaw_window_mismatch_rad"])))
                estimator_nhc_residual_sigma_values.append(abs(float(row["nhc_residual_sigma"])))

            master_time_us = int(row["master_time_us"])
            encoder_timestamp_us = int(row["encoder_timestamp_us"])
            imu_timestamp_us = int(row["imu_timestamp_us"])
            encoder_age_us_values.append(float(master_time_us - encoder_timestamp_us))
            imu_age_us_values.append(float(master_time_us - imu_timestamp_us))

            phase_id = int(row["phase_id"])
            active_rotation_phase = (
                (section_id == 4 and phase_id == 8) or
                (section_id == 5 and phase_id == 11)
            )
            if active_rotation_phase:
                diff_speed_mps = (
                    float(row["left_encoder_velocity_mps"]) -
                    float(row["right_encoder_velocity_mps"])
                )
                gyro_raw_radps = float(row["gyro_raw_radps"])
                rotational_label = SECTION_NAMES.get(section_id, f"SEC_{section_id:02d}")
                rotational_key = (
                    rotational_label,
                    primitive_id,
                    int(row["repeat_index"]),
                    direction_id_for_row(row),
                    int(row["speed_bin"]),
                )
                if current_rotational_segment_key != rotational_key:
                    if current_rotational_track_width_samples:
                        rotational_track_width_segments_by_label[current_rotational_segment_key[0]].append(
                            RotationalTrackWidthSegment(
                                section_label=current_rotational_segment_key[0],
                                primitive_id=current_rotational_segment_key[1],
                                repeat_index=current_rotational_segment_key[2],
                                direction_id=current_rotational_segment_key[3],
                                speed_bin=current_rotational_segment_key[4],
                                samples=current_rotational_track_width_samples,
                            )
                        )
                        rotational_lag_samples_by_label[current_rotational_segment_key[0]].extend(
                            current_rotational_lag_samples
                        )
                    current_rotational_segment_key = rotational_key
                    current_rotational_track_width_samples = []
                    current_rotational_lag_samples = []
                current_rotational_track_width_samples.append((diff_speed_mps, gyro_raw_radps))
                current_rotational_lag_samples.append((diff_speed_mps, gyro_raw_radps))
            elif current_rotational_segment_key is not None and current_rotational_track_width_samples:
                rotational_track_width_segments_by_label[current_rotational_segment_key[0]].append(
                    RotationalTrackWidthSegment(
                        section_label=current_rotational_segment_key[0],
                        primitive_id=current_rotational_segment_key[1],
                        repeat_index=current_rotational_segment_key[2],
                        direction_id=current_rotational_segment_key[3],
                        speed_bin=current_rotational_segment_key[4],
                        samples=current_rotational_track_width_samples,
                    )
                )
                rotational_lag_samples_by_label[current_rotational_segment_key[0]].extend(
                    current_rotational_lag_samples
                )
                current_rotational_segment_key = None
                current_rotational_track_width_samples = []
                current_rotational_lag_samples = []

            decay_phase = (
                (section_id == 4 and phase_id == 9) or
                (section_id == 5 and phase_id == 12)
            )
            if decay_phase:
                decay_label = SECTION_NAMES.get(section_id, f"SEC_{section_id:02d}")
                decay_key = (
                    decay_label,
                    primitive_id,
                    int(row["repeat_index"]),
                    direction_id_for_row(row),
                    int(row["speed_bin"]),
                )
                if current_decay_key != decay_key:
                    if current_decay_values:
                        decay_segments.append((current_decay_key[0], current_decay_values, current_decay_dt_seconds))
                    current_decay_key = decay_key
                    current_decay_values = []
                current_decay_values.append(float(row["gyro_raw_radps"]))
                current_decay_dt_seconds = 1.0e-6 * int(row["dt_us"])
            elif current_decay_key is not None and current_decay_values:
                decay_segments.append((current_decay_key[0], current_decay_values, current_decay_dt_seconds))
                current_decay_key = None
                current_decay_values = []
                current_decay_dt_seconds = 0.0

            if section_id != LAUNCH_SECTION_ID or primitive_id != LAUNCH_PRIMITIVE_ID:
                continue

            signed_command = float(row["left_drive_command"])
            abs_command = round(abs(signed_command), 2)
            if abs_command <= 0.0:
                continue

            repeat_index = int(row["repeat_index"])
            direction_id = direction_id_for_row(row)
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
    if current_rotational_segment_key is not None and current_rotational_track_width_samples:
        rotational_track_width_segments_by_label[current_rotational_segment_key[0]].append(
            RotationalTrackWidthSegment(
                section_label=current_rotational_segment_key[0],
                primitive_id=current_rotational_segment_key[1],
                repeat_index=current_rotational_segment_key[2],
                direction_id=current_rotational_segment_key[3],
                speed_bin=current_rotational_segment_key[4],
                samples=current_rotational_track_width_samples,
            )
        )
        rotational_lag_samples_by_label[current_rotational_segment_key[0]].extend(
            current_rotational_lag_samples
        )
    if current_decay_key is not None and current_decay_values:
        decay_segments.append((current_decay_key[0], current_decay_values, current_decay_dt_seconds))

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
    allan_noise_summary = build_allan_noise_summary(
        sample_period_s=(1.0e-6 * statistics.fmean(static_dt_us_values)) if static_dt_us_values else 0.0,
        gyro_values=static_gyro_independent_corrected,
        accel_x_values=static_accel_body_x,
        accel_y_values=static_accel_body_y,
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
    yaw_fft_summary = summarize_yaw_fft(
        dict(yaw_rows_by_key),
        control_log_path,
        yaw_fft_fallback_control_log_path,
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
    yaw_track_width_summary = summarize_track_width_fit(
        "SEC_40_YAW",
        rotational_track_width_segments_by_label.get("SEC_40_YAW", []),
        stationary_summary.gyro_independent_bias_radps,
    )
    smooth_track_width_summary = summarize_track_width_fit(
        "SEC_50_SMOOTH",
        rotational_track_width_segments_by_label.get("SEC_50_SMOOTH", []),
        stationary_summary.gyro_independent_bias_radps,
    )
    lag_samples = rotational_lag_samples_by_label.get("SEC_40_YAW", []) + rotational_lag_samples_by_label.get("SEC_50_SMOOTH", [])
    signal_lag_summary = estimate_signal_lag(
        "encoder_vs_gyro_yaw_rate",
        lag_samples,
        stationary_summary.gyro_independent_bias_radps,
        max_lag_samples=20,
    )
    estimator_envelope_summary = EstimatorEnvelopeSummary(
        sample_count=len(estimator_yaw_consistency_values),
        yaw_consistency_abs=tail_stats(estimator_yaw_consistency_values),
        yaw_window_mismatch_abs=tail_stats(estimator_yaw_window_mismatch_values),
        nhc_residual_sigma_abs=tail_stats(estimator_nhc_residual_sigma_values),
    )
    yaw_decay_summary = estimate_yaw_decay_summary(
        decay_segments,
        stationary_summary.gyro_independent_corrected.sigma,
        stationary_summary.gyro_independent_bias_radps,
        yaw_fft_summary.yaw_inertia_kg_m2 if yaw_fft_summary is not None else None,
    )
    sensor_timestamp_summary = SensorTimestampSummary(
        row_count=len(encoder_age_us_values),
        encoder_age_us=tail_stats(encoder_age_us_values),
        imu_age_us=tail_stats(imu_age_us_values),
    )

    return (
        stationary_summary,
        allan_noise_summary,
        launch_summaries,
        launch_floor_summary,
        repeatability_summary,
        suggestions,
        tire_plant_fit,
        yaw_fft_summary,
        feedforward_alignment,
        recovery_summaries,
        recovery_aggregate,
        yaw_track_width_summary,
        smooth_track_width_summary,
        signal_lag_summary,
        estimator_envelope_summary,
        yaw_decay_summary,
        sensor_timestamp_summary,
    )


def analyze_timing_csv(
    path: Path,
    *,
    max_control_tick_sequence: int | None,
    max_master_time_us: int | None,
) -> tuple[TimingSummary, TimingBreakdownSummary | None]:
    dt_values: list[int] = []
    predict_values: list[int] = []
    update_values: list[int] = []
    control_duration_values: list[float] = []
    encoder_window_values: list[float] = []
    encoder_to_predict_values: list[float] = []
    imu_read_duration_values: list[float] = []
    imu_drdy_to_read_start_values: list[float] = []
    front_sensor_ready_values: list[float] = []
    left_sensor_ready_values: list[float] = []
    right_sensor_ready_values: list[float] = []
    sensor_ready_skew_values: list[float] = []

    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            if not row_within_trim(
                row,
                max_control_tick_sequence=max_control_tick_sequence,
                max_master_time_us=max_master_time_us,
            ):
                continue
            dt_values.append(int(row["dt_us"]))
            predict_values.append(int(row["ukf_predict_duration_us"]))
            update_values.append(int(row["ukf_update_duration_us"]))
            control_duration_values.append(float(int(row["control_end_us"]) - int(row["control_start_us"])))
            encoder_window_values.append(float(int(row["encoder_read_done_us"]) - int(row["encoder_latch_us"])))
            encoder_to_predict_values.append(float(int(row["ukf_predict_start_us"]) - int(row["encoder_read_done_us"])))
            imu_read_duration_values.append(float(int(row["imu_read_done_us"]) - int(row["imu_read_start_us"])))
            imu_drdy_us = int(row["imu_drdy_us"])
            if imu_drdy_us > 0:
                imu_drdy_to_read_start_values.append(float(int(row["imu_read_start_us"]) - imu_drdy_us))
            front_sensor_ready_values.append(float(int(row["front_ready_us"]) - int(row["front_adc_on_us"])))
            left_sensor_ready_values.append(float(int(row["left_ready_us"]) - int(row["left_adc_on_us"])))
            right_sensor_ready_values.append(float(int(row["right_ready_us"]) - int(row["right_adc_on_us"])))
            ready_times = [
                int(row["front_ready_us"]),
                int(row["left_ready_us"]),
                int(row["right_ready_us"]),
            ]
            sensor_ready_skew_values.append(float(max(ready_times) - min(ready_times)))

    if not dt_values:
        return (
            TimingSummary(
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
            ),
            None,
        )

    dt_sorted = sorted(dt_values)
    predict_sorted = sorted(predict_values)
    update_sorted = sorted(update_values)
    timing_summary = TimingSummary(
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
    breakdown_summary = TimingBreakdownSummary(
        row_count=len(dt_values),
        control_duration_us=tail_stats(control_duration_values),
        encoder_window_us=tail_stats(encoder_window_values),
        encoder_to_predict_us=tail_stats(encoder_to_predict_values),
        imu_read_duration_us=tail_stats(imu_read_duration_values),
        imu_drdy_to_read_start_us=tail_stats(imu_drdy_to_read_start_values),
        front_sensor_ready_us=tail_stats(front_sensor_ready_values),
        left_sensor_ready_us=tail_stats(left_sensor_ready_values),
        right_sensor_ready_us=tail_stats(right_sensor_ready_values),
        sensor_ready_skew_us=tail_stats(sensor_ready_skew_values),
    )
    return timing_summary, breakdown_summary


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


def print_tail_stats(label: str, stats: TailStats, unit: str) -> None:
    print(
        f"{label}: mean={stats.mean:.6f} {unit}, sigma={stats.sigma:.6f} {unit}, "
        f"P95={stats.p95:.6f} {unit}, P99={stats.p99:.6f} {unit}, P99.9={stats.p999:.6f} {unit}, "
        f"max={stats.maximum:.6f} {unit}"
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
    if (
        args.fft_fallback_control_log is not None and
        not args.fft_fallback_control_log.is_file()
    ):
        print(
            f"error: FFT fallback control log not found: {args.fft_fallback_control_log}",
            file=sys.stderr,
        )
        return 1

    (
        stationary,
        allan_noise_summary,
        launch_summaries,
        launch_floor_summary,
        repeatability,
        suggestions,
        tire_plant_fit,
        yaw_fft_summary,
        feedforward_alignment,
        recovery_summaries,
        recovery_aggregate,
        yaw_track_width_summary,
        smooth_track_width_summary,
        signal_lag_summary,
        estimator_envelope_summary,
        yaw_decay_summary,
        sensor_timestamp_summary,
    ) = analyze_main_csv(
        args.main,
        control_log_path,
        args.fft_fallback_control_log,
        max_control_tick_sequence=args.max_control_tick_sequence,
        max_master_time_us=args.max_master_time_us,
    )

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
    if allan_noise_summary is not None:
        print()
        print("Stationary Allan-deviation summary")
        print(
            "method=overlapping Allan deviation on the stationary hold; short-tau coefficients estimate gyro angle random walk "
            "and accel velocity random walk, while the Allan minimum provides a bias-instability lower bound"
        )
        print(
            f"sample_period_s={allan_noise_summary.sample_period_s:.6f}, "
            f"sample_count={allan_noise_summary.sample_count}, "
            f"gyro_angle_random_walk_rad_sqrt_s={format_optional_float(allan_noise_summary.gyro_angle_random_walk_rad_sqrt_s, 9)}, "
            f"gyro_bias_instability_radps_lower_bound={format_optional_float(allan_noise_summary.gyro_bias_instability_radps_lower_bound, 9)}, "
            f"accel_x_velocity_random_walk_mps_sqrt_s={format_optional_float(allan_noise_summary.accel_x_velocity_random_walk_mps_sqrt_s, 9)}, "
            f"accel_y_velocity_random_walk_mps_sqrt_s={format_optional_float(allan_noise_summary.accel_y_velocity_random_walk_mps_sqrt_s, 9)}, "
            f"accel_x_bias_instability_mps2_lower_bound={format_optional_float(allan_noise_summary.accel_x_bias_instability_mps2_lower_bound, 9)}, "
            f"accel_y_bias_instability_mps2_lower_bound={format_optional_float(allan_noise_summary.accel_y_bias_instability_mps2_lower_bound, 9)}"
        )
        for point in allan_noise_summary.points:
            print(
                f"tau_s={point.tau_s:.6f}: gyro_allan_dev={point.gyro_allan_dev_radps:.9f} rad/s, "
                f"accel_x_allan_dev={point.accel_x_allan_dev_mps2:.9f} m/s^2, "
                f"accel_y_allan_dev={point.accel_y_allan_dev_mps2:.9f} m/s^2"
            )
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
    if estimator_envelope_summary is not None:
        print()
        print("Estimator envelope summary")
        print(
            "method=extract absolute motion-phase envelopes from phase-11 rows only so gating candidates are based on active "
            "tracking data instead of stationary noise or startup/stop transients"
        )
        print(f"sample_count={estimator_envelope_summary.sample_count}")
        if estimator_envelope_summary.yaw_consistency_abs is not None:
            print_tail_stats(
                "yaw_consistency_lp_abs_radps",
                estimator_envelope_summary.yaw_consistency_abs,
                "rad/s",
            )
        if estimator_envelope_summary.yaw_window_mismatch_abs is not None:
            print_tail_stats(
                "yaw_window_mismatch_abs_rad",
                estimator_envelope_summary.yaw_window_mismatch_abs,
                "rad",
            )
        if estimator_envelope_summary.nhc_residual_sigma_abs is not None:
            print_tail_stats(
                "nhc_residual_sigma_abs",
                estimator_envelope_summary.nhc_residual_sigma_abs,
                "sigma",
            )

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

    if yaw_fft_summary is not None:
        print()
        print("Yaw oscillation FFT summary")
        print(
            "method=SEC_40_YAW phase-9 rows only; high-pass the planned turn envelope with a centered moving average, "
            "FFT the differential wheel speed, gyro yaw rate, and differential motor torque, then keep the rigid-body "
            "yaw contribution separate from the wheel-side inertia estimate"
        )
        print(
            f"run_id={'unknown' if yaw_fft_summary.run_id is None else yaw_fft_summary.run_id}, "
            f"configured_equivalent_wheel_inertia_kg_m2={yaw_fft_summary.configured_equivalent_wheel_inertia_kg_m2:.9f}, "
            f"rigid_body_equivalent_inertia_kg_m2={yaw_fft_summary.rigid_body_equivalent_inertia_kg_m2:.9f}, "
            f"track_width_m={yaw_fft_summary.track_width_m:.9f}, "
            f"wheel_radius_m={yaw_fft_summary.wheel_radius_m:.9f}, "
            f"yaw_inertia_kg_m2={yaw_fft_summary.yaw_inertia_kg_m2:.9f}, "
            f"drive_parameter_source={'none' if yaw_fft_summary.drive_parameter_source_path is None else yaw_fft_summary.drive_parameter_source_path}, "
            f"used_fallback_drive_parameters={int(yaw_fft_summary.used_fallback_drive_parameters)}"
        )
        if yaw_fft_summary.aggregate is not None:
            aggregate = yaw_fft_summary.aggregate
            print(
                f"repeats={aggregate.repeat_count}, "
                f"strong_repeats={aggregate.strong_repeat_count}, "
                f"recommended_candidates={aggregate.recommended_candidate_count}, "
                f"dominant_frequency_hz_median={format_optional_float(aggregate.dominant_frequency_hz_median, 6)}, "
                f"wheel_speed_vs_torque_phase_deg_median={format_optional_float(aggregate.wheel_speed_vs_torque_phase_deg_median, 3)}, "
                f"yaw_vs_rigid_wheel_phase_deg_median={format_optional_float(aggregate.yaw_vs_rigid_wheel_phase_deg_median, 3)}, "
                f"yaw_coupling_magnitude_median={format_optional_float(aggregate.yaw_coupling_magnitude_median, 6)}, "
                f"raw_total_inertia_kg_m2_median={format_optional_float(aggregate.raw_total_inertia_kg_m2_median, 9)}, "
                f"phase_corrected_wheel_inertia_kg_m2_median={format_optional_float(aggregate.phase_corrected_wheel_inertia_kg_m2_median, 9)}, "
                f"recommended_wheel_inertia_kg_m2={format_optional_float(aggregate.recommended_wheel_inertia_kg_m2, 9)}"
            )
        for summary in yaw_fft_summary.repeat_summaries:
            print(
                f"primitive={PRIMITIVE_NAMES.get(summary.primitive_id, str(summary.primitive_id))}, "
                f"repeat={summary.repeat_index}, rows={summary.sample_count}, strong_peak={int(summary.strong_peak)}, "
                f"dominant_frequency_hz={summary.dominant_frequency_hz:.6f}, "
                f"oscillation_prominence={summary.oscillation_prominence:.3f}, "
                f"wheel_speed_vs_torque_phase_deg={summary.wheel_speed_vs_torque_phase_deg:+.3f}, "
                f"yaw_vs_rigid_wheel_phase_deg={summary.yaw_vs_rigid_wheel_phase_deg:+.3f}, "
                f"yaw_coupling_magnitude={summary.yaw_coupling_magnitude:.6f}, "
                f"raw_total_inertia_kg_m2={summary.raw_total_inertia_kg_m2:.9f}, "
                f"rigid_body_subtracted_wheel_inertia_kg_m2={summary.rigid_body_subtracted_wheel_inertia_kg_m2:.9f}, "
                f"phase_corrected_wheel_inertia_kg_m2={summary.phase_corrected_wheel_inertia_kg_m2:.9f}, "
                f"raw_damping_nm_per_radps={summary.raw_damping_nm_per_radps:.9f}, "
                f"phase_corrected_damping_nm_per_radps={summary.phase_corrected_damping_nm_per_radps:.9f}"
            )

    for track_width_summary in (yaw_track_width_summary, smooth_track_width_summary):
        if track_width_summary is None:
            continue
        print()
        print(f"{track_width_summary.label} track-width summary")
        print(
            "method=per-maneuver robust encoder-vs-gyro fit on completed phase-11 turn segments, plus an aggregate "
            "bootstrap over maneuver segments so track width remains granular by primitive and speed bin"
        )
        print(
            f"segments={track_width_summary.segment_count}, samples={track_width_summary.sample_count}, "
            f"aggregate_weighted_track_width_m={format_optional_float(track_width_summary.weighted_track_width_m, 6)}, "
            f"aggregate_median_track_width_m={format_optional_float(track_width_summary.robust_track_width_m, 6)}, "
            f"bootstrap_l05_m={format_optional_float(track_width_summary.bootstrap_l05_m, 6)}, "
            f"bootstrap_l50_m={format_optional_float(track_width_summary.bootstrap_l50_m, 6)}, "
            f"bootstrap_l95_m={format_optional_float(track_width_summary.bootstrap_l95_m, 6)}, "
            f"aggregate_predicted_vs_gyro_corr={format_optional_float(track_width_summary.predicted_vs_gyro_correlation, 4)}"
        )
        if track_width_summary.track_width_distribution is not None:
            print_distribution_summary(
                f"{track_width_summary.label.lower()}_per_sample_track_width_m",
                track_width_summary.track_width_distribution,
                "m",
            )
        for segment_summary in track_width_summary.segment_summaries:
            print(
                f"primitive={segment_summary.primitive_name}, repeat={segment_summary.repeat_index}, "
                f"direction={segment_summary.direction_name}, speed_bin={segment_summary.speed_bin}, "
                f"samples={segment_summary.sample_count}, "
                f"weighted_track_width_m={format_optional_float(segment_summary.weighted_track_width_m, 6)}, "
                f"median_track_width_m={format_optional_float(segment_summary.robust_track_width_m, 6)}, "
                f"predicted_vs_gyro_corr={format_optional_float(segment_summary.predicted_vs_gyro_correlation, 4)}"
            )
            if segment_summary.track_width_distribution is not None:
                print_distribution_summary(
                    f"{segment_summary.primitive_name}_repeat_{segment_summary.repeat_index}_track_width_m",
                    segment_summary.track_width_distribution,
                    "m",
                )

    if signal_lag_summary is not None:
        print()
        print("Encoder-gyro lag summary")
        print(
            "method=bounded cross-correlation over rotational segments; the reported lag is the integer-millisecond shift "
            "that maximizes encoder-vs-gyro yaw-rate correlation"
        )
        print(
            f"label={signal_lag_summary.label}, samples={signal_lag_summary.sample_count}, "
            f"best_lag_samples={signal_lag_summary.best_lag_samples}, "
            f"best_lag_ms={format_optional_float(signal_lag_summary.best_lag_ms, 3)}, "
            f"zero_lag_corr={format_optional_float(signal_lag_summary.zero_lag_correlation, 4)}, "
            f"peak_corr={format_optional_float(signal_lag_summary.peak_correlation, 4)}"
        )

    if yaw_decay_summary is not None:
        print()
        print("Yaw free-decay summary")
        print(
            "method=fit an exponential decay to phase-12 corrected gyro tails after completed yaw/smooth turns; "
            "convert the fitted time constant into an equivalent first-order yaw-rate damping using the configured yaw inertia"
        )
        print(f"segments={yaw_decay_summary.segment_count}")
        if yaw_decay_summary.decay_time_constant_distribution is not None:
            print_distribution_summary(
                "yaw_decay_time_constant_s",
                yaw_decay_summary.decay_time_constant_distribution,
                "s",
            )
        if yaw_decay_summary.damping_distribution is not None:
            print_distribution_summary(
                "equivalent_yaw_rate_damping_nms_per_rad",
                yaw_decay_summary.damping_distribution,
                "Nms/rad",
            )
        for segment_summary in yaw_decay_summary.segment_summaries:
            print(
                f"label={segment_summary.label}, segment_id={segment_summary.segment_id}, "
                f"samples={segment_summary.sample_count}, initial_abs_gyro_radps={segment_summary.initial_abs_gyro_radps:.6f}, "
                f"tau_s={segment_summary.decay_time_constant_s:.6f}, "
                f"equivalent_yaw_rate_damping_nms_per_rad={format_optional_float(segment_summary.equivalent_yaw_rate_damping_nms_per_rad, 9)}, "
                f"fit_r2={segment_summary.fit_r2:.4f}"
            )

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

    if sensor_timestamp_summary is not None:
        print()
        print("Sensor timestamp age summary")
        print(
            "method=measure master_time_us minus the logged encoder and IMU timestamps on the trimmed dataset to expose "
            "effective sensor age presented to the estimator"
        )
        print(f"rows={sensor_timestamp_summary.row_count}")
        if sensor_timestamp_summary.encoder_age_us is not None:
            print_tail_stats("encoder_age_us", sensor_timestamp_summary.encoder_age_us, "us")
        if sensor_timestamp_summary.imu_age_us is not None:
            print_tail_stats("imu_age_us", sensor_timestamp_summary.imu_age_us, "us")

    if args.timing is not None:
        timing, timing_breakdown = analyze_timing_csv(
            args.timing,
            max_control_tick_sequence=args.max_control_tick_sequence,
            max_master_time_us=args.max_master_time_us,
        )
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
        if timing_breakdown is not None:
            print("Timing breakdown summary")
            print(
                "method=decompose the timing rows into control, encoder, IMU, and wall-sensor stage latencies so "
                "timing constants are visible beyond the coarse predict/update buckets"
            )
            if timing_breakdown.control_duration_us is not None:
                print_tail_stats("control_duration_us", timing_breakdown.control_duration_us, "us")
            if timing_breakdown.encoder_window_us is not None:
                print_tail_stats("encoder_window_us", timing_breakdown.encoder_window_us, "us")
            if timing_breakdown.encoder_to_predict_us is not None:
                print_tail_stats("encoder_to_predict_us", timing_breakdown.encoder_to_predict_us, "us")
            if timing_breakdown.imu_read_duration_us is not None:
                print_tail_stats("imu_read_duration_us", timing_breakdown.imu_read_duration_us, "us")
            if timing_breakdown.imu_drdy_to_read_start_us is not None:
                print_tail_stats("imu_drdy_to_read_start_us", timing_breakdown.imu_drdy_to_read_start_us, "us")
            if timing_breakdown.front_sensor_ready_us is not None:
                print_tail_stats("front_sensor_ready_us", timing_breakdown.front_sensor_ready_us, "us")
            if timing_breakdown.left_sensor_ready_us is not None:
                print_tail_stats("left_sensor_ready_us", timing_breakdown.left_sensor_ready_us, "us")
            if timing_breakdown.right_sensor_ready_us is not None:
                print_tail_stats("right_sensor_ready_us", timing_breakdown.right_sensor_ready_us, "us")
            if timing_breakdown.sensor_ready_skew_us is not None:
                print_tail_stats("sensor_ready_skew_us", timing_breakdown.sensor_ready_skew_us, "us")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
