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
    gyro_bias: ScalarStats
    gyro_corrected: ScalarStats
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
) -> tuple[
    StationarySummary,
    list[LaunchMagnitudeSummary],
    RepeatabilitySummary,
    dict[str, float],
]:
    static_dt_seconds = 0.0
    static_gyro_raw: list[float] = []
    static_gyro_bias: list[float] = []
    static_gyro_corrected: list[float] = []
    static_accel_body_x: list[float] = []
    static_accel_body_y: list[float] = []
    static_planar_accel: list[float] = []

    launch_active_sample_index_by_repeat: dict[int, int] = {}
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

    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            section_id = int(row["section_id"])
            primitive_id = int(row["primitive_id"])

            if section_id == STATIC_SECTION_ID and primitive_id == STATIC_PRIMITIVE_ID:
                static_dt_seconds += 1.0e-6 * int(row["dt_us"])
                static_gyro_raw.append(float(row["gyro_raw_radps"]))
                static_gyro_bias.append(float(row["gyro_bias_radps"]))
                static_gyro_corrected.append(float(row["gyro_radps"]))
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

    stationary_summary = StationarySummary(
        row_count=len(static_gyro_corrected),
        duration_seconds=static_dt_seconds,
        gyro_raw=scalar_stats(static_gyro_raw),
        gyro_bias=scalar_stats(static_gyro_bias),
        gyro_corrected=scalar_stats(static_gyro_corrected),
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
        "imu_yaw_sigma_radps": stationary_summary.gyro_corrected.sigma,
        "imu_accel_sigma_mps2_conservative": max(
            stationary_summary.accel_body_x.sigma,
            stationary_summary.accel_body_y.sigma,
        ),
        "encoder_linear_sigma_mps": repeatability_summary.linear_sigma_mps,
        "encoder_yaw_sigma_radps": repeatability_summary.yaw_sigma_radps,
    }

    return stationary_summary, launch_summaries, repeatability_summary, suggestions


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


def main() -> int:
    args = parse_args()
    if not args.main.is_file():
        print(f"error: main CSV not found: {args.main}", file=sys.stderr)
        return 1
    if args.timing is not None and not args.timing.is_file():
        print(f"error: timing CSV not found: {args.timing}", file=sys.stderr)
        return 1

    stationary, launch_summaries, repeatability, suggestions = analyze_main_csv(args.main)

    print(f"Open-floor main CSV: {args.main}")
    print()
    print("Static hold summary")
    print(
        f"rows={stationary.row_count}, duration_s={stationary.duration_seconds:.3f}, "
        f"section={SECTION_NAMES[STATIC_SECTION_ID]}, primitive={PRIMITIVE_NAMES[STATIC_PRIMITIVE_ID]}"
    )
    print_scalar_summary("gyro_raw", stationary.gyro_raw, "rad/s")
    print_scalar_summary("gyro_bias", stationary.gyro_bias, "rad/s")
    print_scalar_summary("gyro_corrected", stationary.gyro_corrected, "rad/s")
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
