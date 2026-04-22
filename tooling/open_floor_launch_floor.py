from __future__ import annotations

import statistics
from dataclasses import dataclass

from open_floor_recovery import DistributionSummary
from open_floor_recovery import parse_float
from open_floor_recovery import row_watchdog_flags
from open_floor_recovery import row_dt_seconds
from open_floor_recovery import summarize_distribution


MIN_SUSTAINED_SPEED_THRESHOLD_MPS = 0.010
SPEED_QUANTUM_MULTIPLIER = 3.0
CLEAR_MOTION_MIN_TIME_S = 0.010
EFFECTIVE_FLOOR_MIN_CLEAR_FRACTION = 0.5


@dataclass
class LaunchFloorRepeatSummary:
    repeat_index: int
    abs_command: float
    signed_command: float
    duration_seconds: float
    peak_abs_linear_speed_mps: float
    time_above_sustained_speed_threshold_s: float
    net_signed_encoder_counts: float
    net_signed_pose_drift_mm: float
    peak_signed_inertial_speed_mps: float
    peak_signed_inertial_displacement_mm: float
    clear_motion: bool


@dataclass
class LaunchFloorCommandSummary:
    abs_command: float
    repeat_count: int
    clear_motion_count: int
    clear_motion_fraction: float
    median_peak_abs_linear_speed_mps: float
    median_time_above_sustained_speed_threshold_s: float
    median_net_signed_encoder_counts: float
    median_net_signed_pose_drift_mm: float
    median_peak_signed_inertial_speed_mps: float
    median_peak_signed_inertial_displacement_mm: float


@dataclass
class LaunchFloorSummary:
    speed_quantum_mps: float
    sustained_speed_threshold_mps: float
    clear_motion_min_time_s: float
    backlash_repeat_count: int
    backlash_net_encoder_counts_stats: DistributionSummary | None
    backlash_net_pose_drift_mm_stats: DistributionSummary | None
    backlash_peak_inertial_speed_mps_stats: DistributionSummary | None
    backlash_peak_inertial_displacement_mm_stats: DistributionSummary | None
    observed_clear_breakaway_command: float | None
    effective_launch_floor_command: float | None
    nonmonotonic_clear_motion: bool
    command_summaries: list[LaunchFloorCommandSummary]
    repeat_summaries: list[LaunchFloorRepeatSummary]


def build_active_launch_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    active_rows: list[dict[str, str]] = []
    for row in rows:
        left_command = parse_float(row["left_drive_command"])
        right_command = parse_float(row["right_drive_command"])
        if (
            abs(left_command) <= 1.0e-6 or
            abs(right_command) <= 1.0e-6 or
            abs(left_command - right_command) > 1.0e-6 or
            int(row["saturation_flags"]) != 0 or
            int(row["clipping_flags"]) != 0 or
            row_watchdog_flags(row) != 0
        ):
            continue
        active_rows.append(row)
    return active_rows


def percentile_l95(stats: DistributionSummary | None) -> float:
    return 0.0 if stats is None else stats.l95


def median_of(repeats: list[LaunchFloorRepeatSummary], accessor: str) -> float:
    if not repeats:
        return 0.0
    return statistics.median(getattr(repeat, accessor) for repeat in repeats)


def summarize_launch_floor(
    launch_rows_by_repeat: dict[int, list[dict[str, str]]],
    accel_bias_y_mps2: float,
) -> LaunchFloorSummary | None:
    active_rows_by_repeat: dict[int, list[dict[str, str]]] = {}
    nonzero_speed_samples_mps: list[float] = []
    for repeat_index, rows in launch_rows_by_repeat.items():
        active_rows = build_active_launch_rows(rows)
        if not active_rows:
            continue
        active_rows_by_repeat[repeat_index] = active_rows
        for row in active_rows:
            speed_mps = abs(parse_float(row["measured_linear_speed_mps"]))
            if speed_mps > 1.0e-9:
                nonzero_speed_samples_mps.append(speed_mps)

    if not active_rows_by_repeat:
        return None

    speed_quantum_mps = min(nonzero_speed_samples_mps) if nonzero_speed_samples_mps else 0.0
    sustained_speed_threshold_mps = max(
        MIN_SUSTAINED_SPEED_THRESHOLD_MPS,
        SPEED_QUANTUM_MULTIPLIER * speed_quantum_mps,
    )

    repeat_summaries: list[LaunchFloorRepeatSummary] = []
    for repeat_index, rows in sorted(active_rows_by_repeat.items()):
        signed_command = parse_float(rows[0]["left_drive_command"])
        sign = 1.0 if signed_command >= 0.0 else -1.0
        duration_seconds = 0.0
        peak_abs_linear_speed_mps = 0.0
        time_above_threshold_s = 0.0
        inertial_speed_mps = 0.0
        peak_inertial_speed_mps = 0.0
        inertial_displacement_m = 0.0
        peak_inertial_displacement_m = 0.0
        for row in rows:
            dt_seconds = row_dt_seconds(row)
            duration_seconds += dt_seconds
            measured_linear_speed_mps = abs(parse_float(row["measured_linear_speed_mps"]))
            peak_abs_linear_speed_mps = max(peak_abs_linear_speed_mps, measured_linear_speed_mps)
            if measured_linear_speed_mps > sustained_speed_threshold_mps:
                time_above_threshold_s += dt_seconds
            signed_accel_mps2 = sign * (parse_float(row["accel_body_y_mps2"]) - accel_bias_y_mps2)
            inertial_speed_mps += signed_accel_mps2 * dt_seconds
            peak_inertial_speed_mps = max(peak_inertial_speed_mps, inertial_speed_mps)
            inertial_displacement_m += inertial_speed_mps * dt_seconds
            peak_inertial_displacement_m = max(peak_inertial_displacement_m, inertial_displacement_m)

        first_row = rows[0]
        last_row = rows[-1]
        net_signed_encoder_counts = sign * 0.5 * (
            (int(last_row["left_encoder_count"]) - int(first_row["left_encoder_count"])) +
            (int(last_row["right_encoder_count"]) - int(first_row["right_encoder_count"]))
        )
        net_signed_pose_drift_mm = sign * 1000.0 * (
            parse_float(last_row["ukf_state_py_m"]) - parse_float(first_row["ukf_state_py_m"])
        )
        repeat_summaries.append(
            LaunchFloorRepeatSummary(
                repeat_index=repeat_index,
                abs_command=round(abs(signed_command), 2),
                signed_command=signed_command,
                duration_seconds=duration_seconds,
                peak_abs_linear_speed_mps=peak_abs_linear_speed_mps,
                time_above_sustained_speed_threshold_s=time_above_threshold_s,
                net_signed_encoder_counts=net_signed_encoder_counts,
                net_signed_pose_drift_mm=net_signed_pose_drift_mm,
                peak_signed_inertial_speed_mps=peak_inertial_speed_mps,
                peak_signed_inertial_displacement_mm=1000.0 * peak_inertial_displacement_m,
                clear_motion=False,
            )
        )

    backlash_repeats = [
        repeat
        for repeat in repeat_summaries
        if repeat.time_above_sustained_speed_threshold_s < CLEAR_MOTION_MIN_TIME_S
    ]
    if not backlash_repeats:
        backlash_repeats = list(repeat_summaries)

    backlash_net_encoder_counts_stats = summarize_distribution(
        [repeat.net_signed_encoder_counts for repeat in backlash_repeats]
    )
    backlash_net_pose_drift_mm_stats = summarize_distribution(
        [repeat.net_signed_pose_drift_mm for repeat in backlash_repeats]
    )
    backlash_peak_inertial_speed_mps_stats = summarize_distribution(
        [repeat.peak_signed_inertial_speed_mps for repeat in backlash_repeats]
    )
    backlash_peak_inertial_displacement_mm_stats = summarize_distribution(
        [repeat.peak_signed_inertial_displacement_mm for repeat in backlash_repeats]
    )

    backlash_encoder_counts_p95 = percentile_l95(backlash_net_encoder_counts_stats)
    backlash_pose_drift_p95_mm = percentile_l95(backlash_net_pose_drift_mm_stats)
    backlash_peak_inertial_speed_p95_mps = percentile_l95(backlash_peak_inertial_speed_mps_stats)
    backlash_peak_inertial_displacement_p95_mm = percentile_l95(backlash_peak_inertial_displacement_mm_stats)

    classified_repeats: list[LaunchFloorRepeatSummary] = []
    for repeat in repeat_summaries:
        encoder_or_pose_clear = (
            repeat.net_signed_encoder_counts > backlash_encoder_counts_p95 or
            repeat.net_signed_pose_drift_mm > backlash_pose_drift_p95_mm
        )
        inertial_clear = (
            repeat.peak_signed_inertial_speed_mps > backlash_peak_inertial_speed_p95_mps or
            repeat.peak_signed_inertial_displacement_mm > backlash_peak_inertial_displacement_p95_mm
        )
        clear_motion = (
            repeat.time_above_sustained_speed_threshold_s >= CLEAR_MOTION_MIN_TIME_S and
            encoder_or_pose_clear and
            inertial_clear
        )
        classified_repeats.append(
            LaunchFloorRepeatSummary(
                repeat_index=repeat.repeat_index,
                abs_command=repeat.abs_command,
                signed_command=repeat.signed_command,
                duration_seconds=repeat.duration_seconds,
                peak_abs_linear_speed_mps=repeat.peak_abs_linear_speed_mps,
                time_above_sustained_speed_threshold_s=repeat.time_above_sustained_speed_threshold_s,
                net_signed_encoder_counts=repeat.net_signed_encoder_counts,
                net_signed_pose_drift_mm=repeat.net_signed_pose_drift_mm,
                peak_signed_inertial_speed_mps=repeat.peak_signed_inertial_speed_mps,
                peak_signed_inertial_displacement_mm=repeat.peak_signed_inertial_displacement_mm,
                clear_motion=clear_motion,
            )
        )

    command_summaries: list[LaunchFloorCommandSummary] = []
    for abs_command in sorted({repeat.abs_command for repeat in classified_repeats}):
        command_repeats = [
            repeat
            for repeat in classified_repeats
            if repeat.abs_command == abs_command
        ]
        clear_motion_count = sum(1 for repeat in command_repeats if repeat.clear_motion)
        repeat_count = len(command_repeats)
        command_summaries.append(
            LaunchFloorCommandSummary(
                abs_command=abs_command,
                repeat_count=repeat_count,
                clear_motion_count=clear_motion_count,
                clear_motion_fraction=(clear_motion_count / repeat_count) if repeat_count else 0.0,
                median_peak_abs_linear_speed_mps=median_of(command_repeats, "peak_abs_linear_speed_mps"),
                median_time_above_sustained_speed_threshold_s=median_of(
                    command_repeats,
                    "time_above_sustained_speed_threshold_s",
                ),
                median_net_signed_encoder_counts=median_of(command_repeats, "net_signed_encoder_counts"),
                median_net_signed_pose_drift_mm=median_of(command_repeats, "net_signed_pose_drift_mm"),
                median_peak_signed_inertial_speed_mps=median_of(
                    command_repeats,
                    "peak_signed_inertial_speed_mps",
                ),
                median_peak_signed_inertial_displacement_mm=median_of(
                    command_repeats,
                    "peak_signed_inertial_displacement_mm",
                ),
            )
        )

    observed_clear_breakaway_command = next(
        (
            summary.abs_command
            for summary in command_summaries
            if summary.clear_motion_count > 0
        ),
        None,
    )
    effective_launch_floor_command = next(
        (
            summary.abs_command
            for summary in command_summaries
            if summary.clear_motion_fraction >= EFFECTIVE_FLOOR_MIN_CLEAR_FRACTION
        ),
        None,
    )

    saw_clear_motion = False
    saw_gap_after_clear_motion = False
    nonmonotonic_clear_motion = False
    for summary in command_summaries:
        if summary.clear_motion_count > 0:
            if saw_gap_after_clear_motion:
                nonmonotonic_clear_motion = True
                break
            saw_clear_motion = True
            continue
        if saw_clear_motion:
            saw_gap_after_clear_motion = True

    return LaunchFloorSummary(
        speed_quantum_mps=speed_quantum_mps,
        sustained_speed_threshold_mps=sustained_speed_threshold_mps,
        clear_motion_min_time_s=CLEAR_MOTION_MIN_TIME_S,
        backlash_repeat_count=len(backlash_repeats),
        backlash_net_encoder_counts_stats=backlash_net_encoder_counts_stats,
        backlash_net_pose_drift_mm_stats=backlash_net_pose_drift_mm_stats,
        backlash_peak_inertial_speed_mps_stats=backlash_peak_inertial_speed_mps_stats,
        backlash_peak_inertial_displacement_mm_stats=backlash_peak_inertial_displacement_mm_stats,
        observed_clear_breakaway_command=observed_clear_breakaway_command,
        effective_launch_floor_command=effective_launch_floor_command,
        nonmonotonic_clear_motion=nonmonotonic_clear_motion,
        command_summaries=command_summaries,
        repeat_summaries=classified_repeats,
    )
