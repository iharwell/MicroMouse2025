#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from codex_analysis.contact_continuum_yaw_identification.features import (  # noqa: E402
    extract_contact_continuum_features as prior,
)
from codex_analysis.contact_correction_log_eval import evaluate_contact_correction as replay  # noqa: E402


DEFAULT_LOG = REPO_ROOT / "TestResults" / "mmlog_decode_2026-05-04_20-35-47" / "open_floor_main.csv"
DEFAULT_SIDECAR = DEFAULT_LOG.with_suffix(".sidecar")
PHASE_YAW_LAUNCH = 20
PRE_WINDOW_SAMPLES = 150
MIN_CLEAN_PRE_SAMPLES = 80
BIAS_PHASE_ID = 1
TUNED_GAIN = -6.496619190


@dataclass(frozen=True)
class Row:
    row_index: int
    raw: dict[str, str]
    master_time_us: int
    tick: int
    dt_us: int
    phase_id: int
    primitive_id: str
    speed_bin: float
    repeat_index: int
    mode_flags: int
    saturation_flags: int
    imu_status: int
    left_command: float
    right_command: float
    left_feedforward: float
    right_feedforward: float
    left_feedback: float
    right_feedback: float
    left_target_velocity_mps: float
    right_target_velocity_mps: float
    left_encoder_velocity_mps: float
    right_encoder_velocity_mps: float
    left_encoder_omega_radps: float
    right_encoder_omega_radps: float
    encoder_timestamp_us: int
    imu_timestamp_us: int
    gyro_raw_radps: float
    gyro_logged_bias_radps: float
    gyro_logged_radps: float


@dataclass(frozen=True)
class StepWindow:
    step_id: int
    command_start_index: int
    command_end_index: int
    pre_start_index: int
    post_end_index: int
    left_command: float
    right_command: float
    command_yaw_proxy: float
    amplitude: float
    direction_sign: int
    direction_label: str
    repeat_index: int
    speed_bin: float


def f(row: dict[str, str], key: str, default: float = 0.0) -> float:
    text = row.get(key, "")
    if text == "":
        return default
    try:
        value = float(text)
    except ValueError:
        return default
    return value if math.isfinite(value) else default


def i(row: dict[str, str], key: str, default: int = 0) -> int:
    text = row.get(key, "")
    if text == "":
        return default
    try:
        return int(float(text))
    except ValueError:
        return default


def read_rows(path: Path) -> list[Row]:
    rows: list[Row] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        for index, raw in enumerate(csv.DictReader(handle)):
            rows.append(
                Row(
                    row_index=index,
                    raw=raw,
                    master_time_us=i(raw, "master_time_us"),
                    tick=i(raw, "control_tick_sequence", index),
                    dt_us=i(raw, "dt_us", 1000),
                    phase_id=i(raw, "phase_id"),
                    primitive_id=raw.get("primitive_id", ""),
                    speed_bin=f(raw, "speed_bin"),
                    repeat_index=i(raw, "repeat_index"),
                    mode_flags=i(raw, "mode_flags"),
                    saturation_flags=i(raw, "saturation_flags"),
                    imu_status=i(raw, "imu_status"),
                    left_command=f(raw, "left_drive_command"),
                    right_command=f(raw, "right_drive_command"),
                    left_feedforward=f(raw, "left_feedforward_command"),
                    right_feedforward=f(raw, "right_feedforward_command"),
                    left_feedback=f(raw, "left_feedback_command"),
                    right_feedback=f(raw, "right_feedback_command"),
                    left_target_velocity_mps=f(raw, "left_target_velocity_mps"),
                    right_target_velocity_mps=f(raw, "right_target_velocity_mps"),
                    left_encoder_velocity_mps=f(raw, "left_encoder_velocity_mps"),
                    right_encoder_velocity_mps=f(raw, "right_encoder_velocity_mps"),
                    left_encoder_omega_radps=f(raw, "left_encoder_omega_radps"),
                    right_encoder_omega_radps=f(raw, "right_encoder_omega_radps"),
                    encoder_timestamp_us=i(raw, "encoder_timestamp_us"),
                    imu_timestamp_us=i(raw, "imu_timestamp_us"),
                    gyro_raw_radps=f(raw, "gyro_raw_radps"),
                    gyro_logged_bias_radps=f(raw, "gyro_bias_radps"),
                    gyro_logged_radps=f(raw, "gyro_radps"),
                )
            )
    return rows


def mean(values: Iterable[float]) -> float:
    materialized = list(values)
    return statistics.fmean(materialized) if materialized else float("nan")


def pstdev(values: Iterable[float]) -> float:
    materialized = list(values)
    return statistics.pstdev(materialized) if len(materialized) > 1 else 0.0


def median(values: Iterable[float]) -> float:
    materialized = list(values)
    return statistics.median(materialized) if materialized else float("nan")


def percentile(values: Iterable[float], pct: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * pct
    lo = math.floor(rank)
    hi = math.ceil(rank)
    if lo == hi:
        return ordered[lo]
    frac = rank - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def rmse(values: Iterable[float]) -> float:
    materialized = list(values)
    return math.sqrt(statistics.fmean(value * value for value in materialized)) if materialized else 0.0


def estimate_independent_stationary_bias(rows: list[Row]) -> tuple[float, int, float, float]:
    stationary = [
        row.gyro_raw_radps
        for row in rows
        if row.phase_id == BIAS_PHASE_ID
        and abs(row.left_command) <= 1.0e-6
        and abs(row.right_command) <= 1.0e-6
        and abs(row.left_encoder_velocity_mps) <= 0.02
        and abs(row.right_encoder_velocity_mps) <= 0.02
    ]
    return mean(stationary), len(stationary), median(stationary), pstdev(stationary)


def find_yaw_launch_steps(phase_rows: list[Row]) -> list[StepWindow]:
    steps: list[StepWindow] = []
    index = 0
    while index < len(phase_rows):
        row = phase_rows[index]
        if abs(row.left_command) <= 1.0e-6 and abs(row.right_command) <= 1.0e-6:
            index += 1
            continue
        start = index
        left_command = row.left_command
        right_command = row.right_command
        while (
            index < len(phase_rows)
            and abs(phase_rows[index].left_command - left_command) <= 1.0e-6
            and abs(phase_rows[index].right_command - right_command) <= 1.0e-6
        ):
            index += 1
        end = index
        command_yaw_proxy = 0.5 * (left_command - right_command)
        direction_sign = 1 if command_yaw_proxy >= 0.0 else -1
        steps.append(
            StepWindow(
                step_id=len(steps) + 1,
                command_start_index=start,
                command_end_index=end,
                pre_start_index=max(0, start - PRE_WINDOW_SAMPLES),
                post_end_index=min(len(phase_rows), end + 150),
                left_command=left_command,
                right_command=right_command,
                command_yaw_proxy=command_yaw_proxy,
                amplitude=abs(command_yaw_proxy),
                direction_sign=direction_sign,
                direction_label="CW" if direction_sign > 0 else "CCW",
                repeat_index=row.repeat_index,
                speed_bin=row.speed_bin,
            )
        )
    return steps


def stationary_pre_rows(all_rows: list[Row], command_row: Row) -> tuple[list[Row], int]:
    collected: list[Row] = []
    index = command_row.row_index - 1
    while index >= 0:
        row = all_rows[index]
        if abs(row.left_command) > 1.0e-6 or abs(row.right_command) > 1.0e-6:
            break
        if abs(row.left_encoder_velocity_mps) > 0.08 or abs(row.right_encoder_velocity_mps) > 0.08:
            break
        collected.append(row)
        index -= 1
    collected.reverse()
    return collected[-PRE_WINDOW_SAMPLES:], len(collected)


def linear_slope(y_values: list[float], start_sample: int, end_sample: int, sample_period_s: float) -> float:
    values = y_values[start_sample:end_sample]
    if len(values) < 3:
        return float("nan")
    xs = [(start_sample + idx) * sample_period_s for idx in range(len(values))]
    x_mean = statistics.fmean(xs)
    y_mean = statistics.fmean(values)
    denom = sum((x - x_mean) * (x - x_mean) for x in xs)
    return sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, values)) / denom if denom > 0.0 else float("nan")


def first_sustained_crossing(values: list[float], threshold: float, run_length: int = 3) -> int | None:
    for index in range(0, max(0, len(values) - run_length + 1)):
        if all(value >= threshold for value in values[index : index + run_length]):
            return index
    return None


def time_to_fraction(values: list[float], target: float, fraction: float) -> int | None:
    threshold = target * fraction
    if threshold <= 0.0:
        return None
    return first_sustained_crossing(values, threshold, run_length=3)


def normalized_row(row: Row) -> prior.NormalizedRow:
    raw = dict(row.raw)
    raw.setdefault("fan_duty_cycle", "0.8")
    return prior.normalize_open_floor(raw, row.row_index)


def model_prediction_summary(
    all_rows: list[Row],
    phase_rows: list[Row],
    steps: list[StepWindow],
    bias_radps: float,
    response_lag_samples: int,
) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    params = replace(replay.source_params(), contact_yaw_patch_force_gain_ns_per_m=TUNED_GAIN)
    denom = replay.yaw_denominator_kg_m2(params)
    sample_rows: list[dict[str, str]] = []
    by_key: dict[tuple[str, str], list[tuple[float, float]]] = defaultdict(list)

    for step in steps:
        _, available_pre_count = stationary_pre_rows(all_rows, phase_rows[step.command_start_index])
        if available_pre_count < MIN_CLEAN_PRE_SAMPLES:
            continue
        for index in range(step.command_start_index, step.command_end_index - response_lag_samples - 1):
            command = phase_rows[index]
            target_start = phase_rows[index + response_lag_samples]
            target_end = phase_rows[index + response_lag_samples + 1]
            dt_s = target_end.dt_us * 1.0e-6
            if dt_s <= 0.0:
                continue
            yaw_rate = command.gyro_raw_radps - bias_radps
            start_yaw = target_start.gyro_raw_radps - bias_radps
            end_yaw = target_end.gyro_raw_radps - bias_radps
            measured_yaw_accel = (end_yaw - start_yaw) / dt_s
            if not math.isfinite(measured_yaw_accel) or abs(measured_yaw_accel) > 4000.0:
                continue
            forward_velocity = 0.5 * (command.left_encoder_velocity_mps + command.right_encoder_velocity_mps)
            command_normalized = normalized_row(command)
            old_result = replay.model_result(
                command_normalized,
                forward_velocity,
                yaw_rate,
                params,
                include_contact_correction=False,
            )
            tuned_result = replay.model_result(
                command_normalized,
                forward_velocity,
                yaw_rate,
                params,
                include_contact_correction=True,
            )
            old_pred = start_yaw + ((old_result.model_yaw_moment_nm / denom) * dt_s)
            tuned_pred = start_yaw + ((tuned_result.model_yaw_moment_nm / denom) * dt_s)
            old_error = old_pred - end_yaw
            tuned_error = tuned_pred - end_yaw
            key = (f"{step.amplitude:.2f}", step.direction_label)
            by_key[key].append((old_error, tuned_error))
            if len(sample_rows) < 25 and index in {
                step.command_start_index,
                step.command_start_index + 2,
                step.command_start_index + 10,
                step.command_start_index + 50,
                step.command_end_index - 5,
            }:
                sample_rows.append(
                    {
                        "step_id": str(step.step_id),
                        "sample_offset": str(index - step.command_start_index),
                        "response_lag_samples": str(response_lag_samples),
                        "amplitude": f"{step.amplitude:.9f}",
                        "direction": step.direction_label,
                        "yaw_rate_radps": f"{yaw_rate:.9f}",
                        "observed_next_yaw_radps": f"{end_yaw:.9f}",
                        "old_pred_next_yaw_radps": f"{old_pred:.9f}",
                        "tuned_pred_next_yaw_radps": f"{tuned_pred:.9f}",
                        "old_error_radps": f"{old_error:.9f}",
                        "tuned_error_radps": f"{tuned_error:.9f}",
                        "patch_delta_yaw_moment_nm": f"{tuned_result.model_yaw_moment_nm - old_result.model_yaw_moment_nm:.9f}",
                    }
                )

    summary_rows: list[dict[str, str]] = []
    for key in sorted(by_key):
        pairs = by_key[key]
        old_errors = [pair[0] for pair in pairs]
        tuned_errors = [pair[1] for pair in pairs]
        old_rmse = rmse(old_errors)
        tuned_rmse = rmse(tuned_errors)
        summary_rows.append(
            {
                "amplitude": key[0],
                "direction": key[1],
                "samples": str(len(pairs)),
                "response_lag_samples": str(response_lag_samples),
                "old_no_contact_correction_rmse_radps": f"{old_rmse:.9f}",
                "tuned_contact_correction_rmse_radps": f"{tuned_rmse:.9f}",
                "relative_delta_percent": f"{((tuned_rmse / old_rmse) - 1.0) * 100.0 if old_rmse > 0.0 else 0.0:.6f}",
            }
        )
    return summary_rows, sample_rows


def step_metrics(all_rows: list[Row], phase_rows: list[Row], steps: list[StepWindow], bias_radps: float) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for step in steps:
        command_rows = phase_rows[step.command_start_index : step.command_end_index]
        pre_rows, available_pre_count = stationary_pre_rows(all_rows, command_rows[0])
        post_rows = phase_rows[step.command_end_index : step.post_end_index]
        sample_period_s = median(row.dt_us for row in command_rows) * 1.0e-6
        corrected_pre = [row.gyro_raw_radps - bias_radps for row in pre_rows]
        pre_mean = mean(corrected_pre)
        pre_std = pstdev(corrected_pre)
        direction = step.direction_sign
        corrected_command = [row.gyro_raw_radps - bias_radps for row in command_rows]
        baseline_relative = [(value - pre_mean) * direction for value in corrected_command]
        post_relative = [(row.gyro_raw_radps - bias_radps - pre_mean) * direction for row in post_rows]
        baseline_relative_with_post = baseline_relative + post_relative
        steady_window = baseline_relative[-50:] if len(baseline_relative) >= 50 else baseline_relative
        peak_value = max(baseline_relative_with_post) if baseline_relative_with_post else float("nan")
        peak_sample = baseline_relative_with_post.index(peak_value) if baseline_relative_with_post else -1
        steady = mean(steady_window)
        final_50_abs = mean(abs(value) for value in steady_window)
        response_threshold = max(0.05, 5.0 * pre_std)
        delay_sample = first_sustained_crossing(baseline_relative, response_threshold)
        t10 = time_to_fraction(baseline_relative, steady, 0.10)
        t63 = time_to_fraction(baseline_relative, steady, 0.632)
        t90 = time_to_fraction(baseline_relative, steady, 0.90)
        initial_slope = linear_slope(baseline_relative, 2, 26, sample_period_s)
        slope_same = linear_slope(baseline_relative, 0, 24, sample_period_s)
        slope_plus1 = linear_slope(baseline_relative, 1, 25, sample_period_s)
        slope_plus2 = linear_slope(baseline_relative, 2, 26, sample_period_s)
        max_abs_encoder_pre = max(
            [abs(row.left_encoder_velocity_mps) for row in pre_rows]
            + [abs(row.right_encoder_velocity_mps) for row in pre_rows],
            default=0.0,
        )
        max_abs_encoder_cmd = max(
            [abs(row.left_encoder_velocity_mps) for row in command_rows]
            + [abs(row.right_encoder_velocity_mps) for row in command_rows],
            default=0.0,
        )
        command_mode_flags = Counter(row.mode_flags for row in command_rows)
        command_saturation_flags = Counter(row.saturation_flags for row in command_rows)
        command_imu_status = Counter(row.imu_status for row in command_rows)
        stale_rows = sum(
            1
            for row in command_rows
            if (row.master_time_us - row.imu_timestamp_us > 3000) or (row.master_time_us - row.encoder_timestamp_us > 3000)
        )
        quality_flags: list[str] = []
        if available_pre_count < MIN_CLEAN_PRE_SAMPLES:
            quality_flags.append("exclude_short_pre_baseline")
        if abs(pre_mean) > 0.05:
            quality_flags.append("pre_yaw_bias_or_motion")
        if max_abs_encoder_pre > 0.08:
            quality_flags.append("pre_encoder_motion")
        if any(row.saturation_flags != 0 for row in command_rows):
            quality_flags.append("saturation_flag")
        if stale_rows:
            quality_flags.append("stale_sensor_timestamp")
        if final_50_abs < 0.20 and peak_value >= 0.50:
            quality_flags.append("impulse_only_not_sustained")
        if steady >= 1.0:
            quality_flags.append("sustained_launch")
        rows.append(
            {
                "step_id": str(step.step_id),
                "phase_id": str(PHASE_YAW_LAUNCH),
                "repeat_index": str(step.repeat_index),
                "speed_bin": f"{step.speed_bin:.9f}",
                "direction": step.direction_label,
                "direction_sign": str(step.direction_sign),
                "amplitude": f"{step.amplitude:.9f}",
                "command_yaw_proxy": f"{step.command_yaw_proxy:.9f}",
                "left_command": f"{step.left_command:.9f}",
                "right_command": f"{step.right_command:.9f}",
                "start_time_us": str(command_rows[0].master_time_us),
                "end_time_us": str(command_rows[-1].master_time_us),
                "command_samples": str(len(command_rows)),
                "pre_samples": str(len(pre_rows)),
                "available_stationary_pre_samples": str(available_pre_count),
                "post_samples": str(len(post_rows)),
                "pre_corrected_gyro_mean_radps": f"{pre_mean:.9f}",
                "pre_corrected_gyro_std_radps": f"{pre_std:.9f}",
                "response_threshold_radps": f"{response_threshold:.9f}",
                "response_delay_samples": "" if delay_sample is None else str(delay_sample),
                "response_delay_ms": "" if delay_sample is None else f"{delay_sample * sample_period_s * 1000.0:.3f}",
                "initial_yaw_accel_plus2_radps2": f"{initial_slope:.9f}",
                "initial_yaw_accel_same_radps2": f"{slope_same:.9f}",
                "initial_yaw_accel_plus1_radps2": f"{slope_plus1:.9f}",
                "initial_yaw_accel_plus2_check_radps2": f"{slope_plus2:.9f}",
                "peak_directional_yaw_rate_radps": f"{peak_value:.9f}",
                "peak_sample_from_onset": str(peak_sample),
                "steady_last50_directional_yaw_rate_radps": f"{steady:.9f}",
                "rise_time_10_90_ms": ""
                if t10 is None or t90 is None or t90 < t10
                else f"{(t90 - t10) * sample_period_s * 1000.0:.3f}",
                "time_constant_63_ms": "" if t63 is None else f"{t63 * sample_period_s * 1000.0:.3f}",
                "max_abs_encoder_pre_mps": f"{max_abs_encoder_pre:.9f}",
                "max_abs_encoder_cmd_mps": f"{max_abs_encoder_cmd:.9f}",
                "mean_left_encoder_cmd_mps": f"{mean(row.left_encoder_velocity_mps for row in command_rows):.9f}",
                "mean_right_encoder_cmd_mps": f"{mean(row.right_encoder_velocity_mps for row in command_rows):.9f}",
                "mean_left_target_cmd_mps": f"{mean(row.left_target_velocity_mps for row in command_rows):.9f}",
                "mean_right_target_cmd_mps": f"{mean(row.right_target_velocity_mps for row in command_rows):.9f}",
                "mean_left_feedforward": f"{mean(row.left_feedforward for row in command_rows):.9f}",
                "mean_right_feedforward": f"{mean(row.right_feedforward for row in command_rows):.9f}",
                "mean_left_feedback": f"{mean(row.left_feedback for row in command_rows):.9f}",
                "mean_right_feedback": f"{mean(row.right_feedback for row in command_rows):.9f}",
                "stale_sensor_rows": str(stale_rows),
                "mode_flags_counts": ";".join(f"{key}:{value}" for key, value in sorted(command_mode_flags.items())),
                "saturation_flags_counts": ";".join(f"{key}:{value}" for key, value in sorted(command_saturation_flags.items())),
                "imu_status_counts": ";".join(f"{key}:{value}" for key, value in sorted(command_imu_status.items())),
                "quality_flags": ";".join(quality_flags) if quality_flags else "clean",
            }
        )
    return rows


def aggregate_metrics(per_step: list[dict[str, str]]) -> list[dict[str, str]]:
    groups: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for row in per_step:
        if "exclude_short_pre_baseline" in row["quality_flags"]:
            continue
        groups[(f"{float(row['amplitude']):.2f}", row["direction"])].append(row)

    aggregate_rows: list[dict[str, str]] = []
    for key in sorted(groups):
        rows = groups[key]
        steady = [float(row["steady_last50_directional_yaw_rate_radps"]) for row in rows]
        peak = [float(row["peak_directional_yaw_rate_radps"]) for row in rows]
        accel = [float(row["initial_yaw_accel_plus2_radps2"]) for row in rows]
        delay = [float(row["response_delay_ms"]) for row in rows if row["response_delay_ms"] != ""]
        t63 = [float(row["time_constant_63_ms"]) for row in rows if row["time_constant_63_ms"] != ""]
        rise = [float(row["rise_time_10_90_ms"]) for row in rows if row["rise_time_10_90_ms"] != ""]
        impulse_only = sum("impulse_only_not_sustained" in row["quality_flags"] for row in rows)
        sustained = sum("sustained_launch" in row["quality_flags"] for row in rows)
        aggregate_rows.append(
            {
                "amplitude": key[0],
                "direction": key[1],
                "clean_steps": str(len(rows)),
                "sustained_launch_steps": str(sustained),
                "impulse_only_steps": str(impulse_only),
                "median_delay_ms": f"{median(delay):.3f}",
                "mean_initial_yaw_accel_plus2_radps2": f"{mean(accel):.6f}",
                "median_initial_yaw_accel_plus2_radps2": f"{median(accel):.6f}",
                "mean_peak_directional_yaw_rate_radps": f"{mean(peak):.6f}",
                "median_peak_directional_yaw_rate_radps": f"{median(peak):.6f}",
                "mean_steady_last50_directional_yaw_rate_radps": f"{mean(steady):.6f}",
                "median_steady_last50_directional_yaw_rate_radps": f"{median(steady):.6f}",
                "p10_steady_radps": f"{percentile(steady, 0.10):.6f}",
                "p90_steady_radps": f"{percentile(steady, 0.90):.6f}",
                "median_time_constant_63_ms": f"{median(t63):.3f}",
                "median_rise_time_10_90_ms": f"{median(rise):.3f}",
            }
        )
    return aggregate_rows


def alignment_profile(all_rows: list[Row], phase_rows: list[Row], steps: list[StepWindow], bias_radps: float) -> list[dict[str, str]]:
    bins: dict[tuple[str, int], list[float]] = defaultdict(list)
    for step in steps:
        pre_rows, available_pre_count = stationary_pre_rows(all_rows, phase_rows[step.command_start_index])
        if available_pre_count < MIN_CLEAN_PRE_SAMPLES:
            continue
        pre_mean = mean(row.gyro_raw_radps - bias_radps for row in pre_rows)
        for sample_offset in range(0, 16):
            row = phase_rows[step.command_start_index + sample_offset]
            value = (row.gyro_raw_radps - bias_radps - pre_mean) * step.direction_sign
            bins[(f"{step.amplitude:.2f}", sample_offset)].append(value)

    rows: list[dict[str, str]] = []
    for key in sorted(bins):
        values = bins[key]
        rows.append(
            {
                "amplitude": key[0],
                "sample_offset_from_command": str(key[1]),
                "samples": str(len(values)),
                "mean_directional_yaw_delta_radps": f"{mean(values):.9f}",
                "median_directional_yaw_delta_radps": f"{median(values):.9f}",
                "p10_directional_yaw_delta_radps": f"{percentile(values, 0.10):.9f}",
                "p90_directional_yaw_delta_radps": f"{percentile(values, 0.90):.9f}",
            }
        )
    return rows


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    fieldnames = list(rows[0].keys()) if rows else ["empty"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def sidecar_phase_map(path: Path) -> dict[int, str]:
    phase_map: dict[int, str] = {}
    if not path.is_file():
        return phase_map
    pending_name = ""
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "_name=" in line:
            pending_name = line.split("=", 1)[1]
        elif "_phase_id=" in line:
            try:
                phase_id = int(line.split("=", 1)[1])
            except ValueError:
                continue
            if pending_name:
                phase_map[phase_id] = pending_name
                pending_name = ""
    return phase_map


def write_report(
    path: Path,
    log_path: Path,
    sidecar_path: Path,
    rows: list[Row],
    phase_rows: list[Row],
    steps: list[StepWindow],
    per_step: list[dict[str, str]],
    aggregate: list[dict[str, str]],
    alignment: list[dict[str, str]],
    model_summary: list[dict[str, str]],
    bias: tuple[float, int, float, float],
) -> None:
    bias_mean, bias_rows, bias_median, bias_std = bias
    clean_steps = [row for row in per_step if "exclude_short_pre_baseline" not in row["quality_flags"]]
    excluded = [row for row in per_step if "exclude_short_pre_baseline" in row["quality_flags"]]
    exclusion_counts = Counter()
    for row in per_step:
        for flag in row["quality_flags"].split(";"):
            if flag != "clean":
                exclusion_counts[flag] += 1
    phase_map = sidecar_phase_map(sidecar_path)
    phase_counts = Counter(row.phase_id for row in rows)
    saturation_counts = Counter(row.saturation_flags for row in phase_rows)
    mode_counts = Counter(row.mode_flags for row in phase_rows)
    stale_rows = sum(
        1
        for row in phase_rows
        if (row.master_time_us - row.imu_timestamp_us > 3000) or (row.master_time_us - row.encoder_timestamp_us > 3000)
    )
    align_all = defaultdict(list)
    for row in alignment:
        align_all[int(row["sample_offset_from_command"])].append(float(row["mean_directional_yaw_delta_radps"]))
    first_offsets = ", ".join(
        f"+{offset}={mean(values):.4f}" for offset, values in sorted(align_all.items())[:10]
    )
    aggregate_lines = [
        "| Amp | Dir | Steps | Sustained | Impulse-only | Delay ms | Initial accel rad/s^2 | Peak rad/s | Steady rad/s | t63 ms |",
        "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in aggregate:
        aggregate_lines.append(
            "| {amplitude} | {direction} | {clean_steps} | {sustained_launch_steps} | {impulse_only_steps} | "
            "{median_delay_ms} | {mean_initial_yaw_accel_plus2_radps2} | "
            "{mean_peak_directional_yaw_rate_radps} | {mean_steady_last50_directional_yaw_rate_radps} | "
            "{median_time_constant_63_ms} |".format(**row)
        )
    model_lines = [
        "| Amp | Dir | Samples | Old RMSE | Tuned RMSE | Delta |",
        "| ---: | --- | ---: | ---: | ---: | ---: |",
    ]
    for row in model_summary:
        model_lines.append(
            "| {amplitude} | {direction} | {samples} | {old_no_contact_correction_rmse_radps} | "
            "{tuned_contact_correction_rmse_radps} | {relative_delta_percent}% |".format(**row)
        )
    text = "\n".join(
        [
            "# Yaw Launch Step Response Extraction",
            "",
            "## Source",
            "",
            f"- Primary log: `{log_path}`",
            f"- Sidecar: `{sidecar_path}`",
            "- Selection: newest decoded `TestResults/mmlog_decode_*` directory with `open_floor_main.csv`; sidecar maps `phase_id=20` to `Yaw Launch`.",
            "- Raw `D:\\open_floor_main.mmlog` was not decoded because the decoded copy is present and matches the expected current yaw-launch source.",
            f"- Phase map: {dict(sorted(phase_map.items()))}",
            f"- Phase row counts: {dict(sorted(phase_counts.items()))}",
            "",
            "## Bias And Data Quality",
            "",
            f"- Independent stationary gyro bias: mean `{bias_mean:.9f}` rad/s from `{bias_rows}` static phase rows; median `{bias_median:.9f}`, population std `{bias_std:.9f}`.",
            f"- Yaw-launch rows: `{len(phase_rows)}` over `{(phase_rows[-1].master_time_us - phase_rows[0].master_time_us) / 1.0e6:.3f}` s.",
            f"- Command steps found: `{len(steps)}`; clean aggregate steps: `{len(clean_steps)}`; excluded from aggregate: `{len(excluded)}`.",
            f"- Quality flag counts: {dict(sorted(exclusion_counts.items()))}.",
            "- Pre-baselines are gathered from the full log, not just inside phase 20, while requiring zero drive commands and stationary encoder evidence.",
            f"- Phase-level mode flags: {dict(sorted(mode_counts.items()))}; saturation flags: {dict(sorted(saturation_counts.items()))}; stale sensor rows over 3 ms: `{stale_rows}`.",
            "",
            "## Alignment",
            "",
            f"- Direction-normalized yaw-rate delta around onset: {first_offsets}.",
            "- Same/+1/+2 samples remain effectively baseline. The first consistent sensor rise is +4 to +6 samples, so +2 is a better command/model alignment than same-sample but still precedes most of the measured gyro step response.",
            "",
            "## Aggregate Step Metrics",
            "",
            *aggregate_lines,
            "",
            "## Threshold Interpretation",
            "",
            "- The +/-0.50 and +/-0.55 commands create a repeatable short impulse but do not sustain yaw; their last-50-ms steady yaw is near zero.",
            "- +/-0.60 is transitional: it moves more than 0.55 but usually does not meet the 1 rad/s sustained-launch criterion.",
            "- +/-0.65 is the practical launch boundary in these clean steps: CW sustains in all clean repeats, CCW sustains in most but not all repeats.",
            "- This looks less like a single hard command threshold and more like a static-to-dynamic contact/bristle resistance transition: below the boundary the robot can twitch, then contact resistance arrests the yaw while command remains applied.",
            "",
            "## Secondary Model Check",
            "",
            "- Prediction check uses the exact command windows with +2 response alignment, raw gyro minus the independent static bias, encoder wheel speeds, and drive commands.",
            "- Old means the mirrored PlantModel path with contact-yaw correction disabled. Tuned means `kContactYawPatchForceGainNsPerM = -6.496619190`.",
            "",
            *model_lines,
            "",
            "## Outputs",
            "",
            "- `yaw_launch_step_metrics.csv`: one row per command step.",
            "- `yaw_launch_aggregate.csv`: clean aggregate by amplitude and direction.",
            "- `yaw_launch_alignment_profile.csv`: onset samples 0..15 by amplitude.",
            "- `yaw_launch_model_prediction_plus2.csv`: old vs tuned +2 prediction RMSE by amplitude and direction.",
            "- `yaw_launch_model_prediction_samples_plus2.csv`: small sampled prediction audit.",
            "",
            "## Reproduce",
            "",
            "```powershell",
            "python codex_analysis\\yaw_launch_step_response\\extract_yaw_launch_step_response.py",
            "```",
            "",
            "## Recommended Next Use",
            "",
            "- Use these windows as a launch-specific calibration set, not as a general yaw-surface replacement. Fit static launch, bristle/history, and low-speed contact resistance behavior against the step windows before changing broad contact-continuum gains.",
            "- Keep the +2 command/model alignment for production replay comparisons, but treat first-response timing as a separate sensor/plant delay to model or gate because the gyro response visibly rises after +4 samples.",
        ]
    )
    path.write_text(text + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--sidecar", type=Path, default=DEFAULT_SIDECAR)
    args = parser.parse_args()

    rows = read_rows(args.log)
    phase_rows = [row for row in rows if row.phase_id == PHASE_YAW_LAUNCH]
    if not phase_rows:
        raise SystemExit(f"No phase_id={PHASE_YAW_LAUNCH} rows found in {args.log}")
    bias = estimate_independent_stationary_bias(rows)
    steps = find_yaw_launch_steps(phase_rows)
    per_step = step_metrics(rows, phase_rows, steps, bias[0])
    aggregate = aggregate_metrics(per_step)
    alignment = alignment_profile(rows, phase_rows, steps, bias[0])
    model_summary, model_samples = model_prediction_summary(rows, phase_rows, steps, bias[0], response_lag_samples=2)

    write_csv(OUT_DIR / "yaw_launch_step_metrics.csv", per_step)
    write_csv(OUT_DIR / "yaw_launch_aggregate.csv", aggregate)
    write_csv(OUT_DIR / "yaw_launch_alignment_profile.csv", alignment)
    write_csv(OUT_DIR / "yaw_launch_model_prediction_plus2.csv", model_summary)
    write_csv(OUT_DIR / "yaw_launch_model_prediction_samples_plus2.csv", model_samples)
    (OUT_DIR / "commands_run.txt").write_text(
        "python codex_analysis\\yaw_launch_step_response\\extract_yaw_launch_step_response.py\n",
        encoding="utf-8",
    )
    write_report(
        OUT_DIR / "yaw_launch_step_response_report.md",
        args.log,
        args.sidecar,
        rows,
        phase_rows,
        steps,
        per_step,
        aggregate,
        alignment,
        model_summary,
        bias,
    )
    print(f"rows={len(rows)} phase20_rows={len(phase_rows)} steps={len(steps)}")
    print(f"bias_mean_radps={bias[0]:.9f} bias_rows={bias[1]}")
    print(f"wrote={OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
