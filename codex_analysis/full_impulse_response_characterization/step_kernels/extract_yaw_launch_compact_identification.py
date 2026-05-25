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


REPO_ROOT = Path(__file__).resolve().parents[3]
OUT_DIR = Path(__file__).resolve().parent
DEFAULT_LOG = REPO_ROOT / "TestResults" / "mmlog_decode_2026-05-04_20-35-47" / "open_floor_main.csv"
DEFAULT_SIDECAR = DEFAULT_LOG.with_suffix(".sidecar")
PHASE_STATIC = 1
PHASE_YAW_LAUNCH = 20
PRE_SAMPLES = 150
RECOVERY_SAMPLES = 300
PRIMARY_RESPONSE_LAG_SAMPLES = 4
SANITY_RESPONSE_LAG_SAMPLES = 5
VALIDATION_REPEATS = {3, 7, 10}
PROVISIONAL_FILTER_CUTOFF_HZ = 160.0

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from codex_analysis.contact_continuum_yaw_identification.features import (  # noqa: E402
    extract_contact_continuum_features as prior,
)
from codex_analysis.contact_correction_log_eval import evaluate_contact_correction as replay  # noqa: E402


@dataclass(frozen=True)
class Row:
    row_index: int
    raw: dict[str, str]
    master_time_us: int
    tick: int
    dt_us: int
    phase_id: int
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
    imu_temp: int
    gyro_raw_radps: float


@dataclass(frozen=True)
class Step:
    step_id: int
    phase_start_index: int
    phase_end_index: int
    all_start_index: int
    all_end_index: int
    left_command: float
    right_command: float
    yaw_command_proxy: float
    amplitude: float
    direction_sign: int
    direction: str
    repeat_index: int
    speed_bin: float


@dataclass
class StepMetrics:
    step_id: int
    amplitude: float
    direction: str
    direction_sign: int
    repeat_index: int
    split: str
    quality_class: str
    pre_filtered_std_radps: float
    pre_highpass_std_radps: float
    pre_encoder_max_mps: float
    command_encoder_max_mps: float
    initial_accel_radps2: float
    peak_accel_radps2: float
    peak_yaw_rate_radps: float
    peak_sample: int
    steady_yaw_rate_radps: float
    command_area_rad: float
    t63_ms: float
    rise_10_90_ms: float
    recovery_tau37_ms: float
    recovery_last50_radps: float
    response_delay_ms: float
    imu_temp_mean: float
    quality_flags: str


def finite_float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    text = row.get(key, "")
    if text == "":
        return default
    try:
        value = float(text)
    except ValueError:
        return default
    return value if math.isfinite(value) else default


def finite_int(row: dict[str, str], key: str, default: int = 0) -> int:
    text = row.get(key, "")
    if text == "":
        return default
    try:
        return int(float(text))
    except ValueError:
        return default


def mean(values: Iterable[float]) -> float:
    data = list(values)
    return statistics.fmean(data) if data else float("nan")


def median(values: Iterable[float]) -> float:
    data = list(values)
    return statistics.median(data) if data else float("nan")


def percentile(values: Iterable[float], pct: float) -> float:
    data = sorted(values)
    if not data:
        return float("nan")
    if len(data) == 1:
        return data[0]
    rank = (len(data) - 1) * pct
    lo = math.floor(rank)
    hi = math.ceil(rank)
    frac = rank - lo
    return (data[lo] * (1.0 - frac)) + (data[hi] * frac)


def pstdev(values: Iterable[float]) -> float:
    data = list(values)
    return statistics.pstdev(data) if len(data) > 1 else 0.0


def rmse(values: Iterable[float]) -> float:
    data = list(values)
    return math.sqrt(statistics.fmean(v * v for v in data)) if data else float("nan")


def slope_by_x(pairs: list[tuple[float, float]]) -> float:
    pairs = [(x, y) for x, y in pairs if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 2:
        return float("nan")
    mx = statistics.fmean(x for x, _ in pairs)
    my = statistics.fmean(y for _, y in pairs)
    den = sum((x - mx) * (x - mx) for x, _ in pairs)
    if den <= 0.0:
        return float("nan")
    return sum((x - mx) * (y - my) for x, y in pairs) / den


def first_sustained_crossing(values: list[float], threshold: float, run_length: int = 3) -> int | None:
    for index in range(0, max(0, len(values) - run_length + 1)):
        if all(value >= threshold for value in values[index : index + run_length]):
            return index
    return None


def first_fraction_time(values: list[float], threshold: float) -> float:
    for index in range(1, len(values)):
        y0 = values[index - 1]
        y1 = values[index]
        if (y0 < threshold <= y1) and y1 != y0:
            return (index - 1) + ((threshold - y0) / (y1 - y0))
    return float("nan")


def write_csv(path: Path, rows: list[dict[str, str]], fieldnames: list[str] | None = None) -> None:
    if fieldnames is None:
        fieldnames = list(rows[0].keys()) if rows else ["empty"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def read_rows(path: Path) -> list[Row]:
    rows: list[Row] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        for index, raw in enumerate(csv.DictReader(handle)):
            rows.append(
                Row(
                    row_index=index,
                    raw=raw,
                    master_time_us=finite_int(raw, "master_time_us"),
                    tick=finite_int(raw, "control_tick_sequence", index),
                    dt_us=finite_int(raw, "dt_us", 1000),
                    phase_id=finite_int(raw, "phase_id"),
                    speed_bin=finite_float(raw, "speed_bin"),
                    repeat_index=finite_int(raw, "repeat_index"),
                    mode_flags=finite_int(raw, "mode_flags"),
                    saturation_flags=finite_int(raw, "saturation_flags"),
                    imu_status=finite_int(raw, "imu_status"),
                    left_command=finite_float(raw, "left_drive_command"),
                    right_command=finite_float(raw, "right_drive_command"),
                    left_feedforward=finite_float(raw, "left_feedforward_command"),
                    right_feedforward=finite_float(raw, "right_feedforward_command"),
                    left_feedback=finite_float(raw, "left_feedback_command"),
                    right_feedback=finite_float(raw, "right_feedback_command"),
                    left_target_velocity_mps=finite_float(raw, "left_target_velocity_mps"),
                    right_target_velocity_mps=finite_float(raw, "right_target_velocity_mps"),
                    left_encoder_velocity_mps=finite_float(raw, "left_encoder_velocity_mps"),
                    right_encoder_velocity_mps=finite_float(raw, "right_encoder_velocity_mps"),
                    left_encoder_omega_radps=finite_float(raw, "left_encoder_omega_radps"),
                    right_encoder_omega_radps=finite_float(raw, "right_encoder_omega_radps"),
                    encoder_timestamp_us=finite_int(raw, "encoder_timestamp_us"),
                    imu_timestamp_us=finite_int(raw, "imu_timestamp_us"),
                    imu_temp=finite_int(raw, "imu_temp"),
                    gyro_raw_radps=finite_float(raw, "gyro_raw_radps"),
                )
            )
    return rows


def parse_sidecar(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key] = value
    return values


def yaw_proxy(row: Row) -> float:
    return 0.5 * (row.left_command - row.right_command)


def zero_phase_lowpass(values: list[float], cutoff_hz: float, sample_hz: float) -> list[float]:
    if not values:
        return []
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    dt = 1.0 / sample_hz
    alpha = dt / (rc + dt)
    forward = [values[0]]
    for value in values[1:]:
        forward.append(forward[-1] + alpha * (value - forward[-1]))
    backward = [0.0] * len(values)
    backward[-1] = forward[-1]
    for index in range(len(values) - 2, -1, -1):
        backward[index] = backward[index + 1] + alpha * (forward[index] - backward[index + 1])
    return backward


def detect_steps(phase_rows: list[Row]) -> list[Step]:
    steps: list[Step] = []
    index = 0
    while index < len(phase_rows):
        command = yaw_proxy(phase_rows[index])
        if abs(command) < 0.05:
            index += 1
            continue
        start = index
        sign = 1 if command > 0.0 else -1
        amplitude = round(abs(command), 2)
        index += 1
        while index < len(phase_rows):
            next_command = yaw_proxy(phase_rows[index])
            if abs(next_command) < 0.05:
                break
            if (1 if next_command > 0.0 else -1) != sign:
                break
            if abs(abs(next_command) - amplitude) > 0.03:
                break
            index += 1
        end = index
        first = phase_rows[start]
        last = phase_rows[end - 1]
        steps.append(
            Step(
                step_id=len(steps) + 1,
                phase_start_index=start,
                phase_end_index=end,
                all_start_index=first.row_index,
                all_end_index=last.row_index + 1,
                left_command=first.left_command,
                right_command=first.right_command,
                yaw_command_proxy=command,
                amplitude=amplitude,
                direction_sign=sign,
                direction="CW" if sign > 0 else "CCW",
                repeat_index=first.repeat_index,
                speed_bin=first.speed_bin,
            )
        )
        index = max(index + 1, end + 1)
    return steps


def stationary_pre_indices(rows: list[Row], start_index: int) -> list[int]:
    indices: list[int] = []
    index = start_index - 1
    while index >= 0 and len(indices) < PRE_SAMPLES:
        row = rows[index]
        stationary_command = abs(row.left_command) < 0.02 and abs(row.right_command) < 0.02
        stationary_encoder = max(abs(row.left_encoder_velocity_mps), abs(row.right_encoder_velocity_mps)) < 0.03
        if stationary_command and stationary_encoder:
            indices.append(index)
        elif indices:
            break
        index -= 1
    indices.reverse()
    return indices


def trace_offsets(step: Step, rows: list[Row]) -> list[int]:
    start = max(0, step.all_start_index - PRE_SAMPLES)
    end = min(len(rows), step.all_end_index + RECOVERY_SAMPLES)
    return list(range(start, end))


def quality_class(amplitude: float, steady: float, peak: float) -> str:
    if amplitude <= 0.55 and steady < 0.15:
        return "twitch_static_bristle"
    if amplitude == 0.60 or steady < 1.0:
        return "transitional"
    return "sustained_launch"


def normalized_row(row: Row) -> prior.NormalizedRow:
    raw = dict(row.raw)
    raw.setdefault("fan_duty_cycle", "0.8")
    return prior.normalize_open_floor(raw, row.row_index)


def build_step_outputs(
    rows: list[Row],
    phase_rows: list[Row],
    steps: list[Step],
    corrected: list[float],
    filtered: list[float],
) -> tuple[list[StepMetrics], list[dict[str, str]], dict[int, dict[int, float]], dict[int, float]]:
    metrics: list[StepMetrics] = []
    trace_rows: list[dict[str, str]] = []
    step_unit_response: dict[int, dict[int, float]] = {}
    step_baseline: dict[int, float] = {}

    for step in steps:
        pre_indices = stationary_pre_indices(rows, step.all_start_index)
        baseline_indices = pre_indices if len(pre_indices) >= 40 else list(range(max(0, step.all_start_index - PRE_SAMPLES), step.all_start_index))
        pre_filtered = [filtered[index] for index in baseline_indices]
        pre_corrected = [corrected[index] for index in baseline_indices]
        pre_highpass = [corrected[index] - filtered[index] for index in baseline_indices]
        baseline = mean(pre_filtered)
        step_baseline[step.step_id] = baseline
        command_indices = list(range(step.all_start_index, step.all_end_index))
        on_values = [step.direction_sign * (filtered[index] - baseline) for index in command_indices]
        steady = mean(on_values[-50:]) if len(on_values) >= 50 else mean(on_values)
        peak = max(on_values) if on_values else float("nan")
        peak_sample = on_values.index(peak) if on_values else -1
        quality = quality_class(step.amplitude, steady, peak)
        split = "validation" if step.repeat_index in VALIDATION_REPEATS else "train"

        accel = [(on_values[i + 1] - on_values[i]) * 1000.0 for i in range(max(0, len(on_values) - 1))]
        initial_slice = []
        for offset in range(PRIMARY_RESPONSE_LAG_SAMPLES, min(len(on_values) - 1, PRIMARY_RESPONSE_LAG_SAMPLES + 6)):
            initial_slice.append((on_values[offset + 1] - on_values[offset]) * 1000.0)
        initial_accel = mean(initial_slice)
        peak_accel = max(accel[:80]) if accel else float("nan")
        command_area = sum(on_values) * 0.001
        threshold = max(0.05, 4.0 * pstdev(pre_filtered), 4.0 * pstdev(pre_highpass))
        crossing = first_sustained_crossing(on_values[:80], threshold, run_length=3)
        delay_ms = float(crossing) if crossing is not None else float("nan")

        t63 = float("nan")
        rise = float("nan")
        if steady > 0.15:
            t63 = first_fraction_time(on_values, 0.632 * steady)
            t10 = first_fraction_time(on_values, 0.10 * steady)
            t90 = first_fraction_time(on_values, 0.90 * steady)
            if math.isfinite(t10) and math.isfinite(t90) and t90 >= t10:
                rise = t90 - t10

        recovery_values = [
            step.direction_sign * (filtered[index] - baseline)
            for index in range(step.all_end_index, min(len(rows), step.all_end_index + RECOVERY_SAMPLES))
        ]
        recovery_tau = float("nan")
        if recovery_values and abs(on_values[-1]) > 0.05:
            target = 0.37 * on_values[-1]
            for offset, value in enumerate(recovery_values):
                if value <= target:
                    recovery_tau = float(offset)
                    break
        recovery_last = mean(recovery_values[-50:]) if len(recovery_values) >= 50 else mean(recovery_values)

        pre_encoder_max = max(
            [max(abs(rows[index].left_encoder_velocity_mps), abs(rows[index].right_encoder_velocity_mps)) for index in baseline_indices]
            or [0.0]
        )
        command_encoder_max = max(
            [max(abs(rows[index].left_encoder_velocity_mps), abs(rows[index].right_encoder_velocity_mps)) for index in command_indices]
            or [0.0]
        )
        flags: list[str] = []
        if len(pre_indices) < 80:
            flags.append("short_stationary_prebaseline")
        if quality == "twitch_static_bristle":
            flags.append("twitch_only")
        if quality == "sustained_launch":
            flags.append("sustained")
        if pstdev(pre_highpass) > 0.020:
            flags.append("high_pre_vibration")
        if command_encoder_max > 0.15:
            flags.append("wheel_motion")

        metrics.append(
            StepMetrics(
                step_id=step.step_id,
                amplitude=step.amplitude,
                direction=step.direction,
                direction_sign=step.direction_sign,
                repeat_index=step.repeat_index,
                split=split,
                quality_class=quality,
                pre_filtered_std_radps=pstdev(pre_filtered),
                pre_highpass_std_radps=pstdev(pre_highpass),
                pre_encoder_max_mps=pre_encoder_max,
                command_encoder_max_mps=command_encoder_max,
                initial_accel_radps2=initial_accel,
                peak_accel_radps2=peak_accel,
                peak_yaw_rate_radps=peak,
                peak_sample=peak_sample,
                steady_yaw_rate_radps=steady,
                command_area_rad=command_area,
                t63_ms=t63,
                rise_10_90_ms=rise,
                recovery_tau37_ms=recovery_tau,
                recovery_last50_radps=recovery_last,
                response_delay_ms=delay_ms,
                imu_temp_mean=mean(rows[index].imu_temp for index in command_indices),
                quality_flags=";".join(flags),
            )
        )

        unit_by_offset: dict[int, float] = {}
        for index in trace_offsets(step, rows):
            offset = index - step.all_start_index
            row = rows[index]
            command_phase = "pre" if offset < 0 else ("on" if index < step.all_end_index else "recovery")
            direction_delta = step.direction_sign * (filtered[index] - baseline)
            per_unit = direction_delta / step.amplitude if step.amplitude > 0.0 else 0.0
            unit_by_offset[offset] = per_unit
            trace_rows.append(
                {
                    "step_id": str(step.step_id),
                    "sample_offset": str(offset),
                    "time_ms": f"{offset:.3f}",
                    "command_phase": command_phase,
                    "split": split,
                    "amplitude": f"{step.amplitude:.2f}",
                    "direction": step.direction,
                    "direction_sign": str(step.direction_sign),
                    "repeat_index": str(step.repeat_index),
                    "quality_class": quality,
                    "left_command": f"{row.left_command:.9f}",
                    "right_command": f"{row.right_command:.9f}",
                    "gyro_raw_minus_bias_radps": f"{corrected[index]:.9f}",
                    "gyro_filtered_minus_bias_radps": f"{filtered[index]:.9f}",
                    "gyro_filter_removed_radps": f"{corrected[index] - filtered[index]:.9f}",
                    "direction_normalized_yaw_delta_radps": f"{direction_delta:.9f}",
                    "direction_normalized_per_unit_command_radps": f"{per_unit:.9f}",
                    "left_encoder_velocity_mps": f"{row.left_encoder_velocity_mps:.9f}",
                    "right_encoder_velocity_mps": f"{row.right_encoder_velocity_mps:.9f}",
                    "imu_temp": str(row.imu_temp),
                }
            )
        step_unit_response[step.step_id] = unit_by_offset

    return metrics, trace_rows, step_unit_response, step_baseline


def summarize_conditions(metrics: list[StepMetrics]) -> list[dict[str, str]]:
    groups: dict[tuple[str, str, str], list[StepMetrics]] = defaultdict(list)
    combined_groups: dict[tuple[str, str], list[StepMetrics]] = defaultdict(list)
    for item in metrics:
        groups[(f"{item.amplitude:.2f}", item.direction, item.quality_class)].append(item)
        combined_groups[(f"{item.amplitude:.2f}", item.quality_class)].append(item)

    rows: list[dict[str, str]] = []
    for (amp, direction, quality), group in sorted(groups.items()):
        rows.append(condition_row(amp, direction, quality, "signed", group))
    for (amp, quality), group in sorted(combined_groups.items()):
        directions = {item.direction for item in group}
        if directions == {"CW", "CCW"}:
            rows.append(condition_row(amp, "COMBINED_DIRECTION_NORMALIZED", quality, "combined", group))
    return rows


def condition_row(amp: str, direction: str, quality: str, scope: str, group: list[StepMetrics]) -> dict[str, str]:
    steady = [item.steady_yaw_rate_radps for item in group]
    peak = [item.peak_yaw_rate_radps for item in group]
    initial = [item.initial_accel_radps2 for item in group]
    repeat_slope = slope_by_x([(item.repeat_index, item.steady_yaw_rate_radps) for item in group])
    noise_slope = slope_by_x([(item.pre_highpass_std_radps, item.steady_yaw_rate_radps) for item in group])
    return {
        "amplitude": amp,
        "direction": direction,
        "quality_class": quality,
        "scope": scope,
        "steps": str(len(group)),
        "train_steps": str(sum(1 for item in group if item.split == "train")),
        "validation_steps": str(sum(1 for item in group if item.split == "validation")),
        "median_delay_ms": f"{median(item.response_delay_ms for item in group):.3f}",
        "mean_initial_accel_radps2": f"{mean(initial):.6f}",
        "median_initial_accel_radps2": f"{median(initial):.6f}",
        "mean_peak_accel_radps2": f"{mean(item.peak_accel_radps2 for item in group):.6f}",
        "mean_peak_yaw_rate_radps": f"{mean(peak):.6f}",
        "median_peak_yaw_rate_radps": f"{median(peak):.6f}",
        "mean_steady_yaw_rate_radps": f"{mean(steady):.6f}",
        "median_steady_yaw_rate_radps": f"{median(steady):.6f}",
        "p10_steady_yaw_rate_radps": f"{percentile(steady, 0.10):.6f}",
        "p90_steady_yaw_rate_radps": f"{percentile(steady, 0.90):.6f}",
        "median_command_area_rad": f"{median(item.command_area_rad for item in group):.6f}",
        "median_t63_ms": f"{median(item.t63_ms for item in group):.3f}",
        "median_rise_10_90_ms": f"{median(item.rise_10_90_ms for item in group):.3f}",
        "median_recovery_tau37_ms": f"{median(item.recovery_tau37_ms for item in group):.3f}",
        "median_recovery_last50_radps": f"{median(item.recovery_last50_radps for item in group):.6f}",
        "mean_pre_highpass_std_radps": f"{mean(item.pre_highpass_std_radps for item in group):.6f}",
        "mean_command_encoder_max_mps": f"{mean(item.command_encoder_max_mps for item in group):.6f}",
        "steady_vs_repeat_slope_radps_per_repeat": f"{repeat_slope:.9f}",
        "steady_vs_pre_vibration_slope": f"{noise_slope:.9f}",
    }


def build_kernels(
    metrics: list[StepMetrics],
    step_unit_response: dict[int, dict[int, float]],
    train_only: bool,
) -> list[dict[str, str]]:
    groups: dict[tuple[str, str, str, str], list[StepMetrics]] = defaultdict(list)
    for item in metrics:
        if train_only and item.split != "train":
            continue
        groups[(f"{item.amplitude:.2f}", item.direction, item.quality_class, "signed")].append(item)
        groups[(f"{item.amplitude:.2f}", "COMBINED_DIRECTION_NORMALIZED", item.quality_class, "combined")].append(item)

    rows: list[dict[str, str]] = []
    for (amp, direction, quality, scope), group in sorted(groups.items()):
        offsets = sorted(set().union(*(set(step_unit_response[item.step_id].keys()) for item in group)))
        for offset in offsets:
            values = [step_unit_response[item.step_id][offset] for item in group if offset in step_unit_response[item.step_id]]
            if not values:
                continue
            rows.append(
                {
                    "amplitude": amp,
                    "direction": direction,
                    "quality_class": quality,
                    "scope": scope,
                    "sample_offset": str(offset),
                    "time_ms": f"{offset:.3f}",
                    "steps": str(len(values)),
                    "mean_direction_normalized_per_unit_command_radps": f"{mean(values):.9f}",
                    "median_direction_normalized_per_unit_command_radps": f"{median(values):.9f}",
                    "p10_direction_normalized_per_unit_command_radps": f"{percentile(values, 0.10):.9f}",
                    "p90_direction_normalized_per_unit_command_radps": f"{percentile(values, 0.90):.9f}",
                }
            )
    return rows


def kernel_lookup(kernel_rows: list[dict[str, str]]) -> dict[tuple[str, str, str], dict[int, float]]:
    lookup: dict[tuple[str, str, str], dict[int, float]] = defaultdict(dict)
    for row in kernel_rows:
        if row["scope"] != "signed":
            continue
        key = (row["amplitude"], row["direction"], row["quality_class"])
        lookup[key][int(row["sample_offset"])] = float(row["mean_direction_normalized_per_unit_command_radps"])
    return lookup


def prediction_upper_bound(
    rows: list[Row],
    phase_rows: list[Row],
    steps: list[Step],
    metrics: list[StepMetrics],
    filtered: list[float],
    train_kernel_rows: list[dict[str, str]],
) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    params = replay.source_params()
    denom = replay.yaw_denominator_kg_m2(params)
    lookup = kernel_lookup(train_kernel_rows)
    metrics_by_step = {item.step_id: item for item in metrics}
    baseline_by_group: dict[tuple[str, str, str], list[float]] = defaultdict(list)
    empirical_by_group: dict[tuple[str, str, str], list[float]] = defaultdict(list)
    no_response_by_group: dict[tuple[str, str, str], list[float]] = defaultdict(list)
    sample_rows: list[dict[str, str]] = []

    for step in steps:
        item = metrics_by_step[step.step_id]
        if item.split != "validation":
            continue
        key = (f"{step.amplitude:.2f}", step.direction, item.quality_class)
        kernel = lookup.get(key)
        if not kernel:
            continue
        max_offset = step.phase_end_index - step.phase_start_index - PRIMARY_RESPONSE_LAG_SAMPLES - 1
        for command_offset in range(0, max_offset):
            command_phase_index = step.phase_start_index + command_offset
            command_row = phase_rows[command_phase_index]
            target_start = rows[command_row.row_index + PRIMARY_RESPONSE_LAG_SAMPLES]
            target_end = rows[command_row.row_index + PRIMARY_RESPONSE_LAG_SAMPLES + 1]
            dt_s = target_end.dt_us * 1.0e-6
            if dt_s <= 0.0:
                continue
            target_start_yaw = filtered[target_start.row_index]
            target_end_yaw = filtered[target_end.row_index]
            model_state_yaw = filtered[command_row.row_index]
            forward_velocity = 0.5 * (command_row.left_encoder_velocity_mps + command_row.right_encoder_velocity_mps)
            model_result = replay.model_result(
                normalized_row(command_row),
                forward_velocity,
                model_state_yaw,
                params,
                include_contact_correction=True,
            )
            baseline_pred = target_start_yaw + ((model_result.model_yaw_moment_nm / denom) * dt_s)
            empirical_delta = None
            response_offset = command_offset + PRIMARY_RESPONSE_LAG_SAMPLES
            if response_offset in kernel and (response_offset + 1) in kernel:
                empirical_delta = step.direction_sign * step.amplitude * (kernel[response_offset + 1] - kernel[response_offset])
            if empirical_delta is None:
                continue
            empirical_pred = target_start_yaw + empirical_delta
            no_response_pred = target_start_yaw
            baseline_error = baseline_pred - target_end_yaw
            empirical_error = empirical_pred - target_end_yaw
            no_response_error = no_response_pred - target_end_yaw
            baseline_by_group[key].append(baseline_error)
            empirical_by_group[key].append(empirical_error)
            no_response_by_group[key].append(no_response_error)
            if len(sample_rows) < 80 and command_offset in {0, 4, 5, 10, 30, 100, 250, 340}:
                sample_rows.append(
                    {
                        "step_id": str(step.step_id),
                        "sample_offset": str(command_offset),
                        "amplitude": f"{step.amplitude:.2f}",
                        "direction": step.direction,
                        "quality_class": item.quality_class,
                        "target_start_yaw_filtered_radps": f"{target_start_yaw:.9f}",
                        "target_end_yaw_filtered_radps": f"{target_end_yaw:.9f}",
                        "current_plantmodel_pred_end_radps": f"{baseline_pred:.9f}",
                        "empirical_condition_response_pred_end_radps": f"{empirical_pred:.9f}",
                        "no_response_pred_end_radps": f"{no_response_pred:.9f}",
                        "current_plantmodel_error_radps": f"{baseline_error:.9f}",
                        "empirical_condition_response_error_radps": f"{empirical_error:.9f}",
                    }
                )

    summary: list[dict[str, str]] = []
    for key in sorted(baseline_by_group):
        baseline_errors = baseline_by_group[key]
        empirical_errors = empirical_by_group[key]
        no_response_errors = no_response_by_group[key]
        baseline_rmse = rmse(baseline_errors)
        empirical_rmse = rmse(empirical_errors)
        no_response_rmse = rmse(no_response_errors)
        summary.append(
            {
                "amplitude": key[0],
                "direction": key[1],
                "quality_class": key[2],
                "split": "validation",
                "samples": str(len(baseline_errors)),
                "response_lag_samples": str(PRIMARY_RESPONSE_LAG_SAMPLES),
                "no_response_derivative_rmse_radps": f"{no_response_rmse:.9f}",
                "current_plantmodel_derivative_rmse_radps": f"{baseline_rmse:.9f}",
                "empirical_condition_response_derivative_rmse_radps": f"{empirical_rmse:.9f}",
                "rmse_reduction_vs_current_plantmodel_percent": f"{(1.0 - (empirical_rmse / baseline_rmse)) * 100.0 if baseline_rmse > 0.0 else 0.0:.6f}",
                "variance_explained_vs_current_plantmodel_percent": f"{(1.0 - ((empirical_rmse * empirical_rmse) / (baseline_rmse * baseline_rmse))) * 100.0 if baseline_rmse > 0.0 else 0.0:.6f}",
            }
        )
    if summary:
        all_baseline = [err for values in baseline_by_group.values() for err in values]
        all_empirical = [err for values in empirical_by_group.values() for err in values]
        all_no_response = [err for values in no_response_by_group.values() for err in values]
        baseline_rmse = rmse(all_baseline)
        empirical_rmse = rmse(all_empirical)
        no_response_rmse = rmse(all_no_response)
        summary.append(
            {
                "amplitude": "ALL",
                "direction": "ALL",
                "quality_class": "ALL",
                "split": "validation",
                "samples": str(len(all_baseline)),
                "response_lag_samples": str(PRIMARY_RESPONSE_LAG_SAMPLES),
                "no_response_derivative_rmse_radps": f"{no_response_rmse:.9f}",
                "current_plantmodel_derivative_rmse_radps": f"{baseline_rmse:.9f}",
                "empirical_condition_response_derivative_rmse_radps": f"{empirical_rmse:.9f}",
                "rmse_reduction_vs_current_plantmodel_percent": f"{(1.0 - (empirical_rmse / baseline_rmse)) * 100.0 if baseline_rmse > 0.0 else 0.0:.6f}",
                "variance_explained_vs_current_plantmodel_percent": f"{(1.0 - ((empirical_rmse * empirical_rmse) / (baseline_rmse * baseline_rmse))) * 100.0 if baseline_rmse > 0.0 else 0.0:.6f}",
            }
        )
    return summary, sample_rows


def estimate_breakaway(summary_rows: list[dict[str, str]], direction: str) -> float:
    points: list[tuple[float, float]] = []
    for row in summary_rows:
        if row["scope"] != "signed" or row["direction"] != direction:
            continue
        points.append((float(row["amplitude"]), float(row["median_steady_yaw_rate_radps"])))
    points.sort()
    previous: tuple[float, float] | None = None
    for point in points:
        if previous is not None and previous[1] < 1.0 <= point[1] and point[1] != previous[1]:
            frac = (1.0 - previous[1]) / (point[1] - previous[1])
            return previous[0] + frac * (point[0] - previous[0])
        previous = point
    return float("nan")


def compact_parameter_rows(
    metrics: list[StepMetrics],
    condition_rows: list[dict[str, str]],
    prediction_rows: list[dict[str, str]],
    yaw_denominator: float,
) -> list[dict[str, str]]:
    sustained = [item for item in metrics if item.quality_class == "sustained_launch"]
    twitch = [item for item in metrics if item.quality_class == "twitch_static_bristle"]
    transitional = [item for item in metrics if item.quality_class == "transitional"]
    cw_breakaway = estimate_breakaway(condition_rows, "CW")
    ccw_breakaway = estimate_breakaway(condition_rows, "CCW")
    all_prediction = next((row for row in prediction_rows if row["amplitude"] == "ALL"), None)
    condition_by_key = {
        (row["amplitude"], row["direction"], row["quality_class"]): row
        for row in condition_rows
        if row["scope"] == "signed"
    }

    def steady_ratio(amp: str, quality: str) -> float:
        cw = condition_by_key.get((amp, "CW", quality))
        ccw = condition_by_key.get((amp, "CCW", quality))
        if cw is None or ccw is None:
            return float("nan")
        ccw_value = float(ccw["median_steady_yaw_rate_radps"])
        if abs(ccw_value) <= 1.0e-9:
            return float("nan")
        return float(cw["median_steady_yaw_rate_radps"]) / ccw_value

    rows = [
        {
            "compact_model_term": "input_to_gyro_timing",
            "inferred_range": f"primary {PRIMARY_RESPONSE_LAG_SAMPLES} ms; derivative/onset sanity {SANITY_RESPONSE_LAG_SAMPLES} ms",
            "evidence": "Prior measured alignment plus this filtered replay; same/+1/+2 samples remain baseline and first consistent rise is +4.",
            "plantmodel_implication": "Use a compact command/torque or measurement-effective-time delay term; do not amplitude-schedule the delay except for explicit twitch-vs-sustained diagnostics.",
        },
        {
            "compact_model_term": "motor_torque_buildup",
            "inferred_range": f"initial yaw accel median {median(item.initial_accel_radps2 for item in metrics):.1f} rad/s^2; sustained median {median(item.initial_accel_radps2 for item in sustained):.1f} rad/s^2; effective yaw moment {yaw_denominator * median(item.initial_accel_radps2 for item in sustained):.9f} Nm",
            "evidence": "Initial acceleration is measured from filtered gyro at +4..+10 ms after command onset.",
            "plantmodel_implication": "Represent as a short torque/current rise or launch torque lag before the contact model sees full bank torque.",
        },
        {
            "compact_model_term": "static_bristle_twitch_resistance",
            "inferred_range": f"0.50/0.55 commands: peak yaw {median(item.peak_yaw_rate_radps for item in twitch):.3f} rad/s, steady {median(item.steady_yaw_rate_radps for item in twitch):.3f} rad/s, command-area {median(item.command_area_rad for item in twitch):.4f} rad",
            "evidence": "Twitch-only pulses move initially but decay/arrest while command remains on.",
            "plantmodel_implication": "Add low-speed bristle/static displacement or launch-resistance state/term rather than a scalar gain update.",
        },
        {
            "compact_model_term": "breakaway_sustained_resistance",
            "inferred_range": f"CW breakaway near command {cw_breakaway:.3f}; CCW breakaway near command {ccw_breakaway:.3f}; transitional samples {len(transitional)}",
            "evidence": "0.60 is transitional; 0.65/0.70 mostly sustain, with sign-dependent strength.",
            "plantmodel_implication": "Use a smooth breakaway/resistance curve over command-derived torque and low wheel speed, not a hard mode branch.",
        },
        {
            "compact_model_term": "damping_relaxation",
            "inferred_range": f"sustained t63 median {median(item.t63_ms for item in sustained):.1f} ms; recovery tau37 median {median(item.recovery_tau37_ms for item in sustained):.1f} ms",
            "evidence": "Rise and post-command decay are computed from filtered, direction-normalized yaw-rate traces.",
            "plantmodel_implication": "Fit yaw/contact damping and relaxation so the model explains both rise and off-recovery, not only steady yaw rate.",
        },
        {
            "compact_model_term": "amplitude_sign_asymmetry",
            "inferred_range": (
                f"steady CW/CCW ratio: 0.60 transitional {steady_ratio('0.60', 'transitional'):.2f}, "
                f"0.65 sustained {steady_ratio('0.65', 'sustained_launch'):.2f}, "
                f"0.70 sustained {steady_ratio('0.70', 'sustained_launch'):.2f}"
            ),
            "evidence": "The same measurement sequence alternates command sign at each amplitude/repeat.",
            "plantmodel_implication": "Before adding a yaw table, test left/right torque scale, static threshold, and contact-force asymmetry parameters.",
        },
    ]
    if all_prediction is not None:
        no_response_rmse = float(all_prediction["no_response_derivative_rmse_radps"])
        empirical_rmse = float(all_prediction["empirical_condition_response_derivative_rmse_radps"])
        no_response_reduction = (1.0 - (empirical_rmse / no_response_rmse)) * 100.0 if no_response_rmse > 0.0 else 0.0
        no_response_variance = (
            (1.0 - ((empirical_rmse * empirical_rmse) / (no_response_rmse * no_response_rmse))) * 100.0
            if no_response_rmse > 0.0
            else 0.0
        )
        rows.append(
            {
                "compact_model_term": "deterministic_error_upper_bound",
                "inferred_range": (
                    f"validation empirical response derivative RMSE {all_prediction['empirical_condition_response_derivative_rmse_radps']} rad/s "
                    f"vs current PlantModel {all_prediction['current_plantmodel_derivative_rmse_radps']} rad/s; "
                    f"vs no-response {all_prediction['no_response_derivative_rmse_radps']} rad/s; "
                    f"variance explained vs current PlantModel {all_prediction['variance_explained_vs_current_plantmodel_percent']}%, "
                    f"vs no-response {no_response_variance:.6f}%"
                ),
                "evidence": "Condition responses are trained on repeats not in validation and evaluated one-step with the known +4 sample lag.",
                "plantmodel_implication": (
                    "A compact model that captures the same deterministic response structure plausibly removes a large share of yaw-launch prediction error. "
                    f"The current PlantModel mirror overpredicts these windows enough that even no-response is closer; the empirical response still improves no-response RMSE by {no_response_reduction:.6f}%."
                ),
            }
        )
    return rows


def metrics_csv_rows(metrics: list[StepMetrics]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for item in metrics:
        rows.append(
            {
                "step_id": str(item.step_id),
                "amplitude": f"{item.amplitude:.2f}",
                "direction": item.direction,
                "direction_sign": str(item.direction_sign),
                "repeat_index": str(item.repeat_index),
                "split": item.split,
                "quality_class": item.quality_class,
                "pre_filtered_std_radps": f"{item.pre_filtered_std_radps:.9f}",
                "pre_highpass_std_radps": f"{item.pre_highpass_std_radps:.9f}",
                "pre_encoder_max_mps": f"{item.pre_encoder_max_mps:.9f}",
                "command_encoder_max_mps": f"{item.command_encoder_max_mps:.9f}",
                "response_delay_ms": f"{item.response_delay_ms:.3f}",
                "initial_accel_radps2": f"{item.initial_accel_radps2:.9f}",
                "peak_accel_radps2": f"{item.peak_accel_radps2:.9f}",
                "peak_yaw_rate_radps": f"{item.peak_yaw_rate_radps:.9f}",
                "peak_sample": str(item.peak_sample),
                "steady_yaw_rate_radps": f"{item.steady_yaw_rate_radps:.9f}",
                "command_area_rad": f"{item.command_area_rad:.9f}",
                "t63_ms": f"{item.t63_ms:.3f}",
                "rise_10_90_ms": f"{item.rise_10_90_ms:.3f}",
                "recovery_tau37_ms": f"{item.recovery_tau37_ms:.3f}",
                "recovery_last50_radps": f"{item.recovery_last50_radps:.9f}",
                "imu_temp_mean": f"{item.imu_temp_mean:.3f}",
                "quality_flags": item.quality_flags,
            }
        )
    return rows


def write_report(
    args: argparse.Namespace,
    sidecar: dict[str, str],
    rows: list[Row],
    metrics: list[StepMetrics],
    condition_rows: list[dict[str, str]],
    prediction_rows: list[dict[str, str]],
    parameter_rows: list[dict[str, str]],
    filter_stats: dict[str, float],
) -> None:
    quality_counts = Counter(item.quality_class for item in metrics)
    phase_counts = Counter(row.phase_id for row in rows)
    all_prediction = next((row for row in prediction_rows if row["amplitude"] == "ALL"), None)
    lines: list[str] = [
        "# Yaw Launch Compact Plant Identification Evidence",
        "",
        "## Source And Boundary",
        "",
        f"- Primary log: `{args.log}`",
        f"- Sidecar: `{args.sidecar}`",
        f"- Phase 20 mapping: `{sidecar.get('phase_battery_02_name', 'unknown')}`",
        f"- Fan duty metadata: `{sidecar.get('fan_duty_cycle', 'unknown')}`",
        f"- Rows read: `{len(rows)}`; phase counts: `{dict(sorted(phase_counts.items()))}`",
        f"- Yaw-launch command steps: `{len(metrics)}`",
        "- Scope: scratch analysis only. Production code and tests were not modified.",
        "- Targets: raw gyro minus independently estimated static bias, filtered only to remove fan/vibration before identification. UKF targets are not used.",
        "",
        "## Provisional Fan/Vibration Filter",
        "",
        "- No IR-A fan-filter output was present, so this run used a replaceable provisional filter.",
        f"- Filter: forward/backward single-pole low-pass at `{PROVISIONAL_FILTER_CUTOFF_HZ:.1f}` Hz over the bias-removed raw gyro. This is offline zero-phase conditioning, not a runtime proposal.",
        f"- Static bias: `{filter_stats['bias_radps']:.9f}` rad/s from `{int(filter_stats['static_samples'])}` static rows.",
        f"- Static raw std: `{filter_stats['static_raw_std_radps']:.9f}` rad/s; filtered static std: `{filter_stats['static_filtered_std_radps']:.9f}` rad/s; removed high-frequency std: `{filter_stats['static_removed_std_radps']:.9f}` rad/s.",
        "",
        "## Measurement Protocol Structure",
        "",
        f"- Known timing: primary command-to-first-gyro-response alignment is `+{PRIMARY_RESPONSE_LAG_SAMPLES}` samples; derivative/onset sanity is `+{SANITY_RESPONSE_LAG_SAMPLES}` samples.",
        f"- Quality classes: `{dict(quality_counts)}`.",
        "- Train/validation split: repeats 3, 7, and 10 are validation; all other repeats train the evidence kernels.",
        "- Amplitudes/signs are preserved. Combined-direction kernels are direction-normalized evidence only and are not presented as runtime tables.",
        "",
        "## Compact PlantModel Interpretation",
        "",
    ]
    for row in parameter_rows:
        lines.extend(
            [
                f"### {row['compact_model_term']}",
                "",
                f"- Inferred range: {row['inferred_range']}",
                f"- Evidence: {row['evidence']}",
                f"- PlantModel implication: {row['plantmodel_implication']}",
                "",
            ]
        )

    lines.extend(
        [
            "## Condition Summary",
            "",
            "| Amp | Direction | Quality | Steps | Initial accel | Peak yaw | Steady yaw | t63 ms | Recovery tau37 ms |",
            "| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in condition_rows:
        if row["scope"] != "signed":
            continue
        lines.append(
            f"| {row['amplitude']} | {row['direction']} | {row['quality_class']} | {row['steps']} | "
            f"{row['median_initial_accel_radps2']} | {row['median_peak_yaw_rate_radps']} | "
            f"{row['median_steady_yaw_rate_radps']} | {row['median_t63_ms']} | {row['median_recovery_tau37_ms']} |"
        )

    lines.extend(["", "## Prediction Error Upper Bound", ""])
    if all_prediction is not None:
        lines.extend(
            [
                (
                    f"- Validation one-step derivative RMSE using current PlantModel mirror: "
                    f"`{all_prediction['current_plantmodel_derivative_rmse_radps']}` rad/s."
                ),
                (
                    f"- Validation one-step derivative RMSE using no-response derivative baseline: "
                    f"`{all_prediction['no_response_derivative_rmse_radps']}` rad/s."
                ),
                (
                    f"- Validation one-step derivative RMSE using held-out condition response evidence: "
                    f"`{all_prediction['empirical_condition_response_derivative_rmse_radps']}` rad/s."
                ),
                (
                    f"- Upper-bound RMSE reduction: `{all_prediction['rmse_reduction_vs_current_plantmodel_percent']}%`; "
                    f"variance explained: `{all_prediction['variance_explained_vs_current_plantmodel_percent']}%`."
                ),
                "- Interpretation: this measures deterministic structure available to a compact physical model. It is not a recommendation to deploy an impulse response.",
            ]
        )
    else:
        lines.append("- No validation prediction rows were generated.")

    lines.extend(
        [
            "",
            "## Artifacts",
            "",
            "- `per_step_aligned_filtered_traces.csv`: aligned evidence traces, including raw-minus-bias, filtered gyro, removed vibration, encoders, commands, and quality class.",
            "- `per_step_response_metrics.csv`: per-step compact-response metrics.",
            "- `condition_summary_metrics.csv`: per-amplitude/sign/quality aggregates plus repeat/noise trend slopes.",
            "- `per_condition_step_kernels.csv`: averaged direction-normalized response evidence by condition.",
            "- `train_condition_step_kernels.csv`: training-only averaged evidence used for validation prediction.",
            "- `prediction_error_upper_bound.csv`: current PlantModel mirror vs held-out empirical response one-step error estimates.",
            "- `compact_model_parameter_ranges.csv`: compact PlantModel terms and inferred parameter ranges.",
            "",
            "## Reproduce",
            "",
            "```powershell",
            "python codex_analysis\\full_impulse_response_characterization\\step_kernels\\extract_yaw_launch_compact_identification.py",
            "```",
            "",
        ]
    )
    (OUT_DIR / "yaw_launch_compact_identification_report.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Extract yaw-launch step response evidence for compact PlantModel identification.")
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--sidecar", type=Path, default=DEFAULT_SIDECAR)
    args = parser.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = read_rows(args.log)
    sidecar = parse_sidecar(args.sidecar)
    static_values = [row.gyro_raw_radps for row in rows if row.phase_id == PHASE_STATIC]
    bias = mean(static_values)
    corrected = [row.gyro_raw_radps - bias for row in rows]
    sample_hz = 1000.0
    filtered = zero_phase_lowpass(corrected, PROVISIONAL_FILTER_CUTOFF_HZ, sample_hz)
    static_indices = [index for index, row in enumerate(rows) if row.phase_id == PHASE_STATIC]
    filter_stats = {
        "bias_radps": bias,
        "static_samples": float(len(static_indices)),
        "static_raw_std_radps": pstdev(corrected[index] for index in static_indices),
        "static_filtered_std_radps": pstdev(filtered[index] for index in static_indices),
        "static_removed_std_radps": pstdev(corrected[index] - filtered[index] for index in static_indices),
    }

    phase_rows = [row for row in rows if row.phase_id == PHASE_YAW_LAUNCH]
    steps = detect_steps(phase_rows)
    metrics, trace_rows, step_unit_response, _ = build_step_outputs(rows, phase_rows, steps, corrected, filtered)
    condition_rows = summarize_conditions(metrics)
    all_kernel_rows = build_kernels(metrics, step_unit_response, train_only=False)
    train_kernel_rows = build_kernels(metrics, step_unit_response, train_only=True)
    prediction_rows, prediction_samples = prediction_upper_bound(rows, phase_rows, steps, metrics, filtered, train_kernel_rows)
    params = replay.source_params()
    parameter_rows = compact_parameter_rows(metrics, condition_rows, prediction_rows, replay.yaw_denominator_kg_m2(params))

    write_csv(OUT_DIR / "per_step_aligned_filtered_traces.csv", trace_rows)
    write_csv(OUT_DIR / "per_step_response_metrics.csv", metrics_csv_rows(metrics))
    write_csv(OUT_DIR / "condition_summary_metrics.csv", condition_rows)
    write_csv(OUT_DIR / "per_condition_step_kernels.csv", all_kernel_rows)
    write_csv(OUT_DIR / "train_condition_step_kernels.csv", train_kernel_rows)
    write_csv(OUT_DIR / "prediction_error_upper_bound.csv", prediction_rows)
    write_csv(OUT_DIR / "prediction_error_samples.csv", prediction_samples)
    write_csv(OUT_DIR / "compact_model_parameter_ranges.csv", parameter_rows)
    write_csv(
        OUT_DIR / "filter_summary.csv",
        [
            {
                "filter_status": "provisional_replaceable_no_IR_A_output_found",
                "cutoff_hz": f"{PROVISIONAL_FILTER_CUTOFF_HZ:.3f}",
                "static_bias_radps": f"{filter_stats['bias_radps']:.9f}",
                "static_samples": str(int(filter_stats["static_samples"])),
                "static_raw_std_radps": f"{filter_stats['static_raw_std_radps']:.9f}",
                "static_filtered_std_radps": f"{filter_stats['static_filtered_std_radps']:.9f}",
                "static_removed_high_frequency_std_radps": f"{filter_stats['static_removed_std_radps']:.9f}",
            }
        ],
    )
    commands = [
        "Get-Content -LiteralPath AGENTS.md",
        "Get-Content -LiteralPath TestResults\\mmlog_decode_2026-05-04_20-35-47\\open_floor_main.sidecar",
        "Get-Content -LiteralPath codex_analysis\\yaw_launch_step_response\\yaw_launch_delay_calibration_report.md",
        "Get-Content -LiteralPath codex_analysis\\yaw_launch_step_response\\yaw_launch_step_response_report.md",
        "Get-Content -LiteralPath micromouse_ukf_plant_measurement_noise_theory_only_spec.md",
        "python codex_analysis\\full_impulse_response_characterization\\step_kernels\\extract_yaw_launch_compact_identification.py",
    ]
    write_csv(OUT_DIR / "commands_run.csv", [{"command": command} for command in commands])
    write_report(args, sidecar, rows, metrics, condition_rows, prediction_rows, parameter_rows, filter_stats)
    print(f"Wrote yaw-launch compact identification artifacts to {OUT_DIR}")
    print(f"Steps={len(metrics)} quality={dict(Counter(item.quality_class for item in metrics))}")
    if prediction_rows:
        print(prediction_rows[-1])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
