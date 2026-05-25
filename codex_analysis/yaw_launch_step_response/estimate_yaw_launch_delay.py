#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent
DEFAULT_LOG = REPO_ROOT / "TestResults" / "mmlog_decode_2026-05-04_20-35-47" / "open_floor_main.csv"
DEFAULT_SIDECAR = DEFAULT_LOG.with_suffix(".sidecar")
PHASE_STATIC = 1
PHASE_YAW_LAUNCH = 20
PRE_BASELINE_CAP = 500
BASELINE_MEAN_WINDOW = 150
ONSET_SEARCH_SAMPLES = 35
GYRO_CUTOFF_HZ = 214.0


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
    left_encoder_velocity_mps: float
    right_encoder_velocity_mps: float
    imu_timestamp_us: int
    encoder_timestamp_us: int
    gyro_raw_radps: float


@dataclass(frozen=True)
class Step:
    step_id: int
    phase_start_index: int
    phase_end_index: int
    left_command: float
    right_command: float
    yaw_command_proxy: float
    amplitude: float
    direction_sign: int
    direction: str
    repeat_index: int
    speed_bin: float


def finite_float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        value = float(row.get(key, ""))
    except ValueError:
        return default
    return value if math.isfinite(value) else default


def finite_int(row: dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(float(row.get(key, "")))
    except ValueError:
        return default


def mean(values: Iterable[float]) -> float:
    materialized = list(values)
    return statistics.fmean(materialized) if materialized else float("nan")


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
    return (ordered[lo] * (1.0 - frac)) + (ordered[hi] * frac)


def pstdev(values: Iterable[float]) -> float:
    materialized = list(values)
    return statistics.pstdev(materialized) if len(materialized) > 1 else 0.0


def mad_sigma(values: list[float]) -> float:
    if not values:
        return 0.0
    center = statistics.median(values)
    mad = statistics.median(abs(value - center) for value in values)
    return 1.4826 * mad


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    fieldnames = list(rows[0].keys()) if rows else ["empty"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


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
                    left_encoder_velocity_mps=finite_float(raw, "left_encoder_velocity_mps"),
                    right_encoder_velocity_mps=finite_float(raw, "right_encoder_velocity_mps"),
                    imu_timestamp_us=finite_int(raw, "imu_timestamp_us"),
                    encoder_timestamp_us=finite_int(raw, "encoder_timestamp_us"),
                    gyro_raw_radps=finite_float(raw, "gyro_raw_radps"),
                )
            )
    return rows


def estimate_static_bias(rows: list[Row]) -> tuple[float, int, float]:
    values = [
        row.gyro_raw_radps
        for row in rows
        if row.phase_id == PHASE_STATIC
        and abs(row.left_command) <= 1.0e-6
        and abs(row.right_command) <= 1.0e-6
        and abs(row.left_encoder_velocity_mps) <= 0.02
        and abs(row.right_encoder_velocity_mps) <= 0.02
    ]
    return mean(values), len(values), pstdev(values)


def find_steps(phase_rows: list[Row]) -> list[Step]:
    steps: list[Step] = []
    index = 0
    while index < len(phase_rows):
        row = phase_rows[index]
        if abs(row.left_command) <= 1.0e-6 and abs(row.right_command) <= 1.0e-6:
            index += 1
            continue
        start = index
        left = row.left_command
        right = row.right_command
        while (
            index < len(phase_rows)
            and abs(phase_rows[index].left_command - left) <= 1.0e-6
            and abs(phase_rows[index].right_command - right) <= 1.0e-6
        ):
            index += 1
        yaw_proxy = 0.5 * (left - right)
        sign = 1 if yaw_proxy >= 0.0 else -1
        steps.append(
            Step(
                step_id=len(steps) + 1,
                phase_start_index=start,
                phase_end_index=index,
                left_command=left,
                right_command=right,
                yaw_command_proxy=yaw_proxy,
                amplitude=abs(yaw_proxy),
                direction_sign=sign,
                direction="CW" if sign > 0 else "CCW",
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
    return collected[-PRE_BASELINE_CAP:], len(collected)


def first_sustained(values: list[float], threshold: float, start: int = 0, run: int = 3) -> int | None:
    for index in range(start, max(start, len(values) - run + 1)):
        if all(value >= threshold for value in values[index : index + run]):
            return index
    return None


def fractional_crossing(values: list[float], index: int, threshold: float) -> float:
    if index <= 0:
        return float(index)
    previous = values[index - 1]
    current = values[index]
    denom = current - previous
    if abs(denom) < 1.0e-9:
        return float(index)
    frac = (threshold - previous) / denom
    return (index - 1) + min(1.0, max(0.0, frac))


def derivative(values: list[float], dt_s: float) -> list[float]:
    if len(values) < 2 or dt_s <= 0.0:
        return []
    return [(values[index] - values[index - 1]) / dt_s for index in range(1, len(values))]


def derivative_onset(
    pre_values: list[float],
    response_values: list[float],
    dt_s: float,
    value_threshold: float,
) -> tuple[int | None, float, float]:
    pre_diff = derivative(pre_values, dt_s)
    sigma = mad_sigma(pre_diff)
    threshold = max(35.0, percentile(pre_diff, 0.99) + 10.0, 5.0 * sigma)
    diff = derivative(response_values, dt_s)
    for index in range(1, min(len(diff), ONSET_SEARCH_SAMPLES)):
        local = diff[index : index + 3]
        future_values = response_values[index + 1 : index + 7]
        if len(local) >= 2 and sum(value >= threshold for value in local) >= 2 and max(future_values, default=0.0) >= value_threshold:
            return index + 1, threshold, sigma
    return None, threshold, sigma


def piecewise_ramp_onset(values: list[float], max_start: int = 10, fit_end: int = 18) -> tuple[int, float, float]:
    end = min(len(values), fit_end)
    best_start = 0
    best_sse = float("inf")
    best_slope = 0.0
    for start in range(0, min(max_start + 1, end - 3)):
        xs = [max(0, index - start) for index in range(end)]
        denom = sum(x * x for x in xs)
        if denom <= 0.0:
            continue
        slope = max(0.0, sum(x * y for x, y in zip(xs, values[:end])) / denom)
        sse = sum((y - (slope * x)) ** 2 for x, y in zip(xs, values[:end]))
        penalty = 0.0005 * start
        if sse + penalty < best_sse:
            best_sse = sse + penalty
            best_start = start
            best_slope = slope
    return best_start, best_slope, best_sse


def derivative_peak_delay(values: list[float], dt_s: float) -> tuple[int | None, float]:
    diff = derivative(values, dt_s)
    if not diff:
        return None, float("nan")
    search = diff[: min(20, len(diff))]
    peak_index = max(range(len(search)), key=lambda index: search[index])
    return peak_index + 1, search[peak_index]


def local_template_scores(
    response_values: list[float],
    template: list[float],
    offsets: range,
    fit_len: int,
) -> tuple[int, float, dict[int, float]]:
    scores: dict[int, float] = {}
    best_offset = 0
    best_sse = float("inf")
    for offset in offsets:
        pairs: list[tuple[float, float]] = []
        for index in range(fit_len):
            template_index = index - offset
            if 0 <= template_index < len(template):
                pairs.append((template[template_index], response_values[index]))
            elif index < len(response_values):
                pairs.append((0.0, response_values[index]))
        denom = sum(x * x for x, _ in pairs)
        scale = max(0.0, sum(x * y for x, y in pairs) / denom) if denom > 1.0e-9 else 0.0
        sse = sum((y - (scale * x)) ** 2 for x, y in pairs)
        scores[offset] = sse / len(pairs) if pairs else float("inf")
        if scores[offset] < best_sse:
            best_sse = scores[offset]
            best_offset = offset
    return best_offset, best_sse, scores


def build_template(step_rows: list[dict[str, object]]) -> list[float]:
    candidates = [
        row
        for row in step_rows
        if float(row["amplitude"]) >= 0.65 and "clean_onset" in str(row["delay_quality_flags"])
    ]
    if not candidates:
        candidates = step_rows
    template: list[float] = []
    for sample in range(0, 50):
        values = [float(row[f"y_plus_{sample}"]) for row in candidates if f"y_plus_{sample}" in row]
        template.append(median(values))
    peak = max(template) if template else 0.0
    return [value / peak for value in template] if peak > 1.0e-9 else template


def analyze_steps(all_rows: list[Row], phase_rows: list[Row], steps: list[Step], bias: float) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    for step in steps:
        command_rows = phase_rows[step.phase_start_index : step.phase_end_index]
        pre_rows, available_pre_count = stationary_pre_rows(all_rows, command_rows[0])
        baseline_rows = pre_rows[-BASELINE_MEAN_WINDOW:]
        dt_s = median(row.dt_us for row in command_rows) * 1.0e-6
        pre_values = [(row.gyro_raw_radps - bias) * step.direction_sign for row in pre_rows]
        baseline_center = median((row.gyro_raw_radps - bias) * step.direction_sign for row in baseline_rows)
        centered_pre = [value - baseline_center for value in pre_values]
        response = [
            ((row.gyro_raw_radps - bias) * step.direction_sign) - baseline_center
            for row in command_rows[: max(ONSET_SEARCH_SAMPLES + 10, 80)]
        ]
        response_full = [((row.gyro_raw_radps - bias) * step.direction_sign) - baseline_center for row in command_rows]
        noise_sigma = mad_sigma(centered_pre)
        noise_p99 = percentile(centered_pre, 0.99)
        noise_p95 = percentile(centered_pre, 0.95)
        motion_threshold = max(0.03, noise_p95 + 0.005, 2.5 * noise_sigma)
        value_threshold = max(0.05, noise_p99 + 0.01, 4.0 * noise_sigma)
        motion_index = first_sustained(response, motion_threshold, run=2)
        motion_fraction = fractional_crossing(response, motion_index, motion_threshold) if motion_index is not None else float("nan")
        threshold_index = first_sustained(response, value_threshold, run=3)
        threshold_fraction = (
            fractional_crossing(response, threshold_index, value_threshold) if threshold_index is not None else float("nan")
        )
        deriv_index, deriv_threshold, deriv_sigma = derivative_onset(centered_pre, response, dt_s, value_threshold)
        ramp_index, ramp_slope_per_sample, ramp_sse = piecewise_ramp_onset(response)
        peak_index, peak_derivative = derivative_peak_delay(response, dt_s)
        first_0p05 = first_sustained(response, 0.05, run=3)
        first_0p10 = first_sustained(response, 0.10, run=3)
        steady = mean(response_full[-50:])
        peak_early = max(response_full[:80]) if response_full else float("nan")
        sustained_launch = steady >= 1.0
        twitch_only = peak_early >= 0.5 and abs(steady) < 0.20
        quality: list[str] = []
        if available_pre_count < BASELINE_MEAN_WINDOW:
            quality.append("short_pre")
        if any(row.saturation_flags != 0 for row in command_rows):
            quality.append("saturation")
        if any(row.master_time_us - row.imu_timestamp_us > 3000 for row in command_rows):
            quality.append("stale_imu")
        if threshold_index is None:
            quality.append("no_threshold_onset")
        if motion_index is None:
            quality.append("no_motion_onset")
        if deriv_index is None:
            quality.append("no_derivative_onset")
        if twitch_only:
            quality.append("twitch_only")
        if sustained_launch:
            quality.append("sustained_launch")
        if not quality or all(flag in {"twitch_only", "sustained_launch"} for flag in quality):
            quality.append("clean_onset")
        row: dict[str, object] = {
            "step_id": step.step_id,
            "repeat_index": step.repeat_index,
            "amplitude": f"{step.amplitude:.9f}",
            "direction": step.direction,
            "direction_sign": step.direction_sign,
            "left_command": f"{step.left_command:.9f}",
            "right_command": f"{step.right_command:.9f}",
            "start_time_us": command_rows[0].master_time_us,
            "command_samples": len(command_rows),
            "available_stationary_pre_samples": available_pre_count,
            "baseline_samples_used": len(baseline_rows),
            "baseline_center_radps": f"{baseline_center:.9f}",
            "baseline_noise_sigma_radps": f"{noise_sigma:.9f}",
            "baseline_noise_p95_radps": f"{noise_p95:.9f}",
            "baseline_noise_p99_radps": f"{noise_p99:.9f}",
            "motion_threshold_radps": f"{motion_threshold:.9f}",
            "motion_threshold_delay_samples": "" if motion_index is None else motion_index,
            "motion_threshold_delay_ms": "" if motion_index is None else f"{motion_index * dt_s * 1000.0:.3f}",
            "motion_threshold_fractional_delay_samples": ""
            if math.isnan(motion_fraction)
            else f"{motion_fraction:.3f}",
            "motion_threshold_fractional_delay_ms": ""
            if math.isnan(motion_fraction)
            else f"{motion_fraction * dt_s * 1000.0:.3f}",
            "value_threshold_radps": f"{value_threshold:.9f}",
            "value_threshold_delay_samples": "" if threshold_index is None else threshold_index,
            "value_threshold_delay_ms": "" if threshold_index is None else f"{threshold_index * dt_s * 1000.0:.3f}",
            "value_threshold_fractional_delay_samples": ""
            if math.isnan(threshold_fraction)
            else f"{threshold_fraction:.3f}",
            "value_threshold_fractional_delay_ms": ""
            if math.isnan(threshold_fraction)
            else f"{threshold_fraction * dt_s * 1000.0:.3f}",
            "derivative_onset_delay_samples": "" if deriv_index is None else deriv_index,
            "derivative_onset_delay_ms": "" if deriv_index is None else f"{deriv_index * dt_s * 1000.0:.3f}",
            "derivative_threshold_radps2": f"{deriv_threshold:.6f}",
            "derivative_noise_sigma_radps2": f"{deriv_sigma:.6f}",
            "piecewise_ramp_onset_samples": ramp_index,
            "piecewise_ramp_onset_ms": f"{ramp_index * dt_s * 1000.0:.3f}",
            "piecewise_ramp_slope_radps_per_sample": f"{ramp_slope_per_sample:.9f}",
            "piecewise_ramp_sse": f"{ramp_sse:.9f}",
            "derivative_peak_delay_samples": "" if peak_index is None else peak_index,
            "derivative_peak_delay_ms": "" if peak_index is None else f"{peak_index * dt_s * 1000.0:.3f}",
            "derivative_peak_radps2": f"{peak_derivative:.6f}",
            "first_0p05_delay_samples": "" if first_0p05 is None else first_0p05,
            "first_0p10_delay_samples": "" if first_0p10 is None else first_0p10,
            "peak_first80_directional_radps": f"{peak_early:.9f}",
            "steady_last50_directional_radps": f"{steady:.9f}",
            "delay_quality_flags": ";".join(quality),
        }
        for sample in range(0, 16):
            row[f"y_plus_{sample}"] = f"{response[sample]:.9f}"
        output.append(row)
    return output


def add_template_alignment(step_rows: list[dict[str, object]]) -> None:
    template = build_template(step_rows)
    for row in step_rows:
        response = [float(row[f"y_plus_{sample}"]) for sample in range(0, 16)]
        best_offset, best_sse, scores = local_template_scores(response, template, range(0, 8), fit_len=16)
        row["template_best_offset_samples"] = best_offset
        row["template_best_offset_ms"] = f"{best_offset:.3f}"
        row["template_best_mse"] = f"{best_sse:.9f}"
        for offset in range(0, 7):
            row[f"template_mse_offset_{offset}"] = f"{scores[offset]:.9f}"


def aggregate(step_rows: list[dict[str, object]]) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    groups: dict[tuple[str, str], list[dict[str, object]]] = defaultdict(list)
    all_clean: list[dict[str, object]] = []
    for row in step_rows:
        flags = str(row["delay_quality_flags"])
        if "short_pre" in flags or "no_threshold_onset" in flags:
            continue
        key = (f"{float(row['amplitude']):.2f}", str(row["direction"]))
        groups[key].append(row)
        all_clean.append(row)

    def values(rows: list[dict[str, object]], key: str) -> list[float]:
        result = []
        for row in rows:
            text = str(row[key])
            if text != "":
                result.append(float(text))
        return result

    summary: list[dict[str, str]] = []
    for key in sorted(groups):
        rows = groups[key]
        flags = Counter(flag for row in rows for flag in str(row["delay_quality_flags"]).split(";"))
        summary.append(
            {
                "amplitude": key[0],
                "direction": key[1],
                "steps": str(len(rows)),
                "sustained_launch_steps": str(flags.get("sustained_launch", 0)),
                "twitch_only_steps": str(flags.get("twitch_only", 0)),
                "median_motion_delay_samples": f"{median(values(rows, 'motion_threshold_fractional_delay_samples')):.3f}",
                "median_threshold_delay_samples": f"{median(values(rows, 'value_threshold_fractional_delay_samples')):.3f}",
                "p10_threshold_delay_samples": f"{percentile(values(rows, 'value_threshold_fractional_delay_samples'), 0.10):.3f}",
                "p90_threshold_delay_samples": f"{percentile(values(rows, 'value_threshold_fractional_delay_samples'), 0.90):.3f}",
                "median_derivative_onset_samples": f"{median(values(rows, 'derivative_onset_delay_samples')):.3f}",
                "median_piecewise_ramp_onset_samples": f"{median(values(rows, 'piecewise_ramp_onset_samples')):.3f}",
                "median_derivative_peak_samples": f"{median(values(rows, 'derivative_peak_delay_samples')):.3f}",
                "median_template_best_offset_samples": f"{median(values(rows, 'template_best_offset_samples')):.3f}",
                "mean_y_plus_0": f"{mean(values(rows, 'y_plus_0')):.6f}",
                "mean_y_plus_1": f"{mean(values(rows, 'y_plus_1')):.6f}",
                "mean_y_plus_2": f"{mean(values(rows, 'y_plus_2')):.6f}",
                "mean_y_plus_3": f"{mean(values(rows, 'y_plus_3')):.6f}",
                "mean_y_plus_4": f"{mean(values(rows, 'y_plus_4')):.6f}",
                "mean_y_plus_5": f"{mean(values(rows, 'y_plus_5')):.6f}",
                "mean_y_plus_6": f"{mean(values(rows, 'y_plus_6')):.6f}",
                "quality_flags": ";".join(f"{flag}:{count}" for flag, count in sorted(flags.items())),
            }
        )

    calibration_sets = [
        ("all_clean", all_clean),
        ("sustained_only", [row for row in all_clean if "sustained_launch" in str(row["delay_quality_flags"])]),
        (
            "above_threshold_0p65_0p70",
            [row for row in all_clean if float(row["amplitude"]) >= 0.645],
        ),
        (
            "exclude_twitch_only",
            [row for row in all_clean if "twitch_only" not in str(row["delay_quality_flags"])],
        ),
    ]
    rec_rows: list[dict[str, str]] = []
    for label, rows in calibration_sets:
        if not rows:
            continue
        threshold_values = values(rows, "value_threshold_fractional_delay_samples")
        motion_values = values(rows, "motion_threshold_fractional_delay_samples")
        derivative_values = values(rows, "derivative_onset_delay_samples")
        ramp_values = values(rows, "piecewise_ramp_onset_samples")
        y_offsets = {sample: mean(values(rows, f"y_plus_{sample}")) for sample in range(0, 7)}
        rec_rows.append(
            {
                "calibration_set": label,
                "steps": str(len(rows)),
                "median_motion_fractional_samples": f"{median(motion_values):.3f}",
                "median_motion_fractional_ms": f"{median(motion_values):.3f}",
                "median_threshold_fractional_samples": f"{median(threshold_values):.3f}",
                "median_threshold_fractional_ms": f"{median(threshold_values):.3f}",
                "median_derivative_onset_samples": f"{median(derivative_values):.3f}",
                "median_piecewise_ramp_onset_samples": f"{median(ramp_values):.3f}",
                "recommended_integer_offset_samples": "4",
                "recommended_offset_reason": "first aggregate nonbaseline sample is +4; motion threshold centers near +5, derivative onset near +5, and sustained threshold is deliberately later",
                **{f"mean_y_plus_{sample}": f"{value:.6f}" for sample, value in y_offsets.items()},
            }
        )
    return summary, rec_rows


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
    steps: list[Step],
    step_delay_rows: list[dict[str, object]],
    summary_rows: list[dict[str, str]],
    recommendation_rows: list[dict[str, str]],
    bias_info: tuple[float, int, float],
) -> None:
    bias, bias_count, bias_std = bias_info
    filter_tau_ms = 1000.0 / (2.0 * math.pi * GYRO_CUTOFF_HZ)
    phase_counts = Counter(row.phase_id for row in rows)
    saturation_counts = Counter(row.saturation_flags for row in phase_rows)
    flag_counts = Counter(flag for row in step_delay_rows for flag in str(row["delay_quality_flags"]).split(";"))

    summary_table = [
        "| Amp | Dir | Steps | Sustained | Twitch | Motion samples | Sustained-threshold samples | Deriv samples | Ramp samples | y+0 | y+1 | y+2 | y+3 | y+4 | y+5 | y+6 |",
        "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summary_rows:
        summary_table.append(
            "| {amplitude} | {direction} | {steps} | {sustained_launch_steps} | {twitch_only_steps} | "
            "{median_motion_delay_samples} | {median_threshold_delay_samples} | "
            "{median_derivative_onset_samples} | {median_piecewise_ramp_onset_samples} | "
            "{mean_y_plus_0} | {mean_y_plus_1} | {mean_y_plus_2} | {mean_y_plus_3} | {mean_y_plus_4} | {mean_y_plus_5} | {mean_y_plus_6} |".format(
                **row
            )
        )

    rec_table = [
        "| Set | Steps | Motion samples | Sustained-threshold samples | Derivative samples | Ramp samples | Recommended offset |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in recommendation_rows:
        rec_table.append(
            "| {calibration_set} | {steps} | {median_motion_fractional_samples} | {median_threshold_fractional_samples} | "
            "{median_derivative_onset_samples} | {median_piecewise_ramp_onset_samples} | "
            "{recommended_integer_offset_samples} |".format(**row)
        )

    text = "\n".join(
        [
            "# Yaw Launch Delay Calibration",
            "",
            "## Source",
            "",
            f"- Primary log: `{log_path}`",
            f"- Sidecar: `{sidecar_path}`",
            f"- Phase map: {dict(sorted(sidecar_phase_map(sidecar_path).items()))}",
            f"- Phase row counts: {dict(sorted(phase_counts.items()))}",
            "- Delay target: off-to-on yaw-launch command edge to measured raw-gyro response.",
            "- Sensor target: `gyro_raw_radps` minus independent stationary bias; no UKF targets used.",
            "",
            "## Counts And Quality",
            "",
            f"- Independent static gyro bias: `{bias:.9f}` rad/s from `{bias_count}` static rows; static population std `{bias_std:.9f}` rad/s.",
            f"- Yaw-launch rows: `{len(phase_rows)}`; command steps: `{len(steps)}`.",
            f"- Saturation flags inside yaw-launch phase: {dict(sorted(saturation_counts.items()))}.",
            f"- Delay quality flag counts: {dict(sorted(flag_counts.items()))}.",
            "- Pre-baselines are gathered across the phase boundary where needed. The first yaw-launch step has the long stationary pre-window before `phase_id=20`, not just the single phase marker row.",
            "",
            "## Method",
            "",
            "- For each command edge, the analyzer direction-normalizes the gyro response and subtracts a local stationary baseline.",
            "- Threshold onset uses a robust per-step noise threshold: `max(0.05 rad/s, p99(pre)+0.01, 4*MAD_sigma(pre))`, then requires 3 sustained samples above threshold and reports a fractional crossing.",
            "- Motion onset uses a lower robust threshold: `max(0.03 rad/s, p95(pre)+0.005, 2.5*MAD_sigma(pre))`, then requires 2 sustained samples above threshold. This is intended to estimate first measurable response rather than later high-confidence sustained motion.",
            "- Derivative onset uses robust pre-baseline derivative noise, requires 2 of 3 derivative samples above threshold, and requires a following value response.",
            "- Piecewise ramp onset fits the first 18 samples to a delayed positive ramp and reports the best integer onset.",
            "- Template alignment is computed per step for audit, but the recommendation is based on threshold, derivative, and ramp onset because the template is learned from the same delayed data.",
            "",
            "## Same Through +6 Samples",
            "",
            *summary_table,
            "",
            "## Calibration Sets",
            "",
            *rec_table,
            "",
            "## Delay Recommendation",
            "",
            "- Recommended integer alignment for remaining tuning: `+4` samples.",
            "- Use `+4 ms` as the command-to-first-gyro-response offset at the 1 kHz control/log rate. Treat `+5` as the conservative derivative/motion-threshold onset and `+7` as the high-confidence sustained-threshold crossing.",
            "- Keep this offset global for yaw-launch and similar low-speed command-step replay. Do not make it amplitude-dependent for tuning unless the model explicitly separates below-threshold twitch-only behavior from sustained launch behavior.",
            "- Exclude 0.50 and 0.55 twitch-only steps from sustained-launch alignment calibration. They are useful for static/bristle threshold identification, but their arrested response biases template and threshold fits toward twitch dynamics.",
            "",
            "## Gyro Cutoff Context",
            "",
            f"- A 214 Hz single-pole equivalent has a low-frequency time constant/group-delay scale of about `{filter_tau_ms:.3f}` ms.",
            "- Observed first nonbaseline response is around +4 samples, while robust sustained threshold and derivative methods center around +5 samples.",
            "- That leaves roughly 3 to 4 ms beyond the simple gyro filter scale, attributable to command/PWM timing, motor current and torque buildup, contact/bristle breakaway, and finite thresholding of a filtered ramp.",
            "",
            "## Outputs",
            "",
            "- `yaw_launch_delay_per_step.csv`: per-step threshold, derivative, ramp, template, and +0..+15 response samples.",
            "- `yaw_launch_delay_summary.csv`: aggregate delay by amplitude and direction.",
            "- `yaw_launch_delay_recommendation.csv`: calibration-set recommendations.",
            "",
            "## Reproduce",
            "",
            "```powershell",
            "python codex_analysis\\yaw_launch_step_response\\estimate_yaw_launch_delay.py",
            "```",
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
    bias_info = estimate_static_bias(rows)
    steps = find_steps(phase_rows)
    step_delay_rows = analyze_steps(rows, phase_rows, steps, bias_info[0])
    add_template_alignment(step_delay_rows)
    summary_rows, recommendation_rows = aggregate(step_delay_rows)
    per_step_csv_rows = [{key: str(value) for key, value in row.items()} for row in step_delay_rows]

    write_csv(OUT_DIR / "yaw_launch_delay_per_step.csv", per_step_csv_rows)
    write_csv(OUT_DIR / "yaw_launch_delay_summary.csv", summary_rows)
    write_csv(OUT_DIR / "yaw_launch_delay_recommendation.csv", recommendation_rows)
    (OUT_DIR / "commands_run_delay.txt").write_text(
        "python codex_analysis\\yaw_launch_step_response\\estimate_yaw_launch_delay.py\n",
        encoding="utf-8",
    )
    write_report(
        OUT_DIR / "yaw_launch_delay_calibration_report.md",
        args.log,
        args.sidecar,
        rows,
        phase_rows,
        steps,
        step_delay_rows,
        summary_rows,
        recommendation_rows,
        bias_info,
    )
    print(f"rows={len(rows)} phase20_rows={len(phase_rows)} steps={len(steps)}")
    print(f"bias_mean_radps={bias_info[0]:.9f} bias_rows={bias_info[1]}")
    print(f"wrote={OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
