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


REPO_ROOT = Path(__file__).resolve().parents[3]
OUT_DIR = Path(__file__).resolve().parent
DEFAULT_LOG = REPO_ROOT / "TestResults" / "mmlog_decode_2026-05-04_20-35-47" / "open_floor_main.csv"
DEFAULT_SIDECAR = DEFAULT_LOG.with_suffix(".sidecar")
PHASE_STATIC = 1
PHASE_YAW_LAUNCH = 20
FS_HZ = 1000.0
FAN_SEARCH_MIN_HZ = 148.9
FAN_SEARCH_MAX_HZ = 149.5
FAN_SEARCH_STEP_HZ = 0.025
PRE_FIT_MAX_SAMPLES = 300
PRE_BASELINE_SAMPLES = 150
POST_SAMPLES = 150


@dataclass(frozen=True)
class Row:
    row_index: int
    raw: dict[str, str]
    master_time_us: int
    tick: int
    dt_us: int
    phase_id: int
    repeat_index: int
    speed_bin: float
    mode_flags: int
    saturation_flags: int
    imu_status: int
    left_command: float
    right_command: float
    left_encoder_velocity_mps: float
    right_encoder_velocity_mps: float
    left_encoder_omega_radps: float
    right_encoder_omega_radps: float
    gyro_raw_radps: float
    gyro_logged_radps: float
    accel_body_x_mps2: float
    accel_body_y_mps2: float
    planar_accel_mps2: float


@dataclass(frozen=True)
class Step:
    step_id: int
    start_row_index: int
    end_row_index: int
    left_command: float
    right_command: float
    yaw_command_proxy: float
    amplitude: float
    direction_sign: int
    direction: str
    repeat_index: int
    speed_bin: float


@dataclass(frozen=True)
class ToneModel:
    frequency_hz: float
    coefficients: list[float]
    include_harmonic: bool
    main_amplitude: float
    harmonic_amplitude: float
    pre_rms_before: float
    pre_rms_after: float


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


def pstdev(values: Iterable[float]) -> float:
    materialized = list(values)
    return statistics.pstdev(materialized) if len(materialized) > 1 else 0.0


def rms_centered(values: Iterable[float]) -> float:
    materialized = list(values)
    if not materialized:
        return 0.0
    center = statistics.fmean(materialized)
    return math.sqrt(statistics.fmean((value - center) ** 2 for value in materialized))


def rmse(values: Iterable[float]) -> float:
    materialized = list(values)
    return math.sqrt(statistics.fmean(value * value for value in materialized)) if materialized else 0.0


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


def mad_sigma(values: Iterable[float]) -> float:
    materialized = list(values)
    if not materialized:
        return 0.0
    center = statistics.median(materialized)
    mad = statistics.median(abs(value - center) for value in materialized)
    return 1.4826 * mad


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
                    repeat_index=finite_int(raw, "repeat_index"),
                    speed_bin=finite_float(raw, "speed_bin"),
                    mode_flags=finite_int(raw, "mode_flags"),
                    saturation_flags=finite_int(raw, "saturation_flags"),
                    imu_status=finite_int(raw, "imu_status"),
                    left_command=finite_float(raw, "left_drive_command"),
                    right_command=finite_float(raw, "right_drive_command"),
                    left_encoder_velocity_mps=finite_float(raw, "left_encoder_velocity_mps"),
                    right_encoder_velocity_mps=finite_float(raw, "right_encoder_velocity_mps"),
                    left_encoder_omega_radps=finite_float(raw, "left_encoder_omega_radps"),
                    right_encoder_omega_radps=finite_float(raw, "right_encoder_omega_radps"),
                    gyro_raw_radps=finite_float(raw, "gyro_raw_radps"),
                    gyro_logged_radps=finite_float(raw, "gyro_radps"),
                    accel_body_x_mps2=finite_float(raw, "accel_body_x_mps2"),
                    accel_body_y_mps2=finite_float(raw, "accel_body_y_mps2"),
                    planar_accel_mps2=finite_float(raw, "planar_accel_mps2"),
                )
            )
    return rows


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    if not fieldnames:
        fieldnames = ["empty"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def format_float(value: float, digits: int = 9) -> str:
    if not math.isfinite(value):
        return ""
    return f"{value:.{digits}f}"


def load_sidecar(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.exists():
        return values
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if "=" in line:
                key, value = line.rstrip("\n").split("=", 1)
                values[key] = value
    return values


def stationary(row: Row) -> bool:
    return (
        abs(row.left_command) <= 1.0e-9
        and abs(row.right_command) <= 1.0e-9
        and abs(row.left_encoder_velocity_mps) <= 0.02
        and abs(row.right_encoder_velocity_mps) <= 0.02
    )


def find_yaw_launch_steps(rows: list[Row]) -> list[Step]:
    phase_rows = [row for row in rows if row.phase_id == PHASE_YAW_LAUNCH]
    steps: list[Step] = []
    index = 0
    while index < len(phase_rows):
        row = phase_rows[index]
        if abs(row.left_command) <= 1.0e-9 and abs(row.right_command) <= 1.0e-9:
            index += 1
            continue
        start = index
        left = row.left_command
        right = row.right_command
        while (
            index < len(phase_rows)
            and abs(phase_rows[index].left_command - left) <= 1.0e-9
            and abs(phase_rows[index].right_command - right) <= 1.0e-9
        ):
            index += 1
        yaw_proxy = 0.5 * (left - right)
        sign = 1 if yaw_proxy >= 0.0 else -1
        steps.append(
            Step(
                step_id=len(steps) + 1,
                start_row_index=phase_rows[start].row_index,
                end_row_index=phase_rows[index - 1].row_index + 1,
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


def collect_pre_rows(rows: list[Row], start_index: int, max_samples: int) -> tuple[list[Row], int]:
    collected: list[Row] = []
    index = start_index - 1
    available = 0
    while index >= 0:
        row = rows[index]
        if not stationary(row):
            break
        available += 1
        collected.append(row)
        index -= 1
    collected.reverse()
    return collected[-max_samples:], available


def fft(values: list[complex]) -> list[complex]:
    data = list(values)
    n = len(data)
    j = 0
    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j ^= bit
        if i < j:
            data[i], data[j] = data[j], data[i]
    size = 2
    while size <= n:
        angle = -2.0 * math.pi / size
        wm = complex(math.cos(angle), math.sin(angle))
        half = size // 2
        for start in range(0, n, size):
            w = 1.0 + 0.0j
            for offset in range(half):
                t = w * data[start + offset + half]
                u = data[start + offset]
                data[start + offset] = u + t
                data[start + offset + half] = u - t
                w *= wm
        size *= 2
    return data


def welch_psd(values: list[float], fs_hz: float, nper: int = 2048) -> tuple[list[float], list[float]]:
    if len(values) < nper:
        nper = 1 << max(1, int(math.floor(math.log2(max(2, len(values))))))
    step = nper // 2
    window = [0.5 - 0.5 * math.cos(2.0 * math.pi * i / (nper - 1)) for i in range(nper)]
    scale = fs_hz * sum(w * w for w in window)
    psd_sum = [0.0] * (nper // 2 + 1)
    count = 0
    for start in range(0, len(values) - nper + 1, step):
        segment = values[start : start + nper]
        center = statistics.fmean(segment)
        spectrum = fft([complex((segment[i] - center) * window[i], 0.0) for i in range(nper)])
        for i in range(nper // 2 + 1):
            psd = (abs(spectrum[i]) ** 2) / scale
            if i != 0 and i != nper // 2:
                psd *= 2.0
            psd_sum[i] += psd
        count += 1
    if count > 0:
        psd_sum = [value / count for value in psd_sum]
    frequencies = [fs_hz * i / nper for i in range(nper // 2 + 1)]
    return frequencies, psd_sum


def top_spectral_peaks(values: list[float], min_hz: float = 5.0, max_hz: float = 450.0, count: int = 12) -> list[dict[str, float]]:
    frequencies, psd = welch_psd(values, FS_HZ)
    asd = [math.sqrt(max(0.0, value)) for value in psd]
    peaks: list[int] = []
    for index in range(1, len(frequencies) - 1):
        if frequencies[index] < min_hz or frequencies[index] > max_hz:
            continue
        if asd[index] > asd[index - 1] and asd[index] >= asd[index + 1]:
            peaks.append(index)
    selected = sorted(peaks, key=lambda i: asd[i], reverse=True)[:count]
    return [
        {
            "frequency_hz": frequencies[index],
            "asd": asd[index],
            "psd": psd[index],
        }
        for index in selected
    ]


def tone_amplitude(values: list[float], times_s: list[float], frequency_hz: float) -> float:
    center = statistics.fmean(values)
    c = 0.0
    s = 0.0
    for value, time_s in zip(values, times_s):
        angle = 2.0 * math.pi * frequency_hz * time_s
        centered = value - center
        c += centered * math.cos(angle)
        s += centered * math.sin(angle)
    return 2.0 * math.hypot(c, s) / max(1, len(values))


def estimate_local_fan_frequency(values: list[float], times_s: list[float]) -> float:
    best_frequency = 149.25
    best_amplitude = -1.0
    steps = int(round((FAN_SEARCH_MAX_HZ - FAN_SEARCH_MIN_HZ) / FAN_SEARCH_STEP_HZ))
    for index in range(steps + 1):
        frequency = FAN_SEARCH_MIN_HZ + index * FAN_SEARCH_STEP_HZ
        amplitude = tone_amplitude(values, times_s, frequency)
        if amplitude > best_amplitude:
            best_amplitude = amplitude
            best_frequency = frequency
    return best_frequency


def solve_linear_system(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    n = len(rhs)
    aug = [row[:] + [rhs[i]] for i, row in enumerate(matrix)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1.0e-12:
            aug[pivot][col] = 1.0e-12
        if pivot != col:
            aug[col], aug[pivot] = aug[pivot], aug[col]
        scale = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] /= scale
        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            if factor == 0.0:
                continue
            for j in range(col, n + 1):
                aug[row][j] -= factor * aug[col][j]
    return [aug[i][n] for i in range(n)]


def design_row(time_s: float, frequency_hz: float, include_harmonic: bool) -> list[float]:
    angle = 2.0 * math.pi * frequency_hz * time_s
    row = [1.0, time_s, math.cos(angle), math.sin(angle)]
    if include_harmonic:
        harmonic = 2.0 * angle
        row.extend([math.cos(harmonic), math.sin(harmonic)])
    return row


def fit_tone_model(
    values: list[float],
    times_s: list[float],
    include_harmonic: bool = True,
    frequency_hz: float | None = None,
) -> ToneModel:
    if frequency_hz is None:
        frequency_hz = estimate_local_fan_frequency(values, times_s)
    width = 6 if include_harmonic else 4
    normal = [[0.0] * width for _ in range(width)]
    rhs = [0.0] * width
    for value, time_s in zip(values, times_s):
        row = design_row(time_s, frequency_hz, include_harmonic)
        for i in range(width):
            rhs[i] += row[i] * value
            for j in range(width):
                normal[i][j] += row[i] * row[j]
    coefficients = solve_linear_system(normal, rhs)
    before = [value - statistics.fmean(values) for value in values]
    after = [value - fan_sinusoid_at(time_s, frequency_hz, coefficients, include_harmonic) for value, time_s in zip(values, times_s)]
    after_centered = [value - statistics.fmean(after) for value in after]
    main_amplitude = math.hypot(coefficients[2], coefficients[3])
    harmonic_amplitude = math.hypot(coefficients[4], coefficients[5]) if include_harmonic else 0.0
    return ToneModel(
        frequency_hz=frequency_hz,
        coefficients=coefficients,
        include_harmonic=include_harmonic,
        main_amplitude=main_amplitude,
        harmonic_amplitude=harmonic_amplitude,
        pre_rms_before=rmse(before),
        pre_rms_after=rmse(after_centered),
    )


def fan_sinusoid_at(time_s: float, frequency_hz: float, coefficients: list[float], include_harmonic: bool) -> float:
    angle = 2.0 * math.pi * frequency_hz * time_s
    value = coefficients[2] * math.cos(angle) + coefficients[3] * math.sin(angle)
    if include_harmonic:
        harmonic = 2.0 * angle
        value += coefficients[4] * math.cos(harmonic) + coefficients[5] * math.sin(harmonic)
    return value


def subtract_tone(values: list[float], times_s: list[float], model: ToneModel) -> tuple[list[float], list[float]]:
    estimates = [
        fan_sinusoid_at(time_s, model.frequency_hz, model.coefficients, model.include_harmonic)
        for time_s in times_s
    ]
    return [value - estimate for value, estimate in zip(values, estimates)], estimates


def first_sustained(values: list[float], threshold: float, start: int = 0, run: int = 2) -> int | None:
    for index in range(start, max(start, len(values) - run + 1)):
        if all(values[index + offset] >= threshold for offset in range(run)):
            return index
    return None


def fit_slope(values: list[float], start: int, stop: int) -> float:
    xs = list(range(start, stop))
    ys = values[start:stop]
    if len(xs) < 2:
        return float("nan")
    x_mean = statistics.fmean(xs)
    y_mean = statistics.fmean(ys)
    denom = sum((x - x_mean) ** 2 for x in xs)
    if denom == 0.0:
        return float("nan")
    slope_per_sample = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denom
    return slope_per_sample * FS_HZ


def linear_fit_quality(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    if len(xs) < 2:
        return float("nan"), float("nan"), float("nan")
    x_mean = statistics.fmean(xs)
    y_mean = statistics.fmean(ys)
    denom = sum((x - x_mean) ** 2 for x in xs)
    if denom == 0.0:
        return float("nan"), float("nan"), float("nan")
    slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denom
    intercept = y_mean - slope * x_mean
    residuals = [y - (intercept + slope * x) for x, y in zip(xs, ys)]
    sse = sum(value * value for value in residuals)
    sst = sum((y - y_mean) ** 2 for y in ys)
    r2 = 1.0 - sse / sst if sst > 0.0 else float("nan")
    return slope, intercept, r2


def command_row_counts(rows: list[Row]) -> Counter:
    return Counter(row.phase_id for row in rows)


def analyze(args: argparse.Namespace) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = read_rows(args.log)
    sidecar = load_sidecar(args.sidecar)
    steps = find_yaw_launch_steps(rows)
    static_rows = [row for row in rows if row.phase_id == PHASE_STATIC and stationary(row)]
    yaw_rows = [row for row in rows if row.phase_id == PHASE_YAW_LAUNCH]

    spectral_rows: list[dict[str, object]] = []
    stationary_channels = {
        "gyro_raw_radps": [row.gyro_raw_radps for row in static_rows],
        "accel_body_x_mps2": [row.accel_body_x_mps2 for row in static_rows],
        "accel_body_y_mps2": [row.accel_body_y_mps2 for row in static_rows],
        "left_encoder_omega_radps": [row.left_encoder_omega_radps for row in static_rows],
        "right_encoder_omega_radps": [row.right_encoder_omega_radps for row in static_rows],
    }
    for channel, values in stationary_channels.items():
        for rank, peak in enumerate(top_spectral_peaks(values), start=1):
            spectral_rows.append(
                {
                    "source": "phase1_stationary_welch",
                    "channel": channel,
                    "rank": rank,
                    "frequency_hz": format_float(peak["frequency_hz"], 6),
                    "amplitude_spectral_density": format_float(peak["asd"], 9),
                    "power_spectral_density": format_float(peak["psd"], 12),
                    "stationary_mean": format_float(mean(values), 9),
                    "stationary_std": format_float(pstdev(values), 9),
                }
            )
    write_csv(OUT_DIR / "fan_vibration_component_summary.csv", spectral_rows)

    drift_rows: list[dict[str, object]] = []
    chunk_size = 2048
    for channel, values in stationary_channels.items():
        if channel.startswith("left_encoder") or channel.startswith("right_encoder"):
            continue
        times_s = [(row.master_time_us - static_rows[0].master_time_us) / 1.0e6 for row in static_rows]
        for start in range(0, len(values) - chunk_size + 1, chunk_size):
            seg_values = values[start : start + chunk_size]
            seg_times = times_s[start : start + chunk_size]
            frequency = estimate_local_fan_frequency(seg_values, seg_times)
            amplitude = tone_amplitude(seg_values, seg_times, frequency)
            drift_rows.append(
                {
                    "channel": channel,
                    "segment_start_s": format_float(seg_times[0], 3),
                    "segment_end_s": format_float(seg_times[-1], 3),
                    "fan_frequency_hz": format_float(frequency, 3),
                    "fan_amplitude": format_float(amplitude, 9),
                    "segment_std": format_float(pstdev(seg_values), 9),
                }
            )
    write_csv(OUT_DIR / "fan_vibration_drift_by_segment.csv", drift_rows)
    gyro_drift_frequencies = [
        float(row["fan_frequency_hz"])
        for row in drift_rows
        if row["channel"] == "gyro_raw_radps"
    ]
    stationary_fan_frequency_hz = statistics.median(gyro_drift_frequencies) if gyro_drift_frequencies else 149.25

    stationary_filter_rows: list[dict[str, object]] = []
    for channel, values in stationary_channels.items():
        if channel.startswith("left_encoder") or channel.startswith("right_encoder"):
            continue
        channel_rows = static_rows
        for start in range(0, len(values) - chunk_size + 1, chunk_size):
            seg_values = values[start : start + chunk_size]
            seg_rows = channel_rows[start : start + chunk_size]
            t0_us = seg_rows[0].master_time_us
            seg_times = [(row.master_time_us - t0_us) / 1.0e6 for row in seg_rows]
            model = fit_tone_model(seg_values, seg_times, include_harmonic=True)
            filtered, _ = subtract_tone(seg_values, seg_times, model)
            raw_rms = rms_centered(seg_values)
            filtered_rms = rms_centered(filtered)
            stationary_filter_rows.append(
                {
                    "channel": channel,
                    "segment_start_s": format_float((seg_rows[0].master_time_us - static_rows[0].master_time_us) / 1.0e6, 3),
                    "segment_end_s": format_float((seg_rows[-1].master_time_us - static_rows[0].master_time_us) / 1.0e6, 3),
                    "fan_frequency_hz": format_float(model.frequency_hz, 3),
                    "fan_amplitude": format_float(model.main_amplitude, 9),
                    "fan_harmonic_amplitude": format_float(model.harmonic_amplitude, 9),
                    "raw_centered_rms": format_float(raw_rms, 9),
                    "filtered_centered_rms": format_float(filtered_rms, 9),
                    "rms_reduction_pct": format_float(100.0 * (1.0 - filtered_rms / raw_rms) if raw_rms > 0.0 else 0.0, 3),
                }
            )
    write_csv(OUT_DIR / "fan_vibration_stationary_filter_check.csv", stationary_filter_rows)

    filtered_sample_rows: list[dict[str, object]] = []
    step_metric_rows: list[dict[str, object]] = []
    onset_detail: dict[tuple[str, int], list[float]] = defaultdict(list)
    group_slopes: dict[tuple[str, float, str], list[float]] = defaultdict(list)
    step_slope_rows: list[dict[str, object]] = []

    for step in steps:
        pre_fit_rows, available_pre = collect_pre_rows(rows, step.start_row_index, PRE_FIT_MAX_SAMPLES)
        if len(pre_fit_rows) < PRE_BASELINE_SAMPLES:
            continue
        start_extract = max(0, step.start_row_index - PRE_BASELINE_SAMPLES)
        end_extract = min(len(rows), step.end_row_index + POST_SAMPLES)
        window_rows = rows[start_extract:end_extract]
        pre_eval_rows = rows[step.start_row_index - PRE_BASELINE_SAMPLES : step.start_row_index]
        fit_t0_us = pre_fit_rows[0].master_time_us
        pre_times_s = [(row.master_time_us - fit_t0_us) / 1.0e6 for row in pre_fit_rows]
        window_times_s = [(row.master_time_us - fit_t0_us) / 1.0e6 for row in window_rows]

        gyro_model = fit_tone_model(
            [row.gyro_raw_radps for row in pre_fit_rows],
            pre_times_s,
            include_harmonic=True,
            frequency_hz=stationary_fan_frequency_hz,
        )
        ax_model = fit_tone_model(
            [row.accel_body_x_mps2 for row in pre_fit_rows],
            pre_times_s,
            include_harmonic=True,
            frequency_hz=stationary_fan_frequency_hz,
        )
        ay_model = fit_tone_model(
            [row.accel_body_y_mps2 for row in pre_fit_rows],
            pre_times_s,
            include_harmonic=True,
            frequency_hz=stationary_fan_frequency_hz,
        )
        filtered_gyro, gyro_fan = subtract_tone([row.gyro_raw_radps for row in window_rows], window_times_s, gyro_model)
        filtered_ax, ax_fan = subtract_tone([row.accel_body_x_mps2 for row in window_rows], window_times_s, ax_model)
        filtered_ay, ay_fan = subtract_tone([row.accel_body_y_mps2 for row in window_rows], window_times_s, ay_model)

        pre_offset = step.start_row_index - start_extract
        command_end_offset = step.end_row_index - start_extract
        raw_pre = [step.direction_sign * row.gyro_raw_radps for row in pre_eval_rows]
        filt_pre = [
            step.direction_sign * filtered_gyro[pre_offset - PRE_BASELINE_SAMPLES + i]
            for i in range(PRE_BASELINE_SAMPLES)
        ]
        raw_center = statistics.fmean(raw_pre)
        filt_center = statistics.fmean(filt_pre)
        raw_pre_resid = [value - raw_center for value in raw_pre]
        filt_pre_resid = [value - filt_center for value in filt_pre]
        raw_sigma = mad_sigma(raw_pre_resid)
        filt_sigma = mad_sigma(filt_pre_resid)
        raw_p95 = percentile([abs(value) for value in raw_pre_resid], 0.95)
        filt_p95 = percentile([abs(value) for value in filt_pre_resid], 0.95)
        raw_threshold = max(0.03, raw_p95 + 0.005, 2.5 * raw_sigma)
        filt_threshold = max(0.03, filt_p95 + 0.005, 2.5 * filt_sigma)

        raw_response = [
            step.direction_sign * window_rows[pre_offset + i].gyro_raw_radps - raw_center
            for i in range(min(80, len(window_rows) - pre_offset))
        ]
        filt_response = [
            step.direction_sign * filtered_gyro[pre_offset + i] - filt_center
            for i in range(min(80, len(window_rows) - pre_offset))
        ]
        raw_onset = first_sustained(raw_response, raw_threshold, start=0, run=2)
        filt_onset = first_sustained(filt_response, filt_threshold, start=0, run=2)
        raw_slope = fit_slope(raw_response, 4, min(11, len(raw_response)))
        filt_slope = fit_slope(filt_response, 4, min(11, len(filt_response)))
        quality = "sustained_launch" if max(raw_response[:80]) >= 1.0 else "twitch_or_transition"
        for offset in (0, 4, 5):
            if offset < len(raw_response):
                onset_detail[("raw", offset)].append(raw_response[offset] / raw_threshold if raw_threshold > 0.0 else float("nan"))
                onset_detail[("filtered", offset)].append(
                    filt_response[offset] / filt_threshold if filt_threshold > 0.0 else float("nan")
                )
        group_key_raw = ("raw", step.amplitude, step.direction)
        group_key_filt = ("filtered", step.amplitude, step.direction)
        group_slopes[group_key_raw].append(raw_slope)
        group_slopes[group_key_filt].append(filt_slope)
        step_slope_rows.append(
            {
                "step_id": step.step_id,
                "amplitude": format_float(step.amplitude, 3),
                "direction": step.direction,
                "quality": quality,
                "raw_initial_slope_radps2": format_float(raw_slope, 6),
                "filtered_initial_slope_radps2": format_float(filt_slope, 6),
                "raw_motion_threshold_radps": format_float(raw_threshold, 9),
                "filtered_motion_threshold_radps": format_float(filt_threshold, 9),
                "raw_onset_samples": "" if raw_onset is None else raw_onset,
                "filtered_onset_samples": "" if filt_onset is None else filt_onset,
            }
        )

        step_metric_rows.append(
            {
                "step_id": step.step_id,
                "repeat_index": step.repeat_index,
                "amplitude": format_float(step.amplitude, 3),
                "direction": step.direction,
                "start_time_us": rows[step.start_row_index].master_time_us,
                "command_samples": step.end_row_index - step.start_row_index,
                "available_stationary_pre_samples": available_pre,
                "fan_frequency_gyro_hz": format_float(gyro_model.frequency_hz, 3),
                "fan_amplitude_gyro_radps": format_float(gyro_model.main_amplitude, 9),
                "fan_harmonic_amplitude_gyro_radps": format_float(gyro_model.harmonic_amplitude, 9),
                "fan_frequency_accel_x_hz": format_float(ax_model.frequency_hz, 3),
                "fan_amplitude_accel_x_mps2": format_float(ax_model.main_amplitude, 9),
                "fan_harmonic_amplitude_accel_x_mps2": format_float(ax_model.harmonic_amplitude, 9),
                "fan_frequency_accel_y_hz": format_float(ay_model.frequency_hz, 3),
                "fan_amplitude_accel_y_mps2": format_float(ay_model.main_amplitude, 9),
                "fan_harmonic_amplitude_accel_y_mps2": format_float(ay_model.harmonic_amplitude, 9),
                "pre_raw_mad_sigma_radps": format_float(raw_sigma, 9),
                "pre_filtered_mad_sigma_radps": format_float(filt_sigma, 9),
                "pre_raw_p95_abs_radps": format_float(raw_p95, 9),
                "pre_filtered_p95_abs_radps": format_float(filt_p95, 9),
                "pre_raw_centered_rms_radps": format_float(rms_centered(raw_pre), 9),
                "pre_filtered_centered_rms_radps": format_float(rms_centered(filt_pre), 9),
                "raw_threshold_radps": format_float(raw_threshold, 9),
                "filtered_threshold_radps": format_float(filt_threshold, 9),
                "raw_onset_samples": "" if raw_onset is None else raw_onset,
                "filtered_onset_samples": "" if filt_onset is None else filt_onset,
                "raw_y_plus_0_radps": format_float(raw_response[0], 9),
                "filtered_y_plus_0_radps": format_float(filt_response[0], 9),
                "raw_y_plus_4_radps": format_float(raw_response[4], 9),
                "filtered_y_plus_4_radps": format_float(filt_response[4], 9),
                "raw_y_plus_5_radps": format_float(raw_response[5], 9),
                "filtered_y_plus_5_radps": format_float(filt_response[5], 9),
                "raw_initial_slope_plus4_to_plus10_radps2": format_float(raw_slope, 6),
                "filtered_initial_slope_plus4_to_plus10_radps2": format_float(filt_slope, 6),
                "quality": quality,
            }
        )

        for offset, row in enumerate(window_rows):
            sample_offset = row.row_index - step.start_row_index
            filtered_sample_rows.append(
                {
                    "step_id": step.step_id,
                    "sample_offset": sample_offset,
                    "master_time_us": row.master_time_us,
                    "phase_id": row.phase_id,
                    "repeat_index": step.repeat_index,
                    "amplitude": format_float(step.amplitude, 3),
                    "direction": step.direction,
                    "direction_sign": step.direction_sign,
                    "left_command": format_float(row.left_command, 6),
                    "right_command": format_float(row.right_command, 6),
                    "gyro_raw_radps": format_float(row.gyro_raw_radps, 9),
                    "gyro_fan_estimate_radps": format_float(gyro_fan[offset], 9),
                    "gyro_fan_filtered_radps": format_float(filtered_gyro[offset], 9),
                    "gyro_directional_raw_baseline_removed_radps": format_float(
                        step.direction_sign * row.gyro_raw_radps - raw_center, 9
                    ),
                    "gyro_directional_filtered_baseline_removed_radps": format_float(
                        step.direction_sign * filtered_gyro[offset] - filt_center, 9
                    ),
                    "accel_x_raw_mps2": format_float(row.accel_body_x_mps2, 9),
                    "accel_x_fan_estimate_mps2": format_float(ax_fan[offset], 9),
                    "accel_x_fan_filtered_mps2": format_float(filtered_ax[offset], 9),
                    "accel_y_raw_mps2": format_float(row.accel_body_y_mps2, 9),
                    "accel_y_fan_estimate_mps2": format_float(ay_fan[offset], 9),
                    "accel_y_fan_filtered_mps2": format_float(filtered_ay[offset], 9),
                    "left_encoder_velocity_mps": format_float(row.left_encoder_velocity_mps, 9),
                    "right_encoder_velocity_mps": format_float(row.right_encoder_velocity_mps, 9),
                    "imu_status": row.imu_status,
                    "mode_flags": row.mode_flags,
                    "saturation_flags": row.saturation_flags,
                }
            )

    write_csv(OUT_DIR / "yaw_launch_fan_filter_step_metrics.csv", step_metric_rows)
    write_csv(OUT_DIR / "yaw_launch_fan_filtered_samples.csv", filtered_sample_rows)
    write_csv(OUT_DIR / "yaw_launch_fan_filter_step_slopes.csv", step_slope_rows)

    detectability_rows: list[dict[str, object]] = []
    for signal in ("raw", "filtered"):
        for offset in (0, 4, 5):
            snrs = onset_detail[(signal, offset)]
            detectability_rows.append(
                {
                    "signal": signal,
                    "alignment_offset_samples": offset,
                    "steps": len(snrs),
                    "median_value_over_threshold": format_float(statistics.median(snrs), 6),
                    "mean_value_over_threshold": format_float(mean(snrs), 6),
                    "p10_value_over_threshold": format_float(percentile(snrs, 0.10), 6),
                    "p90_value_over_threshold": format_float(percentile(snrs, 0.90), 6),
                }
            )
    write_csv(OUT_DIR / "yaw_launch_onset_detectability_same_plus4_plus5.csv", detectability_rows)

    ident_rows: list[dict[str, object]] = []
    for key, slopes in sorted(group_slopes.items(), key=lambda item: (item[0][1], item[0][2], item[0][0])):
        signal, amplitude, direction = key
        ident_rows.append(
            {
                "signal": signal,
                "amplitude": format_float(amplitude, 3),
                "direction": direction,
                "steps": len(slopes),
                "mean_initial_slope_plus4_to_plus10_radps2": format_float(mean(slopes), 6),
                "std_initial_slope_plus4_to_plus10_radps2": format_float(pstdev(slopes), 6),
                "cv_initial_slope": format_float(pstdev(slopes) / abs(mean(slopes)) if mean(slopes) else float("nan"), 6),
            }
        )
    for signal in ("raw", "filtered"):
        xs: list[float] = []
        ys: list[float] = []
        for row in step_slope_rows:
            value = row[f"{signal}_initial_slope_radps2"]
            if value == "":
                continue
            xs.append(float(row["amplitude"]))
            ys.append(float(value))
        slope, intercept, r2 = linear_fit_quality(xs, ys)
        ident_rows.append(
            {
                "signal": signal,
                "amplitude": "all",
                "direction": "both",
                "steps": len(xs),
                "mean_initial_slope_plus4_to_plus10_radps2": format_float(mean(ys), 6),
                "std_initial_slope_plus4_to_plus10_radps2": format_float(pstdev(ys), 6),
                "cv_initial_slope": format_float(pstdev(ys) / abs(mean(ys)) if mean(ys) else float("nan"), 6),
                "linear_slope_vs_command": format_float(slope, 6),
                "linear_intercept_vs_command": format_float(intercept, 6),
                "linear_r2_vs_command": format_float(r2, 6),
            }
        )
    write_csv(OUT_DIR / "yaw_launch_filter_identifiability_summary.csv", ident_rows)

    report = build_report(
        args.log,
        args.sidecar,
        sidecar,
        rows,
        yaw_rows,
        static_rows,
        steps,
        spectral_rows,
        drift_rows,
        stationary_filter_rows,
        stationary_fan_frequency_hz,
        step_metric_rows,
        detectability_rows,
        ident_rows,
    )
    (OUT_DIR / "fan_vibration_filter_report.md").write_text(report, encoding="utf-8")
    (OUT_DIR / "commands_run.txt").write_text(
        f"python {Path(__file__).relative_to(REPO_ROOT)}\n",
        encoding="utf-8",
    )


def summarize_improvement(step_metric_rows: list[dict[str, object]]) -> dict[str, float]:
    raw_rms = [float(row["pre_raw_centered_rms_radps"]) for row in step_metric_rows]
    filt_rms = [float(row["pre_filtered_centered_rms_radps"]) for row in step_metric_rows]
    raw_p95 = [float(row["pre_raw_p95_abs_radps"]) for row in step_metric_rows]
    filt_p95 = [float(row["pre_filtered_p95_abs_radps"]) for row in step_metric_rows]
    return {
        "median_raw_rms": statistics.median(raw_rms),
        "median_filtered_rms": statistics.median(filt_rms),
        "median_rms_reduction_pct": 100.0 * (1.0 - statistics.median(filt_rms) / statistics.median(raw_rms)),
        "median_raw_p95": statistics.median(raw_p95),
        "median_filtered_p95": statistics.median(filt_p95),
        "median_p95_reduction_pct": 100.0 * (1.0 - statistics.median(filt_p95) / statistics.median(raw_p95)),
    }


def table(rows: list[list[object]]) -> str:
    return "\n".join("| " + " | ".join(str(cell) for cell in row) + " |" for row in rows)


def build_report(
    log_path: Path,
    sidecar_path: Path,
    sidecar: dict[str, str],
    rows: list[Row],
    yaw_rows: list[Row],
    static_rows: list[Row],
    steps: list[Step],
    spectral_rows: list[dict[str, object]],
    drift_rows: list[dict[str, object]],
    stationary_filter_rows: list[dict[str, object]],
    stationary_fan_frequency_hz: float,
    step_metric_rows: list[dict[str, object]],
    detectability_rows: list[dict[str, object]],
    ident_rows: list[dict[str, object]],
) -> str:
    phase_counts = command_row_counts(rows)
    improvement = summarize_improvement(step_metric_rows)
    gyro_peaks = [row for row in spectral_rows if row["channel"] == "gyro_raw_radps"][:8]
    accel_x_peaks = [row for row in spectral_rows if row["channel"] == "accel_body_x_mps2"][:6]
    accel_y_peaks = [row for row in spectral_rows if row["channel"] == "accel_body_y_mps2"][:6]
    gyro_drift = [row for row in drift_rows if row["channel"] == "gyro_raw_radps"]
    step_amps = [float(row["fan_amplitude_gyro_radps"]) for row in step_metric_rows]
    step_harmonics = [float(row["fan_harmonic_amplitude_gyro_radps"]) for row in step_metric_rows]
    static_filter_by_channel: dict[str, list[float]] = defaultdict(list)
    for row in stationary_filter_rows:
        static_filter_by_channel[str(row["channel"])].append(float(row["rms_reduction_pct"]))
    detect_table = [["Signal", "Offset", "Median value/threshold", "P10", "P90"]]
    detect_table.append(["---", "---:", "---:", "---:", "---:"])
    for row in detectability_rows:
        detect_table.append(
            [
                row["signal"],
                row["alignment_offset_samples"],
                row["median_value_over_threshold"],
                row["p10_value_over_threshold"],
                row["p90_value_over_threshold"],
            ]
        )
    ident_compact = [row for row in ident_rows if row["amplitude"] == "all"]
    ident_table = [["Signal", "Steps", "Slope-vs-command R2", "Mean +4..+10 yaw accel", "Std"]]
    ident_table.append(["---", "---:", "---:", "---:", "---:"])
    for row in ident_compact:
        ident_table.append(
            [
                row["signal"],
                row["steps"],
                row.get("linear_r2_vs_command", ""),
                row["mean_initial_slope_plus4_to_plus10_radps2"],
                row["std_initial_slope_plus4_to_plus10_radps2"],
            ]
        )

    lines = [
        "# Fan/Vibration Filtering For Yaw-Launch Plant Identification",
        "",
        "## Source",
        "",
        f"- Primary log: `{log_path}`",
        f"- Sidecar: `{sidecar_path}`",
        f"- Fan duty from sidecar: `{sidecar.get('fan_duty_cycle', 'unknown')}`",
        f"- Phase counts: `{dict(sorted(phase_counts.items()))}`",
        f"- Static stationary rows used: `{len(static_rows)}`",
        f"- Yaw-launch rows: `{len(yaw_rows)}`",
        f"- Command steps found: `{len(steps)}`",
        "- Sensor target: raw gyro and raw/body accelerometer channels only; UKF targets and UKF state were not used.",
        "",
        "## Dominant Fan/Vibration Components",
        "",
        "- The stationary phase is dominated by a narrow fan/vibration line near `149.2 Hz`.",
        f"- The per-step subtraction uses the stationary-derived fan frequency `{stationary_fan_frequency_hz:.3f} Hz` and fits only local amplitude/phase from each stationary pre-window.",
        f"- Per-step gyro fan amplitude median: `{statistics.median(step_amps):.6f} rad/s`; 10th/90th percentiles: `{percentile(step_amps, 0.10):.6f}`/`{percentile(step_amps, 0.90):.6f} rad/s`.",
        f"- Per-step gyro second-harmonic fit median: `{statistics.median(step_harmonics):.6f} rad/s`; it is small relative to the 149 Hz line but kept in the subtraction model.",
        "- Accel X/Y show the same 149 Hz line at much larger physical amplitude, plus a visible harmonic near 298 Hz; encoders have no stationary fan line because stationary encoder rates are zero.",
        "",
        "Top stationary gyro peaks:",
        "",
        table([["Rank", "Frequency Hz", "ASD", "PSD"], ["---:", "---:", "---:", "---:"]] + [[row["rank"], row["frequency_hz"], row["amplitude_spectral_density"], row["power_spectral_density"]] for row in gyro_peaks]),
        "",
        "Top stationary accel peaks:",
        "",
        table([["Channel", "Rank", "Frequency Hz", "ASD"], ["---", "---:", "---:", "---:"]] + [[row["channel"], row["rank"], row["frequency_hz"], row["amplitude_spectral_density"]] for row in (accel_x_peaks + accel_y_peaks)]),
        "",
        "## Drift And Aliasing Risk",
        "",
        f"- Static 2.048 s segment gyro fan estimates span `{min(float(row['fan_frequency_hz']) for row in gyro_drift):.3f}` to `{max(float(row['fan_frequency_hz']) for row in gyro_drift):.3f} Hz`.",
        "- The fan period is about `6.7 ms`, directly overlapping the +4/+5 ms launch-onset region. That does not alias below Nyquist, but it aliases into the step-response measurement problem because fixed sample offsets can land on different fan phases from step to step.",
        "- The 144 to 157 Hz shoulders in finite-window spectra are consistent with short-window leakage plus mild fan-speed/amplitude modulation. They should be treated as fan contamination, not as yaw plant dynamics.",
        "",
        "## Filter Recommendation",
        "",
        "- Recommended preprocessing for compact PlantModel identification: per-step pre-window sinusoid subtraction on `gyro_raw_radps`, using the stationary-derived 149 Hz fundamental and small 298 Hz harmonic.",
        "- The fit uses only stationary samples before each command edge to estimate local amplitude/phase. It fits bias/trend only to isolate the sinusoid, then subtracts only the sinusoidal components, preserving low-frequency yaw dynamics and command-onset timing.",
        "- This is offline analysis, but it has a causal-equivalent interpretation: estimate fan phase/amplitude from a stationary pre-window and hold those parameters through the measurement window. Do not deploy this as a runtime impulse-response mechanism.",
        "- A broad zero-phase 142-158 Hz notch is not recommended as the primary identification preprocessing because it can remove real launch transient energy. Use it only as an audit if a later worker needs to bound residual narrowband contamination.",
        "",
        "Downstream function entry points in `characterize_fan_vibration_filter.py`: `fit_tone_model`, `subtract_tone`, and the generated `yaw_launch_fan_filtered_samples.csv` measurement windows.",
        "",
        "## Noise Floor And Onset Detectability",
        "",
        f"- Initial stationary phase adaptive fan subtraction reduces centered RMS by median `{statistics.median(static_filter_by_channel['gyro_raw_radps']):.1f}%` on gyro, `{statistics.median(static_filter_by_channel['accel_body_x_mps2']):.1f}%` on accel X, and `{statistics.median(static_filter_by_channel['accel_body_y_mps2']):.1f}%` on accel Y over 2.048 s chunks.",
        f"- Median pre-window gyro centered RMS: raw `{improvement['median_raw_rms']:.6f} rad/s`, filtered `{improvement['median_filtered_rms']:.6f} rad/s`, reduction `{improvement['median_rms_reduction_pct']:.1f}%`.",
        f"- Median pre-window gyro P95 absolute residual: raw `{improvement['median_raw_p95']:.6f} rad/s`, filtered `{improvement['median_filtered_p95']:.6f} rad/s`, reduction `{improvement['median_p95_reduction_pct']:.1f}%`.",
        "",
        table(detect_table),
        "",
        "## Compact-Parameter Identifiability",
        "",
        "The following uses the direction-normalized gyro slope from +4 through +10 samples after command onset as a compact initial yaw-acceleration proxy. It is not a final plant fit; it is an identifiability check for whether filtering improves a plausible physical-parameter observable.",
        "",
        table(ident_table),
        "",
        "- Filtering clearly removes the stationary fan component and moderately improves the +4/+5 onset threshold margin. It does not materially improve this simple slope-vs-command R2, which means fan removal is necessary cleanup but not by itself a compact PlantModel fit.",
        "- Filtered raw gyro is reliable for estimating yaw launch timing, initial yaw acceleration, twitch/launch threshold, and repeatability of compact yaw/contact terms.",
        "- Filtered accel X/Y are reliable for identifying and auditing vibration contamination and may help with timing corroboration. They are not primary yaw-torque observables here because the fan line is larger than the useful low-speed planar acceleration signal and because yaw-launch accelerometer response mixes body acceleration with IMU placement and contact transients.",
        "- Encoders are reliable for stationary gating and for later drivetrain response checks. They do not show stationary fan contamination in this log.",
        "",
        "## Caveats",
        "",
        "- The sidecar records fan duty `0.800`; the frequency/amplitude conclusions should be re-estimated for other fan duties.",
        "- The subtraction model is trained on stationary pre-windows. Moving-contact spectra can differ after launch, so residual fan content during motion should be audited before fitting high-frequency model terms.",
        "- Transient contact/bristle breakaway near +4 to +10 ms is real plant content. Do not interpret the filtered signal as pure rigid-body response.",
        "- The subtraction has no phase delay because it subtracts an evaluated sinusoid rather than applying a causal IIR/FIR filter. A conventional causal notch would introduce phase/group-delay concerns unless compensated offline.",
        "",
        "## Outputs",
        "",
        "- `fan_vibration_component_summary.csv`: stationary spectral peaks.",
        "- `fan_vibration_drift_by_segment.csv`: static-phase 2.048 s fan frequency/amplitude drift.",
        "- `fan_vibration_stationary_filter_check.csv`: adaptive stationary fan-subtraction check by 2.048 s chunk.",
        "- `yaw_launch_fan_filter_step_metrics.csv`: per-step fan fit, noise-floor, onset, and slope metrics.",
        "- `yaw_launch_onset_detectability_same_plus4_plus5.csv`: same/+4/+5 threshold-margin summary.",
        "- `yaw_launch_filter_identifiability_summary.csv`: compact initial-slope repeatability and slope-vs-command audit.",
        "- `yaw_launch_fan_filtered_samples.csv`: per-step measurement windows with raw and fan-filtered gyro/accel channels for downstream physical parameter identification.",
        "",
        "## Reproduce",
        "",
        "```powershell",
        "python codex_analysis\\full_impulse_response_characterization\\fan_filter\\characterize_fan_vibration_filter.py",
        "```",
    ]
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Characterize and subtract fan vibration in yaw-launch measurements.")
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--sidecar", type=Path, default=DEFAULT_SIDECAR)
    return parser.parse_args()


if __name__ == "__main__":
    analyze(parse_args())
