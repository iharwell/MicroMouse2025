from __future__ import annotations

import cmath
import math
import statistics
from dataclasses import dataclass
from pathlib import Path

from open_floor_plant_fit import load_run_id
from open_floor_recovery import RunPlantParameters
from open_floor_recovery import drive_torque_from_command
from open_floor_recovery import load_control_log_parameters
from open_floor_recovery import parse_key_value_fields
from open_floor_recovery import row_watchdog_flags


FFT_MIN_FREQUENCY_HZ = 3.0
FFT_MAX_FREQUENCY_HZ = 30.0
FFT_HIGH_PASS_WINDOW_SAMPLES = 251
FFT_MIN_PHASE_ROWS = 256
FFT_STRONG_PROMINENCE_THRESHOLD = 3.0
FFT_RECOMMENDED_MIN_FREQUENCY_HZ = 10.0


@dataclass
class YawFftRepeatSummary:
    primitive_id: int
    repeat_index: int
    sample_count: int
    dominant_frequency_hz: float
    oscillation_prominence: float
    strong_peak: bool
    wheel_speed_vs_torque_phase_deg: float
    yaw_vs_rigid_wheel_phase_deg: float
    yaw_coupling_magnitude: float
    raw_total_inertia_kg_m2: float
    rigid_body_subtracted_wheel_inertia_kg_m2: float
    phase_corrected_wheel_inertia_kg_m2: float
    raw_damping_nm_per_radps: float
    phase_corrected_damping_nm_per_radps: float


@dataclass
class YawFftAggregateSummary:
    repeat_count: int
    strong_repeat_count: int
    recommended_candidate_count: int
    dominant_frequency_hz_median: float | None
    wheel_speed_vs_torque_phase_deg_median: float | None
    yaw_vs_rigid_wheel_phase_deg_median: float | None
    yaw_coupling_magnitude_median: float | None
    raw_total_inertia_kg_m2_median: float | None
    phase_corrected_wheel_inertia_kg_m2_median: float | None
    recommended_wheel_inertia_kg_m2: float | None


@dataclass
class YawFftSummary:
    run_id: str | None
    control_log_path: Path | None
    drive_parameter_source_path: Path | None
    used_fallback_drive_parameters: bool
    track_width_m: float
    wheel_radius_m: float
    yaw_inertia_kg_m2: float
    configured_equivalent_wheel_inertia_kg_m2: float
    rigid_body_equivalent_inertia_kg_m2: float
    repeat_summaries: list[YawFftRepeatSummary]
    aggregate: YawFftAggregateSummary | None


@dataclass
class YawFftParameters:
    run_id: str | None
    drive: RunPlantParameters
    drive_parameter_source_path: Path
    used_fallback_drive_parameters: bool
    track_width_m: float
    wheel_radius_m: float
    yaw_inertia_kg_m2: float
    configured_equivalent_wheel_inertia_kg_m2: float


def _phase_deg(value: complex) -> float:
    degrees = math.degrees(cmath.phase(value))
    while degrees <= -180.0:
        degrees += 360.0
    while degrees > 180.0:
        degrees -= 360.0
    return degrees


def _next_power_of_two(value: int) -> int:
    power = 1
    while power < value:
        power <<= 1
    return power


def _fft(values: list[complex]) -> list[complex]:
    sample_count = len(values)
    output = list(values)
    bit_reversed_index = 0
    for sample_index in range(1, sample_count):
        bit = sample_count >> 1
        while bit_reversed_index & bit:
            bit_reversed_index ^= bit
            bit >>= 1
        bit_reversed_index ^= bit
        if sample_index < bit_reversed_index:
            output[sample_index], output[bit_reversed_index] = (
                output[bit_reversed_index],
                output[sample_index],
            )

    size = 2
    while size <= sample_count:
        twiddle_increment = cmath.exp((-2.0j * math.pi) / size)
        half_size = size >> 1
        for offset in range(0, sample_count, size):
            twiddle = 1.0 + 0.0j
            for index in range(half_size):
                odd = twiddle * output[offset + index + half_size]
                even = output[offset + index]
                output[offset + index] = even + odd
                output[offset + index + half_size] = even - odd
                twiddle *= twiddle_increment
        size <<= 1
    return output


def _centered_moving_average(values: list[float], window: int) -> list[float]:
    prefix = [0.0]
    for value in values:
        prefix.append(prefix[-1] + value)

    result = [0.0] * len(values)
    half_window = window // 2
    for index in range(len(values)):
        start = max(0, index - half_window)
        end = min(len(values), index + half_window + 1)
        result[index] = (prefix[end] - prefix[start]) / (end - start)
    return result


def _hann_window(sample_count: int) -> list[float]:
    if sample_count <= 1:
        return [1.0]
    return [
        0.5 - (0.5 * math.cos((2.0 * math.pi * index) / (sample_count - 1)))
        for index in range(sample_count)
    ]


def _finite_median(values: list[float]) -> float | None:
    finite_values = [value for value in values if math.isfinite(value)]
    return statistics.median(finite_values) if finite_values else None


def _extract_mass_geometry_fields(path: Path | None) -> dict[str, float] | None:
    if path is None or not path.is_file():
        return None

    blob = path.read_bytes()
    marker = b"plant_dump_params_mass_geometry:"
    start = blob.find(marker)
    if start < 0:
        return None

    start += len(marker)
    end_candidates = [
        position
        for position in (blob.find(b"\n", start), blob.find(b"\0", start))
        if position >= 0
    ]
    end = min(end_candidates) if end_candidates else min(len(blob), start + 512)
    return parse_key_value_fields(blob[start:end].decode("utf-8", errors="replace"))


def load_yaw_fft_parameters(
    control_log_path: Path | None,
    fallback_control_log_path: Path | None,
) -> YawFftParameters | None:
    geometry_fields = _extract_mass_geometry_fields(control_log_path)
    if geometry_fields is None:
        geometry_fields = _extract_mass_geometry_fields(fallback_control_log_path)
    if geometry_fields is None:
        return None

    required_geometry = {
        "track_width_m",
        "wheel_radius_m",
        "yaw_inertia_kg_m2",
        "equivalent_wheel_inertia_kg_m2",
    }
    if not required_geometry.issubset(geometry_fields):
        return None

    drive = load_control_log_parameters(control_log_path) if control_log_path is not None else None
    drive_parameter_source_path = control_log_path
    used_fallback_drive_parameters = False
    if drive is None and fallback_control_log_path is not None:
        drive = load_control_log_parameters(fallback_control_log_path)
        drive_parameter_source_path = fallback_control_log_path
        used_fallback_drive_parameters = drive is not None
    if drive is None or drive_parameter_source_path is None:
        return None

    return YawFftParameters(
        run_id=load_run_id(control_log_path),
        drive=drive,
        drive_parameter_source_path=drive_parameter_source_path,
        used_fallback_drive_parameters=used_fallback_drive_parameters,
        track_width_m=geometry_fields["track_width_m"],
        wheel_radius_m=geometry_fields["wheel_radius_m"],
        yaw_inertia_kg_m2=geometry_fields["yaw_inertia_kg_m2"],
        configured_equivalent_wheel_inertia_kg_m2=geometry_fields["equivalent_wheel_inertia_kg_m2"],
    )


def summarize_yaw_fft(
    yaw_rows_by_key: dict[tuple[int, int], list[dict[str, str]]],
    control_log_path: Path | None,
    fallback_control_log_path: Path | None,
) -> YawFftSummary | None:
    params = load_yaw_fft_parameters(control_log_path, fallback_control_log_path)
    if params is None or params.track_width_m <= 0.0 or params.wheel_radius_m <= 0.0:
        return None

    turn_gain = (2.0 * params.wheel_radius_m) / params.track_width_m
    rigid_body_equivalent_inertia_kg_m2 = (
        (2.0 * params.wheel_radius_m * params.wheel_radius_m * params.yaw_inertia_kg_m2) /
        (params.track_width_m * params.track_width_m)
    )

    repeat_summaries: list[YawFftRepeatSummary] = []
    for (primitive_id, repeat_index), rows in sorted(yaw_rows_by_key.items()):
        phase_rows = [
            row
            for row in rows
            if int(row["phase_id"]) == 9 and
            int(row["saturation_flags"]) == 0 and
            row_watchdog_flags(row) == 0
        ]
        if len(phase_rows) < FFT_MIN_PHASE_ROWS:
            continue

        wheel_differential_speed_radps: list[float] = []
        yaw_rate_radps: list[float] = []
        differential_motor_torque_nm: list[float] = []
        sample_times_s: list[float] = []
        elapsed_time_s = 0.0
        for row in phase_rows:
            left_wheel_speed_radps = float(row["left_encoder_wheel_speed_radps"])
            right_wheel_speed_radps = float(row["right_encoder_wheel_speed_radps"])
            left_command = float(row["left_drive_command"])
            right_command = float(row["right_drive_command"])

            wheel_differential_speed_radps.append(
                0.5 * (left_wheel_speed_radps - right_wheel_speed_radps)
            )
            yaw_rate_radps.append(float(row["gyro_radps"]))
            differential_motor_torque_nm.append(
                0.5 * (
                    drive_torque_from_command(left_command, left_wheel_speed_radps, params.drive) -
                    drive_torque_from_command(right_command, right_wheel_speed_radps, params.drive)
                )
            )
            elapsed_time_s += 1.0e-6 * int(row["dt_us"])
            sample_times_s.append(elapsed_time_s)

        wheel_speed_high_passed = [
            raw - baseline
            for raw, baseline in zip(
                wheel_differential_speed_radps,
                _centered_moving_average(
                    wheel_differential_speed_radps,
                    FFT_HIGH_PASS_WINDOW_SAMPLES,
                ),
            )
        ]
        yaw_high_passed = [
            raw - baseline
            for raw, baseline in zip(
                yaw_rate_radps,
                _centered_moving_average(yaw_rate_radps, FFT_HIGH_PASS_WINDOW_SAMPLES),
            )
        ]
        torque_high_passed = [
            raw - baseline
            for raw, baseline in zip(
                differential_motor_torque_nm,
                _centered_moving_average(
                    differential_motor_torque_nm,
                    FFT_HIGH_PASS_WINDOW_SAMPLES,
                ),
            )
        ]

        average_dt_s = (
            statistics.fmean(
                current - previous
                for previous, current in zip(sample_times_s[:-1], sample_times_s[1:])
            )
            if len(sample_times_s) > 1
            else 0.001
        )
        if average_dt_s <= 0.0:
            continue

        sample_count = len(phase_rows)
        fft_size = _next_power_of_two(sample_count)
        window = _hann_window(sample_count)

        def spectrum(values: list[float]) -> list[complex]:
            padded = [
                complex(value * scale, 0.0)
                for value, scale in zip(values, window)
            ]
            padded.extend([0.0j] * (fft_size - sample_count))
            return _fft(padded)

        wheel_speed_spectrum = spectrum(wheel_speed_high_passed)
        yaw_spectrum = spectrum(yaw_high_passed)
        torque_spectrum = spectrum(torque_high_passed)
        sample_rate_hz = 1.0 / average_dt_s
        frequency_resolution_hz = sample_rate_hz / fft_size

        band_bins: list[tuple[float, int, float]] = []
        for bin_index in range(1, fft_size // 2):
            frequency_hz = bin_index * frequency_resolution_hz
            if FFT_MIN_FREQUENCY_HZ <= frequency_hz <= FFT_MAX_FREQUENCY_HZ:
                band_bins.append((abs(wheel_speed_spectrum[bin_index]), bin_index, frequency_hz))
        if not band_bins:
            continue

        band_bins.sort(reverse=True)
        peak_amplitude, peak_bin_index, dominant_frequency_hz = band_bins[0]
        median_band_amplitude = statistics.median(amplitude for amplitude, _, _ in band_bins)
        oscillation_prominence = peak_amplitude / max(median_band_amplitude, 1.0e-12)

        torque_bin = torque_spectrum[peak_bin_index]
        wheel_speed_bin = wheel_speed_spectrum[peak_bin_index]
        if abs(torque_bin) <= 1.0e-12 or abs(wheel_speed_bin) <= 1.0e-12:
            continue

        wheel_speed_over_torque = wheel_speed_bin / torque_bin
        yaw_over_rigid_wheel = yaw_spectrum[peak_bin_index] / (turn_gain * wheel_speed_bin)
        angular_frequency_radps = 2.0 * math.pi * dominant_frequency_hz
        torque_over_wheel_speed = 1.0 / wheel_speed_over_torque
        phase_corrected_torque_over_wheel_speed = (
            torque_over_wheel_speed -
            (1.0j * angular_frequency_radps * rigid_body_equivalent_inertia_kg_m2 * yaw_over_rigid_wheel)
        )

        repeat_summaries.append(
            YawFftRepeatSummary(
                primitive_id=primitive_id,
                repeat_index=repeat_index,
                sample_count=sample_count,
                dominant_frequency_hz=dominant_frequency_hz,
                oscillation_prominence=oscillation_prominence,
                strong_peak=oscillation_prominence >= FFT_STRONG_PROMINENCE_THRESHOLD,
                wheel_speed_vs_torque_phase_deg=_phase_deg(wheel_speed_over_torque),
                yaw_vs_rigid_wheel_phase_deg=_phase_deg(yaw_over_rigid_wheel),
                yaw_coupling_magnitude=abs(yaw_over_rigid_wheel),
                raw_total_inertia_kg_m2=torque_over_wheel_speed.imag / angular_frequency_radps,
                rigid_body_subtracted_wheel_inertia_kg_m2=(
                    (torque_over_wheel_speed.imag / angular_frequency_radps) -
                    rigid_body_equivalent_inertia_kg_m2
                ),
                phase_corrected_wheel_inertia_kg_m2=(
                    phase_corrected_torque_over_wheel_speed.imag / angular_frequency_radps
                ),
                raw_damping_nm_per_radps=torque_over_wheel_speed.real,
                phase_corrected_damping_nm_per_radps=phase_corrected_torque_over_wheel_speed.real,
            )
        )

    if not repeat_summaries:
        return None

    strong_repeats = [summary for summary in repeat_summaries if summary.strong_peak]
    aggregate_source = strong_repeats if strong_repeats else repeat_summaries
    recommended_candidates = sorted(
        summary.phase_corrected_wheel_inertia_kg_m2
        for summary in strong_repeats
        if (
            math.isfinite(summary.phase_corrected_wheel_inertia_kg_m2) and
            summary.phase_corrected_wheel_inertia_kg_m2 >= 0.0 and
            summary.dominant_frequency_hz >= FFT_RECOMMENDED_MIN_FREQUENCY_HZ
        )
    )
    trimmed_recommended_candidates = recommended_candidates[
        : max(1, (len(recommended_candidates) + 1) // 2)
    ]
    aggregate = YawFftAggregateSummary(
        repeat_count=len(repeat_summaries),
        strong_repeat_count=len(strong_repeats),
        recommended_candidate_count=len(recommended_candidates),
        dominant_frequency_hz_median=_finite_median(
            [summary.dominant_frequency_hz for summary in aggregate_source]
        ),
        wheel_speed_vs_torque_phase_deg_median=_finite_median(
            [summary.wheel_speed_vs_torque_phase_deg for summary in aggregate_source]
        ),
        yaw_vs_rigid_wheel_phase_deg_median=_finite_median(
            [summary.yaw_vs_rigid_wheel_phase_deg for summary in aggregate_source]
        ),
        yaw_coupling_magnitude_median=_finite_median(
            [summary.yaw_coupling_magnitude for summary in aggregate_source]
        ),
        raw_total_inertia_kg_m2_median=_finite_median(
            [summary.raw_total_inertia_kg_m2 for summary in aggregate_source]
        ),
        phase_corrected_wheel_inertia_kg_m2_median=_finite_median(
            [summary.phase_corrected_wheel_inertia_kg_m2 for summary in aggregate_source]
        ),
        recommended_wheel_inertia_kg_m2=(
            statistics.median(trimmed_recommended_candidates)
            if trimmed_recommended_candidates
            else max(
                0.0,
                _finite_median(
                    [summary.phase_corrected_wheel_inertia_kg_m2 for summary in aggregate_source]
                ) or 0.0,
            )
        ),
    )

    return YawFftSummary(
        run_id=params.run_id,
        control_log_path=control_log_path,
        drive_parameter_source_path=params.drive_parameter_source_path,
        used_fallback_drive_parameters=params.used_fallback_drive_parameters,
        track_width_m=params.track_width_m,
        wheel_radius_m=params.wheel_radius_m,
        yaw_inertia_kg_m2=params.yaw_inertia_kg_m2,
        configured_equivalent_wheel_inertia_kg_m2=params.configured_equivalent_wheel_inertia_kg_m2,
        rigid_body_equivalent_inertia_kg_m2=rigid_body_equivalent_inertia_kg_m2,
        repeat_summaries=repeat_summaries,
        aggregate=aggregate,
    )
