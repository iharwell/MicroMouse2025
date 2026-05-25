#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LOG_DIRS = [
    REPO_ROOT / "TestResults" / "mmlog_decode_2026-04-20_08-38-39",
    REPO_ROOT / "TestResults" / "mmlog_decode_2026-04-20_10-22-09",
    REPO_ROOT / "TestResults" / "mmlog_decode_2026-04-20_12-10-58",
    REPO_ROOT / "TestResults" / "mmlog_decode_2026-04-21_01-09-34",
]


@dataclass(frozen=True)
class SourceParams:
    mass_kg: float
    track_width_m: float
    yaw_inertia_kg_m2: float
    wheel_radius_m: float
    wheel_bank_inertia_kg_m2: float
    drive_voltage_v: float
    drive_resistance_ohms: float
    torque_constant_nm_per_a: float
    speed_constant_radps_per_volt: float
    no_load_current_a: float
    gear_ratio: float
    rolling_friction_torque_nm: float
    static_launch_command: float
    static_friction_max_speed_mps: float
    yaw_rate_damping_nms_per_rad: float


@dataclass(frozen=True)
class RunWindow:
    run_dir: Path
    run_id: str
    csv_path: Path
    logging_path: Path | None
    input_rows: int
    kept_rows: int
    cutoff_reason: str
    cutoff_key: tuple[str, str, str, str] | None
    cutoff_first_tick: int | None
    cutoff_first_time_us: int | None


@dataclass(frozen=True)
class Sample:
    run_id: str
    csv_path: Path
    section_id: int
    phase_id: int
    primitive_id: int
    repeat_index: int
    speed_bin: str
    tick: int
    dt_s: float
    yaw_radps: float
    next_yaw_radps: float
    left_velocity_mps: float
    right_velocity_mps: float
    left_wheel_speed_radps: float
    right_wheel_speed_radps: float
    left_command: float
    right_command: float


@dataclass(frozen=True)
class FitResult:
    proposed_track_width_m: float
    proposed_yaw_inertia_kg_m2: float
    proposed_yaw_rate_damping_nms_per_rad: float
    fitted_denominator_kg_m2: float
    track_sample_count: int
    alpha_sample_count: int
    current_rmse_radps: float
    proposed_rmse_radps: float
    current_alpha_rmse_radps2: float
    proposed_alpha_rmse_radps2: float


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def const_float(text: str, name: str) -> float:
    pattern = rf"\b{name}\s*=\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)[fF]?"
    match = re.search(pattern, text)
    if not match:
        raise ValueError(f"Could not find {name}")
    return float(match.group(1))


def converted_const_float(text: str, name: str, function_name: str, scale: float) -> float:
    pattern = rf"\b{name}\s*=\s*{function_name}\(\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)[fF]?\s*\)"
    match = re.search(pattern, text)
    if not match:
        raise ValueError(f"Could not find {name}")
    return scale * float(match.group(1))


def source_params() -> SourceParams:
    vehicle_h = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "Vehicle.h")
    motor_h = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "MotorEncoderDrive.h")
    plant_h = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "PlantModel.h")

    wheel_diameter_m = const_float(vehicle_h, "kDriveWheelDiameterM")
    return SourceParams(
        mass_kg=const_float(vehicle_h, "kPhysicalMassKg"),
        track_width_m=const_float(vehicle_h, "kPhysicalTrackWidthM"),
        yaw_inertia_kg_m2=const_float(vehicle_h, "kPhysicalYawInertiaKgM2"),
        wheel_radius_m=0.5 * wheel_diameter_m,
        wheel_bank_inertia_kg_m2=const_float(motor_h, "kDefaultWheelBankEquivalentInertiaKgM2"),
        drive_voltage_v=const_float(vehicle_h, "kDriveSupplyVoltageV"),
        drive_resistance_ohms=const_float(vehicle_h, "kDriveResistanceOhms"),
        torque_constant_nm_per_a=converted_const_float(
            vehicle_h,
            "kDriveTorqueConstantNmPerA",
            "MilliNewtonMetersToNewtonMeters",
            1.0e-3,
        ),
        speed_constant_radps_per_volt=(
            (const_float(vehicle_h, "kDriveNominalNoLoadSpeedRpm") * (2.0 * math.pi / 60.0)) /
            (const_float(vehicle_h, "kDriveNominalVoltageV") -
             (converted_const_float(vehicle_h, "kDriveNoLoadCurrentA", "MilliAmpsToAmps", 1.0e-3) *
              const_float(vehicle_h, "kDriveResistanceOhms")))
        ),
        no_load_current_a=converted_const_float(vehicle_h, "kDriveNoLoadCurrentA", "MilliAmpsToAmps", 1.0e-3),
        gear_ratio=56.0 / 17.0,
        rolling_friction_torque_nm=const_float(plant_h, "kRollingFrictionTorqueNm"),
        static_launch_command=const_float(plant_h, "kReliableLaunchDriveCommand"),
        static_friction_max_speed_mps=const_float(plant_h, "kStaticFrictionMaxSpeedMps"),
        yaw_rate_damping_nms_per_rad=const_float(plant_h, "kYawRateDampingNmsPerRad"),
    )


def run_id_from_sidecar(csv_path: Path) -> str | None:
    sidecar = csv_path.with_suffix(".sidecar")
    if not sidecar.is_file():
        return None
    for line in read_text(sidecar).splitlines():
        if line.startswith("run_id="):
            return line.split("=", 1)[1].strip() or None
    return None


def run_id_from_logging(logging_path: Path | None) -> str | None:
    if logging_path is None or not logging_path.is_file():
        return None
    text = read_text(logging_path).split("\0", 1)[0]
    match = re.search(r"open_floor_[^\n]*run_id=([^;\s]+)", text)
    if not match:
        match = re.search(r"run_id=(ofm_[^;\s]+)", text)
    if not match:
        match = re.search(r"run_id=([^;\s]+)", text)
    return match.group(1) if match else None


def terminal_fault_time_us(logging_path: Path | None) -> int | None:
    if logging_path is None or not logging_path.is_file():
        return None
    last: int | None = None
    text = read_text(logging_path).split("\0", 1)[0]
    for match in re.finditer(r"\[(\d+)\].*fault", text):
        last = int(match.group(1))
    return last


def terminal_fault_key(logging_path: Path | None) -> tuple[str, str, str, str] | None:
    if logging_path is None or not logging_path.is_file():
        return None
    result = None
    for line in read_text(logging_path).split("\0", 1)[0].splitlines():
        if "fault:" not in line:
            continue
        fields: dict[str, str] = {}
        for part in line.split(";"):
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            fields[key.rsplit(" ", 1)[-1].strip()] = value.strip()
        if {"section_id", "primitive_id", "speed_bin", "repeat_index"}.issubset(fields):
            result = (
                fields["section_id"],
                fields["primitive_id"],
                fields["speed_bin"],
                fields["repeat_index"],
            )
    return result


def row_key(row: dict[str, str]) -> tuple[str, str, str, str]:
    return (
        row.get("section_id", ""),
        row.get("primitive_id", ""),
        row.get("speed_bin", ""),
        row.get("repeat_index", ""),
    )


def load_rows_with_window(run_dir: Path) -> tuple[RunWindow, list[dict[str, str]]]:
    csv_path = run_dir / "open_floor_main.csv"
    logging_path = run_dir / "logging.txt"
    if not csv_path.is_file():
        raise FileNotFoundError(csv_path)
    if not logging_path.is_file():
        logging_path = None

    with csv_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    cutoff_reason = "no terminal fault found; kept all rows"
    cutoff_key = terminal_fault_key(logging_path)
    fault_time = terminal_fault_time_us(logging_path)
    cutoff_index = len(rows)

    if rows and cutoff_key is not None:
        matching = [index for index, row in enumerate(rows) if row_key(row) == cutoff_key]
        if matching:
            cutoff_index = matching[0]
            cutoff_reason = "dropped terminal fault segment from first matching section/primitive/speed/repeat key"

    if rows and cutoff_index == len(rows) and fault_time is not None:
        prior_indices = [
            index
            for index, row in enumerate(rows)
            if int(row.get("master_time_us", "0")) <= fault_time
        ]
        if prior_indices:
            last_index = prior_indices[-1]
            final_key = row_key(rows[last_index])
            start = last_index
            while start > 0 and row_key(rows[start - 1]) == final_key:
                start -= 1
            cutoff_index = start
            cutoff_key = final_key
            cutoff_reason = "dropped trailing terminal segment before fault timestamp"

    kept_rows = rows[:cutoff_index]
    first_dropped = rows[cutoff_index] if cutoff_index < len(rows) else None
    run_id = (
        run_id_from_sidecar(csv_path) or
        run_id_from_logging(logging_path) or
        run_dir.name
    )
    return (
        RunWindow(
            run_dir=run_dir,
            run_id=run_id,
            csv_path=csv_path,
            logging_path=logging_path,
            input_rows=len(rows),
            kept_rows=len(kept_rows),
            cutoff_reason=cutoff_reason,
            cutoff_key=cutoff_key,
            cutoff_first_tick=(int(first_dropped["control_tick_sequence"]) if first_dropped else None),
            cutoff_first_time_us=(int(first_dropped["master_time_us"]) if first_dropped else None),
        ),
        kept_rows,
    )


def finite_float(row: dict[str, str], key: str) -> float:
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(key)
    return value


def first_finite_float(row: dict[str, str], *keys: str) -> float:
    for key in keys:
        if key in row:
            return finite_float(row, key)
    raise KeyError(keys[0])


def stationary_gyro_bias(rows: list[dict[str, str]]) -> float:
    static_values = [
        finite_float(row, "gyro_raw_radps")
        for row in rows
        if row.get("section_id") == "1" and row.get("primitive_id") in {"0", "2"}
    ]
    return statistics.fmean(static_values) if static_values else 0.0


def samples_from_rows(window: RunWindow, rows: list[dict[str, str]]) -> list[Sample]:
    bias = stationary_gyro_bias(rows)
    samples: list[Sample] = []
    for current, nxt in zip(rows[:-1], rows[1:]):
        try:
            section_id = int(current["section_id"])
            phase_id = int(current["phase_id"])
            next_section_id = int(nxt["section_id"])
            primitive_id = int(current["primitive_id"])
            repeat_index = int(current["repeat_index"])
            if row_key(current) != row_key(nxt):
                continue
            if section_id != next_section_id:
                continue
            if section_id != 4 or phase_id not in {8, 9}:
                continue
            if int(current["saturation_flags"]) != 0 or int(nxt["saturation_flags"]) != 0:
                continue
            if int(current.get("watchdog_flags", "0")) != 0 or int(nxt.get("watchdog_flags", "0")) != 0:
                continue
            dt_s = 1.0e-6 * int(nxt["dt_us"])
            if not (0.0005 <= dt_s <= 0.003):
                continue
            left_velocity_mps = finite_float(current, "left_encoder_velocity_mps")
            right_velocity_mps = finite_float(current, "right_encoder_velocity_mps")
            yaw_radps = finite_float(current, "gyro_raw_radps") - bias
            next_yaw_radps = finite_float(nxt, "gyro_raw_radps") - bias
            left_wheel_speed_radps = first_finite_float(
                current,
                "left_encoder_wheel_speed_radps",
                "left_encoder_omega_radps",
            )
            right_wheel_speed_radps = first_finite_float(
                current,
                "right_encoder_wheel_speed_radps",
                "right_encoder_omega_radps",
            )
            left_command = finite_float(current, "left_drive_command")
            right_command = finite_float(current, "right_drive_command")
        except (KeyError, ValueError):
            continue
        if not all(math.isfinite(value) for value in (
            dt_s,
            yaw_radps,
            next_yaw_radps,
            left_velocity_mps,
            right_velocity_mps,
            left_wheel_speed_radps,
            right_wheel_speed_radps,
            left_command,
            right_command,
        )):
            continue
        samples.append(
            Sample(
                run_id=window.run_id,
                csv_path=window.csv_path,
                section_id=section_id,
                phase_id=phase_id,
                primitive_id=primitive_id,
                repeat_index=repeat_index,
                speed_bin=current["speed_bin"],
                tick=int(current["control_tick_sequence"]),
                dt_s=dt_s,
                yaw_radps=yaw_radps,
                next_yaw_radps=next_yaw_radps,
                left_velocity_mps=left_velocity_mps,
                right_velocity_mps=right_velocity_mps,
                left_wheel_speed_radps=left_wheel_speed_radps,
                right_wheel_speed_radps=right_wheel_speed_radps,
                left_command=left_command,
                right_command=right_command,
            )
        )
    return samples


def signed_direction(preferred_value: float, fallback_value: float = 0.0) -> float:
    eps = 1.0e-6
    if preferred_value > eps:
        return 1.0
    if preferred_value < -eps:
        return -1.0
    if fallback_value > eps:
        return 1.0
    if fallback_value < -eps:
        return -1.0
    return 0.0


def torque_from_command(command: float, wheel_speed_radps: float, params: SourceParams) -> float:
    applied_voltage_v = max(-1.0, min(1.0, command)) * params.drive_voltage_v
    back_emf_voltage_v = (wheel_speed_radps * params.gear_ratio) / params.speed_constant_radps_per_volt
    armature_current_a = (applied_voltage_v - back_emf_voltage_v) / params.drive_resistance_ohms
    no_load_direction = signed_direction(armature_current_a, wheel_speed_radps)
    load_current_a = armature_current_a - (no_load_direction * params.no_load_current_a)
    if no_load_direction > 0.0 and load_current_a < 0.0:
        load_current_a = 0.0
    elif no_load_direction < 0.0 and load_current_a > 0.0:
        load_current_a = 0.0
    return params.torque_constant_nm_per_a * params.gear_ratio * load_current_a


def applied_torque(command: float, wheel_speed_radps: float, params: SourceParams) -> float:
    direct = torque_from_command(command, wheel_speed_radps, params)
    positive_limit = max(0.0, torque_from_command(1.0, wheel_speed_radps, params))
    negative_limit = min(0.0, torque_from_command(-1.0, wheel_speed_radps, params))
    limited = max(negative_limit, min(positive_limit, direct)) if positive_limit > negative_limit else direct
    static_launch_torque_nm = max(0.0, torque_from_command(params.static_launch_command, 0.0, params))
    surface_speed_mps = params.wheel_radius_m * wheel_speed_radps
    slow_ratio = abs(surface_speed_mps) / params.static_friction_max_speed_mps
    launch_torque_nm = static_launch_torque_nm * math.exp(-(slow_ratio * slow_ratio))
    output = 0.0
    direction = signed_direction(limited, wheel_speed_radps)
    if abs(limited) > launch_torque_nm:
        output = limited - (direction * launch_torque_nm)
    loss_direction = signed_direction(wheel_speed_radps, output)
    return output - (params.rolling_friction_torque_nm * loss_direction)


def yaw_moment_nm(sample: Sample, track_width_m: float, params: SourceParams) -> float:
    left_torque = applied_torque(sample.left_command, sample.left_wheel_speed_radps, params)
    right_torque = applied_torque(sample.right_command, sample.right_wheel_speed_radps, params)
    return 0.5 * abs(track_width_m) * ((left_torque / params.wheel_radius_m) - (right_torque / params.wheel_radius_m))


def yaw_denominator_kg_m2(track_width_m: float, yaw_inertia_kg_m2: float, params: SourceParams) -> float:
    wheel_spinup_mass_kg = (2.0 * params.wheel_bank_inertia_kg_m2) / (params.wheel_radius_m * params.wheel_radius_m)
    half_track = 0.5 * abs(track_width_m)
    return yaw_inertia_kg_m2 + (wheel_spinup_mass_kg * half_track * half_track)


def predicted_next_yaw(
    sample: Sample,
    track_width_m: float,
    yaw_inertia_kg_m2: float,
    yaw_damping_nms_per_rad: float,
    params: SourceParams,
) -> float:
    denom = yaw_denominator_kg_m2(track_width_m, yaw_inertia_kg_m2, params)
    moment = yaw_moment_nm(sample, track_width_m, params)
    alpha = (moment - (yaw_damping_nms_per_rad * sample.yaw_radps)) / denom
    return sample.yaw_radps + (alpha * sample.dt_s)


def rmse(values: Iterable[float]) -> float:
    values = list(values)
    if not values:
        return 0.0
    return math.sqrt(statistics.fmean(value * value for value in values))


def huber_track_width(samples: list[Sample]) -> tuple[float, int]:
    pairs = [
        (sample.yaw_radps, sample.left_velocity_mps - sample.right_velocity_mps)
        for sample in samples
        if abs(sample.yaw_radps) >= 0.25
    ]
    if len(pairs) < 8:
        raise ValueError("not enough yaw/encoder samples for track-width fit")
    weight_pairs = [(yaw, diff, max(abs(yaw), 0.25)) for yaw, diff in pairs]
    beta = sum(w * yaw * diff for yaw, diff, w in weight_pairs) / sum(w * yaw * yaw for yaw, _, w in weight_pairs)
    for _ in range(5):
        residuals = [diff - (beta * yaw) for yaw, diff, _ in weight_pairs]
        mad = statistics.median(abs(value) for value in residuals)
        scale = max(1.0e-6, 1.4826 * mad)
        numerator = 0.0
        denominator = 0.0
        for (yaw, diff, base_weight), residual in zip(weight_pairs, residuals):
            weight = base_weight * min(1.0, 1.5 * scale / max(abs(residual), 1.0e-9))
            numerator += weight * yaw * diff
            denominator += weight * yaw * yaw
        if denominator <= 0.0:
            break
        beta = numerator / denominator
    return abs(beta), len(pairs)


def least_squares_two_columns(rows: list[tuple[float, float, float]]) -> tuple[float, float]:
    s11 = sum(x1 * x1 for x1, _, _ in rows)
    s12 = sum(x1 * x2 for x1, x2, _ in rows)
    s22 = sum(x2 * x2 for _, x2, _ in rows)
    b1 = sum(x1 * y for x1, _, y in rows)
    b2 = sum(x2 * y for _, x2, y in rows)
    det = (s11 * s22) - (s12 * s12)
    if abs(det) <= 1.0e-18:
        raise ValueError("singular alpha fit")
    return ((b1 * s22 - b2 * s12) / det, (s11 * b2 - s12 * b1) / det)


def fit_alpha_params(samples: list[Sample], track_width_m: float, params: SourceParams) -> tuple[float, float, int]:
    rows = []
    for sample in samples:
        measured_alpha = (sample.next_yaw_radps - sample.yaw_radps) / sample.dt_s
        moment = yaw_moment_nm(sample, track_width_m, params)
        if abs(measured_alpha) < 10.0 and abs(moment) < 1.0e-6:
            continue
        rows.append((moment, -sample.yaw_radps, measured_alpha))
    if len(rows) < 8:
        raise ValueError("not enough yaw-acceleration samples")
    theta_moment, theta_damping = least_squares_two_columns(rows)
    for _ in range(3):
        residuals = [y - ((theta_moment * m) + (theta_damping * neg_yaw)) for m, neg_yaw, y in rows]
        abs_residuals = sorted(abs(value) for value in residuals)
        trim = abs_residuals[int(0.90 * (len(abs_residuals) - 1))]
        kept = [row for row, residual in zip(rows, residuals) if abs(residual) <= trim]
        theta_moment, theta_damping = least_squares_two_columns(kept)
        rows = kept
    if theta_moment <= 0.0:
        raise ValueError("yaw moment fit resolved non-positive inverse inertia")
    denominator = 1.0 / theta_moment
    damping = theta_damping / theta_moment
    return denominator, max(0.0, damping), len(rows)


def fit(samples: list[Sample], params: SourceParams) -> FitResult:
    proposed_track_width_m, track_count = huber_track_width(samples)
    denominator, damping, alpha_count = fit_alpha_params(samples, proposed_track_width_m, params)
    wheel_spinup_mass_kg = (2.0 * params.wheel_bank_inertia_kg_m2) / (params.wheel_radius_m * params.wheel_radius_m)
    half_track = 0.5 * proposed_track_width_m
    proposed_yaw_inertia = max(1.0e-8, denominator - (wheel_spinup_mass_kg * half_track * half_track))

    current_errors = [
        predicted_next_yaw(
            sample,
            params.track_width_m,
            params.yaw_inertia_kg_m2,
            params.yaw_rate_damping_nms_per_rad,
            params,
        ) - sample.next_yaw_radps
        for sample in samples
    ]
    proposed_errors = [
        predicted_next_yaw(
            sample,
            proposed_track_width_m,
            proposed_yaw_inertia,
            damping,
            params,
        ) - sample.next_yaw_radps
        for sample in samples
    ]
    current_alpha_errors = [
        error / sample.dt_s
        for error, sample in zip(current_errors, samples)
    ]
    proposed_alpha_errors = [
        error / sample.dt_s
        for error, sample in zip(proposed_errors, samples)
    ]
    return FitResult(
        proposed_track_width_m=proposed_track_width_m,
        proposed_yaw_inertia_kg_m2=proposed_yaw_inertia,
        proposed_yaw_rate_damping_nms_per_rad=damping,
        fitted_denominator_kg_m2=denominator,
        track_sample_count=track_count,
        alpha_sample_count=alpha_count,
        current_rmse_radps=rmse(current_errors),
        proposed_rmse_radps=rmse(proposed_errors),
        current_alpha_rmse_radps2=rmse(current_alpha_errors),
        proposed_alpha_rmse_radps2=rmse(proposed_alpha_errors),
    )


def markdown_report(
    params: SourceParams,
    windows: list[RunWindow],
    samples: list[Sample],
    result: FitResult,
) -> str:
    by_run: dict[str, int] = {}
    for sample in samples:
        by_run[sample.run_id] = by_run.get(sample.run_id, 0) + 1
    current_by_run: dict[str, list[float]] = {}
    proposed_by_run: dict[str, list[float]] = {}
    for sample in samples:
        current_by_run.setdefault(sample.run_id, []).append(
            predicted_next_yaw(
                sample,
                params.track_width_m,
                params.yaw_inertia_kg_m2,
                params.yaw_rate_damping_nms_per_rad,
                params,
            ) - sample.next_yaw_radps
        )
        proposed_by_run.setdefault(sample.run_id, []).append(
            predicted_next_yaw(
                sample,
                result.proposed_track_width_m,
                result.proposed_yaw_inertia_kg_m2,
                result.proposed_yaw_rate_damping_nms_per_rad,
                params,
            ) - sample.next_yaw_radps
        )
    lines = [
        "# Yaw Plant Parameter Fit",
        "",
        "## Analysis Path",
        "",
        "This scratch analysis uses decoded `open_floor_main.csv` logs directly. It uses raw gyro minus the run's stationary raw-gyro mean, encoder velocities, encoder wheel speeds, and logged drive commands. It does not use UKF state estimates as fit targets.",
        "",
        "The fitted one-step yaw model mirrors the yaw-relevant PlantModel terms needed for in-place yaw sections: motor command to wheel-bank torque, static/rolling drive losses, differential drive yaw moment, effective yaw inertia including wheel spin-up, and yaw-rate damping.",
        "",
        "Reproduce with:",
        "",
        "```powershell",
        "python codex_analysis\\yaw_fit\\fit_yaw_plant_params.py",
        "```",
        "",
        "## Logs And Windows",
        "",
        "| Run ID | CSV | Input rows | Kept rows | Fit samples | Cutoff | First dropped tick/time |",
        "| --- | --- | ---: | ---: | ---: | --- | --- |",
    ]
    for window in windows:
        dropped = (
            ""
            if window.cutoff_first_tick is None
            else f"{window.cutoff_first_tick} / {window.cutoff_first_time_us}"
        )
        lines.append(
            f"| `{window.run_id}` | `{window.csv_path.relative_to(REPO_ROOT)}` | {window.input_rows} | {window.kept_rows} | {by_run.get(window.run_id, 0)} | {window.cutoff_reason} | {dropped} |"
        )
    lines.extend([
        "",
        "Fit rows are limited to `SEC_40_YAW` phase `8` active rotation and phase `9` stop/decay rows, with zero saturation flags, zero watchdog flags when present, same section/primitive/speed/repeat on adjacent samples, and `0.5..3.0 ms` sample intervals.",
        "",
        "## Current Vs Proposed",
        "",
        "| Parameter | Current | Proposed / fitted |",
        "| --- | ---: | ---: |",
        f"| `Vehicle::track_width_m` | {params.track_width_m:.9f} | {result.proposed_track_width_m:.9f} |",
        f"| `Vehicle::yaw_inertia_kg_m2` | {params.yaw_inertia_kg_m2:.9f} | {result.proposed_yaw_inertia_kg_m2:.9f} |",
        f"| `PlantModel::yaw_rate_damping_nms_per_rad` | {params.yaw_rate_damping_nms_per_rad:.9f} | {result.proposed_yaw_rate_damping_nms_per_rad:.9f} |",
        f"| fitted yaw denominator including wheel spin-up | {yaw_denominator_kg_m2(params.track_width_m, params.yaw_inertia_kg_m2, params):.9f} | {result.fitted_denominator_kg_m2:.9f} |",
        "",
        "## RMSE",
        "",
        "| Metric | Samples | Current | Proposed |",
        "| --- | ---: | ---: | ---: |",
        f"| one-step yaw-rate RMSE (rad/s) | {len(samples)} | {result.current_rmse_radps:.9f} | {result.proposed_rmse_radps:.9f} |",
        f"| implied yaw-accel RMSE (rad/s^2) | {len(samples)} | {result.current_alpha_rmse_radps2:.3f} | {result.proposed_alpha_rmse_radps2:.3f} |",
        "",
        "## Per-Run One-Step Yaw RMSE",
        "",
        "| Run ID | Samples | Current RMSE (rad/s) | Proposed RMSE (rad/s) |",
        "| --- | ---: | ---: | ---: |",
    ])
    for window in windows:
        run_id = window.run_id
        lines.append(
            f"| `{run_id}` | {by_run.get(run_id, 0)} | {rmse(current_by_run.get(run_id, [])):.9f} | {rmse(proposed_by_run.get(run_id, [])):.9f} |"
        )
    lines.extend([
        "",
        "## Fit Counts",
        "",
        f"- Track-width fit samples: {result.track_sample_count}",
        f"- Yaw-acceleration fit samples after robust trimming: {result.alpha_sample_count}",
    ])
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fit yaw-relevant PlantModel parameters from decoded open-floor logs.")
    parser.add_argument(
        "log_dirs",
        nargs="*",
        type=Path,
        default=DEFAULT_LOG_DIRS,
        help="Directories containing open_floor_main.csv and optional logging.txt.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path(__file__).resolve().parent / "yaw_fit_report.md",
        help="Markdown report path.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    params = source_params()
    windows: list[RunWindow] = []
    samples: list[Sample] = []
    for log_dir in args.log_dirs:
        window, rows = load_rows_with_window(log_dir)
        windows.append(window)
        samples.extend(samples_from_rows(window, rows))
    if not samples:
        raise SystemExit("No yaw fit samples found.")
    result = fit(samples, params)
    report = markdown_report(params, windows, samples, result)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(report, encoding="utf-8")

    print(f"report={args.report}")
    print(f"samples={len(samples)}")
    print(f"track_samples={result.track_sample_count}")
    print(f"alpha_samples={result.alpha_sample_count}")
    print(f"current_track_width_m={params.track_width_m:.9f}")
    print(f"proposed_track_width_m={result.proposed_track_width_m:.9f}")
    print(f"current_yaw_inertia_kg_m2={params.yaw_inertia_kg_m2:.9f}")
    print(f"proposed_yaw_inertia_kg_m2={result.proposed_yaw_inertia_kg_m2:.9f}")
    print(f"current_yaw_rate_damping_nms_per_rad={params.yaw_rate_damping_nms_per_rad:.9f}")
    print(f"proposed_yaw_rate_damping_nms_per_rad={result.proposed_yaw_rate_damping_nms_per_rad:.9f}")
    print(f"current_yaw_rate_rmse_radps={result.current_rmse_radps:.9f}")
    print(f"proposed_yaw_rate_rmse_radps={result.proposed_rmse_radps:.9f}")
    print(f"current_yaw_accel_rmse_radps2={result.current_alpha_rmse_radps2:.3f}")
    print(f"proposed_yaw_accel_rmse_radps2={result.proposed_alpha_rmse_radps2:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
