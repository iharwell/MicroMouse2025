#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent

RUNS = [
    "mmlog_decode_2026-04-21_05-32-06",
    "mmlog_decode_2026-04-20_08-38-39",
    "mmlog_decode_2026-05-04_20-35-47",
    "mmlog_decode_2026-05-04_16-57-53",
    "mmlog_decode_2026-04-21_05-59-46",
    "mmlog_decode_2026-04-21_00-16-10",
    "mmlog_decode_2026-04-21_01-09-34",
    "mmlog_decode_2026-04-20_02-33-07",
    "mmlog_decode_2026-04-20_12-10-58",
    "mmlog_decode_2026-04-14_04-43-48",
    "mmlog_decode_2026-04-20_10-22-09",
]

WORKER_C_RUNS = {
    "mmlog_decode_2026-04-20_08-38-39",
    "mmlog_decode_2026-04-20_10-22-09",
    "mmlog_decode_2026-04-20_12-10-58",
    "mmlog_decode_2026-04-21_01-09-34",
}

V_BINS = [
    (-2.5, -1.0, "-2.5..-1.0"),
    (-1.0, -0.5, "-1.0..-0.5"),
    (-0.5, -0.1, "-0.5..-0.1"),
    (-0.1, 0.1, "-0.1..0.1"),
    (0.1, 0.5, "0.1..0.5"),
    (0.5, 1.0, "0.5..1.0"),
    (1.0, 1.5, "1.0..1.5"),
    (1.5, 2.5, "1.5..2.5"),
]

YAW_ABS_BINS = [
    (0.20, 1.0, "0.2..1"),
    (1.0, 3.0, "1..3"),
    (3.0, 6.0, "3..6"),
    (6.0, 10.0, "6..10"),
    (10.0, 15.0, "10..15"),
    (15.0, 22.0, "15..22"),
    (22.0, 35.0, "22..35"),
]


@dataclass(frozen=True)
class Params:
    track_m: float
    yaw_inertia_kg_m2: float
    wheel_radius_m: float
    wheel_bank_inertia_kg_m2: float
    drive_voltage_v: float
    resistance_ohm: float
    kt_nm_a: float
    kv_radps_v: float
    no_load_a: float
    gear_ratio: float
    rolling_friction_torque_nm: float
    launch_command: float
    static_friction_max_speed_mps: float
    yaw_damping_nms_per_rad: float


@dataclass(frozen=True)
class Row:
    run: str
    t_us: int
    tick: int
    section_id: int | None
    phase_id: int | None
    primitive_id: str
    speed_bin: str
    repeat_index: str
    left_cmd: float
    right_cmd: float
    cmd_yaw: float
    left_vel_mps: float
    right_vel_mps: float
    left_wheel_radps: float
    right_wheel_radps: float
    yaw_radps: float
    saturation_flags: int
    watchdog_flags: int
    imu_age_s: float | None
    encoder_age_s: float | None


@dataclass(frozen=True)
class Sample:
    run: str
    regime: str
    forward_mps: float
    yaw_radps: float
    alpha_radps2: float
    dt_s: float
    commanded_yaw_nm: float
    observed_yaw_nm: float
    residual_counter_yaw_nm: float
    opposing_yaw_nm: float
    command_aligned_with_yaw: bool
    left_cmd: float
    right_cmd: float
    tick: int


@dataclass(frozen=True)
class RunSummary:
    run: str
    rows: int
    kept_rows: int
    samples: int
    in_place_samples: int
    moving_samples: int
    cut_time_us: int | None
    notes: str


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def const_float(text: str, name: str) -> float:
    match = re.search(rf"\b{name}\s*=\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)[fF]?", text)
    if not match:
        raise ValueError(f"Could not find {name}")
    return float(match.group(1))


def converted_const(text: str, name: str, function_name: str, scale: float) -> float:
    match = re.search(rf"\b{name}\s*=\s*{function_name}\(\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)[fF]?\s*\)", text)
    if not match:
        raise ValueError(f"Could not find {name}")
    return scale * float(match.group(1))


def load_params() -> Params:
    vehicle_h = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "Vehicle.h")
    motor_h = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "MotorEncoderDrive.h")
    plant_h = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "PlantModel.h")
    wheel_diam = const_float(vehicle_h, "kDriveWheelDiameterM")
    no_load_a = converted_const(vehicle_h, "kDriveNoLoadCurrentA", "MilliAmpsToAmps", 1.0e-3)
    resistance = const_float(vehicle_h, "kDriveResistanceOhms")
    nominal_voltage = const_float(vehicle_h, "kDriveNominalVoltageV")
    nominal_rpm = const_float(vehicle_h, "kDriveNominalNoLoadSpeedRpm")
    kv = (nominal_rpm * (2.0 * math.pi / 60.0)) / (nominal_voltage - (no_load_a * resistance))
    return Params(
        track_m=const_float(vehicle_h, "kPhysicalTrackWidthM"),
        yaw_inertia_kg_m2=const_float(vehicle_h, "kPhysicalYawInertiaKgM2"),
        wheel_radius_m=0.5 * wheel_diam,
        wheel_bank_inertia_kg_m2=const_float(motor_h, "kDefaultWheelBankEquivalentInertiaKgM2"),
        drive_voltage_v=const_float(vehicle_h, "kDriveSupplyVoltageV"),
        resistance_ohm=resistance,
        kt_nm_a=converted_const(vehicle_h, "kDriveTorqueConstantNmPerA", "MilliNewtonMetersToNewtonMeters", 1.0e-3),
        kv_radps_v=kv,
        no_load_a=no_load_a,
        gear_ratio=const_float(vehicle_h, "kDriveGearRatio"),
        rolling_friction_torque_nm=const_float(plant_h, "kRollingFrictionTorqueNm"),
        launch_command=const_float(plant_h, "kReliableLaunchDriveCommand"),
        static_friction_max_speed_mps=const_float(plant_h, "kStaticFrictionMaxSpeedMps"),
        yaw_damping_nms_per_rad=const_float(plant_h, "kYawRateDampingNmsPerRad"),
    )


def get_float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    if value == "":
        return default
    try:
        result = float(value)
    except ValueError:
        return default
    return result if math.isfinite(result) else default


def get_int(row: dict[str, str], key: str, default: int = 0) -> int:
    value = row.get(key, "")
    if value == "":
        return default
    try:
        return int(float(value))
    except ValueError:
        return default


def get_optional_int(row: dict[str, str], key: str) -> int | None:
    if key not in row or row[key] == "":
        return None
    return get_int(row, key)


def stationary_gyro_bias(raw_rows: list[dict[str, str]]) -> float:
    candidates: list[float] = []
    for row in raw_rows[: min(5000, len(raw_rows))]:
        quiet_command = abs(get_float(row, "left_drive_command")) < 0.03 and abs(get_float(row, "right_drive_command")) < 0.03
        quiet_encoder = abs(get_float(row, "left_encoder_velocity_mps")) < 0.03 and abs(get_float(row, "right_encoder_velocity_mps")) < 0.03
        old_static = row.get("section_id") == "1" and row.get("primitive_id") in {"0", "2", ""}
        if (old_static or (quiet_command and quiet_encoder)) and row.get("gyro_raw_radps", "") != "":
            candidates.append(get_float(row, "gyro_raw_radps"))
    if len(candidates) >= 20:
        return statistics.median(candidates)
    fallback = [get_float(row, "gyro_raw_radps") for row in raw_rows[: min(1000, len(raw_rows))] if row.get("gyro_raw_radps", "") != ""]
    return statistics.median(fallback) if fallback else 0.0


def terminal_fault_time_us(run_dir: Path) -> int | None:
    logging = run_dir / "logging.txt"
    if not logging.is_file():
        return None
    last = None
    for match in re.finditer(r"\[(\d+)\].*fault", read_text(logging).split("\0", 1)[0], re.IGNORECASE):
        last = int(match.group(1))
    return last


def find_bad_tail(rows: list[Row]) -> tuple[int | None, str]:
    run_length = 0
    first_bad_index = 0
    first_bad_time = None
    first_reason = ""
    terminal_index_floor = int(0.70 * len(rows))
    ignored_early_reason = ""
    for index, row in enumerate(rows):
        sensor_stale = (
            (row.imu_age_s is not None and row.imu_age_s > 0.050) or
            (row.encoder_age_s is not None and row.encoder_age_s > 0.050)
        )
        uncommanded_motion = (
            abs(row.left_cmd) < 0.03 and abs(row.right_cmd) < 0.03 and
            (abs(row.left_vel_mps) > 0.12 or abs(row.right_vel_mps) > 0.12 or abs(row.yaw_radps) > 0.80)
        )
        commanded_stall = (
            abs(row.left_cmd - row.right_cmd) > 0.25 and
            abs(row.left_vel_mps - row.right_vel_mps) < 0.025 and
            abs(row.yaw_radps) < 0.20
        )
        bad = sensor_stale or uncommanded_motion or commanded_stall
        if bad:
            if run_length == 0:
                first_bad_index = index
                first_bad_time = row.t_us
                first_reason = (
                    "sensor freshness failure" if sensor_stale else
                    "uncommanded motion" if uncommanded_motion else
                    "commanded stall/lift-like window"
                )
            run_length += 1
            if run_length >= 100:
                if first_bad_index >= terminal_index_floor:
                    return max(0, (first_bad_time or row.t_us) - 200_000), first_reason
                ignored_early_reason = f"ignored early {first_reason}"
                run_length = 0
                first_bad_time = None
                first_reason = ""
        else:
            run_length = 0
            first_bad_time = None
            first_reason = ""
    return None, ignored_early_reason


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


def torque_from_command(command: float, wheel_speed_radps: float, params: Params) -> float:
    applied_voltage = max(-1.0, min(1.0, command)) * params.drive_voltage_v
    back_emf_voltage = (wheel_speed_radps * params.gear_ratio) / params.kv_radps_v
    armature_current = (applied_voltage - back_emf_voltage) / params.resistance_ohm
    no_load_direction = signed_direction(armature_current, wheel_speed_radps)
    load_current = armature_current - (no_load_direction * params.no_load_a)
    if no_load_direction > 0.0 and load_current < 0.0:
        load_current = 0.0
    elif no_load_direction < 0.0 and load_current > 0.0:
        load_current = 0.0
    return params.kt_nm_a * params.gear_ratio * load_current


def applied_bank_torque(command: float, wheel_speed_radps: float, params: Params) -> float:
    direct = torque_from_command(command, wheel_speed_radps, params)
    positive_limit = max(0.0, torque_from_command(1.0, wheel_speed_radps, params))
    negative_limit = min(0.0, torque_from_command(-1.0, wheel_speed_radps, params))
    limited = max(negative_limit, min(positive_limit, direct)) if positive_limit > negative_limit else direct
    static_launch = max(0.0, torque_from_command(params.launch_command, 0.0, params))
    surface_speed = params.wheel_radius_m * wheel_speed_radps
    slow_ratio = abs(surface_speed) / params.static_friction_max_speed_mps if params.static_friction_max_speed_mps > 0.0 else 0.0
    launch_torque = static_launch * math.exp(-(slow_ratio * slow_ratio))
    output = 0.0
    direction = signed_direction(limited, wheel_speed_radps)
    if abs(limited) > launch_torque:
        output = limited - (direction * launch_torque)
    loss_direction = signed_direction(wheel_speed_radps, output)
    return output - (params.rolling_friction_torque_nm * loss_direction)


def yaw_denominator(params: Params) -> float:
    wheel_spinup_mass = (2.0 * params.wheel_bank_inertia_kg_m2) / (params.wheel_radius_m * params.wheel_radius_m)
    return params.yaw_inertia_kg_m2 + (wheel_spinup_mass * (0.5 * params.track_m) ** 2)


def commanded_yaw_moment(row: Row, params: Params) -> float:
    left_torque = applied_bank_torque(row.left_cmd, row.left_wheel_radps, params)
    right_torque = applied_bank_torque(row.right_cmd, row.right_wheel_radps, params)
    left_force = left_torque / params.wheel_radius_m
    right_force = right_torque / params.wheel_radius_m
    return 0.5 * abs(params.track_m) * (left_force - right_force)


def regime_for(row: Row) -> str | None:
    if row.section_id == 4 or (row.section_id is None and row.phase_id in {4, 20}):
        return "in_place"
    if row.section_id in {5, 6, 7} or (row.section_id is None and row.phase_id in {5, 6, 7}):
        return "moving"
    return None


def active_yaw_command(row: Row) -> bool:
    return abs(row.left_cmd - row.right_cmd) >= 0.05 or abs(row.cmd_yaw) >= 0.20


def same_segment(a: Row, b: Row) -> bool:
    return (
        a.section_id == b.section_id and
        a.phase_id == b.phase_id and
        a.primitive_id == b.primitive_id and
        a.speed_bin == b.speed_bin and
        a.repeat_index == b.repeat_index
    )


def load_run(run: str, params: Params) -> tuple[RunSummary, list[Sample]]:
    run_dir = REPO_ROOT / "TestResults" / run
    csv_path = run_dir / "open_floor_main.csv"
    with csv_path.open(newline="", encoding="utf-8") as stream:
        raw_rows = list(csv.DictReader(stream))
    bias = stationary_gyro_bias(raw_rows)
    rows: list[Row] = []
    for raw in raw_rows:
        t_us = get_int(raw, "master_time_us")
        imu_ts = get_optional_int(raw, "imu_timestamp_us")
        enc_ts = get_optional_int(raw, "encoder_timestamp_us")
        yaw = get_float(raw, "gyro_raw_radps") - bias if raw.get("gyro_raw_radps", "") != "" else get_float(raw, "gyro_radps")
        rows.append(Row(
            run=run,
            t_us=t_us,
            tick=get_int(raw, "control_tick_sequence"),
            section_id=get_optional_int(raw, "section_id"),
            phase_id=get_optional_int(raw, "phase_id"),
            primitive_id=raw.get("primitive_id", ""),
            speed_bin=raw.get("speed_bin", ""),
            repeat_index=raw.get("repeat_index", ""),
            left_cmd=get_float(raw, "left_drive_command"),
            right_cmd=get_float(raw, "right_drive_command"),
            cmd_yaw=get_float(raw, "cmd_angular_radps"),
            left_vel_mps=get_float(raw, "left_encoder_velocity_mps"),
            right_vel_mps=get_float(raw, "right_encoder_velocity_mps"),
            left_wheel_radps=get_float(raw, "left_encoder_wheel_speed_radps", get_float(raw, "left_encoder_omega_radps")),
            right_wheel_radps=get_float(raw, "right_encoder_wheel_speed_radps", get_float(raw, "right_encoder_omega_radps")),
            yaw_radps=yaw,
            saturation_flags=get_int(raw, "saturation_flags"),
            watchdog_flags=get_int(raw, "watchdog_flags"),
            imu_age_s=(1.0e-6 * max(0, t_us - imu_ts)) if imu_ts is not None else None,
            encoder_age_s=(1.0e-6 * max(0, t_us - enc_ts)) if enc_ts is not None else None,
        ))

    notes: list[str] = []
    cut_time, cut_reason = find_bad_tail(rows)
    if cut_time is not None:
        notes.append(cut_reason)
    fault_time = terminal_fault_time_us(run_dir)
    if fault_time is not None:
        fault_cut = max(0, fault_time - 200_000)
        cut_time = min(cut_time, fault_cut) if cut_time is not None else fault_cut
        notes.append("terminal fault upper bound")
    kept = [row for row in rows if cut_time is None or row.t_us <= cut_time]

    denominator = yaw_denominator(params)
    samples: list[Sample] = []
    window_steps = 5
    for start in range(0, max(0, len(kept) - window_steps)):
        cur = kept[start]
        nxt = kept[start + window_steps]
        regime = regime_for(cur)
        if regime is None or regime_for(nxt) != regime:
            continue
        window = kept[start:start + window_steps + 1]
        if any(not same_segment(cur, row) for row in window[1:]):
            continue
        if any(row.saturation_flags != 0 for row in window):
            continue
        if any(row.watchdog_flags != 0 for row in window):
            continue
        if not active_yaw_command(cur):
            continue
        if abs(cur.yaw_radps) < 0.20:
            continue
        dt_s = 1.0e-6 * (nxt.t_us - cur.t_us)
        if not (0.0030 <= dt_s <= 0.0100):
            continue
        alpha = (nxt.yaw_radps - cur.yaw_radps) / dt_s
        command_moment = commanded_yaw_moment(cur, params)
        observed_moment = denominator * alpha
        residual_counter = command_moment - observed_moment - (params.yaw_damping_nms_per_rad * cur.yaw_radps)
        opposing = residual_counter * signed_direction(cur.yaw_radps, command_moment)
        samples.append(Sample(
            run=run,
            regime=regime,
            forward_mps=0.5 * (cur.left_vel_mps + cur.right_vel_mps),
            yaw_radps=cur.yaw_radps,
            alpha_radps2=alpha,
            dt_s=dt_s,
            commanded_yaw_nm=command_moment,
            observed_yaw_nm=observed_moment,
            residual_counter_yaw_nm=residual_counter,
            opposing_yaw_nm=opposing,
            command_aligned_with_yaw=(command_moment * signed_direction(cur.yaw_radps) > 0.0),
            left_cmd=cur.left_cmd,
            right_cmd=cur.right_cmd,
            tick=cur.tick,
        ))
    return (
        RunSummary(
            run=run,
            rows=len(rows),
            kept_rows=len(kept),
            samples=len(samples),
            in_place_samples=sum(1 for sample in samples if sample.regime == "in_place"),
            moving_samples=sum(1 for sample in samples if sample.regime == "moving"),
            cut_time_us=cut_time,
            notes=", ".join(dict.fromkeys(notes)),
        ),
        samples,
    )


def bin_label(value: float, bins: list[tuple[float, float, str]]) -> str | None:
    for lo, hi, label in bins:
        if lo <= value < hi:
            return label
    return None


def sample_bin(sample: Sample) -> tuple[str, str] | None:
    v = bin_label(sample.forward_mps, V_BINS)
    yaw = bin_label(abs(sample.yaw_radps), YAW_ABS_BINS)
    if v is None or yaw is None:
        return None
    return v, yaw


def rmse(values: Iterable[float]) -> float:
    data = list(values)
    return math.sqrt(statistics.fmean(value * value for value in data)) if data else 0.0


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    sorted_values = sorted(values)
    index = min(len(sorted_values) - 1, max(0, int(round(q * (len(sorted_values) - 1)))))
    return sorted_values[index]


def build_surface(samples: list[Sample], minimum_count: int = 80, aligned_only: bool = False) -> dict[tuple[str, str], float]:
    bins: dict[tuple[str, str], list[float]] = {}
    for sample in samples:
        if aligned_only and not sample.command_aligned_with_yaw:
            continue
        key = sample_bin(sample)
        if key is None:
            continue
        bins.setdefault(key, []).append(sample.opposing_yaw_nm)
    return {
        key: statistics.median(values)
        for key, values in bins.items()
        if len(values) >= minimum_count
    }


def current_error(sample: Sample, params: Params) -> float:
    predicted_next = sample.yaw_radps + (sample.commanded_yaw_nm / yaw_denominator(params)) * sample.dt_s
    return predicted_next - (sample.yaw_radps + sample.alpha_radps2 * sample.dt_s)


def corrected_error(sample: Sample, params: Params, surface: dict[tuple[str, str], float]) -> float:
    key = sample_bin(sample)
    correction_opposing = surface.get(key, 0.0) if key is not None else 0.0
    residual_counter = correction_opposing * signed_direction(sample.yaw_radps, sample.commanded_yaw_nm)
    corrected_moment = sample.commanded_yaw_nm - residual_counter
    predicted_next = sample.yaw_radps + (corrected_moment / yaw_denominator(params)) * sample.dt_s
    return predicted_next - (sample.yaw_radps + sample.alpha_radps2 * sample.dt_s)


def surface_rows(samples: list[Sample], regime: str, aligned_only: bool) -> list[list[str]]:
    rows: list[list[str]] = []
    selected = [
        sample
        for sample in samples
        if sample.regime == regime and (sample.command_aligned_with_yaw or not aligned_only)
    ]
    for v_label in [label for _, _, label in V_BINS]:
        for yaw_label in [label for _, _, label in YAW_ABS_BINS]:
            values = [
                sample.opposing_yaw_nm
                for sample in selected
                if sample_bin(sample) == (v_label, yaw_label)
            ]
            if not values:
                continue
            positive_fraction = sum(1 for value in values if value > 0.0) / len(values)
            rows.append([
                "aligned" if aligned_only else "all",
                regime,
                v_label,
                yaw_label,
                str(len(values)),
                f"{statistics.median(values):.9f}",
                f"{statistics.fmean(values):.9f}",
                f"{percentile(values, 0.20):.9f}",
                f"{percentile(values, 0.80):.9f}",
                f"{positive_fraction:.3f}",
            ])
    return rows


def write_outputs(params: Params, summaries: list[RunSummary], samples: list[Sample]) -> None:
    in_place = [sample for sample in samples if sample.regime == "in_place"]
    moving = [sample for sample in samples if sample.regime == "moving"]
    all_surface = build_surface(samples)
    aligned_surface = build_surface(samples, aligned_only=True)
    in_place_surface = build_surface(in_place, aligned_only=True)
    moving_surface = build_surface(moving, aligned_only=True)

    surface_csv = OUT_DIR / "counter_yaw_torque_surface_bins.csv"
    with surface_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "selection",
            "regime",
            "forward_mps_bin",
            "abs_yaw_radps_bin",
            "samples",
            "median_opposing_yaw_nm",
            "mean_opposing_yaw_nm",
            "p20_opposing_yaw_nm",
            "p80_opposing_yaw_nm",
            "fraction_positive_opposes_yaw",
        ])
        writer.writerows(surface_rows(samples, "in_place", aligned_only=False))
        writer.writerows(surface_rows(samples, "moving", aligned_only=False))
        writer.writerows(surface_rows(samples, "in_place", aligned_only=True))
        writer.writerows(surface_rows(samples, "moving", aligned_only=True))

    summary_csv = OUT_DIR / "counter_yaw_run_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(RunSummary.__dataclass_fields__.keys())
        for summary in summaries:
            writer.writerow([getattr(summary, field) for field in RunSummary.__dataclass_fields__])

    rmse_rows = []
    for label, subset, surface in [
        ("all_samples_global_surface", samples, all_surface),
        ("in_place_global_surface", in_place, all_surface),
        ("moving_global_surface", moving, all_surface),
        ("all_samples_aligned_surface", samples, aligned_surface),
        ("in_place_aligned_surface", in_place, aligned_surface),
        ("moving_aligned_surface", moving, aligned_surface),
        ("in_place_own_surface", in_place, in_place_surface),
        ("moving_own_surface", moving, moving_surface),
        ("aligned_samples_aligned_surface", [s for s in samples if s.command_aligned_with_yaw], aligned_surface),
    ]:
        rmse_rows.append((
            label,
            len(subset),
            rmse(current_error(sample, params) for sample in subset),
            rmse(corrected_error(sample, params, surface) for sample in subset),
            len(surface),
        ))
    for summary in summaries:
        run_samples = [sample for sample in samples if sample.run == summary.run]
        train = [sample for sample in samples if sample.run != summary.run]
        surface = build_surface(train, aligned_only=True)
        rmse_rows.append((
            f"leave_run_out:{summary.run}",
            len(run_samples),
            rmse(current_error(sample, params) for sample in run_samples),
            rmse(corrected_error(sample, params, surface) for sample in run_samples),
            len(surface),
        ))

    rmse_csv = OUT_DIR / "counter_yaw_surface_rmse.csv"
    with rmse_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["subset", "samples", "current_rmse_radps", "surface_corrected_rmse_radps", "surface_bins_used"])
        writer.writerows(rmse_rows)

    def markdown_table(rows: list[list[str]], limit: int = 60) -> list[str]:
        out = [
            "| Selection | Regime | Forward m/s | abs yaw rad/s | Samples | Median opposing Nm | Mean opposing Nm | P20 | P80 | Positive fraction |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
        for row in rows[:limit]:
            out.append(f"| `{row[0]}` | `{row[1]}` | `{row[2]}` | `{row[3]}` | {row[4]} | {row[5]} | {row[6]} | {row[7]} | {row[8]} | {row[9]} |")
        if len(rows) > limit:
            out.append(f"| ... | ... | ... | ... | {len(rows) - limit} more rows in CSV | | | | | |")
        return out

    surface_all_rows = (
        surface_rows(samples, "in_place", aligned_only=True) +
        surface_rows(samples, "moving", aligned_only=True) +
        surface_rows(samples, "in_place", aligned_only=False) +
        surface_rows(samples, "moving", aligned_only=False)
    )
    report = [
        "# Counter-Yaw Torque Surface Audit",
        "",
        "Worker C's scalar track/yaw-inertia fit is treated here as a diagnostic baseline only. This audit estimates a residual torque surface from historical sensor data using current authoritative constants.",
        "",
        "Sign convention: `+Yaw` is clockwise. Observed yaw acceleration is a 5-control-sample slope from raw gyro after opening stationary-bias subtraction. The current-constant commanded yaw moment is computed from drive commands and encoder wheel speeds using the motor model, launch torque, rolling torque, current track width, and current yaw denominator. Residual counter-yaw torque is `commanded_yaw_moment - yaw_denominator * observed_yaw_accel`. `opposing_yaw_nm = residual_counter_yaw_nm * sign(gyro_yaw_rate)`, so positive values indicate torque that resists the current yaw rotation.",
        "",
        "No UKF targets are used. The script intentionally avoids `ukf_state_*`, `ukf_state_bgz_radps`, `measured_linear_speed_mps`, `measured_angular_speed_radps`, `yaw_consistency_lp_radps`, `yaw_window_mismatch_rad`, and `nhc_*` fields.",
        "",
        "Worker C baseline reproduced separately: 104017 one-step samples, current scalar-model RMSE 0.226296125 rad/s, proposed scalar-model RMSE 0.161693351 rad/s, proposed track 0.104595474 m, proposed yaw inertia 0.000603133 kg m^2, yaw damping 0. This remains a diagnostic baseline, not the final recommendation.",
        "",
        "## Commands",
        "",
        "```powershell",
        "python codex_analysis\\yaw_fit\\fit_yaw_plant_params.py --report codex_analysis\\yaw_fit_validation\\worker_c_rerun_report.md",
        "python codex_analysis\\yaw_fit_validation\\counter_yaw_torque_surface.py",
        "```",
        "",
        "## Run Coverage",
        "",
        "| Run | Worker C | Rows | Kept | Samples | In-place | Moving | Cut note |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for summary in summaries:
        report.append(f"| `{summary.run}` | {'yes' if summary.run in WORKER_C_RUNS else ''} | {summary.rows} | {summary.kept_rows} | {summary.samples} | {summary.in_place_samples} | {summary.moving_samples} | {summary.notes} |")
    report.extend([
        "",
        "## RMSE",
        "",
        "| Subset | Samples | Current RMSE rad/s | Surface-corrected RMSE rad/s | Surface bins |",
        "| --- | ---: | ---: | ---: | ---: |",
    ])
    for row in rmse_rows:
        report.append(f"| `{row[0]}` | {row[1]} | {row[2]:.9f} | {row[3]:.9f} | {row[4]} |")
    report.extend([
        "",
        "## Discrete Torque Surface",
        "",
        "Rows with fewer than 80 samples are retained in the CSV for coverage inspection, but bins below that count are not used by the RMSE correction surface. The aligned rows are the cleaner resistance estimate because commanded yaw moment has the same sign as current yaw rate; all rows include braking and phase-transition dynamics.",
        "",
    ])
    report.extend(markdown_table(surface_all_rows))
    report.extend([
        "",
        "## Interpretation",
        "",
        "Aligned bins show a consistent positive opposing torque, so the historical data can estimate a yaw-resisting surface where the drive command is trying to add yaw in the same direction as current yaw rate. The same surface improves aligned-sample RMSE but worsens many all-sample and leave-run-out cases because braking and phase-transition rows contain different control dynamics. In-place data provides the strongest yaw-rate coverage near zero forward velocity. Moving-yaw data mostly covers 0.1..0.5 m/s and low-to-mid yaw rates; high forward-speed/high-yaw bins are sparse, so current logs are insufficient for a full production-quality two-dimensional tire CoF surface.",
        "",
        f"Created files: `{surface_csv.relative_to(REPO_ROOT)}`, `{rmse_csv.relative_to(REPO_ROOT)}`, `{summary_csv.relative_to(REPO_ROOT)}`, `{(OUT_DIR / 'counter_yaw_torque_surface_report.md').relative_to(REPO_ROOT)}`.",
    ])
    (OUT_DIR / "counter_yaw_torque_surface_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")


def main() -> int:
    params = load_params()
    summaries: list[RunSummary] = []
    samples: list[Sample] = []
    for run in RUNS:
        summary, run_samples = load_run(run, params)
        summaries.append(summary)
        samples.extend(run_samples)
        print(
            f"{run}: samples={summary.samples} in_place={summary.in_place_samples} "
            f"moving={summary.moving_samples} note={summary.notes}"
        )
    write_outputs(params, summaries, samples)
    print(f"report={OUT_DIR / 'counter_yaw_torque_surface_report.md'}")
    print(f"samples={len(samples)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
