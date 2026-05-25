#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent

WORKER_C_LOGS = [
    "mmlog_decode_2026-04-20_08-38-39",
    "mmlog_decode_2026-04-20_10-22-09",
    "mmlog_decode_2026-04-20_12-10-58",
    "mmlog_decode_2026-04-21_01-09-34",
]

EXPLORER_IN_PLACE = [
    "mmlog_decode_2026-04-21_05-32-06",
    "mmlog_decode_2026-04-20_08-38-39",
    "mmlog_decode_2026-05-04_20-35-47",
    "mmlog_decode_2026-05-04_16-57-53",
    "mmlog_decode_2026-04-21_05-59-46",
]

EXPLORER_MOVING = [
    "mmlog_decode_2026-04-21_00-16-10",
    "mmlog_decode_2026-04-21_01-09-34",
    "mmlog_decode_2026-04-20_02-33-07",
    "mmlog_decode_2026-04-20_12-10-58",
    "mmlog_decode_2026-04-14_04-43-48",
]

VALIDATION_LOGS = list(dict.fromkeys(EXPLORER_IN_PLACE + EXPLORER_MOVING + WORKER_C_LOGS))


@dataclass(frozen=True)
class Params:
    mass_kg: float
    track_m: float
    yaw_inertia: float
    wheel_radius_m: float
    wheel_bank_inertia: float
    drive_voltage_v: float
    resistance_ohm: float
    kt_nm_a: float
    kv_radps_v: float
    no_load_a: float
    gear: float
    rolling_torque_nm: float
    launch_command: float
    static_friction_max_speed_mps: float
    viscous_friction_nm_per_radps: float
    longitudinal_tire_stiffness_n: float
    front_right_gain_n_per_mps: float
    rear_right_gain_n_per_mps: float
    yaw_damping: float
    fan_downforce_n: float
    sustained_lateral_accel_mps2: float
    drive_longitudinal_offset_m: float


@dataclass(frozen=True)
class Row:
    run: str
    index: int
    t_us: int
    tick: int
    dt_s: float
    section: int | None
    phase: int | None
    primitive: str
    speed_bin: str
    repeat: str
    left_cmd: float
    right_cmd: float
    cmd_yaw: float
    left_vel: float
    right_vel: float
    left_wheel_radps: float
    right_wheel_radps: float
    gyro_raw: float
    gyro_sensor: float
    accel_x: float | None
    accel_y: float | None
    fan_duty: float
    saturation_flags: int
    watchdog_flags: int
    imu_age_s: float | None
    encoder_age_s: float | None


@dataclass(frozen=True)
class Sample:
    run: str
    category: str
    active: bool
    yaw: float
    next_yaw: float
    dt_s: float
    forward_mps: float
    left_vel: float
    right_vel: float
    left_wheel_radps: float
    right_wheel_radps: float
    left_cmd: float
    right_cmd: float
    fan_duty: float
    tick: int


@dataclass(frozen=True)
class RunSummary:
    run: str
    rows: int
    kept_rows: int
    yaw_rows: int
    active_yaw_rows: int
    moving_rows: int
    active_moving_rows: int
    samples: int
    bad_tail_cut_time_us: int | None
    terminal_fault_time_us: int | None
    notes: str


@dataclass(frozen=True)
class Candidate:
    name: str
    track_m: float
    yaw_inertia: float
    yaw_damping: float = 0.0
    coulomb_yaw_nm: float = 0.0


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def const_float(text: str, name: str) -> float:
    match = re.search(rf"\b{name}\s*=\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)[fF]?", text)
    if not match:
        raise ValueError(f"missing constant {name}")
    return float(match.group(1))


def converted_const(text: str, name: str, fn: str, scale: float) -> float:
    match = re.search(rf"\b{name}\s*=\s*{fn}\(\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)[fF]?\s*\)", text)
    if not match:
        raise ValueError(f"missing converted constant {name}")
    return scale * float(match.group(1))


def load_params() -> Params:
    vehicle = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "Vehicle.h")
    plant = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "PlantModel.h")
    motor = read_text(REPO_ROOT / "MazeMap" / "MazeMap" / "MotorEncoderDrive.h")
    wheel_diam = const_float(vehicle, "kDriveWheelDiameterM")
    no_load_a = converted_const(vehicle, "kDriveNoLoadCurrentA", "MilliAmpsToAmps", 1.0e-3)
    resistance = const_float(vehicle, "kDriveResistanceOhms")
    nominal_rpm = const_float(vehicle, "kDriveNominalNoLoadSpeedRpm")
    nominal_v = const_float(vehicle, "kDriveNominalVoltageV")
    kv = (nominal_rpm * (2.0 * math.pi / 60.0)) / (nominal_v - (no_load_a * resistance))
    return Params(
        mass_kg=const_float(vehicle, "kPhysicalMassKg"),
        track_m=const_float(vehicle, "kPhysicalTrackWidthM"),
        yaw_inertia=const_float(vehicle, "kPhysicalYawInertiaKgM2"),
        wheel_radius_m=0.5 * wheel_diam,
        wheel_bank_inertia=const_float(motor, "kDefaultWheelBankEquivalentInertiaKgM2"),
        drive_voltage_v=const_float(vehicle, "kDriveSupplyVoltageV"),
        resistance_ohm=resistance,
        kt_nm_a=converted_const(vehicle, "kDriveTorqueConstantNmPerA", "MilliNewtonMetersToNewtonMeters", 1.0e-3),
        kv_radps_v=kv,
        no_load_a=no_load_a,
        gear=const_float(vehicle, "kDriveGearRatio"),
        rolling_torque_nm=const_float(plant, "kRollingFrictionTorqueNm"),
        launch_command=const_float(plant, "kReliableLaunchDriveCommand"),
        static_friction_max_speed_mps=const_float(plant, "kStaticFrictionMaxSpeedMps"),
        viscous_friction_nm_per_radps=const_float(plant, "kViscousFrictionNmPerRadps"),
        longitudinal_tire_stiffness_n=const_float(motor, "kDefaultLongitudinalTireStiffnessN"),
        front_right_gain_n_per_mps=const_float(motor, "kDefaultFrontRightContactForceGainNPerMps"),
        rear_right_gain_n_per_mps=const_float(motor, "kDefaultRearRightContactForceGainNPerMps"),
        yaw_damping=const_float(plant, "kYawRateDampingNmsPerRad"),
        fan_downforce_n=const_float(vehicle, "kFanDownforceAtFullDutyN"),
        sustained_lateral_accel_mps2=1.91 * 9.80665,
        drive_longitudinal_offset_m=const_float(vehicle, "kDriveWheelLongitudinalOffsetM"),
    )


def f(row: dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    if value == "":
        return default
    try:
        out = float(value)
    except ValueError:
        return default
    return out if math.isfinite(out) else default


def i(row: dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(float(row.get(key, "")))
    except ValueError:
        return default


def opt_i(row: dict[str, str], key: str) -> int | None:
    if key not in row or row[key] == "":
        return None
    return i(row, key)


def stationary_bias(raw_rows: list[dict[str, str]]) -> float:
    opening: list[float] = []
    for row in raw_rows[: min(len(raw_rows), 5000)]:
        quiet_command = abs(f(row, "left_drive_command")) < 0.03 and abs(f(row, "right_drive_command")) < 0.03
        quiet_encoder = abs(f(row, "left_encoder_velocity_mps")) < 0.03 and abs(f(row, "right_encoder_velocity_mps")) < 0.03
        old_static = row.get("section_id") == "1" and row.get("primitive_id") in {"0", "2", ""}
        if (old_static or (quiet_command and quiet_encoder)) and row.get("gyro_raw_radps", "") != "":
            opening.append(f(row, "gyro_raw_radps"))
    if len(opening) >= 8:
        return statistics.fmean(opening)
    fallback = [f(row, "gyro_raw_radps") for row in raw_rows[: min(len(raw_rows), 1000)] if row.get("gyro_raw_radps", "") != ""]
    return statistics.fmean(fallback) if fallback else 0.0


def terminal_fault_time(run_dir: Path) -> int | None:
    log = run_dir / "logging.txt"
    if not log.is_file():
        return None
    last = None
    for match in re.finditer(r"\[(\d+)\].*fault", read_text(log).split("\0", 1)[0], re.IGNORECASE):
        last = int(match.group(1))
    return last


def row_category(row: Row) -> str | None:
    if row.section == 4 or (row.section is None and row.phase in {4, 20}):
        return "in_place"
    if row.section in {5, 6, 7} or (row.section is None and row.phase in {5, 6, 7}):
        return "moving"
    return None


def is_active(row: Row) -> bool:
    return abs(row.left_cmd - row.right_cmd) >= 0.05 or abs(row.cmd_yaw) >= 0.20


def same_segment(a: Row, b: Row) -> bool:
    return (
        a.section == b.section and
        a.phase == b.phase and
        a.primitive == b.primitive and
        a.speed_bin == b.speed_bin and
        a.repeat == b.repeat
    )


def find_bad_tail(rows: list[Row]) -> tuple[int | None, str]:
    if len(rows) < 100:
        return None, ""
    first_bad = None
    run = 0
    reason = ""
    for row in rows:
        sensor_stale = (row.imu_age_s is not None and row.imu_age_s > 0.050) or (row.encoder_age_s is not None and row.encoder_age_s > 0.050)
        uncommanded_motion = (
            abs(row.left_cmd) < 0.03 and abs(row.right_cmd) < 0.03 and
            (abs(row.left_vel) > 0.12 or abs(row.right_vel) > 0.12 or abs(row.gyro_sensor) > 0.80)
        )
        commanded_stall = (
            abs(row.left_cmd - row.right_cmd) > 0.25 and
            abs(row.left_vel - row.right_vel) < 0.025 and
            abs(row.gyro_sensor) < 0.20
        )
        bad = sensor_stale or uncommanded_motion or commanded_stall
        if bad:
            if run == 0:
                first_bad = row.t_us
                if sensor_stale:
                    reason = "sustained sensor freshness failure"
                elif uncommanded_motion:
                    reason = "sustained uncommanded motion"
                else:
                    reason = "sustained commanded stall/lift-like window"
            run += 1
            if run >= 100:
                return max(0, (first_bad or row.t_us) - 200_000), reason
        else:
            run = 0
            first_bad = None
            reason = ""
    return None, ""


def load_run(run: str, params: Params) -> tuple[RunSummary, list[Sample]]:
    run_dir = REPO_ROOT / "TestResults" / run
    csv_path = run_dir / "open_floor_main.csv"
    with csv_path.open(newline="", encoding="utf-8") as stream:
        raw_rows = list(csv.DictReader(stream))
    bias = stationary_bias(raw_rows)
    parsed: list[Row] = []
    for idx, raw in enumerate(raw_rows):
        t_us = i(raw, "master_time_us")
        imu_ts = opt_i(raw, "imu_timestamp_us")
        enc_ts = opt_i(raw, "encoder_timestamp_us")
        gyro_sensor = (f(raw, "gyro_raw_radps") - bias) if raw.get("gyro_raw_radps", "") != "" else f(raw, "gyro_radps")
        parsed.append(Row(
            run=run,
            index=idx,
            t_us=t_us,
            tick=i(raw, "control_tick_sequence", idx),
            dt_s=1.0e-6 * i(raw, "dt_us", 1000),
            section=opt_i(raw, "section_id"),
            phase=opt_i(raw, "phase_id"),
            primitive=raw.get("primitive_id", ""),
            speed_bin=raw.get("speed_bin", ""),
            repeat=raw.get("repeat_index", ""),
            left_cmd=f(raw, "left_drive_command"),
            right_cmd=f(raw, "right_drive_command"),
            cmd_yaw=f(raw, "cmd_angular_radps"),
            left_vel=f(raw, "left_encoder_velocity_mps"),
            right_vel=f(raw, "right_encoder_velocity_mps"),
            left_wheel_radps=f(raw, "left_encoder_wheel_speed_radps", f(raw, "left_encoder_omega_radps")),
            right_wheel_radps=f(raw, "right_encoder_wheel_speed_radps", f(raw, "right_encoder_omega_radps")),
            gyro_raw=f(raw, "gyro_raw_radps"),
            gyro_sensor=gyro_sensor,
            accel_x=f(raw, "accel_body_x_mps2") if "accel_body_x_mps2" in raw else None,
            accel_y=f(raw, "accel_body_y_mps2") if "accel_body_y_mps2" in raw else None,
            fan_duty=f(raw, "fan_duty_cycle", 0.0),
            saturation_flags=i(raw, "saturation_flags"),
            watchdog_flags=i(raw, "watchdog_flags"),
            imu_age_s=(1.0e-6 * max(0, t_us - imu_ts)) if imu_ts is not None else None,
            encoder_age_s=(1.0e-6 * max(0, t_us - enc_ts)) if enc_ts is not None else None,
        ))
    bad_tail, bad_reason = find_bad_tail(parsed)
    fault_time = terminal_fault_time(run_dir)
    cutoff_time = None
    notes: list[str] = []
    if bad_tail is not None:
        cutoff_time = bad_tail
        notes.append(bad_reason)
    if fault_time is not None:
        fault_cut = max(0, fault_time - 200_000)
        cutoff_time = min(cutoff_time, fault_cut) if cutoff_time is not None else fault_cut
        notes.append("terminal fault upper bound")
    kept = [row for row in parsed if cutoff_time is None or row.t_us <= cutoff_time]
    samples: list[Sample] = []
    yaw_rows = active_yaw = moving_rows = active_moving = 0
    for row in kept:
        cat = row_category(row)
        if cat == "in_place":
            yaw_rows += 1
            active_yaw += 1 if is_active(row) else 0
        elif cat == "moving":
            moving_rows += 1
            active_moving += 1 if is_active(row) else 0
    for cur, nxt in zip(kept[:-1], kept[1:]):
        cat = row_category(cur)
        if cat is None or row_category(nxt) != cat:
            continue
        if not same_segment(cur, nxt):
            continue
        if cur.saturation_flags or nxt.saturation_flags or cur.watchdog_flags or nxt.watchdog_flags:
            continue
        dt_s = 1.0e-6 * max(1, nxt.t_us - cur.t_us)
        if not (0.0005 <= dt_s <= 0.0030):
            continue
        active = is_active(cur)
        if not active and abs(cur.gyro_sensor) < 0.20:
            continue
        samples.append(Sample(
            run=run,
            category=cat,
            active=active,
            yaw=cur.gyro_sensor,
            next_yaw=nxt.gyro_sensor,
            dt_s=dt_s,
            forward_mps=0.5 * (cur.left_vel + cur.right_vel),
            left_vel=cur.left_vel,
            right_vel=cur.right_vel,
            left_wheel_radps=cur.left_wheel_radps,
            right_wheel_radps=cur.right_wheel_radps,
            left_cmd=cur.left_cmd,
            right_cmd=cur.right_cmd,
            fan_duty=cur.fan_duty,
            tick=cur.tick,
        ))
    return RunSummary(
        run=run,
        rows=len(parsed),
        kept_rows=len(kept),
        yaw_rows=yaw_rows,
        active_yaw_rows=active_yaw,
        moving_rows=moving_rows,
        active_moving_rows=active_moving,
        samples=len(samples),
        bad_tail_cut_time_us=cutoff_time,
        terminal_fault_time_us=fault_time,
        notes=", ".join(dict.fromkeys(notes)),
    ), samples


def signed_direction(preferred: float, fallback: float = 0.0) -> float:
    eps = 1.0e-6
    if preferred > eps:
        return 1.0
    if preferred < -eps:
        return -1.0
    if fallback > eps:
        return 1.0
    if fallback < -eps:
        return -1.0
    return 0.0


def torque_from_command(command: float, wheel_speed_radps: float, p: Params) -> float:
    applied_v = max(-1.0, min(1.0, command)) * p.drive_voltage_v
    back_emf_v = (wheel_speed_radps * p.gear) / p.kv_radps_v
    armature_a = (applied_v - back_emf_v) / p.resistance_ohm
    no_load_dir = signed_direction(armature_a, wheel_speed_radps)
    load_a = armature_a - no_load_dir * p.no_load_a
    if no_load_dir > 0.0 and load_a < 0.0:
        load_a = 0.0
    elif no_load_dir < 0.0 and load_a > 0.0:
        load_a = 0.0
    return p.kt_nm_a * p.gear * load_a


def applied_bank_torque(command: float, wheel_speed_radps: float, p: Params) -> float:
    direct = torque_from_command(command, wheel_speed_radps, p)
    positive = max(0.0, torque_from_command(1.0, wheel_speed_radps, p))
    negative = min(0.0, torque_from_command(-1.0, wheel_speed_radps, p))
    limited = max(negative, min(positive, direct)) if positive > negative else direct
    launch_static = max(0.0, torque_from_command(p.launch_command, 0.0, p))
    surface_mps = p.wheel_radius_m * wheel_speed_radps
    slow_ratio = abs(surface_mps) / p.static_friction_max_speed_mps if p.static_friction_max_speed_mps > 0 else 0.0
    launch = launch_static * math.exp(-(slow_ratio * slow_ratio))
    out = 0.0
    direction = signed_direction(limited, wheel_speed_radps)
    if abs(limited) > launch:
        out = limited - direction * launch
    loss_direction = signed_direction(wheel_speed_radps, out)
    return out - ((p.rolling_torque_nm * loss_direction) + (p.viscous_friction_nm_per_radps * wheel_speed_radps))


def wheel_spinup_mass(p: Params) -> float:
    return (2.0 * p.wheel_bank_inertia) / (p.wheel_radius_m * p.wheel_radius_m)


def denom(track_m: float, inertia: float, p: Params) -> float:
    return inertia + wheel_spinup_mass(p) * (0.5 * abs(track_m)) ** 2


def faithful_yaw_moment(s: Sample, track_m: float, p: Params) -> float:
    left_torque = applied_bank_torque(s.left_cmd, s.left_wheel_radps, p)
    right_torque = applied_bank_torque(s.right_cmd, s.right_wheel_radps, p)
    wheel_radius = p.wheel_radius_m
    left_drive_force = left_torque / wheel_radius if wheel_radius > 1.0e-4 else 0.0
    right_drive_force = right_torque / wheel_radius if wheel_radius > 1.0e-4 else 0.0
    total_normal = (p.mass_kg * 9.80665) + (max(0.0, min(1.0, s.fan_duty)) * p.fan_downforce_n)
    front_normal = 0.5 * 0.5 * total_normal
    rear_normal = 0.5 * 0.5 * total_normal
    left_bank_normal = front_normal + rear_normal
    right_bank_normal = front_normal + rear_normal
    fl_drive = left_drive_force * (front_normal / left_bank_normal) if left_bank_normal > 1.0e-4 else 0.5 * left_drive_force
    rl_drive = left_drive_force * (rear_normal / left_bank_normal) if left_bank_normal > 1.0e-4 else 0.5 * left_drive_force
    fr_drive = right_drive_force * (front_normal / right_bank_normal) if right_bank_normal > 1.0e-4 else 0.5 * right_drive_force
    rr_drive = right_drive_force * (rear_normal / right_bank_normal) if right_bank_normal > 1.0e-4 else 0.5 * right_drive_force

    half_track = 0.5 * abs(track_m)
    longitudinal_offset = abs(p.drive_longitudinal_offset_m)
    left_body = s.forward_mps + half_track * s.yaw
    right_body = s.forward_mps - half_track * s.yaw
    left_rel = s.left_vel - left_body
    right_rel = s.right_vel - right_body
    front_right_body = longitudinal_offset * s.yaw
    rear_right_body = -longitudinal_offset * s.yaw
    fl_raw_f = fl_drive + 0.5 * p.longitudinal_tire_stiffness_n * left_rel
    rl_raw_f = rl_drive + 0.5 * p.longitudinal_tire_stiffness_n * left_rel
    fr_raw_f = fr_drive + 0.5 * p.longitudinal_tire_stiffness_n * right_rel
    rr_raw_f = rr_drive + 0.5 * p.longitudinal_tire_stiffness_n * right_rel
    fl_raw_r = p.front_right_gain_n_per_mps * (-front_right_body)
    fr_raw_r = p.front_right_gain_n_per_mps * (-front_right_body)
    rl_raw_r = p.rear_right_gain_n_per_mps * (-rear_right_body)
    rr_raw_r = p.rear_right_gain_n_per_mps * (-rear_right_body)
    sustained_mu = (p.sustained_lateral_accel_mps2 * p.mass_kg / total_normal) if total_normal > 1.0e-4 else 0.0

    def project(forward: float, right: float, normal: float) -> tuple[float, float]:
        max_force = max(0.0, sustained_mu * normal)
        mag = math.hypot(forward, right)
        scale = max_force / mag if mag > max_force and mag > 1.0e-4 else 1.0
        return scale * forward, scale * right

    fl_f, fl_r = project(fl_raw_f, fl_raw_r, front_normal)
    fr_f, fr_r = project(fr_raw_f, fr_raw_r, front_normal)
    rl_f, rl_r = project(rl_raw_f, rl_raw_r, rear_normal)
    rr_f, rr_r = project(rr_raw_f, rr_raw_r, rear_normal)
    left_bank_f = fl_f + rl_f
    right_bank_f = fr_f + rr_f
    front_right_f = fl_r + fr_r
    rear_right_f = rl_r + rr_r
    return (half_track * (left_bank_f - right_bank_f)) + (longitudinal_offset * (front_right_f - rear_right_f))


def simple_yaw_moment(s: Sample, track_m: float, p: Params) -> float:
    left_torque = applied_bank_torque(s.left_cmd, s.left_wheel_radps, p)
    right_torque = applied_bank_torque(s.right_cmd, s.right_wheel_radps, p)
    return 0.5 * abs(track_m) * ((left_torque / p.wheel_radius_m) - (right_torque / p.wheel_radius_m))


def predict(s: Sample, c: Candidate, p: Params, moment_fn: Callable[[Sample, float, Params], float]) -> float:
    d = denom(c.track_m, c.yaw_inertia, p)
    moment = moment_fn(s, c.track_m, p)
    if c.coulomb_yaw_nm > 0.0:
        moment -= c.coulomb_yaw_nm * signed_direction(s.yaw, moment)
    alpha = (moment - (c.yaw_damping * s.yaw)) / d
    return s.yaw + alpha * s.dt_s


def rmse(errors: Iterable[float]) -> float:
    values = list(errors)
    return math.sqrt(statistics.fmean(e * e for e in values)) if values else 0.0


def eval_candidate(samples: list[Sample], c: Candidate, p: Params, moment_fn: Callable[[Sample, float, Params], float]) -> float:
    return rmse(predict(s, c, p, moment_fn) - s.next_yaw for s in samples)


def split_samples(samples: list[Sample]) -> tuple[list[Sample], list[Sample]]:
    train = [s for s in samples if s.run not in {"mmlog_decode_2026-04-21_05-32-06", "mmlog_decode_2026-04-21_00-16-10", "mmlog_decode_2026-05-04_20-35-47"}]
    holdout = [s for s in samples if s not in train]
    return train, holdout


def golden(lo: float, hi: float, objective: Callable[[float], float], rounds: int = 44) -> tuple[float, float]:
    gr = (math.sqrt(5.0) - 1.0) / 2.0
    x1 = hi - gr * (hi - lo)
    x2 = lo + gr * (hi - lo)
    f1 = objective(x1)
    f2 = objective(x2)
    for _ in range(rounds):
        if f1 > f2:
            lo = x1
            x1 = x2
            f1 = f2
            x2 = lo + gr * (hi - lo)
            f2 = objective(x2)
        else:
            hi = x2
            x2 = x1
            f2 = f1
            x1 = hi - gr * (hi - lo)
            f1 = objective(x1)
    x = 0.5 * (lo + hi)
    return x, objective(x)


def fit_damping(samples: list[Sample], base: Candidate, p: Params, moment_fn: Callable[[Sample, float, Params], float]) -> float:
    d = denom(base.track_m, base.yaw_inertia, p)
    num = 0.0
    den = 0.0
    for s in samples:
        moment = moment_fn(s, base.track_m, p)
        base_err = s.yaw + (s.dt_s * moment / d) - s.next_yaw
        coef = -s.dt_s * s.yaw / d
        num += coef * base_err
        den += coef * coef
    return max(0.0, -num / den) if den > 0.0 else 0.0


def fit_coulomb(samples: list[Sample], base: Candidate, p: Params, moment_fn: Callable[[Sample, float, Params], float]) -> float:
    d = denom(base.track_m, base.yaw_inertia, p)
    num = 0.0
    den = 0.0
    for s in samples:
        moment = moment_fn(s, base.track_m, p)
        base_err = s.yaw + (s.dt_s * moment / d) - s.next_yaw
        coef = -s.dt_s * signed_direction(s.yaw, moment) / d
        num += coef * base_err
        den += coef * coef
    return max(0.0, -num / den) if den > 0.0 else 0.0


def coordinate_fit(samples: list[Sample], start: Candidate, p: Params, moment_fn: Callable[[Sample, float, Params], float], damping: bool) -> Candidate:
    current = start
    for _ in range(5):
        track, _ = golden(0.060, 0.145, lambda x: eval_candidate(samples, Candidate(current.name, x, current.yaw_inertia, current.yaw_damping), p, moment_fn), 34)
        current = Candidate(current.name, track, current.yaw_inertia, current.yaw_damping)
        inertia, _ = golden(0.00008, 0.00120, lambda x: eval_candidate(samples, Candidate(current.name, current.track_m, x, current.yaw_damping), p, moment_fn), 34)
        current = Candidate(current.name, current.track_m, inertia, current.yaw_damping)
        if damping:
            yd = fit_damping(samples, current, p, moment_fn)
            current = Candidate(current.name, current.track_m, current.yaw_inertia, yd)
    return current


def write_report(params: Params, summaries: list[RunSummary], samples: list[Sample]) -> None:
    in_place_active = [s for s in samples if s.category == "in_place" and s.active]
    moving_active = [s for s in samples if s.category == "moving" and s.active]
    fit_samples = in_place_active + moving_active
    train, holdout = split_samples(fit_samples)
    current = Candidate("current", params.track_m, params.yaw_inertia, params.yaw_damping)
    worker_c = Candidate("worker_c_proposed", 0.104595474, 0.000603133, 0.0)

    faithful_candidates = [
        current,
        worker_c,
    ]
    track, _ = golden(0.060, 0.145, lambda x: eval_candidate(train, Candidate("track_only", x, params.yaw_inertia, 0.0), params, faithful_yaw_moment), 40)
    faithful_candidates.append(Candidate("track_only", track, params.yaw_inertia, 0.0))
    inertia, _ = golden(0.00008, 0.00120, lambda x: eval_candidate(train, Candidate("inertia_only", params.track_m, x, 0.0), params, faithful_yaw_moment), 40)
    faithful_candidates.append(Candidate("inertia_only", params.track_m, inertia, 0.0))
    yd = fit_damping(train, current, params, faithful_yaw_moment)
    faithful_candidates.append(Candidate("yaw_damping_only", params.track_m, params.yaw_inertia, yd))
    combined_no_damping = coordinate_fit(train, current, params, faithful_yaw_moment, damping=False)
    faithful_candidates.append(Candidate("combined_track_inertia", combined_no_damping.track_m, combined_no_damping.yaw_inertia, 0.0))
    combined = coordinate_fit(train, current, params, faithful_yaw_moment, damping=True)
    faithful_candidates.append(Candidate("combined_with_yaw_damping", combined.track_m, combined.yaw_inertia, combined.yaw_damping))
    coul = fit_coulomb(train, current, params, faithful_yaw_moment)
    faithful_candidates.append(Candidate("diagnostic_coulomb_yaw_resistance", params.track_m, params.yaw_inertia, 0.0, coul))

    simple_candidates = [
        current,
        worker_c,
        coordinate_fit(train, current, params, simple_yaw_moment, damping=False),
    ]
    simple_candidates[-1] = Candidate("simple_combined_refit", simple_candidates[-1].track_m, simple_candidates[-1].yaw_inertia, 0.0)

    rows = []
    for c in faithful_candidates:
        rows.append((
            c.name,
            c.track_m,
            c.yaw_inertia,
            c.yaw_damping,
            c.coulomb_yaw_nm,
            eval_candidate(train, c, params, faithful_yaw_moment),
            eval_candidate(holdout, c, params, faithful_yaw_moment),
            eval_candidate(in_place_active, c, params, faithful_yaw_moment),
            eval_candidate(moving_active, c, params, faithful_yaw_moment),
        ))
    simple_rows = []
    for c in simple_candidates:
        simple_rows.append((
            c.name,
            c.track_m,
            c.yaw_inertia,
            c.yaw_damping,
            eval_candidate(train, c, params, simple_yaw_moment),
            eval_candidate(holdout, c, params, simple_yaw_moment),
            eval_candidate(in_place_active, c, params, simple_yaw_moment),
            eval_candidate(moving_active, c, params, simple_yaw_moment),
        ))

    by_run_lines = []
    for summary in summaries:
        run_samples = [s for s in fit_samples if s.run == summary.run]
        if not run_samples:
            continue
        by_run_lines.append((
            summary.run,
            len(run_samples),
            eval_candidate(run_samples, current, params, faithful_yaw_moment),
            eval_candidate(run_samples, worker_c, params, faithful_yaw_moment),
            eval_candidate(run_samples, combined, params, faithful_yaw_moment),
        ))

    phase_lines = []
    for name, subset in [
        ("in_place_active", in_place_active),
        ("in_place_decay", [s for s in samples if s.category == "in_place" and not s.active]),
        ("moving_active", moving_active),
        ("moving_decay", [s for s in samples if s.category == "moving" and not s.active]),
    ]:
        if subset:
            phase_lines.append((
                name,
                len(subset),
                eval_candidate(subset, current, params, faithful_yaw_moment),
                eval_candidate(subset, worker_c, params, faithful_yaw_moment),
                eval_candidate(subset, combined, params, faithful_yaw_moment),
            ))

    report = [
        "# Yaw Fit Validation Audit",
        "",
        "This validation uses decoded `TestResults/mmlog_decode_2026-*/open_floor_main.csv` files, not replay bundles. Targets are `gyro_raw_radps` minus an opening stationary bias when present, with `gyro_radps` only as a sensor-derived fallback if raw gyro is missing. The script does not read UKF state, `measured_*`, `yaw_consistency*`, or `nhc_*` columns for targets.",
        "",
        "## Worker C Reproduction",
        "",
        "Worker C's original script was rerun separately and reproduced: 104017 samples, current RMSE 0.226296125 rad/s, proposed RMSE 0.161693351 rad/s, proposed track 0.104595474 m, proposed yaw inertia 0.000603133 kg m^2, yaw damping 0.",
        "",
        "## Log Selection",
        "",
        "| Run | In Worker C | Explorer in-place rank | Explorer moving rank | Rows | Kept rows | Yaw rows | Active yaw rows | Moving rows | Active moving rows | Fit samples | Cut note |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for s in summaries:
        in_rank = EXPLORER_IN_PLACE.index(s.run) + 1 if s.run in EXPLORER_IN_PLACE else ""
        mov_rank = EXPLORER_MOVING.index(s.run) + 1 if s.run in EXPLORER_MOVING else ""
        report.append(f"| `{s.run}` | {'yes' if s.run in WORKER_C_LOGS else ''} | {in_rank} | {mov_rank} | {s.rows} | {s.kept_rows} | {s.yaw_rows} | {s.active_yaw_rows} | {s.moving_rows} | {s.active_moving_rows} | {s.samples} | {s.notes or ''} |")
    report.extend([
        "",
        "## Faithful-Equation Evaluation",
        "",
        "This model mirrors the current yaw-relevant `PlantModel` backend more closely than Worker C's script: command-to-bank torque, static/rolling/viscous wheel friction, encoder-input wheel surface velocity, longitudinal tire slip force, lateral contact slip force, contact force projection, differential yaw moment, wheel spin-up inertia, and optional yaw-rate damping. The remaining known mismatch is that this sensor-only audit sets unobserved body rightward velocity to zero and derives body forward speed from encoders to avoid UKF state inputs.",
        "",
        "| Candidate | Track m | Yaw inertia kg m^2 | Yaw damping Nms/rad | Diagnostic Coulomb yaw Nm | Train RMSE | Holdout RMSE | In-place active RMSE | Moving active RMSE |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for r in rows:
        report.append(f"| `{r[0]}` | {r[1]:.9f} | {r[2]:.9f} | {r[3]:.9f} | {r[4]:.9f} | {r[5]:.9f} | {r[6]:.9f} | {r[7]:.9f} | {r[8]:.9f} |")
    report.extend([
        "",
        "## Simplified Worker-C-Style Evaluation",
        "",
        "This omits PlantModel contact slip and projection terms. It is useful for reproducing Worker C's direction but is not faithful enough for a parameter change by itself.",
        "",
        "| Candidate | Track m | Yaw inertia kg m^2 | Yaw damping Nms/rad | Train RMSE | Holdout RMSE | In-place active RMSE | Moving active RMSE |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for r in simple_rows:
        report.append(f"| `{r[0]}` | {r[1]:.9f} | {r[2]:.9f} | {r[3]:.9f} | {r[4]:.9f} | {r[5]:.9f} | {r[6]:.9f} | {r[7]:.9f} |")
    report.extend([
        "",
        "## Per-Run Faithful RMSE",
        "",
        "| Run | Samples | Current | Worker C proposed | Best combined+damping |",
        "| --- | ---: | ---: | ---: | ---: |",
    ])
    for r in by_run_lines:
        report.append(f"| `{r[0]}` | {r[1]} | {r[2]:.9f} | {r[3]:.9f} | {r[4]:.9f} |")
    report.extend([
        "",
        "## Residual Sections",
        "",
        "| Subset | Samples | Current | Worker C proposed | Best combined+damping |",
        "| --- | ---: | ---: | ---: | ---: |",
    ])
    for r in phase_lines:
        report.append(f"| `{r[0]}` | {r[1]} | {r[2]:.9f} | {r[3]:.9f} | {r[4]:.9f} |")
    report.extend([
        "",
        "## Recommendation",
        "",
        "The historical sensor data supports the existence of yaw resistance/contact-slip behavior more strongly than it supports rewriting physical `Vehicle` facts to Worker C's proposed 104.6 mm track and 0.000603 kg m^2 yaw inertia. Worker C's fit improves a simplified model by inflating inertia and effective track width, but the faithful-equation audit shows that the current PlantModel contact terms already explain much of the in-place resistance. A defensible production change should be based on a PlantModel-owned resistance/contact calibration, not a Vehicle physical yaw inertia change, unless a dedicated physical inertia measurement corroborates it.",
    ])
    (OUT_DIR / "validation_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")

    with (OUT_DIR / "validation_rmse_table.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["candidate", "track_m", "yaw_inertia_kg_m2", "yaw_damping_nms_per_rad", "diagnostic_coulomb_yaw_nm", "train_rmse", "holdout_rmse", "in_place_active_rmse", "moving_active_rmse"])
        writer.writerows(rows)
    with (OUT_DIR / "validation_log_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(RunSummary.__dataclass_fields__.keys())
        for s in summaries:
            writer.writerow([getattr(s, name) for name in RunSummary.__dataclass_fields__.keys()])


def main() -> int:
    params = load_params()
    summaries: list[RunSummary] = []
    samples: list[Sample] = []
    for run in VALIDATION_LOGS:
        summary, run_samples = load_run(run, params)
        summaries.append(summary)
        samples.extend(run_samples)
        print(f"{run}: rows={summary.rows} samples={summary.samples} yaw={summary.active_yaw_rows} moving={summary.active_moving_rows} notes={summary.notes}")
    write_report(params, summaries, samples)
    print(f"report={OUT_DIR / 'validation_report.md'}")
    print(f"rmse_csv={OUT_DIR / 'validation_rmse_table.csv'}")
    print(f"log_summary_csv={OUT_DIR / 'validation_log_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
