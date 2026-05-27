#!/usr/bin/env python3
"""Fit a cheap scalar Stribeck-to-slip-angle yaw support law.

Analysis-only tooling. Reads the shared yaw/traction feature artifacts and
writes outputs only beside this script.
"""

from __future__ import annotations

import csv
import math
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent

PRIMARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "ablation" / "phase_classified_feature_sample.csv"
SECONDARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "contact_continuum_feature_sample.csv"
CONSTANTS = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "plant_mirror_constants.csv"
FORCE_IN_PLACE = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck" / "in_place_1radps_command.csv"
STANDALONE_IN_PLACE = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "standalone_contact_traction_testbed" / "in_place_1radps_command.csv"
REFERENCE_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "standalone_contact_traction_testbed" / "split_metrics.csv"
REFERENCE_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "standalone_contact_traction_testbed" / "selected_log_metrics.csv"

SELECTED_RUNS = [
    "2026-05-04_20-35-47",
    "2026-05-04_16-57-53",
    "2026-04-22_12-10-34",
    "2026-04-22_01-06-32",
    "2026-04-21_05-32-06",
    "2026-04-21_00-16-10",
    "2026-04-20_12-10-58",
    "2026-04-20_08-38-39",
    "diag003",
]

PRIMARY_COLUMNS = [
    "run_id",
    "family",
    "schema",
    "recommendation",
    "dataset_split",
    "row_index",
    "time_us",
    "physics_phase",
    "forward_velocity_mps",
    "yaw_rate_radps",
    "vbar_rel_mps",
    "vbar_yaw_mps",
    "max_force_preprojection_utilization",
    "max_force_limiter_activity",
    "hardware_saturation_evidence",
    "gyro_derivative_spike",
    "residual_additive_yaw_torque_nm",
    "residual_opposes_yaw_nm",
    "patch_yaw_req_basis_nm",
]

SECONDARY_COLUMNS = [
    "run_id",
    "row_index",
    "total_normal_load_n",
]

SPLITS = [
    "primary_open_floor_fit_authoritative",
    "open_floor_fit_downweighted",
    "open_floor_validation_only",
    "diag_validation_only",
    "aux_downweighted_validation",
]

HARD_GATE_COMMAND_MIN = 0.60
LAUNCH_COMMAND_TARGET = 0.646
STRIBECK_QUARTIC = 0.20


@dataclass(frozen=True)
class Bounds:
    name: str
    lo: float
    hi: float


BOUNDS = [
    Bounds("yaw_activation_mps", 0.001, 0.080),
    Bounds("stribeck_speed_mps", 0.010, 0.180),
    Bounds("slide_ratio", 0.0, 0.90),
    Bounds("speed_gate_mps", 0.002, 1.200),
    Bounds("v_floor_mps", 0.010, 0.600),
    Bounds("alpha_knee_rad", 0.002, 1.500),
    Bounds("slip_cap_nm", 0.0, 0.140),
    Bounds("alpha_lever_m", 0.005, 0.080),
]


def sign(value: float, eps: float = 1.0e-6) -> float:
    if value > eps:
        return 1.0
    if value < -eps:
        return -1.0
    return 0.0


def sign_array(values: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    return (values > eps).astype(float) - (values < -eps).astype(float)


def read_constants() -> dict[str, float]:
    with CONSTANTS.open(newline="", encoding="utf-8") as fh:
        return {row["name"]: float(row["value"]) for row in csv.DictReader(fh)}


def load_frame(constants: dict[str, float]) -> tuple[pd.DataFrame, float]:
    frame = pd.read_csv(PRIMARY, usecols=PRIMARY_COLUMNS)
    secondary = pd.read_csv(SECONDARY, usecols=SECONDARY_COLUMNS)
    frame = frame.merge(secondary, how="left", on=["run_id", "row_index"])

    numeric = [
        "row_index",
        "time_us",
        "forward_velocity_mps",
        "yaw_rate_radps",
        "vbar_rel_mps",
        "vbar_yaw_mps",
        "max_force_preprojection_utilization",
        "max_force_limiter_activity",
        "hardware_saturation_evidence",
        "gyro_derivative_spike",
        "residual_additive_yaw_torque_nm",
        "residual_opposes_yaw_nm",
        "patch_yaw_req_basis_nm",
        "total_normal_load_n",
    ]
    for column in numeric:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(
        subset=[
            "forward_velocity_mps",
            "yaw_rate_radps",
            "residual_additive_yaw_torque_nm",
            "residual_opposes_yaw_nm",
            "patch_yaw_req_basis_nm",
        ]
    )

    primary = frame["dataset_split"] == "primary_open_floor_fit_authoritative"
    nominal_load = float(frame.loc[primary, "total_normal_load_n"].median())
    if not math.isfinite(nominal_load) or nominal_load <= 0.0:
        nominal_load = float(frame["total_normal_load_n"].median())
    if not math.isfinite(nominal_load) or nominal_load <= 0.0:
        nominal_load = constants["mass_kg"] * 9.80665 + constants.get("fan_downforce_full_duty_n", 0.0)
    frame["total_normal_load_n"] = frame["total_normal_load_n"].fillna(nominal_load)
    frame["load_factor"] = frame["total_normal_load_n"] / nominal_load

    yaw_sign = sign_array(frame["yaw_rate_radps"].to_numpy())
    zero = yaw_sign == 0.0
    if np.any(zero):
        yaw_sign[zero] = sign_array(frame.loc[zero, "patch_yaw_req_basis_nm"].to_numpy())
    frame["yaw_sign"] = yaw_sign
    return frame, nominal_load


def torque_from_command(command: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]

    applied_voltage = command * battery
    current = (applied_voltage / resistance) - ((wheel_speed_radps * (gear_ratio / speed_constant)) / resistance)
    armature_sign = sign(current)
    wheel_sign = sign(wheel_speed_radps)
    no_load_sign = armature_sign if armature_sign else wheel_sign
    load_current = current - no_load_sign * no_load
    if no_load_sign > 0.0 and load_current < 0.0:
        load_current = 0.0
    elif no_load_sign < 0.0 and load_current > 0.0:
        load_current = 0.0
    return torque_constant * gear_ratio * load_current


def command_from_torque(command_torque: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]

    motor_torque = command_torque / gear_ratio
    torque_sign = sign(motor_torque)
    wheel_sign = sign(wheel_speed_radps)
    no_load_sign = torque_sign if torque_sign else wheel_sign
    current = (motor_torque / torque_constant) + no_load_sign * no_load
    back_emf = wheel_speed_radps * (gear_ratio / speed_constant)
    return ((current * resistance) + back_emf) / battery


def static_launch_torque(constants: dict[str, float]) -> float:
    return max(0.0, torque_from_command(constants["static_launch_command"], 0.0, constants))


def wheel_speeds(vf_mps: float, yaw_rate: float, constants: dict[str, float]) -> tuple[float, float, float, float]:
    half_track = 0.5 * constants["track_width_m"]
    radius = constants["wheel_radius_m"]
    left_surface_mps = vf_mps + half_track * yaw_rate
    right_surface_mps = vf_mps - half_track * yaw_rate
    return (
        left_surface_mps,
        right_surface_mps,
        left_surface_mps / radius,
        right_surface_mps / radius,
    )


def signed_direction(preferred: float, fallback: float) -> float:
    preferred_sign = sign(preferred)
    return preferred_sign if preferred_sign != 0.0 else sign(fallback)


def command_torque_for_applied(applied_torque: float, wheel_speed_radps: float, constants: dict[str, float]) -> tuple[float, float]:
    surface_speed = constants["wheel_radius_m"] * wheel_speed_radps
    ratio = abs(surface_speed) / constants["static_friction_max_speed_mps"]
    launch = static_launch_torque(constants) * math.exp(-(ratio * ratio))
    launch_dir = signed_direction(applied_torque, wheel_speed_radps)
    loss_dir = signed_direction(wheel_speed_radps, applied_torque)
    rolling = constants["rolling_friction_torque_nm"] * loss_dir
    command_torque = applied_torque
    if signed_direction(applied_torque, wheel_speed_radps) != 0.0:
        command_torque += launch_dir * launch + rolling
    return command_torque, launch


def motor_commands_for_opposing_torque(opposing_yaw_torque: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> dict[str, float]:
    radius = constants["wheel_radius_m"]
    track = constants["track_width_m"]
    left_surface, right_surface, left_speed, right_speed = wheel_speeds(vf_mps, yaw_rate, constants)
    applied_bank_torque = opposing_yaw_torque * radius / track
    left_torque, _ = command_torque_for_applied(applied_bank_torque, left_speed, constants)
    right_torque, _ = command_torque_for_applied(-applied_bank_torque, right_speed, constants)
    left_command = command_from_torque(left_torque, left_speed, constants)
    right_command = command_from_torque(right_torque, right_speed, constants)
    return {
        "applied_bank_torque_nm": applied_bank_torque,
        "left_command_torque_nm": left_torque,
        "right_command_torque_nm": right_torque,
        "left_command": left_command,
        "right_command": right_command,
        "lr_delta_command": left_command - right_command,
        "left_surface_mps": left_surface,
        "right_surface_mps": right_surface,
    }


def baseline_opposing_yaw_torque(constants: dict[str, float], yaw_rate: float) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    front_gain = constants["front_right_contact_force_gain_n_per_mps"]
    rear_gain = constants["rear_right_contact_force_gain_n_per_mps"]
    front_right_velocity = -longitudinal * yaw_rate
    rear_right_velocity = longitudinal * yaw_rate
    front_right_force_total = 2.0 * front_gain * front_right_velocity
    rear_right_force_total = 2.0 * rear_gain * rear_right_velocity
    yaw_moment = longitudinal * (front_right_force_total - rear_right_force_total)
    return -yaw_moment


def launch_extra_for_command(constants: dict[str, float], abs_command: float) -> tuple[float, dict[str, float]]:
    baseline = baseline_opposing_yaw_torque(constants, 1.0)
    lo = 0.0
    hi = 0.20
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        cmd = motor_commands_for_opposing_torque(baseline + mid, constants, 0.0, 1.0)
        if abs(cmd["left_command"]) >= abs_command and abs(cmd["right_command"]) >= abs_command:
            hi = mid
        else:
            lo = mid
    extra = hi
    cmd = motor_commands_for_opposing_torque(baseline + extra, constants, 0.0, 1.0)
    cmd["baseline_opposing_yaw_torque_nm"] = baseline
    cmd["extra_opposing_yaw_torque_nm"] = extra
    cmd["total_opposing_yaw_torque_nm"] = baseline + extra
    return extra, cmd


def pack(params: dict[str, float]) -> np.ndarray:
    out = []
    for bound in BOUNDS:
        x = min(max(params[bound.name], bound.lo + 1.0e-12), bound.hi - 1.0e-12)
        p = (x - bound.lo) / (bound.hi - bound.lo)
        out.append(math.log(p / (1.0 - p)))
    return np.asarray(out, dtype=float)


def unpack(values: np.ndarray) -> dict[str, float]:
    out = {}
    clipped = np.clip(values, -50.0, 50.0)
    for z, bound in zip(clipped, BOUNDS):
        p = 1.0 / (1.0 + math.exp(-float(z)))
        out[bound.name] = bound.lo + p * (bound.hi - bound.lo)
    return out


def model_predict_arrays(params: dict[str, float], arrays: dict[str, np.ndarray], constants: dict[str, float], launch_extra: float) -> tuple[np.ndarray, float]:
    half_track = 0.5 * constants["track_width_m"]
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    static_lever = math.hypot(half_track, longitudinal)

    yaw_activation = max(params["yaw_activation_mps"], 1.0e-9)
    stribeck_speed = max(params["stribeck_speed_mps"], 1.0e-9)
    slide_ratio = min(max(params["slide_ratio"], 0.0), 0.99)
    speed_gate = max(params["speed_gate_mps"], 1.0e-9)
    v_floor = max(params["v_floor_mps"], 1.0e-9)
    alpha_knee = max(params["alpha_knee_rad"], 1.0e-9)
    slip_cap = max(params["slip_cap_nm"], 0.0)
    alpha_lever = max(params["alpha_lever_m"], 1.0e-9)

    launch_v = static_lever
    launch_a = launch_v * launch_v / (launch_v * launch_v + yaw_activation * yaw_activation)
    launch_x = launch_v / stribeck_speed
    launch_x2 = launch_x * launch_x
    launch_s = 1.0 / (1.0 + launch_x2 + STRIBECK_QUARTIC * launch_x2 * launch_x2)
    launch_factor = slide_ratio + (1.0 - slide_ratio) * launch_s
    static_peak = launch_extra / max(launch_a * launch_factor, 1.0e-9)

    abs_yaw = arrays["abs_yaw"]
    abs_vf = arrays["abs_vf"]
    load = arrays["load"]

    vy = abs_yaw * static_lever
    vy2 = vy * vy
    activation = vy2 / (vy2 + yaw_activation * yaw_activation)
    sx = vy / stribeck_speed
    sx2 = sx * sx
    stribeck_shape = 1.0 / (1.0 + sx2 + STRIBECK_QUARTIC * sx2 * sx2)
    low_branch = static_peak * activation * (slide_ratio + (1.0 - slide_ratio) * stribeck_shape)

    gate_x = abs_vf / speed_gate
    low_gate = 1.0 / (1.0 + gate_x * gate_x)
    high_gate = 1.0 - low_gate

    alpha = abs_yaw * alpha_lever / (abs_vf + v_floor)
    alpha_norm = alpha / alpha_knee
    slip_branch = slip_cap * alpha_norm / np.sqrt(1.0 + alpha_norm * alpha_norm)

    pred = (low_gate * low_branch + high_gate * slip_branch) * load
    return pred, static_peak


def build_arrays(frame: pd.DataFrame) -> dict[str, np.ndarray]:
    return {
        "target": frame["residual_opposes_yaw_nm"].to_numpy(dtype=float),
        "raw": frame["residual_additive_yaw_torque_nm"].to_numpy(dtype=float),
        "yaw_sign": frame["yaw_sign"].to_numpy(dtype=float),
        "abs_yaw": np.abs(frame["yaw_rate_radps"].to_numpy(dtype=float)),
        "abs_vf": np.abs(frame["forward_velocity_mps"].to_numpy(dtype=float)),
        "load": frame["load_factor"].to_numpy(dtype=float),
    }


def fit_weights(frame: pd.DataFrame) -> np.ndarray:
    split = frame["dataset_split"].to_numpy()
    family = frame["family"].to_numpy()
    recommendation = frame["recommendation"].to_numpy()
    weights = np.zeros(len(frame), dtype=float)
    weights[split == "primary_open_floor_fit_authoritative"] = 1.0
    down = (split == "open_floor_fit_downweighted") & (family == "open_floor") & (recommendation == "fit_downweighted")
    weights[down] = 0.25
    weights[split == "open_floor_validation_only"] = 0.10
    weights[split == "diag_validation_only"] = 0.05
    weights[split == "aux_downweighted_validation"] = 0.05

    limiter = np.clip(frame["max_force_limiter_activity"].to_numpy(dtype=float), 0.0, 1.0)
    saturation = np.clip(frame["hardware_saturation_evidence"].to_numpy(dtype=float), 0.0, 1.0)
    spike = np.clip(frame["gyro_derivative_spike"].to_numpy(dtype=float), 0.0, 1.0)
    quality = (1.0 / (1.0 + 4.0 * limiter)) * (1.0 - 0.75 * saturation) * (1.0 - 0.90 * spike)
    weights *= np.clip(quality, 0.02, 1.0)

    counts = Counter(frame["run_id"].astype(str))
    rb = np.asarray([1.0 / max(counts[str(run_id)], 1) for run_id in frame["run_id"]], dtype=float)
    weights *= rb
    positive = weights > 0.0
    if np.any(positive):
        weights *= float(np.sum(positive)) / float(np.sum(weights[positive]))
    return weights


def objective_factory(arrays: dict[str, np.ndarray], weights: np.ndarray, constants: dict[str, float], launch_extra: float):
    target = arrays["target"]
    delta = 0.025
    active = weights > 0.0

    def objective(z: np.ndarray) -> float:
        params = unpack(z)
        pred, static_peak = model_predict_arrays(params, arrays, constants, launch_extra)
        residual = target[active] - pred[active]
        scaled = residual / delta
        robust = delta * delta * (np.sqrt(1.0 + scaled * scaled) - 1.0)
        score = math.sqrt(max(float(np.sum(weights[active] * robust) / np.sum(weights[active])), 0.0))
        if static_peak > 0.25:
            score += 0.020 * ((static_peak - 0.25) / 0.25) ** 2
        if not math.isfinite(score):
            return 1.0e9
        return score

    return objective


def nelder_mead(func, start: np.ndarray, step: float = 0.75, max_iter: int = 360, tol: float = 1.0e-7) -> tuple[np.ndarray, float, int]:
    n = len(start)
    simplex = [start.copy()]
    for i in range(n):
        point = start.copy()
        point[i] += step
        simplex.append(point)
    simplex = np.asarray(simplex)
    values = np.asarray([func(point) for point in simplex], dtype=float)

    alpha = 1.0
    gamma = 2.0
    rho = 0.5
    sigma = 0.5
    evals = len(values)

    for _ in range(max_iter):
        order = np.argsort(values)
        simplex = simplex[order]
        values = values[order]
        if float(np.max(np.abs(values[0] - values[1:]))) < tol:
            break
        centroid = np.mean(simplex[:-1], axis=0)
        worst = simplex[-1]
        reflected = centroid + alpha * (centroid - worst)
        reflected_value = func(reflected)
        evals += 1
        if values[0] <= reflected_value < values[-2]:
            simplex[-1] = reflected
            values[-1] = reflected_value
            continue
        if reflected_value < values[0]:
            expanded = centroid + gamma * (reflected - centroid)
            expanded_value = func(expanded)
            evals += 1
            if expanded_value < reflected_value:
                simplex[-1] = expanded
                values[-1] = expanded_value
            else:
                simplex[-1] = reflected
                values[-1] = reflected_value
            continue
        contracted = centroid + rho * (worst - centroid)
        contracted_value = func(contracted)
        evals += 1
        if contracted_value < values[-1]:
            simplex[-1] = contracted
            values[-1] = contracted_value
            continue
        best = simplex[0].copy()
        for i in range(1, n + 1):
            simplex[i] = best + sigma * (simplex[i] - best)
            values[i] = func(simplex[i])
        evals += n

    order = np.argsort(values)
    return simplex[order[0]], float(values[order[0]]), evals


def seed_params(constants: dict[str, float]) -> list[dict[str, float]]:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    half_track = 0.5 * constants["track_width_m"]
    hyp = math.hypot(longitudinal, half_track)
    seeds = [
        (0.008, 0.025, 0.00, 0.12, 0.06, 0.25, 0.025, longitudinal),
        (0.008, 0.025, 0.00, 0.64, 0.08, 0.30, 0.040, longitudinal),
        (0.008, 0.040, 0.15, 0.30, 0.10, 0.35, 0.040, longitudinal),
        (0.004, 0.060, 0.25, 0.70, 0.15, 0.50, 0.060, longitudinal),
        (0.012, 0.035, 0.05, 0.18, 0.05, 0.18, 0.035, half_track),
        (0.006, 0.080, 0.40, 0.45, 0.12, 0.45, 0.050, half_track),
        (0.015, 0.120, 0.60, 0.90, 0.20, 0.80, 0.080, hyp),
        (0.003, 0.020, 0.00, 0.05, 0.03, 0.12, 0.020, hyp),
        (0.020, 0.150, 0.80, 1.00, 0.30, 1.00, 0.100, hyp),
        (0.010, 0.050, 0.20, 0.08, 0.50, 0.20, 0.010, longitudinal),
        (0.005, 0.030, 0.00, 1.10, 0.04, 1.20, 0.120, 0.060),
        (0.025, 0.100, 0.50, 0.25, 0.25, 0.60, 0.030, 0.030),
    ]
    return [
        {
            "yaw_activation_mps": row[0],
            "stribeck_speed_mps": row[1],
            "slide_ratio": row[2],
            "speed_gate_mps": row[3],
            "v_floor_mps": row[4],
            "alpha_knee_rad": row[5],
            "slip_cap_nm": row[6],
            "alpha_lever_m": row[7],
        }
        for row in seeds
    ]


def finite(values: np.ndarray) -> np.ndarray:
    return values[np.isfinite(values)]


def rmse(values: np.ndarray) -> float:
    clean = finite(values)
    if len(clean) == 0:
        return 0.0
    return float(math.sqrt(float(np.mean(clean * clean))))


def mae(values: np.ndarray) -> float:
    clean = finite(values)
    if len(clean) == 0:
        return 0.0
    return float(np.mean(np.abs(clean)))


def median(values: np.ndarray) -> float:
    clean = finite(values)
    if len(clean) == 0:
        return 0.0
    return float(np.median(clean))


def run_balanced_weights(frame: pd.DataFrame) -> np.ndarray:
    counts = Counter(frame["run_id"].astype(str))
    weights = np.asarray([1.0 / max(counts[str(run_id)], 1) for run_id in frame["run_id"]], dtype=float)
    if float(np.sum(weights)) > 0.0:
        weights *= len(weights) / float(np.sum(weights))
    return weights


def weighted_rmse(values: np.ndarray, weights: np.ndarray) -> float:
    clean = np.isfinite(values) & np.isfinite(weights)
    if not np.any(clean):
        return 0.0
    total = float(np.sum(weights[clean]))
    if total <= 0.0:
        return 0.0
    return float(math.sqrt(max(float(np.sum(weights[clean] * values[clean] * values[clean]) / total), 0.0)))


def metric_row(group: str, frame: pd.DataFrame, pred: np.ndarray) -> dict[str, object]:
    raw = frame["residual_additive_yaw_torque_nm"].to_numpy(dtype=float)
    sign_yaw = frame["yaw_sign"].to_numpy(dtype=float)
    pred_raw = -sign_yaw * pred
    corrected_raw = raw - pred_raw
    rb = run_balanced_weights(frame)
    baseline_rmse = rmse(raw)
    corrected_rmse = rmse(corrected_raw)
    return {
        "group": group,
        "count": int(len(frame)),
        "run_count": int(frame["run_id"].nunique()) if len(frame) else 0,
        "baseline_rmse_nm": baseline_rmse,
        "corrected_rmse_nm": corrected_rmse,
        "baseline_mae_nm": mae(raw),
        "corrected_mae_nm": mae(corrected_raw),
        "baseline_median_abs_nm": median(np.abs(raw)),
        "corrected_median_abs_nm": median(np.abs(corrected_raw)),
        "baseline_signed_median_nm": median(raw),
        "corrected_signed_median_nm": median(corrected_raw),
        "run_balanced_baseline_rmse_nm": weighted_rmse(raw, rb),
        "run_balanced_corrected_rmse_nm": weighted_rmse(corrected_raw, rb),
        "prediction_median_opposes_nm": median(pred),
        "improvement_fraction_rmse": (baseline_rmse - corrected_rmse) / baseline_rmse if baseline_rmse > 0.0 else 0.0,
    }


def prediction_for_frame(params: dict[str, float], frame: pd.DataFrame, constants: dict[str, float], launch_extra: float) -> tuple[np.ndarray, float]:
    return model_predict_arrays(params, build_arrays(frame), constants, launch_extra)


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def read_reference_in_place(path: Path, label: str) -> dict[str, object] | None:
    if not path.exists():
        return None
    with path.open(newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            name = row.get("variant") or row.get("candidate") or label
            if label.lower() in name.lower() or label == "first":
                return {key: row.get(key, "") for key in row}
    return None


def comparison_split_rows(own_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    own_by_group = {str(row["group"]): row for row in own_rows}
    if not REFERENCE_SPLIT.exists():
        return []
    ref = pd.read_csv(REFERENCE_SPLIT)
    rows: list[dict[str, object]] = []
    for _, r in ref.iterrows():
        group = str(r["group"])
        own = own_by_group.get(group)
        if own is None:
            continue
        rows.append(
            {
                "split": group,
                "count": int(r["count"]),
                "current_baseline_rmse_nm": float(r["baseline_rmse_nm"]),
                "force_domain_stribeck_rmse_nm": float(r["force_domain_stribeck_rmse_nm"]) if pd.notna(r.get("force_domain_stribeck_rmse_nm")) else "",
                "rational_residual_reference_rmse_nm": float(r["rational_residual_reference_rmse_nm"]) if pd.notna(r.get("rational_residual_reference_rmse_nm")) else "",
                "standalone_contact_traction_rmse_nm": float(r["standalone_rmse_nm"]) if pd.notna(r.get("standalone_rmse_nm")) else "",
                "scalar_slip_angle_partition_rmse_nm": own["corrected_rmse_nm"],
            }
        )
    return rows


def comparison_selected_rows(own_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    own_by_run = {str(row["run_id"]): row for row in own_rows if row.get("present")}
    if not REFERENCE_SELECTED.exists():
        return []
    ref = pd.read_csv(REFERENCE_SELECTED)
    rows: list[dict[str, object]] = []
    for _, r in ref.iterrows():
        run_id = str(r["run_id"])
        own = own_by_run.get(run_id)
        if own is None:
            rows.append({"run_id": run_id, "present": False})
            continue
        rows.append(
            {
                "run_id": run_id,
                "present": True,
                "dataset_split": own["dataset_split"],
                "count": int(own["count"]),
                "current_baseline_rmse_nm": float(r["baseline_rmse_nm"]),
                "force_domain_stribeck_rmse_nm": float(r["force_domain_stribeck_rmse_nm"]) if pd.notna(r.get("force_domain_stribeck_rmse_nm")) else "",
                "rational_residual_reference_rmse_nm": float(r["rational_residual_reference_rmse_nm"]) if pd.notna(r.get("rational_residual_reference_rmse_nm")) else "",
                "standalone_contact_traction_rmse_nm": float(r["standalone_rmse_nm"]) if pd.notna(r.get("standalone_rmse_nm")) else "",
                "scalar_slip_angle_partition_rmse_nm": own["corrected_rmse_nm"],
                "scalar_signed_median_nm": own["corrected_signed_median_nm"],
            }
        )
    return rows


def format_float(value: object, digits: int = 6) -> str:
    if value == "":
        return ""
    try:
        x = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(x):
        return ""
    return f"{x:.{digits}f}"


def make_report(
    params: dict[str, float],
    static_peak: float,
    launch_cmd: dict[str, float],
    optimizer_rows: list[dict[str, object]],
    split_rows: list[dict[str, object]],
    selected_rows: list[dict[str, object]],
    risk_rows: list[dict[str, object]],
    split_comparison: list[dict[str, object]],
    selected_comparison: list[dict[str, object]],
    constants: dict[str, float],
    nominal_load: float,
) -> None:
    near_bounds = []
    for bound in BOUNDS:
        value = params[bound.name]
        span = bound.hi - bound.lo
        if value <= bound.lo + 0.03 * span:
            near_bounds.append(f"{bound.name}=near lower bound")
        if value >= bound.hi - 0.03 * span:
            near_bounds.append(f"{bound.name}=near upper bound")

    lines: list[str] = []
    lines.append("# Scalar Slip-Angle Partition Yaw Support")
    lines.append("")
    lines.append("Analysis-only output. Production code, build metadata, tests, and prior analysis artifacts were not edited.")
    lines.append("")
    lines.append("## Model Form")
    lines.append("")
    lines.append("The model predicts a positive yaw-opposing support torque `M_opp`, then converts it to additive yaw torque with `M_add = -sign(yawRate) * M_opp` for residual evaluation.")
    lines.append("")
    lines.append("Low-speed branch:")
    lines.append("")
    lines.append("`v_y = abs(yawRate) * hypot(track_width/2, drive_wheel_longitudinal_offset)`")
    lines.append("")
    lines.append("`A_y = v_y^2 / (v_y^2 + k_y^2)`")
    lines.append("")
    lines.append("`S = 1 / (1 + (v_y/k_s)^2 + 0.2*(v_y/k_s)^4)`")
    lines.append("")
    lines.append("`M_stribeck = M0 * A_y * (slide_ratio + (1-slide_ratio)*S)`")
    lines.append("")
    lines.append("High-speed branch:")
    lines.append("")
    lines.append("`alpha_proxy = abs(yawRate) * alpha_lever / (abs(Vf) + V_floor)`")
    lines.append("")
    lines.append("`M_alpha = M_cap * (alpha_proxy/alpha_knee) / sqrt(1 + (alpha_proxy/alpha_knee)^2)`")
    lines.append("")
    lines.append("Partition:")
    lines.append("")
    lines.append("`G_low = 1 / (1 + (abs(Vf)/V_gate)^2)`")
    lines.append("")
    lines.append("`M_opp = load_factor * (G_low*M_stribeck + (1-G_low)*M_alpha)`")
    lines.append("")
    lines.append("There is no command, request, preprojection, trig, tanh, exp, lookup table, old-force branch, or residual table in the fitted law. The only exponential in the script is reused in the existing motor static-launch command estimate, outside the model.")
    lines.append("")
    lines.append("## Launch Constraint")
    lines.append("")
    lines.append(f"The Stribeck peak `M0` is analytically derived so `Vf=0`, `yawRate=+1 rad/s` produces the requested command target `{LAUNCH_COMMAND_TARGET:.3f}`. The resulting estimate is left/right `{launch_cmd['left_command']:.6f}/{launch_cmd['right_command']:.6f}`, extra support {launch_cmd['extra_opposing_yaw_torque_nm']:.6f} Nm, and total opposing yaw torque {launch_cmd['total_opposing_yaw_torque_nm']:.6f} Nm.")
    lines.append("")
    lines.append("## Selected Parameters")
    lines.append("")
    lines.append("| parameter | value |")
    lines.append("| --- | ---: |")
    for bound in BOUNDS:
        lines.append(f"| {bound.name} | {params[bound.name]:.9f} |")
    lines.append(f"| stribeck_quartic_fixed | {STRIBECK_QUARTIC:.9f} |")
    lines.append(f"| derived_static_peak_nm | {static_peak:.9f} |")
    lines.append(f"| nominal_load_n | {nominal_load:.9f} |")
    lines.append(f"| static_lever_m | {math.hypot(0.5 * constants['track_width_m'], constants['drive_wheel_longitudinal_offset_m']):.9f} |")
    lines.append("")
    lines.append("## Optimizer")
    lines.append("")
    best = min(optimizer_rows, key=lambda row: float(row["objective_score"]))
    lines.append(f"Bounded Nelder-Mead was run from {len(optimizer_rows)} continuous seeds using primary fit rows plus a light non-authoritative validation guard. Best objective `{float(best['objective_score']):.9f}` came from seed `{best['seed']}` after `{best['evals']}` evaluations.")
    if near_bounds:
        lines.append(f"Boundary notes: {', '.join(near_bounds)}.")
    else:
        lines.append("No selected parameter finished within 3% of its configured bound.")
    lines.append("")
    lines.append("| seed | objective | evals | static_peak_nm | boundary_flags |")
    lines.append("| ---: | ---: | ---: | ---: | --- |")
    for row in sorted(optimizer_rows, key=lambda row: float(row["objective_score"]))[:8]:
        lines.append(f"| {row['seed']} | {float(row['objective_score']):.9f} | {row['evals']} | {float(row['derived_static_peak_nm']):.6f} | {row['boundary_flags']} |")
    lines.append("")
    lines.append("## Split Metrics")
    lines.append("")
    lines.append("| split | count | baseline RMSE | scalar RMSE | improvement |")
    lines.append("| --- | ---: | ---: | ---: | ---: |")
    for row in split_rows:
        lines.append(f"| {row['group']} | {row['count']} | {float(row['baseline_rmse_nm']):.6f} | {float(row['corrected_rmse_nm']):.6f} | {100.0 * float(row['improvement_fraction_rmse']):.2f}% |")
    lines.append("")
    lines.append("## Comparison")
    lines.append("")
    lines.append("| split | baseline | force Stribeck | rational residual | standalone traction | scalar partition |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: |")
    for row in split_comparison:
        lines.append(
            f"| {row['split']} | {format_float(row['current_baseline_rmse_nm'])} | {format_float(row['force_domain_stribeck_rmse_nm'])} | "
            f"{format_float(row['rational_residual_reference_rmse_nm'])} | {format_float(row['standalone_contact_traction_rmse_nm'])} | {format_float(row['scalar_slip_angle_partition_rmse_nm'])} |"
        )
    lines.append("")
    lines.append("## Selected Logs")
    lines.append("")
    lines.append("| run | split | baseline RMSE | scalar RMSE | signed median after |")
    lines.append("| --- | --- | ---: | ---: | ---: |")
    for row in selected_rows:
        if not row.get("present"):
            lines.append(f"| {row['run_id']} | absent |  |  |  |")
            continue
        lines.append(f"| {row['run_id']} | {row['dataset_split']} | {float(row['baseline_rmse_nm']):.6f} | {float(row['corrected_rmse_nm']):.6f} | {float(row['corrected_signed_median_nm']):.6f} |")
    lines.append("")
    lines.append("## May 4 Focus")
    lines.append("")
    for run_id in ["2026-05-04_20-35-47", "2026-05-04_16-57-53"]:
        row = next((r for r in selected_comparison if r.get("run_id") == run_id and r.get("present")), None)
        if row:
            lines.append(f"- `{run_id}`: baseline {format_float(row['current_baseline_rmse_nm'])}, force Stribeck {format_float(row['force_domain_stribeck_rmse_nm'])}, rational residual {format_float(row['rational_residual_reference_rmse_nm'])}, standalone traction {format_float(row['standalone_contact_traction_rmse_nm'])}, scalar partition {format_float(row['scalar_slip_angle_partition_rmse_nm'])}.")
        else:
            lines.append(f"- `{run_id}`: absent from selected comparison.")
    lines.append("")
    lines.append("## Risk Slices")
    lines.append("")
    lines.append("| slice | count | baseline RMSE | scalar RMSE | improvement | median abs after |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: |")
    for row in risk_rows:
        lines.append(f"| {row['group']} | {row['count']} | {float(row['baseline_rmse_nm']):.6f} | {float(row['corrected_rmse_nm']):.6f} | {100.0 * float(row['improvement_fraction_rmse']):.2f}% | {float(row['corrected_median_abs_nm']):.6f} |")
    lines.append("")
    lines.append("## Assessment")
    lines.append("")
    primary = next(row for row in split_rows if row["group"] == "primary_open_floor_fit_authoritative")
    validation = next(row for row in split_rows if row["group"] == "validation_non_authoritative")
    lines.append(f"- Fit-authoritative corrected RMSE: `{float(primary['corrected_rmse_nm']):.6f}` Nm versus baseline `{float(primary['baseline_rmse_nm']):.6f}` Nm.")
    lines.append(f"- Non-authoritative validation corrected RMSE: `{float(validation['corrected_rmse_nm']):.6f}` Nm versus baseline `{float(validation['baseline_rmse_nm']):.6f}` Nm.")
    lines.append("- Viability: useful as a very cheap launch-preserving scalar support law, but not competitive with the standalone contact-patch traction testbed and generally behind the rational residual reference. The positive-support constraint is the main limitation on splits where the current plant already over-resists yaw.")
    lines.append("- Cost estimate: per tick this law needs abs/multiply operations, three rational divides, and two square roots. It uses no trig/atan, tanh, exp, table lookup, or persistent state.")
    lines.append("")
    lines.append("## Outputs")
    lines.append("")
    for name in [
        "fit_scalar_slip_angle_partition.py",
        "scalar_slip_angle_partition_report.md",
        "optimizer_summary.csv",
        "selected_parameters.csv",
        "split_metrics.csv",
        "phase_metrics.csv",
        "selected_log_metrics.csv",
        "risk_slices.csv",
        "split_comparison_vs_references.csv",
        "selected_log_comparison_vs_references.csv",
        "launch_command_estimate.csv",
        "prediction_sample.csv",
        "commands_run.txt",
    ]:
        lines.append(f"- `{name}`")
    (OUT / "scalar_slip_angle_partition_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    constants = read_constants()
    frame, nominal_load = load_frame(constants)
    arrays = build_arrays(frame)
    weights = fit_weights(frame)
    launch_extra, launch_cmd = launch_extra_for_command(constants, LAUNCH_COMMAND_TARGET)
    objective = objective_factory(arrays, weights, constants, launch_extra)

    optimizer_rows: list[dict[str, object]] = []
    best_z: np.ndarray | None = None
    best_score = float("inf")
    for i, seed in enumerate(seed_params(constants), start=1):
        z0 = pack(seed)
        z, score, evals = nelder_mead(objective, z0)
        params = unpack(z)
        _, static_peak = model_predict_arrays(params, arrays, constants, launch_extra)
        flags = []
        for bound in BOUNDS:
            value = params[bound.name]
            span = bound.hi - bound.lo
            if value <= bound.lo + 0.03 * span:
                flags.append(f"{bound.name}:low")
            if value >= bound.hi - 0.03 * span:
                flags.append(f"{bound.name}:high")
        row = {
            "seed": i,
            "objective_score": score,
            "evals": evals,
            "derived_static_peak_nm": static_peak,
            "boundary_flags": ";".join(flags),
        }
        row.update(params)
        optimizer_rows.append(row)
        if score < best_score:
            best_score = score
            best_z = z

    if best_z is None:
        raise RuntimeError("optimizer produced no result")
    selected = unpack(best_z)
    pred_all, static_peak = model_predict_arrays(selected, arrays, constants, launch_extra)

    split_rows: list[dict[str, object]] = []
    for split in SPLITS:
        mask = (frame["dataset_split"] == split).to_numpy()
        split_rows.append(metric_row(split, frame.loc[mask], pred_all[mask]))
    validation_mask = (frame["dataset_split"] != "primary_open_floor_fit_authoritative").to_numpy()
    split_rows.append(metric_row("validation_non_authoritative", frame.loc[validation_mask], pred_all[validation_mask]))

    phase_rows: list[dict[str, object]] = []
    for split in SPLITS:
        for phase in ["entry", "plateau", "exit"]:
            mask = ((frame["dataset_split"] == split) & (frame["physics_phase"] == phase)).to_numpy()
            if np.any(mask):
                row = metric_row(f"{split}:{phase}", frame.loc[mask], pred_all[mask])
                row["dataset_split"] = split
                row["physics_phase"] = phase
                phase_rows.append(row)

    selected_rows: list[dict[str, object]] = []
    for run_id in SELECTED_RUNS:
        mask = (frame["run_id"] == run_id).to_numpy()
        if not np.any(mask):
            selected_rows.append({"run_id": run_id, "present": False, "count": 0, "dataset_split": ""})
            continue
        row = metric_row(run_id, frame.loc[mask], pred_all[mask])
        row["run_id"] = run_id
        row["present"] = True
        row["dataset_split"] = ";".join(sorted(str(x) for x in frame.loc[mask, "dataset_split"].unique()))
        row["family"] = ";".join(sorted(str(x) for x in frame.loc[mask, "family"].unique()))
        selected_rows.append(row)

    risk_specs = [
        ("straightish_abs_yaw_lt_0p05", np.abs(frame["yaw_rate_radps"].to_numpy(dtype=float)) < 0.05),
        (
            "straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05",
            (np.abs(frame["yaw_rate_radps"].to_numpy(dtype=float)) < 0.05)
            & (np.abs(frame["forward_velocity_mps"].to_numpy(dtype=float)) >= 0.05),
        ),
        (
            "low_speed_yaw_vf_lt_0p05_yaw_ge_0p2",
            (np.abs(frame["forward_velocity_mps"].to_numpy(dtype=float)) < 0.05)
            & (np.abs(frame["yaw_rate_radps"].to_numpy(dtype=float)) >= 0.2),
        ),
        (
            "low_speed_launchish_vf_lt_0p15_yaw_ge_0p5",
            (np.abs(frame["forward_velocity_mps"].to_numpy(dtype=float)) < 0.15)
            & (np.abs(frame["yaw_rate_radps"].to_numpy(dtype=float)) >= 0.5),
        ),
        ("high_forward_vf_ge_0p5", np.abs(frame["forward_velocity_mps"].to_numpy(dtype=float)) >= 0.5),
        ("high_speed_abs_vf_ge_0p7", np.abs(frame["forward_velocity_mps"].to_numpy(dtype=float)) >= 0.7),
        ("limiter_active", frame["max_force_limiter_activity"].to_numpy(dtype=float) > 0.0),
        ("hardware_saturation_evidence", frame["hardware_saturation_evidence"].to_numpy(dtype=float) > 0.0),
        ("negative_yaw_opposes_target", frame["residual_opposes_yaw_nm"].to_numpy(dtype=float) < 0.0),
        ("may4_selected_logs", frame["run_id"].isin(["2026-05-04_20-35-47", "2026-05-04_16-57-53"]).to_numpy()),
    ]
    risk_rows = []
    for name, mask in risk_specs:
        if np.any(mask):
            risk_rows.append(metric_row(name, frame.loc[mask], pred_all[mask]))

    split_comparison = comparison_split_rows(split_rows)
    selected_comparison = comparison_selected_rows(selected_rows)

    baseline_total = baseline_opposing_yaw_torque(constants, 1.0)
    scalar_cmd = motor_commands_for_opposing_torque(baseline_total + launch_extra, constants, 0.0, 1.0)
    scalar_cmd.update(
        {
            "variant": "ScalarSlipAnglePartition",
            "baseline_opposing_yaw_torque_nm": baseline_total,
            "extra_opposing_yaw_torque_nm": launch_extra,
            "total_opposing_yaw_torque_nm": baseline_total + launch_extra,
            "max_abs_command": max(abs(scalar_cmd["left_command"]), abs(scalar_cmd["right_command"])),
            "passes_abs_0p6_gate": int(abs(scalar_cmd["left_command"]) >= HARD_GATE_COMMAND_MIN and abs(scalar_cmd["right_command"]) >= HARD_GATE_COMMAND_MIN),
        }
    )
    launch_rows: list[dict[str, object]] = [
        {
            "variant": "Current baseline",
            "baseline_opposing_yaw_torque_nm": baseline_total,
            "extra_opposing_yaw_torque_nm": 0.0,
            "total_opposing_yaw_torque_nm": baseline_total,
            **motor_commands_for_opposing_torque(baseline_total, constants, 0.0, 1.0),
        },
        scalar_cmd,
    ]
    force_ref = read_reference_in_place(FORCE_IN_PLACE, "ForceDomainStribeck")
    if force_ref:
        launch_rows.append({"variant": "ForceDomainStribeck_reference", **force_ref})
    standalone_ref = read_reference_in_place(STANDALONE_IN_PLACE, "first")
    if standalone_ref:
        launch_rows.append({"variant": "StandaloneContactTraction_reference", **standalone_ref})

    prediction_sample = frame.loc[:, ["run_id", "dataset_split", "physics_phase", "row_index", "forward_velocity_mps", "yaw_rate_radps", "residual_additive_yaw_torque_nm", "residual_opposes_yaw_nm"]].copy()
    prediction_sample["scalar_predicted_opposes_nm"] = pred_all
    prediction_sample["scalar_predicted_additive_nm"] = -frame["yaw_sign"].to_numpy(dtype=float) * pred_all
    prediction_sample["scalar_corrected_additive_nm"] = prediction_sample["residual_additive_yaw_torque_nm"] - prediction_sample["scalar_predicted_additive_nm"]
    prediction_sample = prediction_sample.iloc[np.linspace(0, len(prediction_sample) - 1, min(1000, len(prediction_sample)), dtype=int)]

    selected_param_rows = [{"parameter": key, "value": value} for key, value in selected.items()]
    selected_param_rows.extend(
        [
            {"parameter": "stribeck_quartic_fixed", "value": STRIBECK_QUARTIC},
            {"parameter": "derived_static_peak_nm", "value": static_peak},
            {"parameter": "launch_extra_opposing_nm", "value": launch_extra},
            {"parameter": "launch_target_abs_command", "value": LAUNCH_COMMAND_TARGET},
            {"parameter": "nominal_load_n", "value": nominal_load},
        ]
    )

    write_csv(OUT / "optimizer_summary.csv", optimizer_rows)
    write_csv(OUT / "selected_parameters.csv", selected_param_rows)
    write_csv(OUT / "split_metrics.csv", split_rows)
    write_csv(OUT / "phase_metrics.csv", phase_rows)
    write_csv(OUT / "selected_log_metrics.csv", selected_rows)
    write_csv(OUT / "risk_slices.csv", risk_rows)
    write_csv(OUT / "split_comparison_vs_references.csv", split_comparison)
    write_csv(OUT / "selected_log_comparison_vs_references.csv", selected_comparison)
    write_csv(OUT / "launch_command_estimate.csv", launch_rows)
    prediction_sample.to_csv(OUT / "prediction_sample.csv", index=False)
    (OUT / "commands_run.txt").write_text(
        "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' "
        "codex_analysis\\yaw_model_variant_fits\\stribeck_to_slip_angle_candidates\\scalar_slip_angle_partition\\fit_scalar_slip_angle_partition.py\n",
        encoding="utf-8",
    )
    make_report(
        selected,
        static_peak,
        scalar_cmd,
        optimizer_rows,
        split_rows,
        selected_rows,
        risk_rows,
        split_comparison,
        selected_comparison,
        constants,
        nominal_load,
    )


if __name__ == "__main__":
    main()
