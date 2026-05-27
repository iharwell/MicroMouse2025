#!/usr/bin/env python3
"""Smoothstep transition fits for yaw launch authority plus moving contact.

Analysis-only tooling. Outputs are written beside this script. Production code,
build metadata, and tests are intentionally untouched.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent

PRIMARY_INPUT = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "ablation"
    / "phase_classified_feature_sample.csv"
)
SECONDARY_INPUT = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "features"
    / "contact_continuum_feature_sample.csv"
)
CONSTANTS_INPUT = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "features"
    / "plant_mirror_constants.csv"
)
B_REFERENCE_DIR = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck"
C_REFERENCE_DIR = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface"

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
    "measured_yaw_accel_radps2",
    "vbar_rel_mps",
    "vbar_yaw_mps",
    "max_force_preprojection_utilization",
    "max_force_limiter_activity",
    "hardware_saturation_evidence",
    "gyro_derivative_spike",
    "residual_additive_yaw_torque_nm",
    "residual_opposes_yaw_nm",
    "patch_yaw_force_basis_nm",
]

SECONDARY_COLUMNS = [
    "run_id",
    "row_index",
    "total_normal_load_n",
    "fl_normal_n",
    "fr_normal_n",
    "rl_normal_n",
    "rr_normal_n",
    "fl_v_rel_f_mps",
    "fr_v_rel_f_mps",
    "rl_v_rel_f_mps",
    "rr_v_rel_f_mps",
    "fl_v_rel_r_mps",
    "fr_v_rel_r_mps",
    "rl_v_rel_r_mps",
    "rr_v_rel_r_mps",
    "fl_force_f_n",
    "fr_force_f_n",
    "rl_force_f_n",
    "rr_force_f_n",
    "fl_force_r_n",
    "fr_force_r_n",
    "rl_force_r_n",
    "rr_force_r_n",
]

C_FEATURES = [
    "gain_right_total_basis__base",
    "gain_right_total_basis__low_rel",
    "gain_right_total_basis__high_forward",
    "gain_right_total_basis__util",
    "gain_long_total_basis__base",
    "gain_long_total_basis__low_rel",
    "gain_long_total_basis__high_forward",
    "gain_long_total_basis__util",
    "force_moment_opposes_yaw_nm__high_forward",
    "force_moment_opposes_yaw_nm__util",
]


@dataclass(frozen=True)
class Candidate:
    role: str
    form: str
    smoothstep: str
    transition_variable: str
    v0_mps: float
    v1_mps: float
    rel_weight: float
    util_k: float
    launch_activation: str
    k_launch_nm: float
    k_unconstrained_nm: float
    k_gate_min_nm: float
    primary_rmse_nm: float
    validation_non_authoritative_rmse_nm: float
    open_floor_validation_rmse_nm: float
    high_transition_rmse_nm: float
    low_speed_yaw_rmse_nm: float
    straightish_rmse_nm: float
    in_place_left_command: float
    in_place_right_command: float
    in_place_extra_opposing_nm: float
    score: float


def read_constants() -> dict[str, float]:
    table = pd.read_csv(CONSTANTS_INPUT)
    return {str(row.name): float(row.value) for row in table.itertuples(index=False)}


def sign_array(values: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    return (values > eps).astype(float) - (values < -eps).astype(float)


def sign(value: float, eps: float = 1.0e-6) -> float:
    return float((value > eps) - (value < -eps))


def smooth_positive(values: np.ndarray | float, epsilon: float = 1.0e-6) -> np.ndarray | float:
    return 0.5 * (values + np.sqrt(values * values + epsilon * epsilon))


def cubic_smoothstep(t: np.ndarray | float) -> np.ndarray | float:
    x = np.clip(t, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def quintic_smoothstep(t: np.ndarray | float) -> np.ndarray | float:
    x = np.clip(t, 0.0, 1.0)
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x))


def smoothstep(kind: str, t: np.ndarray | float) -> np.ndarray | float:
    return quintic_smoothstep(t) if kind == "quintic" else cubic_smoothstep(t)


def read_keyed_csv(path: Path, key: str) -> dict[str, dict[str, str]]:
    if not path.exists():
        return {}
    with path.open(newline="", encoding="utf-8") as handle:
        return {row[key]: row for row in csv.DictReader(handle)}


def load_frame(constants: dict[str, float]) -> tuple[pd.DataFrame, dict[str, float]]:
    frame = pd.read_csv(PRIMARY_INPUT, usecols=PRIMARY_COLUMNS)
    secondary = pd.read_csv(SECONDARY_INPUT, usecols=SECONDARY_COLUMNS)
    frame = frame.merge(secondary, how="left", on=["run_id", "row_index"])

    for column in set(PRIMARY_COLUMNS + SECONDARY_COLUMNS) - {
        "run_id",
        "family",
        "schema",
        "recommendation",
        "dataset_split",
        "physics_phase",
    }:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(
        subset=[
            "forward_velocity_mps",
            "yaw_rate_radps",
            "vbar_rel_mps",
            "vbar_yaw_mps",
            "residual_additive_yaw_torque_nm",
            "residual_opposes_yaw_nm",
            "patch_yaw_force_basis_nm",
        ]
    )
    frame = frame[frame["dataset_split"] != "excluded_or_unclassified"].copy()

    primary = frame["dataset_split"] == "primary_open_floor_fit_authoritative"
    nominal_total_load = float(frame.loc[primary, "total_normal_load_n"].median())
    if not math.isfinite(nominal_total_load) or nominal_total_load <= 0.0:
        nominal_total_load = constants["mass_kg"] * 9.80665 + constants.get(
            "fan_downforce_full_duty_n", 0.0
        )
    frame["total_normal_load_n"] = frame["total_normal_load_n"].fillna(nominal_total_load)
    for contact in ["fl", "fr", "rl", "rr"]:
        frame[f"{contact}_normal_n"] = frame[f"{contact}_normal_n"].fillna(
            frame["total_normal_load_n"] / 4.0
        )

    yaw_sign = sign_array(frame["yaw_rate_radps"].to_numpy())
    fallback_sign = sign_array(frame["patch_yaw_force_basis_nm"].to_numpy())
    yaw_sign = np.where(yaw_sign == 0.0, fallback_sign, yaw_sign)
    frame["yaw_sign"] = yaw_sign
    frame["abs_forward_velocity_mps"] = frame["forward_velocity_mps"].abs()
    frame["abs_yaw_rate_radps"] = frame["yaw_rate_radps"].abs()
    frame["util_smooth"] = np.clip(frame["max_force_preprojection_utilization"], 0.0, 5.0) / (
        1.0 + np.clip(frame["max_force_preprojection_utilization"], 0.0, 5.0)
    )
    frame["limiter_smooth"] = np.clip(frame["max_force_limiter_activity"], 0.0, 5.0) / (
        1.0 + np.clip(frame["max_force_limiter_activity"], 0.0, 5.0)
    )

    half_track = 0.5 * constants["track_width_m"]
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    contacts = {
        "fl": (-half_track, longitudinal),
        "fr": (half_track, longitudinal),
        "rl": (-half_track, -longitudinal),
        "rr": (half_track, -longitudinal),
    }
    add_contact_bases(frame, contacts)

    mu_ref = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / nominal_total_load
    total_normal = (
        frame["fl_normal_n"] + frame["fr_normal_n"] + frame["rl_normal_n"] + frame["rr_normal_n"]
    )
    frame["yield_longitudinal_moment_nm"] = mu_ref * half_track * total_normal
    metadata = {
        "nominal_total_load_n": nominal_total_load,
        "mu_ref": mu_ref,
        "nominal_longitudinal_yield_nm": float(
            frame.loc[primary, "yield_longitudinal_moment_nm"].median()
        ),
        "rows": int(len(frame)),
        "runs": int(frame["run_id"].nunique()),
    }
    return frame, metadata


def add_contact_bases(
    frame: pd.DataFrame, contacts: dict[str, tuple[float, float]]
) -> None:
    yaw_sign = frame["yaw_sign"].to_numpy()
    right_total = np.zeros(len(frame))
    long_total = np.zeros(len(frame))
    force_moment = np.zeros(len(frame))
    load_weighted_rel = np.zeros(len(frame))
    total_normal = np.maximum(frame["total_normal_load_n"].to_numpy(), 1.0e-9)
    for name, (r_pos, f_pos) in contacts.items():
        vf_rel = frame[f"{name}_v_rel_f_mps"].fillna(0.0).to_numpy()
        vr_rel = frame[f"{name}_v_rel_r_mps"].fillna(0.0).to_numpy()
        normal_frac = frame[f"{name}_normal_n"].fillna(0.0).to_numpy() / total_normal
        right_total += -yaw_sign * f_pos * vr_rel
        long_total += yaw_sign * r_pos * vf_rel
        force_f = frame[f"{name}_force_f_n"].fillna(0.0).to_numpy()
        force_r = frame[f"{name}_force_r_n"].fillna(0.0).to_numpy()
        force_moment += f_pos * force_r - r_pos * force_f
        load_weighted_rel += normal_frac * np.sqrt(vf_rel * vf_rel + vr_rel * vr_rel)
    frame["gain_right_total_basis"] = right_total
    frame["gain_long_total_basis"] = long_total
    frame["force_moment_opposes_yaw_nm"] = -yaw_sign * force_moment
    frame["load_weighted_rel_mps"] = load_weighted_rel


def training_weights(frame: pd.DataFrame) -> np.ndarray:
    weights = np.zeros(len(frame), dtype=float)
    split = frame["dataset_split"].to_numpy()
    family = frame["family"].to_numpy()
    recommendation = frame["recommendation"].to_numpy()
    weights[split == "primary_open_floor_fit_authoritative"] = 1.0
    weights[
        (split == "open_floor_fit_downweighted")
        & (family == "open_floor")
        & (recommendation == "fit_downweighted")
    ] = 0.25
    limiter = np.clip(frame["max_force_limiter_activity"].to_numpy(), 0.0, 1.0)
    saturation = np.clip(frame["hardware_saturation_evidence"].to_numpy(), 0.0, 1.0)
    spike = np.clip(frame["gyro_derivative_spike"].to_numpy(), 0.0, 1.0)
    quality = (1.0 / (1.0 + 4.0 * limiter)) * (1.0 - 0.75 * saturation) * (
        1.0 - 0.75 * spike
    )
    weights *= np.clip(quality, 0.02, 1.0)
    fit_counts = frame.loc[weights > 0.0, "run_id"].value_counts()
    if not fit_counts.empty:
        run_scale = frame["run_id"].map(
            {run: 1.0 / math.sqrt(count) for run, count in fit_counts.items()}
        ).fillna(0.0)
        weights *= run_scale.to_numpy()
        positive = weights > 0.0
        weights[positive] *= positive.sum() / weights[positive].sum()
    return weights


def c_schedules(frame: pd.DataFrame, vrel_knee: float = 0.06, fwd_knee: float = 0.70) -> dict[str, np.ndarray]:
    vrel = np.maximum(frame["vbar_rel_mps"].to_numpy(), frame["load_weighted_rel_mps"].to_numpy())
    vf = frame["abs_forward_velocity_mps"].to_numpy()
    low_rel = 1.0 / (1.0 + np.square(vrel / vrel_knee))
    low_forward = 1.0 / (1.0 + np.square(vf / fwd_knee))
    return {
        "base": np.ones(len(frame)),
        "low_rel": low_rel,
        "high_forward": 1.0 - low_forward,
        "util": frame["util_smooth"].to_numpy(),
    }


def build_c_features(frame: pd.DataFrame) -> tuple[np.ndarray, list[float]]:
    schedules = c_schedules(frame)
    columns: list[np.ndarray] = []
    for feature in C_FEATURES:
        base, suffix = feature.split("__", 1)
        columns.append(frame[base].to_numpy() * schedules[suffix])
    raw = np.column_stack(columns)
    scales = []
    for col in raw.T:
        scale = float(np.quantile(np.abs(col), 0.80))
        if not math.isfinite(scale) or scale < 1.0e-12:
            scale = float(np.sqrt(np.mean(col * col))) if len(col) else 1.0
        scales.append(scale if scale > 1.0e-12 else 1.0)
    return raw / np.array(scales), scales


def weighted_ridge(features: np.ndarray, target: np.ndarray, weights: np.ndarray, ridge: float) -> np.ndarray:
    sqrt_w = np.sqrt(np.clip(weights, 0.0, None))
    xw = features * sqrt_w[:, None]
    yw = target * sqrt_w
    xtx = xw.T @ xw
    xty = xw.T @ yw
    xtx.flat[:: xtx.shape[0] + 1] += ridge
    return np.linalg.solve(xtx, xty)


def corrected_residuals(frame: pd.DataFrame, pred_opposes: np.ndarray) -> np.ndarray:
    pred_additive = -frame["yaw_sign"].to_numpy() * pred_opposes
    return frame["residual_additive_yaw_torque_nm"].to_numpy() - pred_additive


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else math.nan


def mae(values: np.ndarray) -> float:
    return float(np.mean(np.abs(values))) if len(values) else math.nan


def transition_speed(frame: pd.DataFrame, rel_weight: float, variable: str) -> np.ndarray:
    if variable == "vbar_rel_only":
        return rel_weight * frame["vbar_rel_mps"].to_numpy()
    return np.sqrt(
        np.square(rel_weight * frame["vbar_rel_mps"].to_numpy())
        + np.square(frame["abs_forward_velocity_mps"].to_numpy())
    )


def launch_basis(frame: pd.DataFrame, util_k: float, activation_kind: str) -> np.ndarray:
    source = smooth_positive(frame["patch_yaw_force_basis_nm"].to_numpy(), epsilon=1.0e-6)
    utilization = source / np.maximum(frame["yield_longitudinal_moment_nm"].to_numpy(), 1.0e-12)
    return smoothstep(activation_kind, utilization / max(util_k, 1.0e-6))


def candidate_prediction(
    frame: pd.DataFrame,
    c_pred: np.ndarray,
    candidate: Candidate,
) -> np.ndarray:
    speed = transition_speed(frame, candidate.rel_weight, candidate.transition_variable)
    h = smoothstep(candidate.smoothstep, (speed - candidate.v0_mps) / (candidate.v1_mps - candidate.v0_mps))
    launch = candidate.k_launch_nm * launch_basis(frame, candidate.util_k, candidate.launch_activation)
    if candidate.form == "additive_no_subtract":
        return c_pred + (1.0 - h) * launch
    if candidate.form == "partition_low_ref_same_window":
        low_ref = 1.0 - h
        return c_pred + (1.0 - h) * (launch - low_ref * c_pred)
    return (1.0 - h) * launch + h * c_pred


def metric_for_subset(frame: pd.DataFrame, pred: np.ndarray) -> dict[str, float | int]:
    baseline = frame["residual_additive_yaw_torque_nm"].to_numpy()
    corrected = corrected_residuals(frame, pred)
    return {
        "count": int(len(frame)),
        "run_count": int(frame["run_id"].nunique()) if len(frame) else 0,
        "baseline_rmse_nm": rmse(baseline),
        "corrected_rmse_nm": rmse(corrected),
        "baseline_mae_nm": mae(baseline),
        "corrected_mae_nm": mae(corrected),
        "baseline_median_abs_nm": float(np.median(np.abs(baseline))) if len(frame) else math.nan,
        "corrected_median_abs_nm": float(np.median(np.abs(corrected))) if len(frame) else math.nan,
        "baseline_signed_median_nm": float(np.median(baseline)) if len(frame) else math.nan,
        "corrected_signed_median_nm": float(np.median(corrected)) if len(frame) else math.nan,
        "rmse_improvement_pct": 100.0 * (rmse(baseline) - rmse(corrected)) / rmse(baseline)
        if len(frame) and rmse(baseline) > 0.0
        else math.nan,
    }


def torque_from_command(command: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]
    applied_voltage = command * battery
    current = (applied_voltage / resistance) - (
        (wheel_speed_radps * (gear_ratio / speed_constant)) / resistance
    )
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


def signed_direction(preferred: float, fallback: float) -> float:
    preferred_sign = sign(preferred)
    return preferred_sign if preferred_sign != 0.0 else sign(fallback)


def command_torque_for_applied(
    applied_torque: float, wheel_speed_radps: float, constants: dict[str, float]
) -> tuple[float, float]:
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


def motor_commands_for_opposing_torque(
    opposing_yaw_torque: float, constants: dict[str, float], vf_mps: float, yaw_rate: float
) -> dict[str, float]:
    radius = constants["wheel_radius_m"]
    track = constants["track_width_m"]
    half_track = 0.5 * track
    left_surface = vf_mps + half_track * yaw_rate
    right_surface = vf_mps - half_track * yaw_rate
    left_speed = left_surface / radius
    right_speed = right_surface / radius
    applied_bank_torque = opposing_yaw_torque * radius / track
    left_command_torque, left_launch = command_torque_for_applied(applied_bank_torque, left_speed, constants)
    right_command_torque, right_launch = command_torque_for_applied(
        -applied_bank_torque, right_speed, constants
    )
    left_command = command_from_torque(left_command_torque, left_speed, constants)
    right_command = command_from_torque(right_command_torque, right_speed, constants)
    return {
        "left_command": left_command,
        "right_command": right_command,
        "lr_delta_command": left_command - right_command,
        "left_surface_mps": left_surface,
        "right_surface_mps": right_surface,
        "left_wheel_speed_radps": left_speed,
        "right_wheel_speed_radps": right_speed,
        "left_launch_torque_nm": left_launch,
        "right_launch_torque_nm": right_launch,
    }


def in_place_command_for_extra(extra_nm: float, constants: dict[str, float]) -> dict[str, float]:
    base = baseline_opposing_yaw_torque(constants, 1.0)
    total = base + extra_nm
    command = motor_commands_for_opposing_torque(total, constants, vf_mps=0.0, yaw_rate=1.0)
    command.update(
        {
            "baseline_opposing_yaw_torque_nm": base,
            "extra_opposing_yaw_torque_nm": extra_nm,
            "total_opposing_yaw_torque_nm": total,
        }
    )
    return command


def in_place_extra_for_k(
    k_launch_nm: float, util_k: float, activation_kind: str, constants: dict[str, float], metadata: dict[str, float]
) -> float:
    base = baseline_opposing_yaw_torque(constants, 1.0)
    extra = 0.0
    nominal_yield = metadata["nominal_longitudinal_yield_nm"]
    for _ in range(80):
        source = min(max(0.0, base + extra), nominal_yield)
        util = source / max(nominal_yield, 1.0e-12)
        activation = float(smoothstep(activation_kind, util / max(util_k, 1.0e-6)))
        next_extra = k_launch_nm * activation
        if abs(next_extra - extra) < 1.0e-12:
            return next_extra
        extra = next_extra
    return extra


def k_gate_min(util_k: float, activation_kind: str, constants: dict[str, float], metadata: dict[str, float]) -> float:
    lo = 0.0
    hi = 0.20
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        extra = in_place_extra_for_k(mid, util_k, activation_kind, constants, metadata)
        command = in_place_command_for_extra(extra, constants)
        if max(abs(command["left_command"]), abs(command["right_command"])) >= 0.600000:
            hi = mid
        else:
            lo = mid
    return hi


def fit_transition_candidates(
    frame: pd.DataFrame,
    c_pred: np.ndarray,
    constants: dict[str, float],
    metadata: dict[str, float],
) -> list[Candidate]:
    weights = training_weights(frame)
    target = frame["residual_opposes_yaw_nm"].to_numpy()
    split = frame["dataset_split"].to_numpy()
    primary_mask = split == "primary_open_floor_fit_authoritative"
    validation_non_auth_mask = split != "primary_open_floor_fit_authoritative"
    open_floor_validation_mask = split == "open_floor_validation_only"
    high_transition_mask = (
        np.sqrt(np.square(frame["vbar_rel_mps"].to_numpy()) + np.square(frame["abs_forward_velocity_mps"].to_numpy()))
        >= 0.50
    )
    low_speed_yaw_mask = (frame["abs_forward_velocity_mps"].to_numpy() < 0.05) & (
        frame["abs_yaw_rate_radps"].to_numpy() >= 0.2
    )
    straightish_mask = frame["abs_yaw_rate_radps"].to_numpy() < 0.05

    candidates: list[Candidate] = []
    forms = ["direct_blend", "partition_low_ref_same_window", "additive_no_subtract"]
    smoothsteps = ["cubic", "quintic"]
    activation_kinds = ["cubic", "quintic"]
    transition_variables = ["speed_hypot", "vbar_rel_only"]
    rel_weights = [0.75, 1.0]
    util_ks = [0.20, 0.30, 0.40, 0.55]
    windows = [
        (0.00, 0.08),
        (0.00, 0.12),
        (0.02, 0.12),
        (0.02, 0.18),
        (0.04, 0.18),
        (0.04, 0.30),
        (0.08, 0.30),
        (0.08, 0.50),
        (0.12, 0.70),
    ]

    for transition_variable in transition_variables:
        for rel_weight in rel_weights:
            speed = transition_speed(frame, rel_weight, transition_variable)
            for v0, v1 in windows:
                for smooth_kind in smoothsteps:
                    h = smoothstep(smooth_kind, (speed - v0) / (v1 - v0))
                    for activation_kind in activation_kinds:
                        for util_k in util_ks:
                            launch_shape = launch_basis(frame, util_k, activation_kind)
                            gate_k = k_gate_min(util_k, activation_kind, constants, metadata)
                            for form in forms:
                                if form == "additive_no_subtract":
                                    a = (1.0 - h) * launch_shape
                                    b = c_pred.copy()
                                elif form == "partition_low_ref_same_window":
                                    a = (1.0 - h) * launch_shape
                                    low_ref = 1.0 - h
                                    b = c_pred * (1.0 - (1.0 - h) * low_ref)
                                else:
                                    a = (1.0 - h) * launch_shape
                                    b = h * c_pred

                                positive = weights > 0.0
                                denom = float(np.sum(weights[positive] * a[positive] * a[positive]))
                                if denom <= 1.0e-18:
                                    continue
                                k_fit = float(
                                    np.sum(weights[positive] * a[positive] * (target[positive] - b[positive]))
                                    / denom
                                )
                                k_fit = max(0.0, k_fit)
                                k_launch = max(k_fit, gate_k)
                                pred = a * k_launch + b

                                primary_rmse = rmse(corrected_residuals(frame.loc[primary_mask], pred[primary_mask]))
                                validation_rmse = rmse(
                                    corrected_residuals(
                                        frame.loc[validation_non_auth_mask], pred[validation_non_auth_mask]
                                    )
                                )
                                open_val_rmse = rmse(
                                    corrected_residuals(
                                        frame.loc[open_floor_validation_mask],
                                        pred[open_floor_validation_mask],
                                    )
                                )
                                high_rmse = rmse(
                                    corrected_residuals(
                                        frame.loc[high_transition_mask], pred[high_transition_mask]
                                    )
                                )
                                low_rmse = rmse(
                                    corrected_residuals(
                                        frame.loc[low_speed_yaw_mask], pred[low_speed_yaw_mask]
                                    )
                                )
                                straight_rmse = rmse(
                                    corrected_residuals(
                                        frame.loc[straightish_mask], pred[straightish_mask]
                                    )
                                )
                                extra = in_place_extra_for_k(
                                    k_launch, util_k, activation_kind, constants, metadata
                                )
                                command = in_place_command_for_extra(extra, constants)
                                gate_margin = max(
                                    0.0, 0.6 - max(abs(command["left_command"]), abs(command["right_command"]))
                                )
                                score = (
                                    validation_rmse
                                    + 0.30 * primary_rmse
                                    + 0.20 * high_rmse
                                    + 4.0 * gate_margin
                                )
                                candidates.append(
                                    Candidate(
                                        role="grid",
                                        form=form,
                                        smoothstep=smooth_kind,
                                        transition_variable=transition_variable,
                                        v0_mps=v0,
                                        v1_mps=v1,
                                        rel_weight=rel_weight,
                                        util_k=util_k,
                                        launch_activation=activation_kind,
                                        k_launch_nm=k_launch,
                                        k_unconstrained_nm=k_fit,
                                        k_gate_min_nm=gate_k,
                                        primary_rmse_nm=primary_rmse,
                                        validation_non_authoritative_rmse_nm=validation_rmse,
                                        open_floor_validation_rmse_nm=open_val_rmse,
                                        high_transition_rmse_nm=high_rmse,
                                        low_speed_yaw_rmse_nm=low_rmse,
                                        straightish_rmse_nm=straight_rmse,
                                        in_place_left_command=command["left_command"],
                                        in_place_right_command=command["right_command"],
                                        in_place_extra_opposing_nm=extra,
                                        score=score,
                                    )
                                )
    return candidates


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str] | None = None) -> None:
    if fieldnames is None:
        fieldnames = []
        for row in rows:
            for key in row:
                if key not in fieldnames:
                    fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def candidate_to_row(candidate: Candidate, role: str | None = None) -> dict[str, object]:
    return {
        "role": role or candidate.role,
        "form": candidate.form,
        "smoothstep": candidate.smoothstep,
        "transition_variable": candidate.transition_variable,
        "v0_mps": candidate.v0_mps,
        "v1_mps": candidate.v1_mps,
        "rel_weight": candidate.rel_weight,
        "util_k": candidate.util_k,
        "launch_activation": candidate.launch_activation,
        "k_launch_nm": candidate.k_launch_nm,
        "k_unconstrained_nm": candidate.k_unconstrained_nm,
        "k_gate_min_nm": candidate.k_gate_min_nm,
        "primary_rmse_nm": candidate.primary_rmse_nm,
        "validation_non_authoritative_rmse_nm": candidate.validation_non_authoritative_rmse_nm,
        "open_floor_validation_rmse_nm": candidate.open_floor_validation_rmse_nm,
        "high_transition_rmse_nm": candidate.high_transition_rmse_nm,
        "low_speed_yaw_rmse_nm": candidate.low_speed_yaw_rmse_nm,
        "straightish_rmse_nm": candidate.straightish_rmse_nm,
        "in_place_left_command": candidate.in_place_left_command,
        "in_place_right_command": candidate.in_place_right_command,
        "in_place_extra_opposing_nm": candidate.in_place_extra_opposing_nm,
        "score": candidate.score,
    }


def format_float(value: object, digits: int = 6) -> str:
    try:
        x = float(value)
        if not math.isfinite(x):
            return ""
        return f"{x:.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def markdown_table(
    rows: list[dict[str, object]],
    columns: list[str],
    labels: list[str] | None = None,
    limit: int | None = None,
) -> list[str]:
    if labels is None:
        labels = columns
    selected = rows[:limit] if limit is not None else rows
    lines = ["| " + " | ".join(labels) + " |", "| " + " | ".join("---" for _ in labels) + " |"]
    for row in selected:
        values = []
        for column in columns:
            value = row.get(column, "")
            values.append(format_float(value) if isinstance(value, float) else str(value))
        lines.append("| " + " | ".join(values) + " |")
    return lines


def reference_split_rows() -> tuple[dict[str, dict[str, str]], dict[str, dict[str, str]]]:
    b = read_keyed_csv(B_REFERENCE_DIR / "split_rmse.csv", "dataset_split")
    c = read_keyed_csv(C_REFERENCE_DIR / "split_metrics.csv", "group")
    return b, c


def write_outputs(
    frame: pd.DataFrame,
    constants: dict[str, float],
    metadata: dict[str, float],
    c_beta: np.ndarray,
    c_scales: list[float],
    c_pred: np.ndarray,
    candidates: list[Candidate],
    selected: Candidate,
) -> None:
    selected_pred = candidate_prediction(frame, c_pred, selected)
    c_corrected = corrected_residuals(frame, c_pred)

    top_non_additive = [
        candidate
        for candidate in sorted(candidates, key=lambda c: c.score)
        if candidate.form != "additive_no_subtract"
    ][:40]
    top_additive = [
        candidate
        for candidate in sorted(candidates, key=lambda c: c.score)
        if candidate.form == "additive_no_subtract"
    ][:20]
    selected_rows = [candidate_to_row(selected, "selected")]
    selected_rows.extend(candidate_to_row(candidate, "non_additive_grid_top") for candidate in top_non_additive)
    selected_rows.extend(candidate_to_row(candidate, "additive_double_count_diagnostic") for candidate in top_additive)
    write_csv(OUT / "candidate_scores.csv", selected_rows)

    write_csv(
        OUT / "moving_contact_coefficients.csv",
        [
            {
                "feature": feature,
                "standardized_coefficient_nm": float(beta),
                "feature_scale": scale,
                "raw_coefficient_nm_per_feature": float(beta / scale),
            }
            for feature, beta, scale in zip(C_FEATURES, c_beta, c_scales)
        ],
    )
    write_csv(
        OUT / "selected_parameters.csv",
        [
            {"parameter": "form", "value": selected.form, "unit": "enum"},
            {"parameter": "smoothstep", "value": selected.smoothstep, "unit": "enum"},
            {"parameter": "transition_variable", "value": selected.transition_variable, "unit": "enum"},
            {"parameter": "v0_mps", "value": selected.v0_mps, "unit": "m/s"},
            {"parameter": "v1_mps", "value": selected.v1_mps, "unit": "m/s"},
            {"parameter": "rel_weight", "value": selected.rel_weight, "unit": "dimensionless"},
            {"parameter": "util_k", "value": selected.util_k, "unit": "yield fraction"},
            {"parameter": "launch_activation", "value": selected.launch_activation, "unit": "enum"},
            {"parameter": "k_launch_nm", "value": selected.k_launch_nm, "unit": "Nm"},
            {"parameter": "k_unconstrained_nm", "value": selected.k_unconstrained_nm, "unit": "Nm"},
            {"parameter": "k_gate_min_nm", "value": selected.k_gate_min_nm, "unit": "Nm"},
            {
                "parameter": "nominal_longitudinal_yield_nm",
                "value": metadata["nominal_longitudinal_yield_nm"],
                "unit": "Nm",
            },
        ],
        ["parameter", "value", "unit"],
    )

    split_rows: list[dict[str, object]] = []
    b_ref, c_ref = reference_split_rows()
    for split_name, subset in frame.groupby("dataset_split", sort=True):
        indexer = frame.index.get_indexer(subset.index)
        row = {"dataset_split": split_name}
        row.update(metric_for_subset(subset, selected_pred[indexer]))
        row["compact_c_force_only_corrected_rmse_nm"] = rmse(c_corrected[indexer])
        b_row = b_ref.get(split_name, {})
        c_row = c_ref.get(split_name, {})
        row["force_domain_stribeck_rmse_nm"] = (
            float(b_row["corrected_rmse_nm"]) if b_row else math.nan
        )
        row["published_variant_c_rmse_nm"] = (
            float(c_row["corrected_rmse_nm"]) if c_row else math.nan
        )
        split_rows.append(row)
    validation_subset = frame[frame["dataset_split"] != "primary_open_floor_fit_authoritative"]
    validation_idx = frame.index.get_indexer(validation_subset.index)
    validation_row = {"dataset_split": "validation_non_authoritative"}
    validation_row.update(metric_for_subset(validation_subset, selected_pred[validation_idx]))
    validation_row["compact_c_force_only_corrected_rmse_nm"] = rmse(c_corrected[validation_idx])
    validation_row["force_domain_stribeck_rmse_nm"] = math.nan
    validation_row["published_variant_c_rmse_nm"] = 0.030342
    split_rows.append(validation_row)
    write_csv(OUT / "split_metrics.csv", split_rows)

    phase_rows = []
    for phase, subset in frame.groupby("physics_phase", sort=True):
        indexer = frame.index.get_indexer(subset.index)
        row = {"physics_phase": phase}
        row.update(metric_for_subset(subset, selected_pred[indexer]))
        phase_rows.append(row)
    write_csv(OUT / "phase_metrics.csv", phase_rows)

    log_rows = []
    for run_id in SELECTED_RUNS:
        subset = frame[frame["run_id"] == run_id]
        row: dict[str, object] = {"run_id": run_id, "present": bool(len(subset))}
        if len(subset):
            indexer = frame.index.get_indexer(subset.index)
            row["dataset_split"] = ";".join(sorted(subset["dataset_split"].unique()))
            row["family"] = ";".join(sorted(subset["family"].unique()))
            row.update(metric_for_subset(subset, selected_pred[indexer]))
            row["compact_c_force_only_corrected_rmse_nm"] = rmse(c_corrected[indexer])
        log_rows.append(row)
    write_csv(OUT / "selected_log_metrics.csv", log_rows)

    risk_defs = [
        ("straightish_abs_yaw_lt_0p05", frame["abs_yaw_rate_radps"].to_numpy() < 0.05),
        (
            "straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05",
            (frame["abs_yaw_rate_radps"].to_numpy() < 0.05)
            & (frame["abs_forward_velocity_mps"].to_numpy() >= 0.05),
        ),
        (
            "low_speed_yaw_vf_lt_0p05_yaw_ge_0p2",
            (frame["abs_forward_velocity_mps"].to_numpy() < 0.05)
            & (frame["abs_yaw_rate_radps"].to_numpy() >= 0.2),
        ),
        (
            "high_transition_speed_ge_0p5",
            (
                np.sqrt(
                    np.square(frame["vbar_rel_mps"].to_numpy())
                    + np.square(frame["abs_forward_velocity_mps"].to_numpy())
                )
                >= 0.5
            ),
        ),
        ("limiter_active", frame["max_force_limiter_activity"].to_numpy() > 0.0),
    ]
    risk_rows = []
    for label, mask in risk_defs:
        subset = frame.loc[mask]
        indexer = frame.index.get_indexer(subset.index)
        row = {"group": label}
        row.update(metric_for_subset(subset, selected_pred[indexer]))
        row["compact_c_force_only_corrected_rmse_nm"] = rmse(c_corrected[indexer])
        risk_rows.append(row)
    write_csv(OUT / "risk_metrics.csv", risk_rows)

    extra = in_place_extra_for_k(
        selected.k_launch_nm, selected.util_k, selected.launch_activation, constants, metadata
    )
    command = in_place_command_for_extra(extra, constants)
    write_csv(
        OUT / "in_place_1radps_command.csv",
        [
            {
                "variant": "selected_smoothstep_partition",
                "extra_opposing_yaw_torque_nm": extra,
                "total_opposing_yaw_torque_nm": command["total_opposing_yaw_torque_nm"],
                "left_command": command["left_command"],
                "right_command": command["right_command"],
                "lr_delta_command": command["lr_delta_command"],
                "max_abs_command": max(abs(command["left_command"]), abs(command["right_command"])),
                "passes_abs_0p6_gate": max(abs(command["left_command"]), abs(command["right_command"]))
                >= 0.6,
            }
        ],
    )

    grid_rows = []
    for vf_mps in [round(i * 0.15 / 5.0, 9) for i in range(6)]:
        for yaw_rate in [round(0.2 + i * (6.0 - 0.2) / 9.0, 9) for i in range(10)]:
            if selected.transition_variable == "vbar_rel_only":
                v_transition = selected.rel_weight * constants["drive_wheel_longitudinal_offset_m"] * abs(yaw_rate)
            else:
                v_transition = math.hypot(
                    selected.rel_weight * constants["drive_wheel_longitudinal_offset_m"] * abs(yaw_rate),
                    abs(vf_mps),
                )
            h = float(
                smoothstep(
                    selected.smoothstep,
                    (v_transition - selected.v0_mps) / (selected.v1_mps - selected.v0_mps),
                )
            )
            extra_grid = (1.0 - h) * in_place_extra_for_k(
                selected.k_launch_nm, selected.util_k, selected.launch_activation, constants, metadata
            )
            command_grid = in_place_command_for_extra(extra_grid, constants)
            grid_rows.append(
                {
                    "vf_mps": vf_mps,
                    "yaw_rate_radps": yaw_rate,
                    "transition_speed_mps": v_transition,
                    "h": h,
                    "launch_extra_only_opposing_nm": extra_grid,
                    "left_command": command_grid["left_command"],
                    "right_command": command_grid["right_command"],
                    "lr_delta_command": command_grid["lr_delta_command"],
                    "note": "grid command omits C moving branch; shows launch partition fade only",
                }
            )
    write_csv(OUT / "launch_fade_grid.csv", grid_rows)

    sample_cols = [
        "run_id",
        "row_index",
        "dataset_split",
        "physics_phase",
        "forward_velocity_mps",
        "yaw_rate_radps",
        "vbar_rel_mps",
        "residual_additive_yaw_torque_nm",
    ]
    sample = frame.loc[:, sample_cols].copy()
    sample["predicted_opposes_nm"] = selected_pred
    sample["predicted_additive_nm"] = -frame["yaw_sign"].to_numpy() * selected_pred
    sample["corrected_residual_nm"] = corrected_residuals(frame, selected_pred)
    sample.iloc[:: max(1, len(sample) // 1000)].to_csv(OUT / "prediction_sample.csv", index=False)

    metadata_payload = {
        "inputs": {
            "primary": str(PRIMARY_INPUT.relative_to(ROOT)),
            "secondary": str(SECONDARY_INPUT.relative_to(ROOT)),
            "constants": str(CONSTANTS_INPUT.relative_to(ROOT)),
        },
        "metadata": metadata,
        "production_code_edited": False,
        "build_metadata_edited": False,
        "tests_edited": False,
        "selected": candidate_to_row(selected, "selected"),
    }
    (OUT / "metadata.json").write_text(json.dumps(metadata_payload, indent=2) + "\n", encoding="utf-8")
    (OUT / "commands_run.txt").write_text(
        "python codex_analysis\\yaw_model_variant_fits\\transition_options\\cubic_smoothstep_partition\\fit_cubic_smoothstep_partition.py\n",
        encoding="utf-8",
    )
    write_report(metadata, selected)


def write_report(metadata: dict[str, float], selected: Candidate) -> None:
    params = list(csv.DictReader((OUT / "selected_parameters.csv").open(newline="", encoding="utf-8")))
    split_rows = list(csv.DictReader((OUT / "split_metrics.csv").open(newline="", encoding="utf-8")))
    risk_rows = list(csv.DictReader((OUT / "risk_metrics.csv").open(newline="", encoding="utf-8")))
    in_place = list(csv.DictReader((OUT / "in_place_1radps_command.csv").open(newline="", encoding="utf-8")))
    candidates = list(csv.DictReader((OUT / "candidate_scores.csv").open(newline="", encoding="utf-8")))

    lines: list[str] = [
        "# Cubic/Quintic Smoothstep Launch-to-C Partition",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not edited.",
        "",
        "## Recommendation",
        "",
        f"Use the `{selected.form}` shape with `{selected.smoothstep}` speed smoothstep as the best candidate from this pass. It enforces the `|cmd| >= 0.6` in-place gate, fades the launch authority to zero at high transition speed, and avoids command/request traction inputs in the selected equation.",
        "",
        "The selected moving branch is a compact force-only Variant-C-style fit over projected contact forces and contact-relative velocities. The published Variant C artifact is retained as a comparison reference, but it is not the selected runtime shape here because its strongest prior coefficients include request-derived terms.",
        "",
        "## Selected Equations",
        "",
        "`v_t = sqrt((rel_weight * vbar_rel)^2 + |Vf|^2)`",
        "",
        "`t = clamp((v_t - v0) / (v1 - v0), 0, 1)`",
        "",
        "Cubic: `h = t*t*(3 - 2*t)`; quintic option: `h = t^3*(10 - 15*t + 6*t^2)`.",
        "",
        "`u = smooth_positive(M_projected_force_opposes_yaw) / M_yield`",
        "",
        "`a = smoothstep(clamp(u / util_k, 0, 1))`",
        "",
        "`M_launch = K_launch * a`",
        "",
        "`M_pred = M_C_force + (1 - h) * (M_launch - (1 - h) * M_C_force)`",
        "",
        "That partition subtracts the low-speed reference portion of the C branch instead of adding launch on top of all of C. At `h=0`, it reduces to launch authority; at `h=1`, it is exactly the moving-contact C branch.",
        "",
        "## Selected Parameters",
        "",
    ]
    lines.extend(markdown_table(params, ["parameter", "value", "unit"]))
    lines.extend(
        [
            "",
            "## Split Metrics",
            "",
        ]
    )
    lines.extend(
        markdown_table(
            split_rows,
            [
                "dataset_split",
                "count",
                "baseline_rmse_nm",
                "corrected_rmse_nm",
                "compact_c_force_only_corrected_rmse_nm",
                "force_domain_stribeck_rmse_nm",
                "published_variant_c_rmse_nm",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## In-Place Gate",
            "",
        ]
    )
    lines.extend(
        markdown_table(
            in_place,
            [
                "variant",
                "extra_opposing_yaw_torque_nm",
                "total_opposing_yaw_torque_nm",
                "left_command",
                "right_command",
                "lr_delta_command",
                "max_abs_command",
                "passes_abs_0p6_gate",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Risk Slices",
            "",
        ]
    )
    lines.extend(
        markdown_table(
            risk_rows,
            [
                "group",
                "count",
                "baseline_rmse_nm",
                "corrected_rmse_nm",
                "compact_c_force_only_corrected_rmse_nm",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Top Candidate Families",
            "",
        ]
    )
    lines.extend(
        markdown_table(
            candidates,
            [
                "role",
                "form",
                "smoothstep",
                "transition_variable",
                "v0_mps",
                "v1_mps",
                "util_k",
                "k_launch_nm",
                "primary_rmse_nm",
                "validation_non_authoritative_rmse_nm",
                "high_transition_rmse_nm",
                "score",
            ],
            limit=12,
        )
    )
    lines.extend(
        [
            "",
            "## Cost",
            "",
            "Selected prediction cost after `M_C_force` terms are available:",
            "",
            "- comparisons/clamps: 4 to 6 (`smooth_positive` sign-free positive part, utilization clamp, transition clamp, optional force/yaw zero guards).",
            "- multiplies: about 17 for cubic speed partition plus cubic utilization activation and partition composition; quintic adds about 4 multiplies.",
            "- divisions: 2 (`u / util_k`, `(v_t - v0)/(v1-v0)`) if reciprocals are not precomputed; 0 runtime divisions for these if `1/util_k` and `1/(v1-v0)` are constants.",
            "- sqrt calls: 2 (`smooth_positive` and transition `sqrt`). If positive-part is replaced by a cheap branch `max(0, x)`, this drops to 1 sqrt.",
            "- trig/exp/tanh: 0 in the selected form.",
            "",
            "The compact moving branch adds 10 coefficients. The launch partition adds four scalar parameters (`v0`, `v1`, `util_k`, `K_launch`) plus the existing `rel_weight` choice.",
            "",
            "## Notes",
            "",
            "- `additive_no_subtract` candidates were kept in the grid to expose double-counting risk; their low-speed behavior adds launch on top of C and was not selected.",
            "- The `launch_fade_grid.csv` command grid reports the launch fade contribution only; full high-speed command behavior must come from the moving-contact branch replay, not from the static launch envelope.",
            "- The command estimator uses the prior analysis motor inverse for comparability. That inverse contains the existing motor static-friction exponential, but the selected yaw correction equation itself has no exp/tanh/trig.",
            "",
            "## Output Files",
            "",
            "- `fit_cubic_smoothstep_partition.py`",
            "- `cubic_smoothstep_partition_report.md`",
            "- `candidate_scores.csv`",
            "- `selected_parameters.csv`",
            "- `moving_contact_coefficients.csv`",
            "- `split_metrics.csv`",
            "- `phase_metrics.csv`",
            "- `selected_log_metrics.csv`",
            "- `risk_metrics.csv`",
            "- `in_place_1radps_command.csv`",
            "- `launch_fade_grid.csv`",
            "- `prediction_sample.csv`",
            "- `metadata.json`",
            "- `commands_run.txt`",
        ]
    )
    (OUT / "cubic_smoothstep_partition_report.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )


def main() -> None:
    constants = read_constants()
    frame, metadata = load_frame(constants)
    c_features, c_scales = build_c_features(frame)
    c_beta = weighted_ridge(
        c_features,
        frame["residual_opposes_yaw_nm"].to_numpy(),
        training_weights(frame),
        ridge=0.001,
    )
    c_pred = c_features @ c_beta
    candidates = fit_transition_candidates(frame, c_pred, constants, metadata)
    gated = [
        candidate
        for candidate in candidates
        if max(abs(candidate.in_place_left_command), abs(candidate.in_place_right_command)) >= 0.6
        and candidate.form != "additive_no_subtract"
    ]
    if not gated:
        raise RuntimeError("no non-additive candidate passed the in-place command gate")
    selected = min(gated, key=lambda candidate: candidate.score)
    write_outputs(frame, constants, metadata, c_beta, c_scales, c_pred, candidates, selected)
    print(f"selected={selected.form} {selected.smoothstep} v0={selected.v0_mps} v1={selected.v1_mps}")
    print(f"validation_non_authoritative_rmse_nm={selected.validation_non_authoritative_rmse_nm:.9f}")
    print(f"in_place_left_right={selected.in_place_left_command:.9f}/{selected.in_place_right_command:.9f}")


if __name__ == "__main__":
    main()
