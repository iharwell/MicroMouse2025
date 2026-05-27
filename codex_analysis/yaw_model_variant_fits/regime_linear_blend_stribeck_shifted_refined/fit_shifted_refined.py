#!/usr/bin/env python3
"""Refined shifted linear-blend-to-Stribeck yaw scrub analysis.

Analysis-only tooling. It writes outputs only in this directory.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent

PRIMARY_INPUT = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "ablation"
    / "phase_classified_feature_sample.csv"
)
CONSTANTS_INPUT = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "features"
    / "plant_mirror_constants.csv"
)

FORCE_DOMAIN_SPLIT = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "round2_force_domain_stribeck"
    / "split_rmse.csv"
)
FORCE_DOMAIN_SELECTED = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "round2_force_domain_stribeck"
    / "selected_log_rmse.csv"
)
RATIONAL_SPLIT = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "transition_options"
    / "rational_speed_force_blend"
    / "split_metrics.csv"
)
RATIONAL_SELECTED = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "transition_options"
    / "rational_speed_force_blend"
    / "selected_log_metrics.csv"
)
STANDALONE_SPLIT = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "standalone_contact_traction_testbed"
    / "split_metrics.csv"
)
STANDALONE_SELECTED = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "standalone_contact_traction_testbed"
    / "selected_log_metrics.csv"
)
TRUE_TRACTION_SPLIT = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "contact_patch_true_traction_testbed"
    / "baseline_comparison.csv"
)

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
LATEST_RUNS = {"2026-05-04_20-35-47", "2026-05-04_16-57-53"}

USECOLS = [
    "run_id",
    "family",
    "recommendation",
    "dataset_split",
    "row_index",
    "time_us",
    "physics_phase",
    "forward_velocity_mps",
    "yaw_rate_radps",
    "max_force_limiter_activity",
    "hardware_saturation_evidence",
    "gyro_derivative_spike",
    "residual_additive_yaw_torque_nm",
    "residual_opposes_yaw_nm",
]


@dataclass(frozen=True)
class Params:
    interpretation: str
    k1: float
    k2: float
    k3: float
    peak_frac: float
    decay0: float
    decay_v: float
    vf_fade: float


@dataclass
class Candidate:
    branch: str
    params: Params
    slide_nm: float
    peak_delta_nm: float
    train_rmse_nm: float
    validation_non_authoritative_rmse_nm: float
    primary_rmse_nm: float
    open_floor_validation_rmse_nm: float
    latest_may4_rmse_nm: float
    launch_extra_nm: float
    launch_total_nm: float
    launch_left_command: float
    launch_right_command: float
    launch_max_abs_command: float
    launch_target_abs_error: float
    peak_extra_nm: float
    selected_score: float
    source: str
    boundary_status: str = ""


def read_constants() -> dict[str, float]:
    table = pd.read_csv(CONSTANTS_INPUT)
    return {str(row.name): float(row.value) for row in table.itertuples(index=False)}


def sign_array(values: np.ndarray, eps: float = 1.0e-9) -> np.ndarray:
    return (values > eps).astype(float) - (values < -eps).astype(float)


def sign(value: float, eps: float = 1.0e-9) -> float:
    return float((value > eps) - (value < -eps))


def load_frame() -> pd.DataFrame:
    frame = pd.read_csv(PRIMARY_INPUT, usecols=USECOLS)
    numeric_columns = [
        "row_index",
        "time_us",
        "forward_velocity_mps",
        "yaw_rate_radps",
        "max_force_limiter_activity",
        "hardware_saturation_evidence",
        "gyro_derivative_spike",
        "residual_additive_yaw_torque_nm",
        "residual_opposes_yaw_nm",
    ]
    for column in numeric_columns:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(
        subset=[
            "forward_velocity_mps",
            "yaw_rate_radps",
            "residual_additive_yaw_torque_nm",
            "residual_opposes_yaw_nm",
        ]
    )
    frame["abs_forward_velocity_mps"] = frame["forward_velocity_mps"].abs()
    frame["abs_yaw_rate_radps"] = frame["yaw_rate_radps"].abs()
    frame["yaw_sign"] = sign_array(frame["yaw_rate_radps"].to_numpy())
    frame["yaw_sign"] = frame["yaw_sign"].replace(0.0, 1.0)
    return frame


def base_training_weights(frame: pd.DataFrame) -> np.ndarray:
    base = np.zeros(len(frame), dtype=float)
    split = frame["dataset_split"].to_numpy()
    recommendation = frame["recommendation"].to_numpy()
    family = frame["family"].to_numpy()
    base[split == "primary_open_floor_fit_authoritative"] = 1.0
    downweighted_fit = (
        (recommendation == "fit_downweighted")
        & (family == "open_floor")
        & (split == "open_floor_fit_downweighted")
    )
    base[downweighted_fit] = 0.25
    limiter = np.clip(frame["max_force_limiter_activity"].fillna(0.0).to_numpy(), 0.0, 1.0)
    saturation = np.clip(frame["hardware_saturation_evidence"].fillna(0.0).to_numpy(), 0.0, 1.0)
    spike = np.clip(frame["gyro_derivative_spike"].fillna(0.0).to_numpy(), 0.0, 1.0)
    quality = (1.0 / (1.0 + 4.0 * limiter)) * (1.0 - 0.75 * saturation) * (1.0 - 0.75 * spike)
    quality = np.clip(quality, 0.02, 1.0)
    weights = base * quality
    fit_counts = frame.loc[weights > 0.0, "run_id"].value_counts()
    if not fit_counts.empty:
        run_scale = frame["run_id"].map({run: 1.0 / math.sqrt(count) for run, count in fit_counts.items()}).fillna(0.0)
        weights *= run_scale.to_numpy()
        positive = weights > 0.0
        weights[positive] *= positive.sum() / weights[positive].sum()
    return weights


def latest_weighted_training_weights(frame: pd.DataFrame, latest_fraction: float = 0.30) -> np.ndarray:
    weights = base_training_weights(frame)
    latest_mask = frame["run_id"].isin(LATEST_RUNS).to_numpy()
    positive_latest = latest_mask & (frame["dataset_split"].isin(["open_floor_fit_downweighted", "open_floor_validation_only"]).to_numpy())
    latest_quality = np.zeros(len(frame), dtype=float)
    latest_quality[positive_latest] = 1.0
    fit_counts = frame.loc[positive_latest, "run_id"].value_counts()
    if not fit_counts.empty:
        run_scale = frame["run_id"].map({run: 1.0 / math.sqrt(count) for run, count in fit_counts.items()}).fillna(0.0)
        latest_quality *= run_scale.to_numpy()
    if latest_quality.sum() <= 0.0 or weights.sum() <= 0.0:
        return weights
    base_scaled = weights * (1.0 - latest_fraction) / weights.sum()
    latest_scaled = latest_quality * latest_fraction / latest_quality.sum()
    combined = base_scaled + latest_scaled
    positive = combined > 0.0
    combined[positive] *= positive.sum() / combined[positive].sum()
    return combined


def search_sample(frame: pd.DataFrame, weights: np.ndarray, seed: int, max_rows: int = 14000) -> tuple[pd.DataFrame, np.ndarray]:
    positive = weights > 0.0
    mandatory = positive | frame["run_id"].isin(LATEST_RUNS).to_numpy()
    indices = np.flatnonzero(mandatory)
    if len(indices) <= max_rows:
        sample = frame.iloc[indices].reset_index(drop=True)
        sample_weights = weights[indices].copy()
    else:
        rng = np.random.default_rng(seed)
        probabilities = weights[indices].copy()
        probabilities = np.where(probabilities > 0.0, probabilities, 0.02 * np.mean(probabilities[probabilities > 0.0]))
        probabilities = probabilities / probabilities.sum()
        chosen = rng.choice(indices, size=max_rows, replace=False, p=probabilities)
        chosen.sort()
        sample = frame.iloc[chosen].reset_index(drop=True)
        sample_weights = weights[chosen].copy()
    latest = sample["run_id"].isin(LATEST_RUNS).to_numpy()
    if np.any(latest) and np.all(sample_weights[latest] <= 0.0):
        sample_weights[latest] = 0.05
    positive_sample = sample_weights > 0.0
    if np.any(positive_sample):
        sample_weights[positive_sample] *= positive_sample.sum() / sample_weights[positive_sample].sum()
    return sample, sample_weights


def two_column_nnls(features: np.ndarray, target: np.ndarray, weights: np.ndarray) -> tuple[float, float]:
    positive = weights > 0.0
    x = features[positive]
    y = target[positive]
    w = weights[positive]
    x0 = x[:, 0]
    x1 = x[:, 1]
    a00 = float(np.dot(w * x0, x0))
    a01 = float(np.dot(w * x0, x1))
    a11 = float(np.dot(w * x1, x1))
    b0 = float(np.dot(w * x0, y))
    b1 = float(np.dot(w * x1, y))
    candidates: list[tuple[float, float]] = [(0.0, 0.0)]
    det = a00 * a11 - a01 * a01
    if det > 1.0e-18:
        c0 = (b0 * a11 - b1 * a01) / det
        c1 = (a00 * b1 - a01 * b0) / det
        if c0 >= 0.0 and c1 >= 0.0:
            candidates.append((c0, c1))
    if a00 > 1.0e-18:
        candidates.append((max(0.0, b0 / a00), 0.0))
    if a11 > 1.0e-18:
        candidates.append((0.0, max(0.0, b1 / a11)))
    best = min(candidates, key=lambda c: float(np.average(np.square(y - c[0] * x0 - c[1] * x1), weights=w)))
    return best


def shifted_shape(abs_yaw: np.ndarray, abs_vf: np.ndarray, params: Params) -> np.ndarray:
    y = abs_yaw
    v = abs_vf
    x1 = v * params.k1
    x2 = v * params.k2 + params.k3
    x2 = np.maximum(x2, 1.0e-9)
    x1 = np.minimum(x1, x2 - 1.0e-9)
    xp = x1 + params.peak_frac * (x2 - x1)
    decay = np.maximum(params.decay0 + params.decay_v * v, 1.0e-6)
    w_peak = np.clip((xp - x1) / np.maximum(x2 - x1, 1.0e-9), 0.0, 1.0)

    if params.interpretation == "x1_shifted_stribeck_peak_slope":
        # Stribeck parameters are fixed; the yaw-rate input is shifted by
        # x1(Vf), so the Stribeck section is already present between x1 and
        # the interior peak.  The line slope varies with Vf through
        # x1/x2/xp and is derived so the blended peak basis equals one at xp.
        fixed_decay = np.maximum(params.decay0, 1.0e-6)
        peak_offset = xp - x1
        shifted_y = y - x1
        raw_slide = np.ones_like(y)
        raw_peak = np.exp(-np.square((shifted_y - peak_offset) / fixed_decay))
        raw_peak_at_xp = np.ones_like(y)
        line_slide = y / x2
        peak_slope = (1.0 - w_peak * raw_peak_at_xp) / np.maximum((1.0 - w_peak) * xp, 1.0e-6)
        peak_slope = np.clip(peak_slope, 0.0, 100.0)
        line_peak = peak_slope * y
        w = np.clip((y - x1) / np.maximum(x2 - x1, 1.0e-9), 0.0, 1.0)
        slide = np.where(y <= x1, line_slide, np.where(y < x2, (1.0 - w) * line_slide + w * raw_slide, raw_slide))
        peak = np.where(y <= x1, line_peak, np.where(y < x2, (1.0 - w) * line_peak + w * raw_peak, raw_peak))
        return np.column_stack([slide, peak])

    fwd = 1.0 / (1.0 + np.square(v / params.vf_fade))
    raw_slide = fwd
    e_y = np.exp(-np.square((y - xp) / decay))
    e_x1 = np.exp(-np.square((x1 - xp) / decay))
    e_x2 = np.exp(-np.square((x2 - xp) / decay))

    if params.interpretation == "line_reaches_x2":
        denom = x2
        line_boundary_peak = e_x2
        peak_normalizer = (1.0 - w_peak) * line_boundary_peak * xp / denom + w_peak
    elif params.interpretation == "line_meets_x1":
        denom = np.maximum(x1, 1.0e-6)
        line_boundary_peak = e_x1
        peak_normalizer = (1.0 - w_peak) * line_boundary_peak * xp / denom + w_peak
    elif params.interpretation == "line_maintains_peak":
        denom = np.maximum(xp, 1.0e-6)
        line_boundary_peak = 1.0
        peak_normalizer = 1.0
    else:
        raise ValueError(params.interpretation)
    peak_normalizer = np.maximum(peak_normalizer, 1.0e-6)

    line_slide = raw_slide * y / denom
    raw_peak = e_y / peak_normalizer
    line_peak = line_boundary_peak * y / denom / peak_normalizer
    w = np.clip((y - x1) / np.maximum(x2 - x1, 1.0e-9), 0.0, 1.0)
    slide = np.where(y <= x1, line_slide, np.where(y < x2, (1.0 - w) * line_slide + w * raw_slide, raw_slide))
    peak = np.where(y <= x1, line_peak, np.where(y < x2, (1.0 - w) * line_peak + w * raw_peak, raw_peak))
    return np.column_stack([slide, peak])


def predict_opposes(frame: pd.DataFrame, candidate: Candidate) -> np.ndarray:
    features = shifted_shape(
        frame["abs_yaw_rate_radps"].to_numpy(),
        frame["abs_forward_velocity_mps"].to_numpy(),
        candidate.params,
    )
    return features @ np.array([candidate.slide_nm, candidate.peak_delta_nm])


def corrected_residuals(frame: pd.DataFrame, pred_opposes: np.ndarray) -> np.ndarray:
    pred_additive = -frame["yaw_sign"].to_numpy() * pred_opposes
    return frame["residual_additive_yaw_torque_nm"].to_numpy() - pred_additive


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else math.nan


def weighted_rmse(errors: np.ndarray, weights: np.ndarray) -> float:
    positive = weights > 0.0
    if not np.any(positive):
        return math.nan
    return float(np.sqrt(np.average(np.square(errors[positive]), weights=weights[positive])))


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
    return left_surface_mps, right_surface_mps, left_surface_mps / radius, right_surface_mps / radius


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


def motor_commands_for_opposing_torque(opposing_yaw_torque: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> dict[str, float]:
    radius = constants["wheel_radius_m"]
    track = constants["track_width_m"]
    left_surface, right_surface, left_speed, right_speed = wheel_speeds(vf_mps, yaw_rate, constants)
    applied_bank_torque = opposing_yaw_torque * radius / track
    left_command_torque, left_launch = command_torque_for_applied(applied_bank_torque, left_speed, constants)
    right_command_torque, right_launch = command_torque_for_applied(-applied_bank_torque, right_speed, constants)
    left_command = command_from_torque(left_command_torque, left_speed, constants)
    right_command = command_from_torque(right_command_torque, right_speed, constants)
    return {
        "applied_bank_torque_nm": applied_bank_torque,
        "left_command_torque_nm": left_command_torque,
        "right_command_torque_nm": right_command_torque,
        "left_command": left_command,
        "right_command": right_command,
        "lr_delta_command": left_command - right_command,
        "max_abs_command": max(abs(left_command), abs(right_command)),
        "left_surface_mps": left_surface,
        "right_surface_mps": right_surface,
        "left_wheel_speed_radps": left_speed,
        "right_wheel_speed_radps": right_speed,
        "left_launch_torque_nm": left_launch,
        "right_launch_torque_nm": right_launch,
    }


def candidate_launch(candidate: Candidate, constants: dict[str, float]) -> dict[str, float]:
    shape = shifted_shape(np.array([1.0]), np.array([0.0]), candidate.params)
    extra = float(shape @ np.array([candidate.slide_nm, candidate.peak_delta_nm]))
    base = baseline_opposing_yaw_torque(constants, 1.0)
    total = base + extra
    command = motor_commands_for_opposing_torque(total, constants, 0.0, 1.0)
    command.update(
        {
            "baseline_opposing_yaw_torque_nm": base,
            "extra_opposing_yaw_torque_nm": extra,
            "total_opposing_yaw_torque_nm": total,
        }
    )
    return command


def build_candidate(
    branch: str,
    params: Params,
    frame: pd.DataFrame,
    weights: np.ndarray,
    constants: dict[str, float],
    source: str,
) -> Candidate:
    target = frame["residual_opposes_yaw_nm"].to_numpy()
    features = shifted_shape(frame["abs_yaw_rate_radps"].to_numpy(), frame["abs_forward_velocity_mps"].to_numpy(), params)
    slide_nm, peak_delta_nm = two_column_nnls(features, target, weights)
    pred = features @ np.array([slide_nm, peak_delta_nm])
    train_rmse = weighted_rmse(target - pred, weights)
    corrected = corrected_residuals(frame, pred)
    split = frame["dataset_split"].to_numpy()
    nonauth = split != "primary_open_floor_fit_authoritative"
    latest = frame["run_id"].isin(LATEST_RUNS).to_numpy()

    shell = Candidate(
        branch=branch,
        params=params,
        slide_nm=slide_nm,
        peak_delta_nm=peak_delta_nm,
        train_rmse_nm=train_rmse,
        validation_non_authoritative_rmse_nm=rmse(corrected[nonauth]),
        primary_rmse_nm=rmse(corrected[split == "primary_open_floor_fit_authoritative"]),
        open_floor_validation_rmse_nm=rmse(corrected[split == "open_floor_validation_only"]),
        latest_may4_rmse_nm=rmse(corrected[latest]),
        launch_extra_nm=0.0,
        launch_total_nm=0.0,
        launch_left_command=0.0,
        launch_right_command=0.0,
        launch_max_abs_command=0.0,
        launch_target_abs_error=0.0,
        peak_extra_nm=slide_nm + peak_delta_nm,
        selected_score=train_rmse,
        source=source,
    )
    launch = candidate_launch(shell, constants)
    launch_error = abs(launch["max_abs_command"] - 0.646)
    gate_penalty = 0.0
    if launch["max_abs_command"] < 0.6:
        gate_penalty += 10.0 * (0.6 - launch["max_abs_command"])
    if launch["max_abs_command"] > 0.78:
        gate_penalty += 2.0 * (launch["max_abs_command"] - 0.78)
    if shell.peak_extra_nm > 0.105:
        gate_penalty += 2.0 * (shell.peak_extra_nm - 0.105)
    if shell.peak_extra_nm < 0.040:
        gate_penalty += 0.5 * (0.040 - shell.peak_extra_nm)
    shell.launch_extra_nm = launch["extra_opposing_yaw_torque_nm"]
    shell.launch_total_nm = launch["total_opposing_yaw_torque_nm"]
    shell.launch_left_command = launch["left_command"]
    shell.launch_right_command = launch["right_command"]
    shell.launch_max_abs_command = launch["max_abs_command"]
    shell.launch_target_abs_error = launch_error
    shell.selected_score = train_rmse + 0.25 * launch_error + gate_penalty
    return shell


def log_uniform(rng: np.random.Generator, lo: float, hi: float, size: int) -> np.ndarray:
    return np.exp(rng.uniform(math.log(lo), math.log(hi), size=size))


def initial_ranges() -> dict[str, tuple[float, float]]:
    return {
        "k1": (0.025, 1.20),
        "k2": (0.25, 8.00),
        "k3": (0.035, 1.60),
        "peak_frac": (0.12, 0.88),
        "decay0": (0.12, 9.00),
        "decay_v": (0.0, 6.00),
        "vf_fade": (0.08, 4.00),
    }


def sample_params(rng: np.random.Generator, ranges: dict[str, tuple[float, float]], count: int) -> list[Params]:
    k1s = log_uniform(rng, *ranges["k1"], size=count)
    k2s = log_uniform(rng, *ranges["k2"], size=count)
    k3s = log_uniform(rng, *ranges["k3"], size=count)
    peak_fracs = rng.uniform(*ranges["peak_frac"], size=count)
    decay0s = log_uniform(rng, *ranges["decay0"], size=count)
    decay_vs = rng.uniform(*ranges["decay_v"], size=count)
    vf_fades = log_uniform(rng, *ranges["vf_fade"], size=count)
    interpretations = rng.choice(
        ["x1_shifted_stribeck_peak_slope", "line_reaches_x2", "line_meets_x1", "line_maintains_peak"],
        size=count,
        p=[0.55, 0.22, 0.08, 0.15],
    )
    out: list[Params] = []
    for i in range(count):
        interpretation = str(interpretations[i])
        k1 = float(k1s[i])
        k2 = float(k2s[i])
        if k2 <= k1:
            k1, k2 = min(k1, k2 * 0.75), max(k2, k1 * 1.25)
        if k2 <= k1:
            k2 = k1 + 0.05
        decay_v = float(decay_vs[i])
        vf_fade = float(vf_fades[i])
        if interpretation == "x1_shifted_stribeck_peak_slope":
            decay_v = 0.5 * (ranges["decay_v"][0] + ranges["decay_v"][1])
            vf_fade = math.sqrt(ranges["vf_fade"][0] * ranges["vf_fade"][1])
        out.append(
            Params(
                interpretation=interpretation,
                k1=k1,
                k2=k2,
                k3=float(k3s[i]),
                peak_frac=float(peak_fracs[i]),
                decay0=float(decay0s[i]),
                decay_v=decay_v,
                vf_fade=vf_fade,
            )
        )
    return out


def grid_params(ranges: dict[str, tuple[float, float]]) -> list[Params]:
    def geo(name: str, n: int) -> list[float]:
        lo, hi = ranges[name]
        return [float(v) for v in np.exp(np.linspace(math.log(lo), math.log(hi), n))]

    def lin(name: str, n: int) -> list[float]:
        lo, hi = ranges[name]
        return [float(v) for v in np.linspace(lo, hi, n)]

    out: list[Params] = []
    for interpretation in ["x1_shifted_stribeck_peak_slope", "line_reaches_x2", "line_meets_x1", "line_maintains_peak"]:
        for k1 in geo("k1", 3):
            for k2 in geo("k2", 4):
                if k2 <= k1:
                    continue
                for k3 in geo("k3", 4):
                    for peak_frac in lin("peak_frac", 3):
                        decay_v_values = [0.5 * (ranges["decay_v"][0] + ranges["decay_v"][1])] if interpretation == "x1_shifted_stribeck_peak_slope" else lin("decay_v", 2)
                        vf_fade_values = [math.sqrt(ranges["vf_fade"][0] * ranges["vf_fade"][1])] if interpretation == "x1_shifted_stribeck_peak_slope" else geo("vf_fade", 3)
                        for decay0 in geo("decay0", 4):
                            for decay_v in decay_v_values:
                                for vf_fade in vf_fade_values:
                                    out.append(Params(interpretation, k1, k2, k3, peak_frac, decay0, decay_v, vf_fade))
    return out


def refine_params(rng: np.random.Generator, seed: Params, ranges: dict[str, tuple[float, float]], count: int) -> list[Params]:
    out: list[Params] = []
    for _ in range(count):
        values = {}
        for name in ["k1", "k2", "k3", "decay0", "vf_fade"]:
            lo, hi = ranges[name]
            center = getattr(seed, name)
            sigma = 0.32
            value = math.exp(rng.normal(math.log(center), sigma))
            values[name] = min(hi, max(lo, value))
        lo, hi = ranges["peak_frac"]
        values["peak_frac"] = min(hi, max(lo, rng.normal(seed.peak_frac, 0.10)))
        lo, hi = ranges["decay_v"]
        values["decay_v"] = min(hi, max(lo, rng.normal(seed.decay_v, max(0.15, 0.25 * (hi - lo)))))
        if seed.interpretation == "x1_shifted_stribeck_peak_slope":
            values["decay_v"] = 0.5 * (ranges["decay_v"][0] + ranges["decay_v"][1])
            values["vf_fade"] = math.sqrt(ranges["vf_fade"][0] * ranges["vf_fade"][1])
        if values["k2"] <= values["k1"]:
            values["k2"] = min(ranges["k2"][1], max(values["k1"] + 0.02, values["k1"] * 1.25))
        out.append(
            Params(
                seed.interpretation,
                float(values["k1"]),
                float(values["k2"]),
                float(values["k3"]),
                float(values["peak_frac"]),
                float(values["decay0"]),
                float(values["decay_v"]),
                float(values["vf_fade"]),
            )
        )
    return out


def evaluate_many(
    branch: str,
    params_list: list[Params],
    frame: pd.DataFrame,
    weights: np.ndarray,
    constants: dict[str, float],
    source: str,
) -> list[Candidate]:
    candidates = []
    seen: set[tuple[object, ...]] = set()
    for params in params_list:
        key = (
            params.interpretation,
            round(params.k1, 8),
            round(params.k2, 8),
            round(params.k3, 8),
            round(params.peak_frac, 8),
            round(params.decay0, 8),
            round(params.decay_v, 8),
            round(params.vf_fade, 8),
        )
        if key in seen:
            continue
        seen.add(key)
        if not (params.k1 < params.k2 and params.k3 > 0.0):
            continue
        candidate = build_candidate(branch, params, frame, weights, constants, source)
        if math.isfinite(candidate.train_rmse_nm):
            candidates.append(candidate)
    return candidates


def acceptable(candidate: Candidate) -> bool:
    return (
        candidate.launch_max_abs_command >= 0.6
        and candidate.launch_max_abs_command <= 0.78
        and candidate.peak_extra_nm >= 0.040
        and candidate.peak_extra_nm <= 0.105
        and candidate.slide_nm >= 0.0
        and candidate.peak_delta_nm >= 0.0
    )


def pick_candidate(candidates: list[Candidate]) -> Candidate:
    viable = [c for c in candidates if acceptable(c)]
    if not viable:
        viable = candidates
    near = [c for c in viable if c.launch_target_abs_error <= 0.030]
    pool = near if near else viable
    return min(pool, key=lambda c: (c.selected_score, c.train_rmse_nm))


def boundary_hits(params: Params, ranges: dict[str, tuple[float, float]], tolerance_fraction: float = 0.04) -> list[str]:
    hits = []
    for name in ["k1", "k2", "k3", "peak_frac", "decay0", "decay_v", "vf_fade"]:
        if params.interpretation == "x1_shifted_stribeck_peak_slope" and name in {"decay_v", "vf_fade"}:
            continue
        lo, hi = ranges[name]
        value = getattr(params, name)
        span = hi - lo
        if span <= 0.0:
            continue
        if value <= lo + tolerance_fraction * span:
            hits.append(f"{name}:low")
        if value >= hi - tolerance_fraction * span:
            hits.append(f"{name}:high")
    return hits


def expand_ranges(ranges: dict[str, tuple[float, float]], hits: list[str]) -> dict[str, tuple[float, float]]:
    expanded = dict(ranges)
    for hit in hits:
        name, side = hit.split(":")
        lo, hi = expanded[name]
        if side == "low":
            if name == "peak_frac":
                lo = max(0.02, lo - 0.08)
            elif name == "decay_v":
                lo = 0.0
            else:
                lo = max(1.0e-4, lo * 0.45)
        else:
            if name == "peak_frac":
                hi = min(0.98, hi + 0.08)
            elif name == "decay_v":
                hi = hi * 1.75 + 0.5
            else:
                hi = hi * 1.85
        expanded[name] = (lo, hi)
    return expanded


def search_branch(branch: str, frame: pd.DataFrame, weights: np.ndarray, constants: dict[str, float]) -> tuple[list[Candidate], Candidate, list[dict[str, object]], dict[str, tuple[float, float]]]:
    rng = np.random.default_rng(20260526 if branch == "base" else 20260527)
    ranges = initial_ranges()
    all_candidates: list[Candidate] = []
    audit: list[dict[str, object]] = []
    unresolved = False
    for iteration in range(3):
        before = len(all_candidates)
        broad_params = grid_params(ranges) + sample_params(rng, ranges, 9000)
        broad = evaluate_many(branch, broad_params, frame, weights, constants, f"iteration{iteration}_broad")
        broad_sorted = sorted(broad, key=lambda c: c.selected_score)
        seeds = broad_sorted[:20]
        refine_list: list[Params] = []
        for seed in seeds:
            refine_list.extend(refine_params(rng, seed.params, ranges, 260))
        refined = evaluate_many(branch, refine_list, frame, weights, constants, f"iteration{iteration}_local")
        all_candidates.extend(broad)
        all_candidates.extend(refined)
        selected = pick_candidate(all_candidates)
        hits = boundary_hits(selected.params, ranges)
        selected.boundary_status = "interior" if not hits else ";".join(hits)
        audit.append(
            {
                "branch": branch,
                "iteration": iteration,
                "range_k1": f"{ranges['k1'][0]}..{ranges['k1'][1]}",
                "range_k2": f"{ranges['k2'][0]}..{ranges['k2'][1]}",
                "range_k3": f"{ranges['k3'][0]}..{ranges['k3'][1]}",
                "range_peak_frac": f"{ranges['peak_frac'][0]}..{ranges['peak_frac'][1]}",
                "range_decay0": f"{ranges['decay0'][0]}..{ranges['decay0'][1]}",
                "range_decay_v": f"{ranges['decay_v'][0]}..{ranges['decay_v'][1]}",
                "range_vf_fade": f"{ranges['vf_fade'][0]}..{ranges['vf_fade'][1]}",
                "broad_evaluated": len(broad),
                "local_evaluated": len(refined),
                "cumulative_evaluated": len(all_candidates),
                "new_candidates": len(all_candidates) - before,
                "selected_interpretation": selected.params.interpretation,
                "selected_k1": selected.params.k1,
                "selected_k2": selected.params.k2,
                "selected_k3": selected.params.k3,
                "selected_peak_frac": selected.params.peak_frac,
                "selected_decay0": selected.params.decay0,
                "selected_decay_v": selected.params.decay_v,
                "selected_vf_fade": selected.params.vf_fade,
                "selected_train_rmse_nm": selected.train_rmse_nm,
                "selected_launch_max_abs_command": selected.launch_max_abs_command,
                "selected_peak_extra_nm": selected.peak_extra_nm,
                "boundary_hits": ";".join(hits) if hits else "",
                "action": "expand_and_rerun" if hits and iteration < 2 else "accept" if not hits else "unresolved_boundary",
            }
        )
        if not hits:
            break
        if iteration == 2:
            unresolved = True
            break
        ranges = expand_ranges(ranges, hits)
    selected = pick_candidate(all_candidates)
    hits = boundary_hits(selected.params, ranges)
    selected.boundary_status = "unresolved_boundary:" + ";".join(hits) if unresolved and hits else ("interior" if not hits else ";".join(hits))
    return all_candidates, selected, audit, ranges


def metric_row(name: str, frame: pd.DataFrame, pred: np.ndarray) -> dict[str, object]:
    baseline = frame["residual_additive_yaw_torque_nm"].to_numpy()
    corrected = corrected_residuals(frame, pred)
    baseline_rmse = rmse(baseline)
    corrected_rmse = rmse(corrected)
    baseline_mae = float(np.mean(np.abs(baseline))) if len(frame) else math.nan
    corrected_mae = float(np.mean(np.abs(corrected))) if len(frame) else math.nan
    return {
        "group": name,
        "count": int(len(frame)),
        "run_count": int(frame["run_id"].nunique()) if len(frame) else 0,
        "baseline_rmse_nm": baseline_rmse,
        "corrected_rmse_nm": corrected_rmse,
        "baseline_mae_nm": baseline_mae,
        "corrected_mae_nm": corrected_mae,
        "baseline_median_abs_nm": float(np.median(np.abs(baseline))) if len(frame) else math.nan,
        "corrected_median_abs_nm": float(np.median(np.abs(corrected))) if len(frame) else math.nan,
        "baseline_signed_median_nm": float(np.median(baseline)) if len(frame) else math.nan,
        "corrected_signed_median_nm": float(np.median(corrected)) if len(frame) else math.nan,
        "rmse_improvement_fraction": (baseline_rmse - corrected_rmse) / baseline_rmse if baseline_rmse > 0.0 else math.nan,
        "median_predicted_opposing_nm": float(np.median(pred)) if len(frame) else math.nan,
    }


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def reference_split_map() -> dict[str, dict[str, str]]:
    refs: dict[str, dict[str, str]] = {}
    for row in read_rows(FORCE_DOMAIN_SPLIT):
        refs.setdefault(row.get("dataset_split", ""), {})["force_domain_stribeck_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(RATIONAL_SPLIT):
        refs.setdefault(row.get("group", row.get("dataset_split", "")), {})["rational_residual_reference_rmse_nm"] = row.get("candidate_rmse_nm", row.get("corrected_rmse_nm", ""))
    for row in read_rows(STANDALONE_SPLIT):
        refs.setdefault(row.get("group", row.get("dataset_split", "")), {})["standalone_contact_traction_rmse_nm"] = row.get("standalone_rmse_nm", "")
    for row in read_rows(TRUE_TRACTION_SPLIT):
        refs.setdefault(row.get("group", ""), {}).update(row)
    return refs


def reference_selected_map() -> dict[str, dict[str, str]]:
    refs: dict[str, dict[str, str]] = {}
    for row in read_rows(FORCE_DOMAIN_SELECTED):
        refs.setdefault(row.get("run_id", ""), {})["force_domain_stribeck_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(RATIONAL_SELECTED):
        refs.setdefault(row.get("run_id", ""), {})["rational_residual_reference_rmse_nm"] = row.get("candidate_rmse_nm", row.get("corrected_rmse_nm", ""))
    for row in read_rows(STANDALONE_SELECTED):
        refs.setdefault(row.get("run_id", ""), {})["standalone_contact_traction_rmse_nm"] = row.get("standalone_rmse_nm", "")
    return refs


def candidate_row(candidate: Candidate, rank: int | str) -> dict[str, object]:
    return {
        "rank": rank,
        "branch": candidate.branch,
        "source": candidate.source,
        "interpretation": candidate.params.interpretation,
        "k1": candidate.params.k1,
        "k2": candidate.params.k2,
        "k3": candidate.params.k3,
        "peak_frac": candidate.params.peak_frac,
        "decay0": candidate.params.decay0,
        "decay_v": candidate.params.decay_v,
        "vf_fade": candidate.params.vf_fade,
        "slide_nm": candidate.slide_nm,
        "peak_delta_nm": candidate.peak_delta_nm,
        "peak_extra_nm": candidate.peak_extra_nm,
        "train_rmse_nm": candidate.train_rmse_nm,
        "validation_non_authoritative_rmse_nm": candidate.validation_non_authoritative_rmse_nm,
        "primary_rmse_nm": candidate.primary_rmse_nm,
        "open_floor_validation_rmse_nm": candidate.open_floor_validation_rmse_nm,
        "latest_may4_rmse_nm": candidate.latest_may4_rmse_nm,
        "launch_extra_nm": candidate.launch_extra_nm,
        "launch_total_nm": candidate.launch_total_nm,
        "launch_left_command": candidate.launch_left_command,
        "launch_right_command": candidate.launch_right_command,
        "launch_max_abs_command": candidate.launch_max_abs_command,
        "launch_target_abs_error": candidate.launch_target_abs_error,
        "selected_score": candidate.selected_score,
        "acceptable": acceptable(candidate),
        "boundary_status": candidate.boundary_status,
    }


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


def write_outputs(
    frame: pd.DataFrame,
    constants: dict[str, float],
    base_candidates: list[Candidate],
    selected: Candidate,
    latest_candidates: list[Candidate],
    latest_selected: Candidate,
    search_audit: list[dict[str, object]],
    final_ranges: dict[str, tuple[float, float]],
) -> None:
    pred = predict_opposes(frame, selected)
    split_refs = reference_split_map()
    selected_refs = reference_selected_map()

    selected_rank = "selected_base" if selected.boundary_status == "interior" else "best_unresolved_boundary_base"
    candidate_rows = [candidate_row(selected, selected_rank)]
    top = sorted(base_candidates, key=lambda c: (not acceptable(c), c.selected_score, c.train_rmse_nm))[:599]
    candidate_rows.extend(candidate_row(c, i) for i, c in enumerate(top, start=1))
    write_csv(OUT / "candidate_scores.csv", candidate_rows)

    latest_rows = [candidate_row(latest_selected, "selected_latest_weighted")]
    latest_top = sorted(latest_candidates, key=lambda c: (not acceptable(c), c.selected_score, c.train_rmse_nm))[:199]
    latest_rows.extend(candidate_row(c, i) for i, c in enumerate(latest_top, start=1))
    write_csv(OUT / "latest_weighted_sensitivity_candidates.csv", latest_rows)

    write_csv(
        OUT / "selected_parameters.csv",
        [
            {
                "parameter": "search_status",
                "value": "accepted_interior_candidate" if selected.boundary_status == "interior" else "unresolved_no_accepted_interior_candidate",
            },
            {"parameter": "branch", "value": selected.branch},
            {"parameter": "interpretation", "value": selected.params.interpretation},
            {"parameter": "k1", "value": selected.params.k1},
            {"parameter": "k2", "value": selected.params.k2},
            {"parameter": "k3", "value": selected.params.k3},
            {"parameter": "peak_frac", "value": selected.params.peak_frac},
            {"parameter": "decay0", "value": selected.params.decay0},
            {"parameter": "decay_v", "value": selected.params.decay_v},
            {"parameter": "vf_fade", "value": selected.params.vf_fade},
            {"parameter": "slide_nm", "value": selected.slide_nm},
            {"parameter": "peak_delta_nm", "value": selected.peak_delta_nm},
            {"parameter": "peak_extra_nm", "value": selected.peak_extra_nm},
            {"parameter": "linear_slope_policy", "value": "derived from selected interpretation; no independent slope fitted"},
            {"parameter": "boundary_status", "value": selected.boundary_status},
        ],
    )

    split_rows = []
    for split, subset in frame.groupby("dataset_split", sort=True):
        idx = subset.index.to_numpy()
        row = metric_row(split, subset, pred[idx])
        row.update(split_refs.get(split, {}))
        split_rows.append(row)
    nonauth = frame["dataset_split"] != "primary_open_floor_fit_authoritative"
    row = metric_row("validation_non_authoritative", frame.loc[nonauth], pred[nonauth.to_numpy()])
    row.update(split_refs.get("validation_non_authoritative", {}))
    split_rows.append(row)
    write_csv(OUT / "split_metrics.csv", split_rows)

    selected_rows = []
    for run_id in SELECTED_RUNS:
        subset = frame.loc[frame["run_id"] == run_id]
        if subset.empty:
            selected_rows.append({"run_id": run_id, "present": False})
            continue
        idx = subset.index.to_numpy()
        row = metric_row(run_id, subset, pred[idx])
        row["run_id"] = run_id
        row["present"] = True
        row["dataset_split"] = ";".join(sorted(subset["dataset_split"].unique()))
        row.update(selected_refs.get(run_id, {}))
        selected_rows.append(row)
    write_csv(OUT / "selected_log_metrics.csv", selected_rows)

    risk_defs = {
        "latest_may4_all": frame["run_id"].isin(LATEST_RUNS),
        "launch_neighborhood_abs_vf_lt_0p08_yaw_0p5_to_1p5": (frame["abs_forward_velocity_mps"] < 0.08)
        & (frame["abs_yaw_rate_radps"] >= 0.5)
        & (frame["abs_yaw_rate_radps"] <= 1.5),
        "low_speed_yaw_abs_vf_lt_0p15_abs_yaw_ge_0p5": (frame["abs_forward_velocity_mps"] < 0.15)
        & (frame["abs_yaw_rate_radps"] >= 0.5),
        "high_speed_abs_vf_ge_0p7": frame["abs_forward_velocity_mps"] >= 0.7,
        "limiter_active": frame["max_force_limiter_activity"].fillna(0.0) > 0.1,
        "hardware_saturation_evidence": frame["hardware_saturation_evidence"].fillna(0.0) > 0.1,
        "line_branch_rows": frame["abs_yaw_rate_radps"].to_numpy() <= frame["abs_forward_velocity_mps"].to_numpy() * selected.params.k1,
        "blend_branch_rows": (frame["abs_yaw_rate_radps"].to_numpy() > frame["abs_forward_velocity_mps"].to_numpy() * selected.params.k1)
        & (frame["abs_yaw_rate_radps"].to_numpy() < frame["abs_forward_velocity_mps"].to_numpy() * selected.params.k2 + selected.params.k3),
        "stribeck_branch_rows": frame["abs_yaw_rate_radps"].to_numpy() >= frame["abs_forward_velocity_mps"].to_numpy() * selected.params.k2 + selected.params.k3,
    }
    risk_rows = []
    for name, mask in risk_defs.items():
        mask_arr = np.asarray(mask, dtype=bool)
        subset = frame.loc[mask_arr]
        risk_rows.append(metric_row(name, subset, pred[mask_arr]))
    write_csv(OUT / "risk_metrics.csv", risk_rows)

    launch = candidate_launch(selected, constants)
    write_csv(
        OUT / "in_place_command_estimate.csv",
        [
            {
                "variant": "shifted_refined_selected" if selected.boundary_status == "interior" else "best_unresolved_boundary_candidate",
                **launch,
                "launch_target_left_command": 0.646,
                "launch_gate_min_abs_command": 0.6,
                "gate_pass": launch["max_abs_command"] >= 0.6,
                "target_abs_error": abs(launch["max_abs_command"] - 0.646),
            },
            {
                "variant": "latest_weighted_sensitivity",
                **candidate_launch(latest_selected, constants),
                "launch_target_left_command": 0.646,
                "launch_gate_min_abs_command": 0.6,
                "gate_pass": latest_selected.launch_max_abs_command >= 0.6,
                "target_abs_error": latest_selected.launch_target_abs_error,
            },
        ],
    )

    boundary_rows = []
    for name in ["k1", "k2", "k3", "peak_frac", "decay0", "decay_v", "vf_fade"]:
        lo, hi = final_ranges[name]
        value = getattr(selected.params, name)
        span = hi - lo
        boundary_rows.append(
            {
                "parameter": name,
                "range_min": lo,
                "range_max": hi,
                "selected_value": value,
                "fraction_from_low": (value - lo) / span if span > 0.0 else "",
                "fraction_from_high": (hi - value) / span if span > 0.0 else "",
                "boundary_adjacent_4pct": (value <= lo + 0.04 * span) or (value >= hi - 0.04 * span),
            }
        )
    write_csv(OUT / "boundary_audit.csv", boundary_rows)
    write_csv(OUT / "search_audit.csv", search_audit)

    prediction_sample = frame.loc[:, ["run_id", "dataset_split", "row_index", "forward_velocity_mps", "yaw_rate_radps", "residual_additive_yaw_torque_nm", "residual_opposes_yaw_nm"]].copy()
    prediction_sample["predicted_opposing_nm"] = pred
    prediction_sample["corrected_residual_nm"] = corrected_residuals(frame, pred)
    prediction_sample.head(2500).to_csv(OUT / "prediction_sample.csv", index=False)

    metadata = {
        "rows": int(len(frame)),
        "runs": int(frame["run_id"].nunique()),
        "base_candidate_evaluations": len(base_candidates),
        "latest_weighted_candidate_evaluations": len(latest_candidates),
        "base_search_rows": int(getattr(selected, "search_rows", 0)),
        "latest_weighted_search_rows": int(getattr(latest_selected, "search_rows", 0)),
        "final_ranges": final_ranges,
        "selected_boundary_status": selected.boundary_status,
        "selected_is_interior": selected.boundary_status == "interior",
    }
    (OUT / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    write_report(split_rows, selected_rows, risk_rows, search_audit, boundary_rows, selected, latest_selected, metadata)
    (OUT / "commands_run.txt").write_text(
        "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' "
        "codex_analysis\\yaw_model_variant_fits\\regime_linear_blend_stribeck_shifted_refined\\fit_shifted_refined.py\n",
        encoding="utf-8",
    )


def fmt(value: object, digits: int = 6) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(number):
        return ""
    return f"{number:.{digits}f}"


def write_report(
    split_rows: list[dict[str, object]],
    selected_rows: list[dict[str, object]],
    risk_rows: list[dict[str, object]],
    search_audit: list[dict[str, object]],
    boundary_rows: list[dict[str, object]],
    selected: Candidate,
    latest_selected: Candidate,
    metadata: dict[str, object],
) -> None:
    lines = [
        "# Shifted/Peak-Maintained Linear-Blend-to-Stribeck Refined Search",
        "",
        "Analysis-only output. Production code, build metadata, tests, and prior analysis artifacts were not edited.",
        "",
        "## Model",
        "",
        "`y = abs(yawRate)`, `v = abs(Vf)`, `x1 = v*k1`, `x2 = v*k2 + k3`, with `k1 < k2` and `x2 > x1`.",
        "",
        "The selected form uses a shifted Stribeck section whose peak is maintained inside the blend interval. The linear branch is anchored at zero and its slope is derived from the clean handoff or peak-maintenance condition; no independent linear slope is fitted.",
        "",
        "The primary interpretation is `x1_shifted_stribeck_peak_slope`: Stribeck parameters stay fixed, the Stribeck input is shifted by `x1`, and the peak lies at `xp=x1+peak_frac*(x2-x1)`. That means the rising Stribeck curve is already present between `x1` and `xp`, blended against the derived line continuation.",
        "",
        "The search also evaluated `line_reaches_x2`, `line_meets_x1`, and `line_maintains_peak` as comparison interpretations. The blend section always uses `w=(y-x1)/(x2-x1)` and `(1-w)*L(y,v)+w*S_shifted(y,v)`.",
        "",
        "The Stribeck peak is not forced to a transition endpoint. It is searched inside the transition region and the curve shifts with `Vf` so the maintained peak magnitude remains meaningful while the handoff locations move.",
        "",
        "## Search Status",
        "",
        "Accepted interior candidate." if selected.boundary_status == "interior" else "No candidate is accepted as selected. The best scored candidate remained boundary-adjacent after range expansion, so the search is marked unresolved per the boundary rule.",
        "",
        "## Selected Candidate" if selected.boundary_status == "interior" else "## Best Unresolved Candidate",
        "",
        "| parameter | value |",
        "| --- | ---: |",
        f"| interpretation | {selected.params.interpretation} |",
        f"| k1 | {fmt(selected.params.k1)} |",
        f"| k2 | {fmt(selected.params.k2)} |",
        f"| k3 | {fmt(selected.params.k3)} |",
        f"| peak_frac | {fmt(selected.params.peak_frac)} |",
        f"| decay0 | {fmt(selected.params.decay0)} |",
        f"| decay_v | {fmt(selected.params.decay_v)} |",
        f"| vf_fade | {fmt(selected.params.vf_fade)} |",
        f"| slide_nm | {fmt(selected.slide_nm)} |",
        f"| peak_delta_nm | {fmt(selected.peak_delta_nm)} |",
        f"| peak_extra_nm | {fmt(selected.peak_extra_nm)} |",
        f"| weighted_train_rmse_nm | {fmt(selected.train_rmse_nm)} |",
        f"| launch_extra_nm | {fmt(selected.launch_extra_nm)} |",
        f"| launch_total_nm | {fmt(selected.launch_total_nm)} |",
        f"| launch_left_command | {fmt(selected.launch_left_command)} |",
        f"| launch_right_command | {fmt(selected.launch_right_command)} |",
        f"| launch_max_abs_command | {fmt(selected.launch_max_abs_command)} |",
        f"| boundary_status | {selected.boundary_status} |",
        "",
        "## Search Audit",
        "",
        f"Base branch evaluated `{metadata['base_candidate_evaluations']}` candidates. Latest-weighted sensitivity evaluated `{metadata['latest_weighted_candidate_evaluations']}` candidates.",
        "",
        "| branch | iteration | broad | local | cumulative | boundary hits | action |",
        "| --- | ---: | ---: | ---: | ---: | --- | --- |",
    ]
    for row in search_audit:
        lines.append(
            f"| {row['branch']} | {row['iteration']} | {row['broad_evaluated']} | {row['local_evaluated']} | "
            f"{row['cumulative_evaluated']} | {row['boundary_hits']} | {row['action']} |"
        )
    lines.extend(
        [
            "",
            "## Boundary Audit",
            "",
            "| parameter | range min | selected | range max | from low | from high | adjacent |",
            "| --- | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for row in boundary_rows:
        lines.append(
            f"| {row['parameter']} | {fmt(row['range_min'])} | {fmt(row['selected_value'])} | {fmt(row['range_max'])} | "
            f"{fmt(row['fraction_from_low'])} | {fmt(row['fraction_from_high'])} | {row['boundary_adjacent_4pct']} |"
        )
    lines.extend(
        [
            "",
            "No selected base parameter is accepted if it remains boundary-adjacent after expansion. This run's selected base candidate is "
            + ("interior." if selected.boundary_status == "interior" else f"not interior: `{selected.boundary_status}`."),
            "",
            "## Split Metrics",
            "",
            "| split | count | baseline RMSE | shifted RMSE | standalone ref | force-domain ref | rational ref |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in split_rows:
        lines.append(
            f"| {row['group']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['corrected_rmse_nm'])} | "
            f"{fmt(row.get('standalone_contact_traction_rmse_nm', row.get('true_patch_corrected_rmse_nm', '')))} | "
            f"{fmt(row.get('force_domain_stribeck_rmse_nm', ''))} | {fmt(row.get('rational_residual_reference_rmse_nm', ''))} |"
        )
    lines.extend(
        [
            "",
            "## Selected Logs",
            "",
            "| run | split | count | baseline RMSE | shifted RMSE | standalone ref | force-domain ref | rational ref |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in selected_rows:
        if not row.get("present", False):
            lines.append(f"| {row['run_id']} | absent | 0 |  |  |  |  |  |")
            continue
        lines.append(
            f"| {row['run_id']} | {row['dataset_split']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['corrected_rmse_nm'])} | "
            f"{fmt(row.get('standalone_contact_traction_rmse_nm', ''))} | {fmt(row.get('force_domain_stribeck_rmse_nm', ''))} | "
            f"{fmt(row.get('rational_residual_reference_rmse_nm', ''))} |"
        )
    lines.extend(
        [
            "",
            "## Risk Metrics",
            "",
            "| slice | count | baseline RMSE | shifted RMSE | improvement |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in risk_rows:
        lines.append(
            f"| {row['group']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['corrected_rmse_nm'])} | "
            f"{fmt(row['rmse_improvement_fraction'])} |"
        )
    lines.extend(
        [
            "",
            "## Latest-Weighted Sensitivity",
            "",
            "The sensitivity branch reserves 30% effective fit weight for the two May 4 logs while retaining quality penalties and run balancing. It is reported as sensitivity, not silently substituted for the base selection.",
            "",
            "| parameter | base selected | latest-weighted selected |",
            "| --- | ---: | ---: |",
            f"| k1 | {fmt(selected.params.k1)} | {fmt(latest_selected.params.k1)} |",
            f"| k2 | {fmt(selected.params.k2)} | {fmt(latest_selected.params.k2)} |",
            f"| k3 | {fmt(selected.params.k3)} | {fmt(latest_selected.params.k3)} |",
            f"| peak_frac | {fmt(selected.params.peak_frac)} | {fmt(latest_selected.params.peak_frac)} |",
            f"| decay0 | {fmt(selected.params.decay0)} | {fmt(latest_selected.params.decay0)} |",
            f"| decay_v | {fmt(selected.params.decay_v)} | {fmt(latest_selected.params.decay_v)} |",
            f"| vf_fade | {fmt(selected.params.vf_fade)} | {fmt(latest_selected.params.vf_fade)} |",
            f"| peak_extra_nm | {fmt(selected.peak_extra_nm)} | {fmt(latest_selected.peak_extra_nm)} |",
            f"| launch_max_abs_command | {fmt(selected.launch_max_abs_command)} | {fmt(latest_selected.launch_max_abs_command)} |",
            f"| latest_may4_rmse_nm | {fmt(selected.latest_may4_rmse_nm)} | {fmt(latest_selected.latest_may4_rmse_nm)} |",
            "",
            "## Launch Tradeoff",
            "",
            (
                f"The selected base launch estimate at `Vf=0`, `Vr=0`, `yawRate=+1` is `{fmt(selected.launch_left_command)}/{fmt(selected.launch_right_command)}` with `|cmd|={fmt(selected.launch_max_abs_command)}`. It passes the `|cmd| >= 0.6` gate and is compared to the `0.646` target as a constrained objective, not as a dominating pseudo-row."
                if selected.boundary_status == "interior"
                else f"The best unresolved boundary candidate's launch estimate at `Vf=0`, `Vr=0`, `yawRate=+1` is `{fmt(selected.launch_left_command)}/{fmt(selected.launch_right_command)}` with `|cmd|={fmt(selected.launch_max_abs_command)}`. It passes the `|cmd| >= 0.6` gate but is not accepted because the candidate is boundary-bound."
            ),
            "",
            "## Output Files",
            "",
            "- `candidate_scores.csv`",
            "- `latest_weighted_sensitivity_candidates.csv`",
            "- `selected_parameters.csv`",
            "- `split_metrics.csv`",
            "- `selected_log_metrics.csv`",
            "- `risk_metrics.csv`",
            "- `in_place_command_estimate.csv`",
            "- `boundary_audit.csv`",
            "- `search_audit.csv`",
            "- `prediction_sample.csv`",
            "- `metadata.json`",
            "- `commands_run.txt`",
        ]
    )
    (OUT / "shifted_refined_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    constants = read_constants()
    frame = load_frame()
    base_weights = base_training_weights(frame)
    latest_weights = latest_weighted_training_weights(frame)
    base_search_frame, base_search_weights = search_sample(frame, base_weights, seed=20260526)
    latest_search_frame, latest_search_weights = search_sample(frame, latest_weights, seed=20260527)
    base_candidates, selected, base_audit, base_ranges = search_branch("base", base_search_frame, base_search_weights, constants)
    latest_candidates, latest_selected, latest_audit, _ = search_branch("latest_weighted", latest_search_frame, latest_search_weights, constants)
    setattr(selected, "search_rows", len(base_search_frame))
    setattr(latest_selected, "search_rows", len(latest_search_frame))
    write_outputs(
        frame,
        constants,
        base_candidates,
        selected,
        latest_candidates,
        latest_selected,
        base_audit + latest_audit,
        base_ranges,
    )


if __name__ == "__main__":
    main()
