#!/usr/bin/env python3
"""Round 2 hybrid B/C yaw residual fit.

Analysis-only tooling. This script reads existing feature samples and prior
variant artifacts, then writes outputs beside itself. It does not edit
production code, build metadata, or tests.
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

PRIMARY = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "ablation"
    / "phase_classified_feature_sample.csv"
)
SECONDARY = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "features"
    / "contact_continuum_feature_sample.csv"
)
CONSTANTS = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "features"
    / "plant_mirror_constants.csv"
)
B_DIR = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "stribeck_scrub"
C_DIR = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface"
RUNTIME_PYTHON = (
    Path.home()
    / ".cache"
    / "codex-runtimes"
    / "codex-primary-runtime"
    / "dependencies"
    / "python"
    / "python.exe"
)
IN_PLACE_ACCEPT_MIN_ABS_COMMAND = 0.600

SELECTED_LOGS = [
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
    "vbar_lat_mps",
    "vbar_yaw_mps",
    "patch_yaw_velocity_basis_m2ps",
    "max_force_preprojection_utilization",
    "max_force_limiter_activity",
    "hardware_saturation_evidence",
    "gyro_derivative_spike",
    "residual_additive_yaw_torque_nm",
    "residual_opposes_yaw_nm",
    "patch_yaw_force_basis_nm",
]

CONTACT_FIELDS = [
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

C_BASE_FEATURES = [
    "gain_front_right_basis__base",
    "gain_front_right_basis__low_rel",
    "gain_front_right_basis__high_forward",
    "gain_front_right_basis__force_util",
    "gain_front_right_basis__load_delta",
    "gain_rear_right_basis__base",
    "gain_rear_right_basis__low_rel",
    "gain_rear_right_basis__high_forward",
    "gain_rear_right_basis__force_util",
    "gain_rear_right_basis__load_delta",
    "gain_left_long_basis__base",
    "gain_left_long_basis__low_rel",
    "gain_left_long_basis__high_forward",
    "gain_left_long_basis__force_util",
    "gain_left_long_basis__load_delta",
    "gain_right_long_basis__base",
    "gain_right_long_basis__low_rel",
    "gain_right_long_basis__high_forward",
    "gain_right_long_basis__force_util",
    "gain_right_long_basis__load_delta",
    "force_moment_opposes_yaw_nm__base",
    "force_moment_opposes_yaw_nm__low_rel",
    "force_moment_opposes_yaw_nm__high_forward",
    "force_moment_opposes_yaw_nm__force_util",
    "force_abs_contact_moment_nm__force_util_signed",
]

STATIC_FEATURES = [
    "bristle_static_basis",
    "bristle_sliding_basis",
]


def read_constants() -> dict[str, float]:
    with CONSTANTS.open(newline="", encoding="utf-8") as fh:
        return {row["name"]: float(row["value"]) for row in csv.DictReader(fh)}


def read_key_value_csv(path: Path) -> dict[str, float]:
    with path.open(newline="", encoding="utf-8") as fh:
        return {row["parameter"]: float(row["value"]) for row in csv.DictReader(fh)}


def read_coeff_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def sign_array(values: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    return np.where(values > eps, 1.0, np.where(values < -eps, -1.0, 0.0))


def sign(value: float, eps: float = 1.0e-6) -> float:
    if value > eps:
        return 1.0
    if value < -eps:
        return -1.0
    return 0.0


def signed_direction(preferred: float, fallback: float) -> float:
    preferred_sign = sign(preferred)
    return preferred_sign if preferred_sign != 0.0 else sign(fallback)


def smooth_positive(values: np.ndarray, epsilon: float = 1.0e-6) -> np.ndarray:
    return 0.5 * (values + np.sqrt(values * values + epsilon * epsilon))


def smooth_positive_scalar(value: float, epsilon: float = 1.0e-6) -> float:
    return 0.5 * (value + math.sqrt(value * value + epsilon * epsilon))


def smooth_gate(value: float, scale: float) -> float:
    ratio = abs(value) / max(scale, 1.0e-12)
    return 1.0 / (1.0 + ratio * ratio)


def quantile_abs(values: np.ndarray, q: float, fallback: float = 1.0) -> float:
    clean = np.abs(values[np.isfinite(values)])
    if clean.size == 0:
        return fallback
    out = float(np.quantile(clean, q))
    if not math.isfinite(out) or out < 1.0e-12:
        rms = float(np.sqrt(np.mean(clean * clean)))
        out = rms if math.isfinite(rms) and rms > 1.0e-12 else fallback
    return out


def rmse(values: np.ndarray) -> float:
    values = values[np.isfinite(values)]
    if values.size == 0:
        return math.nan
    return float(np.sqrt(np.mean(values * values)))


def mae(values: np.ndarray) -> float:
    values = values[np.isfinite(values)]
    if values.size == 0:
        return math.nan
    return float(np.mean(np.abs(values)))


def weighted_rmse(values: np.ndarray, weights: np.ndarray) -> float:
    mask = np.isfinite(values) & np.isfinite(weights) & (weights > 0.0)
    if not np.any(mask):
        return math.nan
    return float(np.sqrt(np.average(values[mask] * values[mask], weights=weights[mask])))


def weighted_mae(values: np.ndarray, weights: np.ndarray) -> float:
    mask = np.isfinite(values) & np.isfinite(weights) & (weights > 0.0)
    if not np.any(mask):
        return math.nan
    return float(np.average(np.abs(values[mask]), weights=weights[mask]))


def run_balanced_weights(frame: pd.DataFrame) -> np.ndarray:
    counts = frame["run_id"].value_counts()
    return frame["run_id"].map(lambda run: 1.0 / max(float(counts[run]), 1.0)).to_numpy()


def load_rows(constants: dict[str, float]) -> pd.DataFrame:
    primary = pd.read_csv(PRIMARY, usecols=PRIMARY_COLUMNS)
    secondary = pd.read_csv(SECONDARY, usecols=["run_id", "row_index"] + CONTACT_FIELDS)
    frame = primary.merge(secondary, on=["run_id", "row_index"], how="left")

    string_columns = {
        "run_id",
        "family",
        "schema",
        "recommendation",
        "dataset_split",
        "physics_phase",
    }
    for column in frame.columns:
        if column not in string_columns:
            frame[column] = pd.to_numeric(frame[column], errors="coerce")

    required = [
        "forward_velocity_mps",
        "yaw_rate_radps",
        "vbar_rel_mps",
        "vbar_yaw_mps",
        "residual_additive_yaw_torque_nm",
        "patch_yaw_velocity_basis_m2ps",
    ]
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(subset=required)
    frame = frame[frame["dataset_split"] != "excluded_or_unclassified"].copy()
    frame[CONTACT_FIELDS] = frame[CONTACT_FIELDS].fillna(0.0)

    frame["abs_forward_velocity_mps"] = frame["forward_velocity_mps"].abs()
    frame["abs_yaw_rate_radps"] = frame["yaw_rate_radps"].abs()
    yaw_dir = sign_array(frame["yaw_rate_radps"].to_numpy(), eps=1.0e-5)
    velocity_dir = -sign_array(frame["patch_yaw_velocity_basis_m2ps"].to_numpy(), eps=1.0e-10)
    frame["yaw_direction"] = np.where(yaw_dir != 0.0, yaw_dir, velocity_dir)
    frame["target_opposes_yaw_nm"] = (
        -frame["yaw_direction"] * frame["residual_additive_yaw_torque_nm"]
    )

    add_contact_bases(frame, constants)
    return frame


def add_contact_bases(frame: pd.DataFrame, constants: dict[str, float]) -> None:
    half_track = 0.5 * constants["track_width_m"]
    front_f = constants["drive_wheel_longitudinal_offset_m"]
    contacts = {
        "fl": (-half_track, front_f),
        "fr": (half_track, front_f),
        "rl": (-half_track, -front_f),
        "rr": (half_track, -front_f),
    }
    yaw_dir = frame["yaw_direction"]
    total_normal = frame["total_normal_load_n"].where(frame["total_normal_load_n"] > 1.0e-9, 1.0)

    right_front = np.zeros(len(frame))
    right_rear = np.zeros(len(frame))
    long_left = np.zeros(len(frame))
    long_right = np.zeros(len(frame))
    force_moment = np.zeros(len(frame))
    force_abs_moment = np.zeros(len(frame))
    load_weighted_rel = np.zeros(len(frame))
    load_weighted_lat = np.zeros(len(frame))
    actual_force_util = np.zeros(len(frame))
    normal_reference = (
        constants["mass_kg"] * 9.80665
        + constants.get("fan_downforce_full_duty_n", 0.0) * 0.8
    )
    mu_effective = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / max(
        normal_reference, 1.0e-9
    )

    for name, (r_pos, f_pos) in contacts.items():
        vf_rel = frame[f"{name}_v_rel_f_mps"].to_numpy()
        vr_rel = frame[f"{name}_v_rel_r_mps"].to_numpy()
        normal_frac = (frame[f"{name}_normal_n"] / total_normal).to_numpy()

        right_basis = (-yaw_dir * f_pos * frame[f"{name}_v_rel_r_mps"]).to_numpy()
        long_basis = (yaw_dir * r_pos * frame[f"{name}_v_rel_f_mps"]).to_numpy()
        if f_pos > 0.0:
            right_front += right_basis
        else:
            right_rear += right_basis
        if r_pos < 0.0:
            long_left += long_basis
        else:
            long_right += long_basis

        force_f = frame[f"{name}_force_f_n"].to_numpy()
        force_r = frame[f"{name}_force_r_n"].to_numpy()
        force_local = f_pos * force_r - r_pos * force_f
        force_moment += force_local
        force_abs_moment += np.abs(force_local)
        load_weighted_rel += normal_frac * np.hypot(vf_rel, vr_rel)
        load_weighted_lat += normal_frac * np.abs(vr_rel)
        normal = frame[f"{name}_normal_n"].to_numpy()
        force_capacity = np.maximum(mu_effective * normal, 1.0e-9)
        actual_force_util = np.maximum(actual_force_util, np.hypot(force_f, force_r) / force_capacity)

    frame["gain_front_right_basis"] = right_front
    frame["gain_rear_right_basis"] = right_rear
    frame["gain_left_long_basis"] = long_left
    frame["gain_right_long_basis"] = long_right
    frame["force_moment_opposes_yaw_nm"] = -yaw_dir.to_numpy() * force_moment
    frame["force_abs_contact_moment_nm"] = force_abs_moment
    frame["load_weighted_rel_mps"] = load_weighted_rel
    frame["load_weighted_lat_mps"] = load_weighted_lat
    actual_force_util = np.clip(actual_force_util, 0.0, 5.0)
    frame["actual_force_utilization"] = actual_force_util
    frame["actual_force_util_smooth"] = actual_force_util / (1.0 + actual_force_util)


@dataclass(frozen=True)
class HybridConfig:
    vrel_knee_mps: float
    fwd_knee_mps: float
    ridge: float
    static_ridge: float
    downweighted_fit_weight: float
    bristle_velocity_mps: float
    slide_gate_power: float
    stribeck_speed_mps: float = 0.100
    speed_fade_mps: float = 0.640
    rel_weight: float = 0.750


@dataclass
class HybridModel:
    config: HybridConfig
    nominal_load_n: float
    feature_names: list[str]
    scales: np.ndarray
    beta: np.ndarray
    anchor_extra_nm: float
    anchor_constraint_lagrange: float
    static_raw_coefficients: dict[str, float]

    def predict_frame(self, frame: pd.DataFrame) -> np.ndarray:
        raw = feature_matrix(frame, self.config, self.feature_names, self.nominal_load_n)
        clipped = clip_and_scale(raw, self.scales)
        pred = clipped @ self.beta
        return np.where(frame["yaw_direction"].to_numpy() != 0.0, pred, 0.0)


def feature_names() -> list[str]:
    return C_BASE_FEATURES + STATIC_FEATURES


def selected_nominal_load(frame: pd.DataFrame) -> float:
    train = frame[
        (frame["dataset_split"] == "primary_open_floor_fit_authoritative")
        & (frame["total_normal_load_n"] > 0.0)
    ]["total_normal_load_n"]
    if len(train):
        return float(train.median())
    positive = frame[frame["total_normal_load_n"] > 0.0]["total_normal_load_n"]
    return float(positive.median()) if len(positive) else 1.0


def feature_matrix(
    frame: pd.DataFrame,
    config: HybridConfig,
    names: list[str],
    nominal_load_n: float,
) -> np.ndarray:
    vrel = np.maximum(frame["vbar_rel_mps"].to_numpy(), frame["load_weighted_rel_mps"].to_numpy())
    vyaw_contact = np.maximum(frame["vbar_yaw_mps"].to_numpy(), frame["load_weighted_lat_mps"].to_numpy())
    vf = frame["abs_forward_velocity_mps"].to_numpy()
    low_rel = 1.0 / (1.0 + np.square(vrel / max(config.vrel_knee_mps, 1.0e-9)))
    high_forward = 1.0 - (
        1.0 / (1.0 + np.square(vf / max(config.fwd_knee_mps, 1.0e-9)))
    )
    load_delta = frame["total_normal_load_n"].to_numpy() / max(nominal_load_n, 1.0e-9) - 1.0
    transition = np.hypot(config.rel_weight * vrel, vf)
    stribeck = np.exp(-np.square(transition / max(config.stribeck_speed_mps, 1.0e-9)))
    speed_relief = 1.0 / (1.0 + np.square(transition / max(config.speed_fade_mps, 1.0e-9)))
    bristle_activation = 1.0 - np.exp(
        -np.square(config.rel_weight * vyaw_contact / max(config.bristle_velocity_mps, 1.0e-9))
    )
    adhesion_lock = np.clip(bristle_activation * stribeck * speed_relief, 0.0, 1.0)
    moving_contact_gate = np.power(1.0 - adhesion_lock, config.slide_gate_power)
    load_ratio = frame["total_normal_load_n"].to_numpy() / max(nominal_load_n, 1.0e-9)
    schedules = {
        "base": moving_contact_gate,
        "low_rel": low_rel * moving_contact_gate,
        "high_forward": high_forward * moving_contact_gate,
        "force_util": frame["actual_force_util_smooth"].to_numpy() * moving_contact_gate,
        "load_delta": load_delta * moving_contact_gate,
    }
    static_values = {
        "bristle_static_basis": load_ratio * bristle_activation * stribeck * speed_relief,
        "bristle_sliding_basis": load_ratio * bristle_activation * speed_relief,
    }

    cols: list[np.ndarray] = []
    for name in names:
        if name in static_values:
            cols.append(static_values[name])
            continue
        base, suffix = name.split("__", 1)
        if suffix == "force_util_signed":
            cols.append(frame[base].to_numpy() * schedules["force_util"] * frame["yaw_direction"].to_numpy())
        else:
            cols.append(frame[base].to_numpy() * schedules.get(suffix, 1.0))
    return np.column_stack(cols)


def clip_and_scale(raw: np.ndarray, scales: np.ndarray, clip_sigma: float = 8.0) -> np.ndarray:
    limit = clip_sigma * scales
    clipped = np.minimum(np.maximum(raw, -limit), limit)
    return clipped / scales


def fit_weights(frame: pd.DataFrame, config: HybridConfig) -> np.ndarray:
    split = frame["dataset_split"]
    recommendation = frame["recommendation"]
    family = frame["family"]
    weights = np.zeros(len(frame))
    weights[split == "primary_open_floor_fit_authoritative"] = 1.0
    downweighted = (
        (split == "open_floor_fit_downweighted")
        & (recommendation == "fit_downweighted")
        & (family == "open_floor")
    )
    weights[downweighted.to_numpy()] = config.downweighted_fit_weight

    limiter = np.clip(frame["max_force_limiter_activity"].to_numpy(), 0.0, 1.0)
    saturation = np.clip(frame["hardware_saturation_evidence"].to_numpy(), 0.0, 1.0)
    spike = np.clip(frame["gyro_derivative_spike"].to_numpy(), 0.0, 1.0)
    quality = (1.0 / (1.0 + 4.0 * limiter)) * (1.0 - 0.75 * saturation) * (
        1.0 - 0.75 * spike
    )
    weights *= np.clip(quality, 0.02, 1.0)

    fit_counts = frame.loc[weights > 0.0, "run_id"].value_counts()
    if not fit_counts.empty:
        scale = frame["run_id"].map(
            {run: 1.0 / math.sqrt(float(count)) for run, count in fit_counts.items()}
        ).fillna(0.0)
        weights *= scale.to_numpy()
        positive = weights > 0.0
        weights[positive] *= positive.sum() / max(float(weights[positive].sum()), 1.0e-12)
    return weights


def solve_constrained_ridge(
    x_scaled: np.ndarray,
    y: np.ndarray,
    weights: np.ndarray,
    ridge_diag: np.ndarray,
    anchor_scaled: np.ndarray,
    anchor_target: float,
) -> tuple[np.ndarray, float]:
    sqrt_w = np.sqrt(np.clip(weights, 0.0, None))
    xw = x_scaled * sqrt_w[:, None]
    yw = y * sqrt_w
    xtx = xw.T @ xw
    xty = xw.T @ yw
    xtx += np.diag(ridge_diag)

    p = x_scaled.shape[1]
    kkt = np.zeros((p + 1, p + 1))
    kkt[:p, :p] = xtx
    kkt[:p, p] = anchor_scaled
    kkt[p, :p] = anchor_scaled
    rhs = np.zeros(p + 1)
    rhs[:p] = xty
    rhs[p] = anchor_target
    try:
        solved = np.linalg.solve(kkt, rhs)
    except np.linalg.LinAlgError:
        solved = np.linalg.lstsq(kkt, rhs, rcond=None)[0]
    return solved[:p], float(solved[p])


def synthetic_base_frame(
    constants: dict[str, float],
    config: HybridConfig,
    nominal_load_n: float,
    vf_mps: float,
    yaw_rate_radps: float,
    extra_opposes_nm: float,
) -> pd.DataFrame:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    base = baseline_opposing_yaw_torque(constants, yaw_rate_radps)
    force_moment = extra_opposes_nm
    util = min(max(contact_utilization(constants, yaw_rate_radps), 0.0), 5.0)
    limiter = min(max(limiter_activity(constants, yaw_rate_radps), 0.0), 5.0)
    vrel = longitudinal * abs(yaw_rate_radps)

    row = {
        "forward_velocity_mps": vf_mps,
        "abs_forward_velocity_mps": abs(vf_mps),
        "yaw_rate_radps": yaw_rate_radps,
        "abs_yaw_rate_radps": abs(yaw_rate_radps),
        "yaw_direction": sign(yaw_rate_radps, eps=1.0e-9) or 1.0,
        "vbar_rel_mps": vrel,
        "vbar_yaw_mps": vrel,
        "load_weighted_rel_mps": vrel,
        "load_weighted_lat_mps": vrel,
        "total_normal_load_n": nominal_load_n,
        "gain_front_right_basis": 2.0 * longitudinal * longitudinal * abs(yaw_rate_radps),
        "gain_rear_right_basis": 2.0 * longitudinal * longitudinal * abs(yaw_rate_radps),
        "gain_left_long_basis": 0.0,
        "gain_right_long_basis": 0.0,
        "force_moment_opposes_yaw_nm": -force_moment,
        "force_abs_contact_moment_nm": abs(force_moment),
        "max_force_preprojection_utilization": util,
        "max_force_limiter_activity": limiter,
        "actual_force_utilization": util,
        "actual_force_util_smooth": util / (1.0 + util),
        "residual_additive_yaw_torque_nm": 0.0,
    }
    return pd.DataFrame([row])


def fit_model(
    frame: pd.DataFrame,
    constants: dict[str, float],
    config: HybridConfig,
    anchor_extra_nm: float,
    nominal_load_n: float,
) -> HybridModel:
    names = feature_names()
    b_coeff = read_key_value_csv(B_DIR / "stribeck_coefficients.csv")
    static_raw = {
        "bristle_static_basis": b_coeff["static_extra_nm"],
        "bristle_sliding_basis": b_coeff["sliding_nm"],
    }
    weights = fit_weights(frame, config)
    fit_mask = weights > 0.0
    raw = feature_matrix(frame, config, names, nominal_load_n)
    scales = np.array([quantile_abs(raw[fit_mask, idx], 0.80) for idx in range(raw.shape[1])])
    for idx, name in enumerate(names):
        if name in STATIC_FEATURES:
            scales[idx] = 1.0
    x_scaled = clip_and_scale(raw, scales)
    beta = np.zeros(len(names))

    static_indices = [idx for idx, name in enumerate(names) if name in STATIC_FEATURES]
    moving_indices = [idx for idx, name in enumerate(names) if name not in STATIC_FEATURES]
    anchor_raw = feature_matrix(
        synthetic_base_frame(constants, config, nominal_load_n, 0.0, 1.0, anchor_extra_nm),
        config,
        names,
        nominal_load_n,
    )[0]
    anchor_unscaled_static = sum(anchor_raw[idx] * static_raw[names[idx]] for idx in static_indices)
    static_scale = anchor_extra_nm / max(anchor_unscaled_static, 1.0e-12)
    static_raw = {name: value * static_scale for name, value in static_raw.items()}
    for idx in static_indices:
        raw_coeff = static_raw[names[idx]]
        beta[idx] = raw_coeff * scales[idx]
    static_pred = np.zeros(len(frame))
    for idx in static_indices:
        static_pred += raw[:, idx] * static_raw[names[idx]]

    y = frame["target_opposes_yaw_nm"].to_numpy() - static_pred
    sqrt_w = np.sqrt(np.clip(weights, 0.0, None))
    xm = x_scaled[:, moving_indices] * sqrt_w[:, None]
    ym = y * sqrt_w
    xtx = xm.T @ xm
    xty = xm.T @ ym
    xtx += np.eye(len(moving_indices)) * config.ridge
    try:
        beta_moving = np.linalg.solve(xtx, xty)
    except np.linalg.LinAlgError:
        beta_moving = np.linalg.lstsq(xtx, xty, rcond=None)[0]
    for dst, value in zip(moving_indices, beta_moving):
        beta[dst] = value

    return HybridModel(
        config=config,
        nominal_load_n=nominal_load_n,
        feature_names=names,
        scales=scales,
        beta=beta,
        anchor_extra_nm=anchor_extra_nm,
        anchor_constraint_lagrange=0.0,
        static_raw_coefficients=static_raw,
    )


def predict_raw_correction(frame: pd.DataFrame, model: HybridModel) -> np.ndarray:
    pred_opposes = model.predict_frame(frame)
    return -frame["yaw_direction"].to_numpy() * pred_opposes


def corrected_residuals(frame: pd.DataFrame, model: HybridModel) -> np.ndarray:
    return frame["residual_additive_yaw_torque_nm"].to_numpy() - predict_raw_correction(frame, model)


def metric_row(label: str, frame: pd.DataFrame, model: HybridModel) -> dict[str, object]:
    baseline = frame["residual_additive_yaw_torque_nm"].to_numpy()
    corrected = corrected_residuals(frame, model)
    weights = run_balanced_weights(frame)
    baseline_rmse = rmse(baseline)
    corrected_rmse = rmse(corrected)
    rb_baseline = weighted_rmse(baseline, weights)
    rb_corrected = weighted_rmse(corrected, weights)
    return {
        "group": label,
        "count": int(len(frame)),
        "run_count": int(frame["run_id"].nunique()) if len(frame) else 0,
        "baseline_rmse_nm": baseline_rmse,
        "corrected_rmse_nm": corrected_rmse,
        "baseline_mae_nm": mae(baseline),
        "corrected_mae_nm": mae(corrected),
        "baseline_median_abs_nm": float(np.median(np.abs(baseline))) if len(frame) else math.nan,
        "corrected_median_abs_nm": float(np.median(np.abs(corrected))) if len(frame) else math.nan,
        "baseline_signed_median_nm": float(np.median(baseline)) if len(frame) else math.nan,
        "corrected_signed_median_nm": float(np.median(corrected)) if len(frame) else math.nan,
        "run_balanced_baseline_rmse_nm": rb_baseline,
        "run_balanced_corrected_rmse_nm": rb_corrected,
        "run_balanced_baseline_mae_nm": weighted_mae(baseline, weights),
        "run_balanced_corrected_mae_nm": weighted_mae(corrected, weights),
        "rmse_improvement_pct": 100.0 * (baseline_rmse - corrected_rmse) / baseline_rmse
        if baseline_rmse > 0.0
        else math.nan,
        "run_balanced_rmse_improvement_pct": 100.0 * (rb_baseline - rb_corrected) / rb_baseline
        if rb_baseline > 0.0
        else math.nan,
    }


def risk_groups(frame: pd.DataFrame) -> dict[str, pd.DataFrame]:
    return {
        "straightish_abs_yaw_lt_0p05": frame[frame["abs_yaw_rate_radps"] < 0.05],
        "straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05": frame[
            (frame["abs_yaw_rate_radps"] < 0.05)
            & (frame["abs_forward_velocity_mps"] >= 0.05)
        ],
        "low_speed_yaw_vf_lt_0p05_yaw_ge_0p2": frame[
            (frame["abs_forward_velocity_mps"] < 0.05)
            & (frame["abs_yaw_rate_radps"] >= 0.2)
        ],
        "high_forward_vf_ge_0p5": frame[frame["abs_forward_velocity_mps"] >= 0.5],
        "limiter_active": frame[frame["max_force_limiter_activity"] > 0.0],
    }


def objective_score(model: HybridModel, frame: pd.DataFrame) -> dict[str, float]:
    validation = frame[
        frame["dataset_split"].isin(
            ["open_floor_validation_only", "diag_validation_only", "aux_downweighted_validation"]
        )
    ]
    validation_metric = metric_row("validation_objective", validation, model)
    primary_metric = metric_row(
        "primary_open_floor_fit_authoritative",
        frame[frame["dataset_split"] == "primary_open_floor_fit_authoritative"],
        model,
    )
    open_fit_metric = metric_row(
        "open_floor_fit_downweighted",
        frame[frame["dataset_split"] == "open_floor_fit_downweighted"],
        model,
    )
    straight_metric = metric_row(
        "straightish_abs_yaw_lt_0p05", risk_groups(frame)["straightish_abs_yaw_lt_0p05"], model
    )
    high_metric = metric_row("high_forward_vf_ge_0p5", risk_groups(frame)["high_forward_vf_ge_0p5"], model)
    low_launch_metric = metric_row(
        "low_speed_yaw_vf_lt_0p05_yaw_ge_0p2",
        risk_groups(frame)["low_speed_yaw_vf_lt_0p05_yaw_ge_0p2"],
        model,
    )

    score = float(validation_metric["run_balanced_corrected_rmse_nm"])
    score += 0.25 * max(
        0.0,
        float(straight_metric["run_balanced_corrected_rmse_nm"])
        - float(straight_metric["run_balanced_baseline_rmse_nm"]),
    )
    score += 0.20 * max(
        0.0,
        float(high_metric["run_balanced_corrected_rmse_nm"])
        - 1.03 * float(high_metric["run_balanced_baseline_rmse_nm"]),
    )
    return {
        "objective_score": score,
        "validation_objective_rb_corrected_rmse_nm": validation_metric[
            "run_balanced_corrected_rmse_nm"
        ],
        "validation_objective_rb_baseline_rmse_nm": validation_metric[
            "run_balanced_baseline_rmse_nm"
        ],
        "primary_rb_corrected_rmse_nm": primary_metric["run_balanced_corrected_rmse_nm"],
        "open_fit_rb_corrected_rmse_nm": open_fit_metric["run_balanced_corrected_rmse_nm"],
        "straight_rb_corrected_rmse_nm": straight_metric["run_balanced_corrected_rmse_nm"],
        "high_forward_rb_corrected_rmse_nm": high_metric["run_balanced_corrected_rmse_nm"],
        "low_launch_rb_corrected_rmse_nm": low_launch_metric["run_balanced_corrected_rmse_nm"],
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


def wheel_speeds(
    vf_mps: float, yaw_rate: float, constants: dict[str, float]
) -> tuple[float, float, float, float]:
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


def motor_commands_for_opposing_torque(
    opposing_yaw_torque: float,
    constants: dict[str, float],
    vf_mps: float,
    yaw_rate: float,
) -> dict[str, float]:
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
        "left_surface_mps": left_surface,
        "right_surface_mps": right_surface,
        "left_wheel_speed_radps": left_speed,
        "right_wheel_speed_radps": right_speed,
        "left_launch_torque_nm": left_launch,
        "right_launch_torque_nm": right_launch,
    }


def contact_utilization(constants: dict[str, float], yaw_rate: float) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    front_force = abs(constants["front_right_contact_force_gain_n_per_mps"] * longitudinal * yaw_rate)
    rear_force = abs(constants["rear_right_contact_force_gain_n_per_mps"] * longitudinal * yaw_rate)
    force_limit = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / 4.0
    return max(front_force, rear_force) / force_limit


def limiter_activity(constants: dict[str, float], yaw_rate: float) -> float:
    return max(0.0, contact_utilization(constants, yaw_rate) - 1.0)


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


def variant_b_extra(base_opposing: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> float:
    coeff = read_key_value_csv(B_DIR / "stribeck_coefficients.csv")
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    vbar_rel = longitudinal * abs(yaw_rate)
    transition_speed = math.hypot(coeff["rel_weight"] * vbar_rel, abs(vf_mps))
    stribeck = math.exp(-((transition_speed / coeff["stribeck_speed_mps"]) ** 2))
    speed_relief = 1.0 / (1.0 + (transition_speed / coeff["speed_fade_mps"]) ** 2)

    extra = 0.0
    for _ in range(80):
        requested = base_opposing + extra
        activation = 1.0 - math.exp(
            -((smooth_positive_scalar(requested) / coeff["req_activation_nm"]) ** 2)
        )
        next_extra = activation * speed_relief * (
            coeff["static_extra_nm"] * stribeck + coeff["sliding_nm"]
        )
        if abs(next_extra - extra) < 1.0e-13:
            return next_extra
        extra = next_extra
    return extra


def variant_c_extra(base_opposing: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> float:
    rows = read_coeff_rows(C_DIR / "model_coefficients.csv")
    coeff_rows = [row for row in rows if row["candidate"] == "saturation_aware_surface"]
    coeffs = {row["feature"]: float(row["standardized_coefficient_nm"]) for row in coeff_rows}
    scales = {row["feature"]: float(row["feature_scale"]) for row in coeff_rows}
    vrel_knee = float(coeff_rows[0]["vrel_knee_mps"])
    fwd_knee = float(coeff_rows[0]["fwd_knee_mps"])

    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    v_rel = longitudinal * abs(yaw_rate)
    util = min(max(contact_utilization(constants, yaw_rate), 0.0), 5.0)
    limiter = min(max(limiter_activity(constants, yaw_rate), 0.0), 5.0)
    util_smooth = util / (1.0 + util)
    limiter_smooth = limiter / (1.0 + limiter)
    low_rel = smooth_gate(v_rel, vrel_knee)
    high_forward = 1.0 - smooth_gate(vf_mps, fwd_knee)
    right_front = 2.0 * (longitudinal * longitudinal) * abs(yaw_rate)
    right_rear = 2.0 * (longitudinal * longitudinal) * abs(yaw_rate)

    def feature_value(feature: str, extra: float) -> float:
        total_req = base_opposing + extra
        values = {
            "gain_front_right_basis": right_front,
            "gain_rear_right_basis": right_rear,
            "gain_left_long_basis": 0.0,
            "gain_right_long_basis": 0.0,
            "force_gap_opposes_yaw_nm": -base_opposing,
            "req_moment_opposes_yaw_nm": -total_req,
            "force_moment_opposes_yaw_nm": -extra,
            "req_abs_contact_moment_nm": abs(total_req),
            "force_abs_contact_moment_nm": abs(extra),
        }
        base, suffix = feature.split("__", 1)
        x = values.get(base, 0.0)
        if suffix == "base":
            return x
        if suffix == "low_rel":
            return x * low_rel
        if suffix == "high_forward":
            return x * high_forward
        if suffix == "util":
            return x * util_smooth
        if suffix == "limiter":
            return x * limiter_smooth
        if suffix == "limiter_signed":
            return x * limiter_smooth
        if suffix == "load_delta":
            return 0.0
        return 0.0

    extra = 0.0
    for _ in range(80):
        predicted = 0.0
        for feature, beta in coeffs.items():
            scale = max(scales.get(feature, 1.0), 1.0e-12)
            value = feature_value(feature, extra)
            value = min(max(value, -8.0 * scale), 8.0 * scale)
            predicted += beta * (value / scale)
        if abs(predicted - extra) < 1.0e-13:
            return predicted
        extra = predicted
    return extra


def hybrid_extra(
    model: HybridModel,
    constants: dict[str, float],
    vf_mps: float,
    yaw_rate: float,
    seed: float | None = None,
) -> float:
    extra = model.anchor_extra_nm if seed is None and abs(vf_mps) < 1.0e-12 and abs(yaw_rate - 1.0) < 1.0e-12 else (seed or 0.0)
    for _ in range(100):
        row = synthetic_base_frame(constants, model.config, model.nominal_load_n, vf_mps, yaw_rate, extra)
        predicted = float(model.predict_frame(row)[0])
        if abs(predicted - extra) < 1.0e-12:
            return predicted
        extra = 0.5 * extra + 0.5 * predicted
    return extra


def vf_grid() -> list[float]:
    return [round(i * 0.15 / 5.0, 9) for i in range(6)]


def yaw_grid() -> list[float]:
    return [round(0.2 + i * (6.0 - 0.2) / 9.0, 9) for i in range(10)]


def make_command_row(
    variant: str,
    extra: float,
    constants: dict[str, float],
    vf_mps: float,
    yaw_rate: float,
    caveat: str,
) -> dict[str, object]:
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    total = base + extra
    cmd = motor_commands_for_opposing_torque(total, constants, vf_mps, yaw_rate)
    max_abs_cmd = max(abs(cmd["left_command"]), abs(cmd["right_command"]))
    util = contact_utilization(constants, yaw_rate)
    limiter = limiter_activity(constants, yaw_rate)
    in_place_gate_applies = abs(vf_mps) < 1.0e-12 and abs(yaw_rate - 1.0) < 1.0e-12
    return {
        "vf_mps": vf_mps,
        "yaw_rate_radps": yaw_rate,
        "variant": variant,
        "baseline_opposing_yaw_torque_nm": base,
        "extra_opposing_yaw_torque_nm": extra,
        "total_opposing_yaw_torque_nm": total,
        "required_applied_bank_torque_nm": cmd["applied_bank_torque_nm"],
        "left_command": cmd["left_command"],
        "right_command": cmd["right_command"],
        "lr_delta_command": cmd["lr_delta_command"],
        "left_surface_mps": cmd["left_surface_mps"],
        "right_surface_mps": cmd["right_surface_mps"],
        "left_command_torque_nm": cmd["left_command_torque_nm"],
        "right_command_torque_nm": cmd["right_command_torque_nm"],
        "left_launch_torque_nm": cmd["left_launch_torque_nm"],
        "right_launch_torque_nm": cmd["right_launch_torque_nm"],
        "max_abs_command": max_abs_cmd,
        "in_place_acceptance_min_abs_command": IN_PLACE_ACCEPT_MIN_ABS_COMMAND
        if in_place_gate_applies
        else "",
        "in_place_acceptance_gate_applies": in_place_gate_applies,
        "in_place_acceptance_pass": bool(max_abs_cmd >= IN_PLACE_ACCEPT_MIN_ABS_COMMAND)
        if in_place_gate_applies
        else "",
        "command_outside_unit": max_abs_cmd > 1.0,
        "contact_utilization_raw": util,
        "limiter_activity_proxy": limiter,
        "contact_projection_sensitive": util > 1.0,
        "low_speed_launch_sensitive": min(abs(cmd["left_surface_mps"]), abs(cmd["right_surface_mps"]))
        < 2.0 * constants["static_friction_max_speed_mps"],
        "caveat": caveat,
    }


def command_rows_for_point(
    model: HybridModel, constants: dict[str, float], vf_mps: float, yaw_rate: float
) -> list[dict[str, object]]:
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    b = variant_b_extra(base, constants, vf_mps, yaw_rate)
    c = variant_c_extra(base, constants, vf_mps, yaw_rate)
    h = hybrid_extra(model, constants, vf_mps, yaw_rate, seed=b if abs(vf_mps) < 0.02 else c)
    return [
        make_command_row("Baseline", 0.0, constants, vf_mps, yaw_rate, "current raw right-contact scrub approximation"),
        make_command_row("B_stribeck", b, constants, vf_mps, yaw_rate, "request-activated Stribeck scrub"),
        make_command_row("C_combined_slip", c, constants, vf_mps, yaw_rate, "saturation-aware moving contact surface"),
        make_command_row(
            "Hybrid_BC_adhesion_partition",
            h,
            constants,
            vf_mps,
            yaw_rate,
            "B-calibrated static adhesion reservoir plus gated moving contact surface",
        ),
    ]


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("empty\n", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_frame(path: Path, frame: pd.DataFrame) -> None:
    frame.to_csv(path, index=False)


def split_metrics(frame: pd.DataFrame, model: HybridModel) -> pd.DataFrame:
    rows = []
    for split in [
        "primary_open_floor_fit_authoritative",
        "open_floor_fit_downweighted",
        "open_floor_validation_only",
        "diag_validation_only",
        "aux_downweighted_validation",
    ]:
        subset = frame[frame["dataset_split"] == split]
        if len(subset):
            rows.append(metric_row(split, subset, model))
    validation = frame[
        frame["dataset_split"].isin(
            [
                "open_floor_fit_downweighted",
                "open_floor_validation_only",
                "diag_validation_only",
                "aux_downweighted_validation",
            ]
        )
    ]
    rows.append(metric_row("validation_non_authoritative", validation, model))
    return pd.DataFrame(rows)


def selected_log_metrics(frame: pd.DataFrame, model: HybridModel) -> pd.DataFrame:
    rows = []
    for run_id in SELECTED_LOGS:
        subset = frame[frame["run_id"] == run_id]
        if len(subset):
            row = metric_row(run_id, subset, model)
            row["run_id"] = run_id
            row["present"] = True
            row["dataset_split"] = ";".join(sorted(subset["dataset_split"].unique()))
            rows.append(row)
        else:
            rows.append({"run_id": run_id, "present": False, "count": 0, "dataset_split": ""})
    return pd.DataFrame(rows)


def risk_metrics(frame: pd.DataFrame, model: HybridModel) -> pd.DataFrame:
    rows = []
    for group, subset in risk_groups(frame).items():
        if len(subset):
            rows.append(metric_row(group, subset, model))
    return pd.DataFrame(rows)


def coefficient_frame(model: HybridModel) -> pd.DataFrame:
    rows = []
    for name, scale, beta in zip(model.feature_names, model.scales, model.beta):
        branch = "static_launch" if name in STATIC_FEATURES else "moving_contact"
        rows.append(
            {
                "branch": branch,
                "feature": name,
                "standardized_coefficient_nm": beta,
                "feature_scale": scale,
                "raw_coefficient_nm_per_feature": beta / scale,
                "abs_standardized_coefficient_nm": abs(beta),
            }
        )
    return pd.DataFrame(rows).sort_values("abs_standardized_coefficient_nm", ascending=False)


def compare_split_metrics(hybrid: pd.DataFrame) -> pd.DataFrame:
    b = pd.read_csv(B_DIR / "metrics_by_split.csv").rename(
        columns={
            "dataset_split": "split",
            "corrected_rmse_nm": "b_corrected_rmse_nm",
            "baseline_rmse_nm": "b_baseline_rmse_nm",
        }
    )
    c = pd.read_csv(C_DIR / "split_metrics.csv").rename(
        columns={
            "group": "split",
            "corrected_rmse_nm": "c_corrected_rmse_nm",
            "baseline_rmse_nm": "c_baseline_rmse_nm",
        }
    )
    h = hybrid.rename(
        columns={
            "group": "split",
            "corrected_rmse_nm": "hybrid_corrected_rmse_nm",
            "baseline_rmse_nm": "hybrid_baseline_rmse_nm",
            "run_balanced_corrected_rmse_nm": "hybrid_rb_corrected_rmse_nm",
        }
    )
    merged = h[
        [
            "split",
            "count",
            "run_count",
            "hybrid_baseline_rmse_nm",
            "hybrid_corrected_rmse_nm",
            "hybrid_rb_corrected_rmse_nm",
            "rmse_improvement_pct",
        ]
    ].merge(b[["split", "b_corrected_rmse_nm"]], on="split", how="left")
    merged = merged.merge(c[["split", "c_corrected_rmse_nm"]], on="split", how="left")
    merged["hybrid_vs_b_rmse_pct"] = 100.0 * (
        merged["b_corrected_rmse_nm"] - merged["hybrid_corrected_rmse_nm"]
    ) / merged["b_corrected_rmse_nm"]
    merged["hybrid_vs_c_rmse_pct"] = 100.0 * (
        merged["c_corrected_rmse_nm"] - merged["hybrid_corrected_rmse_nm"]
    ) / merged["c_corrected_rmse_nm"]
    return merged


def compare_selected_metrics(hybrid: pd.DataFrame) -> pd.DataFrame:
    b = pd.read_csv(B_DIR / "metrics_by_selected_run.csv").rename(
        columns={"corrected_rmse_nm": "b_corrected_rmse_nm", "baseline_rmse_nm": "b_baseline_rmse_nm"}
    )
    c = pd.read_csv(C_DIR / "selected_log_metrics.csv").rename(
        columns={"Run": "run_id"} if "Run" in pd.read_csv(C_DIR / "selected_log_metrics.csv", nrows=0).columns else {}
    )
    c = c.rename(columns={"corrected_rmse_nm": "c_corrected_rmse_nm", "baseline_rmse_nm": "c_baseline_rmse_nm"})
    h = hybrid.rename(
        columns={
            "corrected_rmse_nm": "hybrid_corrected_rmse_nm",
            "baseline_rmse_nm": "hybrid_baseline_rmse_nm",
        }
    )
    merged = h[
        ["run_id", "present", "dataset_split", "count", "hybrid_baseline_rmse_nm", "hybrid_corrected_rmse_nm"]
    ].merge(b[["run_id", "b_corrected_rmse_nm"]], on="run_id", how="left")
    merged = merged.merge(c[["run_id", "c_corrected_rmse_nm"]], on="run_id", how="left")
    merged["hybrid_vs_b_rmse_pct"] = 100.0 * (
        merged["b_corrected_rmse_nm"] - merged["hybrid_corrected_rmse_nm"]
    ) / merged["b_corrected_rmse_nm"]
    merged["hybrid_vs_c_rmse_pct"] = 100.0 * (
        merged["c_corrected_rmse_nm"] - merged["hybrid_corrected_rmse_nm"]
    ) / merged["c_corrected_rmse_nm"]
    return merged


def pivot_table_lines(grid: pd.DataFrame, variant: str) -> list[str]:
    subset = grid[grid["variant"] == variant]
    by_key = {
        (float(row.vf_mps), float(row.yaw_rate_radps)): float(row.lr_delta_command)
        for row in subset.itertuples(index=False)
    }
    yaws = yaw_grid()
    lines = [
        f"## {variant} L/R Delta",
        "",
        "| Vf \\ yaw | " + " | ".join(f"{yaw:.3g}" for yaw in yaws) + " |",
        "| ---: | " + " | ".join("---:" for _ in yaws) + " |",
    ]
    for vf in vf_grid():
        values = [f"{by_key[(vf, yaw)]:.3f}" for yaw in yaws]
        lines.append(f"| {vf:.3f} | " + " | ".join(values) + " |")
    lines.append("")
    return lines


def fmt(value: object, digits: int = 6) -> str:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return str(value)
    if math.isnan(f):
        return ""
    return f"{f:.{digits}f}"


def table_lines(frame: pd.DataFrame, columns: list[str], labels: list[str] | None = None, limit: int | None = None) -> list[str]:
    if limit is not None:
        frame = frame.head(limit)
    labels = labels or columns
    lines = ["| " + " | ".join(labels) + " |", "| " + " | ".join("---" for _ in labels) + " |"]
    for _, row in frame.iterrows():
        values = []
        for col in columns:
            value = row.get(col, "")
            if isinstance(value, (float, np.floating)):
                values.append(fmt(value))
            else:
                values.append(str(value))
        lines.append("| " + " | ".join(values) + " |")
    return lines


def make_report(
    model: HybridModel,
    tuning: pd.DataFrame,
    coeffs: pd.DataFrame,
    split_compare: pd.DataFrame,
    selected_compare: pd.DataFrame,
    risk: pd.DataFrame,
    command_grid: pd.DataFrame,
    in_place: pd.DataFrame,
    constants: dict[str, float],
) -> None:
    static_coeffs = coeffs[coeffs["branch"] == "static_launch"].copy()
    dominant = coeffs.head(14).copy()
    hybrid_grid = command_grid[command_grid["variant"] == "Hybrid_BC_adhesion_partition"]
    b_grid = command_grid[command_grid["variant"] == "B_stribeck"].set_index(["vf_mps", "yaw_rate_radps"])
    c_grid = command_grid[command_grid["variant"] == "C_combined_slip"].set_index(["vf_mps", "yaw_rate_radps"])
    h_grid = hybrid_grid.set_index(["vf_mps", "yaw_rate_radps"])
    diff_b = (h_grid["lr_delta_command"] - b_grid["lr_delta_command"]).abs()
    diff_c = (h_grid["lr_delta_command"] - c_grid["lr_delta_command"]).abs()

    lines: list[str] = [
        "# Round 2 Hybrid B/C Yaw Model",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not edited.",
        "",
        "## Reproduce",
        "",
        "```powershell",
        f"& '{RUNTIME_PYTHON}' codex_analysis\\yaw_model_variant_fits\\round2_hybrid_b_c\\fit_round2_hybrid_b_c.py",
        "```",
        "",
        "## Model Family",
        "",
        "The selected hybrid predicts yaw-opposing residual torque and converts it back to the raw additive yaw moment with:",
        "",
        "`M_raw_pred = -d_yaw * M_opp_pred`",
        "",
        "where `d_yaw = sign(yaw_rate)` with signed contact-velocity fallback only at near-zero yaw. The residual after correction is `M_raw_residual - M_raw_pred`.",
        "",
        "The fitted opposing torque is one coherent hybrid contact-force law with two coupled mechanisms:",
        "",
        "`M_opp_pred = G_slide * X_contact(v_contact, F_projected, load, utilization_actual) * beta_contact + B(v_y, N) * R(v_t) * (K_slide + K_static * exp(-(v_t / v_s)^2))`",
        "",
        "with:",
        "",
        f"- `v_t = sqrt((rel_weight * max(vbar_rel, load_weighted_rel))^2 + |Vf|^2)`, `rel_weight={model.config.rel_weight:.3f}`.",
        f"- `v_y = rel_weight * max(vbar_yaw, load_weighted_lat)` and `B(v_y, N) = (N / N_nominal) * (1 - exp(-(v_y / {model.config.bristle_velocity_mps:.3f} m/s)^2))` is the reduced bristle-deflection fill term.",
        f"- `R(v_t) = 1 / (1 + (v_t / {model.config.speed_fade_mps:.3f} m/s)^2)`.",
        f"- `v_s={model.config.stribeck_speed_mps:.3f} m/s` for the Stribeck fade.",
        f"- `G_slide = (1 - clamp((1 - exp(-(v_y / v_bristle)^2)) * exp(-(v_t / v_s)^2) * R(v_t), 0, 1))^{model.config.slide_gate_power:.1f}` fades the moving-contact surface while static adhesion is loaded.",
        "- `X_contact` is a force-state combined-slip basis: per-contact lateral/longitudinal velocity bases, projected contact yaw moment, projected contact-moment magnitude, actual force utilization, normal load delta, low relative speed, and high forward speed.",
        "",
        "## New Mechanism Versus A/C/D",
        "",
        "The rejected draft used requested yaw moment to load a static reservoir; that violates the rule that traction must not differ for the same physical contact state merely because command/request differs. The revised mechanism is a physical bristle-displacement static adhesion reservoir:",
        "",
        "`M_static_capacity = B(v_y, N) * R(v_t) * (K_slide + K_static * exp(-(v_t / v_s)^2))`",
        "",
        "This term can produce finite yaw-opposing torque at low contact speed because contact bristles deflect over relative displacement and load, then release continuously with transition speed. That is the missing degree of freedom in the low-order A/C/D-style residual surfaces: they only fit instantaneous residual surfaces, while this family adds a tire/contact micro-state approximation before sliding. No command, requested force, or mode label selects the traction law.",
        "",
        "The +1 rad/s in-place B launch torque calibrates the static reservoir magnitude at the reference physical contact state. The moving-contact coefficients are fit only from physical projected-force/contact-velocity features after subtracting the bristle branch. This is not a runtime mode label or residual lookup table.",
        "",
        "## Selected Hyperparameters",
        "",
    ]
    hp = pd.DataFrame(
        [
            {
                "vrel_knee_mps": model.config.vrel_knee_mps,
                "fwd_knee_mps": model.config.fwd_knee_mps,
                "bristle_velocity_mps": model.config.bristle_velocity_mps,
                "slide_gate_power": model.config.slide_gate_power,
                "moving_surface_source": "fit from physical projected-force/contact-velocity features",
                "anchor_extra_nm": model.anchor_extra_nm,
                "nominal_load_n": model.nominal_load_n,
                "static_branch_source": "Variant B Stribeck coefficient ratio scaled to physical velocity/load bristle basis",
            }
        ]
    )
    lines.extend(table_lines(hp, list(hp.columns)))
    lines.extend(["", "## Static Branch Coefficients", ""])
    lines.extend(
        table_lines(
            static_coeffs,
            ["feature", "standardized_coefficient_nm", "feature_scale", "raw_coefficient_nm_per_feature"],
            ["feature", "std coeff Nm", "scale", "raw coeff"],
        )
    )
    lines.extend(["", "## Dominant Coefficients", ""])
    lines.extend(
        table_lines(
            dominant,
            ["branch", "feature", "standardized_coefficient_nm", "feature_scale", "raw_coefficient_nm_per_feature"],
            ["branch", "feature", "std coeff Nm", "scale", "raw coeff"],
        )
    )
    lines.extend(["", "## +1 rad/s In-Place Command", ""])
    lines.extend(
        table_lines(
            in_place,
            [
                "variant",
                "extra_opposing_yaw_torque_nm",
                "total_opposing_yaw_torque_nm",
                "left_command",
                "right_command",
                "lr_delta_command",
                "max_abs_command",
                "in_place_acceptance_pass",
            ],
            [
                "variant",
                "extra Nm",
                "total opp Nm",
                "left cmd",
                "right cmd",
                "L-R delta",
                "max abs cmd",
                "gate",
            ],
        )
    )
    hybrid_in_place = in_place[in_place["variant"] == "Hybrid_BC_adhesion_partition"].iloc[0]
    gate_word = "PASS" if bool(hybrid_in_place["in_place_acceptance_pass"]) else "FAIL"
    lines.append("")
    lines.append(
        f"Hard gate at `Vf=0`, `Vr=0`, `yaw=+1 rad/s`: {gate_word}. Hybrid predicts left/right "
        f"`{float(hybrid_in_place['left_command']):.3f}/{float(hybrid_in_place['right_command']):.3f}`, "
        f"so `max |cmd|={float(hybrid_in_place['max_abs_command']):.3f}` versus the required "
        f"`>= {IN_PLACE_ACCEPT_MIN_ABS_COMMAND:.3f}`. The reference measured/calculated command is approximately "
        "`+0.646/-0.646`."
    )
    validation_row = split_compare[split_compare["split"] == "validation_non_authoritative"].iloc[0]
    c_validation = validation_row.get("c_corrected_rmse_nm")
    h_validation = validation_row.get("hybrid_corrected_rmse_nm")
    primary_row = split_compare[split_compare["split"] == "primary_open_floor_fit_authoritative"].iloc[0]
    lines.extend(["", "## Decision", ""])
    if bool(hybrid_in_place["in_place_acceptance_pass"]) and float(h_validation) <= 1.10 * float(c_validation):
        lines.append(
            "Candidate status: provisionally acceptable for further physical validation. It passes the in-place launch gate and stays within 10% of C on non-authoritative validation RMSE."
        )
    else:
        lines.append(
            "Candidate status: rejected as a production tune. It satisfies the no-command traction rule and passes the in-place launch gate, but it does not retain C-like broad residual performance."
        )
    lines.append(
        f"Primary fit RMSE is {float(primary_row['hybrid_corrected_rmse_nm']):.6f} Nm versus B {float(primary_row['b_corrected_rmse_nm']):.6f} and C {float(primary_row['c_corrected_rmse_nm']):.6f}. "
        f"Non-authoritative validation RMSE is {float(h_validation):.6f} Nm versus C {float(c_validation):.6f}."
    )
    lines.append(
        "The earlier request-loaded static reservoir is explicitly rejected because it changes traction for the same physical contact state when only command/request differs."
    )
    lines.extend(["", "## Split RMSE Versus B/C", ""])
    lines.extend(
        table_lines(
            split_compare,
            [
                "split",
                "count",
                "hybrid_baseline_rmse_nm",
                "b_corrected_rmse_nm",
                "c_corrected_rmse_nm",
                "hybrid_corrected_rmse_nm",
                "hybrid_vs_b_rmse_pct",
                "hybrid_vs_c_rmse_pct",
            ],
            [
                "split",
                "count",
                "baseline",
                "B RMSE",
                "C RMSE",
                "Hybrid RMSE",
                "Hybrid vs B",
                "Hybrid vs C",
            ],
        )
    )
    lines.extend(["", "## Selected Log RMSE Versus B/C", ""])
    lines.extend(
        table_lines(
            selected_compare,
            [
                "run_id",
                "dataset_split",
                "count",
                "hybrid_baseline_rmse_nm",
                "b_corrected_rmse_nm",
                "c_corrected_rmse_nm",
                "hybrid_corrected_rmse_nm",
                "hybrid_vs_b_rmse_pct",
                "hybrid_vs_c_rmse_pct",
            ],
            [
                "run",
                "split",
                "count",
                "baseline",
                "B RMSE",
                "C RMSE",
                "Hybrid RMSE",
                "Hybrid vs B",
                "Hybrid vs C",
            ],
        )
    )
    lines.extend(["", "## Risk Slices", ""])
    lines.extend(
        table_lines(
            risk,
            [
                "group",
                "count",
                "baseline_rmse_nm",
                "corrected_rmse_nm",
                "baseline_median_abs_nm",
                "corrected_median_abs_nm",
                "run_balanced_rmse_improvement_pct",
            ],
            ["group", "count", "baseline", "hybrid", "median abs before", "median abs after", "RB change"],
        )
    )
    lines.extend(["", "## 6x10 Vf/Yaw L-R Delta Grid Summary", ""])
    lines.append(
        f"Hybrid absolute L/R-delta difference from B over the grid: median {float(diff_b.median()):.3f}, p90 {float(diff_b.quantile(0.90)):.3f}."
    )
    lines.append(
        f"Hybrid absolute L/R-delta difference from C over the grid: median {float(diff_c.median()):.3f}, p90 {float(diff_c.quantile(0.90)):.3f}."
    )
    lines.append(
        f"Rows with `|cmd| > 1` for the hybrid grid: {int(hybrid_grid['command_outside_unit'].sum())} of {len(hybrid_grid)}; raw contact-utilization > 1: {int(hybrid_grid['contact_projection_sensitive'].sum())} of {len(hybrid_grid)}."
    )
    lines.append("")
    lines.extend(pivot_table_lines(command_grid, "Hybrid_BC_adhesion_partition"))
    lines.extend(["## Tuning Scores", ""])
    lines.extend(
        table_lines(
            tuning.head(12),
            [
                "objective_with_gate",
                "objective_score",
                "vrel_knee_mps",
                "fwd_knee_mps",
                "bristle_velocity_mps",
                "slide_gate_power",
                "ridge",
                "in_place_max_abs_command",
                "in_place_acceptance_pass",
                "validation_objective_rb_corrected_rmse_nm",
                "primary_rb_corrected_rmse_nm",
                "open_fit_rb_corrected_rmse_nm",
            ],
            [
                "gated obj",
                "objective",
                "k_rel",
                "k_fwd",
                "v_bristle",
                "gate_pow",
                "ridge",
                "1rad |cmd|",
                "gate",
                "validation RB",
                "primary RB",
                "open-fit RB",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Production Risks",
            "",
            "- The static reservoir calibration is derived from the B command inversion, not a direct production validation. It should be checked on explicit in-place launch logs before any production tune.",
            "- The near-zero-yaw direction fallback must be implemented as a continuous command/contact direction, not a raw noisy gyro sign branch.",
            "- Moving-contact terms depend on projected contact forces and actual force utilization. Any PlantModel force-projection change invalidates the coefficients and requires a refit.",
            "- Several high-yaw grid cells exceed unit command or raw contact utilization. Those cells need full force-projection replay before treating command magnitudes as feasible.",
            "- The model can add or subtract residual resistance through moving-contact coefficients. That improves broad RMSE but increases sign-convention risk if contact bases are transposed or direction conventions drift.",
            "- This fit uses open-floor/diagnostic feature exports only. Maze wall contact, fan-duty changes outside the sampled envelope, and high-performance maneuver transitions remain out-of-sample.",
            "",
            "## Output Files",
            "",
            "- `fit_round2_hybrid_b_c.py`",
            "- `hybrid_b_c_report.md`",
            "- `hybrid_model_coefficients.csv`",
            "- `selected_hyperparameters.json`",
            "- `candidate_tuning_scores.csv`",
            "- `split_metrics.csv`",
            "- `split_rmse_comparison_vs_b_c.csv`",
            "- `selected_log_metrics.csv`",
            "- `selected_log_rmse_comparison_vs_b_c.csv`",
            "- `risk_metrics.csv`",
            "- `in_place_1radps_command.csv`",
            "- `lr_delta_grid_hybrid_b_c.csv`",
            "- `commands_run.txt`",
        ]
    )
    (OUT / "hybrid_b_c_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    constants = read_constants()
    frame = load_rows(constants)
    nominal_load_n = selected_nominal_load(frame)
    anchor_base = baseline_opposing_yaw_torque(constants, 1.0)
    anchor_extra = variant_b_extra(anchor_base, constants, 0.0, 1.0)

    configs = [
        HybridConfig(vrel, 0.70, ridge, 0.0, 0.0, v_bristle, gate_power)
        for vrel in [0.05, 0.06, 0.08]
        for ridge in [3.0e-3, 1.0e-2, 3.0e-2, 1.0e-1]
        for v_bristle in [0.008, 0.012, 0.016, 0.024]
        for gate_power in [2.0, 4.0]
    ]

    tuning_rows: list[dict[str, object]] = []
    best_model: HybridModel | None = None
    best_score = float("inf")
    for config in configs:
        model = fit_model(frame, constants, config, anchor_extra, nominal_load_n)
        score = objective_score(model, frame)
        h_extra = hybrid_extra(model, constants, 0.0, 1.0)
        h_total = baseline_opposing_yaw_torque(constants, 1.0) + h_extra
        h_cmd = motor_commands_for_opposing_torque(h_total, constants, 0.0, 1.0)
        in_place_max_abs = max(abs(h_cmd["left_command"]), abs(h_cmd["right_command"]))
        in_place_pass = in_place_max_abs >= IN_PLACE_ACCEPT_MIN_ABS_COMMAND
        objective_with_gate = float(score["objective_score"])
        if not in_place_pass:
            objective_with_gate += 10.0 + (IN_PLACE_ACCEPT_MIN_ABS_COMMAND - in_place_max_abs)
        row = {
            "vrel_knee_mps": config.vrel_knee_mps,
            "fwd_knee_mps": config.fwd_knee_mps,
            "bristle_velocity_mps": config.bristle_velocity_mps,
            "slide_gate_power": config.slide_gate_power,
            "ridge": config.ridge,
            "static_ridge": config.static_ridge,
            "downweighted_fit_weight": config.downweighted_fit_weight,
            "anchor_extra_nm": anchor_extra,
            "in_place_max_abs_command": in_place_max_abs,
            "in_place_acceptance_pass": in_place_pass,
            "objective_with_gate": objective_with_gate,
            "anchor_constraint_lagrange": model.anchor_constraint_lagrange,
            **score,
        }
        tuning_rows.append(row)
        if objective_with_gate < best_score:
            best_score = objective_with_gate
            best_model = model

    if best_model is None:
        raise RuntimeError("no hybrid model fit")

    tuning = pd.DataFrame(tuning_rows).sort_values("objective_with_gate")
    coeffs = coefficient_frame(best_model)
    split = split_metrics(frame, best_model)
    selected = selected_log_metrics(frame, best_model)
    risk = risk_metrics(frame, best_model)
    split_compare = compare_split_metrics(split)
    selected_compare = compare_selected_metrics(selected)

    grid_rows: list[dict[str, object]] = []
    for vf in vf_grid():
        for yaw in yaw_grid():
            grid_rows.extend(command_rows_for_point(best_model, constants, vf, yaw))
    command_grid = pd.DataFrame(grid_rows)

    in_place_rows = command_rows_for_point(best_model, constants, 0.0, 1.0)
    in_place = pd.DataFrame(in_place_rows)

    write_frame(OUT / "candidate_tuning_scores.csv", tuning)
    write_frame(OUT / "hybrid_model_coefficients.csv", coeffs)
    write_frame(OUT / "split_metrics.csv", split)
    write_frame(OUT / "split_rmse_comparison_vs_b_c.csv", split_compare)
    write_frame(OUT / "selected_log_metrics.csv", selected)
    write_frame(OUT / "selected_log_rmse_comparison_vs_b_c.csv", selected_compare)
    write_frame(OUT / "risk_metrics.csv", risk)
    write_frame(OUT / "lr_delta_grid_hybrid_b_c.csv", command_grid)
    write_frame(OUT / "in_place_1radps_command.csv", in_place)

    hyperparameters = {
        "model": "Hybrid_BC_adhesion_partition",
        "vrel_knee_mps": best_model.config.vrel_knee_mps,
        "fwd_knee_mps": best_model.config.fwd_knee_mps,
        "bristle_velocity_mps": best_model.config.bristle_velocity_mps,
        "slide_gate_power": best_model.config.slide_gate_power,
        "ridge": best_model.config.ridge,
        "moving_surface_source": "fit from physical projected-force/contact-velocity features",
        "static_branch_source": "Variant B Stribeck coefficient ratio scaled to physical velocity/load bristle basis",
        "stribeck_speed_mps": best_model.config.stribeck_speed_mps,
        "speed_fade_mps": best_model.config.speed_fade_mps,
        "rel_weight": best_model.config.rel_weight,
        "anchor_vf_mps": 0.0,
        "anchor_yaw_rate_radps": 1.0,
        "anchor_extra_opposing_yaw_torque_nm": best_model.anchor_extra_nm,
        "nominal_load_n": best_model.nominal_load_n,
        "feature_count": len(best_model.feature_names),
        "direction_convention": "positive yaw clockwise; raw correction = -direction * opposing correction",
    }
    (OUT / "selected_hyperparameters.json").write_text(
        json.dumps(hyperparameters, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (OUT / "commands_run.txt").write_text(
        f"& '{RUNTIME_PYTHON}' codex_analysis\\yaw_model_variant_fits\\round2_hybrid_b_c\\fit_round2_hybrid_b_c.py\n",
        encoding="utf-8",
    )
    make_report(
        best_model,
        tuning,
        coeffs,
        split_compare,
        selected_compare,
        risk,
        command_grid,
        in_place,
        constants,
    )


if __name__ == "__main__":
    main()
