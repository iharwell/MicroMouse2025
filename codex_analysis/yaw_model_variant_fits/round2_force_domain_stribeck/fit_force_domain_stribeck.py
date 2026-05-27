#!/usr/bin/env python3
"""Force-domain rewrite of the prior Variant B Stribeck yaw scrub fit.

Analysis-only tooling. This script reads existing feature artifacts and writes
outputs only beside itself.
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
B_REFERENCE_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "round2_b_correct_branch_reference"
)
C_REFERENCE_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "combined_slip_surface"
)
IN_PLACE_REFERENCE = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "in_place_1radps_command"
    / "in_place_1radps_command_estimate.csv"
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
    "max_force_limiter_activity",
    "hardware_saturation_evidence",
    "gyro_derivative_spike",
    "residual_additive_yaw_torque_nm",
    "residual_opposes_yaw_nm",
    "patch_yaw_req_basis_nm",
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
]


@dataclass(frozen=True)
class Candidate:
    activation_source: str
    source_column: str
    yield_geometry: str
    yield_column: str
    equivalent_activation_nm: float
    utilization_activation: float
    stribeck_speed_mps: float
    speed_fade_mps: float
    rel_weight: float
    static_extra_nm: float
    sliding_nm: float
    weighted_train_opposes_rmse_nm: float
    primary_corrected_rmse_nm: float
    open_floor_validation_corrected_rmse_nm: float
    in_place_left_command: float
    in_place_right_command: float
    in_place_extra_opposing_yaw_torque_nm: float


def read_constants() -> dict[str, float]:
    table = pd.read_csv(CONSTANTS_INPUT)
    return {str(row.name): float(row.value) for row in table.itertuples(index=False)}


def sign_array(values: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    return (values > eps).astype(float) - (values < -eps).astype(float)


def sign(value: float, eps: float = 1.0e-6) -> float:
    return float((value > eps) - (value < -eps))


def smooth_positive(values: np.ndarray | float, epsilon: float = 1.0e-6) -> np.ndarray | float:
    return 0.5 * (values + np.sqrt(values * values + epsilon * epsilon))


def read_keyed_csv(path: Path, key: str) -> dict[str, dict[str, str]]:
    if not path.exists():
        return {}
    with path.open(newline="", encoding="utf-8") as handle:
        return {row[key]: row for row in csv.DictReader(handle)}


def load_frame(constants: dict[str, float]) -> tuple[pd.DataFrame, dict[str, float]]:
    frame = pd.read_csv(PRIMARY_INPUT, usecols=PRIMARY_COLUMNS)
    secondary = pd.read_csv(SECONDARY_INPUT, usecols=SECONDARY_COLUMNS)
    frame = frame.merge(secondary, how="left", on=["run_id", "row_index"])

    numeric_columns = [
        "row_index",
        "time_us",
        "forward_velocity_mps",
        "yaw_rate_radps",
        "measured_yaw_accel_radps2",
        "vbar_rel_mps",
        "vbar_yaw_mps",
        "max_force_limiter_activity",
        "hardware_saturation_evidence",
        "gyro_derivative_spike",
        "residual_additive_yaw_torque_nm",
        "residual_opposes_yaw_nm",
        "patch_yaw_req_basis_nm",
        "patch_yaw_force_basis_nm",
        "total_normal_load_n",
        "fl_normal_n",
        "fr_normal_n",
        "rl_normal_n",
        "rr_normal_n",
    ]
    for column in numeric_columns:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(
        subset=[
            "forward_velocity_mps",
            "yaw_rate_radps",
            "vbar_rel_mps",
            "vbar_yaw_mps",
            "residual_additive_yaw_torque_nm",
            "residual_opposes_yaw_nm",
            "patch_yaw_req_basis_nm",
            "patch_yaw_force_basis_nm",
        ]
    )

    primary = frame["dataset_split"] == "primary_open_floor_fit_authoritative"
    nominal_total_load = float(frame.loc[primary, "total_normal_load_n"].median())
    if not math.isfinite(nominal_total_load) or nominal_total_load <= 0.0:
        nominal_total_load = float(frame["total_normal_load_n"].median())
    if not math.isfinite(nominal_total_load) or nominal_total_load <= 0.0:
        nominal_total_load = constants["mass_kg"] * 9.80665 + constants.get("fan_downforce_full_duty_n", 0.0)

    frame["total_normal_load_n"] = frame["total_normal_load_n"].fillna(nominal_total_load)
    for contact in ["fl", "fr", "rl", "rr"]:
        frame[f"{contact}_normal_n"] = frame[f"{contact}_normal_n"].fillna(
            frame["total_normal_load_n"] / 4.0
        )

    mu_ref = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / nominal_total_load
    half_track = 0.5 * constants["track_width_m"]
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    total_normal = (
        frame["fl_normal_n"]
        + frame["fr_normal_n"]
        + frame["rl_normal_n"]
        + frame["rr_normal_n"]
    )
    frame["yield_longitudinal_moment_nm"] = mu_ref * half_track * total_normal
    frame["yield_full_yaw_moment_nm"] = mu_ref * math.hypot(half_track, longitudinal) * total_normal

    frame["abs_forward_velocity_mps"] = frame["forward_velocity_mps"].abs()
    frame["abs_yaw_rate_radps"] = frame["yaw_rate_radps"].abs()
    frame["yaw_sign"] = sign_array(frame["yaw_rate_radps"].to_numpy())
    zero_yaw = frame["yaw_sign"] == 0.0
    frame.loc[zero_yaw, "yaw_sign"] = sign_array(
        frame.loc[zero_yaw, "patch_yaw_req_basis_nm"].to_numpy()
    )
    frame["yaw_sign"] = frame["yaw_sign"].replace(0.0, 1.0)

    metadata = {
        "nominal_total_load_n": nominal_total_load,
        "mu_ref": mu_ref,
        "nominal_longitudinal_yield_nm": float(
            frame.loc[primary, "yield_longitudinal_moment_nm"].median()
        ),
        "nominal_full_yaw_yield_nm": float(frame.loc[primary, "yield_full_yaw_moment_nm"].median()),
        "rows": int(len(frame)),
        "runs": int(frame["run_id"].nunique()),
    }
    return frame, metadata


def training_weights(frame: pd.DataFrame) -> np.ndarray:
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

    limiter = np.clip(frame["max_force_limiter_activity"].to_numpy(), 0.0, 1.0)
    saturation = np.clip(frame["hardware_saturation_evidence"].to_numpy(), 0.0, 1.0)
    spike = np.clip(frame["gyro_derivative_spike"].to_numpy(), 0.0, 1.0)
    quality = (1.0 / (1.0 + 4.0 * limiter)) * (1.0 - 0.75 * saturation) * (
        1.0 - 0.75 * spike
    )
    quality = np.clip(quality, 0.02, 1.0)

    weights = base * quality
    fit_counts = frame.loc[weights > 0.0, "run_id"].value_counts()
    if not fit_counts.empty:
        run_scale = frame["run_id"].map(
            {run: 1.0 / math.sqrt(count) for run, count in fit_counts.items()}
        ).fillna(0.0)
        weights *= run_scale.to_numpy()
        positive = weights > 0.0
        weights[positive] *= positive.sum() / weights[positive].sum()
    return weights


def weighted_nnls(
    features: np.ndarray,
    target: np.ndarray,
    weights: np.ndarray,
    max_iter: int = 80,
    tolerance: float = 1.0e-11,
) -> np.ndarray:
    sqrt_w = np.sqrt(np.clip(weights, 0.0, None))
    xw = features * sqrt_w[:, None]
    yw = target * sqrt_w
    beta = np.zeros(features.shape[1], dtype=float)
    col_norm = np.sum(xw * xw, axis=0)
    col_norm = np.where(col_norm > 1.0e-18, col_norm, 1.0)
    residual = yw.copy()

    for _ in range(max_iter):
        max_delta = 0.0
        for col in range(features.shape[1]):
            old = beta[col]
            residual += xw[:, col] * old
            new = np.dot(xw[:, col], residual) / col_norm[col]
            if new < 0.0:
                new = 0.0
            beta[col] = new
            residual -= xw[:, col] * new
            max_delta = max(max_delta, abs(new - old))
        if max_delta < tolerance:
            break
    return beta


def build_basis(
    frame: pd.DataFrame,
    source_column: str,
    yield_column: str,
    utilization_activation: float,
    stribeck_speed_mps: float,
    speed_fade_mps: float,
    rel_weight: float,
) -> np.ndarray:
    source_moment = smooth_positive(frame[source_column].to_numpy(), epsilon=1.0e-6)
    yield_moment = np.maximum(frame[yield_column].to_numpy(), 1.0e-12)
    utilization = source_moment / yield_moment
    activation = 1.0 - np.exp(-np.square(utilization / utilization_activation))
    transition_speed = np.sqrt(
        np.square(rel_weight * frame["vbar_rel_mps"].to_numpy())
        + np.square(frame["abs_forward_velocity_mps"].to_numpy())
    )
    stribeck = np.exp(-np.square(transition_speed / stribeck_speed_mps))
    speed_relief = 1.0 / (1.0 + np.square(transition_speed / speed_fade_mps))
    return np.column_stack([activation * stribeck * speed_relief, activation * speed_relief])


def corrected_residuals(frame: pd.DataFrame, pred_opposes: np.ndarray) -> np.ndarray:
    pred_additive = -frame["yaw_sign"].to_numpy() * pred_opposes
    return frame["residual_additive_yaw_torque_nm"].to_numpy() - pred_additive


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else math.nan


def metric_row(frame: pd.DataFrame, pred_opposes: np.ndarray) -> dict[str, object]:
    baseline = frame["residual_additive_yaw_torque_nm"].to_numpy()
    corrected = corrected_residuals(frame, pred_opposes)
    baseline_rmse = rmse(baseline)
    corrected_rmse = rmse(corrected)
    baseline_mae = float(np.mean(np.abs(baseline))) if len(frame) else math.nan
    corrected_mae = float(np.mean(np.abs(corrected))) if len(frame) else math.nan
    return {
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
        "rmse_improvement_pct": 100.0 * (baseline_rmse - corrected_rmse) / baseline_rmse
        if baseline_rmse > 0.0
        else math.nan,
        "mae_improvement_pct": 100.0 * (baseline_mae - corrected_mae) / baseline_mae
        if baseline_mae > 0.0
        else math.nan,
        "median_predicted_opposing_scrub_nm": float(np.median(pred_opposes)) if len(frame) else math.nan,
        "median_residual_opposes_yaw_before_nm": float(
            np.median(frame["residual_opposes_yaw_nm"].to_numpy())
        )
        if len(frame)
        else math.nan,
        "median_residual_opposes_yaw_after_nm": float(
            np.median(-frame["yaw_sign"].to_numpy() * corrected)
        )
        if len(frame)
        else math.nan,
    }


def predict_candidate(frame: pd.DataFrame, candidate: Candidate) -> np.ndarray:
    basis = build_basis(
        frame,
        candidate.source_column,
        candidate.yield_column,
        candidate.utilization_activation,
        candidate.stribeck_speed_mps,
        candidate.speed_fade_mps,
        candidate.rel_weight,
    )
    return basis @ np.array([candidate.static_extra_nm, candidate.sliding_nm])


def fit_candidates(frame: pd.DataFrame, metadata: dict[str, float]) -> tuple[list[Candidate], Candidate, Candidate]:
    weights = training_weights(frame)
    target = frame["residual_opposes_yaw_nm"].to_numpy()
    source_defs = [
        ("request_moment_utilization", "patch_yaw_req_basis_nm"),
        ("projected_force_moment_utilization", "patch_yaw_force_basis_nm"),
    ]
    yield_defs = [
        ("longitudinal_moment_support", "yield_longitudinal_moment_nm"),
        ("full_yaw_moment_support", "yield_full_yaw_moment_nm"),
    ]
    equivalent_activation_nms = [0.006, 0.015, 0.025, 0.035, 0.050, 0.080]
    stribeck_speeds = [0.025, 0.050, 0.100]
    speed_fades = [0.16, 0.32, 0.64]
    rel_weights = [0.75, 1.25]
    candidates: list[Candidate] = []

    for activation_source, source_column in source_defs:
        for yield_geometry, yield_column in yield_defs:
            nominal_yield = float(frame.loc[
                frame["dataset_split"] == "primary_open_floor_fit_authoritative", yield_column
            ].median())
            for equivalent_activation_nm in equivalent_activation_nms:
                utilization_activation = equivalent_activation_nm / max(nominal_yield, 1.0e-12)
                for stribeck_speed_mps in stribeck_speeds:
                    for speed_fade_mps in speed_fades:
                        for rel_weight in rel_weights:
                            basis = build_basis(
                                frame,
                                source_column,
                                yield_column,
                                utilization_activation,
                                stribeck_speed_mps,
                                speed_fade_mps,
                                rel_weight,
                            )
                            beta = weighted_nnls(basis, target, weights)
                            pred = basis @ beta
                            positive = weights > 0.0
                            score = float(
                                np.sqrt(
                                    np.average(
                                        np.square((target - pred)[positive]),
                                        weights=weights[positive],
                                    )
                                )
                            )
                            corrected = corrected_residuals(frame, pred)
                            split = frame["dataset_split"].to_numpy()
                            primary_rmse = rmse(corrected[split == "primary_open_floor_fit_authoritative"])
                            validation_rmse = rmse(corrected[split == "open_floor_validation_only"])
                            in_place = force_domain_extra_and_command(
                                constants=read_constants(),
                                candidate_values={
                                    "activation_source": activation_source,
                                    "yield_geometry": yield_geometry,
                                    "equivalent_activation_nm": equivalent_activation_nm,
                                    "utilization_activation": utilization_activation,
                                    "stribeck_speed_mps": stribeck_speed_mps,
                                    "speed_fade_mps": speed_fade_mps,
                                    "rel_weight": rel_weight,
                                    "static_extra_nm": float(beta[0]),
                                    "sliding_nm": float(beta[1]),
                                },
                                metadata=metadata,
                                vf_mps=0.0,
                                yaw_rate=1.0,
                            )
                            candidates.append(
                                Candidate(
                                    activation_source=activation_source,
                                    source_column=source_column,
                                    yield_geometry=yield_geometry,
                                    yield_column=yield_column,
                                    equivalent_activation_nm=equivalent_activation_nm,
                                    utilization_activation=utilization_activation,
                                    stribeck_speed_mps=stribeck_speed_mps,
                                    speed_fade_mps=speed_fade_mps,
                                    rel_weight=rel_weight,
                                    static_extra_nm=float(beta[0]),
                                    sliding_nm=float(beta[1]),
                                    weighted_train_opposes_rmse_nm=score,
                                    primary_corrected_rmse_nm=primary_rmse,
                                    open_floor_validation_corrected_rmse_nm=validation_rmse,
                                    in_place_left_command=in_place["left_command"],
                                    in_place_right_command=in_place["right_command"],
                                    in_place_extra_opposing_yaw_torque_nm=in_place[
                                        "extra_opposing_yaw_torque_nm"
                                    ],
                                )
                            )

    projected_candidates = [
        c
        for c in candidates
        if c.activation_source == "projected_force_moment_utilization"
        and max(abs(c.in_place_left_command), abs(c.in_place_right_command)) >= 0.6
    ]
    if not projected_candidates:
        raise RuntimeError("no projected-force force-domain candidate passed the hard in-place gate")
    near_reference_candidates = [
        c for c in projected_candidates if abs(abs(c.in_place_left_command) - 0.646) <= 0.020
    ]
    selected_pool = near_reference_candidates if near_reference_candidates else projected_candidates
    selected = min(selected_pool, key=lambda c: c.weighted_train_opposes_rmse_nm)
    request_diagnostic = min(
        [c for c in candidates if c.activation_source == "request_moment_utilization"],
        key=lambda c: c.weighted_train_opposes_rmse_nm,
    )
    return candidates, selected, request_diagnostic


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


def command_torque_for_applied(
    applied_torque: float,
    wheel_speed_radps: float,
    constants: dict[str, float],
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


def nominal_yield_for_candidate(
    constants: dict[str, float],
    candidate_values: dict[str, float | str],
    metadata: dict[str, float],
) -> float:
    if candidate_values["yield_geometry"] == "full_yaw_moment_support":
        return metadata["nominal_full_yaw_yield_nm"]
    return metadata["nominal_longitudinal_yield_nm"]


def force_domain_extra(
    base_opposing: float,
    constants: dict[str, float],
    candidate_values: dict[str, float | str],
    metadata: dict[str, float],
    vf_mps: float,
    yaw_rate: float,
) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    vbar_rel = longitudinal * abs(yaw_rate)
    transition_speed = math.hypot(float(candidate_values["rel_weight"]) * vbar_rel, abs(vf_mps))
    stribeck = math.exp(-((transition_speed / float(candidate_values["stribeck_speed_mps"])) ** 2))
    speed_relief = 1.0 / (1.0 + (transition_speed / float(candidate_values["speed_fade_mps"])) ** 2)
    nominal_yield = nominal_yield_for_candidate(constants, candidate_values, metadata)

    extra = 0.0
    for _ in range(80):
        total_contact_moment = max(0.0, base_opposing + extra)
        if candidate_values["activation_source"] == "projected_force_moment_utilization":
            source_moment = min(total_contact_moment, nominal_yield)
        else:
            source_moment = total_contact_moment
        utilization = source_moment / max(nominal_yield, 1.0e-12)
        activation = 1.0 - math.exp(
            -((utilization / float(candidate_values["utilization_activation"])) ** 2)
        )
        next_extra = activation * speed_relief * (
            float(candidate_values["sliding_nm"])
            + float(candidate_values["static_extra_nm"]) * stribeck
        )
        if abs(next_extra - extra) < 1.0e-12:
            return next_extra
        extra = next_extra
    return extra


def candidate_values(candidate: Candidate) -> dict[str, float | str]:
    return {
        "activation_source": candidate.activation_source,
        "yield_geometry": candidate.yield_geometry,
        "equivalent_activation_nm": candidate.equivalent_activation_nm,
        "utilization_activation": candidate.utilization_activation,
        "stribeck_speed_mps": candidate.stribeck_speed_mps,
        "speed_fade_mps": candidate.speed_fade_mps,
        "rel_weight": candidate.rel_weight,
        "static_extra_nm": candidate.static_extra_nm,
        "sliding_nm": candidate.sliding_nm,
    }


def force_domain_extra_and_command(
    constants: dict[str, float],
    candidate_values: dict[str, float | str],
    metadata: dict[str, float],
    vf_mps: float,
    yaw_rate: float,
) -> dict[str, float]:
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    extra = force_domain_extra(base, constants, candidate_values, metadata, vf_mps, yaw_rate)
    total = base + extra
    command = motor_commands_for_opposing_torque(total, constants, vf_mps, yaw_rate)
    command.update(
        {
            "baseline_opposing_yaw_torque_nm": base,
            "extra_opposing_yaw_torque_nm": extra,
            "total_opposing_yaw_torque_nm": total,
            "nominal_yield_nm": nominal_yield_for_candidate(constants, candidate_values, metadata),
        }
    )
    return command


def vf_grid() -> list[float]:
    return [round(i * 0.15 / 5.0, 9) for i in range(6)]


def yaw_grid() -> list[float]:
    return [round(0.2 + i * (6.0 - 0.2) / 9.0, 9) for i in range(10)]


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


def candidate_to_row(candidate: Candidate, role: str) -> dict[str, object]:
    return {
        "role": role,
        "activation_source": candidate.activation_source,
        "yield_geometry": candidate.yield_geometry,
        "equivalent_activation_nm": candidate.equivalent_activation_nm,
        "utilization_activation": candidate.utilization_activation,
        "stribeck_speed_mps": candidate.stribeck_speed_mps,
        "speed_fade_mps": candidate.speed_fade_mps,
        "rel_weight": candidate.rel_weight,
        "static_extra_nm": candidate.static_extra_nm,
        "sliding_nm": candidate.sliding_nm,
        "weighted_train_opposes_rmse_nm": candidate.weighted_train_opposes_rmse_nm,
        "primary_corrected_rmse_nm": candidate.primary_corrected_rmse_nm,
        "open_floor_validation_corrected_rmse_nm": candidate.open_floor_validation_corrected_rmse_nm,
        "in_place_left_command": candidate.in_place_left_command,
        "in_place_right_command": candidate.in_place_right_command,
        "in_place_extra_opposing_yaw_torque_nm": candidate.in_place_extra_opposing_yaw_torque_nm,
    }


def write_model_outputs(
    frame: pd.DataFrame,
    constants: dict[str, float],
    metadata: dict[str, float],
    candidates: list[Candidate],
    selected: Candidate,
    request_diagnostic: Candidate,
) -> None:
    selected_pred = predict_candidate(frame, selected)
    split_rows = []
    for split, subset in frame.groupby("dataset_split", sort=True):
        idx = subset.index
        row = {"dataset_split": split}
        row.update(metric_row(subset, selected_pred[frame.index.get_indexer(idx)]))
        split_rows.append(row)
    write_csv(OUT / "split_rmse.csv", split_rows)

    selected_rows = []
    for run_id in SELECTED_RUNS:
        subset = frame[frame["run_id"] == run_id]
        row = {"run_id": run_id, "present": bool(len(subset))}
        if len(subset):
            idx = subset.index
            row.update(
                {
                    "dataset_split": ";".join(sorted(subset["dataset_split"].unique())),
                    "recommendation": ";".join(sorted(subset["recommendation"].unique())),
                    "family": ";".join(sorted(subset["family"].unique())),
                }
            )
            row.update(metric_row(subset, selected_pred[frame.index.get_indexer(idx)]))
        selected_rows.append(row)
    write_csv(OUT / "selected_log_rmse.csv", selected_rows)

    top_candidates = sorted(candidates, key=lambda c: c.weighted_train_opposes_rmse_nm)[:80]
    candidate_rows = [candidate_to_row(c, "grid_top") for c in top_candidates]
    candidate_rows.insert(0, candidate_to_row(selected, "selected_projected_force_domain"))
    candidate_rows.insert(1, candidate_to_row(request_diagnostic, "rejected_request_moment_command_gate"))
    write_csv(OUT / "candidate_tuning_scores.csv", candidate_rows)

    write_csv(
        OUT / "force_domain_coefficients.csv",
        [
            {"parameter": "activation_source", "value": selected.activation_source, "unit": "enum"},
            {"parameter": "yield_geometry", "value": selected.yield_geometry, "unit": "enum"},
            {
                "parameter": "nominal_longitudinal_yield_nm",
                "value": metadata["nominal_longitudinal_yield_nm"],
                "unit": "Nm",
            },
            {
                "parameter": "nominal_full_yaw_yield_nm",
                "value": metadata["nominal_full_yaw_yield_nm"],
                "unit": "Nm",
            },
            {
                "parameter": "equivalent_activation_nm",
                "value": selected.equivalent_activation_nm,
                "unit": "Nm at nominal load",
            },
            {
                "parameter": "utilization_activation",
                "value": selected.utilization_activation,
                "unit": "dimensionless",
            },
            {
                "parameter": "stribeck_speed_mps",
                "value": selected.stribeck_speed_mps,
                "unit": "m/s",
            },
            {"parameter": "speed_fade_mps", "value": selected.speed_fade_mps, "unit": "m/s"},
            {"parameter": "rel_weight", "value": selected.rel_weight, "unit": "dimensionless"},
            {"parameter": "static_extra_nm", "value": selected.static_extra_nm, "unit": "Nm"},
            {"parameter": "sliding_nm", "value": selected.sliding_nm, "unit": "Nm"},
            {
                "parameter": "weighted_train_opposes_rmse_nm",
                "value": selected.weighted_train_opposes_rmse_nm,
                "unit": "Nm",
            },
        ],
        ["parameter", "value", "unit"],
    )

    in_place_rows = []
    for name, candidate in [
        ("ForceDomainStribeck_projected_force", selected),
        ("RequestMomentDiagnostic_rejected_command_gate", request_diagnostic),
    ]:
        command = force_domain_extra_and_command(
            constants, candidate_values(candidate), metadata, vf_mps=0.0, yaw_rate=1.0
        )
        in_place_rows.append(
            {
                "variant": name,
                    "activation_source": candidate.activation_source,
                "baseline_opposing_yaw_torque_nm": command["baseline_opposing_yaw_torque_nm"],
                "extra_opposing_yaw_torque_nm": command["extra_opposing_yaw_torque_nm"],
                "total_opposing_yaw_torque_nm": command["total_opposing_yaw_torque_nm"],
                "nominal_yield_nm": command["nominal_yield_nm"],
                "left_command": command["left_command"],
                "right_command": command["right_command"],
                "lr_delta_command": command["lr_delta_command"],
                    "max_abs_command": max(abs(command["left_command"]), abs(command["right_command"])),
                }
            )
    if IN_PLACE_REFERENCE.exists():
        with IN_PLACE_REFERENCE.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                if row["variant"] in {"Variant B Stribeck scrub", "Variant C combined slip"}:
                    in_place_rows.append(
                        {
                            "variant": row["variant"],
                            "activation_source": "reference_artifact",
                            "baseline_opposing_yaw_torque_nm": "",
                            "extra_opposing_yaw_torque_nm": row["extra_opposing_yaw_torque_nm"],
                            "total_opposing_yaw_torque_nm": row["total_opposing_yaw_torque_nm"],
                            "nominal_yield_nm": "",
                            "left_command": row["left_command"],
                            "right_command": row["right_command"],
                            "lr_delta_command": float(row["left_command"]) - float(row["right_command"]),
                            "max_abs_command": max(abs(float(row["left_command"])), abs(float(row["right_command"]))),
                        }
                    )
    write_csv(OUT / "in_place_1radps_command.csv", in_place_rows)

    grid_rows = []
    selected_values = candidate_values(selected)
    for vf_mps in vf_grid():
        for yaw_rate in yaw_grid():
            command = force_domain_extra_and_command(
                constants, selected_values, metadata, vf_mps=vf_mps, yaw_rate=yaw_rate
            )
            grid_rows.append(
                {
                    "vf_mps": vf_mps,
                    "yaw_rate_radps": yaw_rate,
                    "variant": "ForceDomainStribeck_projected_force",
                    "baseline_opposing_yaw_torque_nm": command["baseline_opposing_yaw_torque_nm"],
                    "extra_opposing_yaw_torque_nm": command["extra_opposing_yaw_torque_nm"],
                    "total_opposing_yaw_torque_nm": command["total_opposing_yaw_torque_nm"],
                    "left_command": command["left_command"],
                    "right_command": command["right_command"],
                    "lr_delta_command": command["lr_delta_command"],
                    "left_surface_mps": command["left_surface_mps"],
                    "right_surface_mps": command["right_surface_mps"],
                    "nominal_yield_nm": command["nominal_yield_nm"],
                    "command_outside_unit": max(abs(command["left_command"]), abs(command["right_command"])) > 1.0,
                }
            )
    write_csv(OUT / "lr_delta_grid_6x10.csv", grid_rows)
    write_pivot(OUT / "lr_delta_pivot.md", grid_rows)

    write_comparisons(split_rows, selected_rows)
    write_metadata(metadata, selected, request_diagnostic)
    write_report(metadata, selected, request_diagnostic)


def write_pivot(path: Path, rows: list[dict[str, object]]) -> None:
    yaws = yaw_grid()
    by_key = {(float(row["vf_mps"]), float(row["yaw_rate_radps"])): row for row in rows}
    lines = [
        "# Force-Domain Stribeck L-R Delta Command Grid",
        "",
        "Values are `left_command - right_command` for positive clockwise yaw.",
        "",
        "| Vf m/s | " + " | ".join(f"{yaw:.3f}" for yaw in yaws) + " |",
        "| ---: | " + " | ".join("---:" for _ in yaws) + " |",
    ]
    for vf_mps in vf_grid():
        values = [f"{float(by_key[(vf_mps, yaw)]['lr_delta_command']):.6f}" for yaw in yaws]
        lines.append(f"| {vf_mps:.3f} | " + " | ".join(values) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_comparisons(split_rows: list[dict[str, object]], selected_rows: list[dict[str, object]]) -> None:
    b_split = read_keyed_csv(B_REFERENCE_DIR / "split_rmse.csv", "dataset_split")
    c_split = read_keyed_csv(C_REFERENCE_DIR / "split_metrics.csv", "group")
    comparison_rows = []
    for row in split_rows:
        split = str(row["dataset_split"])
        b = b_split.get(split, {})
        c = c_split.get(split, {})
        force_rmse = float(row["corrected_rmse_nm"])
        b_rmse = float(b["corrected_rmse_nm"]) if b else math.nan
        c_rmse = float(c["corrected_rmse_nm"]) if c else math.nan
        comparison_rows.append(
            {
                "dataset_split": split,
                "count": row["count"],
                "baseline_rmse_nm": row["baseline_rmse_nm"],
                "force_domain_corrected_rmse_nm": force_rmse,
                "variant_b_corrected_rmse_nm": b_rmse,
                "variant_c_corrected_rmse_nm": c_rmse,
                "force_domain_minus_b_rmse_nm": force_rmse - b_rmse if math.isfinite(b_rmse) else "",
                "force_domain_minus_c_rmse_nm": force_rmse - c_rmse if math.isfinite(c_rmse) else "",
                "force_domain_improvement_pct": row["rmse_improvement_pct"],
            }
        )
    write_csv(OUT / "split_rmse_comparison.csv", comparison_rows)

    b_selected = read_keyed_csv(B_REFERENCE_DIR / "selected_log_rmse.csv", "run_id")
    c_selected = read_keyed_csv(C_REFERENCE_DIR / "selected_log_metrics.csv", "run_id")
    selected_comparison_rows = []
    for row in selected_rows:
        run_id = str(row["run_id"])
        b = b_selected.get(run_id, {})
        c = c_selected.get(run_id, {})
        force_rmse = float(row["corrected_rmse_nm"]) if row.get("present") else math.nan
        b_rmse = float(b["corrected_rmse_nm"]) if b else math.nan
        c_rmse = float(c["corrected_rmse_nm"]) if c else math.nan
        selected_comparison_rows.append(
            {
                "run_id": run_id,
                "present": row.get("present", False),
                "dataset_split": row.get("dataset_split", ""),
                "count": row.get("count", 0),
                "baseline_rmse_nm": row.get("baseline_rmse_nm", ""),
                "force_domain_corrected_rmse_nm": force_rmse,
                "variant_b_corrected_rmse_nm": b_rmse,
                "variant_c_corrected_rmse_nm": c_rmse,
                "force_domain_minus_b_rmse_nm": force_rmse - b_rmse if math.isfinite(b_rmse) else "",
                "force_domain_minus_c_rmse_nm": force_rmse - c_rmse if math.isfinite(c_rmse) else "",
            }
        )
    write_csv(OUT / "selected_log_rmse_comparison.csv", selected_comparison_rows)


def write_metadata(metadata: dict[str, float], selected: Candidate, request_diagnostic: Candidate) -> None:
    payload = {
        "inputs": {
            "primary": str(PRIMARY_INPUT.relative_to(ROOT)),
            "secondary": str(SECONDARY_INPUT.relative_to(ROOT)),
            "constants": str(CONSTANTS_INPUT.relative_to(ROOT)),
        },
        "metadata": metadata,
        "selected": candidate_to_row(selected, "selected_projected_force_domain"),
        "rejected_request_moment_diagnostic": candidate_to_row(
            request_diagnostic, "rejected_request_moment_command_gate"
        ),
        "production_code_edited": False,
        "build_metadata_edited": False,
        "tests_edited": False,
    }
    (OUT / "reference_metadata.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def fmt(value: object, digits: int = 6) -> str:
    try:
        x = float(value)
        if math.isnan(x):
            return ""
        return f"{x:.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def markdown_table(rows: list[dict[str, object]], columns: list[str], labels: list[str] | None = None, limit: int | None = None) -> list[str]:
    if labels is None:
        labels = columns
    if limit is not None:
        rows = rows[:limit]
    lines = ["| " + " | ".join(labels) + " |", "| " + " | ".join("---" for _ in labels) + " |"]
    for row in rows:
        values = []
        for col in columns:
            value = row.get(col, "")
            if isinstance(value, float):
                values.append(fmt(value))
            else:
                values.append(str(value))
        lines.append("| " + " | ".join(values) + " |")
    return lines


def write_report(metadata: dict[str, float], selected: Candidate, request_diagnostic: Candidate) -> None:
    coefficients = list(csv.DictReader((OUT / "force_domain_coefficients.csv").open(newline="", encoding="utf-8")))
    in_place = list(csv.DictReader((OUT / "in_place_1radps_command.csv").open(newline="", encoding="utf-8")))
    split_comparison = list(csv.DictReader((OUT / "split_rmse_comparison.csv").open(newline="", encoding="utf-8")))
    selected_comparison = list(csv.DictReader((OUT / "selected_log_rmse_comparison.csv").open(newline="", encoding="utf-8")))
    grid_rows = list(csv.DictReader((OUT / "lr_delta_grid_6x10.csv").open(newline="", encoding="utf-8")))

    force_in_place = next(row for row in in_place if row["variant"] == "ForceDomainStribeck_projected_force")
    request_in_place = next(
        row for row in in_place if row["variant"] == "RequestMomentDiagnostic_rejected_command_gate"
    )

    lines: list[str] = [
        "# Round2 Force-Domain Stribeck Rewrite",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not modified.",
        "",
        "## Decision",
        "",
        "The physically acceptable rewrite is the projected-force utilization form: it keeps Variant B's static-to-sliding Stribeck torque law, but replaces raw request/command activation with yaw-moment utilization computed from the projected/actual tire contact force state and normal-load-derived yaw-moment yield.",
        "",
        f"At `Vf=0`, `Vr=0`, `yawRate=+1 rad/s`, it predicts left/right command `{fmt(force_in_place['left_command'], 12)}/{fmt(force_in_place['right_command'], 12)}` with extra opposing yaw torque `{fmt(force_in_place['extra_opposing_yaw_torque_nm'], 12)}` Nm. That passes the hard `|cmd| >= 0.6` gate and stays near the prior B target of about `+0.646/-0.646`.",
        "",
        "## Equations",
        "",
        "For each contact `i`, with lateral/right coordinate `r_i`, longitudinal/forward coordinate `f_i`, normal load `N_i`, and projected/actual contact force `(F_f,i, F_r,i)`:",
        "",
        "`M_contact = sum_i(f_i * F_r,i - r_i * F_f,i)`",
        "",
        "`M_drive = smooth_positive(sign(yawRate) * M_contact)`",
        "",
        "`M_yield = mu_ref * sum_i(|r_i| * N_i)` for the selected longitudinal/differential-drive yaw-moment support.",
        "",
        "`u = M_drive / M_yield`",
        "",
        "`A_u = 1 - exp(-(u / u_activation)^2)`",
        "",
        "`v_transition = sqrt((rel_weight * vbar_rel)^2 + |Vf|^2)`",
        "",
        "`S(v) = exp(-(v_transition / stribeck_speed)^2)`",
        "",
        "`R(v) = 1 / (1 + (v_transition / speed_fade)^2)`",
        "",
        "`M_extra = A_u * R(v) * (K_slide + K_static * S(v))`",
        "",
        "For command estimates, `M_extra` is solved as a fixed point because the projected contact force needed to hold the same contact state includes the extra scrub. The synthetic in-place/grid evaluator caps the projected source moment at the nominal contact yaw-moment yield.",
        "",
        "## Fitted Parameters",
        "",
    ]
    lines.extend(markdown_table(coefficients, ["parameter", "value", "unit"]))
    lines.extend(
        [
            "",
            f"The utilization knee is equivalent to `{selected.equivalent_activation_nm:.6g} Nm` at nominal load, expressed as `u_activation = equivalent_activation_nm / M_yield_nominal`.",
            "",
            "## +1 rad/s In-Place",
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
            ],
        )
    )
    lines.extend(
        [
            "",
            "The request-moment diagnostic is not the selected rewrite because command/request gates are prohibited. It is retained only to show how close the rejected algebraic B branch remains.",
            "",
            "## 6x10 L-R Delta Grid",
            "",
            "Full machine-readable grid: `lr_delta_grid_6x10.csv`. Pivot:",
            "",
        ]
    )
    lines.extend((OUT / "lr_delta_pivot.md").read_text(encoding="utf-8").splitlines()[4:])
    lines.extend(
        [
            "",
            "## Split RMSE Versus B/C",
            "",
        ]
    )
    lines.extend(
        markdown_table(
            split_comparison,
            [
                "dataset_split",
                "count",
                "baseline_rmse_nm",
                "force_domain_corrected_rmse_nm",
                "variant_b_corrected_rmse_nm",
                "variant_c_corrected_rmse_nm",
                "force_domain_minus_b_rmse_nm",
                "force_domain_minus_c_rmse_nm",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Selected-Log RMSE Versus B/C",
            "",
        ]
    )
    lines.extend(
        markdown_table(
            selected_comparison,
            [
                "run_id",
                "dataset_split",
                "count",
                "baseline_rmse_nm",
                "force_domain_corrected_rmse_nm",
                "variant_b_corrected_rmse_nm",
                "variant_c_corrected_rmse_nm",
                "force_domain_minus_b_rmse_nm",
                "force_domain_minus_c_rmse_nm",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## No-Command-Conditioning Argument",
            "",
            "The selected law contains no left/right command, unit command, requested command scalar, pre-projection force request, or mode-local command branch. Its activation inputs are contact-relative speed, normal load, and the projected/actual contact force vector reduced to a yaw-moment utilization.",
            "",
            "Therefore, if two cases have the same contact state `{v_rel_i, N_i, yawRate}` and the same projected tire/contact force state `{F_f,i, F_r,i}`, they produce the same `M_contact`, `M_yield`, `u`, Stribeck schedule, and `M_extra`, regardless of which upstream command representation happened to produce that force state. Command can still affect motor torque and thus the physical contact force state; it is not an independent traction selector.",
            "",
            "The pre-projection request branch is rejected here even though the initial task allowed it as a possible analysis input. With the clarification that command gates are prohibited, projected/actual contact force is the only acceptable selector among the two fitted branches.",
            "",
            "## Failure Modes",
            "",
            "- The projected-force form is solve-order sensitive: it must run after, or be coupled with, contact projection. Evaluating it from pre-projection demand would reintroduce the prohibited command/request gate.",
            "- The law inherits Variant B's one-sided positive contact-force branch. A production form should make the sign convention explicit and symmetric rather than depending on a logged feature name.",
            "- The 6x10 grid is an algebraic command estimate, not a full force replay. High-yaw cells can require commands outside `[-1, 1]`, so those cells are feasibility warnings, not validated reachable behavior.",
            "- The model fits low-speed yaw scrub and intentionally fades with forward/contact speed. It is not a replacement for the broader Variant C combined-slip surface, which still has better validation and diagnostic split RMSE.",
            "- The selected fit is target-aware after the hard physical gate: among projected-force candidates that pass `|cmd| >= 0.6`, it prefers candidates within `0.020` command of the `0.646` reference before minimizing RMSE. Production should still pick the yield owner deliberately rather than baking this analysis helper into runtime.",
            "",
            "## Reproduce",
            "",
            "```powershell",
            "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' codex_analysis\\yaw_model_variant_fits\\round2_force_domain_stribeck\\fit_force_domain_stribeck.py",
            "```",
            "",
            "## Output Files",
            "",
            "- `fit_force_domain_stribeck.py`",
            "- `force_domain_stribeck_report.md`",
            "- `force_domain_coefficients.csv`",
            "- `candidate_tuning_scores.csv`",
            "- `in_place_1radps_command.csv`",
            "- `lr_delta_grid_6x10.csv`",
            "- `lr_delta_pivot.md`",
            "- `split_rmse.csv`",
            "- `selected_log_rmse.csv`",
            "- `split_rmse_comparison.csv`",
            "- `selected_log_rmse_comparison.csv`",
            "- `reference_metadata.json`",
            "- `commands_run.txt`",
        ]
    )
    (OUT / "force_domain_stribeck_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    constants = read_constants()
    frame, metadata = load_frame(constants)
    candidates, selected, request_diagnostic = fit_candidates(frame, metadata)
    write_model_outputs(frame, constants, metadata, candidates, selected, request_diagnostic)
    (OUT / "commands_run.txt").write_text(
        "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' "
        "codex_analysis\\yaw_model_variant_fits\\round2_force_domain_stribeck\\fit_force_domain_stribeck.py\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
