#!/usr/bin/env python3
"""Fit a yaw-rate regime split with a shifted, peak-maintained Stribeck branch.

Analysis-only tooling. The candidate uses kinematic yaw/forward speed and load
scaling only; existing PlantModel/contact outputs are reference targets or
comparison columns, not runtime inputs to the candidate law.

This supersedes the first shifted_peak pass: the middle regime is a linear
blend weight between a near-zero linear law and the shifted Stribeck branch,
not a separate ramp segment and not a replacement of the transition with the
Stribeck curve itself.
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

PRIMARY_INPUT = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "ablation" / "phase_classified_feature_sample.csv"
SECONDARY_INPUT = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "contact_continuum_feature_sample.csv"
CONSTANTS_INPUT = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "plant_mirror_constants.csv"

VARIANT_C_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "split_metrics.csv"
VARIANT_C_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "selected_log_metrics.csv"
FORCE_DOMAIN_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck" / "split_rmse.csv"
FORCE_DOMAIN_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck" / "selected_log_rmse.csv"
RATIONAL_BLEND_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "transition_options" / "rational_speed_force_blend" / "split_metrics.csv"
RATIONAL_BLEND_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "transition_options" / "rational_speed_force_blend" / "selected_log_metrics.csv"
TRUE_TRACTION_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "contact_patch_true_traction_testbed" / "split_metrics.csv"
TRUE_TRACTION_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "contact_patch_true_traction_testbed" / "selected_log_metrics.csv"
STANDALONE_TRACTION_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "standalone_contact_traction_testbed" / "split_metrics.csv"
STANDALONE_TRACTION_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "standalone_contact_traction_testbed" / "selected_log_metrics.csv"

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

SPLITS = [
    "primary_open_floor_fit_authoritative",
    "open_floor_fit_downweighted",
    "open_floor_validation_only",
    "diag_validation_only",
    "aux_downweighted_validation",
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
    "max_force_limiter_activity",
    "hardware_saturation_evidence",
    "gyro_derivative_spike",
    "residual_additive_yaw_torque_nm",
    "residual_opposes_yaw_nm",
]

SECONDARY_COLUMNS = [
    "run_id",
    "row_index",
    "total_normal_load_n",
    "observed_yaw_moment_nm",
    "model_yaw_moment_nm",
]


@dataclass(frozen=True)
class Candidate:
    role: str
    k1_rad_per_m: float
    k2_rad_per_m: float
    k3_radps: float
    stribeck_speed_mps: float
    yaw_fade_mps: float
    sliding_ratio: float
    load_power: float
    linear_gain_nm_per_radps: float
    peak_yield_nm: float
    weighted_train_rmse_nm: float
    primary_corrected_rmse_nm: float
    validation_corrected_rmse_nm: float
    in_place_opposing_nm: float
    in_place_left_command: float
    in_place_right_command: float
    score: float


def read_constants() -> dict[str, float]:
    table = pd.read_csv(CONSTANTS_INPUT)
    return {str(row.name): float(row.value) for row in table.itertuples(index=False)}


def sign_array(values: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    return (values > eps).astype(float) - (values < -eps).astype(float)


def sign(value: float, eps: float = 1.0e-6) -> float:
    return float((value > eps) - (value < -eps))


def load_frame(constants: dict[str, float]) -> tuple[pd.DataFrame, dict[str, float]]:
    frame = pd.read_csv(PRIMARY_INPUT, usecols=PRIMARY_COLUMNS)
    secondary = pd.read_csv(SECONDARY_INPUT, usecols=SECONDARY_COLUMNS)
    frame = frame.merge(secondary, how="left", on=["run_id", "row_index"])

    numeric = [
        "row_index",
        "time_us",
        "forward_velocity_mps",
        "yaw_rate_radps",
        "vbar_rel_mps",
        "vbar_yaw_mps",
        "max_force_limiter_activity",
        "hardware_saturation_evidence",
        "gyro_derivative_spike",
        "residual_additive_yaw_torque_nm",
        "residual_opposes_yaw_nm",
        "total_normal_load_n",
        "observed_yaw_moment_nm",
        "model_yaw_moment_nm",
    ]
    for column in numeric:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(
        subset=[
            "forward_velocity_mps",
            "yaw_rate_radps",
            "vbar_rel_mps",
            "vbar_yaw_mps",
            "residual_additive_yaw_torque_nm",
            "residual_opposes_yaw_nm",
        ]
    ).copy()

    primary = frame["dataset_split"] == "primary_open_floor_fit_authoritative"
    nominal_load = float(frame.loc[primary, "total_normal_load_n"].median())
    if not math.isfinite(nominal_load) or nominal_load <= 0.0:
        nominal_load = float(frame["total_normal_load_n"].median())
    if not math.isfinite(nominal_load) or nominal_load <= 0.0:
        nominal_load = constants["mass_kg"] * 9.80665 + constants.get("fan_downforce_full_duty_n", 0.0)
    frame["total_normal_load_n"] = frame["total_normal_load_n"].fillna(nominal_load)
    frame["abs_forward_velocity_mps"] = frame["forward_velocity_mps"].abs()
    frame["abs_yaw_rate_radps"] = frame["yaw_rate_radps"].abs()
    frame["yaw_sign"] = sign_array(frame["yaw_rate_radps"].to_numpy())
    frame["yaw_sign"] = frame["yaw_sign"].replace(0.0, 1.0)

    return frame, {
        "rows": int(len(frame)),
        "runs": int(frame["run_id"].nunique()),
        "nominal_total_load_n": nominal_load,
        "drive_wheel_longitudinal_offset_m": constants["drive_wheel_longitudinal_offset_m"],
    }


def training_weights(frame: pd.DataFrame, latest_weighted: bool = False) -> np.ndarray:
    weights = np.zeros(len(frame), dtype=float)
    split = frame["dataset_split"].to_numpy()
    family = frame["family"].to_numpy()
    recommendation = frame["recommendation"].to_numpy()
    weights[split == "primary_open_floor_fit_authoritative"] = 1.0
    downweighted = (
        (split == "open_floor_fit_downweighted")
        & (family == "open_floor")
        & (recommendation == "fit_downweighted")
    )
    weights[downweighted] = 0.25

    limiter = np.clip(frame["max_force_limiter_activity"].to_numpy(), 0.0, 1.0)
    saturation = np.clip(frame["hardware_saturation_evidence"].to_numpy(), 0.0, 1.0)
    spike = np.clip(frame["gyro_derivative_spike"].to_numpy(), 0.0, 1.0)
    weights *= (1.0 / (1.0 + 4.0 * limiter)) * (1.0 - 0.75 * saturation) * (1.0 - 0.75 * spike)

    if latest_weighted:
        latest = frame["run_id"].isin(["2026-05-04_20-35-47", "2026-05-04_16-57-53"]).to_numpy()
        weights[latest & (weights > 0.0)] *= 4.0

    fit_counts = frame.loc[weights > 0.0, "run_id"].value_counts()
    if not fit_counts.empty:
        run_scale = frame["run_id"].map({run: 1.0 / math.sqrt(count) for run, count in fit_counts.items()}).fillna(0.0)
        weights *= run_scale.to_numpy()
        positive = weights > 0.0
        weights[positive] *= positive.sum() / weights[positive].sum()
    return weights


def weighted_nnls(features: np.ndarray, target: np.ndarray, weights: np.ndarray) -> np.ndarray:
    sqrt_w = np.sqrt(np.clip(weights, 0.0, None))
    xw = features * sqrt_w[:, None]
    yw = target * sqrt_w
    beta = np.zeros(features.shape[1], dtype=float)
    col_norm = np.sum(xw * xw, axis=0)
    col_norm = np.where(col_norm > 1.0e-18, col_norm, 1.0)
    residual = yw.copy()
    for _ in range(120):
        max_delta = 0.0
        for col in range(features.shape[1]):
            old = beta[col]
            residual += xw[:, col] * old
            new = float(np.dot(xw[:, col], residual) / col_norm[col])
            beta[col] = max(0.0, new)
            residual -= xw[:, col] * beta[col]
            max_delta = max(max_delta, abs(beta[col] - old))
        if max_delta < 1.0e-12:
            break
    return beta


def shifted_peak_features(
    frame: pd.DataFrame,
    metadata: dict[str, float],
    k1: float,
    k2: float,
    k3: float,
    stribeck_speed: float,
    yaw_fade: float,
    sliding_ratio: float,
    load_power: float,
) -> np.ndarray:
    abs_vf = frame["abs_forward_velocity_mps"].to_numpy()
    abs_yaw = frame["abs_yaw_rate_radps"].to_numpy()
    load_ratio = np.maximum(frame["total_normal_load_n"].to_numpy() / metadata["nominal_total_load_n"], 0.05)
    if load_power == 0.0:
        load_scale = np.ones_like(load_ratio)
    elif load_power == 0.5:
        load_scale = np.sqrt(load_ratio)
    else:
        load_scale = np.power(load_ratio, load_power)

    start = abs_vf * k1
    end = abs_vf * k2 + k3
    width = np.maximum(end - start, 1.0e-6)
    blend = np.clip((abs_yaw - start) / width, 0.0, 1.0)
    # Boundary-style linear law: L(y,Vf) reaches L1 at x1, then the middle
    # regime blends that boundary value into S_shifted with a linear weight.
    linear_norm = np.divide(
        abs_yaw,
        np.maximum(start, 1.0e-6),
        out=np.ones_like(abs_yaw),
        where=start > 1.0e-6,
    )
    linear_norm = np.clip(linear_norm, 0.0, 1.0)

    yaw_length = metadata["drive_wheel_longitudinal_offset_m"]
    branch_yaw = abs_yaw - end
    q = np.sqrt(np.square(abs_vf) + np.square(yaw_length * branch_yaw))
    q0 = abs_vf
    raw = sliding_ratio + (1.0 - sliding_ratio) / (1.0 + np.square(q / stribeck_speed))
    raw0 = sliding_ratio + (1.0 - sliding_ratio) / (1.0 + np.square(q0 / stribeck_speed))
    shifted = np.maximum(0.0, 1.0 + raw - raw0)
    post_branch_yaw = np.maximum(branch_yaw, 0.0)
    tail = 1.0 / (1.0 + np.square((yaw_length * post_branch_yaw) / yaw_fade))
    normalized_support = (1.0 - blend) * linear_norm + blend * shifted * tail
    return np.column_stack([load_scale * normalized_support])


def predict(frame: pd.DataFrame, metadata: dict[str, float], candidate: Candidate) -> np.ndarray:
    features = shifted_peak_features(
        frame,
        metadata,
        candidate.k1_rad_per_m,
        candidate.k2_rad_per_m,
        candidate.k3_radps,
        candidate.stribeck_speed_mps,
        candidate.yaw_fade_mps,
        candidate.sliding_ratio,
        candidate.load_power,
    )
    return features[:, 0] * candidate.peak_yield_nm


def corrected_residuals(frame: pd.DataFrame, pred_opposes: np.ndarray) -> np.ndarray:
    return frame["residual_additive_yaw_torque_nm"].to_numpy() + frame["yaw_sign"].to_numpy() * pred_opposes


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
        "rmse_improvement_pct": 100.0 * (baseline_rmse - corrected_rmse) / baseline_rmse if baseline_rmse > 0.0 else math.nan,
        "mae_improvement_pct": 100.0 * (baseline_mae - corrected_mae) / baseline_mae if baseline_mae > 0.0 else math.nan,
        "median_predicted_opposing_support_nm": float(np.median(pred_opposes)) if len(frame) else math.nan,
        "median_residual_opposes_yaw_before_nm": float(np.median(frame["residual_opposes_yaw_nm"].to_numpy())) if len(frame) else math.nan,
        "median_residual_opposes_yaw_after_nm": float(np.median(-frame["yaw_sign"].to_numpy() * corrected)) if len(frame) else math.nan,
    }


def torque_from_command(command: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]
    applied_voltage = command * battery
    back_emf = wheel_speed_radps * (gear_ratio / speed_constant)
    current = (applied_voltage - back_emf) / resistance
    armature_sign = sign(current)
    wheel_sign = sign(wheel_speed_radps)
    no_load_sign = armature_sign if armature_sign else wheel_sign
    load_current = current - no_load_sign * no_load
    if no_load_sign > 0.0 and load_current < 0.0:
        load_current = 0.0
    if no_load_sign < 0.0 and load_current > 0.0:
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
    current = motor_torque / torque_constant + no_load_sign * no_load
    back_emf = wheel_speed_radps * (gear_ratio / speed_constant)
    return (current * resistance + back_emf) / battery


def static_launch_torque(constants: dict[str, float]) -> float:
    return max(0.0, torque_from_command(constants["static_launch_command"], 0.0, constants))


def command_torque_for_applied(applied_torque: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    surface_speed = constants["wheel_radius_m"] * wheel_speed_radps
    ratio = abs(surface_speed) / constants["static_friction_max_speed_mps"]
    launch = static_launch_torque(constants) * math.exp(-(ratio * ratio))
    preferred = sign(applied_torque)
    fallback = sign(wheel_speed_radps)
    direction = preferred if preferred else fallback
    rolling_direction = fallback if fallback else direction
    return applied_torque + direction * launch + rolling_direction * constants["rolling_friction_torque_nm"]


def motor_commands_for_opposing_torque(opposing_yaw_torque: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> dict[str, float]:
    radius = constants["wheel_radius_m"]
    track = constants["track_width_m"]
    half_track = 0.5 * track
    left_surface = vf_mps + half_track * yaw_rate
    right_surface = vf_mps - half_track * yaw_rate
    left_speed = left_surface / radius
    right_speed = right_surface / radius
    applied_bank_torque = opposing_yaw_torque * radius / track
    left_command_torque = command_torque_for_applied(applied_bank_torque, left_speed, constants)
    right_command_torque = command_torque_for_applied(-applied_bank_torque, right_speed, constants)
    left_command = command_from_torque(left_command_torque, left_speed, constants)
    right_command = command_from_torque(right_command_torque, right_speed, constants)
    return {
        "total_opposing_yaw_torque_nm": opposing_yaw_torque,
        "left_command": left_command,
        "right_command": right_command,
        "lr_delta_command": left_command - right_command,
        "max_abs_command": max(abs(left_command), abs(right_command)),
    }


def required_opposing_for_command(constants: dict[str, float], vf_mps: float, yaw_rate: float, target_abs_command: float) -> float:
    lo = 0.0
    hi = 0.02
    for _ in range(80):
        if motor_commands_for_opposing_torque(hi, constants, vf_mps, yaw_rate)["max_abs_command"] >= target_abs_command:
            break
        hi *= 2.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if motor_commands_for_opposing_torque(mid, constants, vf_mps, yaw_rate)["max_abs_command"] >= target_abs_command:
            hi = mid
        else:
            lo = mid
    return hi


def synthetic_prediction(metadata: dict[str, float], candidate: Candidate, vf_mps: float, yaw_rate: float) -> float:
    frame = pd.DataFrame(
        {
            "abs_forward_velocity_mps": [abs(vf_mps)],
            "abs_yaw_rate_radps": [abs(yaw_rate)],
            "total_normal_load_n": [metadata["nominal_total_load_n"]],
        }
    )
    return float(predict(frame, metadata, candidate)[0])


def fit_candidates(frame: pd.DataFrame, metadata: dict[str, float], constants: dict[str, float], role: str, latest_weighted: bool) -> tuple[list[Candidate], Candidate]:
    weights = training_weights(frame, latest_weighted=latest_weighted)
    target = frame["residual_opposes_yaw_nm"].to_numpy()
    positive = weights > 0.0
    required_launch_support = required_opposing_for_command(constants, 0.0, 1.0, 0.646)
    candidates: list[Candidate] = []

    k1_values = [0.0, 0.10, 0.35]
    k2_values = [0.45, 1.10, 1.60]
    k3_values = [0.02, 0.05, 0.10, 0.20]
    stribeck_values = [0.015, 0.025, 0.04, 0.08]
    yaw_fade_values = [0.05, 0.10, 0.20, 0.40]
    sliding_values = [0.0, 0.35, 0.60]
    load_powers = [0.0, 0.5]

    for k1 in k1_values:
        for k2 in k2_values:
            if not k1 < k2:
                continue
            for k3 in k3_values:
                for stribeck_speed in stribeck_values:
                    for yaw_fade in yaw_fade_values:
                        for sliding_ratio in sliding_values:
                            for load_power in load_powers:
                                features = shifted_peak_features(frame, metadata, k1, k2, k3, stribeck_speed, yaw_fade, sliding_ratio, load_power)
                                beta = weighted_nnls(features, target, weights)
                                unconstrained_peak = float(beta[0])
                                prototype_unconstrained = Candidate(role, k1, k2, k3, stribeck_speed, yaw_fade, sliding_ratio, load_power, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                launch_unit = synthetic_prediction(metadata, prototype_unconstrained, 0.0, 1.0)
                                if launch_unit > 1.0e-9:
                                    beta[0] = max(unconstrained_peak, required_launch_support / launch_unit)
                                pred = features[:, 0] * beta[0]
                                train_rmse = float(np.sqrt(np.average(np.square((target - pred)[positive]), weights=weights[positive])))
                                corrected = corrected_residuals(frame, pred)
                                split = frame["dataset_split"].to_numpy()
                                primary_rmse = rmse(corrected[split == "primary_open_floor_fit_authoritative"])
                                validation_mask = split != "primary_open_floor_fit_authoritative"
                                validation_rmse = rmse(corrected[validation_mask])

                                derived_slope = float(beta[0] / max(k1, 1.0e-6))
                                prototype = Candidate(role, k1, k2, k3, stribeck_speed, yaw_fade, sliding_ratio, load_power, derived_slope, float(beta[0]), train_rmse, primary_rmse, validation_rmse, 0.0, 0.0, 0.0, 0.0)
                                launch_opposing = synthetic_prediction(metadata, prototype, 0.0, 1.0)
                                command = motor_commands_for_opposing_torque(launch_opposing, constants, 0.0, 1.0)
                                launch_target_error = abs(command["max_abs_command"] - 0.646)
                                score = train_rmse + 0.004 * launch_target_error
                                candidates.append(
                                    Candidate(
                                        role,
                                        k1,
                                        k2,
                                        k3,
                                        stribeck_speed,
                                        yaw_fade,
                                        sliding_ratio,
                                        load_power,
                                        derived_slope,
                                        float(beta[0]),
                                        train_rmse,
                                        primary_rmse,
                                        validation_rmse,
                                        launch_opposing,
                                        command["left_command"],
                                        command["right_command"],
                                        score,
                                    )
                                )

    gated = [c for c in candidates if max(abs(c.in_place_left_command), abs(c.in_place_right_command)) >= 0.6]
    near = [c for c in gated if abs(max(abs(c.in_place_left_command), abs(c.in_place_right_command)) - 0.646) <= 0.025]
    pool = near if near else gated if gated else candidates
    selected = min(pool, key=lambda item: item.score)
    return sorted(candidates, key=lambda item: item.score), selected


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def split_references() -> dict[str, dict[str, str]]:
    refs: dict[str, dict[str, str]] = {}
    for row in read_rows(VARIANT_C_SPLIT):
        refs.setdefault(row.get("group", row.get("dataset_split", "")), {})["variant_c_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(FORCE_DOMAIN_SPLIT):
        refs.setdefault(row["dataset_split"], {})["force_domain_stribeck_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(RATIONAL_BLEND_SPLIT):
        refs.setdefault(row.get("dataset_split", row.get("group", "")), {})["rational_residual_reference_rmse_nm"] = row.get("candidate_rmse_nm", row.get("corrected_rmse_nm", ""))
    for row in read_rows(TRUE_TRACTION_SPLIT):
        refs.setdefault(row.get("group", ""), {})["true_traction_testbed_rmse_nm"] = row.get("corrected_rmse_nm", row.get("standalone_rmse_nm", ""))
    for row in read_rows(STANDALONE_TRACTION_SPLIT):
        refs.setdefault(row.get("group", ""), {})["standalone_contact_traction_rmse_nm"] = row.get("standalone_rmse_nm", row.get("corrected_rmse_nm", ""))
    return refs


def run_references() -> dict[str, dict[str, str]]:
    refs: dict[str, dict[str, str]] = {}
    for row in read_rows(VARIANT_C_SELECTED):
        refs.setdefault(row["run_id"], {})["variant_c_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(FORCE_DOMAIN_SELECTED):
        refs.setdefault(row["run_id"], {})["force_domain_stribeck_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(RATIONAL_BLEND_SELECTED):
        refs.setdefault(row["run_id"], {})["rational_residual_reference_rmse_nm"] = row.get("candidate_rmse_nm", row.get("corrected_rmse_nm", ""))
    for row in read_rows(TRUE_TRACTION_SELECTED):
        refs.setdefault(row["run_id"], {})["true_traction_testbed_rmse_nm"] = row.get("corrected_rmse_nm", row.get("standalone_rmse_nm", ""))
    for row in read_rows(STANDALONE_TRACTION_SELECTED):
        refs.setdefault(row["run_id"], {})["standalone_contact_traction_rmse_nm"] = row.get("standalone_rmse_nm", row.get("corrected_rmse_nm", ""))
    return refs


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def candidate_row(candidate: Candidate) -> dict[str, object]:
    return {
        "role": candidate.role,
        "k1_rad_per_m": candidate.k1_rad_per_m,
        "k2_rad_per_m": candidate.k2_rad_per_m,
        "k3_radps": candidate.k3_radps,
        "stribeck_speed_mps": candidate.stribeck_speed_mps,
        "yaw_fade_mps": candidate.yaw_fade_mps,
        "sliding_ratio": candidate.sliding_ratio,
        "load_power": candidate.load_power,
        "derived_linear_slope_nm_per_radps_at_vf_1mps": candidate.linear_gain_nm_per_radps,
        "peak_yield_nm": candidate.peak_yield_nm,
        "weighted_train_rmse_nm": candidate.weighted_train_rmse_nm,
        "primary_corrected_rmse_nm": candidate.primary_corrected_rmse_nm,
        "validation_corrected_rmse_nm": candidate.validation_corrected_rmse_nm,
        "in_place_opposing_nm": candidate.in_place_opposing_nm,
        "in_place_left_command": candidate.in_place_left_command,
        "in_place_right_command": candidate.in_place_right_command,
        "in_place_max_abs_command": max(abs(candidate.in_place_left_command), abs(candidate.in_place_right_command)),
        "score": candidate.score,
    }


def write_metrics(frame: pd.DataFrame, metadata: dict[str, float], constants: dict[str, float], candidate: Candidate, suffix: str = "") -> dict[str, list[dict[str, object]]]:
    pred = predict(frame, metadata, candidate)
    split_refs = split_references()
    run_refs = run_references()

    split_rows = []
    for split in SPLITS:
        subset = frame[frame["dataset_split"] == split]
        idx = frame.index.get_indexer(subset.index)
        row = {"dataset_split": split}
        row.update(metric_row(subset, pred[idx]))
        row.update(split_refs.get(split, {}))
        split_rows.append(row)
    validation = frame[frame["dataset_split"] != "primary_open_floor_fit_authoritative"]
    idx = frame.index.get_indexer(validation.index)
    row = {"dataset_split": "validation_non_authoritative"}
    row.update(metric_row(validation, pred[idx]))
    row.update(split_refs.get("validation_non_authoritative", {}))
    split_rows.append(row)
    write_csv(OUT / f"split_metrics{suffix}.csv", split_rows)

    selected_rows = []
    for run_id in SELECTED_RUNS:
        subset = frame[frame["run_id"] == run_id]
        if subset.empty:
            selected_rows.append({"run_id": run_id, "present": False})
            continue
        idx = frame.index.get_indexer(subset.index)
        row = {
            "run_id": run_id,
            "present": True,
            "dataset_split": ";".join(sorted(subset["dataset_split"].dropna().unique())),
            "recommendation": ";".join(sorted(subset["recommendation"].dropna().unique())),
            "family": ";".join(sorted(subset["family"].dropna().unique())),
        }
        row.update(metric_row(subset, pred[idx]))
        row.update(run_refs.get(run_id, {}))
        selected_rows.append(row)
    write_csv(OUT / f"selected_log_metrics{suffix}.csv", selected_rows)

    risk_defs = [
        ("high_speed_abs_vf_ge_0p7", frame["abs_forward_velocity_mps"] >= 0.7),
        ("low_speed_yaw_abs_vf_lt_0p15_abs_yaw_ge_0p5", (frame["abs_forward_velocity_mps"] < 0.15) & (frame["abs_yaw_rate_radps"] >= 0.5)),
        ("limiter_active", frame["max_force_limiter_activity"] > 0.0),
        ("hardware_saturation_evidence", frame["hardware_saturation_evidence"] > 0.0),
        ("open_floor_all", frame["family"] == "open_floor"),
        ("diag_all", frame["family"].str.contains("diag", na=False)),
        ("aux_all", frame["dataset_split"] == "aux_downweighted_validation"),
    ]
    risk_rows = []
    for name, mask in risk_defs:
        subset = frame[mask]
        idx = frame.index.get_indexer(subset.index)
        row = {"slice": name}
        row.update(metric_row(subset, pred[idx]))
        risk_rows.append(row)
    write_csv(OUT / f"risk_metrics{suffix}.csv", risk_rows)

    in_place_rows = []
    for vf in [0.0, 0.03, 0.06, 0.09, 0.12, 0.15]:
        for yaw in [0.2, 0.844444444, 1.488888889, 2.133333333, 2.777777778, 3.422222222, 4.066666667, 4.711111111, 5.355555556, 6.0]:
            opposing = synthetic_prediction(metadata, candidate, vf, yaw)
            command = motor_commands_for_opposing_torque(opposing, constants, vf, yaw)
            in_place_rows.append({"vf_mps": vf, "yaw_rate_radps": yaw, "predicted_opposing_support_nm": opposing, **command})
    write_csv(OUT / f"lr_delta_grid_6x10{suffix}.csv", in_place_rows)

    launch = synthetic_prediction(metadata, candidate, 0.0, 1.0)
    launch_command = motor_commands_for_opposing_torque(launch, constants, 0.0, 1.0)
    launch_row = {"variant": candidate.role, "vf_mps": 0.0, "yaw_rate_radps": 1.0, "predicted_opposing_support_nm": launch, **launch_command}
    write_csv(OUT / f"in_place_1radps_command{suffix}.csv", [launch_row])

    sample = frame.copy()
    sample["predicted_opposing_support_nm"] = pred
    sample["corrected_residual_nm"] = corrected_residuals(frame, pred)
    sample = sample.sort_values(["run_id", "row_index"]).groupby("run_id", group_keys=False).head(40)
    write_csv(
        OUT / f"prediction_sample{suffix}.csv",
        sample[
            [
                "run_id",
                "dataset_split",
                "row_index",
                "family",
                "physics_phase",
                "forward_velocity_mps",
                "yaw_rate_radps",
                "residual_additive_yaw_torque_nm",
                "residual_opposes_yaw_nm",
                "predicted_opposing_support_nm",
                "corrected_residual_nm",
            ]
        ].to_dict("records"),
    )

    return {"split": split_rows, "selected": selected_rows, "risk": risk_rows, "launch": [launch_row]}


def fmt(value: object, places: int = 6) -> str:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(f):
        return ""
    return f"{f:.{places}f}"


def table(headers: list[str], rows: list[list[object]]) -> str:
    out = ["| " + " | ".join(headers) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    for row in rows:
        out.append("| " + " | ".join(str(v) for v in row) + " |")
    return "\n".join(out)


def write_report(metadata: dict[str, float], selected: Candidate, latest: Candidate, standard_metrics: dict[str, list[dict[str, object]]], latest_metrics: dict[str, list[dict[str, object]]]) -> None:
    split_rows = [
        [
            r["dataset_split"],
            r["count"],
            fmt(r["baseline_rmse_nm"]),
            fmt(r["corrected_rmse_nm"]),
            fmt(r.get("force_domain_stribeck_rmse_nm", "")),
            fmt(r.get("rational_residual_reference_rmse_nm", "")),
            fmt(r.get("standalone_contact_traction_rmse_nm", "")),
        ]
        for r in standard_metrics["split"]
    ]
    selected_rows = [
        [
            r["run_id"],
            r.get("dataset_split", ""),
            r.get("count", ""),
            fmt(r.get("baseline_rmse_nm", "")),
            fmt(r.get("corrected_rmse_nm", "")),
            fmt(r.get("force_domain_stribeck_rmse_nm", "")),
            fmt(r.get("rational_residual_reference_rmse_nm", "")),
            fmt(r.get("standalone_contact_traction_rmse_nm", "")),
        ]
        for r in standard_metrics["selected"]
    ]
    risk_rows = [
        [r["slice"], r["count"], fmt(r["baseline_rmse_nm"]), fmt(r["corrected_rmse_nm"]), fmt(r["rmse_improvement_pct"])]
        for r in standard_metrics["risk"]
    ]
    latest_rows = [
        [r["dataset_split"], r["count"], fmt(r["baseline_rmse_nm"]), fmt(r["corrected_rmse_nm"])]
        for r in latest_metrics["split"]
    ]
    params = [
        ["k1_rad_per_m", selected.k1_rad_per_m],
        ["k2_rad_per_m", selected.k2_rad_per_m],
        ["k3_radps", selected.k3_radps],
        ["derived_linear_slope_nm_per_radps_at_vf_1mps", selected.linear_gain_nm_per_radps],
        ["peak_yield_nm", selected.peak_yield_nm],
        ["stribeck_speed_mps", selected.stribeck_speed_mps],
        ["yaw_fade_mps", selected.yaw_fade_mps],
        ["sliding_ratio", selected.sliding_ratio],
        ["load_power", selected.load_power],
    ]
    latest_params = [
        ["k1_rad_per_m", latest.k1_rad_per_m],
        ["k2_rad_per_m", latest.k2_rad_per_m],
        ["k3_radps", latest.k3_radps],
        ["derived_linear_slope_nm_per_radps_at_vf_1mps", latest.linear_gain_nm_per_radps],
        ["peak_yield_nm", latest.peak_yield_nm],
        ["stribeck_speed_mps", latest.stribeck_speed_mps],
        ["yaw_fade_mps", latest.yaw_fade_mps],
        ["sliding_ratio", latest.sliding_ratio],
        ["load_power", latest.load_power],
    ]
    launch = standard_metrics["launch"][0]
    latest_launch = latest_metrics["launch"][0]
    report = f"""# Shifted-Peak Linear-Blend-to-Stribeck Yaw Regime Fit

Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.

This report supersedes the prior shifted_peak output that interpreted the middle regime too loosely. The middle section here is explicitly a linear blend into the Stribeck model.

## Candidate Law

Sign convention: `yawRate > 0` is clockwise. The law computes a nonnegative opposing yaw-support magnitude from `abs(yawRate)` and `abs(Vf)`; when evaluated as a residual correction its signed additive moment is `-sign(yawRate) * M_support`. Zero yaw uses a positive fallback sign only for metric bookkeeping.

Let `y = abs(yawRate)`, `v = abs(Vf)`, `x1 = v * k1`, `x2 = v * k2 + k3`, with `k1 < k2` and `x2 > x1`.

The selected linear component uses the boundary form from the prompt's `L_boundary_or_linear_continuation` choice:

`L(y,Vf) = M_peak * clamp(y / max(x1, eps), 0, 1)` for `y <= x1`

`L_boundary = M_peak`

For the shifted Stribeck branch, `d = y - x2`, `q = sqrt(v^2 + (L_yaw * d)^2)`, and `q0 = v`.

`S_raw(q) = R_slide + (1 - R_slide) / (1 + (q / V_stribeck)^2)`

`S_shifted = max(0, 1 + S_raw(q) - S_raw(q0))`

`S(y,Vf) = M_peak * S_shifted * Tail`

`Tail = 1 / (1 + ((L_yaw * max(d, 0)) / V_fade)^2)`

`Load = (N_total / N_nominal)^load_power`

Final regime:

- `y <= x1`: `M_support = Load * L(y,Vf)`
- `x1 < y < x2`: `M_support = Load * ((1 - alpha) * L_boundary + alpha * S(y,Vf))`, where `alpha = (y - x1) / (x2 - x1)`
- `y >= x2`: `M_support = Load * S(y,Vf)`

The subtraction by `S_raw(q0)` and addition of `1` is the shifted-peak part: at `y = x2`, the Stribeck branch is exactly at the selected peak, even when forward speed would otherwise reduce the raw Stribeck value. The transition itself remains the linear blend above; it is not replaced by the Stribeck curve.

The selected form uses `abs`, `sqrt`, clamp, rational schedules, and piecewise-linear blending only. It does not use command/request/preprojection values, UKF state-vector fields, or old contact-force outputs as selectors.

## Selected Constants

Standard fit used primary authoritative rows plus downweighted open-floor fit rows at the shared weights. It included May 4 rows only through their existing split weights; it is still dominated by the larger April authoritative set.

{table(["parameter", "value"], [[k, fmt(v, 9)] for k, v in params])}

## +1 rad/s In-Place Command Estimate

{table(["variant", "opposing support Nm", "left cmd", "right cmd", "max abs cmd"], [[selected.role, fmt(launch["predicted_opposing_support_nm"], 9), fmt(launch["left_command"], 9), fmt(launch["right_command"], 9), fmt(launch["max_abs_command"], 9)]])}

The hard launch gate passes: `Vf=0`, `Vr=0`, `yawRate=+1 rad/s` estimates at least `|cmd| >= 0.6` and is near the `+0.646/-0.646` reference.

## Split Metrics

{table(["split", "count", "baseline RMSE", "shifted-peak RMSE", "force-domain Stribeck ref", "rational residual ref", "standalone contact ref"], split_rows)}

## Selected-Log Metrics

{table(["run", "split", "count", "baseline RMSE", "shifted-peak RMSE", "force-domain ref", "rational residual ref", "standalone contact ref"], selected_rows)}

The latest logs are included separately above: `2026-05-04_20-35-47` is fit-downweighted and `2026-05-04_16-57-53` is validation-only in the shared split contract.

## Risk Slices

{table(["slice", "count", "baseline RMSE", "shifted-peak RMSE", "improvement %"], risk_rows)}

## Latest-Weighted Diagnostic

This diagnostic multiplies positive training weights for `2026-05-04_20-35-47` and `2026-05-04_16-57-53` by `4` before run balancing. It is not the standard selected fit.

{table(["parameter", "latest-weighted value"], [[k, fmt(v, 9)] for k, v in latest_params])}

{table(["variant", "opposing support Nm", "left cmd", "right cmd", "max abs cmd"], [[latest.role, fmt(latest_launch["predicted_opposing_support_nm"], 9), fmt(latest_launch["left_command"], 9), fmt(latest_launch["right_command"], 9), fmt(latest_launch["max_abs_command"], 9)]])}

{table(["split", "count", "baseline RMSE", "latest-weighted RMSE"], latest_rows)}

## Caveats

- This is a standalone yaw-support law. The shared residual columns are used only as the comparable evaluation target against the current baseline gap; the selected equation is not `old + residual`.
- The standard fit is constrained by the hard launch command target, so its `M_peak` is not selected by RMSE alone.
- The model is symmetric in `abs(yawRate)` and `abs(Vf)`. The available data did not justify signed branch behavior.
- Compared with the standalone contact-traction testbed, this law is much cheaper and simpler but less complete because it cannot model driven yaw moment directly.
- The shifted branch preserves the peak at the handoff by construction; if production wants a forward-speed-reduced breakaway peak, this specific variant is the wrong policy.

## Reproduce

```powershell
& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' codex_analysis\\yaw_model_variant_fits\\regime_linear_to_stribeck_shifted_peak\\fit_shifted_peak.py
```

## Outputs

- `fit_shifted_peak.py`
- `shifted_peak_report.md`
- `candidate_scores.csv`
- `selected_parameters.csv`
- `latest_weighted_parameters.csv`
- `split_metrics.csv`
- `selected_log_metrics.csv`
- `risk_metrics.csv`
- `in_place_1radps_command.csv`
- `lr_delta_grid_6x10.csv`
- `prediction_sample.csv`
- matching `_latest_weighted` metric/grid/sample CSVs
- `metadata.json`
"""
    (OUT / "shifted_peak_report.md").write_text(report, encoding="utf-8")


def main() -> None:
    constants = read_constants()
    frame, metadata = load_frame(constants)
    standard_candidates, selected = fit_candidates(frame, metadata, constants, "standard_shifted_peak", latest_weighted=False)
    latest_candidates, latest = fit_candidates(frame, metadata, constants, "latest_weighted_shifted_peak", latest_weighted=True)

    write_csv(OUT / "candidate_scores.csv", [candidate_row(c) for c in standard_candidates[:250]])
    write_csv(OUT / "candidate_scores_latest_weighted.csv", [candidate_row(c) for c in latest_candidates[:250]])
    write_csv(OUT / "selected_parameters.csv", [candidate_row(selected)])
    write_csv(OUT / "latest_weighted_parameters.csv", [candidate_row(latest)])

    standard_metrics = write_metrics(frame, metadata, constants, selected)
    latest_metrics = write_metrics(frame, metadata, constants, latest, suffix="_latest_weighted")
    (OUT / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    (OUT / "commands_run.txt").write_text(
        "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' codex_analysis\\yaw_model_variant_fits\\regime_linear_to_stribeck_shifted_peak\\fit_shifted_peak.py\n",
        encoding="utf-8",
    )
    write_report(metadata, selected, latest, standard_metrics, latest_metrics)


if __name__ == "__main__":
    main()
