#!/usr/bin/env python3
"""Fit a yaw-rate regime split into an unshifted Stribeck yaw-support law.

Analysis-only tooling. It reads shared yaw-model fit artifacts and writes
outputs only beside this script.
"""

from __future__ import annotations

import csv
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
RATIONAL_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "transition_options"
    / "rational_speed_force_blend"
)
FORCE_DOMAIN_DIR = (
    ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck"
)
TRUE_TRACTION_DIR = (
    ROOT / "codex_analysis" / "yaw_model_variant_fits" / "contact_patch_true_traction_testbed"
)
STANDALONE_TRACTION_DIR = (
    ROOT / "codex_analysis" / "yaw_model_variant_fits" / "standalone_contact_traction_testbed"
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
    "vbar_rel_mps",
    "vbar_yaw_mps",
    "max_force_limiter_activity",
    "hardware_saturation_evidence",
    "gyro_derivative_spike",
    "residual_additive_yaw_torque_nm",
    "residual_opposes_yaw_nm",
]

SECONDARY_COLUMNS = ["run_id", "row_index", "total_normal_load_n"]

SPLITS = [
    "primary_open_floor_fit_authoritative",
    "open_floor_fit_downweighted",
    "open_floor_validation_only",
    "diag_validation_only",
    "aux_downweighted_validation",
]


@dataclass(frozen=True)
class Shape:
    k1: float
    k2: float
    k3_radps: float
    stribeck_speed_mps: float
    speed_fade_mps: float
    rel_weight: float
    load_exponent: float


@dataclass(frozen=True)
class FitResult:
    variant: str
    shape: Shape
    sliding_nm: float
    static_extra_nm: float
    train_weighted_rmse_nm: float
    primary_rmse_nm: float
    validation_rmse_nm: float
    launch_extra_nm: float
    launch_left_command: float
    launch_right_command: float
    score: float


def read_constants() -> dict[str, float]:
    table = pd.read_csv(CONSTANTS_INPUT)
    return {str(row.name): float(row.value) for row in table.itertuples(index=False)}


def sign_array(values: np.ndarray, eps: float = 1.0e-8) -> np.ndarray:
    return (values > eps).astype(float) - (values < -eps).astype(float)


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else math.nan


def mae(values: np.ndarray) -> float:
    return float(np.mean(np.abs(values))) if len(values) else math.nan


def median_abs(values: np.ndarray) -> float:
    return float(np.median(np.abs(values))) if len(values) else math.nan


def run_balanced_rmse(frame: pd.DataFrame, values: np.ndarray) -> float:
    if not len(frame):
        return math.nan
    counts = frame["run_id"].astype(str).value_counts().to_dict()
    weights = frame["run_id"].astype(str).map(lambda run: 1.0 / counts[run]).to_numpy(float)
    return float(np.sqrt(np.average(np.square(values), weights=weights)))


def torque_from_command(command: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]

    applied_voltage = command * battery
    back_emf_per_wheel_radps = gear_ratio / speed_constant
    current = (applied_voltage / resistance) - (
        (wheel_speed_radps * back_emf_per_wheel_radps) / resistance
    )
    armature_sign = (current > 1.0e-6) - (current < -1.0e-6)
    wheel_sign = (wheel_speed_radps > 1.0e-6) - (wheel_speed_radps < -1.0e-6)
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
    torque_sign = (motor_torque > 1.0e-6) - (motor_torque < -1.0e-6)
    wheel_sign = (wheel_speed_radps > 1.0e-6) - (wheel_speed_radps < -1.0e-6)
    no_load_sign = torque_sign if torque_sign else wheel_sign
    current = (motor_torque / torque_constant) + no_load_sign * no_load
    back_emf = wheel_speed_radps * (gear_ratio / speed_constant)
    return ((current * resistance) + back_emf) / battery


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
    opposing_yaw_torque: float, constants: dict[str, float], yaw_rate: float
) -> dict[str, float]:
    track = constants["track_width_m"]
    radius = constants["wheel_radius_m"]
    half_track = 0.5 * track
    left_speed = half_track * yaw_rate / radius
    right_speed = -left_speed
    applied_bank_torque = opposing_yaw_torque * radius / track
    rolling = constants["rolling_friction_torque_nm"]
    left_command_torque = applied_bank_torque + rolling
    right_command_torque = -applied_bank_torque - rolling
    return {
        "applied_bank_torque_nm": applied_bank_torque,
        "left_command": command_from_torque(left_command_torque, left_speed, constants),
        "right_command": command_from_torque(right_command_torque, right_speed, constants),
    }


def extra_for_target_command(constants: dict[str, float], yaw_rate: float, target_command: float) -> float:
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    lo = 0.0
    hi = 0.2
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        cmd = motor_commands_for_opposing_torque(base + mid, constants, yaw_rate)["left_command"]
        if cmd < target_command:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def load_frame(constants: dict[str, float]) -> tuple[pd.DataFrame, float]:
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
    frame = frame.replace([np.inf, -np.inf], np.nan)
    frame = frame.dropna(
        subset=[
            "forward_velocity_mps",
            "yaw_rate_radps",
            "vbar_rel_mps",
            "residual_additive_yaw_torque_nm",
            "residual_opposes_yaw_nm",
        ]
    )
    frame = frame[frame["dataset_split"] != "excluded_or_unclassified"].copy()
    primary = frame["dataset_split"] == "primary_open_floor_fit_authoritative"
    nominal_load = float(frame.loc[primary, "total_normal_load_n"].median())
    if not math.isfinite(nominal_load) or nominal_load <= 0.0:
        nominal_load = constants["mass_kg"] * 9.80665 + constants.get("fan_downforce_full_duty_n", 0.0)
    frame["total_normal_load_n"] = frame["total_normal_load_n"].fillna(nominal_load).clip(lower=1.0e-6)
    frame["abs_forward_velocity_mps"] = frame["forward_velocity_mps"].abs()
    frame["abs_yaw_rate_radps"] = frame["yaw_rate_radps"].abs()
    yaw_sign = sign_array(frame["yaw_rate_radps"].to_numpy(float))
    additive_sign = -sign_array(frame["residual_additive_yaw_torque_nm"].to_numpy(float))
    yaw_sign = np.where(yaw_sign == 0.0, additive_sign, yaw_sign)
    yaw_sign = np.where(yaw_sign == 0.0, 1.0, yaw_sign)
    frame["yaw_sign"] = yaw_sign
    return frame.reset_index(drop=True), nominal_load


def bases_for_arrays(
    shape: Shape,
    yaw_abs: np.ndarray,
    vf_abs: np.ndarray,
    vbar_rel: np.ndarray,
    total_load: np.ndarray,
    nominal_load: float,
    yaw_rel_per_radps: float,
) -> np.ndarray:
    a = vf_abs * shape.k1
    b = vf_abs * shape.k2 + shape.k3_radps
    width = np.maximum(b - a, 1.0e-9)
    blend = np.clip((yaw_abs - a) / width, 0.0, 1.0)

    yaw_rel = yaw_rel_per_radps * yaw_abs
    vt2 = np.square(vf_abs) + np.square(shape.rel_weight * yaw_rel)
    fade = (shape.speed_fade_mps * shape.speed_fade_mps) / (
        shape.speed_fade_mps * shape.speed_fade_mps + vt2
    )
    stribeck = (shape.stribeck_speed_mps * shape.stribeck_speed_mps) / (
        shape.stribeck_speed_mps * shape.stribeck_speed_mps + vt2
    )
    load_scale = np.power(np.maximum(total_load / nominal_load, 1.0e-6), shape.load_exponent)

    boundary_vbar_rel = yaw_rel_per_radps * a
    boundary_vt2 = np.square(vf_abs) + np.square(shape.rel_weight * boundary_vbar_rel)
    boundary_fade = (shape.speed_fade_mps * shape.speed_fade_mps) / (
        shape.speed_fade_mps * shape.speed_fade_mps + boundary_vt2
    )
    boundary_stribeck = (shape.stribeck_speed_mps * shape.stribeck_speed_mps) / (
        shape.stribeck_speed_mps * shape.stribeck_speed_mps + boundary_vt2
    )

    # The linear slope is not an independent fitted constant. For each forward
    # speed, the zero-yaw line is chosen to hit the raw, unshifted Stribeck curve
    # at yawRate = abs(Vf) * k1. That preserves a clean handoff and keeps the
    # Stribeck branch vertically unshifted.
    safe_a = np.maximum(a, 1.0e-9)
    has_linear_width = a > 1.0e-9
    linear_fraction = np.where(has_linear_width, np.clip(yaw_abs / safe_a, 0.0, 1.0), 0.0)
    in_linear = yaw_abs <= a
    in_transition = (yaw_abs > a) & (yaw_abs < b)

    current_slide = load_scale * fade
    current_static = load_scale * fade * stribeck
    boundary_slide = load_scale * boundary_fade
    boundary_static = load_scale * boundary_fade * boundary_stribeck
    linear_slide = boundary_slide * linear_fraction
    linear_static = boundary_static * linear_fraction

    slide_basis = np.where(
        in_linear,
        linear_slide,
        np.where(in_transition, (1.0 - blend) * boundary_slide + blend * current_slide, current_slide),
    )
    static_basis = np.where(
        in_linear,
        linear_static,
        np.where(
            in_transition,
            (1.0 - blend) * boundary_static + blend * current_static,
            current_static,
        ),
    )

    return np.column_stack([slide_basis, static_basis])


def predict_support(
    frame: pd.DataFrame,
    nominal_load: float,
    shape: Shape,
    coeffs: np.ndarray,
    yaw_rel_per_radps: float,
) -> np.ndarray:
    bases = bases_for_arrays(
        shape,
        frame["abs_yaw_rate_radps"].to_numpy(float),
        frame["abs_forward_velocity_mps"].to_numpy(float),
        frame["vbar_rel_mps"].to_numpy(float),
        frame["total_normal_load_n"].to_numpy(float),
        nominal_load,
        yaw_rel_per_radps,
    )
    return bases @ coeffs


def launch_support(
    constants: dict[str, float], nominal_load: float, shape: Shape, coeffs: np.ndarray
) -> float:
    yaw = np.array([1.0])
    vf = np.array([0.0])
    vbar_rel = np.array([constants["drive_wheel_longitudinal_offset_m"]])
    load = np.array([nominal_load])
    value = (
        bases_for_arrays(
            shape,
            yaw,
            vf,
            vbar_rel,
            load,
            nominal_load,
            constants["drive_wheel_longitudinal_offset_m"],
        )
        @ coeffs
    )
    return float(value[0])


def nonnegative_lstsq(bases: np.ndarray, target: np.ndarray, weights: np.ndarray) -> np.ndarray:
    sw = np.sqrt(np.maximum(weights, 0.0))
    best_coeffs = np.zeros(bases.shape[1], dtype=float)
    best_sse = math.inf
    for mask in range(1, 1 << bases.shape[1]):
        cols = [i for i in range(bases.shape[1]) if mask & (1 << i)]
        x = bases[:, cols] * sw[:, None]
        y = target * sw
        try:
            coeff_sub, *_ = np.linalg.lstsq(x, y, rcond=None)
        except np.linalg.LinAlgError:
            continue
        if np.any(coeff_sub < -1.0e-10):
            continue
        coeff = np.zeros(bases.shape[1], dtype=float)
        coeff[cols] = np.maximum(coeff_sub, 0.0)
        residual = target - bases @ coeff
        sse = float(np.sum(weights * np.square(residual)))
        if sse < best_sse:
            best_sse = sse
            best_coeffs = coeff
    return best_coeffs


def training_weights(frame: pd.DataFrame, variant: str) -> np.ndarray:
    split = frame["dataset_split"].astype(str)
    weights = np.zeros(len(frame), dtype=float)
    weights[split == "primary_open_floor_fit_authoritative"] = 1.0
    weights[split == "open_floor_fit_downweighted"] = 0.25
    if variant == "latest_weighted":
        latest = frame["run_id"].astype(str).isin(["2026-05-04_20-35-47", "2026-05-04_16-57-53"])
        weights[latest] = np.maximum(weights[latest], 3.0)
    return weights


def fit_shape(
    frame: pd.DataFrame,
    nominal_load: float,
    constants: dict[str, float],
    shape: Shape,
    variant: str,
    launch_target_extra: float,
    coarse_indices: np.ndarray | None = None,
) -> tuple[np.ndarray, float]:
    if coarse_indices is None:
        fit_frame = frame
        weights = training_weights(frame, variant)
    else:
        fit_frame = frame.iloc[coarse_indices]
        weights = training_weights(fit_frame, variant)

    active = weights > 0.0
    fit_frame = fit_frame.loc[active]
    weights = weights[active]
    bases = bases_for_arrays(
        shape,
        fit_frame["abs_yaw_rate_radps"].to_numpy(float),
        fit_frame["abs_forward_velocity_mps"].to_numpy(float),
        fit_frame["vbar_rel_mps"].to_numpy(float),
        fit_frame["total_normal_load_n"].to_numpy(float),
        nominal_load,
        constants["drive_wheel_longitudinal_offset_m"],
    )
    target = fit_frame["residual_opposes_yaw_nm"].to_numpy(float)

    launch_basis = bases_for_arrays(
        shape,
        np.array([1.0]),
        np.array([0.0]),
        np.array([constants["drive_wheel_longitudinal_offset_m"]]),
        np.array([nominal_load]),
        nominal_load,
        constants["drive_wheel_longitudinal_offset_m"],
    )
    bases = np.vstack([bases, launch_basis])
    target = np.concatenate([target, np.array([launch_target_extra])])
    weights = np.concatenate([weights, np.array([1.0e7])])
    coeffs = nonnegative_lstsq(bases, target, weights)
    residual = target - bases @ coeffs
    score = float(np.sqrt(np.average(np.square(residual), weights=weights)))
    return coeffs, score


def evaluate_fit(
    frame: pd.DataFrame,
    nominal_load: float,
    constants: dict[str, float],
    variant: str,
    shape: Shape,
    coeffs: np.ndarray,
    train_score: float,
) -> FitResult:
    pred = predict_support(
        frame, nominal_load, shape, coeffs, constants["drive_wheel_longitudinal_offset_m"]
    )
    err = frame["residual_opposes_yaw_nm"].to_numpy(float) - pred
    primary_mask = frame["dataset_split"] == "primary_open_floor_fit_authoritative"
    validation_mask = frame["dataset_split"] != "primary_open_floor_fit_authoritative"
    launch_extra = launch_support(constants, nominal_load, shape, coeffs)
    total = baseline_opposing_yaw_torque(constants, 1.0) + launch_extra
    commands = motor_commands_for_opposing_torque(total, constants, 1.0)
    primary_rmse = rmse(err[primary_mask.to_numpy()])
    validation_rmse = rmse(err[validation_mask.to_numpy()])
    score = train_score
    if abs(commands["left_command"]) < 0.6:
        score += 100.0
    return FitResult(
        variant=variant,
        shape=shape,
        sliding_nm=float(coeffs[0]),
        static_extra_nm=float(coeffs[1]),
        train_weighted_rmse_nm=train_score,
        primary_rmse_nm=primary_rmse,
        validation_rmse_nm=validation_rmse,
        launch_extra_nm=launch_extra,
        launch_left_command=float(commands["left_command"]),
        launch_right_command=float(commands["right_command"]),
        score=score,
    )


def candidate_shapes(seed: int = 20260526, random_count: int = 4500) -> list[Shape]:
    rng = np.random.default_rng(seed)
    shapes: list[Shape] = []
    seeds = [
        Shape(0.08, 0.53, 0.06, 0.025, 0.64, 0.75, 0.0),
        Shape(0.15, 0.60, 0.10, 0.040, 0.64, 0.75, 0.0),
        Shape(0.30, 0.75, 0.18, 0.070, 0.90, 1.00, 0.5),
        Shape(0.03, 0.23, 0.03, 0.025, 0.40, 0.50, 0.0),
        Shape(0.60, 1.50, 0.32, 0.100, 1.30, 1.00, 1.0),
    ]
    shapes.extend(seeds)

    for _ in range(random_count):
        k1 = float(np.exp(rng.uniform(math.log(0.015), math.log(0.9))))
        delta = float(np.exp(rng.uniform(math.log(0.025), math.log(2.7))))
        k2 = k1 + delta
        k3 = float(np.exp(rng.uniform(math.log(0.006), math.log(1.1))))
        ss = float(np.exp(rng.uniform(math.log(0.01), math.log(0.35))))
        fade = float(np.exp(rng.uniform(math.log(0.12), math.log(2.5))))
        rel = float(rng.uniform(0.0, 1.8))
        load_exp = float(rng.choice([0.0, 0.25, 0.5, 0.75, 1.0]))
        shapes.append(Shape(k1, k2, k3, ss, fade, rel, load_exp))
    return shapes


def tune_variant(
    frame: pd.DataFrame,
    nominal_load: float,
    constants: dict[str, float],
    variant: str,
    launch_target_extra: float,
) -> tuple[FitResult, pd.DataFrame]:
    rng = np.random.default_rng(6174 if variant == "standard" else 8171)
    train = training_weights(frame, variant) > 0.0
    train_idx = np.flatnonzero(train)
    sample_count = min(len(train_idx), 26000)
    coarse_idx = rng.choice(train_idx, size=sample_count, replace=False)

    rows = []
    shapes = candidate_shapes(random_count=1800 if variant == "standard" else 1200)
    for idx, shape in enumerate(shapes):
        coeffs, coarse_score = fit_shape(
            frame, nominal_load, constants, shape, variant, launch_target_extra, coarse_idx
        )
        if idx % 200 == 0:
            pass
        rows.append((coarse_score, shape, coeffs))

    rows.sort(key=lambda item: item[0])
    full_results: list[FitResult] = []
    full_rows = []
    for coarse_score, shape, _ in rows[:96]:
        coeffs, train_score = fit_shape(
            frame, nominal_load, constants, shape, variant, launch_target_extra, None
        )
        result = evaluate_fit(frame, nominal_load, constants, variant, shape, coeffs, train_score)
        full_results.append(result)
        full_rows.append(result_to_row(result))

    full_results.sort(key=lambda result: result.score)
    return full_results[0], pd.DataFrame(full_rows).sort_values("score").head(80)


def result_to_row(result: FitResult) -> dict[str, float | str | bool]:
    return {
        "variant": result.variant,
        "k1": result.shape.k1,
        "k2": result.shape.k2,
        "k3_radps": result.shape.k3_radps,
        "stribeck_speed_mps": result.shape.stribeck_speed_mps,
        "speed_fade_mps": result.shape.speed_fade_mps,
        "rel_weight": result.shape.rel_weight,
        "load_exponent": result.shape.load_exponent,
        "sliding_nm": result.sliding_nm,
        "static_extra_nm": result.static_extra_nm,
        "train_weighted_rmse_nm": result.train_weighted_rmse_nm,
        "primary_rmse_nm": result.primary_rmse_nm,
        "validation_rmse_nm": result.validation_rmse_nm,
        "launch_extra_nm": result.launch_extra_nm,
        "launch_left_command": result.launch_left_command,
        "launch_right_command": result.launch_right_command,
        "launch_max_abs_command": max(abs(result.launch_left_command), abs(result.launch_right_command)),
        "passes_abs_0p6_gate": max(abs(result.launch_left_command), abs(result.launch_right_command))
        >= 0.6,
        "score": result.score,
    }


def metrics_for_group(
    name: str, group: pd.DataFrame, pred: np.ndarray, corrected_additive: np.ndarray
) -> dict[str, float | str]:
    target = group["residual_opposes_yaw_nm"].to_numpy(float)
    additive = group["residual_additive_yaw_torque_nm"].to_numpy(float)
    err = target - pred
    return {
        "group": name,
        "count": len(group),
        "run_count": int(group["run_id"].nunique()) if len(group) else 0,
        "baseline_rmse_nm": rmse(additive),
        "corrected_rmse_nm": rmse(err),
        "baseline_mae_nm": mae(additive),
        "corrected_mae_nm": mae(err),
        "baseline_median_abs_nm": median_abs(additive),
        "corrected_median_abs_nm": median_abs(err),
        "baseline_signed_median_nm": float(np.median(additive)) if len(group) else math.nan,
        "corrected_signed_median_nm": float(np.median(corrected_additive)) if len(group) else math.nan,
        "run_balanced_corrected_rmse_nm": run_balanced_rmse(group, err),
        "median_support_nm": float(np.median(pred)) if len(group) else math.nan,
        "rmse_improvement_pct": 100.0 * (rmse(additive) - rmse(err)) / rmse(additive)
        if len(group) and rmse(additive)
        else math.nan,
    }


def add_predictions(
    frame: pd.DataFrame, nominal_load: float, constants: dict[str, float], result: FitResult
) -> pd.DataFrame:
    coeffs = np.array([result.sliding_nm, result.static_extra_nm], dtype=float)
    pred = predict_support(
        frame,
        nominal_load,
        result.shape,
        coeffs,
        constants["drive_wheel_longitudinal_offset_m"],
    )
    out = frame.copy()
    out["predicted_opposing_yaw_support_nm"] = pred
    out["corrected_residual_opposes_yaw_nm"] = out["residual_opposes_yaw_nm"] - pred
    out["candidate_additive_yaw_torque_nm"] = -out["yaw_sign"] * pred
    out["corrected_residual_additive_yaw_torque_nm"] = (
        out["residual_additive_yaw_torque_nm"] - out["candidate_additive_yaw_torque_nm"]
    )
    return out


def write_metrics(frame: pd.DataFrame, variant: str) -> None:
    pred = frame["predicted_opposing_yaw_support_nm"].to_numpy(float)
    corrected_add = frame["corrected_residual_additive_yaw_torque_nm"].to_numpy(float)

    split_rows = []
    for split in SPLITS:
        mask = frame["dataset_split"] == split
        split_rows.append(
            metrics_for_group(
                split,
                frame.loc[mask],
                pred[mask.to_numpy()],
                corrected_add[mask.to_numpy()],
            )
        )
    val_mask = frame["dataset_split"] != "primary_open_floor_fit_authoritative"
    split_rows.append(
        metrics_for_group(
            "validation_non_authoritative",
            frame.loc[val_mask],
            pred[val_mask.to_numpy()],
            corrected_add[val_mask.to_numpy()],
        )
    )
    pd.DataFrame(split_rows).to_csv(OUT / f"{variant}_split_metrics.csv", index=False)

    selected_rows = []
    for run in SELECTED_RUNS:
        mask = frame["run_id"].astype(str) == run
        if not bool(mask.any()):
            selected_rows.append({"run_id": run, "present": False, "count": 0})
            continue
        group = frame.loc[mask]
        row = metrics_for_group(
            run, group, pred[mask.to_numpy()], corrected_add[mask.to_numpy()]
        )
        row["run_id"] = run
        row["present"] = True
        row["dataset_split"] = str(group["dataset_split"].mode().iloc[0])
        selected_rows.append(row)
    pd.DataFrame(selected_rows).to_csv(OUT / f"{variant}_selected_log_metrics.csv", index=False)

    latest_mask = frame["run_id"].astype(str).isin(["2026-05-04_20-35-47", "2026-05-04_16-57-53"])
    april_mask = frame["run_id"].astype(str).str.startswith("2026-04")
    risk_defs = {
        "straightish_abs_yaw_lt_0p05": frame["abs_yaw_rate_radps"] < 0.05,
        "straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05": (frame["abs_yaw_rate_radps"] < 0.05)
        & (frame["abs_forward_velocity_mps"] >= 0.05),
        "low_speed_yaw_vf_lt_0p05_yaw_ge_0p2": (frame["abs_forward_velocity_mps"] < 0.05)
        & (frame["abs_yaw_rate_radps"] >= 0.2),
        "low_speed_yaw_vf_lt_0p15_yaw_ge_0p5": (frame["abs_forward_velocity_mps"] < 0.15)
        & (frame["abs_yaw_rate_radps"] >= 0.5),
        "high_forward_vf_ge_0p5": frame["abs_forward_velocity_mps"] >= 0.5,
        "high_speed_abs_vf_ge_0p7": frame["abs_forward_velocity_mps"] >= 0.7,
        "limiter_active": frame["max_force_limiter_activity"].fillna(0.0) > 0.05,
        "hardware_saturation_evidence": frame["hardware_saturation_evidence"].fillna(0.0) > 0.0,
        "may4_latest_logs": latest_mask,
        "april_rows": april_mask,
    }
    risk_rows = []
    for name, mask in risk_defs.items():
        risk_rows.append(
            metrics_for_group(
                name,
                frame.loc[mask],
                pred[mask.to_numpy()],
                corrected_add[mask.to_numpy()],
            )
        )
    pd.DataFrame(risk_rows).to_csv(OUT / f"{variant}_risk_metrics.csv", index=False)

    phase_rows = []
    for split in SPLITS:
        for phase in ["entry", "plateau", "exit"]:
            mask = (frame["dataset_split"] == split) & (frame["physics_phase"] == phase)
            if bool(mask.any()):
                phase_rows.append(
                    metrics_for_group(
                        f"{split}:{phase}",
                        frame.loc[mask],
                        pred[mask.to_numpy()],
                        corrected_add[mask.to_numpy()],
                    )
                )
    pd.DataFrame(phase_rows).to_csv(OUT / f"{variant}_phase_metrics.csv", index=False)


def read_reference_metric(path: Path, key_col: str, value_cols: dict[str, str]) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame(columns=[key_col, *value_cols.values()])
    frame = pd.read_csv(path)
    keep = [key_col, *value_cols.keys()]
    missing = [column for column in keep if column not in frame.columns]
    if missing:
        return pd.DataFrame(columns=[key_col, *value_cols.values()])
    return frame[keep].rename(columns=value_cols)


def write_comparisons() -> None:
    base = pd.read_csv(OUT / "standard_split_metrics.csv")[
        ["group", "baseline_rmse_nm", "corrected_rmse_nm"]
    ].rename(columns={"corrected_rmse_nm": "unshifted_corrected_rmse_nm"})
    refs = [
        read_reference_metric(
            FORCE_DOMAIN_DIR / "split_rmse.csv",
            "dataset_split",
            {"corrected_rmse_nm": "force_domain_stribeck_rmse_nm"},
        ).rename(columns={"dataset_split": "group"}),
        read_reference_metric(
            RATIONAL_DIR / "split_metrics.csv",
            "group",
            {"corrected_rmse_nm": "rational_residual_reference_rmse_nm"},
        ),
        read_reference_metric(
            TRUE_TRACTION_DIR / "baseline_comparison.csv",
            "group",
            {
                "true_patch_corrected_rmse_nm": "true_traction_testbed_rmse_nm",
                "cubic_force_only_partition_rmse_nm": "cubic_force_only_partition_rmse_nm",
            },
        ),
        read_reference_metric(
            STANDALONE_TRACTION_DIR / "split_metrics.csv",
            "group",
            {"standalone_rmse_nm": "standalone_contact_traction_rmse_nm"},
        ),
    ]
    comparison = base
    for ref in refs:
        comparison = comparison.merge(ref, how="left", on="group")
    comparison.to_csv(OUT / "baseline_comparison.csv", index=False)

    selected = pd.read_csv(OUT / "standard_selected_log_metrics.csv")[
        ["run_id", "dataset_split", "baseline_rmse_nm", "corrected_rmse_nm"]
    ].rename(columns={"corrected_rmse_nm": "unshifted_corrected_rmse_nm"})
    selected_refs = [
        read_reference_metric(
            FORCE_DOMAIN_DIR / "selected_log_rmse.csv",
            "run_id",
            {"corrected_rmse_nm": "force_domain_stribeck_rmse_nm"},
        ),
        read_reference_metric(
            TRUE_TRACTION_DIR / "selected_log_metrics.csv",
            "run_id",
            {"corrected_rmse_nm": "true_traction_testbed_rmse_nm"},
        ),
        read_reference_metric(
            STANDALONE_TRACTION_DIR / "selected_log_metrics.csv",
            "run_id",
            {"standalone_rmse_nm": "standalone_contact_traction_rmse_nm"},
        ),
    ]
    selected_comp = selected
    for ref in selected_refs:
        selected_comp = selected_comp.merge(ref, how="left", on="run_id")
    selected_comp.to_csv(OUT / "selected_log_comparison.csv", index=False)


def format_float(value: object, digits: int = 6) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(number):
        return ""
    return f"{number:.{digits}f}"


def markdown_table(frame: pd.DataFrame, columns: list[str], max_rows: int | None = None) -> str:
    if max_rows is not None:
        frame = frame.head(max_rows)
    lines = []
    lines.append("| " + " | ".join(columns) + " |")
    lines.append("| " + " | ".join("---" for _ in columns) + " |")
    for _, row in frame.iterrows():
        values = []
        for col in columns:
            value = row.get(col, "")
            if isinstance(value, (int, np.integer)):
                values.append(str(value))
            elif isinstance(value, (float, np.floating)):
                values.append(format_float(value))
            else:
                values.append(str(value))
        lines.append("| " + " | ".join(values) + " |")
    return "\n".join(lines)


def write_report(
    standard: FitResult,
    latest: FitResult,
    constants: dict[str, float],
    nominal_load: float,
    launch_target_extra: float,
    frame: pd.DataFrame,
) -> None:
    selected_params = pd.DataFrame([result_to_row(standard), result_to_row(latest)])
    split = pd.read_csv(OUT / "standard_split_metrics.csv")
    selected = pd.read_csv(OUT / "standard_selected_log_metrics.csv")
    latest_selected = pd.read_csv(OUT / "latest_weighted_selected_log_metrics.csv")
    risk = pd.read_csv(OUT / "standard_risk_metrics.csv")
    comp = pd.read_csv(OUT / "baseline_comparison.csv")
    selected_comp = pd.read_csv(OUT / "selected_log_comparison.csv")
    weights = training_weights(frame, "standard")
    weighted = frame.loc[weights > 0.0, ["run_id", "dataset_split"]].copy()
    weighted["weight"] = weights[weights > 0.0]
    latest_rows = weighted["run_id"].astype(str).isin(
        ["2026-05-04_20-35-47", "2026-05-04_16-57-53"]
    )
    latest_weight_fraction = float(weighted.loc[latest_rows, "weight"].sum() / weighted["weight"].sum())
    april_weight_fraction = float(
        weighted.loc[weighted["run_id"].astype(str).str.startswith("2026-04"), "weight"].sum()
        / weighted["weight"].sum()
    )

    launch_table = pd.DataFrame(
        [
            {
                "variant": "standard",
                "target_extra_nm_for_0p646_cmd": launch_target_extra,
                "extra_opposing_yaw_torque_nm": standard.launch_extra_nm,
                "total_opposing_yaw_torque_nm": baseline_opposing_yaw_torque(constants, 1.0)
                + standard.launch_extra_nm,
                "left_command": standard.launch_left_command,
                "right_command": standard.launch_right_command,
                "max_abs_command": max(abs(standard.launch_left_command), abs(standard.launch_right_command)),
                "passes_abs_0p6_gate": max(abs(standard.launch_left_command), abs(standard.launch_right_command))
                >= 0.6,
            },
            {
                "variant": "latest_weighted",
                "target_extra_nm_for_0p646_cmd": launch_target_extra,
                "extra_opposing_yaw_torque_nm": latest.launch_extra_nm,
                "total_opposing_yaw_torque_nm": baseline_opposing_yaw_torque(constants, 1.0)
                + latest.launch_extra_nm,
                "left_command": latest.launch_left_command,
                "right_command": latest.launch_right_command,
                "max_abs_command": max(abs(latest.launch_left_command), abs(latest.launch_right_command)),
                "passes_abs_0p6_gate": max(abs(latest.launch_left_command), abs(latest.launch_right_command))
                >= 0.6,
            },
        ]
    )
    launch_table.to_csv(OUT / "in_place_1radps_command.csv", index=False)

    report = f"""# Regime Linear To Unshifted Stribeck Yaw-Support Fit

Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.

## Candidate Equation

The selected candidate is a standalone yaw-support law, expressed as an additional yaw-opposing support magnitude for replacement analysis rather than as `old + residual` runtime logic. It uses kinematic/load inputs only: `abs(yawRate)`, `abs(Vf)`, yaw-contact geometry, and total normal load. The old PlantModel and prior fitted models are used only as external baselines for error reporting.

Let `w = abs(yawRate)`, `v = abs(Vf)`, `N_scale = (N / N_nom)^load_exponent`, and `r_yaw` be the kinematic contact-relative speed per rad/s of yaw.

`a = v * k1`

`b = v * k2 + k3`, with `k1 < k2` and `b - a >= 0`.

`vt2 = v^2 + (rel_weight * r_yaw * w)^2`

`R = speed_fade^2 / (speed_fade^2 + vt2)`

`S = stribeck_speed^2 / (stribeck_speed^2 + vt2)`

Raw unshifted Stribeck branch:

`M_stribeck = N_scale * R * (sliding_nm + static_extra_nm * S)`

Boundary Stribeck value at the end of the linear section:

`vt2_a = v^2 + (rel_weight * r_yaw * a)^2`

`M_a = N_scale * R(vt2_a) * (sliding_nm + static_extra_nm * S(vt2_a))`

Linear branch with derived slope:

`M_linear = M_a * w / a` when `a > 0`; if `a = 0`, the linear region has zero width.

Regime blend:

`M_support = M_linear` for `w <= a`

`M_support = (1 - t) * L_boundary + t * M_stribeck`, `t = clamp((w - a) / (b - a), 0, 1)`, `L_boundary = M_a`, for `a < w < b`

`M_support = M_stribeck` for `w >= b`

Additive yaw torque sign convention for evaluation:

`M_additive = -sign(yawRate) * M_support`

The Stribeck branch is intentionally unshifted: the middle section is a linear blend weight into the raw Stribeck curve, with no vertical offset applied to the Stribeck curve. The linear slope is derived from `M_a / a`, so it is not an independent fitted gain and changes with `Vf`. This run uses the `L_boundary` interpretation for the non-Stribeck side of the blend; that preserves the raw peak instead of extending the line above the Stribeck branch.

## Selected Constants

{markdown_table(selected_params, ["variant", "k1", "k2", "k3_radps", "stribeck_speed_mps", "speed_fade_mps", "rel_weight", "load_exponent", "sliding_nm", "static_extra_nm", "train_weighted_rmse_nm", "primary_rmse_nm", "validation_rmse_nm", "launch_max_abs_command"])}

Nominal load used for scaling: `{nominal_load:.6f} N`.

## In-Place Command Estimate

Synthetic reference: `Vf=0`, `Vr=0`, `yawRate=+1 rad/s`. The command target `+0.646/-0.646` corresponds to extra opposing yaw support `{launch_target_extra:.6f} Nm` with the shared motor-command helper.

{markdown_table(launch_table, ["variant", "extra_opposing_yaw_torque_nm", "total_opposing_yaw_torque_nm", "left_command", "right_command", "max_abs_command", "passes_abs_0p6_gate"])}

## Standard Split Metrics

{markdown_table(split, ["group", "count", "run_count", "baseline_rmse_nm", "corrected_rmse_nm", "rmse_improvement_pct", "corrected_mae_nm", "corrected_median_abs_nm", "run_balanced_corrected_rmse_nm", "median_support_nm"])}

## Selected-Log Metrics

{markdown_table(selected, ["run_id", "dataset_split", "count", "baseline_rmse_nm", "corrected_rmse_nm", "rmse_improvement_pct", "baseline_signed_median_nm", "corrected_signed_median_nm", "median_support_nm"])}

## Latest-Log Behavior

The standard fit includes May 4 rows only through their normal split membership, chiefly downweighted open-floor rows. In standard training weights, April rows contribute `{april_weight_fraction:.3f}` of row-weight and the two May 4 requested latest logs contribute `{latest_weight_fraction:.3f}` of row-weight. Therefore the standard constants are still April-dominated, though not April-only.

Latest-weighted rerun: the two requested May 4 logs were raised to at least weight `3.0` regardless of split. This is a sensitivity check, not the selected standard fit.

{markdown_table(latest_selected[latest_selected["run_id"].isin(["2026-05-04_20-35-47", "2026-05-04_16-57-53"])], ["run_id", "dataset_split", "count", "baseline_rmse_nm", "corrected_rmse_nm", "rmse_improvement_pct", "median_support_nm"])}

## Risk Slices

{markdown_table(risk, ["group", "count", "run_count", "baseline_rmse_nm", "corrected_rmse_nm", "rmse_improvement_pct", "run_balanced_corrected_rmse_nm", "median_support_nm"])}

## Comparison To Existing References

{markdown_table(comp, ["group", "baseline_rmse_nm", "unshifted_corrected_rmse_nm", "force_domain_stribeck_rmse_nm", "rational_residual_reference_rmse_nm", "true_traction_testbed_rmse_nm", "standalone_contact_traction_rmse_nm"])}

Selected-log comparison:

{markdown_table(selected_comp, ["run_id", "dataset_split", "baseline_rmse_nm", "unshifted_corrected_rmse_nm", "force_domain_stribeck_rmse_nm", "true_traction_testbed_rmse_nm", "standalone_contact_traction_rmse_nm"])}

## Interpretation

- The unshifted regime law passes the hard launch gate and lands near the requested `+0.646/-0.646` in-place command scale without command/request or preprojection inputs.
- The standard fit improves the primary/open-floor residual target, but broad validation remains materially worse than the standalone contact-traction and true-traction testbeds because this law is still one-sided yaw-opposing support. It cannot remove rows where the current baseline already over-resists yaw.
- The low-yaw branch is slope-matched to the raw Stribeck boundary for each `Vf`; the launch reference at `Vf=0`, `yawRate=1` enters the ordinary raw Stribeck branch for the selected constants.
- The May 4 latest-weighted rerun moves constants only modestly; the main failure mode is model shape, not simply April log dominance.

## Caveats

- This is not a production `PlantModel` change and was not built or unit-tested by design.
- The residual target is derived against the current PlantModel mirror for evaluation, but the candidate itself does not consume old force outputs at runtime.
- Total normal load comes from the shared feature extraction artifacts; yaw support speed is derived from `abs(yawRate)` and geometry. No logged UKF state-vector fields, command/request values, or preprojection force requests are used as selectors.
- Because the Stribeck term is unshifted, a mismatch between the linear branch and raw Stribeck branch can create a slope/value kink at the end of the transition. That kink is the cost of preserving peak raw Stribeck support without vertical offset.

## Reproduce

```powershell
& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' codex_analysis\\yaw_model_variant_fits\\regime_linear_to_stribeck_unshifted\\fit_unshifted_regime_stribeck.py
```

## Output Files

- `fit_unshifted_regime_stribeck.py`
- `unshifted_report.md`
- `selected_parameters.csv`
- `candidate_scores.csv`
- `standard_split_metrics.csv`
- `standard_selected_log_metrics.csv`
- `standard_risk_metrics.csv`
- `standard_phase_metrics.csv`
- `latest_weighted_split_metrics.csv`
- `latest_weighted_selected_log_metrics.csv`
- `latest_weighted_risk_metrics.csv`
- `baseline_comparison.csv`
- `selected_log_comparison.csv`
- `in_place_1radps_command.csv`
- `prediction_sample.csv`
- `commands_run.txt`
"""
    (OUT / "unshifted_report.md").write_text(report, encoding="utf-8")


def main() -> None:
    constants = read_constants()
    frame, nominal_load = load_frame(constants)
    launch_target_extra = extra_for_target_command(constants, 1.0, 0.646)

    standard, standard_scores = tune_variant(
        frame, nominal_load, constants, "standard", launch_target_extra
    )
    latest, latest_scores = tune_variant(
        frame, nominal_load, constants, "latest_weighted", launch_target_extra
    )
    pd.concat([standard_scores, latest_scores], ignore_index=True).to_csv(
        OUT / "candidate_scores.csv", index=False
    )
    pd.DataFrame([result_to_row(standard), result_to_row(latest)]).to_csv(
        OUT / "selected_parameters.csv", index=False
    )

    standard_frame = add_predictions(frame, nominal_load, constants, standard)
    latest_frame = add_predictions(frame, nominal_load, constants, latest)
    write_metrics(standard_frame, "standard")
    write_metrics(latest_frame, "latest_weighted")

    sample_cols = [
        "run_id",
        "dataset_split",
        "row_index",
        "forward_velocity_mps",
        "yaw_rate_radps",
        "vbar_rel_mps",
        "total_normal_load_n",
        "residual_additive_yaw_torque_nm",
        "residual_opposes_yaw_nm",
        "predicted_opposing_yaw_support_nm",
        "corrected_residual_opposes_yaw_nm",
        "candidate_additive_yaw_torque_nm",
        "corrected_residual_additive_yaw_torque_nm",
    ]
    sample = (
        standard_frame.sort_values(["run_id", "row_index"])
        .groupby("run_id", group_keys=False)
        .head(25)[sample_cols]
    )
    sample.to_csv(OUT / "prediction_sample.csv", index=False)

    write_comparisons()
    write_report(standard, latest, constants, nominal_load, launch_target_extra, frame)
    (OUT / "commands_run.txt").write_text(
        "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' "
        "codex_analysis\\yaw_model_variant_fits\\regime_linear_to_stribeck_unshifted\\fit_unshifted_regime_stribeck.py\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
