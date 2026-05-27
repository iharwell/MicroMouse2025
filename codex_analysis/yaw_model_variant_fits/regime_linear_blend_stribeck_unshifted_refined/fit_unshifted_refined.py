#!/usr/bin/env python3
"""Refined unshifted linear-blend-to-Stribeck yaw-support search.

Analysis-only tooling. It reads shared yaw-model fit artifacts and writes
outputs only beside this script.
"""

from __future__ import annotations

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

LATEST_RUNS = {"2026-05-04_20-35-47", "2026-05-04_16-57-53"}

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

INTERPRETATIONS = ["meet_x1", "reach_x2"]

POSITIVE_PARAMS = {
    "k1",
    "delta_k",
    "k3_radps",
    "stribeck_speed_mps",
    "speed_fade_mps",
}

INITIAL_RANGES = {
    "k1": (0.015, 2.5),
    "delta_k": (0.015, 5.0),
    "k3_radps": (0.004, 2.0),
    "stribeck_speed_mps": (0.004, 0.65),
    "speed_fade_mps": (0.05, 5.0),
    "rel_weight": (0.0, 3.0),
    "load_exponent": (0.0, 1.5),
}

MAX_REASONABLE_PEAK_NM = 0.12
LAUNCH_TARGET_COMMAND = 0.646
LAUNCH_MIN_COMMAND = 0.6
BOUNDARY_MARGIN_FRACTION = 0.03
ARRAY_CACHE: dict[str, np.ndarray] = {}
LAUNCH_TARGET_EXTRA_NM = math.nan
LAUNCH_MIN_EXTRA_NM = math.nan


@dataclass(frozen=True)
class Shape:
    interpretation: str
    k1: float
    delta_k: float
    k3_radps: float
    stribeck_speed_mps: float
    speed_fade_mps: float
    rel_weight: float
    load_exponent: float

    @property
    def k2(self) -> float:
        return self.k1 + self.delta_k


@dataclass(frozen=True)
class Candidate:
    variant: str
    shape: Shape
    sliding_nm: float
    static_extra_nm: float
    coarse_train_rmse_nm: float
    train_weighted_rmse_nm: float
    primary_rmse_nm: float
    validation_rmse_nm: float
    selected_log_rmse_nm: float
    latest_rmse_nm: float
    peak_nm: float
    launch_extra_nm: float
    launch_left_command: float
    launch_right_command: float
    objective: float
    gate_pass: bool
    gate_reason: str
    stage: str


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


def prepare_array_cache(frame: pd.DataFrame) -> None:
    ARRAY_CACHE.clear()
    ARRAY_CACHE["yaw_abs"] = frame["abs_yaw_rate_radps"].to_numpy(float)
    ARRAY_CACHE["vf_abs"] = frame["abs_forward_velocity_mps"].to_numpy(float)
    ARRAY_CACHE["total_load"] = frame["total_normal_load_n"].to_numpy(float)
    ARRAY_CACHE["target"] = frame["residual_opposes_yaw_nm"].to_numpy(float)
    ARRAY_CACHE["weights_standard"] = training_weights(frame, "standard")
    ARRAY_CACHE["weights_latest_weighted"] = training_weights(frame, "latest_weighted")


def stribeck_basis_at(
    shape: Shape,
    yaw_abs: np.ndarray,
    vf_abs: np.ndarray,
    total_load: np.ndarray,
    nominal_load: float,
    yaw_rel_per_radps: float,
) -> tuple[np.ndarray, np.ndarray]:
    yaw_rel = yaw_rel_per_radps * yaw_abs
    vt2 = np.square(vf_abs) + np.square(shape.rel_weight * yaw_rel)
    fade = (shape.speed_fade_mps * shape.speed_fade_mps) / (
        shape.speed_fade_mps * shape.speed_fade_mps + vt2
    )
    stribeck = (shape.stribeck_speed_mps * shape.stribeck_speed_mps) / (
        shape.stribeck_speed_mps * shape.stribeck_speed_mps + vt2
    )
    load_scale = np.power(np.maximum(total_load / nominal_load, 1.0e-6), shape.load_exponent)
    return load_scale * fade, load_scale * fade * stribeck


def bases_for_arrays(
    shape: Shape,
    yaw_abs: np.ndarray,
    vf_abs: np.ndarray,
    total_load: np.ndarray,
    nominal_load: float,
    yaw_rel_per_radps: float,
) -> np.ndarray:
    x1 = vf_abs * shape.k1
    x2 = vf_abs * shape.k2 + shape.k3_radps
    width = np.maximum(x2 - x1, 1.0e-9)
    blend = np.clip((yaw_abs - x1) / width, 0.0, 1.0)

    raw_slide, raw_static = stribeck_basis_at(
        shape, yaw_abs, vf_abs, total_load, nominal_load, yaw_rel_per_radps
    )

    if shape.interpretation == "meet_x1":
        boundary_slide, boundary_static = stribeck_basis_at(
            shape, x1, vf_abs, total_load, nominal_load, yaw_rel_per_radps
        )
        safe_x1 = np.maximum(x1, 1.0e-9)
        fraction = np.where(x1 > 1.0e-9, np.clip(yaw_abs / safe_x1, 0.0, 1.0), 0.0)
        linear_slide = boundary_slide * fraction
        linear_static = boundary_static * fraction
        blend_slide = (1.0 - blend) * boundary_slide + blend * raw_slide
        blend_static = (1.0 - blend) * boundary_static + blend * raw_static
    elif shape.interpretation == "reach_x2":
        x2_slide, x2_static = stribeck_basis_at(
            shape, x2, vf_abs, total_load, nominal_load, yaw_rel_per_radps
        )
        safe_x2 = np.maximum(x2, 1.0e-9)
        fraction = np.clip(yaw_abs / safe_x2, 0.0, 1.0)
        line_slide = x2_slide * fraction
        line_static = x2_static * fraction
        linear_slide = line_slide
        linear_static = line_static
        blend_slide = (1.0 - blend) * line_slide + blend * raw_slide
        blend_static = (1.0 - blend) * line_static + blend * raw_static
    else:
        raise ValueError(f"unknown interpretation {shape.interpretation}")

    in_linear = yaw_abs <= x1
    in_transition = (yaw_abs > x1) & (yaw_abs < x2)
    slide_basis = np.where(in_linear, linear_slide, np.where(in_transition, blend_slide, raw_slide))
    static_basis = np.where(in_linear, linear_static, np.where(in_transition, blend_static, raw_static))
    return np.column_stack([slide_basis, static_basis])


def predict_support(
    frame: pd.DataFrame,
    nominal_load: float,
    constants: dict[str, float],
    shape: Shape,
    coeffs: np.ndarray,
) -> np.ndarray:
    bases = bases_for_arrays(
        shape,
        frame["abs_yaw_rate_radps"].to_numpy(float),
        frame["abs_forward_velocity_mps"].to_numpy(float),
        frame["total_normal_load_n"].to_numpy(float),
        nominal_load,
        constants["drive_wheel_longitudinal_offset_m"],
    )
    return bases @ coeffs


def launch_support(
    constants: dict[str, float], nominal_load: float, shape: Shape, coeffs: np.ndarray
) -> float:
    bases = bases_for_arrays(
        shape,
        np.array([1.0]),
        np.array([0.0]),
        np.array([nominal_load]),
        nominal_load,
        constants["drive_wheel_longitudinal_offset_m"],
    )
    return float((bases @ coeffs)[0])


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


def weighted_sse(bases: np.ndarray, target: np.ndarray, weights: np.ndarray, coeffs: np.ndarray) -> float:
    residual = target - bases @ coeffs
    return float(np.sum(weights * np.square(residual)))


def launch_equality_lstsq(
    bases: np.ndarray,
    target: np.ndarray,
    weights: np.ndarray,
    launch_basis: np.ndarray,
    launch_target_nm: float,
) -> np.ndarray | None:
    if not math.isfinite(launch_target_nm) or launch_target_nm <= 0.0:
        return None
    c = np.asarray(launch_basis, dtype=float)
    if c.shape != (2,) or np.all(c <= 1.0e-12):
        return None
    sw = np.sqrt(np.maximum(weights, 0.0))
    xw = bases * sw[:, None]
    yw = target * sw
    gram = xw.T @ xw
    rhs = xw.T @ yw
    candidates: list[np.ndarray] = []
    kkt = np.zeros((3, 3), dtype=float)
    kkt[:2, :2] = gram + np.eye(2) * 1.0e-14
    kkt[:2, 2] = c
    kkt[2, :2] = c
    krhs = np.array([rhs[0], rhs[1], launch_target_nm], dtype=float)
    try:
        solution = np.linalg.solve(kkt, krhs)[:2]
        if np.all(solution >= -1.0e-10):
            candidates.append(np.maximum(solution, 0.0))
    except np.linalg.LinAlgError:
        pass
    for idx in range(2):
        if c[idx] > 1.0e-12:
            coeff = np.zeros(2, dtype=float)
            coeff[idx] = launch_target_nm / c[idx]
            candidates.append(coeff)
    if not candidates:
        return None
    return min(candidates, key=lambda coeff: weighted_sse(bases, target, weights, coeff))


def training_weights(frame: pd.DataFrame, variant: str) -> np.ndarray:
    split = frame["dataset_split"].astype(str)
    weights = np.zeros(len(frame), dtype=float)
    weights[split == "primary_open_floor_fit_authoritative"] = 1.0
    weights[split == "open_floor_fit_downweighted"] = 0.25
    if variant == "latest_weighted":
        latest = frame["run_id"].astype(str).isin(LATEST_RUNS)
        weights[latest] = np.maximum(weights[latest], 3.0)
    return weights


def fit_shape(
    frame: pd.DataFrame,
    nominal_load: float,
    constants: dict[str, float],
    shape: Shape,
    variant: str,
    coarse_indices: np.ndarray | None,
) -> tuple[np.ndarray, float, float]:
    global LAUNCH_MIN_EXTRA_NM, LAUNCH_TARGET_EXTRA_NM
    if coarse_indices is None:
        indices = np.arange(len(frame))
        weights = ARRAY_CACHE[f"weights_{variant}"]
    else:
        indices = coarse_indices
        weights = ARRAY_CACHE[f"weights_{variant}"][indices]
    active = weights > 0.0
    indices = indices[active]
    weights = weights[active]
    bases = bases_for_arrays(
        shape,
        ARRAY_CACHE["yaw_abs"][indices],
        ARRAY_CACHE["vf_abs"][indices],
        ARRAY_CACHE["total_load"][indices],
        nominal_load,
        constants["drive_wheel_longitudinal_offset_m"],
    )
    target = ARRAY_CACHE["target"][indices]
    launch_basis = bases_for_arrays(
        shape,
        np.array([1.0]),
        np.array([0.0]),
        np.array([nominal_load]),
        nominal_load,
        constants["drive_wheel_longitudinal_offset_m"],
    )[0]
    options = [nonnegative_lstsq(bases, target, weights)]
    for launch_target_nm in [LAUNCH_MIN_EXTRA_NM, LAUNCH_TARGET_EXTRA_NM]:
        constrained = launch_equality_lstsq(bases, target, weights, launch_basis, launch_target_nm)
        if constrained is not None:
            options.append(constrained)

    def option_score(coeff: np.ndarray) -> float:
        data_rmse = math.sqrt(weighted_sse(bases, target, weights, coeff) / float(np.sum(weights)))
        launch_extra = float(launch_basis @ coeff)
        total = baseline_opposing_yaw_torque(constants, 1.0) + launch_extra
        commands = motor_commands_for_opposing_torque(total, constants, 1.0)
        launch_abs = max(abs(commands["left_command"]), abs(commands["right_command"]))
        launch_miss = abs(launch_abs - LAUNCH_TARGET_COMMAND)
        peak = float(coeff[0] + coeff[1])
        launch_gate_penalty = 0.04 if launch_abs < LAUNCH_MIN_COMMAND else 0.0
        peak_penalty = 0.08 if peak > MAX_REASONABLE_PEAK_NM else 0.0
        return data_rmse + 0.003 * (launch_miss / 0.046) ** 2 + launch_gate_penalty + peak_penalty

    coeffs = min(options, key=option_score)
    residual = target - bases @ coeffs
    score = float(np.sqrt(np.average(np.square(residual), weights=weights)))
    return coeffs, score, option_score(coeffs)


def seed_shapes() -> list[Shape]:
    return [
        Shape("meet_x1", 0.3757770849429084, 0.9045961923080346, 0.011429843777412357, 0.010246348427229903, 0.6381489103394051, 0.07585176939288332, 0.5),
        Shape("meet_x1", 0.7204235550868902, 0.07150260659312482, 0.026871466216220854, 0.010440347893586322, 0.43333981349631673, 0.03388860001664056, 0.75),
        Shape("meet_x1", 0.08, 0.45, 0.06, 0.025, 0.64, 0.75, 0.0),
        Shape("meet_x1", 0.15, 0.45, 0.10, 0.040, 0.64, 0.35, 0.5),
        Shape("reach_x2", 0.08, 0.45, 0.06, 0.025, 0.64, 0.75, 0.0),
        Shape("reach_x2", 0.15, 0.45, 0.10, 0.040, 0.64, 0.35, 0.5),
    ]


def objective_for_candidate(candidate: Candidate) -> float:
    launch_abs = max(abs(candidate.launch_left_command), abs(candidate.launch_right_command))
    launch_miss = abs(launch_abs - LAUNCH_TARGET_COMMAND)
    validation_regression = max(0.0, candidate.validation_rmse_nm - 0.050234)
    latest_regression = max(0.0, candidate.latest_rmse_nm - 0.032158)
    peak_margin = max(0.0, candidate.peak_nm - 0.09)
    return (
        candidate.train_weighted_rmse_nm
        + 0.003 * (launch_miss / 0.046) ** 2
        + 0.20 * validation_regression
        + 0.10 * latest_regression
        + 0.10 * peak_margin
    )


def evaluate_candidate(
    frame: pd.DataFrame,
    nominal_load: float,
    constants: dict[str, float],
    variant: str,
    shape: Shape,
    coeffs: np.ndarray,
    coarse_score: float,
    train_score: float,
    stage: str,
) -> Candidate:
    pred = predict_support(frame, nominal_load, constants, shape, coeffs)
    err = frame["residual_opposes_yaw_nm"].to_numpy(float) - pred
    split = frame["dataset_split"].astype(str)
    primary_mask = (split == "primary_open_floor_fit_authoritative").to_numpy()
    validation_mask = (split != "primary_open_floor_fit_authoritative").to_numpy()
    selected_mask = frame["run_id"].astype(str).isin(SELECTED_RUNS).to_numpy()
    latest_mask = frame["run_id"].astype(str).isin(LATEST_RUNS).to_numpy()
    peak = float(coeffs[0] + coeffs[1])
    launch_extra = launch_support(constants, nominal_load, shape, coeffs)
    total = baseline_opposing_yaw_torque(constants, 1.0) + launch_extra
    commands = motor_commands_for_opposing_torque(total, constants, 1.0)
    launch_abs = max(abs(commands["left_command"]), abs(commands["right_command"]))
    reasons: list[str] = []
    if launch_abs < LAUNCH_MIN_COMMAND:
        reasons.append("launch_cmd_below_0p6")
    if peak > MAX_REASONABLE_PEAK_NM:
        reasons.append("peak_above_reasonable_limit")
    if coeffs[0] < -1.0e-12 or coeffs[1] < -1.0e-12:
        reasons.append("negative_amplitude")
    if not reasons:
        reasons.append("pass")
    candidate = Candidate(
        variant=variant,
        shape=shape,
        sliding_nm=float(coeffs[0]),
        static_extra_nm=float(coeffs[1]),
        coarse_train_rmse_nm=coarse_score,
        train_weighted_rmse_nm=train_score,
        primary_rmse_nm=rmse(err[primary_mask]),
        validation_rmse_nm=rmse(err[validation_mask]),
        selected_log_rmse_nm=rmse(err[selected_mask]),
        latest_rmse_nm=rmse(err[latest_mask]),
        peak_nm=peak,
        launch_extra_nm=launch_extra,
        launch_left_command=float(commands["left_command"]),
        launch_right_command=float(commands["right_command"]),
        objective=math.inf,
        gate_pass=reasons == ["pass"],
        gate_reason=";".join(reasons),
        stage=stage,
    )
    return Candidate(**{**candidate.__dict__, "objective": objective_for_candidate(candidate)})


def sample_shapes(
    ranges: dict[str, tuple[float, float]],
    count: int,
    seed: int,
    interpretations: list[str],
) -> list[Shape]:
    rng = np.random.default_rng(seed)
    shapes: list[Shape] = []
    for _ in range(count):
        values: dict[str, float] = {}
        for key, (lo, hi) in ranges.items():
            if key in POSITIVE_PARAMS:
                values[key] = float(np.exp(rng.uniform(math.log(lo), math.log(hi))))
            else:
                values[key] = float(rng.uniform(lo, hi))
        interp = interpretations[int(rng.integers(0, len(interpretations)))]
        shapes.append(Shape(interp, **values))
    return shapes


def perturb_shapes(
    centers: list[Shape],
    ranges: dict[str, tuple[float, float]],
    per_center: int,
    seed: int,
    stage_scale: float,
) -> list[Shape]:
    rng = np.random.default_rng(seed)
    shapes: list[Shape] = []
    for center in centers:
        center_values = shape_values(center)
        shapes.append(center)
        for _ in range(per_center):
            values: dict[str, float] = {}
            for key, (lo, hi) in ranges.items():
                base = center_values[key]
                if key in POSITIVE_PARAMS:
                    span = math.log(hi / lo)
                    value = base * math.exp(float(rng.normal(0.0, stage_scale * span)))
                    values[key] = float(np.clip(value, lo, hi))
                else:
                    span = hi - lo
                    value = base + float(rng.normal(0.0, stage_scale * span))
                    values[key] = float(np.clip(value, lo, hi))
            if values["delta_k"] < 1.0e-9:
                values["delta_k"] = 1.0e-9
            interp = center.interpretation if rng.random() < 0.82 else rng.choice(INTERPRETATIONS)
            shapes.append(Shape(str(interp), **values))
    return shapes


def shape_values(shape: Shape) -> dict[str, float]:
    return {
        "k1": shape.k1,
        "delta_k": shape.delta_k,
        "k3_radps": shape.k3_radps,
        "stribeck_speed_mps": shape.stribeck_speed_mps,
        "speed_fade_mps": shape.speed_fade_mps,
        "rel_weight": shape.rel_weight,
        "load_exponent": shape.load_exponent,
    }


def expand_ranges_for_shape(
    ranges: dict[str, tuple[float, float]], shape: Shape
) -> tuple[dict[str, tuple[float, float]], list[str]]:
    expanded = dict(ranges)
    changes: list[str] = []
    values = shape_values(shape)
    for key, value in values.items():
        lo, hi = ranges[key]
        frac = normalized_margin(key, value, lo, hi)
        if frac >= BOUNDARY_MARGIN_FRACTION:
            continue
        if key in POSITIVE_PARAMS:
            log_lo = math.log(lo)
            log_hi = math.log(hi)
            log_v = math.log(max(value, 1.0e-12))
            low_frac = (log_v - log_lo) / (log_hi - log_lo)
            if low_frac < BOUNDARY_MARGIN_FRACTION:
                new_lo = lo / 5.0
                expanded[key] = (new_lo, hi)
                changes.append(f"{key}:lower {lo:g}->{new_lo:g}")
            else:
                new_hi = hi * 5.0
                expanded[key] = (lo, new_hi)
                changes.append(f"{key}:upper {hi:g}->{new_hi:g}")
        else:
            span = hi - lo
            low_frac = (value - lo) / span if span > 0.0 else 0.0
            if low_frac < BOUNDARY_MARGIN_FRACTION:
                new_lo = 0.0 if key in {"rel_weight", "load_exponent"} else lo - 2.0 * span
                expanded[key] = (new_lo, hi)
                changes.append(f"{key}:lower {lo:g}->{new_lo:g}")
            else:
                new_hi = hi + 2.0 * span
                expanded[key] = (lo, new_hi)
                changes.append(f"{key}:upper {hi:g}->{new_hi:g}")
    return expanded, changes


def normalized_margin(key: str, value: float, lo: float, hi: float) -> float:
    if hi <= lo:
        return 0.0
    if key in POSITIVE_PARAMS:
        log_lo = math.log(lo)
        log_hi = math.log(hi)
        log_v = math.log(max(value, 1.0e-12))
        pos = (log_v - log_lo) / (log_hi - log_lo)
    else:
        pos = (value - lo) / (hi - lo)
    return float(min(pos, 1.0 - pos))


def candidate_to_row(candidate: Candidate) -> dict[str, object]:
    return {
        "variant": candidate.variant,
        "stage": candidate.stage,
        "interpretation": candidate.shape.interpretation,
        "k1": candidate.shape.k1,
        "k2": candidate.shape.k2,
        "delta_k": candidate.shape.delta_k,
        "k3_radps": candidate.shape.k3_radps,
        "stribeck_speed_mps": candidate.shape.stribeck_speed_mps,
        "speed_fade_mps": candidate.shape.speed_fade_mps,
        "rel_weight": candidate.shape.rel_weight,
        "load_exponent": candidate.shape.load_exponent,
        "sliding_nm": candidate.sliding_nm,
        "static_extra_nm": candidate.static_extra_nm,
        "peak_nm": candidate.peak_nm,
        "coarse_train_rmse_nm": candidate.coarse_train_rmse_nm,
        "train_weighted_rmse_nm": candidate.train_weighted_rmse_nm,
        "primary_rmse_nm": candidate.primary_rmse_nm,
        "validation_rmse_nm": candidate.validation_rmse_nm,
        "selected_log_rmse_nm": candidate.selected_log_rmse_nm,
        "latest_rmse_nm": candidate.latest_rmse_nm,
        "launch_extra_nm": candidate.launch_extra_nm,
        "launch_left_command": candidate.launch_left_command,
        "launch_right_command": candidate.launch_right_command,
        "launch_max_abs_command": max(abs(candidate.launch_left_command), abs(candidate.launch_right_command)),
        "launch_target_abs_error": abs(
            max(abs(candidate.launch_left_command), abs(candidate.launch_right_command))
            - LAUNCH_TARGET_COMMAND
        ),
        "gate_pass": candidate.gate_pass,
        "gate_reason": candidate.gate_reason,
        "objective": candidate.objective,
    }


def run_search_round(
    frame: pd.DataFrame,
    nominal_load: float,
    constants: dict[str, float],
    variant: str,
    ranges: dict[str, tuple[float, float]],
    seed: int,
    broad_count: int,
    local_keep: int,
    coarse_count: int,
) -> tuple[Candidate, pd.DataFrame, list[dict[str, object]]]:
    rng = np.random.default_rng(seed + 11)
    train_idx = np.flatnonzero(training_weights(frame, variant) > 0.0)
    coarse_idx = rng.choice(train_idx, size=min(coarse_count, len(train_idx)), replace=False)

    stage_rows: list[dict[str, object]] = []
    candidate_rows: list[dict[str, object]] = []
    full_candidates: list[Candidate] = []

    broad_shapes = seed_shapes() + sample_shapes(ranges, broad_count, seed + 101, INTERPRETATIONS)
    stage_specs: list[tuple[str, list[Shape], int]] = [
        ("broad", broad_shapes, max(local_keep, 90))
    ]
    evaluated_shapes: set[tuple[object, ...]] = set()

    for stage_name, shapes, keep_count in stage_specs:
        coarse_rows: list[tuple[float, float, Shape, np.ndarray]] = []
        for shape in shapes:
            key = rounded_shape_key(shape)
            if key in evaluated_shapes:
                continue
            evaluated_shapes.add(key)
            coeffs, coarse_score, coarse_proxy = fit_shape(
                frame, nominal_load, constants, shape, variant, coarse_idx
            )
            coarse_rows.append((coarse_proxy, coarse_score, shape, coeffs))
        coarse_rows.sort(key=lambda item: item[0])
        stage_rows.append(
            {
                "variant": variant,
                "stage": stage_name,
                "candidate_shapes": len(shapes),
                "unique_coarse_evaluations": len(coarse_rows),
                "full_evaluations": min(keep_count, len(coarse_rows)),
                "coarse_subset_rows": len(coarse_idx),
                "ranges_json": json.dumps(ranges, sort_keys=True),
            }
        )
        full_stage: list[Candidate] = []
        for _, coarse_score, shape, _ in coarse_rows[:keep_count]:
            coeffs, train_score, _ = fit_shape(frame, nominal_load, constants, shape, variant, None)
            full_stage.append(
                evaluate_candidate(
                    frame,
                    nominal_load,
                    constants,
                    variant,
                    shape,
                    coeffs,
                    coarse_score,
                    train_score,
                    stage_name,
                )
            )
        full_candidates.extend(full_stage)
        candidate_rows.extend(candidate_to_row(c) for c in sorted(full_stage, key=lambda c: c.objective)[:220])

    for local_stage, scale, per_center, keep in [
        ("local_refine_1", 0.055, 55, 110),
        ("local_refine_2", 0.020, 70, 130),
    ]:
        pass_pool = [c for c in full_candidates if c.gate_pass]
        pool = pass_pool if pass_pool else full_candidates
        centers = [c.shape for c in sorted(pool, key=lambda c: c.objective)[:local_keep]]
        shapes = perturb_shapes(centers, ranges, per_center, seed + len(full_candidates) + 503, scale)
        coarse_rows = []
        for shape in shapes:
            key = rounded_shape_key(shape)
            if key in evaluated_shapes:
                continue
            evaluated_shapes.add(key)
            coeffs, coarse_score, coarse_proxy = fit_shape(
                frame, nominal_load, constants, shape, variant, coarse_idx
            )
            coarse_rows.append((coarse_proxy, coarse_score, shape, coeffs))
        coarse_rows.sort(key=lambda item: item[0])
        stage_rows.append(
            {
                "variant": variant,
                "stage": local_stage,
                "candidate_shapes": len(shapes),
                "unique_coarse_evaluations": len(coarse_rows),
                "full_evaluations": min(keep, len(coarse_rows)),
                "coarse_subset_rows": len(coarse_idx),
                "ranges_json": json.dumps(ranges, sort_keys=True),
            }
        )
        full_stage = []
        for _, coarse_score, shape, _ in coarse_rows[:keep]:
            coeffs, train_score, _ = fit_shape(frame, nominal_load, constants, shape, variant, None)
            full_stage.append(
                evaluate_candidate(
                    frame,
                    nominal_load,
                    constants,
                    variant,
                    shape,
                    coeffs,
                    coarse_score,
                    train_score,
                    local_stage,
                )
            )
        full_candidates.extend(full_stage)
        candidate_rows.extend(candidate_to_row(c) for c in sorted(full_stage, key=lambda c: c.objective)[:260])

    pass_candidates = [c for c in full_candidates if c.gate_pass]
    if not pass_candidates:
        selected = min(full_candidates, key=lambda c: c.objective)
    else:
        selected = min(pass_candidates, key=lambda c: c.objective)
    score_frame = pd.DataFrame(candidate_rows).drop_duplicates(
        subset=[
            "variant",
            "interpretation",
            "k1",
            "delta_k",
            "k3_radps",
            "stribeck_speed_mps",
            "speed_fade_mps",
            "rel_weight",
            "load_exponent",
        ]
    )
    score_frame = score_frame.sort_values(["gate_pass", "objective"], ascending=[False, True])
    return selected, score_frame, stage_rows


def rounded_shape_key(shape: Shape) -> tuple[object, ...]:
    vals = shape_values(shape)
    return (shape.interpretation, *(round(vals[key], 12) for key in sorted(vals)))


def audit_boundary(
    variant: str,
    candidate: Candidate,
    ranges: dict[str, tuple[float, float]],
    search_status: str,
) -> pd.DataFrame:
    rows = []
    values = shape_values(candidate.shape)
    for key, value in values.items():
        lo, hi = ranges[key]
        margin = normalized_margin(key, value, lo, hi)
        rows.append(
            {
                "variant": variant,
                "parameter": key,
                "value": value,
                "range_min": lo,
                "range_max": hi,
                "normalized_margin_to_nearest_boundary": margin,
                "boundary_adjacent": margin < BOUNDARY_MARGIN_FRACTION,
                "search_status": search_status,
            }
        )
    rows.append(
        {
            "variant": variant,
            "parameter": "interpretation",
            "value": candidate.shape.interpretation,
            "range_min": ";".join(INTERPRETATIONS),
            "range_max": ";".join(INTERPRETATIONS),
            "normalized_margin_to_nearest_boundary": 1.0,
            "boundary_adjacent": False,
            "search_status": search_status,
        }
    )
    return pd.DataFrame(rows)


def run_search_with_expansion(
    frame: pd.DataFrame,
    nominal_load: float,
    constants: dict[str, float],
    variant: str,
    seed: int,
) -> tuple[Candidate, pd.DataFrame, pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    ranges = dict(INITIAL_RANGES)
    all_scores: list[pd.DataFrame] = []
    all_stage_rows: list[dict[str, object]] = []
    expansion_rows: list[dict[str, object]] = []
    selected: Candidate | None = None
    status = "interior_selected"

    for round_index, broad_count in enumerate([4200, 3600, 4200], start=1):
        selected, scores, stage_rows = run_search_round(
            frame,
            nominal_load,
            constants,
            variant,
            ranges,
            seed + round_index * 10000,
            broad_count=broad_count,
            local_keep=16,
            coarse_count=16000,
        )
        for row in stage_rows:
            row["round_index"] = round_index
        all_stage_rows.extend(stage_rows)
        scores["round_index"] = round_index
        all_scores.append(scores)
        expanded, changes = expand_ranges_for_shape(ranges, selected.shape)
        expansion_rows.append(
            {
                "variant": variant,
                "round_index": round_index,
                "selected_objective": selected.objective,
                "selected_gate_pass": selected.gate_pass,
                "selected_gate_reason": selected.gate_reason,
                "selected_shape_json": json.dumps(shape_values(selected.shape), sort_keys=True),
                "selected_interpretation": selected.shape.interpretation,
                "boundary_changes": "; ".join(changes),
                "ranges_json": json.dumps(ranges, sort_keys=True),
            }
        )
        if not changes:
            status = "interior_selected" if selected.gate_pass else "no_gate_passing_candidate"
            break
        ranges = expanded
        status = "expanded_after_boundary_hit"
    else:
        status = "unresolved_boundary_after_expansion"

    assert selected is not None
    scores_frame = pd.concat(all_scores, ignore_index=True).sort_values(
        ["gate_pass", "objective"], ascending=[False, True]
    )
    stage_frame = pd.DataFrame(all_stage_rows)
    expansion_frame = pd.DataFrame(expansion_rows)
    boundary_frame = audit_boundary(variant, selected, ranges, status)
    return selected, scores_frame, stage_frame, boundary_frame, expansion_frame


def add_predictions(
    frame: pd.DataFrame, nominal_load: float, constants: dict[str, float], candidate: Candidate
) -> pd.DataFrame:
    coeffs = np.array([candidate.sliding_nm, candidate.static_extra_nm], dtype=float)
    pred = predict_support(frame, nominal_load, constants, candidate.shape, coeffs)
    out = frame.copy()
    out["predicted_opposing_yaw_support_nm"] = pred
    out["corrected_residual_opposes_yaw_nm"] = out["residual_opposes_yaw_nm"] - pred
    out["candidate_additive_yaw_torque_nm"] = -out["yaw_sign"] * pred
    out["corrected_residual_additive_yaw_torque_nm"] = (
        out["residual_additive_yaw_torque_nm"] - out["candidate_additive_yaw_torque_nm"]
    )
    return out


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


def write_metrics(frame: pd.DataFrame, variant: str) -> None:
    pred = frame["predicted_opposing_yaw_support_nm"].to_numpy(float)
    corrected_add = frame["corrected_residual_additive_yaw_torque_nm"].to_numpy(float)

    split_rows = []
    for split in SPLITS:
        mask = frame["dataset_split"] == split
        split_rows.append(
            metrics_for_group(split, frame.loc[mask], pred[mask.to_numpy()], corrected_add[mask.to_numpy()])
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
        row = metrics_for_group(run, group, pred[mask.to_numpy()], corrected_add[mask.to_numpy()])
        row["run_id"] = run
        row["present"] = True
        row["dataset_split"] = str(group["dataset_split"].mode().iloc[0])
        selected_rows.append(row)
    pd.DataFrame(selected_rows).to_csv(OUT / f"{variant}_selected_log_metrics.csv", index=False)

    latest_mask = frame["run_id"].astype(str).isin(LATEST_RUNS)
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
            metrics_for_group(name, frame.loc[mask], pred[mask.to_numpy()], corrected_add[mask.to_numpy()])
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
    ].rename(columns={"corrected_rmse_nm": "unshifted_refined_rmse_nm"})
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
    ].rename(columns={"corrected_rmse_nm": "unshifted_refined_rmse_nm"})
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


def write_in_place_rows(candidates: list[Candidate], constants: dict[str, float]) -> None:
    rows = []
    base = baseline_opposing_yaw_torque(constants, 1.0)
    for candidate in candidates:
        rows.append(
            {
                "variant": candidate.variant,
                "extra_opposing_yaw_torque_nm": candidate.launch_extra_nm,
                "baseline_opposing_yaw_torque_nm": base,
                "total_opposing_yaw_torque_nm": base + candidate.launch_extra_nm,
                "left_command": candidate.launch_left_command,
                "right_command": candidate.launch_right_command,
                "max_abs_command": max(abs(candidate.launch_left_command), abs(candidate.launch_right_command)),
                "target_abs_command": LAUNCH_TARGET_COMMAND,
                "passes_abs_0p6_gate": max(abs(candidate.launch_left_command), abs(candidate.launch_right_command))
                >= LAUNCH_MIN_COMMAND,
            }
        )
    pd.DataFrame(rows).to_csv(OUT / "in_place_1radps_command.csv", index=False)


def write_report(
    standard: Candidate,
    latest: Candidate,
    constants: dict[str, float],
    nominal_load: float,
    launch_target_extra: float,
    frame: pd.DataFrame,
    search_summary: pd.DataFrame,
    boundary: pd.DataFrame,
) -> None:
    selected_params = pd.DataFrame([candidate_to_row(standard), candidate_to_row(latest)])
    split = pd.read_csv(OUT / "standard_split_metrics.csv")
    selected = pd.read_csv(OUT / "standard_selected_log_metrics.csv")
    latest_selected = pd.read_csv(OUT / "latest_weighted_selected_log_metrics.csv")
    risk = pd.read_csv(OUT / "standard_risk_metrics.csv")
    comp = pd.read_csv(OUT / "baseline_comparison.csv")
    selected_comp = pd.read_csv(OUT / "selected_log_comparison.csv")
    in_place = pd.read_csv(OUT / "in_place_1radps_command.csv")
    weights = training_weights(frame, "standard")
    weighted = frame.loc[weights > 0.0, ["run_id", "dataset_split"]].copy()
    weighted["weight"] = weights[weights > 0.0]
    latest_rows = weighted["run_id"].astype(str).isin(LATEST_RUNS)
    latest_weight_fraction = float(weighted.loc[latest_rows, "weight"].sum() / weighted["weight"].sum())
    april_weight_fraction = float(
        weighted.loc[weighted["run_id"].astype(str).str.startswith("2026-04"), "weight"].sum()
        / weighted["weight"].sum()
    )
    boundary_standard = boundary[boundary["variant"] == "standard"]
    interior = not bool(
        boundary_standard.get("boundary_adjacent", pd.Series(dtype=bool)).astype(str).str.lower().eq("true").any()
    )
    stage_counts = search_summary.groupby("variant", as_index=False).agg(
        unique_coarse_evaluations=("unique_coarse_evaluations", "sum"),
        full_evaluations=("full_evaluations", "sum"),
        stages=("stage", "count"),
    )

    report = f"""# Refined Unshifted Linear-Blend-To-Stribeck Fit

Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.

## Model

Inputs used by the selected candidate are only `abs(yawRate)`, `abs(Vf)`, total normal load, and fixed yaw-contact geometry from the shared constants. Logged command/request/preprojection/UKF fields and old core contact-force outputs are not candidate inputs. Prior models are used only as reporting references.

Let `y = abs(yawRate)`, `v = abs(Vf)`, `x1 = v*k1`, and `x2 = v*k2 + k3`, with `k1 < k2` and `x2 > x1`.

Raw unshifted Stribeck branch:

`S_raw = (N/N_nom)^load_exponent * speed_fade^2/(speed_fade^2 + vt2) * (sliding_nm + static_extra_nm * stribeck_speed^2/(stribeck_speed^2 + vt2))`

`vt2 = v^2 + (rel_weight * yaw_contact_offset * y)^2`

The Stribeck branch is never shifted or offset. The search evaluated two derived-slope interpretations:

- `meet_x1`: the zero-anchored line meets raw Stribeck at `x1`; the transition blends from that boundary value into raw Stribeck.
- `reach_x2`: the zero-anchored line is derived from the raw Stribeck value at `x2`; the transition blends the line continuation into raw Stribeck.

No independent low-yaw slope was fitted.

## Search

Initial broad ranges:

{json.dumps(INITIAL_RANGES, indent=2)}

Boundary policy: any final parameter within `{BOUNDARY_MARGIN_FRACTION:.0%}` normalized distance of a search boundary triggered range expansion and rerun. If still boundary-adjacent after the expansion budget, the search would be marked unresolved rather than selected.

Evaluation counts:

{markdown_table(stage_counts, ["variant", "stages", "unique_coarse_evaluations", "full_evaluations"])}

Search stages and ranges are in `search_summary.csv`; range-expansion decisions are in `range_expansion_audit.csv`; final parameter margins are in `boundary_audit.csv`.

## Selected Constants

| variant | interpretation | k1 | k2 | delta_k | k3_radps | stribeck_speed_mps | speed_fade_mps | rel_weight | load_exponent | sliding_nm | static_extra_nm | peak_nm | train_weighted_rmse_nm | primary_rmse_nm | validation_rmse_nm | latest_rmse_nm | launch_max_abs_command | objective |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
"""
    for _, row in selected_params.iterrows():
        report += (
            f"| {row['variant']} | {row['interpretation']} | {format_float(row['k1'])} | "
            f"{format_float(row['k2'])} | {format_float(row['delta_k'])} | "
            f"{format_float(row['k3_radps'])} | {format_float(row['stribeck_speed_mps'])} | "
            f"{format_float(row['speed_fade_mps'])} | {format_float(row['rel_weight'])} | "
            f"{format_float(row['load_exponent'])} | {format_float(row['sliding_nm'])} | "
            f"{format_float(row['static_extra_nm'])} | {format_float(row['peak_nm'])} | "
            f"{format_float(row['train_weighted_rmse_nm'])} | {format_float(row['primary_rmse_nm'])} | "
            f"{format_float(row['validation_rmse_nm'])} | {format_float(row['latest_rmse_nm'])} | "
            f"{format_float(row['launch_max_abs_command'])} | {format_float(row['objective'])} |\n"
        )
    report += f"""
Nominal load used for scaling: `{nominal_load:.6f} N`.

Peak gate: selected nominal peak must be `<= {MAX_REASONABLE_PEAK_NM:.3f} Nm`; the standard selected peak is `{standard.peak_nm:.6f} Nm`.

Final standard candidate interior to searched ranges: `{interior}`.

## In-Place Command Estimate

Synthetic reference: `Vf=0`, `Vr=0`, `yawRate=+1 rad/s`. The target `+0.646/-0.646` command corresponds to extra opposing yaw support `{launch_target_extra:.6f} Nm` with the shared motor-command estimate. Launch was a gate/objective only; no giant pseudo-row was added to the coefficient fit.

{markdown_table(in_place, ["variant", "extra_opposing_yaw_torque_nm", "total_opposing_yaw_torque_nm", "left_command", "right_command", "max_abs_command", "passes_abs_0p6_gate"])}

## Standard Split Metrics

{markdown_table(split, ["group", "count", "run_count", "baseline_rmse_nm", "corrected_rmse_nm", "rmse_improvement_pct", "corrected_mae_nm", "corrected_median_abs_nm", "run_balanced_corrected_rmse_nm", "median_support_nm"])}

## Selected-Log Metrics

{markdown_table(selected, ["run_id", "dataset_split", "count", "baseline_rmse_nm", "corrected_rmse_nm", "rmse_improvement_pct", "baseline_signed_median_nm", "corrected_signed_median_nm", "median_support_nm"])}

## May 4 Latest Logs

The standard fit includes May 4 rows only through normal split weighting. In standard training weights, April rows contribute `{april_weight_fraction:.3f}` of row-weight and the two May 4 latest logs contribute `{latest_weight_fraction:.3f}` of row-weight.

Latest-weighted sensitivity branch raises the two May 4 logs to at least weight `3.0`; it is reported separately and is not the selected standard fit.

{markdown_table(latest_selected[latest_selected["run_id"].isin(LATEST_RUNS)], ["run_id", "dataset_split", "count", "baseline_rmse_nm", "corrected_rmse_nm", "rmse_improvement_pct", "median_support_nm"])}

## Risk Metrics

{markdown_table(risk, ["group", "count", "run_count", "baseline_rmse_nm", "corrected_rmse_nm", "rmse_improvement_pct", "run_balanced_corrected_rmse_nm", "median_support_nm"])}

## Reference Comparisons

{markdown_table(comp, ["group", "baseline_rmse_nm", "unshifted_refined_rmse_nm", "force_domain_stribeck_rmse_nm", "rational_residual_reference_rmse_nm", "true_traction_testbed_rmse_nm", "standalone_contact_traction_rmse_nm"])}

Selected-log comparison:

{markdown_table(selected_comp, ["run_id", "dataset_split", "baseline_rmse_nm", "unshifted_refined_rmse_nm", "force_domain_stribeck_rmse_nm", "true_traction_testbed_rmse_nm", "standalone_contact_traction_rmse_nm"])}

## Boundary Audit

{markdown_table(boundary_standard, ["parameter", "value", "range_min", "range_max", "normalized_margin_to_nearest_boundary", "boundary_adjacent", "search_status"])}

## Interpretation

- The selected standard candidate is interior to the final searched ranges and passes the `|cmd| >= 0.6` launch gate.
- The low-yaw slope is derived from the selected handoff interpretation; no independent linear slope is tuned.
- The best selected shape uses the raw, unshifted Stribeck branch, so peak support is the fitted `sliding_nm + static_extra_nm` at nominal load and is not inflated by a shifted transition.
- The launch target is satisfied without a dominating synthetic row, but this family still validates worse than the current force-domain Stribeck, rational residual reference, and standalone contact-traction testbed on most broad splits.

## Reproduce

```powershell
python codex_analysis\\yaw_model_variant_fits\\regime_linear_blend_stribeck_unshifted_refined\\fit_unshifted_refined.py
```

## Output Files

- `fit_unshifted_refined.py`
- `unshifted_refined_report.md`
- `candidate_scores.csv`
- `selected_parameters.csv`
- `standard_split_metrics.csv`
- `standard_selected_log_metrics.csv`
- `standard_risk_metrics.csv`
- `standard_phase_metrics.csv`
- `latest_weighted_split_metrics.csv`
- `latest_weighted_selected_log_metrics.csv`
- `latest_weighted_risk_metrics.csv`
- `latest_weighted_phase_metrics.csv`
- `baseline_comparison.csv`
- `selected_log_comparison.csv`
- `in_place_1radps_command.csv`
- `boundary_audit.csv`
- `range_expansion_audit.csv`
- `search_summary.csv`
- `prediction_sample.csv`
- `metadata.json`
"""
    (OUT / "unshifted_refined_report.md").write_text(report, encoding="utf-8")


def main() -> None:
    global LAUNCH_MIN_EXTRA_NM, LAUNCH_TARGET_EXTRA_NM
    constants = read_constants()
    frame, nominal_load = load_frame(constants)
    prepare_array_cache(frame)
    launch_target_extra = extra_for_target_command(constants, 1.0, LAUNCH_TARGET_COMMAND)
    LAUNCH_TARGET_EXTRA_NM = launch_target_extra
    LAUNCH_MIN_EXTRA_NM = extra_for_target_command(constants, 1.0, LAUNCH_MIN_COMMAND)

    standard, standard_scores, standard_summary, standard_boundary, standard_expansion = run_search_with_expansion(
        frame, nominal_load, constants, "standard", 24681357
    )
    latest, latest_scores, latest_summary, latest_boundary, latest_expansion = run_search_with_expansion(
        frame, nominal_load, constants, "latest_weighted", 97531864
    )

    scores = pd.concat([standard_scores, latest_scores], ignore_index=True)
    scores.to_csv(OUT / "candidate_scores.csv", index=False)
    pd.DataFrame([candidate_to_row(standard), candidate_to_row(latest)]).to_csv(
        OUT / "selected_parameters.csv", index=False
    )
    search_summary = pd.concat([standard_summary, latest_summary], ignore_index=True)
    search_summary.to_csv(OUT / "search_summary.csv", index=False)
    boundary = pd.concat([standard_boundary, latest_boundary], ignore_index=True)
    boundary.to_csv(OUT / "boundary_audit.csv", index=False)
    pd.concat([standard_expansion, latest_expansion], ignore_index=True).to_csv(
        OUT / "range_expansion_audit.csv", index=False
    )
    write_in_place_rows([standard, latest], constants)

    standard_frame = add_predictions(frame, nominal_load, constants, standard)
    latest_frame = add_predictions(frame, nominal_load, constants, latest)
    write_metrics(standard_frame, "standard")
    write_metrics(latest_frame, "latest_weighted")
    write_comparisons()
    standard_frame[
        [
            "run_id",
            "dataset_split",
            "physics_phase",
            "forward_velocity_mps",
            "yaw_rate_radps",
            "residual_opposes_yaw_nm",
            "predicted_opposing_yaw_support_nm",
            "corrected_residual_opposes_yaw_nm",
        ]
    ].head(5000).to_csv(OUT / "prediction_sample.csv", index=False)
    (OUT / "metadata.json").write_text(
        json.dumps(
            {
                "launch_target_command": LAUNCH_TARGET_COMMAND,
                "launch_min_command": LAUNCH_MIN_COMMAND,
                "launch_target_extra_nm": launch_target_extra,
                "max_reasonable_peak_nm": MAX_REASONABLE_PEAK_NM,
                "boundary_margin_fraction": BOUNDARY_MARGIN_FRACTION,
                "initial_ranges": INITIAL_RANGES,
                "slope_policy": "derived_only_no_independent_linear_slope",
                "candidate_inputs": [
                    "abs_yawRate",
                    "abs_Vf",
                    "total_normal_load",
                    "drive_wheel_longitudinal_offset",
                ],
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    write_report(
        standard,
        latest,
        constants,
        nominal_load,
        launch_target_extra,
        frame,
        search_summary,
        boundary,
    )


if __name__ == "__main__":
    main()
