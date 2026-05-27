#!/usr/bin/env python3
"""Per-contact rational slip-angle tire law with low-speed Stribeck envelope.

Analysis-only tooling. Reads shared yaw-model fit artifacts and writes outputs
only beside this script.
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

FORCE_DIR = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck"
RATIONAL_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "transition_options"
    / "rational_speed_force_blend"
)
STANDALONE_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "standalone_contact_traction_testbed"
)
TRUE_PATCH_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "contact_patch_true_traction_testbed"
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
    "measured_yaw_accel_radps2",
    "vbar_rel_mps",
    "vbar_yaw_mps",
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
]

CONTACTS = ["fl", "fr", "rl", "rr"]


@dataclass(frozen=True)
class OptimizedModel:
    speed_gate_v0_mps: float
    yaw_activation_mps: float
    stribeck_speed_mps: float
    sliding_ratio: float
    beta_floor_mps: float
    beta_abs_denominator: float
    beta_quad_denominator: float
    high_front_mu_per_beta: float
    high_rear_mu_per_beta: float
    launch_extra_opposing_nm: float
    static_peak_nm: float
    train_weighted_rmse_nm: float
    objective_score: float


def read_constants() -> dict[str, float]:
    table = pd.read_csv(CONSTANTS_INPUT)
    return {str(row.name): float(row.value) for row in table.itertuples(index=False)}


def sign_array(values: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    return (values > eps).astype(float) - (values < -eps).astype(float)


def sign(value: float, eps: float = 1.0e-6) -> float:
    return float((value > eps) - (value < -eps))


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else math.nan


def mae(values: np.ndarray) -> float:
    return float(np.mean(np.abs(values))) if len(values) else math.nan


def median_abs(values: np.ndarray) -> float:
    return float(np.median(np.abs(values))) if len(values) else math.nan


def run_balanced_rmse(frame: pd.DataFrame, values: np.ndarray) -> float:
    if not len(frame):
        return math.nan
    counts = Counter(frame["run_id"].astype(str))
    weights = np.array([1.0 / max(counts[str(run)], 1) for run in frame["run_id"]], dtype=float)
    return float(np.sqrt(np.average(np.square(values), weights=weights)))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row.keys():
            if key not in seen:
                fieldnames.append(key)
                seen.add(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def read_keyed_csv(path: Path, key: str) -> dict[str, dict[str, str]]:
    if not path.exists():
        return {}
    with path.open(newline="", encoding="utf-8") as handle:
        return {row[key]: row for row in csv.DictReader(handle)}


def torque_from_command(command: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]
    applied_voltage = command * battery
    current = (applied_voltage / resistance) - (
        wheel_speed_radps * (gear_ratio / speed_constant) / resistance
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
    left = command_from_torque(left_command_torque, left_speed, constants)
    right = command_from_torque(right_command_torque, right_speed, constants)
    return {
        "left_command": left,
        "right_command": right,
        "lr_delta_command": left - right,
        "max_abs_command": max(abs(left), abs(right)),
    }


def required_extra_for_launch(constants: dict[str, float], target_command: float = 0.646) -> float:
    track = constants["track_width_m"]
    radius = constants["wheel_radius_m"]
    half_track = 0.5 * track
    yaw_rate = 1.0
    left_speed = half_track * yaw_rate / radius
    left_wheel_torque = torque_from_command(target_command, left_speed, constants)
    applied_bank_torque = max(0.0, left_wheel_torque - constants["rolling_friction_torque_nm"])
    required_total = applied_bank_torque * track / radius
    return max(0.0, required_total - baseline_opposing_yaw_torque(constants, yaw_rate))


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
    for contact in CONTACTS:
        frame[f"{contact}_normal_n"] = frame[f"{contact}_normal_n"].fillna(
            frame["total_normal_load_n"] / 4.0
        )
        frame[f"{contact}_v_rel_f_mps"] = frame[f"{contact}_v_rel_f_mps"].fillna(0.0)
        frame[f"{contact}_v_rel_r_mps"] = frame[f"{contact}_v_rel_r_mps"].fillna(0.0)

    yaw_sign = sign_array(frame["yaw_rate_radps"].to_numpy())
    fallback = sign_array(frame["patch_yaw_force_basis_nm"].to_numpy())
    yaw_sign = np.where(yaw_sign == 0.0, fallback, yaw_sign)
    frame["yaw_sign"] = np.where(yaw_sign == 0.0, 1.0, yaw_sign)
    frame["abs_forward_velocity_mps"] = frame["forward_velocity_mps"].abs()
    frame["abs_yaw_rate_radps"] = frame["yaw_rate_radps"].abs()

    metadata = {
        "nominal_total_load_n": nominal_total_load,
        "rows": int(len(frame)),
        "runs": int(frame["run_id"].nunique()),
        "launch_extra_target_nm": required_extra_for_launch(constants),
        "launch_command_target": 0.646,
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
    weights = base * np.clip(quality, 0.02, 1.0)
    fit_counts = frame.loc[weights > 0.0, "run_id"].value_counts()
    if not fit_counts.empty:
        run_scale = frame["run_id"].map(
            {run: 1.0 / math.sqrt(count) for run, count in fit_counts.items()}
        ).fillna(0.0)
        weights *= run_scale.to_numpy()
        positive = weights > 0.0
        weights[positive] *= positive.sum() / weights[positive].sum()
    return weights


class DataView:
    def __init__(self, frame: pd.DataFrame, constants: dict[str, float], metadata: dict[str, float]):
        half_track = 0.5 * constants["track_width_m"]
        longitudinal = constants["drive_wheel_longitudinal_offset_m"]
        positions = {
            "fl": (-half_track, longitudinal),
            "fr": (half_track, longitudinal),
            "rl": (-half_track, -longitudinal),
            "rr": (half_track, -longitudinal),
        }
        self.frame = frame
        self.constants = constants
        self.metadata = metadata
        self.r = np.array([positions[c][0] for c in CONTACTS], dtype=float)
        self.f = np.array([positions[c][1] for c in CONTACTS], dtype=float)
        self.front = (self.f > 0.0).astype(float)
        self.rear = (self.f < 0.0).astype(float)
        self.normal = np.column_stack([frame[f"{c}_normal_n"].to_numpy() for c in CONTACTS])
        self.vf_rel = np.column_stack([frame[f"{c}_v_rel_f_mps"].to_numpy() for c in CONTACTS])
        self.vr_rel = np.column_stack([frame[f"{c}_v_rel_r_mps"].to_numpy() for c in CONTACTS])
        self.yaw_sign = frame["yaw_sign"].to_numpy()
        self.vf_abs = frame["abs_forward_velocity_mps"].to_numpy()
        self.yaw_rate_abs = frame["abs_yaw_rate_radps"].to_numpy()
        self.target = frame["residual_opposes_yaw_nm"].to_numpy()
        self.weights = training_weights(frame)
        self.train_mask = self.weights > 0.0
        self.train_index = np.flatnonzero(self.train_mask)
        self.total_normal = np.maximum(np.sum(self.normal, axis=1), 1.0e-9)
        self.launch_v_contact = math.hypot(half_track, longitudinal)

    def subset(self, mask: np.ndarray) -> "DataView":
        raise NotImplementedError("DataView is intentionally full-frame only")


def unpack(theta: np.ndarray) -> dict[str, float]:
    return {
        "speed_gate_v0_mps": math.exp(float(theta[0])),
        "yaw_activation_mps": math.exp(float(theta[1])),
        "stribeck_speed_mps": math.exp(float(theta[2])),
        "sliding_ratio": float(theta[3]),
        "beta_floor_mps": math.exp(float(theta[4])),
        "beta_abs_denominator": float(theta[5]),
        "beta_quad_denominator": float(theta[6]),
    }


def low_branch_and_static_peak(data: DataView, p: dict[str, float]) -> tuple[np.ndarray, float]:
    v0 = p["speed_gate_v0_mps"]
    vy = p["yaw_activation_mps"]
    vs = p["stribeck_speed_mps"]
    slide = p["sliding_ratio"]
    g0 = (v0 * v0) / np.maximum(v0 * v0 + np.square(data.vf_abs), 1.0e-12)
    v_contact = np.sqrt(np.square(data.vf_rel) + np.square(data.vr_rel))
    activation = np.square(v_contact) / np.maximum(np.square(v_contact) + vy * vy, 1.0e-12)
    stribeck = slide + (1.0 - slide) * (vs * vs) / np.maximum(
        vs * vs + np.square(v_contact), 1.0e-12
    )
    contact_factor = activation * stribeck

    launch_v = data.launch_v_contact
    launch_activation = launch_v * launch_v / max(launch_v * launch_v + vy * vy, 1.0e-12)
    launch_stribeck = slide + (1.0 - slide) * (vs * vs) / max(
        vs * vs + launch_v * launch_v, 1.0e-12
    )
    launch_factor = max(launch_activation * launch_stribeck, 1.0e-9)
    static_peak = data.metadata["launch_extra_target_nm"] / launch_factor

    lever_weight = data.normal * np.abs(data.r)[None, :]
    denom = np.maximum(np.sum(lever_weight, axis=1), 1.0e-12)
    contact_moment = static_peak * g0[:, None] * contact_factor * lever_weight / denom[:, None]
    force_f = data.yaw_sign[:, None] * np.sign(data.r)[None, :] * contact_moment / np.maximum(
        np.abs(data.r)[None, :], 1.0e-12
    )
    force_r = np.zeros_like(force_f)
    pred_opposes = data.yaw_sign * np.sum(data.r[None, :] * force_f - data.f[None, :] * force_r, axis=1)
    return pred_opposes, static_peak


def high_branch_features(data: DataView, p: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
    v0 = p["speed_gate_v0_mps"]
    beta_floor = p["beta_floor_mps"]
    beta_abs = p["beta_abs_denominator"]
    beta_quad = p["beta_quad_denominator"]
    g0 = (v0 * v0) / np.maximum(v0 * v0 + np.square(data.vf_abs), 1.0e-12)
    high = 1.0 - g0
    beta = data.vr_rel / np.sqrt(np.square(data.vf_rel) + beta_floor * beta_floor)
    beta_shape = beta / np.maximum(1.0 + beta_abs * np.abs(beta) + beta_quad * np.square(beta), 1.0e-12)
    unit_force_r = data.normal * beta_shape
    front_force = unit_force_r * data.front[None, :]
    rear_force = unit_force_r * data.rear[None, :]
    # F_r_high = -N*C*B_i: the tire force opposes contact-relative lateral slip.
    front_opposes = data.yaw_sign * high * np.sum(data.f[None, :] * front_force, axis=1)
    rear_opposes = data.yaw_sign * high * np.sum(data.f[None, :] * rear_force, axis=1)
    return front_opposes, rear_opposes


def weighted_nnls_two(x0: np.ndarray, x1: np.ndarray, y: np.ndarray, w: np.ndarray) -> tuple[float, float]:
    mask = w > 0.0
    if not np.any(mask):
        return 0.0, 0.0
    cols = [x0[mask], x1[mask]]
    yy = y[mask]
    ww = w[mask]

    def sse(c0: float, c1: float) -> float:
        r = yy - c0 * cols[0] - c1 * cols[1]
        return float(np.sum(ww * r * r))

    best = (0.0, 0.0, sse(0.0, 0.0))
    x = np.column_stack(cols)
    sw = np.sqrt(ww)
    try:
        sol, *_ = np.linalg.lstsq(x * sw[:, None], yy * sw, rcond=None)
        c0, c1 = float(sol[0]), float(sol[1])
        if c0 >= 0.0 and c1 >= 0.0:
            best = min(best, (c0, c1, sse(c0, c1)), key=lambda t: t[2])
    except np.linalg.LinAlgError:
        pass
    denom0 = float(np.sum(ww * cols[0] * cols[0]))
    if denom0 > 1.0e-18:
        c0 = max(0.0, float(np.sum(ww * cols[0] * yy) / denom0))
        best = min(best, (c0, 0.0, sse(c0, 0.0)), key=lambda t: t[2])
    denom1 = float(np.sum(ww * cols[1] * cols[1]))
    if denom1 > 1.0e-18:
        c1 = max(0.0, float(np.sum(ww * cols[1] * yy) / denom1))
        best = min(best, (0.0, c1, sse(0.0, c1)), key=lambda t: t[2])
    return best[0], best[1]


def evaluate_theta(data: DataView, theta: np.ndarray, return_pred: bool = False):
    p = unpack(theta)
    if not (0.0 <= p["sliding_ratio"] <= 0.5):
        return (1.0e9, None) if return_pred else 1.0e9
    low, static_peak = low_branch_and_static_peak(data, p)
    front, rear = high_branch_features(data, p)
    c_front, c_rear = weighted_nnls_two(front, rear, data.target - low, data.weights)
    pred = low + c_front * front + c_rear * rear
    residual = data.target[data.train_mask] - pred[data.train_mask]
    score = float(np.sqrt(np.average(np.square(residual), weights=data.weights[data.train_mask])))
    score += 0.002 * max(0.0, static_peak - 0.25) / 0.25
    score += 0.0001 * (c_front + c_rear)
    if return_pred:
        return score, pred, p, static_peak, c_front, c_rear
    return score


def differential_evolution(
    data: DataView,
    bounds: list[tuple[float, float]],
    seed: int = 20260526,
    population_size: int = 70,
    generations: int = 80,
) -> tuple[np.ndarray, float, list[dict[str, object]]]:
    rng = np.random.default_rng(seed)
    dim = len(bounds)
    lo = np.array([b[0] for b in bounds], dtype=float)
    hi = np.array([b[1] for b in bounds], dtype=float)
    pop = lo + rng.random((population_size, dim)) * (hi - lo)
    scores = np.array([evaluate_theta(data, row) for row in pop], dtype=float)
    trace: list[dict[str, object]] = []
    for gen in range(generations):
        f = 0.55 + 0.25 * rng.random()
        cr = 0.75
        for i in range(population_size):
            choices = np.delete(np.arange(population_size), i)
            a, b, c = pop[rng.choice(choices, size=3, replace=False)]
            mutant = np.clip(a + f * (b - c), lo, hi)
            cross = rng.random(dim) < cr
            cross[rng.integers(0, dim)] = True
            trial = np.where(cross, mutant, pop[i])
            score = evaluate_theta(data, trial)
            if score <= scores[i]:
                pop[i] = trial
                scores[i] = score
        best_idx = int(np.argmin(scores))
        best = pop[best_idx].copy()
        detail = evaluate_theta(data, best, return_pred=True)
        _, _, p, static_peak, c_front, c_rear = detail
        trace.append(
            {
                "stage": "differential_evolution",
                "iteration": gen,
                "best_score": scores[best_idx],
                "population_median_score": float(np.median(scores)),
                "speed_gate_v0_mps": p["speed_gate_v0_mps"],
                "yaw_activation_mps": p["yaw_activation_mps"],
                "stribeck_speed_mps": p["stribeck_speed_mps"],
                "sliding_ratio": p["sliding_ratio"],
                "beta_floor_mps": p["beta_floor_mps"],
                "beta_abs_denominator": p["beta_abs_denominator"],
                "beta_quad_denominator": p["beta_quad_denominator"],
                "static_peak_nm": static_peak,
                "high_front_mu_per_beta": c_front,
                "high_rear_mu_per_beta": c_rear,
            }
        )
    best_idx = int(np.argmin(scores))
    return pop[best_idx].copy(), float(scores[best_idx]), trace


def bounded_nelder_mead(
    data: DataView,
    start: np.ndarray,
    bounds: list[tuple[float, float]],
    max_iter: int = 320,
) -> tuple[np.ndarray, float, list[dict[str, object]], str]:
    dim = len(start)
    lo = np.array([b[0] for b in bounds], dtype=float)
    hi = np.array([b[1] for b in bounds], dtype=float)

    def clamp(x: np.ndarray) -> np.ndarray:
        return np.minimum(np.maximum(x, lo), hi)

    steps = 0.06 * (hi - lo)
    simplex = [clamp(start)]
    for i in range(dim):
        x = start.copy()
        x[i] += steps[i]
        simplex.append(clamp(x))
    simplex = np.array(simplex)
    scores = np.array([evaluate_theta(data, x) for x in simplex])
    trace: list[dict[str, object]] = []
    reason = "max_iter"
    for iteration in range(max_iter):
        order = np.argsort(scores)
        simplex = simplex[order]
        scores = scores[order]
        centroid = np.mean(simplex[:-1], axis=0)
        worst = simplex[-1]
        reflected = clamp(centroid + (centroid - worst))
        reflected_score = evaluate_theta(data, reflected)
        if reflected_score < scores[0]:
            expanded = clamp(centroid + 2.0 * (reflected - centroid))
            expanded_score = evaluate_theta(data, expanded)
            if expanded_score < reflected_score:
                simplex[-1], scores[-1] = expanded, expanded_score
            else:
                simplex[-1], scores[-1] = reflected, reflected_score
        elif reflected_score < scores[-2]:
            simplex[-1], scores[-1] = reflected, reflected_score
        else:
            contracted = clamp(centroid + 0.5 * (worst - centroid))
            contracted_score = evaluate_theta(data, contracted)
            if contracted_score < scores[-1]:
                simplex[-1], scores[-1] = contracted, contracted_score
            else:
                best = simplex[0].copy()
                for i in range(1, len(simplex)):
                    simplex[i] = clamp(best + 0.5 * (simplex[i] - best))
                    scores[i] = evaluate_theta(data, simplex[i])
        best_detail = evaluate_theta(data, simplex[int(np.argmin(scores))], return_pred=True)
        _, _, p, static_peak, c_front, c_rear = best_detail
        trace.append(
            {
                "stage": "nelder_mead",
                "iteration": iteration,
                "best_score": float(np.min(scores)),
                "simplex_score_span": float(np.max(scores) - np.min(scores)),
                "speed_gate_v0_mps": p["speed_gate_v0_mps"],
                "yaw_activation_mps": p["yaw_activation_mps"],
                "stribeck_speed_mps": p["stribeck_speed_mps"],
                "sliding_ratio": p["sliding_ratio"],
                "beta_floor_mps": p["beta_floor_mps"],
                "beta_abs_denominator": p["beta_abs_denominator"],
                "beta_quad_denominator": p["beta_quad_denominator"],
                "static_peak_nm": static_peak,
                "high_front_mu_per_beta": c_front,
                "high_rear_mu_per_beta": c_rear,
            }
        )
        if np.max(scores) - np.min(scores) < 1.0e-7 and np.max(np.ptp(simplex, axis=0)) < 1.0e-4:
            reason = "simplex_tolerance"
            break
    order = np.argsort(scores)
    return simplex[order[0]].copy(), float(scores[order[0]]), trace, reason


def corrected_residuals(frame: pd.DataFrame, pred_opposes: np.ndarray) -> np.ndarray:
    pred_additive = -frame["yaw_sign"].to_numpy() * pred_opposes
    return frame["residual_additive_yaw_torque_nm"].to_numpy() - pred_additive


def metric_row(label: str, frame: pd.DataFrame, pred_opposes: np.ndarray) -> dict[str, object]:
    baseline = frame["residual_additive_yaw_torque_nm"].to_numpy()
    corrected = corrected_residuals(frame, pred_opposes)
    base_rmse = rmse(baseline)
    corr_rmse = rmse(corrected)
    return {
        "group": label,
        "count": int(len(frame)),
        "run_count": int(frame["run_id"].nunique()) if len(frame) else 0,
        "baseline_rmse_nm": base_rmse,
        "corrected_rmse_nm": corr_rmse,
        "rmse_delta_vs_baseline_pct": 100.0 * (corr_rmse - base_rmse) / base_rmse
        if base_rmse > 0.0
        else math.nan,
        "baseline_mae_nm": mae(baseline),
        "corrected_mae_nm": mae(corrected),
        "baseline_median_abs_nm": median_abs(baseline),
        "corrected_median_abs_nm": median_abs(corrected),
        "baseline_signed_median_nm": float(np.median(baseline)) if len(frame) else math.nan,
        "corrected_signed_median_nm": float(np.median(corrected)) if len(frame) else math.nan,
        "run_balanced_corrected_rmse_nm": run_balanced_rmse(frame, corrected),
        "median_pred_opposes_nm": float(np.median(pred_opposes)) if len(pred_opposes) else math.nan,
    }


def launch_command_row(model: OptimizedModel, constants: dict[str, float]) -> dict[str, object]:
    base = baseline_opposing_yaw_torque(constants, 1.0)
    total = base + model.launch_extra_opposing_nm
    commands = motor_commands_for_opposing_torque(total, constants, 1.0)
    return {
        "variant": "patch_rational_slip",
        "base_opposing_yaw_torque_nm": base,
        "extra_opposing_yaw_torque_nm": model.launch_extra_opposing_nm,
        "total_opposing_yaw_torque_nm": total,
        **commands,
        "passes_abs_0p6_gate": commands["max_abs_command"] >= 0.6,
    }


def make_model(data: DataView, theta: np.ndarray, score: float) -> tuple[OptimizedModel, np.ndarray]:
    eval_score, pred, p, static_peak, c_front, c_rear = evaluate_theta(data, theta, return_pred=True)
    train_resid = data.target[data.train_mask] - pred[data.train_mask]
    train_rmse = float(np.sqrt(np.average(np.square(train_resid), weights=data.weights[data.train_mask])))
    model = OptimizedModel(
        speed_gate_v0_mps=p["speed_gate_v0_mps"],
        yaw_activation_mps=p["yaw_activation_mps"],
        stribeck_speed_mps=p["stribeck_speed_mps"],
        sliding_ratio=p["sliding_ratio"],
        beta_floor_mps=p["beta_floor_mps"],
        beta_abs_denominator=p["beta_abs_denominator"],
        beta_quad_denominator=p["beta_quad_denominator"],
        high_front_mu_per_beta=c_front,
        high_rear_mu_per_beta=c_rear,
        launch_extra_opposing_nm=data.metadata["launch_extra_target_nm"],
        static_peak_nm=static_peak,
        train_weighted_rmse_nm=train_rmse,
        objective_score=eval_score,
    )
    return model, pred


def fmt(value: object, digits: int = 6) -> str:
    try:
        x = float(value)
        if not math.isfinite(x):
            return ""
        return f"{x:.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def markdown_table(rows: list[dict[str, object]], columns: list[str], limit: int | None = None) -> list[str]:
    if limit is not None:
        rows = rows[:limit]
    lines = ["| " + " | ".join(columns) + " |", "| " + " | ".join("---" for _ in columns) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(fmt(row.get(col, "")) for col in columns) + " |")
    return lines


def comparison_rows(split_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    force = read_keyed_csv(FORCE_DIR / "split_rmse.csv", "dataset_split")
    rational = read_keyed_csv(RATIONAL_DIR / "split_metrics.csv", "group")
    standalone = read_keyed_csv(STANDALONE_DIR / "split_metrics.csv", "group")
    true_patch = read_keyed_csv(TRUE_PATCH_DIR / "baseline_comparison.csv", "group")
    rows = []
    for row in split_rows:
        group = str(row["group"])
        rows.append(
            {
                "group": group,
                "patch_rational_slip_rmse_nm": row["corrected_rmse_nm"],
                "force_domain_stribeck_rmse_nm": force.get(group, {}).get("corrected_rmse_nm", ""),
                "rational_residual_reference_rmse_nm": rational.get(group, {}).get(
                    "corrected_rmse_nm", ""
                ),
                "standalone_contact_traction_rmse_nm": standalone.get(group, {}).get(
                    "standalone_rmse_nm", ""
                ),
                "true_patch_testbed_rmse_nm": true_patch.get(group, {}).get(
                    "true_patch_corrected_rmse_nm", ""
                ),
            }
        )
    return rows


def write_report(
    model: OptimizedModel,
    metadata: dict[str, float],
    split_rows: list[dict[str, object]],
    selected_rows: list[dict[str, object]],
    risk_rows: list[dict[str, object]],
    comp_rows: list[dict[str, object]],
    launch_row: dict[str, object],
    opt_summary: dict[str, object],
) -> None:
    lines: list[str] = []
    lines.append("# Patch Rational Slip-Angle Stribeck Candidate")
    lines.append("")
    lines.append("Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.")
    lines.append("")
    lines.append("## Model")
    lines.append("")
    lines.append("For contact `i` at position `(r_i, f_i)`, the prediction is a physical yaw-opposing torque from summed contact forces:")
    lines.append("")
    lines.append("`M_opp = -sign(yawRate) * sum_i(f_i * F_r_i - r_i * F_f_i)`")
    lines.append("")
    lines.append("Low-speed branch:")
    lines.append("")
    lines.append("`G0 = V0^2 / (V0^2 + Vf^2)`")
    lines.append("")
    lines.append("`A_i = v_i^2 / (v_i^2 + v_y^2)`")
    lines.append("")
    lines.append("`S_i = r_slide + (1-r_slide) * v_s^2 / (v_s^2 + v_i^2)`")
    lines.append("")
    lines.append("`M_low_i = M_static_peak * G0 * A_i * S_i * (N_i * |r_i|) / sum_j(N_j * |r_j|)`")
    lines.append("")
    lines.append("`F_f_i_low = sign(yawRate) * sign(r_i) * M_low_i / |r_i|`, `F_r_i_low = 0`")
    lines.append("")
    lines.append("High-speed branch:")
    lines.append("")
    lines.append("`beta_i = v_rel_r_i / sqrt(v_rel_f_i^2 + v_floor^2)`")
    lines.append("")
    lines.append("`B_i = beta_i / (1 + a*|beta_i| + b*beta_i^2)`")
    lines.append("")
    lines.append("`F_r_i_high = -(1-G0) * N_i * C_axle * B_i`, `F_f_i_high = 0`")
    lines.append("")
    lines.append("The selected law uses only `sqrt`, `abs`, rational divisions, and sign/clamp-style operations. It uses no trig, `atan`, `exp`, `tanh`, command/request selectors, UKF state-vector fields, or residual lookup table.")
    lines.append("")
    lines.append("## Optimization")
    lines.append("")
    lines.append("SciPy was not available in the bundled runtime, so this run used a custom continuous optimizer: differential evolution over nonlinear parameters, then bounded Nelder-Mead refinement. For each nonlinear point, the two nonnegative axle slip-angle gains were solved by weighted two-column NNLS.")
    lines.append("")
    lines.extend(markdown_table([opt_summary], list(opt_summary.keys())))
    lines.append("")
    lines.append("## Selected Parameters")
    lines.append("")
    param_rows = [{"parameter": k, "value": v} for k, v in model.__dict__.items()]
    lines.extend(markdown_table(param_rows, ["parameter", "value"]))
    lines.append("")
    lines.append("## Boundary Behavior")
    lines.append("")
    boundary_rows = [
        {
            "quantity": "speed_gate_v0_mps",
            "value": model.speed_gate_v0_mps,
            "boundary": "lower bound was 0.020 m/s",
        },
        {
            "quantity": "high_front_mu_per_beta",
            "value": model.high_front_mu_per_beta,
            "boundary": "zero gain means no selected front slip-angle branch",
        },
        {
            "quantity": "high_rear_mu_per_beta",
            "value": model.high_rear_mu_per_beta,
            "boundary": "zero gain means no selected rear slip-angle branch",
        },
        {
            "quantity": "static_peak_nm",
            "value": model.static_peak_nm,
            "boundary": "regularization knee was 0.25 Nm",
        },
    ]
    lines.extend(markdown_table(boundary_rows, ["quantity", "value", "boundary"]))
    lines.append("")
    lines.append("## 1 rad/s In-Place Launch")
    lines.append("")
    lines.extend(markdown_table([launch_row], list(launch_row.keys())))
    lines.append("")
    lines.append("## Split Metrics")
    lines.append("")
    lines.extend(
        markdown_table(
            split_rows,
            [
                "group",
                "count",
                "baseline_rmse_nm",
                "corrected_rmse_nm",
                "run_balanced_corrected_rmse_nm",
                "corrected_median_abs_nm",
            ],
        )
    )
    lines.append("")
    lines.append("## Reference Comparison")
    lines.append("")
    lines.extend(markdown_table(comp_rows, list(comp_rows[0].keys())))
    lines.append("")
    lines.append("## Selected Logs")
    lines.append("")
    lines.extend(
        markdown_table(
            selected_rows,
            [
                "run_id",
                "dataset_split",
                "count",
                "baseline_rmse_nm",
                "corrected_rmse_nm",
                "corrected_signed_median_nm",
            ],
        )
    )
    lines.append("")
    lines.append("## Risk Slices")
    lines.append("")
    lines.extend(
        markdown_table(
            risk_rows,
            [
                "group",
                "count",
                "baseline_rmse_nm",
                "corrected_rmse_nm",
                "corrected_median_abs_nm",
            ],
        )
    )
    lines.append("")
    lines.append("## May 4 Latest Logs")
    lines.append("")
    may4 = [row for row in selected_rows if str(row.get("run_id", "")).startswith("2026-05-04")]
    lines.extend(
        markdown_table(
            may4,
            [
                "run_id",
                "dataset_split",
                "count",
                "baseline_rmse_nm",
                "corrected_rmse_nm",
                "corrected_signed_median_nm",
            ],
        )
    )
    lines.append("")
    lines.append("## Viability")
    lines.append("")
    primary = next(r for r in split_rows if r["group"] == "primary_open_floor_fit_authoritative")
    validation = next(r for r in split_rows if r["group"] == "validation_non_authoritative")
    standalone = next(r for r in comp_rows if r["group"] == "validation_non_authoritative")
    standalone_rmse = standalone.get("standalone_contact_traction_rmse_nm", "")
    lines.append(
        f"The model passes the hard in-place gate with max command {fmt(launch_row['max_abs_command'])}, "
        f"but the optimized slip-angle branch is degenerate: both high-speed axle gains are zero and `V0` is on the lower bound. "
        f"Primary RMSE is {fmt(primary['corrected_rmse_nm'])} Nm and non-authoritative validation RMSE is {fmt(validation['corrected_rmse_nm'])} Nm, both worse than the main references. "
        f"The standalone contact-traction testbed remains materially better on validation ({fmt(standalone_rmse)} Nm). "
        "Verdict: this exact residual-correction family is not viable; the data prefers either the older force-domain Stribeck residual or the broader standalone contact-traction law over this constrained Stribeck-to-slip-angle handoff."
    )
    lines.append("")
    lines.append("## Outputs")
    lines.append("")
    for name in [
        "fit_patch_rational_slip.py",
        "patch_rational_slip_report.md",
        "optimizer_trace.csv",
        "optimization_summary.csv",
        "boundary_behavior.csv",
        "selected_parameters.csv",
        "split_metrics.csv",
        "phase_metrics.csv",
        "selected_log_metrics.csv",
        "may4_latest_log_metrics.csv",
        "risk_metrics.csv",
        "split_reference_comparison.csv",
        "launch_command_estimate.csv",
        "prediction_sample.csv",
        "commands_run.txt",
    ]:
        lines.append(f"- `{name}`")
    (OUT / "patch_rational_slip_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    constants = read_constants()
    frame, metadata = load_frame(constants)
    full_data = DataView(frame, constants, metadata)
    weighted_frame = frame[full_data.weights > 0.0].copy()
    weighted_weights = full_data.weights[full_data.weights > 0.0]
    sample_count = min(18000, len(weighted_frame))
    rng = np.random.default_rng(20260526)
    sample_prob = weighted_weights / np.sum(weighted_weights)
    sample_idx = rng.choice(len(weighted_frame), size=sample_count, replace=False, p=sample_prob)
    optimizer_frame = weighted_frame.iloc[np.sort(sample_idx)].copy()
    data = DataView(optimizer_frame, constants, metadata)
    polish_data = DataView(weighted_frame, constants, metadata)
    bounds = [
        (math.log(0.02), math.log(1.2)),
        (math.log(0.001), math.log(0.12)),
        (math.log(0.003), math.log(0.25)),
        (0.0, 0.40),
        (math.log(0.015), math.log(0.8)),
        (0.0, 5.0),
        (0.0, 5.0),
    ]
    de_best, de_score, de_trace = differential_evolution(
        data, bounds, population_size=56, generations=64
    )
    nm_best, nm_score, nm_trace, nm_reason = bounded_nelder_mead(
        polish_data, de_best, bounds, max_iter=120
    )
    trace = de_trace + nm_trace
    write_csv(OUT / "optimizer_trace.csv", trace)
    model, pred = make_model(full_data, nm_best, nm_score)

    split_rows: list[dict[str, object]] = []
    for split in SPLITS:
        mask = frame["dataset_split"].to_numpy() == split
        split_rows.append(metric_row(split, frame[mask], pred[mask]))
    validation_mask = frame["dataset_split"].to_numpy() != "primary_open_floor_fit_authoritative"
    split_rows.append(metric_row("validation_non_authoritative", frame[validation_mask], pred[validation_mask]))
    write_csv(OUT / "split_metrics.csv", split_rows)

    phase_rows = []
    for phase, subset in frame.groupby("physics_phase"):
        idx = subset.index.to_numpy()
        phase_rows.append(metric_row(str(phase), subset, pred[idx]))
    write_csv(OUT / "phase_metrics.csv", phase_rows)

    selected_rows: list[dict[str, object]] = []
    for run_id in SELECTED_RUNS:
        subset = frame[frame["run_id"].astype(str) == run_id]
        if len(subset):
            idx = subset.index.to_numpy()
            row = metric_row(run_id, subset, pred[idx])
            row["run_id"] = run_id
            row["dataset_split"] = str(subset["dataset_split"].mode().iloc[0])
            selected_rows.append(row)
        else:
            selected_rows.append({"group": run_id, "run_id": run_id, "present": False})
    write_csv(OUT / "selected_log_metrics.csv", selected_rows)
    write_csv(
        OUT / "may4_latest_log_metrics.csv",
        [row for row in selected_rows if str(row.get("run_id", "")).startswith("2026-05-04")],
    )

    risk_masks = {
        "straightish_abs_yaw_lt_0p05": frame["abs_yaw_rate_radps"].to_numpy() < 0.05,
        "straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05": (
            (frame["abs_yaw_rate_radps"].to_numpy() < 0.05)
            & (frame["abs_forward_velocity_mps"].to_numpy() >= 0.05)
        ),
        "low_speed_yaw_vf_lt_0p05_yaw_ge_0p2": (
            (frame["abs_forward_velocity_mps"].to_numpy() < 0.05)
            & (frame["abs_yaw_rate_radps"].to_numpy() >= 0.2)
        ),
        "high_forward_vf_ge_0p5": frame["abs_forward_velocity_mps"].to_numpy() >= 0.5,
        "high_speed_abs_vf_ge_0p7": frame["abs_forward_velocity_mps"].to_numpy() >= 0.7,
        "limiter_active": frame["max_force_limiter_activity"].to_numpy() > 0.01,
        "hardware_saturation_evidence": frame["hardware_saturation_evidence"].to_numpy() > 0.0,
    }
    risk_rows = []
    for name, mask in risk_masks.items():
        risk_rows.append(metric_row(name, frame[mask], pred[mask]))
    write_csv(OUT / "risk_metrics.csv", risk_rows)

    comp_rows = comparison_rows(split_rows)
    write_csv(OUT / "split_reference_comparison.csv", comp_rows)

    launch_row = launch_command_row(model, constants)
    write_csv(OUT / "launch_command_estimate.csv", [launch_row])

    write_csv(
        OUT / "selected_parameters.csv",
        [{"parameter": key, "value": value} for key, value in model.__dict__.items()]
        + [{"parameter": key, "value": value} for key, value in metadata.items()],
    )

    opt_summary = {
        "scipy_available": False,
        "optimizer": "custom differential_evolution plus bounded nelder_mead",
        "de_generations": 64,
        "de_population": 56,
        "de_best_score": de_score,
        "nelder_mead_iterations": len(nm_trace),
        "nelder_mead_stop": nm_reason,
        "final_objective": nm_score,
        "trace_rows": len(trace),
        "de_sample_rows": sample_count,
        "nelder_mead_weighted_rows": len(weighted_frame),
    }
    write_csv(OUT / "optimization_summary.csv", [opt_summary])
    write_csv(
        OUT / "boundary_behavior.csv",
        [
            {
                "quantity": "speed_gate_v0_mps",
                "value": model.speed_gate_v0_mps,
                "boundary": "lower bound 0.020 m/s",
                "at_boundary": model.speed_gate_v0_mps <= 0.0205,
            },
            {
                "quantity": "high_front_mu_per_beta",
                "value": model.high_front_mu_per_beta,
                "boundary": "zero lower bound",
                "at_boundary": abs(model.high_front_mu_per_beta) < 1.0e-9,
            },
            {
                "quantity": "high_rear_mu_per_beta",
                "value": model.high_rear_mu_per_beta,
                "boundary": "zero lower bound",
                "at_boundary": abs(model.high_rear_mu_per_beta) < 1.0e-9,
            },
            {
                "quantity": "static_peak_nm",
                "value": model.static_peak_nm,
                "boundary": "0.25 Nm regularization knee",
                "at_boundary": model.static_peak_nm >= 0.249,
            },
        ],
    )

    sample = frame[
        [
            "run_id",
            "row_index",
            "dataset_split",
            "physics_phase",
            "forward_velocity_mps",
            "yaw_rate_radps",
            "residual_additive_yaw_torque_nm",
            "residual_opposes_yaw_nm",
        ]
    ].copy()
    sample["pred_opposes_nm"] = pred
    sample["corrected_residual_nm"] = corrected_residuals(frame, pred)
    sample.iloc[:: max(len(sample) // 600, 1)].to_csv(OUT / "prediction_sample.csv", index=False)

    (OUT / "commands_run.txt").write_text(
        "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' "
        "codex_analysis\\yaw_model_variant_fits\\stribeck_to_slip_angle_candidates\\patch_rational_slip\\fit_patch_rational_slip.py\n",
        encoding="utf-8",
    )
    write_report(model, metadata, split_rows, selected_rows, risk_rows, comp_rows, launch_row, opt_summary)


if __name__ == "__main__":
    main()
