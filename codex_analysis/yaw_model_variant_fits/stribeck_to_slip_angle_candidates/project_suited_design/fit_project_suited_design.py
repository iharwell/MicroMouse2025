#!/usr/bin/env python3
"""Project-suited Stribeck-to-slip-angle contact model fit.

Analysis-only tooling. Reads shared yaw-model fit artifacts and writes outputs
only beside this script.
"""

from __future__ import annotations

import csv
import math
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from codex_analysis.yaw_model_variant_fits.regime_weighting import (
    PRIMARY_ONLY_SPLIT_WEIGHTS,
    QualityConfig,
    RegimeWeightConfig,
    add_forward_accel_columns_to_frame,
    compute_regime_weights_for_frame,
    write_regime_diagnostics,
)
from codex_analysis.yaw_model_variant_fits.common_range_metrics import (
    COMMON_RANGE_REPORT_COLUMNS,
    write_common_range_metrics,
)

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

BRUSH_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "stribeck_to_slip_angle_candidates"
    / "brush_combined_slip"
)
BRISTLE_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "stribeck_to_slip_angle_candidates"
    / "relaxation_free_bristle_slip"
)
SCALAR_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "stribeck_to_slip_angle_candidates"
    / "scalar_slip_angle_partition"
)
STANDALONE_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "standalone_contact_traction_testbed"
)
FORCE_STRIBECK_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "round2_force_domain_stribeck"
)
RATIONAL_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "transition_options"
    / "rational_speed_force_blend"
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

CONTACTS = ["fl", "fr", "rl", "rr"]

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
]

SECONDARY_COLUMNS = [
    "run_id",
    "row_index",
    "observed_yaw_moment_nm",
    "model_yaw_moment_nm",
    "left_drive_force_n",
    "right_drive_force_n",
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


@dataclass(frozen=True)
class ModelParams:
    drive_scale: float
    longitudinal_mu: float
    longitudinal_k_mps: float
    mu_peak: float
    mu_slide: float
    stribeck_speed_mps: float
    low_speed_gate_mps: float
    lateral_sign_eps_mps: float
    alpha_floor_mps: float
    alpha_knee: float
    corner_mu_front: float
    corner_mu_rear: float
    derived_static_extra_mu: float
    launch_total_opposing_nm: float
    launch_static_solve_error_nm: float
    objective_score: float


BOUNDS = [
    ("drive_scale", 0.030, 0.180),
    ("longitudinal_mu", 0.000, 1.200),
    ("longitudinal_k_mps", 0.020, 0.700),
    ("mu_peak", 1.650, 5.000),
    ("mu_slide", 0.000, 0.500),
    ("stribeck_speed_mps", 0.020, 0.450),
    ("low_speed_gate_mps", 0.080, 0.600),
    ("lateral_sign_eps_mps", 0.001, 0.050),
    ("alpha_floor_mps", 0.030, 0.800),
    ("alpha_knee", 0.020, 0.900),
    ("corner_mu_front", 0.050, 3.500),
    ("corner_mu_rear", 0.050, 3.500),
]


def read_constants() -> dict[str, float]:
    table = pd.read_csv(CONSTANTS_INPUT)
    return {str(row.name): float(row.value) for row in table.itertuples(index=False)}


def sign(value: float, eps: float = 1.0e-9) -> float:
    return float((value > eps) - (value < -eps))


def smooth_sat(values: np.ndarray, knee: float) -> np.ndarray:
    return values / np.sqrt(values * values + knee * knee)


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
    weights = np.array([1.0 / max(counts[str(run)], 1) for run in frame["run_id"]])
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
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or key not in reader.fieldnames:
            return {}
        return {row[key]: row for row in reader}


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


def required_total_launch_opposing_nm(constants: dict[str, float], target_command: float = 0.646) -> float:
    track = constants["track_width_m"]
    radius = constants["wheel_radius_m"]
    half_track = 0.5 * track
    yaw_rate = 1.0
    left_speed = half_track * yaw_rate / radius
    left_wheel_torque = torque_from_command(target_command, left_speed, constants)
    applied_bank_torque = max(0.0, left_wheel_torque - constants["rolling_friction_torque_nm"])
    return applied_bank_torque * track / radius


def motor_commands_for_opposing_torque(
    opposing_yaw_torque: float, constants: dict[str, float], yaw_rate: float = 1.0
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


def load_frame(constants: dict[str, float]) -> pd.DataFrame:
    primary = pd.read_csv(PRIMARY_INPUT, usecols=PRIMARY_COLUMNS)
    secondary = pd.read_csv(SECONDARY_INPUT, usecols=SECONDARY_COLUMNS)
    frame = primary.merge(secondary, how="left", on=["run_id", "row_index"])
    numeric_columns = set(PRIMARY_COLUMNS + SECONDARY_COLUMNS) - {
        "run_id",
        "family",
        "schema",
        "recommendation",
        "dataset_split",
        "physics_phase",
    }
    for column in numeric_columns:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(
        subset=[
            "forward_velocity_mps",
            "yaw_rate_radps",
            "observed_yaw_moment_nm",
            "model_yaw_moment_nm",
            "residual_additive_yaw_torque_nm",
        ]
    )
    frame = frame[frame["dataset_split"] != "excluded_or_unclassified"].copy()
    primary_mask = frame["dataset_split"] == "primary_open_floor_fit_authoritative"
    nominal_load = float(frame.loc[primary_mask, "total_normal_load_n"].median())
    if not math.isfinite(nominal_load) or nominal_load <= 0.0:
        nominal_load = constants["mass_kg"] * 9.80665 + 0.8 * constants.get(
            "fan_downforce_full_duty_n", 0.0
        )
    frame["total_normal_load_n"] = frame["total_normal_load_n"].fillna(nominal_load)
    for contact in CONTACTS:
        frame[f"{contact}_normal_n"] = frame[f"{contact}_normal_n"].fillna(
            frame["total_normal_load_n"] / 4.0
        )
        frame[f"{contact}_v_rel_f_mps"] = frame[f"{contact}_v_rel_f_mps"].fillna(0.0)
        frame[f"{contact}_v_rel_r_mps"] = frame[f"{contact}_v_rel_r_mps"].fillna(0.0)
    frame["abs_forward_velocity_mps"] = frame["forward_velocity_mps"].abs()
    frame["abs_yaw_rate_radps"] = frame["yaw_rate_radps"].abs()
    frame["baseline_error_nm"] = frame["model_yaw_moment_nm"] - frame["observed_yaw_moment_nm"]
    return add_forward_accel_columns_to_frame(frame.reset_index(drop=True))


class DataView:
    def __init__(self, frame: pd.DataFrame, constants: dict[str, float]):
        half_track = 0.5 * constants["track_width_m"]
        longitudinal = constants["drive_wheel_longitudinal_offset_m"]
        positions = {
            "fl": (-half_track, longitudinal, "left"),
            "fr": (half_track, longitudinal, "right"),
            "rl": (-half_track, -longitudinal, "left"),
            "rr": (half_track, -longitudinal, "right"),
        }
        self.frame = frame
        self.constants = constants
        self.r = np.array([positions[c][0] for c in CONTACTS], dtype=float)[None, :]
        self.f = np.array([positions[c][1] for c in CONTACTS], dtype=float)[None, :]
        self.front = (self.f > 0.0).astype(float)
        self.rear = (self.f < 0.0).astype(float)
        self.side = [positions[c][2] for c in CONTACTS]
        self.normal = np.column_stack([frame[f"{c}_normal_n"].to_numpy(float) for c in CONTACTS])
        self.vf_rel = np.column_stack([frame[f"{c}_v_rel_f_mps"].to_numpy(float) for c in CONTACTS])
        self.vr_rel = np.column_stack([frame[f"{c}_v_rel_r_mps"].to_numpy(float) for c in CONTACTS])
        self.vf_body = frame["forward_velocity_mps"].to_numpy(float)[:, None]
        self.yaw_rate = frame["yaw_rate_radps"].to_numpy(float)[:, None]
        self.vf_abs = np.abs(self.vf_body)
        self.target = frame["observed_yaw_moment_nm"].to_numpy(float)
        self.baseline_error = frame["baseline_error_nm"].to_numpy(float)
        self.left_drive = frame["left_drive_force_n"].to_numpy(float)
        self.right_drive = frame["right_drive_force_n"].to_numpy(float)
        self.drive = self._normal_load_split_drive_force()
        self.train_primary_weights = self._objective_weights("primary")
        self.train_downweighted_weights = self._objective_weights("downweighted")
        self.latest_weights = self._objective_weights("latest")

    def _normal_load_split_drive_force(self) -> np.ndarray:
        drive = np.zeros_like(self.normal)
        for side_name, side_drive in [("left", self.left_drive), ("right", self.right_drive)]:
            cols = [i for i, side in enumerate(self.side) if side == side_name]
            side_normal = np.maximum(np.sum(self.normal[:, cols], axis=1), 1.0e-9)
            for col in cols:
                drive[:, col] = side_drive * (self.normal[:, col] / side_normal)
        return drive

    def _objective_weights(self, tier: str) -> np.ndarray:
        split = self.frame["dataset_split"].astype(str)
        family = self.frame["family"].astype(str)
        recommendation = self.frame["recommendation"].astype(str)
        runs = self.frame["run_id"].astype(str)
        quality = QualityConfig(
            gyro_spike_multiplier=0.25,
            saturation_multiplier=0.25,
            use_limiter_penalty=True,
            limiter_gain=4.0,
            use_low_yaw_no_motion_penalty=False,
            quality_floor=0.02,
        )
        if tier == "primary":
            result = compute_regime_weights_for_frame(
                self.frame,
                RegimeWeightConfig(split_weights=PRIMARY_ONLY_SPLIT_WEIGHTS, quality=quality),
            )
        elif tier == "downweighted":
            eligible = (
                (split == "open_floor_fit_downweighted")
                & (family == "open_floor")
                & (recommendation == "fit_downweighted")
            )
            result = compute_regime_weights_for_frame(
                self.frame,
                RegimeWeightConfig(
                    split_weights={
                        "primary_open_floor_fit_authoritative": 0.0,
                        "open_floor_fit_downweighted": 1.0,
                        "open_floor_validation_only": 0.0,
                        "diag_validation_only": 0.0,
                        "aux_downweighted_validation": 0.0,
                    },
                    quality=quality,
                ),
                eligible_mask=eligible,
            )
        elif tier == "latest":
            eligible = runs.isin(["2026-05-04_20-35-47", "2026-05-04_16-57-53"])
            result = compute_regime_weights_for_frame(
                self.frame,
                RegimeWeightConfig(
                    split_weights={
                        "primary_open_floor_fit_authoritative": 1.0,
                        "open_floor_fit_downweighted": 1.0,
                        "open_floor_validation_only": 1.0,
                        "diag_validation_only": 1.0,
                        "aux_downweighted_validation": 1.0,
                    },
                    quality=quality,
                ),
                eligible_mask=eligible,
            )
        else:
            raise ValueError(tier)
        return np.asarray(result.weights, dtype=float)


def unpack(theta: np.ndarray) -> dict[str, float]:
    return {
        name: lo + float(np.clip(theta[i], 0.0, 1.0)) * (hi - lo)
        for i, (name, lo, hi) in enumerate(BOUNDS)
    }


def pack(values: dict[str, float]) -> np.ndarray:
    out: list[float] = []
    for name, lo, hi in BOUNDS:
        out.append((values[name] - lo) / (hi - lo))
    return np.clip(np.array(out, dtype=float), 0.0, 1.0)


def raw_forces(
    data: DataView, p: dict[str, float], static_extra_mu: float
) -> tuple[np.ndarray, np.ndarray]:
    vf_rel = data.vf_rel
    vr_rel = data.vr_rel
    normal = data.normal
    vmag2 = vf_rel * vf_rel + vr_rel * vr_rel
    stribeck = (
        p["mu_slide"]
        + static_extra_mu
        * (p["stribeck_speed_mps"] * p["stribeck_speed_mps"])
        / np.maximum(p["stribeck_speed_mps"] * p["stribeck_speed_mps"] + vmag2, 1.0e-12)
    )
    low_fr = normal * stribeck * smooth_sat(vr_rel, p["lateral_sign_eps_mps"])
    local_forward_flow = data.vf_body - data.yaw_rate * data.r
    alpha = vr_rel / np.sqrt(local_forward_flow * local_forward_flow + p["alpha_floor_mps"] ** 2)
    corner_mu = p["corner_mu_front"] * data.front + p["corner_mu_rear"] * data.rear
    high_fr = normal * corner_mu * smooth_sat(alpha, p["alpha_knee"])
    low_gate = (p["low_speed_gate_mps"] * p["low_speed_gate_mps"]) / np.maximum(
        p["low_speed_gate_mps"] * p["low_speed_gate_mps"] + data.vf_abs * data.vf_abs,
        1.0e-12,
    )
    force_r = low_gate * low_fr + (1.0 - low_gate) * high_fr
    force_f = p["drive_scale"] * data.drive + normal * p["longitudinal_mu"] * smooth_sat(
        vf_rel, p["longitudinal_k_mps"]
    )
    return force_f, force_r


def project_forces(
    force_f: np.ndarray, force_r: np.ndarray, normal: np.ndarray, mu_peak: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    limit = np.maximum(mu_peak * normal, 1.0e-9)
    utilization2 = (force_f * force_f + force_r * force_r) / (limit * limit)
    scale = 1.0 / np.sqrt(1.0 + utilization2)
    return force_f * scale, force_r * scale, np.sqrt(utilization2)


def yaw_moment_from_forces(data: DataView, force_f: np.ndarray, force_r: np.ndarray) -> np.ndarray:
    return np.sum(data.f * force_r - data.r * force_f, axis=1)


def launch_static_solve(
    p: dict[str, float], constants: dict[str, float], target_opposing_nm: float
) -> tuple[float, float]:
    track = constants["track_width_m"]
    half_track = 0.5 * track
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    total_normal = constants["mass_kg"] * 9.80665 + 0.8 * constants.get(
        "fan_downforce_full_duty_n", 0.0
    )
    normal = np.full((1, 4), 0.25 * total_normal)
    r = np.array([[-half_track, half_track, -half_track, half_track]], dtype=float)
    f = np.array([[longitudinal, longitudinal, -longitudinal, -longitudinal]], dtype=float)
    vf_rel = np.zeros((1, 4), dtype=float)
    vr_rel = -f
    vmag2 = vf_rel * vf_rel + vr_rel * vr_rel
    sign_r = smooth_sat(vr_rel, p["lateral_sign_eps_mps"])
    slide = p["mu_slide"] * normal * sign_r
    extra_basis = (
        normal
        * ((p["stribeck_speed_mps"] * p["stribeck_speed_mps"]) / np.maximum(
            p["stribeck_speed_mps"] * p["stribeck_speed_mps"] + vmag2, 1.0e-12
        ))
        * sign_r
    )

    def opposing_for(extra_mu: float) -> float:
        force_r = slide + extra_mu * extra_basis
        force_f = np.zeros_like(force_r)
        pf, pr, _ = project_forces(force_f, force_r, normal, p["mu_peak"])
        moment = float(np.sum(f * pr - r * pf))
        return -moment

    lo = 0.0
    hi = max(p["mu_peak"] * 4.0, p["mu_slide"] + 1.0)
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if opposing_for(mid) < target_opposing_nm:
            lo = mid
        else:
            hi = mid
    solved = 0.5 * (lo + hi)
    error = opposing_for(solved) - target_opposing_nm
    return solved, error


def predict(data: DataView, theta: np.ndarray) -> tuple[np.ndarray, dict[str, float], float, float]:
    p = unpack(theta)
    target_launch = required_total_launch_opposing_nm(data.constants)
    static_extra, launch_error = launch_static_solve(p, data.constants, target_launch)
    ff_raw, fr_raw = raw_forces(data, p, static_extra)
    ff, fr, util = project_forces(ff_raw, fr_raw, data.normal, p["mu_peak"])
    pred = yaw_moment_from_forces(data, ff, fr)
    return pred, p, static_extra, launch_error


def objective(data: DataView, theta: np.ndarray) -> float:
    pred, p, static_extra, launch_error = predict(data, theta)
    error = pred - data.target

    def weighted_rmse(weights: np.ndarray) -> float:
        mask = weights > 0.0
        if not np.any(mask):
            return 0.0
        return float(np.sqrt(np.average(np.square(error[mask]), weights=weights[mask])))

    primary = weighted_rmse(data.train_primary_weights)
    downweighted = weighted_rmse(data.train_downweighted_weights)
    latest = weighted_rmse(data.latest_weights)
    score = (0.72 * primary) + (0.18 * downweighted) + (0.10 * latest)
    score += 0.005 * max(0.0, static_extra - p["mu_peak"] * 1.5)
    score += 0.010 * abs(launch_error)
    score += 0.00015 * (p["corner_mu_front"] + p["corner_mu_rear"] + p["longitudinal_mu"])
    return float(score)


def differential_evolution(
    data: DataView,
    rng: np.random.Generator,
    population_size: int,
    generations: int,
) -> tuple[np.ndarray, float, list[dict[str, object]]]:
    dim = len(BOUNDS)
    seeds = [
        {
            "drive_scale": 0.10,
            "longitudinal_mu": 0.18,
            "longitudinal_k_mps": 0.28,
            "mu_peak": 2.0,
            "mu_slide": 0.08,
            "stribeck_speed_mps": 0.20,
            "low_speed_gate_mps": 0.12,
            "lateral_sign_eps_mps": 0.008,
            "alpha_floor_mps": 0.35,
            "alpha_knee": 0.08,
            "corner_mu_front": 0.3,
            "corner_mu_rear": 0.3,
        },
        {
            "drive_scale": 0.112,
            "longitudinal_mu": 0.21,
            "longitudinal_k_mps": 0.42,
            "mu_peak": 4.5,
            "mu_slide": 0.04,
            "stribeck_speed_mps": 0.23,
            "low_speed_gate_mps": 0.08,
            "lateral_sign_eps_mps": 0.003,
            "alpha_floor_mps": 0.40,
            "alpha_knee": 0.04,
            "corner_mu_front": 0.25,
            "corner_mu_rear": 0.25,
        },
        {
            "drive_scale": 0.09,
            "longitudinal_mu": 0.16,
            "longitudinal_k_mps": 0.16,
            "mu_peak": 1.6,
            "mu_slide": 0.12,
            "stribeck_speed_mps": 0.08,
            "low_speed_gate_mps": 0.16,
            "lateral_sign_eps_mps": 0.010,
            "alpha_floor_mps": 0.20,
            "alpha_knee": 0.10,
            "corner_mu_front": 0.6,
            "corner_mu_rear": 0.45,
        },
    ]
    pop = rng.random((population_size, dim))
    for i, seed in enumerate(seeds[:population_size]):
        pop[i, :] = pack(seed)
    scores = np.array([objective(data, row) for row in pop])
    trace: list[dict[str, object]] = []
    for gen in range(generations):
        for i in range(population_size):
            candidates = [idx for idx in range(population_size) if idx != i]
            a, b, c = rng.choice(candidates, size=3, replace=False)
            mutant = np.clip(pop[a] + 0.65 * (pop[b] - pop[c]), 0.0, 1.0)
            cross = rng.random(dim) < 0.85
            cross[rng.integers(0, dim)] = True
            trial = np.where(cross, mutant, pop[i])
            trial_score = objective(data, trial)
            if trial_score < scores[i]:
                pop[i] = trial
                scores[i] = trial_score
        best_i = int(np.argmin(scores))
        best_p = unpack(pop[best_i])
        trace.append(
            {
                "stage": "differential_evolution",
                "generation": gen,
                "best_score": float(scores[best_i]),
                "median_score": float(np.median(scores)),
                **best_p,
            }
        )
    best_i = int(np.argmin(scores))
    return pop[best_i].copy(), float(scores[best_i]), trace


def coordinate_polish(
    data: DataView, start: np.ndarray, iterations: int
) -> tuple[np.ndarray, float, list[dict[str, object]]]:
    current = start.copy()
    current_score = objective(data, current)
    step = np.full_like(current, 0.08)
    trace: list[dict[str, object]] = []
    for iteration in range(iterations):
        improved = False
        for axis in range(len(current)):
            best_axis = current.copy()
            best_score = current_score
            for direction in [-1.0, 1.0]:
                trial = current.copy()
                trial[axis] = np.clip(trial[axis] + direction * step[axis], 0.0, 1.0)
                score = objective(data, trial)
                if score < best_score:
                    best_score = score
                    best_axis = trial
            if best_score < current_score:
                current = best_axis
                current_score = best_score
                improved = True
        if not improved:
            step *= 0.55
        p = unpack(current)
        trace.append(
            {
                "stage": "coordinate_polish",
                "iteration": iteration,
                "best_score": current_score,
                "max_step": float(np.max(step)),
                **p,
            }
        )
        if float(np.max(step)) < 4.0e-4:
            break
    return current, current_score, trace


def errors_for(frame: pd.DataFrame, pred: np.ndarray) -> np.ndarray:
    return pred - frame["observed_yaw_moment_nm"].to_numpy(float)


def metric_row(label: str, frame: pd.DataFrame, pred: np.ndarray) -> dict[str, object]:
    baseline = frame["baseline_error_nm"].to_numpy(float)
    direct_error = errors_for(frame, pred)
    base_rmse = rmse(baseline)
    direct_rmse = rmse(direct_error)
    return {
        "group": label,
        "count": int(len(frame)),
        "run_count": int(frame["run_id"].nunique()) if len(frame) else 0,
        "baseline_rmse_nm": base_rmse,
        "direct_model_rmse_nm": direct_rmse,
        "rmse_improvement_fraction": (base_rmse - direct_rmse) / base_rmse
        if base_rmse > 0.0
        else math.nan,
        "baseline_mae_nm": mae(baseline),
        "direct_model_mae_nm": mae(direct_error),
        "baseline_median_abs_nm": median_abs(baseline),
        "direct_model_median_abs_nm": median_abs(direct_error),
        "baseline_signed_median_nm": float(np.median(baseline)) if len(frame) else math.nan,
        "direct_model_signed_median_nm": float(np.median(direct_error)) if len(frame) else math.nan,
        "run_balanced_direct_rmse_nm": run_balanced_rmse(frame, direct_error),
    }


def reference_comparison_rows(split_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    brush = read_keyed_csv(BRUSH_DIR / "split_metrics.csv", "split")
    if not brush:
        brush = read_keyed_csv(BRUSH_DIR / "split_metrics.csv", "group")
    bristle = read_keyed_csv(BRISTLE_DIR / "split_metrics.csv", "group")
    scalar = read_keyed_csv(SCALAR_DIR / "split_metrics.csv", "split")
    if not scalar:
        scalar = read_keyed_csv(SCALAR_DIR / "split_metrics.csv", "group")
    standalone = read_keyed_csv(STANDALONE_DIR / "split_metrics.csv", "group")
    force = read_keyed_csv(FORCE_STRIBECK_DIR / "split_rmse.csv", "dataset_split")
    rational = read_keyed_csv(RATIONAL_DIR / "split_metrics.csv", "group")
    rows: list[dict[str, object]] = []
    for row in split_rows:
        group = str(row["group"])
        rows.append(
            {
                "group": group,
                "project_suited_direct_rmse_nm": row["direct_model_rmse_nm"],
                "brush_combined_slip_rmse_nm": brush.get(group, {}).get("brush_rmse", "")
                or brush.get(group, {}).get("brush_rmse_nm", ""),
                "bristle_slip_rmse_nm": bristle.get(group, {}).get("bristle_slip_rmse_nm", ""),
                "scalar_partition_rmse_nm": scalar.get(group, {}).get("scalar_rmse", "")
                or scalar.get(group, {}).get("scalar_rmse_nm", "")
                or scalar.get(group, {}).get("corrected_rmse_nm", ""),
                "standalone_contact_traction_rmse_nm": standalone.get(group, {}).get(
                    "standalone_rmse_nm", ""
                ),
                "force_domain_stribeck_rmse_nm": force.get(group, {}).get("corrected_rmse_nm", ""),
                "rational_residual_reference_rmse_nm": rational.get(group, {}).get(
                    "corrected_rmse_nm", ""
                ),
            }
        )
    return rows


def make_model_params(data: DataView, theta: np.ndarray, score: float) -> ModelParams:
    pred, p, static_extra, launch_error = predict(data, theta)
    target_launch = required_total_launch_opposing_nm(data.constants)
    return ModelParams(
        drive_scale=p["drive_scale"],
        longitudinal_mu=p["longitudinal_mu"],
        longitudinal_k_mps=p["longitudinal_k_mps"],
        mu_peak=p["mu_peak"],
        mu_slide=p["mu_slide"],
        stribeck_speed_mps=p["stribeck_speed_mps"],
        low_speed_gate_mps=p["low_speed_gate_mps"],
        lateral_sign_eps_mps=p["lateral_sign_eps_mps"],
        alpha_floor_mps=p["alpha_floor_mps"],
        alpha_knee=p["alpha_knee"],
        corner_mu_front=p["corner_mu_front"],
        corner_mu_rear=p["corner_mu_rear"],
        derived_static_extra_mu=static_extra,
        launch_total_opposing_nm=target_launch,
        launch_static_solve_error_nm=launch_error,
        objective_score=score,
    )


def launch_row(model: ModelParams, constants: dict[str, float]) -> dict[str, object]:
    achieved_opposing = model.launch_total_opposing_nm + model.launch_static_solve_error_nm
    commands = motor_commands_for_opposing_torque(achieved_opposing, constants, 1.0)
    return {
        "variant": "project_suited_stribeck_slip_angle",
        "yaw_rate_radps": 1.0,
        "target_total_opposing_yaw_torque_nm": model.launch_total_opposing_nm,
        "achieved_total_opposing_yaw_torque_nm": achieved_opposing,
        "derived_static_extra_mu": model.derived_static_extra_mu,
        "launch_solve_error_nm": model.launch_static_solve_error_nm,
        **commands,
        "passes_abs_0p6_gate": commands["max_abs_command"] >= 0.6,
        "target_command_abs": 0.646,
        "target_abs_error": commands["max_abs_command"] - 0.646,
    }


def fmt(value: object, digits: int = 6) -> str:
    try:
        number = float(value)
        if not math.isfinite(number):
            return ""
        return f"{number:.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def markdown_table(rows: list[dict[str, object]], columns: list[str], limit: int | None = None) -> list[str]:
    if limit is not None:
        rows = rows[:limit]
    lines = ["| " + " | ".join(columns) + " |", "| " + " | ".join("---" for _ in columns) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(fmt(row.get(col, "")) for col in columns) + " |")
    return lines


def write_report(
    model: ModelParams,
    split_rows: list[dict[str, object]],
    selected_rows: list[dict[str, object]],
    risk_rows: list[dict[str, object]],
    common_range_rows: list[dict[str, object]],
    latest_rows: list[dict[str, object]],
    comparison_rows: list[dict[str, object]],
    launch: dict[str, object],
    opt_summary: dict[str, object],
) -> None:
    primary = next(r for r in split_rows if r["group"] == "primary_open_floor_fit_authoritative")
    validation = next(r for r in split_rows if r["group"] == "validation_non_authoritative")
    lines: list[str] = []
    lines.append("# Project-Suited Stribeck-To-Slip-Angle Design")
    lines.append("")
    lines.append("Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.")
    lines.append("")
    lines.append("## Design Rationale")
    lines.append("")
    lines.append(
        "The chosen form is a PlantModel-shaped contact law, not a scalar residual table. "
        "It keeps Vehicle-owned facts as inputs, computes per-contact forces from contact relative velocity and geometry, "
        "then accumulates yaw with `sum_i(f_i * F_r_i - r_i * F_f_i)`. "
        "The May 4 launch logs are deliberately visible as a launch constraint and latest-log objective, but they are not promoted to full authority because the provenance report marks them incomplete."
    )
    lines.append("")
    lines.append("The form was chosen before fitting:")
    lines.append("")
    lines.append("- `Vf = 0` uses a conventional rational Stribeck/static breakaway branch with smooth sign `v_r / sqrt(v_r^2 + eps^2)`.")
    lines.append("- Moving-speed lateral force uses a geometry-derived slip-angle proxy `alpha_i = v_rel_r_i / sqrt((Vf - yaw*r_i)^2 + alpha_floor^2)`.")
    lines.append("- Longitudinal drive is distributed by normal load within each left/right bank, not split equally front/rear.")
    lines.append("- Raw longitudinal and lateral patch forces are projected through a cheap smooth force envelope.")
    lines.append("- Runtime operations are `sqrt`, `abs`, clamps, multiplies, and divides only. There is no trig, `exp`, `tanh`, UKF target, command selector, or hidden state machine.")
    lines.append("")
    lines.append("## Equations")
    lines.append("")
    lines.append("For contact `i` at right offset `r_i` and forward offset `f_i`:")
    lines.append("")
    lines.append("`F_drive_i = drive_scale * F_drive_side * N_i / sum_side(N)`")
    lines.append("")
    lines.append("`G_low = v_gate^2 / (v_gate^2 + Vf^2)`")
    lines.append("")
    lines.append("`mu_stribeck_i = mu_slide + mu_static_extra * v_s^2 / (v_s^2 + v_rel_f_i^2 + v_rel_r_i^2)`")
    lines.append("")
    lines.append("`F_r_low_i = N_i * mu_stribeck_i * v_rel_r_i / sqrt(v_rel_r_i^2 + eps^2)`")
    lines.append("")
    lines.append("`alpha_i = v_rel_r_i / sqrt((Vf - yaw_rate*r_i)^2 + alpha_floor^2)`")
    lines.append("")
    lines.append("`F_r_high_i = N_i * mu_corner_axle * alpha_i / sqrt(alpha_i^2 + alpha_knee^2)`")
    lines.append("")
    lines.append("`F_f_raw_i = F_drive_i + N_i * mu_long * v_rel_f_i / sqrt(v_rel_f_i^2 + k_long^2)`")
    lines.append("")
    lines.append("`F_r_raw_i = G_low*F_r_low_i + (1-G_low)*F_r_high_i`")
    lines.append("")
    lines.append("`scale_i = 1 / sqrt(1 + (F_f_raw_i^2 + F_r_raw_i^2)/(mu_peak*N_i)^2)`")
    lines.append("")
    lines.append("`M_yaw = sum_i(f_i*scale_i*F_r_raw_i - r_i*scale_i*F_f_raw_i)`")
    lines.append("")
    lines.append("`mu_static_extra` is solved analytically/numerically from the measured `+/-0.646` in-place launch command at `Vf=0`, `yawRate=1 rad/s` for each optimizer point.")
    lines.append("")
    lines.append("## Optimization")
    lines.append("")
    lines.append(
        "SciPy is not installed in this workspace, so the fit used a continuous custom optimizer: differential evolution followed by coordinate polish. "
        "The objective is not an all-rows blind fit: 72% primary April authoritative, 18% downweighted open-floor, and 10% May 4 latest-log guard, all run-balanced with quality penalties."
    )
    lines.append("")
    lines.extend(markdown_table([opt_summary], list(opt_summary.keys())))
    lines.append("")
    lines.append("## Selected Parameters")
    lines.append("")
    lines.extend(markdown_table([{"parameter": k, "value": v} for k, v in model.__dict__.items()], ["parameter", "value"]))
    lines.append("")
    lines.append("## Bound And Identifiability Notes")
    lines.append("")
    lines.append(
        "Several fitted parameters can hit bounds, and those hits are written to `parameter_bound_hits.csv`. "
        "When that happens, read this result as a design-comparison fit rather than a clean coefficient identification: "
        "the launch/static Stribeck requirement is hard, the per-contact/normal-load/envelope shape is production-aligned, "
        "and the broad data may still prefer the brush or standalone candidates for raw fit quality."
    )
    lines.append("")
    lines.append("## Launch Estimate")
    lines.append("")
    lines.extend(markdown_table([launch], list(launch.keys())))
    lines.append("")
    lines.append("## Fit Results")
    lines.append("")
    lines.extend(
        markdown_table(
            split_rows,
            [
                "group",
                "count",
                "baseline_rmse_nm",
                "direct_model_rmse_nm",
                "rmse_improvement_fraction",
                "run_balanced_direct_rmse_nm",
            ],
        )
    )
    lines.append("")
    lines.append("## Latest Logs")
    lines.append("")
    lines.extend(
        markdown_table(
            latest_rows,
            [
                "run_id",
                "dataset_split",
                "count",
                "baseline_rmse_nm",
                "direct_model_rmse_nm",
                "direct_model_signed_median_nm",
            ],
        )
    )
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
                "direct_model_rmse_nm",
                "direct_model_signed_median_nm",
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
                "direct_model_rmse_nm",
                "direct_model_median_abs_nm",
            ],
        )
    )
    lines.append("")
    lines.append("## Common Range Metrics")
    lines.append("")
    lines.append(
        "These rows use the shared operating-range definitions in `common_range_metrics.csv`; `0.7 m/s` is reported as pre-design turn speed, not high speed."
    )
    lines.append("")
    lines.extend(markdown_table(common_range_rows, COMMON_RANGE_REPORT_COLUMNS))
    lines.append("")
    lines.append("## Candidate Comparison")
    lines.append("")
    lines.extend(markdown_table(comparison_rows, list(comparison_rows[0].keys())))
    lines.append("")
    lines.append("## Production-Shape Implications")
    lines.append("")
    lines.append(
        "If this shape were ever promoted, it belongs inside `PlantModel` as the single plant-equation owner and should use Vehicle-owned geometry/load facts directly. "
        "It should not become a new production type, residual overlay, command/request selector, lookup table, or UKF-dependent target path. "
        "The force-envelope projection should remain a PlantModel concept: raw desired patch forces first, then a continuous envelope/yield projection."
    )
    lines.append("")
    lines.append("## Assessment")
    lines.append("")
    lines.append(
        f"The design satisfies the launch target by construction: max command {fmt(launch['max_abs_command'])}. "
        f"The fitted direct model reaches primary RMSE {fmt(primary['direct_model_rmse_nm'])} Nm and non-authoritative validation RMSE {fmt(validation['direct_model_rmse_nm'])} Nm. "
        "The comparison table should be read separately from the design rationale: a model can be project-suited yet still lose to a broader empirical brush or standalone candidate on current noisy data."
    )
    lines.append("")
    lines.append("## Outputs")
    lines.append("")
    for name in [
        "fit_project_suited_design.py",
        "project_suited_design_report.md",
        "selected_parameters.csv",
        "optimizer_summary.csv",
        "optimizer_trace.csv",
        "split_metrics.csv",
        "latest_weighted_metrics.csv",
        "selected_log_metrics.csv",
        "risk_slices.csv",
        "common_range_metrics.csv",
        "launch_estimate.csv",
        "reference_comparison.csv",
        "parameter_bound_hits.csv",
        "prediction_sample.csv",
        "commands_run.txt",
    ]:
        lines.append(f"- `{name}`")
    (OUT / "project_suited_design_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    constants = read_constants()
    frame = load_frame(constants)
    project_quality = QualityConfig(
        gyro_spike_multiplier=0.25,
        saturation_multiplier=0.25,
        use_limiter_penalty=True,
        limiter_gain=4.0,
        use_low_yaw_no_motion_penalty=False,
        quality_floor=0.02,
    )
    write_regime_diagnostics(
        OUT,
        "primary_",
        compute_regime_weights_for_frame(
            frame,
            RegimeWeightConfig(split_weights=PRIMARY_ONLY_SPLIT_WEIGHTS, quality=project_quality),
        ),
    )
    downweighted_eligible = (
        (frame["dataset_split"].astype(str) == "open_floor_fit_downweighted")
        & (frame["family"].astype(str) == "open_floor")
        & (frame["recommendation"].astype(str) == "fit_downweighted")
    )
    write_regime_diagnostics(
        OUT,
        "downweighted_",
        compute_regime_weights_for_frame(
            frame,
            RegimeWeightConfig(
                split_weights={
                    "primary_open_floor_fit_authoritative": 0.0,
                    "open_floor_fit_downweighted": 1.0,
                    "open_floor_validation_only": 0.0,
                    "diag_validation_only": 0.0,
                    "aux_downweighted_validation": 0.0,
                },
                quality=project_quality,
            ),
            eligible_mask=downweighted_eligible,
        ),
    )
    latest_eligible = frame["run_id"].astype(str).isin(["2026-05-04_20-35-47", "2026-05-04_16-57-53"])
    write_regime_diagnostics(
        OUT,
        "latest_",
        compute_regime_weights_for_frame(
            frame,
            RegimeWeightConfig(
                split_weights={
                    "primary_open_floor_fit_authoritative": 1.0,
                    "open_floor_fit_downweighted": 1.0,
                    "open_floor_validation_only": 1.0,
                    "diag_validation_only": 1.0,
                    "aux_downweighted_validation": 1.0,
                },
                quality=project_quality,
            ),
            eligible_mask=latest_eligible,
        ),
    )
    rng = np.random.default_rng(20260527)
    full_data = DataView(frame, constants)

    fit_weight = (
        full_data.train_primary_weights
        + 0.25 * full_data.train_downweighted_weights
        + 0.15 * full_data.latest_weights
    )
    fit_frame = frame[fit_weight > 0.0].copy()
    fit_probs = fit_weight[fit_weight > 0.0]
    fit_probs = fit_probs / np.sum(fit_probs)
    sample_count = min(22000, len(fit_frame))
    sample_index = rng.choice(len(fit_frame), size=sample_count, replace=False, p=fit_probs)
    sampled_frame = fit_frame.iloc[np.sort(sample_index)].copy()
    sampled_data = DataView(sampled_frame, constants)
    polish_data = DataView(fit_frame, constants)

    de_best, de_score, de_trace = differential_evolution(
        sampled_data, rng, population_size=54, generations=58
    )
    polished, polished_score, polish_trace = coordinate_polish(polish_data, de_best, iterations=90)
    trace = de_trace + polish_trace
    write_csv(OUT / "optimizer_trace.csv", trace)

    pred, _, _, _ = predict(full_data, polished)
    model = make_model_params(full_data, polished, polished_score)
    common_range_rows = write_common_range_metrics(
        OUT / "common_range_metrics.csv",
        frame,
        frame["baseline_error_nm"].to_numpy(float),
        pred - frame["observed_yaw_moment_nm"].to_numpy(float),
        "project_suited_design",
    )

    split_rows: list[dict[str, object]] = []
    for split in SPLITS:
        mask = frame["dataset_split"].astype(str).to_numpy() == split
        split_rows.append(metric_row(split, frame[mask], pred[mask]))
    validation_mask = frame["dataset_split"].astype(str).to_numpy() != "primary_open_floor_fit_authoritative"
    split_rows.append(metric_row("validation_non_authoritative", frame[validation_mask], pred[validation_mask]))
    write_csv(OUT / "split_metrics.csv", split_rows)

    selected_rows: list[dict[str, object]] = []
    for run_id in SELECTED_RUNS:
        mask = frame["run_id"].astype(str).to_numpy() == run_id
        subset = frame[mask]
        if len(subset):
            row = metric_row(run_id, subset, pred[mask])
            row["run_id"] = run_id
            row["dataset_split"] = str(subset["dataset_split"].mode().iloc[0])
            selected_rows.append(row)
        else:
            selected_rows.append({"group": run_id, "run_id": run_id, "present": False})
    write_csv(OUT / "selected_log_metrics.csv", selected_rows)

    latest_rows = [
        row for row in selected_rows if str(row.get("run_id", "")).startswith("2026-05-04")
    ]
    latest_mask = frame["run_id"].astype(str).isin(["2026-05-04_20-35-47", "2026-05-04_16-57-53"]).to_numpy()
    latest_rows.append(metric_row("may4_latest_combined", frame[latest_mask], pred[latest_mask]))
    latest_rows[-1]["run_id"] = "may4_latest_combined"
    latest_rows[-1]["dataset_split"] = "mixed_downweighted_and_validation"
    write_csv(OUT / "latest_weighted_metrics.csv", latest_rows)

    risk_masks = {
        "calibration_low_vf_nonzero_yaw": (
            (frame["abs_forward_velocity_mps"].to_numpy(float) < 0.15)
            & (frame["abs_yaw_rate_radps"].to_numpy(float) >= 0.1)
        ),
        "in_place_scrub": (
            (frame["abs_forward_velocity_mps"].to_numpy(float) < 0.05)
            & (frame["abs_yaw_rate_radps"].to_numpy(float) >= 0.2)
        ),
        "slow_forward_turn": (
            (frame["abs_forward_velocity_mps"].to_numpy(float) >= 0.15)
            & (frame["abs_forward_velocity_mps"].to_numpy(float) < 0.70)
            & (frame["abs_yaw_rate_radps"].to_numpy(float) >= 0.1)
        ),
        "pre_design_turn_speed": (
            (frame["abs_forward_velocity_mps"].to_numpy(float) >= 0.70)
            & (frame["abs_forward_velocity_mps"].to_numpy(float) < 0.95)
            & (frame["abs_yaw_rate_radps"].to_numpy(float) >= 0.1)
        ),
        "design_turn_speed_and_up": (
            (frame["abs_forward_velocity_mps"].to_numpy(float) >= 0.95)
            & (frame["abs_yaw_rate_radps"].to_numpy(float) >= 0.1)
        ),
        "fast_forward": frame["abs_forward_velocity_mps"].to_numpy(float) >= 1.50,
        "straightish_forward": (
            (frame["abs_yaw_rate_radps"].to_numpy(float) < 0.05)
            & (frame["abs_forward_velocity_mps"].to_numpy(float) >= 0.05)
        ),
        "limiter_active": frame["max_force_limiter_activity"].to_numpy(float) > 0.0,
        "hardware_saturation_evidence": frame["hardware_saturation_evidence"].to_numpy(float) > 0.0,
        "may4_latest_logs": latest_mask,
    }
    risk_rows = []
    for name, mask in risk_masks.items():
        risk_rows.append(metric_row(name, frame[mask], pred[mask]))
    write_csv(OUT / "risk_slices.csv", risk_rows)

    comparison = reference_comparison_rows(split_rows)
    write_csv(OUT / "reference_comparison.csv", comparison)

    launch = launch_row(model, constants)
    write_csv(OUT / "launch_estimate.csv", [launch])
    write_csv(OUT / "selected_parameters.csv", [{"parameter": k, "value": v} for k, v in model.__dict__.items()])
    bound_rows = []
    model_values = model.__dict__
    for name, lo, hi in BOUNDS:
        value = float(model_values[name])
        span = hi - lo
        bound_rows.append(
            {
                "parameter": name,
                "lower": lo,
                "value": value,
                "upper": hi,
                "near_lower": value <= lo + 0.01 * span,
                "near_upper": value >= hi - 0.01 * span,
            }
        )
    write_csv(OUT / "parameter_bound_hits.csv", bound_rows)

    opt_summary = {
        "optimizer": "custom differential_evolution_plus_coordinate_polish",
        "scipy_available": False,
        "de_population": 54,
        "de_generations": 58,
        "de_sample_rows": sample_count,
        "polish_rows": len(fit_frame),
        "polish_iterations": len(polish_trace),
        "de_best_objective": de_score,
        "final_objective": polished_score,
        "objective_primary_weight": 0.72,
        "objective_downweighted_weight": 0.18,
        "objective_latest_may4_weight": 0.10,
    }
    write_csv(OUT / "optimizer_summary.csv", [opt_summary])

    sample = frame[
        [
            "run_id",
            "row_index",
            "dataset_split",
            "physics_phase",
            "forward_velocity_mps",
            "yaw_rate_radps",
            "observed_yaw_moment_nm",
            "model_yaw_moment_nm",
            "baseline_error_nm",
        ]
    ].copy()
    sample["project_suited_yaw_moment_nm"] = pred
    sample["project_suited_error_nm"] = pred - frame["observed_yaw_moment_nm"].to_numpy(float)
    sample.iloc[:: max(len(sample) // 700, 1)].to_csv(OUT / "prediction_sample.csv", index=False)

    (OUT / "commands_run.txt").write_text(
        "python codex_analysis\\yaw_model_variant_fits\\stribeck_to_slip_angle_candidates\\project_suited_design\\fit_project_suited_design.py\n",
        encoding="utf-8",
    )

    write_report(
        model,
        split_rows,
        selected_rows,
        risk_rows,
        common_range_rows,
        latest_rows,
        comparison,
        launch,
        opt_summary,
    )


if __name__ == "__main__":
    main()
