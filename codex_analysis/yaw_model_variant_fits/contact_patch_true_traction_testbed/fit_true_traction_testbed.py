#!/usr/bin/env python3
"""Contact-patch true-traction yaw model testbed.

Analysis-only tooling. Reads existing yaw_model_variant_fits artifacts and
writes outputs only beside this script.
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


ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from codex_analysis.yaw_model_variant_fits.regime_weighting import (
    LATEST_INCLUSIVE_FIT_SPLIT_WEIGHTS,
    QualityConfig,
    RegimeWeightConfig,
    add_forward_accel_columns_to_frame,
    compute_regime_weights_for_frame,
    latest_inclusive_fit_mask_for_frame,
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
MOVING_COEFFS = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "transition_options"
    / "cubic_smoothstep_partition"
    / "moving_contact_coefficients.csv"
)
RATIONAL_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "transition_options"
    / "rational_speed_force_blend"
)
FORCE_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "round2_force_domain_stribeck"
)
CUBIC_DIR = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "transition_options"
    / "cubic_smoothstep_partition"
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
    "fl_force_f_n",
    "fr_force_f_n",
    "rl_force_f_n",
    "rr_force_f_n",
    "fl_force_r_n",
    "fr_force_r_n",
    "rl_force_r_n",
    "rr_force_r_n",
]

CONTACTS = ["fl", "fr", "rl", "rr"]


@dataclass(frozen=True)
class Candidate:
    name: str
    gate_scope: str
    reserve_yield: str
    speed_knee_mps: float
    util_knee: float
    rel_weight: float
    speed_fade_mps: float
    force_sliding_nm: float
    primary_rmse_nm: float
    primary_regime_rmse_nm: float
    validation_rmse_nm: float
    validation_rb_rmse_nm: float
    validation_regime_rmse_nm: float
    forward_ge_0p5_rmse_nm: float
    straight_rmse_nm: float
    low_speed_yaw_rmse_nm: float
    in_place_max_abs_command: float
    in_place_extra_opposing_nm: float
    score: float


def read_constants() -> dict[str, float]:
    table = pd.read_csv(CONSTANTS_INPUT)
    return {str(row.name): float(row.value) for row in table.itertuples(index=False)}


def sign_array(values: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    return (values > eps).astype(float) - (values < -eps).astype(float)


def smooth_positive(values: np.ndarray | float, epsilon: float = 1.0e-6) -> np.ndarray | float:
    return 0.5 * (values + np.sqrt(values * values + epsilon * epsilon))


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


def weighted_rmse(values: np.ndarray, weights: np.ndarray) -> float:
    mask = weights > 0.0
    if not np.any(mask):
        return math.nan
    return float(np.sqrt(np.average(np.square(values[mask]), weights=weights[mask])))


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
        for suffix in ["v_rel_f_mps", "v_rel_r_mps", "force_f_n", "force_r_n"]:
            frame[f"{contact}_{suffix}"] = frame[f"{contact}_{suffix}"].fillna(0.0)

    yaw_sign = sign_array(frame["yaw_rate_radps"].to_numpy())
    fallback = sign_array(frame["patch_yaw_force_basis_nm"].to_numpy())
    yaw_sign = np.where(yaw_sign == 0.0, fallback, yaw_sign)
    yaw_sign = np.where(yaw_sign == 0.0, 1.0, yaw_sign)
    frame["yaw_sign"] = yaw_sign
    frame["abs_forward_velocity_mps"] = frame["forward_velocity_mps"].abs()
    frame["abs_yaw_rate_radps"] = frame["yaw_rate_radps"].abs()

    half_track = 0.5 * constants["track_width_m"]
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    positions = {
        "fl": (-half_track, longitudinal),
        "fr": (half_track, longitudinal),
        "rl": (-half_track, -longitudinal),
        "rr": (half_track, -longitudinal),
    }
    total_normal = np.maximum(frame["total_normal_load_n"].to_numpy(), 1.0e-9)
    load_weighted_rel = np.zeros(len(frame))
    gain_right = np.zeros(len(frame))
    gain_long = np.zeros(len(frame))
    force_moment = np.zeros(len(frame))
    abs_force_moment = np.zeros(len(frame))
    for contact, (r_pos, f_pos) in positions.items():
        vf_rel = frame[f"{contact}_v_rel_f_mps"].to_numpy()
        vr_rel = frame[f"{contact}_v_rel_r_mps"].to_numpy()
        force_f = frame[f"{contact}_force_f_n"].to_numpy()
        force_r = frame[f"{contact}_force_r_n"].to_numpy()
        local_moment = f_pos * force_r - r_pos * force_f
        force_moment += local_moment
        abs_force_moment += np.abs(local_moment)
        gain_right += -yaw_sign * f_pos * vr_rel
        gain_long += yaw_sign * r_pos * vf_rel
        normal_frac = frame[f"{contact}_normal_n"].to_numpy() / total_normal
        load_weighted_rel += normal_frac * np.sqrt(vf_rel * vf_rel + vr_rel * vr_rel)

    limiter = np.clip(frame["max_force_limiter_activity"].to_numpy(), 0.0, 5.0)
    frame["limiter_smooth"] = limiter / (1.0 + limiter)
    # Projected/actual force utilization, not preprojection utilization.
    force_util = np.clip(abs_force_moment / np.maximum(abs_force_moment + 0.035, 1.0e-12), 0.0, 1.0)
    frame["force_util_smooth"] = force_util
    frame["gain_right_total_basis"] = gain_right
    frame["gain_long_total_basis"] = gain_long
    frame["force_moment_opposes_yaw_nm"] = -yaw_sign * force_moment
    frame["load_weighted_rel_mps"] = load_weighted_rel

    mu_ref = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / nominal_total_load
    total_load = sum(frame[f"{contact}_normal_n"] for contact in CONTACTS)
    frame["yield_longitudinal_moment_nm"] = mu_ref * half_track * total_load
    frame["yield_full_yaw_moment_nm"] = mu_ref * math.hypot(half_track, longitudinal) * total_load
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
    return add_forward_accel_columns_to_frame(frame.reset_index(drop=True)), metadata


def schedules(frame: pd.DataFrame, vrel_knee: float = 0.06, fwd_knee: float = 0.70) -> dict[str, np.ndarray]:
    vrel = np.maximum(frame["vbar_rel_mps"].to_numpy(), frame["load_weighted_rel_mps"].to_numpy())
    vf = frame["abs_forward_velocity_mps"].to_numpy()
    low_rel = 1.0 / (1.0 + np.square(vrel / vrel_knee))
    low_forward = 1.0 / (1.0 + np.square(vf / fwd_knee))
    return {
        "base": np.ones(len(frame)),
        "low_rel": low_rel,
        "high_forward": 1.0 - low_forward,
        "util": frame["force_util_smooth"].to_numpy(),
    }


def read_moving_coefficients() -> dict[tuple[str, str], float]:
    out: dict[tuple[str, str], float] = {}
    with MOVING_COEFFS.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            feature = row["feature"]
            base, suffix = feature.split("__", 1)
            scale = max(float(row["feature_scale"]), 1.0e-12)
            out[(base, suffix)] = float(row["standardized_coefficient_nm"]) / scale
    return out


def moving_coefficients_by_row(frame: pd.DataFrame) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    coeffs = read_moving_coefficients()
    sched = schedules(frame)

    def row_coeff(base: str) -> np.ndarray:
        value = np.zeros(len(frame))
        for suffix, schedule in sched.items():
            value += coeffs.get((base, suffix), 0.0) * schedule
        return value

    k_right = row_coeff("gain_right_total_basis")
    k_long = row_coeff("gain_long_total_basis")
    k_force = row_coeff("force_moment_opposes_yaw_nm")
    return k_right, k_long, k_force


def moving_contact_forces(frame: pd.DataFrame, constants: dict[str, float]) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], np.ndarray]:
    k_right, k_long, k_force = moving_coefficients_by_row(frame)
    delta_f: dict[str, np.ndarray] = {}
    delta_r: dict[str, np.ndarray] = {}
    yaw_sign = frame["yaw_sign"].to_numpy()
    half_track = 0.5 * constants["track_width_m"]
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    positions = {
        "fl": (-half_track, longitudinal),
        "fr": (half_track, longitudinal),
        "rl": (-half_track, -longitudinal),
        "rr": (half_track, -longitudinal),
    }
    pred_opposes = np.zeros(len(frame))
    for contact, (r_pos, f_pos) in positions.items():
        df = k_long * frame[f"{contact}_v_rel_f_mps"].to_numpy() + k_force * frame[
            f"{contact}_force_f_n"
        ].to_numpy()
        dr = k_right * frame[f"{contact}_v_rel_r_mps"].to_numpy() + k_force * frame[
            f"{contact}_force_r_n"
        ].to_numpy()
        delta_f[contact] = df
        delta_r[contact] = dr
        pred_opposes += yaw_sign * r_pos * df - yaw_sign * f_pos * dr
    return delta_f, delta_r, pred_opposes


def true_patch_prediction(
    frame: pd.DataFrame,
    constants: dict[str, float],
    c_delta_f: dict[str, np.ndarray],
    c_delta_r: dict[str, np.ndarray],
    cand: Candidate | dict[str, object],
) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, np.ndarray], np.ndarray]:
    speed_knee = float(cand.speed_knee_mps if isinstance(cand, Candidate) else cand["speed_knee_mps"])
    util_knee = float(cand.util_knee if isinstance(cand, Candidate) else cand["util_knee"])
    rel_weight = float(cand.rel_weight if isinstance(cand, Candidate) else cand["rel_weight"])
    speed_fade = float(cand.speed_fade_mps if isinstance(cand, Candidate) else cand["speed_fade_mps"])
    force_sliding = float(cand.force_sliding_nm if isinstance(cand, Candidate) else cand["force_sliding_nm"])
    gate_scope = str(cand.gate_scope if isinstance(cand, Candidate) else cand["gate_scope"])
    reserve_yield = str(cand.reserve_yield if isinstance(cand, Candidate) else cand["reserve_yield"])

    yaw_sign = frame["yaw_sign"].to_numpy()
    vf_abs = frame["abs_forward_velocity_mps"].to_numpy()
    half_track = 0.5 * constants["track_width_m"]
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    positions = {
        "fl": (-half_track, longitudinal),
        "fr": (half_track, longitudinal),
        "rl": (-half_track, -longitudinal),
        "rr": (half_track, -longitudinal),
    }
    mu_ref = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / float(
        frame["total_normal_load_n"].median()
    )

    yield_weight: dict[str, np.ndarray] = {}
    drive: dict[str, np.ndarray] = {}
    speed_low: dict[str, np.ndarray] = {}
    speed_relief: dict[str, np.ndarray] = {}
    util_gate: dict[str, np.ndarray] = {}
    denom = np.zeros(len(frame))
    for contact, (r_pos, f_pos) in positions.items():
        normal = frame[f"{contact}_normal_n"].to_numpy()
        lever = abs(r_pos) if reserve_yield == "longitudinal" else math.hypot(r_pos, f_pos)
        yield_i = np.maximum(mu_ref * normal * lever, 1.0e-12)
        yield_weight[contact] = yield_i
        denom += yield_i
        force_f = frame[f"{contact}_force_f_n"].to_numpy()
        force_r = frame[f"{contact}_force_r_n"].to_numpy()
        local_moment = f_pos * force_r - r_pos * force_f
        # Positive means the current projected contact force is driving yaw in
        # the same direction as the measured yaw rate, so static reserve should
        # oppose it. This is actual/projected force state, not request state.
        drive[contact] = smooth_positive(yaw_sign * local_moment)
        vrel = np.sqrt(
            np.square(frame[f"{contact}_v_rel_f_mps"].to_numpy())
            + np.square(frame[f"{contact}_v_rel_r_mps"].to_numpy())
        )
        v2 = np.square(vf_abs) + np.square(rel_weight * vrel)
        speed_low[contact] = speed_knee * speed_knee / np.maximum(speed_knee * speed_knee + v2, 1.0e-12)
        speed_relief[contact] = speed_fade * speed_fade / np.maximum(
            speed_fade * speed_fade + v2, 1.0e-12
        )
        u = drive[contact] / yield_i
        util_gate[contact] = np.square(u) / np.maximum(np.square(u) + util_knee * util_knee, 1.0e-12)

    if gate_scope == "bank_aggregate":
        drive_sum = sum(drive.values())
        yield_sum = np.maximum(sum(yield_weight.values()), 1.0e-12)
        v2 = np.square(vf_abs) + np.square(rel_weight * frame["vbar_rel_mps"].to_numpy())
        low = speed_knee * speed_knee / np.maximum(speed_knee * speed_knee + v2, 1.0e-12)
        u = drive_sum / yield_sum
        ug = np.square(u) / np.maximum(np.square(u) + util_knee * util_knee, 1.0e-12)
        sg = low * ug
        for contact in CONTACTS:
            speed_low[contact] = sg
            util_gate[contact] = np.ones(len(frame))

    out_f: dict[str, np.ndarray] = {}
    out_r: dict[str, np.ndarray] = {}
    pred_opposes = np.zeros(len(frame))
    blend_samples = []
    for contact, (r_pos, f_pos) in positions.items():
        reserve_moment = (
            force_sliding
            * yield_weight[contact]
            / np.maximum(denom, 1.0e-12)
            * speed_relief[contact]
            * util_gate[contact]
        )
        reserve_force_f = yaw_sign * (1.0 if r_pos >= 0.0 else -1.0) * reserve_moment / max(
            abs(r_pos), 1.0e-12
        )
        reserve_force_r = np.zeros(len(frame))
        g = np.clip(speed_low[contact] * util_gate[contact], 0.0, 1.0)
        out_f[contact] = (1.0 - g) * c_delta_f[contact] + g * reserve_force_f
        out_r[contact] = (1.0 - g) * c_delta_r[contact] + g * reserve_force_r
        pred_opposes += yaw_sign * r_pos * out_f[contact] - yaw_sign * f_pos * out_r[contact]
        blend_samples.append(g)
    return pred_opposes, out_f, out_r, np.mean(np.vstack(blend_samples), axis=0)


def corrected_residuals(frame: pd.DataFrame, pred_opposes: np.ndarray) -> np.ndarray:
    pred_additive = -frame["yaw_sign"].to_numpy() * pred_opposes
    return frame["residual_additive_yaw_torque_nm"].to_numpy() - pred_additive


def metric_row(label: str, frame: pd.DataFrame, pred_opposes: np.ndarray, mean_blend: np.ndarray | None = None) -> dict[str, object]:
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
        "rmse_delta_vs_baseline_pct": 100.0 * (corr_rmse - base_rmse) / base_rmse if base_rmse else math.nan,
        "corrected_mae_nm": mae(corrected),
        "corrected_median_abs_nm": median_abs(corrected),
        "run_balanced_corrected_rmse_nm": run_balanced_rmse(frame, corrected),
        "median_pred_opposes_nm": float(np.median(pred_opposes)) if len(pred_opposes) else math.nan,
        "mean_blend": float(np.mean(mean_blend)) if mean_blend is not None and len(mean_blend) else "",
    }


def torque_from_command(command: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]
    applied_voltage = command * battery
    back_emf_per_wheel_radps = gear_ratio / speed_constant
    current = (applied_voltage / resistance) - ((wheel_speed_radps * back_emf_per_wheel_radps) / resistance)
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


def motor_commands_for_opposing_torque(opposing_yaw_torque: float, constants: dict[str, float], yaw_rate: float) -> dict[str, float]:
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
        "left_command": command_from_torque(left_command_torque, left_speed, constants),
        "right_command": command_from_torque(right_command_torque, right_speed, constants),
        "lr_delta_command": command_from_torque(left_command_torque, left_speed, constants)
        - command_from_torque(right_command_torque, right_speed, constants),
    }


def synthetic_in_place(cand: dict[str, object], constants: dict[str, float]) -> dict[str, float | bool]:
    yaw_rate = 1.0
    half_track = 0.5 * constants["track_width_m"]
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    total_load = constants["mass_kg"] * 9.80665 + constants.get("fan_downforce_full_duty_n", 0.0)
    mu_ref = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / total_load
    speed_knee = float(cand["speed_knee_mps"])
    util_knee = float(cand["util_knee"])
    rel_weight = float(cand["rel_weight"])
    speed_fade = float(cand["speed_fade_mps"])
    force_sliding = float(cand["force_sliding_nm"])
    reserve_yield = str(cand["reserve_yield"])
    lever = half_track if reserve_yield == "longitudinal" else math.hypot(half_track, longitudinal)
    yield_total = mu_ref * total_load * lever
    v_rel = math.hypot(0.0, longitudinal * yaw_rate)
    v2 = (rel_weight * v_rel) ** 2
    speed_low = speed_knee * speed_knee / (speed_knee * speed_knee + v2)
    speed_relief = speed_fade * speed_fade / (speed_fade * speed_fade + v2)
    # Diagnostic launch reference assumes projected contact force has reached
    # the local yaw yield envelope; it is not used as a selection constraint.
    util_gate = 1.0 / (1.0 + util_knee * util_knee)
    blend = speed_low * util_gate

    k_right, _, _ = synthetic_moving_coefficients(v_rel, 0.0)
    c_extra = k_right * (4.0 * longitudinal * longitudinal * abs(yaw_rate))
    reserve_extra = force_sliding * speed_relief * util_gate
    extra = (1.0 - blend) * c_extra + blend * reserve_extra
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    total = base + extra
    commands = motor_commands_for_opposing_torque(total, constants, yaw_rate)
    return {
        "extra_opposing_yaw_torque_nm": extra,
        "base_opposing_yaw_torque_nm": base,
        "total_opposing_yaw_torque_nm": total,
        "left_command": commands["left_command"],
        "right_command": commands["right_command"],
        "lr_delta_command": commands["lr_delta_command"],
        "max_abs_command": max(abs(commands["left_command"]), abs(commands["right_command"])),
        "passes_abs_0p6_gate": max(abs(commands["left_command"]), abs(commands["right_command"])) >= 0.6,
        "launch_lock_policy": "diagnostic_only",
        "speed_low": speed_low,
        "util_gate": util_gate,
        "blend": blend,
        "nominal_yield_nm": yield_total,
    }


def synthetic_moving_coefficients(vrel: float, vf: float) -> tuple[float, float, float]:
    coeffs = read_moving_coefficients()
    low_rel = 1.0 / (1.0 + (vrel / 0.06) ** 2)
    low_forward = 1.0 / (1.0 + (abs(vf) / 0.70) ** 2)
    sched = {
        "base": 1.0,
        "low_rel": low_rel,
        "high_forward": 1.0 - low_forward,
        "util": 1.0,
    }

    def one(base: str) -> float:
        return sum(coeffs.get((base, suffix), 0.0) * value for suffix, value in sched.items())

    return one("gain_right_total_basis"), one("gain_long_total_basis"), one("force_moment_opposes_yaw_nm")


def evaluate_candidates(frame: pd.DataFrame, constants: dict[str, float], metadata: dict[str, float]) -> tuple[list[Candidate], dict[str, object], np.ndarray, np.ndarray]:
    c_delta_f, c_delta_r, c_pred = moving_contact_forces(frame, constants)
    c_corr = corrected_residuals(frame, c_pred)
    primary_weight_result = compute_regime_weights_for_frame(
        frame,
        RegimeWeightConfig(
            split_weights=LATEST_INCLUSIVE_FIT_SPLIT_WEIGHTS,
            quality=QualityConfig(
                gyro_spike_multiplier=0.10,
                saturation_multiplier=0.35,
                use_limiter_penalty=True,
                use_low_yaw_no_motion_penalty=False,
            ),
        ),
        eligible_mask=latest_inclusive_fit_mask_for_frame(frame),
    )
    validation_split_weights = {
        "primary_open_floor_fit_authoritative": 0.0,
        "open_floor_fit_downweighted": 0.25,
        "open_floor_validation_only": 1.0,
        "diag_validation_only": 1.0,
        "aux_downweighted_validation": 1.0,
    }
    validation_weight_result = compute_regime_weights_for_frame(
        frame,
        RegimeWeightConfig(
            split_weights=validation_split_weights,
            quality=QualityConfig(
                gyro_spike_multiplier=0.10,
                saturation_multiplier=0.35,
                use_limiter_penalty=True,
                use_low_yaw_no_motion_penalty=False,
            ),
        ),
    )
    primary_weights = np.asarray(primary_weight_result.weights, dtype=float)
    validation_weights = np.asarray(validation_weight_result.weights, dtype=float)
    candidates: list[Candidate] = []
    grid = []
    for gate_scope in ["per_patch", "bank_aggregate"]:
        for reserve_yield in ["longitudinal", "full_yaw"]:
            for speed_knee in [0.50, 0.65]:
                for util_knee in [0.05, 0.10, 0.16]:
                    for rel_weight in [0.75, 1.00]:
                        for force_sliding in [0.0000, 0.0250, 0.0500, 0.0750, 0.1000, 0.1250]:
                            grid.append(
                                {
                                    "name": f"{gate_scope}_{reserve_yield}",
                                    "gate_scope": gate_scope,
                                    "reserve_yield": reserve_yield,
                                    "speed_knee_mps": speed_knee,
                                    "util_knee": util_knee,
                                    "rel_weight": rel_weight,
                                    "speed_fade_mps": 0.64,
                                    "force_sliding_nm": force_sliding,
                                }
                            )
    for values in grid:
        pred, _, _, blend = true_patch_prediction(frame, constants, c_delta_f, c_delta_r, values)
        train_mask = frame["dataset_split"].to_numpy() == "primary_open_floor_fit_authoritative"
        validation_mask = ~train_mask
        forward_ge_0p5_mask = frame["abs_forward_velocity_mps"].to_numpy() >= 0.5
        straight_mask = (frame["abs_yaw_rate_radps"].to_numpy() < 0.05) & (
            frame["abs_forward_velocity_mps"].to_numpy() >= 0.05
        )
        low_yaw_mask = (frame["abs_forward_velocity_mps"].to_numpy() < 0.05) & (
            frame["abs_yaw_rate_radps"].to_numpy() >= 0.2
        )
        corrected = corrected_residuals(frame, pred)
        train_rmse = rmse(corrected[train_mask])
        train_regime_rmse = weighted_rmse(corrected, primary_weights)
        val_rmse = rmse(corrected[validation_mask])
        val_rb = run_balanced_rmse(
            frame[validation_mask], corrected[validation_mask]
        )
        val_regime_rmse = weighted_rmse(corrected, validation_weights)
        forward_ge_0p5_rmse = rmse(corrected[forward_ge_0p5_mask])
        straight_rmse = rmse(corrected[straight_mask])
        low_yaw_rmse = rmse(corrected[low_yaw_mask])
        in_place = synthetic_in_place(values, constants)
        forward_penalty = max(0.0, forward_ge_0p5_rmse - rmse(c_corr[forward_ge_0p5_mask])) * 0.35
        score = (
            train_regime_rmse
            + 0.10 * val_regime_rmse
            + 0.15 * forward_ge_0p5_rmse
            + forward_penalty
        )
        candidates.append(
            Candidate(
                name=str(values["name"]),
                gate_scope=str(values["gate_scope"]),
                reserve_yield=str(values["reserve_yield"]),
                speed_knee_mps=float(values["speed_knee_mps"]),
                util_knee=float(values["util_knee"]),
                rel_weight=float(values["rel_weight"]),
                speed_fade_mps=float(values["speed_fade_mps"]),
                force_sliding_nm=float(values["force_sliding_nm"]),
                primary_rmse_nm=train_rmse,
                primary_regime_rmse_nm=train_regime_rmse,
                validation_rmse_nm=val_rmse,
                validation_rb_rmse_nm=val_rb,
                validation_regime_rmse_nm=val_regime_rmse,
                forward_ge_0p5_rmse_nm=forward_ge_0p5_rmse,
                straight_rmse_nm=straight_rmse,
                low_speed_yaw_rmse_nm=low_yaw_rmse,
                in_place_max_abs_command=float(in_place["max_abs_command"]),
                in_place_extra_opposing_nm=float(in_place["extra_opposing_yaw_torque_nm"]),
                score=score,
            )
        )
    candidates.sort(key=lambda c: (c.score, c.validation_rb_rmse_nm))
    selected = candidates[0]
    selected_values = {
        "name": selected.name,
        "gate_scope": selected.gate_scope,
        "reserve_yield": selected.reserve_yield,
        "speed_knee_mps": selected.speed_knee_mps,
        "util_knee": selected.util_knee,
        "rel_weight": selected.rel_weight,
        "speed_fade_mps": selected.speed_fade_mps,
        "force_sliding_nm": selected.force_sliding_nm,
    }
    pred, _, _, blend = true_patch_prediction(frame, constants, c_delta_f, c_delta_r, selected_values)
    return candidates, selected_values, pred, blend


def read_existing_split(path: Path, key: str = "group") -> dict[str, dict[str, str]]:
    if not path.exists():
        return {}
    with path.open(newline="", encoding="utf-8") as handle:
        return {row[key]: row for row in csv.DictReader(handle)}


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def fmt(value: object, digits: int = 6) -> str:
    try:
        x = float(value)
        if not math.isfinite(x):
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
        for column in columns:
            value = row.get(column, "")
            values.append(fmt(value))
        lines.append("| " + " | ".join(values) + " |")
    return lines


def write_outputs(
    frame: pd.DataFrame,
    constants: dict[str, float],
    metadata: dict[str, float],
    candidates: list[Candidate],
    selected_values: dict[str, object],
    pred: np.ndarray,
    blend: np.ndarray,
) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    candidate_rows = [candidate.__dict__ for candidate in candidates]
    write_csv(OUT / "candidate_scores.csv", candidate_rows)

    split_rows = []
    for split in SPLITS:
        mask = frame["dataset_split"].to_numpy() == split
        split_rows.append(metric_row(split, frame[mask], pred[mask], blend[mask]))
    validation_mask = frame["dataset_split"].to_numpy() != "primary_open_floor_fit_authoritative"
    split_rows.append(metric_row("validation_non_authoritative", frame[validation_mask], pred[validation_mask], blend[validation_mask]))
    write_csv(OUT / "split_metrics.csv", split_rows)

    selected_rows = []
    for run_id in SELECTED_RUNS:
        subset = frame[frame["run_id"].astype(str) == run_id]
        if len(subset):
            idx = subset.index.to_numpy()
            row = metric_row(run_id, subset, pred[idx], blend[idx])
            row["run_id"] = run_id
            row["dataset_split"] = str(subset["dataset_split"].mode().iloc[0])
            selected_rows.append(row)
        else:
            selected_rows.append({"group": run_id, "run_id": run_id, "present": False})
    write_csv(OUT / "selected_log_metrics.csv", selected_rows)

    phase_rows = []
    for phase, subset in frame.groupby("physics_phase"):
        idx = subset.index.to_numpy()
        phase_rows.append(metric_row(str(phase), subset, pred[idx], blend[idx]))
    write_csv(OUT / "phase_metrics.csv", phase_rows)

    risk_groups = {
        "calibration_low_vf_nonzero_yaw": (
            (frame["abs_forward_velocity_mps"].to_numpy() < 0.15)
            & (frame["abs_yaw_rate_radps"].to_numpy() >= 0.1)
        ),
        "in_place_scrub": (
            (frame["abs_forward_velocity_mps"].to_numpy() < 0.05)
            & (frame["abs_yaw_rate_radps"].to_numpy() >= 0.2)
        ),
        "slow_forward_turn": (
            (frame["abs_forward_velocity_mps"].to_numpy() >= 0.15)
            & (frame["abs_forward_velocity_mps"].to_numpy() < 0.70)
            & (frame["abs_yaw_rate_radps"].to_numpy() >= 0.1)
        ),
        "pre_design_turn_speed": (
            (frame["abs_forward_velocity_mps"].to_numpy() >= 0.70)
            & (frame["abs_forward_velocity_mps"].to_numpy() < 0.95)
            & (frame["abs_yaw_rate_radps"].to_numpy() >= 0.1)
        ),
        "design_turn_speed_and_up": (
            (frame["abs_forward_velocity_mps"].to_numpy() >= 0.95)
            & (frame["abs_yaw_rate_radps"].to_numpy() >= 0.1)
        ),
        "fast_forward": frame["abs_forward_velocity_mps"].to_numpy() >= 1.50,
        "straightish_forward": (
            (frame["abs_yaw_rate_radps"].to_numpy() < 0.05)
            & (frame["abs_forward_velocity_mps"].to_numpy() >= 0.05)
        ),
        "limiter_active": frame["max_force_limiter_activity"].to_numpy() > 0.0,
        "hardware_saturation_evidence": frame["hardware_saturation_evidence"].to_numpy() > 0.0,
        "may4_latest_logs": frame["run_id"].astype(str).isin(["2026-05-04_20-35-47", "2026-05-04_16-57-53"]).to_numpy(),
    }
    risk_rows = []
    for name, mask in risk_groups.items():
        risk_rows.append(metric_row(name, frame[mask], pred[mask], blend[mask]))
    write_csv(OUT / "risk_metrics.csv", risk_rows)
    common_range_rows = write_common_range_metrics(
        OUT / "common_range_metrics.csv",
        frame,
        frame["residual_additive_yaw_torque_nm"].to_numpy(float),
        corrected_residuals(frame, pred),
        "contact_patch_true_traction_testbed",
    )

    in_place = synthetic_in_place(selected_values, constants)
    write_csv(
        OUT / "in_place_1radps_command.csv",
        [
            {
                "variant": "selected_true_patch_force_blend",
                **selected_values,
                **in_place,
            }
        ],
    )

    c_split = read_existing_split(CUBIC_DIR / "split_metrics.csv", "dataset_split")
    r_split = read_existing_split(RATIONAL_DIR / "split_metrics.csv", "group")
    f_split = read_existing_split(FORCE_DIR / "split_rmse.csv", "dataset_split")
    comparison_rows = []
    for row in split_rows:
        group = str(row["group"])
        comparison_rows.append(
            {
                "group": group,
                "true_patch_corrected_rmse_nm": row["corrected_rmse_nm"],
                "true_patch_run_balanced_rmse_nm": row["run_balanced_corrected_rmse_nm"],
                "rational_residual_corrected_rmse_nm": r_split.get(group, {}).get("corrected_rmse_nm", ""),
                "cubic_force_only_partition_rmse_nm": c_split.get(group, {}).get("corrected_rmse_nm", ""),
                "force_domain_stribeck_rmse_nm": f_split.get(group, {}).get("corrected_rmse_nm", ""),
            }
        )
    write_csv(OUT / "baseline_comparison.csv", comparison_rows)

    write_csv(
        OUT / "selected_parameters.csv",
        [{"parameter": key, "value": value} for key, value in selected_values.items()]
        + [{"parameter": key, "value": value} for key, value in metadata.items()],
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
        ]
    ].copy()
    sample["pred_opposes_nm"] = pred
    sample["corrected_residual_nm"] = corrected_residuals(frame, pred)
    sample["mean_patch_blend"] = blend
    sample.iloc[:: max(len(sample) // 500, 1)].to_csv(OUT / "prediction_sample.csv", index=False)

    (OUT / "commands_run.txt").write_text(
        "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' "
        "codex_analysis\\yaw_model_variant_fits\\contact_patch_true_traction_testbed\\fit_true_traction_testbed.py\n",
        encoding="utf-8",
    )
    write_report(metadata, selected_values)


def write_report(metadata: dict[str, float], selected_values: dict[str, object]) -> None:
    candidates = list(csv.DictReader((OUT / "candidate_scores.csv").open(newline="", encoding="utf-8")))
    split_rows = list(csv.DictReader((OUT / "split_metrics.csv").open(newline="", encoding="utf-8")))
    selected_rows = list(csv.DictReader((OUT / "selected_log_metrics.csv").open(newline="", encoding="utf-8")))
    risk_rows = list(csv.DictReader((OUT / "risk_metrics.csv").open(newline="", encoding="utf-8")))
    common_range_rows = list(csv.DictReader((OUT / "common_range_metrics.csv").open(newline="", encoding="utf-8")))
    comparison_rows = list(csv.DictReader((OUT / "baseline_comparison.csv").open(newline="", encoding="utf-8")))
    in_place = list(csv.DictReader((OUT / "in_place_1radps_command.csv").open(newline="", encoding="utf-8")))[0]
    top = candidates[:8]
    selected = candidates[0]
    reserve_lever = "sqrt(r_i^2 + f_i^2)" if str(selected_values.get("reserve_yield")) == "full_yaw" else "|r_i|"

    lines: list[str] = [
        "# Contact-Patch True-Traction Rational Testbed",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not modified.",
        "",
        "## Recommendation",
        "",
        "Viable as a true contact-patch traction formulation, with a qualification: the selected implementation must be expressed as force increments at each contact before yaw moment accumulation. The algebraic version that first computes one yaw residual scalar and then subtracts it remains a residual in disguise and should be rejected for production shape.",
        "",
        "The best low-dimensional testbed candidate uses the compact force-only moving-contact branch as per-contact force increments, then blends those increments with a low-speed longitudinal static/yield reserve at each patch. It uses projected/actual contact forces, normal load, contact-relative velocity, and contact geometry. It does not use command/request/preprojection values as traction selectors and does not use UKF state-vector fields.",
        "",
        "## Selected Equations",
        "",
        "For contact `i`, lateral coordinate `r_i`, longitudinal coordinate `f_i`, normal load `N_i`, projected force `(F_f,i, F_r,i)`, and relative velocity `(v_f,i, v_r,i)`:",
        "",
        "`M_i = f_i*F_r,i - r_i*F_f,i`",
        "",
        "`drive_i = max_smooth( sign(yawRate) * M_i )`",
        "",
        f"`Y_i = mu_ref * N_i * {reserve_lever}` for the selected reserve weighting geometry.",
        "",
        "`u_i = drive_i / Y_i`",
        "",
        "`v2_i = |Vf|^2 + (rel_weight * sqrt(v_f,i^2 + v_r,i^2))^2`",
        "",
        "`G_i = k_v^2/(k_v^2 + v2_i) * u_i^2/(u_i^2 + k_u^2)`",
        "",
        "`R_i = speed_fade^2/(speed_fade^2 + v2_i)`",
        "",
        "`Delta M_reserve_i = K_slide * (Y_i / sum_j Y_j) * R_i * u_i^2/(u_i^2 + k_u^2)`",
        "",
        "`Delta F_reserve_f,i = sign(yawRate) * sign(r_i) * Delta M_reserve_i / |r_i|`, `Delta F_reserve_r,i = 0`",
        "",
        "The moving-contact branch is also force-shaped. Coefficients multiplying `gain_long_total_basis`, `gain_right_total_basis`, and `force_moment_opposes_yaw_nm` are applied as per-contact `Delta F_f`, `Delta F_r`, and projected-force-proportional increments, then yaw support is recomputed from geometry.",
        "",
        "`Delta F_i = (1 - G_i) * Delta F_C,i + G_i * Delta F_reserve_i`",
        "",
        "`M_pred_opposes = sum_i sign(yawRate) * (r_i*Delta F_f,i - f_i*Delta F_r,i)`",
        "",
        "That last line is an accumulation of contact-patch forces. A scalar-only implementation of `M_pred_opposes` is not the accepted production interpretation.",
        "",
        "## Selected Parameters",
        "",
    ]
    param_rows = [{"parameter": key, "value": value} for key, value in selected_values.items()]
    lines.extend(markdown_table(param_rows, ["parameter", "value"]))
    lines.extend(
        [
            "",
            "## Candidate Summary",
            "",
        ]
    )
    lines.extend(
        markdown_table(
            top,
            [
                "name",
                "speed_knee_mps",
                "util_knee",
                "rel_weight",
                "force_sliding_nm",
                "primary_rmse_nm",
                "primary_regime_rmse_nm",
                "validation_rmse_nm",
                "validation_rb_rmse_nm",
                "validation_regime_rmse_nm",
                "forward_ge_0p5_rmse_nm",
                "in_place_max_abs_command",
                "score",
            ],
        )
    )
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
                "group",
                "count",
                "baseline_rmse_nm",
                "corrected_rmse_nm",
                "corrected_mae_nm",
                "corrected_median_abs_nm",
                "run_balanced_corrected_rmse_nm",
                "mean_blend",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Comparison To Existing Evidence",
            "",
        ]
    )
    lines.extend(
        markdown_table(
            comparison_rows,
            [
                "group",
                "true_patch_corrected_rmse_nm",
                "rational_residual_corrected_rmse_nm",
                "cubic_force_only_partition_rmse_nm",
                "force_domain_stribeck_rmse_nm",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Selected Logs",
            "",
        ]
    )
    lines.extend(
        markdown_table(
            selected_rows,
            [
                "run_id",
                "dataset_split",
                "count",
                "baseline_rmse_nm",
                "corrected_rmse_nm",
                "corrected_mae_nm",
                "mean_blend",
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
                "run_balanced_corrected_rmse_nm",
                "mean_blend",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Common Range Metrics",
            "",
            "These rows use the shared operating-range definitions in `common_range_metrics.csv`; `0.7 m/s` is reported as pre-design turn speed, not high speed.",
            "",
        ]
    )
    lines.extend(markdown_table(common_range_rows, COMMON_RANGE_REPORT_COLUMNS))
    lines.extend(
        [
            "",
            "## In-Place Command",
            "",
        ]
    )
    lines.extend(
        markdown_table(
            [in_place],
            [
                "extra_opposing_yaw_torque_nm",
                "total_opposing_yaw_torque_nm",
                "left_command",
                "right_command",
                "max_abs_command",
                "passes_abs_0p6_gate",
                "launch_lock_policy",
                "blend",
            ],
        )
    )
    lines.extend(
        [
            "",
            "The launch check is diagnostic only. It is not used as a score term or candidate-selection gate in this unconstrained run.",
            "",
            "## Hardware Cost",
            "",
            "Per contact, the selected law needs one local moment, one smooth positive or branch max, one relative-speed square sum, two rational gates, one normal-load weight, and one force blend. For four contacts this is roughly four sqrt calls if exact per-patch relative speed is used, eight divisions unless reciprocals are shared, and no trig, exp, tanh, table lookup, or history state. If `sqrt(v_f^2+v_r^2)` is replaced by the already available `vbar_rel` for the gate, the selected shape drops to one shared speed rational plus per-contact utilization rationals.",
            "",
            "## Residual-In-Disguise Check",
            "",
            "- Accepted: the force-level blend implemented here. Both moving branch and reserve branch produce `Delta F_f,i`/`Delta F_r,i`, then yaw support is recomputed from `sum_i r_i*F_f - f_i*F_r`.",
            "- Rejected: computing `M_C`, `M_force`, and `M_pred` as yaw scalars and subtracting `M_pred` after the normal yaw moment accumulation. That is the current residual interpretation in different algebra.",
            "- Borderline: using a bank-aggregate gate is physically defensible only if it is computed from actual projected contact force state. It must not be sourced from command, request, preprojection utilization, or UKF state fields.",
            "",
            "## Provenance",
            "",
            "Feature inputs are the existing selected `yaw_model_variant_fits` shared CSVs. `forward_velocity_mps` is encoder-derived, `yaw_rate_radps` is raw gyro minus stationary bias, residual targets are gyro-differentiated yaw torque residuals against the PlantModel mirror, and contact features are reconstructed from sensor/encoder/drive telemetry. This testbed does not read logged `ukf_state_*`, estimator state-vector, Kalman, or estimator yaw-rate fields.",
            "",
            "## Reproduce",
            "",
            "```powershell",
            "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' codex_analysis\\yaw_model_variant_fits\\contact_patch_true_traction_testbed\\fit_true_traction_testbed.py",
            "```",
            "",
            "## Output Files",
            "",
            "- `fit_true_traction_testbed.py`",
            "- `true_traction_testbed_report.md`",
            "- `candidate_scores.csv`",
            "- `selected_parameters.csv`",
            "- `split_metrics.csv`",
            "- `selected_log_metrics.csv`",
            "- `phase_metrics.csv`",
            "- `risk_metrics.csv`",
            "- `common_range_metrics.csv`",
            "- `baseline_comparison.csv`",
            "- `in_place_1radps_command.csv`",
            "- `prediction_sample.csv`",
            "- `fit_regime_weighting_summary.json`",
            "- `fit_regime_weighting_cells.csv`",
            "- `fit_regime_weighting_marginals.csv`",
            "- `validation_regime_weighting_summary.json`",
            "- `validation_regime_weighting_cells.csv`",
            "- `validation_regime_weighting_marginals.csv`",
            "- `commands_run.txt`",
        ]
    )
    (OUT / "true_traction_testbed_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    constants = read_constants()
    frame, metadata = load_frame(constants)
    shared_quality = QualityConfig(
        gyro_spike_multiplier=0.10,
        saturation_multiplier=0.35,
        use_limiter_penalty=True,
        use_low_yaw_no_motion_penalty=False,
    )
    write_regime_diagnostics(
        OUT,
        "fit_",
        compute_regime_weights_for_frame(
            frame,
            RegimeWeightConfig(split_weights=LATEST_INCLUSIVE_FIT_SPLIT_WEIGHTS, quality=shared_quality),
            eligible_mask=latest_inclusive_fit_mask_for_frame(frame),
        ),
    )
    write_regime_diagnostics(
        OUT,
        "validation_",
        compute_regime_weights_for_frame(
            frame,
            RegimeWeightConfig(
                split_weights={
                    "primary_open_floor_fit_authoritative": 0.0,
                    "open_floor_fit_downweighted": 0.25,
                    "open_floor_validation_only": 1.0,
                    "diag_validation_only": 1.0,
                    "aux_downweighted_validation": 1.0,
                },
                quality=shared_quality,
            ),
        ),
    )
    candidates, selected_values, pred, blend = evaluate_candidates(frame, constants, metadata)
    write_outputs(frame, constants, metadata, candidates, selected_values, pred, blend)


if __name__ == "__main__":
    main()
