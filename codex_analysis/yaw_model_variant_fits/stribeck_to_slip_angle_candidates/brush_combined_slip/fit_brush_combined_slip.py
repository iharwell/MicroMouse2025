#!/usr/bin/env python3
"""Brush/Pacejka-lite combined-slip contact-law fit.

Analysis-only tooling. Writes outputs only beside this script.
"""

from __future__ import annotations

import csv
import json
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
    LATEST_INCLUSIVE_FIT_SPLIT_WEIGHTS,
    QualityConfig,
    RegimeWeightConfig,
    add_forward_accel_columns_to_frame,
    compute_regime_weights_for_frame,
    latest_inclusive_fit_mask_for_frame,
    write_regime_diagnostics,
)
from codex_analysis.yaw_model_variant_fits.common_range_metrics import (
    write_common_range_metrics,
)

PRIMARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "ablation" / "phase_classified_feature_sample.csv"
SECONDARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "contact_continuum_feature_sample.csv"
CONSTANTS = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "plant_mirror_constants.csv"

REF_VARIANT_C = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface"
REF_FORCE_STRIBECK = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck"
REF_RATIONAL = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "transition_options" / "rational_speed_force_blend"
REF_TRUE_TRACTION = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "contact_patch_true_traction_testbed"
REF_STANDALONE = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "standalone_contact_traction_testbed"

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
class FitResult:
    restart: int
    seed_name: str
    converged: bool
    iterations: int
    evaluations: int
    objective: float
    x: np.ndarray


PARAM_BOUNDS = [
    ("drive_scale", 0.035, 0.180),
    ("mu_peak", 0.650, 5.000),
    ("mu_longitudinal", 0.000, 1.500),
    ("mu_low_speed_lateral", 0.000, 1.500),
    ("mu_cornering", 0.000, 5.000),
    ("alpha_knee", 0.020, 0.900),
    ("long_speed_floor_mps", 0.020, 1.000),
    ("speed_gate_knee_mps", 0.030, 1.500),
    ("static_slip_knee_mps", 0.003, 0.100),
    ("longitudinal_slip_knee_mps", 0.010, 0.700),
    ("longitudinal_envelope_shape", 0.450, 2.500),
    ("launch_static_opposing_nm", 0.000, 0.020),
]


def read_constants() -> dict[str, float]:
    table = pd.read_csv(CONSTANTS)
    return {str(row.name): float(row.value) for row in table.itertuples(index=False)}


def load_frame(constants: dict[str, float]) -> pd.DataFrame:
    primary = pd.read_csv(PRIMARY, usecols=PRIMARY_COLUMNS)
    secondary = pd.read_csv(SECONDARY, usecols=SECONDARY_COLUMNS)
    frame = primary.merge(secondary, how="left", on=["run_id", "row_index"])
    numeric = set(PRIMARY_COLUMNS + SECONDARY_COLUMNS) - {
        "run_id",
        "family",
        "schema",
        "recommendation",
        "dataset_split",
        "physics_phase",
    }
    for column in numeric:
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
        nominal_load = constants["mass_kg"] * 9.80665 + 0.8 * constants.get("fan_downforce_full_duty_n", 0.0)
    frame["total_normal_load_n"] = frame["total_normal_load_n"].fillna(nominal_load)
    for contact in CONTACTS:
        frame[f"{contact}_normal_n"] = frame[f"{contact}_normal_n"].fillna(frame["total_normal_load_n"] / 4.0)
        frame[f"{contact}_v_rel_f_mps"] = frame[f"{contact}_v_rel_f_mps"].fillna(0.0)
        frame[f"{contact}_v_rel_r_mps"] = frame[f"{contact}_v_rel_r_mps"].fillna(0.0)
    frame["abs_forward_velocity_mps"] = frame["forward_velocity_mps"].abs()
    frame["abs_yaw_rate_radps"] = frame["yaw_rate_radps"].abs()
    frame["baseline_error_nm"] = frame["model_yaw_moment_nm"] - frame["observed_yaw_moment_nm"]
    return add_forward_accel_columns_to_frame(frame.reset_index(drop=True))


def positions(constants: dict[str, float]) -> dict[str, tuple[float, float, str]]:
    half_track = 0.5 * constants["track_width_m"]
    long_pos = constants["drive_wheel_longitudinal_offset_m"]
    return {
        "fl": (-half_track, long_pos, "left"),
        "fr": (half_track, long_pos, "right"),
        "rl": (-half_track, -long_pos, "left"),
        "rr": (half_track, -long_pos, "right"),
    }


def frame_arrays(frame: pd.DataFrame, constants: dict[str, float]) -> dict[str, object]:
    pos = positions(constants)
    n = len(frame)
    normals = np.vstack([frame[f"{c}_normal_n"].to_numpy(float) for c in CONTACTS])
    vf_rel = np.vstack([frame[f"{c}_v_rel_f_mps"].to_numpy(float) for c in CONTACTS])
    vr_rel = np.vstack([frame[f"{c}_v_rel_r_mps"].to_numpy(float) for c in CONTACTS])
    r = np.array([pos[c][0] for c in CONTACTS], dtype=float)[:, None]
    f = np.array([pos[c][1] for c in CONTACTS], dtype=float)[:, None]
    drive = np.zeros((4, n), dtype=float)
    left_drive = frame["left_drive_force_n"].to_numpy(float)
    right_drive = frame["right_drive_force_n"].to_numpy(float)
    for i, c in enumerate(CONTACTS):
        drive[i, :] = 0.5 * (left_drive if pos[c][2] == "left" else right_drive)
    return {
        "normal": normals,
        "vf_rel": vf_rel,
        "vr_rel": vr_rel,
        "r": r,
        "f": f,
        "drive": drive,
        "vf_abs": frame["abs_forward_velocity_mps"].to_numpy(float)[None, :],
        "target": frame["observed_yaw_moment_nm"].to_numpy(float),
        "baseline_error": frame["baseline_error_nm"].to_numpy(float),
        "run_id": frame["run_id"].astype(str).to_numpy(),
    }


def unpack(x: np.ndarray) -> dict[str, float]:
    return {name: float(lo + np.clip(x[i], 0.0, 1.0) * (hi - lo)) for i, (name, lo, hi) in enumerate(PARAM_BOUNDS)}


def pack(values: dict[str, float]) -> np.ndarray:
    out = []
    for name, lo, hi in PARAM_BOUNDS:
        out.append((float(values[name]) - lo) / (hi - lo))
    return np.clip(np.array(out, dtype=float), 0.0, 1.0)


def sat(value: np.ndarray, knee: float) -> np.ndarray:
    return value / np.sqrt(value * value + knee * knee)


def launch_static_mu(params: dict[str, float], constants: dict[str, float]) -> tuple[float, float]:
    total_load = constants["mass_kg"] * 9.80665 + 0.8 * constants.get("fan_downforce_full_duty_n", 0.0)
    n_q = 0.25 * total_load
    f_abs = constants["drive_wheel_longitudinal_offset_m"]
    vr = f_abs
    desired_force = params["launch_static_opposing_nm"] / max(4.0 * f_abs, 1.0e-12)
    y = params["mu_peak"] * n_q
    penalty = 0.0
    if desired_force >= 0.96 * y:
        penalty += 10.0 * (desired_force / max(y, 1.0e-12) - 0.96) ** 2
        desired_force = 0.96 * y
    raw_total = desired_force / max(math.sqrt(max(1.0 - (desired_force / y) ** 2, 1.0e-9)), 1.0e-9)
    s_static = vr / math.sqrt(vr * vr + params["static_slip_knee_mps"] ** 2)
    raw_dyn = params["mu_low_speed_lateral"] * n_q * s_static
    raw_static = raw_total - raw_dyn
    if raw_static < 0.0:
        penalty += 2.0 * (raw_static / max(raw_total, 1.0e-9)) ** 2
        raw_static = 0.0
    mu_static = raw_static / max(n_q * s_static, 1.0e-12)
    return mu_static, penalty


def predict_from_arrays(arr: dict[str, object], constants: dict[str, float], x: np.ndarray) -> tuple[np.ndarray, dict[str, float]]:
    params = unpack(x)
    mu_static, static_penalty = launch_static_mu(params, constants)
    normal = arr["normal"]
    vf_rel = arr["vf_rel"]
    vr_rel = arr["vr_rel"]
    drive = arr["drive"]
    r = arr["r"]
    f = arr["f"]
    vf_abs = arr["vf_abs"]

    speed_gate = vf_abs * vf_abs / (vf_abs * vf_abs + params["speed_gate_knee_mps"] ** 2)
    low_gate = 1.0 - speed_gate

    f_drive = params["drive_scale"] * drive
    f_long_slip = params["mu_longitudinal"] * normal * sat(vf_rel, params["longitudinal_slip_knee_mps"])

    speed_floor = np.sqrt(vf_abs * vf_abs + params["long_speed_floor_mps"] ** 2)
    alpha = vr_rel / speed_floor
    f_corner = params["mu_cornering"] * normal * alpha / np.sqrt(alpha * alpha + params["alpha_knee"] ** 2)
    f_low_dyn = params["mu_low_speed_lateral"] * normal * sat(vr_rel, params["static_slip_knee_mps"])
    f_static = mu_static * normal * sat(vr_rel, params["static_slip_knee_mps"])
    f_lat_raw = low_gate * (f_static + f_low_dyn) + speed_gate * f_corner
    f_long_raw = f_drive + f_long_slip

    y = np.maximum(params["mu_peak"] * normal, 1.0e-12)
    norm = np.sqrt((f_long_raw / params["longitudinal_envelope_shape"]) ** 2 + f_lat_raw * f_lat_raw)
    scale = 1.0 / np.sqrt(1.0 + (norm / y) ** 2)
    f_long = f_long_raw * scale
    f_lat = f_lat_raw * scale
    pred = np.sum(f * f_lat - r * f_long, axis=0)
    params["mu_static_breakaway"] = mu_static
    params["static_penalty"] = static_penalty
    return pred, params


def command_from_torque(command_torque: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]
    motor_torque = command_torque / gear_ratio
    torque_sign = (motor_torque > 1.0e-9) - (motor_torque < -1.0e-9)
    wheel_sign = (wheel_speed_radps > 1.0e-9) - (wheel_speed_radps < -1.0e-9)
    no_load_sign = torque_sign or wheel_sign
    current = motor_torque / torque_constant + no_load_sign * no_load
    back_emf = wheel_speed_radps * (gear_ratio / speed_constant)
    return ((current * resistance) + back_emf) / battery


def launch_estimate(constants: dict[str, float], x: np.ndarray) -> dict[str, float | bool]:
    params = unpack(x)
    total_opposing = params["launch_static_opposing_nm"]
    half_track = 0.5 * constants["track_width_m"]
    wheel_speed = half_track / constants["wheel_radius_m"]
    drive_scale = max(params["drive_scale"], 1.0e-9)
    applied_bank_torque = total_opposing * constants["wheel_radius_m"] / (constants["track_width_m"] * drive_scale)
    rolling = constants["rolling_friction_torque_nm"]
    left = command_from_torque(applied_bank_torque + rolling, wheel_speed, constants)
    right = command_from_torque(-applied_bank_torque - rolling, -wheel_speed, constants)
    return {
        "total_opposing_yaw_torque_nm": total_opposing,
        "left_command": left,
        "right_command": right,
        "lr_delta_command": left - right,
        "max_abs_command": max(abs(left), abs(right)),
        "passes_abs_0p6_gate": max(abs(left), abs(right)) >= 0.6,
        "launch_lock_policy": "diagnostic_only",
    }


def fit_weights(frame: pd.DataFrame) -> np.ndarray:
    result = compute_regime_weights_for_frame(
        frame,
        RegimeWeightConfig(
            split_weights=LATEST_INCLUSIVE_FIT_SPLIT_WEIGHTS,
            quality=QualityConfig(
                gyro_spike_multiplier=0.15,
                saturation_multiplier=0.45,
                use_limiter_penalty=False,
                use_low_yaw_no_motion_penalty=False,
            ),
        ),
        eligible_mask=latest_inclusive_fit_mask_for_frame(frame),
    )
    return np.asarray(result.weights, dtype=float)


def objective_factory(arr: dict[str, object], constants: dict[str, float], weights: np.ndarray):
    target = arr["target"]

    def objective(x: np.ndarray) -> float:
        pred, params = predict_from_arrays(arr, constants, x)
        err = pred - target
        scale = 0.026
        robust = np.mean(weights * (2.0 * scale * scale * (np.sqrt(1.0 + (err / scale) ** 2) - 1.0)))
        rmse_term = math.sqrt(float(np.average(err * err, weights=weights)))
        sane_penalty = params["static_penalty"]
        if params["mu_static_breakaway"] > params["mu_peak"] * 1.35:
            sane_penalty += 0.02 * (params["mu_static_breakaway"] / max(params["mu_peak"], 1.0e-9) - 1.35) ** 2
        return float(robust + 0.08 * rmse_term + sane_penalty)

    return objective


def nelder_mead_bounded(
    objective,
    x0: np.ndarray,
    restart: int,
    seed_name: str,
    max_iter: int,
    trace_rows: list[dict[str, object]],
    tol: float = 1.0e-7,
) -> FitResult:
    n = len(x0)
    simplex = [np.clip(x0, 0.0, 1.0)]
    for i in range(n):
        x = np.array(x0, copy=True)
        step = 0.08 if 0.10 < x[i] < 0.90 else 0.04
        x[i] = np.clip(x[i] + step, 0.0, 1.0)
        simplex.append(x)
    values = [objective(x) for x in simplex]
    evals = len(values)
    converged = False
    for iteration in range(max_iter):
        order = np.argsort(values)
        simplex = [simplex[i] for i in order]
        values = [values[i] for i in order]
        best = values[0]
        spread = max(values) - min(values)
        diameter = max(float(np.linalg.norm(simplex[i] - simplex[0], ord=np.inf)) for i in range(1, n + 1))
        if iteration % 5 == 0 or iteration == max_iter - 1:
            trace_rows.append(
                {
                    "restart": restart,
                    "seed_name": seed_name,
                    "iteration": iteration,
                    "evaluations": evals,
                    "best_objective": best,
                    "simplex_spread": spread,
                    "simplex_diameter_inf": diameter,
                }
            )
        if spread < tol and diameter < 3.0e-4:
            converged = True
            break
        centroid = np.mean(np.vstack(simplex[:-1]), axis=0)
        worst = simplex[-1]

        def clipped(point: np.ndarray) -> np.ndarray:
            return np.clip(point, 0.0, 1.0)

        xr = clipped(centroid + (centroid - worst))
        fr = objective(xr)
        evals += 1
        if values[0] <= fr < values[-2]:
            simplex[-1] = xr
            values[-1] = fr
            continue
        if fr < values[0]:
            xe = clipped(centroid + 2.0 * (xr - centroid))
            fe = objective(xe)
            evals += 1
            simplex[-1], values[-1] = (xe, fe) if fe < fr else (xr, fr)
            continue
        xc = clipped(centroid + 0.5 * (worst - centroid))
        fc = objective(xc)
        evals += 1
        if fc < values[-1]:
            simplex[-1] = xc
            values[-1] = fc
            continue
        best_x = simplex[0]
        for i in range(1, n + 1):
            simplex[i] = clipped(best_x + 0.5 * (simplex[i] - best_x))
            values[i] = objective(simplex[i])
        evals += n
    order = np.argsort(values)
    best_i = int(order[0])
    return FitResult(restart, seed_name, converged, iteration + 1, evals, float(values[best_i]), simplex[best_i])


def coordinate_polish_bounded(
    objective,
    x0: np.ndarray,
    restart: int,
    seed_name: str,
    trace_rows: list[dict[str, object]],
    initial_step: float = 0.035,
    min_step: float = 1.5e-4,
    max_passes: int = 260,
) -> FitResult:
    x = np.clip(np.array(x0, copy=True), 0.0, 1.0)
    best = float(objective(x))
    evals = 1
    step = initial_step
    passes = 0
    while passes < max_passes and step >= min_step:
        improved = False
        for i in range(len(x)):
            for direction in (1.0, -1.0):
                trial = np.array(x, copy=True)
                trial[i] = np.clip(trial[i] + direction * step, 0.0, 1.0)
                value = float(objective(trial))
                evals += 1
                if value + 1.0e-12 < best:
                    x = trial
                    best = value
                    improved = True
                    break
            if improved:
                break
        if not improved:
            step *= 0.5
        if passes % 4 == 0 or step < min_step:
            trace_rows.append(
                {
                    "restart": restart,
                    "seed_name": seed_name,
                    "iteration": passes,
                    "evaluations": evals,
                    "best_objective": best,
                    "simplex_spread": 0.0,
                    "simplex_diameter_inf": step,
                }
            )
        passes += 1
    return FitResult(restart, seed_name, step < min_step, passes, evals, best, x)


def optimize(frame: pd.DataFrame, constants: dict[str, float]) -> tuple[FitResult, list[FitResult], list[dict[str, object]]]:
    train = frame[latest_inclusive_fit_mask_for_frame(frame)].copy()
    # Deterministic run-balanced optimizer subset, then full-data polish.
    opt = pd.concat(
        [g.sample(n=min(len(g), 1800), random_state=17) for _, g in train.groupby("run_id")],
        ignore_index=True,
    )
    arr = frame_arrays(opt, constants)
    weights = fit_weights(opt)
    objective = objective_factory(arr, constants, weights)
    seeds = [
        ("standalone_like", dict(drive_scale=0.093, mu_peak=2.2, mu_longitudinal=0.18, mu_low_speed_lateral=0.20, mu_cornering=0.80, alpha_knee=0.16, long_speed_floor_mps=0.18, speed_gate_knee_mps=0.35, static_slip_knee_mps=0.020, longitudinal_slip_knee_mps=0.16, longitudinal_envelope_shape=1.00, launch_static_opposing_nm=0.0073)),
        ("brush_high_corner", dict(drive_scale=0.105, mu_peak=2.8, mu_longitudinal=0.10, mu_low_speed_lateral=0.08, mu_cornering=1.60, alpha_knee=0.10, long_speed_floor_mps=0.25, speed_gate_knee_mps=0.55, static_slip_knee_mps=0.016, longitudinal_slip_knee_mps=0.20, longitudinal_envelope_shape=1.25, launch_static_opposing_nm=0.0084)),
        ("soft_envelope", dict(drive_scale=0.075, mu_peak=1.6, mu_longitudinal=0.35, mu_low_speed_lateral=0.35, mu_cornering=0.55, alpha_knee=0.24, long_speed_floor_mps=0.12, speed_gate_knee_mps=0.20, static_slip_knee_mps=0.030, longitudinal_slip_knee_mps=0.10, longitudinal_envelope_shape=0.80, launch_static_opposing_nm=0.0065)),
        ("fast_gate", dict(drive_scale=0.115, mu_peak=3.5, mu_longitudinal=0.05, mu_low_speed_lateral=0.12, mu_cornering=2.20, alpha_knee=0.22, long_speed_floor_mps=0.45, speed_gate_knee_mps=0.12, static_slip_knee_mps=0.012, longitudinal_slip_knee_mps=0.28, longitudinal_envelope_shape=1.60, launch_static_opposing_nm=0.0100)),
    ]
    trace_rows: list[dict[str, object]] = []
    results = [
        nelder_mead_bounded(objective, pack(values), i, name, 260, trace_rows)
        for i, (name, values) in enumerate(seeds, start=1)
    ]
    best = min(results, key=lambda r: r.objective)

    full_arr = frame_arrays(train, constants)
    full_weights = fit_weights(train)
    full_objective = objective_factory(full_arr, constants, full_weights)
    polished = nelder_mead_bounded(full_objective, best.x, 99, "full_primary_polish", 360, trace_rows, tol=6.0e-8)
    stable = coordinate_polish_bounded(full_objective, polished.x, 100, "coordinate_stability_polish", trace_rows)
    all_results = results + [polished, stable]
    return stable, all_results, trace_rows


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(values * values))) if len(values) else math.nan


def mae(values: np.ndarray) -> float:
    return float(np.mean(np.abs(values))) if len(values) else math.nan


def run_balanced_rmse(frame: pd.DataFrame, values: np.ndarray) -> float:
    if not len(frame):
        return math.nan
    counts = Counter(frame["run_id"].astype(str))
    weights = np.array([1.0 / max(counts[str(run)], 1) for run in frame["run_id"]], dtype=float)
    return float(np.sqrt(np.average(values * values, weights=weights)))


def metric_row(label: str, subset: pd.DataFrame, pred: np.ndarray) -> dict[str, object]:
    base = subset["baseline_error_nm"].to_numpy(float)
    err = pred - subset["observed_yaw_moment_nm"].to_numpy(float)
    return {
        "group": label,
        "count": int(len(subset)),
        "run_count": int(subset["run_id"].nunique()) if len(subset) else 0,
        "baseline_rmse_nm": rmse(base),
        "brush_rmse_nm": rmse(err),
        "baseline_mae_nm": mae(base),
        "brush_mae_nm": mae(err),
        "baseline_median_abs_nm": float(np.median(np.abs(base))) if len(subset) else math.nan,
        "brush_median_abs_nm": float(np.median(np.abs(err))) if len(subset) else math.nan,
        "baseline_signed_median_nm": float(np.median(base)) if len(subset) else math.nan,
        "brush_signed_median_nm": float(np.median(err)) if len(subset) else math.nan,
        "run_balanced_baseline_rmse_nm": run_balanced_rmse(subset, base),
        "run_balanced_brush_rmse_nm": run_balanced_rmse(subset, err),
        "rmse_improvement_fraction": (rmse(base) - rmse(err)) / rmse(base) if len(subset) and rmse(base) else math.nan,
    }


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def refs_by_split() -> dict[str, dict[str, str]]:
    out: dict[str, dict[str, str]] = {}
    for row in read_csv_rows(REF_VARIANT_C / "split_metrics.csv"):
        out.setdefault(row["group"], {})["variant_c_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_csv_rows(REF_FORCE_STRIBECK / "split_rmse.csv"):
        out.setdefault(row["dataset_split"], {})["force_domain_stribeck_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_csv_rows(REF_RATIONAL / "split_metrics.csv"):
        out.setdefault(row["group"], {})["rational_residual_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_csv_rows(REF_TRUE_TRACTION / "split_metrics.csv"):
        out.setdefault(row["group"], {})["true_traction_testbed_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_csv_rows(REF_STANDALONE / "split_metrics.csv"):
        out.setdefault(row["group"], {})["standalone_contact_rmse_nm"] = row.get("standalone_rmse_nm", "")
    return out


def refs_by_run() -> dict[str, dict[str, str]]:
    out: dict[str, dict[str, str]] = {}
    for row in read_csv_rows(REF_VARIANT_C / "selected_log_metrics.csv"):
        out.setdefault(row["run_id"], {})["variant_c_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_csv_rows(REF_FORCE_STRIBECK / "selected_log_rmse.csv"):
        out.setdefault(row["run_id"], {})["force_domain_stribeck_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_csv_rows(REF_RATIONAL / "selected_log_metrics.csv"):
        out.setdefault(row["run_id"], {})["rational_residual_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_csv_rows(REF_TRUE_TRACTION / "selected_log_metrics.csv"):
        out.setdefault(row["run_id"], {})["true_traction_testbed_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_csv_rows(REF_STANDALONE / "selected_log_metrics.csv"):
        out.setdefault(row["run_id"], {})["standalone_contact_rmse_nm"] = row.get("standalone_rmse_nm", "")
    return out


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def fmt(value: object) -> str:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(f):
        return ""
    return f"{f:.6f}"


def write_report(
    constants: dict[str, float],
    best: FitResult,
    params: dict[str, float],
    launch: dict[str, object],
    split_rows: list[dict[str, object]],
    selected_rows: list[dict[str, object]],
    risk_rows: list[dict[str, object]],
    common_range_rows: list[dict[str, object]],
    optimizer_rows: list[dict[str, object]],
) -> None:
    bound_hits = []
    for name, lo, hi in PARAM_BOUNDS:
        value = params[name]
        if abs(value - lo) <= 1.0e-8 or abs(value - hi) <= 1.0e-8:
            bound_hits.append(name)
    bounds_lines = ["| parameter | lower | selected | upper |", "| --- | ---: | ---: | ---: |"]
    for name, lo, hi in PARAM_BOUNDS:
        bounds_lines.append(f"| {name} | {lo:.6g} | {params[name]:.9g} | {hi:.6g} |")
    bounds_lines.append(f"| mu_static_breakaway | derived | {params['mu_static_breakaway']:.9g} | derived |")

    lines = [
        "# Brush/Pacejka-Lite Combined-Slip Contact Law",
        "",
        "Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.",
        "",
        "## Model",
        "",
        "For each contact patch `i` at position `(r_i, f_i)`, the law predicts contact forces and then accumulates yaw as:",
        "",
        "`M_yaw = sum_i(f_i * F_r,i - r_i * F_f,i)`",
        "",
        "Longitudinal raw demand:",
        "",
        "`F_f,raw = drive_scale * F_drive_side/2 + mu_longitudinal * N_i * v_f / sqrt(v_f^2 + s_long^2)`",
        "",
        "Low-speed yaw/Stribeck branch:",
        "",
        "`F_r,low = (mu_static_breakaway + mu_low_speed_lateral) * N_i * v_r / sqrt(v_r^2 + s_static^2)`",
        "",
        "High-speed slip-angle branch:",
        "",
        "`alpha_proxy = v_r / sqrt(|Vf|^2 + v_floor^2)`",
        "",
        "`F_r,corner = mu_cornering * N_i * alpha_proxy / sqrt(alpha_proxy^2 + alpha_knee^2)`",
        "",
        "Rational speed transition:",
        "",
        "`g = Vf^2 / (Vf^2 + speed_gate_knee^2)` and `F_r,raw = (1-g)*F_r,low + g*F_r,corner`",
        "",
        "Combined-slip envelope:",
        "",
        "`Y_i = mu_peak * N_i`",
        "",
        "`scale_i = 1 / sqrt(1 + (((F_f,raw / long_shape)^2 + F_r,raw^2) / Y_i^2))`",
        "",
        "`F_f,i = scale_i * F_f,raw`, `F_r,i = scale_i * F_r,raw`",
        "",
        "`mu_static_breakaway` is analytically derived for each optimizer step from the fitted free `launch_static_opposing_nm` parameter. That parameter is optimized only through the data objective; the resulting `Vf=0`, `yawRate=1 rad/s` command is diagnostic.",
        "",
        "## Selected Parameters",
        "",
        *bounds_lines,
        "",
        "## Launch Estimate",
        "",
        "| total opposing Nm | left command | right command | max abs command | pass | launch lock policy |",
        "| ---: | ---: | ---: | ---: | --- | --- |",
        f"| {float(launch['total_opposing_yaw_torque_nm']):.6f} | {float(launch['left_command']):.6f} | {float(launch['right_command']):.6f} | {float(launch['max_abs_command']):.6f} | {launch['passes_abs_0p6_gate']} | {launch['launch_lock_policy']} |",
        "",
        "## Convergence",
        "",
        f"Selected result: restart `{best.restart}` / `{best.seed_name}`, objective `{best.objective:.9f}`, iterations `{best.iterations}`, evaluations `{best.evaluations}`, converged `{best.converged}`.",
        "",
        "| restart | seed | objective | iterations | evaluations | converged |",
        "| ---: | --- | ---: | ---: | ---: | --- |",
    ]
    for row in optimizer_rows:
        lines.append(
            f"| {row['restart']} | {row['seed_name']} | {float(row['objective']):.9f} | {row['iterations']} | {row['evaluations']} | {row['converged']} |"
        )
    lines.extend(["", "## Split Metrics", "", "| split | count | baseline RMSE | brush RMSE | improvement | force-domain Stribeck | rational residual | standalone contact | true-traction testbed | Variant C |", "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"])
    for row in split_rows:
        lines.append(
            f"| {row['group']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['brush_rmse_nm'])} | {fmt(row['rmse_improvement_fraction'])} | "
            f"{row.get('force_domain_stribeck_rmse_nm', '')} | {row.get('rational_residual_rmse_nm', '')} | {row.get('standalone_contact_rmse_nm', '')} | {row.get('true_traction_testbed_rmse_nm', '')} | {row.get('variant_c_rmse_nm', '')} |"
        )
    lines.extend(["", "## Selected Logs", "", "| run | split | count | baseline RMSE | brush RMSE | force-domain | rational | standalone | true-traction | Variant C |", "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"])
    for row in selected_rows:
        if row.get("present") is False:
            lines.append(f"| {row['run_id']} | absent | 0 |  |  |  |  |  |  |  |")
            continue
        lines.append(
            f"| {row['run_id']} | {row['dataset_split']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['brush_rmse_nm'])} | "
            f"{row.get('force_domain_stribeck_rmse_nm', '')} | {row.get('rational_residual_rmse_nm', '')} | {row.get('standalone_contact_rmse_nm', '')} | {row.get('true_traction_testbed_rmse_nm', '')} | {row.get('variant_c_rmse_nm', '')} |"
        )
    lines.extend(["", "## Risk Slices", "", "| slice | count | baseline RMSE | brush RMSE | improvement |", "| --- | ---: | ---: | ---: | ---: |"])
    for row in risk_rows:
        lines.append(f"| {row['group']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['brush_rmse_nm'])} | {fmt(row['rmse_improvement_fraction'])} |")
    lines.extend(
        [
            "",
            "## Common Range Metrics",
            "",
            "These rows use the shared operating-range definitions in `common_range_metrics.csv`; `0.7 m/s` is reported as pre-design turn speed, not high speed.",
            "",
        ]
    )
    lines.extend(
        [
            "| range | count | baseline RMSE | candidate RMSE | candidate MAE | candidate median abs |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in common_range_rows:
        lines.append(
            f"| {row['range_name']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['candidate_rmse_nm'])} | {fmt(row['candidate_mae_nm'])} | {fmt(row['candidate_median_abs_nm'])} |"
        )
    sane = (
        params["drive_scale"] > 0.0
        and params["mu_peak"] >= params["mu_static_breakaway"] * 0.65
        and params["speed_gate_knee_mps"] > params["static_slip_knee_mps"]
    )
    sanity_label = "qualified_true" if sane and bound_hits else str(sane)
    lines.extend(
        [
            "",
            "## Assessment",
            "",
            f"Physical sanity: `{sanity_label}`. The selected equation shape is continuous, odd in local slip velocity, uses a rational zero-to-speed gate, and forces longitudinal/lateral patch demands to compete inside a smooth yield envelope. It does not use UKF state-vector fields, command/request/preprojection traction selectors, or an old-plus-residual runtime branch.",
            "",
            f"Coefficient sanity caveat: `{', '.join(bound_hits) if bound_hits else 'none'}` hit optimizer bounds. Treat the family as viable for comparison, but not as an identified production calibration without tighter targeted data or a narrower prior on the envelope/speed-floor terms.",
            "",
            "The tradeoff is that the direct brush law is more physically shaped than the rational residual reference, but it is still fitted from noisy yaw-acceleration-derived moments. Validation and operating-range slices are reported, not imposed as hard constraints.",
            "",
            "## Outputs",
            "",
            "- `fit_brush_combined_slip.py`",
            "- `brush_combined_slip_report.md`",
            "- `selected_parameters.csv`",
            "- `parameter_bound_hits.csv`",
            "- `optimizer_summary.csv`",
            "- `optimizer_trace.csv`",
            "- `split_metrics.csv`",
            "- `selected_log_metrics.csv`",
            "- `may4_latest_metrics.csv`",
            "- `risk_slices.csv`",
            "- `common_range_metrics.csv`",
            "- `launch_command_estimate.csv`",
            "- `prediction_sample.csv`",
            "- `commands_run.txt`",
        ]
    )
    (OUT / "brush_combined_slip_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    constants = read_constants()
    frame = load_frame(constants)
    write_regime_diagnostics(
        OUT,
        "",
        compute_regime_weights_for_frame(
            frame,
            RegimeWeightConfig(
                split_weights=LATEST_INCLUSIVE_FIT_SPLIT_WEIGHTS,
                quality=QualityConfig(
                    gyro_spike_multiplier=0.15,
                    saturation_multiplier=0.45,
                    use_limiter_penalty=False,
                    use_low_yaw_no_motion_penalty=False,
                ),
            ),
            eligible_mask=latest_inclusive_fit_mask_for_frame(frame),
        ),
    )
    best, all_results, trace_rows = optimize(frame, constants)
    arr = frame_arrays(frame, constants)
    pred, params = predict_from_arrays(arr, constants, best.x)
    launch = launch_estimate(constants, best.x)
    common_range_rows = write_common_range_metrics(
        OUT / "common_range_metrics.csv",
        frame,
        frame["baseline_error_nm"].to_numpy(float),
        pred - frame["observed_yaw_moment_nm"].to_numpy(float),
        "brush_combined_slip",
    )

    split_refs = refs_by_split()
    split_rows = []
    for split in SPLITS:
        subset = frame[frame["dataset_split"] == split]
        row = metric_row(split, subset, pred[subset.index.to_numpy()])
        row.update(split_refs.get(split, {}))
        split_rows.append(row)
    validation = frame[frame["dataset_split"] != "primary_open_floor_fit_authoritative"]
    row = metric_row("validation_non_authoritative", validation, pred[validation.index.to_numpy()])
    row.update(split_refs.get("validation_non_authoritative", {}))
    split_rows.append(row)

    run_refs = refs_by_run()
    selected_rows = []
    for run_id in SELECTED_RUNS:
        subset = frame[frame["run_id"].astype(str) == run_id]
        if subset.empty:
            selected_rows.append({"run_id": run_id, "present": False})
            continue
        row = metric_row(run_id, subset, pred[subset.index.to_numpy()])
        row["run_id"] = run_id
        row["dataset_split"] = str(subset["dataset_split"].iloc[0])
        row.update(run_refs.get(run_id, {}))
        selected_rows.append(row)

    risk_defs = {
        "calibration_low_vf_nonzero_yaw": (frame["abs_forward_velocity_mps"] < 0.15) & (frame["abs_yaw_rate_radps"] >= 0.1),
        "in_place_scrub": (frame["abs_forward_velocity_mps"] < 0.05) & (frame["abs_yaw_rate_radps"] >= 0.2),
        "slow_forward_turn": (frame["abs_forward_velocity_mps"] >= 0.15) & (frame["abs_forward_velocity_mps"] < 0.70) & (frame["abs_yaw_rate_radps"] >= 0.1),
        "pre_design_turn_speed": (frame["abs_forward_velocity_mps"] >= 0.70) & (frame["abs_forward_velocity_mps"] < 0.95) & (frame["abs_yaw_rate_radps"] >= 0.1),
        "design_turn_speed_and_up": (frame["abs_forward_velocity_mps"] >= 0.95) & (frame["abs_yaw_rate_radps"] >= 0.1),
        "fast_forward": frame["abs_forward_velocity_mps"] >= 1.50,
        "straightish_forward": (frame["abs_yaw_rate_radps"] < 0.05) & (frame["abs_forward_velocity_mps"] >= 0.05),
        "limiter_active": frame["max_force_limiter_activity"] > 0.0,
        "hardware_saturation_evidence": frame["hardware_saturation_evidence"] > 0.0,
        "may4_latest_logs": frame["run_id"].astype(str).isin(["2026-05-04_20-35-47", "2026-05-04_16-57-53"]),
    }
    risk_rows = []
    for label, mask in risk_defs.items():
        subset = frame[mask]
        risk_rows.append(metric_row(label, subset, pred[subset.index.to_numpy()]))

    optimizer_rows = []
    for result in all_results:
        optimizer_rows.append(
            {
                "restart": result.restart,
                "seed_name": result.seed_name,
                "objective": result.objective,
                "iterations": result.iterations,
                "evaluations": result.evaluations,
                "converged": result.converged,
                **{name: unpack(result.x)[name] for name, _, _ in PARAM_BOUNDS},
            }
        )

    parameter_rows = [{"parameter": name, "value": params[name], "lower_bound": lo, "upper_bound": hi} for name, lo, hi in PARAM_BOUNDS]
    parameter_rows.append({"parameter": "mu_static_breakaway", "value": params["mu_static_breakaway"], "lower_bound": "derived", "upper_bound": "derived"})
    parameter_rows.append({"parameter": "track_width_m", "value": constants["track_width_m"], "lower_bound": "constant", "upper_bound": "constant"})
    parameter_rows.append({"parameter": "drive_wheel_longitudinal_offset_m", "value": constants["drive_wheel_longitudinal_offset_m"], "lower_bound": "constant", "upper_bound": "constant"})
    bound_hit_rows = []
    for name, lo, hi in PARAM_BOUNDS:
        value = params[name]
        if abs(value - lo) <= 1.0e-8 or abs(value - hi) <= 1.0e-8:
            bound_hit_rows.append(
                {
                    "parameter": name,
                    "value": value,
                    "hit": "lower" if abs(value - lo) <= 1.0e-8 else "upper",
                    "lower_bound": lo,
                    "upper_bound": hi,
                }
            )

    sample = frame[["run_id", "dataset_split", "physics_phase", "row_index", "forward_velocity_mps", "yaw_rate_radps", "observed_yaw_moment_nm", "model_yaw_moment_nm"]].copy()
    sample["brush_predicted_yaw_moment_nm"] = pred
    sample["baseline_error_nm"] = frame["baseline_error_nm"].to_numpy(float)
    sample["brush_error_nm"] = pred - frame["observed_yaw_moment_nm"].to_numpy(float)
    sample = sample.iloc[np.linspace(0, len(sample) - 1, min(1200, len(sample))).astype(int)]

    write_csv(OUT / "selected_parameters.csv", parameter_rows)
    write_csv(OUT / "parameter_bound_hits.csv", bound_hit_rows)
    write_csv(OUT / "optimizer_summary.csv", optimizer_rows)
    write_csv(OUT / "optimizer_trace.csv", trace_rows)
    write_csv(OUT / "split_metrics.csv", split_rows)
    write_csv(OUT / "selected_log_metrics.csv", selected_rows)
    write_csv(OUT / "may4_latest_metrics.csv", [r for r in selected_rows if str(r.get("run_id", "")).startswith("2026-05-04")])
    write_csv(OUT / "risk_slices.csv", risk_rows)
    write_csv(OUT / "launch_command_estimate.csv", [launch])
    sample.to_csv(OUT / "prediction_sample.csv", index=False)
    (OUT / "commands_run.txt").write_text(
        f"{sys.executable} {Path(__file__).as_posix()}\n",
        encoding="utf-8",
    )
    (OUT / "metadata.json").write_text(
        json.dumps({"rows": int(len(frame)), "runs": int(frame['run_id'].nunique())}, indent=2),
        encoding="utf-8",
    )
    write_report(
        constants,
        best,
        params,
        launch,
        split_rows,
        selected_rows,
        risk_rows,
        common_range_rows,
        optimizer_rows,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
