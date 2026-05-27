"""Memoryless bristle/Stribeck-to-slip yaw traction fit.

Analysis-only script. It reads the shared yaw/contact feature artifacts and
writes outputs only beside this script.
"""

from __future__ import annotations

import csv
import math
import random
import sys
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
    COMMON_RANGE_REPORT_COLUMNS,
    write_common_range_metrics,
)

PRIMARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "ablation" / "phase_classified_feature_sample.csv"
SECONDARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "contact_continuum_feature_sample.csv"
CONSTANTS = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "plant_mirror_constants.csv"

VARIANT_C_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "split_metrics.csv"
VARIANT_C_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "selected_log_metrics.csv"
FORCE_DOMAIN_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck" / "split_rmse.csv"
FORCE_DOMAIN_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck" / "selected_log_rmse.csv"
RATIONAL_RESIDUAL_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "transition_options" / "rational_speed_force_blend" / "split_metrics.csv"
RATIONAL_RESIDUAL_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "transition_options" / "rational_speed_force_blend" / "selected_log_metrics.csv"
STANDALONE_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "standalone_contact_traction_testbed" / "split_metrics.csv"
STANDALONE_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "standalone_contact_traction_testbed" / "selected_log_metrics.csv"
TRUE_TRACTION_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "contact_patch_true_traction_testbed" / "split_metrics.csv"
TRUE_TRACTION_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "contact_patch_true_traction_testbed" / "selected_log_metrics.csv"

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
    "dataset_split",
    "row_index",
    "time_us",
    "physics_phase",
    "forward_velocity_mps",
    "yaw_rate_radps",
    "measured_yaw_accel_radps2",
    "vbar_yaw_mps",
    "limiter_active",
    "hardware_saturation_evidence",
    "gyro_derivative_spike",
]

SECONDARY_COLUMNS = [
    "run_id",
    "row_index",
    "left_encoder_velocity_mps",
    "right_encoder_velocity_mps",
    "left_drive_force_n",
    "right_drive_force_n",
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
    "observed_yaw_moment_nm",
    "model_yaw_moment_nm",
    "residual_additive_yaw_torque_nm",
]

PARAM_NAMES = [
    "drive_scale",
    "longitudinal_mu",
    "longitudinal_k_mps",
    "stribeck_slide_mu",
    "stribeck_static_extra_mu",
    "stribeck_speed_mps",
    "transition_vf_mps",
    "alpha_stiff_base",
    "alpha_stiff_speed_gain",
    "alpha_stiff_speed_k_mps",
    "alpha_saturation_mu",
    "alpha_floor_mps",
]

BOUNDS = np.array(
    [
        [0.050, 0.150],
        [0.000, 0.700],
        [0.005, 0.600],
        [0.000, 0.400],
        [0.020, 0.700],
        [0.004, 0.300],
        [0.020, 0.900],
        [0.000, 8.000],
        [0.000, 16.000],
        [0.030, 2.000],
        [0.050, 2.200],
        [0.015, 0.600],
    ],
    dtype=float,
)


@dataclass
class FitResult:
    label: str
    params: np.ndarray
    objective: float
    train_rmse: float
    train_robust: float
    launch_command: float
    launch_opposing_nm: float
    iterations: int
    evaluations: int
    converged: bool
    boundary_hits: int
    collapse_notes: str


def read_constants() -> dict[str, float]:
    table = pd.read_csv(CONSTANTS)
    return {str(row.name): float(row.value) for row in table.itertuples(index=False)}


def load_frame() -> pd.DataFrame:
    primary = pd.read_csv(PRIMARY, usecols=PRIMARY_COLUMNS)
    secondary = pd.read_csv(SECONDARY, usecols=SECONDARY_COLUMNS)
    frame = primary.merge(secondary, how="inner", on=["run_id", "row_index"], suffixes=("", "_secondary"))
    frame = frame[frame["dataset_split"] != "excluded_or_unclassified"].copy()
    for col in frame.columns:
        if col not in ("run_id", "family", "dataset_split", "physics_phase"):
            frame[col] = pd.to_numeric(frame[col], errors="coerce")
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(
        subset=["observed_yaw_moment_nm", "model_yaw_moment_nm"]
    )
    return add_forward_accel_columns_to_frame(frame.reset_index(drop=True))


def contact_geometry(constants: dict[str, float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    half_track = 0.5 * constants["track_width_m"]
    offset = constants["drive_wheel_longitudinal_offset_m"]
    r = np.array([-half_track, half_track, -half_track, half_track], dtype=float)
    f = np.array([offset, offset, -offset, -offset], dtype=float)
    side = np.array([0, 1, 0, 1], dtype=int)
    return r, f, side


def matrix(frame: pd.DataFrame, constants: dict[str, float]) -> dict[str, np.ndarray]:
    r, f, side = contact_geometry(constants)
    n = np.vstack(
        [
            frame["fl_normal_n"].to_numpy(float),
            frame["fr_normal_n"].to_numpy(float),
            frame["rl_normal_n"].to_numpy(float),
            frame["rr_normal_n"].to_numpy(float),
        ]
    ).T
    vf_logged = np.vstack(
        [
            frame["fl_v_rel_f_mps"].to_numpy(float),
            frame["fr_v_rel_f_mps"].to_numpy(float),
            frame["rl_v_rel_f_mps"].to_numpy(float),
            frame["rr_v_rel_f_mps"].to_numpy(float),
        ]
    ).T
    vr = np.vstack(
        [
            frame["fl_v_rel_r_mps"].to_numpy(float),
            frame["fr_v_rel_r_mps"].to_numpy(float),
            frame["rl_v_rel_r_mps"].to_numpy(float),
            frame["rr_v_rel_r_mps"].to_numpy(float),
        ]
    ).T
    forward = frame["forward_velocity_mps"].to_numpy(float)
    yaw = frame["yaw_rate_radps"].to_numpy(float)
    left_encoder = frame["left_encoder_velocity_mps"].to_numpy(float)
    right_encoder = frame["right_encoder_velocity_mps"].to_numpy(float)
    wheel_surface = np.where(side[None, :] == 0, left_encoder[:, None], right_encoder[:, None])
    vf_recomputed = wheel_surface - (forward[:, None] - yaw[:, None] * r[None, :])
    vf = np.where(np.isfinite(wheel_surface), vf_recomputed, vf_logged)
    left_drive = frame["left_drive_force_n"].to_numpy(float)
    right_drive = frame["right_drive_force_n"].to_numpy(float)
    drive = np.where(side[None, :] == 0, left_drive[:, None], right_drive[:, None])
    return {
        "r": r,
        "f": f,
        "normal": n,
        "vf": vf,
        "vr": vr,
        "forward": forward,
        "yaw": yaw,
        "drive": drive,
        "observed": frame["observed_yaw_moment_nm"].to_numpy(float),
        "baseline": frame["model_yaw_moment_nm"].to_numpy(float),
        "gyro_spike": frame["gyro_derivative_spike"].fillna(0.0).to_numpy(float),
        "saturation": frame["hardware_saturation_evidence"].fillna(0.0).to_numpy(float),
        "limiter": frame["limiter_active"].fillna(0.0).to_numpy(float),
        "run_id": frame["run_id"].to_numpy(str),
    }


def sign_eps(values: np.ndarray, eps: float = 1.0e-9) -> np.ndarray:
    return np.where(values > eps, 1.0, np.where(values < -eps, -1.0, 0.0))


def predict_from_matrix(data: dict[str, np.ndarray], params: np.ndarray) -> np.ndarray:
    (
        drive_scale,
        longitudinal_mu,
        longitudinal_k,
        stribeck_slide_mu,
        stribeck_static_extra_mu,
        stribeck_speed,
        transition_vf,
        alpha_stiff_base,
        alpha_stiff_speed_gain,
        alpha_stiff_speed_k,
        alpha_saturation_mu,
        alpha_floor,
    ) = params

    r = data["r"][None, :]
    f = data["f"][None, :]
    n = data["normal"]
    vf = data["vf"]
    vr = data["vr"]
    drive = data["drive"]
    forward_abs = np.abs(data["forward"])[:, None]

    contact_speed = np.sqrt(vf * vf + vr * vr)
    g_stribeck = 1.0 / (1.0 + (forward_abs / max(transition_vf, 1.0e-9)) ** 2)

    mu_stribeck = stribeck_slide_mu + stribeck_static_extra_mu / (
        1.0 + (contact_speed / max(stribeck_speed, 1.0e-9)) ** 2
    )
    force_r_stribeck = n * mu_stribeck * sign_eps(vr)

    alpha_proxy = vr / (forward_abs + alpha_floor)
    speed_stiff_gate = forward_abs / (forward_abs + max(alpha_stiff_speed_k, 1.0e-9))
    alpha_stiff = alpha_stiff_base + alpha_stiff_speed_gain * speed_stiff_gate
    alpha_raw_mu = alpha_stiff * alpha_proxy
    force_r_alpha = n * alpha_raw_mu / (1.0 + np.abs(alpha_raw_mu) / max(alpha_saturation_mu, 1.0e-9))

    force_r = g_stribeck * force_r_stribeck + (1.0 - g_stribeck) * force_r_alpha

    force_f_drive = 0.5 * drive_scale * drive
    long_slip = vf / (np.abs(vf) + max(longitudinal_k, 1.0e-9))
    force_f_dynamic = n * longitudinal_mu * long_slip
    force_f = force_f_drive + force_f_dynamic
    return np.sum(f * force_r - r * force_f, axis=1)


def fit_weights(frame: pd.DataFrame) -> np.ndarray:
    result = compute_regime_weights_for_frame(
        frame,
        RegimeWeightConfig(
            split_weights=LATEST_INCLUSIVE_FIT_SPLIT_WEIGHTS,
            quality=QualityConfig(
                gyro_spike_multiplier=0.10,
                saturation_multiplier=0.35,
                use_limiter_penalty=False,
                use_low_yaw_no_motion_penalty=False,
            ),
        ),
        eligible_mask=latest_inclusive_fit_mask_for_frame(frame),
    )
    return np.asarray(result.weights, dtype=float)


def run_balanced_weights(run_ids: np.ndarray) -> np.ndarray:
    values, counts = np.unique(run_ids, return_counts=True)
    lookup = {run: count for run, count in zip(values, counts)}
    return np.array([1.0 / lookup[run] for run in run_ids], dtype=float)


def rmse(errors: np.ndarray) -> float:
    return float(math.sqrt(np.mean(errors * errors))) if len(errors) else math.nan


def mae(errors: np.ndarray) -> float:
    return float(np.mean(np.abs(errors))) if len(errors) else math.nan


def weighted_rmse(errors: np.ndarray, weights: np.ndarray) -> float:
    total = float(np.sum(weights))
    return float(math.sqrt(np.sum(weights * errors * errors) / total)) if total > 0.0 else math.nan


def robust_loss(errors: np.ndarray, weights: np.ndarray, delta: float = 0.035) -> float:
    scaled = errors / delta
    loss = delta * delta * (np.sqrt(1.0 + scaled * scaled) - 1.0)
    total = float(np.sum(weights))
    return float(np.sum(weights * loss) / total) if total > 0.0 else 1.0e9


def logit_from_params(params: np.ndarray) -> np.ndarray:
    lo = BOUNDS[:, 0]
    hi = BOUNDS[:, 1]
    u = np.clip((params - lo) / (hi - lo), 1.0e-6, 1.0 - 1.0e-6)
    return np.log(u / (1.0 - u))


def params_from_logit(x: np.ndarray) -> np.ndarray:
    u = 1.0 / (1.0 + np.exp(-np.clip(x, -50.0, 50.0)))
    return BOUNDS[:, 0] + u * (BOUNDS[:, 1] - BOUNDS[:, 0])


def sign_scalar(value: float, eps: float = 1.0e-9) -> float:
    if value > eps:
        return 1.0
    if value < -eps:
        return -1.0
    return 0.0


def command_from_torque(command_torque: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]

    motor_torque = command_torque / gear_ratio
    torque_sign = sign_scalar(motor_torque)
    wheel_sign = sign_scalar(wheel_speed_radps)
    no_load_sign = torque_sign if torque_sign else wheel_sign
    current = (motor_torque / torque_constant) + no_load_sign * no_load
    back_emf = wheel_speed_radps * (gear_ratio / speed_constant)
    return ((current * resistance) + back_emf) / battery


def launch_estimate(constants: dict[str, float], params: np.ndarray) -> dict[str, float | str | bool]:
    total_normal = constants["mass_kg"] * 9.80665 + constants.get("fan_downforce_full_duty_n", 0.0) * 0.8
    r, f, _side = contact_geometry(constants)
    vf = np.zeros((1, 4), dtype=float)
    vr = np.array(
        [[
            -constants["drive_wheel_longitudinal_offset_m"],
            -constants["drive_wheel_longitudinal_offset_m"],
            constants["drive_wheel_longitudinal_offset_m"],
            constants["drive_wheel_longitudinal_offset_m"],
        ]],
        dtype=float,
    )
    synthetic = {
        "r": r,
        "f": f,
        "normal": np.full((1, 4), 0.25 * total_normal, dtype=float),
        "vf": vf,
        "vr": vr,
        "forward": np.array([0.0], dtype=float),
        "yaw": np.array([1.0], dtype=float),
        "drive": np.zeros((1, 4), dtype=float),
    }
    drive_free_moment = float(predict_from_matrix(synthetic, params)[0])
    opposing = max(-drive_free_moment, 0.0)
    drive_scale = float(params[0])
    half_track = 0.5 * constants["track_width_m"]
    wheel_speed = half_track / constants["wheel_radius_m"]
    applied_bank_torque = opposing * constants["wheel_radius_m"] / (constants["track_width_m"] * max(drive_scale, 1.0e-12))
    rolling = constants["rolling_friction_torque_nm"]
    left = command_from_torque(applied_bank_torque + rolling, wheel_speed, constants)
    right = command_from_torque(-applied_bank_torque - rolling, -wheel_speed, constants)
    max_abs = max(abs(left), abs(right))
    return {
        "scenario": "Vf0_Vr0_yawRate_pos1",
        "opposing_bristle_scrub_nm": opposing,
        "left_command": left,
        "right_command": right,
        "max_abs_command": max_abs,
        "gate_pass": bool(max_abs >= 0.6),
        "launch_lock_policy": "diagnostic_only",
    }


def objective_factory(data: dict[str, np.ndarray], weights: np.ndarray, constants: dict[str, float]):
    observed = data["observed"]
    eval_count = {"count": 0}

    def objective(x: np.ndarray) -> float:
        eval_count["count"] += 1
        params = params_from_logit(x)
        pred = predict_from_matrix(data, params)
        errors = pred - observed
        loss = robust_loss(errors, weights)
        penalty = 0.0
        if params[4] < 0.02:
            penalty += 0.01
        if params[8] < 0.02 and params[7] < 0.02:
            penalty += 0.01
        return loss + penalty

    return objective, eval_count


def nelder_mead(
    objective,
    x0: np.ndarray,
    step: np.ndarray,
    max_iter: int,
    tol: float,
    label: str,
    trace_rows: list[dict[str, object]],
) -> tuple[np.ndarray, float, int, bool]:
    n = len(x0)
    simplex = [x0.copy()]
    for i in range(n):
        point = x0.copy()
        point[i] += step[i]
        simplex.append(point)
    values = [objective(point) for point in simplex]
    converged = False
    iterations = 0
    for iteration in range(max_iter):
        order = np.argsort(values)
        simplex = [simplex[i] for i in order]
        values = [values[i] for i in order]
        iterations = iteration + 1
        if iteration % 25 == 0 or iteration == max_iter - 1:
            trace_rows.append(
                {
                    "label": label,
                    "iteration": iteration,
                    "best_objective": values[0],
                    "worst_objective": values[-1],
                    "simplex_spread": float(np.max(np.std(np.vstack(simplex), axis=0))),
                }
            )
        spread = float(np.std(values))
        geom = float(np.max(np.std(np.vstack(simplex), axis=0)))
        if spread < tol and geom < tol:
            converged = True
            break

        centroid = np.mean(np.vstack(simplex[:-1]), axis=0)
        worst = simplex[-1]
        xr = centroid + (centroid - worst)
        fr = objective(xr)
        if values[0] <= fr < values[-2]:
            simplex[-1] = xr
            values[-1] = fr
            continue
        if fr < values[0]:
            xe = centroid + 2.0 * (xr - centroid)
            fe = objective(xe)
            if fe < fr:
                simplex[-1] = xe
                values[-1] = fe
            else:
                simplex[-1] = xr
                values[-1] = fr
            continue
        xc = centroid + 0.5 * (worst - centroid)
        fc = objective(xc)
        if fc < values[-1]:
            simplex[-1] = xc
            values[-1] = fc
            continue
        best = simplex[0]
        for i in range(1, n + 1):
            simplex[i] = best + 0.5 * (simplex[i] - best)
            values[i] = objective(simplex[i])
    order = np.argsort(values)
    return simplex[int(order[0])], float(values[int(order[0])]), iterations, converged


def seed_params() -> list[tuple[str, np.ndarray]]:
    base = np.array([0.096, 0.18, 0.16, 0.05, 0.22, 0.080, 0.160, 0.8, 2.8, 0.45, 0.85, 0.12], dtype=float)
    seeds = [("standalone_static_yield_inspired", base)]
    seeds.append(("force_domain_launch_heavier", np.array([0.092, 0.10, 0.12, 0.08, 0.32, 0.045, 0.090, 0.5, 1.5, 0.35, 0.75, 0.08])))
    seeds.append(("slip_angle_forward_biased", np.array([0.105, 0.22, 0.22, 0.02, 0.18, 0.060, 0.260, 1.6, 5.0, 0.55, 1.10, 0.18])))
    rng = random.Random(42025)
    for index in range(5):
        u = np.array([rng.random() for _ in PARAM_NAMES])
        params = BOUNDS[:, 0] + u * (BOUNDS[:, 1] - BOUNDS[:, 0])
        params[0] = 0.075 + rng.random() * 0.050
        params[4] = 0.12 + rng.random() * 0.35
        params[6] = 0.05 + rng.random() * 0.35
        seeds.append((f"deterministic_random_seed_{index}", params))
    return seeds


def fit(frame: pd.DataFrame, constants: dict[str, float]) -> tuple[FitResult, list[FitResult], list[dict[str, object]]]:
    train_frame = frame[latest_inclusive_fit_mask_for_frame(frame)].copy()
    data = matrix(train_frame, constants)
    weights = fit_weights(train_frame)
    objective, eval_counter = objective_factory(data, weights, constants)
    trace_rows: list[dict[str, object]] = []
    results: list[FitResult] = []
    starts = seed_params()
    for label, params0 in starts:
        x0 = logit_from_params(params0)
        step = np.full(len(PARAM_NAMES), 0.42, dtype=float)
        x_best, value, iterations, converged = nelder_mead(
            objective,
            x0,
            step,
            max_iter=520,
            tol=2.0e-7,
            label=label,
            trace_rows=trace_rows,
        )
        params = params_from_logit(x_best)
        pred = predict_from_matrix(data, params)
        errors = pred - data["observed"]
        launch = launch_estimate(constants, params)
        near_bounds = np.logical_or(params <= BOUNDS[:, 0] + 0.02 * (BOUNDS[:, 1] - BOUNDS[:, 0]), params >= BOUNDS[:, 1] - 0.02 * (BOUNDS[:, 1] - BOUNDS[:, 0]))
        notes = []
        if params[4] <= BOUNDS[4, 0] + 0.02 * (BOUNDS[4, 1] - BOUNDS[4, 0]):
            notes.append("static_extra_collapsed")
        if params[7] <= BOUNDS[7, 0] + 0.02 * (BOUNDS[7, 1] - BOUNDS[7, 0]) and params[8] <= BOUNDS[8, 0] + 0.02 * (BOUNDS[8, 1] - BOUNDS[8, 0]):
            notes.append("alpha_stiffness_collapsed")
        if params[6] <= BOUNDS[6, 0] + 0.02 * (BOUNDS[6, 1] - BOUNDS[6, 0]):
            notes.append("transition_at_lower_bound")
        results.append(
            FitResult(
                label=label,
                params=params,
                objective=value,
                train_rmse=rmse(errors),
                train_robust=robust_loss(errors, weights),
                launch_command=float(launch["max_abs_command"]),
                launch_opposing_nm=float(launch["opposing_bristle_scrub_nm"]),
                iterations=iterations,
                evaluations=eval_counter["count"],
                converged=converged,
                boundary_hits=int(np.sum(near_bounds)),
                collapse_notes=";".join(notes) if notes else "none",
            )
        )
    selected = min(results, key=lambda r: r.objective)
    return selected, sorted(results, key=lambda r: r.objective), trace_rows


def metric_row(name: str, subset: pd.DataFrame, constants: dict[str, float], params: np.ndarray) -> dict[str, object]:
    if subset.empty:
        return {"group": name, "present": False, "count": 0}
    data = matrix(subset, constants)
    pred = predict_from_matrix(data, params)
    model_errors = pred - data["observed"]
    baseline_errors = data["baseline"] - data["observed"]
    rbw = run_balanced_weights(data["run_id"])
    return {
        "group": name,
        "present": True,
        "count": int(len(subset)),
        "run_count": int(subset["run_id"].nunique()),
        "baseline_rmse_nm": rmse(baseline_errors),
        "bristle_slip_rmse_nm": rmse(model_errors),
        "baseline_mae_nm": mae(baseline_errors),
        "bristle_slip_mae_nm": mae(model_errors),
        "baseline_median_abs_nm": float(np.median(np.abs(baseline_errors))),
        "bristle_slip_median_abs_nm": float(np.median(np.abs(model_errors))),
        "baseline_signed_median_nm": float(np.median(baseline_errors)),
        "bristle_slip_signed_median_nm": float(np.median(model_errors)),
        "run_balanced_baseline_rmse_nm": weighted_rmse(baseline_errors, rbw),
        "run_balanced_bristle_slip_rmse_nm": weighted_rmse(model_errors, rbw),
        "rmse_improvement_fraction": (rmse(baseline_errors) - rmse(model_errors)) / rmse(baseline_errors) if rmse(baseline_errors) > 0.0 else 0.0,
    }


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def reference_by_split() -> dict[str, dict[str, str]]:
    refs: dict[str, dict[str, str]] = {}
    for row in read_rows(VARIANT_C_SPLIT):
        refs.setdefault(row.get("group", row.get("dataset_split", "")), {})["variant_c_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(FORCE_DOMAIN_SPLIT):
        refs.setdefault(row.get("dataset_split", row.get("group", "")), {})["force_domain_stribeck_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(RATIONAL_RESIDUAL_SPLIT):
        refs.setdefault(row.get("dataset_split", row.get("group", "")), {})["rational_residual_reference_rmse_nm"] = row.get("candidate_rmse_nm", row.get("corrected_rmse_nm", ""))
    for row in read_rows(STANDALONE_SPLIT):
        refs.setdefault(row.get("group", row.get("dataset_split", "")), {})["standalone_contact_traction_rmse_nm"] = row.get("standalone_rmse_nm", row.get("corrected_rmse_nm", ""))
    for row in read_rows(TRUE_TRACTION_SPLIT):
        refs.setdefault(row.get("group", row.get("dataset_split", "")), {})["force_level_contact_testbed_rmse_nm"] = row.get("corrected_rmse_nm", "")
    return refs


def reference_by_run() -> dict[str, dict[str, str]]:
    refs: dict[str, dict[str, str]] = {}
    for row in read_rows(VARIANT_C_SELECTED):
        refs.setdefault(row.get("run_id", ""), {})["variant_c_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(FORCE_DOMAIN_SELECTED):
        refs.setdefault(row.get("run_id", ""), {})["force_domain_stribeck_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(RATIONAL_RESIDUAL_SELECTED):
        refs.setdefault(row.get("run_id", ""), {})["rational_residual_reference_rmse_nm"] = row.get("candidate_rmse_nm", row.get("corrected_rmse_nm", ""))
    for row in read_rows(STANDALONE_SELECTED):
        refs.setdefault(row.get("run_id", ""), {})["standalone_contact_traction_rmse_nm"] = row.get("standalone_rmse_nm", row.get("corrected_rmse_nm", ""))
    for row in read_rows(TRUE_TRACTION_SELECTED):
        refs.setdefault(row.get("run_id", ""), {})["force_level_contact_testbed_rmse_nm"] = row.get("corrected_rmse_nm", "")
    return refs


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def fmt(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None or value == "":
        return ""
    if isinstance(value, str) and value in ("True", "False"):
        return value.lower()
    try:
        f = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(f):
        return ""
    return f"{f:.6f}"


def markdown_table(rows: list[dict[str, object]], columns: list[str]) -> list[str]:
    out = ["| " + " | ".join(columns) + " |", "| " + " | ".join("---" for _ in columns) + " |"]
    for row in rows:
        out.append("| " + " | ".join(fmt(row.get(col, "")) for col in columns) + " |")
    return out


def make_report(
    selected: FitResult,
    optimizer_rows: list[dict[str, object]],
    split_rows: list[dict[str, object]],
    selected_rows: list[dict[str, object]],
    risk_rows: list[dict[str, object]],
    launch_rows: list[dict[str, object]],
    common_range_rows: list[dict[str, object]],
) -> None:
    params = {name: value for name, value in zip(PARAM_NAMES, selected.params)}
    launch = launch_rows[0]
    lines: list[str] = [
        "# Relaxation-Free Bristle Slip Fit",
        "",
        "Analysis-only output. Production code, build metadata, tests, and existing analysis artifacts were not edited.",
        "",
        "## Model",
        "",
        "For each contact `i` at `(r_i, f_i)` with normal load `N_i`, longitudinal contact slip `vf_i`, lateral contact slip `vr_i`, and body forward speed `Vf`:",
        "",
        "`g0 = 1 / (1 + (abs(Vf) / transition_vf)^2)`",
        "",
        "`mu_stribeck_i = mu_slide + mu_static_extra / (1 + (sqrt(vf_i^2 + vr_i^2) / stribeck_speed)^2)`",
        "",
        "`Fr_stribeck_i = N_i * mu_stribeck_i * sign(vr_i)`",
        "",
        "`alpha_proxy_i = vr_i / (abs(Vf) + alpha_floor)`",
        "",
        "`K_alpha_i = alpha_stiff_base + alpha_stiff_speed_gain * abs(Vf) / (abs(Vf) + alpha_stiff_speed_k)`",
        "",
        "`Fr_alpha_i = N_i * (K_alpha_i * alpha_proxy_i) / (1 + abs(K_alpha_i * alpha_proxy_i) / alpha_saturation_mu)`",
        "",
        "`Fr_i = g0 * Fr_stribeck_i + (1 - g0) * Fr_alpha_i`",
        "",
        "`Ff_i = 0.5 * drive_scale * F_drive_side_i + N_i * longitudinal_mu * vf_i / (abs(vf_i) + longitudinal_k)`",
        "",
        "`M_yaw = sum_i(f_i * Fr_i - r_i * Ff_i)`",
        "",
        "At `Vf=0`, `g0=1`, so the lateral branch is the rational Stribeck/static breakaway law. As `abs(Vf)` rises, the same contact force accumulation transitions into the algebraic slip-angle proxy branch. The law has no state, no command/request selector, no old-force residual branch, and uses no trig, exp, or tanh in the runtime equations.",
        "",
        "## Selected Parameters",
        "",
        "| parameter | value |",
        "| --- | ---: |",
    ]
    for name in PARAM_NAMES:
        lines.append(f"| {name} | {params[name]:.9g} |")
    lines.extend(
        [
            "",
            "## Launch",
            "",
            "| opposing bristle scrub Nm | left command | right command | max abs command | gate | launch lock policy |",
            "| ---: | ---: | ---: | ---: | --- | --- |",
            f"| {fmt(launch['opposing_bristle_scrub_nm'])} | {fmt(launch['left_command'])} | {fmt(launch['right_command'])} | {fmt(launch['max_abs_command'])} | {'pass' if launch['gate_pass'] else 'fail'} | {launch['launch_lock_policy']} |",
            "",
            "## Optimization",
            "",
            "Bounded Nelder-Mead was run from deterministic spaced and random seeds. Launch command is diagnostic only; broad RMSE was optimized with run-balanced pseudo-Huber loss on the current 4D-regime-weighted fit rows.",
            "",
        ]
    )
    lines.extend(
        markdown_table(
            optimizer_rows[:8],
            [
                "label",
                "selected",
                "objective",
                "train_rmse_nm",
                "train_robust_loss",
                "launch_max_abs_command",
                "iterations",
                "converged",
                "boundary_hits",
                "collapse_notes",
            ],
        )
    )
    lines.extend(["", "## Split Metrics", ""])
    lines.extend(
        markdown_table(
            split_rows,
            [
                "group",
                "count",
                "baseline_rmse_nm",
                "bristle_slip_rmse_nm",
                "variant_c_rmse_nm",
                "force_domain_stribeck_rmse_nm",
                "rational_residual_reference_rmse_nm",
                "standalone_contact_traction_rmse_nm",
                "force_level_contact_testbed_rmse_nm",
            ],
        )
    )
    lines.extend(["", "## Selected Logs", ""])
    lines.extend(
        markdown_table(
            selected_rows,
            [
                "run_id",
                "dataset_split",
                "count",
                "baseline_rmse_nm",
                "bristle_slip_rmse_nm",
                "variant_c_rmse_nm",
                "force_domain_stribeck_rmse_nm",
                "rational_residual_reference_rmse_nm",
                "standalone_contact_traction_rmse_nm",
            ],
        )
    )
    lines.extend(["", "## Risk Slices", ""])
    lines.extend(
        markdown_table(
            risk_rows,
            ["group", "count", "baseline_rmse_nm", "bristle_slip_rmse_nm", "rmse_improvement_fraction"],
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
            "## Viability",
            "",
            "This pass is viable as a clean memoryless bristle/slip-angle candidate, but not as the best broad-envelope model. The launch estimate is diagnostic only; broad RMSE should be judged against the standalone contact traction testbed and force-level testbed. If those remain lower on validation, this law is better treated as a simpler interpretable baseline or as a shape for a later stateful LuGre/brush pass.",
            "",
            "Unresolved hysteresis remains out of scope by design. Rows with limiter or hardware saturation evidence are reported as risk slices, not fit authority.",
            "",
            "## Outputs",
            "",
            "- `fit_relaxation_free_bristle_slip.py`",
            "- `optimizer_summary.csv`",
            "- `optimizer_trace.csv`",
            "- `selected_parameters.csv`",
            "- `split_metrics.csv`",
            "- `selected_log_metrics.csv`",
            "- `risk_slices.csv`",
            "- `common_range_metrics.csv`",
            "- `launch_estimate.csv`",
            "- `prediction_sample.csv`",
            "- `relaxation_free_bristle_slip_report.md`",
        ]
    )
    (OUT / "relaxation_free_bristle_slip_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    constants = read_constants()
    frame = load_frame()
    write_regime_diagnostics(
        OUT,
        "",
        compute_regime_weights_for_frame(
            frame,
            RegimeWeightConfig(
                split_weights=LATEST_INCLUSIVE_FIT_SPLIT_WEIGHTS,
                quality=QualityConfig(
                    gyro_spike_multiplier=0.10,
                    saturation_multiplier=0.35,
                    use_limiter_penalty=False,
                    use_low_yaw_no_motion_penalty=False,
                ),
            ),
            eligible_mask=latest_inclusive_fit_mask_for_frame(frame),
        ),
    )
    selected, results, trace_rows = fit(frame, constants)

    params = selected.params
    optimizer_rows = [
        {
            "rank": rank,
            "label": result.label,
            "selected": bool(result.label == selected.label and np.allclose(result.params, selected.params)),
            "objective": result.objective,
            "train_rmse_nm": result.train_rmse,
            "train_robust_loss": result.train_robust,
            "launch_max_abs_command": result.launch_command,
            "launch_opposing_nm": result.launch_opposing_nm,
            "iterations": result.iterations,
            "evaluations_cumulative": result.evaluations,
            "converged": result.converged,
            "boundary_hits": result.boundary_hits,
            "collapse_notes": result.collapse_notes,
        }
        for rank, result in enumerate(results, start=1)
    ]
    parameter_rows = [
        {
            "parameter": name,
            "value": value,
            "lower_bound": BOUNDS[index, 0],
            "upper_bound": BOUNDS[index, 1],
            "near_boundary": bool(
                value <= BOUNDS[index, 0] + 0.02 * (BOUNDS[index, 1] - BOUNDS[index, 0])
                or value >= BOUNDS[index, 1] - 0.02 * (BOUNDS[index, 1] - BOUNDS[index, 0])
            ),
        }
        for index, (name, value) in enumerate(zip(PARAM_NAMES, params))
    ]

    split_refs = reference_by_split()
    split_rows: list[dict[str, object]] = []
    for split in sorted(frame["dataset_split"].dropna().unique()):
        row = metric_row(split, frame[frame["dataset_split"] == split], constants, params)
        row.update(split_refs.get(split, {}))
        split_rows.append(row)
    validation = frame[frame["dataset_split"] != "primary_open_floor_fit_authoritative"]
    row = metric_row("validation_non_authoritative", validation, constants, params)
    row.update(split_refs.get("validation_non_authoritative", {}))
    split_rows.append(row)

    run_refs = reference_by_run()
    selected_rows: list[dict[str, object]] = []
    for run_id in SELECTED_LOGS:
        subset = frame[frame["run_id"] == run_id]
        if subset.empty:
            selected_rows.append({"run_id": run_id, "present": False, "count": 0})
            continue
        row = metric_row(run_id, subset, constants, params)
        row["run_id"] = run_id
        row["dataset_split"] = str(subset["dataset_split"].iloc[0])
        row["family"] = str(subset["family"].iloc[0])
        row.update(run_refs.get(run_id, {}))
        selected_rows.append(row)

    risk_definitions = [
        ("calibration_low_vf_nonzero_yaw", (frame["forward_velocity_mps"].abs() < 0.15) & (frame["yaw_rate_radps"].abs() >= 0.1)),
        ("in_place_scrub", (frame["forward_velocity_mps"].abs() < 0.05) & (frame["yaw_rate_radps"].abs() >= 0.2)),
        ("slow_forward_turn", (frame["forward_velocity_mps"].abs() >= 0.15) & (frame["forward_velocity_mps"].abs() < 0.70) & (frame["yaw_rate_radps"].abs() >= 0.1)),
        ("pre_design_turn_speed", (frame["forward_velocity_mps"].abs() >= 0.70) & (frame["forward_velocity_mps"].abs() < 0.95) & (frame["yaw_rate_radps"].abs() >= 0.1)),
        ("design_turn_speed_and_up", (frame["forward_velocity_mps"].abs() >= 0.95) & (frame["yaw_rate_radps"].abs() >= 0.1)),
        ("fast_forward", frame["forward_velocity_mps"].abs() >= 1.50),
        ("straightish_forward", (frame["yaw_rate_radps"].abs() < 0.05) & (frame["forward_velocity_mps"].abs() >= 0.05)),
        ("limiter_active", frame["limiter_active"].fillna(0.0) > 0.0),
        ("hardware_saturation_evidence", frame["hardware_saturation_evidence"].fillna(0.0) > 0.0),
        ("open_floor_all", frame["family"] == "open_floor"),
        ("diag_all", frame["family"] == "competition_diag"),
        ("aux_all", frame["family"] == "competition_aux"),
        ("may4_latest_logs", frame["run_id"].isin(["2026-05-04_20-35-47", "2026-05-04_16-57-53"])),
    ]
    risk_rows = [metric_row(name, frame[mask], constants, params) for name, mask in risk_definitions]
    launch_rows = [launch_estimate(constants, params)]

    pred_data = matrix(frame, constants)
    pred = predict_from_matrix(pred_data, params)
    common_range_rows = write_common_range_metrics(
        OUT / "common_range_metrics.csv",
        frame,
        pred_data["baseline"] - pred_data["observed"],
        pred - pred_data["observed"],
        "relaxation_free_bristle_slip",
    )
    step = max(1, len(frame) // 1000)
    sample_frame = frame.iloc[::step].copy().head(1000)
    sample_pred = pred[::step][: len(sample_frame)]
    prediction_rows = []
    for row, value in zip(sample_frame.itertuples(index=False), sample_pred):
        prediction_rows.append(
            {
                "run_id": row.run_id,
                "row_index": int(row.row_index),
                "dataset_split": row.dataset_split,
                "physics_phase": row.physics_phase,
                "forward_velocity_mps": row.forward_velocity_mps,
                "yaw_rate_radps": row.yaw_rate_radps,
                "observed_yaw_moment_nm": row.observed_yaw_moment_nm,
                "baseline_yaw_moment_nm": row.model_yaw_moment_nm,
                "bristle_slip_yaw_moment_nm": value,
                "baseline_error_nm": row.model_yaw_moment_nm - row.observed_yaw_moment_nm,
                "bristle_slip_error_nm": value - row.observed_yaw_moment_nm,
            }
        )

    write_csv(OUT / "optimizer_summary.csv", optimizer_rows)
    write_csv(OUT / "optimizer_trace.csv", trace_rows)
    write_csv(OUT / "selected_parameters.csv", parameter_rows)
    write_csv(OUT / "split_metrics.csv", split_rows)
    write_csv(OUT / "selected_log_metrics.csv", selected_rows)
    write_csv(OUT / "risk_slices.csv", risk_rows)
    # `common_range_metrics.csv` is written by the shared utility above.
    write_csv(OUT / "launch_estimate.csv", launch_rows)
    write_csv(OUT / "prediction_sample.csv", prediction_rows)
    (OUT / "commands_run.txt").write_text(
        "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' "
        "codex_analysis\\yaw_model_variant_fits\\stribeck_to_slip_angle_candidates\\relaxation_free_bristle_slip\\fit_relaxation_free_bristle_slip.py\n",
        encoding="utf-8",
    )
    make_report(selected, optimizer_rows, split_rows, selected_rows, risk_rows, launch_rows, common_range_rows)


if __name__ == "__main__":
    main()
